"""
gen_machine.py

This file is the script that "builds the gantt". (Gantt_Schedule.py is the adapter through which the simulation reads this output)

Purpose
-------
Create a fixed, transport-aware FJSP-like production Gantt for wafer lots.
The output is intended to be used as exogenous transport demand for OHT simulation.

Core idea
---------
- Job = wafer lot, follows ProcessStepInfo steps 1..40 (METRO steps may skip).
- Each step requires one MachineType; eligible machines share that type.
- Planned processing time = ProcessingTimeAvg(sec) * pt_scale.
- Greedy ECT rule selects the machine giving earliest completion, considering
  machine availability, lot precedence, and travel time.

Two added features (vs. the original)
--------------------------
1) machine-to-machine travel  (--machine_travel machine_travel.csv)
   Uses measured layout machine-pair distance (free-flow+load+unload) in the ECT
   calculation instead of abstract bay-to-bay distance. Uses the with_handling_sec
   column. Missing pairs fall back to bay distance.
   -> planned_travel matches the simulation's actual physics -> the deviation label becomes clean.

2) CONWIP release            (--wip_cap N)
   If N>0, use CONWIP instead of open-loop staggered release:
   initially release N (uniform init step) -> each time a lot completes its last step
   (fab-out), release the next waiting lot -> keeps WIP inside the fab always at N. No more
   releases if release_time>horizon. The gantt's end_sec = planned_ready (includes machine queuing).

Outputs
-------
planned_gantt.csv (+ prev_machine, release_time columns), transport_tasks.csv,
skipped_steps.csv, summary.json, validation_report.txt

Finalized command (10 bay / CONWIP WIP 300 / 24h / measured machine travel)
----------------------------------------------------------------
python gen_machine.py \
  --data_dir dataset_10bay \
  --out_dir ./gantt_final \
  --layout file \
  --num_lots 1500 \
  --wip_cap 300 \
  --horizon_hours 24 \
  --pt_scale 0.75 \
  --init_step_mode uniform \
  --use_skip_probability \
  --machine_travel machine_travel.csv \
  --seed 42

(--layout ten_bay can also generate the same layout — the built-in 10bay matches dataset_10bay)
"""

from __future__ import annotations

import argparse
import heapq
import json
import os
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd


@dataclass
class LotState:
    lot_id: str
    next_step: int
    prev_end: float
    prev_bay: Optional[str]
    release_seq: int


REQUIRED_FILES = {
    "machine": "MachineInfo.txt",
    "steps": "ProcessStepInfo.txt",
    "travel": "TravelTimeInfo.txt",
}


TEN_BAY_COUNTS = {
    "Bay1":  {"ETCH": 6, "CMP": 4},
    "Bay2":  {"CVD": 6, "IMP": 4},
    "Bay3":  {"PHOTO": 4, "IMP": 3, "METRO": 3},
    "Bay4":  {"CVD": 5, "CMP": 3, "METRO": 2},
    "Bay5":  {"PHOTO": 4, "ETCH": 6},
    "Bay6":  {"PHOTO": 4, "ETCH": 6},
    "Bay7":  {"CVD": 5, "CMP": 3, "IMP": 2},
    "Bay8":  {"PHOTO": 4, "METRO": 4, "IMP": 2},
    "Bay9":  {"CVD": 2, "CMP": 3, "IMP": 5},
    "Bay10": {"ETCH": 3, "METRO": 5, "IMP": 2},
}


def clean_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [c.strip() for c in df.columns]
    return df


def read_tsv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")
    return clean_columns(pd.read_csv(path, sep="\t"))


def build_ten_bay_machine_info() -> pd.DataFrame:
    rows = []
    group_counter = {"CMP": 0, "PHOTO": 0, "ETCH": 0, "METRO": 0, "CVD": 0, "IMP": 0}
    machine_id = 1
    for bay in [f"Bay{i}" for i in range(1, 11)]:
        for mtype, count in TEN_BAY_COUNTS[bay].items():
            group_counter[mtype] += 1
            group = f"{mtype}{group_counter[mtype]}"
            for _ in range(count):
                rows.append({
                    "Machine": f"M{machine_id}",
                    "MachineType": mtype,
                    "MachineGroup": group,
                    "Location": bay,
                })
                machine_id += 1
    df = pd.DataFrame(rows)
    assert len(df) == 100, len(df)
    return df


def build_loop_travel_info(num_bays: int = 10, same_bay_sec: float = 65.0, adjacent_sec: float = 110.0) -> pd.DataFrame:
    """Generate simple symmetric travel times on a closed loop.

    The loop order follows Bay1-Bay2-...-Bay5-Bay6-Bay7-...-Bay10-Bay1.
    This is intentionally simple. Replace with measured OHT travel times when available.
    """
    bays = [f"Bay{i}" for i in range(1, num_bays + 1)]
    rows = []
    for i, fb in enumerate(bays):
        for j, tb in enumerate(bays):
            if i == j:
                avg = same_bay_sec
            else:
                d = abs(i - j)
                ring_dist = min(d, num_bays - d)
                avg = adjacent_sec * ring_dist
            rows.append({
                "FromBay": fb,
                "ToBay": tb,
                "TravelTimeAvg(sec)": avg,
                "TraveTimeSD": max(10.0, avg * 0.15),
                "TraveTimeMin(sec)": max(20.0, avg * 0.55),
            })
    return pd.DataFrame(rows)


def normalize_step_info(df: pd.DataFrame) -> pd.DataFrame:
    df = clean_columns(df)
    # tolerate the uploaded file's spacing typo around the average column
    rename = {}
    for c in df.columns:
        cc = c.strip()
        if cc.startswith("ProcessingTimeAvg"):
            rename[c] = "ProcessingTimeAvg(sec)"
        elif cc.startswith("ProcessingTimeSD"):
            rename[c] = "ProcessingTimeSD"
        elif cc.startswith("ProcessingTimeMinimum"):
            rename[c] = "ProcessingTimeMinimum(sec)"
    df = df.rename(columns=rename)
    numeric_cols = ["No", "ProcessingTimeAvg(sec)", "ProcessingTimeSD", "ProcessingTimeMinimum(sec)", "SkipProbabilityOfMetrology"]
    for c in numeric_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.sort_values("No").reset_index(drop=True)
    return df


def travel_lookup(travel_df: pd.DataFrame) -> Dict[Tuple[str, str], float]:
    travel_df = clean_columns(travel_df)
    ren = {}
    for c in travel_df.columns:
        if c.strip().startswith("TravelTimeAvg"):
            ren[c] = "TravelTimeAvg(sec)"
    travel_df = travel_df.rename(columns=ren)
    travel_df["TravelTimeAvg(sec)"] = pd.to_numeric(travel_df["TravelTimeAvg(sec)"], errors="coerce")
    return {(r.FromBay, r.ToBay): float(r["TravelTimeAvg(sec)"]) for _, r in travel_df.iterrows()}


def get_travel_sec(prev_bay: Optional[str], next_bay: str, lookup: Dict[Tuple[str, str], float]) -> float:
    if prev_bay is None or prev_bay == "ENTRY":
        return 0.0
    if (prev_bay, next_bay) in lookup:
        return lookup[(prev_bay, next_bay)]
    if (next_bay, prev_bay) in lookup:
        return lookup[(next_bay, prev_bay)]
    # fallback for missing travel entries
    return 300.0


def initial_step_for_lot(idx: int, num_lots: int, max_step: int, mode: str, rng: random.Random) -> int:
    if mode == "all_step1":
        return 1
    if mode == "uniform":
        # deterministic-ish balanced distribution across steps
        return (idx % max_step) + 1
    if mode == "random_uniform":
        return rng.randint(1, max_step)
    raise ValueError(f"Unknown init_step_mode: {mode}")


def schedule_gantt(
    machine_df: pd.DataFrame,
    step_df: pd.DataFrame,
    travel_sec: Dict[Tuple[str, str], float],
    num_lots: int = 500,
    pt_scale: float = 0.75,
    init_step_mode: str = "uniform",
    release_interval_sec: float = 0.0,
    use_skip_probability: bool = False,
    seed: int = 42,
    machine_travel: Optional[Dict[Tuple[str, str], float]] = None,
    balance_tol: float = 0.0,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    rng = random.Random(seed)
    np_rng = np.random.default_rng(seed)

    machine_df = machine_df.copy()
    step_df = normalize_step_info(step_df)
    max_step = int(step_df["No"].max())
    step_by_no = {int(r.No): r for _, r in step_df.iterrows()}

    machines_by_type: Dict[str, List[dict]] = {}
    for _, r in machine_df.iterrows():
        rec = {
            "Machine": str(r.Machine),
            "MachineType": str(r.MachineType),
            "MachineGroup": str(r.MachineGroup),
            "Location": str(r.Location),
        }
        machines_by_type.setdefault(rec["MachineType"], []).append(rec)

    # availability of each physical machine
    machine_available = {m: 0.0 for m in machine_df["Machine"].astype(str)}
    # load-balancing: number of ops assigned so far per machine (to use same-type machines evenly)
    machine_ops = {m: 0 for m in machine_df["Machine"].astype(str)}

    # priority queue of lots ready to schedule their next operation
    # heap item: (lot_ready_time, release_seq, lot_id, next_step, prev_end, prev_bay)
    pq = []
    for i in range(num_lots):
        lot_id = f"L{i + 1:04d}"
        init_step = initial_step_for_lot(i, num_lots, max_step, init_step_mode, rng)
        release_t = i * release_interval_sec
        heapq.heappush(pq, (release_t, i, lot_id, init_step, release_t, None, None))

    gantt_rows = []
    skipped_rows = []
    op_uid = 0

    while pq:
        _, release_seq, lot_id, step_no, prev_end, prev_bay, prev_machine = heapq.heappop(pq)
        if step_no > max_step:
            continue

        step = step_by_no[step_no]
        mtype = str(step.MachineType).strip()

        if use_skip_probability and mtype == "METRO":
            p_skip = float(step.SkipProbabilityOfMetrology)
            if np_rng.random() < p_skip:
                skipped_rows.append({
                    "lot_id": lot_id,
                    "step_no": step_no,
                    "machine_type": mtype,
                    "skip_time": prev_end,
                    "prev_bay": prev_bay,
                })
                heapq.heappush(pq, (prev_end, release_seq, lot_id, step_no + 1, prev_end, prev_bay, prev_machine))
                continue

        candidates = machines_by_type.get(mtype, [])
        if not candidates:
            raise ValueError(f"No candidate machine found for MachineType={mtype} at step={step_no}")

        proc = float(step["ProcessingTimeAvg(sec)"]) * pt_scale

        evals = []
        for cand in candidates:
            m = cand["Machine"]
            bay = cand["Location"]
            if machine_travel is not None and prev_machine is not None:
                tr = machine_travel.get((prev_machine, m))
                if tr is None:
                    tr = get_travel_sec(prev_bay, bay, travel_sec)  # fallback
            else:
                tr = get_travel_sec(prev_bay, bay, travel_sec)
            ready_at_machine = prev_end + tr
            start = max(ready_at_machine, machine_available[m])
            end = start + proc
            evals.append((end, tr, start, m, cand, ready_at_machine))

        def _mnum(mm):
            return int(mm[1:]) if mm.startswith("M") and mm[1:].isdigit() else 10**9

        best_end = min(e[0] for e in evals)
        # load-balancing: among candidates whose completion time is within balance_tol of best,
        # prefer the machine with fewer assignments so far (use same-type machines evenly).
        # balance_tol=0 means pure ECT (earliest end). Since it spreads only within tol,
        # makespan stays almost unchanged.
        pool = [e for e in evals if e[0] <= best_end + balance_tol]
        pool.sort(key=lambda e: (machine_ops[e[3]], e[0], e[1], e[2], _mnum(e[3])))
        end, tr, start, _m, cand, ready_at_machine = pool[0]
        machine_available[cand["Machine"]] = end
        machine_ops[cand["Machine"]] += 1
        op_uid += 1
        gantt_rows.append({
            "op_uid": op_uid,
            "lot_id": lot_id,
            "step_no": step_no,
            "machine_type": mtype,
            "machine": cand["Machine"],
            "machine_group": cand["MachineGroup"],
            "bay": cand["Location"],
            "prev_bay": prev_bay if prev_bay is not None else "ENTRY",
            "prev_machine": prev_machine if prev_machine is not None else "ENTRY",
            "planned_travel_sec": tr,
            "ready_at_machine_sec": ready_at_machine,
            "start_sec": start,
            "end_sec": end,
            "planned_proc_sec": proc,
            "machine_wait_sec": max(0.0, machine_available[cand["Machine"]] - end),  # always 0, kept for schema compatibility
        })

        heapq.heappush(pq, (end, release_seq, lot_id, step_no + 1, end, cand["Location"], cand["Machine"]))

    gantt = pd.DataFrame(gantt_rows).sort_values(["start_sec", "op_uid"]).reset_index(drop=True)
    skipped = pd.DataFrame(skipped_rows)
    return gantt, skipped


def schedule_gantt_conwip(
    machine_df: pd.DataFrame,
    step_df: pd.DataFrame,
    travel_sec: Dict[Tuple[str, str], float],
    pool_size: int = 1500,
    wip_cap: int = 500,
    horizon_sec: float = 86400.0,
    pt_scale: float = 0.75,
    init_step_mode: str = "uniform",
    use_skip_probability: bool = False,
    seed: int = 42,
    machine_travel: Optional[Dict[Tuple[str, str], float]] = None,
    balance_tol: float = 0.0,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    CONWIP scheduler (planned world).
      - Keeps the lot count inside the fab (WIP) always at wip_cap.
      - Initial: release wip_cap lots at t=0 (spread across the pipeline via uniform init step).
      - Then: each time a lot completes its last step (= fab-out), release the next waiting lot at that time.
      - Completion/release times include machine queuing (decision A): planned end serialized via machine_available.
      - No more releases if release_time > horizon (24h boundary).
    This gantt's end_sec = planned_ready (includes machine queuing). deviation = realized - planned
    then captures only the extra delay created by OHT/transport in the simulation (= positioning target).
    """
    machine_df = machine_df.copy()
    step_df = normalize_step_info(step_df)
    max_step = int(step_df["No"].max())
    step_by_no = {int(r.No): r for _, r in step_df.iterrows()}

    machines_by_type: Dict[str, List[dict]] = {}
    for _, r in machine_df.iterrows():
        rec = {"Machine": str(r.Machine), "MachineType": str(r.MachineType),
               "MachineGroup": str(r.MachineGroup), "Location": str(r.Location)}
        machines_by_type.setdefault(rec["MachineType"], []).append(rec)

    rng = random.Random(seed)
    np_rng = np.random.default_rng(seed)
    machine_available = {m: 0.0 for m in machine_df["Machine"].astype(str)}
    machine_ops = {m: 0 for m in machine_df["Machine"].astype(str)}  # load-balancing

    gantt_rows, skipped_rows = [], []
    lot_release: Dict[str, float] = {}
    op_uid = 0
    seq = 0
    wip = 0
    pool_ptr = 0

    pq: list = []

    def release_lot(lot_id: str, init_step: int, t: float):
        nonlocal seq
        lot_release[lot_id] = t
        heapq.heappush(pq, (t, seq, lot_id, init_step, t, None, None))
        seq += 1

    # initial fill: release wip_cap lots at t=0 with uniform init step
    while wip < wip_cap and pool_ptr < pool_size:
        lot_id = f"L{pool_ptr + 1:05d}"
        init_step = initial_step_for_lot(pool_ptr, pool_size, max_step, init_step_mode, rng)
        release_lot(lot_id, init_step, 0.0)
        wip += 1
        pool_ptr += 1

    def on_completion(t_complete: float):
        """One lot fab-out -> WIP down by 1 -> release one waiting lot (if possible)."""
        nonlocal wip, pool_ptr
        wip -= 1
        if wip < wip_cap and pool_ptr < pool_size and t_complete <= horizon_sec:
            lot_id = f"L{pool_ptr + 1:05d}"
            release_lot(lot_id, 1, t_complete)   # new release starts from step 1 (fresh lot)
            wip += 1
            pool_ptr += 1

    while pq:
        t_ready, _, lot_id, step_no, prev_end, prev_bay, prev_machine = heapq.heappop(pq)
        if step_no > max_step:
            continue
        step = step_by_no[step_no]
        mtype = str(step.MachineType).strip()

        # METRO skip
        if use_skip_probability and mtype == "METRO":
            if np_rng.random() < float(step.SkipProbabilityOfMetrology):
                skipped_rows.append({"lot_id": lot_id, "step_no": step_no,
                                     "machine_type": mtype, "skip_time": prev_end,
                                     "prev_bay": prev_bay})
                nxt = step_no + 1
                if nxt > max_step:
                    on_completion(prev_end)
                else:
                    heapq.heappush(pq, (prev_end, seq, lot_id, nxt, prev_end, prev_bay, prev_machine))
                    seq += 1
                continue

        candidates = machines_by_type.get(mtype, [])
        if not candidates:
            raise ValueError(f"No candidate machine for MachineType={mtype} step={step_no}")
        proc = float(step["ProcessingTimeAvg(sec)"]) * pt_scale

        evals = []
        for cand in candidates:
            m = cand["Machine"]
            if machine_travel is not None and prev_machine is not None:
                tr = machine_travel.get((prev_machine, m))
                if tr is None:
                    tr = get_travel_sec(prev_bay, cand["Location"], travel_sec)
            else:
                tr = get_travel_sec(prev_bay, cand["Location"], travel_sec)
            ready_at_machine = prev_end + tr
            start = max(ready_at_machine, machine_available[m])
            end = start + proc
            evals.append((end, tr, start, m, cand, ready_at_machine))

        def _mnum(mm):
            return int(mm[1:]) if mm.startswith("M") and mm[1:].isdigit() else 10**9

        best_end = min(e[0] for e in evals)
        # load-balancing: among candidates within balance_tol of the best completion time, prefer the machine with fewer assignments.
        # tol=0 means pure ECT. Spreads only within tol -> keeps makespan almost unchanged while using same-type machines evenly.
        pool = [e for e in evals if e[0] <= best_end + balance_tol]
        pool.sort(key=lambda e: (machine_ops[e[3]], e[0], e[1], e[2], _mnum(e[3])))
        end, tr, start, _m, cand, ready_at_machine = pool[0]
        machine_available[cand["Machine"]] = end
        machine_ops[cand["Machine"]] += 1
        op_uid += 1
        gantt_rows.append({
            "op_uid": op_uid, "lot_id": lot_id, "step_no": step_no,
            "machine_type": mtype, "machine": cand["Machine"],
            "machine_group": cand["MachineGroup"], "bay": cand["Location"],
            "prev_bay": prev_bay if prev_bay is not None else "ENTRY",
            "prev_machine": prev_machine if prev_machine is not None else "ENTRY",
            "planned_travel_sec": tr, "ready_at_machine_sec": ready_at_machine,
            "start_sec": start, "end_sec": end, "planned_proc_sec": proc,
            "machine_wait_sec": max(0.0, start - ready_at_machine),
            "release_time": lot_release.get(lot_id, 0.0),
        })

        nxt = step_no + 1
        if nxt > max_step:
            on_completion(end)         # last step completed = fab-out
        else:
            heapq.heappush(pq, (end, seq, lot_id, nxt, end, cand["Location"], cand["Machine"]))
            seq += 1

    gantt = pd.DataFrame(gantt_rows).sort_values(["start_sec", "op_uid"]).reset_index(drop=True)
    skipped = pd.DataFrame(skipped_rows)
    return gantt, skipped


def derive_transport_tasks(gantt: pd.DataFrame, travel_sec: Dict[Tuple[str, str], float]) -> pd.DataFrame:
    rows = []
    task_id = 0
    for lot_id, g in gantt.sort_values(["lot_id", "step_no", "start_sec"]).groupby("lot_id"):
        ops = g.sort_values("step_no").to_dict("records")
        for prev, cur in zip(ops[:-1], ops[1:]):
            # If consecutive scheduled ops are not adjacent in step_no due to skipped metrology,
            # this still creates the actual required movement from previous scheduled op to next scheduled op.
            task_id += 1
            source_bay = prev["bay"]
            dest_bay = cur["bay"]
            avg_travel = get_travel_sec(source_bay, dest_bay, travel_sec)
            rows.append({
                "task_id": f"T{task_id:07d}",
                "lot_id": lot_id,
                "from_step_no": int(prev["step_no"]),
                "to_step_no": int(cur["step_no"]),
                "source_machine": prev["machine"],
                "source_group": prev["machine_group"],
                "source_bay": source_bay,
                "dest_machine": cur["machine"],
                "dest_group": cur["machine_group"],
                "dest_bay": dest_bay,
                "planned_ready_sec": float(prev["end_sec"]),
                "planned_due_sec": float(cur["start_sec"]),
                "planned_avg_travel_sec": avg_travel,
                "planned_slack_sec": float(cur["start_sec"]) - float(prev["end_sec"]) - avg_travel,
            })
    return pd.DataFrame(rows)


def validate_gantt(gantt: pd.DataFrame, machine_df: pd.DataFrame) -> Dict[str, object]:
    report: Dict[str, object] = {}

    machine_type_map = dict(zip(machine_df["Machine"].astype(str), machine_df["MachineType"].astype(str)))
    wrong_type = 0
    for _, r in gantt.iterrows():
        if machine_type_map.get(str(r.machine)) != str(r.machine_type):
            wrong_type += 1
    report["wrong_machine_type_assignments"] = wrong_type

    precedence_violations = 0
    for _, g in gantt.sort_values(["lot_id", "step_no"]).groupby("lot_id"):
        prev_end = -math.inf
        prev_step = None
        for _, r in g.iterrows():
            if prev_step is not None and int(r.step_no) <= prev_step:
                precedence_violations += 1
            if float(r.start_sec) + 1e-9 < prev_end:
                precedence_violations += 1
            prev_end = float(r.end_sec)
            prev_step = int(r.step_no)
    report["precedence_violations"] = precedence_violations

    overlap_violations = 0
    for machine, g in gantt.sort_values(["machine", "start_sec"]).groupby("machine"):
        last_end = -math.inf
        for _, r in g.iterrows():
            if float(r.start_sec) + 1e-9 < last_end:
                overlap_violations += 1
            last_end = max(last_end, float(r.end_sec))
    report["machine_overlap_violations"] = overlap_violations

    return report


def summarize(gantt: pd.DataFrame, transport: pd.DataFrame, horizon_sec: float, num_lots: int) -> Dict[str, object]:
    lot_completion = gantt.groupby("lot_id")["end_sec"].max()
    completed_by_horizon = int((lot_completion <= horizon_sec).sum())
    makespan = float(lot_completion.max()) if not lot_completion.empty else 0.0

    machine_util = (
        gantt.groupby("machine")["planned_proc_sec"].sum() / max(horizon_sec, 1.0)
    ).clip(upper=1.0)

    by_type = gantt.groupby("machine_type").agg(
        num_ops=("op_uid", "count"),
        total_proc_sec=("planned_proc_sec", "sum"),
        avg_proc_sec=("planned_proc_sec", "mean"),
    ).reset_index().to_dict("records")

    return {
        "num_lots_input": num_lots,
        "num_operations_scheduled": int(len(gantt)),
        "num_transport_tasks": int(len(transport)),
        "horizon_sec": horizon_sec,
        "horizon_hours": horizon_sec / 3600.0,
        "completed_lots_by_horizon": completed_by_horizon,
        "completed_lots_rate_per_hour": completed_by_horizon / max(horizon_sec / 3600.0, 1e-9),
        "planned_makespan_sec": makespan,
        "planned_makespan_hours": makespan / 3600.0,
        "avg_machine_utilization_within_horizon": float(machine_util.mean()) if not machine_util.empty else 0.0,
        "p90_machine_utilization_within_horizon": float(machine_util.quantile(0.9)) if not machine_util.empty else 0.0,
        "machine_type_loads": by_type,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str, default=".", help="Directory containing MachineInfo.txt, ProcessStepInfo.txt, TravelTimeInfo.txt")
    parser.add_argument("--out_dir", type=str, default="./gantt_final")
    parser.add_argument("--layout", choices=["file", "ten_bay"], default="file", help="Use uploaded MachineInfo/TravelTimeInfo or generated 10-bay layout")
    parser.add_argument("--num_lots", type=int, default=500)
    parser.add_argument("--horizon_hours", type=float, default=24.0)
    parser.add_argument("--pt_scale", type=float, default=0.75)
    parser.add_argument("--init_step_mode", choices=["all_step1", "uniform", "random_uniform"], default="uniform")
    parser.add_argument("--release_interval_sec", type=float, default=0.0, help="Optional staggered release interval for lots")
    parser.add_argument("--use_skip_probability", action="store_true", help="Randomly skip METRO steps according to SkipProbabilityOfMetrology")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--wip_cap", type=int, default=0,
                        help=">0 enables CONWIP mode: keep the lot count inside the fab at this value (release on completion). 0=open-loop")
    parser.add_argument("--machine_travel", type=str, default="machine_travel.csv",
                        help="machine-to-machine travel CSV (gen_machine_travel.py output). "
                             "If present, precise M->M transport; otherwise bay-distance fallback")
    parser.add_argument("--balance_tol", type=float, default=60.0,
                        help="load-balancing allowed completion-time tolerance (sec). Among candidates "
                             "within this value of best end, pick the less-used machine to use same-type machines evenly. "
                             "0=pure ECT (allows skew)")
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.layout == "file":
        machine_df = read_tsv(data_dir / REQUIRED_FILES["machine"])
        travel_df = read_tsv(data_dir / REQUIRED_FILES["travel"])
    else:
        machine_df = build_ten_bay_machine_info()
        travel_df = build_loop_travel_info(10)
        machine_df.to_csv(out_dir / "generated_10bay_MachineInfo.csv", index=False)
        travel_df.to_csv(out_dir / "generated_10bay_TravelTimeInfo.csv", index=False)

    step_df = normalize_step_info(read_tsv(data_dir / REQUIRED_FILES["steps"]))
    travel = travel_lookup(travel_df)
    mtravel = None
    if args.machine_travel and os.path.exists(args.machine_travel):
        import csv as _csv
        mtravel = {}
        with open(args.machine_travel, newline='', encoding='utf-8') as _f:
            for _r in _csv.DictReader(_f):
                mtravel[(_r['from_machine'], _r['to_machine'])] = float(_r['with_handling_sec'])
        print(f'[machine_travel] {len(mtravel)} pairs loaded (free-flow + load + unload)')
    elif args.machine_travel:
        print(f'[machine_travel] {args.machine_travel} not found -> using bay-distance fallback')

    if args.wip_cap and args.wip_cap > 0:
        print(f"[CONWIP] WIP cap={args.wip_cap}, route pool={args.num_lots}, horizon={args.horizon_hours}h")
        gantt, skipped = schedule_gantt_conwip(
            machine_df=machine_df,
            step_df=step_df,
            travel_sec=travel,
            pool_size=args.num_lots,
            wip_cap=args.wip_cap,
            horizon_sec=args.horizon_hours * 3600.0,
            pt_scale=args.pt_scale,
            init_step_mode=args.init_step_mode,
            use_skip_probability=args.use_skip_probability,
            seed=args.seed,
            machine_travel=mtravel,
            balance_tol=args.balance_tol,
        )
    else:
        gantt, skipped = schedule_gantt(
            machine_df=machine_df,
            step_df=step_df,
            travel_sec=travel,
            num_lots=args.num_lots,
            pt_scale=args.pt_scale,
            init_step_mode=args.init_step_mode,
            release_interval_sec=args.release_interval_sec,
            use_skip_probability=args.use_skip_probability,
            seed=args.seed,
            machine_travel=mtravel,
            balance_tol=args.balance_tol,
        )
    transport = derive_transport_tasks(gantt, travel)
    validation = validate_gantt(gantt, machine_df)
    summary = summarize(gantt, transport, args.horizon_hours * 3600.0, args.num_lots)
    summary.update({
        "pt_scale": args.pt_scale,
        "init_step_mode": args.init_step_mode,
        "layout": args.layout,
        "use_skip_probability": args.use_skip_probability,
        "validation": validation,
    })

    gantt.to_csv(out_dir / "planned_gantt.csv", index=False)
    transport.to_csv(out_dir / "transport_tasks.csv", index=False)
    skipped.to_csv(out_dir / "skipped_steps.csv", index=False)
    with open(out_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    with open(out_dir / "validation_report.txt", "w", encoding="utf-8") as f:
        f.write("Validation report\n")
        f.write("=================\n")
        for k, v in validation.items():
            f.write(f"{k}: {v}\n")
        f.write("\nSummary\n")
        f.write("=======\n")
        f.write(json.dumps(summary, indent=2, ensure_ascii=False))
        f.write("\n")

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\nSaved outputs to: {out_dir.resolve()}")


if __name__ == "__main__":
    main()
