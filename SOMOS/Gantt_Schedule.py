"""
Gantt_Schedule.py

Adapter that reads the planned_gantt.csv produced by generate_fjsp_gantt_500.py
and converts it into the routing + planned-timing interface consumed by the simulation.

Replaces the legacy JSSPData (ta01-based). The identity scheme differs:
  JSSPData : job_type_id(0..14)  <-> machine_id(int)
  GanttSchedule : lot_id(L0001..) <-> machine(M1..M100), step_no(1..40)

Key points:
  - Targeting is fixed by the gantt (each step's physical machine is already decided).
  - planned_ready(lot, step) = that step's planned end_sec
      -> the time transport "can plan to release" = the planned side of the training label.
  - realized proc is sampled at runtime (if a step distribution is given) -> deviation from planned.

planned_gantt.csv columns:
  op_uid, lot_id, step_no, machine_type, machine, machine_group, bay,
  prev_bay, planned_travel_sec, ready_at_machine_sec, start_sec, end_sec,
  planned_proc_sec, machine_wait_sec
"""
from __future__ import annotations

import csv
import os
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class PlannedOp:
    lot_id:       str
    step_no:      int
    machine:      str      # physical machine name (e.g. "M37")
    machine_type: str      # "PHOTO", etc.
    machine_group:str
    bay:          str
    planned_proc: float    # planned processing time (avg x pt_scale)
    planned_start:float    # gantt planned start
    planned_end:  float    # gantt planned end = this op's planned_ready
    planned_travel: float = 0.0  # free-flow travel time into this op (gantt plan)


@dataclass
class LotRoute:
    lot_id: str
    ops:    List[PlannedOp] = field(default_factory=list)   # ascending step_no

    def first_step(self) -> int:
        return self.ops[0].step_no

    def op_at(self, step_no: int) -> Optional[PlannedOp]:
        for op in self.ops:
            if op.step_no == step_no:
                return op
        return None

    def next_op_after(self, step_no: int) -> Optional[PlannedOp]:
        """The op following the current step_no (adjacent in the schedule). Steps may be skipped."""
        found = False
        for op in self.ops:
            if found:
                return op
            if op.step_no == step_no:
                found = True
        return None


# ──────────────────────────────────────────────────────────────────────
# Optional: per-step distribution for realized-proc sampling (if ProcessStepInfo.txt exists)
# ──────────────────────────────────────────────────────────────────────

@dataclass
class StepDist:
    avg: float
    sd:  float
    mn:  float    # minimum (truncation)


class GanttSchedule:
    """
    Loads planned_gantt.csv as per-lot routing.

    Parameters
    ----------
    gantt_csv : path to planned_gantt.csv
    step_info_csv : (optional) ProcessStepInfo.txt (TSV). If given, realized proc is
                    sampled from Normal(avg, sd) (truncated at min). Otherwise planned_proc is used.
    pt_scale : scale multiplied into step_info times (must match gantt generation).
    """

    def __init__(self,
                 gantt_csv: str,
                 step_info_csv: Optional[str] = None,
                 pt_scale: float = 0.75):
        self.gantt_csv = gantt_csv
        self.pt_scale  = pt_scale
        self.routes: Dict[str, LotRoute] = {}
        self.step_dist: Dict[int, StepDist] = {}
        self._machines: set = set()
        self._load_gantt()
        if step_info_csv:
            self._load_step_info(step_info_csv)

    # ------------------------------------------------------------------
    def _load_gantt(self):
        if not os.path.exists(self.gantt_csv):
            raise FileNotFoundError(f"planned_gantt.csv not found: {self.gantt_csv}")
        with open(self.gantt_csv, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                op = PlannedOp(
                    lot_id        = row["lot_id"],
                    step_no       = int(row["step_no"]),
                    machine       = row["machine"],
                    machine_type  = row["machine_type"],
                    machine_group = row.get("machine_group", ""),
                    bay           = row["bay"],
                    planned_proc  = float(row["planned_proc_sec"]),
                    planned_start = float(row["start_sec"]),
                    planned_end   = float(row["end_sec"]),
                    planned_travel= float(row.get("planned_travel_sec", 0.0) or 0.0),
                )
                self.routes.setdefault(op.lot_id, LotRoute(op.lot_id)).ops.append(op)
                self._machines.add(op.machine)
        for r in self.routes.values():
            r.ops.sort(key=lambda o: o.step_no)

    def _load_step_info(self, path: str):
        if not os.path.exists(path):
            print(f"[GanttSchedule] step_info not found -> using realized=planned: {path}")
            return
        with open(path, newline="", encoding="utf-8") as f:
            # TSV; absorb whitespace/naming variation in column headers
            reader = csv.DictReader(f, delimiter="\t")
            for row in reader:
                row = {k.strip(): v for k, v in row.items()}
                no = None
                for k in row:
                    if k == "No":
                        no = int(float(row[k]))
                        break
                if no is None:
                    continue
                def pick(prefix):
                    for k in row:
                        if k.replace(" ", "").startswith(prefix):
                            try:
                                return float(row[k])
                            except (ValueError, TypeError):
                                return None
                    return None
                avg = pick("ProcessingTimeAvg")
                sd  = pick("ProcessingTimeSD") or 0.0
                mn  = pick("ProcessingTimeMinimum") or 0.0
                if avg is not None:
                    self.step_dist[no] = StepDist(avg, sd, mn)

    # ------------------------------------------------------------------
    # routing / timing API  (consumed by MachineStation)
    # ------------------------------------------------------------------
    def lot_ids(self) -> List[str]:
        return list(self.routes.keys())

    def machines(self) -> List[str]:
        return sorted(self._machines)

    def first_op(self, lot_id: str) -> Optional[PlannedOp]:
        r = self.routes.get(lot_id)
        return r.ops[0] if r and r.ops else None

    def op(self, lot_id: str, step_no: int) -> Optional[PlannedOp]:
        r = self.routes.get(lot_id)
        return r.op_at(step_no) if r else None

    def next_machine_after(self, lot_id: str, step_no: int) -> Optional[str]:
        """Physical machine assigned to the op after the current step completes. None = lot finished."""
        r = self.routes.get(lot_id)
        if r is None:
            return None
        nxt = r.next_op_after(step_no)
        return nxt.machine if nxt else None

    def next_step_no(self, lot_id: str, step_no: int) -> Optional[int]:
        r = self.routes.get(lot_id)
        if r is None:
            return None
        nxt = r.next_op_after(step_no)
        return nxt.step_no if nxt else None

    def planned_proc(self, lot_id: str, step_no: int) -> Optional[float]:
        op = self.op(lot_id, step_no)
        return op.planned_proc if op else None

    def planned_ready(self, lot_id: str, step_no: int) -> Optional[float]:
        """This step's planned end time = the time transport can plan to release (planned label)."""
        op = self.op(lot_id, step_no)
        return op.planned_end if op else None

    def planned_due(self, lot_id: str, step_no: int) -> Optional[float]:
        """Next step's planned start time = reference arrival time at the next machine."""
        r = self.routes.get(lot_id)
        if r is None:
            return None
        nxt = r.next_op_after(step_no)
        return nxt.planned_start if nxt else None

    def planned_travel(self, lot_id: str, step_no: int) -> Optional[float]:
        """Gantt planned travel time (free-flow) into this step. Planned side of the transport-delay label."""
        op = self.op(lot_id, step_no)
        return op.planned_travel if op else None

    def realized_proc(self, lot_id: str, step_no: int,
                      rng: Optional[random.Random] = None) -> Optional[float]:
        """
        Runtime processing time. If a step distribution exists, sample Normal(avg,sd)
        (truncated at min); otherwise use planned_proc as-is (= no deviation).
        """
        op = self.op(lot_id, step_no)
        if op is None:
            return None
        d = self.step_dist.get(step_no)
        if d is None or d.sd <= 0:
            return op.planned_proc
        rng = rng or random
        val = rng.gauss(d.avg * self.pt_scale, d.sd * self.pt_scale)
        return max(d.mn * self.pt_scale, val)
