from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


@dataclass
class MachineConfig:
    """
    Machine station physical parameters.

    NOTE: processing time is determined by the gantt (planned_gantt.csv) planned_proc and
      the ProcessStepInfo.txt distribution samples (realized proc), so there is
      no process_time_min / max. (the ta01/JSSPData path is deprecated)
    """
    input_buffer_cap:     int   = 3
    output_buffer_cap:    int   = 3
    auto_start_when_idle: bool  = True


@dataclass
class JSSPConfig:
    """
    JSSP problem configuration and CSV output paths.

    base_throughput:
        Reference production quantity for the job type with the shortest total processing time.
        Other job types are auto-computed by their processing-time ratio.

        e.g. base=2, Job12(shortest)=4956s, Job8(longest)=8004s
            -> Job12: 2 units, Job8: round(2 x 8004/4956) = 3 units
    """
    scale_factor:     float = 1.0
    scale_offset:     float = 0.0
    window_sec:       int = 86400          # gantt axis/display range = 24h

    base_throughput:  int = 1

    # ── gantt / CONWIP linkage ───────────────────────────────────────────
    planned_gantt_csv: str = "gantt_final/planned_gantt.csv"   # produced by gen_machine.py
    step_info_path:    str = "ProcessStepInfo.txt"  # for realized-proc sampling (Normal avg/sd/min)
    pt_scale:          float = 0.75       # same as gantt generation
    wip_cap:           int = 250          # CONWIP: max lots in the fab
    inject_cutoff_sec: float = 0.0        # >0 stops new injection after that time
    pool_limit:        int = 0            # >0 fixes pool to the first N (for fixed-workload makespan)
    machine_strict_plan_order: bool = False  # default: schedule fixed, process arrivals (late lots shift), no deadlock
                                             # True=force planned order (wait if late) -> deadlock under finite buffers, unused

    # number of throughput units produced when one job is processed to completion (= last op done)
    # e.g. units_per_job=10000 -> 5 completed jobs -> 50,000 throughput units
    units_per_job:    int = 1000

    # Job release schedule (scaled time units, same as processing time)
    # released in groups of release_batch_size at release_interval spacing.
    # e.g. batch=10, interval=500 -> job 1~10: t=0, 11~20: t=500, 21~30: t=1000, ...
    release_interval:   float = 5.0
    initial_batch:      int   = 25
    release_batch_size: int   = 20

    # ── plan (initial) ───────────────────────────────────────────────
    initial_gantt_log_csv: str = "output/initial_gantt_log.csv"
    initial_gantt_csv:     str = "output/initial_gantt.csv"

    # ── actual results (result) ────────────────────────────────────────────
    result_gantt_log_csv:  str = "output/result_gantt_log.csv"
    result_gantt_csv:      str = "output/result_gantt.csv"
    simulation_log_gantt_csv: str = "output/simulation_log_gantt.csv"

    # ── legacy (kept for internal use) ────────────────────────────────────────────
    gantt_csv:        str = "output/gantt.csv"
    log_machine_csv:  str = "output/log_machine.csv"
    from_to_csv:      str = "output/from_to.csv"
    log_oht_csv:      str = "output/log_oht.csv"
    gantt_html:       str = "output/gantt.html"

    # ── additional: planning/execution rule experiment settings ────────────────────────────────
    planning_algorithm:    str = "SPT"
    # options: "SPT", "FIFO", "MWKR", "EXTERNAL"
    # EXTERNAL: follow the (job, op, machine, sequence) set by the CSV at external_schedule_path
    #           as-is, and only recompute earliest-start reflecting transport+release.

    # external schedule CSV path (required when planning_algorithm="EXTERNAL").
    # format: header job_id,op_index,machine_id (1-based); within one machine, row order is
    #         that machine's processing order.
    external_schedule_path: Optional[str] = None

    machine_dispatch_rule: str = "COMPOSITE_ATC_PD"
    # options: "SPT", "EDD", "CR", "ATC", "COMPOSITE_ATC_PD"

    # ── Machine layout optimization ─────────────────────────────────
    # If True, keeps the physical positions in layout_machine.csv, and
    # places equipment with high machine-to-machine flow in the JSSP route on nearby nodes.
    optimize_machine_layout: bool = False
    machine_layout_opt_iterations: int = 5000
    machine_layout_opt_seed: int = 42

    # ATC parameter
    atc_k:                 float = 2.0

    # Proposed = ATC + planned deviation penalty
    composite_order_w:     float = 0.60
    composite_start_w:     float = 0.25
    composite_atc_w:       float = 1.00
