"""Unified experiment CLI for the single-rail AMHS OHT dispatch simulator.

Replaces the former per-experiment scripts with one entry point and shared
configuration.

Subcommands
-----------
  recovery   HUNGARIAN + global deadlock recovery (optionally + plan-congestion / gantt-priority / positioning)
  coverage   recovery + gantt-aware idle-vehicle coverage (the main result)
  rule       a classic dispatch rule (NVF/STD/EDD/FIFO/PRIORITY/QS_STD/BA_STD) standalone, no recovery
  plan       plan-congestion-aware Hungarian (standalone)

Examples
--------
  python run.py coverage out_cov 99 --cov-on 1              # recovery + coverage
  python run.py coverage out_rec 99 --cov-on 0              # recovery-only control
  python run.py recovery out_rec 99                         # recovery only
  python run.py rule     out_nvf 99 NVF                     # classic rule, no recovery
  python run.py plan     out_pl  99 2.0                     # plan-congestion Hungarian (weight 2.0)

Each subcommand prints a single result line (REC_RES / COV_RES / RULE_RES /
PLAN_RES).
"""
import argparse
import sys

import Simualtion_Main as M

LAYOUT_OHT = "layout_oht.csv"
LAYOUT_MACHINE = "layout_machine.csv"


def build_runner(out_dir, seed, n_oht, horizon, wip=0, plan_csv=""):
    """Create a SimulationRunner with the settings shared by every experiment.

    Reactive heuristics are all disabled here; each subcommand enables only the
    mechanism it studies (recovery, coverage, a specific dispatch mode, ...).
    """
    cfg = M._jssp_cfg_with_output_dir(M.JSSPConfig(), out_dir)
    if wip > 0:
        import dataclasses as _dc
        cfg = _dc.replace(cfg, wip_cap=wip)
    if plan_csv:
        setattr(cfg, "planned_gantt_csv", plan_csv)

    r = M.SimulationRunner(layout_csv_path=LAYOUT_OHT,
                           machine_csv_path=LAYOUT_MACHINE, jssp_cfg=cfg)
    r.oht_config.enable_animation = False
    r.oht_config.sim_horizon = horizon
    r.oht_config.seed = seed
    r.oht_config.n_oht = n_oht
    for a, b in [("load_time_min", 25), ("load_time_max", 25),
                 ("unload_time_min", 25), ("unload_time_max", 25)]:
        setattr(r.oht_config, a, b)
    r.oht_config.oht_dispatch_mode = "HUNGARIAN"
    r.oht_config.oht_blocking_weight = 0
    r.oht_config.oht_urgency_weight = 0
    r.oht_config.oht_congestion_cost_weight = 0
    r.oht_config.oht_idle_positioning = False
    r.oht_config.oht_predispatch_lookahead = 0.0
    r.oht_config.oht_plan_congestion_weight = 0.0
    return r


def cmd_recovery(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon, a.wip, a.plan_csv)
    r.oht_config.oht_predispatch_lookahead = a.predisp
    r.oht_config.oht_deadlock_recovery = True              # global detection + recovery
    r.oht_config.oht_dl_gantt_priority = bool(a.gantt_prio)  # cycle-breaking leader: True=prioritize plan delay
    r.oht_config.oht_plan_congestion_weight = a.plan_pw     # 0=recovery only, >0=2-layer
    kpi = r.run(enable_animation=False)
    print("REC_DEBUG grants=%s" % getattr(r, "_dl_grants", 0), flush=True)
    print("REC_RES out=%s seed=%d n=%d plan_pw=%.1f gp=%d predisp=%.0f thr=%s makespan_attain_h=%s "
          "util=%s source_wait=%s empty2src=%s transp_dev=%s" %
          (a.out, a.seed, a.n_oht, a.plan_pw, int(bool(a.gantt_prio)), a.predisp,
           kpi.get("throughput_24h"), kpi.get("makespan_attain_h"),
           kpi.get("machine_utilization_pct"), kpi.get("mean_source_wait_s"),
           kpi.get("mean_empty_to_source_s"), kpi.get("mean_transport_deviation_s")),
          flush=True)


def cmd_coverage(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon)
    r.oht_config.oht_deadlock_recovery = True              # recovery always on (guarantees n120 survival)
    r.oht_config.oht_coverage_positioning = bool(a.cov_on)  # (A) gantt-aware coverage
    r.oht_config.oht_coverage_window = a.window
    r.oht_config.oht_coverage_prior_weight = a.prior
    r.oht_config.oht_coverage_grid = a.grid
    kpi = r.run(enable_animation=False)
    print("REC_DEBUG grants=%s cov_calls=%s cov_assigns=%s cov_errs=%s" %
          (getattr(r, "_dl_grants", 0), getattr(r, "_cov_calls", 0),
           getattr(r, "_cov_assigns", 0), getattr(r, "_cov_errs", 0)), flush=True)
    print("COV_RES out=%s seed=%d n=%d cov=%d win=%.0f prior=%.1f grid=%.1f "
          "thr=%s makespan_attain_h=%s util=%s "
          "sw_mean=%s sw_p90=%s sw_p95=%s sw_max=%s sw_std=%s "
          "empty2src=%s transp_dev=%s" %
          (a.out, a.seed, a.n_oht, int(bool(a.cov_on)), a.window, a.prior, a.grid,
           kpi.get("throughput_24h"), kpi.get("makespan_attain_h"),
           kpi.get("machine_utilization_pct"),
           kpi.get("mean_source_wait_s"), kpi.get("source_wait_p90_s"),
           kpi.get("source_wait_p95_s"), kpi.get("source_wait_max_s"),
           kpi.get("source_wait_std_s"),
           kpi.get("mean_empty_to_source_s"), kpi.get("mean_transport_deviation_s")),
          flush=True)


def cmd_rule(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon)
    r.oht_config.oht_dispatch_mode = a.mode
    r.oht_config.oht_deadlock_recovery = False            # existing rule as-is (no recovery)
    r.oht_config.oht_coverage_positioning = False
    kpi = r.run(enable_animation=False)
    print("RULE_RES mode=%s seed=%d n=%d thr=%s makespan_attain_h=%s rate=%s "
          "makespan_24h_h=%s util=%s" %
          (a.mode, a.seed, a.n_oht, kpi.get("throughput_24h"),
           kpi.get("makespan_attain_h"), kpi.get("makespan_attain_rate_pct"),
           kpi.get("makespan_24h_h"), kpi.get("machine_utilization_pct")),
          flush=True)


def cmd_plan(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon)
    r.oht_config.oht_plan_congestion_weight = a.pw
    kpi = r.run(enable_animation=False)
    print("PLAN_RES out=%s seed=%d pw=%.2f n=%d thr=%s makespan_attain_h=%s util=%s" %
          (a.out, a.seed, a.pw, a.n_oht, kpi.get("throughput_24h"),
           kpi.get("makespan_attain_h"), kpi.get("machine_utilization_pct")),
          flush=True)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    # common positional args helper
    def add_common(sp):
        sp.add_argument("out", help="output directory")
        sp.add_argument("seed", type=int, help="RNG seed")

    rec = sub.add_parser("recovery", help="Hungarian + global deadlock recovery")
    add_common(rec)
    rec.add_argument("--horizon", type=float, default=86400.0)
    rec.add_argument("--n-oht", type=int, default=120)
    rec.add_argument("--plan-pw", type=float, default=0.0, help="plan-congestion weight (0=recovery only)")
    rec.add_argument("--gantt-prio", action="store_true", help="cycle-breaking leader = most-delayed lot")
    rec.add_argument("--predisp", type=float, default=0.0, help="per-source positioning lookahead (sec)")
    rec.add_argument("--wip", type=int, default=0, help="wip_cap override (0=default)")
    rec.add_argument("--plan-csv", default="", help="planned_gantt.csv path (''=default)")
    rec.set_defaults(func=cmd_recovery)

    cov = sub.add_parser("coverage", help="recovery + gantt-aware idle-vehicle coverage")
    add_common(cov)
    cov.add_argument("--cov-on", type=int, choices=(0, 1), default=1, help="1=+coverage, 0=recovery-only control")
    cov.add_argument("--horizon", type=float, default=86400.0)
    cov.add_argument("--n-oht", type=int, default=120)
    cov.add_argument("--window", type=float, default=900.0, help="demand lookahead window (sec)")
    cov.add_argument("--prior", type=float, default=1.0, help="gantt geographic prior weight")
    cov.add_argument("--grid", type=float, default=0.0, help="cell size (0=auto = bbox/4)")
    cov.set_defaults(func=cmd_coverage)

    rule = sub.add_parser("rule", help="classic dispatch rule standalone (no recovery)")
    add_common(rule)
    rule.add_argument("mode", help="NVF | STD | EDD | FIFO | PRIORITY | QS_STD | BA_STD")
    rule.add_argument("--n-oht", type=int, default=120)
    rule.add_argument("--horizon", type=float, default=86400.0)
    rule.set_defaults(func=cmd_rule)

    plan = sub.add_parser("plan", help="plan-congestion-aware Hungarian (standalone)")
    add_common(plan)
    plan.add_argument("pw", type=float, help="plan-congestion weight")
    plan.add_argument("--horizon", type=float, default=86400.0)
    plan.add_argument("--n-oht", type=int, default=120)
    plan.set_defaults(func=cmd_plan)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
