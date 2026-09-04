"""Single-run CLI for the single-rail AMHS OHT dispatch simulator.

One invocation = one (method, seed) run. Use this to reproduce or inspect a
single cell; `main_exp_run.py` drives the full factorial.

There is no deadlock-recovery controller anywhere in this simulator: vehicles never receive a
force-proceed token, so a run is governed purely by the dispatching policy under
test. A run that gridlocks is not an error -- the watchdog ends it and its 24 h
numbers are still reported (see README, "Runs that freeze").

Subcommands
-----------
  savd   SAVD -- Hungarian dispatch + schedule-aware idle-vehicle positioning
  rule   one of the six baselines (NVF / STD / EDD / FIFO / PRIORITY / HUNGARIAN)

Examples
--------
  python run.py savd out_savd 1                  # proposed method
  python run.py savd out_hun  1 --savd-on 0      # its control: plain Hungarian
  python run.py rule out_nvf  1 NVF              # a baseline rule
  python run.py rule out_hun  1 HUNGARIAN --horizon 3600   # short smoke test
"""
import argparse
import os

import Simulation_Main as M

# Resolve the layout files from the repository, not the working directory, so
# `python /path/to/run.py ...` works from anywhere (the UI relies on this too).
_REPO = os.path.dirname(os.path.abspath(__file__))
LAYOUT_OHT = os.path.join(_REPO, "layout_oht.csv")
LAYOUT_MACHINE = os.path.join(_REPO, "layout_machine.csv")

RULES = ("NVF", "STD", "EDD", "FIFO", "PRIORITY", "HUNGARIAN")


def build_runner(out_dir, seed, n_oht, horizon):
    """Create a SimulationRunner with the settings shared by every experiment.

    Everything that is not the policy under test is pinned here -- fleet size,
    horizon, load/unload times, seed -- so two arms differ in exactly one thing.
    """
    cfg = M._jssp_cfg_with_output_dir(M.JSSPConfig(), out_dir)

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
    r.oht_config.oht_savd_positioning = False
    return r


def cmd_savd(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon)
    r.oht_config.oht_savd_positioning = bool(a.savd_on)
    r.oht_config.oht_savd_window = a.window
    r.oht_config.oht_savd_prior_weight = a.prior
    r.oht_config.oht_savd_grid = a.grid
    kpi = r.run(enable_animation=False)
    print("SAVD_DEBUG calls=%s assigns=%s errs=%s" %
          (getattr(r, "_savd_calls", 0), getattr(r, "_savd_assigns", 0),
           getattr(r, "_savd_errs", 0)), flush=True)
    print("SAVD_RES out=%s seed=%d n=%d savd=%d win=%.0f prior=%.1f grid=%.1f "
          "thr=%s makespan_attain_h=%s rate=%s util=%s "
          "sw_mean=%s sw_p90=%s sw_p95=%s empty2src=%s transp_dev=%s" %
          (a.out, a.seed, a.n_oht, int(bool(a.savd_on)), a.window, a.prior, a.grid,
           kpi.get("throughput_24h"), kpi.get("makespan_attain_h"),
           kpi.get("makespan_attain_rate_pct"),
           kpi.get("machine_utilization_pct"),
           kpi.get("mean_source_wait_s"), kpi.get("source_wait_p90_s"),
           kpi.get("source_wait_p95_s"),
           kpi.get("mean_empty_to_source_s"), kpi.get("mean_transport_deviation_s")),
          flush=True)


def cmd_rule(a):
    r = build_runner(a.out, a.seed, a.n_oht, a.horizon)
    r.oht_config.oht_dispatch_mode = a.mode
    kpi = r.run(enable_animation=False)
    print("RULE_RES mode=%s seed=%d n=%d thr=%s makespan_attain_h=%s rate=%s "
          "makespan_24h_h=%s util=%s sw_mean=%s sw_p90=%s empty2src=%s" %
          (a.mode, a.seed, a.n_oht, kpi.get("throughput_24h"),
           kpi.get("makespan_attain_h"), kpi.get("makespan_attain_rate_pct"),
           kpi.get("makespan_24h_h"), kpi.get("machine_utilization_pct"),
           kpi.get("mean_source_wait_s"), kpi.get("source_wait_p90_s"),
           kpi.get("mean_empty_to_source_s")),
          flush=True)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp):
        sp.add_argument("out", help="output directory")
        sp.add_argument("seed", type=int, help="RNG seed")
        sp.add_argument("--n-oht", type=int, default=120, help="fleet size")
        sp.add_argument("--horizon", type=float, default=86400.0, help="sim seconds")

    savd = sub.add_parser("savd", help="SAVD: Hungarian + schedule-aware idle positioning")
    add_common(savd)
    savd.add_argument("--savd-on", type=int, choices=(0, 1), default=1,
                      help="1 = SAVD (proposed), 0 = plain Hungarian control")
    savd.add_argument("--window", type=float, default=900.0,
                      help="demand look-ahead window H (sec)")
    savd.add_argument("--prior", type=float, default=1.0,
                      help="schedule geographic-prior weight lambda")
    savd.add_argument("--grid", type=float, default=0.0,
                      help="SAVD cell size (0 = auto = layout span / 4)")
    savd.set_defaults(func=cmd_savd)

    rule = sub.add_parser("rule", help="one of the six baseline dispatch rules")
    add_common(rule)
    rule.add_argument("mode", choices=RULES, help=" | ".join(RULES))
    rule.set_defaults(func=cmd_rule)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
