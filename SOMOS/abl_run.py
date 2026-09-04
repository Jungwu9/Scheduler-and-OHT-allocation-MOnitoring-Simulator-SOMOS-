"""abl_run.py -- SAVD ablation / hyper-parameter sensitivity sweep.

One-factor-at-a-time around the proposed default
(prior_weight=1.0, window=900 s, grid=auto=span/4, dt=45 s, terms='all'):

  demand-term    : prior0 (realized only) | terms_prior (plan geography only)
  prior weight   : 0.5 | 2.0
  window length  : 150 | 300 | 450 | 1800 | 2700 s
  grid resolution: span/2 (coarse) | span/6 (fine)          [machine-node span = 598.0]
  update period  : 15 s | 120 s
  rolling window : roll300 / roll900 / roll2700 / rollonly900 / roll900_noanc

The `roll*` configs replace the static gantt prior (a whole-horizon histogram with
no time axis) with the pickups actually due in [anchor, anchor+H] on the plan time
axis. In the default build `window` only gates the one job currently in process at
each machine, so it saturates above the longest processing time; the rolling term
is what makes it a genuine look-ahead.

The two reference points -- `HUNGARIAN` (SAVD off) and `SAVD` (the default
configuration) -- are NOT re-run here: they come from the main
experiment (main_exp_run.py) on the same seeds, so every comparison stays
paired.

    python abl_run.py                       # all configs, seeds 1-10
    python abl_run.py --seeds 1-3           # smoke test
    python abl_run.py --configs prior0,dt15
"""
import argparse
import csv
import os
import time

from run import build_runner
from main_exp_run import KPI_FIELDS, SURVIVAL_RATE_PCT, _mean_std

# machine-node bounding-box span (measured from the built layout); auto grid = span/4
LAYOUT_SPAN = 598.0

# proposed default -- every config below is this with exactly one factor changed
DEFAULT = dict(prior=1.0, window=900.0, grid=0.0, dt=45.0, terms="all", anchor=True)


# (name, factor, overrides, human-readable level)
CONFIGS = [
    ("prior0",      "demand term",   dict(prior=0.0),                  "realized only (no gantt prior)"),
    ("terms_prior", "demand term",   dict(terms="prior"),              "gantt prior only (no realized)"),
    ("prior05",     "prior weight",  dict(prior=0.5),                  "lambda = 0.5"),
    ("prior2",      "prior weight",  dict(prior=2.0),                  "lambda = 2.0"),
    ("win150",      "look-ahead",    dict(window=150.0),               "H = 150 s"),
    ("win300",      "look-ahead",    dict(window=300.0),               "H = 300 s"),
    ("win450",      "look-ahead",    dict(window=450.0),               "H = 450 s"),
    ("win1800",     "look-ahead",    dict(window=1800.0),              "H = 1800 s"),
    ("win2700",     "look-ahead",    dict(window=2700.0),              "H = 2700 s"),
    ("grid2",       "grid",          dict(grid=LAYOUT_SPAN / 2.0),     "span/2 (2 cells/axis)"),
    ("grid6",       "grid",          dict(grid=LAYOUT_SPAN / 6.0),     "span/6 (6 cells/axis)"),
    ("dt15",        "update period", dict(dt=15.0),                    "dt = 15 s"),
    ("dt120",       "update period", dict(dt=120.0),                   "dt = 120 s"),
    # Rolling plan window: replaces the STATIC prior (a whole-horizon histogram with no
    # time axis) with the pickups actually due in [anchor, anchor+H] on the plan time
    # axis. This is what turns `window` into a real look-ahead instead of a gate that
    # saturates once H exceeds the longest processing time (676.5 s in this plan).
    ("roll300",       "rolling window", dict(terms="roll", window=300.0),  "rolling, H = 300 s"),
    ("roll900",       "rolling window", dict(terms="roll"),                "rolling, H = 900 s"),
    ("roll2700",      "rolling window", dict(terms="roll", window=2700.0), "rolling, H = 2700 s"),
    ("rollonly900",   "rolling window", dict(terms="rollonly"),            "rolling only, H = 900 s"),
    ("roll900_noanc", "rolling window", dict(terms="roll", anchor=False),  "rolling, no plan anchor"),
]


def apply_config(r, over):
    cfg = dict(DEFAULT)
    cfg.update(over)
    r.oht_config.oht_dispatch_mode = "HUNGARIAN"
    r.oht_config.oht_savd_positioning = True
    r.oht_config.oht_savd_prior_weight = cfg["prior"]
    r.oht_config.oht_savd_window = cfg["window"]
    r.oht_config.oht_savd_grid = cfg["grid"]
    r.oht_config.oht_savd_dt = cfg["dt"]
    r.oht_config.oht_savd_terms = cfg["terms"]
    r.oht_config.oht_savd_roll_anchor = cfg["anchor"]
    return cfg


def run_one(name, over, seed, n_oht, horizon, out_dir):
    r = build_runner(out_dir, seed, n_oht, horizon)
    cfg = apply_config(r, over)
    kpi = r.run(enable_animation=False)
    row = {"config": name, "seed": seed, "status": "ok",
           "prior": cfg["prior"], "window": cfg["window"], "grid": round(cfg["grid"], 1),
           "dt": cfg["dt"], "terms": cfg["terms"], "anchor": int(cfg["anchor"]),
           "savd_assigns": getattr(r, "_savd_assigns", 0)}
    for k in KPI_FIELDS:
        row[k] = kpi.get(k)
    rate = kpi.get("makespan_attain_rate_pct")
    row["survived"] = int(isinstance(rate, (int, float)) and rate >= SURVIVAL_RATE_PCT)
    return row


FACTOR = {n: f for n, f, _, _ in CONFIGS}
LEVEL = {n: l for n, _, _, l in CONFIGS}


def write_summary(rows, path):
    groups = {}
    for row in rows:
        if row.get("status") != "ok":
            continue
        groups.setdefault(row["config"], []).append(row)

    fieldnames = ["config", "factor", "level", "n_ok", "n_survived"]
    for k in KPI_FIELDS:
        fieldnames += [f"{k}_mean", f"{k}_std"]

    order = {n: i for i, (n, _, _, _) in enumerate(CONFIGS)}
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for name in sorted(groups, key=lambda k: order.get(k, 99)):
            grp = groups[name]
            out = {"config": name, "factor": FACTOR.get(name, ""),
                   "level": LEVEL.get(name, ""), "n_ok": len(grp),
                   "n_survived": sum(int(g.get("survived") or 0) for g in grp)}
            for k in KPI_FIELDS:
                m, s, _ = _mean_std([g.get(k) for g in grp])
                out[f"{k}_mean"] = None if m is None else round(m, 4)
                out[f"{k}_std"] = None if s is None else round(s, 4)
            w.writerow(out)
    return groups


def print_console_summary(groups):
    print("\n" + "=" * 96)
    print("COVERAGE ABLATION  (mean over seeds)  --  makespan_attain_h lower = better")
    print("=" * 96)
    print(f"{'config':<12} {'factor':<14} {'n':>3} {'surv':>6} {'attain_h':>9} "
          f"{'thr24h':>8} {'src_wait':>9} {'sw_p90':>8} {'e2src':>8}")
    print("-" * 96)
    order = {n: i for i, (n, _, _, _) in enumerate(CONFIGS)}
    for name in sorted(groups, key=lambda k: order.get(k, 99)):
        grp = groups[name]

        def mean(k):
            m, _, _ = _mean_std([g.get(k) for g in grp])
            return m

        def fmt(x, p=2):
            return f"{x:.{p}f}" if isinstance(x, (int, float)) else "  n/a"

        n_srv = sum(int(g.get("survived") or 0) for g in grp)
        surv = f"{n_srv}/{len(grp)}"
        print(f"{name:<12} {FACTOR.get(name, ''):<14} {len(grp):>3} {surv:>6} "
              f"{fmt(mean('makespan_attain_h')):>9} "
              f"{fmt(mean('throughput_24h'), 1):>8} "
              f"{fmt(mean('mean_source_wait_s'), 1):>9} "
              f"{fmt(mean('source_wait_p90_s'), 1):>8} "
              f"{fmt(mean('mean_empty_to_source_s'), 1):>8}")
    print("=" * 96)


def parse_seeds(spec):
    spec = spec.strip()
    if "-" in spec and "," not in spec:
        lo, hi = spec.split("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",") if x.strip()]


def main(argv=None):
    p = argparse.ArgumentParser(description="SAVD ablation / sensitivity sweep")
    p.add_argument("--out", default=os.path.join("results", "abl"))
    p.add_argument("--seeds", default="1-10")
    p.add_argument("--n-oht", type=int, default=120)
    p.add_argument("--horizon", type=float, default=86400.0)
    p.add_argument("--configs", default="", help="comma-separated subset of config names")
    a = p.parse_args(argv)

    seeds = parse_seeds(a.seeds)
    configs = CONFIGS
    if a.configs:
        want = {c.strip() for c in a.configs.split(",") if c.strip()}
        configs = [c for c in CONFIGS if c[0] in want]
        if not configs:
            raise SystemExit(f"no configs matched {sorted(want)}")

    os.makedirs(a.out, exist_ok=True)
    scratch = os.path.join(a.out, "scratch")
    os.makedirs(scratch, exist_ok=True)
    runs_csv = os.path.join(a.out, "runs.csv")
    summary_csv = os.path.join(a.out, "summary.csv")

    total = len(configs) * len(seeds)
    print(f"Planned runs: {total}  ({len(configs)} configs x {len(seeds)} seeds)")
    print(f"  configs : {[c[0] for c in configs]}")
    print(f"  seeds   : {seeds}")
    print(f"  n_oht={a.n_oht}  horizon={a.horizon:.0f}s  (no deadlock recovery)\n")

    meta = ["config", "seed", "status", "prior", "window", "grid", "dt", "terms",
            "anchor", "savd_assigns"]
    fieldnames = meta + KPI_FIELDS + ["survived", "elapsed_s"]
    rows = []
    t0 = time.time()
    done = 0

    with open(runs_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        f.flush()
        for name, _factor, over, _level in configs:
            for seed in seeds:
                done += 1
                rt0 = time.time()
                try:
                    row = run_one(name, over, seed, a.n_oht, a.horizon, scratch)
                except Exception as exc:
                    row = {"config": name, "seed": seed,
                           "status": f"ERROR: {type(exc).__name__}: {exc}"}
                    for k in KPI_FIELDS:
                        row[k] = None
                    row["survived"] = None
                row["elapsed_s"] = round(time.time() - rt0, 1)
                rows.append(row)
                writer.writerow(row)
                f.flush()
                attain = row.get("makespan_attain_h")
                att = f"{attain:.2f}" if isinstance(attain, (int, float)) else row["status"]
                eta = (time.time() - t0) / done * (total - done)
                print(f"[{done}/{total}] {name} seed={seed} attain_h={att} "
                      f"({row['elapsed_s']:.0f}s, ETA {eta/60:.1f}m)", flush=True)

    groups = write_summary(rows, summary_csv)
    print_console_summary(groups)
    print(f"\nDone: {len(rows)} runs, {(time.time()-t0)/60:.1f} min")
    print(f"  per-run : {runs_csv}")
    print(f"  summary : {summary_csv}")


if __name__ == "__main__":
    main()
