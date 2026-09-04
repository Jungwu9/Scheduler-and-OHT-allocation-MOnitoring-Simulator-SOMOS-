"""main_exp_run.py -- main experiment: 7 dispatch methods x N seeds.

There is no deadlock-recovery controller anywhere in this simulator, so every
run is governed purely by the dispatching policy under test.

    7 methods x 20 seeds = 140 runs

The 7 methods are the 6 classic baselines (NVF / STD / EDD / FIFO / PRIORITY /
HUNGARIAN) plus the proposed method SAVD (= HUNGARIAN dispatch + schedule-aware
idle-vehicle SAVD positioning).

Because no recovery controller intervenes, a run can freeze (merge-yield
gridlock) before the 24 h horizon. Such a run is NOT an error: the simulation
still finishes and its 24 h numbers are recorded. `makespan_24h_h` (time of the
last completion within 24 h) exposes the freeze time, and `survived` flags runs
that stayed within 25% of the plan's 24 h progress (attainment rate >= 75%).

Results are written incrementally (one row per run, flushed immediately) so a
crash mid-experiment never loses completed runs, plus an aggregated summary.

Just run it:
    python main_exp_run.py

Optional overrides:
    python main_exp_run.py --seeds 1-5 --n-oht 80        # smaller smoke test
    python main_exp_run.py --methods NVF,SAVD            # subset of methods
"""
import argparse
import csv
import os
import statistics
import time

from run import build_runner  # reuse the shared runner factory from run.py


# ---------------------------------------------------------------------------
# Method definitions
# ---------------------------------------------------------------------------
# 6 classic baselines: each is just a dispatch mode with SAVD positioning off.
BASELINE_RULES = ["NVF", "STD", "EDD", "FIFO", "PRIORITY", "HUNGARIAN"]

# a run is "survived" if its 24 h output matches >= this % of the plan's progress
SURVIVAL_RATE_PCT = 75.0


def _apply_rule(r, mode):
    """Configure the runner for a classic dispatch rule (SAVD off)."""
    r.oht_config.oht_dispatch_mode = mode
    r.oht_config.oht_savd_positioning = False


def _apply_savd(r):
    """Configure the runner for the proposed method, SAVD.

    HUNGARIAN dispatch + schedule-aware idle-vehicle SAVD positioning, with
    the same parameters as run.py's `savd` subcommand defaults.
    """
    r.oht_config.oht_dispatch_mode = "HUNGARIAN"
    r.oht_config.oht_savd_positioning = True
    r.oht_config.oht_savd_window = 900.0
    r.oht_config.oht_savd_prior_weight = 1.0
    r.oht_config.oht_savd_grid = 0.0


def build_methods():
    """Return an ordered list of (name, configure_fn) for the 7 methods."""
    methods = [(m, (lambda r, mm=m: _apply_rule(r, mm))) for m in BASELINE_RULES]
    methods.append(("SAVD", _apply_savd))
    return methods


# KPIs captured per run. Names must match keys returned by SimulationRunner.run().
KPI_FIELDS = [
    "throughput_24h",
    "makespan_attain_h",
    "makespan_attain_rate_pct",
    "makespan_24h_h",
    "machine_utilization_pct",
    "mean_source_wait_s",
    "source_wait_p90_s",
    "source_wait_p95_s",
    "mean_empty_to_source_s",
    "mean_transport_deviation_s",
    "mean_cycle_time_24h_h",
]


# ---------------------------------------------------------------------------
# Single run
# ---------------------------------------------------------------------------
def run_one(method_name, configure, seed, n_oht, horizon, out_dir):
    """Execute one (method, seed) run and return a result-row dict."""
    r = build_runner(out_dir, seed, n_oht, horizon)
    configure(r)
    kpi = r.run(enable_animation=False)

    row = {"method": method_name, "seed": seed, "status": "ok"}
    for k in KPI_FIELDS:
        row[k] = kpi.get(k)
    rate = kpi.get("makespan_attain_rate_pct")
    row["survived"] = int(isinstance(rate, (int, float)) and rate >= SURVIVAL_RATE_PCT)
    return row


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
def _mean_std(values):
    vals = [v for v in values if isinstance(v, (int, float))]
    if not vals:
        return (None, None, 0)
    m = statistics.fmean(vals)
    s = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return (m, s, len(vals))


def write_summary(rows, path):
    """Aggregate per method: mean/std of each KPI over seeds, plus survival count."""
    groups = {}
    for row in rows:
        if row.get("status") != "ok":
            continue
        groups.setdefault(row["method"], []).append(row)

    fieldnames = ["method", "n_ok", "n_survived", "survival_rate"]
    for k in KPI_FIELDS:
        fieldnames += [f"{k}_mean", f"{k}_std"]

    order = {name: i for i, (name, _) in enumerate(build_methods())}
    keys = sorted(groups.keys(), key=lambda k: order.get(k, 99))

    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for method in keys:
            grp = groups[method]
            n_srv = sum(int(g.get("survived") or 0) for g in grp)
            out = {"method": method, "n_ok": len(grp), "n_survived": n_srv,
                   "survival_rate": round(n_srv / len(grp), 3) if grp else None}
            for k in KPI_FIELDS:
                m, s, _ = _mean_std([g.get(k) for g in grp])
                out[f"{k}_mean"] = None if m is None else round(m, 4)
                out[f"{k}_std"] = None if s is None else round(s, 4)
            w.writerow(out)
    return groups


def print_console_summary(groups):
    """Print a compact per-method comparison to stdout."""
    print("\n" + "=" * 88)
    print("SUMMARY  (mean over seeds)  --  key metric: makespan_attain_h (lower=better)")
    print("=" * 88)
    print(f"{'method':<10} {'n':>3} {'surv':>6} {'attain_h':>9} {'rate%':>7} "
          f"{'thr24h':>8} {'util%':>7} {'src_wait':>9} {'sw_p90':>8}")
    print("-" * 88)
    order = {name: i for i, (name, _) in enumerate(build_methods())}
    for method in sorted(groups.keys(), key=lambda k: order.get(k, 99)):
        grp = groups[method]

        def mean(k):
            m, _, _ = _mean_std([g.get(k) for g in grp])
            return m

        def fmt(x, p=2):
            return f"{x:.{p}f}" if isinstance(x, (int, float)) else "  n/a"

        n_srv = sum(int(g.get("survived") or 0) for g in grp)
        surv = f"{n_srv}/{len(grp)}"
        print(f"{method:<10} {len(grp):>3} {surv:>6} "
              f"{fmt(mean('makespan_attain_h')):>9} "
              f"{fmt(mean('makespan_attain_rate_pct'), 1):>7} "
              f"{fmt(mean('throughput_24h'), 1):>8} "
              f"{fmt(mean('machine_utilization_pct'), 1):>7} "
              f"{fmt(mean('mean_source_wait_s'), 1):>9} "
              f"{fmt(mean('source_wait_p90_s'), 1):>8}")
    print("=" * 88)

    # per-method gap against the proposed method on the headline metric
    prop = groups.get("SAVD")
    if prop:
        m_prop, _, _ = _mean_std([g.get("makespan_attain_h") for g in prop])
        if m_prop is not None:
            print("\nmakespan_attain_h vs proposed (SAVD)  "
                  "(baseline - proposed; positive => proposed better)")
            print("-" * 64)
            for name, _ in build_methods():
                if name == "SAVD" or name not in groups:
                    continue
                m_b, _, _ = _mean_std([g.get("makespan_attain_h") for g in groups[name]])
                if m_b is None:
                    continue
                print(f"  {name:<10}  base={m_b:8.2f}  proposed={m_prop:8.2f}  "
                      f"delta={m_b - m_prop:+8.2f}")
            print("-" * 64)


# ---------------------------------------------------------------------------
# Seed / arg parsing
# ---------------------------------------------------------------------------
def parse_seeds(spec):
    """Parse '1-20' or '1,2,5' or '42-61' into a list of ints."""
    spec = spec.strip()
    if "-" in spec and "," not in spec:
        lo, hi = spec.split("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",") if x.strip()]


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="main experiment: 7 dispatch methods x N seeds",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.join("results", "main_exp"),
                   help="results directory (default: results/main_exp)")
    p.add_argument("--seeds", default="1-20",
                   help="seed spec: '1-20' or '1,2,3' (default: 1-20)")
    p.add_argument("--n-oht", type=int, default=120, help="fleet size (default: 120)")
    p.add_argument("--horizon", type=float, default=86400.0,
                   help="sim horizon in seconds (default: 86400 = 24h)")
    p.add_argument("--methods", default="",
                   help="comma-separated subset of methods (default: all 7)")
    return p.parse_args(argv)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv=None):
    a = parse_args(argv)
    seeds = parse_seeds(a.seeds)

    methods = build_methods()
    if a.methods:
        wanted = {m.strip() for m in a.methods.split(",") if m.strip()}
        methods = [(n, f) for (n, f) in methods if n in wanted]
        if not methods:
            raise SystemExit(f"no methods matched {sorted(wanted)}")

    os.makedirs(a.out, exist_ok=True)
    scratch = os.path.join(a.out, "scratch")   # sim output dir, reused across runs
    os.makedirs(scratch, exist_ok=True)
    runs_csv = os.path.join(a.out, "runs.csv")
    summary_csv = os.path.join(a.out, "summary.csv")

    total = len(methods) * len(seeds)
    print(f"Planned runs: {total}  ({len(methods)} methods x {len(seeds)} seeds)")
    print(f"  methods : {[n for n, _ in methods]}")
    print(f"  seeds   : {seeds}")
    print(f"  n_oht={a.n_oht}  horizon={a.horizon:.0f}s  (no deadlock recovery)")
    print(f"  writing : {runs_csv}\n")

    fieldnames = ["method", "seed", "status"] + KPI_FIELDS + ["survived", "elapsed_s"]
    rows = []
    t0 = time.time()
    done = 0

    with open(runs_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        f.flush()

        for name, configure in methods:
            for seed in seeds:
                done += 1
                tag = f"[{done}/{total}] {name} seed={seed}"
                rt0 = time.time()
                try:
                    row = run_one(name, configure, seed, a.n_oht, a.horizon, scratch)
                except Exception as exc:  # keep the batch alive on a single failure
                    row = {"method": name, "seed": seed,
                           "status": f"ERROR: {type(exc).__name__}: {exc}"}
                    for k in KPI_FIELDS:
                        row[k] = None
                    row["survived"] = None
                row["elapsed_s"] = round(time.time() - rt0, 1)

                rows.append(row)
                writer.writerow(row)
                f.flush()

                elapsed = time.time() - t0
                eta = (elapsed / done) * (total - done)
                attain = row.get("makespan_attain_h")
                attain_s = f"{attain:.2f}" if isinstance(attain, (int, float)) else row["status"]
                print(f"{tag:<36} attain_h={attain_s:<10} surv={row.get('survived')} "
                      f"({row['elapsed_s']:.0f}s, ETA {eta/60:.1f}m)", flush=True)

    groups = write_summary(rows, summary_csv)
    print_console_summary(groups)

    n_err = sum(1 for r in rows if r.get("status") != "ok")
    print(f"\nDone: {len(rows)} runs, {n_err} errors, "
          f"{(time.time() - t0)/60:.1f} min total")
    print(f"  per-run : {runs_csv}")
    print(f"  summary : {summary_csv}")


if __name__ == "__main__":
    main()
