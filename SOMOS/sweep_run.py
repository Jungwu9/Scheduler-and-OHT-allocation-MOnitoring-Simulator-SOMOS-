"""sweep_run.py -- robustness sweep: 7 dispatch methods x fleet size x seeds.

This is the density-sweep counterpart to `main_exp_run.py` (which is fixed at
n=120). The question it answers is robustness: how does each policy degrade as
the fleet gets denser and the rail saturates?

The methods are the same seven as the main experiment: the six baselines
(NVF / STD / EDD / FIFO / PRIORITY / HUNGARIAN) and the proposed SAVD.

Because this build has no deadlock recovery, a run can freeze in a merge-yield
gridlock. The simulator's watchdog (see `Simulation_Main._run_event_loop`) ends
such a run early and reports `gridlock_time_h`, the moment progress stopped.
That is lossless for the 24 h KPIs -- a frozen system completes nothing more --
and it is what makes the full sweep affordable.

Results
-------
Every shard writes ONE csv into a single flat directory, named by its slice:

    results/sweep/runs_n120_s7.csv

so the whole sweep lives in one folder and `--merge` concatenates it into
`results/sweep/all_runs.csv`.

    python sweep_run.py --n-oht 80                    # 7 methods x seeds 1-20
    python sweep_run.py --n-oht 80 --seeds 1          # one shard (for parallel launch)
    python sweep_run.py --merge                       # combine shards + summary
"""
import argparse
import csv
import glob
import os
import time

from run import build_runner
from main_exp_run import (KPI_FIELDS, SURVIVAL_RATE_PCT, BASELINE_RULES,
                          _apply_rule, _apply_savd, parse_seeds)


# ---------------------------------------------------------------------------
# Methods -- identical to the main experiment, so the two tables are comparable
# ---------------------------------------------------------------------------
METHODS = ([(m, (lambda r, mm=m: _apply_rule(r, mm))) for m in BASELINE_RULES]
           + [("SAVD", _apply_savd)])

METHOD_ORDER = [n for n, _ in METHODS]

# gridlock diagnostics appended to the shared KPI schema
GRIDLOCK_FIELDS = ["gridlock_detected", "gridlock_time_h", "sim_end_h"]
FIELDS = (["method", "n_oht", "seed", "status"]
          + KPI_FIELDS + GRIDLOCK_FIELDS + ["survived", "elapsed_s"])


# ---------------------------------------------------------------------------
# Single run
# ---------------------------------------------------------------------------
def run_one(name, configure, seed, n_oht, horizon, scratch):
    r = build_runner(scratch, seed, n_oht, horizon)
    configure(r)
    kpi = r.run(enable_animation=False)

    row = {"method": name, "n_oht": n_oht, "seed": seed, "status": "ok"}
    for k in KPI_FIELDS + GRIDLOCK_FIELDS:
        row[k] = kpi.get(k)
    rate = kpi.get("makespan_attain_rate_pct")
    row["survived"] = int(isinstance(rate, (int, float)) and rate >= SURVIVAL_RATE_PCT)
    return row


# ---------------------------------------------------------------------------
# Merge
# ---------------------------------------------------------------------------
def merge(out_dir):
    """Concatenate every shard csv in `out_dir` into all_runs.csv + summary.csv."""
    import statistics as st

    shards = sorted(glob.glob(os.path.join(out_dir, "runs_n*_s*.csv")))
    rows = []
    for path in shards:
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    if not rows:
        raise SystemExit("no shard csv found in " + out_dir)

    all_csv = os.path.join(out_dir, "all_runs.csv")
    with open(all_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    # per (n_oht, method): survival count + mean of each KPI
    groups = {}
    for r in rows:
        groups.setdefault((int(r["n_oht"]), r["method"]), []).append(r)

    def fnum(rs, k):
        v = []
        for r in rs:
            try:
                v.append(float(r[k]))
            except (TypeError, ValueError):
                pass
        return v

    def surv_count(rs):
        return sum(int(float(r["survived"])) for r in rs
                   if r.get("survived") not in (None, ""))

    sum_csv = os.path.join(out_dir, "summary.csv")
    hdr = ["n_oht", "method", "runs", "survived", "survival_pct",
           "gridlocked", "mean_gridlock_time_h"]
    # Robustness is a claim about the whole seed distribution, so every KPI is
    # the UNCONDITIONAL mean over all seeds -- collapsed runs included, never
    # conditioned on survival -- paired with its spread. A mean alone cannot
    # separate "consistently mediocre" from "usually great, one blow-up", and
    # that distinction is the whole point of a robustness claim.
    for k in KPI_FIELDS:
        hdr += [k + "_mean", k + "_std"]
    ns = sorted({int(r["n_oht"]) for r in rows})
    with open(sum_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        for n in ns:
            for m in METHOD_ORDER:
                rs = groups.get((n, m))
                if not rs:
                    continue
                surv = surv_count(rs)
                glt = fnum(rs, "gridlock_time_h")
                out = [n, m, len(rs), surv, round(surv / len(rs) * 100.0, 1),
                       int(sum(fnum(rs, "gridlock_detected"))),
                       round(st.fmean(glt), 3) if glt else ""]
                for k in KPI_FIELDS:
                    v = fnum(rs, k)
                    out.append(round(st.fmean(v), 3) if v else "")
                    out.append(round(st.pstdev(v), 3) if len(v) > 1 else "")
                w.writerow(out)

    print("merged %d shards, %d runs" % (len(shards), len(rows)))
    print("  runs    : " + all_csv)
    print("  summary : " + sum_csv)

    # console robustness table -- the headline of this experiment
    print("\nSurvival rate (attainment >= %.0f%%)" % SURVIVAL_RATE_PCT)
    print("  " + "method".ljust(11) + "".join(("n%d" % n).ljust(9) for n in ns))
    for m in METHOD_ORDER:
        cells = []
        for n in ns:
            rs = groups.get((n, m))
            if not rs:
                cells.append("-".ljust(9))
            else:
                cells.append(("%2d/%-2d" % (surv_count(rs), len(rs))).ljust(9))
        print("  " + m.ljust(11) + "".join(cells))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="robustness sweep: 8 methods x fleet size x seeds",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.join("results", "sweep"),
                   help="single flat results directory (default: results/sweep)")
    p.add_argument("--n-oht", type=int, default=120, help="fleet size")
    p.add_argument("--seeds", default="1-20", help="seed spec, e.g. 1-20 or 1,2,3")
    p.add_argument("--methods", default="", help="comma-separated subset (default: all 8)")
    p.add_argument("--horizon", type=float, default=86400.0)
    p.add_argument("--merge", action="store_true",
                   help="merge existing shards in --out and exit (no simulation)")
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    if a.merge:
        merge(a.out)
        return

    seeds = parse_seeds(a.seeds)
    methods = METHODS
    if a.methods:
        want = {m.strip() for m in a.methods.split(",") if m.strip()}
        methods = [(n, f) for (n, f) in METHODS if n in want]
        if not methods:
            raise SystemExit("no methods matched " + str(sorted(want)))

    span = str(seeds[0]) if len(seeds) == 1 else "%d-%d" % (seeds[0], seeds[-1])
    tag = "n%d_s%s" % (a.n_oht, span)
    runs_csv = os.path.join(a.out, "runs_%s.csv" % tag)
    scratch = os.path.join(a.out, "_scratch", tag)   # per-shard, so shards never collide
    os.makedirs(scratch, exist_ok=True)

    total = len(methods) * len(seeds)
    print("sweep: %d runs  n_oht=%d  seeds=%s" % (total, a.n_oht, seeds))
    print("  methods : %s" % [n for n, _ in methods])
    print("  writing : %s\n" % runs_csv)

    t0 = time.time()
    done = 0
    with open(runs_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        f.flush()
        for name, configure in methods:
            for seed in seeds:
                done += 1
                rt0 = time.time()
                try:
                    row = run_one(name, configure, seed, a.n_oht, a.horizon, scratch)
                except Exception as exc:      # keep the shard alive on one failure
                    row = {"method": name, "n_oht": a.n_oht, "seed": seed,
                           "status": "ERROR: %s: %s" % (type(exc).__name__, exc)}
                    for k in KPI_FIELDS + GRIDLOCK_FIELDS:
                        row[k] = None
                    row["survived"] = None
                row["elapsed_s"] = round(time.time() - rt0, 1)
                w.writerow(row)
                f.flush()

                eta = (time.time() - t0) / done * (total - done)
                attain = row.get("makespan_attain_h")
                attain_s = ("%.2f" % attain) if isinstance(attain, (int, float)) else str(row["status"])
                gl = row.get("gridlock_time_h")
                gl_s = (" gridlock@%sh" % gl) if gl not in (None, "", 0) else ""
                print("[%d/%d] %-10s n=%d seed=%-3d attain_h=%-8s surv=%s%s (%.0fs, ETA %.1fm)"
                      % (done, total, name, a.n_oht, seed, attain_s, row.get("survived"),
                         gl_s, row["elapsed_s"], eta / 60), flush=True)

    print("\nshard done: %d runs in %.1f min -> %s" % (total, (time.time() - t0) / 60, runs_csv))


if __name__ == "__main__":
    main()
