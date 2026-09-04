"""main_exp_merge.py -- merge shard results from a split main_exp_run.py batch.

When the 140-run experiment is split across several terminals (one seed range
per terminal, each with its own --out), every shard writes its own runs.csv.
This script concatenates them and rebuilds the single aggregated summary /
console table exactly as an unsplit run would have produced.

    python main_exp_merge.py results/main_exp/p1 results/main_exp/p2 ...
    python main_exp_merge.py results/main_exp/p*          # shell glob
    python main_exp_merge.py --out results/main_exp results/main_exp/p1 ...

Shard dirs may be given as the directory (runs.csv is appended) or as the
csv path itself.
"""
import argparse
import csv
import glob
import os

from main_exp_run import KPI_FIELDS, write_summary, print_console_summary


NUMERIC = set(KPI_FIELDS) | {"elapsed_s"}


def _load(path):
    """Read one shard runs.csv back into row dicts with numeric KPIs restored."""
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            row["seed"] = int(row["seed"])
            sv = row.get("survived")
            row["survived"] = int(sv) if sv not in (None, "", "None") else None
            for k in NUMERIC:
                v = row.get(k)
                row[k] = float(v) if v not in (None, "", "None") else None
            yield row


def _shard_paths(specs):
    paths = []
    for spec in specs:
        for p in sorted(glob.glob(spec)) or [spec]:
            p = os.path.join(p, "runs.csv") if os.path.isdir(p) else p
            if not os.path.exists(p):
                raise SystemExit(f"missing: {p}")
            paths.append(p)
    return paths


def main(argv=None):
    p = argparse.ArgumentParser(description="merge split main_exp_run.py shards")
    p.add_argument("shards", nargs="+", help="shard dirs (or runs.csv paths)")
    p.add_argument("--out", default=os.path.join("results", "main_exp"),
                   help="where to write merged runs.csv/summary.csv")
    a = p.parse_args(argv)

    rows = []
    for path in _shard_paths(a.shards):
        shard = list(_load(path))
        rows.extend(shard)
        print(f"  {path}: {len(shard)} rows")

    # a (method, seed) cell run twice by overlapping shards is a bug,
    # not something to silently average over
    seen = {}
    for r in rows:
        key = (r["method"], r["seed"])
        seen[key] = seen.get(key, 0) + 1
    dups = sorted(k for k, n in seen.items() if n > 1)
    if dups:
        print(f"\nWARNING: {len(dups)} duplicated cells (overlapping seed ranges?):")
        for k in dups[:10]:
            print(f"    {k}")

    os.makedirs(a.out, exist_ok=True)
    merged_csv = os.path.join(a.out, "runs_merged.csv")
    summary_csv = os.path.join(a.out, "summary.csv")

    fieldnames = ["method", "seed", "status"] + KPI_FIELDS + ["survived", "elapsed_s"]
    with open(merged_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r["method"], r["seed"])):
            w.writerow(r)

    groups = write_summary(rows, summary_csv)
    print_console_summary(groups)

    n_err = sum(1 for r in rows if r.get("status") != "ok")
    print(f"\nMerged {len(rows)} runs ({n_err} errors), "
          f"{len(seen)} unique cells of an expected 140")
    print(f"  per-run : {merged_csv}")
    print(f"  summary : {summary_csv}")


if __name__ == "__main__":
    main()
