"""Command-line and Spyder entry point for the OHT trace UI exporter."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from oht_trace_data import (
    load_trace_data,
    write_selected_csv,
    write_summary_csv,
)
from oht_trace_html_exporter import export_html


# Spyder settings. Running this file without arguments uses these values.
OUTPUT_DIR = "output"
RESULT_DIR = "oht_trace_ui_V3/output"
DEFAULT_SELECTED_OHT_IDS = None
TOP_K_OHT = 5


def _parse_selected_oht(value: str | None) -> list[str] | None:
    if not value:
        return None
    return [item.strip() for item in value.split(",") if item.strip()]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create a standalone OHT trace replay UI from simulation CSV files."
    )
    parser.add_argument("--output-dir", default=OUTPUT_DIR, help="Simulation output directory")
    parser.add_argument(
        "--result-dir",
        default=None,
        help="Result directory (default: <output-dir>/oht_trace_ui)",
    )
    parser.add_argument(
        "--selected-oht",
        default=None,
        help="Comma-separated OHT IDs, for example: 1,3,7,12",
    )
    parser.add_argument("--top-k", type=int, default=TOP_K_OHT)
    return parser


def report_event_trace_mode(output_path: Path) -> Path | None:
    trace_path = output_path / "oht_event_trace.csv"
    if trace_path.is_file():
        print(
            f"main OHT event trace enabled: {trace_path}"
        )
        return trace_path
    print(
        f"WARNING: exact OHT trace is missing: {trace_path}. "
        "Main will not be executed. The UI will use transport.csv route reconstruction."
    )
    return None


def run(
    output_dir: str,
    result_dir: str | None,
    selected_oht_ids: list[str] | None,
    top_k: int,
) -> tuple[Path, Path, Path]:
    output_path = Path(output_dir).expanduser().resolve()
    report_event_trace_mode(output_path)
    if result_dir:
        result_path = Path(result_dir).expanduser().resolve()
    elif output_dir == OUTPUT_DIR and RESULT_DIR:
        result_path = Path(RESULT_DIR).expanduser().resolve()
    else:
        result_path = output_path / "oht_trace_ui"

    data = load_trace_data(output_path, selected_oht_ids, top_k)
    result_path.mkdir(parents=True, exist_ok=True)
    html_path = result_path / "oht_trace_ui.html"
    summary_path = result_path / "oht_trace_summary.csv"
    selected_path = result_path / "oht_trace_selected.csv"
    export_html(data, html_path)
    write_summary_csv(summary_path, data.summary_rows)
    write_selected_csv(
        selected_path,
        data.transport_rows,
        data.original_transport_headers,
        data.selected_oht_ids,
    )

    print(f"input transport file path: {data.transport_path}")
    print(f"input machine log file path: {data.machine_path}")
    print(f"input planned gantt file path: {data.planned_path}")
    print(f"planned_Cmax: {data.planned_cmax}")
    print(f"actual_Cmax: {data.actual_cmax}")
    print(f"Cmax_delay: {data.cmax_delay}")
    print(f"cmax_lot: {data.cmax_lot}")
    print(f"selected_oht_ids: {','.join(data.selected_oht_ids)}")
    print(f"oht_trace_ui.html path: {html_path}")
    print(f"oht_trace_summary.csv path: {summary_path}")
    print(f"oht_trace_selected.csv path: {selected_path}")
    return html_path, summary_path, selected_path


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cli_selected = _parse_selected_oht(args.selected_oht)
    selected = cli_selected if cli_selected is not None else DEFAULT_SELECTED_OHT_IDS
    try:
        run(args.output_dir, args.result_dir, selected, args.top_k)
    except (FileNotFoundError, ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
