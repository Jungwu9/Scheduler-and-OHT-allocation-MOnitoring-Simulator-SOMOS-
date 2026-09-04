"""Load simulation CSV files and prepare OHT replay data."""

from __future__ import annotations

import csv
import heapq
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


FILE_CANDIDATES = {
    "transport": (
        "transport.csv",
        "transport_live.csv",
        "oht_transport.csv",
        "transport_event_log.csv",
    ),
    "machine": ("log_machine_sim.csv", "actual_gantt.csv", "machine_sim_log.csv"),
    "planned": ("planned_gantt.csv", "gantt_planned.csv", "planned_schedule.csv"),
}

COLUMN_ALIASES = {
    "oht_id": ("oht_id", "vehicle_id", "OHT_ID"),
    "lot_id": ("lot_id", "job_id", "job_instance_id"),
    "job_instance_id": ("job_instance_id", "lot_instance_id", "instance_id"),
    "step_no": ("step_no", "op_index", "operation_index"),
    "from_machine": ("from_machine", "source_machine", "origin_machine"),
    "to_machine": ("to_machine", "destination_machine", "dest_machine"),
    "from_node": ("from_node", "source_node", "origin_node"),
    "to_node": ("to_node", "destination_node", "dest_node"),
    "dispatch_time": ("dispatch_time", "dispatch_sec"),
    "prev_machine_end_time": (
        "prev_machine_end_time",
        "previous_machine_end_time",
        "request_time",
    ),
    "source_arrival_time": ("source_arrival_time", "source_arrival_sec"),
    "pickup_time": ("pickup_time", "pick_up_time"),
    "load_start_time": ("load_start_time", "loading_start_time"),
    "load_end_time": ("load_end_time", "loading_end_time"),
    "loaded_travel_start_time": (
        "loaded_travel_start_time",
        "loaded_start_time",
    ),
    "dest_arrival_time": (
        "dest_arrival_time",
        "destination_arrival_time",
        "arrival_time",
    ),
    "unload_start_time": ("unload_start_time", "unloading_start_time"),
    "unload_end_time": ("unload_end_time", "unloading_end_time"),
    "dropoff_time": (
        "dropoff_time",
        "drop_off_time",
        "arrival_complete_time",
    ),
    "empty_to_source_time": ("empty_to_source_time", "empty_travel_time"),
    "source_wait_time": ("source_wait_time", "wait_at_source_time"),
    "loading_time": ("loading_time", "load_time"),
    "loaded_travel_time": ("loaded_travel_time", "loaded_time"),
    "unloading_time": ("unloading_time", "unload_time"),
    "drop_wait_time": ("drop_wait_time", "wait_at_destination_time"),
    "actual_transport_time": ("actual_transport_time", "transport_time"),
    "planned_travel": (
        "planned_travel",
        "planned_travel_sec",
        "planned_transport_time",
    ),
    "transport_deviation": (
        "transport_deviation",
        "transport_delay",
        "travel_deviation",
    ),
    "blocking_loaded_time": (
        "blocking_loaded_time",
        "loaded_blocking_time",
        "traffic_delay_time",
    ),
    "detour_ratio": ("detour_ratio", "path_detour_ratio"),
}

PLANNED_COLUMN_ALIASES = {
    "end": ("end_sec", "planned_end", "planned_finish", "end_time"),
}

ACTUAL_COLUMN_ALIASES = {
    "end": ("end_time", "actual_end", "actual_finish", "finish_time"),
    "lot": ("lot_id", "job_id", "job_instance_id"),
    "job_instance_id": ("job_instance_id", "lot_instance_id", "instance_id"),
    "job_id": ("lot_id", "job_id", "job_type_id"),
    "step_no": ("step_no", "op_index", "operation_index"),
    "machine": ("machine_no", "physical_machine_no", "machine_id", "machine", "jssp_mach_id"),
    "physical_machine": ("machine_no", "physical_machine_no"),
    "machine_name": ("node_name", "machine_name"),
    "start": ("start_time", "actual_start", "start_sec"),
    "product_type": ("product_type", "job_type_id"),
}

PLANNED_MACHINE_COLUMN_ALIASES = {
    "job_id": ("job_id", "lot_id"),
    "step_no": ("op_index", "step_no", "operation_index"),
    "machine": ("machine_id", "machine_no", "machine"),
    "start": ("start_time", "start_sec", "planned_start"),
    "end": ("end_time", "end_sec", "planned_end"),
}

PHASES = (
    ("empty_to_source", "dispatch_time", "source_arrival_time"),
    ("source_wait", "source_arrival_time", "pickup_time"),
    ("loading", "load_start_time", "load_end_time"),
    ("loaded_travel", "loaded_travel_start_time", "dest_arrival_time"),
    ("unloading", "unload_start_time", "unload_end_time"),
    ("drop_wait", "unload_end_time", "dropoff_time"),
)

SUMMARY_COLUMNS = (
    "oht_id",
    "n_transport_tasks",
    "total_empty_to_source_time",
    "total_source_wait_time",
    "total_loading_time",
    "total_loaded_travel_time",
    "total_unloading_time",
    "total_drop_wait_time",
    "total_blocking_loaded_time",
    "total_transport_deviation",
    "max_transport_deviation",
    "carried_cmax_lot",
)


SOMOS_DIR = Path(__file__).resolve().parents[1]


def _existing_dirs(*paths: Path) -> tuple[Path, ...]:
    seen: set[Path] = set()
    result: list[Path] = []
    for path in paths:
        resolved = path.expanduser().resolve()
        if resolved in seen or not resolved.is_dir():
            continue
        seen.add(resolved)
        result.append(resolved)
    return tuple(result)


@dataclass
class TraceData:
    transport_path: Path
    machine_path: Path
    planned_path: Path
    transport_rows: list[dict[str, Any]]
    machine_rows: list[dict[str, Any]]
    planned_machine_rows: list[dict[str, Any]]
    event_trace_rows: list[dict[str, Any]]
    layout: dict[str, Any]
    original_transport_headers: list[str]
    summary_rows: list[dict[str, Any]]
    selected_oht_ids: list[str]
    planned_cmax: float
    actual_cmax: float
    cmax_delay: float
    cmax_lot: str
    dispatch_mode: str

    @property
    def max_time(self) -> float:
        trace_max = max(
            (
                max(_number(row.get("sim_time")), _number(row.get("end_time")))
                for row in self.event_trace_rows
            ),
            default=0.0,
        )
        return max(self.planned_cmax, self.actual_cmax, trace_max, 0.0)


def _normalize_event_trace(
    rows: list[dict[str, str]],
) -> list[dict[str, Any]]:
    numeric = {
        "seq", "sim_time", "end_time", "x", "y", "x1", "y1",
        "wait_duration", "total_cost", "empty_cost", "loaded_cost",
    }
    result = []
    for raw in rows:
        row = {}
        for key, value in raw.items():
            if key in numeric:
                row[key] = _number(value) if str(value).strip() else None
            else:
                row[key] = _text(value)
        result.append(row)
    result.sort(key=lambda row: (
        _number(row.get("sim_time")),
        _number(row.get("seq")),
    ))
    return result


def _read_dispatch_mode(output_path: Path, event_trace_path: Path) -> str:
    if event_trace_path.is_file():
        with event_trace_path.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                mode = _text(row.get("dispatch_mode"))
                if mode:
                    return mode

    kpi_path = output_path / "kpi.csv"
    if kpi_path.is_file():
        with kpi_path.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                mode = _text(row.get("dispatch_mode"))
                if mode:
                    return mode
    return ""


def _read_selected_event_trace(
    path: Path,
    selected_oht_ids: Iterable[str],
) -> list[dict[str, Any]]:
    selected = {str(oht_id) for oht_id in selected_oht_ids}
    all_oht_motion_events = {
        "INIT", "EDGE_START", "BLOCK_START", "BLOCK_END", "STATE_CHANGE",
    }
    selected_detail_events = {
        "INIT", "STATE_CHANGE",
        "EDGE_START", "EDGE_END", "BLOCK_START", "BLOCK_END",
        "DISPATCH", "REASSIGN", "TASK_ASSIGNED",
        "SERVICE_WAIT_START", "SERVICE_WAIT_END",
        "PICKUP", "LOAD_START", "LOAD_END",
        "UNLOAD_START", "UNLOAD_END", "DROPOFF", "DROPOFF_FAILED",
        "TASK_TIMEOUT", "TRANSPORT_COMPLETE",
        "ZCU_RESERVE", "ZCU_RELEASE",
    }
    numeric = {
        "seq", "sim_time", "end_time", "x", "y", "x1", "y1",
        "wait_duration", "total_cost", "empty_cost", "loaded_cost",
    }
    def convert(key: str, value: Any) -> Any:
        if key in numeric:
            return _number(value) if str(value).strip() else None
        return _text(value)

    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            oht_id = _text(raw.get("oht_id"))
            event = _text(raw.get("event"))
            is_selected = oht_id in selected
            if event not in all_oht_motion_events and not (
                is_selected and event in selected_detail_events
            ):
                continue
            row = {
                key: convert(key, value)
                for key, value in raw.items()
                if is_selected
                or key in {
                    "seq", "sim_time", "end_time", "oht_id", "event",
                    "state", "old_state", "new_state", "reason",
                    "from_node", "to_node", "x", "y", "x1", "y1",
                }
            }
            row["route"] = ""
            rows.append(row)
    rows.sort(key=lambda row: (
        _number(row.get("sim_time")),
        _number(row.get("seq")),
    ))
    return rows


def find_input_file(
    output_dir: Path,
    kind: str,
    additional_dirs: Iterable[Path] = (),
) -> Path:
    search_dirs = [output_dir, *additional_dirs]
    for search_dir in search_dirs:
        for name in FILE_CANDIDATES[kind]:
            path = search_dir / name
            if path.is_file():
                return path
    names = ", ".join(FILE_CANDIDATES[kind])
    locations = ", ".join(f"'{path}'" for path in search_dirs)
    raise FileNotFoundError(
        f"Missing {kind} CSV. Searched directories: {locations}. "
        f"Tried filenames: {names}"
    )


def _read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"CSV has no header: {path}")
        return list(reader), list(reader.fieldnames)


def _resolve_columns(
    headers: Iterable[str],
    aliases: dict[str, Iterable[str]],
    required: Iterable[str],
    path: Path,
) -> dict[str, str | None]:
    headers = list(headers)
    exact = {header: header for header in headers}
    folded = {header.casefold(): header for header in headers}
    resolved: dict[str, str | None] = {}
    for canonical, candidates in aliases.items():
        resolved[canonical] = next(
            (
                exact.get(candidate) or folded.get(candidate.casefold())
                for candidate in candidates
                if exact.get(candidate) or folded.get(candidate.casefold())
            ),
            None,
        )
    missing = [name for name in required if not resolved.get(name)]
    if missing:
        available = ", ".join(headers)
        raise ValueError(
            f"Missing required columns in '{path}': {', '.join(missing)}. "
            f"Available columns: {available}"
        )
    return resolved


def _number(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
        return number if math.isfinite(number) else default
    except (TypeError, ValueError):
        return default


def _text(value: Any) -> str:
    if value is None:
        return ""
    text = str(value).strip()
    if text.endswith(".0"):
        try:
            return str(int(float(text)))
        except ValueError:
            pass
    return text


def _machine_number(value: Any) -> str:
    text = _text(value)
    match = re.search(r"\d+", text)
    return str(int(match.group(0))) if match else text


def _value(row: dict[str, str], mapping: dict[str, str | None], name: str) -> str:
    source = mapping.get(name)
    return row.get(source, "") if source else ""


def _same_lot(left: Any, right: Any) -> bool:
    a, b = _text(left), _text(right)
    if not a or not b:
        return False
    if a.casefold() == b.casefold():
        return True
    digits_a = "".join(character for character in a if character.isdigit())
    digits_b = "".join(character for character in b if character.isdigit())
    return bool(digits_a and digits_b and int(digits_a) == int(digits_b))


def _instance_from_lot(lot_id: Any, fallback: Any = "") -> str:
    fallback_text = _text(fallback)
    if fallback_text:
        return fallback_text
    digits = "".join(character for character in _text(lot_id) if character.isdigit())
    return str(int(digits)) if digits else ""


def _task_key(lot_id: Any, job_instance_id: Any, step_no: Any) -> str:
    step = _text(step_no)
    lot = _text(lot_id)
    if lot and step:
        return f"lot:{lot}|step:{step}"
    instance = _instance_from_lot(lot_id, job_instance_id)
    if instance and step:
        return f"inst:{instance}|step:{step}"
    return ""


def _instance_step_key(lot_id: Any, job_instance_id: Any, step_no: Any) -> str:
    instance = _instance_from_lot(lot_id, job_instance_id)
    step = _text(step_no)
    return f"inst:{instance}|step:{step}" if instance and step else ""


def _normalize_transport(
    rows: list[dict[str, str]],
    headers: list[str],
    path: Path,
) -> list[dict[str, Any]]:
    mapping = _resolve_columns(
        headers,
        COLUMN_ALIASES,
        ("oht_id", "dispatch_time", "dropoff_time", "prev_machine_end_time", "pickup_time"),
        path,
    )
    numeric_fields = {
        "dispatch_time",
        "prev_machine_end_time",
        "source_arrival_time",
        "pickup_time",
        "load_start_time",
        "load_end_time",
        "loaded_travel_start_time",
        "dest_arrival_time",
        "unload_start_time",
        "unload_end_time",
        "dropoff_time",
        "empty_to_source_time",
        "source_wait_time",
        "loading_time",
        "loaded_travel_time",
        "unloading_time",
        "drop_wait_time",
        "actual_transport_time",
        "planned_travel",
        "transport_deviation",
        "blocking_loaded_time",
        "detour_ratio",
    }
    normalized = []
    for index, source_row in enumerate(rows, start=1):
        row: dict[str, Any] = {"_original": source_row, "_row_number": index}
        for name in COLUMN_ALIASES:
            raw = _value(source_row, mapping, name)
            row[name] = _number(raw) if name in numeric_fields else _text(raw)

        row["oht_id"] = _text(row["oht_id"])
        row["lot_id"] = _text(row["lot_id"])
        row["job_instance_id"] = _instance_from_lot(
            row["lot_id"], row["job_instance_id"]
        )
        row["step_no"] = _text(row["step_no"])
        row["task_key"] = _task_key(
            row["lot_id"], row["job_instance_id"], row["step_no"]
        )
        row["instance_step_key"] = _instance_step_key(
            row["lot_id"], row["job_instance_id"], row["step_no"]
        )
        row["lot_step_key"] = f"{row['lot_id']}|{row['step_no']}"
        if not mapping.get("planned_travel"):
            row["planned_travel"] = 0.0
        row["planned_transport_finish"] = (
            row["prev_machine_end_time"] + row["planned_travel"]
        )
        if not mapping.get("transport_deviation"):
            row["transport_deviation"] = (
                row["dropoff_time"] - row["planned_transport_finish"]
                if mapping.get("planned_travel")
                else 0.0
            )

        phases = []
        for phase_name, start_name, end_name in PHASES:
            if not mapping.get(start_name) or not mapping.get(end_name):
                continue
            start, end = row[start_name], row[end_name]
            if end >= start:
                phases.append(
                    {
                        "name": phase_name,
                        "start": start,
                        "end": end,
                        "duration": end - start,
                    }
                )
        row["phases"] = phases
        normalized.append(row)
    return normalized


def _calculate_cmax(
    planned_rows: list[dict[str, str]],
    planned_headers: list[str],
    planned_path: Path,
    actual_rows: list[dict[str, str]],
    actual_headers: list[str],
    actual_path: Path,
) -> tuple[float, float, str]:
    planned_map = _resolve_columns(
        planned_headers, PLANNED_COLUMN_ALIASES, ("end",), planned_path
    )
    actual_map = _resolve_columns(
        actual_headers, ACTUAL_COLUMN_ALIASES, ("end", "lot"), actual_path
    )
    planned_cmax = max(
        (_number(row[planned_map["end"]]) for row in planned_rows), default=0.0
    )
    actual_max_row = max(
        actual_rows,
        key=lambda row: _number(row[actual_map["end"]]),
        default=None,
    )
    actual_cmax = (
        _number(actual_max_row[actual_map["end"]]) if actual_max_row else 0.0
    )
    cmax_lot = _text(actual_max_row[actual_map["lot"]]) if actual_max_row else ""
    return planned_cmax, actual_cmax, cmax_lot


def _normalize_machine_rows(
    rows: list[dict[str, str]],
    headers: list[str],
    path: Path,
) -> list[dict[str, Any]]:
    mapping = _resolve_columns(
        headers,
        ACTUAL_COLUMN_ALIASES,
        ("job_instance_id", "step_no", "machine", "start", "end"),
        path,
    )
    normalized = []
    for index, source_row in enumerate(rows, start=1):
        start = _number(_value(source_row, mapping, "start"))
        end = _number(_value(source_row, mapping, "end"))
        lot_id = _text(_value(source_row, mapping, "lot"))
        job_instance_id = _instance_from_lot(
            lot_id, _value(source_row, mapping, "job_instance_id")
        )
        step_no = _text(_value(source_row, mapping, "step_no"))
        normalized.append(
            {
                "_row_number": index,
                "job_instance_id": job_instance_id,
                "job_id": _text(_value(source_row, mapping, "job_id")),
                "lot_id": lot_id,
                "step_no": step_no,
                "task_key": _task_key(lot_id, job_instance_id, step_no),
                "instance_step_key": _instance_step_key(
                    lot_id, job_instance_id, step_no
                ),
                "lot_step_key": f"{lot_id}|{step_no}",
                "machine": _machine_number(_value(source_row, mapping, "machine")),
                "physical_machine": _text(
                    _value(source_row, mapping, "physical_machine")
                ),
                "machine_name": _text(_value(source_row, mapping, "machine_name")),
                "product_type": _text(_value(source_row, mapping, "product_type")),
                "start": start,
                "end": end,
                "duration": max(0.0, end - start),
            }
        )
    return normalized


def _normalize_planned_machine_rows(
    rows: list[dict[str, str]],
    headers: list[str],
    path: Path,
) -> list[dict[str, Any]]:
    mapping = _resolve_columns(
        headers,
        PLANNED_MACHINE_COLUMN_ALIASES,
        ("job_id", "step_no", "machine", "start", "end"),
        path,
    )
    normalized = []
    for index, source_row in enumerate(rows, start=1):
        start = _number(_value(source_row, mapping, "start"))
        end = _number(_value(source_row, mapping, "end"))
        lot_id = _text(_value(source_row, mapping, "job_id"))
        lot_digits = "".join(character for character in lot_id if character.isdigit())
        normalized.append(
            {
                "_row_number": index,
                "job_id": lot_id,
                "lot_id": lot_id,
                "job_instance_id": str(int(lot_digits)) if lot_digits else "",
                "step_no": _text(_value(source_row, mapping, "step_no")),
                "machine": _machine_number(_value(source_row, mapping, "machine")),
                "start": start,
                "end": end,
                "duration": max(0.0, end - start),
            }
        )
        normalized[-1]["task_key"] = _task_key(
            normalized[-1]["lot_id"],
            normalized[-1]["job_instance_id"],
            normalized[-1]["step_no"],
        )
        normalized[-1]["instance_step_key"] = _instance_step_key(
            normalized[-1]["lot_id"],
            normalized[-1]["job_instance_id"],
            normalized[-1]["step_no"],
        )
        normalized[-1]["lot_step_key"] = (
            f"{normalized[-1]['lot_id']}|{normalized[-1]['step_no']}"
        )
    return normalized


_TILE_CORNERS = {
    "1": ("NW", "NE"), "2": ("SW", "NW"), "3": ("NE", "SE"),
    "4": ("SE", "SW"), "5": ("NW", "SE"), "6": ("SE", "NW"),
    "7": ("NE", "SW"), "8": ("SW", "NE"), "9": ("NW", "SE"),
    "10": ("SE", "NW"), "11": ("SW", "NE"), "12": ("NE", "SW"),
}
_CORNER_DV = {"NW": (0, 0), "NE": (0, 1), "SW": (1, 0), "SE": (1, 1)}
_ARC_CP = {
    "5": "NE", "6": "NE", "7": "NW", "8": "NW",
    "9": "SW", "10": "SW", "11": "SE", "12": "SE",
}
_JUNCTIONS = {
    "13": ("NW", "NE", "SE", "NE", "fork"),
    "14": ("NW", "NE", "SE", "NE", "merge"),
    "15": ("NE", "NW", "SW", "NW", "fork"),
    "16": ("NE", "NW", "SW", "NW", "merge"),
    "17": ("SE", "SW", "NW", "SW", "merge"),
    "18": ("SE", "SW", "NW", "SW", "fork"),
    "19": ("SW", "SE", "NE", "SE", "fork"),
    "20": ("SW", "SE", "NE", "SE", "merge"),
}


def _read_grid(path: Path) -> list[list[str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = [[cell.strip() for cell in row] for row in csv.reader(handle)]
    width = max((len(row) for row in rows), default=0)
    return [row + [""] * (width - len(row)) for row in rows]


def _base_code(value: str) -> str:
    match = re.match(r"^\s*(\d+)", value)
    return match.group(1) if match else ""


def build_layout_data(repository_dir: Path) -> dict[str, Any]:
    layout_path = repository_dir / "layout_oht.csv"
    machine_path = repository_dir / "layout_machine.csv"
    if not layout_path.is_file():
        return {"nodes": {}, "edges": [], "machines": [], "bounds": [0, 0, 1, 1]}

    grid = _read_grid(layout_path)
    machine_grid = _read_grid(machine_path) if machine_path.is_file() else []
    rows, cols, cell_size = len(grid), len(grid[0]) if grid else 0, 10.0

    def span(value: str) -> float:
        return {"#": 1.10, "##": 0.68, "###": 0.05}.get(
            value, 0.22 if not value else 1.0
        ) * cell_size

    col_widths = [
        max(span(grid[r][c]) for r in range(rows)) for c in range(cols)
    ]
    row_heights = [
        max(span(grid[r][c]) for c in range(cols)) for r in range(rows)
    ]
    x_edges, y_edges = [0.0], [0.0]
    for width in col_widths:
        x_edges.append(x_edges[-1] + width)
    for height in row_heights:
        y_edges.append(y_edges[-1] + height)

    nodes: dict[str, tuple[float, float]] = {}
    edges: list[tuple[str, str]] = []

    def vertex(r: int, c: int) -> str:
        name = f"V_{r}_{c}"
        nodes.setdefault(name, (x_edges[c], -y_edges[r]))
        return name

    def corner(r: int, c: int, name: str) -> str:
        dr, dc = _CORNER_DV[name]
        return vertex(r + dr, c + dc)

    def corner_xy(r: int, c: int, name: str) -> tuple[float, float]:
        dr, dc = _CORNER_DV[name]
        return x_edges[c + dc], -y_edges[r + dr]

    def add_curve(start: str, end: str, control: tuple[float, float]) -> None:
        previous = start
        x0, y0 = nodes[start]
        x2, y2 = nodes[end]
        for index in range(1, 4):
            t, mt = index / 4, 1 - index / 4
            name = f"ARC_{len(nodes):06d}"
            nodes[name] = (
                mt * mt * x0 + 2 * mt * t * control[0] + t * t * x2,
                mt * mt * y0 + 2 * mt * t * control[1] + t * t * y2,
            )
            edges.append((previous, name))
            previous = name
        edges.append((previous, end))

    for r, row in enumerate(grid):
        for c, raw in enumerate(row):
            code = _base_code(raw)
            if code in _TILE_CORNERS:
                start_corner, end_corner = _TILE_CORNERS[code]
                start, end = corner(r, c, start_corner), corner(r, c, end_corner)
                if code in {"1", "2", "3", "4"}:
                    edges.append((start, end))
                else:
                    add_curve(start, end, corner_xy(r, c, _ARC_CP[code]))
            elif code in _JUNCTIONS:
                shared, straight, arc, control, mode = _JUNCTIONS[code]
                shared_node = corner(r, c, shared)
                straight_node = corner(r, c, straight)
                arc_node = corner(r, c, arc)
                if mode == "fork":
                    edges.append((shared_node, straight_node))
                    add_curve(
                        shared_node,
                        arc_node,
                        corner_xy(r, c, control),
                    )
                else:
                    edges.append((straight_node, shared_node))
                    add_curve(
                        arc_node,
                        shared_node,
                        corner_xy(r, c, control),
                    )

    machines = []
    for r, row in enumerate(machine_grid[:rows]):
        for c, raw in enumerate(row[:cols]):
            match = re.match(r"^M_(\d+)$", raw, re.IGNORECASE)
            code = _base_code(grid[r][c])
            if not match or code not in _TILE_CORNERS:
                continue
            node_name = corner(r, c, _TILE_CORNERS[code][0])
            x, y = nodes[node_name]
            direction = int(code)
            dx, dy = {
                1: (0, cell_size), 4: (0, -cell_size),
                2: (cell_size, 0), 3: (-cell_size, 0),
            }.get(direction, (0, cell_size))
            machines.append(
                {
                    "machine": match.group(1),
                    "node": node_name,
                    "x": x + dx,
                    "y": y + dy,
                }
            )

    adjacency: dict[str, list[tuple[str, float]]] = defaultdict(list)
    for left, right in edges:
        distance = math.dist(nodes[left], nodes[right])
        adjacency[left].append((right, distance))

    return {
        "nodes": {name: {"x": xy[0], "y": xy[1]} for name, xy in nodes.items()},
        "edges": [{"from": left, "to": right} for left, right in edges],
        "machines": machines,
        "bounds": [0.0, -y_edges[-1], x_edges[-1], 0.0],
        "_adjacency": adjacency,
    }


def _shortest_layout_path(
    layout: dict[str, Any], start: str, end: str
) -> list[str]:
    nodes = layout["nodes"]
    adjacency = layout.get("_adjacency", {})
    if start not in nodes or end not in nodes:
        return [name for name in (start, end) if name in nodes]
    queue = [(0.0, start)]
    distance = {start: 0.0}
    previous: dict[str, str] = {}
    while queue:
        cost, node = heapq.heappop(queue)
        if node == end:
            break
        if cost != distance.get(node):
            continue
        for neighbor, edge_cost in adjacency.get(node, []):
            candidate = cost + edge_cost
            if candidate < distance.get(neighbor, float("inf")):
                distance[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    if end not in distance:
        return [start, end]
    path = [end]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def add_oht_routes(
    transport_rows: list[dict[str, Any]], layout: dict[str, Any]
) -> None:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        grouped[row["oht_id"]].append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: row["dispatch_time"])
        previous_node = ""
        for row in rows:
            source, destination = row["from_node"], row["to_node"]
            empty_start = previous_node or source
            row["empty_route"] = _shortest_layout_path(
                layout, empty_start, source
            )
            row["loaded_route"] = _shortest_layout_path(
                layout, source, destination
            )
            previous_node = destination


def build_summary(
    transport_rows: list[dict[str, Any]], cmax_lot: str
) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        grouped[row["oht_id"]].append(row)

    summaries = []
    for oht_id, rows in grouped.items():
        deviation_values = [row["transport_deviation"] for row in rows]
        summaries.append(
            {
                "oht_id": oht_id,
                "n_transport_tasks": len(rows),
                "total_empty_to_source_time": sum(
                    row["empty_to_source_time"] for row in rows
                ),
                "total_source_wait_time": sum(row["source_wait_time"] for row in rows),
                "total_loading_time": sum(row["loading_time"] for row in rows),
                "total_loaded_travel_time": sum(
                    row["loaded_travel_time"] for row in rows
                ),
                "total_unloading_time": sum(row["unloading_time"] for row in rows),
                "total_drop_wait_time": sum(row["drop_wait_time"] for row in rows),
                "total_blocking_loaded_time": sum(
                    row["blocking_loaded_time"] for row in rows
                ),
                "total_transport_deviation": sum(deviation_values),
                "max_transport_deviation": max(deviation_values, default=0.0),
                "carried_cmax_lot": any(
                    _same_lot(row["lot_id"], cmax_lot) for row in rows
                ),
            }
        )
    return sorted(summaries, key=lambda row: _sort_id(row["oht_id"]))


def add_event_trace_oht_summaries(
    summaries: list[dict[str, Any]],
    event_trace_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    known = {_text(row.get("oht_id")) for row in summaries}
    for event in event_trace_rows:
        oht_id = _text(event.get("oht_id"))
        if not oht_id or oht_id in known:
            continue
        known.add(oht_id)
        summaries.append(
            {
                "oht_id": oht_id,
                "n_transport_tasks": 0,
                "total_empty_to_source_time": 0.0,
                "total_source_wait_time": 0.0,
                "total_loading_time": 0.0,
                "total_loaded_travel_time": 0.0,
                "total_unloading_time": 0.0,
                "total_drop_wait_time": 0.0,
                "total_blocking_loaded_time": 0.0,
                "total_transport_deviation": 0.0,
                "max_transport_deviation": 0.0,
                "carried_cmax_lot": False,
            }
        )
    return sorted(summaries, key=lambda row: _sort_id(row["oht_id"]))


def _sort_id(value: str) -> tuple[int, Any]:
    try:
        return (0, float(value))
    except ValueError:
        return (1, value.casefold())


def select_oht_ids(
    summaries: list[dict[str, Any]],
    requested_ids: list[str] | None,
    top_k: int,
) -> list[str]:
    available = {row["oht_id"] for row in summaries}
    if requested_ids:
        selected = [_text(value) for value in requested_ids if _text(value) in available]
        missing = [_text(value) for value in requested_ids if _text(value) not in available]
        if missing:
            raise ValueError(
                f"Requested OHT IDs not found: {', '.join(missing)}. "
                f"Available IDs: {', '.join(sorted(available, key=_sort_id))}"
            )
        return selected

    ranked = sorted(
        summaries,
        key=lambda row: (
            row["total_transport_deviation"] + row["total_blocking_loaded_time"],
            row["total_transport_deviation"],
            row["total_blocking_loaded_time"],
        ),
        reverse=True,
    )
    return [row["oht_id"] for row in ranked[: max(top_k, 0)]]


def load_trace_data(
    output_dir: str | Path,
    selected_oht_ids: list[str] | None = None,
    top_k: int = 5,
) -> TraceData:
    output_path = Path(output_dir).expanduser().resolve()
    planned_dirs = _existing_dirs(
        output_path,
        output_path.parent / "gantt_final",
        output_path.parent.parent / "gantt_final",
        output_path.parent.parent.parent / "gantt_final",
        SOMOS_DIR / "gantt_final",
    )
    transport_path = find_input_file(output_path, "transport")
    machine_path = find_input_file(output_path, "machine")
    planned_path = find_input_file(
        output_path,
        "planned",
        additional_dirs=planned_dirs,
    )

    raw_transport, transport_headers = _read_csv(transport_path)
    raw_machine, machine_headers = _read_csv(machine_path)
    raw_planned, planned_headers = _read_csv(planned_path)
    event_trace_path = output_path / "oht_event_trace.csv"
    transport_rows = _normalize_transport(
        raw_transport, transport_headers, transport_path
    )
    machine_rows = _normalize_machine_rows(
        raw_machine, machine_headers, machine_path
    )
    planned_machine_rows = _normalize_planned_machine_rows(
        raw_planned,
        planned_headers,
        planned_path,
    )
    layout_dirs = _existing_dirs(
        output_path.parent,
        output_path.parent.parent,
        output_path.parent.parent.parent,
        SOMOS_DIR,
    )
    layout = next(
        (
            build_layout_data(layout_dir)
            for layout_dir in layout_dirs
            if (layout_dir / "layout_oht.csv").is_file()
        ),
        {"nodes": {}, "edges": [], "machines": [], "bounds": [0, 0, 1, 1]},
    )
    add_oht_routes(transport_rows, layout)
    layout.pop("_adjacency", None)
    planned_cmax, actual_cmax, cmax_lot = _calculate_cmax(
        raw_planned,
        planned_headers,
        planned_path,
        raw_machine,
        machine_headers,
        machine_path,
    )
    summaries = build_summary(transport_rows, cmax_lot)
    if event_trace_path.is_file():
        preview_event_trace_rows = _read_selected_event_trace(event_trace_path, [])
        summaries = add_event_trace_oht_summaries(summaries, preview_event_trace_rows)
    else:
        preview_event_trace_rows = []
    selected = select_oht_ids(summaries, selected_oht_ids, top_k)
    if event_trace_path.is_file():
        event_trace_rows = _read_selected_event_trace(event_trace_path, selected)
    else:
        event_trace_rows = preview_event_trace_rows
    dispatch_mode = _read_dispatch_mode(output_path, event_trace_path)
    return TraceData(
        transport_path=transport_path,
        machine_path=machine_path,
        planned_path=planned_path,
        transport_rows=transport_rows,
        machine_rows=machine_rows,
        planned_machine_rows=planned_machine_rows,
        event_trace_rows=event_trace_rows,
        layout=layout,
        original_transport_headers=transport_headers,
        summary_rows=summaries,
        selected_oht_ids=selected,
        planned_cmax=planned_cmax,
        actual_cmax=actual_cmax,
        cmax_delay=actual_cmax - planned_cmax,
        cmax_lot=cmax_lot,
        dispatch_mode=dispatch_mode,
    )


def write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row[column] for column in SUMMARY_COLUMNS})


def write_selected_csv(
    path: Path,
    rows: list[dict[str, Any]],
    headers: list[str],
    selected_oht_ids: list[str],
) -> None:
    selected = set(selected_oht_ids)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            if row["oht_id"] in selected:
                writer.writerow(row["_original"])
