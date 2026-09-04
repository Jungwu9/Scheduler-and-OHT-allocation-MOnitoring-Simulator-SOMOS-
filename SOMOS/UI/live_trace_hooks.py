from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Any


TRANSPORT_FIELDS = [
    "event_id", "oht_id", "job_id", "job_instance_id", "op_index",
    "from_machine", "to_machine", "from_node", "to_node",
    "dispatch_time", "prev_machine_end_time", "source_arrival_time",
    "pickup_wait_start_time", "pickup_time",
    "load_start_time", "load_end_time",
    "loaded_travel_start_time", "dest_arrival_time",
    "unload_start_time", "unload_end_time", "dropoff_time",
    "source_wait_time", "empty_to_source_time", "pickup_wait_time",
    "loading_time", "loaded_travel_time", "unloading_time", "drop_wait_time",
    "pure_oht_transport_time", "actual_transport_time",
    "free_flow_shortest_time", "free_flow_actual_path_time",
    "blocking_loaded_time", "blocking_ratio", "detour_ratio",
    "lot_id", "step_no", "planned_travel", "transport_deviation",
]

MACHINE_LIVE_FIELDS = [
    "sim_time", "node_name", "machine_no", "machine_name", "jssp_mach_id",
    "job_id", "job_instance_id", "lot_id", "step_no", "job_type_id",
    "op_index", "product_type", "start_time", "end_time", "process_time",
    "planned_ready_time", "realized_ready_time", "ready_deviation",
]

OHT_EVENT_TRACE_FIELDS = [
    "seq", "sim_time", "end_time", "oht_id", "event", "state", "reason",
    "old_state", "new_state", "phase", "route", "wait_duration", "details",
    "from_node", "to_node", "x", "y", "x1", "y1", "job_id",
    "job_instance_id", "op_index", "from_machine", "to_machine",
    "dispatch_mode", "total_cost", "empty_cost", "loaded_cost",
]


def create_live_states(output_dir: str | os.PathLike[str]) -> dict[str, dict[str, Any]]:
    out_dir = Path(output_dir)
    return {
        "transport": {
            "path": str(out_dir / "transport_live.csv"),
            "file": None,
            "writer": None,
        },
        "machine": {
            "path": str(out_dir / "log_machine_sim.csv"),
            "file": None,
            "writer": None,
            "fields": MACHINE_LIVE_FIELDS,
        },
        "oht_event": {
            "path": str(out_dir / "oht_event_trace.csv"),
            "file": None,
            "writer": None,
            "seq": 0,
        },
    }


def _ensure_writer(state: dict[str, Any], fields: list[str]) -> None:
    if state.get("writer") is not None:
        return
    path = state.get("path")
    if not path:
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    handle = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    state["file"] = handle
    state["writer"] = writer


def append_transport_event(
    transport_event_log: list[dict[str, Any]],
    live_state: dict[str, Any] | None,
    rec: dict[str, Any],
) -> None:
    row = dict(rec)
    row["event_id"] = len(transport_event_log) + 1
    transport_event_log.append(row)
    if live_state is None:
        return
    _ensure_writer(live_state, TRANSPORT_FIELDS)
    writer = live_state.get("writer")
    handle = live_state.get("file")
    if writer is not None:
        writer.writerow({k: row.get(k, "") for k in TRANSPORT_FIELDS})
        if handle is not None:
            handle.flush()


def append_oht_event_trace(
    live_state: dict[str, Any] | None,
    rec: dict[str, Any],
) -> None:
    if live_state is None:
        return
    _ensure_writer(live_state, OHT_EVENT_TRACE_FIELDS)
    writer = live_state.get("writer")
    handle = live_state.get("file")
    if writer is None:
        return
    live_state["seq"] = int(live_state.get("seq", 0)) + 1
    row = {k: "" for k in OHT_EVENT_TRACE_FIELDS}
    row.update(rec)
    row["seq"] = live_state["seq"]
    writer.writerow({k: row.get(k, "") for k in OHT_EVENT_TRACE_FIELDS})
    if handle is not None:
        handle.flush()


def append_machine_live_event(
    live_state: dict[str, Any] | None,
    rec: dict[str, Any],
) -> None:
    if live_state is None:
        return
    fields = live_state.get("fields") or MACHINE_LIVE_FIELDS
    _ensure_writer(live_state, fields)
    writer = live_state.get("writer")
    handle = live_state.get("file")
    if writer is None:
        return
    row = dict(rec)
    if row.get("job_type_id") not in (None, ""):
        row["job_id"] = int(row["job_type_id"]) + 1
        row["job_type_id"] = int(row["job_type_id"]) + 1
    writer.writerow({k: row.get(k, "") for k in fields})
    if handle is not None:
        handle.flush()


def close_live_states(*states: dict[str, Any] | None) -> None:
    for state in states:
        handle = state.get("file") if state else None
        if handle is not None and not handle.closed:
            handle.close()
            state["file"] = None
            state["writer"] = None
