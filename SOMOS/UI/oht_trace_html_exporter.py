"""Export a dependency-free standalone OHT trace replay page."""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any

from oht_trace_data import TraceData


def _pack_rows(rows: list[dict[str, Any]]) -> tuple[list[str], list[list[Any]]]:
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fields.append(key)
    return fields, [[row.get(field) for field in fields] for row in rows]


def _json_for_html(value: Any) -> str:
    return (
        json.dumps(value, ensure_ascii=False, separators=(",", ":"))
        .replace("</", "<\\/")
        .replace("\u2028", "\\u2028")
        .replace("\u2029", "\\u2029")
    )


def _image_data_uri(path: Path) -> str:
    if not path.is_file():
        return ""
    return "data:image/png;base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def _build_kpis(data: TraceData) -> dict[str, Any]:
    rows = data.transport_rows
    machine_rows = data.machine_rows
    planned_rows = data.planned_machine_rows
    planned_by_key = {
        row.get("task_key"): row
        for row in planned_rows
        if row.get("task_key")
    }
    planned_keys = {row.get("task_key") for row in planned_rows if row.get("task_key")}
    actual_keys = {row.get("task_key") for row in machine_rows if row.get("task_key")}
    planned_lot_steps: dict[str, set[str]] = {}
    actual_lot_steps: dict[str, set[str]] = {}
    for row in planned_rows:
        planned_lot_steps.setdefault(str(row.get("lot_id") or ""), set()).add(
            str(row.get("step_no") or "")
        )
    for row in machine_rows:
        actual_lot_steps.setdefault(str(row.get("lot_id") or ""), set()).add(
            str(row.get("step_no") or "")
        )
    completed_lots = [
        lot
        for lot, steps in planned_lot_steps.items()
        if lot and steps and steps <= actual_lot_steps.get(lot, set())
    ]
    completed_lot_set = set(completed_lots)
    planned_completed_cmax = max(
        (
            float(row.get("end") or 0.0)
            for row in planned_rows
            if row.get("lot_id") in completed_lot_set
        ),
        default=0.0,
    )
    actual_completed_cmax = max(
        (
            float(row.get("end") or 0.0)
            for row in machine_rows
            if row.get("lot_id") in completed_lot_set
        ),
        default=0.0,
    )
    delayed_ops = 0
    early_ops = 0
    shifted_ops = 0
    max_delay = 0.0
    max_early = 0.0
    for row in machine_rows:
        planned = planned_by_key.get(row.get("task_key"))
        if not planned:
            continue
        shift = float(row.get("start") or 0.0) - float(planned.get("start") or 0.0)
        if abs(shift) > 0.1:
            shifted_ops += 1
        if shift > 0.1:
            delayed_ops += 1
            max_delay = max(max_delay, shift)
        elif shift < -0.1:
            early_ops += 1
            max_early = min(max_early, shift)

    loaded_time = sum(float(row.get("loaded_travel_time") or 0.0) for row in rows)
    empty_time = sum(float(row.get("empty_to_source_time") or 0.0) for row in rows)
    loading_time = sum(float(row.get("loading_time") or 0.0) for row in rows)
    unloading_time = sum(float(row.get("unloading_time") or 0.0) for row in rows)
    blocking_time = sum(float(row.get("blocking_loaded_time") or 0.0) for row in rows)
    waiting_time = sum(
        float(row.get("source_wait_time") or 0.0)
        + float(row.get("drop_wait_time") or 0.0)
        for row in rows
    )
    transport_deviation = sum(
        float(row.get("transport_deviation") or 0.0) for row in rows
    )
    active_oht_count = max(1, len({str(row.get("oht_id")) for row in rows}))
    utilization = (
        (loaded_time + loading_time + unloading_time)
        / max(float(data.actual_cmax or 0.0) * active_oht_count, 1.0)
    )
    return {
        "totalTransportTasks": len(rows),
        "activeOhtCount": active_oht_count,
        "plannedOps": len(planned_keys),
        "actualOps": len(actual_keys),
        "operationCoverage": len(actual_keys & planned_keys) / max(len(planned_keys), 1),
        "plannedLots": len(planned_lot_steps),
        "actualLots": len(actual_lot_steps),
        "completedLots": len(completed_lots),
        "lotCompletionRatio": len(completed_lots) / max(len(planned_lot_steps), 1),
        "plannedCompletedLotCmax": planned_completed_cmax,
        "actualCompletedLotCmax": actual_completed_cmax,
        "isPartialActual": len(actual_keys & planned_keys) < len(planned_keys),
        "totalLoadedTravel": loaded_time,
        "totalEmptyTravel": empty_time,
        "totalServiceTime": loading_time + unloading_time,
        "totalBlockingTime": blocking_time,
        "totalWaitingTime": waiting_time,
        "totalTransportDeviation": transport_deviation,
        "avgOhtUtilization": utilization,
        "delayedOps": delayed_ops,
        "earlyOps": early_ops,
        "shiftedOps": shifted_ops,
        "maxDelay": max_delay,
        "maxEarly": max_early,
    }


def export_html(data: TraceData, path: Path) -> None:
    has_exact_trace = bool(data.event_trace_rows)
    transport_rows = [
        {
            key: value
            for key, value in row.items()
            if key not in {"_original", "phases"}
        }
        for row in data.transport_rows
    ]
    row_fields, packed_rows = _pack_rows(transport_rows)
    machine_fields, packed_machine_rows = _pack_rows(data.machine_rows)
    planned_machine_fields, packed_planned_machine_rows = _pack_rows(
        data.planned_machine_rows
    )
    event_trace_fields, packed_event_trace_rows = _pack_rows(data.event_trace_rows)
    payload = {
        "meta": {
            "plannedCmax": data.planned_cmax,
            "actualCmax": data.actual_cmax,
            "cmaxDelay": data.cmax_delay,
            "cmaxLot": data.cmax_lot,
            "maxTime": data.max_time,
            "selectedOhtIds": data.selected_oht_ids,
            "hasExactTrace": has_exact_trace,
            "dispatchMode": data.dispatch_mode,
        },
        "kpis": _build_kpis(data),
        "rowFields": row_fields,
        "rows": packed_rows,
        "machineFields": machine_fields,
        "machineRows": packed_machine_rows,
        "plannedMachineFields": planned_machine_fields,
        "plannedMachineRows": packed_planned_machine_rows,
        "eventTraceFields": event_trace_fields,
        "eventTrace": packed_event_trace_rows,
        "layout": data.layout,
        "summaries": data.summary_rows,
    }
    logo_dir = Path(r"D:\bjh\OHT_salabim\로고")
    html = (
        _HTML_TEMPLATE.replace("__TRACE_DATA__", _json_for_html(payload))
        .replace("__PILAB_LOGO__", _image_data_uri(logo_dir / "PILAB.png"))
        .replace("__DONGGUK_LOGO__", _image_data_uri(logo_dir / "동국대.png"))
    )
    path.write_text(html, encoding="utf-8")


_HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SOMOS OHT Trace Replay</title>
<style>
:root{--bg:#eef3f8;--panel:#fff;--ink:#172033;--muted:#667085;--line:#d8dee8;--accent:#2563eb;--soft:#f8fafc;--head:#e2e8f0}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 "Segoe UI",Arial,sans-serif}
header{position:sticky;top:0;z-index:5;background:#f8fafc;color:#172033;padding:7px 10px;border-bottom:1px solid #d7e0ea;box-shadow:0 1px 3px #10182814}
.brand-header{display:grid;grid-template-columns:210px minmax(0,1fr) 210px;align-items:center;gap:16px}
.brand-logo{height:58px;max-width:205px;object-fit:contain;display:block}.brand-logo.left{justify-self:start}.brand-logo.right{justify-self:end}
.brand-center{min-width:0}.brand-title-row{display:flex;align-items:center;gap:8px;margin-bottom:5px}
h1{margin:0;font-size:17px;font-weight:800;color:#1f2937;letter-spacing:-.01em}.controls{display:grid;grid-template-columns:104px auto 1fr 96px;gap:10px;align-items:center}
button{border:1px solid #d7dee8;border-radius:7px;background:#fff;color:#172033;padding:7px 14px;cursor:pointer;box-shadow:0 1px 2px #1018280a}
button:hover{background:#f8fafc;border-color:#b9c4d2}.controls button{font-weight:650}.controls select{border:1px solid #d7dee8;border-radius:7px;background:#fff;padding:6px 28px 6px 9px;font:inherit}
input[type=range]{width:100%;accent-color:#476c68}.time{font-variant-numeric:tabular-nums;min-width:95px;text-align:right;color:#344054;font-weight:700}
main{padding:8px;max-width:none;margin:0}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;box-shadow:0 1px 3px #10182812}
.dashboard{height:520px;min-height:430px}
.hero{display:grid;grid-template-columns:330px minmax(640px,1fr) minmax(360px,470px);gap:8px;height:100%;min-height:0}
.panel{padding:10px;margin-top:8px;overflow:auto}.dashboard>.panel,.hero>.panel{margin-top:0}h2{font-size:16px;margin:0 0 9px;color:#243449}
.simulation-panel{background:#242424;color:#fff;display:flex;flex-direction:column;padding:0;overflow:hidden}.simulation-head{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:9px 10px;border-bottom:1px solid #3a3a3a}.simulation-title{display:flex;align-items:center;gap:8px;min-width:0}.simulation-head small{color:#c4ccd6;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.trace-mode{display:inline-block;padding:3px 7px;border-radius:999px;font-size:10px;font-weight:bold}.trace-mode.exact{background:#067647;color:#fff}.trace-mode.fallback{background:#b42318;color:#fff}.dispatch-mode{display:inline-block;padding:3px 8px;border-radius:999px;background:#1d4ed8;color:#fff;font-size:10px;font-weight:900;letter-spacing:.01em}.simulation-tools{display:flex;align-items:center;gap:5px}.simulation-tools button{padding:4px 8px;min-width:30px;background:#344054;color:#fff;border-color:#475467}.simulation-tools .primary{background:#3b82f6}.zoom-text{min-width:45px;text-align:center;color:#d0d5dd;font-variant-numeric:tabular-nums}
#simulation{width:100%;flex:1 1 auto;min-height:500px;background:#292929;border:1px solid #475467;border-radius:0;overflow:hidden;cursor:grab;touch-action:none}
#simulation.dragging{cursor:grabbing}.simulation-panel.fullscreen{position:fixed;inset:0;z-index:1100;margin:0;border:0;border-radius:0;height:100vh;padding:12px}.simulation-panel.fullscreen #simulation{height:calc(100vh - 54px)}.simulation-panel.fullscreen .simulation-head{flex:0 0 auto}
#simulation svg{width:100%;height:100%;display:block}.rail{stroke:#e5e7eb;stroke-width:.75;opacity:.72}.station{fill:#aab2bd}.machine-box{fill:#1a2a3a;stroke:#fff;stroke-width:.7}.machine-box.processing{fill:#e07000}.machine-label{fill:#ffe600;font-size:5px;text-anchor:middle}.oht{stroke:#111;stroke-width:.7;cursor:pointer;opacity:.9}.oht.selected{stroke:#fff;stroke-width:1.8;opacity:1}.oht.blocked{fill:#f59e0b;stroke:#fff;stroke-width:1.8}.oht-label{fill:#fff;font-size:5px;text-anchor:middle;pointer-events:none}.oht-halo{fill:none;stroke-width:2;opacity:.95}.oht-trail{fill:none;stroke-width:2.3;stroke-linecap:round;stroke-linejoin:round;opacity:.92}
.left-column{display:grid;grid-template-rows:auto minmax(0,1fr) auto;gap:8px;min-height:0}.side-column{display:grid;grid-template-rows:minmax(0,1fr);gap:8px;min-height:0}.state-panel,.selection-panel,.impact-panel{display:flex;flex-direction:column;min-height:0}.state-list{display:block;overflow:auto}.state-row{display:grid;grid-template-columns:56px minmax(92px,1fr) minmax(54px,.55fr) minmax(88px,.9fr);align-items:center;gap:0;border:0;border-bottom:1px solid #e5e7eb;border-radius:0;padding:8px 6px;background:#fff;box-shadow:none}.state-row:first-child{border-top:1px solid #e5e7eb}.state-row.blocked{background:#fff7ed}.state-row.loaded{background:#f0fdf4}.state-row.loading{background:#eff6ff}.state-oht{display:inline-flex;align-items:center;justify-content:center;border-radius:0;background:#fff;color:#334155;font-size:12px;font-weight:700;padding:0;white-space:nowrap}.state-name{display:inline-flex;align-items:center;justify-content:flex-start;border-radius:0;padding:0;color:#334155!important;background:transparent!important;font-size:12px;font-weight:600;white-space:nowrap}.state-job,.state-route{font-size:12px;color:#334155;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.state-job b,.state-route b{display:none}.state-empty{color:var(--muted);padding:10px;text-align:center;font-size:12px}
.oht-button-grid{display:grid;grid-template-columns:1fr;gap:6px;overflow:auto;max-height:96px;padding-right:2px}.oht-choice{background:#fff;color:#344054;border:1px solid #d7dee8;padding:7px 9px;font-size:13px;text-align:left}.oht-choice.active{background:#476c68;color:#fff;border-color:#476c68}.job-picker{margin-top:9px;display:grid;gap:5px}.job-picker select{width:100%;border:1px solid var(--line);border-radius:7px;padding:8px 9px;font:inherit;background:#fff;font-size:13px}.selection-help{margin-top:8px;color:var(--muted);font-size:11px}.selection-message{margin-top:5px;color:#b42318;font-weight:600;font-size:11px}
.notify-panel{background:#fff;display:flex;flex-direction:column;min-height:0}.notify-list{display:grid;gap:8px;overflow:auto;min-height:0}.shift-note{border:1px solid #e5e7eb;border-left:4px solid #dc2626;border-radius:10px;background:#fff;box-shadow:0 2px 8px #10182812;padding:8px;cursor:pointer}.shift-note.early{border-left-color:#64748b}.shift-note-head{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:6px}.shift-note-title{font-size:12px;font-weight:900;color:#0f172a;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.shift-note-delay{font-size:11px;font-weight:900;color:#991b1b}.shift-note.early .shift-note-delay{color:#475569}.shift-note-meta{font-size:11px;color:#475467;margin-bottom:6px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.mini-gantt{position:relative;height:34px;border-radius:7px;background:#f8fafc;border:1px solid #eef2f7;overflow:hidden}.mini-bar{position:absolute;height:8px;border-radius:999px}.mini-planned{top:7px;background:#cbd5e1;border:1px dashed #64748b}.mini-actual{top:19px;background:#2563eb;box-shadow:0 1px 2px #0f172a24}.mini-axis-label{position:absolute;left:6px;font-size:9px;color:#64748b}.mini-axis-label.planned{top:3px}.mini-axis-label.actual{top:18px}.notify-empty{color:#667085;font-size:12px;text-align:center;padding:10px;border:1px dashed #d0d5dd;border-radius:8px;background:#f8fafc}
.legend{display:flex;flex-wrap:wrap;gap:10px;margin:8px 0;color:#475467;font-size:12px}.legend span{display:inline-flex;align-items:center;gap:4px;background:#f8fafc;border:1px solid #e5e7eb;border-radius:999px;padding:4px 8px}.sw{display:inline-block;width:11px;height:11px;border-radius:2px;margin-right:4px}
.gantt-panel{display:flex;flex-direction:column;height:calc(100vh - 558px);min-height:380px;overflow:hidden;padding:0}.gantt-head{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:8px 10px;background:#f8fafc;border-bottom:1px solid #dbe4ef}.gantt-head h2{margin:0;font-size:16px;letter-spacing:-.01em}.gantt-actions{display:flex;align-items:center;gap:8px}.mode-toggle{display:inline-flex;border:1px solid #cbd5e1;border-radius:7px;overflow:hidden;background:#fff;box-shadow:0 1px 2px #10182812}.mode-toggle button{border-radius:0;background:#fff;color:#344054;border:0;border-right:1px solid #e5e7eb;padding:6px 12px;font-weight:700}.mode-toggle button:last-child{border-right:0}.mode-toggle button.active{background:#476c68;color:#fff}.gantt-panel .legend{flex:0 0 auto;margin:0;padding:6px 10px;border-bottom:1px solid #e5e7eb;background:#fff}.chart-scroll{flex:1 1 auto;min-height:0;overflow:auto;border-top:0;background:#fff}.gantt-panel.fullscreen{position:fixed;inset:0;z-index:1000;margin:0;border-radius:0;height:100vh;background:#fff;padding:0}.gantt-panel.fullscreen .chart-scroll{height:calc(100vh - 92px)}
#chart{min-width:1700px}.row-band{fill:#f3f7fb}.row-line{stroke:#dbe4ef;stroke-width:1}.row-label{font-size:12px;font-weight:800;fill:#0f172a}.axis{stroke:#b8c4d3;stroke-width:1}.tick{font-size:12px;fill:#334155;font-weight:650}
.planned-task{fill-opacity:.92;stroke:#334155;stroke-width:.6}.planned-ref{fill:#e5e7eb;fill-opacity:.72;stroke:#64748b;stroke-width:1.15;stroke-dasharray:6 4}.live-task{cursor:pointer;stroke:#0f172a;stroke-width:.85;filter:drop-shadow(0 1px 1px #0f172a24)}.live-task.building{filter:drop-shadow(0 0 5px #2563eb55)}.live-task.dim{opacity:.13;filter:none}.live-task.context{opacity:.58;filter:none}.live-task.related{opacity:1}.live-task.updated{stroke:#dc2626;stroke-width:2.6;stroke-dasharray:7 3}.live-task.early{stroke:#64748b;stroke-width:2;stroke-dasharray:5 3}.delay-shift{pointer-events:none}.delay-shift-line{stroke:#dc2626;stroke-width:1.7;stroke-linecap:round;stroke-dasharray:6 3;marker-end:url(#delayArrow)}.delay-shift-label{fill:#991b1b;font-size:10px;font-weight:800;paint-order:stroke;stroke:#fff;stroke-width:3px}.update-from{fill:#fff7ed;stroke:#f97316;stroke-width:2;stroke-dasharray:5 3;opacity:.95}.update-to{fill:none;stroke:#dc2626;stroke-width:3.2;opacity:.95}.update-sweep{stroke:#dc2626;stroke-width:5;stroke-linecap:round;opacity:.82;marker-end:url(#delayArrow)}.update-label{fill:#7f1d1d;font-size:10px;font-weight:950;paint-order:stroke;stroke:#fff;stroke-width:3px}.lift-shadow{fill:#111827;opacity:.16}.lift-guide{fill:none;stroke:#dc2626;stroke-width:2.2;stroke-dasharray:6 4;opacity:.9;marker-end:url(#delayArrow)}.lifted-task{stroke:#dc2626;stroke-width:2.6;filter:drop-shadow(0 8px 7px #7f1d1d55)}.lift-label{fill:#991b1b;font-size:10px;font-weight:950;paint-order:stroke;stroke:#fff;stroke-width:3px}.cursor{stroke:#111827;stroke-width:1.5;stroke-dasharray:5 4;opacity:.8}
.transport-window{fill:#fff1f2;stroke:#dc2626;stroke-width:1.6;stroke-dasharray:5 3;opacity:.88}.transport-guide-label{fill:#991b1b;font-size:9px;font-weight:900;paint-order:stroke;stroke:#fff;stroke-width:3px}.oht-transport-segment{fill-opacity:.2;stroke-width:2.4;stroke-dasharray:5 3;pointer-events:none}.oht-task-outline{fill:none;stroke-width:3;stroke-dasharray:7 3;opacity:.95;pointer-events:none;filter:drop-shadow(0 1px 2px #0f172a33)}.oht-task-stripe{stroke-width:4.5;stroke-linecap:round;opacity:.95;pointer-events:none}.oht-task-label{font-size:9.5px;font-weight:950;paint-order:stroke;stroke:#fff;stroke-width:3px;pointer-events:none}.oht-badge{fill:#0b1220;stroke:#fff;stroke-width:2.2;filter:drop-shadow(0 2px 4px #0f172a66)}.oht-job-marker{fill:#fff;font-size:10.5px;font-weight:950;text-anchor:middle;dominant-baseline:middle;paint-order:stroke;stroke:#0b1220;stroke-width:1.6px}.oht-job-label-bg{fill:#fff;stroke:#cbd5e1;stroke-width:.7;filter:drop-shadow(0 1px 1px #0f172a22)}.oht-job-label{fill:#0f172a;font-size:8.5px;font-weight:900;text-anchor:middle;dominant-baseline:middle}.oht-job-dot{fill:#ff2d2d;stroke:#fff;stroke-width:1}.cascade-stone{fill:#dc2626;stroke:#fff;stroke-width:1.5;filter:drop-shadow(0 1px 2px #991b1b66)}.cascade-ring{fill:none;stroke:#dc2626;stroke-width:1.5;stroke-dasharray:3 2;opacity:.8}
@keyframes flashBar{0%,100%{stroke:#dc2626;stroke-width:2.4;opacity:1}50%{stroke:#facc15;stroke-width:4.2;opacity:.95}}
@keyframes flashFill{0%,100%{opacity:.12}50%{opacity:.72}}
@keyframes shiftPulse{0%,100%{opacity:.08;transform:translateX(0)}18%{opacity:1}62%{opacity:1;transform:translateX(6px)}82%{opacity:.18;transform:translateX(6px)}}
.delay{fill:#b42318;font-size:9px}.tooltip{position:fixed;display:none;pointer-events:none;background:#111827;color:#fff;padding:8px;border-radius:5px;max-width:350px;z-index:10;white-space:pre-line}
.empty{color:var(--muted);padding:18px;text-align:center}
.state-row{cursor:pointer}.state-row.selected{border-color:var(--track-color,#dc2626)!important;border-left-color:var(--track-color,#dc2626)!important;border-left-width:7px!important;box-shadow:inset 0 0 0 2px color-mix(in srgb,var(--track-color,#dc2626) 55%,transparent),0 0 0 4px color-mix(in srgb,var(--track-color,#dc2626) 20%,transparent),0 10px 22px #10182824!important}.state-row.focused{border-color:var(--track-color,#dc2626)!important;box-shadow:0 0 0 2px color-mix(in srgb,var(--track-color,#dc2626) 24%,transparent)!important;background:#fff!important}.oht-focus-ring{fill-opacity:.08;stroke-width:3.2;stroke-dasharray:4 2;opacity:.95;pointer-events:none}.track-swatch{display:inline-block;width:12px;height:12px;border-radius:999px;background:var(--track-color,#cbd5e1);border:2px solid #fff;box-shadow:0 0 0 2px var(--track-color,#cbd5e1)}.shift-legend{display:flex;gap:8px;margin-bottom:5px;color:#475467;font-size:10px;font-weight:800}.shift-legend span{display:inline-flex;align-items:center;gap:4px}.shift-key{width:16px;height:7px;border-radius:999px;display:inline-block}.shift-key.planned{background:#cbd5e1;border:1px dashed #64748b}.shift-key.actual{background:#2563eb}.mini-gantt{height:68px}.mini-planned{top:22px}.mini-actual{top:44px}.mini-axis-label{font-size:10px;font-weight:900}.mini-axis-label.planned{top:17px}.mini-axis-label.actual{top:39px;color:#1d4ed8}.mini-end-label{position:absolute;right:6px;font-size:9px;color:#667085}.mini-end-label.planned{top:17px}.mini-end-label.actual{top:39px}.shift-note{position:relative;overflow:hidden}.shift-note::before{content:"";position:absolute;inset:0 0 auto;height:3px;background:linear-gradient(90deg,#dc2626,#f97316,#facc15);opacity:.85}.shift-note-head{margin-top:2px}.shift-event-row{display:grid;grid-template-columns:auto 1fr auto;align-items:center;gap:8px;margin:5px 0 7px}.shift-event-badge{display:inline-flex;align-items:center;border-radius:999px;background:#fee2e2;color:#991b1b;font-size:10px;font-weight:900;padding:3px 7px;letter-spacing:.03em}.shift-event-main{min-width:0}.shift-event-title{font-size:12px;font-weight:900;color:#0f172a;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.shift-event-sub{font-size:10px;color:#667085;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.shift-delta{font-size:16px;font-weight:950;color:#b42318;font-variant-numeric:tabular-nums}.shift-note.early .shift-delta{color:#475569}.shift-change{display:grid;grid-template-columns:1fr auto 1fr;gap:6px;align-items:center;margin:6px 0}.shift-timebox{border:1px solid #e5e7eb;border-radius:8px;background:#f8fafc;padding:5px 6px}.shift-timebox.actual{border-color:#bfdbfe;background:#eff6ff}.shift-time-label{font-size:9px;font-weight:900;color:#667085}.shift-time-value{font-size:11px;font-weight:900;color:#0f172a;font-variant-numeric:tabular-nums}.shift-arrow-text{font-size:15px;font-weight:950;color:#dc2626}.shift-arrow-line{position:absolute;top:34px;height:0;border-top:2px solid #dc2626;z-index:2}.shift-arrow-line::after{content:"";position:absolute;right:-1px;top:-4px;border-left:6px solid #dc2626;border-top:4px solid transparent;border-bottom:4px solid transparent}.shift-note.early .shift-arrow-line{border-top-color:#64748b}.shift-note.early .shift-arrow-line::after{border-left-color:#64748b}.shift-cascade{display:flex;flex-wrap:wrap;gap:4px;margin-top:6px}.shift-chip{display:inline-flex;align-items:center;gap:3px;border:1px solid #fecaca;background:#fff1f2;color:#991b1b;border-radius:999px;padding:2px 6px;font-size:10px;font-weight:800}.shift-chip.neutral{border-color:#e5e7eb;background:#f8fafc;color:#475467}.shift-age{position:absolute;left:0;right:0;bottom:0;height:3px;background:#e5e7eb}.shift-age-fill{height:100%;background:#dc2626}.notify-empty b{color:#0f172a}main{max-width:none}.dashboard{height:560px}.hero{grid-template-columns:330px minmax(760px,1fr) minmax(360px,470px);gap:8px}.simulation-panel{min-width:0;overflow:hidden}.side-column{min-width:260px}.resizing-side{user-select:none;cursor:col-resize}
@media(max-width:950px){.dashboard{height:auto}.hero{grid-template-columns:1fr}.hero>.panel,.side-column{margin-top:10px}.side-column{grid-template-rows:auto auto}.controls{grid-template-columns:auto 1fr}.controls input{grid-column:1/-1}#simulation{height:55vh}.gantt-panel{height:60vh}}

/* V2 final visual pass: pure HTML executive dashboard layout */
body{background:linear-gradient(180deg,#eef3f8 0,#f6f8fb 100%)}
header{padding:8px 12px;background:#f7fafc}
h1{font-size:16px;margin-bottom:8px}
main{height:calc(100vh - 86px);display:grid;grid-template-rows:minmax(500px,54vh) minmax(390px,1fr);gap:8px;overflow:hidden}
.dashboard{height:auto;min-height:0}
.hero{height:100%;grid-template-columns:320px minmax(780px,1fr) minmax(360px,450px);gap:8px}
.panel{border-color:#d8e0ea;border-radius:8px;box-shadow:0 1px 2px #10182812}
.left-column{grid-template-rows:auto minmax(0,1fr) auto}
.selection-panel,.notify-panel,.info-panel,.state-panel{padding:12px}
.selection-panel h2,.notify-panel h2,.state-panel h2,.info-panel h2{font-size:15px;color:#253247;margin-bottom:10px}
.simulation-panel{min-height:0;border:0;background:#242424}
#simulation{min-height:0;height:100%;border:0}
.simulation-head{min-height:44px;background:#202020}
.simulation-title h2{color:#fff;font-size:15px;margin:0}
.side-column{height:100%;min-height:0}
.state-panel{overflow:hidden}
.state-list{border:1px solid #e5e7eb;border-radius:7px;overflow:auto}
.state-list::before{content:"OHT    State/Event    Job    From → To";display:grid;grid-template-columns:56px 1fr .55fr .9fr;padding:7px 6px;background:#f8fafc;border-bottom:1px solid #e5e7eb;color:#475467;font-size:12px;font-weight:700;white-space:pre}
.state-row{grid-template-columns:56px 1fr .55fr .9fr;padding:8px 6px}
.gantt-panel{height:auto;min-height:0;margin-top:0}
.gantt-head{min-height:42px}
.chart-scroll{background:#fff}
.legend span{font-size:11px;border-radius:5px}
.oht-choice{border-radius:7px}
.oht-choice.active{box-shadow:inset 0 0 0 1px #ffffff55}
.shift-note{border-radius:8px}
.left-column{grid-template-rows:auto minmax(0,1fr)}
.selection-panel,.notify-panel,.state-panel{padding:12px}
.selection-panel h2,.notify-panel h2,.state-panel h2{font-size:15px;color:#253247;margin-bottom:10px}
.notify-panel{min-height:0}
.notify-list{min-height:0;overflow:auto}
.state-list{border:0;border-radius:0;overflow:auto;display:grid;gap:8px;padding:1px}
.state-list::before{display:none}
.state-row{display:grid;grid-template-columns:minmax(62px,.55fr) minmax(120px,1fr);grid-template-areas:"oht state" "job route";gap:6px 8px;padding:10px;border:1px solid #e5e7eb;border-radius:10px;background:linear-gradient(180deg,#fff 0,#f8fafc 100%);box-shadow:0 1px 2px #10182812}
.state-row:first-child{border-top:1px solid #e5e7eb}
.state-row.loaded{background:linear-gradient(180deg,#f0fdf4 0,#fff 100%);border-color:#bbf7d0}
.state-row.loading{background:linear-gradient(180deg,#eff6ff 0,#fff 100%);border-color:#bfdbfe}
.state-row.blocked{background:linear-gradient(180deg,#fff7ed 0,#fff 100%);border-color:#fed7aa}
.state-oht{grid-area:oht;border-radius:999px;background:#0f172a;color:#fff;font-size:12px;font-weight:900;padding:5px 7px;justify-content:center}
.state-name{grid-area:state;justify-content:flex-start;border-radius:999px;padding:5px 9px;font-size:12px;font-weight:900;color:#fff!important;background:#64748b!important;min-width:0;overflow:hidden;text-overflow:ellipsis}
.state-row.loaded .state-name{background:#16a34a!important}.state-row.loading .state-name{background:#2563eb!important}.state-row.blocked .state-name{background:#f97316!important}
.state-job{grid-area:job;font-size:12px;color:#0f172a;font-weight:850;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.state-route{grid-area:route;justify-self:stretch;text-align:center;font-size:12px;color:#475467;background:#eef2f7;border:1px solid #e2e8f0;border-radius:999px;padding:3px 7px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
/* Screenshot-sized presentation layout */
header{padding:4px 8px}
h1{font-size:13px;margin-bottom:5px}
.controls{grid-template-columns:86px auto 1fr 88px;gap:8px}
.controls button{padding:4px 10px;font-size:11px}.controls select{padding:3px 18px 3px 6px;font-size:11px}.time{font-size:11px}
main{height:calc(100vh - 54px);grid-template-rows:minmax(430px,61.5vh) minmax(285px,1fr);gap:6px;padding:4px;overflow:hidden}
.hero{grid-template-columns:220px minmax(820px,1fr) 310px;gap:6px}
.panel{border-radius:6px}
.selection-panel,.notify-panel,.state-panel{padding:8px}
.selection-panel h2,.notify-panel h2,.state-panel h2{font-size:12px;margin-bottom:7px}
.oht-button-grid{max-height:72px;gap:4px}
.oht-choice{padding:4px 7px;font-size:11px;border-radius:4px}
.job-picker{margin-top:5px}.job-picker select{padding:5px 7px;font-size:11px}.job-picker label,.selection-help,.selection-message{font-size:10px}
.simulation-head{min-height:34px;padding:5px 8px}
.simulation-title h2{font-size:12px}.simulation-head small{display:none}
.trace-mode,.dispatch-mode{font-size:8px;padding:2px 5px}
.simulation-tools button{padding:3px 7px;font-size:10px;min-width:24px}.zoom-text{font-size:10px;min-width:38px}
#simulation{height:100%;min-height:0}
.state-row{grid-template-columns:92px 1fr;grid-template-areas:"oht state" "job route";gap:6px;padding:8px;border-radius:8px}
.state-oht{font-size:10px;padding:4px 7px}.state-name{font-size:10px;padding:4px 7px}.state-job,.state-route{font-size:10px}
.gantt-head{min-height:34px;padding:5px 8px}.gantt-head h2{font-size:13px}
.mode-toggle button,.gantt-actions button{padding:4px 8px;font-size:10px}
.gantt-panel .legend{padding:4px 8px;gap:5px}.legend span{font-size:9px;padding:3px 6px}
.chart-scroll{min-height:0}
/* Gantt as popup overlay */
main{height:calc(100vh - 54px);display:block;padding:4px;overflow:hidden}
.dashboard{height:100%;min-height:0}
.hero{height:100%;grid-template-columns:220px minmax(820px,1fr) 310px;gap:6px}
.gantt-launcher{position:fixed;left:50%;bottom:14px;transform:translateX(-50%);z-index:25;display:flex;gap:8px;padding:6px;background:#ffffffd9;border:1px solid #d8e0ea;border-radius:999px;box-shadow:0 10px 28px #10182826;backdrop-filter:blur(8px)}
.gantt-launcher button{padding:8px 14px;border-radius:999px;font-size:12px;font-weight:850;background:#fff}
.gantt-launcher button.primary{background:#1d4ed8;color:#fff;border-color:#1d4ed8}
.gantt-modal{position:fixed;inset:0;z-index:900;display:none;padding:44px 56px;background:#0f172acc;backdrop-filter:blur(4px)}
.gantt-modal.open{display:grid;grid-template-columns:minmax(340px,34vw) minmax(0,1fr);grid-template-rows:1fr;gap:10px}
.gantt-preview{display:flex;flex-direction:column;min-height:0;overflow:hidden;background:#202020;border:1px solid #475467;border-radius:10px;box-shadow:0 12px 36px #02061770}
.gantt-preview-head{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:7px 10px;color:#fff;border-bottom:1px solid #3b3b3b;font-size:12px;font-weight:850}
.preview-tools{display:grid;grid-template-columns:auto auto 42px auto auto 86px;align-items:center;gap:5px}.preview-tools button{padding:3px 7px;border-radius:5px;background:#344054;color:#fff;border-color:#475467;font-size:10px}.preview-tools span{font-size:10px;color:#d0d5dd;text-align:center}
.preview-tools .preview-play{background:#1d4ed8;color:#fff;border-color:#1d4ed8;font-weight:900}
#previewZoomText{min-width:42px}
#ganttPreviewTime{justify-self:end;min-width:86px;margin-left:8px;font-variant-numeric:tabular-nums;text-align:right}
#ganttPreviewSimulation{flex:1 1 auto;min-height:0;overflow:hidden;background:#242424}
#ganttPreviewSimulation svg{width:100%;height:100%;display:block}
.gantt-modal .gantt-panel{height:100%;width:100%;min-width:0;margin:0;border-radius:10px;box-shadow:0 22px 70px #02061780;background:#fff}
.gantt-modal .chart-scroll{height:auto;flex:1 1 auto;overflow:auto}
.gantt-modal .gantt-head{min-height:42px;padding:8px 12px}
.gantt-modal .gantt-head h2{font-size:16px}
.gantt-close{background:#111827!important;color:#fff!important;border-color:#111827!important}
.gantt-modal .gantt-panel.fullscreen{inset:0;z-index:1100;border-radius:0}
.gantt-modal .gantt-panel.fullscreen{height:100vh}
.selection-panel h2,.notify-panel h2,.state-panel h2{font-size:14px}
.oht-choice{font-size:13px;padding:6px 8px}
.job-picker select{font-size:13px;padding:7px 8px}
.job-picker label,.selection-help,.selection-message{font-size:12px}
.shift-event-title{font-size:13px}.shift-event-sub,.shift-note-meta{font-size:11.5px}.shift-delta{font-size:18px}
.state-row{padding:10px;gap:8px}
.state-oht,.state-name{font-size:12px;padding:5px 8px}
.state-job,.state-route{font-size:12px}
/* Presentation left panel expansion */
.hero{height:100%;grid-template-columns:280px minmax(720px,1fr) 330px;gap:6px}
.selection-panel{padding:10px}
.selection-panel h2{font-size:15px}
.oht-button-grid{max-height:156px;gap:6px}
.oht-choice{font-size:14px;padding:8px 10px;border-radius:6px}
.job-picker{margin-top:9px}
.job-picker select{font-size:14px;padding:8px 9px}
.job-picker label,.selection-help,.selection-message{font-size:12px}
/* First screen readable text pass */
h1{font-size:16px}
.controls button,.controls select,.time{font-size:13px}
.selection-panel,.notify-panel,.state-panel{padding:12px}
.selection-panel h2,.notify-panel h2,.state-panel h2{font-size:17px}
.oht-choice{font-size:15px;padding:9px 11px}
.job-picker select{font-size:15px;padding:9px 10px}
.job-picker label,.selection-help,.selection-message{font-size:13px}
.shift-event-title{font-size:15px}.shift-event-sub,.shift-note-meta{font-size:13px}.shift-event-badge{font-size:11px}.shift-delta{font-size:20px}
.state-oht,.state-name{font-size:13px;padding:6px 9px}
.state-job,.state-route{font-size:13px}
.notify-empty{font-size:13px}
.top-tick{font-size:13px;font-weight:900;fill:#27476e}
.gantt-job-filter{display:flex;align-items:center;gap:6px;font-size:12px;font-weight:800;color:#475467}
.gantt-job-filter select{min-width:210px;max-width:320px;border:1px solid #cbd5e1;border-radius:7px;background:#fff;padding:5px 8px;font-size:12px;font-weight:700;color:#172033}
.gantt-modal .gantt-head{display:grid;grid-template-columns:auto minmax(560px,1fr);align-items:center;gap:18px}
.gantt-modal .gantt-head h2{white-space:nowrap}
.gantt-modal .gantt-actions{display:grid;grid-template-columns:minmax(430px,1fr) auto auto auto;align-items:center;gap:12px;min-width:0}
.gantt-modal .gantt-job-filter{justify-self:stretch}
.gantt-modal .gantt-job-filter select{width:100%;max-width:none;min-width:0}
.gantt-modal .mode-toggle button,.gantt-modal .gantt-actions>button{padding:6px 10px;font-size:11px;font-weight:850;border-radius:7px}
.gantt-modal .mode-toggle{border-radius:9px}
.gantt-modal .gantt-close{padding-inline:12px}
/* Executive selected OHT state panel */
.state-panel{background:linear-gradient(180deg,#ffffff 0,#f8fafc 100%)}
.state-list{display:grid;gap:10px;padding:2px;overflow:auto}
.state-row{position:relative;display:grid;grid-template-columns:1fr;grid-template-areas:none;gap:10px;padding:12px 12px 11px 14px;border:1px solid #e2e8f0;border-left:4px solid var(--state-color,#64748b);border-radius:12px;background:#fff;box-shadow:0 5px 16px #10182810;transition:border-color .15s ease,box-shadow .15s ease,transform .15s ease}
.state-row:hover{border-color:#cbd5e1;box-shadow:0 10px 22px #10182818;transform:translateY(-1px)}
.state-row.focused{border-color:#ef4444!important;border-left-color:#ef4444!important;box-shadow:0 0 0 3px #fee2e2,0 10px 22px #10182818!important;background:#fff!important}
.state-row.loaded,.state-row.loading,.state-row.blocked{background:#fff;border-color:#e2e8f0}
.state-top{display:flex;align-items:center;justify-content:space-between;gap:10px}
.state-oht{display:inline-flex;align-items:center;justify-content:center;grid-area:auto;border-radius:8px;background:#0f172a;color:#fff;font-size:13px;font-weight:900;letter-spacing:.02em;padding:6px 10px;min-width:72px}
.state-name{display:inline-flex;align-items:center;grid-area:auto;border-radius:999px;border:1px solid color-mix(in srgb,var(--state-color,#64748b) 28%,#ffffff);background:color-mix(in srgb,var(--state-color,#64748b) 12%,#ffffff)!important;color:var(--state-color,#64748b)!important;font-size:12px;font-weight:900;padding:5px 10px;max-width:180px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.state-metrics{display:grid;grid-template-columns:1fr 1.35fr;gap:8px}
.state-job,.state-route{grid-area:auto;display:grid;gap:2px;border:1px solid #edf2f7;border-radius:10px;background:#f8fafc;padding:7px 8px;color:#0f172a;font-size:12px;text-align:left;min-width:0}
.state-job b,.state-route b{display:block;color:#64748b;font-size:10px;font-weight:800;text-transform:uppercase;letter-spacing:.05em}
.state-job strong,.state-route strong{font-size:13px;font-weight:900;color:#111827;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
/* Compact operations-monitor state rows */
.state-list{gap:8px;padding:1px}
.state-row{display:grid;grid-template-columns:1fr;gap:7px;padding:10px 11px 9px 12px;border:1px solid #e2e8f0;border-left:4px solid var(--state-color,#64748b);border-radius:10px;background:#fff;box-shadow:0 2px 8px #1018280c;transform:none}
.state-row:hover{transform:none;box-shadow:0 6px 16px #10182814}
.state-main{display:grid;grid-template-columns:auto minmax(0,1fr);align-items:center;gap:8px}
.state-oht{grid-area:auto;min-width:66px;border-radius:7px;background:#111827;color:#fff;font-size:13px;font-weight:900;padding:5px 9px;letter-spacing:.01em}
.state-name{grid-area:auto;justify-self:end;max-width:170px;border:1px solid color-mix(in srgb,var(--state-color,#64748b) 28%,#fff);background:color-mix(in srgb,var(--state-color,#64748b) 10%,#fff)!important;color:var(--state-color,#64748b)!important;border-radius:999px;font-size:12px;font-weight:900;padding:4px 9px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.state-sub{display:grid;grid-template-columns:minmax(72px,.75fr) minmax(120px,1.25fr);gap:10px;color:#111827;font-size:13px;font-weight:850;min-width:0}
.state-sub span{min-width:0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.state-sub b{margin-right:5px;color:#64748b;font-size:10px;font-weight:900;text-transform:uppercase;letter-spacing:.05em}
.state-job,.state-route,.state-metrics,.state-top{display:contents}
/* Compact job-shift cards for narrow side panel */
.notify-list{gap:7px}
.shift-note{padding:8px 8px 9px;border-radius:10px;box-shadow:0 2px 10px #10182810}
.shift-note::before{height:2px}
.shift-event-row{display:grid;grid-template-columns:auto minmax(0,1fr) auto;grid-template-areas:"badge title delta";gap:6px;align-items:center;margin:3px 0 5px}
.shift-event-badge{grid-area:badge;font-size:9px;padding:3px 6px;letter-spacing:.02em}
.shift-event-main{display:block;grid-area:title;min-width:0}
.shift-event-title{display:block;white-space:normal;overflow:hidden;text-overflow:clip;font-size:12px;line-height:1.15;max-height:2.3em}
.shift-event-sub{display:none}
.shift-delta{grid-area:delta;justify-self:end;font-size:17px;line-height:1;font-weight:950;white-space:nowrap}
.shift-note-meta{white-space:normal;overflow:hidden;text-overflow:clip;font-size:11px;line-height:1.25;max-height:2.5em;margin:2px 0 5px}
.shift-change{grid-template-columns:minmax(0,1fr) 14px minmax(0,1fr);gap:4px;margin:4px 0}
.shift-timebox{padding:4px 5px;border-radius:7px;min-width:0}
.shift-time-label{font-size:8px}
.shift-time-value{font-size:10px;line-height:1.15;white-space:normal;overflow-wrap:anywhere}
.shift-arrow-text{font-size:12px;overflow:hidden;text-indent:-999px;position:relative}
.shift-arrow-text::after{content:"->";position:absolute;left:0;right:0;text-indent:0;text-align:center;color:#dc2626}
.shift-legend,.mini-axis-label,.mini-end-label{display:none}
.mini-gantt{height:34px;margin-top:4px}
.mini-planned{top:8px}.mini-actual{top:21px}
.shift-cascade{max-height:24px;overflow:hidden;margin-top:5px}
.shift-chip{font-size:9px;padding:2px 5px;max-width:100%;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
/* Readability pass for replay title and OHT state */
.simulation-title h2{font-size:18px!important;font-weight:900;letter-spacing:.01em}
.simulation-head{min-height:42px}
.state-row{padding:12px 12px 11px 13px;gap:9px}
.state-oht{font-size:14px;min-width:74px;padding:6px 10px}
.state-name{background:var(--state-color,#64748b)!important;border-color:var(--state-color,#64748b)!important;color:#fff!important;font-size:13px;padding:5px 11px;box-shadow:inset 0 -1px 0 #00000018}
.state-sub{grid-template-columns:minmax(78px,.7fr) minmax(132px,1.3fr);gap:12px;font-size:15px;font-weight:900}
.state-sub b{font-size:12px;color:#475569;margin-right:6px}
.state-sub span{color:#0f172a}
.state-panel h2{display:flex;align-items:center;justify-content:space-between;gap:10px}
.state-panel h2::after{content:"LIVE";display:inline-flex;align-items:center;border-radius:999px;background:#ecfdf3;color:#067647;border:1px solid #abefc6;padding:3px 8px;font-size:10px;font-weight:950;letter-spacing:.08em}
.live-task.context{opacity:.88;filter:drop-shadow(0 1px 1px #0f172a12)}
.live-task.related{opacity:1;stroke:#0f172a;stroke-width:1.15}
.oht-badge.moving{fill:#dc2626;stroke:#fff;stroke-width:1.4;filter:drop-shadow(0 0 5px #dc262688)}
/* V3: left state, centered replay, right-side machine Gantt. */
body.v3-layout main{height:calc(100vh - 54px);display:block;padding:4px;overflow:hidden}
body.v3-layout .dashboard{height:calc(100% - 70px)!important;min-height:0}
body.v3-layout .hero{height:100%;display:grid;grid-template-columns:240px minmax(0,1fr);gap:8px;min-height:0}
body.v3-layout .left-column{display:grid;grid-template-rows:minmax(0,1fr);min-height:0}
body.v3-layout .main-workspace{display:grid;grid-template-columns:minmax(700px,64%) minmax(440px,36%);grid-template-rows:1fr;gap:8px;min-width:0;min-height:0}
body.v3-layout .main-workspace .simulation-panel{min-height:0;height:100%;align-self:stretch}
body.v3-layout .main-workspace #simulation{min-height:0;height:100%}
body.v3-layout .main-workspace .gantt-panel{height:100%;min-height:0;margin:0;border-radius:8px;background:#fff}
body.v3-layout .main-workspace .chart-scroll{height:auto;flex:1 1 auto;overflow:auto}
body.v3-layout .side-column,body.v3-layout .gantt-launcher,body.v3-layout .gantt-preview{display:none!important}
body.v3-layout .gantt-modal{display:block!important;position:static!important;inset:auto!important;padding:0!important;background:transparent!important;backdrop-filter:none!important}
body.v3-layout .gantt-modal.open{display:block!important}
body.v3-layout .gantt-modal .gantt-panel{box-shadow:none;width:100%;height:100%;border-radius:8px}
body.v3-layout .gantt-close{display:none!important}
@media(max-width:1300px){body.v3-layout .hero{grid-template-columns:220px minmax(0,1fr)}body.v3-layout .main-workspace{grid-template-columns:minmax(600px,62%) minmax(360px,38%)}}
.oht-select-modal{position:fixed;inset:0;z-index:2000;display:none;place-items:center;background:#0f172acc;backdrop-filter:blur(5px)}
.oht-select-modal.open{display:grid}
.oht-select-card{width:min(460px,calc(100vw - 40px));background:#fff;border:1px solid #d8e0ea;border-radius:16px;box-shadow:0 24px 70px #02061770;padding:22px}
.oht-select-card h2{margin:0 0 8px;font-size:21px;color:#0f172a}
.oht-select-card p{margin:0 0 14px;color:#475467;font-size:13px}
.oht-select-card input{width:100%;border:1px solid #cbd5e1;border-radius:10px;padding:12px 13px;font-size:16px;font-weight:800;color:#0f172a}
.oht-select-actions{display:flex;justify-content:flex-end;gap:8px;margin-top:14px}
.oht-select-actions .primary{background:#1d4ed8;color:#fff;border-color:#1d4ed8}
.oht-select-error{min-height:18px;margin-top:8px;color:#b42318;font-size:12px;font-weight:800}
.chart-stage{position:relative;min-width:max-content}
#chart{padding-left:74px}
#chartLabels{position:sticky;left:0;top:0;z-index:6;width:74px;float:left;background:linear-gradient(90deg,#fff 0,#fff 90%,#ffffff00 100%);box-shadow:5px 0 8px #10182812;pointer-events:none}
.machine-label-row{position:absolute;left:0;width:68px;height:34px;display:flex;align-items:center;justify-content:flex-end;padding-right:8px;font-size:12px;font-weight:900;color:#0f172a;background:inherit;border-right:1px solid #dbe4ef}
.machine-label-row:nth-child(even){background:#f3f7fb}
.machine-label-head{position:absolute;left:0;top:0;width:68px;height:32px;background:#f8fafc;border-right:1px solid #dbe4ef;border-bottom:1px solid #b8c4d3}
.row-label{display:none}
.kpi-strip{display:grid;grid-template-columns:repeat(8,minmax(112px,1fr));gap:6px;margin:0 0 6px}
.kpi-card{position:relative;overflow:hidden;border:1px solid #d8e0ea;border-radius:9px;background:linear-gradient(180deg,#fff,#f8fafc);box-shadow:0 1px 3px #10182814;padding:8px 10px;min-height:48px}
.kpi-card::before{content:"";position:absolute;left:0;top:0;bottom:0;width:4px;background:#476c68}
.kpi-card.warn::before{background:#dc2626}.kpi-card.good::before{background:#16a34a}.kpi-card.blue::before{background:#2563eb}.kpi-card.amber::before{background:#f59e0b}
.kpi-label{font-size:10px;font-weight:950;letter-spacing:.06em;text-transform:uppercase;color:#667085;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.kpi-value{margin-top:3px;font-size:18px;line-height:1.05;font-weight:950;color:#111827;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.kpi-sub{display:none}
.kpi-value.negative{color:#047857}.kpi-value.positive{color:#b42318}
.dashboard{height:calc(100% - 70px)!important}
@media(max-width:1400px){.kpi-strip{grid-template-columns:repeat(4,minmax(120px,1fr))}.kpi-card{min-height:52px}.kpi-value{font-size:16px}}
/* V3 clean paper / IEEE figure theme */
body.v3-layout{background:#f3f6fa;color:#172033}
body.v3-layout header{background:#ffffff;border-bottom:1px solid #d7e0ea;box-shadow:0 2px 12px #10182812}
body.v3-layout h1{color:#172033;font-weight:950;letter-spacing:-.02em}
body.v3-layout .controls label,body.v3-layout .time{color:#334155}
body.v3-layout button,body.v3-layout .controls select{background:#ffffff;color:#172033;border-color:#cbd5e1;box-shadow:0 1px 2px #1018280f}
body.v3-layout button:hover{background:#f8fafc;border-color:#94a3b8}
body.v3-layout input[type=range]{accent-color:#345995}
body.v3-layout .panel{background:#ffffff;border:1px solid #d8e0ea;box-shadow:0 8px 24px #10182810;border-radius:12px}
body.v3-layout .simulation-panel,body.v3-layout .main-workspace .gantt-panel,body.v3-layout .state-panel{background:#ffffff;border:1px solid #d8e0ea}
body.v3-layout .simulation-head,body.v3-layout .gantt-head{background:linear-gradient(180deg,#ffffff 0,#f8fafc 100%);border-bottom:1px solid #d8e0ea;color:#172033}
body.v3-layout .simulation-title h2,body.v3-layout .gantt-head h2,body.v3-layout .state-panel h2{color:#172033}
body.v3-layout #simulation{background:#242424;border-top:1px solid #e5e7eb}
body.v3-layout .trace-mode,body.v3-layout .dispatch-mode,body.v3-layout #traceModeHelp{display:none!important}
body.v3-layout .kpi-strip{gap:8px}
body.v3-layout .kpi-card{background:#ffffff;border-color:#d8e0ea;box-shadow:0 4px 14px #10182812;min-height:50px}
body.v3-layout .kpi-card::before{width:5px;background:#345995}
body.v3-layout .kpi-card.good::before{background:#0f9f6e}.kpi-card.warn::before{background:#dc2626}.kpi-card.amber::before{background:#f59e0b}.kpi-card.blue::before{background:#345995}
body.v3-layout .kpi-label{color:#64748b}.kpi-value{color:#172033}.kpi-sub{color:#667085}
body.v3-layout .kpi-value.negative{color:#087f5b}.kpi-value.positive{color:#b42318}
body.v3-layout .state-list{background:transparent}
body.v3-layout .state-row{background:#ffffff;border:1px solid #e5e7eb;border-radius:10px;margin-bottom:8px;box-shadow:0 3px 10px #1018280f}
body.v3-layout .state-row:hover{border-color:#94a3b8;box-shadow:0 6px 16px #10182818}
body.v3-layout .state-row.loaded{background:#f0fdf4}body.v3-layout .state-row.loading{background:#eff6ff}body.v3-layout .state-row.blocked{background:#fff7ed}
body.v3-layout .state-oht{background:#172033;color:#ffffff;border-radius:8px;font-weight:950}
body.v3-layout .state-name{color:#172033!important;background:#f8fafc!important;border:1px solid #e5e7eb;border-radius:999px;padding:4px 8px;font-weight:850}
body.v3-layout .state-sub{color:#334155}
body.v3-layout .state-sub b{color:#64748b}
body.v3-layout .state-sub span{color:#172033}
body.v3-layout .state-panel h2::after{background:#ecfdf3;color:#067647;border-color:#abefc6}
body.v3-layout .state-panel{padding:9px}
body.v3-layout .state-panel h2{font-size:14px;margin-bottom:7px}
body.v3-layout .state-row{padding:8px 9px 8px 10px;margin-bottom:6px}
body.v3-layout .state-main{gap:6px}
body.v3-layout .state-oht{font-size:12px;padding:5px 7px}
body.v3-layout .state-name{font-size:11px;padding:3px 7px}
body.v3-layout .state-sub{grid-template-columns:1fr;gap:4px;font-size:11px}
body.v3-layout .gantt-panel .legend{background:#ffffff;border-bottom:1px solid #e5e7eb;color:#475569}
body.v3-layout .legend span{background:#f8fafc;border-color:#d8e0ea;color:#475569}
body.v3-layout .chart-scroll{background:#ffffff}
body.v3-layout #chart svg{background:#ffffff}
body.v3-layout .row-band{fill:#f4f8fc}
body.v3-layout .row-line{stroke:#dbe4ef;stroke-width:1}
body.v3-layout .axis{stroke:#cbd5e1}
body.v3-layout .tick,.top-tick{fill:#334155}
body.v3-layout .live-task{stroke:#334155;stroke-width:.6;filter:drop-shadow(0 1px 1px #10182820);fill-opacity:.88}
body.v3-layout .live-task.context{opacity:.45;filter:none}
body.v3-layout .live-task.related{opacity:1;stroke:#172033;stroke-width:.9}
body.v3-layout .live-task.updated{stroke:#dc2626;stroke-width:2.2;stroke-dasharray:6 3}
body.v3-layout .live-task.early{stroke:#64748b;stroke-width:1.8;stroke-dasharray:5 3}
body.v3-layout .planned-task{fill-opacity:.82;stroke:#475569;stroke-width:.5}
body.v3-layout .cursor{stroke:#172033;stroke-width:1.5;stroke-dasharray:5 4;opacity:.85}
body.v3-layout #chartLabels{background:linear-gradient(90deg,#ffffff 0,#ffffff 88%,#ffffff00 100%);box-shadow:6px 0 12px #10182814}
body.v3-layout .machine-label-row{color:#172033;background:#ffffff;border-right:1px solid #dbe4ef}
body.v3-layout .machine-label-row:nth-child(even){background:#f4f8fc}
body.v3-layout .machine-label-head{background:#ffffff;border-color:#dbe4ef}
body.v3-layout .gantt-job-filter{color:#334155}
body.v3-layout .gantt-job-filter select{background:#ffffff;color:#172033;border-color:#cbd5e1}
body.v3-layout .mode-toggle{background:#ffffff;border-color:#cbd5e1}
body.v3-layout .mode-toggle button{background:#ffffff;color:#334155;border-color:#e5e7eb}
body.v3-layout .mode-toggle button.active{background:#345995;color:#ffffff}
body.v3-layout .tooltip{background:#172033;border:1px solid #334155;color:#ffffff;box-shadow:0 16px 40px #10182833}
body.v3-layout .oht-select-card{background:#ffffff;border-color:#d8e0ea;color:#172033}
body.v3-layout .oht-select-card h2{color:#172033}.oht-select-card p{color:#475569}.oht-select-card input{background:#ffffff;color:#172033;border-color:#cbd5e1}
</style>
</head>
<body>
<header>
  <div class="brand-header">
    <img class="brand-logo left" src="__PILAB_LOGO__" alt="PILAB logo">
    <div class="brand-center">
      <div class="brand-title-row"><h1>SOMOS OHT Trace Replay</h1></div>
      <div class="controls"><button id="play">Play</button><label>Speed <select id="speed"><option>1</option><option>5</option><option selected>20</option><option>100</option></select>x</label><input id="slider" type="range" min="0" step="0.1"><b class="time" id="timeText">0.0 s</b></div>
    </div>
    <img class="brand-logo right" src="__DONGGUK_LOGO__" alt="Dongguk University logo">
  </div>
</header>
<main>
  <section class="kpi-strip" id="kpiStrip"></section>
  <div class="dashboard">
    <div class="hero">
      <div class="left-column">
        <section class="panel selection-panel"><h2>OHT selection</h2><div class="oht-button-grid" id="ohtButtons"></div><div class="selection-help">Click OHT buttons to select. Click an OHT in the replay for single selection.</div><div class="selection-message" id="selectionMessage"></div></section>
        <section class="panel notify-panel"><h2>Job shift notifications</h2><div class="notify-list" id="shiftNotifications"></div></section>
      </div>
      <section class="panel simulation-panel" id="simulationPanel"><div class="simulation-head"><div class="simulation-title"><h2>OHT simulation replay</h2><span class="trace-mode" id="traceMode"></span><span class="dispatch-mode" id="dispatchMode"></span><small id="traceModeHelp">Wheel to zoom, drag to pan, click an OHT to select</small></div><div class="simulation-tools"><button id="zoomOut" title="Zoom out">−</button><span class="zoom-text" id="zoomText">100%</span><button id="zoomIn" title="Zoom in">+</button><button id="resetZoom">Reset</button><button class="primary" id="fullscreenSimulation">Full screen</button></div></div><div id="simulation"></div></section>
      <div class="side-column">
        <section class="panel state-panel"><h2>Vehicle state / tracking</h2><div class="state-list" id="status"></div></section>
      </div>
    </div>
  </div>
</main>
<div class="gantt-launcher"><button id="openPlannedGantt">Planned Gantt</button><button class="primary" id="openActualGantt">Actual Gantt</button></div>
<div class="gantt-modal" id="ganttModal">
  <section class="gantt-preview"><div class="gantt-preview-head"><span>OHT replay preview</span><div class="preview-tools"><button class="preview-play" id="ganttPreviewPlay">Play</button><button id="previewZoomOut">-</button><span id="previewZoomText">Fit</span><button id="previewZoomIn">+</button><button id="previewFit">Fit</button><span id="ganttPreviewTime">0.0 s</span></div></div><div id="ganttPreviewSimulation"></div></section>
  <section class="panel gantt-panel" id="ganttPanel"><div class="gantt-head"><h2 id="ganttTitle">Machine Gantt V2</h2><div class="gantt-actions"><label class="gantt-job-filter">Job <select id="ganttJobSelect" title="Filter Gantt by selected OHT Job"></select></label><div class="mode-toggle"><button id="ganttPlannedMode">Planned</button><button id="ganttActualMode">Actual</button></div><button id="fullscreenGantt">Full screen</button><button class="gantt-close" id="closeGanttModal">Close</button></div></div><div class="legend" id="legend"></div><div class="chart-scroll"><div class="chart-stage"><div id="chartLabels"></div><div id="chart"></div></div></div></section>
</div>
<div class="tooltip" id="tip"></div>
<script>
"use strict";
const D=__TRACE_DATA__;
const unpackRows=(fields,rows)=>(rows||[]).map(values=>Object.fromEntries(fields.map((field,index)=>[field,values[index]])));
if(D.rowFields)D.rows=unpackRows(D.rowFields,D.rows);
if(D.machineFields)D.machineRows=unpackRows(D.machineFields,D.machineRows);
if(D.plannedMachineFields)D.plannedMachineRows=unpackRows(D.plannedMachineFields,D.plannedMachineRows);
if(D.eventTraceFields)D.eventTrace=unpackRows(D.eventTraceFields,D.eventTrace);
const COLORS={empty_to_source:"#98a2b3",source_wait:"#f79009",loading:"#2e90fa",loaded_travel:"#12b76a",unloading:"#7f56d9",drop_wait:"#f04438"};
const JOB_COLORS=["#5B8DEF","#F59E0B","#EF6F6C","#5EC2B7","#7CB342","#D7B84B","#9B8AD6","#F59CB7","#B08968","#94A3B8","#3B82F6","#F97316","#14B8A6","#A78BFA","#F43F5E","#06B6D4","#EAB308","#8B5CF6","#0EA5E9","#84CC16"];
const $=id=>document.getElementById(id), fmt=n=>Number(n||0).toLocaleString(undefined,{maximumFractionDigits:2});
const esc=s=>String(s??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const jobColor=id=>{const text=String(id??"");let hash=0;for(const ch of text)hash=(hash*31+ch.charCodeAt(0))>>>0;return JOB_COLORS[hash%JOB_COLORS.length]};
const EVENTS_BY_OHT=new Map(),STATE_EVENTS_BY_OHT=new Map(),POSITION_EVENTS_BY_OHT=new Map(),TASK_BY_KEY=new Map();
const TASKS_BY_OHT=new Map();
const POSITION_EVENT_NAMES=new Set(["INIT","EDGE_START","EDGE_END","BLOCK_START","BLOCK_END"]);
function appendEvent(map,id,event){const list=map.get(id)||[];list.push(event);map.set(id,list)}
(D.eventTrace||[]).forEach(e=>{
  const id=String(e.oht_id);
  appendEvent(EVENTS_BY_OHT,id,e);
  if(e.event==="INIT"||e.event==="STATE_CHANGE")appendEvent(STATE_EVENTS_BY_OHT,id,e);
  if(POSITION_EVENT_NAMES.has(e.event))appendEvent(POSITION_EVENTS_BY_OHT,id,e);
});
[EVENTS_BY_OHT,STATE_EVENTS_BY_OHT,POSITION_EVENTS_BY_OHT].forEach(map=>map.forEach(list=>list.sort((a,b)=>Number(a.sim_time)-Number(b.sim_time)||Number(a.seq)-Number(b.seq))));
D.rows.forEach(r=>{
  TASK_BY_KEY.set(`${r.oht_id}|${r.job_instance_id}|${r.step_no}`,r);
  if(r.task_key)TASK_BY_KEY.set(`${r.oht_id}|${r.task_key}`,r);
  if(r.instance_step_key)TASK_BY_KEY.set(`${r.oht_id}|${r.instance_step_key}`,r);
  if(r.lot_step_key)TASK_BY_KEY.set(`${r.oht_id}|lotstep:${r.lot_step_key}`,r);
  const key=String(r.oht_id);
  if(!TASKS_BY_OHT.has(key))TASKS_BY_OHT.set(key,[]);
  TASKS_BY_OHT.get(key).push(r);
});
TASKS_BY_OHT.forEach(rows=>rows.sort((a,b)=>a.dispatch_time-b.dispatch_time));
const ALL_JOBS="__ALL__";
let selected=new Set(), focusedOhts=new Set(), selectedJobInstance=ALL_JOBS, ganttViewMode="actual", current=0, playing=false, lastFrame=0, pinnedTip=false, lastHeavyRenderAt=0, lastSimulationRenderAt=0, lastStatusRenderAt=0, lastKpiRenderAt=0, simulationInitialized=false, chartScale=null;
const SHIFT_NOTE_TTL=100;
const dismissedShiftNotes=new Set();
const TRACK_COLORS=["#dc2626","#2563eb","#16a34a","#f97316","#7c3aed","#0891b2","#db2777","#65a30d","#ea580c","#475569"];
const trackColor=id=>{const text=String(id??"");let hash=0;for(const ch of text)hash=(hash*33+ch.charCodeAt(0))>>>0;return TRACK_COLORS[hash%TRACK_COLORS.length]};
function lotDigits(value){
  const digits=String(value??"").match(/\d+/g);
  return digits?String(Number(digits.join(""))):"";
}
function taskKeys(row){
  const keys=[];
  if(row.task_key)keys.push(String(row.task_key));
  if(row.instance_step_key)keys.push(String(row.instance_step_key));
  const step=String(row.step_no??"");
  const inst=String(row.job_instance_id??"");
  const lot=String(row.lot_id??"");
  if(inst&&step)keys.push(`inst:${inst}|step:${step}`);
  if(lot&&step)keys.push(`lot:${lot}|step:${step}`);
  const digits=lotDigits(lot);
  if(digits&&step)keys.push(`inst:${digits}|step:${step}`);
  if(row.lot_step_key)keys.push(`lotstep:${row.lot_step_key}`);
  return keys;
}
function mapByTaskKeys(rows){
  const map=new Map();
  rows.forEach(row=>taskKeys(row).forEach(key=>{if(!map.has(key))map.set(key,row)}));
  return map;
}
function getByTaskKeys(map,row){
  for(const key of taskKeys(row)){
    if(map.has(key))return map.get(key);
  }
  return null;
}
function machineId(row){
  return String(row?.physical_machine||row?.machine||"").replace(/^M/i,"");
}
function transportDestMatchesMachine(transport,row){
  return !!transport&&String(transport.to_machine||"")===machineId(row);
}
const PLANNED_BY_JOB_STEP=mapByTaskKeys(D.plannedMachineRows);
const ACTUAL_BY_JOB_STEP=mapByTaskKeys(D.machineRows);
const HEAVY_RENDER_INTERVAL_MS=120;
const SIMULATION_RENDER_INTERVAL_MS=100;
const STATUS_RENDER_INTERVAL_MS=500;
const SIM_BOUNDS=D.layout.bounds||[0,0,1,1],SIM_PAD=6;
const SIM_BASE={x:SIM_BOUNDS[0]-SIM_PAD,y:-SIM_BOUNDS[3]-SIM_PAD,w:Math.max(1,SIM_BOUNDS[2]-SIM_BOUNDS[0])+SIM_PAD*2,h:Math.max(1,SIM_BOUNDS[3]-SIM_BOUNDS[1])+SIM_PAD*2};
const DEFAULT_SIM_ZOOM=1.08;
function defaultSimulationCamera(zoom=DEFAULT_SIM_ZOOM){
  const w=SIM_BASE.w/zoom,h=SIM_BASE.h/zoom;
  return{x:SIM_BASE.x+(SIM_BASE.w-w)/2,y:SIM_BASE.y+(SIM_BASE.h-h)/2,w,h};
}
let simCamera=defaultSimulationCamera(),simZoom=DEFAULT_SIM_ZOOM,simDrag=null;
let previewZoom=1;
$("slider").max=Math.max(D.meta.actualCmax,1);
current=0;
$("slider").value=current;
function simViewBox(){return`${simCamera.x} ${simCamera.y} ${simCamera.w} ${simCamera.h}`}
function previewViewBox(){
  const zoom=Math.max(.75,Math.min(4,previewZoom));
  const w=SIM_BASE.w/zoom,h=SIM_BASE.h/zoom;
  const x=SIM_BASE.x+(SIM_BASE.w-w)/2,y=SIM_BASE.y+(SIM_BASE.h-h)/2;
  return`${x} ${y} ${w} ${h}`;
}
function updatePreviewZoomText(){
  const el=$("previewZoomText");
  if(el)el.textContent=previewZoom===1?"Fit":`${Math.round(previewZoom*100)}%`;
}
function setupV3Layout(){
  document.body.classList.add("v3-layout");
  const hero=document.querySelector(".hero");
  const left=document.querySelector(".left-column");
  const side=document.querySelector(".side-column");
  const statePanel=side?.querySelector(".state-panel");
  const simulationPanel=$("simulationPanel");
  const ganttPanel=$("ganttPanel");
  if(left&&statePanel&&!left.contains(statePanel)){
    left.innerHTML="";
    left.appendChild(statePanel);
  }
  const workspace=document.createElement("div");
  workspace.className="main-workspace";
  if(hero&&simulationPanel&&ganttPanel){
    hero.insertBefore(workspace,simulationPanel);
    workspace.appendChild(simulationPanel);
    workspace.appendChild(ganttPanel);
  }
  side?.remove();
  document.querySelector(".gantt-launcher")?.remove();
  document.querySelector(".gantt-preview")?.remove();
  const modal=$("ganttModal");
  if(modal){
    modal.classList.add("open");
    modal.remove();
  }
  const close=$("closeGanttModal");
  close?.remove();
  requestAnimationFrame(()=>{resetSimulationCamera();applySimulationCamera();});
}
function parseOhtInput(text){
  return String(text||"").split(/[,\s]+/).map(item=>item.trim()).filter(Boolean);
}
function showInitialOhtModal(){
  const modal=document.createElement("div");
  modal.className="oht-select-modal open";
  modal.id="ohtSelectModal";
  modal.innerHTML=`<div class="oht-select-card"><h2>Track OHT vehicles</h2><p>Enter OHT IDs before replay starts. Example: <b>1,2,3</b></p><input id="initialOhtInput" type="text" value="${esc([...selected].join(","))}" placeholder="1,2,3"><div class="oht-select-error" id="initialOhtError"></div><div class="oht-select-actions"><button id="initialOhtUseDefault">Use default</button><button class="primary" id="initialOhtApply">Start replay</button></div></div>`;
  document.body.appendChild(modal);
  const input=$("initialOhtInput"),error=$("initialOhtError");
  const close=()=>modal.remove();
  const apply=()=>{
    const ids=parseOhtInput(input.value);
    const available=new Set(D.summaries.map(row=>String(row.oht_id)));
    const invalid=ids.filter(id=>!available.has(String(id)));
    if(!ids.length){error.textContent="Enter at least one OHT ID.";return}
    if(invalid.length){error.textContent=`Unknown OHT ID: ${invalid.join(", ")}`;return}
    setSelectedOhts(ids);
    close();
  };
  $("initialOhtApply").onclick=apply;
  $("initialOhtUseDefault").onclick=close;
  input.onkeydown=event=>{if(event.key==="Enter")apply()};
  input.focus();
  input.select();
}
function updateZoomText(){$("zoomText").textContent=`${Math.round(simZoom*100)}%`}
function applySimulationCamera(){const svg=$("simulation").querySelector("svg");if(svg)svg.setAttribute("viewBox",simViewBox());updateZoomText()}
function zoomSimulation(factor,clientX=null,clientY=null){
  const host=$("simulation"),rect=host.getBoundingClientRect();
  const next=Math.max(1,Math.min(20,simZoom*factor));
  const px=clientX===null ? .5 : Math.max(0,Math.min(1,(clientX-rect.left)/Math.max(1,rect.width)));
  const py=clientY===null ? .5 : Math.max(0,Math.min(1,(clientY-rect.top)/Math.max(1,rect.height)));
  const anchorX=simCamera.x+simCamera.w*px,anchorY=simCamera.y+simCamera.h*py;
  const newW=SIM_BASE.w/next,newH=SIM_BASE.h/next;
  simCamera={x:anchorX-newW*px,y:anchorY-newH*py,w:newW,h:newH};
  simZoom=next;applySimulationCamera();
}
function resetSimulationCamera(){simZoom=DEFAULT_SIM_ZOOM;simCamera=defaultSimulationCamera();applySimulationCamera()}
function bindSimulationNavigation(){
  const host=$("simulation");
  host.onwheel=event=>{event.preventDefault();zoomSimulation(event.deltaY<0?1.2:1/1.2,event.clientX,event.clientY)};
  host.onpointerdown=event=>{if(event.button!==0||event.target.closest(".oht"))return;simDrag={x:event.clientX,y:event.clientY,camera:{...simCamera}};host.classList.add("dragging");host.setPointerCapture(event.pointerId)};
  host.onpointermove=event=>{if(!simDrag)return;const rect=host.getBoundingClientRect();simCamera.x=simDrag.camera.x-(event.clientX-simDrag.x)*simDrag.camera.w/Math.max(1,rect.width);simCamera.y=simDrag.camera.y-(event.clientY-simDrag.y)*simDrag.camera.h/Math.max(1,rect.height);applySimulationCamera()};
  host.onpointerup=host.onpointercancel=event=>{simDrag=null;host.classList.remove("dragging");if(host.hasPointerCapture(event.pointerId))host.releasePointerCapture(event.pointerId)};
}
function bindSideResize(){
  const grip=$("sideResizer"),hero=grip?.parentElement;
  if(!grip||!hero)return;
  let drag=null;
  const clamp=v=>Math.max(260,Math.min(620,v));
  grip.onpointerdown=event=>{
    drag={startX:event.clientX,startWidth:parseFloat(getComputedStyle(hero).getPropertyValue("--side-width"))||390};
    grip.classList.add("dragging");
    document.body.classList.add("resizing-side");
    grip.setPointerCapture(event.pointerId);
  };
  grip.onpointermove=event=>{
    if(!drag)return;
    const next=clamp(drag.startWidth-(event.clientX-drag.startX));
    hero.style.setProperty("--side-width",`${next}px`);
    requestAnimationFrame(applySimulationCamera);
  };
  grip.onpointerup=grip.onpointercancel=event=>{
    drag=null;
    grip.classList.remove("dragging");
    document.body.classList.remove("resizing-side");
    if(grip.hasPointerCapture(event.pointerId))grip.releasePointerCapture(event.pointerId);
    applySimulationCamera();
  };
}
function refreshJobOptions(){
  const jobs=new Map();
  D.rows.filter(r=>selected.has(String(r.oht_id))).forEach(r=>{
    const instance=String(r.job_instance_id||"");
    if(instance)jobs.set(instance,{instance,jobId:String(r.job_id||r.lot_id||"")});
  });
  if(selectedJobInstance&&selectedJobInstance!==ALL_JOBS&&!jobs.has(String(selectedJobInstance))){
    const row=[...D.machineRows,...D.plannedMachineRows].find(r=>String(r.job_instance_id||"")===String(selectedJobInstance));
    jobs.set(String(selectedJobInstance),{instance:String(selectedJobInstance),jobId:String(row?.job_id||row?.lot_id||"clicked")});
  }
  const ordered=[...jobs.values()].sort((a,b)=>Number(a.instance)-Number(b.instance));
  if(!ordered.length)selectedJobInstance=ALL_JOBS;
  if(selectedJobInstance!==ALL_JOBS&&!jobs.has(String(selectedJobInstance)))selectedJobInstance=ordered.length?ALL_JOBS:null;
  const html=ordered.length
    ?`<option value="${ALL_JOBS}" ${selectedJobInstance===ALL_JOBS?"selected":""}>All Jobs carried by selected OHTs (${ordered.length})</option>`+
      ordered.map(job=>`<option value="${esc(job.instance)}" ${job.instance===String(selectedJobInstance)?"selected":""}>Job ${esc(job.jobId)} / Instance ${esc(job.instance)}</option>`).join("")
    :`<option value="${ALL_JOBS}" selected>All machine jobs</option>`;
  const ganttJobSelect=$("ganttJobSelect");
  if(ganttJobSelect)ganttJobSelect.innerHTML=html;
}
function sortedOhtIds(){
  return D.summaries.map(row=>String(row.oht_id)).sort((a,b)=>{
    const an=Number(a),bn=Number(b);
    if(Number.isFinite(an)&&Number.isFinite(bn)&&an!==bn)return an-bn;
    return a.localeCompare(b);
  });
}
function refreshOhtButtons(){
  if(!$("ohtButtons"))return;
  $("ohtButtons").innerHTML=sortedOhtIds().map(id=>`<button type="button" class="oht-choice ${selected.has(id)?"active":""}" data-oht-choice="${esc(id)}">OHT ${esc(id)}</button>`).join("");
  $("ohtButtons").querySelectorAll(".oht-choice").forEach(button=>{
    button.onclick=()=>toggleOhtSelection(button.dataset.ohtChoice);
  });
}
function setSelectedOhts(ids){
  const next=new Set(ids.map(String).filter(Boolean));
  selected=next;
  if($("selectionMessage"))$("selectionMessage").textContent="";
  refreshOhtButtons();
  refreshJobOptions();
  renderAll();
}
function toggleOhtSelection(id){
  const next=new Set(selected);
  if(next.has(id)){
    next.delete(id);
  }else{
    next.add(id);
  }
  setSelectedOhts([...next]);
}
function selectSingleOht(id){
  toggleOhtSelection(id);
}
function buildSelection(){
  $("traceMode").textContent=D.meta.hasExactTrace?"EXACT TRACE":"APPROXIMATE";
  $("traceMode").className=`trace-mode ${D.meta.hasExactTrace?"exact":"fallback"}`;
  $("dispatchMode").textContent=`Dispatch mode: ${D.meta.dispatchMode||"unknown"}`;
  $("traceModeHelp").textContent=D.meta.hasExactTrace
    ?""
    :"No oht_event_trace.csv: blocking is not reproduced; transport paths are interpolated";
  refreshOhtButtons();
  refreshJobOptions();
  $("ganttJobSelect").addEventListener("change",event=>applyJobFilter(event.target.value));
}
function applyJobFilter(value){
  selectedJobInstance=value||null;
  refreshJobOptions();
  renderHeavy(true);
  renderShiftNotifications();
}
function toggleJobFilterFromTask(row){
  const instance=String(row?.job_instance_id||"");
  if(!instance)return;
  selectedJobInstance=String(selectedJobInstance)===instance?ALL_JOBS:instance;
  refreshJobOptions();
  pinnedTip=false;
  $("tip").style.display="none";
  renderHeavy(true);
}
function routePoint(route,progress){
  const points=(route||[]).map(name=>D.layout.nodes[name]).filter(Boolean);
  if(!points.length)return null;
  if(points.length===1)return points[0];
  const lengths=[],total=points.slice(1).reduce((sum,p,i)=>{const a=points[i],d=Math.hypot(p.x-a.x,p.y-a.y);lengths.push(d);return sum+d},0);
  let remaining=Math.max(0,Math.min(1,progress))*total;
  for(let i=0;i<lengths.length;i++){if(remaining<=lengths[i]||i===lengths.length-1){const a=points[i],b=points[i+1],f=lengths[i]?remaining/lengths[i]:0;return{x:a.x+(b.x-a.x)*f,y:a.y+(b.y-a.y)*f}}remaining-=lengths[i]}
  return points[points.length-1];
}
function taskAtTime(id,atTime){
  const tasks=TASKS_BY_OHT.get(String(id))||[];
  for(const r of tasks){
    if(r.dispatch_time<=atTime&&atTime<=r.dropoff_time)return r;
    if(r.dispatch_time>atTime)break;
  }
  return null;
}
function latestEventAt(events,atTime){
  let lo=0,hi=events.length-1,index=-1;
  while(lo<=hi){const mid=(lo+hi)>>1;if(Number(events[mid].sim_time)<=atTime){index=mid;lo=mid+1}else hi=mid-1}
  return index>=0?events[index]:null;
}
function eventState(e){
  if(!e)return "";
  if(e.event==="BLOCK_START")return "Waiting";
  if(e.event==="SERVICE_WAIT_START")return "Service wait";
  if(e.event==="LOAD_START"||e.event==="LOAD_END"||e.event==="PICKUP")return "Loading";
  if(e.event==="UNLOAD_START"||e.event==="UNLOAD_END"||e.event==="DROPOFF")return "Unloading";
  if(e.event==="TASK_ASSIGNED"||e.event==="DISPATCH"||e.event==="REASSIGN")return e.state||"Dispatched";
  if(e.new_state)return e.new_state;
  if(e.state)return e.state;
  return "";
}
function ohtSnapshot(id,atTime=current){
  const key=String(id),events=EVENTS_BY_OHT.get(key)||[];
  const currentEvent=latestEventAt(events,atTime);
  if(currentEvent){
    if(currentEvent.event==="BLOCK_END"){
      return fallbackOhtSnapshot(id,atTime);
    }
    const positionEvent=latestEventAt(POSITION_EVENTS_BY_OHT.get(key)||[],atTime)||currentEvent;
    const stateEvent=latestEventAt(STATE_EVENTS_BY_OHT.get(key)||[],atTime);
    const start=Number(positionEvent.sim_time)||0,end=Number(positionEvent.end_time)||start;
    let point={x:Number(positionEvent.x)||0,y:Number(positionEvent.y)||0};
    if(positionEvent.event==="EDGE_START"&&end>start&&atTime<end){
      const f=Math.max(0,Math.min(1,(atTime-start)/(end-start)));
      point={x:point.x+((Number(positionEvent.x1)||point.x)-point.x)*f,y:point.y+((Number(positionEvent.y1)||point.y)-point.y)*f};
    }else if(positionEvent.event==="EDGE_START"&&end>start&&atTime>=end){
      point={x:Number(positionEvent.x1)||point.x,y:Number(positionEvent.y1)||point.y};
    }else if(positionEvent.event==="EDGE_END"){
      point={x:Number(positionEvent.x1)||point.x,y:Number(positionEvent.y1)||point.y};
    }
    let state=eventState(currentEvent)||eventState(stateEvent)||eventState(positionEvent)||"Empty";
    if(positionEvent.event==="EDGE_START"&&currentEvent.event==="EDGE_START"){
      state=state==="Loaded"?"Loaded travel":"Empty travel";
    }
    const eventStep=String(currentEvent.step_no||currentEvent.op_index||"");
    const eventInst=String(currentEvent.job_instance_id||"");
    const eventLot=String(currentEvent.lot_id||"");
    const task=TASK_BY_KEY.get(`${id}|inst:${eventInst}|step:${eventStep}`)
      ||TASK_BY_KEY.get(`${id}|lot:${eventLot}|step:${eventStep}`)
      ||TASK_BY_KEY.get(`${id}|${eventInst}|${eventStep}`)
      ||taskAtTime(id,atTime)
      ||null;
    return{point,state,task,event:currentEvent,stateEvent,positionEvent};
  }
  return fallbackOhtSnapshot(id,atTime);
}
function fallbackOhtSnapshot(id,atTime=current){
  const tasks=TASKS_BY_OHT.get(String(id))||[];
  if(!tasks.length)return null;
  let last=D.layout.nodes[tasks[0].from_node]||null,state="Idle",task=null;
  for(const r of tasks){
    if(atTime<r.dispatch_time)break;
    task=r;
    if(atTime<=r.source_arrival_time){
      const span=Math.max(.001,r.source_arrival_time-r.dispatch_time);
      return{point:routePoint(r.empty_route,(atTime-r.dispatch_time)/span),state:"Empty travel",task:r,event:null};
    }
    last=D.layout.nodes[r.from_node]||last;
    if(atTime<r.load_start_time)return{point:last,state:"Source wait",task:r,event:null};
    if(atTime<=r.load_end_time)return{point:last,state:"Loading",task:r,event:null};
    if(atTime<=r.dest_arrival_time){
      const span=Math.max(.001,r.dest_arrival_time-r.loaded_travel_start_time);
      return{point:routePoint(r.loaded_route,(atTime-r.loaded_travel_start_time)/span),state:"Loaded travel",task:r,event:null};
    }
    last=D.layout.nodes[r.to_node]||last;
    if(atTime<=r.unload_end_time)return{point:last,state:"Unloading",task:r,event:null};
    if(atTime<=r.dropoff_time)return{point:last,state:"Drop wait",task:r,event:null};
    state="Idle";
  }
  return{point:last,state,task,event:null};
}
function trailPoints(id){
  const points=[];
  for(let t=Math.max(0,current-3);t<=current+.001;t+=.25){
    const point=ohtSnapshot(id,t)?.point;
    if(point&&!points.some(p=>Math.abs(p.x-point.x)<.001&&Math.abs(p.y-point.y)<.001))points.push(point);
  }
  const point=ohtSnapshot(id,current)?.point;
  if(point&&(points.length===0||points[points.length-1].x!==point.x||points[points.length-1].y!==point.y))points.push(point);
  return points;
}
function initSimulation(){
  if(simulationInitialized)return;
  const L=D.layout,b=L.bounds||[0,0,1,1],pad=18,width=Math.max(1,b[2]-b[0]),height=Math.max(1,b[3]-b[1]);
  let svg=`<svg viewBox="${simViewBox()}" preserveAspectRatio="xMidYMid meet"><g transform="scale(1,-1)">`;
  L.edges.forEach(e=>{const a=L.nodes[e.from],z=L.nodes[e.to];if(a&&z)svg+=`<line class="rail" x1="${a.x}" y1="${a.y}" x2="${z.x}" y2="${z.y}"/>`});
  Object.entries(L.nodes).forEach(([name,n])=>{if(name.startsWith("V_"))svg+=`<circle class="station" cx="${n.x}" cy="${n.y}" r=".8"/>`});
  L.machines.forEach(m=>{svg+=`<rect class="machine-box" data-machine="${esc(m.machine)}" x="${m.x-4.25}" y="${m.y-2.75}" width="8.5" height="5.5" rx=".6"/><g transform="translate(${m.x} ${m.y}) scale(1,-1)"><text class="machine-label" y="1.7">M${esc(m.machine)}</text></g>`});
  svg+=`<g id="trailLayer"></g><g id="ohtLayer"></g>`;
  svg+="</g></svg>";
  $("simulation").innerHTML=svg;
  $("simulation").onclick=event=>{
    const target=event.target.closest(".oht");
    if(target)toggleOhtSelection(target.dataset.oht);
  };
  bindSimulationNavigation();
  simulationInitialized=true;
  updateZoomText();
}
function syncGanttPreview(){
  if(!$("ganttModal")||!$("ganttModal").classList.contains("open"))return;
  const source=$("simulation").querySelector("svg");
  if(source){
    $("ganttPreviewSimulation").innerHTML=source.outerHTML;
    const previewSvg=$("ganttPreviewSimulation").querySelector("svg");
    if(previewSvg){
      previewSvg.setAttribute("viewBox",previewViewBox());
      previewSvg.setAttribute("preserveAspectRatio","xMidYMid meet");
    }
  }
  updatePreviewZoomText();
  $("ganttPreviewTime").textContent=fmt(current)+" s";
}
function renderSimulation(){
  initSimulation();
  const activeMachines=new Set(D.machineRows.filter(r=>r.start<=current&&current<=r.end).map(r=>String(r.physical_machine)));
  $("simulation").querySelectorAll(".machine-box").forEach(el=>el.classList.toggle("processing",activeMachines.has(String(el.dataset.machine))));
  let trails="";
  selected.forEach(id=>{const points=trailPoints(id),color=trackColor(id);if(points.length>1)trails+=`<polyline class="oht-trail" points="${points.map(p=>`${p.x},${p.y}`).join(" ")}" stroke="${color}"/>`});
  $("trailLayer").innerHTML=trails;
  let ohtSvg="";
  D.summaries.forEach(s=>{const snap=ohtSnapshot(s.oht_id);if(!snap||!snap.point)return;const id=String(s.oht_id),isSelected=selected.has(id),isFocused=focusedOhts.has(id),isBlocked=snap.state==="Waiting"||snap.event?.event==="BLOCK_START",color=trackColor(id),fill=isSelected?color:(isBlocked?"#f59e0b":snap.state==="Loaded travel"||snap.state==="Unloading"?"#32cd32":snap.state==="Loading"?"#00bfff":"#fff");if(isSelected)ohtSvg+=`<circle class="oht-halo" cx="${snap.point.x}" cy="${snap.point.y}" r="4.2" stroke="${color}"/>`;if(isFocused)ohtSvg+=`<circle class="oht-focus-ring" cx="${snap.point.x}" cy="${snap.point.y}" r="8.8" stroke="${color}" fill="${color}"/>`;ohtSvg+=`<circle class="oht ${isSelected?"selected":""} ${isBlocked?"blocked":""}" data-oht="${esc(s.oht_id)}" cx="${snap.point.x}" cy="${snap.point.y}" r="${isSelected||isBlocked?2.7:1.8}" fill="${fill}"/><g transform="translate(${snap.point.x} ${snap.point.y}) scale(1,-1)"><text class="oht-label" y="-3.5">${esc(s.oht_id)}</text></g>`});
  $("ohtLayer").innerHTML=ohtSvg;
  syncGanttPreview();
}
function tooltip(r,p){
  return `OHT ID: ${r.oht_id}\nLot: ${r.lot_id}\nStep: ${r.step_no}\nMachine: ${r.from_machine} -> ${r.to_machine}\nNode: ${r.from_node} -> ${r.to_node}\nPhase: ${p.name}\nStart: ${fmt(p.start)}\nEnd: ${fmt(p.end)}\nDuration: ${fmt(p.duration)}\nPlanned travel: ${fmt(r.planned_travel)}\nActual transport: ${fmt(r.actual_transport_time)}\nDeviation: ${fmt(r.transport_deviation)}\nBlocking loaded: ${fmt(r.blocking_loaded_time)}\nDetour ratio: ${fmt(r.detour_ratio)}`;
}
function machineTooltip(r){
  const trigger=r.triggerTransport
    ?`\nUpdate OHT: ${r.triggerTransport.oht_id}\nDropoff trigger: ${fmt(r.triggerTransport.dropoff_time)}\nTransport: M${r.triggerTransport.from_machine} -> M${r.triggerTransport.to_machine}`
    :"";
  const actual=`${r.start!==undefined?`\nActual final: ${fmt(r.start)} - ${fmt(r.end)}`:""}`;
  return `Gantt machine: ${r.machine}\nPhysical machine: ${r.physical_machine||r.machine}${r.machine_name?` (${r.machine_name})`:""}\nJob: ${r.job_id}\nJob instance: ${r.job_instance_id||"-"}\nStep: ${r.step_no}\nPlanned: ${fmt(r.plannedStart)} - ${fmt(r.plannedEnd)}\nDisplayed: ${fmt(r.displayStart)} - ${fmt(r.displayEnd)}${actual}\nDelay: ${fmt(r.currentDelay)}\nStatus: ${r.displayState}${trigger}`;
}
function chartX(t){
  if(!chartScale)return null;
  return chartScale.left+Math.max(0,Math.min(chartScale.max,t))/chartScale.max*chartScale.plotW;
}
function updateChartCursor(){
  const xx=chartX(current);
  if(xx===null)return;
  $("chart").querySelectorAll(".cursor").forEach(line=>{
    line.setAttribute("x1",xx);
    line.setAttribute("x2",xx);
  });
  autoScrollGanttToCursor(false);
}
function autoScrollGanttToCursor(force=false){
  const scroll=document.querySelector("#ganttPanel .chart-scroll"),chart=$("chart");
  if(!scroll||!chart||!chartScale)return;
  const paddingLeft=parseFloat(getComputedStyle(chart).paddingLeft)||0;
  const labelWidth=$("chartLabels")?.offsetWidth||0;
  const cursorX=paddingLeft+chartX(current);
  const visibleLeft=scroll.scrollLeft;
  const visibleRight=scroll.scrollLeft+scroll.clientWidth;
  const safeLeft=visibleLeft+labelWidth+90;
  const safeRight=visibleRight-170;
  if(force||cursorX<safeLeft||cursorX>safeRight){
    const target=Math.max(0,Math.min(scroll.scrollWidth-scroll.clientWidth,cursorX-scroll.clientWidth*.34));
    scroll.scrollTo({left:target,behavior:playing?"auto":"smooth"});
  }
}
function statusWaitDuration(snap,r,e){
  if(e&&e.wait_duration!==null&&e.wait_duration!=="")return `${fmt(e.wait_duration)} s`;
  if(!r)return"-";
  if(snap?.state==="Source wait")return `${fmt(r.source_wait_time)} s`;
  if(snap?.state==="Drop wait")return `${fmt(r.drop_wait_time)} s`;
  if(snap?.state==="Loaded travel"&&r.blocking_loaded_time)return `${fmt(r.blocking_loaded_time)} s`;
  return "-";
}
function renderChart(){
  $("ganttPlannedMode").classList.toggle("active",ganttViewMode==="planned");
  $("ganttActualMode").classList.toggle("active",ganttViewMode==="actual");
  const showAll=selectedJobInstance===ALL_JOBS;
  const selectedTransports=D.rows.filter(r=>
    selected.has(String(r.oht_id))&&
    (showAll||String(r.job_instance_id)===String(selectedJobInstance))
  );
  const selectedJobSet=new Set(selectedTransports.map(r=>String(r.job_instance_id)));
  const impactedJobSet=selectedJobSet.size?selectedJobSet:new Set();
  const firstTransportByTask=new Map();
  selectedTransports
    .sort((a,b)=>Number(a.dropoff_time)-Number(b.dropoff_time))
    .forEach(t=>{
      taskKeys(t).forEach(key=>{if(!firstTransportByTask.has(key))firstTransportByTask.set(key,t)});
    });
  const actualSource=showAll
    ?D.machineRows
    :D.machineRows.filter(r=>String(r.job_instance_id)===String(selectedJobInstance));
  const actualTasks=actualSource.map(r=>{
    const planned=getByTaskKeys(PLANNED_BY_JOB_STEP,r)||{start:r.start,end:r.end,duration:r.duration,machine:r.machine};
    const rowKeys=new Set(taskKeys(r));
    const triggerTransport=selectedTransports
      .filter(t=>transportDestMatchesMachine(t,r)&&taskKeys(t).some(key=>rowKeys.has(key)))
      .sort((a,b)=>Number(a.dropoff_time)-Number(b.dropoff_time))[0]||null;
    const updateTransport=selectedTransports
      .filter(t=>
        String(t.job_instance_id)===String(r.job_instance_id)&&
        Number(t.step_no)<=Number(r.step_no)&&
        Number(t.dropoff_time)<=Number(r.end)
      )
      .sort((a,b)=>Number(b.dropoff_time)-Number(a.dropoff_time))[0]||null;
    const currentDelay=r.start-planned.start;
    const related=selectedJobSet.has(String(r.job_instance_id));
    const displayState=triggerTransport?"selected OHT transport":related?"same selected-OHT job":"full machine schedule";
    return{...r,plannedStart:planned.start,plannedEnd:planned.end,plannedDuration:planned.duration,displayStart:r.start,displayEnd:r.end,currentDelay,displayState,triggerTransport,updateTransport,related,cascadeRank:0};
  });
  const cascadeGroups=new Map();
  actualTasks.filter(r=>r.related&&r.updateTransport).forEach(r=>{
    const key=`${r.job_instance_id}|${r.updateTransport.dropoff_time}`;
    if(!cascadeGroups.has(key))cascadeGroups.set(key,[]);
    cascadeGroups.get(key).push(r);
  });
  cascadeGroups.forEach(group=>{
    group
      .sort((a,b)=>
        Number(a.machine)-Number(b.machine)||
        Number(a.step_no)-Number(b.step_no)||
        Number(a.start)-Number(b.start)
      )
      .forEach((r,i)=>{r.cascadeRank=i;});
  });
  const plannedSource=ganttViewMode==="planned"
    ?(!showAll?D.plannedMachineRows.filter(r=>String(r.job_instance_id)===String(selectedJobInstance)):D.plannedMachineRows)
    :[];
  const tasks=ganttViewMode==="planned"
    ?plannedSource.map((r,index)=>({...r,_row_number:`p${index}`,job_instance_id:r.job_instance_id||"",physical_machine:r.machine,machine_name:"",plannedStart:r.start,plannedEnd:r.end,plannedDuration:r.duration,displayStart:r.start,displayEnd:r.end,currentDelay:0,displayState:"planned schedule",triggerTransport:null,related:true}))
    :actualTasks;
  if(!tasks.length){$("chartLabels").innerHTML="";$("chart").innerHTML='<div class="empty">No machine Gantt rows for the selected Job.</div>';return}
  const shiftedCount=actualTasks.filter(r=>Math.abs(r.currentDelay)>0.001).length;
  const transportCount=actualTasks.filter(r=>r.triggerTransport).length;
  $("ganttTitle").textContent=ganttViewMode==="planned"
    ?`Planned machine Gantt${showAll?"":` - Job instance ${selectedJobInstance}`}`
    :`Actual machine Gantt${showAll?"":` - Job instance ${selectedJobInstance}`}`;
  const machines=[...new Set(tasks.map(r=>String(r.machine)))].sort((a,b)=>Number(a)-Number(b));
  const machineIndex=new Map(machines.map((m,i)=>[m,i]));
  const left=24,right=42,top=38,rowH=34,max=Math.max(D.meta.maxTime,D.meta.actualCmax,1);
  const finalCleanMode=current>=max-0.5;
  const plotW=Math.max(3600,max*.62),width=left+plotW+right,height=top+machines.length*rowH+42;
  $("chartLabels").style.height=`${height}px`;
  $("chartLabels").innerHTML=`<div class="machine-label-head"></div>`+machines.map((machine,i)=>
    `<div class="machine-label-row" style="top:${top+i*rowH-2}px">M${esc(machine)}</div>`
  ).join("");
  chartScale={left,plotW,max};
  const x=t=>left+Math.max(0,Math.min(max,t))/max*plotW;
  let svg=`<svg width="${width}" height="${height}" viewBox="0 0 ${width} ${height}"><defs><marker id="delayArrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#dc2626"/></marker></defs>`;
  svg+=`<rect x="${left}" y="0" width="${plotW}" height="${top-6}" fill="#eef4fb" opacity="1"/><line class="axis" x1="${left}" x2="${left+plotW}" y1="${top-6}" y2="${top-6}"/>`;
  const tickStep=300;
  for(let t=0;t<=max;t+=tickStep){const xx=x(t);svg+=`<line class="axis" x1="${xx}" y1="${top-6}" x2="${xx}" y2="${height-22}" opacity=".18"/><text class="tick top-tick" x="${xx}" y="${top-16}" text-anchor="middle">${fmt(t)} s</text><text class="tick" x="${xx}" y="${height-7}" text-anchor="middle">${fmt(t)}</text>`}
  machines.forEach((machine,i)=>{
    const y=top+i*rowH;
    if(i%2===0)svg+=`<rect class="row-band" x="${left}" y="${y-2}" width="${plotW}" height="${rowH}"/>`;
    svg+=`<line class="row-line" x1="${left}" x2="${left+plotW}" y1="${y+rowH-2}" y2="${y+rowH-2}"/>`;
    svg+=`<text class="row-label" x="${left-12}" y="${y+21}" text-anchor="end">M${esc(machine)}</text>`;
  });
  tasks.forEach(r=>{
    const y=top+machineIndex.get(String(r.machine))*rowH;
    const plannedX=x(r.plannedStart),displayX=x(r.displayStart),color=jobColor(r.job_instance_id||r.job_id);
    if(ganttViewMode==="planned"){
      const plannedW=Math.max(1,x(r.plannedEnd)-plannedX);
      svg+=`<rect class="planned-task live-task planned-view" data-mi="${r._row_number}" x="${plannedX}" y="${y+9}" width="${plannedW}" height="16" rx="3" fill="${color}"/>`;
      return;
    }
    const transportStart=r.triggerTransport?Number(r.triggerTransport.load_start_time):NaN;
    const transportEnd=r.triggerTransport?Number(r.triggerTransport.dropoff_time):NaN;
    const activeTransport=r.triggerTransport&&current>=transportStart&&current<=transportEnd;
    if(current<Number(r.displayStart)){
      if(activeTransport){
        const ohtColor=trackColor(r.triggerTransport.oht_id);
        const sx=x(transportStart),ex=x(Math.min(current,transportEnd));
        const markerX=Math.max(sx+8,ex);
        const badgeW=Math.max(48,`OHT ${r.triggerTransport.oht_id}`.length*7.2);
        svg+=`<rect class="oht-transport-segment" x="${sx}" y="${y+13}" width="${Math.max(2,ex-sx)}" height="9" rx="4.5" fill="${ohtColor}" stroke="${ohtColor}"/>`;
        svg+=`<text class="oht-task-label" x="${sx+5}" y="${y+8}" fill="${ohtColor}">transport to M${esc(r.triggerTransport.to_machine)}</text>`;
        svg+=`<rect class="oht-badge moving" x="${markerX-badgeW/2}" y="${y-10}" width="${badgeW}" height="18" rx="9"/><text class="oht-job-marker" x="${markerX}" y="${y-.8}">OHT ${esc(r.triggerTransport.oht_id)}</text>`;
      }
      return;
    }
    if(current<Number(r.displayStart)&&activeTransport){
      const fullActualW=Math.max(18,x(r.displayEnd)-displayX);
      const transportProgress=Math.max(0,Math.min(1,(current-transportStart)/Math.max(.001,transportEnd-transportStart)));
      const labelX=displayX+Math.max(8,Math.min(fullActualW-4,fullActualW*transportProgress));
      const badgeW=Math.max(48,`OHT ${r.triggerTransport.oht_id}`.length*7.2);
      const jobLabel=`J${r.job_id}`;
      const jobLabelW=Math.max(26,Math.min(72,jobLabel.length*5.4+8));
      svg+=`<rect class="transport-window" data-mi="${r._row_number}" x="${displayX}" y="${y+8}" width="${fullActualW}" height="18" rx="4"/>`;
      svg+=`<text class="transport-guide-label" x="${displayX+5}" y="${y+5}">loading → loaded transport to M${esc(r.triggerTransport.to_machine)}</text>`;
      svg+=`<rect class="oht-job-label-bg" x="${displayX+fullActualW/2-jobLabelW/2}" y="${y+26}" width="${jobLabelW}" height="13" rx="6"/><text class="oht-job-label" x="${displayX+fullActualW/2}" y="${y+32.8}">${esc(jobLabel)}</text>`;
      svg+=`<rect class="oht-badge moving" x="${labelX-badgeW/2}" y="${y-10}" width="${badgeW}" height="18" rx="9"/><text class="oht-job-marker" x="${labelX}" y="${y-.8}">OHT ${esc(r.triggerTransport.oht_id)}</text>`;
      return;
    }
    const cascadeDelay=(r.related&&r.updateTransport)?Math.min(3.6,Number(r.cascadeRank||0)*0.18):0;
    const revealTime=Math.max(Number(r.displayStart),r.updateTransport?Number(r.updateTransport.dropoff_time)+cascadeDelay:Number(r.displayStart));
    if(current<revealTime)return;
    const stagedCurrent=Math.max(Number(r.displayStart),current-cascadeDelay);
    const displayVisibleEnd=Math.min(Number(r.displayEnd),stagedCurrent);
    const displayW=Math.max(1,x(displayVisibleEnd)-displayX);
    const isBuilding=stagedCurrent<Number(r.displayEnd);
    const updateAge=current-revealTime;
    const recentUpdate=!finalCleanMode&&r.related&&updateAge>=0&&updateAge<30.0&&Math.abs(displayX-plannedX)>2;
    const liftShiftActive=recentUpdate&&updateAge<2.0;
    const delayClass=finalCleanMode?"":(Math.abs(r.currentDelay)>0.001?(r.currentDelay>0?"updated":"early"):"");
    const relationClass=r.related?"related":"context";
    const showShiftArrow=r.triggerTransport&&Number(r.triggerTransport.dropoff_time)<=current&&recentUpdate;
    if(recentUpdate){
      const plannedW=Math.max(1,x(r.plannedEnd)-plannedX);
      const fullActualW=Math.max(1,x(r.displayEnd)-displayX);
      const visibleActualW=liftShiftActive?fullActualW:Math.max(displayW,Math.min(fullActualW,displayW+8));
      const fromCx=plannedX+plannedW/2;
      const toCx=displayX+visibleActualW/2;
      const sweepY=y+12;
      const shiftText=`UPDATED ${r.currentDelay>=0?"+":""}${fmt(r.currentDelay)}s`;
      const labelX=Math.max(left+8,Math.min(left+plotW-90,(fromCx+toCx)/2+8));
      if(liftShiftActive){
        const t=Math.max(0,Math.min(1,updateAge/2.0));
        const ease=t<.5?4*t*t*t:1-Math.pow(-2*t+2,3)/2;
        const lift=Math.sin(Math.PI*t)*13;
        const movingX=plannedX+(displayX-plannedX)*ease;
        const movingY=y+9-lift;
        const movingW=Math.max(2,plannedW+(fullActualW-plannedW)*ease);
        const phase=t<.28?"LIFT":(t<.78?"SHIFT":"DROP");
        svg+=`<g class="update-shift lift-shift"><rect class="update-from" x="${plannedX}" y="${y+6}" width="${plannedW}" height="22" rx="4"/><rect class="update-to" x="${displayX}" y="${y+6}" width="${fullActualW}" height="22" rx="4"/><line class="lift-guide" x1="${fromCx}" y1="${sweepY+5}" x2="${toCx}" y2="${sweepY+5}"/><rect class="lift-shadow" x="${movingX+3}" y="${y+26}" width="${movingW}" height="4" rx="2"/><rect class="lifted-task" data-mi="${r._row_number}" x="${movingX}" y="${movingY}" width="${movingW}" height="16" rx="4" fill="${color}"/><text class="lift-label" x="${Math.max(left+10,Math.min(left+plotW-90,movingX+movingW/2-24))}" y="${movingY-4}">${phase} ${esc(shiftText)}</text></g>`;
        return;
      }
      svg+=`<g class="update-shift"><rect class="update-from" x="${plannedX}" y="${y+6}" width="${plannedW}" height="22" rx="4"/><line class="update-sweep" x1="${fromCx}" y1="${sweepY+5}" x2="${toCx}" y2="${sweepY+5}"/><rect class="update-to" x="${displayX}" y="${y+6}" width="${Math.max(2,displayW)}" height="22" rx="4"/><text class="update-label" x="${labelX}" y="${y+3}">${esc(shiftText)}</text></g>`;
    }
    if(showShiftArrow){
      const arrowY=y+31;
      const arrowStart=displayX>=plannedX?plannedX+4:plannedX-4;
      const arrowEnd=displayX>=plannedX?displayX-4:displayX+4;
      const directionText=displayX>=plannedX?"delayed":"earlier";
      const byText=r.triggerTransport?` by OHT ${esc(r.triggerTransport.oht_id)}`:"";
      const delayText=`${directionText} ${fmt(Math.abs(r.currentDelay))}s${byText}`;
      const labelX=Math.max(plannedX,displayX)+8;
      svg+=`<g class="delay-shift"><line class="delay-shift-line" x1="${arrowStart}" y1="${arrowY}" x2="${arrowEnd}" y2="${arrowY}"/><text class="delay-shift-label" x="${labelX}" y="${arrowY+3}">${esc(delayText)}</text></g>`;
    }
    svg+=`<rect class="live-task ${relationClass} ${delayClass} ${isBuilding?"building":""}" data-mi="${r._row_number}" x="${displayX}" y="${y+9}" width="${displayW}" height="16" rx="3" fill="${color}"/>`;
    if(r.triggerTransport&&displayW>4){
      const ohtColor=trackColor(r.triggerTransport.oht_id);
      const ohtLabelX=displayX+Math.min(displayW-5,Math.max(10,displayW*.18));
      svg+=`<rect class="oht-task-outline" x="${displayX-1.5}" y="${y+6.5}" width="${Math.max(3,displayW+3)}" height="21" rx="5" stroke="${ohtColor}"/>`;
      svg+=`<line class="oht-task-stripe" x1="${displayX+3}" y1="${y+28.5}" x2="${displayX+Math.max(3,displayW-3)}" y2="${y+28.5}" stroke="${ohtColor}"/>`;
      if(displayW>42)svg+=`<text class="oht-task-label" x="${ohtLabelX}" y="${y+36}" fill="${ohtColor}">OHT ${esc(r.triggerTransport.oht_id)}</text>`;
    }
    if(!finalCleanMode&&r.related&&current-revealTime>=0&&current-revealTime<1.6){
      const stoneX=displayX+Math.min(displayW,8);
      svg+=`<circle class="cascade-ring" cx="${stoneX}" cy="${y+17}" r="${5+Math.max(0,1.6-(current-revealTime))*2}"/><circle class="cascade-stone" cx="${stoneX}" cy="${y+17}" r="4"/>`;
    }
    if(!finalCleanMode&&r.related&&displayW>10){
      const jobLabel=`J${r.job_id}`;
      const jobLabelW=Math.max(26,Math.min(72,jobLabel.length*5.4+8));
      const jobLabelX=displayX+Math.min(displayW-4,Math.max(8,displayW/2));
      svg+=`<rect class="oht-job-label-bg" x="${jobLabelX-jobLabelW/2}" y="${y-5}" width="${jobLabelW}" height="13" rx="6"/><text class="oht-job-label" x="${jobLabelX}" y="${y+1.8}">${esc(jobLabel)}</text>`;
    }
    if(!finalCleanMode&&r.triggerTransport&&current>=Number(r.triggerTransport.load_start_time)&&current<=Number(r.triggerTransport.dropoff_time)){
      const transportStart=Number(r.triggerTransport.load_start_time);
      const transportEnd=Number(r.triggerTransport.dropoff_time);
      const fullActualW=Math.max(1,x(r.displayEnd)-displayX);
      const currentX=x(current);
      const labelX=Math.max(displayX+8,Math.min(displayX+displayW-4,currentX));
      const badgeW=Math.max(48,`OHT ${r.triggerTransport.oht_id}`.length*7.2);
      const badgeClass=current<transportEnd?"oht-badge moving":"oht-badge";
      svg+=`<rect class="${badgeClass}" x="${labelX-badgeW/2}" y="${y-10}" width="${badgeW}" height="18" rx="9"/><text class="oht-job-marker" x="${labelX}" y="${y-.8}">OHT ${esc(r.triggerTransport.oht_id)}</text>`;
    }
  });
  svg+=`<line class="cursor" x1="${x(current)}" x2="${x(current)}" y1="${top-8}" y2="${height-22}"/></svg>`;
  $("chart").innerHTML=svg;
  $("chart").querySelectorAll(".live-task").forEach(el=>{
    el.onmousemove=e=>{if(pinnedTip)return;const r=tasks.find(x=>x._row_number==el.dataset.mi),tip=$("tip");tip.textContent=machineTooltip(r);tip.style.display="block";tip.style.left=Math.min(innerWidth-370,e.clientX+12)+"px";tip.style.top=Math.min(innerHeight-220,e.clientY+12)+"px"};
    el.onclick=e=>{e.stopPropagation();const r=tasks.find(x=>x._row_number==el.dataset.mi);toggleJobFilterFromTask(r)};
    el.onmouseleave=()=>{if(!pinnedTip)$("tip").style.display="none"};
  });
  updateChartCursor();
}
function renderStatus(){
  const colors={Idle:"#667085",Empty:"#98a2b3",Waiting:"#f79009",Breaked:"#f04438","Empty travel":"#98a2b3","Source wait":"#f79009",Loading:"#2e90fa",Loaded:"#12b76a","Loaded travel":"#12b76a",Unloading:"#7f56d9","Drop wait":"#f04438"};
  const stateClass=state=>state==="Waiting"||state==="Breaked"?"blocked":state==="Loaded"||state==="Loaded travel"||state==="Unloading"?"loaded":state==="Loading"?"loading":"";
  const cards=sortedOhtIds().map(id=>{
    const snap=ohtSnapshot(id),r=snap?.task,e=snap?.event,state=snap?.state||"Idle",point=snap?.point,eventName=e&&e.event?e.event:state;
    const job=r?esc(r.job_instance_id):"-";
    const fromRaw=r?r.from_machine:(e&&e.from_machine?e.from_machine:"-");
    const toRaw=r?r.to_machine:(e&&e.to_machine?e.to_machine:"-");
    const route=(fromRaw==="-"&&toRaw==="-")?"-":`M${esc(fromRaw)} → M${esc(toRaw)}`;
    return `<article class="state-row ${stateClass(state)} ${focusedOhts.has(String(id))?"focused":""}" data-state-oht="${esc(id)}" style="--state-color:${colors[state]||"#667085"}"><div class="state-main"><span class="state-oht">OHT ${esc(id)}</span><span class="state-name">${esc(state)}</span></div><div class="state-sub"><span><b>Job</b> ${job}</span><span><b>Route</b> ${route}</span></div></article>`;
  }).join("");
  $("status").innerHTML=cards||'<div class="state-empty">Select an OHT from the replay or list below.</div>';
  $("status").querySelectorAll(".state-row").forEach(row=>{
    row.onclick=event=>{
      event.stopPropagation();
      const id=String(row.dataset.stateOht);
      focusedOhts.has(id)?focusedOhts.delete(id):focusedOhts.add(id);
      renderStatus();
      renderSimulation();
    };
  });
}
function renderStatus(){
  const colors={Idle:"#667085",Empty:"#98a2b3",Waiting:"#f79009",Breaked:"#f04438","Empty travel":"#98a2b3","Source wait":"#f79009",Loading:"#2e90fa",Loaded:"#12b76a","Loaded travel":"#12b76a",Unloading:"#7f56d9","Drop wait":"#f04438"};
  const stateClass=state=>state==="Waiting"||state==="Breaked"?"blocked":state==="Loaded"||state==="Loaded travel"||state==="Unloading"?"loaded":state==="Loading"?"loading":"";
  const cards=sortedOhtIds().map(id=>{
    const snap=ohtSnapshot(id),r=snap?.task,e=snap?.event,state=snap?.state||"Idle";
    const job=r?esc(r.job_instance_id):"-";
    const fromRaw=r?r.from_machine:(e&&e.from_machine?e.from_machine:"-");
    const toRaw=r?r.to_machine:(e&&e.to_machine?e.to_machine:"-");
    const route=(fromRaw==="-"&&toRaw==="-")?"-":`M${esc(fromRaw)} -> M${esc(toRaw)}`;
    const isSelected=selected.has(String(id));
    const color=trackColor(id);
    return `<article class="state-row ${stateClass(state)} ${isSelected?"selected":""} ${focusedOhts.has(String(id))?"focused":""}" data-state-oht="${esc(id)}" style="--state-color:${colors[state]||"#667085"};--track-color:${color}"><div class="state-main"><span class="state-oht"><i class="track-swatch"></i> OHT ${esc(id)}</span><span class="state-name">${esc(state)}</span></div><div class="state-sub"><span><b>Job</b> ${job}</span><span><b>Route</b> ${route}</span></div></article>`;
  }).join("");
  $("status").innerHTML=cards||'<div class="state-empty">No OHT state rows available.</div>';
  $("status").querySelectorAll(".state-row").forEach(row=>{
    row.onclick=event=>{
      event.stopPropagation();
      toggleOhtSelection(String(row.dataset.stateOht));
    };
  });
}
function renderImpactExplanation(){
  const showAll=selectedJobInstance===ALL_JOBS;
  const selectedTransports=D.rows.filter(r=>
    selected.has(String(r.oht_id))&&
    (showAll||String(r.job_instance_id)===String(selectedJobInstance))
  );
  const fired=selectedTransports
    .filter(r=>Number(r.dropoff_time)<=current)
    .sort((a,b)=>Number(b.dropoff_time)-Number(a.dropoff_time));
  const rule='<p class="impact-rule"><b>Replay rule.</b> Actual Gantt follows the planned position until the selected OHT completes a transport. At <code>dropoff_time</code>, the same lot from that step onward is updated to the final actual machine schedule.</p>';
  if(!fired.length){
    $("impactExplanation").innerHTML=rule+'<div class="impact-empty">No selected OHT dropoff has occurred yet at the current replay time. Bars remain at planned positions.</div>';
    return;
  }
  const t=fired[0];
  const affected=D.machineRows.filter(r=>
    (String(r.job_instance_id)===String(t.job_instance_id)||String(r.lot_id)===String(t.lot_id))&&
    Number(r.step_no)>=Number(t.step_no)
  );
  const first=affected.find(r=>String(r.step_no)===String(t.step_no))||affected[0];
  const planned=first?getByTaskKeys(PLANNED_BY_JOB_STEP,first):null;
  const delay=first&&planned?Number(first.start)-Number(planned.start):0;
  $("impactExplanation").innerHTML=rule+`<div class="impact-current"><h3>Current update trigger</h3><dl class="impact-grid">
    <dt>OHT</dt><dd>${esc(t.oht_id)}</dd>
    <dt>Dropoff time</dt><dd>${fmt(t.dropoff_time)} s</dd>
    <dt>Lot / step</dt><dd>${esc(t.lot_id||t.job_id)} / ${esc(t.step_no)}</dd>
    <dt>Transport</dt><dd>M${esc(t.from_machine)} → M${esc(t.to_machine)}</dd>
    <dt>Updated bars</dt><dd>${affected.length} machine operations</dd>
    <dt>First shift</dt><dd>${planned&&first?`${fmt(planned.start)} → ${fmt(first.start)} (${fmt(delay)} s)`:"-"}</dd>
  </dl></div>`;
}
function renderImpactExplanation(){
  const showAll=selectedJobInstance===ALL_JOBS;
  const selectedTransports=D.rows.filter(r=>
    selected.has(String(r.oht_id))&&
    (showAll||String(r.job_instance_id)===String(selectedJobInstance))
  );
  const fired=selectedTransports
    .filter(r=>Number(r.dropoff_time)<=current)
    .sort((a,b)=>Number(b.dropoff_time)-Number(a.dropoff_time));
  const rule='<p class="impact-rule"><b>Replay rule.</b> Actual Gantt follows the planned position until the selected OHT completes a transport. At <code>dropoff_time</code>, the same lot from that step onward is updated to the final actual machine schedule.</p>';
  if(!fired.length){
    $("impactExplanation").innerHTML=rule+'<div class="impact-empty">No selected OHT dropoff has occurred yet at the current replay time. Bars remain at planned positions.</div>';
    return;
  }
  const t=fired[0];
  const affected=D.machineRows.filter(r=>
    (String(r.job_instance_id)===String(t.job_instance_id)||String(r.lot_id)===String(t.lot_id))&&
    Number(r.step_no)>=Number(t.step_no)
  );
  const first=affected.find(r=>String(r.step_no)===String(t.step_no))||affected[0];
  const planned=first?getByTaskKeys(PLANNED_BY_JOB_STEP,first):null;
  const delay=first&&planned?Number(first.start)-Number(planned.start):0;
  $("impactExplanation").innerHTML=rule+`<div class="impact-current"><h3>Current update trigger</h3><dl class="impact-grid">
    <dt>OHT</dt><dd>${esc(t.oht_id)}</dd>
    <dt>Dropoff time</dt><dd>${fmt(t.dropoff_time)} s</dd>
    <dt>Lot / step</dt><dd>${esc(t.lot_id||t.job_id)} / ${esc(t.step_no)}</dd>
    <dt>Transport</dt><dd>M${esc(t.from_machine)} -> M${esc(t.to_machine)}</dd>
    <dt>Updated bars</dt><dd>${affected.length} machine operations</dd>
    <dt>First shift</dt><dd>${planned&&first?`${fmt(planned.start)} -> ${fmt(first.start)} (${fmt(delay)} s)`:"-"}</dd>
  </dl></div>`;
}
function renderShiftNotifications(){
  if(!$("shiftNotifications"))return;
  const showAll=selectedJobInstance===ALL_JOBS;
  const fired=D.rows.filter(r=>{
      const drop=Number(r.dropoff_time);
      const id=`${r.oht_id}|${r.job_instance_id}|${r.step_no}|${drop}`;
      return selected.has(String(r.oht_id))&&
        (showAll||String(r.job_instance_id)===String(selectedJobInstance))&&
        drop<=current&&current<=drop+SHIFT_NOTE_TTL&&!dismissedShiftNotes.has(id);
    })
    .sort((a,b)=>Number(b.dropoff_time)-Number(a.dropoff_time));
  const notes=fired.map(t=>{
    const planned=getByTaskKeys(PLANNED_BY_JOB_STEP,t);
    const actual=getByTaskKeys(ACTUAL_BY_JOB_STEP,t);
    const affected=D.machineRows
      .filter(r=>
        (String(r.job_instance_id)===String(t.job_instance_id)||String(r.lot_id)===String(t.lot_id))&&
        Number(r.step_no)>=Number(t.step_no)
      )
      .map(r=>{
        const p=getByTaskKeys(PLANNED_BY_JOB_STEP,r);
        const delay=p?Number(r.start)-Number(p.start):0;
        return {actual:r,planned:p,delay};
      })
      .filter(x=>x.planned);
    const shifted=affected.filter(x=>Math.abs(x.delay)>0.1);
    const delay=planned&&actual?Number(actual.start)-Number(planned.start):(shifted[0]?.delay||0);
    if(Math.abs(delay)<=0.1&&!shifted.length)return null;
    return {t,planned,actual,delay,shifted};
  }).filter(Boolean);
  if(!notes.length){
    $("shiftNotifications").innerHTML='<div class="notify-empty"><b>No visible delay shift at this time.</b><br>When a selected OHT completion changes a Job from planned to actual timing, a 100-second delay event card appears here.</div>';
    return;
  }
  $("shiftNotifications").innerHTML=notes.slice(0,6).map(note=>{
    const {t,planned,actual,delay,shifted}=note;
    const left=Math.min(planned?Number(planned.start):Number(t.dispatch_time),actual?Number(actual.start):Number(t.dropoff_time));
    const right=Math.max(planned?Number(planned.end):Number(t.pickup_time),actual?Number(actual.end):Number(t.dropoff_time),left+1);
    const span=Math.max(1,right-left);
    const bar=(row,cls)=>{
      if(!row)return "";
      const x=(Number(row.start)-left)/span*100,w=Math.max(2,(Number(row.end)-Number(row.start))/span*100);
      return `<span class="mini-bar ${cls}" style="left:${x}%;width:${w}%"></span>`;
    };
    const arrow=planned&&actual&&Math.abs(Number(actual.start)-Number(planned.start))>0.1
      ?(()=>{
        const x1=(Number(planned.start)-left)/span*100;
        const x2=(Number(actual.start)-left)/span*100;
        const a=Math.min(x1,x2),w=Math.max(2,Math.abs(x2-x1));
        return `<span class="shift-arrow-line" style="left:${a}%;width:${w}%"></span>`;
      })()
      :"";
    const cascade=shifted
      .sort((a,b)=>Math.abs(b.delay)-Math.abs(a.delay))
      .slice(0,4)
      .map(x=>`<span class="shift-chip">M${esc(x.actual.physical_machine||x.actual.machine)} S${esc(x.actual.step_no)} ${x.delay>=0?"+":""}${fmt(x.delay)}s</span>`)
      .join("");
    const maxDelay=shifted.length?Math.max(...shifted.map(x=>x.delay)):delay;
    const agePct=Math.max(0,Math.min(100,(1-(current-Number(t.dropoff_time))/SHIFT_NOTE_TTL)*100));
    const id=`${t.oht_id}|${t.job_instance_id}|${t.step_no}|${Number(t.dropoff_time)}`;
    const cls=delay<0?"shift-note early":"shift-note";
    return `<article class="${cls}" data-shift-id="${esc(id)}">
      <div class="shift-event-row"><span class="shift-event-badge">DELAY SHIFT</span><div class="shift-event-main"><div class="shift-event-title">Job ${esc(t.job_instance_id)} / Step ${esc(t.step_no)} / OHT ${esc(t.oht_id)}</div><div class="shift-event-sub">Dropoff trigger ${fmt(t.dropoff_time)}s · ${shifted.length||1} affected machine operation${(shifted.length||1)>1?"s":""}</div></div><div class="shift-delta">${delay>=0?"+":""}${fmt(delay)}s</div></div>
      <div class="shift-note-meta">M${esc(t.from_machine)} -> M${esc(t.to_machine)} / dropoff ${fmt(t.dropoff_time)} s / machine ${esc(actual?.physical_machine||planned?.physical_machine||"-")}</div>
      <div class="shift-change"><div class="shift-timebox"><div class="shift-time-label">PLANNED</div><div class="shift-time-value">${planned?`${fmt(planned.start)} - ${fmt(planned.end)}`:"-"}</div></div><div class="shift-arrow-text">→</div><div class="shift-timebox actual"><div class="shift-time-label">ACTUAL</div><div class="shift-time-value">${actual?`${fmt(actual.start)} - ${fmt(actual.end)}`:"-"}</div></div></div>
      <div class="shift-legend"><span><i class="shift-key planned"></i>Planned baseline</span><span><i class="shift-key actual"></i>Actual executed</span></div>
      <div class="mini-gantt"><span class="mini-axis-label planned">Planned</span><span class="mini-axis-label actual">Actual</span><span class="mini-end-label planned">${planned?`${fmt(planned.start)}-${fmt(planned.end)}`:"-"}</span><span class="mini-end-label actual">${actual?`${fmt(actual.start)}-${fmt(actual.end)}`:"-"}</span>${arrow}${bar(planned,"mini-planned")}${bar(actual,"mini-actual")}</div>
      <div class="shift-cascade"><span class="shift-chip neutral">Cascade max ${maxDelay>=0?"+":""}${fmt(maxDelay)}s</span>${cascade}</div>
      <div class="shift-age"><div class="shift-age-fill" style="width:${agePct}%"></div></div>
    </article>`;
  }).join("");
  $("shiftNotifications").querySelectorAll(".shift-note").forEach(card=>{
    card.onclick=()=>{dismissedShiftNotes.add(card.dataset.shiftId);renderShiftNotifications()};
  });
}
function updateCounters(){
  $("timeText").textContent=fmt(current)+" s";
  const previewTime=$("ganttPreviewTime");
  if(previewTime)previewTime.textContent=fmt(current)+" s";
}
function pct(value){return `${(Number(value||0)*100).toFixed(1)}%`}
function signed(value){const n=Number(value||0);return `${n>=0?"+":""}${fmt(n)}`}
function renderKpis(){
  const k=D.kpis||{};
  const cmaxDelta=Number(D.meta.cmaxDelay||0);
  const cmaxClass=cmaxDelta>0?"positive":"negative";
  const cards=[
    ["Planned Cmax",`${fmt(D.meta.plannedCmax)} s`,`Baseline machine schedule`,"blue",""],
    ["Actual Cmax",`${fmt(D.meta.actualCmax)} s`,`Final simulation makespan`,"good",""],
    ["ΔCmax",`${signed(cmaxDelta)} s`,`Actual - Planned`,cmaxDelta>0?"warn":"good",cmaxClass],
    ["Blocking time",`${fmt(k.totalBlockingTime)} s`,`Loaded travel blocking`,"amber",""],
    ["Transport deviation",`${signed(k.totalTransportDeviation)} s`,`Σ actual transport gap`,Number(k.totalTransportDeviation)>0?"warn":"good",Number(k.totalTransportDeviation)>0?"positive":"negative"],
    ["Shifted ops",`${fmt(k.shiftedOps)}`,`${fmt(k.delayedOps)} delayed · ${fmt(k.earlyOps)} early`,"blue",""],
    ["OHT utilization",pct(k.avgOhtUtilization),`${fmt(k.activeOhtCount)} OHT · ${fmt(k.totalTransportTasks)} tasks`,"good",""],
    ["Dispatch mode",D.meta.dispatchMode||"unknown",D.meta.hasExactTrace?"Exact event trace":"Reconstructed trace","blue",""],
  ];
  cards[2][0]="Delta Cmax";
  cards[4][2]="sum actual transport gap";
  cards[5][2]=`${fmt(k.delayedOps)} delayed / ${fmt(k.earlyOps)} early`;
  cards[6][2]=`${fmt(k.activeOhtCount)} OHT / ${fmt(k.totalTransportTasks)} tasks`;
  $("kpiStrip").innerHTML=cards.map(([label,value,sub,tone,valueClass])=>
    `<article class="kpi-card ${tone}"><div class="kpi-label">${esc(label)}</div><div class="kpi-value ${valueClass||""}">${esc(value)}</div><div class="kpi-sub">${esc(sub)}</div></article>`
  ).join("");
}
function renderKpis(){
  const k=D.kpis||{};
  const isPartial=!!k.isPartialActual;
  const cmaxDelta=Number(D.meta.cmaxDelay||0);
  const completedDelta=Number(k.actualCompletedLotCmax||0)-Number(k.plannedCompletedLotCmax||0);
  const executedRows=D.machineRows.filter(r=>Number(r.end)<=current);
  const executedOps=executedRows.length;
  const currentLots=new Set(executedRows.map(r=>String(r.lot_id||r.job_id||"")).filter(Boolean));
  const activeTransports=D.rows.filter(r=>Number(r.dispatch_time)<=current&&current<=Number(r.dropoff_time));
  const activeOhts=new Set(activeTransports.map(r=>String(r.oht_id))).size;
  const deliveredTransports=D.rows.filter(r=>Number(r.dropoff_time)<=current).length;
  const elapsedBlocking=D.rows.reduce((sum,r)=>{
    const totalBlock=Number(r.blocking_loaded_time||0);
    if(!totalBlock)return sum;
    const start=Number(r.load_end_time||r.load_start_time||r.dispatch_time||0);
    const end=Number(r.dropoff_time||start);
    if(current<=start)return sum;
    const ratio=Math.max(0,Math.min(1,(current-start)/Math.max(.001,end-start)));
    return sum+totalBlock*ratio;
  },0);
  const cards=[
    ["Planned Cmax",`${fmt(D.meta.plannedCmax)} s`,`full baseline / ${fmt(k.plannedOps)} planned ops`,"blue",""],
    ["Replay time",`${fmt(current)} s`,`live playback cursor`,"blue",""],
    ["Executed ops",`${fmt(executedOps)} / ${fmt(k.plannedOps)}`,`${pct(executedOps/Math.max(Number(k.plannedOps||1),1))} operation coverage now`,"good",""],
    ["Completed lots",`${fmt(currentLots.size)} / ${fmt(k.plannedLots)}`,`lots with at least one finished operation`,"blue",""],
    ["Active OHT moves",`${fmt(activeOhts)} OHT`,` ${fmt(activeTransports.length)} active / ${fmt(deliveredTransports)} delivered transports`,"amber",""],
    ["Blocking elapsed",`${fmt(elapsedBlocking)} s`,`live accumulated loaded-travel blocking`,"amber",""],
    ["Cmax comparability",isPartial?"Partial run":`${signed(cmaxDelta)} s`,isPartial?"full planned Cmax is not directly comparable":"actual - planned",isPartial?"amber":(cmaxDelta>0?"warn":"good"),isPartial?"":(cmaxDelta>0?"positive":"negative")],
    ["Dispatch mode",D.meta.dispatchMode||"unknown",D.meta.hasExactTrace?"exact event trace":"reconstructed trace","blue",""],
  ];
  $("kpiStrip").innerHTML=cards.map(([label,value,sub,tone,valueClass])=>
    `<article class="kpi-card ${tone}"><div class="kpi-label">${esc(label)}</div><div class="kpi-value ${valueClass||""}">${esc(value)}</div><div class="kpi-sub">${esc(sub)}</div></article>`
  ).join("");
}
function updatePlayButtons(){
  $("play").textContent=playing?"Pause":"Play";
  const previewPlay=$("ganttPreviewPlay");
  if(previewPlay)previewPlay.textContent=playing?"Pause":"Play";
}
function setPlaying(next){
  playing=!!next;
  updatePlayButtons();
  lastFrame=performance.now();
  lastSimulationRenderAt=0;
  lastStatusRenderAt=0;
  if(!playing)renderHeavy(true);
  if(playing)requestAnimationFrame(tick);
}
function togglePlayback(){
  setPlaying(!playing);
}
function renderHeavy(force=false){
  const now=performance.now();
  if(!force&&now-lastHeavyRenderAt<HEAVY_RENDER_INTERVAL_MS)return;
  lastHeavyRenderAt=now;
  renderChart();
  renderShiftNotifications();
}
function setTime(value,forceHeavy=false){
  current=Math.max(0,Math.min(D.meta.actualCmax,Number(value)||0));
  $("slider").value=current;
  updateCounters();
  const now=performance.now();
  if(forceHeavy||!playing||now-lastKpiRenderAt>=500){
    lastKpiRenderAt=now;
    renderKpis();
  }
  updateChartCursor();
  if(forceHeavy||!playing||now-lastSimulationRenderAt>=SIMULATION_RENDER_INTERVAL_MS){
    lastSimulationRenderAt=now;
    renderSimulation();
  }
  if(forceHeavy||!playing||now-lastStatusRenderAt>=STATUS_RENDER_INTERVAL_MS){
    lastStatusRenderAt=now;
    renderStatus();
    renderShiftNotifications();
  }
  if(playing){
    renderHeavy(false);
  }
  if(forceHeavy||!playing||current>=D.meta.actualCmax){
    renderHeavy(true);
  }
}
function renderAll(){renderKpis();updateCounters();renderSimulation();renderStatus();renderHeavy(true)}
$("slider").addEventListener("input",e=>setTime(e.target.value,true));
$("play").onclick=togglePlayback;
function tick(now){if(!playing)return;const dt=(now-lastFrame)/1000*Number($("speed").value);lastFrame=now;setTime(current+dt,false);if(current>=D.meta.actualCmax){setPlaying(false);return}requestAnimationFrame(tick)}
function toggleGanttFullscreen(force){
  const panel=$("ganttPanel");
  const enable=force===undefined?!panel.classList.contains("fullscreen"):force;
  panel.classList.toggle("fullscreen",enable);
  $("fullscreenGantt").textContent=enable?"Exit full screen":"Full screen";
  updateBodyOverflow();
}
function openGanttModal(mode="actual"){
  ganttViewMode=mode;
  previewZoom=1;
  refreshJobOptions();
  if(!$("ganttModal")){renderHeavy(true);return}
  $("ganttModal").classList.add("open");
  syncGanttPreview();
  renderHeavy(true);
  updateBodyOverflow();
}
function closeGanttModal(){
  toggleGanttFullscreen(false);
  if(!$("ganttModal"))return;
  $("ganttModal").classList.remove("open");
  updateBodyOverflow();
}
function toggleSimulationFullscreen(force){
  const panel=$("simulationPanel");
  const enable=force===undefined?!panel.classList.contains("fullscreen"):force;
  panel.classList.toggle("fullscreen",enable);
  $("fullscreenSimulation").textContent=enable?"Exit full screen":"Full screen";
  updateBodyOverflow();
  requestAnimationFrame(applySimulationCamera);
}
function updateBodyOverflow(){
  const modalOpen=!!$("ganttModal")&&$("ganttModal").classList.contains("open");
  document.body.style.overflow=modalOpen||$("ganttPanel").classList.contains("fullscreen")||$("simulationPanel").classList.contains("fullscreen")?"hidden":"";
}
$("zoomIn").addEventListener("click",()=>zoomSimulation(1.25));
$("zoomOut").addEventListener("click",()=>zoomSimulation(1/1.25));
$("resetZoom").addEventListener("click",resetSimulationCamera);
$("fullscreenSimulation").addEventListener("click",()=>toggleSimulationFullscreen());
$("fullscreenGantt").addEventListener("click",()=>toggleGanttFullscreen());
$("openPlannedGantt")?.addEventListener("click",()=>openGanttModal("planned"));
$("openActualGantt")?.addEventListener("click",()=>openGanttModal("actual"));
$("closeGanttModal")?.addEventListener("click",closeGanttModal);
$("previewZoomOut")?.addEventListener("click",()=>{previewZoom=Math.max(.75,previewZoom/1.25);syncGanttPreview()});
$("previewZoomIn")?.addEventListener("click",()=>{previewZoom=Math.min(4,previewZoom*1.25);syncGanttPreview()});
$("previewFit")?.addEventListener("click",()=>{previewZoom=1;syncGanttPreview()});
$("ganttPreviewPlay")?.addEventListener("click",togglePlayback);
$("ganttPlannedMode").addEventListener("click",()=>{ganttViewMode="planned";renderHeavy(true)});
$("ganttActualMode").addEventListener("click",()=>{ganttViewMode="actual";renderHeavy(true)});
bindSideResize();
window.addEventListener("resize",()=>requestAnimationFrame(applySimulationCamera));
document.addEventListener("keydown",event=>{if(event.key==="Escape"){closeGanttModal();toggleSimulationFullscreen(false)}});
document.addEventListener("click",()=>{pinnedTip=false;$("tip").style.display="none"});
$("legend").innerHTML='<span><i class="sw" style="background:linear-gradient(90deg,#4E79A7,#F28E2B,#59A14F)"></i>Job color</span><span><b>Machine-centric</b> full schedule</span><span><b>Selected OHT badge</b>: assigned transport job</span><span style="border:2px dashed #dc2626;color:#991b1b">selected-job shift overlay</span>';
setupV3Layout();
buildSelection();
renderAll();
</script>
</body>
</html>
"""
