from __future__ import annotations

# Run from SOMOS:
#   python rune_somos_UI_Live.py
# Or directly:
#   python UI\UI_Live.py

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

UI_DIR = Path(__file__).resolve().parent
SOMOS_DIR = UI_DIR.parent
DEFAULT_OUTPUT_DIR = UI_DIR / "UI_LIVE" / "scratch"
DEFAULT_RESULT_DIR = UI_DIR / "UI_LIVE" / "oht_trace_ui_V3_live"

sys.path.insert(0, str(SOMOS_DIR))
sys.path.insert(0, str(UI_DIR))
from run import build_runner  # noqa: E402
from run_oht_trace_ui import run as build_v3_ui  # noqa: E402


TRANSPORT_HEADERS = [
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

MACHINE_HEADERS = [
    "sim_time", "node_name", "machine_no", "machine_name", "jssp_mach_id",
    "job_id", "job_instance_id", "lot_id", "step_no", "job_type_id",
    "op_index", "product_type", "start_time", "end_time", "process_time",
    "planned_ready_time", "realized_ready_time", "ready_deviation",
]


WRAPPER_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>SOMOS Live UI</title>
  <style>
    *{box-sizing:border-box}
    body{margin:0;background:#eef3f8;color:#172033;font:13px "Segoe UI",Arial,sans-serif;overflow:hidden}
    .livebar{height:34px;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:0 10px;background:#f8fafc;border-bottom:1px solid #d7e0ea;box-shadow:0 1px 3px #10182814}
    .left{display:flex;align-items:center;gap:8px;min-width:0}
    .dot{width:9px;height:9px;border-radius:50%;background:#f79009}.dot.running{background:#12b76a}.dot.done{background:#2e90fa}.dot.error{background:#f04438}
    b{font-weight:900}.muted{color:#667085;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    iframe{width:100vw;height:calc(100vh - 34px);border:0;display:block;background:#fff}
  </style>
</head>
<body>
  <div class="livebar">
    <div class="left"><span id="dot" class="dot"></span><b>SOMOS Live</b><span id="phase" class="muted">starting</span></div>
    <div class="muted" id="meta">waiting for first export</div>
  </div>
  <iframe id="v3" src="/v3/oht_trace_ui.html"></iframe>
  <script>
    const frame=document.getElementById("v3"),dot=document.getElementById("dot"),phase=document.getElementById("phase"),meta=document.getElementById("meta");
    let lastBuild=0;
    window.__somosLiveState = window.__somosLiveState || {};
    window.addEventListener("message", event => {
      if (!event.data || event.data.type !== "SOMOS_LIVE_STATE") return;
      window.__somosLiveState = Object.assign({}, window.__somosLiveState, event.data.state || {});
    });
    async function poll(){
      try{
        const r=await fetch("/api/status?_="+Date.now());
        const s=await r.json();
        dot.className="dot "+(s.phase||"running");
        phase.textContent=s.phase||"running";
        meta.textContent=`build ${s.build_count||0} / transports ${s.transport_rows||0} / ${s.message||""}`;
        if(s.last_build_at && s.last_build_at!==lastBuild){
          lastBuild=s.last_build_at;
          try {
            if (frame.contentWindow && frame.contentWindow.__somosSaveLiveState) {
              frame.contentWindow.__somosSaveLiveState();
            }
          } catch (_) {}
          frame.src="/v3/oht_trace_ui.html?_="+Date.now();
        }
      }catch(e){
        dot.className="dot error";
        phase.textContent="server disconnected";
      }
    }
    setInterval(poll, 2000);
    poll();
  </script>
</body>
</html>
"""


class SharedStatus:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.data: dict[str, Any] = {
            "phase": "starting",
            "message": "",
            "build_count": 0,
            "transport_rows": 0,
            "last_build_at": 0.0,
        }

    def update(self, **values: Any) -> None:
        with self.lock:
            self.data.update(values)
            self.data["updated_at"] = time.time()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.data)


def _ensure_csv_header(path: Path, headers: list[str]) -> None:
    if path.is_file():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(",".join(headers) + "\n", encoding="utf-8")


def _count_data_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    try:
        with path.open("r", encoding="utf-8-sig") as handle:
            return max(0, sum(1 for _ in handle) - 1)
    except OSError:
        return 0


def inject_live_bridge(html_path: Path, autoplay: bool) -> None:
    if not html_path.is_file():
        return
    marker = "<!-- SOMOS_V3_LIVE_BRIDGE_V2 -->"
    text = html_path.read_text(encoding="utf-8")
    if marker in text:
        return
    bridge = f"""
{marker}
<script>
(function(){{
  const KEY = "somos.v3.live.bridge";
  const AUTO_PLAY = {str(autoplay).lower()};
  function readState(){{
    try {{
      if (window.parent && window.parent.__somosLiveState && window.parent.__somosLiveState.savedAt) {{
        return window.parent.__somosLiveState;
      }}
    }} catch (_) {{}}
    try {{ return JSON.parse(localStorage.getItem(KEY) || "{{}}"); }}
    catch (_) {{ return {{}}; }}
  }}
  function saveState(){{
    try {{
      const slider = document.getElementById("slider");
      const play = document.getElementById("play");
      const selectedFromState = (typeof selected !== "undefined") ? Array.from(selected).map(String) : [];
      const selectedFromDom = Array.from(document.querySelectorAll(".oht-choice.active"))
        .map(button => String(button.dataset.ohtChoice || ""))
        .filter(Boolean);
      const selectedIds = Array.from(new Set([...selectedFromState, ...selectedFromDom]));
      const state = {{
        time: Number(slider && slider.value || 0),
        playing: !!(play && play.textContent === "Pause"),
        selectedOhtIds: selectedIds,
        selectedJobInstance: (typeof selectedJobInstance !== "undefined") ? selectedJobInstance : null,
        savedAt: Date.now()
      }};
      localStorage.setItem(KEY, JSON.stringify(state));
      try {{
        if (window.parent) {{
          window.parent.__somosLiveState = state;
          window.parent.postMessage({{type:"SOMOS_LIVE_STATE", state}}, "*");
        }}
      }} catch (_) {{}}
    }} catch (_) {{}}
  }}
  window.__somosSaveLiveState = saveState;
  function wrapFunction(name, after){{
    try {{
      const original = window[name];
      if (typeof original !== "function" || original.__somosWrapped) return;
      const wrapped = function(...args){{
        const result = original.apply(this, args);
        try {{ after(); }} catch (_) {{}}
        return result;
      }};
      wrapped.__somosWrapped = true;
      window[name] = wrapped;
    }} catch (_) {{}}
  }}
  function firstReplayTime(){{
    try {{
      const rows = (typeof D !== "undefined" && D.rows) ? D.rows : [];
      const first = rows.find(r => Number(r.dispatch_time || r.dropoff_time || 0) > 0);
      return first ? Math.max(0, Number(first.dispatch_time || first.dropoff_time || 0) - 5) : 0;
    }} catch (_) {{ return 0; }}
  }}
  function restore(){{
    const saved = readState();
    const hasRecent = saved && Number(saved.savedAt || 0) > Date.now() - 120000;
    if (hasRecent && Array.isArray(saved.selectedOhtIds) && typeof setSelectedOhts === "function") {{
      const available = new Set(
        Array.from(document.querySelectorAll("[data-oht-choice]"))
          .map(button => String(button.dataset.ohtChoice || ""))
      );
      const ids = saved.selectedOhtIds.map(String).filter(id => available.has(id));
      if (ids.length) setSelectedOhts(ids);
    }}
    if (hasRecent && saved.selectedJobInstance && typeof applyJobFilter === "function") {{
      applyJobFilter(saved.selectedJobInstance);
    }}
    const t = hasRecent ? Number(saved.time || 0) : firstReplayTime();
    if (typeof setTime === "function") setTime(t, true);
    if (typeof setPlaying === "function" && (AUTO_PLAY || (hasRecent && saved.playing))) {{
      setPlaying(true);
    }}
    wrapFunction("setSelectedOhts", saveState);
    wrapFunction("toggleOhtSelection", saveState);
    wrapFunction("setPlaying", saveState);
    wrapFunction("togglePlayback", saveState);
    wrapFunction("applyJobFilter", saveState);
    wrapFunction("setTime", saveState);
    saveState();
    setInterval(saveState, 500);
    window.addEventListener("beforeunload", saveState);
  }}
  if (document.readyState === "loading") {{
    document.addEventListener("DOMContentLoaded", () => setTimeout(restore, 80));
  }} else {{
    setTimeout(restore, 80);
  }}
}})();
</script>
"""
    html_path.write_text(text.replace("</body>", bridge + "\n</body>"), encoding="utf-8")


def prepare_live_inputs(output_dir: Path) -> None:
    _ensure_csv_header(output_dir / "transport_live.csv", TRANSPORT_HEADERS)
    _ensure_csv_header(output_dir / "log_machine_sim.csv", MACHINE_HEADERS)


def run_simulation(output_dir: Path, status: SharedStatus, seed: int, n_oht: int,
                   horizon: float, method: str = "SAVD") -> None:
    """Run one simulation in a worker thread, streaming the live trace to CSV.

    Same configuration as the paper: `build_runner` pins the shared settings and
    SAVD only switches idle-vehicle positioning on, so the UI shows exactly the
    arm the paper reports -- no deadlock recovery, no other heuristic.
    """
    status.update(phase="running", message="simulation running")
    try:
        runner = build_runner(str(output_dir), seed, n_oht, horizon)
        runner.oht_config.enable_live_trace = True          # UI: stream the trace
        if method.upper() == "SAVD":
            runner.oht_config.oht_dispatch_mode = "HUNGARIAN"
            runner.oht_config.oht_savd_positioning = True   # the proposed method
            runner.oht_config.oht_savd_window = 900.0       # H
            runner.oht_config.oht_savd_prior_weight = 1.0   # lambda
            runner.oht_config.oht_savd_grid = 0.0           # auto = span / 4
        else:
            runner.oht_config.oht_dispatch_mode = method.upper()
            runner.oht_config.oht_savd_positioning = False
        runner.run(enable_animation=False)
        status.update(phase="done", message="simulation complete")
    except Exception as exc:
        status.update(phase="error", message=f"{type(exc).__name__}: {exc}")
        raise


def _parse_selected_oht(value: str | None) -> list[str] | None:
    if not value:
        return None
    return [item.strip() for item in value.split(",") if item.strip()]


def build_loop(
    output_dir: Path,
    result_dir: Path,
    status: SharedStatus,
    interval: float,
    selected_oht: str | None,
    top_k: int,
    autoplay: bool,
) -> None:
    while True:
        try:
            prepare_live_inputs(output_dir)
            build_v3_ui(
                output_dir=str(output_dir),
                result_dir=str(result_dir),
                selected_oht_ids=_parse_selected_oht(selected_oht),
                top_k=top_k,
            )
            inject_live_bridge(result_dir / "oht_trace_ui.html", autoplay)
            transport_rows = _count_data_rows(output_dir / "transport_live.csv")
            snap = status.snapshot()
            status.update(
                build_count=int(snap.get("build_count", 0)) + 1,
                transport_rows=transport_rows,
                last_build_at=time.time(),
                message="UI refreshed",
            )
        except Exception as exc:
            status.update(message=f"UI export waiting: {type(exc).__name__}: {exc}")
        time.sleep(interval)


def make_handler(result_dir: Path, status: SharedStatus):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path.startswith("/api/status"):
                self._send_json(status.snapshot())
                return
            if self.path == "/" or self.path.startswith("/?"):
                self._send_bytes(WRAPPER_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if self.path.startswith("/v3/"):
                rel = self.path.split("?", 1)[0][len("/v3/"):]
                target = (result_dir / rel).resolve()
                if result_dir.resolve() not in target.parents and target != result_dir.resolve():
                    self.send_error(403)
                    return
                if not target.is_file():
                    self._send_bytes(
                        b"<html><body style='font-family:Segoe UI,Arial'>Waiting for first UI export...</body></html>",
                        "text/html; charset=utf-8",
                    )
                    return
                content_type = "text/html; charset=utf-8" if target.suffix.lower() == ".html" else "text/plain; charset=utf-8"
                self._send_bytes(target.read_bytes(), content_type)
                return
            self.send_error(404)

        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _send_json(self, data: dict[str, Any]) -> None:
            self._send_bytes(json.dumps(data, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

        def _send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    return Handler


def serve(result_dir: Path, status: SharedStatus, host: str, port: int) -> None:
    server = ThreadingHTTPServer((host, port), make_handler(result_dir, status))
    print(f"[SOMOS Live UI] http://{host}:{port}/")
    print(f"[SOMOS Live UI] HTML: {result_dir / 'oht_trace_ui.html'}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[SOMOS Live UI] stopped")
    finally:
        server.server_close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run SOMOS and refresh the SOMOS UI live.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--result-dir", default=str(DEFAULT_RESULT_DIR))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--n-oht", type=int, default=120)
    parser.add_argument("--horizon", type=float, default=86400.0)
    parser.add_argument("--method", default="SAVD",
                        choices=["SAVD", "HUNGARIAN", "NVF", "STD", "EDD", "FIFO", "PRIORITY"],
                        help="dispatching method to run (default: the proposed SAVD)")
    parser.add_argument("--refresh-sec", type=float, default=10.0)
    parser.add_argument("--selected-oht", default=None)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--no-autoplay", action="store_true", help="Do not auto-start the V3 replay after refresh.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-run", action="store_true", help="Only refresh UI from an existing output directory.")
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir).expanduser().resolve()
    result_dir = Path(args.result_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)
    prepare_live_inputs(output_dir)

    status = SharedStatus()
    threading.Thread(
        target=build_loop,
        args=(
            output_dir,
            result_dir,
            status,
            args.refresh_sec,
            args.selected_oht,
            args.top_k,
            not args.no_autoplay,
        ),
        daemon=True,
    ).start()

    if not args.no_run:
        threading.Thread(
            target=run_simulation,
            args=(output_dir, status, args.seed, args.n_oht, args.horizon, args.method),
            daemon=True,
        ).start()
    else:
        status.update(phase="running", message="watching existing output")

    serve(result_dir, status, args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
