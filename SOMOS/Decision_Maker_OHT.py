"""Empty-OHT dispatching on the one-way rail.

Holds everything that decides *which idle vehicle goes to which transport task*:

  * `OHTDecisionMaker.process` -- the dispatch loop. One round collects the tasks
    whose source machine is ready and assigns idle vehicles, either by rule-greedy
    sorting (NVF / STD / EDD / FIFO / PRIORITY) or by a global Hungarian matching.
  * `_dispatch_cost` -- what a (vehicle, task) pair costs: free-flow travel time
    for (vehicle -> source) + (source -> dest) on the directed rail graph, so
    one-way directionality is priced in rather than approximated by straight-line
    distance.
  * `_assign_savd_positioning` -- SAVD. Periodically re-spreads *idle* vehicles so
    their spatial distribution matches the demand density implied by the
    production plan. It never commits a vehicle to a task, so the dispatch
    decision above is untouched.
  * `AStarPath` / `BFSPath` -- shortest-path search on the directed rail graph.

The planned schedule is used for lateness, tie-breaking and SAVD's demand map;
it never overrides the travel-time cost.
"""

from __future__ import annotations

import bisect
import csv
import heapq
import math
import os
import random
from abc import ABC, abstractmethod
from collections import deque, Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import salabim as sim
from Simulation_Machine_Config import JSSPConfig

# The dispatch policies this build implements. NVF / STD / EDD / FIFO / PRIORITY
# are rule-greedy; HUNGARIAN is global minimum-sum matching. SAVD is not a mode:
# it is HUNGARIAN plus SAVD positioning (oht_savd_positioning), so that the
# two arms differ only in what idle vehicles do.
DISPATCH_MODES = ("NVF", "STD", "EDD", "FIFO", "PRIORITY", "HUNGARIAN")


# ══════════════════════════════════════════════════════════════════════
# [Experiment] Machine lock gate — keeps the call site's `in` / `.add`, but
#   dest allows concurrent entry up to the number of free input_buffer slots,
#   and the source lock is released.
#   relaxed=False behaves identically to the original binary lock (1 at a time) → for A/B comparison.
# ══════════════════════════════════════════════════════════════════════

class _DestGate:
    """`dest in gate` → True means 'cannot send more'. When relaxed, allows up to buffer free slots."""
    def __init__(self, machines: dict, inflight: Counter, relaxed: bool = True):
        self.machines = machines
        self.count = Counter(inflight)   # dest_name -> number of vehicles in progress/assigned
        self.relaxed = relaxed

    def _room(self, dest: str) -> int:
        m = self.machines.get(dest)
        if m is None:
            return 1
        cap = getattr(getattr(m, "machine_config", None), "input_buffer_cap", 1)
        used = len(getattr(m, "in_buffer", [])) + getattr(m, "reserved_in_slots", 0)
        return max(0, cap - used)

    def __contains__(self, dest: str) -> bool:
        if not self.relaxed:
            return self.count.get(dest, 0) > 0          # original binary lock
        return self.count.get(dest, 0) >= self._room(dest)  # block only when free slots are exhausted

    def add(self, dest: str):
        self.count[dest] += 1

    def __iter__(self):
        return iter(self.count)


class _SrcGate:
    """relaxed=True releases the source lock (duplicate pickup of the same job is prevented by job-level dedup)."""
    def __init__(self, init_set, relaxed: bool = True):
        self.s = set(init_set)
        self.relaxed = relaxed

    def __contains__(self, src: str) -> bool:
        if self.relaxed:
            return False
        return src in self.s

    def add(self, src: str):
        self.s.add(src)

    def __iter__(self):
        return iter(self.s)


# ══════════════════════════════════════════════════════════════════════
# A. Path search
# ══════════════════════════════════════════════════════════════════════

class PathAlgorithm(ABC):
    @abstractmethod
    def find_path(self, start: str, goal: str,
                  nodes: dict, adj: dict) -> List[str]:
        ...


class BFSPath(PathAlgorithm):
    """Directed BFS: minimize hop count + tie-break by proximity to destination."""

    @staticmethod
    def _goal_dist(n, goal, nodes) -> float:
        if n not in nodes or goal not in nodes:
            return 0.0
        return math.hypot(nodes[n].x - nodes[goal].x, nodes[n].y - nodes[goal].y)

    @staticmethod
    def _edge_dist(u, v, nodes) -> float:
        if u not in nodes or v not in nodes:
            return 1.0
        return math.hypot(nodes[u].x - nodes[v].x, nodes[u].y - nodes[v].y)

    def find_path(self, start, goal, nodes, adj):
        if start == goal:
            return [start]
        if start not in adj or goal not in nodes:
            return [start]

        q = deque([start])
        parent = {start: None}

        while q:
            u = q.popleft()
            nbs = list(adj.get(u, []))
            nbs.sort(key=lambda v: (self._goal_dist(v, goal, nodes), self._edge_dist(u, v, nodes), str(v)))

            for v in nbs:
                if v in parent:
                    continue
                parent[v] = u
                if v == goal:
                    q.clear()
                    break
                q.append(v)

        if goal not in parent:
            return [start]

        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()
        return path


class AStarPath(PathAlgorithm):
    """
    Directed A* — time (travel_time) cost + admissible time heuristic.

    When edges/uv_to_eid are injected, uses edge.travel_time as cost to find a path
    consistent with the OHT's actual travel time. Falls back to coordinate distance
    if not injected (compatible with the old behavior).
    """

    def __init__(self, edges=None, uv_to_eid=None, min_time_per_unit: float = 0.2):
        """
        min_time_per_unit : heuristic admissibility coefficient (sec / coordinate unit).
                            default 0.2 = 1/straight_speed(5 m/s). No edge can be
                            faster than this, so euclidean × 0.2 ≤ actual shortest time.
        """
        self.edges = edges or {}
        self.uv_to_eid = uv_to_eid or {}
        self.min_time_per_unit = float(min_time_per_unit)

    def _edge_dist(self, u, v, nodes) -> float:
        eid = self.uv_to_eid.get((u, v))
        if eid is not None and eid in self.edges:
            return float(getattr(self.edges[eid], 'travel_time', 1.0))
        # fall back to coordinate distance when edge info is not injected
        if u not in nodes or v not in nodes:
            return 1.0
        return math.hypot(nodes[u].x - nodes[v].x, nodes[u].y - nodes[v].y)

    def _goal_dist(self, n, goal, nodes) -> float:
        if n not in nodes or goal not in nodes:
            return 0.0
        # heuristic: euclidean × min_time_per_unit → admissible lower bound converted to time units
        return (math.hypot(nodes[n].x - nodes[goal].x,
                           nodes[n].y - nodes[goal].y)
                * self.min_time_per_unit)

    def find_path(self, start, goal, nodes, adj):
        if start == goal:
            return [start]
        if start not in adj or start not in nodes or goal not in nodes:
            return [start]

        heap = [(self._goal_dist(start, goal, nodes), 0.0, start)]
        came = {start: None}
        gs = {start: 0.0}
        closed = set()

        while heap:
            f, g, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)

            if cur == goal:
                path = []
                nd = goal
                while nd is not None:
                    path.append(nd)
                    nd = came[nd]
                path.reverse()
                return path

            nbs = list(adj.get(cur, []))
            nbs.sort(key=lambda nb: (self._goal_dist(nb, goal, nodes), self._edge_dist(cur, nb, nodes), str(nb)))

            for nb in nbs:
                if nb in closed:
                    continue
                ng = g + self._edge_dist(cur, nb, nodes)
                if ng < gs.get(nb, 1e18):
                    gs[nb] = ng
                    came[nb] = cur
                    heapq.heappush(heap, (ng + self._goal_dist(nb, goal, nodes), ng, nb))

        return [start]


# ══════════════════════════════════════════════════════════════════════
# B. TransportTask
# ══════════════════════════════════════════════════════════════════════

@dataclass
class TransportTask:
    task_id: int
    job_id: int                      # sim job instance id (job_seq) — unique
    op_index: int                    # index of the op just completed (internal 0-based)
    source_name: str
    dest_name: str
    planned_end: float               # for logging / lateness calc / tie-breaking
    job_instance_id: Optional[int] = None
    # ── gantt link ────────────────────────────────────────────────────
    lot_id: str = ""                 # gantt lot id (e.g. "L00001")
    step_no: int = 0                 # destination step (the step this transport arrives to process)
    planned_travel: float = 0.0      # gantt planned transport time (free-flow) = planned part of lateness label
    source_ready_time: float = 0.0   # sim time when the lot became ready in the source out_buffer


# ══════════════════════════════════════════════════════════════════════
# C. OHTDecisionMaker
# ══════════════════════════════════════════════════════════════════════

class OHTDecisionMaker(sim.Component):
    """
    Rule-based directed-distance dispatching.

    process() loop:
      1. Collect ready tasks from machine out_buffers
      2. Evaluate all idle OHT x ready task combinations
      3. cost = (veh.pos -> source) + (source -> dest)
      4. Dispatch the single lowest-cost pair
      5. Mark that OHT / source / dest as occupied, then repeat
    """

    def setup(
        self,
        vehicles: List,
        machines: Dict,
        adj: Dict,
        nodes: Dict,
        jssp_cfg: JSSPConfig,
        machine_node_map: Dict,
        edges: Dict = None,
        uv_to_eid: Dict = None,
        path_algo: PathAlgorithm = None,
        dispatch_dt: float = 0.5,
        dispatch_mode: str = "HUNGARIAN",
        swap_penalty: float = 30.0,
        load_sec: float = 10.0,
        unload_sec: float = 10.0,
        plan_tasks_csv: str = "",
        jssp_data=None,
        schedule=None,
    ):
        self.vehicles = vehicles
        self.machines = machines
        self.adj = adj
        self.nodes = nodes
        self.edges = edges or {}
        self.uv_to_eid = uv_to_eid or {}
        self.jssp_cfg = jssp_cfg
        self.machine_node_map = machine_node_map or {}
        self.path_algo = path_algo or AStarPath()
        self.dispatch_dt = dispatch_dt
        self._task_counter = 0
        # one of NVF / STD / EDD / FIFO / PRIORITY / HUNGARIAN (SAVD runs the
        # HUNGARIAN engine and switches SAVD positioning on separately)
        self.dispatch_mode = str(dispatch_mode or "HUNGARIAN").upper()
        if self.dispatch_mode not in DISPATCH_MODES:
            raise ValueError(
                f"unknown dispatch_mode {self.dispatch_mode!r}; "
                f"expected one of {sorted(DISPATCH_MODES)}")
        self.swap_penalty = max(0.0, float(swap_penalty))   # anti-thrash guard
        self.load_sec = max(0.0, float(load_sec))           # source dwell
        self.unload_sec = max(0.0, float(unload_sec))       # dest dwell
        # plan transports (transport_tasks.csv) -- the demand geography SAVD reads
        self.plan_tasks_csv = str(plan_tasks_csv or "")

        self.jssp_data = jssp_data
        self.schedule = schedule    # GanttSchedule (planned timing source for gantt mode)

        # [Experiment toggle] Relax machine locks. True: dest allows concurrent entry up to
        #   free buffer slots + release source lock / False: original binary lock (1 at a time). For A/B comparison.
        self.relax_machine_locks = True

        # planned_end is no longer a dispatch priority; it is only a reference
        # for logging / lateness calc / tie-breaking
        self.planned_end_map: Dict[Tuple[int, int], float] = {}
        self._load_plan()

        print(
            f"[OHTDecisionMaker] OHT path-switch based dispatch  "
            f"planned_keys={len(self.planned_end_map)}  "
            f"algo={type(self.path_algo).__name__}  "
            f"dispatch_mode={self.dispatch_mode}"
        )

        log_path = jssp_cfg.log_oht_csv
        os.makedirs(os.path.dirname(os.path.abspath(log_path)) or ".", exist_ok=True)
        self._log_file = open(log_path, "w", newline="", encoding="utf-8")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow([
            "sim_time", "oht_id", "job_id", "job_instance_id", "op_index",
            "event", "from_node", "to_node", "planned_end", "delay",
        ])

    # ------------------------------------------------------------------
    def _load_plan(self):
        """
        gantt mode: planned timing is queried directly from GanttSchedule(self.schedule),
        so no separate map load is needed. (removes the old ta01 gantt.csv dependency)
        """
        self.planned_end_map = {}   # keep empty map for backward compatibility

    # ------------------------------------------------------------------
    def find_path(self, start: str, goal: str) -> List[str]:
        return self.path_algo.find_path(start, goal, self.nodes, self.adj)

    def _resolve_node_name(self, name: str) -> str:
        """
        If source/dest is a machine name, convert it to the actual node name via machine_node_map.
        If it is already a node name, use it as-is.
        """
        if name in self.machine_node_map:
            return self.machine_node_map[name]
        return name

    def _path_travel_time(self, start: str, goal: str) -> float:
        """
        Total travel time of the shortest start -> goal path on the directed graph.
        Uses edge.travel_time (reflecting straight/curve speed).
        Returns inf if no path exists.

        Caches results by (start_node, goal_node). Safe because the graph and edge weights
        do not change during the simulation. Without caching, Hungarian × A* calls explode,
        making the dispatch loop cost O(n^2 × A*).
        """
        start_node = self._resolve_node_name(start)
        goal_node = self._resolve_node_name(goal)

        if start_node == goal_node:
            return 0.0

        cache = getattr(self, "_ptt_cache", None)
        if cache is None:
            cache = {}
            self._ptt_cache = cache
        key = (start_node, goal_node)
        cached = cache.get(key)
        if cached is not None:
            return cached

        path = self.find_path(start_node, goal_node)

        if not path:
            cache[key] = float("inf")
            return float("inf")
        if len(path) == 1 and path[0] == start_node and start_node != goal_node:
            cache[key] = float("inf")
            return float("inf")

        ttime = 0.0
        for u, v in zip(path[:-1], path[1:]):
            eid = self.uv_to_eid[(u, v)] if (u, v) in self.uv_to_eid else None
            if eid is not None and eid in getattr(self, 'edges', {}):
                ttime += float(self.edges[eid].travel_time)
            elif u in self.nodes and v in self.nodes:
                ttime += math.hypot(
                    self.nodes[u].x - self.nodes[v].x,
                    self.nodes[u].y - self.nodes[v].y,
                )
            else:
                ttime += 1.0

        cache[key] = ttime
        return ttime

    def _path_nodes_cached(self, start_node: str, goal_node: str) -> List[str]:
        """Path cache for dispatch score calculation. Uses the same path algorithm as _path_travel_time."""
        cache = getattr(self, "_path_nodes_cache", None)
        if cache is None:
            cache = {}
            self._path_nodes_cache = cache
        key = (start_node, goal_node)
        if key not in cache:
            cache[key] = self.find_path(start_node, goal_node)
        return cache[key]

    def _node_xy(self, name: str):
        """Convert a machine name or node name to coordinates. Returns None on failure."""
        node_name = self._resolve_node_name(name)
        node = self.nodes.get(node_name)
        if node is None:
            return None
        return float(node.x), float(node.y)

    def _dispatch_cost(self, veh, task: TransportTask) -> Tuple[float, float, float]:
        """Cost of (empty OHT -> source) + (loaded OHT -> dest), in seconds.

        Free-flow shortest-path travel time on the one-way rail, identical for
        every dispatch rule in this build: the rules differ in *which* task they
        pick, not in how a vehicle-task pair is priced. Hungarian minimises the
        sum of this cost over the assignment.
        """
        empty_move = self._path_travel_time(veh.pos_node, task.source_name)
        loaded_move = self._path_travel_time(task.source_name, task.dest_name)

        if math.isinf(empty_move) or math.isinf(loaded_move):
            return float("inf"), empty_move, loaded_move

        return empty_move + loaded_move, empty_move, loaded_move

    def _reserved_job_keys(self) -> Set[Tuple[int, int]]:
        reserved: Set[Tuple[int, int]] = set()
        for v in self.vehicles:
            task = getattr(v, "assigned_task", None)
            if task is None:
                continue
            job_instance_id = getattr(task, "job_instance_id", None)
            if job_instance_id is None:
                continue
            reserved.add((int(job_instance_id), int(task.op_index)))
        return reserved

    def _collect_ready_tasks(
        self,
        now: float,
        reserved_sources: Set[str],
        reserved_dests: Set[str],
    ) -> List[TransportTask]:
        tasks: List[TransportTask] = []
        reserved_jobs = self._reserved_job_keys()

        for source_name, machine in self.machines.items():
            for job in list(getattr(machine, "out_buffer", [])):
                if not getattr(job, "next_candidate_nodes", None):
                    continue

                dest_name = job.next_candidate_nodes[0]
                if not dest_name:
                    continue

                completed_op = max(0, int(getattr(job, "op_index", 0)) - 1)
                # dedup key: unique sim instance (job_id) + completed op (unique per lot)
                job_key = (int(getattr(job, "job_id", -1)), completed_op)
                if job_key in reserved_jobs:
                    continue

                # prevent duplication if another OHT is already going to pick up the same source
                if source_name in reserved_sources:
                    continue

                # prevent duplication if another OHT is already heading to the same dest
                if dest_name in reserved_dests:
                    continue

                # gantt: destination step = job.step_no (routing update points to the next step)
                _lot = getattr(job, "lot_id", "")
                _dstep = int(getattr(job, "step_no", 0))
                _ptravel = 0.0
                _planned_end = now
                if self.schedule is not None:
                    pt = self.schedule.planned_travel(_lot, _dstep)
                    _ptravel = pt if pt is not None else 0.0
                    pe = self.schedule.planned_ready(_lot, _dstep)
                    _planned_end = pe if pe is not None else now
                # time the lot became ready in the source out_buffer = actual end of the previous step
                _src_ready = float(getattr(job, "prev_realized_ready", now))

                self._task_counter += 1
                tasks.append(TransportTask(
                    task_id=self._task_counter,
                    job_id=int(job.job_id),
                    job_instance_id=int(job.job_id),
                    op_index=completed_op,
                    source_name=source_name,
                    dest_name=dest_name,
                    planned_end=_planned_end,
                    lot_id=_lot,
                    step_no=_dstep,
                    planned_travel=_ptravel,
                    source_ready_time=_src_ready,
                ))

        # Dispatch priority is now decided by the distance cost inside process().
        # Here, sort only deterministically
        tasks.sort(key=lambda t: (
            t.source_name,
            t.dest_name,
            t.job_id,
            t.job_instance_id or 0,
            t.op_index,
        ))
        return tasks

    def _log(
        self,
        oht_id,
        job_id,
        op_index,
        event,
        from_node,
        to_node,
        planned_end,
        delay,
        job_instance_id=None,
    ):
        self._log_writer.writerow([
            round(self.env.now(), 3),
            oht_id,
            job_id + 1,
            job_instance_id if job_instance_id is not None else "",
            op_index + 1,
            event,
            from_node,
            to_node,
            round(planned_end, 2),
            round(delay, 2),
        ])
        self._log_file.flush()

    def close_log(self):
        if not self._log_file.closed:
            self._log_file.close()

    def _get_plan_source_weights(self):
        """gantt geographic prior: total pickup count per source machine in the plan (static).
        Which regions are chronically hot → the demand geography where idle vehicles should reside.
        An aggregate, so unaffected by realized lag (drift)."""
        if getattr(self, "_plan_src_w", None) is not None:
            return self._plan_src_w
        import csv as _csv, os as _os
        W = defaultdict(float)
        path = self.plan_tasks_csv
        if path and _os.path.exists(path):
            with open(path, newline="") as f:
                for row in _csv.DictReader(f):
                    src = self._resolve_node_name(row.get("source_machine", ""))
                    if src and src in self.nodes:
                        W[src] += 1.0
        self._plan_src_w = dict(W)
        return self._plan_src_w

    def _get_plan_rolling_index(self):
        """Plan pickups sorted by planned-ready time, for the rolling-window demand term.

        Returns (times, nodes) as two parallel sorted lists so a window query is a
        pair of bisects. `planned_ready_sec` in transport_tasks.csv is the source
        step's planned end, i.e. the same time axis as Schedule.planned_ready().
        """
        if getattr(self, "_plan_roll", None) is not None:
            return self._plan_roll
        rows = []
        path = self.plan_tasks_csv
        if path and os.path.exists(path):
            with open(path, newline="") as f:
                for row in csv.DictReader(f):
                    src = self._resolve_node_name(row.get("source_machine", ""))
                    if not src or src not in self.nodes:
                        continue
                    try:
                        rows.append((float(row.get("planned_ready_sec", "")), src))
                    except (TypeError, ValueError):
                        continue
        rows.sort()
        self._plan_roll = ([r[0] for r in rows], [r[1] for r in rows])
        return self._plan_roll

    def _plan_time_anchor(self, now: float) -> float:
        """Where the fab currently sits on the *plan* time axis.

        Realized execution drifts behind the plan (badly so once the rail congests),
        so querying the plan at wall-clock `now` would read the wrong slice. The
        median planned end of the operations actually in progress is a direct,
        self-calibrating estimate of plan position. Falls back to `now` when nothing
        is running or when the anchor is switched off.
        """
        if not getattr(self, "savd_roll_anchor", True) or self.schedule is None:
            return now
        pts = []
        for m in self.machines.values():
            pjob = getattr(m, "processing_job", None)
            if pjob is None:
                continue
            lot = getattr(pjob, "lot_id", "")
            step = int(getattr(pjob, "step_no", 0))
            if not lot:
                continue
            pe = self.schedule.planned_ready(lot, step)
            if pe is not None:
                pts.append(float(pe))
        if not pts:
            return now
        pts.sort()
        return pts[len(pts) // 2]

    def _assign_savd_positioning(self, now, window=900.0, grid=0.0, prior_weight=1.0,
                                     terms='all', roll_anchor=True):
        """(A) Gantt demand-density COVERAGE — match idle OHTs' *spatial distribution* to future
        pickup demand density.
          - Unlike per-source positioning (mean, 1:1 with imminent machines, zero-sum), matches the
            *spatial distribution* to eliminate hot regions that 'have demand but no nearby empty
            vehicle' (=cause of the source_wait tail) → aims to shrink the tail.
          - Only idle vehicles in surplus cells are steered to deficit (hot) cells (roam_target) →
            minimal extra empty travel. Not a commit (can_accept_dispatch stays True → reactive dispatch can grab it).
          - terms: which demand terms are active.
              'all'      = realized(out_buffer + imminence) + static gantt prior  [default]
              'realized' = drop the prior          'prior' = drop the realized terms
              'roll'     = realized + **rolling** plan window (replaces the static prior)
              'rollonly' = rolling window alone
            The static prior is a whole-horizon histogram with no time axis; the rolling
            term instead counts plan pickups due in [anchor, anchor+window], which is what
            makes `window` a genuine look-ahead rather than a saturating gate."""
        use_real = terms in ('all', 'realized', 'roll')
        use_prior = terms in ('all', 'prior')
        use_roll = terms in ('roll', 'rollonly')
        self.savd_roll_anchor = bool(roll_anchor)
        idle = [v for v in self.vehicles if v.can_accept_dispatch()]
        if len(idle) < 2:
            return
        # ── 1) per-machine demand weight = realized imminence (schedule) + gantt geographic prior ──
        gprior = self._get_plan_source_weights()
        gmax = max(gprior.values()) if gprior else 1.0
        roll, rmax = None, 1.0
        if use_roll:
            rtimes, rnodes = self._get_plan_rolling_index()
            if rtimes:
                anchor = self._plan_time_anchor(now)
                i = bisect.bisect_left(rtimes, anchor)
                j = bisect.bisect_right(rtimes, anchor + window)
                roll = Counter(rnodes[i:j])
                rmax = max(roll.values()) if roll else 1.0
                self._cov_roll_n = j - i          # tasks in window (diagnostic)
        dem = {}                                          # node -> weight
        for mname, m in self.machines.items():
            node = self._resolve_node_name(mname)
            if not node or node not in self.nodes:
                continue
            w = 0.0
            if use_real and getattr(m, "out_buffer", None):
                w += 3.0                                  # already waiting for pickup = top priority
            pjob = getattr(m, "processing_job", None)
            if use_real and pjob is not None and self.schedule is not None:
                lot = getattr(pjob, "lot_id", ""); cs = int(getattr(pjob, "step_no", 0))
                if lot and self.schedule.next_machine_after(lot, cs):   # next step = pickup arises
                    st = getattr(m, "_processing_start_time", None)
                    du = getattr(m, "_processing_duration", None)
                    rem = ((st + du) - now) if (st is not None and du is not None) else 0.0
                    if rem <= window:
                        w += 1.0 - max(0.0, rem) / window  # larger the more imminent
            if use_prior:
                w += prior_weight * (gprior.get(node, 0.0) / gmax)   # static geographic prior
            if roll is not None:
                # rolling look-ahead: pickups this machine is planned to release
                # within `window` of the current plan position
                w += prior_weight * (roll.get(node, 0) / rmax)
            if w > 1e-9:
                dem[node] = w
        if not dem:
            return
        # ── 2) cell grid (coordinate bbox → grid) ──
        xs = [self.nodes[n].x for n in dem]; ys = [self.nodes[n].y for n in dem]
        if grid <= 0:
            span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
            grid = span / 4.0                             # ~4 cells/axis
        x0, y0 = min(xs), min(ys)
        def cell_of(x, y):
            return (int((x - x0) // grid), int((y - y0) // grid))
        cell_dem = defaultdict(float)
        cell_rep = {}; cell_repw = defaultdict(float)     # cell representative node (max-weight machine)
        for node, w in dem.items():
            c = cell_of(self.nodes[node].x, self.nodes[node].y)
            cell_dem[c] += w
            if w > cell_repw[c]:
                cell_repw[c] = w; cell_rep[c] = node
        total = sum(cell_dem.values())
        if total <= 0:
            return
        # ── 3) idle vehicle cell distribution ──
        idle_by_cell = defaultdict(list)
        for v in idle:
            nd = self.nodes.get(v.pos_node)
            if nd is not None:
                idle_by_cell[cell_of(nd.x, nd.y)].append(v)
        N = len(idle)
        target = {c: cell_dem[c] / total * N for c in cell_dem}
        # ── 4) surplus vehicles = excess in over-demand cells + all vehicles in zero-demand cells ──
        surplus = []
        for c, vs in idle_by_cell.items():
            keep = int(target.get(c, 0.0))                # floor(target)
            if len(vs) > keep:
                surplus.extend(vs[keep:])
        if not surplus:
            return
        for v in surplus:
            v.roam_target = None                          # reassign (clear stale)
        # ── 5) from hottest deficit cell, assign the nearest surplus vehicle (min-max deficit = target the tail) ──
        deficits = sorted(cell_dem.keys(), key=lambda c: cell_dem[c], reverse=True)
        used = set()
        for c in deficits:
            rep = cell_rep.get(c)
            if not rep:
                continue
            need = int(round(target.get(c, 0.0) - len(idle_by_cell.get(c, []))))
            for _ in range(max(0, need)):
                best = None; bestc = float("inf")
                for v in surplus:
                    if id(v) in used:
                        continue
                    cc = self._path_travel_time(v.pos_node, rep)
                    if cc < bestc:
                        bestc = cc; best = v
                if best is None:
                    break
                best.roam_target = rep
                used.add(id(best))
        return len(used)

    def _reserved_machine_nodes(self) -> Tuple[Set[str], Set[str]]:
        """
        Reserve only source / dest machines already in use by another OHT.
        """
        reserved_sources: Set[str] = set()
        reserved_dests: Set[str] = set()

        for v in self.vehicles:
            task = getattr(v, "assigned_task", None)
            if task is None:
                continue
            reserved_sources.add(task.source_name)
            reserved_dests.add(task.dest_name)

        return reserved_sources, reserved_dests

    def _select_best_pair(
        self,
        idle_vehicles: List,
        ready_tasks: List[TransportTask],
        claimed_sources: Set[str],
        claimed_dests: Set[str],
    ):
        """
        Among all idle OHT x ready task combinations,
        select the pair with the smallest directed-distance cost.
        """
        best = None

        for veh in idle_vehicles:
            for task in ready_tasks:
                if task.source_name in claimed_sources:
                    continue
                if task.dest_name in claimed_dests:
                    continue

                total_cost, empty_move, loaded_move = self._dispatch_cost(veh, task)
                if math.isinf(total_cost):
                    continue

                # tie-break:
                # 1) total distance
                # 2) empty move
                # 3) loaded move
                # 4) planned_end (on a tie, the earlier task)
                # 5) job/order info
                key = (
                    total_cost,
                    empty_move,
                    loaded_move,
                    task.planned_end,
                    task.job_id,
                    task.job_instance_id or 0,
                    task.op_index,
                )

                if best is None or key < best[0]:
                    best = (key, veh, task)

        return best

    def _select_assignments_hungarian(
        self,
        idle_vehicles: List,
        ready_tasks: List[TransportTask],
        claimed_sources: Set[str],
        claimed_dests: Set[str],
    ):
        """
        Global minimum-cost matching of idle OHT × ready task via Hungarian (linear_sum_assignment).

        Greedy (_select_best_pair) is optimal only per step, so the sum cost may be sub-optimal.
        Hungarian minimizes the total cost sum at once.

        Returns
        -------
        List[(key, veh, task)] — process() can assign/log directly. Ascending by cost.
        scipy not installed / no feasible matching → None (caller falls back to greedy).
        """
        if not idle_vehicles or not ready_tasks:
            return []
        try:
            import numpy as np
            from scipy.optimize import linear_sum_assignment
        except ImportError:
            return None

        n_v, n_t = len(idle_vehicles), len(ready_tasks)
        big = 1e18
        cost = np.full((n_v, n_t), big, dtype=np.float64)
        empty_arr = np.zeros((n_v, n_t), dtype=np.float64)
        loaded_arr = np.zeros((n_v, n_t), dtype=np.float64)

        penalty = getattr(self, 'swap_penalty', 0.0)
        for j, task in enumerate(ready_tasks):
            if task.source_name in claimed_sources:
                continue
            if task.dest_name in claimed_dests:
                continue
            for i, veh in enumerate(idle_vehicles):
                total, empty_move, loaded_move = self._dispatch_cost(veh, task)
                if math.isinf(total):
                    continue
                # swap_penalty when a vehicle already bound to a task (reassignable) matches a *different* task.
                # The cost saving from progress on the existing path must exceed the patch for Hungarian to choose a swap.
                prev = getattr(veh, "assigned_task", None)
                if prev is not None:
                    same = (getattr(prev, "job_instance_id", None) == task.job_instance_id
                            and getattr(prev, "op_index", None) == task.op_index)
                    if not same:
                        total += penalty
                cost[i, j] = total
                empty_arr[i, j] = empty_move
                loaded_arr[i, j] = loaded_move

        if not np.isfinite(cost).any():
            return []

        row_ind, col_ind = linear_sum_assignment(cost)

        results = []
        for i, j in zip(row_ind, col_ind):
            if not np.isfinite(cost[i, j]):
                continue
            task = ready_tasks[j]
            key = (
                float(cost[i, j]),
                float(empty_arr[i, j]),
                float(loaded_arr[i, j]),
                task.planned_end,
                task.job_id,
                task.job_instance_id or 0,
                task.op_index,
            )
            results.append((key, idle_vehicles[i], task))

        # Process in ascending cost order (so when the same source/dest appears duplicated in
        # ready_tasks, the cheaper pair claims first).
        results.sort(key=lambda r: r[0])
        return results

    # ------------------------------------------------------------------
    def _apply_hungarian_assignments(self, assignments, claimed_sources,
                                     claimed_dests, now):
        """Apply the Hungarian result list to actual veh.assign. Log 'reassign' on a swap."""
        used_vehs = set()
        for _, veh, task in assignments:
            if id(veh) in used_vehs:
                continue
            if task.source_name in claimed_sources:
                continue
            if task.dest_name in claimed_dests:
                continue

            prev = getattr(veh, "assigned_task", None)
            same = (prev is not None
                    and getattr(prev, "job_instance_id", None) == task.job_instance_id
                    and getattr(prev, "op_index", None) == task.op_index)
            if not same:
                setattr(task, "_dispatch_time", now)
                veh.assign(task)
                delay = max(0.0, now - task.planned_end)
                self._log(
                    veh.vid, task.job_id, task.op_index,
                    "reassign" if prev is not None else "dispatch",
                    task.source_name, task.dest_name,
                    task.planned_end, delay,
                    job_instance_id=task.job_instance_id,
                )

            used_vehs.add(id(veh))
            claimed_sources.add(task.source_name)
            claimed_dests.add(task.dest_name)

    def _apply_greedy(self, idle_vehicles, ready_tasks, claimed_sources,
                      claimed_dests, now):
        """Step-by-step greedy. Uses the existing _select_best_pair."""
        while idle_vehicles and ready_tasks:
            best = self._select_best_pair(
                idle_vehicles=idle_vehicles,
                ready_tasks=ready_tasks,
                claimed_sources=claimed_sources,
                claimed_dests=claimed_dests,
            )
            if best is None:
                break
            _, veh, task = best
            setattr(task, "_dispatch_time", now)
            veh.assign(task)
            claimed_sources.add(task.source_name)
            claimed_dests.add(task.dest_name)
            delay = max(0.0, now - task.planned_end)
            self._log(
                veh.vid, task.job_id, task.op_index,
                "dispatch",
                task.source_name, task.dest_name,
                task.planned_end, delay,
                job_instance_id=task.job_instance_id,
            )
            idle_vehicles.remove(veh)
            ready_tasks.remove(task)

    def _apply_rule_greedy(self, idle_vehicles, ready_tasks, claimed_sources,
                           claimed_dests, now, rule):
        """Build a sort key for (veh, task) candidates using one of the rules (see README),
        and commit sequentially in ascending key order (step-by-step greedy). Smaller key = better candidate.
        rule in {NVF, STD, EDD, FIFO, PRIORITY}."""
        if not idle_vehicles or not ready_tasks:
            return

        def out_q(task):
            m = self.machines.get(task.source_name)
            return len(getattr(m, "out_buffer", []) or []) if m is not None else 0

        def key(veh, task, total, empty, loaded):
            vid = getattr(veh, "vid", 1 << 30)
            if rule == "NVF":                                  # nearest vehicle first (shortest to source)
                return (empty, loaded, total, vid)
            if rule == "STD":                                  # min total travel distance
                return (empty + loaded, empty, loaded, vid)
            if rule == "EDD":                                  # earliest planned_end
                return (float(getattr(task, "planned_end", now)), empty, loaded, total, vid)
            if rule == "FIFO":                                 # source ready earliest
                return (float(getattr(task, "source_ready_time", 0.0)), empty, loaded, total, vid)
            if rule == "PRIORITY":                             # largest lateness + queue pressure
                lateness = max(0.0, now - float(getattr(task, "planned_end", now)))
                return (-(lateness + out_q(task)), empty, loaded, total, vid)
            return (total, empty, loaded, vid)                 # unreachable: modes are validated

        pairs = []
        for t in ready_tasks:
            for v in idle_vehicles:
                tc, em, lm = self._dispatch_cost(v, t)
                if math.isinf(tc):
                    continue
                pairs.append((key(v, t, tc, em, lm), v, t))
        pairs.sort(key=lambda x: x[0])

        used_v, used_t = set(), set()
        for _, veh, task in pairs:
            if id(veh) in used_v or id(task) in used_t:
                continue
            if task.source_name in claimed_sources or task.dest_name in claimed_dests:
                continue
            setattr(task, "_dispatch_time", now)
            veh.assign(task)
            claimed_sources.add(task.source_name)
            claimed_dests.add(task.dest_name)
            delay = max(0.0, now - task.planned_end)
            self._log(veh.vid, task.job_id, task.op_index, "dispatch",
                      task.source_name, task.dest_name, task.planned_end, delay,
                      job_instance_id=task.job_instance_id)
            used_v.add(id(veh)); used_t.add(id(task))

    def request_dispatch(self):
        """Event-based dispatch trigger.

        Called at the moment 'state changed', such as a new pickup (loaded into a machine's
        out_buffer) or an OHT switching to idle. If the dispatcher is asleep (passivated) with
        nothing to do, wake it to re-dispatch immediately (ignored if not asleep).
        """
        if self.ispassive():
            self.activate()

    def process(self):
        """Dispatch loop.

        One round = collect the tasks whose source is ready, then hand every idle
        vehicle a task. Two engines exist in this build:

          * HUNGARIAN -- global minimum-sum matching over (idle vehicle, task)
            with `_dispatch_cost`, used by HUNGARIAN itself and by SAVD;
          * rule greedy -- NVF / STD / EDD / FIFO / PRIORITY sort the ready tasks
            by their rule key and commit them one at a time.

        SAVD adds nothing here: its SAVD positioning runs on its own timer
        (`_assign_savd_positioning`, driven from Simulation_Main) and only moves
        *idle* vehicles, so the dispatch decision itself is unchanged.
        """
        mode = self.dispatch_mode
        print(f"[OHTDecisionMaker] dispatch_mode = {mode}")

        while True:
            now = self.env.now()

            reserved_sources, reserved_dests = self._reserved_machine_nodes()
            rlx = getattr(self, "relax_machine_locks", False)
            inflight_dst = Counter()
            for v in self.vehicles:
                t = getattr(v, "assigned_task", None)
                if t is not None:
                    inflight_dst[t.dest_name] += 1
            reserved_sources = _SrcGate(reserved_sources, relaxed=rlx)
            reserved_dests = _DestGate(self.machines, inflight_dst, relaxed=rlx)
            ready_tasks = self._collect_ready_tasks(
                now, reserved_sources, reserved_dests)
            claimed_sources = reserved_sources
            claimed_dests = reserved_dests
            idle_vehicles = [v for v in self.vehicles if v.can_accept_dispatch()]

            if mode in ("NVF", "STD", "EDD", "FIFO", "PRIORITY"):
                # rule-key sorted greedy (see README, "Dispatch rules")
                self._apply_rule_greedy(idle_vehicles, ready_tasks,
                                        claimed_sources, claimed_dests, now, mode)
            else:
                # HUNGARIAN (and SAVD, which is HUNGARIAN + SAVD positioning):
                # minimise the sum of the idle-vehicle x task cost matrix
                assignments = self._select_assignments_hungarian(
                    idle_vehicles, ready_tasks, claimed_sources, claimed_dests)
                if assignments is not None:
                    self._apply_hungarian_assignments(
                        assignments, claimed_sources, claimed_dests, now)
                else:
                    # no feasible matching (all costs infinite) -> nearest-first fallback
                    self._apply_greedy(idle_vehicles, ready_tasks,
                                       claimed_sources, claimed_dests, now)

            # ── Wait until the next dispatch: event-based ─────────────────────
            # Polling every dispatch_dt was mostly wasted work. When there is
            # nothing to do the dispatcher passivates (sleeps) and a new pickup
            # (machine out_buffer) or an OHT going idle wakes it through
            # request_dispatch(). One case still needs a timed retry: an idle
            # vehicle and a pending pickup coexist but this round could not match
            # them (source/dest lock, or the path is blocked) -- then poll until
            # the block clears.
            idle_now = [v for v in self.vehicles if v.can_accept_dispatch()]
            pending_pickup = any(
                getattr(ms, "out_buffer", None) for ms in self.machines.values())
            if idle_now and pending_pickup:
                yield self.hold(self.dispatch_dt)
            else:
                yield self.passivate()


# ══════════════════════════════════════════════════════════════════════
# D. Idle Behavior
# ══════════════════════════════════════════════════════════════════════

class IdleBehavior:
    def next_node(self, pos_node: str, adj: dict) -> str:
        return pos_node


class RandomRoam(IdleBehavior):
    """Select only station nodes (exclude ARC guide nodes)"""

    def __init__(self, nodes: dict = None):
        self._nodes = nodes or {}

    def next_node(self, pos_node: str, adj: dict) -> str:
        neighbors = adj.get(pos_node, [])
        station_nbrs = [
            n for n in neighbors
            if self._nodes.get(n) and self._nodes[n].kind == "station"
        ]
        candidates = station_nbrs if station_nbrs else neighbors
        return random.choice(candidates) if candidates else pos_node


class StayIdle(IdleBehavior):
    def next_node(self, pos_node: str, adj: dict) -> str:
        return pos_node


def _identify_center_rail_nodes(nodes: dict, n_lanes: int = 4,
                                center_lanes: int = 2) -> set:
    """
    Auto-detect the middle N of the horizontal rails.

    Lanes with the most nodes at the same y coordinate are treated as horizontal rails.
    Return only the middle ones (center_lanes count).
    n_lanes=4, center_lanes=2 → pick the 2 inner ones out of 2 outer / 2 inner.
    """
    from collections import Counter
    if not nodes:
        return set()
    yc = Counter(round(n.y, 1) for n in nodes.values())
    top = [y for y, _ in sorted(yc.items(), key=lambda kv: (-kv[1], kv[0]))[:n_lanes]]
    top_sorted = sorted(top)
    # treat the two ends as outer and extract the middle center_lanes ones
    if len(top_sorted) <= center_lanes:
        chosen = set(top_sorted)
    else:
        drop = (len(top_sorted) - center_lanes) // 2
        chosen = set(top_sorted[drop: drop + center_lanes])
    return {name for name, nd in nodes.items() if round(nd.y, 1) in chosen}


class CenterRailRoam(IdleBehavior):
    """
    Idle behavior that keeps idle OHTs on the two (inner) horizontal rails at the layout center.

    - If pos_node is on a center rail, prefer a center-rail neighbor for the next hop
      (keep circulating on center)
    - If pos_node is off-center, pick a neighbor closer to the center y (gravitate)
    """

    def __init__(self, nodes: dict,
                 center_node_names: set = None,
                 n_lanes: int = 4, center_lanes: int = 2):
        self._nodes = nodes or {}
        if center_node_names:
            self._center = set(center_node_names)
        else:
            self._center = _identify_center_rail_nodes(
                self._nodes, n_lanes=n_lanes, center_lanes=center_lanes)
        # reference y for gravitate
        if self._center:
            ys = [self._nodes[n].y for n in self._center if n in self._nodes]
            self._center_y_mid = sum(ys) / len(ys) if ys else 0.0
        else:
            self._center_y_mid = 0.0

    def next_node(self, pos_node: str, adj: dict) -> str:
        nbrs = list(adj.get(pos_node, []))
        if not nbrs:
            return pos_node
        if not self._center:
            return random.choice(nbrs)

        if pos_node in self._center:
            on_center = [n for n in nbrs if n in self._center]
            if on_center:
                return random.choice(on_center)
            return random.choice(nbrs)

        # off-center: direction that gets closer to center_y
        def dy(n):
            nd = self._nodes.get(n)
            return abs(nd.y - self._center_y_mid) if nd else 1e9
        best_dy = min(dy(n) for n in nbrs)
        candidates = [n for n in nbrs if abs(dy(n) - best_dy) < 0.5]
        return random.choice(candidates)