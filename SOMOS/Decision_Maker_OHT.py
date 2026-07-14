"""
Decision_Maker_OHT.py  (directed-distance-based rule dispatch version)

Changes:
  - gantt.csv's planned_end is kept only for 'logging / lateness calc / tie-breaking'
  - Actual dispatch priority uses directed-graph distance cost, not planned_end
  - cost = (current OHT position -> source) + (source -> dest)
  - If adj is a directed graph, directionality is reflected as-is
  - Path cost = sum of actual coordinate (Euclidean) distances between nodes
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
        dispatch_mode: str = "HUNGARIAN_REASSIGN",
        swap_penalty: float = 30.0,
        predispatch_lookahead: float = 0.0,
        congestion_node_penalty: float = 8.0,
        congestion_edge_penalty: float = 12.0,
        buffer_block_penalty: float = 30.0,
        predictive_late_weight: float = 0.25,
        predictive_wait_weight: float = 0.50,
        zone_balance_penalty: float = 25.0,
        predictive_bonus_cap_ratio: float = 0.35,
        blocking_weight: float = 0.0,
        load_sec: float = 10.0,
        unload_sec: float = 10.0,
        urgency_weight: float = 0.0,
        urgency_late_weight: float = 1.0,
        urgency_starve_weight: float = 150.0,
        urgency_starve_ref: float = 2.0,
        urgency_cap_ratio: float = 0.6,
        idle_positioning: bool = False,
        positioning_lookahead: float = 90.0,
        congestion_cost_weight: float = 0.0,
        plan_bottleneck_boost: float = 0.0,
        plan_bottleneck_types: str = "PHOTO",
        plan_pair_yield: bool = False,
        plan_pair_yield_factor: float = 1.5,
        plan_lookahead_positioning: bool = False,
        plan_congestion_weight: float = 0.0,
        plan_bucket_sec: float = 90.0,
        plan_tasks_csv: str = "",
        plan_congestion_cap_ratio: float = 1.0,
        plan_starve_guard_sec: float = 300.0,
        plan_time_anchor: bool = True,
        plan_adaptive_w: bool = False,
        plan_w_min: float = 0.0,
        plan_w_max: float = 4.0,
        plan_w_blk_ref: float = 0.30,
        rail_conflict_weight: float = 1.0,
        rail_merge_weight: float = 1.0,
        rail_null_action: bool = False,
        rail_congestion_threshold: float = 4.0,
        rail_lookahead: float = 300.0,
        rail_conflict_cap_ratio: float = 1.0,
        rail_starve_guard_sec: float = 300.0,
        rail_reassign: bool = False,
        rail_adaptive_w: bool = False,
        rail_w_min: float = 1.0,
        rail_w_max: float = 8.0,
        rail_idle_positioning: bool = False,
        slack_defer: bool = False,
        slack_defer_threshold: float = 0.0,
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
        # "GREEDY" | "HUNGARIAN" | "HUNGARIAN_REASSIGN"
        self.dispatch_mode = str(dispatch_mode or "HUNGARIAN_REASSIGN").upper()
        # patch to prevent swap thrashing during REASSIGN (sec)
        self.swap_penalty = max(0.0, float(swap_penalty))
        # if an in-process job's remaining processing time ≤ predispatch_lookahead, send an OHT in advance
        self.predispatch_lookahead = max(0.0, float(predispatch_lookahead))

        # ── New Empty OHT Dispatching score parameters ───────────────
        # CONGESTION_AWARE_HEURISTIC:
        #   Look at current OHT position / entry intent / reserved source·dest to avoid congested paths/destinations.
        # PREDICTIVE_DISPATCHING:
        #   Consider not only distance but also planned_end lateness and source out_buffer waiting pressure.
        self.congestion_node_penalty = max(0.0, float(congestion_node_penalty))
        self.congestion_edge_penalty = max(0.0, float(congestion_edge_penalty))
        self.buffer_block_penalty = max(0.0, float(buffer_block_penalty))
        self.predictive_late_weight = max(0.0, float(predictive_late_weight))
        self.predictive_wait_weight = max(0.0, float(predictive_wait_weight))
        # Prevents empty OHTs from all crowding into a specific machine island / corridor.
        # A new task near an already-assigned task's source/dest incurs extra cost.
        self.zone_balance_penalty = max(0.0, float(zone_balance_penalty))
        # If the predictive bonus completely dominates distance cost, everything crowds one source/out_buffer, so cap it.
        # e.g. 0.35 → allow bonus up to at most 35% of the travel cost.
        self.predictive_bonus_cap_ratio = max(0.0, min(0.95, float(predictive_bonus_cap_ratio)))
        # ── blocking-externality heuristic ─────────────────────────────
        # Cost of an OHT blocking other OHTs that must pass through the node while it
        # dwells at source(load)·dest(unload) (single-track rail). Added to _dispatch_cost if weight>0.
        self.blocking_weight = max(0.0, float(blocking_weight))
        self.load_sec = max(0.0, float(load_sec))      # source dwell
        self.unload_sec = max(0.0, float(unload_sec))  # dest dwell
        # ── C2: urgency / starvation priority dispatch ────────────────────────────────
        # cost = travel_time − urgency. urgency = late_w·(lateness vs. plan)
        #                                   + starve_w·(work shortfall at dest machine)
        # Replaces Hungarian's "min distance" objective with "throughput protection (starvation avoidance)".
        self.urgency_weight = max(0.0, float(urgency_weight))
        self.urgency_late_weight = max(0.0, float(urgency_late_weight))
        self.urgency_starve_weight = max(0.0, float(urgency_starve_weight))
        self.urgency_starve_ref = max(0.0, float(urgency_starve_ref))
        self.urgency_cap_ratio = max(0.0, min(0.95, float(urgency_cap_ratio)))
        # ── C1: demand-predictive idle positioning ────────────────────────────────
        # Drift idle (unassigned) OHTs toward machines that will soon produce a pickup (roam_target).
        # Not bound to a task → keeps the reactive pool. Uses idle time Hungarian does not exploit.
        self.idle_positioning = bool(idle_positioning)
        self.positioning_lookahead = max(0.0, float(positioning_lookahead))
        # ── congestion-aware cost ──────────────────────────────────────
        # Add *path congestion delay* to the cost Hungarian uses (free-flow travel time) to
        # approximate 'actual (congestion-reflected) travel time'. Added to _dispatch_cost if weight>0.
        # (hand-crafted v0 of GAT — no learning, based on current OHT occupancy density)
        self.congestion_cost_weight = max(0.0, float(congestion_cost_weight))
        # ── PLAN_PRIORITY mode (slack priority sort + sequential commit) ─────────
        self.plan_bottleneck_boost = max(0.0, float(plan_bottleneck_boost))   # extension 2
        self.plan_bottleneck_types = {s.strip().upper()
                                      for s in str(plan_bottleneck_types).split(",") if s.strip()}
        self.plan_pair_yield = bool(plan_pair_yield)                          # yield level 2
        self.plan_pair_yield_factor = max(1.0, float(plan_pair_yield_factor))
        self.plan_lookahead_positioning = bool(plan_lookahead_positioning)    # extension 1
        # ── ★ Plan-congestion-aware Hungarian (plan-induced spatiotemporal congestion) ──
        self.plan_congestion_weight = max(0.0, float(plan_congestion_weight))
        self.plan_bucket_sec = max(1.0, float(plan_bucket_sec))
        self.plan_tasks_csv = str(plan_tasks_csv or "")
        self.plan_congestion_cap_ratio = max(0.0, float(plan_congestion_cap_ratio))
        self.plan_starve_guard_sec = float(plan_starve_guard_sec)
        self.plan_time_anchor = bool(plan_time_anchor)
        self.plan_adaptive_w = bool(plan_adaptive_w)
        self.plan_w_min = max(0.0, float(plan_w_min))
        self.plan_w_max = max(self.plan_w_min, float(plan_w_max))
        self.plan_w_blk_ref = max(1e-6, float(plan_w_blk_ref))
        self._plan_w_cur = float(plan_congestion_weight)   # effective pw for this round
        self._plan_density = None   # lazy build: {edge_key: {bucket: count}}

        # ── RAIL_COORD mode (time-space reservation conflict cost) ────────────────
        self.rail_conflict_weight = max(0.0, float(rail_conflict_weight))     # mechanism 1 weight
        self.rail_merge_weight = max(1.0, float(rail_merge_weight))           # mechanism 2 merge weight
        self.rail_null_action = bool(rail_null_action)                       # mechanism 3
        self.rail_congestion_threshold = float(rail_congestion_threshold)    # edge concurrent-reservation threshold
        self.rail_lookahead = max(1.0, float(rail_lookahead))                # reservation horizon (sec)
        self.rail_conflict_cap_ratio = max(0.0, float(rail_conflict_cap_ratio))  # conflict ≤ ratio·base
        self.rail_starve_guard_sec = float(rail_starve_guard_sec)   # tasks this late are exempt from conflict
        self.rail_reassign = bool(rail_reassign)                             # RAIL+REASSIGN toggle
        # adaptive w: dynamically tune conflict_weight by rail occupancy (busy) (rule-based cap experiment before ML)
        self.rail_adaptive_w = bool(rail_adaptive_w)
        self.rail_w_min = max(0.0, float(rail_w_min))
        self.rail_w_max = max(self.rail_w_min, float(rail_w_max))
        self._rail_w_cur = float(rail_conflict_weight)   # effective w for this round (log/debug)
        self.rail_idle_positioning = bool(rail_idle_positioning)   # B-3
        self._rail_table = None       # per-round time-space reservation table (conflict 0 if absent)
        self._merge_node_set = None   # cache
        self._rel_iv_cache = {}       # (start,goal)->relative edge interval cache (speed)
        # ── hard slack-defer (defer dispatch of push-able lots this round → less input → less congestion) ──
        self.slack_defer = bool(slack_defer)
        self.slack_defer_threshold = float(slack_defer_threshold)
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

    def _traffic_snapshot(self, ignore_vehicle=None):
        """Convert current Empty/Loaded OHT node occupancy and next-hop entry intent into a simple congestion measure."""
        node_load = {}
        edge_load = {}
        for v in self.vehicles:
            if v is ignore_vehicle:
                continue
            pos = getattr(v, "pos_node", None)
            if pos:
                node_load[pos] = node_load.get(pos, 0.0) + 1.0
            nxt = getattr(v, "next_hop_intent", None)
            if pos and nxt:
                node_load[nxt] = node_load.get(nxt, 0.0) + 0.7
                edge_load[(pos, nxt)] = edge_load.get((pos, nxt), 0.0) + 1.0

            # Already-assigned source/dest are likely to draw OHTs in the near future, so reflect as a weak reservation load
            t = getattr(v, "assigned_task", None)
            if t is not None:
                node_load[getattr(t, "source_name", "")] = node_load.get(getattr(t, "source_name", ""), 0.0) + 0.5
                node_load[getattr(t, "dest_name", "")] = node_load.get(getattr(t, "dest_name", ""), 0.0) + 0.3
        return node_load, edge_load

    def _path_congestion_penalty(self, path: List[str], ignore_vehicle=None, empty_leg: bool = True) -> float:
        """Convert node/edge congestion along the path into a per-second penalty. Applied more strongly to the empty leg."""
        if not path or len(path) <= 1:
            return 0.0
        node_load, edge_load = self._traffic_snapshot(ignore_vehicle=ignore_vehicle)
        node_penalty = 0.0
        edge_penalty = 0.0

        # Exclude the start node (the veh itself is there); evaluate only nodes passed afterward
        for nd in path[1:]:
            node_penalty += node_load.get(nd, 0.0) * self.congestion_node_penalty
        for u, v in zip(path[:-1], path[1:]):
            edge_penalty += edge_load.get((u, v), 0.0) * self.congestion_edge_penalty

        # Reducing empty-vehicle travel is the crux of Empty OHT dispatching, so weight the empty leg
        weight = 1.0 if empty_leg else 0.5
        return weight * (node_penalty + edge_penalty)

    def _buffer_pressure_penalty(self, task: TransportTask) -> float:
        """If the destination in_buffer is full, actual unload delay is likely, so penalize in advance."""
        dest_machine = self.machines.get(task.dest_name)
        if dest_machine is None:
            return 0.0
        cap = float(getattr(getattr(dest_machine, "machine_config", None), "input_buffer_cap", 1) or 1)
        used = float(len(getattr(dest_machine, "in_buffer", [])) + getattr(dest_machine, "reserved_in_slots", 0))
        fill_ratio = used / max(cap, 1.0)
        if fill_ratio < 1.0:
            return self.buffer_block_penalty * fill_ratio
        return self.buffer_block_penalty * (2.0 + fill_ratio)

    def _node_xy(self, name: str):
        """Convert a machine name or node name to coordinates. Returns None on failure."""
        node_name = self._resolve_node_name(name)
        node = self.nodes.get(node_name)
        if node is None:
            return None
        return float(node.x), float(node.y)

    def _near_assigned_task_penalty(self, task: TransportTask, ignore_vehicle=None) -> float:
        """
        Even without the same source/dest, if empty OHTs crowd into the same island / nearby
        machines, they queue up along the central rail. If a new task arises near an
        already-assigned task's source/dest, apply zone_balance_penalty to spread them out.
        """
        if self.zone_balance_penalty <= 0:
            return 0.0
        src_xy = self._node_xy(task.source_name)
        dst_xy = self._node_xy(task.dest_name)
        if src_xy is None and dst_xy is None:
            return 0.0

        # Treat within 7 cells (cell_size) as the same island / adjacent corridor
        cell = float(getattr(self.jssp_cfg, "cell_size", 10.0) or 10.0)
        near_radius = 7.0 * cell
        pressure = 0.0

        for v in self.vehicles:
            if v is ignore_vehicle:
                continue
            t = getattr(v, "assigned_task", None)
            if t is None:
                continue
            for xy, other_name, weight in (
                (src_xy, getattr(t, "source_name", None), 1.0),
                (dst_xy, getattr(t, "dest_name", None), 0.7),
            ):
                if xy is None or not other_name:
                    continue
                oxy = self._node_xy(other_name)
                if oxy is None:
                    continue
                if math.hypot(xy[0] - oxy[0], xy[1] - oxy[1]) <= near_radius:
                    pressure += weight

        return pressure * self.zone_balance_penalty

    def _predictive_priority_bonus(self, task: TransportTask, now: float = None) -> float:
        """Bonus subtracted from cost to pick up late/backed-up sources first."""
        if now is None:
            now = self.env.now()
        lateness = max(0.0, now - float(task.planned_end))
        source_machine = self.machines.get(task.source_name)
        out_q = len(getattr(source_machine, "out_buffer", [])) if source_machine is not None else 0
        # The bonus is clamped inside dispatch cost to avoid an overly large negative cost.
        return self.predictive_late_weight * lateness + self.predictive_wait_weight * float(out_q)

    # ── blocking-externality heuristic ────────────────────────────────
    def _path_node_etas(self, start_node: str, goal_node: str):
        """List of arrival ETA (cumulative travel time) at each node on the shortest start->goal path [(node, eta)]."""
        path = self._path_nodes_cached(start_node, goal_node)
        out = []
        t = 0.0
        for a, b in zip(path[:-1], path[1:]):
            eid = self.uv_to_eid.get((a, b))
            if eid is not None and eid in getattr(self, 'edges', {}):
                t += float(self.edges[eid].travel_time)
            elif a in self.nodes and b in self.nodes:
                t += math.hypot(self.nodes[a].x - self.nodes[b].x,
                                self.nodes[a].y - self.nodes[b].y)
            else:
                t += 1.0
            out.append((b, t))
        return out

    # ── RAIL_COORD: time-space reservation-based conflict cost ────────────────────
    def _defer_by_slack(self, ready_tasks, now):
        """hard-defer: tasks that are 'push-able' in the gantt (large slack) are excluded from this round.
        slack = planned_start(next op) − now − planned_travel. Deferred if slack>threshold.
        As now advances, slack shrinks → eventually dispatched (JIT). → fewer concurrently-moving OHTs → less rail congestion."""
        if not self.slack_defer or self.schedule is None:
            return ready_tasks
        kept = []
        for t in ready_tasks:
            op = self.schedule.op(getattr(t, "lot_id", ""), getattr(t, "step_no", 0))
            ps = op.planned_start if op is not None else now
            slack = ps - now - float(getattr(t, "planned_travel", 0.0))
            if slack > self.slack_defer_threshold:
                continue   # still slack → defer (reconsider next round)
            kept.append(t)
        return kept

    def _merge_nodes(self):
        """Set of nodes with in-degree≥2 or out-degree≥2 (merge/junction). Computed once and cached."""
        if self._merge_node_set is not None:
            return self._merge_node_set
        indeg, outdeg = defaultdict(int), defaultdict(int)
        for u, nbrs in self.adj.items():
            outdeg[u] += len(nbrs)
            for v in nbrs:
                indeg[v] += 1
        nodes = set(indeg) | set(outdeg)
        self._merge_node_set = {n for n in nodes if indeg[n] >= 2 or outdeg[n] >= 2}
        return self._merge_node_set

    def _rel_edge_intervals(self, start, goal):
        """*Relative* edge intervals of the start→goal path [(edge_key, rel_enter, rel_exit)]. Cached by (start,goal)."""
        key = (start, goal)
        c = self._rel_iv_cache.get(key)
        if c is not None:
            return c
        path = self._path_nodes_cached(start, goal)
        c = []
        t = 0.0
        for a, b in zip(path[:-1], path[1:]):
            eid = self.uv_to_eid.get((a, b))
            if eid is not None and eid in getattr(self, 'edges', {}):
                dt = float(self.edges[eid].travel_time)
            elif a in self.nodes and b in self.nodes:
                dt = math.hypot(self.nodes[a].x - self.nodes[b].x,
                                self.nodes[a].y - self.nodes[b].y)
            else:
                dt = 1.0
            c.append(((a, b), t, t + dt))
            t += dt
        self._rel_iv_cache[key] = c
        return c

    def _path_edge_intervals(self, start, goal, t0):
        """start→goal edge occupancy intervals [(edge_key, enter, exit)] (absolute time, t0 offset)."""
        return [(ek, t0 + re, t0 + rx)
                for ek, re, rx in self._rel_edge_intervals(start, goal)]

    def _build_reservation_table(self):
        """Reserve the remaining paths of in-flight (task-holding) OHTs as per-edge time intervals.
        edge_key -> [(enter, exit, veh_id)]. Basis for single-track rail conflict prediction."""
        now = self.env.now()
        horizon = now + self.rail_lookahead
        table = defaultdict(list)
        for v in self.vehicles:
            task = getattr(v, "assigned_task", None)
            pos = getattr(v, "pos_node", None)
            if task is None or not pos:
                continue
            src = self._resolve_node_name(getattr(task, "source_name", ""))
            dst = self._resolve_node_name(getattr(task, "dest_name", ""))
            if getattr(v, "cargo_job", None) is not None:
                segs = self._path_edge_intervals(pos, dst, now) if (dst and pos != dst) else []
            else:
                segs = []
                t = now
                if src and pos != src:
                    s1 = self._path_edge_intervals(pos, src, now)
                    segs += s1
                    t = (s1[-1][2] if s1 else now) + self.load_sec
                if src and dst and src != dst:
                    segs += self._path_edge_intervals(src, dst, t)
            for ek, en, ex in segs:
                if en <= horizon:
                    table[ek].append((en, ex, id(v)))
        return table

    def _rail_conflict_cost(self, veh, task, empty_move):
        """Cost (sec) proportional to how much the candidate (veh,task) path *overlaps in time* with the reservation table. Merge nodes weighted."""
        table = self._rail_table
        if not table:
            return 0.0
        now = self.env.now()
        src = self._resolve_node_name(task.source_name)
        dst = self._resolve_node_name(task.dest_name)
        pos = veh.pos_node
        intervals = []
        if src and pos != src:
            intervals += self._path_edge_intervals(pos, src, now)
        tload = now + empty_move + self.load_sec
        if src and dst and src != dst:
            intervals += self._path_edge_intervals(src, dst, tload)
        merge = self._merge_nodes()
        vid = id(veh)
        conflict = 0.0
        for ek, en, ex in intervals:
            for (r_en, r_ex, rid) in table.get(ek, ()):
                if rid == vid:
                    continue
                if min(ex, r_ex) > max(en, r_en):          # time overlap
                    w = self.rail_merge_weight if ek[1] in merge else 1.0
                    conflict += w * (ex - en)              # wait likelihood proportional to that edge's traversal time
        return conflict

    # ── ★ Plan-congestion-aware Hungarian: plan-induced spatiotemporal congestion map ──
    def _get_plan_density(self):
        """Offline once: accumulate planned transports (transport_tasks.csv) into rail edge×time_bucket occupancy
        → D[edge_key][bucket] = number of planned transports crossing that segment in that time window.
        Unlike RAIL's in-flight reservation table, a static congestion prior built from the *entire future schedule*."""
        if self._plan_density is not None:
            return self._plan_density
        import csv as _csv, os as _os
        D = defaultdict(lambda: defaultdict(float))
        bsec = self.plan_bucket_sec
        path = self.plan_tasks_csv
        n_ok = n_skip = 0
        if path and _os.path.exists(path):
            with open(path, newline="") as f:
                for row in _csv.DictReader(f):
                    src = self._resolve_node_name(row.get("source_machine", ""))
                    dst = self._resolve_node_name(row.get("dest_machine", ""))
                    try:
                        t0 = float(row.get("planned_ready_sec", "") or 0.0)
                    except ValueError:
                        t0 = 0.0
                    if not src or not dst or src == dst or src not in self.nodes or dst not in self.nodes:
                        n_skip += 1
                        continue
                    for ek, en, ex in self._path_edge_intervals(src, dst, t0):
                        b = int(en // bsec)
                        D[ek][b] += 1.0
                    n_ok += 1
        # freeze into a plain dict (fast lookup)
        self._plan_density = {ek: dict(bk) for ek, bk in D.items()}
        print(f"[PLAN-CONGESTION] transport {n_ok} mapped / {n_skip} skipped, "
              f"{len(self._plan_density)} edges, bucket={bsec}s", flush=True)
        return self._plan_density

    def _plan_congestion_cost(self, veh, task, empty_move):
        """How congested the candidate (veh,task) path is *at the time I traverse it*, per the plan (sec).
        Prices each edge of the empty(pos→src)·loaded(src→dst) path by the planned density of its ETA bucket.
        Isomorphic to RAIL's _rail_conflict_cost, but queries the plan density table instead of the in-flight reservation table.

        ★Time anchor (drift fix):
          plan_time_anchor=True → align to the task's own plan-time (pr=planned_end−planned_travel,
            =planned_ready_sec used to build the density). No misfire even if realized lags the plan.
          False → v0 (realized ETA=now based). Drifts when realized lags.
        """
        D = self._get_plan_density()
        if not D:
            return 0.0
        now = self.env.now()
        src = self._resolve_node_name(task.source_name)
        dst = self._resolve_node_name(task.dest_name)
        pos = veh.pos_node
        bsec = self.plan_bucket_sec
        if self.plan_time_anchor:
            # plan-time alignment: loaded starts at pr (same as density), empty arrives at pr.
            pr = float(getattr(task, "planned_end", now)) - float(getattr(task, "planned_travel", 0.0))
            t_empty0 = pr - empty_move
            t_loaded0 = pr
        else:
            t_empty0 = now
            t_loaded0 = now + empty_move + self.load_sec
        intervals = []
        if src and pos != src:
            intervals += self._path_edge_intervals(pos, src, t_empty0)
        if src and dst and src != dst:
            intervals += self._path_edge_intervals(src, dst, t_loaded0)
        cost = 0.0
        for ek, en, ex in intervals:
            bk = D.get(ek)
            if not bk:
                continue
            dens = bk.get(int(en // bsec), 0.0)
            if dens > 0.0:
                cost += dens * (ex - en)   # planned density × that edge's traversal time
        return cost

    def _rail_peak_concurrency(self):
        """Rail jam signal: max near-future concurrent reservations on any edge (for null-action decision)."""
        table = self._rail_table
        if not table:
            return 0
        peak = 0
        for ek, lst in table.items():
            if len(lst) <= peak:
                continue
            evts = []
            for en, ex, _ in lst:
                evts.append((en, 1))
                evts.append((ex, -1))
            evts.sort()
            cur = 0
            for _, d in evts:
                cur += d
                if cur > peak:
                    peak = cur
        return peak

    def _build_blocking_node_eta(self):
        """Once per dispatch round: ETA list of in-flight (task-holding) OHTs passing through each node.
        node -> sorted [(eta, oht_id)]. Idle roaming OHTs are excluded since their path is undetermined (main traffic=loaded)."""
        node_eta = defaultdict(list)
        for v in self.vehicles:
            task = getattr(v, "assigned_task", None)
            if task is None:
                continue
            pos = getattr(v, "pos_node", None)
            if not pos:
                continue
            # if loaded, heads to dest; otherwise heads to the next destination source
            if getattr(v, "cargo_job", None) is not None:
                tgt = self._resolve_node_name(getattr(task, "dest_name", ""))
            else:
                tgt = self._resolve_node_name(getattr(task, "source_name", ""))
            if not tgt or pos == tgt:
                continue
            for nd, eta in self._path_node_etas(pos, tgt):
                node_eta[nd].append((eta, id(v)))
        for nd in node_eta:
            node_eta[nd].sort()
        return node_eta

    def _get_blocking_node_eta(self):
        now = self.env.now()
        if getattr(self, "_blk_round_t", None) != now:
            self._blk_eta = self._build_blocking_node_eta()
            self._blk_round_t = now
        return self._blk_eta

    def _blocking_externality_penalty(self, veh, empty_move, loaded_move, task):
        """While veh takes the task and dwells at source(load)·dest(unload),
        cost = (number of other OHTs that must pass that node) × dwell. (single-track rail blocking cost)
        The later veh arrives, the fewer OHTs it blocks behind it → lower penalty → the global optimum prefers a later OHT."""
        w = self.blocking_weight
        if w <= 0.0:
            return 0.0
        node_eta = self._get_blocking_node_eta()
        src = self._resolve_node_name(task.source_name)
        dst = self._resolve_node_name(task.dest_name)
        vid = id(veh)
        pen = 0.0
        # source arrival = after empty_move; dest arrival = after empty_move + load + loaded_move
        for N, t_arr, dwell in ((src, empty_move, self.load_sec),
                                (dst, empty_move + self.load_sec + loaded_move, self.unload_sec)):
            arr = node_eta.get(N)
            if not arr or dwell <= 0.0:
                continue
            etas = [e for e, _ in arr]
            lo = bisect.bisect_left(etas, t_arr)
            hi = bisect.bisect_right(etas, t_arr + dwell)
            cnt = hi - lo
            for _e, ov in arr[lo:hi]:   # exclude veh itself (REASSIGN candidate)
                if ov == vid:
                    cnt -= 1
            if cnt > 0:
                pen += dwell * cnt
        return w * pen

    def _urgency_bonus(self, task) -> float:
        """C2: how urgent this task is → bonus to subtract from cost.
          - lateness : how late this op is vs. plan (planned_end) (lot is backed up)
          - starve   : work shortfall at the dest (next) machine → the closer to starving, the more urgent (directly tied to throughput)
        Hungarian only sees 'distance', but this bonus prioritizes 'starvation/lateness'."""
        if self.urgency_weight <= 0.0:
            return 0.0
        now = self.env.now()
        lateness = max(0.0, now - float(getattr(task, "planned_end", now)))
        starve = 0.0
        dest = self.machines.get(self._resolve_node_name(getattr(task, "dest_name", "")))
        if dest is not None and not getattr(dest, "_broken", False):
            # A broken machine does not count as starving (delivery can't be processed anyway) → prioritize only recovered/starving machines
            work = len(getattr(dest, "in_buffer", []))
            if getattr(dest, "processing_job", None) is not None:
                work += 1
            starve = max(0.0, self.urgency_starve_ref - work)   # work shortfall (larger = starvation imminent)
        return self.urgency_weight * (self.urgency_late_weight * lateness
                                      + self.urgency_starve_weight * starve)

    def _dispatch_cost(self, veh, task: TransportTask) -> Tuple[float, float, float]:
        """
        Cost of Empty OHT -> source, Loaded OHT -> dest.

        dispatch_mode switch:
          - GREEDY/HUNGARIAN/HUNGARIAN_REASSIGN: original distance/time based
          - CONGESTION_AWARE_HEURISTIC: adds path congestion + destination buffer congestion penalty
          - PREDICTIVE_DISPATCHING: applies a priority bonus to process late/waiting sources first
        If blocking_weight>0, the blocking-externality penalty is added in all modes.
        """
        empty_move = self._path_travel_time(veh.pos_node, task.source_name)
        loaded_move = self._path_travel_time(task.source_name, task.dest_name)

        if math.isinf(empty_move) or math.isinf(loaded_move):
            return float("inf"), empty_move, loaded_move

        total = empty_move + loaded_move
        mode = str(getattr(self, "dispatch_mode", "")).upper()

        if mode == "CONGESTION_AWARE_HEURISTIC":
            empty_path = self._path_nodes_cached(veh.pos_node, task.source_name)
            loaded_path = self._path_nodes_cached(task.source_name, task.dest_name)
            total += self._path_congestion_penalty(empty_path, ignore_vehicle=veh, empty_leg=True)
            total += self._path_congestion_penalty(loaded_path, ignore_vehicle=veh, empty_leg=False)
            total += self._buffer_pressure_penalty(task)
            total += self._near_assigned_task_penalty(task, ignore_vehicle=veh)

        elif mode == "PREDICTIVE_DISPATCHING":
            # Separate from predispatch: among already-ready tasks, prioritize late ones and
            # ones whose source out_buffer is backed up.
            # But if the bonus grows too large, everything crowds one source/island, so add cap + zone penalty.
            base = empty_move + loaded_move
            bonus = min(self._predictive_priority_bonus(task), self.predictive_bonus_cap_ratio * base)
            total = total - bonus
            total += self._near_assigned_task_penalty(task, ignore_vehicle=veh)
            total = max(0.2 * base, total)

        # blocking-externality heuristic (common to all modes, only when weight>0)
        total += self._blocking_externality_penalty(veh, empty_move, loaded_move, task)

        # congestion-aware cost: free-flow travel time → add path congestion delay (approximates 'actual travel time')
        #   Makes Hungarian assign on a *congestion-reflected* cost rather than the *wrong* free-flow.
        if self.congestion_cost_weight > 0.0:
            empty_path = self._path_nodes_cached(veh.pos_node, task.source_name)
            loaded_path = self._path_nodes_cached(task.source_name, task.dest_name)
            total += self.congestion_cost_weight * (
                self._path_congestion_penalty(empty_path, ignore_vehicle=veh, empty_leg=True)
                + self._path_congestion_penalty(loaded_path, ignore_vehicle=veh, empty_leg=False))

        # ★ Plan-congestion: add plan-induced spatiotemporal congestion to the cost (approximates 'actual travel time').
        #   Isomorphic to RAIL but looks at the *entire future schedule* instead of in-flight (proactive). Layered on top of HUNGARIAN.
        if self.plan_congestion_weight > 0.0 or self.plan_adaptive_w:
            _pc = self._plan_congestion_cost(veh, task, empty_move)
            _padd = self._plan_w_cur * _pc
            _base = empty_move + loaded_move
            # starve guard: tasks much later than plan are exempt (avoid permanently shunning congested paths → deadlock).
            _plate = self.env.now() - float(getattr(task, "planned_end", self.env.now()))
            if _plate > self.plan_starve_guard_sec:
                _padd = 0.0
            total += min(_padd, self.plan_congestion_cap_ratio * _base)

        # RAIL_COORD: add cost proportional to time conflict with time-space reservations (approximates 'actual travel time').
        #   Reflects, at dispatch time, the 'OHTs colliding on the rail' that Hungarian cannot see.
        if self.rail_conflict_weight > 0.0 and getattr(self, "_rail_table", None):
            _cf = self._rail_conflict_cost(veh, task, empty_move)
            _add = self._rail_w_cur * _cf
            base = empty_move + loaded_move
            # starvation guard: tasks later than plan by ≥ starve_guard are exempt from conflict.
            #   Without it, congested-path tasks are shunned forever → the lot stays in out_buffer → machine blocks → deadlock.
            _late = self.env.now() - float(getattr(task, "planned_end", self.env.now()))
            if _late > self.rail_starve_guard_sec:
                _add = 0.0
            total += min(_add, self.rail_conflict_cap_ratio * base)

        # C2: urgency/starvation priority — subtract from cost (only up to a fraction of base), prevent negatives
        if self.urgency_weight > 0.0:
            base = empty_move + loaded_move
            ub = min(self._urgency_bonus(task), self.urgency_cap_ratio * base)
            total = max(0.2 * base, total - ub)

        return total, empty_move, loaded_move

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

    def _assign_idle_positioning(self, claimed_sources, now: float):
        """C1: set roam_target of idle (unassigned) OHTs toward 'machines that will soon produce a pickup'.
        Since it is *not* bound to a task (can_accept_dispatch stays True), reactive dispatch can grab it anytime.
        Uses idle time Hungarian ignores to reduce future pickups' empty travel in advance."""
        idle = [v for v in self.vehicles if v.can_accept_dispatch()]
        if not idle:
            return
        # Fresh each round: clear existing roam_target then reassign (prevent stale)
        for v in idle:
            v.roam_target = None
        # anticipated source = has out_buffer (immediate), or a processing non-final job completes within lookahead
        cand = []   # (node, remaining)
        for mname, m in self.machines.items():
            if mname in claimed_sources:
                continue
            node = self._resolve_node_name(mname)
            if not node:
                continue
            if getattr(m, "out_buffer", None):
                cand.append((node, 0.0))
                continue
            pjob = getattr(m, "processing_job", None)
            if pjob is None or self.schedule is None:
                continue
            lot = getattr(pjob, "lot_id", "")
            cs = int(getattr(pjob, "step_no", 0))
            if not lot or not self.schedule.next_machine_after(lot, cs):
                continue   # final step → no pickup
            st = getattr(m, "_processing_start_time", None)
            du = getattr(m, "_processing_duration", None)
            rem = ((st + du) - now) if (st is not None and du is not None) else 0.0
            if rem > self.positioning_lookahead:
                continue
            cand.append((node, max(0.0, rem)))
        if not cand:
            return
        cand.sort(key=lambda nr: nr[1])           # cover soonest pickups (small remaining) first
        used = set()
        for node, _ in cand:                       # one OHT per target (spread out)
            if not idle:
                break
            # assign the idle OHT closest to this target
            best = None
            bestc = float("inf")
            for v in idle:
                if getattr(v, "roam_target", None) is not None:
                    continue
                c = self._path_travel_time(v.pos_node, node)
                if c < bestc:
                    bestc = c
                    best = v
            if best is not None:
                best.roam_target = node
                used.add(node)

    def _assign_idle_positioning_railaware(self, claimed_sources, now: float):
        """B-3: reservation-aware idle positioning.
        Same 'drift empty vehicles toward machines that will soon produce a pickup' as H4 (_assign_idle_positioning), but:
          (1) don't send if the route conflicts heavily with the reservation table (_rail_table) (prevent rail blocking)
          (2) add the sent vehicle's route to the reservation table too → the next empty vehicle avoids that path (prevent clustering)
        Removes H4's failure cause (movement *adds* congestion). Not a commit (roam_target only, keeps reactive pool)."""
        idle = [v for v in self.vehicles if v.can_accept_dispatch()]
        if not idle:
            return
        for v in idle:
            v.roam_target = None
        # candidate destinations: has out_buffer (immediate), or a processing non-final job completes within lookahead
        cand = []
        for mname, m in self.machines.items():
            if mname in claimed_sources:
                continue
            node = self._resolve_node_name(mname)
            if not node:
                continue
            if getattr(m, "out_buffer", None):
                cand.append((node, 0.0))
                continue
            pjob = getattr(m, "processing_job", None)
            if pjob is None or self.schedule is None:
                continue
            lot = getattr(pjob, "lot_id", "")
            cs = int(getattr(pjob, "step_no", 0))
            if not lot or not self.schedule.next_machine_after(lot, cs):
                continue
            st = getattr(m, "_processing_start_time", None)
            du = getattr(m, "_processing_duration", None)
            rem = ((st + du) - now) if (st is not None and du is not None) else 0.0
            if rem > self.positioning_lookahead:
                continue
            cand.append((node, max(0.0, rem)))
        if not cand:
            return
        cand.sort(key=lambda nr: nr[1])
        # temporary reservation table to accumulate empty-vehicle movement (added on top of a copy of _rail_table)
        table = self._rail_table if self._rail_table is not None else {}
        sent_targets = set()
        for node, _ in cand:
            if node in sent_targets:
                continue                          # another empty vehicle already heads there → spread out
            # among candidates for this destination, pick the empty vehicle with 'least route conflict'
            best = None
            best_key = None
            for v in idle:
                if getattr(v, "roam_target", None) is not None or v.pos_node == node:
                    continue
                segs = self._path_edge_intervals(v.pos_node, node, now)
                conflict = 0.0
                for ek, en, ex in segs:
                    for (r_en, r_ex, _id) in table.get(ek, ()):
                        if min(ex, r_ex) > max(en, r_en):
                            conflict += (ex - en)
                travel = segs[-1][2] - now if segs else 0.0
                key = (conflict, travel)          # low-conflict, nearby vehicle
                if best_key is None or key < best_key:
                    best_key = key
                    best = v
                    best_segs = segs
            if best is None:
                continue
            # if conflict is excessive (blocking greater than travel time), don't send → just stay parked (avoid adding congestion)
            if best_key[0] > best_key[1] + 1e-6:
                continue
            best.roam_target = node
            sent_targets.add(node)
            # accumulate the sent vehicle's route into the reservation table (next empty vehicle avoids it)
            tbl = dict(table)
            for ek, en, ex in best_segs:
                tbl.setdefault(ek, list(table.get(ek, ())))
                tbl[ek] = list(tbl[ek]) + [(en, ex, id(best))]
            table = tbl

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

    def _assign_coverage_positioning(self, now, window=900.0, grid=0.0, prior_weight=1.0):
        """(A) Gantt demand-density COVERAGE — match idle OHTs' *spatial distribution* to future
        pickup demand density.
          - Unlike per-source positioning (mean, 1:1 with imminent machines, zero-sum), matches the
            *distribution/coverage* to eliminate hot regions that 'have demand but no nearby empty
            vehicle' (=cause of the source_wait tail) → aims to shrink the tail.
          - Only idle vehicles in surplus cells are steered to deficit (hot) cells (roam_target) →
            minimal extra empty travel. Not a commit (can_accept_dispatch stays True → reactive dispatch can grab it)."""
        idle = [v for v in self.vehicles if v.can_accept_dispatch()]
        if len(idle) < 2:
            return
        # ── 1) per-machine demand weight = realized imminence (schedule) + gantt geographic prior ──
        gprior = self._get_plan_source_weights()
        gmax = max(gprior.values()) if gprior else 1.0
        dem = {}                                          # node -> weight
        for mname, m in self.machines.items():
            node = self._resolve_node_name(mname)
            if not node or node not in self.nodes:
                continue
            w = 0.0
            if getattr(m, "out_buffer", None):
                w += 3.0                                  # already waiting for pickup = top priority
            pjob = getattr(m, "processing_job", None)
            if pjob is not None and self.schedule is not None:
                lot = getattr(pjob, "lot_id", ""); cs = int(getattr(pjob, "step_no", 0))
                if lot and self.schedule.next_machine_after(lot, cs):   # next step = pickup arises
                    st = getattr(m, "_processing_start_time", None)
                    du = getattr(m, "_processing_duration", None)
                    rem = ((st + du) - now) if (st is not None and du is not None) else 0.0
                    if rem <= window:
                        w += 1.0 - max(0.0, rem) / window  # larger the more imminent
            w += prior_weight * (gprior.get(node, 0.0) / gmax)   # geographic prior
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
        # ── 5) from hottest deficit cell, assign the nearest surplus vehicle (min-max coverage = target the tail) ──
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

    def _collect_predispatch_tasks(self, now: float,
                                   reserved_sources: Set[str],
                                   reserved_dests: Set[str]) -> List[TransportTask]:
        """
        Add jobs *being processed* at machines with short remaining time to the task pool in advance.
        Time it so the op finishes just as the departing OHT reaches the source → absorb empty travel.

        Conditions:
          - predispatch_lookahead > 0
          - machine.processing_job exists
          - not the last op (a next machine exists)
          - remaining processing time ≤ predispatch_lookahead
          - the same (instance, op) key is not already assigned to another vehicle
        """
        # gantt mode: compute next machine/step from schedule (replaces the old ta01 jssp_data path)
        if self.predispatch_lookahead <= 0 or self.schedule is None:
            return []
        tasks: List[TransportTask] = []
        reserved_jobs = self._reserved_job_keys()

        for source_name, machine in self.machines.items():
            pjob = getattr(machine, "processing_job", None)
            if pjob is None:
                continue
            if source_name in reserved_sources:
                continue
            lot = getattr(pjob, "lot_id", "")
            cur_step = int(getattr(pjob, "step_no", 0))
            if not lot:
                continue
            # next machine (gantt plan). No transport if it's the last step.
            next_m = self.schedule.next_machine_after(lot, cur_step)
            if not next_m:
                continue
            dest_name = self.machine_node_map.get(next_m)
            if not dest_name or dest_name in reserved_dests:
                continue

            # dedup: when processing finishes and it enters out_buffer, op_index+1 → use current
            # op_index so it matches the ready task's completed_op (=op_index-1).
            completed_op = int(getattr(pjob, "op_index", 0))
            job_key = (int(getattr(pjob, "job_id", -1)), completed_op)
            if job_key in reserved_jobs:
                continue

            # remaining processing time (realized value exposed by the machine)
            start_t = getattr(machine, "_processing_start_time", None)
            dur = getattr(machine, "_processing_duration", None)
            if start_t is None or dur is None:
                continue
            remaining = (start_t + dur) - now
            if remaining < 0 or remaining > self.predispatch_lookahead:
                continue

            # future step / planned values
            nstep = self.schedule.next_step_no(lot, cur_step)
            nstep = nstep if nstep is not None else cur_step
            pt = self.schedule.planned_travel(lot, nstep)
            ptravel = pt if pt is not None else 0.0
            pe = self.schedule.planned_ready(lot, nstep)
            planned_end = pe if pe is not None else now

            self._task_counter += 1
            tasks.append(TransportTask(
                task_id=self._task_counter,
                job_id=int(getattr(pjob, "job_id", -1)),
                job_instance_id=int(getattr(pjob, "job_id", -1)),
                op_index=completed_op,
                source_name=source_name,
                dest_name=dest_name,
                planned_end=planned_end,
                lot_id=lot,
                step_no=nstep,
                planned_travel=ptravel,
                source_ready_time=start_t + dur,   # expected ready time
            ))

        tasks.sort(key=lambda t: (t.source_name, t.dest_name, t.job_id,
                                  t.job_instance_id or 0, t.op_index))
        return tasks

    def _collect_reassignable(self):
        """
        Reassignable vehicles + reclaimed tasks.

        Condition: assigned_task ≠ None AND cargo_job is None AND state_name ≠ "Loading"
          → a vehicle not yet started pickup (mid empty-move) can be reassigned to a new task.

        Returns
        -------
        (reassignable_vehicles, freed_tasks)
            freed_tasks is each vehicle's current task. If matching returns the same task, no change;
            if it returns a different task, it's a swap.
        """
        vehs, tasks = [], []
        for v in self.vehicles:
            task = getattr(v, "assigned_task", None)
            if task is None:
                continue
            if getattr(v, "cargo_job", None) is not None:
                continue
            if getattr(v, "state_name", "") == "Loading":
                continue
            vehs.append(v)
            tasks.append(task)
        return vehs, tasks

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
        """Build a sort key for (veh,task) candidates using a rule from DISPATCH_RULE_FUNCTIONS.md,
        and commit sequentially in ascending key order (step-by-step greedy). Smaller key = better candidate.
        rule ∈ {STD, EDD, FIFO, PRIORITY, QS_STD, BA_STD}."""
        if not idle_vehicles or not ready_tasks:
            return
        cap = float(getattr(self, "urgency_cap_ratio", 0.6) or 0.6)

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
            if rule == "QS_STD":                               # STD + queue/lateness bonus
                base = empty + loaded
                lateness = max(0.0, now - float(getattr(task, "planned_end", now)))
                priority = lateness + out_q(task)
                score = base - min(priority, cap * max(base, 1.0))
                return (score, base, empty, loaded, vid)
            if rule == "BA_STD":                               # STD + blocking externality
                base = empty + loaded
                blk = self._blocking_externality_penalty(veh, empty, loaded, task)
                return (base + blk, base, blk, empty, loaded, vid)
            return (total, empty, loaded, vid)                 # fallback

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

    # ══════════════════════════════════════════════════════════════════
    # PLAN_PRIORITY: planned-gantt slack priority sort + sequential commit
    #   Instead of Hungarian (globally optimal matching), for the most urgent task
    #   commit/remove the single min-distance idle OHT that can reach it. No cost-matrix linear_sum_assignment.
    # ══════════════════════════════════════════════════════════════════
    def _apply_plan_priority(self, idle_vehicles, ready_tasks,
                             claimed_sources, claimed_dests, now):
        if not idle_vehicles or not ready_tasks:
            return
        # 1) precompute (veh,task) base travel cost (exclude inf). With add-on weight=0, pure free-flow.
        cost = {}
        for t in ready_tasks:
            for v in idle_vehicles:
                tc, em, lm = self._dispatch_cost(v, t)
                if not math.isinf(tc):
                    cost[(id(v), id(t))] = (tc, em, lm)

        # 2) core 1 + extension 2: priority(t) = −slack(t) (+ bottleneck boost)
        #    slack = planned_start(next op) − (now + travel ETA); ETA = min(empty+loaded)
        def priority(t):
            etas = [em + lm for (vk, tk), (tc, em, lm) in cost.items() if tk == id(t)]
            if not etas:
                return float("-inf")           # no vehicle can reach → lowest
            eta = min(etas)
            op = self.schedule.op(t.lot_id, t.step_no) if self.schedule is not None else None
            ps = op.planned_start if op is not None else now
            slack = ps - (now + eta)
            pri = -slack                       # more imminent/late → higher
            if (self.plan_bottleneck_boost > 0.0 and op is not None
                    and getattr(op, "machine_type", "").upper() in self.plan_bottleneck_types):
                pri += self.plan_bottleneck_boost   # boost tasks right before a bottleneck (PHOTO etc.)
            return pri

        # 3) descending priority (tie: earlier planned_end)
        order = sorted(ready_tasks, key=lambda t: (-priority(t), t.planned_end, t.job_id))

        # 4) commit sequentially from the most urgent task + remove the committed OHT (yield)
        pool = list(idle_vehicles)
        for t in order:
            if t.source_name in claimed_sources or t.dest_name in claimed_dests:
                continue
            cand = [(v, cost[(id(v), id(t))]) for v in pool if (id(v), id(t)) in cost]
            if not cand:
                continue
            if self.plan_pair_yield:
                veh = self._pick_pair_yield(t, cand, pool, order,
                                            claimed_sources, claimed_dests, cost)
            else:
                veh = min(cand, key=lambda vc: (vc[1][0], vc[1][1], vc[1][2]))[0]
            setattr(t, "_dispatch_time", now)
            veh.assign(t)
            claimed_sources.add(t.source_name)
            claimed_dests.add(t.dest_name)
            self._log(veh.vid, t.job_id, t.op_index, "dispatch",
                      t.source_name, t.dest_name, t.planned_end,
                      max(0.0, now - t.planned_end), job_instance_id=t.job_instance_id)
            pool.remove(veh)
            if veh in idle_vehicles:
                idle_vehicles.remove(veh)
            if t in ready_tasks:
                ready_tasks.remove(t)
            if not pool:
                break

    def _pick_pair_yield(self, t, cand, pool, order,
                         claimed_sources, claimed_dests, cost):
        """Yield level 2 (1-step lookahead): avoid an OHT that is the 'only candidate (no alternative)'
        for another unassigned task (so as not to starve that task). But if the current task has only that
        vehicle, use it as protection."""
        factor = self.plan_pair_yield_factor

        def cand_set(task):
            cs = [(v, cost[(id(v), id(task))][0]) for v in pool
                  if (id(v), id(task)) in cost]
            if not cs:
                return []
            mn = min(c for _, c in cs)
            return [v for v, c in cs if c <= factor * mn + 1e-9]   # within factor = candidate

        others = [tt for tt in order if tt is not t
                  and tt.source_name not in claimed_sources
                  and tt.dest_name not in claimed_dests]
        critical = set()
        for tt in others:
            cs = cand_set(tt)
            if len(cs) == 1:           # that task's only candidate → removing it starves the task
                critical.add(id(cs[0]))
        cand_sorted = sorted(cand, key=lambda vc: (vc[1][0], vc[1][1], vc[1][2]))
        non_crit = [v for v, _ in cand_sorted if id(v) not in critical]
        if non_crit:
            return non_crit[0]         # min-distance among vehicles that don't starve others
        return cand_sorted[0][0]       # all critical → protect the current task (best effort)

    def request_dispatch(self):
        """Event-based dispatch trigger.

        Called at the moment 'state changed', such as a new pickup (loaded into a machine's
        out_buffer) or an OHT switching to idle. If the dispatcher is asleep (passivated) with
        nothing to do, wake it to re-dispatch immediately (ignored if not asleep).
        """
        if self.ispassive():
            self.activate()

    def process(self):
        mode = self.dispatch_mode
        print(f"[OHTDecisionMaker] dispatch_mode = {mode}")

        while True:
            now = self.env.now()

            # RAIL_COORD: build per-round time-space reservation table (+ mechanism 3 null action).
            #   non-REASSIGN HUNGARIAN (idle only) engine + add conflict cost to cost (else branch below).
            if mode == "RAIL_COORD":
                self._rail_table = self._build_reservation_table()
                # adaptive w: dynamically tune conflict_weight by rail occupancy (busy = task-running OHTs / total)
                if self.rail_adaptive_w and self.vehicles:
                    busy = sum(1 for v in self.vehicles
                               if getattr(v, "assigned_task", None) is not None) / len(self.vehicles)
                    self._rail_w_cur = self.rail_w_min + (self.rail_w_max - self.rail_w_min) * busy
                else:
                    self._rail_w_cur = self.rail_conflict_weight
                if self.rail_null_action and self._rail_peak_concurrency() >= self.rail_congestion_threshold:
                    # rail jam threshold exceeded → defer new dispatch this round (wait for congestion to clear)
                    self._rail_table = None
                    yield self.hold(self.dispatch_dt)
                    continue

            # ★ Plan-congestion adaptive weight: dynamically tune pw by blocked (Waiting) ratio.
            #   flowing (blocked≈0)→w_min (neutral), jam forming (blocked↑)→w_max (strong avoidance). fragile→robust.
            if self.plan_adaptive_w and self.vehicles:
                _blk = sum(1 for v in self.vehicles
                           if getattr(v, "state_name", "") in ("Waiting", "Breaked")) / len(self.vehicles)
                _ramp = min(1.0, _blk / self.plan_w_blk_ref)
                self._plan_w_cur = self.plan_w_min + (self.plan_w_max - self.plan_w_min) * _ramp
            else:
                self._plan_w_cur = self.plan_congestion_weight

            if mode == "HUNGARIAN_REASSIGN" or (mode == "RAIL_COORD" and self.rail_reassign):
                # include pre-pickup vehicles as candidates to allow mid-flight swap
                reassign_vehs, freed_tasks = self._collect_reassignable()
                freed_veh_set = {id(v) for v in reassign_vehs}

                locked_sources, locked_dests = set(), set()
                inflight_dst = Counter()
                for v in self.vehicles:
                    if id(v) in freed_veh_set:
                        continue
                    t = getattr(v, "assigned_task", None)
                    if t is None:
                        continue
                    locked_sources.add(t.source_name)
                    inflight_dst[t.dest_name] += 1
                rlx = getattr(self, "relax_machine_locks", False)
                locked_sources = _SrcGate(locked_sources, relaxed=rlx)
                locked_dests = _DestGate(self.machines, inflight_dst, relaxed=rlx)

                fresh_tasks = self._collect_ready_tasks(now, locked_sources, locked_dests)
                seen_keys = {(t.job_instance_id, t.op_index) for t in fresh_tasks}
                ready_tasks = list(fresh_tasks)
                for t in freed_tasks:
                    key = (t.job_instance_id, t.op_index)
                    if key in seen_keys:
                        continue
                    if t.source_name in locked_sources or t.dest_name in locked_dests:
                        continue
                    ready_tasks.append(t)
                    seen_keys.add(key)
                ready_tasks = self._defer_by_slack(ready_tasks, now)   # hard slack-defer

                claimed_sources = locked_sources
                claimed_dests = locked_dests
                idle_vehicles = [v for v in self.vehicles if v.can_accept_dispatch()]
                idle_vehicles.extend(reassign_vehs)

                assignments = self._select_assignments_hungarian(
                    idle_vehicles, ready_tasks, claimed_sources, claimed_dests)
                if assignments is not None:
                    self._apply_hungarian_assignments(
                        assignments, claimed_sources, claimed_dests, now)
                else:
                    self._apply_greedy(idle_vehicles, ready_tasks,
                                       claimed_sources, claimed_dests, now)
                if mode == "RAIL_COORD":
                    self._rail_table = None   # end of RAIL+REASSIGN round → release reservation table

            else:
                # HUNGARIAN (global matching only, keeps sticky) / GREEDY / RAIL_COORD (non-reassign) mode
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
                ready_tasks = self._defer_by_slack(ready_tasks, now)   # hard slack-defer
                claimed_sources = reserved_sources
                claimed_dests = reserved_dests
                idle_vehicles = [v for v in self.vehicles if v.can_accept_dispatch()]

                if mode in ("HUNGARIAN", "PREDICTIVE_DISPATCHING", "RAIL_COORD"):
                    # RAIL_COORD: reservation conflict cost is already reflected in cost (_dispatch_cost).
                    # Minimize the idle Empty OHT × task matching sum with Hungarian.
                    assignments = self._select_assignments_hungarian(
                        idle_vehicles, ready_tasks, claimed_sources, claimed_dests)
                    if assignments is not None:
                        self._apply_hungarian_assignments(
                            assignments, claimed_sources, claimed_dests, now)
                    else:
                        self._apply_greedy(idle_vehicles, ready_tasks,
                                           claimed_sources, claimed_dests, now)
                    if mode == "RAIL_COORD":
                        # B-3: while the reservation table is alive, position idle empty vehicles to conflict-free spots
                        if self.rail_idle_positioning:
                            self._assign_idle_positioning_railaware(claimed_sources, now)
                        self._rail_table = None   # end of round → release reservation table
                elif mode == "CONGESTION_AWARE_HEURISTIC":
                    # Select one vehicle at a time safely using the congestion-penalty-included cost.
                    # claimed source/dest updates after each dispatch, reducing crowding at the same spot.
                    self._apply_greedy(idle_vehicles, ready_tasks,
                                       claimed_sources, claimed_dests, now)
                elif mode == "PLAN_PRIORITY":
                    # planned-gantt slack priority sort + sequential commit (no Hungarian)
                    self._apply_plan_priority(idle_vehicles, ready_tasks,
                                              claimed_sources, claimed_dests, now)
                    # extension 1: steer remaining idle OHTs toward soon-to-pickup sources (direction only, no commit)
                    if self.plan_lookahead_positioning:
                        self._assign_idle_positioning(claimed_sources, now)
                elif mode in ("NVF", "STD", "EDD", "FIFO", "PRIORITY", "QS_STD", "BA_STD"):
                    # DISPATCH_RULE_FUNCTIONS.md rules: rule-key sorted greedy
                    self._apply_rule_greedy(idle_vehicles, ready_tasks,
                                            claimed_sources, claimed_dests, now, mode)
                else:  # GREEDY (or unknown → safe default)
                    self._apply_greedy(idle_vehicles, ready_tasks,
                                       claimed_sources, claimed_dests, now)

            # ── 2-pass: predispatch soon-to-finish in-process jobs using remaining idle OHTs ──
            if self.predispatch_lookahead > 0:
                remaining_idle = [v for v in self.vehicles
                                  if v.can_accept_dispatch()]
                if remaining_idle:
                    pre_tasks = self._collect_predispatch_tasks(
                        now, claimed_sources, claimed_dests)
                    if pre_tasks:
                        pre_assignments = self._select_assignments_hungarian(
                            remaining_idle, pre_tasks,
                            claimed_sources, claimed_dests)
                        if pre_assignments:
                            self._apply_hungarian_assignments(
                                pre_assignments, claimed_sources,
                                claimed_dests, now)

            # ── C1: position remaining idle OHTs toward machines that will soon produce a pickup ──
            #   not bound to a task (roam_target only) → keeps the reactive pool. Replaces random roam.
            if self.idle_positioning:
                self._assign_idle_positioning(claimed_sources, now)

            # ── Wait until next dispatch: event-based ──────────────────────────
            # Polling every dispatch_dt (0.5s) was mostly wasted. Now, when there's nothing to do,
            # passivate (sleep), and new pickup (machine out_buffer) / OHT idle-transition events
            # wake it via request_dispatch().
            # However, the two cases below need more than events, so retry (poll) briefly:
            #   (a) predispatch pre-staging is on (needs time-based pre-fetch)
            #   (b) idle OHT and pending pickup coexist but this round couldn't resolve matching
            #       due to source/dest lock / path block → wait for the block to clear.
            idle_now = [v for v in self.vehicles if v.can_accept_dispatch()]
            pending_pickup = any(
                getattr(ms, "out_buffer", None) for ms in self.machines.values())
            if self.predispatch_lookahead > 0 or (idle_now and pending_pickup):
                # blocked retry (source/dest lock / path block) or pre-staging → retry every dispatch_dt.
                # New task/idle are handled immediately via events, so even a large dispatch_dt
                # only reduces re-dispatch churn and round count with no responsiveness loss.
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