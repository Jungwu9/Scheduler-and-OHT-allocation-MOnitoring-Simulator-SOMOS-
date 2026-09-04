from __future__ import annotations

import random
import heapq
import time
import argparse
from dataclasses import replace
from collections import Counter, deque

import salabim as sim

from Simulation_OHT_Config import OHTConfig
from Simulation_OHT_Layout import OHTLayoutBuilder
from Simulation_Machine_Config import MachineConfig, JSSPConfig
from Simulation_Machine import MachineStation, Job, MachineBreakdown
from Decision_Maker_OHT import OHTDecisionMaker, RandomRoam, BFSPath, AStarPath
from Decision_Maker_Machine import MachineDecisionMaker, GanttHTMLExporter
from UI.live_trace_hooks import (
    append_oht_event_trace,
    append_transport_event,
    close_live_states,
    create_live_states,
)


class OHTVehicle(sim.Component):
    def setup(
            self,
            vid,
            start_node,
            nodes,
            edges,
            uv_to_eid,
            adj,
            machines,
            oht_config,
            idle_behavior=None,
            fleet=None,
            node_last_pass_time=None,
            hotspot_escape_next_by_node=None,
            hotspot_watch_nodes=None,
            node_hotspot_wait_start=None,
            hotspot_active_until=None,
            hotspot_node_to_cell_code=None,
            hotspot_node_block_until=None,
            hotspot_fork_pass_times=None,
            hotspot_fork_block_until=None,
            zcu_node_to_zone=None,
            zcu_occupied_by=None,
            route_node_pass_times=None,
            transport_event_log=None,
            transport_live_state=None,     # UI: streams transport events to CSV
            oht_event_trace_state=None,    # UI: streams vehicle moves to CSV

    ):
        self.vid = vid
        self.pos_node = start_node
        self.home_node = start_node

        self.nodes = nodes
        self.edges = edges
        self.uv_to_eid = uv_to_eid
        self.adj = adj
        self.machines = machines
        self.oht_config = oht_config
        self.fleet = fleet if fleet is not None else []
        self.node_last_pass_time = node_last_pass_time if node_last_pass_time is not None else {}

        # Hotspot-aware idle escape rule shared state
        # - hotspot_escape_next_by_node: fork station node -> next hop in the escape direction
        # - hotspot_watch_nodes: set of hotspot watch nodes around the 9/6 cells in layout_oht.csv
        # - node_hotspot_wait_start: first time blocking started at a hotspot node
        # - hotspot_active_until: end time for keeping the escape rule active after a hotspot with >=5s dwell
        self.hotspot_escape_next_by_node = hotspot_escape_next_by_node if hotspot_escape_next_by_node is not None else {}
        self.hotspot_watch_nodes = set(hotspot_watch_nodes or set())
        self.node_hotspot_wait_start = node_hotspot_wait_start if node_hotspot_wait_start is not None else {}
        self.hotspot_active_until = hotspot_active_until if hotspot_active_until is not None else {"until": -1e18}

        # Balanced hotspot/fork rule shared state
        # - hotspot_node_to_cell_code: hotspot node -> layout_oht.csv cell code
        # - hotspot_node_block_until: time to keep avoiding a hotspot node where a lot of waiting has built up
        # - hotspot_fork_pass_times: deque of recent pass times through the 13/18 fork nodes
        # - hotspot_fork_block_until: end time for stopping use of the 13/18 fork as an escape direction once it is congested
        self.hotspot_node_to_cell_code = hotspot_node_to_cell_code if hotspot_node_to_cell_code is not None else {}
        self.hotspot_node_block_until = hotspot_node_block_until if hotspot_node_block_until is not None else {}
        self.hotspot_fork_pass_times = hotspot_fork_pass_times if hotspot_fork_pass_times is not None else {}
        self.hotspot_fork_block_until = hotspot_fork_block_until if hotspot_fork_block_until is not None else {}

        # Station-based Preventive Zone Reservation (SPZR)
        # - Reserve only at the station before entering a conflict zone such as guide/curve/merge.
        # - Never wait inside a guide; release the reservation after arriving at the next station.
        self.zcu_node_to_zone = zcu_node_to_zone if zcu_node_to_zone is not None else {}
        self.zcu_occupied_by = zcu_occupied_by if zcu_occupied_by is not None else {}
        self.current_zcu_zones = set()

        # Count-aware routing shared state
        # Store recent pass times of each node to make paths that keep crowding a point expensive.
        self.route_node_pass_times = route_node_pass_times if route_node_pass_times is not None else {}

        # Shared list of actual transport event logs per OHT
        # Used for transport.csv and computing the transport bar of the actual machine gantt.
        self.transport_event_log = transport_event_log if transport_event_log is not None else []
        self.transport_live_state = transport_live_state
        self.oht_event_trace_state = oht_event_trace_state

        self.next_hop_intent = None
        self.roam_target = None   # C1: idle positioning target node (not a task, for drifting)

        # Idle movement behavior plugin (default: random roaming)
        self.idle_behavior = idle_behavior or RandomRoam()

        self.state_name = "Empty"
        self.assigned_task = None
        self.cargo_job = None
        self.is_idle = False
        self._had_task = False     # whether a task was held in the previous iter (to wake the dispatcher on completion)
        self._dispatcher = None    # back-ref for event-based dispatch (injected in build_logic)

        self.draw_x = self.nodes[start_node].x
        self.draw_y = self.nodes[start_node].y

        # Movement interpolation state
        self.move_active = False
        self.move_t0 = 0.0
        self.move_t1 = 0.0
        self.move_x0 = self.draw_x
        self.move_y0 = self.draw_y
        self.move_x1 = self.draw_x
        self.move_y1 = self.draw_y

        self.dir_x = 1.0
        self.dir_y = 0.0

        self.waiting_time_acc = 0.0
        self._transit_acc = 0.0  # accumulated pure travel (free-flow) time
        self._block_acc = 0.0  # accumulated blocking time stopped by node occupancy
        self._idle_start = 0.0  # last time it moved (idle timer)

        self.merge_wait_key = None
        self.merge_wait_since = None
        self.last_yield_time = -1e18
        self._append_oht_event_trace({
            "sim_time": 0.0, "end_time": 0.0, "oht_id": self.vid,
            "event": "INIT", "state": "Empty", "new_state": "Empty",
            "phase": "init", "from_node": start_node, "to_node": start_node,
            "x": round(float(self.draw_x), 3), "y": round(float(self.draw_y), 3),
        })

    # -----------------------------------------------------
    # dispatch
    # -----------------------------------------------------
    def can_accept_dispatch(self):
        # Dispatchable if no task/cargo (allowed even while roaming Empty)
        return self.assigned_task is None and self.cargo_job is None

    def assign(self, task, job=None):
        self.assigned_task = task
        self.cargo_job = job
        was_idle = self.is_idle
        self.is_idle = False
        # if is_idle=True it is passivate/hold -> wake immediately with activate
        if was_idle:
            self.activate()

    # -----------------------------------------------------
    # occupancy
    # -----------------------------------------------------
    def reserve_start_node(self):
        pass  # node occupancy removed - deadlock prevention

    def _is_machine_node(self, node_name):
        return node_name in self.machines

    def _is_cross_node(self, node_name):
        return len(self.adj.get(node_name, [])) >= 3

    def _other_oht_on_node(self, node_name):
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, 'pos_node', None) == node_name:
                return True
        return False

    def _other_oht_intending_next_node(self, node_name):
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, 'next_hop_intent', None) == node_name:
                return True
        return False

    def _other_oht_targeting_node(self, node_name):
        for v in self.fleet:
            if v is self:
                continue
            task = getattr(v, 'assigned_task', None)
            if task is None:
                continue
            if getattr(task, 'source_name', None) == node_name:
                return True
            if getattr(task, 'dest_name', None) == node_name:
                return True
        return False

    # -----------------------------------------------------
    # hotspot-aware idle escape rule (9/6 hotspot, 5 sec dwell trigger)
    # -----------------------------------------------------
    def _hotspot_escape_enabled(self):
        return bool(getattr(self.oht_config, "enable_hotspot_escape_rule", True))

    def _hotspot_wait_sec(self):
        return float(getattr(self.oht_config, "hotspot_escape_wait_sec", 5.0))

    def _hotspot_hold_sec(self):
        return float(getattr(self.oht_config, "hotspot_escape_hold_sec", 10.0))

    def _is_hotspot_watch_node(self, node_name):
        return node_name in self.hotspot_watch_nodes

    def _is_next_node_safe_for_hotspot_escape(self, node_name):
        """
        Safety check for an escape candidate node.
        False if another OHT is already there, or another OHT intends to enter that node.
        """
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, "pos_node", None) == node_name:
                return False
            if getattr(v, "next_hop_intent", None) == node_name:
                return False
        return True

    def _is_next_node_blocked_now(self, node_name):
        """
        For re-checking right before actual departure / right after waiting on the edge resource.
        Do not depart if another OHT is already on the next node.
        """
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, "pos_node", None) == node_name:
                return True
        return False

    def _instant_node_pressure_for_escape(self, node_name):
        """
        Current occupancy/entry-intent pressure of the node.
        Used as an auxiliary score so escape candidates do not overlap.
        """
        cnt = 0
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, "pos_node", None) == node_name:
                cnt += 1
            if getattr(v, "next_hop_intent", None) == node_name:
                cnt += 1
        return cnt

    def _hotspot_fork_balance_enabled(self):
        return bool(getattr(self.oht_config, "enable_hotspot_fork_balance", False))

    def _hotspot_route_penalty_enabled(self):
        return bool(getattr(self.oht_config, "enable_hotspot_route_penalty", True))

    def _hotspot_cell_code_of_node(self, node_name):
        return self.hotspot_node_to_cell_code.get(node_name)

    def _hotspot_now(self):
        return self.env.now() if hasattr(self, "env") and self.env is not None else 0.0

    def _is_hotspot_node_blocked_active(self, node_name):
        now = self._hotspot_now()
        return float(self.hotspot_node_block_until.get(node_name, -1e18)) > now

    def _is_hotspot_fork_blocked_active(self, fork_node_name):
        now = self._hotspot_now()
        return float(self.hotspot_fork_block_until.get(fork_node_name, -1e18)) > now

    def _hotspot_fork_window_sec(self):
        return float(getattr(self.oht_config, "hotspot_fork_count_window_sec", 20.0))

    def _prune_hotspot_fork_pass_times(self, fork_node_name):
        dq = self.hotspot_fork_pass_times.setdefault(fork_node_name, deque())
        now = self._hotspot_now()
        window = self._hotspot_fork_window_sec()
        while dq and (now - float(dq[0])) > window:
            dq.popleft()
        return dq

    def _recent_hotspot_fork_pass_count(self, fork_node_name):
        return len(self._prune_hotspot_fork_pass_times(fork_node_name))

    def _record_hotspot_fork_pass(self, fork_node_name):
        """
        Record, over the recent window, whether the 13/18 fork keeps being used as an escape exit.
        If the threshold is exceeded, briefly block that fork so the next idle escape/A* picks another path.
        """
        if not self._hotspot_fork_balance_enabled():
            return
        if fork_node_name not in self.hotspot_escape_next_by_node:
            return

        now = self._hotspot_now()
        dq = self._prune_hotspot_fork_pass_times(fork_node_name)
        dq.append(now)

        threshold = int(getattr(self.oht_config, "hotspot_fork_count_threshold", 3))
        if threshold > 0 and len(dq) >= threshold:
            hold = float(getattr(self.oht_config, "hotspot_fork_block_hold_sec", 12.0))
            self.hotspot_fork_block_until[fork_node_name] = max(
                float(self.hotspot_fork_block_until.get(fork_node_name, -1e18)),
                now + hold,
            )
            if bool(getattr(self.oht_config, "debug_hotspot_escape_rule", False)):
                cell = self._hotspot_cell_code_of_node(fork_node_name)
                print(
                    f"[hotspot_fork BLOCK] t={now:.1f}, "
                    f"fork_node={fork_node_name}, cell={cell}, "
                    f"count={len(dq)}, until={self.hotspot_fork_block_until[fork_node_name]:.1f}"
                )

    def _record_hotspot_waiting(self, blocked_node_name):
        """
        If blocking persists at a hotspot watch node, register that node as an active hotspot.

        Change notes:
        - Before: only 9/6 was watched, and 13/18 was used only as an escape exit
        - Now: 9/6/13/18 can all be watched
        - If waiting builds up at 13/18, that fork is also briefly blocked to spread traffic to other paths
        """
        if not self._hotspot_escape_enabled():
            return
        if blocked_node_name is None:
            return
        if blocked_node_name not in self.hotspot_watch_nodes:
            return

        now = self._hotspot_now()
        start = self.node_hotspot_wait_start.get(blocked_node_name)

        if start is None:
            self.node_hotspot_wait_start[blocked_node_name] = now
            return

        wait_time = now - float(start)
        if wait_time >= self._hotspot_wait_sec():
            hold = self._hotspot_hold_sec()
            self.hotspot_node_block_until[blocked_node_name] = max(
                float(self.hotspot_node_block_until.get(blocked_node_name, -1e18)),
                now + hold,
            )
            # Keep compatibility with the existing idle escape flag
            self.hotspot_active_until["until"] = max(
                float(self.hotspot_active_until.get("until", -1e18)),
                now + hold,
            )

            # If the blocked node belongs to the 13/18 fork family, briefly block the fork itself too
            if self._hotspot_fork_balance_enabled():
                cell = self._hotspot_cell_code_of_node(blocked_node_name)
                fork_codes = set(str(x) for x in getattr(self.oht_config, "hotspot_escape_fork_codes", {"13", "18"}))
                if cell in fork_codes or blocked_node_name in self.hotspot_escape_next_by_node:
                    fork_hold = float(getattr(self.oht_config, "hotspot_fork_block_hold_sec", 12.0))
                    # Even if it is not the exact fork source node, block the fork sources of the same cell together.
                    fork_nodes = [blocked_node_name]
                    fork_nodes += [n for n, c in self.hotspot_node_to_cell_code.items() if
                                   c == cell and n in self.hotspot_escape_next_by_node]
                    for fn in fork_nodes:
                        self.hotspot_fork_block_until[fn] = max(
                            float(self.hotspot_fork_block_until.get(fn, -1e18)),
                            now + fork_hold,
                        )

            if bool(getattr(self.oht_config, "debug_hotspot_escape_rule", False)):
                cell = self._hotspot_cell_code_of_node(blocked_node_name)
                print(
                    f"[hotspot ON] t={now:.1f}, node={blocked_node_name}, cell={cell}, "
                    f"wait={wait_time:.1f}s, block_until={self.hotspot_node_block_until[blocked_node_name]:.1f}"
                )

    def _clear_hotspot_waiting_if_needed(self, node_name):
        """
        Reset the node's consecutive-wait start time once it is no longer blocking and actual movement proceeds.
        """
        if node_name in self.node_hotspot_wait_start:
            self.node_hotspot_wait_start.pop(node_name, None)

    def _is_hotspot_escape_active(self):
        """
        Conditions for idle escape being active.
        - True if any of 9/6/13/18 has an active block node
        - Or, as before, True if an OHT is actually inside a watch node
        """
        if not self._hotspot_escape_enabled():
            return False

        now = self._hotspot_now()
        if float(self.hotspot_active_until.get("until", -1e18)) > now:
            return True

        for until in self.hotspot_node_block_until.values():
            if float(until) > now:
                return True

        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, "pos_node", None) in self.hotspot_watch_nodes:
                return True

        return False

    def _escape_candidate_score(self, node_name):
        """
        Score for picking a safe, less-congested escape candidate.
        Penalize heavily if 13/18 has already been used a lot or is an active hotspot.
        """
        now = self._hotspot_now()
        score = 0.0

        # Current occupancy/entry-intent pressure
        score += self._instant_node_pressure_for_escape(node_name) * float(
            getattr(self.oht_config, "hotspot_candidate_live_pressure_weight", 50.0)
        )

        # Avoid active hotspot nodes
        if float(self.hotspot_node_block_until.get(node_name, -1e18)) > now:
            score += float(getattr(self.oht_config, "hotspot_route_penalty", 80.0))

        # Avoid more strongly if the fork node is in a congestion-block state
        if float(self.hotspot_fork_block_until.get(node_name, -1e18)) > now:
            score += float(getattr(self.oht_config, "hotspot_fork_penalty", 120.0))

        # Slightly reflect the recent pass count too
        if hasattr(self, "_recent_route_node_count"):
            score += self._recent_route_node_count(node_name) * float(
                getattr(self.oht_config, "hotspot_candidate_recent_count_weight", 5.0)
            )

        # Guide directions are likely detours, so prefer them slightly, but weaker than the penalty.
        kind_penalty = 0.0 if self.nodes.get(node_name) is not None and self.nodes[node_name].kind == 'guide' else 2.0
        return (score + kind_penalty, str(node_name))

    def _is_candidate_blocked_for_hotspot_escape(self, node_name):
        """
        Blocked-check for an idle escape candidate.
        An active hotspot or a congested fork is also treated as blocked.
        """
        if node_name is None:
            return True
        if node_name not in self.adj.get(self.pos_node, []):
            return True

        now = self._hotspot_now()
        if self._hotspot_fork_balance_enabled():
            if float(self.hotspot_node_block_until.get(node_name, -1e18)) > now:
                return True
            if float(self.hotspot_fork_block_until.get(node_name, -1e18)) > now:
                return True

        front_nodes = self._prewait_buffer_nodes(node_name)
        for nd in front_nodes:
            for v in self.fleet:
                if v is self:
                    continue
                if getattr(v, "pos_node", None) == nd:
                    return True
                if getattr(v, "next_hop_intent", None) == nd:
                    return True

        if self._zcu_blocked(front_nodes):
            return True

        return False

    def _choose_idle_next_node_by_hotspot_escape(self, proposed_next):
        """
        Balanced hotspot idle escape.

        Before: when 9/6 was blocked, send to the fixed escape_next of the 13/18 fork.
        Now: 13/18 is also watched, so if 13/18 is congested, do not use escape_next and
             instead spread to the safest, lowest-pressure neighbor of the current fork.
        """
        if not self._hotspot_escape_enabled():
            return proposed_next
        if not self._is_hotspot_escape_active():
            return proposed_next

        # If the original path is not blocked, do not bother detouring.
        if not self._is_candidate_blocked_for_hotspot_escape(proposed_next):
            return proposed_next

        candidates = []

        # Priority 1: the existing escape_next
        escape_next = self.hotspot_escape_next_by_node.get(self.pos_node)
        if escape_next is not None and escape_next in self.adj.get(self.pos_node, []):
            candidates.append(escape_next)

        # Priority 2: also add other reachable neighbors of the same fork as candidates to balance
        if self._hotspot_fork_balance_enabled():
            for nb in self.adj.get(self.pos_node, []):
                if nb not in candidates:
                    candidates.append(nb)

        safe = [nd for nd in candidates if not self._is_candidate_blocked_for_hotspot_escape(nd)]
        if not safe:
            return proposed_next

        safe.sort(key=self._escape_candidate_score)
        chosen = safe[0]

        if bool(getattr(self.oht_config, "debug_hotspot_escape_rule", False)):
            print(
                f"[hotspot_escape SELECT] t={self._hotspot_now():.1f}, "
                f"OHT={self.vid}, pos={self.pos_node}, proposed={proposed_next}, chosen={chosen}, "
                f"scores={[(n, self._escape_candidate_score(n)) for n in safe[:4]]}"
            )

        return chosen

    def _trace_forward_until_station(self, start_node_name):
        """
        Trace straight from a guide node until the next station appears.
        Used to find the merge station at the end of a curve from the station right before curve entry.
        """
        chain = []
        visited = set()
        cur = start_node_name

        while cur and cur not in visited and cur in self.nodes:
            visited.add(cur)
            chain.append(cur)

            node = self.nodes[cur]
            if node.kind == 'station':
                return chain, cur

            nxts = list(self.adj.get(cur, []))
            if not nxts:
                break
            cur = nxts[0]

        return chain, None

    def _curve_merge_station(self, next_node_name):
        """
        If the current next hop enters a curve (guide),
        return the first station node where that curve ends and merges.
        """
        node = self.nodes.get(next_node_name)
        if node is None or node.kind != 'guide':
            return None, []

        chain, merge_station = self._trace_forward_until_station(next_node_name)
        if merge_station is None:
            return None, chain
        if self.nodes.get(merge_station) is None or self.nodes[merge_station].kind != 'station':
            return None, chain
        return merge_station, chain

    def _merge_key_for_next(self, next_node_name):
        """
        If the current move involves a curve merge, return the merge key.
        """
        merge_station, _ = self._curve_merge_station(next_node_name)
        if merge_station is None:
            return None
        return f"merge::{merge_station}"

    def _update_merge_wait_flag(self, merge_key, blocked):
        if merge_key is None:
            return

        if blocked:
            if self.merge_wait_key != merge_key:
                self.merge_wait_key = merge_key
                self.merge_wait_since = self.env.now()
            elif self.merge_wait_since is None:
                self.merge_wait_since = self.env.now()
        else:
            if self.merge_wait_key == merge_key:
                self.merge_wait_key = None
                self.merge_wait_since = None

    def _curve_waiting_vehicle_for(self, merge_key):
        """
        Among the OHTs currently 'waiting to enter the curve' at the same merge,
        return the one that has waited the longest.
        """
        if merge_key is None:
            return None

        candidates = []
        for v in self.fleet:
            if v is self:
                continue
            if getattr(v, 'merge_wait_key', None) != merge_key:
                continue
            if getattr(v, 'merge_wait_since', None) is None:
                continue

            intent = getattr(v, 'next_hop_intent', None)
            if intent is None:
                continue

            v_merge_station, _ = v._curve_merge_station(intent)
            if v_merge_station is None:
                continue

            if merge_key == f"merge::{v_merge_station}":
                candidates.append(v)

        if not candidates:
            return None

        return min(candidates, key=lambda x: getattr(x, 'merge_wait_since', 1e18))

    def _upcoming_merge_station_on_straight(self, start_node, max_hops=6):
        """
        Follow forward from a straight station and find the first merge station where a guide branch joins.
        Function to pre-stop a straight vehicle at the current point.
        """
        cur = start_node
        visited = set()

        for _ in range(max_hops):
            if cur in visited or cur not in self.nodes:
                break
            visited.add(cur)

            node = self.nodes[cur]
            if node.kind == 'station':
                preds = self._incoming_predecessors(cur)
                if any(self.nodes.get(p) is not None and self.nodes[p].kind == 'guide' for p in preds):
                    return cur

            nxts = list(self.adj.get(cur, []))
            if not nxts:
                break
            cur = nxts[0]

        return None

    def _curve_waiters_for_merge(self, merge_station):
        """
        Among curve vehicles heading into the same merge,
        return only those waiting to enter at the current station (point).
        """
        out = []
        if merge_station is None:
            return out

        key = f"merge::{merge_station}"
        for v in self.fleet:
            intent = getattr(v, 'next_hop_intent', None)
            if not intent:
                continue
            m, chain = v._curve_merge_station(intent)
            if m != merge_station:
                continue
            if getattr(v, 'pos_node', None) in v.nodes and v.nodes[getattr(v, 'pos_node', None)].kind == 'station':
                if getattr(v, 'merge_wait_key', None) == key:
                    out.append(v)
        return out

    def _curve_committed_for_merge(self, merge_station):
        """
        Vehicles that have already entered the curve (guide chain).
        These vehicles must proceed to the merge without being stopped midway.
        Straight vehicles yield to these.
        """
        out = []
        if merge_station is None:
            return out

        for v in self.fleet:
            pos = getattr(v, 'pos_node', None)
            if pos not in v.nodes:
                continue
            node = v.nodes[pos]
            if node.kind != 'guide':
                continue

            chain, m = v._trace_forward_until_station(pos)
            if m == merge_station:
                out.append(v)

        return out

    def _incoming_predecessors(self, node_name):
        preds = []
        for u, nbrs in self.adj.items():
            if node_name in nbrs:
                preds.append(u)
        return preds

    def _is_in_curve(self):
        """
        Determine whether currently inside a curve (on a guide node) or just exiting the curve.
        - If own pos_node is a guide, it is inside the curve
        - Or, if next_hop_intent is a guide, it is entering the curve
        """
        nd = self.nodes.get(self.pos_node)
        if nd is not None and getattr(nd, 'kind', None) == 'guide':
            return True
        nxt = getattr(self, 'next_hop_intent', None)
        if nxt:
            nd2 = self.nodes.get(nxt)
            if nd2 is not None and getattr(nd2, 'kind', None) == 'guide':
                return True
        return False

    def _has_higher_priority_than(self, other):
        """
        Priority:
        1) loaded OHT first
        2) OHT entering/passing a curve first (curve traffic priority — stopping in a curve
           loses re-acceleration and backs up the following vehicles, which is inefficient)
        3) if equal, the smaller vid first
        """
        self_loaded = self.cargo_job is not None
        other_loaded = getattr(other, 'cargo_job', None) is not None
        if self_loaded != other_loaded:
            return self_loaded and not other_loaded

        self_curve = self._is_in_curve()
        other_curve = (other._is_in_curve()
                       if hasattr(other, '_is_in_curve') else False)
        if self_curve != other_curve:
            return self_curve  # the curve side is higher

        return self.vid < other.vid

    def _waiting_hold(self, cur):
        # While waiting, turn off interpolated motion and pin exactly to the current node coordinates
        self.move_active = False
        self.draw_x = cur.x
        self.draw_y = cur.y
        self._set_traffic_wait_or_break()
        self.waiting_time_acc += self.oht_config.block_retry_dt

    def visual_x(self):
        if self.move_active and self.move_t1 > self.move_t0:
            now = self.env.now()
            if now <= self.move_t0:
                return self.move_x0
            if now >= self.move_t1:
                return self.move_x1
            a = (now - self.move_t0) / (self.move_t1 - self.move_t0)
            return self.move_x0 + a * (self.move_x1 - self.move_x0)
        return self.draw_x

    def visual_y(self):
        if self.move_active and self.move_t1 > self.move_t0:
            now = self.env.now()
            if now <= self.move_t0:
                return self.move_y0
            if now >= self.move_t1:
                return self.move_y1
            a = (now - self.move_t0) / (self.move_t1 - self.move_t0)
            return self.move_y0 + a * (self.move_y1 - self.move_y0)
        return self.draw_y

    def _forward_until_next_station(self, start_node_name):
        """
        Follow from start_node_name and return the list of nodes up to the 'next station'.
        The returned list also includes start_node_name.
        """
        chain = []
        visited = set()
        cur = start_node_name

        while cur and cur not in visited and cur in self.nodes:
            visited.add(cur)
            chain.append(cur)

            node = self.nodes[cur]
            # End at the station met next, not the starting node
            if len(chain) > 1 and node.kind == 'station':
                break

            nxts = list(self.adj.get(cur, []))
            if not nxts:
                break
            cur = nxts[0]

        return chain

    def _vehicle_target_node(self, v):
        """Current target node of vehicle v (loaded→dest, assigned-empty→source, idle→None)."""
        cargo = getattr(v, 'cargo_job', None)
        task = getattr(v, 'assigned_task', None)
        if cargo is not None:
            if getattr(cargo, 'next_candidate_nodes', None):
                return cargo.next_candidate_nodes[0]
            return task.dest_name if task is not None else None
        return task.source_name if task is not None else None

    def _forced_next_from(self, v, from_node):
        """Next hop (forced route) when vehicle v goes from from_node to the target. None if absent."""
        tgt = self._vehicle_target_node(v)
        if not tgt or tgt == from_node or tgt not in self.nodes or from_node not in self.nodes:
            return None
        p = self.shortest_path(from_node, tgt)
        return p[1] if len(p) > 1 else None

    def _prewait_buffer_nodes(self, next_node_name):
        """
        Forward segment to check before departing from the current station.
        - If next_node is a guide: check everything from next_node to the next station
        - If next_node is a station: check only next_node
        """
        nxt = self.nodes.get(next_node_name)
        if nxt is None:
            return []

        if nxt.kind == 'guide':
            return self._forward_until_next_station(next_node_name)

        return [next_node_name]

    def _should_wait_before_curve(self, next_node_name):
        """
        Waiting rule before entering a curve.
        - If a straight rival branch has come right up to the merge, the curve OHT yields
        - Occupancy of the merge/curve section: only one vehicle waits by priority
        """
        merge_station, curve_chain = self._curve_merge_station(next_node_name)
        if merge_station is None:
            return False

        curve_chain_set = set(curve_chain)

        rival_preds = set(self._incoming_predecessors(merge_station))
        rival_preds.discard(self.pos_node)
        rival_preds -= curve_chain_set

        for v in self.fleet:
            if v is self:
                continue

            other_pos = getattr(v, 'pos_node', None)
            other_intent = getattr(v, 'next_hop_intent', None)

            # 1) If there is a competitor on the straight rival side, the curve yields
            if other_pos in rival_preds or other_intent in rival_preds:
                return True

            # 2) If a competitor is already inside merge/curve, compare priority
            if (
                    other_pos == merge_station
                    or other_pos in curve_chain_set
                    or other_intent == merge_station
                    or other_intent in curve_chain_set
            ):
                if not self._has_higher_priority_than(v):
                    return True

        return False

    # -----------------------------------------------------
    # count-aware routing helpers
    # -----------------------------------------------------
    def _count_aware_routing_enabled(self):
        algo = str(getattr(self.oht_config, 'oht_path_algorithm', 'ASTAR')).upper().replace('-', '_').replace(' ', '_')
        return algo in {
            'COUNT_AWARE_ASTAR',
            'TRAFFIC_COUNT_ASTAR',
            'CONGESTION_AWARE_ASTAR',
            'CONGESTION_AWARE_ROUTING',
        }

    def _route_count_window_sec(self):
        return float(getattr(self.oht_config, 'oht_route_count_window_sec', 30.0))

    def _prune_route_node_pass_times(self, node_name):
        dq = self.route_node_pass_times.setdefault(node_name, deque())
        window = self._route_count_window_sec()
        now = self.env.now() if hasattr(self, 'env') else 0.0
        while dq and (now - dq[0]) > window:
            dq.popleft()
        return dq

    def _recent_route_node_count(self, node_name):
        return len(self._prune_route_node_pass_times(node_name))

    def _record_route_node_pass(self, node_name):
        if not bool(getattr(self.oht_config, 'enable_oht_route_count_record', True)):
            return
        dq = self._prune_route_node_pass_times(node_name)
        dq.append(self.env.now() if hasattr(self, 'env') else 0.0)

    def _count_blocked_node_for_routing(self, node_name, goal_name):
        if not self._count_aware_routing_enabled():
            return False
        if not bool(getattr(self.oht_config, 'enable_oht_route_count_hard_block', True)):
            return False
        # Blocking the destination itself makes pickup/drop-off impossible, so the goal is exempted.
        if node_name == goal_name:
            return False
        hard_threshold = int(getattr(self.oht_config, 'oht_route_count_hard_threshold', 8))
        if hard_threshold <= 0:
            return False
        return self._recent_route_node_count(node_name) >= hard_threshold

    # -----------------------------------------------------
    # routing
    # -----------------------------------------------------
    def _hotspot_blocked_node_for_routing(self, node_name, goal_name):
        """A* hard-block decision: briefly exclude an active hotspot/fork from candidates unless it is the destination."""
        if not self._hotspot_fork_balance_enabled():
            return False
        if not bool(getattr(self.oht_config, "enable_hotspot_route_hard_block", True)):
            return False
        if node_name == goal_name:
            return False
        now = self._hotspot_now()
        if float(self.hotspot_node_block_until.get(node_name, -1e18)) > now:
            return True
        if float(self.hotspot_fork_block_until.get(node_name, -1e18)) > now:
            return True
        return False

    def _routing_edge_cost(self, u, v):
        """
        Cost based on edge.travel_time.

        If oht_path_algorithm is a COUNT_AWARE_ASTAR variant,
        add, as extra penalty, the count of OHTs that passed node v during the recent window.
        That is, if too many OHTs pass a point, A* sees that point as an expensive path.
        """
        eid = self.uv_to_eid.get((u, v))
        if eid is not None and eid in self.edges:
            base = float(getattr(self.edges[eid], 'travel_time', 1.0))
        elif u in self.nodes and v in self.nodes:
            base = ((self.nodes[u].x - self.nodes[v].x) ** 2
                    + (self.nodes[u].y - self.nodes[v].y) ** 2) ** 0.5
        else:
            base = 1.0

        if not self._count_aware_routing_enabled():
            return base

        penalty = 0.0

        # 1) Penalty based on recent pass count
        threshold = int(getattr(self.oht_config, 'oht_route_count_soft_threshold', 4))
        count = self._recent_route_node_count(v)
        if count > threshold:
            penalty += (count - threshold) * float(getattr(self.oht_config, 'oht_route_count_penalty', 15.0))

        # 2) Also reflecting current occupancy/entry intent avoids instantaneous bottlenecks before the count builds up
        if bool(getattr(self.oht_config, 'enable_oht_route_live_penalty', True)):
            for other in self.fleet:
                if other is self:
                    continue
                if getattr(other, 'pos_node', None) == v:
                    penalty += float(getattr(self.oht_config, 'oht_route_live_node_penalty', 25.0))
                if getattr(other, 'next_hop_intent', None) == v:
                    penalty += float(getattr(self.oht_config, 'oht_route_live_intent_penalty', 20.0))

        # 3) An Empty OHT yields more strongly to a point a Loaded OHT is heading to
        if self.cargo_job is None and bool(getattr(self.oht_config, 'enable_oht_loaded_priority_routing', True)):
            for other in self.fleet:
                if other is self:
                    continue
                if getattr(other, 'cargo_job', None) is not None and getattr(other, 'next_hop_intent', None) == v:
                    penalty += float(getattr(self.oht_config, 'oht_route_loaded_priority_penalty', 25.0))

        # 4) Balanced hotspot/fork penalty: A* avoids not only 9/6 but also 13/18 when blocked
        if self._hotspot_fork_balance_enabled() and self._hotspot_route_penalty_enabled():
            now = self._hotspot_now()
            if float(self.hotspot_node_block_until.get(v, -1e18)) > now:
                penalty += float(getattr(self.oht_config, 'hotspot_route_penalty', 80.0))
            if float(self.hotspot_fork_block_until.get(v, -1e18)) > now:
                penalty += float(getattr(self.oht_config, 'hotspot_fork_penalty', 120.0))

        return base + penalty

    def _routing_goal_dist(self, node_name, goal_name):
        if node_name not in self.nodes or goal_name not in self.nodes:
            return 0.0
        return ((self.nodes[node_name].x - self.nodes[goal_name].x) ** 2
                + (self.nodes[node_name].y - self.nodes[goal_name].y) ** 2) ** 0.5

    def _shortest_path_bfs(self, start, goal):
        """Directed BFS: minimize hop count + nearest-to-goal tie-break."""
        if start == goal:
            return [start]
        if start not in self.adj or goal not in self.nodes:
            return [start]

        q = deque([start])
        parent = {start: None}

        while q:
            u = q.popleft()
            nbs = list(self.adj.get(u, []))
            nbs.sort(key=lambda v: (self._routing_goal_dist(v, goal), self._routing_edge_cost(u, v), str(v)))
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

    def _shortest_path_astar(self, start, goal, ignore_count_block=False):
        """Directed A*: edge.travel_time/coordinate-distance cost + goal heuristic."""
        if start == goal:
            return [start]
        if start not in self.nodes or goal not in self.nodes:
            return [start]

        heap = [(self._routing_goal_dist(start, goal), 0.0, start)]
        parent = {start: None}
        dist = {start: 0.0}
        closed = set()

        while heap:
            f, g, u = heapq.heappop(heap)
            if u in closed:
                continue
            closed.add(u)
            if u == goal:
                path = []
                cur = goal
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                return path

            nbs = list(self.adj.get(u, []))
            nbs.sort(key=lambda v: (self._routing_goal_dist(v, goal), self._routing_edge_cost(u, v), str(v)))
            for v in nbs:
                if v in closed:
                    continue
                if (not ignore_count_block) and self._count_blocked_node_for_routing(v, goal):
                    continue
                if (not ignore_count_block) and self._hotspot_blocked_node_for_routing(v, goal):
                    continue
                ng = g + self._routing_edge_cost(u, v)
                if ng < dist.get(v, 1e18):
                    dist[v] = ng
                    parent[v] = u
                    heapq.heappush(heap, (ng + self._routing_goal_dist(v, goal), ng, v))

        # If the hard block cuts the path entirely, safely lift only the hard block and search once more.
        if self._count_aware_routing_enabled() and (not ignore_count_block):
            return self._shortest_path_astar(start, goal, ignore_count_block=True)
        return [start]

    def shortest_path(self, start, goal):
        """OHTConfig.oht_path_algorithm = BFS / ASTAR / COUNT_AWARE_ASTAR switch."""
        algo = str(getattr(self.oht_config, 'oht_path_algorithm', 'ASTAR')).upper().replace('-', '_').replace(' ', '_')
        if algo in {'BFS', 'BFS_PATH'}:
            return self._shortest_path_bfs(start, goal)
        return self._shortest_path_astar(start, goal)

    # -----------------------------------------------------
    # waiting / breaking
    # -----------------------------------------------------
    def _set_traffic_wait_or_break(self):
        """
        Can go to Breaked only when another OHT blocks the way.
        """
        if self.waiting_time_acc >= self.oht_config.breaking_threshold:
            self.state_name = "Breaked"
        else:
            self.state_name = "Waiting"

    def _set_service_wait(self):
        """
        machine/input/output/service waits are Waiting, not Breaked.
        """
        self.state_name = "Waiting"

    def _reset_wait(self, carrying: bool):
        self.waiting_time_acc = 0.0
        if carrying:
            self.state_name = "Loaded"
        else:
            # Not stopped without product, but in a state able to move again
            self.state_name = "Empty"

    # -----------------------------------------------------
    # Station-based Preventive Zone Reservation (SPZR / ZCU)
    # -----------------------------------------------------
    def _zcu_zones_for_nodes(self, node_names):
        """
        Return the set of ZCU zones containing the node list.
        A zcu_node_to_zone value may be a single string or a set/list/tuple.
        """
        zones = set()
        for nd in node_names:
            z = self.zcu_node_to_zone.get(nd)
            if z is None:
                continue
            if isinstance(z, (set, list, tuple)):
                zones.update(x for x in z if x is not None)
            else:
                zones.add(z)
        return zones

    def _zcu_blocked(self, node_names):
        """
        Before departing a station, check whether the conflict zone is occupied.
        If another OHT occupies it, wait only at the current station.
        """
        if not bool(getattr(self.oht_config, "enable_station_zcu", True)):
            return False

        for zone in self._zcu_zones_for_nodes(node_names):
            owner = self.zcu_occupied_by.get(zone)
            if owner is not None and owner != self.vid:
                return True
        return False

    def _reserve_zcu(self, node_names):
        """
        Reserve right before entering the conflict zone.
        Prevent simultaneous entry into the same zone to avoid OHT overtaking/overlap.
        """
        if not bool(getattr(self.oht_config, "enable_station_zcu", True)):
            return

        for zone in self._zcu_zones_for_nodes(node_names):
            owner = self.zcu_occupied_by.get(zone)
            if owner is None or owner == self.vid:
                self.zcu_occupied_by[zone] = self.vid
                self.current_zcu_zones.add(zone)

    def _release_zcu_if_station_arrival(self):
        """
        Release the ZCU only upon arriving at a station after passing the guide/curve/merge section.
        This prevents waiting inside a guide or a mid-release letting a following vehicle overtake.
        """
        node = self.nodes.get(self.pos_node)
        if node is None or node.kind != 'station':
            return

        for zone in list(self.current_zcu_zones):
            if self.zcu_occupied_by.get(zone) == self.vid:
                self.zcu_occupied_by.pop(zone, None)
            self.current_zcu_zones.discard(zone)

    # -----------------------------------------------------
    # rail move
    # -----------------------------------------------------
    def move_one_hop(self, next_node_name):
        cur = self.nodes[self.pos_node]

        def _has_priority_over(other):
            self_loaded = self.cargo_job is not None
            other_loaded = getattr(other, 'cargo_job', None) is not None
            if self_loaded != other_loaded:
                return self_loaded and not other_loaded
            # A vehicle entering/passing a curve has priority
            self_curve = self._is_in_curve()
            other_curve = (other._is_in_curve()
                           if hasattr(other, '_is_in_curve') else False)
            if self_curve != other_curve:
                return self_curve
            return self.vid < other.vid

        def _wait_here(extra_hold=0.0):
            """
            Waiting must happen only at the current station point.
            Therefore this function is controlled in the move_one_hop flow to be
            called only when cur.kind == 'station'.
            """
            self._record_hotspot_waiting(next_node_name)

            self.draw_x = cur.x
            self.draw_y = cur.y
            self.move_active = False

            self._set_traffic_wait_or_break()
            self.waiting_time_acc += self.oht_config.block_retry_dt
            self._block_acc += self.oht_config.block_retry_dt + max(0.0, extra_hold)

            return self.hold(
                self.oht_config.block_retry_dt + max(0.0, extra_hold)
            )

        def _front_chain_blocked(front_nodes):
            """
            Pre-check the forward segment before departing the current station.

            Key points:
            - If next_node is a guide, check the whole curve guide chain + up to the next station
            - If blocked, do not enter the curve and wait at the current station
            - Thanks to this function, no Breaked occurs inside a guide.
            """
            for nd in front_nodes:
                for v in self.fleet:
                    if v is self:
                        continue

                    other_pos = getattr(v, 'pos_node', None)
                    other_intent = getattr(v, 'next_hop_intent', None)

                    # If already inside the forward chain, entry is forbidden
                    if other_pos == nd:
                        return True

                    # If another OHT intends to enter the same forward node, compare priority
                    if other_intent == nd and (not _has_priority_over(v)):
                        return True

            return False

        def _do_move():
            self.next_hop_intent = next_node_name
            try:
                eid = self.uv_to_eid[(self.pos_node, next_node_name)]
                edge = self.edges[eid]
                nxt = self.nodes[next_node_name]

                yield self.request(edge.res)

                # Important:
                # Do the final re-check only when departing from a station.
                # Inside a guide, calling _wait_here() here would create
                # Waiting/Breaked mid-curve, so never call it.
                if cur.kind == 'station':
                    if self._is_next_node_blocked_now(next_node_name):
                        self.release(edge.res)
                        yield _wait_here()
                        return

                self._clear_hotspot_waiting_if_needed(next_node_name)
                self._reset_wait(carrying=(self.cargo_job is not None))

                x0, y0 = cur.x, cur.y
                x1, y1 = nxt.x, nxt.y

                dx = x1 - x0
                dy = y1 - y0
                norm = (dx ** 2 + dy ** 2) ** 0.5
                if norm > 0:
                    self.dir_x = dx / norm
                    self.dir_y = dy / norm

                self.draw_x = x0
                self.draw_y = y0
                self.move_x0 = x0
                self.move_y0 = y0
                self.move_x1 = x1
                self.move_y1 = y1
                self.move_t0 = self.env.now()
                self.move_t1 = self.env.now() + edge.travel_time
                self.move_active = True
                phase = "loaded" if self.cargo_job is not None else "empty"
                self._append_oht_event_trace({
                    "sim_time": round(float(self.move_t0), 3),
                    "end_time": round(float(self.move_t1), 3),
                    "oht_id": self.vid,
                    "event": "EDGE_START",
                    "state": self.state_name,
                    "phase": phase,
                    "from_node": self.pos_node,
                    "to_node": next_node_name,
                    "x": round(float(x0), 3), "y": round(float(y0), 3),
                    "x1": round(float(x1), 3), "y1": round(float(y1), 3),
                    "job_id": getattr(self.cargo_job, "job_type_id", ""),
                    "job_instance_id": getattr(self.cargo_job, "job_id", ""),
                    "op_index": getattr(self.cargo_job, "op_index", ""),
                    "dispatch_mode": getattr(self.oht_config, "oht_dispatch_mode", ""),
                })

                self._transit_acc += edge.travel_time
                yield self.hold(edge.travel_time)

                self.move_active = False
                self.pos_node = next_node_name
                self._append_oht_event_trace({
                    "sim_time": round(float(self.env.now()), 3),
                    "end_time": round(float(self.env.now()), 3),
                    "oht_id": self.vid,
                    "event": "EDGE_END",
                    "state": self.state_name,
                    "phase": phase,
                    "from_node": cur.name,
                    "to_node": next_node_name,
                    "x": round(float(x1), 3), "y": round(float(y1), 3),
                    "job_id": getattr(self.cargo_job, "job_type_id", ""),
                    "job_instance_id": getattr(self.cargo_job, "job_id", ""),
                    "op_index": getattr(self.cargo_job, "op_index", ""),
                    "dispatch_mode": getattr(self.oht_config, "oht_dispatch_mode", ""),
                })
                self._record_route_node_pass(next_node_name)
                self._record_hotspot_fork_pass(next_node_name)
                self.draw_x = x1
                self.draw_y = y1
                self.release(edge.res)
                self._release_zcu_if_station_arrival()

            finally:
                self.next_hop_intent = None

        # --------------------------------------------------
        # STEP A) An OHT already inside guide/curve never waits midway.
        #         That is, no Waiting/Breaked decision inside a curve.
        # --------------------------------------------------
        if cur.kind != 'station':
            yield from _do_move()
            return

        # --------------------------------------------------
        # STEP B) Forward-buffer check before departing a station
        #         If next_node is a guide, pre-check the whole curve + up to the merge station.
        #         If blocked, wait only at the current station point.
        # --------------------------------------------------
        front_nodes = self._prewait_buffer_nodes(next_node_name)

        if _front_chain_blocked(front_nodes):
            yield _wait_here()
            return

        # ZCU/SPZR: enter the merge/fork/curve conflict zone only when it is empty.
        # If blocked, always wait at the current station.
        if self._zcu_blocked(front_nodes):
            yield _wait_here()
            return

        # --------------------------------------------------
        # STEP 0) Collision/simultaneous entry on the next node itself
        #         This check is now performed only at a station.
        # --------------------------------------------------
        for v in self.fleet:
            if v is self:
                continue

            if getattr(v, 'pos_node', None) == next_node_name:
                yield _wait_here()
                return

            if (
                    getattr(v, 'next_hop_intent', None) == next_node_name
                    and (not _has_priority_over(v))
            ):
                yield _wait_here()
                return

        # --------------------------------------------------
        # STEP 1) Decide entry into curve -> straight merge
        #   - Decide only at the current station point
        #   - Once it enters a guide, it does not stop midway
        # --------------------------------------------------
        merge_station, curve_chain = self._curve_merge_station(next_node_name)
        if merge_station is not None:
            curve_chain_set = set(curve_chain)
            rival_preds = set(self._incoming_predecessors(merge_station))
            rival_preds -= curve_chain_set
            rival_preds.discard(self.pos_node)

            should_wait = False

            for v in self.fleet:
                if v is self:
                    continue

                other_pos = getattr(v, 'pos_node', None)
                other_intent = getattr(v, 'next_hop_intent', None)

                # If a straight rival branch is right before the merge, the curve waits at the current station
                if other_pos in rival_preds or other_intent in rival_preds:
                    should_wait = True
                    break

                # Wait if another OHT is already in the merge station or inside the curve
                in_merge_zone = (
                        other_pos == merge_station
                        or other_pos in curve_chain_set
                        or other_intent == merge_station
                        or other_intent in curve_chain_set
                )

                if in_merge_zone and (not _has_priority_over(v)):
                    should_wait = True
                    break

            if should_wait:
                self._update_merge_wait_flag(f"merge::{merge_station}", True)
                yield _wait_here()
                return
            else:
                self._update_merge_wait_flag(f"merge::{merge_station}", False)

            # If entry is allowed, reserve the ZCU and keep going once inside the guide
            self._reserve_zcu(front_nodes)
            yield from _do_move()
            return

        # --------------------------------------------------
        # STEP 2) On straight approach, yield to curve vehicles
        #   - The straight vehicle also waits only at the current station point
        #   - If the curve is waiting or already committed (inside the guide), the straight yields
        # --------------------------------------------------
        upcoming_merge = self._upcoming_merge_station_on_straight(
            next_node_name,
            max_hops=6
        )

        if upcoming_merge is not None:
            waiters = self._curve_waiters_for_merge(upcoming_merge)
            committed = self._curve_committed_for_merge(upcoming_merge)

            if waiters or committed:
                longest_wait = 0.0
                for v in waiters:
                    since = getattr(v, 'merge_wait_since', None)
                    if since is not None:
                        longest_wait = max(
                            longest_wait,
                            self.env.now() - float(since)
                        )

                extra_yield = 0.0
                curve_wait_sec = float(
                    getattr(self.oht_config, 'curve_priority_wait_sec', 2.0)
                )
                straight_yield_sec = float(
                    getattr(self.oht_config, 'straight_yield_sec', 1.2)
                )

                if longest_wait >= curve_wait_sec:
                    extra_yield = straight_yield_sec

                yield _wait_here(extra_hold=extra_yield)
                return

        # --------------------------------------------------
        # STEP 3) Normal move
        # --------------------------------------------------
        self._reserve_zcu(front_nodes)
        yield from _do_move()

    # -----------------------------------------------------
    # transport event logging
    # -----------------------------------------------------
    def _machine_id_for_node(self, node_name):
        machine = self.machines.get(node_name)
        if machine is not None and getattr(machine, "jssp_machine_id", None) is not None:
            return int(machine.jssp_machine_id) + 1
        node = self.nodes.get(node_name)
        if node is not None and getattr(node, "machine_no", None) is not None:
            return int(node.machine_no)
        return ""

    def _find_source_process_end_time(self, source_machine, job):
        if source_machine is None or job is None:
            return None
        target_instance = int(getattr(job, "job_id", -1))
        target_op = int(getattr(job, "op_index", -1))
        candidates = []
        for row in getattr(source_machine, "_sim_log", []):
            try:
                if int(row.get("job_id", -999)) != target_instance:
                    continue
                if int(row.get("op_index", -999)) != target_op:
                    continue
                candidates.append(float(row.get("end_time", row.get("sim_time", 0.0))))
            except Exception:
                continue
        if candidates:
            return max(candidates)
        return None

    def _append_transport_event(self, rec):
        append_transport_event(self.transport_event_log, self.transport_live_state, rec)

    def _append_oht_event_trace(self, rec):
        """One row per vehicle move. Only written when the UI is attached."""
        append_oht_event_trace(self.oht_event_trace_state, rec)

    # -----------------------------------------------------
    # load / unload
    # -----------------------------------------------------
    def load_job(self):
        self.state_name = "Loading"
        yield self.hold(
            random.uniform(
                self.oht_config.load_time_min,
                self.oht_config.load_time_max,
            )
        )
        self.state_name = "Loaded"

    def unload_job(self):
        self.state_name = "Unloading"
        yield self.hold(
            random.uniform(
                self.oht_config.unload_time_min,
                self.oht_config.unload_time_max,
            )
        )

    def body_fill_color(self):
        # 1) While loading, keep blue meaning 'mission/loading in progress'
        if self.state_name == "Loading":
            return self.oht_config.oht_task_fill_color

        # 2) While unloading / after loading complete, keep green
        if self.state_name in ("Loaded", "Unloading"):
            return self.oht_config.oht_cargo_fill_color

        # 3) Green if actually carrying cargo and moving
        if self.cargo_job is not None:
            return self.oht_config.oht_cargo_fill_color

        # 4) Blue if only a mission has been assigned
        if self.assigned_task is not None:
            return self.oht_config.oht_task_fill_color

        # 5) Otherwise the default color
        return self.oht_config.state_style[self.state_name]["fill"]

    def _blink_on(self):
        period = max(0.1, float(getattr(self.oht_config, "blink_period_sec", 0.4)))
        return int(self.env.now() / period) % 2 == 0

    def outline_color(self):
        if self.state_name == "Waiting":
            return self.oht_config.waiting_line_color

        if self.state_name == "Loading":
            return (
                self.oht_config.loading_blink_on_line
                if self._blink_on()
                else self.oht_config.loading_blink_off_line
            )

        if self.state_name == "Unloading":
            return (
                self.oht_config.unloading_blink_on_line
                if self._blink_on()
                else self.oht_config.unloading_blink_off_line
            )

        return self.oht_config.state_style[self.state_name]["line"]

    # -----------------------------------------------------
    # main process
    # -----------------------------------------------------
    def process(self):
        self.reserve_start_node()
        self._idle_start = self.env.now()

        while True:
            # ── No task: move by IdleBehavior ────────────────────
            if self.assigned_task is None:
                # If it just finished (or was canceled from) a task and is empty → wake the dispatcher
                # to reflect this vehicle as the next dispatch candidate immediately (event-based).
                if self._had_task:
                    self._had_task = False
                    if self._dispatcher is not None:
                        self._dispatcher.request_dispatch()
                # C1: if there is a roam_target (a machine that will soon have a pickup), take one hop there; otherwise random roam.
                #   Since it is not a task, can_accept_dispatch stays true → reactive dispatch can preempt anytime.
                _rt = getattr(self, "roam_target", None)
                if _rt and _rt != self.pos_node and _rt in self.nodes:
                    _p = self.shortest_path(self.pos_node, _rt)
                    next_nd = _p[1] if len(_p) > 1 else self.idle_behavior.next_node(self.pos_node, self.adj)
                else:
                    next_nd = self.idle_behavior.next_node(self.pos_node, self.adj)

                # Keep the existing idle_roaming.
                # But if an OHT blocks for 5+ seconds around the 9/6 hotspot (per layout_oht.csv),
                # steer the 13 fork toward the 17-direction arc and the 18 fork toward the 14-direction arc.
                next_nd = self._choose_idle_next_node_by_hotspot_escape(next_nd)

                if next_nd == self.pos_node:
                    # Cannot move → check accumulated stop time
                    if (self.env.now() - self._idle_start
                            >= self.oht_config.idle_to_waiting_sec):
                        self.state_name = "Waiting"
                        self.is_idle = True
                        # ── Relocate after Waiting timeout ─────────────────
                        # hold(timeout) instead of passivate(): wakes immediately on task dispatch
                        # or relocates to another node after waiting_relocate_sec elapses
                        yield self.hold(self.oht_config.waiting_relocate_sec)
                        self._idle_start = self.env.now()
                        self.is_idle = False
                        # If timed out without a task → move to an adjacent station node
                        if self.assigned_task is None:
                            self.state_name = "Empty"
                            _reloc = [
                                n for n in self.adj.get(self.pos_node, [])
                                if self.nodes.get(n)
                                   and self.nodes[n].kind == 'station'
                            ]
                            if _reloc:
                                yield from self.move_one_hop(random.choice(_reloc))
                    else:
                        yield self.hold(self.oht_config.block_retry_dt)
                else:
                    self._idle_start = self.env.now()
                    self.state_name = "Empty"
                    self.is_idle = False
                    yield from self.move_one_hop(next_nd)
                continue

            # ── Has a task ───────────────────────────────────────────
            self.is_idle = False
            self._had_task = True     # when this task ends, wake the dispatcher on idle transition
            self._idle_start = self.env.now()

            active_task = self.assigned_task
            dispatch_time = getattr(active_task, "_dispatch_time", None)
            if dispatch_time is None:
                dispatch_time = self.env.now()

            src = self.assigned_task.source_name

            # ── Move to source ────────────────────────────────────────
            # Re-read assigned_task.source_name each iter so that if reassigned externally,
            # it immediately redirects to the new source.
            while self.assigned_task is not None and self.pos_node != self.assigned_task.source_name:
                src = self.assigned_task.source_name
                path = self.shortest_path(self.pos_node, src)
                if len(path) <= 1:
                    break
                yield from self.move_one_hop(path[1])

            if self.assigned_task is None:
                self.state_name = "Empty"
                self._idle_start = self.env.now()
                continue
            src = self.assigned_task.source_name
            source_arrival_time = self.env.now()

            # source not reached → cancel the task (Bug 1/2 fix)
            if self.pos_node != src:
                self.assigned_task = None
                self.cargo_job = None
                self.state_name = "Empty"
                self._idle_start = self.env.now()
                continue

            # ── Actual job pickup from source machine out_buffer ──────────
            # Match by job_type_id (pick up the same job type as the schedule)
            src_machine = self.machines.get(src)
            pickup_wait_start_time = self.env.now()
            pickup_time = None
            prev_machine_end_time = None

            if src_machine is not None:
                target_type = (self.assigned_task.job_id
                               if self.assigned_task is not None else None)
                target_instance = (getattr(self.assigned_task, 'job_instance_id', None)
                                   if self.assigned_task is not None else None)

                pickup_timeout = 120.0  # adjust between 30~120 if needed
                waited = 0.0
                pickup_ok = False

                while waited < pickup_timeout:
                    matched = None
                    for _j in src_machine.out_buffer:
                        if target_instance is not None and _j.job_id == target_instance:
                            matched = _j
                            break
                        if target_instance is None and (target_type is None or _j.job_type_id == target_type):
                            matched = _j
                            break

                    if matched is not None:
                        src_machine.out_buffer.remove(matched)
                        self.cargo_job = matched
                        pickup_time = self.env.now()
                        prev_machine_end_time = self._find_source_process_end_time(src_machine, matched)
                        pickup_ok = True
                        break

                    self._set_service_wait()
                    yield self.hold(1.0)
                    waited += 1.0

                # timeout: give up this task → return to idle
                if not pickup_ok:
                    self.assigned_task = None
                    self.cargo_job = None
                    self.state_name = "Empty"
                    self.is_idle = False
                    self._idle_start = self.env.now()
                    continue

            # loading time (meaningful only when there is a job)
            load_start_time = self.env.now()
            yield from self.load_job()
            load_end_time = self.env.now()

            # ── Decide destination: job routing first (Bug 6 fix) ───────────
            if self.cargo_job is not None and self.cargo_job.next_candidate_nodes:
                dst = self.cargo_job.next_candidate_nodes[0]
            else:
                dst = self.assigned_task.dest_name

            # ── Move to destination ─────────────────────────────────
            loaded_travel_start_time = self.env.now()
            _transit0 = self._transit_acc
            _block0 = self._block_acc
            # Theoretical shortest free-flow (at pickup time, no-congestion assumption) — baseline to detect excess detour
            _ff_path = self.shortest_path(self.pos_node, dst)
            free_flow_shortest_time = 0.0
            for _u, _v in zip(_ff_path[:-1], _ff_path[1:]):
                _eid = self.uv_to_eid.get((_u, _v))
                if _eid is not None:
                    free_flow_shortest_time += float(getattr(self.edges[_eid], 'travel_time', 0.0))
            while self.pos_node != dst:
                path = self.shortest_path(self.pos_node, dst)
                if len(path) <= 1:
                    break
                yield from self.move_one_hop(path[1])

            dest_arrival_time = self.env.now()
            # Sum of free-flow over the actually driven path / time stopped-blocked on it
            free_flow_actual_path_time = max(0.0, self._transit_acc - _transit0)
            blocking_loaded_time = max(0.0, self._block_acc - _block0)

            # destination not reached (broken path, etc.) → do not drop the lot, return it to the source out_buffer.
            # (same handling as drop-timeout return) prevents lot loss / WIP leakage and lets it be re-dispatched.
            if self.pos_node != dst and self.cargo_job is not None:
                print(f"[OHT-{self.vid}] warning: {dst} not reached → "
                      f"job {self.cargo_job.job_id} returned to source({src})")
                src_machine = self.machines.get(src)
                if src_machine is not None:
                    src_machine.out_buffer.append(self.cargo_job)
                self.assigned_task = None
                self.cargo_job = None
                self.state_name = "Empty"
                self.is_idle = False
                self._idle_start = self.env.now()
                continue

            # unload time
            self._reset_wait(carrying=(self.cargo_job is not None))
            unload_start_time = self.env.now()
            yield from self.unload_job()
            unload_end_time = self.env.now()

            # ── Actual job hand-off to destination machine in_buffer ────────
            dropoff_time = None
            drop_ok = False
            if self.cargo_job is not None:
                dst_machine = self.machines.get(dst)
                if dst_machine is not None:
                    drop_timeout = 60.0  # adjust between 10~60 if needed
                    waited = 0.0
                    drop_ok = False

                    while waited < drop_timeout:
                        if dst_machine.can_accept_input():
                            dst_machine.reserve_input_slot()
                            try:
                                dst_machine.receive_job(self.cargo_job)
                                dropoff_time = self.env.now()
                                drop_ok = True
                            except Exception:
                                dst_machine.release_reserved_input_slot()
                                drop_ok = False
                            break

                        self._set_service_wait()
                        yield self.hold(0.5)
                        waited += 0.5

                    # timeout: if it could not drop off, give up this task and let it be re-dispatched back to the source
                    if not drop_ok:
                        # return cargo_job back to the source machine out_buffer
                        src_machine = self.machines.get(src)
                        if src_machine is not None:
                            src_machine.out_buffer.append(self.cargo_job)

                        self.assigned_task = None
                        self.cargo_job = None
                        self.state_name = "Empty"
                        self.is_idle = False
                        self._idle_start = self.env.now()
                        continue

                if drop_ok and dropoff_time is not None:
                    job = self.cargo_job
                    prev_end = prev_machine_end_time
                    if prev_end is None:
                        prev_end = pickup_wait_start_time

                    # actual_transport_time basis:
                    # from when the OHT finished loading at the source (load_end_time)
                    # to when unloading finished at the destination machine (unload_end_time).
                    # That is, excluding pickup wait / empty travel / loading time after the previous machine finished,
                    # record the time from actual loaded travel to unloading completion as transport_time.
                    actual_transport_time = max(0.0, float(unload_end_time) - float(load_end_time))
                    pure_oht_transport_time = max(0.0, dropoff_time - float(pickup_time or pickup_wait_start_time))
                    loaded_travel_time = max(0.0, dest_arrival_time - loaded_travel_start_time)

                    self._append_transport_event({
                        "oht_id": self.vid,
                        "job_id": int(getattr(job, "job_type_id", -1)) + 1,
                        "job_instance_id": int(getattr(job, "job_id", 0)),
                        "op_index": int(getattr(job, "op_index", 0)),
                        "from_machine": self._machine_id_for_node(src),
                        "to_machine": self._machine_id_for_node(dst),
                        "from_node": src,
                        "to_node": dst,
                        "dispatch_time": round(float(dispatch_time), 3),
                        "prev_machine_end_time": round(float(prev_end), 3),
                        "source_arrival_time": round(float(source_arrival_time), 3),
                        "pickup_wait_start_time": round(float(pickup_wait_start_time), 3),
                        "pickup_time": round(float(pickup_time or pickup_wait_start_time), 3),
                        "load_start_time": round(float(load_start_time), 3),
                        "load_end_time": round(float(load_end_time), 3),
                        "loaded_travel_start_time": round(float(loaded_travel_start_time), 3),
                        "dest_arrival_time": round(float(dest_arrival_time), 3),
                        "unload_start_time": round(float(unload_start_time), 3),
                        "unload_end_time": round(float(unload_end_time), 3),
                        "dropoff_time": round(float(dropoff_time), 3),
                        "source_wait_time": round(
                            max(0.0, float(pickup_time or pickup_wait_start_time) - float(prev_end)), 3),
                        "empty_to_source_time": round(max(0.0, float(source_arrival_time) - float(dispatch_time)), 3),
                        "pickup_wait_time": round(
                            max(0.0, float(pickup_time or pickup_wait_start_time) - float(pickup_wait_start_time)), 3),
                        "loading_time": round(max(0.0, float(load_end_time) - float(load_start_time)), 3),
                        "loaded_travel_time": round(loaded_travel_time, 3),
                        "free_flow_shortest_time": round(free_flow_shortest_time, 3),
                        "free_flow_actual_path_time": round(free_flow_actual_path_time, 3),
                        "blocking_loaded_time": round(blocking_loaded_time, 3),
                        "blocking_ratio": round(blocking_loaded_time / loaded_travel_time,
                                                3) if loaded_travel_time > 0 else 0.0,
                        "detour_ratio": round(free_flow_actual_path_time / free_flow_shortest_time,
                                              3) if free_flow_shortest_time > 0 else 0.0,
                        "unloading_time": round(max(0.0, float(unload_end_time) - float(unload_start_time)), 3),
                        "drop_wait_time": round(max(0.0, float(dropoff_time) - float(unload_end_time)), 3),
                        "pure_oht_transport_time": round(pure_oht_transport_time, 3),
                        "actual_transport_time": round(actual_transport_time, 3),
                        # ── gantt transport delay label (phase 5b) ──────────────
                        # From when the lot was ready at the source (prev_end) to arrival at dest (dropoff),
                        # actual elapsed − gantt planned free-flow transport time = pure transport delay.
                        # The target that positioning reduces (including empty_to_source + source_wait).
                        "lot_id": getattr(job, "lot_id", "")
                                  or getattr(active_task, "lot_id", ""),
                        "step_no": int(getattr(job, "step_no", 0)
                                       or getattr(active_task, "step_no", 0)),
                        "planned_travel": round(float(getattr(active_task, "planned_travel", 0.0)), 3),
                        "transport_deviation": round(
                            max(0.0, float(dropoff_time) - float(prev_end))
                            - float(getattr(active_task, "planned_travel", 0.0)), 3),
                    })

                self.cargo_job = None

            self.assigned_task = None
            self.is_idle = False
            self._idle_start = self.env.now()
            self.state_name = "Empty"

    # -----------------------------------------------------
    # shape
    # -----------------------------------------------------
    def triangle_spec(self):
        L = self.oht_config.oht_length
        W = self.oht_config.oht_width

        dx = self.dir_x
        dy = self.dir_y

        px = -dy
        py = dx

        fx = dx * (L / 2)
        fy = dy * (L / 2)

        bx = -dx * (L / 2)
        by = -dy * (L / 2)

        lx = bx + px * (W / 2)
        ly = by + py * (W / 2)

        rx = bx - px * (W / 2)
        ry = by - py * (W / 2)

        return (fx, fy, lx, ly, rx, ry)


class JobSource(sim.Component):
    """
    Create lot instances along the gantt lot route and inject them into each machine in_buffer.

    CONWIP: keep the number of lots in the fab (WIP) at wip_cap.
      - Initially: while WIP < cap, take a lot from the pool and inject it into the first-step machine.
      - On each completion (fab-out) signal (notify_fabout), WIP decreases → inject the next lot.
      - Terminate when the pool (=gantt lot_ids) is exhausted.
    """

    def setup(self, machines, schedule, machine_node_map, jssp_cfg):
        self.machines = machines
        self.schedule = schedule
        self.machine_node_map = machine_node_map
        self.jssp_cfg = jssp_cfg
        self.wip_cap = int(getattr(jssp_cfg, "wip_cap", 300) or 300)
        self.pool = list(schedule.lot_ids()) if schedule is not None else []
        # Fixed-workload makespan: limit the pool to the first N (same lot set for all methods → fair makespan)
        _plim = int(getattr(jssp_cfg, "pool_limit", 0) or 0)
        if _plim > 0:
            self.pool = self.pool[:_plim]
        self.ptr = 0
        self.wip = 0
        self.completed = 0
        self.completion_times = []   # fab-out time of each lot (for throughput@24h + makespan)
        self.cycle_times = []        # each lot cycle time = fab-out − release(created_time), index-aligned with completion_times
        self.job_seq = 0

    # Completion (fab-out) callback: called when MachineStation finishes the last step
    def notify_fabout(self, job):
        self.wip = max(0, self.wip - 1)
        self.completed += 1
        _now = self.env.now()
        self.completion_times.append(_now)
        self.cycle_times.append(_now - float(getattr(job, "created_time", _now)))
        if self.ispassive():
            self.activate()

    def _try_inject_next(self) -> bool:
        """Inject the next lot from the pool into the first-step machine. True on success."""
        if self.ptr >= len(self.pool):
            return False
        # 24h fixed workload: stop new releases after cutoff (drain only already-entered lots → measure makespan)
        _cut = float(getattr(self.jssp_cfg, "inject_cutoff_sec", 0.0) or 0.0)
        if _cut > 0.0 and self.env.now() > _cut:
            return False
        lot_id = self.pool[self.ptr]
        first = self.schedule.first_op(lot_id)
        if first is None:
            self.ptr += 1
            return False
        node = self.machine_node_map.get(first.machine)
        if node is None or node not in self.machines:
            self.ptr += 1
            return False
        machine = self.machines[node]
        if not machine.can_accept_input():
            return False  # no buffer room → retry later (keep ptr)

        nxt = self.schedule.next_machine_after(lot_id, first.step_no)
        next_node = self.machine_node_map.get(nxt) if nxt else None
        self.job_seq += 1
        _now = self.env.now()
        job = Job(
            job_id=self.job_seq,
            job_type_id=0,
            op_index=0,
            product_type=lot_id,
            origin=node,
            current_node=node,
            next_candidate_nodes=[next_node] if next_node else [],
            created_time=_now,
            lot_id=lot_id,
            step_no=first.step_no,
        )
        # label B baseline: first-step interval = (release time→first-step end) vs (planned_start→planned_end)
        job.prev_realized_ready = _now
        job.prev_planned_end = first.planned_start
        machine.receive_job(job)
        self.ptr += 1
        self.wip += 1
        return True

    def process(self):
        if self.schedule is None or not self.pool:
            yield self.passivate()
            return

        while self.ptr < len(self.pool):
            if self.wip < self.wip_cap:
                if self._try_inject_next():
                    yield self.hold(0.01)       # prevent duplicate injection at the same time
                else:
                    yield self.hold(1.0)        # buffer full → retry
            else:
                yield self.passivate()          # WIP full → fab-out wakes it

        print(f"[JobSource] CONWIP pool exhausted: released {self.job_seq}, completed {self.completed} "
              f"(t={self.env.now():.1f}s)")
        yield self.passivate()


class _SavdPositioningMonitor(sim.Component):
    """SAVD: every `oht_savd_dt` seconds, match the spatial distribution of idle
    OHTs to the plan's demand density (this is what targets the source_wait tail).

    It only sets roam targets -- no vehicle is committed to a task here, so the
    dispatch decision itself is identical to the plain HUNGARIAN arm."""
    def setup(self, runner):
        self.runner = runner

    def process(self):
        cfg = self.runner.oht_config
        dt = float(getattr(cfg, 'oht_savd_dt', 45.0))
        win = float(getattr(cfg, 'oht_savd_window', 900.0))
        grid = float(getattr(cfg, 'oht_savd_grid', 0.0))
        pw = float(getattr(cfg, 'oht_savd_prior_weight', 1.0))
        terms = str(getattr(cfg, 'oht_savd_terms', 'all'))
        anch = bool(getattr(cfg, 'oht_savd_roll_anchor', True))
        dm = getattr(self.runner, '_oht_dm', None)
        self.runner._savd_calls = 0
        self.runner._savd_assigns = 0
        while True:
            yield self.hold(dt)
            if dm is None:
                continue
            self.runner._savd_calls += 1
            try:
                n = dm._assign_savd_positioning(self.env.now(), window=win,
                                                grid=grid, prior_weight=pw,
                                                terms=terms, roll_anchor=anch)
                self.runner._savd_assigns += int(n or 0)
            except Exception as exc:
                # never let a positioning hiccup kill a run, but do not hide it
                # either: count them and report the first one
                self.runner._savd_errs = getattr(self.runner, '_savd_errs', 0) + 1
                if self.runner._savd_errs == 1:
                    print(f"[SAVD] positioning error (run continues): {exc!r}")


class SimulationRunner:
    def __init__(
            self,
            layout_csv_path: str = "layout_ex.csv",
            machine_csv_path: str = "layout_Machine.csv",
            jssp_cfg: JSSPConfig = None,
    ):
        self.layout_csv_path = layout_csv_path
        self.machine_csv_path = machine_csv_path
        self.jssp_cfg = jssp_cfg or JSSPConfig()

        self.oht_config = OHTConfig()
        self.machine_config = MachineConfig()

        # Before simulation: JSSP scheduling + save 3 CSVs
        self.machine_decision = MachineDecisionMaker(self.jssp_cfg)

        self.env = None
        self.layout_builder = None

        self.nodes = {}
        self.edges = {}
        self.adj = {}
        self.uv_to_eid = {}
        self.machines = {}
        self.vehicles = []
        self.transport_event_log = []
        # UI live-trace sinks: created in run() only when an output dir exists,
        # so a headless experiment run pays nothing for them
        self.transport_live_state = None
        self.machine_live_state = None
        self.oht_event_trace_state = None

        self.machine_node_map: dict = {}  # {jssp_machine_id: node_name}
        self._machine_nodes_sorted = []
        self._oht_dm = None

        self.node_last_pass_time = {}
        self.route_node_pass_times = {}
        self.hotspot_escape_next_by_node = {}
        self.hotspot_watch_nodes = set()
        self.node_hotspot_wait_start = {}
        self.hotspot_active_until = {"until": -1e18}
        self.hotspot_node_to_cell_code = {}
        self.hotspot_node_block_until = {}
        self.hotspot_fork_pass_times = {}
        self.hotspot_fork_block_until = {}
        self.zcu_node_to_zone = {}
        self.zcu_occupied_by = {}
        self.job_quantities = {}
        self.total_planned_jobs = 0
        self.total_planned_ops = 0
        self._panel_geom = {}
        self._window_width = 1900
        self._window_height = 1050
        self._prepare_targets()

    def _prepare_targets(self):
        sched = getattr(self.machine_decision, "schedule", None)
        if sched is None or not getattr(sched, "routes", None):
            self.job_quantities = {}
            self.total_planned_jobs = 0
            self.total_planned_ops = 0
            return
        # gantt: number of lots = pool size, number of ops = total op sum
        self.job_quantities = {}   # in gantt mode there is no per-job-type quantity concept
        self.total_planned_jobs = len(sched.routes)
        self.total_planned_ops = sum(len(r.ops) for r in sched.routes.values())

    def _edge_cost(self, u, v) -> float:
        eid = self.uv_to_eid.get((u, v))
        if eid is not None and eid in self.edges:
            return float(getattr(self.edges[eid], 'travel_time', 1.0))
        if u in self.nodes and v in self.nodes:
            return ((self.nodes[u].x - self.nodes[v].x) ** 2
                    + (self.nodes[u].y - self.nodes[v].y) ** 2) ** 0.5
        return 1.0

    def _dijkstra_cost(self, start, goal) -> float:
        if start == goal:
            return 0.0
        if start not in self.adj or goal not in self.nodes:
            return 0.0
        dist = {start: 0.0}
        heap = [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                return d
            if d > dist.get(u, 1e18):
                continue
            for v in self.adj.get(u, []):
                nd = d + self._edge_cost(u, v)
                if nd < dist.get(v, 1e18):
                    dist[v] = nd
                    heapq.heappush(heap, (nd, v))
        return 0.0

    def _machine_transport_time(self, m_from: int, m_to: int) -> float:
        """
        Layout-based estimate of transport time between machines. Injected into the scheduling algorithm
        to reflect inter-op transport delay in the plan.

        m_from / m_to : 0-based JSSP machine id
        """
        if m_from == m_to:
            return 0.0
        na = self.machine_node_map.get(m_from)
        nb = self.machine_node_map.get(m_to)
        if not na or not nb or na not in self.nodes or nb not in self.nodes:
            return 0.0
        # Cache results (the scheduler calls the same pair repeatedly)
        if not hasattr(self, '_mt_cache'):
            self._mt_cache = {}
        key = (m_from, m_to)
        if key in self._mt_cache:
            return self._mt_cache[key]
        # pure A* travel + load/unload overhead (always incurred when a loaded OHT moves between machines)
        cost = self._dijkstra_cost(na, nb)
        load = float(getattr(self.oht_config, 'load_time_min', 0.0) or 0.0)
        unload = float(getattr(self.oht_config, 'unload_time_min', 0.0) or 0.0)
        cost += load + unload
        self._mt_cache[key] = cost
        return cost

    def _completed_jobs_count(self) -> int:
        return sum(getattr(ms, 'completed_final_jobs', 0) for ms in self.machines.values())

    def _completed_ops_count(self) -> int:
        return sum(getattr(ms, 'completed_ops_count', 0) for ms in self.machines.values())

    def _machine_avg_util_pct(self) -> float:
        if not self.machines:
            return 0.0
        now = self.env.now() if self.env is not None else 0.0
        vals = [ms.utilization_pct(now) for ms in self.machines.values()]
        return sum(vals) / len(vals) if vals else 0.0

    def _wip_count(self) -> int:
        total = 0
        for ms in self.machines.values():
            total += len(ms.in_buffer) + len(ms.out_buffer)
            total += 1 if ms.processing_job is not None else 0
        for veh in self.vehicles:
            total += 1 if veh.cargo_job is not None else 0
        return total

    def _oht_state_counts(self):
        return Counter(v.state_name for v in self.vehicles)

    def _active_oht_count(self) -> int:
        return sum(1 for v in self.vehicles if v.assigned_task is not None or v.cargo_job is not None)

    def _status_color(self, ms):
        if ms.is_processing():
            return '#ffb000'
        if ms.visual_out_count() > 0:
            return '#00d26a'
        if ms.visual_in_count() > 0:
            return '#38bdf8'
        return '#7f8c9a'

    def _status_text_color(self, ms):
        if ms.is_processing():
            return '#ffcf66'
        if ms.visual_out_count() > 0:
            return '#7dff9b'
        if ms.visual_in_count() > 0:
            return '#8fe7ff'
        return '#a8b3c2'

    def _fmt_pct(self, value: float) -> str:
        return f'{value:5.1f}%'

    def _fmt_ratio(self, numer: int, denom: int) -> str:
        if denom <= 0:
            return '0/0'
        return f'{numer}/{denom}'

    # -----------------------------------------------------
    # hotspot escape mapping helpers
    # -----------------------------------------------------
    def _cell_base_code_from_layout(self, r: int, c: int):
        if self.layout_builder is None or getattr(self.layout_builder, "grid", None) is None:
            return None
        return self.layout_builder._cell_base_code(r, c)

    def _corner_node_from_layout_cell(self, r: int, c: int, corner: str):
        if self.layout_builder is None:
            return None
        dvr, dvc = self.layout_builder.CORNER_DV[corner]
        return self.layout_builder._vertex_cache.get((r + dvr, c + dvc))

    def _build_hotspot_escape_next_by_node(self):
        """
        Auto-generate the escape-direction next hop from the fork cells in layout_oht.csv.
        - 13 fork: pick the arc-direction next hop instead of straight -> effect of exiting toward 17
        - 18 fork: pick the arc-direction next hop instead of straight -> effect of exiting toward 14
        """
        mapping = {}
        if self.layout_builder is None or getattr(self.layout_builder, "grid", None) is None:
            return mapping

        lb = self.layout_builder
        fork_codes = set(str(x) for x in getattr(self.oht_config, "hotspot_escape_fork_codes", {"13", "18"}))

        for r in range(lb.rows):
            for c in range(lb.cols):
                tok = self._cell_base_code_from_layout(r, c)
                if tok not in fork_codes:
                    continue
                if tok not in lb.JUNCTION_CODES:
                    continue

                shared_c, str_other_c, arc_other_c, arc_cp_c, mode = lb.TILE_JUNCTION_DEF[tok]
                if mode != "fork":
                    continue

                source = self._corner_node_from_layout_cell(r, c, shared_c)
                straight_other = self._corner_node_from_layout_cell(r, c, str_other_c)
                if source is None:
                    continue

                # Among source's neighbors, the one that is not straight_other is the first next hop in the arc direction.
                candidates = [n for n in self.adj.get(source, []) if n != straight_other]
                if not candidates:
                    continue

                # Prefer selecting an ARC/guide node.
                guide_candidates = [
                    n for n in candidates
                    if self.nodes.get(n) is not None and self.nodes[n].kind == "guide"
                ]
                mapping[source] = guide_candidates[0] if guide_candidates else candidates[0]

        return mapping

    def _build_hotspot_watch_nodes(self):
        """
        Create only the 9/6 cells 'themselves' (per layout_oht.csv) as hotspot watch zones.

        Important:
        - Including TILE_CORNERS as before shares corner nodes with adjacent cells,
          so even the surroundings of 9/6 get recognized as hotspots.
        - Therefore, here only the ARC guide nodes created inside the 9/6 cells are watched.
        - That is, only blockage 'exactly inside the 9/6 curve' counts as a hotspot.
        """
        watch_nodes = set()

        if self.layout_builder is None or getattr(self.layout_builder, "grid", None) is None:
            return watch_nodes

        lb = self.layout_builder
        watch_codes = set(
            str(x) for x in getattr(
                self.oht_config,
                "hotspot_watch_cell_codes",
                {"9", "6"}
            )
        )

        for r in range(lb.rows):
            for c in range(lb.cols):
                tok = self._cell_base_code_from_layout(r, c)

                if tok not in watch_codes:
                    continue

                # Key fix:
                # Corner station nodes are shared with adjacent cells, so exclude them.
                # Register only the ARC guide nodes inside those 9/6 cells as hotspots.
                for node_name, node in self.nodes.items():
                    if (
                            getattr(node, "kind", None) == "guide"
                            and getattr(node, "rc", None) == (r, c)
                    ):
                        watch_nodes.add(node_name)

                # For fork cells like 13/18, watch not only the guide but also the actual branching station source.
                # So that when traffic pours into 13/18, the fork itself becomes a secondary hotspot.
                fork_codes = set(str(x) for x in getattr(self.oht_config, "hotspot_escape_fork_codes", {"13", "18"}))
                if tok in fork_codes and tok in lb.JUNCTION_CODES:
                    shared_c, str_other_c, arc_other_c, arc_cp_c, mode = lb.TILE_JUNCTION_DEF[tok]
                    if mode == "fork":
                        source = self._corner_node_from_layout_cell(r, c, shared_c)
                        if source is not None:
                            watch_nodes.add(source)

        return watch_nodes

    def _build_hotspot_node_to_cell_code(self):
        """
        Store which cell code in layout_oht.csv each hotspot watch node came from.
        9/6 centers on arc guide nodes; 13/18 includes the fork source station + guide nodes.
        """
        mapping = {}
        if self.layout_builder is None or getattr(self.layout_builder, "grid", None) is None:
            return mapping

        lb = self.layout_builder
        watch_codes = set(str(x) for x in getattr(self.oht_config, "hotspot_watch_cell_codes", {"9", "6", "13", "18"}))
        fork_codes = set(str(x) for x in getattr(self.oht_config, "hotspot_escape_fork_codes", {"13", "18"}))

        for r in range(lb.rows):
            for c in range(lb.cols):
                tok = self._cell_base_code_from_layout(r, c)
                if tok not in watch_codes:
                    continue

                # guide node inside the cell
                for node_name, node in self.nodes.items():
                    if getattr(node, "kind", None) == "guide" and getattr(node, "rc", None) == (r, c):
                        mapping[node_name] = tok

                # For fork cells, also include the actual branching station source in watch/penalty
                if tok in fork_codes and tok in lb.JUNCTION_CODES:
                    shared_c, str_other_c, arc_other_c, arc_cp_c, mode = lb.TILE_JUNCTION_DEF[tok]
                    if mode == "fork":
                        source = self._corner_node_from_layout_cell(r, c, shared_c)
                        if source is not None:
                            mapping[source] = tok

        return mapping

    # -----------------------------------------------------
    # Station-based ZCU auto builder
    # -----------------------------------------------------
    def _add_zcu_node(self, mapping, node_name, zone_name):
        if node_name is None:
            return
        if node_name not in self.nodes:
            return
        mapping.setdefault(node_name, set()).add(zone_name)

    def _trace_forward_until_station_runner(self, start_node_name):
        chain = []
        visited = set()
        cur = start_node_name
        while cur and cur not in visited and cur in self.nodes:
            visited.add(cur)
            chain.append(cur)
            node = self.nodes[cur]
            if len(chain) > 1 and node.kind == 'station':
                break
            nxts = list(self.adj.get(cur, []))
            if not nxts:
                break
            cur = nxts[0]
        return chain

    def _build_station_zcu_map(self):
        """
        Auto-generate the ZCU concept from SMAT2022 adapted to the salabim layout.
        - A curve/arc chain entering from station -> guide is grouped into one ZCU.
        - A merge station with indegree >= 2 is grouped into a separate ZCU.
        - Waiting occurs only at stations in OHTVehicle.move_one_hop.
        """
        mapping = {}

        # 1) Group the chain entering from station into guide/curve as one zone
        for u, nbrs in self.adj.items():
            u_node = self.nodes.get(u)
            if u_node is None or u_node.kind != 'station':
                continue
            for v in nbrs:
                v_node = self.nodes.get(v)
                if v_node is None or v_node.kind != 'guide':
                    continue
                chain = self._trace_forward_until_station_runner(v)
                if not chain:
                    continue
                end_station = chain[-1] if self.nodes.get(chain[-1]) and self.nodes[
                    chain[-1]].kind == 'station' else 'END'
                zone = f"ZCU_CURVE::{u}->{end_station}"
                for nd in chain:
                    self._add_zcu_node(mapping, nd, zone)

        # 2) merge station: group the section where multiple branches join into one station as one zone
        incoming = {nm: [] for nm in self.nodes.keys()}
        for u, nbrs in self.adj.items():
            for v in nbrs:
                if v in incoming:
                    incoming[v].append(u)

        for node_name, preds in incoming.items():
            node = self.nodes.get(node_name)
            if node is None or node.kind != 'station':
                continue
            if len(preds) < 2:
                continue
            zone = f"ZCU_MERGE::{node_name}"
            self._add_zcu_node(mapping, node_name, zone)
            for pred in preds:
                self._add_zcu_node(mapping, pred, zone)

        return mapping

    # -----------------------------------------------------
    # build
    # -----------------------------------------------------
    def build_environment(self):
        random.seed(self.oht_config.seed)
        sim.yieldless(False)
        self.env = sim.Environment(trace=False)

    def build_layout(self):
        self.layout_builder = OHTLayoutBuilder(
            csv_path=self.layout_csv_path,
            oht_config=self.oht_config
        )
        self.layout_builder.build()

        # Overlay machine positions from layout_Machine.csv
        self.layout_builder.load_machine_layout(self.machine_csv_path)

        self.nodes = self.layout_builder.nodes
        self.edges = self.layout_builder.edges
        self.adj = self.layout_builder.adj
        self.uv_to_eid = self.layout_builder.uv_to_eid
        self.node_last_pass_time = {nm: -1e18 for nm in self.nodes.keys()}
        self.route_node_pass_times = {nm: deque() for nm in self.nodes.keys()}

        # Initialize Hotspot escape rule shared state
        self.hotspot_escape_next_by_node = self._build_hotspot_escape_next_by_node()
        self.hotspot_watch_nodes = self._build_hotspot_watch_nodes()
        self.hotspot_node_to_cell_code = self._build_hotspot_node_to_cell_code()
        self.node_hotspot_wait_start = {}
        self.hotspot_active_until = {"until": -1e18}
        self.hotspot_node_block_until = {}
        self.hotspot_fork_pass_times = {nm: deque() for nm in self.hotspot_escape_next_by_node.keys()}
        self.hotspot_fork_block_until = {}

        # Initialize Station-based Preventive Zone Reservation shared state
        self.zcu_node_to_zone = self._build_station_zcu_map()
        self.zcu_occupied_by = {}
        if bool(getattr(self.oht_config, "debug_station_zcu", False)):
            zones = set()
            for val in self.zcu_node_to_zone.values():
                zones.update(val if isinstance(val, set) else {val})
            print(f"[station_zcu] nodes={len(self.zcu_node_to_zone)}, zones={len(zones)}")

        if bool(getattr(self.oht_config, "debug_hotspot_escape_rule", False)):
            print("[hotspot_escape] escape source -> next hop")
            for src, nxt in sorted(self.hotspot_escape_next_by_node.items()):
                print(f"  {src} -> {nxt}")
            print(f"[hotspot_escape] watch_nodes={len(self.hotspot_watch_nodes)}")
            print(f"[hotspot_escape] node_to_cell={len(self.hotspot_node_to_cell_code)}")

    # -----------------------------------------------------
    # Machine layout optimization
    # -----------------------------------------------------
    def _machine_flow_matrix(self, jssp_data, n_machines: int):
        """
        Build the consecutive machine-pair frequency F_ij from the JSSP route.
        A larger F_ij means more requests to move from machine i to machine j.
        """
        flow = [[0.0 for _ in range(n_machines)] for _ in range(n_machines)]
        if jssp_data is None:
            return flow

        for j, route in jssp_data.job_routes.items():
            qty = float(self.job_quantities.get(j, 1)) if hasattr(self, 'job_quantities') else 1.0
            ops = list(route.operations)
            for a, b in zip(ops[:-1], ops[1:]):
                if 0 <= a.machine_id < n_machines and 0 <= b.machine_id < n_machines:
                    flow[a.machine_id][b.machine_id] += qty
        return flow

    def _shortest_layout_lengths(self, machine_nodes):
        """
        Compute the shortest rail distance D_ij between machine nodes on the current layout_oht graph.
        Fall back to coordinate-based straight-line distance if there is no path in the directed graph.
        """
        n = len(machine_nodes)
        dist_matrix = [[0.0 for _ in range(n)] for _ in range(n)]

        graph = {}
        for eid, edge in self.edges.items():
            graph.setdefault(edge.u, []).append((edge.v, float(edge.length)))

        for src_idx, src in enumerate(machine_nodes):
            dist = {src: 0.0}
            pq = [(0.0, src)]
            while pq:
                d, u = heapq.heappop(pq)
                if d != dist.get(u, float('inf')):
                    continue
                for v, w in graph.get(u, []):
                    nd = d + w
                    if nd < dist.get(v, float('inf')):
                        dist[v] = nd
                        heapq.heappush(pq, (nd, v))

            src_node = self.nodes[src]
            for dst_idx, dst in enumerate(machine_nodes):
                val = dist.get(dst)
                if val is None:
                    dst_node = self.nodes[dst]
                    val = ((src_node.x - dst_node.x) ** 2 + (src_node.y - dst_node.y) ** 2) ** 0.5
                dist_matrix[src_idx][dst_idx] = float(val)
        return dist_matrix

    def _assignment_cost(self, pos_of_machine, flow, dist):
        """Σ F_ij × D(pos_i, pos_j)."""
        n = len(pos_of_machine)
        cost = 0.0
        for i in range(n):
            pi = pos_of_machine[i]
            row = flow[i]
            for j in range(n):
                fij = row[j]
                if fij:
                    cost += fij * dist[pi][pos_of_machine[j]]
        return cost

    def _optimize_machine_node_map(self, machine_nodes, jssp_data):
        """
        Keep the physical nodes of layout_machine.csv as they are,
        and only reassign which node each JSSP machine id sits on.

        Objective: minimize Σ F_ij × D_ij
          - F_ij: frequency of transitioning i→j in the job route
          - D_ij: shortest distance between two machine positions in the layout rail graph
        """
        # Fix CSV order: use the M_ numbers of layout_machine.csv as-is.
        # Do not use optimal machine reassignment.
        return None

        if not bool(getattr(self.jssp_cfg, 'optimize_machine_layout', False)):
            return None
        if jssp_data is None:
            return None

        n = min(jssp_data.n_machines, len(machine_nodes))
        if n <= 1:
            return None

        pos_nodes = list(machine_nodes[:n])
        flow = self._machine_flow_matrix(jssp_data, n)
        if not any(flow[i][j] for i in range(n) for j in range(n)):
            return None

        dist = self._shortest_layout_lengths(pos_nodes)

        # Initial placement: use the existing M_ number order of layout_machine.csv as-is
        pos_of_machine = list(range(n))
        best_cost = self._assignment_cost(pos_of_machine, flow, dist)
        current_cost = best_cost

        rnd = random.Random(int(getattr(self.jssp_cfg, 'machine_layout_opt_seed', 42)))
        iterations = int(getattr(self.jssp_cfg, 'machine_layout_opt_iterations', 5000))

        for _ in range(max(0, iterations)):
            a, b = rnd.sample(range(n), 2)
            pos_of_machine[a], pos_of_machine[b] = pos_of_machine[b], pos_of_machine[a]
            new_cost = self._assignment_cost(pos_of_machine, flow, dist)

            if new_cost <= current_cost:
                current_cost = new_cost
                if new_cost < best_cost:
                    best_cost = new_cost
            else:
                pos_of_machine[a], pos_of_machine[b] = pos_of_machine[b], pos_of_machine[a]

        optimized_map = {m_id: pos_nodes[pos_of_machine[m_id]] for m_id in range(n)}

        # Also align the machine_no in the on-screen display and CSV logs with the optimal placement result.
        for node_name in pos_nodes:
            self.nodes[node_name].machine_no = None
        for m_id, node_name in optimized_map.items():
            self.nodes[node_name].machine_no = m_id + 1

        improvement = 0.0 if current_cost <= 0 else (1.0 - best_cost / max(1e-9,
                                                                           self._assignment_cost(list(range(n)), flow,
                                                                                                 dist))) * 100.0
        print(
            f"[machine_layout_opt] applied flow-weighted distance optimization "
            f"cost {self._assignment_cost(list(range(n)), flow, dist):.2f} -> {best_cost:.2f} "
            f"({improvement:.1f}% reduction)"
        )
        return optimized_map

    def build_machines(self):
        """
        Only on nodes with has_machine=True (the M_N tokens of layout_Machine.csv)
        create a MachineStation.
        Ordinary rail station nodes are used only as OHT passages and have no machine.
        """
        # gantt (GanttSchedule) supplies routing/timing. Targeting is fixed by gantt.
        schedule = getattr(self.machine_decision, "schedule", None)

        # Collect has_machine=True nodes (M_N of layout_machine.csv), ascending by machine_no
        machine_nodes = sorted(
            [nm for nm, nd in self.nodes.items()
             if nd.kind == "station" and getattr(nd, 'has_machine', False)],
            key=lambda nm: getattr(self.nodes[nm], 'machine_no', 9999)
        )
        if not machine_nodes:
            raise RuntimeError(
                "Could not find M_N nodes in layout_machine.csv (no has_machine=True)")

        import random as _random
        seed0 = int(getattr(self.oht_config, 'seed', 42))

        # machine_node_map = {M-name(str): node_name}.  M{machine_no} ↔ gantt machine name
        # (consistency of layout M_ number ↔ gantt M number is pre-verified: close within the same bay)
        for node_name in machine_nodes:
            node = self.nodes[node_name]
            m_no = int(getattr(node, 'machine_no', 0))
            m_name = f"M{m_no}"
            self.machine_node_map[m_name] = node_name

            self.machines[node_name] = MachineStation(
                name=f"M_{node_name}",
                node=node,
                machine_config=self.machine_config,
                schedule=schedule,                       # gantt routing/timing
                machine_name=m_name,                     # the gantt machine name of this station
                machine_node_map=self.machine_node_map,  # {M-name: node} (completed at runtime)
                rng=_random.Random(seed0 * 1000 + m_no), # realized proc sample reproducibility
                machine_live_state=self.machine_live_state,
            )

        self._machine_nodes_sorted = sorted(
            self.machines.keys(),
            key=lambda nm: getattr(self.nodes[nm], 'machine_no', 9999)
        )

        print(f"[build_machines] created {len(self.machines)} MachineStations "
              f"(gantt mode, machine_node_map: M1..M{len(self.machine_node_map)})")

        # Disturbance robustness: machine breakdown driver (when enabled). Same seed → same breakdown sequence across policies
        if getattr(self.oht_config, "enable_machine_breakdowns", False):
            mtbf = float(getattr(self.oht_config, "machine_mtbf_sec", 3600.0))
            mttr = float(getattr(self.oht_config, "machine_mttr_sec", 600.0))
            bseed = int(getattr(self.oht_config, "breakdown_seed", 123))
            self._breakdowns = [
                MachineBreakdown(name=f"BD_{nm}", machine=self.machines[nm],
                                 mtbf=mtbf, mttr=mttr,
                                 rng=_random.Random(bseed * 100000 + i))
                for i, nm in enumerate(self._machine_nodes_sorted)
            ]
            print(f"[breakdowns] {len(self._breakdowns)} machines, "
                  f"MTBF={mtbf:.0f}s MTTR={mttr:.0f}s seed={bseed}")

    def build_ohts(self):
        # Choose initial positions among station nodes (minimize overlap — Bug 5 fix)
        candidate_spawn = [nm for nm, node in self.nodes.items()
                           if node.kind == "station"]

        if not candidate_spawn:
            raise RuntimeError("No station nodes found for OHT spawn.")

        spawn_pool = candidate_spawn.copy()
        random.shuffle(spawn_pool)
        occupied_starts: set = set()

        for i in range(self.oht_config.n_oht):
            preferred = spawn_pool[i % len(spawn_pool)]
            start_node = preferred

            # If already occupied, search for an adjacent empty station node
            if preferred in occupied_starts:
                adj_free = [
                    n for n in self.adj.get(preferred, [])
                    if n not in occupied_starts
                       and self.nodes.get(n)
                       and self.nodes[n].kind == 'station'
                ]
                if adj_free:
                    start_node = random.choice(adj_free)

            occupied_starts.add(start_node)

            veh = OHTVehicle(
                name=f"OHT_{i + 1:02d}",
                vid=i + 1,
                start_node=start_node,
                nodes=self.nodes,
                edges=self.edges,
                uv_to_eid=self.uv_to_eid,
                adj=self.adj,
                machines=self.machines,
                oht_config=self.oht_config,
                idle_behavior=self._make_idle_behavior(),
                fleet=self.vehicles,
                node_last_pass_time=self.node_last_pass_time,
                hotspot_escape_next_by_node=self.hotspot_escape_next_by_node,
                hotspot_watch_nodes=self.hotspot_watch_nodes,
                node_hotspot_wait_start=self.node_hotspot_wait_start,
                hotspot_active_until=self.hotspot_active_until,
                hotspot_node_to_cell_code=self.hotspot_node_to_cell_code,
                hotspot_node_block_until=self.hotspot_node_block_until,
                hotspot_fork_pass_times=self.hotspot_fork_pass_times,
                hotspot_fork_block_until=self.hotspot_fork_block_until,
                zcu_node_to_zone=self.zcu_node_to_zone,
                zcu_occupied_by=self.zcu_occupied_by,
                route_node_pass_times=self.route_node_pass_times,
                transport_event_log=self.transport_event_log,
                transport_live_state=self.transport_live_state,
                oht_event_trace_state=self.oht_event_trace_state,

            )
            self.vehicles.append(veh)
            print(f"  OHT-{i + 1} initial position: {start_node}")

    def _make_idle_behavior(self):
        # idle OHTs always random-roam (the center-gathering CENTER_RAIL logic is dropped).
        return RandomRoam(nodes=self.nodes)

    def _make_oht_path_algo(self):
        algo = str(getattr(self.oht_config, 'oht_path_algorithm', 'ASTAR')).upper().replace('-', '_').replace(' ', '_')
        if algo in {'BFS', 'BFS_PATH'}:
            return BFSPath()
        # Inject edges/uv_to_eid for travel_time-based path search → consistent with actual OHT movement.
        # min_time_per_unit = 1 / straight_speed (fastest speed → admissible heuristic lower bound)
        straight_speed = float(getattr(self.oht_config, 'straight_speed', 5.0) or 5.0)
        return AStarPath(
            edges=self.edges,
            uv_to_eid=self.uv_to_eid,
            min_time_per_unit=1.0 / max(straight_speed, 1e-6),
        )

    def build_logic(self):
        self._job_source = JobSource(
            name="JobSource",
            machines=self.machines,
            schedule=self.machine_decision.schedule,
            machine_node_map=self.machine_node_map,
            jssp_cfg=self.jssp_cfg,
        )
        # CONWIP: connect every machine's fab-out (completion) signal to JobSource
        for _ms in self.machines.values():
            _ms.on_fabout = self._job_source.notify_fabout

        self._oht_dm = OHTDecisionMaker(
            name="OHTDecisionMaker",
            vehicles=self.vehicles,
            machines=self.machines,  # needed for event-based dispatch
            adj=self.adj,
            nodes=self.nodes,
            jssp_cfg=self.jssp_cfg,
            machine_node_map=self.machine_node_map,
            dispatch_dt=self.oht_config.dispatch_dt,
            path_algo=self._make_oht_path_algo(),
            dispatch_mode=getattr(self.oht_config, 'oht_dispatch_mode', 'HUNGARIAN'),
            swap_penalty=getattr(self.oht_config, 'oht_swap_penalty', 30.0),
            load_sec=float(getattr(self.oht_config, 'load_time_min', 10.0)),
            unload_sec=float(getattr(self.oht_config, 'unload_time_min', 10.0)),
            plan_tasks_csv=getattr(self.oht_config, 'oht_savd_tasks_csv', ''),
            jssp_data=None,
            schedule=self.machine_decision.schedule,
        )

        # Event-based dispatch wiring: on a new pickup (loading into a machine out_buffer) / OHT idle transition,
        # set a back-reference to wake the dispatcher (removes polling).
        for _ms in self.machines.values():
            _ms._dispatcher = self._oht_dm
        for _v in self.vehicles:
            _v._dispatcher = self._oht_dm

    # -----------------------------------------------------
    # animation
    # -----------------------------------------------------
    def _bounds(self):
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        if not xs or not ys:
            self._panel_geom = {}
            return (0, 0, 100, 100)

        m = self.oht_config.margin
        x0 = min(xs) - m
        x1 = max(xs) + m
        y0 = min(ys) - m
        y1 = max(ys) + m

        # Dashboard area removed
        self._panel_geom = {}
        return x0, y0, x1, y1

    def _animation_window_size(self, x0, y0, x1, y1):
        span_x = max(1.0, x1 - x0)
        span_y = max(1.0, y1 - y0)

        base_height = 1050

        aspect = span_x / span_y
        width = int(round(base_height * aspect))
        width = max(1100, min(1800, width))
        height = base_height

        self._window_width = width
        self._window_height = height
        return width, height

    def setup_animation(self):
        x0, y0, x1, y1 = self._bounds()
        win_w, win_h = self._animation_window_size(x0, y0, x1, y1)

        self.env.animation_parameters(
            modelname="",
            background_color=self.oht_config.background_color,
            x0=x0,
            y0=y0,
            x1=x1,
            width=win_w,
            height=win_h,
            show_time=False,
            speed=self.oht_config.sim_speed,
            fps=int(getattr(self.oht_config, 'anim_fps', 10)),   # default 30→10: lower redraw load↓
        )

        sim.AnimateText(
            text='OHT Semiconductor Simulation',
            x=x0 + 5,
            y=y1 - 5,
            fontsize=max(8, self.oht_config.time_font_size - 1),
            textcolor='#d9e6f2',
            screen_coordinates=False,
        )
        sim.AnimateText(
            text=lambda: f'time={self.env.now():.2f}',
            x=x0 + 5,
            y=y1 - 15,
            fontsize=max(7, self.oht_config.time_font_size - 1),
            textcolor='white',
            screen_coordinates=False,
        )

        # ====================================================================================================================================
        # rail
        for _, e in self.edges.items():
            a = self.nodes[e.u]
            b = self.nodes[e.v]
            sim.AnimateLine(
                spec=(a.x, a.y, b.x, b.y),
                linecolor=self.oht_config.rail_color,
                linewidth=self.oht_config.rail_line_width,
                screen_coordinates=False,
            )

        # rail nodes — draw station nodes as small circles on the rail (static color)
        # skip in light mode (saves 1142 objects). Topology is already visible from the rail lines.
        if not bool(getattr(self.oht_config, "anim_light", False)):
            for node_name, node in self.nodes.items():
                if node.kind == "station":
                    sim.AnimateCircle(
                        radius=self.oht_config.node_radius,
                        x=node.x,
                        y=node.y,
                        fillcolor=self.oht_config.station_node_color,
                        linecolor="black",
                        screen_coordinates=False,
                    )

        # ── machine visualization: IN (top) / machine box / OUT (bottom) layout ───────────
        cs = self.oht_config.cell_size
        # machine box size (keep original size)
        mw = cs * 0.85  # width
        mh = cs * 0.55  # height
        # buffer slot size
        cap = self.machine_config.input_buffer_cap
        sw = mw / (cap + 1.2)  # slot width
        sh = cs * 0.18  # slot height
        gap = sw * 0.18  # slot spacing
        buf_w = cap * sw + (cap - 1) * gap  # total buffer width
        fs_m = self.oht_config.node_font_size  # machine-number font

        ##################################################################################

        _light = bool(getattr(self.oht_config, "anim_light", False))   # whether to skip buffer slots
        for node_name, ms in self.machines.items():
            node = self.nodes[node_name]
            if not getattr(node, 'has_machine', False):
                continue

            dx, dy = self.layout_builder.get_machine_draw_offset(node_name)
            cx = node.x + dx
            cy = node.y + dy

            bx1, by1 = cx - mw / 2, cy - mh / 2
            bx2, by2 = cx + mw / 2, cy + mh / 2

            # IN buffer position (used as reference for the machine text y)
            in_y1 = by2 + cs * 0.04
            in_y2 = in_y1 + sh

            # machine text: centered above the in buffer
            ex = cx - cs * 0.3
            ey = in_y2 + cs * 0.12

            # ── connector line: rail node ↔ machine box ────────────────────────
            sim.AnimateLine(
                spec=(node.x, node.y, cx, cy),
                linecolor="#555555", linewidth=0.5,
                screen_coordinates=False,
            )

            # ── machine body box ───────────────────────────────────────
            sim.AnimateRectangle(
                spec=(bx1, by1, bx2, by2),
                fillcolor=lambda m=ms: "#e07000" if m.is_processing() else "#1a2a3a",
                linecolor="white", linewidth=0.5,
                screen_coordinates=False,
            )
            # machine number (box center)
            sim.AnimateText(
                text=f"M{node.machine_no}",
                x=ex, y=ey,
                fontsize=fs_m,
                textcolor="yellow",

                screen_coordinates=False,
            )

            # ── IN buffer: above the machine box ────────────────────────────────
            in_y1 = by2 + cs * 0.04  # slot bottom
            in_y2 = in_y1 + sh  # slot top
            in_x0 = cx - buf_w / 2  # slot start x

            # IN label
            # sim.AnimateText(
            #     text="IN",
            #     x=cx, y=in_y2 + cs * 0.03,
            #     fontsize=fs_b, textcolor="cyan",
            #     screen_coordinates=False,
            # )
            # IN slots (skip in light mode)
            for i in (() if _light else range(cap)):
                sx1 = in_x0 + i * (sw + gap)
                sim.AnimateRectangle(
                    spec=(sx1, in_y1, sx1 + sw, in_y2),
                    fillcolor=lambda m=ms, idx=i: (
                        "cyan" if m.visual_in_count() > idx else "#0a1a2a"),
                    linecolor="#008888", linewidth=0.4,
                    screen_coordinates=False,
                )

            # ── OUT buffer: below the machine box ─────────────────────────────
            out_y2 = by1 - cs * 0.04  # slot top
            out_y1 = out_y2 - sh  # slot bottom
            out_x0 = cx - buf_w / 2

            # # OUT label
            # sim.AnimateText(
            #     text="OUT",
            #     x=cx, y=out_y1 - cs * 0.5,
            #     fontsize=fs_b, textcolor="lime",
            #     screen_coordinates=False,
            # )
            # OUT slots (skip in light mode)
            for i in (() if _light else range(cap)):
                sx1 = out_x0 + i * (sw + gap)
                sim.AnimateRectangle(
                    spec=(sx1, out_y1, sx1 + sw, out_y2),
                    fillcolor=lambda m=ms, idx=i: (
                        "#00cc44" if m.visual_out_count() > idx else "#0a1a2a"),
                    linecolor="#006622", linewidth=0.4,
                    screen_coordinates=False,
                )

        # OHT — triangle + number label + Breaked indicator
        for veh in self.vehicles:
            sim.AnimatePolygon(
                spec=lambda v=veh: v.triangle_spec(),
                x=lambda v=veh: v.visual_x(),
                y=lambda v=veh: v.visual_y(),
                fillcolor=lambda v=veh: v.body_fill_color(),
                linecolor=lambda v=veh: v.outline_color(),
                linewidth=self.oht_config.oht_line_width,
                screen_coordinates=False,
            )

            # OHT number label (small, right above the triangle)
            sim.AnimateText(
                text=f"{veh.vid}",
                x=lambda v=veh: v.visual_x(),
                y=lambda v=veh: v.visual_y() + self.oht_config.oht_label_offset,
                fontsize=self.oht_config.oht_font_size,
                textcolor=self.oht_config.oht_label_color,

                screen_coordinates=False,
            )

            # Breaked indicator (red dot)
            sim.AnimateCircle(
                radius=self.oht_config.breaked_dot_radius,
                x=lambda v=veh: v.visual_x(),
                y=lambda v=veh: v.visual_y(),
                fillcolor="red",
                linecolor="black",
                visible=lambda v=veh: v.state_name == "Breaked",
                screen_coordinates=False,
            )

    # -----------------------------------------------------
    # run
    # -----------------------------------------------------
    def run(self, enable_animation=None):
        """
        enable_animation switch
        - True  : as before, open a salabim window and run the visualization
        - False : run only the event simulation without a window and save CSV/KPI
        - None  : use the OHTConfig.enable_animation value
        """
        if enable_animation is None:
            enable_animation = bool(getattr(self.oht_config, 'enable_animation', True))

        # UI live trace: three CSVs the monitoring interface tails while the run
        # is in progress. Off by default -- a headless experiment writes nothing
        # extra and behaves exactly as before.
        if bool(getattr(self.oht_config, 'enable_live_trace', False)):
            import os as _os
            _live_dir = _os.path.dirname(
                _os.path.abspath(self.jssp_cfg.simulation_log_gantt_csv)) or "."
            _os.makedirs(_live_dir, exist_ok=True)
            _live = create_live_states(_live_dir)
            self.transport_live_state = _live["transport"]
            self.machine_live_state = _live["machine"]
            self.oht_event_trace_state = _live["oht_event"]

        total_start = time.perf_counter()

        self.build_environment()
        self.build_layout()
        self.build_machines()
        # Run transport-aware scheduling once the machine node mapping is determined
        self.machine_decision.run_scheduling(
            transport_func=self._machine_transport_time
        )
        self._prepare_targets()
        self.build_ohts()
        self.build_logic()

        # SAVD: keep the idle-OHT distribution matched to the plan's demand density
        if getattr(self.oht_config, 'oht_savd_positioning', False):
            _SavdPositioningMonitor(runner=self, env=self.env)

        mode_name = 'ANIMATION' if enable_animation else 'HEADLESS'
        print('=' * 56)
        print(f'[RUN MODE] {mode_name}')
        print(f'[PATH ALGO] {getattr(self.oht_config, "oht_path_algorithm", "ASTAR")}')
        print(f'[HORIZON ] {self.oht_config.sim_horizon}s')
        print(f'[SEED    ] {getattr(self.oht_config, "seed", 42)}  (to reproduce, use --seed {getattr(self.oht_config, "seed", 42)})')
        print('=' * 56)

        wall_start = time.perf_counter()

        if enable_animation:
            self.setup_animation()
            self.env.animate(True)
        else:
            # Key: skipping setup_animation() and Animate object creation
            # runs only the pure DES events fast, without a Tk/salabim window.
            self.env.animate(False)

        self._run_event_loop()
        wall_elapsed = time.perf_counter() - wall_start

        if self._oht_dm is not None:
            self._oht_dm.close_log()
        self._save_machine_sim_log()
        kpi = self._evaluate(wall_elapsed_s=wall_elapsed, run_mode=mode_name)

        total_elapsed = time.perf_counter() - total_start
        print(f"Total elapsed (build+run+save+eval): {total_elapsed:,.2f}s "
              f"({total_elapsed / 60:.2f}min)  |  sim event loop: {wall_elapsed:,.2f}s")
        return kpi

    # ── gridlock watchdog ────────────────────────────────────────────
    def _progress_signature(self):
        """A cheap fingerprint of "did anything at all happen".

        Combines lot fab-outs, machine operation completions and the position of
        every vehicle. A merge-yield deadlock freezes all three at once, so an
        unchanged signature over a long stretch of sim time means the system is
        genuinely stuck rather than merely slow.
        """
        js = getattr(self, "_job_source", None)
        return (
            int(getattr(js, "completed", 0)) if js is not None else 0,
            self._completed_ops_count(),
            tuple(v.pos_node for v in self.vehicles),
        )

    def _run_event_loop(self):
        """Run the DES loop, stopping early if the system gridlocks.

        Without the watchdog this is a single ``env.run(till=horizon)``. With it,
        the horizon is walked in ``gridlock_check_dt_s`` slices and the progress
        signature is compared between slices; ``gridlock_timeout_s`` of sim time
        with no change ends the run.

        Sets ``_gridlock_detected`` / ``_gridlock_time_s`` (when progress actually
        stopped, i.e. the freeze onset -- not when the watchdog noticed) and
        ``_sim_end_s`` (where the loop actually stopped).
        """
        horizon = float(self.oht_config.sim_horizon)
        self._gridlock_detected = False
        self._gridlock_time_s = None

        timeout = float(getattr(self.oht_config, "gridlock_timeout_s", 0.0) or 0.0)
        if timeout <= 0.0:
            self.env.run(till=horizon)
            self._sim_end_s = float(self.env.now())
            return

        dt = float(getattr(self.oht_config, "gridlock_check_dt_s", 60.0) or 60.0)
        last_sig = self._progress_signature()
        last_change = float(self.env.now())

        while True:
            now = float(self.env.now())
            if now >= horizon:
                break
            self.env.run(till=min(now + dt, horizon))
            now = float(self.env.now())

            sig = self._progress_signature()
            if sig != last_sig:
                last_sig, last_change = sig, now
            elif now - last_change >= timeout:
                self._gridlock_detected = True
                self._gridlock_time_s = last_change
                print(f"[GRIDLOCK] frozen since t={last_change:,.0f}s "
                      f"({last_change / 3600.0:.2f}h); no progress for "
                      f"{timeout:,.0f}s -> stopping at t={now:,.0f}s "
                      f"(saved {(horizon - now) / 3600.0:.2f}h of sim)", flush=True)
                break

        self._sim_end_s = float(self.env.now())

    def _save_machine_sim_log(self):
        import csv as _csv
        import os as _os

        # ── save the raw sim log (CSV saved 1-based for humans) ───────────────
        rows = []
        for st in self.machines.values():
            rows.extend(getattr(st, '_sim_log', []))
        if not rows:
            return
        rows.sort(key=lambda r: r['sim_time'])

        raw_rows = []
        for row in rows:
            rr = dict(row)
            rr['job_instance_id'] = int(rr.get('job_id', 0))
            if rr.get('job_type_id') not in (None, ''):
                rr['job_id'] = int(rr['job_type_id']) + 1
                rr['job_type_id'] = int(rr['job_type_id']) + 1
            # jssp_mach_id in _sim_log is already stored 1-based, so use it as-is
            # machine_no / op_index are already human-friendly values (1-based)
            raw_rows.append(rr)

        raw_path = self.jssp_cfg.log_machine_csv.replace('.csv', '_sim.csv')
        _os.makedirs(_os.path.dirname(_os.path.abspath(raw_path)) or '.', exist_ok=True)
        raw_fields = [
            'sim_time', 'node_name', 'machine_no', 'machine_name', 'jssp_mach_id',
            'job_id', 'job_instance_id', 'lot_id', 'step_no', 'job_type_id', 'op_index',
            'product_type', 'start_time', 'end_time', 'process_time',
            'planned_ready_time', 'realized_ready_time', 'ready_deviation',
        ]
        with open(raw_path, 'w', newline='', encoding='utf-8') as f:
            w = _csv.DictWriter(f, fieldnames=raw_fields)
            w.writeheader()
            w.writerows(raw_rows)
        print(f"  [SIM LOG] machine_sim        → {raw_path}")

        # ── save the OHT actual transport event log ───────────────────────
        transport_rows = sorted(
            [dict(r) for r in getattr(self, "transport_event_log", [])],
            key=lambda r: (float(r.get("dropoff_time", 0.0)), int(r.get("event_id", 0)))
        )
        transport_path = _os.path.join(
            _os.path.dirname(_os.path.abspath(self.jssp_cfg.simulation_log_gantt_csv)),
            'transport.csv'
        )
        transport_fields = [
            'event_id', 'oht_id', 'job_id', 'job_instance_id', 'op_index',
            'from_machine', 'to_machine', 'from_node', 'to_node',
            'dispatch_time', 'prev_machine_end_time', 'source_arrival_time',
            'pickup_wait_start_time', 'pickup_time',
            'load_start_time', 'load_end_time',
            'loaded_travel_start_time', 'dest_arrival_time',
            'unload_start_time', 'unload_end_time', 'dropoff_time',
            'source_wait_time', 'empty_to_source_time', 'pickup_wait_time',
            'loading_time', 'loaded_travel_time', 'unloading_time', 'drop_wait_time',
            'pure_oht_transport_time', 'actual_transport_time',
            'free_flow_shortest_time', 'free_flow_actual_path_time',
            'blocking_loaded_time', 'blocking_ratio', 'detour_ratio',
            'lot_id', 'step_no', 'planned_travel', 'transport_deviation',
        ]
        with open(transport_path, 'w', newline='', encoding='utf-8') as f:
            w = _csv.DictWriter(f, fieldnames=transport_fields)
            w.writeheader()
            for tr in transport_rows:
                w.writerow({k: tr.get(k, '') for k in transport_fields})
        print(f"  [SIM LOG] transport          → {transport_path}")

        # (gantt mode) removed the ta01 result_gantt reconstruction.
        # Measured results are in machine_sim.csv (realized+ready_deviation) + transport.csv (transport_deviation).


    def _evaluate(self, wall_elapsed_s=None, run_mode=None):
        """gantt-mode KPI: 24h throughput (fab-out) + transport delay + step delay + utilization."""
        import csv as _csv
        import os as _os
        import statistics as _st

        out_dir = _os.path.dirname(_os.path.abspath(self.jssp_cfg.simulation_log_gantt_csv)) or "."
        horizon = float(getattr(self.oht_config, "sim_horizon", 0.0) or 0.0)

        # ── CONWIP throughput (fab-out) ───────────────────────────────
        js = getattr(self, "_job_source", None)
        completed_jobs = int(getattr(js, "completed", 0)) if js is not None else 0
        injected_jobs = int(getattr(js, "job_seq", 0)) if js is not None else 0
        wip_cap = int(getattr(js, "wip_cap", 0)) if js is not None else 0
        thr_per_24h = completed_jobs * (86400.0 / horizon) if horizon > 0 else 0.0
        # Actual metrics: throughput@24h (lots completed by 24h) + makespan (last completion time)
        ct_list = (getattr(js, "completion_times", []) or []) if js is not None else []
        cyc_list = (getattr(js, "cycle_times", []) or []) if js is not None else []
        comp_times = sorted(ct_list)
        comp_24 = [t for t in comp_times if t <= 86400.0]
        throughput_24h = len(comp_24)
        makespan_24h = comp_24[-1] if comp_24 else 0.0       # last completion within 24h (main metric)
        makespan_realized = comp_times[-1] if comp_times else 0.0  # overall last (note: includes the drain tail)
        # cycle time = fab-out − release. Based on lots completed within 24h (matches the throughput window)
        cyc_24 = [c for t, c in zip(ct_list, cyc_list) if t <= 86400.0]
        mean_cycle_24h = _st.mean(cyc_24) if cyc_24 else 0.0
        median_cycle_24h = _st.median(cyc_24) if cyc_24 else 0.0
        # C_max KPI: inflation of realized makespan (full completion) vs plan makespan (all lots)
        cmax_plan = float(getattr(getattr(self, "machine_decision", None), "makespan", 0.0) or 0.0)
        cmax_ratio = (makespan_realized / cmax_plan) if cmax_plan > 0 else 0.0
        cmax_gap_h = (makespan_realized - cmax_plan) / 3600.0
        # ── makespan (attainment): convert the realized shortfall vs plan's 24h completion count into plan time ──
        #   makespan = 24h + (24h − plan_time(N_real)),  N_real = throughput@24h
        #   plan_time(k) = completion time of the k-th lot in the plan (per-lot max planned_end, sorted)
        _sched = getattr(getattr(self, "machine_decision", None), "schedule", None)
        plan_fins = []
        if _sched is not None:
            for _r in _sched.routes.values():
                if getattr(_r, "ops", None):
                    plan_fins.append(max(op.planned_end for op in _r.ops))
            plan_fins.sort()
        plan_done_24h = sum(1 for t in plan_fins if t <= 86400.0)
        if plan_fins and throughput_24h >= 1:
            _pt = plan_fins[min(throughput_24h, len(plan_fins)) - 1]
            makespan_attain_s = 2 * 86400.0 - _pt
        else:
            makespan_attain_s = 2 * 86400.0     # 0 completed → max penalty (48h)
        makespan_attain_h = makespan_attain_s / 3600.0
        # makespan attainment rate (%) = (48h − attain_h)/24h×100 = plan_time(N_real)/24h.
        #   Intuition: how many hours of the planned schedule the actual 24h output corresponds to. 100% = on par with plan, higher is better.
        makespan_attain_rate = (2 * 86400.0 - makespan_attain_s) / 86400.0 * 100.0

        # ── transport.csv statistics (pure transport delay label) ─────────────
        tpath = _os.path.join(out_dir, "transport.csv")
        n_tr = 0
        mean_dev = med_dev = mean_sw = mean_e2s = mean_lt = mean_detour = 0.0
        sw_p90 = sw_p95 = sw_max = sw_std = 0.0
        if _os.path.exists(tpath):
            with open(tpath, newline="", encoding="utf-8") as f:
                trs = list(_csv.DictReader(f))
            n_tr = len(trs)
            def col(name):
                out = []
                for r in trs:
                    try: out.append(float(r.get(name, "") or 0.0))
                    except: pass
                return out
            devs = col("transport_deviation")
            if devs:
                mean_dev = _st.mean(devs); med_dev = _st.median(devs)
            sw = col("source_wait_time");      mean_sw = _st.mean(sw) if sw else 0.0
            # tail metric: SAVD targets the worst case (tail), not the mean.
            def _pctl(xs, q):
                if not xs:
                    return 0.0
                s = sorted(xs)
                return s[min(len(s) - 1, int(q * len(s)))]
            sw_p90 = _pctl(sw, 0.90); sw_p95 = _pctl(sw, 0.95)
            sw_max = max(sw) if sw else 0.0
            sw_std = _st.pstdev(sw) if len(sw) > 1 else 0.0
            e2s = col("empty_to_source_time");  mean_e2s = _st.mean(e2s) if e2s else 0.0
            lt = col("loaded_travel_time");     mean_lt = _st.mean(lt) if lt else 0.0
            dr = col("detour_ratio");           mean_detour = _st.mean(dr) if dr else 0.0

        # ── machine_sim.csv statistics (step delay + utilization) ──────────────────
        mpath = self.jssp_cfg.log_machine_csv.replace(".csv", "_sim.csv")
        completed_ops = 0; mean_ready_dev = 0.0; total_proc = 0.0
        if _os.path.exists(mpath):
            with open(mpath, newline="", encoding="utf-8") as f:
                mrows = list(_csv.DictReader(f))
            completed_ops = len(mrows)
            sdv = []
            for r in mrows:
                try:
                    sdv.append(float(r.get("ready_deviation", "") or 0.0))
                    total_proc += float(r.get("process_time", "") or 0.0)
                except: pass
            if sdv: mean_ready_dev = _st.mean(sdv)
        n_machines = len(self.machines)
        utilization = (total_proc / (n_machines * horizon) * 100.0
                       if n_machines > 0 and horizon > 0 else 0.0)

        # ── OHT dispatch count (log_oht.csv) ─────────────────────────────
        dispatches = 0
        log = self.jssp_cfg.log_oht_csv
        if _os.path.exists(log):
            with open(log, newline="", encoding="utf-8") as f:
                for row in _csv.DictReader(f):
                    if row.get("event") in ("dispatch", "reassign"):
                        dispatches += 1

        # ── save kpi.csv ──────────────────────────────────────────────
        close_live_states(self.transport_live_state, self.machine_live_state,
                          self.oht_event_trace_state)

        kpi_path = _os.path.join(out_dir, "kpi.csv")
        kpi = {
            "wip_cap": wip_cap,
            "injected_jobs": injected_jobs,
            "completed_jobs": completed_jobs,
            # ── gridlock watchdog (see _run_event_loop) ──────────────
            # `gridlock_time_h` is the freeze ONSET (last moment anything moved),
            # not when the watchdog noticed it. `sim_end_h` is where the event
            # loop actually stopped; every 24 h KPI below is still normalized by
            # the configured horizon, so an early stop does not inflate them.
            "gridlock_detected": int(bool(getattr(self, "_gridlock_detected", False))),
            "gridlock_time_s": (round(self._gridlock_time_s, 1)
                                if getattr(self, "_gridlock_time_s", None) is not None else ""),
            "gridlock_time_h": (round(self._gridlock_time_s / 3600.0, 3)
                                if getattr(self, "_gridlock_time_s", None) is not None else ""),
            "sim_end_h": round(float(getattr(self, "_sim_end_s", horizon)) / 3600.0, 3),
            "throughput_24h": throughput_24h,                       # actual lots completed by 24h
            "makespan_24h_s": round(makespan_24h, 1),               # last completion within 24h (main metric)
            "makespan_24h_h": round(makespan_24h / 3600.0, 3),
            "makespan_realized_s": round(makespan_realized, 1),     # overall last (note: drain tail)
            "makespan_realized_h": round(makespan_realized / 3600.0, 3),
            "mean_cycle_time_24h_s": round(mean_cycle_24h, 1),      # avg cycle of lots completed within 24h (completion−release)
            "mean_cycle_time_24h_h": round(mean_cycle_24h / 3600.0, 3),
            "median_cycle_time_24h_s": round(median_cycle_24h, 1),
            "cmax_plan_h": round(cmax_plan / 3600.0, 3),           # plan makespan(reference)
            "cmax_ratio": round(cmax_ratio, 4),                     # realized/plan (>1 = behind)
            "cmax_gap_h": round(cmax_gap_h, 3),                     # realized − plan (h)
            "plan_done_24h": plan_done_24h,                         # plan's completions by 24h (reference count)
            "makespan_attain_h": round(makespan_attain_h, 3),      # headline: 24 h shortfall on the plan time axis
            "makespan_attain_rate_pct": round(makespan_attain_rate, 2),  # attainment rate (%), higher is better
            "makespan_attain_s": round(makespan_attain_s, 1),
            "throughput_per_24h": round(thr_per_24h, 1),
            "completed_ops": completed_ops,
            "machine_utilization_pct": round(utilization, 2),
            "mean_ready_deviation_s": round(mean_ready_dev, 2),
            "n_transports": n_tr,
            "mean_transport_deviation_s": round(mean_dev, 2),
            "median_transport_deviation_s": round(med_dev, 2),
            "mean_source_wait_s": round(mean_sw, 2),
            "source_wait_p90_s": round(sw_p90, 2),                  # tail: what SAVD targets
            "source_wait_p95_s": round(sw_p95, 2),
            "source_wait_max_s": round(sw_max, 2),
            "source_wait_std_s": round(sw_std, 2),
            "mean_empty_to_source_s": round(mean_e2s, 2),
            "mean_loaded_travel_s": round(mean_lt, 2),
            "mean_detour_ratio": round(mean_detour, 3),
            "oht_dispatches": dispatches,
            "n_oht": int(getattr(self.oht_config, "n_oht", 0)),
            "horizon_s": horizon,
            "wall_elapsed_s": round(float(wall_elapsed_s or 0.0), 3),
            "run_mode": run_mode or "",
            "path_algorithm": getattr(self.oht_config, "oht_path_algorithm", ""),
            "dispatch_mode": getattr(self.oht_config, "oht_dispatch_mode", ""),
        }
        with open(kpi_path, "w", newline="", encoding="utf-8") as f:
            w = _csv.writer(f)
            w.writerow(list(kpi.keys())); w.writerow(list(kpi.values()))

        print("=" * 56)
        print("[KPI] (gantt CONWIP)")
        print(f"  WIP cap / injected / completed : {wip_cap} / {injected_jobs} / {completed_jobs}")
        print(f"  Throughput @24h (measured)     : {throughput_24h} lots")
        print(f"  Makespan @24h (last within 24h): {makespan_24h:,.0f}s ({makespan_24h/3600:.2f}h)")
        print(f"  Makespan overall (=realized C_max): {makespan_realized:,.0f}s ({makespan_realized/3600:.2f}h)")
        print(f"  C_max plan / ratio / gap       : {cmax_plan/3600:.2f}h / {cmax_ratio:.3f}x / {cmax_gap_h:+.2f}h")
        print(f"  Makespan(attain) plan@24h={plan_done_24h} N_real={throughput_24h} -> {makespan_attain_h:.2f}h")
        print(f"  Cycle time @24h (mean/median)  : {mean_cycle_24h:,.0f}s / {median_cycle_24h:,.0f}s ({mean_cycle_24h/3600:.2f}h)")
        print(f"  Source-wait (mean)             : {mean_sw:,.1f}s")
        print(f"  Throughput(24h-normalized)     : {thr_per_24h:,.0f} lots")
        print(f"  Completed ops                  : {completed_ops}")
        print(f"  Machine utilization            : {utilization:.1f}%")
        print(f"  Mean ready deviation           : {mean_ready_dev:,.1f}s")
        print(f"  Transports                     : {n_tr}")
        print(f"  Transport deviation (mean/med) : {mean_dev:,.1f} / {med_dev:,.1f}s")
        print(f"  Source wait / empty->source    : {mean_sw:,.1f} / {mean_e2s:,.1f}s")
        print(f"  Mean detour ratio              : {mean_detour:.3f}")
        print(f"  OHT dispatches                 : {dispatches}")
        if wall_elapsed_s is not None:
            print(f"  Wall time                      : {wall_elapsed_s:,.2f}s ({run_mode})")
        print(f"  kpi.csv  -> {kpi_path}")
        print("=" * 56)

        try:
            html_path = GanttHTMLExporter(self.jssp_cfg).export()
            if html_path:
                print(f"  gantt.html -> {html_path}")
        except Exception as _e:
            print(f"  [GanttHTML] skipped: {_e}")

        kpi["kpi_path"] = kpi_path
        return kpi


def _jssp_cfg_with_output_dir(base_cfg: JSSPConfig, output_dir: str) -> JSSPConfig:
    """Only change the output path so result CSVs are not overwritten when running the same experiment multiple times."""
    import os as _os
    _os.makedirs(output_dir, exist_ok=True)
    return replace(
        base_cfg,
        initial_gantt_log_csv=_os.path.join(output_dir, 'initial_gantt_log.csv'),
        initial_gantt_csv=_os.path.join(output_dir, 'initial_gantt.csv'),
        result_gantt_log_csv=_os.path.join(output_dir, 'result_gantt_log.csv'),
        result_gantt_csv=_os.path.join(output_dir, 'result_gantt.csv'),
        simulation_log_gantt_csv=_os.path.join(output_dir, 'simulation_log_gantt.csv'),
        gantt_csv=_os.path.join(output_dir, 'gantt.csv'),
        log_machine_csv=_os.path.join(output_dir, 'log_machine.csv'),
        from_to_csv=_os.path.join(output_dir, 'from_to.csv'),
        log_oht_csv=_os.path.join(output_dir, 'log_oht.csv'),
        gantt_html=_os.path.join(output_dir, 'gantt.html'),
    )


def _write_compare_csv(rows, path):
    import csv as _csv
    import os as _os
    if not rows:
        return
    _os.makedirs(_os.path.dirname(_os.path.abspath(path)) or '.', exist_ok=True)
    fields = [
        'case', 'run_mode', 'path_algorithm', 'completed_jobs', 'planned_jobs',
        'production_ratio', 'actual_makespan_s', 'total_tardiness_s',
        'mean_tardiness_s', 'oht_dispatches', 'wall_elapsed_s', 'kpi_path'
    ]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = _csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, '') for k in fields})
    print(f'[COMPARE] {path}')


if __name__ == "__main__":
    # ═══════════════════════════════════════════════════════════════════
    # Main serves only as the execution entry point.
    #
    # For the paper's experiments use run.py / main_exp_run.py; this entry point is
    # for inspecting a single run (optionally with the animation window).
    #
    # Defaults live in the config files:
    #   - Simulation_Machine_Config.py : plan file, CONWIP release, machine rules
    #   - Simulation_OHT_Config.py     : oht_dispatch_mode, oht_path_algorithm,
    #                                    n_oht, SAVD positioning parameters
    #
    # Empty-OHT dispatching switch:
    #   OHTConfig.oht_dispatch_mode = "NVF" | "STD" | "EDD" | "FIFO" | "PRIORITY"
    #                               | "HUNGARIAN"
    #   OHTConfig.oht_savd_positioning = True   # HUNGARIAN + SAVD = the proposal
    # ═══════════════════════════════════════════════════════════════════
    parser = argparse.ArgumentParser()
    parser.add_argument('--animation', action='store_true', help='run with a salabim window open')
    parser.add_argument('--no-animation', action='store_true', help='run only code/DES without a window')
    parser.add_argument('--output-dir', default='output', help='folder to save results')
    parser.add_argument('--compare-paths', action='store_true', help='run ASTAR/BFS each without a window and save a KPI comparison CSV')
    parser.add_argument('--seed', type=int, default=None,
                        help='random seed override (if unspecified, config default=42 fixed). '
                             'repeated experiments use --seed 1, --seed 2 ... to generate different realizations')
    args = parser.parse_args()

    base_cfg = JSSPConfig()


    def _apply_runtime_options(runner_):
        """Override in main only the options corresponding to the run mode."""
        if args.animation:
            runner_.oht_config.enable_animation = True
        if args.no_animation:
            runner_.oht_config.enable_animation = False
        if args.seed is not None:
            # random seed override → applied to all randomness such as machine realized-proc samples / idle roaming
            runner_.oht_config.seed = int(args.seed)


    if args.compare_paths:
        rows = []
        for algo in ['ASTAR', 'BFS']:
            case_dir = f'{args.output_dir}_{algo.lower()}_headless'
            cfg = _jssp_cfg_with_output_dir(base_cfg, case_dir)
            runner = SimulationRunner(
                layout_csv_path='layout_oht.csv',
                machine_csv_path='layout_machine.csv',
                jssp_cfg=cfg,
            )
            _apply_runtime_options(runner)
            runner.oht_config.enable_animation = True
            runner.oht_config.oht_path_algorithm = algo
            kpi = runner.run(enable_animation=True)
            kpi['case'] = algo
            rows.append(kpi)
        _write_compare_csv(rows, f'{args.output_dir}_compare_kpi.csv')
    else:
        cfg = _jssp_cfg_with_output_dir(base_cfg, args.output_dir)
        runner = SimulationRunner(
            layout_csv_path='layout_oht.csv',
            machine_csv_path='layout_machine.csv',
            jssp_cfg=cfg,
        )
        _apply_runtime_options(runner)
        runner.run()