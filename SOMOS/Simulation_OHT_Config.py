from __future__ import annotations


class OHTConfig:
    def __init__(self):

        # ── simulation control ─────────────────────────────────────
        self.seed             = 42
        self.sim_horizon      = 14400.0   # simulation end time (sec)
        self.dispatch_dt      = 2.00      # OHT blocked-retry / REASSIGN re-dispatch period (sec)
        # note: dispatching is event-based (immediate on new pickup / OHT idle). dispatch_dt now
        #     is not a 'response period' but only controls the lock-blocked retry and
        #     mid-flight re-dispatch period. Larger = fewer rounds/churn, faster (2.0 recommended, up to 5.0 fine).
        self.anim_dt          = 0.12     # animation interpolation time step (sec)
        self.anim_fps         = 5        # animation frame rate (salabim default 30 -> lower reduces lag)
                                          # a 100-machine / 1600-edge (~4000 objects/frame) model is heavy at 30fps.
                                          # 10 -> 3x fewer redraws; only slightly less smooth motion (lossless).
        self.anim_light       = True      # If True, skip heavy decorations (less lag): station circles (1142)
                                          # and buffer slots (600, occupancy-color lambda per frame). Rail/machine-box/OHT kept.
        self.animation_speed  = 20.0      # speed-up vs real time
                                          # 50 -> simulation 1s = screen 0.02s
                                          # first JSSP processing (~28s) -> OHT departs after 0.56s


        # ── run-mode switch ───────────────────────────────────────
        # True  : open the salabim animation window for visual inspection
        # False : run only env.run() without a window, saving just CSV logs/KPIs
        # False recommended for paper experiments / algorithm comparison
        self.enable_animation = True
        # ── layout / physics ─────────────────────────────────────────
        self.cell_size          = 10.0   # station-to-station distance (m)
        self.straight_speed     = 5.0    # straight-segment speed (m/s)
        self.curve_speed        = 2.5    # curved-segment speed (m/s)

        # time-per-unit used by the layout builder (sec/m)
        #   straight: 1 / 5.0 = 0.2 s/m  -> 10m straight = 2.0s
        #   curve:    1 / 1.0 = 1.0 s/m  -> 7.85m arc    = 7.85s


        self.base_time_per_unit  = 1.0 / self.straight_speed  # 0.2 s/m
        self.curve_time_per_unit = 1.0 / self.curve_speed      # 1.0 s/m

        self.edge_capacity = 1
        self.margin        = 30.0    # layout margin (coordinate units)

        # ── OHT vehicles ────────────────────────────────────────────────
        self.n_oht             = 80      # bottleneck avoidance: recommend by machines-per-layout (120->80 congestion-relief test)
        self.arc_samples       = 4      # curve guide-node count (8->4, eases bottleneck)
        self.block_retry_dt    = 0.7     # path-blocked retry period (sec)
        self.breaking_threshold   = 10.0    # traffic-congestion cumulative wait -> Breaked (sec)
        self.idle_to_waiting_sec  = 30.0   # Empty stationary cumulative -> Waiting (sec)
        self.waiting_relocate_sec = 20.0   # relocate to another node if Waiting exceeds this time

        # ── Hotspot-aware Idle OHT Escape Rule ────────────────────────
        # keeps the existing idle_roaming.
        # But when an OHT blocks for 5+ sec around cell 9/6 (per layout_oht.csv),
        # steer idle OHTs: fork 13 toward the arc to 17, fork 18 toward the arc to 14.
        self.enable_hotspot_escape_rule = False   # off: overlaps my dispatching heuristic and congestion avoidance, hard to compare (off for now)

        # hotspot cell codes to monitor
        # monitoring only 9/6 could re-crowd OHTs into the escape exits 13/18.
        # so 13/18 are also monitored as secondary hotspots.
        self.hotspot_watch_cell_codes = {"9", "6", "13", "18"}

        # fork cell codes where escape occurs
        self.hotspot_escape_fork_codes = {"13", "18"}

        # activate the escape rule if blocking persists this long around a hotspot
        self.hotspot_escape_wait_sec = 3.0

        # how long an activated escape rule is held
        # 3 sec is too short and may switch off before reaching the fork, so keep it long.
        self.hotspot_escape_hold_sec = 15.0

        # 13/18 fork balance rule
        # also monitor recent throughput and waiting at 13/18; if high, briefly avoid them in A*/idle escape.
        # pure ASTAR baseline: all route-level congestion avoidance (hotspot routing) off -> remove confound
        self.enable_hotspot_fork_balance = False
        self.enable_hotspot_route_penalty = False
        self.enable_hotspot_route_hard_block = False
        self.hotspot_fork_count_window_sec = 20.0
        self.hotspot_fork_count_threshold = 3
        self.hotspot_fork_block_hold_sec = 12.0
        self.hotspot_route_penalty = 80.0
        self.hotspot_fork_penalty = 120.0
        self.hotspot_candidate_live_pressure_weight = 50.0
        self.hotspot_candidate_recent_count_weight = 5.0

        # debug log
        self.debug_hotspot_escape_rule = False

        # ── load / unload time ─────────────────────────────────────────
        self.load_time_min   = 30.0    # fixed 30 sec
        self.load_time_max   = 30.0
        self.unload_time_min = 30.0    # fixed 30 sec
        self.unload_time_max = 30.0

        # ── blocking-externality dispatching heuristic ────────────────
        # while an OHT dwells at source(load)/dest(unload), add the cost of blocking other
        # OHTs that must pass that node to the Hungarian dispatch cost (beta). 0=baseline(off), 1.0=heuristic on.
        self.oht_blocking_weight = 1.0

        # ── C2: urgency/starvation-priority dispatch ──────────────────────────────
        # cost = travel_time - urgency_weight*(late_w*lateness + starve_w*dest_starvation).
        # replaces Hungarian's 'min distance' with 'throughput protection (anti-starvation)'. 0=off.
        self.oht_urgency_weight       = 0.0    # 0=baseline, >0=C2 on (experiments use 1.0)
        self.oht_urgency_late_weight  = 1.0    # lateness (sec) weight
        self.oht_urgency_starve_weight= 150.0  # weight per unit of dest work shortage (in sec)
        self.oht_urgency_starve_ref   = 2.0    # dest is starving if work-on-hand is below this value
        self.oht_urgency_cap_ratio    = 0.6    # bonus cap = 60% of travel cost

        # ── C1: demand-predictive idle positioning ──────────────────────────────
        # drift idle OHTs toward 'machines about to produce a pickup' instead of random roam (not a task).
        # uses idle time Hungarian doesn't -> proactively cuts future-pickup empty travel.
        self.oht_idle_positioning      = False  # True=C1 on
        self.oht_positioning_lookahead = 90.0   # anticipated source if a machine finishes within this time

        # ── congestion-aware cost (GAT v0) ───────────────────────────
        # add route congestion delay to the Hungarian cost (free-flow travel time) -> approximates 'actual travel time'.
        # 0=baseline(free-flow), >0=reflect congestion (experiments use 1.0). node/edge penalty uses
        # oht_congestion_node_penalty(8) / oht_congestion_edge_penalty(12).
        self.oht_congestion_cost_weight = 0.0

        # ── disturbance robustness: machine breakdown ──────────────────────────────
        self.enable_machine_breakdowns = False
        self.machine_mtbf_sec = 3600.0   # mean time between failures (exponential)
        self.machine_mttr_sec = 600.0    # mean repair time (exponential) -> ~14% downtime
        self.breakdown_seed   = 123      # identical failure sequence across policies

        # ── PLAN_PRIORITY heuristic rules (when oht_dispatch_mode="PLAN_PRIORITY") ──
        # base form = essential-1 (slack priority) + level-1 (commit sequentially by min distance, urgent tasks first).
        # the three below are extension flags for measuring individual contributions (off by default).
        self.oht_pair_yield        = False   # yield level-2: 1-step scarcity lookahead
        self.oht_pair_yield_factor = 1.5     # within factor x min-cost = candidate (urgency-judgment radius)
        self.oht_plan_lookahead    = False   # extension-1: steer remaining idle vehicles toward anticipated source (no commit)
        self.oht_bottleneck_boost  = 0.0     # extension-2: priority boost for tasks just before a bottleneck (e.g. 600)
        self.oht_bottleneck_types  = "PHOTO" # bottleneck machine_type (comma-separable)

        # ── RAIL_COORD heuristic (when oht_dispatch_mode="RAIL_COORD") ──
        # mechanism-1 (core): time-space reserve in-flight OHT paths -> raise cost by the candidate path's time conflict.
        self.oht_rail_conflict_weight    = 1.0   # cost weight per sec of conflict (0=off=pure Hungarian REASSIGN)
        self.oht_rail_merge_weight       = 1.0   # mechanism-2: merge/crossing-node conflict weight (>1 stronger)
        self.oht_rail_null_action        = False # mechanism-3: defer dispatch when jam threshold exceeded
        self.oht_rail_congestion_threshold = 4.0 # jam if edge simultaneous-reservation peak >= this value
        self.oht_rail_lookahead          = 300.0 # reservation horizon (sec)
        self.oht_rail_reassign           = False # True=RAIL+REASSIGN (reassignment on, slower)
        self.oht_rail_conflict_cap_ratio = 1.0
        self.oht_rail_starve_guard_sec   = 300.0  # tasks this late are exempt from conflict (anti-starvation)
        # adaptive w: conflict_weight dynamic with rail occupancy (pre-ML rule-based upper-bound experiment)
        self.oht_rail_adaptive_w         = False # True=dynamic w (busy*(max-min)+min)
        self.oht_rail_w_min              = 1.0
        self.oht_rail_w_max              = 8.0
        # B-3: reservation-aware idle positioning (drift empty vehicles only to conflict-free spots)
        self.oht_rail_idle_positioning   = False

        # ── hard slack-defer (defer dispatch of deferrable lots -> less injection -> less congestion) ──
        self.oht_slack_defer           = False  # True=on (ready_tasks filter, mode-agnostic)
        self.oht_slack_defer_threshold = 0.0    # defer if slack>this value (0=JIT; larger defers less)

        # ── animation — OHT triangle size ─────────────────────────────
        # for cell_size = 10m: set OHT to about 60% of cell size
        self.oht_length      = 6.0    # OHT length (m)
        self.oht_width       = 6.0    # OHT width  (m)
        self.oht_line_width  = 0.7
        self.oht_label_offset = 2.5   # number-label position (above triangle)
        self.oht_font_size   = 5      # small since only the number is shown
        self.oht_label_color = "white"
        self.breaked_dot_radius = 1.0

        # ── animation — layout display ──────────────────────────────
        self.background_color    = "20%gray" #20%gray
        self.rail_color          = "white" #white
        self.rail_line_width     = 0.5

        self.main_node_color     = "cyan"
        self.station_node_color  = "lightgray"
        self.node_radius         = 1.1   # node circle radius (m)
        self.node_label_offset   = 2.0   # label offset (m)
        self.node_font_size      = 6     # size based on machine number (M10)

        self.time_font_size = 10

        # capture / UI
        self.capture_mode = True          # True for capture
        self.show_menu_buttons = True    # whether to show Menu buttons
        self.show_salabim_modelname = False   # whether to show "a salabim model"
        self.show_header_text = False     # whether to show the custom-drawn title/time

        # simulation speed (animation 1 sec = sim_speed simulation sec)
        # JSSP processing time is 28~812s, so set to 100x speed
        self.sim_speed = 5.0

        # ── per-OHT-state colors ──────────────────────────────────────────
        self.state_style = {
            "Empty":     {"fill": "white",      "line": "black"},
            "Loading":   {"fill": "deepskyblue","line": "black"},
            "Loaded":    {"fill": "limegreen",  "line": "black"},
            "Unloading": {"fill": "limegreen",  "line": "black"},
            "Waiting":   {"fill": "white",       "line": "red"},
            "Breaked":   {"fill": "white",      "line": "red"},
            "Returning": {"fill": "plum",       "line": "black"},
        }
        self.prewait_lookahead_nodes = 2

        # ── OHT path algorithm ──────────────────────────────────────
        # "BFS"                 : directed BFS. min hop count + destination-proximity tie-break
        # "ASTAR"               : directed A*. based on edge.travel_time / coordinate distance
        # "COUNT_AWARE_ASTAR"   : penalize/hard-block nodes with high recent pass count
        # "CONGESTION_AWARE_ASTAR" is wired to behave identically to COUNT_AWARE_ASTAR
        self.oht_path_algorithm = "ASTAR"  # ASTAR / BFS / COUNT_AWARE_ASTAR  (less video lag: skip congestion check)

        # ── Count-aware routing ─────────────────────────────────────
        # count how many OHTs passed a node over the recent window; if high, A* avoids it.
        # e.g. to ease crowding onto a single point like the bottleneck where OHT #38 sits.
        self.enable_oht_route_count_record = True
        self.oht_route_count_window_sec = 30.0

        # apply a cost penalty proportional to the amount over the soft threshold
        # count <= 4 : normal, count > 4 : progressively more expensive to pass
        self.oht_route_count_soft_threshold = 4
        self.oht_route_count_penalty = 18.0

        # at or above the hard threshold, exclude from A* candidates. But do not exclude the destination node.
        # if all paths are blocked, release the hard block and do a fallback search.
        self.enable_oht_route_count_hard_block = True
        self.oht_route_count_hard_threshold = 8

        # reflect momentary occupancy/entry-intent before the count accumulates
        self.enable_oht_route_live_penalty = True
        self.oht_route_live_node_penalty = 25.0
        self.oht_route_live_intent_penalty = 20.0

        # Loaded OHT priority: Empty OHTs treat nodes a Loaded OHT is heading to as more expensive
        self.enable_oht_loaded_priority_routing = True
        self.oht_route_loaded_priority_penalty = 25.0

        # ── OHT dispatching algorithm ───────────────────────────────
        # "GREEDY"                       : min-cost greedy per tick step
        # "HUNGARIAN"                    : global sum minimization via scipy linear_sum_assignment
        # "HUNGARIAN_REASSIGN"           : Hungarian + allow reassigning vehicles before pickup
        # "CONGESTION_AWARE_HEURISTIC"   : reflect current OHT/edge congestion and destination-buffer congestion as penalty
        # "PREDICTIVE_DISPATCHING"       : predictive dispatch reflecting planned_end lateness + source out_buffer pressure
        self.oht_dispatch_mode = "GREEDY"  # less video lag: skip Hungarian (removes dispatch freeze). experiments use HUNGARIAN_REASSIGN
        # "PLAN_PRIORITY" : sort by planned-gantt slack priority + sequential commit (no Hungarian, see flags below)
        # "RAIL_COORD"    : time-space reservation conflict cost (rail-occupancy coordination) + Hungarian REASSIGN engine (see flags below)

        # ── * Plan-congestion-aware Hungarian (gantt-based RAIL_COORD) ──
        # build a space-time congestion map D[edge,time_bucket] derived from the plan (transport_tasks),
        # and add to cost how busy the candidate path is, per plan, 'at the time I pass'.
        # unlike RAIL_COORD (current in-flight reservations), it looks at the *entire future schedule*, so it is proactive.
        # weight=0 is off. layered on top of HUNGARIAN mode (no travel-efficiency sacrifice, travel-time correction).
        self.oht_plan_congestion_weight = 0.0
        self.oht_plan_bucket_sec = 90.0
        self.oht_plan_tasks_csv = 'gantt_final/transport_tasks.csv'
        self.oht_plan_congestion_cap_ratio = 1.0    # plan cost cap = cap*base (same guard as RAIL)
        self.oht_plan_starve_guard_sec = 300.0      # tasks this late vs plan are exempt (anti-starvation)
        self.oht_plan_time_anchor = True            # *drift fix: align lookup to task plan-time (False=v0 realized)
        # ── adaptive-pw: dynamically tune plan_weight by blocked(Waiting) ratio ──
        # flowing (blocked~0)->w_min (neutral, don't disturb the stable regime), jam forming (blocked up)->w_max (strong avoidance).
        # attempt to robustify the fragility of a fixed pw (per-seed optimum differs) via live feedback.
        self.oht_plan_adaptive_w = False
        self.oht_plan_w_min = 0.0
        self.oht_plan_w_max = 4.0
        self.oht_plan_w_blk_ref = 0.30              # w_max reached at this blocked ratio

        # ── ** Deadlock recovery: global merge-yield cycle detection + recovery ──────
        # when merge-wait exceeds a threshold, find global wait-for cycles and, per cycle, grant
        # force-proceed (enter ignoring yield) to the min-vid leader to break the loop. Resolves n120
        # collapse (distributed merge-yield cycles) via global detection instead of local rules. 0=off.
        self.oht_deadlock_recovery = False
        self.oht_dl_recovery_dt = 8.0          # recovery check period (sec)
        self.oht_dl_merge_wait_thresh = 15.0   # suspect deadlock if merge-wait exceeds this time
        # gantt-informed resolution: on cycle break, choose the leader by
        # 'lot most behind plan (max lateness=now-planned_end)' (True) instead of min-vid (False).
        # makespan is travel-bound so expected neutral; room to improve schedule adherence (transport_deviation).
        self.oht_dl_gantt_priority = False


        # Congestion-aware heuristic penalty coefficients (in sec)
        self.oht_congestion_node_penalty = 8.0
        self.oht_congestion_edge_penalty = 12.0
        self.oht_buffer_block_penalty = 30.0

        # dispersion penalty to prevent Empty OHTs crowding into one island/corridor
        # higher value : more strongly avoid dispatching extra vehicles into the same zone
        self.oht_zone_balance_penalty = 25.0

        # Predictive dispatching score coefficients
        self.oht_predictive_late_weight = 0.18
        self.oht_predictive_wait_weight = 0.35
        # limit predictive bonus so it doesn't overwhelm travel cost
        self.oht_predictive_bonus_cap_ratio = 0.35

        # swap penalty (sec) to prevent thrashing during REASSIGN.
        # a patch added to the cost matrix when a reassignable vehicle bound to an existing task
        # matches a *different* task. Hungarian swaps only when it shows cost savings above this value.
        # 0 exposes thrashing as-is. Use a large value to ignore small OHT/task fluctuations.
        self.oht_swap_penalty = 30.0

        # ── Idle OHT behavior ────────────────────────────────────────────
        # "RANDOM"      : random among adjacent stations (legacy behavior)
        # "CENTER_RAIL" : steer to stay on the two central horizontal rails (inner 2 lanes).
        #                 on receiving a task, naturally moves outward along shortest_path.
        # "STAY"        : stay in place (debug)
        self.oht_idle_behavior = "RANDOM"   # idle OHTs always random-roam (CENTER_RAIL deprecated)

        # ── Pre-dispatch (look-ahead) ────────────────────────────────
        # add in-process jobs with remaining processing time < look_ahead_sec to the task pool early.
        # the OHT departs early so empty-travel cost is absorbed into machine processing time.
        # 0 disables. Too large -> OHT waits long at source -> pickup timeout.
        # recommended: load_time + avg transport ~= 100~200s
        self.oht_predispatch_lookahead = 60.0

        # ── (A) Coverage positioning: match idle-OHT 'distribution' to gantt demand density ──
        # unlike per-source positioning (mean, zero-sum), the goal is to eliminate uncovered hot regions
        # and reduce the *tail* of source_wait. Only adjusts idle vehicles' roam_target.
        self.oht_coverage_positioning = False   # True=on
        self.oht_coverage_dt = 45.0             # coverage re-placement period (sec)
        self.oht_coverage_window = 900.0        # imminent-pickup judgment window (sec)
        self.oht_coverage_grid = 0.0            # cell size (coordinate units); 0=auto (bbox/4)
        self.oht_coverage_prior_weight = 1.0    # gantt geographic-prior weight (0=realized only)


        # ── Station-based ZCU / SPZR ────────────────────────────────
        # True: curve/merge conflict zones are entered only after pre-reserving at a station
        # False: use only the existing front_nodes/edge.res scheme
        self.enable_station_zcu = True
        self.debug_station_zcu = False

        self.oht_task_fill_color  = "deepskyblue"
        self.oht_cargo_fill_color = "limegreen"

        # ── border-blink parameters ────────────────────────────────
        self.blink_period_sec = 0.40

        # border blink while Loading
        self.loading_blink_on_line  = "deepskyblue"
        self.loading_blink_off_line = "black"

        # border blink while Unloading
        self.unloading_blink_on_line  = "limegreen"
        self.unloading_blink_off_line = "black"

        # Waiting keeps the body + red border
        self.waiting_line_color = "red"
        self.waiting_line_width = 1.4