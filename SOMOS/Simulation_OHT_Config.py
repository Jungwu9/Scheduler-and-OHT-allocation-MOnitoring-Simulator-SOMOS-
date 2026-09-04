from __future__ import annotations

import os

# Input files ship with the repository; resolve them from this file so a run
# works regardless of the caller's working directory.
_REPO = os.path.dirname(os.path.abspath(__file__))


class OHTConfig:
    def __init__(self):

        # ── simulation control ─────────────────────────────────────
        self.seed             = 42
        self.sim_horizon      = 86400.0   # simulation end time (sec); the paper uses 24 h
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
        self.enable_animation = False
        # UI: stream transport / machine / vehicle events to CSV while running
        self.enable_live_trace = False
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
        self.n_oht             = 120     # fleet size; the paper's main experiment uses 120
        self.arc_samples       = 4      # curve guide-node count (8->4, eases bottleneck)
        self.block_retry_dt    = 0.7     # path-blocked retry period (sec)
        self.breaking_threshold   = 10.0    # traffic-congestion cumulative wait -> Breaked (sec)
        self.idle_to_waiting_sec  = 30.0   # Empty stationary cumulative -> Waiting (sec)
        self.waiting_relocate_sec = 20.0   # relocate to another node if Waiting exceeds this time

        # merge-yield rule: a vehicle on the straight lane gives way once a vehicle
        # waiting on the curve has been blocked this long (these were previously
        # read with getattr defaults; the values are unchanged)
        self.curve_priority_wait_sec = 2.0   # curve waiter blocked this long -> gets priority
        self.straight_yield_sec      = 1.2   # extra hold the straight-lane vehicle takes

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
        self.load_time_min   = 25.0    # paper Table 2: 25 s
        self.load_time_max   = 25.0
        self.unload_time_min = 25.0    # paper Table 2: 25 s
        self.unload_time_max = 25.0

        # ── disturbance robustness: machine breakdown ──────────────────────────────
        self.enable_machine_breakdowns = False
        self.machine_mtbf_sec = 3600.0   # mean time between failures (exponential)
        self.machine_mttr_sec = 600.0    # mean repair time (exponential) -> ~14% downtime
        self.breakdown_seed   = 123      # identical failure sequence across policies

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
        # "NVF"       : nearest vehicle first (shortest empty leg)
        # "STD"       : shortest total distance (empty + loaded)
        # "EDD"       : earliest planned due date (planned_end)
        # "FIFO"      : source ready first
        # "PRIORITY"  : largest lateness + source queue pressure
        # "HUNGARIAN" : global sum minimisation (scipy linear_sum_assignment)
        # The proposed method SAVD is not a mode: it runs HUNGARIAN and switches
        # on SAVD positioning below, so the two differ only in idle behaviour.
        self.oht_dispatch_mode = "HUNGARIAN"

        # ── Gridlock watchdog (early termination) ───────────────────
        # A recovery-free run can freeze in a merge-yield deadlock and then burn
        # the rest of the 24 h horizon doing nothing. The watchdog samples a
        # progress signature (lot completions, machine ops, every OHT position)
        # every `gridlock_check_dt_s` of SIM time; if the signature has not moved
        # for `gridlock_timeout_s` of SIM time, the system is frozen and the run
        # is stopped early.
        #
        # This is numerically lossless: a frozen system produces no further
        # completions, so throughput/makespan/attainment are identical to what a
        # full-horizon run would report. Utilization is still normalized by the
        # configured horizon, not by the truncated end time.
        #
        # 0 disables the watchdog (run always to the full horizon).
        self.gridlock_timeout_s = 1200.0    # 20 min of sim time with zero progress
        self.gridlock_check_dt_s = 60.0     # sampling period (sim sec)

        # swap penalty (sec): surcharge added to the cost matrix when a vehicle
        # already bound to a task is matched to a *different* one, so Hungarian
        # only swaps when the saving clearly exceeds the churn.
        self.oht_swap_penalty = 30.0

        # ── Idle OHT behavior ────────────────────────────────────────────
        # "RANDOM"      : random among adjacent stations (legacy behavior)
        # "CENTER_RAIL" : steer to stay on the two central horizontal rails (inner 2 lanes).
        #                 on receiving a task, naturally moves outward along shortest_path.
        # "STAY"        : stay in place (debug)
        self.oht_idle_behavior = "RANDOM"   # idle OHTs always random-roam (CENTER_RAIL deprecated)

        # planned transports; the demand geography SAVD spreads idle vehicles over
        self.oht_savd_tasks_csv = os.path.join(_REPO, 'gantt_final', 'transport_tasks.csv')

        # ── (A) SAVD positioning: match idle-OHT 'distribution' to gantt demand density ──
        # unlike per-source positioning (mean, zero-sum), the goal is to eliminate uncovered hot regions
        # and reduce the *tail* of source_wait. Only adjusts idle vehicles' roam_target.
        self.oht_savd_positioning = False   # True=on
        self.oht_savd_dt = 45.0             # SAVD re-placement period (sec)
        self.oht_savd_window = 900.0        # imminent-pickup judgment window (sec)
        self.oht_savd_grid = 0.0            # cell size (coordinate units); 0=auto (bbox/4)
        self.oht_savd_prior_weight = 1.0    # gantt geographic-prior weight (0=realized only)
        # demand-term ablation: 'all' = realized(out_buffer + imminence) + gantt prior,
        #   'realized' = drop the prior, 'prior' = drop the realized terms (plan geography only)
        self.oht_savd_terms = 'all'
        #   'roll'/'rollonly' use a rolling plan window instead of the static prior;
        #   the anchor maps wall clock onto the plan time axis (realized execution drifts
        #   behind the plan, so a wall-clock query would read the wrong slice)
        self.oht_savd_roll_anchor = True


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