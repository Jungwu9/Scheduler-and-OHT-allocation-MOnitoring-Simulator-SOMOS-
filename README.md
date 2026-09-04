# SOMOS — Scheduler and OHT Allocation Monitoring Simulator

Code for the paper

> **Empty Vehicle Dispatching of OHT Towards Minimized Makespan by Leveraging Machine Schedule**

---

## Simulation

SOMOS is an event-driven simulation of a **10-bay / 100-machine CONWIP wafer fab** served by
**OHTs** (Overhead Hoist Transports) on a **single-lane, one-way overhead rail** (1,622 nodes /
1,686 edges, mean out-degree 1.04, 64 merge points).

Production and transport run in one loop. Each operation is processed with realized (sampled)
processing time; when it finishes, a pickup request is generated for the machine of the next
operation; the dispatcher assigns an empty vehicle; the vehicle travels, loads, and delivers; and
the next operation starts only once the lot has arrived. Transport delay therefore postpones the
operations that depend on it, instead of being absorbed as a fixed inter-operation time.

The production plan is fixed and shipped with the repository — 33,510 operations over 2,000 lots,
679 of them planned to complete within 24 h at 95 % machine utilisation — and is replayed
identically by every method and seed, so run-to-run differences come only from dispatching and the
traffic it creates.

Vehicles move on the directed rail graph with A\* routing, single-vehicle edge occupancy, and
merge-yield rules. There is no deadlock-recovery controller: a run is governed purely by the
dispatching policy under test, and a run that gridlocks is reported rather than repaired.

| File | Role |
|---|---|
| `Simulation_Main.py` | orchestrator (`SimulationRunner`), OHT vehicles and traffic rules, CONWIP job source, KPI evaluation |
| `Decision_Maker_OHT.py` | empty-OHT dispatching and path search (A\*, BFS) |
| `Decision_Maker_Machine.py` | machine-side plan replay |
| `Simulation_Machine.py` | machine station: processing, routing, fab-out |
| `Simulation_OHT_Layout.py` | rail graph builder |
| `Simulation_OHT_Config.py`, `Simulation_Machine_Config.py` | parameters |
| `Gantt_Schedule.py` | fixed-plan loader, realized processing-time sampling |
| `layout_oht.csv`, `layout_machine.csv`, `gantt_final/`, `*Info.txt` | layout, plan and fab specification |

## Method

Seven dispatching methods are implemented:

| Method | Assignment | Idle vehicles |
|---|---|---|
| `NVF` | nearest vehicle first | random roam |
| `STD` | shortest total distance (empty + loaded) | random roam |
| `EDD` | earliest planned due date | random roam |
| `FIFO` | source ready first | random roam |
| `PRIORITY` | largest lateness + source queue pressure | random roam |
| `HUNGARIAN` | global minimum-sum assignment over the vehicle × task travel-time matrix | random roam |
| **`SAVD`** | **the same Hungarian assignment** | **schedule-aware distribution** |

**SAVD (Schedule-Aware Vehicle Distribution)** is the proposed method. It repositions *idle*
vehicles before pickup requests occur, and leaves request assignment untouched — SAVD and
`HUNGARIAN` use the identical Hungarian procedure, so they differ in exactly one thing.

Every 45 s SAVD

1. estimates spatial transport demand per machine from three sources — a fixed weight for a lot
   already waiting in the output buffer, a weight that grows as the operation in process
   approaches completion within the window *H*, and a schedule-derived prior counting the planned
   transport tasks that originate at that machine;
2. aggregates the machine weights into spatial cells and turns them into a target number of idle
   vehicles per cell, proportional to the cell's share of total demand;
3. moves idle vehicles from surplus cells to the highest-demand deficit cells, choosing the
   nearest surplus vehicle by rail travel time.

A repositioned vehicle is only given a roam target: it is never reserved for a future request and
remains available for normal assignment at any moment.

| SAVD parameter | Default | Config key |
|---|---|---|
| immediate pickup demand weight (α) | 3 | constant in `_assign_savd_positioning` |
| remaining processing time window (H) | 900 s | `oht_savd_window` |
| schedule demand weight (λ) | 1.0 | `oht_savd_prior_weight` |
| spatial cell size | layout span / 4 | `oht_savd_grid` (0 = auto) |
| repositioning interval | 45 s | `oht_savd_dt` |

Implementation: `OHTDecisionMaker._assign_savd_positioning` in `Decision_Maker_OHT.py`, driven by
`_SavdPositioningMonitor` in `Simulation_Main.py`.

## UI

The monitoring interface runs a simulation and shows it live. Two views share one simulation
clock: the **rail view** with vehicle positions and movement states, and the **machine Gantt chart**
with the execution state of production operations, alongside system indicators (executed
operations, completed lots, active moves) and per-vehicle / per-lot traces.

While the simulation runs it streams three CSVs — transport events, machine operations and vehicle
moves — and the interface rebuilds itself from them at a fixed interval, so the run can be watched
as it progresses or replayed afterwards.

| File | Role |
|---|---|
| `run_ui.py` | launcher |
| `UI/UI_Live.py` | runs the simulation, serves the page, refreshes it |
| `UI/live_trace_hooks.py` | the three CSV sinks the simulator streams into |
| `UI/oht_trace_data.py` | builds the view model (layout, routes, Gantt, indicators) |
| `UI/oht_trace_html_exporter.py` | renders the interactive page |
| `UI/run_oht_trace_ui.py` | one-shot build from an existing output directory |

## Running

```bash
pip install -r requirements.txt          # salabim, numpy, scipy, pandas (Python 3.11+)
```

**Single run** — 120 OHTs, 24 h, seed 1 by default:

```bash
python run.py savd out_savd 1                 # proposed method
python run.py savd out_hun  1 --savd-on 0     # its control: Hungarian, positioning off
python run.py rule out_nvf  1 NVF             # a baseline rule
python run.py rule out_test 1 HUNGARIAN --horizon 1800 --n-oht 60    # short smoke test
```

Each run writes `kpi.csv`, `transport.csv`, `log_machine_sim.csv` and a Gantt page to the output
directory. Options: `--n-oht`, `--horizon`, `python run.py -h`.

**UI**:

```bash
python run_ui.py                              # SAVD, then open http://127.0.0.1:8765
python run_ui.py --method HUNGARIAN --seed 3 --n-oht 100
python run_ui.py --horizon 7200 --refresh-sec 5 --port 8899
python run_ui.py --no-run --output-dir out_savd    # replay a finished run
```

**Experiments** — every method shares the seed set, so comparisons are paired:

```bash
python main_exp_run.py            # 7 methods x 20 seeds
python abl_run.py                 # SAVD ablation
python sweep_run.py --n-oht 100   # fleet-size sweep, one density per call
python main_exp_merge.py          # merge shards of a split batch
```

A 24 h run with 120 OHTs takes roughly 1.5 h on one core. Runs are independent and
single-threaded, so batches should be launched in parallel with `OMP_NUM_THREADS=1`.

## Copyright

© 2026 the authors. All rights reserved.

* **Simulation (SOMOS) & method (SAVD)** — Jungwoo Ahn
* **User interface** — Jihoon Bae
