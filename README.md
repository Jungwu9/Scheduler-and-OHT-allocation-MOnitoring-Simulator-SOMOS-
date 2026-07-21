# OHT Empty-Vehicle Dispatching in a Single-Rail Semiconductor AMHS

Discrete-event simulation (salabim) of a **10-bay / 100-machine CONWIP wafer fab** served by
a fleet of **OHTs** (Overhead Hoist Transports) running on a **single-lane, one-way overhead rail**.
The project studies how to dispatch **empty vehicles** — which idle OHT to send where — in the
congested regime where the rail is prone to gridlock.

## Repository layout

### Core simulator
| File | Role |
|---|---|
| `Simualtion_Main.py` | Orchestrator (`SimulationRunner`), OHT vehicles, CONWIP `JobSource`, KPI eval, recovery & coverage monitors |
| `Decision_Maker_OHT.py` | **Empty-OHT dispatching** — Hungarian / greedy / RAIL_COORD / rule-based / gantt-aware coverage |
| `Decision_Maker_Machine.py` | Machine-side gantt loader (`choose_next_job` = plan-order replay) |
| `Simulation_Machine.py` | Machine station: processing, routing, fab-out |
| `Simulation_OHT_Layout.py` | Rail graph builder (nodes/edges from `layout_oht.csv`) |
| `Simulation_OHT_Config.py` | OHT / simulation parameters |
| `Simulation_Machine_Config.py` | Path & machine parameters |
| `Gantt_Schedule.py` | Fixed-plan loader / data model |

### Plan generators (Stage 1, offline, run once)
| File | Role |
|---|---|
| `gen_machine.py` | Build the fixed transport-aware production gantt (CONWIP + ECT + load-balancing) |
| `gen_machine_travel.py` | Machine-to-machine travel matrix (rail Dijkstra, all pairs) |

### Experiment CLI — `run.py`
A single entry point (argparse subcommands) runs every experiment:

| Subcommand | Experiment |
|---|---|
| `run.py recovery` | Deadlock recovery (± plan-congestion / gantt-priority / positioning) |
| `run.py coverage` | Recovery + gantt-aware coverage (the main result) |
| `run.py rule` | Classic dispatch rule (NVF / STD / EDD / FIFO / PRIORITY / QS_STD / BA_STD), no recovery |
| `run.py plan` | Plan-congestion-aware Hungarian (standalone) |
| `run.py statelog` | Dynamic-state logging for the source_wait ML study |

### Standalone tools
| File | Role |
|---|---|
| `_zcudiag.py` | Freeze-state wait-for-cycle & topology-hotspot diagnostic |
| `train_conflict_ml.py` | Travel-conflict LightGBM regressor (RAIL cost learning) |

### Input data (fixed plan, shared by all experiments)
`layout_oht.csv` (rail), `layout_machine.csv` (machine placement),
`machine_travel.csv` (M→M travel), `gantt_final/planned_gantt.csv` + `transport_tasks.csv`.

## Quick start

```bash
pip install -r requirements.txt        # salabim, numpy, scipy, pandas (+lightgbm for ML)

# Recovery + coverage at 120 OHTs, seed 99, full 24 h (the main result)
python run.py coverage out_cov 99 --cov-on 1
# Recovery-only baseline (control)
python run.py coverage out_rec 99 --cov-on 0

# A classic rule, no recovery (records numbers even when it collapses)
python run.py rule out_nvf 99 NVF

# Deadlock diagnostic on a collapsed run
python _zcudiag.py out_diag 99

# Full option list
python run.py -h ; python run.py coverage -h
```

Defaults: `--horizon 86400` (24 h), `--n-oht 120`. See `python run.py <cmd> -h` for all flags.

## How the benchmark works

The production gantt is fixed and *transport-aware* (`gen_machine.py`). At runtime, empty OHTs are
matched to transport tasks by the dispatching policy under test — a global Hungarian assignment,
a classic rule (NVF / STD / EDD / FIFO / PRIORITY / QS_STD / BA_STD), a plan-congestion-aware
variant, or a gantt-aware coverage controller (`Decision_Maker_OHT.py`). Policies may be combined
with an optional **deadlock-recovery** controller that detects merge-yield wait-for cycles on the
single rail and force-proceeds one leader per cycle. Each run executes the same fixed plan under a
chosen seed and fleet size, and reports KPIs (makespan attainment, throughput, empty-travel,
source_wait) so dispatching policies can be compared on a common footing. See `run.py <cmd> -h`
for the policy and controller flags exposed per experiment.
