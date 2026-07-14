# OHT Empty-Vehicle Dispatching in a Single-Rail Semiconductor AMHS

Discrete-event simulation (salabim) of a **10-bay / 100-machine CONWIP wafer fab** served by
a fleet of **OHTs** (Overhead Hoist Transports) running on a **single-lane, one-way overhead rail**.
The project studies how to dispatch **empty vehicles** — which idle OHT to send where — in the
congested regime where the rail is prone to gridlock.

## Key results

At the congestion sweet-spot (**120 OHTs**), plain dispatch rules deadlock on many random seeds.
This project (1) explains the deadlock, (2) resolves it, and (3) recovers additional throughput.

| Method (n=120) | makespan attainment | notes |
|---|---|---|
| NVF (Nearest Vehicle First) — baseline | 71.3% | best plain rule; still collapses on hard seeds |
| EDD / FIFO (distance-blind rules) | 4–8% | scatter vehicles → congestion collapse |
| **Deadlock Recovery** | **80.7%** | global cycle detection + force-proceed; 0/13 seed collapse |
| **Recovery + gantt-aware Coverage** | **82.2%** | idle-vehicle spatial coverage of gantt demand |

Three findings:

1. **The n=120 collapse is a distributed *merge-yield deadlock*** — not density, buffers, or ZCU
   zones. Several vehicle queues mutually yield at rail merge points and form a closed wait-for cycle.
2. **A global detect-and-recover controller** stabilizes it (0/13 seeds collapse, vs 17/20 for plain
   Hungarian), making the congested regime usable.
3. **Distributional "coverage" positioning** — keeping the idle-vehicle spatial distribution
   proportional to the gantt's future pickup-demand geography — is the only gantt-aware method that
   improves makespan (empty-travel −27.6%). The system is *coordination-bound*, not raw-capacity-bound.

*(“makespan attainment” = how far the realized 24 h output progresses along the plan’s schedule;
higher is better. See `Simualtion_Main.py::_evaluate`.)*

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

### Plan generators (Stage 1 — offline, run once)

The project is a **two-stage pipeline**. Stage 1 builds a *fixed* production plan; Stage 2 (the
simulator) replays it and measures what transport does to it. A pre-built plan ships in
`gantt_final/`, so you only re-run Stage 1 if you change the fab.

**`gen_machine.py`** — builds the fixed, transport-aware production gantt.
- **Layout**: 10 bays × 100 machines, 6 process types (ETCH / CMP / CVD / IMP / PHOTO / METRO). The
  per-bay type mix is hard-coded in `TEN_BAY_COUNTS` (e.g. Bay1 = 6 ETCH + 4 CMP). Most bays are a
  mix, and each type is spread across several bays — so a lot bounces between bays as it flows.
- **Routing**: every wafer lot follows `ProcessStepInfo` steps 1..40; each step requires one machine
  *type*. The machine is chosen by **ECT (earliest completion time)** — accounting for machine
  availability, lot precedence, and **real rail travel time** (from `machine_travel.csv`, see below),
  so the plan's timing matches the physical layout. METRO steps may be probabilistically skipped.
- **CONWIP** (`--wip_cap N`): fill the fab with `N` lots at t=0, then release one new lot each time a
  lot finishes (fab-out) — keeping work-in-process constant at `N`.
- **Load-balancing** (`--balance_tol`): among machines within `tol` seconds of the best ECT, prefer
  the least-used one, so machines of the same type are used evenly (keeps makespan ~unchanged).
- **Outputs** → `gantt_final/`: `planned_gantt.csv` (op-level schedule), `transport_tasks.csv`
  (each source→dest move with its `planned_ready_sec` — this is the exogenous transport demand the
  OHTs must serve), plus `skipped_steps.csv`, `summary.json`, `validation_report.txt`.

**`gen_machine_travel.py`** — the machine-to-machine travel matrix.
- Runs **Dijkstra on the rail graph** for *every* machine pair (all M_i→M_j) and adds load/unload
  handling time → `machine_travel.csv`. This feeds `gen_machine.py`'s ECT so the plan uses true rail
  distances instead of coarse bay-to-bay approximations.

```bash
# Regenerate the plan (only needed if the layout/steps change; gantt_final/ already ships one)
python gen_machine_travel.py                              # -> machine_travel.csv
python gen_machine.py --out_dir gantt_final --wip_cap 250 \
       --horizon_hours 24 --machine_travel machine_travel.csv --seed 42
```

### Experiment CLI — `run.py`
A single entry point (argparse subcommands) runs every experiment:

| Subcommand | Experiment |
|---|---|
| `run.py recovery` | Deadlock recovery (± plan-congestion / gantt-priority / positioning) |
| `run.py coverage` | Recovery + gantt-aware coverage (the main result) |
| `run.py rule` | Classic dispatch rule (NVF / STD / EDD / FIFO / PRIORITY / QS_STD / BA_STD), no recovery |
| `run.py plan` | Plan-congestion-aware Hungarian (standalone) |

### Diagnostic tool — `_zcudiag.py`

The tool that **identified the deadlock mechanism** (finding #1). It runs a plain Hungarian sim at
n=120 with recovery *off* until it freezes, then inspects the frozen state to reconstruct the
**wait-for graph**:

- for every stopped vehicle it records its position/kind, any held conflict (ZCU) zones, its
  `forced_next` node, and **which other vehicle owns the node/zone it needs to move into**;
- a DFS over that graph then finds the **closed cycle** — the set of vehicles each blocked by the
  next, that no local rule can break (the "17-vehicle merge-yield cycle" of the report).

This is how the collapse was shown to be a *distributed merge-yield deadlock* rather than a density,
buffer, or single-zone problem, and it locates the recurring merge stations (collapse hotspots).

```bash
python _zcudiag.py 99            # diagnose seed 99 (auto horizon ~3500 s catches the freeze)
```

### Input data (fixed plan, shared by all experiments)
`layout_oht.csv` (rail), `layout_machine.csv` (machine placement),
`machine_travel.csv` (M→M travel), `gantt_final/planned_gantt.csv` + `transport_tasks.csv`.

## Quick start

```bash
pip install -r requirements.txt        # salabim, numpy, scipy, pandas

# Recovery + coverage at 120 OHTs, seed 99, full 24 h (the main result)
python run.py coverage out_cov 99 --cov-on 1
# Recovery-only baseline (control)
python run.py coverage out_rec 99 --cov-on 0

# A classic rule, no recovery (records numbers even when it collapses)
python run.py rule out_nvf 99 NVF

# Deadlock diagnostic on a collapsed run
python _zcudiag.py 99

# Full option list
python run.py -h ; python run.py coverage -h
```

Defaults: `--horizon 86400` (24 h), `--n-oht 120`. See `python run.py <cmd> -h` for all flags.

## Method in one paragraph

The production gantt is fixed and *transport-aware* (`gen_machine.py`). At runtime, empty OHTs are
matched to transport tasks by a global Hungarian assignment. On the single rail this is near-optimal
for travel, so most "smart" signals (due-date, urgency, static congestion) do not
help — the system is travel/coordination-bound. Two mechanisms do help: a **global deadlock-recovery**
controller that breaks merge-yield wait-for cycles by force-proceeding one leader per cycle, and a
**coverage** controller that steers *idle* vehicles so their spatial distribution matches the gantt's
future demand geography — reducing empty pickup travel without disturbing reactive assignment.
