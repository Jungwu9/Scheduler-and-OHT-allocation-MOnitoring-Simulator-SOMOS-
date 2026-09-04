from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import salabim as sim

from UI.live_trace_hooks import append_machine_live_event


@dataclass
class Job:
    """
    A single product instance within the simulation.

    job_id        : unique ID of this instance (sequence number across the whole simulation)
    job_type_id   : JSSP job type (0~14) — which process route it follows
    op_index      : number of operations completed so far (start=0, +1 each time processing finishes)
    product_type  : product type string (e.g. "P0")
    origin        : starting layout node name
    current_node  : current layout node name
    next_candidate_nodes: list of next-destination node names
                          holds at most 1 based on JSSP routing.
                          empty list = last operation completed (product finished)
    created_time  : creation time (simulation time)
    """
    job_id: int
    job_type_id: int
    op_index: int
    product_type: str
    origin: str
    current_node: str
    next_candidate_nodes: List[str]
    created_time: float
    # ── for gantt (GanttSchedule) linkage (defaults -> compatible with legacy path) ──────────
    lot_id: str = ""                  # gantt lot instance ID (e.g. "L00001")
    step_no: int = 0                  # gantt step to process now (1..40)
    planned_ready_time: float = 0.0   # gantt planned end time of the just-completed op (for deviation label)


class MachineStation(sim.Component):
    """
    Service processor that processes the job delivered by the OHT.

    Given jssp_data, machine_id, machine_node_map, it automatically updates the job's
    next destination per JSSP routing after processing completes.

    Changes:
    - always select the actual machine by SPT (Shortest Processing Time)
    - removed FIFO fallback
    """

    def setup(
        self,
        node,
        machine_config,
        jssp_data=None,         # JSSPData instance
        jssp_machine_id=None,   # this station's JSSP machine ID (0~n_machines-1)
        machine_node_map=None,  # {machine_id: node_name}
        machine_decision=None,  # selection rule provider (tries singleton fallback if absent)
        schedule=None,          # GanttSchedule instance (gantt mode). If given, used instead of jssp_data
        machine_name=None,      # this station's physical machine name (e.g. "M37"). Required in gantt mode
        rng=None,               # random.Random for realized proc sampling (optional)
        machine_live_state=None,  # UI: streams finished operations to CSV
    ):
        self.node = node
        self.machine_config = machine_config
        self.jssp_data = jssp_data
        self.jssp_machine_id = jssp_machine_id
        self.machine_node_map = machine_node_map or {}
        self.machine_decision = machine_decision
        self.schedule = schedule
        self.machine_name = machine_name
        self._rng = rng
        self.machine_live_state = machine_live_state
        self._broken = False    # disturbance robustness: halt processing while broken

        self.in_buffer: List[Job] = []
        self.processing_job: Optional[Job] = None
        self.out_buffer: List[Job] = []
        self.reserved_in_slots: int = 0

        self.service_res = sim.Resource(name=f"svc_{self.node.name}", capacity=1)

        # real-time execution record
        self._sim_log: list = []

    # ------------------------------------------------------------------
    # Buffer / reservation
    # ------------------------------------------------------------------
    def can_accept_input(self) -> bool:
        return (len(self.in_buffer) + self.reserved_in_slots) < self.machine_config.input_buffer_cap

    def reserve_input_slot(self) -> bool:
        if not self.can_accept_input():
            return False
        self.reserved_in_slots += 1
        return True

    def release_reserved_input_slot(self):
        if self.reserved_in_slots > 0:
            self.reserved_in_slots -= 1

    def has_output_job(self) -> bool:
        return len(self.out_buffer) > 0

    def pop_output_job(self) -> Optional[Job]:
        return self.out_buffer.pop(0) if self.out_buffer else None

    def peek_output_job(self) -> Optional[Job]:
        """Return the first job in out_buffer without removing it."""
        return self.out_buffer[0] if self.out_buffer else None

    # ------------------------------------------------------------------
    # Visual helpers (for animation lambda)
    # ------------------------------------------------------------------
    def is_processing(self) -> bool:
        return self.processing_job is not None

    def visual_in_count(self) -> int:
        return len(self.in_buffer)

    def visual_out_count(self) -> int:
        return len(self.out_buffer)

    def visual_reserved_in_count(self) -> int:
        return self.reserved_in_slots

    def visible_status(self):
        return {
            "in_buffer": len(self.in_buffer),
            "processing": 0 if self.processing_job is None else 1,
            "out_buffer": len(self.out_buffer),
            "reserved_in_slots": self.reserved_in_slots,
            "idle": self.processing_job is None,
        }

    # ------------------------------------------------------------------
    # Job I/O
    # ------------------------------------------------------------------
    def receive_job(self, job: Job):
        if self.reserved_in_slots > 0:
            self.reserved_in_slots -= 1
        elif len(self.in_buffer) >= self.machine_config.input_buffer_cap:
            raise RuntimeError(f"input buffer full at {self.node.name}")

        job.current_node = self.node.name
        self.in_buffer.append(job)

        if self.ispassive():
            self.activate()

    # ------------------------------------------------------------------
    # JSSP routing update
    # ------------------------------------------------------------------
    def _update_job_routing(self, job: Job):
        """
        After processing completes, update the job's next destination per routing.
        gantt mode (schedule) takes priority; otherwise the legacy ta01 (jssp_data) path.
        """
        # ── gantt mode ────────────────────────────────────────────────
        if self.schedule is not None:
            finished_step = job.step_no
            # planned end time of the step just finished = planned dispatch-out time of this transport (planned label)
            pr = self.schedule.planned_ready(job.lot_id, finished_step)
            job.planned_ready_time = pr if pr is not None else self.env.now()

            job.op_index += 1
            next_m = self.schedule.next_machine_after(job.lot_id, finished_step)
            if next_m is None:
                job.next_candidate_nodes = []        # lot finished (fab-out)
                job.step_no = -1
            else:
                next_node = self.machine_node_map.get(next_m)   # {M-name: node}
                job.next_candidate_nodes = [next_node] if next_node else []
                ns = self.schedule.next_step_no(job.lot_id, finished_step)
                job.step_no = ns if ns is not None else -1
            return

        # ── legacy ta01 path (transitional fallback) ────────────────────
        if self.jssp_data is None or self.jssp_machine_id is None:
            return

        route = self.jssp_data.job_routes.get(job.job_type_id)
        if route is None:
            return

        job.op_index += 1  # this operation completed

        next_m_id = route.next_machine_after(self.jssp_machine_id)
        if next_m_id is None:
            # last operation → product finished
            job.next_candidate_nodes = []
        else:
            next_node = self.machine_node_map.get(next_m_id)
            job.next_candidate_nodes = [next_node] if next_node else []

    # ------------------------------------------------------------------
    # Dispatch helper
    # ------------------------------------------------------------------
    def _get_processing_time(self, job: Job) -> float:
        """
        Return the processing time of the given job on the current machine.
        gantt mode -> realized proc (distribution sample); else ta01 scaled; if neither, 10.0.
        """
        if self.schedule is not None:
            t = self.schedule.realized_proc(job.lot_id, job.step_no, self._rng)
            if t is not None:
                return t
        ptime = 10.0
        if self.jssp_data is not None and self.jssp_machine_id is not None:
            t = self.jssp_data.get_scaled_time(job.job_type_id, self.jssp_machine_id)
            if t is not None:
                ptime = t
        return ptime

    def _choose_job(self) -> Optional[Job]:
        """Select the next job using MachineDecisionMaker's dispatch rule."""
        if not self.in_buffer:
            return None

        decision = self.machine_decision
        if decision is None:
            try:
                from Decision_Maker_Machine import get_active_machine_decision
                decision = get_active_machine_decision()
            except Exception:
                decision = None

        if decision is not None:
            chosen = decision.choose_next_job(self)
            if chosen is not None:
                return chosen

        # fallback: SPT
        return min(
            self.in_buffer,
            key=lambda job: (
                self._get_processing_time(job),
                job.created_time,
                job.job_id,
            )
        )

    # ------------------------------------------------------------------
    # Processing
    # ------------------------------------------------------------------
    def process(self):
        while True:
            if getattr(self, "_broken", False):
                # broken: halt processing (MachineBreakdown re-activates on repair)
                yield self.passivate()
                continue
            if not self.in_buffer:
                yield self.passivate()
                continue

            while len(self.out_buffer) >= self.machine_config.output_buffer_cap:
                yield self.hold(0.05)

            # actual machine dispatch uses the MachineDecisionMaker rule
            chosen_job = self._choose_job()
            if chosen_job is None:
                yield self.passivate()
                continue

            self.in_buffer.remove(chosen_job)
            self.processing_job = chosen_job
            _job_start = self.env.now()

            ptime = self._get_processing_time(self.processing_job)
            # expose so external (OHTDecisionMaker) can estimate remaining processing time and pre-dispatch
            self._processing_start_time = _job_start
            self._processing_duration = ptime

            yield self.hold(ptime)

            finished = self.processing_job
            self.processing_job = None
            self._processing_start_time = None
            self._processing_duration = None

            # for gantt label: preserve current step before routing update
            _finished_step = finished.step_no

            # update next destination (gantt schedule or ta01 routing)
            self._update_job_routing(finished)

            # last step completed = fab-out: no transport destination, so do not put in out_buffer.
            # (if put there, the OHT cannot pick it up, out_buffer stays permanently occupied, and the fab jams.)
            # on_fabout -> JobSource.notify_fabout decrements WIP + releases next lot.
            if getattr(finished, "step_no", -1) == -1:
                cb = getattr(self, "on_fabout", None)
                if cb is not None:
                    cb(finished)
            else:
                self.out_buffer.append(finished)
                # event-driven dispatch: new pickup arises -> wake the sleeping OHT dispatcher.
                _disp = getattr(self, "_dispatcher", None)
                if _disp is not None:
                    _disp.request_dispatch()

            # realized ready (time this machine actually finished) vs planned ready (gantt planned end)
            _realized_ready = round(self.env.now(), 3)
            _planned_ready = round(finished.planned_ready_time, 3)

            # gantt and machine scheduling — record execution event
            rec = {
                'sim_time': round(self.env.now(), 3),
                'node_name': self.node.name,
                'machine_no': getattr(self.node, 'machine_no', None),
                'machine_name': self.machine_name,
                'jssp_mach_id': (self.jssp_machine_id + 1) if self.jssp_machine_id is not None else None,
                'job_id': finished.job_id,
                'job_instance_id': finished.job_id,   # UI joins transport events on this
                'lot_id': finished.lot_id,
                'step_no': _finished_step,
                'job_type_id': finished.job_type_id,
                'op_index': finished.op_index,
                'product_type': finished.product_type,
                'start_time': round(_job_start, 3),
                'end_time': round(self.env.now(), 3),
                'process_time': round(ptime, 3),
                # ── learning label source ──────────────────────────────────────
                'planned_ready_time': _planned_ready,    # gantt planned end (= transport planned dispatch-out)
                'realized_ready_time': _realized_ready,  # actual end (= transport actual dispatch-out possible)
                'ready_deviation': round(_realized_ready - _planned_ready, 3),
            }
            self._sim_log.append(rec)
            append_machine_live_event(self.machine_live_state, rec)


class MachineBreakdown(sim.Component):
    """For disturbance robustness experiments: drives one machine's breakdown/repair.
    hold(exp(MTBF)) -> machine._broken=True -> hold(exp(MTTR)) -> recover & activate.
    Using an rng with the same seed gives a fair comparison across policies with the *same failure sequence*."""

    def setup(self, machine, mtbf, mttr, rng):
        self.machine = machine
        self.mtbf = float(mtbf)
        self.mttr = float(mttr)
        self.rng = rng

    def process(self):
        while True:
            yield self.hold(self.rng.expovariate(1.0 / self.mtbf))
            self.machine._broken = True
            yield self.hold(self.rng.expovariate(1.0 / self.mttr))
            self.machine._broken = False
            if self.machine.ispassive():
                self.machine.activate()
