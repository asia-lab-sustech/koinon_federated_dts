from __future__ import annotations
import time
import cvxpy as cp
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
import time
import math
import os
import csv
import json
from collections import Counter, deque

#except Exception:
#    cp = None

"""
Original version

"""

"""
Intersection-level Emergency Vehicle (EV) signal support agent.

This implementation follows the *structure* of Zhong & Chen (2022):
  Stage 1) Saturation reduction (DRRS -> green extension recommendation)
  Stage 2) Preemption:
      2a) Non-intrusive timing adjustment (QP feasibility check; reduced QP version)
      2b) Intrusive preemption (phase jump) using LJT/PST logic
  Stage 3) Restoration (optional LP to compensate timing offset; otherwise restore default program)

Notes for SUMO/TraCI practicality
- SUMO cannot directly "solve & install" a new full-cycle timing plan at once without replacing the program logic.
  This agent therefore applies actions through:
    * traci.trafficlight.setPhaseDuration()  (changes remaining time of the *current* phase)
    * traci.trafficlight.setPhase()          (intrusive phase jump)
    * traci.trafficlight.setProgram()        (restore default program)

- The "non-intrusive QP" in the paper adjusts multiple phase times. For middleware evaluation, this file
  implements a *reduced* QP feasibility check that matches what we can reliably actuate with TraCI:
    * shorten current phase remaining time (hurry)
    * extend target phase when it becomes active

If you want the full QP/LP from the paper, you can extend the variable set to all phases and install
a new ProgramLogic (requires SUMO API support and careful safety constraints).
"""


from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional, List, Tuple, Set
import random

# TraCI is expected to be available at runtime (SUMO loop started)
try:
    import traci
except Exception:  # allow import for static checks / docs
    traci = None  # type: ignore

# Optional optimization dependency
try:
    import cvxpy as cp  # type: ignore
except Exception:
    cp = None  # type: ignore


# =========================
# Messages / plans
# =========================

@dataclass
class EvRequest:
    """Message sent from EV/middleware -> IntersectionAgent."""
    ev_id: str
    sim_time: float
    erl_level: int          # 1..4 (1 highest emergency)
    speed_mps: float
    distance_to_intersection_m: float
    in_edge_id: str         # IMPORTANT: traci.vehicle.getRoadID(ev_id), not internal ':...'
    target_phase_idx: Optional[int] = None
    delta_sec: float = 2.0  # arrival window half-width
    route_intersections: Optional[List[str]] = None
    route_veh: Optional[List[str]] = None
    source_service: Optional[str] = None
    source_tag: Optional[str] = None
    delivery: Optional[str] = None


class PassiveIntersectionDT:
    """Context-only DT for a non-TLS route junction.

    It publishes queue/spillback observations but cannot reserve windows or
    actuate traffic lights. Active TLS agents can consume this context to avoid
    unsafe F2 overrides across unmanaged route gaps.
    """

    def __init__(
        self,
        node_id: str,
        observed_edges: Optional[List[str]] = None,
        route_edges: Optional[List[str]] = None,
        max_edges: int = 4,
    ) -> None:
        self.node_id = str(node_id)
        self.observed_edges = [str(e) for e in list(observed_edges or []) if str(e) and not str(e).startswith(":")]
        self.route_edges = [str(e) for e in list(route_edges or []) if str(e)]
        self.max_edges = max(1, int(max_edges or 4))
        self.role = "PassiveIntersectionObserver"
        self.can_actuate = False
        self.can_coordinate = False
        self.can_observe_downstream = True

    def _edge_snapshot(self, edge_id: str) -> Dict[str, object]:
        edge = str(edge_id or "")
        out: Dict[str, object] = {
            "edge": edge,
            "veh_n": 0,
            "halt_n": 0,
            "mean_speed_mps": -1.0,
            "occupancy_pct": 0.0,
            "lane_n": 0,
        }
        if traci is None or not edge or edge.startswith(":"):
            return out
        try:
            lane_n = int(traci.edge.getLaneNumber(edge))
        except Exception:
            lane_n = 0
        total_veh = 0
        total_halt = 0
        total_occ = 0.0
        speed_weighted = 0.0
        speed_weight = 0
        valid_lanes = 0
        for lane_idx in range(max(0, lane_n)):
            lane_id = f"{edge}_{lane_idx}"
            try:
                veh_n = int(traci.lane.getLastStepVehicleNumber(lane_id))
                halt_n = int(traci.lane.getLastStepHaltingNumber(lane_id))
                occ = float(traci.lane.getLastStepOccupancy(lane_id))
                mean_speed = float(traci.lane.getLastStepMeanSpeed(lane_id))
            except Exception:
                continue
            valid_lanes += 1
            total_veh += max(0, int(veh_n))
            total_halt += max(0, int(halt_n))
            total_occ += max(0.0, float(occ))
            if veh_n > 0 and math.isfinite(mean_speed):
                speed_weighted += float(mean_speed) * float(veh_n)
                speed_weight += int(veh_n)
        out["lane_n"] = int(valid_lanes)
        out["veh_n"] = int(total_veh)
        out["halt_n"] = int(total_halt)
        out["occupancy_pct"] = float(total_occ / max(1, valid_lanes))
        out["mean_speed_mps"] = float(speed_weighted / speed_weight) if speed_weight > 0 else -1.0
        return out

    def build_context(
        self,
        sim_time: float,
        ev_id: str,
        min_halt_n: int = 3,
        max_mean_speed_mps: float = 1.0,
        min_veh_n: int = 3,
        max_occupancy_pct: float = 35.0,
    ) -> Dict[str, object]:
        edges = list(self.observed_edges[: self.max_edges])
        payload: Dict[str, object] = {
            "schema": "federation.passive_intersection_context.v1",
            "provider_id": str(self.node_id),
            "provider_type": "passive_intersection",
            "node_id": str(self.node_id),
            "role": str(self.role),
            "can_actuate": False,
            "can_coordinate": False,
            "can_observe_downstream": True,
            "ev_id": str(ev_id),
            "sim_time": float(sim_time),
            "target_edges": list(edges),
            "lookahead_edges": list(edges),
            "edge_snapshots": [],
            "blocked": False,
            "reason": "clear",
            "worst_edge": "",
            "worst_edge_offset": -1,
            "max_halt_n": 0,
            "max_veh_n": 0,
            "max_occupancy_pct": 0.0,
            "min_mean_speed_mps": -1.0,
            "downstream_queue_margin_sec": 0.0,
            "downstream_spillback_risk": 0.0,
            "downstream_readiness_score": 1.0,
            "confidence": 0.85,
        }
        if not edges:
            payload["reason"] = "no_observed_edges"
            payload["confidence"] = 0.0
            return payload
        worst = None
        min_speed_seen: Optional[float] = None
        for offset, edge in enumerate(edges, start=1):
            snap = self._edge_snapshot(str(edge))
            veh_n = int(snap.get("veh_n", 0) or 0)
            halt_n = int(snap.get("halt_n", 0) or 0)
            occ = float(snap.get("occupancy_pct", 0.0) or 0.0)
            speed = float(snap.get("mean_speed_mps", -1.0) or -1.0)
            snap["offset"] = int(offset)
            try:
                payload["edge_snapshots"].append(dict(snap))  # type: ignore[union-attr]
            except Exception:
                pass
            payload["max_halt_n"] = max(int(payload["max_halt_n"]), int(halt_n))
            payload["max_veh_n"] = max(int(payload["max_veh_n"]), int(veh_n))
            payload["max_occupancy_pct"] = max(float(payload["max_occupancy_pct"]), float(occ))
            if speed >= 0.0:
                min_speed_seen = speed if min_speed_seen is None else min(float(min_speed_seen), float(speed))
            reasons: List[str] = []
            if halt_n >= int(min_halt_n):
                reasons.append("halting")
            if veh_n >= int(min_veh_n) and speed >= 0.0 and speed <= float(max_mean_speed_mps):
                reasons.append("low_speed")
            if occ >= float(max_occupancy_pct):
                reasons.append("occupancy")
            if reasons:
                score = (len(reasons), halt_n, occ, veh_n)
                if worst is None or score > worst[0]:
                    worst = (score, str(edge), int(offset), ",".join(reasons))
        payload["min_mean_speed_mps"] = -1.0 if min_speed_seen is None else float(min_speed_seen)
        if worst is not None:
            payload["blocked"] = True
            payload["worst_edge"] = str(worst[1])
            payload["worst_edge_offset"] = int(worst[2])
            payload["reason"] = str(worst[3])
        max_halt = int(payload["max_halt_n"])
        max_occ = float(payload["max_occupancy_pct"])
        spill = max(0.0, min(1.0, max(float(max_halt) / max(1.0, float(min_halt_n) * 2.0), max_occ / 100.0)))
        payload["downstream_spillback_risk"] = float(spill)
        payload["downstream_queue_margin_sec"] = float(-max(0, max_halt) * 2.0 if bool(payload["blocked"]) else 0.0)
        payload["downstream_readiness_score"] = float(max(0.0, min(1.0, 1.0 - spill)))
        return payload

@dataclass
class PreemptionPlan:
    """
    Control decision returned by the agent.

    plan_type:
      - "saturation_reduction": extend target green if already in target
      - "non_intrusive": QP-feasible mild timing adjustment (hurry +/or extend)
      - "intrusive": phase jump at jump_time_sec
      - "restore": restore default program (and optionally apply restoration schedule)
    """
    plan_type: str
    target_phase_idx: int

    extend_green_sec: float = 0.0
    hurry_current_phase_to_sec: Optional[float] = None  # remaining seconds for current phase
    jump_time_sec: Optional[float] = None
    jump_to_phase_idx: Optional[int] = None
    planned_green_window: Optional[Tuple[float, float]] = None
    # Paper-style non-intrusive plan: change green splits over one or more cycles
    phase_duration_overrides: Optional[Dict[int, float]] = None  # phase_idx -> total duration (sec)
    override_start_time_sec: Optional[float] = None  # when to begin applying overrides
    override_end_time_sec: Optional[float] = None    # when to stop applying overrides (safety)
    notes: str = ""

@dataclass
class OfferWeights:
    wait: float = 1.0
    queue: float = 0.25
    risk: float = 0.8
    intrusive: float = 1.2

@dataclass
class SignalWindowOffer:
    """Counterfactual ('what-if') offer returned by the intersection.

    This is intentionally *lightweight* and JSON-serialisable, so you can publish it via MQTT/REST.

    action:
      - "none": no actuation (baseline SPaT window)
      - "extend": extend target green by `ext` seconds (committed when target becomes current)
      - "hurry": shorten the current phase remaining time to `hurry_to` seconds
      - "jump": intrusive phase jump at `jump_time` (seconds in simulation time)
    """
    offer_id: str
    tls_id: str
    ev_id: str
    created_time: float

    #offer_weights: OfferWeights = field(default_factory = OfferWeights())

    target_phase_idx: int
    action: str
    action_params: Dict[str, float]

    # predicted green window under the action
    green_window: Tuple[float, float]
    # EV arrival window (t_i +/- delta)
    arrival_window: Tuple[float, float]

    # derived metrics to help EV/coordinator choose
    feasible: bool
    expected_wait_sec: float
    expected_miss_sec: float
    cost_to_others_veh_sec: float
    confidence: float
    

    # optional speed advice range to hit the green window
    speed_range_mps: Optional[Tuple[float, float]] = None

    def to_dict(self) -> Dict[str, object]:
        return {
            "offer_id": self.offer_id,
            "tls_id": self.tls_id,
            "ev_id": self.ev_id,
            "created_time": self.created_time,
            "target_phase_idx": self.target_phase_idx,
            "action": self.action,
            "action_params": dict(self.action_params),
            "green_window": list(self.green_window),
            "arrival_window": list(self.arrival_window),
            "feasible": self.feasible,
            "expected_wait_sec": self.expected_wait_sec,
            "expected_miss_sec": self.expected_miss_sec,
            "cost_to_others_veh_sec": self.cost_to_others_veh_sec,
            "confidence": self.confidence,
            "speed_range_mps": list(self.speed_range_mps) if self.speed_range_mps else None,
        }

@dataclass
class OfferSelection:
    offer_id: str
    ev_id: str
    sim_time: float
    chosen_speed_mps: Optional[float] = None



class AgentStage(Enum):
    NO_REQUEST = auto()
    BASELINE_VALIDATION = auto()
    SATURATION_REDUCTION = auto()
    PREEMPTION_NON_INTRUSIVE = auto()
    PREEMPTION = auto()
    RESTORATION = auto()

@dataclass(frozen=True)
class PhaseTemplate:
    idx: int
    duration: float
    state: str

@dataclass(frozen=True)
class TLSProgramTemplate:
    tls_id: str
    program_id: str
    phases: Tuple[PhaseTemplate, ...]
    cycle_sec: float
    captured_at_sim_time: float

# =========================
# Arrival rate estimator (edge-based EMA)
# =========================

class EdgeArrivalRateEMA:
    def __init__(self, alpha: float, dt: float):
        self.alpha = float(alpha)
        self.dt = float(dt)
        self.prev_ids: Dict[str, Set[str]] = {}
        self.ema: Dict[str, float] = {}

    def update(self, edge_id: str) -> float:
        if traci is None:
            return 0.0

        try:
            now_ids = set(traci.edge.getLastStepVehicleIDs(edge_id))
        except Exception:
            now_ids = set()
            try:
                n = traci.edge.getLaneNumber(edge_id)
                for i in range(int(n)):
                    now_ids |= set(traci.lane.getLastStepVehicleIDs(f"{edge_id}_{i}"))
            except Exception:
                pass

        old_ids = self.prev_ids.get(edge_id, set())
        new_arrivals = len(now_ids - old_ids)
        inst = new_arrivals / max(self.dt, 1e-6)

        prev = self.ema.get(edge_id, inst)
        ema = self.alpha * inst + (1.0 - self.alpha) * prev

        self.prev_ids[edge_id] = now_ids
        self.ema[edge_id] = ema
        return float(ema)

@dataclass
class NeighborInfo:
    tls_id: str
    via_out_edge: str
    neighbor_in_edge: str

@dataclass
class NextHopCandidate:
    neighbor_tls: str
    out_edge: str
    movement_id: str
    prob: float = 0.0
    soft_reserved: bool = False
    hard_reserved: bool = False
    eta_start: float = 0.0
    eta_end: float = 0.0
    ttl: float = 0.0

@dataclass
class FederationCache:
    # ev_id -> list of candidates
    next_hops: Dict[str, List[NextHopCandidate]] = field(default_factory=dict)
    # ev_id -> currently hard reserved neighbor
    hard_choice: Dict[str, str] = field(default_factory=dict)
    # reservation_id -> status/info
    reservations: Dict[str, dict] = field(default_factory=dict)
    # neighbor warm states (tls_id -> metadata)
    neighbor_warm: Dict[str, dict] = field(default_factory=dict)

@dataclass
class ReservationState:
    req_id: str
    ev_id: str
    from_tls: str
    to_tls: str
    in_edge_id: Optional[str]
    eta_start: float
    eta_end: float
    soft: bool
    hard: bool
    confidence: float
    ts_created: float
    ts_expire: float
    route_token: Optional[str] = None
    route_intersections_hint: Optional[List[str]] = None
    route_veh_hint: Optional[List[str]] = None
    preferred_next_tls: Optional[str] = None
    status: str = "PENDING"
    reason: str = ""
    # Downstream readiness snapshot at reservation time
    local_queue_margin_sec: float = 0.0
    local_spillback_risk: float = 0.0
    local_readiness_score: float = 0.0

@dataclass
class IntersectionAgentConfig:
    intersection_id: str
    tls_id: str
    dt_mode: str = "active_tls"  # active_tls | passive_observer
    can_actuate: bool = True
    can_coordinate: bool = True
    can_observe_downstream: bool = True

    # Tick cadence: paper updates ~1s. Your sim step might be 0.1s.
    decision_period_sec: float = 1.0

    # Paper-style constants (names align with paper)
    T_lost_sec: float = 5.0
    SIT_sec: float = 5.0
    YT_sec: float = 5.0
    GminS_sec: float = 7.0

    # Green bounds used by optimization / feasibility checks
    tau_min_sec: float = 7.0
    tau_max_sec: float = 200.0
    # Intrusive rescue is a local emergency actuation, not a full queue-clearing
    # recovery plan. Keep it bounded so F2 cannot turn a late rescue into a
    # long phase lock that outlives the EV passage.
    intrusive_hold_queue_cap_sec: float = 12.0
    intrusive_hold_max_sec: float = 30.0
    restore_remaining_clamp_enable: bool = True
    restore_remaining_extra_sec: float = 2.0

    # Non-intrusive reduced-QP actuation caps
    min_current_phase_remaining_sec: float = 0.5
    max_target_green_extension_sec: float = 40.0
    # DRRS-aligned boundary for decoding "extend" offers in fallback mode.
    sat_reduce_max_ext: float = 40.0

    # DRRS AHP weights (paper)
    w_erl: float = 0.1031
    w_clrs: float = 0.6053
    w_tul: float = 0.2915

    # DRRS clusters: (centroid, extension_time_sec)
    drrs_clusters: List[Tuple[float, float]] = field(default_factory=lambda: [
        (1.2273, 40.0),
        (1.6701, 30.0),
        (2.2333, 20.0),
        (2.7658, 10.0),
        (3.3414, 0.0),
    ])

    # Queue model (paper-like Q_i)
    arrival_ema_alpha: float = 0.3
    saturation_flow_per_lane_vehps: float = 0.5  # ~1800 veh/h/lane
    # Queue-metric stability / sensing options
    queue_arrival_first_sample_zero: bool = True
    queue_arrival_cap_ratio_to_s: float = 0.98
    queue_denom_min_vehps: float = 0.05
    queue_clear_time_cap_sec: float = 120.0
    queue_use_induction_loops: bool = False
    # Optional static mapping lane_id -> loop IDs. If empty and queue_use_induction_loops=True,
    # mapping is auto-discovered via traci.inductionloop.getLaneID.
    queue_loop_ids_by_lane: Dict[str, List[str]] = field(default_factory=dict)
    # Loop counting mode for arrival-rate estimation:
    # - "step": use per-step vehicle-ID delta (best when polling every sim-step)
    # - "interval": prefer detector interval counts (best when polling at detector freq, e.g., 1s)
    # - "adaptive": switch to interval counts when polling is sparse relative to sim step
    queue_loop_count_mode: str = "adaptive"
    queue_loop_detector_freq_sec: float = 1.0
    queue_loop_interval_min_poll_gap_sec: float = 0.5
    queue_loop_interval_sparse_factor: float = 1.5
    # Paper-grounded queue-clearing enrichment (Zhong Eq. (6)):
    #   delta_w = 1_i * (Q_i + (n_i + 1) * T_lost + YT)
    queue_metrics_enable_improved: bool = True
    # Strict paper-mode toggle for A/B:
    # - N: full EV-entrance-lane queue (no "ahead of EV only" filter)
    # - T_lost / YT: fixed config values (no dynamic estimators)
    # - A source: explicit gating by queue_use_induction_loops
    queue_metrics_paper_strict_mode: bool = False
    queue_metrics_use_dynamic_t_lost: bool = True
    queue_metrics_use_dynamic_yt: bool = True
    queue_metrics_t_lost_min_sec: float = 0.8
    queue_metrics_t_lost_max_sec: float = 6.0
    queue_metrics_yt_base_sec: float = 0.8
    queue_metrics_yt_per_halt_sec: float = 0.35
    queue_metrics_yt_per_moving_sec: float = 0.15
    queue_metrics_yt_speed_weight_sec: float = 0.8
    queue_metrics_yt_max_sec: float = 5.0
    queue_metrics_cycle_fallback_sec: float = 90.0

    # Middleware stress testing
    enable_volatile_connectivity: bool = False
    drop_prob: float = 0.0
    max_delay_sec: float = 0.0

    # Enable optimization checks (cvxpy optional)
    enable_non_intrusive_qp: bool = True
    enable_restoration_lp: bool = False

    # Paper-QP tuning
    qp_beta_green_dev: float = 1.0
    qp_n_max: int = 6
    qp_override_extra_sec: float = 10.0
    override_apply_grace_sec: float = 0.5

    # Stale-message policy
    stale_ev_after_sec: float = 5.0
    # EV pass/validation sensing:
    # - loop_touch: EV ID detected on induction loop(s) near stopline
    # - request_silence: EV stops sending requests for a short window
    ev_pass_use_loop_touch: bool = True
    ev_pass_use_request_silence: bool = True
    ev_pass_request_silence_sec: float = 1.0
    ev_pass_loop_touch_max_dist_m: float = 25.0
    ev_pass_min_loop_to_silence_sec: float = 0.2
    ev_pass_enable_debug: bool = True
    # Guard left-edge proxy so pass is not declared too early (far from stopline).
    ev_pass_left_edge_max_dist_m: float = 2.5
    # Ignore immediate re-arming messages for same EV after pass at one TLS.
    ev_pass_rearm_cooldown_sec: float = 8.0
    # Stronger post-pass guard: after a TLS has emitted a pass/cross event,
    # reject stale same-approach requests for the same EV. This prevents queued
    # MQTT/FNM messages from re-opening an already completed local episode.
    ev_pass_post_suppress_sec: float = 60.0
    ev_pass_post_suppress_max_dist_m: float = 120.0
    # Keep a short-lived handoff token even if EV session is cleared immediately.
    ev_handoff_pending_ttl_sec: float = 10.0

    # activation
    min_trigger_distance_m: float = 500.0

    # Trace tags for cross-run comparisons (set from real-world knobs)
    ev_request_source_tag: str = ""
    tls_signal_trace_enable: bool = False
    # Emit per-node queue/spillback snapshots during active EV handling.
    queue_snapshot_emit_enable: bool = True
    # Spillback active flag threshold for snapshot event.
    queue_snapshot_spillback_threshold: float = 0.2

    saturation_green_buffer_sec: float = 1.0      # extra safety buffer
    saturation_min_gap_to_act_sec: float = 0.5    # don't twitch for tiny gaps
    saturation_actuation_cooldown_sec: float = 1.0 # avoid rapid re-actuation
    enable_window_offers: bool = True

    # Offer ranking policy (used by F2).
    # - "legacy": original weighted score
    # - "improved_lexicographic": EV service first, then disruption, then intervention severity
    # - "robust_research": multi-objective EV/safety/traffic/federation score
    offer_score_strategy: str = "improved_lexicographic"
    offer_hard_wait_sec: float = 6.0
    offer_hard_miss_sec: float = 0.5
    offer_ev_epsilon_sec: float = 0.25
    offer_disruption_epsilon_vehs: float = 2.0

    # Numeric score terms (for reporting / compatibility with existing logs and sort calls).
    offer_w_wait: float = 1.0
    offer_w_miss: float = 4.0
    offer_w_queue: float = 0.25
    offer_w_severity: float = 1.2
    offer_w_magnitude: float = 0.05
    offer_w_conf_risk: float = 0.2

    # Offer-confidence / impact estimation knobs
    offer_conf_non_ev_scale_veh_sec: float = 300.0
    offer_conf_queue_scale_sec: float = 30.0
    offer_conf_impact_weight: float = 0.35
    offer_conf_non_ev_weight: float = 0.25
    offer_conf_queue_weight: float = 0.20
    offer_conf_window_weight: float = 0.20
    # Guardrail for F2: avoid federated offer overrides that significantly worsen EV-local ETA.
    f2_ev_guard_enable: bool = True
    f2_ev_guard_wait_penalty_sec: float = 2.0
    f2_ev_guard_miss_penalty_sec: float = 0.3
    f2_ev_guard_require_feasible: bool = True
    # F2 selection strategy:
    # - legacy_guard: threshold-based fallback guard
    # - measured: compare measurable objectives (feasibility + robust score)
    f2_selection_policy: str = "measured"
    # In measured mode, require a peer-refined candidate to beat the B1 local
    # anchor by a small measurable margin before it is allowed to override the
    # B1 floor. EV wait/miss improvements can also justify the override.
    f2_measured_override_min_robust_improvement: float = 0.0
    f2_measured_override_min_ev_wait_improvement_sec: float = 0.0
    f2_measured_override_min_ev_miss_improvement_sec: float = 0.0
    # Never actuate an infeasible selected offer when enabled.
    f2_block_infeasible_actuation: bool = True
    # Refinement gating: allow federation refine only when context quality is sufficient.
    f2_refine_require_feedback: bool = True
    f2_refine_feedback_max_age_sec: float = 6.0
    # Adaptive feedback freshness window by EV proximity:
    # stricter when near stopline, looser when far.
    f2_refine_feedback_age_adaptive_enable: bool = True
    f2_refine_feedback_max_age_near_sec: float = -1.0
    f2_refine_feedback_max_age_far_sec: float = -1.0
    f2_refine_feedback_adaptive_far_distance_m: float = 250.0
    # Bootstrap-safe feedback gating:
    # allow initial refine attempts before first downstream feedback exists,
    # then enforce normal feedback freshness gates.
    f2_refine_feedback_bootstrap_enable: bool = True
    f2_refine_feedback_bootstrap_distance_m: float = 450.0
    f2_refine_feedback_bootstrap_max_age_sec: float = 20.0
    # Guard refine against stale downstream phase snapshots (responder-side context age).
    f2_refine_stale_feedback_gate_enable: bool = True
    f2_refine_max_responder_phase_state_age_ms: float = 4000.0
    # Near-EV gate: require fresher responder phase state when EV is close to stopline.
    f2_refine_near_distance_m: float = 40.0
    f2_refine_near_max_responder_phase_state_age_ms: float = 1200.0
    # Optional handoff-aware gate: when EV is near, prefer feedback from top next-hop TLS.
    f2_refine_require_preferred_feedback_when_near: bool = True
    f2_refine_preferred_feedback_near_distance_m: float = 60.0
    # Optional fallback: use recent neighboring TLS live state when reservation
    # feedback is temporarily missing, still freshness-gated.
    f2_refine_neighbor_state_fallback_enable: bool = True
    f2_refine_neighbor_state_max_age_sec: float = 4.0
    f2_refine_neighbor_state_near_max_age_sec: float = 1.5
    f2_refine_require_loop_coverage: bool = True
    f2_refine_min_loop_coverage_ratio: float = 0.5
    # Avoid noisy, repeated fallback applies in F2 when selected plan is unchanged.
    f2_skip_redundant_apply: bool = True
    f2_skip_redundant_apply_min_interval_sec: float = 0.8
    # Distance-aware redundant-apply gate (near stopline can be more permissive).
    f2_skip_redundant_apply_min_interval_near_sec: float = 0.8
    f2_skip_redundant_apply_min_interval_far_sec: float = 0.8
    f2_skip_redundant_apply_near_distance_m: float = 120.0
    f2_skip_redundant_apply_far_distance_m: float = 300.0
    # Active coordination window relax: reduce dedupe conservatism only when
    # federation coordination is actively progressing for the EV.
    f2_active_coord_window_relax_enable: bool = False
    f2_active_coord_window_recent_sec: float = 2.5
    f2_active_coord_window_ev_near_m: float = 180.0
    f2_active_coord_window_min_active_reservations: int = 1
    f2_active_coord_window_interval_scale: float = 0.50
    # Graceful near-stopline degrade: if near-feedback gates repeatedly block refine,
    # temporarily run local-only (B1-equivalent) for a short cooldown window.
    f2_refine_local_cooldown_enable: bool = True
    f2_refine_local_cooldown_trigger_count: int = 3
    f2_refine_local_cooldown_window_sec: float = 2.5
    # <=0: reuse f2_refine_near_distance_m
    f2_refine_local_cooldown_near_distance_m: float = -1.0
    # Reduce hard-request thrash when confidence hovers around threshold.
    f2_hard_req_skip_cooldown_enable: bool = True
    f2_hard_req_skip_streak_trigger: int = 4
    f2_hard_req_skip_streak_window_sec: float = 3.0
    f2_hard_req_skip_cooldown_sec: float = 2.5
    f2_hard_req_cooldown_escape_margin: float = 0.08
    # When hard requests keep failing threshold checks, fail-soft to local offer
    # to avoid degrading EV progress with unstable federation nudges.
    f2_hard_skip_failsoft_enable: bool = True
    f2_hard_skip_failsoft_streak_trigger: int = 3
    f2_hard_skip_failsoft_near_only: bool = True
    f2_hard_skip_failsoft_near_distance_m: float = 120.0
    # Usefulness gate: if federated hard path repeatedly fails, hold refine and
    # run stable local behavior for a short window.
    f2_usefulness_gate_enable: bool = True
    f2_usefulness_gate_skip_streak_trigger: int = 6
    f2_usefulness_gate_hold_sec: float = 3.0
    f2_usefulness_gate_near_only: bool = True
    f2_usefulness_gate_near_distance_m: float = 150.0
    f2_usefulness_gate_require_no_hard_accept: bool = True
    f2_usefulness_gate_failsoft_local: bool = True

    # Optional Drone-DT downstream context requester. This is disabled for
    # plain B0/B1/F2 so drone-augmented runs remain explicit experiment modes.
    f2_drone_context_request_enable: bool = False
    f2_drone_context_provider_id: str = "crazyflie_01"
    f2_drone_context_capability: str = "downstream_context_provider"
    f2_drone_context_request_ttl_sec: float = 3.0
    f2_drone_context_request_min_interval_sec: float = 3.0
    f2_drone_context_request_max_edges: int = 8
    f2_drone_context_include_route_context: bool = True
    f2_drone_context_route_context_max_edges: int = 64
    f2_drone_context_request_on_no_fresh_peer: bool = True
    f2_drone_context_request_on_no_candidate: bool = True
    f2_drone_context_request_on_stale_feedback: bool = True
    f2_drone_context_request_on_low_loop_coverage: bool = True
    f2_drone_context_emit_discovery_query: bool = True
    f2_drone_context_discovery_gate_enable: bool = False
    f2_drone_context_discovery_cache_ttl_sec: float = 5.0
    f2_drone_context_discovery_query_min_interval_sec: float = 1.0
    # F2D-only proactive route scouting. This remains disabled by default so
    # B0/B1/F2/F2P and reactive F2D behavior are unchanged unless explicitly enabled.
    f2d_drone_prescout_enable: bool = False
    f2d_drone_prescout_first_tls_only: bool = True
    f2d_drone_prescout_max_edges: int = 16
    f2d_drone_prescout_min_interval_sec: float = 30.0

    # F2P passive non-TLS context fusion. The strict default makes passive DTs
    # a missing-context rescue signal, not a general override of active TLS peers.
    f2p_passive_context_policy: str = "immediate_missing_severe"  # disabled | missing_feedback_only | severe_or_missing | immediate_missing_severe | always
    f2p_passive_context_max_age_sec: float = 5.0
    f2p_passive_context_lookahead_edges: int = 4
    f2p_passive_context_max_worst_edge_offset: int = 1
    f2p_passive_context_severe_min_halt_n: int = 4
    f2p_passive_context_severe_min_veh_n: int = 6
    f2p_passive_context_severe_max_mean_speed_mps: float = 0.5
    f2p_passive_context_severe_max_occupancy_pct: float = 45.0
    # F2P must remain an opportunistic extension of F2. When passive context is
    # only replacing missing active peer feedback, cap its scoring penalty so a
    # passive observer cannot make F2P more conservative than the F2 floor.
    f2p_passive_context_missing_feedback_floor_enable: bool = True
    f2p_passive_context_missing_feedback_max_queue_deficit_sec: float = 2.0
    f2p_passive_context_missing_feedback_max_spillback_risk: float = 0.15
    f2p_passive_context_missing_feedback_max_timing_sec: float = 1.0
    # Clear passive reports are still useful: they reduce the uncertainty of
    # a missing active peer response without granting full confidence.
    f2p_passive_context_clear_missing_feedback_enable: bool = True
    f2p_passive_context_clear_missing_feedback_no_feedback_penalty: float = 0.25

    # Early dedupe for offer applies, before plan->TLS actuation path.
    f2_offer_preapply_dedupe_enable: bool = True
    f2_offer_preapply_dedupe_min_interval_sec: float = 2.0
    # Distance-aware pre-apply dedupe gate for selected offers.
    f2_offer_preapply_dedupe_min_interval_near_sec: float = 2.0
    f2_offer_preapply_dedupe_min_interval_far_sec: float = 2.0
    f2_offer_preapply_dedupe_near_distance_m: float = 120.0
    f2_offer_preapply_dedupe_far_distance_m: float = 300.0
    # Adaptive hard-thresholding by EV proximity and local offer quality.
    f2_hard_threshold_adaptive_enable: bool = True
    f2_hard_threshold_near_distance_m: float = 120.0
    f2_hard_threshold_far_distance_m: float = 300.0
    f2_hard_threshold_near_delta: float = -0.08
    f2_hard_threshold_far_delta: float = 0.04
    f2_hard_threshold_min: float = 0.35
    f2_hard_threshold_max: float = 0.90
    f2_hard_threshold_quality_relax_enable: bool = True
    f2_hard_threshold_quality_relax_delta: float = 0.05
    f2_hard_threshold_quality_relax_ev_cost_margin_sec: float = 1.5

    # Robust research score configuration (plugable).
    robust_hard_miss_prob: float = 0.35
    robust_hard_queue_margin_sec: float = -0.5
    robust_hard_ttc_sec: float = 2.0
    robust_queue_unstable_sec: float = 600.0
    robust_require_speed_advice_feasible: bool = False
    robust_ttc_step_sec: float = 0.1
    robust_ttc_threshold_sec: float = 5.0
    robust_fed_age_window_sec: float = 15.0

    robust_scale_wait_sec: float = 10.0
    robust_scale_late_sec: float = 10.0
    robust_scale_non_ev_veh_sec: float = 400.0
    robust_scale_control_effort: float = 40.0
    robust_scale_queue_margin_sec: float = 20.0
    robust_scale_speed_risk: float = 1.0
    robust_scale_fed_age_sec: float = 15.0
    robust_scale_fed_active: float = 5.0

    robust_w_cov: float = 1.0
    robust_w_wait: float = 1.0
    robust_w_miss_prob: float = 3.0
    robust_w_late: float = 1.0
    robust_w_queue_margin: float = 1.5
    robust_w_non_ev: float = 1.0
    robust_w_spill_max: float = 1.0
    robust_w_spill_mean: float = 0.5
    robust_w_effort: float = 0.3
    robust_w_ttc: float = 2.0
    robust_w_tet: float = 0.5
    robust_w_tit: float = 0.5
    robust_w_speed_risk: float = 1.0
    robust_w_fed_reject: float = 0.75
    robust_w_fed_age: float = 0.25
    robust_w_fed_active: float = 0.1
    robust_w_fed_accept: float = 0.2
    robust_w_fed_down_queue: float = 1.2
    robust_w_fed_down_spill: float = 1.2
    robust_w_fed_down_timing: float = 0.8
    robust_w_fed_no_feedback: float = 0.25
    robust_w_conf_risk: float = 0.15
    robust_scale_fed_down_queue_sec: float = 20.0
    robust_scale_fed_down_timing_sec: float = 10.0
    robust_fed_resp_max_age_sec: float = 15.0
    robust_fed_down_hard_queue_margin_sec: float = -2.0
    robust_fed_down_hard_spillback: float = 0.85

    # Decision logging (CSV)
    enable_decision_csv_log: bool = True
    decision_log_csv_path: str = "/tmp/intersection_decisions.csv"
    decision_log_run_label: str = ""
    # Structured federation events (JSONL) for metrics extraction.
    enable_fed_event_jsonl: bool = True
    fed_event_jsonl_path: str = ""
    # Compact runtime line for paper variables at decision time
    enable_compact_decision_debug: bool = True
    # Compact runtime line for queue model variables
    enable_queue_metrics_debug: bool = True
    # EV runtime state traces at plan/offer evaluation time
    enable_ev_state_debug: bool = True
    ev_state_debug_on_offer_calc: bool = True
    # Offer-metric enhancements/debugging
    enable_improved_offer_metrics: bool = True
    enable_offer_metric_components_debug: bool = True
    enable_offer_metric_calibration: bool = True
    offer_metric_calibration_csv_path: str = "/tmp/offer_metric_calibration.csv"
    offer_metric_q_scale_sec: float = 60.0
    offer_metric_rho_scale: float = 0.8
    offer_metric_delta_min_sec: float = 1.0
    offer_metric_delta_max_sec: float = 8.0
    offer_metric_no_action_q_threshold_sec: float = 3.0
    offer_metric_no_action_cost_penalty_veh_sec: float = 35.0
    offer_metric_cost_pressure_weight: float = 0.60
    offer_metric_cost_arrival_weight: float = 0.40
    offer_metric_speed_max_accel_mps2: float = 2.5
    offer_metric_speed_max_decel_mps2: float = 3.5
    # Improved offer metric behavior controls:
    # - keep paper arrival window [t_i-delta, t_i+delta] by default
    # - only shift to t_i_eff when explicitly enabled
    offer_metric_use_paper_queue_clearing: bool = True
    offer_metric_use_t_eff: bool = False
    offer_metric_use_delta_scaling: bool = False
    offer_metric_queue_risk_bias_enable: bool = False

    # config-ish defaults (can move to cfg)
    fed_soft_ttl_sec = 20.0
    fed_hard_ttl_sec = 12.0
    fed_min_hard_conf = 0.70
    fed_min_hard_conf_with_hint: float = 0.50
    fed_warm_horizon_sec = 25.0
    fed_refine_period_sec = 0.5
    fed_hard_min_queue_margin_sec: float = -0.5
    fed_hard_max_spillback_risk: float = 0.85
    fed_route_hint_lookahead_hops: int = 4
    fed_route_hint_decay: float = 0.65
    fed_route_hint_weight: float = 6.0
    fed_nexttls_weight: float = 10.0
    fed_nextedge_weight: float = 8.0
    fed_tailroute_weight: float = 4.0
    fed_uniform_prior_weight: float = 1.0
    fed_route_hint_strong_prob: float = 0.55
    fed_enable_route_intersections_hint: bool = True
    fed_enable_nexttls_hint: bool = True
    fed_enable_nextedge_hint: bool = True
    # Phase C: coordination churn control at reservation publish path.
    fed_req_send_min_gap_sec: float = 0.60
    # Distance-aware req publish pacing (smaller gaps allowed when EV is near).
    fed_req_send_min_gap_near_sec: float = 0.60
    fed_req_send_min_gap_far_sec: float = 0.60
    fed_req_send_min_gap_near_distance_m: float = 120.0
    fed_req_send_min_gap_far_distance_m: float = 300.0
    fed_req_pending_per_peer_cap: int = 2
    fed_req_pending_stale_sec: float = 6.0
    # Reservation feasibility tolerance knobs.
    fed_min_hard_overlap_sec: float = 0.50
    fed_hard_overlap_grace_sec: float = 0.80
    fed_soft_window_grace_sec: float = 6.00
    # Adaptive rescue for hard near-miss windows (safety-gated by readiness).
    fed_hard_window_adaptive_relax_enable: bool = False
    fed_hard_window_adaptive_extra_grace_sec: float = 0.60
    fed_hard_window_adaptive_conf_min: float = 0.65
    fed_hard_window_adaptive_readiness_min: float = 0.55
    fed_hard_window_adaptive_spillback_max: float = 0.80
    fed_hard_window_adaptive_queue_margin_min_sec: float = -1.5
    # Use improved paper-grounded queue-clearing metrics in downstream readiness checks.
    fed_readiness_use_improved_queue: bool = True
    # Warmup dampening knobs for F2 stability during startup/transients.
    fed_warmup_enable_in_f2: bool = False
    fed_warmup_min_sim_time_sec: float = 10.0
    fed_warmup_max_apply_per_ev: int = 1
    _next_refine_time = 0.0

    # Baseline gate: if both are low, keep default program (no actuation)
    baseline_clrs_max: int = 2
    baseline_tul_max: int = 1
    saturation_to_preempt_gap_sec: float = 30.0

    # Intrusive fallback guardrails
    intrusive_distance_guard_m: float = 25.0
    intrusive_disturbance_min: float = 1.0

    # Defer non-intrusive/saturation extension until useful (avoid early over-extension)
    extension_commit_horizon_sec: float = 8.0
    extension_commit_distance_m: float = 50.0

    lookahead_horizon_sec: float = 600.0
    

# =========================
# Arrival rate estimator (lane-based EMA) - Option 2 (paper)
# =========================

class LaneArrivalRateEMA:
    """Estimate per-lane arrival rate A_i (veh/s) as an EMA of new vehicle IDs entering the lane.

    This matches the spirit of the paper's option-2 queue model, where the queue is defined on the
    EV entrance lane (not "vehicles ahead of EV").
    """
    def __init__(self, alpha: float, dt: float, first_sample_zero: bool = True):
        self.alpha = float(alpha)
        self.dt = float(dt)
        self.first_sample_zero = bool(first_sample_zero)
        self.prev_ids: Dict[str, Set[str]] = {}
        self.ema: Dict[str, float] = {}
        # Per-lane same-timestep cache (avoid double-updating EMA within one sim step)
        self.last_update_time: Dict[str, float] = {}
        self.last_value: Dict[str, float] = {}

    def update(
        self,
        lane_id: str,
        sim_time: Optional[float] = None,
        inst_count: Optional[float] = None,
    ) -> float:
        if sim_time is not None:
            prev_t = self.last_update_time.get(lane_id)
            if prev_t is not None and abs(float(prev_t) - float(sim_time)) <= 1e-9:
                return float(self.last_value.get(lane_id, self.ema.get(lane_id, 0.0)))

        if traci is None:
            return 0.0

        if inst_count is not None:
            inst = max(0.0, float(inst_count)) / max(self.dt, 1e-6)
        else:
            try:
                now_ids = set(traci.lane.getLastStepVehicleIDs(lane_id))
            except Exception:
                now_ids = set()

            old_ids = self.prev_ids.get(lane_id)
            if old_ids is None:
                # Bootstrap without an artificial spike from pre-existing vehicles.
                self.prev_ids[lane_id] = now_ids
                ema0 = 0.0 if self.first_sample_zero else (len(now_ids) / max(self.dt, 1e-6))
                self.ema[lane_id] = float(ema0)
                if sim_time is not None:
                    self.last_update_time[lane_id] = float(sim_time)
                    self.last_value[lane_id] = float(ema0)
                return float(ema0)

            new_arrivals = len(now_ids - old_ids)
            inst = new_arrivals / max(self.dt, 1e-6)
            self.prev_ids[lane_id] = now_ids

        prev = self.ema.get(lane_id, 0.0)
        ema = self.alpha * inst + (1.0 - self.alpha) * prev

        self.ema[lane_id] = ema
        if sim_time is not None:
            self.last_update_time[lane_id] = float(sim_time)
            self.last_value[lane_id] = float(ema)
        return float(ema)

    def update_many(
        self,
        lane_ids: List[str],
        sim_time: Optional[float] = None,
        inst_counts: Optional[Dict[str, float]] = None,
    ) -> float:
        total = 0.0
        for lid in lane_ids:
            cnt = None if inst_counts is None else inst_counts.get(lid)
            total += float(self.update(lid, sim_time=sim_time, inst_count=cnt))
        return float(total)


# =========================
# Intersection Agent
# =========================

class IntersectionAgent:
    def __init__(self, cfg: IntersectionAgentConfig, step_length_sec: float = 0.1):
        self.cfg = cfg
        self.current_phase = None
        self.stage: AgentStage = AgentStage.NO_REQUEST
        self.active_ev: Optional[EvRequest] = None
        self.last_ev_msg_time: Optional[float] = None

        self.current_plan: Optional[PreemptionPlan] = None
        self._last_f2_primary_plan: Optional[PreemptionPlan] = None
        self.ev_passed: bool = False

        # On-road status, when EV is being tracked   
        self.on_road: bool = True

        # For deciding at 1s cadence even if sim step is 0.1
        self._next_decision_time: float = 0.0

        # Arrival estimator for Q_i
        self.arrivals = EdgeArrivalRateEMA(alpha=cfg.arrival_ema_alpha, dt=step_length_sec)

        # Lane-level arrivals for the paper's queue model (option 2)
        self.lane_arrivals = LaneArrivalRateEMA(
            alpha=cfg.arrival_ema_alpha,
            dt=step_length_sec,
            first_sample_zero=bool(getattr(cfg, "queue_arrival_first_sample_zero", True)),
        )
        self._lane_loop_ids: Dict[str, List[str]] = {}
        self._lane_loops_ready: bool = False

        # Active non-intrusive timing overrides (paper-style QP), applied as phases become current.
        self._active_phase_overrides: Optional[Dict[int, float]] = None
        self._override_start_time_sec: float = 0.0
        self._override_end_time_sec: float = 0.0
        self._override_applied_in_current_phase: bool = False
        self._override_last_seen_phase: Optional[int] = None
        self._phase_change_time_sec: float = 0.0

        # Cache TLS program basics
        self.default_program_id: Optional[str] = None
        self._inbound_edge_to_phase: Dict[str, int] = {}
        self._movement_edge_to_phase: Dict[Tuple[str, str], int] = {}

        # EV passed detection
        self._prev_dist: Optional[float] = None
        self._was_close: bool = False
        self._ev_pred_ti_first: Optional[float] = None
        self._ev_pred_ti_last: Optional[float] = None
        self._ev_loop_touch_time: Optional[float] = None
        self._ev_loop_touch_loop_id: Optional[str] = None
        self._ev_touch_reported: bool = False
        self._ev_request_silence_time: Optional[float] = None
        self._ev_left_approach_time: Optional[float] = None
        self._ev_pass_time_est: Optional[float] = None
        self._ev_pass_detect_time: Optional[float] = None
        self._ev_pass_proxy_time: Optional[float] = None
        self._ev_left_approach_from_edge: Optional[str] = None
        self._ev_left_approach_to_edge: Optional[str] = None
        self._ev_last_seen_road_id: Optional[str] = None
        self._ev_pass_reason: str = ""
        self.last_ev_validation: Dict[str, object] = {}
        self._recent_pass_time_by_ev: Dict[str, float] = {}
        self._recent_pass_info_by_ev: Dict[str, Dict[str, object]] = {}
        self._pending_handoff_ev_id: Optional[str] = None
        self._pending_handoff_time: Optional[float] = None
        # Keep latest route hints per EV even after active session clears.
        self._last_route_intersections_by_ev: Dict[str, List[str]] = {}
        self._last_route_edges_by_ev: Dict[str, List[str]] = {}

        # Restoration helpers (optional)
        self._timing_offset_sec: float = 0.0  # + means we lengthened timings; - means shortened
        self._restoration_schedule: Optional[Dict[int, float]] = None
        self._restoration_applied_phases: Set[int] = set()
        self._restore_program_applied_for_session: bool = False
        #self._last_seen_phase: Optional[int] = None

        self._last_seen_phase = None
        self._last_apply_signature: Optional[Tuple] = None
        self._last_apply_sim_time: float = -1e9
        self._last_apply_source: str = ""
        self._last_tls_signal_state: Optional[str] = None
        self._last_tls_signal_next_switch: Optional[float] = None
        self._last_tls_signal_change_sim_time: Optional[float] = None
        self._sat_last_actuation_time = -1e9

        # --- Federation cache ---
        self.neighbor_map: Dict[str, NeighborInfo] = {}   # neighbor_tls_id -> NeighborInfo
        self.out_edge_to_neighbor: Dict[str, str] = {}    # out_edge -> neighbor_tls_id
        self.active_reservations: Dict[str, ReservationState] = {}  # key: f"{ev_id}:{to_tls}"
        self.resp_cache: Dict[str, str] = {}              # req_id -> ACCEPTED/REJECTED/...
        self.last_handoff_by_ev: Dict[str, float] = {}    # ev_id -> ssim_time

        self.out_edge_to_neighbor, self.neighbor_map = self._build_outgoing_neighbor_map()
        self.neighbour_map_federation: Dict[str, NeighborInfo] = {}
        self._queue_debug_last_t_by_edge: Dict[str, float] = {}
        self._loop_debug_last_t_by_edge: Dict[str, float] = {}
        # Last loop observations for quick runtime inspection
        self.last_loop_lanes: List[str] = []
        self.last_loop_counts_by_lane: Dict[str, float] = {}
        self.last_loop_counts_cum_by_lane: Dict[str, float] = {}
        self._loop_counts_cum_by_lane: Dict[str, float] = {}
        self.last_loop_ids_by_lane: Dict[str, List[str]] = {}
        self.last_loop_source: str = "unknown"
        self.last_loop_error: str = ""
        # Loop-entry tracking (count unique vehicles entering detector, not occupancy every sub-step)
        self._loop_prev_vehicle_ids_by_loop: Dict[str, Set[str]] = {}
        self._loop_entry_cache_time: Optional[float] = None
        self._loop_entry_cache_by_loop: Dict[str, float] = {}
        self._loop_last_read_time_by_loop: Dict[str, float] = {}
        # Prevent cumulative double-increment when multiple readers call in same sim tick
        self._loop_cum_last_update_t_by_lane: Dict[str, float] = {}

        # federation runtime caches
        self.active_reservations: Dict[str, ReservationState] = {}
        self.resp_cache: Dict[str, dict] = {}
        self.last_handoff_by_ev: Dict[str, float] = {}
        self._fed_outbox: List[Tuple[str, dict]] = []
        self._drone_context_req_recent: Dict[Tuple[str, str, str], float] = {}
        self._drone_context_req_seq: int = 0
        self._drone_prescout_sent_by_ev_tls: Dict[Tuple[str, str], float] = {}
        self._drone_provider_discovery_cache: Dict[str, Dict[str, object]] = {}
        self._drone_discovery_query_recent: Dict[Tuple[str, str, str], float] = {}

        self._next_refine_time: float = 0.0
        self._last_soft_sent: Dict[Tuple[str, str], Tuple[float, float, float]] = {}  # (ev,to_tls)->(ts,eta_mid,p)
        self._last_hard_sent: Dict[str, Tuple[str, float, float]] = {}  # ev_id -> (to_tls, ts, p)

        # knobs (safe defaults if cfg doesn't have these)
        self.fed_refine_period_sec = float(getattr(self.cfg, "fed_refine_period_sec", 1.0))
        self.fed_soft_ttl_sec = float(getattr(self.cfg, "fed_soft_ttl_sec", 8.0))
        self.fed_hard_ttl_sec = float(getattr(self.cfg, "fed_hard_ttl_sec", 4.0))
        self.fed_min_hard_conf = float(
            getattr(self.cfg, "fed_min_hard_conf", getattr(self.cfg, "hard_reserve_conf_threshold", 0.65))
        )
        self.fed_min_hard_conf_with_hint = float(getattr(self.cfg, "fed_min_hard_conf_with_hint", 0.50))
        self.fed_soft_topk = int(getattr(self.cfg, "fed_soft_topk", 3))
        self.fed_eta_half_window_soft = float(getattr(self.cfg, "fed_eta_half_window_soft", 3.0))
        self.fed_eta_half_window_hard = float(getattr(self.cfg, "fed_eta_half_window_hard", 1.8))
        self.fed_min_repeat_sec = float(getattr(self.cfg, "fed_min_repeat_sec", 0.8))
        self.fed_eta_resend_thresh_sec = float(getattr(self.cfg, "fed_eta_resend_thresh_sec", 1.0))
        self.fed_prob_resend_thresh = float(getattr(self.cfg, "fed_prob_resend_thresh", 0.12))
        self.fed_req_send_min_gap_sec = float(getattr(self.cfg, "fed_req_send_min_gap_sec", 0.60))
        self.fed_req_pending_per_peer_cap = int(getattr(self.cfg, "fed_req_pending_per_peer_cap", 2))
        self.fed_req_pending_stale_sec = float(getattr(self.cfg, "fed_req_pending_stale_sec", 6.0))
        self.fed_route_hint_lookahead_hops = int(getattr(self.cfg, "fed_route_hint_lookahead_hops", 4))
        self.fed_route_hint_decay = float(getattr(self.cfg, "fed_route_hint_decay", 0.65))
        self.fed_route_hint_weight = float(getattr(self.cfg, "fed_route_hint_weight", 6.0))
        self.fed_nexttls_weight = float(getattr(self.cfg, "fed_nexttls_weight", 10.0))
        self.fed_nextedge_weight = float(getattr(self.cfg, "fed_nextedge_weight", 8.0))
        self.fed_tailroute_weight = float(getattr(self.cfg, "fed_tailroute_weight", 4.0))
        self.fed_uniform_prior_weight = float(getattr(self.cfg, "fed_uniform_prior_weight", 1.0))
        self.fed_route_hint_strong_prob = float(getattr(self.cfg, "fed_route_hint_strong_prob", 0.55))
        
        self.enable_federation_debug = bool(getattr(self.cfg, "enable_federation_debug", False))
        self.fed_force_route_hint_top1 = bool(getattr(self.cfg, "fed_force_route_hint_top1", False))
        self.fed_route_hint_prob_floor = float(getattr(self.cfg, "fed_route_hint_prob_floor", 0.80))
        self.fed_debug_log_path = str(getattr(self.cfg, "fed_debug_log_path", "") or "")
        self.fed_debug_print = bool(getattr(self.cfg, "fed_debug_print", True))
        self.enable_fed_event_jsonl = bool(getattr(self.cfg, "enable_fed_event_jsonl", True))
        self.fed_event_jsonl_path = str(getattr(self.cfg, "fed_event_jsonl_path", "") or "")
        if (not self.fed_event_jsonl_path) and self.fed_debug_log_path:
            _fed_base, _fed_ext = os.path.splitext(self.fed_debug_log_path)
            self.fed_event_jsonl_path = f"{_fed_base}.events.jsonl"
        self._fed_req_sent_ts: Dict[str, float] = {}
        self._fed_req_sent_clock: Dict[str, Dict[str, object]] = {}
        self._fed_req_recent_by_peer: Dict[str, deque] = {}
        self._fed_outbox_depth_peak: int = 0
        self._fed_run_id: str = str(getattr(self.cfg, "fed_run_id", "") or "")
        self._fed_topic_namespace: str = str(getattr(self.cfg, "fed_topic_namespace", "") or "")
        self._fed_evt_write_ok_logged: bool = False
        self._b1_edge_map_logged: Set[str] = set()
        self._b1_map_dumped: bool = False
        self._last_tick_compute_ms: float = 0.0
        self._last_refine_compute_ms: float = 0.0
        self._last_apply_compute_ms: float = 0.0
        self._session_event_counts: Counter = Counter()
        self._session_reason_counts: Counter = Counter()

        self.default_tls_program: Optional[TLSProgramTemplate] = None
        self._default_duration_by_phase: Dict[int, float] = {}
        self._offer_plan_cache: Dict[str, PreemptionPlan] = {}
        self._offer_metric_components_by_offer: Dict[str, Dict[str, float]] = {}
        self._offer_metric_selected_prediction_by_ev: Dict[str, Dict[str, float]] = {}
        self._offer_metric_error_table: Dict[Tuple[str, str, str], Dict[str, float]] = {}
        self._offer_metric_calibration_header_written: bool = False
        self._federation_warmup_ev: Optional[EvRequest] = None
        self._federation_warmup_valid_until: float = -1.0
        self._next_warmup_time: float = 0.0
        self._fed_warmup_disabled_reported: bool = False
        self.neighbor_state_cache: Dict[str, dict] = {}
        self.passive_context_cache: Dict[str, dict] = {}
        self._corridor_advice_by_ev_target: Dict[Tuple[str, str], Dict[str, object]] = {}
        self._corridor_verdict_by_req_id: Dict[str, Dict[str, object]] = {}
        self._corridor_verdict_by_ev: Dict[str, Dict[str, object]] = {}
        self._f2_near_gate_streak_by_ev: Dict[str, int] = {}
        self._f2_local_cooldown_until_by_ev: Dict[str, float] = {}
        self._f2_usefulness_hold_until_by_ev: Dict[str, float] = {}
        self._warmup_apply_count_by_ev: Dict[str, int] = {}
        self._hard_req_skip_tracker: Dict[Tuple[str, str], Dict[str, Any]] = {}
        self._last_offer_apply_signature_by_ev: Dict[str, Tuple[Any, ...]] = {}
        self._last_offer_apply_sim_time_by_ev: Dict[str, float] = {}
        self._f2_selected_offer_source_by_id: Dict[str, str] = {}

    def _decision_log_fields(self) -> List[str]:
        return [
            "wall_time",
            "sim_time",
            "run_label",
            "tls_id",
            "intersection_id",
            "event",
            "decision_source",
            "stage",
            "plan_type",
            "plan_target_phase",
            "plan_extend_sec",
            "plan_hurry_to_sec",
            "plan_jump_time_sec",
            "plan_jump_to_phase",
            "offer_id",
            "offer_action",
            "offer_score",
            "offer_feasible",
            "offer_wait_sec",
            "offer_miss_sec",
            "offer_cost_veh_sec",
            "offer_confidence",
            "offer_green_window_start",
            "offer_green_window_end",
            "offer_arrival_window_start",
            "offer_arrival_window_end",
            "note",
        ]

    def _print_compact_decision_debug(
        self,
        sim_time: float,
        mode: str,
        L_t: Optional[float] = None,
        R: Optional[float] = None,
        Q_i: Optional[float] = None,
        LJT: Optional[float] = None,
        PST: Optional[float] = None,
        D: Optional[float] = None,
        jump_time: Optional[float] = None,
    ) -> None:
        if not bool(getattr(self.cfg, "enable_compact_decision_debug", False)):
            return

        def _fmt(v: Optional[float]) -> str:
            return "NA" if v is None else f"{float(v):.2f}"

        print(
            "[DECISION_DEBUG] "
            f"tls={self.cfg.tls_id} "
            f"stage={self.stage.name} "
            f"mode={mode} "
            f"t={float(sim_time):.2f} "
            f"L_t={_fmt(L_t)} "
            f"R={_fmt(R)} "
            f"Q_i={_fmt(Q_i)} "
            f"LJT={_fmt(LJT)} "
            f"PST={_fmt(PST)} "
            f"D={_fmt(D)} "
            f"jump={_fmt(jump_time)}"
        )

    def _print_queue_metrics_debug(
        self,
        sim_time: float,
        edge_id: str,
        lanes: List[str],
        N: float,
        A_raw: float,
        A_used: float,
        S: float,
        Q: float,
        source: str,
    ) -> None:
        if not bool(getattr(self.cfg, "enable_queue_metrics_debug", False)):
            return
        key = str(edge_id)
        t_now = float(sim_time)
        last_t = self._queue_debug_last_t_by_edge.get(key)
        if last_t is not None and abs(last_t - t_now) <= 1e-9:
            return
        self._queue_debug_last_t_by_edge[key] = t_now
        denom = float(S - A_used)
        print(
            "[QUEUE_DEBUG] "
            f"tls={self.cfg.tls_id} "
            f"t={t_now:.2f} "
            f"edge={edge_id} "
            f"lanes={lanes} "
            f"src={source} "
            f"N={float(N):.2f} "
            f"A_raw={float(A_raw):.3f} "
            f"A={float(A_used):.3f} "
            f"S={float(S):.3f} "
            f"den={float(denom):.3f} "
            f"Q={float(Q):.2f}"
        )

    def _print_ev_state_debug(
        self,
        sim_time: float,
        context: str,
        ev: Optional[EvRequest] = None,
        t_i: Optional[float] = None,
        action: Optional[str] = None,
        target_phase_idx: Optional[int] = None,
    ) -> None:
        if not bool(getattr(self.cfg, "enable_ev_state_debug", False)):
            return
        ev_ctx = ev if ev is not None else self.active_ev
        if ev_ctx is None:
            return

        dyn_state = "NA"
        acc = float("nan")
        speed_now = float(getattr(ev_ctx, "speed_mps", 0.0))
        if traci is not None:
            try:
                dyn_state, acc, speed_now = self.get_vehicle_state(str(ev_ctx.ev_id))
            except Exception:
                pass

        msg_age = None
        if self.last_ev_msg_time is not None:
            msg_age = max(0.0, float(sim_time) - float(self.last_ev_msg_time))

        edge_transition = "none"
        if self._ev_left_approach_from_edge is not None or self._ev_left_approach_to_edge is not None:
            edge_transition = (
                f"{str(self._ev_left_approach_from_edge or 'NA')}"
                f"->{str(self._ev_left_approach_to_edge or 'NA')}"
            )

        print(
            "[EV_STATE_DEBUG] "
            f"tls={self.cfg.tls_id} "
            f"context={context} "
            f"t={float(sim_time):.2f} "
            f"stage={self.stage.name} "
            f"ev={ev_ctx.ev_id} "
            f"edge={ev_ctx.in_edge_id} "
            f"dist={float(getattr(ev_ctx, 'distance_to_intersection_m', 0.0)):.2f} "
            f"speed={float(speed_now):.3f} "
            f"acc={float(acc):.3f} "
            f"dyn={dyn_state} "
            f"t_i={('NA' if t_i is None else f'{float(t_i):.3f}')} "
            f"target_phase={('NA' if target_phase_idx is None else int(target_phase_idx))} "
            f"action={('NA' if action is None else str(action))} "
            f"msg_age={('NA' if msg_age is None else f'{float(msg_age):.3f}')} "
            f"passed={int(bool(self.ev_passed))} "
            f"pass_reason={str(self._ev_pass_reason or 'none')} "
            f"loop_touch={self._ev_loop_touch_time} "
            f"silence_t={self._ev_request_silence_time} "
            f"left_edge_t={self._ev_left_approach_time} "
            f"pass_t={self._ev_pass_time_est} "
            f"pass_detect_t={self._ev_pass_detect_time} "
            f"pass_proxy_t={self._ev_pass_proxy_time} "
            f"edge_transition={edge_transition}"
        )

    def _log_decision_event(
        self,
        event: str,
        sim_time: float,
        decision_source: str,
        plan: Optional[PreemptionPlan] = None,
        offer: Optional[SignalWindowOffer] = None,
        note: str = "",
    ) -> None:
        if not bool(getattr(self.cfg, "enable_decision_csv_log", False)):
            return

        path = str(getattr(self.cfg, "decision_log_csv_path", "") or "").strip()
        if not path:
            return

        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)

            fields = self._decision_log_fields()
            write_header = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
            with open(path, "a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                if write_header:
                    writer.writeheader()

                row = {
                    "wall_time": f"{time.time():.3f}",
                    "sim_time": f"{float(sim_time):.3f}",
                    "run_label": str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                    "tls_id": str(self.cfg.tls_id),
                    "intersection_id": str(self.cfg.intersection_id),
                    "event": str(event),
                    "decision_source": str(decision_source),
                    "stage": str(self.stage),
                    "plan_type": "",
                    "plan_target_phase": "",
                    "plan_extend_sec": "",
                    "plan_hurry_to_sec": "",
                    "plan_jump_time_sec": "",
                    "plan_jump_to_phase": "",
                    "offer_id": "",
                    "offer_action": "",
                    "offer_score": "",
                    "offer_feasible": "",
                    "offer_wait_sec": "",
                    "offer_miss_sec": "",
                    "offer_cost_veh_sec": "",
                    "offer_confidence": "",
                    "offer_green_window_start": "",
                    "offer_green_window_end": "",
                    "offer_arrival_window_start": "",
                    "offer_arrival_window_end": "",
                    "note": str(note or ""),
                }

                if plan is not None:
                    row["plan_type"] = str(plan.plan_type)
                    row["plan_target_phase"] = int(plan.target_phase_idx) if plan.target_phase_idx is not None else ""
                    row["plan_extend_sec"] = float(plan.extend_green_sec or 0.0)
                    row["plan_hurry_to_sec"] = "" if plan.hurry_current_phase_to_sec is None else float(plan.hurry_current_phase_to_sec)
                    row["plan_jump_time_sec"] = "" if plan.jump_time_sec is None else float(plan.jump_time_sec)
                    row["plan_jump_to_phase"] = "" if plan.jump_to_phase_idx is None else int(plan.jump_to_phase_idx)

                if offer is not None:
                    row["offer_id"] = str(getattr(offer, "offer_id", ""))
                    row["offer_action"] = str(getattr(offer, "action", ""))
                    row["offer_score"] = float(getattr(offer, "score", self.score_offer(offer)))
                    row["offer_feasible"] = bool(getattr(offer, "feasible", False))
                    row["offer_wait_sec"] = float(getattr(offer, "expected_wait_sec", 0.0))
                    row["offer_miss_sec"] = float(getattr(offer, "expected_miss_sec", 0.0))
                    row["offer_cost_veh_sec"] = float(getattr(offer, "cost_to_others_veh_sec", 0.0))
                    row["offer_confidence"] = float(getattr(offer, "confidence", 0.0))
                    gw = getattr(offer, "green_window", None)
                    aw = getattr(offer, "arrival_window", None)
                    if gw is not None and len(gw) == 2:
                        row["offer_green_window_start"] = float(gw[0])
                        row["offer_green_window_end"] = float(gw[1])
                    if aw is not None and len(aw) == 2:
                        row["offer_arrival_window_start"] = float(aw[0])
                        row["offer_arrival_window_end"] = float(aw[1])

                writer.writerow(row)
        except Exception:
            # Logging must never break control decisions.
            pass

    def _default_plan(self, sim_time: float, t_i: float, note: str = "") -> PreemptionPlan:
        target = 0
        if self.active_ev is not None:
            target = int(self.active_ev.target_phase_idx or 0)
        return PreemptionPlan(
            plan_type="none",
            target_phase_idx=target,
            notes=note or f"default_keep_program@{float(sim_time):.2f}",
        )


    # -------------
    # One-time warm start (call after traci.start)
    # -------------
    def warm_start(self) -> None:
        if traci is None:
            return
        self.default_program_id = str(traci.trafficlight.getProgram(self.cfg.tls_id))
        self._inbound_edge_to_phase = self._build_inbound_edge_to_phase_map()
        self._movement_edge_to_phase = self._build_movement_edge_to_phase_map()
        if not bool(self._b1_map_dumped):
            self._b1_map_dumped = True
            try:
                items = sorted((str(k), int(v)) for k, v in dict(self._inbound_edge_to_phase).items())
            except Exception:
                items = []
            sample = ",".join(f"{e}->{p}" for e, p in items[:12])
            try:
                mv_items = sorted(
                    (str(k[0]), str(k[1]), int(v))
                    for k, v in dict(self._movement_edge_to_phase).items()
                )
            except Exception:
                mv_items = []
            mv_sample = ",".join(f"{a}->{b}:{p}" for a, b, p in mv_items[:10])
            self._b1_dbg(
                f"warm_start_map size={len(items)} default_prog={self.default_program_id} "
                f"sample={sample if sample else '-'}"
            )
            self._b1_dbg(
                f"warm_start_movement_map size={len(mv_items)} "
                f"sample={mv_sample if mv_sample else '-'}"
            )
        self.capture_default_tls_program()   # add this

    # -------------
    # Middleware / intake
    # -------------
    def _resolve_in_edge_for_ev_message(self, msg: EvRequest) -> Tuple[str, str]:
        """
        Best-effort normalization of EV request inbound edge for this TLS.
        This keeps local actuation robust when upstream adapters provide a
        route edge that is valid globally but not inbound for this node.
        """
        in_edge_id = str(getattr(msg, "in_edge_id", "") or "")
        if in_edge_id and self._inbound_edge_to_phase.get(in_edge_id) is not None:
            return in_edge_id, "message"

        # Prefer explicit route hint carried in the request.
        for e in [str(x) for x in list(getattr(msg, "route_veh", []) or []) if str(x)]:
            if self._inbound_edge_to_phase.get(str(e)) is not None:
                return str(e), "route_veh"

        ev_id = str(getattr(msg, "ev_id", "") or "")
        # Fall back to locally cached route hints.
        for e in [str(x) for x in list(self._last_route_edges_by_ev.get(ev_id, []) or []) if str(x)]:
            if self._inbound_edge_to_phase.get(str(e)) is not None:
                return str(e), "cached_route"

        # Last resort: inspect runtime route from TraCI.
        if traci is not None and ev_id:
            try:
                live_route = [str(e) for e in list(traci.vehicle.getRoute(str(ev_id)) or []) if str(e)]
                ridx = int(traci.vehicle.getRouteIndex(str(ev_id)))
            except Exception:
                live_route = []
                ridx = -1
            if live_route:
                start = max(0, ridx) if ridx >= 0 else 0
                for e in live_route[start : min(len(live_route), start + 10)]:
                    if self._inbound_edge_to_phase.get(str(e)) is not None:
                        return str(e), "traci_route"
            try:
                cur_edge = str(traci.vehicle.getRoadID(str(ev_id)) or "")
            except Exception:
                cur_edge = ""
            if cur_edge and self._inbound_edge_to_phase.get(cur_edge) is not None:
                return cur_edge, "traci_edge"

        if in_edge_id:
            return in_edge_id, "unresolved"
        return "", "empty"

    def receive_ev_message(self, msg: EvRequest) -> None:
        # Optional packet loss / delay

        '''
        if self.cfg.enable_volatile_connectivity:
            if random.random() < self.cfg.drop_prob:
                return
            if self.cfg.max_delay_sec > 0:
                delay = random.random() * float(self.cfg.max_delay_sec)
                msg = EvRequest(**{**msg.__dict__, "sim_time": float(msg.sim_time) + delay})
        '''

        # Normalize inbound edge first so target-phase assignment is route-aware
        # even when EV requests arrive through an adapter path.
        src_edge = str(getattr(msg, "in_edge_id", "") or "")
        resolved_in_edge, resolved_src = self._resolve_in_edge_for_ev_message(msg)
        if resolved_in_edge and resolved_in_edge != src_edge:
            msg.in_edge_id = str(resolved_in_edge)
            self._fed_dbg(
                "evt=EV_REQUEST_IN_EDGE_INFER "
                f"tls={self.cfg.tls_id} ev={msg.ev_id} source={resolved_src} "
                f"in_edge_src={src_edge or '-'} in_edge_resolved={resolved_in_edge}"
            )
            self._fed_evt(
                "ev.request.in_edge.infer",
                role="intersection",
                source_service="intersection_agent",
                ev_id=str(msg.ev_id),
                tls_id=str(self.cfg.tls_id),
                infer_source=str(resolved_src),
                in_edge_src=str(src_edge),
                in_edge_resolved=str(resolved_in_edge),
                sim_time=float(getattr(msg, "sim_time", self._now()) or self._now()),
            )

        # Optional ignore very far EVs
        if float(msg.distance_to_intersection_m) > float(self.cfg.min_trigger_distance_m):
            self._fed_dbg(
                "evt=EV_REQUEST_DROP "
                f"reason=far_distance tls={self.cfg.tls_id} ev={msg.ev_id} "
                f"distance={float(msg.distance_to_intersection_m):.2f} "
                f"trigger_thr={float(self.cfg.min_trigger_distance_m):.2f} "
                f"in_edge={str(getattr(msg, 'in_edge_id', '') or '-')}"
            )
            self._fed_evt(
                "ev.request.drop",
                role="intersection",
                source_service="intersection_agent",
                reason="far_distance",
                ev_id=str(msg.ev_id),
                tls_id=str(self.cfg.tls_id),
                in_edge_id=str(getattr(msg, "in_edge_id", "") or ""),
                distance_to_intersection_m=float(getattr(msg, "distance_to_intersection_m", 0.0) or 0.0),
                trigger_threshold_m=float(getattr(self.cfg, "min_trigger_distance_m", 0.0) or 0.0),
                sim_time=float(getattr(msg, "sim_time", self._now()) or self._now()),
            )
            # Still accept as "tracking" if you want; for now we ignore to reduce churn.
            return

        # Cooldown after a pass event at this TLS to avoid duplicate re-arming.
        now_msg_t = float(msg.sim_time)
        ev_key = str(msg.ev_id)
        cooldown = max(0.0, float(getattr(self.cfg, "ev_pass_rearm_cooldown_sec", 8.0)))
        # Opportunistic prune to prevent unbounded growth.
        if self._recent_pass_time_by_ev:
            stale_cut = now_msg_t - max(5.0, cooldown * 4.0)
            self._recent_pass_time_by_ev = {
                k: float(v) for k, v in self._recent_pass_time_by_ev.items() if float(v) >= stale_cut
            }
            post_stale_cut = now_msg_t - max(
                max(5.0, cooldown * 4.0),
                float(getattr(self.cfg, "ev_pass_post_suppress_sec", 60.0)),
            )
            self._recent_pass_info_by_ev = {
                k: dict(v)
                for k, v in self._recent_pass_info_by_ev.items()
                if float(dict(v).get("pass_detect_time", dict(v).get("pass_time", -1e9)) or -1e9) >= post_stale_cut
            }
        last_pass_t = self._recent_pass_time_by_ev.get(ev_key)
        if last_pass_t is not None and (now_msg_t - float(last_pass_t)) < cooldown:
            if bool(getattr(self.cfg, "ev_pass_enable_debug", True)):
                print(
                    "[EV_PASS_REARM_SUPPRESS] "
                    f"tls={self.cfg.tls_id} "
                    f"ev={ev_key} "
                    f"msg_t={now_msg_t:.2f} "
                    f"last_pass_t={float(last_pass_t):.2f} "
                    f"cooldown={cooldown:.2f}"
                )
            self._fed_evt(
                "ev.request.drop",
                role="intersection",
                source_service="intersection_agent",
                reason="post_pass_cooldown",
                ev_id=str(ev_key),
                tls_id=str(self.cfg.tls_id),
                in_edge_id=str(getattr(msg, "in_edge_id", "") or ""),
                distance_to_intersection_m=float(getattr(msg, "distance_to_intersection_m", 0.0) or 0.0),
                last_pass_time=float(last_pass_t),
                elapsed_since_pass_sec=float(now_msg_t - float(last_pass_t)),
                suppress_window_sec=float(cooldown),
                sim_time=float(now_msg_t),
            )
            return

        pass_info = dict(self._recent_pass_info_by_ev.get(ev_key, {}) or {})
        pass_info_source = "recent_pass"
        if not pass_info:
            # _clear_ev_session() deliberately preserves last_ev_validation as the
            # durable pass record. Use it to reject late duplicate requests after
            # the transient active session has already been cleared.
            last_info = dict(getattr(self, "last_ev_validation", {}) or {})
            if str(last_info.get("ev_id", "") or "") == ev_key and last_info.get("pass_detect_time") is not None:
                pass_info = last_info
                pass_info_source = "last_ev_validation"
        if pass_info:
            pass_t = float(pass_info.get("pass_detect_time", pass_info.get("pass_time", last_pass_t or -1e9)) or -1e9)
            post_window = max(0.0, float(getattr(self.cfg, "ev_pass_post_suppress_sec", 60.0)))
            post_max_dist = max(0.0, float(getattr(self.cfg, "ev_pass_post_suppress_max_dist_m", 120.0)))
            elapsed = float(now_msg_t - pass_t)
            msg_edge = str(getattr(msg, "in_edge_id", "") or "")
            pass_from_edge = str(pass_info.get("left_approach_from_edge", "") or pass_info.get("in_edge_id", "") or "")
            same_approach = bool(msg_edge and pass_from_edge and msg_edge == pass_from_edge)
            dist_now = float(getattr(msg, "distance_to_intersection_m", 0.0) or 0.0)
            if elapsed >= 0.0 and elapsed < post_window and same_approach and dist_now <= post_max_dist:
                if bool(getattr(self.cfg, "ev_pass_enable_debug", True)):
                    print(
                        "[EV_PASS_POST_SUPPRESS] "
                        f"tls={self.cfg.tls_id} ev={ev_key} msg_t={now_msg_t:.2f} "
                        f"pass_t={pass_t:.2f} elapsed={elapsed:.2f} "
                        f"edge={msg_edge} dist={dist_now:.2f} source={pass_info_source}"
                    )
                self._fed_evt(
                    "ev.request.drop",
                    role="intersection",
                    source_service="intersection_agent",
                    reason="post_pass_same_approach",
                    ev_id=str(ev_key),
                    tls_id=str(self.cfg.tls_id),
                    in_edge_id=str(msg_edge),
                    pass_from_edge=str(pass_from_edge),
                    pass_to_edge=str(pass_info.get("left_approach_to_edge", "") or ""),
                    pass_reason=str(pass_info.get("pass_reason", "") or ""),
                    distance_to_intersection_m=float(dist_now),
                    max_distance_m=float(post_max_dist),
                    last_pass_time=float(pass_t),
                    elapsed_since_pass_sec=float(elapsed),
                    suppress_window_sec=float(post_window),
                    pass_info_source=str(pass_info_source),
                    sim_time=float(now_msg_t),
                )
                return

        prev_ev_id = str(self.active_ev.ev_id) if self.active_ev is not None else None
        self.active_ev = msg
        if prev_ev_id is not None and prev_ev_id != str(msg.ev_id):
            self._reset_ev_pass_tracking()
            self._session_event_counts = Counter()
            self._session_reason_counts = Counter()
        # Real EV telemetry supersedes any synthetic warmup context.
        self._federation_warmup_ev = None
        self._federation_warmup_valid_until = -1.0
        self.on_road = True
        self.last_ev_msg_time = float(msg.sim_time)
        self._ev_request_silence_time = None
        ev_key = str(msg.ev_id)
        req_now = float(self._now())
        req_sim = float(getattr(msg, "sim_time", req_now) or req_now)
        req_age_ms = max(0.0, (float(req_now) - float(req_sim)) * 1000.0)
        req_source = str(getattr(msg, "source_service", "") or "unknown")
        req_source_tag = str(getattr(msg, "source_tag", "") or getattr(self.cfg, "ev_request_source_tag", ""))
        req_delivery = str(getattr(msg, "delivery", "") or "")
        self._fed_evt(
            "ev.request.in",
            ev_id=str(msg.ev_id),
            tls_id=str(self.cfg.tls_id),
            in_edge_id=str(getattr(msg, "in_edge_id", "") or ""),
            distance_to_intersection_m=float(getattr(msg, "distance_to_intersection_m", 0.0) or 0.0),
            speed_mps=float(getattr(msg, "speed_mps", 0.0) or 0.0),
            sim_time=float(req_sim),
            source_service="intersection_agent",
            role="intersection",
            ev_request_source=str(req_source),
            ev_request_source_tag=str(req_source_tag),
            ev_request_delivery=str(req_delivery),
            request_age_ms=float(req_age_ms),
        )
        if list(getattr(msg, "route_intersections", []) or []):
            self._last_route_intersections_by_ev[ev_key] = [str(x) for x in list(msg.route_intersections or [])]
        if list(getattr(msg, "route_veh", []) or []):
            self._last_route_edges_by_ev[ev_key] = [str(x) for x in list(msg.route_veh or [])]

        strict_b1_local_req = "strict_b1_local" in str(req_source_tag or "")

        # If upstream did not provide route_intersections, infer them from SUMO runtime.
        # Strict B1 deliberately withholds route/corridor hints so the local
        # baseline behaves as one EV discovering one current TLS at a time.
        if (not strict_b1_local_req) and not list(getattr(self.active_ev, "route_intersections", []) or []):
            inferred_tls = self._infer_route_intersections_from_traci(str(msg.ev_id), max_count=8)
            if inferred_tls:
                self.active_ev.route_intersections = list(inferred_tls)
                self._last_route_intersections_by_ev[ev_key] = [str(x) for x in inferred_tls]
        elif strict_b1_local_req:
            self.active_ev.route_intersections = []
            self.active_ev.route_veh = []

        # F2D proactive mobile observability: the first active SI-DT on the EV
        # route can request a Drone-DT scout before downstream blind gaps are reached.
        self._maybe_prescout_drone_downstream_context(self.active_ev)

        # Auto-pick target phase if not provided
        prior_target = getattr(self.active_ev, "target_phase_idx", None)
        assign_source = "pre_assigned"
        next_out_edge = None
        if self.active_ev.target_phase_idx is None:
            in_edge_id = str(self.active_ev.in_edge_id or "")
            next_out_edge = self._next_out_edge_from_ev_message(msg)
            movement_phase = None
            if in_edge_id and next_out_edge:
                movement_phase = self._movement_edge_to_phase.get((in_edge_id, str(next_out_edge)))
            if movement_phase is not None:
                self.active_ev.target_phase_idx = int(movement_phase)
                assign_source = f"movement:{in_edge_id}->{str(next_out_edge)}"
            else:
                self.active_ev.target_phase_idx = int(self._inbound_edge_to_phase.get(in_edge_id, 0))
                assign_source = "inbound_edge"
        assigned_target = getattr(self.active_ev, "target_phase_idx", None)
        self._b1_dbg(
            f"rx ev={msg.ev_id} sim={float(msg.sim_time):.2f} in_edge={msg.in_edge_id} "
            f"dist={float(msg.distance_to_intersection_m):.2f} speed={float(msg.speed_mps):.2f} "
            f"src={str(getattr(msg, 'source_service', '') or '-')} src_tag={str(getattr(msg, 'source_tag', '') or '-') } "
            f"delivery={str(getattr(msg, 'delivery', '') or '-') } "
            f"target_before={prior_target} target_after={assigned_target} "
            f"next_out_edge={next_out_edge} assign_source={assign_source} "
            f"route_nodes={len(list(getattr(self.active_ev, 'route_intersections', []) or []))} "
            f"route_edges={len(list(getattr(self.active_ev, 'route_veh', []) or []))}"
        )
        edge_key = str(self.active_ev.in_edge_id)
        if edge_key and edge_key not in self._b1_edge_map_logged:
            self._b1_edge_map_logged.add(edge_key)
            self._b1_dbg(
                f"edge_target_map in_edge={edge_key} mapped_phase={self._inbound_edge_to_phase.get(edge_key, None)} "
                f"map_size={len(self._inbound_edge_to_phase)}"
            )
        self._update_ev_pass_runtime_observations(float(msg.sim_time))
    
    def get_last_ev_diag(self):
        if self.active_ev == None:
            return False
        
        else: 
            print(f"Last contact from {self.active_ev}")
            return True
    
    def retrive_current_phase(self):
        print(f"Current tls_id {self.cfg.tls_id}")
        if traci is None:
            self.current_phase = None
            return
        try:
            self.current_phase = traci.trafficlight.getPhase(self.cfg.tls_id)
        except Exception:
            self.current_phase = None

    def get_vehicle_state(self, veh_id):
        # 1. Get the raw acceleration (m/s^2)
        # Note: Returns the acceleration in the PREVIOUS time step
        acceleration = traci.vehicle.getAcceleration(veh_id)
        current_speed = traci.vehicle.getSpeed(veh_id)
        
        # 2. Define a threshold to ignore micro-movements
        threshold = 0.1 

        # 3. Determine Dynamic State
        if acceleration > threshold:
            state = "ACCELERATING"
        elif acceleration < -threshold:
            state = "DECELERATING"
        else:
            state = "STABLE/CONSTANT"

        return state, acceleration, current_speed

    # -------------
    # Tick
    # -------------
    def tick(self, sim_time: float) -> Optional[PreemptionPlan]:

        if self.active_ev is None:
            self._b1_dbg(f"tick_return reason=no_active_ev sim={float(sim_time):.2f} stage={self.stage}")
            self._transition_to(AgentStage.NO_REQUEST)
            self.current_plan = None
            return None
        
        #state, accel_val, current_speed = self.get_vehicle_state(self.active_ev.ev_id)
        #print(f"EV {self.active_ev.ev_id} is currently {state} with acceleration {accel_val:.2f} m/s^2, speed {current_speed:.2f} m/s")

        self.retrive_current_phase()
        self._trace_tls_signal_change(float(sim_time), reason="tick", force=False)
        self._apply_active_phase_overrides(float(sim_time))
        
        print(f"Current stage: {self.stage} -- suceptible to updates")

        # 1) periodic decision ( 1 s)
        if float(sim_time) < float(self._next_decision_time):
            return None
        self._next_decision_time = float(sim_time) + float(self.cfg.decision_period_sec)
        self._update_ev_pass_runtime_observations(float(sim_time))

        # 2) stale EV handling
        if self.last_ev_msg_time is not None:
            if (float(sim_time) - float(self.last_ev_msg_time)) > float(self.cfg.stale_ev_after_sec):
                if self._had_nondefault_actuation():
                    self._b1_dbg(
                        f"stale_ev sim={float(sim_time):.2f} age={float(sim_time)-float(self.last_ev_msg_time):.2f} "
                        f"action=transition_restoration"
                    )
                    self._transition_to(AgentStage.RESTORATION)
                else:
                    self._b1_dbg(
                        f"stale_ev sim={float(sim_time):.2f} age={float(sim_time)-float(self.last_ev_msg_time):.2f} "
                        f"action=clear_to_no_request"
                    )
                    self._clear_ev_session()
                    self._transition_to(AgentStage.NO_REQUEST)
                    return None
        
        ev = self.active_ev
        if ev is None:
            self._b1_dbg(f"tick_return reason=active_ev_cleared sim={float(sim_time):.2f} stage={self.stage}")
            self._transition_to(AgentStage.NO_REQUEST)
            return None

        t_i = float(self._estimate_arrival_time(float(sim_time), ev))
        erl = int(ev.erl_level)
        clrs = int(self._compute_clrs_level(ev.in_edge_id))
        tul = int(self._compute_tul_level(float(sim_time), t_i))
        target_phase = int(ev.target_phase_idx or 0)
        self._emit_queue_snapshot(sim_time=float(sim_time), ev=ev, t_i=float(t_i))

        print(f'Expected arrival time: {t_i}')
        self._print_ev_state_debug(
            sim_time=float(sim_time),
            context="plan_eval",
            ev=ev,
            t_i=float(t_i),
            action="stage_machine",
            target_phase_idx=int(target_phase),
        )
        if self._ev_pred_ti_first is None:
            self._ev_pred_ti_first = float(t_i)
        self._ev_pred_ti_last = float(t_i)
        if (self._ev_loop_touch_time is not None) and (not self._ev_touch_reported):
            e0 = float(self._ev_loop_touch_time - float(self._ev_pred_ti_first or t_i))
            e1 = float(self._ev_loop_touch_time - float(self._ev_pred_ti_last or t_i))
            if bool(getattr(self.cfg, "ev_pass_enable_debug", True)):
                print(
                    "[EV_PASS_VALIDATE] "
                    f"tls={self.cfg.tls_id} "
                    f"ev={ev.ev_id} "
                    f"loop_touch_t={float(self._ev_loop_touch_time):.2f} "
                    f"ti_first={float(self._ev_pred_ti_first or t_i):.2f} "
                    f"ti_last={float(self._ev_pred_ti_last or t_i):.2f} "
                    f"err_first={e0:.2f} "
                    f"err_last={e1:.2f} "
                    f"loop={self._ev_loop_touch_loop_id}"
                )
            self._ev_touch_reported = True
        

        #------
        win = self._predict_next_phase_window(float(sim_time), target_phase)
        window_ok = False
        if win is not None:
            a0, a1 = t_i - float(ev.delta_sec), t_i + float(ev.delta_sec)
            window_ok = (win[0] <= a0) and (win[1] >= a1)
        self._b1_dbg(
            f"tick_ctx sim={float(sim_time):.2f} stage={self.stage} ev={ev.ev_id} edge={ev.in_edge_id} "
            f"target={int(target_phase)} cur_phase={self.current_phase} t_i={float(t_i):.2f} "
            f"arr_win=({float(t_i-float(ev.delta_sec)):.2f},{float(t_i+float(ev.delta_sec)):.2f}) "
            f"base_win={('NA' if win is None else f'({float(win[0]):.2f},{float(win[1]):.2f})')} "
            f"window_ok={int(bool(window_ok))} clrs={int(clrs)} tul={int(tul)}"
        )

        # 3) passed detection: if no intervention ever happened, skip restoration
        passed_now = self._detect_ev_passed(float(sim_time))
        print(f"EV passed: {passed_now}")
        if passed_now:
            self.ev_passed = True
            print("EV passed detected.")
            if self._had_nondefault_actuation():
                print("EV had non-default actuation; moving to restoration.")
                self._b1_dbg(f"ev_passed sim={float(sim_time):.2f} action=transition_restoration")
                self._transition_to(AgentStage.RESTORATION)
            else:
                self._b1_dbg(f"ev_passed sim={float(sim_time):.2f} action=clear_to_no_request")
                self._clear_ev_session()
                self._transition_to(AgentStage.NO_REQUEST)
                return None
        #------
        
        # stage machine
        if self.stage == AgentStage.NO_REQUEST:
                if window_ok and clrs <= int(self.cfg.baseline_clrs_max) and tul <= int(self.cfg.baseline_tul_max):
                    self._transition_to(AgentStage.BASELINE_VALIDATION)
                    plan = self._baseline_validation_plan(float(sim_time), t_i, clrs, tul)
                    self.current_plan = plan
                    self._b1_dbg(
                        f"tick_return reason=no_request_to_baseline sim={float(sim_time):.2f} "
                        f"plan_type={(None if plan is None else plan.plan_type)}"
                    )
                    return plan
                self._transition_to(AgentStage.SATURATION_REDUCTION)
                self._b1_dbg(
                    f"stage_transition sim={float(sim_time):.2f} from=NO_REQUEST to=SATURATION_REDUCTION "
                    f"window_ok={int(bool(window_ok))} clrs={int(clrs)} tul={int(tul)}"
                )

        if self.stage == AgentStage.BASELINE_VALIDATION:
            if passed_now:
                self.ev_passed = True
                if self._had_nondefault_actuation():
                    self._transition_to(AgentStage.RESTORATION)
                    plan = self._restoration_plan(float(sim_time))
                    self.current_plan = plan
                    self._b1_dbg(
                        f"tick_return reason=baseline_passed_restore sim={float(sim_time):.2f} "
                        f"plan_type={(None if plan is None else plan.plan_type)}"
                    )
                    return plan
                self._clear_ev_session()
                self._transition_to(AgentStage.NO_REQUEST)
                self._b1_dbg(f"tick_return reason=baseline_passed_clear sim={float(sim_time):.2f}")
                return None

            plan = self._baseline_validation_plan(float(sim_time), t_i, clrs, tul)
            if plan is not None:
                self.current_plan = plan
                self._b1_dbg(
                    f"tick_return reason=baseline_validation sim={float(sim_time):.2f} "
                    f"plan_type={plan.plan_type}"
                )
                return plan

            self._transition_to(AgentStage.SATURATION_REDUCTION)
            self._b1_dbg(f"stage_transition sim={float(sim_time):.2f} from=BASELINE_VALIDATION to=SATURATION_REDUCTION")

        if self.stage == AgentStage.SATURATION_REDUCTION:
            plan = self._saturation_reduction_plan(float(sim_time), erl, clrs, tul)
            self.current_plan = plan
            print(f"t_i: {t_i} - float(sim_time: {sim_time})) <= float(self.cfg.saturation_to_preempt_gap_sec): {self.cfg.saturation_to_preempt_gap_sec}")

            if (t_i - float(sim_time)) <= float(self.cfg.saturation_to_preempt_gap_sec): # and not :
                print("Transitioning to preemption stage due to proximity to expected arrival time.") 
                self._transition_to(AgentStage.PREEMPTION)
                self._b1_dbg(
                    f"stage_transition sim={float(sim_time):.2f} from=SATURATION_REDUCTION to=PREEMPTION "
                    f"eta_gap={float(t_i-float(sim_time)):.2f} threshold={float(self.cfg.saturation_to_preempt_gap_sec):.2f}"
                )
            
            if window_ok:
                print("Window ok, EV can pass in normal operation. Moving to baseline validation stage.")
                self._transition_to(AgentStage.BASELINE_VALIDATION)
                plan = self._baseline_validation_plan(float(sim_time), t_i, clrs, tul)
                if plan is None:
                    plan = self._default_plan(
                        float(sim_time),
                        float(t_i),
                        note="Window feasible in saturation stage; fallback no intervention",
                    )
                self.current_plan = plan
            self._b1_dbg(
                f"tick_return reason=saturation_reduction sim={float(sim_time):.2f} "
                f"plan_type={(None if plan is None else plan.plan_type)} "
                f"window_ok={int(bool(window_ok))}"
            )
            return plan

        if self.stage in (AgentStage.PREEMPTION_NON_INTRUSIVE, AgentStage.PREEMPTION):
            # Keep a single control state for preemption. We keep the NON_INTRUSIVE enum only
            # for backwards compatibility with logs/offer flows, but normalize here.
            if self.stage == AgentStage.PREEMPTION_NON_INTRUSIVE:
                self._transition_to(AgentStage.PREEMPTION, "normalize_preemption_state")
                self._b1_dbg(f"stage_transition sim={float(sim_time):.2f} from=PREEMPTION_NON_INTRUSIVE to=PREEMPTION")

            # PREEMPTION internal ladder:
            # 1) baseline/no-actuation if already feasible
            # 2) non-intrusive (simplified/QP)
            # 3) intrusive fallback
            if window_ok:
                plan = self._baseline_validation_plan(float(sim_time), t_i, clrs, tul)
                if plan is None:
                    plan = self._default_plan(
                        float(sim_time),
                        float(t_i),
                        note="Window feasible in preemption; no intervention",
                    )
            else:
                plan = self._non_intrusive_preemption_plan(float(sim_time), float(t_i))
                if plan is None:
                    plan = self._intrusive_preemption_plan(float(sim_time), float(t_i))

            self.current_plan = plan

            if passed_now:
                self.ev_passed = True
                if self._had_nondefault_actuation():
                    self._transition_to(AgentStage.RESTORATION)
                    self._b1_dbg(f"stage_transition sim={float(sim_time):.2f} from=PREEMPTION to=RESTORATION reason=passed_after_actuation")
                else:
                    self._clear_ev_session()
                    self._transition_to(AgentStage.NO_REQUEST)
                    self._b1_dbg(f"tick_return reason=preemption_passed_clear sim={float(sim_time):.2f}")
                    return None
            self._b1_dbg(
                f"tick_return reason=preemption sim={float(sim_time):.2f} "
                f"window_ok={int(bool(window_ok))} plan_type={(None if plan is None else plan.plan_type)}"
            )
            return plan

        if self.stage == AgentStage.RESTORATION:
            plan = self._restoration_plan(float(sim_time))
            self.current_plan = plan

            if self._restoration_complete():
                self._clear_ev_session()
                self._transition_to(AgentStage.NO_REQUEST)
                self._b1_dbg(f"stage_transition sim={float(sim_time):.2f} from=RESTORATION to=NO_REQUEST reason=restoration_complete")
            self._b1_dbg(
                f"tick_return reason=restoration sim={float(sim_time):.2f} "
                f"plan_type={(None if plan is None else plan.plan_type)}"
            )
            return plan

        self._b1_dbg(f"tick_return reason=fallthrough_none sim={float(sim_time):.2f} stage={self.stage}")
        return None


    # 1) Add missing method inside IntersectionAgent
    def _baseline_validation_plan(self, sim_time: float, t_i: float, clrs: int, tul: int) -> Optional[PreemptionPlan]:
        ev = self.active_ev
        if ev is None:
            self._transition_to(AgentStage.NO_REQUEST)
            return None

        target = int(ev.target_phase_idx or 0)
        win = self._predict_next_phase_window(float(sim_time), target)
        if win is None:
            return None

        a0 = float(t_i) - float(ev.delta_sec)
        a1 = float(t_i) + float(ev.delta_sec)
        if win[0] <= a0 and win[1] >= a1:
            if clrs <= int(self.cfg.baseline_clrs_max) and tul <= int(self.cfg.baseline_tul_max):
                return PreemptionPlan(
                    plan_type="none",
                    target_phase_idx=target,
                    notes="Baseline feasible; no intervention"
                )
            return PreemptionPlan(
                plan_type="none",
                target_phase_idx=target,
                notes="Window feasible; no intervention despite CLRS/TUL gate"
            )

        return None



    # =========================
    # Plans
    # =========================

    def _saturation_reduction_plan(self, sim_time: float, erl: int, clrs: int, tul: int) -> PreemptionPlan:
        drrs = self._compute_drrs(erl, clrs, tul)
        ext = self._extension_time_from_drrs(drrs)

        return PreemptionPlan(
            plan_type="saturation_reduction",
            target_phase_idx=int(self.active_ev.target_phase_idx or 0),
            extend_green_sec=float(ext),
            notes=f"DRRS={drrs:.3f} -> extend={ext:.1f}s (ERL={erl},CLRS={clrs},TUL={tul})"
        )
    '''
    def _preemption_plan(self, sim_time: float, t_i: float, erl: int, clrs: int, tul: int) -> PreemptionPlan:
        # Non-intrusive: ONLY if QP-feasible (paper spirit).
        non_intrusive = self._try_non_intrusive_qp(sim_time, t_i)
        if non_intrusive is not None:
            return non_intrusive

        # Fallback intrusive
        return self._intrusive_preemption(sim_time, t_i)
    

    def _preemption_plan(self, sim_time: float, t_i: float, erl: int, clrs: int, tul: int) -> PreemptionPlan:
        # Non-intrusive (paper): full-cycle timing adjustment with queue-clearing constraint.
        paper_plan = self._try_non_intrusive_paper_qp(sim_time, t_i)
        if paper_plan is not None:
            # Activate override schedule so it is applied as phases become current.
            self._activate_phase_overrides_from_plan(paper_plan)
            return paper_plan

        # Non-intrusive (reduced TraCI-feasible): hurry current phase and/or extend target phase.
        non_intrusive = self._try_non_intrusive_qp(sim_time, t_i)
        if non_intrusive is not None:
            return non_intrusive

        # Fallback intrusive
        return self._intrusive_preemption(sim_time, t_i)'''
    
    def _preemption_plan(self, sim_time: float, t_i: float, erl: int, clrs: int, tul: int) -> PreemptionPlan:
        plan = self._non_intrusive_preemption_plan(float(sim_time), float(t_i))
        if plan is not None:
            return plan
        return self._intrusive_preemption_plan(float(sim_time), float(t_i))

    def _non_intrusive_preemption_plan(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        # Prefer paper-aligned non-intrusive optimization path first.
        self._b1_dbg(f"preemption_non_intr_enter sim={float(sim_time):.2f} t_i={float(t_i):.2f}")
        plan = self._try_non_intrusive_paper_qp(float(sim_time), float(t_i))
        if plan is not None:
            if plan.phase_duration_overrides:
                self._activate_phase_overrides_from_plan(plan)
                self._b1_dbg(
                    f"preemption_non_intr_overrides activated count={len(dict(plan.phase_duration_overrides or {}))} "
                    f"start={getattr(plan, 'override_start_time_sec', None)} end={getattr(plan, 'override_end_time_sec', None)}"
                )
            self._b1_dbg(
                f"preemption_non_intr_return plan_type={plan.plan_type} target={plan.target_phase_idx} "
                f"ext={float(plan.extend_green_sec or 0.0):.2f} hurry={getattr(plan,'hurry_current_phase_to_sec',None)} "
                f"jump={getattr(plan,'jump_time_sec',None)}"
            )
            return plan
        self._b1_dbg("preemption_non_intr_return none reason=paper_qp_unfeasible")
        return None

    def _intrusive_preemption_plan(self, sim_time: float, t_i: float) -> PreemptionPlan:
        self._b1_dbg(f"preemption_intr_enter sim={float(sim_time):.2f} t_i={float(t_i):.2f}")
        plan = self._intrusive_preemption(float(sim_time), float(t_i))
        if plan is not None:
            self._b1_dbg(
                f"preemption_intr_return plan_type={plan.plan_type} target={plan.target_phase_idx} "
                f"jump_t={getattr(plan,'jump_time_sec',None)} jump_to={getattr(plan,'jump_to_phase_idx',None)}"
            )
            return plan
        self._b1_dbg("preemption_intr_fallback default_none reason=intrusive_unfeasible")
        return self._default_plan(
            float(sim_time),
            float(t_i),
            note="Intrusive fallback not required by current constraints",
        )

    def _restoration_plan(self, sim_time: float) -> PreemptionPlan:
        # Optional: compute a one-cycle restoration schedule
        if self.cfg.enable_restoration_lp and self._restoration_schedule is None:
            self._restoration_schedule = self._compute_restoration_schedule_lp()
            self._restoration_applied_phases.clear()

        return PreemptionPlan(
            plan_type="restore",
            target_phase_idx=int(self.active_ev.target_phase_idx or 0),
            notes="Restore default program (and optionally apply restoration schedule)"
        )
    
    # =========================
    # V2: Predictive window offers ("what-if" decision support)
    # =========================

    def _plan_signature(self, plan: PreemptionPlan) -> Tuple[object, ...]:
        return (
            str(plan.plan_type),
            int(plan.target_phase_idx or 0),
            round(float(plan.extend_green_sec or 0.0), 2),
            round(float(plan.hurry_current_phase_to_sec or 0.0), 2) if plan.hurry_current_phase_to_sec is not None else None,
            round(float(plan.jump_time_sec or 0.0), 2) if plan.jump_time_sec is not None else None,
            int(plan.jump_to_phase_idx) if plan.jump_to_phase_idx is not None else None,
        )

    @staticmethod
    def _plan_type_to_kind(plan_type: str) -> float:
        mapping = {
            "none": 0.0,
            "saturation_reduction": 1.0,
            "non_intrusive": 2.0,
            "intrusive": 3.0,
            "restore": 4.0,
        }
        return float(mapping.get(str(plan_type), 0.0))

    @staticmethod
    def _kind_to_plan_type(kind: float, default: str = "none") -> str:
        mapping = {
            0: "none",
            1: "saturation_reduction",
            2: "non_intrusive",
            3: "intrusive",
            4: "restore",
        }
        return mapping.get(int(kind), default)

    def _resolve_target_phase_from_plan(self, plan: PreemptionPlan, ev: EvRequest) -> int:
        if plan.target_phase_idx is not None:
            return int(plan.target_phase_idx)
        if ev.target_phase_idx is not None:
            return int(ev.target_phase_idx)
        return int(self._inbound_edge_to_phase.get(ev.in_edge_id, 0))

    def _action_from_plan(self, plan: PreemptionPlan) -> str:
        pt = str(plan.plan_type)
        if pt == "intrusive":
            return "jump"
        if pt == "saturation_reduction":
            return "extend"
        if pt == "non_intrusive":
            if plan.hurry_current_phase_to_sec is not None:
                return "hurry"
            if float(plan.extend_green_sec or 0.0) > 0.0:
                return "extend"
            return "none"
        return "none"

    def _action_params_from_plan(self, sim_time: float, plan: PreemptionPlan, target_phase: int) -> Dict[str, float]:
        ap: Dict[str, float] = {"plan_kind": self._plan_type_to_kind(str(plan.plan_type))}

        ext = float(plan.extend_green_sec or 0.0)
        if ext > 0.0:
            ap["ext"] = ext

        if plan.hurry_current_phase_to_sec is not None:
            ap["hurry_to"] = float(plan.hurry_current_phase_to_sec)

        jump_to = int(plan.jump_to_phase_idx) if plan.jump_to_phase_idx is not None else int(target_phase)
        if str(plan.plan_type) == "intrusive":
            ap["jump_to"] = float(jump_to)
            ap["jump_time"] = float(plan.jump_time_sec) if plan.jump_time_sec is not None else float(sim_time + self.cfg.SIT_sec + self.cfg.YT_sec)

        if plan.planned_green_window is not None:
            ap["planned_ws"] = float(plan.planned_green_window[0])
            ap["planned_we"] = float(plan.planned_green_window[1])

        return ap

    def _offer_has_effective_actuation(self, offer: Optional[SignalWindowOffer]) -> bool:
        if offer is None:
            return False
        action = str(getattr(offer, "action", "none") or "none")
        ap = dict(getattr(offer, "action_params", {}) or {})
        if action == "jump":
            return True
        if action == "hurry":
            return "hurry_to" in ap
        if action == "extend":
            return float(ap.get("ext", 0.0) or 0.0) > 0.0
        return False

    def _normalize_noop_offer_plan(self, offer: SignalWindowOffer, plan: PreemptionPlan) -> PreemptionPlan:
        """Prevent F2 no-action offers from shadowing actionable local B1 plans."""
        pt = str(getattr(plan, "plan_type", "") or "")
        if pt in ("none", "restore"):
            return plan
        if self._offer_has_effective_actuation(offer):
            return plan
        ap = dict(getattr(offer, "action_params", {}) or {})
        ap["plan_kind"] = self._plan_type_to_kind("none")
        offer.action_params = ap
        return self._default_plan(
            float(getattr(offer, "created_time", 0.0) or 0.0),
            0.0,
            note=f"normalized_noop_offer_from:{pt}",
        )

    def _green_window_from_plan(
        self,
        sim_time: float,
        action: str,
        action_params: Dict[str, float],
        base_win: Tuple[float, float],
        arr_win: Tuple[float, float],
    ) -> Tuple[float, float]:
        pws = action_params.get("planned_ws", None)
        pwe = action_params.get("planned_we", None)
        if pws is not None and pwe is not None:
            ws, we = float(pws), float(pwe)
            if we >= ws:
                return (ws, we)

        if action == "extend":
            ext = max(0.0, float(action_params.get("ext", 0.0)))
            return (float(base_win[0]), float(base_win[1] + ext))

        if action == "hurry":
            hurry_to = float(action_params.get("hurry_to", 2.0))
            ext = max(0.0, float(action_params.get("ext", 0.0)))
            ws = max(float(sim_time) + float(self.cfg.SIT_sec), float(arr_win[0]) - 0.5)
            we = max(float(arr_win[1]) + 1.0, ws + max(0.5, hurry_to)) + ext
            return (ws, we)

        if action == "jump":
            jump_time = float(action_params.get("jump_time", float(sim_time + self.cfg.SIT_sec + self.cfg.YT_sec)))
            ws = max(jump_time + float(self.cfg.SIT_sec), float(arr_win[0]) - 0.3)
            we = float(arr_win[1]) + 1.5
            return (ws, we)

        return (float(base_win[0]), float(base_win[1]))

    def _plan_type_from_offer(self, offer: SignalWindowOffer) -> str:
        ap = dict(offer.action_params or {})
        if "plan_kind" in ap:
            pt = self._kind_to_plan_type(float(ap.get("plan_kind", 0.0)), default="none")
            if pt not in ("none", "restore") and not self._offer_has_effective_actuation(offer):
                return "none"
            return pt

        action = str(offer.action)
        if action == "jump":
            return "intrusive"
        if action == "hurry":
            return "non_intrusive"
        if action == "extend":
            if float(ap.get("is_sat", 0.0)) >= 0.5:
                return "saturation_reduction"
            sat_thr = float(getattr(self.cfg, "sat_reduce_max_ext", self.cfg.max_target_green_extension_sec))
            return "saturation_reduction" if float(ap.get("ext", 0.0)) <= sat_thr else "non_intrusive"
        return "none"

    def _candidate_plans_for_f2(self, sim_time: float, t_i: float) -> List[PreemptionPlan]:
        plans: List[PreemptionPlan] = []

        # Keep F2 decision cadence aligned with tick()/B1:
        # if not due for a new decision, do not generate new offers this step.
        if float(sim_time) < float(self._next_decision_time):
            # Keep executing the last local plan between decision instants so
            # F2 actuation does not become sparse or federation-dependent.
            if self.current_plan is not None:
                return [self.current_plan]
            return []

        t0_tick = time.perf_counter()
        primary_plan = self.tick(sim_time)
        dt_tick_ms = (time.perf_counter() - t0_tick) * 1000.0
        self._last_tick_compute_ms = float(max(0.0, dt_tick_ms))
        self._fed_evt(
            "intersection.compute.tick.duration_ms",
            sim_time=float(sim_time),
            ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
            duration_ms=float(dt_tick_ms),
        )
        if primary_plan is None:
            primary_plan = self.current_plan
        self._last_f2_primary_plan = primary_plan
        if primary_plan is not None:
            plans.append(primary_plan)

        alt_non_intr = self._try_non_intrusive_paper_qp(sim_time, t_i)
        if alt_non_intr is not None:
            plans.append(alt_non_intr)

        alt_intr = self._intrusive_preemption(sim_time, t_i)
        if alt_intr is not None:
            plans.append(alt_intr)

        if not plans:
            plans.append(self._default_plan(sim_time, t_i, note="F2 fallback: no candidate from tick"))

        dedup: Dict[Tuple[object, ...], PreemptionPlan] = {}
        for p in plans:
            dedup[self._plan_signature(p)] = p
        return list(dedup.values())

    def compute_offers(self, sim_time: float) -> List[SignalWindowOffer]:
        ev = self.active_ev
        if ev is None:
            return []
        t_i = float(self._estimate_arrival_time(float(sim_time), ev))
        offers: List[SignalWindowOffer] = []
        self._offer_plan_cache = {}

        plans = self._candidate_plans_for_f2(sim_time, t_i)
        for plan in plans:
            offer = self._make_offer_from_plan(sim_time, ev, plan)
            offers.append(offer)
            self._offer_plan_cache[offer.offer_id] = self._normalize_noop_offer_plan(offer, plan)

        # de-dup by action/phase/window
        dedup = {}
        for o in offers:
            ap = dict(o.action_params or {})
            key = (
                int(float(ap.get("plan_kind", -1.0))),
                o.action,
                o.target_phase_idx,
                round(o.green_window[0], 1),
                round(o.green_window[1], 1),
                round(float(ap.get("ext", 0.0)), 2),
                round(float(ap.get("hurry_to", 0.0)), 2),
                round(float(ap.get("jump_time", 0.0)), 2),
            )
            dedup[key] = o
        offers = list(dedup.values())

        self._offer_plan_cache = {
            o.offer_id: self._offer_plan_cache.get(o.offer_id, self._fallback_plan_from_offer(o))
            for o in offers
        }

        for o in offers:
            o.score = self.score_offer(o)
        offers.sort(key=self._offer_selection_key)
        return offers

    
    def _make_offer_from_plan(self, sim_time: float, ev: EvRequest, plan: PreemptionPlan) -> SignalWindowOffer:
        target = self._resolve_target_phase_from_plan(plan, ev)
        eta = float(self._estimate_arrival_time(sim_time, ev))
        arr_win = (eta - ev.delta_sec, eta + ev.delta_sec)
        base_win = self._predict_next_phase_window(sim_time, target) or arr_win
        action = self._action_from_plan(plan)
        ap = self._action_params_from_plan(sim_time, plan, target)
        gw = self._green_window_from_plan(
            sim_time=sim_time,
            action=action,
            action_params=ap,
            base_win=base_win,
            arr_win=arr_win,
        )

        # keep jump target explicit on offer surface
        offer_target = int(ap.get("jump_to", target)) if action == "jump" else int(target)
        return self._make_offer(
            sim_time=sim_time,
            ev=ev,
            target_phase_idx=offer_target,
            action=action,
            action_params=ap,
            green_window=gw,
            arrival_window=arr_win,
        )
    
    def _fallback_plan_from_offer(self, offer: SignalWindowOffer) -> PreemptionPlan:
        action = str(offer.action)
        ap = dict(offer.action_params or {})
        target = int(offer.target_phase_idx) if offer.target_phase_idx is not None else None
        plan_type = self._plan_type_from_offer(offer)
        if plan_type == "none" and action == "none":
            plan_type = "none"

        ext = max(0.0, float(ap.get("ext", 0.0)))
        hurry = ap.get("hurry_to", None)
        hurry_to = float(hurry) if hurry is not None else None

        planned_window = None
        if "planned_ws" in ap and "planned_we" in ap:
            ws = float(ap.get("planned_ws"))
            we = float(ap.get("planned_we"))
            if we >= ws:
                planned_window = (ws, we)

        jump_to = int(ap.get("jump_to", target if target is not None else 0))
        jump_t = float(ap.get("jump_time", 0.0))

        if plan_type == "intrusive":
            return PreemptionPlan(
                plan_type="intrusive",
                target_phase_idx=jump_to,
                jump_to_phase_idx=jump_to,
                jump_time_sec=jump_t,
                planned_green_window=planned_window,
                notes=f"from_offer:jump(to={jump_to}, t={jump_t:.2f})",
            )

        if plan_type == "restore":
            return PreemptionPlan(
                plan_type="restore",
                target_phase_idx=int(target if target is not None else 0),
                planned_green_window=planned_window,
                notes="from_offer:restore",
            )

        if plan_type == "saturation_reduction":
            return PreemptionPlan(
                plan_type="saturation_reduction",
                target_phase_idx=int(target if target is not None else 0),
                extend_green_sec=ext,
                planned_green_window=planned_window,
                notes=f"from_offer:extend_sat({ext:.2f})",
            )

        if plan_type == "non_intrusive":
            return PreemptionPlan(
                plan_type="non_intrusive",
                target_phase_idx=int(target if target is not None else 0),
                extend_green_sec=ext,
                hurry_current_phase_to_sec=hurry_to,
                planned_green_window=planned_window,
                notes=f"from_offer:non_intrusive(ext={ext:.2f},hurry={hurry_to})",
            )

        return PreemptionPlan(
            plan_type="none",
            target_phase_idx=int(target if target is not None else 0),
            planned_green_window=planned_window,
            notes="from_offer:unknown",
        )

    
    def _score_offer_legacy(self, offer: SignalWindowOffer) -> float:
        # lower is better (legacy policy)
        w_wait = 1.0
        w_queue = 0.25
        w_risk = 0.8
        w_intr = 1.2
        w_conf = float(getattr(self.cfg, "offer_w_conf_risk", 0.2))

        offer_pt = self._plan_type_from_offer(offer)
        intrusive_pen = 1.0 if offer_pt == "intrusive" else (0.35 if str(offer.action) == "hurry" else 0.0)

        score = (
            w_wait * float(offer.expected_wait_sec)
            + w_queue * float(offer.cost_to_others_veh_sec)
            + w_risk * float(offer.expected_miss_sec)
            + w_intr * intrusive_pen
            + w_conf * max(0.0, 1.0 - float(getattr(offer, "confidence", 0.5)))
        )

        if not offer.feasible:
            score += 1e5
        return float(score)

    def _offer_severity(self, offer: SignalWindowOffer) -> Tuple[int, float]:
        """
        Returns (severity_class, severity_magnitude). Lower is less disruptive.
        """
        pt = self._plan_type_from_offer(offer)
        ap = dict(offer.action_params or {})
        ext = max(0.0, float(ap.get("ext", 0.0)))

        if pt in ("none", "restore"):
            return (0, 0.0)

        if pt == "saturation_reduction":
            return (1, ext)

        if pt == "non_intrusive":
            if str(offer.action) == "hurry":
                hurry_to = float(ap.get("hurry_to", 2.0))
                # Smaller hurry_to implies stronger disruption.
                return (2, max(0.0, 3.0 - hurry_to) + 0.1 * ext)
            return (2, ext)

        # intrusive
        return (3, 100.0 + ext)

    def _offer_actuation_magnitude(self, offer: SignalWindowOffer) -> float:
        """
        A direct actuation-size proxy used as a tie-break:
        when EV service metrics are equal, prefer smaller interventions.
        """
        ap = dict(offer.action_params or {})
        action = str(getattr(offer, "action", "none"))

        if action == "none":
            return 0.0
        if action == "extend":
            return max(0.0, float(ap.get("ext", 0.0)))
        if action == "hurry":
            hurry_to = float(ap.get("hurry_to", 2.0))
            # Smaller hurry_to means stronger intervention.
            return max(0.0, 3.0 - hurry_to) + 0.1 * max(0.0, float(ap.get("ext", 0.0)))
        if action == "jump":
            return 1000.0
        return 10.0

    def improved_score_offer(self, offer: SignalWindowOffer) -> float:
        """
        Numeric improved score:
        EV service priority + disruption + intervention severity.
        """
        wait = max(0.0, float(offer.expected_wait_sec))
        miss = max(0.0, float(offer.expected_miss_sec))
        dis = max(0.0, float(offer.cost_to_others_veh_sec))
        sev_cls, sev_mag = self._offer_severity(offer)

        w_wait = float(getattr(self.cfg, "offer_w_wait", 1.0))
        w_miss = float(getattr(self.cfg, "offer_w_miss", 4.0))
        w_queue = float(getattr(self.cfg, "offer_w_queue", 0.25))
        w_sev = float(getattr(self.cfg, "offer_w_severity", 1.2))
        w_mag = float(getattr(self.cfg, "offer_w_magnitude", 0.05))
        w_conf = float(getattr(self.cfg, "offer_w_conf_risk", 0.2))

        score = (
            w_wait * wait
            + w_miss * miss
            + w_queue * dis
            + w_sev * float(sev_cls)
            + w_mag * float(sev_mag)
            + w_conf * max(0.0, 1.0 - float(getattr(offer, "confidence", 0.5)))
        )

        if not offer.feasible:
            score += 1e6

        hard_wait = float(getattr(self.cfg, "offer_hard_wait_sec", 6.0))
        hard_miss = float(getattr(self.cfg, "offer_hard_miss_sec", 0.5))
        if wait > hard_wait:
            score += 1e4 * (wait - hard_wait + 1.0)
        if miss > hard_miss:
            score += 1e4 * (miss - hard_miss + 1.0)

        return float(score)

    def _robust_norm(self, value: float, scale: float) -> float:
        sc = max(1e-6, float(scale))
        return max(0.0, float(value)) / sc

    def _robust_metrics_for_offer(self, offer: SignalWindowOffer) -> "OfferRobustMetricSnapshot":
        """
        Cached robust metrics per offer to avoid recomputation across rank/score calls.
        """
        cache = getattr(self, "_robust_offer_metrics_cache", None)
        if not isinstance(cache, dict):
            cache = {}
            self._robust_offer_metrics_cache = cache

        key = str(getattr(offer, "offer_id", ""))
        t_offer = float(getattr(offer, "created_time", 0.0))
        hit = cache.get(key)
        if isinstance(hit, tuple) and len(hit) == 2 and abs(float(hit[0]) - t_offer) <= 1e-6:
            return hit[1]

        metrics = collect_offer_robust_metrics(
            agent=self,
            sim_time=t_offer,
            offer=offer,
            sim_step_sec=float(getattr(self.cfg, "robust_ttc_step_sec", 0.1)),
            ttc_threshold_sec=float(getattr(self.cfg, "robust_ttc_threshold_sec", 5.0)),
            federation_age_window_sec=float(getattr(self.cfg, "robust_fed_age_window_sec", 15.0)),
        )
        cache[key] = (t_offer, metrics)
        # Keep cache bounded.
        if len(cache) > 256:
            for old_k in list(cache.keys())[:64]:
                cache.pop(old_k, None)
        return metrics

    def _robust_hard_flags(self, offer: SignalWindowOffer, m: "OfferRobustMetricSnapshot") -> Tuple[int, int, int, int]:
        hard_miss = 1 if float(m.ev_miss_probability_uniform) > float(getattr(self.cfg, "robust_hard_miss_prob", 0.35)) else 0

        queue_gate_enabled = (
            float(m.queue_required_clear_sec) > 0.0 and
            float(m.queue_required_clear_sec) <= float(getattr(self.cfg, "robust_queue_unstable_sec", 600.0))
        )
        hard_queue = 1 if (
            queue_gate_enabled and
            float(m.queue_clear_margin_sec) < float(getattr(self.cfg, "robust_hard_queue_margin_sec", -0.5))
        ) else 0

        hard_ttc = 1 if float(m.ttc_sec) < float(getattr(self.cfg, "robust_hard_ttc_sec", 2.0)) else 0

        hard_speed = 1 if (
            bool(getattr(self.cfg, "robust_require_speed_advice_feasible", False)) and
            float(m.speed_advice_feasible) < 0.5
        ) else 0

        return int(hard_miss), int(hard_queue), int(hard_ttc), int(hard_speed)

    def _latest_downstream_feedback(
        self,
        ev_id: str,
        max_age_sec: float = 15.0,
        responder_tls: Optional[str] = None,
    ) -> Optional[dict]:
        now = self._now()
        best = None
        best_ts = -1e18
        for meta in getattr(self, "resp_cache", {}).values():
            if str(meta.get("ev_id", "")) != str(ev_id):
                continue
            if responder_tls is not None and str(meta.get("responder_tls", "")) != str(responder_tls):
                continue
            age = float(now) - float(meta.get("ts", 0.0))
            if age > float(max_age_sec):
                continue
            ts = float(meta.get("ts", 0.0))
            if ts > best_ts:
                best = meta
                best_ts = ts
        return best

    def on_neighbor_state(self, source_tls: str, payload: dict, sim_time: Optional[float] = None) -> None:
        """
        Receive live federation TLS state from a neighboring intersection.
        Used as fallback context when reservation feedback is temporarily missing.
        """
        try:
            src = str(source_tls or "").strip()
            if not src:
                return
            now = float(self._now())
            sim_t = float(sim_time if sim_time is not None else payload.get("simTime", payload.get("sim_time", now)))
            phase = int(payload.get("phase", -1))
            next_switch = float(payload.get("nextSwitch", payload.get("next_switch", -1.0)) or -1.0)
            p_sim = float(payload.get("simTime", payload.get("sim_time", sim_t)) or sim_t)
            phase_state_age_ms = max(0.0, float(sim_t - p_sim) * 1000.0)
            self.neighbor_state_cache[src] = {
                "responder_tls": str(src),
                "ts": float(now),
                "sim_time": float(sim_t),
                "phase": int(phase),
                "next_switch": float(next_switch),
                "phase_state_age_ms": float(phase_state_age_ms),
            }
            self._fed_evt(
                "coord.neighbor_state.in",
                source_tls=str(src),
                phase=int(phase),
                next_switch=float(next_switch),
                phase_state_age_ms=float(phase_state_age_ms),
            )
        except Exception:
            return

    def _latest_neighbor_state(
        self,
        max_age_sec: float = 4.0,
        responder_tls: Optional[str] = None,
    ) -> Optional[dict]:
        now = self._now()
        best = None
        best_ts = -1e18
        for tls_id, meta in getattr(self, "neighbor_state_cache", {}).items():
            if responder_tls is not None and str(tls_id) != str(responder_tls):
                continue
            age = float(now) - float(meta.get("ts", 0.0))
            if age > float(max_age_sec):
                continue
            ts = float(meta.get("ts", 0.0))
            if ts > best_ts:
                best = meta
                best_ts = ts
        return dict(best) if isinstance(best, dict) else None

    def on_passive_context(self, source_node: str, payload: dict, sim_time: Optional[float] = None) -> None:
        """Receive context from a passive non-TLS intersection observer."""
        try:
            src = str(source_node or payload.get("node_id", "") or payload.get("provider_id", "") or "").strip()
            if not src:
                return
            now = float(self._now())
            sim_t = float(sim_time if sim_time is not None else payload.get("sim_time", payload.get("simTime", now)))
            edges = [str(e) for e in list(payload.get("target_edges", payload.get("lookahead_edges", [])) or []) if str(e)]
            rec = dict(payload or {})
            rec["source_node"] = str(src)
            rec["ts"] = float(now)
            rec["sim_time"] = float(sim_t)
            rec["target_edges"] = list(edges)
            rec["context_id"] = str(
                payload.get("context_id", payload.get("request_id", f"passive:{src}:{payload.get('ev_id', '')}:{sim_t:.1f}"))
                or ""
            )
            self.passive_context_cache[src] = rec
            if len(self.passive_context_cache) > 512:
                for old_key in list(self.passive_context_cache.keys())[:128]:
                    self.passive_context_cache.pop(old_key, None)
            self._passive_context_trace(
                "f2.passive_context.in",
                aliases=("coord.passive_context.in",),
                request_id=str(rec.get("context_id", "")),
                source_node=str(src),
                provider_id=str(payload.get("provider_id", src) or src),
                requester_tls=str(self.cfg.tls_id),
                ev_id=str(payload.get("ev_id", "") or ""),
                blocked=bool(payload.get("blocked", False)),
                reason=str(payload.get("reason", "")),
                worst_edge=str(payload.get("worst_edge", "")),
                max_halt_n=int(payload.get("max_halt_n", 0) or 0),
                max_veh_n=int(payload.get("max_veh_n", 0) or 0),
                min_mean_speed_mps=float(payload.get("min_mean_speed_mps", -1.0) or -1.0),
                max_occupancy_pct=float(payload.get("max_occupancy_pct", 0.0) or 0.0),
                confidence=float(payload.get("confidence", 1.0) or 1.0),
                target_edges=list(edges),
                target_edges_n=int(len(edges)),
                selected_action="cache",
                decision_source="f2p_passive_context",
            )
        except Exception:
            return

    def _latest_passive_downstream_context(
        self,
        ev_id: str,
        max_age_sec: float = 5.0,
        max_route_edges: int = 8,
    ) -> Optional[dict]:
        now = self._now()
        route_edges = [str(e) for e in list(self._route_edges_hint(str(ev_id)) or []) if str(e)]
        if not route_edges:
            return None
        cur_edge = ""
        try:
            if traci is not None:
                cur_edge = str(traci.vehicle.getRoadID(str(ev_id)) or "")
        except Exception:
            cur_edge = ""
        start = 0
        if cur_edge and cur_edge in route_edges:
            start = min(len(route_edges), route_edges.index(cur_edge) + 1)
        tail = set(route_edges[start : min(len(route_edges), start + max(1, int(max_route_edges)))])
        if not tail:
            tail = set(route_edges[: max(1, int(max_route_edges))])
        best = None
        best_score = (-1, -1.0, -1, -1.0)
        for rec in list(getattr(self, "passive_context_cache", {}).values()):
            try:
                age = float(now) - float(rec.get("ts", 0.0))
            except Exception:
                continue
            if age > float(max_age_sec):
                continue
            edges = set(str(e) for e in list(rec.get("target_edges", rec.get("lookahead_edges", [])) or []) if str(e))
            if edges and not (edges & tail):
                continue
            blocked = 1 if bool(rec.get("blocked", False)) else 0
            spill = float(rec.get("downstream_spillback_risk", 0.0) or 0.0)
            halt = int(rec.get("max_halt_n", 0) or 0)
            occ = float(rec.get("max_occupancy_pct", 0.0) or 0.0)
            score = (blocked, spill, halt, occ)
            if score > best_score:
                best_score = score
                best = dict(rec)
                best["age_sec"] = float(age)
        return best

    def _passive_context_is_severe(self, rec: Optional[dict]) -> bool:
        """Return True only for passive observations strong enough to override active TLS feedback."""
        if not rec:
            return False
        if not bool(rec.get("blocked", False)):
            return False
        try:
            max_halt = int(rec.get("max_halt_n", 0) or 0)
        except Exception:
            max_halt = 0
        try:
            max_veh = int(rec.get("max_veh_n", 0) or 0)
        except Exception:
            max_veh = 0
        try:
            min_speed = float(rec.get("min_mean_speed_mps", -1.0) or -1.0)
        except Exception:
            min_speed = -1.0
        try:
            max_occ = float(rec.get("max_occupancy_pct", 0.0) or 0.0)
        except Exception:
            max_occ = 0.0
        halt_severe = max_halt >= int(getattr(self.cfg, "f2p_passive_context_severe_min_halt_n", 4))
        speed_severe = (
            max_veh >= int(getattr(self.cfg, "f2p_passive_context_severe_min_veh_n", 6))
            and min_speed >= 0.0
            and min_speed <= float(getattr(self.cfg, "f2p_passive_context_severe_max_mean_speed_mps", 0.5))
        )
        occ_severe = max_occ >= float(getattr(self.cfg, "f2p_passive_context_severe_max_occupancy_pct", 45.0))
        return bool(halt_severe or speed_severe or occ_severe)

    def _passive_context_trace(
        self,
        event_type: str,
        aliases: Tuple[str, ...] = (),
        **payload: object,
    ) -> None:
        """Emit F2P passive-context traces with legacy aliases for existing analysis scripts."""
        self._fed_evt(str(event_type), **payload)
        for alias in aliases:
            if str(alias) and str(alias) != str(event_type):
                self._fed_evt(str(alias), **payload)

    def _downstream_penalty_for_offer(
        self,
        offer: SignalWindowOffer,
        sim_time: float,
    ) -> Tuple[float, float, float, float]:
        """
        Returns:
          (queue_penalty, spillback_penalty, timing_penalty, no_feedback_penalty)
        """
        if self.active_ev is None:
            return 0.0, 0.0, 0.0, 0.0

        ev_id = str(self.active_ev.ev_id)
        cands = self.rank_next_hop_candidates(ev_id=ev_id, sim_time=float(sim_time), max_hops=1)
        preferred_nb = str(cands[0][0]) if cands else None
        fb = self._latest_downstream_feedback(
            ev_id=ev_id,
            max_age_sec=float(getattr(self.cfg, "robust_fed_resp_max_age_sec", 15.0)),
            responder_tls=preferred_nb,
        )
        if not fb:
            fb = self._latest_downstream_feedback(
                ev_id=ev_id,
                max_age_sec=float(getattr(self.cfg, "robust_fed_resp_max_age_sec", 15.0)),
                responder_tls=None,
            )
        has_active_feedback = bool(fb)
        passive_policy = str(
            getattr(self.cfg, "f2p_passive_context_policy", "severe_or_missing") or "severe_or_missing"
        ).strip().lower()
        if passive_policy not in {
            "disabled",
            "missing_feedback_only",
            "severe_or_missing",
            "immediate_missing_severe",
            "always",
        }:
            passive_policy = "immediate_missing_severe"
        passive_mode_enabled = str(getattr(self.cfg, "decision_log_run_label", "") or "").strip().upper() in {
            "F2P",
            "F2PD",
        }
        passive = self._latest_passive_downstream_context(
            ev_id=ev_id,
            max_age_sec=float(getattr(self.cfg, "f2p_passive_context_max_age_sec", 5.0)),
            max_route_edges=int(getattr(self.cfg, "f2p_passive_context_lookahead_edges", 4)),
        )
        passive_severe = self._passive_context_is_severe(passive)
        passive_for_trace = dict(passive) if passive else {}
        try:
            passive_worst_edge_offset = int(passive_for_trace.get("worst_edge_offset", -1) or -1)
        except Exception:
            passive_worst_edge_offset = -1
        passive_max_worst_edge_offset = max(
            1, int(getattr(self.cfg, "f2p_passive_context_max_worst_edge_offset", 1) or 1)
        )
        passive_immediate = bool(
            passive_worst_edge_offset > 0
            and passive_worst_edge_offset <= passive_max_worst_edge_offset
        )
        passive_decision = "none"
        passive_ignore_reason = ""
        if passive and not passive_mode_enabled:
            passive_ignore_reason = "passive_mode_disabled"
            passive = None
        elif passive_policy == "disabled" and passive:
            passive_ignore_reason = "policy_disabled"
            passive = None
        elif passive_policy == "missing_feedback_only" and has_active_feedback and passive:
            passive_ignore_reason = "active_feedback_present"
            passive = None
        elif passive_policy == "severe_or_missing" and has_active_feedback and passive and not passive_severe:
            passive_ignore_reason = "advisory_nonsevere_with_active_feedback"
            passive = None
        elif passive_policy == "immediate_missing_severe" and passive:
            if has_active_feedback:
                passive_ignore_reason = "active_feedback_present"
                passive = None
            elif not passive_severe:
                if not bool(getattr(self.cfg, "f2p_passive_context_clear_missing_feedback_enable", True)):
                    passive_ignore_reason = "nonsevere_missing_feedback"
                    passive = None
            elif not passive_immediate:
                passive_ignore_reason = "outside_immediate_passive_window"
                passive = None
        elif passive and not has_active_feedback and not passive_severe and passive_policy in {
            "missing_feedback_only",
            "severe_or_missing",
        }:
            if not bool(getattr(self.cfg, "f2p_passive_context_clear_missing_feedback_enable", True)):
                # A passive DT that reports clear/non-severe context should not
                # make F2P more pessimistic than F2 when active feedback is absent.
                passive_ignore_reason = "nonsevere_missing_feedback_safe_floor"
                passive = None
        if passive_ignore_reason:
            passive_age_sec = float(passive_for_trace.get("age_sec", -1.0) or -1.0)
            self._passive_context_trace(
                "f2.passive_context.ignored",
                aliases=("coord.passive_context.ignored",),
                request_id=str(passive_for_trace.get("context_id", passive_for_trace.get("request_id", "")) or ""),
                ev_id=str(ev_id),
                source_node=str(passive_for_trace.get("source_node", passive_for_trace.get("node_id", "")) or ""),
                provider_id=str(
                    passive_for_trace.get(
                        "provider_id",
                        passive_for_trace.get("source_node", passive_for_trace.get("node_id", "")),
                    )
                    or ""
                ),
                requester_tls=str(self.cfg.tls_id),
                mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                policy=str(passive_policy),
                reason=str(passive_ignore_reason),
                active_feedback_present=bool(has_active_feedback),
                passive_mode_enabled=bool(passive_mode_enabled),
                severe=bool(passive_severe),
                blocked=bool(passive_for_trace.get("blocked", False)),
                passive_reason=str(passive_for_trace.get("reason", "")),
                worst_edge=str(passive_for_trace.get("worst_edge", "")),
                target_edges=list(passive_for_trace.get("target_edges", []) or []),
                worst_edge_offset=int(passive_worst_edge_offset),
                max_worst_edge_offset=int(passive_max_worst_edge_offset),
                immediate=bool(passive_immediate),
                max_halt_n=int(passive_for_trace.get("max_halt_n", 0) or 0),
                max_veh_n=int(passive_for_trace.get("max_veh_n", 0) or 0),
                min_mean_speed_mps=float(passive_for_trace.get("min_mean_speed_mps", -1.0) or -1.0),
                max_occupancy_pct=float(passive_for_trace.get("max_occupancy_pct", 0.0) or 0.0),
                downstream_spillback_risk=float(passive_for_trace.get("downstream_spillback_risk", 0.0) or 0.0),
                context_age_ms=(float(passive_age_sec) * 1000.0 if passive_age_sec >= 0.0 else -1.0),
                confidence=float(passive_for_trace.get("confidence", 1.0) or 1.0),
                selected_action="ignore",
                decision_source="f2p_passive_context",
            )
        if not fb:
            if not passive:
                return 0.0, 0.0, 0.0, 1.0
            if not passive_severe:
                passive_decision = "clear_missing_feedback"
                no_feedback_penalty = max(
                    0.0,
                    min(
                        1.0,
                        float(
                            getattr(
                                self.cfg,
                                "f2p_passive_context_clear_missing_feedback_no_feedback_penalty",
                                0.25,
                            )
                            or 0.25
                        ),
                    ),
                )
                passive_age_sec = float(passive.get("age_sec", -1.0) or -1.0)
                self._passive_context_trace(
                    "f2.passive_context.used",
                    aliases=("coord.passive_context.used",),
                    request_id=str(passive.get("context_id", passive.get("request_id", "")) or ""),
                    ev_id=str(ev_id),
                    source_node=str(passive.get("source_node", passive.get("node_id", "")) or ""),
                    provider_id=str(passive.get("provider_id", passive.get("source_node", passive.get("node_id", ""))) or ""),
                    requester_tls=str(self.cfg.tls_id),
                    mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                    policy=str(passive_policy),
                    decision=str(passive_decision),
                    active_feedback_present=bool(has_active_feedback),
                    passive_mode_enabled=bool(passive_mode_enabled),
                    severe=bool(passive_severe),
                    blocked=bool(passive.get("blocked", False)),
                    reason=str(passive.get("reason", "")),
                    worst_edge=str(passive.get("worst_edge", "")),
                    target_edges=list(passive.get("target_edges", []) or []),
                    worst_edge_offset=int(passive_worst_edge_offset),
                    max_worst_edge_offset=int(passive_max_worst_edge_offset),
                    immediate=bool(passive_immediate),
                    max_halt_n=int(passive.get("max_halt_n", 0) or 0),
                    max_veh_n=int(passive.get("max_veh_n", 0) or 0),
                    min_mean_speed_mps=float(passive.get("min_mean_speed_mps", -1.0) or -1.0),
                    max_occupancy_pct=float(passive.get("max_occupancy_pct", 0.0) or 0.0),
                    downstream_queue_margin_sec=float(passive.get("downstream_queue_margin_sec", 0.0) or 0.0),
                    downstream_spillback_risk=float(passive.get("downstream_spillback_risk", 0.0) or 0.0),
                    passive_context_age_sec=float(passive_age_sec),
                    context_age_ms=(float(passive_age_sec) * 1000.0 if passive_age_sec >= 0.0 else -1.0),
                    confidence=float(passive.get("confidence", 1.0) or 1.0),
                    no_feedback_penalty=float(no_feedback_penalty),
                    selected_action=str(passive_decision),
                    decision_source="f2p_passive_context",
                )
                return 0.0, 0.0, 0.0, float(no_feedback_penalty)
            passive_decision = "substitute_missing_feedback"
            fb = {
                "downstream_queue_margin_sec": float(passive.get("downstream_queue_margin_sec", 0.0) or 0.0),
                "downstream_spillback_risk": float(passive.get("downstream_spillback_risk", 0.0) or 0.0),
                "req_eta_start": 0.0,
                "req_eta_end": 0.0,
                "downstream_suggested_eta_shift_sec": 0.0,
            }
        elif passive:
            passive_decision = "merge_severe" if passive_severe else "merge_policy_always"
            fb = dict(fb)
            fb["downstream_queue_margin_sec"] = min(
                float(fb.get("downstream_queue_margin_sec", 0.0) or 0.0),
                float(passive.get("downstream_queue_margin_sec", 0.0) or 0.0),
            )
            fb["downstream_spillback_risk"] = max(
                float(fb.get("downstream_spillback_risk", 0.0) or 0.0),
                float(passive.get("downstream_spillback_risk", 0.0) or 0.0),
            )
        if passive:
            passive_age_sec = float(passive.get("age_sec", -1.0) or -1.0)
            self._passive_context_trace(
                "f2.passive_context.used",
                aliases=("coord.passive_context.used",),
                request_id=str(passive.get("context_id", passive.get("request_id", "")) or ""),
                ev_id=str(ev_id),
                source_node=str(passive.get("source_node", passive.get("node_id", "")) or ""),
                provider_id=str(passive.get("provider_id", passive.get("source_node", passive.get("node_id", ""))) or ""),
                requester_tls=str(self.cfg.tls_id),
                mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                policy=str(passive_policy),
                decision=str(passive_decision),
                active_feedback_present=bool(has_active_feedback),
                passive_mode_enabled=bool(passive_mode_enabled),
                severe=bool(passive_severe),
                blocked=bool(passive.get("blocked", False)),
                reason=str(passive.get("reason", "")),
                worst_edge=str(passive.get("worst_edge", "")),
                target_edges=list(passive.get("target_edges", []) or []),
                worst_edge_offset=int(passive_worst_edge_offset),
                max_worst_edge_offset=int(passive_max_worst_edge_offset),
                immediate=bool(passive_immediate),
                max_halt_n=int(passive.get("max_halt_n", 0) or 0),
                max_veh_n=int(passive.get("max_veh_n", 0) or 0),
                min_mean_speed_mps=float(passive.get("min_mean_speed_mps", -1.0) or -1.0),
                max_occupancy_pct=float(passive.get("max_occupancy_pct", 0.0) or 0.0),
                downstream_queue_margin_sec=float(passive.get("downstream_queue_margin_sec", 0.0) or 0.0),
                downstream_spillback_risk=float(passive.get("downstream_spillback_risk", 0.0) or 0.0),
                passive_context_age_sec=float(passive_age_sec),
                context_age_ms=(float(passive_age_sec) * 1000.0 if passive_age_sec >= 0.0 else -1.0),
                confidence=float(passive.get("confidence", 1.0) or 1.0),
                selected_action=str(passive_decision),
                decision_source="f2p_passive_context",
            )

        q_margin = float(fb.get("downstream_queue_margin_sec", 0.0))
        spill = max(0.0, float(fb.get("downstream_spillback_risk", 0.0)))
        eta_s = float(fb.get("req_eta_start", 0.0))
        eta_e = float(fb.get("req_eta_end", 0.0))
        eta_shift = max(0.0, float(fb.get("downstream_suggested_eta_shift_sec", 0.0)))

        # Approximate downstream ETA under this offer:
        # base next-hop ETA + current-offer EV waiting effects.
        if cands:
            _nb, _p, eta_base = cands[0]
            eta_offer = float(eta_base) + max(0.0, float(offer.expected_wait_sec)) + max(0.0, float(offer.expected_miss_sec))
        else:
            eta_offer = 0.5 * (float(offer.arrival_window[0]) + float(offer.arrival_window[1]))

        eta_offer += float(eta_shift)

        early = max(0.0, float(eta_s - eta_offer))
        late = max(0.0, float(eta_offer - eta_e))
        timing_pen = max(early, late)

        if (
            passive_decision == "substitute_missing_feedback"
            and bool(getattr(self.cfg, "f2p_passive_context_missing_feedback_floor_enable", True))
        ):
            orig_q_margin = float(q_margin)
            orig_spill = float(spill)
            orig_timing = float(timing_pen)
            max_q_def = max(
                0.0,
                float(
                    getattr(
                        self.cfg,
                        "f2p_passive_context_missing_feedback_max_queue_deficit_sec",
                        2.0,
                    )
                    or 2.0
                ),
            )
            max_spill = max(
                0.0,
                float(
                    getattr(
                        self.cfg,
                        "f2p_passive_context_missing_feedback_max_spillback_risk",
                        0.15,
                    )
                    or 0.15
                ),
            )
            max_timing = max(
                0.0,
                float(
                    getattr(
                        self.cfg,
                        "f2p_passive_context_missing_feedback_max_timing_sec",
                        1.0,
                    )
                    or 1.0
                ),
            )
            q_margin = max(float(q_margin), -float(max_q_def))
            spill = min(float(spill), float(max_spill))
            timing_pen = min(float(timing_pen), float(max_timing))
            if (
                abs(float(q_margin) - orig_q_margin) > 1e-9
                or abs(float(spill) - orig_spill) > 1e-9
                or abs(float(timing_pen) - orig_timing) > 1e-9
            ):
                self._passive_context_trace(
                    "f2.passive_context.floor_applied",
                    aliases=("coord.passive_context.floor_applied",),
                    ev_id=str(ev_id),
                    requester_tls=str(self.cfg.tls_id),
                    mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                    policy=str(passive_policy),
                    decision=str(passive_decision),
                    original_queue_margin_sec=float(orig_q_margin),
                    capped_queue_margin_sec=float(q_margin),
                    original_spillback_risk=float(orig_spill),
                    capped_spillback_risk=float(spill),
                    original_timing_penalty_sec=float(orig_timing),
                    capped_timing_penalty_sec=float(timing_pen),
                    max_queue_deficit_sec=float(max_q_def),
                    max_spillback_risk=float(max_spill),
                    max_timing_sec=float(max_timing),
                    selected_action="cap_passive_penalty",
                    decision_source="f2p_passive_context_floor",
                )

        q_pen = max(0.0, -q_margin)
        return float(q_pen), float(spill), float(timing_pen), 0.0

    def robust_score_offer(self, offer: SignalWindowOffer) -> float:
        """
        Paper-facing multi-objective score combining EV service, safety, traffic impact,
        control effort, and federation reliability.
        Lower is better.
        """
        m = self._robust_metrics_for_offer(offer)
        hard_miss, hard_queue, hard_ttc, hard_speed = self._robust_hard_flags(offer, m)

        ttc_thr = float(getattr(self.cfg, "robust_ttc_threshold_sec", 5.0))
        ttc_step = float(getattr(self.cfg, "robust_ttc_step_sec", 0.1))

        cov_cost = max(0.0, 1.0 - float(m.window_coverage_ratio))
        wait_cost = self._robust_norm(m.ev_expected_wait_uniform_sec, getattr(self.cfg, "robust_scale_wait_sec", 10.0))
        miss_cost = max(0.0, float(m.ev_miss_probability_uniform))
        late_cost = self._robust_norm(m.ev_expected_late_uniform_sec, getattr(self.cfg, "robust_scale_late_sec", 10.0))

        qdef = max(0.0, -float(m.queue_clear_margin_sec))
        queue_cost = self._robust_norm(qdef, getattr(self.cfg, "robust_scale_queue_margin_sec", 20.0))

        non_ev_cost = self._robust_norm(m.non_ev_delay_impact_veh_sec, getattr(self.cfg, "robust_scale_non_ev_veh_sec", 400.0))
        spill_max_cost = max(0.0, float(m.spillback_risk_max))
        spill_mean_cost = max(0.0, float(m.spillback_risk_mean))
        effort_cost = self._robust_norm(m.control_effort, getattr(self.cfg, "robust_scale_control_effort", 40.0))

        ttc_cost = self._robust_norm(max(0.0, ttc_thr - float(m.ttc_sec)), max(1e-6, ttc_thr))
        tet_cost = self._robust_norm(m.tet_step_sec, max(1e-6, ttc_step))
        tit_cost = self._robust_norm(m.tit_step, max(1e-6, ttc_step * ttc_thr))

        speed_risk_cost = self._robust_norm(m.speed_advice_risk, getattr(self.cfg, "robust_scale_speed_risk", 1.0))

        fed_reject_cost = max(0.0, float(m.fed_reject_ratio))
        fed_age_cost = self._robust_norm(m.fed_mean_resp_age_sec, getattr(self.cfg, "robust_scale_fed_age_sec", 15.0))
        fed_active_cost = self._robust_norm(m.fed_active_reservations, getattr(self.cfg, "robust_scale_fed_active", 5.0))
        fed_accept_bonus = max(0.0, float(m.fed_accept_ratio))
        down_q_pen_raw, down_spill_raw, down_timing_raw, down_no_fb_raw = self._downstream_penalty_for_offer(
            offer=offer,
            sim_time=float(getattr(offer, "created_time", 0.0)),
        )
        down_q_pen = self._robust_norm(down_q_pen_raw, getattr(self.cfg, "robust_scale_fed_down_queue_sec", 20.0))
        down_spill_pen = max(0.0, float(down_spill_raw))
        down_timing_pen = self._robust_norm(down_timing_raw, getattr(self.cfg, "robust_scale_fed_down_timing_sec", 10.0))
        down_no_fb_pen = max(0.0, float(down_no_fb_raw))
        conf_risk = max(0.0, 1.0 - float(getattr(offer, "confidence", 0.5)))

        score = (
            float(getattr(self.cfg, "robust_w_cov", 1.0)) * cov_cost
            + float(getattr(self.cfg, "robust_w_wait", 1.0)) * wait_cost
            + float(getattr(self.cfg, "robust_w_miss_prob", 3.0)) * miss_cost
            + float(getattr(self.cfg, "robust_w_late", 1.0)) * late_cost
            + float(getattr(self.cfg, "robust_w_queue_margin", 1.5)) * queue_cost
            + float(getattr(self.cfg, "robust_w_non_ev", 1.0)) * non_ev_cost
            + float(getattr(self.cfg, "robust_w_spill_max", 1.0)) * spill_max_cost
            + float(getattr(self.cfg, "robust_w_spill_mean", 0.5)) * spill_mean_cost
            + float(getattr(self.cfg, "robust_w_effort", 0.3)) * effort_cost
            + float(getattr(self.cfg, "robust_w_ttc", 2.0)) * ttc_cost
            + float(getattr(self.cfg, "robust_w_tet", 0.5)) * tet_cost
            + float(getattr(self.cfg, "robust_w_tit", 0.5)) * tit_cost
            + float(getattr(self.cfg, "robust_w_speed_risk", 1.0)) * speed_risk_cost
            + float(getattr(self.cfg, "robust_w_fed_reject", 0.75)) * fed_reject_cost
            + float(getattr(self.cfg, "robust_w_fed_age", 0.25)) * fed_age_cost
            + float(getattr(self.cfg, "robust_w_fed_active", 0.1)) * fed_active_cost
            + float(getattr(self.cfg, "robust_w_fed_down_queue", 1.2)) * down_q_pen
            + float(getattr(self.cfg, "robust_w_fed_down_spill", 1.2)) * down_spill_pen
            + float(getattr(self.cfg, "robust_w_fed_down_timing", 0.8)) * down_timing_pen
            + float(getattr(self.cfg, "robust_w_fed_no_feedback", 0.25)) * down_no_fb_pen
            + float(getattr(self.cfg, "robust_w_conf_risk", 0.15)) * conf_risk
            - float(getattr(self.cfg, "robust_w_fed_accept", 0.2)) * fed_accept_bonus
        )

        # Hard constraints become large penalties so ordering remains robust.
        if not bool(offer.feasible):
            score += 1e6
        if hard_miss:
            score += 5e5
        if hard_queue:
            score += 4e5
        if hard_ttc:
            score += 6e5
        if hard_speed:
            score += 2e5
        if (-float(down_q_pen_raw)) < float(getattr(self.cfg, "robust_fed_down_hard_queue_margin_sec", -2.0)):
            score += 3e5
        if down_spill_raw > float(getattr(self.cfg, "robust_fed_down_hard_spillback", 0.85)):
            score += 3e5

        return float(score)

    def _robust_offer_rank_key(self, offer: SignalWindowOffer) -> Tuple[float, ...]:
        m = self._robust_metrics_for_offer(offer)
        hard_miss, hard_queue, hard_ttc, hard_speed = self._robust_hard_flags(offer, m)
        return (
            0.0 if bool(offer.feasible) else 1.0,
            float(hard_miss),
            float(hard_queue),
            float(hard_ttc),
            float(hard_speed),
            float(round(self.robust_score_offer(offer), 6)),
            float(round(m.control_effort, 3)),
            float(round(self.improved_score_offer(offer), 6)),
        )

    def _improved_offer_rank_key(self, offer: SignalWindowOffer) -> Tuple[float, ...]:
        """
        Lexicographic key:
        1) Feasibility
        2) Hard EV constraints (miss, wait)
        3) EV objective bucket
        4) Disruption bucket
        5) Actuation magnitude (smaller preferred when EV metrics tie)
        6) Severity class + magnitude
        6) Final numeric tie-break
        """
        wait = max(0.0, float(offer.expected_wait_sec))
        miss = max(0.0, float(offer.expected_miss_sec))
        dis = max(0.0, float(offer.cost_to_others_veh_sec))
        sev_cls, sev_mag = self._offer_severity(offer)
        act_mag = self._offer_actuation_magnitude(offer)

        hard_wait = float(getattr(self.cfg, "offer_hard_wait_sec", 6.0))
        hard_miss = float(getattr(self.cfg, "offer_hard_miss_sec", 0.5))
        ev_eps = max(1e-6, float(getattr(self.cfg, "offer_ev_epsilon_sec", 0.25)))
        dis_eps = max(1e-6, float(getattr(self.cfg, "offer_disruption_epsilon_vehs", 2.0)))

        ev_obj = float(getattr(self.cfg, "offer_w_wait", 1.0)) * wait + float(getattr(self.cfg, "offer_w_miss", 4.0)) * miss

        return (
            0.0 if bool(offer.feasible) else 1.0,
            0.0 if miss <= hard_miss else 1.0,
            0.0 if wait <= hard_wait else 1.0,
            float(int(ev_obj / ev_eps)),
            float(int(dis / dis_eps)),
            float(round(act_mag, 3)),
            float(sev_cls),
            float(round(sev_mag, 3)),
            float(round(ev_obj, 3)),
            float(round(dis, 3)),
            float(round(self.improved_score_offer(offer), 6)),
        )

    def _offer_selection_key(self, offer: SignalWindowOffer):
        strategy = str(getattr(self.cfg, "offer_score_strategy", "improved_lexicographic")).strip().lower()
        if strategy == "legacy":
            return float(self._score_offer_legacy(offer))
        if strategy in ("robust_research", "robust"):
            return self._robust_offer_rank_key(offer)
        return self._improved_offer_rank_key(offer)

    def score_offer(self, offer: SignalWindowOffer) -> float:
        """
        Public score hook (kept for compatibility with existing callers).
        Returns numeric score for logs and external sorting APIs.
        """
        strategy = str(getattr(self.cfg, "offer_score_strategy", "improved_lexicographic")).strip().lower()
        if strategy == "legacy":
            return float(self._score_offer_legacy(offer))
        if strategy in ("robust_research", "robust"):
            return float(self.robust_score_offer(offer))
        return float(self.improved_score_offer(offer))


    def pick_best_offer(self, offers: List[SignalWindowOffer]) -> Optional[SignalWindowOffer]:
        if not offers:
            return None
        best = min(offers, key=self._offer_selection_key)
        return best

    def pick_offer_for_current_plan(self, offers: List[SignalWindowOffer]) -> Optional[SignalWindowOffer]:
        """Return the offer that corresponds to the current local plan (B1-equivalent anchor)."""
        if not offers or self.current_plan is None:
            return None
        sig_cur = self._plan_signature(self.current_plan)
        for o in offers:
            try:
                oid = str(getattr(o, "offer_id", "") or "")
                p = self._offer_plan_cache.get(oid)
                if p is None:
                    continue
                if self._plan_signature(p) == sig_cur:
                    return o
            except Exception:
                continue
        return None

    def _best_effective_offer(self, offers: List[SignalWindowOffer]) -> Optional[SignalWindowOffer]:
        effective = [o for o in list(offers or []) if self._offer_has_effective_actuation(o)]
        if not effective:
            return None
        return min(effective, key=self._offer_selection_key)

    def _resolve_f2_local_anchor(
        self,
        offers: List[SignalWindowOffer],
        local_anchor: Optional[SignalWindowOffer],
        local_best: Optional[SignalWindowOffer],
        ev_id: str,
        sim_time: float,
    ) -> Optional[SignalWindowOffer]:
        """
        Build the B1-equivalent local floor used by F2.

        The direct current-plan match is preferred. If it is unavailable or maps
        to a no-op while the EV is near the current TLS, promote the best
        effective local offer so federation refinement cannot suppress local
        EV support with a cheaper but weaker no-op.
        """
        if self._offer_has_effective_actuation(local_anchor):
            return local_anchor

        active_ev = self.active_ev
        ev_dist = -1.0
        if active_ev is not None:
            try:
                ev_dist = float(getattr(active_ev, "distance_to_intersection_m", -1.0))
            except Exception:
                ev_dist = -1.0

        anchor_dist = float(getattr(self.cfg, "f2_local_priority_floor_distance_m", -1.0))
        if anchor_dist <= 0.0:
            anchor_dist = max(120.0, float(getattr(self.cfg, "fed_req_send_min_gap_near_distance_m", 120.0)))
        near_current_tls = ev_dist >= 0.0 and ev_dist <= max(0.0, anchor_dist)

        current_plan_type = str(getattr(self.current_plan, "plan_type", "") or "")
        promoted: Optional[SignalWindowOffer] = None
        if current_plan_type not in ("", "none", "restore"):
            same_type = [
                o for o in list(offers or [])
                if self._offer_has_effective_actuation(o)
                and str(self._plan_type_from_offer(o)) == current_plan_type
            ]
            if same_type:
                promoted = min(same_type, key=self._offer_selection_key)

        if promoted is None and near_current_tls:
            promoted = self._best_effective_offer(offers)

        if promoted is not None:
            self._fed_evt(
                "coord.refine.local_anchor_promoted",
                ev_id=str(ev_id),
                sim_time=float(sim_time),
                ev_distance_m=float(ev_dist),
                anchor_distance_m=float(anchor_dist),
                previous_anchor_offer_id=str(getattr(local_anchor, "offer_id", "") if local_anchor is not None else ""),
                previous_anchor_plan_type=str(self._plan_type_from_offer(local_anchor) if local_anchor is not None else ""),
                local_best_offer_id=str(getattr(local_best, "offer_id", "") if local_best is not None else ""),
                local_best_plan_type=str(self._plan_type_from_offer(local_best) if local_best is not None else ""),
                promoted_offer_id=str(getattr(promoted, "offer_id", "") or ""),
                promoted_plan_type=str(self._plan_type_from_offer(promoted)),
                current_plan_type=str(current_plan_type),
                reason="current_plan_effective" if current_plan_type not in ("", "none", "restore") else "near_current_tls_effective_offer",
            )
            return promoted

        return local_anchor

    def prefer_local_offer_if_ev_worse(
        self,
        chosen_offer: Optional[SignalWindowOffer],
        local_anchor_offer: Optional[SignalWindowOffer],
        stage: str = "",
    ) -> Optional[SignalWindowOffer]:
        """
        Guardrail for F2: keep local baseline when selected offer is significantly worse for EV.
        This keeps F2 "at least local" in expected EV waiting/miss terms.
        """
        if not bool(getattr(self.cfg, "f2_ev_guard_enable", True)):
            return chosen_offer
        if local_anchor_offer is None:
            return chosen_offer
        if chosen_offer is None:
            return local_anchor_offer

        req_feas = bool(getattr(self.cfg, "f2_ev_guard_require_feasible", True))
        if req_feas and (not bool(getattr(chosen_offer, "feasible", False))) and bool(getattr(local_anchor_offer, "feasible", False)):
            self._fed_dbg(
                f"f2_guard stage={stage} action=fallback reason=chosen_infeasible local_feasible=1"
            )
            return local_anchor_offer

        c_wait = max(0.0, float(getattr(chosen_offer, "expected_wait_sec", 0.0) or 0.0))
        c_miss = max(0.0, float(getattr(chosen_offer, "expected_miss_sec", 0.0) or 0.0))
        l_wait = max(0.0, float(getattr(local_anchor_offer, "expected_wait_sec", 0.0) or 0.0))
        l_miss = max(0.0, float(getattr(local_anchor_offer, "expected_miss_sec", 0.0) or 0.0))
        wait_eps = max(0.0, float(getattr(self.cfg, "f2_ev_guard_wait_penalty_sec", 2.0)))
        miss_eps = max(0.0, float(getattr(self.cfg, "f2_ev_guard_miss_penalty_sec", 0.3)))

        if (c_wait > (l_wait + wait_eps)) or (c_miss > (l_miss + miss_eps)):
            self._fed_dbg(
                f"f2_guard stage={stage} action=fallback reason=ev_penalty "
                f"chosen_wait={c_wait:.2f} local_wait={l_wait:.2f} wait_eps={wait_eps:.2f} "
                f"chosen_miss={c_miss:.2f} local_miss={l_miss:.2f} miss_eps={miss_eps:.2f}"
            )
            return local_anchor_offer

        return chosen_offer

    def _offer_ev_cost(self, offer: Optional[SignalWindowOffer]) -> float:
        if offer is None:
            return float("inf")
        wait = max(0.0, float(getattr(offer, "expected_wait_sec", 0.0) or 0.0))
        miss = max(0.0, float(getattr(offer, "expected_miss_sec", 0.0) or 0.0))
        return float(wait + miss)

    def _offer_robust_cost(self, offer: Optional[SignalWindowOffer]) -> float:
        if offer is None:
            return float("inf")
        try:
            return float(self.robust_score_offer(offer))
        except Exception:
            try:
                return float(self.improved_score_offer(offer))
            except Exception:
                return float("inf")

    def _offer_compare_tuple(self, offer: Optional[SignalWindowOffer]) -> Tuple[int, float, float]:
        """
        Lower tuple is better:
        1) feasibility (feasible preferred)
        2) robust measurable multi-objective score
        3) EV-local direct service cost (wait+miss) for deterministic tie-break
        """
        if offer is None:
            return (2, float("inf"), float("inf"))
        feas_rank = 0 if bool(getattr(offer, "feasible", False)) else 1
        return (
            int(feas_rank),
            float(self._offer_robust_cost(offer)),
            float(self._offer_ev_cost(offer)),
        )

    def _select_offer_with_policy(
        self,
        candidate_offer: Optional[SignalWindowOffer],
        local_anchor_offer: Optional[SignalWindowOffer],
        stage: str = "",
    ) -> Optional[SignalWindowOffer]:
        """
        Select between candidate and local anchor.
        - legacy_guard: historical threshold guard behavior.
        - measured: feasibility + robust measurable score (no epsilon thresholds).
        """
        if local_anchor_offer is None:
            return candidate_offer
        if candidate_offer is None:
            return local_anchor_offer

        # F2 should improve B1 with peer context, not replace an actionable
        # local plan with a no-op offer that only looks good because it is less
        # disruptive. Keep the local anchor when the candidate cannot actuate.
        if self._offer_has_effective_actuation(local_anchor_offer) and not self._offer_has_effective_actuation(candidate_offer):
            self._fed_evt(
                "coord.refine.selection_noop_candidate_fallback",
                stage=str(stage or ""),
                candidate_offer_id=str(getattr(candidate_offer, "offer_id", "") or ""),
                local_offer_id=str(getattr(local_anchor_offer, "offer_id", "") or ""),
                candidate_plan_type=str(self._plan_type_from_offer(candidate_offer)),
                local_plan_type=str(self._plan_type_from_offer(local_anchor_offer)),
            )
            return local_anchor_offer

        # Preserve the current-TLS local priority floor. A federation-refined
        # offer may be less disruptive, but it must not weaken a local
        # saturation/current-green extension unless it measurably improves EV
        # service. This is the B1 + downstream coordination invariant.
        local_plan_type = str(self._plan_type_from_offer(local_anchor_offer))
        candidate_plan_type = str(self._plan_type_from_offer(candidate_offer))
        if local_plan_type == "saturation_reduction" and candidate_plan_type != "saturation_reduction":
            try:
                local_ext = float((getattr(local_anchor_offer, "action_params", {}) or {}).get("ext", 0.0) or 0.0)
            except Exception:
                local_ext = 0.0
            try:
                cand_ext = float((getattr(candidate_offer, "action_params", {}) or {}).get("ext", 0.0) or 0.0)
            except Exception:
                cand_ext = 0.0
            c_wait = max(0.0, float(getattr(candidate_offer, "expected_wait_sec", 0.0) or 0.0))
            c_miss = max(0.0, float(getattr(candidate_offer, "expected_miss_sec", 0.0) or 0.0))
            l_wait = max(0.0, float(getattr(local_anchor_offer, "expected_wait_sec", 0.0) or 0.0))
            l_miss = max(0.0, float(getattr(local_anchor_offer, "expected_miss_sec", 0.0) or 0.0))
            wait_eps = max(0.0, float(getattr(self.cfg, "f2_ev_guard_wait_penalty_sec", 2.0)))
            miss_eps = max(0.0, float(getattr(self.cfg, "f2_ev_guard_miss_penalty_sec", 0.3)))
            ext_eps = max(0.0, float(getattr(self.cfg, "f2_local_priority_floor_ext_eps_sec", 0.5)))
            candidate_ev_better = (c_wait + wait_eps < l_wait) or (c_miss + miss_eps < l_miss)
            if (not candidate_ev_better) and local_ext > cand_ext + ext_eps:
                self._fed_evt(
                    "coord.refine.selection_local_priority_floor",
                    stage=str(stage or ""),
                    reason="preserve_saturation_extension",
                    candidate_offer_id=str(getattr(candidate_offer, "offer_id", "") or ""),
                    local_offer_id=str(getattr(local_anchor_offer, "offer_id", "") or ""),
                    candidate_plan_type=str(candidate_plan_type),
                    local_plan_type=str(local_plan_type),
                    candidate_extend_sec=float(cand_ext),
                    local_extend_sec=float(local_ext),
                    candidate_wait_sec=float(c_wait),
                    local_wait_sec=float(l_wait),
                    candidate_miss_sec=float(c_miss),
                    local_miss_sec=float(l_miss),
                )
                return local_anchor_offer

        policy = str(getattr(self.cfg, "f2_selection_policy", "measured") or "measured").strip().lower()
        if policy == "legacy_guard":
            return self.prefer_local_offer_if_ev_worse(
                chosen_offer=candidate_offer,
                local_anchor_offer=local_anchor_offer,
                stage=stage,
            )

        cand_cmp = self._offer_compare_tuple(candidate_offer)
        loc_cmp = self._offer_compare_tuple(local_anchor_offer)
        chosen = local_anchor_offer if (loc_cmp <= cand_cmp) else candidate_offer
        if chosen is candidate_offer:
            robust_improvement = float(loc_cmp[1]) - float(cand_cmp[1])
            c_wait = max(0.0, float(getattr(candidate_offer, "expected_wait_sec", 0.0) or 0.0))
            c_miss = max(0.0, float(getattr(candidate_offer, "expected_miss_sec", 0.0) or 0.0))
            l_wait = max(0.0, float(getattr(local_anchor_offer, "expected_wait_sec", 0.0) or 0.0))
            l_miss = max(0.0, float(getattr(local_anchor_offer, "expected_miss_sec", 0.0) or 0.0))
            wait_improvement = float(l_wait) - float(c_wait)
            miss_improvement = float(l_miss) - float(c_miss)
            min_robust = max(
                0.0,
                float(getattr(self.cfg, "f2_measured_override_min_robust_improvement", 0.0) or 0.0),
            )
            min_wait = max(
                0.0,
                float(getattr(self.cfg, "f2_measured_override_min_ev_wait_improvement_sec", 0.0) or 0.0),
            )
            min_miss = max(
                0.0,
                float(getattr(self.cfg, "f2_measured_override_min_ev_miss_improvement_sec", 0.0) or 0.0),
            )
            robust_ok = bool(robust_improvement >= min_robust)
            ev_ok = bool((wait_improvement >= min_wait and min_wait > 0.0) or (miss_improvement >= min_miss and min_miss > 0.0))
            if min_robust > 0.0 and not (robust_ok or ev_ok):
                self._fed_evt(
                    "coord.refine.selection_margin_fallback",
                    stage=str(stage or ""),
                    policy=str(policy),
                    reason="insufficient_measured_override_margin",
                    candidate_offer_id=str(getattr(candidate_offer, "offer_id", "") or ""),
                    local_offer_id=str(getattr(local_anchor_offer, "offer_id", "") or ""),
                    candidate_plan_type=str(self._plan_type_from_offer(candidate_offer)),
                    local_plan_type=str(self._plan_type_from_offer(local_anchor_offer)),
                    robust_improvement=float(robust_improvement),
                    min_robust_improvement=float(min_robust),
                    wait_improvement_sec=float(wait_improvement),
                    min_wait_improvement_sec=float(min_wait),
                    miss_improvement_sec=float(miss_improvement),
                    min_miss_improvement_sec=float(min_miss),
                    candidate_robust_score=float(cand_cmp[1]),
                    local_robust_score=float(loc_cmp[1]),
                    candidate_ev_cost=float(cand_cmp[2]),
                    local_ev_cost=float(loc_cmp[2]),
                )
                chosen = local_anchor_offer

        # Even in measured policy, keep the EV-penalty guard as a final safety net.
        # This does not force "F2 wins"; it only avoids clearly worse EV-local outcomes.
        chosen_before_guard = chosen
        chosen = self.prefer_local_offer_if_ev_worse(
            chosen_offer=chosen,
            local_anchor_offer=local_anchor_offer,
            stage=f"{stage}:measured_guard",
        )
        chosen_before_id = str(getattr(chosen_before_guard, "offer_id", "") or "")
        chosen_after_id = str(getattr(chosen, "offer_id", "") or "")
        if chosen_before_id != chosen_after_id:
            self._fed_evt(
                "coord.refine.selection_measured_guard_fallback",
                stage=str(stage or ""),
                policy=str(policy),
                previous_offer_id=str(chosen_before_id),
                selected_offer_id=str(chosen_after_id),
                previous_wait_sec=float(max(0.0, float(getattr(chosen_before_guard, "expected_wait_sec", 0.0) or 0.0))),
                previous_miss_sec=float(max(0.0, float(getattr(chosen_before_guard, "expected_miss_sec", 0.0) or 0.0))),
                local_wait_sec=float(max(0.0, float(getattr(local_anchor_offer, "expected_wait_sec", 0.0) or 0.0))),
                local_miss_sec=float(max(0.0, float(getattr(local_anchor_offer, "expected_miss_sec", 0.0) or 0.0))),
                wait_eps=float(max(0.0, float(getattr(self.cfg, "f2_ev_guard_wait_penalty_sec", 2.0)))),
                miss_eps=float(max(0.0, float(getattr(self.cfg, "f2_ev_guard_miss_penalty_sec", 0.3)))),
            )

        self._fed_evt(
            "coord.refine.selection_compare",
            stage=str(stage or ""),
            policy=str(policy),
            candidate_offer_id=str(getattr(candidate_offer, "offer_id", "") or ""),
            local_offer_id=str(getattr(local_anchor_offer, "offer_id", "") or ""),
            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
            candidate_feasible=int(bool(getattr(candidate_offer, "feasible", False))),
            local_feasible=int(bool(getattr(local_anchor_offer, "feasible", False))),
            candidate_wait_sec=float(max(0.0, float(getattr(candidate_offer, "expected_wait_sec", 0.0) or 0.0))),
            candidate_miss_sec=float(max(0.0, float(getattr(candidate_offer, "expected_miss_sec", 0.0) or 0.0))),
            local_wait_sec=float(max(0.0, float(getattr(local_anchor_offer, "expected_wait_sec", 0.0) or 0.0))),
            local_miss_sec=float(max(0.0, float(getattr(local_anchor_offer, "expected_miss_sec", 0.0) or 0.0))),
            candidate_robust_score=float(cand_cmp[1]),
            local_robust_score=float(loc_cmp[1]),
            candidate_ev_cost=float(cand_cmp[2]),
            local_ev_cost=float(loc_cmp[2]),
        )
        return chosen

    def _approach_loop_coverage_ratio(self, ev: Optional[EvRequest]) -> float:
        if ev is None:
            return 1.0
        edge_id = str(getattr(ev, "in_edge_id", "") or "")
        if (not edge_id) or edge_id.startswith(":"):
            return 0.0
        tc = self._resolve_traci()
        if tc is None:
            return 0.0
        try:
            nlanes = max(1, int(tc.edge.getLaneNumber(edge_id)))
        except Exception:
            nlanes = 1
        lanes = [f"{edge_id}_{i}" for i in range(int(nlanes))]
        if not lanes:
            return 0.0
        self._refresh_lane_loop_map()
        mapped = 0
        for lid in lanes:
            loop_ids = list(self._lane_loop_ids.get(str(lid), []) or [])
            if loop_ids:
                mapped += 1
        return float(mapped) / float(max(1, len(lanes)))

    def _should_refine_with_federation(
        self,
        sim_time: float,
        ev_id: str,
    ) -> Tuple[bool, str, Dict[str, float]]:
        """
        Determine whether federation refine should run this tick based on context quality.
        Returns (allow, reason, diagnostics).
        """
        diag: Dict[str, float] = {
            "feedback_age_sec": -1.0,
            "feedback_max_age_sec": -1.0,
            "loop_coverage_ratio": -1.0,
            "responder_phase_state_age_ms": -1.0,
            "ev_distance_m": -1.0,
            "feedback_is_preferred": -1.0,
            "preferred_next_tls": "",
            "feedback_responder_tls": "",
            "feedback_seen_any": 0.0,
            "bootstrap_allowed": 0.0,
            "neighbor_state_used": 0.0,
            "neighbor_state_age_sec": -1.0,
            "neighbor_state_phase_state_age_ms": -1.0,
        }
        preferred_nb: Optional[str] = None
        if self.active_ev is not None:
            try:
                diag["ev_distance_m"] = float(getattr(self.active_ev, "distance_to_intersection_m", -1.0))
            except Exception:
                diag["ev_distance_m"] = -1.0
            try:
                cands = self.rank_next_hop_candidates(ev_id=str(ev_id), sim_time=float(sim_time), max_hops=1)
                if cands:
                    preferred_nb = str(cands[0][0])
                    diag["preferred_next_tls"] = str(preferred_nb)
            except Exception:
                preferred_nb = None

        if bool(getattr(self.cfg, "f2_refine_require_feedback", True)):
            base_max_age = max(0.1, float(getattr(self.cfg, "f2_refine_feedback_max_age_sec", 6.0)))
            max_age = float(base_max_age)
            if bool(getattr(self.cfg, "f2_refine_feedback_age_adaptive_enable", True)):
                ev_dist = float(diag.get("ev_distance_m", -1.0))
                near_dist = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
                far_dist = max(near_dist + 1.0, float(getattr(self.cfg, "f2_refine_feedback_adaptive_far_distance_m", 250.0)))
                near_age_cfg = float(getattr(self.cfg, "f2_refine_feedback_max_age_near_sec", -1.0))
                far_age_cfg = float(getattr(self.cfg, "f2_refine_feedback_max_age_far_sec", -1.0))
                near_age = near_age_cfg if near_age_cfg > 0.0 else max(0.1, 0.75 * base_max_age)
                far_age = far_age_cfg if far_age_cfg > 0.0 else max(near_age, 1.50 * base_max_age)
                if ev_dist >= 0.0:
                    if ev_dist <= near_dist:
                        max_age = float(near_age)
                    elif ev_dist >= far_dist:
                        max_age = float(far_age)
                    else:
                        alpha = (ev_dist - near_dist) / max(1e-6, (far_dist - near_dist))
                        max_age = float(near_age + alpha * (far_age - near_age))
            diag["feedback_max_age_sec"] = float(max_age)
            fb = None
            if preferred_nb:
                fb = self._latest_downstream_feedback(
                    ev_id=str(ev_id),
                    max_age_sec=max_age,
                    responder_tls=preferred_nb,
                )
            if fb is None:
                fb = self._latest_downstream_feedback(ev_id=str(ev_id), max_age_sec=max_age)
            if fb is None:
                has_any_feedback = False
                for _meta in getattr(self, "resp_cache", {}).values():
                    if str(_meta.get("ev_id", "")) == str(ev_id):
                        has_any_feedback = True
                        break
                diag["feedback_seen_any"] = 1.0 if has_any_feedback else 0.0

                # Bootstrap-safe gate: allow limited refine before first-ever feedback
                # to break request/feedback deadlock in coordinated startup.
                if bool(getattr(self.cfg, "f2_refine_feedback_bootstrap_enable", True)) and not has_any_feedback:
                    ev_dist = float(diag.get("ev_distance_m", -1.0))
                    dist_thr = max(0.0, float(getattr(self.cfg, "f2_refine_feedback_bootstrap_distance_m", 450.0)))
                    max_boot_age = max(0.1, float(getattr(self.cfg, "f2_refine_feedback_bootstrap_max_age_sec", 20.0)))
                    ev_req_age = max(0.0, float(self._now()) - float(getattr(self.active_ev, "sim_time", self._now()))) if self.active_ev is not None else 1e9
                    if ev_dist >= 0.0 and ev_dist <= dist_thr and ev_req_age <= max_boot_age:
                        diag["bootstrap_allowed"] = 1.0
                        return True, "bootstrap_no_feedback", diag

                # Optional fallback: allow refine using fresh neighboring TLS live state
                # when reservation feedback is not yet available.
                if bool(getattr(self.cfg, "f2_refine_neighbor_state_fallback_enable", True)):
                    ev_dist = float(diag.get("ev_distance_m", -1.0))
                    near_dist = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
                    near_mode = (ev_dist >= 0.0 and ev_dist <= near_dist)
                    ns_max_age = max(
                        0.1,
                        float(
                            getattr(
                                self.cfg,
                                "f2_refine_neighbor_state_near_max_age_sec" if near_mode else "f2_refine_neighbor_state_max_age_sec",
                                1.5 if near_mode else 4.0,
                            )
                        ),
                    )
                    ns = None
                    if preferred_nb:
                        ns = self._latest_neighbor_state(max_age_sec=ns_max_age, responder_tls=preferred_nb)
                    if ns is None:
                        ns = self._latest_neighbor_state(max_age_sec=ns_max_age)
                    if ns is not None:
                        ns_src = str(ns.get("responder_tls", "") or "")
                        ns_age = max(0.0, float(self._now()) - float(ns.get("ts", self._now())))
                        ns_phase_age_ms = float(ns.get("phase_state_age_ms", ns_age * 1000.0))
                        diag["neighbor_state_used"] = 1.0
                        diag["neighbor_state_age_sec"] = float(ns_age)
                        diag["neighbor_state_phase_state_age_ms"] = float(ns_phase_age_ms)
                        diag["feedback_responder_tls"] = str(ns_src)
                        if preferred_nb:
                            diag["feedback_is_preferred"] = 1.0 if str(ns_src) == str(preferred_nb) else 0.0
                        diag["responder_phase_state_age_ms"] = float(ns_phase_age_ms)

                        near_for_preferred = max(
                            0.0,
                            float(getattr(self.cfg, "f2_refine_preferred_feedback_near_distance_m", 60.0)),
                        )
                        if bool(getattr(self.cfg, "f2_refine_require_preferred_feedback_when_near", True)):
                            if preferred_nb and float(diag["ev_distance_m"]) >= 0.0 and float(diag["ev_distance_m"]) <= near_for_preferred:
                                if str(ns_src) != str(preferred_nb):
                                    return False, "stale_feedback_gate_preferred_mismatch", diag

                        if bool(getattr(self.cfg, "f2_refine_stale_feedback_gate_enable", True)):
                            max_phase_age_ms = max(
                                1.0,
                                float(getattr(self.cfg, "f2_refine_max_responder_phase_state_age_ms", 4000.0)),
                            )
                            near_dist_m = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
                            near_max_phase_age_ms = max(
                                1.0,
                                float(getattr(self.cfg, "f2_refine_near_max_responder_phase_state_age_ms", 1200.0)),
                            )
                            if ns_phase_age_ms > 0.0:
                                if float(diag["ev_distance_m"]) >= 0.0 and float(diag["ev_distance_m"]) <= near_dist_m:
                                    if ns_phase_age_ms > near_max_phase_age_ms:
                                        return False, "stale_feedback_gate_near", diag
                                elif ns_phase_age_ms > max_phase_age_ms:
                                    return False, "stale_feedback_gate", diag
                        return True, "neighbor_state_fallback", diag
                return False, "no_recent_feedback", diag
            try:
                age = float(self._now()) - float(fb.get("ts", self._now()))
            except Exception:
                age = -1.0
            diag["feedback_age_sec"] = float(age)
            fb_responder = str(fb.get("responder_tls", "") or "")
            diag["feedback_responder_tls"] = fb_responder
            if preferred_nb:
                diag["feedback_is_preferred"] = 1.0 if str(fb_responder) == str(preferred_nb) else 0.0
            try:
                phase_age_ms = float(fb.get("responder_phase_state_age_ms", -1.0))
            except Exception:
                phase_age_ms = -1.0
            diag["responder_phase_state_age_ms"] = float(phase_age_ms)

            # Handoff-aware gate near the stopline: avoid refining with stale/wrong-responder context.
            near_for_preferred = max(
                0.0,
                float(getattr(self.cfg, "f2_refine_preferred_feedback_near_distance_m", 60.0)),
            )
            if bool(getattr(self.cfg, "f2_refine_require_preferred_feedback_when_near", True)):
                if preferred_nb and float(diag["ev_distance_m"]) >= 0.0 and float(diag["ev_distance_m"]) <= near_for_preferred:
                    if str(fb_responder) != str(preferred_nb):
                        return False, "stale_feedback_gate_preferred_mismatch", diag

            if bool(getattr(self.cfg, "f2_refine_stale_feedback_gate_enable", True)):
                max_phase_age_ms = max(
                    1.0,
                    float(getattr(self.cfg, "f2_refine_max_responder_phase_state_age_ms", 4000.0)),
                )
                near_dist_m = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
                near_max_phase_age_ms = max(
                    1.0,
                    float(getattr(self.cfg, "f2_refine_near_max_responder_phase_state_age_ms", 1200.0)),
                )
                if phase_age_ms > 0.0:
                    if float(diag["ev_distance_m"]) >= 0.0 and float(diag["ev_distance_m"]) <= near_dist_m:
                        if phase_age_ms > near_max_phase_age_ms:
                            return False, "stale_feedback_gate_near", diag
                    elif phase_age_ms > max_phase_age_ms:
                        return False, "stale_feedback_gate", diag

        if bool(getattr(self.cfg, "f2_refine_require_loop_coverage", True)):
            cov = float(self._approach_loop_coverage_ratio(self.active_ev))
            diag["loop_coverage_ratio"] = float(cov)
            cov_min = max(0.0, min(1.0, float(getattr(self.cfg, "f2_refine_min_loop_coverage_ratio", 0.5))))
            if cov < cov_min:
                return False, "low_loop_coverage", diag

        return True, "ok", diag

    def on_drone_discovery_response(self, payload: Dict[str, object], sim_time: Optional[float] = None) -> None:
        """Cache recent AerialScoutSystem providers for F2D request gating.

        This does not participate in normal F2/F2P peer selection. It only feeds
        the optional Drone-DT request path when explicitly enabled.
        """
        try:
            now_sim = float(sim_time if sim_time is not None else self._now())
        except Exception:
            now_sim = self._now()
        capability = str(getattr(self.cfg, "f2_drone_context_capability", "downstream_context_provider") or "")
        results = list(dict(payload or {}).get("results", []) or [])
        accepted = 0
        for res in results:
            if not isinstance(res, dict):
                continue
            role = str(res.get("role", res.get("dt_type", res.get("node_type", ""))) or "")
            provider_id = str(
                res.get("node_id", res.get("dt_id", res.get("provider_id", res.get("id", ""))))
                or ""
            ).strip()
            if not provider_id:
                continue
            caps_raw = res.get(
                "capabilities",
                res.get("capability_names", res.get("capability_name", res.get("capability", []))),
            )
            if isinstance(caps_raw, str):
                caps = {caps_raw}
            else:
                caps = {str(x) for x in list(caps_raw or []) if str(x)}
            services_raw = res.get("service_names", res.get("services", res.get("service_name", [])))
            if isinstance(services_raw, str):
                services = {services_raw}
            else:
                services = {str(x.get("name", x)) if isinstance(x, dict) else str(x) for x in list(services_raw or []) if x}
            role_ok = (not role) or role == "AerialScoutSystem"
            cap_ok = (not capability) or capability in caps or capability in services
            service_ok = bool(
                {"drone_downstream_context_up", "drone_downstream_inspection_down", "drone_downstream_inspection_down_scenario_ns"}
                & services
            )
            if not (role_ok and (cap_ok or service_ok)):
                continue
            self._drone_provider_discovery_cache[provider_id] = {
                "provider_id": provider_id,
                "role": role,
                "capabilities": sorted(caps),
                "services": sorted(services),
                "gateway_id": str(res.get("gateway_id", "") or ""),
                "status": str(res.get("status", res.get("member_status", "")) or ""),
                "sim_time": float(now_sim),
                "wall_ts": float(time.time()),
                "request_id": str(dict(payload or {}).get("request_id", "") or ""),
                "discovery_latency_ms": float(dict(payload or {}).get("latency_ms", -1.0) or -1.0),
            }
            accepted += 1
        self._fed_evt(
            "f2.drone_context.discovery_response_cached",
            requester_tls=str(self.cfg.tls_id),
            accepted_providers_n=int(accepted),
            providers=list(self._drone_provider_discovery_cache.keys()),
            request_id=str(dict(payload or {}).get("request_id", "") or ""),
            discovery_latency_ms=float(dict(payload or {}).get("latency_ms", -1.0) or -1.0),
            sim_time=float(now_sim),
            decision_source="intersection_agent",
        )

    def _select_discovered_drone_provider(
        self,
        *,
        configured_provider_id: str,
        capability: str,
        sim_time: float,
    ) -> Tuple[str, Dict[str, object]]:
        ttl = max(0.1, float(getattr(self.cfg, "f2_drone_context_discovery_cache_ttl_sec", 5.0) or 5.0))
        fresh: Dict[str, Dict[str, object]] = {}
        for pid, rec in list(self._drone_provider_discovery_cache.items()):
            age = float(sim_time) - float(rec.get("sim_time", -1e9) or -1e9)
            if age <= ttl:
                fresh[str(pid)] = dict(rec)
        selected = ""
        if configured_provider_id and configured_provider_id in fresh:
            selected = str(configured_provider_id)
        elif fresh:
            selected = sorted(fresh.keys())[0]
        rec = dict(fresh.get(selected, {}) or {})
        return selected, {
            "selected_provider_id": selected,
            "configured_provider_id": str(configured_provider_id),
            "fresh_providers": sorted(fresh.keys()),
            "fresh_providers_n": int(len(fresh)),
            "cache_ttl_sec": float(ttl),
            "capability": str(capability),
            "discovery_latency_ms": float(rec.get("discovery_latency_ms", -1.0) or -1.0),
        }

    def _drone_context_target_edges(self, ev_id: str, max_edges: Optional[int] = None) -> List[str]:
        """Return the current/future EV route edges that a mobile context provider should inspect."""
        n = int(max_edges if max_edges is not None else getattr(self.cfg, "f2_drone_context_request_max_edges", 8))
        n = max(1, min(64, n))
        route_edges = [str(e) for e in list(self._route_edges_hint(str(ev_id)) or []) if str(e)]
        if not route_edges:
            return []

        ridx = -1
        if traci is not None:
            try:
                ridx = int(traci.vehicle.getRouteIndex(str(ev_id)))
            except Exception:
                ridx = -1
        if ridx < 0:
            cur_edge = ""
            if self.active_ev is not None and str(getattr(self.active_ev, "ev_id", "")) == str(ev_id):
                cur_edge = str(getattr(self.active_ev, "in_edge_id", "") or "")
            if cur_edge:
                try:
                    ridx = route_edges.index(cur_edge)
                except ValueError:
                    ridx = -1

        start = max(0, ridx) if ridx >= 0 else 0
        out: List[str] = []
        for edge_id in route_edges[start:]:
            e = str(edge_id)
            if not e or e.startswith(":"):
                continue
            if e not in out:
                out.append(e)
            if len(out) >= n:
                break
        return out

    def _drone_context_trigger_allowed(self, reason: str) -> bool:
        r = str(reason or "")
        if not bool(getattr(self.cfg, "f2_drone_context_request_enable", False)):
            return False
        if r == "prescout_first_active_tls":
            return bool(getattr(self.cfg, "f2d_drone_prescout_enable", False))
        if r in {"no_recent_feedback", "cooldown_local_active", "usefulness_hold_active"}:
            return bool(getattr(self.cfg, "f2_drone_context_request_on_no_fresh_peer", True))
        if r in {"no_candidates", "no_nonself_candidate"}:
            return bool(getattr(self.cfg, "f2_drone_context_request_on_no_candidate", True))
        if r in {"stale_feedback_gate", "stale_feedback_gate_near", "stale_feedback_gate_preferred_mismatch"}:
            return bool(getattr(self.cfg, "f2_drone_context_request_on_stale_feedback", True))
        if r == "low_loop_coverage":
            return bool(getattr(self.cfg, "f2_drone_context_request_on_low_loop_coverage", True))
        return False

    def _maybe_request_drone_downstream_context(
        self,
        sim_time: float,
        ev_id: str,
        reason: str,
        *,
        refine_diag: Optional[Dict[str, object]] = None,
        selected_action: str = "request_downstream_context",
        target_max_edges: Optional[int] = None,
    ) -> bool:
        """
        Intersection-owned F2 requester for missing downstream context.

        The agent only emits federation messages. The real-world runner drains the
        outbox as transport plumbing; it does not decide or synthesize this request.
        """
        if not self._drone_context_trigger_allowed(str(reason)):
            return False

        provider_id = str(getattr(self.cfg, "f2_drone_context_provider_id", "crazyflie_01") or "").strip()
        if not provider_id:
            self._fed_evt(
                "f2.drone_context.request_skip",
                ev_id=str(ev_id),
                reason=str(reason),
                skip_reason="empty_provider_id",
                decision_source="intersection_agent",
            )
            return False

        target_edges = self._drone_context_target_edges(
            ev_id=str(ev_id),
            max_edges=(
                int(target_max_edges)
                if target_max_edges is not None
                else int(getattr(self.cfg, "f2_drone_context_request_max_edges", 8))
            ),
        )
        if not target_edges:
            self._fed_evt(
                "f2.drone_context.request_skip",
                ev_id=str(ev_id),
                requester_tls=str(self.cfg.tls_id),
                provider_id=str(provider_id),
                reason=str(reason),
                skip_reason="no_target_edges",
                decision_source="intersection_agent",
            )
            return False

        min_gap = max(0.0, float(getattr(self.cfg, "f2_drone_context_request_min_interval_sec", 3.0)))
        rate_key = (str(ev_id), str(provider_id), str(reason))
        last_t = float(self._drone_context_req_recent.get(rate_key, -1e9))
        if float(sim_time) - last_t < min_gap:
            self._fed_evt(
                "f2.drone_context.request_skip",
                ev_id=str(ev_id),
                requester_tls=str(self.cfg.tls_id),
                provider_id=str(provider_id),
                reason=str(reason),
                skip_reason="rate_limited",
                min_interval_sec=float(min_gap),
                last_request_sim_time=float(last_t),
                decision_source="intersection_agent",
            )
            return False

        self._drone_context_req_seq += 1
        req_id = (
            f"dronectx:{self.cfg.tls_id}:{ev_id}:"
            f"{int(float(sim_time) * 10)}:{self._drone_context_req_seq}"
        )
        ttl_sec = max(0.1, float(getattr(self.cfg, "f2_drone_context_request_ttl_sec", 3.0)))
        capability = str(getattr(self.cfg, "f2_drone_context_capability", "downstream_context_provider") or "")
        diag = dict(refine_diag or {})
        topic_ns = str(getattr(self, "_fed_topic_namespace", "") or "").strip().strip("/")
        reply_logical_topic = f"federation/v1/context/downstream/{provider_id}"
        reply_wire_topic = f"{topic_ns}/{reply_logical_topic}" if topic_ns else reply_logical_topic
        route_edges_all = [str(e) for e in list(self._route_edges_hint(str(ev_id)) or []) if str(e)]
        current_edge = ""
        route_idx = -1
        if traci is not None:
            try:
                current_edge = str(traci.vehicle.getRoadID(str(ev_id)) or "")
            except Exception:
                current_edge = ""
            try:
                route_idx = int(traci.vehicle.getRouteIndex(str(ev_id)))
            except Exception:
                route_idx = -1
        if not current_edge and self.active_ev is not None and str(getattr(self.active_ev, "ev_id", "")) == str(ev_id):
            current_edge = str(getattr(self.active_ev, "in_edge_id", "") or "")
        if route_idx < 0 and current_edge and current_edge in route_edges_all:
            route_idx = int(route_edges_all.index(current_edge))
        route_context_enabled = bool(getattr(self.cfg, "f2_drone_context_include_route_context", True))
        route_context_max_edges = int(getattr(self.cfg, "f2_drone_context_route_context_max_edges", 64) or 64)
        route_context_max_edges = max(0, min(512, route_context_max_edges))
        remaining_route_edges_all: List[str] = []
        if route_edges_all:
            start_idx = max(0, int(route_idx)) if int(route_idx) >= 0 else 0
            remaining_route_edges_all = [
                str(e)
                for e in list(route_edges_all[start_idx:] or [])
                if str(e) and not str(e).startswith(":")
            ]
        route_context_edges = list(route_edges_all)
        remaining_route_edges = list(remaining_route_edges_all)
        route_context_truncated = False
        if route_context_enabled and route_context_max_edges > 0:
            if len(route_context_edges) > route_context_max_edges:
                route_context_edges = route_context_edges[:route_context_max_edges]
                route_context_truncated = True
            if len(remaining_route_edges) > route_context_max_edges:
                remaining_route_edges = remaining_route_edges[:route_context_max_edges]
                route_context_truncated = True
        elif not route_context_enabled:
            route_context_edges = []
            remaining_route_edges = []

        request_wall_ts = float(time.time())
        payload: Dict[str, object] = {
            "schema": "federation.request.downstream_inspection.v1",
            "event_type": "DownstreamInspectionRequest",
            "request_id": str(req_id),
            "ev_id": str(ev_id),
            "requester_tls": str(self.cfg.tls_id),
            "requester_id": str(self.cfg.tls_id),
            "provider_id": str(provider_id),
            "provider_type": "drone",
            "capability": str(capability),
            "context_request_model": "mobile_passive_dt",
            "mobile_passive_context": True,
            "source_equivalent": "mobile_passive_dt",
            "inspection_scope": "route_ahead",
            "target_edges": list(target_edges),
            "target_route_edges": list(target_edges),
            "route_edges": list(route_context_edges),
            "remaining_route_edges": list(remaining_route_edges),
            "route_context": {
                "requester_tls": str(self.cfg.tls_id),
                "current_tls": str(self.cfg.tls_id),
                "current_edge": str(current_edge),
                "route_index": int(route_idx),
                "route_edges_n": int(len(route_edges_all)),
                "route_edges": list(route_context_edges),
                "route_context_edges_n": int(len(route_context_edges)),
                "route_context_truncated": bool(route_context_truncated),
                "remaining_route_edges": list(remaining_route_edges),
                "remaining_route_edges_n": int(len(remaining_route_edges_all)),
                "target_edges_n": int(len(target_edges)),
                "target_route_edges": list(target_edges),
                "preferred_next_tls": str(diag.get("preferred_next_tls", "") or ""),
                "feedback_responder_tls": str(diag.get("feedback_responder_tls", "") or ""),
                "trigger_reason": str(reason),
                "decision_source": "intersection_agent",
            },
            "sim_time": float(sim_time),
            "request_wall_ts": float(request_wall_ts),
            "request_wall_ms": float(request_wall_ts * 1000.0),
            "decision_deadline_sec": float(ttl_sec),
            "ttl_sec": float(ttl_sec),
            "max_age_sec": float(ttl_sec),
            "trigger_reason": str(reason),
            "reason": str(reason),
            "selected_action": str(selected_action),
            "decision_source": "intersection_agent",
            "source_service": "intersection_agent",
            "source_dt": str(self.cfg.tls_id),
            "requester_topic_namespace": str(topic_ns),
            "topic_namespace": str(topic_ns),
            "reply_context_topic": str(reply_wire_topic),
            "reply_context_logical_topic": str(reply_logical_topic),
            "preferred_next_tls": str(diag.get("preferred_next_tls", "") or ""),
            "feedback_responder_tls": str(diag.get("feedback_responder_tls", "") or ""),
            "feedback_age_sec": float(diag.get("feedback_age_sec", -1.0) or -1.0),
            "feedback_max_age_sec": float(diag.get("feedback_max_age_sec", -1.0) or -1.0),
            "ev_distance_m": float(diag.get("ev_distance_m", -1.0) or -1.0),
        }
        payload["request_payload_size_bytes"] = int(
            len(json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8"))
        )

        discovery_gate_enabled = bool(getattr(self.cfg, "f2_drone_context_discovery_gate_enable", False))
        if bool(getattr(self.cfg, "f2_drone_context_emit_discovery_query", True)) or discovery_gate_enabled:
            reply_topic = f"federation/discovery/resp/{self.cfg.tls_id}"
            query_filters = {
                "role": "AerialScoutSystem",
                "roles": ["AerialScoutSystem"],
                "provider_type": "drone",
                "capability": [str(capability)] if capability else [],
                "capabilities": [str(capability)] if capability else [],
                "capability_names": [str(capability)] if capability else [],
                "service_name": [
                    "drone_downstream_context_up",
                    "drone_downstream_inspection_down",
                    "drone_downstream_inspection_down_scenario_ns",
                ],
                "service_names": [
                    "drone_downstream_context_up",
                    "drone_downstream_inspection_down",
                    "drone_downstream_inspection_down_scenario_ns",
                ],
                "event_type": ["DownstreamContext", "DownstreamInspectionRequest"],
                "directions": ["local_to_fed", "fed_to_local", "bidirectional"],
                "status": ["ACTIVE", "REGISTERED", "ALIVE"],
                "node_dedup": True,
                "result_mode": "service",
            }
            discover_payload = {
                "schema": "federation.discovery.query.v1",
                "event": "query",
                "event_type": "CapabilityDiscoveryQuery",
                "request_id": f"{req_id}:discover",
                "ev_id": str(ev_id),
                "requester_id": str(self.cfg.tls_id),
                "requester_tls": str(self.cfg.tls_id),
                "capability": str(capability),
                "required_capabilities": [str(capability)] if capability else [],
                "provider_type": "drone",
                "provider_id": str(provider_id),
                "target_edges": list(target_edges),
                "reply_topic": reply_topic,
                "reply_to": reply_topic,
                "query": dict(query_filters),
                "filters": dict(query_filters),
                "max_results": 5,
                "result_mode": "service",
                "node_dedup": True,
                "purpose": "drone_downstream_context_discovery",
                "ttl_sec": float(ttl_sec),
                "sim_time": float(sim_time),
                "trigger_reason": str(reason),
                "decision_source": "intersection_agent",
            }
            if discovery_gate_enabled:
                selected_provider, discovery_diag = self._select_discovered_drone_provider(
                    configured_provider_id=str(provider_id),
                    capability=str(capability),
                    sim_time=float(sim_time),
                )
                if not selected_provider:
                    q_key = (str(ev_id), str(capability), str(reason))
                    q_gap = max(
                        0.0,
                        float(getattr(self.cfg, "f2_drone_context_discovery_query_min_interval_sec", 1.0) or 1.0),
                    )
                    q_last = float(self._drone_discovery_query_recent.get(q_key, -1e9))
                    query_queued = False
                    if float(sim_time) - q_last >= q_gap:
                        self._queue_fed_msg("federation/discovery/query", discover_payload)
                        self._drone_discovery_query_recent[q_key] = float(sim_time)
                        query_queued = True
                    self._fed_evt(
                        "f2.drone_context.request_deferred",
                        request_id=str(req_id),
                        ev_id=str(ev_id),
                        requester_tls=str(self.cfg.tls_id),
                        provider_id=str(provider_id),
                        target_edges=list(target_edges),
                        reason=str(reason),
                        selected_action="await_drone_discovery",
                        decision_source="intersection_agent",
                        discovery_query_queued=bool(query_queued),
                        discovery_query_min_interval_sec=float(q_gap),
                        **dict(discovery_diag or {}),
                    )
                    return False

                if str(selected_provider) != str(provider_id):
                    old_provider_id = str(provider_id)
                    provider_id = str(selected_provider)
                    rate_key = (str(ev_id), str(provider_id), str(reason))
                    last_t_selected = float(self._drone_context_req_recent.get(rate_key, -1e9))
                    if float(sim_time) - last_t_selected < min_gap:
                        self._fed_evt(
                            "f2.drone_context.request_skip",
                            ev_id=str(ev_id),
                            requester_tls=str(self.cfg.tls_id),
                            provider_id=str(provider_id),
                            configured_provider_id=str(old_provider_id),
                            reason=str(reason),
                            skip_reason="rate_limited_discovered_provider",
                            min_interval_sec=float(min_gap),
                            last_request_sim_time=float(last_t_selected),
                            decision_source="intersection_agent",
                            **dict(discovery_diag or {}),
                        )
                        return False
                    reply_logical_topic = f"federation/v1/context/downstream/{provider_id}"
                    reply_wire_topic = f"{topic_ns}/{reply_logical_topic}" if topic_ns else reply_logical_topic
                    payload["provider_id"] = str(provider_id)
                    payload["reply_context_topic"] = str(reply_wire_topic)
                    payload["reply_context_logical_topic"] = str(reply_logical_topic)
                    payload["discovered_provider_id"] = str(provider_id)
                    payload["configured_provider_id"] = str(old_provider_id)
                    payload["discovery_gate_used"] = True
                    payload["discovery_gate"] = dict(discovery_diag or {})
                else:
                    payload["discovery_gate_used"] = True
                    payload["discovery_gate"] = dict(discovery_diag or {})
                self._fed_evt(
                    "f2.drone_context.discovery_selected",
                    request_id=str(req_id),
                    ev_id=str(ev_id),
                    requester_tls=str(self.cfg.tls_id),
                    provider_id=str(provider_id),
                    reason=str(reason),
                    selected_action="select_drone_provider",
                    decision_source="intersection_agent",
                    **dict(discovery_diag or {}),
                )
            self._queue_fed_msg("federation/discovery/query", discover_payload)

        payload["request_payload_size_bytes"] = int(
            len(json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8"))
        )
        self._queue_fed_msg(f"federation/v1/request/downstream_inspection/{provider_id}", payload)
        self._drone_context_req_recent[rate_key] = float(sim_time)
        self._fed_evt(
            "f2.drone_context.requested",
            request_id=str(req_id),
            ev_id=str(ev_id),
            requester_tls=str(self.cfg.tls_id),
            provider_id=str(provider_id),
            target_edges=list(target_edges),
            decision_deadline_sec=float(ttl_sec),
            reason=str(reason),
            selected_action=str(selected_action),
            decision_source="intersection_agent",
            preferred_next_tls=str(diag.get("preferred_next_tls", "") or ""),
            feedback_responder_tls=str(diag.get("feedback_responder_tls", "") or ""),
            feedback_age_sec=float(diag.get("feedback_age_sec", -1.0) or -1.0),
            ev_distance_m=float(diag.get("ev_distance_m", -1.0) or -1.0),
            request_wall_ts=float(request_wall_ts),
            request_wall_ms=float(request_wall_ts * 1000.0),
            route_edges_n=int(len(route_edges_all)),
            remaining_route_edges_n=int(len(remaining_route_edges_all)),
            route_context_edges_n=int(len(route_context_edges)),
            route_context_truncated=bool(route_context_truncated),
            payload_size_bytes=int(payload.get("request_payload_size_bytes", 0) or 0),
        )
        return True

    def _maybe_prescout_drone_downstream_context(self, msg: EvRequest) -> bool:
        """F2D proactive scout: first active TLS requests mobile downstream context early."""
        if not bool(getattr(self.cfg, "f2d_drone_prescout_enable", False)):
            return False
        if not bool(getattr(self.cfg, "f2_drone_context_request_enable", False)):
            return False

        ev_id = str(getattr(msg, "ev_id", "") or "")
        if not ev_id:
            return False

        sim_time = float(getattr(msg, "sim_time", self._now()) or self._now())
        key = (str(ev_id), str(self.cfg.tls_id))
        last_t = float(self._drone_prescout_sent_by_ev_tls.get(key, -1e9))
        min_gap = max(0.0, float(getattr(self.cfg, "f2d_drone_prescout_min_interval_sec", 30.0)))
        if sim_time - last_t < min_gap:
            self._fed_evt(
                "f2d.drone_prescout.skip",
                ev_id=str(ev_id),
                requester_tls=str(self.cfg.tls_id),
                reason="rate_limited",
                last_request_sim_time=float(last_t),
                min_interval_sec=float(min_gap),
                sim_time=float(sim_time),
                decision_source="intersection_agent",
            )
            return False

        route_tls = [str(x) for x in list(getattr(msg, "route_intersections", []) or []) if str(x)]
        if not route_tls:
            route_tls = [str(x) for x in list(self._last_route_intersections_by_ev.get(ev_id, []) or []) if str(x)]
        first_tls = str(route_tls[0]) if route_tls else ""
        if bool(getattr(self.cfg, "f2d_drone_prescout_first_tls_only", True)):
            if not first_tls:
                self._fed_evt(
                    "f2d.drone_prescout.skip",
                    ev_id=str(ev_id),
                    requester_tls=str(self.cfg.tls_id),
                    reason="missing_route_intersections",
                    sim_time=float(sim_time),
                    decision_source="intersection_agent",
                )
                return False
            if first_tls != str(self.cfg.tls_id):
                self._fed_evt(
                    "f2d.drone_prescout.skip",
                    ev_id=str(ev_id),
                    requester_tls=str(self.cfg.tls_id),
                    first_route_tls=str(first_tls),
                    reason="not_first_active_tls",
                    sim_time=float(sim_time),
                    decision_source="intersection_agent",
                )
                return False

        route_edges = [str(e) for e in list(self._route_edges_hint(ev_id) or []) if str(e)]
        max_edges = max(1, min(64, int(getattr(self.cfg, "f2d_drone_prescout_max_edges", 16) or 16)))
        diag = {
            "trigger_policy": "f2d_drone_prescout",
            "preferred_next_tls": str(route_tls[1]) if len(route_tls) > 1 else "",
            "first_route_tls": str(first_tls),
            "route_tls_n": int(len(route_tls)),
            "route_edges_n": int(len(route_edges)),
            "ev_distance_m": float(getattr(msg, "distance_to_intersection_m", -1.0) or -1.0),
        }
        ok = self._maybe_request_drone_downstream_context(
            sim_time=float(sim_time),
            ev_id=str(ev_id),
            reason="prescout_first_active_tls",
            refine_diag=diag,
            selected_action="prescout_route_ahead",
            target_max_edges=int(max_edges),
        )
        if ok:
            self._drone_prescout_sent_by_ev_tls[key] = float(sim_time)
            self._fed_evt(
                "f2d.drone_prescout.requested",
                ev_id=str(ev_id),
                requester_tls=str(self.cfg.tls_id),
                first_route_tls=str(first_tls),
                route_tls=list(route_tls),
                route_tls_n=int(len(route_tls)),
                route_edges_n=int(len(route_edges)),
                prescout_max_edges=int(max_edges),
                sim_time=float(sim_time),
                decision_source="intersection_agent",
            )
        return bool(ok)

    def select_f2_offer(
        self,
        sim_time: float,
        ev_id: str,
        offers: List[SignalWindowOffer],
        external_offer: Optional[SignalWindowOffer] = None,
    ) -> Tuple[Optional[SignalWindowOffer], Dict[str, object]]:
        """
        F2 decision core owned by intersection agent:
        local offer selection -> optional federation refinement -> final measured selection.
        """
        meta: Dict[str, object] = {
            "policy": str(getattr(self.cfg, "f2_selection_policy", "measured") or "measured"),
            "refine_allowed": 0,
            "refine_reason": "",
            "final_reason": "",
        }
        if not offers:
            meta["final_reason"] = "no_offers"
            return None, meta

        # Local baseline (B1-equivalent) and optional external candidate (shadow/ERS).
        local_best = self.pick_best_offer(offers)
        local_anchor = self.pick_offer_for_current_plan(offers)
        local_anchor = self._resolve_f2_local_anchor(
            offers=offers,
            local_anchor=local_anchor,
            local_best=local_best,
            ev_id=str(ev_id),
            sim_time=float(sim_time),
        )
        chosen = external_offer if external_offer is not None else local_best
        meta["local_best_offer_id"] = str(getattr(local_best, "offer_id", "") if local_best is not None else "")
        meta["local_best_plan_type"] = str(self._plan_type_from_offer(local_best) if local_best is not None else "")
        meta["local_anchor_offer_id"] = str(getattr(local_anchor, "offer_id", "") if local_anchor is not None else "")
        meta["local_anchor_plan_type"] = str(self._plan_type_from_offer(local_anchor) if local_anchor is not None else "")
        meta["current_plan_type"] = str(getattr(self.current_plan, "plan_type", "") or "")

        chosen = self._select_offer_with_policy(
            candidate_offer=chosen,
            local_anchor_offer=local_anchor,
            stage="pre_refine",
        )

        allow_refine, refine_reason, refine_diag = self._should_refine_with_federation(
            sim_time=float(sim_time),
            ev_id=str(ev_id),
        )
        meta["refine_allowed"] = int(1 if allow_refine else 0)
        meta["refine_reason"] = str(refine_reason)
        meta["refine_diag"] = dict(refine_diag)

        try:
            ev_dist = float(refine_diag.get("ev_distance_m", -1.0))
        except Exception:
            ev_dist = -1.0
        near_base = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
        near_cooldown = float(getattr(self.cfg, "f2_refine_local_cooldown_near_distance_m", -1.0))
        near_cooldown = near_base if near_cooldown <= 0.0 else max(0.0, near_cooldown)
        cooldown_enable = bool(getattr(self.cfg, "f2_refine_local_cooldown_enable", True))
        cooldown_trigger = max(1, int(getattr(self.cfg, "f2_refine_local_cooldown_trigger_count", 3)))
        cooldown_window = max(0.5, float(getattr(self.cfg, "f2_refine_local_cooldown_window_sec", 2.5)))

        # Keep cooldown cache clean and reset streak when EV is not near stopline anymore.
        cd_until_prev = float(self._f2_local_cooldown_until_by_ev.get(str(ev_id), -1e9))
        if cd_until_prev > 0.0 and float(sim_time) >= cd_until_prev:
            self._f2_local_cooldown_until_by_ev.pop(str(ev_id), None)
        if not (ev_dist >= 0.0 and ev_dist <= near_cooldown):
            self._f2_near_gate_streak_by_ev.pop(str(ev_id), None)

        def _enter_local_cooldown(reason: str) -> None:
            if not cooldown_enable:
                return
            if not (ev_dist >= 0.0 and ev_dist <= near_cooldown):
                return
            streak = int(self._f2_near_gate_streak_by_ev.get(str(ev_id), 0)) + 1
            self._f2_near_gate_streak_by_ev[str(ev_id)] = streak
            if streak >= cooldown_trigger:
                until = float(sim_time) + float(cooldown_window)
                self._f2_local_cooldown_until_by_ev[str(ev_id)] = float(until)
                self._f2_near_gate_streak_by_ev[str(ev_id)] = 0
                self._fed_evt(
                    "coord.refine.cooldown_local.enter",
                    ev_id=str(ev_id),
                    reason=str(reason),
                    streak_count=int(streak),
                    cooldown_window_sec=float(cooldown_window),
                    cooldown_until_sim_time=float(until),
                    ev_distance_m=float(ev_dist),
                    near_distance_m=float(near_cooldown),
                )

        if cooldown_enable and ev_dist >= 0.0 and ev_dist <= near_cooldown:
            cd_until = float(self._f2_local_cooldown_until_by_ev.get(str(ev_id), -1e9))
            if float(sim_time) < cd_until:
                allow_refine = False
                refine_reason = "cooldown_local_active"
                refine_diag["cooldown_active"] = 1.0
                refine_diag["cooldown_remaining_sec"] = float(max(0.0, cd_until - float(sim_time)))
                meta["refine_allowed"] = 0
                meta["refine_reason"] = str(refine_reason)
                meta["refine_diag"] = dict(refine_diag)
                self._fed_evt(
                    "coord.refine.cooldown_local.active",
                    ev_id=str(ev_id),
                    reason=str(refine_reason),
                    cooldown_remaining_sec=float(max(0.0, cd_until - float(sim_time))),
                    cooldown_until_sim_time=float(cd_until),
                    ev_distance_m=float(ev_dist),
                )

        if bool(getattr(self.cfg, "f2_usefulness_gate_enable", True)):
            hold_until = float(self._f2_usefulness_hold_until_by_ev.get(str(ev_id), -1e9))
            if float(sim_time) >= hold_until and hold_until > 0.0:
                self._f2_usefulness_hold_until_by_ev.pop(str(ev_id), None)
                hold_until = -1e9
            if float(sim_time) < hold_until:
                allow_refine = False
                refine_reason = "usefulness_hold_active"
                refine_diag["usefulness_hold_active"] = 1.0
                refine_diag["usefulness_hold_remaining_sec"] = float(max(0.0, hold_until - float(sim_time)))
                if bool(getattr(self.cfg, "f2_usefulness_gate_failsoft_local", True)) and local_anchor is not None:
                    chosen = local_anchor
                    meta["final_reason"] = "fallback_local_usefulness_hold_active"
            else:
                max_skip_streak = 0
                for (k_ev, _k_peer), rec in list(self._hard_req_skip_tracker.items()):
                    if str(k_ev) != str(ev_id):
                        continue
                    max_skip_streak = max(max_skip_streak, int((rec or {}).get("streak", 0) or 0))
                trig = max(1, int(getattr(self.cfg, "f2_usefulness_gate_skip_streak_trigger", 6)))
                near_only = bool(getattr(self.cfg, "f2_usefulness_gate_near_only", True))
                near_dist = max(0.0, float(getattr(self.cfg, "f2_usefulness_gate_near_distance_m", 150.0)))
                near_ok = (ev_dist >= 0.0 and ev_dist <= near_dist)
                fb = self._reservation_feedback(str(ev_id), max_age_sec=max(2.0, self.fed_soft_ttl_sec))
                hard_acc = int(fb.get("hard_accepted", 0) or 0)
                hard_rej = int(fb.get("hard_rejected", 0) or 0)
                need_no_accept = bool(getattr(self.cfg, "f2_usefulness_gate_require_no_hard_accept", True))
                no_accept_ok = (hard_acc <= 0) if need_no_accept else (hard_rej >= hard_acc)
                if max_skip_streak >= trig and (near_ok or (not near_only)) and no_accept_ok:
                    hold_sec = max(0.5, float(getattr(self.cfg, "f2_usefulness_gate_hold_sec", 3.0)))
                    hold_until = float(sim_time) + hold_sec
                    self._f2_usefulness_hold_until_by_ev[str(ev_id)] = hold_until
                    allow_refine = False
                    refine_reason = "usefulness_hold_entered"
                    refine_diag["usefulness_hold_active"] = 1.0
                    refine_diag["usefulness_hold_remaining_sec"] = float(hold_sec)
                    refine_diag["usefulness_skip_streak"] = float(max_skip_streak)
                    refine_diag["usefulness_hard_accepted"] = float(hard_acc)
                    refine_diag["usefulness_hard_rejected"] = float(hard_rej)
                    self._fed_evt(
                        "coord.refine.usefulness_hold.enter",
                        ev_id=str(ev_id),
                        skip_streak=int(max_skip_streak),
                        trigger=int(trig),
                        hard_accepted=int(hard_acc),
                        hard_rejected=int(hard_rej),
                        hold_sec=float(hold_sec),
                        hold_until_sim_time=float(hold_until),
                        ev_distance_m=float(ev_dist),
                        near_distance_m=float(near_dist),
                    )
                    if bool(getattr(self.cfg, "f2_usefulness_gate_failsoft_local", True)) and local_anchor is not None:
                        chosen = local_anchor
                        meta["final_reason"] = "fallback_local_usefulness_hold_entered"

            meta["refine_allowed"] = int(1 if allow_refine else 0)
            meta["refine_reason"] = str(refine_reason)
            meta["refine_diag"] = dict(refine_diag)

        if allow_refine and chosen is not None:
            if str(refine_reason).startswith("bootstrap_"):
                self._fed_evt(
                    "coord.refine.bootstrap_allow",
                    ev_id=str(ev_id),
                    reason=str(refine_reason),
                    ev_distance_m=float(refine_diag.get("ev_distance_m", -1.0)),
                    feedback_seen_any=float(refine_diag.get("feedback_seen_any", 0.0)),
                    bootstrap_allowed=float(refine_diag.get("bootstrap_allowed", 0.0)),
                )
            elif str(refine_reason).startswith("neighbor_state"):
                self._fed_evt(
                    "coord.refine.neighbor_state_fallback",
                    ev_id=str(ev_id),
                    reason=str(refine_reason),
                    ev_distance_m=float(refine_diag.get("ev_distance_m", -1.0)),
                    preferred_next_tls=str(refine_diag.get("preferred_next_tls", "") or ""),
                    feedback_responder_tls=str(refine_diag.get("feedback_responder_tls", "") or ""),
                    responder_phase_state_age_ms=float(refine_diag.get("responder_phase_state_age_ms", -1.0)),
                    neighbor_state_age_sec=float(refine_diag.get("neighbor_state_age_sec", -1.0)),
                    neighbor_state_phase_state_age_ms=float(refine_diag.get("neighbor_state_phase_state_age_ms", -1.0)),
                )
                self._fed_evt(
                    "coord.refine.state_assisted_refine",
                    ev_id=str(ev_id),
                    reason=str(refine_reason),
                    ev_distance_m=float(refine_diag.get("ev_distance_m", -1.0)),
                    preferred_next_tls=str(refine_diag.get("preferred_next_tls", "") or ""),
                    feedback_responder_tls=str(refine_diag.get("feedback_responder_tls", "") or ""),
                    responder_phase_state_age_ms=float(refine_diag.get("responder_phase_state_age_ms", -1.0)),
                )
            chosen = self.refine_with_federation(
                sim_time=sim_time,
                ev_id=ev_id,
                current_offer=chosen,
                offers=offers,
            )
        else:
            self._maybe_request_drone_downstream_context(
                sim_time=float(sim_time),
                ev_id=str(ev_id),
                reason=str(refine_reason),
                refine_diag=dict(refine_diag),
                selected_action="request_before_local_fallback",
            )
            self._fed_evt(
                "coord.refine.skipped",
                ev_id=str(ev_id),
                reason=str(refine_reason),
                feedback_age_sec=float(refine_diag.get("feedback_age_sec", -1.0)),
                feedback_max_age_sec=float(refine_diag.get("feedback_max_age_sec", -1.0)),
                loop_coverage_ratio=float(refine_diag.get("loop_coverage_ratio", -1.0)),
                responder_phase_state_age_ms=float(refine_diag.get("responder_phase_state_age_ms", -1.0)),
                ev_distance_m=float(refine_diag.get("ev_distance_m", -1.0)),
                feedback_is_preferred=float(refine_diag.get("feedback_is_preferred", -1.0)),
                preferred_next_tls=str(refine_diag.get("preferred_next_tls", "") or ""),
                feedback_responder_tls=str(refine_diag.get("feedback_responder_tls", "") or ""),
                feedback_seen_any=float(refine_diag.get("feedback_seen_any", 0.0)),
                bootstrap_allowed=float(refine_diag.get("bootstrap_allowed", 0.0)),
                neighbor_state_used=float(refine_diag.get("neighbor_state_used", 0.0)),
                neighbor_state_age_sec=float(refine_diag.get("neighbor_state_age_sec", -1.0)),
                neighbor_state_phase_state_age_ms=float(refine_diag.get("neighbor_state_phase_state_age_ms", -1.0)),
            )
            try:
                ev_dist = float(refine_diag.get("ev_distance_m", -1.0))
            except Exception:
                ev_dist = -1.0
            near_dist = max(0.0, float(getattr(self.cfg, "f2_refine_near_distance_m", 40.0)))
            if local_anchor is not None and ev_dist >= 0.0 and ev_dist <= near_dist:
                if str(refine_reason) == "no_recent_feedback":
                    chosen = local_anchor
                    self._fed_evt(
                        "coord.refine.fallback_local.no_recent_feedback_near",
                        ev_id=str(ev_id),
                        reason=str(refine_reason),
                        ev_distance_m=float(ev_dist),
                        selected_offer_id=str(getattr(local_anchor, "offer_id", "") or ""),
                        feedback_seen_any=float(refine_diag.get("feedback_seen_any", 0.0)),
                        feedback_max_age_sec=float(refine_diag.get("feedback_max_age_sec", -1.0)),
                    )
                    meta["final_reason"] = "fallback_local_no_recent_feedback_near"
                    _enter_local_cooldown("no_recent_feedback")
                elif str(refine_reason) in {"stale_feedback_gate_near", "stale_feedback_gate_preferred_mismatch"}:
                    chosen = local_anchor
                    self._fed_evt(
                        "coord.refine.fallback_local.stale_feedback_near",
                        ev_id=str(ev_id),
                        reason=str(refine_reason),
                        ev_distance_m=float(ev_dist),
                        selected_offer_id=str(getattr(local_anchor, "offer_id", "") or ""),
                        responder_phase_state_age_ms=float(refine_diag.get("responder_phase_state_age_ms", -1.0)),
                        feedback_responder_tls=str(refine_diag.get("feedback_responder_tls", "") or ""),
                        preferred_next_tls=str(refine_diag.get("preferred_next_tls", "") or ""),
                    )
                    meta["final_reason"] = "fallback_local_stale_feedback_near"
                    _enter_local_cooldown(str(refine_reason))
                elif str(refine_reason) == "cooldown_local_active":
                    chosen = local_anchor
                    self._fed_evt(
                        "coord.refine.fallback_local.cooldown_active",
                        ev_id=str(ev_id),
                        reason=str(refine_reason),
                        ev_distance_m=float(ev_dist),
                        selected_offer_id=str(getattr(local_anchor, "offer_id", "") or ""),
                        cooldown_remaining_sec=float(refine_diag.get("cooldown_remaining_sec", -1.0)),
                    )
                    meta["final_reason"] = "fallback_local_cooldown_active"

        chosen = self._select_offer_with_policy(
            candidate_offer=chosen,
            local_anchor_offer=local_anchor,
            stage="post_refine",
        )

        # Final safety gate: do not actuate infeasible F2 offer.
        if bool(getattr(self.cfg, "f2_block_infeasible_actuation", True)):
            if chosen is not None and (not bool(getattr(chosen, "feasible", False))):
                if local_anchor is not None and bool(getattr(local_anchor, "feasible", False)):
                    self._fed_dbg(
                        f"f2_infeasible_block ev={ev_id} action=fallback_local "
                        f"chosen={getattr(chosen, 'offer_id', '')} local={getattr(local_anchor, 'offer_id', '')}"
                    )
                    chosen = local_anchor
                    meta["final_reason"] = "fallback_local_feasible"
                else:
                    self._fed_dbg(
                        f"f2_infeasible_block ev={ev_id} action=block_no_local "
                        f"chosen={getattr(chosen, 'offer_id', '')}"
                    )
                    chosen = None
                    meta["final_reason"] = "blocked_infeasible_no_local_feasible"
                    _enter_local_cooldown("blocked_infeasible_no_local_feasible")
            elif not str(meta.get("final_reason", "")).strip():
                meta["final_reason"] = "selected"
        else:
            if not str(meta.get("final_reason", "")).strip():
                meta["final_reason"] = "selected_no_infeasible_block"

        final_reason_for_source = str(meta.get("final_reason", "") or "")
        chosen_offer_id = str(getattr(chosen, "offer_id", "") or "") if chosen is not None else ""
        local_anchor_offer_id = str(meta.get("local_anchor_offer_id", "") or "")
        local_best_offer_id = str(meta.get("local_best_offer_id", "") or "")
        selected_source = "peer_override"
        if final_reason_for_source.startswith("fallback_local_"):
            selected_source = "f2_local_fallback"
        elif final_reason_for_source in {"blocked_infeasible_no_local_feasible", "no_offers"}:
            selected_source = "f2_selected_none"
        elif chosen_offer_id and chosen_offer_id in {local_anchor_offer_id, local_best_offer_id}:
            selected_source = "local_anchor"
        meta["selected_source"] = str(selected_source)
        if chosen is not None:
            if chosen_offer_id:
                self._f2_selected_offer_source_by_id[chosen_offer_id] = str(selected_source)
                # Keep this small and scoped to recently selected offers.
                if len(self._f2_selected_offer_source_by_id) > 256:
                    for old_key in list(self._f2_selected_offer_source_by_id.keys())[:64]:
                        self._f2_selected_offer_source_by_id.pop(old_key, None)

        if str(meta.get("final_reason", "")) == "selected" and str(meta.get("refine_reason", "")) in {"ok", "bootstrap_no_feedback", "neighbor_state_fallback"}:
            self._f2_near_gate_streak_by_ev.pop(str(ev_id), None)

        self._fed_evt(
            "coord.refine.selection_final",
            ev_id=str(ev_id),
            selected_offer_id=str(getattr(chosen, "offer_id", "") if chosen is not None else ""),
            selected_feasible=int(bool(getattr(chosen, "feasible", False)) if chosen is not None else 0),
            selected_robust_score=float(self._offer_robust_cost(chosen)),
            selected_ev_cost=float(self._offer_ev_cost(chosen)),
            final_reason=str(meta.get("final_reason", "")),
            refine_allowed=int(meta.get("refine_allowed", 0)),
            refine_reason=str(meta.get("refine_reason", "")),
            selected_source=str(selected_source),
            local_anchor_offer_id=str(meta.get("local_anchor_offer_id", "")),
            local_anchor_plan_type=str(meta.get("local_anchor_plan_type", "")),
            local_best_offer_id=str(meta.get("local_best_offer_id", "")),
            local_best_plan_type=str(meta.get("local_best_plan_type", "")),
            current_plan_type=str(meta.get("current_plan_type", "")),
        )
        self._session_event_counts["selection_final"] += 1
        sel_reason = str(meta.get("final_reason", "") or "").strip()
        if sel_reason:
            self._session_reason_counts[f"selection_final:{sel_reason}"] += 1

        return chosen, meta

    def _stage_rank(self, stage: AgentStage) -> int:
        order = {
            AgentStage.NO_REQUEST: 0,
            AgentStage.BASELINE_VALIDATION: 1,
            AgentStage.SATURATION_REDUCTION: 2,
            AgentStage.PREEMPTION_NON_INTRUSIVE: 3,
            AgentStage.PREEMPTION: 4,
            AgentStage.RESTORATION: 5,
        }
        return int(order.get(stage, 0))

    def _desired_stage_for_plan(self, plan: PreemptionPlan) -> Optional[AgentStage]:
        if plan.plan_type == "restore":
            return AgentStage.RESTORATION
        if plan.plan_type == "intrusive":
            return AgentStage.PREEMPTION
        if plan.plan_type == "non_intrusive":
            return AgentStage.PREEMPTION_NON_INTRUSIVE
        if plan.plan_type == "saturation_reduction":
            return AgentStage.SATURATION_REDUCTION
        if plan.plan_type == "none":
            return AgentStage.BASELINE_VALIDATION
        return None
    
    def apply_offer_to_tls(self, sim_time: float, offer: SignalWindowOffer) -> None:
            if traci is None or offer is None:
                return
            self._session_event_counts["apply_offer"] += 1
            now = float(sim_time)
            offer_id = str(getattr(offer, "offer_id", "") or "")
            offer_decision_source = str(
                self._f2_selected_offer_source_by_id.pop(offer_id, "offer") if offer_id else "offer"
            )

            self._register_selected_offer_prediction(
                sim_time=float(sim_time),
                offer=offer,
            )

            self._log_decision_event(
                event="apply_offer_to_tls",
                sim_time=float(sim_time),
                decision_source=str(offer_decision_source),
                plan=None,
                offer=offer,
                note="invoked",
            )

            plan = self._offer_plan_cache.get(str(getattr(offer, "offer_id", "")))
            if plan is None:
                plan = self._fallback_plan_from_offer(offer)

            if bool(getattr(self.cfg, "f2_offer_preapply_dedupe_enable", True)):
                ev_id = str(self.active_ev.ev_id) if self.active_ev is not None else str(getattr(offer, "ev_id", "") or "")
                if ev_id:
                    plan_type_s = str(getattr(plan, "plan_type", "") or "")
                    plan_target_i = int(
                        getattr(plan, "target_phase_idx", -1)
                        if getattr(plan, "target_phase_idx", None) is not None
                        else -1
                    )
                    ev_speed_mps = -1.0
                    try:
                        ev_speed_mps = float(getattr(self.active_ev, "speed_mps", -1.0)) if self.active_ev is not None else -1.0
                    except Exception:
                        ev_speed_mps = -1.0
                    tls_phase = -1
                    tls_state = ""
                    tls_next_switch = -1.0
                    if traci is not None:
                        try:
                            tls_phase = int(traci.trafficlight.getPhase(str(self.cfg.tls_id)))
                        except Exception:
                            pass
                        try:
                            tls_state = str(traci.trafficlight.getRedYellowGreenState(str(self.cfg.tls_id)))
                        except Exception:
                            pass
                        try:
                            tls_next_switch = float(traci.trafficlight.getNextSwitch(str(self.cfg.tls_id)))
                        except Exception:
                            pass
                    stopped_weak_offer = bool(
                        ev_speed_mps >= 0.0
                        and ev_speed_mps <= 0.5
                        and plan_type_s in ("saturation_reduction", "non_intrusive")
                    )
                    if stopped_weak_offer:
                        # While the EV is stopped/crawling, dynamic offer fields
                        # such as hurry_to can change every step without changing
                        # the actual TLS state. Coarsen the signature to the
                        # observable TLS schedule so repeated no-op applications
                        # cannot churn the controller.
                        offer_sig = (
                            str(offer_decision_source),
                            "stopped_weak_offer_tls_state",
                            plan_type_s,
                            plan_target_i,
                            str(getattr(offer, "action", "") or ""),
                            int(tls_phase),
                            str(tls_state),
                            round(float(tls_next_switch), 1),
                            int(bool(getattr(offer, "feasible", False))),
                        )
                    else:
                        offer_sig = (
                            str(offer_decision_source),
                            plan_type_s,
                            plan_target_i,
                            round(float(getattr(plan, "extend_green_sec", 0.0) or 0.0), 3),
                            round(float(getattr(plan, "hurry_current_phase_to_sec", -1.0) if getattr(plan, "hurry_current_phase_to_sec", None) is not None else -1.0), 3),
                            round(float(getattr(plan, "jump_time_sec", -1.0) if getattr(plan, "jump_time_sec", None) is not None else -1.0), 3),
                            int(getattr(plan, "jump_to_phase_idx", -1) if getattr(plan, "jump_to_phase_idx", None) is not None else -1),
                            int(bool(getattr(offer, "feasible", False))),
                        )
                    prev_sig = self._last_offer_apply_signature_by_ev.get(ev_id)
                    prev_ts = float(self._last_offer_apply_sim_time_by_ev.get(ev_id, -1e9))
                    ev_dist_m = float(self._active_ev_distance_m(ev_id))
                    min_dt_base = max(
                        0.0,
                        float(getattr(self.cfg, "f2_offer_preapply_dedupe_min_interval_sec", 2.0)),
                    )
                    min_dt_near = max(
                        0.0,
                        float(
                            getattr(
                                self.cfg,
                                "f2_offer_preapply_dedupe_min_interval_near_sec",
                                min_dt_base,
                            )
                        ),
                    )
                    min_dt_far = max(
                        0.0,
                        float(
                            getattr(
                                self.cfg,
                                "f2_offer_preapply_dedupe_min_interval_far_sec",
                                min_dt_base,
                            )
                        ),
                    )
                    near_m = max(
                        0.0,
                        float(getattr(self.cfg, "f2_offer_preapply_dedupe_near_distance_m", 120.0)),
                    )
                    far_m = max(
                        near_m,
                        float(getattr(self.cfg, "f2_offer_preapply_dedupe_far_distance_m", 300.0)),
                    )
                    min_dt = self._distance_adaptive_value(
                        base=min_dt_base,
                        near_value=min_dt_near,
                        far_value=min_dt_far,
                        ev_distance_m=float(ev_dist_m),
                        near_distance_m=float(near_m),
                        far_distance_m=float(far_m),
                    )
                    coord_active = bool(self._coordination_window_active(ev_id, now_sim=now))
                    if coord_active:
                        scale = max(0.1, min(1.0, float(getattr(self.cfg, "f2_active_coord_window_interval_scale", 0.50))))
                        min_dt *= float(scale)
                    if prev_sig == offer_sig and (now - prev_ts) < min_dt:
                        self._session_event_counts["plan_skip"] += 1
                        self._session_reason_counts["plan_skip:redundant_offer_preapply"] += 1
                        self._fed_evt(
                            "coord.apply.offer_skip",
                            sim_time=now,
                            tls_id=str(self.cfg.tls_id),
                            ev_id=str(ev_id),
                            decision_source=str(offer_decision_source),
                            reason="redundant_offer_preapply",
                            signature_mode=("stopped_weak_offer_tls_state" if stopped_weak_offer else "plan_exact"),
                            ev_speed_mps=float(ev_speed_mps),
                            tls_phase=int(tls_phase),
                            tls_state=str(tls_state),
                            tls_next_switch=float(tls_next_switch),
                            ev_distance_m=float(ev_dist_m),
                            coordination_active=int(1 if coord_active else 0),
                            dt_since_last_s=float(now - prev_ts),
                            min_interval_s=float(min_dt),
                            min_interval_near_s=float(min_dt_near),
                            min_interval_far_s=float(min_dt_far),
                            min_interval_near_m=float(near_m),
                            min_interval_far_m=float(far_m),
                        )
                        return
                    self._last_offer_apply_signature_by_ev[ev_id] = offer_sig
                    self._last_offer_apply_sim_time_by_ev[ev_id] = now

            self._plan_selected_dbg(
                sim_time=float(sim_time),
                decision_source=str(f"{offer_decision_source}_selected"),
                plan=plan,
                offer=offer,
                note="before_apply_offer_to_tls",
            )

            # Keep runtime state aligned with the actually selected offer.
            # Without this, logs/state may still reflect a stale plan produced by tick().
            self.current_plan = plan

            # Keep B1/F2 consistent: do not downgrade stage due to offer decoding.
            # Tick() remains the owner of stage-machine progression; we only escalate.
            desired_stage = self._desired_stage_for_plan(plan)
            if desired_stage is not None:
                if self._stage_rank(desired_stage) > self._stage_rank(self.stage):
                    self._transition_to(desired_stage, f"offer:{plan.plan_type}:escalate")

            self.apply_plan_to_tls(sim_time, plan, decision_source=str(offer_decision_source))



# ---------- internal helpers for offers ----------

    def _make_offer(self,
                    sim_time: float,
                    ev: EvRequest,
                    target_phase_idx: int,
                    action: str,
                    action_params: Dict[str, float],
                    green_window: Tuple[float, float],
                    arrival_window: Tuple[float, float]) -> SignalWindowOffer:
        
        gw0, gw1 = float(green_window[0]), float(green_window[1])
        ar0, ar1 = float(arrival_window[0]), float(arrival_window[1])
        use_improved = bool(getattr(self.cfg, "enable_improved_offer_metrics", False))

        if use_improved:
            comp = self._compute_offer_metrics_improved(
                sim_time=float(sim_time),
                ev=ev,
                target_phase_idx=int(target_phase_idx),
                action=str(action),
                action_params=dict(action_params or {}),
                green_window=(gw0, gw1),
                arrival_window=(ar0, ar1),
            )
            eff_ar0, eff_ar1 = float(comp["arrival_eff_start"]), float(comp["arrival_eff_end"])
            q_delay = float(comp["queue_delay_sec"])
            expected_wait = float(comp["expected_wait_sec"])
            expected_miss = float(comp["expected_miss_sec"])
            miss_prob = float(comp["miss_prob"])
            cost_veh_sec = float(comp["cost_to_others_veh_sec"])
            speed_rng = (
                (float(comp["speed_min_mps"]), float(comp["speed_max_mps"]))
                if comp.get("speed_min_mps") is not None and comp.get("speed_max_mps") is not None
                else None
            )
        else:
            # Queue-aware EV arrival adjustment:
            # estimate delay to clear queued vehicles ahead of EV on its approach before crossing.
            q_delay = self._estimate_ev_queue_delay_for_offer(
                ev=ev,
                sim_time=float(sim_time),
                green_window=(gw0, gw1),
                arrival_window=(ar0, ar1),
            )
            eff_ar0, eff_ar1 = float(ar0 + q_delay), float(ar1 + q_delay)

            # expected wait / miss from uniform arrival over effective window
            expected_wait, miss_prob, expected_miss_late = self._expected_wait_miss_from_windows(
                arrival_window=(eff_ar0, eff_ar1),
                green_window=(gw0, gw1),
            )
            # Keep backward compatibility: expected_miss_sec as expected-late seconds.
            expected_miss = float(expected_miss_late)

            # Non-EV impact upgraded with lane demand projection up to EV arrival horizon.
            horizon_sec = max(0.0, float(0.5 * (eff_ar0 + eff_ar1) - float(sim_time)))
            cost_veh_sec = self._estimate_non_ev_cost_veh_sec(
                action,
                action_params,
                target_phase_idx,
                horizon_sec=horizon_sec,
                ev_edge_override=str(ev.in_edge_id),
            )
            speed_rng = self._recommended_speed_range_mps(ev, sim_time, (gw0, gw1))

        # feasibility: effective EV arrival window intersects offered green window
        feasible = not (eff_ar1 < gw0 or eff_ar0 > gw1)

        conf = self._offer_confidence(
            action=action,
            action_params=action_params,
            target_phase=target_phase_idx,
            sim_time=float(sim_time),
            arrival_window=(eff_ar0, eff_ar1),
            green_window=(gw0, gw1),
            ev=ev,
            non_ev_cost_veh_sec=float(cost_veh_sec),
            queue_delay_sec=float(q_delay),
            miss_prob=float(miss_prob),
        )

        offer_id = f"{self.cfg.tls_id}|{ev.ev_id}|{int(sim_time*10)}|{action}|{hash(tuple(sorted(action_params.items())))}"

        if use_improved:
            self._store_offer_metric_components(
                offer_id=offer_id,
                components=dict(comp),
                action=str(action),
                target_phase_idx=int(target_phase_idx),
                green_window=(gw0, gw1),
                arrival_window=(ar0, ar1),
            )

        return SignalWindowOffer(
            offer_id=offer_id,
            tls_id=str(self.cfg.tls_id),
            ev_id=str(ev.ev_id),
            created_time=float(sim_time),
            target_phase_idx=int(target_phase_idx),
            action=str(action),
            action_params=dict(action_params),
            green_window=(gw0, gw1),
            arrival_window=(ar0, ar1),
            feasible=bool(feasible),
            expected_wait_sec=float(expected_wait),
            expected_miss_sec=float(expected_miss),
            cost_to_others_veh_sec=float(cost_veh_sec),
            confidence=float(conf),
            speed_range_mps=speed_rng,
        )

    def _expected_wait_miss_from_windows(
        self,
        arrival_window: Tuple[float, float],
        green_window: Tuple[float, float],
    ) -> Tuple[float, float, float]:
        """
        Uniform-arrival approximation:
        returns (E[wait], P(miss), E[late]).
        """
        a = float(arrival_window[0])
        b = float(arrival_window[1])
        g0 = float(green_window[0])
        g1 = float(green_window[1])
        if b <= a:
            return 0.0, 0.0, 0.0

        # E[max(0, g0 - T)] for T~U[a,b]
        if b <= g0:
            e_wait = g0 - 0.5 * (a + b)
        elif a >= g0:
            e_wait = 0.0
        else:
            e_wait = ((g0 - a) * (g0 - a)) / (2.0 * (b - a))

        # P(T > g1)
        if b <= g1:
            p_miss = 0.0
        elif a >= g1:
            p_miss = 1.0
        else:
            p_miss = (b - g1) / (b - a)

        # E[max(0, T - g1)] for T~U[a,b]
        if a >= g1:
            e_late = 0.5 * (a + b) - g1
        elif b <= g1:
            e_late = 0.0
        else:
            e_late = ((b - g1) * (b - g1)) / (2.0 * (b - a))

        return float(e_wait), float(p_miss), float(e_late)

    def _estimate_ev_queue_delay_for_offer(
        self,
        ev: EvRequest,
        sim_time: float,
        green_window: Tuple[float, float],
        arrival_window: Tuple[float, float],
    ) -> float:
        """
        Estimate additional EV delay due to queue ahead on EV approach lane.
        Uses queue-clearing time Q and available pre-arrival green in the offered window.
        """
        if traci is None or ev is None:
            return 0.0
        edge_id = str(ev.in_edge_id)
        if (not edge_id) or edge_id.startswith(":"):
            return 0.0

        if bool(getattr(self.cfg, "offer_metric_use_paper_queue_clearing", True)) and bool(
            getattr(self.cfg, "queue_metrics_enable_improved", True)
        ):
            try:
                qc = self._queue_clearing_metrics_improved(
                    edge_id=edge_id,
                    ev_id=str(ev.ev_id),
                    sim_time=float(sim_time),
                    t_i=0.5 * (float(arrival_window[0]) + float(arrival_window[1])),
                    green_window=(float(green_window[0]), float(green_window[1])),
                    arrival_window=(float(arrival_window[0]), float(arrival_window[1])),
                )
                return float(max(0.0, qc.get("q_delay_sec", 0.0)))
            except Exception:
                pass

        try:
            _N, _A, _S, Q, _lanes = self._queue_clearing_metrics_for_edge(edge_id=edge_id, ev_id=str(ev.ev_id))
        except Exception:
            return 0.0

        q_req = float(Q) + float(getattr(self.cfg, "T_lost_sec", 5.0)) + float(getattr(self.cfg, "YT_sec", 5.0))
        if q_req <= 0.0:
            return 0.0

        gw0, gw1 = float(green_window[0]), float(green_window[1])
        ar0, _ar1 = float(arrival_window[0]), float(arrival_window[1])
        avail_pre = max(0.0, min(gw1, ar0) - gw0)
        q_delay = max(0.0, q_req - avail_pre)
        return float(q_delay)

    def _tls_action_impact_index(
        self,
        action: str,
        action_params: Dict[str, float],
        target_phase: int,
        non_ev_cost_veh_sec: float,
    ) -> float:
        """
        0..1 impact proxy of how disruptive the TLS modification is.
        """
        ap = dict(action_params or {})
        ext = max(0.0, float(ap.get("ext", 0.0)))
        max_ext = max(1e-6, float(getattr(self.cfg, "max_target_green_extension_sec", 40.0)))

        if action == "none":
            effort = 0.0
        elif action == "extend":
            effort = min(1.0, ext / max_ext)
        elif action == "hurry":
            hurry_to = float(ap.get("hurry_to", 2.0))
            hurry_eff = max(0.0, 3.0 - hurry_to) / 3.0
            effort = min(1.0, hurry_eff + 0.3 * (ext / max_ext))
        elif action == "jump":
            effort = 1.0
        else:
            effort = 0.5

        non_ev_norm = min(
            1.0,
            max(0.0, float(non_ev_cost_veh_sec))
            / max(1e-6, float(getattr(self.cfg, "offer_conf_non_ev_scale_veh_sec", 300.0))),
        )

        # Additional shock when forcing immediate phase change
        phase_shock = 1.0 if action in ("hurry", "jump") else 0.2
        impact = 0.45 * effort + 0.40 * non_ev_norm + 0.15 * phase_shock
        return float(max(0.0, min(1.0, impact)))

    def _offer_confidence(
        self,
        action: str,
        action_params: Optional[Dict[str, float]] = None,
        target_phase: Optional[int] = None,
        sim_time: Optional[float] = None,
        arrival_window: Optional[Tuple[float, float]] = None,
        green_window: Optional[Tuple[float, float]] = None,
        ev: Optional[EvRequest] = None,
        non_ev_cost_veh_sec: Optional[float] = None,
        queue_delay_sec: float = 0.0,
        miss_prob: float = 0.0,
    ) -> float:
        """
        Confidence of offer realization (0..1), combining:
        - action class reliability
        - TLS impact severity
        - queue uncertainty
        - window-overlap adequacy
        """
        # Base by action type (structural reliability)
        if action == "none":
            base = 0.97
        elif action == "extend":
            base = 0.90
        elif action == "hurry":
            base = 0.80
        elif action == "jump":
            base = 0.65
        else:
            base = 0.55

        ap = dict(action_params or {})
        tgt = int(target_phase) if target_phase is not None else int(self.active_ev.target_phase_idx if self.active_ev else 0)
        ne_cost = float(non_ev_cost_veh_sec or 0.0)
        impact = self._tls_action_impact_index(action, ap, tgt, ne_cost)

        queue_norm = min(
            1.0,
            max(0.0, float(queue_delay_sec))
            / max(1e-6, float(getattr(self.cfg, "offer_conf_queue_scale_sec", 30.0))),
        )

        if (arrival_window is not None) and (green_window is not None):
            a0, a1 = float(arrival_window[0]), float(arrival_window[1])
            g0, g1 = float(green_window[0]), float(green_window[1])
            overlap = max(0.0, min(a1, g1) - max(a0, g0))
            cov = overlap / max(1e-6, (a1 - a0))
            window_risk = max(0.0, 1.0 - cov)
        else:
            window_risk = 0.2

        conf = (
            float(base)
            - float(getattr(self.cfg, "offer_conf_impact_weight", 0.35)) * impact
            - float(getattr(self.cfg, "offer_conf_non_ev_weight", 0.25))
            * min(1.0, ne_cost / max(1e-6, float(getattr(self.cfg, "offer_conf_non_ev_scale_veh_sec", 300.0))))
            - float(getattr(self.cfg, "offer_conf_queue_weight", 0.20)) * queue_norm
            - float(getattr(self.cfg, "offer_conf_window_weight", 0.20)) * max(window_risk, float(miss_prob))
        )

        return float(max(0.05, min(0.99, conf)))

    def _recommended_speed_range_mps(self, ev: EvRequest, now: float, green_window: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Speed interval that would let the EV reach the stopline during the offered green window."""
        if traci is None:
            return None
        dist = float(ev.distance_to_intersection_m)
        if dist <= 1.0:
            return None
        t0, t1 = float(green_window[0]), float(green_window[1])
        # If window is in the past, no advice
        if t1 <= now + 1e-6:
            return None

        # Speed to arrive at a particular time: v = dist / (t_arrival - now)
        # Range: [arrive at end, arrive at start] (end -> slower, start -> faster)
        dt_fast = max(0.1, t0 - now)
        dt_slow = max(0.1, t1 - now)
        v_fast = dist / dt_fast
        v_slow = dist / dt_slow

        v_min = min(v_slow, v_fast)
        v_max = max(v_slow, v_fast)

        # Clamp to lane speed limit if we can get it
        try:
            lane_id = traci.vehicle.getLaneID(ev.ev_id)
            v_lim = float(traci.lane.getMaxSpeed(lane_id))
            v_max = min(v_max, v_lim)
        except Exception:
            pass

        # If range is nonsense, skip
        if v_max < 0.5:
            return None
        v_min = max(v_min, 0.5)
        if v_min > v_max:
            return None
        return (float(v_min), float(v_max))

    def _estimate_non_ev_cost_veh_sec(self,
                                     action: str,
                                     action_params: Dict[str, float],
                                     target_phase: int,
                                     horizon_sec: float = 0.0,
                                     ev_edge_override: Optional[str] = None) -> float:
        """Very lightweight cost model: additional red time * queued vehicles on non-EV approaches.

        Returns vehicle-seconds (veh*s) as a proxy for 'impact on others'.
        """
        if traci is None:
            return 0.0

        tls_id = self.cfg.tls_id
        ext = 0.0
        if action == "extend":
            ext = float(action_params.get("ext", 0.0))
        elif action == "hurry":
            # Hurry is less disruptive than extension but not free.
            hurry_to = float(action_params.get("hurry_to", 2.0))
            ext = max(0.0, (3.0 - hurry_to)) * 0.8
        elif action == "jump":
            # Intrusive: approximate as holding target green for SIT+YT+2*delta
            ext = float(self.cfg.SIT_sec + self.cfg.YT_sec + 2.0 * (self.active_ev.delta_sec if self.active_ev else 2.0))
        else:
            ext = 0.0

        if ext <= 0.0:
            return 0.0

        # Collect controlled lanes, estimate queues (halting vehicles)
        try:
            lanes = list(traci.trafficlight.getControlledLanes(tls_id))
        except Exception:
            lanes = []

        ev_edge = None
        if getattr(self.cfg, "offer_exclude_ev_approach_from_cost", True):
            if ev_edge_override:
                ev_edge = str(ev_edge_override)
            elif self.active_ev:
                ev_edge = str(self.active_ev.in_edge_id)

        queued = 0.0
        approaching = 0.0
        for ln in lanes:
            try:
                edge = ln.rsplit("_", 1)[0]
            except Exception:
                edge = None
            if ev_edge and edge == ev_edge:
                continue
            try:
                q = float(traci.lane.getLastStepHaltingNumber(ln))
                n = float(traci.lane.getLastStepVehicleNumber(ln))
                queued += q
                approaching += max(0.0, n - q)
            except Exception:
                pass

        # veh*s proxy with projected arrivals over horizon:
        # demand ≈ queued + 0.5*approaching + 0.3*approaching*horizon_factor
        h_fac = max(0.0, min(1.0, float(horizon_sec) / 20.0))
        demand = float(queued + 0.5 * approaching + 0.3 * approaching * h_fac)
        return float(demand * ext)

    # =========================
    # Core computations
    # =========================

    '''
    def _estimate_arrival_time(self, sim_time: float, ev: EvRequest) -> float:
        v = max(float(ev.speed_mps), 0.1)
        return float(sim_time) + (float(ev.distance_to_intersection_m) / v)
    '''
    
    def _estimate_arrival_time(self, sim_time: float, ev: EvRequest) -> float:
        dist = float(ev.distance_to_intersection_m)
        v = float(ev.speed_mps)

        if dist <= 0:
            return float(sim_time)

        if traci is None:
            v_eff = max(v, 3.0)
            return float(sim_time) + dist / v_eff

        STOPPED = 0.5
        NEAR = 30.0
        FAR = 80.0

        if v >= STOPPED:
            v_eff = v
        else:
            v_lim = 0.0
            try:
                lane_id = traci.vehicle.getLaneID(ev.ev_id)
                v_lim = float(traci.lane.getMaxSpeed(lane_id))
            except Exception:
                v_lim = 0.0

            if dist <= NEAR:
                v_eff = 2.0
            elif dist >= FAR:
                v_eff = max(3.0, 0.3 * v_lim) if v_lim > 0 else 3.0
            else:
                v_eff = 2.0

        v_eff = max(v_eff, 0.5)
        v_eff = min(v_eff, 25.0)
        return float(sim_time) + dist / v_eff

    def _compute_tul_level(self, sim_time: float, t_i: float) -> int:
        eta_min = max((t_i - sim_time) / 60.0, 0.0)
        if eta_min < 5.0:
            return 1
        if eta_min < 10.0:
            return 2
        return 3

    def _compute_clrs_level(self, edge_id: str) -> int:
        """
        CLRS using edge mean speed vs edge speed limit.
        Returns: 1 severe, 2 moderate, 3 light, 4 smooth.
        """
        if traci is None or edge_id.startswith(":"):
            return 2

        # mean speed
        try:
            avg_kmh = float(traci.edge.getLastStepMeanSpeed(edge_id)) * 3.6
        except Exception:
            avg_kmh = 0.0

        # speed limit
        try:
            sl_kmh = float(traci.edge.getSpeed(edge_id)) * 3.6
        except Exception:
            sl_kmh = 0.0
            try:
                n = int(traci.edge.getLaneNumber(edge_id))
                if n > 0:
                    sl_kmh = (sum(float(traci.lane.getMaxSpeed(f"{edge_id}_{i}")) for i in range(n)) / n) * 3.6
            except Exception:
                pass

        return self._clrs_from_speed(avg_kmh, sl_kmh)

    @staticmethod
    def _clrs_from_speed(avg_kmh: float, sl_kmh: float) -> int:
        v = max(float(avg_kmh), 0.0)
        sl = max(float(sl_kmh), 0.0)

        if sl >= 80:
            if v >= 55: return 4
            if 40 <= v < 55: return 3
            if 30 <= v < 40: return 2
            return 1
        if sl >= 70:
            if v >= 50: return 4
            if 40 <= v < 50: return 3
            if 30 <= v < 40: return 2
            return 1
        if sl >= 60:
            if v >= 45: return 4
            if 40 <= v < 45: return 3
            if 30 <= v < 40: return 2
            return 1
        if sl >= 50:
            if v >= 40: return 4
            if 35 <= v < 40: return 3
            if 25 <= v < 35: return 2
            return 1
        if sl >= 40:
            if v >= 35: return 4
            if 30 <= v < 35: return 3
            if 25 <= v < 30: return 2
            return 1

        if v >= 35: return 4
        if 30 <= v < 35: return 3
        if 20 <= v < 30: return 2
        return 1

    def _compute_drrs(self, erl: int, clrs: int, tul: int) -> float:
        return (float(erl) ** float(self.cfg.w_erl)) * (float(clrs) ** float(self.cfg.w_clrs)) * (float(tul) ** float(self.cfg.w_tul))

    def _extension_time_from_drrs(self, drrs: float) -> float:
        best_ext = 0.0
        best_dist = float("inf")
        for centroid, ext in self.cfg.drrs_clusters:
            dist = abs(float(drrs) - float(centroid))
            if dist < best_dist:
                best_dist = dist
                best_ext = float(ext)
        return float(best_ext)

    # =========================
    # Phase-window prediction helpers
    # =========================

    def _phase_timeline(self, sim_time: float, horizon_sec: float = 600.0) -> List[Tuple[int, float, float]]:
        """
        Build a timeline of (phase_idx, start, end) from *now* out to horizon_sec,
        using current remaining time for current phase + program durations for others.
        """
        if traci is None:
            return []
        tls_id = self.cfg.tls_id

        now = float(sim_time)
        cur_phase = int(traci.trafficlight.getPhase(tls_id))
        next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        rem = max(0.0, next_switch - now)

        durations = self._get_program_phase_durations()
        if not durations:
            return []

        n = len(durations)
        out: List[Tuple[int, float, float]] = []
        # current phase interval
        out.append((cur_phase, now, now + rem))

        t = now + rem
        p = (cur_phase + 1) % n
        end_time = now + float(horizon_sec)

        # walk forward
        # (limit iterations to avoid infinite loops)
        for _ in range(int(n * 10)):  # enough for multiple cycles within 600s
            if t >= end_time:
                break
            d = float(durations[p])
            out.append((p, t, t + d))
            t += d
            p = (p + 1) % n

        return out

    def _predict_next_phase_window(self, sim_time: float, phase_idx: int) -> Optional[Tuple[float, float]]:
        """Next interval [start,end] when phase_idx is active (may be current)."""
        timeline = self._phase_timeline(sim_time, horizon_sec=600.0)
        print(f"Current timeline: {timeline}")

        if not timeline:
            return None
        now = float(sim_time)
        for p, s, e in timeline:
            if p == int(phase_idx) and e > now + 1e-6:
                return (float(max(s, now)), float(e))
        return None

    def _predict_phase_window_containing(self, sim_time: float, phase_idx: int, t_query: float) -> Optional[Tuple[float, float]]:
        """
        Find the phase window (start,end) for the occurrence of phase_idx that contains t_query.
        If no window contains t_query, return the next occurrence after t_query.
        """
        timeline = self._phase_timeline(sim_time, horizon_sec=max(600.0, float(t_query) - float(sim_time) + 300.0))
        if not timeline:
            return None

        # 1) contains
        for p, s, e in timeline:
            if p == int(phase_idx) and float(s) <= float(t_query) < float(e):
                return (float(s), float(e))

        # 2) next after
        for p, s, e in timeline:
            if p == int(phase_idx) and float(s) >= float(t_query):
                return (float(s), float(e))

        return None

    def _get_program_phase_durations(self) -> List[float]:
        """Extract phase durations for currently active program."""
        if traci is None:
            return []
        tls_id = self.cfg.tls_id
        cur_prog = str(traci.trafficlight.getProgram(tls_id))

        try:
            logics = traci.trafficlight.getAllProgramLogics(tls_id)
        except Exception:
            return []

        logic = None
        for lg in logics:
            if getattr(lg, "programID", None) == cur_prog:
                logic = lg
                break
        if logic is None and logics:
            logic = logics[0]
        if logic is None:
            return []

        return [float(ph.duration) for ph in logic.phases]

    def _get_current_program_logic(self):
        """Return the active ProgramLogic object for this TLS, if available."""
        if traci is None:
            return None
        tls_id = self.cfg.tls_id
        cur_prog = str(traci.trafficlight.getProgram(tls_id))
        try:
            logics = traci.trafficlight.getAllProgramLogics(tls_id)
        except Exception:
            return None

        logic = None
        for lg in logics:
            if getattr(lg, "programID", None) == cur_prog:
                logic = lg
                break
        if logic is None and logics:
            logic = logics[0]
        return logic

    def _time_to_next_cycle_start(self, now: float, base_durations: List[float], cycle_start_phase: int = 0) -> Tuple[float, float]:
        """Compute L_t (remaining time of current cycle) and absolute time when the next cycle starts.

        We define a 'cycle' as one full traversal of the program phases, starting at phase index `cycle_start_phase`
        (for your experiments, this is phase 0).
        """
        if traci is None:
            return 0.0, float(now)

        tls_id = self.cfg.tls_id
        nph = len(base_durations)
        if nph == 0:
            return 0.0, float(now)

        try:
            cur_phase = int(traci.trafficlight.getPhase(tls_id))
            next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        except Exception:
            return 0.0, float(now)

        rem_cur = max(0.0, next_switch - float(now))
        L_t = float(rem_cur)

        # Sum the remaining phases until we hit the next occurrence of cycle_start_phase
        p = (cur_phase + 1) % nph
        while p != int(cycle_start_phase):
            L_t += float(base_durations[p])
            p = (p + 1) % nph

        cycle_start_time = float(now) + float(L_t)
        return float(L_t), float(cycle_start_time)

    def _resolve_traci(self):
        """Return a live traci module reference, attempting a late import if needed."""
        global traci
        if traci is not None:
            return traci
        try:
            import traci as _traci  # type: ignore
            traci = _traci
            return traci
        except Exception:
            return None

    def _refresh_lane_loop_map(self, force: bool = False) -> None:
        """Build lane -> induction-loop IDs map (static + auto-discovery)."""
        if force:
            self._lane_loops_ready = False
        if self._lane_loops_ready:
            return

        lane_to_loops: Dict[str, List[str]] = {}
        static_map = getattr(self.cfg, "queue_loop_ids_by_lane", {}) or {}
        for lane_id, loop_ids in dict(static_map).items():
            lane_to_loops[str(lane_id)] = [str(lp) for lp in (loop_ids or [])]

        # Always attempt runtime auto-discovery when TraCI is available.
        # This keeps mapping resilient even if queue_use_induction_loops/static-map flags
        # are misconfigured, while actual metric usage is still gated elsewhere.
        tc = self._resolve_traci()
        if tc is not None:
            try:
                for loop_id in list(tc.inductionloop.getIDList()):
                    try:
                        lane_id = str(tc.inductionloop.getLaneID(str(loop_id)))
                    except Exception:
                        continue
                    lane_to_loops.setdefault(lane_id, []).append(str(loop_id))
            except Exception:
                pass

        self._lane_loop_ids = lane_to_loops
        # De-duplicate loop IDs per lane (static map + runtime discovery may overlap).
        for lane_id, ids in list(self._lane_loop_ids.items()):
            seen: Set[str] = set()
            uniq_ids: List[str] = []
            for lp in (ids or []):
                s = str(lp)
                if s in seen:
                    continue
                seen.add(s)
                uniq_ids.append(s)
            self._lane_loop_ids[str(lane_id)] = uniq_ids
        self._lane_loops_ready = True

    def _lane_arrival_counts_from_loops(self, lanes: List[str], sim_time: Optional[float] = None) -> Optional[Dict[str, float]]:
        """Return per-lane counts from induction loops in the last simulation step."""
        self.last_loop_error = ""
        tc = self._resolve_traci()
        if tc is None:
            self.last_loop_error = "traci_unavailable_in_intersection_agent"
            return None

        now_t = float(self._now() if sim_time is None else sim_time)
        same_tick_cache = (
            self._loop_entry_cache_time is not None
            and abs(float(self._loop_entry_cache_time) - now_t) < 1e-9
        )
        if not same_tick_cache:
            self._loop_entry_cache_time = float(now_t)
            self._loop_entry_cache_by_loop = {}

        self._refresh_lane_loop_map()
        # If lanes are requested but no loop IDs are mapped, force one rebuild from runtime.
        if lanes and not any(self._lane_loop_ids.get(str(l), []) for l in lanes):
            self._refresh_lane_loop_map(force=True)
        counts: Dict[str, float] = {}
        loop_count_mode = str(getattr(self.cfg, "queue_loop_count_mode", "adaptive") or "adaptive").strip().lower()
        detector_freq = max(0.0, float(getattr(self.cfg, "queue_loop_detector_freq_sec", 1.0) or 0.0))
        min_poll_gap = max(0.0, float(getattr(self.cfg, "queue_loop_interval_min_poll_gap_sec", 0.5) or 0.0))
        sparse_factor = max(1.0, float(getattr(self.cfg, "queue_loop_interval_sparse_factor", 1.5) or 1.0))
        lane_dt = max(1e-6, float(getattr(self.lane_arrivals, "dt", 0.1) or 0.1))
        for lane_id in lanes:
            loop_ids = list(self._lane_loop_ids.get(str(lane_id), []))
            if not loop_ids:
                continue
            c = 0.0
            for loop_id in loop_ids:
                if loop_id in self._loop_entry_cache_by_loop:
                    c += float(self._loop_entry_cache_by_loop.get(loop_id, 0.0))
                    continue
                last_read_t = self._loop_last_read_time_by_loop.get(str(loop_id))
                poll_gap = float(now_t - float(last_read_t)) if last_read_t is not None else 0.0
                sparse_gap_threshold = max(float(min_poll_gap), float(sparse_factor * lane_dt))
                if detector_freq > 0.0:
                    # Prefer interval counts only when at least one detector interval likely elapsed.
                    sparse_gap_threshold = max(float(sparse_gap_threshold), 0.95 * float(detector_freq))
                use_interval_counts = False
                if loop_count_mode == "interval":
                    use_interval_counts = True
                elif loop_count_mode == "adaptive":
                    use_interval_counts = (last_read_t is not None and poll_gap >= sparse_gap_threshold)
                loop_id_s = str(loop_id)
                if use_interval_counts:
                    try:
                        # Detector interval count preserves arrivals when polling less often than sim-step.
                        # Works best when poll period ~= detector freq (default 1s).
                        entered = float(max(0.0, float(tc.inductionloop.getLastIntervalVehicleNumber(loop_id_s))))
                        self._loop_entry_cache_by_loop[loop_id_s] = float(entered)
                        self._loop_last_read_time_by_loop[loop_id_s] = float(now_t)
                        c += float(entered)
                        continue
                    except Exception as e_interval_mode:
                        if not self.last_loop_error:
                            self.last_loop_error = (
                                f"loop={loop_id_s} interval_mode_fallback:{type(e_interval_mode).__name__}:{e_interval_mode}"
                            )
                try:
                    # Count unique new vehicle IDs entering detector this sim step.
                    # This avoids over-counting occupancy across multiple 0.1s sub-steps.
                    cur_ids = {str(v) for v in list(tc.inductionloop.getLastStepVehicleIDs(loop_id_s))}
                    prev_ids = self._loop_prev_vehicle_ids_by_loop.get(loop_id_s, set())
                    entered = float(len(cur_ids - prev_ids))
                    self._loop_prev_vehicle_ids_by_loop[loop_id_s] = set(cur_ids)
                    self._loop_entry_cache_by_loop[loop_id_s] = float(entered)
                    self._loop_last_read_time_by_loop[loop_id_s] = float(now_t)
                    c += float(entered)
                    continue
                except Exception as e_step:
                    # Compatibility fallbacks for different SUMO/TraCI builds.
                    try:
                        c_val = float(max(0.0, float(tc.inductionloop.getLastIntervalVehicleNumber(loop_id_s))))
                        self._loop_last_read_time_by_loop[loop_id_s] = float(now_t)
                        self._loop_entry_cache_by_loop[loop_id_s] = float(c_val)
                        c += c_val
                        continue
                    except Exception as e_interval:
                        try:
                            c_val = float(len(tc.inductionloop.getVehicleData(loop_id_s)))
                            self._loop_last_read_time_by_loop[loop_id_s] = float(now_t)
                            self._loop_entry_cache_by_loop[loop_id_s] = float(c_val)
                            c += c_val
                            continue
                        except Exception as e_data:
                            if not self.last_loop_error:
                                self.last_loop_error = (
                                    f"loop={loop_id_s} "
                                    f"getLastStepVehicleNumber:{type(e_step).__name__}:{e_step} | "
                                    f"getLastIntervalVehicleNumber:{type(e_interval).__name__}:{e_interval} | "
                                    f"getVehicleData:{type(e_data).__name__}:{e_data}"
                                )
            counts[str(lane_id)] = float(c)

        return counts if counts else None

    def _update_loop_lane_counts_log(
        self,
        sim_time: float,
        edge_id: str,
        lanes: List[str],
        counts: Optional[Dict[str, float]],
        loop_ids_by_lane: Optional[Dict[str, List[str]]] = None,
        emit_log: bool = True,
    ) -> None:
        """
        Cache and print loop detections per lane for this queue-metric evaluation.
        Called every time _queue_clearing_metrics_for_edge runs.
        """
        self.last_loop_lanes = [str(l) for l in (lanes or [])]
        if loop_ids_by_lane is None:
            self.last_loop_ids_by_lane = {str(l): [] for l in self.last_loop_lanes}
        else:
            self.last_loop_ids_by_lane = {
                str(l): [str(x) for x in (loop_ids_by_lane.get(str(l), []) or [])]
                for l in self.last_loop_lanes
            }
        if counts is None:
            self.last_loop_counts_by_lane = {str(l): 0.0 for l in self.last_loop_lanes}
        else:
            self.last_loop_counts_by_lane = {
                str(l): float(counts.get(str(l), 0.0)) for l in self.last_loop_lanes
            }
        has_any_loop = any(len(v) > 0 for v in self.last_loop_ids_by_lane.values())
        if counts is None and not has_any_loop:
            self.last_loop_source = "no_loop_ids_mapped"
        elif counts is None and has_any_loop:
            self.last_loop_source = "loop_read_failed"
        else:
            self.last_loop_source = "loop_counts"

        # Keep a cumulative counter so runtime diagnostics can show total detections,
        # not only per-step crossings.
        t_now = float(sim_time)
        for lane_id in self.last_loop_lanes:
            lid = str(lane_id)
            last_t = self._loop_cum_last_update_t_by_lane.get(lid)
            if last_t is not None and abs(float(last_t) - t_now) < 1e-9:
                continue
            prev = float(self._loop_counts_cum_by_lane.get(lid, 0.0))
            inc = float(self.last_loop_counts_by_lane.get(lid, 0.0))
            self._loop_counts_cum_by_lane[lid] = float(prev + inc)
            self._loop_cum_last_update_t_by_lane[lid] = t_now
        self.last_loop_counts_cum_by_lane = {
            str(l): float(self._loop_counts_cum_by_lane.get(str(l), 0.0))
            for l in self.last_loop_lanes
        }

        if emit_log and bool(getattr(self.cfg, "enable_queue_metrics_debug", False)):
            # Avoid duplicate loop prints at the same sim-time for the same edge.
            key = str(edge_id)
            t_now = float(sim_time)
            last_t = self._loop_debug_last_t_by_edge.get(key)
            if last_t is not None and abs(t_now - float(last_t)) < 1e-9:
                return
            self._loop_debug_last_t_by_edge[key] = t_now
            err = str(self.last_loop_error or "").replace("\n", " ").strip()
            if len(err) > 220:
                err = err[:220] + "..."
            print(
                "[LOOP_DEBUG] "
                f"tls={self.cfg.tls_id} "
                f"t={float(sim_time):.2f} "
                f"edge={edge_id} "
                f"lanes={self.last_loop_lanes} "
                f"loop_ids={self.last_loop_ids_by_lane} "
                f"cfg_use_loops={bool(getattr(self.cfg, 'queue_use_induction_loops', False))} "
                f"src={self.last_loop_source} "
                f"err={err if err else '-'} "
                f"counts_step={self.last_loop_counts_by_lane} "
                f"counts_cum={self.last_loop_counts_cum_by_lane}"
            )

    def trace_loop_detections(
        self,
        sim_time: Optional[float] = None,
        edge_id: Optional[str] = None,
        lanes: Optional[List[str]] = None,
        emit_log: bool = True,
    ) -> Tuple[List[str], Dict[str, float]]:
        """
        Standalone helper for runtime loop inspection, independent of queue metrics.

        Updates:
          - self.last_loop_lanes
          - self.last_loop_counts_by_lane
        and returns (lanes, counts).
        """
        tc = self._resolve_traci()
        if tc is None:
            self.last_loop_lanes = []
            self.last_loop_counts_by_lane = {}
            return [], {}

        inspect_edge = str(edge_id or "")
        inspect_lanes = [str(l) for l in (lanes or [])]

        # Build lane list if caller did not provide explicit lanes.
        if not inspect_lanes:
            if inspect_edge:
                try:
                    nlanes = max(int(tc.edge.getLaneNumber(inspect_edge)), 1)
                except Exception:
                    nlanes = 1
                inspect_lanes = [f"{inspect_edge}_{i}" for i in range(nlanes)]
            elif self.active_ev is not None and str(getattr(self.active_ev, "in_edge_id", "")):
                inspect_edge = str(self.active_ev.in_edge_id)
                if not inspect_edge.startswith(":"):
                    try:
                        nlanes = max(int(tc.edge.getLaneNumber(inspect_edge)), 1)
                    except Exception:
                        nlanes = 1
                    inspect_lanes = [f"{inspect_edge}_{i}" for i in range(nlanes)]
            if not inspect_lanes:
                self._refresh_lane_loop_map()
                inspect_lanes = sorted(self._lane_loop_ids.keys())
                if not inspect_edge:
                    inspect_edge = "<all_loop_lanes>"

        self._refresh_lane_loop_map()
        if inspect_lanes and not any(self._lane_loop_ids.get(str(l), []) for l in inspect_lanes):
            self._refresh_lane_loop_map(force=True)
        loop_ids_by_lane = {
            str(lane_id): [str(x) for x in (self._lane_loop_ids.get(str(lane_id), []) or [])]
            for lane_id in inspect_lanes
        }
        t_now = float(self._now() if sim_time is None else sim_time)
        counts = self._lane_arrival_counts_from_loops(inspect_lanes, sim_time=t_now)
        self._update_loop_lane_counts_log(
            sim_time=t_now,
            edge_id=(inspect_edge or "<none>"),
            lanes=inspect_lanes,
            counts=counts,
            loop_ids_by_lane=loop_ids_by_lane,
            emit_log=bool(emit_log),
        )
        return list(self.last_loop_lanes), dict(self.last_loop_counts_by_lane)

    def _queue_clearing_metrics_for_edge(self, edge_id: str, ev_id: Optional[str] = None) -> Tuple[float, float, float, float, List[str]]:
        """Return (N, A, S, Q, lanes) for the paper option-2 queue model.

        Option A requested by you:
        - If we can identify the EV's current approach lane on `edge_id`, use *that lane only*.
        - Otherwise, approximate by summing across all lanes on `edge_id`.

        N: queued vehicles (EV-ahead when EV lane is known; else lane halting proxy)
        A: arrival rate (veh/s) estimated by lane arrivals EMA (optionally from induction loops)
        S: saturation flow (veh/s)
        Q: clearing time (sec) = N / (S - A)
        """
        tc = self._resolve_traci()
        if tc is None or (not edge_id) or edge_id.startswith(":"):
            return 0.0, 0.0, 0.0, 0.0, []

        lanes: List[str] = []

        # Try to use EV's current lane if it is on this approach edge.
        if ev_id:
            try:
                lane_id = str(tc.vehicle.getLaneID(ev_id))
                lane_edge = lane_id.rsplit("_", 1)[0] if "_" in lane_id else lane_id
                if lane_edge == edge_id:
                    lanes = [lane_id]
            except Exception:
                lanes = []

        # Fallback: all lanes on the edge
        if not lanes:
            try:
                nlanes = max(int(tc.edge.getLaneNumber(edge_id)), 1)
            except Exception:
                nlanes = 1
            lanes = [f"{edge_id}_{i}" for i in range(nlanes)]

        strict_mode = bool(getattr(self.cfg, "queue_metrics_paper_strict_mode", False))

        # Queue length proxy N:
        # - strict paper mode: full EV-entrance-lane queued vehicles (halting count on EV lane)
        # - robust mode: EV-lane vehicles stopped ahead of EV only
        # - fallback: sum halting counts across lanes
        N = 0.0
        n_from_ev_ahead: Optional[float] = None
        if ev_id and len(lanes) == 1:
            if strict_mode:
                try:
                    n_lane = float(tc.lane.getLastStepHaltingNumber(lanes[0]))
                    N = float(max(0.0, n_lane))
                    n_from_ev_ahead = float(N)
                except Exception:
                    n_from_ev_ahead = None
            else:
                try:
                    ev_lane = lanes[0]
                    ev_pos = float(tc.vehicle.getLanePosition(ev_id))
                    vids = tc.lane.getLastStepVehicleIDs(ev_lane)
                    ahead = 0.0
                    for vid in vids:
                        if vid == ev_id:
                            continue
                        try:
                            if float(tc.vehicle.getLanePosition(vid)) > ev_pos and float(tc.vehicle.getSpeed(vid)) < 0.1:
                                ahead += 1.0
                        except Exception:
                            pass
                    n_from_ev_ahead = float(ahead)
                except Exception:
                    n_from_ev_ahead = None

        if n_from_ev_ahead is not None:
            if not strict_mode:
                # Keep robust EV-ahead semantics: do not back-fill with vehicles behind the EV.
                N = float(max(0.0, n_from_ev_ahead))
        else:
            for lid in lanes:
                try:
                    N += float(tc.lane.getLastStepHaltingNumber(lid))
                except Exception:
                    pass

        # Arrival rate (veh/s): EMA over lane arrivals.
        # strict paper mode:
        #   - queue_use_induction_loops=True  -> loop-only counts (no hidden fallback)
        #   - queue_use_induction_loops=False -> lane-ID based arrivals
        # robust mode:
        #   - keep existing behavior (attempt loop counts opportunistically)
        sim_now = float(self._now())
        loop_counts: Optional[Dict[str, float]] = None
        a_source = "lane_ids"
        if strict_mode:
            use_loops = bool(getattr(self.cfg, "queue_use_induction_loops", False))
            if use_loops:
                loop_counts = self._lane_arrival_counts_from_loops(lanes, sim_time=sim_now)
                if loop_counts is None:
                    # Explicit gating: do not silently fall back to lane-ID arrivals when loop mode is requested.
                    forced_zero = {str(l): 0.0 for l in lanes}
                    A_raw = float(self.lane_arrivals.update_many(lanes, sim_time=sim_now, inst_counts=forced_zero))
                    a_source = "loops_missing_forced_zero"
                else:
                    A_raw = float(self.lane_arrivals.update_many(lanes, sim_time=sim_now, inst_counts=loop_counts))
                    a_source = "loops"
            else:
                A_raw = float(self.lane_arrivals.update_many(lanes, sim_time=sim_now, inst_counts=None))
                a_source = "lane_ids_gated"
        else:
            loop_counts = self._lane_arrival_counts_from_loops(lanes, sim_time=sim_now)
            A_raw = float(self.lane_arrivals.update_many(lanes, sim_time=sim_now, inst_counts=loop_counts))
            a_source = "loops" if loop_counts is not None else "lane_ids"

        self._update_loop_lane_counts_log(
            sim_time=sim_now,
            edge_id=str(edge_id),
            lanes=list(lanes),
            counts=loop_counts,
            loop_ids_by_lane={
                str(l): [str(x) for x in (self._lane_loop_ids.get(str(l), []) or [])]
                for l in lanes
            },
        )

        # Saturation flow (veh/s)
        S = float(getattr(self.cfg, "saturation_flow_per_lane_vehps", 0.55)) * float(len(lanes))
        S = max(0.0, float(S))

        # Stabilize A to avoid numerical/estimation spikes dominating Q.
        cap_ratio = max(0.0, min(0.999, float(getattr(self.cfg, "queue_arrival_cap_ratio_to_s", 0.98))))
        A_cap = float(cap_ratio * S)
        A = float(min(max(0.0, A_raw), A_cap)) if S > 0.0 else 0.0

        denom_min = max(1e-6, float(getattr(self.cfg, "queue_denom_min_vehps", 0.05)))
        q_cap = max(1.0, float(getattr(self.cfg, "queue_clear_time_cap_sec", 120.0)))

        if N <= 0.0:
            Q = 0.0
        else:
            denom = float(S - A)
            if denom <= denom_min:
                Q = float(q_cap)
            else:
                Q = float(min(q_cap, float(N / denom)))

        self._print_queue_metrics_debug(
            sim_time=sim_now,
            edge_id=str(edge_id),
            lanes=list(lanes),
            N=float(N),
            A_raw=float(A_raw),
            A_used=float(A),
            S=float(S),
            Q=float(Q),
            source=str(a_source),
        )

        return float(N), float(A), float(S), float(Q), lanes

    def _estimate_cycle_sec_for_queue_metrics(self) -> float:
        durations = self._get_program_phase_durations()
        if durations:
            cyc = float(sum(float(x) for x in durations))
            if cyc > 1e-6:
                return float(cyc)
        return float(max(1.0, float(getattr(self.cfg, "queue_metrics_cycle_fallback_sec", 90.0))))

    def _estimate_dynamic_t_lost_sec(
        self,
        ev: Optional[EvRequest],
        lanes: List[str],
        queue_n: float,
    ) -> float:
        base = max(0.0, float(getattr(self.cfg, "T_lost_sec", 5.0)))
        tmin = max(0.0, float(getattr(self.cfg, "queue_metrics_t_lost_min_sec", 0.8)))
        tmax = max(tmin, float(getattr(self.cfg, "queue_metrics_t_lost_max_sec", 6.0)))
        if not bool(getattr(self.cfg, "queue_metrics_use_dynamic_t_lost", True)):
            return float(max(tmin, min(tmax, base)))
        if ev is None or traci is None:
            return float(max(tmin, min(tmax, base)))

        v_ev = max(0.0, float(getattr(ev, "speed_mps", 0.0)))
        v_ref = 13.89
        tc = self._resolve_traci()
        if tc is not None:
            try:
                lane_id = str(tc.vehicle.getLaneID(str(ev.ev_id)))
                v_ref = float(tc.lane.getMaxSpeed(lane_id))
            except Exception:
                v_ref = 13.89
        v_ref = max(2.0, float(v_ref))

        speed_factor = max(0.0, min(1.0, 1.0 - (v_ev / v_ref)))
        q_ref = max(1.0, 2.0 * float(max(1, len(lanes))))
        queue_factor = max(0.0, min(1.0, float(queue_n) / q_ref))

        # Dynamic startup-loss proxy:
        # - lower when EV speed is high and queue is short
        # - closer to base when EV is slow and queue is heavy
        t_est = base * (0.40 + 0.60 * speed_factor) + 0.60 * queue_factor
        return float(max(tmin, min(tmax, t_est)))

    def _estimate_dynamic_yt_sec(
        self,
        ev: Optional[EvRequest],
        lanes: List[str],
    ) -> float:
        yt_static = max(0.0, float(getattr(self.cfg, "YT_sec", 5.0)))
        if not bool(getattr(self.cfg, "queue_metrics_use_dynamic_yt", True)):
            return float(yt_static)
        tc = self._resolve_traci()
        if tc is None:
            return float(yt_static)

        halts = 0.0
        vehs = 0.0
        for lid in lanes:
            try:
                halts += float(tc.lane.getLastStepHaltingNumber(lid))
                vehs += float(tc.lane.getLastStepVehicleNumber(lid))
            except Exception:
                continue
        moving = max(0.0, vehs - halts)

        v_ev = max(0.0, float(getattr(ev, "speed_mps", 0.0))) if ev is not None else 0.0
        v_ref = 13.89
        if ev is not None:
            try:
                lane_id = str(tc.vehicle.getLaneID(str(ev.ev_id)))
                v_ref = float(tc.lane.getMaxSpeed(lane_id))
            except Exception:
                v_ref = 13.89
        v_ref = max(2.0, float(v_ref))
        speed_factor = max(0.0, min(1.0, v_ev / v_ref))

        yt = (
            float(getattr(self.cfg, "queue_metrics_yt_base_sec", 0.8))
            + float(getattr(self.cfg, "queue_metrics_yt_per_halt_sec", 0.35)) * halts
            + float(getattr(self.cfg, "queue_metrics_yt_per_moving_sec", 0.15)) * moving
            + float(getattr(self.cfg, "queue_metrics_yt_speed_weight_sec", 0.8)) * speed_factor
        )
        yt = max(0.0, min(float(getattr(self.cfg, "queue_metrics_yt_max_sec", 5.0)), yt))
        return float(yt)

    def _queue_clearing_metrics_improved(
        self,
        edge_id: str,
        ev_id: Optional[str] = None,
        sim_time: Optional[float] = None,
        t_i: Optional[float] = None,
        green_window: Optional[Tuple[float, float]] = None,
        arrival_window: Optional[Tuple[float, float]] = None,
    ) -> Dict[str, float]:
        """
        Paper-grounded queue-clearing terms for offer metrics.

        Zhong & Chen (Eq. 6):
          delta_w_i = 1_i * (Q_i + (n_i + 1) * T_lost + YT)

        Here we estimate:
          - Q_i from the existing lane model (N, A, S)
          - n_i from ETA vs cycle time (coarse but stable)
          - dynamic T_lost and YT (with static fallbacks)
          - q_delay = max(0, delta_w_i - pre-arrival available green)
        """
        strict_mode = bool(getattr(self.cfg, "queue_metrics_paper_strict_mode", False))
        N, A, S, Q, lanes = self._queue_clearing_metrics_for_edge(edge_id=str(edge_id), ev_id=ev_id)
        t_now = float(self._now() if sim_time is None else sim_time)
        ev = self.active_ev if (self.active_ev is not None and (ev_id is None or str(self.active_ev.ev_id) == str(ev_id))) else None

        queue_indicator = 1.0 if float(N) > 0.0 else 0.0
        cycle_sec = float(self._estimate_cycle_sec_for_queue_metrics())

        if t_i is None:
            if arrival_window is not None:
                t_center = 0.5 * (float(arrival_window[0]) + float(arrival_window[1]))
            else:
                t_center = float(t_now)
        else:
            t_center = float(t_i)

        if green_window is not None:
            gw0 = float(green_window[0])
        else:
            gw0 = float(t_now)
        n_i = 0
        if t_center > gw0 + 1e-6 and cycle_sec > 1e-6:
            n_i = max(0, int((t_center - gw0) / cycle_sec))

        if strict_mode:
            t_lost_est = max(0.0, float(getattr(self.cfg, "T_lost_sec", 5.0)))
            yt_est = max(0.0, float(getattr(self.cfg, "YT_sec", 5.0)))
        else:
            t_lost_est = self._estimate_dynamic_t_lost_sec(ev=ev, lanes=lanes, queue_n=float(N))
            yt_est = self._estimate_dynamic_yt_sec(ev=ev, lanes=lanes)
        delta_w = float(queue_indicator * (float(Q) + (float(n_i) + 1.0) * float(t_lost_est) + float(yt_est)))

        avail_pre = 0.0
        if green_window is not None and arrival_window is not None:
            gw0, gw1 = float(green_window[0]), float(green_window[1])
            ar0 = float(arrival_window[0])
            # only pre-arrival green that is still usable from "now"
            g_start_eff = max(gw0, t_now)
            avail_pre = max(0.0, min(gw1, ar0) - g_start_eff)

        q_delay = max(0.0, float(delta_w) - float(avail_pre))
        return {
            "N": float(N),
            "A": float(A),
            "S": float(S),
            "Q": float(Q),
            "queue_indicator": float(queue_indicator),
            "cycle_sec": float(cycle_sec),
            "n_i": float(n_i),
            "t_lost_sec": float(t_lost_est),
            "yt_sec": float(yt_est),
            "delta_w_sec": float(delta_w),
            "avail_pre_green_sec": float(avail_pre),
            "q_delay_sec": float(q_delay),
        }

    def _activate_phase_overrides_from_plan(self, plan: PreemptionPlan) -> None:
        """Store paper-QP overrides so they can be applied as phases become current."""
        if plan is None:
            return
        if plan.phase_duration_overrides is None:
            return
        self._active_phase_overrides = dict(plan.phase_duration_overrides)
        self._override_start_time_sec = float(plan.override_start_time_sec or 0.0)
        self._override_end_time_sec = float(plan.override_end_time_sec or 0.0)
        self._override_applied_in_current_phase = False
        # reset tracking so we apply on next phase edge
        self._override_last_seen_phase = None
        self._phase_change_time_sec = float(plan.override_start_time_sec or 0.0)

    def _apply_active_phase_overrides(self, sim_time: float) -> None:
        """Apply stored paper-QP phase duration overrides when a phase becomes current.

        We only apply an override very near the start of a phase (grace window) to avoid
        mid-phase discontinuities.
        """
        if traci is None:
            return
        if self._active_phase_overrides is None:
            return

        now = float(sim_time)
        if now < float(self._override_start_time_sec) - 1e-6:
            return
        if self._override_end_time_sec > 0.0 and now > float(self._override_end_time_sec) + 1e-6:
            # Safety stop
            self._active_phase_overrides = None
            return

        tls_id = self.cfg.tls_id

        try:
            cur_phase = int(traci.trafficlight.getPhase(tls_id))
            next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        except Exception:
            return

        # Track phase changes
        if self._override_last_seen_phase is None or cur_phase != int(self._override_last_seen_phase):
            self._override_last_seen_phase = int(cur_phase)
            self._phase_change_time_sec = float(now)
            self._override_applied_in_current_phase = False

        if self._override_applied_in_current_phase:
            return

        if int(cur_phase) not in self._active_phase_overrides:
            return

        elapsed = float(now) - float(self._phase_change_time_sec)
        grace = float(getattr(self.cfg, "override_apply_grace_sec", 0.5))
        if elapsed > grace:
            return

        desired_total = float(self._active_phase_overrides[int(cur_phase)])

        # Compute desired remaining so that total duration ~ desired_total from phase start.
        desired_remaining = float(desired_total - elapsed)
        # setPhaseDuration sets remaining, so clamp to something small but non-negative
        desired_remaining = max(desired_remaining, 0.01)

        # Optional: do not shorten a green below configured minimum remaining
        min_rem = float(getattr(self.cfg, "min_current_phase_remaining_sec", 0.0))
        desired_remaining = max(desired_remaining, min_rem)

        # Apply
        try:
            traci.trafficlight.setPhaseDuration(tls_id, float(desired_remaining))
            # offset accounting: compare to current remaining
            current_remaining = max(0.0, float(next_switch) - float(now))
            self._timing_offset_sec += float(desired_remaining - current_remaining)
        except Exception:
            return

        self._override_applied_in_current_phase = True

    # =========================
    # Non-intrusive preemption with QP feasibility check (reduced)
    # =========================

    #def _try_non_intrusive_paper_qp(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        """
        Paper-style *non-intrusive* optimization (Section IV-B):

        Decision variables: green durations within a cycle (split adjustment).
        - We keep yellow/all-red (SIT) phases fixed.
        - We do NOT perform an intrusive "jump"; only adjust green times.

        Key paper quantities (mapped to our implementation):
          - L_t : remaining time of the current cycle at request time
          - T'  : new cycle length after split adjustment
          - n   : number of *full* new cycles before the EV-arrival cycle
          - P   : time within the EV-arrival cycle where the EV arrives
          - EAT/LAT : earliest/latest allowable arrival time within the target green
          - Q_i : queue clearing time (option-2 lane queue model)
          - Queue clearing constraint: enough green must be given before EV arrives

        Implementation notes (SUMO/TraCI practicalities):
          - We apply the new green splits starting at the next cycle boundary (phase 0 start),
            matching the paper's use of an "initial cycle" with remaining time L_t.
          - We solve a small convex QP with cvxpy. If cvxpy is unavailable, we fall back to None.
        """
        """
        if traci is None or self.active_ev is None:
            print(f"self.active_ev: {self.active_ev}")
            return None"""
        #if cp is None or not bool(getattr(self.cfg, "enable_non_intrusive_qp", True)):
        #    print(f"enable_non_intrusive_qp: {self.cfg.enable_non_intrusive_qp}")
        #    return None

        """
        tls_id = self.cfg.tls_id
        now = float(sim_time)

        ev = self.active_ev
        target_phase = int(ev.target_phase_idx or 0)
        delta = float(ev.delta_sec)

        # arrival window
        arrival_start, arrival_end = float(t_i) - delta, float(t_i) + delta

        # Pull current program phases (duration + state) so we can detect green vs SIT phases.
        prog = self._get_current_program_logic()
        if prog is None:
            print(f"prog: {prog}")
            return None
        phases = list(getattr(prog, "phases", []))
        if not phases:
            print(f"phases: {phases}")
            return None

        dur0 = [float(getattr(ph, "duration", 0.0)) for ph in phases]
        st0 = [str(getattr(ph, "state", "")) for ph in phases]
        nph = len(dur0)

        # Identify "green" phases as those whose state contains any G/g.
        green_idx = [i for i, st in enumerate(st0) if any(c in ("G", "g") for c in st)]
        if target_phase not in green_idx:
            print("target_phase not in green_idx")
            # target must be a green phase (in your case: 0 or 2)
            return None

        # Fixed SIT duration per phase (yellow/all-red etc.)
        sit_fixed = {i: dur0[i] for i in range(nph) if i not in green_idx}
        sit_total = float(sum(sit_fixed.values()))

        # --- Remaining time of the *initial* cycle L_t (paper) ---
        L_t, next_cycle_start_time = self._time_to_next_cycle_start(now, dur0, cycle_start_phase=0)

        # If EV arrives before the next cycle boundary, the paper-QP isn't the right tool; let reduced-QP handle.
        if arrival_end <= float(next_cycle_start_time) + 1e-6:
            print(f"arrival_end: {arrival_end} <= float(cycle_start_time): {next_cycle_start_time}")
            print(f"Trying non intrusive method instead")
            return self._try_non_intrusive_qp(sim_time, t_i)            
            #return None

        # Time from next cycle boundary until EV arrival (paper uses t_i - t - L_t).
        R = float(t_i) - float(now) - float(L_t)
        if R < 0.0:
            print("R < 0.0")
            return None

        # --- Queue-clearing model (paper option 2) ---
        # Queue defined on the EV entrance lane if known; otherwise all lanes on the EV inbound edge.
        Nq, Aq, Sq, Q_i, queue_lanes = self._queue_clearing_metrics_for_edge(
            edge_id=str(ev.in_edge_id),
            ev_id=str(ev.ev_id)
        )

        queue_indicator = 1.0 if Nq >= 1.0 else 0.0
        T_lost = float(getattr(self.cfg, "T_lost_sec", 0.0))
        YT = float(getattr(self.cfg, "YT_sec", 0.0))

        # --- Decision variables: green durations for each green phase ---
        g = {i: cp.Variable(name=f"g_{i}") for i in green_idx}

        # Phase duration expression list (greens are variables, SIT are fixed)
        dur_expr = [g[i] if i in green_idx else float(dur0[i]) for i in range(nph)]
        T = cp.sum([g[i] for i in green_idx]) + float(sit_total)

        # Bounds (tau_min/max applied to green phases only)
        tau_min = float(getattr(self.cfg, "tau_min_sec", 1.0))
        tau_max = float(getattr(self.cfg, "tau_max_sec", 120.0))

        constraints_base = []
        for i in green_idx:
            constraints_base += [g[i] >= tau_min, g[i] <= tau_max]

        # Cycle length bounds (derived)
        T_min = float(len(green_idx) * tau_min + sit_total)
        T_max = float(len(green_idx) * tau_max + sit_total)
        constraints_base += [T >= T_min, T <= T_max]

        # Start offsets within the cycle (affine expressions)
        start_expr = []
        acc = 0
        for j in range(nph):
            start_expr.append(acc)
            acc = acc + dur_expr[j]

        # EAT/LAT for the target phase
        EAT = start_expr[target_phase]
        LAT = EAT + dur_expr[target_phase]
        g_target = dur_expr[target_phase]

        # Objective (paper-style "minimal disruption"):
        # - keep total cycle length near baseline
        # - keep green splits near baseline
        beta = float(getattr(self.cfg, "qp_beta_green_dev", 1.0))
        T0 = float(sum(dur0))
        obj = cp.square(T - T0) + beta * cp.sum([cp.square(g[i] - float(dur0[i])) for i in green_idx])

        # n = number of full adjusted cycles before the arrival cycle:
        # n = floor(R / T). We enumerate a small set of possible n and enforce consistency:
        #   n*T <= R < (n+1)*T
        # This makes P = R - n*T affine in the decision variables.
        # n_max based on smallest possible cycle length.
        n_max = int(max(0.0, R) // max(T_min, 1e-6)) + 1
        n_max = int(min(n_max, int(getattr(self.cfg, "qp_n_max", 6))))

        best = None  # (cost, n, g_values)
        eps = 1e-6

        for n in range(0, n_max + 1):
            constraints = list(constraints_base)

            constraints += [float(n) * T <= float(R) + eps,
                            float(R) <= float(n + 1) * T - eps]

            P = float(R) - float(n) * T  # time within arrival cycle when EV arrives

            # Arrival window must lie inside target green (paper constraints with EAT/LAT)
            constraints += [P - float(delta) >= EAT,
                            P + float(delta) <= LAT]

            # Queue clearing constraint:
            # total green available before EV arrives in the target movement:
            #   n * g_target  (full cycles) + (P - EAT) (partial within arrival cycle)
            # must exceed the required clearing time:
            #   Q_i + (n+1)*T_lost + YT   (only when queue exists)
            required = queue_indicator * (float(Q_i) + float(n + 1) * float(T_lost) + float(YT))
            constraints += [float(n) * g_target + (P - EAT) >= required]

            prob = cp.Problem(cp.Minimize(obj), constraints)
            try:
                prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            except Exception:
                try:
                    prob.solve(solver=cp.ECOS, warm_start=True, verbose=False)
                except Exception:
                    prob.solve(warm_start=True, verbose=False)

            if prob.status not in ("optimal", "optimal_inaccurate"):
                continue

            g_star = {i: float(g[i].value) for i in green_idx}
            # Compute achieved objective
            cost = float(prob.value)

            if best is None or cost < best[0]:
                best = (cost, int(n), g_star)

        if best is None:
            return None

        _, n_opt, g_star = best

        # Compute numeric outputs for reporting / planned window prediction
        # T_opt and EAT/LAT under solution
        # (Recompute offsets with the solution values)
        dur_star = []
        for j in range(nph):
            if j in green_idx:
                dur_star.append(float(g_star[j]))
            else:
                dur_star.append(float(dur0[j]))

        T_opt = float(sum(dur_star))
        start_star = [0.0] * nph
        acc = 0.0
        for j in range(nph):
            start_star[j] = acc
            acc += dur_star[j]

        EAT_star = float(start_star[target_phase])
        LAT_star = float(EAT_star + dur_star[target_phase])

        # Arrival-cycle absolute start time (cycle boundary + n full cycles)
        cycle_k_start = float(next_cycle_start_time) + float(n_opt) * float(T_opt)
        planned_start = cycle_k_start + EAT_star
        planned_end = cycle_k_start + LAT_star

        overrides = {int(i): float(g_star[i]) for i in green_idx}

        # Apply overrides starting at the next cycle boundary (paper's initial-cycle handling).
        start_apply = float(next_cycle_start_time)
        # Safety end: a bit after the EV arrival window (if EV disappears, we don't keep overrides forever)
        end_apply = float(arrival_end) + float(getattr(self.cfg, "qp_override_extra_sec", 10.0))

        notes = (
            f"Paper-QP feasible: target={target_phase}, L_t={L_t:.2f}, R={R:.2f}, n={n_opt}, "
            f"T0={T0:.2f}->T'={T_opt:.2f}, "
            f"EAT={EAT_star:.2f}, LAT={LAT_star:.2f}, "
            f"queue_lanes={queue_lanes}, N={Nq:.1f}, A={Aq:.3f}, S={Sq:.3f}, Q={Q_i:.2f}, "
            f"overrides={overrides}"
        )

        return PreemptionPlan(
            plan_type="non_intrusive",
            target_phase_idx=int(target_phase),
            extend_green_sec=0.0,  # we are using split overrides, not ad-hoc per-tick extension
            planned_green_window=(float(planned_start), float(planned_end)),
            phase_duration_overrides=overrides,
            override_start_time_sec=float(start_apply),
            override_end_time_sec=float(end_apply),
            notes=notes
        )"""

    def _try_non_intrusive_paper_qp(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        """Paper-aligned non-intrusive attempt.

        This path delegates to the reduced QP solver (TraCI-feasible) and annotates
        outputs with paper variables (L_t, R, Q_i) for runtime validation.
        """
        ev = self.active_ev
        if ev is None:
            self._b1_dbg("qp_paper_return none reason=no_active_ev")
            return None

        # Use optimization-based non-intrusive proposal.
        plan = self._try_non_intrusive_qp(float(sim_time), float(t_i))
        if plan is None:
            self._b1_dbg("qp_paper_return none reason=reduced_qp_unfeasible")
            return None

        # Add paper terms for observability/comparison.
        target_phase = int(ev.target_phase_idx or 0)
        prog = self._get_current_program_logic()
        if prog is not None and getattr(prog, "phases", None):
            dur0 = [float(getattr(ph, "duration", 0.0)) for ph in prog.phases]
            L_t, _next_cycle_start_time = self._time_to_next_cycle_start(float(sim_time), dur0, cycle_start_phase=0)
        else:
            L_t = 0.0
        R = float(t_i) - float(sim_time) - float(L_t)
        _Nq, _Aq, _Sq, Q_i, _lanes = self._queue_clearing_metrics_for_edge(
            edge_id=str(ev.in_edge_id),
            ev_id=str(ev.ev_id),
        )
        plan.notes = (
            f"Paper-aligned reduced-QP: target={target_phase}, L_t={L_t:.2f}, R={R:.2f}, Q_i={Q_i:.2f}; "
            f"{plan.notes}"
        )
        self._print_compact_decision_debug(
            sim_time=float(sim_time),
            mode="non_intrusive",
            L_t=float(L_t),
            R=float(R),
            Q_i=float(Q_i),
        )
        self._b1_dbg(
            f"qp_paper_return plan_type={plan.plan_type} target={getattr(plan, 'target_phase_idx', None)} "
            f"L_t={float(L_t):.2f} R={float(R):.2f} Q_i={float(Q_i):.2f}"
        )
        return plan


    #def _try_non_intrusive_qp(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        """
        Return a "non_intrusive" plan ONLY if QP is feasible.

        Reduced decision variables (TraCI-realistic):
          r  = remaining time of the CURRENT phase (can only be shortened)
          g  = effective duration of the TARGET phase occurrence (can be extended when it is active)

        Constraints:
          r_min <= r <= r0
          g0 <= g <= g0 + g_ext_cap
          (start_target_new <= arrival_start) AND (end_target_new >= arrival_end)

        Objective:
          minimize (r - r0)^2 + (g - g0)^2
        """
        
        """
        if traci is None or self.active_ev is None:
            return None

        tls_id = self.cfg.tls_id
        now = float(sim_time)

        target_phase = int(self.active_ev.target_phase_idx or 0)
        delta = float(self.active_ev.delta_sec)

        arrival_start, arrival_end = float(t_i) - delta, float(t_i) + delta

        # Base next target window (no intervention)
        print(f"Expected target phase: {target_phase} for node: {tls_id}")
        print(f"Arrival_start, arrival_end: t_i - delta: {t_i - delta}, t_i + delta: {t_i + delta}")

        base_win = self._predict_next_phase_window(now, target_phase)

        if base_win is None:
            return None

        # Current phase state
        cur_phase = int(traci.trafficlight.getPhase(tls_id))
        next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        print(f" ----- Next_switch: {next_switch}")
        r0 = max(0.0, next_switch - now)
        r_min = float(self.cfg.min_current_phase_remaining_sec)

        # If target is current, start is now; g0 is remaining
        if cur_phase == target_phase:
            g0 = r0
            # We can only extend current green (g >= g0)
            feasible = (now <= arrival_start) and (arrival_end <= now + (g0 + float(self.cfg.max_target_green_extension_sec)))
            print(f"Feasible time for extension: now:{now}, arrival_start: {arrival_start}, arrival_end: {arrival_end} , g0:{g0}, self.cfg.max_target_green_extension_sec: {self.cfg.max_target_green_extension_sec} ")
            if not feasible:
                return None
            
            print(f" ***** Phase matches target phase ****** ")

            need_ext = max(0.0, arrival_end - (now + g0))
            ext = min(max(need_ext, 0.0), float(self.cfg.max_target_green_extension_sec))

            print(f"need_ext: {need_ext}, ext:{ext}")

            print(f"now {now} + g0 {g0}+ ext ({ext})")

            return PreemptionPlan(
                plan_type="non_intrusive",
                target_phase_idx=target_phase,
                extend_green_sec=float(ext),
                planned_green_window=(now, now + g0 + float(ext)),
                notes=f"QP-feasible (target=current): extend by {ext:.2f}s to cover [{arrival_start:.2f},{arrival_end:.2f}]"
            )

        # Target is in the future. The base window is (s0,e0) for the next occurrence.
        s0, e0 = float(base_win[0]), float(base_win[1])
        g0 = max(0.0, e0 - s0)

        # The only way to shift target earlier is to shorten the CURRENT phase (not intermediate phases).
        # shift_max = r0 - r_min (how much earlier we can make the next switch)
        shift_max = max(0.0, r0 - r_min)

        earliest_start = s0 - shift_max
        latest_end = e0 + float(self.cfg.max_target_green_extension_sec)

        # Quick feasibility screening (equivalent to reduced-QP feasibility)
        if arrival_start < earliest_start - 1e-6:
            return None
        if arrival_end > latest_end + 1e-6:
            return None

        # Solve reduced QP (if cvxpy available), else closed-form
        r_star = None
        g_star = None

        if self.cfg.enable_non_intrusive_qp and cp is not None:
            r = cp.Variable()
            g = cp.Variable()
            # start/end as affine functions (only current phase shift affects start)
            start = s0 - (r0 - r)
            end = start + g

            constraints = [
                r >= r_min,
                r <= r0,
                g >= g0,
                g <= g0 + float(self.cfg.max_target_green_extension_sec),
                start <= arrival_start,
                end >= arrival_end,
            ]
            obj = cp.Minimize(cp.square(r - r0) + cp.square(g - g0))
            prob = cp.Problem(obj, constraints)
            try:
                prob.solve(solver=cp.ECOS, warm_start=True, verbose=False)
            except Exception:
                try:
                    prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
                except Exception:
                    prob.solve(warm_start=True, verbose=False)

            if prob.status not in ("optimal", "optimal_inaccurate"):
                return None

            r_star = float(r.value)
            g_star = float(g.value)
        else:
            # Closed-form for this reduced QP:
            # choose start as late as possible but <= arrival_start, within [earliest_start, s0]
            start_star = min(s0, arrival_start)
            start_star = max(start_star, earliest_start)
            # corresponding r to achieve that start: start = s0 - (r0 - r) -> r = r0 - (s0 - start)
            r_star = r0 - (s0 - start_star)
            r_star = max(min(r_star, r0), r_min)
            # choose smallest g >= g0 that still satisfies end >= arrival_end
            g_req = max(g0, arrival_end - start_star)
            g_star = min(g_req, g0 + float(self.cfg.max_target_green_extension_sec))

            if (start_star > arrival_start + 1e-6) or (start_star + g_star < arrival_end - 1e-6):
                return None

        # Convert to actionable plan:
        # - hurry current phase to r_star if it is significantly shorter
        # - extension is applied when target becomes current (agent will replan; we include a hint)
        hurry = float(r_star) if float(r_star) < (r0 - 1e-3) else None
        # we cannot directly "set" future target duration, but we can report needed extension from base
        ext_needed = max(0.0, float(g_star) - float(g0))

        planned_start = s0 - (r0 - float(r_star))
        planned_end = planned_start + float(g_star)

        return PreemptionPlan(
            plan_type="non_intrusive",
            target_phase_idx=target_phase,
            extend_green_sec=float(ext_needed),
            hurry_current_phase_to_sec=hurry,
            planned_green_window=(float(planned_start), float(planned_end)),
            notes=f"QP-feasible: hurry_to={hurry}, extend_needed={ext_needed:.2f}, covers [{arrival_start:.2f},{arrival_end:.2f}] in window [{planned_start:.2f},{planned_end:.2f}]"
        )"""

    def _try_non_intrusive_qp(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        """
        Return a "non_intrusive" plan ONLY if QP is feasible.

        Reduced decision variables (TraCI-realistic):
          r  = remaining time of the CURRENT phase (can only be shortened)
          g  = effective duration of the TARGET phase occurrence (can be extended when it is active)

        Constraints:
          r_min <= r <= r0
          g0 <= g <= g0 + g_ext_cap
          (start_target_new <= arrival_start) AND (end_target_new >= arrival_end)

        Objective:
          minimize (r - r0)^2 + (g - g0)^2
        """
        if traci is None:
            self._b1_dbg("qp_reduced_return none reason=no_traci")
            return None
        if self.active_ev is None:
            self._b1_dbg("qp_reduced_return none reason=no_active_ev")
            return None

        tls_id = self.cfg.tls_id
        now = float(sim_time)

        target_phase = int(self.active_ev.target_phase_idx or 0)
        delta = float(self.active_ev.delta_sec)

        arrival_start, arrival_end = float(t_i) - delta, float(t_i) + delta
        self._b1_dbg(
            f"qp_reduced_enter sim={float(now):.2f} tls={tls_id} target={target_phase} "
            f"arr=({float(arrival_start):.2f},{float(arrival_end):.2f})"
        )

        # Base next target window (no intervention)
        print(f"Expected target phase: {target_phase} for node: {tls_id}")
        print(f"Arrival_start, arrival_end: t_i - delta: {t_i - delta}, t_i + delta: {t_i + delta}")

        base_win = self._predict_next_phase_window(now, target_phase)

        if base_win is None:
            self._b1_dbg("qp_reduced_return none reason=no_base_window")
            return None

        # Current phase state
        cur_phase = int(traci.trafficlight.getPhase(tls_id))
        next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        print(f" ----- Next_switch: {next_switch}")
        r0 = max(0.0, next_switch - now)
        r_min = float(self.cfg.min_current_phase_remaining_sec)
        self._b1_dbg(
            f"qp_reduced_phase_state cur_phase={cur_phase} target={target_phase} next_switch={float(next_switch):.2f} "
            f"r0={float(r0):.2f} r_min={float(r_min):.2f}"
        )

        # If target is current, start is now; g0 is remaining
        if cur_phase == target_phase:
            g0 = r0
            # We can only extend current green (g >= g0).
            # If the EV is already inside the nominal arrival window, still allow extension
            # to preserve the *remaining* service window instead of forcing a no-op fallback.
            feasible = (arrival_end <= now + (g0 + float(self.cfg.max_target_green_extension_sec)))
            print(f"Feasible time for extension: now:{now}, arrival_start: {arrival_start}, arrival_end: {arrival_end} , g0:{g0}, self.cfg.max_target_green_extension_sec: {self.cfg.max_target_green_extension_sec} ")
            if not feasible:
                self._b1_dbg(
                    f"qp_reduced_return none reason=current_target_not_feasible now={float(now):.2f} "
                    f"g0={float(g0):.2f} max_ext={float(self.cfg.max_target_green_extension_sec):.2f}"
                )
                return None
            
            print(f" ***** Phase matches target phase ****** ")

            need_ext = max(0.0, arrival_end - (now + g0))
            ext = min(max(need_ext, 0.0), float(self.cfg.max_target_green_extension_sec))

            print(f"need_ext: {need_ext}, ext:{ext}")

            print(f"now {now} + g0 {g0}+ ext ({ext})")

            plan = PreemptionPlan(
                plan_type="non_intrusive",
                target_phase_idx=target_phase,
                extend_green_sec=float(ext),
                planned_green_window=(now, now + g0 + float(ext)),
                notes=f"QP-feasible (target=current): extend by {ext:.2f}s to cover [{arrival_start:.2f},{arrival_end:.2f}]"
            )
            self._b1_dbg(
                f"qp_reduced_return plan_type={plan.plan_type} mode=current_target target={target_phase} "
                f"ext={float(ext):.2f} window=({float(plan.planned_green_window[0]):.2f},{float(plan.planned_green_window[1]):.2f})"
            )
            return plan

        # Target is in the future. The base window is (s0,e0) for the next occurrence.
        s0, e0 = float(base_win[0]), float(base_win[1])
        g0 = max(0.0, e0 - s0)

        # The only way to shift target earlier is to shorten the CURRENT phase (not intermediate phases).
        # shift_max = r0 - r_min (how much earlier we can make the next switch)
        shift_max = max(0.0, r0 - r_min)

        earliest_start = s0 - shift_max
        latest_end = e0 + float(self.cfg.max_target_green_extension_sec)
        self._b1_dbg(
            f"qp_reduced_future_base target={target_phase} base_win=({float(s0):.2f},{float(e0):.2f}) "
            f"g0={float(g0):.2f} shift_max={float(shift_max):.2f} earliest_start={float(earliest_start):.2f} "
            f"latest_end={float(latest_end):.2f}"
        )

        # Quick feasibility screening (equivalent to reduced-QP feasibility)
        if arrival_start < earliest_start - 1e-6:
            self._b1_dbg(
                f"qp_reduced_return none reason=arrival_too_early arr_start={float(arrival_start):.2f} "
                f"earliest_start={float(earliest_start):.2f}"
            )
            return None
        if arrival_end > latest_end + 1e-6:
            self._b1_dbg(
                f"qp_reduced_return none reason=arrival_too_late arr_end={float(arrival_end):.2f} "
                f"latest_end={float(latest_end):.2f}"
            )
            return None

        # Solve reduced QP (if cvxpy available), else closed-form
        r_star = None
        g_star = None

        if self.cfg.enable_non_intrusive_qp and cp is not None:
            r = cp.Variable()
            g = cp.Variable()
            # start/end as affine functions (only current phase shift affects start)
            start = s0 - (r0 - r)
            end = start + g

            constraints = [
                r >= r_min,
                r <= r0,
                g >= g0,
                g <= g0 + float(self.cfg.max_target_green_extension_sec),
                start <= arrival_start,
                end >= arrival_end,
            ]
            obj = cp.Minimize(cp.square(r - r0) + cp.square(g - g0))
            prob = cp.Problem(obj, constraints)
            try:
                prob.solve(solver=cp.ECOS, warm_start=True, verbose=False)
            except Exception:
                try:
                    prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
                except Exception:
                    prob.solve(warm_start=True, verbose=False)

            if prob.status not in ("optimal", "optimal_inaccurate"):
                self._b1_dbg(f"qp_reduced_return none reason=solver_status status={str(prob.status)}")
                return None

            r_star = float(r.value)
            g_star = float(g.value)
        else:
            # Closed-form for this reduced QP:
            # choose start as late as possible but <= arrival_start, within [earliest_start, s0]
            start_star = min(s0, arrival_start)
            start_star = max(start_star, earliest_start)
            # corresponding r to achieve that start: start = s0 - (r0 - r) -> r = r0 - (s0 - start)
            r_star = r0 - (s0 - start_star)
            r_star = max(min(r_star, r0), r_min)
            # choose smallest g >= g0 that still satisfies end >= arrival_end
            g_req = max(g0, arrival_end - start_star)
            g_star = min(g_req, g0 + float(self.cfg.max_target_green_extension_sec))

            if (start_star > arrival_start + 1e-6) or (start_star + g_star < arrival_end - 1e-6):
                self._b1_dbg(
                    f"qp_reduced_return none reason=closed_form_postcheck start_star={float(start_star):.2f} "
                    f"g_star={float(g_star):.2f} arr=({float(arrival_start):.2f},{float(arrival_end):.2f})"
                )
                return None

        # Convert to actionable plan:
        # - hurry current phase to r_star if it is significantly shorter
        # - extension is applied when target becomes current (agent will replan; we include a hint)
        hurry = float(r_star) if float(r_star) < (r0 - 1e-3) else None
        # we cannot directly "set" future target duration, but we can report needed extension from base
        ext_needed = max(0.0, float(g_star) - float(g0))

        planned_start = s0 - (r0 - float(r_star))
        planned_end = planned_start + float(g_star)

        plan = PreemptionPlan(
            plan_type="non_intrusive",
            target_phase_idx=target_phase,
            extend_green_sec=float(ext_needed),
            hurry_current_phase_to_sec=hurry,
            planned_green_window=(float(planned_start), float(planned_end)),
            notes=f"QP-feasible: hurry_to={hurry}, extend_needed={ext_needed:.2f}, covers [{arrival_start:.2f},{arrival_end:.2f}] in window [{planned_start:.2f},{planned_end:.2f}]"
        )
        self._b1_dbg(
            f"qp_reduced_return plan_type={plan.plan_type} mode=future_target target={target_phase} "
            f"hurry={hurry} ext_needed={float(ext_needed):.2f} "
            f"planned=({float(planned_start):.2f},{float(planned_end):.2f})"
        )
        return plan


    # =========================
    # Intrusive preemption
    # =========================
    '''
    def _intrusive_preemption(self, sim_time: float, t_i: float) -> PreemptionPlan:
        """
        Paper logic (names preserved):
          LJT = t_i - delta - SIT - YT - Q_i
          PST = start time of the *target phase occurrence that contains LJT* (or next after it)
          D   = LJT - PST
          JumpTime = PST if D < GminS else LJT
        """
        if self.active_ev is None:
            return PreemptionPlan(plan_type="intrusive", target_phase_idx=0, jump_time_sec=float(sim_time), notes="No active EV")

        target_phase = int(self.active_ev.target_phase_idx or 0)
        delta = float(self.active_ev.delta_sec)

        Q_i = float(self._estimate_queue_clear_time(self.active_ev.in_edge_id))

        LJT = float(t_i) - delta - float(self.cfg.SIT_sec) - float(self.cfg.YT_sec) - float(Q_i)

        # PST: start of the target phase occurrence that contains LJT (fix vs "next occurrence only")
        win = self._predict_phase_window_containing(float(sim_time), target_phase, float(LJT))
        PST = float(sim_time) if win is None else float(win[0])

        D = float(LJT) - float(PST)

        jump_time = float(PST) if float(D) < float(self.cfg.GminS_sec) else float(LJT)
        jump_time = max(float(jump_time), float(sim_time))

        return PreemptionPlan(
            plan_type="intrusive",
            target_phase_idx=target_phase,
            jump_time_sec=float(jump_time),
            notes=f"Intrusive: Q={Q_i:.2f}, LJT={LJT:.2f}, PST={PST:.2f}, D={D:.2f}, jump={jump_time:.2f}"
        )
    '''
    
    """
    def _intrusive_preemption(self, sim_time: float, t_i: float) -> PreemptionPlan:
        '''
        Paper logic (names preserved):
          LJT = t_i - delta - SIT - YT - Q_i
          PST = start time of the *target phase occurrence that contains LJT* (or next after it)
          D   = LJT - PST
          JumpTime = PST if D < GminS else LJT
        '''
        if self.active_ev is None:
            return PreemptionPlan(plan_type="intrusive", target_phase_idx=0, jump_time_sec=float(sim_time), notes="No active EV")

        target_phase = int(self.active_ev.target_phase_idx or 0)
        delta = float(self.active_ev.delta_sec)

        Q_i = float(self._estimate_queue_clear_time(self.active_ev.in_edge_id))

        LJT = float(t_i) - delta - float(self.cfg.SIT_sec) - float(self.cfg.YT_sec) - float(Q_i)

        # PST: start of the target phase occurrence that contains LJT (fix vs "next occurrence only")
        win = self._predict_phase_window_containing(float(sim_time), target_phase, float(LJT))
        PST = float(sim_time) if win is None else float(win[0])

        D = float(LJT) - float(PST)

        jump_time = float(PST) if float(D) < float(self.cfg.GminS_sec) else float(LJT)
        jump_time = max(float(jump_time), float(sim_time))

        return PreemptionPlan(
            plan_type="intrusive",
            target_phase_idx=target_phase,
            jump_time_sec=float(jump_time),
            notes=f"Intrusive: Q={Q_i:.2f}, LJT={LJT:.2f}, PST={PST:.2f}, D={D:.2f}, jump={jump_time:.2f}"
        )"""
    
    def _intrusive_preemption(self, sim_time: float, t_i: float) -> Optional[PreemptionPlan]:
        """Paper-style intrusive fallback (Eq. style: LJT/PST/D) with hard guards."""
        ev = self.active_ev
        if ev is None:
            self._b1_dbg("intrusive_return none reason=no_active_ev")
            return None

        target_phase = int(ev.target_phase_idx or 0)
        now = float(sim_time)
        arrival_start = float(t_i) - float(ev.delta_sec)
        arrival_end = float(t_i) + float(ev.delta_sec)

        # Window already over -> no intrusive.
        if arrival_end <= now + 0.05:
            self._b1_dbg(
                f"intrusive_return none reason=arrival_window_over now={float(now):.2f} arrival_end={float(arrival_end):.2f}"
            )
            return None

        try:
            current_phase = int(traci.trafficlight.getPhase(self.cfg.tls_id))
            next_switch = float(traci.trafficlight.getNextSwitch(self.cfg.tls_id))
        except Exception:
            current_phase = target_phase
            next_switch = now

        # HARD GUARD 1: if current target phase can serve with <= max extension, do not jump.
        if current_phase == target_phase:
            g0 = max(0.0, next_switch - now)
            need_ext = max(0.0, arrival_end - (now + g0))
            if need_ext <= float(self.cfg.max_target_green_extension_sec):
                self._b1_dbg(
                    f"intrusive_return none reason=target_green_extendable cur_phase={current_phase} target={target_phase} "
                    f"g0={float(g0):.2f} need_ext={float(need_ext):.2f} max_ext={float(self.cfg.max_target_green_extension_sec):.2f}"
                )
                return None

        # HARD GUARD 2: do not jump too early when EV is far.
        dist = float(getattr(ev, "distance_to_intersection_m", 1e9) or 1e9)
        ev_speed = float(getattr(ev, "speed_mps", 0.0) or 0.0)
        near_override_dist = float(getattr(self.cfg, "intrusive_near_override_dist_m", 12.0))
        near_override_speed = float(getattr(self.cfg, "intrusive_near_override_speed_mps", 0.8))
        force_near_override = (dist <= near_override_dist) or (ev_speed <= near_override_speed)
        dist_guard = float(getattr(self.cfg, "intrusive_distance_guard_m", 25.0))
        if dist > dist_guard and not force_near_override:
            self._b1_dbg(
                f"intrusive_return none reason=distance_guard dist={float(dist):.2f} "
                f"guard={float(dist_guard):.2f}"
            )
            return None
        if dist > dist_guard and force_near_override:
            self._b1_dbg(
                f"intrusive_guard_bypass type=distance_guard dist={float(dist):.2f} guard={float(dist_guard):.2f} "
                f"ev_speed={float(ev_speed):.2f} near_override_dist={float(near_override_dist):.2f} "
                f"near_override_speed={float(near_override_speed):.2f}"
            )

        # Paper variables:
        # LJT = t_i - delta - SIT - YT - Q_i
        # PST = start of target phase containing LJT (or next after LJT)
        # D = LJT - PST
        prog = self._get_current_program_logic()
        if prog is not None and getattr(prog, "phases", None):
            dur0 = [float(getattr(ph, "duration", 0.0)) for ph in prog.phases]
            L_t, _next_cycle_start_time = self._time_to_next_cycle_start(float(sim_time), dur0, cycle_start_phase=0)
        else:
            L_t = 0.0
        R = float(t_i) - float(sim_time) - float(L_t)
        Q_i = float(self._estimate_queue_clear_time(str(ev.in_edge_id)))
        LJT = float(t_i) - float(ev.delta_sec) - float(self.cfg.SIT_sec) - float(self.cfg.YT_sec) - float(Q_i)
        win = self._predict_phase_window_containing(float(sim_time), target_phase, float(LJT))
        if win is None:
            win = self._predict_next_phase_window(float(sim_time), target_phase)
        PST = float(win[0]) if win is not None else float(now)
        D = float(LJT) - float(PST)

        # HARD GUARD 3: only jump if disturbance still meaningful.
        disturbance_min = float(getattr(self.cfg, "intrusive_disturbance_min", 0.0))
        if D < disturbance_min and not force_near_override:
            self._b1_dbg(
                f"intrusive_return none reason=disturbance_guard D={float(D):.2f} "
                f"min={float(disturbance_min):.2f}"
            )
            return None
        if D < disturbance_min and force_near_override:
            self._b1_dbg(
                f"intrusive_guard_bypass type=disturbance_guard D={float(D):.2f} min={float(disturbance_min):.2f} "
                f"dist={float(dist):.2f} ev_speed={float(ev_speed):.2f}"
            )

        # Paper jump selection:
        #   jump = PST if D < GminS else LJT
        jump_time = float(PST) if float(D) < float(self.cfg.GminS_sec) else float(LJT)
        jump_time = max(float(now), float(jump_time))
        if force_near_override and current_phase != target_phase:
            # Safety override: when the EV is near/stopped at the stopline, do not keep
            # postponing intrusive actuation to a moving future PST. Clamp to "now".
            self._b1_dbg(
                f"intrusive_near_override_jump_now now={float(now):.2f} prior_jump_t={float(jump_time):.2f} "
                f"cur_phase={int(current_phase)} target={int(target_phase)} dist={float(dist):.2f} ev_speed={float(ev_speed):.2f}"
            )
            jump_time = float(now)
        self._print_compact_decision_debug(
            sim_time=float(sim_time),
            mode="intrusive",
            L_t=float(L_t),
            R=float(R),
            Q_i=float(Q_i),
            LJT=float(LJT),
            PST=float(PST),
            D=float(D),
            jump_time=float(jump_time),
        )
        plan = PreemptionPlan(
            plan_type="intrusive",
            target_phase_idx=target_phase,
            jump_to_phase_idx=target_phase,
            jump_time_sec=float(jump_time),
            notes=f"Intrusive: Q_i={Q_i:.2f}, LJT={LJT:.2f}, PST={PST:.2f}, D={D:.2f}, jump={jump_time:.2f}",
        )
        self._b1_dbg(
            f"intrusive_return plan_type={plan.plan_type} target={target_phase} cur_phase={current_phase} "
            f"dist={float(dist):.2f} Q_i={float(Q_i):.2f} LJT={float(LJT):.2f} PST={float(PST):.2f} D={float(D):.2f} "
            f"jump_t={float(jump_time):.2f}"
        )
        return plan
    
    '''
    def _estimate_queue_clear_time(self, edge_id: str) -> float:
        """
        Q_i = N / (S - A)
          N = edge halting vehicles
          A = arrival rate EMA (veh/s)
          S = saturation flow (veh/s) ~ lanes * saturation_flow_per_lane_vehps
        """
        if traci is None or edge_id.startswith(":"):
            return 0.0

        try:
            N = float(traci.edge.getLastStepHaltingNumber(edge_id))
        except Exception:
            N = 0.0

        A = float(self.arrivals.update(edge_id))

        try:
            lanes = max(int(traci.edge.getLaneNumber(edge_id)), 1)
        except Exception:
            lanes = 1

        S = float(self.cfg.saturation_flow_per_lane_vehps) * float(lanes)

        denom = S - A
        if denom <= 0.05:
            return 9999.0
        return float(N / denom)
    '''
    
    def _estimate_queue_clear_time(self, edge_id: str) -> float:
        """Queue clearing time Q_i for the paper option-2 model.

        We use the same lane-based calculation as in the paper-QP:
          Q_i = N / (S - A)

        Option A:
          - if the EV is on this approach edge, use the EV's current approach lane only
          - otherwise sum across all lanes on the edge

        Returns:
          Q_i in seconds (capped by cfg.queue_clear_time_cap_sec when effectively unstable)
        """
        if traci is None or (not edge_id) or edge_id.startswith(":"):
            return 0.0

        ev_id = None
        if self.active_ev is not None and str(self.active_ev.in_edge_id) == str(edge_id):
            ev_id = str(self.active_ev.ev_id)

        _, _, _, Q, _ = self._queue_clearing_metrics_for_edge(edge_id=str(edge_id), ev_id=ev_id)
        return float(Q)

    # =========================
    # Restoration (optional LP)
    # =========================

    def _compute_restoration_schedule_lp(self) -> Optional[Dict[int, float]]:
        """
        Optional: compute a one-cycle schedule that compensates accumulated timing offset.

        This is a *middleware-demo-friendly* approximation: it distributes the accumulated offset across
        "adjustable" phases while keeping yellow/all-red phases fixed.

        - If cvxpy is available, solve an LP minimizing L1 deviation from defaults.
        - Else, use a greedy redistribution.
        """
        if traci is None:
            return None

        durations0 = self._get_program_phase_durations()
        if not durations0:
            return None

        n = len(durations0)
        offset = float(self._timing_offset_sec)
        if abs(offset) < 1e-3:
            return None

        # Identify adjustable phases: those with any green in state string (heuristic)
        # If we can't access states, treat all as adjustable (except keep min >= tau_min).
        adjustable = [True] * n
        try:
            logics = traci.trafficlight.getAllProgramLogics(self.cfg.tls_id)
            cur_prog = str(traci.trafficlight.getProgram(self.cfg.tls_id))
            logic = None
            for lg in logics:
                if getattr(lg, "programID", None) == cur_prog:
                    logic = lg
                    break
            if logic is None and logics:
                logic = logics[0]
            if logic is not None:
                for i, ph in enumerate(list(logic.phases)):
                    st = getattr(ph, "state", "")
                    # Consider yellow/all-red fixed (no 'G'/'g')
                    adjustable[i] = any(c in ("G", "g") for c in st)
        except Exception:
            pass

        # Bounds
        dmin = []
        dmax = []
        for i in range(n):
            if adjustable[i]:
                dmin.append(max(float(self.cfg.tau_min_sec), 0.5))
                dmax.append(float(self.cfg.tau_max_sec))
            else:
                dmin.append(float(durations0[i]))
                dmax.append(float(durations0[i]))

        # Target cycle length change: to compensate offset we want sum(d) = sum(d0) - offset
        # (if offset>0 we previously made it longer; now shorten by offset)
        target_sum = float(sum(durations0)) - offset

        # If impossible due to bounds, clamp target_sum
        target_sum = max(target_sum, sum(dmin) + 1e-6)
        target_sum = min(target_sum, sum(dmax) - 1e-6)

        if cp is not None:
            d = cp.Variable(n)
            u = cp.Variable(n)  # abs deviation
            constraints = [
                d >= dmin,
                d <= dmax,
                cp.sum(d) == target_sum,
                u >= d - durations0,
                u >= -(d - durations0),
                u >= 0,
            ]
            obj = cp.Minimize(cp.sum(u))
            prob = cp.Problem(obj, constraints)
            try:
                prob.solve(solver=cp.ECOS, warm_start=True, verbose=False)
            except Exception:
                prob.solve(warm_start=True, verbose=False)

            if prob.status not in ("optimal", "optimal_inaccurate"):
                return None

            vals = [float(x) for x in d.value]
        else:
            # Greedy: distribute required delta across adjustable phases proportional to slack.
            vals = [float(x) for x in durations0]
            delta_total = target_sum - sum(vals)
            # If delta_total negative => need shorten; positive => lengthen
            phases = [i for i in range(n) if adjustable[i]]
            if not phases:
                return None
            # Sort by slack
            if delta_total < 0:
                # shorten where we have room above dmin
                phases.sort(key=lambda i: (vals[i] - dmin[i]), reverse=True)
                remain = -delta_total
                for i in phases:
                    slack = vals[i] - dmin[i]
                    take = min(slack, remain)
                    vals[i] -= take
                    remain -= take
                    if remain <= 1e-6:
                        break
            else:
                # lengthen where we have room below dmax
                phases.sort(key=lambda i: (dmax[i] - vals[i]), reverse=True)
                remain = delta_total
                for i in phases:
                    slack = dmax[i] - vals[i]
                    take = min(slack, remain)
                    vals[i] += take
                    remain -= take
                    if remain <= 1e-6:
                        break

        return {i: float(vals[i]) for i in range(n)}

    # =========================
    #    Actuation (TraCI)
    # =========================

    def apply_plan_to_tls(self, sim_time: float, plan: PreemptionPlan, decision_source: str = "plan") -> None:
        """Call this from your SUMO loop after tick() returns a plan."""
        t0_apply = time.perf_counter()
        now = float(sim_time)
        src = str(decision_source or "")
        self._session_event_counts["apply_plan"] += 1
        self._session_event_counts[f"apply_plan_source:{src or 'unknown'}"] += 1

        if bool(getattr(self.cfg, "f2_skip_redundant_apply", True)):
            noisy_sources = {"offer", "f2_local_fallback", "f2_selected_none", "federation_warmup"}
            if src in noisy_sources:
                plan_sig = (
                    src,
                    str(self.stage),
                    str(getattr(plan, "plan_type", "") or ""),
                    int(getattr(plan, "target_phase_idx", -1) if getattr(plan, "target_phase_idx", None) is not None else -1),
                    round(float(getattr(plan, "extend_green_sec", 0.0) or 0.0), 3),
                    round(float(getattr(plan, "hurry_current_phase_to_sec", -1.0) if getattr(plan, "hurry_current_phase_to_sec", None) is not None else -1.0), 3),
                    round(float(getattr(plan, "jump_time_sec", -1.0) if getattr(plan, "jump_time_sec", None) is not None else -1.0), 3),
                    int(getattr(plan, "jump_to_phase_idx", -1) if getattr(plan, "jump_to_phase_idx", None) is not None else -1),
                )
                ev_id_now = str(self.active_ev.ev_id) if self.active_ev is not None else ""
                ev_dist_m = float(self._active_ev_distance_m(ev_id_now))
                min_dt_base = max(
                    0.0,
                    float(getattr(self.cfg, "f2_skip_redundant_apply_min_interval_sec", 0.8)),
                )
                min_dt_near = max(
                    0.0,
                    float(
                        getattr(
                            self.cfg,
                            "f2_skip_redundant_apply_min_interval_near_sec",
                            min_dt_base,
                        )
                    ),
                )
                min_dt_far = max(
                    0.0,
                    float(
                        getattr(
                            self.cfg,
                            "f2_skip_redundant_apply_min_interval_far_sec",
                            min_dt_base,
                        )
                    ),
                )
                near_m = max(
                    0.0,
                    float(getattr(self.cfg, "f2_skip_redundant_apply_near_distance_m", 120.0)),
                )
                far_m = max(
                    near_m,
                    float(getattr(self.cfg, "f2_skip_redundant_apply_far_distance_m", 300.0)),
                )
                min_dt = self._distance_adaptive_value(
                    base=min_dt_base,
                    near_value=min_dt_near,
                    far_value=min_dt_far,
                    ev_distance_m=float(ev_dist_m),
                    near_distance_m=float(near_m),
                    far_distance_m=float(far_m),
                )
                coord_active = bool(self._coordination_window_active(ev_id_now, now_sim=now))
                if coord_active:
                    scale = max(0.1, min(1.0, float(getattr(self.cfg, "f2_active_coord_window_interval_scale", 0.50))))
                    min_dt *= float(scale)
                if self._last_apply_signature == plan_sig and (now - float(self._last_apply_sim_time)) < min_dt:
                    self._session_event_counts["plan_skip"] += 1
                    self._session_reason_counts["plan_skip:redundant_reapply"] += 1
                    self._fed_evt(
                        "coord.apply.plan_skip",
                        sim_time=now,
                        decision_source=src,
                        stage=str(self.stage),
                        reason="redundant_reapply",
                        ev_distance_m=float(ev_dist_m),
                        coordination_active=int(1 if coord_active else 0),
                        dt_since_last_s=float(now - float(self._last_apply_sim_time)),
                        min_interval_s=float(min_dt),
                        min_interval_near_s=float(min_dt_near),
                        min_interval_far_s=float(min_dt_far),
                        min_interval_near_m=float(near_m),
                        min_interval_far_m=float(far_m),
                        ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                    )
                    return

        def _emit_apply_compute_evt() -> None:
            dt_ms = (time.perf_counter() - t0_apply) * 1000.0
            self._last_apply_compute_ms = float(max(0.0, dt_ms))
            self._fed_evt(
                "intersection.compute.apply.duration_ms",
                sim_time=float(sim_time),
                ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                duration_ms=float(dt_ms),
            )

        self._log_decision_event(
            event="apply_plan_to_tls",
            sim_time=float(sim_time),
            decision_source=str(decision_source),
            plan=plan,
            offer=None,
            note="invoked",
        )
        self._plan_selected_dbg(
            sim_time=float(sim_time),
            decision_source=str(decision_source),
            plan=plan,
            offer=None,
            note="apply_plan_to_tls",
        )
        self._fed_evt(
            "coord.apply.plan",
            sim_time=now,
            decision_source=str(decision_source),
            stage=str(self.stage),
            plan_type=str(getattr(plan, "plan_type", "") or ""),
            target_phase=(int(plan.target_phase_idx) if getattr(plan, "target_phase_idx", None) is not None else -1),
            extend_sec=float(getattr(plan, "extend_green_sec", 0.0) or 0.0),
            hurry_to_sec=(float(plan.hurry_current_phase_to_sec) if getattr(plan, "hurry_current_phase_to_sec", None) is not None else -1.0),
            jump_time_sec=(float(plan.jump_time_sec) if getattr(plan, "jump_time_sec", None) is not None else -1.0),
            jump_to_phase=(int(plan.jump_to_phase_idx) if getattr(plan, "jump_to_phase_idx", None) is not None else -1),
            ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
        )
        self._last_apply_signature = (
            str(decision_source or ""),
            str(self.stage),
            str(getattr(plan, "plan_type", "") or ""),
            int(getattr(plan, "target_phase_idx", -1) if getattr(plan, "target_phase_idx", None) is not None else -1),
            round(float(getattr(plan, "extend_green_sec", 0.0) or 0.0), 3),
            round(float(getattr(plan, "hurry_current_phase_to_sec", -1.0) if getattr(plan, "hurry_current_phase_to_sec", None) is not None else -1.0), 3),
            round(float(getattr(plan, "jump_time_sec", -1.0) if getattr(plan, "jump_time_sec", None) is not None else -1.0), 3),
            int(getattr(plan, "jump_to_phase_idx", -1) if getattr(plan, "jump_to_phase_idx", None) is not None else -1),
        )
        self._last_apply_sim_time = now
        self._last_apply_source = str(decision_source or "")

        if traci is None:
            _emit_apply_compute_evt()
            return

        tls_id = self.cfg.tls_id
        ev_ctx = self.active_ev
        if ev_ctx is None and now <= float(self._federation_warmup_valid_until):
            ev_ctx = self._federation_warmup_ev

        # Detect phase changes (used for restoration schedule application)
        try:
            cur_phase = int(traci.trafficlight.getPhase(tls_id))
        except Exception:
            cur_phase = None
        if cur_phase is not None and cur_phase != self._last_seen_phase:
            self._last_seen_phase = cur_phase
        self._b1_dbg(
            f"apply_enter sim={now:.2f} source={decision_source} plan_type={getattr(plan, 'plan_type', None)} "
            f"target={getattr(plan, 'target_phase_idx', None)} cur_phase={cur_phase} "
            f"ev_target={None if ev_ctx is None else getattr(ev_ctx, 'target_phase_idx', None)} "
            f"ev_dist={('NA' if ev_ctx is None else f'{float(getattr(ev_ctx, 'distance_to_intersection_m', 0.0)):.2f}')}"
        )

        # Helper to measure remaining before changing duration
        def _remaining() -> float:
            try:
                nxt = float(traci.trafficlight.getNextSwitch(tls_id))
                print(f"nxt switch {nxt}")
                return max(0.0, nxt - now)
            except Exception:
                self._b1_dbg("apply_remaining_error")
                return 0.0
        
        def _maybe_extend_target_green(max_allowed_ext: float) -> None:

            """Extend only as much as needed to cover EV arrival_end, capped by max_allowed_ext."""
            if ev_ctx is None or cur_phase is None:
                print("active ev or cur phase is none")
                self._b1_dbg(
                    f"extend_skip reason=no_ev_or_phase cur_phase={cur_phase} "
                    f"ev_present={0 if ev_ctx is None else 1}"
                )
                return
            
            # Only extend when we are currently serving the EV's target green.
            if cur_phase != int(ev_ctx.target_phase_idx or 0):
                self._b1_dbg(
                    f"extend_skip reason=not_in_target_green cur_phase={cur_phase} "
                    f"target={int(ev_ctx.target_phase_idx or 0)}"
                )
                return

            # Cooldown to avoid rapid repeat actuation (optional but helpful)
            if (now - float(self._sat_last_actuation_time)) < float(getattr(self.cfg, "saturation_actuation_cooldown_sec", 1.0)):
                print("Preventing rapid actuation")
                self._b1_dbg(
                    f"extend_skip reason=cooldown dt={float(now - float(self._sat_last_actuation_time)):.2f} "
                    f"cooldown={float(getattr(self.cfg, 'saturation_actuation_cooldown_sec', 1.0)):.2f}"
                )
                return

            rem0 = _remaining()
            if rem0 <= 0.01:
                print("rem0 <= 0.01")
                self._b1_dbg(f"extend_skip reason=remaining_too_small rem0={float(rem0):.3f}")
                return
            
            # Commit guard: defer early extension when EV is still far and phase is not ending soon.
            dist_m = float(
                getattr(
                    ev_ctx,
                    "distance_to_intersection_m",
                    getattr(ev_ctx, "distance_to_stopline_m", 1e9),
                )
            )
            if (
                rem0 > float(getattr(self.cfg, "extension_commit_horizon_sec", 8.0))
                and dist_m > float(getattr(self.cfg, "extension_commit_distance_m", 50.0))
            ):
                print(f"defer extension: rem0={rem0:.2f}s, dist={dist_m:.1f}m")
                self._b1_dbg(
                    f"extend_skip reason=commit_guard rem0={float(rem0):.2f} dist={float(dist_m):.2f} "
                    f"horizon={float(getattr(self.cfg, 'extension_commit_horizon_sec', 8.0)):.2f} "
                    f"dist_thr={float(getattr(self.cfg, 'extension_commit_distance_m', 50.0)):.2f}"
                )
                return

            # recompute arrival window *now* (ETA changes as EV moves)
            t_i = self._estimate_arrival_time(now, ev_ctx)
            delta = float(ev_ctx.delta_sec)
            arrival_end = float(t_i) + delta

            buffer_sec = float(getattr(self.cfg, "saturation_green_buffer_sec", 1.0))
            needed = (arrival_end + buffer_sec) - (now + rem0)

            if needed <= float(getattr(self.cfg, "saturation_min_gap_to_act_sec", 0.5)):
                print("too tiny to matter")
                self._b1_dbg(
                    f"extend_skip reason=needed_too_small needed={float(needed):.3f} "
                    f"min_gap={float(getattr(self.cfg, 'saturation_min_gap_to_act_sec', 0.5)):.3f}"
                )
                return  # already covered (or too tiny to matter)

            ext_apply = min(float(needed), float(max_allowed_ext), float(self.cfg.max_target_green_extension_sec))
            print(f"Extra time to extend: {ext_apply}")

            if ext_apply <= 0.0:
                print("ext_apply<=0")
                self._b1_dbg(
                    f"extend_skip reason=ext_nonpos ext_apply={float(ext_apply):.3f} "
                    f"needed={float(needed):.3f} max_allowed={float(max_allowed_ext):.3f}"
                )
                return

            new_rem = rem0 + ext_apply
            new_rem = min(new_rem, float(self.cfg.tau_max_sec))

            print(f"\nNew remaining: {new_rem}, rem0 {rem0} + ext_apply {ext_apply}, tau_max_sec: {self.cfg.tau_max_sec} \n")
            print(f"+++   Attempting to update extend TLS due to {plan.plan_type} to {new_rem} seconds")
            self._b1_dbg(
                f"extend_apply plan_type={getattr(plan,'plan_type',None)} rem0={float(rem0):.2f} "
                f"needed={float(needed):.2f} ext_apply={float(ext_apply):.2f} new_rem={float(new_rem):.2f} "
                f"arrival_end={float(arrival_end):.2f} sleep_sec=1.0"
            )
            traci.trafficlight.setPhaseDuration(tls_id, float(new_rem))
            self._timing_offset_sec += float(new_rem - rem0)
            self._sat_last_actuation_time = now

            print(f"[{tls_id}] EXTEND: rem0={rem0:.2f} needed={needed:.2f} apply={ext_apply:.2f} new_rem={new_rem:.2f} " f"(arrival_end={arrival_end:.2f}, now={now:.2f})")


        # -------------------
        # Saturation reduction
        # -------------------

        if plan.plan_type == "saturation_reduction":
            print("I am here trying to apply saturation reduction plan")
            self._b1_dbg(
                f"apply_branch saturation_reduction ext={float(plan.extend_green_sec or 0.0):.2f} "
                f"cur_phase={cur_phase}"
            )
            if plan.extend_green_sec > 0:
                print(f"Evaluating actuation: plan.extend_green_sec{plan.extend_green_sec}")
                # treat DRRS extension as a MAX allowance, not “add every tick”
                _maybe_extend_target_green(max_allowed_ext=float(plan.extend_green_sec))
            else:
                self._b1_dbg("apply_noop reason=saturation_reduction_zero_ext")
            _emit_apply_compute_evt()
            return
        

        if plan.plan_type == "non_intrusive":
            self._b1_dbg(
                f"apply_branch non_intrusive hurry={plan.hurry_current_phase_to_sec} "
                f"ext={float(plan.extend_green_sec or 0.0):.2f} cur_phase={cur_phase}"
            )
            # Hurry current phase (if requested)
            if plan.hurry_current_phase_to_sec is not None:
                rem0 = _remaining()
                new_rem = max(float(plan.hurry_current_phase_to_sec), float(self.cfg.min_current_phase_remaining_sec))
                new_rem = min(new_rem, rem0)  # only shorten
                if new_rem < rem0 - 1e-3:
                    print(f"+++   Attempting to update extend TLS due to {plan.plan_type} to {new_rem} seconds")
                    self._b1_dbg(
                        f"hurry_apply rem0={float(rem0):.2f} new_rem={float(new_rem):.2f} sleep_sec=1.0"
                    )
                    traci.trafficlight.setPhaseDuration(tls_id, float(new_rem))
                    self._timing_offset_sec += float(new_rem - rem0)
                else:
                    self._b1_dbg(
                        f"hurry_skip reason=not_shorter rem0={float(rem0):.2f} candidate={float(new_rem):.2f}"
                    )
            #'''
            # Extend target green only as needed (same idea)
            if plan.extend_green_sec > 0:
                _maybe_extend_target_green(max_allowed_ext=float(plan.extend_green_sec))
            elif plan.hurry_current_phase_to_sec is None:
                self._b1_dbg("apply_noop reason=non_intrusive_no_hurry_no_ext")
            _emit_apply_compute_evt()
            return
            #'''
        
        '''
            # Extend green if (and only if) currently in target phase
            if cur_phase == plan.target_phase_idx and plan.extend_green_sec > 0:
                rem0 = _remaining()
                new_rem = rem0 + float(plan.extend_green_sec)
                traci.trafficlight.setPhaseDuration(tls_id, float(new_rem))
                self._timing_offset_sec += float(new_rem - rem0)
            return
        '''

        if plan.plan_type == "intrusive":
            ev_intr = self.active_ev if self.active_ev is not None else ev_ctx
            if ev_intr is None:
                self._b1_dbg("apply_skip reason=intrusive_no_ev_ctx")
                _emit_apply_compute_evt()
                return
            jump_phase = int(plan.jump_to_phase_idx if plan.jump_to_phase_idx is not None else plan.target_phase_idx)
            near_override_dist = float(getattr(self.cfg, "intrusive_near_override_dist_m", 12.0))
            near_override_speed = float(getattr(self.cfg, "intrusive_near_override_speed_mps", 0.8))
            ev_dist_intr = float(
                getattr(
                    ev_intr,
                    "distance_to_intersection_m",
                    getattr(ev_intr, "distance_to_stopline_m", 1e9),
                ) or 1e9
            )
            ev_speed_intr = float(getattr(ev_intr, "speed_mps", 0.0) or 0.0)
            force_near_apply = (ev_dist_intr <= near_override_dist) or (ev_speed_intr <= near_override_speed)
            jump_due = (plan.jump_time_sec is None) or (now >= float(plan.jump_time_sec))
            if (not jump_due) and force_near_apply and cur_phase != jump_phase:
                self._b1_dbg(
                    f"intrusive_apply_bypass reason=jump_time_not_reached_near_override now={float(now):.2f} "
                    f"jump_time={getattr(plan,'jump_time_sec',None)} cur_phase={int(cur_phase)} jump_phase={int(jump_phase)} "
                    f"dist={float(ev_dist_intr):.2f} ev_speed={float(ev_speed_intr):.2f}"
                )
                jump_due = True
            if jump_due:
                # Jump immediately to target phase
                jump_time_dbg = None if plan.jump_time_sec is None else float(plan.jump_time_sec)
                self._b1_dbg(
                    f"intrusive_jump_apply jump_phase={int(jump_phase)} now={float(now):.2f} "
                    f"jump_time={jump_time_dbg}"
                )
                traci.trafficlight.setPhase(tls_id, jump_phase)
                self._trace_tls_signal_change(float(now), reason="apply_intrusive_jump", force=True)

                # Hold green long enough for EV arrival window + clearance buffers
                Q_i = self._estimate_queue_clear_time(str(ev_intr.in_edge_id))
                q_hold = min(
                    float(Q_i),
                    max(0.0, float(getattr(self.cfg, "intrusive_hold_queue_cap_sec", 12.0))),
                )
                hold_raw = max(
                    10.0,
                    2.0 * float(ev_intr.delta_sec) + float(self.cfg.SIT_sec) + float(self.cfg.YT_sec) + float(q_hold),
                )
                hold = min(
                    float(hold_raw),
                    max(10.0, float(getattr(self.cfg, "intrusive_hold_max_sec", 30.0))),
                )
                # Since jump disrupts multiple phases, offset accounting is approximate.
                print(f"+++   Attempting to update extend TLS due to {plan.plan_type} to hold for {hold} seconds")
                self._b1_dbg(
                    f"intrusive_hold_apply hold={float(hold):.2f} raw_q_i={float(Q_i):.2f} "
                    f"q_i_capped={float(q_hold):.2f} hold_raw={float(hold_raw):.2f} "
                    f"hold_max={float(getattr(self.cfg, 'intrusive_hold_max_sec', 30.0)):.2f} sleep_sec=1.0"
                )
                self._fed_evt(
                    "coord.apply.intrusive_hold",
                    sim_time=now,
                    decision_source=str(decision_source),
                    stage=str(self.stage),
                    raw_q_i_sec=float(Q_i),
                    capped_q_i_sec=float(q_hold),
                    hold_raw_sec=float(hold_raw),
                    hold_applied_sec=float(hold),
                    hold_max_sec=float(getattr(self.cfg, "intrusive_hold_max_sec", 30.0)),
                    ev_distance_m=float(ev_dist_intr),
                    ev_speed_mps=float(ev_speed_intr),
                    ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                )
                traci.trafficlight.setPhaseDuration(tls_id, float(hold))
            else:
                self._b1_dbg(
                    f"intrusive_skip reason=jump_time_not_reached now={float(now):.2f} "
                    f"jump_time={getattr(plan,'jump_time_sec',None)}"
                )
            _emit_apply_compute_evt()
            return

        if plan.plan_type == "restore":
            # Restore default program
            pid = self.default_program_id if self.default_program_id is not None else "0"
            try:
                current_program_id = str(traci.trafficlight.getProgram(tls_id))
            except Exception:
                current_program_id = ""
            needs_program_restore = str(current_program_id) != str(pid)
            needs_phase_restore = bool(
                self._restoration_schedule is not None
                and cur_phase is not None
                and cur_phase in self._restoration_schedule
                and cur_phase not in self._restoration_applied_phases
            )
            if (
                self._restore_program_applied_for_session
                and (not needs_program_restore)
                and (not needs_phase_restore)
            ):
                self._session_event_counts["restore_skip"] += 1
                self._session_reason_counts["restore_skip:already_default_program"] += 1
                self._b1_dbg(
                    f"restore_skip reason=already_default_program program={pid} cur_phase={cur_phase} "
                    f"source={src}"
                )
                self._fed_evt(
                    "coord.apply.restore.skip",
                    sim_time=now,
                    decision_source=str(decision_source),
                    stage=str(self.stage),
                    reason="already_default_program",
                    current_program_id=str(current_program_id),
                    default_program_id=str(pid),
                    cur_phase=(int(cur_phase) if cur_phase is not None else -1),
                    ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                )
                _emit_apply_compute_evt()
                return

            if needs_program_restore or (not self._restore_program_applied_for_session):
                apply_reason = "program_changed" if needs_program_restore else "first_restore_in_session"
                self._b1_dbg(
                    f"restore_apply program={pid} cur_phase={cur_phase} source={src} "
                    f"reason={apply_reason}"
                )
                traci.trafficlight.setProgram(tls_id, str(pid))
                self._trace_tls_signal_change(float(now), reason="apply_restore_program", force=True)
                self._restore_program_applied_for_session = True
                if bool(getattr(self.cfg, "restore_remaining_clamp_enable", True)):
                    try:
                        phase_after = int(traci.trafficlight.getPhase(tls_id))
                    except Exception:
                        phase_after = cur_phase
                    try:
                        rem_after = max(0.0, float(traci.trafficlight.getNextSwitch(tls_id)) - float(now))
                    except Exception:
                        rem_after = -1.0
                    durations = self._get_program_phase_durations()
                    default_dur = None
                    if phase_after is not None and 0 <= int(phase_after) < len(durations):
                        default_dur = float(durations[int(phase_after)])
                    if default_dur is not None and rem_after > (default_dur + float(getattr(self.cfg, "restore_remaining_extra_sec", 2.0))):
                        clamp_to = max(
                            float(getattr(self.cfg, "min_current_phase_remaining_sec", 0.5)),
                            float(default_dur),
                        )
                        self._b1_dbg(
                            f"restore_remaining_clamp rem_before={float(rem_after):.2f} "
                            f"default_phase_dur={float(default_dur):.2f} clamp_to={float(clamp_to):.2f} "
                            f"phase={phase_after} source={src}"
                        )
                        traci.trafficlight.setPhaseDuration(tls_id, float(clamp_to))
                        self._trace_tls_signal_change(float(now), reason="apply_restore_remaining_clamp", force=True)
                        self._fed_evt(
                            "coord.apply.restore_clamp",
                            sim_time=now,
                            decision_source=str(decision_source),
                            stage=str(self.stage),
                            reason="remaining_exceeds_default_duration",
                            phase=(int(phase_after) if phase_after is not None else -1),
                            remaining_before_sec=float(rem_after),
                            default_phase_duration_sec=float(default_dur),
                            remaining_after_sec=float(clamp_to),
                            ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                        )
            else:
                self._b1_dbg(
                    f"restore_skip reason=program_already_default_schedule_pending program={pid} "
                    f"cur_phase={cur_phase} source={src}"
                )

            # If restoration schedule exists, apply when each phase becomes current
            if self._restoration_schedule is not None and cur_phase is not None:
                if cur_phase in self._restoration_schedule and cur_phase not in self._restoration_applied_phases:
                    desired = float(self._restoration_schedule[cur_phase])
                    # setPhaseDuration sets remaining, not total; we approximate by setting remaining to desired
                    
                    print(f"+++   Attempting to update extend TLS due to {plan.plan_type}, to {desired} seconds")
                    self._b1_dbg(
                        f"restore_phase_duration_apply phase={int(cur_phase)} desired={float(desired):.2f} sleep_sec=1.0"
                    )
                    traci.trafficlight.setPhaseDuration(tls_id, desired)
                    self._restoration_applied_phases.add(cur_phase)
            else:
                self._b1_dbg("restore_no_phase_duration_override")
            _emit_apply_compute_evt()
            return
        self._b1_dbg(f"apply_fallthrough_unknown_plan_type plan_type={getattr(plan, 'plan_type', None)}")
        _emit_apply_compute_evt()

    # =========================
    # Target-phase mapping (auto)
    # =========================

    def _build_inbound_edge_to_phase_map(self) -> Dict[str, int]:
        """
        Build inbound edge -> best phase index based on:
          - getControlledLinks(): linkIndex -> fromLane
          - program phases: state string, where state[linkIndex] in (G,g) means go
        """
        if traci is None:
            return {}

        tls_id = self.cfg.tls_id

        # Gather linkIndex -> fromLane
        try:
            controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        except Exception:
            return {}

        link_from_lane: Dict[int, str] = {}
        for idx, group in enumerate(controlled_links):
            if not group:
                continue
            from_lane = group[0][0]  # (fromLane, toLane, viaLane)
            link_from_lane[idx] = from_lane

        # Group link indices by inbound edge
        inbound_edge_to_indices: Dict[str, List[int]] = {}
        for li, from_lane in link_from_lane.items():
            in_edge = from_lane.rsplit("_", 1)[0]  # lane -> edge
            inbound_edge_to_indices.setdefault(in_edge, []).append(li)

        # Get phases/state strings
        cur_prog = str(traci.trafficlight.getProgram(tls_id))
        try:
            logics = traci.trafficlight.getAllProgramLogics(tls_id)
        except Exception:
            return {}

        logic = None
        for lg in logics:
            if getattr(lg, "programID", None) == cur_prog:
                logic = lg
                break
        if logic is None and logics:
            logic = logics[0]
        if logic is None:
            return {}

        phases = list(logic.phases)

        # Choose phase with max green chars across link indices
        mapping: Dict[str, int] = {}
        for in_edge, indices in inbound_edge_to_indices.items():
            best_phase = 0
            best_score = -1
            for pidx, ph in enumerate(phases):
                st = ph.state
                score = 0
                for li in indices:
                    if 0 <= li < len(st) and st[li] in ("G", "g"):
                        score += 1
                if score > best_score:
                    best_score = score
                    best_phase = pidx
            mapping[in_edge] = best_phase

        return mapping

    def _build_movement_edge_to_phase_map(self) -> Dict[Tuple[str, str], int]:
        """
        Build (in_edge, out_edge) -> best phase index.
        This is more precise than inbound-edge-only mapping at complex junctions
        where different turns from the same approach edge are served in different phases.
        """
        if traci is None:
            return {}

        tls_id = self.cfg.tls_id
        try:
            controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        except Exception:
            return {}

        cur_prog = str(traci.trafficlight.getProgram(tls_id))
        try:
            logics = traci.trafficlight.getAllProgramLogics(tls_id)
        except Exception:
            return {}

        logic = None
        for lg in logics:
            if getattr(lg, "programID", None) == cur_prog:
                logic = lg
                break
        if logic is None and logics:
            logic = logics[0]
        if logic is None:
            return {}
        phases = list(logic.phases)
        if not phases:
            return {}

        best: Dict[Tuple[str, str], Tuple[int, float, int]] = {}
        for idx, group in enumerate(controlled_links):
            if not group:
                continue
            sig_scores: List[Tuple[int, float, int]] = []
            for pidx, ph in enumerate(phases):
                st = str(getattr(ph, "state", ""))
                if idx < 0 or idx >= len(st):
                    continue
                ch = st[idx]
                if ch in ("G", "g"):
                    score = 2 if ch == "G" else 1
                    dur = float(getattr(ph, "duration", 0.0) or 0.0)
                    sig_scores.append((score, dur, pidx))
            if not sig_scores:
                continue
            sig_scores.sort(key=lambda x: (-x[0], -x[1], x[2]))
            best_phase = int(sig_scores[0][2])
            best_score = int(sig_scores[0][0])
            best_dur = float(sig_scores[0][1])
            for link in group:
                try:
                    from_lane, to_lane, _ = link
                except Exception:
                    continue
                if not from_lane or not to_lane:
                    continue
                in_edge = str(from_lane).rsplit("_", 1)[0]
                out_edge = str(to_lane).rsplit("_", 1)[0]
                key = (str(in_edge), str(out_edge))
                prev = best.get(key)
                cand = (best_score, best_dur, best_phase)
                if prev is None or cand[0] > prev[0] or (cand[0] == prev[0] and cand[1] > prev[1]):
                    best[key] = cand

        return {k: int(v[2]) for k, v in best.items()}
    
    def _build_outgoing_neighbor_map(self) -> Tuple[Dict[str, str], Dict[str, NeighborInfo]]:
        """
        Returns:
        out_edge_to_neighbor: local outgoing edge -> downstream tls_id
        neighbor_map: neighbor tls_id -> NeighborInfo
        """
        if traci is None:
            return {}, {}

        tls_id = self.cfg.tls_id
        out_edge_to_neighbor: Dict[str, str] = {}
        neighbor_map: Dict[str, NeighborInfo] = {}

        try:
            controlled = traci.trafficlight.getControlledLinks(tls_id)
        except Exception:
            return {}, {}

        # controlled[idx] is tuple-set of (fromLane, toLane, viaLane)
        for group in controlled:
            if not group:
                continue
            for link in group:
                from_lane, to_lane, _ = link
                if not to_lane:
                    continue
                out_edge = to_lane.rsplit("_", 1)[0]

                # Find TLS that controls inbound edge == out_edge
                neigh_tls = self._find_tls_controlling_inbound_edge(out_edge)
                if neigh_tls is None or neigh_tls == tls_id:
                    continue

                out_edge_to_neighbor[out_edge] = neigh_tls
                if neigh_tls not in neighbor_map:
                    neighbor_map[neigh_tls] = NeighborInfo(
                        tls_id=neigh_tls,
                        via_out_edge=out_edge,
                        neighbor_in_edge=out_edge
                    )

        return out_edge_to_neighbor, neighbor_map
    
    def _find_tls_controlling_inbound_edge(self, in_edge: str) -> Optional[str]:
        if traci is None:
            return None
        try:
            all_tls = traci.trafficlight.getIDList()
        except Exception:
            return None

        for tid in all_tls:
            try:
                links = traci.trafficlight.getControlledLinks(tid)
            except Exception:
                continue
            for group in links:
                if not group:
                    continue
                # inbound side = fromLane edge
                fr_lane = group[0][0]
                fr_edge = fr_lane.rsplit("_", 1)[0]
                if fr_edge == in_edge:
                    return tid
        return None



    # =========================
    # Termination conditions
    # =========================

    def _reset_ev_pass_tracking(self) -> None:
        self._prev_dist = None
        self._was_close = False
        self._ev_pred_ti_first = None
        self._ev_pred_ti_last = None
        self._ev_loop_touch_time = None
        self._ev_loop_touch_loop_id = None
        self._ev_touch_reported = False
        self._ev_request_silence_time = None
        self._ev_left_approach_time = None
        self._ev_pass_time_est = None
        self._ev_pass_detect_time = None
        self._ev_pass_proxy_time = None
        self._ev_left_approach_from_edge = None
        self._ev_left_approach_to_edge = None
        self._ev_last_seen_road_id = None
        self._ev_seen_on_approach_edge_time = None
        self._ev_pass_reason = ""

    def _update_ev_pass_runtime_observations(self, sim_time: float) -> None:
        """
        Keep runtime pass evidence:
        - EV request silence
        - EV left approach edge
        - EV touched loop near stopline
        """
        ev = self.active_ev
        if ev is None:
            return
        ev_id = str(ev.ev_id)
        now = float(sim_time)

        # Request silence marker.
        if self.last_ev_msg_time is not None:
            silence_age = float(now - float(self.last_ev_msg_time))
            if silence_age >= float(getattr(self.cfg, "ev_pass_request_silence_sec", 1.0)):
                if self._ev_request_silence_time is None:
                    self._ev_request_silence_time = float(now)
            else:
                self._ev_request_silence_time = None

        if traci is None:
            return

        # Left-approach marker.
        try:
            road_id = str(traci.vehicle.getRoadID(ev_id))
            self._ev_last_seen_road_id = road_id
            approach_edge = str(ev.in_edge_id)
            if road_id and road_id == approach_edge:
                if self._ev_seen_on_approach_edge_time is None:
                    self._ev_seen_on_approach_edge_time = float(now)
                self._ev_left_approach_time = None
                self._ev_left_approach_from_edge = None
                self._ev_left_approach_to_edge = None
            elif road_id and self._ev_seen_on_approach_edge_time is not None and road_id != approach_edge:
                if self._ev_left_approach_time is None:
                    self._ev_left_approach_time = float(now)
                if self._ev_left_approach_from_edge is None:
                    self._ev_left_approach_from_edge = str(approach_edge)
                # Keep first external edge as the post-intersection edge when possible.
                if self._ev_left_approach_to_edge is None:
                    self._ev_left_approach_to_edge = road_id
                elif str(self._ev_left_approach_to_edge).startswith(":") and not str(road_id).startswith(":"):
                    self._ev_left_approach_to_edge = road_id
        except Exception:
            pass

        # Loop-touch marker.
        if not bool(getattr(self.cfg, "ev_pass_use_loop_touch", True)):
            return
        if self._ev_loop_touch_time is not None:
            return
        if float(ev.distance_to_intersection_m) > float(getattr(self.cfg, "ev_pass_loop_touch_max_dist_m", 25.0)):
            return

        lanes: List[str] = []
        try:
            lane_id = str(traci.vehicle.getLaneID(ev_id))
            lane_edge = lane_id.rsplit("_", 1)[0] if "_" in lane_id else lane_id
            if lane_edge == str(ev.in_edge_id):
                lanes = [lane_id]
        except Exception:
            lanes = []

        if not lanes:
            try:
                nlanes = max(int(traci.edge.getLaneNumber(str(ev.in_edge_id))), 1)
            except Exception:
                nlanes = 1
            lanes = [f"{str(ev.in_edge_id)}_{i}" for i in range(nlanes)]

        self._refresh_lane_loop_map()
        if lanes and not any(self._lane_loop_ids.get(str(l), []) for l in lanes):
            self._refresh_lane_loop_map(force=True)

        for lane_id in lanes:
            for loop_id in list(self._lane_loop_ids.get(str(lane_id), []) or []):
                try:
                    vids = {str(v) for v in list(traci.inductionloop.getLastStepVehicleIDs(str(loop_id)))}
                except Exception:
                    continue
                if ev_id in vids:
                    self._ev_loop_touch_time = float(now)
                    self._ev_loop_touch_loop_id = str(loop_id)
                    if bool(getattr(self.cfg, "ev_pass_enable_debug", True)):
                        print(
                            "[EV_PASS_TOUCH] "
                            f"tls={self.cfg.tls_id} "
                            f"ev={ev_id} "
                            f"t={float(now):.2f} "
                            f"loop={loop_id} "
                            f"dist={float(ev.distance_to_intersection_m):.2f}"
                        )
                    return

    def _detect_ev_passed(self, sim_time: Optional[float] = None) -> bool:
        """
        Robust pass detection combining:
        - distance trend (baseline)
        - loop-touch + request-silence + left-approach evidence
        """
        if self.active_ev is None:
            return False

        now = float(sim_time) if sim_time is not None else float(self._now())
        self._update_ev_pass_runtime_observations(now)

        dist = float(self.active_ev.distance_to_intersection_m)
        print(f"Distance to intersection: {dist:.1f}m")
        if dist < 10.0:
            self._was_close = True

        passed = False
        reason = ""
        if self._was_close and self._prev_dist is not None:
            if dist > self._prev_dist + 2.0:  # moving away
                passed = True
                reason = "distance_trend"

        # If EV was close and has already moved to a different approach edge,
        # consider it passed even when distance-trend cadence is too coarse.
        left_edge_max_dist = max(0.0, float(getattr(self.cfg, "ev_pass_left_edge_max_dist_m", 2.5)))
        if (
            (not passed)
            and self._was_close
            and (self._ev_left_approach_time is not None)
            and (dist <= left_edge_max_dist)
        ):
            passed = True
            reason = "left_approach_edge"

        if (not passed) and bool(getattr(self.cfg, "ev_pass_use_request_silence", True)):
            if self._ev_loop_touch_time is not None and self._ev_request_silence_time is not None:
                min_gap = max(0.0, float(getattr(self.cfg, "ev_pass_min_loop_to_silence_sec", 0.2)))
                if float(self._ev_request_silence_time - self._ev_loop_touch_time) >= min_gap:
                    if (self._ev_left_approach_time is not None) or (
                        self._prev_dist is not None and dist > self._prev_dist + 0.5
                    ):
                        passed = True
                        reason = "loop_touch_plus_request_silence"

        if passed and self._ev_pass_time_est is None:
            # pass_t aims to approximate physical pass timing from evidence (left-edge/silence),
            # pass_detect_t is when this agent actually detected/processed the event,
            # and pass_proxy_t keeps earliest loop-touch evidence.
            pass_t_est = float(now)
            if reason == "left_approach_edge" and self._ev_left_approach_time is not None:
                pass_t_est = float(self._ev_left_approach_time)
            elif reason == "loop_touch_plus_request_silence" and self._ev_request_silence_time is not None:
                pass_t_est = float(self._ev_request_silence_time)

            self._ev_pass_time_est = float(pass_t_est)
            self._ev_pass_detect_time = float(now)
            self._ev_pass_proxy_time = (
                float(self._ev_loop_touch_time) if self._ev_loop_touch_time is not None else None
            )
            self._ev_pass_reason = str(reason or "unknown")
            ev_id_pass = str(self.active_ev.ev_id)
            self._recent_pass_time_by_ev[ev_id_pass] = float(now)
            self._recent_pass_info_by_ev[ev_id_pass] = {
                "tls_id": str(self.cfg.tls_id),
                "ev_id": str(ev_id_pass),
                "pass_time": float(self._ev_pass_time_est),
                "pass_detect_time": float(self._ev_pass_detect_time if self._ev_pass_detect_time is not None else now),
                "pass_reason": str(self._ev_pass_reason),
                "in_edge_id": str(getattr(self.active_ev, "in_edge_id", "") or ""),
                "left_approach_from_edge": str(self._ev_left_approach_from_edge or getattr(self.active_ev, "in_edge_id", "") or ""),
                "left_approach_to_edge": str(self._ev_left_approach_to_edge or self._ev_last_seen_road_id or ""),
            }
            self._pending_handoff_ev_id = ev_id_pass
            self._pending_handoff_time = float(now)
            self._fed_evt(
                "ev.pass.detected",
                ev_id=str(ev_id_pass),
                tls_id=str(self.cfg.tls_id),
                reason=str(self._ev_pass_reason),
                pass_t=float(self._ev_pass_time_est),
                pass_detect_t=float(self._ev_pass_detect_time if self._ev_pass_detect_time is not None else now),
                pass_proxy_t=(float(self._ev_pass_proxy_time) if self._ev_pass_proxy_time is not None else None),
                in_edge_id=str(getattr(self.active_ev, "in_edge_id", "") or ""),
                left_approach_from_edge=str(
                    self._ev_left_approach_from_edge or getattr(self.active_ev, "in_edge_id", "") or ""
                ),
                left_approach_to_edge=str(self._ev_left_approach_to_edge or self._ev_last_seen_road_id or ""),
                ev_distance_m=float(getattr(self.active_ev, "distance_to_intersection_m", -1.0)),
                ev_speed_mps=float(getattr(self.active_ev, "speed_mps", -1.0)),
                source_service="intersection_agent",
                role="intersection",
            )
            try:
                d_first = (
                    float(self._ev_pass_time_est - self._ev_pred_ti_first)
                    if self._ev_pred_ti_first is not None
                    else float("nan")
                )
            except Exception:
                d_first = float("nan")
            try:
                d_last = (
                    float(self._ev_pass_time_est - self._ev_pred_ti_last)
                    if self._ev_pred_ti_last is not None
                    else float("nan")
                )
            except Exception:
                d_last = float("nan")
            veh_wait = -1.0
            try:
                if traci is not None:
                    veh_wait = float(traci.vehicle.getWaitingTime(ev_id_pass))
            except Exception:
                veh_wait = -1.0
            edge_transition = (
                f"{str(self._ev_left_approach_from_edge or self.active_ev.in_edge_id)}"
                f"->{str(self._ev_left_approach_to_edge or self._ev_last_seen_road_id or 'NA')}"
            )
            self._fed_evt(
                "ev.node.cross",
                ev_id=str(ev_id_pass),
                tls_id=str(self.cfg.tls_id),
                mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                reason=str(self._ev_pass_reason),
                pass_t=float(self._ev_pass_time_est),
                pass_detect_t=float(self._ev_pass_detect_time if self._ev_pass_detect_time is not None else now),
                pass_proxy_t=(float(self._ev_pass_proxy_time) if self._ev_pass_proxy_time is not None else None),
                edge_transition=str(edge_transition),
                delay_vs_first_pred_sec=(None if math.isnan(d_first) else float(d_first)),
                delay_vs_last_pred_sec=(None if math.isnan(d_last) else float(d_last)),
                veh_waiting_time_sec=(float(veh_wait) if veh_wait >= 0.0 else None),
                ev_distance_m=float(getattr(self.active_ev, "distance_to_intersection_m", -1.0)),
                ev_speed_mps=float(getattr(self.active_ev, "speed_mps", -1.0)),
                source_service="intersection_agent",
                role="intersection",
            )
            if bool(getattr(self.cfg, "ev_pass_enable_debug", True)):
                e0 = (
                    float(self._ev_pass_time_est - self._ev_pred_ti_first)
                    if self._ev_pred_ti_first is not None
                    else float("nan")
                )
                e1 = (
                    float(self._ev_pass_time_est - self._ev_pred_ti_last)
                    if self._ev_pred_ti_last is not None
                    else float("nan")
                )
                print(
                    "[EV_PASS_EVENT] "
                    f"tls={self.cfg.tls_id} "
                    f"ev={self.active_ev.ev_id} "
                    f"pass_t={float(self._ev_pass_time_est):.2f} "
                    f"pass_detect_t={self._ev_pass_detect_time} "
                    f"pass_proxy_t={self._ev_pass_proxy_time} "
                    f"reason={self._ev_pass_reason} "
                    f"loop_touch_t={self._ev_loop_touch_time} "
                    f"silence_t={self._ev_request_silence_time} "
                    f"left_edge_t={self._ev_left_approach_time} "
                    f"edge_transition={edge_transition} "
                    f"err_first={e0:.2f} "
                    f"err_last={e1:.2f}"
                )

        self._prev_dist = dist
        print(
            f">>>>> Currently on {self.cfg.tls_id}, distance to intersection: {dist:.1f}m, "
            f"was_close: {self._was_close}, prev_dist: {self._prev_dist:.1f}m, passed: {passed}\n<<<<"
        )
        return passed

    def claim_pending_handoff_ev_id(self, sim_time: float) -> Optional[str]:
        now = float(sim_time)
        ttl = max(0.5, float(getattr(self.cfg, "ev_handoff_pending_ttl_sec", 10.0)))

        # Fast-path when current session still exists and was marked passed.
        if self.active_ev is not None and bool(self.ev_passed):
            ev_id = str(self.active_ev.ev_id)
            if self._pending_handoff_ev_id == ev_id:
                self._pending_handoff_ev_id = None
                self._pending_handoff_time = None
            return ev_id

        if self._pending_handoff_ev_id is None or self._pending_handoff_time is None:
            return None
        if (now - float(self._pending_handoff_time)) > ttl:
            self._pending_handoff_ev_id = None
            self._pending_handoff_time = None
            return None

        ev_id = str(self._pending_handoff_ev_id)
        self._pending_handoff_ev_id = None
        self._pending_handoff_time = None
        return ev_id

    def _restoration_complete(self) -> bool:
        # For middleware evaluation, restoring program is usually sufficient.
        return True

    def _transition_to(self, new_stage: AgentStage, reason: str = "") -> None:
        if new_stage != self.stage:
            old_stage = self.stage
            self.stage = new_stage
            self._fed_evt(
                "agent.stage.transition",
                old_stage=str(old_stage),
                new_stage=str(new_stage),
                reason=str(reason or ""),
                ev_id=(str(self.active_ev.ev_id) if self.active_ev is not None else ""),
                active_reservations=int(len(getattr(self, "active_reservations", {}))),
            )

    def _reservation_feedback(self, ev_id: str, max_age_sec: float = 8.0) -> Dict[str, int]:
        now = self._now()
        out = {
            "hard_accepted": 0,
            "hard_rejected": 0,
            "soft_accepted": 0,
            "soft_rejected": 0,
        }
        for meta in self.resp_cache.values():
            if str(meta.get("ev_id", "")) != str(ev_id):
                continue
            if (now - float(meta.get("ts", 0.0))) > float(max_age_sec):
                continue
            status = str(meta.get("status", "")).upper()
            mode = str(meta.get("mode", "")).lower()
            if mode == "hard":
                if status == "ACCEPTED":
                    out["hard_accepted"] += 1
                elif status == "REJECTED":
                    out["hard_rejected"] += 1
            else:
                if status == "ACCEPTED":
                    out["soft_accepted"] += 1
                elif status == "REJECTED":
                    out["soft_rejected"] += 1
        return out

    def _reservation_eta_shift_feedback(self, ev_id: str, max_age_sec: float = 8.0) -> float:
        """
        Aggregate recent downstream timing feedback from reservation responses.
        We only trust positive shift suggestions from rejections and keep the max
        to quickly move the ETA window out of infeasible slots.
        """
        now = self._now()
        shift_max = 0.0
        for meta in self.resp_cache.values():
            if str(meta.get("ev_id", "")) != str(ev_id):
                continue
            if (now - float(meta.get("ts", 0.0))) > float(max_age_sec):
                continue
            status = str(meta.get("status", "")).upper()
            if status != "REJECTED":
                continue
            try:
                shift = float(meta.get("downstream_suggested_eta_shift_sec", 0.0) or 0.0)
            except Exception:
                shift = 0.0
            if shift > shift_max:
                shift_max = shift
        return float(max(0.0, shift_max))
    
    # ==================================
    # Methods for Federated cooperation

    def _now(self) -> float:
        if traci is not None:
            try:
                return float(traci.simulation.getTime())
            except Exception:
                pass
        return time.time()

    def _norm_eta_window(self, eta_start: float, eta_end: float) -> Tuple[float, float]:
        a, b = float(eta_start), float(eta_end)
        if b < a:
            a, b = b, a
        return a, b

    def _active_ev_distance_m(self, ev_id_hint: str = "") -> float:
        try:
            if self.active_ev is not None:
                ev_id = str(getattr(self.active_ev, "ev_id", "") or "")
                if (not ev_id_hint) or (ev_id == str(ev_id_hint)):
                    return float(getattr(self.active_ev, "distance_to_intersection_m", -1.0))
        except Exception:
            pass
        return -1.0

    @staticmethod
    def _distance_adaptive_value(
        *,
        base: float,
        near_value: float,
        far_value: float,
        ev_distance_m: float,
        near_distance_m: float,
        far_distance_m: float,
    ) -> float:
        if ev_distance_m < 0.0:
            return float(base)
        n = max(0.0, float(near_distance_m))
        f = max(n, float(far_distance_m))
        if f <= n + 1e-6:
            return float(near_value if ev_distance_m <= n else far_value)
        if ev_distance_m <= n:
            return float(near_value)
        if ev_distance_m >= f:
            return float(far_value)
        a = (float(ev_distance_m) - n) / (f - n)
        return float(near_value + a * (far_value - near_value))

    def _coordination_window_active(self, ev_id_hint: str = "", now_sim: Optional[float] = None) -> bool:
        if not bool(getattr(self.cfg, "f2_active_coord_window_relax_enable", False)):
            return False
        now = float(self._now() if now_sim is None else now_sim)
        recent_sec = max(0.1, float(getattr(self.cfg, "f2_active_coord_window_recent_sec", 2.5)))
        near_m = max(0.0, float(getattr(self.cfg, "f2_active_coord_window_ev_near_m", 180.0)))
        min_active = max(0, int(getattr(self.cfg, "f2_active_coord_window_min_active_reservations", 1)))

        ev_id = str(ev_id_hint or (self.active_ev.ev_id if self.active_ev is not None else "") or "")
        ev_dist = float(self._active_ev_distance_m(ev_id))
        if ev_dist >= 0.0 and ev_dist > near_m:
            return False

        active_res_n = 0
        for rs in list(self.active_reservations.values()):
            try:
                if ev_id and str(getattr(rs, "ev_id", "") or "") != ev_id:
                    continue
                if float(getattr(rs, "ts_expire", now + 1.0)) >= now:
                    active_res_n += 1
            except Exception:
                continue

        recent_req = False
        for rec in list(self._fed_req_sent_clock.values()):
            try:
                if ev_id and str((rec or {}).get("ev_id", "")) and str((rec or {}).get("ev_id", "")) != ev_id:
                    continue
                ts0 = float((rec or {}).get("sim", now - 1e9))
                if (now - ts0) <= recent_sec:
                    recent_req = True
                    break
            except Exception:
                continue

        recent_resp = False
        for rec in list(self.resp_cache.values()):
            try:
                if ev_id and str((rec or {}).get("ev_id", "")) and str((rec or {}).get("ev_id", "")) != ev_id:
                    continue
                ts0 = float((rec or {}).get("ts", now - 1e9))
                if (now - ts0) <= recent_sec:
                    recent_resp = True
                    break
            except Exception:
                continue

        if active_res_n >= min_active:
            return True
        return bool(recent_req or recent_resp)


    def _queue_fed_msg(self, topic: str, payload: dict) -> None:
        self._fed_outbox.append((topic, payload))
        cur_depth = int(len(self._fed_outbox))
        if cur_depth > int(self._fed_outbox_depth_peak):
            self._fed_outbox_depth_peak = int(cur_depth)
            self._fed_evt(
                "coord.outbox.depth.peak",
                depth=int(cur_depth),
                topic=str(topic),
            )

    def _fed_dbg(self, msg: str) -> None:
        if bool(getattr(self, "enable_federation_debug", False)):
            t_now = float(self._now())
            line = f"[FED_DEBUG] t={t_now:.2f} tls={self.cfg.tls_id} {msg}"
            if bool(getattr(self, "fed_debug_print", True)):
                print(line)
            log_path = str(getattr(self, "fed_debug_log_path", "") or "")
            if log_path:
                try:
                    os.makedirs(os.path.dirname(log_path), exist_ok=True)
                except Exception:
                    pass
                try:
                    with open(log_path, "a", encoding="utf-8") as f:
                        f.write(line + "\n")
                except Exception:
                    # Debug logging must never affect control behavior.
                    pass

    def _fed_evt(self, event_type: str, **payload: object) -> None:
        if not bool(getattr(self, "enable_fed_event_jsonl", False)):
            self._fed_dbg("evt=EVENT_JSONL_SKIP reason=disabled")
            return

        path = str(getattr(self, "fed_event_jsonl_path", "") or "").strip()
        if not path:
            fb = str(getattr(self, "fed_debug_log_path", "") or "").strip()
            if fb:
                path = fb.replace(".txt", ".events.jsonl")
                self.fed_event_jsonl_path = path
        if not path:
            self._fed_dbg("evt=EVENT_JSONL_SKIP reason=empty_path")
            return

        try:
            sim_t = float(self._now())
        except Exception:
            sim_t = float(time.time())

        rec: Dict[str, object] = {
            "ts_wall": float(time.time()),
            "sim_time": float(sim_t),
            "source_service": "intersection_agent",
            "dt_type": "intersection",
            "tls_id": str(self.cfg.tls_id),
            "event_type": str(event_type),
        }
        if self._fed_run_id:
            rec["run_id"] = str(self._fed_run_id)
        if self._fed_topic_namespace:
            rec["topic_namespace"] = str(self._fed_topic_namespace)
        for k, v in payload.items():
            if v is None:
                continue
            rec[str(k)] = v

        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=True, separators=(",", ":")) + "\n")
            if not bool(getattr(self, "_fed_evt_write_ok_logged", False)):
                self._fed_evt_write_ok_logged = True
                self._fed_dbg(f"evt=EVENT_JSONL_WRITE_OK path={path}")
        except Exception as e:
            self._fed_dbg(f"evt=EVENT_JSONL_WRITE_ERR err={e}")

    def _trace_tls_signal_change(self, sim_time: float, reason: str = "tick", force: bool = False) -> None:
        if not bool(getattr(self.cfg, "tls_signal_trace_enable", False)):
            return
        if traci is None:
            return
        tls_id = str(self.cfg.tls_id)
        try:
            phase_idx = int(traci.trafficlight.getPhase(tls_id))
            signal_state = str(traci.trafficlight.getRedYellowGreenState(tls_id) or "")
            next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
            rem = max(0.0, float(next_switch) - float(sim_time))
        except Exception as e:
            self._fed_dbg(f"evt=TLS_SIGNAL_TRACE_ERR reason={reason} err={type(e).__name__}:{e}")
            return

        changed = bool(force)
        if not changed:
            if self._last_seen_phase is None or int(phase_idx) != int(self._last_seen_phase):
                changed = True
            elif str(signal_state) != str(self._last_tls_signal_state or ""):
                changed = True

        self._last_seen_phase = int(phase_idx)
        self._last_tls_signal_state = str(signal_state)
        self._last_tls_signal_next_switch = float(next_switch)

        if not changed:
            return
        self._last_tls_signal_change_sim_time = float(sim_time)

        active_ev = self.active_ev
        ev_src = str(getattr(active_ev, "source_service", "") or "") if active_ev is not None else ""
        ev_src_tag = str(getattr(active_ev, "source_tag", "") or "") if active_ev is not None else ""
        ev_delivery = str(getattr(active_ev, "delivery", "") or "") if active_ev is not None else ""

        self._fed_dbg(
            f"evt=TLS_SIGNAL_CHANGE reason={reason} phase={int(phase_idx)} rem={float(rem):.2f} "
            f"next_switch={float(next_switch):.2f} src={ev_src or '-'} src_tag={ev_src_tag or '-'} delivery={ev_delivery or '-'}"
        )
        self._fed_evt(
            "tls.signal.change",
            role="intersection",
            source_service="intersection_agent",
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            reason=str(reason),
            phase_idx=int(phase_idx),
            signal_state=str(signal_state),
            next_switch=float(next_switch),
            remaining_sec=float(rem),
            ev_id=(str(active_ev.ev_id) if active_ev is not None else ""),
            ev_request_source=str(ev_src),
            ev_request_source_tag=str(ev_src_tag or getattr(self.cfg, "ev_request_source_tag", "")),
            ev_request_delivery=str(ev_delivery),
        )

    def _b1_dbg(self, msg: str) -> None:
        if bool(getattr(self, "enable_federation_debug", False)):
            self._fed_dbg(f"evt=B1_AGENT {msg}")

    def _plan_selected_dbg(
        self,
        sim_time: float,
        decision_source: str,
        plan: Optional[PreemptionPlan] = None,
        offer: Optional[SignalWindowOffer] = None,
        note: str = "",
    ) -> None:
        if not bool(getattr(self, "enable_federation_debug", False)):
            return
        try:
            t_now = float(sim_time)
        except Exception:
            t_now = float(self._now())
        parts = [
            f"[PLAN_SELECTED_DEBUG] t={t_now:.2f}",
            f"tls={self.cfg.tls_id}",
            f"src={str(decision_source)}",
            f"stage={self.stage}",
        ]
        if plan is not None:
            parts.extend([
                f"plan_type={getattr(plan, 'plan_type', None)}",
                f"plan_target={getattr(plan, 'target_phase_idx', None)}",
                f"ext={float(getattr(plan, 'extend_green_sec', 0.0) or 0.0):.2f}",
                f"hurry={getattr(plan, 'hurry_current_phase_to_sec', None)}",
                f"jump_t={getattr(plan, 'jump_time_sec', None)}",
                f"jump_to={getattr(plan, 'jump_to_phase_idx', None)}",
            ])
        if offer is not None:
            try:
                offer_score = float(getattr(offer, "score", self.score_offer(offer)))
            except Exception:
                offer_score = float("nan")
            parts.extend([
                f"offer_id={getattr(offer, 'offer_id', '')}",
                f"offer_action={getattr(offer, 'action', '')}",
                (f"offer_score={offer_score:.4f}" if math.isfinite(offer_score) else "offer_score=nan"),
                f"offer_feasible={int(bool(getattr(offer, 'feasible', False)))}",
                f"offer_conf={float(getattr(offer, 'confidence', 0.0) or 0.0):.3f}",
            ])
        if note:
            parts.append(f"note={str(note)}")
        line = " ".join(parts)
        if bool(getattr(self, "fed_debug_print", True)):
            print(line)
        log_path = str(getattr(self, "fed_debug_log_path", "") or "")
        if log_path:
            try:
                os.makedirs(os.path.dirname(log_path), exist_ok=True)
            except Exception:
                pass
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass


    def drain_federation_outbox(self) -> List[Tuple[str, dict]]:
        out = self._fed_outbox
        self._fed_outbox = []
        if out:
            self._fed_evt(
                "coord.outbox.drain",
                n=int(len(out)),
                depth_peak=int(self._fed_outbox_depth_peak),
            )
        if out and bool(getattr(self, "enable_federation_debug", False)):
            topics = [str(t) for t, _ in out[:4]]
            if len(out) > 4:
                topics.append("...")
            self._fed_dbg(f"evt=OUTBOX_DRAIN n={len(out)} topics={topics}")
        return out


    def _send_reservation_req(self, to_tls: str, req: dict) -> None:
        # transport-agnostic: main publishes these
        if str(to_tls or "") == str(self.cfg.tls_id):
            self._fed_evt(
                "coord.reservation.req_out_skip",
                reason="self_target",
                req_id=str(req.get("req_id", "") or ""),
                ev_id=str(req.get("ev_id", "") or ""),
                to_tls=str(to_tls or ""),
                pending_req_n=int(len(self._fed_req_sent_clock)),
                outbox_depth=int(len(self._fed_outbox)),
            )
            return
        req_id = str(req.get("req_id", "") or "")
        now_sim = float(self._now())
        now_wall = float(time.time())
        # Clean stale pending request clocks to avoid unbounded pending counts under loss/churn.
        stale_sec = max(1.0, float(getattr(self, "fed_req_pending_stale_sec", 6.0)))
        stale_keys: List[str] = []
        for _rid, _clk in list(self._fed_req_sent_clock.items()):
            try:
                _sim0 = float((_clk or {}).get("sim", now_sim))
                if (now_sim - _sim0) > stale_sec:
                    stale_keys.append(str(_rid))
            except Exception:
                continue
        for _rid in stale_keys:
            self._fed_req_sent_clock.pop(_rid, None)
            self._fed_req_sent_ts.pop(_rid, None)
        if stale_keys:
            self._fed_evt(
                "coord.reservation.req_pending_gc",
                stale_removed_n=int(len(stale_keys)),
                stale_sec=float(stale_sec),
                pending_req_n=int(len(self._fed_req_sent_clock)),
            )
        peer_key = str(to_tls)
        dq = self._fed_req_recent_by_peer.get(peer_key)
        if dq is None:
            dq = deque()
            self._fed_req_recent_by_peer[peer_key] = dq
        while dq and (now_sim - float(dq[0])) > 1.0:
            dq.popleft()
        prev_sim = float(dq[-1]) if dq else None
        dt_prev_ms = (1000.0 * (now_sim - prev_sim)) if prev_sim is not None else None
        ev_dist_m = -1.0
        try:
            ev_dist_m = float(req.get("distance_to_intersection_m", -1.0))
        except Exception:
            ev_dist_m = -1.0
        if ev_dist_m < 0.0:
            ev_dist_m = float(self._active_ev_distance_m(str(req.get("ev_id", "") or "")))
        min_gap_base = max(0.0, float(getattr(self, "fed_req_send_min_gap_sec", 0.60)))
        min_gap_near = max(
            0.0,
            float(getattr(self.cfg, "fed_req_send_min_gap_near_sec", min_gap_base)),
        )
        min_gap_far = max(
            0.0,
            float(getattr(self.cfg, "fed_req_send_min_gap_far_sec", min_gap_base)),
        )
        min_gap_near_m = max(
            0.0,
            float(getattr(self.cfg, "fed_req_send_min_gap_near_distance_m", 120.0)),
        )
        min_gap_far_m = max(
            min_gap_near_m,
            float(getattr(self.cfg, "fed_req_send_min_gap_far_distance_m", 300.0)),
        )
        min_gap_sec = self._distance_adaptive_value(
            base=min_gap_base,
            near_value=min_gap_near,
            far_value=min_gap_far,
            ev_distance_m=float(ev_dist_m),
            near_distance_m=float(min_gap_near_m),
            far_distance_m=float(min_gap_far_m),
        )
        if prev_sim is not None and (now_sim - prev_sim) < min_gap_sec:
            self._fed_evt(
                "coord.reservation.req_out_skip",
                reason="min_gap",
                req_id=req_id,
                ev_id=str(req.get("ev_id", "") or ""),
                to_tls=str(to_tls),
                ev_distance_m=float(ev_dist_m),
                dt_prev_ms=float(dt_prev_ms) if dt_prev_ms is not None else None,
                min_gap_ms=float(1000.0 * min_gap_sec),
                min_gap_near_ms=float(1000.0 * min_gap_near),
                min_gap_far_ms=float(1000.0 * min_gap_far),
                min_gap_near_m=float(min_gap_near_m),
                min_gap_far_m=float(min_gap_far_m),
                pending_req_n=int(len(self._fed_req_sent_clock)),
                outbox_depth=int(len(self._fed_outbox)),
            )
            return
        pending_peer_n = 0
        for _clk in list(self._fed_req_sent_clock.values()):
            try:
                if str((_clk or {}).get("to_tls", "")) == peer_key:
                    pending_peer_n += 1
            except Exception:
                continue
        pending_cap = max(1, int(getattr(self, "fed_req_pending_per_peer_cap", 2)))
        if pending_peer_n >= pending_cap:
            self._fed_evt(
                "coord.reservation.req_out_skip",
                reason="pending_cap",
                req_id=req_id,
                ev_id=str(req.get("ev_id", "") or ""),
                to_tls=str(to_tls),
                pending_peer_n=int(pending_peer_n),
                pending_cap=int(pending_cap),
                pending_req_n=int(len(self._fed_req_sent_clock)),
                outbox_depth=int(len(self._fed_outbox)),
            )
            return
        dq.append(float(now_sim))
        recent_1s_sim = int(len(dq))
        repeated_within_1s = bool(dt_prev_ms is not None and float(dt_prev_ms) <= 1000.0)
        if req_id:
            self._fed_req_sent_ts[req_id] = float(now_sim)
            self._fed_req_sent_clock[req_id] = {
                "sim": float(now_sim),
                "wall": float(now_wall),
                "ev_id": str(req.get("ev_id", "") or ""),
                "from_tls": str(self.cfg.tls_id),
                "to_tls": str(to_tls),
                "mode": str(req.get("mode", "") or ""),
                "source_local_compute_ms": float(req.get("source_local_compute_ms", req.get("local_compute_ms", 0.0)) or 0.0),
                "source_fnm_integration_ms": float(req.get("source_fnm_integration_ms", req.get("fnm_integration_ms", 0.0)) or 0.0),
            }
        self._fed_evt(
            "coord.reservation.req_out",
            req_id=req_id,
            ev_id=str(req.get("ev_id", "") or ""),
            to_tls=str(to_tls),
            mode=str(req.get("mode", "") or ""),
            confidence=float(req.get("confidence", 0.0) or 0.0),
            eta_start=float(req.get("eta_start", 0.0) or 0.0),
            eta_end=float(req.get("eta_end", 0.0) or 0.0),
            in_edge_id=(str(req.get("in_edge_id")) if req.get("in_edge_id") is not None else ""),
            next_edge_id=(str(req.get("next_edge_id")) if req.get("next_edge_id") is not None else ""),
            preferred_next_tls=(str(req.get("preferred_next_tls")) if req.get("preferred_next_tls") is not None else ""),
            req_out_recent_1s_sim=int(recent_1s_sim),
            req_out_dt_prev_ms=float(dt_prev_ms) if dt_prev_ms is not None else None,
            req_out_repeated_within_1s=int(1 if repeated_within_1s else 0),
            pending_req_n=int(len(self._fed_req_sent_ts)),
            outbox_depth=int(len(self._fed_outbox)),
        )
        self._queue_fed_msg(f"federation/reservation/req/{to_tls}", req)
        self._fed_dbg(
            f"evt=REQ_OUT send_req req_id={req.get('req_id')} ev={req.get('ev_id')} to={to_tls} mode={req.get('mode')} "
            f"p={float(req.get('confidence', 0.0)):.3f} "
            f"eta=({float(req.get('eta_start', 0.0)):.2f},{float(req.get('eta_end', 0.0)):.2f}) "
            f"in_edge={req.get('in_edge_id')} next_edge={req.get('next_edge_id')} "
            f"preferred={req.get('preferred_next_tls')}"
        )
        if req.get("next_edge_id") is None or req.get("preferred_next_tls") is None:
            self._fed_dbg(
                f"evt=REQ_OUT_WARN req_id={req.get('req_id')} ev={req.get('ev_id')} to={to_tls} "
                f"missing_next_edge={int(req.get('next_edge_id') is None)} "
                f"missing_preferred={int(req.get('preferred_next_tls') is None)}"
            )


    def _estimate_edge_travel_sec(self, edge_id: str) -> float:
        if traci is None or not edge_id or edge_id.startswith(":"):
            return 3.0
        try:
            L = float(traci.edge.getLength(edge_id))
        except Exception:
            L = 60.0
        try:
            v = float(traci.edge.getSpeed(edge_id))
        except Exception:
            v = 13.9
        v = max(v, 1.0)
        return max(1.0, min(30.0, L / v))

    def _infer_route_intersections_from_traci(self, ev_id: str, max_count: int = 8) -> List[str]:
        """
        Best-effort extraction of upcoming TLS IDs along EV route from SUMO runtime.
        """
        if traci is None:
            return []
        out: List[str] = []
        try:
            nxt = list(traci.vehicle.getNextTLS(str(ev_id)) or [])
        except Exception:
            nxt = []
        for item in nxt:
            try:
                tls = str(item[0])
            except Exception:
                continue
            if tls not in out:
                out.append(tls)
            if len(out) >= int(max_count):
                break
        if not out:
            return []
        # keep current TLS as anchor when available
        if str(self.cfg.tls_id) not in out:
            out = [str(self.cfg.tls_id)] + out
        return out[:max_count]

    def _route_intersection_hints(self, ev_id: str) -> List[str]:
        """
        Preferred downstream TLS sequence from EV request metadata.
        """
        if not bool(getattr(self.cfg, "fed_enable_route_intersections_hint", True)):
            return []
        ev = self.active_ev
        if ev is not None and str(ev.ev_id) == str(ev_id):
            route_tls = list(getattr(ev, "route_intersections", []) or [])
        else:
            route_tls = list(self._last_route_intersections_by_ev.get(str(ev_id), []) or [])
        if not route_tls:
            return []
        route_tls = [str(x) for x in route_tls]
        # Keep only future segment after current TLS if present.
        if str(self.cfg.tls_id) in route_tls:
            idx = route_tls.index(str(self.cfg.tls_id))
            route_tls = route_tls[idx + 1 :]
        lookahead = max(1, int(getattr(self.cfg, "fed_route_hint_lookahead_hops", 4)))
        route_tls = route_tls[:lookahead]
        return [tls for tls in route_tls if tls]

    def _route_edges_hint(self, ev_id: str) -> List[str]:
        """
        Preferred route-edge hint sequence:
        1) active EV message route_veh
        2) cached route_veh for this EV
        3) live SUMO route
        """
        ev = self.active_ev
        if ev is not None and str(ev.ev_id) == str(ev_id):
            edges = list(getattr(ev, "route_veh", []) or [])
            if edges:
                return [str(e) for e in edges if str(e)]
        cached = list(self._last_route_edges_by_ev.get(str(ev_id), []) or [])
        if cached:
            return [str(e) for e in cached if str(e)]
        if traci is not None:
            try:
                live = list(traci.vehicle.getRoute(str(ev_id)) or [])
                if live:
                    return [str(e) for e in live if str(e)]
            except Exception:
                pass
        return []

    def _next_out_edge_from_ev_message(self, msg: EvRequest) -> Optional[str]:
        """
        Infer likely downstream out-edge at this TLS for the EV movement.
        Priority:
          1) msg.route_veh around msg.in_edge_id
          2) cached route_veh for this EV
          3) live SUMO route index fallback
        """
        in_edge = str(getattr(msg, "in_edge_id", "") or "")
        ev_id = str(getattr(msg, "ev_id", "") or "")
        if not in_edge:
            return None

        route_idx_hint = -1
        if traci is not None and ev_id:
            try:
                route_idx_hint = int(traci.vehicle.getRouteIndex(ev_id))
            except Exception:
                route_idx_hint = -1

        candidates: List[List[str]] = []
        msg_route = [str(e) for e in list(getattr(msg, "route_veh", []) or []) if str(e)]
        if msg_route:
            candidates.append(msg_route)
        cached_route = [str(e) for e in list(self._last_route_edges_by_ev.get(ev_id, []) or []) if str(e)]
        if cached_route:
            candidates.append(cached_route)

        for route_edges in candidates:
            idxs = [i for i, e in enumerate(route_edges[:-1]) if str(e) == in_edge]
            if not idxs:
                continue
            if route_idx_hint >= 0:
                idx = next((i for i in idxs if i >= max(0, route_idx_hint - 1)), idxs[-1])
            else:
                idx = idxs[-1]
            for j in range(idx + 1, min(len(route_edges), idx + 7)):
                e2 = str(route_edges[j])
                if e2 and not e2.startswith(":"):
                    return e2

        if traci is not None and ev_id:
            try:
                live_route = [str(e) for e in list(traci.vehicle.getRoute(ev_id) or []) if str(e)]
                ridx = int(traci.vehicle.getRouteIndex(ev_id))
            except Exception:
                live_route = []
                ridx = -1
            if live_route:
                start = max(0, ridx + 1) if ridx >= 0 else 0
                for j in range(start, min(len(live_route), start + 8)):
                    e2 = str(live_route[j])
                    if e2 and not e2.startswith(":"):
                        return e2
        return None

    def _next_tls_hint_from_traci(self, ev_id: str) -> Optional[str]:
        if traci is None or not bool(getattr(self.cfg, "fed_enable_nexttls_hint", True)):
            return None
        try:
            nxt = list(traci.vehicle.getNextTLS(str(ev_id)) or [])
        except Exception:
            nxt = []
        for item in nxt:
            try:
                tls = str(item[0])
            except Exception:
                continue
            if tls:
                return tls
        return None

    def _next_edge_hint_from_route(self, ev_id: str) -> Optional[str]:
        if not bool(getattr(self.cfg, "fed_enable_nextedge_hint", True)):
            return None
        route_edges = self._route_edges_hint(str(ev_id))
        if not route_edges:
            return None
        ridx = -1
        if traci is not None:
            try:
                ridx = int(traci.vehicle.getRouteIndex(str(ev_id)))
            except Exception:
                ridx = -1
        if ridx < 0:
            cur_edge = ""
            ev = self.active_ev
            if ev is not None and str(ev.ev_id) == str(ev_id):
                cur_edge = str(getattr(ev, "in_edge_id", "") or "")
            if cur_edge and cur_edge in route_edges:
                ridx = route_edges.index(cur_edge)
        # Search a few edges ahead for the first mapped outgoing neighbor edge.
        start = max(0, ridx + 1) if ridx >= 0 else 0
        end = min(len(route_edges), start + 8)
        for e in route_edges[start:end]:
            e_str = str(e)
            if e_str in self.out_edge_to_neighbor:
                return str(self.out_edge_to_neighbor[e_str])
        # If no direct hit in forward slice, scan entire hinted route as fallback.
        for e in route_edges:
            e_str = str(e)
            if e_str in self.out_edge_to_neighbor:
                return str(self.out_edge_to_neighbor[e_str])
        return None

    def _next_route_edge_id(self, ev_id: str) -> Optional[str]:
        route_edges = self._route_edges_hint(str(ev_id))
        if not route_edges:
            return None
        ridx = -1
        if traci is not None:
            try:
                ridx = int(traci.vehicle.getRouteIndex(str(ev_id)))
            except Exception:
                ridx = -1
        if ridx < 0:
            cur_edge = ""
            ev = self.active_ev
            if ev is not None and str(ev.ev_id) == str(ev_id):
                cur_edge = str(getattr(ev, "in_edge_id", "") or "")
            if cur_edge and cur_edge in route_edges:
                ridx = route_edges.index(cur_edge)
        start = max(0, ridx + 1) if ridx >= 0 else 0
        if start < len(route_edges):
            return str(route_edges[start])
        return None

    def _tail_route_hint_neighbor(self, ev_id: str) -> Optional[str]:
        """
        Existing route-tail mapping heuristic (kept for backward compatibility).
        """
        route_edges = self._route_edges_hint(str(ev_id))
        if not route_edges:
            return None
        cur_edge = ""
        if traci is not None:
            try:
                cur_edge = str(traci.vehicle.getRoadID(str(ev_id)))
            except Exception:
                cur_edge = ""
        if cur_edge and cur_edge in route_edges:
            i = route_edges.index(cur_edge)
            tail = route_edges[i + 1 : i + 8]
        else:
            tail = route_edges[:8]
        for e in tail:
            e_str = str(e)
            if e_str in self.out_edge_to_neighbor:
                return str(self.out_edge_to_neighbor[e_str])
        return None

    def _strong_route_hint_neighbor(self, ev_id: str) -> Optional[str]:
        """
        Deterministic preferred downstream neighbor when explicit route evidence exists.
        Priority: route_intersections -> nextTLS -> next edge.
        """
        hints = self._route_intersection_hints(ev_id)
        if hints:
            return str(hints[0])
        tls_hint = self._next_tls_hint_from_traci(ev_id)
        if tls_hint is not None:
            return str(tls_hint)
        edge_hint = self._next_edge_hint_from_route(ev_id)
        if edge_hint is not None:
            return str(edge_hint)
        return None


    def _predict_next_tls_distribution(self, ev_id: str) -> Dict[str, float]:
        """
        Returns p(next_tls) using layered evidence:
          1) EV route_intersections hints (if provided)
          2) traci.vehicle.getNextTLS() immediate next controlled junction
          3) next route edge mapped to neighbor
          4) route-tail fallback
          5) uniform prior
        """
        neighs = [
            str(n)
            for n in list(self.neighbor_map.keys())
            if str(n) and str(n) != str(self.cfg.tls_id)
        ]
        candidate_tls: List[str] = list(neighs)
        if not candidate_tls:
            # Fallback for complex nodes where controlled-link based neighbor extraction is empty
            # (e.g., roundabout-like or heavily internalized junctions).
            for tls in list(self._route_intersection_hints(ev_id) or []):
                tls_s = str(tls)
                if tls_s and tls_s != str(self.cfg.tls_id) and tls_s not in candidate_tls:
                    candidate_tls.append(tls_s)
            tls_hint = self._next_tls_hint_from_traci(ev_id)
            if tls_hint is not None:
                tls_s = str(tls_hint)
                if tls_s and tls_s != str(self.cfg.tls_id) and tls_s not in candidate_tls:
                    candidate_tls.append(tls_s)
            for tls in list(self._infer_route_intersections_from_traci(ev_id, max_count=4) or []):
                tls_s = str(tls)
                if tls_s and tls_s != str(self.cfg.tls_id) and tls_s not in candidate_tls:
                    candidate_tls.append(tls_s)
            strong_hint = self._strong_route_hint_neighbor(ev_id)
            if strong_hint is not None:
                tls_s = str(strong_hint)
                if tls_s and tls_s != str(self.cfg.tls_id) and tls_s not in candidate_tls:
                    candidate_tls.append(tls_s)
        if not candidate_tls:
            return {}
        scores: Dict[str, float] = {
            n: float(getattr(self, "fed_uniform_prior_weight", 1.0))
            for n in candidate_tls
        }

        # 1) Explicit EV request hint: route_intersections
        hints = self._route_intersection_hints(ev_id)
        if hints:
            decay = max(0.0, min(1.0, float(getattr(self, "fed_route_hint_decay", 0.65))))
            w = max(0.0, float(getattr(self, "fed_route_hint_weight", 6.0)))
            for rank, tls in enumerate(hints):
                if tls not in scores:
                    scores[tls] = float(getattr(self, "fed_uniform_prior_weight", 1.0))
                scores[tls] += w * (decay ** rank)

        # 2) Direct next TLS hint from SUMO
        tls_hint = self._next_tls_hint_from_traci(ev_id)
        if tls_hint is not None:
            if tls_hint not in scores:
                scores[tls_hint] = float(getattr(self, "fed_uniform_prior_weight", 1.0))
            scores[tls_hint] += max(0.0, float(getattr(self, "fed_nexttls_weight", 10.0)))

        # 3) Next edge mapped to a neighbor
        edge_hint = self._next_edge_hint_from_route(ev_id)
        if edge_hint is not None:
            if edge_hint not in scores:
                scores[edge_hint] = float(getattr(self, "fed_uniform_prior_weight", 1.0))
            scores[edge_hint] += max(0.0, float(getattr(self, "fed_nextedge_weight", 8.0)))

        # 4) Tail-route fallback
        tail_hint = self._tail_route_hint_neighbor(ev_id)
        if tail_hint is not None:
            if tail_hint not in scores:
                scores[tail_hint] = float(getattr(self, "fed_uniform_prior_weight", 1.0))
            scores[tail_hint] += max(0.0, float(getattr(self, "fed_tailroute_weight", 4.0)))

        s = float(sum(max(0.0, v) for v in scores.values()))
        if s <= 1e-9:
            p = 1.0 / max(1, len(scores))
            probs = {n: p for n in scores.keys()}
        else:
            probs = {k: float(max(0.0, v) / s) for k, v in scores.items()}

        # Optional route-priority override: make route-indicated neighbor dominant.
        if bool(getattr(self, "fed_force_route_hint_top1", False)):
            strong = self._strong_route_hint_neighbor(ev_id)
            floor = max(0.0, min(0.999, float(getattr(self, "fed_route_hint_prob_floor", 0.80))))
            if strong is not None and str(strong) in probs:
                strong = str(strong)
                p_strong = float(probs.get(strong, 0.0))
                if p_strong < floor:
                    other_sum = float(sum(v for k, v in probs.items() if k != strong))
                    if other_sum <= 1e-9:
                        probs = {k: (1.0 if k == strong else 0.0) for k in probs.keys()}
                    else:
                        scale = float((1.0 - floor) / other_sum)
                        probs = {
                            k: (floor if k == strong else float(v) * scale)
                            for k, v in probs.items()
                        }
                self._fed_dbg(
                    f"route_override ev={ev_id} strong={strong} floor={floor:.2f} "
                    f"p={float(probs.get(strong, 0.0)):.3f}"
                )
        return probs


    def rank_next_hop_candidates(self, ev_id: str, sim_time: float, max_hops: int = 1) -> List[Tuple[str, float, float]]:
        """
        Returns [(neighbor_tls, probability, eta_center_sec)] sorted by probability.
        max_hops kept for compatibility; current implementation is 1-hop.
        """
        probs = self._predict_next_tls_distribution(ev_id)
        if not probs:
            return []

        # ETA to current stopline
        if self.active_ev is not None and self.active_ev.ev_id == ev_id:
            eta_here = self._estimate_arrival_time(sim_time, self.active_ev)
        else:
            eta_here = float(sim_time) + 2.5

        out = []

        nexttls_dist_m: Dict[str, float] = {}
        if traci is not None:
            try:
                nxt = list(traci.vehicle.getNextTLS(str(ev_id)) or [])
            except Exception:
                nxt = []
            for item in nxt:
                try:
                    tls_i = str(item[0])
                except Exception:
                    continue
                dist_i = None
                try:
                    if len(item) >= 3:
                        dist_i = float(item[2])
                except Exception:
                    dist_i = None
                if tls_i and dist_i is not None and dist_i >= 0.0:
                    prev = nexttls_dist_m.get(tls_i)
                    if prev is None or dist_i < prev:
                        nexttls_dist_m[tls_i] = float(dist_i)

        v_ref = 8.0
        if self.active_ev is not None and str(self.active_ev.ev_id) == str(ev_id):
            try:
                v_ref = max(1.0, float(getattr(self.active_ev, "speed_mps", v_ref) or v_ref))
            except Exception:
                pass
        elif traci is not None:
            try:
                v_ref = max(1.0, float(traci.vehicle.getSpeed(str(ev_id))))
            except Exception:
                pass

        eta_fallback_non_neighbor = float(getattr(self.cfg, "fed_non_neighbor_eta_sec", 12.0))

        for nb_tls, p in probs.items():
            if str(nb_tls) == str(self.cfg.tls_id):
                continue
            ninfo = self.neighbor_map.get(nb_tls)
            if ninfo is not None:
                t_edge = self._estimate_edge_travel_sec(ninfo.via_out_edge)
                eta_nb = eta_here + t_edge
            else:
                d_next = nexttls_dist_m.get(str(nb_tls))
                if d_next is not None:
                    eta_nb = float(sim_time) + (max(0.0, float(d_next)) / max(1.0, float(v_ref)))
                else:
                    eta_nb = eta_here + max(1.0, eta_fallback_non_neighbor)
            out.append((str(nb_tls), float(p), float(eta_nb)))

        out.sort(key=lambda x: (-x[1], x[2]))
        self._fed_dbg(
            f"rank ev={ev_id} hints={self._route_intersection_hints(ev_id)} "
            f"strong={self._strong_route_hint_neighbor(ev_id)} "
            f"cands={[ (a, round(b,3), round(c,2)) for a,b,c in out[:4] ]}"
        )
        return out


    def make_reservation_req(
        self,
        to_tls: str,
        ev_id: str,
        sim_time: float,
        eta: float,
        confidence: float,
        mode: str = "soft",
        eta_shift_sec: float = 0.0,
        corridor_guidance: Optional[dict] = None,
    ) -> dict:
        hard = (mode == "hard")
        half_w = self.fed_eta_half_window_hard if hard else self.fed_eta_half_window_soft
        eta_center = float(eta) + max(0.0, float(eta_shift_sec))
        eta_start, eta_end = self._norm_eta_window(eta_center - half_w, eta_center + half_w)

        ninfo = self.neighbor_map.get(to_tls)
        in_edge_id = ninfo.neighbor_in_edge if ninfo else None
        out_edge_id = ninfo.via_out_edge if ninfo else None
        route_hints = self._route_intersection_hints(ev_id)
        route_edges_hint = self._route_edges_hint(ev_id)
        preferred_next_tls = self._strong_route_hint_neighbor(ev_id)
        next_edge_id = self._next_route_edge_id(ev_id)
        if preferred_next_tls is None and route_hints:
            preferred_next_tls = str(route_hints[0])
        if preferred_next_tls is None and str(to_tls) in self.neighbor_map:
            preferred_next_tls = str(to_tls)
        if next_edge_id is None and ninfo is not None and str(getattr(ninfo, "via_out_edge", "") or ""):
            next_edge_id = str(ninfo.via_out_edge)
        if next_edge_id is None and route_edges_hint:
            next_edge_id = str(route_edges_hint[0])
        cur_edge_id: Optional[str] = None
        if traci is not None:
            try:
                cur_edge_id = str(traci.vehicle.getRoadID(str(ev_id)))
            except Exception:
                cur_edge_id = None

        req_id = f"{self.cfg.tls_id}:{ev_id}:{int(sim_time*10)}:{mode}:{to_tls}"
        route_token = f"{self.cfg.tls_id}|{ev_id}|{to_tls}|{int(sim_time)}"
        assoc_meta = dict(((corridor_guidance or {}).get("assoc") or {}))
        ttl_base = float(self.fed_hard_ttl_sec if hard else self.fed_soft_ttl_sec)
        ttl_eta_cover = max(0.5, float(eta_end - float(sim_time)) + float(getattr(self.cfg, "fed_reservation_ttl_buffer_sec", 2.0)))
        ttl_sec = max(float(ttl_base), float(ttl_eta_cover))
        # Source-side processing only (local CPU); transport is intentionally excluded.
        source_local_compute_ms = float(max(0.0, self._last_tick_compute_ms) + max(0.0, self._last_refine_compute_ms))
        source_fnm_integration_ms = 0.0

        return {
            "req_id": req_id,
            "ev_id": ev_id,
            "from_tls": self.cfg.tls_id,
            "to_tls": to_tls,
            "in_edge_id": in_edge_id,
            "out_edge_id": out_edge_id,
            "current_edge_id": cur_edge_id,
            "next_edge_id": next_edge_id,
            "eta_start": float(eta_start),
            "eta_end": float(eta_end),
            "confidence": float(confidence),
            "soft": not hard,
            "hard": hard,
            "mode": mode,
            "ttl_sec": float(ttl_sec),
            "route_token": route_token,
            "route_intersections": list(route_hints),
            "route_veh": list(route_edges_hint[:64]),
            "preferred_next_tls": preferred_next_tls,
            "corridor_eta_shift_sec": float(max(0.0, eta_shift_sec)),
            "corridor_assoc_state": assoc_meta.get("assoc_state"),
            "source_local_compute_ms": float(source_local_compute_ms),
            "source_fnm_integration_ms": float(source_fnm_integration_ms),
            "source_compute_split": {
                "tick_ms": float(max(0.0, self._last_tick_compute_ms)),
                "refine_ms": float(max(0.0, self._last_refine_compute_ms)),
            },
        }


    def _check_local_window_feasibility(
        self,
        eta_start: float,
        eta_end: float,
        in_edge_id: Optional[str],
        hard: bool,
    ) -> Tuple[bool, Dict[str, float]]:
        """
        Lightweight feasibility:
        - soft: permissive warm-accept
        - hard: requires overlap with predicted inbound phase window
        """
        eta_start, eta_end = self._norm_eta_window(eta_start, eta_end)
        now = self._now()
        diag: Dict[str, float] = {
            "overlap_sec": -1.0,
            "gap_sec": -1.0,
            "window_start": -1.0,
            "window_end": -1.0,
            "min_overlap_sec": -1.0,
            "grace_sec": -1.0,
        }

        if in_edge_id is None:
            return (not hard), diag

        phase_idx = self._inbound_edge_to_phase.get(in_edge_id)
        if phase_idx is None:
            return (not hard), diag

        mid = 0.5 * (eta_start + eta_end)
        win = self._predict_phase_window_containing(now, phase_idx, mid)
        if win is None:
            return (not hard), diag

        ws, we = float(win[0]), float(win[1])
        overlap = max(0.0, min(we, eta_end) - max(ws, eta_start))
        if we < eta_start:
            gap = float(eta_start - we)
        elif ws > eta_end:
            gap = float(ws - eta_end)
        else:
            gap = 0.0
        diag.update(
            {
                "overlap_sec": float(overlap),
                "gap_sec": float(gap),
                "window_start": float(ws),
                "window_end": float(we),
            }
        )

        if hard:
            min_ov = max(0.0, float(getattr(self.cfg, "fed_min_hard_overlap_sec", 0.50)))
            grace = max(0.0, float(getattr(self.cfg, "fed_hard_overlap_grace_sec", 0.80)))
            diag["min_overlap_sec"] = float(min_ov)
            diag["grace_sec"] = float(grace)
            if overlap >= min_ov:
                return True, diag
            # Near-miss fallback: accept if the predicted window is only slightly
            # shifted relative to requested ETA bounds (bounded by grace).
            return (gap <= grace), diag
        soft_grace = max(0.0, float(getattr(self.cfg, "fed_soft_window_grace_sec", 6.0)))
        diag["grace_sec"] = float(soft_grace)
        return ((overlap > 0.0) or (ws <= eta_end + soft_grace)), diag

    def _spillback_risk_for_edge(self, edge_id: Optional[str]) -> float:
        if traci is None or not edge_id or str(edge_id).startswith(":"):
            return 0.0
        try:
            nlanes = max(1, int(traci.edge.getLaneNumber(str(edge_id))))
        except Exception:
            nlanes = 1

        risks: List[float] = []
        for i in range(int(nlanes)):
            lid = f"{edge_id}_{i}"
            try:
                q = float(traci.lane.getLastStepHaltingNumber(lid))
                lane_len = max(1.0, float(traci.lane.getLength(lid)))
                vids = list(traci.lane.getLastStepVehicleIDs(lid))
                if vids:
                    lengths = []
                    gaps = []
                    for vid in vids[:10]:
                        try:
                            lengths.append(float(traci.vehicle.getLength(vid)))
                            gaps.append(float(traci.vehicle.getMinGap(vid)))
                        except Exception:
                            pass
                    vlen = (sum(lengths) / len(lengths)) if lengths else 5.0
                    vgap = (sum(gaps) / len(gaps)) if gaps else 2.5
                else:
                    vlen, vgap = 5.0, 2.5
                cap = max(1.0, lane_len / max(1.0, vlen + vgap))
                risks.append(max(0.0, (q - cap) / cap))
            except Exception:
                continue
        return float(max(risks)) if risks else 0.0

    def _downstream_readiness_snapshot(
        self,
        eta_start: float,
        eta_end: float,
        in_edge_id: Optional[str],
    ) -> dict:
        eta_start, eta_end = self._norm_eta_window(eta_start, eta_end)
        q_margin = 0.0
        q_req = 0.0
        q_avail = 0.0
        spill = self._spillback_risk_for_edge(in_edge_id)
        now = float(self._now())

        if in_edge_id:
            try:
                ph = self._inbound_edge_to_phase.get(str(in_edge_id))
                mid = 0.5 * (float(eta_start) + float(eta_end))
                win = None
                if ph is not None:
                    win = self._predict_phase_window_containing(now, int(ph), mid)

                use_imp = bool(getattr(self.cfg, "fed_readiness_use_improved_queue", True)) and bool(
                    getattr(self.cfg, "queue_metrics_enable_improved", True)
                )
                if use_imp and win is not None:
                    ws, we = float(win[0]), float(win[1])
                    qc = self._queue_clearing_metrics_improved(
                        edge_id=str(in_edge_id),
                        ev_id=None,
                        sim_time=now,
                        t_i=float(mid),
                        green_window=(ws, we),
                        arrival_window=(float(eta_start), float(eta_end)),
                    )
                    q_req = float(qc.get("delta_w_sec", 0.0))
                    q_avail = float(qc.get("avail_pre_green_sec", 0.0))
                else:
                    # Legacy fallback for robustness when improved context is unavailable.
                    N, A, S, Q, _ = self._queue_clearing_metrics_for_edge(str(in_edge_id), ev_id=None)
                    q_req = float(Q) + float(getattr(self.cfg, "T_lost_sec", 5.0)) + float(getattr(self.cfg, "YT_sec", 5.0))
                    if win is not None:
                        ws, we = float(win[0]), float(win[1])
                        q_avail = max(0.0, min(we, float(eta_start)) - ws)
                q_margin = float(q_avail - q_req)
            except Exception:
                pass

        readiness = max(0.0, 1.0 - max(0.0, -q_margin) / max(1.0, float(getattr(self.cfg, "robust_scale_fed_down_queue_sec", 20.0))))
        readiness *= max(0.0, 1.0 - float(spill))
        shift_sec = max(0.0, -float(q_margin)) + max(0.0, float(spill) - 0.5) * 2.0

        return {
            "queue_clear_margin_sec": float(q_margin),
            "queue_required_clear_sec": float(q_req),
            "queue_available_pre_arrival_green_sec": float(q_avail),
            "spillback_risk": float(spill),
            "readiness_score": float(max(0.0, min(1.0, readiness))),
            "suggested_eta_shift_sec": float(shift_sec),
        }

    def _emit_queue_snapshot(self, sim_time: float, ev: Optional[EvRequest], t_i: Optional[float]) -> None:
        if not bool(getattr(self.cfg, "queue_snapshot_emit_enable", True)):
            return
        if ev is None:
            return
        in_edge = str(getattr(ev, "in_edge_id", "") or "")
        if (not in_edge) or in_edge.startswith(":"):
            return

        q_len = 0.0
        q_clear = 0.0
        try:
            if traci is not None:
                q_len = float(traci.edge.getLastStepHaltingNumber(in_edge))
        except Exception:
            q_len = 0.0
        try:
            q_clear = float(self._estimate_queue_clear_time(in_edge))
        except Exception:
            q_clear = 0.0

        eta_mid = float(t_i if t_i is not None else float(sim_time))
        try:
            delta = float(getattr(ev, "delta_sec", 2.0))
        except Exception:
            delta = 2.0
        eta_start = float(eta_mid - delta)
        eta_end = float(eta_mid + delta)
        ready = self._downstream_readiness_snapshot(
            eta_start=float(eta_start),
            eta_end=float(eta_end),
            in_edge_id=str(in_edge),
        )
        spill = float(ready.get("spillback_risk", 0.0) or 0.0)
        spill_thr = max(0.0, float(getattr(self.cfg, "queue_snapshot_spillback_threshold", 0.2)))
        self._fed_evt(
            "coord.queue.snapshot",
            sim_time=float(sim_time),
            mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
            stage=str(self.stage),
            ev_id=str(getattr(ev, "ev_id", "") or ""),
            in_edge_id=str(in_edge),
            eta_mid_sec=float(eta_mid),
            eta_start_sec=float(eta_start),
            eta_end_sec=float(eta_end),
            ev_distance_m=float(getattr(ev, "distance_to_intersection_m", -1.0)),
            ev_speed_mps=float(getattr(ev, "speed_mps", -1.0)),
            queue_len_est_veh=float(q_len),
            queue_clear_time_sec=float(q_clear),
            queue_margin_sec=float(ready.get("queue_clear_margin_sec", 0.0) or 0.0),
            spillback_risk=float(spill),
            spillback_active=int(1 if spill >= spill_thr else 0),
            spillback_threshold=float(spill_thr),
            readiness_score=float(ready.get("readiness_score", 0.0) or 0.0),
        )


    def on_reservation_req(self, msg: dict) -> dict:
        t_req_proc0 = time.perf_counter()
        now = self._now()
        ev_id = str(msg.get("ev_id", ""))
        req_id = str(msg.get("req_id", f"req-{int(now*1000)}"))

        eta_start, eta_end = self._norm_eta_window(
            float(msg.get("eta_start", now)),
            float(msg.get("eta_end", now + 3.0)),
        )
        hard = bool(msg.get("hard", False))
        soft = bool(msg.get("soft", not hard))
        conf = float(msg.get("confidence", 0.0))
        in_edge_id = msg.get("in_edge_id", None)
        if in_edge_id is not None:
            in_edge_id = str(in_edge_id)
        if (not in_edge_id) or (self._inbound_edge_to_phase.get(str(in_edge_id)) is None):
            inferred_in_edge: Optional[str] = None
            route_edges_hint = [str(x) for x in list(msg.get("route_veh", []) or []) if str(x)]
            for e in route_edges_hint:
                if self._inbound_edge_to_phase.get(str(e)) is not None:
                    inferred_in_edge = str(e)
                    break
            if inferred_in_edge is None:
                for probe in [msg.get("next_edge_id"), msg.get("current_edge_id"), msg.get("in_edge_id")]:
                    if probe is None:
                        continue
                    probe_s = str(probe)
                    if self._inbound_edge_to_phase.get(probe_s) is not None:
                        inferred_in_edge = probe_s
                        break
            if inferred_in_edge is not None:
                in_edge_id = str(inferred_in_edge)
                self._fed_dbg(
                    f"evt=REQ_IN_EDGE_INFER req_id={req_id} ev={ev_id} inferred_in_edge={in_edge_id} "
                    f"from={msg.get('from_tls')} to={self.cfg.tls_id}"
                )

        ttl = float(msg.get("ttl_sec", self.fed_hard_ttl_sec if hard else self.fed_soft_ttl_sec))
        phase_state_age_ms = 0.0
        if self._last_tls_signal_change_sim_time is not None:
            phase_state_age_ms = max(0.0, (float(now) - float(self._last_tls_signal_change_sim_time)) * 1000.0)

        self._fed_dbg(
            f"evt=REQ_IN req_id={req_id} ev={ev_id} from={msg.get('from_tls')} to={self.cfg.tls_id} "
            f"mode={'hard' if hard else 'soft'} eta=({eta_start:.2f},{eta_end:.2f}) in_edge={in_edge_id}"
        )
        self._fed_evt(
            "coord.reservation.req_in",
            req_id=req_id,
            ev_id=str(ev_id),
            from_tls=str(msg.get("from_tls", "") or ""),
            mode=("hard" if hard else "soft"),
            eta_start=float(eta_start),
            eta_end=float(eta_end),
            in_edge_id=(str(in_edge_id) if in_edge_id is not None else ""),
        )

        feasible, feas_diag = self._check_local_window_feasibility(
            eta_start=eta_start,
            eta_end=eta_end,
            in_edge_id=in_edge_id,
            hard=hard,
        )
        readiness = self._downstream_readiness_snapshot(
            eta_start=eta_start,
            eta_end=eta_end,
            in_edge_id=in_edge_id,
        )

        reject_reason = ""
        if feasible and hard:
            if float(readiness.get("queue_clear_margin_sec", 0.0)) < float(getattr(self.cfg, "fed_hard_min_queue_margin_sec", -0.5)):
                feasible = False
                reject_reason = "downstream_queue_not_clearing"
            if feasible and float(readiness.get("spillback_risk", 0.0)) > float(getattr(self.cfg, "fed_hard_max_spillback_risk", 0.85)):
                feasible = False
                reject_reason = "downstream_spillback_risk"
        elif (not feasible) and hard:
            # Adaptive near-miss rescue for hard windows:
            # only when readiness is acceptable and mismatch is small.
            if bool(getattr(self.cfg, "fed_hard_window_adaptive_relax_enable", False)):
                conf_min = float(getattr(self.cfg, "fed_hard_window_adaptive_conf_min", 0.65))
                rd_min = float(getattr(self.cfg, "fed_hard_window_adaptive_readiness_min", 0.55))
                spill_max = float(getattr(self.cfg, "fed_hard_window_adaptive_spillback_max", 0.80))
                q_margin_min = float(getattr(self.cfg, "fed_hard_window_adaptive_queue_margin_min_sec", -1.5))
                extra_grace = max(0.0, float(getattr(self.cfg, "fed_hard_window_adaptive_extra_grace_sec", 0.60)))
                base_grace = float(feas_diag.get("grace_sec", 0.0) or 0.0)
                gap_sec = max(0.0, float(feas_diag.get("gap_sec", 1e9) or 1e9))
                adaptive_grace = float(base_grace + extra_grace)
                q_margin = float(readiness.get("queue_clear_margin_sec", 0.0) or 0.0)
                spill = float(readiness.get("spillback_risk", 0.0) or 0.0)
                rd = float(readiness.get("readiness_score", 0.0) or 0.0)
                if (
                    float(conf) >= conf_min
                    and rd >= rd_min
                    and spill <= spill_max
                    and q_margin >= q_margin_min
                    and gap_sec <= adaptive_grace
                ):
                    feasible = True
                    self._fed_evt(
                        "coord.reservation.req_decision_relax",
                        req_id=str(req_id),
                        ev_id=str(ev_id),
                        from_tls=str(msg.get("from_tls", "") or ""),
                        reason="window_near_miss_adaptive_relax",
                        confidence=float(conf),
                        overlap_sec=float(feas_diag.get("overlap_sec", -1.0)),
                        gap_sec=float(gap_sec),
                        base_grace_sec=float(base_grace),
                        adaptive_grace_sec=float(adaptive_grace),
                        q_margin_sec=float(q_margin),
                        spillback_risk=float(spill),
                        readiness_score=float(rd),
                    )

        rs = ReservationState(
            req_id=req_id,
            ev_id=ev_id,
            from_tls=str(msg.get("from_tls", "")),
            to_tls=self.cfg.tls_id,
            in_edge_id=in_edge_id,
            eta_start=eta_start,
            eta_end=eta_end,
            soft=soft,
            hard=hard,
            confidence=conf,
            ts_created=now,
            ts_expire=now + max(0.5, ttl),
            route_token=msg.get("route_token"),
            route_intersections_hint=list(msg.get("route_intersections", []) or []),
            route_veh_hint=list(msg.get("route_veh", []) or []),
            preferred_next_tls=str(msg.get("preferred_next_tls", "")) or None,
            status="ACCEPTED" if feasible else "REJECTED",
            reason="" if feasible else (reject_reason or "window_not_feasible"),
            local_queue_margin_sec=float(readiness.get("queue_clear_margin_sec", 0.0)),
            local_spillback_risk=float(readiness.get("spillback_risk", 0.0)),
            local_readiness_score=float(readiness.get("readiness_score", 0.0)),
        )

        if feasible:
            self.active_reservations[req_id] = rs
        self._fed_dbg(
            f"recv_req ev={ev_id} from={msg.get('from_tls')} mode={'hard' if hard else 'soft'} "
            f"eta=({eta_start:.2f},{eta_end:.2f}) in_edge={in_edge_id} "
            f"decision={rs.status} reason={rs.reason or '-'} "
            f"q_margin={float(readiness.get('queue_clear_margin_sec', 0.0)):.2f} "
            f"spill={float(readiness.get('spillback_risk', 0.0)):.2f}"
        )
        self._fed_dbg(
            f"evt=REQ_DECISION req_id={req_id} status={rs.status} reason={rs.reason or '-'} "
            f"q_margin={float(readiness.get('queue_clear_margin_sec', 0.0)):.2f} "
            f"spill={float(readiness.get('spillback_risk', 0.0)):.2f} active_res={len(self.active_reservations)}"
        )
        self._fed_evt(
            "coord.reservation.req_decision",
            req_id=req_id,
            ev_id=str(ev_id),
            from_tls=str(msg.get("from_tls", "") or ""),
            status=str(rs.status),
            reason=str(rs.reason or ""),
            q_margin_sec=float(readiness.get("queue_clear_margin_sec", 0.0) or 0.0),
            spillback_risk=float(readiness.get("spillback_risk", 0.0) or 0.0),
            readiness_score=float(readiness.get("readiness_score", 0.0) or 0.0),
            overlap_sec=float(feas_diag.get("overlap_sec", -1.0) or -1.0),
            gap_sec=float(feas_diag.get("gap_sec", -1.0) or -1.0),
            window_start=float(feas_diag.get("window_start", -1.0) or -1.0),
            window_end=float(feas_diag.get("window_end", -1.0) or -1.0),
            min_overlap_sec=float(feas_diag.get("min_overlap_sec", -1.0) or -1.0),
            grace_sec=float(feas_diag.get("grace_sec", -1.0) or -1.0),
            active_reservations=int(len(self.active_reservations)),
        )

        responder_processing_ms = max(0.0, (time.perf_counter() - t_req_proc0) * 1000.0)
        return {
            "req_id": req_id,
            "ev_id": ev_id,
            "from_tls": self.cfg.tls_id,                 # responder
            "to_tls": str(msg.get("from_tls", "")),      # upstream target
            "status": rs.status,
            "reason": rs.reason,
            "mode": "hard" if hard else "soft",
            "sim_time": now,
            "req_eta_start": float(eta_start),
            "req_eta_end": float(eta_end),
            "downstream_queue_margin_sec": float(readiness.get("queue_clear_margin_sec", 0.0)),
            "downstream_spillback_risk": float(readiness.get("spillback_risk", 0.0)),
            "downstream_readiness_score": float(readiness.get("readiness_score", 0.0)),
            "downstream_suggested_eta_shift_sec": float(readiness.get("suggested_eta_shift_sec", 0.0)),
            "responder_phase_state_age_ms": float(phase_state_age_ms),
            "responder_processing_ms": float(responder_processing_ms),
        }


    def on_reservation_resp(self, msg: dict) -> None:
        req_id = str(msg.get("req_id", ""))
        if not req_id:
            return
        self.resp_cache[req_id] = {
            "status": str(msg.get("status", "UNKNOWN")),
            "reason": str(msg.get("reason", "")),
            "mode": str(msg.get("mode", "")),
            "ev_id": str(msg.get("ev_id", "")),
            "responder_tls": str(msg.get("from_tls", "")),
            "req_eta_start": float(msg.get("req_eta_start", 0.0)),
            "req_eta_end": float(msg.get("req_eta_end", 0.0)),
            "downstream_queue_margin_sec": float(msg.get("downstream_queue_margin_sec", 0.0)),
            "downstream_spillback_risk": float(msg.get("downstream_spillback_risk", 0.0)),
            "downstream_readiness_score": float(msg.get("downstream_readiness_score", 0.0)),
            "downstream_suggested_eta_shift_sec": float(msg.get("downstream_suggested_eta_shift_sec", 0.0)),
            "responder_phase_state_age_ms": float(msg.get("responder_phase_state_age_ms", 0.0)),
            "ts": self._now(),
        }
        self._fed_dbg(
            f"evt=RESP_IN req_id={req_id} ev={msg.get('ev_id')} from={msg.get('from_tls')} "
            f"status={msg.get('status')} reason={msg.get('reason', '-')}"
        )
        self._fed_dbg(
            f"recv_resp ev={msg.get('ev_id')} from={msg.get('from_tls')} "
            f"status={msg.get('status')} reason={msg.get('reason', '-')}"
        )
        now = float(self._now())
        now_wall = float(time.time())
        self._fed_evt(
            "coord.reservation.resp_in",
            req_id=req_id,
            ev_id=str(msg.get("ev_id", "") or ""),
            from_tls=str(msg.get("from_tls", "") or ""),
            status=str(msg.get("status", "") or ""),
            reason=str(msg.get("reason", "") or ""),
            mode=str(msg.get("mode", "") or ""),
            downstream_queue_margin_sec=float(msg.get("downstream_queue_margin_sec", 0.0) or 0.0),
            downstream_spillback_risk=float(msg.get("downstream_spillback_risk", 0.0) or 0.0),
            responder_phase_state_age_ms=float(msg.get("responder_phase_state_age_ms", 0.0) or 0.0),
            pending_req_n=int(len(self._fed_req_sent_ts)),
        )
        sent_ts = self._fed_req_sent_ts.pop(req_id, None)
        sent_clk = self._fed_req_sent_clock.pop(req_id, None)
        if sent_ts is not None:
            lat_sim_ms = float(max(0.0, now - float(sent_ts)) * 1000.0)
            lat_wall_ms = None
            source_local_compute_ms = 0.0
            source_fnm_integration_ms = 0.0
            if isinstance(sent_clk, dict):
                try:
                    sent_wall = float(sent_clk.get("wall", 0.0) or 0.0)
                    if sent_wall > 0.0:
                        lat_wall_ms = float(max(0.0, now_wall - sent_wall) * 1000.0)
                    source_local_compute_ms = float(
                        sent_clk.get("source_local_compute_ms", sent_clk.get("local_compute_ms", 0.0)) or 0.0
                    )
                    source_fnm_integration_ms = float(
                        sent_clk.get("source_fnm_integration_ms", sent_clk.get("fnm_integration_ms", 0.0)) or 0.0
                    )
                except Exception:
                    lat_wall_ms = None
            responder_processing_ms = float(msg.get("responder_processing_ms", 0.0) or 0.0)
            network_wait_ms = None
            if lat_wall_ms is not None:
                known_processing = max(0.0, source_local_compute_ms) + max(0.0, source_fnm_integration_ms) + max(
                    0.0, responder_processing_ms
                )
                network_wait_ms = float(max(0.0, float(lat_wall_ms) - known_processing))
            self._fed_evt(
                "coord.reservation.req_resp_e2e",
                req_id=req_id,
                ev_id=str(msg.get("ev_id", "") or ""),
                from_tls=str(msg.get("from_tls", "") or ""),
                status=str(msg.get("status", "") or ""),
                mode=str(msg.get("mode", "") or ""),
                latency_ms=float(lat_sim_ms),
                latency_sim_ms=float(lat_sim_ms),
                latency_wall_ms=float(lat_wall_ms) if lat_wall_ms is not None else None,
                source_local_compute_ms=float(source_local_compute_ms),
                source_fnm_integration_ms=float(source_fnm_integration_ms),
                responder_processing_ms=float(responder_processing_ms),
                network_wait_ms=(float(network_wait_ms) if network_wait_ms is not None else None),
                responder_phase_state_age_ms=float(msg.get("responder_phase_state_age_ms", 0.0) or 0.0),
                pending_req_n=int(len(self._fed_req_sent_ts)),
            )

    def _estimate_edge_speed_limit_mps(self, edge_id: Optional[str]) -> float:
        if traci is None or not edge_id or str(edge_id).startswith(":"):
            return 13.9
        try:
            return max(1.0, float(traci.edge.getSpeed(str(edge_id))))
        except Exception:
            return 13.9

    def _select_warmup_reservation(self, sim_time: float) -> Optional[ReservationState]:
        horizon_default = float(getattr(self.cfg, "fed_warm_horizon_sec", 25.0))
        hard_only_default = bool(getattr(self.cfg, "fed_warmup_hard_only", False))
        best: Optional[ReservationState] = None
        best_key: Optional[Tuple[float, float, float]] = None

        for rid, rs in list(self.active_reservations.items()):
            if float(sim_time) > float(rs.ts_expire):
                self.active_reservations.pop(rid, None)
                continue
            if str(rs.status).upper() != "ACCEPTED":
                continue
            advice = self._latest_corridor_advice(str(rs.ev_id), target_tls=str(self.cfg.tls_id), max_age_sec=5.0)
            warmup_guidance = dict((advice or {}).get("warmup_guidance", {}) or {})
            horizon = float(warmup_guidance.get("lead_horizon_sec", horizon_default))
            hard_only = bool(warmup_guidance.get("hard_only", hard_only_default))
            if hard_only and not bool(rs.hard):
                continue
            if rs.in_edge_id is None:
                continue
            if self._inbound_edge_to_phase.get(str(rs.in_edge_id)) is None:
                continue

            eta_mid = 0.5 * (float(rs.eta_start) + float(rs.eta_end))
            lead = float(eta_mid - float(sim_time))
            if lead < -0.5 or lead > horizon:
                continue

            prio = 0.0 if bool(rs.hard) else 1.0
            urgency = max(0.0, -float(getattr(rs, "local_queue_margin_sec", 0.0))) + max(0.0, float(getattr(rs, "local_spillback_risk", 0.0)))
            key = (prio, -urgency, abs(lead), -float(rs.confidence))
            if best_key is None or key < best_key:
                best = rs
                best_key = key

        return best

    def _build_warmup_ev_from_reservation(self, rs: ReservationState, sim_time: float) -> Optional[EvRequest]:
        in_edge = str(rs.in_edge_id) if rs.in_edge_id is not None else ""
        target_phase = self._inbound_edge_to_phase.get(in_edge)
        if target_phase is None:
            return None

        eta_mid = 0.5 * (float(rs.eta_start) + float(rs.eta_end))
        lead = max(0.5, float(eta_mid - float(sim_time)))
        v_ref = self._estimate_edge_speed_limit_mps(in_edge)
        dist = max(5.0, min(400.0, float(v_ref * lead)))
        delta = max(1.0, 0.5 * max(0.0, float(rs.eta_end - rs.eta_start)))

        return EvRequest(
            ev_id=str(rs.ev_id),
            sim_time=float(sim_time),
            erl_level=1,
            speed_mps=float(v_ref),
            distance_to_intersection_m=float(dist),
            in_edge_id=in_edge,
            target_phase_idx=int(target_phase),
            delta_sec=float(delta),
            route_intersections=list(rs.route_intersections_hint or []),
            route_veh=list(rs.route_veh_hint or []),
        )

    def maybe_warmup_from_federation(self, sim_time: float) -> Optional[PreemptionPlan]:
        """
        Optional federation warmup hook:
        use accepted reservations to pre-actuate non-intrusively before direct EV contact.
        """
        if not bool(getattr(self.cfg, "fed_enable_warmup", True)):
            if bool(getattr(self, "enable_federation_debug", False)) and not bool(self._fed_warmup_disabled_reported):
                self._fed_dbg("evt=WARM_DISABLED fed_enable_warmup=0")
                self._fed_warmup_disabled_reported = True
            return None
        mode_label = str(getattr(self.cfg, "decision_log_run_label", "") or "").strip().upper()
        if mode_label == "F2" and not bool(getattr(self.cfg, "fed_warmup_enable_in_f2", False)):
            return None
        min_sim_time = max(0.0, float(getattr(self.cfg, "fed_warmup_min_sim_time_sec", 10.0)))
        if float(sim_time) < min_sim_time:
            return None
        if self.active_ev is not None:
            return None
        if float(sim_time) < float(self._next_warmup_time):
            return None

        self._next_warmup_time = float(sim_time) + float(getattr(self.cfg, "fed_warmup_period_sec", 0.5))
        rs = self._select_warmup_reservation(float(sim_time))
        if rs is None:
            if bool(getattr(self, "enable_federation_debug", False)):
                n_acc = 0
                n_all = 0
                for _rid, _rs in list(self.active_reservations.items()):
                    n_all += 1
                    if str(getattr(_rs, "status", "")).upper() == "ACCEPTED":
                        n_acc += 1
                self._fed_dbg(f"warmup_skip t={float(sim_time):.2f} accepted={n_acc}/{n_all}")
                self._fed_dbg(f"evt=WARM_STATE t={float(sim_time):.2f} selected=none accepted={n_acc} total={n_all}")
            self._federation_warmup_ev = None
            self._federation_warmup_valid_until = -1.0
            return None

        self._fed_dbg(
            f"evt=WARM_STATE t={float(sim_time):.2f} selected={rs.req_id} mode={'hard' if rs.hard else 'soft'} "
            f"eta=({float(rs.eta_start):.2f},{float(rs.eta_end):.2f}) in_edge={rs.in_edge_id}"
        )
        max_apply_per_ev = max(0, int(getattr(self.cfg, "fed_warmup_max_apply_per_ev", 1)))
        if max_apply_per_ev > 0:
            ev_applied_n = int(self._warmup_apply_count_by_ev.get(str(rs.ev_id), 0))
            if ev_applied_n >= max_apply_per_ev:
                self._fed_dbg(
                    f"warmup_skip t={float(sim_time):.2f} reason=max_apply_per_ev ev={rs.ev_id} "
                    f"applied={ev_applied_n} max={max_apply_per_ev}"
                )
                return None
        warm_ev = self._build_warmup_ev_from_reservation(rs, float(sim_time))
        if warm_ev is None:
            self._fed_dbg(
                f"warmup_skip t={float(sim_time):.2f} reason=build_warm_ev_failed req={rs.req_id}"
            )
            self._fed_dbg(
                f"evt=WARM_SKIP t={float(sim_time):.2f} reason=build_warm_ev_failed req={rs.req_id}"
            )
            return None

        prev_ev = self.active_ev
        self.active_ev = warm_ev
        try:
            t_i = 0.5 * (float(rs.eta_start) + float(rs.eta_end))
            plan = self._non_intrusive_preemption_plan(float(sim_time), float(t_i))
            if plan is None:
                plan = self._try_non_intrusive_qp(float(sim_time), float(t_i))
            if plan is None:
                return None
            if str(plan.plan_type) != "non_intrusive":
                return None
            if (
                float(plan.extend_green_sec or 0.0) <= 0.0
                and plan.hurry_current_phase_to_sec is None
                and not bool(plan.phase_duration_overrides)
            ):
                return None

            plan.notes = (
                f"{plan.notes}; fed_warmup:req={rs.req_id},"
                f"mode={'hard' if rs.hard else 'soft'}"
            )
            self._federation_warmup_ev = warm_ev
            self._federation_warmup_valid_until = float(sim_time) + float(
                getattr(self.cfg, "fed_warmup_context_ttl_sec", 2.0)
            )
            self._fed_dbg(
                f"warmup_apply t={float(sim_time):.2f} req={rs.req_id} ev={rs.ev_id} "
                f"mode={'hard' if rs.hard else 'soft'} in_edge={rs.in_edge_id} "
                f"eta=({float(rs.eta_start):.2f},{float(rs.eta_end):.2f}) plan={plan.plan_type}"
            )
            self._fed_dbg(
                f"evt=WARM_APPLY t={float(sim_time):.2f} req_id={rs.req_id} ev={rs.ev_id} plan={plan.plan_type} "
                f"mode={'hard' if rs.hard else 'soft'}"
            )
            self._warmup_apply_count_by_ev[str(rs.ev_id)] = int(self._warmup_apply_count_by_ev.get(str(rs.ev_id), 0)) + 1
            return plan
        finally:
            self.active_ev = prev_ev


    def on_handoff(self, msg: dict) -> None:
        ev_id = str(msg.get("ev_id", ""))
        now = self._now()
        self.last_handoff_by_ev[ev_id] = now
        route_tls = [str(x) for x in list(msg.get("route_intersections", []) or []) if str(x)]
        if route_tls:
            self._last_route_intersections_by_ev[ev_id] = route_tls
        route_edges = [str(x) for x in list(msg.get("route_veh", []) or []) if str(x)]
        if route_edges:
            self._last_route_edges_by_ev[ev_id] = route_edges

        # consume reservations for this EV
        to_remove = []
        for rid, rs in self.active_reservations.items():
            if rs.ev_id == ev_id:
                to_remove.append(rid)
        for rid in to_remove:
            self.active_reservations.pop(rid, None)

    def on_corridor_advice(self, msg: dict) -> None:
        ev_id = str(msg.get("ev_id", ""))
        target_tls = str(msg.get("target_tls", "") or "")
        if not ev_id or not target_tls:
            return
        self._corridor_advice_by_ev_target[(ev_id, target_tls)] = {
            "payload": dict(msg or {}),
            "ts": float(self._now()),
        }
        self._fed_dbg(
            f"evt=CORRIDOR_ADVICE_IN ev={ev_id} recipient={msg.get('recipient_tls')} role={msg.get('recipient_role')} "
            f"target={target_tls} state={((msg.get('assoc') or {}).get('assoc_state'))} "
            f"hard_target={((msg.get('reservation_guidance') or {}).get('hard_target_tls'))}"
        )

    def on_corridor_verdict(self, msg: dict) -> None:
        req_id = str(msg.get("req_id", ""))
        ev_id = str(msg.get("ev_id", ""))
        meta = {
            "payload": dict(msg or {}),
            "ts": float(self._now()),
        }
        if req_id:
            self._corridor_verdict_by_req_id[req_id] = meta
        if ev_id:
            self._corridor_verdict_by_ev[ev_id] = meta
        self._fed_dbg(
            f"evt=CORRIDOR_VERDICT_IN req_id={req_id or '-'} ev={ev_id or '-'} "
            f"verdict={msg.get('verdict')} reason={msg.get('reason', '-')}"
        )

    def _latest_corridor_advice(self, ev_id: str, target_tls: Optional[str] = None, max_age_sec: float = 3.0) -> Optional[dict]:
        ev_key = str(ev_id)
        best_payload: Optional[dict] = None
        best_sort = None
        for (stored_ev, stored_target), meta in list(self._corridor_advice_by_ev_target.items()):
            if stored_ev != ev_key:
                continue
            if target_tls is not None and str(stored_target) != str(target_tls):
                continue
            payload = dict(meta.get("payload", {}) or {})
            age = float(self._now()) - float(meta.get("ts", 0.0))
            ttl = max(float(max_age_sec), float(payload.get("ttl_sec", 0.0) or 0.0))
            if age > max(0.1, ttl):
                self._corridor_advice_by_ev_target.pop((stored_ev, stored_target), None)
                continue
            assoc = dict(payload.get("assoc", {}) or {})
            sort_key = (
                int(assoc.get("route_index", 9999) if assoc.get("route_index") is not None else 9999),
                float(assoc.get("eta_start", 1e18) if assoc.get("eta_start") is not None else 1e18),
                str(stored_target),
            )
            if best_sort is None or sort_key < best_sort:
                best_sort = sort_key
                best_payload = payload
        return best_payload

    def _latest_corridor_verdict(self, ev_id: str, max_age_sec: float = 3.0) -> Optional[dict]:
        meta = self._corridor_verdict_by_ev.get(str(ev_id))
        if not meta:
            return None
        age = float(self._now()) - float(meta.get("ts", 0.0))
        payload = dict(meta.get("payload", {}) or {})
        ttl = max(float(max_age_sec), float(payload.get("ttl_sec", 0.0) or 0.0))
        if age > max(0.1, ttl):
            self._corridor_verdict_by_ev.pop(str(ev_id), None)
            return None
        return payload


    def build_handoff_messages(self, ev_id: str, sim_time: float) -> List[Tuple[str, dict]]:
        cands = self.rank_next_hop_candidates(ev_id=ev_id, sim_time=sim_time, max_hops=1)
        if not cands:
            return []
        nb_tls, prob, eta = cands[0]
        route_hints = self._route_intersection_hints(ev_id)
        route_edges = self._route_edges_hint(ev_id)
        preferred_next_tls = self._strong_route_hint_neighbor(ev_id)
        next_edge_id = self._next_route_edge_id(ev_id)
        if preferred_next_tls is None:
            preferred_next_tls = str(nb_tls)
        if next_edge_id is None:
            ninfo = self.neighbor_map.get(str(nb_tls))
            if ninfo is not None and str(getattr(ninfo, "via_out_edge", "") or ""):
                next_edge_id = str(ninfo.via_out_edge)
        cur_edge_id = None
        if traci is not None:
            try:
                cur_edge_id = str(traci.vehicle.getRoadID(str(ev_id)))
            except Exception:
                cur_edge_id = None

        # Pass metadata may be reset after _clear_ev_session(); fallback to last_ev_validation.
        pass_time = self._ev_pass_time_est
        pass_detect_time = self._ev_pass_detect_time
        pass_proxy_time = self._ev_pass_proxy_time
        pass_from_edge = self._ev_left_approach_from_edge
        pass_to_edge = self._ev_left_approach_to_edge
        lv = dict(self.last_ev_validation or {})
        if str(lv.get("ev_id", "")) == str(ev_id):
            if pass_time is None:
                pass_time = lv.get("pass_time")
            if pass_detect_time is None:
                pass_detect_time = lv.get("pass_detect_time")
            if pass_proxy_time is None:
                pass_proxy_time = lv.get("pass_proxy_time")
            if pass_from_edge is None:
                pass_from_edge = lv.get("left_approach_from_edge")
            if pass_to_edge is None:
                pass_to_edge = lv.get("left_approach_to_edge")
        if pass_to_edge is None and next_edge_id is not None:
            pass_to_edge = str(next_edge_id)
        if pass_from_edge is None and cur_edge_id is not None:
            pass_from_edge = str(cur_edge_id)
        return [(
            nb_tls,
            {
                "ev_id": ev_id,
                "from_tls": self.cfg.tls_id,
                "to_tls": nb_tls,
                "confidence": float(prob),
                "eta": float(eta),
                "sim_time": float(sim_time),
                "route_intersections": list(route_hints),
                "route_veh": list(route_edges[:64]),
                "preferred_next_tls": preferred_next_tls,
                "current_edge_id": cur_edge_id,
                "next_edge_id": next_edge_id,
                "pass_time": pass_time,
                "pass_detect_time": pass_detect_time,
                "pass_proxy_time": pass_proxy_time,
                "pass_from_edge_id": pass_from_edge,
                "pass_to_edge_id": pass_to_edge,
            }
        )]


    def refine_with_federation(
        self,
        sim_time: float,
        ev_id: str,
        current_offer=None,
        offers: Optional[list] = None,
    ):
        """
        Sanitary behavior:
        - periodic prune
        - soft reserve top-K
        - hard reserve top-1 if confidence high
        - does not force offer swap by default (keeps control deterministic)
        """
        t0_refine = time.perf_counter()

        def _emit_refine_compute_evt() -> None:
            dt_ms = (time.perf_counter() - t0_refine) * 1000.0
            self._last_refine_compute_ms = float(max(0.0, dt_ms))
            self._fed_evt(
                "intersection.compute.refine.duration_ms",
                sim_time=float(sim_time),
                ev_id=str(ev_id),
                duration_ms=float(dt_ms),
            )

        # throttle
        if sim_time < self._next_refine_time:
            _emit_refine_compute_evt()
            return current_offer
        self._next_refine_time = sim_time + self.fed_refine_period_sec

        # prune expired reservations
        for rid, rs in list(self.active_reservations.items()):
            if sim_time > rs.ts_expire:
                self.active_reservations.pop(rid, None)

        # ranked neighbors
        cands = self.rank_next_hop_candidates(ev_id=ev_id, sim_time=sim_time, max_hops=1)
        if not cands:
            # Federation is optional: if no neighbor candidates are available,
            # preserve local control and prefer an actionable local offer.
            chosen_local = current_offer
            if offers:
                feasible = [o for o in offers if bool(getattr(o, "feasible", False))]
                actionable = []
                for o in feasible:
                    try:
                        pt = self._plan_type_from_offer(o)
                    except Exception:
                        pt = "none"
                    if pt in ("saturation_reduction", "non_intrusive", "intrusive"):
                        actionable.append(o)
                if actionable:
                    try:
                        chosen_local = min(actionable, key=self._offer_selection_key)
                    except Exception:
                        chosen_local = actionable[0]
            self._fed_dbg(
                f"refine ev={ev_id} no_candidates local_fallback={1 if chosen_local is not None else 0}"
            )
            self._fed_evt(
                "coord.refine.no_candidates",
                ev_id=str(ev_id),
                local_fallback=int(1 if chosen_local is not None else 0),
            )
            self._maybe_request_drone_downstream_context(
                sim_time=float(sim_time),
                ev_id=str(ev_id),
                reason="no_candidates",
                selected_action="request_no_downstream_tls_candidate",
            )
            _emit_refine_compute_evt()
            return chosen_local

        advice_default = self._latest_corridor_advice(str(ev_id), max_age_sec=5.0)
        verdict = self._latest_corridor_verdict(str(ev_id), max_age_sec=5.0)
        res_guidance = dict((advice_default or {}).get("reservation_guidance", {}) or {})
        assoc_guidance = dict((advice_default or {}).get("assoc", {}) or {})
        soft_topk_cfg = int(res_guidance.get("soft_topk", self.fed_soft_topk)) if advice_default else int(self.fed_soft_topk)
        hard_target_tls_override = str(res_guidance.get("hard_target_tls", "") or "") if advice_default else ""
        hard_thr_override = res_guidance.get("hard_conf_threshold_override") if advice_default else None
        eta_shift_override = max(0.0, float(res_guidance.get("eta_shift_sec", 0.0))) if advice_default else 0.0
        eta_shift_feedback = self._reservation_eta_shift_feedback(
            ev_id=str(ev_id),
            max_age_sec=max(2.0, self.fed_soft_ttl_sec),
        )
        eta_shift_effective = max(0.0, max(float(eta_shift_override), float(eta_shift_feedback)))
        if verdict is not None:
            verdict_kind = str(verdict.get("verdict", "")).upper()
            constraints = dict(verdict.get("constraints", {}) or {})
            eta_shift_effective = max(eta_shift_effective, float(constraints.get("eta_shift_sec", 0.0) or 0.0))
            if verdict_kind == "DEFER":
                hard_target_tls_override = ""

        # soft reservations (top-K)
        cands = [c for c in cands if str(c[0]) != str(self.cfg.tls_id)]
        if not cands:
            self._fed_evt(
                "coord.refine.skipped",
                ev_id=str(ev_id),
                reason="no_nonself_candidate",
            )
            self._maybe_request_drone_downstream_context(
                sim_time=float(sim_time),
                ev_id=str(ev_id),
                reason="no_nonself_candidate",
                selected_action="request_no_nonself_downstream_tls_candidate",
            )
            _emit_refine_compute_evt()
            return None
        topk = cands[: max(1, soft_topk_cfg)]
        self._fed_dbg(
            f"refine ev={ev_id} topk={[ (a, round(b,3), round(c,2)) for a,b,c in topk ]}"
        )
        self._fed_evt(
            "coord.refine.candidates",
            ev_id=str(ev_id),
            candidate_count=int(len(cands)),
            topk_count=int(len(topk)),
            top1_tls=(str(cands[0][0]) if cands else ""),
            top1_prob=(float(cands[0][1]) if cands else 0.0),
            top1_eta=(float(cands[0][2]) if cands else 0.0),
            eta_shift_effective=float(eta_shift_effective),
        )
        for nb_tls, prob, eta in topk:
            if str(nb_tls) == str(self.cfg.tls_id):
                continue
            key = (ev_id, nb_tls)
            eta_mid = float(eta)
            prev = self._last_soft_sent.get(key, None)
            resend = True
            if prev is not None:
                prev_ts, prev_eta, prev_p = prev
                if (sim_time - prev_ts) < self.fed_min_repeat_sec \
                and abs(eta_mid - prev_eta) < self.fed_eta_resend_thresh_sec \
                and abs(float(prob) - prev_p) < self.fed_prob_resend_thresh:
                    resend = False

            if resend:
                req = self.make_reservation_req(
                    to_tls=nb_tls, ev_id=ev_id, sim_time=sim_time, eta=eta_mid,
                    confidence=prob, mode="soft", eta_shift_sec=float(eta_shift_effective),
                    corridor_guidance=self._latest_corridor_advice(str(ev_id), target_tls=str(nb_tls), max_age_sec=5.0) or advice_default,
                )
                self._send_reservation_req(nb_tls, req)
                self._last_soft_sent[key] = (sim_time, eta_mid, float(prob))

        # hard reservation:
        # - default top-1
        # - if strong route hint exists, prefer that candidate and use relaxed threshold
        strong_hint_tls = self._strong_route_hint_neighbor(ev_id)
        nb0, p0, eta0 = cands[0]
        if hard_target_tls_override:
            for nbx, px, etax in cands:
                if str(nbx) == str(hard_target_tls_override):
                    nb0, p0, eta0 = nbx, px, etax
                    break
        elif strong_hint_tls is not None:
            for nbx, px, etax in cands:
                if str(nbx) == str(strong_hint_tls):
                    nb0, p0, eta0 = nbx, px, etax
                    break
        hard_thr = float(self.fed_min_hard_conf)
        if strong_hint_tls is not None and str(nb0) == str(strong_hint_tls):
            hard_thr = min(float(hard_thr), float(self.fed_min_hard_conf_with_hint))
            hard_thr = min(float(hard_thr), float(getattr(self.cfg, "fed_route_hint_strong_prob", 0.55)))
        if hard_thr_override is not None:
            try:
                hard_thr = float(hard_thr_override)
            except Exception:
                pass
        base_hard_thr = float(hard_thr)
        if bool(getattr(self.cfg, "f2_hard_threshold_adaptive_enable", True)):
            ev_dist = float(getattr(self.active_ev, "distance_to_intersection_m", -1.0)) if self.active_ev is not None else -1.0
            near_d = max(0.0, float(getattr(self.cfg, "f2_hard_threshold_near_distance_m", 120.0)))
            far_d = max(near_d, float(getattr(self.cfg, "f2_hard_threshold_far_distance_m", 300.0)))
            near_delta = float(getattr(self.cfg, "f2_hard_threshold_near_delta", -0.08))
            far_delta = float(getattr(self.cfg, "f2_hard_threshold_far_delta", 0.04))
            if ev_dist >= 0.0 and ev_dist <= near_d:
                hard_thr += near_delta
            elif ev_dist >= far_d:
                hard_thr += far_delta

            if bool(getattr(self.cfg, "f2_hard_threshold_quality_relax_enable", True)) and current_offer is not None and offers:
                try:
                    feasible = [o for o in offers if bool(getattr(o, "feasible", False))]
                    if feasible and bool(getattr(current_offer, "feasible", False)):
                        best_local_ev_cost = min(float(self._offer_ev_cost(o)) for o in feasible)
                        curr_ev_cost = float(self._offer_ev_cost(current_offer))
                        margin = max(0.0, float(getattr(self.cfg, "f2_hard_threshold_quality_relax_ev_cost_margin_sec", 1.5)))
                        if curr_ev_cost <= (best_local_ev_cost + margin):
                            hard_thr -= float(getattr(self.cfg, "f2_hard_threshold_quality_relax_delta", 0.05))
                except Exception:
                    pass

            hard_thr = max(
                float(getattr(self.cfg, "f2_hard_threshold_min", 0.35)),
                min(float(getattr(self.cfg, "f2_hard_threshold_max", 0.90)), float(hard_thr)),
            )
            if abs(float(hard_thr) - float(base_hard_thr)) > 1e-9:
                self._fed_evt(
                    "coord.refine.hard_threshold.adapt",
                    ev_id=str(ev_id),
                    to_tls=str(nb0),
                    threshold_base=float(base_hard_thr),
                    threshold_adapted=float(hard_thr),
                    ev_distance_m=float(ev_dist),
                    near_distance_m=float(near_d),
                    far_distance_m=float(far_d),
                )

        hard_skip_cfg_enable = bool(getattr(self.cfg, "f2_hard_req_skip_cooldown_enable", True))
        hard_skip_trigger = max(1, int(getattr(self.cfg, "f2_hard_req_skip_streak_trigger", 4)))
        hard_skip_window_sec = max(0.5, float(getattr(self.cfg, "f2_hard_req_skip_streak_window_sec", 3.0)))
        hard_skip_cooldown_sec = max(0.5, float(getattr(self.cfg, "f2_hard_req_skip_cooldown_sec", 2.5)))
        hard_skip_escape_margin = max(0.0, float(getattr(self.cfg, "f2_hard_req_cooldown_escape_margin", 0.08)))
        hard_key = (str(ev_id), str(nb0))

        def _track_hard_skip(reason: str, *, prob: float, threshold: float) -> None:
            if not hard_skip_cfg_enable:
                return
            rec = dict(self._hard_req_skip_tracker.get(hard_key, {}) or {})
            prev_ts = float(rec.get("last_ts", -1e9))
            prev_reason = str(rec.get("last_reason", "") or "")
            streak = int(rec.get("streak", 0))
            if (sim_time - prev_ts) <= hard_skip_window_sec and prev_reason == str(reason):
                streak += 1
            else:
                streak = 1
            rec["streak"] = int(streak)
            rec["last_ts"] = float(sim_time)
            rec["last_reason"] = str(reason)
            rec["last_prob"] = float(prob)
            rec["last_threshold"] = float(threshold)
            if streak >= hard_skip_trigger:
                cooldown_until = float(sim_time + hard_skip_cooldown_sec)
                prev_cd = float(rec.get("cooldown_until", -1e9))
                rec["cooldown_until"] = max(prev_cd, cooldown_until)
                self._fed_evt(
                    "coord.refine.hard_req_cooldown_enter",
                    ev_id=str(ev_id),
                    to_tls=str(nb0),
                    streak=int(streak),
                    reason=str(reason),
                    cooldown_sec=float(hard_skip_cooldown_sec),
                    cooldown_until=float(rec["cooldown_until"]),
                    prob=float(prob),
                    threshold=float(threshold),
                )
            self._hard_req_skip_tracker[hard_key] = rec

        if verdict is not None and str(verdict.get("verdict", "")).upper() == "DEFER":
            self._fed_dbg(
                f"hard_skip ev={ev_id} to={nb0} reason=corridor_defer strong_hint={strong_hint_tls}"
            )
            self._session_event_counts["hard_req_skip"] += 1
            self._session_reason_counts["hard_req_skip:corridor_defer"] += 1
            self._fed_evt(
                "coord.refine.hard_req_skip",
                ev_id=str(ev_id),
                to_tls=str(nb0),
                prob=float(p0),
                threshold=float(hard_thr),
                reason="corridor_defer",
            )
            _track_hard_skip("corridor_defer", prob=float(p0), threshold=float(hard_thr))
        elif float(p0) >= float(hard_thr):
            hard_skip_state = dict(self._hard_req_skip_tracker.get(hard_key, {}) or {})
            cooldown_until = float(hard_skip_state.get("cooldown_until", -1e9))
            if hard_skip_cfg_enable and float(sim_time) < cooldown_until and float(p0) < float(hard_thr + hard_skip_escape_margin):
                self._session_event_counts["hard_req_skip"] += 1
                self._session_reason_counts["hard_req_skip:cooldown_active"] += 1
                self._fed_evt(
                    "coord.refine.hard_req_skip",
                    ev_id=str(ev_id),
                    to_tls=str(nb0),
                    prob=float(p0),
                    threshold=float(hard_thr),
                    reason="cooldown_active",
                    cooldown_until=float(cooldown_until),
                    escape_margin=float(hard_skip_escape_margin),
                )
                self._fed_dbg(
                    f"hard_skip ev={ev_id} to={nb0} reason=cooldown_active p={float(p0):.3f} "
                    f"thr={float(hard_thr):.3f} cd_until={float(cooldown_until):.2f}"
                )
                # Cooldown applies to hard-request publication only; keep local offer path alive.
            prev_h = self._last_hard_sent.get(ev_id, None)
            do_send = True
            if prev_h is not None:
                prev_nb, prev_ts, prev_p = prev_h
                if prev_nb == nb0 and (sim_time - prev_ts) < self.fed_min_repeat_sec \
                and abs(float(p0) - prev_p) < self.fed_prob_resend_thresh:
                    do_send = False

            if do_send:
                req = self.make_reservation_req(
                    to_tls=nb0, ev_id=ev_id, sim_time=sim_time, eta=eta0,
                    confidence=p0, mode="hard", eta_shift_sec=float(eta_shift_effective),
                    corridor_guidance=self._latest_corridor_advice(str(ev_id), target_tls=str(nb0), max_age_sec=5.0) or advice_default,
                )
                self._send_reservation_req(nb0, req)
                self._last_hard_sent[ev_id] = (nb0, sim_time, float(p0))
                self._fed_dbg(
                    f"hard_req ev={ev_id} to={nb0} p={float(p0):.3f} thr={float(hard_thr):.3f} "
                    f"strong_hint={strong_hint_tls} eta_shift={float(eta_shift_effective):.2f} "
                    f"assoc_state={assoc_guidance.get('assoc_state', '-') if assoc_guidance else '-'}"
                )
                self._fed_dbg(
                    f"evt=HARD_REQ_SENT req_id={req.get('req_id')} ev={ev_id} to={nb0} p={float(p0):.3f} thr={float(hard_thr):.3f}"
                )
                self._fed_evt(
                    "coord.refine.hard_req_sent",
                    req_id=str(req.get("req_id", "") or ""),
                    ev_id=str(ev_id),
                    to_tls=str(nb0),
                    prob=float(p0),
                    threshold=float(hard_thr),
                    eta=float(eta0),
                    eta_shift_effective=float(eta_shift_effective),
                )
                self._hard_req_skip_tracker.pop(hard_key, None)
        else:
            self._fed_dbg(
                f"hard_skip ev={ev_id} to={nb0} p={float(p0):.3f} thr={float(hard_thr):.3f} "
                f"strong_hint={strong_hint_tls}"
            )
            self._fed_dbg(
                f"evt=HARD_REQ_SKIP ev={ev_id} to={nb0} p={float(p0):.3f} thr={float(hard_thr):.3f}"
            )
            self._session_event_counts["hard_req_skip"] += 1
            self._session_reason_counts["hard_req_skip:below_threshold"] += 1
            self._fed_evt(
                "coord.refine.hard_req_skip",
                ev_id=str(ev_id),
                to_tls=str(nb0),
                prob=float(p0),
                threshold=float(hard_thr),
                reason="below_threshold",
            )
            _track_hard_skip("below_threshold", prob=float(p0), threshold=float(hard_thr))
            if bool(getattr(self.cfg, "f2_hard_skip_failsoft_enable", True)):
                rec = dict(self._hard_req_skip_tracker.get(hard_key, {}) or {})
                streak = int(rec.get("streak", 0))
                trig = max(1, int(getattr(self.cfg, "f2_hard_skip_failsoft_streak_trigger", 3)))
                near_only = bool(getattr(self.cfg, "f2_hard_skip_failsoft_near_only", True))
                near_dist = max(0.0, float(getattr(self.cfg, "f2_hard_skip_failsoft_near_distance_m", 120.0)))
                ev_dist = float(getattr(self.active_ev, "distance_to_intersection_m", -1.0)) if self.active_ev is not None else -1.0
                near_ok = (ev_dist >= 0.0 and ev_dist <= near_dist)
                if streak >= trig and (near_ok or (not near_only)):
                    self._fed_evt(
                        "coord.refine.hard_req_failsoft_local",
                        ev_id=str(ev_id),
                        to_tls=str(nb0),
                        streak=int(streak),
                        trigger=int(trig),
                        ev_distance_m=float(ev_dist),
                        near_only=int(1 if near_only else 0),
                        near_distance_m=float(near_dist),
                    )
                    _emit_refine_compute_evt()
                    return current_offer

        # selection adaptation from recent reservation responses:
        # if hard reservations are repeatedly rejected, prefer less aggressive options.
        fb = self._reservation_feedback(ev_id, max_age_sec=max(2.0, self.fed_soft_ttl_sec))
        if offers and fb["hard_rejected"] > fb["hard_accepted"]:
            feasible = [o for o in offers if bool(getattr(o, "feasible", False))]
            if feasible:
                def _pt_rank(o: SignalWindowOffer) -> int:
                    pt = self._plan_type_from_offer(o)
                    if pt in ("none", "restore"):
                        return 0
                    if pt == "saturation_reduction":
                        return 1
                    if pt == "non_intrusive":
                        return 2
                    return 3  # intrusive
                conservative = sorted(
                    feasible,
                    key=lambda o: (
                        _pt_rank(o),
                        float(self.score_offer(o)),
                    ),
                )
                _emit_refine_compute_evt()
                return conservative[0]

        # downstream pressure adaptation:
        # if responder reports poor queue margin / spillback risk,
        # prefer lower-disruption options for this tick.
        down_fb = self._latest_downstream_feedback(
            ev_id=str(ev_id),
            max_age_sec=float(getattr(self.cfg, "robust_fed_resp_max_age_sec", 15.0)),
        )
        if offers and down_fb:
            down_qm = float(down_fb.get("downstream_queue_margin_sec", 0.0))
            down_sp = float(down_fb.get("downstream_spillback_risk", 0.0))
            down_alert = (
                down_qm < float(getattr(self.cfg, "robust_fed_down_hard_queue_margin_sec", -2.0))
                or down_sp > float(getattr(self.cfg, "robust_fed_down_hard_spillback", 0.85))
            )
            if down_alert:
                feasible = [o for o in offers if bool(getattr(o, "feasible", False))]
                if feasible:
                    def _pt_rank_down(o: SignalWindowOffer) -> int:
                        pt = self._plan_type_from_offer(o)
                        if pt in ("none", "restore"):
                            return 0
                        if pt == "saturation_reduction":
                            return 1
                        if pt == "non_intrusive":
                            return 2
                        return 3
                    conservative = sorted(
                        feasible,
                        key=lambda o: (
                            _pt_rank_down(o),
                            float(self.score_offer(o)),
                        ),
                    )
                    _emit_refine_compute_evt()
                    return conservative[0]

        _emit_refine_compute_evt()
        return current_offer
    
    # TLS Program

    def capture_default_tls_program(self) -> None:
        if traci is None:
            return
        tls_id = self.cfg.tls_id
        try:
            sim_t = float(traci.simulation.getTime())
            program_id = str(traci.trafficlight.getProgram(tls_id))
            logics = traci.trafficlight.getAllProgramLogics(tls_id)
        except Exception:
            return

        logic = None
        for lg in logics:
            if str(getattr(lg, "programID", "")) == program_id:
                logic = lg
                break
        if logic is None and logics:
            logic = logics[0]
            program_id = str(getattr(logic, "programID", program_id))
        if logic is None:
            return

        phases = []
        cyc = 0.0
        for i, ph in enumerate(getattr(logic, "phases", [])):
            d = float(getattr(ph, "duration", 0.0))
            s = str(getattr(ph, "state", ""))
            phases.append(PhaseTemplate(idx=i, duration=d, state=s))
            cyc += d

        self.default_tls_program = TLSProgramTemplate(
            tls_id=tls_id,
            program_id=program_id,
            phases=tuple(phases),
            cycle_sec=cyc,
            captured_at_sim_time=sim_t,
        )
        self._default_duration_by_phase = {p.idx: p.duration for p in phases}
    
    def schedule_recovery_to_default(self) -> None:
        """
        Soft recovery: restore per-phase durations as phases appear naturally.
        Keeps traffic smooth.
        """
        if not self._default_duration_by_phase:
            return
        self._restoration_schedule = dict(self._default_duration_by_phase)
        self._restoration_applied_phases.clear()

        # clear transient overrides (offer/plans)
        self._active_phase_overrides = None
        self._override_applied_in_current_phase = False
        self._override_last_seen_phase = None


    def hard_reset_to_default_program(self) -> None:
        """
        Hard recovery: reset program immediately to captured baseline.
        Use sparingly (can be abrupt).
        """
        if traci is None or self.default_tls_program is None:
            return
        tls_id = self.cfg.tls_id
        try:
            traci.trafficlight.setProgram(tls_id, self.default_tls_program.program_id)
        except Exception:
            pass

        # clear local intervention state
        self._active_phase_overrides = None
        self._restoration_schedule = None
        self._restoration_applied_phases.clear()
        self._restore_program_applied_for_session = False
        self.current_plan = None

    def _had_nondefault_actuation(self) -> bool:
        # conservative check: if we changed timings/overrides/program behavior
        if self._active_phase_overrides is not None:
            return True
        if abs(float(getattr(self, "_timing_offset_sec", 0.0))) > 1e-6:
            return True
        if self._restoration_schedule is not None:
            return True
        cp = self.current_plan
        if cp is not None and cp.plan_type in (
            "saturation_reduction",
            "non_intrusive",
            "intrusive",
            "restore",
        ):
            return True
        return False

    def _clear_ev_session(self) -> None:
        ev = self.active_ev
        session_ev_id = str(ev.ev_id) if ev is not None else str(self.last_ev_validation.get("ev_id", "") or "")
        if session_ev_id:
            total_evt = int(sum(int(v) for v in self._session_event_counts.values()))
            if total_evt > 0:
                self._fed_evt(
                    "coord.session.summary",
                    ev_id=str(session_ev_id),
                    mode=str(getattr(self.cfg, "decision_log_run_label", "") or ""),
                    apply_offer_n=int(self._session_event_counts.get("apply_offer", 0)),
                    apply_plan_n=int(self._session_event_counts.get("apply_plan", 0)),
                    apply_plan_offer_n=int(self._session_event_counts.get("apply_plan_source:offer", 0)),
                    apply_plan_local_fallback_n=int(self._session_event_counts.get("apply_plan_source:f2_local_fallback", 0)),
                    apply_plan_selected_none_n=int(self._session_event_counts.get("apply_plan_source:f2_selected_none", 0)),
                    apply_plan_warmup_n=int(self._session_event_counts.get("apply_plan_source:federation_warmup", 0)),
                    plan_skip_n=int(self._session_event_counts.get("plan_skip", 0)),
                    hard_req_skip_n=int(self._session_event_counts.get("hard_req_skip", 0)),
                    selection_final_n=int(self._session_event_counts.get("selection_final", 0)),
                    session_reason_counts=dict(self._session_reason_counts),
                    latest_tick_compute_ms=float(max(0.0, self._last_tick_compute_ms)),
                    latest_refine_compute_ms=float(max(0.0, self._last_refine_compute_ms)),
                    latest_apply_compute_ms=float(max(0.0, self._last_apply_compute_ms)),
                )
        self._session_event_counts = Counter()
        self._session_reason_counts = Counter()
        if ev is not None:
            self.last_ev_validation = {
                "tls_id": str(self.cfg.tls_id),
                "ev_id": str(ev.ev_id),
                "pass_time": self._ev_pass_time_est,
                "pass_detect_time": self._ev_pass_detect_time,
                "pass_proxy_time": self._ev_pass_proxy_time,
                "pass_reason": str(self._ev_pass_reason),
                "loop_touch_time": self._ev_loop_touch_time,
                "loop_touch_loop_id": self._ev_loop_touch_loop_id,
                "request_silence_time": self._ev_request_silence_time,
                "left_approach_time": self._ev_left_approach_time,
                "left_approach_from_edge": self._ev_left_approach_from_edge,
                "left_approach_to_edge": self._ev_left_approach_to_edge,
                "last_seen_road_id": self._ev_last_seen_road_id,
                "pred_ti_first": self._ev_pred_ti_first,
                "pred_ti_last": self._ev_pred_ti_last,
                "pred_err_first_sec": (
                    None if (self._ev_pass_time_est is None or self._ev_pred_ti_first is None)
                    else float(self._ev_pass_time_est - self._ev_pred_ti_first)
                ),
                "pred_err_last_sec": (
                    None if (self._ev_pass_time_est is None or self._ev_pred_ti_last is None)
                    else float(self._ev_pass_time_est - self._ev_pred_ti_last)
                ),
            }
        self._finalize_selected_offer_calibration(
            sim_time=float(self._now()),
            passed=bool(self.ev_passed),
        )
        self.active_ev = None
        self._federation_warmup_ev = None
        self._federation_warmup_valid_until = -1.0
        self.last_ev_msg_time = None
        self.ev_passed = False
        self._reset_ev_pass_tracking()
        self.current_plan = None
        self._timing_offset_sec = 0.0
        self._restoration_schedule = None
        self._restoration_applied_phases.clear()
        self._restore_program_applied_for_session = False



# =========================
# Helper for EV distance to TLS (recommended)
# =========================

def distance_to_next_tls_stopline(veh_id: str, tls_id: str) -> Optional[float]:
    """
    Prefer this over Euclidean distance.

    Uses SUMO's own routing/lanes to compute distance along the vehicle's path to the next TLS stop line.
    Works if the vehicle has a next TLS on its route.

    Returns None if no matching TLS ahead.
    """
    if traci is None:
        return None
    try:
        nxt = traci.vehicle.getNextTLS(veh_id)
    except Exception:
        return None

    for item in nxt or []:
        try:
            tid = item[0]
            dist = float(item[1])
        except Exception:
            continue
        if str(tid) == str(tls_id):
            return float(dist)

    # If the next TLS list doesn't include the exact ID, return first distance if any.
    if nxt:
        try:
            return float(nxt[0][1])
        except Exception:
            return None
    return None


# =========================
# Offer Robust Metrics Helpers (paper-facing)
# =========================

@dataclass
class OfferRobustMetricSnapshot:
    # EV service
    window_coverage_ratio: float = 0.0
    ev_expected_wait_uniform_sec: float = 0.0
    ev_miss_probability_uniform: float = 0.0
    ev_expected_late_uniform_sec: float = 0.0

    # Queue-clearing / feasibility
    queue_N_veh: float = 0.0
    queue_A_vehps: float = 0.0
    queue_S_vehps: float = 0.0
    queue_Q_clear_sec: float = 0.0
    queue_required_clear_sec: float = 0.0
    queue_available_pre_arrival_green_sec: float = 0.0
    queue_clear_margin_sec: float = 0.0

    # Impact on others / control burden
    non_ev_delay_impact_veh_sec: float = 0.0
    spillback_risk_max: float = 0.0
    spillback_risk_mean: float = 0.0
    control_effort: float = 0.0

    # Safety proxy (TTC-based one-step exposure)
    ttc_sec: float = 9999.0
    tet_step_sec: float = 0.0
    tit_step: float = 0.0

    # Speed-advice executability/safety
    speed_advice_feasible: float = 1.0
    speed_advice_risk: float = 0.0
    speed_advice_overspeed_mps: float = 0.0
    speed_advice_accel_excess: float = 0.0

    # Federation reliability
    fed_response_count: float = 0.0
    fed_accept_ratio: float = 0.0
    fed_reject_ratio: float = 0.0
    fed_mean_resp_age_sec: float = 0.0
    fed_active_reservations: float = 0.0
    fed_handoff_recent: float = 0.0


def _safe_div(num: float, den: float, default: float = 0.0) -> float:
    if abs(float(den)) <= 1e-9:
        return float(default)
    return float(num) / float(den)


def _interval_overlap_sec(a0: float, a1: float, b0: float, b1: float) -> float:
    return max(0.0, min(float(a1), float(b1)) - max(float(a0), float(b0)))


def offer_window_coverage_ratio(offer: SignalWindowOffer) -> float:
    ar0, ar1 = float(offer.arrival_window[0]), float(offer.arrival_window[1])
    gw0, gw1 = float(offer.green_window[0]), float(offer.green_window[1])
    overlap = _interval_overlap_sec(ar0, ar1, gw0, gw1)
    return max(0.0, min(1.0, _safe_div(overlap, max(1e-6, ar1 - ar0), 0.0)))


def offer_ev_wait_miss_uniform(
    offer: SignalWindowOffer,
) -> Tuple[float, float, float]:
    """
    EV arrival-time uncertainty model:
    T ~ Uniform(arrival_window). Returns:
      (E[wait], P(miss), E[late])
    """
    a = float(offer.arrival_window[0])
    b = float(offer.arrival_window[1])
    g0 = float(offer.green_window[0])
    g1 = float(offer.green_window[1])
    if b <= a:
        return 0.0, 0.0, 0.0

    # E[max(0, g0 - T)] for T~U[a,b]
    if b <= g0:
        e_wait = g0 - 0.5 * (a + b)
    elif a >= g0:
        e_wait = 0.0
    else:
        e_wait = ((g0 - a) * (g0 - a)) / (2.0 * (b - a))

    # P(T > g1)
    if b <= g1:
        p_miss = 0.0
    elif a >= g1:
        p_miss = 1.0
    else:
        p_miss = (b - g1) / (b - a)

    # E[max(0, T - g1)] for T~U[a,b]
    if a >= g1:
        e_late = 0.5 * (a + b) - g1
    elif b <= g1:
        e_late = 0.0
    else:
        e_late = ((b - g1) * (b - g1)) / (2.0 * (b - a))

    return float(e_wait), float(p_miss), float(e_late)


def offer_queue_clearing_metrics(
    agent: "IntersectionAgent",
    sim_time: float,
    offer: SignalWindowOffer,
) -> Tuple[float, float, float, float, float, float, float]:
    """
    Returns:
      (N, A, S, Q, required_clear_sec, available_pre_arrival_green_sec, margin_sec)
    """
    if traci is None or agent.active_ev is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    ev = agent.active_ev
    N, A, S, Q, _lanes = agent._queue_clearing_metrics_for_edge(
        edge_id=str(ev.in_edge_id),
        ev_id=str(ev.ev_id),
    )

    required_clear = float(Q) + float(getattr(agent.cfg, "T_lost_sec", 5.0)) + float(getattr(agent.cfg, "YT_sec", 5.0))
    gw0, gw1 = float(offer.green_window[0]), float(offer.green_window[1])
    aw0 = float(offer.arrival_window[0])

    # Conservative: only green available before EV arrival-start.
    available_pre_arrival_green = max(0.0, min(gw1, aw0) - gw0)
    margin = float(available_pre_arrival_green - required_clear)
    return (
        float(N),
        float(A),
        float(S),
        float(Q),
        float(required_clear),
        float(available_pre_arrival_green),
        float(margin),
    )


def offer_non_ev_delay_impact(agent: "IntersectionAgent", offer: SignalWindowOffer) -> float:
    return float(
        agent._estimate_non_ev_cost_veh_sec(
            action=str(offer.action),
            action_params=dict(offer.action_params or {}),
            target_phase=int(offer.target_phase_idx),
        )
    )


def offer_spillback_risk(
    agent: "IntersectionAgent",
    exclude_ev_approach: bool = True,
) -> Tuple[float, float]:
    if traci is None:
        return 0.0, 0.0

    tls_id = str(agent.cfg.tls_id)
    try:
        lanes = list(traci.trafficlight.getControlledLanes(tls_id))
    except Exception:
        lanes = []

    ev_edge = None
    if exclude_ev_approach and agent.active_ev is not None:
        ev_edge = str(agent.active_ev.in_edge_id)

    risks: List[float] = []
    for lid in lanes:
        try:
            edge = lid.rsplit("_", 1)[0]
            if ev_edge and edge == ev_edge:
                continue

            q = float(traci.lane.getLastStepHaltingNumber(lid))
            lane_len = max(1.0, float(traci.lane.getLength(lid)))
            vids = list(traci.lane.getLastStepVehicleIDs(lid))

            if vids:
                lengths = []
                gaps = []
                for vid in vids[:10]:
                    try:
                        lengths.append(float(traci.vehicle.getLength(vid)))
                        gaps.append(float(traci.vehicle.getMinGap(vid)))
                    except Exception:
                        pass
                vlen = (sum(lengths) / len(lengths)) if lengths else 5.0
                vgap = (sum(gaps) / len(gaps)) if gaps else 2.5
            else:
                vlen, vgap = 5.0, 2.5

            cap = max(1.0, lane_len / max(1.0, vlen + vgap))
            r = max(0.0, (q - cap) / cap)
            risks.append(float(r))
        except Exception:
            continue

    if not risks:
        return 0.0, 0.0
    return float(max(risks)), float(sum(risks) / len(risks))


def offer_control_effort(offer: SignalWindowOffer) -> float:
    ap = dict(offer.action_params or {})
    action = str(offer.action)
    if action == "none":
        return 0.0
    if action == "extend":
        return max(0.0, float(ap.get("ext", 0.0)))
    if action == "hurry":
        hurry_to = float(ap.get("hurry_to", 2.0))
        return max(0.0, 3.0 - hurry_to) + 0.1 * max(0.0, float(ap.get("ext", 0.0)))
    if action == "jump":
        return 1000.0
    return 10.0


def ev_ttc_tet_tit_step(
    agent: "IntersectionAgent",
    sim_step_sec: float = 0.1,
    ttc_threshold_sec: float = 5.0,
    leader_lookahead_m: float = 120.0,
) -> Tuple[float, float, float]:
    """
    One-step TTC proxy (same-lane leader):
      - TTC = gap / max(v_ev - v_leader, eps), inf if not closing.
      - TET_step = dt if TTC < threshold else 0
      - TIT_step = (threshold - TTC)*dt if TTC < threshold else 0
    """
    if traci is None or agent.active_ev is None:
        return 9999.0, 0.0, 0.0

    ev_id = str(agent.active_ev.ev_id)
    try:
        lead = traci.vehicle.getLeader(ev_id, float(leader_lookahead_m))
    except Exception:
        lead = None
    if not lead:
        return 9999.0, 0.0, 0.0

    leader_id, gap = lead[0], float(lead[1])
    try:
        v_ev = float(traci.vehicle.getSpeed(ev_id))
        v_lead = float(traci.vehicle.getSpeed(str(leader_id)))
    except Exception:
        return 9999.0, 0.0, 0.0

    dv = float(v_ev - v_lead)
    if dv <= 1e-3:
        ttc = 9999.0
    else:
        ttc = max(0.0, gap / dv)

    dt = max(0.0, float(sim_step_sec))
    if ttc < float(ttc_threshold_sec):
        tet = dt
        tit = max(0.0, float(ttc_threshold_sec - ttc)) * dt
    else:
        tet = 0.0
        tit = 0.0
    return float(ttc), float(tet), float(tit)


def offer_speed_advice_risk(
    agent: "IntersectionAgent",
    sim_time: float,
    offer: SignalWindowOffer,
    max_accel_mps2: float = 2.5,
    max_decel_mps2: float = 3.5,
) -> Tuple[float, float, float, float]:
    """
    Returns:
      (feasible01, risk, overspeed_mps, accel_excess)
    """
    ev = agent.active_ev
    if ev is None or offer.speed_range_mps is None:
        return 1.0, 0.0, 0.0, 0.0

    vmin, vmax = float(offer.speed_range_mps[0]), float(offer.speed_range_mps[1])
    vcur = float(ev.speed_mps)
    t_to_window = max(0.1, float(offer.green_window[0]) - float(sim_time))

    if traci is not None:
        try:
            lane_id = traci.vehicle.getLaneID(str(ev.ev_id))
            vlim = float(traci.lane.getMaxSpeed(lane_id))
        except Exception:
            vlim = max(vmax, 13.9)
    else:
        vlim = max(vmax, 13.9)

    overspeed = max(0.0, vmax - vlim)

    a_req_for_vmin = (vmin - vcur) / t_to_window
    a_req_for_vmax = (vmax - vcur) / t_to_window
    accel_excess = max(0.0, a_req_for_vmax - max_accel_mps2)
    decel_excess = max(0.0, (-a_req_for_vmin) - max_decel_mps2)

    risk = (overspeed / max(1.0, vlim)) + 0.5 * (accel_excess + decel_excess)
    feasible = 1.0 if (overspeed <= 1e-6 and accel_excess <= 1e-6 and decel_excess <= 1e-6) else 0.0
    return float(feasible), float(max(0.0, risk)), float(overspeed), float(accel_excess + decel_excess)


def federation_reliability_metrics(
    agent: "IntersectionAgent",
    sim_time: float,
    age_window_sec: float = 15.0,
) -> Tuple[float, float, float, float, float, float]:
    recent = []
    for meta in getattr(agent, "resp_cache", {}).values():
        age = float(sim_time) - float(meta.get("ts", 0.0))
        if age <= float(age_window_sec):
            recent.append((str(meta.get("status", "UNKNOWN")).upper(), age))

    n = float(len(recent))
    if n <= 0.0:
        accept = 0.0
        reject = 0.0
        mean_age = 0.0
    else:
        acc = sum(1 for st, _ in recent if st == "ACCEPTED")
        rej = sum(1 for st, _ in recent if st == "REJECTED")
        accept = float(acc) / n
        reject = float(rej) / n
        mean_age = float(sum(age for _, age in recent) / n)

    active_res = float(len(getattr(agent, "active_reservations", {})))
    if agent.active_ev is not None:
        last_h = float(getattr(agent, "last_handoff_by_ev", {}).get(str(agent.active_ev.ev_id), -1e9))
        handoff_recent = 1.0 if (float(sim_time) - last_h) <= float(age_window_sec) else 0.0
    else:
        handoff_recent = 0.0

    return float(n), float(accept), float(reject), float(mean_age), float(active_res), float(handoff_recent)


def collect_offer_robust_metrics(
    agent: "IntersectionAgent",
    sim_time: float,
    offer: SignalWindowOffer,
    sim_step_sec: float = 0.1,
    ttc_threshold_sec: float = 5.0,
    federation_age_window_sec: float = 15.0,
) -> OfferRobustMetricSnapshot:
    wcr = offer_window_coverage_ratio(offer)
    e_wait, p_miss, e_late = offer_ev_wait_miss_uniform(offer)

    (
        qN,
        qA,
        qS,
        qQ,
        qReq,
        qAvail,
        qMargin,
    ) = offer_queue_clearing_metrics(agent, sim_time, offer)

    non_ev = offer_non_ev_delay_impact(agent, offer)
    spill_max, spill_mean = offer_spillback_risk(agent)
    effort = offer_control_effort(offer)

    ttc, tet_step, tit_step = ev_ttc_tet_tit_step(
        agent=agent,
        sim_step_sec=sim_step_sec,
        ttc_threshold_sec=ttc_threshold_sec,
    )

    sp_feas, sp_risk, sp_over, sp_acc_ex = offer_speed_advice_risk(
        agent=agent,
        sim_time=sim_time,
        offer=offer,
    )

    (
        fed_n,
        fed_acc,
        fed_rej,
        fed_age,
        fed_active,
        fed_handoff,
    ) = federation_reliability_metrics(
        agent=agent,
        sim_time=sim_time,
        age_window_sec=federation_age_window_sec,
    )

    return OfferRobustMetricSnapshot(
        window_coverage_ratio=float(wcr),
        ev_expected_wait_uniform_sec=float(e_wait),
        ev_miss_probability_uniform=float(p_miss),
        ev_expected_late_uniform_sec=float(e_late),
        queue_N_veh=float(qN),
        queue_A_vehps=float(qA),
        queue_S_vehps=float(qS),
        queue_Q_clear_sec=float(qQ),
        queue_required_clear_sec=float(qReq),
        queue_available_pre_arrival_green_sec=float(qAvail),
        queue_clear_margin_sec=float(qMargin),
        non_ev_delay_impact_veh_sec=float(non_ev),
        spillback_risk_max=float(spill_max),
        spillback_risk_mean=float(spill_mean),
        control_effort=float(effort),
        ttc_sec=float(ttc),
        tet_step_sec=float(tet_step),
        tit_step=float(tit_step),
        speed_advice_feasible=float(sp_feas),
        speed_advice_risk=float(sp_risk),
        speed_advice_overspeed_mps=float(sp_over),
        speed_advice_accel_excess=float(sp_acc_ex),
        fed_response_count=float(fed_n),
        fed_accept_ratio=float(fed_acc),
        fed_reject_ratio=float(fed_rej),
        fed_mean_resp_age_sec=float(fed_age),
        fed_active_reservations=float(fed_active),
        fed_handoff_recent=float(fed_handoff),
    )


# Improved versions of metrics....
def _ia_overlap_sec(a0: float, a1: float, b0: float, b1: float) -> float:
    return max(0.0, min(float(a1), float(b1)) - max(float(a0), float(b0)))


def _ia_collect_non_ev_snapshot(
    agent: "IntersectionAgent",
    ev_edge_override: Optional[str] = None,
) -> Dict[str, float]:
    if traci is None:
        return {"halts": 0.0, "veh": 0.0, "lanes": 0.0}
    try:
        lanes = list(traci.trafficlight.getControlledLanes(str(agent.cfg.tls_id)))
    except Exception:
        lanes = []
    ev_edge = str(ev_edge_override or "")
    halts = 0.0
    veh = 0.0
    used = 0.0
    for lid in lanes:
        try:
            edge = lid.rsplit("_", 1)[0]
        except Exception:
            edge = ""
        if ev_edge and edge == ev_edge:
            continue
        try:
            halts += float(traci.lane.getLastStepHaltingNumber(lid))
            veh += float(traci.lane.getLastStepVehicleNumber(lid))
            used += 1.0
        except Exception:
            continue
    return {"halts": float(halts), "veh": float(veh), "lanes": float(used)}


def _ia_expected_wait_miss_from_windows_improved(
    self: "IntersectionAgent",
    arrival_window: Tuple[float, float],
    green_window: Tuple[float, float],
    queue_ratio: float = 0.0,
    queue_delay_sec: float = 0.0,
) -> Tuple[float, float, float]:
    e_wait, p_miss, e_late = self._expected_wait_miss_from_windows(arrival_window, green_window)
    if bool(getattr(self.cfg, "offer_metric_queue_risk_bias_enable", False)):
        q_risk = max(0.0, float(queue_ratio) - 0.8)
        e_wait += 0.10 * max(0.0, float(queue_delay_sec))
        p_miss = min(1.0, max(0.0, float(p_miss) + 0.15 * q_risk))
        e_late += 0.25 * max(0.0, float(queue_delay_sec)) * max(0.0, p_miss)
    return float(e_wait), float(p_miss), float(e_late)


def _ia_estimate_non_ev_cost_veh_sec_improved(
    self: "IntersectionAgent",
    action: str,
    action_params: Dict[str, float],
    target_phase: int,
    horizon_sec: float,
    ev_edge_override: Optional[str],
    queue_N: float,
    queue_A: float,
    queue_S: float,
    queue_Q: float,
    queue_delay_sec: float,
) -> float:
    # Backward-compatible call: legacy variants may not accept horizon_sec/ev_edge_override.
    try:
        base_raw = self._estimate_non_ev_cost_veh_sec(
            action=action,
            action_params=action_params,
            target_phase=int(target_phase),
            horizon_sec=float(horizon_sec),
            ev_edge_override=ev_edge_override,
        )
    except TypeError:
        try:
            base_raw = self._estimate_non_ev_cost_veh_sec(
                action=action,
                action_params=action_params,
                target_phase=int(target_phase),
                ev_edge_override=ev_edge_override,
            )
        except TypeError:
            try:
                base_raw = self._estimate_non_ev_cost_veh_sec(
                    action=action,
                    action_params=action_params,
                    target_phase=int(target_phase),
                )
            except TypeError:
                base_raw = self._estimate_non_ev_cost_veh_sec(action, action_params, int(target_phase))

    base = float(base_raw)
    snap = _ia_collect_non_ev_snapshot(self, ev_edge_override=ev_edge_override)
    lane_cnt = max(1.0, float(snap["lanes"]))
    queue_pressure = max(0.0, float(snap["halts"]) + 0.5 * max(0.0, float(snap["veh"]) - float(snap["halts"])))
    pressure_norm = min(2.0, queue_pressure / max(1.0, 6.0 * lane_cnt))
    arrival_load = min(2.0, max(0.0, float(horizon_sec)) / 20.0)
    sat_gap = max(0.02, float(queue_S) - float(queue_A))
    unstable = min(2.0, max(0.0, (0.2 - sat_gap) / 0.2))
    scale = (
        1.0
        + float(getattr(self.cfg, "offer_metric_cost_pressure_weight", 0.60)) * pressure_norm
        + float(getattr(self.cfg, "offer_metric_cost_arrival_weight", 0.40)) * arrival_load
        + 0.25 * unstable
    )
    out = max(0.0, base * scale)

    # Penalize no-action choices when EV-approach queue-clearing time is high.
    if str(action) == "none":
        q_thr = max(0.0, float(getattr(self.cfg, "offer_metric_no_action_q_threshold_sec", 3.0)))
        if float(queue_Q) > q_thr:
            out += float(getattr(self.cfg, "offer_metric_no_action_cost_penalty_veh_sec", 35.0)) * min(
                2.0, (float(queue_Q) - q_thr) / max(1e-6, q_thr)
            )
        out += 0.05 * max(0.0, float(queue_delay_sec)) * max(1.0, queue_pressure)
    return float(out)


def _ia_recommended_speed_range_mps_improved(
    self: "IntersectionAgent",
    ev: EvRequest,
    now: float,
    green_window: Tuple[float, float],
    arrival_window_eff: Optional[Tuple[float, float]] = None,
) -> Optional[Tuple[float, float]]:
    base = self._recommended_speed_range_mps(ev, now, green_window)
    if traci is None:
        return base
    dist = float(ev.distance_to_intersection_m)
    if dist <= 1.0:
        return base
    g0, g1 = float(green_window[0]), float(green_window[1])
    if g1 <= float(now) + 1e-6:
        return None

    dt_fast = max(0.1, g0 - float(now))
    dt_slow = max(0.1, g1 - float(now))
    v_req_fast = dist / dt_fast
    v_req_slow = dist / dt_slow

    try:
        lane_id = traci.vehicle.getLaneID(str(ev.ev_id))
        v_lim = float(traci.lane.getMaxSpeed(lane_id))
    except Exception:
        v_lim = 13.89

    v_cur = max(0.0, float(ev.speed_mps))
    a_max = max(0.1, float(getattr(self.cfg, "offer_metric_speed_max_accel_mps2", 2.5)))
    d_max = max(0.1, float(getattr(self.cfg, "offer_metric_speed_max_decel_mps2", 3.5)))
    v_reach_open = max(0.0, v_cur + a_max * dt_fast)
    v_floor_close = max(0.0, v_cur - d_max * dt_slow)

    v_lo = max(0.5, v_req_slow, v_floor_close)
    v_hi = min(v_lim, v_req_fast, v_reach_open)

    if arrival_window_eff is not None:
        a0, a1 = float(arrival_window_eff[0]), float(arrival_window_eff[1])
        dt_a0 = max(0.1, a0 - float(now))
        dt_a1 = max(0.1, a1 - float(now))
        v_lo = max(v_lo, dist / dt_a1 * 0.85)
        v_hi = min(v_hi, dist / dt_a0 * 1.15)

    if v_hi < v_lo:
        return None if base is None else base
    return (float(v_lo), float(v_hi))


def _ia_store_offer_metric_components(
    self: "IntersectionAgent",
    offer_id: str,
    components: Dict[str, float],
    action: str,
    target_phase_idx: int,
    green_window: Tuple[float, float],
    arrival_window: Tuple[float, float],
) -> None:
    row = dict(components)
    row["action"] = str(action)
    row["target_phase_idx"] = float(target_phase_idx)
    row["gw0"] = float(green_window[0])
    row["gw1"] = float(green_window[1])
    row["ar0_raw"] = float(arrival_window[0])
    row["ar1_raw"] = float(arrival_window[1])
    self._offer_metric_components_by_offer[str(offer_id)] = row
    if bool(getattr(self.cfg, "enable_offer_metric_components_debug", False)):
        print(
            "[OFFER_METRIC_DEBUG] "
            f"tls={self.cfg.tls_id} "
            f"offer={offer_id} "
            f"action={action} "
            f"t_i_raw={float(row.get('t_i_raw', 0.0)):.2f} "
            f"Q={float(row.get('queue_Q', 0.0)):.2f} "
            f"N={float(row.get('queue_N', 0.0)):.2f} "
            f"A={float(row.get('queue_A', 0.0)):.3f} "
            f"S={float(row.get('queue_S', 0.0)):.3f} "
            f"delta_eff={float(row.get('delta_eff', 0.0)):.2f} "
            f"ar_eff=({float(row.get('arrival_eff_start', 0.0)):.2f},{float(row.get('arrival_eff_end', 0.0)):.2f}) "
            f"green=({float(green_window[0]):.2f},{float(green_window[1]):.2f}) "
            f"n_i={float(row.get('queue_n_i', 0.0)):.0f} "
            f"t_lost={float(row.get('queue_t_lost_sec', 0.0)):.2f} "
            f"yt={float(row.get('queue_yt_sec', 0.0)):.2f} "
            f"delta_w={float(row.get('queue_delta_w_sec', 0.0)):.2f} "
            f"avail_pre={float(row.get('queue_avail_pre_green_sec', 0.0)):.2f} "
            f"q_delay={float(row.get('queue_delay_sec', 0.0)):.2f} "
            f"wait={float(row.get('expected_wait_sec', 0.0)):.2f} "
            f"miss_prob={float(row.get('miss_prob', 0.0)):.3f} "
            f"miss_sec={float(row.get('expected_miss_sec', 0.0)):.2f} "
            f"cost={float(row.get('cost_to_others_veh_sec', 0.0)):.2f}"
        )


def _ia_regime_bins_for_offer_metrics(
    self: "IntersectionAgent",
    q_sec: float,
    dist_m: float,
    phase_match: bool,
) -> Tuple[str, str, str]:
    q = max(0.0, float(q_sec))
    if q < 3.0:
        q_bin = "q_low"
    elif q < 10.0:
        q_bin = "q_med"
    else:
        q_bin = "q_high"

    d = max(0.0, float(dist_m))
    if d < 20.0:
        d_bin = "d_near"
    elif d < 60.0:
        d_bin = "d_mid"
    else:
        d_bin = "d_far"

    p_bin = "phase_match" if bool(phase_match) else "phase_mismatch"
    return q_bin, d_bin, p_bin


def _ia_register_selected_offer_prediction(
    self: "IntersectionAgent",
    sim_time: float,
    offer: SignalWindowOffer,
) -> None:
    if not bool(getattr(self.cfg, "enable_offer_metric_calibration", False)):
        return
    ev = self.active_ev
    if ev is None:
        return
    comp = dict(self._offer_metric_components_by_offer.get(str(offer.offer_id), {}))
    if not comp:
        comp = {
            "t_i_raw": 0.5 * (float(offer.arrival_window[0]) + float(offer.arrival_window[1])),
            "queue_Q": 0.0,
            "queue_N": 0.0,
            "queue_A": 0.0,
            "queue_S": 0.0,
            "delta_eff": max(0.0, 0.5 * (float(offer.arrival_window[1]) - float(offer.arrival_window[0]))),
            "arrival_eff_start": float(offer.arrival_window[0]),
            "arrival_eff_end": float(offer.arrival_window[1]),
            "expected_wait_sec": float(offer.expected_wait_sec),
            "expected_miss_sec": float(offer.expected_miss_sec),
            "miss_prob": 0.0,
            "cost_to_others_veh_sec": float(offer.cost_to_others_veh_sec),
        }
    phase_match = False
    try:
        phase_match = int(self.current_phase) == int(offer.target_phase_idx)
    except Exception:
        phase_match = False
    q_bin, d_bin, p_bin = self._ia_regime_bins_for_offer_metrics(
        q_sec=float(comp.get("queue_Q", 0.0)),
        dist_m=float(getattr(ev, "distance_to_intersection_m", 0.0)),
        phase_match=phase_match,
    )
    non_ev_start = _ia_collect_non_ev_snapshot(self, ev_edge_override=str(ev.in_edge_id))
    self._offer_metric_selected_prediction_by_ev[str(ev.ev_id)] = {
        "offer_id": str(offer.offer_id),
        "tls_id": str(self.cfg.tls_id),
        "selected_time": float(sim_time),
        "t_i_raw": float(comp.get("t_i_raw", float(sim_time))),
        "queue_Q": float(comp.get("queue_Q", 0.0)),
        "q_bin": q_bin,
        "d_bin": d_bin,
        "p_bin": p_bin,
        "pred_wait": float(comp.get("expected_wait_sec", offer.expected_wait_sec)),
        "pred_miss": float(comp.get("expected_miss_sec", offer.expected_miss_sec)),
        "pred_cost": float(comp.get("cost_to_others_veh_sec", offer.cost_to_others_veh_sec)),
        "pred_miss_prob": float(comp.get("miss_prob", 0.0)),
        "green_start": float(offer.green_window[0]),
        "green_end": float(offer.green_window[1]),
        "non_ev_halts_start": float(non_ev_start["halts"]),
        "non_ev_veh_start": float(non_ev_start["veh"]),
    }


def _ia_update_offer_metric_error_table(
    self: "IntersectionAgent",
    q_bin: str,
    d_bin: str,
    p_bin: str,
    wait_abs_err: float,
    miss_abs_err: float,
    cost_abs_err: float,
    miss_fp: float,
    miss_fn: float,
) -> None:
    key = (str(q_bin), str(d_bin), str(p_bin))
    row = self._offer_metric_error_table.get(
        key,
        {
            "count": 0.0,
            "wait_abs_err_sum": 0.0,
            "miss_abs_err_sum": 0.0,
            "cost_abs_err_sum": 0.0,
            "miss_fp_sum": 0.0,
            "miss_fn_sum": 0.0,
        },
    )
    row["count"] += 1.0
    row["wait_abs_err_sum"] += float(wait_abs_err)
    row["miss_abs_err_sum"] += float(miss_abs_err)
    row["cost_abs_err_sum"] += float(cost_abs_err)
    row["miss_fp_sum"] += float(miss_fp)
    row["miss_fn_sum"] += float(miss_fn)
    self._offer_metric_error_table[key] = row


def _ia_write_offer_metric_calibration_row(self: "IntersectionAgent", row: Dict[str, object]) -> None:
    path = str(getattr(self.cfg, "offer_metric_calibration_csv_path", "/tmp/offer_metric_calibration.csv"))
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
    except Exception:
        pass
    fieldnames = [
        "sim_time",
        "tls_id",
        "ev_id",
        "offer_id",
        "passed",
        "q_bin",
        "d_bin",
        "p_bin",
        "pred_wait_sec",
        "actual_wait_proxy_sec",
        "pred_miss_sec",
        "actual_miss_sec",
        "pred_cost_veh_sec",
        "actual_cost_proxy_veh_sec",
        "wait_abs_err",
        "miss_abs_err",
        "cost_abs_err",
        "pred_miss_prob",
        "actual_miss_flag",
        "selected_time",
        "duration_sec",
        "queue_Q_sec",
    ]
    write_header = (not bool(self._offer_metric_calibration_header_written)) or (not os.path.exists(path))
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            w.writeheader()
            self._offer_metric_calibration_header_written = True
        out = {k: row.get(k, "") for k in fieldnames}
        w.writerow(out)


def _ia_finalize_selected_offer_calibration(
    self: "IntersectionAgent",
    sim_time: float,
    passed: bool,
) -> None:
    if not bool(getattr(self.cfg, "enable_offer_metric_calibration", False)):
        return
    ev = self.active_ev
    if ev is None:
        return
    pred = self._offer_metric_selected_prediction_by_ev.pop(str(ev.ev_id), None)
    if pred is None:
        return

    t_now = float(sim_time)
    sel_t = float(pred.get("selected_time", t_now))
    duration = max(0.0, t_now - sel_t)
    non_ev_end = _ia_collect_non_ev_snapshot(self, ev_edge_override=str(ev.in_edge_id))
    avg_halts = 0.5 * (float(pred.get("non_ev_halts_start", 0.0)) + float(non_ev_end.get("halts", 0.0)))
    actual_cost_proxy = max(0.0, avg_halts * duration)
    actual_wait_proxy = max(0.0, t_now - float(pred.get("t_i_raw", sel_t)))
    g_end = float(pred.get("green_end", t_now))
    actual_miss_sec = max(0.0, t_now - g_end)
    actual_miss_flag = 1.0 if actual_miss_sec > 1e-3 else 0.0

    pred_wait = max(0.0, float(pred.get("pred_wait", 0.0)))
    pred_miss = max(0.0, float(pred.get("pred_miss", 0.0)))
    pred_cost = max(0.0, float(pred.get("pred_cost", 0.0)))
    wait_abs_err = abs(pred_wait - actual_wait_proxy)
    miss_abs_err = abs(pred_miss - actual_miss_sec)
    cost_abs_err = abs(pred_cost - actual_cost_proxy)

    pred_miss_flag = 1.0 if float(pred.get("pred_miss_prob", 0.0)) > 0.5 else 0.0
    miss_fp = 1.0 if pred_miss_flag > 0.5 and actual_miss_flag < 0.5 else 0.0
    miss_fn = 1.0 if pred_miss_flag < 0.5 and actual_miss_flag > 0.5 else 0.0

    self._ia_update_offer_metric_error_table(
        q_bin=str(pred.get("q_bin", "q_low")),
        d_bin=str(pred.get("d_bin", "d_far")),
        p_bin=str(pred.get("p_bin", "phase_match")),
        wait_abs_err=float(wait_abs_err),
        miss_abs_err=float(miss_abs_err),
        cost_abs_err=float(cost_abs_err),
        miss_fp=float(miss_fp),
        miss_fn=float(miss_fn),
    )

    row = {
        "sim_time": t_now,
        "tls_id": str(pred.get("tls_id", self.cfg.tls_id)),
        "ev_id": str(ev.ev_id),
        "offer_id": str(pred.get("offer_id", "")),
        "passed": int(bool(passed)),
        "q_bin": str(pred.get("q_bin", "q_low")),
        "d_bin": str(pred.get("d_bin", "d_far")),
        "p_bin": str(pred.get("p_bin", "phase_match")),
        "pred_wait_sec": pred_wait,
        "actual_wait_proxy_sec": float(actual_wait_proxy),
        "pred_miss_sec": pred_miss,
        "actual_miss_sec": float(actual_miss_sec),
        "pred_cost_veh_sec": pred_cost,
        "actual_cost_proxy_veh_sec": float(actual_cost_proxy),
        "wait_abs_err": float(wait_abs_err),
        "miss_abs_err": float(miss_abs_err),
        "cost_abs_err": float(cost_abs_err),
        "pred_miss_prob": float(pred.get("pred_miss_prob", 0.0)),
        "actual_miss_flag": float(actual_miss_flag),
        "selected_time": float(sel_t),
        "duration_sec": float(duration),
        "queue_Q_sec": float(pred.get("queue_Q", 0.0)),
    }
    _ia_write_offer_metric_calibration_row(self, row)
    if bool(getattr(self.cfg, "enable_offer_metric_components_debug", False)):
        print(
            "[OFFER_CALIB_DEBUG] "
            f"tls={self.cfg.tls_id} "
            f"ev={ev.ev_id} "
            f"offer={row['offer_id']} "
            f"pred(wait/miss/cost)=({pred_wait:.2f},{pred_miss:.2f},{pred_cost:.2f}) "
            f"actual(wait/miss/cost)=({actual_wait_proxy:.2f},{actual_miss_sec:.2f},{actual_cost_proxy:.2f}) "
            f"abs_err=({wait_abs_err:.2f},{miss_abs_err:.2f},{cost_abs_err:.2f}) "
            f"bins=({row['q_bin']},{row['d_bin']},{row['p_bin']}) "
            f"passed={int(bool(passed))}"
        )


def _ia_print_offer_metric_error_table(self: "IntersectionAgent") -> None:
    if not self._offer_metric_error_table:
        print(f"[OFFER_CALIB_TABLE] tls={self.cfg.tls_id} empty")
        return
    print(f"[OFFER_CALIB_TABLE] tls={self.cfg.tls_id} rows={len(self._offer_metric_error_table)}")
    for key in sorted(self._offer_metric_error_table.keys()):
        q_bin, d_bin, p_bin = key
        row = self._offer_metric_error_table[key]
        n = max(1.0, float(row.get("count", 0.0)))
        print(
            "[OFFER_CALIB_TABLE] "
            f"bin=({q_bin},{d_bin},{p_bin}) "
            f"n={int(n)} "
            f"mae_wait={float(row.get('wait_abs_err_sum', 0.0))/n:.3f} "
            f"mae_miss={float(row.get('miss_abs_err_sum', 0.0))/n:.3f} "
            f"mae_cost={float(row.get('cost_abs_err_sum', 0.0))/n:.3f} "
            f"fp_rate={float(row.get('miss_fp_sum', 0.0))/n:.3f} "
            f"fn_rate={float(row.get('miss_fn_sum', 0.0))/n:.3f}"
        )


def _ia_compute_offer_metrics_improved(
    self: "IntersectionAgent",
    sim_time: float,
    ev: EvRequest,
    target_phase_idx: int,
    action: str,
    action_params: Dict[str, float],
    green_window: Tuple[float, float],
    arrival_window: Tuple[float, float],
) -> Dict[str, float]:
    gw0, gw1 = float(green_window[0]), float(green_window[1])
    ar0, ar1 = float(arrival_window[0]), float(arrival_window[1])
    t_i_raw = 0.5 * (ar0 + ar1)
    delta_raw = max(0.1, 0.5 * (ar1 - ar0))

    qc = {
        "N": 0.0,
        "A": 0.0,
        "S": 0.0,
        "Q": 0.0,
        "queue_indicator": 0.0,
        "cycle_sec": float(getattr(self.cfg, "queue_metrics_cycle_fallback_sec", 90.0)),
        "n_i": 0.0,
        "t_lost_sec": float(getattr(self.cfg, "T_lost_sec", 5.0)),
        "yt_sec": float(getattr(self.cfg, "YT_sec", 5.0)),
        "delta_w_sec": 0.0,
        "avail_pre_green_sec": 0.0,
        "q_delay_sec": 0.0,
    }
    use_paper_q = bool(getattr(self.cfg, "offer_metric_use_paper_queue_clearing", True))
    try:
        if use_paper_q and bool(getattr(self.cfg, "queue_metrics_enable_improved", True)):
            qc = self._queue_clearing_metrics_improved(
                edge_id=str(ev.in_edge_id),
                ev_id=str(ev.ev_id),
                sim_time=float(sim_time),
                t_i=float(t_i_raw),
                green_window=(gw0, gw1),
                arrival_window=(ar0, ar1),
            )
        else:
            N0, A0, S0, Q0, _ = self._queue_clearing_metrics_for_edge(edge_id=str(ev.in_edge_id), ev_id=str(ev.ev_id))
            qc.update({"N": float(N0), "A": float(A0), "S": float(S0), "Q": float(Q0)})
            q_delay_legacy = self._estimate_ev_queue_delay_for_offer(
                ev=ev,
                sim_time=float(sim_time),
                green_window=(gw0, gw1),
                arrival_window=(ar0, ar1),
            )
            qc["q_delay_sec"] = float(q_delay_legacy)
    except Exception:
        pass

    N = float(qc.get("N", 0.0))
    A = float(qc.get("A", 0.0))
    S = float(qc.get("S", 0.0))
    Q = float(qc.get("Q", 0.0))
    q_delay = max(0.0, float(qc.get("q_delay_sec", 0.0)))

    rho = max(0.0, min(2.0, (float(A) / max(1e-6, float(S))) if float(S) > 0 else 0.0))
    if bool(getattr(self.cfg, "offer_metric_use_delta_scaling", False)):
        q_norm = min(2.0, max(0.0, float(Q)) / max(1e-6, float(getattr(self.cfg, "offer_metric_q_scale_sec", 60.0))))
        delta_scale = 1.0 + float(getattr(self.cfg, "offer_metric_rho_scale", 0.8)) * rho + 0.5 * q_norm
        delta_eff = max(
            float(getattr(self.cfg, "offer_metric_delta_min_sec", 1.0)),
            min(float(getattr(self.cfg, "offer_metric_delta_max_sec", 8.0)), float(delta_raw * delta_scale)),
        )
    else:
        delta_eff = float(delta_raw)

    use_t_eff = bool(getattr(self.cfg, "offer_metric_use_t_eff", False))
    t_i_eff = float(t_i_raw + q_delay) if use_t_eff else float(t_i_raw)
    eff_ar0 = float(t_i_eff - delta_eff)
    eff_ar1 = float(t_i_eff + delta_eff)

    e_wait, p_miss, e_late = self._ia_expected_wait_miss_from_windows_improved(
        arrival_window=(eff_ar0, eff_ar1),
        green_window=(gw0, gw1),
        queue_ratio=float(rho),
        queue_delay_sec=float(q_delay),
    )

    horizon_anchor = t_i_eff if use_t_eff else t_i_raw
    horizon_sec = max(0.0, float(horizon_anchor - float(sim_time)))
    cost_veh_sec = self._ia_estimate_non_ev_cost_veh_sec_improved(
        action=str(action),
        action_params=dict(action_params or {}),
        target_phase=int(target_phase_idx),
        horizon_sec=float(horizon_sec),
        ev_edge_override=str(ev.in_edge_id),
        queue_N=float(N),
        queue_A=float(A),
        queue_S=float(S),
        queue_Q=float(Q),
        queue_delay_sec=float(q_delay),
    )

    speed_rng = self._ia_recommended_speed_range_mps_improved(
        ev=ev,
        now=float(sim_time),
        green_window=(gw0, gw1),
        arrival_window_eff=(eff_ar0, eff_ar1),
    )
    overlap = _ia_overlap_sec(eff_ar0, eff_ar1, gw0, gw1)
    cov = overlap / max(1e-6, (eff_ar1 - eff_ar0))

    if bool(getattr(self.cfg, "ev_state_debug_on_offer_calc", True)):
        try:
            self._print_ev_state_debug(
                sim_time=float(sim_time),
                context="offer_eval",
                ev=ev,
                t_i=float(t_i_raw),
                action=str(action),
                target_phase_idx=int(target_phase_idx),
            )
        except Exception:
            pass

    return {
        "t_i_raw": float(t_i_raw),
        "t_i_eff": float(t_i_eff),
        "queue_delay_sec": float(q_delay),
        "queue_N": float(N),
        "queue_A": float(A),
        "queue_S": float(S),
        "queue_Q": float(Q),
        "queue_indicator": float(qc.get("queue_indicator", 0.0)),
        "queue_cycle_sec": float(qc.get("cycle_sec", 0.0)),
        "queue_n_i": float(qc.get("n_i", 0.0)),
        "queue_t_lost_sec": float(qc.get("t_lost_sec", float(getattr(self.cfg, "T_lost_sec", 5.0)))),
        "queue_yt_sec": float(qc.get("yt_sec", float(getattr(self.cfg, "YT_sec", 5.0)))),
        "queue_delta_w_sec": float(qc.get("delta_w_sec", 0.0)),
        "queue_avail_pre_green_sec": float(qc.get("avail_pre_green_sec", 0.0)),
        "queue_rho": float(rho),
        "delta_raw": float(delta_raw),
        "delta_eff": float(delta_eff),
        "use_t_eff": 1.0 if use_t_eff else 0.0,
        "use_paper_queue_clearing": 1.0 if use_paper_q else 0.0,
        "arrival_eff_start": float(eff_ar0),
        "arrival_eff_end": float(eff_ar1),
        "window_coverage": float(max(0.0, min(1.0, cov))),
        "expected_wait_sec": float(e_wait),
        "miss_prob": float(max(0.0, min(1.0, p_miss))),
        "expected_miss_sec": float(max(0.0, e_late)),
        "cost_to_others_veh_sec": float(max(0.0, cost_veh_sec)),
        "speed_min_mps": None if speed_rng is None else float(speed_rng[0]),
        "speed_max_mps": None if speed_rng is None else float(speed_rng[1]),
    }


# Bind improved helpers onto IntersectionAgent at import time.
IntersectionAgent._expected_wait_miss_from_windows_improved = _ia_expected_wait_miss_from_windows_improved
IntersectionAgent._estimate_non_ev_cost_veh_sec_improved = _ia_estimate_non_ev_cost_veh_sec_improved
IntersectionAgent._recommended_speed_range_mps_improved = _ia_recommended_speed_range_mps_improved
IntersectionAgent._ia_expected_wait_miss_from_windows_improved = _ia_expected_wait_miss_from_windows_improved
IntersectionAgent._ia_estimate_non_ev_cost_veh_sec_improved = _ia_estimate_non_ev_cost_veh_sec_improved
IntersectionAgent._ia_recommended_speed_range_mps_improved = _ia_recommended_speed_range_mps_improved
IntersectionAgent._ia_store_offer_metric_components = _ia_store_offer_metric_components
IntersectionAgent._ia_regime_bins_for_offer_metrics = _ia_regime_bins_for_offer_metrics
IntersectionAgent._regime_bins_for_offer_metrics = _ia_regime_bins_for_offer_metrics
IntersectionAgent._ia_update_offer_metric_error_table = _ia_update_offer_metric_error_table
IntersectionAgent.print_offer_metric_error_table = _ia_print_offer_metric_error_table
IntersectionAgent._compute_offer_metrics_improved = _ia_compute_offer_metrics_improved
IntersectionAgent._store_offer_metric_components = _ia_store_offer_metric_components
IntersectionAgent._register_selected_offer_prediction = _ia_register_selected_offer_prediction
IntersectionAgent._finalize_selected_offer_calibration = _ia_finalize_selected_offer_calibration
