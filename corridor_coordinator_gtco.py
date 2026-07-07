#!/usr/bin/env python3
"""
Golden-Time Corridor Orchestrator (GTCO)

Paper-inspired corridor manager for emergency-vehicle green-wave coordination.
This service is designed to interoperate with the current IntersectionAgent F2
federation by consuming existing MQTT federation events and publishing corridor
advice (and optional arbitration verdicts).

Modes
-----
- observe: diagnostics only (no control messages)
- advisory: publish non-binding corridor guidance to intersections
- arbitration: publish bounded verdicts for hard reservation requests

Key existing topics consumed (from the current environment)
-----------------------------------------------------------
- federation/reservation/req/<tls>
- federation/reservation/resp/<tls>
- federation/handoff/<tls>
- rw/vehicle/<ev_id>/state
- rw/vehicle_agent/<ev_id>/state   (optional; for ERL/severity hints)
- rw/tls/<tls_id>/state            (optional; corridor observability)
- rw/agent/<tls_id>/warmup_plan    (optional; corridor observability)
- rw/step                          (optional; global sim clock)

New topics published by this coordinator
----------------------------------------
- federation/corridor/advice/<tls_id>
- federation/corridor/verdict/<tls_id>     (arbitration mode)
- federation/corridor/state/<corridor_id>  (optional summaries)
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Tuple

try:
    import paho.mqtt.client as mqtt
except Exception as e:  # pragma: no cover
    print(f"[GTCO][ERR] Missing paho-mqtt dependency: {e}", file=sys.stderr)
    sys.exit(2)


def _make_mqtt_client(client_id: str):
    kwargs = {"client_id": str(client_id or "").strip()} if str(client_id or "").strip() else {}
    protocol = str(os.environ.get("FNM_MQTT_PROTOCOL") or os.environ.get("FED_MQTT_PROTOCOL") or "").strip().lower()
    if protocol in {"mqttv5", "v5", "5"} and hasattr(mqtt, "MQTTv5"):
        kwargs["protocol"] = mqtt.MQTTv5
    elif protocol in {"mqttv311", "v311", "3.1.1", "311"} and hasattr(mqtt, "MQTTv311"):
        kwargs["protocol"] = mqtt.MQTTv311
    elif protocol in {"mqttv31", "v31", "3.1", "31"} and hasattr(mqtt, "MQTTv31"):
        kwargs["protocol"] = mqtt.MQTTv31
    cb_versions = getattr(mqtt, "CallbackAPIVersion", None)
    cb_api = getattr(cb_versions, "VERSION1", None)
    if cb_api is not None:
        try:
            return mqtt.Client(cb_api, **kwargs)
        except TypeError:
            pass
    try:
        return mqtt.Client(**kwargs)
    except TypeError:
        cb_api = getattr(cb_versions, "VERSION2", None)
        if cb_api is not None:
            try:
                return mqtt.Client(cb_api, **kwargs)
            except TypeError:
                pass
        if "client_id" in kwargs:
            return mqtt.Client(kwargs["client_id"])
        return mqtt.Client()


def _now_wall() -> float:
    return time.time()


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _safe_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _norm_window(a: float, b: float) -> Tuple[float, float]:
    if a <= b:
        return float(a), float(b)
    return float(b), float(a)


def _topic_match(prefix: str, topic: str) -> bool:
    return str(topic).startswith(str(prefix))


def _basename_topic(topic: str) -> str:
    try:
        return str(topic).split("/")[-1]
    except Exception:
        return str(topic)


@dataclass
class VehicleState:
    ev_id: str
    sim_time: float
    edge_id: str = ""
    speed_mps: float = 0.0
    lane_index: int = -1
    x: float = 0.0
    y: float = 0.0
    last_update_wall: float = field(default_factory=_now_wall)

    @staticmethod
    def from_rw_vehicle_state(d: Dict[str, Any]) -> "VehicleState":
        return VehicleState(
            ev_id=str(d.get("vehId", "")),
            sim_time=_safe_float(d.get("simTime", 0.0)),
            edge_id=str(d.get("edge", "") or ""),
            speed_mps=_safe_float(d.get("speed", 0.0)),
            lane_index=_safe_int(d.get("laneIndex", -1)),
            x=_safe_float(d.get("x", 0.0)),
            y=_safe_float(d.get("y", 0.0)),
            last_update_wall=_now_wall(),
        )


@dataclass
class VehicleProfileState:
    ev_id: str
    erl_level: int = 1
    profile: Dict[str, Any] = field(default_factory=dict)
    snapshot: Dict[str, Any] = field(default_factory=dict)
    last_update_wall: float = field(default_factory=_now_wall)

    @staticmethod
    def from_vehicle_agent_state(d: Dict[str, Any]) -> "VehicleProfileState":
        prof = dict(d.get("profile", {}) or {})
        ev_id = str(prof.get("evId", "") or prof.get("ev_id", "") or prof.get("id", ""))
        return VehicleProfileState(
            ev_id=ev_id,
            erl_level=_safe_int(prof.get("erlLevel", prof.get("erl_level", 1)), 1),
            profile=prof,
            snapshot=dict(d.get("snapshot", {}) or {}),
            last_update_wall=_now_wall(),
        )


@dataclass
class ReservationReqEvent:
    req_id: str
    ev_id: str
    from_tls: str
    to_tls: str
    mode: str
    eta_start: float
    eta_end: float
    confidence: float
    hard: bool
    soft: bool
    ttl_sec: float
    current_edge_id: Optional[str]
    next_edge_id: Optional[str]
    preferred_next_tls: Optional[str]
    route_intersections: List[str]
    route_veh: List[str]
    sim_time_hint: Optional[float] = None
    ts_wall: float = field(default_factory=_now_wall)

    @staticmethod
    def from_payload(msg: Dict[str, Any]) -> "ReservationReqEvent":
        eta_start, eta_end = _norm_window(
            _safe_float(msg.get("eta_start", 0.0)),
            _safe_float(msg.get("eta_end", 0.0)),
        )
        hard = bool(msg.get("hard", False))
        soft = bool(msg.get("soft", not hard))
        mode = str(msg.get("mode", "hard" if hard else "soft"))
        return ReservationReqEvent(
            req_id=str(msg.get("req_id", "")),
            ev_id=str(msg.get("ev_id", "")),
            from_tls=str(msg.get("from_tls", "")),
            to_tls=str(msg.get("to_tls", "")),
            mode=mode,
            eta_start=eta_start,
            eta_end=eta_end,
            confidence=_safe_float(msg.get("confidence", 0.0)),
            hard=hard,
            soft=soft,
            ttl_sec=max(0.5, _safe_float(msg.get("ttl_sec", 5.0))),
            current_edge_id=(str(msg.get("current_edge_id", "") or "") or None),
            next_edge_id=(str(msg.get("next_edge_id", "") or "") or None),
            preferred_next_tls=(str(msg.get("preferred_next_tls", "") or "") or None),
            route_intersections=[str(x) for x in list(msg.get("route_intersections", []) or []) if str(x)],
            route_veh=[str(x) for x in list(msg.get("route_veh", []) or []) if str(x)],
            sim_time_hint=msg.get("sim_time", None),
            ts_wall=_now_wall(),
        )


@dataclass
class ReservationRespEvent:
    req_id: str
    ev_id: str
    responder_tls: str
    to_tls: str
    status: str
    reason: str
    mode: str
    req_eta_start: float
    req_eta_end: float
    downstream_queue_margin_sec: float = 0.0
    downstream_spillback_risk: float = 0.0
    downstream_readiness_score: float = 0.0
    downstream_suggested_eta_shift_sec: float = 0.0
    sim_time_hint: Optional[float] = None
    ts_wall: float = field(default_factory=_now_wall)

    @staticmethod
    def from_payload(msg: Dict[str, Any]) -> "ReservationRespEvent":
        a, b = _norm_window(
            _safe_float(msg.get("req_eta_start", 0.0)),
            _safe_float(msg.get("req_eta_end", 0.0)),
        )
        return ReservationRespEvent(
            req_id=str(msg.get("req_id", "")),
            ev_id=str(msg.get("ev_id", "")),
            responder_tls=str(msg.get("from_tls", "")),
            to_tls=str(msg.get("to_tls", "")),
            status=str(msg.get("status", "UNKNOWN")),
            reason=str(msg.get("reason", "")),
            mode=str(msg.get("mode", "")),
            req_eta_start=a,
            req_eta_end=b,
            downstream_queue_margin_sec=_safe_float(msg.get("downstream_queue_margin_sec", 0.0)),
            downstream_spillback_risk=_safe_float(msg.get("downstream_spillback_risk", 0.0)),
            downstream_readiness_score=_safe_float(msg.get("downstream_readiness_score", 0.0)),
            downstream_suggested_eta_shift_sec=_safe_float(msg.get("downstream_suggested_eta_shift_sec", 0.0)),
            sim_time_hint=(msg.get("sim_time") if "sim_time" in msg else None),
            ts_wall=_now_wall(),
        )


@dataclass
class HandoffEvent:
    ev_id: str
    from_tls: str
    to_tls: str
    confidence: float
    eta: float
    sim_time: float
    route_intersections: List[str]
    route_veh: List[str]
    preferred_next_tls: Optional[str]
    current_edge_id: Optional[str]
    next_edge_id: Optional[str]
    pass_time: Optional[float]
    pass_detect_time: Optional[float]
    pass_proxy_time: Optional[float]
    pass_from_edge_id: Optional[str]
    pass_to_edge_id: Optional[str]
    ts_wall: float = field(default_factory=_now_wall)

    @staticmethod
    def from_payload(msg: Dict[str, Any]) -> "HandoffEvent":
        def _optf(k: str) -> Optional[float]:
            if k not in msg or msg.get(k) is None:
                return None
            return _safe_float(msg.get(k))

        def _opts(k: str) -> Optional[str]:
            v = str(msg.get(k, "") or "")
            return v or None

        return HandoffEvent(
            ev_id=str(msg.get("ev_id", "")),
            from_tls=str(msg.get("from_tls", "")),
            to_tls=str(msg.get("to_tls", "")),
            confidence=_safe_float(msg.get("confidence", 0.0)),
            eta=_safe_float(msg.get("eta", 0.0)),
            sim_time=_safe_float(msg.get("sim_time", 0.0)),
            route_intersections=[str(x) for x in list(msg.get("route_intersections", []) or []) if str(x)],
            route_veh=[str(x) for x in list(msg.get("route_veh", []) or []) if str(x)],
            preferred_next_tls=_opts("preferred_next_tls"),
            current_edge_id=_opts("current_edge_id"),
            next_edge_id=_opts("next_edge_id"),
            pass_time=_optf("pass_time"),
            pass_detect_time=_optf("pass_detect_time"),
            pass_proxy_time=_optf("pass_proxy_time"),
            pass_from_edge_id=_opts("pass_from_edge_id"),
            pass_to_edge_id=_opts("pass_to_edge_id"),
            ts_wall=_now_wall(),
        )


@dataclass
class WarmupEvent:
    tls_id: str
    sim_time: float
    plan: Dict[str, Any]
    ts_wall: float = field(default_factory=_now_wall)


@dataclass
class CorridorAssociation:
    assoc_id: str
    ev_id: str
    tls_id: str
    priority_score: float
    eta_start: float
    eta_end: float
    phase_state: str = "phi_init"
    source_mode: str = "observe"
    route_index: int = -1
    preferred_next_tls: Optional[str] = None
    next_edge_id: Optional[str] = None
    current_edge_id: Optional[str] = None
    created_ts_wall: float = field(default_factory=_now_wall)
    updated_ts_wall: float = field(default_factory=_now_wall)
    expires_ts_wall: float = field(default_factory=lambda: _now_wall() + 20.0)
    last_req_id: Optional[str] = None
    last_resp_status: Optional[str] = None
    last_resp_reason: Optional[str] = None
    queue_margin_sec: float = 0.0
    spillback_risk: float = 0.0
    readiness_score: float = 0.0


@dataclass
class EVMission:
    ev_id: str
    severity_level: int = 1  # lower ERL means more urgent in your current system
    incident_recency_ts_wall: float = field(default_factory=_now_wall)
    first_seen_ts_wall: float = field(default_factory=_now_wall)
    last_seen_ts_wall: float = field(default_factory=_now_wall)
    last_sim_time: float = 0.0
    current_tls: Optional[str] = None
    current_edge_id: Optional[str] = None
    next_edge_id: Optional[str] = None
    route_intersections: List[str] = field(default_factory=list)
    route_veh: List[str] = field(default_factory=list)
    preferred_next_tls: Optional[str] = None
    last_handoff: Optional[HandoffEvent] = None
    active_assocs: Dict[str, CorridorAssociation] = field(default_factory=dict)  # tls_id -> assoc
    hard_rejects_recent: int = 0
    hard_accepts_recent: int = 0


class GTCO:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.mode = str(args.mode).strip().lower()
        if self.mode not in ("observe", "advisory", "arbitration"):
            raise ValueError(f"Invalid mode: {self.mode}")

        self.corridor_tls = [x.strip() for x in str(args.corridor_tls).split(",") if x.strip()]
        self.corridor_set = set(self.corridor_tls)
        if not self.corridor_tls:
            raise ValueError("corridor_tls cannot be empty")
        self.corridor_id = str(args.corridor_id or ("-".join(self.corridor_tls)))
        self.ev_filter: Optional[str] = None if str(args.ev_id).strip() in ("", "*", "all") else str(args.ev_id).strip()

        self.reassess_period_sec = float(args.reassess_period)
        self.assoc_ttl_sec = float(args.assoc_ttl_sec)
        self.lookahead_hops = int(args.lookahead_hops)
        self.advice_ttl_sec = float(args.advice_ttl_sec)
        self.state_publish_period_sec = float(args.state_publish_period_sec)
        self.log_file = str(args.log_file or "").strip()
        self.log_jsonl = bool(args.log_jsonl)
        self.print_debug = bool(args.verbose)
        self.publish_state = bool(args.publish_state)

        self.activation_distance_m = float(args.activation_distance_m)
        self.phase_transition_sec = float(args.phase_transition_sec)
        self.arb_fail_open = bool(args.arb_fail_open)
        self.arb_hard_only = bool(args.arb_hard_only)
        self.arb_max_eta_shift_sec = float(args.arb_max_eta_shift_sec)
        self.hard_reject_threshold = int(args.arb_hard_reject_threshold)
        self.spillback_alert_threshold = float(args.spillback_alert_threshold)
        self.queue_margin_alert_threshold = float(args.queue_margin_alert_threshold)

        self.severity_weight = float(args.severity_weight)
        self.recency_weight = float(args.recency_weight)
        self.deadline_weight = float(args.deadline_weight)
        self.route_conf_weight = float(args.route_conf_weight)
        self.bottleneck_weight = float(args.bottleneck_weight)

        self.fed_req_prefix = str(args.fed_req_prefix).rstrip("/")
        self.fed_resp_prefix = str(args.fed_resp_prefix).rstrip("/")
        self.fed_handoff_prefix = str(args.fed_handoff_prefix).rstrip("/")
        self.vehicle_state_prefix = str(args.vehicle_state_prefix).rstrip("/")
        self.vehicle_agent_prefix = str(args.vehicle_agent_prefix).rstrip("/")
        self.tls_state_prefix = str(args.tls_state_prefix).rstrip("/")
        self.agent_prefix = str(args.agent_prefix).rstrip("/")
        self.step_topic = str(args.step_topic)
        self.corridor_advice_prefix = str(args.corridor_advice_prefix).rstrip("/")
        self.corridor_verdict_prefix = str(args.corridor_verdict_prefix).rstrip("/")
        self.corridor_state_prefix = str(args.corridor_state_prefix).rstrip("/")

        self.missions: Dict[str, EVMission] = {}
        self.vehicle_states: Dict[str, VehicleState] = {}
        self.vehicle_profiles: Dict[str, VehicleProfileState] = {}
        self.tls_states: Dict[str, Dict[str, Any]] = {}

        self.req_by_id: Dict[str, ReservationReqEvent] = {}
        self.resp_by_req_id: Dict[str, ReservationRespEvent] = {}
        self.recent_req_order: List[str] = []
        self.recent_resp_order: List[str] = []
        self.handoffs_recent: List[HandoffEvent] = []
        self.warmups_recent: List[WarmupEvent] = []
        self.last_sim_time: float = 0.0

        self._last_reassess_wall = 0.0
        self._last_state_pub_wall = 0.0
        self._last_advice_sent: Dict[Tuple[str, str], float] = {}  # (ev,tls) -> wall_ts
        self._last_verdict_sent: Dict[str, float] = {}  # req_id -> wall_ts

        self.hostname = socket.gethostname()
        self.instance_id = f"gtco-{os.getpid()}-{uuid.uuid4().hex[:8]}"
        mqtt_client_id = f"gtco{uuid.uuid4().hex[:12]}"

        self.client = _make_mqtt_client(mqtt_client_id)
        self.client.on_message = self._on_message
        self.client.on_connect = self._on_connect
        self.client.connect(args.mqtt_host, int(args.mqtt_port), 60)

        if self.log_file and bool(args.log_reset):
            parent = os.path.dirname(self.log_file) or "."
            os.makedirs(parent, exist_ok=True)
            with open(self.log_file, "w", encoding="utf-8") as f:
                f.write("# GTCO log\n")

    # -------------------------
    # Logging / debug
    # -------------------------
    def _log(self, msg: str, **kv: Any) -> None:
        wall = _now_wall()
        line = f"[GTCO] {msg}"
        if kv:
            suffix = " ".join(f"{k}={kv[k]}" for k in sorted(kv))
            line = f"{line} {suffix}"
        if self.print_debug:
            print(line)
        if self.log_file:
            try:
                if self.log_jsonl:
                    payload = {
                        "ts_wall": wall,
                        "instance": self.instance_id,
                        "corridor_id": self.corridor_id,
                        "msg": msg,
                        **kv,
                    }
                    out = json.dumps(payload, sort_keys=True)
                else:
                    out = line
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(out + "\n")
            except Exception:
                pass

    # -------------------------
    # MQTT
    # -------------------------
    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        self._log("connected", host=self.args.mqtt_host, rc=str(reason_code))
        subs = [
            (f"{self.fed_req_prefix}/+", 0),
            (f"{self.fed_resp_prefix}/+", 0),
            (f"{self.fed_handoff_prefix}/+", 0),
            (f"{self.vehicle_state_prefix}/+/state", 0),
            (f"{self.vehicle_agent_prefix}/+/state", 0),
            (f"{self.tls_state_prefix}/+/state", 0),
            (f"{self.agent_prefix}/+/warmup_plan", 0),
            (self.step_topic, 0),
        ]
        for topic, qos in subs:
            try:
                client.subscribe(topic, qos=qos)
            except Exception as e:
                self._log("subscribe_error", topic=topic, err=f"{type(e).__name__}:{e}")

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return

        try:
            if _topic_match(f"{self.fed_req_prefix}/", topic):
                self._handle_reservation_req(payload, topic)
                return
            if _topic_match(f"{self.fed_resp_prefix}/", topic):
                self._handle_reservation_resp(payload, topic)
                return
            if _topic_match(f"{self.fed_handoff_prefix}/", topic):
                self._handle_handoff(payload, topic)
                return
            if _topic_match(f"{self.vehicle_state_prefix}/", topic) and topic.endswith("/state"):
                self._handle_vehicle_state(payload, topic)
                return
            if _topic_match(f"{self.vehicle_agent_prefix}/", topic) and topic.endswith("/state"):
                self._handle_vehicle_agent_state(payload, topic)
                return
            if _topic_match(f"{self.tls_state_prefix}/", topic) and topic.endswith("/state"):
                self._handle_tls_state(payload, topic)
                return
            if _topic_match(f"{self.agent_prefix}/", topic) and topic.endswith("/warmup_plan"):
                self._handle_warmup_plan(payload, topic)
                return
            if topic == self.step_topic:
                self.last_sim_time = _safe_float(payload.get("simTime", self.last_sim_time))
                return
        except Exception as e:
            self._log("message_handler_error", topic=topic, err=f"{type(e).__name__}:{e}")

    # -------------------------
    # Topic handlers
    # -------------------------
    def _ev_selected(self, ev_id: str) -> bool:
        return (self.ev_filter is None) or (str(ev_id) == str(self.ev_filter))

    def _get_mission(self, ev_id: str) -> EVMission:
        m = self.missions.get(ev_id)
        if m is None:
            m = EVMission(ev_id=ev_id)
            self.missions[ev_id] = m
            self._log("mission_create", ev_id=ev_id)
        m.last_seen_ts_wall = _now_wall()
        return m

    def _handle_vehicle_state(self, payload: Dict[str, Any], _topic: str) -> None:
        st = VehicleState.from_rw_vehicle_state(payload)
        if not st.ev_id or not self._ev_selected(st.ev_id):
            return
        self.vehicle_states[st.ev_id] = st
        self.last_sim_time = max(self.last_sim_time, float(st.sim_time))
        mission = self._get_mission(st.ev_id)
        mission.last_sim_time = max(mission.last_sim_time, float(st.sim_time))
        mission.current_edge_id = st.edge_id or mission.current_edge_id

    def _handle_vehicle_agent_state(self, payload: Dict[str, Any], _topic: str) -> None:
        st = VehicleProfileState.from_vehicle_agent_state(payload)
        if not st.ev_id or not self._ev_selected(st.ev_id):
            return
        self.vehicle_profiles[st.ev_id] = st
        mission = self._get_mission(st.ev_id)
        mission.severity_level = max(1, int(st.erl_level))

    def _handle_tls_state(self, payload: Dict[str, Any], topic: str) -> None:
        tls_id = _basename_topic(topic[:-len("/state")] if topic.endswith("/state") else topic)
        if tls_id in self.corridor_set:
            self.tls_states[tls_id] = dict(payload)

    def _handle_warmup_plan(self, payload: Dict[str, Any], topic: str) -> None:
        tls_id = _basename_topic(topic[:-len("/warmup_plan")] if topic.endswith("/warmup_plan") else topic)
        if tls_id not in self.corridor_set:
            return
        ev_id = str((payload.get("plan", {}) or {}).get("notes", ""))
        evt = WarmupEvent(
            tls_id=tls_id,
            sim_time=_safe_float(payload.get("simTime", self.last_sim_time)),
            plan=dict(payload.get("plan", {}) or {}),
        )
        self.warmups_recent.append(evt)
        if len(self.warmups_recent) > 500:
            self.warmups_recent = self.warmups_recent[-500:]
        self._log("warmup_seen", tls_id=tls_id, sim_time=f"{evt.sim_time:.2f}", note_hint=ev_id[:64])

    def _handle_handoff(self, payload: Dict[str, Any], _topic: str) -> None:
        evt = HandoffEvent.from_payload(payload)
        if not evt.ev_id or not self._ev_selected(evt.ev_id):
            return
        self.last_sim_time = max(self.last_sim_time, float(evt.sim_time))
        self.handoffs_recent.append(evt)
        if len(self.handoffs_recent) > 2000:
            self.handoffs_recent = self.handoffs_recent[-2000:]

        mission = self._get_mission(evt.ev_id)
        mission.last_handoff = evt
        mission.current_tls = evt.to_tls or mission.current_tls
        mission.current_edge_id = evt.current_edge_id or mission.current_edge_id
        mission.next_edge_id = evt.next_edge_id or mission.next_edge_id
        mission.preferred_next_tls = evt.preferred_next_tls or mission.preferred_next_tls
        if evt.route_intersections:
            mission.route_intersections = list(evt.route_intersections)
        if evt.route_veh:
            mission.route_veh = list(evt.route_veh)
        mission.last_sim_time = max(mission.last_sim_time, float(evt.sim_time))

        self._log(
            "handoff_in",
            ev=evt.ev_id,
            from_tls=evt.from_tls,
            to_tls=evt.to_tls,
            conf=f"{evt.confidence:.3f}",
            eta=f"{evt.eta:.2f}",
            next_edge=(evt.next_edge_id or "-"),
            preferred=(evt.preferred_next_tls or "-"),
        )

        # Update association phases using handoff as strong progress signal.
        if evt.from_tls in mission.active_assocs:
            a = mission.active_assocs[evt.from_tls]
            a.phase_state = "phi_final"
            a.updated_ts_wall = _now_wall()
        if evt.to_tls and evt.to_tls in self.corridor_set:
            self._ensure_assoc_from_handoff(mission, evt)

    def _handle_reservation_req(self, payload: Dict[str, Any], _topic: str) -> None:
        evt = ReservationReqEvent.from_payload(payload)
        if not evt.ev_id or not self._ev_selected(evt.ev_id):
            return
        if evt.from_tls not in self.corridor_set and evt.to_tls not in self.corridor_set:
            return

        self.req_by_id[evt.req_id] = evt
        self.recent_req_order.append(evt.req_id)
        if len(self.recent_req_order) > 5000:
            drop = self.recent_req_order[:-3000]
            self.recent_req_order = self.recent_req_order[-3000:]
            for rid in drop:
                self.req_by_id.pop(rid, None)

        mission = self._get_mission(evt.ev_id)
        if evt.route_intersections:
            mission.route_intersections = list(evt.route_intersections)
        if evt.route_veh:
            mission.route_veh = list(evt.route_veh)
        mission.preferred_next_tls = evt.preferred_next_tls or mission.preferred_next_tls
        mission.current_edge_id = evt.current_edge_id or mission.current_edge_id
        mission.next_edge_id = evt.next_edge_id or mission.next_edge_id

        self._ensure_assoc_from_req(mission, evt)
        self._log(
            "req_in",
            req_id=evt.req_id,
            ev=evt.ev_id,
            from_tls=evt.from_tls,
            to_tls=evt.to_tls,
            mode=evt.mode,
            eta=f"({evt.eta_start:.2f},{evt.eta_end:.2f})",
            conf=f"{evt.confidence:.3f}",
            next_edge=(evt.next_edge_id or "-"),
            preferred=(evt.preferred_next_tls or "-"),
        )

        if self.mode == "arbitration" and (evt.hard or not self.arb_hard_only):
            self._publish_arbitration_verdict(evt, mission)

    def _handle_reservation_resp(self, payload: Dict[str, Any], _topic: str) -> None:
        evt = ReservationRespEvent.from_payload(payload)
        if not evt.ev_id or not self._ev_selected(evt.ev_id):
            return
        self.resp_by_req_id[evt.req_id] = evt
        self.recent_resp_order.append(evt.req_id)
        if len(self.recent_resp_order) > 5000:
            self.recent_resp_order = self.recent_resp_order[-3000:]

        mission = self._get_mission(evt.ev_id)
        if evt.mode == "hard":
            if str(evt.status).upper() == "ACCEPTED":
                mission.hard_accepts_recent += 1
            elif str(evt.status).upper() in ("REJECTED", "ERROR"):
                mission.hard_rejects_recent += 1

        # Bind response to association if the target TLS is in corridor.
        req = self.req_by_id.get(evt.req_id)
        if req and req.to_tls in mission.active_assocs:
            a = mission.active_assocs[req.to_tls]
            a.last_req_id = evt.req_id
            a.last_resp_status = evt.status
            a.last_resp_reason = evt.reason
            a.queue_margin_sec = float(evt.downstream_queue_margin_sec)
            a.spillback_risk = float(evt.downstream_spillback_risk)
            a.readiness_score = float(evt.downstream_readiness_score)
            a.updated_ts_wall = _now_wall()
            if str(evt.status).upper() == "ACCEPTED":
                if a.phase_state == "phi_init":
                    a.phase_state = "phi_progress"
            elif str(evt.status).upper() == "REJECTED" and a.phase_state == "phi_progress":
                # Back off to init if downstream deteriorates.
                a.phase_state = "phi_init"

        self._log(
            "resp_in",
            req_id=evt.req_id,
            ev=evt.ev_id,
            responder=evt.responder_tls,
            to_tls=evt.to_tls,
            status=evt.status,
            reason=(evt.reason or "-"),
            mode=evt.mode,
            q_margin=f"{evt.downstream_queue_margin_sec:.2f}",
            spill=f"{evt.downstream_spillback_risk:.2f}",
            readiness=f"{evt.downstream_readiness_score:.2f}",
        )

    # -------------------------
    # Association management
    # -------------------------
    def _route_index_for_tls(self, mission: EVMission, tls_id: str) -> int:
        if not mission.route_intersections:
            return -1
        try:
            return [str(x) for x in mission.route_intersections].index(str(tls_id))
        except ValueError:
            return -1

    def _urgency_score(self, mission: EVMission) -> float:
        # In your system ERL=1 is highest urgency.
        sev_component = (5.0 - _clamp(float(mission.severity_level), 1.0, 4.0)) / 4.0
        age_sec = max(0.0, _now_wall() - mission.incident_recency_ts_wall)
        recency_component = 1.0 / (1.0 + age_sec / 30.0)
        route_conf = 0.0
        if mission.last_handoff is not None:
            route_conf = _clamp(float(mission.last_handoff.confidence), 0.0, 1.0)
        deadline_slack_component = 0.0  # placeholder for future ERS deadline integration
        score = (
            self.severity_weight * sev_component
            + self.recency_weight * recency_component
            + self.route_conf_weight * route_conf
            + self.deadline_weight * deadline_slack_component
        )
        return float(score)

    def _ensure_assoc_from_req(self, mission: EVMission, evt: ReservationReqEvent) -> None:
        if evt.to_tls not in self.corridor_set:
            return
        ridx = self._route_index_for_tls(mission, evt.to_tls)
        assoc = mission.active_assocs.get(evt.to_tls)
        if assoc is None:
            assoc = CorridorAssociation(
                assoc_id=f"{mission.ev_id}:{evt.to_tls}:{int(_now_wall()*1000)}",
                ev_id=mission.ev_id,
                tls_id=evt.to_tls,
                priority_score=self._urgency_score(mission),
                eta_start=float(evt.eta_start),
                eta_end=float(evt.eta_end),
                phase_state="phi_init",
                source_mode=self.mode,
                route_index=ridx,
                preferred_next_tls=evt.preferred_next_tls,
                next_edge_id=evt.next_edge_id,
                current_edge_id=evt.current_edge_id,
                expires_ts_wall=_now_wall() + max(self.assoc_ttl_sec, evt.ttl_sec),
                last_req_id=evt.req_id,
            )
            mission.active_assocs[evt.to_tls] = assoc
            self._log("assoc_create", ev=mission.ev_id, tls=evt.to_tls, phase=assoc.phase_state, req_id=evt.req_id)
        else:
            assoc.priority_score = self._urgency_score(mission)
            assoc.eta_start = float(evt.eta_start)
            assoc.eta_end = float(evt.eta_end)
            assoc.preferred_next_tls = evt.preferred_next_tls or assoc.preferred_next_tls
            assoc.next_edge_id = evt.next_edge_id or assoc.next_edge_id
            assoc.current_edge_id = evt.current_edge_id or assoc.current_edge_id
            assoc.route_index = ridx if ridx >= 0 else assoc.route_index
            assoc.updated_ts_wall = _now_wall()
            assoc.expires_ts_wall = max(assoc.expires_ts_wall, _now_wall() + max(self.assoc_ttl_sec, evt.ttl_sec))
            assoc.last_req_id = evt.req_id
            if assoc.phase_state == "phi_finished":
                assoc.phase_state = "phi_init"

    def _ensure_assoc_from_handoff(self, mission: EVMission, evt: HandoffEvent) -> None:
        # Create/update association for the downstream handoff target using point ETA.
        if evt.to_tls not in self.corridor_set:
            return
        eta_start, eta_end = _norm_window(evt.eta - 1.8, evt.eta + 1.8)
        ridx = self._route_index_for_tls(mission, evt.to_tls)
        assoc = mission.active_assocs.get(evt.to_tls)
        if assoc is None:
            assoc = CorridorAssociation(
                assoc_id=f"{mission.ev_id}:{evt.to_tls}:{int(_now_wall()*1000)}",
                ev_id=mission.ev_id,
                tls_id=evt.to_tls,
                priority_score=self._urgency_score(mission),
                eta_start=float(eta_start),
                eta_end=float(eta_end),
                phase_state="phi_init",
                source_mode=self.mode,
                route_index=ridx,
                preferred_next_tls=evt.preferred_next_tls,
                next_edge_id=evt.next_edge_id,
                current_edge_id=evt.current_edge_id,
                expires_ts_wall=_now_wall() + self.assoc_ttl_sec,
            )
            mission.active_assocs[evt.to_tls] = assoc
            self._log("assoc_create_handoff", ev=mission.ev_id, tls=evt.to_tls, eta=f"{evt.eta:.2f}")
        else:
            assoc.eta_start = eta_start
            assoc.eta_end = eta_end
            assoc.priority_score = self._urgency_score(mission)
            assoc.updated_ts_wall = _now_wall()
            assoc.expires_ts_wall = max(assoc.expires_ts_wall, _now_wall() + self.assoc_ttl_sec)
            if assoc.phase_state in ("phi_final", "phi_finished"):
                assoc.phase_state = "phi_progress"

    # -------------------------
    # Decision loop
    # -------------------------
    def run_forever(self) -> None:
        self.client.loop_start()
        self._log(
            "start",
            mode=self.mode,
            corridor_id=self.corridor_id,
            corridor=",".join(self.corridor_tls),
            ev_filter=self.ev_filter or "*",
            host=self.hostname,
            mqtt_host=self.args.mqtt_host,
        )
        try:
            while True:
                self._step()
                time.sleep(0.05)
        finally:
            try:
                self.client.loop_stop()
            except Exception:
                pass

    def _step(self) -> None:
        now = _now_wall()
        if (now - self._last_reassess_wall) >= self.reassess_period_sec:
            self._last_reassess_wall = now
            self._reassess_corridor_state()
            if self.mode in ("advisory", "arbitration"):
                self._publish_advice_cycle()

        if self.publish_state and (now - self._last_state_pub_wall) >= self.state_publish_period_sec:
            self._last_state_pub_wall = now
            self._publish_corridor_state()

    def _reassess_corridor_state(self) -> None:
        now = _now_wall()
        for ev_id, mission in list(self.missions.items()):
            # stale mission pruning
            if (now - mission.last_seen_ts_wall) > float(self.args.mission_stale_sec):
                self._log("mission_prune", ev=ev_id, age=f"{(now - mission.last_seen_ts_wall):.1f}")
                self.missions.pop(ev_id, None)
                continue

            # Decay rolling counts to avoid permanent memory.
            mission.hard_rejects_recent = max(0, int(mission.hard_rejects_recent * 0.95))
            mission.hard_accepts_recent = max(0, int(mission.hard_accepts_recent * 0.95))

            vstate = self.vehicle_states.get(ev_id)
            if vstate is not None:
                mission.current_edge_id = vstate.edge_id or mission.current_edge_id
                mission.last_sim_time = max(mission.last_sim_time, vstate.sim_time)

            # Update / prune associations
            for tls_id, assoc in list(mission.active_assocs.items()):
                if now > assoc.expires_ts_wall:
                    assoc.phase_state = "phi_finished"
                if assoc.phase_state == "phi_finished" and (now - assoc.updated_ts_wall) > 2.0:
                    mission.active_assocs.pop(tls_id, None)
                    self._log("assoc_remove", ev=ev_id, tls=tls_id)
                    continue

                assoc.priority_score = self._urgency_score(mission)

                # Phase transitions (paper-inspired lifecycle, driven by runtime evidence).
                if assoc.last_resp_status and str(assoc.last_resp_status).upper() == "ACCEPTED":
                    if assoc.phase_state == "phi_init":
                        assoc.phase_state = "phi_progress"
                if assoc.last_resp_status and str(assoc.last_resp_status).upper() == "REJECTED":
                    if str(assoc.last_resp_reason or "") in ("downstream_queue_not_clearing", "downstream_spillback_risk"):
                        assoc.phase_state = "phi_init"
                # If handoff already moved beyond this TLS, progress to final.
                if mission.last_handoff is not None and str(mission.last_handoff.from_tls) == str(tls_id):
                    assoc.phase_state = "phi_final"

                assoc.updated_ts_wall = now

    # -------------------------
    # Bottleneck / corridor analytics
    # -------------------------
    def _recent_responses_for_tls(self, tls_id: str, max_age_sec: float = 20.0) -> List[ReservationRespEvent]:
        now = _now_wall()
        out: List[ReservationRespEvent] = []
        for req_id in reversed(self.recent_resp_order[-500:]):
            evt = self.resp_by_req_id.get(req_id)
            if evt is None:
                continue
            if (now - evt.ts_wall) > max_age_sec:
                continue
            if str(evt.responder_tls) != str(tls_id):
                continue
            out.append(evt)
        return out

    def _bottleneck_snapshot_for_tls(self, tls_id: str) -> Dict[str, float]:
        rs = self._recent_responses_for_tls(tls_id, max_age_sec=20.0)
        if not rs:
            return {
                "hard_reject_ratio": 0.0,
                "hard_accept_ratio": 0.0,
                "mean_queue_margin_sec": 0.0,
                "max_spillback_risk": 0.0,
                "severity": 0.0,
            }
        hard = [x for x in rs if str(x.mode).lower() == "hard"]
        hard_acc = [x for x in hard if str(x.status).upper() == "ACCEPTED"]
        hard_rej = [x for x in hard if str(x.status).upper() == "REJECTED"]
        mean_q = sum(float(x.downstream_queue_margin_sec) for x in rs) / max(1, len(rs))
        max_sp = max(float(x.downstream_spillback_risk) for x in rs)
        hard_rej_ratio = (len(hard_rej) / max(1, len(hard))) if hard else 0.0
        hard_acc_ratio = (len(hard_acc) / max(1, len(hard))) if hard else 0.0
        severity = max(
            0.0,
            hard_rej_ratio
            + max(0.0, -mean_q) / 5.0
            + max(0.0, max_sp - 0.5),
        )
        return {
            "hard_reject_ratio": float(hard_rej_ratio),
            "hard_accept_ratio": float(hard_acc_ratio),
            "mean_queue_margin_sec": float(mean_q),
            "max_spillback_risk": float(max_sp),
            "severity": float(severity),
        }

    def _corridor_bottleneck(self) -> Tuple[Optional[str], Dict[str, float]]:
        best_tls = None
        best_snap: Dict[str, float] = {}
        best_score = -1.0
        for tls_id in self.corridor_tls:
            snap = self._bottleneck_snapshot_for_tls(tls_id)
            score = float(snap.get("severity", 0.0))
            if score > best_score:
                best_score = score
                best_tls = tls_id
                best_snap = snap
        return best_tls, best_snap

    # -------------------------
    # Advisory mode
    # -------------------------
    def _publish_advice_cycle(self) -> None:
        now = _now_wall()
        bottleneck_tls, bottleneck_snap = self._corridor_bottleneck()
        for ev_id, mission in list(self.missions.items()):
            if not mission.active_assocs:
                continue
            # Focus on next lookahead associations in corridor order.
            assocs = list(mission.active_assocs.values())
            assocs.sort(key=lambda a: (a.route_index if a.route_index >= 0 else 9999, a.eta_start))
            for assoc in assocs[: max(1, self.lookahead_hops)]:
                key = (ev_id, assoc.tls_id)
                last_sent = self._last_advice_sent.get(key, 0.0)
                if (now - last_sent) < max(0.1, self.args.advice_min_repeat_sec):
                    continue

                advice = self._build_advice_payload(mission, assoc, bottleneck_tls, bottleneck_snap)
                self.client.publish(f"{self.corridor_advice_prefix}/{assoc.tls_id}", json.dumps(advice))
                self._last_advice_sent[key] = now
                self._log(
                    "advice_pub",
                    ev=ev_id,
                    tls=assoc.tls_id,
                    phase=assoc.phase_state,
                    bottleneck=(bottleneck_tls or "-"),
                    conf=f"{_safe_float(advice.get('confidence', 0.0)):.2f}",
                )

    def _build_advice_payload(
        self,
        mission: EVMission,
        assoc: CorridorAssociation,
        bottleneck_tls: Optional[str],
        bottleneck_snap: Dict[str, float],
    ) -> Dict[str, Any]:
        bottleneck_is_target = (str(bottleneck_tls) == str(assoc.tls_id)) if bottleneck_tls else False
        spill = float(bottleneck_snap.get("max_spillback_risk", 0.0)) if bottleneck_is_target else 0.0
        q_margin = float(bottleneck_snap.get("mean_queue_margin_sec", 0.0)) if bottleneck_is_target else 0.0
        hard_rej_ratio = float(bottleneck_snap.get("hard_reject_ratio", 0.0)) if bottleneck_is_target else 0.0

        # ETA shift heuristic: if bottleneck severe and queue margin negative, suggest delaying hard push slightly.
        eta_shift = 0.0
        if bottleneck_is_target:
            eta_shift = min(
                self.arb_max_eta_shift_sec,
                max(0.0, -q_margin) * 0.25 + max(0.0, spill - 0.6) * 2.0,
            )
        hard_thr_override = None
        if not bottleneck_is_target and assoc.preferred_next_tls and str(assoc.tls_id) == str(assoc.preferred_next_tls):
            hard_thr_override = max(0.45, min(0.70, 0.55 + 0.10 * hard_rej_ratio))
        elif bottleneck_is_target:
            hard_thr_override = min(0.90, 0.65 + 0.20 * hard_rej_ratio + 0.10 * max(0.0, spill - 0.7))

        confidence = _clamp(assoc.priority_score / max(0.1, (self.severity_weight + self.recency_weight + self.route_conf_weight + self.deadline_weight)), 0.0, 1.0)

        return {
            "schema": "corridor.v1",
            "msg_type": "advice",
            "msg_id": f"{self.instance_id}:adv:{mission.ev_id}:{assoc.tls_id}:{int(self.last_sim_time*10)}",
            "corridor_id": self.corridor_id,
            "sim_time": float(self.last_sim_time),
            "ttl_sec": float(self.advice_ttl_sec),
            "ev_id": mission.ev_id,
            "source": self.instance_id,
            "target_tls": assoc.tls_id,
            "reason_codes": [
                "paper_gtco_assoc_state",
                "corridor_bottleneck_detected" if bottleneck_tls else "no_bottleneck_signal",
            ],
            "confidence": float(confidence),
            "route_guidance": {
                "preferred_next_tls": assoc.preferred_next_tls or mission.preferred_next_tls,
                "next_edge_id": assoc.next_edge_id or mission.next_edge_id,
            },
            "reservation_guidance": {
                "soft_topk": 2 if bottleneck_is_target else 3,
                "hard_target_tls": assoc.tls_id,
                "hard_conf_threshold_override": hard_thr_override,
                "eta_shift_sec": float(eta_shift),
            },
            "warmup_guidance": {
                "enable": True,
                "lead_horizon_sec": float(max(10.0, min(40.0, self.args.warmup_lead_horizon_sec + (eta_shift * 4.0)))),
                "priority": "high" if assoc.phase_state in ("phi_init", "phi_progress") else "normal",
                "hard_only": bool(self.args.warmup_hard_only),
            },
            "downstream_snapshot": {
                "bottleneck_tls": bottleneck_tls,
                "spillback_risk": float(spill),
                "queue_margin_sec": float(q_margin),
                "hard_reject_ratio": float(hard_rej_ratio),
            },
            "assoc": {
                "assoc_id": assoc.assoc_id,
                "phase_state": assoc.phase_state,
                "eta_start": float(assoc.eta_start),
                "eta_end": float(assoc.eta_end),
            },
        }

    # -------------------------
    # Arbitration (bounded)
    # -------------------------
    def _publish_arbitration_verdict(self, evt: ReservationReqEvent, mission: EVMission) -> None:
        if not evt.req_id or evt.req_id in self._last_verdict_sent:
            return
        target_tls = evt.to_tls
        snap = self._bottleneck_snapshot_for_tls(target_tls)
        hard_rej_ratio = float(snap.get("hard_reject_ratio", 0.0))
        mean_q = float(snap.get("mean_queue_margin_sec", 0.0))
        max_sp = float(snap.get("max_spillback_risk", 0.0))

        verdict = "ALLOW"
        reason = "local_f2_fallback"
        eta_shift_sec = 0.0

        severe_queue = mean_q < self.queue_margin_alert_threshold
        severe_spill = max_sp > self.spillback_alert_threshold

        # Use both corridor history and local request confidence.
        if severe_spill or severe_queue:
            if hard_rej_ratio >= 0.6:
                verdict = "DEFER"
                reason = "corridor_bottleneck_hard_reject_trend"
            else:
                verdict = "RETIME"
                reason = "corridor_bottleneck_eta_shift"
                eta_shift_sec = min(
                    self.arb_max_eta_shift_sec,
                    max(0.5, max(0.0, -mean_q) * 0.2 + max(0.0, max_sp - 0.6) * 1.5),
                )
        if evt.confidence < 0.35 and verdict == "ALLOW":
            verdict = "DEFER"
            reason = "low_route_confidence"

        if verdict == "DEFER" and self.arb_fail_open:
            # Advisory-style softening if fail-open is active.
            verdict = "RETIME"
            reason = f"{reason}_fail_open"
            eta_shift_sec = max(eta_shift_sec, 0.8)

        sender_tls = evt.from_tls or ""
        if not sender_tls:
            return

        payload = {
            "schema": "corridor.v1",
            "msg_type": "verdict",
            "msg_id": f"{self.instance_id}:verdict:{evt.req_id}",
            "corridor_id": self.corridor_id,
            "sim_time": float(self.last_sim_time),
            "ttl_sec": float(self.advice_ttl_sec),
            "ev_id": evt.ev_id,
            "source": self.instance_id,
            "req_id": evt.req_id,
            "target_tls": sender_tls,
            "verdict": verdict,
            "reason": reason,
            "constraints": {
                "eta_shift_sec": float(eta_shift_sec),
                "prefer_soft_until": float(self.last_sim_time + max(0.0, eta_shift_sec)),
            },
            "evidence": {
                "hard_reject_ratio": float(hard_rej_ratio),
                "mean_queue_margin_sec": float(mean_q),
                "max_spillback_risk": float(max_sp),
                "request_confidence": float(evt.confidence),
                "severity_level": int(mission.severity_level),
            },
        }
        self.client.publish(f"{self.corridor_verdict_prefix}/{sender_tls}", json.dumps(payload))
        self._last_verdict_sent[evt.req_id] = _now_wall()
        self._log(
            "verdict_pub",
            req_id=evt.req_id,
            ev=evt.ev_id,
            sender=sender_tls,
            dst=evt.to_tls,
            verdict=verdict,
            reason=reason,
            shift=f"{eta_shift_sec:.2f}",
        )

    # -------------------------
    # Corridor state publishing
    # -------------------------
    def _publish_corridor_state(self) -> None:
        bottleneck_tls, bottleneck_snap = self._corridor_bottleneck()
        active_missions = []
        for ev_id, mission in self.missions.items():
            if not mission.active_assocs:
                continue
            route_head = mission.route_intersections[:3] if mission.route_intersections else []
            active_missions.append(
                {
                    "ev_id": ev_id,
                    "severity_level": int(mission.severity_level),
                    "current_tls": mission.current_tls,
                    "current_edge_id": mission.current_edge_id,
                    "preferred_next_tls": mission.preferred_next_tls,
                    "route_head": route_head,
                    "n_assocs": len(mission.active_assocs),
                    "hard_accepts_recent": int(mission.hard_accepts_recent),
                    "hard_rejects_recent": int(mission.hard_rejects_recent),
                }
            )

        payload = {
            "schema": "corridor.v1",
            "msg_type": "state",
            "corridor_id": self.corridor_id,
            "source": self.instance_id,
            "sim_time": float(self.last_sim_time),
            "mode": self.mode,
            "corridor_tls": list(self.corridor_tls),
            "n_missions": len(active_missions),
            "missions": active_missions,
            "top_bottleneck_tls": bottleneck_tls,
            "top_bottleneck": bottleneck_snap,
            "ts_wall": _now_wall(),
        }
        self.client.publish(f"{self.corridor_state_prefix}/{self.corridor_id}", json.dumps(payload))
        self._log(
            "state_pub",
            n_missions=len(active_missions),
            bottleneck=(bottleneck_tls or "-"),
            severity=f"{float(bottleneck_snap.get('severity', 0.0)):.2f}",
        )


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Golden-Time Corridor Orchestrator (GTCO)")

    # Core runtime
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--mode", choices=["observe", "advisory", "arbitration"], default="observe")
    ap.add_argument("--corridor-id", default="", help="logical corridor identifier")
    ap.add_argument("--corridor-tls", required=True, help="comma-separated TLS IDs in corridor order")
    ap.add_argument("--ev-id", default="*", help="specific EV to track, or '*' for all")

    # Timing / cadence
    ap.add_argument("--reassess-period", type=float, default=0.5, help="GTCO reassessment cadence (s)")
    ap.add_argument("--state-publish-period-sec", type=float, default=1.0)
    ap.add_argument("--publish-state", action="store_true", help="publish corridor state summaries")
    ap.add_argument("--assoc-ttl-sec", type=float, default=20.0)
    ap.add_argument("--lookahead-hops", type=int, default=3)
    ap.add_argument("--mission-stale-sec", type=float, default=30.0)

    # Paper-inspired knobs (informational + guidance heuristics)
    ap.add_argument("--activation-distance-m", type=float, default=300.0)
    ap.add_argument("--phase-transition-sec", type=float, default=8.0)
    ap.add_argument("--warmup-lead-horizon-sec", type=float, default=25.0)
    ap.add_argument("--warmup-hard-only", action="store_true")

    # Advisory / arbitration controls
    ap.add_argument("--advice-ttl-sec", type=float, default=2.0)
    ap.add_argument("--advice-min-repeat-sec", type=float, default=0.5)
    ap.add_argument("--arb-fail-open", action="store_true", default=True)
    ap.add_argument("--no-arb-fail-open", dest="arb_fail_open", action="store_false")
    ap.add_argument("--arb-hard-only", action="store_true", default=True)
    ap.add_argument("--no-arb-hard-only", dest="arb_hard_only", action="store_false")
    ap.add_argument("--arb-max-eta-shift-sec", type=float, default=3.0)
    ap.add_argument("--arb-hard-reject-threshold", type=int, default=3)
    ap.add_argument("--spillback-alert-threshold", type=float, default=0.85)
    ap.add_argument("--queue-margin-alert-threshold", type=float, default=-2.0)

    # Priority scoring (EDF-inspired with severity/recency)
    ap.add_argument("--severity-weight", type=float, default=2.0)
    ap.add_argument("--recency-weight", type=float, default=1.0)
    ap.add_argument("--deadline-weight", type=float, default=0.0)
    ap.add_argument("--route-conf-weight", type=float, default=1.0)
    ap.add_argument("--bottleneck-weight", type=float, default=1.0)

    # Topic prefixes (aligned with current environment)
    ap.add_argument("--fed-req-prefix", default="federation/reservation/req")
    ap.add_argument("--fed-resp-prefix", default="federation/reservation/resp")
    ap.add_argument("--fed-handoff-prefix", default="federation/handoff")
    ap.add_argument("--vehicle-state-prefix", default="rw/vehicle")
    ap.add_argument("--vehicle-agent-prefix", default="rw/vehicle_agent")
    ap.add_argument("--tls-state-prefix", default="rw/tls")
    ap.add_argument("--agent-prefix", default="rw/agent")
    ap.add_argument("--step-topic", default="rw/step")
    ap.add_argument("--corridor-advice-prefix", default="federation/corridor/advice")
    ap.add_argument("--corridor-verdict-prefix", default="federation/corridor/verdict")
    ap.add_argument("--corridor-state-prefix", default="federation/corridor/state")

    # Logging
    ap.add_argument("--log-file", default="", help="optional GTCO log file")
    ap.add_argument("--log-reset", action="store_true", help="reset GTCO log file at startup")
    ap.add_argument("--log-jsonl", action="store_true", help="write JSONL logs instead of plain text")
    ap.add_argument("--verbose", action="store_true", help="print debug logs to console")

    return ap.parse_args()


def main() -> None:
    args = parse_args()
    coord = GTCO(args)
    coord.run_forever()


if __name__ == "__main__":
    main()
