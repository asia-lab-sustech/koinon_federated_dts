#!/usr/bin/env python3
"""
Extract defensible federation metrics from JSONL logs.

Inputs can be one or more JSONL files from membership/catalog/discovery/metrics/gtco.
The script supports:
- strict canonical event taxonomy (federation_event_taxonomy_v1.md)
- backward-compatible alias mapping for older event names
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import statistics
from typing import Any, Dict, Iterable, List, Optional, Tuple

_SERVICE_CANONICAL: Dict[str, str] = {
    "membership": "membership_service",
    "membership_service": "membership_service",
    "catalog": "catalog_service",
    "catalog_service": "catalog_service",
    "discovery": "discovery_service",
    "discovery_service": "discovery_service",
    "metrics": "metrics_service",
    "metrics_service": "metrics_service",
    "observer": "observer_service",
    "observer_service": "observer_service",
    "ev": "ev",
    "vehicle": "ev",
    "emergency_vehicle": "ev",
    "intersection": "intersection_agent",
    "intersection_agent": "intersection_agent",
    "tls": "intersection_agent",
    "gtco": "gtco",
    "orchestrator": "gtco",
    "coordinator": "gtco",
    "corridor": "gtco",
    "gateway": "dt_gateway",
    "dt_gateway": "dt_gateway",
    "real_world": "dt_gateway",
    "rw": "dt_gateway",
    "unknown": "unknown",
}

_DT_PLANE_NODES = {"ev", "intersection_agent", "gtco"}
_CONTROL_PLANE_NODES = {"membership_service", "catalog_service", "discovery_service", "metrics_service", "observer_service"}
_GATEWAY_NODES = {"dt_gateway"}


def _canonical_service_name(x: str) -> str:
    s = str(x or "").strip().lower()
    if not s:
        return ""
    if s in _SERVICE_CANONICAL:
        return _SERVICE_CANONICAL[s]
    if "membership" in s:
        return "membership_service"
    if "catalog" in s:
        return "catalog_service"
    if "discovery" in s:
        return "discovery_service"
    if "metric" in s:
        return "metrics_service"
    if "observer" in s:
        return "observer_service"
    if "corridor" in s or "gtco" in s or "orchestrator" in s or "coordinator" in s:
        return "gtco"
    if "intersection" in s or "tls" in s:
        return "intersection_agent"
    if s in ("ev", "vehicle", "emergency_vehicle"):
        return "ev"
    if s in ("real_world", "rw", "gateway", "dt_gateway"):
        return "dt_gateway"
    return s


def _service_from_file_hint(path: str) -> str:
    base = os.path.basename(str(path or "")).strip().lower()
    if not base:
        return ""
    if "membership" in base:
        return "membership_service"
    if "catalog" in base:
        return "catalog_service"
    if "discovery" in base:
        return "discovery_service"
    if "metrics" in base:
        return "metrics_service"
    if "observer" in base or "event_trace" in base:
        return "observer_service"
    if "gtco" in base or "corridor" in base:
        return "gtco"
    if "events" in base:
        return "intersection_agent"
    if "fed_outcomes" in base:
        return "dt_gateway"
    return ""


def _node_plane(n: str) -> str:
    x = _canonical_service_name(n)
    if x in _DT_PLANE_NODES:
        return "data"
    if x in _CONTROL_PLANE_NODES:
        return "control"
    if x in _GATEWAY_NODES:
        return "gateway"
    return "unknown"


def _derive_topic_from_event(et: str, src_service: str) -> str:
    e = str(et or "").strip().lower()
    s = _canonical_service_name(src_service or "unknown") or "unknown"
    if not e:
        return f"derived/{s}/unknown"
    if e.startswith("lifecycle.") or e.startswith("membership."):
        return "federation/membership/" + e.replace(".", "/")
    if e.startswith("catalog."):
        return "federation/catalog/" + e.replace(".", "/")
    if e.startswith("discovery."):
        return "federation/discovery/" + e.replace(".", "/")
    if e.startswith("metrics."):
        return "federation/metrics/" + e.replace(".", "/")
    if e.startswith("corridor.") or e.startswith("association."):
        return "federation/corridor/" + e.replace(".", "/")
    if e.startswith("federation."):
        return "federation/" + e.replace(".", "/")
    if e.startswith("intersection."):
        return "dt/intersection/" + e.replace(".", "/")
    if e.startswith("ev."):
        return "dt/ev/" + e.replace(".", "/")
    return f"derived/{s}/" + e.replace(".", "/").replace(":", "/")


def _read_jsonl(path: str) -> Iterable[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, start=1):
            s = line.strip()
            if not s:
                continue
            try:
                rec = json.loads(s)
                if isinstance(rec, dict):
                    rec["_line"] = i
                    rec["_file"] = path
                    rec["_raw_bytes"] = len(line.encode("utf-8", errors="ignore"))
                    yield rec
            except Exception:
                parsed = _parse_debug_text_line(s)
                if parsed is None:
                    continue
                parsed["_line"] = i
                parsed["_file"] = path
                parsed["_raw_bytes"] = len(line.encode("utf-8", errors="ignore"))
                yield parsed


_DBG_EVT_RE = re.compile(r"\bevt=([A-Za-z0-9_.:-]+)")
_DBG_T_RE = re.compile(r"\bt=([0-9]+(?:\.[0-9]+)?)")
_DBG_KV_RE = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)=([^ ]+)")


def _parse_debug_text_line(line: str) -> Optional[Dict[str, Any]]:
    if "evt=" not in line:
        return None
    m_evt = _DBG_EVT_RE.search(line)
    if not m_evt:
        return None
    evt = str(m_evt.group(1))
    rec: Dict[str, Any] = {"event_type": evt}

    m_t = _DBG_T_RE.search(line)
    if m_t:
        try:
            t_sim = float(m_t.group(1))
            rec["ts_sim_s"] = t_sim
            rec["ts_wall_ms"] = t_sim * 1000.0
        except Exception:
            pass

    for k, v in _DBG_KV_RE.findall(line):
        if k in ("evt", "t"):
            continue
        vv = str(v).strip().strip(",")
        if vv.startswith('"') and vv.endswith('"') and len(vv) >= 2:
            vv = vv[1:-1]
        rec[k] = vv

    lo = line.lower()
    if "fed_debug_main" in lo:
        rec.setdefault("source_service", "real_world")
    elif "gtco" in lo:
        rec.setdefault("source_service", "gtco")
    else:
        rec.setdefault("source_service", "debug_text")

    if str(evt).startswith("EV_"):
        rec.setdefault("role", "ev")
    elif str(evt).startswith("AGENT_") or str(evt).startswith("B1_"):
        rec.setdefault("role", "intersection")
    return rec


def _get(rec: Dict[str, Any], keys: List[str], default: Any = None) -> Any:
    for k in keys:
        if k in rec and rec[k] is not None:
            return rec[k]
    return default


def _as_ms(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        x = float(v)
    except Exception:
        return None
    # Heuristic: seconds vs milliseconds
    if x > 1e12:
        return x
    if x > 1e9:
        return x * 1000.0
    # Small values likely relative seconds; still convert for consistency.
    return x * 1000.0


def _duration_ms(rec: Dict[str, Any]) -> Optional[float]:
    # Prefer explicit millisecond fields.
    for k in (
        "duration_ms",
        "elapsed_ms",
        "compute_ms",
        "processing_ms",
        "latency_ms",
        "wall_ms",
    ):
        v = rec.get(k, None)
        try:
            if v is not None:
                x = float(v)
                if x >= 0:
                    return x
        except Exception:
            pass

    # Explicit second fields.
    for k in ("duration_s", "elapsed_s", "compute_s", "wall_s"):
        v = rec.get(k, None)
        try:
            if v is not None:
                x = float(v)
                if x >= 0:
                    return x * 1000.0
        except Exception:
            pass

    # Fallback ambiguous fields.
    for k in ("duration", "elapsed", "compute_time", "processing_time"):
        v = rec.get(k, None)
        try:
            if v is None:
                continue
            x = float(v)
            if x < 0:
                continue
            # Heuristic: sub-20 values are likely seconds in these logs.
            return x * 1000.0 if x <= 20.0 else x
        except Exception:
            pass
    return None


def _event_type(rec: Dict[str, Any], alias: Dict[str, str]) -> str:
    raw = _get(
        rec,
        [
            "event_type",
            "type",
            "event",
            "name",
            "evt",
            "msg",
        ],
        "",
    )
    raw_s = str(raw or "").strip()
    if not raw_s:
        return ""
    return alias.get(raw_s, raw_s)


def _state_to(rec: Dict[str, Any]) -> str:
    return str(
        _get(
            rec,
            ["state_to", "to_state", "new_state", "state", "membership_state"],
            "",
        )
        or ""
    )


def _state_from(rec: Dict[str, Any]) -> str:
    return str(_get(rec, ["state_from", "from_state", "old_state"], "") or "")


def _corr_key(rec: Dict[str, Any], keys: List[str]) -> str:
    vals: List[str] = []
    for k in keys:
        v = _get(rec, [k], "")
        vals.append(str(v or ""))
    return "|".join(vals)


def _stats(samples: List[float]) -> Dict[str, Any]:
    if not samples:
        return {
            "n": 0,
            "mean_ms": None,
            "median_ms": None,
            "p25_ms": None,
            "p75_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "iqr_ms": None,
            "max_ms": None,
        }
    xs = sorted(float(x) for x in samples)
    n = len(xs)

    def pct(p: float) -> float:
        if n == 1:
            return xs[0]
        idx = int(math.ceil((p / 100.0) * n)) - 1
        idx = max(0, min(idx, n - 1))
        return xs[idx]

    return {
        "n": n,
        "mean_ms": statistics.fmean(xs),
        "median_ms": statistics.median(xs),
        "p25_ms": pct(25.0),
        "p75_ms": pct(75.0),
        "p95_ms": pct(95.0),
        "p99_ms": pct(99.0),
        "iqr_ms": (pct(75.0) - pct(25.0)),
        "max_ms": xs[-1],
    }


def _default_aliases() -> Dict[str, str]:
    return {
        # Legacy -> canonical
        "discovery_query_in": "discovery.query.request",
        "discovery_query_resp": "discovery.query.response",
        "catalog_upsert": "catalog.upsert.applied",
        "catalog_refresh": "catalog.refresh",
        # Common lifecycle aliases observed in prototypes
        "membership_register_in": "lifecycle.register.request",
        "membership_register_ack": "lifecycle.register.accepted",
        "membership_state_transition": "lifecycle.state.changed",
        "member_state_changed": "lifecycle.state.changed",
        # Membership service lifecycle events (current middleware services)
        "membership_registered": "lifecycle.register.accepted",
        "membership_refreshed": "lifecycle.register.accepted",
        "membership_onboarding_started": "lifecycle.onboarding.started",
        "membership_active": "lifecycle.onboarding.completed",
        "membership_suspended": "lifecycle.suspended",
        "membership_recovered_registered": "lifecycle.resumed",
        "heartbeat_in": "health.heartbeat.received",
        "availability_alive": "health.availability.alive",
        "availability_unavailable": "health.availability.unavailable",
        "timeout_detected": "health.timeout.detected",
        # Association aliases
        "association_create_req": "association.create.request",
        "association_created": "association.created",
        "assoc_create": "association.created",
        "assoc_seed": "association.created",
        "assoc_create_handoff": "association.created",
        "assoc_remove": "association.released",
        # Corridor aliases
        "route_advice_pub": "corridor.route_advice.published",
        "route_advice_apply": "corridor.route_advice.applied",
        "advice_pub": "corridor.intersection.advice.published",
        "reservation_req_out": "federation.reservation.req.sent",
        "reservation_resp_in": "federation.reservation.resp.recv",
        # Real-world debug aliases
        "EV_TRIGGER": "ev.request.received",
        "EV_PASS_BACKFILL": "ev.pass.detected",
        "EV_PASS": "ev.pass.detected",
        "EV_ROUTE_ADVICE_SEEN": "corridor.route_advice.received",
        "corridor.route_advice.seen": "corridor.route_advice.received",
        "EV_ROUTE_APPLY_OK": "corridor.route_advice.applied",
        "EV_ROUTE_APPLY_SKIP": "corridor.route_advice.apply_skipped",
        "EV_STUCK_ENTER": "ev.stuck.enter",
        "EV_STUCK_EXIT": "ev.stuck.exit",
        "EV_ROUTE_APPLY_CHECK": "corridor.route_advice.apply_check",
        "EV_ROUTE_APPLY_GATE": "corridor.route_advice.apply_gate",
        "EV_ROUTE_APPLY_COOLDOWN": "corridor.route_advice.apply_cooldown",
        "AGENT_ACTIVATE_OK": "federation.member.active",
        "AGENT_ACTIVATE_SKIP": "federation.member.activation_skipped",
        "RX_ENQUEUE": "federation.message.enqueued",
        "RX_DISPATCH": "federation.message.dispatched",
        "route_opt_eval": "corridor.route_opt.eval",
        # Intersection-local federation aliases
        "coord.reservation.req_out": "federation.reservation.req.sent",
        "coord.reservation.req_in": "federation.reservation.req.recv",
        "coord.reservation.resp_in": "federation.reservation.resp.recv",
        "coord.reservation.req_resp_e2e": "federation.reservation.req_resp.e2e",
        "coord.reservation.req_resp.e2e": "federation.reservation.req_resp.e2e",
        "coord.reservation.req.decision": "federation.reservation.req.decision",
        "coord.apply.plan": "intersection.plan.applied",
        "agent.stage.transition": "intersection.stage.transition",
        "coord.refine.candidates": "federation.refine.candidates",
        "coord.refine.hard_req_sent": "federation.refine.hard_req.sent",
        "coord.refine.hard_req_skip": "federation.refine.hard_req.skip",
        "TLS_SIGNAL": "tls.signal.change",
        "SIGNAL_CHANGE": "tls.signal.change",
        # Decision CSV / debug aliases commonly produced by real-world/intersection logs
        "apply_plan_to_tls": "intersection.plan.applied",
        "apply_offer_to_tls": "intersection.offer.applied",
        "offer_selected": "intersection.offer.selected",
        # GTCO logs
        "req_in": "federation.reservation.req.sent",
        "resp_in": "federation.reservation.resp.recv",
        "route_advice_pub": "corridor.route_advice.published",
        # Intersection compute duration aliases
        "intersection.compute.tick.duration_ms": "intersection.compute.tick.duration",
        "intersection.compute.refine.duration_ms": "intersection.compute.refine.duration",
        "intersection.compute.apply.duration_ms": "intersection.compute.apply.duration",
        "intersection.tick.compute": "intersection.compute.tick.duration",
        "intersection.refine.compute": "intersection.compute.refine.duration",
        "intersection.apply.compute": "intersection.compute.apply.duration",
        "corridor.compute.reassess.duration": "corridor.compute.reassess.duration",
        "corridor.compute.advice.duration": "corridor.compute.advice.duration",
        "corridor.compute.route_advice_cycle.duration": "corridor.compute.route_advice_cycle.duration",
        "corridor.compute.state_pub.duration": "corridor.compute.state_pub.duration",
    }


def _load_aliases(path: Optional[str]) -> Dict[str, str]:
    base = _default_aliases()
    if not path:
        return base
    try:
        raw = json.loads(open(path, "r", encoding="utf-8").read())
        if isinstance(raw, dict):
            for k, v in raw.items():
                base[str(k)] = str(v)
    except Exception:
        pass
    return base


def _event_ts_ms(rec: Dict[str, Any]) -> Optional[float]:
    return _as_ms(
        _get(
            rec,
            [
                "ts_wall",
                "ts_wall_ms",
                "timestamp_ms",
                "ts_ms",
                "ts",
                "timestamp",
                "time",
            ],
            None,
        )
    )


def _sim_ts_s(rec: Dict[str, Any]) -> Optional[float]:
    x = _get(rec, ["ts_sim_s", "sim_time_s", "sim_time", "t_sim"], None)
    try:
        if x is None:
            return None
        return float(x)
    except Exception:
        return None


def _infer_role(rec: Dict[str, Any]) -> str:
    role = str(_get(rec, ["role", "requester_role", "source_role"], "") or "").strip()
    if not role:
        src = rec.get("source", {})
        if isinstance(src, dict):
            r2 = str(src.get("role", "") or "").strip()
            if r2:
                role = r2
    if not role:
        dt_type = str(_get(rec, ["dt_type", "digital_twin_type", "entity_type"], "") or "").strip().lower()
        src_svc = str(_get(rec, ["source_service", "service", "component"], "") or "").strip().lower()
        ev_type = str(_get(rec, ["event_type", "evt", "type"], "") or "").strip().lower()
        if dt_type in ("intersection", "traffic_light", "tls", "trafficlightsystem"):
            return "intersection"
        if dt_type in ("ev", "emergency_vehicle", "vehicle"):
            return "ev"
        if dt_type in ("orchestrator", "corridor", "coordinator"):
            return "orchestrator"
        if "intersection" in src_svc or "tls" in src_svc:
            return "intersection"
        if "corridor" in src_svc or "orchestrator" in src_svc or "coordinator" in src_svc:
            return "orchestrator"
        if src_svc in ("ev", "vehicle", "emergency_vehicle"):
            return "ev"
        if ev_type.startswith("corridor.") or ev_type.startswith("association."):
            return "orchestrator"
        if ev_type.startswith("intersection."):
            return "intersection"
        if ev_type.startswith("ev."):
            return "ev"
        return ""
    r = role.strip().lower()
    if r in ("source", "target", "peer", "owner"):
        return ""
    if r in ("trafficlightsystem", "tls", "intersection", "traffic_light"):
        return "intersection"
    if r in ("emergencyvehicle", "ev", "vehicle"):
        return "ev"
    if r in ("orchestrator", "corridor", "coordinator"):
        return "orchestrator"
    return role


def _infer_dt_id(rec: Dict[str, Any]) -> str:
    return str(
        _get(
            rec,
            ["dt_id", "requester_dt_id", "node_id", "gateway_id", "source_dt_id"],
            "",
        )
        or ""
    ).strip()


def _infer_tls_id(rec: Dict[str, Any]) -> str:
    tls = str(_get(rec, ["tls_id", "intersection_id", "tls", "recipient", "responder", "target_tls", "dst"], "") or "").strip()
    if tls:
        return tls
    tls2 = str(_get(rec, ["to_tls", "from_tls"], "") or "").strip()
    if tls2:
        return tls2
    dt = _infer_dt_id(rec)
    if dt.startswith("Node"):
        return dt
    return ""


def _infer_source_service(rec: Dict[str, Any]) -> str:
    src_raw = str(
        _get(rec, ["source_service", "service", "svc", "producer", "component"], "")
        or ""
    ).strip()
    src = _canonical_service_name(src_raw)
    if src:
        return src
    topic = str(_get(rec, ["topic"], "") or "")
    if topic.startswith("federation/membership/"):
        return "membership_service"
    if topic.startswith("federation/catalog/"):
        return "catalog_service"
    if topic.startswith("federation/discovery/"):
        return "discovery_service"
    if topic.startswith("federation/corridor/"):
        return "gtco"
    if topic.startswith("federation/reservation/"):
        return "intersection_agent"
    if topic.startswith("rw/vehicle/") or topic.startswith("rw/vehicle_agent/"):
        return "ev"
    if topic.startswith("rw/agent/"):
        return "intersection_agent"
    et = str(_get(rec, ["event_type", "evt", "type"], "") or "").strip().lower()
    if et.startswith("lifecycle.") or et.startswith("membership."):
        return "membership_service"
    if et.startswith("catalog."):
        return "catalog_service"
    if et.startswith("discovery."):
        return "discovery_service"
    if et.startswith("metrics."):
        return "metrics_service"
    if et.startswith("corridor.") or et.startswith("association."):
        return "gtco"
    if et.startswith("intersection.") or et.startswith("federation.reservation."):
        return "intersection_agent"
    if et.startswith("ev."):
        return "ev"
    fh = _service_from_file_hint(str(_get(rec, ["_file"], "") or ""))
    if fh:
        return fh
    return "unknown"


def _infer_interaction_pair(rec: Dict[str, Any], et: str) -> Tuple[str, str]:
    topic = str(_get(rec, ["topic"], "") or "")
    src = _canonical_service_name(_infer_source_service(rec) or "unknown")
    et_l = str(et or "").strip().lower()
    dst = "unknown"

    if topic.startswith("federation/membership/register"):
        return ("dt_gateway", "membership_service")
    if topic.startswith("federation/membership/heartbeat"):
        return ("dt_gateway", "membership_service")
    if topic.startswith("federation/membership/ack/"):
        return ("membership_service", "dt_gateway")
    if topic.startswith("federation/catalog/upsert"):
        return ("dt_gateway", "catalog_service")
    if topic.startswith("federation/discovery/query"):
        return ("dt_gateway", "discovery_service")
    if topic.startswith("federation/discovery/resp/"):
        return ("discovery_service", "dt_gateway")
    if topic.startswith("federation/reservation/req/"):
        return ("intersection_agent", "intersection_agent")
    if topic.startswith("federation/reservation/resp/"):
        return ("intersection_agent", "intersection_agent")
    if topic.startswith("federation/corridor/advice/"):
        return ("gtco", "intersection_agent")
    if topic.startswith("federation/corridor/verdict/"):
        return ("gtco", "intersection_agent")
    if topic.startswith("federation/corridor/route_advice/"):
        return ("gtco", "ev")
    if topic.startswith("rw/vehicle/") and topic.endswith("/route_advice"):
        return ("dt_gateway", "ev")
    if topic.startswith("rw/vehicle_agent/") and topic.endswith("/route_advice"):
        return ("dt_gateway", "ev")
    if topic.startswith("rw/agent/") and topic.endswith("/plan"):
        return ("dt_gateway", "intersection_agent")
    if topic.startswith("rw/agent/") and topic.endswith("/warmup_plan"):
        return ("dt_gateway", "intersection_agent")

    if et_l.startswith("lifecycle.register.request"):
        return ("dt_gateway", "membership_service")
    if et_l.startswith("lifecycle.register.accepted") or et_l.startswith("membership.registered"):
        return ("membership_service", "dt_gateway")
    if et_l.startswith("lifecycle.onboarding.started") or et_l.startswith("lifecycle.onboarding.completed"):
        return ("membership_service", "dt_gateway")
    if et_l.startswith("lifecycle.suspended") or et_l.startswith("membership.suspended"):
        return ("membership_service", "dt_gateway")
    if et_l.startswith("catalog.upsert"):
        if src == "catalog_service":
            return ("catalog_service", "dt_gateway")
        return ("dt_gateway", "catalog_service")
    if et_l.startswith("discovery.query.request"):
        return ("dt_gateway", "discovery_service")
    if et_l.startswith("discovery.query.response"):
        return ("discovery_service", "dt_gateway")

    if et_l.startswith("corridor.") or et_l.startswith("association."):
        dst = "intersection_agent"
        if et_l.endswith(".applied") or et_l.endswith(".apply_skipped"):
            dst = "ev"
        return ("gtco", dst)
    if et_l.startswith("discovery."):
        return ("discovery_service", "dt_gateway")
    if et_l.startswith("catalog."):
        return ("catalog_service", "dt_gateway")
    if et_l.startswith("lifecycle.") or et_l.startswith("health.") or et_l.startswith("membership."):
        return ("membership_service", "dt_gateway")
    if et_l.startswith("ev."):
        return ("ev", "intersection_agent")
    if et_l.startswith("intersection.") or et_l.startswith("federation.reservation."):
        return (src or "intersection_agent", "intersection_agent")
    return (src or "unknown", dst)


def _infer_mode_from_path(path: str) -> str:
    p = str(path or "")
    m = re.search(r"/mode_([A-Za-z0-9]+)(?:/|$)", p)
    return str(m.group(1)) if m else ""


def _infer_scenario_from_text(text: str) -> str:
    t = str(text or "")
    m = re.search(r"(scenario_[A-Za-z0-9_]*_r\d+)", t)
    return str(m.group(1)) if m else ""


def _infer_route_from_scenario(scenario_id: str) -> Optional[int]:
    s = str(scenario_id or "")
    m = re.search(r"_r(\d+)$", s)
    if not m:
        return None
    try:
        return int(m.group(1))
    except Exception:
        return None


def _pair_latency(
    starts: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]],
    key: str,
    end_wall_ms: float,
    end_sim_s: Optional[float],
    end_rec: Dict[str, Any],
    sink: List[Dict[str, Any]],
    metric_name: str,
) -> None:
    if key not in starts:
        return
    st_wall_ms, st_sim_s, st_rec = starts.pop(key)
    d_wall = end_wall_ms - st_wall_ms
    if d_wall < 0:
        return
    d_sim_ms: Optional[float] = None
    if st_sim_s is not None and end_sim_s is not None:
        d_sim_ms = (end_sim_s - st_sim_s) * 1000.0
        if d_sim_ms < 0:
            d_sim_ms = None
    sink.append(
        {
            "metric": metric_name,
            "key": key,
            "latency_wall_ms": d_wall,
            "latency_sim_ms": d_sim_ms,
            "start_ts_wall_ms": st_wall_ms,
            "end_ts_wall_ms": end_wall_ms,
            "start_ts_sim_s": st_sim_s,
            "end_ts_sim_s": end_sim_s,
            "role": _infer_role(end_rec) or _infer_role(st_rec),
            "dt_id": _infer_dt_id(end_rec) or _infer_dt_id(st_rec),
            "tls_id": _infer_tls_id(end_rec) or _infer_tls_id(st_rec),
            "source_service": _infer_source_service(end_rec) or _infer_source_service(st_rec),
            "start_file": st_rec.get("_file", ""),
            "start_line": st_rec.get("_line", 0),
            "end_file": end_rec.get("_file", ""),
            "end_line": end_rec.get("_line", 0),
        }
    )


def extract(paths: List[str], alias: Dict[str, str]) -> Dict[str, Any]:
    event_counts: Dict[str, int] = {}
    event_counts_by_role: Dict[str, Dict[str, int]] = {}
    event_counts_by_dt: Dict[str, Dict[str, int]] = {}
    message_volume_by_role: Dict[str, Dict[str, float]] = {}
    message_volume_by_dt: Dict[str, Dict[str, float]] = {}
    message_volume_by_service: Dict[str, Dict[str, float]] = {}
    service_interaction_counts: Dict[str, Dict[str, int]] = {}
    service_interaction_bytes: Dict[str, Dict[str, float]] = {}
    mqtt_topic_counts: Dict[str, int] = {}
    mqtt_topic_bytes: Dict[str, float] = {}
    mqtt_topic_time_bounds_ms: Dict[str, List[float]] = {}
    mqtt_topic_counts_by_source: Dict[str, Dict[str, int]] = {}
    mqtt_topic_counts_by_edge: Dict[str, Dict[str, int]] = {}
    mqtt_events: List[Tuple[str, float, str, int]] = []
    mqtt_topic_origin_counts: Dict[str, int] = {"raw": 0, "derived": 0}
    role_counts_by_dt: Dict[str, Dict[str, int]] = {}
    role_time_bounds_ms: Dict[str, List[float]] = {}
    communication_overhead_by_tls: Dict[str, Dict[str, int]] = {}
    communication_overhead_by_dt: Dict[str, Dict[str, int]] = {}
    # Integration-level observability (FNM sidecars + mediation flow)
    fnm_state_pull_ok = 0
    fnm_state_pull_error = 0
    fnm_req_published_total = 0
    fnm_req_published_events = 0
    fnm_route_hint_published_total = 0
    fnm_local_to_fed = 0
    fnm_fed_to_local = 0
    fnm_local_to_fed_by_rule: Dict[str, int] = {}
    fnm_fed_to_local_by_rule: Dict[str, int] = {}
    fnm_pull_nearest_tls: Dict[str, int] = {}
    fnm_overhead_rows: List[Dict[str, Any]] = []
    fnm_overhead_by_dt: Dict[str, Dict[str, List[float]]] = {}
    coordination_apply_mix: Dict[str, int] = {
        "plan_applied": 0,
        "offer_applied": 0,
        "offer_selected": 0,
        "f2_local_fallback_applied": 0,
    }
    coordination_apply_mix_by_tls: Dict[str, Dict[str, int]] = {}
    coordination_skip_reasons: Dict[str, int] = {}
    dt_time_bounds_ms: Dict[str, List[float]] = {}
    global_t_min_ms: Optional[float] = None
    global_t_max_ms: Optional[float] = None

    # Pairing caches
    register_req: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    onboard_start: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    discovery_req: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    assoc_req: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    coord_req: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    advice_pub: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    advice_seen: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_stuck_enter: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    suspend_evt: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    phase_marks: Dict[str, Dict[str, float]] = {}
    direct_compute_by_tls: Dict[str, Dict[str, List[float]]] = {}
    direct_compute_by_dt: Dict[str, Dict[str, List[float]]] = {}
    assoc_created_ts: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    assoc_last_state: Dict[str, str] = {}
    assoc_state_counts: Dict[str, int] = {}
    assoc_transition_counts: Dict[str, int] = {}
    advice_improvement_sec: List[float] = []
    coord_local_decision_start: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    coord_apply_start: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_req_to_apply_start: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_req_to_signal_start: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_req_to_apply_by_reqid: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_req_to_signal_by_reqid: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    ev_apply_pair_dropped_stale = 0
    ev_signal_pair_dropped_stale = 0
    ev_apply_pair_matched_reqid = 0
    ev_signal_pair_matched_reqid = 0
    ev_apply_pair_matched_fallback = 0
    ev_signal_pair_matched_fallback = 0
    timeline_events: List[Dict[str, Any]] = []
    timeline_limit = 8000
    timeline_interest = {
        "ev.request.in",
        "ev.request.received",
        "federation.reservation.req.sent",
        "federation.reservation.req.recv",
        "coord.reservation.req_decision",
        "federation.reservation.req.decision",
        "federation.reservation.resp.recv",
        "federation.refine.candidates",
        "federation.refine.hard_req.sent",
        "federation.refine.hard_req.skip",
        "intersection.plan.applied",
        "intersection.offer.applied",
        "tls.signal.change",
        "ev.pass.detected",
    }

    samples: List[Dict[str, Any]] = []
    discovery_hits = 0
    discovery_total = 0
    discovery_requests = 0
    discovery_responses = 0
    discovery_candidates_total = 0
    # Coordination-runtime observability (request churn and queue pressure)
    req_out_recent_1s_sim_samples: List[float] = []
    req_out_dt_prev_ms_samples: List[float] = []
    req_out_pending_n_samples: List[float] = []
    req_outbox_depth_samples: List[float] = []
    req_out_repeated_within_1s_n = 0
    req_out_observed_n = 0
    outbox_depth_peak_samples: List[float] = []
    outbox_drain_n_samples: List[float] = []
    outbox_drain_depth_peak_samples: List[float] = []
    coord_session_summaries: List[Dict[str, Any]] = []

    for p in paths:
        for rec in _read_jsonl(p):
            et = _event_type(rec, alias)
            if not et:
                continue
            event_counts[et] = event_counts.get(et, 0) + 1
            role = _infer_role(rec)
            if role:
                bucket = event_counts_by_role.setdefault(role, {})
                bucket[et] = bucket.get(et, 0) + 1
            raw_bytes = int(_get(rec, ["_raw_bytes"], 0) or 0)
            src_service = str(_infer_source_service(rec) or "").strip() or "unknown"
            src_i, dst_i = _infer_interaction_pair(rec, et)
            row_i = service_interaction_counts.setdefault(src_i, {})
            row_i[dst_i] = row_i.get(dst_i, 0) + 1
            row_b = service_interaction_bytes.setdefault(src_i, {})
            row_b[dst_i] = row_b.get(dst_i, 0.0) + float(raw_bytes)
            topic_raw = str(_get(rec, ["topic"], "") or "").strip()
            topic = topic_raw if topic_raw else _derive_topic_from_event(et, src_service)
            if topic_raw:
                mqtt_topic_origin_counts["raw"] = int(mqtt_topic_origin_counts.get("raw", 0) or 0) + 1
            elif topic:
                mqtt_topic_origin_counts["derived"] = int(mqtt_topic_origin_counts.get("derived", 0) or 0) + 1
            if topic:
                mqtt_topic_counts[topic] = mqtt_topic_counts.get(topic, 0) + 1
                mqtt_topic_bytes[topic] = mqtt_topic_bytes.get(topic, 0.0) + float(raw_bytes)
                sm = mqtt_topic_counts_by_source.setdefault(src_service, {})
                sm[topic] = sm.get(topic, 0) + 1
                ekey = f"{src_i}->{dst_i}"
                em = mqtt_topic_counts_by_edge.setdefault(ekey, {})
                em[topic] = em.get(topic, 0) + 1
            if role:
                vb = message_volume_by_role.setdefault(role, {"messages": 0.0, "bytes": 0.0})
                vb["messages"] += 1.0
                vb["bytes"] += float(raw_bytes)
            sb = message_volume_by_service.setdefault(src_service, {"messages": 0.0, "bytes": 0.0})
            sb["messages"] += 1.0
            sb["bytes"] += float(raw_bytes)
            ts_wall = _event_ts_ms(rec)
            if ts_wall is None:
                continue
            if topic:
                tb = mqtt_topic_time_bounds_ms.setdefault(topic, [float(ts_wall), float(ts_wall)])
                tb[0] = min(float(tb[0]), float(ts_wall))
                tb[1] = max(float(tb[1]), float(ts_wall))
                mqtt_events.append((topic, float(ts_wall), src_service, int(raw_bytes)))
            global_t_min_ms = float(ts_wall) if global_t_min_ms is None else min(float(global_t_min_ms), float(ts_wall))
            global_t_max_ms = float(ts_wall) if global_t_max_ms is None else max(float(global_t_max_ms), float(ts_wall))
            ts_sim = _sim_ts_s(rec)

            dt_id = str(
                _get(rec, ["dt_id", "node_id", "gateway_id", "source_dt_id", "requester_dt_id"], "")
                or ""
            )
            ev_id = str(_get(rec, ["ev_id", "vehicle_id"], "") or "")
            tls_id = str(_get(rec, ["tls_id", "intersection_id"], "") or "")
            if not tls_id:
                tls_id = _infer_tls_id(rec)
            req_id = str(_get(rec, ["req_id", "request_id"], "") or "")
            query_id = str(_get(rec, ["query_id"], "") or "")
            assoc_id = str(_get(rec, ["assoc_id"], "") or "")
            advice_id = str(_get(rec, ["advice_id"], "") or "")
            trace_id = str(_get(rec, ["trace_id"], "") or "")
            did = dt_id or trace_id or str(_get(rec, ["gateway_id", "node_id"], "") or "")
            dt_key = str(dt_id or tls_id or str(_get(rec, ["gateway_id", "node_id", "source_service"], "") or "")).strip()

            if dt_key:
                eb = event_counts_by_dt.setdefault(dt_key, {})
                eb[et] = eb.get(et, 0) + 1
                tb = dt_time_bounds_ms.setdefault(dt_key, [float(ts_wall), float(ts_wall)])
                tb[0] = min(float(tb[0]), float(ts_wall))
                tb[1] = max(float(tb[1]), float(ts_wall))
                db = message_volume_by_dt.setdefault(dt_key, {"messages": 0.0, "bytes": 0.0})
                db["messages"] += 1.0
                db["bytes"] += float(raw_bytes)
                if role:
                    rb = role_counts_by_dt.setdefault(dt_key, {})
                    rb[role] = rb.get(role, 0) + 1

            if role:
                rt = role_time_bounds_ms.setdefault(role, [float(ts_wall), float(ts_wall)])
                rt[0] = min(float(rt[0]), float(ts_wall))
                rt[1] = max(float(rt[1]), float(ts_wall))

            if et == "coord.session.summary":
                src_file = str(_get(rec, ["_file"], "") or "")
                scenario_id = str(
                    _get(rec, ["scenario_id", "scenario", "scenario_name"], "") or ""
                ).strip()
                if not scenario_id:
                    scenario_id = _infer_scenario_from_text(src_file) or _infer_scenario_from_text(
                        str(_get(rec, ["topic_namespace"], "") or "")
                    )
                route_n = _infer_route_from_scenario(scenario_id)
                coord_session_summaries.append(
                    {
                        "ts_wall_ms": float(ts_wall),
                        "ts_sim_s": ts_sim,
                        "ev_id": ev_id or str(_get(rec, ["ev_id"], "") or ""),
                        "tls_id": tls_id or str(_get(rec, ["tls_id"], "") or ""),
                        "dt_id": dt_id or str(_get(rec, ["dt_id"], "") or ""),
                        "topic_namespace": str(_get(rec, ["topic_namespace"], "") or ""),
                        "scenario_id": scenario_id,
                        "route_number": route_n,
                        "mode": str(_get(rec, ["mode"], "") or _infer_mode_from_path(src_file)),
                        "apply_offer_n": int(_get(rec, ["apply_offer_n"], 0) or 0),
                        "apply_plan_n": int(_get(rec, ["apply_plan_n"], 0) or 0),
                        "apply_plan_offer_n": int(_get(rec, ["apply_plan_offer_n"], 0) or 0),
                        "apply_plan_local_fallback_n": int(_get(rec, ["apply_plan_local_fallback_n"], 0) or 0),
                        "apply_plan_selected_none_n": int(_get(rec, ["apply_plan_selected_none_n"], 0) or 0),
                        "apply_plan_warmup_n": int(_get(rec, ["apply_plan_warmup_n"], 0) or 0),
                        "plan_skip_n": int(_get(rec, ["plan_skip_n"], 0) or 0),
                        "hard_req_skip_n": int(_get(rec, ["hard_req_skip_n"], 0) or 0),
                        "selection_final_n": int(_get(rec, ["selection_final_n"], 0) or 0),
                        "latest_tick_compute_ms": _get(rec, ["latest_tick_compute_ms"], None),
                        "latest_refine_compute_ms": _get(rec, ["latest_refine_compute_ms"], None),
                        "latest_apply_compute_ms": _get(rec, ["latest_apply_compute_ms"], None),
                        "session_reason_counts": json.dumps(
                            _get(rec, ["session_reason_counts"], {}) or {},
                            ensure_ascii=True,
                            separators=(",", ":"),
                        ),
                        "source_file": src_file,
                        "source_line": int(_get(rec, ["_line"], 0) or 0),
                    }
                )

            if et in timeline_interest and len(timeline_events) < timeline_limit:
                timeline_events.append(
                    {
                        "event_type": et,
                        "ts_wall_ms": float(ts_wall),
                        "ts_sim_s": ts_sim,
                        "ev_id": ev_id,
                        "tls_id": tls_id,
                        "req_id": req_id,
                        "trace_id": trace_id,
                        "dt_id": dt_id,
                        "source_service": src_service,
                        "role": role,
                        "decision_source": str(_get(rec, ["decision_source"], "") or ""),
                        "reason": str(_get(rec, ["reason"], "") or ""),
                    }
                )

            # 0) FNM integration events (protocol mediation + routing)
            if et == "fnm.adapter.state_pull.ok":
                fnm_state_pull_ok += 1
                try:
                    n_req = int(_get(rec, ["req_published"], 0) or 0)
                except Exception:
                    n_req = 0
                try:
                    n_hint = int(_get(rec, ["route_hint_published"], 0) or 0)
                except Exception:
                    n_hint = 0
                fnm_req_published_total += max(0, n_req)
                fnm_route_hint_published_total += max(0, n_hint)
                if n_req > 0:
                    fnm_req_published_events += 1
                near_tls = str(_get(rec, ["nearest_tls"], "") or "").strip()
                if near_tls:
                    fnm_pull_nearest_tls[near_tls] = fnm_pull_nearest_tls.get(near_tls, 0) + 1
            elif et == "fnm.adapter.state_pull.error":
                fnm_state_pull_error += 1
            elif et == "fnm.route.local_to_fed":
                fnm_local_to_fed += 1
                rule = str(_get(rec, ["rule"], "") or "").strip() or "unknown_rule"
                fnm_local_to_fed_by_rule[rule] = fnm_local_to_fed_by_rule.get(rule, 0) + 1
            elif et == "fnm.route.fed_to_local":
                fnm_fed_to_local += 1
                rule = str(_get(rec, ["rule"], "") or "").strip() or "unknown_rule"
                fnm_fed_to_local_by_rule[rule] = fnm_fed_to_local_by_rule.get(rule, 0) + 1
            elif et == "fnm.overhead.process":
                dt = str(dt_id or _get(rec, ["node_id", "gateway_id"], "") or "").strip() or "unknown_dt"
                wall_runtime_s = None
                cpu_runtime_s = None
                cpu_util_pct = None
                max_rss_kb = None
                try:
                    vv = _get(rec, ["wall_runtime_s"], None)
                    if vv is not None:
                        wall_runtime_s = float(vv)
                except Exception:
                    wall_runtime_s = None
                try:
                    vv = _get(rec, ["cpu_runtime_s"], None)
                    if vv is not None:
                        cpu_runtime_s = float(vv)
                except Exception:
                    cpu_runtime_s = None
                try:
                    vv = _get(rec, ["cpu_util_pct"], None)
                    if vv is not None:
                        cpu_util_pct = float(vv)
                except Exception:
                    cpu_util_pct = None
                try:
                    vv = _get(rec, ["max_rss_kb"], None)
                    if vv is not None:
                        max_rss_kb = float(vv)
                except Exception:
                    max_rss_kb = None
                row = {
                    "dt_id": dt,
                    "wall_runtime_s": wall_runtime_s,
                    "cpu_runtime_s": cpu_runtime_s,
                    "cpu_util_pct": cpu_util_pct,
                    "max_rss_kb": max_rss_kb,
                    "local_to_fed": _get(rec, ["local_to_fed"], None),
                    "fed_to_local": _get(rec, ["fed_to_local"], None),
                }
                fnm_overhead_rows.append(row)
                slot = fnm_overhead_by_dt.setdefault(
                    dt,
                    {"wall_runtime_s": [], "cpu_runtime_s": [], "cpu_util_pct": [], "max_rss_kb": []},
                )
                if wall_runtime_s is not None and wall_runtime_s >= 0:
                    slot["wall_runtime_s"].append(float(wall_runtime_s))
                if cpu_runtime_s is not None and cpu_runtime_s >= 0:
                    slot["cpu_runtime_s"].append(float(cpu_runtime_s))
                if cpu_util_pct is not None and cpu_util_pct >= 0:
                    slot["cpu_util_pct"].append(float(cpu_util_pct))
                if max_rss_kb is not None and max_rss_kb >= 0:
                    slot["max_rss_kb"].append(float(max_rss_kb))

            # Communication overhead counters per TLS (single-run DT/intersection view)
            if tls_id:
                ctr = communication_overhead_by_tls.setdefault(
                    tls_id,
                    {
                        "reservation_req_sent": 0,
                        "reservation_resp_recv": 0,
                        "assoc_created": 0,
                        "assoc_released": 0,
                        "route_advice_published": 0,
                        "intersection_advice_published": 0,
                    },
                )
                if et == "federation.reservation.req.sent":
                    ctr["reservation_req_sent"] += 1
                elif et == "federation.reservation.req.recv":
                    # receive-side request volume at the intersection
                    ctr["reservation_req_sent"] += 1
                elif et == "federation.reservation.resp.recv":
                    ctr["reservation_resp_recv"] += 1
                elif et == "association.created":
                    ctr["assoc_created"] += 1
                elif et == "association.released":
                    ctr["assoc_released"] += 1
                elif et == "corridor.route_advice.published":
                    ctr["route_advice_published"] += 1
                elif et == "corridor.intersection.advice.published":
                    ctr["intersection_advice_published"] += 1

            if dt_key:
                dctr = communication_overhead_by_dt.setdefault(
                    dt_key,
                    {
                        "reservation_req_sent": 0,
                        "reservation_resp_recv": 0,
                        "assoc_created": 0,
                        "assoc_released": 0,
                        "route_advice_published": 0,
                        "intersection_advice_published": 0,
                    },
                )
                if et == "federation.reservation.req.sent":
                    dctr["reservation_req_sent"] += 1
                elif et == "federation.reservation.resp.recv":
                    dctr["reservation_resp_recv"] += 1
                elif et == "association.created":
                    dctr["assoc_created"] += 1
                elif et == "association.released":
                    dctr["assoc_released"] += 1
                elif et == "corridor.route_advice.published":
                    dctr["route_advice_published"] += 1
                elif et == "corridor.intersection.advice.published":
                    dctr["intersection_advice_published"] += 1

            # 0.5) Coordination action mix / decision churn diagnostics
            if et in ("intersection.plan.applied", "intersection.offer.applied", "intersection.offer.selected"):
                ds = str(_get(rec, ["decision_source"], "") or "").strip().lower()
                if et == "intersection.plan.applied":
                    coordination_apply_mix["plan_applied"] += 1
                    # Most F2 actuation is emitted as plan.applied with source=offer.
                    if ds in ("offer", "offer_selected"):
                        coordination_apply_mix["offer_applied"] += 1
                elif et == "intersection.offer.applied":
                    coordination_apply_mix["offer_applied"] += 1
                elif et == "intersection.offer.selected":
                    coordination_apply_mix["offer_selected"] += 1

                # Preserve F2 fallback visibility even when event type is plan.applied.
                if ds == "f2_local_fallback":
                    coordination_apply_mix["f2_local_fallback_applied"] += 1

                tls_for_mix = str(tls_id or _infer_tls_id(rec) or "").strip()
                if tls_for_mix:
                    rowm = coordination_apply_mix_by_tls.setdefault(
                        tls_for_mix,
                        {"plan_applied": 0, "offer_applied": 0, "offer_selected": 0, "f2_local_fallback_applied": 0},
                    )
                    if et == "intersection.plan.applied":
                        rowm["plan_applied"] += 1
                        if ds in ("offer", "offer_selected"):
                            rowm["offer_applied"] += 1
                    elif et == "intersection.offer.applied":
                        rowm["offer_applied"] += 1
                    elif et == "intersection.offer.selected":
                        rowm["offer_selected"] += 1
                    if ds == "f2_local_fallback":
                        rowm["f2_local_fallback_applied"] += 1

            if et in ("federation.refine.hard_req.skip", "coord.refine.hard_req_skip"):
                rs = str(_get(rec, ["reason"], "") or "").strip() or "unspecified"
                coordination_skip_reasons[rs] = coordination_skip_reasons.get(rs, 0) + 1

            # 1) Register latency
            if et == "lifecycle.register.request":
                k = dt_id or trace_id
                if k:
                    register_req[k] = (ts_wall, ts_sim, rec)
            elif et == "lifecycle.register.accepted":
                k = dt_id or trace_id
                if k:
                    _pair_latency(register_req, k, ts_wall, ts_sim, rec, samples, "T_register")
            elif et == "lifecycle.state.changed":
                s_to = _state_to(rec).upper()
                s_fr = _state_from(rec).upper()
                k = dt_id or trace_id
                if s_to == "REGISTERED" and k:
                    _pair_latency(register_req, k, ts_wall, ts_sim, rec, samples, "T_register")
                if s_to == "ONBOARDING" and k:
                    onboard_start[k] = (ts_wall, ts_sim, rec)
                if s_to == "ACTIVE" and k:
                    _pair_latency(onboard_start, k, ts_wall, ts_sim, rec, samples, "T_onboard")
                    # Time-to-active from register request.
                    _pair_latency(register_req, k, ts_wall, ts_sim, rec, samples, "T_time_to_active")
                if s_to == "SUSPENDED" and k:
                    suspend_evt[k] = (ts_wall, ts_sim, rec)
                if s_to == "ACTIVE" and s_fr == "SUSPENDED" and k:
                    _pair_latency(suspend_evt, k, ts_wall, ts_sim, rec, samples, "T_recovery")
            elif et == "lifecycle.onboarding.started":
                k = dt_id or trace_id
                if k:
                    onboard_start[k] = (ts_wall, ts_sim, rec)
            elif et == "lifecycle.onboarding.completed":
                k = dt_id or trace_id
                if k:
                    _pair_latency(onboard_start, k, ts_wall, ts_sim, rec, samples, "T_onboard")
            elif et in ("lifecycle.suspended", "health.timeout.detected"):
                k = dt_id or trace_id
                if k:
                    suspend_evt[k] = (ts_wall, ts_sim, rec)
            elif et in ("lifecycle.resumed", "health.availability.alive"):
                k = dt_id or trace_id
                if k:
                    _pair_latency(suspend_evt, k, ts_wall, ts_sim, rec, samples, "T_recovery")

            # Phase marks for operational phase chart (per DT/gateway)
            if did:
                mark = phase_marks.setdefault(did, {})
                if et == "lifecycle.register.request" and "register_request_ms" not in mark:
                    mark["register_request_ms"] = ts_wall
                if et == "lifecycle.register.accepted" and "register_accepted_ms" not in mark:
                    mark["register_accepted_ms"] = ts_wall
                if et == "lifecycle.onboarding.started" and "onboarding_started_ms" not in mark:
                    mark["onboarding_started_ms"] = ts_wall
                if et in ("lifecycle.onboarding.completed", "lifecycle.state.changed"):
                    s_to = _state_to(rec).upper()
                    if et == "lifecycle.onboarding.completed" or s_to == "ACTIVE":
                        if "active_ms" not in mark:
                            mark["active_ms"] = ts_wall
                if et in ("lifecycle.suspended", "health.timeout.detected"):
                    if "suspended_ms" not in mark:
                        mark["suspended_ms"] = ts_wall

            # 2) Discovery e2e + hit ratio
            if et == "discovery.query.request":
                discovery_requests += 1
                k = query_id or req_id or trace_id or _corr_key(rec, ["requester_dt_id", "purpose"])
                if k:
                    discovery_req[k] = (ts_wall, ts_sim, rec)
            elif et == "discovery.query.response":
                discovery_responses += 1
                k = query_id or req_id or trace_id or _corr_key(rec, ["requester_dt_id", "purpose"])
                if k:
                    _pair_latency(discovery_req, k, ts_wall, ts_sim, rec, samples, "T_discovery_e2e")
                discovery_total += 1
                n_results = _get(rec, ["n_results", "results_count"], 0)
                try:
                    n_res_int = int(n_results)
                    discovery_candidates_total += max(0, n_res_int)
                    if n_res_int > 0:
                        discovery_hits += 1
                except Exception:
                    pass

            # 3) Association setup
            if et == "association.create.request":
                k = assoc_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"])
                if k:
                    assoc_req[k] = (ts_wall, ts_sim, rec)
            elif et == "association.created":
                k = assoc_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"])
                if k:
                    _pair_latency(assoc_req, k, ts_wall, ts_sim, rec, samples, "T_assoc_setup")
                    assoc_created_ts[k] = (ts_wall, ts_sim, rec)
                    st = str(_get(rec, ["assoc_state", "state"], "INIT") or "INIT").upper()
                    assoc_state_counts[st] = assoc_state_counts.get(st, 0) + 1
                    assoc_last_state[k] = st
            elif et == "association.released":
                k = assoc_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"])
                if k and k in assoc_created_ts:
                    _pair_latency(assoc_created_ts, k, ts_wall, ts_sim, rec, samples, "T_assoc_lifecycle")
                st = str(_get(rec, ["assoc_state", "state"], "FINISHED") or "FINISHED").upper()
                assoc_state_counts[st] = assoc_state_counts.get(st, 0) + 1
                if k:
                    prev = assoc_last_state.get(k, "")
                    if prev and prev != st:
                        tkey = f"{prev}->{st}"
                        assoc_transition_counts[tkey] = assoc_transition_counts.get(tkey, 0) + 1
                    assoc_last_state[k] = st
            elif et == "association.state":
                k = assoc_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"])
                st = str(_get(rec, ["assoc_state", "state"], "") or "").upper()
                if st:
                    assoc_state_counts[st] = assoc_state_counts.get(st, 0) + 1
                if k and st:
                    prev = assoc_last_state.get(k, "")
                    if prev and prev != st:
                        tkey = f"{prev}->{st}"
                        assoc_transition_counts[tkey] = assoc_transition_counts.get(tkey, 0) + 1
                    assoc_last_state[k] = st

            # 4) Coordination req->resp
            if et == "federation.reservation.req.sent":
                k = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                if k:
                    coord_req[k] = (ts_wall, ts_sim, rec)
                req_out_observed_n += 1
                try:
                    v = _get(rec, ["req_out_recent_1s_sim"], None)
                    if v is not None:
                        req_out_recent_1s_sim_samples.append(float(v))
                except Exception:
                    pass
                try:
                    v = _get(rec, ["req_out_dt_prev_ms"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            req_out_dt_prev_ms_samples.append(x)
                except Exception:
                    pass
                try:
                    v = _get(rec, ["pending_req_n"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            req_out_pending_n_samples.append(x)
                except Exception:
                    pass
                try:
                    v = _get(rec, ["outbox_depth"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            req_outbox_depth_samples.append(x)
                except Exception:
                    pass
                if bool(_get(rec, ["req_out_repeated_within_1s"], False)):
                    req_out_repeated_within_1s_n += 1
            elif et == "federation.reservation.req.recv":
                k = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                if k:
                    coord_local_decision_start[k] = (ts_wall, ts_sim, rec)
                    coord_apply_start[k] = (ts_wall, ts_sim, rec)
            elif et == "coord.outbox.depth.peak":
                try:
                    v = _get(rec, ["depth", "outbox_depth"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            outbox_depth_peak_samples.append(x)
                except Exception:
                    pass
            elif et == "coord.outbox.drain":
                try:
                    v = _get(rec, ["n", "drained_n"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            outbox_drain_n_samples.append(x)
                except Exception:
                    pass
                try:
                    v = _get(rec, ["depth_peak"], None)
                    if v is not None:
                        x = float(v)
                        if x >= 0:
                            outbox_drain_depth_peak_samples.append(x)
                except Exception:
                    pass
            elif et in ("coord.reservation.req_decision", "federation.reservation.req.decision"):
                k = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                if k:
                    _pair_latency(coord_local_decision_start, k, ts_wall, ts_sim, rec, samples, "T_coord_req_to_decision")
            elif et == "federation.reservation.resp.recv":
                k = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                if k:
                    _pair_latency(coord_req, k, ts_wall, ts_sim, rec, samples, "T_coord_req_resp")
            elif et == "federation.reservation.req_resp.e2e":
                # Directly logged E2E latency from intersection debug stream.
                d_wall = _get(rec, ["latency_wall_ms", "wall_ms"], None)
                d_sim = _get(rec, ["latency_sim_ms", "sim_ms"], None)
                d_fallback = _duration_ms(rec)
                try:
                    d_wall_f = float(d_wall) if d_wall is not None else None
                except Exception:
                    d_wall_f = None
                try:
                    d_sim_f = float(d_sim) if d_sim is not None else None
                except Exception:
                    d_sim_f = None
                # Backward compatibility: old logs only had latency_ms (sim-time in most runs).
                if d_wall_f is None and d_fallback is not None:
                    d_wall_f = float(d_fallback)
                if d_sim_f is None and d_fallback is not None:
                    d_sim_f = float(d_fallback)
                if (d_wall_f is not None and d_wall_f >= 0) or (d_sim_f is not None and d_sim_f >= 0):
                    key_rr = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                    samples.append(
                        {
                            "metric": "T_coord_req_resp",
                            "key": key_rr,
                            "latency_wall_ms": (float(d_wall_f) if d_wall_f is not None and d_wall_f >= 0 else None),
                            "latency_sim_ms": (float(d_sim_f) if d_sim_f is not None and d_sim_f >= 0 else None),
                            "start_ts_wall_ms": ts_wall,
                            "end_ts_wall_ms": ts_wall,
                            "start_ts_sim_s": ts_sim,
                            "end_ts_sim_s": ts_sim,
                            "role": _infer_role(rec) or "intersection",
                            "dt_id": _infer_dt_id(rec),
                            "tls_id": tls_id,
                            "source_service": _infer_source_service(rec),
                            "start_file": rec.get("_file", ""),
                            "start_line": rec.get("_line", 0),
                            "end_file": rec.get("_file", ""),
                            "end_line": rec.get("_line", 0),
                        }
                    )
                    split_fields = {
                        "T_coord_req_resp_source_local_compute": _get(rec, ["source_local_compute_ms"], None),
                        "T_coord_req_resp_source_fnm_integration": _get(rec, ["source_fnm_integration_ms"], None),
                        "T_coord_req_resp_remote_processing": _get(rec, ["responder_processing_ms"], None),
                        "T_coord_req_resp_network_wait": _get(rec, ["network_wait_ms"], None),
                    }
                    split_vals: Dict[str, float] = {}
                    for mk, mv in split_fields.items():
                        try:
                            fv = float(mv) if mv is not None else None
                        except Exception:
                            fv = None
                        if fv is None or fv < 0:
                            continue
                        split_vals[mk] = float(fv)
                        samples.append(
                            {
                                "metric": mk,
                                "key": key_rr,
                                "latency_wall_ms": float(fv),
                                "latency_sim_ms": None,
                                "start_ts_wall_ms": ts_wall,
                                "end_ts_wall_ms": ts_wall,
                                "start_ts_sim_s": ts_sim,
                                "end_ts_sim_s": ts_sim,
                                "role": _infer_role(rec) or "intersection",
                                "dt_id": _infer_dt_id(rec),
                                "tls_id": tls_id,
                                "source_service": _infer_source_service(rec),
                                "start_file": rec.get("_file", ""),
                                "start_line": rec.get("_line", 0),
                                "end_file": rec.get("_file", ""),
                                "end_line": rec.get("_line", 0),
                            }
                        )
                    if split_vals:
                        total_processing = (
                            float(split_vals.get("T_coord_req_resp_source_local_compute", 0.0))
                            + float(split_vals.get("T_coord_req_resp_source_fnm_integration", 0.0))
                            + float(split_vals.get("T_coord_req_resp_remote_processing", 0.0))
                        )
                        samples.append(
                            {
                                "metric": "T_coord_req_resp_total_processing",
                                "key": key_rr,
                                "latency_wall_ms": float(total_processing),
                                "latency_sim_ms": None,
                                "start_ts_wall_ms": ts_wall,
                                "end_ts_wall_ms": ts_wall,
                                "start_ts_sim_s": ts_sim,
                                "end_ts_sim_s": ts_sim,
                                "role": _infer_role(rec) or "intersection",
                                "dt_id": _infer_dt_id(rec),
                                "tls_id": tls_id,
                                "source_service": _infer_source_service(rec),
                                "start_file": rec.get("_file", ""),
                                "start_line": rec.get("_line", 0),
                                "end_file": rec.get("_file", ""),
                                "end_line": rec.get("_line", 0),
                            }
                        )
                    if d_wall_f is not None and d_sim_f is not None:
                        wait_gap = float(d_sim_f) - float(d_wall_f)
                        if wait_gap >= 0:
                            samples.append(
                                {
                                    "metric": "T_coord_req_resp_wait_gap",
                                    "key": key_rr,
                                    "latency_wall_ms": float(wait_gap),
                                    "latency_sim_ms": None,
                                    "start_ts_wall_ms": ts_wall,
                                    "end_ts_wall_ms": ts_wall,
                                    "start_ts_sim_s": ts_sim,
                                    "end_ts_sim_s": ts_sim,
                                    "role": _infer_role(rec) or "intersection",
                                    "dt_id": _infer_dt_id(rec),
                                    "tls_id": tls_id,
                                    "source_service": _infer_source_service(rec),
                                    "start_file": rec.get("_file", ""),
                                    "start_line": rec.get("_line", 0),
                                    "end_file": rec.get("_file", ""),
                                    "end_line": rec.get("_line", 0),
                                }
                            )
            elif et in ("intersection.plan.applied",):
                k = req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id", "source_service"])
                if k:
                    _pair_latency(coord_apply_start, k, ts_wall, ts_sim, rec, samples, "T_coord_req_to_apply")

            # 4.5) EV request -> local TLS actuation/signal (interop pipeline observability)
            if et in ("ev.request.received", "ev.request.in"):
                req_age_ms: Optional[float] = None
                for k_age in ("request_age_ms", "req_age_ms", "age_ms"):
                    try:
                        vv = _get(rec, [k_age], None)
                        if vv is not None:
                            req_age_ms = float(vv)
                            break
                    except Exception:
                        continue
                if req_age_ms is not None and req_age_ms >= 0:
                    samples.append(
                        {
                            "metric": "T_ev_request_age",
                            "key": req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"]),
                            "latency_wall_ms": float(req_age_ms),
                            "latency_sim_ms": None,
                            "start_ts_wall_ms": ts_wall,
                            "end_ts_wall_ms": ts_wall,
                            "start_ts_sim_s": ts_sim,
                            "end_ts_sim_s": ts_sim,
                            "role": _infer_role(rec) or "intersection",
                            "dt_id": _infer_dt_id(rec),
                            "tls_id": tls_id,
                            "source_service": _infer_source_service(rec),
                            "start_file": rec.get("_file", ""),
                            "start_line": rec.get("_line", 0),
                            "end_file": rec.get("_file", ""),
                            "end_line": rec.get("_line", 0),
                        }
                    )
                if ev_id and tls_id:
                    k_ev = f"{ev_id}|{tls_id}"
                    # Keep latest request per EV/TLS pair to avoid stale-first over-pairing.
                    ev_req_to_apply_start[k_ev] = (ts_wall, ts_sim, rec)
                    ev_req_to_signal_start[k_ev] = (ts_wall, ts_sim, rec)
                if req_id:
                    ev_req_to_apply_by_reqid[req_id] = (ts_wall, ts_sim, rec)
                    ev_req_to_signal_by_reqid[req_id] = (ts_wall, ts_sim, rec)

            elif et in ("intersection.plan.applied", "intersection.offer.applied"):
                max_apply_pair_ms = 15000.0
                matched = False
                if req_id and req_id in ev_req_to_apply_by_reqid:
                    _pair_latency(
                        ev_req_to_apply_by_reqid,
                        req_id,
                        ts_wall,
                        ts_sim,
                        rec,
                        samples,
                        "T_ev_req_to_intersection_apply",
                    )
                    ev_apply_pair_matched_reqid += 1
                    matched = True
                if not matched:
                    k_ev = ""
                    if ev_id and tls_id:
                        k_try = f"{ev_id}|{tls_id}"
                        if k_try in ev_req_to_apply_start:
                            k_ev = k_try
                    if not k_ev and tls_id:
                        # Most-recent fallback candidate for this TLS.
                        cand = sorted(
                            (k for k in ev_req_to_apply_start.keys() if k.endswith("|" + tls_id)),
                            key=lambda kk: ev_req_to_apply_start[kk][0],
                            reverse=True,
                        )
                        if cand:
                            k_ev = cand[0]
                    if k_ev:
                        t0 = float((ev_req_to_apply_start.get(k_ev) or (0.0, None, {}))[0])
                        dt_ms = float(ts_wall) - float(t0)
                        if dt_ms <= max_apply_pair_ms:
                            _pair_latency(
                                ev_req_to_apply_start,
                                k_ev,
                                ts_wall,
                                ts_sim,
                                rec,
                                samples,
                                "T_ev_req_to_intersection_apply",
                            )
                            ev_apply_pair_matched_fallback += 1
                        else:
                            ev_apply_pair_dropped_stale += 1

            elif et in ("tls.signal.change",):
                max_signal_pair_ms = 20000.0
                matched = False
                if req_id and req_id in ev_req_to_signal_by_reqid:
                    _pair_latency(
                        ev_req_to_signal_by_reqid,
                        req_id,
                        ts_wall,
                        ts_sim,
                        rec,
                        samples,
                        "T_ev_req_to_signal_change",
                    )
                    ev_signal_pair_matched_reqid += 1
                    matched = True
                if not matched:
                    k_ev = ""
                    if ev_id and tls_id:
                        k_try = f"{ev_id}|{tls_id}"
                        if k_try in ev_req_to_signal_start:
                            k_ev = k_try
                    if not k_ev and tls_id:
                        # Most-recent fallback candidate for this TLS.
                        cand = sorted(
                            (k for k in ev_req_to_signal_start.keys() if k.endswith("|" + tls_id)),
                            key=lambda kk: ev_req_to_signal_start[kk][0],
                            reverse=True,
                        )
                        if cand:
                            k_ev = cand[0]
                    if k_ev:
                        t0 = float((ev_req_to_signal_start.get(k_ev) or (0.0, None, {}))[0])
                        dt_ms = float(ts_wall) - float(t0)
                        if dt_ms <= max_signal_pair_ms:
                            _pair_latency(
                                ev_req_to_signal_start,
                                k_ev,
                                ts_wall,
                                ts_sim,
                                rec,
                                samples,
                                "T_ev_req_to_signal_change",
                            )
                            ev_signal_pair_matched_fallback += 1
                        else:
                            ev_signal_pair_dropped_stale += 1

            # 5) Advice uptake
            if et == "corridor.route_advice.published":
                k = advice_id or trace_id or ev_id
                if k:
                    advice_pub[k] = (ts_wall, ts_sim, rec)
                imp = _get(
                    rec,
                    [
                        "improvement_sec",
                        "improve_sec",
                        "predicted_improvement_sec",
                        "predicted_delta_sec",
                        "eta_gain_sec",
                    ],
                    None,
                )
                try:
                    if imp is not None:
                        advice_improvement_sec.append(float(imp))
                except Exception:
                    pass
            elif et in ("corridor.route_advice.received",):
                k = advice_id or trace_id or ev_id
                if k:
                    advice_seen[k] = (ts_wall, ts_sim, rec)
            elif et == "corridor.route_advice.applied":
                k = advice_id or trace_id or ev_id
                if k:
                    _pair_latency(advice_pub, k, ts_wall, ts_sim, rec, samples, "T_advice_uptake")
                    _pair_latency(advice_seen, k, ts_wall, ts_sim, rec, samples, "T_advice_seen_to_apply")
            elif et == "ev.stuck.enter":
                k = ev_id or trace_id
                if k:
                    ev_stuck_enter[k] = (ts_wall, ts_sim, rec)
            elif et == "ev.stuck.exit":
                k = ev_id or trace_id
                if k:
                    _pair_latency(ev_stuck_enter, k, ts_wall, ts_sim, rec, samples, "T_ev_stuck_episode")

            # 6) Direct intersection computation durations
            if et in (
                "intersection.compute.tick.duration",
                "intersection.compute.refine.duration",
                "intersection.compute.apply.duration",
                "corridor.compute.reassess.duration",
                "corridor.compute.advice.duration",
                "corridor.compute.route_advice_cycle.duration",
                "corridor.compute.state_pub.duration",
                "corridor.route_opt.eval",
            ):
                dms = _duration_ms(rec)
                if dms is not None and dms >= 0:
                    metric_name = {
                        "intersection.compute.tick.duration": "C_intersection_tick_compute",
                        "intersection.compute.refine.duration": "C_intersection_refine_compute",
                        "intersection.compute.apply.duration": "C_intersection_apply_compute",
                        "corridor.compute.reassess.duration": "C_corridor_reassess_compute",
                        "corridor.compute.advice.duration": "C_corridor_advice_compute",
                        "corridor.compute.route_advice_cycle.duration": "C_corridor_route_advice_cycle_compute",
                        "corridor.compute.state_pub.duration": "C_corridor_state_pub_compute",
                        "corridor.route_opt.eval": "C_corridor_route_opt_compute",
                    }.get(et, "C_intersection_compute")
                    samples.append(
                        {
                            "metric": metric_name,
                            "key": req_id or trace_id or _corr_key(rec, ["ev_id", "tls_id"]),
                            "latency_wall_ms": dms,
                            "latency_sim_ms": None,
                            "start_ts_wall_ms": ts_wall,
                            "end_ts_wall_ms": ts_wall,
                            "start_ts_sim_s": ts_sim,
                            "end_ts_sim_s": ts_sim,
                            "role": _infer_role(rec) or "intersection",
                            "dt_id": _infer_dt_id(rec),
                            "tls_id": tls_id,
                            "source_service": _infer_source_service(rec),
                            "start_file": rec.get("_file", ""),
                            "start_line": rec.get("_line", 0),
                            "end_file": rec.get("_file", ""),
                            "end_line": rec.get("_line", 0),
                        }
                    )
                    if tls_id:
                        tmap = direct_compute_by_tls.setdefault(tls_id, {})
                        tmap.setdefault(metric_name, []).append(float(dms))
                    if dt_key:
                        dmap = direct_compute_by_dt.setdefault(dt_key, {})
                        dmap.setdefault(metric_name, []).append(float(dms))

    # Aggregate
    by_metric: Dict[str, List[float]] = {}
    by_metric_sim: Dict[str, List[float]] = {}
    for s in samples:
        m = str(s["metric"])
        w = s.get("latency_wall_ms")
        sim = s.get("latency_sim_ms")
        if w is not None:
            by_metric.setdefault(m, []).append(float(w))
        if sim is not None:
            by_metric_sim.setdefault(m, []).append(float(sim))

    metric_summary: Dict[str, Dict[str, Any]] = {}
    metrics_by_role: Dict[str, Dict[str, Dict[str, Any]]] = {}
    metrics_by_dt: Dict[str, Dict[str, Dict[str, Any]]] = {}
    metrics_by_tls: Dict[str, Dict[str, Dict[str, Any]]] = {}
    for m, xs in by_metric.items():
        metric_summary[m] = {"wall_ms": _stats(xs), "sim_ms": _stats(by_metric_sim.get(m, []))}

    # Breakdown by role and by tls
    tmp_role: Dict[str, Dict[str, List[float]]] = {}
    tmp_role_sim: Dict[str, Dict[str, List[float]]] = {}
    tmp_dt: Dict[str, Dict[str, List[float]]] = {}
    tmp_dt_sim: Dict[str, Dict[str, List[float]]] = {}
    tmp_tls: Dict[str, Dict[str, List[float]]] = {}
    tmp_tls_sim: Dict[str, Dict[str, List[float]]] = {}
    for s in samples:
        metric = str(s.get("metric", ""))
        w = s.get("latency_wall_ms")
        sim = s.get("latency_sim_ms")
        role = str(s.get("role", "") or "")
        dt = str(s.get("dt_id", "") or s.get("tls_id", "") or "")
        tls = str(s.get("tls_id", "") or "")
        if role and w is not None:
            tmp_role.setdefault(role, {}).setdefault(metric, []).append(float(w))
        if role and sim is not None:
            tmp_role_sim.setdefault(role, {}).setdefault(metric, []).append(float(sim))
        if dt and w is not None:
            tmp_dt.setdefault(dt, {}).setdefault(metric, []).append(float(w))
        if dt and sim is not None:
            tmp_dt_sim.setdefault(dt, {}).setdefault(metric, []).append(float(sim))
        if tls and w is not None:
            tmp_tls.setdefault(tls, {}).setdefault(metric, []).append(float(w))
        if tls and sim is not None:
            tmp_tls_sim.setdefault(tls, {}).setdefault(metric, []).append(float(sim))

    # EV local service duration inside each TLS/DT
    ev_req_start_by_key: Dict[str, Tuple[float, Optional[float], Dict[str, Any]]] = {}
    for p in paths:
        for rec in _read_jsonl(p):
            et = _event_type(rec, alias)
            if not et:
                continue
            ts_wall = _event_ts_ms(rec)
            if ts_wall is None:
                continue
            ts_sim = _sim_ts_s(rec)
            ev_id = str(_get(rec, ["ev_id", "vehicle_id"], "") or "")
            if not ev_id:
                continue
            tls_id = _infer_tls_id(rec)
            dt_id = _infer_dt_id(rec)
            key = f"{ev_id}|{tls_id or dt_id}"
            if et in ("ev.request.received", "ev.request.in"):
                if key not in ev_req_start_by_key:
                    ev_req_start_by_key[key] = (float(ts_wall), ts_sim, rec)
            elif et in ("ev.pass.detected",):
                if key in ev_req_start_by_key:
                    _pair_latency(
                        ev_req_start_by_key,
                        key,
                        float(ts_wall),
                        ts_sim,
                        rec,
                        samples,
                        "T_ev_service_tls",
                    )

    for role, mm in tmp_role.items():
        metrics_by_role[role] = {}
        for metric, vals in mm.items():
            metrics_by_role[role][metric] = {
                "wall_ms": _stats(vals),
                "sim_ms": _stats(tmp_role_sim.get(role, {}).get(metric, [])),
            }

    for dt, mm in tmp_dt.items():
        metrics_by_dt[dt] = {}
        for metric, vals in mm.items():
            metrics_by_dt[dt][metric] = {
                "wall_ms": _stats(vals),
                "sim_ms": _stats(tmp_dt_sim.get(dt, {}).get(metric, [])),
            }

    for tls, mm in tmp_tls.items():
        metrics_by_tls[tls] = {}
        for metric, vals in mm.items():
            metrics_by_tls[tls][metric] = {
                "wall_ms": _stats(vals),
                "sim_ms": _stats(tmp_tls_sim.get(tls, {}).get(metric, [])),
            }

    computation_overhead_by_tls: Dict[str, Dict[str, Any]] = {}
    for tls, mm in metrics_by_tls.items():
        row: Dict[str, Any] = {}
        for metric in ("T_coord_req_resp", "T_assoc_setup", "T_advice_uptake"):
            st = dict((mm.get(metric, {}) or {}).get("wall_ms", {}) or {})
            if int(st.get("n", 0) or 0) > 0:
                row[metric] = st
        if row:
            computation_overhead_by_tls[tls] = row

    # Overlay direct compute durations when available.
    for tls, dmap in direct_compute_by_tls.items():
        row = computation_overhead_by_tls.setdefault(tls, {})
        for metric_name, vals in dmap.items():
            row[metric_name] = _stats(vals)

    computation_overhead_by_dt: Dict[str, Dict[str, Any]] = {}
    for dt, mm in metrics_by_dt.items():
        row: Dict[str, Any] = {}
        for metric in ("T_coord_req_resp", "T_assoc_setup", "T_advice_uptake"):
            st = dict((mm.get(metric, {}) or {}).get("wall_ms", {}) or {})
            if int(st.get("n", 0) or 0) > 0:
                row[metric] = st
        if row:
            computation_overhead_by_dt[dt] = row
    for dt, dmap in direct_compute_by_dt.items():
        row = computation_overhead_by_dt.setdefault(dt, {})
        for metric_name, vals in dmap.items():
            row[metric_name] = _stats(vals)

    discovery_hit_ratio = None
    if discovery_total > 0:
        discovery_hit_ratio = float(discovery_hits) / float(discovery_total)

    # Operational phase durations from lifecycle marks
    phase_samples: Dict[str, List[float]] = {
        "register_to_onboarding_ms": [],
        "onboarding_to_active_ms": [],
        "time_to_active_ms": [],
        "active_to_suspended_ms": [],
    }
    for mark in phase_marks.values():
        t_reg = mark.get("register_accepted_ms", mark.get("register_request_ms"))
        t_onb = mark.get("onboarding_started_ms")
        t_act = mark.get("active_ms")
        t_sus = mark.get("suspended_ms")
        if t_reg is not None and t_onb is not None and t_onb >= t_reg:
            phase_samples["register_to_onboarding_ms"].append(float(t_onb - t_reg))
        if t_onb is not None and t_act is not None and t_act >= t_onb:
            phase_samples["onboarding_to_active_ms"].append(float(t_act - t_onb))
        if t_reg is not None and t_act is not None and t_act >= t_reg:
            phase_samples["time_to_active_ms"].append(float(t_act - t_reg))
        if t_act is not None and t_sus is not None and t_sus >= t_act:
            phase_samples["active_to_suspended_ms"].append(float(t_sus - t_act))
    phase_summary = {k: _stats(v) for k, v in phase_samples.items()}

    # Membership population estimates (scalability-oriented)
    n_registered_unique = 0
    n_active_ever = 0
    n_suspended_ever = 0
    for mark in phase_marks.values():
        if ("register_accepted_ms" in mark) or ("register_request_ms" in mark):
            n_registered_unique += 1
        if "active_ms" in mark:
            n_active_ever += 1
        if "suspended_ms" in mark:
            n_suspended_ever += 1

    transition_events: List[Tuple[float, str, str]] = []
    assoc_transition_events: List[Tuple[float, int]] = []
    fcm_discovery_events: List[Dict[str, Any]] = []
    fcm_peer_events: List[Dict[str, Any]] = []
    adaptive_binding_events: List[Dict[str, Any]] = []
    adaptive_state_timeline: List[Dict[str, Any]] = []
    for p in paths:
        for rec in _read_jsonl(p):
            et = _event_type(rec, alias)
            if not et:
                continue
            ts_wall = _event_ts_ms(rec)
            if ts_wall is None:
                continue
            did = str(
                _get(
                    rec,
                    ["dt_id", "gateway_id", "node_id", "requester_dt_id", "trace_id"],
                    "",
                )
                or ""
            )
            if not did:
                continue
            if et in ("lifecycle.onboarding.completed", "lifecycle.resumed", "lifecycle.suspended", "lifecycle.offboarded"):
                transition_events.append((float(ts_wall), did, et))
            if et == "association.created":
                assoc_transition_events.append((float(ts_wall), +1))
            elif et == "association.released":
                assoc_transition_events.append((float(ts_wall), -1))
            if et in (
                "fcm.discovery.query",
                "fcm.discovery.response",
                "fcm.peer_set.update",
                "fcm.peer_set.expire",
                "fcm.peer_set.reject",
                "fcm.peer_set.error",
            ):
                entry = {
                    "ts_wall_ms": float(ts_wall),
                    "ts_sim_s": _sim_ts_s(rec),
                    "event_type": str(et),
                    "dt_id": did,
                    "peer_id": str(_get(rec, ["peer_id", "node_id", "target", "peer"], "") or "").strip(),
                    "reason": str(_get(rec, ["reason"], "") or "").strip(),
                }
                if len(fcm_discovery_events) < 12000:
                    fcm_discovery_events.append(entry)
                if et in ("fcm.peer_set.update", "fcm.peer_set.expire", "fcm.peer_set.reject"):
                    if len(fcm_peer_events) < 24000:
                        fcm_peer_events.append(entry)
            if et in (
                "adaptive.connectivity.binding_set.update",
                "adaptive.connectivity.binding_set.expire",
                "adaptive.connectivity.binding_set.snapshot",
            ):
                a_entry = {
                    "ts_wall_ms": float(ts_wall),
                    "ts_sim_s": _sim_ts_s(rec),
                    "event_type": str(et),
                    "src": str(_get(rec, ["src", "source", "from", "from_tls"], "") or "").strip(),
                    "dst": str(_get(rec, ["dst", "target", "to", "to_tls"], "") or "").strip(),
                    "purpose": str(_get(rec, ["purpose"], "") or "").strip(),
                    "action": str(_get(rec, ["action"], "") or "").strip(),
                    "reason": str(_get(rec, ["reason"], "") or "").strip(),
                    "active_bindings_n": int(_get(rec, ["active_bindings_n"], 0) or 0),
                    "peers_n": int(_get(rec, ["peers_n"], 0) or 0),
                }
                if len(adaptive_binding_events) < 48000:
                    adaptive_binding_events.append(a_entry)
            if et == "adaptive.connectivity.state":
                s_entry = {
                    "ts_wall_ms": float(ts_wall),
                    "ts_sim_s": _sim_ts_s(rec),
                    "active_bindings_n": int(_get(rec, ["active_bindings_n", "connectivity_edges_n"], 0) or 0),
                    "connectivity_edges_n": int(_get(rec, ["connectivity_edges_n"], 0) or 0),
                    "members_n": int(_get(rec, ["members_n"], 0) or 0),
                    "catalog_nodes_n": int(_get(rec, ["catalog_nodes_n"], 0) or 0),
                }
                if len(adaptive_state_timeline) < 12000:
                    adaptive_state_timeline.append(s_entry)

    active_flags: Dict[str, bool] = {}
    active_count = 0
    max_active = 0
    active_timeline: List[Dict[str, Any]] = []
    for ts, did, et in sorted(transition_events, key=lambda x: x[0]):
        if et in ("lifecycle.onboarding.completed", "lifecycle.resumed"):
            if not active_flags.get(did, False):
                active_flags[did] = True
                active_count += 1
        elif et in ("lifecycle.suspended", "lifecycle.offboarded"):
            if active_flags.get(did, False):
                active_flags[did] = False
                active_count = max(0, active_count - 1)
        if active_count > max_active:
            max_active = active_count
        active_timeline.append({"ts_wall_ms": ts, "active_members": active_count})

    assoc_active = 0
    assoc_max_active = 0
    assoc_active_timeline: List[Dict[str, Any]] = []
    for ts, delta in sorted(assoc_transition_events, key=lambda x: x[0]):
        assoc_active = max(0, assoc_active + int(delta))
        if assoc_active > assoc_max_active:
            assoc_max_active = assoc_active
        assoc_active_timeline.append({"ts_wall_ms": float(ts), "active_associations": int(assoc_active)})

    # 1-second wall-time bins for FCM/discovery runtime behavior.
    fcm_discovery_timeline: List[Dict[str, Any]] = []
    if fcm_discovery_events:
        def _as_float_default(v: Any, default: float) -> float:
            try:
                return float(v)
            except Exception:
                return default

        sorted_ev = sorted(
            fcm_discovery_events,
            key=lambda r: _as_float_default((r or {}).get("ts_wall_ms"), -1.0),
        )
        t0_ms = _as_float_default((sorted_ev[0] or {}).get("ts_wall_ms"), 0.0)
        bins: Dict[int, Dict[str, int]] = {}
        for r in sorted_ev:
            t_ms = _as_float_default((r or {}).get("ts_wall_ms"), -1.0)
            if t_ms < 0:
                continue
            sec = int(max(0.0, (t_ms - t0_ms) / 1000.0))
            b = bins.setdefault(
                sec,
                {
                    "query": 0,
                    "response": 0,
                    "peer_update": 0,
                    "peer_expire": 0,
                    "peer_reject": 0,
                    "peer_error": 0,
                },
            )
            et = str((r or {}).get("event_type", "") or "")
            if et == "fcm.discovery.query":
                b["query"] += 1
            elif et == "fcm.discovery.response":
                b["response"] += 1
            elif et == "fcm.peer_set.update":
                b["peer_update"] += 1
            elif et == "fcm.peer_set.expire":
                b["peer_expire"] += 1
            elif et == "fcm.peer_set.reject":
                b["peer_reject"] += 1
            elif et == "fcm.peer_set.error":
                b["peer_error"] += 1
        for sec in sorted(bins.keys()):
            row = {"elapsed_s": int(sec)}
            row.update(bins.get(sec, {}))
            fcm_discovery_timeline.append(row)

    # Federation coordination flow summary
    req_n = int(event_counts.get("federation.reservation.req.sent", 0))
    resp_n = int(event_counts.get("federation.reservation.resp.recv", 0))
    assoc_created_n = int(event_counts.get("association.created", 0))
    assoc_released_n = int(event_counts.get("association.released", 0))
    route_advice_n = int(event_counts.get("corridor.route_advice.published", 0))
    route_advice_seen_n = int(event_counts.get("corridor.route_advice.received", 0))
    route_advice_applied_n = int(event_counts.get("corridor.route_advice.applied", 0))
    route_advice_skipped_n = int(event_counts.get("corridor.route_advice.apply_skipped", 0))
    intersection_advice_n = int(event_counts.get("corridor.intersection.advice.published", 0))
    coord_response_ratio = (float(resp_n) / float(req_n)) if req_n > 0 else None
    route_apply_ratio = (float(route_advice_applied_n) / float(route_advice_n)) if route_advice_n > 0 else None

    assoc_lifecycle_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_assoc_lifecycle" and s.get("latency_wall_ms") is not None
    ]
    advice_uptake_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_advice_uptake" and s.get("latency_wall_ms") is not None
    ]
    advice_seen_apply_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_advice_seen_to_apply" and s.get("latency_wall_ms") is not None
    ]
    ev_service_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_ev_service_tls" and s.get("latency_wall_ms") is not None
    ]
    ev_stuck_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_ev_stuck_episode" and s.get("latency_wall_ms") is not None
    ]
    req_to_decision_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_coord_req_to_decision" and s.get("latency_wall_ms") is not None
    ]
    req_to_apply_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_coord_req_to_apply" and s.get("latency_wall_ms") is not None
    ]
    ev_req_to_apply_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_ev_req_to_intersection_apply" and s.get("latency_wall_ms") is not None
    ]
    ev_req_to_signal_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_ev_req_to_signal_change" and s.get("latency_wall_ms") is not None
    ]
    ev_request_age_samples = [
        float(s.get("latency_wall_ms"))
        for s in samples
        if str(s.get("metric", "")) == "T_ev_request_age" and s.get("latency_wall_ms") is not None
    ]

    # Role index inferred from dominant event role.
    role_by_dt: Dict[str, str] = {}
    for dt, counts in role_counts_by_dt.items():
        if not counts:
            continue
        role_by_dt[dt] = max(counts.items(), key=lambda kv: kv[1])[0]

    # Normalized overhead by DT: message/computation/coordination rate.
    normalized_overhead_by_dt: Dict[str, Dict[str, Any]] = {}
    for dt in sorted(set(event_counts_by_dt.keys()) | set(communication_overhead_by_dt.keys()) | set(metrics_by_dt.keys())):
        events_dt = dict(event_counts_by_dt.get(dt, {}) or {})
        comm_dt = dict(communication_overhead_by_dt.get(dt, {}) or {})
        bounds = dt_time_bounds_ms.get(dt, None)
        active_span_s = None
        if bounds and len(bounds) == 2 and float(bounds[1]) >= float(bounds[0]):
            active_span_s = max(0.001, (float(bounds[1]) - float(bounds[0])) / 1000.0)

        msg_total = int(sum(int(v or 0) for v in events_dt.values()))
        comm_total = int(sum(int(v or 0) for v in comm_dt.values()))
        msg_rate_s = (float(msg_total) / float(active_span_s)) if active_span_s else None
        comm_rate_s = (float(comm_total) / float(active_span_s)) if active_span_s else None

        mm = dict(metrics_by_dt.get(dt, {}) or {})
        coord = dict((mm.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {})
        coord_n = int(coord.get("n", 0) or 0)
        coord_mean_ms = float(coord.get("mean_ms")) if coord.get("mean_ms") is not None else None
        coord_p95_ms = float(coord.get("p95_ms")) if coord.get("p95_ms") is not None else None
        coord_total_ms = (coord_mean_ms * float(coord_n)) if (coord_mean_ms is not None and coord_n > 0) else None
        coord_ms_per_s = (float(coord_total_ms) / float(active_span_s)) if (coord_total_ms is not None and active_span_s) else None

        compute_total_ms = 0.0
        compute_n = 0
        compute_mean_ms: Optional[float] = None
        for mk in ("C_intersection_tick_compute", "C_intersection_refine_compute", "C_intersection_apply_compute"):
            st = dict((mm.get(mk, {}) or {}).get("wall_ms", {}) or {})
            n = int(st.get("n", 0) or 0)
            mn = st.get("mean_ms", None)
            if n > 0 and mn is not None:
                compute_total_ms += float(mn) * float(n)
                compute_n += n
        if compute_n > 0:
            compute_mean_ms = compute_total_ms / float(compute_n)
        compute_ms_per_s = (float(compute_total_ms) / float(active_span_s)) if (compute_n > 0 and active_span_s) else None

        normalized_overhead_by_dt[dt] = {
            "role": role_by_dt.get(dt, ""),
            "active_span_s": active_span_s,
            "message_total": msg_total,
            "message_rate_s": msg_rate_s,
            "comm_message_total": comm_total,
            "comm_message_rate_s": comm_rate_s,
            "coord_n": coord_n,
            "coord_mean_ms": coord_mean_ms,
            "coord_p95_ms": coord_p95_ms,
            "coord_ms_per_s": coord_ms_per_s,
            "compute_n": compute_n,
            "compute_mean_ms": compute_mean_ms,
            "compute_ms_per_s": compute_ms_per_s,
        }

    # Role-level normalized activity.
    role_activity: Dict[str, Dict[str, Any]] = {}
    all_roles = set(event_counts_by_role.keys()) | set(metrics_by_role.keys()) | set(role_time_bounds_ms.keys())
    for role in sorted(all_roles):
        evs = dict(event_counts_by_role.get(role, {}) or {})
        msg_total = int(sum(int(v or 0) for v in evs.values()))
        rb = role_time_bounds_ms.get(role, None)
        span_s = None
        if rb and len(rb) == 2 and float(rb[1]) >= float(rb[0]):
            span_s = max(0.001, (float(rb[1]) - float(rb[0])) / 1000.0)
        msg_rate = (float(msg_total) / float(span_s)) if span_s else None

        mm_role = dict(metrics_by_role.get(role, {}) or {})
        coord_role = dict((mm_role.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {})
        coord_mean = float(coord_role.get("mean_ms")) if coord_role.get("mean_ms") is not None else None
        coord_n = int(coord_role.get("n", 0) or 0)
        coord_total = (coord_mean * float(coord_n)) if (coord_mean is not None and coord_n > 0) else None
        coord_ms_per_s = (float(coord_total) / float(span_s)) if (coord_total is not None and span_s) else None

        comp_total = 0.0
        comp_n = 0
        for mk in ("C_intersection_tick_compute", "C_intersection_refine_compute", "C_intersection_apply_compute"):
            st = dict((mm_role.get(mk, {}) or {}).get("wall_ms", {}) or {})
            n = int(st.get("n", 0) or 0)
            mn = st.get("mean_ms", None)
            if n > 0 and mn is not None:
                comp_total += float(mn) * float(n)
                comp_n += n
        comp_ms_per_s = (float(comp_total) / float(span_s)) if (comp_n > 0 and span_s) else None

        role_activity[role] = {
            "message_total": msg_total,
            "active_span_s": span_s,
            "message_rate_s": msg_rate,
            "coord_n": coord_n,
            "coord_mean_ms": coord_mean,
            "coord_ms_per_s": coord_ms_per_s,
            "compute_n": comp_n,
            "compute_ms_per_s": comp_ms_per_s,
        }

    # Effectiveness summary oriented to reviewer interpretation.
    run_duration_s = None
    if global_t_min_ms is not None and global_t_max_ms is not None and global_t_max_ms >= global_t_min_ms:
        run_duration_s = max(0.001, (float(global_t_max_ms) - float(global_t_min_ms)) / 1000.0)

    message_volume_rates_by_role: Dict[str, Dict[str, Any]] = {}
    for role, vv in message_volume_by_role.items():
        span = None
        rb = role_time_bounds_ms.get(role, None)
        if rb and len(rb) == 2 and float(rb[1]) >= float(rb[0]):
            span = max(0.001, (float(rb[1]) - float(rb[0])) / 1000.0)
        if span is None:
            span = run_duration_s
        msgs = float(vv.get("messages", 0.0) or 0.0)
        bts = float(vv.get("bytes", 0.0) or 0.0)
        message_volume_rates_by_role[role] = {
            "messages": int(msgs),
            "bytes": bts,
            "active_span_s": span,
            "messages_per_s": (msgs / float(span)) if span else None,
            "bytes_per_s": (bts / float(span)) if span else None,
            "mean_bytes_per_message": (bts / msgs) if msgs > 0 else None,
        }

    message_volume_rates_by_service: Dict[str, Dict[str, Any]] = {}
    for svc, vv in message_volume_by_service.items():
        msgs = float(vv.get("messages", 0.0) or 0.0)
        bts = float(vv.get("bytes", 0.0) or 0.0)
        message_volume_rates_by_service[svc] = {
            "messages": int(msgs),
            "bytes": bts,
            "messages_per_s": (msgs / float(run_duration_s)) if run_duration_s else None,
            "bytes_per_s": (bts / float(run_duration_s)) if run_duration_s else None,
            "mean_bytes_per_message": (bts / msgs) if msgs > 0 else None,
        }

    message_volume_rates_by_dt: Dict[str, Dict[str, Any]] = {}
    for dt, vv in message_volume_by_dt.items():
        msgs = float(vv.get("messages", 0.0) or 0.0)
        bts = float(vv.get("bytes", 0.0) or 0.0)
        bounds = dt_time_bounds_ms.get(dt, None)
        span = None
        if bounds and len(bounds) == 2 and float(bounds[1]) >= float(bounds[0]):
            span = max(0.001, (float(bounds[1]) - float(bounds[0])) / 1000.0)
        if span is None:
            span = run_duration_s
        message_volume_rates_by_dt[dt] = {
            "messages": int(msgs),
            "bytes": bts,
            "active_span_s": span,
            "messages_per_s": (msgs / float(span)) if span else None,
            "bytes_per_s": (bts / float(span)) if span else None,
            "mean_bytes_per_message": (bts / msgs) if msgs > 0 else None,
        }

    mqtt_topics_sorted = sorted(mqtt_topic_counts.items(), key=lambda kv: int(kv[1]), reverse=True)
    mqtt_topics_top = mqtt_topics_sorted[:25]
    mqtt_topics_top_set = {t for t, _ in mqtt_topics_top}
    mqtt_topics_top_by_messages: List[Dict[str, Any]] = []
    mqtt_topics_top_by_bytes: List[Dict[str, Any]] = []
    for t, c in mqtt_topics_top:
        b = float(mqtt_topic_bytes.get(t, 0.0) or 0.0)
        tb = mqtt_topic_time_bounds_ms.get(t, None)
        span_s = None
        if tb and len(tb) == 2 and float(tb[1]) >= float(tb[0]):
            span_s = max(0.001, (float(tb[1]) - float(tb[0])) / 1000.0)
        elif run_duration_s:
            span_s = run_duration_s
        mqtt_topics_top_by_messages.append(
            {
                "topic": t,
                "messages": int(c),
                "bytes": b,
                "messages_per_s": (float(c) / float(span_s)) if span_s else None,
                "bytes_per_s": (b / float(span_s)) if span_s else None,
                "mean_bytes_per_message": (b / float(c)) if c > 0 else None,
                "active_span_s": span_s,
            }
        )
    for t, b in sorted(mqtt_topic_bytes.items(), key=lambda kv: float(kv[1]), reverse=True)[:25]:
        c = int(mqtt_topic_counts.get(t, 0) or 0)
        tb = mqtt_topic_time_bounds_ms.get(t, None)
        span_s = None
        if tb and len(tb) == 2 and float(tb[1]) >= float(tb[0]):
            span_s = max(0.001, (float(tb[1]) - float(tb[0])) / 1000.0)
        elif run_duration_s:
            span_s = run_duration_s
        mqtt_topics_top_by_bytes.append(
            {
                "topic": t,
                "messages": c,
                "bytes": float(b),
                "messages_per_s": (float(c) / float(span_s)) if span_s else None,
                "bytes_per_s": (float(b) / float(span_s)) if span_s else None,
                "mean_bytes_per_message": (float(b) / float(c)) if c > 0 else None,
                "active_span_s": span_s,
            }
        )

    # Source-topic matrix restricted to top topics for compactness.
    mqtt_service_topic_matrix: Dict[str, Dict[str, int]] = {}
    for src, row in mqtt_topic_counts_by_source.items():
        if not isinstance(row, dict):
            continue
        src_c = _canonical_service_name(src or "unknown") or "unknown"
        dst = mqtt_service_topic_matrix.setdefault(src_c, {})
        for t, c in row.items():
            if t not in mqtt_topics_top_set:
                continue
            n = int(c or 0)
            if n <= 0:
                continue
            dst[t] = dst.get(t, 0) + n

    # Edge-topic breakdown for top edges.
    edge_totals: List[Tuple[str, int]] = []
    for e, row in mqtt_topic_counts_by_edge.items():
        if not isinstance(row, dict):
            continue
        edge_totals.append((str(e), int(sum(int(v or 0) for v in row.values()))))
    edge_totals.sort(key=lambda kv: kv[1], reverse=True)
    mqtt_edge_topic_breakdown: List[Dict[str, Any]] = []
    for e, total in edge_totals[:16]:
        row = dict(mqtt_topic_counts_by_edge.get(e, {}) or {})
        tops = sorted(row.items(), key=lambda kv: int(kv[1]), reverse=True)[:6]
        mqtt_edge_topic_breakdown.append(
            {
                "edge": e,
                "messages": int(total),
                "top_topics": [{"topic": str(t), "messages": int(c or 0)} for t, c in tops],
            }
        )

    # Topic timeline (top topics only), binned at 1s.
    mqtt_topic_timeline: List[Dict[str, Any]] = []
    if mqtt_events:
        t0_ms = float(global_t_min_ms) if global_t_min_ms is not None else float(min(x[1] for x in mqtt_events))
        timeline_bins: Dict[Tuple[str, int, str], int] = {}
        for t, ts_ms, src, _rb in mqtt_events:
            if t not in mqtt_topics_top_set:
                continue
            sec = int(max(0.0, (float(ts_ms) - t0_ms) / 1000.0))
            src_c = _canonical_service_name(src or "unknown") or "unknown"
            k = (t, sec, src_c)
            timeline_bins[k] = timeline_bins.get(k, 0) + 1
        for (t, sec, src_c), cnt in sorted(timeline_bins.items(), key=lambda kv: (kv[0][1], kv[0][0], kv[0][2])):
            mqtt_topic_timeline.append(
                {
                    "topic": t,
                    "t_s": int(sec),
                    "source_service": src_c,
                    "messages": int(cnt),
                }
            )

    service_interaction_counts_data_plane: Dict[str, Dict[str, int]] = {}
    service_interaction_counts_control_plane: Dict[str, Dict[str, int]] = {}
    service_interaction_counts_cross_plane: Dict[str, Dict[str, int]] = {}
    service_interaction_nodes_by_plane: Dict[str, List[str]] = {"data": [], "control": [], "gateway": [], "unknown": []}

    for src_raw, row in service_interaction_counts.items():
        if not isinstance(row, dict):
            continue
        src = _canonical_service_name(src_raw)
        src_plane = _node_plane(src)
        for dst_raw, cnt in row.items():
            c = int(cnt or 0)
            if c <= 0:
                continue
            dst = _canonical_service_name(dst_raw)
            dst_plane = _node_plane(dst)
            service_interaction_nodes_by_plane.setdefault(src_plane, []).append(src)
            service_interaction_nodes_by_plane.setdefault(dst_plane, []).append(dst)
            if src_plane == "data" and dst_plane == "data":
                rr = service_interaction_counts_data_plane.setdefault(src, {})
                rr[dst] = rr.get(dst, 0) + c
            elif src_plane in ("control", "gateway") and dst_plane in ("control", "gateway"):
                rr = service_interaction_counts_control_plane.setdefault(src, {})
                rr[dst] = rr.get(dst, 0) + c
            else:
                rr = service_interaction_counts_cross_plane.setdefault(src, {})
                rr[dst] = rr.get(dst, 0) + c

    for k, vals in list(service_interaction_nodes_by_plane.items()):
        service_interaction_nodes_by_plane[k] = sorted(set(vals))

    service_interaction_plane_totals = {
        "data_plane_msgs": int(
            sum(int(v or 0) for r in service_interaction_counts_data_plane.values() for v in (r.values() if isinstance(r, dict) else []))
        ),
        "control_plane_msgs": int(
            sum(int(v or 0) for r in service_interaction_counts_control_plane.values() for v in (r.values() if isinstance(r, dict) else []))
        ),
        "cross_plane_msgs": int(
            sum(int(v or 0) for r in service_interaction_counts_cross_plane.values() for v in (r.values() if isinstance(r, dict) else []))
        ),
    }

    assoc_closure_ratio = (float(assoc_released_n) / float(assoc_created_n)) if assoc_created_n > 0 else None
    discovery_hit_query_ratio = (float(discovery_hits) / float(discovery_requests)) if discovery_requests > 0 else None

    coord_tls_total = len(communication_overhead_by_tls)
    coord_tls_with_latency = 0
    for _, mm in metrics_by_tls.items():
        st = dict((dict(mm.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {}))
        if int(st.get("n", 0) or 0) > 0:
            coord_tls_with_latency += 1
    coord_tls_coverage_ratio = (float(coord_tls_with_latency) / float(coord_tls_total)) if coord_tls_total > 0 else None

    coord_wall = dict((metric_summary.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {})
    coord_total_ms = None
    if int(coord_wall.get("n", 0) or 0) > 0 and coord_wall.get("mean_ms") is not None:
        coord_total_ms = float(coord_wall.get("mean_ms")) * float(int(coord_wall.get("n", 0) or 0))

    improvement_total_sec = float(sum(advice_improvement_sec)) if advice_improvement_sec else None
    benefit_cost_ratio = None
    if improvement_total_sec is not None and coord_total_ms and coord_total_ms > 0:
        benefit_cost_ratio = (improvement_total_sec * 1000.0) / float(coord_total_ms)

    ratio_values: List[float] = []
    for r in (coord_response_ratio, assoc_closure_ratio, discovery_hit_query_ratio, route_apply_ratio, coord_tls_coverage_ratio):
        if r is not None:
            ratio_values.append(max(0.0, min(1.0, float(r))))
    effectiveness_score_pct = (100.0 * statistics.fmean(ratio_values)) if ratio_values else None

    # Clear, publication-friendly KPI rollup.
    inter_compute_total_ms = 0.0
    for mk in ("C_intersection_tick_compute", "C_intersection_refine_compute", "C_intersection_apply_compute"):
        st = dict((metric_summary.get(mk, {}) or {}).get("wall_ms", {}) or {})
        n = int(st.get("n", 0) or 0)
        mn = st.get("mean_ms", None)
        if n > 0 and mn is not None:
            inter_compute_total_ms += float(mn) * float(n)
    gtco_compute_total_ms = 0.0
    for mk in (
        "C_corridor_reassess_compute",
        "C_corridor_advice_compute",
        "C_corridor_route_advice_cycle_compute",
        "C_corridor_state_pub_compute",
        "C_corridor_route_opt_compute",
    ):
        st = dict((metric_summary.get(mk, {}) or {}).get("wall_ms", {}) or {})
        n = int(st.get("n", 0) or 0)
        mn = st.get("mean_ms", None)
        if n > 0 and mn is not None:
            gtco_compute_total_ms += float(mn) * float(n)
    compute_total_ms = inter_compute_total_ms + gtco_compute_total_ms
    total_msgs = int(sum(int(vv.get("messages", 0.0) or 0.0) for vv in (message_volume_by_service or {}).values()))
    coord_n_int = int(coord_wall.get("n", 0) or 0)
    federation_kpis = {
        "run_duration_s": run_duration_s,
        "coord_decisions_n": coord_n_int,
        "coord_req_resp_success_pct": (100.0 * float(coord_response_ratio)) if coord_response_ratio is not None else None,
        "association_closure_pct": (100.0 * float(assoc_closure_ratio)) if assoc_closure_ratio is not None else None,
        "discovery_hit_pct": (100.0 * float(discovery_hit_query_ratio)) if discovery_hit_query_ratio is not None else None,
        "route_advice_apply_pct": (100.0 * float(route_apply_ratio)) if route_apply_ratio is not None else None,
        "coord_tls_coverage_pct": (100.0 * float(coord_tls_coverage_ratio)) if coord_tls_coverage_ratio is not None else None,
        "coord_mean_ms": coord_wall.get("mean_ms"),
        "coord_p95_ms": coord_wall.get("p95_ms"),
        "compute_total_ms": compute_total_ms,
        "compute_intersection_total_ms": inter_compute_total_ms,
        "compute_orchestrator_total_ms": gtco_compute_total_ms,
        "compute_per_coord_decision_ms": (compute_total_ms / float(coord_n_int)) if coord_n_int > 0 else None,
        "messages_total": total_msgs,
        "messages_per_coord_decision": (float(total_msgs) / float(coord_n_int)) if coord_n_int > 0 else None,
        "effectiveness_score_pct": effectiveness_score_pct,
    }

    fnm_state_pull_total = int(fnm_state_pull_ok + fnm_state_pull_error)
    fnm_state_pull_success_ratio = (
        float(fnm_state_pull_ok) / float(fnm_state_pull_total) if fnm_state_pull_total > 0 else None
    )
    fnm_req_publish_ratio = (
        float(fnm_req_published_events) / float(fnm_state_pull_ok) if fnm_state_pull_ok > 0 else None
    )

    offer_applied_n = int(coordination_apply_mix.get("offer_applied", 0) or 0)
    plan_applied_n = int(coordination_apply_mix.get("plan_applied", 0) or 0)
    f2_fallback_n = int(coordination_apply_mix.get("f2_local_fallback_applied", 0) or 0)
    coordination_churn_ratio = (
        float(offer_applied_n) / float(max(1, plan_applied_n)) if (offer_applied_n > 0 or plan_applied_n > 0) else None
    )
    fallback_share = (
        float(f2_fallback_n) / float(max(1, plan_applied_n)) if plan_applied_n > 0 else None
    )

    state_prop_stats = dict((metric_summary.get("T_ev_req_to_intersection_apply", {}) or {}).get("wall_ms", {}) or {})
    if int(state_prop_stats.get("n", 0) or 0) <= 0:
        state_prop_stats = dict((metric_summary.get("T_ev_req_to_signal_change", {}) or {}).get("wall_ms", {}) or {})

    req_to_actuation_stats = dict((metric_summary.get("T_coord_req_to_apply", {}) or {}).get("wall_ms", {}) or {})
    if int(req_to_actuation_stats.get("n", 0) or 0) <= 0:
        req_to_actuation_stats = dict(state_prop_stats)

    req_out_repeated_ratio = (
        float(req_out_repeated_within_1s_n) / float(req_out_observed_n)
        if req_out_observed_n > 0
        else None
    )

    integration_latency_pipeline = {
        "state_propagation_latency_ms": state_prop_stats,
        "request_to_response_latency_ms": dict((metric_summary.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {}),
        "request_to_response_latency_sim_ms": dict((metric_summary.get("T_coord_req_resp", {}) or {}).get("sim_ms", {}) or {}),
        "request_to_response_wait_gap_ms": dict((metric_summary.get("T_coord_req_resp_wait_gap", {}) or {}).get("wall_ms", {}) or {}),
        "request_to_decision_latency_ms": dict((metric_summary.get("T_coord_req_to_decision", {}) or {}).get("wall_ms", {}) or {}),
        "request_to_actuation_latency_ms": req_to_actuation_stats,
        "request_to_signal_change_latency_ms": dict((metric_summary.get("T_ev_req_to_signal_change", {}) or {}).get("wall_ms", {}) or {}),
        "request_age_latency_ms": dict((metric_summary.get("T_ev_request_age", {}) or {}).get("wall_ms", {}) or {}),
        "advice_seen_to_apply_latency_ms": dict((metric_summary.get("T_advice_seen_to_apply", {}) or {}).get("wall_ms", {}) or {}),
    }
    coordination_runtime_observability = {
        "request_out_observed_n": int(req_out_observed_n),
        "request_out_repeated_within_1s_n": int(req_out_repeated_within_1s_n),
        "request_out_repeated_within_1s_ratio": req_out_repeated_ratio,
        "request_out_recent_1s_sim": _stats(req_out_recent_1s_sim_samples),
        "request_out_dt_prev_ms": _stats(req_out_dt_prev_ms_samples),
        "request_out_pending_n": _stats(req_out_pending_n_samples),
        "request_outbox_depth": _stats(req_outbox_depth_samples),
        "outbox_depth_peak": _stats(outbox_depth_peak_samples),
        "outbox_drain_n": _stats(outbox_drain_n_samples),
        "outbox_drain_depth_peak": _stats(outbox_drain_depth_peak_samples),
    }

    fnm_overhead_stats = {
        "wall_runtime_s": _stats([float(x) * 1000.0 for x in [r.get("wall_runtime_s") for r in fnm_overhead_rows] if x is not None]),
        "cpu_runtime_s": _stats([float(x) * 1000.0 for x in [r.get("cpu_runtime_s") for r in fnm_overhead_rows] if x is not None]),
        "cpu_util_pct": _stats([float(x) for x in [r.get("cpu_util_pct") for r in fnm_overhead_rows] if x is not None]),
        "max_rss_kb": _stats([float(x) for x in [r.get("max_rss_kb") for r in fnm_overhead_rows] if x is not None]),
    }
    fnm_overhead_by_dt_stats: Dict[str, Dict[str, Any]] = {}
    for dt, row in fnm_overhead_by_dt.items():
        fnm_overhead_by_dt_stats[dt] = {
            "wall_runtime_s": _stats([1000.0 * float(x) for x in list(row.get("wall_runtime_s", []) or [])]),
            "cpu_runtime_s": _stats([1000.0 * float(x) for x in list(row.get("cpu_runtime_s", []) or [])]),
            "cpu_util_pct": _stats([float(x) for x in list(row.get("cpu_util_pct", []) or [])]),
            "max_rss_kb": _stats([float(x) for x in list(row.get("max_rss_kb", []) or [])]),
        }

    return {
        "event_counts": dict(sorted(event_counts.items(), key=lambda kv: kv[0])),
        "event_counts_by_role": event_counts_by_role,
        "metrics": metric_summary,
        "metrics_by_role": metrics_by_role,
        "metrics_by_dt": metrics_by_dt,
        "metrics_by_tls": metrics_by_tls,
        "communication_overhead_by_tls": communication_overhead_by_tls,
        "communication_overhead_by_dt": communication_overhead_by_dt,
        "computation_overhead_by_tls": computation_overhead_by_tls,
        "computation_overhead_by_dt": computation_overhead_by_dt,
        "normalized_overhead_by_dt": normalized_overhead_by_dt,
        "role_activity": role_activity,
        "message_volume_by_role": message_volume_by_role,
        "message_volume_by_dt": message_volume_by_dt,
        "message_volume_by_service": message_volume_by_service,
        "message_volume_rates_by_role": message_volume_rates_by_role,
        "message_volume_rates_by_dt": message_volume_rates_by_dt,
        "message_volume_rates_by_service": message_volume_rates_by_service,
        "service_interaction_counts": service_interaction_counts,
        "service_interaction_bytes": service_interaction_bytes,
        "service_interaction_counts_data_plane": service_interaction_counts_data_plane,
        "service_interaction_counts_control_plane": service_interaction_counts_control_plane,
        "service_interaction_counts_cross_plane": service_interaction_counts_cross_plane,
        "service_interaction_nodes_by_plane": service_interaction_nodes_by_plane,
        "service_interaction_plane_totals": service_interaction_plane_totals,
        "mqtt_topics": {
            "unique_topics": int(len(mqtt_topic_counts)),
            "origin_counts": mqtt_topic_origin_counts,
            "top_by_messages": mqtt_topics_top_by_messages,
            "top_by_bytes": mqtt_topics_top_by_bytes,
        },
        "mqtt_service_topic_matrix": mqtt_service_topic_matrix,
        "mqtt_edge_topic_breakdown": mqtt_edge_topic_breakdown,
        "mqtt_topic_timeline": mqtt_topic_timeline,
        "discovery_hit_ratio": {
            "hits": discovery_hits,
            "total": discovery_total,
            "ratio": discovery_hit_ratio,
        },
        "discovery_funnel": {
            "requests": int(discovery_requests),
            "responses": int(discovery_responses),
            "hits": int(discovery_hits),
            "misses": int(max(0, discovery_total - discovery_hits)),
            "candidates_total": int(max(0, discovery_candidates_total)),
            "avg_candidates_per_response": (
                float(discovery_candidates_total) / float(discovery_responses)
                if discovery_responses > 0
                else None
            ),
        },
        "operational_phases_ms": phase_summary,
        "federation_population": {
            "n_registered_unique": n_registered_unique,
            "n_active_ever": n_active_ever,
            "n_suspended_ever": n_suspended_ever,
            "max_active_members": int(max_active),
            "end_active_members": int(active_count),
            "active_members_timeline": active_timeline,
            "max_active_associations": int(assoc_max_active),
            "end_active_associations": int(assoc_active),
            "active_associations_timeline": assoc_active_timeline,
        },
        "fcm_runtime": {
            "event_counts": {
                "discovery_query": int(event_counts.get("fcm.discovery.query", 0) or 0),
                "discovery_response": int(event_counts.get("fcm.discovery.response", 0) or 0),
                "peer_set_update": int(event_counts.get("fcm.peer_set.update", 0) or 0),
                "peer_set_expire": int(event_counts.get("fcm.peer_set.expire", 0) or 0),
                "peer_set_reject": int(event_counts.get("fcm.peer_set.reject", 0) or 0),
                "peer_set_error": int(event_counts.get("fcm.peer_set.error", 0) or 0),
            },
            "discovery_timeline": fcm_discovery_timeline,
            "peer_events": fcm_peer_events,
        },
        "adaptive_connectivity_runtime": {
            "event_counts": {
                "binding_set_update": int(event_counts.get("adaptive.connectivity.binding_set.update", 0) or 0),
                "binding_set_expire": int(event_counts.get("adaptive.connectivity.binding_set.expire", 0) or 0),
                "binding_set_snapshot": int(event_counts.get("adaptive.connectivity.binding_set.snapshot", 0) or 0),
                "state": int(event_counts.get("adaptive.connectivity.state", 0) or 0),
            },
            "binding_events": adaptive_binding_events,
            "state_timeline": adaptive_state_timeline,
        },
        "coordination_flow": {
            "reservation_req_sent": req_n,
            "reservation_resp_recv": resp_n,
            "req_resp_ratio": coord_response_ratio,
            "association_created": assoc_created_n,
            "association_released": assoc_released_n,
            "route_advice_published": route_advice_n,
            "route_advice_received": route_advice_seen_n,
            "route_advice_applied": route_advice_applied_n,
            "route_advice_skipped": route_advice_skipped_n,
            "route_advice_apply_ratio": route_apply_ratio,
            "intersection_advice_published": intersection_advice_n,
        },
        "association_lifecycle": {
            "created": assoc_created_n,
            "released": assoc_released_n,
            "open_at_end": int(len(assoc_created_ts)),
            "state_counts": dict(sorted(assoc_state_counts.items(), key=lambda kv: kv[0])),
            "state_transition_counts": dict(sorted(assoc_transition_counts.items(), key=lambda kv: kv[0])),
            "lifetime_ms": _stats(assoc_lifecycle_samples),
        },
        "ev_advice_flow": {
            "published": route_advice_n,
            "applied": route_advice_applied_n,
            "apply_ratio": route_apply_ratio,
            "uptake_latency_ms": _stats(advice_uptake_samples),
            "seen_to_apply_latency_ms": _stats(advice_seen_apply_samples),
            "predicted_improvement_sec": _stats(advice_improvement_sec),
        },
        "ev_effectiveness": {
            "ev_service_tls_ms": _stats(ev_service_samples),
            "ev_stuck_episode_ms": _stats(ev_stuck_samples),
            "req_to_decision_ms": _stats(req_to_decision_samples),
            "req_to_apply_ms": _stats(req_to_apply_samples),
            "ev_req_to_intersection_apply_ms": _stats(ev_req_to_apply_samples),
            "ev_req_to_signal_change_ms": _stats(ev_req_to_signal_samples),
            "ev_request_age_ms": _stats(ev_request_age_samples),
            "route_advice_received": int(route_advice_seen_n),
            "route_advice_applied": int(route_advice_applied_n),
            "route_advice_skipped": int(route_advice_skipped_n),
        },
        "federation_effectiveness": {
            "run_duration_s": run_duration_s,
            "effectiveness_score_pct": effectiveness_score_pct,
            "ratios": {
                "reservation_req_resp_ratio": coord_response_ratio,
                "association_closure_ratio": assoc_closure_ratio,
                "discovery_hit_query_ratio": discovery_hit_query_ratio,
                "route_advice_apply_ratio": route_apply_ratio,
                "coord_tls_coverage_ratio": coord_tls_coverage_ratio,
            },
            "coordination_cost": {
                "coord_total_ms": coord_total_ms,
                "coord_mean_ms": coord_wall.get("mean_ms"),
                "coord_p95_ms": coord_wall.get("p95_ms"),
                "coord_n": coord_wall.get("n"),
            },
            "benefit_estimate": {
                "predicted_improvement_total_sec": improvement_total_sec,
                "predicted_improvement_mean_sec": (statistics.fmean(advice_improvement_sec) if advice_improvement_sec else None),
                "benefit_cost_ratio_sec_saved_per_coord_sec": benefit_cost_ratio,
            },
        },
        "federation_kpis": federation_kpis,
        "fnm_integration": {
            "state_pull": {
                "ok": int(fnm_state_pull_ok),
                "error": int(fnm_state_pull_error),
                "total": int(fnm_state_pull_total),
                "success_ratio": fnm_state_pull_success_ratio,
                "req_published_events": int(fnm_req_published_events),
                "req_published_total": int(fnm_req_published_total),
                "req_publish_ratio_over_ok": fnm_req_publish_ratio,
                "route_hint_published_total": int(fnm_route_hint_published_total),
                "nearest_tls_top": dict(sorted(fnm_pull_nearest_tls.items(), key=lambda kv: kv[1], reverse=True)[:12]),
            },
            "route_bridge": {
                "local_to_fed": int(fnm_local_to_fed),
                "fed_to_local": int(fnm_fed_to_local),
                "local_to_fed_by_rule": dict(sorted(fnm_local_to_fed_by_rule.items(), key=lambda kv: kv[1], reverse=True)),
                "fed_to_local_by_rule": dict(sorted(fnm_fed_to_local_by_rule.items(), key=lambda kv: kv[1], reverse=True)),
            },
            "latency_pipeline_ms": integration_latency_pipeline,
            "coordination_runtime_observability": coordination_runtime_observability,
            "delivery_success": {
                "reservation_req_resp_ratio": coord_response_ratio,
                "association_closure_ratio": assoc_closure_ratio,
                "route_advice_apply_ratio": route_apply_ratio,
                "coord_tls_coverage_ratio": coord_tls_coverage_ratio,
            },
            "timeliness": {
                "ev_stuck_episode_ms": _stats(ev_stuck_samples),
                "ev_request_age_ms": _stats(ev_request_age_samples),
                "skip_reasons": dict(sorted(coordination_skip_reasons.items(), key=lambda kv: kv[1], reverse=True)),
            },
        },
        "fnm_overhead": {
            "summary": fnm_overhead_stats,
            "by_dt": fnm_overhead_by_dt_stats,
            "rows": fnm_overhead_rows[:2000],
        },
        "coordination_diagnostics": {
            "apply_mix": coordination_apply_mix,
            "apply_mix_by_tls": coordination_apply_mix_by_tls,
            "offer_to_plan_apply_ratio": coordination_churn_ratio,
            "f2_local_fallback_share_of_plan_apply": fallback_share,
            "hard_req_skip_reasons": dict(sorted(coordination_skip_reasons.items(), key=lambda kv: kv[1], reverse=True)),
            "actuation_pairing_diagnostics": {
                "apply_matched_reqid": int(ev_apply_pair_matched_reqid),
                "apply_matched_fallback": int(ev_apply_pair_matched_fallback),
                "apply_dropped_stale": int(ev_apply_pair_dropped_stale),
                "signal_matched_reqid": int(ev_signal_pair_matched_reqid),
                "signal_matched_fallback": int(ev_signal_pair_matched_fallback),
                "signal_dropped_stale": int(ev_signal_pair_dropped_stale),
            },
        },
        "compute_observability": {
            "direct_compute_tls_count": len(direct_compute_by_tls),
            "direct_compute_metric_names": sorted(
                {
                    m
                    for _, mm in direct_compute_by_tls.items()
                    for m in (mm.keys() if isinstance(mm, dict) else [])
                }
            ),
            "has_direct_compute_samples": any(bool(v) for v in direct_compute_by_tls.values()),
        },
        "dt_phase_marks_ms": phase_marks,
        "coordination_timeline_events": timeline_events,
        "coord_session_summaries": coord_session_summaries,
        "samples": samples,
    }


def _write_summary_json(path: str, obj: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=True)


def _write_samples_csv(path: str, samples: List[Dict[str, Any]]) -> None:
    cols = [
        "metric",
        "key",
        "latency_wall_ms",
        "latency_sim_ms",
        "start_ts_wall_ms",
        "end_ts_wall_ms",
        "start_ts_sim_s",
        "end_ts_sim_s",
        "role",
        "dt_id",
        "tls_id",
        "source_service",
        "start_file",
        "start_line",
        "end_file",
        "end_line",
    ]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for s in samples:
            row = {k: s.get(k, "") for k in cols}
            w.writerow(row)


def _write_metric_summary_csv(path: str, metrics: Dict[str, Any]) -> None:
    cols = [
        "metric",
        "domain",
        "n",
        "mean_ms",
        "median_ms",
        "p25_ms",
        "p75_ms",
        "p95_ms",
        "p99_ms",
        "iqr_ms",
        "max_ms",
    ]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for metric_name, domains in sorted(metrics.items(), key=lambda kv: kv[0]):
            for domain in ("wall_ms", "sim_ms"):
                st = domains.get(domain, {}) or {}
                w.writerow(
                    {
                        "metric": metric_name,
                        "domain": domain,
                        "n": st.get("n", 0),
                        "mean_ms": st.get("mean_ms", ""),
                        "median_ms": st.get("median_ms", ""),
                        "p25_ms": st.get("p25_ms", ""),
                        "p75_ms": st.get("p75_ms", ""),
                        "p95_ms": st.get("p95_ms", ""),
                        "p99_ms": st.get("p99_ms", ""),
                        "iqr_ms": st.get("iqr_ms", ""),
                        "max_ms": st.get("max_ms", ""),
                    }
                )


def _write_coord_session_summary_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    cols = [
        "ts_wall_ms",
        "ts_sim_s",
        "scenario_id",
        "route_number",
        "mode",
        "topic_namespace",
        "ev_id",
        "tls_id",
        "dt_id",
        "apply_offer_n",
        "apply_plan_n",
        "apply_plan_offer_n",
        "apply_plan_local_fallback_n",
        "apply_plan_selected_none_n",
        "apply_plan_warmup_n",
        "plan_skip_n",
        "hard_req_skip_n",
        "selection_final_n",
        "latest_tick_compute_ms",
        "latest_refine_compute_ms",
        "latest_apply_compute_ms",
        "session_reason_counts",
        "source_file",
        "source_line",
    ]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            row = {k: r.get(k, "") for k in cols}
            w.writerow(row)


def main() -> int:
    ap = argparse.ArgumentParser(description="Extract federation metrics from JSONL logs")
    ap.add_argument(
        "--inputs",
        required=True,
        help="comma-separated JSONL files (membership/catalog/discovery/metrics/gtco etc.)",
    )
    ap.add_argument(
        "--alias-map",
        default="",
        help="optional JSON file: {\"legacy_event\":\"canonical.event.type\"}",
    )
    ap.add_argument("--out-dir", default="./fed_metrics_out", help="output directory")
    args = ap.parse_args()

    paths = [x.strip() for x in str(args.inputs).split(",") if x.strip()]
    if not paths:
        print("No input files provided")
        return 2
    for p in paths:
        if not os.path.exists(p):
            print(f"Missing input: {p}")
            return 2

    alias = _load_aliases(args.alias_map or None)
    out_dir = os.path.abspath(str(args.out_dir))
    os.makedirs(out_dir, exist_ok=True)

    res = extract(paths, alias)
    summary_path = os.path.join(out_dir, "summary.json")
    samples_csv = os.path.join(out_dir, "latency_samples.csv")
    metrics_csv = os.path.join(out_dir, "metrics_summary.csv")
    coord_session_csv = os.path.join(out_dir, "coord_session_summary.csv")
    _write_summary_json(summary_path, res)
    _write_samples_csv(samples_csv, res.get("samples", []))
    _write_metric_summary_csv(metrics_csv, res.get("metrics", {}))
    _write_coord_session_summary_csv(coord_session_csv, res.get("coord_session_summaries", []))

    print(
        json.dumps(
            {
                "out_dir": out_dir,
                "summary": summary_path,
                "samples_csv": samples_csv,
                "metrics_csv": metrics_csv,
                "coord_session_csv": coord_session_csv,
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
