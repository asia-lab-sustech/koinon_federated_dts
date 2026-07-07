#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import socket
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple
from urllib import error as url_error
from urllib import request as url_request
from urllib.parse import urlparse

import paho.mqtt.client as mqtt


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Unified EV HTTP -> MQTT bridge (request passthrough + state adaptation)."
    )
    ap.add_argument(
        "--adapter-mode",
        choices=["passthrough", "state_adapter", "hybrid"],
        default="hybrid",
        help=(
            "bridge behavior mode: "
            "passthrough=only /ev/request forwarding, "
            "state_adapter=only /ev/state adaptation (+optional pull), "
            "hybrid=both (default)"
        ),
    )
    ap.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    ap.add_argument("--port", type=int, default=18082, help="HTTP bind port")
    ap.add_argument("--mqtt-host", default="127.0.0.1", help="MQTT broker host")
    ap.add_argument("--mqtt-port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument(
        "--topic-prefix",
        default="federation/ev/request",
        help="MQTT topic prefix; publish topic is <prefix>/<tls_id>",
    )
    ap.add_argument("--qos", type=int, default=0, choices=[0, 1, 2])
    ap.add_argument("--retain", action="store_true", default=False)
    ap.add_argument(
        "--auth-bearer-token",
        default="",
        help="optional static bearer token; if set, requires Authorization: Bearer <token>",
    )
    ap.add_argument(
        "--max-body-bytes",
        type=int,
        default=1_000_000,
        help="maximum POST body size in bytes",
    )
    ap.add_argument(
        "--state-max-next-tls",
        type=int,
        default=1,
        help="when adapting state payloads, limit number of next TLS converted to EVRequest",
    )
    ap.add_argument(
        "--state-default-erl-level",
        type=int,
        default=1,
        help="fallback ERL level when state payload lacks priority information",
    )
    ap.add_argument(
        "--state-pull-url",
        default="",
        help="optional EV state URL to poll (GET) and adapt to MQTT EVRequest internally",
    )
    ap.add_argument(
        "--state-pull-sec",
        type=float,
        default=1.0,
        help="poll period in seconds for --state-pull-url",
    )
    ap.add_argument(
        "--state-pull-timeout-sec",
        type=float,
        default=0.8,
        help="HTTP timeout for state pull",
    )
    ap.add_argument(
        "--state-pull-header",
        action="append",
        default=[],
        help="optional HTTP header for state pull, format 'Key: Value' (repeatable)",
    )
    ap.add_argument(
        "--state-pull-adaptive-enable",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="enable adaptive state pull period based on nearest EV->TLS distance",
    )
    ap.add_argument(
        "--state-pull-adaptive-near-distance-m",
        type=float,
        default=200.0,
        help="nearest-distance threshold (m) for near adaptive polling",
    )
    ap.add_argument(
        "--state-pull-adaptive-near-sec",
        type=float,
        default=0.2,
        help="poll period (s) when nearest-distance <= --state-pull-adaptive-near-distance-m",
    )
    ap.add_argument(
        "--state-pull-adaptive-mid-distance-m",
        type=float,
        default=600.0,
        help="nearest-distance threshold (m) for mid adaptive polling",
    )
    ap.add_argument(
        "--state-pull-adaptive-mid-sec",
        type=float,
        default=0.5,
        help="poll period (s) when nearest-distance <= --state-pull-adaptive-mid-distance-m",
    )
    ap.add_argument(
        "--state-semantics-enable",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="enable request semantics: classify state-derived requests as track/actuate",
    )
    ap.add_argument(
        "--state-actuate-distance-m",
        type=float,
        default=500.0,
        help="distance threshold (m) for request_kind=actuate; farther requests become track",
    )
    ap.add_argument(
        "--state-track-topic-prefix",
        default="federation/ev/track",
        help="MQTT topic prefix for request_kind=track when semantics are enabled",
    )
    ap.add_argument(
        "--state-publish-track",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="publish track-class requests when semantics are enabled",
    )
    ap.add_argument(
        "--state-distance-fusion-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="use min(nextTLS distance, stopline distance) as effective request distance",
    )
    ap.add_argument(
        "--state-first-tls-actuate-only",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="treat only first next-TLS as actuate; downstream next-TLS are emitted as track",
    )
    ap.add_argument(
        "--state-require-live",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="ignore non-live EV states (live=0/false) during state adaptation",
    )
    ap.add_argument(
        "--state-pull-error-log-cooldown-sec",
        type=float,
        default=5.0,
        help="minimum seconds between repeated state_pull.error logs",
    )
    ap.add_argument(
        "--state-pull-startup-warn-sec",
        type=float,
        default=5.0,
        help="before first successful pull, emit waiting log at this cadence",
    )
    ap.add_argument(
        "--log-jsonl-file",
        default="",
        help="optional JSONL log file for bridge ingress/egress tracing",
    )
    ap.add_argument(
        "--log-jsonl-reset",
        action="store_true",
        default=False,
        help="truncate bridge JSONL log file at startup",
    )
    ap.add_argument(
        "--run-id",
        default="",
        help="optional run identifier attached to every bridge log event",
    )
    ap.add_argument(
        "--summary-period-sec",
        type=float,
        default=5.0,
        help="periodic summary event period in seconds for bridge logs; set <=0 to disable",
    )
    ap.add_argument("--verbose", action="store_true", default=False)
    return ap.parse_args()


def _get(d: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    for k in keys:
        if k in d:
            return d.get(k)
    return default


def _as_float(v: Any, default: float) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _as_int(v: Any, default: int) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _as_bool_opt(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return bool(v)
    if isinstance(v, (int, float)):
        return bool(int(v))
    s = str(v or "").strip().lower()
    if not s:
        return None
    if s in {"1", "true", "yes", "on"}:
        return True
    if s in {"0", "false", "no", "off"}:
        return False
    return None


def _unwrap_state_obj(state_obj: Dict[str, Any]) -> Dict[str, Any]:
    obj = dict(state_obj or {})
    nested = obj.get("state")
    if not isinstance(nested, dict):
        return obj
    has_primary = any(k in obj for k in ("evId", "ev_id", "snapshot", "nextTls", "next_tls", "edgeId", "edge_id"))
    if has_primary:
        return obj
    return dict(nested)


def _extract_live_flag(state_obj: Dict[str, Any]) -> Optional[bool]:
    obj = dict(state_obj or {})
    snap = dict(obj.get("snapshot", {}) or {})
    nested = obj.get("state")
    nested_snap = dict(nested.get("snapshot", {}) or {}) if isinstance(nested, dict) else {}
    for cand in (
        obj.get("live"),
        obj.get("is_live"),
        obj.get("vehicleLive"),
        snap.get("live"),
        snap.get("is_live"),
        snap.get("vehicleLive"),
        (nested.get("live") if isinstance(nested, dict) else None),
        (nested.get("is_live") if isinstance(nested, dict) else None),
        nested_snap.get("live"),
        nested_snap.get("is_live"),
    ):
        b = _as_bool_opt(cand)
        if b is not None:
            return bool(b)
    return None


def _parse_headers(raw_headers: List[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for hdr in list(raw_headers or []):
        h = str(hdr or "").strip()
        if not h or ":" not in h:
            continue
        k, v = h.split(":", 1)
        k = str(k).strip()
        v = str(v).strip()
        if k:
            out[k] = v
    return out


def _normalize_ev_request(payload: Dict[str, Any], now: float) -> Dict[str, Any]:
    req = dict(payload)
    if isinstance(req.get("ev_request"), dict):
        req = dict(req["ev_request"])

    out: Dict[str, Any] = {
        "ev_id": str(_get(req, "ev_id", "evId", default="")),
        "sim_time": float(_get(req, "sim_time", "simTime", default=now)),
        "erl_level": int(_get(req, "erl_level", "erlLevel", default=1)),
        "speed_mps": float(_get(req, "speed_mps", "speedMps", default=0.0)),
        "distance_to_intersection_m": float(
            _get(
                req,
                "distance_to_intersection_m",
                "distanceToIntersectionM",
                default=1e9,
            )
        ),
        "in_edge_id": str(_get(req, "in_edge_id", "inEdgeId", default="")),
        "target_phase_idx": _get(req, "target_phase_idx", "targetPhaseIdx", default=None),
        "delta_sec": float(_get(req, "delta_sec", "deltaSec", default=2.0)),
        "route_intersections": list(
            _get(req, "route_intersections", "routeIntersections", default=[]) or []
        )
        or None,
        "route_veh": list(_get(req, "route_veh", "routeVeh", default=[]) or []) or None,
        "source_service": str(_get(req, "source_service", default="ev_http_bridge.request")),
        "delivery": "mqtt",
    }
    return out


def _extract_request_tls_id(path: str, payload: Dict[str, Any]) -> str:
    p = urlparse(path).path
    base = "/ev/request"
    tls_from_path = ""
    if p.startswith(base + "/"):
        tls_from_path = p[len(base) + 1 :].strip("/")
    tls_from_payload = str(_get(payload, "tls_id", "tlsId", default="") or "")
    return str(tls_from_payload or tls_from_path)


def _extract_next_tls(snapshot: Dict[str, Any], max_n: int) -> List[Tuple[str, float]]:
    out: List[Tuple[str, float]] = []

    # real-world snapshot shape: nextTls=[[tls,dist], ...]
    for item in list(snapshot.get("nextTls", []) or []):
        tls_id = ""
        dist = 1e9
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            tls_id = str(item[0] or "")
            dist = _as_float(item[1], 1e9)
        elif isinstance(item, dict):
            tls_id = str(item.get("tls_id", item.get("tlsId", "")) or "")
            dist = _as_float(
                item.get("distance_to_intersection_m", item.get("distanceToIntersectionM", 1e9)),
                1e9,
            )
        if tls_id:
            out.append((tls_id, float(dist)))
        if len(out) >= int(max_n):
            break

    # alternative state shape: next_tls=[{"tls_id":..., "distance_to_intersection_m":...}, ...]
    if not out:
        for item in list(snapshot.get("next_tls", []) or []):
            if not isinstance(item, dict):
                continue
            tls_id = str(item.get("tls_id", item.get("tlsId", "")) or "")
            if not tls_id:
                continue
            dist = _as_float(
                item.get("distance_to_intersection_m", item.get("distanceToIntersectionM", 1e9)),
                1e9,
            )
            out.append((tls_id, float(dist)))
            if len(out) >= int(max_n):
                break

    return out


def _state_to_ev_requests(
    state_obj: Dict[str, Any],
    *,
    max_next_tls: int,
    default_erl_level: int,
    semantics_enable: bool,
    actuate_distance_m: float,
    distance_fusion_enable: bool,
    first_tls_actuate_only: bool,
) -> List[Tuple[str, Dict[str, Any]]]:
    profile = dict(state_obj.get("profile", {}) or {})
    snapshot = dict(state_obj.get("snapshot", {}) or state_obj)

    ev_id = str(
        snapshot.get("evId")
        or snapshot.get("ev_id")
        or state_obj.get("evId")
        or state_obj.get("ev_id")
        or ""
    )
    if not ev_id:
        return []

    now = float(time.time())
    sim_time = _as_float(snapshot.get("simTime", state_obj.get("sim_time", state_obj.get("simTime", now))), now)
    erl_level = _as_int(
        state_obj.get("erl_level", state_obj.get("erlLevel", profile.get("erlLevel", default_erl_level))),
        default_erl_level,
    )
    speed_mps = _as_float(snapshot.get("speedMps", snapshot.get("speed_mps", state_obj.get("speed_mps", 0.0))), 0.0)
    in_edge_id = str(snapshot.get("edgeId", snapshot.get("edge_id", state_obj.get("in_edge_id", ""))) or "")
    stopline_dist_m = _as_float(
        snapshot.get(
            "distToStoplineM",
            snapshot.get("dist_to_stopline_m", state_obj.get("dist_to_stopline_m", -1.0)),
        ),
        -1.0,
    )
    route_veh = list(snapshot.get("routeEdges", snapshot.get("route_veh", state_obj.get("route_veh", []))) or [])
    route_idx = _as_int(snapshot.get("routeIndex", snapshot.get("route_index", -1)), -1)
    route_intersections = list(state_obj.get("route_intersections", state_obj.get("routeIntersections", [])) or [])

    def _infer_route_in_edge_for_tls(tls_id: str, fallback_edge: str) -> str:
        tid = str(tls_id or "")
        if not tid:
            return str(fallback_edge or "")

        def _edge_matches_tls(edge_id: str, tls: str) -> bool:
            e = str(edge_id or "")
            if not e or e.startswith(":"):
                return False
            # Typical SUMO edge naming in this project: Edge<from>-<to>.
            if e.endswith(f"-{tls}"):
                return True
            tail = e.rsplit("-", 1)[-1] if "-" in e else ""
            return tail == tls

        edges = [str(e) for e in list(route_veh or []) if str(e)]
        start = max(0, int(route_idx) - 1) if int(route_idx) >= 0 else 0
        # Prefer first future route edge that approaches the target TLS.
        for e in edges[start:]:
            if _edge_matches_tls(e, tid):
                return str(e)
        # Fallback to any occurrence in full route (still deterministic).
        for e in edges:
            if _edge_matches_tls(e, tid):
                return str(e)
        fb = str(fallback_edge or "")
        if fb and not fb.startswith(":"):
            return fb
        return ""

    req_pairs: List[Tuple[str, Dict[str, Any]]] = []
    next_tls = _extract_next_tls(snapshot=snapshot, max_n=max(1, int(max_next_tls)))
    for idx, (tls_id, dist_m) in enumerate(next_tls):
        is_primary_tls = idx == 0
        eff_dist_m = float(dist_m)
        # Only fuse stopline distance for the first-next TLS. For downstream
        # TLS keep their own network distance to avoid collapsing ETA horizon.
        if bool(distance_fusion_enable) and bool(is_primary_tls) and (not str(in_edge_id).startswith(":")) and float(stopline_dist_m) >= 0.0:
            eff_dist_m = float(min(float(eff_dist_m), float(stopline_dist_m)))
        req_kind = "actuate"
        if bool(first_tls_actuate_only) and not bool(is_primary_tls):
            req_kind = "track"
        elif bool(semantics_enable):
            req_kind = "actuate" if float(eff_dist_m) <= float(actuate_distance_m) else "track"
        # Always derive per-TLS approach edge from route when possible.
        # Using current EV edge for downstream TLS creates wrong-local-context
        # requests (e.g. early Node400/Node342/Node286 with upstream edges).
        req_in_edge_id = _infer_route_in_edge_for_tls(str(tls_id), str(in_edge_id))
        if bool(first_tls_actuate_only) and not bool(is_primary_tls):
            req_in_edge_id = ""
        req = {
            "ev_id": str(ev_id),
            "sim_time": float(sim_time),
            "erl_level": int(erl_level),
            "speed_mps": float(speed_mps),
            "distance_to_intersection_m": float(eff_dist_m),
            "in_edge_id": str(req_in_edge_id),
            "target_phase_idx": None,
            "delta_sec": 2.0,
            "route_intersections": list(route_intersections) if route_intersections else None,
            "route_veh": [str(x) for x in list(route_veh or [])] or None,
            "source_service": "ev_http_bridge.state_adapter",
            "delivery": "mqtt",
            "request_kind": str(req_kind),
            "is_primary_tls": bool(is_primary_tls),
        }
        req_pairs.append((str(tls_id), req))

    # Fallback when no next TLS list is present but a single tls_id exists.
    if not req_pairs:
        tls_id = str(
            snapshot.get("tls_id")
            or snapshot.get("tlsId")
            or state_obj.get("tls_id")
            or state_obj.get("tlsId")
            or ""
        )
        if tls_id:
            fallback_dist_m = float(
                _as_float(
                    snapshot.get(
                        "distance_to_intersection_m",
                        snapshot.get(
                            "distanceToIntersectionM",
                            state_obj.get("distance_to_intersection_m", 1e9),
                        ),
                    ),
                    1e9,
                )
            )
            if bool(distance_fusion_enable) and (not str(in_edge_id).startswith(":")) and float(stopline_dist_m) >= 0.0:
                fallback_dist_m = float(min(float(fallback_dist_m), float(stopline_dist_m)))
            req_kind = "actuate"
            if bool(semantics_enable):
                req_kind = "actuate" if float(fallback_dist_m) <= float(actuate_distance_m) else "track"
            req_in_edge_id = _infer_route_in_edge_for_tls(str(tls_id), str(in_edge_id))
            req = {
                "ev_id": str(ev_id),
                "sim_time": float(sim_time),
                "erl_level": int(erl_level),
                "speed_mps": float(speed_mps),
                "distance_to_intersection_m": float(fallback_dist_m),
                "in_edge_id": str(req_in_edge_id),
                "target_phase_idx": None,
                "delta_sec": 2.0,
                "route_intersections": list(route_intersections) if route_intersections else None,
                "route_veh": [str(x) for x in list(route_veh or [])] or None,
                "source_service": "ev_http_bridge.state_adapter",
                "delivery": "mqtt",
                "request_kind": str(req_kind),
            }
            req_pairs.append((str(tls_id), req))

    return req_pairs


class BridgeHTTPServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: Tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        *,
        adapter_mode: str,
        mqtt_client: mqtt.Client,
        topic_prefix: str,
        qos: int,
        retain: bool,
        auth_bearer_token: str,
        max_body_bytes: int,
        verbose: bool,
        state_max_next_tls: int,
        state_default_erl_level: int,
        state_semantics_enable: bool,
        state_actuate_distance_m: float,
        state_track_topic_prefix: str,
        state_publish_track: bool,
        state_distance_fusion_enable: bool,
        state_first_tls_actuate_only: bool,
        state_require_live: bool,
        log_jsonl_file: str,
        run_id: str,
        summary_period_sec: float,
    ) -> None:
        super().__init__(server_address, handler_class)
        mode = str(adapter_mode or "hybrid").strip().lower()
        if mode not in {"passthrough", "state_adapter", "hybrid"}:
            mode = "hybrid"
        self.adapter_mode = str(mode)
        self.mode_enable_request_passthrough = bool(self.adapter_mode in {"passthrough", "hybrid"})
        self.mode_enable_state_adapter = bool(self.adapter_mode in {"state_adapter", "hybrid"})
        self.mqtt_client = mqtt_client
        self.topic_prefix = str(topic_prefix).rstrip("/")
        self.qos = int(qos)
        self.retain = bool(retain)
        self.auth_bearer_token = str(auth_bearer_token or "")
        self.max_body_bytes = int(max_body_bytes)
        self.verbose = bool(verbose)
        self.state_max_next_tls = max(1, int(state_max_next_tls))
        self.state_default_erl_level = int(state_default_erl_level)
        self.state_semantics_enable = bool(state_semantics_enable)
        self.state_actuate_distance_m = max(0.0, float(state_actuate_distance_m))
        self.state_track_topic_prefix = str(state_track_topic_prefix).rstrip("/")
        self.state_publish_track = bool(state_publish_track)
        self.state_distance_fusion_enable = bool(state_distance_fusion_enable)
        self.state_first_tls_actuate_only = bool(state_first_tls_actuate_only)
        self.state_require_live = bool(state_require_live)
        self.log_jsonl_file = str(log_jsonl_file or "").strip()
        self.run_id = str(run_id or "")
        self.summary_period_sec = float(summary_period_sec)
        self.lock = threading.Lock()
        self.event_seq = 0
        self.started_wall = float(time.time())
        self.last_state: Dict[str, Any] = {}
        self.last_state_source = ""
        self.last_state_wall = 0.0
        self.last_requests: List[Dict[str, Any]] = []
        self.stats: Dict[str, Any] = {
            "post_requests": 0,
            "post_states": 0,
            "pull_states": 0,
            "published_messages": 0,
            "state_adapted_messages": 0,
            "request_passthrough_messages": 0,
            "state_track_messages": 0,
            "state_actuate_messages": 0,
            "state_track_dropped": 0,
            "state_skipped_not_live": 0,
            "pull_errors": 0,
            "last_post_wall": 0.0,
            "last_publish_wall": 0.0,
            "last_error": "",
            "last_record": {},
        }

    def log_event(self, event_type: str, **payload: Any) -> None:
        path = str(self.log_jsonl_file or "").strip()
        if not path:
            return
        with self.lock:
            self.event_seq = int(self.event_seq) + 1
            seq = int(self.event_seq)
        obj = {
            "event_type": str(event_type),
            "wall_time": float(time.time()),
            "run_id": str(self.run_id),
            "seq": int(seq),
            **payload,
        }
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(obj, ensure_ascii=True) + "\n")
        except Exception:
            return

    def publish_request(
        self,
        tls_id: str,
        req: Dict[str, Any],
        source: str,
        *,
        topic_prefix_override: Optional[str] = None,
    ) -> Dict[str, Any]:
        topic_prefix = str(topic_prefix_override or self.topic_prefix).rstrip("/")
        topic = f"{topic_prefix}/{str(tls_id)}"
        info = self.mqtt_client.publish(
            topic,
            payload=json.dumps(req, ensure_ascii=True),
            qos=int(self.qos),
            retain=bool(self.retain),
        )
        rec = {
            "tls_id": str(tls_id),
            "topic": str(topic),
            "mid": int(getattr(info, "mid", -1)),
            "rc": int(getattr(info, "rc", -1)),
            "ev_id": str(req.get("ev_id", "")),
            "distance_to_intersection_m": float(req.get("distance_to_intersection_m", 1e9)),
            "in_edge_id": str(req.get("in_edge_id", "")),
            "request_kind": str(req.get("request_kind", "actuate")),
            "source": str(source),
            "wall": float(time.time()),
        }
        with self.lock:
            self.stats["published_messages"] = int(self.stats.get("published_messages", 0)) + 1
            self.stats["last_publish_wall"] = float(time.time())
            self.stats["last_record"] = dict(rec)
            if str(source).startswith("state"):
                self.stats["state_adapted_messages"] = int(self.stats.get("state_adapted_messages", 0)) + 1
            else:
                self.stats["request_passthrough_messages"] = int(self.stats.get("request_passthrough_messages", 0)) + 1
            self.last_requests = [dict(rec)] + list(self.last_requests[:19])

        if self.verbose:
            print(
                "[ev-http-bridge] "
                f"published source={source} topic={topic} ev={req.get('ev_id')} tls={tls_id} "
                f"dist={float(req.get('distance_to_intersection_m', 1e9)):.2f} "
                f"edge={req.get('in_edge_id', '-')}"
            )
        self.log_event("bridge.publish", record=dict(rec), req=dict(req))
        return rec

    def snapshot_stats(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.stats)

    def ingest_state(self, state_obj: Dict[str, Any], *, source: str) -> List[Dict[str, Any]]:
        raw_state = dict(state_obj or {})
        normalized_state = _unwrap_state_obj(raw_state)
        live_flag = _extract_live_flag(raw_state)
        with self.lock:
            self.last_state = dict(normalized_state)
            self.last_state_source = str(source)
            self.last_state_wall = float(time.time())
        if bool(self.state_require_live) and (live_flag is False):
            with self.lock:
                self.stats["state_skipped_not_live"] = int(self.stats.get("state_skipped_not_live", 0)) + 1
                self.stats["last_error"] = "state_not_live_skipped"
            self.log_event(
                "bridge.state_skip_not_live",
                source=str(source),
                live=bool(live_flag),
                state_summary={
                    "evId": normalized_state.get("evId") or (normalized_state.get("snapshot") or {}).get("evId"),
                    "simTime": normalized_state.get("simTime") or (normalized_state.get("snapshot") or {}).get("simTime"),
                },
            )
            return []
        req_pairs = _state_to_ev_requests(
            state_obj=normalized_state,
            max_next_tls=self.state_max_next_tls,
            default_erl_level=self.state_default_erl_level,
            semantics_enable=bool(self.state_semantics_enable),
            actuate_distance_m=float(self.state_actuate_distance_m),
            distance_fusion_enable=bool(self.state_distance_fusion_enable),
            first_tls_actuate_only=bool(self.state_first_tls_actuate_only),
        )
        self.log_event(
            "bridge.state_ingest",
            source=str(source),
            state_summary={
                "evId": normalized_state.get("evId") or (normalized_state.get("snapshot") or {}).get("evId"),
                "simTime": normalized_state.get("simTime") or (normalized_state.get("snapshot") or {}).get("simTime"),
            },
            live=live_flag,
        )

        recs: List[Dict[str, Any]] = []
        for tls_id, req in req_pairs:
            if not tls_id or not str(req.get("ev_id", "")):
                continue
            req_kind = str(req.get("request_kind", "actuate") or "actuate")
            topic_override: Optional[str] = None
            if req_kind == "track":
                if not bool(self.state_publish_track):
                    with self.lock:
                        self.stats["state_track_dropped"] = int(self.stats.get("state_track_dropped", 0)) + 1
                    self.log_event(
                        "bridge.track.drop",
                        source=str(source),
                        tls_id=str(tls_id),
                        ev_id=str(req.get("ev_id", "")),
                        distance_to_intersection_m=float(req.get("distance_to_intersection_m", 1e9)),
                    )
                    continue
                topic_override = str(self.state_track_topic_prefix or "").rstrip("/") or None
            else:
                topic_override = None
            rec = self.publish_request(
                str(tls_id),
                req,
                source=f"state:{source}",
                topic_prefix_override=topic_override,
            )
            with self.lock:
                if req_kind == "track":
                    self.stats["state_track_messages"] = int(self.stats.get("state_track_messages", 0)) + 1
                else:
                    self.stats["state_actuate_messages"] = int(self.stats.get("state_actuate_messages", 0)) + 1
            recs.append(rec)
        return recs

    def state_view(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "ok": bool(self.last_state),
                "source": str(self.last_state_source or ""),
                "wall": float(self.last_state_wall or 0.0),
                "state": dict(self.last_state or {}),
            }


class Handler(BaseHTTPRequestHandler):
    server: BridgeHTTPServer

    def _json(self, code: int, obj: Dict[str, Any]) -> None:
        body = json.dumps(obj, ensure_ascii=True).encode("utf-8")
        self.send_response(int(code))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _auth_ok(self) -> bool:
        expected = str(self.server.auth_bearer_token or "")
        if not expected:
            return True
        got = str(self.headers.get("Authorization", "") or "")
        return got == f"Bearer {expected}"

    def _read_json_entries(self) -> Tuple[Optional[List[Dict[str, Any]]], Optional[str]]:
        try:
            clen = int(self.headers.get("Content-Length", "0") or "0")
        except Exception:
            clen = 0
        if clen <= 0:
            return None, "empty_body"
        if clen > int(self.server.max_body_bytes):
            return None, "payload_too_large"

        raw = self.rfile.read(clen)
        try:
            payload = json.loads(raw.decode("utf-8", errors="replace"))
        except Exception:
            return None, "invalid_json"

        if isinstance(payload, list):
            entries = [x for x in payload if isinstance(x, dict)]
        elif isinstance(payload, dict):
            entries = [payload]
        else:
            return None, "invalid_payload_type"

        if not entries:
            return None, "no_valid_entries"
        return entries, None

    def do_GET(self) -> None:  # noqa: N802
        p = urlparse(self.path).path
        if p in ("/health", "/healthz"):
            self._json(
                200,
                {
                    "ok": True,
                    "service": "ev_http_to_mqtt_bridge",
                    "adapter_mode": str(self.server.adapter_mode),
                    "topic_prefix": self.server.topic_prefix,
                    "ts": time.time(),
                },
            )
            return
        if p in ("/stats", "/metrics"):
            stats = self.server.snapshot_stats()
            self._json(
                200,
                {
                    "ok": True,
                    "service": "ev_http_to_mqtt_bridge",
                    "adapter_mode": str(self.server.adapter_mode),
                    "run_id": str(self.server.run_id),
                    "stats": stats,
                },
            )
            return
        if p in ("/ev/state",):
            self._json(200, self.server.state_view())
            return
        if p in ("/ev/requests/last",):
            with self.server.lock:
                recs = list(self.server.last_requests)
            self._json(200, {"ok": True, "records": recs, "n": len(recs)})
            return
        self._json(404, {"ok": False, "error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        p = urlparse(self.path).path
        is_req = p == "/ev/request" or p.startswith("/ev/request/")
        is_state = p == "/ev/state"
        if not (is_req or is_state):
            self._json(404, {"ok": False, "error": "not_found"})
            return
        if is_req and not bool(self.server.mode_enable_request_passthrough):
            self.server.log_event(
                "bridge.post.error",
                path=str(self.path),
                error="mode_disallows_request_endpoint",
                adapter_mode=str(self.server.adapter_mode),
            )
            self._json(
                409,
                {
                    "ok": False,
                    "error": "mode_disallows_request_endpoint",
                    "adapter_mode": str(self.server.adapter_mode),
                },
            )
            return
        if is_state and not bool(self.server.mode_enable_state_adapter):
            self.server.log_event(
                "bridge.post.error",
                path=str(self.path),
                error="mode_disallows_state_endpoint",
                adapter_mode=str(self.server.adapter_mode),
            )
            self._json(
                409,
                {
                    "ok": False,
                    "error": "mode_disallows_state_endpoint",
                    "adapter_mode": str(self.server.adapter_mode),
                },
            )
            return

        if not self._auth_ok():
            self._json(401, {"ok": False, "error": "unauthorized"})
            return

        entries, err = self._read_json_entries()
        if err:
            with self.server.lock:
                self.server.stats["last_error"] = str(err)
            self.server.log_event("bridge.post.error", path=str(self.path), error=str(err))
            self._json(400, {"ok": False, "error": err})
            return

        now = float(time.time())
        with self.server.lock:
            self.server.stats["last_post_wall"] = float(now)

        published: List[Dict[str, Any]] = []

        if is_req:
            with self.server.lock:
                self.server.stats["post_requests"] = int(self.server.stats.get("post_requests", 0)) + len(entries or [])
            for ent in list(entries or []):
                tls_id = _extract_request_tls_id(self.path, ent)
                if not tls_id:
                    continue
                req = _normalize_ev_request(ent, now=now)
                if not req.get("ev_id"):
                    continue
                self.server.log_event(
                    "bridge.request_ingest",
                    path=str(self.path),
                    tls_id=str(tls_id),
                    ev_id=str(req.get("ev_id", "")),
                    source="request_post",
                )
                published.append(self.server.publish_request(str(tls_id), req, source="request_post"))

        elif is_state:
            with self.server.lock:
                self.server.stats["post_states"] = int(self.server.stats.get("post_states", 0)) + len(entries or [])
            for ent in list(entries or []):
                recs = self.server.ingest_state(dict(ent), source="post")
                published.extend(recs)
            if self.server.verbose and entries:
                try:
                    sample = dict(entries[0])
                    snap = dict(sample.get("snapshot", {}) or sample)
                    print(
                        "[ev-http-bridge] "
                        f"state_ingest source=post ev={snap.get('evId', snap.get('ev_id', '-'))} "
                        f"edge={snap.get('edgeId', snap.get('edge_id', '-'))} "
                        f"speed={_as_float(snap.get('speedMps', snap.get('speed_mps', 0.0)), 0.0):.2f} "
                        f"published={len(recs)}"
                    )
                except Exception:
                    pass

        if self.server.verbose:
            print(
                f"[ev-http-bridge] path={self.path} in={len(entries or [])} published={len(published)}"
            )

        if not published:
            with self.server.lock:
                self.server.stats["last_error"] = "no_publishable_entries"
            self.server.log_event("bridge.post.error", path=str(self.path), error="no_publishable_entries")
            self._json(400, {"ok": False, "error": "no_publishable_entries"})
            return

        self.server.log_event(
            "bridge.post.ok",
            path=str(self.path),
            accepted=int(len(entries or [])),
            published=int(len(published)),
            first_record=dict(published[0]) if published else {},
        )
        self._json(
            202,
            {
                "ok": True,
                "accepted": len(entries or []),
                "published": len(published),
                "records": published,
            },
        )

    def log_message(self, _fmt: str, *_args: object) -> None:
        return


def _http_get_json(url: str, timeout_sec: float, headers: Dict[str, str]) -> Optional[Dict[str, Any]]:
    req = url_request.Request(str(url), headers=dict(headers), method="GET")
    try:
        with url_request.urlopen(req, timeout=float(timeout_sec)) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
            obj = json.loads(raw)
            if isinstance(obj, dict):
                return dict(obj)
            return None
    except (url_error.URLError, TimeoutError, socket.timeout, ValueError):
        return None
    except Exception:
        return None


def _state_pull_loop(
    srv: BridgeHTTPServer,
    *,
    pull_url: str,
    poll_sec: float,
    adaptive_enable: bool,
    adaptive_near_distance_m: float,
    adaptive_near_sec: float,
    adaptive_mid_distance_m: float,
    adaptive_mid_sec: float,
    timeout_sec: float,
    headers: Dict[str, str],
    error_log_cooldown_sec: float,
    startup_warn_sec: float,
    stop_evt: threading.Event,
) -> None:
    def _nearest_distance_m(state_obj: Dict[str, Any]) -> Optional[float]:
        try:
            snapshot = dict(state_obj.get("snapshot", {}) or state_obj)
            dvals: List[float] = []
            for _tls, dist in _extract_next_tls(snapshot=snapshot, max_n=8):
                try:
                    d = float(dist)
                    if d >= 0.0:
                        dvals.append(d)
                except Exception:
                    continue
            try:
                d_stop = float(
                    snapshot.get(
                        "distToStoplineM",
                        snapshot.get("dist_to_stopline_m", -1.0),
                    )
                )
                if d_stop >= 0.0:
                    dvals.append(float(d_stop))
            except Exception:
                pass
            if not dvals:
                return None
            return float(min(dvals))
        except Exception:
            return None

    ps = max(0.1, float(poll_sec))
    near_sec = max(0.1, float(adaptive_near_sec))
    mid_sec = max(0.1, float(adaptive_mid_sec))
    near_m = max(0.0, float(adaptive_near_distance_m))
    mid_m = max(near_m, float(adaptive_mid_distance_m))
    sleep_target = float(ps)
    err_cooldown = max(0.1, float(error_log_cooldown_sec))
    startup_warn = max(0.1, float(startup_warn_sec))
    had_ok = False
    consecutive_errors = 0
    last_err_log_wall = 0.0
    last_wait_log_wall = 0.0
    while not stop_evt.is_set():
        t0 = time.perf_counter()
        nearest_m: Optional[float] = None
        state_obj = _http_get_json(url=pull_url, timeout_sec=float(timeout_sec), headers=headers)
        if state_obj is None:
            now_wall = float(time.time())
            consecutive_errors += 1
            with srv.lock:
                srv.stats["last_error"] = "state_pull_failed"
                srv.stats["pull_errors"] = int(srv.stats.get("pull_errors", 0)) + 1
            if not bool(had_ok):
                if (now_wall - float(last_wait_log_wall)) >= float(startup_warn):
                    srv.log_event(
                        "bridge.state_pull.waiting_source",
                        url=str(pull_url),
                        consecutive_errors=int(consecutive_errors),
                        next_poll_sec=float(ps),
                    )
                    if srv.verbose:
                        print(
                            "[ev-http-bridge] "
                            f"state_pull waiting_source=1 errors={int(consecutive_errors)} "
                            f"url={pull_url} next_poll={float(ps):.2f}s"
                        )
                    last_wait_log_wall = float(now_wall)
            else:
                if (now_wall - float(last_err_log_wall)) >= float(err_cooldown):
                    srv.log_event(
                        "bridge.state_pull.error",
                        url=str(pull_url),
                        error="state_pull_failed",
                        consecutive_errors=int(consecutive_errors),
                        cooldown_sec=float(err_cooldown),
                    )
                    if srv.verbose:
                        print(
                            "[ev-http-bridge] "
                            f"state_pull error=state_pull_failed errors={int(consecutive_errors)} "
                            f"next_poll={float(ps):.2f}s"
                        )
                    last_err_log_wall = float(now_wall)
            sleep_target = float(ps)
        else:
            if int(consecutive_errors) > 0:
                srv.log_event(
                    "bridge.state_pull.recovered",
                    url=str(pull_url),
                    previous_errors=int(consecutive_errors),
                )
                if srv.verbose:
                    print(
                        "[ev-http-bridge] "
                        f"state_pull recovered previous_errors={int(consecutive_errors)}"
                    )
            had_ok = True
            consecutive_errors = 0
            with srv.lock:
                srv.stats["pull_states"] = int(srv.stats.get("pull_states", 0)) + 1
            recs = srv.ingest_state(state_obj, source="pull")
            nearest_m = _nearest_distance_m(state_obj)
            if bool(adaptive_enable) and nearest_m is not None:
                if float(nearest_m) <= float(near_m):
                    sleep_target = float(near_sec)
                elif float(nearest_m) <= float(mid_m):
                    sleep_target = float(mid_sec)
                else:
                    sleep_target = float(ps)
            else:
                sleep_target = float(ps)
            srv.log_event(
                "bridge.state_pull.ok",
                url=str(pull_url),
                published=int(len(recs)),
                first_record=dict(recs[0]) if recs else {},
                nearest_distance_m=(None if nearest_m is None else float(nearest_m)),
                next_poll_sec=float(sleep_target),
            )
            if srv.verbose:
                snap = dict(state_obj.get("snapshot", {}) or state_obj)
                print(
                    "[ev-http-bridge] "
                    f"state_pull ok=1 ev={snap.get('evId', snap.get('ev_id', '-'))} "
                    f"edge={snap.get('edgeId', snap.get('edge_id', '-'))} "
                    f"speed={_as_float(snap.get('speedMps', snap.get('speed_mps', 0.0)), 0.0):.2f} "
                    f"published={len(recs)} near_m={('-' if nearest_m is None else f'{float(nearest_m):.1f}')} "
                    f"next_poll={float(sleep_target):.2f}s"
                )

        dt = time.perf_counter() - t0
        sleep_s = max(0.05, float(sleep_target) - dt)
        stop_evt.wait(timeout=sleep_s)


def _summary_loop(srv: BridgeHTTPServer, *, stop_evt: threading.Event) -> None:
    period = float(srv.summary_period_sec)
    if period <= 0.0:
        return
    while not stop_evt.wait(timeout=period):
        stats = srv.snapshot_stats()
        uptime_s = max(0.0, float(time.time()) - float(srv.started_wall))
        srv.log_event("bridge.summary", uptime_s=float(uptime_s), stats=stats)
        if srv.verbose:
            print(
                "[ev-http-bridge] "
                f"summary run_id={srv.run_id or '-'} uptime_s={uptime_s:.1f} "
                f"published={int(stats.get('published_messages', 0))} "
                f"post_req={int(stats.get('post_requests', 0))} "
                f"post_state={int(stats.get('post_states', 0))} "
                f"pull_state={int(stats.get('pull_states', 0))} "
                f"last_error={str(stats.get('last_error', '') or '-')}"
            )


def main() -> None:
    args = parse_args()
    adapter_mode = str(args.adapter_mode or "hybrid").strip().lower()
    if adapter_mode not in {"passthrough", "state_adapter", "hybrid"}:
        adapter_mode = "hybrid"
    mode_enable_request_passthrough = bool(adapter_mode in {"passthrough", "hybrid"})
    mode_enable_state_adapter = bool(adapter_mode in {"state_adapter", "hybrid"})

    if (
        bool(mode_enable_state_adapter)
        and bool(args.state_first_tls_actuate_only)
        and (not bool(args.state_publish_track))
        and int(args.state_max_next_tls) > 1
    ):
        # Guardrail: with first-TLS-only + track disabled, downstream requests are
        # dropped entirely, which can delay local preemption at upcoming TLS.
        print(
            "[ev-http-bridge] "
            "warn: state_first_tls_actuate_only=1 with state_publish_track=0 and state_max_next_tls>1 "
            "would drop downstream TLS requests; auto-disabling first_tls_actuate_only for this run."
        )
        args.state_first_tls_actuate_only = False
    run_id = str(args.run_id or "").strip()
    if not run_id:
        run_id = datetime.now().strftime("bridge_%Y%m%d_%H%M%S")
    log_jsonl_file = str(args.log_jsonl_file or "").strip()
    if log_jsonl_file and bool(args.log_jsonl_reset):
        try:
            os.makedirs(os.path.dirname(log_jsonl_file) or ".", exist_ok=True)
            with open(log_jsonl_file, "w", encoding="utf-8"):
                pass
        except Exception:
            pass

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(str(args.mqtt_host), int(args.mqtt_port), 60)
    client.loop_start()

    srv = BridgeHTTPServer(
        (str(args.host), int(args.port)),
        Handler,
        adapter_mode=str(adapter_mode),
        mqtt_client=client,
        topic_prefix=str(args.topic_prefix),
        qos=int(args.qos),
        retain=bool(args.retain),
        auth_bearer_token=str(args.auth_bearer_token or ""),
        max_body_bytes=int(args.max_body_bytes),
        verbose=bool(args.verbose),
        state_max_next_tls=max(1, int(args.state_max_next_tls)),
        state_default_erl_level=int(args.state_default_erl_level),
        state_semantics_enable=bool(args.state_semantics_enable),
        state_actuate_distance_m=float(args.state_actuate_distance_m),
        state_track_topic_prefix=str(args.state_track_topic_prefix),
        state_publish_track=bool(args.state_publish_track),
        state_distance_fusion_enable=bool(args.state_distance_fusion_enable),
        state_first_tls_actuate_only=bool(args.state_first_tls_actuate_only),
        state_require_live=bool(args.state_require_live),
        log_jsonl_file=log_jsonl_file,
        run_id=str(run_id),
        summary_period_sec=float(args.summary_period_sec),
    )

    pull_url = str(args.state_pull_url or "").strip()
    pull_headers = _parse_headers(list(args.state_pull_header or []))
    stop_evt = threading.Event()
    pull_thread: Optional[threading.Thread] = None
    summary_thread: Optional[threading.Thread] = None
    if pull_url and bool(mode_enable_state_adapter):
        pull_thread = threading.Thread(
            target=_state_pull_loop,
            kwargs={
                "srv": srv,
                "pull_url": pull_url,
                "poll_sec": float(args.state_pull_sec),
                "adaptive_enable": bool(args.state_pull_adaptive_enable),
                "adaptive_near_distance_m": float(args.state_pull_adaptive_near_distance_m),
                "adaptive_near_sec": float(args.state_pull_adaptive_near_sec),
                "adaptive_mid_distance_m": float(args.state_pull_adaptive_mid_distance_m),
                "adaptive_mid_sec": float(args.state_pull_adaptive_mid_sec),
                "timeout_sec": float(args.state_pull_timeout_sec),
                "headers": pull_headers,
                "error_log_cooldown_sec": float(args.state_pull_error_log_cooldown_sec),
                "startup_warn_sec": float(args.state_pull_startup_warn_sec),
                "stop_evt": stop_evt,
            },
            name="ev-state-pull-loop",
            daemon=True,
        )
        pull_thread.start()
    elif pull_url and (not bool(mode_enable_state_adapter)):
        print(
            "[ev-http-bridge] "
            "warn: --state-pull-url ignored because adapter_mode=passthrough disables state adaptation."
        )
    if float(args.summary_period_sec) > 0.0:
        summary_thread = threading.Thread(
            target=_summary_loop,
            kwargs={"srv": srv, "stop_evt": stop_evt},
            name="bridge-summary-loop",
            daemon=True,
        )
        summary_thread.start()

    print(
        f"[ev-http-bridge] listening http://{args.host}:{args.port} "
        f"-> mqtt://{args.mqtt_host}:{args.mqtt_port} prefix={str(args.topic_prefix).rstrip('/')} "
        f"mode={adapter_mode} "
        f"req_passthrough={1 if bool(mode_enable_request_passthrough) else 0} "
        f"state_adapter={1 if bool(mode_enable_state_adapter) else 0} "
        f"state_pull={'on' if (pull_url and bool(mode_enable_state_adapter)) else 'off'}"
    )
    print(f"[ev-http-bridge] run_id={run_id} summary_period={float(args.summary_period_sec):.1f}s")
    if log_jsonl_file:
        print(f"[ev-http-bridge] log_jsonl={log_jsonl_file}")
    if pull_url and bool(mode_enable_state_adapter):
        print(
            f"[ev-http-bridge] state_pull_url={pull_url} poll={float(args.state_pull_sec):.2f}s "
            f"timeout={float(args.state_pull_timeout_sec):.2f}s headers_n={len(pull_headers)} "
            f"adaptive={1 if bool(args.state_pull_adaptive_enable) else 0} "
            f"near_m={float(args.state_pull_adaptive_near_distance_m):.1f} near_sec={float(args.state_pull_adaptive_near_sec):.2f} "
            f"mid_m={float(args.state_pull_adaptive_mid_distance_m):.1f} mid_sec={float(args.state_pull_adaptive_mid_sec):.2f}"
        )
    if bool(mode_enable_state_adapter):
        print(
            f"[ev-http-bridge] semantics={1 if bool(args.state_semantics_enable) else 0} "
            f"actuate_m={float(args.state_actuate_distance_m):.1f} "
            f"track_prefix={str(args.state_track_topic_prefix).rstrip('/')} "
            f"publish_track={1 if bool(args.state_publish_track) else 0} "
            f"distance_fusion={1 if bool(args.state_distance_fusion_enable) else 0} "
            f"first_tls_actuate_only={1 if bool(args.state_first_tls_actuate_only) else 0} "
            f"require_live={1 if bool(args.state_require_live) else 0}"
        )
    else:
        print("[ev-http-bridge] state adaptation knobs are inactive in passthrough mode")
    srv.log_event(
        "bridge.start",
        run_id=str(run_id),
        config={
            "adapter_mode": str(adapter_mode),
            "mode_enable_request_passthrough": bool(mode_enable_request_passthrough),
            "mode_enable_state_adapter": bool(mode_enable_state_adapter),
            "host": str(args.host),
            "port": int(args.port),
            "mqtt_host": str(args.mqtt_host),
            "mqtt_port": int(args.mqtt_port),
            "topic_prefix": str(args.topic_prefix),
            "state_pull_url": (str(pull_url) if bool(mode_enable_state_adapter) else ""),
            "state_pull_sec": float(args.state_pull_sec),
            "state_pull_timeout_sec": float(args.state_pull_timeout_sec),
            "state_pull_adaptive_enable": bool(args.state_pull_adaptive_enable),
            "state_pull_adaptive_near_distance_m": float(args.state_pull_adaptive_near_distance_m),
            "state_pull_adaptive_near_sec": float(args.state_pull_adaptive_near_sec),
            "state_pull_adaptive_mid_distance_m": float(args.state_pull_adaptive_mid_distance_m),
            "state_pull_adaptive_mid_sec": float(args.state_pull_adaptive_mid_sec),
            "state_max_next_tls": int(args.state_max_next_tls),
            "state_default_erl_level": int(args.state_default_erl_level),
            "state_semantics_enable": bool(args.state_semantics_enable),
            "state_actuate_distance_m": float(args.state_actuate_distance_m),
            "state_track_topic_prefix": str(args.state_track_topic_prefix),
            "state_publish_track": bool(args.state_publish_track),
            "state_distance_fusion_enable": bool(args.state_distance_fusion_enable),
            "state_first_tls_actuate_only": bool(args.state_first_tls_actuate_only),
            "state_require_live": bool(args.state_require_live),
            "state_pull_error_log_cooldown_sec": float(args.state_pull_error_log_cooldown_sec),
            "state_pull_startup_warn_sec": float(args.state_pull_startup_warn_sec),
            "qos": int(args.qos),
            "retain": bool(args.retain),
        },
    )

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        try:
            if pull_thread is not None:
                pull_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if summary_thread is not None:
                summary_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            srv.log_event(
                "bridge.stop",
                run_id=str(run_id),
                uptime_s=max(0.0, float(time.time()) - float(srv.started_wall)),
                stats=srv.snapshot_stats(),
            )
        except Exception:
            pass
        try:
            srv.server_close()
        finally:
            client.loop_stop()
            client.disconnect()


if __name__ == "__main__":
    main()
