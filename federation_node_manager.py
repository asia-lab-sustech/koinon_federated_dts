#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple
from urllib import error as url_error
from urllib import request as url_request

import paho.mqtt.client as mqtt
import yaml

from fnm_peer_selection_policies import build_peer_selection_policy

try:
    import resource  # type: ignore
except Exception:  # pragma: no cover
    resource = None  # type: ignore


def _now() -> float:
    return float(time.time())


def _new_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:10]}"


def _json_dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def _json_loads(raw: bytes) -> Dict[str, Any]:
    try:
        return dict(json.loads(raw.decode("utf-8")))
    except Exception:
        return {}


def _mqtt_reason_ok(reason_code: Any) -> bool:
    try:
        return int(reason_code) == 0
    except Exception:
        return str(reason_code).strip().lower() in {"0", "success", "normal disconnection"}


def _make_mqtt_client(client_id: str) -> Any:
    """Create a Paho client with conservative callback compatibility."""
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


def _file_sha256_short(path: str, n: int = 12) -> str:
    p = str(path or "").strip()
    if not p or not os.path.exists(p):
        return ""
    try:
        b = open(p, "rb").read()
        return hashlib.sha256(b).hexdigest()[: max(4, int(n))]
    except Exception:
        return ""


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _as_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _as_list(v: Any) -> List[Any]:
    if v is None:
        return []
    if isinstance(v, list):
        return v
    return [v]


def _as_str_list(v: Any) -> List[str]:
    out: List[str] = []
    for x in _as_list(v):
        s = str(x).strip()
        if s:
            out.append(s)
    return out


def _deep_get(d: Dict[str, Any], path: List[str], default: Any = None) -> Any:
    cur: Any = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def _topic_match(sub: str, topic: str) -> bool:
    s = str(sub).split("/")
    t = str(topic).split("/")
    i = j = 0
    while i < len(s) and j < len(t):
        if s[i] == "#":
            return True
        if s[i] == "+":
            i += 1
            j += 1
            continue
        if s[i] != t[j]:
            return False
        i += 1
        j += 1
    if i < len(s) and s[i] == "#":
        return True
    return i == len(s) and j == len(t)


def _topic_wildcard_values(sub: str, topic: str) -> List[str]:
    s = str(sub or "").split("/")
    t = str(topic or "").split("/")
    vals: List[str] = []
    i = j = 0
    while i < len(s) and j < len(t):
        if s[i] == "#":
            vals.extend(t[j:])
            return vals
        if s[i] == "+":
            vals.append(t[j])
        elif s[i] != t[j]:
            return []
        i += 1
        j += 1
    return vals if i == len(s) and j == len(t) else []


def _render_wildcard_publish_topic(sub: str, topic: str, publish_template: str) -> str:
    out_parts: List[str] = []
    wildcard_values = _topic_wildcard_values(sub, topic)
    wildcard_idx = 0
    for part in str(publish_template or "").split("/"):
        if part == "+" and wildcard_idx < len(wildcard_values):
            out_parts.append(str(wildcard_values[wildcard_idx]))
            wildcard_idx += 1
        else:
            out_parts.append(str(part))
    return "/".join(out_parts)


def _expand_topic(template: str, dt_id: str) -> str:
    t = str(template or "")
    if not t:
        return t
    try:
        return t.format(dt_id)
    except Exception:
        pass
    try:
        return t.format(dt_id=dt_id, id=dt_id)
    except Exception:
        return t


class LocalNodeDataManager:
    """
    Lightweight per-node data manager for sidecar runs.
    It is intentionally file-based for prototype experiments.
    """

    def __init__(
        self,
        *,
        node_cfg: Dict[str, Any],
        dt_id: str,
        gateway_id: str,
        explicit_trace_jsonl: str = "",
    ) -> None:
        dm_cfg = dict((node_cfg or {}).get("data_manager", {}) or {})
        self.enabled = bool(dm_cfg.get("enabled", False))
        self.persist_raw_messages = bool(dm_cfg.get("persist_raw_messages", True))
        self.persist_manifest = bool(dm_cfg.get("persist_manifest", True))
        self.dt_id = str(dt_id)
        self.gateway_id = str(gateway_id)

        base_dir = str(dm_cfg.get("base_dir", "") or "").strip()
        run_id = str(dm_cfg.get("run_id", "") or "").strip()
        if not run_id:
            run_id = time.strftime("run_%Y%m%d_%H%M%S", time.localtime(_now()))
        self.run_id = run_id

        self.node_dir = ""
        if self.enabled and base_dir:
            self.node_dir = os.path.join(os.path.abspath(base_dir), self.run_id, self.gateway_id)
            os.makedirs(self.node_dir, exist_ok=True)

        cfg_trace = str(dm_cfg.get("trace_jsonl", "") or "").strip()
        self.trace_jsonl_path = str(explicit_trace_jsonl or cfg_trace or "").strip()
        if not self.trace_jsonl_path and self.node_dir:
            self.trace_jsonl_path = os.path.join(self.node_dir, "trace.jsonl")
        if self.trace_jsonl_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.trace_jsonl_path)), exist_ok=True)

        self.raw_messages_path = ""
        if self.node_dir and self.persist_raw_messages:
            self.raw_messages_path = os.path.join(self.node_dir, "raw_messages.jsonl")

        self.manifest_path = ""
        if self.node_dir and self.persist_manifest:
            self.manifest_path = os.path.join(self.node_dir, "manifest.json")
            self._write_manifest(dm_cfg=dm_cfg, node_cfg=node_cfg)

    def _write_manifest(self, *, dm_cfg: Dict[str, Any], node_cfg: Dict[str, Any]) -> None:
        payload = {
            "schema": "fnm.data_manager.manifest.v1",
            "ts": _now(),
            "run_id": self.run_id,
            "gateway_id": self.gateway_id,
            "dt_id": self.dt_id,
            "data_manager": {
                "enabled": self.enabled,
                "persist_raw_messages": self.persist_raw_messages,
                "trace_jsonl_path": self.trace_jsonl_path,
                "raw_messages_path": self.raw_messages_path,
            },
            "node": {
                "dt_type": str((node_cfg or {}).get("dt_type", "")),
                "domain": str((node_cfg or {}).get("domain", "")),
                "capabilities": list((node_cfg or {}).get("capabilities", []) or []),
            },
            "cfg": dict(dm_cfg or {}),
        }
        try:
            with open(self.manifest_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=True, indent=2)
        except Exception:
            pass

    def write_raw_message(self, *, direction: str, topic: str, payload: Dict[str, Any]) -> None:
        if not self.raw_messages_path:
            return
        row = {
            "ts": _now(),
            "direction": str(direction),
            "topic": str(topic),
            "payload": dict(payload or {}),
        }
        try:
            with open(self.raw_messages_path, "a", encoding="utf-8") as f:
                f.write(_json_dumps(row) + "\n")
        except Exception:
            pass


class TraceLogger:
    def __init__(self, path: str = "", data_manager: Optional[LocalNodeDataManager] = None) -> None:
        self.data_manager = data_manager
        default_path = ""
        if data_manager is not None:
            default_path = str(data_manager.trace_jsonl_path or "")
        self.path = str(path or default_path or "").strip()
        self.fp = open(self.path, "a", encoding="utf-8") if self.path else None

    def write(self, event: str, **kw: Any) -> None:
        row = {"ts": _now(), "event": str(event)}
        row.update(kw)
        line = _json_dumps(row)
        print(line)
        if self.fp is not None:
            self.fp.write(line + "\n")
            self.fp.flush()

    def close(self) -> None:
        if self.fp is not None:
            self.fp.close()
            self.fp = None


@dataclass
class TopicRule:
    name: str
    direction: str  # local_to_fed | fed_to_local
    subscribe_topic: str
    publish_topic: str
    event_type: str = "event"


@dataclass
class MonitorRule:
    name: str
    source: str  # local | federation | any
    subscribe_topic: str
    kind: str  # state | event
    state_key: str = ""
    event_name: str = ""
    store_last_payload: bool = True


class StateManager:
    def __init__(self) -> None:
        self._latest: Dict[str, Dict[str, Any]] = {}

    def set(self, key: str, payload: Dict[str, Any]) -> None:
        self._latest[str(key)] = dict(payload or {})

    def get(self, key: str) -> Dict[str, Any]:
        return dict(self._latest.get(str(key), {}) or {})


class EventManager:
    def __init__(self) -> None:
        self.counters: Dict[str, int] = {}

    def inc(self, event_name: str) -> int:
        k = str(event_name)
        n = int(self.counters.get(k, 0)) + 1
        self.counters[k] = n
        return n


class FederationContextManager:
    """
    Per-FNM dynamic relevant-peer set (FCM).
    It discovers peers via federation discovery responses and keeps a local TTL cache.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        dt_id: str,
        dt_type: str,
        gateway_id: str,
        logger: TraceLogger,
        federation_iface: FederationInterface,
        cfg: Dict[str, Any],
    ) -> None:
        self.enabled = bool(enabled)
        self.dt_id = str(dt_id or "")
        self.dt_type = str(dt_type or "")
        self.gateway_id = str(gateway_id or "")
        self.logger = logger
        self.federation_iface = federation_iface
        c = dict(cfg or {})
        self.query_interval_sec = max(0.5, _as_float(c.get("query_interval_sec", 5.0), 5.0))
        self.peer_ttl_sec = max(1.0, _as_float(c.get("peer_ttl_sec", 20.0), 20.0))
        self.fail_open = bool(c.get("fail_open", True))
        self.discovery_query_topic = str(c.get("discovery_query_topic", "federation/discovery/query") or "federation/discovery/query")
        self.discovery_reply_prefix = str(c.get("discovery_reply_prefix", "federation/discovery/resp") or "federation/discovery/resp").rstrip("/")
        self.discovery_event_filter = str(c.get("discovery_event_filter", "state") or "state")
        self.discovery_purpose = str(c.get("discovery_purpose", "fnm_fcm") or "fnm_fcm")
        self.role_filter = str(c.get("role_filter", "TrafficLightSystem") or "TrafficLightSystem")
        self.result_mode = str(c.get("result_mode", "service") or "service").strip().lower()
        self.discovery_node_dedup = bool(
            c.get("discovery_node_dedup", True if self.result_mode == "service" else False)
        )
        self.max_results = max(1, _as_int(c.get("max_results", 50), 50))
        self.require_active_membership = bool(c.get("require_active_membership", True))
        self.discovery_capabilities = _as_str_list(c.get("discovery_capabilities", []))
        self.discovery_service_names = _as_str_list(c.get("discovery_service_names", []))
        self.discovery_directions = _as_str_list(c.get("discovery_directions", []))
        self.discovery_status_filters = [s.upper() for s in _as_str_list(c.get("discovery_status_filters", []))]
        self.context_gate_enabled = bool(c.get("context_gate_enabled", True))
        self.context_back_hops = max(0, _as_int(c.get("context_back_hops", 1), 1))
        self.query_require_valid_context = bool(c.get("query_require_valid_context", True))
        self.query_require_complete_context = bool(
            c.get(
                "query_require_complete_context",
                str(self.dt_type or "").strip().lower() in {"vehicle", "ev", "emergency_vehicle"},
            )
        )
        self.query_min_route_len = max(1, _as_int(c.get("query_min_route_len", 1), 1))
        self.query_skip_log_cooldown_sec = max(0.0, _as_float(c.get("query_skip_log_cooldown_sec", 3.0), 3.0))
        self.query_context_stable_suppress_sec = max(
            0.0,
            _as_float(c.get("query_context_stable_suppress_sec", 2.0), 2.0),
        )
        dt_type_norm = str(self.dt_type or "").strip().lower()
        is_vehicle_type = dt_type_norm in {"vehicle", "ev", "emergency_vehicle"}
        self.query_on_context_change = bool(c.get("query_on_context_change", is_vehicle_type))
        self.query_context_change_min_interval_sec = max(
            0.0,
            _as_float(
                c.get(
                    "query_context_change_min_interval_sec",
                    0.5 if is_vehicle_type else max(2.0, float(self.query_interval_sec) * 0.5),
                ),
                0.5 if is_vehicle_type else max(2.0, float(self.query_interval_sec) * 0.5),
            ),
        )
        self.query_periodic_refresh_sec = max(
            0.0,
            _as_float(c.get("query_periodic_refresh_sec", float(self.query_interval_sec)), float(self.query_interval_sec)),
        )
        self.query_idle_suspend_sec = max(
            0.0,
            _as_float(c.get("query_idle_suspend_sec", 20.0), 20.0),
        )
        self.query_idle_suspend_log_cooldown_sec = max(
            0.0,
            _as_float(c.get("query_idle_suspend_log_cooldown_sec", 10.0), 10.0),
        )
        self.keep_last_valid_context = bool(c.get("keep_last_valid_context", True))
        self.valid_context_hold_sec = max(0.0, _as_float(c.get("valid_context_hold_sec", float(self.peer_ttl_sec)), float(self.peer_ttl_sec)))
        self.hold_peers_without_context = bool(c.get("hold_peers_without_context", True))
        self.hold_peers_max_sec = max(0.0, _as_float(c.get("hold_peers_max_sec", float(self.peer_ttl_sec)), float(self.peer_ttl_sec)))
        self.snapshot_interval_sec = max(0.0, _as_float(c.get("snapshot_interval_sec", 0.0), 0.0))
        self.snapshot_max_peers = max(1, _as_int(c.get("snapshot_max_peers", 50), 50))
        self.peer_hysteresis_min_hold_sec = max(
            0.0,
            _as_float(c.get("peer_hysteresis_min_hold_sec", 5.0), 5.0),
        )
        self.peer_change_window_sec = max(
            0.0,
            _as_float(c.get("peer_change_window_sec", 2.0), 2.0),
        )
        self.peer_change_max_add = max(
            0,
            _as_int(c.get("peer_change_max_add", 4), 4),
        )
        self.min_active_members_before_query = max(
            0,
            _as_int(
                c.get(
                    "min_active_members_before_query",
                    1 if str(self.dt_type or "").strip().lower() in {"vehicle", "ev", "emergency_vehicle"} else 0,
                ),
                0,
            ),
        )
        self.query_startup_grace_sec = max(0.0, _as_float(c.get("query_startup_grace_sec", 8.0), 8.0))
        sel_cfg_raw = c.get("selection_policy", {})
        sel_cfg = dict(sel_cfg_raw) if isinstance(sel_cfg_raw, dict) else {}
        self.selection_policy_name = str(
            c.get("selection_policy_name", sel_cfg.get("name", "default")) or "default"
        ).strip()
        sel_cfg.setdefault("context_gate_enabled", self.context_gate_enabled)
        sel_cfg.setdefault("context_back_hops", self.context_back_hops)
        self.selection_policy = build_peer_selection_policy(self.selection_policy_name, sel_cfg)
        status_raw = c.get("active_member_statuses", "ACTIVE,REGISTERED,ALIVE")
        if isinstance(status_raw, str):
            self.active_member_statuses: Set[str] = {
                s.strip().upper() for s in str(status_raw).split(",") if s.strip()
            }
        else:
            self.active_member_statuses = {
                str(s).strip().upper() for s in _as_list(status_raw) if str(s).strip()
            }
        if not self.active_member_statuses:
            self.active_member_statuses = {"ACTIVE", "REGISTERED", "ALIVE"}
        self.membership_state_topic = str(c.get("membership_state_topic", "federation/membership/state") or "federation/membership/state")
        self.membership_events_topic = str(c.get("membership_events_topic", "federation/membership/events") or "federation/membership/events")
        self.discovery_query_topic_enabled = bool(c.get("query_enabled", True))
        self._last_query_ts = 0.0
        self._pending_query_ts: Dict[str, float] = {}
        self._pending_query_context: Dict[str, Dict[str, Any]] = {}
        self._member_status_by_gateway: Dict[str, str] = {}
        self._member_seen_by_gateway: Dict[str, float] = {}
        self._peers: Dict[str, Dict[str, Any]] = {}
        self._query_context: Dict[str, Any] = {}
        self._query_context_valid = False
        self._last_valid_context: Dict[str, Any] = {}
        self._last_valid_context_ts = 0.0
        self._last_query_skip_log_ts = 0.0
        self._query_skip_suppressed_n = 0
        self._last_context_signature = ""
        self._query_context_signature = ""
        self._last_query_context_signature = ""
        self._context_invalid_since_ts = 0.0
        self._last_idle_suspend_log_ts = 0.0
        self._idle_suspend_suppressed_n = 0
        self._last_snapshot_ts = 0.0
        self._add_window_start_ts = 0.0
        self._add_window_count = 0
        self._start_wall_ts = _now()
        self._query_context_changed = False
        self._last_query_trigger = ""

    @staticmethod
    def _context_is_valid(ctx: Dict[str, Any]) -> bool:
        if not isinstance(ctx, dict):
            return False
        current_tls = str(ctx.get("current_tls", "") or "").strip()
        route_seq = _as_list(ctx.get("route_tls_sequence", []))
        if not route_seq:
            route_seq = _as_list(ctx.get("route_sequence", []))
        next_order = _as_list(ctx.get("next_tls_order", []))
        return bool(current_tls) or bool(route_seq) or bool(next_order)

    def _context_is_query_ready(self, ctx: Dict[str, Any]) -> bool:
        if not self._context_is_valid(ctx):
            return False
        if not bool(self.query_require_complete_context):
            return True
        current_tls = str((ctx or {}).get("current_tls", "") or "").strip()
        route_seq = _as_list((ctx or {}).get("route_tls_sequence", []))
        if not route_seq:
            route_seq = _as_list((ctx or {}).get("route_sequence", []))
        route_seq = [str(x).strip() for x in route_seq if str(x).strip()]
        if not current_tls:
            return False
        return bool(len(route_seq) >= int(self.query_min_route_len))

    @staticmethod
    def _extract_route_tls(value: Any) -> List[str]:
        out: List[str] = []
        for x in _as_list(value):
            s = str(x).strip()
            if s:
                out.append(s)
        if len(out) == 1 and "," in out[0]:
            out = [s.strip() for s in out[0].split(",") if s.strip()]
        return list(dict.fromkeys(out))

    def _should_hold_peers_without_context(self, now_wall: float) -> bool:
        if not bool(self.hold_peers_without_context):
            return False
        if self._context_is_valid(self._query_context):
            return False
        if float(self._last_valid_context_ts) <= 0.0:
            return False
        dt = max(0.0, float(now_wall) - float(self._last_valid_context_ts))
        return dt <= float(self.hold_peers_max_sec)

    def federation_subscriptions(self) -> List[str]:
        if not self.enabled:
            return []
        subs = [
            f"{self.discovery_reply_prefix}/{self.dt_id}",
            self.membership_state_topic,
            self.membership_events_topic,
        ]
        return list(dict.fromkeys([str(x) for x in subs if str(x)]))

    def _is_member_active(self, gateway_id: str) -> bool:
        gw = str(gateway_id or "")
        st = str(self._member_status_by_gateway.get(gw, "") or "").strip().upper()
        return bool(gw) and bool(st) and (st in self.active_member_statuses)

    @staticmethod
    def _capability_names_from_item(item: Dict[str, Any]) -> List[str]:
        names: List[str] = []
        for k in ("capability_names", "capabilities"):
            for x in _as_list(item.get(k, [])):
                if isinstance(x, dict):
                    nm = str(x.get("name", "") or x.get("id", "") or "").strip()
                else:
                    nm = str(x).strip()
                if nm:
                    names.append(nm)
        cap = item.get("capability", None)
        if isinstance(cap, dict):
            nm = str(cap.get("name", "") or cap.get("id", "") or "").strip()
            if nm:
                names.append(nm)
        out: List[str] = []
        seen: Set[str] = set()
        for n in names:
            if n not in seen:
                out.append(n)
                seen.add(n)
        return out

    def _context_candidate_allowed(self, node_id: str, *, context: Optional[Dict[str, Any]] = None) -> bool:
        ctx = dict(context or self._query_context or {})
        try:
            return bool(self.selection_policy.candidate_allowed(str(node_id or ""), ctx))
        except Exception as e:
            self.logger.write(
                "fcm.policy.error",
                dt_id=self.dt_id,
                policy=str(self.selection_policy_name),
                stage="candidate_allowed",
                peer_id=str(node_id or ""),
                err=f"{type(e).__name__}:{e}",
            )
            return True

    @staticmethod
    def _as_float_map(value: Any) -> Dict[str, float]:
        out: Dict[str, float] = {}
        if not isinstance(value, dict):
            return out
        for k, v in value.items():
            kk = str(k).strip()
            if not kk:
                continue
            vv = _as_float(v, -1.0)
            if vv >= 0.0:
                out[kk] = vv
        return out

    def _candidate_rank_key(self, node_id: str, *, context: Optional[Dict[str, Any]] = None) -> Tuple[float, float, str]:
        ctx = dict(context or self._query_context or {})
        try:
            rk = self.selection_policy.candidate_rank_key(str(node_id or ""), ctx)
            if isinstance(rk, tuple) and len(rk) == 3:
                return (float(rk[0]), float(rk[1]), str(rk[2]))
        except Exception as e:
            self.logger.write(
                "fcm.policy.error",
                dt_id=self.dt_id,
                policy=str(self.selection_policy_name),
                stage="candidate_rank_key",
                peer_id=str(node_id or ""),
                err=f"{type(e).__name__}:{e}",
            )
        return (0.0, 0.0, str(node_id or ""))

    def _emit_peer_set_snapshot(
        self,
        *,
        now_wall: float,
        trigger: str,
        request_id: str = "",
        stats: Optional[Dict[str, Any]] = None,
    ) -> None:
        peers_rows: List[Dict[str, Any]] = []
        for peer_id, rec in sorted(self._peers.items(), key=lambda kv: str(kv[0])):
            gw = str(rec.get("gateway_id", "") or "")
            peers_rows.append(
                {
                    "node_id": str(peer_id),
                    "gateway_id": gw,
                    "role": str(rec.get("role", "") or ""),
                    "membership_status": str(self._member_status_by_gateway.get(gw, "") or ""),
                    "capability_names": list(rec.get("capability_names", []) or []),
                    "service_names": list(rec.get("service_names", []) or []),
                    "last_seen_wall": float(rec.get("last_seen_wall", 0.0) or 0.0),
                    "reason": str(rec.get("reason", "") or ""),
                }
            )
        self.logger.write(
            "fcm.peer_set.snapshot",
            dt_id=self.dt_id,
            gateway_id=self.gateway_id,
            trigger=str(trigger),
            request_id=str(request_id or ""),
            query_context=dict(self._query_context or {}),
            peers_n=int(len(peers_rows)),
            peers=peers_rows[: int(self.snapshot_max_peers)],
            stats=dict(stats or {}),
            ts_wall=now_wall,
        )
        self._last_snapshot_ts = float(now_wall)

    def update_query_context(self, context: Dict[str, Any], *, reason: str = "") -> None:
        if not self.enabled:
            return
        if not isinstance(context, dict):
            return
        policy_ctx: Dict[str, Any] = {}
        try:
            policy_ctx = dict(self.selection_policy.normalize_context(dict(context or {})) or {})
        except Exception as e:
            self.logger.write(
                "fcm.policy.error",
                dt_id=self.dt_id,
                policy=str(self.selection_policy_name),
                stage="normalize_context",
                err=f"{type(e).__name__}:{e}",
            )
        ctx = {
            "scenario_id": str(context.get("scenario_id", "") or ""),
            "mode": str(context.get("mode", "") or ""),
            "ev_id": str(context.get("ev_id", "") or ""),
            "route_id": str(context.get("route_id", "") or ""),
            "current_tls": str(context.get("current_tls", "") or ""),
            "current_edge_id": str(context.get("current_edge_id", "") or ""),
            "lookahead_hops": _as_int(context.get("lookahead_hops", 0), 0),
            "eta_window_sec": _as_float(context.get("eta_window_sec", 0.0), 0.0),
            "radius_m": _as_float(context.get("radius_m", 0.0), 0.0),
            "max_candidates": _as_int(context.get("max_candidates", 0), 0),
            "sim_time": _as_float(context.get("sim_time", 0.0), 0.0),
            "ts_wall": _now(),
        }
        ctx.update(policy_ctx)
        route_seq_norm = self._extract_route_tls(
            ctx.get("route_tls_sequence", ctx.get("route_sequence", []))
        )
        if route_seq_norm:
            ctx["route_tls_sequence"] = list(route_seq_norm)
            ctx["route_sequence"] = list(route_seq_norm)
        elif "route_sequence" in ctx and "route_tls_sequence" not in ctx:
            ctx["route_tls_sequence"] = self._extract_route_tls(ctx.get("route_sequence", []))
        next_order_norm = self._extract_route_tls(ctx.get("next_tls_order", []))
        if next_order_norm:
            ctx["next_tls_order"] = list(next_order_norm)
        now_wall = _now()
        raw_valid = self._context_is_valid(ctx)
        raw_query_ready = self._context_is_query_ready(ctx)
        used_fallback = False
        if raw_query_ready:
            self._last_valid_context = dict(ctx)
            self._last_valid_context_ts = float(now_wall)
            self._query_context_valid = True
        else:
            self._query_context_valid = False
            if bool(self.keep_last_valid_context) and self._last_valid_context:
                age = max(0.0, float(now_wall) - float(self._last_valid_context_ts))
                if age <= float(self.valid_context_hold_sec):
                    ctx = dict(self._last_valid_context)
                    ctx["ts_wall"] = float(now_wall)
                    ctx["context_fallback"] = "last_valid"
                    ctx["context_fallback_age_sec"] = float(age)
                    self._query_context_valid = True
                    used_fallback = True
        if bool(self._query_context_valid):
            self._context_invalid_since_ts = 0.0
            self._idle_suspend_suppressed_n = 0
        else:
            if float(self._context_invalid_since_ts) <= 0.0:
                self._context_invalid_since_ts = float(now_wall)
        sig_ctx = dict(ctx)
        sig_ctx.pop("ts_wall", None)
        sig_ctx.pop("context_fallback_age_sec", None)
        sig_norm = json.dumps(sig_ctx, sort_keys=True, separators=(",", ":"))
        self._query_context = ctx
        self._query_context_signature = sig_norm
        if sig_norm != self._last_context_signature:
            self._last_context_signature = sig_norm
            self._query_context_changed = True
            self.logger.write(
                "fcm.query_context.update",
                dt_id=self.dt_id,
                reason=str(reason or ""),
                raw_context_valid=bool(raw_valid),
                raw_query_ready=bool(raw_query_ready),
                context_valid=bool(self._query_context_valid),
                used_fallback=bool(used_fallback),
                context=ctx,
            )

    def _publish_discovery_query(self, now: float, *, trigger: str = "periodic") -> None:
        if not self.enabled or not self.discovery_query_topic_enabled:
            return
        ctx = dict(self._query_context or {})
        ctx_valid = self._context_is_query_ready(ctx)
        if self.min_active_members_before_query > 0:
            active_n = 0
            now_wall = float(now)
            for gw, st in self._member_status_by_gateway.items():
                if str(st or "").strip().upper() in self.active_member_statuses:
                    seen = float(self._member_seen_by_gateway.get(str(gw), now_wall) or now_wall)
                    if (now_wall - seen) <= float(max(self.peer_ttl_sec, self.query_startup_grace_sec)):
                        active_n += 1
            if active_n < int(self.min_active_members_before_query):
                startup_age = max(0.0, now_wall - float(self._start_wall_ts))
                if startup_age <= float(self.query_startup_grace_sec):
                    self.logger.write(
                        "fcm.discovery.query.skip",
                        dt_id=self.dt_id,
                        reason="membership_warmup",
                        active_members=int(active_n),
                        min_active_members_before_query=int(self.min_active_members_before_query),
                        startup_age_sec=float(startup_age),
                        query_startup_grace_sec=float(self.query_startup_grace_sec),
                    )
                    return
        if bool(self.query_require_valid_context) and not bool(ctx_valid):
            now_wall = float(now)
            invalid_dt = (
                max(0.0, now_wall - float(self._context_invalid_since_ts))
                if float(self._context_invalid_since_ts) > 0.0
                else 0.0
            )
            if float(self.query_idle_suspend_sec) > 0.0 and invalid_dt >= float(self.query_idle_suspend_sec):
                cooldown = float(self.query_idle_suspend_log_cooldown_sec)
                should_log = (cooldown <= 0.0) or ((now_wall - float(self._last_idle_suspend_log_ts)) >= cooldown)
                if should_log:
                    self.logger.write(
                        "fcm.discovery.query.skip",
                        dt_id=self.dt_id,
                        reason="context_idle_suspended",
                        query_idle_suspend_sec=float(self.query_idle_suspend_sec),
                        invalid_for_sec=float(invalid_dt),
                        query_idle_suspend_log_cooldown_sec=float(self.query_idle_suspend_log_cooldown_sec),
                        suppressed_since_last=int(self._idle_suspend_suppressed_n),
                        context=ctx,
                    )
                    self._last_idle_suspend_log_ts = now_wall
                    self._idle_suspend_suppressed_n = 0
                else:
                    self._idle_suspend_suppressed_n += 1
                return
            cooldown = float(self.query_skip_log_cooldown_sec)
            should_log = (cooldown <= 0.0) or ((now_wall - float(self._last_query_skip_log_ts)) >= cooldown)
            if should_log:
                self.logger.write(
                    "fcm.discovery.query.skip",
                    dt_id=self.dt_id,
                    reason=("incomplete_context" if bool(self.query_require_complete_context) else "invalid_context"),
                    query_require_valid_context=bool(self.query_require_valid_context),
                    query_require_complete_context=bool(self.query_require_complete_context),
                    query_min_route_len=int(self.query_min_route_len),
                    query_skip_log_cooldown_sec=float(self.query_skip_log_cooldown_sec),
                    suppressed_since_last=int(self._query_skip_suppressed_n),
                    context=ctx,
                )
                self._last_query_skip_log_ts = now_wall
                self._query_skip_suppressed_n = 0
            else:
                self._query_skip_suppressed_n += 1
            return
        # reset suppression counter once context is valid and query flow resumes
        self._query_skip_suppressed_n = 0
        self._idle_suspend_suppressed_n = 0
        ctx_sig = str(self._query_context_signature or "")
        if (
            float(self.query_context_stable_suppress_sec) > 0.0
            and bool(ctx_sig)
            and str(self._last_query_context_signature or "") == ctx_sig
            and (float(now) - float(self._last_query_ts)) < float(self.query_context_stable_suppress_sec)
        ):
            self.logger.write(
                "fcm.discovery.query.skip",
                dt_id=self.dt_id,
                reason="context_unchanged",
                query_context_stable_suppress_sec=float(self.query_context_stable_suppress_sec),
                since_last_query_sec=max(0.0, float(now) - float(self._last_query_ts)),
            )
            return
        req_id = _new_id("disc")
        query_filters: Dict[str, Any] = {
            "event_type": str(self.discovery_event_filter or ""),
            "role": str(self.role_filter or ""),
            "result_mode": str(self.result_mode or "service"),
            "node_dedup": bool(self.discovery_node_dedup),
        }
        if self.discovery_service_names:
            query_filters["service_name"] = list(self.discovery_service_names)
        if self.discovery_directions:
            query_filters["direction"] = list(self.discovery_directions)
        if self.discovery_capabilities:
            query_filters["capability"] = list(self.discovery_capabilities)
            query_filters["capability_names"] = list(self.discovery_capabilities)
        if self.discovery_status_filters:
            query_filters["status"] = list(self.discovery_status_filters)

        max_results = int(self.max_results)
        ctx_max = _as_int(ctx.get("max_candidates", 0), 0)
        has_route_scope = bool(
            _as_list(ctx.get("route_tls_sequence", []))
            or _as_list(ctx.get("route_sequence", []))
            or _as_list(ctx.get("next_tls_order", []))
        )
        if ctx_max > 0 and has_route_scope:
            max_results = min(max_results, int(ctx_max))
        payload = {
            "schema": "federation.discovery.query.v2",
            "event": "query",
            "request_id": str(req_id),
            "requester_id": str(self.dt_id),
            "requester_gateway_id": str(self.gateway_id),
            "requester_node_id": str(self.dt_id),
            "requester_role": str(self.dt_type),
            "reply_topic": f"{self.discovery_reply_prefix}/{self.dt_id}",
            "reply_to": f"{self.discovery_reply_prefix}/{self.dt_id}",
            "query": dict(query_filters),
            "filters": dict(query_filters),
            "purpose": str(self.discovery_purpose),
            "result_mode": str(self.result_mode or "service"),
            "node_dedup": bool(self.discovery_node_dedup),
            "max_results": int(max_results),
            "context": dict(ctx),
            "ts": float(now),
        }
        pub_res = self.federation_iface.publish(self.discovery_query_topic, payload)
        if not bool(pub_res.get("ok", False)):
            self.logger.write(
                "fcm.discovery.query.publish_error",
                dt_id=self.dt_id,
                topic=str(self.discovery_query_topic),
                wire_topic=str(pub_res.get("wire_topic", "")),
                rc=pub_res.get("rc"),
                err=str(pub_res.get("error", "")),
            )
            return
        self._pending_query_ts[str(req_id)] = float(now)
        self._pending_query_context[str(req_id)] = dict(ctx)
        self._last_query_ts = float(now)
        self._last_query_context_signature = str(ctx_sig)
        self._query_context_changed = False
        self._last_query_trigger = str(trigger or "")
        self.logger.write(
            "fcm.discovery.query",
            dt_id=self.dt_id,
            trigger=str(trigger or ""),
            request_id=str(req_id),
            topic=self.discovery_query_topic,
            reply_topic=str(payload.get("reply_topic", "")),
            role_filter=str(self.role_filter or ""),
            event_filter=str(self.discovery_event_filter or ""),
            capability_filter=list(self.discovery_capabilities),
            direction_filter=list(self.discovery_directions),
            service_filter=list(self.discovery_service_names),
            result_mode=str(self.result_mode or "service"),
            node_dedup=bool(self.discovery_node_dedup),
            max_results=int(max_results),
            context=ctx,
        )

    def _update_membership(self, payload: Dict[str, Any]) -> None:
        now_wall = _now()
        updates: List[Tuple[str, str, float]] = []
        members = list(payload.get("members", []) or [])
        if members:
            for m in members:
                if not isinstance(m, dict):
                    continue
                gw = str(m.get("gateway_id", "") or "").strip()
                st = str(m.get("status_effective", m.get("status", "")) or "").strip().upper()
                if not gw or not st:
                    continue
                seen = _as_float(m.get("last_seen_ts", payload.get("ts", now_wall)), now_wall)
                updates.append((gw, st, seen))
        else:
            gw = str(payload.get("gateway_id", "") or "").strip()
            st = str(payload.get("status_effective", payload.get("status", payload.get("member_status", ""))) or "").strip().upper()
            if gw and st:
                seen = _as_float(payload.get("last_seen_ts", payload.get("ts", now_wall)), now_wall)
                updates.append((gw, st, seen))

        for gw, st, seen in updates:
            prev = str(self._member_status_by_gateway.get(gw, "") or "")
            self._member_status_by_gateway[gw] = st
            self._member_seen_by_gateway[gw] = float(seen)
            if prev != st:
                self.logger.write(
                    "fcm.membership.status",
                    dt_id=self.dt_id,
                    gateway_id=gw,
                    prev_status=prev,
                    status=st,
                    active=bool(st in self.active_member_statuses),
                )

    def _ingest_discovery_response(self, topic: str, payload: Dict[str, Any], now: float) -> None:
        req_id = str(payload.get("request_id", "") or "")
        sent_ts = _as_float(self._pending_query_ts.pop(req_id, 0.0), 0.0)
        req_ctx = dict(self._pending_query_context.pop(req_id, {}) or {})
        rt_ms = (1000.0 * (float(now) - float(sent_ts))) if sent_ts > 0.0 else None
        results = list(payload.get("results", []) or [])
        results = sorted(
            results,
            key=lambda it: self._candidate_rank_key(
                str((it or {}).get("node_id", "") or ""),
                context=req_ctx,
            ),
        )
        peer_limit = _as_int(req_ctx.get("max_candidates", 0), 0)
        accepted_n = 0
        rejected_n = 0
        rejected_reasons: Dict[str, int] = {}
        accepted_nodes: Set[str] = set()
        if float(self.peer_change_window_sec) > 0.0:
            if (float(now) - float(self._add_window_start_ts)) >= float(self.peer_change_window_sec):
                self._add_window_start_ts = float(now)
                self._add_window_count = 0
        for item in results:
            if not isinstance(item, dict):
                continue
            node_id = str(item.get("node_id", "") or "")
            gw = str(item.get("gateway_id", "") or "")
            role = str(item.get("role", "") or "")
            svc_name = str(item.get("service_name", "") or "")
            svc_names_item = [str(x).strip() for x in _as_list(item.get("service_names", [])) if str(x).strip()]
            if svc_name and svc_name not in svc_names_item:
                svc_names_item = [svc_name] + svc_names_item
            direction = str(item.get("direction", "") or "")
            capability_names = self._capability_names_from_item(item)
            if not node_id:
                rejected_n += 1
                rejected_reasons["missing_node_id"] = int(rejected_reasons.get("missing_node_id", 0)) + 1
                continue
            if self.role_filter and role and str(role) != str(self.role_filter):
                rejected_n += 1
                rejected_reasons["role_mismatch"] = int(rejected_reasons.get("role_mismatch", 0)) + 1
                continue
            if self.discovery_service_names:
                if svc_names_item:
                    if not any(str(s) in self.discovery_service_names for s in svc_names_item):
                        rejected_n += 1
                        rejected_reasons["service_mismatch"] = int(rejected_reasons.get("service_mismatch", 0)) + 1
                        continue
                elif svc_name and svc_name not in self.discovery_service_names:
                    rejected_n += 1
                    rejected_reasons["service_mismatch"] = int(rejected_reasons.get("service_mismatch", 0)) + 1
                    continue
            if self.discovery_directions and direction and direction not in self.discovery_directions:
                rejected_n += 1
                rejected_reasons["direction_mismatch"] = int(rejected_reasons.get("direction_mismatch", 0)) + 1
                continue
            if self.discovery_capabilities:
                caps_set = set(capability_names)
                if not any(str(x) in caps_set for x in self.discovery_capabilities):
                    rejected_n += 1
                    rejected_reasons["capability_mismatch"] = int(rejected_reasons.get("capability_mismatch", 0)) + 1
                    self.logger.write(
                        "fcm.peer_set.reject",
                        dt_id=self.dt_id,
                        peer_id=node_id,
                        reason="capability_mismatch",
                        gateway_id=gw,
                        role=role,
                        capability_filter=list(self.discovery_capabilities),
                        capability_names=capability_names,
                    )
                    continue
            if not self._context_candidate_allowed(node_id, context=req_ctx):
                rejected_n += 1
                rejected_reasons["context_mismatch"] = int(rejected_reasons.get("context_mismatch", 0)) + 1
                self.logger.write(
                    "fcm.peer_set.reject",
                    dt_id=self.dt_id,
                    peer_id=node_id,
                    reason="context_mismatch",
                    gateway_id=gw,
                    role=role,
                    context=req_ctx,
                )
                continue
            if self.require_active_membership and not self._is_member_active(gw):
                rejected_n += 1
                rejected_reasons["membership_inactive"] = int(rejected_reasons.get("membership_inactive", 0)) + 1
                self.logger.write(
                    "fcm.peer_set.reject",
                    dt_id=self.dt_id,
                    peer_id=node_id,
                    reason="membership_inactive",
                    gateway_id=gw,
                    role=role,
                )
                continue
            if peer_limit > 0 and (node_id not in accepted_nodes) and (len(accepted_nodes) >= peer_limit):
                rejected_n += 1
                rejected_reasons["peer_limit"] = int(rejected_reasons.get("peer_limit", 0)) + 1
                continue
            prev = dict(self._peers.get(node_id, {}) or {})
            if not prev and self.peer_change_max_add > 0 and self._add_window_count >= int(self.peer_change_max_add):
                rejected_n += 1
                rejected_reasons["hysteresis_add_rate_limit"] = int(
                    rejected_reasons.get("hysteresis_add_rate_limit", 0)
                ) + 1
                self.logger.write(
                    "fcm.peer_set.reject",
                    dt_id=self.dt_id,
                    peer_id=node_id,
                    reason="hysteresis_add_rate_limit",
                    gateway_id=gw,
                    role=role,
                    add_window_count=int(self._add_window_count),
                    peer_change_max_add=int(self.peer_change_max_add),
                    peer_change_window_sec=float(self.peer_change_window_sec),
                )
                continue
            prev_caps = list(prev.get("capability_names", []) or [])
            caps_union = list(dict.fromkeys(prev_caps + capability_names))
            prev_svcs = list(prev.get("service_names", []) or [])
            svcs_union = list(dict.fromkeys(prev_svcs + svc_names_item + ([svc_name] if svc_name else [])))
            first_seen = _as_float(prev.get("first_seen_wall", now), now)
            self._peers[node_id] = {
                "gateway_id": gw,
                "role": role,
                "first_seen_wall": float(first_seen),
                "last_seen_wall": float(now),
                "request_id": req_id,
                "service_names": svcs_union,
                "capability_names": caps_union,
                "reason": "discovery_response",
            }
            if not prev:
                self._add_window_count += 1
            accepted_nodes.add(str(node_id))
            accepted_n += 1
            self.logger.write(
                "fcm.peer_set.update",
                dt_id=self.dt_id,
                peer_id=node_id,
                gateway_id=gw,
                role=role,
                reason="discovery_response",
                service_name=svc_name,
                service_names=svcs_union,
                capability_names=caps_union,
            )
        stats = {
            "n_results": int(len(results)),
            "accepted_n": int(accepted_n),
            "rejected_n": int(rejected_n),
            "rejected_reasons": dict(rejected_reasons),
        }
        self.logger.write(
            "fcm.discovery.response",
            dt_id=self.dt_id,
            topic=str(topic),
            request_id=req_id,
            n_results=int(len(results)),
            accepted_n=int(accepted_n),
            rejected_n=int(rejected_n),
            rejected_reasons=dict(rejected_reasons),
            roundtrip_ms=rt_ms,
            context=req_ctx,
        )
        self._emit_peer_set_snapshot(
            now_wall=float(now),
            trigger="discovery_response",
            request_id=req_id,
            stats=stats,
        )

    def on_federation_message(self, topic: str, payload: Dict[str, Any], now: Optional[float] = None) -> None:
        if not self.enabled:
            return
        now_wall = _now() if now is None else float(now)
        t = str(topic or "")
        if t == self.membership_state_topic or t == self.membership_events_topic:
            self._update_membership(payload)
            return
        if t.startswith(f"{self.discovery_reply_prefix}/"):
            self._ingest_discovery_response(t, dict(payload or {}), now_wall)

    def tick(self, now: Optional[float] = None) -> None:
        if not self.enabled:
            return
        now_wall = _now() if now is None else float(now)
        since_last_q = max(0.0, float(now_wall) - float(self._last_query_ts))
        trigger = ""
        should_query = False
        if (
            bool(self.query_on_context_change)
            and bool(self._query_context_changed)
            and since_last_q >= float(self.query_context_change_min_interval_sec)
        ):
            should_query = True
            trigger = "context_change"
        else:
            periodic_sec = float(self.query_periodic_refresh_sec)
            if periodic_sec <= 0.0:
                periodic_sec = float(self.query_interval_sec)
            if since_last_q >= periodic_sec:
                should_query = True
                trigger = "periodic_refresh"
        if should_query:
            self._publish_discovery_query(now_wall, trigger=str(trigger))
        hold_without_ctx = self._should_hold_peers_without_context(float(now_wall))
        expired: List[str] = []
        if not hold_without_ctx:
            for peer_id, rec in list(self._peers.items()):
                last_seen = _as_float(rec.get("last_seen_wall", 0.0), 0.0)
                if last_seen <= 0.0:
                    continue
                if (now_wall - float(last_seen)) > float(self.peer_ttl_sec):
                    first_seen = _as_float(rec.get("first_seen_wall", last_seen), last_seen)
                    held_age = max(0.0, now_wall - float(first_seen))
                    if (
                        float(self.peer_hysteresis_min_hold_sec) > 0.0
                        and held_age < float(self.peer_hysteresis_min_hold_sec)
                    ):
                        self.logger.write(
                            "fcm.peer_set.hold",
                            dt_id=self.dt_id,
                            reason="hysteresis_min_hold",
                            peer_id=str(peer_id),
                            held_age_sec=float(held_age),
                            peer_hysteresis_min_hold_sec=float(self.peer_hysteresis_min_hold_sec),
                        )
                        continue
                    expired.append(str(peer_id))
        else:
            self.logger.write(
                "fcm.peer_set.hold",
                dt_id=self.dt_id,
                reason="context_empty_hold",
                hold_peers_max_sec=float(self.hold_peers_max_sec),
                last_valid_context_age_sec=max(0.0, float(now_wall) - float(self._last_valid_context_ts)),
                peers_n=int(len(self._peers)),
            )
        for peer_id in expired:
            self._peers.pop(str(peer_id), None)
            self.logger.write(
                "fcm.peer_set.expire",
                dt_id=self.dt_id,
                peer_id=str(peer_id),
                reason="ttl",
            )
        if expired:
            self._emit_peer_set_snapshot(
                now_wall=float(now_wall),
                trigger="ttl_expire",
                request_id="",
                stats={"expired_n": int(len(expired))},
            )
        elif self.snapshot_interval_sec > 0.0 and (now_wall - float(self._last_snapshot_ts)) >= float(self.snapshot_interval_sec):
            self._emit_peer_set_snapshot(
                now_wall=float(now_wall),
                trigger="periodic",
                request_id="",
                stats={},
            )

    def peer_allowed(self, peer_id: str, *, now: Optional[float] = None, context: str = "") -> Tuple[bool, str]:
        if not self.enabled:
            return True, "fcm_disabled"
        p = str(peer_id or "")
        if not p:
            return False, "empty_peer"
        now_wall = _now() if now is None else float(now)
        rec = self._peers.get(p)
        if rec is None:
            return (True, "fail_open_not_discovered") if self.fail_open else (False, "not_discovered")
        age = max(0.0, now_wall - _as_float(rec.get("last_seen_wall", 0.0), 0.0))
        if age > float(self.peer_ttl_sec) and not self._should_hold_peers_without_context(float(now_wall)):
            return (True, "fail_open_stale_peer") if self.fail_open else (False, "stale_peer")
        gw = str(rec.get("gateway_id", "") or "")
        if self.require_active_membership and not self._is_member_active(gw):
            return (True, "fail_open_inactive_member") if self.fail_open else (False, "inactive_member")
        return True, "discovered_active"


class SchemaTranslation:
    """Minimal normalization only. No heavy schema policy for now."""

    def local_to_fed(self, payload: Dict[str, Any], *, dt_id: str, dt_type: str, event_type: str) -> Dict[str, Any]:
        out = dict(payload or {})
        out.setdefault("schema", "federation.min.v1")
        out.setdefault("message_id", _new_id("msg"))
        out.setdefault("trace_id", _new_id("trace"))
        out.setdefault("source_dt_id", str(dt_id))
        out.setdefault("source_dt_type", str(dt_type))
        out.setdefault("event_type", str(event_type))
        return out

    def fed_to_local(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return dict(payload or {})


class EVRequestTranslation:
    """
    Vehicle-specific normalization kept outside the generic schema translation layer.
    """

    @staticmethod
    def normalize_ev_state(payload: Dict[str, Any]) -> Dict[str, Any]:
        state = dict(payload or {})
        snap = dict(state.get("snapshot", {}) or state)
        has_x = any(k in snap or k in state for k in ("x", "sumo_x"))
        has_y = any(k in snap or k in state for k in ("y", "sumo_y"))
        route_intersections = list(
            snap.get(
                "route_intersections",
                snap.get(
                    "routeIntersections",
                    state.get(
                        "route_intersections",
                        state.get(
                            "routeIntersections",
                            state.get("route_tls_sequence", state.get("next_tls_order", [])),
                        ),
                    ),
                ),
            )
            or []
        )
        return {
            "ev_id": str(
                snap.get("evId")
                or snap.get("ev_id")
                or state.get("evId")
                or state.get("ev_id")
                or ""
            ),
            "sim_time": _as_float(snap.get("simTime", snap.get("sim_time", state.get("simTime", _now()))), _now()),
            "x": (
                _as_float(
                    snap.get("x", snap.get("sumo_x", state.get("x", state.get("sumo_x", 0.0)))),
                    0.0,
                )
                if has_x
                else None
            ),
            "y": (
                _as_float(
                    snap.get("y", snap.get("sumo_y", state.get("y", state.get("sumo_y", 0.0)))),
                    0.0,
                )
                if has_y
                else None
            ),
            "sumo_x": (
                _as_float(
                    snap.get("sumo_x", snap.get("x", state.get("sumo_x", state.get("x", 0.0)))),
                    0.0,
                )
                if has_x
                else None
            ),
            "sumo_y": (
                _as_float(
                    snap.get("sumo_y", snap.get("y", state.get("sumo_y", state.get("y", 0.0)))),
                    0.0,
                )
                if has_y
                else None
            ),
            "speed_mps": _as_float(snap.get("speedMps", snap.get("speed_mps", 0.0)), 0.0),
            "in_edge_id": str(snap.get("edgeId", snap.get("edge_id", "")) or ""),
            "edge_id": str(snap.get("edgeId", snap.get("edge_id", "")) or ""),
            "lane_id": str(snap.get("laneId", snap.get("lane_id", "")) or ""),
            "exists_in_sim": bool(snap.get("existsInSim", snap.get("exists_in_sim", True))),
            "stopline_dist_m": _as_float(
                snap.get("distToStoplineM", snap.get("dist_to_stopline_m", 1e9)),
                1e9,
            ),
            "next_tls": list(snap.get("nextTls", snap.get("next_tls", [])) or []),
            "route_veh": list(snap.get("routeEdges", snap.get("route_veh", [])) or []),
            "route_intersections": route_intersections,
            "erl_level": _as_int(state.get("erl_level", state.get("erlLevel", 1)), 1),
            "raw": state,
        }

    @staticmethod
    def ev_state_to_requests(
        state: Dict[str, Any],
        *,
        max_next_tls: int,
        source_service: str,
        source_tag: str,
        default_delta_sec: float,
    ) -> List[Tuple[str, Dict[str, Any]]]:
        ev_id = str(state.get("ev_id", "") or "")
        if not ev_id:
            return []
        next_tls = list(state.get("next_tls", []) or [])
        out: List[Tuple[str, Dict[str, Any]]] = []
        for i, item in enumerate(next_tls):
            if i >= int(max_next_tls):
                break
            tls_id = ""
            dist_m = 1e9
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                tls_id = str(item[0] or "")
                dist_m = _as_float(item[1], 1e9)
            elif isinstance(item, dict):
                tls_id = str(item.get("tls_id", item.get("tlsId", "")) or "")
                dist_m = _as_float(
                    item.get("distance_to_intersection_m", item.get("distanceToIntersectionM", 1e9)),
                    1e9,
                )
            if not tls_id:
                continue
            req = {
                "ev_id": ev_id,
                "sim_time": float(state.get("sim_time", _now())),
                "erl_level": int(state.get("erl_level", 1)),
                "speed_mps": float(state.get("speed_mps", 0.0)),
                "distance_to_intersection_m": float(dist_m),
                "in_edge_id": str(state.get("in_edge_id", "") if i == 0 else ""),
                "target_phase_idx": None,
                "delta_sec": float(default_delta_sec),
                "route_intersections": list(state.get("route_intersections", []) or []) or None,
                "route_veh": [str(x) for x in list(state.get("route_veh", []) or [])] or None,
                "source_service": str(source_service),
                "source_tag": str(source_tag),
                "delivery": "mqtt",
                "request_kind": "actuate" if i == 0 else "track",
                "is_primary_tls": bool(i == 0),
            }
            out.append((str(tls_id), req))
        return out


class MQTTInterface:
    def __init__(
        self,
        *,
        name: str,
        host: str,
        port: int,
        topic_namespace: str,
        client_id: str,
        on_message,
        logger: TraceLogger,
    ) -> None:
        self.name = str(name)
        self.host = str(host)
        self.port = int(port)
        self.client_id = str(client_id)
        self.topic_namespace = str(topic_namespace or "").strip().strip("/")
        self._ns_prefix = f"{self.topic_namespace}/" if self.topic_namespace else ""
        self.client = _make_mqtt_client(self.client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass
        self.on_message_cb = on_message
        self.logger = logger
        self.subscriptions: List[str] = []
        self._connected = False
        self._last_connect_rc = ""
        self._connect_evt = threading.Event()
        self._lock = threading.Lock()
        self._publish_error_count = 0

    def _tx_topic(self, topic: str) -> str:
        t = str(topic or "").strip()
        if not t:
            return t
        if not self._ns_prefix:
            return t
        if t.startswith(self._ns_prefix):
            return t
        return f"{self._ns_prefix}{t}"

    def _rx_topic(self, topic: str) -> Tuple[str, bool]:
        t = str(topic or "").strip()
        if not self._ns_prefix:
            return t, True
        if t.startswith(self._ns_prefix):
            return t[len(self._ns_prefix) :], True
        return t, False

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        ok = _mqtt_reason_ok(reason_code)
        with self._lock:
            self._connected = bool(ok)
            self._last_connect_rc = str(reason_code)
            if ok:
                self._connect_evt.set()
            else:
                self._connect_evt.clear()
        self.logger.write(
            "fnm.mqtt.connected",
            iface=self.name,
            host=self.host,
            port=self.port,
            rc=str(reason_code),
            connected=int(ok),
        )
        for t in self.subscriptions:
            try:
                sub_t = self._tx_topic(str(t))
                res = client.subscribe(sub_t, qos=0)
                sub_rc = res[0] if isinstance(res, tuple) and res else getattr(res, "rc", res)
                self.logger.write("fnm.mqtt.subscribed", iface=self.name, topic=str(t), wire_topic=sub_t, rc=sub_rc)
            except Exception as e:
                self.logger.write("fnm.mqtt.subscribe_error", iface=self.name, topic=str(t), err=f"{type(e).__name__}:{e}")

    def _on_disconnect(self, _client, _userdata, *args):
        if len(args) >= 3:
            disconnect_flags, reason_code = args[0], args[1]
        elif len(args) == 1:
            disconnect_flags, reason_code = "", args[0]
        else:
            disconnect_flags, reason_code = "", ""
        with self._lock:
            self._connected = False
            self._last_connect_rc = str(reason_code)
            self._connect_evt.clear()
        self.logger.write(
            "fnm.mqtt.disconnected",
            iface=self.name,
            host=self.host,
            port=self.port,
            rc=str(reason_code),
            disconnect_flags=str(disconnect_flags),
        )

    def _on_connect_fail(self, _client, _userdata):
        with self._lock:
            self._connected = False
            self._last_connect_rc = "connect_fail"
            self._connect_evt.clear()
        self.logger.write(
            "fnm.mqtt.connect_fail",
            iface=self.name,
            host=self.host,
            port=self.port,
            client_id=self.client_id,
            client_id_len=len(self.client_id),
        )

    def _on_message(self, _client, _userdata, msg):
        payload = _json_loads(msg.payload)
        try:
            in_topic = str(msg.topic)
            topic, ok = self._rx_topic(in_topic)
            if not ok:
                self.logger.write(
                    "fnm.mqtt.drop.out_of_namespace",
                    iface=self.name,
                    wire_topic=in_topic,
                    namespace=self.topic_namespace,
                )
                return
            self.on_message_cb(str(topic), payload, wire_topic=in_topic)
        except Exception as e:
            self.logger.write("fnm.mqtt.callback_error", iface=self.name, topic=str(msg.topic), err=f"{type(e).__name__}:{e}")

    def start(self, subscriptions: Iterable[str]) -> None:
        self.subscriptions = [str(t) for t in list(subscriptions or []) if str(t)]
        try:
            self.client.reconnect_delay_set(min_delay=0.5, max_delay=4.0)
        except Exception:
            pass
        rc = self.client.connect(self.host, self.port, keepalive=30)
        self.logger.write(
            "fnm.mqtt.connect_called",
            iface=self.name,
            host=self.host,
            port=self.port,
            rc=int(rc),
            client_id=self.client_id,
            client_id_len=len(self.client_id),
        )
        self.client.loop_start()

    def is_connected(self) -> bool:
        with self._lock:
            return bool(self._connected)

    def wait_connected(self, timeout_sec: float = 2.0) -> bool:
        return bool(self._connect_evt.wait(max(0.0, float(timeout_sec))))

    def wire_topic(self, topic: str) -> str:
        return self._tx_topic(str(topic))

    def publish(self, topic: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        wire_topic = self._tx_topic(str(topic))
        payload_text = _json_dumps(dict(payload or {}))
        payload_size_bytes = int(len(payload_text.encode("utf-8")))
        connected = self.is_connected()
        if not connected:
            with self._lock:
                last_rc = str(self._last_connect_rc)
                self._publish_error_count += 1
                err_count = int(self._publish_error_count)
            if err_count <= 10 or err_count % 100 == 0:
                self.logger.write(
                    "fnm.mqtt.publish_attempt_not_connected",
                    iface=self.name,
                    topic=str(topic),
                    wire_topic=str(wire_topic),
                    payload_size_bytes=int(payload_size_bytes),
                    connected=0,
                    last_connect_rc=last_rc,
                    publish_error_count=err_count,
                    suppressed=int(max(0, err_count - 10)) if err_count > 10 else 0,
                )
            return {
                "ok": False,
                "topic": str(topic),
                "wire_topic": str(wire_topic),
                "rc": int(getattr(mqtt, "MQTT_ERR_NO_CONN", 4)),
                "connected": False,
                "error": "mqtt_not_connected",
            }
        info = self.client.publish(wire_topic, payload_text, qos=0, retain=False)
        rc = int(getattr(info, "rc", 0) or 0)
        ok = rc == mqtt.MQTT_ERR_SUCCESS
        if not ok:
            with self._lock:
                self._publish_error_count += 1
                err_count = int(self._publish_error_count)
        else:
            with self._lock:
                err_count = int(self._publish_error_count)
        self.logger.write(
            "fnm.mqtt.publish",
            iface=self.name,
            topic=str(topic),
            wire_topic=str(wire_topic),
            payload_size_bytes=int(payload_size_bytes),
            rc=int(rc),
            ok=int(ok),
            connected=int(connected),
            publish_error_count=err_count,
        )
        if not ok:
            self.logger.write(
                "fnm.mqtt.publish_error",
                iface=self.name,
                topic=str(topic),
                wire_topic=str(wire_topic),
                rc=int(rc),
                connected=int(self.is_connected()),
                publish_error_count=err_count,
            )
        return {
            "ok": bool(ok),
            "topic": str(topic),
            "wire_topic": str(wire_topic),
            "rc": int(rc),
            "connected": bool(self.is_connected()),
            "error": "" if ok else f"mqtt_rc_{int(rc)}",
            "payload_size_bytes": int(payload_size_bytes),
        }

    def stop(self) -> None:
        try:
            self.client.disconnect()
        except Exception:
            pass
        try:
            self.client.loop_stop()
        except Exception:
            pass


class BaseProtocolAdapter:
    def tick(self, now: float) -> None:
        return

    def stop(self) -> None:
        return


class EVHttpStatePullAdapter(BaseProtocolAdapter):
    """HTTP pull for EV DT state, translated into canonical EVRequest MQTT messages."""

    def __init__(
        self,
        *,
        enabled: bool,
        state_pull_url: str,
        state_pull_sec: float,
        state_pull_timeout_sec: float,
        max_next_tls: int,
        default_delta_sec: float,
        source_service: str,
        source_tag: str,
        ev_request_topic_prefix: str,
        request_dedupe_enabled: bool = True,
        request_min_sim_interval_sec: float = 0.0,
        request_dedupe_distance_epsilon_m: float = 0.25,
        request_sim_time_bucket_sec: float = 0.0,
        request_bucket_dedupe_enabled: bool = True,
        ev_translation: EVRequestTranslation,
        state_manager: StateManager,
        event_manager: EventManager,
        fed_iface: MQTTInterface,
        dt_id: str,
        gateway_id: str = "",
        logger: TraceLogger,
        emit_state_trace: bool = False,
        error_backoff_max_sec: float = 2.0,
        peer_allow_fn: Optional[Callable[[str, float, str], Tuple[bool, str]]] = None,
        peer_context_update_fn: Optional[Callable[[Dict[str, Any], float], None]] = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.state_pull_url = str(state_pull_url or "")
        self.state_pull_sec = max(0.05, float(state_pull_sec))
        self.state_pull_timeout_sec = max(0.05, float(state_pull_timeout_sec))
        self.max_next_tls = max(1, int(max_next_tls))
        self.default_delta_sec = float(default_delta_sec)
        self.source_service = str(source_service or "fnm.ev.adapter")
        self.source_tag = str(source_tag or "fnm")
        self.ev_request_topic_prefix = str(ev_request_topic_prefix or "federation/ev/request").rstrip("/")
        self.request_dedupe_enabled = bool(request_dedupe_enabled)
        self.request_min_sim_interval_sec = max(0.0, float(request_min_sim_interval_sec))
        self.request_dedupe_distance_epsilon_m = max(0.0, float(request_dedupe_distance_epsilon_m))
        self.request_sim_time_bucket_sec = max(0.0, float(request_sim_time_bucket_sec))
        self.request_bucket_dedupe_enabled = bool(request_bucket_dedupe_enabled)
        self.ev_translation = ev_translation
        self.state_manager = state_manager
        self.event_manager = event_manager
        self.fed_iface = fed_iface
        self.dt_id = str(dt_id)
        self.gateway_id = str(gateway_id or dt_id)
        self.logger = logger
        self._next_pull_ts = 0.0
        self.emit_state_trace = bool(emit_state_trace)
        self.error_backoff_max_sec = max(0.2, float(error_backoff_max_sec))
        self._err_streak = 0
        self.peer_allow_fn = peer_allow_fn
        self.peer_context_update_fn = peer_context_update_fn
        self._last_request_signature_by_tls: Dict[str, Tuple[float, str, str, float]] = {}
        self._last_request_publish_sim_by_tls: Dict[str, float] = {}
        self._last_request_bucket_signature_by_tls: Dict[str, Tuple[int, str, str, int]] = {}
        self._last_request_publish_bucket_by_tls: Dict[str, int] = {}

    def _request_sim_bucket(self, sim_time: float) -> Optional[int]:
        bucket_sec = float(self.request_sim_time_bucket_sec)
        if bucket_sec <= 0.0:
            return None
        return int((float(sim_time) + 1e-9) / bucket_sec)

    def _request_distance_bucket(self, distance_m: float) -> int:
        eps = max(1e-9, float(self.request_dedupe_distance_epsilon_m))
        return int(round(float(distance_m) / eps))

    def _request_snapshot_hash(
        self,
        *,
        ev_id: str,
        tls_id: str,
        sim_bucket: Optional[int],
        req_sim_time: float,
        req_dist_m: float,
        req_edge: str,
        req_kind: str,
    ) -> str:
        payload = {
            "dt_id": str(self.dt_id),
            "ev_id": str(ev_id),
            "tls_id": str(tls_id),
            "sim_bucket": sim_bucket,
            "sim_time_bucket_sec": float(self.request_sim_time_bucket_sec),
            "sim_time": round(float(req_sim_time), 3),
            "distance_bucket": self._request_distance_bucket(float(req_dist_m)),
            "in_edge_id": str(req_edge),
            "request_kind": str(req_kind),
        }
        raw = json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]

    @staticmethod
    def _extract_state_age_ms(raw_payload: Dict[str, Any], now_wall: float) -> Optional[float]:
        try:
            age_direct = _as_float(raw_payload.get("age_ms", -1.0), -1.0)
            if age_direct >= 0.0:
                return float(age_direct)
        except Exception:
            pass
        snap = dict(raw_payload.get("snapshot", {}) or {})
        for k in ("ts", "timestamp", "wall_ts", "wall_time", "event_ts"):
            v = _as_float(snap.get(k, raw_payload.get(k, 0.0)), 0.0)
            if v > 1e9:
                return max(0.0, 1000.0 * (float(now_wall) - float(v)))
        return None

    def tick(self, now: float) -> None:
        if not self.enabled or not self.state_pull_url:
            return
        if float(now) < float(self._next_pull_ts):
            return
        self._next_pull_ts = float(now) + float(self.state_pull_sec)

        t_pull_start = _now()
        req = url_request.Request(self.state_pull_url, method="GET")
        try:
            with url_request.urlopen(req, timeout=float(self.state_pull_timeout_sec)) as resp:
                raw = resp.read()
            t_http_done = _now()
            payload = dict(json.loads(raw.decode("utf-8")))
            t_decode_done = _now()
        except url_error.URLError as e:
            self._err_streak += 1
            backoff = min(
                float(self.error_backoff_max_sec),
                float(self.state_pull_sec) * (2 ** min(int(self._err_streak), 4)),
            )
            self._next_pull_ts = float(now) + float(backoff)
            self.logger.write(
                "fnm.adapter.state_pull.error",
                dt_id=self.dt_id,
                url=self.state_pull_url,
                err=f"{type(e).__name__}:{e}",
                pull_elapsed_ms=(1000.0 * (_now() - t_pull_start)),
            )
            return
        except Exception as e:
            self._err_streak += 1
            backoff = min(
                float(self.error_backoff_max_sec),
                float(self.state_pull_sec) * (2 ** min(int(self._err_streak), 4)),
            )
            self._next_pull_ts = float(now) + float(backoff)
            self.logger.write(
                "fnm.adapter.state_pull.error",
                dt_id=self.dt_id,
                url=self.state_pull_url,
                err=f"{type(e).__name__}:{e}",
                pull_elapsed_ms=(1000.0 * (_now() - t_pull_start)),
            )
            return

        self._err_streak = 0
        t_norm_start = _now()
        state = self.ev_translation.normalize_ev_state(payload)
        t_norm_done = _now()
        ev_id = str(state.get("ev_id", "") or self.dt_id)
        state_age_ms = self._extract_state_age_ms(payload, float(t_norm_done))
        self.state_manager.set(f"ev_state:{ev_id}", state)
        if self.peer_context_update_fn is not None:
            try:
                self.peer_context_update_fn(dict(state or {}), float(t_norm_done))
            except Exception as e:
                self.logger.write(
                    "fcm.query_context.update.error",
                    dt_id=self.dt_id,
                    err=f"{type(e).__name__}:{e}",
                )
        self.event_manager.inc("state_pull_ok")
        if self.emit_state_trace:
            self.logger.write(
                "fnm.adapter.state_pull.state",
                dt_id=self.dt_id,
                ev_id=ev_id,
                sim_time=float(state.get("sim_time", 0.0)),
                in_edge_id=str(state.get("in_edge_id", "") or ""),
                speed_mps=float(state.get("speed_mps", 0.0)),
                stopline_dist_m=float(state.get("stopline_dist_m", 1e9)),
                next_tls_n=len(list(state.get("next_tls", []) or [])),
            )

        state_pub_topic = f"federation/v1/state/ev/{ev_id}"
        state_pub_msg = dict(state or {})
        state_pub_msg.update(
            {
                "schema": "federation.min.v1",
                "event_type": "EmergencyVehicleState",
                "source_dt_id": str(ev_id),
                "source_dt_type": "EmergencyVehicle",
                "dt_id": str(ev_id),
                "role": "EmergencyVehicle",
                "topic_namespace": str(getattr(self.fed_iface, "topic_namespace", "") or ""),
                "message_id": _new_id("msg"),
                "fnm_meta": {
                    "origin_dt_id": self.dt_id,
                    "origin_gateway_id": self.gateway_id,
                    "origin_topic": str(self.state_pull_url),
                    "origin_rule": "ev_http_state_pull",
                    "local_ingest_ts": float(t_pull_start),
                    "schema_done_ts": float(t_norm_done),
                    "fed_publish_ts": _now(),
                },
            }
        )
        try:
            state_pub_res = self.fed_iface.publish(state_pub_topic, state_pub_msg)
            self.logger.write(
                "fnm.delivery.adapter_ev_state_to_fed",
                artefact_kind="state",
                status=("ok" if bool(state_pub_res.get("ok", False)) else "publish_error"),
                dt_id=self.dt_id,
                ev_id=ev_id,
                topic=state_pub_topic,
                wire_topic=str(state_pub_res.get("wire_topic", state_pub_topic)),
                sim_time=float(state.get("sim_time", 0.0)),
                state_age_ms=state_age_ms,
                rc=state_pub_res.get("rc"),
                err=str(state_pub_res.get("error", "")),
            )
        except Exception as e:
            self.logger.write(
                "fnm.delivery.adapter_ev_state_to_fed",
                artefact_kind="state",
                status="publish_error",
                dt_id=self.dt_id,
                ev_id=ev_id,
                topic=state_pub_topic,
                sim_time=float(state.get("sim_time", 0.0)),
                state_age_ms=state_age_ms,
                err=f"{type(e).__name__}:{e}",
            )

        t_build_start = _now()
        reqs = self.ev_translation.ev_state_to_requests(
            state,
            max_next_tls=self.max_next_tls,
            source_service=self.source_service,
            source_tag=self.source_tag,
            default_delta_sec=self.default_delta_sec,
        )
        t_build_done = _now()
        pub_n = 0
        pub_err_n = 0
        publish_total_ms = 0.0
        for tls_id, req_msg in reqs:
            allow_reason = "filter_inactive"
            if self.peer_allow_fn is not None:
                allow_ok, allow_reason = self.peer_allow_fn(str(tls_id), float(now), "adapter_ev_request")
                if not allow_ok:
                    self.event_manager.inc("delivery_local_to_fed:request_response:drop_peer_filter")
                    self.logger.write(
                        "fnm.adapter.ev_request.drop",
                        dt_id=self.dt_id,
                        ev_id=ev_id,
                        tls_id=str(tls_id),
                        reason=str(allow_reason),
                    )
                    continue
            req_msg = dict(req_msg or {})
            req_sim_time = _as_float(req_msg.get("sim_time", 0.0), 0.0)
            req_dist_m = _as_float(req_msg.get("distance_to_intersection_m", 1e9), 1e9)
            req_edge = str(req_msg.get("in_edge_id", "") or "")
            req_kind = str(req_msg.get("request_kind", "") or "")
            req_sim_bucket = self._request_sim_bucket(float(req_sim_time))
            req_dist_bucket = self._request_distance_bucket(float(req_dist_m))
            req_bucket_mode = bool(self.request_bucket_dedupe_enabled and req_sim_bucket is not None)
            req_snapshot_hash = self._request_snapshot_hash(
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_bucket=req_sim_bucket,
                req_sim_time=float(req_sim_time),
                req_dist_m=float(req_dist_m),
                req_edge=str(req_edge),
                req_kind=str(req_kind),
            )
            if self.request_dedupe_enabled:
                if req_bucket_mode:
                    bucket_sig = (int(req_sim_bucket), str(req_edge), str(req_kind), int(req_dist_bucket))
                    last_bucket_sig = self._last_request_bucket_signature_by_tls.get(str(tls_id))
                    if last_bucket_sig == bucket_sig:
                        self.event_manager.inc("delivery_local_to_fed:request_response:drop_sim_bucket_duplicate")
                        self.logger.write(
                            "fnm.adapter.ev_request.drop",
                            dt_id=self.dt_id,
                            ev_id=ev_id,
                            tls_id=str(tls_id),
                            reason="sim_time_bucket_duplicate",
                            sim_time=float(req_sim_time),
                            sim_time_bucket=int(req_sim_bucket),
                            sim_time_bucket_sec=float(self.request_sim_time_bucket_sec),
                            state_snapshot_hash=str(req_snapshot_hash),
                            distance_to_intersection_m=float(req_dist_m),
                            distance_bucket=int(req_dist_bucket),
                            in_edge_id=str(req_edge),
                            request_kind=str(req_kind),
                        )
                        continue
                    last_pub_bucket = self._last_request_publish_bucket_by_tls.get(str(tls_id))
                    if last_pub_bucket is not None and int(req_sim_bucket) <= int(last_pub_bucket):
                        self.event_manager.inc("delivery_local_to_fed:request_response:drop_sim_bucket_pacing")
                        self.logger.write(
                            "fnm.adapter.ev_request.drop",
                            dt_id=self.dt_id,
                            ev_id=ev_id,
                            tls_id=str(tls_id),
                            reason="sim_time_bucket_pacing",
                            sim_time=float(req_sim_time),
                            sim_time_bucket=int(req_sim_bucket),
                            last_publish_sim_time_bucket=int(last_pub_bucket),
                            sim_time_bucket_sec=float(self.request_sim_time_bucket_sec),
                            state_snapshot_hash=str(req_snapshot_hash),
                            distance_to_intersection_m=float(req_dist_m),
                            distance_bucket=int(req_dist_bucket),
                            in_edge_id=str(req_edge),
                            request_kind=str(req_kind),
                        )
                        continue
                else:
                    last_sig = self._last_request_signature_by_tls.get(str(tls_id))
                    if last_sig is not None:
                        last_sim, last_edge, last_kind, last_dist = last_sig
                        same_sim = abs(float(req_sim_time) - float(last_sim)) <= 1e-9
                        same_edge = str(req_edge) == str(last_edge)
                        same_kind = str(req_kind) == str(last_kind)
                        same_dist = abs(float(req_dist_m) - float(last_dist)) <= float(self.request_dedupe_distance_epsilon_m)
                        if same_sim and same_edge and same_kind and same_dist:
                            self.event_manager.inc("delivery_local_to_fed:request_response:drop_duplicate_state")
                            self.logger.write(
                                "fnm.adapter.ev_request.drop",
                                dt_id=self.dt_id,
                                ev_id=ev_id,
                                tls_id=str(tls_id),
                                reason="duplicate_state_snapshot",
                                sim_time=float(req_sim_time),
                                distance_to_intersection_m=float(req_dist_m),
                                in_edge_id=str(req_edge),
                                request_kind=str(req_kind),
                            )
                            continue
                    last_pub_sim = self._last_request_publish_sim_by_tls.get(str(tls_id))
                    if (
                        last_pub_sim is not None
                        and self.request_min_sim_interval_sec > 0.0
                        and (float(req_sim_time) - float(last_pub_sim)) < float(self.request_min_sim_interval_sec) - 1e-9
                    ):
                        self.event_manager.inc("delivery_local_to_fed:request_response:drop_sim_pacing")
                        self.logger.write(
                            "fnm.adapter.ev_request.drop",
                            dt_id=self.dt_id,
                            ev_id=ev_id,
                            tls_id=str(tls_id),
                            reason="sim_time_pacing",
                            sim_time=float(req_sim_time),
                            last_publish_sim_time=float(last_pub_sim),
                            min_sim_interval_sec=float(self.request_min_sim_interval_sec),
                            distance_to_intersection_m=float(req_dist_m),
                            in_edge_id=str(req_edge),
                            request_kind=str(req_kind),
                        )
                        continue
            if req_sim_bucket is not None:
                msg_id = str(
                    req_msg.get("message_id", "")
                    or f"fnmreq-{self.dt_id}-{tls_id}-b{int(req_sim_bucket)}-{req_kind}-{req_snapshot_hash[:8]}"
                )
            else:
                msg_id = str(req_msg.get("message_id", "") or _new_id("msg"))
            req_msg["message_id"] = msg_id
            meta_in = req_msg.get("fnm_meta", {})
            meta = dict(meta_in if isinstance(meta_in, dict) else {})
            meta.setdefault("origin_dt_id", self.dt_id)
            meta.setdefault("origin_dt_type", self.source_tag or "vehicle")
            meta.setdefault("origin_topic", self.state_pull_url)
            meta.setdefault("origin_rule", "ev_http_state_pull")
            meta.setdefault("local_ingest_ts", float(t_pull_start))
            meta.setdefault("schema_done_ts", float(t_build_done))
            if req_sim_bucket is not None:
                meta.setdefault("request_cadence_mode", "sim_time_bucket")
                meta.setdefault("sim_time_bucket", int(req_sim_bucket))
                meta.setdefault("sim_time_bucket_sec", float(self.request_sim_time_bucket_sec))
                meta.setdefault("state_snapshot_hash", str(req_snapshot_hash))
                meta.setdefault("distance_bucket", int(req_dist_bucket))
            req_msg["fnm_meta"] = meta
            topic = f"{self.ev_request_topic_prefix}/{tls_id}"
            wire_topic = self.fed_iface.wire_topic(topic)
            publish_ok = False
            publish_rc: Any = None
            publish_err = ""
            t_pub_start = _now()
            try:
                pub_ts = _now()
                meta["fed_publish_ts"] = float(pub_ts)
                req_msg["fnm_meta"] = meta
                pub_res = self.fed_iface.publish(topic, req_msg)
                wire_topic = str(pub_res.get("wire_topic", wire_topic))
                publish_rc = pub_res.get("rc")
                if not bool(pub_res.get("ok", False)):
                    publish_err = str(pub_res.get("error", ""))
                    raise RuntimeError(
                        f"mqtt_publish_failed rc={pub_res.get('rc')} "
                        f"connected={int(bool(pub_res.get('connected', False)))} "
                        f"error={pub_res.get('error')}"
                    )
                publish_ok = True
                pub_n += 1
                self._last_request_signature_by_tls[str(tls_id)] = (
                    float(req_sim_time),
                    str(req_edge),
                    str(req_kind),
                    float(req_dist_m),
                )
                self._last_request_publish_sim_by_tls[str(tls_id)] = float(req_sim_time)
                if req_sim_bucket is not None:
                    self._last_request_bucket_signature_by_tls[str(tls_id)] = (
                        int(req_sim_bucket),
                        str(req_edge),
                        str(req_kind),
                        int(req_dist_bucket),
                    )
                    self._last_request_publish_bucket_by_tls[str(tls_id)] = int(req_sim_bucket)
                self.event_manager.inc("local_to_fed")
                self.event_manager.inc("delivery_local_to_fed:request_response:ok")
                self.logger.write(
                    "fnm.delivery.adapter_state_to_fed",
                    artefact_kind="request_response",
                    status="ok",
                    dt_id=self.dt_id,
                    ev_id=ev_id,
                    tls_id=str(tls_id),
                    topic=str(topic),
                    wire_topic=str(wire_topic),
                    message_id=msg_id,
                    state_sim_time=float(state.get("sim_time", 0.0)),
                    state_age_ms=state_age_ms,
                    sim_time_bucket=(int(req_sim_bucket) if req_sim_bucket is not None else None),
                    sim_time_bucket_sec=float(self.request_sim_time_bucket_sec),
                    state_snapshot_hash=str(req_snapshot_hash),
                )
            except Exception as e:
                pub_err_n += 1
                self.event_manager.inc("delivery_local_to_fed:request_response:error")
                self.logger.write(
                    "fnm.delivery.adapter_state_to_fed",
                    artefact_kind="request_response",
                    status="publish_error",
                    dt_id=self.dt_id,
                    ev_id=ev_id,
                    tls_id=str(tls_id),
                    topic=str(topic),
                    wire_topic=str(wire_topic),
                    message_id=msg_id,
                    err=f"{type(e).__name__}:{e}",
                )
                continue
            finally:
                t_pub_done = _now()
                pub_ms = 1000.0 * (t_pub_done - t_pub_start)
                publish_total_ms += pub_ms
                self.logger.write(
                    "fnm.stage.adapter_request_publish",
                    dt_id=self.dt_id,
                    ev_id=ev_id,
                    tls_id=str(tls_id),
                    message_id=msg_id,
                    artefact_kind="request_response",
                    topic=str(topic),
                    wire_topic=str(wire_topic),
                    status=("ok" if publish_ok else "publish_error"),
                    rc=publish_rc,
                    sim_time_bucket=(int(req_sim_bucket) if req_sim_bucket is not None else None),
                    sim_time_bucket_sec=float(self.request_sim_time_bucket_sec),
                    state_snapshot_hash=str(req_snapshot_hash),
                    publish_ms=pub_ms,
                    local_ingest_to_schema_ms=(1000.0 * (float(t_build_done) - float(t_pull_start))),
                    schema_to_fed_publish_ms=(1000.0 * (float(t_pub_start) - float(t_build_done))),
                    local_to_fed_total_ms=(1000.0 * (float(t_pub_done) - float(t_pull_start))),
                )
                event_name = "fnm.adapter.ev_request.publish" if publish_ok else "fnm.adapter.ev_request.publish_error"
                self.logger.write(
                    event_name,
                    dt_id=self.dt_id,
                    ev_id=ev_id,
                    tls_id=str(tls_id),
                    topic=str(topic),
                    wire_topic=str(wire_topic),
                    req_kind=str(req_msg.get("request_kind", "")),
                    distance_to_intersection_m=float(req_msg.get("distance_to_intersection_m", 1e9)),
                    sim_time=float(req_msg.get("sim_time", 0.0)),
                    sim_time_bucket=(int(req_sim_bucket) if req_sim_bucket is not None else None),
                    sim_time_bucket_sec=float(self.request_sim_time_bucket_sec),
                    state_snapshot_hash=str(req_snapshot_hash),
                    request_cadence_mode=("sim_time_bucket" if req_sim_bucket is not None else "raw_sim_time"),
                    state_age_ms=state_age_ms,
                    peer_filter_reason=str(allow_reason or ""),
                    message_id=msg_id,
                    publish_ms=pub_ms,
                    rc=publish_rc,
                    err=str(publish_err),
                )

        t_done = _now()
        self.logger.write(
            "fnm.stage.adapter_state_pull",
            dt_id=self.dt_id,
            ev_id=ev_id,
            artefact_kind="state",
            state_age_ms=state_age_ms,
            http_get_ms=(1000.0 * (float(t_http_done) - float(t_pull_start))),
            decode_ms=(1000.0 * (float(t_decode_done) - float(t_http_done))),
            normalize_ms=(1000.0 * (float(t_norm_done) - float(t_norm_start))),
            request_build_ms=(1000.0 * (float(t_build_done) - float(t_build_start))),
            request_publish_total_ms=float(publish_total_ms),
            pull_to_requests_total_ms=(1000.0 * (float(t_done) - float(t_pull_start))),
            req_published=int(pub_n),
            req_publish_error=int(pub_err_n),
        )
        self.logger.write(
            "fnm.adapter.state_pull.ok",
            dt_id=self.dt_id,
            ev_id=ev_id,
            req_published=pub_n,
            req_publish_error=pub_err_n,
            nearest_tls=(reqs[0][0] if reqs else ""),
            state_pull_sec=self.state_pull_sec,
            state_age_ms=state_age_ms,
            pull_to_requests_total_ms=(1000.0 * (float(t_done) - float(t_pull_start))),
        )


class ProtocolAdaptationManager:
    """Orchestrates one or more protocol adapters selected by configuration."""

    def __init__(self, adapters: Optional[List[BaseProtocolAdapter]] = None) -> None:
        self.adapters: List[BaseProtocolAdapter] = list(adapters or [])

    def tick(self, now: float) -> None:
        for adp in self.adapters:
            adp.tick(now)

    def stop(self) -> None:
        for adp in self.adapters:
            try:
                adp.stop()
            except Exception:
                pass


class IntraTwinInterface:
    def __init__(self, endpoint: MQTTInterface) -> None:
        self.endpoint = endpoint

    def start(self, topics: Iterable[str]) -> None:
        self.endpoint.start(topics)

    def is_connected(self) -> bool:
        return self.endpoint.is_connected()

    def wait_connected(self, timeout_sec: float = 2.0) -> bool:
        return self.endpoint.wait_connected(timeout_sec)

    def wire_topic(self, topic: str) -> str:
        return self.endpoint.wire_topic(topic)

    def publish(self, topic: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.endpoint.publish(topic, payload)

    def stop(self) -> None:
        self.endpoint.stop()


class FederationInterface:
    def __init__(self, endpoint: MQTTInterface) -> None:
        self.endpoint = endpoint

    def start(self, topics: Iterable[str]) -> None:
        self.endpoint.start(topics)

    def is_connected(self) -> bool:
        return self.endpoint.is_connected()

    def wait_connected(self, timeout_sec: float = 2.0) -> bool:
        return self.endpoint.wait_connected(timeout_sec)

    def wire_topic(self, topic: str) -> str:
        return self.endpoint.wire_topic(topic)

    def publish(self, topic: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.endpoint.publish(topic, payload)

    def stop(self) -> None:
        self.endpoint.stop()


class FederationNodeCore:
    def __init__(self, cfg: Dict[str, Any], *, log_jsonl: str = "") -> None:
        self.cfg = dict(cfg or {})
        self.dt_id = str(_deep_get(self.cfg, ["node", "dt_id"], "dt-unknown"))
        self.dt_type = str(_deep_get(self.cfg, ["node", "dt_type"], "other"))
        self.gateway_id = str(_deep_get(self.cfg, ["node", "gateway_id"], f"gw-{self.dt_type}-{self.dt_id}"))
        node_cfg = dict(_deep_get(self.cfg, ["node"], {}) or {})
        self.data_manager = LocalNodeDataManager(
            node_cfg=node_cfg,
            dt_id=self.dt_id,
            gateway_id=self.gateway_id,
            explicit_trace_jsonl=str(log_jsonl or ""),
        )
        self.logger = TraceLogger(str(self.data_manager.trace_jsonl_path or ""), data_manager=self.data_manager)
        src_file = str(__file__)
        src_hash = _file_sha256_short(src_file, n=12)
        src_has_fcm = 0
        src_has_fcm_event = 0
        src_has_fcm_query = 0
        try:
            txt = open(src_file, "r", encoding="utf-8", errors="replace").read()
            src_has_fcm = int("class FederationContextManager" in txt)
            src_has_fcm_event = int("fcm.config.loaded" in txt)
            src_has_fcm_query = int("fcm.discovery.query" in txt)
        except Exception:
            pass
        self.logger.write(
            "fnm.runtime.build",
            dt_id=self.dt_id,
            gateway_id=self.gateway_id,
            source_file=src_file,
            source_sha256_12=src_hash,
            has_fcm_class=src_has_fcm,
            has_fcm_config_event=src_has_fcm_event,
            has_fcm_query_event=src_has_fcm_query,
        )
        self.state_manager = StateManager()
        self.event_manager = EventManager()
        self.translator = SchemaTranslation()
        self.ev_translation = EVRequestTranslation()
        self.stop_evt = threading.Event()

        self.rules = self._load_rules()
        self.monitor_rules = self._load_monitor_rules()
        self._scenario_namespace_by_request_id: Dict[str, str] = {}
        self.topic_namespace = str(
            _deep_get(self.cfg, ["node", "communication", "topic_namespace"], "")
            or _deep_get(self.cfg, ["node", "topic_namespace"], "")
            or ""
        ).strip().strip("/")

        in_host = str(_deep_get(self.cfg, ["node", "communication", "internal_event_bus", "broker", "host"], "127.0.0.1"))
        in_port = int(_deep_get(self.cfg, ["node", "communication", "internal_event_bus", "broker", "port"], 1883))
        fed_host = str(_deep_get(self.cfg, ["node", "communication", "federation_event_bus", "broker", "host"], "127.0.0.1"))
        fed_port = int(_deep_get(self.cfg, ["node", "communication", "federation_event_bus", "broker", "port"], 1883))

        # Avoid client-id collisions across stale/overlapping runs without
        # exceeding conservative broker client-id limits. The broker-visible
        # id is intentionally short/alphanumeric; gateway identity remains in
        # payloads and logs.
        mqtt_client_seed = f"{self.gateway_id}:{os.getpid()}:{uuid.uuid4().hex}"
        self._mqtt_client_suffix = hashlib.sha1(mqtt_client_seed.encode("utf-8")).hexdigest()[:12]

        self._inbound_ep = MQTTInterface(
            name="intra",
            host=in_host,
            port=in_port,
            topic_namespace=self.topic_namespace,
            client_id=f"fi{self._mqtt_client_suffix}",
            on_message=self._on_local_message,
            logger=self.logger,
        )
        self._feder_ep = MQTTInterface(
            name="federation",
            host=fed_host,
            port=fed_port,
            topic_namespace=self.topic_namespace,
            client_id=f"ff{self._mqtt_client_suffix}",
            on_message=self._on_federation_message,
            logger=self.logger,
        )
        self.intra = IntraTwinInterface(self._inbound_ep)
        self.federation = FederationInterface(self._feder_ep)

        adapter_cfg = dict(_deep_get(self.cfg, ["node", "protocol_adaptation", "http_state_pull"], {}) or {})
        adapters: List[BaseProtocolAdapter] = []
        if bool(adapter_cfg.get("enabled", False)):
            adapters.append(
                EVHttpStatePullAdapter(
                    enabled=bool(adapter_cfg.get("enabled", False)),
                    state_pull_url=_expand_topic(str(adapter_cfg.get("url", "")), self.dt_id),
                    state_pull_sec=_as_float(adapter_cfg.get("period_sec", 1.0), 1.0),
                    state_pull_timeout_sec=_as_float(adapter_cfg.get("timeout_sec", 0.8), 0.8),
                    max_next_tls=_as_int(adapter_cfg.get("max_next_tls", 1), 1),
                    default_delta_sec=_as_float(adapter_cfg.get("default_delta_sec", 2.0), 2.0),
                    source_service=str(adapter_cfg.get("source_service", "fnm.ev.adapter")),
                    source_tag=str(adapter_cfg.get("source_tag", "fnm")),
                    ev_request_topic_prefix=str(adapter_cfg.get("ev_request_topic_prefix", "federation/ev/request")),
                    request_dedupe_enabled=bool(adapter_cfg.get("request_dedupe_enabled", True)),
                    request_min_sim_interval_sec=_as_float(adapter_cfg.get("request_min_sim_interval_sec", 0.0), 0.0),
                    request_dedupe_distance_epsilon_m=_as_float(
                        adapter_cfg.get("request_dedupe_distance_epsilon_m", 0.25),
                        0.25,
                    ),
                    request_sim_time_bucket_sec=_as_float(
                        adapter_cfg.get("request_sim_time_bucket_sec", 0.0),
                        0.0,
                    ),
                    request_bucket_dedupe_enabled=bool(
                        adapter_cfg.get("request_bucket_dedupe_enabled", True)
                    ),
                    ev_translation=self.ev_translation,
                    state_manager=self.state_manager,
                    event_manager=self.event_manager,
                    fed_iface=self._feder_ep,
                    dt_id=self.dt_id,
                    gateway_id=self.gateway_id,
                    logger=self.logger,
                    emit_state_trace=bool(adapter_cfg.get("emit_state_trace", False)),
                    error_backoff_max_sec=_as_float(adapter_cfg.get("error_backoff_max_sec", 2.0), 2.0),
                    peer_allow_fn=self._fcm_peer_allowed,
                    peer_context_update_fn=self._fcm_update_from_ev_state,
                )
            )
        self.protocol_adaptation = ProtocolAdaptationManager(adapters=adapters)

        self.membership_register_topic = str(_deep_get(self.cfg, ["node", "federation", "membership_register_topic"], "federation/membership/register"))
        self.membership_heartbeat_topic = str(_deep_get(self.cfg, ["node", "federation", "membership_heartbeat_topic"], "federation/membership/heartbeat"))
        self.catalog_upsert_topic = str(_deep_get(self.cfg, ["node", "federation", "catalog_upsert_topic"], "federation/catalog/upsert"))
        self.heartbeat_interval_sec = _as_float(_deep_get(self.cfg, ["node", "federation", "heartbeat_interval_sec"], 5.0), 5.0)
        self.catalog_interval_sec = _as_float(_deep_get(self.cfg, ["node", "federation", "catalog_interval_sec"], 20.0), 20.0)
        self._last_heartbeat = 0.0
        self._last_catalog = 0.0
        self._proc_start_wall_s = _now()
        self._proc_start_cpu_s = time.process_time()
        fcm_cfg = dict(_deep_get(self.cfg, ["node", "federation_context"], {}) or {})
        self.federation_context = FederationContextManager(
            enabled=bool(fcm_cfg.get("enabled", False)),
            dt_id=self.dt_id,
            dt_type=self.dt_type,
            gateway_id=self.gateway_id,
            logger=self.logger,
            federation_iface=self.federation,
            cfg=fcm_cfg,
        )
        # Domain-agnostic alias maps for context extraction.
        alias_cfg = dict(fcm_cfg.get("context_aliases", {}) or {})
        self._fcm_alias_route_sequence = [
            str(x).strip()
            for x in _as_list(
                alias_cfg.get(
                    "route_sequence",
                    ["route_sequence", "route_tls_sequence", "route_intersections", "next_tls_order"],
                )
            )
            if str(x).strip()
        ]
        self._fcm_alias_next_nodes = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("next_nodes", ["next_tls", "next_nodes"]))
            if str(x).strip()
        ]
        self._fcm_alias_current_node = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("current_node", ["current_tls", "tls_id", "to_tls"]))
            if str(x).strip()
        ]
        self._fcm_alias_current_edge = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("current_edge", ["current_edge_id", "in_edge_id", "next_edge_id"]))
            if str(x).strip()
        ]
        self._fcm_alias_entity_id = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("entity_id", ["ev_id", "entity_id", "source_dt_id"]))
            if str(x).strip()
        ]
        self._fcm_alias_radius = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("radius_m", ["radius_m", "distance_to_intersection_m", "stopline_dist_m"]))
            if str(x).strip()
        ]
        self._fcm_alias_lookahead = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("lookahead_hops", ["lookahead_hops"]))
            if str(x).strip()
        ]
        self._fcm_alias_max_candidates = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("max_candidates", ["max_candidates"]))
            if str(x).strip()
        ]
        self._fcm_alias_sim_time = [
            str(x).strip()
            for x in _as_list(alias_cfg.get("sim_time", ["sim_time"]))
            if str(x).strip()
        ]
        self.logger.write(
            "fcm.config.loaded",
            dt_id=self.dt_id,
            gateway_id=self.gateway_id,
            mqtt_client_suffix=str(self._mqtt_client_suffix),
            enabled=bool(self.federation_context.enabled),
            query_enabled=bool(self.federation_context.discovery_query_topic_enabled),
            query_interval_sec=float(self.federation_context.query_interval_sec),
            peer_ttl_sec=float(self.federation_context.peer_ttl_sec),
            fail_open=bool(self.federation_context.fail_open),
            role_filter=str(self.federation_context.role_filter or ""),
            event_filter=str(self.federation_context.discovery_event_filter or ""),
            require_active_membership=bool(self.federation_context.require_active_membership),
            discovery_query_topic=str(self.federation_context.discovery_query_topic or ""),
            discovery_reply_prefix=str(self.federation_context.discovery_reply_prefix or ""),
            membership_state_topic=str(self.federation_context.membership_state_topic or ""),
            membership_events_topic=str(self.federation_context.membership_events_topic or ""),
            discovery_capabilities=list(self.federation_context.discovery_capabilities),
            discovery_service_names=list(self.federation_context.discovery_service_names),
            discovery_directions=list(self.federation_context.discovery_directions),
            discovery_node_dedup=bool(self.federation_context.discovery_node_dedup),
            selection_policy=str(self.federation_context.selection_policy_name),
            context_gate_enabled=bool(self.federation_context.context_gate_enabled),
            context_back_hops=int(self.federation_context.context_back_hops),
            query_require_valid_context=bool(self.federation_context.query_require_valid_context),
            query_require_complete_context=bool(self.federation_context.query_require_complete_context),
            query_min_route_len=int(self.federation_context.query_min_route_len),
            query_skip_log_cooldown_sec=float(self.federation_context.query_skip_log_cooldown_sec),
            keep_last_valid_context=bool(self.federation_context.keep_last_valid_context),
            valid_context_hold_sec=float(self.federation_context.valid_context_hold_sec),
            hold_peers_without_context=bool(self.federation_context.hold_peers_without_context),
            hold_peers_max_sec=float(self.federation_context.hold_peers_max_sec),
            min_active_members_before_query=int(self.federation_context.min_active_members_before_query),
            query_startup_grace_sec=float(self.federation_context.query_startup_grace_sec),
            peer_hysteresis_min_hold_sec=float(self.federation_context.peer_hysteresis_min_hold_sec),
            peer_change_window_sec=float(self.federation_context.peer_change_window_sec),
            peer_change_max_add=int(self.federation_context.peer_change_max_add),
            snapshot_interval_sec=float(self.federation_context.snapshot_interval_sec),
        )

    def _normalized_capabilities(self) -> Tuple[List[Dict[str, Any]], List[str]]:
        raw_caps = list(_deep_get(self.cfg, ["node", "capabilities"], []) or [])
        caps_out: List[Dict[str, Any]] = []
        cap_names: List[str] = []
        for c in raw_caps:
            if isinstance(c, dict):
                name = str(c.get("name", c.get("id", "")) or "").strip()
                if not name:
                    continue
                entry = dict(c)
                entry["name"] = name
            else:
                name = str(c).strip()
                if not name:
                    continue
                entry = {"name": name}
            caps_out.append(entry)
            cap_names.append(name)
        cap_names = list(dict.fromkeys(cap_names))
        return caps_out, cap_names

    def _first_non_empty(self, payload: Dict[str, Any], keys: List[str]) -> Any:
        for k in keys:
            if k in payload and payload.get(k) not in (None, ""):
                return payload.get(k)
        return None

    def _extract_next_nodes(self, payload: Dict[str, Any]) -> Tuple[List[str], Dict[str, float]]:
        raw = self._first_non_empty(payload, self._fcm_alias_next_nodes)
        next_ids: List[str] = []
        next_dist_m: Dict[str, float] = {}
        for x in _as_list(raw):
            tid = ""
            if isinstance(x, (list, tuple)) and len(x) >= 1:
                tid = str(x[0] or "").strip()
                if len(x) >= 2:
                    d = _as_float(x[1], -1.0)
                    if tid and d >= 0.0:
                        next_dist_m[tid] = d
            elif isinstance(x, dict):
                tid = str(x.get("tls_id", x.get("node_id", x.get("id", ""))) or "").strip()
                d = _as_float(x.get("distance_m", x.get("dist_m", x.get("distance", -1.0))), -1.0)
                if tid and d >= 0.0:
                    next_dist_m[tid] = d
            else:
                tid = str(x or "").strip()
            if tid:
                next_ids.append(tid)
        return list(dict.fromkeys(next_ids)), next_dist_m

    def _extract_route_sequence(self, payload: Dict[str, Any]) -> List[str]:
        for k in self._fcm_alias_route_sequence:
            seq = FederationContextManager._extract_route_tls(payload.get(k, []))
            if seq:
                return seq
        return []

    def _build_fcm_context(self, payload: Dict[str, Any], *, current_node_hint: str = "") -> Dict[str, Any]:
        p = dict(payload or {})
        route_seq = self._extract_route_sequence(p)
        next_ids, next_dist_m = self._extract_next_nodes(p)
        if not route_seq and next_ids:
            route_seq = list(next_ids)

        current_node = str(current_node_hint or "").strip()
        if not current_node:
            v = self._first_non_empty(p, self._fcm_alias_current_node)
            current_node = str(v or "").strip()
        if not current_node and self.dt_type.lower() == "trafficlightsystem":
            current_node = str(self.dt_id or "")
        if not current_node and next_ids:
            current_node = str(next_ids[0] or "")
        if not current_node and route_seq:
            current_node = str(route_seq[0] or "")
        if not route_seq and current_node:
            route_seq = [str(current_node)]
        if not next_ids and route_seq:
            next_ids = list(route_seq)

        entity_id = str(self._first_non_empty(p, self._fcm_alias_entity_id) or "").strip()
        cur_edge = str(self._first_non_empty(p, self._fcm_alias_current_edge) or "").strip()
        lookahead = _as_int(self._first_non_empty(p, self._fcm_alias_lookahead), 0)
        if lookahead <= 0:
            lookahead = max(1, int(len(next_ids) or len(route_seq) or 1))
        max_candidates = _as_int(self._first_non_empty(p, self._fcm_alias_max_candidates), 0)
        if max_candidates <= 0:
            max_candidates = max(3, int(min(12, len(route_seq) or len(next_ids) or 3)))
        radius_m = _as_float(self._first_non_empty(p, self._fcm_alias_radius), 0.0)
        sim_time = _as_float(self._first_non_empty(p, self._fcm_alias_sim_time), 0.0)

        ctx = {
            "entity_id": entity_id,
            "ev_id": entity_id,  # backwards-compatible field name in existing traces/analysis
            "route_tls_sequence": list(route_seq),
            "route_sequence": list(route_seq),
            "next_tls_order": list(next_ids),
            "next_tls_distance_m": dict(next_dist_m),
            "current_tls": current_node,
            "current_edge_id": cur_edge,
            "lookahead_hops": int(lookahead),
            "radius_m": float(radius_m),
            "max_candidates": int(max_candidates),
            "sim_time": float(sim_time),
        }
        return ctx

    def _fcm_update_from_state_snapshot(self, state: Dict[str, Any], now_wall: float) -> None:
        if not bool(self.federation_context.enabled):
            return
        context = self._build_fcm_context(dict(state or {}))
        self.federation_context.update_query_context(context, reason="ev_state_pull")

    def _fcm_update_from_message(self, payload: Dict[str, Any], *, source: str, topic: str, now_wall: float) -> None:
        if not bool(self.federation_context.enabled):
            return
        context = self._build_fcm_context(dict(payload or {}))
        if not str(context.get("entity_id", "") or "").strip():
            return
        self.federation_context.update_query_context(context, reason=f"{source}:{topic}")

    # Backwards-compatible wrappers (older callsites/tests may still refer to these names).
    def _fcm_update_from_ev_state(self, state: Dict[str, Any], now_wall: float) -> None:
        self._fcm_update_from_state_snapshot(state, now_wall)

    def _fcm_update_from_payload(self, payload: Dict[str, Any], *, source: str, topic: str, now_wall: float) -> None:
        self._fcm_update_from_message(payload, source=source, topic=topic, now_wall=now_wall)

    def _load_rules(self) -> List[TopicRule]:
        rules_in = list(_deep_get(self.cfg, ["node", "communication", "topic_map", "rules"], []) or [])
        out: List[TopicRule] = []
        for item in rules_in:
            out.append(
                TopicRule(
                    name=str(item.get("name", "rule")),
                    direction=str(item.get("direction", "local_to_fed")),
                    subscribe_topic=_expand_topic(str(item.get("subscribe_topic", "")), self.dt_id),
                    publish_topic=_expand_topic(str(item.get("publish_topic", "")), self.dt_id),
                    event_type=str(item.get("event_type", "event")),
                )
            )
        return [r for r in out if r.subscribe_topic and r.publish_topic]

    def _load_monitor_rules(self) -> List[MonitorRule]:
        rules_in = list(_deep_get(self.cfg, ["node", "communication", "monitor", "rules"], []) or [])
        out: List[MonitorRule] = []
        for item in rules_in:
            out.append(
                MonitorRule(
                    name=str(item.get("name", "monitor_rule")),
                    source=str(item.get("source", "any")).strip().lower(),
                    subscribe_topic=_expand_topic(str(item.get("subscribe_topic", "")), self.dt_id),
                    kind=str(item.get("kind", "state")).strip().lower(),
                    state_key=str(item.get("state_key", "")),
                    event_name=str(item.get("event_name", "")),
                    store_last_payload=bool(item.get("store_last_payload", True)),
                )
            )
        clean: List[MonitorRule] = []
        for r in out:
            if r.source not in {"local", "federation", "any"}:
                continue
            if r.kind not in {"state", "event"}:
                continue
            if not r.subscribe_topic:
                continue
            clean.append(r)
        return clean

    def _rules_for(self, direction: str, topic: str) -> List[TopicRule]:
        return [r for r in self.rules if r.direction == direction and _topic_match(r.subscribe_topic, topic)]

    def _monitor_rules_for(self, source: str, topic: str) -> List[MonitorRule]:
        src = str(source).strip().lower()
        return [
            r
            for r in self.monitor_rules
            if r.source in {src, "any"} and _topic_match(r.subscribe_topic, topic)
        ]

    def _render_monitor_template(self, template: str, topic: str, payload: Dict[str, Any], default: str) -> str:
        t = str(template or "").strip()
        if not t:
            return str(default)
        vals = {
            "dt_id": str(self.dt_id),
            "dt_type": str(self.dt_type),
            "topic": str(topic),
            "source_dt_id": str(payload.get("source_dt_id", payload.get("node_id", payload.get("ev_id", "unknown")))),
            "source_dt_type": str(payload.get("source_dt_type", payload.get("role", "unknown"))),
            "node_id": str(payload.get("node_id", "")),
            "ev_id": str(payload.get("ev_id", "")),
            "tls_id": str(payload.get("tls_id", payload.get("to_tls", payload.get("from_tls", "")))),
        }
        try:
            return str(t.format(**vals))
        except Exception:
            return str(default)

    def _classify_artefact(self, rule: TopicRule, topic: str, payload: Dict[str, Any]) -> str:
        et = str(getattr(rule, "event_type", "") or "").lower()
        tp = str(topic or "").lower()
        pub = str(getattr(rule, "publish_topic", "") or "").lower()
        sub = str(getattr(rule, "subscribe_topic", "") or "").lower()
        hint = f"{et} {tp} {pub} {sub}"
        if ("coord" in hint) or ("reservation" in hint) or ("preemption" in hint):
            return "coordination"
        if ("state" in hint) and ("request" not in hint):
            return "state"
        if ("request" in hint) or ("response" in hint) or ("decision" in hint):
            return "request_response"
        return "event"

    def _extract_source_dt_id(self, payload: Dict[str, Any]) -> str:
        return str(
            payload.get("source_dt_id")
            or payload.get("node_id")
            or payload.get("ev_id")
            or payload.get("tls_id")
            or "unknown"
        )

    def _payload_age_ms(self, payload: Dict[str, Any], now_wall: float) -> Optional[float]:
        for k in ("request_age_ms", "age_ms", "staleness_ms"):
            if k in payload:
                v = _as_float(payload.get(k), -1.0)
                if v >= 0.0:
                    return float(v)
        ts_wall = _as_float(payload.get("ts", payload.get("timestamp", 0.0)), 0.0)
        if ts_wall > 1e9:
            return max(0.0, 1000.0 * (float(now_wall) - float(ts_wall)))
        return None

    def _extract_expiry(self, payload: Dict[str, Any]) -> Tuple[Optional[float], str]:
        expiry_keys = (
            "expires_at",
            "expire_at",
            "expiry_time",
            "valid_until",
            "deadline",
            "window_end",
            "window_end_time",
            "expiry_sim_time",
            "expires_at_sim_time",
            "expires_at_wall",
        )
        for k in expiry_keys:
            if k in payload:
                v = _as_float(payload.get(k), 0.0)
                if v <= 0.0:
                    continue
                lk = str(k).lower()
                if "sim" in lk:
                    return float(v), "sim"
                if "wall" in lk:
                    return float(v), "wall"
                if v > 1e9:
                    return float(v), "wall"
                return float(v), "sim"
        return None, "unknown"

    def _fcm_peer_allowed(self, peer_id: str, now_wall: float, context: str) -> Tuple[bool, str]:
        try:
            return self.federation_context.peer_allowed(
                str(peer_id),
                now=float(now_wall),
                context=str(context),
            )
        except Exception as e:
            self.logger.write(
                "fcm.peer_set.error",
                dt_id=self.dt_id,
                peer_id=str(peer_id),
                context=str(context),
                err=f"{type(e).__name__}:{e}",
            )
            return True, "fail_open_on_error"

    def _apply_monitor_rules(self, source: str, topic: str, payload: Dict[str, Any]) -> None:
        now_wall = _now()
        src_dt = self._extract_source_dt_id(payload)
        age_ms = self._payload_age_ms(payload, now_wall)
        for rule in self._monitor_rules_for(source, topic):
            if rule.kind == "state":
                state_key = self._render_monitor_template(
                    rule.state_key,
                    topic,
                    payload,
                    default=f"monitor.state.{rule.name}",
                )
                self.state_manager.set(state_key, dict(payload or {}))
                n = self.event_manager.inc(f"monitor_state:{rule.name}")
                self.logger.write(
                    "fnm.monitor.state",
                    rule=rule.name,
                    source=str(source),
                    topic=str(topic),
                    source_dt_id=src_dt,
                    state_key=state_key,
                    payload_age_ms=age_ms,
                    count=n,
                )
            else:
                event_name = self._render_monitor_template(
                    rule.event_name,
                    topic,
                    payload,
                    default=f"monitor_event:{rule.name}",
                )
                n = self.event_manager.inc(event_name)
                if bool(rule.store_last_payload):
                    self.state_manager.set(f"last_event:{rule.name}", dict(payload or {}))
                self.logger.write(
                    "fnm.monitor.event",
                    rule=rule.name,
                    source=str(source),
                    topic=str(topic),
                    source_dt_id=src_dt,
                    event_name=event_name,
                    payload_age_ms=age_ms,
                    count=n,
                )

    @staticmethod
    def _namespace_from_wire_topic(wire_topic: str, logical_topic: str) -> str:
        wire = str(wire_topic or "").strip().strip("/")
        logical = str(logical_topic or "").strip().strip("/")
        if not wire or not logical or wire == logical:
            topic = wire or logical
            marker = "/federation/"
            if marker in topic:
                return topic.split(marker, 1)[0].strip("/")
            return ""
        suffix = f"/{logical}"
        if wire.endswith(suffix):
            return wire[: -len(suffix)].strip("/")
        marker = "/federation/"
        if marker in wire:
            return wire.split(marker, 1)[0].strip("/")
        return ""

    @staticmethod
    def _message_correlation_id(payload: Dict[str, Any]) -> str:
        for key in ("request_id", "correlation_id", "conversation_id", "message_id"):
            val = str((payload or {}).get(key, "") or "").strip()
            if val:
                return val
        return ""

    def _on_local_message(self, topic: str, payload: Dict[str, Any], wire_topic: str = "") -> None:
        local_in_ts = _now()
        local_payload = dict(payload or {})
        self._fcm_update_from_payload(local_payload, source="local", topic=str(topic), now_wall=float(local_in_ts))
        if not str(local_payload.get("message_id", "") or "").strip():
            local_payload["message_id"] = _new_id("msg")
        self.data_manager.write_raw_message(direction="local_in", topic=topic, payload=local_payload)
        self._apply_monitor_rules("local", topic, local_payload)
        for rule in self._rules_for("local_to_fed", topic):
            t0 = _now()
            artefact_kind = self._classify_artefact(rule, topic, local_payload)
            try:
                out = self.translator.local_to_fed(
                    dict(local_payload or {}),
                    dt_id=self.dt_id,
                    dt_type=self.dt_type,
                    event_type=rule.event_type,
                )
            except Exception as e:
                self.event_manager.inc(f"delivery_local_to_fed:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.local_to_fed",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="translate_error",
                    src=topic,
                    dst=rule.publish_topic,
                    err=f"{type(e).__name__}:{e}",
                )
                continue
            msg_id = str(out.get("message_id", local_payload.get("message_id", _new_id("msg"))) or _new_id("msg"))
            out["message_id"] = msg_id
            meta_in = out.get("fnm_meta", {})
            meta = dict(meta_in if isinstance(meta_in, dict) else {})
            meta.setdefault("origin_dt_id", self.dt_id)
            meta.setdefault("origin_gateway_id", self.gateway_id)
            meta.setdefault("origin_topic", str(topic))
            meta.setdefault("origin_rule", str(rule.name))
            meta.setdefault("local_ingest_ts", float(local_in_ts))
            schema_done_ts = _now()
            meta["schema_done_ts"] = float(schema_done_ts)
            out["fnm_meta"] = meta
            fed_publish_ts = _now()
            meta["fed_publish_ts"] = float(fed_publish_ts)
            out["fnm_meta"] = meta
            publish_topic = _render_wildcard_publish_topic(rule.subscribe_topic, topic, str(rule.publish_topic))
            reply_topic = str(
                out.get("reply_context_topic", "")
                or local_payload.get("reply_context_topic", "")
                or ""
            ).strip().strip("/")
            reply_logical_topic = str(
                out.get("reply_context_logical_topic", "")
                or local_payload.get("reply_context_logical_topic", "")
                or ""
            ).strip().strip("/")
            # Reply topics are only valid for the aggregate downstream context
            # response. Contextual wildcard routes such as node/+, edge/+ and
            # region/+ must preserve their resolved destination so subscribed
            # SI-DTs receive area-specific drone observations.
            publish_template = str(rule.publish_topic or "")
            is_contextual_wildcard_context = bool(
                str(rule.event_type).lower() == "downstreamcontext"
                and "context/downstream" in publish_template
                and ("+" in str(rule.subscribe_topic or "") or "+" in publish_template)
            )
            use_reply_topic = bool(
                reply_topic
                and str(rule.event_type).lower() == "downstreamcontext"
                and "context/downstream" in reply_topic
                and not is_contextual_wildcard_context
            )
            corr_id = self._message_correlation_id(out) or self._message_correlation_id(local_payload)
            ns = str(
                out.get("requester_topic_namespace", "")
                or out.get("topic_namespace", "")
                or local_payload.get("requester_topic_namespace", "")
                or local_payload.get("topic_namespace", "")
                or (self._scenario_namespace_by_request_id.get(corr_id, "") if corr_id else "")
            ).strip().strip("/")
            if use_reply_topic:
                publish_topic = reply_topic
                out["logical_federation_topic"] = reply_logical_topic or str(rule.publish_topic)
                if ns:
                    out["requester_topic_namespace"] = ns
                    out.setdefault("topic_namespace", ns)
                self.logger.write(
                    "fnm.delivery.local_to_fed.reply_topic_selected",
                    rule=rule.name,
                    correlation_id=corr_id,
                    namespace=ns,
                    src=topic,
                    dst=publish_topic,
                    wire_dst=publish_topic,
                    reply_context_topic=reply_topic,
                    reply_context_logical_topic=reply_logical_topic,
                    fnm_topic_namespace=self.topic_namespace,
                )
            elif ns:
                out["requester_topic_namespace"] = ns
                out.setdefault("topic_namespace", ns)
                out["logical_federation_topic"] = str(publish_topic)
                if not self.topic_namespace:
                    publish_topic = f"{ns}/{publish_topic}"
                self.logger.write(
                    "fnm.delivery.local_to_fed.namespace_preserved",
                    rule=rule.name,
                    correlation_id=corr_id,
                    namespace=ns,
                    src=topic,
                    dst=rule.publish_topic,
                    wire_dst=publish_topic,
                    fnm_topic_namespace=self.topic_namespace,
                )
            pub_res = self.federation.publish(publish_topic, out)
            if not bool(pub_res.get("ok", False)):
                self.event_manager.inc(f"delivery_local_to_fed:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.local_to_fed",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="publish_error",
                    src=topic,
                    dst=publish_topic,
                    wire_topic=str(pub_res.get("wire_topic", "")),
                    rc=pub_res.get("rc"),
                    message_id=msg_id,
                    err=str(pub_res.get("error", "")),
                )
                continue
            self.event_manager.inc("local_to_fed")
            self.event_manager.inc(f"delivery_local_to_fed:{artefact_kind}:ok")
            self.logger.write(
                "fnm.route.local_to_fed",
                rule=rule.name,
                src=topic,
                dst=publish_topic,
                message_id=msg_id,
                artefact_kind=artefact_kind,
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
                duration_ms=(1000.0 * (_now() - t0)),
            )
            self.logger.write(
                "fnm.stage.local_to_fed",
                rule=rule.name,
                message_id=msg_id,
                artefact_kind=artefact_kind,
                local_ingest_to_schema_ms=(1000.0 * (schema_done_ts - float(meta.get("local_ingest_ts", local_in_ts)))),
                schema_to_fed_publish_ms=(1000.0 * (fed_publish_ts - schema_done_ts)),
                local_to_fed_total_ms=(1000.0 * (fed_publish_ts - float(meta.get("local_ingest_ts", local_in_ts)))),
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
            )
            self.logger.write(
                "fnm.delivery.local_to_fed",
                rule=rule.name,
                artefact_kind=artefact_kind,
                status="ok",
                src=topic,
                dst=publish_topic,
                wire_topic=str(pub_res.get("wire_topic", "")),
                message_id=msg_id,
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
            )
            self.data_manager.write_raw_message(direction="fed_out", topic=publish_topic, payload=out)

    def _on_federation_message(self, topic: str, payload: Dict[str, Any], wire_topic: str = "") -> None:
        fed_in_ts = _now()
        fed_payload = dict(payload or {})
        self._fcm_update_from_payload(fed_payload, source="federation", topic=str(topic), now_wall=float(fed_in_ts))
        self.data_manager.write_raw_message(direction="fed_in", topic=topic, payload=fed_payload)
        self._apply_monitor_rules("federation", topic, fed_payload)
        self.federation_context.on_federation_message(str(topic), fed_payload, now=float(fed_in_ts))
        for rule in self._rules_for("fed_to_local", topic):
            t0 = _now()
            artefact_kind = self._classify_artefact(rule, topic, fed_payload)
            try:
                out = self.translator.fed_to_local(dict(fed_payload or {}))
            except Exception as e:
                self.event_manager.inc(f"delivery_fed_to_local:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.fed_to_local",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="translate_error",
                    src=topic,
                    dst=rule.publish_topic,
                    err=f"{type(e).__name__}:{e}",
                )
                continue
            msg_id = str(out.get("message_id", fed_payload.get("message_id", _new_id("msg"))) or _new_id("msg"))
            out["message_id"] = msg_id
            meta_in = out.get("fnm_meta", {})
            meta = dict(meta_in if isinstance(meta_in, dict) else {})
            fed_publish_ts = _as_float(meta.get("fed_publish_ts", 0.0), 0.0)
            local_ingest_ts = _as_float(meta.get("local_ingest_ts", 0.0), 0.0)
            meta["remote_receive_ts"] = float(fed_in_ts)
            local_invoke_ts = _now()
            meta["local_invoke_ts"] = float(local_invoke_ts)
            out["fnm_meta"] = meta
            corr_id = self._message_correlation_id(out) or self._message_correlation_id(fed_payload)
            ns = self._namespace_from_wire_topic(str(wire_topic or ""), str(topic))
            if ns:
                out["requester_topic_namespace"] = ns
                out.setdefault("topic_namespace", ns)
                out["logical_federation_topic"] = str(topic)
                if corr_id:
                    self._scenario_namespace_by_request_id[corr_id] = ns
                self.logger.write(
                    "fnm.delivery.fed_to_local.namespace_captured",
                    rule=rule.name,
                    correlation_id=corr_id,
                    namespace=ns,
                    src=topic,
                    wire_topic=str(wire_topic or ""),
                )
            expiry_value, expiry_ref = self._extract_expiry(out)
            before_expiry: Optional[bool] = None
            expiry_delta_ms: Optional[float] = None
            if expiry_value is not None:
                if expiry_ref == "sim":
                    cur_sim = _as_float(out.get("sim_time", fed_payload.get("sim_time", 0.0)), 0.0)
                    if cur_sim > 0.0:
                        before_expiry = bool(cur_sim <= expiry_value)
                        expiry_delta_ms = 1000.0 * (expiry_value - cur_sim)
                elif expiry_ref == "wall":
                    before_expiry = bool(fed_in_ts <= expiry_value)
                    expiry_delta_ms = 1000.0 * (expiry_value - fed_in_ts)
            publish_topic = _render_wildcard_publish_topic(rule.subscribe_topic, topic, str(rule.publish_topic))
            pub_res = self.intra.publish(publish_topic, out)
            if not bool(pub_res.get("ok", False)):
                self.event_manager.inc(f"delivery_fed_to_local:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.fed_to_local",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="publish_error",
                    src=topic,
                    dst=publish_topic,
                    wire_topic=str(pub_res.get("wire_topic", "")),
                    rc=pub_res.get("rc"),
                    message_id=msg_id,
                    err=str(pub_res.get("error", "")),
                )
                continue
            self.event_manager.inc("fed_to_local")
            self.event_manager.inc(f"delivery_fed_to_local:{artefact_kind}:ok")
            self.logger.write(
                "fnm.route.fed_to_local",
                rule=rule.name,
                src=topic,
                dst=publish_topic,
                message_id=msg_id,
                artefact_kind=artefact_kind,
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
                duration_ms=(1000.0 * (_now() - t0)),
            )
            self.logger.write(
                "fnm.stage.fed_to_local",
                rule=rule.name,
                message_id=msg_id,
                artefact_kind=artefact_kind,
                fed_publish_to_remote_receive_ms=(
                    (1000.0 * (fed_in_ts - fed_publish_ts)) if fed_publish_ts > 0.0 else None
                ),
                remote_receive_to_local_invoke_ms=(1000.0 * (local_invoke_ts - fed_in_ts)),
                fed_to_local_total_ms=(1000.0 * (local_invoke_ts - fed_in_ts)),
                origin_to_local_invoke_ms=(
                    (1000.0 * (local_invoke_ts - local_ingest_ts)) if local_ingest_ts > 0.0 else None
                ),
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
            )
            self.logger.write(
                "fnm.delivery.fed_to_local",
                rule=rule.name,
                artefact_kind=artefact_kind,
                status="ok",
                src=topic,
                dst=rule.publish_topic,
                wire_topic=str(pub_res.get("wire_topic", "")),
                message_id=msg_id,
                before_expiry=before_expiry,
                expiry_delta_ms=expiry_delta_ms,
                expiry_ref=expiry_ref,
                payload_size_bytes=int(pub_res.get("payload_size_bytes", 0) or 0),
            )
            self.data_manager.write_raw_message(direction="local_out", topic=publish_topic, payload=out)

    def _publish_register(self) -> None:
        _caps, cap_names = self._normalized_capabilities()
        payload = {
            "schema": "federation.membership.v1",
            "event": "register",
            "request_id": _new_id("reg"),
            "gateway_id": self.gateway_id,
            "fnm_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "domain": str(_deep_get(self.cfg, ["node", "domain"], "traffic")),
            "capabilities": list(cap_names),
            "status": "REGISTERED",
            "ts": _now(),
        }
        pub_res = self.federation.publish(self.membership_register_topic, payload)
        self.logger.write(
            "fnm.membership.register_pub",
            topic=self.membership_register_topic,
            wire_topic=str(pub_res.get("wire_topic", "")),
            status=("ok" if bool(pub_res.get("ok", False)) else "publish_error"),
            rc=pub_res.get("rc"),
            err=str(pub_res.get("error", "")),
        )

    def _publish_heartbeat(self) -> None:
        payload = {
            "schema": "federation.membership.v1",
            "event": "heartbeat",
            "gateway_id": self.gateway_id,
            "fnm_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "status": "ACTIVE",
            "ts": _now(),
        }
        pub_res = self.federation.publish(self.membership_heartbeat_topic, payload)
        if not bool(pub_res.get("ok", False)):
            self.logger.write(
                "fnm.membership.heartbeat_pub",
                topic=self.membership_heartbeat_topic,
                wire_topic=str(pub_res.get("wire_topic", "")),
                status="publish_error",
                rc=pub_res.get("rc"),
                err=str(pub_res.get("error", "")),
            )

    def _publish_catalog(self) -> None:
        services = []
        for r in self.rules:
            services.append(
                {
                    "name": str(r.name),
                    "direction": str(r.direction),
                    "event_type": str(r.event_type),
                    "publish_topic": str(r.publish_topic),
                    "subscribe_topic": str(r.subscribe_topic),
                }
            )
        capability_entries, capability_names = self._normalized_capabilities()
        payload = {
            "schema": "federation.catalog.v2",
            "event": "upsert",
            "gateway_id": self.gateway_id,
            "fnm_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "services": services,
            "capabilities": capability_entries,
            "capability_names": capability_names,
            "dt_profile": dict(_deep_get(self.cfg, ["node", "dt_profile"], {}) or {}),
            "ts": _now(),
        }
        pub_res = self.federation.publish(self.catalog_upsert_topic, payload)
        self.logger.write(
            "fnm.catalog.upsert_pub",
            topic=self.catalog_upsert_topic,
            wire_topic=str(pub_res.get("wire_topic", "")),
            status=("ok" if bool(pub_res.get("ok", False)) else "publish_error"),
            rc=pub_res.get("rc"),
            err=str(pub_res.get("error", "")),
        )

    def start(self) -> None:
        in_topics = [r.subscribe_topic for r in self.rules if r.direction == "local_to_fed"]
        fed_topics = [r.subscribe_topic for r in self.rules if r.direction == "fed_to_local"]
        for mr in self.monitor_rules:
            if mr.source in {"local", "any"}:
                in_topics.append(mr.subscribe_topic)
            if mr.source in {"federation", "any"}:
                fed_topics.append(mr.subscribe_topic)
        for fcm_topic in self.federation_context.federation_subscriptions():
            fed_topics.append(str(fcm_topic))
        # preserve order while deduplicating
        in_topics = list(dict.fromkeys([str(t) for t in in_topics if str(t)]))
        fed_topics = list(dict.fromkeys([str(t) for t in fed_topics if str(t)]))
        self.logger.write(
            "fnm.start",
            gateway_id=self.gateway_id,
            dt_id=self.dt_id,
            dt_type=self.dt_type,
            n_rules=len(self.rules),
            n_monitor_rules=len(self.monitor_rules),
            adapter_count=len(list(self.protocol_adaptation.adapters)),
            fcm_enabled=bool(self.federation_context.enabled),
            topic_namespace=self.topic_namespace,
        )
        self.intra.start(in_topics)
        self.federation.start(fed_topics)
        intra_ok = self.intra.wait_connected(2.0)
        federation_ok = self.federation.wait_connected(2.0)
        self.logger.write(
            "fnm.mqtt.startup_connectivity",
            intra_connected=int(intra_ok),
            federation_connected=int(federation_ok),
            topic_namespace=self.topic_namespace,
        )
        if not federation_ok:
            self.logger.write(
                "fnm.mqtt.startup_warning",
                reason="federation_mqtt_not_connected",
                topic_namespace=self.topic_namespace,
            )
        self._publish_register()
        self._publish_catalog()
        self._last_catalog = _now()
        self._last_heartbeat = _now()

    def tick(self) -> None:
        now = _now()
        self.protocol_adaptation.tick(now)
        self.federation_context.tick(now)
        if now - self._last_heartbeat >= self.heartbeat_interval_sec:
            self._publish_heartbeat()
            self._last_heartbeat = now
        if now - self._last_catalog >= self.catalog_interval_sec:
            self._publish_catalog()
            self._last_catalog = now

    def stop(self) -> None:
        self.stop_evt.set()
        self.protocol_adaptation.stop()
        self.intra.stop()
        self.federation.stop()
        wall_runtime_s = max(0.0, _now() - float(self._proc_start_wall_s))
        cpu_runtime_s = max(0.0, float(time.process_time()) - float(self._proc_start_cpu_s))
        cpu_util_pct = (100.0 * cpu_runtime_s / wall_runtime_s) if wall_runtime_s > 0 else None
        max_rss_kb = None
        if resource is not None:
            try:
                ru = resource.getrusage(resource.RUSAGE_SELF)
                rss = float(getattr(ru, "ru_maxrss", 0.0) or 0.0)
                # macOS reports bytes; Linux reports kB.
                if rss > 0 and sys.platform == "darwin":
                    rss = rss / 1024.0
                max_rss_kb = rss if rss > 0 else None
            except Exception:
                max_rss_kb = None
        self.logger.write(
            "fnm.overhead.process",
            dt_id=self.dt_id,
            gateway_id=self.gateway_id,
            wall_runtime_s=wall_runtime_s,
            cpu_runtime_s=cpu_runtime_s,
            cpu_util_pct=cpu_util_pct,
            max_rss_kb=max_rss_kb,
            local_to_fed=int(self.event_manager.counters.get("local_to_fed", 0) or 0),
            fed_to_local=int(self.event_manager.counters.get("fed_to_local", 0) or 0),
        )
        self.logger.write("fnm.stop", dt_id=self.dt_id, gateway_id=self.gateway_id)
        self.logger.close()


class FederationNodeManager(FederationNodeCore):
    """Backwards-compatible alias."""
    pass


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return dict(yaml.safe_load(f) or {})


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Federation Node Manager (minimal prototype)")
    ap.add_argument("--config", required=True, help="YAML configuration path")
    ap.add_argument("--tick-sec", type=float, default=0.1, help="main loop period")
    ap.add_argument("--log-jsonl", default="", help="optional jsonl trace log")
    ap.add_argument("--data-base-dir", default="", help="optional override for node.data_manager.base_dir")
    ap.add_argument("--data-run-id", default="", help="optional override for node.data_manager.run_id")
    ap.add_argument(
        "--data-persist-raw-messages",
        choices=["auto", "on", "off"],
        default="auto",
        help="override node.data_manager.persist_raw_messages",
    )
    ap.add_argument(
        "--topic-namespace",
        default="",
        help="optional MQTT topic namespace prefix for full run isolation (e.g., run_20260330_abc)",
    )
    ap.add_argument("--intra-mqtt-host", default="", help="optional override for node.communication.internal_event_bus.broker.host")
    ap.add_argument("--intra-mqtt-port", type=int, default=0, help="optional override for node.communication.internal_event_bus.broker.port")
    ap.add_argument("--federation-mqtt-host", default="", help="optional override for node.communication.federation_event_bus.broker.host")
    ap.add_argument("--federation-mqtt-port", type=int, default=0, help="optional override for node.communication.federation_event_bus.broker.port")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    cfg = _load_yaml(args.config)
    node = dict(cfg.get("node", {}) or {})
    dm = dict(node.get("data_manager", {}) or {})
    if str(args.data_base_dir or "").strip():
        dm["enabled"] = True
        dm["base_dir"] = str(args.data_base_dir)
    if str(args.data_run_id or "").strip():
        dm["enabled"] = True
        dm["run_id"] = str(args.data_run_id)
    if str(args.data_persist_raw_messages) == "on":
        dm["enabled"] = True
        dm["persist_raw_messages"] = True
    elif str(args.data_persist_raw_messages) == "off":
        dm["enabled"] = True
        dm["persist_raw_messages"] = False
    if dm:
        node["data_manager"] = dm
        cfg["node"] = node
    topic_ns = str(args.topic_namespace or "").strip().strip("/")
    if topic_ns:
        comm = dict(node.get("communication", {}) or {})
        comm["topic_namespace"] = topic_ns
        node["communication"] = comm
        cfg["node"] = node
    if (
        str(args.intra_mqtt_host or "").strip()
        or int(args.intra_mqtt_port or 0) > 0
        or str(args.federation_mqtt_host or "").strip()
        or int(args.federation_mqtt_port or 0) > 0
    ):
        comm = dict(node.get("communication", {}) or {})
        if str(args.intra_mqtt_host or "").strip() or int(args.intra_mqtt_port or 0) > 0:
            bus = dict(comm.get("internal_event_bus", {}) or {})
            broker = dict(bus.get("broker", {}) or {})
            if str(args.intra_mqtt_host or "").strip():
                broker["host"] = str(args.intra_mqtt_host).strip()
            if int(args.intra_mqtt_port or 0) > 0:
                broker["port"] = int(args.intra_mqtt_port)
            bus["broker"] = broker
            comm["internal_event_bus"] = bus
        if str(args.federation_mqtt_host or "").strip() or int(args.federation_mqtt_port or 0) > 0:
            bus = dict(comm.get("federation_event_bus", {}) or {})
            broker = dict(bus.get("broker", {}) or {})
            if str(args.federation_mqtt_host or "").strip():
                broker["host"] = str(args.federation_mqtt_host).strip()
            if int(args.federation_mqtt_port or 0) > 0:
                broker["port"] = int(args.federation_mqtt_port)
            bus["broker"] = broker
            comm["federation_event_bus"] = bus
        node["communication"] = comm
        cfg["node"] = node
    fnm = FederationNodeManager(cfg, log_jsonl=str(args.log_jsonl or ""))

    stop_evt = threading.Event()

    def _sig_handler(_sig, _frame):
        stop_evt.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    fnm.start()
    try:
        while not stop_evt.is_set():
            fnm.tick()
            time.sleep(max(0.02, float(args.tick_sec)))
    finally:
        fnm.stop()


if __name__ == "__main__":
    main()
