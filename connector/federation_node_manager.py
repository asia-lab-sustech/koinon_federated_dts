#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib import error as url_error
from urllib import request as url_request

import paho.mqtt.client as mqtt
import yaml

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
        return {
            "ev_id": str(
                snap.get("evId")
                or snap.get("ev_id")
                or state.get("evId")
                or state.get("ev_id")
                or ""
            ),
            "sim_time": _as_float(snap.get("simTime", snap.get("sim_time", state.get("simTime", _now()))), _now()),
            "speed_mps": _as_float(snap.get("speedMps", snap.get("speed_mps", 0.0)), 0.0),
            "in_edge_id": str(snap.get("edgeId", snap.get("edge_id", "")) or ""),
            "stopline_dist_m": _as_float(
                snap.get("distToStoplineM", snap.get("dist_to_stopline_m", 1e9)),
                1e9,
            ),
            "next_tls": list(snap.get("nextTls", snap.get("next_tls", [])) or []),
            "route_veh": list(snap.get("routeEdges", snap.get("route_veh", [])) or []),
            "route_intersections": list(
                state.get("route_intersections", state.get("routeIntersections", [])) or []
            ),
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
        self.topic_namespace = str(topic_namespace or "").strip().strip("/")
        self._ns_prefix = f"{self.topic_namespace}/" if self.topic_namespace else ""
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=str(client_id))
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.on_message_cb = on_message
        self.logger = logger
        self.subscriptions: List[str] = []

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

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        self.logger.write("fnm.mqtt.connected", iface=self.name, host=self.host, port=self.port, rc=str(reason_code))
        for t in self.subscriptions:
            try:
                sub_t = self._tx_topic(str(t))
                client.subscribe(sub_t, qos=0)
                self.logger.write("fnm.mqtt.subscribed", iface=self.name, topic=str(t), wire_topic=sub_t)
            except Exception as e:
                self.logger.write("fnm.mqtt.subscribe_error", iface=self.name, topic=str(t), err=f"{type(e).__name__}:{e}")

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
            self.on_message_cb(str(topic), payload)
        except Exception as e:
            self.logger.write("fnm.mqtt.callback_error", iface=self.name, topic=str(msg.topic), err=f"{type(e).__name__}:{e}")

    def start(self, subscriptions: Iterable[str]) -> None:
        self.subscriptions = [str(t) for t in list(subscriptions or []) if str(t)]
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(self._tx_topic(str(topic)), _json_dumps(dict(payload or {})), qos=0, retain=False)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
        except Exception:
            pass
        try:
            self.client.disconnect()
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
        ev_translation: EVRequestTranslation,
        state_manager: StateManager,
        event_manager: EventManager,
        fed_iface: MQTTInterface,
        dt_id: str,
        logger: TraceLogger,
        emit_state_trace: bool = False,
        error_backoff_max_sec: float = 2.0,
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
        self.ev_translation = ev_translation
        self.state_manager = state_manager
        self.event_manager = event_manager
        self.fed_iface = fed_iface
        self.dt_id = str(dt_id)
        self.logger = logger
        self._next_pull_ts = 0.0
        self.emit_state_trace = bool(emit_state_trace)
        self.error_backoff_max_sec = max(0.2, float(error_backoff_max_sec))
        self._err_streak = 0

    def tick(self, now: float) -> None:
        if not self.enabled or not self.state_pull_url:
            return
        if float(now) < float(self._next_pull_ts):
            return
        self._next_pull_ts = float(now) + float(self.state_pull_sec)

        req = url_request.Request(self.state_pull_url, method="GET")
        try:
            with url_request.urlopen(req, timeout=float(self.state_pull_timeout_sec)) as resp:
                raw = resp.read()
            payload = dict(json.loads(raw.decode("utf-8")))
        except url_error.URLError as e:
            self._err_streak += 1
            backoff = min(
                float(self.error_backoff_max_sec),
                float(self.state_pull_sec) * (2 ** min(int(self._err_streak), 4)),
            )
            self._next_pull_ts = float(now) + float(backoff)
            self.logger.write("fnm.adapter.state_pull.error", dt_id=self.dt_id, url=self.state_pull_url, err=f"{type(e).__name__}:{e}")
            return
        except Exception as e:
            self._err_streak += 1
            backoff = min(
                float(self.error_backoff_max_sec),
                float(self.state_pull_sec) * (2 ** min(int(self._err_streak), 4)),
            )
            self._next_pull_ts = float(now) + float(backoff)
            self.logger.write("fnm.adapter.state_pull.error", dt_id=self.dt_id, url=self.state_pull_url, err=f"{type(e).__name__}:{e}")
            return

        self._err_streak = 0
        state = self.ev_translation.normalize_ev_state(payload)
        ev_id = str(state.get("ev_id", "") or self.dt_id)
        self.state_manager.set(f"ev_state:{ev_id}", state)
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

        reqs = self.ev_translation.ev_state_to_requests(
            state,
            max_next_tls=self.max_next_tls,
            source_service=self.source_service,
            source_tag=self.source_tag,
            default_delta_sec=self.default_delta_sec,
        )
        pub_n = 0
        for tls_id, req_msg in reqs:
            topic = f"{self.ev_request_topic_prefix}/{tls_id}"
            self.fed_iface.publish(topic, req_msg)
            pub_n += 1
            self.logger.write(
                "fnm.adapter.ev_request.publish",
                dt_id=self.dt_id,
                ev_id=ev_id,
                tls_id=str(tls_id),
                topic=str(topic),
                req_kind=str(req_msg.get("request_kind", "")),
                distance_to_intersection_m=float(req_msg.get("distance_to_intersection_m", 1e9)),
                sim_time=float(req_msg.get("sim_time", 0.0)),
            )
        self.logger.write(
            "fnm.adapter.state_pull.ok",
            dt_id=self.dt_id,
            ev_id=ev_id,
            req_published=pub_n,
            nearest_tls=(reqs[0][0] if reqs else ""),
            state_pull_sec=self.state_pull_sec,
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

    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        self.endpoint.publish(topic, payload)

    def stop(self) -> None:
        self.endpoint.stop()


class FederationInterface:
    def __init__(self, endpoint: MQTTInterface) -> None:
        self.endpoint = endpoint

    def start(self, topics: Iterable[str]) -> None:
        self.endpoint.start(topics)

    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        self.endpoint.publish(topic, payload)

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
        self.state_manager = StateManager()
        self.event_manager = EventManager()
        self.translator = SchemaTranslation()
        self.ev_translation = EVRequestTranslation()
        self.stop_evt = threading.Event()

        self.rules = self._load_rules()
        self.monitor_rules = self._load_monitor_rules()
        self.topic_namespace = str(
            _deep_get(self.cfg, ["node", "communication", "topic_namespace"], "")
            or _deep_get(self.cfg, ["node", "topic_namespace"], "")
            or ""
        ).strip().strip("/")

        in_host = str(_deep_get(self.cfg, ["node", "communication", "internal_event_bus", "broker", "host"], "127.0.0.1"))
        in_port = int(_deep_get(self.cfg, ["node", "communication", "internal_event_bus", "broker", "port"], 1883))
        fed_host = str(_deep_get(self.cfg, ["node", "communication", "federation_event_bus", "broker", "host"], "127.0.0.1"))
        fed_port = int(_deep_get(self.cfg, ["node", "communication", "federation_event_bus", "broker", "port"], 1883))

        self._inbound_ep = MQTTInterface(
            name="intra",
            host=in_host,
            port=in_port,
            topic_namespace=self.topic_namespace,
            client_id=f"{self.gateway_id}-in",
            on_message=self._on_local_message,
            logger=self.logger,
        )
        self._feder_ep = MQTTInterface(
            name="federation",
            host=fed_host,
            port=fed_port,
            topic_namespace=self.topic_namespace,
            client_id=f"{self.gateway_id}-fed",
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
                    ev_translation=self.ev_translation,
                    state_manager=self.state_manager,
                    event_manager=self.event_manager,
                    fed_iface=self._feder_ep,
                    dt_id=self.dt_id,
                    logger=self.logger,
                    emit_state_trace=bool(adapter_cfg.get("emit_state_trace", False)),
                    error_backoff_max_sec=_as_float(adapter_cfg.get("error_backoff_max_sec", 2.0), 2.0),
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

    def _on_local_message(self, topic: str, payload: Dict[str, Any]) -> None:
        local_in_ts = _now()
        local_payload = dict(payload or {})
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
            try:
                self.federation.publish(rule.publish_topic, out)
            except Exception as e:
                self.event_manager.inc(f"delivery_local_to_fed:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.local_to_fed",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="publish_error",
                    src=topic,
                    dst=rule.publish_topic,
                    message_id=msg_id,
                    err=f"{type(e).__name__}:{e}",
                )
                continue
            self.event_manager.inc("local_to_fed")
            self.event_manager.inc(f"delivery_local_to_fed:{artefact_kind}:ok")
            self.logger.write(
                "fnm.route.local_to_fed",
                rule=rule.name,
                src=topic,
                dst=rule.publish_topic,
                message_id=msg_id,
                artefact_kind=artefact_kind,
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
            )
            self.logger.write(
                "fnm.delivery.local_to_fed",
                rule=rule.name,
                artefact_kind=artefact_kind,
                status="ok",
                src=topic,
                dst=rule.publish_topic,
                message_id=msg_id,
            )
            self.data_manager.write_raw_message(direction="fed_out", topic=rule.publish_topic, payload=out)

    def _on_federation_message(self, topic: str, payload: Dict[str, Any]) -> None:
        fed_in_ts = _now()
        fed_payload = dict(payload or {})
        self.data_manager.write_raw_message(direction="fed_in", topic=topic, payload=fed_payload)
        self._apply_monitor_rules("federation", topic, fed_payload)
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
            try:
                self.intra.publish(rule.publish_topic, out)
            except Exception as e:
                self.event_manager.inc(f"delivery_fed_to_local:{artefact_kind}:error")
                self.logger.write(
                    "fnm.delivery.fed_to_local",
                    rule=rule.name,
                    artefact_kind=artefact_kind,
                    status="publish_error",
                    src=topic,
                    dst=rule.publish_topic,
                    message_id=msg_id,
                    err=f"{type(e).__name__}:{e}",
                )
                continue
            self.event_manager.inc("fed_to_local")
            self.event_manager.inc(f"delivery_fed_to_local:{artefact_kind}:ok")
            self.logger.write(
                "fnm.route.fed_to_local",
                rule=rule.name,
                src=topic,
                dst=rule.publish_topic,
                message_id=msg_id,
                artefact_kind=artefact_kind,
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
            )
            self.logger.write(
                "fnm.delivery.fed_to_local",
                rule=rule.name,
                artefact_kind=artefact_kind,
                status="ok",
                src=topic,
                dst=rule.publish_topic,
                message_id=msg_id,
                before_expiry=before_expiry,
                expiry_delta_ms=expiry_delta_ms,
                expiry_ref=expiry_ref,
            )
            self.data_manager.write_raw_message(direction="local_out", topic=rule.publish_topic, payload=out)

    def _publish_register(self) -> None:
        payload = {
            "schema": "federation.membership.v1",
            "event": "register",
            "request_id": _new_id("reg"),
            "gateway_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "domain": str(_deep_get(self.cfg, ["node", "domain"], "traffic")),
            "capabilities": list(_deep_get(self.cfg, ["node", "capabilities"], []) or []),
            "ts": _now(),
        }
        self.federation.publish(self.membership_register_topic, payload)
        self.logger.write("fnm.membership.register_pub", topic=self.membership_register_topic)

    def _publish_heartbeat(self) -> None:
        payload = {
            "schema": "federation.membership.v1",
            "event": "heartbeat",
            "gateway_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "ts": _now(),
        }
        self.federation.publish(self.membership_heartbeat_topic, payload)

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
        payload = {
            "schema": "federation.catalog.v1",
            "event": "upsert",
            "gateway_id": self.gateway_id,
            "node_id": self.dt_id,
            "role": self.dt_type,
            "services": services,
            "dt_profile": dict(_deep_get(self.cfg, ["node", "dt_profile"], {}) or {}),
            "ts": _now(),
        }
        self.federation.publish(self.catalog_upsert_topic, payload)

    def start(self) -> None:
        in_topics = [r.subscribe_topic for r in self.rules if r.direction == "local_to_fed"]
        fed_topics = [r.subscribe_topic for r in self.rules if r.direction == "fed_to_local"]
        for mr in self.monitor_rules:
            if mr.source in {"local", "any"}:
                in_topics.append(mr.subscribe_topic)
            if mr.source in {"federation", "any"}:
                fed_topics.append(mr.subscribe_topic)
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
            topic_namespace=self.topic_namespace,
        )
        self.intra.start(in_topics)
        self.federation.start(fed_topics)
        time.sleep(0.5)
        self._publish_register()
        self._publish_catalog()
        self._last_catalog = _now()
        self._last_heartbeat = _now()

    def tick(self) -> None:
        now = _now()
        self.protocol_adaptation.tick(now)
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
