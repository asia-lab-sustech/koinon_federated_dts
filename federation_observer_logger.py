import argparse
import json
import os
import signal
import time
import uuid
from collections import Counter
from typing import Any, Dict, List, Optional

import paho.mqtt.client as mqtt


def now_ts() -> float:
    return float(time.time())


def json_loads_safe(raw: bytes) -> Dict[str, Any]:
    try:
        return dict(json.loads(raw.decode("utf-8")))
    except Exception:
        return {}


def json_dumps_safe(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def _coalesce(payload: Dict[str, Any], keys: List[str], default: str = "") -> str:
    for key in keys:
        if key in payload:
            v = payload.get(key)
            if v is None:
                continue
            s = str(v).strip()
            if s:
                return s
    return str(default)


def _safe_float(payload: Dict[str, Any], key: str) -> Optional[float]:
    if key not in payload:
        return None
    try:
        return float(payload.get(key))
    except Exception:
        return None


def _topic_event_kind(topic: str) -> str:
    t = str(topic or "")
    if t.startswith("federation/membership/register"):
        return "membership.register.req"
    if t.startswith("federation/membership/heartbeat"):
        return "membership.heartbeat"
    if t.startswith("federation/membership/ack/"):
        return "membership.register.ack"
    if t.startswith("federation/membership/state"):
        return "membership.state"
    if t.startswith("federation/membership/events"):
        return "membership.event"
    if t.startswith("federation/catalog/upsert"):
        return "catalog.upsert"
    if t.startswith("federation/catalog/state"):
        return "catalog.state"
    if t.startswith("federation/catalog/events"):
        return "catalog.event"
    if t.startswith("federation/discovery/query"):
        return "discovery.query"
    if t.startswith("federation/discovery/resp/"):
        return "discovery.response"
    if t.startswith("federation/discovery/events"):
        return "discovery.event"
    if t.startswith("federation/reservation/req/"):
        return "federation.reservation.req"
    if t.startswith("federation/reservation/resp/"):
        return "federation.reservation.resp"
    if t.startswith("federation/handoff/"):
        return "federation.handoff"
    if t.startswith("federation/corridor/advice/"):
        return "corridor.advice"
    if t.startswith("federation/corridor/verdict/"):
        return "corridor.verdict"
    if t.startswith("federation/corridor/state/"):
        return "corridor.state"
    if t.startswith("federation/corridor/route_advice/"):
        return "corridor.route_advice"
    if t.startswith("federation/core/metrics"):
        return "core.metrics"
    if t.startswith("federation/core/audit"):
        return "core.audit"
    if t.startswith("rw/vehicle/") and t.endswith("/route_advice"):
        return "vehicle.route_advice.local"
    if t.startswith("rw/vehicle_agent/") and t.endswith("/route_advice"):
        return "vehicle_agent.route_advice.local"
    if t.startswith("rw/agent/") and t.endswith("/plan"):
        return "intersection.plan"
    if t.startswith("rw/agent/") and t.endswith("/warmup_plan"):
        return "intersection.warmup_plan"
    if t.startswith("rw/evaluation/state"):
        return "simulation.evaluation_state"
    if t.startswith("cmd/"):
        return "command"
    if t.startswith("rw/"):
        return "real_world"
    if t.startswith("ers/"):
        return "ers"
    return "unknown"


def _topic_tail(topic: str) -> str:
    parts = str(topic or "").split("/")
    if not parts:
        return ""
    return str(parts[-1])


def _topic_field(topic: str, prefix: str) -> str:
    t = str(topic or "")
    p = str(prefix or "")
    if not t.startswith(p):
        return ""
    return _topic_tail(t)


class JsonlFile:
    def __init__(self, path: str, reset: bool):
        self.path = os.path.abspath(str(path))
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        mode = "w" if bool(reset) else "a"
        self.fp = open(self.path, mode, encoding="utf-8")

    def write(self, row: Dict[str, Any]) -> None:
        self.fp.write(json_dumps_safe(dict(row or {})) + "\n")
        self.fp.flush()

    def close(self) -> None:
        try:
            self.fp.close()
        except Exception:
            pass


class FederationObserverLogger:
    def __init__(self, args):
        self.args = args
        self.instance = str(args.client_id or f"federation-observer-{uuid.uuid4().hex[:8]}")
        self.out = JsonlFile(path=str(args.output), reset=bool(args.reset))
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.instance)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.stop_flag = {"stop": False}
        self.topics = [str(t) for t in list(args.topics or []) if str(t).strip()]
        self.event_ctr = Counter()
        self.topic_ctr = Counter()
        self.total_msgs = 0
        self.last_summary_ts = now_ts()

    def _emit_meta(self, event: str, **kw: Any) -> None:
        row = {
            "ts": now_ts(),
            "observer": self.instance,
            "event": str(event),
        }
        row.update(kw)
        self.out.write(row)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        for topic in self.topics:
            client.subscribe(topic, qos=int(self.args.qos))
        self._emit_meta(
            "observer_connected",
            mqtt_host=str(self.args.mqtt_host),
            mqtt_port=int(self.args.mqtt_port),
            n_topics=len(self.topics),
            topics=list(self.topics),
            rc=str(reason_code),
        )

    def _on_disconnect(self, _client, _userdata, reason_code, _properties):
        self._emit_meta("observer_disconnected", rc=str(reason_code))

    def _on_message(self, _client, _userdata, msg):
        t_wall = now_ts()
        raw = bytes(msg.payload or b"")
        payload = json_loads_safe(raw)
        topic = str(msg.topic or "")
        is_json = bool(payload)
        event_kind = _topic_event_kind(topic)
        req_id = _coalesce(payload, ["req_id", "request_id", "message_id"])
        ev_id = _coalesce(payload, ["ev_id", "vehId"])
        tls_id = _coalesce(payload, ["tls_id", "tlsId"])
        from_tls = _coalesce(payload, ["from_tls"])
        to_tls = _coalesce(payload, ["to_tls"])
        corridor_id = _coalesce(payload, ["corridor_id"])
        mode = _coalesce(payload, ["mode", "coordinator_mode"])
        gateway_id = _coalesce(payload, ["gateway_id"])
        node_id = _coalesce(payload, ["node_id"])
        trace_id = _coalesce(payload, ["trace_id"])

        if not tls_id:
            # Pull useful IDs from topic suffix when payload does not include them.
            tls_id = _topic_field(topic, "federation/reservation/req/") or tls_id
            tls_id = _topic_field(topic, "federation/reservation/resp/") or tls_id
            tls_id = _topic_field(topic, "federation/handoff/") or tls_id
            tls_id = _topic_field(topic, "federation/corridor/advice/") or tls_id
            tls_id = _topic_field(topic, "federation/corridor/verdict/") or tls_id

        if not corridor_id:
            corridor_id = _topic_field(topic, "federation/corridor/state/") or corridor_id

        if not ev_id:
            ev_id = _topic_field(topic, "federation/corridor/route_advice/") or ev_id
            ev_id = _topic_field(topic, "rw/vehicle/") if topic.endswith("/route_advice") else ev_id
            if topic.startswith("rw/vehicle_agent/") and topic.endswith("/route_advice"):
                parts = topic.split("/")
                if len(parts) >= 3:
                    ev_id = str(parts[2] or ev_id)

        latency_ms = None
        for lk in ("latency_ms", "onboarding_latency_ms", "catalog_upsert_latency_ms", "discovery_latency_ms"):
            latency_ms = _safe_float(payload, lk)
            if latency_ms is not None:
                break

        row: Dict[str, Any] = {
            "ts": t_wall,
            "observer": self.instance,
            "event": "trace",
            "event_kind": event_kind,
            "topic": topic,
            "qos": int(msg.qos),
            "retain": bool(msg.retain),
            "payload_bytes": len(raw),
            "payload_is_json": bool(is_json),
            "event_name": _coalesce(payload, ["event", "msg_type", "type"]),
            "req_id": req_id,
            "trace_id": trace_id,
            "ev_id": ev_id,
            "tls_id": tls_id,
            "from_tls": from_tls,
            "to_tls": to_tls,
            "corridor_id": corridor_id,
            "mode": mode,
            "gateway_id": gateway_id,
            "node_id": node_id,
            "n_results": int(payload.get("n_results", 0) or 0) if is_json else 0,
            "status": _coalesce(payload, ["status", "verdict"]),
            "latency_ms": latency_ms,
        }

        if bool(self.args.include_payload):
            if is_json:
                row["payload"] = payload
            else:
                txt = raw.decode("utf-8", errors="replace")
                max_chars = max(16, int(self.args.payload_preview_chars))
                row["payload_preview"] = txt[:max_chars]

        self.out.write(row)

        self.total_msgs += 1
        self.event_ctr[event_kind] += 1
        self.topic_ctr[topic] += 1

        now = now_ts()
        if float(self.args.summary_sec) > 0 and (now - self.last_summary_ts) >= float(self.args.summary_sec):
            self.last_summary_ts = now
            top_events = self.event_ctr.most_common(8)
            summary = {
                "ts": now,
                "observer": self.instance,
                "event": "summary",
                "total_msgs": int(self.total_msgs),
                "unique_topics": int(len(self.topic_ctr)),
                "top_event_kinds": [{"event_kind": str(k), "count": int(v)} for k, v in top_events],
            }
            self.out.write(summary)

    def start(self) -> None:
        self.client.connect(str(self.args.mqtt_host), int(self.args.mqtt_port), 60)
        self.client.loop_start()
        self._emit_meta("observer_start", output=os.path.abspath(str(self.args.output)))

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self._emit_meta(
            "observer_stop",
            total_msgs=int(self.total_msgs),
            unique_topics=int(len(self.topic_ctr)),
            top_event_kinds=[{"event_kind": str(k), "count": int(v)} for k, v in self.event_ctr.most_common(12)],
        )
        self.out.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Observe federation/runtime MQTT topics and emit unified event trace JSONL")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument(
        "--topics",
        nargs="+",
        default=[
            "federation/#",
            "rw/vehicle/+/route_advice",
            "rw/vehicle_agent/+/route_advice",
            "rw/agent/+/plan",
            "rw/agent/+/warmup_plan",
            "rw/evaluation/state",
        ],
        help="MQTT topic filters to observe",
    )
    ap.add_argument("--qos", type=int, default=0)
    ap.add_argument("--output", default="./tmp/event_trace.jsonl")
    ap.add_argument("--reset", action="store_true", default=False, help="truncate output file at startup")
    ap.add_argument("--summary-sec", type=float, default=5.0, help="periodic summary emission interval in seconds (<=0 disables)")
    ap.add_argument("--include-payload", action="store_true", default=False, help="include full payload for JSON messages")
    ap.add_argument("--payload-preview-chars", type=int, default=240, help="preview length for non-JSON payloads")
    ap.add_argument("--client-id", default="")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    obs = FederationObserverLogger(args)
    stop_flag = {"stop": False}

    def _stop(_sig, _frm):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    obs.start()
    try:
        while not stop_flag["stop"]:
            time.sleep(0.2)
    finally:
        obs.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
