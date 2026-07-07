import argparse
import signal
import time
from typing import Any, Dict

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, short_mqtt_client_id, make_mqtt_client, topic_match_namespace, topic_with_namespace


class LifecycleHealthService:
    """
    Health/lifecycle monitor:
    - Consumes DT keepalives (heartbeats)
    - Emits lifecycle availability events
    - Detects timeout and emits unavailable transitions
    """

    def __init__(self, args):
        self.args = args
        self.instance = f"lifecycle-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.mqtt_client_id = short_mqtt_client_id("life", self.instance)
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass

        self.last_seen_ts: Dict[str, float] = {}
        self.last_state: Dict[str, str] = {}  # alive | unavailable
        self.gateway_to_node: Dict[str, str] = {}
        self.last_state_pub = 0.0
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "exact") or "exact").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "lifecycle", "instance": self.instance, "event": event}
        availability = str(kw.get("availability", "") or "").lower().strip()
        if availability:
            row.update(
                self._runtime_lifecycle_trace(
                    availability,
                    previous_availability=str(kw.get("previous_availability", "") or ""),
                )
            )
        row.update(kw)
        self.log.write(row)

    @staticmethod
    def _runtime_lifecycle_trace(availability: str, previous_availability: str = "") -> Dict[str, Any]:
        status = str(availability or "").lower().strip()
        prev = str(previous_availability or "").lower().strip()
        mapping = {
            "alive": ("runtime_connected_ready", "active_runtime_operations"),
            "up": ("runtime_connected_ready", "active_runtime_operations"),
            "healthy": ("runtime_connected_ready", "active_runtime_operations"),
            "available": ("runtime_connected_ready", "active_runtime_operations"),
            "unavailable": ("runtime_unavailable", "runtime_disruption"),
            "down": ("runtime_unavailable", "runtime_disruption"),
            "expired": ("runtime_unavailable", "runtime_disruption"),
            "stale": ("runtime_unavailable", "runtime_disruption"),
        }
        state, phase = mapping.get(status, ("runtime_status_observed", "runtime_observation"))
        trace = {
            "lifecycle_trace_only": 1,
            "lifecycle_model": "runtime_lifecycle",
            "implemented_availability": status,
            "paper_lifecycle_state": state,
            "paper_lifecycle_phase": phase,
        }
        if prev:
            prev_state, _prev_phase = mapping.get(prev, ("runtime_status_observed", "runtime_observation"))
            trace["previous_implemented_availability"] = prev
            trace["paper_lifecycle_transition"] = f"{prev_state}->{state}" if prev_state != state else state
        return trace

    @staticmethod
    def _latency_ms(t0: float | None) -> float | None:
        if t0 is None:
            return None
        try:
            return round((now_ts() - float(t0)) * 1000.0, 3)
        except Exception:
            return None

    def _pub(self, topic: str, payload: Dict[str, Any], namespace: str = "") -> None:
        t = topic_with_namespace(str(topic), str(namespace or ""))
        self.client.publish(str(t), json_dumps(payload), qos=0, retain=False)

    def _match_ns(self, topic: str, base_topic: str):
        return topic_match_namespace(str(topic), str(base_topic), mode=self.topic_match_mode)

    def _remember_ns(self, ns: str) -> None:
        x = str(ns or "").strip().strip("/")
        if x:
            self.observed_namespaces.add(x)

    def _publish_across_namespaces(self, topic: str, payload: Dict[str, Any]) -> None:
        if self.topic_match_mode == "suffix" and self.observed_namespaces:
            for ns in sorted(self.observed_namespaces):
                self._pub(topic, payload, namespace=ns)
            return
        self._pub(topic, payload)

    def _publish_lifecycle(
        self,
        gateway_id: str,
        availability: str,
        source: str,
        transition: bool,
        namespace: str = "",
        *,
        t0: float | None = None,
        latency_scope: str = "",
        previous_availability: str = "",
    ) -> None:
        latency_ms = self._latency_ms(t0)
        payload = {
            "schema": "federation.lifecycle.v1",
            "event": "health_status",
            "gateway_id": str(gateway_id),
            "node_id": str(self.gateway_to_node.get(str(gateway_id), "") or ""),
            "availability": str(availability),
            "source": str(source),
            "transition": int(bool(transition)),
            "last_seen_ts": float(self.last_seen_ts.get(str(gateway_id), now_ts()) or now_ts()),
            "ts": now_ts(),
        }
        self._pub(self.args.lifecycle_events_topic, payload, namespace=namespace)
        self._emit(
            "lifecycle_health_pub",
            gateway_id=gateway_id,
            availability=availability,
            source=source,
            transition=int(bool(transition)),
            topic_namespace=str(namespace or "-") or "-",
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope=str(latency_scope or "lifecycle_trigger_to_health_publish") if latency_ms is not None else "",
            previous_availability=str(previous_availability or ""),
        )

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            client.subscribe(self.args.heartbeat_topic, qos=0)
            client.subscribe(self.args.register_topic, qos=0)
            client.subscribe(self.args.membership_state_topic, qos=0)
        self._emit(
            "connected",
            host=self.args.mqtt_host,
            rc=str(reason_code),
            topic_match_mode=self.topic_match_mode,
            mqtt_client_id=self.mqtt_client_id,
        )

    def _on_disconnect(self, _client, _userdata, *args):
        reason_code = args[-2] if len(args) >= 2 else (args[-1] if args else "")
        self._emit("disconnected", host=self.args.mqtt_host, rc=str(reason_code), mqtt_client_id=self.mqtt_client_id)

    def _on_connect_fail(self, _client, _userdata):
        self._emit("connect_fail", host=self.args.mqtt_host, mqtt_client_id=self.mqtt_client_id)

    def _handle_heartbeat(self, payload: Dict[str, Any], namespace: str = "", *, t0: float | None = None) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        if not gid:
            self._emit("heartbeat_reject", reason="missing_gateway_id")
            return
        if node_id:
            self.gateway_to_node[gid] = node_id
        self.last_seen_ts[gid] = now_ts()
        prev = str(self.last_state.get(gid, "") or "")
        self.last_state[gid] = "alive"
        self._publish_lifecycle(
            gid,
            "alive",
            source="heartbeat",
            transition=(prev != "alive"),
            namespace=namespace,
            t0=t0,
            latency_scope="heartbeat_to_lifecycle_publish",
            previous_availability=prev,
        )

    def _handle_register(self, payload: Dict[str, Any], namespace: str = "", *, t0: float | None = None) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        if gid and node_id:
            self.gateway_to_node[gid] = node_id
            latency_ms = self._latency_ms(t0)
            self._emit(
                "member_seen",
                gateway_id=gid,
                node_id=node_id,
                latency_ms=latency_ms if latency_ms is not None else "",
                latency_scope="register_to_lifecycle_index_update" if latency_ms is not None else "",
            )

    def _handle_membership_state(self, payload: Dict[str, Any], namespace: str = "", *, t0: float | None = None) -> None:
        indexed = 0
        for m in list(payload.get("members", []) or []):
            gid = str(m.get("gateway_id", "") or "")
            node_id = str(m.get("node_id", "") or "")
            if gid and node_id:
                self.gateway_to_node[gid] = node_id
                indexed += 1
        latency_ms = self._latency_ms(t0)
        self._emit(
            "lifecycle_membership_state_indexed",
            members_indexed_n=int(indexed),
            gateways_n=int(len(self.gateway_to_node)),
            topic_namespace=str(namespace or "-") or "-",
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope="membership_state_to_lifecycle_index_update" if latency_ms is not None else "",
        )

    def _on_message(self, _client, _userdata, msg):
        t0 = now_ts()
        topic = str(msg.topic)
        payload = json_loads(msg.payload)

        ns = self._match_ns(topic, self.args.heartbeat_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "lifecycle_rx",
                channel="heartbeat",
                topic=str(topic),
                canonical_topic=str(self.args.heartbeat_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
            )
            self._handle_heartbeat(payload, namespace=str(ns or ""), t0=t0)
            return

        ns = self._match_ns(topic, self.args.register_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "lifecycle_rx",
                channel="register",
                topic=str(topic),
                canonical_topic=str(self.args.register_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
            )
            self._handle_register(payload, namespace=str(ns or ""), t0=t0)
            return

        ns = self._match_ns(topic, self.args.membership_state_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "lifecycle_rx",
                channel="membership_state",
                topic=str(topic),
                canonical_topic=str(self.args.membership_state_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_membership_state(payload, namespace=str(ns or ""), t0=t0)
            return

    def _check_timeouts(self) -> None:
        timeout_sec = max(1.0, float(self.args.heartbeat_timeout_sec))
        t = now_ts()
        for gid, seen in list(self.last_seen_ts.items()):
            age = float(t - float(seen))
            prev = str(self.last_state.get(gid, "") or "")
            if age > timeout_sec and prev != "unavailable":
                self.last_state[gid] = "unavailable"
                self._publish_lifecycle(
                    gid,
                    "unavailable",
                    source="timeout",
                    transition=True,
                    t0=t,
                    latency_scope="timeout_detection_to_lifecycle_publish",
                    previous_availability=prev,
                )

    def _publish_state(self, *, t0: float | None = None) -> None:
        latency_ms = self._latency_ms(t0)
        payload = {
            "schema": "federation.lifecycle.v1",
            "event": "state",
            "service": "lifecycle",
            "n_gateways": len(self.last_state),
            "gateways": [
                {
                    "gateway_id": str(gid),
                    "node_id": str(self.gateway_to_node.get(str(gid), "") or ""),
                    "availability": str(st),
                    "last_seen_ts": float(self.last_seen_ts.get(str(gid), 0.0) or 0.0),
                }
                for gid, st in self.last_state.items()
            ],
            "ts": now_ts(),
        }
        self._publish_across_namespaces(self.args.state_topic, payload)
        self._emit(
            "lifecycle_state_pub",
            n_gateways=int(len(self.last_state)),
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope="state_tick_to_lifecycle_state_publish" if latency_ms is not None else "",
        )

    def tick(self) -> None:
        self._check_timeouts()
        t = now_ts()
        if (t - self.last_state_pub) >= max(0.5, float(self.args.state_interval_sec)):
            self.last_state_pub = t
            self._publish_state(t0=t)

    def start(self) -> None:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self._emit("connect_called", mqtt_host=self.args.mqtt_host, mqtt_port=int(self.args.mqtt_port), rc=int(rc), mqtt_client_id=self.mqtt_client_id)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host, heartbeat_topic=self.args.heartbeat_topic)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation Lifecycle/Health Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--heartbeat-topic", default="federation/membership/heartbeat")
    ap.add_argument("--register-topic", default="federation/membership/register")
    ap.add_argument("--membership-state-topic", default="federation/membership/state")
    ap.add_argument("--lifecycle-events-topic", default="federation/lifecycle/events")
    ap.add_argument("--state-topic", default="federation/lifecycle/state")
    ap.add_argument("--heartbeat-timeout-sec", type=float, default=30.0)
    ap.add_argument("--state-interval-sec", type=float, default=1.0)
    ap.add_argument(
        "--topic-match-mode",
        choices=["exact", "suffix"],
        default="exact",
        help="exact=legacy topics; suffix=match namespaced topics ending with canonical federation topics",
    )
    ap.add_argument(
        "--topic-subscribe-wildcard",
        default="#",
        help="MQTT subscription used when --topic-match-mode suffix is enabled",
    )
    ap.add_argument("--log-jsonl", default="")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    svc = LifecycleHealthService(args)
    stop_flag = {"stop": False}

    def _stop(_sig, _frm):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    svc.start()
    try:
        while not stop_flag["stop"]:
            svc.tick()
            time.sleep(0.5)
    finally:
        svc.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
