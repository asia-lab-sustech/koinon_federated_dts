import argparse
import signal
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts


@dataclass
class CatalogEntry:
    key: str
    gateway_id: str
    node_id: str
    role: str
    service_name: str
    direction: str
    event_type: str
    publish_topic: str
    subscribe_topic: str
    updated_ts: float


class CatalogService:
    def __init__(self, args):
        self.args = args
        self.instance = f"catalog-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.instance)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.entries: Dict[str, CatalogEntry] = {}
        self.member_status: Dict[str, str] = {}
        self.last_state_pub = 0.0

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "catalog", "instance": self.instance, "event": event}
        row.update(kw)
        self.log.write(row)

    def _pub(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(str(topic), json_dumps(payload), qos=0, retain=False)

    def _publish_state(self) -> None:
        payload = {
            "schema": "federation.catalog.v1",
            "event": "state",
            "service": "catalog",
            "n_entries": len(self.entries),
            "entries": [asdict(e) for e in self.entries.values()],
            "ts": now_ts(),
        }
        self._pub(self.args.state_topic, payload)

    def _publish_event(self, event: str, payload: Dict[str, Any]) -> None:
        out = {"schema": "federation.catalog.v1", "event": str(event), "service": "catalog", "ts": now_ts()}
        out.update(payload)
        self._pub(self.args.events_topic, out)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        client.subscribe(self.args.upsert_topic, qos=0)
        client.subscribe(self.args.membership_state_topic, qos=0)
        client.subscribe(self.args.membership_events_topic, qos=0)
        self._emit("connected", host=self.args.mqtt_host, rc=str(reason_code))

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        t0 = now_ts()
        if topic == self.args.upsert_topic:
            self._handle_upsert(payload, t0)
            return
        if topic == self.args.membership_state_topic:
            self._handle_membership_state(payload)
            return
        if topic == self.args.membership_events_topic:
            self._handle_membership_event(payload)
            return

    def _handle_membership_state(self, payload: Dict[str, Any]) -> None:
        for m in list(payload.get("members", []) or []):
            gid = str(m.get("gateway_id", "") or "")
            status = str(m.get("status", "") or "")
            if gid:
                self.member_status[gid] = status

    def _handle_membership_event(self, payload: Dict[str, Any]) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        status = str(payload.get("status", "") or "")
        if gid and status:
            self.member_status[gid] = status

    def _handle_upsert(self, payload: Dict[str, Any], t0: float) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "") or "")
        services = list(payload.get("services", []) or [])

        if not gid or not node_id:
            self._emit("catalog_upsert_reject", reason="missing_ids")
            return

        member_status = self.member_status.get(gid, "")
        if self.args.require_registered_member and member_status and member_status != "REGISTERED":
            self._emit("catalog_upsert_reject", reason="member_not_registered", gateway_id=gid, status=member_status)
            return

        n = 0
        for s in services:
            name = str(s.get("name", "") or "")
            if not name:
                continue
            key = f"{gid}:{name}"
            ent = CatalogEntry(
                key=key,
                gateway_id=gid,
                node_id=node_id,
                role=role,
                service_name=name,
                direction=str(s.get("direction", "")),
                event_type=str(s.get("event_type", "")),
                publish_topic=str(s.get("publish_topic", "")),
                subscribe_topic=str(s.get("subscribe_topic", "")),
                updated_ts=now_ts(),
            )
            self.entries[key] = ent
            n += 1

        lat_ms = round((now_ts() - t0) * 1000.0, 3)
        self._emit("catalog_upsert", gateway_id=gid, node_id=node_id, n_services=n, latency_ms=lat_ms)
        self._publish_event("catalog_upsert", {
            "gateway_id": gid,
            "node_id": node_id,
            "n_services": n,
            "latency_ms": lat_ms,
        })

    def tick(self) -> None:
        t = now_ts()
        if (t - self.last_state_pub) >= max(0.5, float(self.args.state_interval_sec)):
            self.last_state_pub = t
            self._publish_state()

    def start(self) -> None:
        self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host, upsert_topic=self.args.upsert_topic)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation Catalog Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--upsert-topic", default="federation/catalog/upsert")
    ap.add_argument("--state-topic", default="federation/catalog/state")
    ap.add_argument("--events-topic", default="federation/catalog/events")
    ap.add_argument("--membership-state-topic", default="federation/membership/state")
    ap.add_argument("--membership-events-topic", default="federation/membership/events")
    ap.add_argument("--require-registered-member", action="store_true", default=False)
    ap.add_argument("--state-interval-sec", type=float, default=2.0)
    ap.add_argument("--log-jsonl", default="")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    svc = CatalogService(args)
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
