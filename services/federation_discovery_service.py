import argparse
import signal
import time
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts


class DiscoveryService:
    def __init__(self, args):
        self.args = args
        self.instance = f"discovery-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.instance)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        self.catalog_entries: Dict[str, Dict[str, Any]] = {}
        self.member_status: Dict[str, str] = {}

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "discovery", "instance": self.instance, "event": event}
        row.update(kw)
        self.log.write(row)

    def _pub(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(str(topic), json_dumps(payload), qos=0, retain=False)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        client.subscribe(self.args.query_topic, qos=0)
        client.subscribe(self.args.catalog_upsert_topic, qos=0)
        client.subscribe(self.args.catalog_state_topic, qos=0)
        client.subscribe(self.args.membership_state_topic, qos=0)
        client.subscribe(self.args.membership_events_topic, qos=0)
        self._emit("connected", host=self.args.mqtt_host, rc=str(reason_code))

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        if topic == self.args.query_topic:
            self._handle_query(payload)
            return
        if topic == self.args.catalog_upsert_topic:
            self._handle_catalog_upsert(payload)
            return
        if topic == self.args.catalog_state_topic:
            self._handle_catalog_state(payload)
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

    def _handle_catalog_upsert(self, payload: Dict[str, Any]) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "") or "")
        for svc in list(payload.get("services", []) or []):
            name = str(svc.get("name", "") or "")
            if not name:
                continue
            key = f"{gid}:{name}"
            self.catalog_entries[key] = {
                "node_id": node_id,
                "gateway_id": gid,
                "role": role,
                "service_name": name,
                "direction": str(svc.get("direction", "")),
                "event_type": str(svc.get("event_type", "")),
                "publish_topic": str(svc.get("publish_topic", "")),
                "subscribe_topic": str(svc.get("subscribe_topic", "")),
                "updated_ts": now_ts(),
            }

    def _handle_catalog_state(self, payload: Dict[str, Any]) -> None:
        for ent in list(payload.get("entries", []) or []):
            key = str(ent.get("key", "") or "")
            if not key:
                continue
            self.catalog_entries[key] = dict(ent)

    def _allowed_for_requester(self, requester_role: str, ent: Dict[str, Any]) -> bool:
        # Lightweight policy placeholder: observers can only see local_to_fed resources.
        if str(requester_role) == "observer" and str(ent.get("direction", "")) != "local_to_fed":
            return False
        return True

    def _is_member_active(self, gateway_id: str) -> bool:
        status = str(self.member_status.get(str(gateway_id), "REGISTERED") or "REGISTERED")
        return status == "REGISTERED"

    def _handle_query(self, payload: Dict[str, Any]) -> None:
        t0 = now_ts()
        request_id = str(payload.get("request_id", ""))
        requester = str(payload.get("requester", "") or "")
        requester_role = str(payload.get("requester_role", "") or "")
        reply_topic = str(payload.get("reply_topic", f"{self.args.reply_prefix}/{requester or 'unknown'}"))
        filters = dict(payload.get("filters", {}) or {})
        max_results = int(payload.get("max_results", 50) or 50)

        role_filter = str(filters.get("role", "") or "")
        event_filter = str(filters.get("event_type", "") or "")
        service_filter = str(filters.get("service_name", "") or "")

        out: List[Dict[str, Any]] = []
        for ent in self.catalog_entries.values():
            if role_filter and str(ent.get("role", "")) != role_filter:
                continue
            if event_filter and str(ent.get("event_type", "")) != event_filter:
                continue
            if service_filter and str(ent.get("service_name", "")) != service_filter:
                continue
            if self.args.only_active_members and not self._is_member_active(str(ent.get("gateway_id", ""))):
                continue
            if not self._allowed_for_requester(requester_role, ent):
                continue
            out.append(dict(ent))
            if len(out) >= max_results:
                break

        lat_ms = round((now_ts() - t0) * 1000.0, 3)
        resp = {
            "schema": "federation.discovery.v1",
            "event": "query_resp",
            "request_id": request_id,
            "requester": requester,
            "requester_role": requester_role,
            "n_results": len(out),
            "results": out,
            "latency_ms": lat_ms,
            "ts": now_ts(),
        }
        self._pub(reply_topic, resp)
        self._pub(self.args.events_topic, {
            "schema": "federation.discovery.v1",
            "event": "query_resp",
            "request_id": request_id,
            "requester": requester,
            "n_results": len(out),
            "latency_ms": lat_ms,
            "ts": now_ts(),
        })
        self._emit("discovery_query_resp", request_id=request_id, requester=requester, n_results=len(out), latency_ms=lat_ms)

    def start(self) -> None:
        self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host, query_topic=self.args.query_topic)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation Discovery Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--query-topic", default="federation/discovery/query")
    ap.add_argument("--reply-prefix", default="federation/discovery/resp")
    ap.add_argument("--events-topic", default="federation/discovery/events")
    ap.add_argument("--catalog-upsert-topic", default="federation/catalog/upsert")
    ap.add_argument("--catalog-state-topic", default="federation/catalog/state")
    ap.add_argument("--membership-state-topic", default="federation/membership/state")
    ap.add_argument("--membership-events-topic", default="federation/membership/events")
    ap.add_argument("--only-active-members", action="store_true", default=True)
    ap.add_argument("--log-jsonl", default="")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    svc = DiscoveryService(args)

    stop_flag = {"stop": False}

    def _stop(_sig, _frm):
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    svc.start()
    try:
        while not stop_flag["stop"]:
            time.sleep(0.5)
    finally:
        svc.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
