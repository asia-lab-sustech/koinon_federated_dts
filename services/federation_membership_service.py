import argparse
import signal
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts


@dataclass
class Member:
    gateway_id: str
    node_id: str
    role: str
    domain: str
    capabilities: List[str]
    registered_ts: float
    last_seen_ts: float
    status: str


class MembershipService:
    def __init__(self, args):
        self.args = args
        self.instance = f"membership-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.instance)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.members: Dict[str, Member] = {}
        self.last_state_pub = 0.0

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "membership", "instance": self.instance, "event": event}
        row.update(kw)
        self.log.write(row)

    def _pub(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(str(topic), json_dumps(payload), qos=0, retain=False)

    def _publish_member_event(self, event: str, member: Member, **kw: Any) -> None:
        payload = {
            "schema": "federation.membership.v1",
            "event": str(event),
            "gateway_id": member.gateway_id,
            "node_id": member.node_id,
            "role": member.role,
            "domain": member.domain,
            "status": member.status,
            "last_seen_ts": member.last_seen_ts,
            "ts": now_ts(),
        }
        payload.update(kw)
        self._pub(self.args.events_topic, payload)

    def _publish_state(self) -> None:
        payload = {
            "schema": "federation.membership.v1",
            "event": "state",
            "service": "membership",
            "n_members": len(self.members),
            "members": [asdict(m) for m in self.members.values()],
            "ts": now_ts(),
        }
        self._pub(self.args.state_topic, payload)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        client.subscribe(self.args.register_topic, qos=0)
        client.subscribe(self.args.heartbeat_topic, qos=0)
        self._emit("connected", host=self.args.mqtt_host, rc=str(reason_code))

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        t0 = now_ts()
        if topic == self.args.register_topic:
            self._handle_register(payload, t0)
            return
        if topic == self.args.heartbeat_topic:
            self._handle_heartbeat(payload, t0)
            return

    def _handle_register(self, payload: Dict[str, Any], t0: float) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "other") or "other")
        domain = str(payload.get("domain", "traffic") or "traffic")
        caps = [str(x) for x in list(payload.get("capabilities", []) or []) if str(x)]

        if not gid or not node_id:
            self._emit("register_reject", reason="missing_ids")
            return

        member = self.members.get(gid)
        created = member is None
        if member is None:
            member = Member(
                gateway_id=gid,
                node_id=node_id,
                role=role,
                domain=domain,
                capabilities=caps,
                registered_ts=now_ts(),
                last_seen_ts=now_ts(),
                status="REGISTERED",
            )
            self.members[gid] = member
        else:
            member.node_id = node_id
            member.role = role
            member.domain = domain
            member.capabilities = caps
            member.last_seen_ts = now_ts()
            member.status = "REGISTERED"

        ack_topic = f"{self.args.ack_prefix}/{gid}"
        ack = {
            "schema": "federation.membership.v1",
            "event": "register_ack",
            "request_id": payload.get("request_id", ""),
            "gateway_id": gid,
            "node_id": node_id,
            "status": "ACCEPTED",
            "ts": now_ts(),
        }
        self._pub(ack_topic, ack)

        latency_ms = round((now_ts() - t0) * 1000.0, 3)
        ev = "membership_registered" if created else "membership_refreshed"
        self._emit(ev, gateway_id=gid, node_id=node_id, role=role, latency_ms=latency_ms)
        self._publish_member_event(ev, member, latency_ms=latency_ms)

    def _handle_heartbeat(self, payload: Dict[str, Any], t0: float) -> None:
        gid = str(payload.get("gateway_id", "") or "")
        if not gid:
            self._emit("heartbeat_reject", reason="missing_gateway_id")
            return
        member = self.members.get(gid)
        if member is None:
            self._emit("heartbeat_unknown", gateway_id=gid)
            return
        member.last_seen_ts = now_ts()
        if member.status != "REGISTERED":
            member.status = "REGISTERED"
        latency_ms = round((now_ts() - t0) * 1000.0, 3)
        self._emit("membership_heartbeat", gateway_id=gid, latency_ms=latency_ms)
        self._publish_member_event("membership_heartbeat", member, latency_ms=latency_ms)

    def _prune(self) -> None:
        ttl = float(self.args.member_ttl_sec)
        if ttl <= 0:
            return
        t = now_ts()
        for m in self.members.values():
            if (t - float(m.last_seen_ts)) > ttl and m.status != "SUSPENDED":
                m.status = "SUSPENDED"
                idle = round(t - float(m.last_seen_ts), 3)
                self._emit("membership_suspended", gateway_id=m.gateway_id, node_id=m.node_id, idle_sec=idle)
                self._publish_member_event("membership_suspended", m, idle_sec=idle)

    def tick(self) -> None:
        self._prune()
        t = now_ts()
        if (t - self.last_state_pub) >= max(0.5, float(self.args.state_interval_sec)):
            self.last_state_pub = t
            self._publish_state()

    def start(self) -> None:
        self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host, register_topic=self.args.register_topic)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation Membership Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--register-topic", default="federation/membership/register")
    ap.add_argument("--heartbeat-topic", default="federation/membership/heartbeat")
    ap.add_argument("--ack-prefix", default="federation/membership/ack")
    ap.add_argument("--state-topic", default="federation/membership/state")
    ap.add_argument("--events-topic", default="federation/membership/events")
    ap.add_argument("--member-ttl-sec", type=float, default=30.0)
    ap.add_argument("--state-interval-sec", type=float, default=2.0)
    ap.add_argument("--log-jsonl", default="")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    svc = MembershipService(args)
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
