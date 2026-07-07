import argparse
import signal
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, short_mqtt_client_id, make_mqtt_client, topic_match_namespace, topic_with_namespace


@dataclass
class Member:
    topic_namespace: str
    gateway_id: str
    node_id: str
    role: str
    domain: str
    capabilities: List[str]
    registered_ts: float
    last_seen_ts: float
    last_catalog_ts: float
    onboarding_started_ts: float
    active_ts: float
    has_catalog: bool
    has_heartbeat: bool
    status: str


class MembershipService:
    def __init__(self, args):
        self.args = args
        self.instance = f"membership-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.mqtt_client_id = short_mqtt_client_id("mem", self.instance)
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass
        self.members: Dict[str, Member] = {}
        self.last_state_pub = 0.0
        self.lifecycle_statuses = {
            "candidate": "CANDIDATE",
            "verifying": "VERIFYING",
            "registered": "REGISTERED",
            "onboarding": "ONBOARDING",
            "active": "ACTIVE",
            "expired": "EXPIRED",
            "suspended": "SUSPENDED",
            "revoked": "REVOKED",
            "retired": "RETIRED",
            "deleted": "DELETED",
        }
        self.heartbeat_mode = str(getattr(args, "heartbeat_mode", "monitor") or "monitor").strip().lower()
        self.heartbeat_fresh_sec = max(
            0.0,
            float(getattr(args, "heartbeat_fresh_sec", 12.0) or 12.0),
        )
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "exact") or "exact").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()

    @staticmethod
    def _norm_ns(namespace: str) -> str:
        return str(namespace or "").strip().strip("/")

    def _member_key(self, gateway_id: str, namespace: str = "") -> str:
        gid = str(gateway_id or "").strip()
        ns = self._norm_ns(namespace)
        return f"{ns}::{gid}" if ns else gid

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "membership", "instance": self.instance, "event": event}
        status = str(kw.get("status", "") or "").upper().strip()
        if status:
            row.update(self._membership_lifecycle_trace(status, previous_status=kw.get("previous_status", "")))
        row.update(kw)
        self.log.write(row)

    @staticmethod
    def _membership_lifecycle_trace(status: str, previous_status: Any = "") -> Dict[str, Any]:
        """Trace-only mapping from implementation states to paper lifecycle states."""
        st = str(status or "").upper().strip()
        prev = str(previous_status or "").upper().strip()
        paper_state = {
            "CANDIDATE": "candidate_dt",
            "VERIFYING": "registration_verification",
            "REGISTERED": "registered_member",
            # ONBOARDING remains internal; the paper treats it as readiness inside registration.
            "ONBOARDING": "registered_member",
            "ACTIVE": "active_runtime_ready",
            "EXPIRED": "expired_member",
            "SUSPENDED": "suspended_member",
            "REVOKED": "revoked_member",
            "RETIRED": "retired_member",
            "DELETED": "deleted_member",
        }.get(st, "unknown")
        phase = {
            "CANDIDATE": "registration",
            "VERIFYING": "registration",
            "REGISTERED": "registration_readiness",
            "ONBOARDING": "registration_readiness",
            "ACTIVE": "active_runtime_operations",
            "EXPIRED": "suspension_or_expiry",
            "SUSPENDED": "suspension_or_expiry",
            "REVOKED": "governance_future",
            "RETIRED": "governance_future",
            "DELETED": "governance_future",
        }.get(st, "unknown")
        trace = {
            "lifecycle_trace_only": 1,
            "lifecycle_model": "membership_lifecycle",
            "implemented_status": st,
            "paper_lifecycle_state": paper_state,
            "paper_lifecycle_phase": phase,
        }
        if prev:
            trace["previous_implemented_status"] = prev
            trace["paper_lifecycle_transition"] = f"{prev}->{st}"
        return trace

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

    def _publish_member_event(self, event: str, member: Member, namespace: str = "", **kw: Any) -> None:
        ns = self._norm_ns(namespace or member.topic_namespace)
        payload = {
            "schema": "federation.membership.v1",
            "event": str(event),
            "gateway_id": member.gateway_id,
            "node_id": member.node_id,
            "role": member.role,
            "domain": member.domain,
            "status": member.status,
            "topic_namespace": (ns or ""),
            "last_seen_ts": member.last_seen_ts,
            "ts": now_ts(),
        }
        payload.update(kw)
        self._pub(self.args.events_topic, payload, namespace=ns)

    def _effective_status(self, m: Member, now_ts_wall: float) -> str:
        # "ACTIVE" is treated as a freshness-backed effective state to avoid
        # brief transition churn (REGISTERED/ONBOARDING) from destabilizing discovery.
        idle = max(0.0, float(now_ts_wall) - float(m.last_seen_ts or 0.0))
        if bool(m.has_heartbeat) and (
            self.heartbeat_fresh_sec <= 0.0 or idle <= float(self.heartbeat_fresh_sec)
        ):
            return self.lifecycle_statuses["active"]
        return str(m.status or self.lifecycle_statuses["registered"])

    def _publish_state(self) -> None:
        t_now = now_ts()
        members_by_ns: Dict[str, List[Dict[str, Any]]] = {}
        for m in self.members.values():
            row = asdict(m)
            row["status_effective"] = self._effective_status(m, t_now)
            row["is_active_effective"] = bool(str(row["status_effective"]).upper() == self.lifecycle_statuses["active"])
            row["idle_sec"] = round(max(0.0, float(t_now) - float(m.last_seen_ts or 0.0)), 3)
            ns = self._norm_ns(str(row.get("topic_namespace", "") or ""))
            members_by_ns.setdefault(ns, []).append(row)

        if self.topic_match_mode == "suffix":
            target_namespaces = sorted(self.observed_namespaces) if self.observed_namespaces else [""]
            for ns in target_namespaces:
                ns_norm = self._norm_ns(ns)
                rows = list(members_by_ns.get(ns_norm, []))
                payload = {
                    "schema": "federation.membership.v1",
                    "event": "state",
                    "service": "membership",
                    "topic_namespace": (ns_norm or ""),
                    "n_members": len(rows),
                    "members": rows,
                    "ts": t_now,
                }
                self._pub(self.args.state_topic, payload, namespace=ns_norm)
            return

        members_rows = [r for rows in members_by_ns.values() for r in rows]
        payload = {
            "schema": "federation.membership.v1",
            "event": "state",
            "service": "membership",
            "n_members": len(members_rows),
            "members": members_rows,
            "ts": t_now,
        }
        self._pub(self.args.state_topic, payload)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            client.subscribe(self.args.register_topic, qos=0)
            client.subscribe(self.args.catalog_upsert_topic, qos=0)
            if self.heartbeat_mode == "direct":
                client.subscribe(self.args.heartbeat_topic, qos=0)
            else:
                client.subscribe(self.args.lifecycle_events_topic, qos=0)
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

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        t0 = now_ts()

        ns = self._match_ns(topic, self.args.register_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "membership_rx",
                channel="register",
                topic=str(topic),
                canonical_topic=str(self.args.register_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
            )
            self._handle_register(payload, t0, namespace=str(ns or ""))
            return

        if self.heartbeat_mode == "direct":
            ns = self._match_ns(topic, self.args.heartbeat_topic)
            if ns is not None:
                self._remember_ns(ns)
                self._emit(
                    "membership_rx",
                    channel="heartbeat",
                    topic=str(topic),
                    canonical_topic=str(self.args.heartbeat_topic),
                    topic_namespace=str(ns or "-") or "-",
                    gateway_id=str(payload.get("gateway_id", "") or ""),
                    node_id=str(payload.get("node_id", "") or ""),
                )
                self._handle_heartbeat(payload, t0, namespace=str(ns or ""))
                return
        else:
            ns = self._match_ns(topic, self.args.lifecycle_events_topic)
            if ns is not None:
                self._remember_ns(ns)
                self._emit(
                    "membership_rx",
                    channel="lifecycle_event",
                    topic=str(topic),
                    canonical_topic=str(self.args.lifecycle_events_topic),
                    topic_namespace=str(ns or "-") or "-",
                    gateway_id=str(payload.get("gateway_id", "") or ""),
                    availability=str(payload.get("availability", "") or ""),
                )
                self._handle_lifecycle_event(payload, t0, namespace=str(ns or ""))
                return

        ns = self._match_ns(topic, self.args.catalog_upsert_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "membership_rx",
                channel="catalog_upsert",
                topic=str(topic),
                canonical_topic=str(self.args.catalog_upsert_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
            )
            self._handle_catalog_upsert(payload, t0, namespace=str(ns or ""))
            return

    def _publish_member_transition(self, member: Member, event: str, namespace: str = "", **kw: Any) -> None:
        ns = self._norm_ns(namespace or member.topic_namespace)
        self._emit(event, gateway_id=member.gateway_id, node_id=member.node_id, status=member.status, topic_namespace=(ns or "-"), **kw)
        self._publish_member_event(event, member, namespace=ns, **kw)

    def _mark_onboarding(self, member: Member, source: str, namespace: str = "") -> None:
        # Do not regress ACTIVE members back to ONBOARDING on periodic heartbeats.
        if member.status == self.lifecycle_statuses["active"]:
            return
        t = now_ts()
        if float(member.onboarding_started_ts or 0.0) <= 0.0:
            member.onboarding_started_ts = t
        if member.status != self.lifecycle_statuses["onboarding"]:
            member.status = self.lifecycle_statuses["onboarding"]
            self._publish_member_transition(member, "membership_onboarding_started", source=str(source), namespace=namespace)

    def _mark_active(self, member: Member, source: str, namespace: str = "") -> None:
        t = now_ts()
        if float(member.active_ts or 0.0) <= 0.0:
            member.active_ts = t
        if member.status != self.lifecycle_statuses["active"]:
            prev = str(member.status)
            member.status = self.lifecycle_statuses["active"]
            self._publish_member_transition(member, "membership_active", source=str(source), previous_status=prev, namespace=namespace)

    def _handle_register(self, payload: Dict[str, Any], t0: float, namespace: str = "") -> None:
        ns = self._norm_ns(namespace)
        gid = str(payload.get("gateway_id", "") or "")
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "other") or "other")
        domain = str(payload.get("domain", "traffic") or "traffic")
        caps: List[str] = []
        for x in list(payload.get("capabilities", []) or []):
            if isinstance(x, dict):
                s = str(x.get("name", x.get("id", "")) or "").strip()
            else:
                s = str(x).strip()
            if s:
                caps.append(s)
        # Keep stable deterministic order while deduplicating.
        caps = list(dict.fromkeys(caps))

        if not gid or not node_id:
            self._emit("register_reject", reason="missing_ids")
            return

        key = self._member_key(gid, ns)
        member = self.members.get(key)
        created = member is None
        if member is None:
            member = Member(
                topic_namespace=ns,
                gateway_id=gid,
                node_id=node_id,
                role=role,
                domain=domain,
                capabilities=caps,
                registered_ts=now_ts(),
                last_seen_ts=now_ts(),
                last_catalog_ts=0.0,
                onboarding_started_ts=0.0,
                active_ts=0.0,
                has_catalog=False,
                has_heartbeat=False,
                status=self.lifecycle_statuses["registered"],
            )
            self.members[key] = member
        else:
            prev_status = str(member.status or "")
            member.topic_namespace = ns
            member.node_id = node_id
            member.role = role
            member.domain = domain
            member.capabilities = caps
            member.last_seen_ts = now_ts()
            # Keep readiness state for steady-state refreshes/reconnects.
            # Re-register should not demote ACTIVE peers back to ONBOARDING.
            if prev_status in {
                self.lifecycle_statuses["suspended"],
                self.lifecycle_statuses["expired"],
                self.lifecycle_statuses["revoked"],
                self.lifecycle_statuses["retired"],
                self.lifecycle_statuses["deleted"],
            }:
                member.last_catalog_ts = 0.0
                member.onboarding_started_ts = 0.0
                member.active_ts = 0.0
                member.has_catalog = False
                member.has_heartbeat = False
                member.status = self.lifecycle_statuses["registered"]

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
        self._pub(ack_topic, ack, namespace=ns)

        latency_ms = round((now_ts() - t0) * 1000.0, 3)
        ev = "membership_registered" if created else "membership_refreshed"
        self._emit(ev, gateway_id=gid, node_id=node_id, role=role, status=member.status, latency_ms=latency_ms)
        self._publish_member_event(ev, member, namespace=ns, latency_ms=latency_ms)

    def _handle_heartbeat(self, payload: Dict[str, Any], t0: float, namespace: str = "") -> None:
        ns = self._norm_ns(namespace)
        gid = str(payload.get("gateway_id", "") or "")
        if not gid:
            self._emit("heartbeat_reject", reason="missing_gateway_id")
            return
        key = self._member_key(gid, ns)
        member = self.members.get(key)
        if member is None and ns:
            member = self.members.get(self._member_key(gid, ""))
        if member is None:
            self._emit("heartbeat_unknown", gateway_id=gid, topic_namespace=(ns or "-"))
            return
        member.topic_namespace = ns or member.topic_namespace
        member.last_seen_ts = now_ts()
        member.has_heartbeat = True
        if member.has_catalog:
            # Registered -> Onboarding (if needed) -> Active
            self._mark_onboarding(member, source="heartbeat", namespace=ns)
            self._mark_active(member, source="heartbeat", namespace=ns)
        else:
            if member.status == self.lifecycle_statuses["suspended"]:
                member.status = self.lifecycle_statuses["registered"]
                self._publish_member_transition(member, "membership_recovered_registered", source="heartbeat", namespace=ns)
            elif member.status not in (
                self.lifecycle_statuses["registered"],
                self.lifecycle_statuses["onboarding"],
                self.lifecycle_statuses["active"],
            ):
                member.status = self.lifecycle_statuses["registered"]
        latency_ms = round((now_ts() - t0) * 1000.0, 3)
        if bool(getattr(self.args, "emit_heartbeat_events", False)):
            self._emit("membership_heartbeat", gateway_id=gid, status=member.status, latency_ms=latency_ms)
            self._publish_member_event("membership_heartbeat", member, namespace=ns, latency_ms=latency_ms)

    def _handle_lifecycle_event(self, payload: Dict[str, Any], t0: float, namespace: str = "") -> None:
        ns = self._norm_ns(namespace)
        gid = str(payload.get("gateway_id", "") or "")
        if not gid:
            self._emit("lifecycle_event_reject", reason="missing_gateway_id")
            return
        key = self._member_key(gid, ns)
        member = self.members.get(key)
        if member is None and ns:
            member = self.members.get(self._member_key(gid, ""))
        if member is None:
            self._emit("lifecycle_unknown_member", gateway_id=gid, topic_namespace=(ns or "-"))
            return
        member.topic_namespace = ns or member.topic_namespace

        availability = str(payload.get("availability", "") or "").strip().lower()
        source = str(payload.get("source", "lifecycle_monitor") or "lifecycle_monitor")
        ts_hint = float(payload.get("last_seen_ts", now_ts()) or now_ts())
        member.last_seen_ts = max(float(member.last_seen_ts or 0.0), ts_hint)
        latency_ms = round((now_ts() - t0) * 1000.0, 3)

        if availability in ("alive", "up", "healthy"):
            member.has_heartbeat = True
            if member.has_catalog:
                self._mark_onboarding(member, source=source, namespace=ns)
                self._mark_active(member, source=source, namespace=ns)
            elif member.status == self.lifecycle_statuses["suspended"]:
                member.status = self.lifecycle_statuses["registered"]
                self._publish_member_transition(member, "membership_recovered_registered", source=source, namespace=ns)
            return

        if availability in ("unavailable", "down", "timeout", "expired", "stale"):
            member.has_heartbeat = False
            if member.status != self.lifecycle_statuses["suspended"]:
                member.status = self.lifecycle_statuses["suspended"]
                self._emit(
                    "membership_suspended",
                    gateway_id=member.gateway_id,
                    node_id=member.node_id,
                    status=member.status,
                    idle_sec=0.0,
                    source=source,
                    latency_ms=latency_ms,
                )
                self._publish_member_event("membership_suspended", member, namespace=ns, idle_sec=0.0, source=source, latency_ms=latency_ms)
            return

        # Unknown availability values are ignored to keep the lifecycle strict.
        self._emit("lifecycle_event_ignored", gateway_id=gid, availability=availability or "-", latency_ms=latency_ms)

    def _handle_catalog_upsert(self, payload: Dict[str, Any], t0: float, namespace: str = "") -> None:
        ns = self._norm_ns(namespace)
        gid = str(payload.get("gateway_id", "") or "")
        if not gid:
            return
        key = self._member_key(gid, ns)
        member = self.members.get(key)
        if member is None and ns:
            member = self.members.get(self._member_key(gid, ""))
        if member is None:
            self._emit("catalog_upsert_unknown_member", gateway_id=gid, topic_namespace=(ns or "-"))
            return
        member.topic_namespace = ns or member.topic_namespace
        member.last_catalog_ts = now_ts()
        member.has_catalog = True
        self._mark_onboarding(member, source="catalog_upsert", namespace=ns)
        if member.has_heartbeat:
            self._mark_active(member, source="catalog_upsert", namespace=ns)
        latency_ms = round((now_ts() - t0) * 1000.0, 3)
        self._emit("membership_catalog_seen", gateway_id=gid, latency_ms=latency_ms, status=member.status, topic_namespace=(ns or "-"))

    def _prune(self) -> None:
        ttl = float(self.args.member_ttl_sec)
        if ttl <= 0:
            return
        t = now_ts()
        for m in self.members.values():
            if (t - float(m.last_seen_ts)) > ttl and m.status != self.lifecycle_statuses["suspended"]:
                m.status = self.lifecycle_statuses["suspended"]
                idle = round(t - float(m.last_seen_ts), 3)
                ns = self._norm_ns(m.topic_namespace)
                self._emit("membership_suspended", gateway_id=m.gateway_id, node_id=m.node_id, status=m.status, idle_sec=idle, topic_namespace=(ns or "-"))
                self._publish_member_event("membership_suspended", m, namespace=ns, idle_sec=idle)

    def tick(self) -> None:
        if self.heartbeat_mode == "direct":
            self._prune()
        t = now_ts()
        if (t - self.last_state_pub) >= max(0.5, float(self.args.state_interval_sec)):
            self.last_state_pub = t
            self._publish_state()

    def start(self) -> None:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self._emit("connect_called", mqtt_host=self.args.mqtt_host, mqtt_port=int(self.args.mqtt_port), rc=int(rc), mqtt_client_id=self.mqtt_client_id)
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
    ap.add_argument("--lifecycle-events-topic", default="federation/lifecycle/events")
    ap.add_argument(
        "--heartbeat-mode",
        default="monitor",
        choices=["monitor", "direct"],
        help="membership lifecycle source: monitor=federation/lifecycle/events, direct=federation/membership/heartbeat",
    )
    ap.add_argument(
        "--emit-heartbeat-events",
        action="store_true",
        default=False,
        help="in direct mode, also emit membership_heartbeat events (high-volume)",
    )
    ap.add_argument("--catalog-upsert-topic", default="federation/catalog/upsert")
    ap.add_argument("--ack-prefix", default="federation/membership/ack")
    ap.add_argument("--state-topic", default="federation/membership/state")
    ap.add_argument("--events-topic", default="federation/membership/events")
    ap.add_argument("--member-ttl-sec", type=float, default=60.0)
    ap.add_argument(
        "--heartbeat-fresh-sec",
        type=float,
        default=30.0,
        help="freshness window used to expose effective ACTIVE state in membership snapshots",
    )
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
