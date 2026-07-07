#!/usr/bin/env python3
import argparse
import signal
import time
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, short_mqtt_client_id, make_mqtt_client, topic_match, topic_match_namespace, topic_with_namespace


class FederationStateManagerService:
    """
    Passive federation-state observer.

    This service does not participate in membership/catalog/discovery decisions.
    It observes federation/FNM/DT topics, keeps a current DT snapshot, writes
    trace logs, and publishes read-only state topics for external applications.
    """

    def __init__(self, args):
        self.args = args
        self.instance = f"state-manager-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.mqtt_client_id = short_mqtt_client_id("fsm", self.instance)
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "suffix") or "suffix").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()
        self.dt_state: Dict[str, Dict[str, Any]] = {}
        self.last_state_pub = 0.0

    @staticmethod
    def _norm_ns(namespace: str) -> str:
        return str(namespace or "").strip().strip("/")

    def _state_key(self, gateway_id: str = "", node_id: str = "", dt_id: str = "", namespace: str = "") -> str:
        ns = self._norm_ns(namespace)
        ident = str(gateway_id or node_id or dt_id or "").strip()
        if not ident:
            ident = "unknown"
        return f"{ns}::{ident}" if ns else ident

    def _emit(self, event: str, **kw: Any) -> None:
        row = {
            "ts": now_ts(),
            "service": "state_manager",
            "instance": self.instance,
            "event": event,
            "observer_only": 1,
        }
        row.update(kw)
        self.log.write(row)

    def _pub(self, topic: str, payload: Dict[str, Any], namespace: str = "") -> None:
        self.client.publish(topic_with_namespace(str(topic), str(namespace or "")), json_dumps(payload), qos=0, retain=False)

    def _match_ns(self, topic: str, base_topic: str):
        t = str(topic or "").strip().strip("/")
        b = str(base_topic or "").strip().strip("/")
        if "+" in b or "#" in b:
            if topic_match(b, t):
                return ""
            if self.topic_match_mode == "suffix":
                parts = t.split("/")
                for i in range(1, len(parts)):
                    suffix = "/".join(parts[i:])
                    if topic_match(b, suffix):
                        return "/".join(parts[:i]).strip("/")
            return None
        return topic_match_namespace(str(topic), str(base_topic), mode=self.topic_match_mode)

    def _remember_ns(self, ns: str) -> None:
        x = self._norm_ns(ns)
        if x:
            self.observed_namespaces.add(x)

    def _update_dt(
        self,
        *,
        namespace: str = "",
        gateway_id: str = "",
        node_id: str = "",
        dt_id: str = "",
        role: str = "",
        domain: str = "",
        status: str = "",
        availability: str = "",
        source: str = "",
        payload: Dict[str, Any] | None = None,
    ) -> None:
        ns = self._norm_ns(namespace)
        gid = str(gateway_id or "").strip()
        nid = str(node_id or "").strip()
        did = str(dt_id or "").strip()
        key = self._state_key(gid, nid, did, ns)
        now = now_ts()
        existing = dict(self.dt_state.get(key, {}))
        previous = dict(existing)
        existing.update(
            {
                "topic_namespace": ns,
                "gateway_id": gid or str(existing.get("gateway_id", "") or ""),
                "node_id": nid or str(existing.get("node_id", "") or ""),
                "dt_id": did or str(existing.get("dt_id", "") or ""),
                "role": str(role or existing.get("role", "") or ""),
                "domain": str(domain or existing.get("domain", "") or ""),
                "status": str(status or existing.get("status", "") or ""),
                "availability": str(availability or existing.get("availability", "") or ""),
                "last_seen_ts": now,
                "last_source": str(source or existing.get("last_source", "") or ""),
            }
        )
        if payload:
            # Preserve lightweight telemetry for dashboards. This service is
            # observer-only, so these fields are never fed back into decisions.
            pose = payload.get("pose")
            if isinstance(pose, dict):
                existing["pose"] = dict(pose)
                for k in ("x", "y", "z", "yaw", "sumo_x", "sumo_y", "simTime", "sim_time"):
                    if k in pose:
                        existing[k] = pose.get(k)
            snapshot = payload.get("snapshot")
            if isinstance(snapshot, dict):
                existing["snapshot"] = dict(snapshot)
                for k in ("x", "y", "z", "yaw", "sumo_x", "sumo_y", "simTime", "sim_time", "speedMps", "speed_mps", "edgeId", "edge_id", "in_edge_id", "laneId", "lane_id", "existsInSim", "exists_in_sim"):
                    if k in snapshot:
                        existing[k] = snapshot.get(k)
            for k in (
                "x",
                "y",
                "z",
                "yaw",
                "sumo_x",
                "sumo_y",
                "simTime",
                "sim_time",
                "speedMps",
                "speed_mps",
                "edgeId",
                "edge_id",
                "in_edge_id",
                "laneId",
                "lane_id",
                "existsInSim",
                "exists_in_sim",
                "battery_level",
                "vbat",
                "last_event",
                "timestamp",
                "mission_request_id",
                "mission_name",
                "waypoint_index",
                "waypoint_id",
                "waypoint_edge",
                "waypoint_kind",
                "waypoint_node",
                "waypoint_node_type",
                "waypoint_region_id",
                "waypoint_region_label",
                "waypoint_region_from",
                "waypoint_region_to",
                "waypoint_sumo_x",
                "waypoint_sumo_y",
            ):
                if k in payload:
                    existing[k] = payload.get(k)
            if "connected" in payload:
                existing["connected"] = bool(payload.get("connected"))
            caps = payload.get("capabilities")
            if isinstance(caps, list):
                existing["capabilities"] = [str(x) for x in caps]
            services = payload.get("services")
            if isinstance(services, list):
                existing["services_n"] = len(services)
            if payload.get("schema"):
                existing["last_schema"] = str(payload.get("schema", "") or "")
        self.dt_state[key] = existing
        changed_fields = []
        for field in ["gateway_id", "node_id", "dt_id", "role", "domain", "status", "availability"]:
            if str(previous.get(field, "") or "") != str(existing.get(field, "") or ""):
                changed_fields.append(field)
        if not previous or changed_fields:
            self._publish_event(
                "dt_observed" if not previous else "dt_updated",
                {
                    "gateway_id": existing.get("gateway_id", ""),
                    "node_id": existing.get("node_id", ""),
                    "dt_id": existing.get("dt_id", ""),
                    "role": existing.get("role", ""),
                    "domain": existing.get("domain", ""),
                    "status": existing.get("status", ""),
                    "availability": existing.get("availability", ""),
                    "source": source,
                    "changed_fields": changed_fields,
                },
                namespace=ns,
            )
        self._emit(
            "state_manager_dt_observed",
            topic_namespace=ns or "-",
            gateway_id=existing.get("gateway_id", ""),
            node_id=existing.get("node_id", ""),
            dt_id=existing.get("dt_id", ""),
            role=existing.get("role", ""),
            status=existing.get("status", ""),
            availability=existing.get("availability", ""),
            source=source,
            active_dt_n=self._active_count(),
            x=existing.get("x"),
            y=existing.get("y"),
            z=existing.get("z"),
            yaw=existing.get("yaw"),
            sumo_x=existing.get("sumo_x"),
            sumo_y=existing.get("sumo_y"),
            sim_time=existing.get("sim_time", existing.get("simTime")),
            battery_level=existing.get("battery_level"),
            vbat=existing.get("vbat"),
            last_event=existing.get("last_event"),
            mission_request_id=existing.get("mission_request_id"),
            mission_name=existing.get("mission_name"),
            waypoint_index=existing.get("waypoint_index"),
            waypoint_id=existing.get("waypoint_id"),
            waypoint_kind=existing.get("waypoint_kind"),
            waypoint_node=existing.get("waypoint_node"),
            waypoint_region_id=existing.get("waypoint_region_id"),
            waypoint_region_from=existing.get("waypoint_region_from"),
            waypoint_region_to=existing.get("waypoint_region_to"),
            waypoint_sumo_x=existing.get("waypoint_sumo_x"),
            waypoint_sumo_y=existing.get("waypoint_sumo_y"),
        )

    def _active_count(self) -> int:
        active_statuses = {"ACTIVE", "REGISTERED", "ONBOARDING", "ALIVE"}
        active_availability = {"alive", "available", "healthy", "up"}
        n = 0
        for row in self.dt_state.values():
            status = str(row.get("status", "") or "").upper()
            availability = str(row.get("availability", "") or "").lower()
            if status in active_statuses or availability in active_availability:
                n += 1
        return n

    def _public_rows(self, namespace: str = "") -> List[Dict[str, Any]]:
        ns = self._norm_ns(namespace)
        now = now_ts()
        max_age = max(0.0, float(getattr(self.args, "active_max_age_sec", 0.0) or 0.0))
        rows: List[Dict[str, Any]] = []
        for row in self.dt_state.values():
            row_ns = self._norm_ns(str(row.get("topic_namespace", "") or ""))
            if ns and row_ns != ns:
                continue
            idle = max(0.0, now - float(row.get("last_seen_ts", 0.0) or 0.0))
            if max_age > 0.0 and idle > max_age:
                continue
            out = dict(row)
            out["idle_sec"] = round(idle, 3)
            rows.append(out)
        rows.sort(key=lambda r: (str(r.get("topic_namespace", "")), str(r.get("role", "")), str(r.get("node_id", "")), str(r.get("gateway_id", ""))))
        return rows

    def _publish_event(self, event: str, payload: Dict[str, Any], namespace: str = "") -> None:
        msg = {
            "schema": "federation.state_manager.v1",
            "event": str(event),
            "service": "state_manager",
            "observer_only": 1,
            "topic_namespace": self._norm_ns(namespace),
            "ts": now_ts(),
        }
        msg.update(payload)
        self._pub(self.args.events_topic, msg, namespace=namespace)

    def _publish_state_for_namespace(self, namespace: str = "") -> None:
        ns = self._norm_ns(namespace)
        rows = self._public_rows(ns)
        by_role: Dict[str, int] = {}
        for row in rows:
            role = str(row.get("role", "") or "unknown")
            by_role[role] = by_role.get(role, 0) + 1
        summary = {
            "schema": "federation.state_manager.v1",
            "event": "summary",
            "service": "state_manager",
            "observer_only": 1,
            "topic_namespace": ns,
            "n_dts": len(rows),
            "n_active_dts": self._active_count() if not ns else len(rows),
            "roles": by_role,
            "ts": now_ts(),
        }
        full = {
            "schema": "federation.state_manager.v1",
            "event": "dts",
            "service": "state_manager",
            "observer_only": 1,
            "topic_namespace": ns,
            "n_dts": len(rows),
            "dts": rows,
            "ts": now_ts(),
        }
        self._pub(self.args.summary_topic, summary, namespace=ns)
        self._pub(self.args.dts_topic, full, namespace=ns)
        self._emit(
            "state_manager_state_pub",
            topic_namespace=ns or "-",
            n_dts=len(rows),
            roles=by_role,
            summary_topic=topic_with_namespace(self.args.summary_topic, ns),
            dts_topic=topic_with_namespace(self.args.dts_topic, ns),
        )

    def _publish_state(self) -> None:
        if self.topic_match_mode == "suffix" and self.observed_namespaces:
            for ns in sorted(self.observed_namespaces):
                self._publish_state_for_namespace(ns)
        else:
            self._publish_state_for_namespace("")

    def _on_connect(self, client, _userdata, _flags, reason_code, _props=None):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            for topic in [
                self.args.membership_state_topic,
                self.args.membership_events_topic,
                self.args.lifecycle_events_topic,
                self.args.catalog_state_topic,
                self.args.catalog_upsert_topic,
                self.args.dt_state_topic,
                self.args.dt_health_topic,
                self.args.dt_capabilities_topic,
                self.args.fed_dt_state_topic,
                self.args.fed_dt_health_topic,
                self.args.fed_dt_capabilities_topic,
            ]:
                client.subscribe(topic, qos=0)
                self._emit("subscribed", mode="exact", topic=topic)
        self._emit(
            "connected",
            host=self.args.mqtt_host,
            port=int(self.args.mqtt_port),
            rc=str(reason_code),
            topic_match_mode=self.topic_match_mode,
            mqtt_client_id=self.mqtt_client_id,
        )

    def _on_disconnect(self, _client, _userdata, *args):
        reason_code = args[-2] if len(args) >= 2 else (args[-1] if args else "")
        self._emit("disconnected", rc=str(reason_code), mqtt_client_id=self.mqtt_client_id)

    def _on_connect_fail(self, _client, _userdata):
        self._emit("connect_fail", host=self.args.mqtt_host, port=int(self.args.mqtt_port), mqtt_client_id=self.mqtt_client_id)

    def _observe_membership_state(self, payload: Dict[str, Any], namespace: str, source: str) -> None:
        for member in list(payload.get("members", []) or []):
            if isinstance(member, dict):
                self._update_dt(
                    namespace=namespace or str(member.get("topic_namespace", "") or ""),
                    gateway_id=str(member.get("gateway_id", "") or ""),
                    node_id=str(member.get("node_id", "") or ""),
                    role=str(member.get("role", "") or ""),
                    domain=str(member.get("domain", "") or ""),
                    status=str(member.get("status_effective", member.get("status", "")) or ""),
                    source=source,
                    payload=member,
                )

    def _observe_membership_event(self, payload: Dict[str, Any], namespace: str, source: str) -> None:
        self._update_dt(
            namespace=namespace or str(payload.get("topic_namespace", "") or ""),
            gateway_id=str(payload.get("gateway_id", "") or ""),
            node_id=str(payload.get("node_id", "") or ""),
            role=str(payload.get("role", "") or ""),
            domain=str(payload.get("domain", "") or ""),
            status=str(payload.get("status", "") or ""),
            source=source,
            payload=payload,
        )

    def _observe_lifecycle_event(self, payload: Dict[str, Any], namespace: str, source: str) -> None:
        self._update_dt(
            namespace=namespace or str(payload.get("topic_namespace", "") or ""),
            gateway_id=str(payload.get("gateway_id", "") or ""),
            node_id=str(payload.get("node_id", "") or ""),
            availability=str(payload.get("availability", "") or ""),
            source=source,
            payload=payload,
        )

    def _observe_catalog_state(self, payload: Dict[str, Any], namespace: str, source: str) -> None:
        for entry in list(payload.get("entries", []) or payload.get("services", []) or []):
            if isinstance(entry, dict):
                self._update_dt(
                    namespace=namespace or str(entry.get("topic_namespace", "") or ""),
                    gateway_id=str(entry.get("gateway_id", "") or ""),
                    node_id=str(entry.get("node_id", "") or ""),
                    role=str(entry.get("role", "") or ""),
                    source=source,
                    payload=entry,
                )

    def _observe_direct_dt(self, payload: Dict[str, Any], namespace: str, source: str) -> None:
        self._update_dt(
            namespace=namespace or str(payload.get("topic_namespace", "") or ""),
            gateway_id=str(payload.get("gateway_id", payload.get("fnm_id", "")) or ""),
            node_id=str(payload.get("node_id", payload.get("dt_id", "")) or ""),
            dt_id=str(payload.get("dt_id", "") or ""),
            role=str(payload.get("role", payload.get("dt_type", "")) or ""),
            domain=str(payload.get("domain", "") or ""),
            status=str(payload.get("status", "") or ""),
            availability=str(payload.get("availability", payload.get("health", "")) or ""),
            source=source,
            payload=payload,
        )

    def _infer_fed_dt_payload(self, topic: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        Enrich FNM-republished DT topics such as
        federation/v1/state/drone/crazyflie_01 with a role and dt_id when the
        payload itself is intentionally lightweight.
        """
        out = dict(payload)
        parts = str(topic or "").strip("/").split("/")
        for marker in ("state", "health", "capabilities"):
            if marker not in parts:
                continue
            i = parts.index(marker)
            if len(parts) <= i + 2:
                continue
            kind = parts[i + 1]
            dt_id = parts[i + 2]
            if dt_id and not out.get("dt_id"):
                out["dt_id"] = dt_id
            if kind == "drone":
                out.setdefault("role", "AerialScoutSystem")
                out.setdefault("dt_type", "AerialScoutSystem")
            elif kind == "intersection":
                out.setdefault("role", "TrafficLightSystem")
                out.setdefault("dt_type", "TrafficLightSystem")
            elif kind == "ev":
                out.setdefault("role", "EmergencyVehicle")
                out.setdefault("dt_type", "EmergencyVehicle")
            break
        return out

    def _handle_topic(self, topic: str, payload: Dict[str, Any]) -> bool:
        checks = [
            (self.args.membership_state_topic, "membership_state"),
            (self.args.membership_events_topic, "membership_events"),
            (self.args.lifecycle_events_topic, "lifecycle_events"),
            (self.args.catalog_state_topic, "catalog_state"),
            (self.args.catalog_upsert_topic, "catalog_upsert"),
            (self.args.dt_state_topic, "dt_state"),
            (self.args.dt_health_topic, "dt_health"),
            (self.args.dt_capabilities_topic, "dt_capabilities"),
            (self.args.fed_dt_state_topic, "fed_dt_state"),
            (self.args.fed_dt_health_topic, "fed_dt_health"),
            (self.args.fed_dt_capabilities_topic, "fed_dt_capabilities"),
        ]
        for base_topic, source in checks:
            ns = self._match_ns(topic, base_topic)
            if ns is None:
                continue
            self._remember_ns(ns)
            ns_str = str(ns or "")
            self._emit("state_manager_rx", topic=topic, canonical_topic=base_topic, topic_namespace=ns_str or "-", source=source)
            if source == "membership_state":
                self._observe_membership_state(payload, ns_str, source)
            elif source == "membership_events":
                self._observe_membership_event(payload, ns_str, source)
            elif source == "lifecycle_events":
                self._observe_lifecycle_event(payload, ns_str, source)
            elif source == "catalog_state":
                self._observe_catalog_state(payload, ns_str, source)
            else:
                if source.startswith("fed_dt_"):
                    payload = self._infer_fed_dt_payload(topic, payload)
                self._observe_direct_dt(payload, ns_str, source)
            return True
        return False

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        if not isinstance(payload, dict):
            payload = {}
        self._handle_topic(topic, payload)

    def start(self) -> None:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), keepalive=30)
        self._emit("connect_called", mqtt_host=self.args.mqtt_host, mqtt_port=int(self.args.mqtt_port), rc=int(rc), mqtt_client_id=self.mqtt_client_id)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host, summary_topic=self.args.summary_topic, dts_topic=self.args.dts_topic)

    def tick(self) -> None:
        t = now_ts()
        if (t - float(self.last_state_pub or 0.0)) >= max(0.25, float(self.args.publish_interval_sec)):
            self.last_state_pub = t
            self._publish_state()

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation State Manager Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--membership-state-topic", default="federation/membership/state")
    ap.add_argument("--membership-events-topic", default="federation/membership/events")
    ap.add_argument("--lifecycle-events-topic", default="federation/lifecycle/events")
    ap.add_argument("--catalog-state-topic", default="federation/catalog/state")
    ap.add_argument("--catalog-upsert-topic", default="federation/catalog/upsert")
    ap.add_argument("--dt-state-topic", default="dt/+/state")
    ap.add_argument("--dt-health-topic", default="dt/+/health")
    ap.add_argument("--dt-capabilities-topic", default="dt/+/capabilities")
    ap.add_argument("--fed-dt-state-topic", default="federation/v1/state/+/+")
    ap.add_argument("--fed-dt-health-topic", default="federation/v1/health/+/+")
    ap.add_argument("--fed-dt-capabilities-topic", default="federation/v1/capabilities/+/+")
    ap.add_argument("--summary-topic", default="federation/state/summary")
    ap.add_argument("--dts-topic", default="federation/state/dts")
    ap.add_argument("--events-topic", default="federation/state/events")
    ap.add_argument("--publish-interval-sec", type=float, default=1.0)
    ap.add_argument(
        "--active-max-age-sec",
        type=float,
        default=0.0,
        help="omit stale DTs from published state after this age; 0 keeps last observed DTs",
    )
    ap.add_argument(
        "--topic-match-mode",
        choices=["exact", "suffix"],
        default="suffix",
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
    svc = FederationStateManagerService(args)
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
