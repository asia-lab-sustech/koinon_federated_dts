import argparse
import json
import re
import signal
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, short_mqtt_client_id, make_mqtt_client, topic_match_namespace, topic_with_namespace


@dataclass
class CatalogEntry:
    key: str
    gateway_id: str
    fnm_id: str
    node_id: str
    role: str
    topic_namespace: str
    service_name: str
    direction: str
    event_type: str
    publish_topic: str
    subscribe_topic: str
    capabilities: List[Dict[str, Any]]
    capability_names: List[str]
    updated_ts: float


class CatalogService:
    _TOKEN_RE = re.compile(r"^[A-Za-z0-9_.+\-{}]+$")
    _ID_RE = re.compile(r"^[A-Za-z0-9_.:\-]+$")

    def __init__(self, args):
        self.args = args
        self.instance = f"catalog-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.mqtt_client_id = short_mqtt_client_id("cat", self.instance)
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass
        self.entries: Dict[str, CatalogEntry] = {}
        self.dt_profiles: Dict[str, Dict[str, Any]] = {}
        self.gateway_signatures: Dict[str, Any] = {}
        self.member_status: Dict[str, str] = {}
        self.last_state_pub = 0.0
        allowed = [str(x).strip().upper() for x in str(args.membership_allowed_statuses or "").split(",") if str(x).strip()]
        self.allowed_member_statuses = set(allowed or ["REGISTERED", "ONBOARDING", "ACTIVE"])
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "exact") or "exact").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()
        self.topic_contract_mode = str(
            getattr(args, "topic_contract_mode", "enforce") or "enforce"
        ).strip().lower()
        if self.topic_contract_mode not in {"off", "warn", "enforce"}:
            self.topic_contract_mode = "enforce"
        self.topic_contract_federation_prefix = str(
            getattr(args, "topic_contract_federation_prefix", "federation/") or "federation/"
        ).strip().strip("/")
        if self.topic_contract_federation_prefix:
            self.topic_contract_federation_prefix += "/"
        self.topic_contract_local_prefixes = [
            str(x).strip().strip("/")
            for x in str(getattr(args, "topic_contract_local_prefixes", "rw/,dt/") or "rw/,dt/").split(",")
            if str(x).strip()
        ]
        self.topic_contract_local_prefixes = [
            (p + "/") if p and not p.endswith("/") else p for p in self.topic_contract_local_prefixes
        ]
        self.allowed_directions = {"local_to_fed", "fed_to_local", "bidirectional"}

    @staticmethod
    def _norm_ns(namespace: str) -> str:
        return str(namespace or "").strip().strip("/")

    def _scoped_key(self, gateway_id: str, namespace: str = "") -> str:
        gid = str(gateway_id or "").strip()
        ns = self._norm_ns(namespace)
        return f"{ns}::{gid}" if ns else gid

    def _service_key(self, gateway_id: str, service_name: str, namespace: str = "") -> str:
        return f"{self._scoped_key(gateway_id, namespace)}:{str(service_name or '').strip()}"

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "catalog", "instance": self.instance, "event": event}
        status = str(kw.get("status", "") or "").upper().strip()
        if status:
            row.update(self._membership_lifecycle_trace(status))
        row.update(kw)
        self.log.write(row)

    @staticmethod
    def _membership_lifecycle_trace(status: str) -> Dict[str, Any]:
        status_u = str(status or "").upper().strip()
        mapping = {
            "CANDIDATE": ("candidate_dt", "registration"),
            "VERIFYING": ("registration_verification", "registration"),
            "REGISTERED": ("registered_member", "registration_readiness"),
            "ONBOARDING": ("registered_member", "registration_readiness"),
            "ACTIVE": ("active_runtime_ready", "active_runtime_operations"),
            "ALIVE": ("active_runtime_ready", "active_runtime_operations"),
            "EXPIRED": ("expired_member", "suspension_or_expiry"),
            "SUSPENDED": ("suspended_member", "suspension_or_expiry"),
            "REVOKED": ("revoked_member", "governance_future"),
            "RETIRED": ("retired_member", "governance_future"),
            "DELETED": ("deleted_member", "governance_future"),
        }
        state, phase = mapping.get(status_u, ("membership_status_observed", "membership_observation"))
        return {
            "lifecycle_trace_only": 1,
            "lifecycle_model": "membership_lifecycle_observer",
            "implemented_status": status_u,
            "paper_lifecycle_state": state,
            "paper_lifecycle_phase": phase,
        }

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

    def _publish_state(self) -> None:
        def _profile_rows(ns_filter: str = "") -> List[Dict[str, Any]]:
            rows: List[Dict[str, Any]] = []
            for gid, profile in self.dt_profiles.items():
                p_ns = self._norm_ns(str(profile.get("topic_namespace", "") or ""))
                if ns_filter and p_ns != ns_filter:
                    continue
                rows.append(
                    {
                        "gateway_id": str(profile.get("gateway_id", "") or gid),
                        "fnm_id": str(
                            profile.get("fnm_id", profile.get("gateway_id", gid))
                            or profile.get("gateway_id", gid)
                        ),
                        "node_id": str(profile.get("node_id", "") or ""),
                        "role": str(profile.get("role", "") or ""),
                        "dt_profile": dict(profile.get("dt_profile", {}) or {}),
                        "capabilities": list(profile.get("capabilities", []) or []),
                        "capability_names": list(profile.get("capability_names", []) or []),
                        "topic_namespace": p_ns,
                        "updated_ts": float(profile.get("updated_ts", now_ts()) or now_ts()),
                    }
                )
            return rows

        def _entry_rows(ns_filter: str = "") -> List[Dict[str, Any]]:
            rows: List[Dict[str, Any]] = []
            for e in self.entries.values():
                row = asdict(e)
                e_ns = self._norm_ns(str(row.get("topic_namespace", "") or ""))
                if ns_filter and e_ns != ns_filter:
                    continue
                rows.append(row)
            return rows

        if self.topic_match_mode == "suffix":
            namespaces = {
                self._norm_ns(ns)
                for ns in self.observed_namespaces
                if self._norm_ns(ns)
            }
            namespaces.update(
                self._norm_ns(str(asdict(e).get("topic_namespace", "") or ""))
                for e in self.entries.values()
                if self._norm_ns(str(asdict(e).get("topic_namespace", "") or ""))
            )
            namespaces.update(
                self._norm_ns(str(p.get("topic_namespace", "") or ""))
                for p in self.dt_profiles.values()
                if self._norm_ns(str(p.get("topic_namespace", "") or ""))
            )
            for ns in sorted(namespaces or {""}):
                entries = _entry_rows(ns)
                profiles = _profile_rows(ns)
                payload = {
                    "schema": "federation.catalog.v2",
                    "event": "state",
                    "service": "catalog",
                    "topic_namespace": ns,
                    "n_entries": len(entries),
                    "n_dt_profiles": len(profiles),
                    "entries": entries,
                    "dt_profiles": profiles,
                    "ts": now_ts(),
                }
                self._pub(self.args.state_topic, payload, namespace=ns)
            return

        entries = _entry_rows("")
        profiles = _profile_rows("")
        payload = {
            "schema": "federation.catalog.v2",
            "event": "state",
            "service": "catalog",
            "n_entries": len(entries),
            "n_dt_profiles": len(profiles),
            "entries": entries,
            "dt_profiles": profiles,
            "ts": now_ts(),
        }
        self._pub(self.args.state_topic, payload)

    def _publish_event(self, event: str, payload: Dict[str, Any], namespace: str = "") -> None:
        out = {"schema": "federation.catalog.v2", "event": str(event), "service": "catalog", "ts": now_ts()}
        out.update(payload)
        self._pub(self.args.events_topic, out, namespace=namespace)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            client.subscribe(self.args.upsert_topic, qos=0)
            client.subscribe(self.args.membership_state_topic, qos=0)
            client.subscribe(self.args.membership_events_topic, qos=0)
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

        ns = self._match_ns(topic, self.args.upsert_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "catalog_rx",
                channel="upsert",
                topic=str(topic),
                canonical_topic=str(self.args.upsert_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
                role=str(payload.get("role", "") or ""),
            )
            self._handle_upsert(payload, t0, namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.membership_state_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "catalog_rx",
                channel="membership_state",
                topic=str(topic),
                canonical_topic=str(self.args.membership_state_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_membership_state(payload, namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.membership_events_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "catalog_rx",
                channel="membership_events",
                topic=str(topic),
                canonical_topic=str(self.args.membership_events_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                status=str(payload.get("status", "") or ""),
            )
            self._handle_membership_event(payload, namespace=str(ns or ""))
            return

    def _handle_membership_state(self, payload: Dict[str, Any], namespace: str = "") -> None:
        for m in list(payload.get("members", []) or []):
            gid = str(m.get("gateway_id", "") or "")
            status = str(m.get("status", "") or "")
            if gid:
                self.member_status[self._scoped_key(gid, namespace)] = status
                if not namespace:
                    self.member_status[gid] = status

    def _handle_membership_event(self, payload: Dict[str, Any], namespace: str = "") -> None:
        gid = str(payload.get("gateway_id", "") or "")
        status = str(payload.get("status", "") or "")
        if gid and status:
            self.member_status[self._scoped_key(gid, namespace)] = status
            if not namespace:
                self.member_status[gid] = status

    def _normalize_capability_names(self, value: Any) -> List[str]:
        out: List[str] = []
        if isinstance(value, str):
            s = value.strip()
            if s:
                out.append(s)
        elif isinstance(value, list):
            for x in value:
                if isinstance(x, dict):
                    s = str(x.get("name", x.get("id", "")) or "").strip()
                else:
                    s = str(x).strip()
                if s:
                    out.append(s)
        seen = set()
        uniq: List[str] = []
        for n in out:
            if n not in seen:
                uniq.append(n)
                seen.add(n)
        return uniq

    def _normalize_dt_profile(
        self,
        gid: str,
        node_id: str,
        role: str,
        payload_profile: Dict[str, Any],
    ) -> Dict[str, Any]:
        p = dict(payload_profile or {})
        geo_scope = dict(p.get("geo_scope", {}) or {})
        qos_sla = dict(p.get("qos_sla", {}) or {})
        ownership = dict(p.get("ownership", {}) or {})
        policy_tags = list(p.get("policy_tags", []) or [])
        policy_tags = sorted({str(x).strip() for x in policy_tags if str(x).strip()})
        return {
            "gateway_id": str(gid),
            "node_id": str(node_id),
            "role": str(role),
            "dt_profile": {
                "dt_description": str(p.get("dt_description", "") or f"{role} {node_id}"),
                "geo_scope": {
                    "type": str(geo_scope.get("type", "") or ""),
                    "city": str(geo_scope.get("city", "") or ""),
                    "zone": str(geo_scope.get("zone", "") or ""),
                    "network": str(geo_scope.get("network", "") or ""),
                    "id": str(geo_scope.get("id", "") or ""),
                },
                "policy_tags": policy_tags,
                "qos_sla": {
                    "update_period_sec": float(qos_sla.get("update_period_sec", 1.0) or 1.0),
                    "latency_budget_ms": float(qos_sla.get("latency_budget_ms", 500.0) or 500.0),
                    "availability_target": str(qos_sla.get("availability_target", "best_effort") or "best_effort"),
                },
                "interface_version": str(p.get("interface_version", "") or "federation.catalog.paper_profile.v1"),
                "ownership": {
                    "organization": str(ownership.get("organization", "") or ""),
                    "domain": str(ownership.get("domain", "") or ""),
                    "operator": str(ownership.get("operator", "") or ""),
                },
            },
            "updated_ts": now_ts(),
        }

    @staticmethod
    def _has_whitespace(s: str) -> bool:
        return any(ch.isspace() for ch in str(s or ""))

    def _valid_id(self, value: Any) -> bool:
        s = str(value or "").strip()
        if not s or self._has_whitespace(s):
            return False
        return bool(self._ID_RE.match(s))

    def _valid_topic(self, topic: Any) -> bool:
        t = str(topic or "").strip()
        if not t or self._has_whitespace(t):
            return False
        if t.startswith("/") or t.endswith("/") or "//" in t:
            return False
        parts = t.split("/")
        if not parts:
            return False
        for p in parts:
            if not p:
                return False
            if not self._TOKEN_RE.match(p):
                return False
        return True

    def _is_federation_topic(self, topic: str) -> bool:
        pref = str(self.topic_contract_federation_prefix or "")
        if not pref:
            return False
        t = str(topic or "")
        # Federated runs may advertise bridge subscriptions with a dynamic
        # scenario namespace before the canonical federation prefix, e.g.
        # +/+/+/+/federation/v1/request/...
        return t.startswith(pref) or f"/{pref}" in t

    def _is_local_topic(self, topic: str) -> bool:
        t = str(topic or "")
        for pref in self.topic_contract_local_prefixes:
            if pref and t.startswith(pref):
                return True
        return False

    def _validate_service_contract(self, service: Dict[str, Any]) -> List[str]:
        reasons: List[str] = []
        s = dict(service or {})
        name = str(s.get("name", "") or "").strip()
        direction = str(s.get("direction", "") or "").strip()
        event_type = str(s.get("event_type", "") or "").strip()
        publish_topic = str(s.get("publish_topic", "") or "").strip()
        subscribe_topic = str(s.get("subscribe_topic", "") or "").strip()
        missing = []
        if not name:
            missing.append("name")
        if not direction:
            missing.append("direction")
        if not event_type:
            missing.append("event_type")
        if not publish_topic:
            missing.append("publish_topic")
        if not subscribe_topic:
            missing.append("subscribe_topic")
        if missing:
            reasons.append("missing_fields:" + ",".join(missing))
            return reasons

        if direction not in self.allowed_directions:
            reasons.append("invalid_direction")
        if not self._valid_id(name):
            reasons.append("invalid_service_name")
        if not self._valid_id(event_type):
            reasons.append("invalid_event_type")
        if publish_topic == subscribe_topic:
            reasons.append("identical_pub_sub")
        if not self._valid_topic(publish_topic):
            reasons.append("invalid_publish_topic")
        if not self._valid_topic(subscribe_topic):
            reasons.append("invalid_subscribe_topic")

        is_pub_fed = self._is_federation_topic(publish_topic)
        is_sub_fed = self._is_federation_topic(subscribe_topic)
        is_pub_local = self._is_local_topic(publish_topic)
        is_sub_local = self._is_local_topic(subscribe_topic)
        if direction == "local_to_fed":
            if not is_pub_fed:
                reasons.append("local_to_fed_publish_not_federation")
            if self.topic_contract_local_prefixes and not is_sub_local:
                reasons.append("local_to_fed_subscribe_not_local")
        elif direction == "fed_to_local":
            if not is_sub_fed:
                reasons.append("fed_to_local_subscribe_not_federation")
            if self.topic_contract_local_prefixes and not is_pub_local:
                reasons.append("fed_to_local_publish_not_local")
        elif direction == "bidirectional":
            if not (is_pub_fed or is_sub_fed):
                reasons.append("bidirectional_missing_federation_topic")
        return reasons

    def _validate_capability_names(self, capability_names: List[str]) -> List[str]:
        bad: List[str] = []
        for c in list(capability_names or []):
            if not self._valid_id(c):
                bad.append(str(c))
        return bad

    def _handle_upsert(self, payload: Dict[str, Any], t0: float, namespace: str = "") -> None:
        gid = str(payload.get("gateway_id", "") or "")
        fnm_id = str(payload.get("fnm_id", gid) or gid)
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "") or "")
        services = list(payload.get("services", []) or [])
        payload_dt_profile = dict(payload.get("dt_profile", {}) or {})
        payload_capabilities = list(payload.get("capabilities", []) or [])
        payload_capability_names = self._normalize_capability_names(payload.get("capability_names", []))
        if not payload_capability_names and payload_capabilities:
            payload_capability_names = self._normalize_capability_names(payload_capabilities)

        if not gid or not node_id:
            self._emit("catalog_upsert_reject", reason="missing_ids")
            return

        ns = self._norm_ns(namespace)
        member_status = str(
            self.member_status.get(self._scoped_key(gid, ns), self.member_status.get(gid, ""))
            or ""
        ).upper()
        if self.args.require_registered_member and member_status and member_status not in self.allowed_member_statuses:
            self._emit(
                "catalog_upsert_reject",
                reason="member_status_not_allowed",
                gateway_id=gid,
                status=member_status,
                allowed=sorted(self.allowed_member_statuses),
            )
            return

        contract_service_errors: List[Dict[str, Any]] = []
        if self.topic_contract_mode != "off":
            seen_service_names = set()
            for idx, s in enumerate(services):
                service = dict(s or {})
                sname = str(service.get("name", "") or "")
                if sname in seen_service_names and sname:
                    contract_service_errors.append(
                        {"index": int(idx), "service_name": sname, "reasons": ["duplicate_service_name"]}
                    )
                else:
                    seen_service_names.add(sname)
                reasons = self._validate_service_contract(service)
                if reasons:
                    contract_service_errors.append(
                        {"index": int(idx), "service_name": sname, "reasons": list(reasons)}
                    )
            bad_caps = self._validate_capability_names(payload_capability_names)
            if bad_caps:
                contract_service_errors.append(
                    {
                        "index": -1,
                        "service_name": "-",
                        "reasons": ["invalid_capability_names:" + ",".join(sorted(set(bad_caps)))],
                    }
                )
            if contract_service_errors:
                self._emit(
                    "catalog_upsert_reject",
                    reason="topic_contract_invalid",
                    gateway_id=gid,
                    node_id=node_id,
                    mode=self.topic_contract_mode,
                    invalid_n=len(contract_service_errors),
                    invalid_examples=contract_service_errors[:5],
                    topic_namespace=str(namespace or "-") or "-",
                )
                if self.topic_contract_mode == "enforce":
                    return

        norm_services = []
        for s in services:
            name = str(s.get("name", "") or "")
            if not name:
                continue
            norm_services.append((
                name,
                str(s.get("direction", "")),
                str(s.get("event_type", "")),
                str(s.get("publish_topic", "")),
                str(s.get("subscribe_topic", "")),
            ))
        normalized_profile = self._normalize_dt_profile(gid, node_id, role, payload_dt_profile)
        normalized_profile["fnm_id"] = fnm_id
        normalized_profile["capabilities"] = payload_capabilities
        normalized_profile["capability_names"] = payload_capability_names
        profile_sig = json.dumps(normalized_profile.get("dt_profile", {}), sort_keys=True, separators=(",", ":"))
        normalized_profile["gateway_id"] = gid
        normalized_profile["topic_namespace"] = ns
        profile_key = self._scoped_key(gid, ns)
        self.dt_profiles[profile_key] = normalized_profile
        profile = dict(normalized_profile.get("dt_profile", {}) or {})
        geo_scope = dict(profile.get("geo_scope", {}) or {})
        ownership = dict(profile.get("ownership", {}) or {})
        policy_tags = list(profile.get("policy_tags", []) or [])
        signature = (
            node_id,
            role,
            tuple(sorted(norm_services)),
            tuple(payload_capability_names),
            profile_sig,
        )
        changed = signature != self.gateway_signatures.get(profile_key)
        self.gateway_signatures[profile_key] = signature

        n = 0
        for s in services:
            name = str(s.get("name", "") or "")
            if not name:
                continue
            key = self._service_key(gid, name, ns)
            ent = CatalogEntry(
                key=key,
                gateway_id=gid,
                fnm_id=fnm_id,
                node_id=node_id,
                role=role,
                topic_namespace=ns,
                service_name=name,
                direction=str(s.get("direction", "")),
                event_type=str(s.get("event_type", "")),
                publish_topic=str(s.get("publish_topic", "")),
                subscribe_topic=str(s.get("subscribe_topic", "")),
                capabilities=payload_capabilities,
                capability_names=payload_capability_names,
                updated_ts=now_ts(),
            )
            self.entries[key] = ent
            n += 1

        lat_ms = round((now_ts() - t0) * 1000.0, 3)
        ev_name = "catalog_upsert" if changed else "catalog_refresh"
        self._emit(
            ev_name,
            gateway_id=gid,
            node_id=node_id,
            n_services=n,
            changed=int(changed),
            dt_profile_present=int(bool(payload_dt_profile)),
            interface_version=str(profile.get("interface_version", "") or ""),
            geo_scope_type=str(geo_scope.get("type", "") or ""),
            geo_scope_city=str(geo_scope.get("city", "") or ""),
            geo_scope_zone=str(geo_scope.get("zone", "") or ""),
            ownership_org=str(ownership.get("organization", "") or ""),
            policy_tags_n=len(policy_tags),
            latency_ms=lat_ms,
            topic_namespace=str(namespace or "-") or "-",
        )
        self._publish_event(ev_name, {
            "gateway_id": gid,
            "node_id": node_id,
            "n_services": n,
            "changed": bool(changed),
            "dt_profile_present": int(bool(payload_dt_profile)),
            "interface_version": str(profile.get("interface_version", "") or ""),
            "geo_scope_type": str(geo_scope.get("type", "") or ""),
            "geo_scope_city": str(geo_scope.get("city", "") or ""),
            "geo_scope_zone": str(geo_scope.get("zone", "") or ""),
            "ownership_org": str(ownership.get("organization", "") or ""),
            "policy_tags_n": len(policy_tags),
            "latency_ms": lat_ms,
        }, namespace=namespace)

    def tick(self) -> None:
        t = now_ts()
        if (t - self.last_state_pub) >= max(0.5, float(self.args.state_interval_sec)):
            self.last_state_pub = t
            self._publish_state()

    def start(self) -> None:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self._emit("connect_called", mqtt_host=self.args.mqtt_host, mqtt_port=int(self.args.mqtt_port), rc=int(rc), mqtt_client_id=self.mqtt_client_id)
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
    ap.add_argument(
        "--membership-allowed-statuses",
        default="REGISTERED,ONBOARDING,ACTIVE",
        help="comma-separated membership statuses allowed to upsert catalog entries when --require-registered-member is enabled",
    )
    ap.add_argument("--state-interval-sec", type=float, default=2.0)
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
    ap.add_argument(
        "--topic-contract-mode",
        choices=["off", "warn", "enforce"],
        default="enforce",
        help="catalog service contract validation mode for upserted services/topics",
    )
    ap.add_argument(
        "--topic-contract-federation-prefix",
        default="federation/",
        help="required federation topic prefix used by topic contract checks",
    )
    ap.add_argument(
        "--topic-contract-local-prefixes",
        default="rw/,dt/",
        help="comma-separated local topic prefixes used by topic contract checks",
    )
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
