import argparse
import signal
import time
from typing import Any, Dict, List, Optional

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, short_mqtt_client_id, make_mqtt_client, topic_match_namespace, topic_with_namespace


class DiscoveryService:
    def __init__(self, args):
        self.args = args
        self.instance = f"discovery-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.mqtt_client_id = short_mqtt_client_id("disc", self.instance)
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass

        self.catalog_entries: Dict[str, Dict[str, Any]] = {}
        self.dt_profiles: Dict[str, Dict[str, Any]] = {}
        self.member_status: Dict[str, Dict[str, Any]] = {}
        active = [str(x).strip().upper() for x in str(args.active_member_statuses or "").split(",") if str(x).strip()]
        self.active_member_statuses = set(active or ["ACTIVE", "REGISTERED"])
        self.member_status_max_age_sec = max(
            0.0,
            float(getattr(args, "member_status_max_age_sec", 25.0) or 25.0),
        )
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "exact") or "exact").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "discovery", "instance": self.instance, "event": event}
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

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            client.subscribe(self.args.query_topic, qos=0)
            client.subscribe(self.args.catalog_upsert_topic, qos=0)
            client.subscribe(self.args.catalog_state_topic, qos=0)
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

        ns = self._match_ns(topic, self.args.query_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "discovery_rx",
                channel="query",
                topic=str(topic),
                canonical_topic=str(self.args.query_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_query(payload, topic_namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.catalog_upsert_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "discovery_rx",
                channel="catalog_upsert",
                topic=str(topic),
                canonical_topic=str(self.args.catalog_upsert_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_catalog_upsert(payload, topic_namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.catalog_state_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "discovery_rx",
                channel="catalog_state",
                topic=str(topic),
                canonical_topic=str(self.args.catalog_state_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_catalog_state(payload, topic_namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.membership_state_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "discovery_rx",
                channel="membership_state",
                topic=str(topic),
                canonical_topic=str(self.args.membership_state_topic),
                topic_namespace=str(ns or "-") or "-",
            )
            self._handle_membership_state(payload, topic_namespace=str(ns or ""))
            return

        ns = self._match_ns(topic, self.args.membership_events_topic)
        if ns is not None:
            self._remember_ns(ns)
            self._emit(
                "discovery_rx",
                channel="membership_events",
                topic=str(topic),
                canonical_topic=str(self.args.membership_events_topic),
                topic_namespace=str(ns or "-") or "-",
                gateway_id=str(payload.get("gateway_id", "") or ""),
                node_id=str(payload.get("node_id", "") or ""),
                status=str(payload.get("status", "") or ""),
            )
            self._handle_membership_event(payload, topic_namespace=str(ns or ""))
            return

    @staticmethod
    def _member_key(gateway_id: str, topic_namespace: str = "") -> str:
        ns = str(topic_namespace or "").strip().strip("/")
        gid = str(gateway_id or "").strip()
        return f"{ns}|{gid}" if ns else gid

    @staticmethod
    def _scoped_key(gateway_id: str, topic_namespace: str = "") -> str:
        ns = str(topic_namespace or "").strip().strip("/")
        gid = str(gateway_id or "").strip()
        return f"{ns}|{gid}" if ns else gid

    def _service_key(self, gateway_id: str, service_name: str, topic_namespace: str = "") -> str:
        return f"{self._scoped_key(gateway_id, topic_namespace)}:{str(service_name or '').strip()}"

    def _set_member_status(self, gateway_id: str, status: str, *, topic_namespace: str = "", seen_ts: Optional[float] = None) -> None:
        gid = str(gateway_id or "").strip()
        st = str(status or "").strip().upper()
        if not gid or not st:
            return
        ts_seen = float(seen_ts if seen_ts is not None else now_ts())
        rec = {
            "status": st,
            "seen_ts": ts_seen,
            "topic_namespace": str(topic_namespace or "").strip().strip("/"),
        }
        self.member_status[self._member_key(gid, topic_namespace)] = rec
        if str(topic_namespace or "").strip():
            # Keep last global fallback too (for non-namespaced queries).
            self.member_status[self._member_key(gid, "")] = rec

    def _member_status_record(self, gateway_id: str, topic_namespace: str = "") -> Dict[str, Any]:
        gid = str(gateway_id or "").strip()
        if not gid:
            return {}
        ns = str(topic_namespace or "").strip().strip("/")
        if ns:
            rec = dict(self.member_status.get(self._member_key(gid, ns), {}) or {})
            if rec:
                return rec
        return dict(self.member_status.get(self._member_key(gid, ""), {}) or {})

    def _member_status_text(self, gateway_id: str, topic_namespace: str = "") -> str:
        return str((self._member_status_record(gateway_id, topic_namespace) or {}).get("status", "") or "").upper().strip()

    def _handle_membership_state(self, payload: Dict[str, Any], topic_namespace: str = "") -> None:
        payload_ts = float(payload.get("ts", now_ts()) or now_ts())
        for m in list(payload.get("members", []) or []):
            gid = str(m.get("gateway_id", "") or "")
            status = str(m.get("status_effective", m.get("status", "")) or "")
            seen_ts = float(m.get("last_seen_ts", payload_ts) or payload_ts)
            self._set_member_status(gid, status, topic_namespace=topic_namespace, seen_ts=seen_ts)

    def _handle_membership_event(self, payload: Dict[str, Any], topic_namespace: str = "") -> None:
        gid = str(payload.get("gateway_id", "") or "")
        status = str(payload.get("status", "") or "")
        seen_ts = float(payload.get("last_seen_ts", payload.get("ts", now_ts())) or now_ts())
        self._set_member_status(gid, status, topic_namespace=topic_namespace, seen_ts=seen_ts)

    def _handle_catalog_upsert(self, payload: Dict[str, Any], topic_namespace: str = "") -> None:
        gid = str(payload.get("gateway_id", "") or "")
        fnm_id = str(payload.get("fnm_id", gid) or gid)
        node_id = str(payload.get("node_id", "") or "")
        role = str(payload.get("role", "") or "")
        namespace = str(topic_namespace or "").strip().strip("/")
        dt_profile = dict(payload.get("dt_profile", {}) or {})
        capabilities = list(payload.get("capabilities", []) or [])
        capability_names = self._as_list_filter(payload.get("capability_names", []))
        if not capability_names:
            for c in capabilities:
                if isinstance(c, dict):
                    nm = str(c.get("name", c.get("id", "")) or "").strip()
                else:
                    nm = str(c).strip()
                if nm:
                    capability_names.append(nm)
        capability_names = list(dict.fromkeys(capability_names))
        if gid:
            self.dt_profiles[self._scoped_key(gid, namespace)] = {
                "gateway_id": gid,
                "fnm_id": fnm_id,
                "node_id": node_id,
                "role": role,
                "dt_profile": dt_profile,
                "capabilities": capabilities,
                "capability_names": capability_names,
                "topic_namespace": namespace,
                "updated_ts": now_ts(),
            }
        for svc in list(payload.get("services", []) or []):
            name = str(svc.get("name", "") or "")
            if not name:
                continue
            key = self._service_key(gid, name, namespace)
            self.catalog_entries[key] = {
                "node_id": node_id,
                "gateway_id": gid,
                "role": role,
                "service_name": name,
                "direction": str(svc.get("direction", "")),
                "event_type": str(svc.get("event_type", "")),
                "publish_topic": str(svc.get("publish_topic", "")),
                "subscribe_topic": str(svc.get("subscribe_topic", "")),
                "capabilities": capabilities,
                "capability_names": capability_names,
                "fnm_id": fnm_id,
                "topic_namespace": namespace,
                "updated_ts": now_ts(),
            }

    def _handle_catalog_state(self, payload: Dict[str, Any], topic_namespace: str = "") -> None:
        namespace = str(topic_namespace or "").strip().strip("/")
        for ent in list(payload.get("entries", []) or []):
            key = str(ent.get("key", "") or "")
            if not key:
                continue
            row = dict(ent)
            row["topic_namespace"] = str(row.get("topic_namespace", namespace) or namespace).strip().strip("/")
            self.catalog_entries[key] = row
        for p in list(payload.get("dt_profiles", []) or []):
            gid = str(p.get("gateway_id", "") or "")
            if not gid:
                continue
            profile_ns = str(p.get("topic_namespace", namespace) or namespace).strip().strip("/")
            self.dt_profiles[self._scoped_key(gid, profile_ns)] = {
                "gateway_id": gid,
                "fnm_id": str(p.get("fnm_id", gid) or gid),
                "node_id": str(p.get("node_id", "") or ""),
                "role": str(p.get("role", "") or ""),
                "dt_profile": dict(p.get("dt_profile", {}) or {}),
                "capabilities": list(p.get("capabilities", []) or []),
                "capability_names": self._as_list_filter(p.get("capability_names", [])),
                "topic_namespace": profile_ns,
                "updated_ts": float(p.get("updated_ts", now_ts()) or now_ts()),
            }

    def _allowed_for_requester(self, requester_role: str, ent: Dict[str, Any]) -> bool:
        # Lightweight policy placeholder: observers can only see local_to_fed resources.
        if str(requester_role) == "observer" and str(ent.get("direction", "")) != "local_to_fed":
            return False
        return True

    def _is_member_active(self, gateway_id: str, topic_namespace: str = "") -> bool:
        # Unknown members are treated as inactive to avoid leaking not-yet-ready
        # peers into discovery responses.
        rec = self._member_status_record(gateway_id, topic_namespace)
        status = str(rec.get("status", "") or "").upper().strip()
        seen_ts = float(rec.get("seen_ts", 0.0) or 0.0)
        if not status:
            return False
        if self.member_status_max_age_sec > 0.0 and seen_ts > 0.0:
            if (now_ts() - seen_ts) > float(self.member_status_max_age_sec):
                return False
        return status in self.active_member_statuses

    def _as_list_filter(self, value: Any) -> List[str]:
        if isinstance(value, str):
            return [s.strip() for s in value.split(",") if s.strip()]
        if isinstance(value, list):
            out: List[str] = []
            for x in value:
                s = str(x).strip()
                if s:
                    out.append(s)
            return out
        return []

    @staticmethod
    def _as_bool(value: Any, default: bool = False) -> bool:
        if value is None:
            return bool(default)
        if isinstance(value, bool):
            return value
        s = str(value).strip().lower()
        if s in ("1", "true", "yes", "on", "y"):
            return True
        if s in ("0", "false", "no", "off", "n"):
            return False
        return bool(default)

    def _service_entries_to_node_candidates(
        self,
        entries: List[Dict[str, Any]],
        *,
        context: Dict[str, Any],
        max_results: int,
    ) -> List[Dict[str, Any]]:
        grouped: Dict[str, Dict[str, Any]] = {}

        route_seq = [str(x).strip() for x in self._as_list_filter((context or {}).get("route_tls_sequence", [])) if str(x).strip()]
        next_order = [str(x).strip() for x in self._as_list_filter((context or {}).get("next_tls_order", [])) if str(x).strip()]
        if not route_seq and next_order:
            route_seq = list(next_order)
        cur_tls = str((context or {}).get("current_tls", "") or "").strip()
        if not cur_tls and next_order:
            cur_tls = str(next_order[0] or "").strip()
        dist_raw = dict((context or {}).get("next_tls_distance_m", {}) or {})
        dist_map: Dict[str, float] = {}
        for k, v in dist_raw.items():
            kk = str(k).strip()
            if not kk:
                continue
            try:
                dist_map[kk] = float(v)
            except Exception:
                continue

        def _rank(node_id: str) -> tuple:
            route_rank = 1e9
            if route_seq and cur_tls and node_id:
                try:
                    i_cur = route_seq.index(cur_tls)
                    i_peer = route_seq.index(node_id)
                    if i_peer >= i_cur:
                        route_rank = float(i_peer - i_cur)
                    else:
                        route_rank = float(1000 + (i_cur - i_peer))
                except ValueError:
                    route_rank = 1e9
            dist_rank = float(dist_map.get(str(node_id or ""), 1e9))
            return (route_rank, dist_rank, str(node_id or ""))

        for ent in entries:
            gid = str(ent.get("gateway_id", "") or "")
            node_id = str(ent.get("node_id", "") or "")
            k = str(gid or node_id or "")
            if not k:
                continue
            row = grouped.get(k)
            if row is None:
                row = {
                    "gateway_id": gid,
                    "fnm_id": str(ent.get("fnm_id", gid) or gid),
                    "node_id": node_id,
                    "role": str(ent.get("role", "") or ""),
                    "membership_status": str(
                        self._member_status_text(
                            gid,
                            str(ent.get("topic_namespace", "") or ""),
                        )
                        or ""
                    ),
                    "service_name": str(ent.get("service_name", "") or ""),
                    "service_names": [],
                    "event_type": str(ent.get("event_type", "") or ""),
                    "event_types": [],
                    "direction": str(ent.get("direction", "") or ""),
                    "directions": [],
                    "publish_topic": str(ent.get("publish_topic", "") or ""),
                    "publish_topics": [],
                    "subscribe_topic": str(ent.get("subscribe_topic", "") or ""),
                    "subscribe_topics": [],
                    "capabilities": list(ent.get("capabilities", []) or []),
                    "capability_names": [],
                    "dt_profile": dict(ent.get("dt_profile", {}) or {}),
                    "topic_namespace": str(ent.get("topic_namespace", "") or ""),
                    "query_topic_namespace": str(ent.get("query_topic_namespace", "") or ""),
                    "namespace_scope": str(ent.get("namespace_scope", "") or ""),
                    "global_provider_for_namespace": bool(ent.get("global_provider_for_namespace", False)),
                }
                grouped[k] = row
            svc = str(ent.get("service_name", "") or "")
            if svc and svc not in row["service_names"]:
                row["service_names"].append(svc)
            ev = str(ent.get("event_type", "") or "")
            if ev and ev not in row["event_types"]:
                row["event_types"].append(ev)
            d = str(ent.get("direction", "") or "")
            if d and d not in row["directions"]:
                row["directions"].append(d)
            pt = str(ent.get("publish_topic", "") or "")
            if pt and pt not in row["publish_topics"]:
                row["publish_topics"].append(pt)
            st = str(ent.get("subscribe_topic", "") or "")
            if st and st not in row["subscribe_topics"]:
                row["subscribe_topics"].append(st)
            for cn in self._capability_names_from_entry(ent):
                if cn and cn not in row["capability_names"]:
                    row["capability_names"].append(cn)
            if not row.get("dt_profile"):
                row["dt_profile"] = dict(ent.get("dt_profile", {}) or {})
            if not row.get("service_name") and svc:
                row["service_name"] = svc
            if not row.get("event_type") and ev:
                row["event_type"] = ev
            if not row.get("direction") and d:
                row["direction"] = d
            if not row.get("publish_topic") and pt:
                row["publish_topic"] = pt
            if not row.get("subscribe_topic") and st:
                row["subscribe_topic"] = st
            if not row.get("topic_namespace"):
                row["topic_namespace"] = str(ent.get("topic_namespace", "") or "")
            if not row.get("query_topic_namespace"):
                row["query_topic_namespace"] = str(ent.get("query_topic_namespace", "") or "")
            if bool(ent.get("global_provider_for_namespace", False)):
                row["global_provider_for_namespace"] = True
                row["namespace_scope"] = "global_provider"
            elif not row.get("namespace_scope"):
                row["namespace_scope"] = str(ent.get("namespace_scope", "") or "")

        out = list(grouped.values())
        out.sort(key=lambda x: _rank(str(x.get("node_id", "") or "")))
        return out[: max(1, int(max_results or 1))]

    @staticmethod
    def _capability_names_from_entry(ent: Dict[str, Any]) -> List[str]:
        names: List[str] = []
        for x in list(ent.get("capability_names", []) or []):
            s = str(x).strip()
            if s:
                names.append(s)
        for x in list(ent.get("capabilities", []) or []):
            if isinstance(x, dict):
                s = str(x.get("name", x.get("id", "")) or "").strip()
            else:
                s = str(x).strip()
            if s:
                names.append(s)
        out: List[str] = []
        seen = set()
        for n in names:
            if n not in seen:
                out.append(n)
                seen.add(n)
        return out

    def _matches_capability_filter(self, ent: Dict[str, Any], filters: List[str]) -> bool:
        if not filters:
            return True
        ent_caps = set(self._capability_names_from_entry(ent))
        for f in filters:
            if str(f).strip() in ent_caps:
                return True
        return False

    def _context_matches(self, ent: Dict[str, Any], context: Dict[str, Any]) -> bool:
        if not isinstance(context, dict) or not context:
            return True
        route_seq = [str(x).strip() for x in self._as_list_filter(context.get("route_tls_sequence", []))]
        current_tls = str(context.get("current_tls", "") or "").strip()
        node_id = str(ent.get("node_id", "") or "").strip()
        if route_seq and current_tls and node_id:
            try:
                idx = route_seq.index(current_tls)
            except ValueError:
                return True
            lookahead = int(context.get("lookahead_hops", 0) or 0)
            back = int(context.get("context_back_hops", 1) or 1)
            lo = max(0, idx - max(0, back))
            hi = min(len(route_seq) - 1, idx + max(0, lookahead))
            allowed = set(route_seq[lo : hi + 1])
            if node_id not in allowed:
                return False
        return True

    def _matches_any_exact(self, value: str, filters: List[str]) -> bool:
        if not filters:
            return True
        v = str(value or "").strip()
        return any(v == str(f or "").strip() for f in filters)

    def _matches_event_semantic(self, event_type_value: str, filters: List[str]) -> bool:
        """
        Event filter matcher with semantic aliases.
        Examples:
          - state  -> matches *State
          - event  -> matches *Event
          - request -> matches *Request
          - response -> matches *Response
          - decision -> matches *Decision
          - advice -> matches *Advice
        """
        if not filters:
            return True

        ev = str(event_type_value or "").strip()
        ev_l = ev.lower()
        if not ev_l:
            return False

        suffix_aliases = {
            "state": "state",
            "event": "event",
            "request": "request",
            "response": "response",
            "decision": "decision",
            "advice": "advice",
        }
        for raw in filters:
            t = str(raw or "").strip()
            if not t:
                continue
            tl = t.lower()
            if ev == t or ev_l == tl:
                return True
            suf = suffix_aliases.get(tl)
            if suf and ev_l.endswith(suf):
                return True
        return False

    def _profile_matches_filters(self, profile_wrapper: Dict[str, Any], filters: Dict[str, Any]) -> bool:
        profile = dict(profile_wrapper.get("dt_profile", {}) or {})
        geo_scope = dict(profile.get("geo_scope", {}) or {})
        qos_sla = dict(profile.get("qos_sla", {}) or {})
        ownership = dict(profile.get("ownership", {}) or {})

        desc_contains = str(filters.get("dt_description_contains", "") or "").strip().lower()
        if desc_contains:
            desc = str(profile.get("dt_description", "") or "").lower()
            if desc_contains not in desc:
                return False

        interface_version = str(filters.get("interface_version", "") or "").strip()
        if interface_version and str(profile.get("interface_version", "") or "") != interface_version:
            return False

        geo_scope_type = str(filters.get("geo_scope_type", "") or "").strip()
        if geo_scope_type and str(geo_scope.get("type", "") or "") != geo_scope_type:
            return False

        geo_scope_id = str(filters.get("geo_scope_id", "") or "").strip()
        if geo_scope_id:
            geo_id = str(geo_scope.get("id", "") or "")
            if geo_scope_id != geo_id:
                return False

        geo_scope_city = str(filters.get("geo_scope_city", "") or "").strip()
        if geo_scope_city and str(geo_scope.get("city", "") or "") != geo_scope_city:
            return False

        geo_scope_zone = str(filters.get("geo_scope_zone", "") or "").strip()
        if geo_scope_zone and str(geo_scope.get("zone", "") or "") != geo_scope_zone:
            return False

        policy_any = self._as_list_filter(filters.get("policy_tags_any", []))
        policy_all = self._as_list_filter(filters.get("policy_tags_all", []))
        policy_tags = {str(x).strip() for x in list(profile.get("policy_tags", []) or []) if str(x).strip()}
        if policy_any and not any(tag in policy_tags for tag in policy_any):
            return False
        if policy_all and not all(tag in policy_tags for tag in policy_all):
            return False

        owner_org = str(filters.get("ownership_organization", "") or "").strip()
        if owner_org and str(ownership.get("organization", "") or "") != owner_org:
            return False
        owner_domain = str(filters.get("ownership_domain", "") or "").strip()
        if owner_domain and str(ownership.get("domain", "") or "") != owner_domain:
            return False
        owner_operator = str(filters.get("ownership_operator", "") or "").strip()
        if owner_operator and str(ownership.get("operator", "") or "") != owner_operator:
            return False

        qos_max_update = filters.get("qos_sla_max_update_period_sec", "")
        if str(qos_max_update).strip():
            try:
                if float(qos_sla.get("update_period_sec", 1e18) or 1e18) > float(qos_max_update):
                    return False
            except Exception:
                return False

        qos_max_latency = filters.get("qos_sla_max_latency_budget_ms", "")
        if str(qos_max_latency).strip():
            try:
                if float(qos_sla.get("latency_budget_ms", 1e18) or 1e18) > float(qos_max_latency):
                    return False
            except Exception:
                return False

        qos_min_availability = str(filters.get("qos_sla_availability_target", "") or "").strip()
        if qos_min_availability and str(qos_sla.get("availability_target", "") or "") != qos_min_availability:
            return False

        return True

    def _handle_query(self, payload: Dict[str, Any], topic_namespace: str = "") -> None:
        t0 = now_ts()
        request_id = str(payload.get("request_id", ""))
        requester_id = str(payload.get("requester_id", "") or "")
        requester_node_id = str(
            payload.get("requester_node_id", payload.get("requester", "")) or ""
        )
        requester_gateway_id = str(payload.get("requester_gateway_id", "") or "")
        requester_role = str(payload.get("requester_role", "") or "")
        purpose = str(payload.get("purpose", "") or "unspecified")
        reply_topic_raw = str(payload.get("reply_topic", "") or "").strip()
        reply_topic = reply_topic_raw or f"{self.args.reply_prefix}/{requester_node_id or 'unknown'}"
        # Backward/forward compatibility:
        # - legacy callers may use "query"
        # - newer callers may use "filters"
        # We normalize both into one effective filter object.
        query_obj = dict(payload.get("query", {}) or {})
        filters = dict(payload.get("filters", {}) or {})
        if not filters and query_obj:
            filters = dict(query_obj)
        else:
            for k, v in query_obj.items():
                filters.setdefault(k, v)
        max_results = int(payload.get("max_results", 50) or 50)
        context = dict(payload.get("context", {}) or {})

        role_filters = self._as_list_filter(filters.get("role", ""))
        event_filters = self._as_list_filter(filters.get("event_type", ""))
        service_filters = self._as_list_filter(filters.get("service_name", ""))
        direction_filters = self._as_list_filter(filters.get("direction", ""))
        capability_filters = self._as_list_filter(filters.get("capability", filters.get("capability_names", "")))
        status_filters = [str(x).strip().upper() for x in self._as_list_filter(filters.get("status", "")) if str(x).strip()]
        result_mode = str(payload.get("result_mode", filters.get("result_mode", "service")) or "service").strip().lower()
        node_dedup = self._as_bool(
            payload.get("node_dedup", filters.get("node_dedup", None)),
            default=bool(result_mode == "service" and str(purpose) == "fnm_fcm"),
        )
        self._emit(
            "discovery_query_in",
            request_id=request_id,
            requester_id=str(requester_id),
            requester=str(requester_node_id),
            requester_node_id=str(requester_node_id),
            requester_gateway_id=str(requester_gateway_id),
            requester_role=requester_role,
            reply_topic=str(reply_topic),
            purpose=purpose,
            result_mode=result_mode,
            role_filter=list(role_filters),
            event_filter=list(event_filters),
            service_filter=list(service_filters),
            direction_filter=list(direction_filters),
            capability_filter=list(capability_filters),
            status_filter=list(status_filters),
            role_filter_csv=",".join(role_filters) if role_filters else "-",
            event_filter_csv=",".join(event_filters) if event_filters else "-",
            service_filter_csv=",".join(service_filters) if service_filters else "-",
            direction_filter_csv=",".join(direction_filters) if direction_filters else "-",
            capability_filter_csv=",".join(capability_filters) if capability_filters else "-",
            status_filter_csv=",".join(status_filters) if status_filters else "-",
            node_dedup=bool(node_dedup),
            max_results=max_results,
            query_filters=dict(filters),
            context=dict(context),
            topic_namespace=str(topic_namespace or "-") or "-",
        )

        matched_entries: List[Dict[str, Any]] = []
        rejected_inactive = 0
        rejected_direction = 0
        rejected_capability = 0
        rejected_context = 0
        rejected_status = 0
        rejected_namespace = 0
        for ent in self.catalog_entries.values():
            gid = str(ent.get("gateway_id", "") or "")
            ent_ns = str(ent.get("topic_namespace", "") or "").strip().strip("/")
            q_ns = str(topic_namespace or "").strip().strip("/")
            namespace_scope = "same"
            if q_ns and ent_ns and ent_ns != q_ns:
                rejected_namespace += 1
                continue
            if q_ns and not ent_ns:
                # Global providers (for example edge Drone-DTs) are intentionally
                # discoverable from scenario-namespaced experiments. They still
                # have to pass role/capability/status filters below.
                namespace_scope = "global_provider"
            if not self._matches_any_exact(str(ent.get("role", "")), role_filters):
                continue
            if not self._matches_event_semantic(str(ent.get("event_type", "")), event_filters):
                continue
            if not self._matches_any_exact(str(ent.get("service_name", "")), service_filters):
                continue
            if not self._matches_any_exact(str(ent.get("direction", "")), direction_filters):
                rejected_direction += 1
                continue
            if capability_filters and not self._matches_capability_filter(ent, capability_filters):
                rejected_capability += 1
                continue
            if context and not self._context_matches(ent, context):
                rejected_context += 1
                continue
            status_namespace = ent_ns if ent_ns else ""
            if self.args.only_active_members and not self._is_member_active(gid, status_namespace):
                rejected_inactive += 1
                continue
            if status_filters:
                st = self._member_status_text(gid, status_namespace)
                if st and st not in status_filters:
                    rejected_status += 1
                    continue
            if not self._allowed_for_requester(requester_role, ent):
                continue
            profile_wrapper = dict(
                self.dt_profiles.get(self._scoped_key(gid, ent_ns), self.dt_profiles.get(gid, {}))
                or {}
            )
            if not self._profile_matches_filters(profile_wrapper, filters):
                continue
            ent_out = dict(ent)
            ent_out["query_topic_namespace"] = str(q_ns)
            ent_out["namespace_scope"] = str(namespace_scope)
            ent_out["global_provider_for_namespace"] = bool(namespace_scope == "global_provider")
            if profile_wrapper:
                ent_out["dt_profile"] = dict(profile_wrapper.get("dt_profile", {}) or {})
            matched_entries.append(ent_out)

        out: List[Dict[str, Any]] = []
        if result_mode in ("dt", "twin", "participant"):
            grouped: Dict[str, Dict[str, Any]] = {}
            for ent in matched_entries:
                gid = str(ent.get("gateway_id", "") or "")
                ent_ns = str(ent.get("topic_namespace", "") or "").strip().strip("/")
                if not gid:
                    continue
                row = grouped.get(gid)
                if row is None:
                    profile_wrapper = dict(
                        self.dt_profiles.get(self._scoped_key(gid, ent_ns), self.dt_profiles.get(gid, {}))
                        or {}
                    )
                    row = {
                        "gateway_id": gid,
                        "fnm_id": str(ent.get("fnm_id", gid) or gid),
                        "node_id": str(ent.get("node_id", "") or ""),
                        "role": str(ent.get("role", "") or ""),
                        "dt_profile": dict(profile_wrapper.get("dt_profile", {}) or {}),
                        "capabilities": list(ent.get("capabilities", []) or []),
                        "capability_names": list(ent.get("capability_names", []) or []),
                        "services": [],
                    }
                    grouped[gid] = row
                row["services"].append({
                    "service_name": str(ent.get("service_name", "") or ""),
                    "event_type": str(ent.get("event_type", "") or ""),
                    "direction": str(ent.get("direction", "") or ""),
                    "publish_topic": str(ent.get("publish_topic", "") or ""),
                    "subscribe_topic": str(ent.get("subscribe_topic", "") or ""),
                })
            out = sorted(grouped.values(), key=lambda x: str(x.get("gateway_id", "")))[:max_results]
        else:
            if node_dedup:
                out = self._service_entries_to_node_candidates(
                    matched_entries,
                    context=context,
                    max_results=max_results,
                )
            else:
                out = matched_entries[:max_results]

        lat_ms = round((now_ts() - t0) * 1000.0, 3)
        resp = {
            "schema": "federation.discovery.v1",
            "event": "query_resp",
            "request_id": request_id,
            "requester": requester_node_id,
            "requester_node_id": requester_node_id,
            "requester_gateway_id": requester_gateway_id,
            "requester_role": requester_role,
            "purpose": purpose,
            "result_mode": result_mode,
            "node_dedup": bool(node_dedup),
            "n_results": len(out),
            "results": out,
            "summary": {
                "n_total_catalog_entries": len(self.catalog_entries),
                "n_matched_entries": len(matched_entries),
                "rejected_inactive_members": int(rejected_inactive),
                "rejected_namespace_mismatch": int(rejected_namespace),
                "rejected_direction_mismatch": int(rejected_direction),
                "rejected_capability_mismatch": int(rejected_capability),
                "rejected_context_mismatch": int(rejected_context),
                "rejected_status_mismatch": int(rejected_status),
            },
            "latency_ms": lat_ms,
            "ts": now_ts(),
        }
        # Preserve requester-provided fully qualified reply topics; otherwise,
        # when running with topic namespaces, publish discovery responses back
        # into the same namespace observed on the query channel.
        out_ns = str(topic_namespace or "")
        if out_ns:
            ns_prefix = f"{out_ns.strip('/')}/"
            if str(reply_topic).startswith(ns_prefix):
                out_ns = ""
        self._pub(reply_topic, resp, namespace=out_ns)
        self._pub(self.args.events_topic, {
            "schema": "federation.discovery.v1",
            "event": "query_resp",
            "request_id": request_id,
            "requester": requester_node_id,
            "requester_node_id": requester_node_id,
            "requester_gateway_id": requester_gateway_id,
            "purpose": purpose,
            "result_mode": result_mode,
            "node_dedup": bool(node_dedup),
            "n_results": len(out),
            "latency_ms": lat_ms,
            "ts": now_ts(),
        }, namespace=str(topic_namespace or ""))
        discovered_gateways: List[str] = []
        if result_mode in ("dt", "twin", "participant"):
            discovered_gateways = [str(x.get("gateway_id", "") or "") for x in out if str(x.get("gateway_id", "") or "")]
        else:
            discovered_gateways = sorted({
                str(x.get("gateway_id", "") or "")
                for x in out
                if str(x.get("gateway_id", "") or "")
            })
        self._emit(
            "discovery_query_resp",
            request_id=request_id,
            requester_id=str(requester_id),
            requester=str(requester_node_id),
            requester_node_id=str(requester_node_id),
            requester_gateway_id=str(requester_gateway_id),
            requester_role=str(requester_role),
            reply_topic=str(reply_topic),
            reply_namespace=str(out_ns or "-"),
            purpose=purpose,
            result_mode=result_mode,
            node_dedup=bool(node_dedup),
            n_results=len(out),
            discovered_gateways_sample=discovered_gateways[:10],
            rejected_inactive_members=int(rejected_inactive),
            rejected_namespace_mismatch=int(rejected_namespace),
            rejected_direction_mismatch=int(rejected_direction),
            rejected_capability_mismatch=int(rejected_capability),
            rejected_context_mismatch=int(rejected_context),
            rejected_status_mismatch=int(rejected_status),
            lifecycle_trace_only=1,
            lifecycle_model="discoverability_gate",
            paper_lifecycle_state="discoverability_filter_applied",
            paper_lifecycle_phase="active_runtime_operations",
            active_member_statuses=sorted(str(x) for x in self.active_member_statuses),
            only_active_members=int(bool(self.args.only_active_members)),
            latency_ms=lat_ms,
        )

    def start(self) -> None:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self._emit("connect_called", mqtt_host=self.args.mqtt_host, mqtt_port=int(self.args.mqtt_port), rc=int(rc), mqtt_client_id=self.mqtt_client_id)
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
    ap.add_argument(
        "--active-member-statuses",
        default="ACTIVE,REGISTERED,ONBOARDING,ALIVE",
        help="comma-separated membership statuses considered active for discovery filtering",
    )
    ap.add_argument(
        "--member-status-max-age-sec",
        type=float,
        default=0.0,
        help="ignore membership statuses older than this age (seconds) when filtering active members; 0 disables age check",
    )
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
