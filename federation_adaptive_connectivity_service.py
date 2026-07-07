#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import threading
import time
from typing import Any, Dict, Tuple

import paho.mqtt.client as mqtt

from _federation_common import short_mqtt_client_id, make_mqtt_client


def _now() -> float:
    return float(time.time())


def _latency_ms(t0: float | None) -> float | None:
    if t0 is None:
        return None
    try:
        return round((_now() - float(t0)) * 1000.0, 3)
    except Exception:
        return None


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


def _topic_match_namespace(topic: str, base_topic: str, mode: str = "exact") -> Tuple[str | None, str | None]:
    """
    Return (namespace, canonical_topic) when topic matches the canonical federation topic.
    - exact: only direct match/prefix match with no namespace
    - suffix: allow <namespace>/<base_topic...>
    canonical_topic is the matched topic rooted at base_topic.
    """
    t = str(topic or "").strip().strip("/")
    b = str(base_topic or "").strip().strip("/")
    m = str(mode or "exact").strip().lower()
    if not t or not b:
        return None, None
    if m == "exact":
        if t == b or t.startswith(b + "/"):
            return "", t
        return None, None
    if t == b or t.startswith(b + "/"):
        return "", t
    needle = "/" + b
    idx = t.find(needle)
    if idx < 0:
        return None, None
    ns = t[:idx].strip("/")
    can = t[idx + 1 :]  # drops leading '/'
    if can == b or can.startswith(b + "/"):
        return ns, can
    return None, None


class JsonlWriter:
    def __init__(self, path: str) -> None:
        self.path = str(path or "").strip()
        self.fp = None
        if self.path:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            self.fp = open(self.path, "a", encoding="utf-8")

    def write(self, event: str, **kw: Any) -> None:
        row = {"ts": _now(), "event": str(event)}
        row.update(kw)
        line = json.dumps(row, ensure_ascii=True, separators=(",", ":"))
        print(line)
        if self.fp is not None:
            self.fp.write(line + "\n")
            self.fp.flush()

    def close(self) -> None:
        if self.fp is not None:
            self.fp.close()
            self.fp = None


class AdaptiveConnectivityService:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.log = JsonlWriter(str(args.log_jsonl or ""))
        self.stop_evt = threading.Event()
        self.mqtt_client_id = short_mqtt_client_id("adap", str(args.client_id or "adaptive"))
        self.client = make_mqtt_client(mqtt, self.mqtt_client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        try:
            self.client.on_connect_fail = self._on_connect_fail
        except Exception:
            pass
        self.membership: Dict[str, Dict[str, Any]] = {}
        self.catalog: Dict[str, Dict[str, Any]] = {}
        self.edges: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
        self.active_bindings: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
        self.last_state_emit = 0.0
        self.last_binding_prune = 0.0
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "suffix") or "suffix").strip().lower()
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()
        self.observed_namespaces = set()

    def _on_connect(self, client, _userdata, _flags, reason_code, _props=None):
        self.log.write(
            "adaptive.connectivity.connected",
            host=self.args.mqtt_host,
            port=int(self.args.mqtt_port),
            rc=str(reason_code),
            topic_match_mode=self.topic_match_mode,
            mqtt_client_id=self.mqtt_client_id,
        )
        topics = [
            "federation/membership/register",
            "federation/membership/heartbeat",
            "federation/membership/state",
            "federation/membership/events",
            "federation/catalog/upsert",
            "federation/discovery/query",
            "federation/discovery/resp/#",
            "federation/ev/request/#",
            "federation/reservation/req/#",
            "federation/reservation/resp/#",
        ]
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self.log.write("adaptive.connectivity.subscribed", topic=self.topic_subscribe_wildcard, mode="suffix")
        else:
            for t in topics:
                client.subscribe(t, qos=0)
                self.log.write("adaptive.connectivity.subscribed", topic=t, mode="exact")

    def _on_disconnect(self, _client, _userdata, *args):
        reason_code = args[-2] if len(args) >= 2 else (args[-1] if args else "")
        self.log.write(
            "adaptive.connectivity.disconnected",
            host=self.args.mqtt_host,
            port=int(self.args.mqtt_port),
            rc=str(reason_code),
            mqtt_client_id=self.mqtt_client_id,
        )

    def _on_connect_fail(self, _client, _userdata):
        self.log.write(
            "adaptive.connectivity.connect_fail",
            host=self.args.mqtt_host,
            port=int(self.args.mqtt_port),
            mqtt_client_id=self.mqtt_client_id,
        )

    @staticmethod
    def _loads(raw: bytes) -> Dict[str, Any]:
        try:
            return dict(json.loads(raw.decode("utf-8")))
        except Exception:
            return {}

    def _touch_edge(self, src: str, dst: str, purpose: str, payload: Dict[str, Any], *, t0: float | None = None) -> None:
        key = (str(src or "unknown"), str(dst or "unknown"), str(purpose or "unknown"))
        rec = dict(self.edges.get(key, {}) or {})
        rec["src"] = key[0]
        rec["dst"] = key[1]
        rec["purpose"] = key[2]
        rec["count"] = int(rec.get("count", 0)) + 1
        rec["last_seen"] = _now()
        rec["sample"] = {
            "request_id": str(payload.get("request_id", payload.get("message_id", "")) or ""),
            "event_type": str(payload.get("event_type", "") or ""),
            "topic_hint": str(payload.get("topic", "") or ""),
        }
        self.edges[key] = rec

        now = _now()
        was_present = key in self.active_bindings
        self.active_bindings[key] = {"src": key[0], "dst": key[1], "purpose": key[2], "last_seen": now}
        if (not was_present) or bool(getattr(self.args, "binding_log_refresh", False)):
            latency_ms = _latency_ms(t0)
            self.log.write(
                "adaptive.connectivity.binding_set.update",
                action=("refresh" if was_present else "add"),
                src=key[0],
                dst=key[1],
                purpose=key[2],
                ts_wall_s=now,
                active_bindings_n=int(len(self.active_bindings)),
                latency_ms=latency_ms if latency_ms is not None else "",
                latency_scope="message_to_connectivity_binding_update" if latency_ms is not None else "",
            )
            self._emit_binding_snapshot_for_source(key[0], now, t0=t0)

    def _emit_binding_snapshot_for_source(self, src: str, ts_wall_s: float | None = None, *, t0: float | None = None) -> None:
        src_s = str(src or "").strip()
        if not src_s:
            return
        peers = sorted(
            {
                str(k[1])
                for k in self.active_bindings.keys()
                if isinstance(k, tuple) and len(k) == 3 and str(k[0]) == src_s
            }
        )
        now = float(ts_wall_s if ts_wall_s is not None else _now())
        latency_ms = _latency_ms(t0)
        self.log.write(
            "adaptive.connectivity.binding_set.snapshot",
            src=src_s,
            peers=peers[:200],
            peers_n=int(len(peers)),
            ts_wall_s=now,
            active_bindings_n=int(len(self.active_bindings)),
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope="message_to_connectivity_binding_snapshot" if latency_ms is not None else "",
        )

    def _prune_stale_bindings(self, *, t0: float | None = None) -> None:
        ttl = float(getattr(self.args, "binding_ttl_sec", 0.0) or 0.0)
        if ttl <= 0.0:
            return
        now = _now()
        expired = []
        for key, rec in list(self.active_bindings.items()):
            last_seen = float((rec or {}).get("last_seen", now) or now)
            if (now - last_seen) > ttl:
                expired.append((key, last_seen))
        if not expired:
            return
        changed_src = set()
        for key, last_seen in expired:
            self.active_bindings.pop(key, None)
            changed_src.add(str(key[0]))
            latency_ms = _latency_ms(t0)
            self.log.write(
                "adaptive.connectivity.binding_set.expire",
                action="expire",
                src=str(key[0]),
                dst=str(key[1]),
                purpose=str(key[2]),
                last_seen_s=float(last_seen),
                ts_wall_s=now,
                reason="ttl",
                active_bindings_n=int(len(self.active_bindings)),
                latency_ms=latency_ms if latency_ms is not None else "",
                latency_scope="prune_tick_to_connectivity_binding_expire" if latency_ms is not None else "",
            )
        for src in sorted(changed_src):
            self._emit_binding_snapshot_for_source(src, now, t0=t0)

    def _log_rx(
        self,
        *,
        topic: str,
        canonical_topic: str,
        topic_namespace: str,
        channel: str,
        payload: Dict[str, Any],
        source: str = "",
        target: str = "",
        purpose: str = "",
        t0: float | None = None,
    ) -> None:
        req_id = str(payload.get("request_id", payload.get("message_id", payload.get("req_id", ""))) or "")
        latency_ms = _latency_ms(t0)
        status = str(payload.get("status", payload.get("member_status", payload.get("status_effective", ""))) or "").upper().strip()
        lifecycle_trace = _membership_lifecycle_trace(status) if status else {}
        self.log.write(
            "adaptive.connectivity.rx",
            topic=str(topic),
            canonical_topic=str(canonical_topic or "-"),
            topic_namespace=str(topic_namespace or "-"),
            channel=str(channel or ""),
            request_id=str(req_id),
            source=str(source or ""),
            target=str(target or ""),
            purpose=str(purpose or ""),
            requester_node_id=str(payload.get("requester_node_id", payload.get("requester", "")) or ""),
            requester_gateway_id=str(payload.get("requester_gateway_id", "") or ""),
            requester_role=str(payload.get("requester_role", "") or ""),
            gateway_id=str(payload.get("gateway_id", "") or ""),
            node_id=str(payload.get("node_id", "") or ""),
            role=str(payload.get("role", "") or ""),
            status=status,
            event_type=str(payload.get("event_type", payload.get("event", "")) or ""),
            **lifecycle_trace,
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope="message_to_connectivity_observation" if latency_ms is not None else "",
        )

    def _handle_membership(self, payload: Dict[str, Any]) -> None:
        gw = str(payload.get("gateway_id", "") or "")
        if not gw:
            return
        rec = dict(self.membership.get(gw, {}) or {})
        rec.update(
            {
                "gateway_id": gw,
                "node_id": str(payload.get("node_id", "") or ""),
                "role": str(payload.get("role", "") or ""),
                "status": str(payload.get("status", payload.get("member_status", "")) or ""),
                "last_seen": _now(),
            }
        )
        self.membership[gw] = rec

    def _handle_catalog(self, payload: Dict[str, Any]) -> None:
        gw = str(payload.get("gateway_id", "") or "")
        if not gw:
            return
        self.catalog[gw] = {
            "gateway_id": gw,
            "node_id": str(payload.get("node_id", "") or ""),
            "role": str(payload.get("role", "") or ""),
            "services_n": len(list(payload.get("services", []) or [])),
            "last_seen": _now(),
        }

    def _on_message(self, _client, _userdata, msg):
        t0 = _now()
        topic = str(msg.topic or "")
        payload = self._loads(msg.payload)
        ns, canonical = _topic_match_namespace(topic, "federation/membership/register", self.topic_match_mode)
        if ns is not None:
            self._handle_membership(payload)
            self._log_rx(topic=topic, canonical_topic=str(canonical or "-"), topic_namespace=(ns or "-"), channel="membership_register", payload=payload, t0=t0)
            return
        ns, canonical = _topic_match_namespace(topic, "federation/membership/heartbeat", self.topic_match_mode)
        if ns is not None:
            self._handle_membership(payload)
            self._log_rx(topic=topic, canonical_topic=str(canonical or "-"), topic_namespace=(ns or "-"), channel="membership_heartbeat", payload=payload, t0=t0)
            return
        ns, canonical = _topic_match_namespace(topic, "federation/membership/state", self.topic_match_mode)
        if ns is not None:
            self._handle_membership(payload)
            self._log_rx(topic=topic, canonical_topic=str(canonical or "-"), topic_namespace=(ns or "-"), channel="membership_state", payload=payload, t0=t0)
            return
        ns, canonical = _topic_match_namespace(topic, "federation/membership/events", self.topic_match_mode)
        if ns is not None:
            self._handle_membership(payload)
            self._log_rx(topic=topic, canonical_topic=str(canonical or "-"), topic_namespace=(ns or "-"), channel="membership_events", payload=payload, t0=t0)
            return

        ns, canonical = _topic_match_namespace(topic, "federation/catalog/upsert", self.topic_match_mode)
        if ns is not None:
            self._handle_catalog(payload)
            self._log_rx(topic=topic, canonical_topic=str(canonical or "-"), topic_namespace=(ns or "-"), channel="catalog_upsert", payload=payload, t0=t0)
            return

        ns, canonical = _topic_match_namespace(topic, "federation/discovery/query", self.topic_match_mode)
        if ns is not None:
            requester = str(payload.get("requester_node_id", payload.get("requester", "unknown")) or "unknown")
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical or "-"),
                topic_namespace=(ns or "-"),
                channel="discovery_query",
                payload=payload,
                source=requester,
                target="discovery_service",
                purpose="discovery_query",
                t0=t0,
            )
            return

        ns, canonical = _topic_match_namespace(topic, "federation/discovery/resp", self.topic_match_mode)
        if ns is not None:
            requester = str(payload.get("requester_node_id", payload.get("requester", "unknown")) or "unknown")
            n_results = int(payload.get("n_results", 0) or 0)
            results = list(payload.get("results", []) or [])
            peers = []
            for r in results:
                if not isinstance(r, dict):
                    continue
                node_id = str(r.get("node_id", "") or "").strip()
                gw = str(r.get("gateway_id", "") or "").strip()
                pid = node_id or gw
                if not pid:
                    continue
                peers.append(pid)
                self._touch_edge(requester, pid, "discovery_candidate_peer", payload, t0=t0)
            peers = sorted(set(peers))
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical or "-"),
                topic_namespace=(ns or "-"),
                channel="discovery_response",
                payload=payload,
                source="discovery_service",
                target=requester,
                purpose="discovery_response",
                t0=t0,
            )
            latency_ms = _latency_ms(t0)
            self.log.write(
                "adaptive.connectivity.discovery.response",
                requester_node_id=str(requester),
                request_id=str(payload.get("request_id", "") or ""),
                n_results=int(n_results),
                candidate_peers_n=int(len(peers)),
                candidate_peers=peers[:200],
                topic_namespace=str(ns or "-"),
                latency_ms=latency_ms if latency_ms is not None else "",
                latency_scope="message_to_discovery_response_binding_update" if latency_ms is not None else "",
            )
            self._emit_binding_snapshot_for_source(requester, t0=t0)
            return

        ns, canonical = _topic_match_namespace(topic, "federation/ev/request", self.topic_match_mode)
        if ns is not None and canonical is not None and canonical.startswith("federation/ev/request/"):
            dst = canonical.split("/")[-1]
            src = str(payload.get("ev_id", payload.get("source_dt_id", "ev_unknown")) or "ev_unknown")
            self._touch_edge(src, dst, "ev_priority_request", payload, t0=t0)
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical),
                topic_namespace=(ns or "-"),
                channel="ev_request",
                payload=payload,
                source=src,
                target=dst,
                purpose="ev_priority_request",
                t0=t0,
            )
            return

        ns, canonical = _topic_match_namespace(topic, "federation/reservation/req", self.topic_match_mode)
        if ns is not None and canonical is not None and canonical.startswith("federation/reservation/req/"):
            dst = canonical.split("/")[-1]
            src = str(payload.get("from_tls", payload.get("source_dt_id", "tls_unknown")) or "tls_unknown")
            self._touch_edge(src, dst, "intersection_coord_req", payload, t0=t0)
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical),
                topic_namespace=(ns or "-"),
                channel="reservation_req",
                payload=payload,
                source=src,
                target=dst,
                purpose="intersection_coord_req",
                t0=t0,
            )
            return

        ns, canonical = _topic_match_namespace(topic, "federation/reservation/resp", self.topic_match_mode)
        if ns is not None and canonical is not None and canonical.startswith("federation/reservation/resp/"):
            dst = str(payload.get("from_tls", payload.get("source_dt_id", "tls_unknown")) or "tls_unknown")
            src = str(payload.get("to_tls", payload.get("responder_tls", "tls_unknown")) or "tls_unknown")
            self._touch_edge(src, dst, "intersection_coord_resp", payload, t0=t0)
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical),
                topic_namespace=(ns or "-"),
                channel="reservation_resp",
                payload=payload,
                source=src,
                target=dst,
                purpose="intersection_coord_resp",
                t0=t0,
            )
            return

        ns, canonical = _topic_match_namespace(topic, "federation/discovery/resp", self.topic_match_mode)
        if ns is not None and canonical is not None and canonical.startswith("federation/discovery/resp/"):
            requester = canonical.split("/")[-1]
            for res in list(payload.get("results", []) or []):
                if not isinstance(res, dict):
                    continue
                peer = str(res.get("node_id", "") or "")
                if peer:
                    self._touch_edge(requester, peer, "discovery_candidate", payload, t0=t0)
            self._log_rx(
                topic=topic,
                canonical_topic=str(canonical),
                topic_namespace=(ns or "-"),
                channel="discovery_response",
                payload=payload,
                source="discovery_service",
                target=requester,
                purpose="discovery_response",
                t0=t0,
            )
            return

        self._log_rx(topic=topic, canonical_topic="-", topic_namespace="-", channel="other", payload=payload, t0=t0)

    def _emit_state(self, *, t0: float | None = None) -> None:
        now = _now()
        latency_ms = _latency_ms(t0)
        self.log.write(
            "adaptive.connectivity.state",
            members_n=int(len(self.membership)),
            catalog_nodes_n=int(len(self.catalog)),
            connectivity_edges_n=int(len(self.edges)),
            active_bindings_n=int(len(self.active_bindings)),
            edges=list(self.edges.values())[:200],
            latency_ms=latency_ms if latency_ms is not None else "",
            latency_scope="state_tick_to_connectivity_state_snapshot" if latency_ms is not None else "",
        )
        self.last_state_emit = now

    def run(self) -> int:
        rc = self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), keepalive=30)
        self.log.write(
            "adaptive.connectivity.connect_called",
            host=self.args.mqtt_host,
            port=int(self.args.mqtt_port),
            rc=int(rc),
            mqtt_client_id=self.mqtt_client_id,
        )
        self.client.loop_start()
        self._emit_state(t0=_now())
        try:
            while not self.stop_evt.is_set():
                now = _now()
                if (now - self.last_binding_prune) >= float(getattr(self.args, "binding_prune_sec", 1.0) or 1.0):
                    self._prune_stale_bindings(t0=now)
                    self.last_binding_prune = now
                if (now - self.last_state_emit) >= float(self.args.state_emit_sec):
                    self._emit_state(t0=now)
                time.sleep(0.2)
        finally:
            try:
                self.client.loop_stop()
            except Exception:
                pass
            try:
                self.client.disconnect()
            except Exception:
                pass
            self.log.close()
        return 0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Passive adaptive-connectivity monitor service")
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--client-id", default="federation-adaptive-connectivity")
    ap.add_argument(
        "--topic-match-mode",
        choices=["exact", "suffix"],
        default="suffix",
        help="exact=legacy topics only; suffix=accept namespaced topics ending in canonical federation topics",
    )
    ap.add_argument(
        "--topic-subscribe-wildcard",
        default="#",
        help="MQTT subscription used when --topic-match-mode suffix is enabled",
    )
    ap.add_argument("--log-jsonl", default="./tmp/federation_core_logs/adaptive_connectivity.jsonl")
    ap.add_argument("--state-emit-sec", type=float, default=5.0)
    ap.add_argument(
        "--binding-ttl-sec",
        type=float,
        default=15.0,
        help="active binding expires after this many seconds without seeing a message (<=0 disables expiration)",
    )
    ap.add_argument(
        "--binding-prune-sec",
        type=float,
        default=1.0,
        help="stale-binding prune cadence in wall-clock seconds",
    )
    ap.add_argument(
        "--binding-log-refresh",
        action="store_true",
        help="log binding_set.update on every touch (default logs only on add)",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    svc = AdaptiveConnectivityService(args)

    def _sig(_sig, _frame):
        svc.stop_evt.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    return int(svc.run())


if __name__ == "__main__":
    raise SystemExit(main())
