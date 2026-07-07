import argparse
import signal
import time
from collections import Counter, defaultdict
from typing import Any, DefaultDict, Dict, List

import paho.mqtt.client as mqtt

from _federation_common import JsonlLogger, json_dumps, json_loads, now_ts, stats, topic_match_namespace


def _latency_ms(t0: float | None) -> float | None:
    if t0 is None:
        return None
    try:
        return round((now_ts() - float(t0)) * 1000.0, 3)
    except Exception:
        return None


class MetricsService:
    def __init__(self, args):
        self.args = args
        self.instance = f"metrics-{int(now_ts())}"
        self.log = JsonlLogger(args.log_jsonl)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.instance)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        self.event_counts: Counter = Counter()
        self.latencies: DefaultDict[str, List[float]] = defaultdict(list)
        self.last_pub = 0.0
        self.topic_match_mode = str(getattr(args, "topic_match_mode", "exact") or "exact").strip().lower()
        if self.topic_match_mode not in {"exact", "suffix"}:
            self.topic_match_mode = "exact"
        self.topic_subscribe_wildcard = str(getattr(args, "topic_subscribe_wildcard", "#") or "#").strip()

    def _emit(self, event: str, **kw: Any) -> None:
        row = {"ts": now_ts(), "service": "metrics", "instance": self.instance, "event": event}
        row.update(kw)
        self.log.write(row)

    def _pub(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(str(topic), json_dumps(payload), qos=0, retain=False)

    def _match_ns(self, topic: str, base_topic: str):
        return topic_match_namespace(str(topic), str(base_topic), mode=self.topic_match_mode)

    def _on_connect(self, client, _userdata, _flags, reason_code, _properties):
        if self.topic_match_mode == "suffix":
            client.subscribe(self.topic_subscribe_wildcard, qos=0)
            self._emit("subscribed", mode="suffix", wildcard=self.topic_subscribe_wildcard)
        else:
            for t in self.args.subscribe_topics:
                client.subscribe(str(t), qos=0)
        self._emit(
            "connected",
            host=self.args.mqtt_host,
            rc=str(reason_code),
            topics=self.args.subscribe_topics,
            topic_match_mode=self.topic_match_mode,
        )

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        matched_base = None
        topic_namespace = None
        for base in self.args.subscribe_topics:
            ns = self._match_ns(topic, str(base))
            if ns is not None:
                matched_base = str(base)
                topic_namespace = str(ns or "")
                break
        if matched_base is None:
            return

        payload = json_loads(msg.payload)
        service = str(payload.get("service", "unknown") or "unknown")
        event = str(payload.get("event", "unknown") or "unknown")
        key = f"{service}:{event}"
        self.event_counts[key] += 1
        self.event_counts[f"topic:{matched_base}"] += 1
        if topic_namespace:
            self.event_counts[f"ns:{topic_namespace}"] += 1

        for lk in ("latency_ms", "onboarding_latency_ms", "catalog_upsert_latency_ms", "discovery_latency_ms"):
            if lk in payload:
                try:
                    v = payload[lk]
                    if isinstance(v, dict):
                        # Already a summary; skip aggregation of summary bins.
                        continue
                    self.latencies[lk].append(float(v))
                except Exception:
                    pass

    def tick(self) -> None:
        t = now_ts()
        if (t - self.last_pub) < max(0.5, float(self.args.publish_interval_sec)):
            return
        t0 = now_ts()
        self.last_pub = t

        latency_summary = {k: stats(v) for k, v in self.latencies.items()}
        publish_latency_ms = _latency_ms(t0)
        payload = {
            "schema": "federation.metrics.v1",
            "event": "metrics",
            "service": "metrics",
            "instance": self.instance,
            "event_counts": dict(self.event_counts),
            "latency_summary": latency_summary,
            "ts": t,
        }
        self._pub(self.args.metrics_topic, payload)
        self._pub(self.args.audit_topic, payload)
        self._emit(
            "metrics_pub",
            n_event_keys=len(self.event_counts),
            n_latency_keys=len(latency_summary),
            latency_ms=publish_latency_ms if publish_latency_ms is not None else "",
            latency_scope="metrics_tick_to_metrics_publish" if publish_latency_ms is not None else "",
        )

    def start(self) -> None:
        self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), 60)
        self.client.loop_start()
        self._emit("start", mqtt_host=self.args.mqtt_host)

    def stop(self) -> None:
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.log.close()


def parse_args():
    ap = argparse.ArgumentParser(description="Federation Metrics Service")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--subscribe-topics", nargs="+", default=[
        "federation/membership/events",
        "federation/catalog/events",
        "federation/discovery/events",
        "federation/core/audit",
    ])
    ap.add_argument("--publish-interval-sec", type=float, default=5.0)
    ap.add_argument("--metrics-topic", default="federation/core/metrics")
    ap.add_argument("--audit-topic", default="federation/core/audit")
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
    svc = MetricsService(args)

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
