#!/usr/bin/env python3
"""
HTTP read bridge for Federation State Manager MQTT topics.

The state manager remains MQTT-only and observer-only. This companion process is
for dashboards that cannot consume raw MQTT directly from a browser.
"""

import argparse
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Tuple
from urllib.parse import parse_qs, urlparse

import paho.mqtt.client as mqtt

from _federation_common import json_loads, make_mqtt_client, now_ts, short_mqtt_client_id, topic_match_namespace


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.summary_by_ns: Dict[str, Dict[str, Any]] = {}
        self.dts_by_ns: Dict[str, Dict[str, Any]] = {}
        self.last_rx_ts = 0.0

    @staticmethod
    def _ns(payload: Dict[str, Any]) -> str:
        return str(payload.get("topic_namespace", "") or "").strip().strip("/")

    def put(self, kind: str, payload: Dict[str, Any]) -> None:
        ns = self._ns(payload)
        with self.lock:
            if kind == "summary":
                self.summary_by_ns[ns] = payload
            elif kind == "dts":
                self.dts_by_ns[ns] = payload
            self.last_rx_ts = now_ts()

    def get(self, kind: str, namespace: str = "") -> Dict[str, Any]:
        ns = str(namespace or "").strip().strip("/")
        with self.lock:
            store = self.summary_by_ns if kind == "summary" else self.dts_by_ns
            if ns in store:
                return dict(store[ns])
            if "" in store:
                return dict(store[""])
            if store:
                return dict(next(iter(store.values())))
            return {
                "schema": "federation.state_bridge.v1",
                "event": kind,
                "topic_namespace": ns,
                "n_dts": 0,
                "dts": [] if kind == "dts" else None,
                "roles": {},
                "ts": now_ts(),
                "bridge_status": "waiting_for_state_manager",
            }

    def health(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "schema": "federation.state_bridge.v1",
                "event": "health",
                "status": "ok",
                "summary_namespaces": sorted(self.summary_by_ns.keys()),
                "dts_namespaces": sorted(self.dts_by_ns.keys()),
                "last_rx_ts": self.last_rx_ts,
                "idle_sec": round(max(0.0, now_ts() - self.last_rx_ts), 3) if self.last_rx_ts else None,
                "ts": now_ts(),
            }


def make_handler(cache: StateCache):
    class Handler(BaseHTTPRequestHandler):
        server_version = "FederationStateHTTPBridge/1.0"

        def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
            body = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self) -> None:
            self._send_json(200, {"ok": True})

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            namespace = qs.get("namespace", [""])[0]
            if parsed.path == "/health":
                self._send_json(200, cache.health())
            elif parsed.path == "/state/summary":
                self._send_json(200, cache.get("summary", namespace))
            elif parsed.path == "/state/dts":
                self._send_json(200, cache.get("dts", namespace))
            else:
                self._send_json(404, {"error": "not_found", "path": parsed.path})

        def log_message(self, fmt: str, *args: Tuple[Any, ...]) -> None:
            return

    return Handler


class MQTTStateReader:
    def __init__(self, args, cache: StateCache):
        self.args = args
        self.cache = cache
        self.client = make_mqtt_client(mqtt, short_mqtt_client_id("fsb", str(int(now_ts()))))
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _match(self, topic: str, base_topic: str):
        return topic_match_namespace(topic, base_topic, mode=str(self.args.topic_match_mode))

    def _on_connect(self, client, _userdata, _flags, reason_code, _props=None):
        client.subscribe(str(self.args.topic_subscribe_wildcard), qos=0)
        print(json.dumps({"event": "state_bridge.mqtt.connected", "rc": str(reason_code), "topic": self.args.topic_subscribe_wildcard}))

    def _on_message(self, _client, _userdata, msg):
        topic = str(msg.topic)
        payload = json_loads(msg.payload)
        if not isinstance(payload, dict):
            return
        if self._match(topic, self.args.summary_topic) is not None:
            self.cache.put("summary", payload)
        elif self._match(topic, self.args.dts_topic) is not None:
            self.cache.put("dts", payload)

    def start(self):
        self.client.connect(self.args.mqtt_host, int(self.args.mqtt_port), keepalive=30)
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


def parse_args():
    ap = argparse.ArgumentParser(description="HTTP bridge for Federation State Manager topics")
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=18885)
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=19100)
    ap.add_argument("--summary-topic", default="federation/state/summary")
    ap.add_argument("--dts-topic", default="federation/state/dts")
    ap.add_argument("--topic-match-mode", choices=["exact", "suffix"], default="suffix")
    ap.add_argument("--topic-subscribe-wildcard", default="#")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    cache = StateCache()
    mqtt_reader = MQTTStateReader(args, cache)
    mqtt_reader.start()
    httpd = ThreadingHTTPServer((args.http_host, int(args.http_port)), make_handler(cache))
    stop_flag = {"stop": False}

    def _stop(_sig, _frm):
        stop_flag["stop"] = True
        httpd.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    print(json.dumps({"event": "state_bridge.http.start", "host": args.http_host, "port": int(args.http_port)}))
    try:
        httpd.serve_forever(poll_interval=0.25)
    finally:
        mqtt_reader.stop()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
