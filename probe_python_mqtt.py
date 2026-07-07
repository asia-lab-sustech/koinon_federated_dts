#!/usr/bin/env python3
"""Small Paho/Mosquitto connectivity probe for federation experiments."""

from __future__ import annotations

import argparse
import json
import socket
import time
import uuid
from typing import Any, Dict

import paho.mqtt.client as mqtt


def _callback_api(name: str):
    versions = getattr(mqtt, "CallbackAPIVersion", None)
    return getattr(versions, name, None) if versions is not None else None


def _new_client(client_id: str, *, api: str, protocol: str):
    kwargs: Dict[str, Any] = {}
    if client_id:
        kwargs["client_id"] = client_id
    protocol = str(protocol or "default").strip().lower()
    if protocol == "v311":
        kwargs["protocol"] = mqtt.MQTTv311
        kwargs["transport"] = "tcp"
    elif protocol == "v31":
        kwargs["protocol"] = mqtt.MQTTv31
        kwargs["transport"] = "tcp"
    elif protocol == "v5" and hasattr(mqtt, "MQTTv5"):
        kwargs["protocol"] = mqtt.MQTTv5
        kwargs["transport"] = "tcp"

    if api == "v1":
        cb_api = _callback_api("VERSION1")
        if cb_api is not None:
            try:
                return mqtt.Client(cb_api, **kwargs)
            except TypeError:
                pass
    elif api == "v2":
        cb_api = _callback_api("VERSION2")
        if cb_api is not None:
            try:
                return mqtt.Client(cb_api, **kwargs)
            except TypeError:
                pass

    try:
        return mqtt.Client(**kwargs)
    except TypeError:
        if client_id:
            return mqtt.Client(client_id)
        return mqtt.Client()


def _reason_ok(reason_code: Any) -> bool:
    try:
        return int(reason_code) == 0
    except Exception:
        return str(reason_code).strip().lower() in {"0", "success"}


def _probe(host: str, port: int, label: str, *, api: str, client_id: str, protocol: str, keepalive: int, timeout: float) -> dict:
    cid = client_id
    topic_token = cid or f"anon{uuid.uuid4().hex[:10]}"
    topic = f"test/fnm_probe_python/{topic_token}"
    events = []
    got_message = {"ok": False}
    connected = {"ok": False, "rc": ""}

    c = _new_client(cid, api=api, protocol=protocol)

    def on_connect(client, _userdata, _flags, reason_code, _properties=None):
        connected["ok"] = _reason_ok(reason_code)
        connected["rc"] = str(reason_code)
        events.append({"event": "connect", "rc": str(reason_code)})
        client.subscribe(topic, qos=0)
        client.publish(topic, json.dumps({"ok": 1, "label": label}), qos=0, retain=False)

    def on_disconnect(_client, _userdata, *args):
        reason_code = args[-2] if len(args) >= 2 else (args[-1] if args else "")
        flags = args[-3] if len(args) >= 3 else ""
        events.append({"event": "disconnect", "rc": str(reason_code), "flags": str(flags)})

    def on_connect_fail(_client, _userdata):
        events.append({"event": "connect_fail"})

    def on_message(_client, _userdata, msg):
        got_message["ok"] = True
        events.append({"event": "message", "topic": str(msg.topic), "payload": msg.payload.decode("utf-8", "replace")})

    def on_log(_client, _userdata, level, buf):
        if len([e for e in events if e.get("event") == "log"]) < 12:
            events.append({"event": "log", "level": str(level), "message": str(buf)})

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect
    c.on_message = on_message
    c.on_log = on_log
    try:
        c.on_connect_fail = on_connect_fail
    except Exception:
        pass

    out = {
        "label": label,
        "host": host,
        "port": port,
        "api": api,
        "client_id": cid or "",
        "protocol": protocol,
        "keepalive": int(keepalive),
        "connect_rc": None,
        "connected": False,
        "roundtrip_message": False,
        "events": events,
    }
    try:
        rc = c.connect(host, int(port), keepalive=int(keepalive))
        out["connect_rc"] = int(rc)
        c.loop_start()
        deadline = time.time() + max(1.0, float(timeout))
        while time.time() < deadline and not got_message["ok"]:
            time.sleep(0.05)
        out["connected"] = bool(connected["ok"])
        out["connack_rc"] = str(connected["rc"])
        out["roundtrip_message"] = bool(got_message["ok"])
    except Exception as e:
        out["error"] = f"{type(e).__name__}:{e}"
    finally:
        try:
            c.loop_stop()
        except Exception:
            pass
        try:
            c.disconnect()
        except Exception:
            pass
    return out


def _encode_remaining_length(n: int) -> bytes:
    out = bytearray()
    while True:
        digit = n % 128
        n //= 128
        if n:
            digit |= 0x80
        out.append(digit)
        if not n:
            return bytes(out)


def _utf8(s: str) -> bytes:
    b = str(s).encode("utf-8")
    return len(b).to_bytes(2, "big") + b


def _raw_probe(host: str, port: int, label: str, *, protocol: str, client_id: str, keepalive: int, timeout: float) -> dict:
    events = []
    out = {
        "label": label,
        "api": "raw-socket",
        "host": host,
        "port": port,
        "protocol": protocol,
        "client_id": client_id,
        "keepalive": int(keepalive),
        "connected": False,
        "connack_rc": "",
        "events": events,
    }
    try:
        if protocol == "v31":
            variable = _utf8("MQIsdp") + bytes([3, 0x02]) + int(keepalive).to_bytes(2, "big")
        elif protocol == "v5":
            variable = _utf8("MQTT") + bytes([5, 0x02]) + int(keepalive).to_bytes(2, "big") + b"\x00"
        else:
            variable = _utf8("MQTT") + bytes([4, 0x02]) + int(keepalive).to_bytes(2, "big")
        payload = _utf8(client_id)
        packet = b"\x10" + _encode_remaining_length(len(variable) + len(payload)) + variable + payload
        events.append({"event": "send_raw_connect", "n_bytes": len(packet), "packet_hex_prefix": packet[:32].hex()})
        with socket.create_connection((host, int(port)), timeout=max(1.0, float(timeout))) as sock:
            sock.settimeout(max(1.0, float(timeout)))
            sock.sendall(packet)
            data = sock.recv(8)
        events.append({"event": "recv", "n_bytes": len(data), "data_hex": data.hex()})
        if protocol == "v5" and len(data) >= 5 and data[0] == 0x20:
            out["connected"] = data[3] == 0
            out["connack_rc"] = str(data[3])
        elif len(data) >= 4 and data[0] == 0x20:
            out["connected"] = data[3] == 0
            out["connack_rc"] = str(data[3])
    except Exception as e:
        out["error"] = f"{type(e).__name__}:{e}"
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe Python/Paho MQTT connectivity.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--timeout-sec", type=float, default=5.0)
    args = ap.parse_args()

    base = f"probe{uuid.uuid4().hex[:10]}"
    rows = [
        {"label": "legacy_short_default_k15", "api": "v1", "client_id": base + "a", "protocol": "default", "keepalive": 15},
        {"label": "legacy_empty_default_k15", "api": "v1", "client_id": "", "protocol": "default", "keepalive": 15},
        {"label": "v2_short_default_k15", "api": "v2", "client_id": base + "b", "protocol": "default", "keepalive": 15},
        {"label": "v2_empty_default_k15", "api": "v2", "client_id": "", "protocol": "default", "keepalive": 15},
        {"label": "legacy_short_v311_k15", "api": "v1", "client_id": base + "c", "protocol": "v311", "keepalive": 15},
        {"label": "legacy_short_v31_k15", "api": "v1", "client_id": base + "d", "protocol": "v31", "keepalive": 15},
        {"label": "v2_short_v5_k15", "api": "v2", "client_id": base + "e", "protocol": "v5", "keepalive": 15},
        {"label": "auto_short_default_k60", "api": "auto", "client_id": base + "f", "protocol": "default", "keepalive": 60},
    ]

    print(
        json.dumps(
            {
                "probe": "paho_python_mqtt",
                "paho_version": getattr(mqtt, "__version__", ""),
                "has_callback_api": bool(getattr(mqtt, "CallbackAPIVersion", None)),
                "host": args.host,
                "port": args.port,
            },
            ensure_ascii=True,
            sort_keys=True,
        )
    )
    results = [
        _probe(args.host, args.port, timeout=args.timeout_sec, **row)
        for row in rows
    ]
    results.extend(
        [
            _raw_probe(args.host, args.port, "raw_v311_short_k60", protocol="v311", client_id=base + "r1", keepalive=60, timeout=args.timeout_sec),
            _raw_probe(args.host, args.port, "raw_v311_empty_k60", protocol="v311", client_id="", keepalive=60, timeout=args.timeout_sec),
            _raw_probe(args.host, args.port, "raw_v31_short_k60", protocol="v31", client_id=base + "r2", keepalive=60, timeout=args.timeout_sec),
            _raw_probe(args.host, args.port, "raw_v5_short_k60", protocol="v5", client_id=base + "r3", keepalive=60, timeout=args.timeout_sec),
        ]
    )
    for row in results:
        print(json.dumps(row, ensure_ascii=True, sort_keys=True))
    return 0 if any(r.get("connected") and r.get("roundtrip_message") for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
