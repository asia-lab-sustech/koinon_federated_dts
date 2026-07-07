import json
import hashlib
import os
import re
import time
import uuid
from statistics import median
from typing import Any, Dict, List


def now_ts() -> float:
    return float(time.time())


def new_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:10]}"


def short_mqtt_client_id(prefix: str, seed: str = "", max_len: int = 16) -> str:
    """Return a conservative MQTT client id accepted by older brokers.

    Some broker deployments still enforce short, alphanumeric MQTT 3.1-era
    client ids. Keeping transport ids compact avoids hard-to-diagnose pre-CONNACK
    disconnects while application identity remains in message payloads.
    """
    raw_prefix = re.sub(r"[^A-Za-z0-9]", "", str(prefix or "c")) or "c"
    n = max(8, int(max_len))
    p = raw_prefix[: max(1, min(4, n - 10))]
    material = f"{prefix}:{seed}:{os.getpid()}:{uuid.uuid4().hex}:{time.time_ns()}"
    digest = hashlib.sha1(material.encode("utf-8")).hexdigest()[: max(6, n - len(p))]
    return (p + digest)[:n]


def make_mqtt_client(mqtt_module: Any, client_id: str = "") -> Any:
    """Create a Paho client with conservative callback compatibility.

    We keep the broker-visible client id compact, but deliberately leave the
    protocol and transport at Paho defaults. Prefer the legacy callback API
    when Paho exposes it: the federation services use callback signatures that
    must run consistently across Paho 1.x and 2.x lab environments.
    """
    kwargs: Dict[str, Any] = {}
    if str(client_id or "").strip():
        kwargs["client_id"] = str(client_id).strip()
    protocol = str(os.environ.get("FNM_MQTT_PROTOCOL") or os.environ.get("FED_MQTT_PROTOCOL") or "").strip().lower()
    if protocol in {"mqttv5", "v5", "5"} and hasattr(mqtt_module, "MQTTv5"):
        kwargs["protocol"] = mqtt_module.MQTTv5
    elif protocol in {"mqttv311", "v311", "3.1.1", "311"} and hasattr(mqtt_module, "MQTTv311"):
        kwargs["protocol"] = mqtt_module.MQTTv311
    elif protocol in {"mqttv31", "v31", "3.1", "31"} and hasattr(mqtt_module, "MQTTv31"):
        kwargs["protocol"] = mqtt_module.MQTTv31

    cb_versions = getattr(mqtt_module, "CallbackAPIVersion", None)
    cb_api = getattr(cb_versions, "VERSION1", None)
    if cb_api is not None:
        try:
            return mqtt_module.Client(cb_api, **kwargs)
        except TypeError:
            pass

    try:
        return mqtt_module.Client(**kwargs)
    except TypeError:
        cb_api = getattr(cb_versions, "VERSION2", None)
        if cb_api is not None:
            try:
                return mqtt_module.Client(cb_api, **kwargs)
            except TypeError:
                pass
        if "client_id" in kwargs:
            return mqtt_module.Client(kwargs["client_id"])
        return mqtt_module.Client()


def json_loads(raw: bytes) -> Dict[str, Any]:
    try:
        return dict(json.loads(raw.decode("utf-8")))
    except Exception:
        return {}


def json_dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def topic_match(sub: str, topic: str) -> bool:
    s = str(sub).split("/")
    t = str(topic).split("/")
    i = j = 0
    while i < len(s) and j < len(t):
        if s[i] == "#":
            return True
        if s[i] == "+":
            i += 1
            j += 1
            continue
        if s[i] != t[j]:
            return False
        i += 1
        j += 1
    if i < len(s) and s[i] == "#":
        return True
    return i == len(s) and j == len(t)


def _norm_topic(x: str) -> str:
    return str(x or "").strip().strip("/")


def topic_namespace_for(topic: str, base_topic: str):
    t = _norm_topic(topic)
    b = _norm_topic(base_topic)
    if not b:
        return None
    if t == b:
        return ""
    suffix = "/" + b
    if t.endswith(suffix):
        ns = t[: -len(suffix)].strip("/")
        return ns
    return None


def topic_with_namespace(base_topic: str, namespace: str) -> str:
    b = _norm_topic(base_topic)
    ns = _norm_topic(namespace)
    if ns:
        return f"{ns}/{b}"
    return b


def topic_match_namespace(topic: str, base_topic: str, mode: str = "exact"):
    m = str(mode or "exact").strip().lower()
    if m == "exact":
        return "" if _norm_topic(topic) == _norm_topic(base_topic) else None
    if m == "suffix":
        return topic_namespace_for(topic, base_topic)
    return None


def stats(xs: List[float]) -> Dict[str, float]:
    if not xs:
        return {"count": 0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    ys = sorted(float(x) for x in xs)
    p50 = median(ys)
    p95 = ys[min(len(ys) - 1, int(0.95 * (len(ys) - 1)))]
    return {
        "count": len(ys),
        "p50": round(float(p50), 3),
        "p95": round(float(p95), 3),
        "max": round(float(ys[-1]), 3),
    }


class JsonlLogger:
    def __init__(self, path: str = "") -> None:
        self.path = str(path or "").strip()
        self.fp = None
        if self.path:
            self.fp = open(self.path, "a", encoding="utf-8")

    def write(self, row: Dict[str, Any]) -> None:
        rec = dict(row or {})
        # Normalize key fields to simplify downstream extraction/plotting.
        if "event_type" not in rec:
            ev = rec.get("event")
            if ev is not None:
                rec["event_type"] = str(ev)
        if "ts_wall_s" not in rec:
            ts_val = rec.get("ts")
            try:
                if ts_val is not None:
                    rec["ts_wall_s"] = float(ts_val)
            except Exception:
                pass
        if "ts_wall_ms" not in rec:
            ts_val = rec.get("ts")
            try:
                if ts_val is not None:
                    rec["ts_wall_ms"] = 1000.0 * float(ts_val)
            except Exception:
                pass
        if "source_service" not in rec and rec.get("service") is not None:
            rec["source_service"] = str(rec.get("service"))
        line = json_dumps(rec)
        print(line)
        if self.fp is not None:
            self.fp.write(line + "\n")
            self.fp.flush()

    def close(self) -> None:
        if self.fp is not None:
            self.fp.close()
            self.fp = None
