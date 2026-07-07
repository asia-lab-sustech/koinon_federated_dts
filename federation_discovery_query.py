import argparse
import json
import threading
import time
import uuid

import paho.mqtt.client as mqtt


def main() -> int:
    ap = argparse.ArgumentParser(description="Send one discovery query to federation core services")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--requester", required=True)
    ap.add_argument("--requester-role", default="orchestrator")
    ap.add_argument("--role-filter", default="")
    ap.add_argument("--event-filter", default="")
    ap.add_argument("--service-filter", default="")
    ap.add_argument("--result-mode", default="dt", choices=["service", "dt"])
    ap.add_argument("--dt-description-contains", default="")
    ap.add_argument("--geo-scope-type", default="")
    ap.add_argument("--geo-scope-id", default="")
    ap.add_argument("--geo-scope-city", default="")
    ap.add_argument("--geo-scope-zone", default="")
    ap.add_argument("--policy-tags-any", default="")
    ap.add_argument("--policy-tags-all", default="")
    ap.add_argument("--ownership-organization", default="")
    ap.add_argument("--ownership-domain", default="")
    ap.add_argument("--ownership-operator", default="")
    ap.add_argument("--interface-version", default="")
    ap.add_argument("--qos-sla-max-update-period-sec", default="")
    ap.add_argument("--qos-sla-max-latency-budget-ms", default="")
    ap.add_argument("--qos-sla-availability-target", default="")
    ap.add_argument("--query-topic", default="federation/discovery/query")
    ap.add_argument("--reply-prefix", default="federation/discovery/resp")
    ap.add_argument("--timeout-sec", type=float, default=5.0)
    args = ap.parse_args()

    request_id = f"dq-{uuid.uuid4().hex[:10]}"
    reply_topic = f"{args.reply_prefix}/{args.requester}"
    got = {"done": False}

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dq-{args.requester}")

    def on_connect(c, _u, _f, rc, _p):
        print(f"[DQ] connected rc={rc}")
        c.subscribe(reply_topic, qos=0)
        filters = {
            "role": args.role_filter,
            "event_type": args.event_filter,
            "service_name": args.service_filter,
            "result_mode": args.result_mode,
            "dt_description_contains": args.dt_description_contains,
            "geo_scope_type": args.geo_scope_type,
            "geo_scope_id": args.geo_scope_id,
            "geo_scope_city": args.geo_scope_city,
            "geo_scope_zone": args.geo_scope_zone,
            "policy_tags_any": [x.strip() for x in str(args.policy_tags_any or "").split(",") if x.strip()],
            "policy_tags_all": [x.strip() for x in str(args.policy_tags_all or "").split(",") if x.strip()],
            "ownership_organization": args.ownership_organization,
            "ownership_domain": args.ownership_domain,
            "ownership_operator": args.ownership_operator,
            "interface_version": args.interface_version,
            "qos_sla_max_update_period_sec": args.qos_sla_max_update_period_sec,
            "qos_sla_max_latency_budget_ms": args.qos_sla_max_latency_budget_ms,
            "qos_sla_availability_target": args.qos_sla_availability_target,
        }
        filters = {k: v for k, v in filters.items() if v not in ("", [], None)}
        payload = {
            "schema": "federation.discovery.v1",
            "request_id": request_id,
            "requester": args.requester,
            "requester_role": args.requester_role,
            "reply_topic": reply_topic,
            "filters": filters,
            "max_results": 100,
            "ts": time.time(),
        }
        c.publish(args.query_topic, json.dumps(payload, ensure_ascii=True, separators=(",", ":")))
        print(f"[DQ] query sent request_id={request_id} topic={args.query_topic}")

    def on_message(_c, _u, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            payload = {}
        if str(payload.get("request_id", "")) != request_id:
            return
        print(json.dumps(payload, indent=2, ensure_ascii=True))
        got["done"] = True

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.mqtt_host, int(args.mqtt_port), 60)
    client.loop_start()

    t_end = time.time() + max(0.1, float(args.timeout_sec))
    try:
        while not got["done"] and time.time() < t_end:
            time.sleep(0.05)
    finally:
        client.loop_stop()
        client.disconnect()

    if not got["done"]:
        print(f"[DQ] timeout waiting for response on {reply_topic}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
