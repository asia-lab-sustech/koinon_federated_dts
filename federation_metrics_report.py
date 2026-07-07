import argparse
import json
import os
import re
from collections import Counter, defaultdict
from statistics import median
from typing import Any, Dict, List


def _stats(xs: List[float]) -> Dict[str, float]:
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


def parse_jsonl(path: str) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    if not path or not os.path.exists(path):
        return out
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            s = ln.strip()
            if not s:
                continue
            try:
                out.append(dict(json.loads(s)))
            except Exception:
                continue
    return out


def extract_latencies(rows: List[Dict[str, Any]]) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = defaultdict(list)
    for r in rows:
        for k in ["latency_ms", "onboarding_latency_ms", "catalog_upsert_latency_ms", "discovery_latency_ms"]:
            if k not in r:
                continue
            v = r.get(k)
            try:
                if isinstance(v, dict):
                    continue
                out[k].append(float(v))
            except Exception:
                pass
    return out


def _query_resp_stats_from_fed_debug(path: str) -> Dict[str, Any]:
    if not path or not os.path.exists(path):
        return {"count": 0, "delta_sec": {"count": 0, "p50": 0.0, "p95": 0.0, "max": 0.0}}
    q_ts: Dict[str, float] = {}
    deltas: List[float] = []
    pat = re.compile(r"t=([0-9.]+)\s+evt=FED_BOOTSTRAP_DISCOVERY_(QUERY|RESP).*req_id=([^ ]+)")
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            m = pat.search(ln)
            if not m:
                continue
            t = float(m.group(1))
            kind = str(m.group(2))
            req_id = str(m.group(3))
            if kind == "QUERY":
                q_ts[req_id] = t
            elif req_id in q_ts:
                deltas.append(float(t - q_ts[req_id]))
    return {"count": len(deltas), "delta_sec": _stats(deltas)}


def _rows_time_span(rows: List[Dict[str, Any]]) -> Dict[str, float]:
    if not rows:
        return {"start_ts": 0.0, "end_ts": 0.0, "duration_sec": 0.0}
    ts = [float(r.get("ts", 0.0) or 0.0) for r in rows if str(r.get("ts", "")) != ""]
    if not ts:
        return {"start_ts": 0.0, "end_ts": 0.0, "duration_sec": 0.0}
    t0 = min(ts)
    t1 = max(ts)
    return {"start_ts": round(t0, 3), "end_ts": round(t1, 3), "duration_sec": round(t1 - t0, 3)}


def main() -> int:
    ap = argparse.ArgumentParser(description="Summarize federation middleware metrics from JSONL logs")
    ap.add_argument("--service-logs", nargs="+", default=[], help="JSONL logs from split services")
    ap.add_argument("--connector-log", default="", help="Optional connector stdout redirected to JSONL")
    ap.add_argument("--membership-log", default="", help="Path to membership.jsonl")
    ap.add_argument("--catalog-log", default="", help="Path to catalog.jsonl")
    ap.add_argument("--discovery-log", default="", help="Path to discovery.jsonl")
    ap.add_argument("--metrics-log", default="", help="Path to metrics.jsonl")
    ap.add_argument("--fed-debug-log", default="", help="Optional fed_outcomes debug txt for discovery query->resp delay")
    args = ap.parse_args()

    service_rows: List[Dict[str, Any]] = []
    for p in args.service_logs:
        service_rows.extend(parse_jsonl(p))
    for p in [args.membership_log, args.catalog_log, args.discovery_log, args.metrics_log]:
        if p:
            service_rows.extend(parse_jsonl(p))

    conn_rows = parse_jsonl(args.connector_log) if args.connector_log else []
    membership_rows = [r for r in service_rows if str(r.get("service", "")) == "membership"]
    catalog_rows = [r for r in service_rows if str(r.get("service", "")) == "catalog"]
    discovery_rows = [r for r in service_rows if str(r.get("service", "")) == "discovery"]

    svc_event_ctr = Counter()
    svc_type_ctr = Counter()
    member_last = {}

    for r in service_rows:
        svc = str(r.get("service", "unknown") or "unknown")
        evt = str(r.get("event", "unknown") or "unknown")
        svc_event_ctr[f"{svc}:{evt}"] += 1
        svc_type_ctr[svc] += 1
        if evt in ("membership_registered", "membership_refreshed", "membership_suspended"):
            gid = str(r.get("gateway_id", "") or "")
            if gid:
                member_last[gid] = evt

    member_transitions = Counter(str(r.get("event", "")) for r in membership_rows)
    member_by_gateway = Counter(str(r.get("gateway_id", "")) for r in membership_rows if str(r.get("gateway_id", "")))

    catalog_evt = Counter(str(r.get("event", "")) for r in catalog_rows)
    catalog_by_gateway = Counter(str(r.get("gateway_id", "")) for r in catalog_rows if str(r.get("gateway_id", "")))

    discovery_evt = Counter(str(r.get("event", "")) for r in discovery_rows)
    discovery_n_results = [
        float(r.get("n_results", 0) or 0)
        for r in discovery_rows
        if str(r.get("event", "")) == "discovery_query_resp"
    ]
    discovery_modes = Counter(
        str(r.get("result_mode", "service") or "service")
        for r in discovery_rows
        if str(r.get("event", "")) == "discovery_query_resp"
    )

    lats = extract_latencies(service_rows)
    lat_summary = {k: _stats(v) for k, v in lats.items()}

    conn_event_ctr = Counter(str(r.get("event", "unknown") or "unknown") for r in conn_rows)
    query_resp_stats = _query_resp_stats_from_fed_debug(args.fed_debug_log)

    report = {
        "services": {
            "n_rows": len(service_rows),
            "time_span": _rows_time_span(service_rows),
            "rows_by_service": dict(svc_type_ctr),
            "events": dict(svc_event_ctr),
            "latencies": lat_summary,
            "members_last_state": member_last,
            "n_members_seen": len(member_last),
        },
        "membership": {
            "n_rows": len(membership_rows),
            "time_span": _rows_time_span(membership_rows),
            "events": dict(member_transitions),
            "active_gateways_seen": len(member_by_gateway),
            "top_gateways_by_events": member_by_gateway.most_common(10),
        },
        "catalog": {
            "n_rows": len(catalog_rows),
            "time_span": _rows_time_span(catalog_rows),
            "events": dict(catalog_evt),
            "upsert_count": int(catalog_evt.get("catalog_upsert", 0)),
            "refresh_count": int(catalog_evt.get("catalog_refresh", 0)),
            "changed_ratio": round(
                float(catalog_evt.get("catalog_upsert", 0))
                / max(1, int(catalog_evt.get("catalog_upsert", 0) + catalog_evt.get("catalog_refresh", 0))),
                3,
            ),
            "distinct_gateways_seen": len(catalog_by_gateway),
        },
        "discovery": {
            "n_rows": len(discovery_rows),
            "time_span": _rows_time_span(discovery_rows),
            "events": dict(discovery_evt),
            "query_resp_count": int(discovery_evt.get("discovery_query_resp", 0)),
            "result_mode_counts": dict(discovery_modes),
            "n_results_stats": _stats(discovery_n_results),
            "query_to_resp_from_fed_debug": query_resp_stats,
        },
        "connector": {
            "n_rows": len(conn_rows),
            "events": dict(conn_event_ctr),
            "local_to_fed_forwards": int(conn_event_ctr.get("fwd_local_to_fed", 0)),
            "fed_to_local_forwards": int(conn_event_ctr.get("fwd_fed_to_local", 0)),
            "membership_register_pub": int(conn_event_ctr.get("membership_register_pub", 0)),
            "membership_heartbeat_pub": int(conn_event_ctr.get("membership_heartbeat_pub", 0)),
            "catalog_upsert_pub": int(conn_event_ctr.get("catalog_upsert_pub", 0)),
        },
        "kpi_notes": [
            "Membership latencies are service-side processing times; use fed_debug query->resp for end-to-end probe delay.",
            "Catalog upsert vs refresh separates capability changes from periodic re-advertisement.",
            "Discovery result_mode counts indicate DT-level versus service-level discovery usage.",
            "Use connector counters to verify control/data plane continuity at DT boundary.",
        ],
    }

    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
