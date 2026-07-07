import argparse
import json
import os
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


def main() -> int:
    ap = argparse.ArgumentParser(description="Summarize federation middleware metrics from JSONL logs")
    ap.add_argument("--service-logs", nargs="+", default=[], help="JSONL logs from split services")
    ap.add_argument("--connector-log", default="", help="Optional connector stdout redirected to JSONL")
    args = ap.parse_args()

    service_rows: List[Dict[str, Any]] = []
    for p in args.service_logs:
        service_rows.extend(parse_jsonl(p))

    conn_rows = parse_jsonl(args.connector_log) if args.connector_log else []

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

    lats = extract_latencies(service_rows)
    lat_summary = {k: _stats(v) for k, v in lats.items()}

    conn_event_ctr = Counter(str(r.get("event", "unknown") or "unknown") for r in conn_rows)

    report = {
        "services": {
            "n_rows": len(service_rows),
            "rows_by_service": dict(svc_type_ctr),
            "events": dict(svc_event_ctr),
            "latencies": lat_summary,
            "members_last_state": member_last,
            "n_members_seen": len(member_last),
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
            "Onboarding latency: membership register processing latency.",
            "Catalog latency: upsert processing latency in catalog service.",
            "Discovery latency: query response latency in discovery service.",
            "Use connector counters to verify control/data plane continuity at DT boundary.",
        ],
    }

    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
