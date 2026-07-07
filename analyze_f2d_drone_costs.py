#!/usr/bin/env python3
"""
Extract F2D drone benefit/cost evidence from a completed experiment bundle.

The script is intentionally read-only. It reconstructs:
  - traffic-domain outcomes for F2/F2D,
  - drone request/context/use events,
  - physical waypoint timing from Crazyflie mission traces,
  - MQTT topic/message/payload overhead.

Current F2D runs publish a consolidated downstream context after the physical
mission completes. Therefore waypoint timing is physical inspection timing; it
is not yet per-waypoint context delivery unless future runs emit partial
waypoint contexts.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional


def _load_jsonl(path: str | Path) -> Iterable[Dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except Exception:
                continue
            if isinstance(row, dict):
                yield row


def _event_name(row: Dict[str, Any]) -> str:
    return str(row.get("event_type") or row.get("event") or "")


def _as_float(x: Any, default: float = float("nan")) -> float:
    try:
        v = float(x)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def _payload_size(payload: Any) -> int:
    if isinstance(payload, (dict, list)):
        return len(json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8"))
    if isinstance(payload, str):
        return len(payload.encode("utf-8"))
    if isinstance(payload, bytes):
        return len(payload)
    return 0


def _find_one(root: Path, pattern: str) -> Optional[Path]:
    xs = sorted(root.glob(pattern))
    return xs[0] if xs else None


def _find_many(root: Path, pattern: str) -> List[Path]:
    return sorted(root.glob(pattern))


def _write_csv(path: Path, rows: List[Dict[str, Any]], fieldnames: Optional[List[str]] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: List[str] = []
        for row in rows:
            for k in row.keys():
                if k not in keys:
                    keys.append(k)
        fieldnames = keys
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _route_metrics(root: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for csv_path in _find_many(root, "**/ev_matrix_results.csv"):
        with csv_path.open("r", encoding="utf-8", newline="") as f:
            for row in csv.DictReader(f):
                out = dict(row)
                out["source_file"] = str(csv_path)
                rows.append(out)
    return rows


def _event_rows(root: Path, mode: str, route: str) -> List[Dict[str, Any]]:
    pats = [
        f"**/runs/*/route_{route}/{mode}/fed_outcomes.events.jsonl",
        f"**/route_{route}/{mode}/fed_outcomes.events.jsonl",
    ]
    rows: List[Dict[str, Any]] = []
    seen = set()
    for pat in pats:
        for path in _find_many(root, pat):
            if path in seen:
                continue
            seen.add(path)
            for row in _load_jsonl(path):
                row["_source_file"] = str(path)
                rows.append(row)
    return rows


def _context_chain(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    interesting = {
        "f2.drone_context.requested",
        "f2.drone_context.received",
        "f2.drone_context.used",
        "f2.drone_context.rejected",
        "f2.drone_context.stale",
        "f2.drone_context.conflict_with_peer",
        "downstream_context.external_rx",
        "f2d.mobile_passive.received",
        "f2d.mobile_passive.used",
        "f2d.mobile_passive.blockage_detected",
        "f2d.ev_advisory.reroute_recommended",
        "f2d.queue_release.requested",
        "f2d.queue_release.applied",
    }
    rows: List[Dict[str, Any]] = []
    fields = [
        "sim_time",
        "request_id",
        "ev_id",
        "requester_tls",
        "provider_id",
        "worst_edge",
        "worst_edge_offset",
        "blocked",
        "reason",
        "confidence",
        "selected_action",
        "decision_source",
        "request_latency_ms",
        "request_to_drone_rx_latency_ms",
        "response_latency_ms",
        "mission_latency_ms",
        "observation_latency_ms",
        "sumo_proxy_latency_ms",
        "drone_publish_to_realworld_rx_latency_ms",
        "request_to_realworld_rx_latency_ms",
        "context_age_ms",
        "request_payload_size_bytes",
        "response_payload_size_bytes",
        "drone_rx_payload_size_bytes",
    ]
    for r in events:
        ev = _event_name(r)
        if ev not in interesting:
            continue
        out = {"event_type": ev}
        for k in fields:
            if k in r:
                out[k] = r.get(k)
        target_edges = r.get("target_edges")
        if isinstance(target_edges, list):
            out["target_edges_n"] = len(target_edges)
            out["target_edges"] = "|".join(str(x) for x in target_edges)
        rows.append(out)
    return rows


def _latency_summary(chain_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    metrics = [
        "request_latency_ms",
        "request_to_drone_rx_latency_ms",
        "response_latency_ms",
        "mission_latency_ms",
        "observation_latency_ms",
        "sumo_proxy_latency_ms",
        "drone_publish_to_realworld_rx_latency_ms",
        "request_to_realworld_rx_latency_ms",
        "context_age_ms",
    ]
    rows: List[Dict[str, Any]] = []
    for event_type in sorted(set(str(r.get("event_type", "")) for r in chain_rows)):
        subset = [r for r in chain_rows if str(r.get("event_type", "")) == event_type]
        for m in metrics:
            vals = [_as_float(r.get(m)) for r in subset]
            vals = [v for v in vals if math.isfinite(v) and v >= 0.0]
            if not vals:
                continue
            rows.append(
                {
                    "event_type": event_type,
                    "metric": m,
                    "n": len(vals),
                    "min": min(vals),
                    "mean": mean(vals),
                    "max": max(vals),
                }
            )
    return rows


def _waypoint_timeline(root: Path, request_id: str = "") -> List[Dict[str, Any]]:
    trace_paths = _find_many(root, "**/route6_2k_sumo_lab_sliding_window_full_route*.jsonl")
    if not trace_paths:
        trace_paths = _find_many(root, "**/*sliding_window*.jsonl")
    rows: List[Dict[str, Any]] = []
    by_req: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for path in trace_paths:
        for row in _load_jsonl(path):
            row["_source_file"] = str(path)
            by_req[str(row.get("request_id", "") or "")].append(row)
    if request_id and request_id in by_req:
        selected_req = request_id
    else:
        complete = [
            (req, rs)
            for req, rs in by_req.items()
            if any(_event_name(r) == "checkpoint_mission.started" for r in rs)
            and any(_event_name(r) == "checkpoint_mission.completed" for r in rs)
        ]
        if not complete:
            return []
        selected_req, _ = max(complete, key=lambda item: len(item[1]))
    rs = by_req[selected_req]
    starts = [float(r["ts"]) for r in rs if _event_name(r) == "checkpoint_mission.started" and "ts" in r]
    completes = [float(r["ts"]) for r in rs if _event_name(r) == "checkpoint_mission.completed" and "ts" in r]
    if not starts:
        return []
    start_ts = min(starts)
    completed_ts = max(completes) if completes else float("nan")
    targets: Dict[int, Dict[str, Any]] = {}
    reached: Dict[int, Dict[str, Any]] = {}
    hover: Dict[int, Dict[str, Any]] = {}
    for r in rs:
        idx = r.get("waypoint_index")
        if idx is None:
            continue
        idx = int(idx)
        ev = _event_name(r)
        if ev == "checkpoint.target":
            targets[idx] = r
        elif ev == "checkpoint.reached_or_command_returned":
            reached[idx] = r
        elif ev == "checkpoint.hover_complete":
            hover[idx] = r
    for idx in sorted(targets):
        t = targets[idx]
        wp = t.get("waypoint") if isinstance(t.get("waypoint"), dict) else {}
        target = t.get("target") if isinstance(t.get("target"), dict) else {}
        reach_ts = _as_float(reached.get(idx, {}).get("ts"))
        hover_ts = _as_float(hover.get(idx, {}).get("ts"))
        rows.append(
            {
                "request_id": selected_req,
                "mission_start_ts": start_ts,
                "mission_complete_ts": completed_ts,
                "mission_duration_ms": (completed_ts - start_ts) * 1000.0 if math.isfinite(completed_ts) else "",
                "waypoint_index": idx,
                "waypoint_id": wp.get("id", ""),
                "node": wp.get("node", ""),
                "edge": wp.get("edge", ""),
                "kind": wp.get("kind", ""),
                "node_type": wp.get("node_type", ""),
                "region_id": wp.get("region_id", ""),
                "region_label": wp.get("region_label", ""),
                "lab_x": wp.get("x", ""),
                "lab_y": wp.get("y", ""),
                "lab_z": wp.get("z", target.get("z", "")),
                "sumo_x": wp.get("sumo_x", ""),
                "sumo_y": wp.get("sumo_y", ""),
                "target_wall_ts": t.get("ts", ""),
                "reached_wall_ts": reach_ts if math.isfinite(reach_ts) else "",
                "hover_complete_wall_ts": hover_ts if math.isfinite(hover_ts) else "",
                "target_elapsed_s": _as_float(t.get("ts")) - start_ts,
                "reached_elapsed_s": reach_ts - start_ts if math.isfinite(reach_ts) else "",
                "hover_complete_elapsed_s": hover_ts - start_ts if math.isfinite(hover_ts) else "",
                "hover_sec": target.get("hover_sec", ""),
                "velocity_mps": target.get("velocity_mps", ""),
            }
        )
    return rows


def _topic_overhead(root: Path) -> List[Dict[str, Any]]:
    rows_by_topic: Dict[str, Dict[str, Any]] = {}
    for path in _find_many(root, "**/raw_messages.jsonl"):
        for rec in _load_jsonl(path):
            topic = str(rec.get("topic") or rec.get("wire_topic") or rec.get("mqtt_topic") or "")
            if not topic:
                continue
            payload = rec.get("payload")
            size = _payload_size(payload)
            row = rows_by_topic.setdefault(
                topic,
                {
                    "topic": topic,
                    "message_count": 0,
                    "payload_bytes": 0,
                    "source_files": set(),
                },
            )
            row["message_count"] += 1
            row["payload_bytes"] += size
            row["source_files"].add(str(path))
    rows: List[Dict[str, Any]] = []
    for topic, row in rows_by_topic.items():
        b = int(row["payload_bytes"])
        rows.append(
            {
                "topic": topic,
                "message_count": int(row["message_count"]),
                "payload_bytes": b,
                "payload_kb": b / 1024.0,
                "payload_mb": b / (1024.0 * 1024.0),
                "source_file_count": len(row["source_files"]),
            }
        )
    rows.sort(key=lambda r: int(r["message_count"]), reverse=True)
    return rows


def _summary(route_metrics: List[Dict[str, Any]], chain: List[Dict[str, Any]], waypoints: List[Dict[str, Any]], overhead: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for r in route_metrics:
        if str(r.get("mode", "")).upper() in {"F2", "F2D"}:
            rows.append(
                {
                    "section": "domain_outcome",
                    "metric": f"{r.get('mode')}_travel_time_s",
                    "value": r.get("travel_time_s", ""),
                    "unit": "s",
                    "detail": f"route={r.get('route_id')} density={r.get('density_label')} arrived={r.get('arrived')}",
                }
            )
            for k in ("waiting_time_s", "time_loss_s", "waiting_count_n", "wall_elapsed_s"):
                rows.append({"section": "domain_outcome", "metric": f"{r.get('mode')}_{k}", "value": r.get(k, ""), "unit": "s_or_count", "detail": ""})
    for event in ("f2.drone_context.received", "f2.drone_context.used", "f2d.mobile_passive.used", "f2d.ev_advisory.reroute_recommended"):
        n = sum(1 for r in chain if r.get("event_type") == event)
        rows.append({"section": "context_use", "metric": event, "value": n, "unit": "count", "detail": ""})
    if waypoints:
        rows.append({"section": "physical_mission", "metric": "waypoints_n", "value": len(waypoints), "unit": "count", "detail": ""})
        rows.append({"section": "physical_mission", "metric": "mission_duration_ms", "value": waypoints[0].get("mission_duration_ms", ""), "unit": "ms", "detail": ""})
    for topic in overhead[:10]:
        rows.append({"section": "communication", "metric": topic["topic"], "value": topic["message_count"], "unit": "messages", "detail": f"{topic['payload_kb']:.2f} KB"})
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze F2D drone benefit/cost logs")
    ap.add_argument("--experiment-root", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--mode", default="F2D")
    ap.add_argument("--route-id", default="6")
    ap.add_argument("--request-id", default="")
    args = ap.parse_args()

    root = Path(args.experiment_root).expanduser().resolve()
    out = Path(args.out_dir).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    route_metrics = _route_metrics(root)
    events = _event_rows(root, str(args.mode).upper(), str(args.route_id))
    chain = _context_chain(events)
    latency = _latency_summary(chain)
    request_id = str(args.request_id or "")
    if not request_id:
        for row in chain:
            if row.get("event_type") in ("f2.drone_context.received", "f2d.mobile_passive.received") and row.get("request_id"):
                request_id = str(row.get("request_id"))
                break
    waypoints = _waypoint_timeline(root, request_id=request_id)
    overhead = _topic_overhead(root)
    summary = _summary(route_metrics, chain, waypoints, overhead)

    _write_csv(out / "f2d_route_metrics.csv", route_metrics)
    _write_csv(out / "f2d_context_chain.csv", chain)
    _write_csv(out / "f2d_latency_summary.csv", latency)
    _write_csv(out / "f2d_waypoint_timeline.csv", waypoints)
    _write_csv(out / "f2d_topic_overhead.csv", overhead)
    _write_csv(out / "f2d_summary.csv", summary)

    print(json.dumps({
        "out_dir": str(out),
        "route_metrics_rows": len(route_metrics),
        "event_rows": len(events),
        "context_chain_rows": len(chain),
        "waypoint_rows": len(waypoints),
        "topic_rows": len(overhead),
        "request_id": request_id,
        "outputs": [
            "f2d_route_metrics.csv",
            "f2d_context_chain.csv",
            "f2d_latency_summary.csv",
            "f2d_waypoint_timeline.csv",
            "f2d_topic_overhead.csv",
            "f2d_summary.csv",
        ],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
