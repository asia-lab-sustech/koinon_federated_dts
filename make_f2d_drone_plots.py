#!/usr/bin/env python3
"""Generate paper-facing plots for F2D drone-augmented federation runs.

The script intentionally uses only the Python standard library plus NumPy, so it
can run on the server without matplotlib. It reads a drone experiment result
folder, optional F2P reference results, route-level event logs, core discovery
logs, and edge drone raw-message logs when present.
"""

from __future__ import annotations

import argparse
import csv
import glob
import html
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean
from typing import Iterable, Any

import numpy as np

MODE_DISPLAY = {
    "F2": "FCDP",
    "F2P": "FCDP-P",
    "F2D": "FCDP-D",
    "F2D-Q": "FCDP-DQ",
    "F2D-OFF": "FCDP-D unavailable",
}
MODE_COLORS = {
    "F2": "#2D7DD2",
    "F2P": "#1B9E77",
    "F2D": "#C65D32",
    "F2D-Q": "#8A5AAB",
    "F2D-OFF": "#596773",
}
EVENT_COUNT_MAP = {
    "f2.drone_context.requested": "Drone requests",
    "f2.drone_context.request_skip": "Request skips",
    "f2.drone_context.received": "Context received",
    "downstream_context.external_rx": "External context RX",
    "f2.drone_context.used": "Context used",
    "f2d.mobile_passive.used": "Mobile-passive used",
    "f2.drone_context.stale": "Stale rejected",
    "f2d.mobile_passive.stale": "Mobile stale",
    "f2d.mobile_passive.blockage_detected": "Blockage detected",
    "f2d.ev_advisory.reroute_recommended": "Reroute advisory",
    "f2.downstream_apply_guard.drone_skip": "Drone guard skip",
    "f2.strict_b1_floor.apply_drone_guard": "B1-floor drone guard",
    "f2d.queue_release.requested": "Queue release requested",
    "f2d.queue_release.applied": "Queue release applied",
}
TRAFFIC_METRICS = [
    ("travel_time_s", "Travel Time (s)"),
    ("waiting_time_s", "Waiting Time (s)"),
    ("time_loss_s", "Time Loss (s)"),
    ("waiting_count_n", "Stops / Waiting Count"),
]
DRONE_LATENCY_METRICS = [
    ("request_latency_ms", "Drone task acceptance"),
    ("request_to_drone_rx_latency_ms", "SI-DT request -> drone RX"),
    ("mission_latency_ms", "Physical/simulated scout mission"),
    ("observation_latency_ms", "Observation query"),
    ("sumo_proxy_latency_ms", "SUMO proxy query"),
    ("request_to_publish_latency_ms", "SI-DT request -> drone publish"),
    ("drone_rx_to_publish_latency_ms", "Drone RX -> drone publish"),
    ("drone_publish_to_realworld_rx_latency_ms", "Drone publish -> SI-DT RX"),
    ("request_to_realworld_rx_latency_ms", "SI-DT request -> SI-DT RX"),
    ("response_latency_ms", "Request -> context created"),
    ("context_age_ms", "Context age at use"),
]
DRONE_PAYLOAD_METRICS = [
    ("request_payload_size_bytes", "Request payload"),
    ("drone_rx_payload_size_bytes", "Drone RX payload"),
    ("response_payload_size_bytes", "Context response payload"),
]
LATENCY_CHAIN_COMPONENTS = [
    ("request_to_drone_rx_latency_ms", "Request transit"),
    ("mission_latency_ms", "Scout mission"),
    ("sumo_proxy_latency_ms", "SUMO observation"),
    ("drone_publish_to_realworld_rx_latency_ms", "Response transit"),
]


def safe_float(value: Any, default: float = math.nan) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except Exception:
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        if value in (None, ""):
            return default
        return int(float(value))
    except Exception:
        return default


def esc(text: object) -> str:
    return html.escape(str(text), quote=True)


class Svg:
    def __init__(self, width: int, height: int, title: str = ""):
        self.width = width
        self.height = height
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            "<defs>",
            "<style>",
            "text{font-family:Arial,Helvetica,sans-serif;fill:#263238}",
            ".title{font-size:24px;font-weight:700}.subtitle{font-size:13px;fill:#56616b}",
            ".axis{stroke:#70808f;stroke-width:1}.grid{stroke:#d9e1e8;stroke-width:1}",
            ".tick{font-size:11px;fill:#596773}.label{font-size:12px;fill:#43515c;font-weight:600}",
            ".xtick{font-size:14px;fill:#43515c;font-weight:700}.small{font-size:10px;fill:#596773}",
            ".legend{font-size:12px;fill:#263238}",
            "</style>",
            "</defs>",
            f'<rect x="0" y="0" width="{width}" height="{height}" fill="#fbfcfd"/>',
        ]
        if title:
            self.text(36, 34, title, cls="title")

    def rect(self, x, y, w, h, fill="#fff", stroke="none", sw=1, rx=0, opacity=1):
        self.parts.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" rx="{rx}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"/>'
        )

    def line(self, x1, y1, x2, y2, stroke="#000", sw=1, opacity=1, dash=None):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"{dash_attr}/>'
        )

    def circle(self, x, y, r=3, fill="#000", stroke="none", sw=1, opacity=1):
        self.parts.append(
            f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{sw}" opacity="{opacity}"/>'
        )

    def polyline(self, pts: Iterable[tuple[float, float]], stroke="#000", sw=2, fill="none", opacity=1, dash=None):
        points = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<polyline points="{points}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'opacity="{opacity}" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>'
        )

    def text(self, x, y, text, cls="", anchor="start", size=None, fill=None, rotate=None):
        cls_attr = f' class="{cls}"' if cls else ""
        size_attr = f' font-size="{size}"' if size else ""
        fill_attr = f' fill="{fill}"' if fill else ""
        transform = f' transform="rotate({rotate} {x:.2f} {y:.2f})"' if rotate else ""
        self.parts.append(
            f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}"{cls_attr}{size_attr}{fill_attr}{transform}>{esc(text)}</text>'
        )

    def finish(self) -> str:
        self.parts.append("</svg>")
        return "\n".join(self.parts)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.finish())


def y_scale(v, vmin, vmax, top, bottom):
    if vmax <= vmin:
        return bottom
    return bottom - (v - vmin) / (vmax - vmin) * (bottom - top)


def draw_y_axis(svg: Svg, x, top, bottom, vmin, vmax, ticks=6, label="", x2=None):
    x2 = x2 if x2 is not None else svg.width - 40
    for i in range(ticks + 1):
        value = vmin + (vmax - vmin) * i / ticks
        y = y_scale(value, vmin, vmax, top, bottom)
        svg.line(x, y, x2, y, stroke="#e6edf2", sw=1)
        svg.line(x - 5, y, x, y, stroke="#70808f")
        svg.text(x - 9, y + 4, f"{value:.0f}", cls="tick", anchor="end")
    svg.line(x, top, x, bottom, stroke="#70808f")
    svg.line(x, bottom, x2, bottom, stroke="#70808f")
    if label:
        svg.text(18, (top + bottom) / 2, label, cls="label", anchor="middle", rotate=-90)


def write_csv(path: Path, rows: list[dict], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        seen = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    keys.append(key)
                    seen.add(key)
        fieldnames = keys
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def infer_mode_from_text(text: str) -> str:
    """Infer experiment mode from scenario namespace/path text."""
    text_u = str(text or "").upper()
    # Check longer/suffixed modes first.
    for mode in ("F2D-Q", "F2D", "F2P", "F2D-OFF", "F2"):
        if f"/{mode}" in text_u or f"_{mode}" in text_u or text_u.endswith(mode):
            return mode
    return ""


def find_result_csv(run_dir: Path) -> Path:
    direct = run_dir / "ev_matrix_results.csv"
    if direct.exists():
        return direct
    matches = sorted(run_dir.glob("**/ev_matrix_results.csv"), key=lambda p: (p.stat().st_mtime, p.stat().st_size))
    if not matches:
        raise FileNotFoundError(f"No ev_matrix_results.csv found under {run_dir}")
    return matches[-1]


def read_result_rows(path: Path, source_label: str) -> list[dict]:
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            row = dict(row)
            row["source_csv"] = str(path)
            row["source_label"] = source_label
            row["route_id_int"] = safe_int(row.get("route_id"), -1)
            row["arrived_int"] = safe_int(row.get("arrived"), 0)
            for field, _ in TRAFFIC_METRICS:
                row[field + "_float"] = safe_float(row.get(field))
            row["wall_elapsed_s_float"] = safe_float(row.get("wall_elapsed_s"))
            row["route_length_m_float"] = safe_float(row.get("route_length_m"))
            rows.append(row)
    return rows


def select_rows(rows: list[dict], scenario_id: str | None, route_id: int | None, modes: list[str]) -> list[dict]:
    out = []
    for row in rows:
        if scenario_id and row.get("scenario_id") != scenario_id:
            continue
        if route_id is not None and row.get("route_id_int") != route_id:
            continue
        if row.get("mode") not in modes:
            continue
        out.append(row)
    return out


def mode_sort_key(mode: str) -> int:
    order = ["F2", "F2P", "F2D", "F2D-OFF", "F2D-Q"]
    return order.index(mode) if mode in order else 99


def local_event_path(run_dir: Path, row: dict) -> Path | None:
    mode = row.get("mode", "")
    scenario = row.get("scenario_id", "")
    route = row.get("route_id", "")
    candidates = sorted(run_dir.glob(f"**/{scenario}/**/route_{route}/{mode}/fed_outcomes.events.jsonl"))
    if not candidates:
        candidates = sorted(run_dir.glob(f"**/{mode}/fed_outcomes.events.jsonl"))
    if candidates:
        return candidates[0]
    return None


def parse_event_logs(run_dir: Path, traffic_rows: list[dict]) -> tuple[list[dict], list[dict], list[dict]]:
    event_rows: list[dict] = []
    metric_rows: list[dict] = []
    context_timeline: list[dict] = []
    numeric_fields = sorted(
        {
            "confidence",
            "decision_deadline_sec",
            "request_latency_ms",
            "response_latency_ms",
            "context_age_ms",
            "request_to_drone_rx_latency_ms",
            "mission_latency_ms",
            "observation_latency_ms",
            "sumo_proxy_latency_ms",
            "request_to_publish_latency_ms",
            "drone_rx_to_publish_latency_ms",
            "request_to_realworld_rx_latency_ms",
            "drone_publish_to_realworld_rx_latency_ms",
            "request_payload_size_bytes",
            "drone_rx_payload_size_bytes",
            "response_payload_size_bytes",
        }
    )
    for row in traffic_rows:
        mode = row.get("mode", "")
        if mode not in {"F2D", "F2D-Q", "F2D-OFF"}:
            continue
        path = local_event_path(run_dir, row)
        if not path or not path.exists():
            continue
        event_counts = Counter()
        with open(path, errors="ignore") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                et = str(obj.get("event_type") or obj.get("event") or "")
                if not et:
                    continue
                event_counts[et] += 1
                if et.startswith("f2.drone_context.") or et.startswith("f2d.mobile_passive.") or et.startswith("f2d.ev_advisory.") or et in {
                    "downstream_context.external_rx",
                    "f2.downstream_apply_guard.drone_skip",
                    "f2.strict_b1_floor.apply_drone_guard",
                    "f2d.queue_release.requested",
                    "f2d.queue_release.applied",
                }:
                    base = {
                        "scenario_id": row.get("scenario_id", ""),
                        "route_id": row.get("route_id", ""),
                        "mode": mode,
                        "event_type": et,
                        "sim_time": safe_float(obj.get("sim_time", obj.get("ts_sim_s"))),
                        "ts_wall_ms": safe_float(obj.get("ts_wall_ms")),
                        "request_id": str(obj.get("request_id", "") or ""),
                        "ev_id": str(obj.get("ev_id", "") or row.get("ev_id", "")),
                        "requester_tls": str(obj.get("requester_tls", "") or ""),
                        "tls_id": str(obj.get("tls_id", "") or ""),
                        "provider_id": str(obj.get("provider_id", "") or ""),
                        "blocked": str(obj.get("blocked", "") or ""),
                        "reason": str(obj.get("reason", "") or ""),
                        "worst_edge": str(obj.get("worst_edge", "") or ""),
                        "selected_action": str(obj.get("selected_action", "") or ""),
                        "decision_source": str(obj.get("decision_source", "") or ""),
                        "target_edges_n": len(obj.get("target_edges", [])) if isinstance(obj.get("target_edges"), list) else safe_int(obj.get("target_edges_n"), 0),
                    }
                    for field in numeric_fields:
                        base[field] = safe_float(obj.get(field))
                    event_rows.append(base)
                    metric_specs = [(name, "ms") for name, _ in DRONE_LATENCY_METRICS]
                    metric_specs.extend((name, "bytes") for name, _ in DRONE_PAYLOAD_METRICS)
                    metric_specs.append(("confidence", "ratio"))
                    for metric_name, unit in metric_specs:
                        value = base.get(metric_name, math.nan)
                        # Runtime traces use -1 as an explicit missing-value sentinel.
                        if isinstance(value, float) and math.isfinite(value) and value >= 0.0:
                            metric_rows.append({**base, "metric_name": metric_name, "metric_value": value, "unit": unit})
                    if et in {"f2.drone_context.used", "f2.drone_context.received", "f2.drone_context.stale"}:
                        context_timeline.append(base)
        for et, n in sorted(event_counts.items()):
            if et in EVENT_COUNT_MAP or et.startswith("f2.drone_context") or et.startswith("f2d.mobile_passive"):
                metric_rows.append({
                    "scenario_id": row.get("scenario_id", ""),
                    "route_id": row.get("route_id", ""),
                    "mode": mode,
                    "event_type": et,
                    "metric_name": "event_count",
                    "metric_value": float(n),
                    "unit": "count",
                })
    return event_rows, metric_rows, context_timeline


def _event_node_id(obj: dict) -> str:
    for key in ("requester_tls", "tls_id", "node_id", "target_tls", "src_tls", "dst_tls"):
        value = str(obj.get(key, "") or "").strip()
        if value:
            return value
    return ""


def _node_activity_category(event_type: str) -> str:
    et = str(event_type or "")
    low = et.lower()
    if et in {"f2.drone_context.requested", "f2d.drone_prescout.requested"}:
        return "Drone request"
    if et in {"f2.drone_context.received", "f2.drone_context.used", "f2d.mobile_passive.used", "f2d.mobile_passive.received"}:
        return "Drone context"
    if "stale" in low or "guard" in low or "skip" in low or "conflict" in low:
        return "Freshness/safety guard"
    if "discovery" in low:
        return "Discovery"
    if "request" in low and "drone" not in low:
        return "Priority request"
    if "decision" in low or "apply" in low or "preemption" in low:
        return "Local decision"
    if "coordination" in low or low.startswith("f2.") or low.startswith("f2d."):
        return "Federated coordination"
    return ""


def parse_node_activity_logs(run_dir: Path, traffic_rows: list[dict]) -> list[dict]:
    rows: list[dict] = []
    for row in traffic_rows:
        mode = row.get("mode", "")
        if mode not in {"F2", "F2D", "F2D-Q", "F2D-OFF"}:
            continue
        path = local_event_path(run_dir, row)
        if not path or not path.exists():
            continue
        counts: Counter[tuple[str, str]] = Counter()
        with open(path, errors="ignore") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                et = str(obj.get("event_type") or obj.get("event") or "")
                cat = _node_activity_category(et)
                node_id = _event_node_id(obj)
                if not cat or not node_id:
                    continue
                counts[(node_id, cat)] += 1
        for (node_id, cat), n in sorted(counts.items()):
            rows.append({
                "scenario_id": row.get("scenario_id", ""),
                "route_id": row.get("route_id", ""),
                "mode": mode,
                "node_id": node_id,
                "activity_category": cat,
                "count": int(n),
                "source_file": str(path),
            })
    return rows


def parse_discovery_logs(core_dirs: list[Path], scenario_id: str | None = None) -> list[dict]:
    rows: list[dict] = []
    for core_dir in core_dirs:
        for fp in sorted(core_dir.glob("**/discovery.jsonl")):
            pending: dict[str, float] = {}
            with open(fp, errors="ignore") as f:
                for line in f:
                    try:
                        obj = json.loads(line)
                    except Exception:
                        continue
                    event = obj.get("event")
                    request_id = str(obj.get("request_id", "") or "")
                    purpose = str(obj.get("purpose", "") or "")
                    is_drone = "drone" in purpose.lower() or "aerial" in str(obj).lower() or "crazyflie" in str(obj).lower()
                    if scenario_id and scenario_id not in str(obj):
                        # Keep global core events if no explicit namespace is present.
                        if str(obj.get("reply_namespace", "")):
                            continue
                    if event == "discovery_query_in":
                        pending[request_id] = safe_float(obj.get("ts_wall_ms"), safe_float(obj.get("ts_wall_s")) * 1000.0)
                    elif event == "discovery_query_resp":
                        ts = safe_float(obj.get("ts_wall_ms"), safe_float(obj.get("ts_wall_s")) * 1000.0)
                        latency = safe_float(obj.get("latency_ms"))
                        if not math.isfinite(latency) and request_id in pending:
                            latency = ts - pending[request_id]
                        rows.append({
                            "source_file": str(fp),
                            "request_id": request_id,
                            "purpose": purpose,
                            "is_drone_query": int(is_drone),
                            "reply_namespace": str(obj.get("reply_namespace", "") or ""),
                            "mode": infer_mode_from_text(str(obj.get("reply_namespace", "") or "")),
                            "n_results": safe_int(obj.get("n_results"), 0),
                            "rejected_namespace_mismatch": safe_int(obj.get("rejected_namespace_mismatch"), 0),
                            "rejected_inactive_members": safe_int(obj.get("rejected_inactive_members"), 0),
                            "latency_ms": latency,
                        })
    return rows


def parse_edge_raw(edge_raw_files: list[Path]) -> list[dict]:
    rows: list[dict] = []
    for fp in edge_raw_files:
        with open(fp, errors="ignore") as f:
            for line in f:
                raw_bytes = len(line.encode("utf-8", "ignore"))
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                topic = str(obj.get("topic") or obj.get("wire_topic") or "")
                payload = obj.get("payload")
                payload_s = ""
                payload_type = ""
                if isinstance(payload, dict):
                    payload_type = str(payload.get("event_type") or payload.get("event") or payload.get("type") or "")
                    try:
                        payload_s = json.dumps(payload, separators=(",", ":"))
                    except Exception:
                        payload_s = str(payload)
                elif payload is not None:
                    payload_s = str(payload)
                payload_bytes = len(payload_s.encode("utf-8", "ignore")) if payload_s else 0
                if "drone" not in topic.lower() and "downstream" not in topic.lower() and "crazyflie" not in payload_s.lower():
                    continue
                mode = infer_mode_from_text(topic) or infer_mode_from_text(payload_s)
                rows.append({
                    "source_file": str(fp),
                    "topic": topic,
                    "mode": mode,
                    "payload_type": payload_type,
                    "raw_bytes": raw_bytes,
                    "payload_bytes": payload_bytes,
                    "direction": str(obj.get("direction") or obj.get("iface") or ""),
                })
    return rows


def parse_fnm_traces(trace_files: list[Path]) -> list[dict]:
    """Parse FNM trace.jsonl files into hop-level middleware rows.

    These rows are intentionally generic: they describe FNM mediation work
    independent of the traffic-domain payload, which makes them useful for
    communication/computation overhead plots in the F2D subsection.
    """
    rows: list[dict] = []
    interesting_events = {
        "fnm.route.local_to_fed",
        "fnm.stage.local_to_fed",
        "fnm.delivery.local_to_fed",
        "fnm.route.fed_to_local",
        "fnm.stage.fed_to_local",
        "fnm.delivery.fed_to_local",
        "fnm.mqtt.publish",
        "fnm.mqtt.publish_attempt_not_connected",
        "fnm.mqtt.publish_error",
    }
    for fp in trace_files:
        node_id = fp.parent.name
        with open(fp, errors="ignore") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                event = str(obj.get("event", "") or "")
                if event not in interesting_events:
                    continue
                topic_text = " ".join(
                    str(obj.get(k, "") or "")
                    for k in ("src", "dst", "topic", "wire_topic", "rule", "artefact_kind")
                )
                if (
                    "drone" not in topic_text.lower()
                    and "downstream" not in topic_text.lower()
                    and "crazyflie" not in topic_text.lower()
                    and node_id.lower().find("drone") < 0
                ):
                    continue
                direction = ""
                if "local_to_fed" in event:
                    direction = "local_to_fed"
                elif "fed_to_local" in event:
                    direction = "fed_to_local"
                elif str(obj.get("iface", "")):
                    direction = str(obj.get("iface"))
                rows.append({
                    "source_file": str(fp),
                    "node_dir": node_id,
                    "ts": safe_float(obj.get("ts")),
                    "event": event,
                    "direction": direction,
                    "rule": str(obj.get("rule", "") or ""),
                    "artefact_kind": str(obj.get("artefact_kind", "") or ""),
                    "status": str(obj.get("status", "") or ""),
                    "src": str(obj.get("src", "") or obj.get("topic", "") or ""),
                    "dst": str(obj.get("dst", "") or ""),
                    "wire_topic": str(obj.get("wire_topic", "") or ""),
                    "message_id": str(obj.get("message_id", "") or ""),
                    "payload_size_bytes": safe_int(obj.get("payload_size_bytes"), 0),
                    "duration_ms": safe_float(obj.get("duration_ms")),
                    "local_ingest_to_schema_ms": safe_float(obj.get("local_ingest_to_schema_ms")),
                    "schema_to_fed_publish_ms": safe_float(obj.get("schema_to_fed_publish_ms")),
                    "local_to_fed_total_ms": safe_float(obj.get("local_to_fed_total_ms")),
                    "fed_publish_to_remote_receive_ms": safe_float(obj.get("fed_publish_to_remote_receive_ms")),
                    "remote_receive_to_local_invoke_ms": safe_float(obj.get("remote_receive_to_local_invoke_ms")),
                    "fed_to_local_total_ms": safe_float(obj.get("fed_to_local_total_ms")),
                    "origin_to_local_invoke_ms": safe_float(obj.get("origin_to_local_invoke_ms")),
                    "ok": str(obj.get("ok", "") or ""),
                    "rc": str(obj.get("rc", "") or ""),
                })
    return rows


def summarize_values(rows: list[dict], field: str, group_fields: list[str]) -> list[dict]:
    grouped: dict[tuple, list[float]] = defaultdict(list)
    for r in rows:
        v = safe_float(r.get(field))
        if math.isfinite(v):
            grouped[tuple(r.get(g, "") for g in group_fields)].append(v)
    out = []
    for key, vals in sorted(grouped.items()):
        arr = np.array(vals, dtype=float)
        row = {g: key[i] for i, g in enumerate(group_fields)}
        row.update({
            "n": len(vals),
            "mean": f"{float(np.mean(arr)):.6f}",
            "median": f"{float(np.median(arr)):.6f}",
            "p95": f"{float(np.percentile(arr, 95)):.6f}",
            "min": f"{float(np.min(arr)):.6f}",
            "max": f"{float(np.max(arr)):.6f}",
        })
        out.append(row)
    return out


def plot_traffic_bars(rows: list[dict], path: Path, title_suffix: str) -> None:
    modes = sorted({r["mode"] for r in rows}, key=mode_sort_key)
    svg = Svg(1320, 760, f"F2D aerial context traffic outcomes{title_suffix}")
    svg.text(36, 56, "Traffic metrics for the same route/scenario. Lower is better.", cls="subtitle")
    panel_w, panel_h = 620, 280
    positions = [(76, 112), (720, 112), (76, 430), (720, 430)]
    for (metric, label), (left, top) in zip(TRAFFIC_METRICS, positions):
        bottom = top + 210
        right = left + 520
        vals = [safe_float(r.get(metric)) for r in rows if math.isfinite(safe_float(r.get(metric)))]
        vmax = max(1.0, math.ceil(max(vals) * 1.18 / 25) * 25) if vals else 1.0
        draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label=label, x2=right)
        svg.text(left, top - 18, label, cls="label")
        group_w = (right - left - 70) / max(1, len(modes))
        for i, mode in enumerate(modes):
            r = next((x for x in rows if x["mode"] == mode), None)
            if not r:
                continue
            v = safe_float(r.get(metric))
            x = left + 38 + group_w * i
            y = y_scale(v, 0, vmax, top, bottom)
            color = MODE_COLORS.get(mode, "#555")
            svg.rect(x - 23, y, 46, bottom - y, fill=color, opacity=0.76, rx=4)
            svg.text(x, y - 6, f"{v:.1f}", cls="small", anchor="middle")
            svg.text(x, bottom + 24, MODE_DISPLAY.get(mode, mode), cls="xtick", anchor="middle")
    svg.save(path)


def plot_latency_boxplots(metric_rows: list[dict], discovery_rows: list[dict], path: Path) -> None:
    metrics = [
        ("capability_discovery_latency_ms", "Discovery latency"),
        ("request_latency_ms", "Request latency"),
        ("response_latency_ms", "Observation latency"),
        ("context_age_ms", "Context age"),
    ]
    # Add discovery rows into the same structure.
    all_rows = list(metric_rows)
    for r in discovery_rows:
        if safe_int(r.get("is_drone_query"), 0) != 1:
            continue
        lat = safe_float(r.get("latency_ms"))
        if math.isfinite(lat):
            all_rows.append({"mode": "F2D", "metric_name": "capability_discovery_latency_ms", "metric_value": lat, "event_type": "discovery_query_resp"})
    svg = Svg(1320, 720, "F2D middleware latency and freshness")
    svg.text(36, 56, "Boxplots from route-level events and discovery service logs; lower is better.", cls="subtitle")
    top, bottom, left, right = 116, 560, 88, 1260
    vals = [safe_float(r.get("metric_value")) for r in all_rows if r.get("metric_name") in {m[0] for m in metrics} and math.isfinite(safe_float(r.get("metric_value")))]
    vmax = max(1.0, math.ceil(np.percentile(vals, 98) * 1.25 / 10) * 10) if vals else 1.0
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Milliseconds", x2=right)
    group_w = (right - left) / len(metrics)
    rng = np.random.default_rng(11)
    for i, (metric, label) in enumerate(metrics):
        x = left + group_w * (i + 0.5)
        data = [safe_float(r.get("metric_value")) for r in all_rows if r.get("metric_name") == metric and math.isfinite(safe_float(r.get("metric_value")))]
        svg.text(x, bottom + 34, label, cls="xtick", anchor="middle")
        if not data:
            svg.text(x, (top + bottom) / 2, "no data", cls="small", anchor="middle")
            continue
        arr = np.array(data, dtype=float)
        q1, med, q3 = np.percentile(arr, [25, 50, 75])
        low, high = float(np.min(arr)), float(np.percentile(arr, 95))
        color = "#C65D32" if metric != "capability_discovery_latency_ms" else "#365C8D"
        yq1, ymed, yq3 = (y_scale(v, 0, vmax, top, bottom) for v in (q1, med, q3))
        ylow, yhigh = y_scale(low, 0, vmax, top, bottom), y_scale(high, 0, vmax, top, bottom)
        svg.line(x, ylow, x, yhigh, stroke=color, sw=1.8)
        svg.rect(x - 32, yq3, 64, max(1, yq1 - yq3), fill=color, stroke=color, sw=1, opacity=0.35, rx=4)
        svg.line(x - 32, ymed, x + 32, ymed, stroke=color, sw=2.3)
        for v in arr[:: max(1, len(arr) // 180)]:
            svg.circle(x + float(rng.uniform(-26, 26)), y_scale(float(v), 0, vmax, top, bottom), r=1.7, fill=color, opacity=0.25)
        svg.text(x, top - 10, f"n={len(arr)} p95={float(np.percentile(arr,95)):.1f}", cls="small", anchor="middle")
    svg.save(path)


def plot_event_counts(metric_rows: list[dict], path: Path) -> None:
    counts = Counter()
    for r in metric_rows:
        if r.get("metric_name") == "event_count" and r.get("event_type") in EVENT_COUNT_MAP:
            counts[r.get("event_type")] += safe_float(r.get("metric_value"), 0.0)
    items = [(EVENT_COUNT_MAP[k], counts[k]) for k in EVENT_COUNT_MAP if counts[k] > 0]
    svg = Svg(1320, 780, "F2D drone-context middleware events")
    svg.text(36, 56, "Counts show when the drone was requested, context was received/used, and safety guards fired.", cls="subtitle")
    top, bottom, left, right = 116, 610, 92, 1260
    vmax = max(1.0, math.ceil(max([v for _, v in items] or [1]) * 1.15 / 100) * 100)
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Event count", x2=right)
    bar_w = min(52, (right - left - 80) / max(1, len(items)) * 0.62)
    gap = (right - left - 80) / max(1, len(items))
    for i, (label, v) in enumerate(items):
        x = left + 50 + gap * i
        y = y_scale(v, 0, vmax, top, bottom)
        color = "#C65D32" if "Queue" not in label else "#8A5AAB"
        if "Stale" in label or "skip" in label.lower() or "guard" in label.lower():
            color = "#A64B3C"
        svg.rect(x - bar_w / 2, y, bar_w, bottom - y, fill=color, opacity=0.76, rx=4)
        svg.text(x, y - 6, f"{v:.0f}", cls="small", anchor="middle")
        svg.text(x, bottom + 18, label, cls="small", anchor="end", rotate=-38)
    svg.save(path)


def plot_context_timeline(context_rows: list[dict], path: Path) -> None:
    rows = [r for r in context_rows if math.isfinite(safe_float(r.get("sim_time"))) and math.isfinite(safe_float(r.get("context_age_ms")))]
    svg = Svg(1320, 650, "F2D context freshness over EV mission")
    svg.text(36, 56, "Each point is a drone context receive/use/stale event; stale events are highlighted.", cls="subtitle")
    top, bottom, left, right = 98, 520, 88, 1240
    if not rows:
        svg.text(620, 320, "No context-age timeline data found", cls="label", anchor="middle")
        svg.save(path)
        return
    xs = [safe_float(r.get("sim_time")) for r in rows]
    ys = [safe_float(r.get("context_age_ms")) for r in rows]
    xmin, xmax = min(xs), max(xs)
    ymax = max(10.0, math.ceil(np.percentile(ys, 98) * 1.25 / 100) * 100)
    draw_y_axis(svg, left, top, bottom, 0, ymax, ticks=6, label="Context age (ms)", x2=right)
    for i in range(7):
        xval = xmin + (xmax - xmin) * i / 6
        x = left + (xval - xmin) / max(1e-9, xmax - xmin) * (right - left)
        svg.line(x, bottom, x, bottom + 5, stroke="#70808f")
        svg.text(x, bottom + 24, f"{xval:.0f}", cls="tick", anchor="middle")
    svg.text((left + right) / 2, bottom + 48, "Simulation Time (s)", cls="label", anchor="middle")
    pts = []
    for r in rows:
        x = left + (safe_float(r.get("sim_time")) - xmin) / max(1e-9, xmax - xmin) * (right - left)
        y = y_scale(safe_float(r.get("context_age_ms")), 0, ymax, top, bottom)
        if r.get("event_type") == "f2.drone_context.used":
            pts.append((x, y))
    if pts:
        svg.polyline(pts, stroke="#C65D32", sw=2.0, opacity=0.65)
    for r in rows[:: max(1, len(rows)//600)]:
        x = left + (safe_float(r.get("sim_time")) - xmin) / max(1e-9, xmax - xmin) * (right - left)
        y = y_scale(safe_float(r.get("context_age_ms")), 0, ymax, top, bottom)
        stale = "stale" in str(r.get("event_type", ""))
        svg.circle(x, y, r=4.0 if stale else 2.2, fill="#A64B3C" if stale else "#C65D32", opacity=0.85 if stale else 0.35)
    svg.save(path)


def plot_payload_overhead(raw_rows: list[dict], path: Path) -> None:
    grouped = defaultdict(lambda: {"messages": 0, "payload_bytes": 0, "raw_bytes": 0})
    for r in raw_rows:
        topic = str(r.get("topic", ""))
        if "downstream_inspection" in topic:
            kind = "Inspection requests"
        elif "downstream_context" in topic:
            kind = "Context responses"
        elif "discovery" in topic:
            kind = "Discovery"
        else:
            kind = "Other drone topics"
        grouped[kind]["messages"] += 1
        grouped[kind]["payload_bytes"] += safe_int(r.get("payload_bytes"), 0)
        grouped[kind]["raw_bytes"] += safe_int(r.get("raw_bytes"), 0)
    items = [(k, v) for k, v in grouped.items()]
    svg = Svg(1180, 640, "F2D communication overhead")
    svg.text(36, 56, "Message and payload sizes from edge/FNM raw MQTT captures.", cls="subtitle")
    top, bottom, left, right = 108, 500, 88, 1080
    maxv = max([v["messages"] for _, v in items] + [1])
    draw_y_axis(svg, left, top, bottom, 0, math.ceil(maxv * 1.2 / 10) * 10, ticks=5, label="Messages", x2=right)
    group_w = (right - left) / max(1, len(items))
    for i, (label, vals) in enumerate(items):
        x = left + group_w * (i + 0.5)
        y = y_scale(vals["messages"], 0, math.ceil(maxv * 1.2 / 10) * 10, top, bottom)
        svg.rect(x - 34, y, 68, bottom - y, fill="#365C8D", opacity=0.78, rx=5)
        svg.text(x, y - 8, f"{vals['messages']}", cls="small", anchor="middle")
        svg.text(x, bottom + 24, label, cls="label", anchor="middle")
        svg.text(x, bottom + 44, f"payload {vals['payload_bytes']/1000.0:.1f} kB", cls="small", anchor="middle")
    svg.save(path)


def _mean_metric(metric_rows: list[dict], metric_name: str, mode: str | None = None) -> float:
    vals = [
        safe_float(r.get("metric_value"))
        for r in metric_rows
        if r.get("metric_name") == metric_name
        and (mode is None or r.get("mode") == mode)
        and math.isfinite(safe_float(r.get("metric_value")))
    ]
    if not vals:
        return 0.0
    return float(np.mean(np.array(vals, dtype=float)))


def plot_latency_chain(metric_rows: list[dict], path: Path) -> None:
    """Stack mean F2D request-response latency components.

    This is intentionally not a full sum of every emitted latency field because
    several fields are inclusive wall-clock spans. The stack uses non-overlapping
    segments that approximate the intersection-drone-intersection path.
    """
    modes = sorted(
        {str(r.get("mode", "")) for r in metric_rows if str(r.get("mode", "")).startswith("F2D")},
        key=mode_sort_key,
    )
    if not modes:
        modes = ["F2D"]
    svg = Svg(1240, 620, "F2D request-response latency decomposition")
    svg.text(
        36,
        56,
        "Mean wall-clock latency segments for SI-DT -> drone -> SUMO proxy -> SI-DT context delivery.",
        cls="subtitle",
    )
    top, bottom, left, right = 112, 500, 92, 1160
    totals = {
        mode: sum(_mean_metric(metric_rows, field, mode) for field, _ in LATENCY_CHAIN_COMPONENTS)
        for mode in modes
    }
    vmax = max(1.0, math.ceil(max(totals.values() or [1.0]) * 1.2 / 100.0) * 100.0)
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="Latency (ms)", x2=right)
    colors = ["#8E6BBE", "#C65D32", "#2D7DD2", "#4BAF7C"]
    group_w = (right - left - 120) / max(1, len(modes))
    for i, mode in enumerate(modes):
        x = left + 70 + group_w * i + group_w / 2
        y_cursor = bottom
        for j, (field, label) in enumerate(LATENCY_CHAIN_COMPONENTS):
            v = _mean_metric(metric_rows, field, mode)
            h = bottom - y_scale(v, 0, vmax, top, bottom)
            if h > 0:
                svg.rect(x - 44, y_cursor - h, 88, h, fill=colors[j % len(colors)], opacity=0.82, rx=3)
                y_cursor -= h
        svg.text(x, y_cursor - 8, f"{totals.get(mode, 0.0):.1f}", cls="small", anchor="middle")
        svg.text(x, bottom + 28, MODE_DISPLAY.get(mode, mode), cls="xtick", anchor="middle")
    lx = left + 18
    ly = 82
    for i, (_, label) in enumerate(LATENCY_CHAIN_COMPONENTS):
        svg.rect(lx + i * 270, ly - 10, 12, 12, fill=colors[i % len(colors)], opacity=0.82)
        svg.text(lx + i * 270 + 18, ly, label, cls="legend")
    svg.save(path)


def plot_payload_metric_bars(metric_rows: list[dict], path: Path) -> None:
    modes = sorted(
        {str(r.get("mode", "")) for r in metric_rows if str(r.get("mode", "")).startswith("F2D")},
        key=mode_sort_key,
    )
    if not modes:
        modes = ["F2D"]
    svg = Svg(1240, 620, "F2D request/response payload overhead")
    svg.text(36, 56, "Payload sizes from traced request and downstream-context response artefacts.", cls="subtitle")
    top, bottom, left, right = 112, 500, 92, 1160
    all_vals = [
        _mean_metric(metric_rows, field, mode) / 1_000.0
        for mode in modes
        for field, _ in DRONE_PAYLOAD_METRICS
    ]
    vmax = max(1.0, math.ceil(max(all_vals or [1.0]) * 1.25))
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="Payload Size (kB)", x2=right)
    colors = ["#365C8D", "#6BAED6", "#C65D32"]
    group_w = (right - left - 90) / max(1, len(modes))
    bar_w = min(42, group_w / 5)
    for i, mode in enumerate(modes):
        group_x = left + 55 + group_w * i + group_w / 2
        for j, (field, label) in enumerate(DRONE_PAYLOAD_METRICS):
            v = _mean_metric(metric_rows, field, mode) / 1_000.0
            x = group_x + (j - 1) * (bar_w + 8)
            y = y_scale(v, 0, vmax, top, bottom)
            svg.rect(x - bar_w / 2, y, bar_w, bottom - y, fill=colors[j], opacity=0.82, rx=3)
            if v > 0:
                svg.text(x, y - 7, f"{v:.1f}", cls="small", anchor="middle")
        svg.text(group_x, bottom + 28, MODE_DISPLAY.get(mode, mode), cls="xtick", anchor="middle")
    lx = left + 120
    ly = 82
    for i, (_, label) in enumerate(DRONE_PAYLOAD_METRICS):
        svg.rect(lx + i * 250, ly - 10, 12, 12, fill=colors[i], opacity=0.82)
        svg.text(lx + i * 250 + 18, ly, label, cls="legend")
    svg.save(path)


def plot_fnm_hop_overhead(fnm_rows: list[dict], path: Path) -> None:
    rows = [
        r for r in fnm_rows
        if r.get("event") in {"fnm.route.local_to_fed", "fnm.route.fed_to_local", "fnm.stage.local_to_fed", "fnm.stage.fed_to_local"}
    ]
    svg = Svg(1240, 650, "F2D FNM mediation overhead")
    svg.text(36, 56, "Hop-level FNM traces for drone request/response routing and protocol mediation.", cls="subtitle")
    if not rows:
        svg.text(620, 330, "No FNM hop traces found", cls="label", anchor="middle")
        svg.save(path)
        return
    groups = [
        ("local_to_fed", "Local -> federation"),
        ("fed_to_local", "Federation -> local"),
    ]
    metrics = [
        ("duration_ms", "Route duration"),
        ("local_to_fed_total_ms", "Local->fed stage"),
        ("fed_to_local_total_ms", "Fed->local stage"),
    ]
    top, bottom, left, right = 112, 500, 92, 1160
    def mean_for(direction: str, field: str) -> float:
        vals = [
            safe_float(r.get(field))
            for r in rows
            if r.get("direction") == direction and math.isfinite(safe_float(r.get(field))) and safe_float(r.get(field)) >= 0
        ]
        return float(np.mean(np.array(vals, dtype=float))) if vals else 0.0
    vals = [mean_for(direction, field) for direction, _ in groups for field, _ in metrics]
    vmax = max(1.0, math.ceil(max(vals or [1.0]) * 1.25 / 5.0) * 5.0)
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="Mean Latency (ms)", x2=right)
    colors = ["#365C8D", "#C65D32", "#4BAF7C"]
    group_w = (right - left - 120) / max(1, len(groups))
    bar_w = 44
    for i, (direction, label) in enumerate(groups):
        group_x = left + 70 + group_w * i + group_w / 2
        for j, (field, _) in enumerate(metrics):
            v = mean_for(direction, field)
            if v <= 0:
                continue
            x = group_x + (j - 1) * (bar_w + 10)
            y = y_scale(v, 0, vmax, top, bottom)
            svg.rect(x - bar_w / 2, y, bar_w, bottom - y, fill=colors[j], opacity=0.82, rx=3)
            svg.text(x, y - 7, f"{v:.1f}", cls="small", anchor="middle")
        svg.text(group_x, bottom + 28, label, cls="xtick", anchor="middle")
    lx = left + 150
    ly = 82
    for i, (_, label) in enumerate(metrics):
        svg.rect(lx + i * 245, ly - 10, 12, 12, fill=colors[i], opacity=0.82)
        svg.text(lx + i * 245 + 18, ly, label, cls="legend")
    svg.save(path)


def plot_node_activity_comparison(node_rows: list[dict], path: Path) -> None:
    rows = [r for r in node_rows if r.get("mode") in {"F2", "F2D"}]
    svg = Svg(1400, 720, "F2 vs F2D node-level federation activity")
    svg.text(36, 56, "Per-node runtime events; F2D should add mobile context activity where F2 has observability gaps.", cls="subtitle")
    if not rows:
        svg.text(700, 360, "No node-level activity rows found", cls="label", anchor="middle")
        svg.save(path)
        return
    nodes = sorted({str(r.get("node_id", "")) for r in rows if str(r.get("node_id", ""))})
    if len(nodes) > 18:
        # Keep the plot readable; the CSV still contains all nodes.
        totals = Counter()
        for r in rows:
            totals[str(r.get("node_id", ""))] += safe_int(r.get("count"), 0)
        nodes = [n for n, _ in totals.most_common(18)]
    categories = [
        "Discovery",
        "Priority request",
        "Federated coordination",
        "Local decision",
        "Drone request",
        "Drone context",
        "Freshness/safety guard",
    ]
    colors = {
        "Discovery": "#9B76C5",
        "Priority request": "#E68A4A",
        "Federated coordination": "#2D7DD2",
        "Local decision": "#5B9BD5",
        "Drone request": "#C65D32",
        "Drone context": "#1B9E77",
        "Freshness/safety guard": "#C94F56",
    }
    grouped = defaultdict(int)
    for r in rows:
        grouped[(str(r.get("mode", "")), str(r.get("node_id", "")), str(r.get("activity_category", "")))] += safe_int(r.get("count"), 0)
    totals_by_mode_node = {
        (mode, node): sum(grouped[(mode, node, cat)] for cat in categories)
        for mode in ("F2", "F2D")
        for node in nodes
    }
    vmax = max(1, max(totals_by_mode_node.values() or [1]))
    vmax = math.ceil(vmax * 1.2 / 10) * 10
    top, bottom, left, right = 112, 540, 92, 1320
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="Activity count", x2=right)
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(18, group_w / 4.5)
    for i, node in enumerate(nodes):
        cx = left + group_w * (i + 0.5)
        for m_i, mode in enumerate(("F2", "F2D")):
            x = cx + (-bar_w * 0.75 if mode == "F2" else bar_w * 0.75)
            y_cursor = bottom
            for cat in categories:
                v = grouped[(mode, node, cat)]
                if v <= 0:
                    continue
                h = bottom - y_scale(v, 0, vmax, top, bottom)
                svg.rect(x - bar_w / 2, y_cursor - h, bar_w, h, fill=colors.get(cat, "#888"), opacity=0.82, rx=2)
                y_cursor -= h
        svg.text(cx, bottom + 22, node.replace("Node", "N"), cls="small", anchor="end", rotate=-42)
    svg.text(left + 12, 86, "F2/F2D bars are paired per node", cls="small")
    lx, ly = 330, 82
    for i, cat in enumerate(categories):
        x = lx + (i % 4) * 230
        y = ly + (i // 4) * 20
        svg.rect(x, y - 10, 12, 12, fill=colors.get(cat, "#888"), opacity=0.82)
        svg.text(x + 18, y, cat, cls="legend")
    svg.save(path)


def build_summary_rows(traffic_rows: list[dict], event_rows: list[dict], discovery_rows: list[dict], raw_rows: list[dict]) -> list[dict]:
    out = []
    for row in traffic_rows:
        mode = row.get("mode", "")
        er = [r for r in event_rows if r.get("mode") == mode]
        is_drone_mode = mode.startswith("F2D")
        discovery_for_mode = [
            r for r in discovery_rows
            if safe_int(r.get("is_drone_query"), 0) == 1
            and (r.get("mode") == mode or (not r.get("mode") and is_drone_mode))
        ]
        raw_for_mode = [
            r for r in raw_rows
            if r.get("mode") == mode or (not r.get("mode") and is_drone_mode)
        ]
        out.append({
            "scenario_id": row.get("scenario_id", ""),
            "route_id": row.get("route_id", ""),
            "mode": mode,
            "arrived": row.get("arrived", ""),
            "travel_time_s": row.get("travel_time_s", ""),
            "waiting_time_s": row.get("waiting_time_s", ""),
            "time_loss_s": row.get("time_loss_s", ""),
            "waiting_count_n": row.get("waiting_count_n", ""),
            "route_length_m": row.get("route_length_m", ""),
            "drone_context_received_n": sum(1 for r in er if r.get("event_type") == "f2.drone_context.received"),
            "drone_context_used_n": sum(1 for r in er if r.get("event_type") == "f2.drone_context.used"),
            "drone_context_stale_n": sum(1 for r in er if r.get("event_type") == "f2.drone_context.stale"),
            "drone_blockage_detected_n": sum(1 for r in er if r.get("event_type") == "f2d.mobile_passive.blockage_detected"),
            "drone_advisory_n": sum(1 for r in er if r.get("event_type") == "f2d.ev_advisory.reroute_recommended"),
            "drone_discovery_queries_n": len(discovery_for_mode) if is_drone_mode else 0,
            "drone_discovery_success_n": (
                sum(1 for r in discovery_for_mode if safe_int(r.get("n_results"), 0) > 0)
                if is_drone_mode
                else 0
            ),
            "edge_raw_drone_messages_n": len(raw_for_mode) if is_drone_mode else 0,
            "edge_raw_drone_payload_bytes": (
                sum(safe_int(r.get("payload_bytes"), 0) for r in raw_for_mode)
                if is_drone_mode
                else 0
            ),
            "edge_raw_drone_wire_bytes": (
                sum(safe_int(r.get("raw_bytes"), 0) for r in raw_for_mode)
                if is_drone_mode
                else 0
            ),
        })
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate F2D drone middleware and traffic plots.")
    p.add_argument("--run-dir", required=True, help="Drone experiment result folder containing ev_matrix_results.csv and scenario_runs/.")
    p.add_argument("--out-dir", default=None, help="Output folder. Default: $ROOT/tmp/plots/f2d_drone or run-dir/f2d_plots.")
    p.add_argument("--reference-results-csv", action="append", default=[], help="Optional CSV with F2P/F2 reference rows for same route/scenario.")
    p.add_argument("--scenario-id", default=None, help="Scenario id to plot. Default: first scenario in result CSV.")
    p.add_argument("--route-id", type=int, default=None, help="Route id to plot. Default: first route in result CSV.")
    p.add_argument("--modes", default="F2,F2P,F2D", help="Comma-separated modes to include. Use F2,F2P,F2D,F2D-Q if desired.")
    p.add_argument("--core-log-dir", action="append", default=[], help="Core log dir containing discovery/catalog/membership jsonl. Auto-detected if omitted.")
    p.add_argument("--edge-raw", action="append", default=[], help="Edge drone raw_messages.jsonl. Auto-detected from sibling folders if omitted.")
    p.add_argument("--fnm-trace", action="append", default=[], help="FNM trace.jsonl files for hop-level route/stage/delivery overhead. Auto-detected if omitted.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    root = os.environ.get("ROOT")
    out_dir = Path(args.out_dir).expanduser() if args.out_dir else (Path(root) / "tmp" / "plots" / "f2d_drone" if root else run_dir / "f2d_plots")
    out_dir.mkdir(parents=True, exist_ok=True)
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]

    main_csv = find_result_csv(run_dir)
    rows = read_result_rows(main_csv, "drone_run")
    scenario_id = args.scenario_id or (rows[0].get("scenario_id") if rows else None)
    route_id = args.route_id if args.route_id is not None else (rows[0].get("route_id_int") if rows else None)
    traffic_rows = select_rows(rows, scenario_id, route_id, modes)

    for ref in args.reference_results_csv:
        ref_rows = read_result_rows(Path(ref).expanduser(), "reference")
        traffic_rows.extend(select_rows(ref_rows, scenario_id, route_id, modes))

    # Deduplicate mode rows, preferring direct drone_run rows over reference except for F2P.
    by_mode: dict[str, dict] = {}
    for row in traffic_rows:
        mode = row.get("mode", "")
        if mode not in by_mode or (mode == "F2P" and row.get("source_label") == "reference"):
            by_mode[mode] = row
    traffic_rows = [by_mode[m] for m in sorted(by_mode, key=mode_sort_key)]

    core_dirs = [Path(x).expanduser() for x in args.core_log_dir]
    if not core_dirs:
        parent = run_dir.parent
        core_dirs = [p for p in parent.glob("federation_core_logs*") if p.is_dir()]
        core_dirs.extend([p for p in run_dir.glob("**/federation_core_logs*") if p.is_dir()])
    edge_raw_files = [Path(x).expanduser() for x in args.edge_raw]
    if not edge_raw_files:
        search_roots = [run_dir.parent, run_dir.parent.parent]
        for sr in search_roots:
            edge_raw_files.extend(Path(p) for p in glob.glob(str(sr / "**/gw-drone*/raw_messages.jsonl"), recursive=True))
    edge_raw_files = sorted(set(p for p in edge_raw_files if p.exists()))
    fnm_trace_files = [Path(x).expanduser() for x in args.fnm_trace]
    if not fnm_trace_files:
        search_roots = [run_dir.parent, run_dir.parent.parent]
        for sr in search_roots:
            fnm_trace_files.extend(Path(p) for p in glob.glob(str(sr / "**/gw-drone*/trace.jsonl"), recursive=True))
            fnm_trace_files.extend(Path(p) for p in glob.glob(str(sr / "**/trace.jsonl"), recursive=True))
    fnm_trace_files = sorted(set(p for p in fnm_trace_files if p.exists()))

    event_rows, metric_rows, timeline_rows = parse_event_logs(run_dir, traffic_rows)
    node_activity_rows = parse_node_activity_logs(run_dir, traffic_rows)
    discovery_rows = parse_discovery_logs(core_dirs, scenario_id=scenario_id)
    raw_rows = parse_edge_raw(edge_raw_files)
    fnm_rows = parse_fnm_traces(fnm_trace_files)

    summary_rows = build_summary_rows(traffic_rows, event_rows, discovery_rows, raw_rows)
    write_csv(out_dir / "f2d_traffic_outcomes.csv", traffic_rows)
    write_csv(out_dir / "f2d_middleware_events.csv", event_rows)
    write_csv(out_dir / "f2d_node_activity_by_mode.csv", node_activity_rows)
    write_csv(out_dir / "f2d_middleware_metric_samples.csv", metric_rows)
    write_csv(out_dir / "f2d_context_timeline.csv", timeline_rows)
    write_csv(out_dir / "f2d_discovery_summary.csv", discovery_rows)
    write_csv(out_dir / "f2d_edge_payload_messages.csv", raw_rows)
    write_csv(out_dir / "f2d_fnm_hop_traces.csv", fnm_rows)
    write_csv(out_dir / "f2d_summary.csv", summary_rows)
    latency_metric_rows = [
        r for r in metric_rows
        if r.get("metric_name") in {name for name, _ in DRONE_LATENCY_METRICS} | {"confidence", "capability_discovery_latency_ms"}
    ]
    write_csv(out_dir / "f2d_latency_summary.csv", summarize_values(latency_metric_rows, "metric_value", ["mode", "metric_name"]))
    payload_metric_rows = [
        r for r in metric_rows
        if r.get("metric_name") in {name for name, _ in DRONE_PAYLOAD_METRICS}
    ]
    write_csv(out_dir / "f2d_payload_summary.csv", summarize_values(payload_metric_rows, "metric_value", ["mode", "metric_name"]))
    chain_metric_rows = [
        r for r in metric_rows
        if r.get("metric_name") in {name for name, _ in LATENCY_CHAIN_COMPONENTS}
    ]
    write_csv(out_dir / "f2d_latency_chain_summary.csv", summarize_values(chain_metric_rows, "metric_value", ["mode", "metric_name"]))
    write_csv(out_dir / "f2d_fnm_route_duration_summary.csv", summarize_values(fnm_rows, "duration_ms", ["direction", "event", "artefact_kind"]))
    write_csv(out_dir / "f2d_fnm_payload_summary.csv", summarize_values(fnm_rows, "payload_size_bytes", ["direction", "event", "artefact_kind"]))

    suffix = f" - route {route_id}, {scenario_id}" if scenario_id and route_id is not None else ""
    if traffic_rows:
        plot_traffic_bars(traffic_rows, out_dir / "f2d_traffic_outcomes.svg", suffix)
    plot_latency_boxplots(metric_rows, discovery_rows, out_dir / "f2d_latency_freshness_boxplots.svg")
    plot_event_counts(metric_rows, out_dir / "f2d_event_counts.svg")
    plot_context_timeline(timeline_rows, out_dir / "f2d_context_freshness_timeline.svg")
    plot_payload_overhead(raw_rows, out_dir / "f2d_payload_overhead.svg")
    plot_latency_chain(metric_rows, out_dir / "f2d_latency_chain_decomposition.svg")
    plot_payload_metric_bars(metric_rows, out_dir / "f2d_payload_metric_bars.svg")
    plot_fnm_hop_overhead(fnm_rows, out_dir / "f2d_fnm_hop_overhead.svg")
    plot_node_activity_comparison(node_activity_rows, out_dir / "f2_vs_f2d_node_activity.svg")

    print(f"Wrote F2D plots and CSVs to {out_dir}")
    print(f"Main results CSV: {main_csv}")
    print(f"Scenario: {scenario_id} route: {route_id} modes: {','.join(modes)}")
    print(f"Core log dirs: {len(core_dirs)} edge raw files: {len(edge_raw_files)} fnm traces: {len(fnm_trace_files)}")


if __name__ == "__main__":
    main()
