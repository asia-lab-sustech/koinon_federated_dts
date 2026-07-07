#!/usr/bin/env python3
"""Paper-level Route 6 F2D middleware plots.

This script consumes the final Route 6 run folder plus the CSV bundle emitted by
make_f2d_drone_plots.py. It focuses on node-level cooperation: SI-DT request,
Drone-DT observations, context delivery/use, freshness, and middleware cost.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import shutil
import statistics
import subprocess
from collections import Counter, defaultdict
from pathlib import Path


ROUTE6_NODES = [
    ("Node265", "non-TLS"),
    ("Node267", "TLS"),
    ("Node300", "non-TLS"),
    ("Node308", "non-TLS"),
    ("Node352", "non-TLS"),
    ("Node423", "TLS"),
    ("Node523", "TLS"),
    ("Node525", "non-TLS"),
    ("Node532", "TLS"),
    ("Node610", "TLS"),
    ("Node657", "TLS"),
    ("Node785", "TLS"),
    ("Node844", "TLS"),
    ("Node908", "TLS"),
    ("Node952", "TLS"),
    ("Node1043", "TLS"),
    ("Node1083", "TLS"),
    ("Node1086", "TLS"),
    ("Node1190", "TLS"),
    ("Node1189", "non-TLS"),
    ("Node1183", "non-TLS"),
    ("Node1181", "non-TLS"),
]

COLORS = {
    "F2": "#2D7DD2",
    "F2D": "#1B9E77",
    "Discovery": "#A77CCC",
    "Priority request": "#E07A32",
    "Local decision": "#5B9BD5",
    "Local SI-DT context augmentation": "#5B9BD5",
    "Federated coordination": "#C44E52",
    "Drone context": "#1B9E77",
    "Freshness/safety guard": "#7A9E7E",
    "request": "#9467BD",
    "scout": "#1B9E77",
    "use": "#2D7DD2",
    "stale": "#C44E52",
    "ev": "#E07A32",
    "payload": "#5B6472",
}


class Svg:
    def __init__(self, width: int, height: int, style: dict[str, float] | None = None) -> None:
        style = style or {}
        self.tick_font_size = float(style.get("tick_font_size", 17))
        self.ytick_font_size = float(style.get("ytick_font_size", 15))
        self.label_font_size = float(style.get("label_font_size", 20))
        self.legend_font_size = float(style.get("legend_font_size", 17))
        self.legend_font_weight = int(style.get("legend_font_weight", 700))
        self.small_font_size = float(style.get("small_font_size", 14))
        self.value_font_size = float(style.get("value_font_size", 14))
        self.width = width
        self.height = height
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            "<style>",
            """
            text{font-family:Helvetica,Arial,sans-serif;fill:#314150}
            .tick{font-size:%(tick)spx;font-weight:700}
            .ytick{font-size:%(ytick)spx}
            .label{font-size:%(label)spx;font-weight:700}
            .legend{font-size:%(legend)spx;font-weight:%(legend_weight)s}
            .small{font-size:%(small)spx}
            .value{font-size:%(value)spx;font-weight:700;fill:#415263}
            .grid{stroke:#DCE3EA;stroke-width:1;opacity:.72}
            .axis{stroke:#788697;stroke-width:1.5}
            """ % {
                "tick": self.tick_font_size,
                "ytick": self.ytick_font_size,
                "label": self.label_font_size,
                "legend": self.legend_font_size,
                "legend_weight": self.legend_font_weight,
                "small": self.small_font_size,
                "value": self.value_font_size,
            },
            "</style>",
        ]

    def add(self, s: str) -> None:
        self.parts.append(s)

    def text(self, x: float, y: float, text: str, cls: str = "", anchor: str = "middle", rotate: float | None = None) -> None:
        tr = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
        self.add(f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="{cls}"{tr}>{html.escape(str(text))}</text>')

    def line(self, x1: float, y1: float, x2: float, y2: float, cls: str = "", stroke: str | None = None, sw: float | None = None, dash: str | None = None, opacity: float | None = None) -> None:
        extra = ""
        if stroke:
            extra += f' stroke="{stroke}"'
        if sw:
            extra += f' stroke-width="{sw}"'
        if dash:
            extra += f' stroke-dasharray="{dash}"'
        if opacity is not None:
            extra += f' opacity="{opacity}"'
        self.add(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" class="{cls}"{extra}/>')

    def rect(self, x: float, y: float, w: float, h: float, fill: str, stroke: str = "none", sw: float = 1.0, opacity: float = 1.0, rx: float = 0) -> None:
        self.add(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}" rx="{rx}"/>')

    def circle(self, x: float, y: float, r: float, fill: str, stroke: str = "white", sw: float = 1.2) -> None:
        self.add(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def diamond(self, x: float, y: float, r: float, fill: str) -> None:
        pts = f"{x:.2f},{y-r:.2f} {x+r:.2f},{y:.2f} {x:.2f},{y+r:.2f} {x-r:.2f},{y:.2f}"
        self.add(f'<polygon points="{pts}" fill="{fill}" stroke="white" stroke-width="1.2"/>')

    def arrow(self, x1: float, y1: float, x2: float, y2: float, color: str = "#7B8794", sw: float = 2.0) -> None:
        self.line(x1, y1, x2, y2, stroke=color, sw=sw)
        ang = math.atan2(y2 - y1, x2 - x1)
        size = 8.0
        p1 = (x2, y2)
        p2 = (x2 - size * math.cos(ang - math.pi / 6), y2 - size * math.sin(ang - math.pi / 6))
        p3 = (x2 - size * math.cos(ang + math.pi / 6), y2 - size * math.sin(ang + math.pi / 6))
        pts = f"{p1[0]:.2f},{p1[1]:.2f} {p2[0]:.2f},{p2[1]:.2f} {p3[0]:.2f},{p3[1]:.2f}"
        self.add(f'<polygon points="{pts}" fill="{color}" stroke="none"/>')

    def save(self, path: Path) -> None:
        self.parts.append("</svg>")
        path.write_text("\n".join(self.parts), encoding="utf-8")


def fnum(v, default=math.nan) -> float:
    try:
        if v is None or v == "":
            return default
        return float(v)
    except Exception:
        return default


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def read_jsonl(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except Exception:
                continue
    return rows


def default_core_log_dir(run_dir: Path) -> Path:
    # run_dir usually points to .../final_runs/<run_id>/ev_matrix_results.
    parent = run_dir.parent
    candidate = parent / "federation_core_logs"
    return candidate


def load_core_logs(core_log_dir: Path) -> dict[str, list[dict]]:
    services = [
        "membership",
        "catalog",
        "discovery",
        "lifecycle",
        "state_manager",
        "adaptive_connectivity",
        "metrics",
    ]
    return {svc: read_jsonl(core_log_dir / f"{svc}.jsonl") for svc in services}


def locate_event_files(run_dir: Path) -> list[Path]:
    return sorted(run_dir.glob("scenario_runs/*/matrix_out_f2d/runs/*/route_6/F2D/*.events.jsonl"))


def locate_mode_event_files(run_dir: Path, mode: str) -> list[Path]:
    return sorted(run_dir.glob(f"scenario_runs/*/matrix_out_{mode.lower()}/runs/*/route_6/{mode}/*.events.jsonl"))


def load_events(run_dir: Path) -> list[dict]:
    rows = []
    for path in locate_event_files(run_dir):
        # Some SUMO/FNM event logs can contain occasional non-UTF8 bytes from
        # process output. Treat those bytes as lossy text so the JSONL scan does
        # not fail before reaching the valid structured event lines.
        with path.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                obj["_source_file"] = str(path)
                obj["_etype"] = obj.get("event_type") or obj.get("event") or ""
                rows.append(obj)
    return rows


def load_mode_events(run_dir: Path, mode: str) -> list[dict]:
    rows = []
    for path in locate_mode_event_files(run_dir, mode):
        with path.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                obj["_source_file"] = str(path)
                obj["_etype"] = obj.get("event_type") or obj.get("event") or ""
                obj["_mode"] = mode
                rows.append(obj)
    return rows


def first_by(rows: list[dict], key_fn, time_key: str = "sim_time") -> dict:
    out = {}
    for r in sorted(rows, key=lambda x: fnum(x.get(time_key) or x.get("ts_sim_s"), 1e18)):
        k = key_fn(r)
        if k and k not in out:
            out[k] = r
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def draw_legend(svg: Svg, items: list[tuple[str, str]], x: float, y: float) -> None:
    cur = x
    for label, color in items:
        svg.rect(cur, y - 12, 15, 15, color)
        svg.text(cur + 22, y, label, "legend", "start")
        cur += max(115, len(label) * 8.5 + 38)


def draw_centered_legend(svg: Svg, items: list[tuple[str, str]], center_x: float, y: float) -> None:
    """Draw one legend row centered over the shared plotting grid."""
    char_width = svg.legend_font_size * 0.53
    widths = [max(116.0, len(label) * char_width + 43.0) for label, _ in items]
    cur = center_x - sum(widths) / 2
    marker_size = max(15.0, svg.legend_font_size * 0.78)
    for (label, color), item_width in zip(items, widths):
        svg.rect(cur, y - marker_size + 3, marker_size, marker_size, color)
        svg.text(cur + marker_size + 8, y, label, "legend", "start")
        cur += item_width


def draw_ordered_legend_grid(
    svg: Svg,
    rows: list[list[tuple[str, str]]],
    *,
    left_x: float,
    first_y: float,
    column_width: float = 250.0,
    row_height: float = 32.0,
) -> None:
    """Draw a fixed ordered legend grid matching the Route 5 paper figures."""
    marker_size = max(15.0, svg.legend_font_size * 0.78)
    for row_idx, row in enumerate(rows):
        y = first_y + row_idx * row_height
        for col_idx, (label, color) in enumerate(row):
            x = left_x + col_idx * column_width
            svg.rect(x, y - marker_size + 3, marker_size, marker_size, color)
            lines = str(label).split("\n")
            text_x = x + marker_size + 8
            if len(lines) == 1:
                svg.text(text_x, y, lines[0], "legend", "start")
            else:
                line_step = svg.legend_font_size * 0.92
                start_y = y - line_step * (len(lines) - 1) / 2
                for line_idx, line in enumerate(lines):
                    svg.text(text_x, start_y + line_idx * line_step, line, "legend", "start")


def draw_multiline_text(svg: Svg, x: float, y: float, text: str, cls: str = "small", anchor: str = "middle", line_h: float = 16.0, rotate: float | None = None) -> None:
    lines = str(text).split("\n")
    if len(lines) == 1:
        svg.text(x, y, text, cls, anchor, rotate=rotate)
        return
    if rotate is not None:
        # Keep rotated labels single-line; multi-line rotated SVG is hard to
        # read and not needed in these paper figures.
        svg.text(x, y, " ".join(lines), cls, anchor, rotate=rotate)
        return
    start_y = y - (len(lines) - 1) * line_h / 2
    svg.add(f'<text x="{x:.2f}" y="{start_y:.2f}" text-anchor="{anchor}" class="{cls}">')
    for i, line in enumerate(lines):
        dy = 0 if i == 0 else line_h
        svg.add(f'<tspan x="{x:.2f}" dy="{dy:.2f}">{html.escape(line)}</tspan>')
    svg.add("</text>")


def draw_rotated_multiline_text(svg: Svg, x: float, y: float, text: str, cls: str = "small", anchor: str = "middle", line_h: float = 15.0, rotate: float = 24.0) -> None:
    lines = str(text).split("\n")
    if len(lines) == 1:
        svg.text(x, y, text, cls, anchor, rotate=rotate)
        return
    start_y = y - (len(lines) - 1) * line_h / 2
    svg.add(f'<text x="{x:.2f}" y="{start_y:.2f}" text-anchor="{anchor}" class="{cls}" transform="rotate({rotate} {x:.2f} {y:.2f})">')
    for i, line in enumerate(lines):
        dy = 0 if i == 0 else line_h
        svg.add(f'<tspan x="{x:.2f}" dy="{dy:.2f}">{html.escape(line)}</tspan>')
    svg.add("</text>")


def wrap_axis_label(label: str) -> str:
    manual = {
        "SI-DT -> Drone-DT Request": "SI-DT to\nDrone-DT",
        "Real-world scouting": "Real-world\nscouting",
        "Drone-DT -> SI-DT Support": "Drone-DT to\nSI-DT",
        "State manager": "State\nmanager",
        "Adaptive connectivity": "Adaptive\nconnectivity",
        "Median service processing": "Median service\nprocessing",
    }
    return manual.get(label.replace("\n", " "), label)


def percentile(vals: list[float], pct: float) -> float:
    vals = sorted(v for v in vals if math.isfinite(v))
    if not vals:
        return math.nan
    idx = max(0, min(len(vals) - 1, int(math.ceil(len(vals) * pct)) - 1))
    return vals[idx]


def event_time(r: dict) -> float:
    return fnum(r.get("sim_time") or r.get("ts_sim_s"))


def event_tls(r: dict) -> str:
    return str(r.get("tls_id") or r.get("requester_tls") or r.get("route_current_tls") or "")


def service_category(event_type: str) -> str | None:
    if event_type == "ev.intersection.discovery.observed":
        return "Discovery service"
    if event_type.startswith("ev.request.") or event_type in {"f2.drone_context.requested", "f2.drone_context.request_skip", "f2.drone_context.rejected"}:
        return "FNM request routing"
    if event_type in {
        "downstream_context.external_rx",
        "f2.drone_context.received",
        "f2d.context.provider_observed_contextual_delivery",
        "f2d.context.si_dt_received",
        "f2d.context.si_dt_cache_update",
        "f2d.mobile_passive.received",
    }:
        return "Drone context delivery"
    if event_type in {
        "f2.drone_context.used",
        "f2.drone_context.missing",
        "f2.drone_context.stale",
        "f2.drone_context.conflict_with_peer",
        "f2d.mobile_passive.used",
        "f2d.mobile_passive.stale",
        "f2d.mobile_passive.blockage_detected",
        "f2d.ev_advisory.reroute_recommended",
        "f2d.drone_context.recovery_candidate",
    }:
        return "Freshness/context guard"
    if event_type in {"f2.apply", "f2.apply_skipped", "f2.strict_b1_floor.apply", "f2.b1_continuity.apply", "f2.b1_continuity.skip"}:
        return "Local SI-DT context augmentation"
    if event_type in {"f2.strict_b1_floor.apply_drone_guard", "f2d.queue_release.requested"}:
        return "Drone-supported guard"
    return None


def plot_node_timeline(
    events: list[dict],
    path: Path,
    *,
    width: int = 1040,
    height: int = 680,
    xmin: float = 1020.0,
    xmax: float = 1120.0,
    legend_offset_y: float = 32.0,
    style: dict[str, float] | None = None,
) -> list[dict]:
    node_index = {n: i for i, (n, _) in enumerate(ROUTE6_NODES)}
    request_rows = [r for r in events if r.get("_etype") == "f2.drone_context.requested"]
    scout_rows = [r for r in events if r.get("_etype") == "f2.drone_context.received" and r.get("waypoint_node")]
    use_rows = [r for r in events if r.get("_etype") == "f2.drone_context.used" and r.get("tls_id")]
    stale_rows = [r for r in events if r.get("_etype") == "f2.drone_context.stale" and r.get("tls_id")]
    ev_rows = [r for r in events if r.get("_etype") == "ev.node.cross"]

    first_req = first_by(request_rows, lambda r: r.get("requester_tls"))
    first_scout = first_by(scout_rows, lambda r: r.get("waypoint_node"))
    first_use = first_by(use_rows, lambda r: r.get("tls_id"))
    first_stale = first_by(stale_rows, lambda r: r.get("tls_id"))

    # Reserve the same three-row legend band used by the paired service plot.
    left, right, top, bottom = 150, 42, 150, 88
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height, style)
    legend_items = [
        ("Drone request", COLORS["request"]),
        ("Drone context published", COLORS["scout"]),
        ("SI-DT context used", COLORS["use"]),
        ("Stale rejected", COLORS["stale"]),
    ]
    draw_ordered_legend_grid(
        svg,
        [legend_items[:2], legend_items[2:]],
        left_x=250,
        first_y=38 + legend_offset_y,
        column_width=340,
    )

    def xscale(v: float) -> float:
        return left + (v - xmin) / (xmax - xmin) * plot_w

    def yscale(node: str) -> float:
        return top + node_index[node] / (len(ROUTE6_NODES) - 1) * plot_h

    tick_vals = [xmin + i * (xmax - xmin) / 5 for i in range(6)]
    for val in tick_vals:
        x = xscale(val)
        svg.line(x, top, x, top + plot_h, "grid")
        svg.text(x, top + plot_h + 31, f"{val:.0f}", "tick")
    for node, typ in ROUTE6_NODES:
        y = yscale(node)
        svg.line(left, y, left + plot_w, y, "grid", opacity=0.45)
        label = f"{node}" if typ == "TLS" else f"{node}*"
        svg.text(left - 10, y + 4, label, "ytick", "end")

    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(left + plot_w / 2, top + plot_h + 65, "Simulation Time (s)", "label")

    def in_window(row: dict) -> bool:
        t = fnum(row.get("sim_time") or row.get("ts_sim_s"))
        return math.isfinite(t) and xmin <= t <= xmax

    for r in first_req.values():
        node = r.get("requester_tls")
        if node in node_index and in_window(r):
            svg.diamond(xscale(fnum(r.get("sim_time"))), yscale(node), 7, COLORS["request"])
    for r in first_scout.values():
        node = r.get("waypoint_node")
        if node in node_index and in_window(r):
            x, y = xscale(fnum(r.get("sim_time"))), yscale(node)
            svg.rect(x - 5, y - 5, 10, 10, COLORS["scout"], stroke="white", sw=1.1)
    for r in first_use.values():
        node = r.get("tls_id")
        if node in node_index and in_window(r):
            svg.circle(xscale(fnum(r.get("sim_time"))), yscale(node), 5.8, COLORS["use"])
    for r in first_stale.values():
        node = r.get("tls_id")
        if node in node_index and in_window(r):
            x, y = xscale(fnum(r.get("sim_time"))), yscale(node)
            svg.line(x - 6, y - 6, x + 6, y + 6, stroke=COLORS["stale"], sw=2.4)
            svg.line(x - 6, y + 6, x + 6, y - 6, stroke=COLORS["stale"], sw=2.4)
    svg.text(left + 4, top + plot_h - 4, "* non-TLS route node", "small", "start")
    svg.save(path)

    summary = []
    for node, typ in ROUTE6_NODES:
        summary.append({
            "node_id": node,
            "node_type": typ,
            "request_sim_time": fnum(first_req.get(node, {}).get("sim_time"), ""),
            "drone_context_publish_sim_time": fnum(first_scout.get(node, {}).get("sim_time"), ""),
            "si_dt_context_use_sim_time": fnum(first_use.get(node, {}).get("sim_time"), ""),
            "first_use_context_age_ms": fnum(first_use.get(node, {}).get("context_age_ms"), ""),
            "first_use_worst_edge": first_use.get(node, {}).get("worst_edge", ""),
        })
    return summary


def plot_freshness(events: list[dict], path: Path) -> list[dict]:
    use_rows = [r for r in events if r.get("_etype") == "f2.drone_context.used" and r.get("tls_id")]
    by_tls = defaultdict(list)
    for r in use_rows:
        age = fnum(r.get("context_age_ms"))
        if math.isfinite(age):
            by_tls[r.get("tls_id")].append(age)
    nodes = [n for n, typ in ROUTE6_NODES if typ == "TLS" and n in by_tls]
    rows = []
    for n in nodes:
        vals = by_tls[n]
        rows.append({"tls_id": n, "n": len(vals), "mean_age_ms": statistics.mean(vals), "p95_age_ms": sorted(vals)[max(0, int(math.ceil(len(vals) * 0.95)) - 1)], "max_age_ms": max(vals)})

    width, height = 960, 430
    left, right, top, bottom = 82, 24, 52, 82
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    ymax = 8500
    for i in range(6):
        val = ymax * i / 5
        y = top + plot_h - val / ymax * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 8, y + 5, f"{val/1000:.1f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(24, top + plot_h / 2, "Context age (s)", "label", rotate=-90)
    # Freshness thresholds used by the run.
    for sec, color, label in [(2, "#C44E52", "2 s max-age"), (8, "#7A9E7E", "8 s TTL")]:
        y = top + plot_h - sec * 1000 / ymax * plot_h
        svg.line(left, y, left + plot_w, y, stroke=color, sw=1.6, dash="6 4", opacity=0.8)
        svg.text(left + plot_w - 4, y - 5, label, "small", "end")
    bar_w = max(16, min(34, plot_w / max(1, len(rows)) * 0.45))
    for i, r in enumerate(rows):
        x = left + (i + 0.5) / len(rows) * plot_w
        val = min(ymax, r["mean_age_ms"])
        h = val / ymax * plot_h
        svg.rect(x - bar_w / 2, top + plot_h - h, bar_w, h, COLORS["F2D"])
        svg.text(x, top + plot_h - h - 7, f"{r['mean_age_ms']/1000:.1f}", "value")
        svg.text(x, top + plot_h + 30, r["tls_id"].replace("Node", "N"), "tick", rotate=28)
    svg.save(path)
    return rows


def plot_node_activity(plot_dir: Path, path: Path) -> None:
    rows = read_csv(plot_dir / "f2d_node_activity_by_mode.csv")
    cats = ["Local decision", "Federated coordination", "Drone context"]
    nodes = [n for n, typ in ROUTE6_NODES if typ == "TLS"]
    data = defaultdict(lambda: defaultdict(int))
    for r in rows:
        if r.get("node_id") in nodes and r.get("mode") in {"F2", "F2D"} and r.get("activity_category") in cats:
            data[(r["mode"], r["node_id"])][r["activity_category"]] += int(float(r.get("count") or 0))
    width, height = 1180, 450
    left, right, top, bottom = 76, 24, 56, 80
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    draw_legend(svg, [("FCDP", COLORS["F2"]), ("FCDP-D", COLORS["F2D"])], width / 2 - 82, 32)
    totals = {(m, n): sum(data[(m, n)].values()) for m in ["F2", "F2D"] for n in nodes}
    ymax = max(1, max(totals.values()) * 1.12)
    ymax = math.ceil(ymax / 200) * 200
    for i in range(5):
        val = ymax * i / 4
        y = top + plot_h - val / ymax * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 8, y + 5, f"{val:.0f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(22, top + plot_h / 2, "Federation Service Activity (count)", "label", rotate=-90)
    group_w = plot_w / len(nodes)
    bar_w = min(20, group_w * 0.28)
    for i, node in enumerate(nodes):
        cx = left + (i + 0.5) * group_w
        for j, mode in enumerate(["F2", "F2D"]):
            x = cx + (j - 0.5) * (bar_w + 4)
            val = totals[(mode, node)]
            h = val / ymax * plot_h
            svg.rect(x - bar_w / 2, top + plot_h - h, bar_w, h, COLORS[mode])
        svg.text(cx, top + plot_h + 28, node.replace("Node", "N"), "tick", rotate=32)
    svg.save(path)


def plot_route_node_activity_stacked(plot_dir: Path, path: Path) -> None:
    rows = read_csv(plot_dir / "f2d_node_activity_by_mode.csv")
    cats = ["Priority request", "Local decision", "Federated coordination", "Freshness/safety guard", "Drone context"]
    data = defaultdict(lambda: defaultdict(int))
    for r in rows:
        node = r.get("node_id")
        mode = r.get("mode")
        cat = r.get("activity_category")
        if node and mode in {"F2", "F2D"} and cat in cats:
            data[(mode, node)][cat] += int(float(r.get("count") or 0))

    width, height = 1360, 560
    left, right, top, bottom = 82, 24, 76, 98
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    legend_items = [(c, COLORS[c]) for c in cats]
    draw_legend(svg, legend_items[:3], left, 30)
    draw_legend(svg, legend_items[3:], left + 650, 30)
    totals = {(m, n): sum(data[(m, n)].values()) for m in ["F2", "F2D"] for n, _ in ROUTE6_NODES}
    ymax = max(1, max(totals.values()) * 1.14)
    ymax = math.ceil(ymax / 250) * 250
    for i in range(6):
        val = ymax * i / 5
        y = top + plot_h - val / ymax * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 8, y + 5, f"{val:.0f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(24, top + plot_h / 2, "Federation Service Activity (count)", "label", rotate=-90)
    group_w = plot_w / len(ROUTE6_NODES)
    bar_w = min(18, group_w * 0.26)
    for i, (node, typ) in enumerate(ROUTE6_NODES):
        cx = left + (i + 0.5) * group_w
        for j, mode in enumerate(["F2", "F2D"]):
            x = cx + (j - 0.5) * (bar_w + 5)
            ybase = top + plot_h
            for cat in cats:
                val = data[(mode, node)][cat]
                if val <= 0:
                    continue
                h = val / ymax * plot_h
                svg.rect(x - bar_w / 2, ybase - h, bar_w, h, COLORS[cat])
                ybase -= h
        label = node.replace("Node", "N") + ("*" if typ != "TLS" else "")
        svg.text(cx, top + plot_h + 28, label, "tick", rotate=38)
    svg.text(left + 4, top + plot_h - 4, "* non-TLS route gap", "small", "start")
    svg.save(path)


def plot_route_node_activity_vertical(plot_dir: Path, path: Path) -> None:
    """Route-ordered F2/F2D activity with nodes on the y-axis, matching the context timeline."""
    rows = read_csv(plot_dir / "f2d_node_activity_by_mode.csv")
    cats = ["Priority request", "Local decision", "Federated coordination", "Freshness/safety guard", "Drone context"]
    data = defaultdict(lambda: defaultdict(int))
    for r in rows:
        node = r.get("node_id")
        mode = r.get("mode")
        cat = r.get("activity_category")
        if node and mode in {"F2", "F2D"} and cat in cats:
            data[(mode, node)][cat] += int(float(r.get("count") or 0))

    width, height = 1180, 680
    left, right, top, bottom = 118, 36, 64, 70
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    draw_legend(svg, [("FCDP", COLORS["F2"]), ("FCDP-D", COLORS["F2D"])], left + 360, 34)
    totals = {(m, n): sum(data[(m, n)].values()) for m in ["F2", "F2D"] for n, _ in ROUTE6_NODES}
    xmax = max(1, max(totals.values()) * 1.12)
    xmax = math.ceil(xmax / 250) * 250

    def xscale(v: float) -> float:
        return left + v / xmax * plot_w

    row_h = plot_h / max(1, len(ROUTE6_NODES) - 1)
    bar_h = min(10, row_h * 0.32)

    for i in range(6):
        val = xmax * i / 5
        x = xscale(val)
        svg.line(x, top, x, top + plot_h, "grid")
        svg.text(x, top + plot_h + 30, f"{val:.0f}", "tick")
    for i, (node, typ) in enumerate(ROUTE6_NODES):
        y = top + i / max(1, len(ROUTE6_NODES) - 1) * plot_h
        svg.line(left, y, left + plot_w, y, "grid", opacity=0.45)
        label = node + ("*" if typ != "TLS" else "")
        svg.text(left - 10, y + 5, label, "ytick", "end")
        for j, mode in enumerate(["F2", "F2D"]):
            ybar = y + (j - 0.5) * (bar_h + 4)
            val = totals[(mode, node)]
            if val <= 0:
                continue
            svg.rect(left, ybar - bar_h / 2, max(1, xscale(val) - left), bar_h, COLORS[mode])
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(width / 2, height - 8, "Federation Service Activity (count)", "label")
    svg.text(left + 4, top + plot_h - 4, "* non-TLS route gap", "small", "start")
    svg.save(path)


def plot_runtime_service_burst(
    events: list[dict],
    path: Path,
    *,
    width: int = 1040,
    height: int = 680,
    xmin: float = 1020.0,
    xmax: float = 1120.0,
    ylabel_x: float = 82.0,
    style: dict[str, float] | None = None,
) -> None:
    categories = [
        ("Discovery service", "#A77CCC"),
        ("FNM request routing", "#E07A32"),
        ("Drone context delivery", "#1B9E77"),
        ("Freshness/context guard", "#7A9E7E"),
        ("Local SI-DT context augmentation", "#5B9BD5"),
        ("Drone-supported guard", "#C44E52"),
    ]
    bin_s = 5.0
    bins = [xmin + i * bin_s for i in range(int((xmax - xmin) / bin_s) + 1)]
    counts = {name: [0 for _ in bins] for name, _ in categories}
    for r in events:
        t = event_time(r)
        if not math.isfinite(t) or t < xmin or t > xmax:
            continue
        cat = service_category(r.get("_etype", ""))
        if not cat:
            continue
        idx = min(len(bins) - 1, max(0, int((t - xmin) // bin_s)))
        counts[cat][idx] += 1

    totals = [sum(counts[name][i] for name, _ in categories) for i in range(len(bins))]
    ymax = max(1, max(totals) * 1.15)
    ymax = math.ceil(ymax / 100) * 100
    left, right, top, bottom = 150, 42, 150, 88
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height, style)
    category_colors = dict(categories)
    legend_rows = [
        [
            ("Discovery service", category_colors["Discovery service"]),
            ("FNM request routing", category_colors["FNM request routing"]),
            ("SI-DT context augmentation", category_colors["Local SI-DT context augmentation"]),
        ],
        [
            ("Drone context delivery", category_colors["Drone context delivery"]),
            ("Drone-supported guard", category_colors["Drone-supported guard"]),
            ("Freshness/context guard", category_colors["Freshness/context guard"]),
        ],
    ]
    draw_ordered_legend_grid(svg, legend_rows, left_x=left, first_y=70)

    def xscale(v: float) -> float:
        return left + (v - xmin) / (xmax - xmin) * plot_w

    def yscale(v: float) -> float:
        return top + plot_h - v / ymax * plot_h

    for i in range(5):
        val = ymax * i / 4
        y = yscale(val)
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 8, y + 5, f"{val:.0f}", "ytick", "end")
    for val in [xmin + i * (xmax - xmin) / 5 for i in range(6)]:
        x = xscale(val)
        svg.line(x, top, x, top + plot_h, "grid", opacity=0.42)
        svg.text(x, top + plot_h + 31, f"{val:.0f}", "tick")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(ylabel_x, top + plot_h / 2, "Federation Service Activity (count)", "label", rotate=-90)
    svg.text(left + plot_w / 2, top + plot_h + 65, "Simulation Time (s)", "label")

    lower = [0.0 for _ in bins]
    for name, color in categories:
        upper = [lower[i] + counts[name][i] for i in range(len(bins))]
        top_pts = " ".join(f"{xscale(bins[i]):.2f},{yscale(upper[i]):.2f}" for i in range(len(bins)))
        bot_pts = " ".join(f"{xscale(bins[i]):.2f},{yscale(lower[i]):.2f}" for i in reversed(range(len(bins))))
        svg.add(f'<polygon points="{top_pts} {bot_pts}" fill="{color}" opacity="0.82" stroke="none"/>')
        lower = upper
    svg.save(path)


def request_to_decision_samples(events_by_mode: dict[str, list[dict]]) -> list[dict]:
    decision_types = {
        "f2.apply",
        "f2.strict_b1_floor.apply",
        "f2.strict_b1_floor.apply_drone_guard",
        "f2.b1_continuity.apply",
    }
    rows = []
    for mode, events in events_by_mode.items():
        requests = []
        decisions = []
        for r in events:
            et = r.get("_etype", "")
            if et == "ev.request.dispatched":
                tls = event_tls(r)
                t = fnum(r.get("dispatch_sim_time") or r.get("sim_time") or r.get("ts_sim_s"))
                wt = fnum(r.get("ts_wall_ms"))
                if tls and math.isfinite(t):
                    requests.append((tls, t, wt, r))
            elif et in decision_types:
                tls = event_tls(r)
                t = event_time(r)
                wt = fnum(r.get("ts_wall_ms"))
                if tls and math.isfinite(t):
                    decisions.append((tls, t, wt, et, r))
        decisions_by_tls = defaultdict(list)
        for d in decisions:
            decisions_by_tls[d[0]].append(d)
        for tls in decisions_by_tls:
            decisions_by_tls[tls].sort(key=lambda x: x[1])
        for tls, t, wt, req in requests:
            candidates = [d for d in decisions_by_tls.get(tls, []) if d[1] >= t and d[1] <= t + 3.0]
            if not candidates:
                continue
            d = candidates[0]
            wall_ms = d[2] - wt if math.isfinite(wt) and math.isfinite(d[2]) else math.nan
            rows.append({
                "mode": mode,
                "tls_id": tls,
                "request_sim_time": t,
                "decision_sim_time": d[1],
                "decision_event": d[3],
                "sim_loop_latency_ms": (d[1] - t) * 1000.0,
                "wall_latency_ms": wall_ms,
            })
    return rows


def plot_request_to_decision_boxplot(samples: list[dict], path: Path) -> None:
    nodes = [n for n, typ in ROUTE6_NODES if typ == "TLS"]
    values = defaultdict(list)
    for r in samples:
        v = fnum(r.get("sim_loop_latency_ms"))
        if r.get("tls_id") in nodes and r.get("mode") in {"F2", "F2D"} and math.isfinite(v):
            values[(r["mode"], r["tls_id"])].append(v)

    width, height = 1220, 500
    left, right, top, bottom = 82, 24, 62, 92
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    draw_legend(svg, [("FCDP", COLORS["F2"]), ("FCDP-D", COLORS["F2D"])], width / 2 - 85, 30)
    ymax = 1600.0
    for i in range(5):
        val = ymax * i / 4
        y = top + plot_h - val / ymax * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 8, y + 5, f"{val:.0f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(24, top + plot_h / 2, "Request-to-decision/apply latency (ms)", "label", rotate=-90)
    group_w = plot_w / len(nodes)
    bw = min(24, group_w * 0.24)
    for i, node in enumerate(nodes):
        cx = left + (i + 0.5) * group_w
        for j, mode in enumerate(["F2", "F2D"]):
            vals = values[(mode, node)]
            if not vals:
                continue
            vals = sorted(vals)
            q1, med, q3 = percentile(vals, 0.25), percentile(vals, 0.5), percentile(vals, 0.75)
            lo, hi = max(0.0, min(vals)), min(ymax, max(vals))
            x = cx + (j - 0.5) * (bw + 6)
            def y(v: float) -> float:
                return top + plot_h - min(ymax, v) / ymax * plot_h
            svg.line(x, y(lo), x, y(hi), stroke="#2E3A46", sw=1.2)
            svg.line(x - bw * 0.35, y(lo), x + bw * 0.35, y(lo), stroke="#2E3A46", sw=1.2)
            svg.line(x - bw * 0.35, y(hi), x + bw * 0.35, y(hi), stroke="#2E3A46", sw=1.2)
            svg.rect(x - bw / 2, y(q3), bw, max(1, y(q1) - y(q3)), COLORS[mode], stroke="#2E3A46", sw=0.8, opacity=0.72)
            svg.line(x - bw / 2, y(med), x + bw / 2, y(med), stroke="#111111", sw=1.6)
        svg.text(cx, top + plot_h + 30, node.replace("Node", "N"), "tick", rotate=32)
    svg.save(path)


def topic_family(topic: str, payload_type: str = "") -> str:
    topic = topic or ""
    payload_type = payload_type or ""
    if "downstream" in topic or "inspection" in topic or "context" in topic:
        if "request" in topic:
            return "Drone request"
        return "Drone context"
    if "/state" in topic:
        return "Drone state"
    if "/health" in topic:
        return "Drone health"
    if "capabilities" in topic or "catalog" in topic or "membership" in topic or payload_type in {"DroneCapabilities", "CrazyflieSquareDigitalTwin"}:
        return "Discovery/catalog"
    return "Other"


def plot_drone_fnm_communication_volume(plot_dir: Path, path: Path) -> None:
    rows = read_csv(plot_dir / "f2d_edge_payload_messages.csv")
    families = ["Discovery/catalog", "Drone state", "Drone health", "Drone request", "Drone context", "Other"]
    counts = Counter()
    payload = Counter()
    for r in rows:
        fam = topic_family(r.get("topic", ""), r.get("payload_type", ""))
        counts[fam] += 1
        payload[fam] += fnum(r.get("payload_bytes"), 0.0)
    width, height = 1040, 390
    svg = Svg(width, height)
    panels = [
        (72, 486, "Messages (count)", counts, 1.0, "#5B9BD5"),
        (600, 1010, "Payload Volume (MB)", payload, 1_000_000, "#1B9E77"),
    ]
    for left, right, title, data, denom, color in panels:
        top, bottom = 58, 294
        vals = [(f, data.get(f, 0) / denom) for f in families if data.get(f, 0) > 0]
        if not vals:
            vals = [(families[0], 0)]
        ymax = max(1, max(v for _, v in vals) * 1.25)
        if denom > 1:
            ymax = max(0.01, ymax)
        svg.text((left + right) / 2, 28, title, "label")
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            label = f"{val:.0f}" if denom == 1 else f"{val:.2f}"
            svg.text(left - 8, y + 5, label, "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(42, (right - left) / max(1, len(vals)) * 0.5)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            shown = f"{val:.0f}" if denom == 1 else f"{val:.2f}"
            svg.text(x, bottom - h - 7, shown, "value")
            svg.text(x, bottom + 24, label, "small", rotate=24)
    svg.save(path)


def is_drone_row(row: dict) -> bool:
    try:
        blob = json.dumps(row)
    except Exception:
        blob = str(row)
    return "crazyflie_01" in blob or "gw-drone-crazyflie-01" in blob or "drone" in blob.lower()


def summarize_core_services(core_logs: dict[str, list[dict]]) -> list[dict]:
    out = []
    labels = {
        "membership": "Membership",
        "catalog": "Catalog",
        "discovery": "Discovery",
        "lifecycle": "Lifecycle",
        "state_manager": "State manager",
        "adaptive_connectivity": "Adaptive connectivity",
        "metrics": "Metrics",
    }
    for svc, rows in core_logs.items():
        latencies = []
        drone_rows = 0
        for r in rows:
            if is_drone_row(r):
                drone_rows += 1
            v = fnum(r.get("latency_ms"))
            if math.isfinite(v):
                latencies.append(v)
        out.append({
            "service": svc,
            "label": labels.get(svc, svc),
            "event_count": len(rows),
            "drone_event_count": drone_rows,
            "latency_samples_n": len(latencies),
            "median_latency_ms": statistics.median(latencies) if latencies else math.nan,
            "p95_latency_ms": percentile(latencies, 0.95) if latencies else math.nan,
        })
    return out


def plot_core_service_costs(summary_rows: list[dict], path: Path) -> None:
    rows = [r for r in summary_rows if r["event_count"] > 0]
    labels = [r["label"] for r in rows]
    width, height = 1240, 390
    svg = Svg(width, height)
    panels = [
        (86, 390, "Total service activity", "Service Events (thousands)", [(r["label"], r["event_count"] / 1000.0) for r in rows], "#5B9BD5", "{:.1f}"),
        (500, 785, "Drone-DT related activity", "Events (count)", [(r["label"], r["drone_event_count"]) for r in rows], "#1B9E77", "{:.0f}"),
        (900, 1208, "Median service processing", "Latency (ms)", [(r["label"], r["median_latency_ms"]) for r in rows if math.isfinite(r["median_latency_ms"])], "#E07A32", "{:.2f}"),
    ]
    for left, right, title, ylabel, vals, color, fmt in panels:
        top, bottom = 58, 288
        vals = [(k, v) for k, v in vals if math.isfinite(v)]
        ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
        svg.text((left + right) / 2, 28, title, "label")
        svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            svg.text(left - 8, y + 5, f"{val:.0f}" if ymax > 10 else f"{val:.1f}", "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(28, (right - left) / max(1, len(vals)) * 0.45)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, fmt.format(val), "value")
            short = {
                "Membership": "Member.",
                "Catalog": "Catalog",
                "Discovery": "Discover.",
                "Lifecycle": "Lifecycle",
                "State manager": "State",
                "Adaptive connectivity": "Adaptive",
                "Metrics": "Metrics",
            }.get(label, label)
            svg.text(x, bottom + 24, short, "small", rotate=25)
    svg.save(path)


def drone_integration_steps(core_logs: dict[str, list[dict]]) -> list[dict]:
    candidates = [
        ("membership", "membership_rx", "Membership register RX"),
        ("membership", "membership_registered", "Membership registered"),
        ("lifecycle", "member_seen", "Lifecycle indexed"),
        ("catalog", "catalog_upsert", "Catalogued as AerialScoutSystem"),
        ("discovery", "discovery_rx", "Discovery observes member"),
        ("state_manager", "state_manager_dt_observed", "State manager observes DT"),
        ("adaptive_connectivity", "adaptive.connectivity.rx", "Adaptive connectivity observes"),
        ("membership", "membership_active", "Membership active"),
    ]
    rows = []
    for svc, event, label in candidates:
        matches = []
        for r in core_logs.get(svc, []):
            et = r.get("event") or r.get("event_type")
            if et != event:
                continue
            if event == "discovery_query_resp":
                if not (str(r.get("purpose", "")).find("drone") >= 0 or str(r.get("is_drone_query", "")) in {"1", "true", "True"}):
                    continue
            elif not is_drone_row(r):
                continue
            t = fnum(r.get("ts") or r.get("ts_wall_s"))
            if math.isfinite(t):
                matches.append((t, r))
        if matches:
            t, r = sorted(matches, key=lambda x: x[0])[0]
            rows.append({
                "service": svc,
                "event": event,
                "label": label,
                "ts_wall_s": t,
                "latency_ms": fnum(r.get("latency_ms")),
                "status": r.get("status", ""),
                "paper_lifecycle_state": r.get("paper_lifecycle_state", ""),
                "paper_lifecycle_phase": r.get("paper_lifecycle_phase", ""),
            })
    if rows:
        t0 = min(r["ts_wall_s"] for r in rows)
        for r in rows:
            r["relative_time_ms"] = (r["ts_wall_s"] - t0) * 1000.0
    return sorted(rows, key=lambda r: r["ts_wall_s"])


def plot_drone_integration_timeline(step_rows: list[dict], path: Path) -> None:
    width, height = 1100, 390
    left, right, top, bottom = 255, 34, 54, 58
    plot_w, plot_h = width - left - right, height - top - bottom
    svg = Svg(width, height)
    if not step_rows:
        svg.text(width / 2, height / 2, "No Drone-DT integration events found", "label")
        svg.save(path)
        return
    xmax = max(r["relative_time_ms"] for r in step_rows) * 1.12
    xmax = max(10.0, xmax)
    # If the active transition is around heartbeat cadence, keep a readable scale.
    xmax = max(xmax, 5500.0)
    for i in range(6):
        val = xmax * i / 5
        x = left + val / xmax * plot_w
        svg.line(x, top, x, top + plot_h, "grid")
        svg.text(x, top + plot_h + 30, f"{val/1000:.1f}", "tick")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(width / 2, height - 8, "Time since Drone-DT registration observed (s)", "label")
    for i, r in enumerate(step_rows):
        y = top + (i + 0.5) / len(step_rows) * plot_h
        x = left + r["relative_time_ms"] / xmax * plot_w
        svg.line(left, y, x, y, stroke="#C9D3DF", sw=2)
        svg.circle(x, y, 7, COLORS.get("Drone context", "#1B9E77"))
        svg.text(left - 10, y + 5, r["label"], "ytick", "end")
        lat = r.get("latency_ms")
        suffix = f"{lat:.2f} ms" if math.isfinite(lat) else ""
        if suffix:
            svg.text(x + 12, y + 5, suffix, "small", "start")
    svg.save(path)


def fnm_rule_family(rule: str, event: str, direction: str, artefact_kind: str) -> str:
    rule = rule or ""
    event = event or ""
    if "capabilities" in rule:
        return "Capabilities"
    if "state" in rule:
        return "Drone state"
    if "health" in rule:
        return "Drone health"
    if "events" in rule:
        return "Drone events"
    if "downstream_inspection" in rule:
        return "Drone request"
    if "downstream_context" in rule:
        return "Drone context"
    if event == "fnm.mqtt.publish":
        return "MQTT publish"
    return artefact_kind or direction or "Other"


def display_fnm_family_label(label: str) -> str:
    label = label.replace("Drone ", "")
    if not label:
        return label
    return label[0].upper() + label[1:]


def fnm_integration_cost_panels(plot_dir: Path) -> list[tuple]:
    rows = read_csv(plot_dir / "f2d_fnm_hop_traces.csv")
    families = ["Capabilities", "Drone state", "Drone health", "Drone events", "Drone request", "Drone context"]
    counts = Counter()
    payload = Counter()
    latencies = defaultdict(list)
    for r in rows:
        if not r.get("rule"):
            continue
        fam = fnm_rule_family(r.get("rule", ""), r.get("event", ""), r.get("direction", ""), r.get("artefact_kind", ""))
        if fam not in families:
            continue
        if r.get("event") == "fnm.route.local_to_fed" or r.get("event") == "fnm.route.fed_to_local":
            counts[fam] += 1
            payload[fam] += fnum(r.get("payload_size_bytes"), 0.0)
            v = fnum(r.get("duration_ms"))
            if math.isfinite(v):
                latencies[fam].append(v)
    return [
        (86, 370, "FNM route operations", "Operations (count)", [(f, counts[f]) for f in families], "#5B9BD5", "{:.0f}"),
        (470, 745, "FNM routed payload", "Payload (MB)", [(f, payload[f] / 1_000_000.0) for f in families], "#1B9E77", "{:.2f}"),
        (850, 1128, "Median FNM route latency", "Latency (ms)", [(f, statistics.median(latencies[f]) if latencies[f] else math.nan) for f in families], "#E07A32", "{:.2f}"),
    ]


def plot_fnm_integration_cost(plot_dir: Path, path: Path) -> None:
    panels = fnm_integration_cost_panels(plot_dir)
    width, height = 1160, 390
    svg = Svg(width, height)
    for left, right, title, ylabel, vals, color, fmt in panels:
        top, bottom = 58, 288
        vals = [(k, v) for k, v in vals if math.isfinite(v)]
        ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
        svg.text((left + right) / 2, 28, title, "label")
        svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            label = f"{val:.0f}" if ymax >= 10 else f"{val:.2f}"
            svg.text(left - 8, y + 5, label, "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(34, (right - left) / max(1, len(vals)) * 0.46)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, fmt.format(val), "value")
            svg.text(x, bottom + 24, display_fnm_family_label(label), "small", rotate=24)
    svg.save(path)


def plot_single_fnm_integration_cost_panel(panel: tuple, path: Path) -> list[dict]:
    _left_src, _right_src, title, ylabel, vals, color, fmt = panel
    vals = [(k, v) for k, v in vals if math.isfinite(v)]
    width, height = 520, 390
    left, right, top, bottom = 86, width - 28, 34, 270
    svg = Svg(width, height)
    ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
    svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
    for i in range(4):
        val = ymax * i / 3
        y = bottom - val / ymax * (bottom - top)
        svg.line(left, y, right, y, "grid")
        label = f"{val:.0f}" if ymax >= 10 else f"{val:.2f}"
        svg.text(left - 8, y + 5, label, "ytick", "end")
    svg.line(left, top, left, bottom, "axis")
    svg.line(left, bottom, right, bottom, "axis")
    bw = min(22, (right - left) / max(1, len(vals)) * 0.46)
    rows_out = []
    for i, (label, val) in enumerate(vals):
        x = left + (i + 0.5) / len(vals) * (right - left)
        h = val / ymax * (bottom - top)
        svg.rect(x - bw / 2, bottom - h, bw, h, color)
        svg.text(x, bottom - h - 7, fmt.format(val), "value")
        svg.text(x, bottom + 34, display_fnm_family_label(label), "small", rotate=24)
        rows_out.append({"panel": title, "metric": display_fnm_family_label(label), "value": val, "unit": ylabel})
    svg.save(path)
    return rows_out


def plot_fnm_integration_cost_split_panels(plot_dir: Path, out_dir: Path) -> list[dict]:
    panels = fnm_integration_cost_panels(plot_dir)
    filenames = [
        "route6_f2d_fnm_route_operations_3_1.svg",
        "route6_f2d_fnm_routed_payload_3_1.svg",
        "route6_f2d_fnm_route_latency_3_1.svg",
    ]
    rows = []
    for panel, filename in zip(panels, filenames):
        rows.extend(plot_single_fnm_integration_cost_panel(panel, out_dir / filename))
    return rows


def summary_by_mode(plot_dir: Path) -> dict[str, dict]:
    return {r.get("mode", ""): r for r in read_csv(plot_dir / "f2d_summary.csv")}


def pct_gain(base: float, improved: float) -> float:
    if not math.isfinite(base) or base == 0 or not math.isfinite(improved):
        return math.nan
    return (base - improved) / base * 100.0


def median_from_summary(plot_dir: Path, filename: str, metric_name: str) -> float:
    for r in read_csv(plot_dir / filename):
        if r.get("metric_name") == metric_name:
            return fnum(r.get("median"))
    return math.nan


def fnm_median(plot_dir: Path, direction: str, artefact_kind: str) -> float:
    for r in read_csv(plot_dir / "f2d_fnm_route_duration_summary.csv"):
        if r.get("direction") == direction and r.get("artefact_kind") == artefact_kind and r.get("event", "").startswith("fnm.route."):
            return fnum(r.get("median"))
    return math.nan


def service_latency(summary_rows: list[dict], service: str) -> float:
    for r in summary_rows:
        if r.get("service") == service:
            return fnum(r.get("median_latency_ms"))
    return math.nan


def plot_federated_plumbing_pipeline(
    plot_dir: Path,
    core_summary: list[dict],
    integration_steps: list[dict],
    events: list[dict],
    path: Path,
) -> list[dict]:
    summary = summary_by_mode(plot_dir)
    f2d = summary.get("F2D", {})
    request_n = int(fnum(f2d.get("drone_discovery_queries_n"), 0))
    context_rx_n = int(fnum(f2d.get("drone_context_received_n"), 0))
    context_used_n = int(fnum(f2d.get("drone_context_used_n"), 0))
    stale_n = int(fnum(f2d.get("drone_context_stale_n"), 0))
    edge_messages_n = int(fnum(f2d.get("edge_raw_drone_messages_n"), 0))
    edge_payload_mb = fnum(f2d.get("edge_raw_drone_payload_bytes"), 0) / 1_000_000.0
    active_step = next((r for r in integration_steps if r.get("event") == "membership_active"), {})
    active_sec = fnum(active_step.get("relative_time_ms")) / 1000.0
    catalog_ms = fnum(next((r for r in integration_steps if r.get("event") == "catalog_upsert"), {}).get("relative_time_ms"))
    request_med_ms = median_from_summary(plot_dir, "f2d_latency_chain_summary.csv", "request_to_drone_rx_latency_ms")
    context_med_ms = median_from_summary(plot_dir, "f2d_latency_chain_summary.csv", "drone_publish_to_realworld_rx_latency_ms")
    sumo_med_ms = median_from_summary(plot_dir, "f2d_latency_chain_summary.csv", "sumo_proxy_latency_ms")
    fnm_req_ms = fnm_median(plot_dir, "fed_to_local", "request_response")
    fnm_event_ms = fnm_median(plot_dir, "local_to_fed", "event")
    discovery_ms = service_latency(core_summary, "discovery")

    width, height = 1280, 530
    svg = Svg(width, height)
    svg.text(width / 2, 30, "Drone-DT federation plumbing for downstream context", "label")
    boxes = [
        (55, 95, 170, 145, "#EAF3FF", "Drone-DT", ["Physical scout", f"{edge_messages_n:,} edge msgs", f"{edge_payload_mb:.2f} MB payload"]),
        (270, 95, 185, 145, "#EFF8F1", "Edge FNM", ["Register/cap/state", f"Req route {fnm_req_ms:.2f} ms", f"Context route {fnm_event_ms:.2f} ms"]),
        (505, 80, 240, 175, "#FFF6E8", "Core federation services", ["Membership + lifecycle", "Catalog + discovery", f"Discovery med {discovery_ms:.1f} ms"]),
        (790, 95, 190, 145, "#F0F7F7", "Context topics", [f"{request_n} requests", f"{context_rx_n} context pubs", "node/edge/region scope"]),
        (1030, 95, 195, 145, "#F5F0FF", "SI-DT consumers", [f"{context_used_n} context uses", f"{stale_n} stale rejected", "freshness guard"]),
    ]
    for x, y, w, h, fill, title, lines in boxes:
        svg.rect(x, y, w, h, fill, stroke="#8A98A8", sw=1.4, rx=10)
        svg.text(x + w / 2, y + 32, title, "legend")
        for i, line in enumerate(lines):
            svg.text(x + w / 2, y + 64 + i * 24, line, "small")
    for i in range(len(boxes) - 1):
        x, y, w, h = boxes[i][:4]
        nx, ny, nw, nh = boxes[i + 1][:4]
        svg.arrow(x + w + 12, y + h / 2, nx - 12, ny + nh / 2, "#64748B", 2.2)

    bottom_boxes = [
        (110, 325, 260, 110, "#FFFFFF", "Join/readiness", [f"Catalogued in {catalog_ms:.1f} ms", f"Active in {active_sec:.1f} s"]),
        (510, 325, 260, 110, "#FFFFFF", "Runtime context chain", [f"SI-DT→Drone {request_med_ms:.1f} ms", f"SUMO query {sumo_med_ms:.1f} ms", f"Drone→SI-DT {context_med_ms:.1f} ms"]),
        (905, 325, 260, 110, "#FFFFFF", "Middleware safety", ["Context cached per SI-DT", "Freshness checked before use", "Stale context rejected"]),
    ]
    for x, y, w, h, fill, title, lines in bottom_boxes:
        svg.rect(x, y, w, h, fill, stroke="#ADB7C3", sw=1.2, rx=10)
        svg.text(x + w / 2, y + 28, title, "legend")
        for i, line in enumerate(lines):
            svg.text(x + w / 2, y + 58 + i * 22, line, "small")
    svg.save(path)

    return [
        {"metric": "edge_messages_n", "value": edge_messages_n, "unit": "count"},
        {"metric": "edge_payload_mb", "value": edge_payload_mb, "unit": "MB"},
        {"metric": "drone_requests_n", "value": request_n, "unit": "count"},
        {"metric": "drone_context_publications_n", "value": context_rx_n, "unit": "count"},
        {"metric": "si_dt_context_uses_n", "value": context_used_n, "unit": "count"},
        {"metric": "stale_context_rejections_n", "value": stale_n, "unit": "count"},
        {"metric": "join_to_catalog_ms", "value": catalog_ms, "unit": "ms"},
        {"metric": "join_to_active_sec", "value": active_sec, "unit": "s"},
        {"metric": "request_to_drone_median_ms", "value": request_med_ms, "unit": "ms"},
        {"metric": "sumo_query_median_ms", "value": sumo_med_ms, "unit": "ms"},
        {"metric": "drone_to_si_dt_median_ms", "value": context_med_ms, "unit": "ms"},
        {"metric": "fnm_request_route_median_ms", "value": fnm_req_ms, "unit": "ms"},
        {"metric": "fnm_context_route_median_ms", "value": fnm_event_ms, "unit": "ms"},
    ]


def plot_value_vs_federation_cost(plot_dir: Path, path: Path) -> list[dict]:
    summary = summary_by_mode(plot_dir)
    f2 = summary.get("F2", {})
    f2d = summary.get("F2D", {})
    benefits = [
        ("Travel", pct_gain(fnum(f2.get("travel_time_s")), fnum(f2d.get("travel_time_s")))),
        ("Waiting", pct_gain(fnum(f2.get("waiting_time_s")), fnum(f2d.get("waiting_time_s")))),
        ("Time Loss", pct_gain(fnum(f2.get("time_loss_s")), fnum(f2d.get("time_loss_s")))),
        ("Stops", pct_gain(fnum(f2.get("waiting_count_n")), fnum(f2d.get("waiting_count_n")))),
    ]
    cost_rows = [
        ("Messages", fnum(f2d.get("edge_raw_drone_messages_n"), 0), "count"),
        ("Payload", fnum(f2d.get("edge_raw_drone_payload_bytes"), 0) / 1_000_000.0, "MB"),
        ("Context pubs", fnum(f2d.get("drone_context_received_n"), 0), "count"),
        ("Context uses", fnum(f2d.get("drone_context_used_n"), 0), "count"),
        ("Pipeline med.", median_from_summary(plot_dir, "f2d_latency_chain_summary.csv", "drone_publish_to_realworld_rx_latency_ms"), "ms"),
        ("FNM route med.", fnm_median(plot_dir, "local_to_fed", "event"), "ms"),
    ]

    width, height = 1120, 430
    svg = Svg(width, height)
    panels = [
        (78, 510, "Traffic improvement vs FCDP (%)", benefits, "#1B9E77", 100.0, "{:.1f}"),
        (665, 1070, "Federation cost indicators", [(k, v) for k, v, _ in cost_rows], "#5B6472", None, "{:.1f}"),
    ]
    for left, right, title, vals, color, forced_ymax, fmt in panels:
        top, bottom = 58, 305
        ymax = forced_ymax or max(1.0, max((v for _, v in vals if math.isfinite(v)), default=1.0) * 1.25)
        svg.text((left + right) / 2, 28, title, "label")
        for i in range(5):
            val = ymax * i / 4
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            label = f"{val:.0f}" if ymax > 20 else f"{val:.1f}"
            svg.text(left - 8, y + 5, label, "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(46, (right - left) / max(1, len(vals)) * 0.46)
        for i, (label, val) in enumerate(vals):
            if not math.isfinite(val):
                val = 0.0
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = min(ymax, val) / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, fmt.format(val), "value")
            svg.text(x, bottom + 24, label, "small", rotate=20)
    svg.text(665, 365, "Cost panel mixes units; values are labeled in CSV for paper text.", "small", "start")
    svg.save(path)

    rows = [{"category": "benefit", "metric": k, "value": v, "unit": "percent"} for k, v in benefits]
    rows.extend({"category": "cost", "metric": k, "value": v, "unit": u} for k, v, u in cost_rows)
    return rows


def plot_cost_summary(plot_dir: Path, path: Path) -> None:
    latency = {r["metric_name"]: fnum(r.get("median")) for r in read_csv(plot_dir / "f2d_latency_chain_summary.csv")}
    payload = {r["metric_name"]: fnum(r.get("median")) for r in read_csv(plot_dir / "f2d_payload_summary.csv")}
    fnm = read_csv(plot_dir / "f2d_fnm_route_duration_summary.csv")
    fnm_rows = [
        r
        for r in fnm
        if r.get("event", "").startswith("fnm.route.")
        and r.get("direction") in {"local_to_fed", "fed_to_local"}
        and math.isfinite(fnum(r.get("median")))
    ]
    width, height = 1120, 380
    svg = Svg(width, height)
    panels = [
        (84, 340, "Median context pipeline", "Latency (ms)", [
            ("SI-DT -> Drone-DT\nRequest", latency.get("request_to_drone_rx_latency_ms", 0)),
            ("Real-world\nscouting", latency.get("sumo_proxy_latency_ms", 0)),
            ("Drone-DT -> SI-DT\nSupport", latency.get("drone_publish_to_realworld_rx_latency_ms", 0)),
        ], COLORS["F2D"]),
        (420, 675, "Median context payload", "Payload (kB)", [
            ("SI-DT request", payload.get("request_payload_size_bytes", 0) / 1_000.0),
            ("Drone-DT received\nrequest", payload.get("drone_rx_payload_size_bytes", 0) / 1_000.0),
            ("Drone-DT support\ncontext", payload.get("response_payload_size_bytes", 0) / 1_000.0),
        ], "#5B6472"),
        (755, 1060, "Median FNM route", "Latency (ms)", [
            (
                {
                    ("fed_to_local", "request_response"): "Federation -> DT\nrequest",
                    ("local_to_fed", "event"): "DT -> federation\nevent",
                    ("local_to_fed", "state"): "DT -> federation\nstate",
                }.get((r.get("direction"), r.get("artefact_kind")), f"{r.get('direction')} {r.get('artefact_kind')}"),
                fnum(r.get("median")),
            )
            for r in fnm_rows
        ], COLORS["F2"]),
    ]
    for left, right, title, ylabel, vals, color in panels:
        top, bottom = 58, 306
        svg.text((left + right) / 2, 28, title, "label")
        vals = [(label, value) for label, value in vals if math.isfinite(value)]
        if not vals:
            svg.line(left, top, left, bottom, "axis")
            svg.line(left, bottom, right, bottom, "axis")
            svg.text((left + right) / 2, (top + bottom) / 2, "No FNM route samples", "small")
            continue
        ymax = max(1, max(v for _, v in vals) * 1.25)
        svg.text(left - 52, (top + bottom) / 2, ylabel, "small", rotate=-90)
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            svg.text(left - 8, y + 5, f"{val:.0f}" if ymax >= 10 else f"{val:.1f}", "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(46, (right - left) / max(1, len(vals)) * 0.46)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, f"{val:.1f}", "value")
            draw_multiline_text(svg, x, bottom + 30, label, "small", line_h=15)
    if not fnm_rows:
        print(
            "WARNING: No FNM route-duration samples found in "
            f"{plot_dir / 'f2d_fnm_route_duration_summary.csv'}. "
            "Regenerate the F2D source bundle with the correct --fnm-trace input."
        )
    svg.save(path)


def context_service_cost_panels(plot_dir: Path, core_summary: list[dict]) -> list[tuple]:
    latency = {r["metric_name"]: fnum(r.get("median")) for r in read_csv(plot_dir / "f2d_latency_chain_summary.csv")}
    service_rows = [r for r in core_summary if r["event_count"] > 0]
    service_activity = [(r["label"], r["drone_event_count"]) for r in service_rows if r["drone_event_count"] > 0]
    service_latency = [(r["label"], r["median_latency_ms"]) for r in service_rows if math.isfinite(r["median_latency_ms"])]
    return [
        (
            86,
            368,
            "Median context pipeline",
            "Latency (ms)",
            [
                ("SI-DT ->\nDrone-DT\nRequest", latency.get("request_to_drone_rx_latency_ms", 0)),
                ("Real-world\nscouting", latency.get("sumo_proxy_latency_ms", 0)),
                ("Drone-DT ->\nSI-DT\nSupport", latency.get("drone_publish_to_realworld_rx_latency_ms", 0)),
            ],
            "#5B9BD5",
            "{:.1f}",
        ),
        (
            465,
            790,
            "Drone-DT service activity",
            "Events (count)",
            service_activity,
            "#1B9E77",
            "{:.0f}",
        ),
        (
            885,
            1190,
            "Median service processing",
            "Latency (ms)",
            service_latency,
            "#E07A32",
            "{:.2f}",
        ),
    ]


def plot_context_service_cost_consolidated(plot_dir: Path, core_summary: list[dict], path: Path) -> list[dict]:
    """Single 3-panel paper plot: drone context latency + service activity + service latency."""
    panels = context_service_cost_panels(plot_dir, core_summary)
    width, height = 1240, 410
    top, bottom = 58, 292
    svg = Svg(width, height)
    rows_out = []
    for left, right, title, ylabel, vals, color, fmt in panels:
        vals = [(k, v) for k, v in vals if math.isfinite(v)]
        ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
        svg.text((left + right) / 2, 28, title, "label")
        svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            svg.text(left - 8, y + 5, f"{val:.0f}" if ymax >= 10 else f"{val:.1f}", "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        # Keep visual weight consistent across panels; otherwise the 3-item
        # context pipeline panel looks artificially heavier than service panels.
        bw = min(22, (right - left) / max(1, len(vals)) * 0.46)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, fmt.format(val), "value")
            draw_rotated_multiline_text(svg, x, bottom + 34, wrap_axis_label(label), "small", line_h=14, rotate=24)
            rows_out.append({"panel": title, "metric": label.replace("\n", " "), "value": val, "unit": ylabel})
    svg.save(path)
    return rows_out


def plot_single_context_service_cost_panel(panel: tuple, path: Path) -> list[dict]:
    """Render one cost panel without an internal title for LaTeX subfigures."""
    _left_src, _right_src, title, ylabel, vals, color, fmt = panel
    vals = [(k, v) for k, v in vals if math.isfinite(v)]
    width, height = 520, 390
    left, right, top, bottom = 86, width - 28, 34, 270
    svg = Svg(width, height)
    ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
    svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
    for i in range(4):
        val = ymax * i / 3
        y = bottom - val / ymax * (bottom - top)
        svg.line(left, y, right, y, "grid")
        svg.text(left - 8, y + 5, f"{val:.0f}" if ymax >= 10 else f"{val:.1f}", "ytick", "end")
    svg.line(left, top, left, bottom, "axis")
    svg.line(left, bottom, right, bottom, "axis")
    bw = min(22, (right - left) / max(1, len(vals)) * 0.46)
    rows_out = []
    for i, (label, val) in enumerate(vals):
        x = left + (i + 0.5) / len(vals) * (right - left)
        h = val / ymax * (bottom - top)
        svg.rect(x - bw / 2, bottom - h, bw, h, color)
        svg.text(x, bottom - h - 7, fmt.format(val), "value")
        draw_rotated_multiline_text(svg, x, bottom + 36, wrap_axis_label(label), "small", line_h=14, rotate=24)
        rows_out.append({"panel": title, "metric": label.replace("\n", " "), "value": val, "unit": ylabel})
    svg.save(path)
    return rows_out


def plot_context_service_cost_split_panels(plot_dir: Path, core_summary: list[dict], out_dir: Path) -> list[dict]:
    panels = context_service_cost_panels(plot_dir, core_summary)
    filenames = [
        "route6_f2d_context_pipeline_latency_3_1.svg",
        "route6_f2d_drone_dt_service_activity_3_1.svg",
        "route6_f2d_service_processing_latency_3_1.svg",
    ]
    rows = []
    for panel, filename in zip(panels, filenames):
        rows.extend(plot_single_context_service_cost_panel(panel, out_dir / filename))
    return rows


def plot_fnm_service_cost_consolidated(plot_dir: Path, core_summary: list[dict], path: Path) -> list[dict]:
    """Single 3-panel plot focused on FNM routing plus core service cost."""
    fnm_rows = read_csv(plot_dir / "f2d_fnm_hop_traces.csv")
    families = ["Capabilities", "Drone state", "Drone health", "Drone events", "Drone request", "Drone context"]
    counts = Counter()
    route_latencies = defaultdict(list)
    for r in fnm_rows:
        fam = fnm_rule_family(r.get("rule", ""), r.get("event", ""), r.get("direction", ""), r.get("artefact_kind", ""))
        if fam not in families:
            continue
        if r.get("event") in {"fnm.route.local_to_fed", "fnm.route.fed_to_local"}:
            counts[fam] += 1
            v = fnum(r.get("duration_ms"))
            if math.isfinite(v):
                route_latencies[fam].append(v)
    service_rows = [r for r in core_summary if r["event_count"] > 0]
    panels = [
        (
            86,
            370,
            "FNM route operations",
            "Operations (count)",
            [(f, counts[f]) for f in families],
            "#5B9BD5",
            "{:.0f}",
        ),
        (
            480,
            750,
            "Drone-DT service activity",
            "Events (count)",
            [(r["label"], r["drone_event_count"]) for r in service_rows if r["drone_event_count"] > 0],
            "#1B9E77",
            "{:.0f}",
        ),
        (
            875,
            1190,
            "Median FNM route latency",
            "Latency (ms)",
            [(f, statistics.median(route_latencies[f]) if route_latencies[f] else math.nan) for f in families],
            "#E07A32",
            "{:.2f}",
        ),
    ]
    width, height = 1240, 410
    top, bottom = 58, 292
    svg = Svg(width, height)
    rows_out = []
    short_labels = {
        "Membership": "Member.",
        "Catalog": "Catalog",
        "Discovery": "Discover.",
        "Lifecycle": "Lifecycle",
        "State manager": "State",
        "Adaptive connectivity": "Adaptive",
        "Metrics": "Metrics",
    }
    for left, right, title, ylabel, vals, color, fmt in panels:
        vals = [(k, v) for k, v in vals if math.isfinite(v)]
        ymax = max(1.0, max((v for _, v in vals), default=1.0) * 1.28)
        svg.text((left + right) / 2, 28, title, "label")
        svg.text(left - 54, (top + bottom) / 2, ylabel, "small", rotate=-90)
        for i in range(4):
            val = ymax * i / 3
            y = bottom - val / ymax * (bottom - top)
            svg.line(left, y, right, y, "grid")
            svg.text(left - 8, y + 5, f"{val:.0f}" if ymax >= 10 else f"{val:.2f}", "ytick", "end")
        svg.line(left, top, left, bottom, "axis")
        svg.line(left, bottom, right, bottom, "axis")
        bw = min(34, (right - left) / max(1, len(vals)) * 0.46)
        for i, (label, val) in enumerate(vals):
            x = left + (i + 0.5) / len(vals) * (right - left)
            h = val / ymax * (bottom - top)
            svg.rect(x - bw / 2, bottom - h, bw, h, color)
            svg.text(x, bottom - h - 7, fmt.format(val), "value")
            out_label = short_labels.get(label, label.replace("Drone ", ""))
            svg.text(x, bottom + 24, out_label, "small", rotate=24)
            rows_out.append({"panel": title, "metric": label, "value": val, "unit": ylabel})
    svg.save(path)
    return rows_out


def convert_pdfs(out_dir: Path) -> None:
    exe = shutil.which("rsvg-convert")
    if not exe:
        return
    pdf_dir = out_dir / "pdf"
    pdf_dir.mkdir(exist_ok=True)
    for svg in out_dir.glob("*.svg"):
        subprocess.run([exe, "-f", "pdf", "-o", str(pdf_dir / f"{svg.stem}.pdf"), str(svg)], check=True)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", type=Path, required=True, help="Route 6 ev_matrix_results folder")
    p.add_argument("--f2d-plot-dir", type=Path, required=True, help="CSV bundle emitted by make_f2d_drone_plots.py")
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--core-log-dir", type=Path, default=None, help="Federation core log dir. Defaults to sibling federation_core_logs next to ev_matrix_results.")
    p.add_argument("--export-pdf", action="store_true")
    p.add_argument("--runtime-panel-width", type=int, default=1040, help="Width shared by the paired runtime plots.")
    p.add_argument("--runtime-panel-height", type=int, default=680, help="Height shared by the paired runtime plots.")
    p.add_argument("--runtime-xmin", type=float, default=1020.0, help="Shared simulation-time lower bound.")
    p.add_argument("--runtime-xmax", type=float, default=1120.0, help="Shared simulation-time upper bound.")
    p.add_argument("--runtime-legend-font-size", type=float, default=20.0)
    p.add_argument("--runtime-axis-label-font-size", type=float, default=22.0)
    p.add_argument("--runtime-x-tick-font-size", type=float, default=19.0)
    p.add_argument("--runtime-y-tick-font-size", type=float, default=17.0)
    p.add_argument(
        "--runtime-timeline-legend-offset-y",
        type=float,
        default=32.0,
        help="Downward offset for the timeline legend rows within the shared legend band.",
    )
    p.add_argument(
        "--runtime-service-ylabel-x",
        type=float,
        default=82.0,
        help="Horizontal position of the service-activity y-axis label; larger values move it toward the ticks.",
    )
    args = p.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    events = load_events(args.run_dir)
    runtime_style = {
        "legend_font_size": args.runtime_legend_font_size,
        "legend_font_weight": 400,
        "label_font_size": args.runtime_axis_label_font_size,
        "tick_font_size": args.runtime_x_tick_font_size,
        "ytick_font_size": args.runtime_y_tick_font_size,
        "small_font_size": max(14.0, args.runtime_y_tick_font_size - 1.0),
    }
    runtime_plot_options = {
        "width": args.runtime_panel_width,
        "height": args.runtime_panel_height,
        "xmin": args.runtime_xmin,
        "xmax": args.runtime_xmax,
        "style": runtime_style,
    }
    timeline_summary = plot_node_timeline(
        events,
        args.out_dir / "route6_f2d_node_context_timeline_2_1.svg",
        legend_offset_y=args.runtime_timeline_legend_offset_y,
        **runtime_plot_options,
    )
    freshness_summary = plot_freshness(events, args.out_dir / "route6_f2d_context_freshness_by_tls_1_1.svg")
    plot_node_activity(args.f2d_plot_dir, args.out_dir / "route6_f2_vs_f2d_node_activity_1_1.svg")
    plot_route_node_activity_stacked(args.f2d_plot_dir, args.out_dir / "route6_f2_vs_f2d_route_node_activity_stacked_1_1.svg")
    plot_route_node_activity_vertical(args.f2d_plot_dir, args.out_dir / "route6_f2_vs_f2d_route_node_activity_vertical_2_1.svg")
    plot_runtime_service_burst(
        events,
        args.out_dir / "route6_f2d_runtime_service_burst_area_2_1.svg",
        ylabel_x=args.runtime_service_ylabel_x,
        **runtime_plot_options,
    )
    plot_cost_summary(args.f2d_plot_dir, args.out_dir / "route6_f2d_latency_payload_overhead_3_1.svg")
    plot_drone_fnm_communication_volume(args.f2d_plot_dir, args.out_dir / "route6_f2d_drone_fnm_communication_volume_2_1.svg")
    mode_events = {"F2": load_mode_events(args.run_dir, "F2"), "F2D": load_mode_events(args.run_dir, "F2D")}
    rtd_samples = request_to_decision_samples(mode_events)
    plot_request_to_decision_boxplot(rtd_samples, args.out_dir / "route6_f2_vs_f2d_request_to_decision_latency_by_node_1_1.svg")
    core_log_dir = args.core_log_dir or default_core_log_dir(args.run_dir)
    core_logs = load_core_logs(core_log_dir)
    core_summary = summarize_core_services(core_logs)
    integration_steps = drone_integration_steps(core_logs)
    plot_core_service_costs(core_summary, args.out_dir / "route6_f2d_core_service_integration_costs_3_1.svg")
    plot_drone_integration_timeline(integration_steps, args.out_dir / "route6_f2d_drone_dt_integration_timeline_2_1.svg")
    plot_fnm_integration_cost(args.f2d_plot_dir, args.out_dir / "route6_f2d_fnm_integration_cost_3_1.svg")
    split_fnm_integration_cost = plot_fnm_integration_cost_split_panels(args.f2d_plot_dir, args.out_dir)
    consolidated_cost = plot_context_service_cost_consolidated(
        args.f2d_plot_dir,
        core_summary,
        args.out_dir / "route6_f2d_context_services_cost_consolidated_3_1.svg",
    )
    split_context_service_cost = plot_context_service_cost_split_panels(args.f2d_plot_dir, core_summary, args.out_dir)
    fnm_services_cost = plot_fnm_service_cost_consolidated(
        args.f2d_plot_dir,
        core_summary,
        args.out_dir / "route6_f2d_fnm_services_cost_consolidated_3_1.svg",
    )
    plumbing_summary = plot_federated_plumbing_pipeline(
        args.f2d_plot_dir,
        core_summary,
        integration_steps,
        events,
        args.out_dir / "route6_f2d_federated_plumbing_pipeline_1_1.svg",
    )
    value_cost_summary = plot_value_vs_federation_cost(
        args.f2d_plot_dir,
        args.out_dir / "route6_f2d_value_vs_federation_cost_2_1.svg",
    )
    write_csv(args.out_dir / "route6_f2d_node_context_timeline_summary.csv", timeline_summary)
    write_csv(args.out_dir / "route6_f2d_context_freshness_by_tls.csv", freshness_summary)
    write_csv(args.out_dir / "route6_f2_vs_f2d_request_to_decision_latency_by_node.csv", rtd_samples)
    write_csv(args.out_dir / "route6_f2d_core_service_integration_costs.csv", core_summary)
    write_csv(args.out_dir / "route6_f2d_context_services_cost_consolidated.csv", consolidated_cost)
    write_csv(args.out_dir / "route6_f2d_context_services_cost_split_panels.csv", split_context_service_cost)
    write_csv(args.out_dir / "route6_f2d_fnm_integration_cost_split_panels.csv", split_fnm_integration_cost)
    write_csv(args.out_dir / "route6_f2d_fnm_services_cost_consolidated.csv", fnm_services_cost)
    write_csv(args.out_dir / "route6_f2d_drone_dt_integration_timeline.csv", integration_steps)
    write_csv(args.out_dir / "route6_f2d_federated_plumbing_pipeline.csv", plumbing_summary)
    write_csv(args.out_dir / "route6_f2d_value_vs_federation_cost.csv", value_cost_summary)
    if args.export_pdf:
        convert_pdfs(args.out_dir)
    print(f"Wrote Route 6 F2D middleware paper plots to {args.out_dir}")


if __name__ == "__main__":
    main()
