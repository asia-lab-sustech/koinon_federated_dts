#!/usr/bin/env python3
"""
Render GTCO DOT snapshots into SVG frames and a lightweight HTML animation.

Typical usage:

python3 animate_gtco_graph.py \
  --dot-dir /tmp/gtco_graphviz \
  --output-dir /tmp/gtco_graphviz_anim \
  --network-file /path/to/madrid_simplified.net.xml \
  --render-mode both \
  --watch
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>GTCO Graph Animation</title>
  <style>
    :root {{
      --bg: #f5f1e8;
      --fg: #1d2a33;
      --muted: #5a6871;
      --accent: #8c3b2f;
      --panel: #fffaf0;
      --line: #d9cfbf;
      --chip: #f0e4d1;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "IBM Plex Sans", "Helvetica Neue", sans-serif;
      background: radial-gradient(circle at top, #fffaf0, var(--bg) 60%);
      color: var(--fg);
    }}
    .wrap {{
      max-width: 1380px;
      margin: 0 auto;
      padding: 24px;
    }}
    .top {{
      display: flex;
      gap: 16px;
      align-items: center;
      justify-content: space-between;
      flex-wrap: wrap;
      margin-bottom: 16px;
    }}
    .meta {{
      color: var(--muted);
      font-size: 14px;
    }}
    .controls {{
      display: flex;
      gap: 10px;
      align-items: center;
      flex-wrap: wrap;
    }}
    button, select {{
      border: 1px solid var(--line);
      background: var(--panel);
      color: var(--fg);
      padding: 8px 12px;
      border-radius: 999px;
      cursor: pointer;
    }}
    button:hover, select:hover {{
      border-color: var(--accent);
      color: var(--accent);
    }}
    input[type="range"] {{
      width: 220px;
    }}
    .stage {{
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.7);
      border-radius: 24px;
      overflow: hidden;
      min-height: 70vh;
      box-shadow: 0 20px 60px rgba(40, 35, 25, 0.08);
    }}
    .frame {{
      width: 100%;
      height: 72vh;
      display: block;
      object-fit: contain;
      background: #f8f7f2;
    }}
    .caption {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      padding: 12px 16px;
      border-top: 1px solid var(--line);
      color: var(--muted);
      background: rgba(255,250,240,0.9);
      font-size: 14px;
    }}
    .metrics {{
      display: grid;
      grid-template-columns: repeat(6, minmax(120px, 1fr));
      gap: 10px;
      margin-top: 14px;
    }}
    .metric {{
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 10px 12px;
      background: rgba(255,250,240,0.88);
    }}
    .metric .k {{
      display: block;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 4px;
    }}
    .metric .v {{
      font-size: 18px;
      font-weight: 600;
    }}
    .legend {{
      display: flex;
      gap: 14px;
      flex-wrap: wrap;
      margin-top: 14px;
      color: var(--muted);
      font-size: 13px;
    }}
    .legend span::before {{
      content: "";
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 999px;
      margin-right: 6px;
      vertical-align: -1px;
    }}
    .current::before {{ background: dodgerblue; }}
    .alt::before {{ background: firebrick; }}
    .overlap::before {{ background: goldenrod; }}
    .bg::before {{ background: #c7c3bc; }}
    .note {{
      margin-top: 8px;
      color: var(--muted);
      font-size: 13px;
    }}
    .hidden {{ display: none; }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div>
        <h1 style="margin:0 0 6px 0; font-size: 28px;">GTCO Graph Evolution</h1>
        <div class="meta" id="meta">Loading frames...</div>
      </div>
      <div class="controls">
        <button id="playBtn">Pause</button>
        <label>Speed <input id="speed" type="range" min="200" max="3000" step="100" value="{interval_ms}"></label>
        <label>Frame <input id="seek" type="range" min="0" max="0" step="1" value="0"></label>
        <label>View
          <select id="viewMode">
            <option value="preferred">Preferred</option>
            <option value="geo">Geographic</option>
            <option value="graphviz">Graphviz</option>
          </select>
        </label>
      </div>
    </div>
    <div class="stage">
      <img id="frame" class="frame" alt="GTCO frame" src="">
      <div class="caption">
        <div id="frameLabel">Frame: -</div>
        <div id="simLabel">Sim: -</div>
      </div>
    </div>
    <div class="metrics">
      <div class="metric"><span class="k">Current Cost</span><span class="v" id="mCurrentCost">-</span></div>
      <div class="metric"><span class="k">Optimized Cost</span><span class="v" id="mOptimizedCost">-</span></div>
      <div class="metric"><span class="k">Improvement</span><span class="v" id="mImprove">-</span></div>
      <div class="metric"><span class="k">Current Edges</span><span class="v" id="mCurrentEdges">-</span></div>
      <div class="metric"><span class="k">Alt Edges</span><span class="v" id="mAltEdges">-</span></div>
      <div class="metric"><span class="k">Overlap</span><span class="v" id="mOverlap">-</span></div>
    </div>
    <div class="legend">
      <span class="bg">Network background</span>
      <span class="current">Current EV route</span>
      <span class="alt">Optimized alternative</span>
      <span class="overlap">Overlap</span>
    </div>
    <div class="note">
      For local `file://` viewing, this page uses the embedded manifest and reloads itself every {refresh_sec}s to pick up new frames.
    </div>
  </div>
  <script>
    const manifestPath = "manifest.json";
    const embeddedManifest = {manifest_json};
    let frames = [];
    let idx = 0;
    let playing = true;
    let timer = null;

    const frameEl = document.getElementById("frame");
    const metaEl = document.getElementById("meta");
    const frameLabelEl = document.getElementById("frameLabel");
    const simLabelEl = document.getElementById("simLabel");
    const playBtn = document.getElementById("playBtn");
    const seekEl = document.getElementById("seek");
    const speedEl = document.getElementById("speed");
    const viewModeEl = document.getElementById("viewMode");

    const metrics = {{
      currentCost: document.getElementById("mCurrentCost"),
      optimizedCost: document.getElementById("mOptimizedCost"),
      improve: document.getElementById("mImprove"),
      currentEdges: document.getElementById("mCurrentEdges"),
      altEdges: document.getElementById("mAltEdges"),
      overlap: document.getElementById("mOverlap"),
    }};

    function fmtSec(v) {{
      if (v === null || v === undefined || Number.isNaN(Number(v))) return "-";
      return `${{Number(v).toFixed(1)}}s`;
    }}

    function preferredSvg(frame) {{
      const mode = viewModeEl.value || "preferred";
      if (mode === "geo") return frame.geo_svg || frame.graphviz_svg || "";
      if (mode === "graphviz") return frame.graphviz_svg || frame.geo_svg || "";
      return frame.svg || frame.geo_svg || frame.graphviz_svg || "";
    }}

    function applyManifest(data) {{
      const prevPath = frames[idx] ? preferredSvg(frames[idx]) : null;
      frames = Array.isArray(data.frames) ? data.frames : [];
      metaEl.textContent = `${{frames.length}} frames | generated ${{data.generated_at || "-"}}`;
      seekEl.max = Math.max(0, frames.length - 1);
      if (!frames.length) {{
        frameEl.src = "";
        frameLabelEl.textContent = "Frame: -";
        simLabelEl.textContent = "Sim: -";
        return;
      }}
      if (prevPath) {{
        const found = frames.findIndex(f => preferredSvg(f) === prevPath);
        idx = found >= 0 ? found : Math.min(idx, frames.length - 1);
      }} else {{
        idx = Math.min(idx, frames.length - 1);
      }}
      renderFrame(idx);
    }}

    async function loadManifest() {{
      if (window.location.protocol === "file:") {{
        applyManifest(embeddedManifest || {{}});
        return;
      }}
      const res = await fetch(manifestPath + "?t=" + Date.now());
      const data = await res.json();
      applyManifest(data || {{}});
    }}

    function renderFrame(newIdx) {{
      if (!frames.length) return;
      idx = ((newIdx % frames.length) + frames.length) % frames.length;
      const f = frames[idx];
      frameEl.src = preferredSvg(f);
      frameLabelEl.textContent = `Frame: ${{idx + 1}} / ${{frames.length}}`;
      simLabelEl.textContent = `Sim: ${{f.sim_time ?? "-"}} | EV: ${{f.ev_id ?? "-"}} | View: ${{viewModeEl.value}}`;
      seekEl.value = String(idx);
      metrics.currentCost.textContent = fmtSec(f.current_cost_sec);
      metrics.optimizedCost.textContent = fmtSec(f.optimized_cost_sec);
      metrics.improve.textContent = fmtSec(f.improvement_sec);
      metrics.currentEdges.textContent = String(f.current_edge_count ?? "-");
      metrics.altEdges.textContent = String(f.optimized_edge_count ?? "-");
      metrics.overlap.textContent = String(f.overlap_edge_count ?? "-");
    }}

    function schedule() {{
      if (timer) clearInterval(timer);
      const ms = Number(speedEl.value || {interval_ms});
      if (!playing) return;
      timer = setInterval(() => {{
        if (!frames.length) return;
        renderFrame(idx + 1);
      }}, ms);
    }}

    playBtn.addEventListener("click", () => {{
      playing = !playing;
      playBtn.textContent = playing ? "Pause" : "Play";
      schedule();
    }});
    seekEl.addEventListener("input", () => renderFrame(Number(seekEl.value || 0)));
    speedEl.addEventListener("input", schedule);
    viewModeEl.addEventListener("change", () => renderFrame(idx));

    loadManifest().then(schedule);
    if (window.location.protocol === "file:") {{
      setInterval(() => window.location.reload(), {refresh_ms});
    }} else {{
      setInterval(loadManifest, {refresh_ms});
    }}
  </script>
</body>
</html>
"""


@dataclass
class GeoEdge:
    edge_id: str
    from_node: str
    to_node: str
    points: List[Tuple[float, float]]


@dataclass
class GeoNetwork:
    nodes: Dict[str, Tuple[float, float]]
    edges: Dict[str, GeoEdge]
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass
class DotSnapshot:
    sim_time: Optional[float]
    ev_id: str
    current_edges: List[str]
    optimized_edges: List[str]
    overlap_edges: List[str]
    current_cost_sec: float
    optimized_cost_sec: float
    improvement_sec: float
    start_node: Optional[str]
    destination_node: Optional[str]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Animate GTCO graph DOT snapshots")
    ap.add_argument("--dot-dir", required=True, help="directory containing GTCO .dot snapshots")
    ap.add_argument("--output-dir", default="", help="directory for rendered SVGs and HTML viewer")
    ap.add_argument("--network-file", default="madrid_short_area.net.xml", help="SUMO net.xml used to render geographic topology frames")
    ap.add_argument("--render-mode", choices=["graphviz", "geo", "both"], default="both")
    ap.add_argument("--interval-ms", type=int, default=900, help="default playback interval in milliseconds")
    ap.add_argument("--refresh-sec", type=float, default=2.0, help="manifest refresh interval in seconds")
    ap.add_argument("--watch", action="store_true", help="keep rebuilding while new DOT files arrive")
    ap.add_argument("--poll-sec", type=float, default=2.0, help="watch polling interval in seconds")
    ap.add_argument("--verbose", action="store_true")
    return ap.parse_args()


def require_dot() -> str:
    try:
        proc = subprocess.run(["dot", "-V"], capture_output=True, text=True)
    except FileNotFoundError:
        print("Missing Graphviz 'dot' executable", file=sys.stderr)
        sys.exit(2)
    if proc.returncode not in (0, 1):
        print("Graphviz 'dot' is not usable", file=sys.stderr)
        sys.exit(2)
    return "dot"


def _safe_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _parse_shape(shape: str) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for part in str(shape or "").split():
        try:
            xs, ys = part.split(",", 1)
            pts.append((float(xs), float(ys)))
        except Exception:
            continue
    return pts


def load_geo_network(network_file: Path) -> GeoNetwork:
    root = ET.parse(network_file).getroot()
    nodes: Dict[str, Tuple[float, float]] = {}
    edges: Dict[str, GeoEdge] = {}
    min_x = math.inf
    min_y = math.inf
    max_x = -math.inf
    max_y = -math.inf

    for junction in root.findall("junction"):
        jid = str(junction.get("id", "") or "")
        if not jid:
            continue
        x = _safe_float(junction.get("x", 0.0))
        y = _safe_float(junction.get("y", 0.0))
        nodes[jid] = (x, y)
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)

    for edge in root.findall("edge"):
        edge_id = str(edge.get("id", "") or "")
        if not edge_id or edge_id.startswith(":"):
            continue
        if str(edge.get("function", "") or "") == "internal":
            continue
        from_node = str(edge.get("from", "") or "")
        to_node = str(edge.get("to", "") or "")
        if not from_node or not to_node:
            continue
        points: List[Tuple[float, float]] = []
        lanes = list(edge.findall("lane"))
        for lane in lanes:
            shape = _parse_shape(str(lane.get("shape", "") or ""))
            if shape:
                points = shape
                break
        if not points:
            p0 = nodes.get(from_node)
            p1 = nodes.get(to_node)
            if p0 and p1:
                points = [p0, p1]
        if len(points) < 2:
            continue
        for x, y in points:
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
        edges[edge_id] = GeoEdge(edge_id=edge_id, from_node=from_node, to_node=to_node, points=points)

    if not edges:
        raise RuntimeError(f"No geographic edges found in {network_file}")
    return GeoNetwork(nodes=nodes, edges=edges, min_x=min_x, min_y=min_y, max_x=max_x, max_y=max_y)


def _parse_frame_meta(dot_path: Path) -> Dict[str, object]:
    stem = dot_path.stem
    parts = stem.split("_")
    sim_time = None
    ev_id = ""
    if len(parts) >= 3 and parts[-1].startswith("t"):
        raw = parts[-1][1:]
        try:
            sim_time = float(int(raw)) / 10.0
        except Exception:
            sim_time = None
        ev_id = parts[-2]
    return {"sim_time": sim_time, "ev_id": ev_id}


def parse_dot_snapshot(dot_path: Path) -> DotSnapshot:
    meta = _parse_frame_meta(dot_path)
    current_edges: List[str] = []
    optimized_edges: List[str] = []
    overlap_edges: List[str] = []
    current_cost = 0.0
    optimized_cost = 0.0
    start_node: Optional[str] = None
    destination_node: Optional[str] = None

    for raw_line in dot_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if "->" in line and "label=\"" in line and line.startswith('"'):
            try:
                left, right = line.split("->", 1)
                from_node = left.strip().strip('"')
                to_node = right.split("[", 1)[0].strip().strip('"')
                attrs = right.split("[", 1)[1].rsplit("]", 1)[0]
            except Exception:
                continue
            attr_map: Dict[str, str] = {}
            for chunk in attrs.split(","):
                if "=" not in chunk:
                    continue
                k, v = chunk.split("=", 1)
                attr_map[k.strip()] = v.strip().strip('"')
            label = attr_map.get("label", "")
            edge_id = label.split("\\n", 1)[0] if label else f"{from_node}->{to_node}"
            cost_str = label.split("\\n", 1)[1] if "\\n" in label else "0"
            cost = _safe_float(str(cost_str).rstrip("s"), 0.0)
            color = attr_map.get("color", "")
            if color == "goldenrod3":
                overlap_edges.append(edge_id)
                current_edges.append(edge_id)
                optimized_edges.append(edge_id)
                current_cost += cost
                optimized_cost += cost
            elif color == "dodgerblue3":
                current_edges.append(edge_id)
                current_cost += cost
            elif color == "firebrick3":
                optimized_edges.append(edge_id)
                optimized_cost += cost
            continue
        if line.startswith('"') and "[" in line and "->" not in line:
            try:
                node = line.split("[", 1)[0].strip().strip('"')
                attrs = line.split("[", 1)[1].rsplit("]", 1)[0]
            except Exception:
                continue
            if 'fillcolor="lightgreen"' in attrs:
                start_node = node
            elif 'fillcolor="lightyellow"' in attrs:
                destination_node = node

    return DotSnapshot(
        sim_time=meta.get("sim_time"),
        ev_id=str(meta.get("ev_id") or ""),
        current_edges=current_edges,
        optimized_edges=optimized_edges,
        overlap_edges=overlap_edges,
        current_cost_sec=float(current_cost),
        optimized_cost_sec=float(optimized_cost),
        improvement_sec=max(0.0, float(current_cost) - float(optimized_cost)),
        start_node=start_node,
        destination_node=destination_node,
    )


def _transform_fn(net: GeoNetwork, width: int = 1400, height: int = 900, margin: int = 36):
    span_x = max(1.0, net.max_x - net.min_x)
    span_y = max(1.0, net.max_y - net.min_y)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)
    draw_w = span_x * scale
    draw_h = span_y * scale
    offset_x = margin + max(0.0, (width - 2 * margin - draw_w) / 2.0)
    offset_y = margin + max(0.0, (height - 2 * margin - draw_h) / 2.0)

    def tx(pt: Tuple[float, float]) -> Tuple[float, float]:
        x, y = pt
        px = offset_x + (x - net.min_x) * scale
        py = offset_y + (net.max_y - y) * scale
        return px, py

    return tx


def _polyline_str(points: List[Tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def render_geo_svg(dot_path: Path, svg_path: Path, net: GeoNetwork, verbose: bool = False) -> Dict[str, object]:
    snap = parse_dot_snapshot(dot_path)
    tx = _transform_fn(net)
    width = 1400
    height = 900
    bg_lines: List[str] = []
    for edge in net.edges.values():
        pts = [tx(pt) for pt in edge.points]
        bg_lines.append(
            f'<polyline points="{_polyline_str(pts)}" fill="none" stroke="#c9c5bd" stroke-width="1.1" stroke-opacity="0.38" stroke-linecap="round" stroke-linejoin="round" />'
        )

    def draw_edge(edge_ids: List[str], stroke: str, width_px: float, opacity: float = 0.96, dash: str = "") -> List[str]:
        lines: List[str] = []
        seen = set()
        for edge_id in edge_ids:
            if edge_id in seen:
                continue
            seen.add(edge_id)
            edge = net.edges.get(edge_id)
            if edge is None:
                continue
            pts = [tx(pt) for pt in edge.points]
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            lines.append(
                f'<polyline points="{_polyline_str(pts)}" fill="none" stroke="{stroke}" stroke-width="{width_px:.1f}" stroke-opacity="{opacity:.2f}" stroke-linecap="round" stroke-linejoin="round"{dash_attr} />'
            )
        return lines

    overlap_set = set(snap.overlap_edges)
    current_only = [eid for eid in snap.current_edges if eid not in overlap_set]
    optimized_only = [eid for eid in snap.optimized_edges if eid not in overlap_set]

    fg_lines: List[str] = []
    fg_lines.extend(draw_edge(snap.overlap_edges, "#c88d00", 5.4))
    fg_lines.extend(draw_edge(current_only, "#1668c7", 4.2))
    fg_lines.extend(draw_edge(optimized_only, "#b32929", 4.0, dash="10 8"))

    marker_lines: List[str] = []
    for node_id, fill, radius, stroke in [
        (snap.start_node, "#b6db95", 9.0, "#355e1f"),
        (snap.destination_node, "#f7efad", 10.0, "#7b6c09"),
    ]:
        if not node_id:
            continue
        pt = net.nodes.get(node_id)
        if pt is None:
            continue
        x, y = tx(pt)
        marker_lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="2" />')
        marker_lines.append(f'<text x="{x + 12:.2f}" y="{y - 10:.2f}" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="14" fill="#22313a">{node_id}</text>')

    title = f"GTCO Geographic Snapshot ev={snap.ev_id or '-'} sim={snap.sim_time if snap.sim_time is not None else '-'}"
    summary = f"current={snap.current_cost_sec:.1f}s optimized={snap.optimized_cost_sec:.1f}s improve={snap.improvement_sec:.1f}s"
    svg = "\n".join([
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect x="0" y="0" width="100%" height="100%" fill="#f7f4ec" />',
        f'<text x="36" y="42" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="28" fill="#1d2a33">{title}</text>',
        f'<text x="36" y="68" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="16" fill="#5a6871">{summary}</text>',
        '<g id="network-background">',
        *bg_lines,
        '</g>',
        '<g id="corridor-overlay">',
        *fg_lines,
        '</g>',
        '<g id="markers">',
        *marker_lines,
        '</g>',
        '<g id="legend">',
        '<rect x="36" y="780" width="360" height="86" rx="16" fill="#fffaf0" stroke="#d9cfbf" />',
        '<line x1="60" y1="810" x2="110" y2="810" stroke="#c9c5bd" stroke-opacity="0.65" stroke-width="3" />',
        '<text x="126" y="815" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="14" fill="#5a6871">Madrid network background</text>',
        '<line x1="60" y1="833" x2="110" y2="833" stroke="#1668c7" stroke-width="4" />',
        '<text x="126" y="838" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="14" fill="#5a6871">Current EV route</text>',
        '<line x1="60" y1="856" x2="110" y2="856" stroke="#b32929" stroke-width="4" stroke-dasharray="10 8" />',
        '<text x="126" y="861" font-family="IBM Plex Sans, Helvetica, sans-serif" font-size="14" fill="#5a6871">Optimized alternative</text>',
        '</g>',
        '</svg>',
    ])
    svg_path.write_text(svg, encoding="utf-8")
    if verbose:
        print(f"[animate] rendered geographic {dot_path.name} -> {svg_path.name}")
    return {
        "geo_svg": svg_path.name,
        "sim_time": snap.sim_time,
        "ev_id": snap.ev_id,
        "current_cost_sec": snap.current_cost_sec,
        "optimized_cost_sec": snap.optimized_cost_sec,
        "improvement_sec": snap.improvement_sec,
        "current_edge_count": len(snap.current_edges),
        "optimized_edge_count": len(snap.optimized_edges),
        "overlap_edge_count": len(snap.overlap_edges),
    }


def render_frames(
    dot_cmd: str,
    dot_dir: Path,
    output_dir: Path,
    render_mode: str,
    net: Optional[GeoNetwork] = None,
    verbose: bool = False,
) -> List[Dict[str, object]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    frames: List[Dict[str, object]] = []
    for dot_path in sorted(dot_dir.glob("*.dot")):
        frame: Dict[str, object] = {"dot": dot_path.name}
        meta = _parse_frame_meta(dot_path)
        frame["sim_time"] = meta.get("sim_time")
        frame["ev_id"] = meta.get("ev_id")

        if render_mode in ("graphviz", "both"):
            svg_name = f"{dot_path.stem}.svg"
            svg_path = output_dir / svg_name
            needs_render = (not svg_path.exists()) or (dot_path.stat().st_mtime > svg_path.stat().st_mtime)
            if needs_render:
                cmd = [dot_cmd, "-Tsvg", str(dot_path), "-o", str(svg_path)]
                proc = subprocess.run(cmd, capture_output=True, text=True)
                if proc.returncode != 0:
                    print(f"[animate][WARN] dot failed for {dot_path.name}: {proc.stderr.strip()}", file=sys.stderr)
                elif verbose:
                    print(f"[animate] rendered {dot_path.name} -> {svg_name}")
            frame["graphviz_svg"] = svg_name

        if render_mode in ("geo", "both"):
            geo_name = f"{dot_path.stem}.geo.svg"
            geo_path = output_dir / geo_name
            source_geo_candidates = [
                dot_path.with_suffix(".geo.svg"),
                dot_path.parent / "geo_snapshots" / f"{dot_path.stem}.geo.svg",
            ]
            source_geo = next((p for p in source_geo_candidates if p.exists()), None)
            if source_geo is not None:
                needs_copy = (not geo_path.exists()) or (source_geo.stat().st_mtime > geo_path.stat().st_mtime)
                if needs_copy:
                    shutil.copy2(source_geo, geo_path)
                    if verbose:
                        print(f"[animate] copied geographic {source_geo.name} -> {geo_name}")
                snap = parse_dot_snapshot(dot_path)
                frame.update(
                    {
                        "geo_svg": geo_name,
                        "sim_time": snap.sim_time,
                        "ev_id": snap.ev_id,
                        "current_cost_sec": snap.current_cost_sec,
                        "optimized_cost_sec": snap.optimized_cost_sec,
                        "improvement_sec": snap.improvement_sec,
                        "current_edge_count": len(snap.current_edges),
                        "optimized_edge_count": len(snap.optimized_edges),
                        "overlap_edge_count": len(snap.overlap_edges),
                    }
                )
            elif net is not None:
                needs_geo = (not geo_path.exists()) or (dot_path.stat().st_mtime > geo_path.stat().st_mtime)
                if needs_geo:
                    geo_meta = render_geo_svg(dot_path, geo_path, net=net, verbose=verbose)
                else:
                    geo_meta = render_geo_svg(dot_path, geo_path, net=net, verbose=False)
                frame.update(geo_meta)

        if "geo_svg" in frame:
            frame["svg"] = frame["geo_svg"]
        elif "graphviz_svg" in frame:
            frame["svg"] = frame["graphviz_svg"]

        if "current_cost_sec" not in frame:
            snap = parse_dot_snapshot(dot_path)
            frame.update(
                {
                    "current_cost_sec": snap.current_cost_sec,
                    "optimized_cost_sec": snap.optimized_cost_sec,
                    "improvement_sec": snap.improvement_sec,
                    "current_edge_count": len(snap.current_edges),
                    "optimized_edge_count": len(snap.optimized_edges),
                    "overlap_edge_count": len(snap.overlap_edges),
                }
            )
        frames.append(frame)
    return frames


def write_manifest(output_dir: Path, frames: List[Dict[str, object]]) -> None:
    payload = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frames": frames,
    }
    with open(output_dir / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def write_html(output_dir: Path, interval_ms: int, refresh_sec: float, frames: List[Dict[str, object]]) -> None:
    manifest = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frames": frames,
    }
    html = HTML_TEMPLATE.format(
        interval_ms=int(interval_ms),
        refresh_sec=float(refresh_sec),
        refresh_ms=int(max(250, round(float(refresh_sec) * 1000.0))),
        manifest_json=json.dumps(manifest),
    )
    with open(output_dir / "index.html", "w", encoding="utf-8") as f:
        f.write(html)


def build_once(
    dot_cmd: str,
    dot_dir: Path,
    output_dir: Path,
    render_mode: str,
    net: Optional[GeoNetwork],
    interval_ms: int,
    refresh_sec: float,
    verbose: bool,
) -> int:
    frames = render_frames(dot_cmd, dot_dir, output_dir, render_mode=render_mode, net=net, verbose=verbose)
    write_manifest(output_dir, frames)
    write_html(output_dir, interval_ms=interval_ms, refresh_sec=refresh_sec, frames=frames)
    if verbose:
        print(f"[animate] frames={len(frames)} output={output_dir} mode={render_mode}")
    return len(frames)


def main() -> None:
    args = parse_args()
    dot_cmd = require_dot()
    dot_dir = Path(args.dot_dir).expanduser().resolve()
    output_dir = Path(args.output_dir or (dot_dir / "animation")).expanduser().resolve()
    if not dot_dir.exists():
        print(f"DOT directory does not exist: {dot_dir}", file=sys.stderr)
        sys.exit(2)

    net: Optional[GeoNetwork] = None
    if args.render_mode in ("geo", "both"):
        if args.network_file:
            net = load_geo_network(Path(args.network_file).expanduser().resolve())
        else:
            dot_root = Path(args.dot_dir).expanduser().resolve()
            has_geo = any(dot_root.glob("*.geo.svg")) or any((dot_root / "geo_snapshots").glob("*.geo.svg"))
            if not has_geo:
                print("--network-file is required when render-mode is geo or both and no pre-rendered .geo.svg frames exist", file=sys.stderr)
                sys.exit(2)

    try:
        while True:
            build_once(
                dot_cmd=dot_cmd,
                dot_dir=dot_dir,
                output_dir=output_dir,
                render_mode=str(args.render_mode),
                net=net,
                interval_ms=int(args.interval_ms),
                refresh_sec=float(args.refresh_sec),
                verbose=bool(args.verbose),
            )
            if not bool(args.watch):
                break
            time.sleep(max(0.2, float(args.poll_sec)))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
