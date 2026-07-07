#!/usr/bin/env python3
"""Plot SUMO network topology and scenario demand density panels.

This produces paper-style panels like: base network + light/medium/heavy
scenario overlays. Density is computed from route XML demand by default:
for each vehicle, every edge in its route contributes one traversal count.

If edgeData XML is supplied instead of route XML, edge metric values can be
plotted directly with --source edge-data and --edge-metric.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable


def require_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        from matplotlib.colors import LogNorm, Normalize
        from matplotlib.cm import ScalarMappable
    except Exception as exc:  # pragma: no cover - depends on local env
        raise SystemExit(
            "matplotlib is required for this plotting script. Install/use it on "
            "the server environment where you already generate paper plots."
        ) from exc
    return plt, LineCollection, LogNorm, Normalize, ScalarMappable


def parse_xy_pairs(shape: str) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for token in str(shape or "").split():
        if "," not in token:
            continue
        x, y = token.split(",", 1)
        try:
            pts.append((float(x), float(y)))
        except ValueError:
            continue
    return pts


def parse_net(net_file: Path) -> tuple[dict[str, list[tuple[float, float]]], dict[str, tuple[float, float]], dict[str, str]]:
    edge_shapes: dict[str, list[tuple[float, float]]] = {}
    junction_xy: dict[str, tuple[float, float]] = {}
    edge_type: dict[str, str] = {}
    root = ET.parse(net_file).getroot()
    for elem in root.iter("junction"):
        jid = elem.get("id", "")
        if jid and not jid.startswith(":"):
            try:
                junction_xy[jid] = (float(elem.get("x", "nan")), float(elem.get("y", "nan")))
            except ValueError:
                pass
    for elem in root.iter("edge"):
        eid = elem.get("id", "")
        if not eid or eid.startswith(":") or elem.get("function") == "internal":
            continue
        edge_type[eid] = elem.get("type", "") or ""
        lane_shapes = []
        for lane in elem.findall("lane"):
            pts = parse_xy_pairs(lane.get("shape", ""))
            if pts:
                lane_shapes.append(pts)
        if lane_shapes:
            # Use the first lane as the edge geometry. For the paper-scale map
            # this is visually cleaner than drawing every lane.
            edge_shapes[eid] = lane_shapes[0]
    return edge_shapes, junction_xy, edge_type


def route_edges_from_vehicle(elem: ET.Element, route_defs: dict[str, list[str]]) -> list[str]:
    route_attr = elem.get("route", "")
    if route_attr and route_attr in route_defs:
        return route_defs[route_attr]
    route_child = elem.find("route")
    if route_child is not None:
        return str(route_child.get("edges", "") or "").split()
    return []


def parse_route_density(route_file: Path, *, ev_prefix: str = "emergency") -> tuple[Counter, dict[str, list[str]]]:
    route_defs: dict[str, list[str]] = {}
    counts: Counter = Counter()
    ev_routes: dict[str, list[str]] = {}
    root = ET.parse(route_file).getroot()
    for elem in root.iter("route"):
        if elem.get("id"):
            route_defs[str(elem.get("id"))] = str(elem.get("edges", "") or "").split()
    for elem in root:
        tag = elem.tag.rsplit("}", 1)[-1]
        if tag not in {"vehicle", "trip"}:
            continue
        vid = str(elem.get("id", "") or "")
        edges = route_edges_from_vehicle(elem, route_defs)
        for edge in edges:
            if edge and not edge.startswith(":"):
                counts[edge] += 1
        if vid.startswith(ev_prefix) and edges:
            ev_routes[vid] = edges
    return counts, ev_routes


def parse_edge_data(edge_data_file: Path, metric: str) -> Counter:
    values: dict[str, list[float]] = defaultdict(list)
    for _event, elem in ET.iterparse(edge_data_file, events=("end",)):
        tag = elem.tag.rsplit("}", 1)[-1]
        if tag == "edge":
            eid = elem.get("id", "")
            if eid and not eid.startswith(":"):
                raw = elem.get(metric)
                if raw is not None:
                    try:
                        values[eid].append(float(raw))
                    except ValueError:
                        pass
        elem.clear()
    out = Counter()
    for eid, arr in values.items():
        if arr:
            out[eid] = sum(arr) / len(arr)
    return out


def parse_scenario_spec(spec: str) -> tuple[str, Path]:
    if "=" not in spec:
        raise argparse.ArgumentTypeError("Use --scenario Label=/path/to/file.rou.xml")
    label, path = spec.split("=", 1)
    return label.strip(), Path(os.path.expandvars(os.path.expanduser(path.strip())))


def parse_label_overrides(specs: Iterable[str] | None) -> dict[str, str]:
    out: dict[str, str] = {}
    for spec in specs or []:
        if "=" not in spec:
            raise argparse.ArgumentTypeError("Use --panel-label old=new")
        old, new = spec.split("=", 1)
        out[old.strip()] = new.strip()
    return out


def parse_marker_spec(spec: str) -> tuple[str, str]:
    """Parse marker specs like Node423 or Node423=Scout gap A."""
    if "=" in spec:
        node, label = spec.split("=", 1)
        return node.strip(), label.strip()
    node = spec.strip()
    return node, node


def parse_marker_label_offset(spec: str) -> tuple[str, tuple[float, float]]:
    """Parse per-node label offsets like Node417=45,0."""
    if "=" not in spec:
        raise argparse.ArgumentTypeError("Use --scout-marker-label-offset NodeId=dx,dy")
    node, raw = spec.split("=", 1)
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Use --scout-marker-label-offset NodeId=dx,dy")
    try:
        return node.strip(), (float(parts[0]), float(parts[1]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Scout marker label offsets must be numeric dx,dy values") from exc


def parse_grid_spec(spec: str) -> tuple[int, int]:
    """Parse grid specs like 3x2 into columns, rows."""
    raw = str(spec or "").strip().lower().replace(",", "x")
    if "x" not in raw:
        raise argparse.ArgumentTypeError("Use --region-grid COLSxROWS, e.g. 3x2")
    left, right = raw.split("x", 1)
    try:
        cols = int(left)
        rows = int(right)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Use integer grid dimensions, e.g. 3x2") from exc
    if cols <= 0 or rows <= 0:
        raise argparse.ArgumentTypeError("Grid dimensions must be positive.")
    return cols, rows


def parse_grid_bounds(spec: str) -> tuple[float, float, float, float]:
    """Parse x_min,x_max,y_min,y_max."""
    parts = [x.strip() for x in str(spec or "").split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("Use --region-grid-bounds XMIN,XMAX,YMIN,YMAX")
    try:
        xmin, xmax, ymin, ymax = [float(x) for x in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Grid bounds must be numeric: XMIN,XMAX,YMIN,YMAX") from exc
    if xmax <= xmin or ymax <= ymin:
        raise argparse.ArgumentTypeError("Grid bounds must satisfy XMAX>XMIN and YMAX>YMIN.")
    return xmin, xmax, ymin, ymax


def load_manifest_scenarios(
    manifest: Path,
    labels: Iterable[str] | None = None,
    route_filter: Iterable[str] | None = None,
) -> list[tuple[str, Path]]:
    wanted = {x.strip() for x in labels or [] if x.strip()}
    wanted_routes = {str(x).strip() for x in route_filter or [] if str(x).strip()}
    out: list[tuple[str, Path]] = []
    with manifest.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            density_label = str(row.get("density_label") or row.get("scenario_label") or "").strip()
            density_count = str(row.get("density_count") or "").strip()
            route_id = str(row.get("route_id") or row.get("route_idx") or "").strip()
            if wanted and density_label not in wanted and density_count not in wanted:
                continue
            if wanted_routes and route_id not in wanted_routes:
                continue
            route_file = row.get("route_file") or row.get("routes_file") or row.get("scenario_routes") or row.get("rou_file")
            if not route_file:
                continue
            label = density_label or density_count or Path(route_file).stem
            if route_id:
                label = f"{label} R{route_id}"
            out.append((label, Path(os.path.expandvars(os.path.expanduser(route_file)))))
    return out


def route_id_sort_key(label: str) -> tuple[int, str]:
    m = re.search(r"(\d+(?:\.\d+)?)\s*[Kk]?", label)
    if not m:
        return (999999, label)
    value = float(m.group(1))
    if "k" in label.lower():
        value *= 1000.0
    return (int(value), label)


def pick_ev_route(
    ev_routes: dict[str, list[str]],
    ev_id: str | None,
    *,
    strict: bool = False,
) -> tuple[list[str], str]:
    if not ev_routes:
        if strict and ev_id:
            raise SystemExit(f"Requested --ev-id {ev_id!r}, but no EV routes were found in the selected route file.")
        return [], ""
    if ev_id and ev_id in ev_routes:
        return ev_routes[ev_id], ev_id
    if ev_id:
        # Accept route number convenience, e.g. --ev-id 5 -> emergency5.
        candidate = f"emergency{ev_id}"
        if candidate in ev_routes:
            return ev_routes[candidate], candidate
        available = ", ".join(sorted(ev_routes)[:12])
        msg = f"Requested --ev-id {ev_id!r} was not found. Available EV ids include: {available}"
        if strict:
            raise SystemExit(msg)
        print(f"warning: {msg}; falling back to first available EV route.", file=os.sys.stderr)
    fallback = sorted(ev_routes)[0]
    return ev_routes[fallback], fallback


def build_segments(edge_shapes: dict[str, list[tuple[float, float]]], edge_ids: Iterable[str]) -> tuple[list[list[tuple[float, float]]], list[str]]:
    segs: list[list[tuple[float, float]]] = []
    ids: list[str] = []
    for eid in edge_ids:
        pts = edge_shapes.get(eid)
        if pts and len(pts) >= 2:
            segs.append(pts)
            ids.append(eid)
    return segs, ids


def bounds_for_edges(
    edge_shapes: dict[str, list[tuple[float, float]]],
    edge_ids: Iterable[str],
) -> tuple[float, float, float, float] | None:
    xs: list[float] = []
    ys: list[float] = []
    for eid in edge_ids:
        for x, y in edge_shapes.get(eid, []):
            xs.append(x)
            ys.append(y)
    if not xs or not ys:
        return None
    return min(xs), max(xs), min(ys), max(ys)


def plot(args: argparse.Namespace) -> None:
    plt, LineCollection, LogNorm, Normalize, ScalarMappable = require_matplotlib()
    net_file = Path(args.net_file)
    edge_shapes, junction_xy, _edge_type = parse_net(net_file)
    if not edge_shapes:
        raise SystemExit(f"No drawable edges found in {net_file}")

    scenarios: list[tuple[str, Path]] = []
    for spec in args.scenario or []:
        scenarios.append(parse_scenario_spec(spec))
    if args.manifest_csv:
        scenarios.extend(load_manifest_scenarios(
            Path(args.manifest_csv),
            args.manifest_density_filter or [],
            args.manifest_route_filter or [],
        ))
    if not scenarios:
        raise SystemExit("Provide at least one --scenario Label=/path/file.rou.xml or --manifest-csv.")
    scenarios = sorted(scenarios, key=lambda x: route_id_sort_key(x[0]))
    if args.max_panels:
        scenarios = scenarios[: args.max_panels]
    label_overrides = parse_label_overrides(args.panel_label or [])
    scenarios = [(label_overrides.get(label, label), path) for label, path in scenarios]

    base_segments, _ = build_segments(edge_shapes, edge_shapes.keys())
    scenario_values: list[tuple[str, Path, Counter, list[str]]] = []
    global_values: list[float] = []
    ev_route_edges: list[str] = []
    selected_ev_id = ""
    for label, file_path in scenarios:
        if args.source == "routes":
            counts, ev_routes = parse_route_density(file_path, ev_prefix=args.ev_prefix)
            if not ev_route_edges:
                ev_route_edges, selected_ev_id = pick_ev_route(ev_routes, args.ev_id, strict=args.strict_ev_id)
        else:
            counts = parse_edge_data(file_path, args.edge_metric)
            ev_routes = {}
        if args.debug_selection:
            print(
                "selected_panel "
                f"label={label!r} route_file={str(file_path)!r} "
                f"ev_requested={args.ev_id!r} ev_selected={selected_ev_id!r} "
                f"available_evs={','.join(sorted(ev_routes)[:20]) if ev_routes else ''}"
            )
        values = [float(v) for eid, v in counts.items() if eid in edge_shapes and float(v) > 0]
        global_values.extend(values)
        scenario_values.append((label, file_path, counts, ev_route_edges))

    if not global_values:
        raise SystemExit("No positive edge values found for supplied scenario files.")
    positive_min = max(args.log_min, min(v for v in global_values if v > 0))
    vmax = max(global_values)
    norm = LogNorm(vmin=positive_min, vmax=vmax) if args.log_scale else Normalize(vmin=0, vmax=vmax)
    cmap = plt.get_cmap(args.cmap)

    n = len(scenario_values)
    include_base = not args.no_base_panel
    cols = n + (1 if include_base else 0)
    fig_w = float(args.fig_width) if float(args.fig_width) > 0 else max(7.5, float(args.panel_width) * cols + 0.55)
    fig_h = float(args.fig_height) if float(args.fig_height) > 0 else float(args.panel_height)
    fig, axes = plt.subplots(1, cols, figsize=(fig_w, fig_h), constrained_layout=True)
    if cols == 1:
        axes = [axes]

    density_edge_ids = {
        eid
        for _label, _file_path, values, _ev_edges in scenario_values
        for eid in values
        if values.get(eid, 0) > 0 and eid in edge_shapes
    }
    ev_edge_ids = {eid for eid in ev_route_edges if eid in edge_shapes}
    if args.map_fit_bounds == "density":
        fit_bounds = bounds_for_edges(edge_shapes, density_edge_ids)
    elif args.map_fit_bounds == "ev-route":
        fit_bounds = bounds_for_edges(edge_shapes, ev_edge_ids)
    elif args.map_fit_bounds == "density-ev-route":
        fit_bounds = bounds_for_edges(edge_shapes, density_edge_ids | ev_edge_ids)
    else:
        fit_bounds = bounds_for_edges(edge_shapes, edge_shapes.keys())
    if fit_bounds is None:
        fit_bounds = bounds_for_edges(edge_shapes, edge_shapes.keys())
    if fit_bounds is None:
        raise SystemExit("No drawable map bounds found.")
    xmin, xmax, ymin, ymax_xy = fit_bounds
    span_x = xmax - xmin
    span_y = ymax_xy - ymin
    pad_x = span_x * args.map_x_pad_ratio
    pad_y = span_y * args.map_y_pad_ratio
    xlim = (xmin - pad_x, xmax + pad_x)
    ylim = (ymin - pad_y, ymax_xy + pad_y)
    if args.square_map_limits:
        # Preserve equal map scale but expand the shorter axis so the rendered
        # panel can become square/taller instead of being constrained by the
        # raw network bounding box.
        cx = (xlim[0] + xlim[1]) / 2.0
        cy = (ylim[0] + ylim[1]) / 2.0
        half = max((xlim[1] - xlim[0]), (ylim[1] - ylim[0])) / 2.0
        xlim = (cx - half, cx + half)
        ylim = (cy - half, cy + half)
    if args.map_zoom and args.map_zoom > 0 and not math.isclose(args.map_zoom, 1.0):
        # Values >1 zoom in around the current center. Values <1 zoom out.
        cx = (xlim[0] + xlim[1]) / 2.0
        cy = (ylim[0] + ylim[1]) / 2.0
        half_x = (xlim[1] - xlim[0]) / (2.0 * args.map_zoom)
        half_y = (ylim[1] - ylim[0]) / (2.0 * args.map_zoom)
        xlim = (cx - half_x, cx + half_x)
        ylim = (cy - half_y, cy + half_y)
    crop_left = (xlim[1] - xlim[0]) * max(0.0, args.crop_left_ratio)
    crop_right = (xlim[1] - xlim[0]) * max(0.0, args.crop_right_ratio)
    crop_bottom = (ylim[1] - ylim[0]) * max(0.0, args.crop_bottom_ratio)
    crop_top = (ylim[1] - ylim[0]) * max(0.0, args.crop_top_ratio)
    if crop_left + crop_right < (xlim[1] - xlim[0]):
        xlim = (xlim[0] + crop_left, xlim[1] - crop_right)
    if crop_bottom + crop_top < (ylim[1] - ylim[0]):
        ylim = (ylim[0] + crop_bottom, ylim[1] - crop_top)
    if args.map_xmin is not None:
        xlim = (args.map_xmin, xlim[1])
    if args.map_xmax is not None:
        xlim = (xlim[0], args.map_xmax)
    if args.map_ymin is not None:
        ylim = (args.map_ymin, ylim[1])
    if args.map_ymax is not None:
        ylim = (ylim[0], args.map_ymax)

    def style_axis(ax):
        ax.set_aspect(args.map_aspect, adjustable="box")
        if args.map_box_aspect > 0:
            ax.set_box_aspect(args.map_box_aspect)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        tick_fontsize = args.axis_tick_fontsize if args.axis_tick_fontsize is not None else args.tick_fontsize
        ax.tick_params(labelsize=tick_fontsize)
        if args.axis_units:
            x_label_fontsize = args.x_axis_label_fontsize if args.x_axis_label_fontsize is not None else args.axis_label_fontsize
            y_label_fontsize = args.y_axis_label_fontsize if args.y_axis_label_fontsize is not None else args.axis_label_fontsize
            ax.set_xlabel("[m]", fontsize=x_label_fontsize)
            ax.set_ylabel("[m]", fontsize=y_label_fontsize)
        if args.hide_ticks:
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_xlabel("")
            ax.set_ylabel("")

    def add_panel_title(ax, title: str, *, is_base: bool = False) -> None:
        if is_base:
            title_fontsize = args.base_title_fontsize if args.base_title_fontsize is not None else args.title_fontsize
            title_pad = args.base_title_pad if args.base_title_pad is not None else args.title_pad
            title_inside_y = args.base_title_inside_y if args.base_title_inside_y is not None else args.title_inside_y
        else:
            title_fontsize = args.panel_title_fontsize if args.panel_title_fontsize is not None else args.title_fontsize
            title_pad = args.panel_title_pad if args.panel_title_pad is not None else args.title_pad
            title_inside_y = args.panel_title_inside_y if args.panel_title_inside_y is not None else args.title_inside_y
        if args.title_inside:
            ax.text(
                0.5,
                title_inside_y,
                title,
                transform=ax.transAxes,
                ha="center",
                va="top",
                fontsize=title_fontsize,
                fontweight="bold",
                bbox=dict(facecolor="white", alpha=0.72, edgecolor="none", pad=1.5),
                zorder=10,
            )
        else:
            ax.set_title(title, fontsize=title_fontsize, fontweight="bold", pad=title_pad)

    scenario_markers = [parse_marker_spec(x) for x in (args.scout_marker_node or [])]
    scout_marker_label_offsets = dict(args.scout_marker_label_offset or [])
    region_grid = parse_grid_spec(args.region_grid) if args.region_grid else None
    region_grid_bounds = parse_grid_bounds(args.region_grid_bounds) if args.region_grid_bounds else None

    def draw_scout_markers(ax) -> None:
        marker = "*" if args.scout_marker_style == "star" else "o"
        for node_id, label in scenario_markers:
            if node_id not in junction_xy:
                print(f"warning: scout marker node {node_id!r} was not found in network.", file=os.sys.stderr)
                continue
            x, y = junction_xy[node_id]
            ax.scatter(
                [x],
                [y],
                s=args.scout_marker_size,
                marker=marker,
                c=args.scout_marker_color,
                edgecolors=args.scout_marker_edge_color,
                linewidths=args.scout_marker_linewidth,
                zorder=8,
            )
            if args.scout_marker_label:
                label_dx, label_dy = scout_marker_label_offsets.get(
                    node_id,
                    (args.scout_marker_label_dx, args.scout_marker_label_dy),
                )
                ax.text(
                    x + label_dx,
                    y + label_dy,
                    label,
                    fontsize=args.scout_marker_label_fontsize,
                    color=args.scout_marker_label_color,
                    fontweight="bold",
                    zorder=9,
                    bbox=dict(facecolor="white", alpha=0.68, edgecolor="none", pad=1.0),
                )

    def draw_region_grid(ax) -> None:
        if not region_grid:
            return
        cols, rows = region_grid
        gxmin, gxmax, gymin, gymax = region_grid_bounds or (xlim[0], xlim[1], ylim[0], ylim[1])
        dx = (gxmax - gxmin) / cols
        dy = (gymax - gymin) / rows
        for idx in range(cols + 1):
            x = gxmin + idx * dx
            ax.plot(
                [x, x],
                [gymin, gymax],
                color=args.region_grid_color,
                linewidth=args.region_grid_linewidth,
                alpha=args.region_grid_alpha,
                linestyle=args.region_grid_linestyle,
                zorder=args.region_grid_zorder,
            )
        for idx in range(rows + 1):
            y = gymin + idx * dy
            ax.plot(
                [gxmin, gxmax],
                [y, y],
                color=args.region_grid_color,
                linewidth=args.region_grid_linewidth,
                alpha=args.region_grid_alpha,
                linestyle=args.region_grid_linestyle,
                zorder=args.region_grid_zorder,
            )
        if not args.region_grid_label:
            return
        # Number regions left-to-right, top-to-bottom: 1..cols for top row.
        for row in range(rows):
            for col in range(cols):
                region_id = row * cols + col + 1
                x = gxmin + (col + args.region_grid_label_x_frac) * dx
                y = gymax - (row + args.region_grid_label_y_frac) * dy
                ax.text(
                    x,
                    y,
                    f"{args.region_grid_label_prefix}{region_id}",
                    fontsize=args.region_grid_label_fontsize,
                    color=args.region_grid_label_color,
                    fontweight=args.region_grid_label_weight,
                    ha=args.region_grid_label_ha,
                    va=args.region_grid_label_va,
                    zorder=args.region_grid_zorder + 1,
                    bbox=dict(
                        facecolor=args.region_grid_label_box_color,
                        alpha=args.region_grid_label_box_alpha,
                        edgecolor="none",
                        pad=1.0,
                    ) if args.region_grid_label_box else None,
                )

    ax_idx = 0
    if include_base:
        ax = axes[0]
        ax.add_collection(LineCollection(base_segments, colors=args.base_color, linewidths=args.base_linewidth, alpha=0.82))
        if args.marker_node and args.marker_node in junction_xy:
            x, y = junction_xy[args.marker_node]
            ax.scatter([x], [y], s=120, c="red", edgecolors="black", linewidths=1.0, zorder=5)
        if args.scout_marker_on_base:
            draw_scout_markers(ax)
        if args.region_grid_on_base:
            draw_region_grid(ax)
        add_panel_title(ax, args.base_title, is_base=True)
        style_axis(ax)
        ax_idx = 1

    for panel_i, (label, file_path, values, ev_edges) in enumerate(scenario_values):
        ax = axes[ax_idx + panel_i]
        ax.add_collection(LineCollection(base_segments, colors=args.background_color, linewidths=args.background_linewidth, alpha=0.45))
        colored_edges = [eid for eid in edge_shapes if values.get(eid, 0) > 0]
        colored_segments, colored_ids = build_segments(edge_shapes, colored_edges)
        edge_values = [float(values[eid]) for eid in colored_ids]
        if colored_segments:
            lc = LineCollection(colored_segments, cmap=cmap, norm=norm, linewidths=args.density_linewidth, alpha=0.95)
            lc.set_array(edge_values)
            ax.add_collection(lc)
        if ev_edges and args.show_ev_route:
            ev_segments, _ = build_segments(edge_shapes, ev_edges)
            if ev_segments:
                ax.add_collection(LineCollection(ev_segments, colors=args.ev_route_color, linewidths=args.ev_route_linewidth, alpha=0.95, zorder=4))
        draw_region_grid(ax)
        draw_scout_markers(ax)
        if args.panel_title:
            title = args.panel_title
            if len(scenario_values) > 1:
                title = f"{title} {panel_i + 1}"
        else:
            title = args.panel_title_prefix + label if args.panel_title_prefix else label
        add_panel_title(ax, title)
        style_axis(ax)

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    colorbar_shrink = args.colorbar_height_ratio if args.colorbar_height_ratio is not None else args.colorbar_shrink
    colorbar_label_fontsize = args.colorbar_label_fontsize
    colorbar_tick_fontsize = args.colorbar_tick_fontsize
    if args.colorbar_fontsize is not None:
        colorbar_label_fontsize = args.colorbar_fontsize if colorbar_label_fontsize is None else colorbar_label_fontsize
        colorbar_tick_fontsize = args.colorbar_fontsize if colorbar_tick_fontsize is None else colorbar_tick_fontsize
    if colorbar_label_fontsize is None:
        colorbar_label_fontsize = 10.0
    if colorbar_tick_fontsize is None:
        colorbar_tick_fontsize = 8.0
    if args.colorbar_match_panel_height:
        fig.canvas.draw()
        ref_ax = axes[-1]
        bbox = ref_ax.get_position()
        cbar_height = bbox.height * colorbar_shrink
        cbar_y = bbox.y0 + (bbox.height - cbar_height) / 2.0 + args.colorbar_y_offset
        cbar_x = bbox.x1 + args.colorbar_pad
        cax = fig.add_axes([cbar_x, cbar_y, args.colorbar_width, cbar_height])
        cbar = fig.colorbar(sm, cax=cax)
    else:
        cbar = fig.colorbar(
            sm,
            ax=axes,
            fraction=args.colorbar_fraction,
            pad=args.colorbar_pad,
            shrink=colorbar_shrink,
            aspect=args.colorbar_aspect,
        )
    cbar.set_label(args.colorbar_label, fontsize=colorbar_label_fontsize)
    cbar.ax.tick_params(labelsize=colorbar_tick_fontsize)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=args.dpi, transparent=args.transparent, bbox_inches="tight", pad_inches=0.02)
    if args.extra_svg and out.suffix.lower() != ".svg":
        fig.savefig(out.with_suffix(".svg"), transparent=args.transparent, bbox_inches="tight", pad_inches=0.02)
    if args.extra_pdf and out.suffix.lower() != ".pdf":
        fig.savefig(out.with_suffix(".pdf"), transparent=args.transparent, bbox_inches="tight", pad_inches=0.02)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot SUMO network topology with route/edge density panels.")
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--scenario", action="append", help="Label=/path/to/scenario.rou.xml or Label=/path/to/edgeData.xml; repeatable")
    ap.add_argument("--manifest-csv", help="Optional scenario manifest with route_file/routes_file/scenario_routes column")
    ap.add_argument("--manifest-density-filter", action="append", help="Keep manifest rows with matching density_label or density_count; repeatable")
    ap.add_argument("--manifest-route-filter", action="append", help="Keep manifest rows with this route_id; repeatable")
    ap.add_argument("--source", choices=["routes", "edge-data"], default="routes")
    ap.add_argument("--edge-metric", default="density", help="edgeData metric if --source edge-data")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-panels", type=int, default=0)
    ap.add_argument("--ev-id", help="EV id to highlight, e.g. emergency5 or 5")
    ap.add_argument("--strict-ev-id", action="store_true", help="Fail instead of falling back when --ev-id is not present in the selected route file.")
    ap.add_argument("--debug-selection", action="store_true", help="Print selected manifest rows, route files, and highlighted EV id.")
    ap.add_argument("--ev-prefix", default="emergency")
    ap.add_argument("--show-ev-route", action="store_true")
    ap.add_argument("--marker-node", help="Node id to mark in the base panel")
    ap.add_argument("--scout-marker-node", action="append", help="Node to mark on scenario panels, optionally NodeId=Label. Repeatable.")
    ap.add_argument("--scout-marker-style", choices=["star", "dot"], default="star")
    ap.add_argument("--scout-marker-size", type=float, default=170.0)
    ap.add_argument("--scout-marker-color", default="#E31A1C")
    ap.add_argument("--scout-marker-edge-color", default="#111111")
    ap.add_argument("--scout-marker-linewidth", type=float, default=0.8)
    ap.add_argument("--scout-marker-label", action="store_true", help="Draw labels next to scout markers.")
    ap.add_argument("--scout-marker-label-fontsize", type=float, default=8.0)
    ap.add_argument("--scout-marker-label-color", default="#111111")
    ap.add_argument("--scout-marker-label-dx", type=float, default=18.0)
    ap.add_argument("--scout-marker-label-dy", type=float, default=18.0)
    ap.add_argument(
        "--scout-marker-label-offset",
        action="append",
        type=parse_marker_label_offset,
        help="Override one scout label offset as NodeId=dx,dy in SUMO map units. Repeatable.",
    )
    ap.add_argument("--scout-marker-on-base", action="store_true", help="Also draw scout markers on the optional base network panel.")
    ap.add_argument("--region-grid", help="Overlay a region grid as COLSxROWS, e.g. 3x2 for six lab-mapping regions.")
    ap.add_argument("--region-grid-bounds", help="Explicit grid bounds as XMIN,XMAX,YMIN,YMAX in SUMO coordinates. Defaults to current map view.")
    ap.add_argument("--region-grid-color", default="#E31A1C")
    ap.add_argument("--region-grid-linewidth", type=float, default=1.2)
    ap.add_argument("--region-grid-alpha", type=float, default=0.92)
    ap.add_argument("--region-grid-linestyle", default="-")
    ap.add_argument("--region-grid-zorder", type=float, default=7.0)
    ap.add_argument("--region-grid-on-base", action="store_true", help="Also draw region grid on the optional base network panel.")
    ap.add_argument("--region-grid-label", action="store_true", help="Label grid cells left-to-right, top-to-bottom.")
    ap.add_argument("--region-grid-label-prefix", default="R")
    ap.add_argument("--region-grid-label-fontsize", type=float, default=9.0)
    ap.add_argument("--region-grid-label-color", default="#E31A1C")
    ap.add_argument("--region-grid-label-weight", default="bold")
    ap.add_argument("--region-grid-label-x-frac", type=float, default=0.06, help="Label x-position within each grid cell, as cell-width fraction.")
    ap.add_argument("--region-grid-label-y-frac", type=float, default=0.10, help="Label y-position within each grid cell, as cell-height fraction from top.")
    ap.add_argument("--region-grid-label-ha", default="left")
    ap.add_argument("--region-grid-label-va", default="top")
    ap.add_argument("--region-grid-label-box", action="store_true")
    ap.add_argument("--region-grid-label-box-color", default="white")
    ap.add_argument("--region-grid-label-box-alpha", type=float, default=0.70)
    ap.add_argument("--base-title", default="Generated Simulation Network")
    ap.add_argument("--base-title-fontsize", type=float, default=None, help="Font size for the base network title. Defaults to --title-fontsize.")
    ap.add_argument("--base-title-pad", type=float, default=None, help="Padding between the base network title and map panel. Defaults to --title-pad.")
    ap.add_argument("--base-title-inside-y", type=float, default=None, help="Inside-title y-position for base network panel only. Defaults to --title-inside-y.")
    ap.add_argument("--panel-title", default=None, help="Use this title for every scenario map panel. If multiple scenario panels exist, appends panel index.")
    ap.add_argument("--panel-title-prefix", default="")
    ap.add_argument("--panel-title-fontsize", type=float, default=None, help="Font size for scenario map panel titles. Defaults to --title-fontsize.")
    ap.add_argument("--panel-title-pad", type=float, default=None, help="Padding between scenario panel titles and top map border. Defaults to --title-pad.")
    ap.add_argument("--panel-title-inside-y", type=float, default=None, help="Inside-title y-position for scenario panels only. Defaults to --title-inside-y.")
    ap.add_argument("--panel-label", action="append", help="Override panel title as old=new; repeatable.")
    ap.add_argument("--colorbar-label", default="Route demand density")
    ap.add_argument("--cmap", default="RdYlGn_r")
    ap.add_argument("--log-scale", action="store_true", default=True)
    ap.add_argument("--no-log-scale", dest="log_scale", action="store_false")
    ap.add_argument("--log-min", type=float, default=0.1)
    ap.add_argument("--base-color", default="#686868")
    ap.add_argument("--background-color", default="#9A9A9A")
    ap.add_argument("--ev-route-color", default="#111111")
    ap.add_argument("--base-linewidth", type=float, default=0.55)
    ap.add_argument("--background-linewidth", type=float, default=0.45)
    ap.add_argument("--density-linewidth", type=float, default=1.10)
    ap.add_argument("--ev-route-linewidth", type=float, default=3.20)
    ap.add_argument("--hide-ticks", action="store_true")
    ap.add_argument("--axis-units", action="store_true", help="Show x/y axes in SUMO map meters.")
    ap.add_argument("--no-base-panel", action="store_true")
    ap.add_argument("--panel-width", type=float, default=3.20, help="Width per panel in inches when --fig-width is not set.")
    ap.add_argument("--panel-height", type=float, default=3.80, help="Figure height in inches when --fig-height is not set.")
    ap.add_argument("--fig-width", type=float, default=0.0, help="Override total figure width in inches.")
    ap.add_argument("--fig-height", type=float, default=0.0, help="Override total figure height in inches.")
    ap.add_argument("--map-aspect", choices=["equal", "auto"], default="equal", help="Map aspect mode. Use equal for true map scale; auto can fill the panel but distorts geometry.")
    ap.add_argument("--map-box-aspect", type=float, default=0.0, help="Axes box height/width ratio. Use 1.0 for square axes when compatible with the chosen aspect.")
    ap.add_argument(
        "--map-fit-bounds",
        choices=["network", "density", "ev-route", "density-ev-route"],
        default="network",
        help="Which geometry defines the initial map view before padding/square/zoom/crop. Use density-ev-route to zoom around the plotted heatmap and highlighted route.",
    )
    ap.add_argument("--map-zoom", type=float, default=1.0, help="Zoom around current map center after padding/square expansion. Values >1 zoom in, e.g. 1.25.")
    ap.add_argument("--square-map-limits", action="store_true", help="Expand map x/y limits to equal span so a single panel can render square without distorting coordinates.")
    ap.add_argument("--map-x-pad-ratio", type=float, default=0.04, help="Horizontal map padding as fraction of network width.")
    ap.add_argument("--map-y-pad-ratio", type=float, default=0.04, help="Vertical map padding as fraction of network height.")
    ap.add_argument("--crop-left-ratio", type=float, default=0.0, help="Crop this fraction from the final left map limit after padding/square expansion.")
    ap.add_argument("--crop-right-ratio", type=float, default=0.0, help="Crop this fraction from the final right map limit after padding/square expansion.")
    ap.add_argument("--crop-bottom-ratio", type=float, default=0.0, help="Crop this fraction from the final bottom map limit after padding/square expansion.")
    ap.add_argument("--crop-top-ratio", type=float, default=0.0, help="Crop this fraction from the final top map limit after padding/square expansion.")
    ap.add_argument("--map-xmin", type=float, default=None, help="Explicit final x-axis minimum in SUMO map coordinates.")
    ap.add_argument("--map-xmax", type=float, default=None, help="Explicit final x-axis maximum in SUMO map coordinates.")
    ap.add_argument("--map-ymin", type=float, default=None, help="Explicit final y-axis minimum in SUMO map coordinates.")
    ap.add_argument("--map-ymax", type=float, default=None, help="Explicit final y-axis maximum in SUMO map coordinates.")
    ap.add_argument("--title-inside", action="store_true", help="Draw panel titles inside plot borders.")
    ap.add_argument("--title-fontsize", type=float, default=12.0)
    ap.add_argument("--title-inside-y", type=float, default=0.945, help="Inside title y-position in axes coordinates; lower values add more top padding.")
    ap.add_argument("--title-pad", type=float, default=3.0)
    ap.add_argument("--tick-fontsize", type=float, default=8.0)
    ap.add_argument("--axis-tick-fontsize", type=float, default=None, help="Alias/override for map x/y tick font size.")
    ap.add_argument("--axis-label-fontsize", type=float, default=9.0)
    ap.add_argument("--x-axis-label-fontsize", type=float, default=None, help="X-axis unit label font size. Defaults to --axis-label-fontsize.")
    ap.add_argument("--y-axis-label-fontsize", type=float, default=None, help="Y-axis unit label font size. Defaults to --axis-label-fontsize.")
    ap.add_argument("--colorbar-shrink", type=float, default=0.82, help="Matplotlib colorbar shrink ratio. Kept for compatibility; --colorbar-height-ratio is clearer.")
    ap.add_argument("--colorbar-height-ratio", "--colorbar-height", dest="colorbar_height_ratio", type=float, default=None, help="Right-side heatmap colorbar height as a fraction of panel height, e.g. 0.55 for a shorter bar.")
    ap.add_argument("--colorbar-fraction", type=float, default=0.026)
    ap.add_argument("--colorbar-pad", type=float, default=0.012)
    ap.add_argument("--colorbar-aspect", type=float, default=24.0)
    ap.add_argument("--colorbar-match-panel-height", action="store_true", help="Manually place the colorbar so its height is measured against the visible map panel.")
    ap.add_argument("--colorbar-width", type=float, default=0.022, help="Manual colorbar width in figure coordinates when --colorbar-match-panel-height is used.")
    ap.add_argument("--colorbar-y-offset", type=float, default=0.0, help="Manual colorbar vertical offset in figure coordinates when --colorbar-match-panel-height is used.")
    ap.add_argument("--colorbar-fontsize", type=float, default=None, help="Convenience font size for both heatmap colorbar label and tick labels.")
    ap.add_argument("--colorbar-label-fontsize", "--heatmap-label-fontsize", dest="colorbar_label_fontsize", type=float, default=None, help="Heatmap colorbar label font size. Overrides --colorbar-fontsize for the label.")
    ap.add_argument("--colorbar-tick-fontsize", type=float, default=None, help="Heatmap colorbar tick font size. Overrides --colorbar-fontsize for ticks.")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--transparent", action="store_true")
    ap.add_argument("--extra-svg", action="store_true")
    ap.add_argument("--extra-pdf", action="store_true")
    args = ap.parse_args()
    plot(args)


if __name__ == "__main__":
    main()
