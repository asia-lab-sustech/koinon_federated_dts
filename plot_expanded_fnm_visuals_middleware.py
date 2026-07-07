#!/usr/bin/env python3
"""Create expanded 4:3 visual set from extracted CSV metrics."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean
from typing import Dict, List, Optional, Tuple

try:
    import matplotlib.pyplot as plt
    from matplotlib.sankey import Sankey
except Exception:
    plt = None
    Sankey = None


FIG_43 = (12, 9)


def _mkdir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def _read_csv(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def _esc(s: object) -> str:
    return (
        str(s)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _write_svg_bar(path: Path, title: str, rows: List[Tuple[str, float]], *, subtitle: str = "", color: str = "#2f6f73") -> None:
    rows = [(str(k), float(v)) for k, v in rows if float(v) >= 0.0]
    if not rows:
        return
    rows = rows[:24]
    width, height = 1280, 760
    ml, mr, mt, mb = 110, 50, 90, 175
    pw, ph = width - ml - mr, height - mt - mb
    max_v = max(v for _, v in rows) or 1.0
    n = len(rows)
    gap = max(4, min(14, int(pw / max(1, n) * 0.12)))
    bw = max(8, (pw - gap * (n - 1)) / max(1, n))
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfaf7"/>',
        f'<text x="{ml}" y="38" font-family="Avenir Next, Helvetica, Arial" font-size="28" font-weight="700" fill="#21302f">{_esc(title)}</text>',
    ]
    if subtitle:
        parts.append(f'<text x="{ml}" y="64" font-family="Avenir Next, Helvetica, Arial" font-size="14" fill="#586765">{_esc(subtitle)}</text>')
    for i in range(5):
        y = mt + ph - ph * i / 4.0
        val = max_v * i / 4.0
        parts.append(f'<line x1="{ml}" y1="{y:.1f}" x2="{width-mr}" y2="{y:.1f}" stroke="#d8dedb" stroke-width="1"/>')
        parts.append(f'<text x="{ml-12}" y="{y+4:.1f}" text-anchor="end" font-family="Avenir Next, Helvetica, Arial" font-size="12" fill="#65716f">{val:,.0f}</text>')
    for i, (label, val) in enumerate(rows):
        x = ml + i * (bw + gap)
        h = ph * val / max_v if max_v else 0.0
        y = mt + ph - h
        parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw:.1f}" height="{h:.1f}" rx="4" fill="{color}" opacity="0.92"/>')
        parts.append(f'<text x="{x + bw / 2:.1f}" y="{y-7:.1f}" text-anchor="middle" font-family="Avenir Next, Helvetica, Arial" font-size="11" fill="#21302f">{val:,.1f}</text>')
        parts.append(f'<text transform="translate({x + bw / 2:.1f},{mt + ph + 18:.1f}) rotate(28)" text-anchor="start" font-family="Avenir Next, Helvetica, Arial" font-size="12" fill="#2f3d3b">{_esc(label[:48])}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def _write_svg_grouped(path: Path, title: str, groups: List[str], series: Dict[str, List[float]], *, subtitle: str = "") -> None:
    if not groups or not series:
        return
    width, height = 1320, 780
    ml, mr, mt, mb = 110, 50, 90, 140
    pw, ph = width - ml - mr, height - mt - mb
    names = list(series.keys())
    vals = [v for arr in series.values() for v in arr if v == v]
    max_v = max(vals or [1.0])
    group_w = pw / max(1, len(groups))
    bw = max(8, group_w / max(1, len(names) + 1))
    colors = ["#386f9f", "#c06f3e", "#2f806d", "#7a5c99"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfaf7"/>',
        f'<text x="{ml}" y="38" font-family="Avenir Next, Helvetica, Arial" font-size="28" font-weight="700" fill="#21302f">{_esc(title)}</text>',
    ]
    if subtitle:
        parts.append(f'<text x="{ml}" y="64" font-family="Avenir Next, Helvetica, Arial" font-size="14" fill="#586765">{_esc(subtitle)}</text>')
    for i in range(5):
        y = mt + ph - ph * i / 4.0
        val = max_v * i / 4.0
        parts.append(f'<line x1="{ml}" y1="{y:.1f}" x2="{width-mr}" y2="{y:.1f}" stroke="#d8dedb" stroke-width="1"/>')
        parts.append(f'<text x="{ml-12}" y="{y+4:.1f}" text-anchor="end" font-family="Avenir Next, Helvetica, Arial" font-size="12" fill="#65716f">{val:,.0f}</text>')
    for gi, g in enumerate(groups):
        base = ml + gi * group_w + (group_w - bw * len(names)) / 2.0
        for si, name in enumerate(names):
            arr = series[name]
            val = arr[gi] if gi < len(arr) and arr[gi] == arr[gi] else 0.0
            h = ph * val / max_v if max_v else 0.0
            x = base + si * bw
            y = mt + ph - h
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw*0.86:.1f}" height="{h:.1f}" rx="3" fill="{colors[si % len(colors)]}"/>')
        parts.append(f'<text x="{ml + gi * group_w + group_w/2:.1f}" y="{mt+ph+24:.1f}" text-anchor="middle" font-family="Avenir Next, Helvetica, Arial" font-size="13" fill="#2f3d3b">{_esc(g)}</text>')
    lx = ml
    ly = height - 35
    for si, name in enumerate(names):
        parts.append(f'<rect x="{lx}" y="{ly-12}" width="14" height="14" fill="{colors[si % len(colors)]}"/>')
        parts.append(f'<text x="{lx+20}" y="{ly}" font-family="Avenir Next, Helvetica, Arial" font-size="13" fill="#2f3d3b">{_esc(name)}</text>')
        lx += 130
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def _plot_svg_fallbacks(
    metrics_dir: Path,
    out_dir: Path,
    travel: List[dict],
    artifact: List[dict],
    fnm_events: List[dict],
    fnm_micro: List[dict],
    coord_rows: List[dict],
    overhead_route: List[dict],
) -> None:
    _mkdir(out_dir)
    routes = sorted({_i(r.get("route_id")) for r in travel if _i(r.get("route_id")) is not None})
    mode_vals: Dict[str, List[float]] = {}
    for mode in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"]:
        mode_vals[mode] = []
        for route in routes:
            vals = [_f(r.get("travel_time_s")) for r in travel if _i(r.get("route_id")) == route and str(r.get("mode")) == mode]
            mode_vals[mode].append(vals[0] if vals and vals[0] is not None else float("nan"))
    _write_svg_grouped(out_dir / "middleware_travel_time_by_route.svg", "Travel Time by Route and Mode", [f"R{r}" for r in routes], mode_vals, subtitle="Attached 2K full-set run")

    fam_counts: Dict[str, Dict[str, float]] = {}
    for r in artifact:
        mode = str(r.get("mode", "") or "")
        fam = str(r.get("artifact_family", "") or "")
        fam_counts.setdefault(mode, {}).setdefault(fam, 0.0)
        fam_counts[mode][fam] += _f(r.get("count"), 0.0) or 0.0
    art_rows = [(f"{m}:{fam}", v) for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] for fam, v in sorted(fam_counts.get(m, {}).items())]
    _write_svg_bar(out_dir / "middleware_artifact_volume_by_mode.svg", "Artifact Volume by Mode", art_rows, subtitle="Event family counts extracted from fed_outcomes JSONL", color="#7a5c99")

    evt_counts: Dict[str, float] = {}
    for r in fnm_events:
        evt = str(r.get("event", "") or "")
        evt_counts[evt] = evt_counts.get(evt, 0.0) + (_f(r.get("count"), 0.0) or 0.0)
    _write_svg_bar(out_dir / "middleware_fnm_event_counts.svg", "FNM Sidecar Event Counts", sorted(evt_counts.items(), key=lambda x: x[1], reverse=True)[:18], subtitle="Gateway/DT sidecar middleware behavior", color="#c06f3e")

    med_rows = []
    for r in overhead_route:
        label = f"R{_i(r.get('route_id'))}-{r.get('mode')}"
        med_rows.append((label, _f(r.get("total_e2e_ms"), 0.0) or 0.0))
    _write_svg_bar(out_dir / "middleware_e2e_overhead_by_route.svg", "Estimated Middleware + Coordination Overhead", med_rows, subtitle="Local compute + apply + FNM mediation + coordination request/response", color="#2f806d")

    coord_rows_plot = []
    for r in coord_rows:
        if str(r.get("mode")) == "F2":
            coord_rows_plot.append((f"R{_i(r.get('route_id'))}", _f(r.get("request_to_decision_median_ms"), 0.0) or 0.0))
    _write_svg_bar(out_dir / "middleware_f2_request_to_decision_latency.svg", "F2 Request-to-Decision Median Latency", coord_rows_plot, subtitle="Per-route median coordination decision latency", color="#386f9f")

    micro: Dict[str, List[float]] = defaultdict(list)
    for r in fnm_micro:
        ev = str(r.get("event", "") or "")
        val = _f(r.get("total_stage_ms"), None)
        if ev and val is not None:
            micro[ev].append(float(val))
    means = [(k, mean(v)) for k, v in micro.items() if v]
    _write_svg_bar(out_dir / "middleware_fnm_micro_latency_mean.svg", "FNM Micro-Latency Mean by Stage", sorted(means, key=lambda x: x[1], reverse=True), subtitle="Schema/protocol and routing/orchestration stage durations", color="#9a6b2f")


def _f(v: object, d: Optional[float] = None) -> Optional[float]:
    if v is None:
        return d
    try:
        return float(v)
    except Exception:
        return d


def _i(v: object, d: Optional[int] = None) -> Optional[int]:
    if v is None:
        return d
    try:
        return int(float(v))
    except Exception:
        return d


def _save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _timeline_artifact_family(event_type: str) -> str:
    et = str(event_type or "")
    if et in {
        "ev.request.in",
        "coord.reservation.req_in",
        "coord.reservation.req_out",
        "coord.reservation.req_decision",
        "coord.reservation.resp_in",
        "coord.reservation.req_resp_e2e",
    }:
        return "request_response"
    if et.startswith("coord.refine.") or et.startswith("coord.apply.") or et.startswith("agent.stage."):
        return "coordination"
    if et.startswith("tls.signal.") or et in {"ev.pass.detected", "ev.node.cross"}:
        return "event"
    if et.startswith("membership.") or et.startswith("catalog.") or et.startswith("discovery."):
        return "state"
    return "other"


def _scenario_label_short(scenario_id: str) -> str:
    sid = str(scenario_id or "")
    if "moderate05k" in sid or "_d500_" in sid:
        return "0.5K"
    if "severe1p5k" in sid or "_d1500_" in sid:
        return "1.5K"
    if "severe1k" in sid or "_d1000_" in sid:
        return "1K"
    if "severe2k" in sid or "_d2000_" in sid:
        return "2K"
    return sid


def _pick_focus_case(
    timeline_rows: List[dict],
    burst_rows: List[dict],
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> Optional[Tuple[str, str, int]]:
    def _case_exists(ds: str, sc: str, rt: int) -> bool:
        for r in burst_rows:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and (_i(r.get("route_id")) == int(rt))
            ):
                return True
        for r in timeline_rows:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and (_i(r.get("route_id")) == int(rt))
            ):
                return True
        return False

    if dataset_filter and scenario_filter and route_filter >= 0 and _case_exists(dataset_filter, scenario_filter, int(route_filter)):
        return (dataset_filter, scenario_filter, int(route_filter))
    # Prefer highest F2 coordination/request volume as representative.
    score: Dict[Tuple[str, str, int], float] = defaultdict(float)
    for r in burst_rows:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _i(r.get("route_id"))
        md = str(r.get("mode", "") or "")
        fam = str(r.get("artifact_family", "") or "")
        cnt = _f(r.get("count"), 0.0) or 0.0
        if ds and sc and rt is not None and md == "F2" and fam in {"request_response", "coordination"}:
            score[(ds, sc, int(rt))] += cnt
    if score:
        return max(score.items(), key=lambda kv: kv[1])[0]
    # Fallback to timeline.
    for r in timeline_rows:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _i(r.get("route_id"))
        if ds and sc and rt is not None:
            return (ds, sc, int(rt))
    return None


def _route_node_order_from_timeline(
    timeline_rows: List[dict],
    dataset: str,
    scenario_id: str,
    route_id: int,
    preferred_mode: str = "F2",
) -> List[str]:
    rows = [
        r
        for r in timeline_rows
        if str(r.get("dataset", "") or "") == dataset
        and str(r.get("scenario_id", "") or "") == scenario_id
        and (_i(r.get("route_id")) == int(route_id))
        and str(r.get("tls_id", "") or "")
        and _f(r.get("sim_time")) is not None
    ]
    if not rows:
        return []
    pref = [r for r in rows if str(r.get("mode", "") or "") == preferred_mode]
    use = pref if pref else rows
    first_seen: Dict[str, float] = {}
    for r in use:
        tls = str(r.get("tls_id", "") or "")
        t = _f(r.get("sim_time"), 1e12) or 1e12
        if tls and tls not in first_seen:
            first_seen[tls] = t
    return [k for k, _ in sorted(first_seen.items(), key=lambda kv: kv[1])]


def _grouped_travel_by_dataset(travel: List[dict], out_dir: Path) -> None:
    by_ds_mode_route: Dict[Tuple[str, str, int], List[float]] = defaultdict(list)
    for r in travel:
        ds = str(r.get("dataset", "") or "")
        mode = str(r.get("mode", "") or "")
        route = _i(r.get("route_id"))
        tt = _f(r.get("travel_time_s"))
        if ds and mode and route is not None and tt is not None:
            by_ds_mode_route[(ds, mode, route)].append(tt)

    datasets = sorted({k[0] for k in by_ds_mode_route.keys()})
    for ds in datasets:
        routes = sorted({k[2] for k in by_ds_mode_route.keys() if k[0] == ds})
        modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if any((ds, m, r) in by_ds_mode_route for r in routes)]
        if not routes or not modes:
            continue

        fig, ax = plt.subplots(figsize=FIG_43)
        n_modes = len(modes)
        width = 0.22
        x = list(range(len(routes)))
        offset_base = -0.5 * width * (n_modes - 1)
        for mi, mode in enumerate(modes):
            y = []
            for r in routes:
                vals = by_ds_mode_route.get((ds, mode, r), [])
                y.append(mean(vals) if vals else math.nan)
            ax.bar([xi + offset_base + mi * width for xi in x], y, width=width, label=mode)

        ax.set_title(f"EV Travel Time by Route and Mode ({ds})")
        ax.set_xlabel("Route")
        ax.set_ylabel("Travel time (s)")
        ax.set_xticks(x)
        ax.set_xticklabels([f"R{r}" for r in routes])
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend()
        _save(fig, out_dir / f"travel_grouped_{ds}.png")


def _boxplot_latency(lat_rr: List[dict], lat_rd: List[dict], lat_ra: List[dict], out_dir: Path) -> None:
    # K + P style latency boxplots by mode.
    by_mode_rr: Dict[str, List[float]] = defaultdict(list)
    by_mode_rd: Dict[str, List[float]] = defaultdict(list)
    by_mode_ra: Dict[str, List[float]] = defaultdict(list)
    for r in lat_rr:
        m = str(r.get("mode", "") or "")
        v = _f(r.get("latency_ms"))
        if m and v is not None:
            by_mode_rr[m].append(v)
    for r in lat_rd:
        m = str(r.get("mode", "") or "")
        v = _f(r.get("latency_ms"))
        if m and v is not None:
            by_mode_rd[m].append(v)
    for r in lat_ra:
        m = str(r.get("mode", "") or "")
        v = _f(r.get("latency_ms"))
        if m and v is not None:
            by_mode_ra[m].append(v)

    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if by_mode_rr.get(m) or by_mode_rd.get(m) or by_mode_ra.get(m)]
    if not modes:
        return

    fig, axs = plt.subplots(1, 3, figsize=FIG_43)
    for ax, title, source in [
        (axs[0], "Request->Response Latency", by_mode_rr),
        (axs[1], "Request->Decision Latency", by_mode_rd),
        (axs[2], "Request->Actuation Latency (approx)", by_mode_ra),
    ]:
        data = [source.get(m, []) for m in modes]
        if any(len(x) > 0 for x in data):
            ax.boxplot(data, tick_labels=modes, showfliers=False)
        ax.set_title(title)
        ax.set_xlabel("Mode")
        ax.set_ylabel("Latency (ms)")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "latency_boxplots_by_mode.png")


def _boxplot_staleness(stale: List[dict], out_dir: Path) -> None:
    # L style age-of-information by mode and type.
    by_type_mode: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    for r in stale:
        typ = str(r.get("staleness_type", "") or "")
        mode = str(r.get("mode", "") or "")
        v = _f(r.get("value_ms"))
        if typ and mode and v is not None:
            by_type_mode[(typ, mode)].append(v)

    types = sorted({k[0] for k in by_type_mode.keys()})
    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if any((t, m) in by_type_mode for t in types)]
    if not types or not modes:
        return

    fig, axs = plt.subplots(1, max(1, len(types)), figsize=FIG_43)
    # Matplotlib returns either a single Axes or a numpy.ndarray of Axes.
    # Normalize to a flat list of Axes without importing numpy.
    if hasattr(axs, "ravel"):
        axs = list(axs.ravel())
    else:
        axs = [axs]
    for i, t in enumerate(types):
        ax = axs[i]
        data = [by_type_mode.get((t, m), []) for m in modes]
        if any(len(x) > 0 for x in data):
            ax.boxplot(data, tick_labels=modes, showfliers=False)
        ax.set_title(t)
        ax.set_xlabel("Mode")
        ax.set_ylabel("Age (ms)")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "staleness_boxplots_by_mode.png")


def _boxplot_aoi_by_source(stale: List[dict], out_dir: Path) -> None:
    # L plot: age of information by source type (seconds).
    source_map = {
        "ev_request_age_ms": "EV state used by intersection",
        "neighbor_state_phase_state_age_ms": "Intersection state used for local decision",
        "responder_phase_state_age_ms": "Coordination artefact used by neighbor",
        "feedback_age_ms": "Coordination feedback age",
    }
    by_source: Dict[str, List[float]] = defaultdict(list)
    for r in stale:
        st = str(r.get("staleness_type", "") or "")
        v_ms = _f(r.get("value_ms"))
        if st in source_map and v_ms is not None:
            by_source[source_map[st]].append(v_ms / 1000.0)
    sources = [k for k in source_map.values() if by_source.get(k)]
    if not sources:
        return
    data = [by_source[s] for s in sources]
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.boxplot(data, tick_labels=sources, showfliers=False)
    ax.set_title("Age of Information at Decision Time")
    ax.set_xlabel("Artefact/source type")
    ax.set_ylabel("Age (s)")
    ax.tick_params(axis="x", rotation=20)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "aoi_boxplot_by_source.png")


def _artifact_volume_stacked(artifact_rows: List[dict], out_dir: Path) -> None:
    # M style stacked artifact exchange by mode.
    fams = ["state", "event", "request_response", "coordination", "other"]
    by_mode = {m: Counter() for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"]}
    for r in artifact_rows:
        mode = str(r.get("mode", "") or "")
        fam = str(r.get("artifact_family", "") or "")
        cnt = _i(r.get("count"), 0) or 0
        if mode in by_mode and fam:
            by_mode[mode][fam] += cnt

    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if sum(by_mode[m].values()) > 0]
    if not modes:
        return

    fig, ax = plt.subplots(figsize=FIG_43)
    x = list(range(len(modes)))
    bottoms = [0.0] * len(modes)
    for fam in fams:
        vals = [float(by_mode[m].get(fam, 0)) for m in modes]
        ax.bar(x, vals, bottom=bottoms, label=fam)
        bottoms = [bottoms[i] + vals[i] for i in range(len(vals))]
    ax.set_title("Artefact Exchange Volume by Mode")
    ax.set_xlabel("Mode")
    ax.set_ylabel("Count")
    ax.set_xticks(x)
    ax.set_xticklabels(modes)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    _save(fig, out_dir / "artifact_volume_stacked_by_mode.png")


def _updated_artifact_volume_by_node_composite(
    artifact_node_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> None:
    # Node-level stacked artefact volume for selected route; B1 top, F2 bottom.
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        for r in artifact_node_rows:
            ds = str(r.get("dataset", "") or "")
            sc = str(r.get("scenario_id", "") or "")
            rt = _i(r.get("route_id"))
            if not ds or not sc or rt is None:
                continue
            if dataset_filter and ds != dataset_filter:
                continue
            if scenario_filter and sc != scenario_filter:
                continue
            if route_filter >= 0 and int(rt) != int(route_filter):
                continue
            focus = (ds, sc, int(rt))
            break
    if focus is None:
        return
    ds, sc, rt = focus
    fams = ["state", "event", "request_response", "coordination"]
    fam_colors = {
        "state": "#6baed6",
        "event": "#9ecae1",
        "request_response": "#fb6a4a",
        "coordination": "#cb181d",
    }

    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode="F2")
    by_mode_node_fam: Dict[Tuple[str, str, str], int] = defaultdict(int)
    for r in artifact_node_rows:
        if (
            str(r.get("dataset", "") or "") != ds
            or str(r.get("scenario_id", "") or "") != sc
            or _i(r.get("route_id")) != int(rt)
        ):
            continue
        md = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "")
        fam = str(r.get("artifact_family", "") or "")
        cnt = _i(r.get("count"), 0) or 0
        if md in {"B1", "F2", "F2P", "F2D", "F2PD"} and tls and fam in fams:
            by_mode_node_fam[(md, tls, fam)] += int(cnt)

    # Fallback from timeline if extractor CSV not present or empty.
    if not by_mode_node_fam:
        for r in timeline_rows:
            if (
                str(r.get("dataset", "") or "") != ds
                or str(r.get("scenario_id", "") or "") != sc
                or _i(r.get("route_id")) != int(rt)
            ):
                continue
            md = str(r.get("mode", "") or "")
            tls = str(r.get("tls_id", "") or "")
            fam = _timeline_artifact_family(str(r.get("event_type", "") or ""))
            if md in {"B1", "F2", "F2P", "F2D", "F2PD"} and tls and fam in fams:
                by_mode_node_fam[(md, tls, fam)] += 1

    if not by_mode_node_fam:
        return

    if not ordered_nodes:
        node_seen = []
        for (md, tls, fam), _ in by_mode_node_fam.items():
            if tls not in node_seen:
                node_seen.append(tls)
        ordered_nodes = node_seen
    nodes = [n for n in ordered_nodes if any((m, n, f) in by_mode_node_fam for m in {"B1", "F2", "F2P", "F2D", "F2PD"} for f in fams)]
    if not nodes:
        return

    fig, axs = plt.subplots(2, 1, figsize=FIG_43, sharex=True)
    pane_modes = [("B1", "LIDP"), ("F2", "FCDP")]
    global_max = 0.0
    for md, _ in pane_modes:
        for ni, n in enumerate(nodes):
            total = sum(float(by_mode_node_fam.get((md, n, fam), 0)) for fam in fams)
            if total > global_max:
                global_max = total
    if global_max <= 0.0:
        global_max = 1.0

    x = list(range(len(nodes)))
    labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]
    for pi, (md, md_label) in enumerate(pane_modes):
        ax = axs[pi]
        bottoms = [0.0] * len(nodes)
        for fam in fams:
            vals = [float(by_mode_node_fam.get((md, n, fam), 0)) for n in nodes]
            ax.bar(x, vals, bottom=bottoms, color=fam_colors[fam], label=fam)
            bottoms = [bottoms[i] + vals[i] for i in range(len(vals))]
        ax.set_ylim(0.0, global_max * 1.10)
        ax.set_ylabel("Count")
        ax.set_title(f"{md_label} Artefact Volume by Node")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
    axs[-1].set_xlabel("Intersection sequence")
    axs[-1].set_xticks(x)
    axs[-1].set_xticklabels(labels)
    handles, labels_h = axs[0].get_legend_handles_labels()
    if handles:
        uniq = {}
        for h, l in zip(handles, labels_h):
            if l not in uniq:
                uniq[l] = h
        axs[0].legend(uniq.values(), uniq.keys(), loc="upper right", ncol=2)
    fig.suptitle(f"Node-Level Artefact Volume (B1/F2) ({ds} | {sc} | route {rt})", fontsize=13, y=1.02)
    _save(fig, out_dir / "updated_artifact_volume_stacked_by_node_composite.png")


def _processing_ratio_grouped(proc_ratio: List[dict], out_dir: Path) -> None:
    # N plot: grouped bars by artefact type (received/translated/accepted/deferred).
    by_ds: Dict[str, List[dict]] = defaultdict(list)
    for r in proc_ratio:
        ds = str(r.get("dataset", "") or "")
        if ds:
            by_ds[ds].append(r)
    for ds, rows in by_ds.items():
        arts = [a for a in ["state", "event", "request_response", "coordination", "other"] if any(str(r.get("artifact_type", "")) == a for r in rows)]
        if not arts:
            continue
        fig, ax = plt.subplots(figsize=FIG_43)
        x = list(range(len(arts)))
        width = 0.2
        bars = [
            ("received_pct", "Received"),
            ("translated_pct", "Translated"),
            ("accepted_pct", "Accepted valid"),
            ("deferred_rejected_pct", "Deferred/Rejected"),
        ]
        offset_base = -0.5 * width * (len(bars) - 1)
        for bi, (key, label) in enumerate(bars):
            vals = []
            for art in arts:
                cand = [r for r in rows if str(r.get("artifact_type", "") or "") == art]
                v = mean([_f(c.get(key), 0.0) or 0.0 for c in cand]) if cand else 0.0
                vals.append(v)
            ax.bar([xi + offset_base + bi * width for xi in x], vals, width=width, label=label)
        ax.set_title(f"Successful Processing Ratio by Artefact Type ({ds})")
        ax.set_xlabel("Artefact type")
        ax.set_ylabel("Percentage (%)")
        ax.set_xticks(x)
        ax.set_xticklabels(arts)
        ax.set_ylim(0, 105)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend()
        _save(fig, out_dir / f"processing_ratio_grouped_{ds}.png")


def _coordination_success(summary: List[dict], out_dir: Path) -> None:
    # O style bar chart.
    by_mode: Dict[str, List[float]] = defaultdict(list)
    for r in summary:
        mode = str(r.get("mode", "") or "")
        v = _f(r.get("coordination_success_rate"))
        if mode and v is not None and not math.isnan(v):
            by_mode[mode].append(v)
    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if by_mode.get(m)]
    if not modes:
        return
    vals = [100.0 * mean(by_mode[m]) for m in modes]
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.bar(modes, vals)
    ax.set_title("Coordination Success Rate by Mode")
    ax.set_xlabel("Mode")
    ax.set_ylabel("Success rate (%)")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "coordination_success_rate_by_mode.png")


def _coordination_success_o(coord_rows: List[dict], out_dir: Path) -> None:
    # O plot: coordination success proxy in B1/F2.
    by_mode: Dict[str, List[float]] = defaultdict(list)
    for r in coord_rows:
        mode = str(r.get("mode", "") or "")
        if mode not in {"B1", "F2", "F2P", "F2D", "F2PD"}:
            continue
        v = _f(r.get("coordination_success_proxy_pct"))
        if v is not None and not math.isnan(v):
            by_mode[mode].append(v)
    modes = [m for m in ["B1", "F2", "F2P", "F2D", "F2PD"] if by_mode.get(m)]
    if not modes:
        return
    vals = [mean(by_mode[m]) for m in modes]
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.bar(modes, vals)
    ax.set_title("Coordination Success Rate (Proxy)")
    ax.set_xlabel("Mode")
    ax.set_ylabel("Successful coordination episodes (%)")
    ax.set_ylim(0, 105)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "coordination_success_proxy_O.png")


def _request_to_decision_boxplot_p(lat_rd: List[dict], out_dir: Path) -> None:
    # P plot: request->decision latency in seconds for B1/F2.
    by_mode: Dict[str, List[float]] = defaultdict(list)
    for r in lat_rd:
        mode = str(r.get("mode", "") or "")
        if mode not in {"B1", "F2", "F2P", "F2D", "F2PD"}:
            continue
        v_ms = _f(r.get("latency_ms"))
        if v_ms is not None:
            by_mode[mode].append(v_ms / 1000.0)
    modes = [m for m in ["B1", "F2", "F2P", "F2D", "F2PD"] if by_mode.get(m)]
    if not modes:
        return
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.boxplot([by_mode[m] for m in modes], tick_labels=modes, showfliers=False)
    ax.set_title("Time from Request to Decision")
    ax.set_xlabel("Mode")
    ax.set_ylabel("Latency (s)")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "request_to_decision_boxplot_P.png")


def _request_to_decision_boxplot_by_node(
    lat_rd_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> None:
    # Restored plot: node-level request->decision latency distribution.
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        for r in lat_rd_rows:
            ds = str(r.get("dataset", "") or "")
            sc = str(r.get("scenario_id", "") or "")
            rt = _i(r.get("route_id"))
            if not ds or not sc or rt is None:
                continue
            if dataset_filter and ds != dataset_filter:
                continue
            if scenario_filter and sc != scenario_filter:
                continue
            if route_filter >= 0 and int(rt) != int(route_filter):
                continue
            focus = (ds, sc, int(rt))
            break
    if focus is None:
        return
    ds, sc, rt = focus

    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode="F2")
    if not ordered_nodes:
        # Fallback to EV-sequence proxy from earliest F2 request->decision timestamps.
        first_seen_f2: Dict[str, float] = {}
        for r in lat_rd_rows:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and _i(r.get("route_id")) == int(rt)
                and str(r.get("mode", "") or "") == "F2"
            ):
                tls = str(r.get("tls_id", "") or "")
                ts = _f(r.get("sim_time"), 1e12) or 1e12
                if tls and tls not in first_seen_f2:
                    first_seen_f2[tls] = ts
        if first_seen_f2:
            ordered_nodes = [k for k, _ in sorted(first_seen_f2.items(), key=lambda kv: kv[1])]
        else:
            seen = []
            for r in lat_rd_rows:
                if (
                    str(r.get("dataset", "") or "") == ds
                    and str(r.get("scenario_id", "") or "") == sc
                    and _i(r.get("route_id")) == int(rt)
                ):
                    tls = str(r.get("tls_id", "") or "")
                    if tls and tls not in seen:
                        seen.append(tls)
            ordered_nodes = seen
    if not ordered_nodes:
        return

    by_node_mode: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    for r in lat_rd_rows:
        if (
            str(r.get("dataset", "") or "") != ds
            or str(r.get("scenario_id", "") or "") != sc
            or _i(r.get("route_id")) != int(rt)
        ):
            continue
        tls = str(r.get("tls_id", "") or "")
        md = str(r.get("mode", "") or "")
        lat = _f(r.get("latency_ms"))
        if tls and md in {"B1", "F2", "F2P", "F2D", "F2PD"} and lat is not None:
            by_node_mode[(tls, md)].append(float(lat) / 1000.0)

    nodes = [n for n in ordered_nodes if by_node_mode.get((n, "B1")) or by_node_mode.get((n, "F2"))]
    if not nodes:
        return

    # One figure per mode to keep readability.
    for mode, color in [("B1", "#1f77b4"), ("F2", "#ff7f0e")]:
        data = [by_node_mode.get((n, mode), []) for n in nodes]
        if not any(len(v) > 0 for v in data):
            continue
        labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]
        fig, ax = plt.subplots(figsize=FIG_43)
        bp = ax.boxplot(data, tick_labels=labels, showfliers=False, patch_artist=True)
        for patch in bp["boxes"]:
            patch.set_facecolor(color)
            patch.set_alpha(0.35)
        ax.set_title(f"Request-to-Decision Latency by Node ({mode}; {ds} | {sc} | route {rt})")
        ax.set_xlabel("Intersection sequence")
        ax.set_ylabel("Latency (s)")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
        suffix = "B1" if mode == "B1" else "F2"
        _save(fig, out_dir / f"request_to_decision_boxplot_by_node_{suffix}.png")

    # Combined grouped-by-node boxplot (B1/F2 side by side) for parity with previous use.
    fig, ax = plt.subplots(figsize=FIG_43)
    positions = []
    box_data = []
    tick_pos = []
    tick_labels = []
    p = 1
    for i, n in enumerate(nodes):
        d1 = by_node_mode.get((n, "B1"), [])
        d2 = by_node_mode.get((n, "F2"), [])
        if d1:
            positions.append(p)
            box_data.append(d1)
        if d2:
            positions.append(p + 0.35)
            box_data.append(d2)
        tick_pos.append(p + 0.175)
        tick_labels.append(f"I{i+1}\n{n}")
        p += 1.0
    if box_data:
        bp = ax.boxplot(box_data, positions=positions, widths=0.28, showfliers=False, patch_artist=True)
        # Color alternating B1/F2 boxes.
        for bi, patch in enumerate(bp["boxes"]):
            patch.set_facecolor("#1f77b4" if (bi % 2 == 0) else "#ff7f0e")
            patch.set_alpha(0.35)
        ax.set_xticks(tick_pos)
        ax.set_xticklabels(tick_labels)
        ax.set_title(f"Request-to-Decision Latency by Node (B1/F2; {ds} | {sc} | route {rt})")
        ax.set_xlabel("Intersection sequence")
        ax.set_ylabel("Latency (s)")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
        ax.plot([], [], color="#1f77b4", linewidth=8, alpha=0.35, label="B1")
        ax.plot([], [], color="#ff7f0e", linewidth=8, alpha=0.35, label="F2")
        ax.legend(loc="upper right")
        _save(fig, out_dir / "request_to_decision_boxplot_by_node.png")

    # Additional requested layout: top pane B1, bottom pane F2.
    fig, axs = plt.subplots(2, 1, figsize=FIG_43, sharex=True)
    pane_modes = [("B1", "#1f77b4"), ("F2", "#ff7f0e")]
    any_data = False
    labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]
    for i, (mode, color) in enumerate(pane_modes):
        axp = axs[i]
        data = [by_node_mode.get((n, mode), []) for n in nodes]
        if any(len(v) > 0 for v in data):
            any_data = True
            bp = axp.boxplot(data, tick_labels=labels, showfliers=False, patch_artist=True)
            for patch in bp["boxes"]:
                patch.set_facecolor(color)
                patch.set_alpha(0.35)
        axp.set_title(f"{mode} Request-to-Decision by Node")
        axp.set_ylabel("Latency (s)")
        axp.grid(axis="y", linestyle="--", alpha=0.20)
    axs[-1].set_xlabel("Intersection sequence")
    if any_data:
        _save(fig, out_dir / "request_to_decision_boxplot_by_node_panes.png")
    else:
        plt.close(fig)


def _corridor_continuity_q(coord_rows: List[dict], out_dir: Path) -> None:
    # Q plot: corridor continuity metrics by mode.
    by_mode_coord_nodes: Dict[str, List[float]] = defaultdict(list)
    by_mode_no_stop_streak: Dict[str, List[float]] = defaultdict(list)
    for r in coord_rows:
        mode = str(r.get("mode", "") or "")
        if mode not in {"B0", "B1", "F2", "F2P", "F2D", "F2PD"}:
            continue
        c = _f(r.get("coordinated_intersections_per_trip"))
        s = _f(r.get("consecutive_intersections_no_stop_max"))
        if c is not None and not math.isnan(c):
            by_mode_coord_nodes[mode].append(c)
        if s is not None and not math.isnan(s):
            by_mode_no_stop_streak[mode].append(s)
    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if by_mode_coord_nodes.get(m) or by_mode_no_stop_streak.get(m)]
    if not modes:
        return
    fig, axs = plt.subplots(1, 2, figsize=FIG_43)
    vals1 = [mean(by_mode_coord_nodes[m]) if by_mode_coord_nodes.get(m) else 0.0 for m in modes]
    vals2 = [mean(by_mode_no_stop_streak[m]) if by_mode_no_stop_streak.get(m) else 0.0 for m in modes]
    axs[0].bar(modes, vals1)
    axs[0].set_title("Average Coordinated Intersections per Trip")
    axs[0].set_xlabel("Mode")
    axs[0].set_ylabel("Count")
    axs[0].grid(axis="y", linestyle="--", alpha=0.35)
    axs[1].bar(modes, vals2)
    axs[1].set_title("Consecutive Intersections Crossed Without Stop")
    axs[1].set_xlabel("Mode")
    axs[1].set_ylabel("Max streak")
    axs[1].grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "corridor_continuity_Q.png")


def _node_spillback_heatmap(queue_rows: List[dict], out_dir: Path, top_k: int) -> None:
    # D/E proxy using spillback_risk and q_margin from req_decision.
    by_mode_tls: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    tls_totals: Counter = Counter()
    for r in queue_rows:
        mode = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "")
        sp = _f(r.get("spillback_risk"))
        if mode and tls and sp is not None:
            by_mode_tls[(mode, tls)].append(sp)
            tls_totals[tls] += 1

    top_tls = [tls for tls, _ in tls_totals.most_common(top_k)]
    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if any((m, t) in by_mode_tls for t in top_tls)]
    if not top_tls or not modes:
        return

    matrix: List[List[float]] = []
    for m in modes:
        row = []
        for tls in top_tls:
            vals = by_mode_tls.get((m, tls), [])
            row.append(mean(vals) if vals else 0.0)
        matrix.append(row)

    fig, ax = plt.subplots(figsize=FIG_43)
    im = ax.imshow(matrix, aspect="auto")
    ax.set_title("Mean Spillback Risk by Node and Mode")
    ax.set_xlabel("TLS node")
    ax.set_ylabel("Mode")
    ax.set_xticks(list(range(len(top_tls))))
    ax.set_xticklabels(top_tls, rotation=45, ha="right")
    ax.set_yticks(list(range(len(modes))))
    ax.set_yticklabels(modes)
    fig.colorbar(im, ax=ax, label="spillback_risk")
    _save(fig, out_dir / "node_spillback_heatmap.png")


def _queue_margin_grouped(queue_rows: List[dict], out_dir: Path, top_k: int) -> None:
    # D proxy grouped bar with mean q_margin_sec.
    by_mode_tls: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    tls_totals: Counter = Counter()
    for r in queue_rows:
        mode = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "")
        qm = _f(r.get("q_margin_sec"))
        if mode and tls and qm is not None:
            by_mode_tls[(mode, tls)].append(qm)
            tls_totals[tls] += 1
    top_tls = [tls for tls, _ in tls_totals.most_common(top_k)]
    modes = [m for m in ["B0", "B1", "F2", "F2P", "F2D", "F2PD"] if any((m, t) in by_mode_tls for t in top_tls)]
    if not top_tls or not modes:
        return

    fig, ax = plt.subplots(figsize=FIG_43)
    x = list(range(len(top_tls)))
    width = 0.22
    offset_base = -0.5 * width * (len(modes) - 1)
    for mi, mode in enumerate(modes):
        y = []
        for tls in top_tls:
            vals = by_mode_tls.get((mode, tls), [])
            y.append(mean(vals) if vals else math.nan)
        ax.bar([xi + offset_base + mi * width for xi in x], y, width=width, label=mode)
    ax.set_title("Queue Margin by Node and Mode")
    ax.set_xlabel("TLS node")
    ax.set_ylabel("Mean q_margin_sec")
    ax.set_xticks(x)
    ax.set_xticklabels(top_tls, rotation=45, ha="right")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    _save(fig, out_dir / "queue_margin_grouped_by_node_mode.png")


def _timeline_plot(timeline_rows: List[dict], out_dir: Path) -> None:
    # G/H style: sequence markers for one selected case already filtered in extractor.
    if not timeline_rows:
        return
    timeline_rows = [r for r in timeline_rows if _f(r.get("sim_time")) is not None]
    if not timeline_rows:
        return
    timeline_rows.sort(key=lambda r: _f(r.get("sim_time")) or 0.0)

    ev_rows = [r for r in timeline_rows if str(r.get("event_type")) == "ev.request.in"]
    speed_rows = [r for r in ev_rows if _f(r.get("speed_mps")) is not None]

    # Timeline
    lane_map = {
        "ev.request.in": 0,
        "coord.refine.candidates": 1,
        "coord.reservation.req_out": 2,
        "coord.reservation.req_decision": 3,
        "coord.apply.plan": 4,
        "tls.signal.change": 5,
        "ev.pass.detected": 6,
    }
    labels = {
        0: "EV request in",
        1: "Refine candidates",
        2: "Reservation req out",
        3: "Reservation decision",
        4: "Plan apply",
        5: "Signal change",
        6: "EV pass",
    }
    xs: List[float] = []
    ys: List[int] = []
    for r in timeline_rows:
        et = str(r.get("event_type", "") or "")
        if et in lane_map:
            xs.append(_f(r.get("sim_time")) or 0.0)
            ys.append(lane_map[et])

    if xs:
        fig, ax = plt.subplots(figsize=FIG_43)
        ax.scatter(xs, ys, s=18, alpha=0.8)
        ax.set_title("Route Coordination Timeline")
        ax.set_xlabel("Simulation time (s)")
        ax.set_yticks(sorted(labels.keys()))
        ax.set_yticklabels([labels[k] for k in sorted(labels.keys())])
        ax.grid(axis="x", linestyle="--", alpha=0.35)
        _save(fig, out_dir / "coordination_timeline.png")

    # EV speed profile
    if speed_rows:
        fig, ax = plt.subplots(figsize=FIG_43)
        t = [_f(r.get("sim_time")) or 0.0 for r in speed_rows]
        v = [_f(r.get("speed_mps")) or 0.0 for r in speed_rows]
        ax.plot(t, v)
        ax.set_title("EV Speed Profile (from ev.request.in)")
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.grid(True, linestyle="--", alpha=0.35)
        _save(fig, out_dir / "ev_speed_profile.png")


def _critical_intersection_timeline(timeline_rows: List[dict], out_dir: Path, window_sec: float = 60.0) -> None:
    # Option 1: one critical intersection interaction timeline.
    if not timeline_rows:
        return
    rows = [r for r in timeline_rows if _f(r.get("sim_time")) is not None]
    if not rows:
        return
    # Prefer F2 rows for coordination narrative.
    f2_rows = [r for r in rows if str(r.get("mode", "") or "") == "F2"]
    use_rows = f2_rows if f2_rows else rows
    by_tls = Counter(str(r.get("tls_id", "") or "") for r in use_rows if str(r.get("tls_id", "") or ""))
    if not by_tls:
        return
    tls_id, _ = by_tls.most_common(1)[0]
    use_rows = [r for r in use_rows if str(r.get("tls_id", "") or "") == tls_id]
    use_rows.sort(key=lambda r: _f(r.get("sim_time")) or 0.0)

    # Keep only core coordination milestones for a cleaner IEEE-ready timeline.
    lane_map = {
        "ev.request.in": 0,
        "coord.reservation.req_out": 1,
        "coord.refine.selection_final": 2,
        "coord.reservation.req_decision": 3,
        "coord.apply.plan": 4,
    }
    labels = {
        0: "EV enters region",
        1: "CoordinationRequest (out)",
        2: "PreemptionDecision",
        3: "Neighbor decision",
        4: "Actuation apply",
    }
    pts: List[Tuple[float, int, str]] = []
    for r in use_rows:
        et = str(r.get("event_type", "") or "")
        if et in lane_map:
            pts.append((_f(r.get("sim_time")) or 0.0, lane_map[et], et))
    if not pts:
        return
    pts.sort(key=lambda x: x[0])
    # Focus view on the densest <=window_sec segment to avoid overcrowding.
    win = max(15.0, float(window_sec or 60.0))
    if pts:
        t_min = pts[0][0]
        t_max = pts[-1][0]
        span = max(0.0, t_max - t_min)
        if span > win:
            i = 0
            best_i = 0
            best_j = 0
            best_score = -1
            for j in range(len(pts)):
                tj = pts[j][0]
                while i <= j and (tj - pts[i][0]) > win:
                    i += 1
                # Favor windows with more coordination semantics than raw count.
                seg = pts[i : j + 1]
                score = 0
                for _t, _lane, et in seg:
                    if et in {"coord.reservation.req_out", "coord.refine.selection_final", "coord.apply.plan"}:
                        score += 3
                    elif et.startswith("coord.") or et == "ev.request.in":
                        score += 2
                    else:
                        score += 1
                if score > best_score:
                    best_score = score
                    best_i = i
                    best_j = j
            pts = pts[best_i : best_j + 1]
        # If still too long due to sparse points, clip to exact window around median time.
        if pts:
            mid_t = pts[len(pts) // 2][0]
            a = mid_t - win / 2.0
            b = mid_t + win / 2.0
            pts = [p for p in pts if a <= p[0] <= b]
    if not pts:
        return
    fig, ax = plt.subplots(figsize=FIG_43)
    # Episode windows: derive from active-vs-quiet seconds so quiet periods stay visible.
    anchor_events = {
        "ev.request.in",
        "coord.reservation.req_out",
        "coord.refine.selection_final",
        "coord.reservation.req_decision",
        "coord.apply.plan",
    }
    anchor_times = [t for t, _lane, et in pts if et in anchor_events]
    episodes: List[Tuple[float, float]] = []
    if anchor_times:
        # Build per-second occupancy and split by true quiet gaps.
        sec_counts: Dict[int, int] = defaultdict(int)
        for t in anchor_times:
            sec_counts[int(round(t))] += 1
        active_secs = sorted(sec_counts.keys())
        if active_secs:
            clusters: List[Tuple[int, int]] = []
            c_start = active_secs[0]
            c_prev = active_secs[0]
            for s in active_secs[1:]:
                # New episode when we have at least one fully quiet second between activity.
                if (s - c_prev) > 1:
                    clusters.append((c_start, c_prev))
                    c_start = s
                c_prev = s
            clusters.append((c_start, c_prev))
            # Drop tiny one-point noise clusters.
            clusters = [c for c in clusters if (c[1] - c[0] + 1) >= 2]
            pre_pad = 0.5
            post_pad = 0.5
            for a, b in clusters:
                episodes.append((float(a) - pre_pad, float(b) + post_pad))
    for i, (a, b) in enumerate(episodes):
        color = "#dbe9f6" if i % 2 == 0 else "#e8f5e9"
        ax.axvspan(a, b, color=color, alpha=0.18, zorder=0)
        ax.text(
            (a + b) / 2.0,
            max(labels.keys()) + 0.35,
            f"Coordination Episode {i+1}",
            ha="center",
            va="bottom",
            fontsize=11,
            color="#333333",
        )
    xs = [t for t, _lane, _et in pts]
    ys = [lane for _t, lane, _et in pts]
    ax.scatter(xs, ys, s=30, alpha=0.90, color="#1f77b4", zorder=3)
    ax.set_title(f"Critical Intersection Interaction Timeline ({tls_id})")
    ax.set_xlabel("Simulation time (s)")
    ax.set_yticks(sorted(labels.keys()))
    ax.set_yticklabels([labels[k] for k in sorted(labels.keys())])
    ax.set_ylim(-0.6, max(labels.keys()) + 0.8)
    ax.grid(axis="x", linestyle="--", alpha=0.30)
    _save(fig, out_dir / "critical_intersection_timeline.png")
    _save(fig, out_dir / "updated_critical_intersection_timeline_episodes.png")


def _series_mean_by_sec(rows: List[dict], value_key: str) -> Tuple[List[float], List[float]]:
    by_t: Dict[int, List[float]] = defaultdict(list)
    for r in rows:
        t = _f(r.get("sim_time"))
        v = _f(r.get(value_key))
        if t is None or v is None:
            continue
        by_t[int(round(float(t)))].append(float(v))
    if not by_t:
        return [], []
    ts = sorted(by_t.keys())
    ys = [mean(by_t[t]) for t in ts]
    return [float(t) for t in ts], ys


def _queue_evolution_downstream(queue_ts_rows: List[dict], out_dir: Path, max_nodes: int = 2) -> None:
    # Option 2: downstream queue length over time for B1/F2 in critical nodes.
    if not queue_ts_rows:
        return
    rows = [
        r
        for r in queue_ts_rows
        if str(r.get("mode", "") or "") in {"B1", "F2", "F2P", "F2D", "F2PD"}
        and str(r.get("tls_id", "") or "")
        and _f(r.get("sim_time")) is not None
    ]
    if not rows:
        return
    # Critical node score: F2 spillback-active count, then mean spillback risk.
    score_active = Counter()
    score_risk: Dict[str, List[float]] = defaultdict(list)
    for r in rows:
        tls = str(r.get("tls_id", "") or "")
        mode = str(r.get("mode", "") or "")
        if mode == "F2":
            act = _i(r.get("spillback_active"), 0) or 0
            score_active[tls] += act
            sp = _f(r.get("spillback_risk"))
            if sp is not None:
                score_risk[tls].append(sp)
    ranked = sorted(
        {str(r.get("tls_id", "") or "") for r in rows},
        key=lambda t: (score_active.get(t, 0), mean(score_risk[t]) if score_risk.get(t) else 0.0),
        reverse=True,
    )
    tls_pick = ranked[: max(1, int(max_nodes))]
    if not tls_pick:
        return

    fig, axs = plt.subplots(len(tls_pick), 1, figsize=FIG_43, sharex=False)
    if hasattr(axs, "ravel"):
        axs = list(axs.ravel())
    else:
        axs = [axs]
    for i, tls in enumerate(tls_pick):
        ax = axs[i]
        for mode in ["B1", "F2", "F2P", "F2D", "F2PD"]:
            sub = [r for r in rows if str(r.get("tls_id", "") or "") == tls and str(r.get("mode", "") or "") == mode]
            tx, vy = _series_mean_by_sec(sub, "queue_len_est_veh")
            if tx and vy:
                ax.plot(tx, vy, label=mode)
        ax.set_title(f"Queue Length Over Time ({tls})")
        ax.set_xlabel("Simulation time (s)")
        ax.set_ylabel("Queue length (veh)")
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend()
    _save(fig, out_dir / "queue_evolution_downstream_B1_F2.png")


def _fnm_overhead_plot(fnm_event_rows: List[dict], out_dir: Path) -> None:
    # H style process/traffic overhead proxy from sidecar event counts.
    by_comp_evt: Dict[Tuple[str, str], int] = defaultdict(int)
    for r in fnm_event_rows:
        comp = str(r.get("component", "") or "")
        evt = str(r.get("event", "") or "")
        cnt = _i(r.get("count"), 0) or 0
        if comp and evt:
            by_comp_evt[(comp, evt)] += cnt

    tracked = [
        ("ev", "fnm.adapter.state_pull.ok"),
        ("ev", "fnm.adapter.ev_request.publish"),
        ("intersection", "fnm.route.local_to_fed"),
        ("intersection", "fnm.delivery.local_to_fed"),
        ("intersection", "fnm.stage.local_to_fed"),
    ]
    labels = [f"{c}:{e}" for c, e in tracked]
    vals = [by_comp_evt.get((c, e), 0) for c, e in tracked]
    if not any(v > 0 for v in vals):
        return
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.bar(range(len(labels)), vals)
    ax.set_title("FNM Processing/Traffic Overhead (Event Counts)")
    ax.set_xlabel("Component:event")
    ax.set_ylabel("Count")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out_dir / "fnm_overhead_event_counts.png")


def _stacked_latency_by_route(overhead_route: List[dict], out_dir: Path, modes: List[str]) -> None:
    segs = [
        ("local_compute_ms", "Local Compute"),
        ("local_apply_ms", "Local Apply"),
        ("fnm_mediation_ms", "FNM Mediation"),
        ("coordination_req_resp_ms", "Coordination Req-Resp"),
    ]
    by_key: Dict[Tuple[str, int, str], List[dict]] = defaultdict(list)
    for r in overhead_route:
        ds = str(r.get("dataset", "") or "")
        rt = _i(r.get("route_id"))
        md = str(r.get("mode", "") or "")
        if not ds or rt is None or md not in modes:
            continue
        by_key[(ds, rt, md)].append(r)
    datasets = sorted({k[0] for k in by_key.keys()})
    for ds in datasets:
        routes = sorted({k[1] for k in by_key.keys() if k[0] == ds})
        if not routes:
            continue
        fig, ax = plt.subplots(figsize=FIG_43)
        x = list(range(len(routes)))
        width = 0.36 if len(modes) <= 2 else 0.24
        offset_base = -0.5 * width * (len(modes) - 1)

        for mi, mode in enumerate(modes):
            bottoms = [0.0] * len(routes)
            bar_x = [xi + offset_base + mi * width for xi in x]
            totals = [0.0] * len(routes)
            for seg_key, seg_label in segs:
                vals: List[float] = []
                for ri, route in enumerate(routes):
                    samples = by_key.get((ds, route, mode), [])
                    if not samples:
                        vals.append(0.0)
                        continue
                    seg_vals = [_f(s.get(seg_key), 0.0) or 0.0 for s in samples]
                    v = mean(seg_vals) if seg_vals else 0.0
                    vals.append(v)
                    totals[ri] += v
                ax.bar(bar_x, vals, width=width, bottom=bottoms, label=f"{mode}: {seg_label}" if mi == 0 else None)
                bottoms = [bottoms[i] + vals[i] for i in range(len(vals))]
            for ri, tx in enumerate(bar_x):
                if totals[ri] > 0:
                    ax.text(tx, totals[ri], f"{totals[ri]:.1f}", ha="center", va="bottom", fontsize=8, rotation=90)

        ax.set_title(f"Stacked E2E Latency Overhead by Route ({ds})")
        ax.set_xlabel("Route")
        ax.set_ylabel("Latency (ms)")
        ax.set_xticks(x)
        ax.set_xticklabels([f"R{r}" for r in routes])
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        # Build a compact segment legend once (mode-agnostic segment names).
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            unique = {}
            for h, l in zip(handles, labels):
                if not l:
                    continue
                seg_name = l.split(": ", 1)[-1]
                if seg_name not in unique:
                    unique[seg_name] = h
            ax.legend(unique.values(), unique.keys(), loc="upper right")
        _save(fig, out_dir / f"stacked_e2e_latency_by_route_{ds}.png")


def _stacked_latency_by_node(
    overhead_node: List[dict],
    out_dir: Path,
    modes: List[str],
    top_tls: int,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> None:
    segs = [
        ("local_compute_ms", "Local Compute"),
        ("local_apply_ms", "Local Apply"),
        ("fnm_mediation_ms", "FNM Mediation"),
        ("coordination_req_resp_ms", "Coordination Req-Resp"),
    ]
    filtered: List[dict] = []
    for r in overhead_node:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _i(r.get("route_id"))
        md = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "")
        if md not in modes or rt is None or not tls:
            continue
        if dataset_filter and ds != dataset_filter:
            continue
        if scenario_filter and sc != scenario_filter:
            continue
        if route_filter >= 0 and rt != route_filter:
            continue
        filtered.append(r)
    if not filtered:
        return

    # If not specified, pick the first dataset/scenario/route that has F2 rows.
    if not dataset_filter or not scenario_filter or route_filter < 0:
        chosen = None
        for r in filtered:
            if str(r.get("mode", "")) == "F2":
                chosen = (
                    str(r.get("dataset", "") or ""),
                    str(r.get("scenario_id", "") or ""),
                    _i(r.get("route_id"), -1) or -1,
                )
                break
        if chosen is None:
            rr = filtered[0]
            chosen = (
                str(rr.get("dataset", "") or ""),
                str(rr.get("scenario_id", "") or ""),
                _i(rr.get("route_id"), -1) or -1,
            )
        dataset_filter, scenario_filter, route_filter = chosen
        filtered = [
            r
            for r in filtered
            if str(r.get("dataset", "") or "") == dataset_filter
            and str(r.get("scenario_id", "") or "") == scenario_filter
            and (_i(r.get("route_id"), -1) == route_filter)
        ]
    if not filtered:
        return

    by_tls_mode: Dict[Tuple[str, str], List[dict]] = defaultdict(list)
    tls_weight: Counter = Counter()
    for r in filtered:
        tls = str(r.get("tls_id", "") or "")
        md = str(r.get("mode", "") or "")
        by_tls_mode[(tls, md)].append(r)
        tls_weight[tls] += int(_i(r.get("coord_samples"), 0) or 0) + int(_i(r.get("compute_samples"), 0) or 0)

    top_nodes = [tls for tls, _ in tls_weight.most_common(max(3, int(top_tls)))]
    if not top_nodes:
        top_nodes = sorted({str(r.get("tls_id", "") or "") for r in filtered})[: max(3, int(top_tls))]

    fig, ax = plt.subplots(figsize=FIG_43)
    x = list(range(len(top_nodes)))
    width = 0.36 if len(modes) <= 2 else 0.24
    offset_base = -0.5 * width * (len(modes) - 1)
    for mi, mode in enumerate(modes):
        bar_x = [xi + offset_base + mi * width for xi in x]
        bottoms = [0.0] * len(top_nodes)
        totals = [0.0] * len(top_nodes)
        for seg_key, seg_label in segs:
            vals: List[float] = []
            for ni, tls in enumerate(top_nodes):
                rows = by_tls_mode.get((tls, mode), [])
                if not rows:
                    vals.append(0.0)
                    continue
                seg_vals = [_f(r.get(seg_key), 0.0) or 0.0 for r in rows]
                v = mean(seg_vals) if seg_vals else 0.0
                vals.append(v)
                totals[ni] += v
            ax.bar(bar_x, vals, width=width, bottom=bottoms, label=f"{mode}: {seg_label}" if mi == 0 else None)
            bottoms = [bottoms[i] + vals[i] for i in range(len(vals))]
        for ni, tx in enumerate(bar_x):
            if totals[ni] > 0:
                ax.text(tx, totals[ni], f"{totals[ni]:.1f}", ha="center", va="bottom", fontsize=8, rotation=90)

    ax.set_title(f"Stacked E2E Latency by Node ({dataset_filter} | {scenario_filter} | route {route_filter})")
    ax.set_xlabel("TLS node")
    ax.set_ylabel("Latency (ms)")
    ax.set_xticks(x)
    ax.set_xticklabels(top_nodes, rotation=45, ha="right")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        unique = {}
        for h, l in zip(handles, labels):
            if not l:
                continue
            seg_name = l.split(": ", 1)[-1]
            if seg_name not in unique:
                unique[seg_name] = h
        ax.legend(unique.values(), unique.keys(), loc="upper right")
    _save(fig, out_dir / "stacked_e2e_latency_by_node_representative.png")


def _updated_dynamic_boundary_burst_over_time(
    burst_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> None:
    focus = _pick_focus_case(timeline_rows, burst_rows, dataset_filter, scenario_filter, route_filter)
    if focus is None:
        return
    ds, sc, rt = focus
    modes = ["B1", "F2", "F2P", "F2D", "F2PD"]
    fams = ["state", "event", "request_response", "coordination"]
    best_tls = "route_scope"

    # Select a node that best shows low-high-low burst pattern in F2.
    case_rows = [
        r
        for r in timeline_rows
        if str(r.get("dataset", "") or "") == ds
        and str(r.get("scenario_id", "") or "") == sc
        and _i(r.get("route_id")) == int(rt)
        and str(r.get("tls_id", "") or "")
        and _f(r.get("sim_time")) is not None
    ]
    f2_rows = [r for r in case_rows if str(r.get("mode", "") or "") == "F2"]

    by_tls = defaultdict(list)
    for r in f2_rows:
        by_tls[str(r.get("tls_id", "") or "")].append(r)

    best_anchor = None
    best_exit = None
    best_score = -1e18
    if f2_rows:
        for tls, rows in by_tls.items():
            rows = sorted(rows, key=lambda r: _f(r.get("sim_time"), 0.0) or 0.0)
            t_enter = None
            t_exit = None
            t_last_apply = None
            for r in rows:
                et = str(r.get("event_type", "") or "")
                ts = _f(r.get("sim_time"))
                if ts is None:
                    continue
                if et == "ev.request.in" and t_enter is None:
                    t_enter = float(ts)
                if et == "coord.apply.plan":
                    t_last_apply = float(ts)
                if et in {"ev.pass.detected", "ev.node.cross"} and t_exit is None:
                    t_exit = float(ts)
            if t_enter is None:
                continue
            if t_exit is None:
                t_exit = t_last_apply if t_last_apply is not None else (t_enter + 15.0)
            if t_exit <= t_enter:
                t_exit = t_enter + 10.0

            per_sec = defaultdict(float)
            for r in rows:
                ts = _f(r.get("sim_time"))
                if ts is None:
                    continue
                fam = _timeline_artifact_family(str(r.get("event_type", "") or ""))
                if fam in {"request_response", "coordination"}:
                    per_sec[int(round(float(ts)))] += 1.0

            pre_start = int(math.floor(t_enter - 20.0))
            pre_end = int(math.floor(t_enter - 1.0))
            mid_start = int(math.floor(t_enter))
            mid_end = int(math.ceil(t_exit))
            post_start = int(math.ceil(t_exit + 1.0))
            post_end = int(math.ceil(t_exit + 30.0))
            pre_vals = [per_sec.get(t, 0.0) for t in range(pre_start, pre_end + 1)] if pre_end >= pre_start else []
            mid_vals = [per_sec.get(t, 0.0) for t in range(mid_start, mid_end + 1)] if mid_end >= mid_start else []
            post_vals = [per_sec.get(t, 0.0) for t in range(post_start, post_end + 1)] if post_end >= post_start else []
            pre_m = mean(pre_vals) if pre_vals else 0.0
            mid_m = mean(mid_vals) if mid_vals else 0.0
            post_m = mean(post_vals) if post_vals else 0.0
            score = mid_m - max(pre_m, post_m)
            # Allow non-zero tails; prefer strongest center lift, not strict zero tails.
            if score > best_score and mid_m > 0.0:
                best_score = score
                best_tls = tls
                best_anchor = t_enter
                best_exit = t_exit

        if best_tls == "route_scope":
            # fallback to most active node in F2
            tls_counts = Counter(str(r.get("tls_id", "") or "") for r in f2_rows)
            if tls_counts:
                best_tls = tls_counts.most_common(1)[0][0]
                best_anchor = min(
                    (_f(r.get("sim_time")) for r in f2_rows if str(r.get("tls_id", "") or "") == best_tls and _f(r.get("sim_time")) is not None),
                    default=None,
                )
                best_exit = max(
                    (_f(r.get("sim_time")) for r in f2_rows if str(r.get("tls_id", "") or "") == best_tls and _f(r.get("sim_time")) is not None),
                    default=None,
                )

    # Route-level fallback markers when timeline is sparse/missing.
    if best_anchor is None:
        cand = [
            _f(r.get("sim_time"))
            for r in case_rows
            if str(r.get("mode", "") or "") == "F2" and str(r.get("event_type", "") or "") == "ev.request.in" and _f(r.get("sim_time")) is not None
        ]
        if cand:
            best_anchor = min(cand)
    if best_exit is None:
        cand = [
            _f(r.get("sim_time"))
            for r in case_rows
            if str(r.get("mode", "") or "") == "F2" and str(r.get("event_type", "") or "") in {"ev.pass.detected", "ev.node.cross", "coord.apply.plan"} and _f(r.get("sim_time")) is not None
        ]
        if cand:
            best_exit = max(cand)

    # Build per-mode per-family second-wise counts for selected node.
    by_key: Dict[Tuple[str, int, str], float] = defaultdict(float)
    secs: set = set()
    if case_rows:
        for r in case_rows:
            if best_tls != "route_scope" and str(r.get("tls_id", "") or "") != best_tls:
                continue
            md = str(r.get("mode", "") or "")
            if md not in modes:
                continue
            ts = _f(r.get("sim_time"))
            if ts is None:
                continue
            sec = int(round(float(ts)))
            fam = _timeline_artifact_family(str(r.get("event_type", "") or ""))
            if fam in fams:
                by_key[(md, sec, fam)] += 1.0
                secs.add(sec)
    if not secs:
        # Final fallback from burst CSV route-scope aggregates.
        for r in burst_rows:
            if str(r.get("dataset", "") or "") != ds:
                continue
            if str(r.get("scenario_id", "") or "") != sc:
                continue
            if _i(r.get("route_id")) != int(rt):
                continue
            md = str(r.get("mode", "") or "")
            fam = str(r.get("artifact_family", "") or "")
            sec = _i(r.get("sim_time_sec"))
            cnt = _f(r.get("count"), 0.0) or 0.0
            if md in modes and fam in fams and sec is not None:
                by_key[(md, int(sec), fam)] += cnt
                secs.add(int(sec))
    if not secs:
        return
    t_anchor = float(best_anchor) if best_anchor is not None else float(min(secs) + 20)
    t_cross = float(best_exit) if best_exit is not None else float(t_anchor + 20.0)
    sec_min, sec_max = min(secs), max(secs)
    if t_anchor is not None and t_cross is not None and t_cross > t_anchor:
        during = float(t_cross - t_anchor)
        # Rule of thirds: before 25%, during 50%, after 25%.
        # Equivalent to before ~= during/2 and after ~= during/2.
        pre = max(20.0, 0.5 * during)
        post = max(30.0, 0.5 * during)
        win_start = int(math.floor(t_anchor - pre))
        win_end = int(math.ceil(t_cross + post))
    elif t_anchor is not None:
        win_start = int(math.floor(t_anchor - 20.0))
        win_end = int(math.ceil(t_anchor + 60.0))
    else:
        win_start = sec_min
        win_end = min(sec_max, sec_min + 90)
    win_start = max(sec_min, win_start)
    win_end = min(sec_max, win_end)
    sec_axis = list(range(int(win_start), int(win_end) + 1))
    if not sec_axis:
        return

    colors = {
        "state": "#6baed6",
        "event": "#9ecae1",
        "request_response": "#fb6a4a",
        "coordination": "#cb181d",
    }
    mode_series: Dict[str, Dict[str, List[float]]] = {}
    global_max = 0.0
    for md in modes:
        fam_map: Dict[str, List[float]] = {}
        for fam in fams:
            vals = [float(by_key.get((md, sec, fam), 0.0)) for sec in sec_axis]
            fam_map[fam] = vals
        mode_series[md] = fam_map
        for i in range(len(sec_axis)):
            total_i = sum(fam_map[f][i] for f in fams)
            if total_i > global_max:
                global_max = total_i
    if global_max <= 0.0:
        return
    y_lim = max(1.0, global_max * 1.10)

    # Variant A: stacked area (shared y-scale).
    fig, axs = plt.subplots(2, 1, figsize=FIG_43, sharex=True, sharey=True)
    for i, md in enumerate(modes):
        ax = axs[i]
        ys = [mode_series[md][fam] for fam in fams]
        ax.stackplot(sec_axis, ys, labels=fams, colors=[colors[f] for f in fams], alpha=0.90)
        if t_anchor is not None:
            ax.axvline(float(t_anchor), color="black", linestyle="--", linewidth=1.0, label="EV enters region")
        if t_cross is not None:
            ax.axvline(float(t_cross), color="dimgray", linestyle="--", linewidth=1.0, label="EV exits / preemption resolved")
        ax.set_ylim(0.0, y_lim)
        ax.set_ylabel("Messages/s")
        ax.set_title(f"{md} - Runtime Federation Interactions")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
        if t_anchor is not None:
            ax.text(float(t_anchor), y_lim * 0.96, "Enter", fontsize=8, ha="left", va="top")
        if t_cross is not None:
            ax.text(float(t_cross), y_lim * 0.88, "Exit/Resolved", fontsize=8, ha="left", va="top")
    axs[-1].set_xlabel("Simulation time (s)")
    handles, labels = axs[0].get_legend_handles_labels()
    if handles:
        uniq = {}
        for h, l in zip(handles, labels):
            if l not in uniq:
                uniq[l] = h
        axs[0].legend(uniq.values(), uniq.keys(), loc="upper right", ncol=3)
    fig.suptitle(f"Dynamic Boundary Burst Over Time ({ds} | {sc} | route {rt} | node {best_tls})", fontsize=13, y=1.02)
    _save(fig, out_dir / "updated_dynamic_boundary_burst_over_time_area.png")

    # Variant B: stacked bars (same scale, clearer burst density).
    fig, axs = plt.subplots(2, 1, figsize=FIG_43, sharex=True, sharey=True)
    for i, md in enumerate(modes):
        ax = axs[i]
        bottom = [0.0] * len(sec_axis)
        for fam in fams:
            vals = mode_series[md][fam]
            ax.bar(sec_axis, vals, bottom=bottom, width=0.95, color=colors[fam], label=fam, alpha=0.95)
            bottom = [bottom[j] + vals[j] for j in range(len(vals))]
        if t_anchor is not None:
            ax.axvline(float(t_anchor), color="black", linestyle="--", linewidth=1.0, label="EV enters region")
        if t_cross is not None:
            ax.axvline(float(t_cross), color="dimgray", linestyle="--", linewidth=1.0, label="EV exits / preemption resolved")
        ax.set_ylim(0.0, y_lim)
        ax.set_ylabel("Messages/s")
        ax.set_title(f"{md} - Runtime Federation Interactions")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
        if t_anchor is not None:
            ax.text(float(t_anchor), y_lim * 0.96, "Enter", fontsize=8, ha="left", va="top")
        if t_cross is not None:
            ax.text(float(t_cross), y_lim * 0.88, "Exit/Resolved", fontsize=8, ha="left", va="top")
    axs[-1].set_xlabel("Simulation time (s)")
    handles, labels = axs[0].get_legend_handles_labels()
    if handles:
        uniq = {}
        for h, l in zip(handles, labels):
            if l not in uniq:
                uniq[l] = h
        axs[0].legend(uniq.values(), uniq.keys(), loc="upper right", ncol=3)
    fig.suptitle(f"Dynamic Boundary Burst Over Time ({ds} | {sc} | route {rt} | node {best_tls})", fontsize=13, y=1.02)
    _save(fig, out_dir / "updated_dynamic_boundary_burst_over_time_bars.png")


def _updated_stacked_e2e_latency_by_node_route(
    overhead_node: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
    modes: Optional[List[str]] = None,
) -> None:
    # Requested: per-node stacked bars in a given route, with same segment legend as route-level plot.
    if modes is None:
        modes = ["B1", "F2", "F2P", "F2D", "F2PD"]
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        # fallback from overhead rows
        for r in overhead_node:
            ds = str(r.get("dataset", "") or "")
            sc = str(r.get("scenario_id", "") or "")
            rt = _i(r.get("route_id"))
            md = str(r.get("mode", "") or "")
            if not ds or not sc or rt is None:
                continue
            if dataset_filter and ds != dataset_filter:
                continue
            if scenario_filter and sc != scenario_filter:
                continue
            if route_filter >= 0 and int(rt) != int(route_filter):
                continue
            if md not in modes:
                continue
            focus = (ds, sc, int(rt))
            break
    if focus is None:
        return
    ds, sc, rt = focus
    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode="F2")
    if not ordered_nodes:
        # fallback order by first seen in overhead rows
        seen = []
        for r in overhead_node:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and _i(r.get("route_id")) == int(rt)
            ):
                tls = str(r.get("tls_id", "") or "")
                if tls and tls not in seen:
                    seen.append(tls)
        ordered_nodes = seen
    if not ordered_nodes:
        return

    segs = [
        ("local_compute_ms", "Local Compute"),
        ("local_apply_ms", "Local Apply"),
        ("fnm_mediation_ms", "FNM Mediation"),
        ("coordination_req_resp_ms", "Coordination Req-Resp"),
    ]
    by_node_mode: Dict[Tuple[str, str], List[dict]] = defaultdict(list)
    for r in overhead_node:
        if (
            str(r.get("dataset", "") or "") != ds
            or str(r.get("scenario_id", "") or "") != sc
            or _i(r.get("route_id")) != int(rt)
        ):
            continue
        tls = str(r.get("tls_id", "") or "")
        md = str(r.get("mode", "") or "")
        if tls and md in modes:
            by_node_mode[(tls, md)].append(r)

    nodes = [n for n in ordered_nodes if any(by_node_mode.get((n, m)) for m in modes)]
    if not nodes:
        return

    seg_colors = {
        "local_compute_ms": "#1f77b4",
        "local_apply_ms": "#ff7f0e",
        "fnm_mediation_ms": "#2ca02c",
        "coordination_req_resp_ms": "#d62728",
    }
    x = list(range(len(nodes)))

    # Clean composite layout: top=B1, bottom=F2, shared y-scale.
    pane_modes = [m for m in ["B1", "F2", "F2P", "F2D", "F2PD"] if m in modes]
    if not pane_modes:
        pane_modes = modes[:]
    fig, axs = plt.subplots(len(pane_modes), 1, figsize=FIG_43, sharex=True, sharey=True)
    if hasattr(axs, "ravel"):
        axs = list(axs.ravel())
    else:
        axs = [axs]

    y_max = 0.0
    for mode in pane_modes:
        totals = []
        for node in nodes:
            rows = by_node_mode.get((node, mode), [])
            t = 0.0
            for seg_key, _ in segs:
                seg_vals = [_f(r.get(seg_key), 0.0) or 0.0 for r in rows]
                t += (mean(seg_vals) if seg_vals else 0.0)
            totals.append(t)
        if totals:
            y_max = max(y_max, max(totals))
    if y_max <= 0.0:
        y_max = 1.0

    for pi, mode in enumerate(pane_modes):
        ax = axs[pi]
        bottoms = [0.0] * len(nodes)
        totals = [0.0] * len(nodes)
        for seg_key, seg_label in segs:
            vals: List[float] = []
            for ni, node in enumerate(nodes):
                rows = by_node_mode.get((node, mode), [])
                if not rows:
                    vals.append(0.0)
                    continue
                seg_vals = [_f(r.get(seg_key), 0.0) or 0.0 for r in rows]
                v = mean(seg_vals) if seg_vals else 0.0
                vals.append(v)
                totals[ni] += v
            ax.bar(x, vals, bottom=bottoms, color=seg_colors.get(seg_key, None), label=seg_label)
            bottoms = [bottoms[i] + vals[i] for i in range(len(vals))]
        for ni, tx in enumerate(x):
            if totals[ni] > 0:
                ax.text(tx, totals[ni], f"{totals[ni]:.1f}", ha="center", va="bottom", fontsize=8, rotation=90)
        ax.set_ylim(0.0, y_max * 1.12)
        ax.set_ylabel("Latency (ms)")
        ax.set_title(f"{mode} - Stacked E2E Latency by Node")
        ax.grid(axis="y", linestyle="--", alpha=0.35)

    axs[-1].set_xlabel("Intersection sequence")
    axs[-1].set_xticks(x)
    axs[-1].set_xticklabels([f"I{i+1}\n{n}" for i, n in enumerate(nodes)])
    handles, labels = axs[0].get_legend_handles_labels()
    if handles:
        uniq = {}
        for h, l in zip(handles, labels):
            if l not in uniq:
                uniq[l] = h
        axs[0].legend(uniq.values(), uniq.keys(), loc="upper right")
    fig.suptitle(f"Stacked E2E Latency by Node ({ds} | {sc} | route {rt})", fontsize=13, y=1.02)
    _save(fig, out_dir / "updated_stacked_e2e_latency_by_node_route.png")


def _updated_e2e_coordination_stack_by_node(
    lat_rr_rows: List[dict],
    compute_rows: List[dict],
    micro_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
    threshold_ms: float = 1000.0,
) -> None:
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        # Fallback from latency/compute rows when timeline-driven focus misses.
        for r in lat_rr_rows:
            ds = str(r.get("dataset", "") or "")
            sc = str(r.get("scenario_id", "") or "")
            rt = _i(r.get("route_id"))
            md = str(r.get("mode", "") or "")
            if not ds or not sc or rt is None:
                continue
            if dataset_filter and ds != dataset_filter:
                continue
            if scenario_filter and sc != scenario_filter:
                continue
            if route_filter >= 0 and int(rt) != int(route_filter):
                continue
            if md != "F2":
                continue
            focus = (ds, sc, int(rt))
            break
    if focus is None:
        return
    ds, sc, rt = focus
    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode="F2")
    if not ordered_nodes:
        # Fallback ordering from available node ids in rr/compute for selected route.
        candidates = []
        for r in lat_rr_rows:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and _i(r.get("route_id")) == int(rt)
                and str(r.get("mode", "") or "") == "F2"
            ):
                tls = str(r.get("tls_id", "") or "")
                if tls:
                    candidates.append(tls)
        for r in compute_rows:
            if (
                str(r.get("dataset", "") or "") == ds
                and str(r.get("scenario_id", "") or "") == sc
                and _i(r.get("route_id")) == int(rt)
                and str(r.get("mode", "") or "") == "F2"
            ):
                tls = str(r.get("tls_id", "") or "")
                if tls:
                    candidates.append(tls)
        ordered_nodes = sorted(dict.fromkeys(candidates))
    if not ordered_nodes:
        return

    # Source local processing (no physical actuation, no network).
    by_node_local_compute: Dict[str, List[float]] = defaultdict(list)
    for r in compute_rows:
        if (
            str(r.get("dataset", "") or "") == ds
            and str(r.get("scenario_id", "") or "") == sc
            and _i(r.get("route_id")) == int(rt)
            and str(r.get("mode", "") or "") == "F2"
            and str(r.get("stage", "") or "") == "local_compute"
        ):
            tls = str(r.get("tls_id", "") or "")
            d = _f(r.get("duration_ms"))
            if tls and d is not None:
                by_node_local_compute[tls].append(float(d))

    by_node_schema: Dict[str, List[float]] = defaultdict(list)
    by_node_route_local: Dict[str, List[float]] = defaultdict(list)
    by_node_validation_local: Dict[str, List[float]] = defaultdict(list)
    by_node_remote_local: Dict[str, List[float]] = defaultdict(list)
    by_node_net: Dict[str, List[float]] = defaultdict(list)
    for r in micro_rows:
        if (
            str(r.get("dataset", "") or "") != ds
            or str(r.get("scenario_id", "") or "") != sc
            or _i(r.get("route_id")) != int(rt)
        ):
            continue
        node = str(r.get("node_id", "") or "")
        evt = str(r.get("event", "") or "")
        if not node:
            continue
        s_ms = _f(r.get("schema_protocol_ms"))
        ro_ms = _f(r.get("routing_orchestration_ms"))
        nt_ms = _f(r.get("network_transport_ms"))
        total_ms = _f(r.get("total_stage_ms"))
        if evt == "fnm.stage.local_to_fed":
            if s_ms is not None:
                by_node_schema[node].append(float(s_ms))
            if ro_ms is not None:
                by_node_route_local[node].append(float(ro_ms))
            if total_ms is not None:
                proxy = float(total_ms) - float(s_ms or 0.0) - float(ro_ms or 0.0)
                if proxy > 0.0:
                    by_node_validation_local[node].append(proxy)
        elif evt == "fnm.stage.fed_to_local":
            if s_ms is not None:
                by_node_remote_local[node].append(float(s_ms))
            if nt_ms is not None:
                # Treat as combined transport; split into outbound/inbound estimate.
                by_node_net[node].append(float(nt_ms))

    by_node_rr_ms: Dict[str, List[float]] = defaultdict(list)
    for r in lat_rr_rows:
        if (
            str(r.get("dataset", "") or "") == ds
            and str(r.get("scenario_id", "") or "") == sc
            and _i(r.get("route_id")) == int(rt)
            and str(r.get("mode", "") or "") == "F2"
        ):
            tls = str(r.get("tls_id", "") or "")
            lat = _f(r.get("latency_ms"))
            if tls and lat is not None:
                by_node_rr_ms[tls].append(float(lat))

    nodes = [
        n
        for n in ordered_nodes
        if (
            by_node_local_compute.get(n)
            or by_node_schema.get(n)
            or by_node_route_local.get(n)
            or by_node_remote_local.get(n)
            or by_node_rr_ms.get(n)
        )
    ]
    if not nodes:
        return

    src_local_vals: List[float] = []
    net_out_vals: List[float] = []
    remote_vals: List[float] = []
    net_in_vals: List[float] = []
    for n in nodes:
        src_local = (mean(by_node_local_compute[n]) if by_node_local_compute.get(n) else 0.0) + (
            mean(by_node_schema[n]) if by_node_schema.get(n) else 0.0
        ) + (mean(by_node_route_local[n]) if by_node_route_local.get(n) else 0.0) + (
            mean(by_node_validation_local[n]) if by_node_validation_local.get(n) else 0.0
        )
        remote_local = mean(by_node_remote_local[n]) if by_node_remote_local.get(n) else 0.0
        net_combined = mean(by_node_net[n]) if by_node_net.get(n) else 0.0
        if net_combined <= 0.0 and by_node_rr_ms.get(n):
            rr = mean(by_node_rr_ms[n])
            # Residual estimate from rr after local pieces.
            residual = max(0.0, rr - src_local - remote_local)
            net_combined = residual
        net_out = 0.5 * net_combined
        net_in = 0.5 * net_combined
        src_local_vals.append(src_local)
        net_out_vals.append(net_out)
        remote_vals.append(remote_local)
        net_in_vals.append(net_in)

    x = list(range(len(nodes)))
    labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]
    totals = [
        src_local_vals[i] + net_out_vals[i] + remote_vals[i] + net_in_vals[i]
        for i in range(len(nodes))
    ]
    y_cap = max(max(totals) * 1.15 if totals else 0.0, threshold_ms * 1.15, 200.0)

    fig, ax = plt.subplots(figsize=FIG_43)
    b0 = ax.bar(x, src_local_vals, color="#1f77b4", label="Source local processing")
    b1 = ax.bar(x, net_out_vals, bottom=src_local_vals, color="#7f7f7f", label="Network transport (outbound)")
    bottom2 = [src_local_vals[i] + net_out_vals[i] for i in range(len(nodes))]
    b2 = ax.bar(x, remote_vals, bottom=bottom2, color="#ffbf00", label="Remote neighbor processing")
    bottom3 = [bottom2[i] + remote_vals[i] for i in range(len(nodes))]
    b3 = ax.bar(x, net_in_vals, bottom=bottom3, color="#d62728", label="Network transport (inbound)")
    for i, t in enumerate(totals):
        ax.text(x[i], t, f"{t:.1f}", ha="center", va="bottom", fontsize=8)
    ax.axhline(float(threshold_ms), color="black", linestyle="--", linewidth=1.0, label="Safe Actuation Threshold")
    ax.set_title(f"E2E Coordination Latency Breakdown ({ds} | {sc} | route {rt}, F2)")
    ax.set_xlabel("Intersection sequence")
    ax.set_ylabel("End-to-end latency (ms)")
    ax.set_ylim(0.0, y_cap)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right")
    _save(fig, out_dir / "updated_e2e_coordination_stack_by_node.png")

    # Reliability variant by mode.
    by_mode_sec: Dict[str, List[float]] = defaultdict(list)
    for r in lat_rr_rows:
        if (
            str(r.get("dataset", "") or "") == ds
            and str(r.get("scenario_id", "") or "") == sc
            and _i(r.get("route_id")) == int(rt)
        ):
            md = str(r.get("mode", "") or "")
            lat = _f(r.get("latency_ms"))
            if md in {"B1", "F2", "F2P", "F2D", "F2PD"} and lat is not None:
                by_mode_sec[md].append(float(lat) / 1000.0)
    modes = [m for m in ["B1", "F2", "F2P", "F2D", "F2PD"] if by_mode_sec.get(m)]
    if modes:
        fig, ax = plt.subplots(figsize=FIG_43)
        by_mode_ms = {m: [1000.0 * v for v in by_mode_sec[m]] for m in modes}
        ax.boxplot([by_mode_ms[m] for m in modes], tick_labels=modes, showfliers=False)
        ax.axhline(float(threshold_ms), color="black", linestyle="--", linewidth=1.0, label="Safe Actuation Threshold")
        ax.set_title(f"E2E Coordination Latency Distribution ({ds} | {sc} | route {rt})")
        ax.set_xlabel("Mode")
        ax.set_ylabel("End-to-end latency (ms)")
        ax.grid(axis="y", linestyle="--", alpha=0.20)
        ax.legend(loc="upper right")
        _save(fig, out_dir / "updated_e2e_coordination_boxplot_by_mode.png")


def _updated_micro_latency_stack_by_node(
    compute_rows: List[dict],
    micro_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
    mode: str = "F2",
) -> None:
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        # Fallback: pick from compute rows directly when timeline filtering misses.
        for r in compute_rows:
            ds = str(r.get("dataset", "") or "")
            sc = str(r.get("scenario_id", "") or "")
            rt = _i(r.get("route_id"))
            md = str(r.get("mode", "") or "")
            if not ds or not sc or rt is None:
                continue
            if dataset_filter and ds != dataset_filter:
                continue
            if scenario_filter and sc != scenario_filter:
                continue
            if route_filter >= 0 and int(rt) != int(route_filter):
                continue
            if mode and md != mode:
                continue
            focus = (ds, sc, int(rt))
            break
    if focus is None:
        return
    ds, sc, rt = focus
    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode=mode)
    if not ordered_nodes:
        # Fallback ordering from compute rows by first sim_time.
        first_seen: Dict[str, float] = {}
        for r in compute_rows:
            if (
                str(r.get("dataset", "") or "") != ds
                or str(r.get("scenario_id", "") or "") != sc
                or _i(r.get("route_id")) != int(rt)
                or str(r.get("mode", "") or "") != mode
            ):
                continue
            tls = str(r.get("tls_id", "") or "")
            t = _f(r.get("sim_time"), 1e12) or 1e12
            if tls and tls not in first_seen:
                first_seen[tls] = t
        ordered_nodes = [k for k, _ in sorted(first_seen.items(), key=lambda kv: kv[1])]
    if not ordered_nodes:
        return

    by_node_local: Dict[str, List[float]] = defaultdict(list)
    for r in compute_rows:
        if str(r.get("dataset", "") or "") != ds:
            continue
        if str(r.get("scenario_id", "") or "") != sc:
            continue
        if _i(r.get("route_id")) != int(rt):
            continue
        if str(r.get("mode", "") or "") != mode:
            continue
        if str(r.get("stage", "") or "") != "local_compute":
            # Keep blue stack as pure control-logic compute only.
            continue
        tls = str(r.get("tls_id", "") or "")
        d = _f(r.get("duration_ms"))
        if tls and d is not None:
            by_node_local[tls].append(float(d))

    # Yellow stack: schema/protocol translation + local adaptation handling.
    by_node_schema_stage: Dict[str, List[float]] = defaultdict(list)
    # Red stack: local FNM routing/orchestration + validation/policy/filter checks.
    by_node_routing_stage: Dict[str, List[float]] = defaultdict(list)
    by_node_routing_route_fallback: Dict[str, List[float]] = defaultdict(list)
    by_node_validation_proxy: Dict[str, List[float]] = defaultdict(list)
    for r in micro_rows:
        if str(r.get("dataset", "") or "") != ds:
            continue
        if str(r.get("scenario_id", "") or "") != sc:
            continue
        if _i(r.get("route_id")) != int(rt):
            continue
        node = str(r.get("node_id", "") or "")
        if not node:
            continue
        evt = str(r.get("event", "") or "")
        s_ms = _f(r.get("schema_protocol_ms"))
        r_ms = _f(r.get("routing_orchestration_ms"))
        total_ms = _f(r.get("total_stage_ms"))
        if evt == "fnm.stage.local_to_fed":
            if s_ms is not None:
                by_node_schema_stage[node].append(float(s_ms))
            if r_ms is not None:
                by_node_routing_stage[node].append(float(r_ms))
            # Keep strictly local: proxy extra local checks from stage residual only.
            if total_ms is not None:
                proxy = float(total_ms) - float(s_ms or 0.0) - float(r_ms or 0.0)
                if proxy > 0.0:
                    by_node_validation_proxy[node].append(proxy)
        elif evt == "fnm.stage.fed_to_local":
            # remote_receive_to_local_invoke is local decode/adaptation handling.
            if s_ms is not None:
                by_node_schema_stage[node].append(float(s_ms))
        elif evt in {"fnm.route.local_to_fed", "fnm.route.fed_to_local"}:
            # Route-level fallback can include tiny publish-call overhead.
            # Keep only sub-ms samples so red stack remains local CPU-bound.
            if r_ms is not None:
                rv = float(r_ms)
                if 0.0 < rv <= 1.5:
                    by_node_routing_route_fallback[node].append(rv)

    nodes = [
        n
        for n in ordered_nodes
        if (by_node_local.get(n) or by_node_schema_stage.get(n) or by_node_routing_stage.get(n))
    ]
    if not nodes:
        return
    local_vals = [mean(by_node_local[n]) if by_node_local.get(n) else 0.0 for n in nodes]
    schema_vals = [mean(by_node_schema_stage[n]) if by_node_schema_stage.get(n) else 0.0 for n in nodes]
    routing_vals: List[float] = []
    for n in nodes:
        base = mean(by_node_routing_stage[n]) if by_node_routing_stage.get(n) else 0.0
        proxy = mean(by_node_validation_proxy[n]) if by_node_validation_proxy.get(n) else 0.0
        # If stage split is unavailable, use strictly tiny local fallback only.
        if base <= 0.0 and by_node_routing_route_fallback.get(n):
            base = mean(by_node_routing_route_fallback[n])
        routing_vals.append(float(base) + float(proxy))

    totals = [local_vals[i] + schema_vals[i] + routing_vals[i] for i in range(len(nodes))]
    max_total = max(totals) if totals else 0.0
    y_cap = max(10.0, max_total * 1.15)
    x = list(range(len(nodes)))
    labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]

    # Variant A: color stack.
    fig, ax = plt.subplots(figsize=FIG_43)
    b1 = ax.bar(x, local_vals, color="#1f77b4", label="Local DT execution")
    b2 = ax.bar(x, schema_vals, bottom=local_vals, color="#ffbf00", label="FNM schema/protocol")
    bottom2 = [local_vals[i] + schema_vals[i] for i in range(len(nodes))]
    b3 = ax.bar(x, routing_vals, bottom=bottom2, color="#d62728", label="FNM routing + policy/filter (local)")
    for i, t in enumerate(totals):
        ax.text(x[i], t, f"{t:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_title(f"Micro-Latency Breakdown by Route-Ordered Node ({ds} | {sc} | route {rt}, {mode})")
    ax.set_xlabel("Intersection sequence")
    ax.set_ylabel("Local processing latency (ms)")
    ax.set_ylim(0.0, y_cap)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right")
    if max(routing_vals) <= 1e-6:
        ax.text(
            0.99,
            0.02,
            "Routing/policy local overhead below logger resolution",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            color="#555555",
        )
    _save(fig, out_dir / "updated_micro_latency_stack_by_node.png")

    # Variant B: print-friendly hatched.
    fig, ax = plt.subplots(figsize=FIG_43)
    ax.bar(x, local_vals, color="#4d4d4d", edgecolor="black", label="Local DT execution", hatch="////")
    ax.bar(
        x,
        schema_vals,
        bottom=local_vals,
        color="#bdbdbd",
        edgecolor="black",
        label="FNM schema/protocol",
        hatch="\\\\\\\\",
    )
    ax.bar(
        x,
        routing_vals,
        bottom=bottom2,
        color="#f0f0f0",
        edgecolor="black",
        label="FNM routing + policy/filter (local)",
        hatch="....",
    )
    for i, t in enumerate(totals):
        ax.text(x[i], t, f"{t:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_title(f"Micro-Latency Breakdown by Route-Ordered Node ({ds} | {sc} | route {rt}, {mode})")
    ax.set_xlabel("Intersection sequence")
    ax.set_ylabel("Local processing latency (ms)")
    ax.set_ylim(0.0, y_cap)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right")
    _save(fig, out_dir / "updated_micro_latency_stack_by_node_hatched.png")

    # Variant C: compact paper-friendly split, emphasizing tiny middleware cost.
    fnm_total_vals = [schema_vals[i] + routing_vals[i] for i in range(len(nodes))]
    fig, ax = plt.subplots(figsize=(12, 4.8))
    bar_w = 0.62
    ax.bar(x, local_vals, color="#1f77b4", label="Local DT execution", width=bar_w)
    ax.bar(
        x,
        fnm_total_vals,
        bottom=local_vals,
        color="#2ca02c",
        label="FNM integration overhead",
        width=bar_w,
    )
    totals_compact = [local_vals[i] + fnm_total_vals[i] for i in range(len(nodes))]
    for i, t in enumerate(totals_compact):
        ax.text(x[i], t, f"{t:.2f}", ha="center", va="bottom", fontsize=11)
    compact_y_cap = max(1.0, (max(totals_compact) if totals_compact else 0.0) * 1.12)
    ax.set_title(f"Local Compute vs FNM Overhead (Scenario: {_scenario_label_short(sc)} - route {rt})", fontsize=16)
    ax.set_xlabel("")
    ax.set_ylabel("Mean local processing latency (ms)", fontsize=14)
    ax.set_ylim(0.0, compact_y_cap)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=13)
    ax.tick_params(axis="y", labelsize=13)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 1.01), borderaxespad=0.2, fontsize=13)
    _save(fig, out_dir / "updated_micro_latency_total_fnm_by_node.png")


def _updated_coordination_roundtrip_by_node(
    lat_rr_rows: List[dict],
    timeline_rows: List[dict],
    out_dir: Path,
    dataset_filter: str,
    scenario_filter: str,
    route_filter: int,
) -> None:
    focus = _pick_focus_case(timeline_rows, [], dataset_filter, scenario_filter, route_filter)
    if focus is None:
        return
    ds, sc, rt = focus
    ordered_nodes = _route_node_order_from_timeline(timeline_rows, ds, sc, rt, preferred_mode="F2")
    if not ordered_nodes:
        return
    by_node_mode: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    for r in lat_rr_rows:
        if str(r.get("dataset", "") or "") != ds:
            continue
        if str(r.get("scenario_id", "") or "") != sc:
            continue
        if _i(r.get("route_id")) != int(rt):
            continue
        md = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "")
        lat = _f(r.get("latency_ms"))
        if md in {"B1", "F2", "F2P", "F2D", "F2PD"} and tls and lat is not None:
            by_node_mode[(tls, md)].append(float(lat) / 1000.0)
    nodes = [n for n in ordered_nodes if by_node_mode.get((n, "B1")) or by_node_mode.get((n, "F2"))]
    if not nodes:
        return
    x = list(range(len(nodes)))
    b1_vals = [mean(by_node_mode[(n, "B1")]) if by_node_mode.get((n, "B1")) else math.nan for n in nodes]
    f2_vals = [mean(by_node_mode[(n, "F2")]) if by_node_mode.get((n, "F2")) else math.nan for n in nodes]
    labels = [f"I{i+1}\n{n}" for i, n in enumerate(nodes)]

    fig, ax = plt.subplots(figsize=FIG_43)
    width = 0.38
    ax.bar([xi - width / 2.0 for xi in x], b1_vals, width=width, color="#1f77b4", label="B1 (LIDP)")
    ax.bar([xi + width / 2.0 for xi in x], f2_vals, width=width, color="#ff7f0e", label="F2 (FCDP)")
    ax.set_title(f"Coordination Round-Trip Latency by Node ({ds} | {sc} | route {rt})")
    ax.set_xlabel("Intersection sequence")
    ax.set_ylabel("Request-response latency (s)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right")
    _save(fig, out_dir / "updated_coordination_roundtrip_by_node.png")

    fig, ax = plt.subplots(figsize=FIG_43)
    ax.plot(x, b1_vals, marker="o", linestyle="--", color="#1f77b4", label="B1 (LIDP)")
    ax.plot(x, f2_vals, marker="s", linestyle="-", color="#ff7f0e", label="F2 (FCDP)")
    ax.set_title(f"Coordination Round-Trip Latency by Node ({ds} | {sc} | route {rt})")
    ax.set_xlabel("Intersection sequence")
    ax.set_ylabel("Request-response latency (s)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", alpha=0.20)
    ax.legend(loc="upper right")
    _save(fig, out_dir / "updated_coordination_roundtrip_by_node_line.png")


def _updated_interoperability_sankey(
    proc_ratio: List[dict],
    fnm_event_rows: List[dict],
    artifact_rows: List[dict],
    out_dir: Path,
    dataset_filter: str = "",
) -> None:
    rows = [r for r in proc_ratio if (not dataset_filter or str(r.get("dataset", "") or "") == dataset_filter)]
    if dataset_filter and not rows:
        # Fallback when caller passes wrapper seed label instead of internal dataset labels.
        rows = list(proc_ratio)
    # Focus only architecture-relevant artefacts.
    rows = [r for r in rows if str(r.get("artifact_type", "") or "") in {"state", "event", "request_response", "coordination"}]
    received = sum((_f(r.get("received"), 0.0) or 0.0) for r in rows)
    translated = sum((_f(r.get("translated"), 0.0) or 0.0) for r in rows)
    accepted = sum((_f(r.get("accepted"), 0.0) or 0.0) for r in rows)
    deferred = sum((_f(r.get("deferred_rejected"), 0.0) or 0.0) for r in rows)

    # Fallback path from raw fnm_event_counts if proc_ratio is empty/zero.
    if received <= 0.0:
        evs = [
            r
            for r in fnm_event_rows
            if (not dataset_filter or str(r.get("dataset", "") or "") == dataset_filter)
        ]
        if dataset_filter and not evs:
            evs = list(fnm_event_rows)
        rec = 0.0
        trn = 0.0
        acc = 0.0
        for r in evs:
            evt = str(r.get("event", "") or "")
            cnt = float(_i(r.get("count"), 0) or 0)
            if evt.startswith("fnm.route.") or evt == "fnm.adapter.state_pull.ok":
                rec += cnt
            if evt.startswith("fnm.stage."):
                trn += cnt
            if evt.startswith("fnm.delivery.") or evt == "fnm.adapter.ev_request.publish":
                acc += cnt
        received = rec
        translated = trn if trn > 0.0 else rec
        accepted = min(acc if acc > 0.0 else translated, translated if translated > 0.0 else rec)
        deferred = max(0.0, received - accepted)

    # Last fallback from artifact volume.
    if received <= 0.0:
        arts = [
            r
            for r in artifact_rows
            if str(r.get("artifact_family", "") or "") in {"state", "event", "request_response", "coordination"}
            and (not dataset_filter or str(r.get("dataset", "") or "") == dataset_filter)
        ]
        if dataset_filter and not arts:
            arts = [
                r
                for r in artifact_rows
                if str(r.get("artifact_family", "") or "") in {"state", "event", "request_response", "coordination"}
            ]
        received = sum(float(_i(r.get("count"), 0) or 0) for r in arts)
        translated = received
        accepted = received
        deferred = 0.0

    if received <= 0.0:
        # Always emit a placeholder to avoid missing file confusion.
        fig, ax = plt.subplots(figsize=FIG_43)
        ax.text(0.5, 0.5, "No interoperability flow data available", ha="center", va="center")
        ax.set_axis_off()
        _save(fig, out_dir / "updated_interoperability_sankey.png")
        return
    # Build numerically balanced flows to avoid Sankey connection errors
    # caused by small floating mismatches across aggregated counters.
    translated = min(max(0.0, translated), max(0.0, received))
    accepted = min(max(0.0, accepted), translated)
    deferred_balanced = max(0.0, received - translated)
    translated_to_other = max(0.0, translated - accepted)

    fig, ax = plt.subplots(figsize=FIG_43)
    try:
        sankey = Sankey(ax=ax, scale=1.0 / max(received, 1.0), unit=None, gap=0.4, tolerance=1e-4)
        sankey.add(
            flows=[received, -translated, -deferred_balanced],
            labels=["Local artefacts in", "Translated", "Discarded/Kept local"],
            orientations=[0, 1, -1],
            trunklength=1.0,
            pathlengths=[0.2, 0.3, 0.3],
        )
        if translated > 1e-9:
            # Use a second (unconnected) stage to avoid strict connection constraints
            # across tiny rounded values in some matplotlib versions.
            sankey.add(
                flows=[translated, -accepted, -translated_to_other],
                labels=["Translated", "Delivered/Accepted", "Not-delivered/expired"],
                orientations=[0, 1, -1],
                trunklength=1.0,
                pathlengths=[0.25, 0.3, 0.3],
            )
        sankey.finish()
        ax.set_title("Interoperability Boundary Flow (FNM)")
    except Exception:
        # Guaranteed fallback so plot generation never breaks the pipeline.
        ax.clear()
        labels = ["Local in", "Translated", "Delivered", "Discarded/local", "Not-delivered"]
        vals = [
            float(received),
            float(translated),
            float(accepted),
            float(deferred_balanced),
            float(translated_to_other),
        ]
        ax.bar(labels, vals, color=["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756"])
        ax.set_ylabel("Count")
        ax.set_title("Interoperability Boundary Flow (FNM) - Fallback")
        ax.tick_params(axis="x", rotation=20)
        ax.grid(axis="y", linestyle="--", alpha=0.20)
    _save(fig, out_dir / "updated_interoperability_sankey.png")


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot expanded visuals from extracted CSV metrics.")
    ap.add_argument("--metrics-dir", required=True, help="Directory created by extract_expanded_fnm_visuals.py")
    ap.add_argument("--out-dir", required=True, help="Where to save generated plots.")
    ap.add_argument("--top-tls", type=int, default=10, help="Top N TLS nodes for node-level plots.")
    ap.add_argument("--latency-modes", default="B1,F2,F2P,F2D,F2PD", help="Comma-separated modes for stacked E2E latency plots.")
    ap.add_argument("--node-dataset", default="", help="Optional dataset filter for node-level stacked E2E plot.")
    ap.add_argument("--node-scenario", default="", help="Optional scenario filter for node-level stacked E2E plot.")
    ap.add_argument("--node-route", type=int, default=-1, help="Optional route filter for node-level stacked E2E plot.")
    ap.add_argument(
        "--critical-window-sec",
        type=float,
        default=60.0,
        help="Focused time window (seconds) for critical intersection timeline plot.",
    )
    args = ap.parse_args()

    metrics_dir = Path(args.metrics_dir)
    out_dir = Path(args.out_dir)
    _mkdir(out_dir)

    travel = _read_csv(metrics_dir / "travel_times.csv")
    lat_rr = _read_csv(metrics_dir / "latency_req_resp_samples.csv")
    lat_rd = _read_csv(metrics_dir / "latency_req_decision_samples.csv")
    lat_ra = _read_csv(metrics_dir / "latency_req_actuation_samples.csv")
    stale = _read_csv(metrics_dir / "staleness_samples.csv")
    queue_rows = _read_csv(metrics_dir / "queue_spillback_samples.csv")
    queue_ts_rows = _read_csv(metrics_dir / "queue_timeseries.csv")
    compute = _read_csv(metrics_dir / "compute_duration_samples.csv")
    artifact = _read_csv(metrics_dir / "artifact_volume.csv")
    artifact_node = _read_csv(metrics_dir / "artifact_volume_by_node.csv")
    artifact_burst = _read_csv(metrics_dir / "artifact_burst_timeseries.csv")
    proc_ratio = _read_csv(metrics_dir / "fnm_processing_ratio.csv")
    coord_rows = _read_csv(metrics_dir / "coordination_metrics.csv")
    summary = _read_csv(metrics_dir / "mode_route_summary.csv")
    timeline = _read_csv(metrics_dir / "timeline_events.csv")
    fnm_events = _read_csv(metrics_dir / "fnm_event_counts.csv")
    fnm_micro = _read_csv(metrics_dir / "fnm_micro_latency_samples.csv")
    overhead_route = _read_csv(metrics_dir / "e2e_overhead_segments_route.csv")
    overhead_node = _read_csv(metrics_dir / "e2e_overhead_segments_node.csv")
    latency_modes = [m.strip() for m in str(args.latency_modes or "").split(",") if m.strip()]
    if not latency_modes:
        latency_modes = ["B1", "F2", "F2P", "F2D", "F2PD"]

    if plt is None:
        _plot_svg_fallbacks(
            metrics_dir,
            out_dir,
            travel,
            artifact,
            fnm_events,
            fnm_micro,
            coord_rows,
            overhead_route,
        )
        print(f'{{"status":"ok","out_dir":"{out_dir}","backend":"svg_fallback"}}')
        return

    _grouped_travel_by_dataset(travel, out_dir)
    _boxplot_latency(lat_rr, lat_rd, lat_ra, out_dir)
    _boxplot_staleness(stale, out_dir)
    _boxplot_aoi_by_source(stale, out_dir)
    _artifact_volume_stacked(artifact, out_dir)
    _updated_artifact_volume_by_node_composite(
        artifact_node,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
    )
    _processing_ratio_grouped(proc_ratio, out_dir)
    _coordination_success(summary, out_dir)
    _coordination_success_o(coord_rows, out_dir)
    _request_to_decision_boxplot_p(lat_rd, out_dir)
    _request_to_decision_boxplot_by_node(
        lat_rd,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
    )
    _corridor_continuity_q(coord_rows, out_dir)
    _node_spillback_heatmap(queue_rows, out_dir, top_k=max(3, int(args.top_tls)))
    _queue_margin_grouped(queue_rows, out_dir, top_k=max(3, int(args.top_tls)))
    _timeline_plot(timeline, out_dir)
    _critical_intersection_timeline(timeline, out_dir, window_sec=float(args.critical_window_sec))
    _queue_evolution_downstream(queue_ts_rows, out_dir, max_nodes=2)
    _fnm_overhead_plot(fnm_events, out_dir)
    _stacked_latency_by_route(overhead_route, out_dir, modes=latency_modes)
    _stacked_latency_by_node(
        overhead_node,
        out_dir,
        modes=latency_modes,
        top_tls=max(3, int(args.top_tls)),
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
    )
    _updated_stacked_e2e_latency_by_node_route(
        overhead_node,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
        modes=latency_modes,
    )
    _updated_dynamic_boundary_burst_over_time(
        artifact_burst,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
    )
    _updated_micro_latency_stack_by_node(
        compute,
        fnm_micro,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
        mode="F2",
    )
    _updated_coordination_roundtrip_by_node(
        lat_rr,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
    )
    _updated_e2e_coordination_stack_by_node(
        lat_rr,
        compute,
        fnm_micro,
        timeline,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
        scenario_filter=str(args.node_scenario or ""),
        route_filter=int(args.node_route),
        threshold_ms=1000.0,
    )
    _updated_interoperability_sankey(
        proc_ratio,
        fnm_events,
        artifact,
        out_dir,
        dataset_filter=str(args.node_dataset or ""),
    )

    print(f'{{"status":"ok","out_dir":"{out_dir}"}}')


if __name__ == "__main__":
    main()
