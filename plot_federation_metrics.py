#!/usr/bin/env python3
"""
Plot federation metrics extracted by extract_federation_metrics.py.

Expected input directory contents:
- summary.json
- metrics_summary.csv
- latency_samples.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
from typing import Any, Dict, List, Tuple


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return raw if isinstance(raw, dict) else {}


def _read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(x: Any, default: float = 0.0) -> float:
    try:
        if x is None or x == "":
            return default
        return float(x)
    except Exception:
        return default


_INTERACTION_ALIAS: Dict[str, str] = {
    "membership": "membership_service",
    "membership_service": "membership_service",
    "catalog": "catalog_service",
    "catalog_service": "catalog_service",
    "discovery": "discovery_service",
    "discovery_service": "discovery_service",
    "metrics": "metrics_service",
    "metrics_service": "metrics_service",
    "observer": "observer_service",
    "observer_service": "observer_service",
    "intersection": "intersection_agent",
    "intersection_agent": "intersection_agent",
    "ev": "ev",
    "vehicle": "ev",
    "gtco": "gtco",
    "orchestrator": "gtco",
    "coordinator": "gtco",
    "corridor": "gtco",
    "real_world": "dt_gateway",
    "gateway": "dt_gateway",
    "dt_gateway": "dt_gateway",
    "unknown": "unknown",
}

_DT_NODES = {"ev", "intersection_agent", "gtco"}
_CONTROL_NODES = {"membership_service", "catalog_service", "discovery_service", "metrics_service", "observer_service"}
_GATEWAY_NODES = {"dt_gateway"}


def _norm_interaction_node(x: str) -> str:
    s = str(x or "").strip().lower()
    if not s:
        return "unknown"
    if s in _INTERACTION_ALIAS:
        return _INTERACTION_ALIAS[s]
    if "membership" in s:
        return "membership_service"
    if "catalog" in s:
        return "catalog_service"
    if "discovery" in s:
        return "discovery_service"
    if "metric" in s:
        return "metrics_service"
    if "observer" in s:
        return "observer_service"
    if "intersection" in s or "tls" in s:
        return "intersection_agent"
    if "corridor" in s or "gtco" in s or "orchestrator" in s or "coordinator" in s:
        return "gtco"
    if s in ("ev", "vehicle", "emergency_vehicle"):
        return "ev"
    if s in ("real_world", "gateway", "dt_gateway", "rw"):
        return "dt_gateway"
    return s


def _node_plane(x: str) -> str:
    n = _norm_interaction_node(x)
    if n in _DT_NODES:
        return "data"
    if n in _CONTROL_NODES:
        return "control"
    if n in _GATEWAY_NODES:
        return "gateway"
    return "unknown"


def _normalized_interaction_matrix(summary: Dict[str, Any], key: str = "service_interaction_counts") -> Dict[str, Dict[str, int]]:
    src_mat = dict(summary.get(key, {}) or {})
    out: Dict[str, Dict[str, int]] = {}
    for s_raw, row in src_mat.items():
        if not isinstance(row, dict):
            continue
        s = _norm_interaction_node(str(s_raw))
        for d_raw, c in row.items():
            n = int(_f(c, 0))
            if n <= 0:
                continue
            d = _norm_interaction_node(str(d_raw))
            rr = out.setdefault(s, {})
            rr[d] = rr.get(d, 0) + n
    return out


def _split_interaction_by_plane(
    mat: Dict[str, Dict[str, int]]
) -> Tuple[Dict[str, Dict[str, int]], Dict[str, Dict[str, int]], Dict[str, Dict[str, int]]]:
    dt: Dict[str, Dict[str, int]] = {}
    ctrl: Dict[str, Dict[str, int]] = {}
    cross: Dict[str, Dict[str, int]] = {}
    for s, row in mat.items():
        if not isinstance(row, dict):
            continue
        ps = _node_plane(s)
        for d, c in row.items():
            n = int(c or 0)
            if n <= 0:
                continue
            pd = _node_plane(d)
            if ps == "data" and pd == "data":
                rr = dt.setdefault(s, {})
                rr[d] = rr.get(d, 0) + n
            elif ps in ("control", "gateway") and pd in ("control", "gateway"):
                rr = ctrl.setdefault(s, {})
                rr[d] = rr.get(d, 0) + n
            else:
                rr = cross.setdefault(s, {})
                rr[d] = rr.get(d, 0) + n
    return dt, ctrl, cross


def _matrix_nodes(mat: Dict[str, Dict[str, int]]) -> List[str]:
    return sorted({str(s) for s in mat.keys()} | {str(d) for row in mat.values() if isinstance(row, dict) for d in row.keys()})


def _ordered_nodes(nodes: List[str]) -> List[str]:
    order = {"ev": 1, "intersection_agent": 2, "gtco": 3, "dt_gateway": 4, "membership_service": 5, "catalog_service": 6, "discovery_service": 7, "metrics_service": 8, "observer_service": 9, "unknown": 50}
    return sorted(nodes, key=lambda n: (order.get(n, 40), str(n)))


def _service_color(svc: str) -> str:
    s = _norm_interaction_node(svc)
    palette = {
        "ev": "#4C78A8",
        "intersection_agent": "#F58518",
        "gtco": "#54A24B",
        "dt_gateway": "#B279A2",
        "membership_service": "#72B7B2",
        "catalog_service": "#E45756",
        "discovery_service": "#EECA3B",
        "metrics_service": "#FF9DA6",
        "observer_service": "#9D755D",
        "unknown": "#BAB0AC",
    }
    return palette.get(s, "#BAB0AC")


def _topic_is_noise(topic: str) -> bool:
    t = str(topic or "").strip().lower()
    if not t:
        return True
    noise_fragments = [
        "warm_state",
        "b1_agent",
        "metrics_pub",
        "catalog/refresh",
        "membership_catalog_seen",
        "state_pub",
        "geo_svg_skip",
        "assoc_state",
        "warmup_seen",
        "perf",
    ]
    return any(f in t for f in noise_fragments)


def _filter_topic_rows(rows: List[Dict[str, Any]], min_msgs: int = 1) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for r in rows:
        d = dict(r or {})
        t = str(d.get("topic", "")).strip()
        m = int(_f(d.get("messages"), 0.0))
        if m < min_msgs:
            continue
        if _topic_is_noise(t):
            continue
        out.append(d)
    return out


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _percentile(vals: List[float], p: float) -> float:
    if not vals:
        return 0.0
    xs = sorted(float(v) for v in vals)
    if len(xs) == 1:
        return xs[0]
    idx = int(round((p / 100.0) * (len(xs) - 1)))
    idx = max(0, min(idx, len(xs) - 1))
    return xs[idx]


def _group_samples(samples: List[Dict[str, str]], metric: str) -> Dict[str, List[float]]:
    grouped: Dict[str, List[float]] = {}
    for s in samples:
        if str(s.get("metric", "")) != metric:
            continue
        tls = str(s.get("tls_id", "") or "").strip()
        if not tls:
            continue
        y = _f(s.get("latency_wall_ms"))
        if y < 0:
            continue
        grouped.setdefault(tls, []).append(y)
    return grouped


def _metric_values(samples: List[Dict[str, str]], metric: str, role: str = "") -> List[float]:
    out: List[float] = []
    role_s = str(role or "").strip().lower()
    for s in samples:
        if str(s.get("metric", "")) != metric:
            continue
        if role_s:
            r = str(s.get("role", "") or "").strip().lower()
            if r != role_s:
                continue
        y = _f(s.get("latency_wall_ms"), -1.0)
        if y >= 0:
            out.append(y)
    return out


def _plot_metric_bars(rows: List[Dict[str, str]], out_dir: str, domain: str, plt) -> None:
    data = [r for r in rows if str(r.get("domain", "")) == domain and int(_f(r.get("n", 0), 0)) > 0]
    if not data:
        return
    data.sort(key=lambda r: _f(r.get("mean_ms")), reverse=True)

    metrics = [str(r.get("metric", "")) for r in data]
    means = [_f(r.get("mean_ms")) for r in data]
    p95s = [_f(r.get("p95_ms")) for r in data]

    fig = plt.figure(figsize=(max(10, 0.7 * len(metrics)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(metrics)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(metrics, rotation=35, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title(f"Federation Metrics ({domain})")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"metrics_bar_{domain}.png"), dpi=160)
    plt.close(fig)


def _plot_event_counts(summary: Dict[str, Any], out_dir: str, plt, exclude: List[str]) -> None:
    events = dict(summary.get("event_counts", {}) or {})
    if not events:
        return
    ex = set(exclude or [])
    items = [(k, v) for k, v in events.items() if k not in ex]
    if not items:
        return
    items = sorted(items, key=lambda kv: int(kv[1]), reverse=True)[:25]
    labels = [k for k, _ in items]
    vals = [int(v) for _, v in items]

    fig = plt.figure(figsize=(max(10, 0.6 * len(labels)), 6))
    ax = fig.add_subplot(111)
    ax.bar(list(range(len(labels))), vals)
    ax.set_xticks(list(range(len(labels))))
    ax.set_xticklabels(labels, rotation=40, ha="right")
    ax.set_ylabel("Count")
    ax.set_title("Top Event Counts (Filtered)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "event_counts_top_filtered.png"), dpi=160)
    plt.close(fig)


def _plot_event_counts_by_role(summary: Dict[str, Any], out_dir: str, plt, exclude: List[str]) -> None:
    by_role = dict(summary.get("event_counts_by_role", {}) or {})
    if not by_role:
        return
    ex = set(exclude or [])
    # Build top 12 events globally from role maps
    global_counts: Dict[str, int] = {}
    for _, m in by_role.items():
        if not isinstance(m, dict):
            continue
        for evt, c in m.items():
            if evt in ex:
                continue
            global_counts[str(evt)] = global_counts.get(str(evt), 0) + int(c)
    if not global_counts:
        return
    top_events = [k for k, _ in sorted(global_counts.items(), key=lambda kv: kv[1], reverse=True)[:12]]
    roles = sorted(by_role.keys())
    if not roles or not top_events:
        return

    fig = plt.figure(figsize=(max(10, 0.8 * len(top_events)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(top_events)))
    w = 0.8 / max(1, len(roles))
    for i, role in enumerate(roles):
        role_map = by_role.get(role, {}) if isinstance(by_role.get(role, {}), dict) else {}
        vals = [int(role_map.get(evt, 0) or 0) for evt in top_events]
        offset = (i - (len(roles) - 1) / 2.0) * w
        ax.bar([x + offset for x in xs], vals, width=w, label=role)

    ax.set_xticks(xs)
    ax.set_xticklabels(top_events, rotation=35, ha="right")
    ax.set_ylabel("Count")
    ax.set_title("Event Counts by Role (Filtered)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "event_counts_by_role_filtered.png"), dpi=160)
    plt.close(fig)


def _plot_discovery_ratio(summary: Dict[str, Any], out_dir: str, plt) -> None:
    ratio = dict(summary.get("discovery_hit_ratio", {}) or {})
    total = int(ratio.get("total", 0) or 0)
    hits = int(ratio.get("hits", 0) or 0)
    if total <= 0:
        return
    misses = max(0, total - hits)

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111)
    ax.bar(["hits", "misses"], [hits, misses])
    ax.set_ylabel("Discovery Queries")
    ax.set_title(f"Discovery Hit Ratio: {hits}/{total} ({(hits/total):.2%})")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "discovery_hit_ratio.png"), dpi=160)
    plt.close(fig)


def _phase_rows(summary: Dict[str, Any]) -> List[Tuple[str, float, float, int]]:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    out: List[Tuple[str, float, float, int]] = []
    order = [
        "register_to_onboarding_ms",
        "onboarding_to_active_ms",
        "time_to_active_ms",
        "active_to_suspended_ms",
    ]
    for name in order:
        p = dict(phase.get(name, {}) or {})
        n = int(p.get("n", 0) or 0)
        if n <= 0:
            continue
        out.append((name, _f(p.get("mean_ms")), _f(p.get("p95_ms")), n))
    return out


def _plot_operational_phases(summary: Dict[str, Any], out_dir: str, plt) -> None:
    rows = _phase_rows(summary)
    if not rows:
        return
    labels = [r[0] for r in rows]
    means = [r[1] for r in rows]
    p95s = [r[2] for r in rows]
    fig = plt.figure(figsize=(max(9, 1.2 * len(labels)), 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("Execution time (ms)")
    # If active dwell dominates, use log scale to keep startup phases visible.
    nonzero = [v for v in (means + p95s) if v > 0]
    if nonzero and (max(nonzero) / max(min(nonzero), 1e-9)) > 20.0:
        ax.set_yscale("log")
        ax.set_title("Federation Operational Phases (log scale)")
    else:
        ax.set_title("Federation Operational Phases")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "operational_phases_ms.png"), dpi=160)
    plt.close(fig)


def _plot_membership_startup_phases(summary: Dict[str, Any], out_dir: str, plt) -> None:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    keys = ["register_to_onboarding_ms", "onboarding_to_active_ms", "time_to_active_ms"]
    rows: List[Tuple[str, float, float, int]] = []
    for k in keys:
        p = dict(phase.get(k, {}) or {})
        n = int(p.get("n", 0) or 0)
        if n <= 0:
            continue
        rows.append((k, _f(p.get("mean_ms")), _f(p.get("p95_ms")), n))
    if not rows:
        return
    labels = [r[0] for r in rows]
    means = [r[1] for r in rows]
    p95s = [r[2] for r in rows]
    fig = plt.figure(figsize=(max(8, 1.0 * len(labels)), 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Membership Lifecycle Startup Phases")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "membership_startup_phases_ms.png"), dpi=160)
    plt.close(fig)


def _plot_active_dwell(summary: Dict[str, Any], out_dir: str, plt) -> None:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    p = dict(phase.get("active_to_suspended_ms", {}) or {})
    n = int(p.get("n", 0) or 0)
    if n <= 0:
        return
    mean_s = _f(p.get("mean_ms")) / 1000.0
    p95_s = _f(p.get("p95_ms")) / 1000.0
    fig = plt.figure(figsize=(5, 4))
    ax = fig.add_subplot(111)
    ax.bar(["mean_s", "p95_s"], [mean_s, p95_s])
    ax.set_ylabel("Seconds")
    ax.set_title("Active Dwell (Not Onboarding Latency)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "active_dwell_seconds.png"), dpi=160)
    plt.close(fig)


def _plot_active_members_timeline(summary: Dict[str, Any], out_dir: str, plt) -> None:
    pop = dict(summary.get("federation_population", {}) or {})
    tl = list(pop.get("active_members_timeline", []) or [])
    if not tl:
        return
    pts: List[Tuple[float, float]] = []
    for r in tl:
        if not isinstance(r, dict):
            continue
        t = _f(r.get("ts_wall_ms"), -1.0)
        a = _f(r.get("active"), -1.0)
        if t < 0 or a < 0:
            continue
        pts.append((t / 1000.0, a))
    if not pts:
        return
    pts.sort(key=lambda x: x[0])
    t0 = pts[0][0]
    xs = [p[0] - t0 for p in pts]
    ys = [p[1] for p in pts]
    fig = plt.figure(figsize=(9.5, 4.5))
    ax = fig.add_subplot(111)
    ax.step(xs, ys, where="post", linewidth=2.0, color="#4C78A8")
    ax.set_xlabel("Elapsed wall time (s)")
    ax.set_ylabel("Active members")
    ax.set_title("Membership Dynamics: Active Federation Members Over Time")
    ax.grid(axis="both", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "active_members_timeline.png"), dpi=170)
    plt.close(fig)


def _plot_updated_connectivity_lifecycle_timeline(summary: Dict[str, Any], out_dir: str, plt) -> None:
    pop = dict(summary.get("federation_population", {}) or {})
    members_tl = list(pop.get("active_members_timeline", []) or [])
    assoc_tl = list(pop.get("active_associations_timeline", []) or [])

    def _series(rows: List[Dict[str, Any]], ykey: str) -> Tuple[List[float], List[float]]:
        pts: List[Tuple[float, float]] = []
        for r in rows:
            if not isinstance(r, dict):
                continue
            t_ms = _f(r.get("ts_wall_ms"), -1.0)
            y = _f(r.get(ykey), -1.0)
            if t_ms < 0 or y < 0:
                continue
            pts.append((t_ms / 1000.0, y))
        if not pts:
            return [], []
        pts.sort(key=lambda x: x[0])
        t0 = pts[0][0]
        xs = [p[0] - t0 for p in pts]
        ys = [p[1] for p in pts]
        return xs, ys

    mx, my = _series(members_tl, "active_members")
    ax_, ay_ = _series(assoc_tl, "active_associations")
    if not (mx or ax_):
        return

    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(211)
    if mx:
        ax1.step(mx, my, where="post", linewidth=2.2, color="#4C78A8", label="Active members")
        ax1.legend(loc="upper right")
    else:
        ax1.text(0.5, 0.5, "No member timeline data", ha="center", va="center")
    ax1.set_ylabel("Members")
    ax1.set_title("updated Federation Lifecycle: Membership and Connectivity Evolution")
    ax1.grid(axis="both", linestyle="--", alpha=0.3)

    ax2 = fig.add_subplot(212, sharex=ax1)
    if ax_:
        ax2.step(ax_, ay_, where="post", linewidth=2.2, color="#F58518", label="Active associations")
        ax2.legend(loc="upper right")
    else:
        ax2.text(0.5, 0.5, "No association timeline data", ha="center", va="center")
    ax2.set_xlabel("Elapsed wall time (s)")
    ax2.set_ylabel("Associations")
    ax2.grid(axis="both", linestyle="--", alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_connectivity_lifecycle_timeline.png"), dpi=180)
    plt.close(fig)


def _plot_updated_fcm_discovery_runtime(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fcm = dict(summary.get("fcm_runtime", {}) or {})
    rows = list(fcm.get("discovery_timeline", []) or [])
    if not rows:
        return
    rows = [r for r in rows if isinstance(r, dict)]
    if not rows:
        return
    rows.sort(key=lambda r: int(_f(r.get("elapsed_s"), 0.0)))
    xs = [int(_f(r.get("elapsed_s"), 0.0)) for r in rows]
    q = [int(_f(r.get("query"), 0.0)) for r in rows]
    resp = [int(_f(r.get("response"), 0.0)) for r in rows]
    upd = [int(_f(r.get("peer_update"), 0.0)) for r in rows]
    exp = [int(_f(r.get("peer_expire"), 0.0)) for r in rows]
    rej = [int(_f(r.get("peer_reject"), 0.0)) for r in rows]

    fig = plt.figure(figsize=(12, 6.5))
    ax1 = fig.add_subplot(211)
    ax1.plot(xs, q, color="#4C78A8", linewidth=1.9, label="Discovery query")
    ax1.plot(xs, resp, color="#54A24B", linewidth=1.9, label="Discovery response")
    ax1.set_ylabel("Events/s bin")
    ax1.set_title("updated FCM Discovery Runtime Activity")
    ax1.grid(axis="both", linestyle="--", alpha=0.3)
    ax1.legend(loc="upper right", ncol=2)

    ax2 = fig.add_subplot(212, sharex=ax1)
    ax2.plot(xs, upd, color="#F58518", linewidth=1.9, label="Peer-set update")
    ax2.plot(xs, exp, color="#B279A2", linewidth=1.9, label="Peer-set expire")
    ax2.plot(xs, rej, color="#E45756", linewidth=1.9, label="Peer-set reject")
    ax2.set_xlabel("Elapsed wall time (s)")
    ax2.set_ylabel("Events/s bin")
    ax2.grid(axis="both", linestyle="--", alpha=0.3)
    ax2.legend(loc="upper right", ncol=3)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_fcm_discovery_runtime_timeline.png"), dpi=180)
    plt.close(fig)


def _plot_updated_peer_connectivity_graph(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fcm = dict(summary.get("fcm_runtime", {}) or {})
    rows = list(fcm.get("peer_events", []) or [])
    if not rows:
        return

    edge_w: Dict[Tuple[str, str], int] = {}
    node_w: Dict[str, int] = {}
    for r in rows:
        if not isinstance(r, dict):
            continue
        et = str(r.get("event_type", "") or "")
        if et != "fcm.peer_set.update":
            continue
        src = str(r.get("dt_id", "") or "").strip()
        dst = str(r.get("peer_id", "") or "").strip()
        if not src or not dst:
            continue
        k = (src, dst)
        edge_w[k] = edge_w.get(k, 0) + 1
        node_w[src] = node_w.get(src, 0) + 1
        node_w[dst] = node_w.get(dst, 0) + 1

    if not edge_w:
        return

    # Keep figure readable: top nodes by degree, then edges among them.
    nodes = [n for n, _ in sorted(node_w.items(), key=lambda kv: kv[1], reverse=True)[:16]]
    if len(nodes) < 2:
        return
    nset = set(nodes)
    f_edges = [(s, d, c) for (s, d), c in edge_w.items() if s in nset and d in nset]
    if not f_edges:
        return

    # Circular layout without extra deps.
    ang_step = (2.0 * math.pi) / float(max(1, len(nodes)))
    pos: Dict[str, Tuple[float, float]] = {}
    for i, n in enumerate(nodes):
        a = i * ang_step
        pos[n] = (math.cos(a), math.sin(a))

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    max_w = max(c for _, _, c in f_edges)
    for s, d, c in f_edges:
        x1, y1 = pos[s]
        x2, y2 = pos[d]
        lw = 0.8 + (4.0 * float(c) / float(max_w or 1))
        ax.plot([x1, x2], [y1, y2], color="#72B7B2", alpha=0.45, linewidth=lw, zorder=1)

    for n in nodes:
        x, y = pos[n]
        size = 60.0 + (400.0 * float(node_w.get(n, 0)) / float(max(1, max(node_w.values()))))
        ax.scatter([x], [y], s=size, color="#4C78A8", alpha=0.9, edgecolors="white", linewidths=0.8, zorder=2)
        short = n if len(n) <= 16 else (n[:13] + "...")
        ax.text(x, y, short, ha="center", va="center", fontsize=8, color="white", zorder=3)

    ax.set_title("updated Peer Connectivity Graph (FCM update-derived)")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(-1.25, 1.25)
    ax.set_ylim(-1.25, 1.25)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_peer_connectivity_graph.png"), dpi=180)
    plt.close(fig)


def _select_ev_fcm_dt_id(peer_rows: List[Dict[str, Any]]) -> str:
    counts: Dict[str, int] = {}
    for r in peer_rows:
        if not isinstance(r, dict):
            continue
        did = str(r.get("dt_id", "") or "").strip()
        if not did:
            continue
        counts[did] = counts.get(did, 0) + 1
    if not counts:
        return ""
    pref = [d for d in counts.keys() if ("emergency" in d.lower() or d.lower().startswith("veh") or d.lower().startswith("ev"))]
    if pref:
        pref.sort(key=lambda d: counts.get(d, 0), reverse=True)
        return str(pref[0])
    non_node = [d for d in counts.keys() if not d.lower().startswith("node")]
    if non_node:
        non_node.sort(key=lambda d: counts.get(d, 0), reverse=True)
        return str(non_node[0])
    return str(max(counts.items(), key=lambda kv: kv[1])[0])


def _peer_x_value(rows: List[Dict[str, Any]]) -> Tuple[List[float], str]:
    sim_ok = 0
    for r in rows:
        xs = _f(r.get("ts_sim_s"), -1.0)
        if xs >= 0.0:
            sim_ok += 1
    use_sim = (sim_ok >= max(1, int(0.6 * len(rows))))
    out: List[float] = []
    if use_sim:
        out = [_f(r.get("ts_sim_s"), -1.0) for r in rows]
        out = [x for x in out if x >= 0.0]
        return out, "Simulation time (s)"
    t0 = min([_f(r.get("ts_wall_ms"), 0.0) for r in rows] or [0.0])
    out = [max(0.0, (_f(r.get("ts_wall_ms"), 0.0) - t0) / 1000.0) for r in rows]
    return out, "Elapsed wall time (s)"


def _plot_updated_ev_peer_selection_evolution(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fcm = dict(summary.get("fcm_runtime", {}) or {})
    rows_all = [r for r in list(fcm.get("peer_events", []) or []) if isinstance(r, dict)]
    if not rows_all:
        return
    ev_dt = _select_ev_fcm_dt_id(rows_all)
    if not ev_dt:
        return
    rows = [r for r in rows_all if str(r.get("dt_id", "") or "").strip() == ev_dt]
    if not rows:
        return
    rows.sort(key=lambda r: (_f(r.get("ts_wall_ms"), 0.0), _f(r.get("ts_sim_s"), -1.0)))
    x_label_mode = "sim"
    sim_ok = sum(1 for r in rows if _f(r.get("ts_sim_s"), -1.0) >= 0.0)
    if sim_ok < max(1, int(0.6 * len(rows))):
        x_label_mode = "wall"
    t0_ms = min([_f(r.get("ts_wall_ms"), 0.0) for r in rows] or [0.0])

    def _x(r: Dict[str, Any]) -> float:
        if x_label_mode == "sim":
            return _f(r.get("ts_sim_s"), -1.0)
        return max(0.0, (_f(r.get("ts_wall_ms"), 0.0) - t0_ms) / 1000.0)

    events: List[Tuple[float, str, str]] = []
    for r in rows:
        et = str(r.get("event_type", "") or "")
        if et not in {"fcm.peer_set.update", "fcm.peer_set.expire", "fcm.peer_set.reject"}:
            continue
        pid = str(r.get("peer_id", "") or "").strip()
        if not pid:
            continue
        xv = _x(r)
        if xv < 0:
            continue
        events.append((float(xv), et, pid))
    if not events:
        return
    events.sort(key=lambda z: z[0])

    # active peer count evolution + interval reconstruction
    active: set[str] = set()
    x_count: List[float] = []
    y_count: List[int] = []
    open_start: Dict[str, float] = {}
    intervals: Dict[str, List[Tuple[float, float]]] = {}
    upd_x: List[float] = []
    upd_p: List[str] = []
    exp_x: List[float] = []
    exp_p: List[str] = []
    rej_x: List[float] = []
    rej_p: List[str] = []
    for xv, et, pid in events:
        if et == "fcm.peer_set.update":
            if pid not in active:
                active.add(pid)
            if pid not in open_start:
                open_start[pid] = xv
            upd_x.append(xv)
            upd_p.append(pid)
        elif et == "fcm.peer_set.expire":
            if pid in active:
                active.discard(pid)
            st = open_start.pop(pid, None)
            if st is not None:
                intervals.setdefault(pid, []).append((st, max(0.001, xv - st)))
            exp_x.append(xv)
            exp_p.append(pid)
        else:
            rej_x.append(xv)
            rej_p.append(pid)
        x_count.append(xv)
        y_count.append(len(active))
    x_max = max(x_count) if x_count else 0.0
    for pid, st in list(open_start.items()):
        intervals.setdefault(pid, []).append((st, max(0.001, x_max - st)))

    # CSV timeline
    csv_path = os.path.join(out_dir, "updated_ev_peer_set_timeline.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["dt_id", "x_time", "event_type", "peer_id", "active_peer_count_after_event"])
        active2: set[str] = set()
        for xv, et, pid in events:
            if et == "fcm.peer_set.update":
                active2.add(pid)
            elif et == "fcm.peer_set.expire":
                active2.discard(pid)
            w.writerow([ev_dt, f"{xv:.6f}", et, pid, int(len(active2))])

    # Plot 1: active peer count over time
    fig = plt.figure(figsize=(11.5, 4.8))
    ax = fig.add_subplot(111)
    ax.step(x_count, y_count, where="post", linewidth=2.2, color="#1f77b4", label=f"{ev_dt} active peers")
    if upd_x:
        ax.scatter(upd_x, [y_count[min(i, len(y_count)-1)] for i in range(len(upd_x))], s=18, c="#2ca02c", marker="^", alpha=0.8, label="update")
    if exp_x:
        ax.scatter(exp_x, [0.0] * len(exp_x), s=20, c="#d62728", marker="x", alpha=0.8, label="expire")
    ax.set_xlabel("Simulation time (s)" if x_label_mode == "sim" else "Elapsed wall time (s)")
    ax.set_ylabel("Active peer set size")
    ax.set_title(f"updated EV Peer Selection Evolution ({ev_dt})")
    ax.grid(axis="both", linestyle="--", alpha=0.3)
    ax.legend(loc="upper right", ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_ev_peer_set_size_over_time.png"), dpi=180)
    plt.close(fig)

    # Plot 2: per-peer activity timeline (interval bars + event markers)
    peers = sorted(set([p for p in list(intervals.keys()) + upd_p + exp_p + rej_p if p]))
    if peers:
        idx = {p: i for i, p in enumerate(peers)}
        fig = plt.figure(figsize=(12.0, max(4.0, 0.42 * len(peers) + 1.8)))
        ax = fig.add_subplot(111)
        for p in peers:
            i = idx[p]
            segs = intervals.get(p, [])
            if segs:
                ax.broken_barh(segs, (i - 0.35, 0.7), facecolors="#4C78A8", alpha=0.55)
        if upd_x:
            ax.scatter(upd_x, [idx.get(p, -1) for p in upd_p], c="#2ca02c", s=22, marker="^", alpha=0.85, label="peer update")
        if exp_x:
            ax.scatter(exp_x, [idx.get(p, -1) for p in exp_p], c="#d62728", s=24, marker="x", alpha=0.85, label="peer expire")
        if rej_x:
            ax.scatter(rej_x, [idx.get(p, -1) for p in rej_p], c="#ff7f0e", s=16, marker="o", alpha=0.65, label="peer reject")
        ax.set_xlabel("Simulation time (s)" if x_label_mode == "sim" else "Elapsed wall time (s)")
        ax.set_ylabel("Peer DT")
        ax.set_yticks(list(range(len(peers))))
        ax.set_yticklabels(peers)
        ax.set_title(f"updated EV Peer Activity Timeline ({ev_dt})")
        ax.grid(axis="x", linestyle="--", alpha=0.25)
        ax.legend(loc="upper right", ncol=3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "updated_ev_peer_activity_timeline.png"), dpi=180)
        plt.close(fig)

        # Plot 3: topology snapshots at quartiles of time.
        xs = [e[0] for e in events]
        x_min = min(xs)
        x_max = max(xs)
        if x_max > x_min:
            probes = [x_min + q * (x_max - x_min) for q in (0.1, 0.35, 0.65, 0.9)]
            fig = plt.figure(figsize=(12, 9))
            for k, t_probe in enumerate(probes, start=1):
                active_k: set[str] = set()
                for xv, et, pid in events:
                    if xv > t_probe:
                        break
                    if et == "fcm.peer_set.update":
                        active_k.add(pid)
                    elif et == "fcm.peer_set.expire":
                        active_k.discard(pid)
                ax = fig.add_subplot(2, 2, k)
                ax.scatter([0.0], [0.0], s=520, c="#1f77b4", edgecolors="white", linewidths=1.0, zorder=3)
                ax.text(0.0, 0.0, ev_dt, ha="center", va="center", color="white", fontsize=9, zorder=4)
                peers_k = sorted(active_k)
                n = len(peers_k)
                if n > 0:
                    for i, p in enumerate(peers_k):
                        a = (2.0 * math.pi * i) / float(n)
                        x = 1.0 * math.cos(a)
                        y = 1.0 * math.sin(a)
                        ax.plot([0.0, x], [0.0, y], color="#72B7B2", alpha=0.65, linewidth=1.5, zorder=1)
                        ax.scatter([x], [y], s=240, c="#54A24B", edgecolors="white", linewidths=0.8, zorder=2)
                        ax.text(x, y, p, ha="center", va="center", fontsize=8, color="white", zorder=3)
                ax.set_title(f"t={t_probe:.1f}s, active={len(peers_k)}")
                ax.set_xlim(-1.4, 1.4)
                ax.set_ylim(-1.4, 1.4)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_aspect("equal")
            fig.suptitle("updated EV Peer Topology Snapshots (FCM peer set)", fontsize=13)
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            fig.savefig(os.path.join(out_dir, "updated_ev_peer_topology_snapshots.png"), dpi=180)
            plt.close(fig)


def _plot_updated_adaptive_binding_evolution(summary: Dict[str, Any], out_dir: str, plt) -> None:
    ac = dict(summary.get("adaptive_connectivity_runtime", {}) or {})
    rows = list(ac.get("binding_events", []) or [])
    if not rows:
        return
    rows = [dict(r or {}) for r in rows]
    rows.sort(key=lambda r: (_f(r.get("ts_wall_ms"), 0.0), _f(r.get("ts_sim_s"), -1.0)))

    sim_ok = sum(1 for r in rows if _f(r.get("ts_sim_s"), -1.0) >= 0.0)
    use_sim = sim_ok >= max(5, int(0.3 * len(rows)))

    def _x_val(r: Dict[str, Any]) -> float:
        if use_sim:
            return _f(r.get("ts_sim_s"), -1.0)
        return _f(r.get("ts_wall_ms"), 0.0) / 1000.0

    x0 = None
    active_keys = set()
    x_hist: List[float] = []
    y_active: List[int] = []
    by_purpose_active: Dict[str, set] = {}
    by_purpose_hist: Dict[str, List[int]] = {}
    kept_rows: List[Dict[str, Any]] = []

    for r in rows:
        et = str(r.get("event_type", "") or "")
        if et not in {
            "adaptive.connectivity.binding_set.update",
            "adaptive.connectivity.binding_set.expire",
        }:
            continue
        x = _x_val(r)
        if x < 0:
            continue
        if x0 is None:
            x0 = x
        src = str(r.get("src", "") or "").strip()
        dst = str(r.get("dst", "") or "").strip()
        purpose = str(r.get("purpose", "") or "unknown").strip() or "unknown"
        key = (src, dst, purpose)
        if et == "adaptive.connectivity.binding_set.update":
            active_keys.add(key)
            by_purpose_active.setdefault(purpose, set()).add(key)
        else:
            active_keys.discard(key)
            by_purpose_active.setdefault(purpose, set()).discard(key)
        kept_rows.append(r)

        xx = float(x - (x0 or 0.0))
        x_hist.append(xx)
        y_active.append(len(active_keys))
        for p in list(by_purpose_active.keys()):
            by_purpose_hist.setdefault(p, []).append(len(by_purpose_active.get(p, set())))

    if not x_hist:
        return

    fig = plt.figure(figsize=(11.0, 4.8))
    ax = fig.add_subplot(111)
    ax.step(x_hist, y_active, where="post", linewidth=2.2, color="#2ca02c")
    ax.set_xlabel("Simulation time (s)" if use_sim else "Elapsed wall time (s)")
    ax.set_ylabel("Active bindings")
    ax.set_title("Updated: Adaptive Connectivity Active Bindings Over Time")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_adaptive_active_bindings_over_time.png"), dpi=180)
    plt.close(fig)

    top_purposes = sorted(by_purpose_hist.keys(), key=lambda p: max(by_purpose_hist.get(p, [0]) or [0]), reverse=True)[:6]
    if top_purposes:
        fig = plt.figure(figsize=(11.0, 5.2))
        ax = fig.add_subplot(111)
        colors = ["#1f77b4", "#ff7f0e", "#9467bd", "#8c564b", "#17becf", "#7f7f7f"]
        for i, p in enumerate(top_purposes):
            ys = by_purpose_hist.get(p, [])
            if len(ys) == len(x_hist):
                ax.step(x_hist, ys, where="post", linewidth=1.9, label=p, color=colors[i % len(colors)])
        ax.set_xlabel("Simulation time (s)" if use_sim else "Elapsed wall time (s)")
        ax.set_ylabel("Active bindings by purpose")
        ax.set_title("Updated: Adaptive Binding Dynamics by Purpose")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(loc="upper left", fontsize=9, ncol=2)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "updated_adaptive_binding_purpose_dynamics.png"), dpi=180)
        plt.close(fig)

    csv_path = os.path.join(out_dir, "updated_adaptive_binding_timeline.csv")
    with open(csv_path, "w", encoding="utf-8", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(
            [
                "t",
                "source_mode",
                "event_type",
                "src",
                "dst",
                "purpose",
                "action",
                "active_bindings_n",
                "reason",
            ]
        )
        for r in kept_rows:
            w.writerow(
                [
                    _x_val(r),
                    ("sim_s" if use_sim else "wall_s"),
                    str(r.get("event_type", "") or ""),
                    str(r.get("src", "") or ""),
                    str(r.get("dst", "") or ""),
                    str(r.get("purpose", "") or ""),
                    str(r.get("action", "") or ""),
                    int(_f(r.get("active_bindings_n"), 0.0)),
                    str(r.get("reason", "") or ""),
                ]
            )


def _plot_updated_coord_latency_clock_split(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fnm = dict(summary.get("fnm_integration", {}) or {})
    pipe = dict(fnm.get("latency_pipeline_ms", {}) or {})
    wall = dict(pipe.get("request_to_response_latency_ms", {}) or {})
    sim = dict(pipe.get("request_to_response_latency_sim_ms", {}) or {})
    gap = dict(pipe.get("request_to_response_wait_gap_ms", {}) or {})

    rows = [
        ("Wall req->resp", wall),
        ("Sim req->resp", sim),
        ("Sim-Wall gap", gap),
    ]
    labels: List[str] = []
    means: List[float] = []
    p95s: List[float] = []
    ns: List[int] = []
    for lbl, st in rows:
        n = int(st.get("n", 0) or 0)
        if n <= 0:
            continue
        labels.append(lbl)
        means.append(_f(st.get("mean_ms")))
        p95s.append(_f(st.get("p95_ms")))
        ns.append(n)
    if not labels:
        return

    xs = list(range(len(labels)))
    w = 0.36
    fig = plt.figure(figsize=(10.2, 5.2))
    ax = fig.add_subplot(111)
    ax.bar([x - (w / 2.0) for x in xs], means, width=w, color="#1f77b4", label="mean_ms")
    ax.bar([x + (w / 2.0) for x in xs], p95s, width=w, color="#ff7f0e", label="p95_ms")
    for i, n in enumerate(ns):
        y = max(means[i], p95s[i])
        ax.text(float(i), float(y) + max(0.8, 0.03 * y), f"n={n}", ha="center", va="bottom", fontsize=9)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=12, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Updated: Coordination Req-Resp Clock Split")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_coord_latency_clock_split.png"), dpi=180)
    plt.close(fig)


def _plot_updated_coord_churn_queue(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fnm = dict(summary.get("fnm_integration", {}) or {})
    obs = dict(fnm.get("coordination_runtime_observability", {}) or {})
    if not obs:
        return

    def _st(name: str) -> Dict[str, Any]:
        return dict(obs.get(name, {}) or {})

    recent = _st("request_out_recent_1s_sim")
    pending = _st("request_out_pending_n")
    outbox = _st("request_outbox_depth")
    outbox_peak = _st("outbox_depth_peak")
    drain = _st("outbox_drain_n")

    labels = [
        "Req burst (1s)",
        "Pending req",
        "Outbox depth",
        "Outbox peak",
        "Drain batch",
    ]
    means = [
        _f(recent.get("mean_ms")),
        _f(pending.get("mean_ms")),
        _f(outbox.get("mean_ms")),
        _f(outbox_peak.get("mean_ms")),
        _f(drain.get("mean_ms")),
    ]
    p95s = [
        _f(recent.get("p95_ms")),
        _f(pending.get("p95_ms")),
        _f(outbox.get("p95_ms")),
        _f(outbox_peak.get("p95_ms")),
        _f(drain.get("p95_ms")),
    ]
    ns = int(obs.get("request_out_observed_n", 0) or 0)
    rep_n = int(obs.get("request_out_repeated_within_1s_n", 0) or 0)
    rep_ratio = _f(obs.get("request_out_repeated_within_1s_ratio"), -1.0)

    fig = plt.figure(figsize=(11.2, 5.4))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.36
    ax.bar([x - (w / 2.0) for x in xs], means, width=w, color="#2ca02c", label="mean")
    ax.bar([x + (w / 2.0) for x in xs], p95s, width=w, color="#d62728", alpha=0.85, label="p95")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=10, ha="right")
    ax.set_ylabel("Count / batch size")
    ax.set_title("Updated: Coordination Churn and Queue Pressure")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(loc="upper left")
    txt = f"req_out n={ns} | repeated<1s n={rep_n}"
    if rep_ratio >= 0:
        txt += f" ({100.0 * rep_ratio:.1f}%)"
    ax.text(
        0.995,
        0.98,
        txt,
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor="#cccccc"),
    )
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "updated_coord_churn_queue_pressure.png"), dpi=180)
    plt.close(fig)


def _plot_lifecycle_waterfall(summary: Dict[str, Any], out_dir: str, plt) -> None:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    keys = [
        ("register_to_onboarding_ms", "Register->Onboarding"),
        ("onboarding_to_active_ms", "Onboarding->Active"),
        ("time_to_active_ms", "Register->Active"),
    ]
    labels: List[str] = []
    medians: List[float] = []
    low_err: List[float] = []
    hi_err: List[float] = []
    for k, lbl in keys:
        st = dict(phase.get(k, {}) or {})
        n = int(st.get("n", 0) or 0)
        if n <= 0:
            continue
        med = _f(st.get("median_ms"))
        p25 = _f(st.get("p25_ms"), med)
        p75 = _f(st.get("p75_ms"), med)
        labels.append(lbl)
        medians.append(med)
        low_err.append(max(0.0, med - p25))
        hi_err.append(max(0.0, p75 - med))
    if not labels:
        return
    xs = list(range(len(labels)))
    fig = plt.figure(figsize=(max(8, 1.2 * len(labels)), 5))
    ax = fig.add_subplot(111)
    ax.bar(xs, medians, alpha=0.85, color="tab:blue")
    ax.errorbar(xs, medians, yerr=[low_err, hi_err], fmt="none", ecolor="black", capsize=5, linewidth=1.2)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Federation Lifecycle Waterfall (Median with IQR)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "lifecycle_waterfall_median_iqr.png"), dpi=160)
    plt.close(fig)


def _plot_discovery_funnel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    f = dict(summary.get("discovery_funnel", {}) or {})
    req = int(f.get("requests", 0) or 0)
    resp = int(f.get("responses", 0) or 0)
    hits = int(f.get("hits", 0) or 0)
    if req <= 0 and resp <= 0 and hits <= 0:
        return
    labels = ["queries", "responses", "hits"]
    vals = [req, resp, hits]
    fig = plt.figure(figsize=(7, 4.5))
    ax = fig.add_subplot(111)
    ax.bar(labels, vals, color=["#4C78A8", "#72B7B2", "#54A24B"])
    ax.set_ylabel("Count")
    ratio = (float(hits) / float(req)) if req > 0 else 0.0
    ax.set_title(f"Discovery Funnel (query->response->hit, hit/query={ratio:.2%})")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "discovery_funnel.png"), dpi=160)
    plt.close(fig)


def _plot_association_lifecycle(summary: Dict[str, Any], out_dir: str, plt) -> None:
    a = dict(summary.get("association_lifecycle", {}) or {})
    created = int(a.get("created", 0) or 0)
    released = int(a.get("released", 0) or 0)
    open_end = int(a.get("open_at_end", 0) or 0)
    lt = dict(a.get("lifetime_ms", {}) or {})
    med = _f(lt.get("median_ms"))
    p25 = _f(lt.get("p25_ms"), med)
    p75 = _f(lt.get("p75_ms"), med)

    fig = plt.figure(figsize=(10, 4.5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.bar(["created", "released", "open_end"], [created, released, open_end], color=["#4C78A8", "#F58518", "#E45756"])
    ax1.set_ylabel("Count")
    ax1.set_title("Association Lifecycle Counts")
    ax1.grid(axis="y", linestyle="--", alpha=0.4)

    ax2 = fig.add_subplot(1, 2, 2)
    if int(lt.get("n", 0) or 0) > 0:
        ax2.bar([0], [med], color="#54A24B")
        ax2.errorbar([0], [med], yerr=[[max(0.0, med - p25)], [max(0.0, p75 - med)]], fmt="none", ecolor="black", capsize=5)
        ax2.set_xticks([0])
        ax2.set_xticklabels(["assoc_lifetime"])
        ax2.set_ylabel("Latency (ms)")
        ax2.set_title("Association Lifetime (Median/IQR)")
        ax2.grid(axis="y", linestyle="--", alpha=0.4)
    else:
        ax2.text(0.5, 0.5, "No association lifetime samples", ha="center", va="center")
        ax2.set_axis_off()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "association_lifecycle_panel.png"), dpi=160)
    plt.close(fig)


def _plot_ev_advice_flow(summary: Dict[str, Any], out_dir: str, plt) -> None:
    e = dict(summary.get("ev_advice_flow", {}) or {})
    pub_n = int(e.get("published", 0) or 0)
    app_n = int(e.get("applied", 0) or 0)
    if pub_n <= 0 and app_n <= 0:
        return
    ratio = e.get("apply_ratio", None)
    ratio_val = float(ratio) if ratio is not None else 0.0
    uptake = dict(e.get("uptake_latency_ms", {}) or {})
    med_up = _f(uptake.get("median_ms"))
    fig = plt.figure(figsize=(9, 4.5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.bar(["advice_pub", "advice_apply"], [pub_n, app_n], color=["#4C78A8", "#54A24B"])
    ax1.set_title(f"EV Advice Flow (apply ratio={ratio_val:.2%})")
    ax1.set_ylabel("Count")
    ax1.grid(axis="y", linestyle="--", alpha=0.4)
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.bar(["uptake_median_ms"], [med_up], color="#F58518")
    ax2.set_title("EV Advice Uptake Latency")
    ax2.set_ylabel("Latency (ms)")
    ax2.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "ev_advice_flow_panel.png"), dpi=160)
    plt.close(fig)


def _plot_effectiveness_scorecard(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fe = dict(summary.get("federation_effectiveness", {}) or {})
    ratios = dict(fe.get("ratios", {}) or {})
    pairs = [
        ("reservation_req_resp_ratio", "Req->Resp"),
        ("association_closure_ratio", "Assoc Closure"),
        ("discovery_hit_query_ratio", "Discovery Hit"),
        ("route_advice_apply_ratio", "Advice Apply"),
        ("coord_tls_coverage_ratio", "TLS Coverage"),
    ]
    labels: List[str] = []
    vals: List[float] = []
    for key, lbl in pairs:
        v = ratios.get(key, None)
        if v is None:
            continue
        labels.append(lbl)
        vals.append(max(0.0, min(100.0, 100.0 * float(v))))
    if not labels:
        return

    score = fe.get("effectiveness_score_pct", None)
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    ys = list(range(len(labels)))
    ax.barh(ys, vals, color="#4C78A8")
    ax.set_yticks(ys)
    ax.set_yticklabels(labels)
    ax.set_xlim(0, 100)
    ax.set_xlabel("Success ratio (%)")
    ttl = "Middleware Effectiveness Scorecard"
    if score is not None:
        ttl += f" (overall={float(score):.1f}%)"
    ax.set_title(ttl)
    for y, v in zip(ys, vals):
        ax.text(v + 1.0, y, f"{v:.1f}%", va="center", fontsize=9)
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "middleware_effectiveness_scorecard.png"), dpi=170)
    plt.close(fig)


def _plot_role_overhead_rates(summary: Dict[str, Any], out_dir: str, plt) -> None:
    ra = dict(summary.get("role_activity", {}) or {})
    if not ra:
        return
    roles = [r for r in ("ev", "intersection", "orchestrator") if r in ra] + [r for r in sorted(ra.keys()) if r not in ("ev", "intersection", "orchestrator")]
    if not roles:
        return

    msg_rate = [_f(dict(ra.get(r, {}) or {}).get("message_rate_s"), 0.0) for r in roles]
    coord_rate = [_f(dict(ra.get(r, {}) or {}).get("coord_ms_per_s"), 0.0) for r in roles]
    comp_rate = [_f(dict(ra.get(r, {}) or {}).get("compute_ms_per_s"), 0.0) for r in roles]

    fig = plt.figure(figsize=(12, 4.8))
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.bar(roles, msg_rate, color="#4C78A8")
    ax1.set_title("Message Rate")
    ax1.set_ylabel("messages/s")
    ax1.grid(axis="y", linestyle="--", alpha=0.35)
    ax1.tick_params(axis="x", rotation=20)

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.bar(roles, coord_rate, color="#F58518")
    ax2.set_title("Coordination Cost Rate")
    ax2.set_ylabel("coordination ms/s")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)
    ax2.tick_params(axis="x", rotation=20)

    ax3 = fig.add_subplot(1, 3, 3)
    ax3.bar(roles, comp_rate, color="#54A24B")
    ax3.set_title("Computation Cost Rate")
    ax3.set_ylabel("compute ms/s")
    ax3.grid(axis="y", linestyle="--", alpha=0.35)
    ax3.tick_params(axis="x", rotation=20)

    fig.suptitle("Role-Normalized Overhead (Middleware View)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(out_dir, "role_normalized_overhead_panel.png"), dpi=170)
    plt.close(fig)


def _role_color(role: str) -> str:
    r = str(role or "").strip().lower()
    if r == "intersection":
        return "#4C78A8"
    if r == "orchestrator":
        return "#F58518"
    if r == "ev":
        return "#54A24B"
    return "#9D9DA1"


def _plot_dt_normalized_overhead(summary: Dict[str, Any], out_dir: str, plt, top_n: int = 12) -> None:
    nd = dict(summary.get("normalized_overhead_by_dt", {}) or {})
    if not nd:
        return
    rows: List[Tuple[str, Dict[str, Any]]] = []
    for dt, rec in nd.items():
        d = dict(rec or {})
        if _f(d.get("message_rate_s"), 0.0) <= 0 and _f(d.get("coord_mean_ms"), 0.0) <= 0:
            continue
        rows.append((str(dt), d))
    if not rows:
        return
    rows.sort(key=lambda kv: (_f(kv[1].get("message_rate_s"), 0.0), _f(kv[1].get("coord_mean_ms"), 0.0)), reverse=True)
    rows = rows[:top_n]

    labels = [r[0] for r in rows]
    msg_rate = [_f(r[1].get("message_rate_s"), 0.0) for r in rows]
    coord_mean = [_f(r[1].get("coord_mean_ms"), 0.0) for r in rows]
    coord_rate = [_f(r[1].get("coord_ms_per_s"), 0.0) for r in rows]
    comp_rate = [_f(r[1].get("compute_ms_per_s"), 0.0) for r in rows]
    colors = [_role_color(str(r[1].get("role", ""))) for r in rows]

    fig = plt.figure(figsize=(max(12, 0.8 * len(labels)), 8))
    ax1 = fig.add_subplot(2, 1, 1)
    xs = list(range(len(labels)))
    ax1.bar(xs, msg_rate, color=colors, alpha=0.85)
    ax1.set_ylabel("messages/s")
    ax1.set_title("Top DTs by Normalized Message Rate")
    ax1.grid(axis="y", linestyle="--", alpha=0.35)
    ax1.set_xticks(xs)
    ax1.set_xticklabels(labels, rotation=30, ha="right")
    ax1b = ax1.twinx()
    ax1b.plot(xs, coord_mean, color="black", marker="o", linewidth=1.6, label="coord mean ms")
    ax1b.set_ylabel("coord mean (ms)")
    ax1b.legend(loc="upper right", fontsize=8)

    ax2 = fig.add_subplot(2, 1, 2)
    w = 0.4
    ax2.bar([x - w / 2.0 for x in xs], coord_rate, width=w, label="coord_ms/s", color="#F58518")
    ax2.bar([x + w / 2.0 for x in xs], comp_rate, width=w, label="compute_ms/s", color="#54A24B")
    ax2.set_ylabel("ms/s")
    ax2.set_title("Normalized Coordination vs Computation Cost")
    ax2.set_xticks(xs)
    ax2.set_xticklabels(labels, rotation=30, ha="right")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "dt_normalized_overhead_panel.png"), dpi=170)
    plt.close(fig)


def _plot_dt_cost_scatter(summary: Dict[str, Any], out_dir: str, plt) -> None:
    nd = dict(summary.get("normalized_overhead_by_dt", {}) or {})
    if not nd:
        return
    pts: List[Tuple[str, float, float, str]] = []
    for dt, rec in nd.items():
        d = dict(rec or {})
        x = _f(d.get("message_rate_s"), -1.0)
        y = _f(d.get("coord_mean_ms"), -1.0)
        if x <= 0 or y <= 0:
            continue
        pts.append((str(dt), x, y, str(d.get("role", ""))))
    if len(pts) < 2:
        return

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111)
    for dt, x, y, role in pts:
        ax.scatter([x], [y], color=_role_color(role), alpha=0.75, s=35)
    # Label a few outliers by x*y score.
    for dt, x, y, _ in sorted(pts, key=lambda r: r[1] * r[2], reverse=True)[:8]:
        ax.annotate(dt, (x, y), fontsize=8)
    ax.set_xlabel("Message rate (messages/s)")
    ax.set_ylabel("Coordination mean latency (ms)")
    ax.set_title("DT Overhead Outliers: Load vs Coordination Latency")
    ax.grid(True, linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "dt_load_vs_coord_latency_scatter.png"), dpi=170)
    plt.close(fig)


def _plot_message_volume_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_role = dict(summary.get("message_volume_rates_by_role", {}) or {})
    by_service = dict(summary.get("message_volume_rates_by_service", {}) or {})
    if not by_role and not by_service:
        return

    fig = plt.figure(figsize=(12, 5.2))
    ax1 = fig.add_subplot(1, 2, 1)
    if by_role:
        roles = sorted(by_role.keys())
        msgs = [_f(dict(by_role.get(r, {}) or {}).get("messages_per_s"), 0.0) for r in roles]
        bps = [_f(dict(by_role.get(r, {}) or {}).get("bytes_per_s"), 0.0) for r in roles]
        xs = list(range(len(roles)))
        w = 0.4
        ax1.bar([x - w / 2.0 for x in xs], msgs, width=w, label="messages/s", color="#4C78A8")
        ax1.set_ylabel("messages/s")
        ax1.set_xticks(xs)
        ax1.set_xticklabels(roles, rotation=20, ha="right")
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
        ax1b = ax1.twinx()
        ax1b.bar([x + w / 2.0 for x in xs], bps, width=w, label="bytes/s", color="#F58518", alpha=0.85)
        ax1b.set_ylabel("bytes/s")
        ax1.set_title("Communication Load by Role")
    else:
        ax1.text(0.5, 0.5, "No role volume data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(1, 2, 2)
    if by_service:
        items = sorted(
            [(k, dict(v or {})) for k, v in by_service.items()],
            key=lambda kv: _f(kv[1].get("bytes_per_s"), 0.0),
            reverse=True,
        )[:8]
        labels = [k for k, _ in items]
        vals = [_f(v.get("bytes_per_s"), 0.0) for _, v in items]
        ax2.bar(labels, vals, color="#72B7B2")
        ax2.set_ylabel("bytes/s")
        ax2.set_title("Top Services by Payload Throughput")
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
        ax2.tick_params(axis="x", rotation=25)
    else:
        ax2.text(0.5, 0.5, "No service volume data", ha="center", va="center")
        ax2.set_axis_off()

    fig.suptitle("Middleware Communication Overhead (Rate-Normalized)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(out_dir, "message_volume_rate_panel.png"), dpi=170)
    plt.close(fig)


def _plot_service_interaction_matrix(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mat = _normalized_interaction_matrix(summary, "service_interaction_counts")
    if not mat:
        return
    nodes = _ordered_nodes(_matrix_nodes(mat))
    if len(nodes) < 2:
        return
    idx = {n: i for i, n in enumerate(nodes)}
    n = len(nodes)
    vals = [[0.0 for _ in range(n)] for _ in range(n)]
    for s, row in mat.items():
        if not isinstance(row, dict):
            continue
        i = idx.get(str(s))
        if i is None:
            continue
        for d, c in row.items():
            j = idx.get(str(d))
            if j is None:
                continue
            vals[i][j] = math.log1p(float(c or 0.0))

    fig = plt.figure(figsize=(max(8, 0.8 * n), max(7, 0.7 * n)))
    ax = fig.add_subplot(111)
    im = ax.imshow(vals, aspect="auto", cmap="Blues")
    ax.set_xticks(list(range(n)))
    ax.set_yticks(list(range(n)))
    ax.set_xticklabels(nodes, rotation=35, ha="right")
    ax.set_yticklabels(nodes)
    ax.set_xlabel("Destination service")
    ax.set_ylabel("Source service")
    ax.set_title("MQTT Service Interaction Matrix (log1p message count)")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="log(1+messages)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "service_interaction_matrix.png"), dpi=170)
    plt.close(fig)


def _plot_service_interaction_top_edges(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mat = _normalized_interaction_matrix(summary, "service_interaction_counts")
    if not mat:
        return
    edges: List[Tuple[str, str, int]] = []
    for s, row in mat.items():
        if not isinstance(row, dict):
            continue
        for d, c in row.items():
            n = int(c or 0)
            if n > 0:
                edges.append((str(s), str(d), n))
    if not edges:
        return
    edges.sort(key=lambda x: x[2], reverse=True)
    edges = edges[:15]
    labels = [f"{s}->{d}" for s, d, _ in edges]
    vals = [n for _, _, n in edges]
    fig = plt.figure(figsize=(max(10, 0.75 * len(labels)), 5.2))
    ax = fig.add_subplot(111)
    ax.bar(list(range(len(labels))), vals, color="#4C78A8")
    ax.set_xticks(list(range(len(labels))))
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("Message count")
    ax.set_title("Top Service Interaction Edges")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "service_interaction_top_edges.png"), dpi=170)
    plt.close(fig)


def _plot_service_interaction_planes_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    full = _normalized_interaction_matrix(summary, "service_interaction_counts")
    if not full:
        return
    dt, ctrl, cross = _split_interaction_by_plane(full)

    def _matrix_plot(ax, mat: Dict[str, Dict[str, int]], title: str) -> None:
        nodes = _ordered_nodes(_matrix_nodes(mat))
        if len(nodes) < 2:
            ax.text(0.5, 0.5, f"No {title.lower()} edges", ha="center", va="center")
            ax.set_axis_off()
            return
        idx = {n: i for i, n in enumerate(nodes)}
        m = len(nodes)
        vals = [[0.0 for _ in range(m)] for _ in range(m)]
        for s, row in mat.items():
            if not isinstance(row, dict):
                continue
            i = idx.get(str(s))
            if i is None:
                continue
            for d, c in row.items():
                j = idx.get(str(d))
                if j is None:
                    continue
                vals[i][j] = math.log1p(float(c or 0.0))
        im = ax.imshow(vals, aspect="auto", cmap="Blues")
        ax.set_xticks(list(range(m)))
        ax.set_yticks(list(range(m)))
        ax.set_xticklabels(nodes, rotation=30, ha="right", fontsize=8)
        ax.set_yticklabels(nodes, fontsize=8)
        ax.set_title(title)
        return im

    fig = plt.figure(figsize=(16, 9))
    ax1 = fig.add_subplot(2, 2, 1)
    im1 = _matrix_plot(ax1, dt, "Data Plane (DT <-> DT)")
    if im1 is not None:
        fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.03)

    ax2 = fig.add_subplot(2, 2, 2)
    im2 = _matrix_plot(ax2, ctrl, "Control Plane (Gateway/Core)")
    if im2 is not None:
        fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.03)

    ax3 = fig.add_subplot(2, 2, 3)
    edges: List[Tuple[str, str, int]] = []
    for s, row in cross.items():
        if not isinstance(row, dict):
            continue
        for d, c in row.items():
            n = int(c or 0)
            if n > 0:
                edges.append((str(s), str(d), n))
    if edges:
        edges.sort(key=lambda x: x[2], reverse=True)
        edges = edges[:12]
        labels = [f"{s}->{d}" for s, d, _ in edges]
        vals = [n for _, _, n in edges]
        ax3.bar(list(range(len(labels))), vals, color="#4C78A8")
        ax3.set_xticks(list(range(len(labels))))
        ax3.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
        ax3.set_ylabel("messages")
        ax3.set_title("Cross-Plane Edges (Top)")
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No cross-plane edges", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    totals = dict(summary.get("service_interaction_plane_totals", {}) or {})
    d = int(totals.get("data_plane_msgs", 0) or 0)
    c = int(totals.get("control_plane_msgs", 0) or 0)
    x = int(totals.get("cross_plane_msgs", 0) or 0)
    if (d + c + x) <= 0:
        d = sum(int(v or 0) for r in dt.values() for v in (r.values() if isinstance(r, dict) else []))
        c = sum(int(v or 0) for r in ctrl.values() for v in (r.values() if isinstance(r, dict) else []))
        x = sum(int(v or 0) for r in cross.values() for v in (r.values() if isinstance(r, dict) else []))
    ax4.bar(["data-plane", "control-plane", "cross-plane"], [d, c, x], color=["#4C78A8", "#72B7B2", "#F58518"])
    ax4.set_ylabel("messages")
    ax4.set_title("Interaction Mix by Plane")
    ax4.grid(axis="y", linestyle="--", alpha=0.35)

    fig.suptitle("MQTT Interaction Planes (DT vs Core Services)", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "service_interaction_planes_panel.png"), dpi=180)
    plt.close(fig)


def _plot_mqtt_topic_profile(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mt = dict(summary.get("mqtt_topics", {}) or {})
    top = _filter_topic_rows(list(mt.get("top_by_messages", []) or []), min_msgs=2)
    if not top:
        top = list(mt.get("top_by_messages", []) or [])
    if not top:
        return
    top = top[:15]
    labels = [str(r.get("topic", "")) for r in top]
    msgs = [_f(r.get("messages"), 0.0) for r in top]
    bps = [_f(r.get("bytes_per_s"), 0.0) for r in top]
    mean_b = [_f(r.get("mean_bytes_per_message"), 0.0) for r in top]

    fig = plt.figure(figsize=(16, 9))
    ax1 = fig.add_subplot(2, 1, 1)
    xs = list(range(len(labels)))
    ax1.bar(xs, msgs, color="#4C78A8")
    ax1.set_xticks(xs)
    ax1.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax1.set_ylabel("messages")
    ax1.set_title("Top MQTT Topics by Message Volume (noise-filtered)")
    ax1.grid(axis="y", linestyle="--", alpha=0.35)

    ax2 = fig.add_subplot(2, 1, 2)
    w = 0.42
    ax2.bar([x - w / 2.0 for x in xs], bps, width=w, label="bytes/s", color="#72B7B2")
    ax2.bar([x + w / 2.0 for x in xs], mean_b, width=w, label="mean bytes/msg", color="#F58518")
    ax2.set_xticks(xs)
    ax2.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax2.set_ylabel("payload")
    ax2.set_title("Topic Payload Intensity")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)
    ax2.legend()

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mqtt_topic_profile.png"), dpi=180)
    plt.close(fig)


def _plot_mqtt_topic_origin_mix(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mt = dict(summary.get("mqtt_topics", {}) or {})
    oc = dict(mt.get("origin_counts", {}) or {})
    raw_n = int(_f(oc.get("raw"), 0.0))
    drv_n = int(_f(oc.get("derived"), 0.0))
    if raw_n + drv_n <= 0:
        return
    fig = plt.figure(figsize=(6.5, 4.2))
    ax = fig.add_subplot(111)
    labels = ["raw topic", "derived topic"]
    vals = [raw_n, drv_n]
    ax.bar(labels, vals, color=["#4C78A8", "#F58518"])
    ax.set_ylabel("events")
    ratio = (float(raw_n) / float(raw_n + drv_n)) if (raw_n + drv_n) > 0 else 0.0
    ax.set_title(f"MQTT Topic Availability (raw coverage={ratio:.1%})")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mqtt_topic_origin_mix.png"), dpi=170)
    plt.close(fig)


def _plot_mqtt_service_topic_heatmap(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mat = dict(summary.get("mqtt_service_topic_matrix", {}) or {})
    if not mat:
        return
    topics = _filter_topic_rows(list((dict(summary.get("mqtt_topics", {}) or {}).get("top_by_messages", []) or [])), min_msgs=2)
    if not topics:
        topics = list((dict(summary.get("mqtt_topics", {}) or {}).get("top_by_messages", []) or []))
    topic_labels = [str(r.get("topic", "")) for r in topics if str(r.get("topic", "")) and not _topic_is_noise(str(r.get("topic", "")))]
    topic_labels = topic_labels[:15]
    if not topic_labels:
        return
    services = _ordered_nodes(sorted({_norm_interaction_node(str(s)) for s in mat.keys()}))
    if not services:
        return
    vals = [[0.0 for _ in topic_labels] for _ in services]
    for i, s in enumerate(services):
        row = dict(mat.get(s, {}) or {})
        for j, t in enumerate(topic_labels):
            vals[i][j] = math.log1p(float(_f(row.get(t), 0.0)))

    fig = plt.figure(figsize=(max(12, 0.75 * len(topic_labels)), max(5, 0.5 * len(services) + 3)))
    ax = fig.add_subplot(111)
    im = ax.imshow(vals, aspect="auto", cmap="YlGnBu")
    ax.set_xticks(list(range(len(topic_labels))))
    ax.set_yticks(list(range(len(services))))
    ax.set_xticklabels(topic_labels, rotation=35, ha="right", fontsize=8)
    ax.set_yticklabels(services, fontsize=9)
    ax.set_xlabel("MQTT topic")
    ax.set_ylabel("Source service")
    ax.set_title("MQTT Service-to-Topic Heatmap (log1p, noise-filtered)")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03, label="log(1+messages)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mqtt_service_topic_heatmap.png"), dpi=180)
    plt.close(fig)


def _plot_mqtt_topic_timeline(summary: Dict[str, Any], out_dir: str, plt) -> None:
    rows = list(summary.get("mqtt_topic_timeline", []) or [])
    if not rows:
        return
    topic_order: List[str] = []
    seen = set()
    for r in rows:
        t = str((r or {}).get("topic", "")).strip()
        if _topic_is_noise(t):
            continue
        if t and t not in seen:
            topic_order.append(t)
            seen.add(t)
    topic_order = topic_order[:12]
    if not topic_order:
        return
    idx = {t: i for i, t in enumerate(topic_order)}

    fig = plt.figure(figsize=(15, 8))
    ax = fig.add_subplot(111)
    for r in rows:
        d = dict(r or {})
        t = str(d.get("topic", "")).strip()
        if _topic_is_noise(t):
            continue
        if t not in idx:
            continue
        x = _f(d.get("t_s"), -1.0)
        y = float(idx[t])
        c = max(1.0, _f(d.get("messages"), 1.0))
        s = str(d.get("source_service", "unknown"))
        ax.scatter([x], [y], s=18.0 + min(180.0, 14.0 * math.sqrt(c)), color=_service_color(s), alpha=0.65, edgecolors="none")

    ax.set_yticks(list(range(len(topic_order))))
    ax.set_yticklabels(topic_order, fontsize=8)
    ax.set_xlabel("Elapsed wall time (s)")
    ax.set_ylabel("MQTT topic (top)")
    ax.set_title("MQTT Topic Activity Timeline (noise-filtered)")
    ax.grid(axis="x", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mqtt_topic_timeline.png"), dpi=180)
    plt.close(fig)


def _plot_mqtt_edge_topic_breakdown(summary: Dict[str, Any], out_dir: str, plt) -> None:
    rows = list(summary.get("mqtt_edge_topic_breakdown", []) or [])
    if not rows:
        return
    cleaned = []
    for r in rows:
        d = dict(r or {})
        tops = [dict(x or {}) for x in list(d.get("top_topics", []) or []) if not _topic_is_noise(str(dict(x or {}).get("topic", "")))]
        d["top_topics"] = tops
        if tops:
            cleaned.append(d)
    rows = (cleaned or rows)[:10]
    labels = [str((r or {}).get("edge", "")) for r in rows]
    totals = [_f((r or {}).get("messages"), 0.0) for r in rows]
    # Keep top 4 topic slots for stack colors.
    slot_vals = [[0.0 for _ in rows] for _ in range(4)]
    slot_lbls = [f"topic#{i+1}" for i in range(4)]
    for j, r in enumerate(rows):
        tops = list((dict(r or {}).get("top_topics", []) or []))
        for i in range(min(4, len(tops))):
            tr = dict(tops[i] or {})
            slot_vals[i][j] = _f(tr.get("messages"), 0.0)
            if j == 0:
                slot_lbls[i] = str(tr.get("topic", f"topic#{i+1}"))

    fig = plt.figure(figsize=(max(10, 0.9 * len(labels)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    bottom = [0.0 for _ in labels]
    colors = ["#4C78A8", "#F58518", "#54A24B", "#72B7B2"]
    for i in range(4):
        vals = slot_vals[i]
        if sum(vals) <= 0:
            continue
        ax.bar(xs, vals, bottom=bottom, color=colors[i], label=slot_lbls[i])
        bottom = [bottom[k] + vals[k] for k in range(len(vals))]
    residual = [max(0.0, totals[i] - bottom[i]) for i in range(len(bottom))]
    if sum(residual) > 0:
        ax.bar(xs, residual, bottom=bottom, color="#BAB0AC", label="other topics")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("messages")
    ax.set_title("MQTT Edge Composition by Topic")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "mqtt_edge_topic_breakdown.png"), dpi=180)
    plt.close(fig)


def _plot_mqtt_analysis_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mt = dict(summary.get("mqtt_topics", {}) or {})
    svc_mat = dict(summary.get("mqtt_service_topic_matrix", {}) or {})
    edge_rows = list(summary.get("mqtt_edge_topic_breakdown", []) or [])
    timeline_rows = list(summary.get("mqtt_topic_timeline", []) or [])
    top_any = _filter_topic_rows(list(mt.get("top_by_messages", []) or []), min_msgs=2)
    if not top_any:
        top_any = list(mt.get("top_by_messages", []) or [])
    if not (top_any or svc_mat or edge_rows or timeline_rows):
        return

    top = top_any[:8]
    oc = dict(mt.get("origin_counts", {}) or {})
    raw_n = int(_f(oc.get("raw"), 0.0))
    drv_n = int(_f(oc.get("derived"), 0.0))
    fig = plt.figure(figsize=(15, 9))

    ax1 = fig.add_subplot(2, 2, 1)
    if top:
        ax1.bar([str(r.get("topic", "")) for r in top], [_f(r.get("messages"), 0.0) for r in top], color="#4C78A8")
        ax1.set_title("Top Topics (raw + derived)")
        ax1.set_ylabel("messages")
        ax1.tick_params(axis="x", rotation=30)
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No topic volume data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    if svc_mat:
        svc_totals = []
        for s, row in svc_mat.items():
            svc_totals.append((str(s), int(sum(int(v or 0) for v in (dict(row or {}).values())))))
        svc_totals.sort(key=lambda kv: kv[1], reverse=True)
        svc_totals = svc_totals[:8]
        ax2.bar([k for k, _ in svc_totals], [v for _, v in svc_totals], color="#F58518")
        ax2.set_title("Messages by Producer Service")
        ax2.set_ylabel("messages")
        ax2.tick_params(axis="x", rotation=25)
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No service-topic matrix", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    if edge_rows:
        r0 = edge_rows[:8]
        ax3.bar([str(r.get("edge", "")) for r in r0], [_f(r.get("messages"), 0.0) for r in r0], color="#54A24B")
        ax3.set_title("Top Interaction Edges")
        ax3.set_ylabel("messages")
        ax3.tick_params(axis="x", rotation=30)
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No edge-topic data", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    if timeline_rows:
        tmax = max(_f((r or {}).get("t_s"), 0.0) for r in timeline_rows)
        ax4.bar(["timeline span (s)", "timeline points", "raw topics", "derived topics"], [tmax, float(len(timeline_rows)), float(raw_n), float(drv_n)], color=["#72B7B2", "#E45756", "#4C78A8", "#F58518"])
        ax4.set_title("Timeline + Topic Origin Coverage")
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        if raw_n + drv_n > 0:
            ax4.bar(["raw topics", "derived topics"], [float(raw_n), float(drv_n)], color=["#4C78A8", "#F58518"])
            ax4.set_title("Topic Origin Coverage")
            ax4.grid(axis="y", linestyle="--", alpha=0.35)
        else:
            ax4.text(0.5, 0.5, "No topic timeline", ha="center", va="center")
            ax4.set_axis_off()

    fig.suptitle("MQTT Communication Analysis Panel", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "mqtt_analysis_panel.png"), dpi=180)
    plt.close(fig)


def _plot_ev_effectiveness_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    ee = dict(summary.get("ev_effectiveness", {}) or {})
    ea = dict(summary.get("ev_advice_flow", {}) or {})
    if not ee and not ea:
        return

    req_dec = dict(ee.get("req_to_decision_ms", {}) or {})
    req_app = dict(ee.get("req_to_apply_ms", {}) or {})
    ev_srv = dict(ee.get("ev_service_tls_ms", {}) or {})
    ev_stuck = dict(ee.get("ev_stuck_episode_ms", {}) or {})
    seen_apply = dict(ea.get("seen_to_apply_latency_ms", {}) or {})

    labels = [
        "req->decision",
        "req->apply",
        "advice seen->apply",
        "EV service@TLS",
        "EV stuck episode",
    ]
    vals = [
        _f(req_dec.get("median_ms"), 0.0),
        _f(req_app.get("median_ms"), 0.0),
        _f(seen_apply.get("median_ms"), 0.0),
        _f(ev_srv.get("median_ms"), 0.0),
        _f(ev_stuck.get("median_ms"), 0.0),
    ]
    if not any(v > 0 for v in vals):
        return

    applied = int(ee.get("route_advice_applied", 0) or 0)
    seen = int(ee.get("route_advice_received", 0) or 0)
    skipped = int(ee.get("route_advice_skipped", 0) or 0)

    fig = plt.figure(figsize=(11, 5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.bar(labels, vals, color=["#4C78A8", "#72B7B2", "#54A24B", "#F58518", "#E45756"])
    ax1.set_ylabel("Median latency (ms)")
    ax1.set_title("EV-Centric E2E Timings")
    ax1.tick_params(axis="x", rotation=25)
    ax1.grid(axis="y", linestyle="--", alpha=0.35)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.bar(["advice_received", "advice_applied", "advice_skipped"], [seen, applied, skipped], color=["#4C78A8", "#54A24B", "#E45756"])
    ratio = (float(applied) / float(seen)) if seen > 0 else 0.0
    ax2.set_title(f"Route Advice Outcome (apply ratio={ratio:.1%})")
    ax2.set_ylabel("Count")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)

    fig.suptitle("EV Effectiveness and Coordination Outcome", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(out_dir, "ev_effectiveness_panel.png"), dpi=170)
    plt.close(fig)


def _plot_category_intersection_nodes_panel(summary: Dict[str, Any], samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("metrics_by_tls", {}) or {})
    comm = dict(summary.get("communication_overhead_by_tls", {}) or {})
    nd = dict(summary.get("normalized_overhead_by_dt", {}) or {})

    rows = []
    for tls, mm in by_tls.items():
        st = dict((dict(mm.get("T_coord_req_resp", {}) or {}).get("wall_ms", {}) or {}))
        n = int(st.get("n", 0) or 0)
        if n <= 0:
            continue
        rows.append((str(tls), _f(st.get("mean_ms")), _f(st.get("p95_ms")), n))
    if not rows and not comm:
        return
    rows.sort(key=lambda x: x[1], reverse=True)
    rows = rows[:8]

    fig = plt.figure(figsize=(14, 9))
    ax1 = fig.add_subplot(2, 2, 1)
    if rows:
        labels = [r[0] for r in rows]
        means = [r[1] for r in rows]
        p95 = [r[2] for r in rows]
        xs = list(range(len(labels)))
        w = 0.4
        ax1.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
        ax1.bar([x + w / 2.0 for x in xs], p95, width=w, label="p95_ms")
        ax1.set_xticks(xs)
        ax1.set_xticklabels(labels, rotation=30, ha="right")
        ax1.set_title("Top TLS Coordination Latency")
        ax1.set_ylabel("ms")
        ax1.legend(fontsize=8)
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No TLS latency data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    comm_rows = []
    for tls, c in comm.items():
        d = dict(c or {})
        total = int(
            d.get("reservation_req_sent", 0)
            + d.get("reservation_resp_recv", 0)
            + d.get("assoc_created", 0)
            + d.get("assoc_released", 0)
            + d.get("route_advice_published", 0)
            + d.get("intersection_advice_published", 0)
        )
        if total > 0:
            comm_rows.append((str(tls), d, total))
    comm_rows.sort(key=lambda x: x[2], reverse=True)
    comm_rows = comm_rows[:8]
    if comm_rows:
        labels = [r[0] for r in comm_rows]
        req = [int(r[1].get("reservation_req_sent", 0) or 0) for r in comm_rows]
        resp = [int(r[1].get("reservation_resp_recv", 0) or 0) for r in comm_rows]
        adv = [int(r[1].get("intersection_advice_published", 0) or 0) for r in comm_rows]
        xs = list(range(len(labels)))
        ax2.bar(xs, req, label="req", color="#4C78A8")
        ax2.bar(xs, resp, bottom=req, label="resp", color="#F58518")
        b2 = [req[i] + resp[i] for i in range(len(req))]
        ax2.bar(xs, adv, bottom=b2, label="advice", color="#54A24B")
        ax2.set_xticks(xs)
        ax2.set_xticklabels(labels, rotation=30, ha="right")
        ax2.set_title("TLS Communication Mix")
        ax2.set_ylabel("messages")
        ax2.legend(fontsize=8)
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No communication data", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    nd_rows = []
    for dt, rec in nd.items():
        d = dict(rec or {})
        if str(d.get("role", "")) != "intersection":
            continue
        nd_rows.append((str(dt), _f(d.get("message_rate_s")), _f(d.get("coord_ms_per_s")), _f(d.get("compute_ms_per_s"))))
    nd_rows.sort(key=lambda x: x[1], reverse=True)
    nd_rows = nd_rows[:8]
    if nd_rows:
        labels = [r[0] for r in nd_rows]
        msg = [r[1] for r in nd_rows]
        coord = [r[2] for r in nd_rows]
        comp = [r[3] for r in nd_rows]
        xs = list(range(len(labels)))
        ax3.bar(xs, msg, color="#4C78A8", alpha=0.85, label="msg/s")
        ax3.set_xticks(xs)
        ax3.set_xticklabels(labels, rotation=30, ha="right")
        ax3.set_ylabel("msg/s")
        ax3.set_title("Intersection Normalized Load")
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
        ax3b = ax3.twinx()
        ax3b.plot(xs, coord, color="#F58518", marker="o", linewidth=1.8, label="coord ms/s")
        ax3b.plot(xs, comp, color="#54A24B", marker="s", linewidth=1.4, label="compute ms/s")
        lines1, labels1 = ax3.get_legend_handles_labels()
        lines2, labels2 = ax3b.get_legend_handles_labels()
        ax3.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="upper right")
    else:
        ax3.text(0.5, 0.5, "No normalized DT data", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    # SLO <=250ms for top TLS by samples
    grouped = _group_samples(samples, "T_coord_req_resp")
    items = sorted(grouped.items(), key=lambda kv: len(kv[1]), reverse=True)[:8]
    if items:
        labels = [k for k, _ in items]
        slo250 = [100.0 * sum(1 for v in vals if v <= 250.0) / max(1, len(vals)) for _, vals in items]
        ax4.bar(labels, slo250, color="#72B7B2")
        ax4.set_ylim(0, 100)
        ax4.set_ylabel("SLO <=250ms (%)")
        ax4.set_title("TLS Coordination SLO Compliance")
        ax4.tick_params(axis="x", rotation=30)
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No coordination samples", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Category: Intersection Nodes", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "category_intersection_nodes_panel.png"), dpi=180)
    plt.close(fig)


def _plot_category_ev_intersection_panel(summary: Dict[str, Any], samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    ee = dict(summary.get("ev_effectiveness", {}) or {})
    ea = dict(summary.get("ev_advice_flow", {}) or {})
    cf = dict(summary.get("coordination_flow", {}) or {})
    fig = plt.figure(figsize=(14, 9))

    ax1 = fig.add_subplot(2, 2, 1)
    labels = ["req->decision", "req->apply", "seen->apply", "stuck_episode"]
    vals = [
        _f(dict(ee.get("req_to_decision_ms", {}) or {}).get("median_ms"), 0.0),
        _f(dict(ee.get("req_to_apply_ms", {}) or {}).get("median_ms"), 0.0),
        _f(dict(ea.get("seen_to_apply_latency_ms", {}) or {}).get("median_ms"), 0.0),
        _f(dict(ee.get("ev_stuck_episode_ms", {}) or {}).get("median_ms"), 0.0),
    ]
    if any(v > 0 for v in vals):
        ax1.bar(labels, vals, color=["#4C78A8", "#72B7B2", "#54A24B", "#E45756"])
        ax1.set_title("EV-Intersection Timing Chain")
        ax1.set_ylabel("median ms")
        ax1.tick_params(axis="x", rotation=20)
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No EV timing chain data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    seen = int(ee.get("route_advice_received", cf.get("route_advice_published", 0)) or 0)
    applied = int(ee.get("route_advice_applied", cf.get("route_advice_applied", 0)) or 0)
    skipped = int(ee.get("route_advice_skipped", cf.get("route_advice_skipped", 0)) or 0)
    if seen > 0 or applied > 0 or skipped > 0:
        ax2.bar(["received", "applied", "skipped"], [seen, applied, skipped], color=["#4C78A8", "#54A24B", "#E45756"])
        ratio = (float(applied) / float(seen)) if seen > 0 else 0.0
        ax2.set_title(f"Route Advice Uptake (apply={ratio:.1%})")
        ax2.set_ylabel("count")
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No route advice outcome data", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    r_int = _metric_values(samples, "T_coord_req_resp", role="intersection")
    r_ev = _metric_values(samples, "T_coord_req_resp", role="ev")
    box_data, box_labels = [], []
    if len(r_int) >= 3:
        box_data.append(r_int)
        box_labels.append("intersection")
    if len(r_ev) >= 3:
        box_data.append(r_ev)
        box_labels.append("ev")
    if box_data:
        ax3.boxplot(box_data, labels=box_labels, showfliers=False)
        ax3.set_title("Coordination Latency by Role")
        ax3.set_ylabel("ms")
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No role latency samples", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    flow_labels = ["req", "resp", "assoc+", "assoc-"]
    flow_vals = [
        int(cf.get("reservation_req_sent", 0) or 0),
        int(cf.get("reservation_resp_recv", 0) or 0),
        int(cf.get("association_created", 0) or 0),
        int(cf.get("association_released", 0) or 0),
    ]
    if any(v > 0 for v in flow_vals):
        ax4.bar(flow_labels, flow_vals, color="#4C78A8")
        ax4.set_title("EV↔Intersection Coordination Flow")
        ax4.set_ylabel("count")
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No coordination flow data", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Category: EV and Intersection Coordination", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "category_ev_intersection_coord_panel.png"), dpi=180)
    plt.close(fig)


def _plot_category_orchestrator_intersection_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    mat = _normalized_interaction_matrix(summary, "service_interaction_counts")
    metrics = dict(summary.get("metrics", {}) or {})
    assoc = dict(summary.get("association_lifecycle", {}) or {})
    fe = dict(summary.get("federation_effectiveness", {}) or {})

    fig = plt.figure(figsize=(14, 9))
    ax1 = fig.add_subplot(2, 2, 1)
    edges = []
    for s, row in mat.items():
        if not isinstance(row, dict):
            continue
        for d, c in row.items():
            n = int(c or 0)
            if n <= 0:
                continue
            if str(s) == "gtco" or str(d) == "gtco":
                edges.append((str(s), str(d), n))
    edges.sort(key=lambda x: x[2], reverse=True)
    edges = edges[:10]
    if edges:
        labels = [f"{s}->{d}" for s, d, _ in edges]
        vals = [n for _, _, n in edges]
        ax1.bar(labels, vals, color="#4C78A8")
        ax1.set_title("Orchestrator Interaction Edges")
        ax1.set_ylabel("messages")
        ax1.tick_params(axis="x", rotation=35)
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No GTCO interaction edge data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    ckeys = [
        ("C_corridor_reassess_compute", "reassess"),
        ("C_corridor_advice_compute", "advice"),
        ("C_corridor_route_advice_cycle_compute", "route_cycle"),
        ("C_corridor_state_pub_compute", "state_pub"),
        ("C_corridor_route_opt_compute", "route_opt"),
    ]
    clabels, cvals = [], []
    for k, lbl in ckeys:
        st = dict((dict(metrics.get(k, {}) or {}).get("wall_ms", {}) or {}))
        if int(st.get("n", 0) or 0) <= 0:
            continue
        clabels.append(lbl)
        cvals.append(_f(st.get("mean_ms")))
    if clabels:
        ax2.bar(clabels, cvals, color="#F58518")
        ax2.set_title("Orchestrator Compute Cost")
        ax2.set_ylabel("mean ms")
        ax2.tick_params(axis="x", rotation=20)
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No corridor compute metrics", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    cost = dict(fe.get("coordination_cost", {}) or {})
    ben = dict(fe.get("benefit_estimate", {}) or {})
    c_ms = _f(cost.get("coord_total_ms"), 0.0)
    b_sec = _f(ben.get("predicted_improvement_total_sec"), 0.0)
    if c_ms > 0 or b_sec > 0:
        ax3.bar(["coord_cost_s", "pred_benefit_s"], [c_ms / 1000.0, b_sec], color=["#E45756", "#54A24B"])
        ax3.set_title("Coordination Cost vs Predicted Benefit")
        ax3.set_ylabel("seconds")
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No cost-benefit summary", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    created = int(assoc.get("created", 0) or 0)
    released = int(assoc.get("released", 0) or 0)
    open_end = int(assoc.get("open_at_end", 0) or 0)
    if created > 0 or released > 0 or open_end > 0:
        ax4.bar(["assoc_created", "assoc_released", "assoc_open_end"], [created, released, open_end], color="#72B7B2")
        ax4.set_title("Association Lifecycle Outcome")
        ax4.set_ylabel("count")
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No association data", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Category: Orchestrator and Intersection Nodes", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "category_orchestrator_intersection_panel.png"), dpi=180)
    plt.close(fig)


def _plot_category_middleware_core_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    funnel = dict(summary.get("discovery_funnel", {}) or {})
    svc_rates = dict(summary.get("message_volume_rates_by_service", {}) or {})
    mat = _normalized_interaction_matrix(summary, "service_interaction_counts_control_plane")
    if not mat:
        mat = _normalized_interaction_matrix(summary, "service_interaction_counts")
        _, mat, _ = _split_interaction_by_plane(mat)
    fig = plt.figure(figsize=(14, 9))

    ax1 = fig.add_subplot(2, 2, 1)
    keys = [("register_to_onboarding_ms", "reg->onb"), ("onboarding_to_active_ms", "onb->active"), ("time_to_active_ms", "reg->active")]
    labels, vals = [], []
    for k, l in keys:
        st = dict(phase.get(k, {}) or {})
        if int(st.get("n", 0) or 0) <= 0:
            continue
        labels.append(l)
        vals.append(_f(st.get("median_ms")))
    if labels:
        ax1.bar(labels, vals, color="#4C78A8")
        ax1.set_title("Lifecycle Startup Phases")
        ax1.set_ylabel("median ms")
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No lifecycle startup data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    req = int(funnel.get("requests", 0) or 0)
    resp = int(funnel.get("responses", 0) or 0)
    hit = int(funnel.get("hits", 0) or 0)
    if req > 0 or resp > 0 or hit > 0:
        ax2.bar(["query", "response", "hit"], [req, resp, hit], color=["#4C78A8", "#72B7B2", "#54A24B"])
        ratio = (float(hit) / float(req)) if req > 0 else 0.0
        ax2.set_title(f"Discovery Funnel (hit/query={ratio:.1%})")
        ax2.set_ylabel("count")
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No discovery funnel data", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    if svc_rates:
        items = []
        for s, rec in svc_rates.items():
            n = _norm_interaction_node(str(s))
            if n not in _CONTROL_NODES and n not in _GATEWAY_NODES:
                continue
            items.append((n, _f(dict(rec or {}).get("messages_per_s"), 0.0)))
        agg: Dict[str, float] = {}
        for s, v in items:
            agg[s] = agg.get(s, 0.0) + float(v)
        labels = _ordered_nodes(list(agg.keys()))
        if labels:
            msgs = [agg.get(k, 0.0) for k in labels]
            ax3.bar(labels, msgs, color="#F58518")
            ax3.set_title("Control-Plane Message Rate by Service")
            ax3.set_ylabel("messages/s")
            ax3.tick_params(axis="x", rotation=20)
            ax3.grid(axis="y", linestyle="--", alpha=0.35)
        else:
            ax3.text(0.5, 0.5, "No control-plane service rates", ha="center", va="center")
            ax3.set_axis_off()
    else:
        ax3.text(0.5, 0.5, "No service message-rate data", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    if mat:
        nodes = _ordered_nodes(_matrix_nodes(mat))
        n = len(nodes)
        if n >= 2:
            idx = {k: i for i, k in enumerate(nodes)}
            vals2 = [[0.0 for _ in range(n)] for _ in range(n)]
            for s, row in mat.items():
                if not isinstance(row, dict):
                    continue
                for d, c in row.items():
                    vals2[idx[str(s)]][idx[str(d)]] = math.log1p(float(c or 0.0))
            im = ax4.imshow(vals2, aspect="auto", cmap="Blues")
            ax4.set_xticks(list(range(n)))
            ax4.set_yticks(list(range(n)))
            ax4.set_xticklabels(nodes, rotation=35, ha="right", fontsize=8)
            ax4.set_yticklabels(nodes, fontsize=8)
            ax4.set_title("Control-Plane Interaction Matrix (log1p)")
            fig.colorbar(im, ax=ax4, fraction=0.046, pad=0.04, label="log(1+messages)")
        else:
            ax4.text(0.5, 0.5, "Not enough service nodes", ha="center", va="center")
            ax4.set_axis_off()
    else:
        ax4.text(0.5, 0.5, "No service interaction matrix", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Category: Middleware Core Services", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "category_middleware_core_panel.png"), dpi=180)
    plt.close(fig)


def _plot_category_collaborative_decision_panel(summary: Dict[str, Any], samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    cf = dict(summary.get("coordination_flow", {}) or {})
    fe = dict(summary.get("federation_effectiveness", {}) or {})
    ratios = dict(fe.get("ratios", {}) or {})
    fig = plt.figure(figsize=(14, 9))

    ax1 = fig.add_subplot(2, 2, 1)
    labels = ["req", "resp", "tls_advice", "route_advice", "assoc+"]
    vals = [
        int(cf.get("reservation_req_sent", 0) or 0),
        int(cf.get("reservation_resp_recv", 0) or 0),
        int(cf.get("intersection_advice_published", 0) or 0),
        int(cf.get("route_advice_published", 0) or 0),
        int(cf.get("association_created", 0) or 0),
    ]
    if any(v > 0 for v in vals):
        ax1.bar(labels, vals, color="#4C78A8")
        ax1.set_title("Collaborative Decision Flow Volume")
        ax1.set_ylabel("count")
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No collaboration flow data", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 2, 2)
    # Windowed throughput -> median latency
    rows = []
    for s in samples:
        if str(s.get("metric", "")) != "T_coord_req_resp":
            continue
        ts = _f(s.get("end_ts_wall_ms"), -1.0)
        lat = _f(s.get("latency_wall_ms"), -1.0)
        if ts >= 0 and lat >= 0:
            rows.append((ts, lat))
    if len(rows) >= 6:
        rows.sort(key=lambda x: x[0])
        t0 = rows[0][0]
        bins: Dict[int, List[float]] = {}
        win_s = 10.0
        for ts, lat in rows:
            b = int((ts - t0) / (win_s * 1000.0))
            bins.setdefault(b, []).append(lat)
        xs, ys = [], []
        for b in sorted(bins.keys()):
            vals_b = bins[b]
            xs.append(float(len(vals_b)) / win_s)
            ys.append(_percentile(vals_b, 50.0))
        ax2.scatter(xs, ys, color="#F58518", alpha=0.75)
        ax2.set_title("Throughput vs Median Latency")
        ax2.set_xlabel("req/s")
        ax2.set_ylabel("median ms")
        ax2.grid(True, linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No enough req/resp samples", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 2, 3)
    dlabels = ["assoc_setup", "coord_req_resp", "advice_uptake"]
    dvals = []
    for m in ("T_assoc_setup", "T_coord_req_resp", "T_advice_uptake"):
        st = dict((dict(summary.get("metrics", {}) or {}).get(m, {}) or {}).get("wall_ms", {}))
        dvals.append(_f(st.get("median_ms"), 0.0))
    if any(v > 0 for v in dvals):
        ax3.bar(dlabels, dvals, color=["#72B7B2", "#4C78A8", "#54A24B"])
        ax3.set_title("Decision-Path Median Delays")
        ax3.set_ylabel("ms")
        ax3.tick_params(axis="x", rotation=20)
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No decision-path medians", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 2, 4)
    rkeys = [
        ("reservation_req_resp_ratio", "req->resp"),
        ("association_closure_ratio", "assoc_close"),
        ("discovery_hit_query_ratio", "discovery_hit"),
        ("route_advice_apply_ratio", "advice_apply"),
    ]
    rl, rv = [], []
    for k, lbl in rkeys:
        v = ratios.get(k, None)
        if v is None:
            continue
        rl.append(lbl)
        rv.append(100.0 * max(0.0, min(1.0, float(v))))
    if rl:
        ax4.bar(rl, rv, color="#54A24B")
        ax4.set_ylim(0, 100)
        ax4.set_ylabel("%")
        ax4.set_title("Collaboration Effectiveness Ratios")
        ax4.tick_params(axis="x", rotation=20)
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No effectiveness ratios", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Category: Collaborative Decision Making", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "category_collaborative_decision_panel.png"), dpi=180)
    plt.close(fig)


def _plot_federation_storyline_panel(summary: Dict[str, Any], samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    phase = dict(summary.get("operational_phases_ms", {}) or {})
    flow = dict(summary.get("coordination_flow", {}) or {})
    ratios = dict((dict(summary.get("federation_effectiveness", {}) or {}).get("ratios", {}) or {}))
    ra = dict(summary.get("role_activity", {}) or {})
    plane_tot = dict(summary.get("service_interaction_plane_totals", {}) or {})

    fig = plt.figure(figsize=(16, 10))

    ax1 = fig.add_subplot(2, 3, 1)
    lk = [("register_to_onboarding_ms", "reg->onb"), ("onboarding_to_active_ms", "onb->active"), ("time_to_active_ms", "reg->active")]
    labels1, vals1 = [], []
    for k, lbl in lk:
        st = dict(phase.get(k, {}) or {})
        if int(st.get("n", 0) or 0) <= 0:
            continue
        labels1.append(lbl)
        vals1.append(_f(st.get("median_ms"), 0.0))
    if labels1:
        ax1.bar(labels1, vals1, color="#4C78A8")
        ax1.set_ylabel("median ms")
        ax1.set_title("Lifecycle Startup")
        ax1.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax1.text(0.5, 0.5, "No lifecycle startup samples", ha="center", va="center")
        ax1.set_axis_off()

    ax2 = fig.add_subplot(2, 3, 2)
    d = _f(plane_tot.get("data_plane_msgs"), 0.0)
    c = _f(plane_tot.get("control_plane_msgs"), 0.0)
    x = _f(plane_tot.get("cross_plane_msgs"), 0.0)
    if d + c + x > 0:
        ax2.bar(["data", "control", "cross"], [d, c, x], color=["#4C78A8", "#72B7B2", "#F58518"])
        ax2.set_ylabel("messages")
        ax2.set_title("Interaction Mix by Plane")
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax2.text(0.5, 0.5, "No plane interaction data", ha="center", va="center")
        ax2.set_axis_off()

    ax3 = fig.add_subplot(2, 3, 3)
    f_labels = ["req", "resp", "assoc+", "assoc-", "advice_applied"]
    f_vals = [
        _f(flow.get("reservation_req_sent"), 0.0),
        _f(flow.get("reservation_resp_recv"), 0.0),
        _f(flow.get("association_created"), 0.0),
        _f(flow.get("association_released"), 0.0),
        _f(flow.get("route_advice_applied"), 0.0),
    ]
    if sum(f_vals) > 0:
        ax3.bar(f_labels, f_vals, color="#54A24B")
        ax3.set_ylabel("count")
        ax3.set_title("Coordination Funnel")
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No coordination flow data", ha="center", va="center")
        ax3.set_axis_off()

    ax4 = fig.add_subplot(2, 3, 4)
    mset = [
        ("T_coord_req_resp", "coord req->resp"),
        ("T_onboard", "onboard"),
        ("T_discovery_e2e", "discovery"),
        ("T_advice_uptake", "advice uptake"),
    ]
    bx, bl = [], []
    for mk, lbl in mset:
        vals = _metric_values(samples, mk)
        if vals:
            bx.append(vals)
            bl.append(lbl)
    if bx:
        ax4.boxplot(bx, labels=bl, showfliers=False)
        ax4.set_ylabel("ms")
        ax4.set_title("E2E Latency Distributions")
        ax4.tick_params(axis="x", rotation=20)
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No E2E latency samples", ha="center", va="center")
        ax4.set_axis_off()

    ax5 = fig.add_subplot(2, 3, 5)
    roles = [r for r in ("ev", "intersection", "orchestrator") if r in ra] + [r for r in sorted(ra.keys()) if r not in ("ev", "intersection", "orchestrator")]
    if roles:
        msg = [_f(dict(ra.get(r, {}) or {}).get("message_rate_s"), 0.0) for r in roles]
        coord = [_f(dict(ra.get(r, {}) or {}).get("coord_ms_per_s"), 0.0) for r in roles]
        comp = [_f(dict(ra.get(r, {}) or {}).get("compute_ms_per_s"), 0.0) for r in roles]
        xs = list(range(len(roles)))
        w = 0.28
        ax5.bar([x - w for x in xs], msg, width=w, label="msg/s", color="#4C78A8")
        ax5.bar(xs, coord, width=w, label="coord ms/s", color="#F58518")
        ax5.bar([x + w for x in xs], comp, width=w, label="compute ms/s", color="#54A24B")
        ax5.set_xticks(xs)
        ax5.set_xticklabels(roles, rotation=20, ha="right")
        ax5.set_title("Role Normalized Overhead")
        ax5.grid(axis="y", linestyle="--", alpha=0.35)
        ax5.legend(fontsize=8)
    else:
        ax5.text(0.5, 0.5, "No role activity data", ha="center", va="center")
        ax5.set_axis_off()

    ax6 = fig.add_subplot(2, 3, 6)
    rk = [
        ("reservation_req_resp_ratio", "req/resp"),
        ("association_closure_ratio", "assoc close"),
        ("discovery_hit_query_ratio", "disc hit"),
        ("route_advice_apply_ratio", "advice apply"),
        ("coord_tls_coverage_ratio", "tls coverage"),
    ]
    l6, v6 = [], []
    for k, lbl in rk:
        val = ratios.get(k, None)
        if val is None:
            continue
        l6.append(lbl)
        v6.append(100.0 * _f(val, 0.0))
    if l6:
        ax6.bar(l6, v6, color="#72B7B2")
        ax6.set_ylim(0, 100)
        ax6.set_ylabel("%")
        ax6.set_title("Federation Effectiveness Ratios")
        ax6.tick_params(axis="x", rotation=20)
        ax6.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax6.text(0.5, 0.5, "No effectiveness ratio data", ha="center", va="center")
        ax6.set_axis_off()

    fig.suptitle("Federation Storyline Panel (Lifecycle, Coordination, Overhead, E2E)", fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "federation_storyline_panel.png"), dpi=180)
    plt.close(fig)


def _plot_fnm_integration_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fi = dict(summary.get("fnm_integration", {}) or {})
    if not fi:
        return

    state_pull = dict(fi.get("state_pull", {}) or {})
    route_bridge = dict(fi.get("route_bridge", {}) or {})
    lpipe = dict(fi.get("latency_pipeline_ms", {}) or {})
    delivery = dict(fi.get("delivery_success", {}) or {})

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.25)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])

    # (a) HTTP pull + request emission health
    l1 = ["pull_ok", "pull_error", "req_pub_events", "req_pub_total"]
    v1 = [
        float(_f(state_pull.get("ok"), 0.0)),
        float(_f(state_pull.get("error"), 0.0)),
        float(_f(state_pull.get("req_published_events"), 0.0)),
        float(_f(state_pull.get("req_published_total"), 0.0)),
    ]
    c1 = ["#4C78A8", "#E45756", "#54A24B", "#F58518"]
    ax1.bar(l1, v1, color=c1)
    ax1.set_title("FNM EV Integration Health")
    ax1.set_ylabel("Count")
    ax1.grid(axis="y", linestyle="--", alpha=0.35)
    succ = state_pull.get("success_ratio")
    if succ is not None:
        ax1.text(
            0.02,
            0.95,
            f"pull_success={100.0 * _f(succ, 0.0):.1f}%",
            transform=ax1.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            bbox={"boxstyle": "round", "facecolor": "#f0f0f0", "alpha": 0.8},
        )

    # (b) Directional protocol mediation traffic
    l2 = ["local_to_fed", "fed_to_local"]
    v2 = [float(_f(route_bridge.get("local_to_fed"), 0.0)), float(_f(route_bridge.get("fed_to_local"), 0.0))]
    ax2.bar(l2, v2, color=["#72B7B2", "#B279A2"])
    ax2.set_title("FNM Routing Direction Mix")
    ax2.set_ylabel("Messages")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)

    # (c) Pipeline latencies (median)
    lp_order = [
        ("state_propagation_latency_ms", "state->apply"),
        ("request_to_signal_change_latency_ms", "req->signal"),
        ("request_to_decision_latency_ms", "req->decision"),
        ("request_to_actuation_latency_ms", "req->apply"),
        ("request_to_response_latency_ms", "req->resp"),
        ("request_age_latency_ms", "request_age"),
        ("advice_seen_to_apply_latency_ms", "advice->apply"),
    ]
    l3, v3 = [], []
    for k, lbl in lp_order:
        st = dict(lpipe.get(k, {}) or {})
        if int(_f(st.get("n"), 0.0)) <= 0:
            continue
        l3.append(lbl)
        v3.append(float(_f(st.get("median_ms"), 0.0)))
    if l3:
        ax3.bar(l3, v3, color="#4C78A8")
        ax3.set_ylabel("Median latency (ms)")
        ax3.set_title("Integration Pipeline Latency")
        ax3.tick_params(axis="x", rotation=18)
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No pipeline latency samples", ha="center", va="center")
        ax3.set_axis_off()

    # (d) Delivery/timeliness success ratios
    l4, v4 = [], []
    for k, lbl in [
        ("reservation_req_resp_ratio", "req/resp"),
        ("association_closure_ratio", "assoc closure"),
        ("route_advice_apply_ratio", "advice apply"),
        ("coord_tls_coverage_ratio", "TLS coverage"),
    ]:
        x = delivery.get(k, None)
        if x is None:
            continue
        l4.append(lbl)
        v4.append(100.0 * _f(x, 0.0))
    if l4:
        ax4.bar(l4, v4, color="#54A24B")
        ax4.set_ylim(0, 100)
        ax4.set_ylabel("%")
        ax4.set_title("Delivery Success Ratios")
        ax4.tick_params(axis="x", rotation=15)
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No delivery ratio data", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("FNM Integration Panel", fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "fnm_integration_panel.png"), dpi=180)
    plt.close(fig)


def _plot_fnm_overhead_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    fo = dict(summary.get("fnm_overhead", {}) or {})
    if not fo:
        return
    by_dt = dict(fo.get("by_dt", {}) or {})
    if not by_dt:
        return

    rows = []
    for dt, rec in by_dt.items():
        d = dict(rec or {})
        cpu = _f(dict(d.get("cpu_util_pct", {}) or {}).get("mean_ms"), 0.0)
        rss = _f(dict(d.get("max_rss_kb", {}) or {}).get("mean_ms"), 0.0)
        wall_ms = _f(dict(d.get("wall_runtime_s", {}) or {}).get("mean_ms"), 0.0)
        if cpu <= 0 and rss <= 0 and wall_ms <= 0:
            continue
        rows.append((str(dt), cpu, rss, wall_ms / 1000.0))
    if not rows:
        return
    rows.sort(key=lambda x: x[1], reverse=True)
    rows = rows[:12]
    labels = [r[0] for r in rows]
    cpu_vals = [r[1] for r in rows]
    rss_vals = [r[2] for r in rows]
    wall_vals = [r[3] for r in rows]

    fig = plt.figure(figsize=(14, 6))
    ax1 = fig.add_subplot(1, 2, 1)
    xs = list(range(len(labels)))
    ax1.bar(xs, cpu_vals, color="#4C78A8")
    ax1.set_xticks(xs)
    ax1.set_xticklabels(labels, rotation=30, ha="right")
    ax1.set_ylabel("CPU util (%)")
    ax1.set_title("Per-FNM CPU Overhead")
    ax1.grid(axis="y", linestyle="--", alpha=0.35)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.bar(xs, rss_vals, color="#F58518", label="max RSS (kB)")
    ax2.set_xticks(xs)
    ax2.set_xticklabels(labels, rotation=30, ha="right")
    ax2.set_ylabel("Memory (kB)")
    ax2.set_title("Per-FNM Memory Overhead")
    ax2.grid(axis="y", linestyle="--", alpha=0.35)
    ax2b = ax2.twinx()
    ax2b.plot(xs, wall_vals, color="#54A24B", marker="o", linewidth=1.5, label="runtime (s)")
    ax2b.set_ylabel("Runtime (s)")
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="upper right")

    fig.suptitle("FNM Process Overhead Panel", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(out_dir, "fnm_overhead_panel.png"), dpi=170)
    plt.close(fig)


def _plot_coordination_diagnostics_panel(summary: Dict[str, Any], out_dir: str, plt) -> None:
    cd = dict(summary.get("coordination_diagnostics", {}) or {})
    if not cd:
        return

    apply_mix = dict(cd.get("apply_mix", {}) or {})
    by_tls = dict(cd.get("apply_mix_by_tls", {}) or {})
    skip = dict(cd.get("hard_req_skip_reasons", {}) or {})

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.25)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])

    # (a) Global apply mix
    keys1 = ["plan_applied", "offer_applied", "offer_selected", "f2_local_fallback_applied"]
    vals1 = [float(_f(apply_mix.get(k), 0.0)) for k in keys1]
    ax1.bar(keys1, vals1, color=["#4C78A8", "#F58518", "#54A24B", "#E45756"])
    ax1.set_title("Coordination Apply Mix")
    ax1.set_ylabel("Count")
    ax1.tick_params(axis="x", rotation=18)
    ax1.grid(axis="y", linestyle="--", alpha=0.35)

    # (b) Per-TLS offer-vs-plan churn
    rows = []
    for tls, rr in by_tls.items():
        pa = float(_f(rr.get("plan_applied"), 0.0))
        oa = float(_f(rr.get("offer_applied"), 0.0))
        fa = float(_f(rr.get("f2_local_fallback_applied"), 0.0))
        tot = pa + oa + fa
        if tot <= 0:
            continue
        rows.append((str(tls), pa, oa, fa, tot))
    rows.sort(key=lambda x: x[4], reverse=True)
    rows = rows[:10]
    if rows:
        labels = [r[0] for r in rows]
        pa = [r[1] for r in rows]
        oa = [r[2] for r in rows]
        fa = [r[3] for r in rows]
        xs = list(range(len(labels)))
        ax2.bar(xs, pa, label="plan_applied", color="#4C78A8")
        ax2.bar(xs, oa, bottom=pa, label="offer_applied", color="#F58518")
        bottoms = [pa[i] + oa[i] for i in range(len(pa))]
        ax2.bar(xs, fa, bottom=bottoms, label="f2_local_fallback", color="#E45756")
        ax2.set_xticks(xs)
        ax2.set_xticklabels(labels, rotation=30, ha="right")
        ax2.set_title("Per-TLS Apply Mix (Top by volume)")
        ax2.set_ylabel("Count")
        ax2.grid(axis="y", linestyle="--", alpha=0.35)
        ax2.legend(fontsize=8)
    else:
        ax2.text(0.5, 0.5, "No per-TLS apply mix data", ha="center", va="center")
        ax2.set_axis_off()

    # (c) Hard skip reasons
    top_skip = sorted(skip.items(), key=lambda kv: int(kv[1] or 0), reverse=True)[:8]
    if top_skip:
        l3 = [str(k) for k, _ in top_skip]
        v3 = [float(_f(v, 0.0)) for _, v in top_skip]
        ax3.bar(l3, v3, color="#B279A2")
        ax3.set_title("Hard Reservation Skip Reasons")
        ax3.set_ylabel("Count")
        ax3.tick_params(axis="x", rotation=25)
        ax3.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax3.text(0.5, 0.5, "No hard skip reason data", ha="center", va="center")
        ax3.set_axis_off()

    # (d) Guardrail ratios
    l4, v4 = [], []
    r1 = cd.get("offer_to_plan_apply_ratio", None)
    r2 = cd.get("f2_local_fallback_share_of_plan_apply", None)
    if r1 is not None:
        l4.append("offer/plan apply")
        v4.append(float(_f(r1, 0.0)))
    if r2 is not None:
        l4.append("fallback/plan apply")
        v4.append(float(_f(r2, 0.0)))
    if l4:
        ax4.bar(l4, v4, color="#72B7B2")
        ax4.set_title("Coordination Guardrail Ratios")
        ax4.set_ylabel("Ratio")
        ax4.grid(axis="y", linestyle="--", alpha=0.35)
    else:
        ax4.text(0.5, 0.5, "No ratio data", ha="center", va="center")
        ax4.set_axis_off()

    fig.suptitle("Coordination Diagnostics Panel", fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(out_dir, "coordination_diagnostics_panel.png"), dpi=180)
    plt.close(fig)


def _write_category_plot_index(summary: Dict[str, Any], out_dir: str) -> None:
    index = {
        "category_panels": [
            {
                "name": "intersection_nodes",
                "file": "category_intersection_nodes_panel.png",
                "focus": "per-TLS latency/overhead/slo/load",
            },
            {
                "name": "ev_intersection_coordination",
                "file": "category_ev_intersection_coord_panel.png",
                "focus": "EV advice uptake and EV-TLS timing chain",
            },
            {
                "name": "orchestrator_intersection",
                "file": "category_orchestrator_intersection_panel.png",
                "focus": "GTCO interactions, compute cost, association outcomes",
            },
            {
                "name": "middleware_core",
                "file": "category_middleware_core_panel.png",
                "focus": "membership/discovery/interaction matrix and load",
            },
            {
                "name": "collaborative_decision",
                "file": "category_collaborative_decision_panel.png",
                "focus": "decision flow, throughput-latency, effectiveness ratios",
            },
            {
                "name": "interaction_planes",
                "file": "service_interaction_planes_panel.png",
                "focus": "data/control/cross-plane MQTT interaction separation",
            },
            {
                "name": "ev_transit_sequence",
                "file": "ev_transit_tls_sequence.png",
                "focus": "approx EV traversal sequence across coordinated TLS",
            },
            {
                "name": "mqtt_topic_profile",
                "file": "mqtt_topic_profile.png",
                "focus": "top MQTT topics by volume and payload intensity",
            },
            {
                "name": "mqtt_topic_origin_mix",
                "file": "mqtt_topic_origin_mix.png",
                "focus": "raw vs derived topic coverage from logs",
            },
            {
                "name": "mqtt_service_topic_heatmap",
                "file": "mqtt_service_topic_heatmap.png",
                "focus": "producer services versus topic traffic concentration",
            },
            {
                "name": "mqtt_analysis_panel",
                "file": "mqtt_analysis_panel.png",
                "focus": "condensed MQTT communication storyline",
            },
            {
                "name": "federation_storyline",
                "file": "federation_storyline_panel.png",
                "focus": "single-view lifecycle, e2e latency, coordination, and overhead",
            },
            {
                "name": "fnm_integration",
                "file": "fnm_integration_panel.png",
                "focus": "protocol mediation health, routing direction, and latency pipeline",
            },
            {
                "name": "coordination_diagnostics",
                "file": "coordination_diagnostics_panel.png",
                "focus": "offer-vs-plan churn, fallback share, and skip-reason diagnostics",
            },
        ],
        "summary_available_keys": sorted(list((summary or {}).keys())),
    }
    with open(os.path.join(out_dir, "category_plot_index.json"), "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2, ensure_ascii=True)


def _write_dt_overhead_table(summary: Dict[str, Any], out_dir: str) -> None:
    nd = dict(summary.get("normalized_overhead_by_dt", {}) or {})
    if not nd:
        return
    rows: List[Dict[str, Any]] = []
    for dt, rec in nd.items():
        d = dict(rec or {})
        rows.append(
            {
                "dt_id": str(dt),
                "role": str(d.get("role", "")),
                "active_span_s": d.get("active_span_s"),
                "message_total": d.get("message_total"),
                "message_rate_s": d.get("message_rate_s"),
                "comm_message_total": d.get("comm_message_total"),
                "comm_message_rate_s": d.get("comm_message_rate_s"),
                "coord_n": d.get("coord_n"),
                "coord_mean_ms": d.get("coord_mean_ms"),
                "coord_p95_ms": d.get("coord_p95_ms"),
                "coord_ms_per_s": d.get("coord_ms_per_s"),
                "compute_n": d.get("compute_n"),
                "compute_mean_ms": d.get("compute_mean_ms"),
                "compute_ms_per_s": d.get("compute_ms_per_s"),
            }
        )
    if not rows:
        return
    rows.sort(
        key=lambda r: (
            _f(r.get("message_rate_s"), 0.0),
            _f(r.get("coord_mean_ms"), 0.0),
            _f(r.get("coord_ms_per_s"), 0.0),
        ),
        reverse=True,
    )
    path = os.path.join(out_dir, "dt_normalized_overhead_ranking.csv")
    cols = [
        "dt_id",
        "role",
        "active_span_s",
        "message_total",
        "message_rate_s",
        "comm_message_total",
        "comm_message_rate_s",
        "coord_n",
        "coord_mean_ms",
        "coord_p95_ms",
        "coord_ms_per_s",
        "compute_n",
        "compute_mean_ms",
        "compute_ms_per_s",
    ]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

def _plot_decision_path_latency(rows: List[Dict[str, str]], out_dir: str, plt) -> None:
    keep_metrics = [
        "T_discovery_e2e",
        "T_assoc_setup",
        "T_coord_req_resp",
        "T_advice_uptake",
        "T_time_to_active",
        "T_onboard",
    ]
    data = [r for r in rows if str(r.get("domain", "")) == "wall_ms" and str(r.get("metric", "")) in keep_metrics]
    if not data:
        return
    data.sort(key=lambda r: keep_metrics.index(str(r.get("metric", ""))))
    labels = [str(r.get("metric", "")) for r in data]
    means = [_f(r.get("mean_ms")) for r in data]
    p95s = [_f(r.get("p95_ms")) for r in data]
    fig = plt.figure(figsize=(max(10, 0.8 * len(labels)), 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylabel("E2E delay (ms)")
    ax.set_title("Federation Decision-Path Delays")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "decision_path_latency_ms.png"), dpi=160)
    plt.close(fig)


def _plot_decision_path_boxplots(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    keep_metrics = [
        "T_register",
        "T_onboard",
        "T_time_to_active",
        "T_discovery_e2e",
        "T_assoc_setup",
        "T_coord_req_resp",
        "T_advice_uptake",
    ]
    grouped: Dict[str, List[float]] = {}
    for s in samples:
        m = str(s.get("metric", ""))
        if m not in keep_metrics:
            continue
        y = _f(s.get("latency_wall_ms"))
        if y <= 0:
            continue
        grouped.setdefault(m, []).append(y)
    labels = [m for m in keep_metrics if len(grouped.get(m, [])) >= 3]
    if not labels:
        return
    data = [grouped[m] for m in labels]
    fig = plt.figure(figsize=(max(10, 1.2 * len(labels)), 6))
    ax = fig.add_subplot(111)
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Decision-Path Latency Distributions (Boxplots)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    plt.setp(ax.get_xticklabels(), rotation=25, ha="right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "decision_path_latency_boxplots.png"), dpi=160)
    plt.close(fig)


def _plot_coord_latency_boxplot_by_tls(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    grouped = _group_samples(samples, "T_coord_req_resp")
    if not grouped:
        return
    items = sorted(grouped.items(), key=lambda kv: len(kv[1]), reverse=True)[:12]
    labels = [k for k, _ in items]
    data = [v for _, v in items]
    fig = plt.figure(figsize=(max(10, 0.8 * len(labels)), 6))
    ax = fig.add_subplot(111)
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.set_ylabel("Coordination latency (ms)")
    ax.set_title("Per-Intersection Coordination Latency Distribution")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_coord_latency_boxplot_by_tls.png"), dpi=160)
    plt.close(fig)


def _plot_coord_slo_by_tls(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    grouped = _group_samples(samples, "T_coord_req_resp")
    if not grouped:
        return
    items = sorted(grouped.items(), key=lambda kv: len(kv[1]), reverse=True)[:12]
    labels = [k for k, _ in items]
    slo_100 = []
    slo_250 = []
    slo_400 = []
    for _, vals in items:
        n = max(1, len(vals))
        slo_100.append(100.0 * sum(1 for x in vals if x <= 100.0) / n)
        slo_250.append(100.0 * sum(1 for x in vals if x <= 250.0) / n)
        slo_400.append(100.0 * sum(1 for x in vals if x <= 400.0) / n)

    fig = plt.figure(figsize=(max(10, 0.9 * len(labels)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.26
    ax.bar([x - w for x in xs], slo_100, width=w, label="<=100ms")
    ax.bar(xs, slo_250, width=w, label="<=250ms")
    ax.bar([x + w for x in xs], slo_400, width=w, label="<=400ms")
    ax.set_ylim(0, 100)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.set_ylabel("SLO compliance (%)")
    ax.set_title("Coordination Latency SLO Compliance by Intersection")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_coord_slo_by_tls.png"), dpi=160)
    plt.close(fig)


def _plot_coord_load_vs_latency(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("metrics_by_tls", {}) or {})
    comm_by_tls = dict(summary.get("communication_overhead_by_tls", {}) or {})
    rows: List[Tuple[str, float, float, int, int]] = []
    for tls, mm in by_tls.items():
        coord = dict(mm.get("T_coord_req_resp", {}) or {})
        wall = dict(coord.get("wall_ms", {}) or {})
        n = int(wall.get("n", 0) or 0)
        if n <= 0:
            continue
        mean_ms = _f(wall.get("mean_ms"))
        p95_ms = _f(wall.get("p95_ms"))
        c = dict(comm_by_tls.get(tls, {}) or {})
        load = int(
            c.get("reservation_req_sent", 0)
            + c.get("reservation_resp_recv", 0)
            + c.get("assoc_created", 0)
            + c.get("assoc_released", 0)
            + c.get("route_advice_published", 0)
            + c.get("intersection_advice_published", 0)
        )
        rows.append((str(tls), mean_ms, p95_ms, n, load))
    if not rows:
        return

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    xs = [r[4] for r in rows]
    ys = [r[1] for r in rows]
    sizes = [max(20, 10 + int(r[3] * 0.5)) for r in rows]
    ax.scatter(xs, ys, s=sizes, alpha=0.65, label="mean latency")
    for tls, mean_ms, _, _, load in sorted(rows, key=lambda r: r[4], reverse=True)[:8]:
        ax.annotate(tls, (load, mean_ms), fontsize=8)
    ax.set_xlabel("Coordination message load (count)")
    ax.set_ylabel("Mean coordination latency (ms)")
    ax.set_title("Intersection Load vs Coordination Latency")
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_load_vs_latency_scatter.png"), dpi=160)
    plt.close(fig)


def _plot_compute_metric_distributions(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("metrics_by_tls", {}) or {})
    wanted = [
        "C_intersection_tick_compute",
        "C_intersection_refine_compute",
        "C_intersection_apply_compute",
    ]
    vals: Dict[str, List[float]] = {k: [] for k in wanted}
    for _, mm in by_tls.items():
        for m in wanted:
            st = dict((dict(mm.get(m, {}) or {}).get("wall_ms", {}) or {}))
            n = int(st.get("n", 0) or 0)
            if n <= 0:
                continue
            med = _f(st.get("median_ms"), -1.0)
            if med >= 0:
                vals[m].append(med)
    labels = [m for m in wanted if vals[m]]
    if not labels:
        return
    data = [vals[m] for m in labels]
    fig = plt.figure(figsize=(max(8, 1.0 * len(labels)), 5))
    ax = fig.add_subplot(111)
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.set_ylabel("Median compute duration per TLS (ms)")
    ax.set_title("Direct Computation Cost Distributions")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    plt.setp(ax.get_xticklabels(), rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_direct_compute_distributions.png"), dpi=160)
    plt.close(fig)


def _plot_compute_vs_comm_by_tls(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("metrics_by_tls", {}) or {})
    comm = dict(summary.get("communication_overhead_by_tls", {}) or {})
    rows: List[Tuple[str, int, float]] = []
    for tls, mm in by_tls.items():
        c = dict(comm.get(tls, {}) or {})
        load = int(
            c.get("reservation_req_sent", 0)
            + c.get("reservation_resp_recv", 0)
            + c.get("assoc_created", 0)
            + c.get("assoc_released", 0)
            + c.get("route_advice_published", 0)
            + c.get("intersection_advice_published", 0)
        )
        comp_vals = []
        for m in (
            "C_intersection_tick_compute",
            "C_intersection_refine_compute",
            "C_intersection_apply_compute",
        ):
            st = dict((dict(mm.get(m, {}) or {}).get("wall_ms", {}) or {}))
            if int(st.get("n", 0) or 0) > 0:
                comp_vals.append(_f(st.get("mean_ms")))
        if not comp_vals:
            continue
        rows.append((str(tls), load, statistics.fmean(comp_vals)))
    if not rows:
        return

    rows.sort(key=lambda x: x[1], reverse=True)
    rows = rows[:12]
    labels = [r[0] for r in rows]
    load_vals = [r[1] for r in rows]
    comp_vals = [r[2] for r in rows]
    xs = list(range(len(labels)))
    fig = plt.figure(figsize=(max(10, 0.8 * len(labels)), 6))
    ax1 = fig.add_subplot(111)
    ax1.bar(xs, load_vals, alpha=0.75, label="comm messages")
    ax1.set_ylabel("Communication load (count)")
    ax1.set_xticks(xs)
    ax1.set_xticklabels(labels, rotation=30, ha="right")
    ax1.grid(axis="y", linestyle="--", alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(xs, comp_vals, color="tab:red", marker="o", linewidth=2.0, label="compute mean ms")
    ax2.set_ylabel("Computation mean (ms)")
    ax1.set_title("Communication vs Computation Cost by Intersection")
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_comm_vs_compute_by_tls.png"), dpi=160)
    plt.close(fig)


def _plot_role_latency_boxplots(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    roles = ["ev", "intersection", "orchestrator"]
    data: List[List[float]] = []
    labels: List[str] = []
    for r in roles:
        vals = _metric_values(samples, "T_coord_req_resp", role=r)
        if len(vals) < 3:
            continue
        data.append(vals)
        labels.append(r)
    if not labels:
        return
    fig = plt.figure(figsize=(max(7, 1.3 * len(labels)), 5))
    ax = fig.add_subplot(111)
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.set_ylabel("Coordination E2E latency (ms)")
    ax.set_title("Coordination Latency by Federation Role")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "role_coord_latency_boxplots.png"), dpi=160)
    plt.close(fig)


def _plot_coord_throughput_vs_latency(samples: List[Dict[str, str]], out_dir: str, plt, window_sec: float = 10.0) -> None:
    rows: List[Tuple[float, float]] = []
    for s in samples:
        if str(s.get("metric", "")) != "T_coord_req_resp":
            continue
        ts = _f(s.get("end_ts_wall_ms"), -1.0)
        lat = _f(s.get("latency_wall_ms"), -1.0)
        if ts < 0 or lat < 0:
            continue
        rows.append((ts, lat))
    if len(rows) < 5:
        return
    rows.sort(key=lambda x: x[0])
    t0 = rows[0][0]
    bins: Dict[int, List[float]] = {}
    for ts, lat in rows:
        b = int((ts - t0) / (window_sec * 1000.0))
        bins.setdefault(b, []).append(lat)
    xs: List[float] = []
    ys_med: List[float] = []
    ys_iqr: List[float] = []
    for b in sorted(bins.keys()):
        vals = bins[b]
        tp = float(len(vals)) / float(window_sec)
        xs.append(tp)
        ys_med.append(_percentile(vals, 50.0))
        ys_iqr.append(max(0.0, _percentile(vals, 75.0) - _percentile(vals, 25.0)))
    if not xs:
        return
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111)
    ax.scatter(xs, ys_med, alpha=0.65, label="window median latency")
    # Draw trend line by sorting throughput
    pts = sorted(zip(xs, ys_med), key=lambda p: p[0])
    ax.plot([p[0] for p in pts], [p[1] for p in pts], linewidth=1.2, alpha=0.8)
    ax.set_xlabel(f"Coordination throughput (req/s per {int(window_sec)}s window)")
    ax.set_ylabel("Median coordination latency (ms)")
    ax.set_title("Throughput vs Latency (Windowed)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "coord_throughput_vs_latency.png"), dpi=160)
    plt.close(fig)

def _plot_coordination_flow(summary: Dict[str, Any], out_dir: str, plt) -> None:
    cf = dict(summary.get("coordination_flow", {}) or {})
    req = int(cf.get("reservation_req_sent", 0) or 0)
    resp = int(cf.get("reservation_resp_recv", 0) or 0)
    assoc_c = int(cf.get("association_created", 0) or 0)
    assoc_r = int(cf.get("association_released", 0) or 0)
    adv_route = int(cf.get("route_advice_published", 0) or 0)
    adv_tls = int(cf.get("intersection_advice_published", 0) or 0)
    vals = [req, resp, assoc_c, assoc_r, adv_route, adv_tls]
    if not any(v > 0 for v in vals):
        return
    labels = ["req_sent", "resp_recv", "assoc_created", "assoc_released", "route_advice", "tls_advice"]
    fig = plt.figure(figsize=(9, 4.5))
    ax = fig.add_subplot(111)
    ax.bar(labels, vals)
    ax.set_ylabel("Count")
    ratio = cf.get("req_resp_ratio", None)
    ratio_txt = "-" if ratio is None else f"{float(ratio):.2f}"
    ax.set_title(f"Coordination Flow Counts (req/resp ratio={ratio_txt})")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "coordination_flow_counts.png"), dpi=160)
    plt.close(fig)


def _plot_message_load_by_service(summary: Dict[str, Any], out_dir: str, plt, exclude: List[str]) -> None:
    by_svc = dict(summary.get("message_volume_by_service", {}) or {})
    if by_svc:
        items = []
        for svc, rec in by_svc.items():
            d = dict(rec or {})
            msgs = int(_f(d.get("messages"), 0.0))
            if msgs <= 0:
                continue
            items.append((_norm_interaction_node(str(svc)), msgs))
        if not items:
            return
        agg: Dict[str, int] = {}
        for s, c in items:
            agg[s] = agg.get(s, 0) + int(c)
        items2 = sorted(agg.items(), key=lambda kv: kv[1], reverse=True)
        labels = [k for k, _ in items2]
        vals = [int(v) for _, v in items2]
        fig = plt.figure(figsize=(max(8, 0.9 * len(labels)), 4.5))
        ax = fig.add_subplot(111)
        ax.bar(list(range(len(labels))), vals)
        ax.set_xticks(list(range(len(labels))))
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_ylabel("Message count")
        ax.set_title("MQTT Message Load by Producer Service")
        ax.grid(axis="y", linestyle="--", alpha=0.4)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "message_load_by_service.png"), dpi=160)
        plt.close(fig)
        return

    # Backward-compatible fallback from event families.
    events = dict(summary.get("event_counts", {}) or {})
    if not events:
        return
    ex = set(exclude or [])
    # Group by first token before dot for canonical events; fallback to raw key.
    svc_counts: Dict[str, int] = {}
    for evt, c in events.items():
        if str(evt) in ex:
            continue
        e = str(evt)
        head = e.split(".")[0] if "." in e else e.split("_")[0]
        svc_counts[head] = svc_counts.get(head, 0) + int(c)
    if not svc_counts:
        return
    items = sorted(svc_counts.items(), key=lambda kv: kv[1], reverse=True)
    labels = [k for k, _ in items]
    vals = [int(v) for _, v in items]
    fig = plt.figure(figsize=(max(8, 0.9 * len(labels)), 4.5))
    ax = fig.add_subplot(111)
    ax.bar(list(range(len(labels))), vals)
    ax.set_xticks(list(range(len(labels))))
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("Message count")
    ax.set_title("Federation Message Load by Service Family")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "message_load_by_service.png"), dpi=160)
    plt.close(fig)


def _plot_paper_panel(summary: Dict[str, Any], metrics_rows: List[Dict[str, str]], out_dir: str, plt, exclude: List[str]) -> None:
    # 4-panel figure inspired by middleware papers:
    # (a) operational phases, (b) decision delays, (c) filtered service event mix, (d) discovery hit ratio.
    phase_all = _phase_rows(summary)
    phase_rows = [r for r in phase_all if r[0] != "active_to_suspended_ms"]
    dec_keep = ["T_discovery_e2e", "T_assoc_setup", "T_coord_req_resp", "T_advice_uptake", "T_time_to_active", "T_onboard"]
    dec_rows = [r for r in metrics_rows if str(r.get("domain", "")) == "wall_ms" and str(r.get("metric", "")) in dec_keep]
    dec_rows.sort(key=lambda r: dec_keep.index(str(r.get("metric", ""))) if str(r.get("metric", "")) in dec_keep else 999)
    events = dict(summary.get("event_counts", {}) or {})
    ex = set(exclude or [])
    event_items = sorted([(k, v) for k, v in events.items() if k not in ex], key=lambda kv: int(kv[1]), reverse=True)[:10]
    ratio = dict(summary.get("discovery_hit_ratio", {}) or {})
    total = int(ratio.get("total", 0) or 0)
    hits = int(ratio.get("hits", 0) or 0)

    if not (phase_rows or dec_rows or event_items or total > 0):
        return

    fig = plt.figure(figsize=(15, 8))
    ax1 = fig.add_subplot(2, 2, 1)
    if phase_rows:
        labels = [r[0] for r in phase_rows]
        vals = [r[1] for r in phase_rows]
        ax1.bar(list(range(len(labels))), vals)
        ax1.set_xticks(list(range(len(labels))))
        ax1.set_xticklabels(labels, rotation=25, ha="right", fontsize=8)
        ax1.set_ylabel("ms")
    ax1.set_title("(a) Operational Phases")
    ax1.grid(axis="y", linestyle="--", alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    if dec_rows:
        labels = [str(r.get("metric", "")) for r in dec_rows]
        vals = [_f(r.get("mean_ms")) for r in dec_rows]
        ax2.bar(list(range(len(labels))), vals)
        ax2.set_xticks(list(range(len(labels))))
        ax2.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
        ax2.set_ylabel("ms")
    ax2.set_title("(b) Decision-Path E2E Delay")
    ax2.grid(axis="y", linestyle="--", alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    if event_items:
        labels = [k for k, _ in event_items]
        vals = [int(v) for _, v in event_items]
        ax3.bar(list(range(len(labels))), vals)
        ax3.set_xticks(list(range(len(labels))))
        ax3.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
        ax3.set_ylabel("count")
    ax3.set_title("(c) Federation Event Mix")
    ax3.grid(axis="y", linestyle="--", alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    if total > 0:
        misses = max(0, total - hits)
        ax4.bar(["hits", "misses"], [hits, misses])
        ax4.set_ylabel("queries")
        ax4.set_title(f"(d) Discovery Hit Ratio ({hits}/{total})")
    else:
        ax4.set_title("(d) Discovery Hit Ratio")
    ax4.grid(axis="y", linestyle="--", alpha=0.3)

    fig.suptitle("Federation Middleware Performance Overview", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(os.path.join(out_dir, "paper_style_federation_panel.png"), dpi=170)
    plt.close(fig)


def _plot_intersection_metric_bars(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_role = dict(summary.get("metrics_by_role", {}) or {})
    intr = dict(by_role.get("intersection", {}) or {})
    if not intr:
        return
    rows = []
    for metric, d in intr.items():
        wall = dict(d.get("wall_ms", {}) or {})
        n = int(wall.get("n", 0) or 0)
        if n <= 0:
            continue
        rows.append(
            {
                "metric": str(metric),
                "mean_ms": _f(wall.get("mean_ms")),
                "p95_ms": _f(wall.get("p95_ms")),
            }
        )
    if not rows:
        return
    rows.sort(key=lambda r: r["mean_ms"], reverse=True)
    metrics = [r["metric"] for r in rows]
    means = [r["mean_ms"] for r in rows]
    p95s = [r["p95_ms"] for r in rows]
    fig = plt.figure(figsize=(max(10, 0.7 * len(metrics)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(metrics)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(metrics, rotation=35, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Intersection Perspective: Federation Latency Metrics")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_metrics_bar_wall_ms.png"), dpi=160)
    plt.close(fig)


def _plot_coord_latency_by_tls(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("metrics_by_tls", {}) or {})
    rows = []
    for tls, mm in by_tls.items():
        coord = dict(mm.get("T_coord_req_resp", {}) or {})
        wall = dict(coord.get("wall_ms", {}) or {})
        n = int(wall.get("n", 0) or 0)
        if n <= 0:
            continue
        rows.append(
            {
                "tls": str(tls),
                "mean_ms": _f(wall.get("mean_ms")),
                "p95_ms": _f(wall.get("p95_ms")),
                "n": n,
            }
        )
    if not rows:
        return
    rows.sort(key=lambda r: r["mean_ms"], reverse=True)
    rows = rows[:20]
    labels = [r["tls"] for r in rows]
    means = [r["mean_ms"] for r in rows]
    p95s = [r["p95_ms"] for r in rows]
    fig = plt.figure(figsize=(max(10, 0.7 * len(labels)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("Coordination latency (ms)")
    ax.set_title("Top Intersections by Coordination Latency (T_coord_req_resp)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_coord_latency_by_tls.png"), dpi=160)
    plt.close(fig)


def _top_metrics(samples: List[Dict[str, str]], k: int = 6) -> List[str]:
    counts: Dict[str, int] = {}
    for s in samples:
        m = str(s.get("metric", ""))
        counts[m] = counts.get(m, 0) + 1
    return [m for m, _ in sorted(counts.items(), key=lambda kv: kv[1], reverse=True)[:k]]


def _plot_latency_lines(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    if not samples:
        return
    keep = set(_top_metrics(samples, 6))
    seq: Dict[str, List[Tuple[int, float]]] = {}
    for idx, s in enumerate(samples):
        m = str(s.get("metric", ""))
        if m not in keep:
            continue
        y = _f(s.get("latency_wall_ms"))
        seq.setdefault(m, []).append((idx, y))
    if not seq:
        return

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111)
    for m, pts in sorted(seq.items()):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, label=m, linewidth=1.5)
    ax.set_xlabel("Sample index (arrival order)")
    ax.set_ylabel("Latency wall ms")
    ax.set_title("Latency Time Series (Top Metrics)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "latency_timeseries_top.png"), dpi=160)
    plt.close(fig)


def _plot_latency_cdf(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    if not samples:
        return
    keep = set(_top_metrics(samples, 6))
    grouped: Dict[str, List[float]] = {}
    for s in samples:
        m = str(s.get("metric", ""))
        if m not in keep:
            continue
        grouped.setdefault(m, []).append(_f(s.get("latency_wall_ms")))

    if not grouped:
        return

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111)
    for m, vals in sorted(grouped.items()):
        xs = sorted(vals)
        if not xs:
            continue
        ys = [(i + 1) / len(xs) for i in range(len(xs))]
        ax.plot(xs, ys, label=m, linewidth=1.5)
    ax.set_xlabel("Latency wall ms")
    ax.set_ylabel("CDF")
    ax.set_title("Latency CDF (Top Metrics)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "latency_cdf_top.png"), dpi=160)
    plt.close(fig)


def _plot_intersection_coord_timeseries(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    filt = [
        s
        for s in samples
        if str(s.get("metric", "")) == "T_coord_req_resp" and str(s.get("tls_id", "")).strip()
    ]
    if not filt:
        return
    # Keep top TLS by sample count
    counts: Dict[str, int] = {}
    for s in filt:
        tls = str(s.get("tls_id", ""))
        counts[tls] = counts.get(tls, 0) + 1
    keep = {tls for tls, _ in sorted(counts.items(), key=lambda kv: kv[1], reverse=True)[:6]}

    grouped: Dict[str, List[Tuple[float, float]]] = {}
    ts0 = None
    # Use wall timestamps when available; fallback to sample index.
    ts_candidates = [_f(s.get("end_ts_wall_ms"), -1.0) for s in filt if _f(s.get("end_ts_wall_ms"), -1.0) >= 0]
    if ts_candidates:
        ts0 = min(ts_candidates)
    for idx, s in enumerate(filt):
        tls = str(s.get("tls_id", ""))
        if tls not in keep:
            continue
        x = float(idx)
        ts = _f(s.get("end_ts_wall_ms"), -1.0)
        if ts0 is not None and ts >= 0:
            x = (ts - ts0) / 1000.0
        grouped.setdefault(tls, []).append((x, _f(s.get("latency_wall_ms"))))

    if not grouped:
        return
    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111)
    for tls, pts in sorted(grouped.items()):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, label=tls, linewidth=1.5)
    ax.set_xlabel("Time (s)" if ts0 is not None else "Coordination sample index")
    ax.set_ylabel("Latency wall ms")
    ax.set_title("Intersection Coordination Latency Over Time (Top TLS)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_coord_latency_timeseries.png"), dpi=160)
    plt.close(fig)


def _plot_ev_transit_tls_sequence(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    rows = []
    for s in samples:
        if str(s.get("metric", "")) != "T_coord_req_resp":
            continue
        tls = str(s.get("tls_id", "") or "").strip()
        if not tls:
            tls = str(s.get("dt_id", "") or "").strip()
        if not tls:
            continue
        st_sim = _f(s.get("start_ts_sim_s"), -1.0)
        en_sim = _f(s.get("end_ts_sim_s"), -1.0)
        st_wall = _f(s.get("start_ts_wall_ms"), -1.0)
        en_wall = _f(s.get("end_ts_wall_ms"), -1.0)
        lat = _f(s.get("latency_wall_ms"), -1.0)
        if lat < 0:
            continue
        if st_sim >= 0 and en_sim >= 0:
            st = st_sim
            en = en_sim
        elif st_wall >= 0 and en_wall >= 0:
            st = st_wall / 1000.0
            en = en_wall / 1000.0
        else:
            continue
        if en < st:
            en = st
        rows.append((tls, st, en, lat))
    if not rows:
        return

    agg: Dict[str, Dict[str, Any]] = {}
    for tls, st, en, lat in rows:
        rec = agg.setdefault(tls, {"first": st, "last": en, "lats": [], "n": 0})
        rec["first"] = min(float(rec["first"]), float(st))
        rec["last"] = max(float(rec["last"]), float(en))
        rec["lats"].append(float(lat))
        rec["n"] = int(rec["n"]) + 1

    items = sorted(agg.items(), key=lambda kv: float(kv[1]["first"]))
    if len(items) > 18:
        items = items[:18]
    labels = [k for k, _ in items]
    t0 = float(items[0][1]["first"])
    starts = [float(v["first"]) - t0 for _, v in items]
    ends = [float(v["last"]) - t0 for _, v in items]
    spans = [max(0.001, ends[i] - starts[i]) for i in range(len(items))]
    med_lat = [statistics.median([float(x) for x in v["lats"]]) if v["lats"] else 0.0 for _, v in items]
    ns = [int(v["n"]) for _, v in items]

    fig = plt.figure(figsize=(14, 8))
    ax1 = fig.add_subplot(2, 1, 1)
    ys = list(range(len(labels)))
    ax1.barh(ys, spans, left=starts, color="#72B7B2")
    ax1.set_yticks(ys)
    ax1.set_yticklabels(labels, fontsize=8)
    ax1.invert_yaxis()
    ax1.set_xlabel("Elapsed time (s)")
    ax1.set_title("Approx EV Transit Sequence Across Coordinated TLS")
    ax1.grid(axis="x", linestyle="--", alpha=0.35)

    ax2 = fig.add_subplot(2, 1, 2)
    sizes = [40 + min(240, 10 * n) for n in ns]
    sc = ax2.scatter(starts, med_lat, s=sizes, c=ns, cmap="viridis", alpha=0.85, edgecolors="black", linewidths=0.3)
    for i, lbl in enumerate(labels):
        if i < 12:
            ax2.annotate(lbl, (starts[i], med_lat[i]), fontsize=7)
    ax2.set_xlabel("First coordination time since start (s)")
    ax2.set_ylabel("Median coordination latency (ms)")
    ax2.set_title("TLS Entry Order vs Latency (bubble size = coordination count)")
    ax2.grid(True, linestyle="--", alpha=0.35)
    fig.colorbar(sc, ax=ax2, fraction=0.046, pad=0.03, label="coordination count")

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "ev_transit_tls_sequence.png"), dpi=170)
    plt.close(fig)


def _plot_fnm_pipeline_boxplot(samples: List[Dict[str, str]], out_dir: str, plt) -> None:
    metric_defs = [
        ("T_ev_req_to_intersection_apply", "state->actuation"),
        ("T_ev_req_to_signal_change", "state->signal"),
        ("T_coord_req_to_decision", "req->decision"),
        ("T_coord_req_resp", "req->resp"),
        ("T_coord_req_to_apply", "req->apply"),
        ("T_ev_request_age", "request_age"),
    ]
    data = []
    labels = []
    for metric, label in metric_defs:
        vals = _metric_values(samples, metric)
        if vals:
            data.append(vals)
            labels.append(label)
    if not data:
        return

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111)
    ax.boxplot(data, labels=labels, showfliers=False)
    ax.set_ylabel("Latency (ms)")
    ax.set_title("FNM Integration Pipeline Latency (Boxplot)")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fnm_pipeline_latency_boxplot.png"), dpi=170)
    plt.close(fig)


def _plot_coordination_sequence_timeline(summary: Dict[str, Any], out_dir: str, plt) -> None:
    rows = list(summary.get("coordination_timeline_events", []) or [])
    if not rows:
        return

    ordered_types = [
        "ev.request.in",
        "ev.request.received",
        "federation.reservation.req.sent",
        "federation.reservation.req.recv",
        "federation.reservation.req.decision",
        "federation.reservation.resp.recv",
        "intersection.plan.applied",
        "intersection.offer.applied",
        "tls.signal.change",
        "ev.pass.detected",
    ]
    type_rank = {k: i for i, k in enumerate(ordered_types)}
    filtered = [r for r in rows if str(r.get("event_type", "")) in type_rank]
    if not filtered:
        return

    # Keep timeline readable for long runs.
    max_pts = 1600
    if len(filtered) > max_pts:
        step = max(1, int(len(filtered) / float(max_pts)))
        filtered = filtered[::step]

    sim_vals = [_f(r.get("ts_sim_s"), -1.0) for r in filtered]
    has_sim = any(v >= 0 for v in sim_vals)
    if has_sim:
        valid = [v for v in sim_vals if v >= 0]
        t0 = min(valid) if valid else 0.0
        xs = [(v - t0) if v >= 0 else float("nan") for v in sim_vals]
        xlbl = "Simulation time since first event (s)"
    else:
        wall_vals = [_f(r.get("ts_wall_ms"), 0.0) for r in filtered]
        t0 = min(wall_vals) if wall_vals else 0.0
        xs = [(v - t0) / 1000.0 for v in wall_vals]
        xlbl = "Wall time since first event (s)"

    ys = [type_rank[str(r.get("event_type", ""))] for r in filtered]
    tls_vals = [str(r.get("tls_id", "") or "n/a") for r in filtered]
    palette = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2", "#B279A2", "#FF9DA6", "#9D755D"]
    tls_order: List[str] = []
    tls_color: Dict[str, str] = {}
    colors: List[str] = []
    for tls in tls_vals:
        if tls not in tls_color:
            tls_order.append(tls)
            tls_color[tls] = palette[(len(tls_order) - 1) % len(palette)]
        colors.append(tls_color[tls])

    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(111)
    ax.scatter(xs, ys, c=colors, s=16, alpha=0.78, edgecolors="none")
    ax.set_yticks(list(range(len(ordered_types))))
    ax.set_yticklabels(ordered_types)
    ax.set_xlabel(xlbl)
    ax.set_title("EV-Intersection Coordination Sequence Timeline")
    ax.grid(axis="x", linestyle="--", alpha=0.35)

    # Small legend with top TLS by point count.
    tls_counts: Dict[str, int] = {}
    for tls in tls_vals:
        tls_counts[tls] = tls_counts.get(tls, 0) + 1
    top_tls = sorted(tls_counts.items(), key=lambda kv: kv[1], reverse=True)[:8]
    if top_tls:
        handles = []
        labels = []
        for tls, n in top_tls:
            h = ax.scatter([], [], c=tls_color[tls], s=28)
            handles.append(h)
            labels.append(f"{tls} ({n})")
        ax.legend(handles, labels, title="Top TLS", fontsize=8, loc="upper right")

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "coordination_sequence_timeline.png"), dpi=170)
    plt.close(fig)


def _plot_intersection_comm_overhead(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("communication_overhead_by_tls", {}) or {})
    rows = []
    for tls, c in by_tls.items():
        d = dict(c or {})
        total = int(
            d.get("reservation_req_sent", 0)
            + d.get("reservation_resp_recv", 0)
            + d.get("assoc_created", 0)
            + d.get("assoc_released", 0)
            + d.get("route_advice_published", 0)
            + d.get("intersection_advice_published", 0)
        )
        if total <= 0:
            continue
        rows.append((str(tls), d, total))
    if not rows:
        return
    rows.sort(key=lambda x: x[2], reverse=True)
    rows = rows[:15]
    labels = [r[0] for r in rows]
    req = [int(r[1].get("reservation_req_sent", 0) or 0) for r in rows]
    resp = [int(r[1].get("reservation_resp_recv", 0) or 0) for r in rows]
    ac = [int(r[1].get("assoc_created", 0) or 0) for r in rows]
    ar = [int(r[1].get("assoc_released", 0) or 0) for r in rows]
    ra = [int(r[1].get("route_advice_published", 0) or 0) for r in rows]
    ta = [int(r[1].get("intersection_advice_published", 0) or 0) for r in rows]

    fig = plt.figure(figsize=(max(10, 0.7 * len(labels)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    b0 = [0] * len(labels)
    ax.bar(xs, req, label="req_sent")
    b1 = [b0[i] + req[i] for i in range(len(labels))]
    ax.bar(xs, resp, bottom=b1, label="resp_recv")
    b2 = [b1[i] + resp[i] for i in range(len(labels))]
    ax.bar(xs, ac, bottom=b2, label="assoc_created")
    b3 = [b2[i] + ac[i] for i in range(len(labels))]
    ax.bar(xs, ar, bottom=b3, label="assoc_released")
    b4 = [b3[i] + ar[i] for i in range(len(labels))]
    ax.bar(xs, ra, bottom=b4, label="route_advice")
    b5 = [b4[i] + ra[i] for i in range(len(labels))]
    ax.bar(xs, ta, bottom=b5, label="tls_advice")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("Message count")
    ax.set_title("Intersection Communication Overhead (Top TLS)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_comm_overhead_by_tls.png"), dpi=160)
    plt.close(fig)


def _plot_intersection_comp_overhead(summary: Dict[str, Any], out_dir: str, plt) -> None:
    by_tls = dict(summary.get("computation_overhead_by_tls", {}) or {})
    rows = []
    for tls, m in by_tls.items():
        d = dict(m or {})
        direct_means: List[float] = []
        direct_p95s: List[float] = []
        direct_n = 0
        for mk in ("C_intersection_tick_compute", "C_intersection_refine_compute", "C_intersection_apply_compute"):
            st = dict(d.get(mk, {}) or {})
            n = int(st.get("n", 0) or 0)
            if n <= 0:
                continue
            direct_n += n
            direct_means.append(_f(st.get("mean_ms")))
            direct_p95s.append(_f(st.get("p95_ms")))
        if direct_means:
            rows.append((str(tls), statistics.fmean(direct_means), max(direct_p95s), direct_n))
            continue

        # Fallback when direct compute events are unavailable.
        st = dict(d.get("T_coord_req_resp", {}) or {})
        n = int(st.get("n", 0) or 0)
        if n > 0:
            rows.append((str(tls), _f(st.get("mean_ms")), _f(st.get("p95_ms")), n))
    if not rows:
        return
    rows.sort(key=lambda x: x[1], reverse=True)
    rows = rows[:15]
    labels = [r[0] for r in rows]
    means = [r[1] for r in rows]
    p95s = [r[2] for r in rows]
    fig = plt.figure(figsize=(max(10, 0.7 * len(labels)), 6))
    ax = fig.add_subplot(111)
    xs = list(range(len(labels)))
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], means, width=w, label="mean_ms")
    ax.bar([x + w / 2.0 for x in xs], p95s, width=w, label="p95_ms")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Intersection Computation Overhead (Direct, fallback to E2E)")
    ax.grid(axis="y", linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "intersection_comp_overhead_by_tls.png"), dpi=160)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot federation metrics from extracted outputs")
    ap.add_argument("--metrics-dir", required=True, help="directory with summary.json and CSVs")
    ap.add_argument("--out-dir", default="", help="output plot directory (default: <metrics-dir>/plots)")
    ap.add_argument(
        "--exclude-events",
        default="metrics_pub,catalog.refresh,membership_catalog_seen,state_pub,geo_svg_skip,assoc_state,warmup_seen",
        help="comma-separated event names to omit from event-mix charts",
    )
    args = ap.parse_args()

    metrics_dir = os.path.abspath(str(args.metrics_dir))
    out_dir = os.path.abspath(str(args.out_dir or os.path.join(metrics_dir, "plots")))
    _ensure_dir(out_dir)

    summary_path = os.path.join(metrics_dir, "summary.json")
    metrics_csv = os.path.join(metrics_dir, "metrics_summary.csv")
    samples_csv = os.path.join(metrics_dir, "latency_samples.csv")

    missing = [p for p in [summary_path, metrics_csv, samples_csv] if not os.path.exists(p)]
    if missing:
        print(json.dumps({"status": "error", "missing_files": missing}))
        return 2

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(
            json.dumps(
                {
                    "status": "error",
                    "reason": "matplotlib_missing",
                    "message": str(e),
                    "hint": "Install matplotlib (e.g., pip install matplotlib) and rerun.",
                }
            )
        )
        return 3

    summary = _read_json(summary_path)
    metrics_rows = _read_csv(metrics_csv)
    samples_rows = _read_csv(samples_csv)
    exclude_events = [x.strip() for x in str(args.exclude_events).split(",") if x.strip()]

    _plot_metric_bars(metrics_rows, out_dir, "wall_ms", plt)
    _plot_metric_bars(metrics_rows, out_dir, "sim_ms", plt)
    _plot_operational_phases(summary, out_dir, plt)
    _plot_lifecycle_waterfall(summary, out_dir, plt)
    _plot_membership_startup_phases(summary, out_dir, plt)
    _plot_active_dwell(summary, out_dir, plt)
    _plot_active_members_timeline(summary, out_dir, plt)
    _plot_updated_connectivity_lifecycle_timeline(summary, out_dir, plt)
    _plot_updated_fcm_discovery_runtime(summary, out_dir, plt)
    _plot_updated_peer_connectivity_graph(summary, out_dir, plt)
    _plot_updated_ev_peer_selection_evolution(summary, out_dir, plt)
    _plot_updated_adaptive_binding_evolution(summary, out_dir, plt)
    _plot_updated_coord_latency_clock_split(summary, out_dir, plt)
    _plot_updated_coord_churn_queue(summary, out_dir, plt)
    _plot_decision_path_latency(metrics_rows, out_dir, plt)
    _plot_decision_path_boxplots(samples_rows, out_dir, plt)
    _plot_coordination_flow(summary, out_dir, plt)
    _plot_message_load_by_service(summary, out_dir, plt, exclude_events)
    _plot_event_counts(summary, out_dir, plt, exclude_events)
    _plot_event_counts_by_role(summary, out_dir, plt, exclude_events)
    _plot_discovery_ratio(summary, out_dir, plt)
    _plot_discovery_funnel(summary, out_dir, plt)
    _plot_latency_lines(samples_rows, out_dir, plt)
    _plot_latency_cdf(samples_rows, out_dir, plt)
    _plot_intersection_metric_bars(summary, out_dir, plt)
    _plot_coord_latency_by_tls(summary, out_dir, plt)
    _plot_coord_latency_boxplot_by_tls(samples_rows, out_dir, plt)
    _plot_coord_slo_by_tls(samples_rows, out_dir, plt)
    _plot_coord_load_vs_latency(summary, out_dir, plt)
    _plot_intersection_coord_timeseries(samples_rows, out_dir, plt)
    _plot_ev_transit_tls_sequence(samples_rows, out_dir, plt)
    _plot_fnm_pipeline_boxplot(samples_rows, out_dir, plt)
    _plot_coordination_sequence_timeline(summary, out_dir, plt)
    _plot_intersection_comm_overhead(summary, out_dir, plt)
    _plot_intersection_comp_overhead(summary, out_dir, plt)
    _plot_compute_metric_distributions(summary, out_dir, plt)
    _plot_compute_vs_comm_by_tls(summary, out_dir, plt)
    _plot_role_latency_boxplots(samples_rows, out_dir, plt)
    _plot_coord_throughput_vs_latency(samples_rows, out_dir, plt)
    _plot_association_lifecycle(summary, out_dir, plt)
    _plot_ev_advice_flow(summary, out_dir, plt)
    _plot_effectiveness_scorecard(summary, out_dir, plt)
    _plot_message_volume_panel(summary, out_dir, plt)
    _plot_service_interaction_matrix(summary, out_dir, plt)
    _plot_service_interaction_top_edges(summary, out_dir, plt)
    _plot_service_interaction_planes_panel(summary, out_dir, plt)
    _plot_mqtt_topic_profile(summary, out_dir, plt)
    _plot_mqtt_topic_origin_mix(summary, out_dir, plt)
    _plot_mqtt_service_topic_heatmap(summary, out_dir, plt)
    _plot_mqtt_topic_timeline(summary, out_dir, plt)
    _plot_mqtt_edge_topic_breakdown(summary, out_dir, plt)
    _plot_mqtt_analysis_panel(summary, out_dir, plt)
    _plot_role_overhead_rates(summary, out_dir, plt)
    _plot_dt_normalized_overhead(summary, out_dir, plt)
    _plot_dt_cost_scatter(summary, out_dir, plt)
    _plot_ev_effectiveness_panel(summary, out_dir, plt)
    _plot_category_intersection_nodes_panel(summary, samples_rows, out_dir, plt)
    _plot_category_ev_intersection_panel(summary, samples_rows, out_dir, plt)
    _plot_category_orchestrator_intersection_panel(summary, out_dir, plt)
    _plot_category_middleware_core_panel(summary, out_dir, plt)
    _plot_category_collaborative_decision_panel(summary, samples_rows, out_dir, plt)
    _plot_federation_storyline_panel(summary, samples_rows, out_dir, plt)
    _plot_fnm_integration_panel(summary, out_dir, plt)
    _plot_fnm_overhead_panel(summary, out_dir, plt)
    _plot_coordination_diagnostics_panel(summary, out_dir, plt)
    _write_dt_overhead_table(summary, out_dir)
    _write_category_plot_index(summary, out_dir)
    _plot_paper_panel(summary, metrics_rows, out_dir, plt, exclude_events)

    print(
        json.dumps(
            {
                "status": "ok",
                "plots_dir": out_dir,
                "generated": sorted(os.listdir(out_dir)),
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
