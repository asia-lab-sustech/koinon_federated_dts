#!/usr/bin/env python3
"""
Generate paper-ready 4:3 grouped bar charts for the 1.5K scenario:
1) Total travel time by route and mode
2) Node-level waiting time by mode for one representative route

Mode labels:
- B0 -> FTCM (Fixed-Time Control Method)
- B1 -> LIDP (Locally Interoperable DT Preemption)
- F2 -> FCDP (Federated Coordinated DT Preemption)
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


MODE_ORDER = ["B0", "B1", "F2"]
MODE_LABEL = {
    "B0": "FTCM",
    "B1": "LIDP",
    "F2": "FCDP",
}
MODE_COLOR = {
    "B0": "#7f7f7f",
    "B1": "#1f77b4",
    "F2": "#ff7f0e",
}


def _annotate_bars(ax, bars, fontsize: int = 12) -> None:
    for b in bars:
        h = b.get_height()
        if h is None:
            continue
        try:
            hv = float(h)
        except Exception:
            continue
        if not math.isfinite(hv):
            continue
        ax.annotate(
            f"{hv:.1f}",
            xy=(b.get_x() + b.get_width() / 2.0, hv),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=fontsize,
            rotation=0,
        )


def _safe_float(v, default=float("nan")) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _read_results_csv(root_15k: str) -> List[dict]:
    p = os.path.join(root_15k, "ev_matrix_results.csv")
    with open(p, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _plot_travel_time_grouped(rows: List[dict], out_dir: str) -> str:
    routes = sorted({int(r["route_id"]) for r in rows})

    values: Dict[str, List[float]] = {m: [] for m in MODE_ORDER}
    for mode in MODE_ORDER:
        for rt in routes:
            rec = [
                r for r in rows
                if r.get("mode") == mode and int(r.get("route_id", -1)) == int(rt)
            ]
            values[mode].append(_safe_float(rec[0].get("travel_time_s")) if rec else float("nan"))

    fig = plt.figure(figsize=(12, 9), dpi=140)  # 4:3
    ax = fig.add_subplot(111)

    x = np.arange(len(routes), dtype=float)
    w = 0.24

    for i, mode in enumerate(MODE_ORDER):
        bars = ax.bar(
            x + (i - 1) * w,
            values[mode],
            width=w,
            label=MODE_LABEL[mode],
            color=MODE_COLOR[mode],
            edgecolor="black",
            linewidth=0.6,
        )
        _annotate_bars(ax, bars, fontsize=12)

    ax.set_title("1.5K Scenario: EV Total Travel Time by Route", fontsize=26, pad=14)
    ax.set_xlabel("Route", fontsize=22)
    ax.set_ylabel("Total Travel Time (s)", fontsize=22)
    ax.set_xticks(x)
    ax.set_xticklabels([str(r) for r in routes], fontsize=18)
    ax.tick_params(axis="y", labelsize=18)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(fontsize=16, loc="best")
    fig.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "paper_15k_total_travel_time_grouped_4x3.png")
    fig.savefig(out)
    plt.close(fig)
    return out


def _resolve_event_file(root_15k: str, scenario_id: str, density_label: str, route_id: int, mode: str) -> str:
    p1 = os.path.join(
        root_15k,
        "scenario_runs",
        scenario_id,
        "matrix_out",
        "runs",
        density_label,
        f"route_{route_id}",
        mode,
        f"fed_outcomes_{mode}_*.events.jsonl",
    )
    files = sorted(glob.glob(p1))
    if files:
        return files[-1]

    p2 = os.path.join(
        root_15k,
        "scenario_runs",
        scenario_id,
        "matrix_out",
        "runs",
        density_label,
        f"route_{route_id}",
        mode,
        "fed_outcomes.events.jsonl",
    )
    if os.path.exists(p2):
        return p2

    raise FileNotFoundError(f"No events jsonl found for route={route_id}, mode={mode}, scenario={scenario_id}")


def _extract_node_wait(events_jsonl: str) -> Dict[str, float]:
    """
    Node waiting proxy: first(ev.request.in) -> first(ev.pass.detected) per TLS node.
    """
    first_req: Dict[str, float] = {}
    first_pass: Dict[str, float] = {}

    with open(events_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except Exception:
                continue

            et = str(o.get("event_type", ""))
            tls = str(o.get("tls_id", "") or "")
            sim_t = _safe_float(o.get("sim_time"), default=float("nan"))
            if not tls or math.isnan(sim_t):
                continue

            if et == "ev.request.in" and tls not in first_req:
                first_req[tls] = sim_t
            elif et == "ev.pass.detected" and tls not in first_pass:
                first_pass[tls] = sim_t

    wait: Dict[str, float] = {}
    for tls, t_req in first_req.items():
        t_pass = first_pass.get(tls)
        if t_pass is None:
            continue
        dt = float(t_pass) - float(t_req)
        if dt >= 0:
            wait[tls] = dt
    return wait


def _plot_node_wait_grouped(rows: List[dict], root_15k: str, representative_route: int, out_dir: str) -> str:
    route_rows = [r for r in rows if int(r.get("route_id", -1)) == int(representative_route)]
    if not route_rows:
        raise RuntimeError(f"Route {representative_route} not present in ev_matrix_results.csv")

    scenario_id = str(route_rows[0]["scenario_id"])
    density_label = str(route_rows[0]["density_label"])

    wait_by_mode: Dict[str, Dict[str, float]] = {}
    for mode in MODE_ORDER:
        evf = _resolve_event_file(root_15k, scenario_id, density_label, representative_route, mode)
        wait_by_mode[mode] = _extract_node_wait(evf)

    # Node order follows FCDP first (if available), then union fallback.
    nodes = list(wait_by_mode.get("F2", {}).keys())
    if not nodes:
        nodes = sorted({n for d in wait_by_mode.values() for n in d.keys()})
    if not nodes:
        raise RuntimeError(f"No node-level waiting extracted for route {representative_route}")

    fig = plt.figure(figsize=(12, 9), dpi=140)  # 4:3
    ax = fig.add_subplot(111)

    x = np.arange(len(nodes), dtype=float)
    w = 0.24

    for i, mode in enumerate(MODE_ORDER):
        vals = [wait_by_mode[mode].get(n, np.nan) for n in nodes]
        bars = ax.bar(
            x + (i - 1) * w,
            vals,
            width=w,
            label=MODE_LABEL[mode],
            color=MODE_COLOR[mode],
            edgecolor="black",
            linewidth=0.6,
        )
        _annotate_bars(ax, bars, fontsize=10)

    ax.set_title(f"1.5K Scenario: Node-Level Average Waiting (Route {representative_route})", fontsize=24, pad=14)
    ax.set_xlabel("Intersection Node (TLS)", fontsize=22)
    ax.set_ylabel("Average Waiting Time per Node (s)", fontsize=22)
    ax.set_xticks(x)
    ax.set_xticklabels(nodes, rotation=35, ha="right", fontsize=14)
    ax.tick_params(axis="y", labelsize=18)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend(fontsize=16, loc="best")
    fig.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"paper_15k_node_wait_grouped_route_{representative_route}_4x3.png")
    fig.savefig(out)
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate 1.5K grouped bar charts (4:3) for paper")
    ap.add_argument("--root-15k", required=True, help="Path to ev_matrix_runs_stress_measured_short_clean_1_5K")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--representative-route", type=int, default=4)
    args = ap.parse_args()

    rows = _read_results_csv(args.root_15k)
    p1 = _plot_travel_time_grouped(rows, args.out_dir)
    p2 = _plot_node_wait_grouped(rows, args.root_15k, int(args.representative_route), args.out_dir)

    print("Generated:")
    print(p1)
    print(p2)


if __name__ == "__main__":
    main()
