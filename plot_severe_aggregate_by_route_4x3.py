#!/usr/bin/env python3
"""
Aggregate severe-congestion travel time by route across multiple run roots.

Expected input:
  each runs root contains `ev_matrix_results.csv` with scenario_id/mode/travel_time_s.

Output:
  - severe_aggregate_by_route.csv
  - severe_aggregate_by_route_4x3.png
"""

from __future__ import annotations

import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean
from typing import DefaultDict, Dict, List, Tuple


ROUTE_RE = re.compile(r"_r(\d+)$")
MODES = ("B0", "B1", "F2")
MODE_LABEL = {"B0": "FTCM", "B1": "LIDP", "F2": "FCDP"}
MODE_COLOR = {"B0": "#8c564b", "B1": "#1f77b4", "F2": "#2ca02c"}


def _route_from_scenario_id(scenario_id: str) -> int:
    m = ROUTE_RE.search(str(scenario_id or "").strip())
    if not m:
        return -1
    try:
        return int(m.group(1))
    except Exception:
        return -1


def _load_results(csv_path: Path) -> Dict[Tuple[int, str], List[float]]:
    data: DefaultDict[Tuple[int, str], List[float]] = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        for row in r:
            mode = str(row.get("mode", "")).strip()
            if mode not in MODES:
                continue
            route_id = _route_from_scenario_id(str(row.get("scenario_id", "")))
            if route_id < 0:
                continue
            try:
                travel = float(row.get("travel_time_s", ""))
            except Exception:
                continue
            if travel <= 0:
                continue
            data[(route_id, mode)].append(travel)
    return dict(data)


def _aggregate_across_roots(roots: List[Path]) -> Dict[int, Dict[str, float]]:
    per_root: List[Dict[Tuple[int, str], List[float]]] = []
    for root in roots:
        in_csv = root / "ev_matrix_results.csv"
        if not in_csv.exists():
            raise FileNotFoundError(f"Missing ev_matrix_results.csv: {in_csv}")
        per_root.append(_load_results(in_csv))

    merged_vals: DefaultDict[Tuple[int, str], List[float]] = defaultdict(list)
    for root_data in per_root:
        for key, vals in root_data.items():
            if vals:
                # First average inside one root, then aggregate across roots.
                merged_vals[key].append(mean(vals))

    out: Dict[int, Dict[str, float]] = {}
    for (route_id, mode), vals in merged_vals.items():
        out.setdefault(route_id, {})[mode] = float(mean(vals))
    return out


def _write_csv(data: Dict[int, Dict[str, float]], out_csv: Path) -> None:
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["route_id", "B0", "B1", "F2"])
        for route_id in sorted(data.keys()):
            row = data[route_id]
            w.writerow(
                [
                    route_id,
                    row.get("B0", ""),
                    row.get("B1", ""),
                    row.get("F2", ""),
                ]
            )


def _annotate(ax, bars) -> None:
    for b in bars:
        h = float(b.get_height())
        ax.annotate(
            f"{h:.1f}",
            xy=(b.get_x() + b.get_width() / 2.0, h),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=11,
        )


def _plot(data: Dict[int, Dict[str, float]], out_png: Path, title: str) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    routes = [r for r in sorted(data.keys()) if all(m in data[r] for m in MODES)]
    if not routes:
        raise RuntimeError("No complete routes found with B0/B1/F2.")

    x = np.arange(len(routes), dtype=float)
    w = 0.24

    fig, ax = plt.subplots(figsize=(12, 9))  # 4:3

    b0 = [data[r]["B0"] for r in routes]
    b1 = [data[r]["B1"] for r in routes]
    f2 = [data[r]["F2"] for r in routes]

    bars_b0 = ax.bar(x - w, b0, width=w, label=MODE_LABEL["B0"], color=MODE_COLOR["B0"])
    bars_b1 = ax.bar(x, b1, width=w, label=MODE_LABEL["B1"], color=MODE_COLOR["B1"])
    bars_f2 = ax.bar(x + w, f2, width=w, label=MODE_LABEL["F2"], color=MODE_COLOR["F2"])
    _annotate(ax, bars_b0)
    _annotate(ax, bars_b1)
    _annotate(ax, bars_f2)

    ax.set_title(title, fontsize=18)
    ax.set_xlabel("Route", fontsize=15)
    ax.set_ylabel("EV Travel Time (s)", fontsize=15)
    ax.set_xticks(x)
    ax.set_xticklabels([str(r) for r in routes], fontsize=13)
    ax.tick_params(axis="y", labelsize=12)
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    ax.legend(fontsize=13, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=200)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot severe aggregate travel-time by route (4:3).")
    ap.add_argument(
        "--runs-root",
        action="append",
        required=True,
        help="Run root containing ev_matrix_results.csv. Repeat this arg for 1K/1.5K/2K.",
    )
    ap.add_argument("--out-dir", required=True, help="Output directory")
    ap.add_argument(
        "--title",
        default="Severe Congestion Aggregate (1K + 1.5K + 2K)",
        help="Plot title",
    )
    args = ap.parse_args()

    roots = [Path(p).expanduser().resolve() for p in args.runs_root]
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    data = _aggregate_across_roots(roots)
    out_csv = out_dir / "severe_aggregate_by_route.csv"
    out_png = out_dir / "severe_aggregate_by_route_4x3.png"
    _write_csv(data, out_csv)
    _plot(data, out_png, title=str(args.title))
    print(f"wrote: {out_csv}")
    print(f"wrote: {out_png}")


if __name__ == "__main__":
    main()
