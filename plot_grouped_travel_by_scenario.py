#!/usr/bin/env python3
"""
Grouped bars per scenario (B0/B1/F2) from ev_matrix_results.csv.

Produces:
  - grouped_travel_time_by_scenario_4x3.png
  - grouped_travel_time_by_scenario.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple


def _scenario_sort_key(scn: str) -> Tuple[str, int]:
    # scenario_severe1k_d1000_r3 -> (severe1k, 3)
    density = "zzz"
    route = 999999
    try:
        parts = scn.split("_")
        # [scenario, severe1k, d1000, r3]
        if len(parts) >= 4:
            density = parts[1]
            rp = parts[-1]
            if rp.startswith("r"):
                route = int(rp[1:])
    except Exception:
        pass
    return (density, route)


def load_results(csv_path: Path) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    with csv_path.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            scn = str(row.get("scenario_id", "")).strip()
            mode = str(row.get("mode", "")).strip()
            if not scn or mode not in {"B0", "B1", "F2"}:
                continue
            try:
                travel = float(row.get("travel_time_s", ""))
            except Exception:
                continue
            out.setdefault(scn, {})[mode] = travel
    return out


def write_long_csv(data: Dict[str, Dict[str, float]], out_csv: Path) -> None:
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["scenario_id", "mode", "travel_time_s"])
        for scn in sorted(data.keys(), key=_scenario_sort_key):
            for mode in ("B0", "B1", "F2"):
                if mode in data[scn]:
                    w.writerow([scn, mode, data[scn][mode]])


def plot_grouped(data: Dict[str, Dict[str, float]], out_png: Path, title: str) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    scenarios: List[str] = [
        scn for scn in sorted(data.keys(), key=_scenario_sort_key)
        if all(m in data[scn] for m in ("B0", "B1", "F2"))
    ]
    if not scenarios:
        raise RuntimeError("No complete scenarios with B0/B1/F2 found.")

    x = np.arange(len(scenarios))
    w = 0.26
    b0 = [data[s]["B0"] for s in scenarios]
    b1 = [data[s]["B1"] for s in scenarios]
    f2 = [data[s]["F2"] for s in scenarios]

    fig, ax = plt.subplots(figsize=(12, 9))  # 4:3
    ax.bar(x - w, b0, width=w, label="B0", color="#8c564b")
    ax.bar(x, b1, width=w, label="B1", color="#1f77b4")
    ax.bar(x + w, f2, width=w, label="F2", color="#2ca02c")

    ax.set_title(title)
    ax.set_ylabel("EV Travel Time (s)")
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=30, ha="right")
    ax.grid(axis="y", alpha=0.25, linestyle="--")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Grouped travel-time bars by scenario and mode.")
    ap.add_argument("--runs-root", required=True, help="Matrix runs root containing ev_matrix_results.csv")
    ap.add_argument("--out-dir", required=True, help="Output directory")
    ap.add_argument(
        "--title",
        default="Travel Time by Scenario (B0/B1/F2)",
        help="Plot title",
    )
    args = ap.parse_args()

    runs_root = Path(args.runs_root)
    in_csv = runs_root / "ev_matrix_results.csv"
    if not in_csv.exists():
        raise FileNotFoundError(f"Missing ev_matrix_results.csv: {in_csv}")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    data = load_results(in_csv)
    write_long_csv(data, out_dir / "grouped_travel_time_by_scenario.csv")
    plot_grouped(data, out_dir / "grouped_travel_time_by_scenario_4x3.png", args.title)
    print(f"wrote: {out_dir / 'grouped_travel_time_by_scenario_4x3.png'}")
    print(f"wrote: {out_dir / 'grouped_travel_time_by_scenario.csv'}")


if __name__ == "__main__":
    main()

