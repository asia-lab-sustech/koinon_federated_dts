#!/usr/bin/env python3
"""Aggregate multi-seed EV results (excluding 2K by default) and plot final paper charts.

Outputs under <results-root>/compiled_summary:
  - compiled_route_means.csv
  - compiled_scenario_mode_means.csv
  - compiled_scenario_gains.csv
  - compiled_global_gain.csv
  - compiled_route_mode_table_<scenario>.csv
  - plot_mean_travel_by_route_0.5K.png
  - plot_mean_travel_by_route_1K.png
  - plot_mean_travel_by_route_1.5K.png
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCENARIO_MAP = {
    "moderate05k": "0.5K",
    "severe1k": "1K",
    "severe1p5k": "1.5K",
    "severe2k": "2K",
}


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def _write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> None:
    ap = argparse.ArgumentParser(description="Aggregate seeds and build paper plots.")
    ap.add_argument(
        "--results-root",
        required=True,
        help="Root containing simulations_<seed>/ folders (e.g. /Users/.../paper_results)",
    )
    ap.add_argument(
        "--out-dir",
        default="",
        help=(
            "Optional output directory for compiled CSV/plots. "
            "If omitted, defaults to <results-root>/compiled_summary."
        ),
    )
    ap.add_argument(
        "--scenarios",
        default="0.5K,1K,1.5K",
        help="Comma-separated scenario labels to include (default: 0.5K,1K,1.5K)",
    )
    ap.add_argument(
        "--seed-list",
        default="",
        help=(
            "Optional comma-separated seed IDs to include only these seeds "
            "(e.g., 20260329,20260390,20260367,20260604,20260430)."
        ),
    )
    ap.add_argument(
        "--y-max",
        type=float,
        default=300.0,
        help="Fixed y-axis maximum for scenario plots (default: 300). Set <=0 to disable fixed scaling.",
    )
    ap.add_argument(
        "--legend-style",
        choices=["center", "left"],
        default="center",
        help="Legend placement style for scenario plots (default: center).",
    )
    args = ap.parse_args()

    root = Path(args.results_root).expanduser().resolve()
    out = Path(args.out_dir).expanduser().resolve() if str(args.out_dir).strip() else (root / "compiled_summary")
    out.mkdir(parents=True, exist_ok=True)

    use_scenarios = {x.strip() for x in str(args.scenarios).split(",") if x.strip()}
    if not use_scenarios:
        raise SystemExit("No scenarios selected.")

    seed_allow = {x.strip() for x in str(args.seed_list).split(",") if x.strip()}

    rows: list[dict] = []
    # Support both "simulations_<seed>" and typo variant "simulaitons_<seed>".
    seed_dirs = {p.resolve() for p in root.glob("simulations_*")}
    seed_dirs.update({p.resolve() for p in root.glob("simulaitons_*")})

    for seed_dir in sorted(seed_dirs):
        seed_name = seed_dir.name
        if seed_name.startswith("simulations_"):
            seed = seed_name.replace("simulations_", "")
        elif seed_name.startswith("simulaitons_"):
            seed = seed_name.replace("simulaitons_", "")
        else:
            seed = seed_name
        if seed_allow and seed not in seed_allow:
            continue
        for csv_path in sorted(seed_dir.glob("*/ev_matrix_results.csv")):
            for r in _read_rows(csv_path):
                dens = str(r.get("density_label", "")).strip()
                sc = SCENARIO_MAP.get(dens)
                if sc not in use_scenarios:
                    continue
                try:
                    tt = float(r["travel_time_s"]) if str(r.get("travel_time_s", "")).strip() else None
                except Exception:
                    tt = None
                try:
                    rc = int(float(r.get("return_code") or -1))
                    arrived = int(float(r.get("arrived") or 0))
                    http_pre = int(float(r.get("http_precheck_ok") or 0))
                    http_start = int(float(r.get("http_startup_ok") or 0))
                    foreign_fail = int(float(r.get("foreign_ev_drop_fail") or 0))
                except Exception:
                    continue
                ok = (
                    tt is not None
                    and rc == 0
                    and arrived == 1
                    and http_pre == 1
                    and http_start == 1
                    and foreign_fail == 0
                )
                if not ok:
                    continue
                rows.append(
                    {
                        "seed": seed,
                        "scenario": sc,
                        "route_id": int(float(r["route_id"])),
                        "mode": str(r["mode"]),
                        "ev_id": str(r.get("ev_id", "")),
                        "travel_time_s": float(tt),
                    }
                )

    if not rows:
        raise SystemExit("No valid rows found.")

    agg_route: dict[tuple[str, int, str], list[float]] = defaultdict(list)
    agg_mode: dict[tuple[str, str], list[float]] = defaultdict(list)
    for r in rows:
        agg_route[(r["scenario"], r["route_id"], r["mode"])].append(r["travel_time_s"])
        agg_mode[(r["scenario"], r["mode"])].append(r["travel_time_s"])

    route_means: list[dict] = []
    for (sc, route, mode), vals in sorted(agg_route.items(), key=lambda x: (x[0][0], x[0][1], x[0][2])):
        route_means.append(
            {
                "scenario": sc,
                "route_id": route,
                "mode": mode,
                "mean_travel_time_s": round(sum(vals) / len(vals), 6),
                "n_samples": len(vals),
            }
        )

    scenario_mode_means: list[dict] = []
    for (sc, mode), vals in sorted(agg_mode.items(), key=lambda x: (x[0][0], x[0][1])):
        scenario_mode_means.append(
            {
                "scenario": sc,
                "mode": mode,
                "mean_travel_time_s": round(sum(vals) / len(vals), 6),
                "n_samples": len(vals),
            }
        )

    idx = {(x["scenario"], x["mode"]): x["mean_travel_time_s"] for x in scenario_mode_means}
    scenario_gain: list[dict] = []
    for sc in sorted(use_scenarios):
        b0 = idx.get((sc, "B0"))
        b1 = idx.get((sc, "B1"))
        f2 = idx.get((sc, "F2"))
        if b0 is None or b1 is None or f2 is None:
            continue
        scenario_gain.append(
            {
                "scenario": sc,
                "B0_mean_s": b0,
                "B1_mean_s": b1,
                "F2_mean_s": f2,
                "F2_minus_B1_s": round(f2 - b1, 6),
                "F2_minus_B0_s": round(f2 - b0, 6),
                "B1_minus_B0_s": round(b1 - b0, 6),
                "F2_vs_B1_gain_pct": round(((b1 - f2) / b1) * 100.0 if b1 else math.nan, 6),
                "F2_vs_B0_gain_pct": round(((b0 - f2) / b0) * 100.0 if b0 else math.nan, 6),
            }
        )

    all_by_mode: dict[str, list[float]] = defaultdict(list)
    for r in rows:
        all_by_mode[r["mode"]].append(r["travel_time_s"])
    b0 = sum(all_by_mode["B0"]) / len(all_by_mode["B0"])
    b1 = sum(all_by_mode["B1"]) / len(all_by_mode["B1"])
    f2 = sum(all_by_mode["F2"]) / len(all_by_mode["F2"])
    global_row = {
        "B0_mean_s": round(b0, 6),
        "B1_mean_s": round(b1, 6),
        "F2_mean_s": round(f2, 6),
        "F2_minus_B1_s": round(f2 - b1, 6),
        "F2_minus_B0_s": round(f2 - b0, 6),
        "F2_vs_B1_gain_pct": round(((b1 - f2) / b1) * 100.0 if b1 else math.nan, 6),
        "F2_vs_B0_gain_pct": round(((b0 - f2) / b0) * 100.0 if b0 else math.nan, 6),
        "n_B0": len(all_by_mode["B0"]),
        "n_B1": len(all_by_mode["B1"]),
        "n_F2": len(all_by_mode["F2"]),
    }

    _write_csv(
        out / "compiled_route_means.csv",
        route_means,
        ["scenario", "route_id", "mode", "mean_travel_time_s", "n_samples"],
    )
    _write_csv(
        out / "compiled_scenario_mode_means.csv",
        scenario_mode_means,
        ["scenario", "mode", "mean_travel_time_s", "n_samples"],
    )
    _write_csv(
        out / "compiled_scenario_gains.csv",
        scenario_gain,
        [
            "scenario",
            "B0_mean_s",
            "B1_mean_s",
            "F2_mean_s",
            "F2_minus_B1_s",
            "F2_minus_B0_s",
            "B1_minus_B0_s",
            "F2_vs_B1_gain_pct",
            "F2_vs_B0_gain_pct",
        ],
    )
    _write_csv(
        out / "compiled_global_gain.csv",
        [global_row],
        [
            "B0_mean_s",
            "B1_mean_s",
            "F2_mean_s",
            "F2_minus_B1_s",
            "F2_minus_B0_s",
            "F2_vs_B1_gain_pct",
            "F2_vs_B0_gain_pct",
            "n_B0",
            "n_B1",
            "n_F2",
        ],
    )

    # Per-scenario wide table format:
    # rows = modes, columns = routes, values = mean travel time.
    scenario_route_mode = defaultdict(dict)
    for r in route_means:
        scenario = str(r["scenario"])
        route_id = int(r["route_id"])
        mode = str(r["mode"])
        scenario_route_mode[(scenario, mode)][route_id] = float(r["mean_travel_time_s"])

    for sc in sorted(use_scenarios, key=lambda x: (len(x), x)):
        rows_wide: list[dict] = []
        for mode in ["B0", "B1", "F2"]:
            r = {"mode": mode}
            for route_id in [1, 2, 3, 4, 5]:
                val = scenario_route_mode.get((sc, mode), {}).get(route_id)
                r[f"route_{route_id}"] = "" if val is None else round(val, 6)
            rows_wide.append(r)
        _write_csv(
            out / f"compiled_route_mode_table_{sc}.csv",
            rows_wide,
            ["mode", "route_1", "route_2", "route_3", "route_4", "route_5"],
        )

    palette = {"B0": "#7f7f7f", "B1": "#1f77b4", "F2": "#ff7f0e"}
    label_map = {"B0": "FTCM", "B1": "LIDP", "F2": "FCDP"}
    for sc in sorted(use_scenarios, key=lambda x: (len(x), x)):
        sub = [r for r in route_means if r["scenario"] == sc]
        if not sub:
            continue
        routes = sorted({int(r["route_id"]) for r in sub})
        modes = ["B0", "B1", "F2"]
        x = list(range(len(routes)))
        width = 0.24
        fig, ax = plt.subplots(figsize=(12, 9))
        offs = [-width, 0, width]
        for i, m in enumerate(modes):
            y = []
            for rt in routes:
                vals = [r["mean_travel_time_s"] for r in sub if int(r["route_id"]) == rt and r["mode"] == m]
                y.append(vals[0] if vals else math.nan)
            bars = ax.bar([xi + offs[i] for xi in x], y, width=width, color=palette[m], label=label_map[m])
            for b, v in zip(bars, y):
                if v == v:
                    ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.1f}", ha="center", va="bottom", fontsize=14)
        ax.set_title(
            f"Scenario: {sc}",
            fontsize=24,
        )
        ax.set_xlabel("EV Route Number", fontsize=21)
        ax.set_ylabel("Mean Travel Time (s)", fontsize=21)
        ax.set_xticks(x)
        ax.set_xticklabels([str(r) for r in routes], fontsize=20)
        ax.tick_params(axis="y", labelsize=20)
        if float(args.y_max) > 0:
            ax.set_ylim(0, float(args.y_max))
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        if args.legend_style == "center":
            ax.legend(
                fontsize=18,
                loc="upper center",
                bbox_to_anchor=(0.5, 1.0),
                ncol=3,
                frameon=True,
            )
        else:
            ax.legend(
                fontsize=18,
                loc="upper left",
                frameon=True,
            )
        fig.tight_layout()
        fig.savefig(out / f"plot_mean_travel_by_route_{sc}.png", dpi=180)
        plt.close(fig)

    print(f"done: {out}")


if __name__ == "__main__":
    main()
