#!/usr/bin/env python3
"""Run EV travel-time experiment matrix for B0/B1/F2 across routes and densities.

Pipeline:
1) Generate scenario route files (random background + random EV route) with reproducible seeds.
2) Build per-scenario SUMO cfg files pointing to each generated route file.
3) Execute real-world.py for each (density, route_id, mode).
4) Extract emergency EV trip duration from SUMO tripinfo output.
5) Export CSV tables and publication-ready grouped bar charts.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Scenario:
    density: int
    density_label: str
    route_id: int
    route_file: Path
    sumocfg_file: Path
    ev_seed: int
    bg_seed: int


def _parse_csv_ints(s: str) -> List[int]:
    out: List[int] = []
    for tok in str(s or "").split(","):
        t = tok.strip()
        if not t:
            continue
        out.append(int(t))
    return out


def _parse_csv_strs(s: str) -> List[str]:
    return [t.strip() for t in str(s or "").split(",") if t.strip()]


def _ensure_dir(p: Path) -> Path:
    p.mkdir(parents=True, exist_ok=True)
    return p


def _set_or_create_value(parent: ET.Element, tag: str, value: str) -> None:
    el = parent.find(tag)
    if el is None:
        el = ET.SubElement(parent, tag)
    el.set("value", str(value))


def _write_sumocfg_variant(
    *,
    base_sumocfg: Path,
    out_sumocfg: Path,
    net_file: Path,
    route_file: Path,
    sim_begin: Optional[float],
    sim_end: Optional[float],
) -> None:
    root = ET.parse(base_sumocfg).getroot()
    input_el = root.find("input")
    if input_el is None:
        input_el = ET.SubElement(root, "input")
    _set_or_create_value(input_el, "net-file", str(net_file))
    _set_or_create_value(input_el, "route-files", str(route_file))

    if sim_begin is not None or sim_end is not None:
        time_el = root.find("time")
        if time_el is None:
            time_el = ET.SubElement(root, "time")
        if sim_begin is not None:
            _set_or_create_value(time_el, "begin", f"{float(sim_begin):.3f}")
        if sim_end is not None:
            _set_or_create_value(time_el, "end", f"{float(sim_end):.3f}")

    out_sumocfg.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out_sumocfg, encoding="utf-8", xml_declaration=True)


def _network_lane_km(net_file: Path) -> float:
    root = ET.parse(net_file).getroot()
    lane_m = 0.0
    for edge in root.findall("edge"):
        eid = edge.get("id")
        if not eid or eid.startswith(":") or edge.get("function") == "internal":
            continue
        for lane in edge.findall("lane"):
            try:
                lane_m += float(lane.get("length") or 0.0)
            except Exception:
                continue
    return max(1e-6, lane_m / 1000.0)


def _default_density_labels(densities: Sequence[int]) -> List[str]:
    canonical = ["smooth", "moderate", "severe"]
    dens = list(densities)
    if len(dens) <= len(canonical):
        return canonical[: len(dens)]
    return [f"density_{i+1}" for i in range(len(dens))]


def _run_cmd(cmd: List[str], *, cwd: Path, log_file: Path, dry_run: bool) -> int:
    if dry_run:
        print("DRY_RUN:", " ".join(shlex.quote(x) for x in cmd))
        return 0
    log_file.parent.mkdir(parents=True, exist_ok=True)
    with log_file.open("w", encoding="utf-8") as f:
        f.write("# CMD\n")
        f.write(" ".join(shlex.quote(x) for x in cmd) + "\n\n")
        f.flush()
        proc = subprocess.run(cmd, cwd=str(cwd), stdout=f, stderr=subprocess.STDOUT, check=False)
    return int(proc.returncode)


def _extract_ev_tripinfo(tripinfo_xml: Path, ev_id: str) -> Dict[str, Optional[float]]:
    out: Dict[str, Optional[float]] = {
        "found": 0.0,
        "arrived": 0.0,
        "depart_s": None,
        "arrival_s": None,
        "travel_time_s": None,
    }
    if not tripinfo_xml.exists():
        return out
    try:
        root = ET.parse(tripinfo_xml).getroot()
    except Exception:
        return out

    for el in root.findall("tripinfo"):
        if str(el.get("id", "")) != str(ev_id):
            continue
        out["found"] = 1.0
        depart = el.get("depart")
        arrival = el.get("arrival")
        duration = el.get("duration")
        try:
            out["depart_s"] = float(depart) if depart is not None else None
        except Exception:
            out["depart_s"] = None
        try:
            out["arrival_s"] = float(arrival) if arrival is not None else None
        except Exception:
            out["arrival_s"] = None
        try:
            out["travel_time_s"] = float(duration) if duration is not None else None
        except Exception:
            out["travel_time_s"] = None
        # consider "arrived" true only with finite non-negative arrival and duration
        out["arrived"] = 1.0 if (out["arrival_s"] is not None and out["arrival_s"] >= 0.0 and out["travel_time_s"] is not None) else 0.0
        return out
    return out


def _write_csv(path: Path, rows: List[Dict[str, object]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fieldnames))
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _plot_matrix(
    *,
    results: List[Dict[str, object]],
    modes: Sequence[str],
    densities: Sequence[int],
    density_labels: Sequence[str],
    lane_km: float,
    out_png: Path,
    out_svg: Path,
) -> None:
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as e:
        print(f"[matrix][WARN] plotting skipped (missing dependency): {type(e).__name__}:{e}")
        return

    mode_colors = {
        "B0": "#7f8c8d",
        "B1": "#2e8b57",
        "F2": "#e67e22",
    }
    mode_list = [str(m) for m in modes]
    dens_list = [int(d) for d in densities]

    route_ids = sorted({int(r["route_id"]) for r in results})
    x = np.arange(len(route_ids))
    width = 0.24 if len(mode_list) <= 3 else max(0.12, 0.8 / max(1, len(mode_list)))

    fig, axes = plt.subplots(1, len(dens_list), figsize=(6.8 * len(dens_list), 5.8), squeeze=False)

    for j, density in enumerate(dens_list):
        ax = axes[0][j]
        dens_rows = [r for r in results if int(r["density"]) == density]
        for i, mode in enumerate(mode_list):
            vals: List[float] = []
            for rid in route_ids:
                cand = [
                    r for r in dens_rows
                    if int(r["route_id"]) == int(rid) and str(r["mode"]) == str(mode)
                ]
                if cand and cand[0].get("travel_time_s") not in ("", None):
                    vals.append(float(cand[0]["travel_time_s"]))
                else:
                    vals.append(float("nan"))
            ax.bar(
                x + (i - (len(mode_list) - 1) / 2.0) * width,
                vals,
                width,
                label=mode,
                color=mode_colors.get(mode, None),
                edgecolor="black",
                linewidth=0.4,
            )

        dens_idx = dens_list.index(density)
        dens_label = density_labels[dens_idx] if dens_idx < len(density_labels) else f"density_{dens_idx+1}"
        veh_per_lane_km = float(density) / float(lane_km)
        ax.set_title(
            f"{dens_label.title()} ({density} veh)\n{veh_per_lane_km:.2f} veh/lane-km",
            fontsize=11,
        )
        ax.set_xticks(x)
        ax.set_xticklabels([str(r) for r in route_ids])
        ax.set_xlabel("Route Number")
        if j == 0:
            ax.set_ylabel("EV Travel Time (s)")
        ax.grid(axis="y", alpha=0.3, linestyle="--")

    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=len(mode_list), frameon=False)
    fig.suptitle("Emergency Vehicle Travel Time Matrix (B0/B1/F2)", fontsize=14, y=1.02)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    fig.savefig(out_svg, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run B0/B1/F2 EV travel-time experiment matrix.")
    ap.add_argument("--sim-root", default=".", help="directory where scripts are executed")
    ap.add_argument("--python-bin", default=sys.executable or "python3")
    ap.add_argument("--generator-script", default="./generate_madrid_traffic.py")
    ap.add_argument("--real-world-script", default="./real-world.py")
    ap.add_argument("--base-sumocfg", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--out-dir", default="./tmp/ev_matrix")
    ap.add_argument("--densities", default="200,500,1000")
    ap.add_argument("--density-labels", default="", help="optional comma-separated labels matching --densities")
    ap.add_argument("--modes", default="B0,B1,F2")
    ap.add_argument("--num-routes", type=int, default=6)
    ap.add_argument("--route-seed-base", type=int, default=1000)
    ap.add_argument("--bg-seed-base", type=int, default=2000)
    ap.add_argument("--ev-id", default="emergency1")
    ap.add_argument("--sumo-bin", default="sumo")
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--vehicles", default="veh0001,veh0002,veh0003")
    ap.add_argument("--step-length", type=float, default=0.1)
    ap.add_argument("--sim-begin", type=float, default=None)
    ap.add_argument("--sim-end", type=float, default=None)
    ap.add_argument("--sumo-lateral-resolution", type=float, default=-1.0)
    ap.add_argument("--sumo-extra-base", default="", help="extra SUMO args appended in every run")
    ap.add_argument("--realworld-extra-args", default="", help="extra args appended to real-world command")
    ap.add_argument("--skip-existing-runs", action="store_true", default=False)
    ap.add_argument("--dry-run", action="store_true", default=False)
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    sim_root = Path(args.sim_root).resolve()
    generator_script = Path(args.generator_script).resolve()
    real_world_script = Path(args.real_world_script).resolve()
    base_sumocfg = Path(args.base_sumocfg).resolve()
    net_file = Path(args.net_file).resolve()
    out_dir = Path(args.out_dir).resolve()
    route_dir = _ensure_dir(out_dir / "scenarios" / "routes")
    sumocfg_dir = _ensure_dir(out_dir / "scenarios" / "sumocfg")
    runs_dir = _ensure_dir(out_dir / "runs")

    densities = _parse_csv_ints(args.densities)
    if not densities:
        raise SystemExit("No densities provided")
    modes = _parse_csv_strs(args.modes)
    if not modes:
        raise SystemExit("No modes provided")
    density_labels = _parse_csv_strs(args.density_labels)
    if not density_labels:
        density_labels = _default_density_labels(densities)
    if len(density_labels) != len(densities):
        raise SystemExit("--density-labels must match the number of --densities")

    lane_km = _network_lane_km(net_file)
    print(f"[matrix] network lane-km={lane_km:.3f}")

    scenarios: List[Scenario] = []
    for di, density in enumerate(densities):
        dens_label = density_labels[di]
        for route_id in range(1, int(args.num_routes) + 1):
            ev_seed = int(args.route_seed_base) + int(route_id)
            bg_seed = int(args.bg_seed_base) + int(density) * 1000 + int(route_id)

            route_file = route_dir / f"madrid_d{density}_r{route_id}.rou.xml"
            sumocfg_file = sumocfg_dir / f"madrid_d{density}_r{route_id}.sumocfg"

            gen_cmd = [
                str(args.python_bin),
                str(generator_script),
                "--net-file",
                str(net_file),
                "--out-routes",
                str(route_file),
                "--num-vehicles",
                str(int(density)),
                "--num-emergency-vehicles",
                "1",
                "--ev-id",
                str(args.ev_id),
                "--ev-seed",
                str(ev_seed),
                "--background-seed",
                str(bg_seed),
            ]
            gen_log = out_dir / "logs" / f"generate_d{density}_r{route_id}.log"
            rc_gen = _run_cmd(gen_cmd, cwd=sim_root, log_file=gen_log, dry_run=args.dry_run)
            if rc_gen != 0:
                raise SystemExit(f"generator failed rc={rc_gen} for density={density} route={route_id}")

            if not args.dry_run:
                _write_sumocfg_variant(
                    base_sumocfg=base_sumocfg,
                    out_sumocfg=sumocfg_file,
                    net_file=net_file,
                    route_file=route_file,
                    sim_begin=args.sim_begin,
                    sim_end=args.sim_end,
                )

            scenarios.append(
                Scenario(
                    density=int(density),
                    density_label=str(dens_label),
                    route_id=int(route_id),
                    route_file=route_file,
                    sumocfg_file=sumocfg_file,
                    ev_seed=ev_seed,
                    bg_seed=bg_seed,
                )
            )

    scenario_rows: List[Dict[str, object]] = []
    for sc in scenarios:
        scenario_rows.append(
            {
                "density": sc.density,
                "density_label": sc.density_label,
                "route_id": sc.route_id,
                "route_file": str(sc.route_file),
                "sumocfg_file": str(sc.sumocfg_file),
                "ev_seed": sc.ev_seed,
                "background_seed": sc.bg_seed,
                "veh_per_lane_km": float(sc.density) / float(lane_km),
            }
        )
    _write_csv(
        out_dir / "scenario_manifest.csv",
        scenario_rows,
        fieldnames=[
            "density",
            "density_label",
            "route_id",
            "route_file",
            "sumocfg_file",
            "ev_seed",
            "background_seed",
            "veh_per_lane_km",
        ],
    )

    results: List[Dict[str, object]] = []
    extra_rw_args = shlex.split(str(args.realworld_extra_args or "").strip()) if str(args.realworld_extra_args or "").strip() else []

    for sc in scenarios:
        for mode in modes:
            mode = str(mode).upper()
            run_dir = _ensure_dir(runs_dir / f"d{sc.density}" / f"r{sc.route_id}" / mode)
            tripinfo_xml = run_dir / "tripinfo.xml"
            rw_log = run_dir / "realworld.log"
            fed_log = run_dir / "fed_outcomes.txt"

            if args.skip_existing_runs and tripinfo_xml.exists():
                print(f"[matrix] skip existing run d={sc.density} r={sc.route_id} mode={mode}")
                trip = _extract_ev_tripinfo(tripinfo_xml, str(args.ev_id))
                results.append(
                    {
                        "density": sc.density,
                        "density_label": sc.density_label,
                        "route_id": sc.route_id,
                        "mode": mode,
                        "trip_found": int(trip["found"] or 0),
                        "arrived": int(trip["arrived"] or 0),
                        "depart_s": trip["depart_s"],
                        "arrival_s": trip["arrival_s"],
                        "travel_time_s": trip["travel_time_s"],
                        "tripinfo_xml": str(tripinfo_xml),
                        "realworld_log": str(rw_log),
                        "fed_log": str(fed_log),
                        "return_code": 0,
                    }
                )
                continue

            sumo_extra_parts = []
            if str(args.sumo_extra_base or "").strip():
                sumo_extra_parts.extend(shlex.split(str(args.sumo_extra_base)))
            sumo_extra_parts.extend(
                [
                    "--tripinfo-output",
                    str(tripinfo_xml),
                    "--tripinfo-output.write-unfinished",
                    "true",
                ]
            )
            sumo_extra = " ".join(shlex.quote(x) for x in sumo_extra_parts)

            cmd = [
                str(args.python_bin),
                str(real_world_script),
                "--sumo-bin",
                str(args.sumo_bin),
                "--sumo-cfg",
                str(sc.sumocfg_file),
                "--net-file",
                str(net_file),
                "--mqtt-host",
                str(args.mqtt_host),
                "--vehicles",
                str(args.vehicles),
                "--emergency-veh",
                str(args.ev_id),
                "--step-length",
                str(float(args.step_length)),
                "--evaluation",
                str(mode),
                "--disable-ers",
                "--agent-subset",
                "ev-route",
                "--agent-subset-neighbor-hops",
                "1",
                "--main-loop-sleep-sec",
                "0",
                "--no-shadow",
                "--ev-request-delivery",
                "direct",
                "--ev-request-source-tag",
                "matrix",
                "--fed-debug",
                "--fed-debug-log-file",
                str(fed_log),
                "--fed-debug-log-reset",
                "--sumo-extra-args",
                str(sumo_extra),
            ]
            if float(args.sumo_lateral_resolution) > 0.0:
                cmd.extend(["--sumo-lateral-resolution", str(float(args.sumo_lateral_resolution))])
            cmd.extend(extra_rw_args)

            print(f"[matrix] run density={sc.density} route={sc.route_id} mode={mode}")
            rc = _run_cmd(cmd, cwd=sim_root, log_file=rw_log, dry_run=args.dry_run)
            trip = _extract_ev_tripinfo(tripinfo_xml, str(args.ev_id))
            results.append(
                {
                    "density": sc.density,
                    "density_label": sc.density_label,
                    "route_id": sc.route_id,
                    "mode": mode,
                    "trip_found": int(trip["found"] or 0),
                    "arrived": int(trip["arrived"] or 0),
                    "depart_s": trip["depart_s"],
                    "arrival_s": trip["arrival_s"],
                    "travel_time_s": trip["travel_time_s"],
                    "tripinfo_xml": str(tripinfo_xml),
                    "realworld_log": str(rw_log),
                    "fed_log": str(fed_log),
                    "return_code": int(rc),
                }
            )

    _write_csv(
        out_dir / "ev_travel_time_results.csv",
        results,
        fieldnames=[
            "density",
            "density_label",
            "route_id",
            "mode",
            "trip_found",
            "arrived",
            "depart_s",
            "arrival_s",
            "travel_time_s",
            "tripinfo_xml",
            "realworld_log",
            "fed_log",
            "return_code",
        ],
    )

    summary_rows: List[Dict[str, object]] = []
    for density, dens_label in zip(densities, density_labels):
        for mode in modes:
            vals = [
                float(r["travel_time_s"])
                for r in results
                if int(r["density"]) == int(density)
                and str(r["mode"]) == str(mode)
                and r.get("travel_time_s") not in (None, "")
            ]
            summary_rows.append(
                {
                    "density": int(density),
                    "density_label": str(dens_label),
                    "mode": str(mode),
                    "n_routes": int(len(vals)),
                    "mean_travel_time_s": (float(statistics.mean(vals)) if vals else None),
                    "std_travel_time_s": (float(statistics.pstdev(vals)) if len(vals) > 1 else 0.0 if vals else None),
                    "min_travel_time_s": (float(min(vals)) if vals else None),
                    "max_travel_time_s": (float(max(vals)) if vals else None),
                    "veh_per_lane_km": float(int(density) / float(lane_km)),
                }
            )
    _write_csv(
        out_dir / "ev_travel_time_summary.csv",
        summary_rows,
        fieldnames=[
            "density",
            "density_label",
            "mode",
            "n_routes",
            "mean_travel_time_s",
            "std_travel_time_s",
            "min_travel_time_s",
            "max_travel_time_s",
            "veh_per_lane_km",
        ],
    )

    if not args.dry_run:
        _plot_matrix(
            results=results,
            modes=[str(m).upper() for m in modes],
            densities=densities,
            density_labels=density_labels,
            lane_km=lane_km,
            out_png=out_dir / "ev_travel_time_matrix.png",
            out_svg=out_dir / "ev_travel_time_matrix.svg",
        )

    meta = {
        "generated_at_epoch": time.time(),
        "sim_root": str(sim_root),
        "base_sumocfg": str(base_sumocfg),
        "net_file": str(net_file),
        "densities": [int(d) for d in densities],
        "density_labels": list(density_labels),
        "modes": [str(m).upper() for m in modes],
        "num_routes": int(args.num_routes),
        "lane_km": float(lane_km),
    }
    (out_dir / "run_meta.json").write_text(json.dumps(meta, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    print(f"[matrix] done. outputs at: {out_dir}")


if __name__ == "__main__":
    main()
