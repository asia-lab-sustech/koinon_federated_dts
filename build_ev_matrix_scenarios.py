#!/usr/bin/env python3
"""Materialize EV experiment scenarios from one master routes file.

Use one master .rou.xml containing:
- many background vehicles (e.g., 2000)
- emergency1..emergencyN (e.g., 6)

This script creates per-scenario route files:
- one EV route per file (emergency<route_id>)
- one background subset per density (200/500/1000), reproducible and optionally nested.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import random
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _parse_density_map(s: str) -> List[Tuple[str, int]]:
    out: List[Tuple[str, int]] = []
    for tok in str(s or "").split(","):
        t = tok.strip()
        if not t:
            continue
        if "=" not in t:
            raise ValueError(f"Invalid density token '{t}', expected label=count")
        k, v = t.split("=", 1)
        label = str(k).strip()
        count = int(str(v).strip())
        if not label:
            raise ValueError(f"Invalid empty density label in token '{t}'")
        if count <= 0:
            raise ValueError(f"Density count must be > 0 in token '{t}'")
        out.append((label, count))
    if not out:
        raise ValueError("No densities provided")
    return out


def _ev_route_index(ev_id: str, ev_prefix: str) -> Optional[int]:
    m = re.fullmatch(re.escape(ev_prefix) + r"(\d+)", str(ev_id))
    if not m:
        return None
    return int(m.group(1))


def _is_vehicle(el: ET.Element) -> bool:
    return str(el.tag) == "vehicle"


def _clone(el: ET.Element) -> ET.Element:
    return copy.deepcopy(el)


def _vehicle_route_edges(v: ET.Element) -> List[str]:
    route_child = v.find("route")
    if route_child is not None:
        edges = str(route_child.get("edges", "") or "").strip()
        if edges:
            return [e for e in edges.split() if e]
    edges_attr = str(v.get("route", "") or "").strip()
    if edges_attr:
        return [e for e in edges_attr.split() if e]
    return []


def _split_csv_list(s: object) -> List[str]:
    out: List[str] = []
    if s is None:
        return out
    if isinstance(s, (list, tuple)):
        for item in s:
            out.extend(_split_csv_list(item))
        return out
    for tok in str(s or "").replace(";", ",").split(","):
        t = tok.strip()
        if t:
            out.append(t)
    return out


def _parse_float_pair(s: object, default: Tuple[float, float], *, label: str) -> Tuple[float, float]:
    text = str(s or "").strip()
    if not text:
        return default
    try:
        a_raw, b_raw = [float(x.strip()) for x in text.split(",", 1)]
    except Exception as e:
        raise SystemExit(f"Invalid {label} '{text}', expected start,end floats") from e
    return (a_raw, b_raw) if b_raw >= a_raw else (b_raw, a_raw)


def _load_stress_profiles(path: str) -> List[Dict[str, Any]]:
    p = Path(str(path or "")).expanduser()
    if not str(path or "").strip():
        return []
    if not p.exists():
        raise SystemExit(f"--stress-config not found: {p}")
    text = p.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise SystemExit(
            f"Invalid --stress-config JSON: {p}. "
            "Use JSON for now so the generator does not depend on PyYAML."
        ) from e
    if isinstance(data, list):
        profiles = data
    elif isinstance(data, dict):
        profiles = data.get("profiles", [])
    else:
        raise SystemExit(f"Invalid --stress-config root in {p}; expected object with profiles[] or list")
    out: List[Dict[str, Any]] = []
    for i, prof in enumerate(profiles):
        if not isinstance(prof, dict):
            raise SystemExit(f"Invalid stress profile #{i + 1} in {p}; expected object")
        out.append(dict(prof))
    return out


def _match_stress_profile(
    profiles: Sequence[Dict[str, Any]],
    *,
    route_id: int,
    density_label: str,
    density_count: int,
) -> Optional[Dict[str, Any]]:
    for prof in profiles:
        route_ids = prof.get("route_ids", prof.get("routes", []))
        density_labels = prof.get("density_labels", prof.get("densities", []))
        density_counts = prof.get("density_counts", [])
        route_ok = True
        density_label_ok = True
        density_count_ok = True
        if route_ids not in (None, "", []):
            route_ok = int(route_id) in {int(x) for x in _split_csv_list(route_ids)}
        if density_labels not in (None, "", []):
            density_label_ok = str(density_label) in {str(x) for x in _split_csv_list(density_labels)}
        if density_counts not in (None, "", []):
            density_count_ok = int(density_count) in {int(x) for x in _split_csv_list(density_counts)}
        if route_ok and density_label_ok and density_count_ok:
            return prof
    return None


def _retime_corridor_stress_background(
    *,
    rng: random.Random,
    bg_vehicles: List[ET.Element],
    ev_vehicle: ET.Element,
    share: float,
    window_start_frac: float,
    window_end_frac: float,
    depart_start_offset_s: float,
    depart_end_offset_s: float,
    selection_basis: str,
    min_overlap_edges: int,
    depart_profile: str,
    platoon_period_sec: float,
    platoon_jitter_sec: float,
    target_edges_override: Optional[Sequence[str]] = None,
) -> Tuple[int, int]:
    if not bg_vehicles:
        return 0, 0
    ev_edges = _vehicle_route_edges(ev_vehicle)
    if len(ev_edges) < 3:
        return 0, 0

    override_edges = [str(e).strip() for e in (target_edges_override or []) if str(e).strip()]
    if override_edges:
        target_edges = set(override_edges)
    else:
        n_ev = len(ev_edges)
        w0 = max(0, min(n_ev - 1, int(round(window_start_frac * (n_ev - 1)))))
        w1 = max(w0, min(n_ev - 1, int(round(window_end_frac * (n_ev - 1)))))
        target_edges = set(ev_edges[w0 : w1 + 1])
    if not target_edges:
        return 0, 0

    ev_depart = float(ev_vehicle.get("depart", "0") or "0")
    t0 = ev_depart + float(depart_start_offset_s)
    t1 = ev_depart + float(depart_end_offset_s)
    if t1 < t0:
        t1 = t0

    candidates: List[ET.Element] = []
    for v in bg_vehicles:
        vedges = _vehicle_route_edges(v)
        if not vedges:
            continue
        overlap_n = sum(1 for e in vedges if e in target_edges)
        if overlap_n >= max(1, int(min_overlap_edges)):
            candidates.append(v)

    if not candidates:
        return 0, 0

    basis_n = len(candidates) if str(selection_basis).strip().lower() == "candidates" else len(bg_vehicles)
    n_target = int(round(max(0.0, min(1.0, float(share))) * float(basis_n)))
    if n_target <= 0:
        return len(candidates), 0
    n_pick = min(len(candidates), n_target)
    selected = rng.sample(candidates, n_pick) if n_pick < len(candidates) else list(candidates)
    profile = str(depart_profile or "uniform").strip().lower()
    period = max(0.1, float(platoon_period_sec or 0.1))
    jitter = max(0.0, float(platoon_jitter_sec or 0.0))
    window = max(0.0, float(t1 - t0))
    slot_count = max(1, int(window / period) + 1)
    # Keep retiming deterministic for a fixed seed even when XML input order changes.
    selected = sorted(selected, key=lambda x: str(x.get("id", "") or ""))
    for i, v in enumerate(selected):
        if profile == "platoon":
            base = t0 + float(i % slot_count) * period
            new_depart = base + (rng.uniform(-jitter, jitter) if jitter > 0.0 else 0.0)
        elif profile == "front_loaded":
            # Bias departures toward the start of the window to create queue pressure
            # ahead of the EV without changing the route topology.
            new_depart = t0 + (rng.betavariate(1.0, 3.0) * window if window > 0.0 else 0.0)
        else:
            new_depart = float(rng.uniform(t0, t1))
        new_depart = min(max(float(new_depart), float(t0)), float(t1))
        v.set("depart", f"{new_depart:.3f}")
    return len(candidates), len(selected)


def _write_routes(
    *,
    out_file: Path,
    preamble_children: Sequence[ET.Element],
    vehicles: Sequence[ET.Element],
) -> None:
    root = ET.Element("routes")
    for el in preamble_children:
        root.append(_clone(el))
    for v in vehicles:
        root.append(_clone(v))
    out_file.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out_file, encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Build EV matrix scenario route files from a master routes file.")
    ap.add_argument("--master-routes", required=True, help="master .rou.xml with background + emergency1..N")
    ap.add_argument("--out-dir", required=True, help="output directory for scenario route files and manifest")
    ap.add_argument(
        "--densities",
        default="smooth=200,moderate=500,severe=1000",
        help="comma-separated density map label=count",
    )
    ap.add_argument("--ev-prefix", default="emergency", help="EV id prefix (default: emergency)")
    ap.add_argument("--num-routes", type=int, default=6, help="number of EV routes to materialize")
    ap.add_argument("--seed", type=int, default=20260327, help="random seed for background subset")
    ap.add_argument(
        "--nested-subsets",
        action="store_true",
        default=True,
        help="if enabled, smaller densities are nested subsets of larger ones (default: enabled)",
    )
    ap.add_argument(
        "--no-nested-subsets",
        dest="nested_subsets",
        action="store_false",
        help="disable nested subsets (independent random sample per density)",
    )
    ap.add_argument("--manifest-csv", default="", help="optional manifest output path")
    ap.add_argument(
        "--stress-config",
        default="",
        help=(
            "optional JSON file with route/density-specific stress profiles. "
            "Profiles can override corridor stress knobs and target explicit downstream edges"
        ),
    )
    ap.add_argument(
        "--corridor-stress-preset",
        choices=["none", "spillback"],
        default="none",
        help=(
            "named corridor stress preset. 'spillback' concentrates overlapping "
            "background traffic on downstream EV-corridor edges near EV arrival; "
            "explicit corridor-stress knobs can still override the preset defaults"
        ),
    )
    ap.add_argument(
        "--corridor-stress-enable",
        action="store_true",
        help=(
            "retime a share of background vehicles that intersect downstream EV-route edges "
            "into a burst window near EV depart; keeps vehicle count fixed"
        ),
    )
    ap.add_argument(
        "--corridor-stress-share",
        type=float,
        default=0.35,
        help="share [0..1] of scenario background vehicles to retime when corridor stress is enabled",
    )
    ap.add_argument(
        "--corridor-stress-edge-window",
        default="0.35,0.95",
        help="fractional EV-edge window start,end for downstream target (default: 0.35,0.95)",
    )
    ap.add_argument(
        "--corridor-stress-target-edges",
        default="",
        help=(
            "optional comma-separated edge ids to stress instead of the fractional EV-edge window; "
            "useful for deliberately stressing known downstream spillback regions"
        ),
    )
    ap.add_argument(
        "--corridor-stress-depart-offset",
        default="10,180",
        help="EV-relative depart offset window seconds start,end for retimed background (default: 10,180)",
    )
    ap.add_argument(
        "--corridor-stress-seed",
        type=int,
        default=0,
        help="optional seed for corridor retiming; 0 derives from --seed",
    )
    ap.add_argument(
        "--corridor-stress-selection-basis",
        choices=["scenario", "candidates"],
        default="scenario",
        help=(
            "how --corridor-stress-share is interpreted: scenario=share of all background "
            "vehicles, candidates=share of only vehicles overlapping the EV corridor"
        ),
    )
    ap.add_argument(
        "--corridor-stress-min-overlap-edges",
        type=int,
        default=1,
        help="minimum number of downstream EV-corridor edges a background route must overlap to be retimed",
    )
    ap.add_argument(
        "--corridor-stress-depart-profile",
        choices=["uniform", "front_loaded", "platoon"],
        default="uniform",
        help=(
            "retiming profile for selected corridor vehicles: uniform spreads demand, "
            "front_loaded biases to the start, platoon creates tight repeated departures"
        ),
    )
    ap.add_argument(
        "--corridor-stress-platoon-period-sec",
        type=float,
        default=2.0,
        help="departure spacing used by --corridor-stress-depart-profile platoon",
    )
    ap.add_argument(
        "--corridor-stress-platoon-jitter-sec",
        type=float,
        default=0.25,
        help="bounded random jitter around platoon departures",
    )
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    if str(args.corridor_stress_preset or "none").strip().lower() == "spillback":
        args.corridor_stress_enable = True
        # Only replace parser defaults, so explicitly supplied values still win.
        if float(args.corridor_stress_share) == 0.35:
            args.corridor_stress_share = 0.90
        if str(args.corridor_stress_edge_window) == "0.35,0.95":
            args.corridor_stress_edge_window = "0.45,1.00"
        if str(args.corridor_stress_depart_offset) == "10,180":
            args.corridor_stress_depart_offset = "-60,60"
        if str(args.corridor_stress_selection_basis) == "scenario":
            args.corridor_stress_selection_basis = "candidates"
        if int(args.corridor_stress_min_overlap_edges) == 1:
            args.corridor_stress_min_overlap_edges = 3
        if str(args.corridor_stress_depart_profile) == "uniform":
            args.corridor_stress_depart_profile = "platoon"
        if float(args.corridor_stress_platoon_period_sec) == 2.0:
            args.corridor_stress_platoon_period_sec = 0.8
        if float(args.corridor_stress_platoon_jitter_sec) == 0.25:
            args.corridor_stress_platoon_jitter_sec = 0.15

    master_routes = Path(args.master_routes).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_routes_dir = out_dir / "routes"
    out_routes_dir.mkdir(parents=True, exist_ok=True)

    density_map = _parse_density_map(args.densities)
    max_density = max(c for _, c in density_map)

    root = ET.parse(master_routes).getroot()
    children = list(root)
    preamble = [c for c in children if not _is_vehicle(c)]
    vehicles = [c for c in children if _is_vehicle(c)]

    ev_by_idx: Dict[int, ET.Element] = {}
    background: List[ET.Element] = []
    for v in vehicles:
        vid = str(v.get("id", "") or "")
        idx = _ev_route_index(vid, str(args.ev_prefix))
        if idx is not None:
            ev_by_idx[idx] = v
        else:
            background.append(v)

    missing = [i for i in range(1, int(args.num_routes) + 1) if i not in ev_by_idx]
    if missing:
        raise SystemExit(
            f"Master routes missing EV ids for requested routes: "
            f"{', '.join([f'{args.ev_prefix}{i}' for i in missing])}"
        )
    if len(background) < int(max_density):
        raise SystemExit(
            f"Not enough background vehicles in master routes ({len(background)}) for max density {max_density}"
        )

    rng = random.Random(int(args.seed))
    bg_indices = list(range(len(background)))
    rng.shuffle(bg_indices)

    selected_bg_by_density: Dict[str, List[int]] = {}
    if bool(args.nested_subsets):
        for label, count in sorted(density_map, key=lambda x: x[1]):
            selected_bg_by_density[label] = list(bg_indices[: int(count)])
    else:
        for label, count in density_map:
            selected_bg_by_density[label] = sorted(rng.sample(bg_indices, int(count)))

    w0_raw, w1_raw = _parse_float_pair(
        args.corridor_stress_edge_window,
        (0.35, 0.95),
        label="--corridor-stress-edge-window",
    )
    w0 = max(0.0, min(1.0, w0_raw))
    w1 = max(0.0, min(1.0, w1_raw))
    if w1 < w0:
        w0, w1 = w1, w0

    d0, d1 = _parse_float_pair(
        args.corridor_stress_depart_offset,
        (10.0, 180.0),
        label="--corridor-stress-depart-offset",
    )

    stress_seed_base = int(args.corridor_stress_seed) if int(args.corridor_stress_seed) != 0 else int(args.seed) + 100003
    stress_profiles = _load_stress_profiles(str(args.stress_config or ""))
    cli_target_edges = _split_csv_list(args.corridor_stress_target_edges)

    manifest_rows: List[Dict[str, object]] = []
    for density_label, density_count in density_map:
        bg_sel = [background[i] for i in selected_bg_by_density[density_label]]
        for route_id in range(1, int(args.num_routes) + 1):
            ev_id = f"{args.ev_prefix}{route_id}"
            ev_vehicle = ev_by_idx[route_id]
            scenario_name = f"scenario_{density_label}_d{density_count}_r{route_id}"
            out_route = out_routes_dir / f"{scenario_name}.rou.xml"

            bg_for_scenario = [_clone(v) for v in bg_sel]
            ev_for_scenario = _clone(ev_vehicle)
            stress_candidates_n = 0
            stress_retimed_n = 0
            stress_profile = _match_stress_profile(
                stress_profiles,
                route_id=int(route_id),
                density_label=str(density_label),
                density_count=int(density_count),
            )
            stress_profile_id = ""
            stress_profile_source = "cli"
            stress_target_edges = list(cli_target_edges)
            stress_target_tls = ""
            stress_enable = bool(args.corridor_stress_enable)
            stress_share = float(args.corridor_stress_share)
            stress_selection_basis = str(args.corridor_stress_selection_basis)
            stress_min_overlap_edges = int(args.corridor_stress_min_overlap_edges)
            stress_depart_profile = str(args.corridor_stress_depart_profile)
            stress_platoon_period_sec = float(args.corridor_stress_platoon_period_sec)
            stress_platoon_jitter_sec = float(args.corridor_stress_platoon_jitter_sec)
            eff_w0, eff_w1 = float(w0), float(w1)
            eff_d0, eff_d1 = float(d0), float(d1)
            if stress_profile:
                stress_profile_source = "config"
                stress_profile_id = str(stress_profile.get("id", stress_profile.get("name", "")) or "")
                stress_enable = bool(stress_profile.get("enabled", True))
                if "share" in stress_profile:
                    stress_share = float(stress_profile["share"])
                if "selection_basis" in stress_profile:
                    stress_selection_basis = str(stress_profile["selection_basis"])
                if "min_overlap_edges" in stress_profile:
                    stress_min_overlap_edges = int(stress_profile["min_overlap_edges"])
                if "depart_profile" in stress_profile:
                    stress_depart_profile = str(stress_profile["depart_profile"])
                if "platoon_period_sec" in stress_profile:
                    stress_platoon_period_sec = float(stress_profile["platoon_period_sec"])
                if "platoon_jitter_sec" in stress_profile:
                    stress_platoon_jitter_sec = float(stress_profile["platoon_jitter_sec"])
                if "edge_window" in stress_profile:
                    ew0, ew1 = _parse_float_pair(
                        stress_profile["edge_window"],
                        (eff_w0, eff_w1),
                        label=f"edge_window in stress profile {stress_profile_id or route_id}",
                    )
                    eff_w0 = max(0.0, min(1.0, ew0))
                    eff_w1 = max(0.0, min(1.0, ew1))
                if "depart_offset" in stress_profile:
                    eff_d0, eff_d1 = _parse_float_pair(
                        stress_profile["depart_offset"],
                        (eff_d0, eff_d1),
                        label=f"depart_offset in stress profile {stress_profile_id or route_id}",
                    )
                if "target_edges" in stress_profile:
                    stress_target_edges = _split_csv_list(stress_profile["target_edges"])
                if "target_tls" in stress_profile:
                    stress_target_tls = ",".join(_split_csv_list(stress_profile["target_tls"]))

            if stress_enable:
                sid = (
                    (int(density_count) * 1009)
                    + (int(route_id) * 9173)
                    + sum(ord(c) for c in str(density_label))
                )
                rng_stress = random.Random(stress_seed_base + sid)
                stress_candidates_n, stress_retimed_n = _retime_corridor_stress_background(
                    rng=rng_stress,
                    bg_vehicles=bg_for_scenario,
                    ev_vehicle=ev_for_scenario,
                    share=float(stress_share),
                    window_start_frac=float(eff_w0),
                    window_end_frac=float(eff_w1),
                    depart_start_offset_s=float(eff_d0),
                    depart_end_offset_s=float(eff_d1),
                    selection_basis=str(stress_selection_basis),
                    min_overlap_edges=int(stress_min_overlap_edges),
                    depart_profile=str(stress_depart_profile),
                    platoon_period_sec=float(stress_platoon_period_sec),
                    platoon_jitter_sec=float(stress_platoon_jitter_sec),
                    target_edges_override=stress_target_edges,
                )

            scenario_vehicles = sorted(
                bg_for_scenario + [ev_for_scenario],
                key=lambda v: (
                    float(v.get("depart", "0") or "0"),
                    str(v.get("id", "") or ""),
                ),
            )
            _write_routes(out_file=out_route, preamble_children=preamble, vehicles=scenario_vehicles)
            manifest_rows.append(
                {
                    "scenario_id": scenario_name,
                    "density_label": density_label,
                    "density_count": int(density_count),
                    "route_id": int(route_id),
                    "ev_id": ev_id,
                    "background_count": int(len(bg_sel)),
                    "stress_profile_id": stress_profile_id,
                    "stress_profile_source": stress_profile_source,
                    "corridor_stress_enabled": 1 if bool(stress_enable) else 0,
                    "corridor_stress_candidates": int(stress_candidates_n),
                    "corridor_stress_retimed": int(stress_retimed_n),
                    "corridor_stress_selection_basis": str(stress_selection_basis),
                    "corridor_stress_min_overlap_edges": int(stress_min_overlap_edges),
                    "corridor_stress_depart_profile": str(stress_depart_profile),
                    "corridor_stress_edge_window": f"{float(eff_w0):.3f},{float(eff_w1):.3f}",
                    "corridor_stress_depart_offset": f"{float(eff_d0):.3f},{float(eff_d1):.3f}",
                    "corridor_stress_target_edges": ",".join(stress_target_edges),
                    "corridor_stress_target_tls": stress_target_tls,
                    "corridor_stress_share": f"{float(stress_share):.3f}",
                    "corridor_stress_platoon_period_sec": f"{float(stress_platoon_period_sec):.3f}",
                    "corridor_stress_platoon_jitter_sec": f"{float(stress_platoon_jitter_sec):.3f}",
                    "route_file": str(out_route),
                }
            )

    manifest_csv = Path(args.manifest_csv).resolve() if str(args.manifest_csv).strip() else (out_dir / "scenario_manifest.csv")
    manifest_csv.parent.mkdir(parents=True, exist_ok=True)
    with manifest_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "scenario_id",
                "density_label",
                "density_count",
                "route_id",
                "ev_id",
                "background_count",
                "stress_profile_id",
                "stress_profile_source",
                "corridor_stress_enabled",
                "corridor_stress_candidates",
                "corridor_stress_retimed",
                "corridor_stress_selection_basis",
                "corridor_stress_min_overlap_edges",
                "corridor_stress_depart_profile",
                "corridor_stress_edge_window",
                "corridor_stress_depart_offset",
                "corridor_stress_target_edges",
                "corridor_stress_target_tls",
                "corridor_stress_share",
                "corridor_stress_platoon_period_sec",
                "corridor_stress_platoon_jitter_sec",
                "route_file",
            ],
        )
        w.writeheader()
        for r in manifest_rows:
            w.writerow(r)

    print(f"[build_scenarios] master={master_routes}")
    print(f"[build_scenarios] out_routes={out_routes_dir}")
    print(f"[build_scenarios] scenarios={len(manifest_rows)}")
    print(f"[build_scenarios] manifest={manifest_csv}")


if __name__ == "__main__":
    main()
