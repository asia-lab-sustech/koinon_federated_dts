#!/usr/bin/env python3
"""
Build a routes file where emergency EV routes are constrained to TLS-controlled corridors.

Typical usage:
  python3 build_tls_controlled_ev_routes.py \
    --master-routes ./tmp/madrid_master_2000_6ev_burst.rou.xml \
    --net-file ./madrid_short_area.net.xml \
    --out-routes ./tmp/madrid_master_tlscorr_6ev.rou.xml \
    --num-routes 6 \
    --min-tls 4 \
    --max-tls 10
"""

from __future__ import annotations

import argparse
import copy
import random
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


@dataclass
class CandidateRoute:
    source_vehicle_id: str
    original_edges: List[str]
    selected_edges: List[str]
    tls_seq: List[str]
    node_seq: List[str]
    tls_coverage_ratio: float
    non_tls_junctions: int
    max_non_tls_gap: int
    max_non_tls_gap_distance_m: float
    matched_filters: bool
    source_vehicle_elem: ET.Element


def _parse_edges(text: str) -> List[str]:
    return [e for e in str(text or "").split() if e]


def _load_tls_and_edge_map(
    net_file: Path,
) -> Tuple[set[str], set[str], Dict[str, str], Dict[str, str], Dict[str, str], Dict[str, float]]:
    tree = ET.parse(str(net_file))
    root = tree.getroot()

    tls_nodes: set[str] = set()
    junction_nodes: set[str] = set()
    for j in root.findall("junction"):
        j_id = str(j.get("id", "") or "")
        j_type = str(j.get("type", "") or "")
        if j_id:
            junction_nodes.add(j_id)
        if j_id and "traffic_light" in j_type:
            tls_nodes.add(j_id)

    edge_to_to_node: Dict[str, str] = {}
    edge_to_from_node: Dict[str, str] = {}
    edge_to_to_raw: Dict[str, str] = {}
    edge_to_length_m: Dict[str, float] = {}
    for e in root.findall("edge"):
        edge_id = str(e.get("id", "") or "")
        from_node = str(e.get("from", "") or "")
        to_node = str(e.get("to", "") or "")
        function = str(e.get("function", "") or "")
        if not edge_id or not from_node or not to_node:
            continue
        if function == "internal":
            continue
        edge_to_to_node[edge_id] = to_node
        edge_to_from_node[edge_id] = from_node
        edge_to_to_raw[edge_id] = to_node
        lane_lengths: List[float] = []
        for lane in e.findall("lane"):
            try:
                lane_lengths.append(float(lane.get("length", "") or "0"))
            except ValueError:
                pass
        edge_to_length_m[edge_id] = max(lane_lengths) if lane_lengths else 0.0
    return tls_nodes, junction_nodes, edge_to_to_node, edge_to_from_node, edge_to_to_raw, edge_to_length_m


def _route_tls_seq(edges: Sequence[str], edge_to_to_node: Dict[str, str], tls_nodes: set[str]) -> Tuple[List[str], List[int]]:
    seq: List[str] = []
    idxs: List[int] = []
    seen: set[str] = set()
    for i, edge_id in enumerate(edges):
        n_to = edge_to_to_node.get(edge_id)
        if not n_to or n_to not in tls_nodes:
            continue
        if n_to in seen:
            continue
        seen.add(n_to)
        seq.append(n_to)
        idxs.append(i)
    return seq, idxs


def _route_node_seq(edges: Sequence[str], edge_to_to_node: Dict[str, str], junction_nodes: set[str]) -> List[str]:
    seq: List[str] = []
    seen: set[str] = set()
    for edge_id in edges:
        n_to = edge_to_to_node.get(edge_id)
        if not n_to or n_to not in junction_nodes:
            continue
        if n_to in seen:
            continue
        seen.add(n_to)
        seq.append(n_to)
    return seq


def _route_node_seq_ordered(edges: Sequence[str], edge_to_to_node: Dict[str, str], junction_nodes: set[str]) -> List[str]:
    seq: List[str] = []
    for edge_id in edges:
        n_to = edge_to_to_node.get(edge_id)
        if n_to and n_to in junction_nodes:
            seq.append(n_to)
    return seq


def _route_non_tls_gap_metrics(
    edges: Sequence[str],
    edge_to_to_node: Dict[str, str],
    edge_to_length_m: Dict[str, float],
    junction_nodes: set[str],
    tls_nodes: set[str],
) -> Tuple[int, float]:
    """Return the largest consecutive non-TLS gap between TLS-controlled nodes."""
    ordered_nodes = _route_node_seq_ordered(edges, edge_to_to_node, junction_nodes)
    if not ordered_nodes:
        return 0, 0.0
    tls_positions = [i for i, node_id in enumerate(ordered_nodes) if node_id in tls_nodes]
    if len(tls_positions) < 2:
        return 0, 0.0
    max_gap = 0
    max_gap_dist_m = 0.0
    for left, right in zip(tls_positions, tls_positions[1:]):
        gap = max(0, right - left - 1)
        # Edges whose to-nodes span (left, right] connect the two TLS positions.
        gap_dist_m = sum(float(edge_to_length_m.get(edge_id, 0.0) or 0.0) for edge_id in edges[left:right])
        if gap > max_gap or (gap == max_gap and gap_dist_m > max_gap_dist_m):
            max_gap = gap
            max_gap_dist_m = gap_dist_m
    return max_gap, max_gap_dist_m


def _is_edge_chain_connected(
    edges: Sequence[str],
    edge_to_from_node: Dict[str, str],
    edge_to_to_node_raw: Dict[str, str],
) -> bool:
    if len(edges) <= 1:
        return True
    for i in range(len(edges) - 1):
        e_curr = str(edges[i])
        e_next = str(edges[i + 1])
        curr_to = edge_to_to_node_raw.get(e_curr)
        next_from = edge_to_from_node.get(e_next)
        if not curr_to or not next_from:
            return False
        if curr_to != next_from:
            return False
    return True


def _load_route_defs(root: ET.Element) -> Dict[str, List[str]]:
    route_defs: Dict[str, List[str]] = {}
    for r in root.findall("route"):
        rid = str(r.get("id", "") or "")
        edges = _parse_edges(r.get("edges", ""))
        if rid and edges:
            route_defs[rid] = edges
    return route_defs


def _extract_vehicle_edges(v: ET.Element, route_defs: Dict[str, List[str]]) -> List[str]:
    child_route = v.find("route")
    if child_route is not None and child_route.get("edges"):
        return _parse_edges(child_route.get("edges", ""))
    route_ref = str(v.get("route", "") or "")
    if route_ref and route_ref in route_defs:
        return list(route_defs[route_ref])
    return []


def _build_candidates(
    routes_root: ET.Element,
    route_defs: Dict[str, List[str]],
    edge_to_to_node: Dict[str, str],
    edge_to_from_node: Dict[str, str],
    edge_to_to_node_raw: Dict[str, str],
    tls_nodes: set[str],
    junction_nodes: set[str],
    ev_prefix: str,
    min_tls: int,
    max_tls: int,
    min_tls_coverage_ratio: float,
    max_tls_coverage_ratio: float,
    min_non_tls_junctions: int,
    max_non_tls_junctions: int | None,
    max_non_tls_gap: int | None,
    max_non_tls_gap_distance_m: float | None,
    min_edges: int,
    validate_edge_adjacency: bool,
    truncate_at_max_tls: bool,
    include_nonmatching: bool,
    edge_to_length_m: Dict[str, float],
) -> List[CandidateRoute]:
    out: List[CandidateRoute] = []
    vehicles = routes_root.findall("vehicle")
    for v in vehicles:
        vid = str(v.get("id", "") or "")
        if not vid.startswith(ev_prefix):
            continue
        edges = _extract_vehicle_edges(v, route_defs)
        if not edges:
            continue
        selected_edges = list(edges)
        edge_chain_ok = _is_edge_chain_connected(
            selected_edges,
            edge_to_from_node=edge_to_from_node,
            edge_to_to_node_raw=edge_to_to_node_raw,
        )
        if validate_edge_adjacency and not edge_chain_ok:
            continue
        tls_seq, tls_idxs = _route_tls_seq(selected_edges, edge_to_to_node, tls_nodes)
        if truncate_at_max_tls and len(tls_seq) > max_tls:
            cut_idx = tls_idxs[max_tls - 1]
            # Keep one extra edge when possible so the EV passes beyond the last kept TLS.
            cut_idx = min(cut_idx + 1, len(selected_edges) - 1)
            selected_edges = selected_edges[: cut_idx + 1]
            edge_chain_ok = _is_edge_chain_connected(
                selected_edges,
                edge_to_from_node=edge_to_from_node,
                edge_to_to_node_raw=edge_to_to_node_raw,
            )
            if validate_edge_adjacency and not edge_chain_ok:
                continue
            tls_seq, _ = _route_tls_seq(selected_edges, edge_to_to_node, tls_nodes)
        node_seq = _route_node_seq(selected_edges, edge_to_to_node, junction_nodes)
        tls_cov = float(len(tls_seq)) / float(max(1, len(node_seq)))
        non_tls_junctions = max(0, len(node_seq) - len(tls_seq))
        max_gap, max_gap_dist_m = _route_non_tls_gap_metrics(
            selected_edges,
            edge_to_to_node=edge_to_to_node,
            edge_to_length_m=edge_to_length_m,
            junction_nodes=junction_nodes,
            tls_nodes=tls_nodes,
        )
        ok = (
            (len(selected_edges) >= min_edges)
            and (min_tls <= len(tls_seq) <= max_tls)
            and (tls_cov >= float(min_tls_coverage_ratio))
            and (tls_cov <= float(max_tls_coverage_ratio))
            and (non_tls_junctions >= int(min_non_tls_junctions))
            and (max_non_tls_junctions is None or non_tls_junctions <= int(max_non_tls_junctions))
            and (max_non_tls_gap is None or max_gap <= int(max_non_tls_gap))
            and (max_non_tls_gap_distance_m is None or max_gap_dist_m <= float(max_non_tls_gap_distance_m))
        )
        if ok or include_nonmatching:
            out.append(
                CandidateRoute(
                    source_vehicle_id=vid,
                    original_edges=list(edges),
                    selected_edges=selected_edges,
                    tls_seq=list(tls_seq),
                    node_seq=list(node_seq),
                    tls_coverage_ratio=float(tls_cov),
                    non_tls_junctions=int(non_tls_junctions),
                    max_non_tls_gap=int(max_gap),
                    max_non_tls_gap_distance_m=float(max_gap_dist_m),
                    matched_filters=bool(ok),
                    source_vehicle_elem=v,
                )
            )
    return out


def _strip_existing_emergency_vehicles(routes_root: ET.Element, ev_prefix: str) -> None:
    for child in list(routes_root):
        if child.tag != "vehicle":
            continue
        vid = str(child.get("id", "") or "")
        if vid.startswith(ev_prefix):
            routes_root.remove(child)


def _new_ev_vehicle_from_template(
    src: ET.Element,
    new_id: str,
    edges: Sequence[str],
    depart: float | None,
) -> ET.Element:
    v = copy.deepcopy(src)
    v.set("id", str(new_id))
    if depart is not None:
        v.set("depart", f"{depart:.1f}")
    if "route" in v.attrib:
        del v.attrib["route"]
    # Replace inline route; keep other children (stops, params, etc.).
    for ch in list(v):
        if ch.tag == "route":
            v.remove(ch)
    route_el = ET.Element("route")
    route_el.set("edges", " ".join(edges))
    v.insert(0, route_el)
    return v


def _rank_tls_rich(c: CandidateRoute) -> Tuple[object, ...]:
    return (
        0 if bool(c.matched_filters) else 1,
        int(c.non_tls_junctions),
        int(c.max_non_tls_gap),
        float(c.max_non_tls_gap_distance_m),
        -float(c.tls_coverage_ratio),
        -len(c.tls_seq),
        str(c.source_vehicle_id),
    )


def _rank_coverage_mid(c: CandidateRoute, target_cov: float) -> Tuple[object, ...]:
    return (
        0 if bool(c.matched_filters) else 1,
        abs(float(c.tls_coverage_ratio) - float(target_cov)),
        -len(c.tls_seq),
        -max(0, len(c.node_seq) - len(c.tls_seq)),
        str(c.source_vehicle_id),
    )


def _balanced_mix_bucket(c: CandidateRoute) -> int:
    """Assign routes to coarse active/passive evaluation strata."""
    cov = float(c.tls_coverage_ratio)
    non_tls = int(c.non_tls_junctions)
    if non_tls <= 1 or cov >= 0.85:
        return 0  # Mostly active TLS; F2 should be strong.
    if cov >= 0.65 and non_tls <= 5:
        return 1  # Mixed but still mostly controllable.
    return 2  # Passive-context useful; F2P should have something to add.


def _rank_balanced_mix(cand: Sequence[CandidateRoute], target_cov: float) -> List[CandidateRoute]:
    bins: Dict[int, List[CandidateRoute]] = {0: [], 1: [], 2: []}
    for c in cand:
        bins[_balanced_mix_bucket(c)].append(c)
    for bucket in bins.values():
        bucket.sort(
            key=lambda c: (
                0 if bool(c.matched_filters) else 1,
                int(c.max_non_tls_gap),
                float(c.max_non_tls_gap_distance_m),
                _balanced_mix_bucket(c),
                _rank_coverage_mid(c, target_cov),
            )
        )

    ranked: List[CandidateRoute] = []
    seen_sources: set[str] = set()
    while any(bins.values()):
        for bucket_id in (0, 1, 2):
            bucket = bins[bucket_id]
            if not bucket:
                continue
            c = bucket.pop(0)
            if c.source_vehicle_id in seen_sources:
                continue
            seen_sources.add(c.source_vehicle_id)
            ranked.append(c)
    return ranked


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Build EV routes constrained to TLS-controlled corridors.")
    ap.add_argument("--master-routes", required=True, help="source .rou.xml with background + emergency vehicles")
    ap.add_argument("--net-file", required=True, help="SUMO .net.xml")
    ap.add_argument("--out-routes", required=True, help="output .rou.xml")
    ap.add_argument("--ev-prefix", default="emergency", help="source emergency vehicle id prefix")
    ap.add_argument("--num-routes", type=int, default=6, help="how many emergency routes to keep in output")
    ap.add_argument("--min-tls", type=int, default=4, help="minimum unique TLS intersections along kept route")
    ap.add_argument("--max-tls", type=int, default=10, help="maximum unique TLS intersections along kept route")
    ap.add_argument(
        "--min-tls-coverage-ratio",
        type=float,
        default=0.0,
        help="minimum fraction of traversed junctions that must be TLS-controlled (0..1)",
    )
    ap.add_argument(
        "--max-tls-coverage-ratio",
        type=float,
        default=1.0,
        help=(
            "maximum fraction of traversed junctions that may be TLS-controlled (0..1); "
            "use <1.0 to force mixed TLS/non-TLS EV routes"
        ),
    )
    ap.add_argument(
        "--min-non-tls-junctions",
        type=int,
        default=0,
        help="minimum number of non-TLS junctions along the selected EV route",
    )
    ap.add_argument(
        "--max-non-tls-junctions",
        type=int,
        default=None,
        help=(
            "maximum number of non-TLS junctions along the selected EV route; "
            "set 0 to keep only fully TLS-covered traversed junctions"
        ),
    )
    ap.add_argument(
        "--max-non-tls-gap",
        type=int,
        default=None,
        help=(
            "maximum consecutive non-TLS junctions allowed between two TLS-controlled junctions; "
            "set 0 to require adjacent TLS-to-TLS control along the selected route"
        ),
    )
    ap.add_argument(
        "--max-non-tls-gap-distance-m",
        type=float,
        default=None,
        help="maximum edge-distance span, in meters, allowed between adjacent TLS peers through a non-TLS gap",
    )
    ap.add_argument(
        "--prefer-tls-rich",
        action="store_true",
        default=False,
        help=(
            "rank matched candidates by fewer non-TLS junctions/gaps and higher TLS coverage. "
            "Use this for F2 peer-to-peer evaluation corridors."
        ),
    )
    ap.add_argument(
        "--selection-profile",
        choices=("auto", "tls-rich", "coverage-mid", "balanced-mix"),
        default="auto",
        help=(
            "route ranking strategy. auto preserves legacy behavior; tls-rich favors active TLS corridors; "
            "coverage-mid targets the middle of the requested TLS coverage range; balanced-mix round-robins "
            "TLS-rich, mixed, and passive-context-heavy routes for B0/B1/F2/F2P comparisons"
        ),
    )
    ap.add_argument("--min-edges", type=int, default=8, help="minimum edge length for a kept route")
    ap.add_argument(
        "--validate-edge-adjacency",
        dest="validate_edge_adjacency",
        action="store_true",
        default=True,
        help="enforce consecutive edge connectivity (edge[i].to == edge[i+1].from) for selected EV routes",
    )
    ap.add_argument(
        "--no-validate-edge-adjacency",
        dest="validate_edge_adjacency",
        action="store_false",
        help="disable explicit edge-chain adjacency validation",
    )
    ap.add_argument(
        "--truncate-at-max-tls",
        dest="truncate_at_max_tls",
        action="store_true",
        default=True,
        help="truncate EV route once max TLS intersections is reached (default: enabled)",
    )
    ap.add_argument(
        "--no-truncate-at-max-tls",
        dest="truncate_at_max_tls",
        action="store_false",
        help="do not truncate long routes; only keep those already <= max TLS",
    )
    ap.add_argument(
        "--include-nonmatching-fallback",
        action="store_true",
        default=False,
        help=(
            "if strict filters produce too few candidates, allow best-effort selection from nonmatching EV routes. "
            "Useful for mixed TLS/non-TLS stress corridors where a soft route ranking is preferable to failing."
        ),
    )
    ap.add_argument("--seed", type=int, default=20260330)
    ap.add_argument(
        "--enforce-unique-routes",
        dest="enforce_unique_routes",
        action="store_true",
        default=True,
        help="ensure selected emergency routes have unique edge signatures (default: enabled)",
    )
    ap.add_argument(
        "--no-enforce-unique-routes",
        dest="enforce_unique_routes",
        action="store_false",
        help="allow duplicate route geometries in selected emergency routes",
    )
    ap.add_argument(
        "--reset-ev-depart-times",
        action="store_true",
        default=True,
        help="set selected EV depart times to a regular schedule (default: enabled)",
    )
    ap.add_argument(
        "--no-reset-ev-depart-times",
        dest="reset_ev_depart_times",
        action="store_false",
        help="keep selected source EV depart times",
    )
    ap.add_argument("--ev-depart-start", type=float, default=30.0)
    ap.add_argument("--ev-depart-interval", type=float, default=120.0)
    ap.add_argument(
        "--route-audit-csv",
        default=None,
        help="optional CSV path with selected route TLS/non-TLS audit metrics",
    )
    return ap.parse_args()


def main() -> int:
    args = _parse_args()
    master_routes = Path(args.master_routes).expanduser().resolve()
    out_routes = Path(args.out_routes).expanduser().resolve()
    net_file = Path(args.net_file).expanduser().resolve()

    if args.min_tls > args.max_tls:
        raise SystemExit("--min-tls must be <= --max-tls")
    if args.num_routes < 1:
        raise SystemExit("--num-routes must be >= 1")
    if args.max_non_tls_junctions is not None and int(args.min_non_tls_junctions) > int(args.max_non_tls_junctions):
        raise SystemExit("--min-non-tls-junctions must be <= --max-non-tls-junctions")

    (
        tls_nodes,
        junction_nodes,
        edge_to_to_node,
        edge_to_from_node,
        edge_to_to_raw,
        edge_to_length_m,
    ) = _load_tls_and_edge_map(net_file)
    rt_tree = ET.parse(str(master_routes))
    rt_root = rt_tree.getroot()
    route_defs = _load_route_defs(rt_root)

    cand = _build_candidates(
        routes_root=rt_root,
        route_defs=route_defs,
        edge_to_to_node=edge_to_to_node,
        edge_to_from_node=edge_to_from_node,
        edge_to_to_node_raw=edge_to_to_raw,
        tls_nodes=tls_nodes,
        junction_nodes=junction_nodes,
        ev_prefix=str(args.ev_prefix),
        min_tls=int(args.min_tls),
        max_tls=int(args.max_tls),
        min_tls_coverage_ratio=float(args.min_tls_coverage_ratio),
        max_tls_coverage_ratio=float(args.max_tls_coverage_ratio),
        min_non_tls_junctions=int(args.min_non_tls_junctions),
        max_non_tls_junctions=args.max_non_tls_junctions,
        max_non_tls_gap=args.max_non_tls_gap,
        max_non_tls_gap_distance_m=args.max_non_tls_gap_distance_m,
        min_edges=int(args.min_edges),
        validate_edge_adjacency=bool(args.validate_edge_adjacency),
        truncate_at_max_tls=bool(args.truncate_at_max_tls),
        include_nonmatching=bool(args.include_nonmatching_fallback),
        edge_to_length_m=edge_to_length_m,
    )

    if not cand:
        raise SystemExit("No emergency candidates found after TLS-controlled filtering.")

    rng = random.Random(int(args.seed))
    rng.shuffle(cand)

    if bool(args.enforce_unique_routes):
        selected: List[CandidateRoute] = []
        seen_sig: set[Tuple[str, ...]] = set()
        target_cov = min(
            1.0,
            max(0.0, (float(args.min_tls_coverage_ratio) + float(args.max_tls_coverage_ratio)) / 2.0),
        )
        selection_profile = str(getattr(args, "selection_profile", "auto") or "auto")
        if selection_profile == "auto":
            selection_profile = "tls-rich" if bool(args.prefer_tls_rich) else "coverage-mid"
        if selection_profile == "tls-rich":
            ranked_cand = sorted(cand, key=_rank_tls_rich)
        elif selection_profile == "balanced-mix":
            ranked_cand = _rank_balanced_mix(cand, target_cov)
        else:
            ranked_cand = sorted(cand, key=lambda c: _rank_coverage_mid(c, target_cov))
        for c in ranked_cand:
            sig = tuple(c.selected_edges)
            if sig in seen_sig:
                continue
            seen_sig.add(sig)
            selected.append(c)
            if len(selected) >= int(args.num_routes):
                break
    else:
        selected = cand[: int(args.num_routes)]
    if len(selected) < int(args.num_routes):
        raise SystemExit(
            f"Only {len(selected)} candidate routes available but --num-routes={int(args.num_routes)} requested."
        )

    _strip_existing_emergency_vehicles(rt_root, str(args.ev_prefix))

    selected = sorted(selected, key=lambda c: c.source_vehicle_id)
    for i, c in enumerate(selected, start=1):
        new_id = f"{args.ev_prefix}{i}"
        dep = None
        if bool(args.reset_ev_depart_times):
            dep = float(args.ev_depart_start) + float(i - 1) * float(args.ev_depart_interval)
        rt_root.append(
            _new_ev_vehicle_from_template(
                src=c.source_vehicle_elem,
                new_id=new_id,
                edges=c.selected_edges,
                depart=dep,
            )
        )

    out_routes.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(rt_tree, space="  ")
    rt_tree.write(str(out_routes), encoding="utf-8", xml_declaration=True)

    print("[tls_route_builder] done")
    print(f"[tls_route_builder] out_routes={out_routes}")
    max_gap_dist_label = (
        f"{float(args.max_non_tls_gap_distance_m):.1f}"
        if args.max_non_tls_gap_distance_m is not None
        else "ANY"
    )
    print(
        f"[tls_route_builder] selected_routes={len(selected)} "
        f"min_tls={args.min_tls} max_tls={args.max_tls} min_edges={args.min_edges} "
        f"min_tls_coverage_ratio={float(args.min_tls_coverage_ratio):.2f} "
        f"max_tls_coverage_ratio={float(args.max_tls_coverage_ratio):.2f} "
        f"min_non_tls_junctions={int(args.min_non_tls_junctions)} "
        f"max_non_tls_junctions={args.max_non_tls_junctions if args.max_non_tls_junctions is not None else 'ANY'} "
        f"max_non_tls_gap={args.max_non_tls_gap if args.max_non_tls_gap is not None else 'ANY'} "
        f"max_non_tls_gap_distance_m={max_gap_dist_label} "
        f"validate_edge_adjacency={int(bool(args.validate_edge_adjacency))} "
        f"enforce_unique_routes={int(bool(args.enforce_unique_routes))} "
        f"truncate_at_max_tls={int(bool(args.truncate_at_max_tls))} "
        f"prefer_tls_rich={int(bool(args.prefer_tls_rich))} "
        f"selection_profile={str(getattr(args, 'selection_profile', 'auto'))}"
    )
    for i, c in enumerate(selected, start=1):
        print(
            f"[tls_route_builder] route={i} src={c.source_vehicle_id} "
            f"edges={len(c.selected_edges)} tls={len(c.tls_seq)} nodes={len(c.node_seq)} "
            f"non_tls={int(c.non_tls_junctions)} "
            f"max_non_tls_gap={int(c.max_non_tls_gap)} "
            f"max_non_tls_gap_distance_m={float(c.max_non_tls_gap_distance_m):.1f} "
            f"tls_cov={c.tls_coverage_ratio:.2f} matched={int(bool(c.matched_filters))} "
            f"tls_seq={','.join(c.tls_seq[:12])}"
        )
    if args.route_audit_csv:
        import csv

        audit_path = Path(str(args.route_audit_csv)).expanduser().resolve()
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        with audit_path.open("w", newline="") as fh:
            writer = csv.DictWriter(
                fh,
                fieldnames=[
                    "route_index",
                    "new_ev_id",
                    "source_vehicle_id",
                    "edges_n",
                    "nodes_n",
                    "tls_n",
                    "non_tls_n",
                    "max_non_tls_gap",
                    "max_non_tls_gap_distance_m",
                    "tls_coverage_ratio",
                    "selection_bucket",
                    "matched_filters",
                    "tls_seq",
                    "edge_signature",
                ],
            )
            writer.writeheader()
            for i, c in enumerate(selected, start=1):
                writer.writerow(
                    {
                        "route_index": i,
                        "new_ev_id": f"{args.ev_prefix}{i}",
                        "source_vehicle_id": c.source_vehicle_id,
                        "edges_n": len(c.selected_edges),
                        "nodes_n": len(c.node_seq),
                        "tls_n": len(c.tls_seq),
                        "non_tls_n": int(c.non_tls_junctions),
                        "max_non_tls_gap": int(c.max_non_tls_gap),
                        "max_non_tls_gap_distance_m": f"{float(c.max_non_tls_gap_distance_m):.3f}",
                        "tls_coverage_ratio": f"{float(c.tls_coverage_ratio):.6f}",
                        "selection_bucket": int(_balanced_mix_bucket(c)),
                        "matched_filters": int(bool(c.matched_filters)),
                        "tls_seq": " ".join(c.tls_seq),
                        "edge_signature": " ".join(c.selected_edges),
                    }
                )
        print(f"[tls_route_builder] route_audit_csv={audit_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
