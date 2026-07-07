#!/usr/bin/env python3
"""Generate a SUMO route file for madrid_simplified.net.xml.

Creates background traffic plus one or more emergency vehicles with random end-to-end
routes (entry-to-exit style across the Madrid network).
"""

from __future__ import annotations

import argparse
import random
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class EdgeMeta:
    edge_id: str
    from_node: str
    to_node: str
    lane_count: int
    max_speed: float
    length: float


def _lane_allows_vclass(lane_el: ET.Element, vclass: str) -> bool:
    """Conservative lane permission check for a given SUMO vClass."""
    vclass = str(vclass or "").strip()
    allow = {x for x in (lane_el.get("allow") or "").split() if x}
    disallow = {x for x in (lane_el.get("disallow") or "").split() if x}
    if not vclass:
        return False
    if "all" in disallow or vclass in disallow:
        return False
    if allow:
        return ("all" in allow) or (vclass in allow)
    return True


def load_network_graph(
    net_file: Path,
    vehicle_class: str = "passenger",
) -> Tuple[Dict[str, EdgeMeta], Dict[str, List[str]], Dict[str, int], Dict[str, int]]:
    root = ET.parse(net_file).getroot()

    edges: Dict[str, EdgeMeta] = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":") or e.get("function") == "internal":
            continue
        lanes = list(e.findall("lane"))
        if not lanes:
            continue
        if not any(_lane_allows_vclass(ln, vehicle_class) for ln in lanes):
            continue

        max_speed = max(float((ln.get("speed") or "13.9")) for ln in lanes)
        max_length = max(float((ln.get("length") or "1.0")) for ln in lanes)
        edges[eid] = EdgeMeta(
            edge_id=eid,
            from_node=str(e.get("from") or ""),
            to_node=str(e.get("to") or ""),
            lane_count=len(lanes),
            max_speed=max_speed,
            length=max_length,
        )

    adj_set: Dict[str, set[str]] = defaultdict(set)
    indegree: Dict[str, int] = defaultdict(int)
    outdegree: Dict[str, int] = defaultdict(int)
    for c in root.findall("connection"):
        src = c.get("from")
        dst = c.get("to")
        if not src or not dst:
            continue
        if src.startswith(":") or dst.startswith(":"):
            continue
        if src not in edges or dst not in edges or src == dst:
            continue
        if dst not in adj_set[src]:
            adj_set[src].add(dst)
            outdegree[src] += 1
            indegree[dst] += 1

    for eid in edges:
        indegree.setdefault(eid, 0)
        outdegree.setdefault(eid, 0)
        adj_set.setdefault(eid, set())

    adj = {eid: sorted(list(nbs)) for eid, nbs in adj_set.items()}
    return edges, adj, dict(indegree), dict(outdegree)


def bfs_shortest_path(adj: Dict[str, List[str]], src: str, dst: str, max_visits: int = 30000) -> Optional[List[str]]:
    if src == dst:
        return [src]
    q: deque[str] = deque([src])
    prev: Dict[str, Optional[str]] = {src: None}
    visits = 0
    while q and dst not in prev:
        u = q.popleft()
        visits += 1
        if visits > max_visits:
            break
        for v in adj.get(u, []):
            if v not in prev:
                prev[v] = u
                q.append(v)
    if dst not in prev:
        return None
    path: List[str] = []
    cur: Optional[str] = dst
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


def sample_route(
    rng: random.Random,
    edge_ids: Sequence[str],
    adj: Dict[str, List[str]],
    *,
    min_edges: int,
    max_tries: int,
    src_pool: Optional[Sequence[str]] = None,
    dst_pool: Optional[Sequence[str]] = None,
) -> List[str]:
    src_candidates = list(src_pool or edge_ids)
    dst_candidates = list(dst_pool or edge_ids)
    if not src_candidates or not dst_candidates:
        raise RuntimeError("No candidate edges available for route sampling")

    for _ in range(max_tries):
        src = rng.choice(src_candidates)
        dst = rng.choice(dst_candidates)
        if src == dst:
            continue
        path = bfs_shortest_path(adj, src, dst)
        if not path:
            continue
        if len(path) < min_edges:
            continue
        return path
    raise RuntimeError(
        f"Failed to sample a route after {max_tries} attempts (min_edges={min_edges})."
    )


def write_routes(
    out_file: Path,
    routes: List[Tuple[str, str, float, List[str]]],
    emergency_routes: List[Tuple[str, str, float, List[str]]],
) -> None:
    lines: List[str] = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append('<routes>')
    lines.append('    <vType id="car" vClass="passenger" accel="2.6" decel="4.5" sigma="0.5" length="5.0" minGap="2.5" maxSpeed="13.9" guiShape="passenger"/>')
    lines.append('    <vType id="ev" vClass="emergency" accel="3.5" decel="6.0" sigma="0.2" length="5.0" minGap="1.0" maxSpeed="18.0" guiShape="emergency" color="1,0,0"/>')
    lines.append('')

    for veh_id, vtype, depart, route_edges in sorted(routes, key=lambda x: (x[2], x[0])):
        edges_str = " ".join(route_edges)
        lines.append(
            f'    <vehicle id="{veh_id}" type="{vtype}" depart="{depart:.1f}" departLane="best" departSpeed="max">'
        )
        lines.append(f'        <route edges="{edges_str}"/>')
        lines.append('    </vehicle>')

    for ev_id, ev_type, ev_depart, ev_edges in sorted(emergency_routes, key=lambda x: (x[2], x[0])):
        ev_edges_str = " ".join(ev_edges)
        lines.append('')
        lines.append(
            f'    <vehicle id="{ev_id}" type="{ev_type}" depart="{ev_depart:.1f}" departLane="best" departSpeed="max">'
        )
        lines.append(f'        <route edges="{ev_edges_str}"/>')
        lines.append('    </vehicle>')

    lines.append('</routes>')
    out_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate traffic and one or more emergency vehicle routes for the Madrid SUMO net")
    p.add_argument("--net-file", default="madrid_short_area.net.xml")
    p.add_argument("--out-routes", default="madrid_traffic.rou.xml")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument(
        "--background-seed",
        type=int,
        default=None,
        help="optional seed for background-traffic route sampling (defaults to --seed)",
    )
    p.add_argument(
        "--ev-seed",
        type=int,
        default=None,
        help=(
            "optional seed for emergency-route sampling; use this to keep EV routes stable "
            "while varying background density"
        ),
    )
    p.add_argument("--num-vehicles", type=int, default=600, help="background vehicles (excluding emergency vehicles)")
    p.add_argument("--depart-start", type=float, default=0.0)
    p.add_argument("--depart-end", type=float, default=900.0)

    # Backward-compatible single-EV knobs
    p.add_argument("--ev-id", default="emergency1", help="used when --num-emergency-vehicles=1")
    p.add_argument("--ev-depart", type=float, default=30.0, help="used as first EV depart time")

    # Multi-EV knobs
    p.add_argument("--num-emergency-vehicles", type=int, default=1, help="number of emergency vehicles to generate")
    p.add_argument("--ev-prefix", default="emergency", help="emergency vehicle ID prefix (emergency1, emergency2, ...)")
    p.add_argument("--ev-depart-interval", type=float, default=30.0, help="depart spacing between consecutive emergency vehicles")
    p.add_argument("--ev-depart-start", type=float, default=None, help="optional override for first emergency depart time")

    p.add_argument("--veh-prefix", default="veh")
    p.add_argument("--min-route-edges", type=int, default=6)
    p.add_argument("--min-ev-route-edges", type=int, default=16)
    p.add_argument("--max-route-sample-tries", type=int, default=800)
    p.add_argument(
        "--density-options",
        default="",
        help=(
            "optional comma-separated density map label=count (e.g., smooth=200,moderate=500,severe=1000). "
            "If set, generates one route file per density with fixed EV routes and nested background subsets."
        ),
    )
    p.add_argument(
        "--density-out-dir",
        default="",
        help=(
            "output directory for --density-options files. "
            "Default: sibling folder next to --out-routes named route_profiles."
        ),
    )
    p.add_argument(
        "--density-depart-end-options",
        default="",
        help=(
            "optional comma-separated label=depart_end map for density-specific burst control "
            "(e.g., smooth=900,moderate=700,severe=500)."
        ),
    )
    p.add_argument(
        "--density-nested-subsets",
        action="store_true",
        default=True,
        help="use nested background subsets across densities (default: enabled)",
    )
    p.add_argument(
        "--no-density-nested-subsets",
        dest="density_nested_subsets",
        action="store_false",
        help="disable nested subsets for --density-options",
    )
    return p.parse_args()


def _parse_label_count_map(raw: str) -> List[Tuple[str, int]]:
    out: List[Tuple[str, int]] = []
    for tok in str(raw or "").split(","):
        t = tok.strip()
        if not t:
            continue
        if "=" not in t:
            raise ValueError(f"Invalid token '{t}', expected label=value")
        k, v = t.split("=", 1)
        label = str(k).strip()
        try:
            value = int(str(v).strip())
        except Exception as e:
            raise ValueError(f"Invalid integer value in token '{t}'") from e
        if not label:
            raise ValueError(f"Invalid empty label in token '{t}'")
        if value <= 0:
            raise ValueError(f"Invalid non-positive value in token '{t}'")
        out.append((label, value))
    return out


def _parse_label_float_map(raw: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for tok in str(raw or "").split(","):
        t = tok.strip()
        if not t:
            continue
        if "=" not in t:
            raise ValueError(f"Invalid token '{t}', expected label=value")
        k, v = t.split("=", 1)
        label = str(k).strip()
        try:
            value = float(str(v).strip())
        except Exception as e:
            raise ValueError(f"Invalid float value in token '{t}'") from e
        if not label:
            raise ValueError(f"Invalid empty label in token '{t}'")
        out[label] = float(value)
    return out


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    # Backward-compatible default behavior: when no dedicated seeds are provided,
    # keep using one RNG stream for both background and EV route sampling.
    if args.background_seed is None and args.ev_seed is None:
        rng_bg = rng
        rng_ev = rng
    else:
        bg_seed = args.seed if args.background_seed is None else int(args.background_seed)
        ev_seed = (args.seed + 7919) if args.ev_seed is None else int(args.ev_seed)
        rng_bg = random.Random(bg_seed)
        rng_ev = random.Random(ev_seed)

    net_file = Path(args.net_file)
    out_routes = Path(args.out_routes)
    if not net_file.exists():
        raise FileNotFoundError(f"Net file not found: {net_file}")

    car_edges, car_adj, car_indegree, car_outdegree = load_network_graph(net_file, vehicle_class="passenger")
    ev_edges, ev_adj, ev_indegree, ev_outdegree = load_network_graph(net_file, vehicle_class="emergency")

    car_edge_ids = sorted(car_edges.keys())
    car_src_pool = [eid for eid in car_edge_ids if car_outdegree.get(eid, 0) > 0]
    car_dst_pool = [eid for eid in car_edge_ids if car_indegree.get(eid, 0) > 0]

    ev_edge_ids = sorted(ev_edges.keys())
    ev_src_pool = [eid for eid in ev_edge_ids if ev_outdegree.get(eid, 0) > 0]
    ev_dst_pool = [eid for eid in ev_edge_ids if ev_indegree.get(eid, 0) > 0]
    ev_entry_edges = [eid for eid in ev_edge_ids if ev_indegree.get(eid, 0) == 0 and ev_outdegree.get(eid, 0) > 0]
    ev_exit_edges = [eid for eid in ev_edge_ids if ev_outdegree.get(eid, 0) == 0 and ev_indegree.get(eid, 0) > 0]

    if not car_src_pool or not car_dst_pool:
        raise RuntimeError("Could not find enough passenger-allowed edges to build background routes")
    if not ev_src_pool or not ev_dst_pool:
        raise RuntimeError("Could not find enough emergency-allowed edges to build EV routes")

    n_ev = max(1, int(args.num_emergency_vehicles))
    first_depart = float(args.ev_depart) if args.ev_depart_start is None else float(args.ev_depart_start)
    emergency_ids: List[str]
    emergency_departs: List[float]
    if n_ev == 1:
        emergency_ids = [str(args.ev_id)]
        emergency_departs = [float(args.ev_depart)]
    else:
        emergency_ids = [f"{args.ev_prefix}{i+1}" for i in range(n_ev)]
        emergency_departs = [first_depart + (i * float(args.ev_depart_interval)) for i in range(n_ev)]

    emergency_routes: List[Tuple[str, str, float, List[str]]] = []
    for idx in range(n_ev):
        ev_route_edges = sample_route(
            rng_ev,
            ev_edge_ids,
            ev_adj,
            min_edges=max(2, int(args.min_ev_route_edges)),
            max_tries=max(int(args.max_route_sample_tries), 2000),
            src_pool=ev_entry_edges or ev_src_pool,
            dst_pool=ev_exit_edges or ev_dst_pool,
        )
        emergency_routes.append((str(emergency_ids[idx]), "ev", float(emergency_departs[idx]), ev_route_edges))

    density_options = _parse_label_count_map(str(args.density_options or ""))
    density_depart_end = _parse_label_float_map(str(args.density_depart_end_options or ""))

    if not density_options:
        background: List[Tuple[str, str, float, List[str]]] = []
        for i in range(int(args.num_vehicles)):
            depart = rng_bg.uniform(float(args.depart_start), float(args.depart_end))
            route_edges = sample_route(
                rng_bg,
                car_edge_ids,
                car_adj,
                min_edges=max(2, int(args.min_route_edges)),
                max_tries=int(args.max_route_sample_tries),
                src_pool=car_src_pool,
                dst_pool=car_dst_pool,
            )
            veh_id = f"{args.veh_prefix}{i+1:04d}"
            background.append((veh_id, "car", depart, route_edges))

        out_routes.parent.mkdir(parents=True, exist_ok=True)
        write_routes(out_routes, background, emergency_routes)

        print(f"Wrote routes to {out_routes}")
        print(f"Passenger edges: {len(car_edge_ids)} | emergency edges: {len(ev_edge_ids)}")
        print(f"EV entry edges: {len(ev_entry_edges)} | EV exit edges: {len(ev_exit_edges)}")
        print(f"Background vehicles: {len(background)} | emergency vehicles: {len(emergency_routes)}")
    else:
        max_count = max(c for _, c in density_options)
        bg_pool: List[Tuple[str, str, float, List[str]]] = []
        for i in range(int(max_count)):
            route_edges = sample_route(
                rng_bg,
                car_edge_ids,
                car_adj,
                min_edges=max(2, int(args.min_route_edges)),
                max_tries=int(args.max_route_sample_tries),
                src_pool=car_src_pool,
                dst_pool=car_dst_pool,
            )
            veh_id = f"{args.veh_prefix}{i+1:04d}"
            bg_pool.append((veh_id, "car", 0.0, route_edges))

        idx = list(range(len(bg_pool)))
        rng_bg.shuffle(idx)
        if not bool(args.density_nested_subsets):
            idx_by_label: Dict[str, List[int]] = {}
            for label, count in density_options:
                sample = list(idx)
                rng_bg.shuffle(sample)
                idx_by_label[label] = sample[: int(count)]
        else:
            idx_by_label = {label: idx[: int(count)] for label, count in density_options}

        out_dir = Path(args.density_out_dir).resolve() if str(args.density_out_dir).strip() else (out_routes.parent / "route_profiles").resolve()
        out_dir.mkdir(parents=True, exist_ok=True)

        print(f"Generating density profile routes in {out_dir}")
        for label, count in density_options:
            depart_end = float(density_depart_end.get(str(label), float(args.depart_end)))
            selected = [bg_pool[i] for i in idx_by_label[str(label)]]
            background: List[Tuple[str, str, float, List[str]]] = []
            for veh_id, vtype, _d0, edges in selected:
                depart = rng_bg.uniform(float(args.depart_start), float(depart_end))
                background.append((veh_id, vtype, depart, edges))

            density_file = out_dir / f"madrid_traffic_{label}_d{int(count)}.rou.xml"
            write_routes(density_file, background, emergency_routes)
            print(
                f"  [{label}] vehicles={int(count)} depart_end={depart_end:.1f} "
                f"evs={len(emergency_routes)} -> {density_file}"
            )

        print(f"Passenger edges: {len(car_edge_ids)} | emergency edges: {len(ev_edge_ids)}")
        print(f"EV entry edges: {len(ev_entry_edges)} | EV exit edges: {len(ev_exit_edges)}")
        print(f"Background pool size: {len(bg_pool)} | emergency vehicles: {len(emergency_routes)}")

    if emergency_routes:
        for ev_id, _, ev_depart, ev_edges in emergency_routes[: min(5, len(emergency_routes))]:
            print(
                f"Emergency route [{ev_id}]: {ev_edges[0]} -> {ev_edges[-1]} | edges={len(ev_edges)} | depart={float(ev_depart):.1f}s"
            )
        if len(emergency_routes) > 5:
            print(f"... ({len(emergency_routes) - 5} more emergency vehicles)")
        print(f"Suggested --emergency-veh for focused run: {emergency_routes[0][0]}")

    if background:
        sample_ids = ",".join([background[i][0] for i in range(min(3, len(background)))])
        print(f"Suggested --vehicles list for telemetry: {sample_ids}")


if __name__ == "__main__":
    main()
