#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Set

try:
    import yaml  # type: ignore
except Exception:
    yaml = None  # type: ignore


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Generate per-intersection FNM YAML configs from template + EV route/net."
    )
    ap.add_argument("--template", required=True, help="base intersection FNM template yaml")
    ap.add_argument("--out-dir", required=True, help="output directory for generated YAML files")
    ap.add_argument("--tls-list", default="", help="comma-separated TLS ids (overrides route parsing)")
    ap.add_argument("--route-file", default="", help="SUMO route file (.rou.xml)")
    ap.add_argument("--ev-id", default="emergency1", help="EV id in route file")
    ap.add_argument("--net-file", default="", help="SUMO network file (.net.xml)")
    ap.add_argument("--all-tls", action="store_true", default=False, help="generate for all traffic-light junctions in net")
    ap.add_argument("--clean-out-dir", action="store_true", default=False, help="remove previous fnm_intersection_*.yml in out-dir")
    ap.add_argument("--manifest-file", default="", help="optional manifest json file path")
    ap.add_argument(
        "--f2d-contextual-subscriptions-enable",
        action="store_true",
        default=False,
        help=(
            "augment generated intersection FNMs with F2D route-relevant node/edge "
            "downstream-context subscriptions"
        ),
    )
    ap.add_argument("--f2d-contextual-back-nodes", type=int, default=2)
    ap.add_argument("--f2d-contextual-forward-nodes", type=int, default=6)
    ap.add_argument("--f2d-contextual-back-edges", type=int, default=2)
    ap.add_argument("--f2d-contextual-forward-edges", type=int, default=8)
    return ap.parse_args()


def _ordered_unique(xs: List[str]) -> List[str]:
    out: List[str] = []
    seen: Set[str] = set()
    for x in xs:
        k = str(x).strip()
        if not k or k in seen:
            continue
        seen.add(k)
        out.append(k)
    return out


def _parse_net(net_file: str) -> tuple[Set[str], Dict[str, str], Dict[str, str]]:
    tls_nodes: Set[str] = set()
    edge_to_from_node: Dict[str, str] = {}
    edge_to_to_node: Dict[str, str] = {}
    if not net_file:
        return tls_nodes, edge_to_from_node, edge_to_to_node

    root = ET.parse(net_file).getroot()

    for j in root.findall("junction"):
        jid = str(j.get("id", "") or "")
        jtype = str(j.get("type", "") or "")
        if jid and jtype.startswith("traffic_light"):
            tls_nodes.add(jid)

    for e in root.findall("edge"):
        eid = str(e.get("id", "") or "")
        if not eid or eid.startswith(":"):
            continue
        from_node = str(e.get("from", "") or "")
        to_node = str(e.get("to", "") or "")
        if from_node:
            edge_to_from_node[eid] = from_node
        if to_node:
            edge_to_to_node[eid] = to_node

    return tls_nodes, edge_to_from_node, edge_to_to_node


def _infer_to_node_from_edge_name(edge_id: str) -> str:
    # Handles "EdgeNode491-Node490" or "NodeA-NodeB"
    e = str(edge_id or "")
    if "-" not in e:
        return ""
    right = e.rsplit("-", 1)[-1].strip()
    return right


def _extract_ev_route_edges(route_file: str, ev_id: str) -> List[str]:
    if not route_file:
        return []
    root = ET.parse(route_file).getroot()
    for veh in root.findall("vehicle"):
        if str(veh.get("id", "") or "") != str(ev_id):
            continue
        route_el = veh.find("route")
        if route_el is None:
            continue
        edges_attr = str(route_el.get("edges", "") or "").strip()
        if edges_attr:
            return [x for x in edges_attr.split() if x]
    return []


def _derive_tls_from_route_edges(route_edges: List[str], edge_to_to_node: Dict[str, str], tls_nodes: Set[str]) -> List[str]:
    tls_seq: List[str] = []
    for e in list(route_edges or []):
        if str(e).startswith(":"):
            continue
        to_node = edge_to_to_node.get(str(e)) or _infer_to_node_from_edge_name(str(e))
        if not to_node:
            continue
        if tls_nodes and to_node not in tls_nodes:
            continue
        tls_seq.append(str(to_node))
    return _ordered_unique(tls_seq)


def _derive_route_nodes(
    route_edges: List[str],
    edge_to_from_node: Dict[str, str],
    edge_to_to_node: Dict[str, str],
) -> List[str]:
    nodes: List[str] = []
    for e in list(route_edges or []):
        edge_id = str(e)
        if not edge_id or edge_id.startswith(":"):
            continue
        from_node = edge_to_from_node.get(edge_id, "")
        to_node = edge_to_to_node.get(edge_id, "") or _infer_to_node_from_edge_name(edge_id)
        if from_node and not nodes:
            nodes.append(str(from_node))
        if to_node:
            nodes.append(str(to_node))
    return _ordered_unique(nodes)


def _route_context_for_tls(
    *,
    route_edges: List[str],
    route_nodes: List[str],
    tls_id: str,
    back_nodes: int,
    forward_nodes: int,
    back_edges: int,
    forward_edges: int,
) -> Dict[str, List[str]]:
    tls_s = str(tls_id)
    try:
        node_idx = list(route_nodes).index(tls_s)
    except ValueError:
        node_idx = 0
    node_lo = max(0, int(node_idx) - max(0, int(back_nodes)))
    node_hi = min(len(route_nodes), int(node_idx) + max(0, int(forward_nodes)) + 1)
    edge_lo = max(0, int(node_idx) - max(0, int(back_edges)))
    edge_hi = min(len(route_edges), int(node_idx) + max(0, int(forward_edges)) + 1)
    return {
        "nodes": _ordered_unique([str(n) for n in route_nodes[node_lo:node_hi] if str(n)]),
        "edges": _ordered_unique([
            str(e) for e in route_edges[edge_lo:edge_hi] if str(e) and not str(e).startswith(":")
        ]),
    }


def _safe_rule_token(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", str(value or "")).strip("_") or "unknown"


def _ensure_list_path(cfg: Dict, path: List[str]) -> List:
    cur = cfg
    for p in path[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    key = path[-1]
    if key not in cur or not isinstance(cur[key], list):
        cur[key] = []
    return cur[key]


def _add_unique_list_values(cfg: Dict, path: List[str], values: List[str]) -> None:
    xs = _ensure_list_path(cfg, path)
    seen = {str(x) for x in xs}
    for v in values:
        s = str(v)
        if s and s not in seen:
            xs.append(s)
            seen.add(s)


def _add_f2d_contextual_rules(cfg: Dict, tls_id: str, context: Dict[str, List[str]]) -> Dict:
    rules = _ensure_list_path(cfg, ["node", "communication", "topic_map", "rules"])
    existing = {str(r.get("name", "")) for r in rules if isinstance(r, dict)}
    for node_id in list(context.get("nodes", []) or []):
        n = str(node_id).strip()
        if not n:
            continue
        rule_name = f"f2d_context_node_{_safe_rule_token(n)}_down"
        if rule_name in existing:
            continue
        rules.append({
            "name": rule_name,
            "direction": "fed_to_local",
            "subscribe_topic": f"federation/v1/context/downstream/node/{n}",
            "publish_topic": f"rw/tls/{tls_id}/downstream_context",
            "event_type": "DownstreamContext",
        })
        existing.add(rule_name)
    for edge_id in list(context.get("edges", []) or []):
        e = str(edge_id).strip()
        if not e:
            continue
        rule_name = f"f2d_context_edge_{_safe_rule_token(e)}_down"
        if rule_name in existing:
            continue
        rules.append({
            "name": rule_name,
            "direction": "fed_to_local",
            "subscribe_topic": f"federation/v1/context/downstream/edge/{e}",
            "publish_topic": f"rw/tls/{tls_id}/downstream_context",
            "event_type": "DownstreamContext",
        })
        existing.add(rule_name)
    _add_unique_list_values(cfg, ["node", "capabilities"], ["downstream_context_consume"])
    _add_unique_list_values(
        cfg,
        ["node", "federation_context", "discovery_service_names"],
        ["downstream_context_down", "f2d_contextual_downstream_context"],
    )
    _add_unique_list_values(
        cfg,
        ["node", "federation_context", "discovery_directions"],
        ["fed_to_local"],
    )
    _add_unique_list_values(
        cfg,
        ["node", "federation_context", "discovery_capabilities"],
        ["downstream_context_consume"],
    )
    prof = cfg.get("node", {}).get("dt_profile", {})
    if isinstance(prof, dict):
        tags = prof.get("policy_tags", [])
        if not isinstance(tags, list):
            tags = []
        for tag in ("f2d-contextual-subscriber", "drone-downstream-context-consumer"):
            if tag not in tags:
                tags.append(tag)
        prof["policy_tags"] = tags
        cfg["node"]["dt_profile"] = prof
    return cfg


def _set_nested(d: Dict, path: List[str], value) -> None:
    cur = d
    for p in path[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    cur[path[-1]] = value


def _make_cfg(base_cfg: Dict, tls_id: str) -> Dict:
    cfg = copy.deepcopy(base_cfg)
    _set_nested(cfg, ["node", "dt_id"], str(tls_id))
    _set_nested(cfg, ["node", "gateway_id"], f"gw-tls-{str(tls_id).lower()}")

    prof = cfg.get("node", {}).get("dt_profile", {})
    if isinstance(prof, dict):
        desc = str(prof.get("dt_description", "") or "")
        if desc:
            prof["dt_description"] = f"{desc} ({tls_id})"
        geo = prof.get("geo_scope", {})
        if isinstance(geo, dict):
            geo["id"] = f"tls.{str(tls_id).lower()}"
            prof["geo_scope"] = geo
        cfg["node"]["dt_profile"] = prof

    return cfg


def _make_cfg_text_fallback(template_text: str, tls_id: str) -> str:
    txt = str(template_text or "")
    tls = str(tls_id)
    tls_l = tls.lower()

    # Conservative line-level substitutions for known keys in template.
    txt = re.sub(r'(^\s*dt_id:\s*").*?(".*$)', rf'\1{tls}\2', txt, flags=re.M)
    txt = re.sub(r'(^\s*gateway_id:\s*").*?(".*$)', rf'\1gw-tls-{tls_l}\2', txt, flags=re.M)
    txt = re.sub(r'(^\s*id:\s*").*?(".*$)', rf'\1tls.{tls_l}\2', txt, flags=re.M)

    # Append TLS in description if not already dynamic.
    m = re.search(r'(^\s*dt_description:\s*")(.*?)(".*$)', txt, flags=re.M)
    if m:
        base_desc = m.group(2)
        if f"({tls})" not in base_desc:
            rep = f'{m.group(1)}{base_desc} ({tls}){m.group(3)}'
            txt = txt[: m.start()] + rep + txt[m.end() :]
    return txt


def main() -> None:
    args = _parse_args()
    template_path = Path(args.template).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    template_text = template_path.read_text(encoding="utf-8")
    base_cfg: Dict = {}
    if yaml is not None:
        base_cfg = yaml.safe_load(template_text) or {}
    tls_nodes, edge_to_from_node, edge_to_to_node = _parse_net(str(args.net_file or ""))

    selected_tls: List[str] = []

    route_edges: List[str] = []
    route_nodes: List[str] = []
    if str(args.tls_list).strip():
        selected_tls = _ordered_unique([x.strip() for x in str(args.tls_list).split(",") if x.strip()])
    elif bool(args.all_tls):
        selected_tls = sorted(list(tls_nodes))
    else:
        route_edges = _extract_ev_route_edges(str(args.route_file or ""), str(args.ev_id or "emergency1"))
        route_nodes = _derive_route_nodes(route_edges, edge_to_from_node, edge_to_to_node)
        selected_tls = _derive_tls_from_route_edges(route_edges, edge_to_to_node, tls_nodes)

    if not selected_tls:
        raise SystemExit(
            "No TLS ids selected. Provide --tls-list, or --route-file/--ev-id with --net-file, or --all-tls."
        )

    if bool(args.clean_out_dir):
        for p in out_dir.glob("fnm_intersection_*.yml"):
            p.unlink(missing_ok=True)

    generated: List[str] = []
    for tls in selected_tls:
        out_file = out_dir / f"fnm_intersection_{tls}.yml"
        if yaml is not None:
            cfg = _make_cfg(base_cfg, tls)
            if bool(args.f2d_contextual_subscriptions_enable):
                if not route_edges:
                    route_edges = _extract_ev_route_edges(str(args.route_file or ""), str(args.ev_id or "emergency1"))
                if not route_nodes:
                    route_nodes = _derive_route_nodes(route_edges, edge_to_from_node, edge_to_to_node)
                ctx = _route_context_for_tls(
                    route_edges=route_edges,
                    route_nodes=route_nodes,
                    tls_id=str(tls),
                    back_nodes=int(args.f2d_contextual_back_nodes),
                    forward_nodes=int(args.f2d_contextual_forward_nodes),
                    back_edges=int(args.f2d_contextual_back_edges),
                    forward_edges=int(args.f2d_contextual_forward_edges),
                )
                cfg = _add_f2d_contextual_rules(cfg, str(tls), ctx)
            out_file.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
        else:
            if bool(args.f2d_contextual_subscriptions_enable):
                raise SystemExit("PyYAML is required for --f2d-contextual-subscriptions-enable")
            out_file.write_text(_make_cfg_text_fallback(template_text, tls), encoding="utf-8")
        generated.append(str(out_file))

    manifest_path = Path(args.manifest_file).resolve() if str(args.manifest_file).strip() else (out_dir / "fnm_intersection_manifest.json")
    manifest = {
        "template": str(template_path),
        "out_dir": str(out_dir),
        "n_generated": len(generated),
        "tls_ids": selected_tls,
        "files": generated,
        "f2d_contextual_subscriptions": {
            "enabled": bool(args.f2d_contextual_subscriptions_enable),
            "back_nodes": int(args.f2d_contextual_back_nodes),
            "forward_nodes": int(args.f2d_contextual_forward_nodes),
            "back_edges": int(args.f2d_contextual_back_edges),
            "forward_edges": int(args.f2d_contextual_forward_edges),
        },
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=True, indent=2), encoding="utf-8")

    print(json.dumps(manifest, ensure_ascii=True))


if __name__ == "__main__":
    main()
