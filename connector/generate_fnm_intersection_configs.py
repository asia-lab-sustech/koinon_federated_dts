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


def _parse_net(net_file: str) -> tuple[Set[str], Dict[str, str]]:
    tls_nodes: Set[str] = set()
    edge_to_to_node: Dict[str, str] = {}
    if not net_file:
        return tls_nodes, edge_to_to_node

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
        to_node = str(e.get("to", "") or "")
        if to_node:
            edge_to_to_node[eid] = to_node

    return tls_nodes, edge_to_to_node


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
    tls_nodes, edge_to_to_node = _parse_net(str(args.net_file or ""))

    selected_tls: List[str] = []

    if str(args.tls_list).strip():
        selected_tls = _ordered_unique([x.strip() for x in str(args.tls_list).split(",") if x.strip()])
    elif bool(args.all_tls):
        selected_tls = sorted(list(tls_nodes))
    else:
        route_edges = _extract_ev_route_edges(str(args.route_file or ""), str(args.ev_id or "emergency1"))
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
            out_file.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
        else:
            out_file.write_text(_make_cfg_text_fallback(template_text, tls), encoding="utf-8")
        generated.append(str(out_file))

    manifest_path = Path(args.manifest_file).resolve() if str(args.manifest_file).strip() else (out_dir / "fnm_intersection_manifest.json")
    manifest = {
        "template": str(template_path),
        "out_dir": str(out_dir),
        "n_generated": len(generated),
        "tls_ids": selected_tls,
        "files": generated,
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=True, indent=2), encoding="utf-8")

    print(json.dumps(manifest, ensure_ascii=True))


if __name__ == "__main__":
    main()
