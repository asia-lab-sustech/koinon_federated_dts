#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Set, Tuple

try:
    import yaml  # type: ignore
except Exception:
    yaml = None  # type: ignore


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Batch-generate FNM config sets for multiple EV routes."
    )
    ap.add_argument("--template", required=True, help="intersection FNM template yaml")
    ap.add_argument("--out-root", required=True, help="output root directory")
    ap.add_argument("--route-file", required=True, help="SUMO route file (.rou.xml)")
    ap.add_argument("--net-file", required=True, help="SUMO network file (.net.xml)")
    ap.add_argument("--ev-ids", default="", help="comma-separated EV ids; default=auto detect emergency*")
    ap.add_argument(
        "--mode",
        choices=["per-ev", "union", "both"],
        default="both",
        help="per-ev directories, union directory, or both",
    )
    ap.add_argument("--ev-template", default="", help="optional EV FNM template yaml (generate one per EV)")
    ap.add_argument("--clean-out-root", action="store_true", default=False, help="remove previous out-root")
    ap.add_argument("--manifest-file", default="", help="optional manifest json path")
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


def _parse_net(net_file: str) -> Tuple[Set[str], Dict[str, str]]:
    tls_nodes: Set[str] = set()
    edge_to_to_node: Dict[str, str] = {}
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
    e = str(edge_id or "")
    if "-" not in e:
        return ""
    return str(e.rsplit("-", 1)[-1].strip())


def _parse_vehicle_routes(route_file: str) -> Dict[str, List[str]]:
    root = ET.parse(route_file).getroot()
    out: Dict[str, List[str]] = {}
    for veh in root.findall("vehicle"):
        vid = str(veh.get("id", "") or "")
        if not vid:
            continue
        route_el = veh.find("route")
        if route_el is None:
            continue
        edges_attr = str(route_el.get("edges", "") or "").strip()
        if not edges_attr:
            continue
        out[vid] = [x for x in edges_attr.split() if x]
    return out


def _select_ev_ids(all_routes: Dict[str, List[str]], ev_ids_csv: str) -> List[str]:
    if str(ev_ids_csv).strip():
        ids = _ordered_unique([x.strip() for x in str(ev_ids_csv).split(",") if x.strip()])
        return [x for x in ids if x in all_routes]

    auto = [vid for vid in all_routes.keys() if str(vid).lower().startswith("emergency")]
    return _ordered_unique(sorted(auto))


def _derive_tls_from_route(route_edges: List[str], edge_to_to_node: Dict[str, str], tls_nodes: Set[str]) -> List[str]:
    tls_seq: List[str] = []
    for e in list(route_edges or []):
        if str(e).startswith(":"):
            continue
        to_node = edge_to_to_node.get(str(e)) or _infer_to_node_from_edge_name(str(e))
        if not to_node:
            continue
        if to_node not in tls_nodes:
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


def _cfg_for_intersection(base_cfg: Dict, tls_id: str) -> Dict:
    cfg = copy.deepcopy(base_cfg)
    _set_nested(cfg, ["node", "dt_id"], str(tls_id))
    _set_nested(cfg, ["node", "gateway_id"], f"gw-tls-{str(tls_id).lower()}")
    prof = cfg.get("node", {}).get("dt_profile", {})
    if isinstance(prof, dict):
        if "dt_description" in prof:
            prof["dt_description"] = f"{str(prof.get('dt_description') or '').rstrip()} ({tls_id})".strip()
        geo = prof.get("geo_scope", {})
        if isinstance(geo, dict):
            geo["id"] = f"tls.{str(tls_id).lower()}"
            prof["geo_scope"] = geo
        cfg["node"]["dt_profile"] = prof
    return cfg


def _cfg_for_ev(base_cfg: Dict, ev_id: str) -> Dict:
    cfg = copy.deepcopy(base_cfg)
    _set_nested(cfg, ["node", "dt_id"], str(ev_id))
    _set_nested(cfg, ["node", "gateway_id"], f"gw-ev-{str(ev_id).lower()}")
    url = str(
        cfg.get("node", {})
        .get("protocol_adaptation", {})
        .get("http_state_pull", {})
        .get("url", "")
    )
    if "{dt_id}" in url:
        _set_nested(
            cfg,
            ["node", "protocol_adaptation", "http_state_pull", "url"],
            str(url).replace("{dt_id}", str(ev_id)),
        )
    prof = cfg.get("node", {}).get("dt_profile", {})
    if isinstance(prof, dict):
        if "dt_description" in prof:
            prof["dt_description"] = f"{str(prof.get('dt_description') or '').rstrip()} ({ev_id})".strip()
        geo = prof.get("geo_scope", {})
        if isinstance(geo, dict):
            geo["id"] = f"ev.{str(ev_id).lower()}"
            prof["geo_scope"] = geo
        cfg["node"]["dt_profile"] = prof
    return cfg


def _text_replace_for_intersection(template_text: str, tls_id: str) -> str:
    txt = str(template_text or "")
    tls = str(tls_id)
    tls_l = tls.lower()
    txt = re.sub(r'(^\s*dt_id:\s*").*?(".*$)', rf'\1{tls}\2', txt, flags=re.M)
    txt = re.sub(r'(^\s*gateway_id:\s*").*?(".*$)', rf'\1gw-tls-{tls_l}\2', txt, flags=re.M)
    txt = re.sub(r'(^\s*id:\s*").*?(".*$)', rf'\1tls.{tls_l}\2', txt, flags=re.M)
    return txt


def _text_replace_for_ev(template_text: str, ev_id: str) -> str:
    txt = str(template_text or "")
    e = str(ev_id)
    e_l = e.lower()
    txt = re.sub(r'(^\s*dt_id:\s*").*?(".*$)', rf'\1{e}\2', txt, flags=re.M)
    txt = re.sub(r'(^\s*gateway_id:\s*").*?(".*$)', rf'\1gw-ev-{e_l}\2', txt, flags=re.M)
    txt = txt.replace("{dt_id}", e)
    txt = re.sub(r'(^\s*id:\s*").*?(".*$)', rf'\1ev.{e_l}\2', txt, flags=re.M)
    return txt


def _dump_yaml_or_text(path: Path, cfg: Dict, fallback_text: str) -> None:
    if yaml is not None:
        path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    else:
        path.write_text(fallback_text, encoding="utf-8")


def main() -> None:
    args = _parse_args()
    template_path = Path(args.template).resolve()
    out_root = Path(args.out_root).resolve()
    route_file = Path(args.route_file).resolve()
    net_file = Path(args.net_file).resolve()

    if bool(args.clean_out_root) and out_root.exists():
        shutil.rmtree(out_root)
    out_root.mkdir(parents=True, exist_ok=True)

    all_routes = _parse_vehicle_routes(str(route_file))
    ev_ids = _select_ev_ids(all_routes, str(args.ev_ids or ""))
    if not ev_ids:
        raise SystemExit("No EV ids selected. Provide --ev-ids or ensure route file has emergency* ids.")

    tls_nodes, edge_to_to_node = _parse_net(str(net_file))
    template_text = template_path.read_text(encoding="utf-8")
    base_inter_cfg = yaml.safe_load(template_text) if yaml is not None else {}

    ev_template_text = ""
    base_ev_cfg: Dict = {}
    if str(args.ev_template).strip():
        ev_template_path = Path(args.ev_template).resolve()
        ev_template_text = ev_template_path.read_text(encoding="utf-8")
        base_ev_cfg = yaml.safe_load(ev_template_text) if yaml is not None else {}

    mode = str(args.mode or "both")
    manifest: Dict = {
        "template": str(template_path),
        "ev_template": str(Path(args.ev_template).resolve()) if str(args.ev_template).strip() else "",
        "route_file": str(route_file),
        "net_file": str(net_file),
        "ev_ids": ev_ids,
        "mode": mode,
        "out_root": str(out_root),
        "per_ev": {},
        "union": {},
    }

    union_tls: List[str] = []

    if mode in ("per-ev", "both"):
        for ev_id in ev_ids:
            route_edges = list(all_routes.get(ev_id, []) or [])
            tls_seq = _derive_tls_from_route(route_edges, edge_to_to_node, tls_nodes)
            union_tls.extend(tls_seq)
            ev_dir = out_root / ev_id
            inter_dir = ev_dir / "intersections"
            inter_dir.mkdir(parents=True, exist_ok=True)

            files: List[str] = []
            for tls in tls_seq:
                out_file = inter_dir / f"fnm_intersection_{tls}.yml"
                if yaml is not None:
                    cfg = _cfg_for_intersection(base_inter_cfg, tls)
                    _dump_yaml_or_text(out_file, cfg, "")
                else:
                    _dump_yaml_or_text(out_file, {}, _text_replace_for_intersection(template_text, tls))
                files.append(str(out_file))

            ev_cfg_file = ""
            if base_ev_cfg or ev_template_text:
                ev_cfg_path = ev_dir / f"fnm_ev_{ev_id}.yml"
                if yaml is not None:
                    cfg = _cfg_for_ev(base_ev_cfg, ev_id)
                    _dump_yaml_or_text(ev_cfg_path, cfg, "")
                else:
                    _dump_yaml_or_text(ev_cfg_path, {}, _text_replace_for_ev(ev_template_text, ev_id))
                ev_cfg_file = str(ev_cfg_path)

            manifest["per_ev"][ev_id] = {
                "n_tls": len(tls_seq),
                "tls_ids": tls_seq,
                "intersection_dir": str(inter_dir),
                "intersection_files": files,
                "ev_cfg": ev_cfg_file,
            }

    if mode in ("union", "both"):
        tls_union = _ordered_unique(union_tls if union_tls else [
            x for ev_id in ev_ids for x in _derive_tls_from_route(list(all_routes.get(ev_id, []) or []), edge_to_to_node, tls_nodes)
        ])
        union_dir = out_root / "union" / "intersections"
        union_dir.mkdir(parents=True, exist_ok=True)
        files: List[str] = []
        for tls in tls_union:
            out_file = union_dir / f"fnm_intersection_{tls}.yml"
            if yaml is not None:
                cfg = _cfg_for_intersection(base_inter_cfg, tls)
                _dump_yaml_or_text(out_file, cfg, "")
            else:
                _dump_yaml_or_text(out_file, {}, _text_replace_for_intersection(template_text, tls))
            files.append(str(out_file))
        manifest["union"] = {
            "n_tls": len(tls_union),
            "tls_ids": tls_union,
            "intersection_dir": str(union_dir),
            "intersection_files": files,
        }

    manifest_file = Path(args.manifest_file).resolve() if str(args.manifest_file).strip() else (out_root / "fnm_batch_manifest.json")
    manifest_file.write_text(json.dumps(manifest, ensure_ascii=True, indent=2), encoding="utf-8")
    print(json.dumps({"ok": True, "manifest": str(manifest_file), "ev_ids": ev_ids}, ensure_ascii=True))


if __name__ == "__main__":
    main()
