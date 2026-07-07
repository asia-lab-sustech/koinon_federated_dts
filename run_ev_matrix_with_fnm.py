#!/usr/bin/env python3
"""Orchestrate EV matrix runs with per-scenario FNM sidecars.

Why this script:
- `launch_ev_matrix_experiments.py` already handles per-run `--emergency-veh` correctly from manifest.
- But FNM sidecars need scenario-specific EV/intersection configs (ev_id + route_file).
- This wrapper starts FNM sidecars per scenario, runs matrix launcher for that scenario, then stops sidecars.

Flow per scenario row in manifest:
1) start FNM sidecars (optionally auto-generate intersection configs for that route/ev)
2) run `launch_ev_matrix_experiments.py` with a single-row manifest and requested modes
3) stop sidecars
4) collect/aggregate results
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import random
import re
import shlex
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse
from pathlib import Path
from typing import Dict, List, Sequence

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


def _read_manifest(path: Path) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append({k: str(v) for k, v in row.items()})
    return rows


def _write_manifest(path: Path, rows: Sequence[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise RuntimeError("Cannot write empty manifest")
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return [{k: str(v) for k, v in row.items()} for row in csv.DictReader(f)]


def _write_csv(path: Path, rows: List[Dict[str, object]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fields))
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _render_ev_cfg_from_template_text(template_text: str, ev_id: str) -> str:
    """
    Lightweight EV config materialization from a single EV template.
    Keeps YAML structure intact and only replaces EV-specific identity fields.
    """
    txt = str(template_text or "")
    e = str(ev_id or "").strip()
    e_l = e.lower()
    if not e:
        return txt
    txt = re.sub(r'(^\s*dt_id:\s*").*?(".*$)', rf"\1{e}\2", txt, flags=re.M)
    txt = re.sub(r'(^\s*gateway_id:\s*").*?(".*$)', rf"\1gw-ev-{e_l}\2", txt, flags=re.M)
    txt = txt.replace("{dt_id}", e)
    txt = re.sub(r'(^\s*id:\s*").*?(".*$)', rf"\1ev.{e_l}\2", txt, flags=re.M)
    return txt


def _sync_ev_batch_configs_from_template(
    *,
    sim_root: Path,
    template_path: Path,
    ev_ids: List[str],
    ev_cfg_pattern: str,
    dry_run: bool = False,
) -> None:
    if not template_path.exists():
        raise SystemExit(f"Missing EV template source: {template_path}")
    template_text = template_path.read_text(encoding="utf-8")
    if not str(template_text).strip():
        raise SystemExit(f"EV template source is empty: {template_path}")

    for ev_id in list(ev_ids or []):
        ev = str(ev_id or "").strip()
        if not ev:
            continue
        out_path = (sim_root / str(ev_cfg_pattern).format(ev_id=ev)).resolve()
        rendered = _render_ev_cfg_from_template_text(template_text, ev)
        if dry_run:
            print(
                f"[matrix+fnm] DRY_RUN sync_ev_batch_cfg ev={ev} src={template_path} dst={out_path}"
            )
            continue
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(rendered, encoding="utf-8")
        print(
            f"[matrix+fnm] sync_ev_batch_cfg ev={ev} src={template_path} dst={out_path}"
        )


def _select_http_port(
    *,
    strategy: str,
    host: str,
    base: int,
    step: int,
    scenario_idx: int,
) -> int | None:
    mode = str(strategy or "none").strip().lower()
    if mode == "none":
        return None

    base = int(base)
    step = max(1, int(step))
    preferred = max(1024, base + (max(1, int(scenario_idx)) - 1) * step)

    if mode in {"fixed", "incremental"}:
        return preferred

    if mode == "free":
        # Random bindable port selection to avoid repeated collisions on
        # deterministic bases (e.g., 20000) across repeated experiments.
        # We use a deterministic seed per process/scenario for reproducibility
        # within a single run while still spreading ports globally.
        lo = max(15000, preferred + 500)
        hi = 62000
        if lo >= hi:
            lo, hi = 20000, 62000
        rng = random.Random((os.getpid() << 16) ^ int(time.time()) ^ int(scenario_idx))
        tried: set[int] = set()
        max_tries = 256
        for _ in range(max_tries):
            cand = int(rng.randint(lo, hi))
            if cand in tried:
                continue
            tried.add(cand)
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind((str(host), int(cand)))
                return int(cand)
            except Exception:
                continue
        # Fallback: bounded linear scan near preferred.
        start = max(20000, preferred + 1000)
        for cand in range(int(start), int(start) + 4000):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind((str(host), int(cand)))
                return int(cand)
            except Exception:
                continue
        raise SystemExit(
            f"Could not find bindable EV HTTP port for scenario={scenario_idx} "
            f"host={host} start={start}"
        )

    raise SystemExit(f"Unsupported --ev-http-port-strategy: {strategy}")


def _rewrite_ev_cfg_http_port(src_cfg: Path, dst_cfg: Path, http_port: int) -> None:
    if yaml is None:
        raise SystemExit(
            "PyYAML is required for runtime EV config patching. "
            "Activate your project venv (e.g., fdt_venv) and rerun."
        )
    try:
        raw_obj = yaml.safe_load(src_cfg.read_text(encoding="utf-8"))
    except Exception as e:
        raise SystemExit(f"Invalid EV config YAML (cannot parse): {src_cfg} ({e})") from e
    if not isinstance(raw_obj, dict):
        raise SystemExit(f"Invalid EV config YAML structure (expected mapping): {src_cfg}")

    obj: Dict[str, object] = dict(raw_obj)
    node = dict(obj.get("node") or {})
    if not node:
        raise SystemExit(f"Invalid EV config YAML (missing node): {src_cfg}")

    ev_id = str(node.get("dt_id") or "emergency1").strip()
    protocol_adaptation = dict(node.get("protocol_adaptation") or {})
    http_pull = dict(protocol_adaptation.get("http_state_pull") or {})
    url = str(http_pull.get("url") or "").strip()
    if not url:
        raise SystemExit(f"Could not patch HTTP pull URL port in EV config (missing url): {src_cfg}")

    p = urlparse(url)
    if not (p.scheme and p.path):
        raise SystemExit(f"Could not patch HTTP pull URL port in EV config (invalid url): {src_cfg}")
    host = p.hostname or "127.0.0.1"
    new_url = f"{p.scheme}://{host}:{int(http_port)}{p.path}"
    http_pull["url"] = new_url
    protocol_adaptation["http_state_pull"] = http_pull
    node["protocol_adaptation"] = protocol_adaptation

    monitor = dict(node.get("monitor") or {})
    rules = list(monitor.get("rules") or [])
    if not isinstance(rules, list):
        rules = []

    def _ensure_rule(name: str, payload: Dict[str, str]) -> None:
        for r in rules:
            if isinstance(r, dict) and str(r.get("name") or "") == name:
                return
        rules.append(dict(payload))

    _ensure_rule(
        "ev_local_state_observed",
        {
            "name": "ev_local_state_observed",
            "source": "local",
            "kind": "state",
            "subscribe_topic": f"rw/vehicle_agent/{ev_id}/state",
            "state_key": f"monitor.ev.state.{ev_id}",
        },
    )
    _ensure_rule(
        "intersection_state_seen",
        {
            "name": "intersection_state_seen",
            "source": "federation",
            "kind": "state",
            "subscribe_topic": "federation/v1/state/intersection/+",
            "state_key": "monitor.intersection.state.{source_dt_id}",
        },
    )
    _ensure_rule(
        "intersection_event_seen",
        {
            "name": "intersection_event_seen",
            "source": "federation",
            "kind": "event",
            "subscribe_topic": "federation/v1/event/intersection/+",
            "event_name": "monitor.intersection.event",
        },
    )
    monitor["rules"] = rules
    node["monitor"] = monitor

    obj["node"] = node
    dst_cfg.parent.mkdir(parents=True, exist_ok=True)
    dst_cfg.write_text(yaml.safe_dump(obj, sort_keys=False, allow_unicode=False), encoding="utf-8")


def _validate_ev_runtime_cfg(cfg_path: Path, expected_http_port: int) -> tuple[bool, str]:
    if yaml is None:
        return False, "yaml_unavailable"
    try:
        raw_obj = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    except Exception as e:
        return False, f"runtime_yaml_parse_error: {e}"
    if not isinstance(raw_obj, dict):
        return False, "runtime_yaml_not_mapping"
    node = dict(raw_obj.get("node") or {})
    if not node:
        return False, "runtime_yaml_missing_node"
    protocol_adaptation = dict(node.get("protocol_adaptation") or {})
    http_pull = dict(protocol_adaptation.get("http_state_pull") or {})
    url = str(http_pull.get("url") or "").strip()
    if not url:
        return False, "runtime_yaml_missing_http_state_pull_url"
    p = urlparse(url)
    if not (p.scheme and p.path):
        return False, "runtime_yaml_invalid_http_state_pull_url"
    if int(p.port or -1) != int(expected_http_port):
        return False, (
            f"runtime_yaml_http_port_mismatch expected={int(expected_http_port)} "
            f"got={int(p.port or -1)} url={url}"
        )
    monitor = dict(node.get("monitor") or {})
    rules = list(monitor.get("rules") or [])
    names = {
        str(r.get("name") or "")
        for r in rules
        if isinstance(r, dict)
    }
    required = {"ev_local_state_observed", "intersection_state_seen", "intersection_event_seen"}
    missing = sorted(required - names)
    if missing:
        return False, f"runtime_yaml_missing_monitor_rules: {','.join(missing)}"
    return True, "ok"


def _ev_sidecar_preflight(
    log_dir: Path,
    timeout_sec: float,
    *,
    require_fcm_config_loaded: bool = True,
    min_fcm_context_updates: int = 1,
    require_federation_mqtt_connected: bool = True,
) -> tuple[bool, str]:
    ev_stdout = log_dir / "fnm_ev.stdout.log"
    ev_jsonl = log_dir / "fnm_ev.jsonl"
    deadline = time.time() + max(0.2, float(timeout_sec))
    parse_err_markers = ("ParserError", "Traceback", "Invalid EV config YAML")
    last_start_ok = False
    last_federation_mqtt_ok = False
    last_federation_mqtt_warning = False
    last_fcm_cfg_ok = (not bool(require_fcm_config_loaded))
    last_fcm_ctx_n = 0
    saw_jsonl = False
    saw_stdout = False
    while time.time() < deadline:
        try:
            if ev_stdout.exists():
                saw_stdout = True
                txt = ev_stdout.read_text(encoding="utf-8", errors="replace")
                if any(m in txt for m in parse_err_markers):
                    return False, "fnm_ev_startup_error_in_stdout"
            if ev_jsonl.exists() and ev_jsonl.stat().st_size > 0:
                saw_jsonl = True
                start_ok = False
                federation_mqtt_ok = False
                fcm_cfg_ok = (not bool(require_fcm_config_loaded))
                fcm_ctx_n = 0
                lines = ev_jsonl.read_text(encoding="utf-8", errors="replace").splitlines()[-1000:]
                for line in lines:
                        line = str(line).strip()
                        if not line:
                            continue
                        try:
                            obj = dict(json.loads(line))
                        except Exception:
                            continue
                        evt = str(obj.get("event") or obj.get("evt") or "")
                        if evt in {"fnm.start", "fnm.runtime.build"}:
                            start_ok = True
                        elif evt == "fnm.mqtt.connected":
                            start_ok = True
                            if str(obj.get("iface", "")) == "federation" and int(obj.get("connected", 0) or 0) == 1:
                                federation_mqtt_ok = True
                        elif evt == "fnm.mqtt.startup_connectivity":
                            if int(obj.get("federation_connected", 0) or 0) == 1:
                                federation_mqtt_ok = True
                        elif evt == "fnm.mqtt.startup_warning" and str(obj.get("reason", "")) == "federation_mqtt_not_connected":
                            last_federation_mqtt_warning = True
                        elif evt == "fcm.config.loaded":
                            fcm_cfg_ok = True
                        elif evt == "fcm.query_context.update":
                            fcm_ctx_n += 1
                last_start_ok = bool(start_ok)
                last_federation_mqtt_ok = bool(federation_mqtt_ok)
                last_fcm_cfg_ok = bool(fcm_cfg_ok)
                last_fcm_ctx_n = int(fcm_ctx_n)
                federation_gate_ok = federation_mqtt_ok or (not bool(require_federation_mqtt_connected))
                if start_ok and federation_gate_ok and fcm_cfg_ok and int(fcm_ctx_n) >= int(max(0, min_fcm_context_updates)):
                    return True, "ok"
        except Exception:
            pass
        time.sleep(0.15)
    return (
        False,
        "fnm_ev_preflight_timeout "
        f"start_ok={int(last_start_ok)} "
        f"federation_mqtt_ok={int(last_federation_mqtt_ok)} "
        f"federation_mqtt_required={int(bool(require_federation_mqtt_connected))} "
        f"federation_mqtt_startup_warning={int(last_federation_mqtt_warning)} "
        f"fcm_cfg_ok={int(last_fcm_cfg_ok)} "
        f"fcm_ctx_n={int(last_fcm_ctx_n)} "
        f"required_ctx_n={int(max(0, min_fcm_context_updates))} "
        f"saw_jsonl={int(saw_jsonl)} saw_stdout={int(saw_stdout)}",
    )


def _parse_token_set(raw: str) -> set[str]:
    return {x.strip() for x in str(raw or "").split(",") if x.strip()}


def _parse_int_token_set(raw: str, *, arg_name: str) -> set[int]:
    out: set[int] = set()
    for tok in str(raw or "").split(","):
        t = tok.strip()
        if not t:
            continue
        try:
            out.add(int(t))
        except Exception as e:
            raise SystemExit(f"Invalid integer token '{t}' for {arg_name}") from e
    return out


def _slug(s: str) -> str:
    out = []
    for ch in str(s or ""):
        if ch.isalnum() or ch in {"-", "_"}:
            out.append(ch)
        else:
            out.append("_")
    return "".join(out).strip("_") or "na"


def _build_topic_namespace(
    *,
    strategy: str,
    prefix: str,
    scenario_id: str,
    ev_id: str,
    density_label: str,
    route_id: str,
) -> str:
    mode = str(strategy or "scenario").strip().lower()
    pfx = _slug(str(prefix or "mx"))
    sid = _slug(str(scenario_id or "scenario"))
    if mode == "none":
        return ""
    if mode == "scenario":
        return f"{pfx}/{sid}"
    if mode == "scenario_unique":
        uniq = f"{int(time.time())}_{os.getpid()}"
        return f"{pfx}/{sid}/{uniq}"
    if mode == "semantic":
        return f"{pfx}/{_slug(density_label)}/r{_slug(route_id)}/{_slug(ev_id)}"
    raise SystemExit(f"Unsupported --mqtt-topic-namespace-strategy: {strategy}")


def _group_summary(rows: List[Dict[str, str]]) -> List[Dict[str, object]]:
    from collections import defaultdict

    g: Dict[tuple, List[float]] = defaultdict(list)
    all_g: Dict[tuple, List[Dict[str, str]]] = defaultdict(list)
    for r in rows:
        key = (
            str(r.get("density_label", "")),
            int(str(r.get("density_count", "0") or "0")),
            str(r.get("mode", "")),
        )
        all_g[key].append(r)
        try:
            tt = float(r.get("travel_time_s", ""))
        except Exception:
            continue
        g[key].append(tt)

    out: List[Dict[str, object]] = []
    for (dens, dens_count, mode), group_rows in sorted(all_g.items(), key=lambda x: (x[0][0], x[0][2])):
        vals = list(g.get((dens, dens_count, mode), []))
        vals = sorted(vals)
        n = len(vals)
        mean = (sum(vals) / max(1, n)) if vals else None
        p50 = vals[int((n - 1) * 0.50)] if n else None
        p95 = vals[int((n - 1) * 0.95)] if n else None
        out.append(
            {
                "density_label": dens,
                "density_count": dens_count,
                "mode": mode,
                "n_runs": len(group_rows),
                "n": n,
                "n_arrived": sum(1 for r in group_rows if str(r.get("arrived", "0")) == "1"),
                "n_censored": sum(1 for r in group_rows if str(r.get("ev_nonarrival_censored", "0")) == "1"),
                "travel_time_mean_s": ("" if mean is None else round(mean, 6)),
                "travel_time_p50_s": ("" if p50 is None else round(p50, 6)),
                "travel_time_p95_s": ("" if p95 is None else round(p95, 6)),
                "travel_time_min_s": ("" if not vals else round(vals[0], 6)),
                "travel_time_max_s": ("" if not vals else round(vals[-1], 6)),
            }
        )
    return out


def _fnm_script_fingerprint(path: Path) -> Dict[str, object]:
    out: Dict[str, object] = {
        "path": str(path),
        "exists": bool(path.exists()),
        "size_bytes": -1,
        "sha256_12": "",
        "has_fcm_class": False,
        "has_fcm_config_event": False,
        "has_fcm_query_event": False,
    }
    if not path.exists():
        return out
    try:
        b = path.read_bytes()
        out["size_bytes"] = int(len(b))
        out["sha256_12"] = hashlib.sha256(b).hexdigest()[:12]
        txt = b.decode("utf-8", errors="replace")
        out["has_fcm_class"] = ("class FederationContextManager" in txt)
        out["has_fcm_config_event"] = ("fcm.config.loaded" in txt)
        out["has_fcm_query_event"] = ("fcm.discovery.query" in txt)
    except Exception:
        return out
    return out


def _sha256_12_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()[:12]


def _sha256_12_text(text: str) -> str:
    return _sha256_12_bytes(str(text or "").encode("utf-8", errors="replace"))


def _file_fingerprint(path: Path) -> Dict[str, object]:
    out: Dict[str, object] = {
        "path": str(path),
        "exists": bool(path.exists()),
        "size_bytes": -1,
        "sha256_12": "",
        "mtime_epoch": None,
    }
    if not path.exists():
        return out
    try:
        st = path.stat()
        b = path.read_bytes()
        out["size_bytes"] = int(len(b))
        out["sha256_12"] = _sha256_12_bytes(b)
        out["mtime_epoch"] = float(st.st_mtime)
    except Exception:
        return out
    return out


def _compact_json(obj: object) -> str:
    return json.dumps(obj, ensure_ascii=True, sort_keys=True, separators=(",", ":"))


def _jsonl_event_summary(path: Path) -> Dict[str, object]:
    from collections import Counter

    counts: Counter[str] = Counter()
    bad_json = 0
    lines = 0
    first_ts: Dict[str, object] = {}
    last_ts: Dict[str, object] = {}
    if path.exists():
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    if not line.strip():
                        continue
                    lines += 1
                    try:
                        r = json.loads(line)
                    except Exception:
                        bad_json += 1
                        continue
                    ev = str(r.get("event") or r.get("event_type") or "-")
                    counts[ev] += 1
                    ts = r.get("ts", r.get("ts_wall_s", ""))
                    if ev not in first_ts:
                        first_ts[ev] = ts
                    last_ts[ev] = ts
        except Exception:
            pass
    return {
        "path": str(path),
        "fingerprint": _file_fingerprint(path),
        "lines": int(lines),
        "bad_json": int(bad_json),
        "event_counts_top": dict(counts.most_common(40)),
        "first_ts_top": {k: first_ts.get(k, "") for k, _ in counts.most_common(40)},
        "last_ts_top": {k: last_ts.get(k, "") for k, _ in counts.most_common(40)},
        "fnm_mqtt_connect_fail_n": int(counts.get("fnm.mqtt.connect_fail", 0)),
        "fnm_mqtt_connected_n": int(counts.get("fnm.mqtt.connected", 0)),
        "fnm_state_pull_ok_n": int(counts.get("fnm.adapter.state_pull.ok", 0)),
        "fnm_state_pull_error_n": int(counts.get("fnm.adapter.state_pull.error", 0)),
        "fnm_ev_request_publish_n": int(counts.get("fnm.adapter.ev_request.publish", 0)),
        "fnm_ev_request_drop_n": int(counts.get("fnm.adapter.ev_request.drop", 0)),
        "fcm_peer_set_update_n": int(counts.get("fcm.peer_set.update", 0)),
        "fcm_discovery_query_n": int(counts.get("fcm.discovery.query", 0)),
        "fcm_discovery_response_n": int(counts.get("fcm.discovery.response", 0)),
    }


def _sidecar_log_summary(log_dir: Path) -> Dict[str, object]:
    jsonl_paths = sorted(log_dir.glob("fnm*.jsonl")) if log_dir.exists() else []
    by_name: Dict[str, object] = {}
    totals: Dict[str, int] = {
        "fnm_mqtt_connect_fail_n": 0,
        "fnm_mqtt_connected_n": 0,
        "fnm_state_pull_ok_n": 0,
        "fnm_state_pull_error_n": 0,
        "fnm_ev_request_publish_n": 0,
        "fnm_ev_request_drop_n": 0,
        "fcm_peer_set_update_n": 0,
        "fcm_discovery_query_n": 0,
        "fcm_discovery_response_n": 0,
    }
    for p in jsonl_paths:
        s = _jsonl_event_summary(p)
        by_name[p.name] = s
        for k in list(totals.keys()):
            totals[k] += int(s.get(k, 0) or 0)
    return {
        "log_dir": str(log_dir),
        "jsonl_count": len(jsonl_paths),
        "totals": totals,
        "by_name": by_name,
        "runner_stdout": _file_fingerprint(log_dir / "run_fnm_sidecars.stdout.log"),
    }


def _start_sidecars(
    *,
    python_bin: str,
    runner_script: Path,
    fnm_script: Path,
    ev_config: Path,
    inter_cfg_dir: Path,
    log_dir: Path,
    tick_sec: float,
    stagger_sec: float,
    auto_generate: bool,
    generator_template: Path,
    generator_route_file: Path,
    generator_ev_id: str,
    generator_net_file: Path,
    generator_clean_out_dir: bool,
    generator_f2d_contextual_subscriptions_enable: bool,
    generator_f2d_contextual_back_nodes: int,
    generator_f2d_contextual_forward_nodes: int,
    generator_f2d_contextual_back_edges: int,
    generator_f2d_contextual_forward_edges: int,
    data_base_dir: str,
    data_run_id: str,
    data_persist_raw_messages: str,
    topic_namespace: str,
    mqtt_host: str,
    mqtt_port: int,
    sidecars_runner_ev_preflight_enable: bool,
    sidecars_runner_ev_preflight_timeout_sec: float,
    sidecars_runner_ev_preflight_min_state_ok: int,
    sidecars_runner_ev_preflight_min_req_published: int,
    dry_run: bool,
) -> subprocess.Popen | None:
    cmd: List[str] = [
        str(python_bin),
        str(runner_script),
        "--python-bin",
        str(python_bin),
        "--fnm-script",
        str(fnm_script),
        "--ev-config",
        str(ev_config),
        "--intersection-config-dir",
        str(inter_cfg_dir),
        "--tick-sec",
        str(float(tick_sec)),
        "--stagger-sec",
        str(float(stagger_sec)),
        "--log-dir",
        str(log_dir),
    ]

    if str(data_base_dir).strip():
        cmd += ["--data-base-dir", str(data_base_dir)]
    if str(data_run_id).strip():
        cmd += ["--data-run-id", str(data_run_id)]
    if str(data_persist_raw_messages).strip() in {"on", "off"}:
        cmd += ["--data-persist-raw-messages", str(data_persist_raw_messages).strip()]
    if str(topic_namespace).strip():
        cmd += ["--topic-namespace", str(topic_namespace).strip()]
    if str(mqtt_host or "").strip():
        cmd += ["--mqtt-host", str(mqtt_host).strip()]
    if int(mqtt_port or 0) > 0:
        cmd += ["--mqtt-port", str(int(mqtt_port))]
    if bool(sidecars_runner_ev_preflight_enable):
        cmd += [
            "--ev-preflight-enable",
            "--ev-preflight-timeout-sec",
            str(float(sidecars_runner_ev_preflight_timeout_sec)),
            "--ev-preflight-min-state-ok",
            str(int(sidecars_runner_ev_preflight_min_state_ok)),
            "--ev-preflight-min-req-published",
            str(int(sidecars_runner_ev_preflight_min_req_published)),
        ]

    if auto_generate:
        cmd += [
            "--auto-generate-intersection-configs",
            "--generator-template",
            str(generator_template),
            "--generator-route-file",
            str(generator_route_file),
            "--generator-ev-id",
            str(generator_ev_id),
            "--generator-net-file",
            str(generator_net_file),
        ]
        if generator_clean_out_dir:
            cmd.append("--generator-clean-out-dir")
        if bool(generator_f2d_contextual_subscriptions_enable):
            cmd += [
                "--generator-f2d-contextual-subscriptions-enable",
                "--generator-f2d-contextual-back-nodes",
                str(int(generator_f2d_contextual_back_nodes)),
                "--generator-f2d-contextual-forward-nodes",
                str(int(generator_f2d_contextual_forward_nodes)),
                "--generator-f2d-contextual-back-edges",
                str(int(generator_f2d_contextual_back_edges)),
                "--generator-f2d-contextual-forward-edges",
                str(int(generator_f2d_contextual_forward_edges)),
            ]

    if dry_run:
        print("DRY_RUN sidecars:", " ".join(shlex.quote(x) for x in cmd))
        return None

    log_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = log_dir / "run_fnm_sidecars.stdout.log"
    fp = stdout_path.open("a", encoding="utf-8")
    print("START sidecars:", " ".join(shlex.quote(x) for x in cmd))
    start_epoch = time.time()
    proc = subprocess.Popen(
        cmd,
        stdout=fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        start_context = {
            "schema": "fnm_sidecar_start_context.v1",
            "start_epoch": float(start_epoch),
            "pid": int(proc.pid),
            "command": {
                "argv": list(cmd),
                "shell_escaped": " ".join(shlex.quote(x) for x in cmd),
                "argv_sha256_12": _sha256_12_text(_compact_json(list(cmd))),
            },
            "inputs": {
                "runner_script": _file_fingerprint(runner_script),
                "fnm_script": _file_fingerprint(fnm_script),
                "ev_config": _file_fingerprint(ev_config),
                "intersection_config_dir": str(inter_cfg_dir),
                "generator_route_file": _file_fingerprint(generator_route_file),
                "generator_template": _file_fingerprint(generator_template),
                "generator_net_file": _file_fingerprint(generator_net_file),
            },
            "runtime": {
                "topic_namespace": str(topic_namespace or ""),
                "mqtt_host": str(mqtt_host or ""),
                "mqtt_port": int(mqtt_port or 0),
                "data_base_dir": str(data_base_dir or ""),
                "data_run_id": str(data_run_id or ""),
                "data_persist_raw_messages": str(data_persist_raw_messages or ""),
                "tick_sec": float(tick_sec),
                "stagger_sec": float(stagger_sec),
                "auto_generate": bool(auto_generate),
                "generator_clean_out_dir": bool(generator_clean_out_dir),
                "generator_f2d_contextual_subscriptions_enable": bool(generator_f2d_contextual_subscriptions_enable),
                "generator_f2d_contextual_back_nodes": int(generator_f2d_contextual_back_nodes),
                "generator_f2d_contextual_forward_nodes": int(generator_f2d_contextual_forward_nodes),
                "generator_f2d_contextual_back_edges": int(generator_f2d_contextual_back_edges),
                "generator_f2d_contextual_forward_edges": int(generator_f2d_contextual_forward_edges),
            },
        }
        (log_dir / "sidecar_start_context.json").write_text(
            json.dumps(start_context, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except Exception:
        pass
    return proc


def _stop_sidecars(proc: subprocess.Popen | None, timeout_sec: float = 8.0) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            return
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _tail_text(path: Path, max_lines: int = 80) -> str:
    try:
        if not path.exists():
            return f"<missing: {path}>"
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        return "\n".join(lines[-max(1, int(max_lines)):])
    except Exception as e:
        return f"<failed to read {path}: {type(e).__name__}:{e}>"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run EV matrix with per-scenario FNM sidecars.")

    # Base matrix launcher inputs
    ap.add_argument("--manifest-csv", required=True)
    ap.add_argument("--sim-root", required=True)
    ap.add_argument("--python-bin", default=sys.executable or "python3")
    ap.add_argument("--launch-script", default="")
    ap.add_argument("--real-world-script", required=True)
    ap.add_argument("--base-sumocfg", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument(
        "--modes",
        default="B0,B1,F2",
        help=(
            "comma-separated experiment modes forwarded to launch_ev_matrix_experiments.py. "
            "Use F2P for F2+passive non-TLS DTs, F2D for F2+Drone-DT downstream context, "
            "F2P-Q for F2P plus experimental passive-triggered queue release, "
            "F2D-Q for F2D plus experimental drone-triggered queue release, "
            "or F2PD for passive+drone as separate plotted modes."
        ),
    )
    ap.add_argument("--realworld-common-args", default="")
    ap.add_argument("--realworld-common-args-file", default="")
    ap.add_argument("--sumo-extra-base", default="")
    ap.add_argument(
        "--max-sim-time-sec",
        type=float,
        default=0.0,
        help=(
            "forwarded to launch_ev_matrix_experiments.py/real-world.py as a SUMO simulation-time hard stop; "
            "0 disables. Use for 2.5K route screening to avoid unbounded non-arrival logs."
        ),
    )
    ap.add_argument(
        "--terminate-on-ev-finish",
        action="store_true",
        default=False,
        help=(
            "forwarded to launch_ev_matrix_experiments.py/real-world.py; "
            "successful runs stop when the emergency vehicle arrives while max-sim-time still caps non-arrivals"
        ),
    )
    ap.add_argument(
        "--realtime-sumo-enable",
        action="store_true",
        default=False,
        help="forwarded to real-world.py; pace SUMO simulation time to wall-clock time for physical drone F2D runs",
    )
    ap.add_argument("--realtime-sumo-factor", type=float, default=1.0)
    ap.add_argument("--realtime-sumo-max-sleep-sec", type=float, default=0.5)
    ap.add_argument("--realtime-sumo-log-period-sec", type=float, default=5.0)
    ap.add_argument(
        "--realtime-sumo-start-sim-time-sec",
        type=float,
        default=0.0,
        help=(
            "forwarded to real-world.py for realtime-enabled modes; "
            "simulation time at which wall-clock pacing starts after a fast pre-roll"
        ),
    )
    ap.add_argument(
        "--realtime-sumo-modes",
        default="",
        help=(
            "comma-separated modes that should receive real-time SUMO pacing when "
            "--realtime-sumo-enable is set. Empty means all modes for backward compatibility. "
            "Use F2D for hybrid physical drone runs while keeping F2 fast."
        ),
    )
    ap.add_argument("--passive-intersection-dt-enable", action="store_true", default=False)
    ap.add_argument("--passive-intersection-context-period-sec", type=float, default=0.0)
    ap.add_argument("--passive-intersection-max-nodes", type=int, default=-1)
    ap.add_argument("--passive-intersection-lookahead-edges", type=int, default=0)
    ap.add_argument(
        "--passive-intersection-context-route-fanout-enable",
        dest="passive_intersection_context_route_fanout_enable",
        action="store_true",
        default=True,
    )
    ap.add_argument(
        "--no-passive-intersection-context-route-fanout-enable",
        dest="passive_intersection_context_route_fanout_enable",
        action="store_false",
    )
    ap.add_argument("--passive-intersection-context-fanout-back-edges", type=int, default=0)
    ap.add_argument("--passive-intersection-context-fanout-forward-edges", type=int, default=0)
    ap.add_argument("--ev-intersection-discovery-enable", action="store_true", default=False)
    ap.add_argument("--ev-intersection-discovery-delay-sec", type=float, default=0.0)
    ap.add_argument("--ev-intersection-discovery-modes", default="")
    ap.add_argument(
        "--ev-intersection-discovery-repeat-scope",
        choices=["tls", "edge"],
        default="",
    )
    ap.add_argument("--ev-intersection-discovery-wait-log-period-sec", type=float, default=0.0)
    ap.add_argument(
        "--f2p-passive-context-policy",
        choices=[
            "disabled",
            "missing_feedback_only",
            "severe_or_missing",
            "immediate_missing_severe",
            "always",
        ],
        default="",
    )
    ap.add_argument("--f2p-passive-context-max-age-sec", type=float, default=0.0)
    ap.add_argument("--f2p-passive-context-lookahead-edges", type=int, default=0)
    ap.add_argument("--f2p-passive-context-max-worst-edge-offset", type=int, default=0)
    ap.add_argument("--f2p-passive-context-severe-min-halt-n", type=int, default=0)
    ap.add_argument("--f2p-passive-context-severe-min-veh-n", type=int, default=0)
    ap.add_argument("--f2p-passive-context-severe-max-mean-speed-mps", type=float, default=0.0)
    ap.add_argument("--f2p-passive-context-severe-max-occupancy-pct", type=float, default=0.0)
    ap.add_argument(
        "--f2p-passive-context-missing-feedback-floor-enable",
        dest="f2p_passive_context_missing_feedback_floor_enable",
        action="store_true",
        default=True,
    )
    ap.add_argument(
        "--no-f2p-passive-context-missing-feedback-floor-enable",
        dest="f2p_passive_context_missing_feedback_floor_enable",
        action="store_false",
    )
    ap.add_argument("--f2p-passive-context-missing-feedback-max-queue-deficit-sec", type=float, default=0.0)
    ap.add_argument("--f2p-passive-context-missing-feedback-max-spillback-risk", type=float, default=0.0)
    ap.add_argument("--f2p-passive-context-missing-feedback-max-timing-sec", type=float, default=0.0)
    ap.add_argument(
        "--f2p-passive-context-clear-missing-feedback-enable",
        dest="f2p_passive_context_clear_missing_feedback_enable",
        action="store_true",
        default=True,
    )
    ap.add_argument(
        "--no-f2p-passive-context-clear-missing-feedback-enable",
        dest="f2p_passive_context_clear_missing_feedback_enable",
        action="store_false",
    )
    ap.add_argument("--f2p-passive-context-clear-missing-feedback-no-feedback-penalty", type=float, default=0.0)
    ap.add_argument("--f2p-queue-release-enable", action="store_true", default=False)
    ap.add_argument("--f2p-queue-release-hold-sec", type=float, default=0.0)
    ap.add_argument("--f2p-queue-release-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2p-queue-release-max-worst-edge-offset", type=int, default=0)
    ap.add_argument("--external-downstream-context-enable", action="store_true", default=False)
    ap.add_argument("--external-downstream-context-max-age-sec", type=float, default=0.0)
    ap.add_argument("--f2-drone-context-request-enable", action="store_true", default=False)
    ap.add_argument("--f2-drone-context-provider-id", default="")
    ap.add_argument("--f2-drone-context-request-ttl-sec", type=float, default=0.0)
    ap.add_argument("--f2-drone-context-request-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2-drone-context-request-max-edges", type=int, default=0)
    ap.add_argument(
        "--f2-drone-context-include-route-context",
        dest="f2_drone_context_include_route_context",
        action="store_true",
        default=True,
    )
    ap.add_argument(
        "--no-f2-drone-context-include-route-context",
        dest="f2_drone_context_include_route_context",
        action="store_false",
    )
    ap.add_argument("--f2-drone-context-route-context-max-edges", type=int, default=0)
    ap.add_argument("--no-f2-drone-context-emit-discovery-query", action="store_true", default=False)
    ap.add_argument("--f2-drone-context-discovery-gate-enable", action="store_true", default=False)
    ap.add_argument("--f2-drone-context-discovery-cache-ttl-sec", type=float, default=0.0)
    ap.add_argument("--f2-drone-context-discovery-query-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2d-queue-release-enable", action="store_true", default=False)
    ap.add_argument("--f2d-queue-release-hold-sec", type=float, default=0.0)
    ap.add_argument("--f2d-queue-release-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2d-queue-release-max-worst-edge-offset", type=int, default=0)
    ap.add_argument("--f2d-drone-prescout-enable", action="store_true", default=False)
    ap.add_argument("--no-f2d-drone-prescout-first-tls-only", action="store_true", default=False)
    ap.add_argument("--f2d-drone-prescout-max-edges", type=int, default=0)
    ap.add_argument("--f2d-drone-prescout-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2d-contextual-topic-delivery-enable", action="store_true", default=False)
    ap.add_argument("--no-f2d-directed-context-delivery-enable", action="store_true", default=False)
    ap.add_argument("--f2d-directed-context-self-delivery-enable", action="store_true", default=False)
    ap.add_argument(
        "--fed-debug-log-mode",
        choices=["per-run", "common-if-set", "common-only"],
        default="per-run",
        help=(
            "Forwarded to launch_ev_matrix_experiments.py. "
            "Control whether per-run fed_outcomes.txt is forced or common args fed log path is respected."
        ),
    )
    ap.add_argument(
        "--mqtt-host",
        default="",
        help="optional MQTT broker host override for real-world and FNM sidecars",
    )
    ap.add_argument(
        "--mqtt-port",
        type=int,
        default=0,
        help="optional MQTT broker port override for real-world and FNM sidecars",
    )
    ap.add_argument("--skip-existing", action="store_true", default=False)

    # FNM sidecar controls
    ap.add_argument("--no-sidecars", action="store_true", default=False)
    ap.add_argument(
        "--sidecars-isolation",
        choices=["scenario", "mode"],
        default="scenario",
        help=(
            "scenario=start one FNM sidecar set per scenario and run all modes; "
            "mode=restart FNM sidecars for each mode with a mode-specific namespace/log dir"
        ),
    )
    ap.add_argument("--sidecars-runner-script", default="")
    ap.add_argument("--fnm-script", default="")
    ap.add_argument(
        "--fnm-require-fcm-markers",
        action="store_true",
        default=False,
        help=(
            "fail fast when resolved --fnm-script does not appear to include FCM markers "
            "(FederationContextManager + fcm events)."
        ),
    )
    ap.add_argument(
        "--fnm-ev-config-pattern",
        default="config/fnm/batch/{ev_id}/fnm_ev_{ev_id}.yml",
        help="pattern relative to sim-root",
    )
    ap.add_argument(
        "--fnm-sync-ev-batch-configs",
        action="store_true",
        default=False,
        help="before scenario runs, refresh batch EV FNM configs from a single template source",
    )
    ap.add_argument(
        "--fnm-ev-template-source",
        default="config/fnm/fnm_ev_emergency1.yml",
        help="single EV template source used when --fnm-sync-ev-batch-configs is enabled",
    )
    ap.add_argument(
        "--fnm-intersection-config-dir-pattern",
        default="config/fnm/batch/{ev_id}/intersections",
        help="pattern relative to sim-root",
    )
    ap.add_argument("--fnm-auto-generate-intersection-configs", action="store_true", default=True)
    ap.add_argument("--no-fnm-auto-generate-intersection-configs", dest="fnm_auto_generate_intersection_configs", action="store_false")
    ap.add_argument("--fnm-generator-template", default="config/fnm/fnm_intersection_node400.yml")
    ap.add_argument("--fnm-generator-clean-out-dir", action="store_true", default=False)
    ap.add_argument(
        "--fnm-generator-f2d-contextual-subscriptions-enable",
        action="store_true",
        default=False,
        help=(
            "F2D-only experiments: generate active SI-DT FNM configs with route-relevant "
            "node/edge downstream-context subscriptions for Drone-DT observations"
        ),
    )
    ap.add_argument("--fnm-generator-f2d-contextual-back-nodes", type=int, default=2)
    ap.add_argument("--fnm-generator-f2d-contextual-forward-nodes", type=int, default=6)
    ap.add_argument("--fnm-generator-f2d-contextual-back-edges", type=int, default=2)
    ap.add_argument("--fnm-generator-f2d-contextual-forward-edges", type=int, default=8)
    ap.add_argument("--fnm-tick-sec", type=float, default=0.1)
    ap.add_argument("--fnm-stagger-sec", type=float, default=0.15)
    ap.add_argument("--sidecars-ready-wait-sec", type=float, default=1.5)
    ap.add_argument(
        "--sidecars-preflight-timeout-sec",
        type=float,
        default=4.0,
        help="seconds to wait for EV sidecar fnm.start marker in fnm_ev.jsonl",
    )
    ap.add_argument(
        "--sidecars-preflight-require-fcm-config-loaded",
        dest="sidecars_preflight_require_fcm_config_loaded",
        action="store_true",
        help="require fcm.config.loaded marker in EV sidecar preflight",
    )
    ap.add_argument(
        "--no-sidecars-preflight-require-fcm-config-loaded",
        dest="sidecars_preflight_require_fcm_config_loaded",
        action="store_false",
        help="do not require fcm.config.loaded marker in EV sidecar preflight",
    )
    ap.set_defaults(sidecars_preflight_require_fcm_config_loaded=True)
    ap.add_argument(
        "--sidecars-preflight-min-fcm-context-updates",
        type=int,
        default=0,
        help="minimum number of fcm.query_context.update events required in EV sidecar preflight",
    )
    ap.add_argument(
        "--no-sidecars-preflight",
        dest="sidecars_preflight_enable",
        action="store_false",
        help="disable EV sidecar startup preflight checks",
    )
    ap.set_defaults(sidecars_preflight_enable=True)
    ap.add_argument(
        "--sidecars-runner-ev-preflight-enable",
        dest="sidecars_runner_ev_preflight_enable",
        action="store_true",
        help="enable run_fnm_sidecars internal EV preflight (state-pull readiness) before launching all sidecars",
    )
    ap.add_argument(
        "--no-sidecars-runner-ev-preflight-enable",
        dest="sidecars_runner_ev_preflight_enable",
        action="store_false",
        help="disable run_fnm_sidecars internal EV preflight",
    )
    ap.set_defaults(sidecars_runner_ev_preflight_enable=False)
    ap.add_argument("--sidecars-runner-ev-preflight-timeout-sec", type=float, default=30.0)
    ap.add_argument("--sidecars-runner-ev-preflight-min-state-ok", type=int, default=1)
    ap.add_argument("--sidecars-runner-ev-preflight-min-req-published", type=int, default=0)
    ap.add_argument(
        "--fail-on-foreign-ev-drop",
        action="store_true",
        default=False,
        help="propagate to launcher: fail run when drop_foreign_ev_id exceeds threshold",
    )
    ap.add_argument(
        "--foreign-ev-drop-fail-threshold",
        type=int,
        default=0,
        help="threshold passed to launcher for foreign EV drop fail gate",
    )
    ap.add_argument("--fnm-data-base-dir", default="")
    ap.add_argument("--fnm-data-run-id-prefix", default="")
    ap.add_argument("--fnm-data-persist-raw-messages", choices=["auto", "on", "off"], default="auto")
    ap.add_argument(
        "--ev-http-port-strategy",
        choices=["none", "fixed", "incremental", "free"],
        default="free",
        help="how to assign real-world EV HTTP state server port per scenario",
    )
    ap.add_argument("--ev-http-port-host", default="127.0.0.1")
    ap.add_argument("--ev-http-port-base", type=int, default=18083)
    ap.add_argument("--ev-http-port-step", type=int, default=1)
    ap.add_argument(
        "--mqtt-topic-namespace-strategy",
        choices=["none", "scenario", "scenario_unique", "semantic"],
        default="scenario_unique",
        help="topic namespace strategy for full run isolation across concurrent runs",
    )
    ap.add_argument(
        "--mqtt-topic-namespace-prefix",
        default="mx",
        help="prefix used when building topic namespaces",
    )

    ap.add_argument("--scenario-filter", default="", help="comma-separated scenario_id filter")
    ap.add_argument("--ev-filter", default="", help="comma-separated EV ids (e.g., emergency1,emergency2)")
    ap.add_argument("--route-filter", default="", help="comma-separated route ids (e.g., 1,2,6)")
    ap.add_argument("--density-filter", default="", help="comma-separated density labels (e.g., smooth,moderate)")
    ap.add_argument("--density-count-filter", default="", help="comma-separated density counts (e.g., 200,500)")
    ap.add_argument("--max-scenarios", type=int, default=0, help="limit number of scenarios after filters (0=all)")
    ap.add_argument("--list-scenarios", action="store_true", default=False, help="print filtered scenarios and exit")
    ap.add_argument("--dry-run", action="store_true", default=False)
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    sim_root = Path(args.sim_root).resolve()
    manifest_csv = Path(args.manifest_csv).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    launch_script = Path(args.launch_script).resolve() if str(args.launch_script).strip() else (sim_root / "launch_ev_matrix_experiments.py")
    sidecars_runner = Path(args.sidecars_runner_script).resolve() if str(args.sidecars_runner_script).strip() else (sim_root / "run_fnm_sidecars.py")
    fnm_script = Path(args.fnm_script).resolve() if str(args.fnm_script).strip() else (sim_root / "federation_node_manager.py")
    fnm_fp = _fnm_script_fingerprint(fnm_script)
    print(
        "[matrix+fnm] fnm_script "
        f"path={fnm_fp.get('path')} exists={int(bool(fnm_fp.get('exists')))} "
        f"size={fnm_fp.get('size_bytes')} sha256_12={fnm_fp.get('sha256_12') or '-'} "
        f"has_fcm_class={int(bool(fnm_fp.get('has_fcm_class')))} "
        f"has_fcm_config_event={int(bool(fnm_fp.get('has_fcm_config_event')))} "
        f"has_fcm_query_event={int(bool(fnm_fp.get('has_fcm_query_event')))}"
    )
    if bool(args.fnm_require_fcm_markers):
        ok_fcm = bool(fnm_fp.get("has_fcm_class")) and bool(fnm_fp.get("has_fcm_config_event")) and bool(
            fnm_fp.get("has_fcm_query_event")
        )
        if not ok_fcm:
            raise SystemExit(
                "[matrix+fnm] fnm_script appears not FCM-capable. "
                "Pass --fnm-script pointing to the expected connector/federation_node_manager.py "
                "or disable --fnm-require-fcm-markers."
            )

    rows = _read_manifest(manifest_csv)
    if not rows:
        raise SystemExit("Manifest is empty")

    scenario_filter = _parse_token_set(str(args.scenario_filter or ""))
    ev_filter = _parse_token_set(str(args.ev_filter or ""))
    route_filter = _parse_int_token_set(str(args.route_filter or ""), arg_name="--route-filter")
    density_filter = _parse_token_set(str(args.density_filter or ""))
    density_count_filter = _parse_int_token_set(str(args.density_count_filter or ""), arg_name="--density-count-filter")

    if scenario_filter:
        rows = [r for r in rows if str(r.get("scenario_id", "")) in scenario_filter]
    if ev_filter:
        rows = [r for r in rows if str(r.get("ev_id", "")) in ev_filter]
    if route_filter:
        rows = [r for r in rows if int(str(r.get("route_id", "0") or "0")) in route_filter]
    if density_filter:
        rows = [r for r in rows if str(r.get("density_label", "")) in density_filter]
    if density_count_filter:
        rows = [r for r in rows if int(str(r.get("density_count", "0") or "0")) in density_count_filter]
    if int(args.max_scenarios or 0) > 0:
        rows = rows[: int(args.max_scenarios)]

    if not rows:
        raise SystemExit("No scenarios remaining after applying filters")

    if args.list_scenarios:
        print("scenario_id,density_label,density_count,route_id,ev_id")
        for r in rows:
            print(
                f"{str(r.get('scenario_id',''))},{str(r.get('density_label',''))},{str(r.get('density_count',''))},{str(r.get('route_id',''))},{str(r.get('ev_id',''))}"
            )
        return

    total_scenarios = len(rows)
    matrix_t0 = time.time()
    requested_modes = [m.strip().upper() for m in str(args.modes or "").split(",") if m.strip()]
    if not requested_modes:
        raise SystemExit("No modes provided")

    print(
        f"[matrix+fnm] scenarios={total_scenarios} modes={','.join(requested_modes)} "
        f"sidecars={'off' if args.no_sidecars else 'on'} isolation={args.sidecars_isolation}"
    )
    print(
        f"[matrix+fnm] filters scenario={sorted(scenario_filter) if scenario_filter else 'ALL'} ev={sorted(ev_filter) if ev_filter else 'ALL'} route={sorted(route_filter) if route_filter else 'ALL'} density={sorted(density_filter) if density_filter else 'ALL'} density_count={sorted(density_count_filter) if density_count_filter else 'ALL'} max={int(args.max_scenarios or 0) or 'ALL'}"
    )

    if bool(args.fnm_sync_ev_batch_configs):
        ev_ids_for_sync = sorted(
            {
                str(r.get("ev_id", "") or "").strip()
                for r in rows
                if str(r.get("ev_id", "") or "").strip()
            }
        )
        ev_template_src = (sim_root / str(args.fnm_ev_template_source)).resolve()
        print(
            f"[matrix+fnm] sync_ev_batch_configs enabled ev_count={len(ev_ids_for_sync)} src={ev_template_src}"
        )
        _sync_ev_batch_configs_from_template(
            sim_root=sim_root,
            template_path=ev_template_src,
            ev_ids=ev_ids_for_sync,
            ev_cfg_pattern=str(args.fnm_ev_config_pattern),
            dry_run=bool(args.dry_run),
        )

    all_results: List[Dict[str, str]] = []

    for scenario_idx, row in enumerate(rows, start=1):
        scenario_id = str(row.get("scenario_id", "scenario"))
        ev_id = str(row.get("ev_id", "emergency1"))
        route_file = Path(str(row.get("route_file", ""))).resolve()
        if not route_file.exists() and not args.dry_run:
            raise SystemExit(f"Missing route_file: {route_file}")

        density_label = str(row.get("density_label", ""))
        density_count = str(row.get("density_count", ""))
        route_id = str(row.get("route_id", ""))
        scenario_t0 = time.time()
        topic_namespace = _build_topic_namespace(
            strategy=str(args.mqtt_topic_namespace_strategy),
            prefix=str(args.mqtt_topic_namespace_prefix),
            scenario_id=scenario_id,
            ev_id=ev_id,
            density_label=density_label,
            route_id=route_id,
        )

        print(
            f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}] id={scenario_id} density={density_label}({density_count}) route={route_id} ev={ev_id}"
        )

        scenario_dir = out_dir / "scenario_runs" / scenario_id
        scenario_dir.mkdir(parents=True, exist_ok=True)
        scenario_manifest = scenario_dir / "manifest_one.csv"
        _write_manifest(scenario_manifest, [row])

        ev_cfg = (sim_root / str(args.fnm_ev_config_pattern.format(ev_id=ev_id))).resolve()
        inter_cfg_dir = (sim_root / str(args.fnm_intersection_config_dir_pattern.format(ev_id=ev_id))).resolve()
        gen_template = (sim_root / str(args.fnm_generator_template)).resolve()
        selected_http_port = _select_http_port(
            strategy=str(args.ev_http_port_strategy),
            host=str(args.ev_http_port_host),
            base=int(args.ev_http_port_base),
            step=int(args.ev_http_port_step),
            scenario_idx=scenario_idx,
        )
        ev_cfg_runtime = ev_cfg

        run_id = ""
        if str(args.fnm_data_run_id_prefix).strip():
            run_id = f"{str(args.fnm_data_run_id_prefix).strip()}_{scenario_id}"

        def _run_matrix_unit(
            *,
            unit_modes: List[str],
            unit_label: str,
            unit_idx: int,
            unit_total: int,
        ) -> None:
            nonlocal ev_cfg_runtime, all_results
            unit_modes_csv = ",".join(unit_modes)
            unit_suffix = "" if str(unit_label).lower() == "scenario" else f"_{str(unit_label).lower()}"
            unit_topic_namespace = str(topic_namespace or "")
            if str(args.sidecars_isolation) == "mode" and unit_label.upper() in requested_modes:
                unit_topic_namespace = f"{unit_topic_namespace}/{unit_label.upper()}" if unit_topic_namespace else unit_label.upper()
            unit_run_id = run_id
            if str(args.sidecars_isolation) == "mode" and unit_label.upper() in requested_modes and unit_run_id:
                unit_run_id = f"{unit_run_id}_{unit_label.upper()}"
            realtime_modes = {
                str(x).strip().upper()
                for x in str(args.realtime_sumo_modes or "").split(",")
                if str(x).strip()
            }
            realtime_enabled_for_unit = bool(args.realtime_sumo_enable) and (
                not realtime_modes
                or any(str(m).strip().upper() in realtime_modes for m in unit_modes)
            )

            sidecar_proc: subprocess.Popen | None = None
            unit_start_epoch = time.time()
            sidecar_log_dir = scenario_dir / f"fnm_sidecars{unit_suffix}"
            launch_out = scenario_dir / f"matrix_out{unit_suffix}"
            unit_context_path = scenario_dir / f"unit_context{unit_suffix or '_scenario'}.json"
            launch_cmd: List[str] = []
            sidecar_ready_epoch: float | None = None
            launch_start_epoch: float | None = None
            launch_end_epoch: float | None = None
            collected_rows_n = 0
            try:
                if not args.no_sidecars:
                    if selected_http_port is not None:
                        ev_cfg_runtime = sidecar_log_dir / f"fnm_ev_{ev_id}.runtime.yml"
                        if not args.dry_run:
                            _rewrite_ev_cfg_http_port(ev_cfg, ev_cfg_runtime, int(selected_http_port))
                            ok_cfg, reason_cfg = _validate_ev_runtime_cfg(ev_cfg_runtime, int(selected_http_port))
                            if not ok_cfg:
                                raise SystemExit(
                                    f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                                    f"invalid_runtime_ev_cfg ev={ev_id} reason={reason_cfg} cfg={ev_cfg_runtime}"
                                )
                        else:
                            print(
                                f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                                f"DRY_RUN patch_ev_cfg_http_port ev={ev_id} port={selected_http_port} src={ev_cfg} dst={ev_cfg_runtime}"
                            )
                    print(
                        f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                        f"starting_sidecars mode={unit_label} ev={ev_id} log_dir={sidecar_log_dir} "
                        f"ev_http_port={selected_http_port if selected_http_port is not None else 'UNCHANGED'} "
                        f"topic_ns={unit_topic_namespace or '-'}"
                    )
                    unit_is_f2d = str(unit_label or "").upper() == "F2D"
                    sidecar_proc = _start_sidecars(
                        python_bin=str(args.python_bin),
                        runner_script=sidecars_runner,
                        fnm_script=fnm_script,
                        ev_config=ev_cfg_runtime,
                        inter_cfg_dir=inter_cfg_dir,
                        log_dir=sidecar_log_dir,
                        tick_sec=float(args.fnm_tick_sec),
                        stagger_sec=float(args.fnm_stagger_sec),
                        auto_generate=bool(args.fnm_auto_generate_intersection_configs),
                        generator_template=gen_template,
                        generator_route_file=route_file,
                        generator_ev_id=ev_id,
                        generator_net_file=Path(args.net_file).resolve(),
                        generator_clean_out_dir=bool(args.fnm_generator_clean_out_dir),
                        generator_f2d_contextual_subscriptions_enable=bool(
                            args.fnm_generator_f2d_contextual_subscriptions_enable and unit_is_f2d
                        ),
                        generator_f2d_contextual_back_nodes=int(args.fnm_generator_f2d_contextual_back_nodes),
                        generator_f2d_contextual_forward_nodes=int(args.fnm_generator_f2d_contextual_forward_nodes),
                        generator_f2d_contextual_back_edges=int(args.fnm_generator_f2d_contextual_back_edges),
                        generator_f2d_contextual_forward_edges=int(args.fnm_generator_f2d_contextual_forward_edges),
                        data_base_dir=str(args.fnm_data_base_dir),
                        data_run_id=unit_run_id,
                        data_persist_raw_messages=str(args.fnm_data_persist_raw_messages),
                        topic_namespace=str(unit_topic_namespace),
                        mqtt_host=str(args.mqtt_host or ""),
                        mqtt_port=int(args.mqtt_port or 0),
                        sidecars_runner_ev_preflight_enable=bool(args.sidecars_runner_ev_preflight_enable),
                        sidecars_runner_ev_preflight_timeout_sec=float(args.sidecars_runner_ev_preflight_timeout_sec),
                        sidecars_runner_ev_preflight_min_state_ok=int(args.sidecars_runner_ev_preflight_min_state_ok),
                        sidecars_runner_ev_preflight_min_req_published=int(args.sidecars_runner_ev_preflight_min_req_published),
                        dry_run=bool(args.dry_run),
                    )
                    if not args.dry_run:
                        wait_s = max(0.0, float(args.sidecars_ready_wait_sec))
                        time.sleep(wait_s)
                        if sidecar_proc is not None and sidecar_proc.poll() is not None:
                            sidecar_stdout_log = sidecar_log_dir / "run_fnm_sidecars.stdout.log"
                            log_tail = _tail_text(sidecar_stdout_log, max_lines=100)
                            raise SystemExit(
                                f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                                f"sidecars_runner_exited_early rc={int(sidecar_proc.returncode or 0)} "
                                f"log={sidecar_stdout_log}\n"
                                f"--- sidecar log tail ---\n{log_tail}\n"
                                f"--- end sidecar log tail ---"
                            )
                        if bool(args.sidecars_preflight_enable):
                            # If we launch FNM sidecars, require their federation
                            # MQTT path to be healthy before running SUMO. B0 does
                            # not consume EV requests, but allowing B0 to proceed
                            # with disconnected sidecars hides substrate failures
                            # for several minutes and floods the logs.
                            require_fed_mqtt = True
                            ok_pf, reason_pf = _ev_sidecar_preflight(
                                sidecar_log_dir,
                                timeout_sec=float(args.sidecars_preflight_timeout_sec),
                                require_fcm_config_loaded=bool(args.sidecars_preflight_require_fcm_config_loaded),
                                min_fcm_context_updates=int(args.sidecars_preflight_min_fcm_context_updates),
                                require_federation_mqtt_connected=bool(require_fed_mqtt),
                            )
                            if not ok_pf:
                                raise SystemExit(
                                    f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                                    f"sidecars_preflight_failed ev={ev_id} reason={reason_pf} log_dir={sidecar_log_dir}"
                                )
                        print(
                            f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                            f"sidecars_ready wait_s={wait_s:.2f}"
                        )
                        sidecar_ready_epoch = time.time()

                cmd: List[str] = [
                    str(args.python_bin),
                    str(launch_script),
                    "--manifest-csv",
                    str(scenario_manifest),
                    "--sim-root",
                    str(sim_root),
                    "--python-bin",
                    str(args.python_bin),
                    "--real-world-script",
                    str(Path(args.real_world_script).resolve()),
                    "--base-sumocfg",
                    str(Path(args.base_sumocfg).resolve()),
                    "--net-file",
                    str(Path(args.net_file).resolve()),
                    "--out-dir",
                    str(launch_out),
                    "--modes",
                    unit_modes_csv,
                ]
                if str(args.realworld_common_args_file).strip():
                    cmd += ["--realworld-common-args-file", str(Path(args.realworld_common_args_file).resolve())]
                merged_rw_args = str(args.realworld_common_args or "").strip()
                if selected_http_port is not None:
                    override = (
                        f"--ev-http-state-server-host {shlex.quote(str(args.ev_http_port_host).strip())} "
                        f"--ev-http-state-server-port {int(selected_http_port)}"
                    )
                    merged_rw_args = f"{merged_rw_args} {override}".strip()
                if str(unit_topic_namespace).strip():
                    override_ns = f"--mqtt-topic-namespace {str(unit_topic_namespace).strip()}"
                    merged_rw_args = f"{merged_rw_args} {override_ns}".strip()
                if str(args.mqtt_host or "").strip():
                    merged_rw_args = f"{merged_rw_args} --mqtt-host {shlex.quote(str(args.mqtt_host).strip())}".strip()
                if int(args.mqtt_port or 0) > 0:
                    merged_rw_args = f"{merged_rw_args} --mqtt-port {int(args.mqtt_port)}".strip()
                if merged_rw_args:
                    cmd += ["--realworld-common-args", merged_rw_args]
                if str(args.sumo_extra_base).strip():
                    cmd += ["--sumo-extra-base", str(args.sumo_extra_base)]
                if float(args.max_sim_time_sec or 0.0) > 0.0:
                    cmd += ["--max-sim-time-sec", str(float(args.max_sim_time_sec))]
                if bool(args.terminate_on_ev_finish):
                    cmd.append("--terminate-on-ev-finish")
                if realtime_enabled_for_unit:
                    cmd.append("--realtime-sumo-enable")
                    if realtime_modes:
                        cmd += ["--realtime-sumo-modes", ",".join(sorted(realtime_modes))]
                    cmd += ["--realtime-sumo-factor", str(float(args.realtime_sumo_factor))]
                    cmd += ["--realtime-sumo-max-sleep-sec", str(float(args.realtime_sumo_max_sleep_sec))]
                    cmd += ["--realtime-sumo-log-period-sec", str(float(args.realtime_sumo_log_period_sec))]
                    cmd += [
                        "--realtime-sumo-start-sim-time-sec",
                        str(float(args.realtime_sumo_start_sim_time_sec)),
                    ]
                if bool(args.passive_intersection_dt_enable):
                    cmd.append("--passive-intersection-dt-enable")
                if float(args.passive_intersection_context_period_sec or 0.0) > 0.0:
                    cmd += [
                        "--passive-intersection-context-period-sec",
                        str(float(args.passive_intersection_context_period_sec)),
                    ]
                if int(args.passive_intersection_max_nodes or -1) >= 0:
                    cmd += ["--passive-intersection-max-nodes", str(int(args.passive_intersection_max_nodes))]
                if int(args.passive_intersection_lookahead_edges or 0) > 0:
                    cmd += ["--passive-intersection-lookahead-edges", str(int(args.passive_intersection_lookahead_edges))]
                if not bool(args.passive_intersection_context_route_fanout_enable):
                    cmd.append("--no-passive-intersection-context-route-fanout-enable")
                if int(args.passive_intersection_context_fanout_back_edges or 0) > 0:
                    cmd += [
                        "--passive-intersection-context-fanout-back-edges",
                        str(int(args.passive_intersection_context_fanout_back_edges)),
                    ]
                if int(args.passive_intersection_context_fanout_forward_edges or 0) > 0:
                    cmd += [
                        "--passive-intersection-context-fanout-forward-edges",
                        str(int(args.passive_intersection_context_fanout_forward_edges)),
                    ]
                if bool(args.ev_intersection_discovery_enable):
                    cmd.append("--ev-intersection-discovery-enable")
                if float(args.ev_intersection_discovery_delay_sec or 0.0) > 0.0:
                    cmd += [
                        "--ev-intersection-discovery-delay-sec",
                        str(float(args.ev_intersection_discovery_delay_sec)),
                    ]
                if str(args.ev_intersection_discovery_modes or "").strip():
                    cmd += ["--ev-intersection-discovery-modes", str(args.ev_intersection_discovery_modes).strip()]
                if str(args.ev_intersection_discovery_repeat_scope or "").strip():
                    cmd += [
                        "--ev-intersection-discovery-repeat-scope",
                        str(args.ev_intersection_discovery_repeat_scope).strip(),
                    ]
                if float(args.ev_intersection_discovery_wait_log_period_sec or 0.0) > 0.0:
                    cmd += [
                        "--ev-intersection-discovery-wait-log-period-sec",
                        str(float(args.ev_intersection_discovery_wait_log_period_sec)),
                    ]
                if str(args.f2p_passive_context_policy or "").strip():
                    cmd += ["--f2p-passive-context-policy", str(args.f2p_passive_context_policy)]
                if float(args.f2p_passive_context_max_age_sec or 0.0) > 0.0:
                    cmd += ["--f2p-passive-context-max-age-sec", str(float(args.f2p_passive_context_max_age_sec))]
                if int(args.f2p_passive_context_lookahead_edges or 0) > 0:
                    cmd += ["--f2p-passive-context-lookahead-edges", str(int(args.f2p_passive_context_lookahead_edges))]
                if int(args.f2p_passive_context_max_worst_edge_offset or 0) > 0:
                    cmd += [
                        "--f2p-passive-context-max-worst-edge-offset",
                        str(int(args.f2p_passive_context_max_worst_edge_offset)),
                    ]
                if int(args.f2p_passive_context_severe_min_halt_n or 0) > 0:
                    cmd += ["--f2p-passive-context-severe-min-halt-n", str(int(args.f2p_passive_context_severe_min_halt_n))]
                if int(args.f2p_passive_context_severe_min_veh_n or 0) > 0:
                    cmd += ["--f2p-passive-context-severe-min-veh-n", str(int(args.f2p_passive_context_severe_min_veh_n))]
                if float(args.f2p_passive_context_severe_max_mean_speed_mps or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-severe-max-mean-speed-mps",
                        str(float(args.f2p_passive_context_severe_max_mean_speed_mps)),
                    ]
                if float(args.f2p_passive_context_severe_max_occupancy_pct or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-severe-max-occupancy-pct",
                        str(float(args.f2p_passive_context_severe_max_occupancy_pct)),
                    ]
                if not bool(args.f2p_passive_context_missing_feedback_floor_enable):
                    cmd.append("--no-f2p-passive-context-missing-feedback-floor-enable")
                if float(args.f2p_passive_context_missing_feedback_max_queue_deficit_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-missing-feedback-max-queue-deficit-sec",
                        str(float(args.f2p_passive_context_missing_feedback_max_queue_deficit_sec)),
                    ]
                if float(args.f2p_passive_context_missing_feedback_max_spillback_risk or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-missing-feedback-max-spillback-risk",
                        str(float(args.f2p_passive_context_missing_feedback_max_spillback_risk)),
                    ]
                if float(args.f2p_passive_context_missing_feedback_max_timing_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-missing-feedback-max-timing-sec",
                        str(float(args.f2p_passive_context_missing_feedback_max_timing_sec)),
                    ]
                if not bool(args.f2p_passive_context_clear_missing_feedback_enable):
                    cmd.append("--no-f2p-passive-context-clear-missing-feedback-enable")
                if float(args.f2p_passive_context_clear_missing_feedback_no_feedback_penalty or 0.0) > 0.0:
                    cmd += [
                        "--f2p-passive-context-clear-missing-feedback-no-feedback-penalty",
                        str(float(args.f2p_passive_context_clear_missing_feedback_no_feedback_penalty)),
                    ]
                if bool(args.f2p_queue_release_enable):
                    cmd.append("--f2p-queue-release-enable")
                if float(args.f2p_queue_release_hold_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2p-queue-release-hold-sec",
                        str(float(args.f2p_queue_release_hold_sec)),
                    ]
                if float(args.f2p_queue_release_min_interval_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2p-queue-release-min-interval-sec",
                        str(float(args.f2p_queue_release_min_interval_sec)),
                    ]
                if int(args.f2p_queue_release_max_worst_edge_offset or 0) > 0:
                    cmd += [
                        "--f2p-queue-release-max-worst-edge-offset",
                        str(int(args.f2p_queue_release_max_worst_edge_offset)),
                    ]
                if bool(args.external_downstream_context_enable):
                    cmd.append("--external-downstream-context-enable")
                if float(args.external_downstream_context_max_age_sec or 0.0) > 0.0:
                    cmd += [
                        "--external-downstream-context-max-age-sec",
                        str(float(args.external_downstream_context_max_age_sec)),
                    ]
                if unit_is_f2d and bool(args.f2_drone_context_request_enable):
                    cmd.append("--f2-drone-context-request-enable")
                if unit_is_f2d and str(args.f2_drone_context_provider_id or "").strip():
                    cmd += ["--f2-drone-context-provider-id", str(args.f2_drone_context_provider_id).strip()]
                if unit_is_f2d and float(args.f2_drone_context_request_ttl_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2-drone-context-request-ttl-sec",
                        str(float(args.f2_drone_context_request_ttl_sec)),
                    ]
                if unit_is_f2d and float(args.f2_drone_context_request_min_interval_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2-drone-context-request-min-interval-sec",
                        str(float(args.f2_drone_context_request_min_interval_sec)),
                    ]
                if unit_is_f2d and int(args.f2_drone_context_request_max_edges or 0) > 0:
                    cmd += ["--f2-drone-context-request-max-edges", str(int(args.f2_drone_context_request_max_edges))]
                if unit_is_f2d and not bool(args.f2_drone_context_include_route_context):
                    cmd.append("--no-f2-drone-context-include-route-context")
                if unit_is_f2d and int(args.f2_drone_context_route_context_max_edges or 0) > 0:
                    cmd += [
                        "--f2-drone-context-route-context-max-edges",
                        str(int(args.f2_drone_context_route_context_max_edges)),
                    ]
                if unit_is_f2d and bool(args.no_f2_drone_context_emit_discovery_query):
                    cmd.append("--no-f2-drone-context-emit-discovery-query")
                if unit_is_f2d and bool(args.f2_drone_context_discovery_gate_enable):
                    cmd.append("--f2-drone-context-discovery-gate-enable")
                if unit_is_f2d and float(args.f2_drone_context_discovery_cache_ttl_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2-drone-context-discovery-cache-ttl-sec",
                        str(float(args.f2_drone_context_discovery_cache_ttl_sec)),
                    ]
                if unit_is_f2d and float(args.f2_drone_context_discovery_query_min_interval_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2-drone-context-discovery-query-min-interval-sec",
                        str(float(args.f2_drone_context_discovery_query_min_interval_sec)),
                    ]
                if unit_is_f2d and bool(args.f2d_queue_release_enable):
                    cmd.append("--f2d-queue-release-enable")
                if unit_is_f2d and float(args.f2d_queue_release_hold_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2d-queue-release-hold-sec",
                        str(float(args.f2d_queue_release_hold_sec)),
                    ]
                if unit_is_f2d and float(args.f2d_queue_release_min_interval_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2d-queue-release-min-interval-sec",
                        str(float(args.f2d_queue_release_min_interval_sec)),
                    ]
                if unit_is_f2d and int(args.f2d_queue_release_max_worst_edge_offset or 0) > 0:
                    cmd += [
                        "--f2d-queue-release-max-worst-edge-offset",
                        str(int(args.f2d_queue_release_max_worst_edge_offset)),
                    ]
                if unit_is_f2d and bool(args.f2d_drone_prescout_enable):
                    cmd.append("--f2d-drone-prescout-enable")
                if unit_is_f2d and bool(args.no_f2d_drone_prescout_first_tls_only):
                    cmd.append("--no-f2d-drone-prescout-first-tls-only")
                if unit_is_f2d and int(args.f2d_drone_prescout_max_edges or 0) > 0:
                    cmd += [
                        "--f2d-drone-prescout-max-edges",
                        str(int(args.f2d_drone_prescout_max_edges)),
                    ]
                if unit_is_f2d and float(args.f2d_drone_prescout_min_interval_sec or 0.0) > 0.0:
                    cmd += [
                        "--f2d-drone-prescout-min-interval-sec",
                        str(float(args.f2d_drone_prescout_min_interval_sec)),
                    ]
                if unit_is_f2d and bool(args.f2d_contextual_topic_delivery_enable):
                    cmd.append("--f2d-contextual-topic-delivery-enable")
                if unit_is_f2d and bool(args.no_f2d_directed_context_delivery_enable):
                    cmd.append("--no-f2d-directed-context-delivery-enable")
                if unit_is_f2d and bool(args.f2d_directed_context_self_delivery_enable):
                    cmd.append("--f2d-directed-context-self-delivery-enable")
                if str(args.fed_debug_log_mode).strip():
                    cmd += ["--fed-debug-log-mode", str(args.fed_debug_log_mode).strip()]
                if bool(args.skip_existing):
                    cmd.append("--skip-existing")
                if bool(args.fail_on_foreign_ev_drop):
                    cmd.append("--fail-on-foreign-ev-drop")
                    cmd += ["--foreign-ev-drop-fail-threshold", str(int(args.foreign_ev_drop_fail_threshold))]
                if bool(args.dry_run):
                    cmd.append("--dry-run")

                launch_cmd = list(cmd)
                print("RUN launch:", " ".join(shlex.quote(x) for x in cmd))
                if not args.dry_run:
                    launch_start_epoch = time.time()
                    subprocess.check_call(cmd, cwd=str(sim_root))
                    launch_end_epoch = time.time()
                    one = _read_csv(launch_out / "ev_matrix_results.csv")
                    collected_rows_n = len(one)
                    sidecar_summary_now = _sidecar_log_summary(sidecar_log_dir)
                    sidecar_totals_now = dict(sidecar_summary_now.get("totals") or {})
                    ev_sidecar_now = dict((sidecar_summary_now.get("by_name") or {}).get("fnm_ev.jsonl") or {})
                    ev_sidecar_fp_now = dict(ev_sidecar_now.get("fingerprint") or {})
                    for one_row in one:
                        one_row["matrix_unit_context_json"] = str(unit_context_path)
                        one_row["sidecar_log_dir"] = str(sidecar_log_dir)
                        one_row["sidecar_ev_jsonl_sha256_12"] = str(ev_sidecar_fp_now.get("sha256_12", ""))
                        one_row["sidecar_ev_jsonl_lines"] = str(ev_sidecar_now.get("lines", ""))
                        one_row["sidecar_ev_mqtt_connect_fail_n"] = str(ev_sidecar_now.get("fnm_mqtt_connect_fail_n", 0))
                        one_row["sidecar_ev_state_pull_ok_n"] = str(ev_sidecar_now.get("fnm_state_pull_ok_n", 0))
                        one_row["sidecar_ev_request_publish_n"] = str(ev_sidecar_now.get("fnm_ev_request_publish_n", 0))
                        one_row["sidecar_all_mqtt_connect_fail_n"] = str(sidecar_totals_now.get("fnm_mqtt_connect_fail_n", 0))
                        one_row["sidecar_all_state_pull_ok_n"] = str(sidecar_totals_now.get("fnm_state_pull_ok_n", 0))
                        one_row["mqtt_topic_namespace"] = str(unit_topic_namespace or "")
                        one_row["ev_http_port"] = str(selected_http_port if selected_http_port is not None else "")
                        one_row["fnm_data_run_id"] = str(unit_run_id or "")
                    all_results.extend(one)
                    print(
                        f"[matrix+fnm][scenario {scenario_idx}/{total_scenarios}][unit {unit_idx}/{unit_total}] "
                        f"collected_rows={len(one)} cumulative_rows={len(all_results)}"
                    )
            finally:
                _stop_sidecars(sidecar_proc)
                if not args.dry_run:
                    unit_end_epoch = time.time()
                    sidecar_summary = _sidecar_log_summary(sidecar_log_dir)
                    unit_context = {
                        "schema": "ev_matrix_fnm_unit_context.v1",
                        "generated_at_epoch": float(unit_end_epoch),
                        "scenario": {
                            "scenario_idx": int(scenario_idx),
                            "total_scenarios": int(total_scenarios),
                            "scenario_id": scenario_id,
                            "density_label": density_label,
                            "density_count": str(density_count),
                            "route_id": str(route_id),
                            "ev_id": ev_id,
                            "manifest_row": dict(row),
                        },
                        "unit": {
                            "unit_idx": int(unit_idx),
                            "unit_total": int(unit_total),
                            "unit_label": str(unit_label),
                            "unit_modes": list(unit_modes),
                            "sidecars_isolation": str(args.sidecars_isolation),
                            "topic_namespace": str(unit_topic_namespace or ""),
                            "fnm_data_run_id": str(unit_run_id or ""),
                        },
                        "timing": {
                            "unit_start_epoch": float(unit_start_epoch),
                            "sidecar_ready_epoch": sidecar_ready_epoch,
                            "launch_start_epoch": launch_start_epoch,
                            "launch_end_epoch": launch_end_epoch,
                            "unit_end_epoch": float(unit_end_epoch),
                            "unit_wall_elapsed_s": round(float(unit_end_epoch - unit_start_epoch), 3),
                            "sidecar_ready_delay_s": (
                                round(float(sidecar_ready_epoch - unit_start_epoch), 3)
                                if sidecar_ready_epoch is not None
                                else None
                            ),
                            "launch_wall_elapsed_s": (
                                round(float(launch_end_epoch - launch_start_epoch), 3)
                                if launch_start_epoch is not None and launch_end_epoch is not None
                                else None
                            ),
                            "post_launch_to_stop_s": (
                                round(float(unit_end_epoch - launch_end_epoch), 3)
                                if launch_end_epoch is not None
                                else None
                            ),
                        },
                        "runtime": {
                            "ev_http_port": int(selected_http_port) if selected_http_port is not None else None,
                            "ev_http_port_strategy": str(args.ev_http_port_strategy),
                            "mqtt_host": str(args.mqtt_host or ""),
                            "mqtt_port": int(args.mqtt_port or 0),
                            "fnm_tick_sec": float(args.fnm_tick_sec),
                            "fnm_stagger_sec": float(args.fnm_stagger_sec),
                            "sidecars_ready_wait_sec": float(args.sidecars_ready_wait_sec),
                            "sidecars_preflight_timeout_sec": float(args.sidecars_preflight_timeout_sec),
                            "sidecars_runner_ev_preflight_enable": bool(args.sidecars_runner_ev_preflight_enable),
                        },
                        "inputs": {
                            "route_file": _file_fingerprint(route_file),
                            "ev_cfg": _file_fingerprint(ev_cfg),
                            "ev_cfg_runtime": _file_fingerprint(ev_cfg_runtime),
                            "intersection_config_dir": str(inter_cfg_dir),
                            "generator_template": _file_fingerprint(gen_template),
                            "net_file": _file_fingerprint(Path(args.net_file).resolve()),
                            "real_world_script": _file_fingerprint(Path(args.real_world_script).resolve()),
                            "base_sumocfg": _file_fingerprint(Path(args.base_sumocfg).resolve()),
                            "fnm_script": _file_fingerprint(fnm_script),
                            "sidecars_runner": _file_fingerprint(sidecars_runner),
                            "launch_script": _file_fingerprint(launch_script),
                        },
                        "commands": {
                            "launch_argv": list(launch_cmd),
                            "launch_argv_sha256_12": _sha256_12_text(_compact_json(list(launch_cmd))),
                            "launch_shell": " ".join(shlex.quote(x) for x in launch_cmd),
                        },
                        "outputs": {
                            "launch_out": str(launch_out),
                            "launch_results": _file_fingerprint(launch_out / "ev_matrix_results.csv"),
                            "launch_run_meta": _file_fingerprint(launch_out / "run_meta.json"),
                            "sidecar_log_dir": str(sidecar_log_dir),
                            "sidecar_start_context": _file_fingerprint(sidecar_log_dir / "sidecar_start_context.json"),
                            "sidecar_summary": sidecar_summary,
                            "collected_rows_n": int(collected_rows_n),
                        },
                    }
                    unit_context_path.write_text(
                        json.dumps(unit_context, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                if str(args.sidecars_isolation) == "mode" and not args.no_sidecars:
                    # Give MQTT clients a short, deterministic disconnect window before
                    # the next mode starts and reuses the same EV HTTP port.
                    time.sleep(0.5)

        try:
            if str(args.sidecars_isolation) == "mode" and not args.no_sidecars:
                for unit_idx, mode in enumerate(requested_modes, start=1):
                    _run_matrix_unit(
                        unit_modes=[mode],
                        unit_label=mode,
                        unit_idx=unit_idx,
                        unit_total=len(requested_modes),
                    )
            else:
                _run_matrix_unit(
                    unit_modes=requested_modes,
                    unit_label="scenario",
                    unit_idx=1,
                    unit_total=1,
                )

        finally:
            scenario_elapsed_s = time.time() - scenario_t0
            print(
                f"[matrix+fnm][scenario_done {scenario_idx}/{total_scenarios}] id={scenario_id} elapsed_s={scenario_elapsed_s:.2f}"
            )

    total_elapsed_s = time.time() - matrix_t0

    if not args.dry_run:
        results_csv = out_dir / "ev_matrix_results.csv"
        summary_csv = out_dir / "ev_matrix_summary.csv"
        if all_results:
            fields = list(all_results[0].keys())
            _write_csv(results_csv, [dict(r) for r in all_results], fields)
            policy_fields = [
                "scenario_id",
                "density_label",
                "density_count",
                "route_id",
                "mode",
                "ev_id",
                "policy_args_json",
                "policy_args_sha256_12",
                "behavior_policy_args_sha256_12",
                "effective_realworld_args_sha256_12",
                "run_context_json",
                "matrix_unit_context_json",
                "mqtt_topic_namespace",
                "fnm_data_run_id",
            ]
            if any(str(r.get("policy_args_json", "")).strip() for r in all_results):
                _write_csv(
                    out_dir / "policy_args_manifest.csv",
                    [{k: r.get(k, "") for k in policy_fields} for r in all_results],
                    policy_fields,
                )
            summary_rows = _group_summary(all_results)
            _write_csv(
                summary_csv,
                summary_rows,
                [
                    "density_label",
                    "density_count",
                    "mode",
                    "n_runs",
                    "n",
                    "n_arrived",
                    "n_censored",
                    "travel_time_mean_s",
                    "travel_time_p50_s",
                    "travel_time_p95_s",
                    "travel_time_min_s",
                    "travel_time_max_s",
                ],
            )
            print(f"[matrix+fnm] results={results_csv}")
            print(f"[matrix+fnm] summary={summary_csv}")
        else:
            print("[matrix+fnm][WARN] no results collected")

    print(f"[matrix+fnm] total_elapsed_s={total_elapsed_s:.2f}")


if __name__ == "__main__":
    main()
