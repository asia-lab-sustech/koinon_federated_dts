#!/usr/bin/env python3
"""Launch EV matrix experiments from scenario manifest.

For each scenario row and each mode (B0/B1/F2/F2P/F2P-Q/F2D/F2D-Q/F2PD):
- build a per-run SUMO config referencing scenario route file
- run real-world.py with forced run-specific args
- run F2-family labels through the same F2 control family:
  F2P=passive non-TLS observers,
  F2P-Q=F2P plus experimental passive-triggered downstream queue release,
  F2D=Drone-DT context,
  F2D-Q=F2D plus experimental downstream queue release, F2PD=both
- parse SUMO tripinfo for EV travel time
- export per-run and summary CSV + optional figure
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import select
import shlex
import socket
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence


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
        data = path.read_bytes()
        out["size_bytes"] = int(len(data))
        out["sha256_12"] = _sha256_12_bytes(data)
        out["mtime_epoch"] = float(st.st_mtime)
    except Exception:
        return out
    return out


def _compact_json(obj: object) -> str:
    return json.dumps(obj, ensure_ascii=True, sort_keys=True, separators=(",", ":"))


POLICY_ARG_PREFIXES = (
    "--b1-",
    "--f2-",
    "--f2p-",
    "--fed-",
    "--federation-",
    "--drrs-",
    "--ev-request-",
    "--passive-intersection-",
    "--priority-",
    "--route-",
    "--service-window-",
)
POLICY_ARG_NAMES = {
    "--evaluation",
    "--emergency-veh",
    "--external-downstream-context-enable",
    "--external-downstream-context-max-age-sec",
    "--max-sim-time-sec",
    "--mqtt-host",
    "--mqtt-port",
    "--mqtt-topic-namespace",
    "--mqtt-topic-namespace-strategy",
    "--sumo-cfg",
    "--sumo-extra-args",
    "--time-to-teleport",
}
RUN_SPECIFIC_POLICY_ARG_NAMES = {
    "--decision-log-csv",
    "--emergency-veh",
    "--fed-debug-log-file",
    "--fed-debug-log-reset",
    "--mqtt-topic-namespace",
    "--sumo-cfg",
    "--sumo-extra-args",
}


F2_FAMILY_MODES = {"F2", "F2P", "F2P-Q", "F2D", "F2D-Q", "F2PD"}
PASSIVE_DT_MODES = {"F2P", "F2P-Q", "F2PD"}
DRONE_AUGMENTED_MODES = {"F2D", "F2D-Q", "F2PD"}
F2P_QUEUE_RELEASE_MODES = {"F2P-Q"}
F2D_QUEUE_RELEASE_MODES = {"F2D-Q"}
DRONE_CONTEXT_ARG_VALUE_COUNTS = {
    "--external-downstream-context-enable": 0,
    "--no-external-downstream-context-enable": 0,
    "--external-downstream-context-max-age-sec": 1,
    "--f2-drone-context-request-enable": 0,
    "--no-f2-drone-context-request-enable": 0,
    "--f2-drone-context-provider-id": 1,
    "--f2-drone-context-request-ttl-sec": 1,
    "--f2-drone-context-request-min-interval-sec": 1,
    "--f2-drone-context-request-max-edges": 1,
    "--f2-drone-context-include-route-context": 0,
    "--no-f2-drone-context-include-route-context": 0,
    "--f2-drone-context-route-context-max-edges": 1,
    "--f2-drone-context-emit-discovery-query": 0,
    "--no-f2-drone-context-emit-discovery-query": 0,
    "--f2d-queue-release-enable": 0,
    "--no-f2d-queue-release-enable": 0,
    "--f2d-queue-release-hold-sec": 1,
    "--f2d-queue-release-min-interval-sec": 1,
    "--f2d-queue-release-max-worst-edge-offset": 1,
    "--f2d-drone-prescout-enable": 0,
    "--no-f2d-drone-prescout-first-tls-only": 0,
    "--f2d-drone-prescout-max-edges": 1,
    "--f2d-drone-prescout-min-interval-sec": 1,
    "--f2d-contextual-topic-delivery-enable": 0,
    "--no-f2d-contextual-topic-delivery-enable": 0,
    "--no-f2d-directed-context-delivery-enable": 0,
    "--f2d-directed-context-self-delivery-enable": 0,
}
F2P_QUEUE_RELEASE_ARG_VALUE_COUNTS = {
    "--f2p-queue-release-enable": 0,
    "--no-f2p-queue-release-enable": 0,
    "--f2p-queue-release-hold-sec": 1,
    "--f2p-queue-release-min-interval-sec": 1,
    "--f2p-queue-release-max-worst-edge-offset": 1,
}


def _controller_mode(mode: str) -> str:
    """Map experiment-label modes onto the underlying control family."""
    mode_u = str(mode or "").upper()
    return "F2" if mode_u in F2_FAMILY_MODES else mode_u


def _mode_uses_passive_dt(mode: str, explicit_enable: bool) -> bool:
    return bool(explicit_enable) or str(mode or "").upper() in PASSIVE_DT_MODES


def _mode_uses_drone_context(mode: str, explicit_enable: bool = False) -> bool:
    # Drone context is an explicit experiment family so F2 traces are not
    # accidentally polluted by shared F2D args.
    _ = explicit_enable
    return str(mode or "").upper() in DRONE_AUGMENTED_MODES


def _strip_cli_options(tokens: List[str], option_value_counts: Dict[str, int]) -> List[str]:
    out: List[str] = []
    i = 0
    while i < len(tokens):
        tok = str(tokens[i])
        opt = tok.split("=", 1)[0] if tok.startswith("--") else tok
        if opt in option_value_counts:
            n_values = int(option_value_counts.get(opt, 0) or 0)
            if "=" not in tok:
                i += 1 + n_values
            else:
                i += 1
            continue
        out.append(tok)
        i += 1
    return out


def _argv_to_arg_map(tokens: Sequence[str]) -> Dict[str, object]:
    """Best-effort argparse-style map for run fingerprinting."""
    out: Dict[str, object] = {}
    i = 0
    while i < len(tokens):
        tok = str(tokens[i])
        if not tok.startswith("--"):
            i += 1
            continue
        key = tok
        val: object = True
        if i + 1 < len(tokens) and not str(tokens[i + 1]).startswith("--"):
            val = str(tokens[i + 1])
            i += 1
        if key in out:
            prev = out[key]
            if isinstance(prev, list):
                prev.append(val)
            else:
                out[key] = [prev, val]
        else:
            out[key] = val
        i += 1
    return out


def _policy_args_snapshot(tokens: Sequence[str], mode: str) -> Dict[str, object]:
    arg_map = _argv_to_arg_map(tokens)
    policy_keys = sorted(
        k
        for k in arg_map
        if k in POLICY_ARG_NAMES or any(k.startswith(prefix) for prefix in POLICY_ARG_PREFIXES)
    )
    policy_args = {k: arg_map[k] for k in policy_keys}
    behavior_policy_args = {
        k: v for k, v in policy_args.items() if k not in RUN_SPECIFIC_POLICY_ARG_NAMES
    }
    effective_args = [str(x) for x in tokens]
    effective_shell = " ".join(shlex.quote(x) for x in effective_args)
    policy_json = _compact_json(policy_args)
    behavior_policy_json = _compact_json(behavior_policy_args)
    return {
        "schema": "ev_matrix_policy_args.v1",
        "mode": str(mode),
        "controller_mode": _controller_mode(str(arg_map.get("--evaluation", mode))),
        "passive_intersection_dt_enabled": bool("--passive-intersection-dt-enable" in arg_map),
        "drone_context_enabled": bool(
            "--f2-drone-context-request-enable" in arg_map
            or "--external-downstream-context-enable" in arg_map
        ),
        "policy_args": policy_args,
        "policy_args_sha256_12": _sha256_12_text(policy_json),
        "behavior_policy_args": behavior_policy_args,
        "behavior_policy_args_sha256_12": _sha256_12_text(behavior_policy_json),
        "effective_realworld_args": effective_args,
        "effective_realworld_args_shell": effective_shell,
        "effective_realworld_args_sha256_12": _sha256_12_text(_compact_json(effective_args)),
    }


def _find_event_jsonl(run_path: Path, fed_log: str) -> Path:
    candidates: List[Path] = []
    if str(fed_log or "").strip():
        p = Path(str(fed_log)).resolve()
        candidates.append(p.with_suffix(".events.jsonl"))
        if p.name.endswith(".txt"):
            candidates.append(p.with_name(p.name[:-4] + ".events.jsonl"))
    candidates.extend(sorted(run_path.glob("*.events.jsonl")))
    for p in candidates:
        if p.exists():
            return p
    return candidates[0] if candidates else (run_path / "fed_outcomes.events.jsonl")


def _event_jsonl_summary(path: Path) -> Dict[str, object]:
    from collections import Counter

    event_counts: Counter[str] = Counter()
    b1_apply_by_tls: Counter[str] = Counter()
    f2_apply_by_tls: Counter[str] = Counter()
    f2_skip_by_reason: Counter[str] = Counter()
    f2_guard_by_tls: Counter[str] = Counter()
    service_by_tls: Counter[str] = Counter()
    first_sim: Dict[str, object] = {}
    last_sim: Dict[str, object] = {}
    bad_json = 0
    lines = 0

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
                    ev = str(r.get("event_type") or r.get("event") or "")
                    if not ev:
                        ev = "-"
                    event_counts[ev] += 1
                    sim = r.get("sim_time", r.get("sim_time_s", r.get("time_s", "")))
                    if ev not in first_sim:
                        first_sim[ev] = sim
                    last_sim[ev] = sim
                    tls = str(r.get("tls_id") or r.get("node_id") or "")
                    if ev == "b1.apply":
                        b1_apply_by_tls[tls or "-"] += 1
                    if ev == "f2.apply":
                        f2_apply_by_tls[tls or "-"] += 1
                    if ev == "f2.apply_skipped":
                        reason = str(r.get("reason") or r.get("skip_reason") or "-")
                        f2_skip_by_reason[reason] += 1
                    if ev.startswith("f2.") and "guard" in ev:
                        f2_guard_by_tls[tls or "-"] += 1
                    if ev in {"ev.service_window.missed", "ev.service_window.stop_wait"}:
                        service_by_tls[f"{ev}:{tls or '-'}"] += 1
        except Exception:
            pass

    return {
        "event_jsonl": str(path),
        "event_jsonl_fingerprint": _file_fingerprint(path),
        "event_jsonl_lines": int(lines),
        "event_jsonl_bad_json": int(bad_json),
        "event_counts_top": dict(event_counts.most_common(40)),
        "event_first_sim_top": {k: first_sim.get(k, "") for k, _ in event_counts.most_common(40)},
        "event_last_sim_top": {k: last_sim.get(k, "") for k, _ in event_counts.most_common(40)},
        "b1_apply_n": int(event_counts.get("b1.apply", 0)),
        "b1_downstream_blockage_n": int(event_counts.get("b1.downstream_blockage", 0)),
        "b1_apply_by_tls": dict(b1_apply_by_tls.most_common()),
        "f2_apply_n": int(event_counts.get("f2.apply", 0)),
        "f2_apply_by_tls": dict(f2_apply_by_tls.most_common()),
        "f2_apply_skipped_n": int(event_counts.get("f2.apply_skipped", 0)),
        "f2_skip_by_reason": dict(f2_skip_by_reason.most_common()),
        "f2_guard_n": int(sum(f2_guard_by_tls.values())),
        "f2_guard_by_tls": dict(f2_guard_by_tls.most_common()),
        "f2p_queue_release_requested_n": int(event_counts.get("f2p.queue_release.requested", 0)),
        "f2p_queue_release_applied_n": int(event_counts.get("f2p.queue_release.applied", 0)),
        "f2d_queue_release_requested_n": int(event_counts.get("f2d.queue_release.requested", 0)),
        "f2d_queue_release_applied_n": int(event_counts.get("f2d.queue_release.applied", 0)),
        "service_window_missed_n": int(event_counts.get("ev.service_window.missed", 0)),
        "service_window_stop_wait_n": int(event_counts.get("ev.service_window.stop_wait", 0)),
        "service_window_by_tls": dict(service_by_tls.most_common()),
    }


def _read_manifest(path: Path) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append({k: str(v) for k, v in row.items()})
    return rows


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
) -> None:
    root = ET.parse(base_sumocfg).getroot()
    input_el = root.find("input")
    if input_el is None:
        input_el = ET.SubElement(root, "input")
    _set_or_create_value(input_el, "net-file", str(net_file))
    _set_or_create_value(input_el, "route-files", str(route_file))
    out_sumocfg.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out_sumocfg, encoding="utf-8", xml_declaration=True)


def _extract_tripinfo(tripinfo_xml: Path, ev_id: str) -> Dict[str, Optional[float]]:
    out: Dict[str, Optional[float]] = {
        "found": 0.0,
        "arrived": 0.0,
        "depart_s": None,
        "arrival_s": None,
        "travel_time_s": None,
        "waiting_time_s": None,
        "waiting_count_n": None,
        "time_loss_s": None,
        "stop_time_s": None,
        "route_length_m": None,
    }
    if not tripinfo_xml.exists():
        return out
    try:
        root = ET.parse(tripinfo_xml).getroot()
    except Exception:
        return out
    for ti in root.findall("tripinfo"):
        if str(ti.get("id", "")) != str(ev_id):
            continue
        out["found"] = 1.0
        try:
            out["depart_s"] = float(ti.get("depart")) if ti.get("depart") is not None else None
        except Exception:
            out["depart_s"] = None
        try:
            out["arrival_s"] = float(ti.get("arrival")) if ti.get("arrival") is not None else None
        except Exception:
            out["arrival_s"] = None
        try:
            out["travel_time_s"] = float(ti.get("duration")) if ti.get("duration") is not None else None
        except Exception:
            out["travel_time_s"] = None
        try:
            wt = ti.get("waitingTime")
            out["waiting_time_s"] = float(wt) if wt is not None else None
        except Exception:
            out["waiting_time_s"] = None
        try:
            wc = ti.get("waitingCount")
            out["waiting_count_n"] = float(wc) if wc is not None else None
        except Exception:
            out["waiting_count_n"] = None
        try:
            tl = ti.get("timeLoss")
            out["time_loss_s"] = float(tl) if tl is not None else None
        except Exception:
            out["time_loss_s"] = None
        try:
            st = ti.get("stopTime")
            out["stop_time_s"] = float(st) if st is not None else None
        except Exception:
            out["stop_time_s"] = None
        try:
            rl = ti.get("routeLength")
            out["route_length_m"] = float(rl) if rl is not None else None
        except Exception:
            out["route_length_m"] = None
        vaporized = str(ti.get("vaporized", "") or "").strip().lower()
        arrival_ok = out["arrival_s"] is not None and float(out["arrival_s"]) >= 0.0
        duration_ok = out["travel_time_s"] is not None and float(out["travel_time_s"]) >= 0.0
        out["arrived"] = 1.0 if (arrival_ok and duration_ok and not vaporized) else 0.0
        if not out["arrived"]:
            # SUMO --tripinfo-output.write-unfinished writes duration/waiting
            # for still-running vehicles at shutdown. Keep wait/loss diagnostics,
            # but do not expose that censored duration as EV travel time.
            out["arrival_s"] = None
            out["travel_time_s"] = None
        return out
    return out


def _extract_realworld_stop_summary(rw_log: Path) -> Dict[str, object]:
    out: Dict[str, object] = {
        "sim_stop_reason": "",
        "sim_stop_sim_time_s": "",
        "max_sim_time_sec": "",
        "ev_last_edge": "",
        "ev_last_speed_mps": "",
        "ev_nonarrival_censored": 0,
    }
    if not rw_log.exists():
        return out

    last_ev_edge = ""
    last_ev_speed = ""
    try:
        with rw_log.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "evt=EV_STATE_TRACE" in line:
                    m_edge = re.search(r"\bedge=([^ ]+)", line)
                    m_speed = re.search(r"\bspeed=([0-9.+-]+)", line)
                    if m_edge:
                        last_ev_edge = m_edge.group(1)
                    if m_speed:
                        last_ev_speed = m_speed.group(1)
                if "evt=EV_FINISH_EARLY_STOP" in line:
                    m_sim = re.search(r"\bsim=([0-9.+-]+)", line)
                    out.update(
                        {
                            "sim_stop_reason": "ev_arrived",
                            "sim_stop_sim_time_s": m_sim.group(1) if m_sim else "",
                            "ev_nonarrival_censored": 0,
                        }
                    )
                elif "evt=EV_MAX_SIM_TIME_STOP" in line:
                    m_sim = re.search(r"\bsim=([0-9.+-]+)", line)
                    m_max = re.search(r"\bmax_sim=([0-9.+-]+)", line)
                    m_edge = re.search(r"\bedge=([^ ]+)", line)
                    m_speed = re.search(r"\bspeed=([0-9.+-]+)", line)
                    out.update(
                        {
                            "sim_stop_reason": "max_sim_time",
                            "sim_stop_sim_time_s": m_sim.group(1) if m_sim else "",
                            "max_sim_time_sec": m_max.group(1) if m_max else "",
                            "ev_last_edge": m_edge.group(1) if m_edge else last_ev_edge,
                            "ev_last_speed_mps": m_speed.group(1) if m_speed else last_ev_speed,
                            "ev_nonarrival_censored": 1,
                        }
                    )
        if not out.get("ev_last_edge") and last_ev_edge:
            out["ev_last_edge"] = last_ev_edge
        if not out.get("ev_last_speed_mps") and last_ev_speed:
            out["ev_last_speed_mps"] = last_ev_speed
    except Exception:
        return out
    return out


def _write_csv(path: Path, rows: List[Dict[str, object]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fieldnames))
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _extract_cli_value(tokens: Sequence[str], flag: str) -> Optional[str]:
    # Argparse uses the last occurrence when the same flag appears multiple
    # times. Mirror that behavior for preflight checks.
    for i in range(len(tokens) - 2, -1, -1):
        tok = tokens[i]
        if tok == flag and i + 1 < len(tokens):
            return str(tokens[i + 1])
    return None


def _is_port_bindable(host: str, port: int) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((str(host), int(port)))
        return True
    except Exception:
        return False


def _run_realworld_logged(
    *,
    cmd: Sequence[str],
    cwd: Path,
    rw_log: Path,
    startup_check_enable: bool,
    startup_timeout_sec: float,
) -> Dict[str, object]:
    rw_log.parent.mkdir(parents=True, exist_ok=True)
    startup_ok = 0
    startup_fail_reason = ""
    with rw_log.open("w", encoding="utf-8") as f:
        f.write("# CMD\n")
        f.write(" ".join(shlex.quote(x) for x in cmd) + "\n\n")
        f.flush()
        proc = subprocess.Popen(
            list(cmd),
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )

        deadline = time.time() + max(1.0, float(startup_timeout_sec))
        saw_startup_line = False
        saw_startup_ok = False
        saw_startup_fail = False

        while True:
            if proc.stdout is None:
                break
            ready, _, _ = select.select([proc.stdout], [], [], 0.2)
            if ready:
                line = proc.stdout.readline()
                if line:
                    f.write(line)
                    if startup_check_enable and "EV_HTTP_STATE_SERVER_START" in line:
                        saw_startup_line = True
                        if "ok=1" in line:
                            saw_startup_ok = True
                        if "ok=0" in line:
                            saw_startup_fail = True
                            startup_fail_reason = "http_server_start_failed"
                            proc.terminate()
                            break
                elif proc.poll() is not None:
                    break

            if startup_check_enable and (not saw_startup_line) and (time.time() >= deadline):
                startup_fail_reason = "http_server_start_timeout"
                proc.terminate()
                break

            if proc.poll() is not None:
                break

        try:
            proc.wait(timeout=4.0)
        except Exception:
            proc.kill()
            proc.wait(timeout=2.0)

        if proc.stdout is not None:
            tail = proc.stdout.read() or ""
            if tail:
                f.write(tail)

        startup_ok = 1 if saw_startup_ok else 0
        rc = int(proc.returncode if proc.returncode is not None else -1)
        if startup_fail_reason and rc == 0:
            rc = 98

    return {
        "return_code": int(rc),
        "http_startup_ok": int(startup_ok),
        "http_startup_fail_reason": str(startup_fail_reason),
    }


def _extract_foreign_ev_drop_stats(rw_log: Path) -> Dict[str, int]:
    out = {
        "drop_foreign_ev_id_max": 0,
        "rx_drop_foreign_ev_lines": 0,
    }
    if not rw_log.exists():
        return out
    patt = re.compile(r"drop_foreign_ev_id=(\d+)")
    try:
        with rw_log.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "RX_DROP_FOREIGN_EV" in line:
                    out["rx_drop_foreign_ev_lines"] += 1
                if "drop_foreign_ev_id=" in line:
                    m = patt.search(line)
                    if m:
                        try:
                            n = int(m.group(1))
                            if n > out["drop_foreign_ev_id_max"]:
                                out["drop_foreign_ev_id_max"] = n
                        except Exception:
                            pass
    except Exception:
        return out
    return out


def _extract_ev_request_pipeline_stats(rw_log: Path) -> Dict[str, int]:
    out = {
        "ev_request_wait_for_fnm_enabled": 0,
        "ev_request_rx_total_max": 0,
        "ev_request_dispatch_ok_max": 0,
        "ev_request_wait_for_fnm_timeout_lines": 0,
        "ev_request_rx_enqueue_lines": 0,
        "ev_request_rx_dispatch_lines": 0,
    }
    if not rw_log.exists():
        return out
    rx_patt = re.compile(r"\brx_total=(\d+)")
    dispatch_patt = re.compile(r"\bdispatch_ok=(\d+)")
    try:
        with rw_log.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "evt=EV_REQ_WAIT_FOR_FNM" in line and "enabled=1" in line:
                    out["ev_request_wait_for_fnm_enabled"] = 1
                if "evt=ev.request.wait_for_fnm.timeout" in line or "ev.request.wait_for_fnm.timeout" in line:
                    out["ev_request_wait_for_fnm_timeout_lines"] += 1
                if "evt=RX_ENQUEUE topic=federation/ev/request/" in line:
                    out["ev_request_rx_enqueue_lines"] += 1
                if "evt=RX_DISPATCH topic=federation/ev/request/" in line:
                    out["ev_request_rx_dispatch_lines"] += 1
                if "EV_PIPELINE_STATS" in line:
                    m = rx_patt.search(line)
                    if m:
                        out["ev_request_rx_total_max"] = max(out["ev_request_rx_total_max"], int(m.group(1)))
                    m = dispatch_patt.search(line)
                    if m:
                        out["ev_request_dispatch_ok_max"] = max(out["ev_request_dispatch_ok_max"], int(m.group(1)))
    except Exception:
        return out
    return out


def _plot_results(results: List[Dict[str, object]], out_png: Path, out_svg: Path) -> None:
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as e:
        print(f"[launch_matrix][WARN] plotting skipped (missing dependency): {type(e).__name__}:{e}")
        return

    modes = sorted({str(r["mode"]) for r in results})
    densities = sorted({str(r["density_label"]) for r in results})
    route_ids = sorted({int(r["route_id"]) for r in results})
    mode_order = {"B0": 0, "B1": 1, "F2": 2, "F2P": 3, "F2P-Q": 4, "F2D": 5, "F2D-Q": 6, "F2PD": 7}
    modes = sorted(modes, key=lambda m: (mode_order.get(str(m), 99), str(m)))
    mode_colors = {
        "B0": "#7f8c8d",
        "B1": "#2e8b57",
        "F2": "#e67e22",
        "F2P": "#8e44ad",
        "F2P-Q": "#16a085",
        "F2D": "#2c7fb8",
        "F2D-Q": "#f39c12",
        "F2PD": "#c0392b",
    }

    fig, axes = plt.subplots(1, len(densities), figsize=(6.5 * len(densities), 5.4), squeeze=False)
    width = 0.24
    x = np.arange(len(route_ids))

    for j, dens in enumerate(densities):
        ax = axes[0][j]
        sub = [r for r in results if str(r["density_label"]) == dens]
        dens_count = sorted({int(r["density_count"]) for r in sub})
        dens_txt = dens_count[0] if dens_count else -1
        for i, mode in enumerate(modes):
            vals = []
            for rid in route_ids:
                cand = [r for r in sub if int(r["route_id"]) == rid and str(r["mode"]) == mode]
                if cand and cand[0].get("travel_time_s") not in ("", None):
                    vals.append(float(cand[0]["travel_time_s"]))
                else:
                    vals.append(float("nan"))
            ax.bar(
                x + (i - (len(modes) - 1) / 2.0) * width,
                vals,
                width=width,
                color=mode_colors.get(mode),
                edgecolor="black",
                linewidth=0.35,
                label=mode,
            )
        ax.set_title(f"{dens.title()} ({dens_txt} vehicles)")
        ax.set_xticks(x)
        ax.set_xticklabels([str(r) for r in route_ids])
        ax.set_xlabel("Route Number")
        if j == 0:
            ax.set_ylabel("EV Travel Time (s)")
        ax.grid(axis="y", linestyle="--", alpha=0.25)

    h, l = axes[0][0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=max(1, len(modes)), frameon=False)
    fig.suptitle("EV Travel Time by Route, Density, and Mode")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    fig.savefig(out_svg, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Launch EV experiment matrix from scenario manifest.")
    ap.add_argument("--manifest-csv", required=True)
    ap.add_argument("--sim-root", default=".")
    ap.add_argument("--python-bin", default=sys.executable or "python3")
    ap.add_argument("--real-world-script", required=True)
    ap.add_argument("--base-sumocfg", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument(
        "--modes",
        default="B0,B1,F2",
        help=(
            "comma-separated experiment modes. F2P=F2+passive non-TLS DTs, "
            "F2P-Q=F2P plus experimental passive-triggered downstream queue release, "
            "F2D=F2+Drone-DT downstream context, F2D-Q=F2D plus experimental "
            "drone-triggered downstream queue release, F2PD=passive+drone."
        ),
    )
    ap.add_argument(
        "--realworld-common-args",
        default="",
        help="raw args string appended to real-world.py for every run",
    )
    ap.add_argument(
        "--realworld-common-args-file",
        default="",
        help="optional text file with raw args for every run (one line or multi-line)",
    )
    ap.add_argument(
        "--sumo-extra-base",
        default="",
        help="base SUMO extra args; tripinfo output flags are appended automatically",
    )
    ap.add_argument(
        "--max-sim-time-sec",
        type=float,
        default=0.0,
        help=(
            "forwarded to real-world.py as a simulation-time hard stop; "
            "0 disables. Non-arrivals are recorded as censored outcomes."
        ),
    )
    ap.add_argument(
        "--terminate-on-ev-finish",
        action="store_true",
        default=False,
        help=(
            "forwarded to real-world.py; stop successful runs immediately after the emergency vehicle arrives"
        ),
    )
    ap.add_argument(
        "--realtime-sumo-enable",
        action="store_true",
        default=False,
        help="forwarded to real-world.py for selected modes; pace SUMO simulation time to wall-clock time",
    )
    ap.add_argument(
        "--realtime-sumo-modes",
        default="",
        help=(
            "comma-separated modes that should receive real-time SUMO pacing when "
            "--realtime-sumo-enable is set. Empty means all modes."
        ),
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
            "simulation time at which wall-clock pacing starts after fast pre-roll"
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
    ap.add_argument(
        "--f2p-passive-context-policy",
        choices=["disabled", "missing_feedback_only", "severe_or_missing", "immediate_missing_severe", "always"],
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
    ap.add_argument("--f2d-queue-release-enable", action="store_true", default=False)
    ap.add_argument("--f2d-queue-release-hold-sec", type=float, default=0.0)
    ap.add_argument("--f2d-queue-release-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2d-queue-release-max-worst-edge-offset", type=int, default=0)
    ap.add_argument("--f2d-drone-prescout-enable", action="store_true", default=False)
    ap.add_argument("--no-f2d-drone-prescout-first-tls-only", action="store_true", default=False)
    ap.add_argument("--f2d-drone-prescout-max-edges", type=int, default=0)
    ap.add_argument("--f2d-drone-prescout-min-interval-sec", type=float, default=0.0)
    ap.add_argument("--f2d-contextual-topic-delivery-enable", action="store_true", default=False)
    ap.add_argument("--no-f2d-contextual-topic-delivery-enable", action="store_true", default=False)
    ap.add_argument("--no-f2d-directed-context-delivery-enable", action="store_true", default=False)
    ap.add_argument("--f2d-directed-context-self-delivery-enable", action="store_true", default=False)
    ap.add_argument(
        "--fed-debug-log-mode",
        choices=["per-run", "common-if-set", "common-only"],
        default="per-run",
        help=(
            "How to set --fed-debug-log-file for real-world runs: "
            "per-run=always force run-local fed_outcomes.txt; "
            "common-if-set=use common args value when present else force per-run; "
            "common-only=never force, rely only on common args."
        ),
    )
    ap.add_argument("--skip-existing", action="store_true", default=False)
    ap.add_argument("--http-startup-check-enable", action="store_true", default=True)
    ap.add_argument("--no-http-startup-check-enable", dest="http_startup_check_enable", action="store_false")
    ap.add_argument("--http-startup-timeout-sec", type=float, default=25.0)
    ap.add_argument("--http-precheck-bind-enable", action="store_true", default=True)
    ap.add_argument("--no-http-precheck-bind-enable", dest="http_precheck_bind_enable", action="store_false")
    ap.add_argument(
        "--fail-on-foreign-ev-drop",
        action="store_true",
        default=False,
        help="fail a run when real-world log reports drop_foreign_ev_id above threshold",
    )
    ap.add_argument(
        "--foreign-ev-drop-fail-threshold",
        type=int,
        default=0,
        help="threshold for drop_foreign_ev_id when --fail-on-foreign-ev-drop is enabled",
    )
    ap.add_argument("--dry-run", action="store_true", default=False)
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    manifest_csv = Path(args.manifest_csv).resolve()
    sim_root = Path(args.sim_root).resolve()
    rw_script = Path(args.real_world_script).resolve()
    base_sumocfg = Path(args.base_sumocfg).resolve()
    net_file = Path(args.net_file).resolve()
    out_dir = Path(args.out_dir).resolve()
    run_dir = out_dir / "runs"
    cfg_dir = out_dir / "sumocfg"
    logs_dir = out_dir / "logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    cfg_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    rows = _read_manifest(manifest_csv)
    modes = [m.strip().upper() for m in str(args.modes or "").split(",") if m.strip()]
    if not modes:
        raise SystemExit("No modes provided")
    realtime_sumo_modes = {
        str(x).strip().upper()
        for x in str(args.realtime_sumo_modes or "").split(",")
        if str(x).strip()
    }

    common_tokens: List[str] = []
    if str(args.realworld_common_args_file).strip():
        txt = Path(args.realworld_common_args_file).resolve().read_text(encoding="utf-8")
        common_tokens.extend(shlex.split(txt))
    if str(args.realworld_common_args).strip():
        common_tokens.extend(shlex.split(str(args.realworld_common_args)))
    common_fed_log = _extract_cli_value(common_tokens, "--fed-debug-log-file")
    launcher_fp = _file_fingerprint(Path(__file__).resolve())
    real_world_fp = _file_fingerprint(rw_script)
    base_sumocfg_fp = _file_fingerprint(base_sumocfg)
    net_file_fp = _file_fingerprint(net_file)
    common_args_file = Path(args.realworld_common_args_file).resolve() if str(args.realworld_common_args_file).strip() else None
    common_args_fp = {
        "realworld_common_args": str(args.realworld_common_args or ""),
        "realworld_common_args_sha256_12": _sha256_12_text(str(args.realworld_common_args or "")),
        "realworld_common_args_file": str(common_args_file) if common_args_file else "",
        "realworld_common_args_file_fingerprint": _file_fingerprint(common_args_file) if common_args_file else {},
        "common_tokens": list(common_tokens),
        "common_tokens_sha256_12": _sha256_12_text(_compact_json(list(common_tokens))),
    }

    total_scenarios = len(rows)
    total_runs = total_scenarios * len(modes)
    matrix_t0 = time.time()
    run_index = 0

    print(
        f"[launch_matrix] scenarios={total_scenarios} modes={len(modes)} total_runs={total_runs}"
    )

    results: List[Dict[str, object]] = []
    for scenario_idx, row in enumerate(rows, start=1):
        scenario_id = str(row.get("scenario_id", "scenario"))
        density_label = str(row.get("density_label", "density"))
        density_count = int(str(row.get("density_count", "0") or "0"))
        route_id = int(str(row.get("route_id", "0") or "0"))
        ev_id = str(row.get("ev_id", "emergency1"))
        route_file = Path(str(row.get("route_file", ""))).resolve()
        if not route_file.exists() and not args.dry_run:
            raise SystemExit(f"Missing route_file in manifest: {route_file}")

        print(
            f"[launch_matrix][scenario {scenario_idx}/{total_scenarios}] id={scenario_id} density={density_label}({density_count}) route={route_id} ev={ev_id}"
        )

        sumocfg_variant = cfg_dir / f"{scenario_id}.sumocfg"
        if not args.dry_run:
            _write_sumocfg_variant(
                base_sumocfg=base_sumocfg,
                out_sumocfg=sumocfg_variant,
                net_file=net_file,
                route_file=route_file,
            )

        for mode in modes:
            controller_mode = _controller_mode(mode)
            passive_dt_enabled_for_mode = _mode_uses_passive_dt(
                mode, bool(args.passive_intersection_dt_enable)
            )
            realtime_sumo_enabled_for_mode = bool(args.realtime_sumo_enable) and (
                not realtime_sumo_modes or str(mode).upper() in realtime_sumo_modes
            )
            drone_context_enabled_for_mode = _mode_uses_drone_context(
                mode,
                bool(args.f2_drone_context_request_enable)
                or bool(args.external_downstream_context_enable),
            )
            run_common_tokens = list(common_tokens)
            if not drone_context_enabled_for_mode:
                run_common_tokens = _strip_cli_options(
                    run_common_tokens,
                    DRONE_CONTEXT_ARG_VALUE_COUNTS,
                )
            if not passive_dt_enabled_for_mode:
                run_common_tokens = _strip_cli_options(
                    run_common_tokens,
                    F2P_QUEUE_RELEASE_ARG_VALUE_COUNTS,
                )
            run_index += 1
            run_t0 = time.time()
            print(
                f"[launch_matrix][run {run_index}/{total_runs}] scenario={scenario_id} mode={mode} controller={controller_mode} density={density_label} route={route_id} ev={ev_id}"
            )

            run_key = f"{scenario_id}_{mode}"
            run_path = run_dir / density_label / f"route_{route_id}" / mode
            run_path.mkdir(parents=True, exist_ok=True)
            tripinfo_xml = run_path / "tripinfo.xml"
            force_per_run_fed_log = bool(args.fed_debug_log_mode == "per-run")
            if args.fed_debug_log_mode == "common-if-set" and not common_fed_log:
                force_per_run_fed_log = True
            if args.fed_debug_log_mode == "common-only":
                force_per_run_fed_log = False

            fed_log_common = str(common_fed_log or "").strip()
            fed_log = str(run_path / "fed_outcomes.txt") if force_per_run_fed_log else fed_log_common
            rw_log = logs_dir / f"{run_key}.log"
            decision_csv = run_path / "intersection_decisions.csv"
            run_context_json = run_path / "run_context.json"
            policy_args_json = run_path / "policy_args_effective.json"

            if args.skip_existing and tripinfo_xml.exists():
                trip = _extract_tripinfo(tripinfo_xml, ev_id)
                event_summary = _event_jsonl_summary(_find_event_jsonl(run_path, fed_log))
                stop_summary = _extract_realworld_stop_summary(rw_log)
                if int(trip["arrived"] or 0) > 0:
                    stop_summary["ev_nonarrival_censored"] = 0
                    if not str(stop_summary.get("sim_stop_reason", "")):
                        stop_summary["sim_stop_reason"] = "ev_arrived"
                results.append(
                    {
                        "scenario_id": scenario_id,
                        "density_label": density_label,
                        "density_count": density_count,
                        "route_id": route_id,
                        "mode": mode,
                        "controller_mode": controller_mode,
                        "passive_intersection_dt_enabled": int(passive_dt_enabled_for_mode),
                        "drone_context_enabled": int(drone_context_enabled_for_mode),
                        "ev_id": ev_id,
                        "travel_time_s": trip["travel_time_s"],
                        "depart_s": trip["depart_s"],
                        "arrival_s": trip["arrival_s"],
                        "arrived": int(trip["arrived"] or 0),
                        "return_code": 0,
                        "tripinfo_xml": str(tripinfo_xml),
                        "fed_log": str(fed_log),
                        "realworld_log": str(rw_log),
                        "wall_elapsed_s": 0.0,
                        "sim_stop_reason": str(stop_summary.get("sim_stop_reason", "")),
                        "sim_stop_sim_time_s": str(stop_summary.get("sim_stop_sim_time_s", "")),
                        "max_sim_time_sec": str(stop_summary.get("max_sim_time_sec", "")),
                        "ev_nonarrival_censored": int(stop_summary.get("ev_nonarrival_censored", 0)),
                        "ev_last_edge": str(stop_summary.get("ev_last_edge", "")),
                        "ev_last_speed_mps": str(stop_summary.get("ev_last_speed_mps", "")),
                        "waiting_time_s": trip["waiting_time_s"],
                        "waiting_count_n": trip["waiting_count_n"],
                        "time_loss_s": trip["time_loss_s"],
                        "stop_time_s": trip["stop_time_s"],
                        "route_length_m": trip["route_length_m"],
                        "http_precheck_ok": "",
                        "http_startup_ok": "",
                        "http_startup_fail_reason": "",
                        "drop_foreign_ev_id_max": "",
                        "rx_drop_foreign_ev_lines": "",
                        "foreign_ev_drop_fail": "",
                        "ev_request_wait_for_fnm_enabled": "",
                        "ev_request_rx_total_max": "",
                        "ev_request_dispatch_ok_max": "",
                        "ev_request_wait_for_fnm_timeout_lines": "",
                        "ev_request_rx_enqueue_lines": "",
                        "ev_request_rx_dispatch_lines": "",
                        "zero_ev_request_rx_fail": "",
                        "event_jsonl": str(event_summary.get("event_jsonl", "")),
                        "event_jsonl_sha256_12": str(
                            dict(event_summary.get("event_jsonl_fingerprint") or {}).get("sha256_12", "")
                        ),
                        "event_jsonl_lines": int(event_summary.get("event_jsonl_lines", 0)),
                        "b1_apply_n": int(event_summary.get("b1_apply_n", 0)),
                        "b1_downstream_blockage_n": int(event_summary.get("b1_downstream_blockage_n", 0)),
                        "f2_apply_n": int(event_summary.get("f2_apply_n", 0)),
                        "f2_apply_skipped_n": int(event_summary.get("f2_apply_skipped_n", 0)),
                        "f2_guard_n": int(event_summary.get("f2_guard_n", 0)),
                        "f2p_queue_release_requested_n": int(
                            event_summary.get("f2p_queue_release_requested_n", 0)
                        ),
                        "f2p_queue_release_applied_n": int(
                            event_summary.get("f2p_queue_release_applied_n", 0)
                        ),
                        "f2d_queue_release_requested_n": int(
                            event_summary.get("f2d_queue_release_requested_n", 0)
                        ),
                        "f2d_queue_release_applied_n": int(
                            event_summary.get("f2d_queue_release_applied_n", 0)
                        ),
                        "service_window_missed_n": int(event_summary.get("service_window_missed_n", 0)),
                        "service_window_stop_wait_n": int(event_summary.get("service_window_stop_wait_n", 0)),
                        "policy_args_json": str(policy_args_json),
                        "policy_args_sha256_12": "",
                        "behavior_policy_args_sha256_12": "",
                        "effective_realworld_args_sha256_12": "",
                        "run_context_json": str(run_context_json),
                    }
                )
                print(
                    f"[launch_matrix][skip {run_index}/{total_runs}] scenario={scenario_id} mode={mode} existing_tripinfo=1 travel_time_s={trip['travel_time_s']}"
                )
                continue

            sumo_extra_parts: List[str] = []
            if str(args.sumo_extra_base).strip():
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

            cmd = [str(args.python_bin), str(rw_script)]
            cmd.extend(run_common_tokens)
            # Force run-specific values at the end (override if present in common args)
            cmd.extend(
                [
                    "--sumo-cfg",
                    str(sumocfg_variant),
                    "--net-file",
                    str(net_file),
                    "--emergency-veh",
                    str(ev_id),
                    "--evaluation",
                    str(mode),
                ]
            )
            if float(args.max_sim_time_sec or 0.0) > 0.0:
                cmd.extend(["--max-sim-time-sec", str(float(args.max_sim_time_sec))])
            if bool(args.terminate_on_ev_finish):
                cmd.append("--terminate-on-ev-finish")
            if realtime_sumo_enabled_for_mode:
                cmd.append("--realtime-sumo-enable")
                cmd.extend(["--realtime-sumo-factor", str(float(args.realtime_sumo_factor))])
                cmd.extend(["--realtime-sumo-max-sleep-sec", str(float(args.realtime_sumo_max_sleep_sec))])
                cmd.extend(["--realtime-sumo-log-period-sec", str(float(args.realtime_sumo_log_period_sec))])
                cmd.extend([
                    "--realtime-sumo-start-sim-time-sec",
                    str(float(args.realtime_sumo_start_sim_time_sec)),
                ])
            if passive_dt_enabled_for_mode:
                cmd.append("--passive-intersection-dt-enable")
            if float(args.passive_intersection_context_period_sec or 0.0) > 0.0:
                cmd.extend([
                    "--passive-intersection-context-period-sec",
                    str(float(args.passive_intersection_context_period_sec)),
                ])
            if int(args.passive_intersection_max_nodes or -1) >= 0:
                cmd.extend(["--passive-intersection-max-nodes", str(int(args.passive_intersection_max_nodes))])
            if int(args.passive_intersection_lookahead_edges or 0) > 0:
                cmd.extend(["--passive-intersection-lookahead-edges", str(int(args.passive_intersection_lookahead_edges))])
            if not bool(args.passive_intersection_context_route_fanout_enable):
                cmd.append("--no-passive-intersection-context-route-fanout-enable")
            if int(args.passive_intersection_context_fanout_back_edges or 0) > 0:
                cmd.extend([
                    "--passive-intersection-context-fanout-back-edges",
                    str(int(args.passive_intersection_context_fanout_back_edges)),
                ])
            if int(args.passive_intersection_context_fanout_forward_edges or 0) > 0:
                cmd.extend([
                    "--passive-intersection-context-fanout-forward-edges",
                    str(int(args.passive_intersection_context_fanout_forward_edges)),
                ])
            if str(args.f2p_passive_context_policy or "").strip():
                cmd.extend(["--f2p-passive-context-policy", str(args.f2p_passive_context_policy)])
            if float(args.f2p_passive_context_max_age_sec or 0.0) > 0.0:
                cmd.extend(["--f2p-passive-context-max-age-sec", str(float(args.f2p_passive_context_max_age_sec))])
            if int(args.f2p_passive_context_lookahead_edges or 0) > 0:
                cmd.extend(["--f2p-passive-context-lookahead-edges", str(int(args.f2p_passive_context_lookahead_edges))])
            if int(args.f2p_passive_context_max_worst_edge_offset or 0) > 0:
                cmd.extend([
                    "--f2p-passive-context-max-worst-edge-offset",
                    str(int(args.f2p_passive_context_max_worst_edge_offset)),
                ])
            if int(args.f2p_passive_context_severe_min_halt_n or 0) > 0:
                cmd.extend(["--f2p-passive-context-severe-min-halt-n", str(int(args.f2p_passive_context_severe_min_halt_n))])
            if int(args.f2p_passive_context_severe_min_veh_n or 0) > 0:
                cmd.extend(["--f2p-passive-context-severe-min-veh-n", str(int(args.f2p_passive_context_severe_min_veh_n))])
            if float(args.f2p_passive_context_severe_max_mean_speed_mps or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-severe-max-mean-speed-mps",
                    str(float(args.f2p_passive_context_severe_max_mean_speed_mps)),
                ])
            if float(args.f2p_passive_context_severe_max_occupancy_pct or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-severe-max-occupancy-pct",
                    str(float(args.f2p_passive_context_severe_max_occupancy_pct)),
                ])
            if not bool(args.f2p_passive_context_missing_feedback_floor_enable):
                cmd.append("--no-f2p-passive-context-missing-feedback-floor-enable")
            if float(args.f2p_passive_context_missing_feedback_max_queue_deficit_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-missing-feedback-max-queue-deficit-sec",
                    str(float(args.f2p_passive_context_missing_feedback_max_queue_deficit_sec)),
                ])
            if float(args.f2p_passive_context_missing_feedback_max_spillback_risk or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-missing-feedback-max-spillback-risk",
                    str(float(args.f2p_passive_context_missing_feedback_max_spillback_risk)),
                ])
            if float(args.f2p_passive_context_missing_feedback_max_timing_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-missing-feedback-max-timing-sec",
                    str(float(args.f2p_passive_context_missing_feedback_max_timing_sec)),
                ])
            if not bool(args.f2p_passive_context_clear_missing_feedback_enable):
                cmd.append("--no-f2p-passive-context-clear-missing-feedback-enable")
            if float(args.f2p_passive_context_clear_missing_feedback_no_feedback_penalty or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-passive-context-clear-missing-feedback-no-feedback-penalty",
                    str(float(args.f2p_passive_context_clear_missing_feedback_no_feedback_penalty)),
                ])
            if passive_dt_enabled_for_mode and (
                str(mode).upper() in F2P_QUEUE_RELEASE_MODES
                or bool(args.f2p_queue_release_enable)
            ):
                cmd.append("--f2p-queue-release-enable")
            if passive_dt_enabled_for_mode and float(args.f2p_queue_release_hold_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-queue-release-hold-sec",
                    str(float(args.f2p_queue_release_hold_sec)),
                ])
            if passive_dt_enabled_for_mode and float(args.f2p_queue_release_min_interval_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2p-queue-release-min-interval-sec",
                    str(float(args.f2p_queue_release_min_interval_sec)),
                ])
            if passive_dt_enabled_for_mode and int(args.f2p_queue_release_max_worst_edge_offset or 0) > 0:
                cmd.extend([
                    "--f2p-queue-release-max-worst-edge-offset",
                    str(int(args.f2p_queue_release_max_worst_edge_offset)),
                ])
            if drone_context_enabled_for_mode:
                cmd.append("--f2-drone-context-request-enable")
                cmd.append("--external-downstream-context-enable")
            if float(args.external_downstream_context_max_age_sec or 0.0) > 0.0:
                cmd.extend([
                    "--external-downstream-context-max-age-sec",
                    str(float(args.external_downstream_context_max_age_sec)),
                ])
            if str(args.f2_drone_context_provider_id or "").strip():
                cmd.extend(["--f2-drone-context-provider-id", str(args.f2_drone_context_provider_id).strip()])
            if float(args.f2_drone_context_request_ttl_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2-drone-context-request-ttl-sec",
                    str(float(args.f2_drone_context_request_ttl_sec)),
                ])
            if float(args.f2_drone_context_request_min_interval_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2-drone-context-request-min-interval-sec",
                    str(float(args.f2_drone_context_request_min_interval_sec)),
                ])
            if int(args.f2_drone_context_request_max_edges or 0) > 0:
                cmd.extend(["--f2-drone-context-request-max-edges", str(int(args.f2_drone_context_request_max_edges))])
            if not bool(args.f2_drone_context_include_route_context):
                cmd.append("--no-f2-drone-context-include-route-context")
            if int(args.f2_drone_context_route_context_max_edges or 0) > 0:
                cmd.extend([
                    "--f2-drone-context-route-context-max-edges",
                    str(int(args.f2_drone_context_route_context_max_edges)),
                ])
            if bool(args.no_f2_drone_context_emit_discovery_query):
                cmd.append("--no-f2-drone-context-emit-discovery-query")
            if drone_context_enabled_for_mode and (
                str(mode).upper() in F2D_QUEUE_RELEASE_MODES
                or bool(args.f2d_queue_release_enable)
            ):
                cmd.append("--f2d-queue-release-enable")
            if float(args.f2d_queue_release_hold_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2d-queue-release-hold-sec",
                    str(float(args.f2d_queue_release_hold_sec)),
                ])
            if float(args.f2d_queue_release_min_interval_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2d-queue-release-min-interval-sec",
                    str(float(args.f2d_queue_release_min_interval_sec)),
                ])
            if int(args.f2d_queue_release_max_worst_edge_offset or 0) > 0:
                cmd.extend([
                    "--f2d-queue-release-max-worst-edge-offset",
                    str(int(args.f2d_queue_release_max_worst_edge_offset)),
                ])
            if drone_context_enabled_for_mode and str(mode).upper() == "F2D" and bool(args.f2d_drone_prescout_enable):
                cmd.append("--f2d-drone-prescout-enable")
            if drone_context_enabled_for_mode and bool(args.no_f2d_drone_prescout_first_tls_only):
                cmd.append("--no-f2d-drone-prescout-first-tls-only")
            if drone_context_enabled_for_mode and int(args.f2d_drone_prescout_max_edges or 0) > 0:
                cmd.extend([
                    "--f2d-drone-prescout-max-edges",
                    str(int(args.f2d_drone_prescout_max_edges)),
                ])
            if drone_context_enabled_for_mode and float(args.f2d_drone_prescout_min_interval_sec or 0.0) > 0.0:
                cmd.extend([
                    "--f2d-drone-prescout-min-interval-sec",
                    str(float(args.f2d_drone_prescout_min_interval_sec)),
                ])
            if drone_context_enabled_for_mode and bool(args.f2d_contextual_topic_delivery_enable):
                cmd.append("--f2d-contextual-topic-delivery-enable")
            if drone_context_enabled_for_mode and bool(args.no_f2d_contextual_topic_delivery_enable):
                cmd.append("--no-f2d-contextual-topic-delivery-enable")
            if drone_context_enabled_for_mode and bool(args.no_f2d_directed_context_delivery_enable):
                cmd.append("--no-f2d-directed-context-delivery-enable")
            if drone_context_enabled_for_mode and bool(args.f2d_directed_context_self_delivery_enable):
                cmd.append("--f2d-directed-context-self-delivery-enable")
            if force_per_run_fed_log:
                cmd.extend(
                    [
                        "--fed-debug-log-file",
                        str(fed_log),
                        "--fed-debug-log-reset",
                    ]
                )
            cmd.extend(
                [
                    "--decision-log-csv",
                    str(decision_csv),
                    "--sumo-extra-args",
                    str(sumo_extra),
                ]
            )
            policy_snapshot = _policy_args_snapshot(cmd[2:], mode)
            if not args.dry_run:
                policy_args_json.write_text(
                    json.dumps(policy_snapshot, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )

            if args.dry_run:
                print("DRY_RUN:", " ".join(shlex.quote(x) for x in cmd))
                rc = 0
                http_precheck_ok = ""
                http_startup_ok = ""
                http_startup_fail_reason = ""
                ev_http_enabled = "--ev-http-state-server-enable" in cmd
                http_host = _extract_cli_value(cmd, "--ev-http-state-server-host") or "127.0.0.1"
                raw_port = _extract_cli_value(cmd, "--ev-http-state-server-port")
                http_port = int(raw_port) if raw_port is not None else None
            else:
                ev_http_enabled = "--ev-http-state-server-enable" in cmd
                http_host = _extract_cli_value(cmd, "--ev-http-state-server-host") or "127.0.0.1"
                raw_port = _extract_cli_value(cmd, "--ev-http-state-server-port")
                http_port = int(raw_port) if raw_port is not None else None
                startup_check = bool(args.http_startup_check_enable and ev_http_enabled)

                http_precheck_ok = ""
                if startup_check and bool(args.http_precheck_bind_enable) and http_port is not None:
                    bindable = _is_port_bindable(http_host, int(http_port))
                    http_precheck_ok = 1 if bindable else 0
                    if not bindable:
                        rw_log.parent.mkdir(parents=True, exist_ok=True)
                        with rw_log.open("w", encoding="utf-8") as f:
                            f.write("# CMD\n")
                            f.write(" ".join(shlex.quote(x) for x in cmd) + "\n\n")
                            f.write(
                                f"[launch_matrix][precheck] http_bind_busy host={http_host} port={int(http_port)}\n"
                            )
                        rc = 98
                        http_startup_ok = 0
                        http_startup_fail_reason = "http_bind_precheck_failed"
                    else:
                        run_info = _run_realworld_logged(
                            cmd=cmd,
                            cwd=sim_root,
                            rw_log=rw_log,
                            startup_check_enable=startup_check,
                            startup_timeout_sec=float(args.http_startup_timeout_sec),
                        )
                        rc = int(run_info["return_code"])
                        http_startup_ok = int(run_info["http_startup_ok"])
                        http_startup_fail_reason = str(run_info["http_startup_fail_reason"])
                else:
                    run_info = _run_realworld_logged(
                        cmd=cmd,
                        cwd=sim_root,
                        rw_log=rw_log,
                        startup_check_enable=startup_check,
                        startup_timeout_sec=float(args.http_startup_timeout_sec),
                    )
                    rc = int(run_info["return_code"])
                    http_startup_ok = int(run_info["http_startup_ok"]) if startup_check else ""
                    http_startup_fail_reason = str(run_info["http_startup_fail_reason"])

            trip = _extract_tripinfo(tripinfo_xml, ev_id)
            foreign_stats = _extract_foreign_ev_drop_stats(rw_log) if not args.dry_run else {
                "drop_foreign_ev_id_max": 0,
                "rx_drop_foreign_ev_lines": 0,
            }
            ev_req_stats = _extract_ev_request_pipeline_stats(rw_log) if not args.dry_run else {
                "ev_request_wait_for_fnm_enabled": 0,
                "ev_request_rx_total_max": 0,
                "ev_request_dispatch_ok_max": 0,
                "ev_request_wait_for_fnm_timeout_lines": 0,
                "ev_request_rx_enqueue_lines": 0,
                "ev_request_rx_dispatch_lines": 0,
            }
            event_jsonl = _find_event_jsonl(run_path, fed_log)
            event_summary = _event_jsonl_summary(event_jsonl)
            stop_summary = _extract_realworld_stop_summary(rw_log) if not args.dry_run else {
                "sim_stop_reason": "",
                "sim_stop_sim_time_s": "",
                "max_sim_time_sec": "",
                "ev_nonarrival_censored": 0,
                "ev_last_edge": "",
                "ev_last_speed_mps": "",
            }
            if int(trip["arrived"] or 0) > 0:
                stop_summary["ev_nonarrival_censored"] = 0
                if not str(stop_summary.get("sim_stop_reason", "")):
                    stop_summary["sim_stop_reason"] = "ev_arrived"
            foreign_fail = 0
            if (
                not args.dry_run
                and bool(args.fail_on_foreign_ev_drop)
                and int(foreign_stats.get("drop_foreign_ev_id_max", 0)) > int(args.foreign_ev_drop_fail_threshold)
            ):
                foreign_fail = 1
                if int(rc) == 0:
                    rc = 97
                reason = f"foreign_ev_drop_gt_{int(args.foreign_ev_drop_fail_threshold)}"
                if not str(http_startup_fail_reason):
                    http_startup_fail_reason = reason
                else:
                    http_startup_fail_reason = f"{http_startup_fail_reason};{reason}"

            zero_ev_request_rx_fail = 0
            if (
                not args.dry_run
                and str(mode).upper() in {"B1", "F2", "F2P"}
                and int(ev_req_stats.get("ev_request_wait_for_fnm_enabled", 0)) > 0
                and int(ev_req_stats.get("ev_request_rx_total_max", 0)) <= 0
            ):
                zero_ev_request_rx_fail = 1
                if int(rc) == 0:
                    rc = 96
                reason = "zero_ev_request_rx_with_wait_for_fnm"
                if not str(http_startup_fail_reason):
                    http_startup_fail_reason = reason
                else:
                    http_startup_fail_reason = f"{http_startup_fail_reason};{reason}"

            run_cmd_str = " ".join(shlex.quote(x) for x in cmd)
            run_elapsed_s = round(float(time.time() - run_t0), 3)
            run_context = {
                "schema": "ev_matrix_run_context.v1",
                "generated_at_epoch": float(time.time()),
                "scenario": {
                    "scenario_idx": int(scenario_idx),
                    "scenario_id": scenario_id,
                    "density_label": density_label,
                    "density_count": int(density_count),
                    "route_id": int(route_id),
                    "ev_id": ev_id,
                    "manifest_row": dict(row),
                },
                "mode": str(mode),
                "controller_mode": str(controller_mode),
                "passive_intersection_dt_enabled": bool(passive_dt_enabled_for_mode),
                "run_index": int(run_index),
                "total_runs": int(total_runs),
                "timing": {
                    "run_start_epoch": float(run_t0),
                    "run_end_epoch": float(time.time()),
                    "wall_elapsed_s": float(run_elapsed_s),
                },
                "command": {
                    "argv": list(cmd),
                    "shell_escaped": run_cmd_str,
                    "argv_sha256_12": _sha256_12_text(_compact_json(list(cmd))),
                    "shell_sha256_12": _sha256_12_text(run_cmd_str),
                    "cwd": str(sim_root),
                    "launcher_pid": int(os.getpid()),
                    "python_bin": str(args.python_bin),
                },
                "inputs": {
                    "manifest_csv": str(manifest_csv),
                    "route_file": _file_fingerprint(route_file),
                    "sumocfg_variant": _file_fingerprint(sumocfg_variant),
                    "base_sumocfg": base_sumocfg_fp,
                    "net_file": net_file_fp,
                    "real_world_script": real_world_fp,
                    "launcher_script": launcher_fp,
                    "common_args": common_args_fp,
                    "sumo_extra": str(sumo_extra),
                    "sumo_extra_sha256_12": _sha256_12_text(str(sumo_extra)),
                },
                "runtime": {
                    "ev_http_enabled": bool(ev_http_enabled),
                    "ev_http_host": str(http_host),
                    "ev_http_port": int(http_port) if http_port is not None else None,
                    "mqtt_host": _extract_cli_value(cmd, "--mqtt-host") or "",
                    "mqtt_port": _extract_cli_value(cmd, "--mqtt-port") or "",
                    "mqtt_topic_namespace": _extract_cli_value(cmd, "--mqtt-topic-namespace") or "",
                    "fed_debug_log_file": str(fed_log),
                    "fed_debug_log_mode": str(args.fed_debug_log_mode),
                },
                "policy": {
                    **dict(policy_snapshot),
                    "policy_args_json": str(policy_args_json),
                },
                "outputs": {
                    "tripinfo_xml": _file_fingerprint(tripinfo_xml),
                    "realworld_log": _file_fingerprint(rw_log),
                    "decision_csv": _file_fingerprint(decision_csv),
                    "fed_log": _file_fingerprint(Path(fed_log)) if str(fed_log).strip() else {},
                    "event_summary": event_summary,
                },
                "result": {
                    "return_code": int(rc),
                    "trip": dict(trip),
                    "stop_summary": dict(stop_summary),
                    "foreign_ev_drop": dict(foreign_stats),
                    "ev_request_pipeline": dict(ev_req_stats),
                    "http_precheck_ok": http_precheck_ok,
                    "http_startup_ok": http_startup_ok,
                    "http_startup_fail_reason": str(http_startup_fail_reason),
                    "foreign_ev_drop_fail": int(foreign_fail),
                    "zero_ev_request_rx_fail": int(zero_ev_request_rx_fail),
                },
            }
            if not args.dry_run:
                run_context_json.write_text(
                    json.dumps(run_context, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )

            results.append(
                {
                    "scenario_id": scenario_id,
                    "density_label": density_label,
                    "density_count": density_count,
                    "route_id": route_id,
                    "mode": mode,
                    "controller_mode": controller_mode,
                    "passive_intersection_dt_enabled": int(passive_dt_enabled_for_mode),
                    "drone_context_enabled": int(drone_context_enabled_for_mode),
                    "ev_id": ev_id,
                    "travel_time_s": trip["travel_time_s"],
                    "depart_s": trip["depart_s"],
                    "arrival_s": trip["arrival_s"],
                    "arrived": int(trip["arrived"] or 0),
                    "return_code": int(rc),
                    "tripinfo_xml": str(tripinfo_xml),
                    "fed_log": str(fed_log),
                    "realworld_log": str(rw_log),
                    "wall_elapsed_s": run_elapsed_s,
                    "sim_stop_reason": str(stop_summary.get("sim_stop_reason", "")),
                    "sim_stop_sim_time_s": str(stop_summary.get("sim_stop_sim_time_s", "")),
                    "max_sim_time_sec": str(stop_summary.get("max_sim_time_sec", "")),
                    "ev_nonarrival_censored": int(stop_summary.get("ev_nonarrival_censored", 0)),
                    "ev_last_edge": str(stop_summary.get("ev_last_edge", "")),
                    "ev_last_speed_mps": str(stop_summary.get("ev_last_speed_mps", "")),
                    "waiting_time_s": trip["waiting_time_s"],
                    "waiting_count_n": trip["waiting_count_n"],
                    "time_loss_s": trip["time_loss_s"],
                    "stop_time_s": trip["stop_time_s"],
                    "route_length_m": trip["route_length_m"],
                    "http_precheck_ok": http_precheck_ok,
                    "http_startup_ok": http_startup_ok,
                    "http_startup_fail_reason": http_startup_fail_reason,
                    "drop_foreign_ev_id_max": int(foreign_stats.get("drop_foreign_ev_id_max", 0)),
                    "rx_drop_foreign_ev_lines": int(foreign_stats.get("rx_drop_foreign_ev_lines", 0)),
                    "foreign_ev_drop_fail": int(foreign_fail),
                    "ev_request_wait_for_fnm_enabled": int(ev_req_stats.get("ev_request_wait_for_fnm_enabled", 0)),
                    "ev_request_rx_total_max": int(ev_req_stats.get("ev_request_rx_total_max", 0)),
                    "ev_request_dispatch_ok_max": int(ev_req_stats.get("ev_request_dispatch_ok_max", 0)),
                    "ev_request_wait_for_fnm_timeout_lines": int(ev_req_stats.get("ev_request_wait_for_fnm_timeout_lines", 0)),
                    "ev_request_rx_enqueue_lines": int(ev_req_stats.get("ev_request_rx_enqueue_lines", 0)),
                    "ev_request_rx_dispatch_lines": int(ev_req_stats.get("ev_request_rx_dispatch_lines", 0)),
                    "zero_ev_request_rx_fail": int(zero_ev_request_rx_fail),
                    "event_jsonl": str(event_summary.get("event_jsonl", "")),
                    "event_jsonl_sha256_12": str(
                        dict(event_summary.get("event_jsonl_fingerprint") or {}).get("sha256_12", "")
                    ),
                    "event_jsonl_lines": int(event_summary.get("event_jsonl_lines", 0)),
                    "b1_apply_n": int(event_summary.get("b1_apply_n", 0)),
                    "b1_downstream_blockage_n": int(event_summary.get("b1_downstream_blockage_n", 0)),
                    "f2_apply_n": int(event_summary.get("f2_apply_n", 0)),
                    "f2_apply_skipped_n": int(event_summary.get("f2_apply_skipped_n", 0)),
                    "f2_guard_n": int(event_summary.get("f2_guard_n", 0)),
                    "f2p_queue_release_requested_n": int(
                        event_summary.get("f2p_queue_release_requested_n", 0)
                    ),
                    "f2p_queue_release_applied_n": int(
                        event_summary.get("f2p_queue_release_applied_n", 0)
                    ),
                    "f2d_queue_release_requested_n": int(
                        event_summary.get("f2d_queue_release_requested_n", 0)
                    ),
                    "f2d_queue_release_applied_n": int(
                        event_summary.get("f2d_queue_release_applied_n", 0)
                    ),
                    "service_window_missed_n": int(event_summary.get("service_window_missed_n", 0)),
                    "service_window_stop_wait_n": int(event_summary.get("service_window_stop_wait_n", 0)),
                    "policy_args_json": str(policy_args_json),
                    "policy_args_sha256_12": str(policy_snapshot.get("policy_args_sha256_12", "")),
                    "behavior_policy_args_sha256_12": str(
                        policy_snapshot.get("behavior_policy_args_sha256_12", "")
                    ),
                    "effective_realworld_args_sha256_12": str(
                        policy_snapshot.get("effective_realworld_args_sha256_12", "")
                    ),
                    "run_context_json": str(run_context_json),
                }
            )
            elapsed_s = time.time() - run_t0
            print(
                f"[launch_matrix][done {run_index}/{total_runs}] scenario={scenario_id} mode={mode} rc={rc} travel_time_s={trip['travel_time_s']} elapsed_s={elapsed_s:.2f} "
                f"arrived={int(trip['arrived'] or 0)} stop_reason={str(stop_summary.get('sim_stop_reason', '') or '-')} "
                f"http_precheck_ok={http_precheck_ok} http_startup_ok={http_startup_ok} http_fail={http_startup_fail_reason or '-'} "
                f"drop_foreign_ev_id_max={int(foreign_stats.get('drop_foreign_ev_id_max', 0))} "
                f"rx_drop_foreign_ev_lines={int(foreign_stats.get('rx_drop_foreign_ev_lines', 0))} "
                f"ev_req_rx={int(ev_req_stats.get('ev_request_rx_total_max', 0))} "
                f"ev_req_dispatch={int(ev_req_stats.get('ev_request_dispatch_ok_max', 0))} "
                f"zero_ev_req_rx_fail={int(zero_ev_request_rx_fail)}"
            )

    _write_csv(
        out_dir / "ev_matrix_results.csv",
        results,
        [
            "scenario_id",
            "density_label",
            "density_count",
            "route_id",
            "mode",
            "controller_mode",
            "passive_intersection_dt_enabled",
            "drone_context_enabled",
            "ev_id",
            "travel_time_s",
            "depart_s",
            "arrival_s",
            "arrived",
            "return_code",
            "tripinfo_xml",
            "fed_log",
            "realworld_log",
            "wall_elapsed_s",
            "sim_stop_reason",
            "sim_stop_sim_time_s",
            "max_sim_time_sec",
            "ev_nonarrival_censored",
            "ev_last_edge",
            "ev_last_speed_mps",
            "waiting_time_s",
            "waiting_count_n",
            "time_loss_s",
            "stop_time_s",
            "route_length_m",
            "http_precheck_ok",
            "http_startup_ok",
            "http_startup_fail_reason",
            "drop_foreign_ev_id_max",
            "rx_drop_foreign_ev_lines",
            "foreign_ev_drop_fail",
            "ev_request_wait_for_fnm_enabled",
            "ev_request_rx_total_max",
            "ev_request_dispatch_ok_max",
            "ev_request_wait_for_fnm_timeout_lines",
            "ev_request_rx_enqueue_lines",
            "ev_request_rx_dispatch_lines",
            "zero_ev_request_rx_fail",
            "event_jsonl",
            "event_jsonl_sha256_12",
            "event_jsonl_lines",
            "b1_apply_n",
            "b1_downstream_blockage_n",
            "f2_apply_n",
            "f2_apply_skipped_n",
            "f2_guard_n",
            "f2p_queue_release_requested_n",
            "f2p_queue_release_applied_n",
            "f2d_queue_release_requested_n",
            "f2d_queue_release_applied_n",
            "service_window_missed_n",
            "service_window_stop_wait_n",
            "policy_args_json",
            "policy_args_sha256_12",
            "behavior_policy_args_sha256_12",
            "effective_realworld_args_sha256_12",
            "run_context_json",
        ],
    )

    policy_manifest_fields = [
        "scenario_id",
        "density_label",
        "density_count",
        "route_id",
        "mode",
        "controller_mode",
        "passive_intersection_dt_enabled",
        "drone_context_enabled",
        "ev_id",
        "policy_args_json",
        "policy_args_sha256_12",
        "behavior_policy_args_sha256_12",
        "effective_realworld_args_sha256_12",
        "run_context_json",
    ]
    _write_csv(
        out_dir / "policy_args_manifest.csv",
        [{k: r.get(k, "") for k in policy_manifest_fields} for r in results],
        policy_manifest_fields,
    )

    summary_rows: List[Dict[str, object]] = []
    dens_labels = sorted({str(r["density_label"]) for r in results})
    for dens in dens_labels:
        for mode in modes:
            rows_dm = [
                r
                for r in results
                if str(r["density_label"]) == dens and str(r["mode"]) == mode
            ]
            vals = [
                float(r["travel_time_s"])
                for r in rows_dm
                if r.get("travel_time_s") not in ("", None)
            ]
            summary_rows.append(
                {
                    "density_label": dens,
                    "mode": mode,
                    "n_runs": len(rows_dm),
                    "n": len(vals),
                    "n_arrived": sum(1 for r in rows_dm if str(r.get("arrived", "0")) == "1"),
                    "n_censored": sum(1 for r in rows_dm if str(r.get("ev_nonarrival_censored", "0")) == "1"),
                    "mean_s": (statistics.mean(vals) if vals else None),
                    "std_s": (statistics.pstdev(vals) if len(vals) > 1 else 0.0 if vals else None),
                    "min_s": (min(vals) if vals else None),
                    "max_s": (max(vals) if vals else None),
                    "waiting_time_mean_s": (
                        statistics.mean(
                            [
                                float(r["waiting_time_s"])
                                for r in results
                                if str(r["density_label"]) == dens
                                and str(r["mode"]) == mode
                                and r.get("waiting_time_s") not in ("", None)
                            ]
                        )
                        if any(
                            str(r["density_label"]) == dens
                            and str(r["mode"]) == mode
                            and r.get("waiting_time_s") not in ("", None)
                            for r in results
                        )
                        else None
                    ),
                    "time_loss_mean_s": (
                        statistics.mean(
                            [
                                float(r["time_loss_s"])
                                for r in results
                                if str(r["density_label"]) == dens
                                and str(r["mode"]) == mode
                                and r.get("time_loss_s") not in ("", None)
                            ]
                        )
                        if any(
                            str(r["density_label"]) == dens
                            and str(r["mode"]) == mode
                            and r.get("time_loss_s") not in ("", None)
                            for r in results
                        )
                        else None
                    ),
                    "stop_time_mean_s": (
                        statistics.mean(
                            [
                                float(r["stop_time_s"])
                                for r in results
                                if str(r["density_label"]) == dens
                                and str(r["mode"]) == mode
                                and r.get("stop_time_s") not in ("", None)
                            ]
                        )
                        if any(
                            str(r["density_label"]) == dens
                            and str(r["mode"]) == mode
                            and r.get("stop_time_s") not in ("", None)
                            for r in results
                        )
                        else None
                    ),
                    "waiting_count_mean_n": (
                        statistics.mean(
                            [
                                float(r["waiting_count_n"])
                                for r in results
                                if str(r["density_label"]) == dens
                                and str(r["mode"]) == mode
                                and r.get("waiting_count_n") not in ("", None)
                            ]
                        )
                        if any(
                            str(r["density_label"]) == dens
                            and str(r["mode"]) == mode
                            and r.get("waiting_count_n") not in ("", None)
                            for r in results
                        )
                        else None
                    ),
                }
            )
    _write_csv(
        out_dir / "ev_matrix_summary.csv",
        summary_rows,
        [
            "density_label",
            "mode",
            "n_runs",
            "n",
            "n_arrived",
            "n_censored",
            "mean_s",
            "std_s",
            "min_s",
            "max_s",
            "waiting_time_mean_s",
            "time_loss_mean_s",
            "stop_time_mean_s",
            "waiting_count_mean_n",
        ],
    )

    if not args.dry_run:
        _plot_results(
            results=results,
            out_png=out_dir / "ev_matrix_plot.png",
            out_svg=out_dir / "ev_matrix_plot.svg",
        )

    meta = {
        "generated_at_epoch": float(time.time()),
        "manifest_csv": str(manifest_csv),
        "sim_root": str(sim_root),
        "real_world_script": str(rw_script),
        "base_sumocfg": str(base_sumocfg),
        "net_file": str(net_file),
        "modes": list(modes),
        "launcher_script_fingerprint": launcher_fp,
        "real_world_script_fingerprint": real_world_fp,
        "base_sumocfg_fingerprint": base_sumocfg_fp,
        "net_file_fingerprint": net_file_fp,
        "common_args": common_args_fp,
    }
    (out_dir / "run_meta.json").write_text(json.dumps(meta, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    total_elapsed_s = time.time() - matrix_t0
    print(f"[launch_matrix] done. results: {out_dir / 'ev_matrix_results.csv'} elapsed_s={total_elapsed_s:.2f}")


if __name__ == "__main__":
    main()
