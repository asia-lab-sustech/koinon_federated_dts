#!/usr/bin/env python3
"""Compare EV matrix run fingerprints.

This is intentionally small and dependency-free. It compares the per-run
`run_context.json` files written by `launch_ev_matrix_experiments.py`, plus the
optional `unit_context*.json` written by `run_ev_matrix_with_fnm.py`.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def _get(obj: Dict[str, Any], dotted: str, default: Any = "") -> Any:
    cur: Any = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _find_run_context(path: Path) -> Path:
    if path.is_file():
        return path
    p = path / "run_context.json"
    if p.exists():
        return p
    hits = sorted(path.glob("**/run_context.json"))
    if not hits:
        raise SystemExit(f"No run_context.json found under: {path}")
    if len(hits) > 1:
        raise SystemExit(f"Multiple run_context.json files found under {path}; pass one explicitly")
    return hits[0]


def _maybe_unit_context(run_ctx: Dict[str, Any]) -> Dict[str, Any]:
    p = _get(run_ctx, "result.matrix_unit_context_json", "")
    if not p:
        p = _get(run_ctx, "outputs.matrix_unit_context_json", "")
    if p and Path(str(p)).exists():
        return _load_json(Path(str(p)))
    run_path = Path(str(_get(run_ctx, "outputs.tripinfo_xml.path", ""))).parent
    if run_path.exists():
        # Typical layout: scenario_runs/<id>/matrix_out_<mode>/runs/.../run_context.json
        for parent in run_path.parents:
            hits = sorted(parent.glob("unit_context*.json"))
            if hits:
                try:
                    return _load_json(hits[0])
                except Exception:
                    return {}
    return {}


def _print_diff(label: str, a: Any, b: Any) -> None:
    mark = "==" if a == b else "!="
    print(f"{mark} {label}: A={a!r} B={b!r}")


def _main() -> None:
    ap = argparse.ArgumentParser(description="Compare two EV matrix run fingerprints.")
    ap.add_argument("a", help="run_context.json or directory containing one")
    ap.add_argument("b", help="run_context.json or directory containing one")
    args = ap.parse_args()

    a_path = _find_run_context(Path(args.a).resolve())
    b_path = _find_run_context(Path(args.b).resolve())
    a = _load_json(a_path)
    b = _load_json(b_path)
    au = _maybe_unit_context(a)
    bu = _maybe_unit_context(b)

    print(f"A: {a_path}")
    print(f"B: {b_path}")
    print()

    keys: List[str] = [
        "scenario.scenario_id",
        "scenario.density_label",
        "scenario.density_count",
        "scenario.route_id",
        "scenario.ev_id",
        "mode",
        "result.trip.travel_time_s",
        "result.trip.waiting_time_s",
        "result.trip.time_loss_s",
        "result.trip.stop_time_s",
        "result.return_code",
        "command.argv_sha256_12",
        "inputs.route_file.sha256_12",
        "inputs.sumocfg_variant.sha256_12",
        "inputs.real_world_script.sha256_12",
        "runtime.ev_http_port",
        "runtime.mqtt_host",
        "runtime.mqtt_port",
        "runtime.mqtt_topic_namespace",
        "outputs.event_summary.event_jsonl_fingerprint.sha256_12",
        "outputs.event_summary.event_jsonl_lines",
        "outputs.event_summary.b1_apply_n",
        "outputs.event_summary.b1_downstream_blockage_n",
        "outputs.event_summary.f2_apply_n",
        "outputs.event_summary.f2_apply_skipped_n",
        "outputs.event_summary.f2_guard_n",
        "outputs.event_summary.service_window_missed_n",
        "outputs.event_summary.service_window_stop_wait_n",
    ]
    for key in keys:
        _print_diff(key, _get(a, key), _get(b, key))

    if au or bu:
        print()
        print("Sidecar Unit Context")
        unit_keys: Iterable[str] = [
            "unit.unit_label",
            "unit.topic_namespace",
            "runtime.ev_http_port",
            "timing.sidecar_ready_delay_s",
            "timing.launch_wall_elapsed_s",
            "outputs.sidecar_summary.totals.fnm_mqtt_connect_fail_n",
            "outputs.sidecar_summary.totals.fnm_mqtt_connected_n",
            "outputs.sidecar_summary.totals.fnm_state_pull_ok_n",
            "outputs.sidecar_summary.totals.fnm_state_pull_error_n",
            "outputs.sidecar_summary.totals.fnm_ev_request_publish_n",
            "outputs.sidecar_summary.totals.fcm_peer_set_update_n",
        ]
        for key in unit_keys:
            _print_diff(key, _get(au, key), _get(bu, key))
        a_ev = _get(au, "outputs.sidecar_summary.by_name", {}).get("fnm_ev.jsonl", {})
        b_ev = _get(bu, "outputs.sidecar_summary.by_name", {}).get("fnm_ev.jsonl", {})
        for key in [
            "fingerprint.sha256_12",
            "lines",
            "fnm_mqtt_connect_fail_n",
            "fnm_state_pull_ok_n",
            "fnm_ev_request_publish_n",
        ]:
            _print_diff(f"ev_sidecar.{key}", _get(a_ev, key), _get(b_ev, key))


if __name__ == "__main__":
    _main()
