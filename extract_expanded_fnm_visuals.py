#!/usr/bin/env python3
"""Extract expanded FNM/federation metrics from EV matrix run folders.

This script reads one or more run roots that contain:
  - ev_matrix_results.csv
  - scenario_runs/<scenario_id>/matrix_out/runs/<density>/route_<n>/<mode>/fed_outcomes_<mode>_*.events.jsonl
  - scenario_runs/<scenario_id>/fnm_sidecars/*.jsonl

Outputs normalized CSVs under --out-dir for plotting:
  - travel_times.csv
  - latency_req_resp_samples.csv
  - latency_req_decision_samples.csv
  - latency_req_actuation_samples.csv
  - compute_duration_samples.csv
  - e2e_overhead_segments_route.csv
  - e2e_overhead_segments_node.csv
  - staleness_samples.csv
  - ev_request_samples.csv
  - decision_samples.csv
  - queue_spillback_samples.csv
  - queue_timeseries.csv
  - artifact_volume.csv
  - artifact_volume_by_node.csv
  - event_counts.csv
  - fnm_event_counts.csv
  - fnm_latency_samples.csv
  - timeline_events.csv
  - mode_route_summary.csv
  - node_cross_samples.csv
  - coordination_metrics.csv
  - fnm_processing_ratio.csv
  - artifact_burst_timeseries.csv
  - fnm_micro_latency_samples.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


SCENARIO_ROUTE_RE = re.compile(r"_r(\d+)$")
NODE_FILE_RE = re.compile(r"fnm_(Node[^.]+)")


def _to_float(v: object, default: Optional[float] = None) -> Optional[float]:
    if v is None:
        return default
    try:
        return float(v)
    except Exception:
        return default


def _to_int(v: object, default: Optional[int] = None) -> Optional[int]:
    if v is None:
        return default
    try:
        return int(float(v))
    except Exception:
        return default


def _split_run_dirs(raw: str) -> List[Path]:
    out: List[Path] = []
    for part in (raw or "").split(","):
        p = Path(part.strip())
        if part.strip():
            out.append(p)
    return out


def _expand_run_dirs(roots: List[Path]) -> List[Path]:
    expanded: List[Path] = []
    for root in roots:
        if (root / "ev_matrix_results.csv").exists():
            expanded.append(root)
            continue
        children = sorted([p for p in root.iterdir() if p.is_dir() and (p / "ev_matrix_results.csv").exists()]) if root.exists() else []
        if children:
            expanded.extend(children)
        else:
            expanded.append(root)
    # Stable de-duplication.
    seen = set()
    out: List[Path] = []
    for p in expanded:
        sp = str(p)
        if sp not in seen:
            out.append(p)
            seen.add(sp)
    return out


def _dataset_label(root: Path) -> str:
    return root.name


def _read_csv_rows(path: Path) -> List[dict]:
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def _parse_route_from_row(row: dict) -> Optional[int]:
    sid = str(row.get("scenario_id", "") or "")
    m = SCENARIO_ROUTE_RE.search(sid)
    if m:
        return _to_int(m.group(1))
    return _to_int(row.get("route_id"))


def _resolve_mode_dir(root: Path, scenario_id: str, route_id: int, mode: str) -> Optional[Path]:
    base = root / "scenario_runs" / scenario_id / "matrix_out" / "runs"
    if not base.exists():
        return None
    density_dirs = sorted([p for p in base.iterdir() if p.is_dir()])
    if not density_dirs:
        return None
    route_dir = density_dirs[0] / f"route_{int(route_id)}"
    if not route_dir.exists():
        # fallback for unexpected numbering
        candidates = sorted([p for p in density_dirs[0].iterdir() if p.is_dir() and p.name.startswith("route_")])
        if not candidates:
            return None
        route_dir = candidates[0]
    mode_dir = route_dir / str(mode)
    return mode_dir if mode_dir.exists() else None


def _resolve_event_file(mode_dir: Path, mode: str) -> Optional[Path]:
    if mode_dir is None:
        return None
    specific = sorted(mode_dir.glob(f"fed_outcomes_{mode}_*.events.jsonl"))
    if specific:
        return specific[-1]
    generic = mode_dir / "fed_outcomes.events.jsonl"
    if generic.exists():
        return generic
    any_evt = sorted(mode_dir.glob("fed_outcomes_*.events.jsonl"))
    return any_evt[-1] if any_evt else None


def _iter_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            try:
                j = json.loads(s)
            except Exception:
                continue
            yield j


def _event_type(j: dict) -> Optional[str]:
    et = j.get("event_type")
    if et is None:
        et = j.get("event")
    if et is None:
        return None
    return str(et)


def _event_ts(j: dict) -> Optional[float]:
    # Prefer wall timestamp to derive cross-event latencies.
    t = _to_float(j.get("ts_wall"))
    if t is not None:
        return t
    return _to_float(j.get("sim_time"))


def _artifact_family(et: str) -> str:
    if et == "ev.request.in":
        return "request_response"
    if et.startswith("coord.reservation."):
        return "request_response"
    if et.startswith("coord.refine.") or et.startswith("coord.apply.") or et.startswith("agent.stage."):
        return "coordination"
    if et.startswith("tls.signal."):
        return "event"
    if et.startswith("membership.") or et.startswith("catalog.") or et.startswith("discovery."):
        return "state"
    if ".state" in et or et.endswith(".state"):
        return "state"
    if ".event" in et or et.endswith(".event"):
        return "event"
    return "other"


def _mkdir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _mean(vals: List[float]) -> Optional[float]:
    if not vals:
        return None
    return float(sum(vals) / len(vals))


def _safe_ratio(num: float, den: float) -> float:
    if den <= 0.0:
        return math.nan
    return float(num / den)


def _classify_artifact(topic: str, evt: str) -> str:
    s = f"{(topic or '').lower()} {(evt or '').lower()}"
    if ("ev/request" in s) or ("reservation" in s) or ("req_" in s) or ("request" in s):
        return "request_response"
    if ("coord" in s) or ("warmup" in s) or ("proposal" in s) or ("decision" in s):
        return "coordination"
    if ("state" in s) or ("catalog" in s) or ("membership" in s) or ("discovery" in s):
        return "state"
    if ("event" in s) or ("signal" in s) or ("phase" in s):
        return "event"
    return "other"


def _write_csv(path: Path, rows: List[dict], fieldnames: List[str]) -> None:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _node_id_from_source_file(name: str, component: str) -> str:
    s = str(name or "")
    m = NODE_FILE_RE.search(s)
    if m:
        return str(m.group(1))
    if str(component or "") == "ev":
        return "EV"
    return "unknown"


def main() -> None:
    ap = argparse.ArgumentParser(description="Extract expanded FNM/federation metrics CSVs from run folders.")
    ap.add_argument(
        "--run-dirs",
        required=True,
        help="Comma-separated run roots (e.g. .../ev_matrix_runs_stress_measured_short_clean_1K,...)",
    )
    ap.add_argument("--out-dir", required=True, help="Output directory for extracted CSV files.")
    ap.add_argument("--timeline-dataset", default="", help="Dataset label for timeline extraction (optional).")
    ap.add_argument("--timeline-scenario", default="", help="Scenario id for timeline extraction (optional).")
    ap.add_argument("--timeline-route", type=int, default=-1, help="Route id for timeline extraction (optional).")
    ap.add_argument("--timeline-mode", default="F2", help="Mode for timeline extraction (default: F2).")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    run_dirs = _expand_run_dirs(_split_run_dirs(args.run_dirs))
    if not run_dirs:
        raise SystemExit("No valid --run-dirs provided.")

    # If caller passes a wrapper seed folder label (e.g. simulations_20260367)
    # while extractor labels are child run-folder names, fall back to no dataset
    # filter for timeline extraction to avoid empty timeline rows.
    timeline_dataset_filter = str(args.timeline_dataset or "").strip()
    dataset_labels = {_dataset_label(p) for p in run_dirs if (p / "ev_matrix_results.csv").exists()}
    if timeline_dataset_filter and timeline_dataset_filter not in dataset_labels:
        if args.verbose:
            print(
                f"[warn] --timeline-dataset='{timeline_dataset_filter}' not found in datasets={sorted(dataset_labels)}; ignoring dataset filter."
            )
        timeline_dataset_filter = ""

    out_dir = Path(args.out_dir)
    _mkdir(out_dir)

    travel_rows: List[dict] = []
    lat_rr_rows: List[dict] = []
    lat_rd_rows: List[dict] = []
    lat_ra_rows: List[dict] = []
    compute_rows: List[dict] = []
    stale_rows: List[dict] = []
    ev_req_rows: List[dict] = []
    decision_rows: List[dict] = []
    node_cross_rows: List[dict] = []
    coord_metrics_rows: List[dict] = []
    queue_rows: List[dict] = []
    queue_ts_rows: List[dict] = []
    artifact_rows: List[dict] = []
    artifact_node_counter: Counter = Counter()
    event_count_rows: List[dict] = []
    fnm_event_rows: List[dict] = []
    fnm_lat_rows: List[dict] = []
    fnm_proc_rows: List[dict] = []
    fnm_processing_ratio_rows: List[dict] = []
    fnm_micro_rows: List[dict] = []
    timeline_rows: List[dict] = []
    summary_rows: List[dict] = []
    burst_counter: Counter = Counter()

    for root in run_dirs:
        dataset = _dataset_label(root)
        res_csv = root / "ev_matrix_results.csv"
        if not res_csv.exists():
            if args.verbose:
                print(f"[skip] missing {res_csv}")
            continue
        res_rows = _read_csv_rows(res_csv)
        if args.verbose:
            print(f"[dataset] {dataset}: rows={len(res_rows)}")

        # Store travel/per-run KPI rows.
        for r in res_rows:
            route = _parse_route_from_row(r)
            if route is None:
                continue
            travel_rows.append(
                {
                    "dataset": dataset,
                    "scenario_id": str(r.get("scenario_id", "") or ""),
                    "density_label": str(r.get("density_label", "") or ""),
                    "density_count": _to_int(r.get("density_count"), default=None),
                    "route_id": route,
                    "mode": str(r.get("mode", "") or ""),
                    "ev_id": str(r.get("ev_id", "") or ""),
                    "travel_time_s": _to_float(r.get("travel_time_s"), default=None),
                    "waiting_time_s": _to_float(r.get("waiting_time_s"), default=None),
                    "time_loss_s": _to_float(r.get("time_loss_s"), default=None),
                    "stop_time_s": _to_float(r.get("stop_time_s"), default=None),
                    "wall_elapsed_s": _to_float(r.get("wall_elapsed_s"), default=None),
                    "arrived": _to_int(r.get("arrived"), default=None),
                    "return_code": _to_int(r.get("return_code"), default=None),
                    "http_precheck_ok": _to_int(r.get("http_precheck_ok"), default=None),
                    "http_startup_ok": _to_int(r.get("http_startup_ok"), default=None),
                    "foreign_ev_drop_fail": _to_int(r.get("foreign_ev_drop_fail"), default=None),
                }
            )

        # Parse per-mode events for each run row.
        # Keyed by scenario+mode, avoid duplicate parsing if rows are repeated.
        parsed_mode_key: set = set()

        # For summary table per dataset/route/mode.
        by_mode_route_tt: Dict[Tuple[str, int], float] = {}
        for r in res_rows:
            route = _parse_route_from_row(r)
            mode = str(r.get("mode", "") or "")
            tt = _to_float(r.get("travel_time_s"), default=None)
            if route is not None and mode and tt is not None:
                by_mode_route_tt[(mode, route)] = tt

        for r in res_rows:
            scenario_id = str(r.get("scenario_id", "") or "")
            mode = str(r.get("mode", "") or "")
            route = _parse_route_from_row(r)
            if not scenario_id or not mode or route is None:
                continue

            pk = (dataset, scenario_id, route, mode)
            if pk in parsed_mode_key:
                continue
            parsed_mode_key.add(pk)

            mode_dir = _resolve_mode_dir(root, scenario_id, route, mode)
            if mode_dir is None:
                continue
            evt_file = _resolve_event_file(mode_dir, mode)
            if evt_file is None:
                continue

            req_out_ts: Dict[str, float] = {}
            pending_req_for_apply: Dict[Tuple[str, str], List[float]] = defaultdict(list)
            mode_event_counter: Counter = Counter()
            mode_art_counter: Counter = Counter()
            mode_decision_counter: Counter = Counter()
            mode_req_decision_latency_ms: List[float] = []
            mode_cross_points: List[Tuple[float, Optional[float], str, str]] = []
            mode_coord_tls: set = set()
            mode_accepted_nonspill = 0

            for j in _iter_jsonl(evt_file):
                et = _event_type(j)
                if not et:
                    continue
                mode_event_counter[et] += 1
                fam = _artifact_family(et)
                mode_art_counter[fam] += 1
                ts = _event_ts(j)
                tls_id = str(j.get("tls_id", "") or "")
                ev_id = str(j.get("ev_id", "") or "")
                sim_time = _to_float(j.get("sim_time"), default=None)
                if tls_id:
                    artifact_node_counter[(dataset, scenario_id, route, mode, tls_id, fam)] += 1
                if sim_time is not None:
                    burst_counter[(dataset, scenario_id, route, mode, int(round(float(sim_time))), fam)] += 1

                if et == "ev.request.in":
                    ev_req_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "sim_time": sim_time,
                            "distance_to_intersection_m": _to_float(j.get("distance_to_intersection_m"), default=None),
                            "speed_mps": _to_float(j.get("speed_mps"), default=None),
                            "request_age_ms": _to_float(j.get("request_age_ms"), default=None),
                            "source_tag": str(j.get("ev_request_source_tag", "") or ""),
                            "delivery": str(j.get("ev_request_delivery", "") or ""),
                        }
                    )
                    age = _to_float(j.get("request_age_ms"), default=None)
                    if age is not None:
                        stale_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "staleness_type": "ev_request_age_ms",
                                "value_ms": age,
                                "tls_id": tls_id,
                                "ev_id": ev_id,
                                "sim_time": sim_time,
                            }
                        )

                elif et == "coord.reservation.req_out":
                    req_id = str(j.get("req_id", "") or "")
                    if req_id and ts is not None:
                        req_out_ts[req_id] = ts
                    if req_id and ts is not None and ev_id and tls_id:
                        pending_req_for_apply[(tls_id, ev_id)].append(ts)
                    if tls_id:
                        mode_coord_tls.add(tls_id)

                elif et == "coord.reservation.req_resp_e2e":
                    lat = _to_float(j.get("latency_ms"), default=None)
                    age = _to_float(j.get("responder_phase_state_age_ms"), default=None)
                    lat_rr_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "from_tls": str(j.get("from_tls", "") or ""),
                            "req_id": str(j.get("req_id", "") or ""),
                            "sim_time": sim_time,
                            "latency_ms": lat,
                            "responder_phase_state_age_ms": age,
                            "status": str(j.get("status", "") or ""),
                        }
                    )
                    if age is not None:
                        stale_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "staleness_type": "responder_phase_state_age_ms",
                                "value_ms": age,
                                "tls_id": tls_id,
                                "ev_id": ev_id,
                                "sim_time": sim_time,
                            }
                        )

                elif et == "coord.reservation.req_decision":
                    req_id = str(j.get("req_id", "") or "")
                    status = str(j.get("status", "") or "")
                    reason = str(j.get("reason", "") or "")
                    mode_decision_counter[status] += 1
                    decision_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "req_id": req_id,
                            "status": status,
                            "reason": reason,
                            "sim_time": sim_time,
                            "q_margin_sec": _to_float(j.get("q_margin_sec"), default=None),
                            "spillback_risk": _to_float(j.get("spillback_risk"), default=None),
                            "readiness_score": _to_float(j.get("readiness_score"), default=None),
                            "active_reservations": _to_int(j.get("active_reservations"), default=None),
                        }
                    )
                    queue_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "sim_time": sim_time,
                            "q_margin_sec": _to_float(j.get("q_margin_sec"), default=None),
                            "spillback_risk": _to_float(j.get("spillback_risk"), default=None),
                            "decision_status": status,
                            "decision_reason": reason,
                        }
                    )
                    sp = _to_float(j.get("spillback_risk"), default=None)
                    qm = _to_float(j.get("q_margin_sec"), default=None)
                    if status.upper() == "ACCEPTED":
                        if (sp is None or sp < 0.5) and (qm is None or qm > 0.0):
                            mode_accepted_nonspill += 1
                        if tls_id:
                            mode_coord_tls.add(tls_id)
                    if req_id and ts is not None and req_id in req_out_ts:
                        d_ms = max(0.0, (ts - req_out_ts[req_id]) * 1000.0)
                        mode_req_decision_latency_ms.append(d_ms)
                        lat_rd_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "ev_id": ev_id,
                                "tls_id": tls_id,
                                "req_id": req_id,
                                "sim_time": sim_time,
                                "latency_ms": d_ms,
                                "status": status,
                                "reason": reason,
                            }
                        )

                elif et == "coord.queue.snapshot":
                    queue_ts_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "sim_time": sim_time,
                            "in_edge_id": str(j.get("in_edge_id", "") or ""),
                            "queue_len_est_veh": _to_float(j.get("queue_len_est_veh"), default=None),
                            "queue_clear_time_sec": _to_float(j.get("queue_clear_time_sec"), default=None),
                            "queue_margin_sec": _to_float(j.get("queue_margin_sec"), default=None),
                            "spillback_risk": _to_float(j.get("spillback_risk"), default=None),
                            "spillback_active": _to_int(j.get("spillback_active"), default=None),
                            "spillback_threshold": _to_float(j.get("spillback_threshold"), default=None),
                            "readiness_score": _to_float(j.get("readiness_score"), default=None),
                            "eta_mid_sec": _to_float(j.get("eta_mid_sec"), default=None),
                            "eta_start_sec": _to_float(j.get("eta_start_sec"), default=None),
                            "eta_end_sec": _to_float(j.get("eta_end_sec"), default=None),
                            "ev_distance_m": _to_float(j.get("ev_distance_m"), default=None),
                            "ev_speed_mps": _to_float(j.get("ev_speed_mps"), default=None),
                        }
                    )

                elif et == "coord.apply.plan":
                    # Approximate request->actuation from last pending req_out at same (tls, ev).
                    if ts is not None and tls_id and ev_id:
                        k = (tls_id, ev_id)
                        if pending_req_for_apply.get(k):
                            t0 = pending_req_for_apply[k].pop(0)
                            d_ms = max(0.0, (ts - t0) * 1000.0)
                            lat_ra_rows.append(
                                {
                                    "dataset": dataset,
                                    "scenario_id": scenario_id,
                                    "route_id": route,
                                    "mode": mode,
                                    "ev_id": ev_id,
                                    "tls_id": tls_id,
                                    "sim_time": sim_time,
                                    "latency_ms": d_ms,
                                    "decision_source": str(j.get("decision_source", "") or ""),
                                    "plan_type": str(j.get("plan_type", "") or ""),
                                }
                            )

                elif et == "intersection.compute.tick.duration_ms" or et == "intersection.compute.apply.duration_ms":
                    d_ms = _to_float(j.get("duration_ms"), default=None)
                    if d_ms is None:
                        d_ms = _to_float(j.get("value_ms"), default=None)
                    if d_ms is not None:
                        stage = "local_compute" if "tick" in et else "local_apply"
                        compute_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "ev_id": ev_id,
                                "tls_id": tls_id,
                                "sim_time": sim_time,
                                "stage": stage,
                                "duration_ms": d_ms,
                            }
                        )

                elif et == "ev.node.cross" or et == "ev.pass.detected":
                    wait_s = _to_float(j.get("vehicle_waiting_time_s"), default=None)
                    if wait_s is None:
                        wait_s = _to_float(j.get("waiting_time_s"), default=None)
                    if wait_s is None:
                        wait_s = _to_float(j.get("ev_waiting_time_s"), default=None)
                    node_cross_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "ev_id": ev_id,
                            "tls_id": tls_id,
                            "sim_time": sim_time,
                            "event_type": et,
                            "vehicle_waiting_time_s": wait_s,
                            "distance_to_intersection_m": _to_float(j.get("distance_to_intersection_m"), default=None),
                            "speed_mps": _to_float(j.get("speed_mps"), default=None),
                        }
                    )
                    if sim_time is not None:
                        mode_cross_points.append((float(sim_time), wait_s, tls_id, ev_id))

                if et in {
                    "coord.refine.selection_compare",
                    "coord.refine.selection_final",
                    "coord.refine.fallback_local.no_recent_feedback_near",
                    "coord.refine.fallback_local.stale_feedback_near",
                    "coord.refine.state_assisted_refine",
                }:
                    fb_age_sec = _to_float(j.get("feedback_age_sec"), default=None)
                    if fb_age_sec is not None and fb_age_sec >= 0.0:
                        stale_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "staleness_type": "feedback_age_ms",
                                "value_ms": fb_age_sec * 1000.0,
                                "tls_id": tls_id,
                                "ev_id": ev_id,
                                "sim_time": sim_time,
                            }
                        )
                    ns_age = _to_float(j.get("neighbor_state_phase_state_age_ms"), default=None)
                    if ns_age is not None and ns_age >= 0.0:
                        stale_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route,
                                "mode": mode,
                                "staleness_type": "neighbor_state_phase_state_age_ms",
                                "value_ms": ns_age,
                                "tls_id": tls_id,
                                "ev_id": ev_id,
                                "sim_time": sim_time,
                            }
                        )

                # Build timeline rows for selected representative case.
                want_timeline = True
                if timeline_dataset_filter and dataset != timeline_dataset_filter:
                    want_timeline = False
                if args.timeline_scenario and scenario_id != args.timeline_scenario:
                    want_timeline = False
                if args.timeline_route >= 0 and route != int(args.timeline_route):
                    want_timeline = False
                if args.timeline_mode and mode != args.timeline_mode:
                    want_timeline = False
                if want_timeline and et in {
                    "ev.request.in",
                    "coord.refine.candidates",
                    "coord.refine.selection_final",
                    "coord.reservation.req_in",
                    "coord.reservation.req_out",
                    "coord.reservation.req_decision",
                    "coord.reservation.resp_in",
                    "coord.reservation.req_resp_e2e",
                    "coord.apply.plan",
                    "coord.apply.plan_skip",
                    "tls.signal.change",
                    "ev.pass.detected",
                    "ev.node.cross",
                }:
                    timeline_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route,
                            "mode": mode,
                            "sim_time": sim_time,
                            "tls_id": tls_id,
                            "ev_id": ev_id,
                            "event_type": et,
                            "decision_source": str(j.get("decision_source", "") or ""),
                            "status": str(j.get("status", "") or ""),
                            "reason": str(j.get("reason", "") or ""),
                            "plan_type": str(j.get("plan_type", "") or ""),
                            "signal_state": str(j.get("signal_state", "") or ""),
                            "phase_idx": _to_int(j.get("phase_idx"), default=None),
                            "distance_to_intersection_m": _to_float(j.get("distance_to_intersection_m"), default=None),
                            "speed_mps": _to_float(j.get("speed_mps"), default=None),
                            "request_age_ms": _to_float(j.get("request_age_ms"), default=None),
                        }
                    )

            # Save counters per mode/route.
            for et, cnt in sorted(mode_event_counter.items()):
                event_count_rows.append(
                    {
                        "dataset": dataset,
                        "scenario_id": scenario_id,
                        "route_id": route,
                        "mode": mode,
                        "event_type": et,
                        "count": int(cnt),
                    }
                )
            for fam, cnt in sorted(mode_art_counter.items()):
                artifact_rows.append(
                    {
                        "dataset": dataset,
                        "scenario_id": scenario_id,
                        "route_id": route,
                        "mode": mode,
                        "artifact_family": fam,
                        "count": int(cnt),
                    }
                )

            accepted = int(mode_decision_counter.get("ACCEPTED", 0))
            rejected = int(mode_decision_counter.get("REJECTED", 0))
            total_dec = accepted + rejected
            succ = (float(accepted) / float(total_dec)) if total_dec > 0 else math.nan
            summary_rows.append(
                {
                    "dataset": dataset,
                    "scenario_id": scenario_id,
                    "route_id": route,
                    "mode": mode,
                    "travel_time_s": by_mode_route_tt.get((mode, route)),
                    "decision_accepted": accepted,
                    "decision_rejected": rejected,
                    "decision_total": total_dec,
                    "coordination_success_rate": succ,
                    "req_out_count": int(mode_event_counter.get("coord.reservation.req_out", 0)),
                    "req_resp_e2e_count": int(mode_event_counter.get("coord.reservation.req_resp_e2e", 0)),
                    "apply_plan_count": int(mode_event_counter.get("coord.apply.plan", 0)),
                    "apply_plan_skip_count": int(mode_event_counter.get("coord.apply.plan_skip", 0)),
                }
            )

            # O + P + Q coordination metrics.
            requests_in = int(mode_event_counter.get("ev.request.in", 0))
            decisions_total = int(mode_decision_counter.get("ACCEPTED", 0) + mode_decision_counter.get("REJECTED", 0))
            proposals = int(mode_event_counter.get("coord.reservation.req_out", 0))
            accepted_nonspill = int(mode_accepted_nonspill)
            req_handled_rate = _safe_ratio(float(decisions_total), float(requests_in)) if requests_in > 0 else math.nan
            proposal_rate = _safe_ratio(float(proposals), float(requests_in)) if requests_in > 0 else math.nan
            downstream_prep_rate = _safe_ratio(float(accepted_nonspill), float(decisions_total)) if decisions_total > 0 else math.nan

            mode_cross_points.sort(key=lambda x: x[0])
            no_stop_count = 0
            observed_wait_count = 0
            streak = 0
            max_streak = 0
            for _, w_s, _, _ in mode_cross_points:
                if w_s is None:
                    continue
                observed_wait_count += 1
                if float(w_s) <= 0.5:
                    no_stop_count += 1
                    streak += 1
                    if streak > max_streak:
                        max_streak = streak
                else:
                    streak = 0
            ev_clear_rate = _safe_ratio(float(no_stop_count), float(observed_wait_count)) if observed_wait_count > 0 else math.nan
            components = [x for x in [req_handled_rate, proposal_rate, downstream_prep_rate, ev_clear_rate] if not math.isnan(x)]
            success_proxy = (100.0 * float(sum(components) / len(components))) if components else math.nan

            mode_req_decision_latency_ms_sorted = sorted(mode_req_decision_latency_ms)
            lat_med_ms = None
            if mode_req_decision_latency_ms_sorted:
                nlat = len(mode_req_decision_latency_ms_sorted)
                mid = nlat // 2
                if nlat % 2 == 1:
                    lat_med_ms = float(mode_req_decision_latency_ms_sorted[mid])
                else:
                    lat_med_ms = 0.5 * float(mode_req_decision_latency_ms_sorted[mid - 1] + mode_req_decision_latency_ms_sorted[mid])

            coord_metrics_rows.append(
                {
                    "dataset": dataset,
                    "scenario_id": scenario_id,
                    "route_id": route,
                    "mode": mode,
                    "requests_in": requests_in,
                    "proposals_out": proposals,
                    "decisions_total": decisions_total,
                    "accepted_nonspill": accepted_nonspill,
                    "req_handled_rate_pct": (100.0 * req_handled_rate) if not math.isnan(req_handled_rate) else math.nan,
                    "proposal_before_expiry_rate_pct": (100.0 * proposal_rate) if not math.isnan(proposal_rate) else math.nan,
                    "downstream_prep_rate_pct": (100.0 * downstream_prep_rate) if not math.isnan(downstream_prep_rate) else math.nan,
                    "ev_clear_no_spill_rate_pct": (100.0 * ev_clear_rate) if not math.isnan(ev_clear_rate) else math.nan,
                    "coordination_success_proxy_pct": success_proxy,
                    "request_to_decision_median_ms": lat_med_ms,
                    "coordinated_intersections_per_trip": int(len(mode_coord_tls)),
                    "consecutive_intersections_no_stop_max": int(max_streak),
                    "node_cross_samples_with_wait": int(observed_wait_count),
                }
            )

        # Parse FNM sidecar logs (scenario-level).
        for sc_dir in sorted((root / "scenario_runs").glob("*")):
            if not sc_dir.is_dir():
                continue
            scenario_id = sc_dir.name
            route_guess = None
            m = SCENARIO_ROUTE_RE.search(scenario_id)
            if m:
                route_guess = _to_int(m.group(1), default=None)
            fnm_dir = sc_dir / "fnm_sidecars"
            if not fnm_dir.exists():
                continue
            for jf in sorted(fnm_dir.glob("*.jsonl")):
                comp = "ev" if jf.name.startswith("fnm_ev") else ("intersection" if jf.name.startswith("fnm_Node") else "other")
                node_id = _node_id_from_source_file(jf.name, comp)
                local_counter: Counter = Counter()
                for j in _iter_jsonl(jf):
                    evt = str(j.get("event", "") or j.get("event_type", "") or "")
                    if not evt:
                        continue
                    local_counter[evt] += 1
                    status = str(j.get("status", "") or "")
                    src = str(j.get("src", "") or j.get("source_topic", "") or "")
                    dst = str(j.get("dst", "") or j.get("publish_topic", "") or j.get("topic", "") or "")
                    topic = src if src else dst
                    artifact = _classify_artifact(topic=topic, evt=evt)
                    stage = ""
                    if evt.startswith("fnm.route.") or evt == "fnm.adapter.state_pull.ok":
                        stage = "received"
                    elif evt.startswith("fnm.stage."):
                        stage = "translated"
                    elif evt.startswith("fnm.delivery.") or evt == "fnm.adapter.ev_request.publish":
                        stage = "accepted" if (not status or status == "success") else "deferred_rejected"
                    elif ".drop" in evt or "error" in evt or status in {"publish_error", "drop", "error"}:
                        stage = "deferred_rejected"
                    if stage:
                        fnm_proc_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route_guess,
                                "component": comp,
                                "event": evt,
                                "status": status,
                                "artifact_type": artifact,
                                "stage": stage,
                            }
                        )

                    # Optional latency fields if available in future runs.
                    lat = None
                    for k in ("latency_ms", "e2e_latency_ms", "pull_latency_ms", "processing_latency_ms", "duration_ms"):
                        if k in j:
                            lat = _to_float(j.get(k), default=None)
                            if lat is not None:
                                break
                    if lat is not None:
                        fnm_lat_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route_guess,
                                "component": comp,
                                "source_file": jf.name,
                                "event": evt,
                                "latency_ms": lat,
                            }
                        )
                    if evt in {
                        "fnm.stage.local_to_fed",
                        "fnm.stage.fed_to_local",
                        "fnm.route.local_to_fed",
                        "fnm.route.fed_to_local",
                    }:
                        schema_ms = None
                        routing_ms = None
                        network_ms = None
                        total_ms = None
                        if evt == "fnm.stage.local_to_fed":
                            schema_ms = _to_float(j.get("local_ingest_to_schema_ms"), default=None)
                            routing_ms = _to_float(j.get("schema_to_fed_publish_ms"), default=None)
                            total_ms = _to_float(j.get("local_to_fed_total_ms"), default=None)
                            if routing_ms is None and total_ms is not None and schema_ms is not None:
                                routing_ms = max(0.0, float(total_ms) - float(schema_ms))
                        else:
                            if evt == "fnm.stage.fed_to_local":
                                # Keep network transport isolated; exclude from micro-latency stacks by default.
                                schema_ms = _to_float(j.get("remote_receive_to_local_invoke_ms"), default=None)
                                routing_ms = 0.0
                                network_ms = _to_float(j.get("fed_publish_to_remote_receive_ms"), default=None)
                                total_ms = _to_float(j.get("fed_to_local_total_ms"), default=None)
                            else:
                                # Route-level duration is a useful fallback for orchestration cost.
                                routing_ms = _to_float(j.get("duration_ms"), default=None)
                                schema_ms = 0.0
                                total_ms = routing_ms
                        fnm_micro_rows.append(
                            {
                                "dataset": dataset,
                                "scenario_id": scenario_id,
                                "route_id": route_guess,
                                "component": comp,
                                "node_id": node_id,
                                "source_file": jf.name,
                                "event": evt,
                                "artifact_kind": str(j.get("artefact_kind", "") or ""),
                                "schema_protocol_ms": schema_ms,
                                "routing_orchestration_ms": routing_ms,
                                "network_transport_ms": network_ms,
                                "total_stage_ms": total_ms,
                                "sim_time": _to_float(j.get("sim_time"), default=None),
                            }
                        )
                for evt, cnt in sorted(local_counter.items()):
                    fnm_event_rows.append(
                        {
                            "dataset": dataset,
                            "scenario_id": scenario_id,
                            "route_id": route_guess,
                            "component": comp,
                            "source_file": jf.name,
                            "event": evt,
                            "count": int(cnt),
                        }
                    )

    # Build explicit E2E-overhead segment tables (route and node level) for stacked bars.
    route_compute: Dict[Tuple[str, str, int, str], List[float]] = defaultdict(list)
    route_apply: Dict[Tuple[str, str, int, str], List[float]] = defaultdict(list)
    route_coord: Dict[Tuple[str, str, int, str], List[float]] = defaultdict(list)
    node_compute: Dict[Tuple[str, str, int, str, str], List[float]] = defaultdict(list)
    node_apply: Dict[Tuple[str, str, int, str, str], List[float]] = defaultdict(list)
    node_coord: Dict[Tuple[str, str, int, str, str], List[float]] = defaultdict(list)
    route_fnm: Dict[Tuple[str, str, int], List[float]] = defaultdict(list)
    route_overhead_rows: List[dict] = []
    node_overhead_rows: List[dict] = []

    for r in compute_rows:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _to_int(r.get("route_id"))
        md = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "unknown")
        st = str(r.get("stage", "") or "")
        d_ms = _to_float(r.get("duration_ms"), default=None)
        if not ds or not sc or rt is None or not md or d_ms is None:
            continue
        rk = (ds, sc, rt, md)
        nk = (ds, sc, rt, md, tls)
        if st == "local_compute":
            route_compute[rk].append(d_ms)
            node_compute[nk].append(d_ms)
        elif st == "local_apply":
            route_apply[rk].append(d_ms)
            node_apply[nk].append(d_ms)

    for r in lat_rr_rows:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _to_int(r.get("route_id"))
        md = str(r.get("mode", "") or "")
        tls = str(r.get("tls_id", "") or "unknown")
        d_ms = _to_float(r.get("latency_ms"), default=None)
        if not ds or not sc or rt is None or not md or d_ms is None:
            continue
        route_coord[(ds, sc, rt, md)].append(d_ms)
        node_coord[(ds, sc, rt, md, tls)].append(d_ms)

    fnm_events_for_latency = {
        "fnm.route.local_to_fed",
        "fnm.route.fed_to_local",
        "fnm.delivery.local_to_fed",
        "fnm.delivery.fed_to_local",
        "fnm.stage.local_to_fed",
        "fnm.stage.fed_to_local",
        "fnm.adapter.state_pull.ok",
        "fnm.adapter.ev_request.publish",
    }
    for r in fnm_lat_rows:
        ds = str(r.get("dataset", "") or "")
        sc = str(r.get("scenario_id", "") or "")
        rt = _to_int(r.get("route_id"))
        evt = str(r.get("event", "") or "")
        d_ms = _to_float(r.get("latency_ms"), default=None)
        if not ds or not sc or rt is None or d_ms is None:
            continue
        if evt and evt not in fnm_events_for_latency:
            continue
        route_fnm[(ds, sc, rt)].append(d_ms)

    route_keys = sorted(set(route_compute.keys()) | set(route_apply.keys()) | set(route_coord.keys()))
    for ds, sc, rt, md in route_keys:
        c1 = route_compute.get((ds, sc, rt, md), [])
        c2 = route_apply.get((ds, sc, rt, md), [])
        c3 = route_coord.get((ds, sc, rt, md), [])
        c4 = route_fnm.get((ds, sc, rt), [])
        local_compute_ms = _mean(c1)
        local_apply_ms = _mean(c2)
        coordination_req_resp_ms = _mean(c3)
        fnm_mediation_ms = _mean(c4)
        total = (
            (local_compute_ms or 0.0)
            + (local_apply_ms or 0.0)
            + (coordination_req_resp_ms or 0.0)
            + (fnm_mediation_ms or 0.0)
        )
        route_overhead_rows.append(
            {
                "dataset": ds,
                "scenario_id": sc,
                "route_id": rt,
                "mode": md,
                "local_compute_ms": local_compute_ms,
                "local_apply_ms": local_apply_ms,
                "fnm_mediation_ms": fnm_mediation_ms,
                "coordination_req_resp_ms": coordination_req_resp_ms,
                "total_e2e_ms": total,
                "compute_samples": len(c1),
                "apply_samples": len(c2),
                "coord_samples": len(c3),
                "fnm_samples": len(c4),
            }
        )

    node_keys = sorted(set(node_compute.keys()) | set(node_apply.keys()) | set(node_coord.keys()))
    for ds, sc, rt, md, tls in node_keys:
        c1 = node_compute.get((ds, sc, rt, md, tls), [])
        c2 = node_apply.get((ds, sc, rt, md, tls), [])
        c3 = node_coord.get((ds, sc, rt, md, tls), [])
        c4 = route_fnm.get((ds, sc, rt), [])
        local_compute_ms = _mean(c1)
        local_apply_ms = _mean(c2)
        coordination_req_resp_ms = _mean(c3)
        fnm_mediation_ms = _mean(c4)
        total = (
            (local_compute_ms or 0.0)
            + (local_apply_ms or 0.0)
            + (coordination_req_resp_ms or 0.0)
            + (fnm_mediation_ms or 0.0)
        )
        node_overhead_rows.append(
            {
                "dataset": ds,
                "scenario_id": sc,
                "route_id": rt,
                "mode": md,
                "tls_id": tls,
                "local_compute_ms": local_compute_ms,
                "local_apply_ms": local_apply_ms,
                "fnm_mediation_ms": fnm_mediation_ms,
                "coordination_req_resp_ms": coordination_req_resp_ms,
                "total_e2e_ms": total,
                "compute_samples": len(c1),
                "apply_samples": len(c2),
                "coord_samples": len(c3),
                "fnm_samples": len(c4),
                        }
                    )

    # Aggregate N plot ratios by dataset + artifact_type from FNM processing rows.
    by_ds_art_stage: Dict[Tuple[str, str, str], int] = defaultdict(int)
    for r in fnm_proc_rows:
        ds = str(r.get("dataset", "") or "")
        art = str(r.get("artifact_type", "") or "")
        stage = str(r.get("stage", "") or "")
        if ds and art and stage:
            by_ds_art_stage[(ds, art, stage)] += 1
    datasets = sorted({k[0] for k in by_ds_art_stage.keys()})
    for ds in datasets:
        arts = sorted({k[1] for k in by_ds_art_stage.keys() if k[0] == ds})
        for art in arts:
            rec = int(by_ds_art_stage.get((ds, art, "received"), 0))
            trn = int(by_ds_art_stage.get((ds, art, "translated"), 0))
            acc = int(by_ds_art_stage.get((ds, art, "accepted"), 0))
            rej = int(by_ds_art_stage.get((ds, art, "deferred_rejected"), 0))
            den = max(1, rec)
            fnm_processing_ratio_rows.append(
                {
                    "dataset": ds,
                    "artifact_type": art,
                    "received": rec,
                    "translated": trn,
                    "accepted": acc,
                    "deferred_rejected": rej,
                    "received_pct": 100.0 if rec > 0 else 0.0,
                    "translated_pct": min(100.0, 100.0 * float(trn) / float(den)),
                    "accepted_pct": min(100.0, 100.0 * float(acc) / float(den)),
                    "deferred_rejected_pct": min(100.0, 100.0 * float(rej) / float(den)),
                }
            )

    artifact_burst_rows: List[dict] = []
    for (ds, sc, rt, md, sec_bin, fam), cnt in sorted(burst_counter.items()):
        artifact_burst_rows.append(
            {
                "dataset": ds,
                "scenario_id": sc,
                "route_id": rt,
                "mode": md,
                "sim_time_sec": sec_bin,
                "artifact_family": fam,
                "count": int(cnt),
            }
        )

    artifact_node_rows: List[dict] = []
    for (ds, sc, rt, md, tls, fam), cnt in sorted(artifact_node_counter.items()):
        artifact_node_rows.append(
            {
                "dataset": ds,
                "scenario_id": sc,
                "route_id": rt,
                "mode": md,
                "tls_id": tls,
                "artifact_family": fam,
                "count": int(cnt),
            }
        )

    # Persist all outputs.
    _write_csv(
        out_dir / "travel_times.csv",
        travel_rows,
        [
            "dataset",
            "scenario_id",
            "density_label",
            "density_count",
            "route_id",
            "mode",
            "ev_id",
            "travel_time_s",
            "waiting_time_s",
            "time_loss_s",
            "stop_time_s",
            "wall_elapsed_s",
            "arrived",
            "return_code",
            "http_precheck_ok",
            "http_startup_ok",
            "foreign_ev_drop_fail",
        ],
    )
    _write_csv(
        out_dir / "latency_req_resp_samples.csv",
        lat_rr_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "from_tls",
            "req_id",
            "sim_time",
            "latency_ms",
            "responder_phase_state_age_ms",
            "status",
        ],
    )
    _write_csv(
        out_dir / "latency_req_decision_samples.csv",
        lat_rd_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "req_id",
            "sim_time",
            "latency_ms",
            "status",
            "reason",
        ],
    )
    _write_csv(
        out_dir / "latency_req_actuation_samples.csv",
        lat_ra_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "latency_ms",
            "decision_source",
            "plan_type",
        ],
    )
    _write_csv(
        out_dir / "compute_duration_samples.csv",
        compute_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "stage",
            "duration_ms",
        ],
    )
    _write_csv(
        out_dir / "e2e_overhead_segments_route.csv",
        route_overhead_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "local_compute_ms",
            "local_apply_ms",
            "fnm_mediation_ms",
            "coordination_req_resp_ms",
            "total_e2e_ms",
            "compute_samples",
            "apply_samples",
            "coord_samples",
            "fnm_samples",
        ],
    )
    _write_csv(
        out_dir / "e2e_overhead_segments_node.csv",
        node_overhead_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "tls_id",
            "local_compute_ms",
            "local_apply_ms",
            "fnm_mediation_ms",
            "coordination_req_resp_ms",
            "total_e2e_ms",
            "compute_samples",
            "apply_samples",
            "coord_samples",
            "fnm_samples",
        ],
    )
    _write_csv(
        out_dir / "staleness_samples.csv",
        stale_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "staleness_type",
            "value_ms",
            "tls_id",
            "ev_id",
            "sim_time",
        ],
    )
    _write_csv(
        out_dir / "ev_request_samples.csv",
        ev_req_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "distance_to_intersection_m",
            "speed_mps",
            "request_age_ms",
            "source_tag",
            "delivery",
        ],
    )
    _write_csv(
        out_dir / "decision_samples.csv",
        decision_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "req_id",
            "status",
            "reason",
            "sim_time",
            "q_margin_sec",
            "spillback_risk",
            "readiness_score",
            "active_reservations",
        ],
    )
    _write_csv(
        out_dir / "node_cross_samples.csv",
        node_cross_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "event_type",
            "vehicle_waiting_time_s",
            "distance_to_intersection_m",
            "speed_mps",
        ],
    )
    _write_csv(
        out_dir / "coordination_metrics.csv",
        coord_metrics_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "requests_in",
            "proposals_out",
            "decisions_total",
            "accepted_nonspill",
            "req_handled_rate_pct",
            "proposal_before_expiry_rate_pct",
            "downstream_prep_rate_pct",
            "ev_clear_no_spill_rate_pct",
            "coordination_success_proxy_pct",
            "request_to_decision_median_ms",
            "coordinated_intersections_per_trip",
            "consecutive_intersections_no_stop_max",
            "node_cross_samples_with_wait",
        ],
    )
    _write_csv(
        out_dir / "queue_spillback_samples.csv",
        queue_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "q_margin_sec",
            "spillback_risk",
            "decision_status",
            "decision_reason",
        ],
    )
    _write_csv(
        out_dir / "queue_timeseries.csv",
        queue_ts_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "ev_id",
            "tls_id",
            "sim_time",
            "in_edge_id",
            "queue_len_est_veh",
            "queue_clear_time_sec",
            "queue_margin_sec",
            "spillback_risk",
            "spillback_active",
            "spillback_threshold",
            "readiness_score",
            "eta_mid_sec",
            "eta_start_sec",
            "eta_end_sec",
            "ev_distance_m",
            "ev_speed_mps",
        ],
    )
    _write_csv(
        out_dir / "artifact_volume.csv",
        artifact_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "artifact_family",
            "count",
        ],
    )
    _write_csv(
        out_dir / "artifact_volume_by_node.csv",
        artifact_node_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "tls_id",
            "artifact_family",
            "count",
        ],
    )
    _write_csv(
        out_dir / "event_counts.csv",
        event_count_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "event_type",
            "count",
        ],
    )
    _write_csv(
        out_dir / "fnm_event_counts.csv",
        fnm_event_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "component",
            "source_file",
            "event",
            "count",
        ],
    )
    _write_csv(
        out_dir / "fnm_latency_samples.csv",
        fnm_lat_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "component",
            "source_file",
            "event",
            "latency_ms",
        ],
    )
    _write_csv(
        out_dir / "fnm_processing_ratio.csv",
        fnm_processing_ratio_rows,
        [
            "dataset",
            "artifact_type",
            "received",
            "translated",
            "accepted",
            "deferred_rejected",
            "received_pct",
            "translated_pct",
            "accepted_pct",
            "deferred_rejected_pct",
        ],
    )
    _write_csv(
        out_dir / "artifact_burst_timeseries.csv",
        artifact_burst_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "sim_time_sec",
            "artifact_family",
            "count",
        ],
    )
    _write_csv(
        out_dir / "fnm_micro_latency_samples.csv",
        fnm_micro_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "component",
            "node_id",
            "source_file",
            "event",
            "artifact_kind",
            "schema_protocol_ms",
            "routing_orchestration_ms",
            "network_transport_ms",
            "total_stage_ms",
            "sim_time",
        ],
    )
    _write_csv(
        out_dir / "timeline_events.csv",
        timeline_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "sim_time",
            "tls_id",
            "ev_id",
            "event_type",
            "decision_source",
            "status",
            "reason",
            "plan_type",
            "signal_state",
            "phase_idx",
            "distance_to_intersection_m",
            "speed_mps",
            "request_age_ms",
        ],
    )
    _write_csv(
        out_dir / "mode_route_summary.csv",
        summary_rows,
        [
            "dataset",
            "scenario_id",
            "route_id",
            "mode",
            "travel_time_s",
            "decision_accepted",
            "decision_rejected",
            "decision_total",
            "coordination_success_rate",
            "req_out_count",
            "req_resp_e2e_count",
            "apply_plan_count",
            "apply_plan_skip_count",
        ],
    )

    print(json.dumps({"status": "ok", "out_dir": str(out_dir)}))


if __name__ == "__main__":
    main()
