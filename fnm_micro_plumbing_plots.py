#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, median
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Tuple

FIGSIZE_4_3 = (10.24, 7.68)


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _to_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8", errors="ignore") as f:
        return list(csv.DictReader(f))


def _read_jsonl(path: Path) -> Iterable[Dict[str, Any]]:
    if not path.exists():
        return []
    out: List[Dict[str, Any]] = []
    with path.open(encoding="utf-8", errors="ignore") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                obj = json.loads(ln)
            except Exception:
                continue
            if isinstance(obj, dict):
                out.append(obj)
    return out


def _find_events_file(run_dir: Path) -> Optional[Path]:
    cands = sorted(run_dir.glob("fed_outcomes*.events.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
    return cands[0] if cands else None


def _events_wall_time_window(events_path: Path) -> Tuple[Optional[float], Optional[float]]:
    lo: Optional[float] = None
    hi: Optional[float] = None
    for ev in _read_jsonl(events_path):
        ts = _to_float(ev.get("ts", 0.0), 0.0)
        if ts <= 1e9:
            continue
        if lo is None or ts < lo:
            lo = ts
        if hi is None or ts > hi:
            hi = ts
    return lo, hi


def _is_clean_row(r: Dict[str, str], expected_ev: str = "") -> bool:
    if str(r.get("return_code", "")).strip() not in ("0",):
        return False
    if str(r.get("arrived", "")).strip() not in ("1", "", "None"):
        return False
    if str(r.get("http_precheck_ok", "")).strip() not in ("1", "", "None"):
        return False
    if str(r.get("http_startup_ok", "")).strip() not in ("1", "", "None"):
        return False
    tt = str(r.get("travel_time_s", "")).strip()
    if tt in ("", "None", "nan"):
        return False
    if expected_ev:
        ev = str(r.get("ev_id", "")).strip()
        if ev and ev != expected_ev:
            return False
    drop_foreign = str(r.get("drop_foreign_ev_id", "")).strip()
    if drop_foreign not in ("", "None"):
        if _to_int(drop_foreign, 0) > 0:
            return False
    return True


def _pick_row(
    rows: List[Dict[str, str]],
    scenario_id: str,
    mode: str,
    route_id: int,
    expected_ev: str,
    clean_only: bool,
) -> Dict[str, str]:
    cand = rows
    if scenario_id:
        cand = [r for r in cand if str(r.get("scenario_id", "")).strip() == scenario_id]
    if mode:
        cand = [r for r in cand if str(r.get("mode", "")).strip() == mode]
    if route_id > 0:
        cand = [
            r
            for r in cand
            if _to_int(r.get("route_id", r.get("route_idx", "-1")), -1) == int(route_id)
        ]
    if expected_ev:
        cand = [r for r in cand if str(r.get("ev_id", "")).strip() == expected_ev]
    if clean_only:
        cand = [r for r in cand if _is_clean_row(r, expected_ev=expected_ev)]
    if not cand:
        raise RuntimeError("No matching run row found (check scenario/mode/route filters).")
    # Prefer lowest travel time among filtered rows for representative micro-case.
    cand = sorted(cand, key=lambda r: _to_float(r.get("travel_time_s", 1e18), 1e18))
    return cand[0]


def _resolve_run_dir(runs_root: Path, row: Dict[str, str]) -> Path:
    scenario_id = str(row.get("scenario_id", "")).strip()
    density = str(row.get("density_label", "")).strip()
    route_id = _to_int(row.get("route_id", row.get("route_idx", "-1")), -1)
    mode = str(row.get("mode", "")).strip()
    p_new = runs_root / "scenario_runs" / scenario_id / "matrix_out" / "runs" / density / f"route_{route_id}" / mode
    if p_new.exists():
        return p_new
    p_old = runs_root / "scenario_runs" / scenario_id / "matrix_out" / f"mode_{mode}"
    if p_old.exists():
        return p_old
    raise RuntimeError(f"Could not resolve mode run directory for scenario={scenario_id} mode={mode}.")


def _extract_metrics(
    events_path: Path,
    fnm_dir: Path,
    *,
    min_wall_ts: Optional[float] = None,
    max_wall_ts: Optional[float] = None,
) -> Dict[str, Any]:
    # 1A: stage latency distributions (ms)
    stage: DefaultDict[str, List[float]] = defaultdict(list)
    stage_by_kind: DefaultDict[str, DefaultDict[str, List[float]]] = defaultdict(lambda: defaultdict(list))
    # 1C: staleness
    staleness_request_age_ms: List[float] = []
    staleness_state_pull_age_ms: List[float] = []
    staleness_phase_state_age_ms: List[float] = []
    # 1D: delivery / handling
    counts = defaultdict(int)
    delivery_by_kind: DefaultDict[str, Dict[str, int]] = defaultdict(lambda: {"ok": 0, "error": 0, "total": 0})
    delivery_before_expiry_by_kind: DefaultDict[str, Dict[str, int]] = defaultdict(
        lambda: {"before_expiry": 0, "with_expiry": 0}
    )

    # Request->actuation proxy from event timeline
    req_queue_by_tls: DefaultDict[str, List[float]] = defaultdict(list)
    req_to_signal_ms: List[float] = []

    for ev in _read_jsonl(events_path):
        et = str(ev.get("event_type", "")).strip()
        if not et:
            continue
        sim_t = _to_float(ev.get("sim_time"), -1.0)
        tls_id = str(ev.get("tls_id", "")).strip()

        if et == "ev.request.dispatched":
            counts["request_emitted"] += 1
        elif et == "ev.request.in":
            counts["request_received"] += 1
            age = _to_float(ev.get("request_age_ms"), -1.0)
            if age >= 0:
                staleness_request_age_ms.append(age)
            if tls_id and sim_t >= 0:
                req_queue_by_tls[tls_id].append(sim_t)
        elif et == "coord.reservation.req_out":
            counts["coord_req_out"] += 1
        elif et == "coord.reservation.req_resp_e2e":
            counts["coord_req_resp"] += 1
            lat = _to_float(ev.get("latency_ms"), -1.0)
            if lat >= 0:
                stage["request_to_response_e2e_ms"].append(lat)
            for k in (
                "phase_state_age_ms",
                "reply_state_age_ms",
                "intersection_state_age_ms",
                "responder_phase_state_age_ms",
            ):
                vv = _to_float(ev.get(k), -1.0)
                if vv >= 0:
                    staleness_phase_state_age_ms.append(vv)
        elif et == "coord.reservation.req_decision":
            counts["coord_decision"] += 1
            status = str(ev.get("status", "")).upper()
            reason = str(ev.get("reason", "")).lower()
            if status == "ACCEPTED":
                counts["coord_accepted"] += 1
            elif status == "REJECTED":
                counts["coord_rejected"] += 1
            if "stale" in reason or "expired" in reason or "defer" in reason:
                counts["coord_stale_or_deferred"] += 1
        elif et == "tls.signal.change":
            if tls_id and sim_t >= 0:
                q = req_queue_by_tls.get(tls_id, [])
                if q:
                    req_t = q.pop(0)
                    if sim_t >= req_t:
                        req_to_signal_ms.append(1000.0 * (sim_t - req_t))

    if req_to_signal_ms:
        stage["request_to_actuation_ms"] = req_to_signal_ms

    # Parse FNM trace logs for stage-level plumbing timing.
    state_pull_ok = 0
    state_pull_err = 0
    for fp in sorted(fnm_dir.glob("fnm_*.jsonl")):
        for j in _read_jsonl(fp):
            wall_ts = _to_float(j.get("ts", 0.0), 0.0)
            if min_wall_ts is not None and wall_ts > 0.0 and wall_ts < float(min_wall_ts):
                continue
            if max_wall_ts is not None and wall_ts > 0.0 and wall_ts > float(max_wall_ts):
                continue
            evt = str(j.get("event", "")).strip()
            if evt == "fnm.stage.local_to_fed":
                kind = str(j.get("artefact_kind", "event")).strip().lower() or "event"
                v1 = _to_float(j.get("local_ingest_to_schema_ms"), -1.0)
                v2 = _to_float(j.get("schema_to_fed_publish_ms"), -1.0)
                v3 = _to_float(j.get("local_to_fed_total_ms"), -1.0)
                if v1 >= 0:
                    stage["local_dt_to_fnm_ingestion_ms"].append(v1)
                    stage_by_kind[kind]["local_dt_to_fnm_ingestion_ms"].append(v1)
                if v2 >= 0:
                    stage["protocol_schema_mediation_ms"].append(v2)
                    stage_by_kind[kind]["schema_to_fed_publish_ms"].append(v2)
                if v3 >= 0:
                    stage_by_kind[kind]["local_to_fed_total_ms"].append(v3)
            elif evt == "fnm.stage.adapter_state_pull":
                http_ms = _to_float(j.get("http_get_ms"), -1.0)
                decode_ms = _to_float(j.get("decode_ms"), -1.0)
                norm_ms = _to_float(j.get("normalize_ms"), -1.0)
                build_ms = _to_float(j.get("request_build_ms"), -1.0)
                pub_total_ms = _to_float(j.get("request_publish_total_ms"), -1.0)
                req_pub = max(1, _to_int(j.get("req_published", 0), 0))
                ingest_ms = (max(0.0, http_ms) if http_ms >= 0 else 0.0) + (max(0.0, decode_ms) if decode_ms >= 0 else 0.0)
                mediate_ms = (max(0.0, norm_ms) if norm_ms >= 0 else 0.0) + (max(0.0, build_ms) if build_ms >= 0 else 0.0)
                publish_ms = (pub_total_ms / float(req_pub)) if pub_total_ms >= 0 else -1.0
                if ingest_ms > 0:
                    stage["adapter_http_decode_ms"].append(ingest_ms)
                    stage["local_dt_to_fnm_ingestion_ms"].append(ingest_ms)
                    stage_by_kind["state"]["local_dt_to_fnm_ingestion_ms"].append(ingest_ms)
                if mediate_ms > 0:
                    stage["adapter_normalize_build_ms"].append(mediate_ms)
                    stage["protocol_schema_mediation_ms"].append(mediate_ms)
                    stage_by_kind["state"]["protocol_schema_mediation_ms"].append(mediate_ms)
                if publish_ms >= 0:
                    stage["adapter_publish_per_request_ms"].append(publish_ms)
                    stage_by_kind["state"]["schema_to_fed_publish_ms"].append(publish_ms)
                state_age = _to_float(j.get("state_age_ms"), -1.0)
                if state_age >= 0:
                    staleness_state_pull_age_ms.append(state_age)
            elif evt == "fnm.stage.adapter_request_publish":
                kind = str(j.get("artefact_kind", "request_response")).strip().lower() or "request_response"
                v1 = _to_float(j.get("local_ingest_to_schema_ms"), -1.0)
                v2 = _to_float(j.get("schema_to_fed_publish_ms"), -1.0)
                v3 = _to_float(j.get("local_to_fed_total_ms"), -1.0)
                if v1 >= 0:
                    stage_by_kind[kind]["local_dt_to_fnm_ingestion_ms"].append(v1)
                if v2 >= 0:
                    stage_by_kind[kind]["schema_to_fed_publish_ms"].append(v2)
                if v3 >= 0:
                    stage_by_kind[kind]["local_to_fed_total_ms"].append(v3)
            elif evt == "fnm.stage.fed_to_local":
                kind = str(j.get("artefact_kind", "")).strip().lower()
                v1 = _to_float(j.get("fed_publish_to_remote_receive_ms"), -1.0)
                v2 = _to_float(j.get("remote_receive_to_local_invoke_ms"), -1.0)
                v3 = _to_float(j.get("origin_to_local_invoke_ms"), -1.0)
                if v1 >= 0:
                    stage["federation_publication_to_remote_reception_ms"].append(v1)
                    stage_by_kind[kind]["federation_publication_to_remote_reception_ms"].append(v1)
                if v2 >= 0:
                    stage["remote_reception_to_local_invocation_ms"].append(v2)
                    stage_by_kind[kind]["remote_reception_to_local_invocation_ms"].append(v2)
                if v3 >= 0:
                    if kind == "state":
                        stage["state_propagation_latency_ms"].append(v3)
                    elif kind == "event":
                        stage["event_propagation_latency_ms"].append(v3)
                    elif kind in ("request_response", "request"):
                        stage["request_response_propagation_latency_ms"].append(v3)
                    elif kind == "coordination":
                        stage["coordination_artefact_propagation_latency_ms"].append(v3)
            elif evt in ("fnm.delivery.local_to_fed", "fnm.delivery.fed_to_local", "fnm.delivery.adapter_state_to_fed"):
                kind = str(j.get("artefact_kind", "event")).strip().lower() or "event"
                st = str(j.get("status", "")).strip().lower()
                delivery_by_kind[kind]["total"] += 1
                if st == "ok":
                    delivery_by_kind[kind]["ok"] += 1
                else:
                    delivery_by_kind[kind]["error"] += 1
                be = j.get("before_expiry", None)
                if be is not None:
                    delivery_before_expiry_by_kind[kind]["with_expiry"] += 1
                    if bool(be):
                        delivery_before_expiry_by_kind[kind]["before_expiry"] += 1
            elif evt == "fnm.adapter.state_pull.ok":
                state_pull_ok += 1
                age = _to_float(j.get("state_age_ms", j.get("age_ms", -1.0)), -1.0)
                if age >= 0:
                    staleness_state_pull_age_ms.append(age)
                counts["adapter_req_published"] += max(0, _to_int(j.get("req_published", 0), 0))
                counts["adapter_req_publish_error"] += max(0, _to_int(j.get("req_publish_error", 0), 0))
            elif evt == "fnm.adapter.state_pull.error":
                state_pull_err += 1

    counts["state_pull_ok"] = state_pull_ok
    counts["state_pull_error"] = state_pull_err

    return {
        "stage_latency": dict(stage),
        "staleness_request_age_ms": staleness_request_age_ms,
        "staleness_state_pull_age_ms": staleness_state_pull_age_ms,
        "staleness_phase_state_age_ms": staleness_phase_state_age_ms,
        "counts": dict(counts),
        "delivery_by_kind": dict(delivery_by_kind),
        "delivery_before_expiry_by_kind": dict(delivery_before_expiry_by_kind),
        "stage_by_kind": {k: dict(v) for k, v in stage_by_kind.items()},
    }


def _write_outputs(out_dir: Path, selected_row: Dict[str, str], metrics: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stage_latency: Dict[str, List[float]] = metrics["stage_latency"]
    counts: Dict[str, int] = metrics["counts"]
    delivery_by_kind: Dict[str, Dict[str, int]] = metrics["delivery_by_kind"]
    delivery_before_expiry_by_kind: Dict[str, Dict[str, int]] = metrics["delivery_before_expiry_by_kind"]
    stage_by_kind: Dict[str, Dict[str, List[float]]] = metrics.get("stage_by_kind", {}) or {}

    with (out_dir / "selected_run_context.json").open("w", encoding="utf-8") as f:
        json.dump(selected_row, f, indent=2, ensure_ascii=True)

    with (out_dir / "stage_latency_samples.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["stage", "latency_ms"])
        for k, vals in stage_latency.items():
            for v in vals:
                w.writerow([k, f"{v:.6f}"])

    with (out_dir / "stage_latency_by_kind_samples.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["artefact_kind", "stage_component", "latency_ms"])
        for kind, comp_map in stage_by_kind.items():
            for comp, vals in comp_map.items():
                for v in vals:
                    w.writerow([str(kind), str(comp), f"{float(v):.6f}"])

    with (out_dir / "staleness_samples.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value_ms"])
        for v in metrics["staleness_request_age_ms"]:
            w.writerow(["request_age_at_intersection_decision_ms", f"{float(v):.6f}"])
        for v in metrics["staleness_state_pull_age_ms"]:
            w.writerow(["state_pull_age_ms", f"{float(v):.6f}"])
        for v in metrics["staleness_phase_state_age_ms"]:
            w.writerow(["intersection_phase_state_age_at_ev_reply_ms", f"{float(v):.6f}"])

    request_emitted = counts.get("request_emitted", 0)
    request_received = counts.get("request_received", 0)
    coord_req_out = counts.get("coord_req_out", 0)
    coord_req_resp = counts.get("coord_req_resp", 0)
    coord_decision = counts.get("coord_decision", 0)
    coord_accepted = counts.get("coord_accepted", 0)
    coord_stale_or_deferred = counts.get("coord_stale_or_deferred", 0)
    state_pull_ok = counts.get("state_pull_ok", 0)
    state_pull_error = counts.get("state_pull_error", 0)

    ratio_rows = [
        (
            "state_updates_success_ratio",
            state_pull_ok / max(1, state_pull_ok + state_pull_error),
            state_pull_ok,
            state_pull_ok + state_pull_error,
        ),
        (
            "request_delivery_ratio",
            request_received / max(1, request_emitted),
            request_received,
            request_emitted,
        ),
        (
            "coord_response_ratio",
            coord_req_resp / max(1, coord_req_out),
            coord_req_resp,
            coord_req_out,
        ),
        (
            "coord_accept_ratio",
            coord_accepted / max(1, coord_decision),
            coord_accepted,
            coord_decision,
        ),
        (
            "coord_timely_nonstale_ratio",
            1.0 - (coord_stale_or_deferred / max(1, coord_decision)),
            max(0, coord_decision - coord_stale_or_deferred),
            coord_decision,
        ),
    ]
    for kind in ("state", "event", "request_response", "coordination"):
        d = dict(delivery_by_kind.get(kind, {}) or {})
        ok = int(d.get("ok", 0) or 0)
        tot = int(d.get("total", 0) or 0)
        ratio_rows.append((f"{kind}_propagation_success_ratio", ok / max(1, tot), ok, tot))
        de = dict(delivery_before_expiry_by_kind.get(kind, {}) or {})
        bef = int(de.get("before_expiry", 0) or 0)
        with_exp = int(de.get("with_expiry", 0) or 0)
        if with_exp > 0:
            ratio_rows.append((f"{kind}_before_expiry_ratio", bef / max(1, with_exp), bef, with_exp))
    with (out_dir / "handling_success_ratios.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["metric", "ratio", "numerator", "denominator"])
        for name, ratio, num, den in ratio_rows:
            w.writerow([name, f"{float(ratio):.6f}", int(num), int(den)])


def _make_plots(out_dir: Path, metrics: Dict[str, Any]) -> None:
    import matplotlib.pyplot as plt

    stage_latency: Dict[str, List[float]] = metrics["stage_latency"]
    st_req: List[float] = metrics["staleness_request_age_ms"]
    st_pull: List[float] = metrics["staleness_state_pull_age_ms"]
    st_phase: List[float] = metrics["staleness_phase_state_age_ms"]
    counts: Dict[str, int] = metrics["counts"]
    delivery_by_kind: Dict[str, Dict[str, int]] = metrics["delivery_by_kind"]
    delivery_before_expiry_by_kind: Dict[str, Dict[str, int]] = metrics["delivery_before_expiry_by_kind"]
    stage_by_kind: Dict[str, Dict[str, List[float]]] = metrics.get("stage_by_kind", {}) or {}

    # 1A: E2E interaction latency
    stage_order = [
        "state_propagation_latency_ms",
        "event_propagation_latency_ms",
        "local_dt_to_fnm_ingestion_ms",
        "protocol_schema_mediation_ms",
        "federation_publication_to_remote_reception_ms",
        "remote_reception_to_local_invocation_ms",
        "request_to_response_e2e_ms",
        "request_to_actuation_ms",
    ]
    vals = [stage_latency[k] for k in stage_order if stage_latency.get(k)]
    labels = [k for k in stage_order if stage_latency.get(k)]
    if vals:
        plt.figure(figsize=FIGSIZE_4_3)
        plt.boxplot(vals, labels=labels, showfliers=False)
        plt.xticks(rotation=24, ha="right")
        plt.ylabel("Latency (ms)")
        plt.title("1A. End-to-End Interaction Latency by Stage")
        plt.tight_layout()
        plt.savefig(out_dir / "plot_1a_e2e_stage_latency_box_4x3.png", dpi=180)
        plt.close()

        means = [mean(v) for v in vals]
        medians = [median(v) for v in vals]
        x = list(range(len(labels)))
        w = 0.38
        plt.figure(figsize=FIGSIZE_4_3)
        plt.bar([i - w / 2 for i in x], means, width=w, label="mean_ms")
        plt.bar([i + w / 2 for i in x], medians, width=w, label="median_ms")
        plt.xticks(x, labels, rotation=24, ha="right")
        plt.ylabel("Latency (ms)")
        plt.title("1A. Stage Latency Mean vs Median")
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "plot_1a_e2e_stage_latency_bar_4x3.png", dpi=180)
        plt.close()

    # 1B: FNM internal processing by artefact type (stacked means)
    kinds = ["state", "event", "request_response", "coordination"]
    comps = [
        ("local_dt_to_fnm_ingestion_ms", "ingest"),
        ("protocol_schema_mediation_ms", "mediate"),
        ("schema_to_fed_publish_ms", "publish"),
        ("remote_reception_to_local_invocation_ms", "remote_local"),
    ]
    stacked: Dict[str, List[float]] = {lbl: [] for _, lbl in comps}
    nonzero = False
    for kind in kinds:
        km = dict(stage_by_kind.get(kind, {}) or {})
        for src, lbl in comps:
            vals = list(km.get(src, []) or [])
            m = float(mean(vals)) if vals else 0.0
            stacked[lbl].append(m)
            if m > 0.0:
                nonzero = True
    if nonzero:
        x = list(range(len(kinds)))
        plt.figure(figsize=FIGSIZE_4_3)
        bottom = [0.0 for _ in kinds]
        colors = {
            "ingest": "#4e79a7",
            "mediate": "#f28e2b",
            "publish": "#59a14f",
            "remote_local": "#e15759",
        }
        for lbl in ["ingest", "mediate", "publish", "remote_local"]:
            vals = stacked[lbl]
            plt.bar(x, vals, bottom=bottom, label=lbl, color=colors.get(lbl))
            bottom = [bottom[i] + vals[i] for i in range(len(bottom))]
        plt.xticks(x, kinds, rotation=20, ha="right")
        plt.ylabel("Mean latency (ms)")
        plt.title("1B. FNM Internal Processing Latency by Artefact Type")
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "plot_1b_internal_processing_stacked_4x3.png", dpi=180)
        plt.close()

    # 1C: Timeliness / staleness
    if st_req or st_pull or st_phase:
        fig, axs = plt.subplots(1, 2, figsize=FIGSIZE_4_3)
        left_data = []
        left_labels = []
        if st_req:
            left_data.append(st_req)
            left_labels.append("request_age_ms")
        if st_pull:
            left_data.append(st_pull)
            left_labels.append("state_pull_age_ms")
        if st_phase:
            left_data.append(st_phase)
            left_labels.append("phase_state_age_ms")
        axs[0].boxplot(left_data, labels=left_labels, showfliers=False)
        axs[0].set_ylabel("Age (ms)")
        axs[0].set_title("1C. Staleness at Decision Time")
        axs[0].tick_params(axis="x", rotation=20)

        if st_req:
            s = sorted(st_req)
            y = [(i + 1) / len(s) for i in range(len(s))]
            axs[1].plot(s, y, label="request_age_ms")
        if st_pull:
            s = sorted(st_pull)
            y = [(i + 1) / len(s) for i in range(len(s))]
            axs[1].plot(s, y, label="state_pull_age_ms")
        if st_phase:
            s = sorted(st_phase)
            y = [(i + 1) / len(s) for i in range(len(s))]
            axs[1].plot(s, y, label="phase_state_age_ms")
        axs[1].set_xlabel("Age (ms)")
        axs[1].set_ylabel("CDF")
        axs[1].set_title("1C. Staleness CDF")
        axs[1].legend()
        fig.tight_layout()
        fig.savefig(out_dir / "plot_1c_timeliness_staleness_4x3.png", dpi=180)
        plt.close(fig)

    # 1D: Delivery / handling success
    request_emitted = counts.get("request_emitted", 0)
    request_received = counts.get("request_received", 0)
    coord_req_out = counts.get("coord_req_out", 0)
    coord_req_resp = counts.get("coord_req_resp", 0)
    coord_decision = counts.get("coord_decision", 0)
    coord_accepted = counts.get("coord_accepted", 0)
    coord_stale_or_deferred = counts.get("coord_stale_or_deferred", 0)
    state_pull_ok = counts.get("state_pull_ok", 0)
    state_pull_error = counts.get("state_pull_error", 0)
    adapter_req_published = counts.get("adapter_req_published", 0)
    adapter_req_publish_error = counts.get("adapter_req_publish_error", 0)

    names = ["state_success", "event_success", "request_success", "coord_success", "coord_before_expiry"]
    state_d = dict(delivery_by_kind.get("state", {}) or {})
    event_d = dict(delivery_by_kind.get("event", {}) or {})
    req_d = dict(delivery_by_kind.get("request_response", {}) or {})
    coord_d = dict(delivery_by_kind.get("coordination", {}) or {})
    coord_exp = dict(delivery_before_expiry_by_kind.get("coordination", {}) or {})
    ratios = [
        int(state_d.get("ok", 0) or 0) / max(1, int(state_d.get("total", 0) or 0)),
        int(event_d.get("ok", 0) or 0) / max(1, int(event_d.get("total", 0) or 0)),
        max(
            int(req_d.get("ok", 0) or 0) / max(1, int(req_d.get("total", 0) or 0)),
            coord_req_resp / max(1, coord_req_out),
            request_received / max(1, request_emitted),
        ),
        max(
            int(coord_d.get("ok", 0) or 0) / max(1, int(coord_d.get("total", 0) or 0)),
            coord_accepted / max(1, coord_decision),
        ),
        int(coord_exp.get("before_expiry", 0) or 0) / max(1, int(coord_exp.get("with_expiry", 0) or 0)),
    ]
    plt.figure(figsize=FIGSIZE_4_3)
    plt.bar(range(len(names)), [100.0 * x for x in ratios])
    plt.xticks(range(len(names)), names, rotation=20, ha="right")
    plt.ylim(0, 105)
    plt.ylabel("Success ratio (%)")
    plt.title("1D. Delivery / Handling Success Ratios")
    plt.tight_layout()
    plt.savefig(out_dir / "plot_1d_delivery_handling_success_4x3.png", dpi=180)
    plt.close()

    if (adapter_req_published + adapter_req_publish_error) > 0:
        plt.figure(figsize=FIGSIZE_4_3)
        labels2 = ["adapter_request_publish_success", "state_pull_success"]
        vals2 = [
            100.0 * (float(adapter_req_published) / max(1.0, float(adapter_req_published + adapter_req_publish_error))),
            100.0 * (float(state_pull_ok) / max(1.0, float(state_pull_ok + state_pull_error))),
        ]
        plt.bar(range(len(labels2)), vals2, color=["#59a14f", "#4e79a7"])
        plt.xticks(range(len(labels2)), labels2, rotation=20, ha="right")
        plt.ylim(0, 105)
        plt.ylabel("Success ratio (%)")
        plt.title("1D. Adapter Success Ratios")
        plt.tight_layout()
        plt.savefig(out_dir / "plot_1d_adapter_success_4x3.png", dpi=180)
        plt.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate micro-level FNM integration plots (1A/1C/1D) from one clean representative run.")
    ap.add_argument("--runs-root", required=True, help="Root matrix runs directory (contains ev_matrix_results.csv + scenario_runs/)")
    ap.add_argument("--out-dir", required=True, help="Output directory for CSV and plots")
    ap.add_argument("--scenario-id", default="", help="Scenario ID (recommended)")
    ap.add_argument("--mode", default="F2", help="Mode to analyze (default: F2)")
    ap.add_argument("--route-id", type=int, default=-1, help="Route ID filter (optional)")
    ap.add_argument("--ev-id", default="", help="EV ID filter (optional)")
    ap.add_argument("--clean-only", action="store_true", help="Use only runs that pass clean checks")
    args = ap.parse_args()

    runs_root = Path(args.runs_root).resolve()
    out_dir = Path(args.out_dir).resolve()
    rows = _read_csv(runs_root / "ev_matrix_results.csv")
    if not rows:
        raise RuntimeError(f"Missing or empty ev_matrix_results.csv in {runs_root}")

    row = _pick_row(
        rows=rows,
        scenario_id=str(args.scenario_id or "").strip(),
        mode=str(args.mode or "").strip(),
        route_id=int(args.route_id),
        expected_ev=str(args.ev_id or "").strip(),
        clean_only=bool(args.clean_only),
    )
    run_dir = _resolve_run_dir(runs_root, row)
    events = _find_events_file(run_dir)
    if events is None:
        raise RuntimeError(f"No fed_outcomes*.events.jsonl found in {run_dir}")

    scenario_id = str(row.get("scenario_id", "")).strip()
    fnm_dir = runs_root / "scenario_runs" / scenario_id / "fnm_sidecars"
    if not fnm_dir.exists():
        raise RuntimeError(f"Missing fnm_sidecars directory: {fnm_dir}")

    win_lo, win_hi = _events_wall_time_window(events)
    metrics = _extract_metrics(
        events,
        fnm_dir,
        min_wall_ts=(win_lo - 2.0) if win_lo is not None else None,
        max_wall_ts=(win_hi + 2.0) if win_hi is not None else None,
    )
    _write_outputs(out_dir, row, metrics)
    _make_plots(out_dir, metrics)
    print(f"OK: micro plots written to {out_dir}")


if __name__ == "__main__":
    main()
