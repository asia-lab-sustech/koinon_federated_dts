#!/usr/bin/env python3
"""
Generate paper-style benchmark plots from multiple federation runs.

Manifest CSV columns (required):
- run_id
- metrics_dir
- profile            (e.g., FED, NO_FED, OBSERVE, ADVISORY)
- msg_rate           (numeric; msgs/sec)
- payload_bytes      (numeric; bytes)
- qos                (string/int label; e.g., qos0, qos1)

Optional:
- label

Each metrics_dir should contain:
- summary.json
- metrics_summary.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class RunRow:
    run_id: str
    metrics_dir: str
    profile: str
    msg_rate: float
    payload_bytes: float
    qos: str
    label: str


def _f(v: Any, d: float = 0.0) -> float:
    try:
        if v is None or v == "":
            return d
        return float(v)
    except Exception:
        return d


def _read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return raw if isinstance(raw, dict) else {}


def _load_manifest(path: str) -> List[RunRow]:
    rows = _read_csv(path)
    base = os.path.dirname(os.path.abspath(path))
    out: List[RunRow] = []
    for r in rows:
        run_id = str(r.get("run_id", "") or "").strip()
        metrics_dir = str(r.get("metrics_dir", "") or "").strip()
        profile = str(r.get("profile", "") or "").strip()
        if not run_id or not metrics_dir or not profile:
            continue
        mdir = metrics_dir
        if not os.path.isabs(mdir):
            mdir = os.path.join(base, mdir)
        out.append(
            RunRow(
                run_id=run_id,
                metrics_dir=os.path.abspath(mdir),
                profile=profile,
                msg_rate=_f(r.get("msg_rate", 0.0)),
                payload_bytes=_f(r.get("payload_bytes", 0.0)),
                qos=str(r.get("qos", "") or "").strip(),
                label=str(r.get("label", run_id) or run_id),
            )
        )
    return out


def _metrics_map(metrics_csv_path: str) -> Dict[Tuple[str, str], Dict[str, float]]:
    rows = _read_csv(metrics_csv_path)
    out: Dict[Tuple[str, str], Dict[str, float]] = {}
    for r in rows:
        metric = str(r.get("metric", "") or "")
        domain = str(r.get("domain", "") or "")
        if not metric or not domain:
            continue
        out[(metric, domain)] = {
            "n": _f(r.get("n", 0)),
            "mean_ms": _f(r.get("mean_ms", 0)),
            "p95_ms": _f(r.get("p95_ms", 0)),
            "p99_ms": _f(r.get("p99_ms", 0)),
        }
    return out


def _phase_map(summary_json_path: str) -> Dict[str, Dict[str, float]]:
    s = _read_json(summary_json_path)
    phase = dict(s.get("operational_phases_ms", {}) or {})
    out: Dict[str, Dict[str, float]] = {}
    for k, v in phase.items():
        d = dict(v or {})
        out[str(k)] = {
            "n": _f(d.get("n", 0)),
            "mean_ms": _f(d.get("mean_ms", 0)),
            "p95_ms": _f(d.get("p95_ms", 0)),
        }
    return out


def _run_payload(rows: List[RunRow]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    out: List[Dict[str, Any]] = []
    skipped: List[Dict[str, Any]] = []
    for rr in rows:
        summary_path = os.path.join(rr.metrics_dir, "summary.json")
        metrics_path = os.path.join(rr.metrics_dir, "metrics_summary.csv")
        if not (os.path.exists(summary_path) and os.path.exists(metrics_path)):
            skipped.append(
                {
                    "run_id": rr.run_id,
                    "metrics_dir": rr.metrics_dir,
                    "missing_summary": int(not os.path.exists(summary_path)),
                    "missing_metrics_csv": int(not os.path.exists(metrics_path)),
                }
            )
            continue
        out.append(
            {
                "row": rr,
                "metrics": _metrics_map(metrics_path),
                "phase": _phase_map(summary_path),
                "summary": _read_json(summary_path),
            }
        )
    return out, skipped


def _profiles(payload: List[Dict[str, Any]]) -> List[str]:
    return sorted({str(x["row"].profile) for x in payload})


def _plot_operational_phases(payload: List[Dict[str, Any]], out_dir: str, plt) -> None:
    phases = ["register_to_onboarding_ms", "onboarding_to_active_ms", "time_to_active_ms"]
    profs = _profiles(payload)
    if not profs:
        return
    # Mean across runs per profile
    vals: Dict[str, List[float]] = {p: [] for p in profs}
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(phases)))
    w = 0.8 / max(1, len(profs))
    for i, prof in enumerate(profs):
        means = []
        for ph in phases:
            ms = [float(item["phase"].get(ph, {}).get("mean_ms", 0.0)) for item in payload if item["row"].profile == prof]
            means.append(sum(ms) / len(ms) if ms else 0.0)
        offset = (i - (len(profs) - 1) / 2.0) * w
        ax.bar([x + offset for x in xs], means, width=w, label=prof)
    ax.set_xticks(xs)
    ax.set_xticklabels(phases, rotation=20, ha="right")
    ax.set_ylabel("Execution time (ms)")
    ax.set_title("Operational Phases by Profile")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_operational_phases.png"), dpi=170)
    plt.close(fig)


def _plot_metric_vs_x(
    payload: List[Dict[str, Any]],
    out_png: str,
    metric: str,
    x_key: str,
    x_label: str,
    title: str,
    plt,
) -> None:
    profs = _profiles(payload)
    if not profs:
        return
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111)
    any_data = False
    for prof in profs:
        pts: List[Tuple[float, float]] = []
        for item in payload:
            rr: RunRow = item["row"]
            if rr.profile != prof:
                continue
            x = float(getattr(rr, x_key))
            m = item["metrics"].get((metric, "wall_ms"), {})
            n = int(_f(m.get("n", 0)))
            if n <= 0:
                continue
            y = float(_f(m.get("mean_ms", 0.0)))
            pts.append((x, y))
        if not pts:
            continue
        any_data = True
        pts.sort(key=lambda t: t[0])
        ax.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=prof)
    if not any_data:
        plt.close(fig)
        return
    ax.set_xlabel(x_label)
    ax.set_ylabel("E2E Delay (ms)")
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=170)
    plt.close(fig)


def _plot_qos_effect(payload: List[Dict[str, Any]], out_dir: str, metric: str, plt) -> None:
    profs = _profiles(payload)
    qos_vals = sorted({str(item["row"].qos) for item in payload if str(item["row"].qos)})
    if not profs or not qos_vals:
        return
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(qos_vals)))
    w = 0.8 / max(1, len(profs))
    any_data = False
    for i, prof in enumerate(profs):
        ys = []
        for q in qos_vals:
            vals = []
            for item in payload:
                rr: RunRow = item["row"]
                if rr.profile != prof or rr.qos != q:
                    continue
                m = item["metrics"].get((metric, "wall_ms"), {})
                if int(_f(m.get("n", 0))) <= 0:
                    continue
                vals.append(float(_f(m.get("mean_ms", 0.0))))
            ys.append((sum(vals) / len(vals)) if vals else 0.0)
        if any(y > 0 for y in ys):
            any_data = True
        off = (i - (len(profs) - 1) / 2.0) * w
        ax.bar([x + off for x in xs], ys, width=w, label=prof)
    if not any_data:
        plt.close(fig)
        return
    ax.set_xticks(xs)
    ax.set_xticklabels(qos_vals)
    ax.set_ylabel("E2E Delay (ms)")
    ax.set_xlabel("QoS configuration")
    ax.set_title(f"QoS Effect on {metric}")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_qos_effect.png"), dpi=170)
    plt.close(fig)


def _coord_flow(summary: Dict[str, Any]) -> Dict[str, float]:
    c = dict(summary.get("coordination_flow", {}) or {})
    return {
        "req": _f(c.get("reservation_req_sent", 0)),
        "resp": _f(c.get("reservation_resp_recv", 0)),
        "assoc_created": _f(c.get("association_created", 0)),
        "assoc_released": _f(c.get("association_released", 0)),
        "route_adv": _f(c.get("route_advice_published", 0)),
        "tls_adv": _f(c.get("intersection_advice_published", 0)),
    }


def _population(summary: Dict[str, Any]) -> Dict[str, float]:
    p = dict(summary.get("federation_population", {}) or {})
    return {
        "registered": _f(p.get("n_registered_unique", 0)),
        "active_ever": _f(p.get("n_active_ever", 0)),
        "max_active": _f(p.get("max_active_members", 0)),
        "end_active": _f(p.get("end_active_members", 0)),
    }


def _plot_active_members_scalability(payload: List[Dict[str, Any]], out_dir: str, metric: str, plt) -> None:
    profs = _profiles(payload)
    if not profs:
        return
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111)
    any_data = False
    for prof in profs:
        pts: List[Tuple[float, float]] = []
        for item in payload:
            rr: RunRow = item["row"]
            if rr.profile != prof:
                continue
            pop = _population(item.get("summary", {}))
            x = float(pop.get("max_active", 0))
            if x <= 0:
                x = float(pop.get("active_ever", 0))
            m = item["metrics"].get((metric, "wall_ms"), {})
            if int(_f(m.get("n", 0))) <= 0:
                continue
            y = float(_f(m.get("mean_ms", 0)))
            if x <= 0:
                continue
            pts.append((x, y))
        if not pts:
            continue
        any_data = True
        pts.sort(key=lambda t: t[0])
        ax.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=prof)
    if not any_data:
        plt.close(fig)
        return
    ax.set_xlabel("Active members (max)")
    ax.set_ylabel("E2E delay (ms)")
    ax.set_title(f"Scalability: {metric} vs Active Members")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_scalability_active_members.png"), dpi=170)
    plt.close(fig)


def _plot_communication_overhead(payload: List[Dict[str, Any]], out_dir: str, plt) -> None:
    profs = _profiles(payload)
    if not profs:
        return
    vals_total: Dict[str, List[float]] = {p: [] for p in profs}
    vals_per_decision: Dict[str, List[float]] = {p: [] for p in profs}
    for item in payload:
        prof = str(item["row"].profile)
        cf = _coord_flow(item.get("summary", {}))
        total_msgs = cf["req"] + cf["resp"] + cf["assoc_created"] + cf["assoc_released"] + cf["route_adv"] + cf["tls_adv"]
        decisions = max(1.0, cf["assoc_created"], cf["resp"])
        vals_total[prof].append(total_msgs)
        vals_per_decision[prof].append(total_msgs / decisions if decisions > 0 else 0.0)

    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    labels = profs
    xs = list(range(len(labels)))
    avg_total = [sum(vals_total[p]) / len(vals_total[p]) if vals_total[p] else 0.0 for p in labels]
    avg_mpd = [sum(vals_per_decision[p]) / len(vals_per_decision[p]) if vals_per_decision[p] else 0.0 for p in labels]
    w = 0.42
    ax.bar([x - w / 2.0 for x in xs], avg_total, width=w, label="total_coord_msgs")
    ax.bar([x + w / 2.0 for x in xs], avg_mpd, width=w, label="msgs_per_decision")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Count")
    ax.set_title("Communication Overhead by Profile")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_communication_overhead.png"), dpi=170)
    plt.close(fig)


def _plot_computation_overhead(payload: List[Dict[str, Any]], out_dir: str, plt) -> None:
    profs = _profiles(payload)
    if not profs:
        return
    metric_set = ["T_discovery_e2e", "T_coord_req_resp", "T_assoc_setup", "T_advice_uptake", "T_onboard"]
    fig = plt.figure(figsize=(11, 5))
    ax = fig.add_subplot(111)
    xs = list(range(len(metric_set)))
    w = 0.8 / max(1, len(profs))
    for i, prof in enumerate(profs):
        ys = []
        for mname in metric_set:
            vals = []
            for item in payload:
                if str(item["row"].profile) != prof:
                    continue
                m = item["metrics"].get((mname, "wall_ms"), {})
                if int(_f(m.get("n", 0))) <= 0:
                    continue
                vals.append(float(_f(m.get("mean_ms", 0.0))))
            ys.append(sum(vals) / len(vals) if vals else 0.0)
        off = (i - (len(profs) - 1) / 2.0) * w
        ax.bar([x + off for x in xs], ys, width=w, label=prof)
    ax.set_xticks(xs)
    ax.set_xticklabels(metric_set, rotation=20, ha="right")
    ax.set_ylabel("Mean processing / E2E delay (ms)")
    ax.set_title("Computation Overhead Profile")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_computation_overhead.png"), dpi=170)
    plt.close(fig)


def _plot_resource_timeline(resource_csv: str, out_dir: str, plt) -> None:
    if not resource_csv or not os.path.exists(resource_csv):
        return
    rows = _read_csv(resource_csv)
    # Expected columns: run_id,profile,time_s,cpu_pct,mem_mb
    by_key: Dict[str, List[Tuple[float, float, float]]] = {}
    for r in rows:
        rid = str(r.get("run_id", "") or "")
        prof = str(r.get("profile", "") or "")
        key = f"{prof}:{rid}"
        t = _f(r.get("time_s", 0))
        cpu = _f(r.get("cpu_pct", 0))
        mem = _f(r.get("mem_mb", 0))
        by_key.setdefault(key, []).append((t, cpu, mem))
    if not by_key:
        return

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    for key, pts in sorted(by_key.items()):
        pts.sort(key=lambda x: x[0])
        t = [p[0] for p in pts]
        c = [p[1] for p in pts]
        m = [p[2] for p in pts]
        ax1.plot(t, c, label=key, linewidth=1.1)
        ax2.plot(t, m, label=key, linewidth=1.1)
    ax1.set_title("CPU Timeline")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("CPU %")
    ax1.grid(True, linestyle="--", alpha=0.3)
    ax2.set_title("Memory Timeline")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Memory (MB)")
    ax2.grid(True, linestyle="--", alpha=0.3)
    ax2.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "bench_resource_timeline.png"), dpi=170)
    plt.close(fig)


def _plot_paper_like_panel(payload: List[Dict[str, Any]], out_dir: str, metric: str, plt) -> None:
    # 4-panel inspired by the figures user shared (without QoS panel):
    # (a) operational phases, (b) E2E vs msg rate, (c) E2E vs payload, (d) scalability vs active members.
    profs = _profiles(payload)
    if not profs:
        return

    fig = plt.figure(figsize=(15, 8))

    # (a) Operational phases
    ax1 = fig.add_subplot(2, 2, 1)
    phases = ["register_to_onboarding_ms", "onboarding_to_active_ms", "time_to_active_ms"]
    xs = list(range(len(phases)))
    w = 0.8 / max(1, len(profs))
    for i, prof in enumerate(profs):
        means = []
        for ph in phases:
            vals = [float(item["phase"].get(ph, {}).get("mean_ms", 0.0)) for item in payload if item["row"].profile == prof]
            means.append(sum(vals) / len(vals) if vals else 0.0)
        off = (i - (len(profs) - 1) / 2.0) * w
        ax1.bar([x + off for x in xs], means, width=w, label=prof)
    ax1.set_xticks(xs)
    ax1.set_xticklabels(phases, rotation=20, ha="right", fontsize=8)
    ax1.set_ylabel("ms")
    ax1.set_title("(a) Operational Phases")
    ax1.grid(axis="y", linestyle="--", alpha=0.3)
    ax1.legend(fontsize=8)

    # (b) E2E vs msg rate
    ax2 = fig.add_subplot(2, 2, 2)
    for prof in profs:
        pts = []
        for item in payload:
            rr: RunRow = item["row"]
            if rr.profile != prof:
                continue
            m = item["metrics"].get((metric, "wall_ms"), {})
            if int(_f(m.get("n", 0))) <= 0:
                continue
            pts.append((rr.msg_rate, float(_f(m.get("mean_ms", 0)))))
        if pts:
            pts.sort(key=lambda t: t[0])
            ax2.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=prof)
    ax2.set_xlabel("Message rate (msg/s)")
    ax2.set_ylabel("E2E delay (ms)")
    ax2.set_title("(b) Avg E2E Delay - Message Rate")
    ax2.grid(True, linestyle="--", alpha=0.3)

    # (c) E2E vs payload
    ax3 = fig.add_subplot(2, 2, 3)
    for prof in profs:
        pts = []
        for item in payload:
            rr: RunRow = item["row"]
            if rr.profile != prof:
                continue
            m = item["metrics"].get((metric, "wall_ms"), {})
            if int(_f(m.get("n", 0))) <= 0:
                continue
            pts.append((rr.payload_bytes, float(_f(m.get("mean_ms", 0)))))
        if pts:
            pts.sort(key=lambda t: t[0])
            ax3.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=prof)
    ax3.set_xlabel("Payload size (bytes)")
    ax3.set_ylabel("E2E delay (ms)")
    ax3.set_title("(c) Avg E2E Delay - Payload Size")
    ax3.grid(True, linestyle="--", alpha=0.3)

    # (d) scalability vs active members
    ax4 = fig.add_subplot(2, 2, 4)
    any_sc = False
    for prof in profs:
        pts = []
        for item in payload:
            rr: RunRow = item["row"]
            if rr.profile != prof:
                continue
            pop = _population(item.get("summary", {}))
            x = float(pop.get("max_active", 0))
            if x <= 0:
                x = float(pop.get("active_ever", 0))
            m = item["metrics"].get((metric, "wall_ms"), {})
            if int(_f(m.get("n", 0))) <= 0 or x <= 0:
                continue
            pts.append((x, float(_f(m.get("mean_ms", 0.0)))))
        if not pts:
            continue
        any_sc = True
        pts.sort(key=lambda t: t[0])
        ax4.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=prof)
    ax4.set_ylabel("E2E delay (ms)")
    ax4.set_xlabel("Active members (max)")
    ax4.set_title("(d) Scalability by Active Members")
    ax4.grid(True, linestyle="--", alpha=0.3)
    if any_sc:
        ax4.legend(fontsize=8)

    fig.suptitle("Federation Middleware Benchmark Panel", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(os.path.join(out_dir, "bench_paper_like_panel.png"), dpi=170)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="Paper-style benchmark plots for federation middleware")
    ap.add_argument("--manifest-csv", required=True, help="CSV with experimental runs and factors")
    ap.add_argument("--out-dir", required=True, help="output directory for benchmark plots")
    ap.add_argument(
        "--metric",
        default="T_coord_req_resp",
        help="primary E2E metric for msg-rate/payload/qos charts (default: T_coord_req_resp)",
    )
    ap.add_argument("--resource-csv", default="", help="optional resource timeline CSV (run_id,profile,time_s,cpu_pct,mem_mb)")
    ap.add_argument("--include-qos", action="store_true", default=False, help="also generate QoS-effect plot")
    ap.add_argument("--verbose", action="store_true", help="print skipped manifest rows and missing files")
    args = ap.parse_args()

    rows = _load_manifest(args.manifest_csv)
    if not rows:
        print(json.dumps({"status": "error", "reason": "empty_manifest"}, ensure_ascii=True))
        return 2
    payload, skipped = _run_payload(rows)
    if not payload:
        print(
            json.dumps(
                {"status": "error", "reason": "no_valid_metrics_dirs", "skipped": skipped[:50]},
                ensure_ascii=True,
            )
        )
        return 2

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(json.dumps({"status": "error", "reason": "matplotlib_missing", "message": str(e)}, ensure_ascii=True))
        return 3

    _plot_operational_phases(payload, out_dir, plt)
    _plot_metric_vs_x(
        payload,
        os.path.join(out_dir, "bench_e2e_vs_msg_rate.png"),
        metric=str(args.metric),
        x_key="msg_rate",
        x_label="Message rate (msg/s)",
        title=f"Avg {args.metric} - Message Rate",
        plt=plt,
    )
    _plot_metric_vs_x(
        payload,
        os.path.join(out_dir, "bench_e2e_vs_payload_size.png"),
        metric=str(args.metric),
        x_key="payload_bytes",
        x_label="Payload size (bytes)",
        title=f"Avg {args.metric} - Payload Size",
        plt=plt,
    )
    if bool(args.include_qos):
        _plot_qos_effect(payload, out_dir, metric=str(args.metric), plt=plt)
    _plot_active_members_scalability(payload, out_dir, metric=str(args.metric), plt=plt)
    _plot_communication_overhead(payload, out_dir, plt=plt)
    _plot_computation_overhead(payload, out_dir, plt=plt)
    _plot_resource_timeline(str(args.resource_csv), out_dir, plt)
    _plot_paper_like_panel(payload, out_dir, metric=str(args.metric), plt=plt)

    if bool(args.verbose):
        print(json.dumps({"status": "info", "valid_runs": len(payload), "skipped_runs": skipped}, ensure_ascii=True))
    print(
        json.dumps(
            {"status": "ok", "out_dir": out_dir, "valid_runs": len(payload), "generated": sorted(os.listdir(out_dir))},
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
