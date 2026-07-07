#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
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


def _is_clean_row(r: Dict[str, str]) -> bool:
    if str(r.get("return_code", "")).strip() != "0":
        return False
    if str(r.get("travel_time_s", "")).strip() in ("", "None", "nan"):
        return False
    hp = str(r.get("http_precheck_ok", "")).strip()
    hs = str(r.get("http_startup_ok", "")).strip()
    if hp not in ("", "None", "1"):
        return False
    if hs not in ("", "None", "1"):
        return False
    d = str(r.get("drop_foreign_ev_id", "")).strip()
    if d not in ("", "None") and _to_int(d, 0) > 0:
        return False
    return True


def _resolve_run_dir(runs_root: Path, row: Dict[str, str]) -> Optional[Path]:
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
    return None


def _find_events_file(run_dir: Path) -> Optional[Path]:
    cands = sorted(run_dir.glob("fed_outcomes*.events.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
    return cands[0] if cands else None


def _extract_run_latencies(events_file: Path) -> Tuple[List[float], List[float]]:
    # request->actuation and request->response e2e
    req_to_act_ms: List[float] = []
    req_to_resp_ms: List[float] = []
    # FIFO matching per TLS
    req_queue_by_tls: DefaultDict[str, List[float]] = defaultdict(list)

    for ev in _read_jsonl(events_file):
        et = str(ev.get("event_type", "")).strip()
        if not et:
            continue
        sim_t = _to_float(ev.get("sim_time"), -1.0)
        tls_id = str(ev.get("tls_id", "")).strip()

        if et == "ev.request.in":
            if tls_id and sim_t >= 0:
                req_queue_by_tls[tls_id].append(sim_t)
        elif et == "tls.signal.change":
            if tls_id and sim_t >= 0:
                q = req_queue_by_tls.get(tls_id, [])
                if q:
                    t_req = q.pop(0)
                    if sim_t >= t_req:
                        req_to_act_ms.append(1000.0 * (sim_t - t_req))
        elif et == "coord.reservation.req_resp_e2e":
            lat = _to_float(ev.get("latency_ms"), -1.0)
            if lat >= 0:
                req_to_resp_ms.append(lat)

    return req_to_act_ms, req_to_resp_ms


def _boxplot_grouped(
    *,
    out_file: Path,
    title: str,
    ylabel: str,
    groups: List[str],
    modes: List[str],
    values: Dict[Tuple[str, str], List[float]],
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=FIGSIZE_4_3)
    width = 0.22
    offsets = [(-width), 0.0, width]
    while len(offsets) < len(modes):
        offsets.append(offsets[-1] + width)

    used_any = False
    for mi, mode in enumerate(modes):
        data: List[List[float]] = []
        pos: List[float] = []
        for gi, g in enumerate(groups):
            arr = values.get((g, mode), [])
            if arr:
                data.append(arr)
                pos.append(float(gi) + offsets[mi])
        if not data:
            continue
        used_any = True
        bp = ax.boxplot(
            data,
            positions=pos,
            widths=0.18,
            patch_artist=True,
            showfliers=False,
            manage_ticks=False,
        )
        color = {"B0": "#4e79a7", "B1": "#f28e2b", "F2": "#59a14f"}.get(mode, "#999999")
        for b in bp["boxes"]:
            b.set_facecolor(color)
            b.set_alpha(0.75)
        for m in bp["medians"]:
            m.set_color("black")
            m.set_linewidth(1.3)
        ax.plot([], [], color=color, lw=8, label=mode)

    if not used_any:
        return

    ax.set_xticks(list(range(len(groups))))
    ax.set_xticklabels(groups, rotation=0)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_file, dpi=180)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Boxplots per route or per density for EV interaction latency (request->actuation / request->response)."
    )
    ap.add_argument("--runs-root", required=True, help="Root matrix runs directory")
    ap.add_argument("--out-dir", required=True, help="Output directory")
    ap.add_argument("--group-by", choices=["route", "density"], default="route", help="How to group the x-axis")
    ap.add_argument("--modes", default="B0,B1,F2", help="Modes to include (comma-separated)")
    ap.add_argument("--clean-only", action="store_true", help="Keep only clean runs")
    ap.add_argument("--density-filter", default="", help="Optional density filter (e.g., severe2k)")
    ap.add_argument("--route-filter", default="", help="Optional route filter list (e.g., 1,3,4)")
    args = ap.parse_args()

    runs_root = Path(args.runs_root).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = _read_csv(runs_root / "ev_matrix_results.csv")
    if not rows:
        raise RuntimeError(f"Missing ev_matrix_results.csv in {runs_root}")

    modes = [m.strip() for m in str(args.modes).split(",") if m.strip()]
    density_filter = str(args.density_filter).strip()
    route_filter = {int(x.strip()) for x in str(args.route_filter).split(",") if x.strip()} if args.route_filter else set()

    samples: List[Dict[str, Any]] = []
    for r in rows:
        if str(r.get("mode", "")).strip() not in modes:
            continue
        if args.clean_only and not _is_clean_row(r):
            continue
        density = str(r.get("density_label", "")).strip()
        route_id = _to_int(r.get("route_id", r.get("route_idx", "-1")), -1)
        if density_filter and density != density_filter:
            continue
        if route_filter and route_id not in route_filter:
            continue

        run_dir = _resolve_run_dir(runs_root, r)
        if run_dir is None:
            continue
        events_file = _find_events_file(run_dir)
        if events_file is None:
            continue

        req_act, req_resp = _extract_run_latencies(events_file)
        mode = str(r.get("mode", "")).strip()
        scenario_id = str(r.get("scenario_id", "")).strip()
        for v in req_act:
            samples.append(
                {
                    "scenario_id": scenario_id,
                    "mode": mode,
                    "density_label": density,
                    "route_id": route_id,
                    "metric": "request_to_actuation_ms",
                    "value_ms": float(v),
                }
            )
        for v in req_resp:
            samples.append(
                {
                    "scenario_id": scenario_id,
                    "mode": mode,
                    "density_label": density,
                    "route_id": route_id,
                    "metric": "request_to_response_ms",
                    "value_ms": float(v),
                }
            )

    # Save long-form samples CSV for paper tables/secondary plots.
    with (out_dir / "route_interaction_latency_samples.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["scenario_id", "mode", "density_label", "route_id", "metric", "value_ms"])
        w.writeheader()
        for s in samples:
            w.writerow(s)

    # Prepare grouped values
    group_key = "route_id" if args.group_by == "route" else "density_label"
    groups_sorted = sorted({str(s[group_key]) for s in samples}, key=lambda x: int(x) if x.isdigit() else x)

    for metric, title, ylabel, out_name in [
        (
            "request_to_actuation_ms",
            f"EV Request -> TLS Actuation Latency by {args.group_by.title()}",
            "Latency (ms)",
            "boxplot_request_to_actuation_by_group_4x3.png",
        ),
        (
            "request_to_response_ms",
            f"EV Request -> Coordination Response Latency by {args.group_by.title()}",
            "Latency (ms)",
            "boxplot_request_to_response_by_group_4x3.png",
        ),
    ]:
        values: Dict[Tuple[str, str], List[float]] = defaultdict(list)
        for s in samples:
            if str(s["metric"]) != metric:
                continue
            g = str(s[group_key])
            m = str(s["mode"])
            values[(g, m)].append(float(s["value_ms"]))
        _boxplot_grouped(
            out_file=out_dir / out_name,
            title=title,
            ylabel=ylabel,
            groups=groups_sorted,
            modes=modes,
            values=values,
        )

    print(f"OK: wrote route interaction boxplots + samples to {out_dir}")


if __name__ == "__main__":
    main()
