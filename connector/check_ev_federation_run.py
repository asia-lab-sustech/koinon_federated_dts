#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
from collections import Counter
from typing import Dict, List, Tuple


def _load_jsonl(path: str) -> List[Dict]:
    out: List[Dict] = []
    if not os.path.isfile(path):
        return out
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(dict(json.loads(line)))
            except Exception:
                continue
    return out


def _count_event_types(events_jsonl: str) -> Counter:
    c = Counter()
    for row in _load_jsonl(events_jsonl):
        et = str(row.get("event_type", "")).strip()
        if et:
            c[et] += 1
    return c


def _find_fed_outcomes_txt(run_dir: str, mode: str) -> str:
    pat = os.path.join(run_dir, mode, f"fed_outcomes_{mode}_*.txt")
    files = sorted(glob.glob(pat))
    return files[0] if files else ""


def _count_keywords(path: str, keys: List[str]) -> Dict[str, int]:
    out = {k: 0 for k in keys}
    if not path or not os.path.isfile(path):
        return out
    txt = ""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        txt = f.read()
    for k in keys:
        out[k] = txt.count(k)
    return out


def _ev_fnm_summary(path: str) -> Dict[str, object]:
    rows = _load_jsonl(path)
    evt = Counter()
    max_req = 0
    ok_n = 0
    err_n = 0
    req_publish_evt_n = 0
    for r in rows:
        e = str(r.get("event", "")).strip()
        if e:
            evt[e] += 1
        if e == "fnm.adapter.state_pull.ok":
            ok_n += 1
            max_req = max(max_req, int(r.get("req_published", 0) or 0))
        elif e == "fnm.adapter.state_pull.error":
            err_n += 1
        elif e == "fnm.adapter.ev_request.publish":
            req_publish_evt_n += 1
    return {
        "rows": len(rows),
        "ok_n": ok_n,
        "error_n": err_n,
        "max_req_published": max_req,
        "request_publish_events": req_publish_evt_n,
        "top_events": evt.most_common(12),
    }


def _load_results(results_csv: str) -> List[Dict[str, str]]:
    if not os.path.isfile(results_csv):
        return []
    with open(results_csv, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    ap = argparse.ArgumentParser(description="Health-check one EV matrix scenario run.")
    ap.add_argument("--scenario-dir", required=True, help="scenario directory containing matrix_out/ and fnm_sidecars/")
    ap.add_argument("--route-id", default="4")
    ap.add_argument("--density-label", default="severe2k")
    ap.add_argument("--require-request-publish", action="store_true", default=False)
    args = ap.parse_args()

    scenario_dir = os.path.abspath(str(args.scenario_dir))
    matrix_out = os.path.join(scenario_dir, "matrix_out")
    route_dir = os.path.join(matrix_out, "runs", str(args.density_label), f"route_{args.route_id}")
    results_csv = os.path.join(matrix_out, "ev_matrix_results.csv")
    ev_fnm_jsonl = os.path.join(scenario_dir, "fnm_sidecars", "fnm_ev.jsonl")

    summary: Dict[str, object] = {
        "scenario_dir": scenario_dir,
        "route_dir": route_dir,
        "results": _load_results(results_csv),
        "ev_fnm": _ev_fnm_summary(ev_fnm_jsonl),
        "modes": {},
    }

    fail = False
    for mode in ("B0", "B1", "F2"):
        mode_dir = os.path.join(route_dir, mode)
        events_jsonl = os.path.join(mode_dir, "fed_outcomes.events.jsonl")
        fed_txt = _find_fed_outcomes_txt(route_dir, mode)
        et = _count_event_types(events_jsonl)
        key_counts = _count_keywords(
            fed_txt,
            [
                "ev.request.in",
                "EV_REQUEST_IN",
                "reservation.req.sent",
                "reservation.resp.recv",
                "plan.applied",
                "warmup_accept",
                "F2_REFINE",
                "refine.candidates",
            ],
        )
        mode_summary = {
            "events_jsonl": events_jsonl,
            "fed_txt": fed_txt,
            "event_types": dict(et),
            "key_counts": key_counts,
        }
        summary["modes"][mode] = mode_summary

    req_publish_events = int(summary["ev_fnm"]["request_publish_events"])  # type: ignore[index]
    if bool(args.require_request_publish) and req_publish_events <= 0:
        fail = True

    print(json.dumps(summary, ensure_ascii=True, indent=2))
    if fail:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

