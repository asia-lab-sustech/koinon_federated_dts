#!/usr/bin/env python3
"""
Compatibility wrapper for federation metrics charting.

It supports two modes:
1) Legacy-style logs input -> runs extractor then plotter.
2) Existing metrics-dir -> runs plotter only.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from typing import List


def _run(cmd: List[str]) -> None:
    cp = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if cp.returncode != 0:
        msg = {
            "status": "error",
            "cmd": cmd,
            "returncode": cp.returncode,
            "stdout": cp.stdout,
            "stderr": cp.stderr,
        }
        print(json.dumps(msg, ensure_ascii=True))
        raise SystemExit(cp.returncode)


def main() -> int:
    ap = argparse.ArgumentParser(description="Federation paper-style charts wrapper")
    ap.add_argument("--metrics-dir", default="", help="existing extracted metrics directory")
    ap.add_argument("--out-dir", default="./tmp/fed_metrics_out", help="output directory for extracted metrics")
    ap.add_argument("--plots-dir", default="", help="plot output dir (default: <out-dir>/plots)")
    ap.add_argument(
        "--inputs",
        default="",
        help="comma-separated JSONL files (preferred explicit mode for extraction)",
    )
    ap.add_argument(
        "--auto-fnm-jsonl-root",
        default="",
        help="optional run output root; auto-appends */fnm_sidecars/*.jsonl files",
    )
    # Legacy-compatible inputs:
    ap.add_argument("--membership-log", default="")
    ap.add_argument("--catalog-log", default="")
    ap.add_argument("--discovery-log", default="")
    ap.add_argument("--metrics-log", default="")
    ap.add_argument("--gtco-log", default="")
    ap.add_argument("--exclude-events", default="metrics_pub,catalog.refresh,membership_catalog_seen")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    extractor = os.path.join(here, "extract_federation_metrics.py")
    plotter = os.path.join(here, "plot_federation_metrics.py")

    metrics_dir = str(args.metrics_dir or "").strip()
    if not metrics_dir:
        in_files: List[str] = []
        if str(args.inputs or "").strip():
            in_files.extend([x.strip() for x in str(args.inputs).split(",") if x.strip()])
        for p in [args.membership_log, args.catalog_log, args.discovery_log, args.metrics_log, args.gtco_log]:
            p2 = str(p or "").strip()
            if p2:
                in_files.append(p2)
        auto_root = os.path.abspath(str(args.auto_fnm_jsonl_root or "").strip()) if str(args.auto_fnm_jsonl_root or "").strip() else ""
        if auto_root and os.path.isdir(auto_root):
            for root, _, files in os.walk(auto_root):
                if os.path.basename(root) != "fnm_sidecars":
                    continue
                for fn in files:
                    if fn.endswith(".jsonl"):
                        in_files.append(os.path.join(root, fn))
        # Deduplicate while preserving order.
        deduped: List[str] = []
        seen = set()
        for p in in_files:
            if p and p not in seen:
                deduped.append(p)
                seen.add(p)
        in_files = deduped
        in_files = [p for p in in_files if p]
        if not in_files:
            print(json.dumps({"status": "error", "reason": "no_inputs"}, ensure_ascii=True))
            return 2
        out_dir = os.path.abspath(str(args.out_dir))
        os.makedirs(out_dir, exist_ok=True)
        _run(
            [
                sys.executable,
                extractor,
                "--inputs",
                ",".join(in_files),
                "--out-dir",
                out_dir,
            ]
        )
        metrics_dir = out_dir

    plots_dir = os.path.abspath(str(args.plots_dir or os.path.join(metrics_dir, "plots")))
    os.makedirs(plots_dir, exist_ok=True)
    _run(
        [
            sys.executable,
            plotter,
            "--metrics-dir",
            os.path.abspath(metrics_dir),
            "--out-dir",
            plots_dir,
            "--exclude-events",
            str(args.exclude_events),
        ]
    )

    print(
        json.dumps(
            {
                "status": "ok",
                "metrics_dir": os.path.abspath(metrics_dir),
                "plots_dir": plots_dir,
                "generated": sorted(os.listdir(plots_dir)),
            },
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
