#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Dict, List


def _extract_drop_max_from_log(path: Path) -> int:
    if not path.exists():
        return 0
    patt = re.compile(r"drop_foreign_ev_id=(\d+)")
    mmax = 0
    try:
        with path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "drop_foreign_ev_id=" not in line:
                    continue
                m = patt.search(line)
                if not m:
                    continue
                try:
                    n = int(m.group(1))
                    if n > mmax:
                        mmax = n
                except Exception:
                    pass
    except Exception:
        return 0
    return mmax


def main() -> None:
    ap = argparse.ArgumentParser("Fail matrix when foreign EV drops are detected")
    ap.add_argument("--results-csv", required=True, help="ev_matrix_results.csv path")
    ap.add_argument("--threshold", type=int, default=0, help="fail when drop_foreign_ev_id_max > threshold")
    ap.add_argument("--out-report-csv", default="", help="optional report CSV for failing rows")
    ap.add_argument("--print-ok", action="store_true", default=False)
    args = ap.parse_args()

    p = Path(args.results_csv).expanduser().resolve()
    if not p.exists():
        raise SystemExit(f"missing results csv: {p}")

    rows = list(csv.DictReader(p.open("r", encoding="utf-8", newline="")))
    bad: List[Dict[str, str]] = []
    th = int(args.threshold)

    for r in rows:
        cur = str(r.get("drop_foreign_ev_id_max", "") or "").strip()
        if cur:
            try:
                n = int(float(cur))
            except Exception:
                n = 0
        else:
            rw_log = Path(str(r.get("realworld_log", "") or ""))
            n = _extract_drop_max_from_log(rw_log)
        if n > th:
            rr = dict(r)
            rr["drop_foreign_ev_id_max"] = str(n)
            bad.append(rr)

    if args.out_report_csv:
        out = Path(args.out_report_csv).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        fields = list(rows[0].keys()) if rows else []
        if "drop_foreign_ev_id_max" not in fields:
            fields.append("drop_foreign_ev_id_max")
        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in bad:
                w.writerow(r)

    if bad:
        print(
            f'{{"status":"fail","reason":"foreign_ev_drop_detected","threshold":{th},"failed_runs":{len(bad)},"results_csv":"{p}"}}'
        )
        raise SystemExit(2)
    if bool(args.print_ok):
        print(f'{{"status":"ok","threshold":{th},"checked_runs":{len(rows)},"results_csv":"{p}"}}')
    raise SystemExit(0)


if __name__ == "__main__":
    main()
