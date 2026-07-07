#!/usr/bin/env python3
"""Generate paper-facing F2D route comparison plots from final run folders."""

from __future__ import annotations

import argparse
import csv
import html
import math
import shutil
import subprocess
from pathlib import Path


MODE_LABEL = {
    "B0": "FTCM",
    "B1": "LIDP",
    "F2": "FCDP",
    "F2D": "FCDP-D",
}

MODE_COLOR = {
    "B0": "#5B6472",
    "B1": "#E07A32",
    "F2": "#2D7DD2",
    # Match the Route 5 FCDP-P color slot: FCDP-D plays the same
    # downstream-context support role, but with mobile aerial sensing.
    "F2D": "#1B9E77",
}

METRICS = [
    ("travel_time_s", "Travel Time"),
    ("waiting_time_s", "Waiting Time"),
    ("time_loss_s", "Time Loss"),
]


class Svg:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self.parts: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            "<style>",
            """
            text{font-family:Helvetica,Arial,sans-serif;fill:#314150}
            .tick{font-size:18px;font-weight:700}
            .metric-tick{font-size:20px;font-weight:700}
            .ytick{font-size:17px}
            .label{font-size:20px;font-weight:700}
            .legend{font-size:18px;font-weight:700}
            .value{font-size:15px;font-weight:700;fill:#415263}
            .small{font-size:15px}
            .grid{stroke:#D8DEE6;stroke-width:1;stroke-dasharray:4 4;opacity:.65}
            .axis{stroke:#7A8795;stroke-width:1.6}
            .sep{stroke:#B8C2CF;stroke-width:1.4;stroke-dasharray:5 5}
            """,
            "</style>",
        ]

    def add(self, s: str) -> None:
        self.parts.append(s)

    def text(self, x: float, y: float, text: str, cls: str = "", anchor: str = "middle", rotate: float | None = None) -> None:
        t = html.escape(str(text))
        tr = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
        self.add(f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" class="{cls}"{tr}>{t}</text>')

    def rect(self, x: float, y: float, w: float, h: float, fill: str, cls: str = "") -> None:
        self.add(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" class="{cls}"/>')

    def line(self, x1: float, y1: float, x2: float, y2: float, cls: str = "", stroke: str | None = None, width: float | None = None) -> None:
        extra = ""
        if stroke:
            extra += f' stroke="{stroke}"'
        if width:
            extra += f' stroke-width="{width}"'
        self.add(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" class="{cls}"{extra}/>')

    def path(self, d: str, stroke: str, width: float = 3.0, fill: str = "none") -> None:
        self.add(f'<path d="{d}" stroke="{stroke}" stroke-width="{width}" fill="{fill}" stroke-linecap="round" stroke-linejoin="round"/>')

    def circle(self, x: float, y: float, r: float, fill: str) -> None:
        self.add(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{fill}"/>')

    def save(self, path: Path) -> None:
        self.parts.append("</svg>")
        path.write_text("\n".join(self.parts), encoding="utf-8")


def nice_max(v: float) -> float:
    if v <= 0:
        return 1.0
    exp = math.floor(math.log10(v))
    base = 10 ** exp
    frac = v / base
    if frac <= 1.2:
        nice = 1.2
    elif frac <= 2:
        nice = 2
    elif frac <= 5:
        nice = 5
    else:
        nice = 10
    return nice * base


def read_results(run_dir: Path) -> list[dict]:
    csv_path = run_dir / "ev_matrix_results.csv"
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)
    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("travel_time_s", "waiting_time_s", "time_loss_s", "waiting_count_n", "wall_elapsed_s"):
            try:
                r[k] = float(r.get(k, "") or 0.0)
            except ValueError:
                r[k] = 0.0
    return rows


def route_label(rows: list[dict], fallback: str) -> str:
    if rows:
        return f"Route {rows[0].get('route_id') or fallback}"
    return f"Route {fallback}"


def mode_row(rows: list[dict], mode: str) -> dict | None:
    for r in rows:
        if r.get("mode") == mode:
            return r
    return None


def draw_legend(svg: Svg, modes: list[str], cx: float, y: float) -> None:
    item_w = 118
    start = cx - item_w * len(modes) / 2
    for i, mode in enumerate(modes):
        x = start + i * item_w
        svg.rect(x, y - 13, 16, 16, MODE_COLOR[mode])
        svg.text(x + 24, y, MODE_LABEL[mode], "legend", "start")


def grouped_outcomes(
    rows: list[dict],
    path: Path,
    *,
    modes: list[str],
    width: int = 760,
    height: int = 500,
    y_max_override: float | None = None,
) -> None:
    svg = Svg(width, height)
    left, right, top, bottom = 92, 34, 72, 56
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_v = max(float(mode_row(rows, m).get(k, 0.0)) for m in modes if mode_row(rows, m) for k, _ in METRICS)
    y_max = y_max_override if y_max_override is not None else nice_max(max_v * 1.12)
    draw_legend(svg, modes, width / 2, 38)
    for i in range(6):
        val = y_max * i / 5
        y = top + plot_h - (val / y_max) * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 10, y + 6, f"{val:.0f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    svg.text(24, top + plot_h / 2, "Mean Time (s)", "label", rotate=-90)
    group_w = plot_w / len(METRICS)
    bar_gap = 8
    bar_w = min(32, (group_w - 54) / max(1, len(modes)))
    for g, (key, label) in enumerate(METRICS):
        gx = left + group_w * g
        if g:
            svg.line(gx, top + 4, gx, top + plot_h + 8, "sep")
        center = gx + group_w / 2
        start = center - (len(modes) * bar_w + (len(modes) - 1) * bar_gap) / 2
        for i, mode in enumerate(modes):
            r = mode_row(rows, mode)
            if not r:
                continue
            val = float(r.get(key, 0.0))
            h = (val / y_max) * plot_h
            x = start + i * (bar_w + bar_gap)
            y = top + plot_h - h
            svg.rect(x, y, bar_w, h, MODE_COLOR[mode])
            svg.text(x + bar_w / 2, y - 7, f"{val:.1f}", "value")
        svg.text(center, top + plot_h + 34, label, "tick")
    svg.save(path)


def route_comparison(
    route_rows: dict[str, list[dict]],
    path: Path,
    *,
    metric_keys: list[tuple[str, str]],
    modes: list[str] = ["F2", "F2D"],
    width: int = 1040,
    height: int = 500,
    y_max_override: float | None = None,
) -> None:
    svg = Svg(width, height)
    left, right, top, bottom = 92, 34, 72, 56
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_v = max(float(mode_row(rows, m).get(k, 0.0)) for rows in route_rows.values() for m in modes if mode_row(rows, m) for k, _ in metric_keys)
    y_max = y_max_override if y_max_override is not None else nice_max(max_v * 1.12)
    draw_legend(svg, modes, width / 2, 36)
    for i in range(6):
        val = y_max * i / 5
        y = top + plot_h - (val / y_max) * plot_h
        svg.line(left, y, left + plot_w, y, "grid")
        svg.text(left - 10, y + 6, f"{val:.0f}", "ytick", "end")
    svg.line(left, top, left, top + plot_h, "axis")
    svg.line(left, top + plot_h, left + plot_w, top + plot_h, "axis")
    ylabel = "Mean Time (s)" if len(metric_keys) > 1 else metric_keys[0][1]
    svg.text(26, top + plot_h / 2, ylabel, "label", rotate=-90)
    route_count = len(route_rows)
    per_route_w = plot_w / route_count
    for i, route in enumerate(route_rows):
        svg.text(left + per_route_w * (i + 0.5), top + 24, route, "label")
    groups: list[tuple[str, str, str]] = []
    for route, rows in route_rows.items():
        for key, metric in metric_keys:
            groups.append((route, key, metric))
    group_w = plot_w / len(groups)
    bar_gap = 10
    bar_w = min(34, (group_w - 36) / max(1, len(modes)))
    last_route = None
    for g, (route, key, metric) in enumerate(groups):
        gx = left + g * group_w
        if last_route is not None and route != last_route:
            svg.line(gx, top + 4, gx, top + plot_h + 8, "sep")
        last_route = route
        rows = route_rows[route]
        center = gx + group_w / 2
        start = center - (len(modes) * bar_w + (len(modes) - 1) * bar_gap) / 2
        for i, mode in enumerate(modes):
            r = mode_row(rows, mode)
            if not r:
                continue
            val = float(r.get(key, 0.0))
            h = (val / y_max) * plot_h
            x = start + i * (bar_w + bar_gap)
            y = top + plot_h - h
            svg.rect(x, y, bar_w, h, MODE_COLOR[mode])
            svg.text(x + bar_w / 2, y - 7, f"{val:.1f}", "value")
        svg.text(center, top + plot_h + 34, metric, "metric-tick")
    svg.save(path)


def gains_csv(route_rows: dict[str, list[dict]], path: Path) -> None:
    fields = ["route", "metric", "f2", "f2d", "absolute_delta", "gain_percent"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for route, rows in route_rows.items():
            f2 = mode_row(rows, "F2")
            f2d = mode_row(rows, "F2D")
            if not f2 or not f2d:
                continue
            for key, label in METRICS + [("waiting_count_n", "Stops")]:
                a = float(f2.get(key, 0.0))
                b = float(f2d.get(key, 0.0))
                gain = ((a - b) / a * 100.0) if a else 0.0
                w.writerow({"route": route, "metric": label, "f2": f"{a:.3f}", "f2d": f"{b:.3f}", "absolute_delta": f"{a-b:.3f}", "gain_percent": f"{gain:.3f}"})


def write_summary(route_rows: dict[str, list[dict]], path: Path) -> None:
    fields = ["route", "mode", "travel_time_s", "waiting_time_s", "time_loss_s", "waiting_count_n", "wall_elapsed_s", "arrived"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for route, rows in route_rows.items():
            for r in rows:
                w.writerow({k: r.get(k, "") for k in fields if k != "route"} | {"route": route})


def convert_pdfs(out_dir: Path) -> None:
    exe = shutil.which("rsvg-convert")
    if not exe:
        return
    pdf_dir = out_dir / "pdf"
    pdf_dir.mkdir(exist_ok=True)
    for svg in out_dir.glob("*.svg"):
        subprocess.run([exe, "-f", "pdf", "-o", str(pdf_dir / (svg.stem + ".pdf")), str(svg)], check=True)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--route4-run-dir", type=Path, required=True)
    p.add_argument("--route6-run-dir", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument(
        "--route-comparison-width",
        type=int,
        default=1040,
        help="Width in pixels of the Route 4/Route 6 F2 versus F2D comparison plot.",
    )
    p.add_argument("--export-pdf", action="store_true")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for stale_name in (
        "route6_2k_f2d_outcomes_grouped_metrics_3_1.svg",
        "pdf/route6_2k_f2d_outcomes_grouped_metrics_3_1.pdf",
    ):
        stale = args.out_dir / stale_name
        if stale.exists():
            stale.unlink()
    r4 = read_results(args.route4_run_dir)
    r6 = read_results(args.route6_run_dir)
    route_rows = {"Route 4": r4, "Route 6": r6}

    grouped_outcomes(
        r4,
        args.out_dir / "route4_2k_f2d_outcomes_grouped_metrics_3_1.svg",
        modes=["B0", "B1", "F2", "F2D"],
        width=760,
        height=440,
        y_max_override=400,
    )
    # Route 6 is primarily the mobile-observability-gap demonstrator. Its
    # B0/B1/F2 outcomes are intentionally similar, so the paper-facing plot
    # compares only F2 against F2D across both routes.
    route_comparison(
        route_rows,
        args.out_dir / "route4_route6_2k_f2_vs_f2d_outcomes_grouped_metrics_3_1.svg",
        metric_keys=METRICS,
        width=args.route_comparison_width,
        height=440,
        y_max_override=400,
    )
    route_comparison(route_rows, args.out_dir / "route4_route6_2k_f2_vs_f2d_domain_comparison_2_1.svg", metric_keys=METRICS, width=1100, height=520)
    route_comparison(route_rows, args.out_dir / "route4_route6_2k_f2_vs_f2d_stops_2_1.svg", metric_keys=[("waiting_count_n", "Stops (count)")], width=760, height=460)
    write_summary(route_rows, args.out_dir / "f2d_route4_route6_domain_summary.csv")
    gains_csv(route_rows, args.out_dir / "route4_route6_f2d_gains_vs_f2.csv")
    if args.export_pdf:
        convert_pdfs(args.out_dir)
    print(f"Wrote F2D paper route plots to {args.out_dir}")


if __name__ == "__main__":
    main()
