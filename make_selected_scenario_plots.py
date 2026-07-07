#!/usr/bin/env python3
"""Generate paper-ready plots for selected FNM/SUMO scenario results.

The script intentionally uses only the standard library plus NumPy so it can run
in the current lightweight analysis environment without matplotlib/pandas.
"""

from __future__ import annotations

import csv
import argparse
import glob
import html
import json
import math
import os
import shutil
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean
from typing import Iterable

import numpy as np


DEFAULT_LOCAL_OUT_DIR = Path("/Users/chrisvergaram/Documents/Prototype/outputs/final_selected_scenario_plots_20260608")
OUT_DIR = DEFAULT_LOCAL_OUT_DIR

SCENARIOS = [
    ("spillback0p5k", "0.5K", 500, "/Users/chrisvergaram/Downloads/Scenario 0_5K"),
    ("spillback1k", "1K", 1000, "/Users/chrisvergaram/Downloads/Scenario 1K"),
    ("spillback1p25k", "1.25K", 1250, "/Users/chrisvergaram/Downloads/Scenario 1_25K"),
    ("spillback1p5k", "1.5K", 1500, "/Users/chrisvergaram/Downloads/Scenario 1_5K"),
    ("spillback1p75k", "1.75K", 1750, "/Users/chrisvergaram/Downloads/Scenario 1_75K"),
    ("spillback2k", "2K", 2000, "/Users/chrisvergaram/Downloads/Scenario 2K"),
]

MODES = ["B0", "B1", "F2", "F2P"]
MODE_DISPLAY = {
    "B0": "FTCM",
    "B1": "LIDP",
    "F2": "FCDP",
    "F2P": "FCDP-P",
}
MODE_COLORS = {
    "B0": "#5B6472",
    "B1": "#E07A32",
    "F2": "#2D7DD2",
    "F2P": "#1B9E77",
}

# Routes 2, 3, and 9 are excluded from the main comparative figure based on
# prior health inspection: route 2 is a fixed-signal timing outlier, route 3 has
# non-arrival/pathology in 1.75K, and route 9 has an F2P regression at 1.75K.
SELECTED_ROUTES = [1, 4, 5, 6, 7, 8, 10]
EXCLUDED_ROUTES = {
    2: "B0 timing outlier / not representative of progressive modes",
    3: "F2P non-arrival or pathological behavior in 1.75K",
    9: "F2P regression in 1.75K, keep for sensitivity not main plot",
}

REPRESENTATIVE = {
    "scenario_key": "spillback1p75k",
    "scenario_label": "1.75K",
    "route_id": 7,
    "mode": "F2P",
}

FIGURE_PROFILE = "standard"
LEGEND_POSITION = "left"
LEGEND_STYLE = "inline"
SHOW_TITLES = True
SHOW_SUBTITLES = False
EXPORT_PDF = True
GENERATE_PAPER_SUFFIX_PLOTS = True
CURRENT_PAPER_SLOT: str | None = None
CORE_SERVICE_3_1_WIDTH = 780
RUNTIME_EVENTS_3_1_WIDTH = 750
RUNTIME_EVENTS_3_1_LEFT_MARGIN = 116
RUNTIME_EVENTS_3_1_YLABEL_X = 38
RUNTIME_EVENTS_3_1_LEGEND_COLUMN_WIDTH = 220
RUNTIME_EVENTS_3_1_LEGEND_FONT_SIZE = 19
PAIRED_3_1_MIN_HEIGHT = 560
PAPER_2_1_LEFT_MARGIN = 150
PAPER_2_1_SERVICE_YLABEL_X = 72
PAPER_2_1_LEGEND_COLUMN_WIDTH = 285
SELECTED_ROUTE_FONT_SIZES = {
    "tick_size": 24,
    "label_size": 24,
    "xtick_size": 26,
    "legend_size": 23,
    "small_size": 21,
}
PAPER_LAYOUTS = {
    "3_1": {
        "width": 740,
        "height": 380,
        "tick_size": 20,
        "label_size": 21,
        "xlabel_size": 21,
        "ylabel_size": 21,
        "xtick_size": 22,
        "xtick_rotation": 0.0,
        "legend_size": 20,
        "small_size": 18,
        "ylabel_x": 44,
        "legend_position": "top-center",
    },
    "2_1": {
        "width": 980,
        "height": 680,
        "tick_size": 20,
        "label_size": 22,
        "xlabel_size": 22,
        "ylabel_size": 22,
        "xtick_size": 22,
        "xtick_rotation": 0.0,
        "legend_size": 21,
        "small_size": 16,
        "ylabel_x": 24,
        "legend_position": "top-center",
    },
    "1_1": {
        "width": 1380,
        "height": 610,
        "tick_size": 20,
        "label_size": 22,
        "xlabel_size": 22,
        "ylabel_size": 22,
        "xtick_size": 23,
        "xtick_rotation": 0.0,
        "legend_size": 21,
        "small_size": 18,
        "ylabel_x": 28,
        "legend_position": "top-center",
    },
}


def active_paper_layout() -> dict:
    if CURRENT_PAPER_SLOT:
        return PAPER_LAYOUTS.get(CURRENT_PAPER_SLOT, {})
    return {}


def paper_dim(default_width: int, default_height: int) -> tuple[int, int]:
    layout = active_paper_layout()
    return int(layout.get("width", default_width)), int(layout.get("height", default_height))


def paper_y_label_x(default: float) -> float:
    return float(active_paper_layout().get("ylabel_x", default))


def paper_x_label_size(default: int | None = None) -> int | None:
    layout = active_paper_layout()
    if not layout:
        return default
    return int(layout.get("xlabel_size", layout.get("label_size", default or 20)))


def paper_y_label_size(default: int | None = None) -> int | None:
    layout = active_paper_layout()
    if not layout:
        return default
    return int(layout.get("ylabel_size", layout.get("label_size", default or 20)))


def paper_value_size(default: int | None = None) -> int | None:
    layout = active_paper_layout()
    if not layout:
        return default
    return int(layout.get("small_size", default or 16))


def paper_xtick_rotation(default: float = 0.0) -> float:
    layout = active_paper_layout()
    if not layout:
        return default
    return float(layout.get("xtick_rotation", default))


def paper_legend_position(default: str = "top-center") -> str:
    return str(active_paper_layout().get("legend_position", default))


def slot_suffix(path: Path, slot: str) -> Path:
    return path.with_name(f"{path.stem}_{slot}{path.suffix}")


def latex_composite_profile() -> bool:
    return FIGURE_PROFILE == "latex_composite"


def composite_canvas() -> tuple[int, int]:
    # Compact paper subfigure profile: enough room for axis labels, no large
    # trailing whitespace at the bottom.
    return 760, 420


def composite_plot_bottom() -> int:
    return 360


def composite_xtick_offset() -> int:
    return 24


def default_out_dir(root: str | None = None) -> Path:
    root = root or os.environ.get("ROOT")
    if root:
        return Path(root) / "tmp" / "plots"
    return DEFAULT_LOCAL_OUT_DIR


def parse_scenario_spec(spec: str) -> tuple[str, str, int, str]:
    """Parse key,label,density,path for CLI scenario overrides."""
    parts = spec.split(",", 3)
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "Scenario specs must be key,label,density,path; "
            "example: spillback2k,2K,2000,$ROOT/tmp/Scenario_2K"
        )
    key, label, density, folder = [p.strip() for p in parts]
    try:
        density_i = int(density)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid density in scenario spec: {density}") from exc
    return key, label, density_i, os.path.expandvars(os.path.expanduser(folder))


def parse_int_list(value: str) -> list[int]:
    out = []
    for part in value.split(","):
        part = part.strip()
        if part:
            out.append(int(part))
    return out


def add_paper_layout_args(parser: argparse.ArgumentParser, slot: str, defaults: dict) -> None:
    opt = slot.replace("_", "-")
    prefix = f"--paper-{opt}"
    parser.add_argument(f"{prefix}-width", type=int, default=defaults["width"], help=f"Canvas width for _{slot} paper plots.")
    parser.add_argument(f"{prefix}-height", type=int, default=defaults["height"], help=f"Canvas height for _{slot} paper plots.")
    parser.add_argument(f"{prefix}-tick-font-size", type=int, default=defaults["tick_size"], help=f"Y tick font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-xtick-font-size", type=int, default=defaults["xtick_size"], help=f"X tick font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-xtick-rotation", type=float, default=defaults.get("xtick_rotation", 0.0), help=f"X tick rotation angle in degrees for _{slot} plots.")
    parser.add_argument(f"{prefix}-label-font-size", type=int, default=defaults["label_size"], help=f"Axis label font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-xlabel-font-size", type=int, default=defaults.get("xlabel_size", defaults["label_size"]), help=f"X-axis label font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-ylabel-font-size", type=int, default=defaults.get("ylabel_size", defaults["label_size"]), help=f"Y-axis label font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-legend-font-size", type=int, default=defaults["legend_size"], help=f"Legend font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-value-font-size", type=int, default=defaults["small_size"], help=f"Bar value label font size for _{slot} plots.")
    parser.add_argument(f"{prefix}-ylabel-x", type=float, default=defaults["ylabel_x"], help=f"Horizontal y-label position for _{slot} plots; smaller values move the rotated y-label farther left from tick values.")
    parser.add_argument(
        f"{prefix}-legend-position",
        choices=["top-center", "top-left", "top-right"],
        default=defaults["legend_position"],
        help=f"Legend placement for _{slot} plots.",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate selected-route travel-time, domain-metric, and middleware plots. "
            "By default writes to $ROOT/tmp/plots when ROOT is set."
        )
    )
    parser.add_argument("--root", default=os.environ.get("ROOT"), help="Project root; used for default output $ROOT/tmp/plots.")
    parser.add_argument("--out-dir", default=None, help="Output directory. Default: $ROOT/tmp/plots if ROOT is set, otherwise local outputs/.")
    parser.add_argument(
        "--scenario-folder",
        action="append",
        default=[],
        type=parse_scenario_spec,
        help="Override/add scenario folder as key,label,density,path. Repeat for each scenario.",
    )
    parser.add_argument("--selected-routes", default=",".join(map(str, SELECTED_ROUTES)), help="Comma-separated route ids for main plots.")
    parser.add_argument("--representative-scenario-key", default=REPRESENTATIVE["scenario_key"])
    parser.add_argument("--representative-scenario-label", default=REPRESENTATIVE["scenario_label"])
    parser.add_argument("--representative-route-id", type=int, default=REPRESENTATIVE["route_id"])
    parser.add_argument("--representative-mode", default=REPRESENTATIVE["mode"])
    # Clear aliases for the same representative/zoom-in selector. These are
    # easier to read in paper-plot commands and drive the node-level plots too.
    parser.add_argument("--zoom-scenario-key", default=None, help="Alias for --representative-scenario-key.")
    parser.add_argument("--zoom-scenario-label", default=None, help="Alias for --representative-scenario-label.")
    parser.add_argument("--zoom-route-id", type=int, default=None, help="Alias for --representative-route-id.")
    parser.add_argument("--zoom-mode", default=None, help="Alias for --representative-mode.")
    parser.add_argument(
        "--figure-profile",
        choices=["standard", "latex_composite"],
        default="standard",
        help="Use latex_composite for compact subfigure-ready SVGs with no embedded titles.",
    )
    parser.add_argument("--legend-position", choices=["left", "center", "right"], default="left")
    parser.add_argument("--legend-style", choices=["inline", "stacked"], default="inline")
    parser.add_argument("--show-subtitles", action="store_true", help="Include explanatory subtitles inside SVGs. Default is off for paper figures.")
    parser.add_argument("--hide-titles", action="store_true", help="Suppress SVG titles; useful when LaTeX subcaptions provide titles.")
    parser.add_argument("--selected-route-tick-font-size", type=int, default=SELECTED_ROUTE_FONT_SIZES["tick_size"], help="Y-axis tick font size for the selected-route traffic-result figures.")
    parser.add_argument("--selected-route-label-font-size", type=int, default=SELECTED_ROUTE_FONT_SIZES["label_size"], help="Axis-label font size for the selected-route traffic-result figures.")
    parser.add_argument("--selected-route-xtick-font-size", type=int, default=SELECTED_ROUTE_FONT_SIZES["xtick_size"], help="X-axis tick font size for the selected-route traffic-result figures.")
    parser.add_argument("--selected-route-legend-font-size", type=int, default=SELECTED_ROUTE_FONT_SIZES["legend_size"], help="Legend font size for the selected-route traffic-result figures.")
    parser.add_argument("--selected-route-value-font-size", type=int, default=SELECTED_ROUTE_FONT_SIZES["small_size"], help="Bar-top value font size for the selected-route grouped-bar figures.")
    parser.add_argument("--core-service-3-1-width", type=int, default=CORE_SERVICE_3_1_WIDTH, help="Canvas width for each core-service _3_1 panel.")
    parser.add_argument("--runtime-events-3-1-width", type=int, default=RUNTIME_EVENTS_3_1_WIDTH, help="Minimum canvas width for the compact runtime-events _3_1 panel; the paired layout still matches the outcomes-panel width.")
    parser.add_argument("--runtime-events-3-1-left-margin", type=float, default=RUNTIME_EVENTS_3_1_LEFT_MARGIN, help="Left grid margin for the compact runtime-events _3_1 panel.")
    parser.add_argument("--runtime-events-3-1-ylabel-x", type=float, default=RUNTIME_EVENTS_3_1_YLABEL_X, help="Horizontal position of the compact runtime-events y-label; smaller values move it left.")
    parser.add_argument("--runtime-events-3-1-legend-column-width", type=float, default=RUNTIME_EVENTS_3_1_LEGEND_COLUMN_WIDTH, help="Horizontal spacing between runtime-event legend items.")
    parser.add_argument("--runtime-events-3-1-legend-font-size", type=int, default=RUNTIME_EVENTS_3_1_LEGEND_FONT_SIZE, help="Legend font size used only by the compact runtime-events _3_1 panel.")
    parser.add_argument("--paired-3-1-min-height", type=int, default=PAIRED_3_1_MIN_HEIGHT, help="Shared minimum canvas height for the Route 5 outcomes and runtime-events _3_1 panels.")
    parser.add_argument("--paper-2-1-left-margin", type=float, default=PAPER_2_1_LEFT_MARGIN, help="Shared left grid margin for the paired Route 5 _2_1 timeline and service-activity panels.")
    parser.add_argument("--paper-2-1-service-ylabel-x", type=float, default=PAPER_2_1_SERVICE_YLABEL_X, help="Horizontal center of the wrapped service-activity y-label; larger values move it toward the y-ticks.")
    parser.add_argument("--paper-2-1-legend-column-width", type=float, default=PAPER_2_1_LEGEND_COLUMN_WIDTH, help="Horizontal spacing between legend columns in the paired Route 5 runtime panels.")
    parser.add_argument("--export-pdf", dest="export_pdf", action="store_true", default=True, help="Export generated SVG plots to OUT_DIR/pdf/*.pdf. Default: enabled.")
    parser.add_argument("--no-export-pdf", dest="export_pdf", action="store_false", help="Disable SVG-to-PDF export.")
    parser.add_argument("--skip-middleware", action="store_true", help="Only generate scenario/domain plots; skip heavy raw-message/core-log middleware panels.")
    parser.add_argument("--paper-suffix-plots", dest="paper_suffix_plots", action="store_true", default=True, help="Generate extra _3_1, _2_1, and _1_1 plot variants for LaTeX placement. Default: enabled.")
    parser.add_argument("--no-paper-suffix-plots", dest="paper_suffix_plots", action="store_false", help="Disable extra paper-placement suffixed plots.")
    for slot, defaults in PAPER_LAYOUTS.items():
        add_paper_layout_args(parser, slot, defaults)
    return parser.parse_args()


def configure_from_args(args: argparse.Namespace) -> None:
    global OUT_DIR, SCENARIOS, SELECTED_ROUTES, REPRESENTATIVE
    global FIGURE_PROFILE, LEGEND_POSITION, LEGEND_STYLE, SHOW_TITLES, SHOW_SUBTITLES, EXPORT_PDF, GENERATE_PAPER_SUFFIX_PLOTS, PAPER_LAYOUTS, SELECTED_ROUTE_FONT_SIZES, CORE_SERVICE_3_1_WIDTH
    global RUNTIME_EVENTS_3_1_WIDTH, RUNTIME_EVENTS_3_1_LEFT_MARGIN, RUNTIME_EVENTS_3_1_YLABEL_X, RUNTIME_EVENTS_3_1_LEGEND_COLUMN_WIDTH, RUNTIME_EVENTS_3_1_LEGEND_FONT_SIZE, PAIRED_3_1_MIN_HEIGHT
    global PAPER_2_1_LEFT_MARGIN, PAPER_2_1_SERVICE_YLABEL_X, PAPER_2_1_LEGEND_COLUMN_WIDTH
    OUT_DIR = Path(args.out_dir).expanduser() if args.out_dir else default_out_dir(args.root)
    if args.scenario_folder:
        SCENARIOS = args.scenario_folder
    SELECTED_ROUTES = parse_int_list(args.selected_routes)
    FIGURE_PROFILE = str(args.figure_profile)
    LEGEND_POSITION = str(args.legend_position)
    LEGEND_STYLE = str(args.legend_style)
    SHOW_SUBTITLES = bool(args.show_subtitles)
    SHOW_TITLES = not bool(args.hide_titles) and not latex_composite_profile()
    EXPORT_PDF = bool(args.export_pdf)
    GENERATE_PAPER_SUFFIX_PLOTS = bool(args.paper_suffix_plots)
    CORE_SERVICE_3_1_WIDTH = max(500, int(args.core_service_3_1_width))
    RUNTIME_EVENTS_3_1_WIDTH = max(560, int(args.runtime_events_3_1_width))
    RUNTIME_EVENTS_3_1_LEFT_MARGIN = max(80.0, float(args.runtime_events_3_1_left_margin))
    RUNTIME_EVENTS_3_1_YLABEL_X = max(16.0, float(args.runtime_events_3_1_ylabel_x))
    RUNTIME_EVENTS_3_1_LEGEND_COLUMN_WIDTH = max(180.0, float(args.runtime_events_3_1_legend_column_width))
    RUNTIME_EVENTS_3_1_LEGEND_FONT_SIZE = max(12, int(args.runtime_events_3_1_legend_font_size))
    PAIRED_3_1_MIN_HEIGHT = max(440, int(args.paired_3_1_min_height))
    PAPER_2_1_LEFT_MARGIN = max(120.0, float(args.paper_2_1_left_margin))
    PAPER_2_1_SERVICE_YLABEL_X = max(40.0, float(args.paper_2_1_service_ylabel_x))
    PAPER_2_1_LEGEND_COLUMN_WIDTH = max(240.0, float(args.paper_2_1_legend_column_width))
    SELECTED_ROUTE_FONT_SIZES = {
        "tick_size": args.selected_route_tick_font_size,
        "label_size": args.selected_route_label_font_size,
        "xtick_size": args.selected_route_xtick_font_size,
        "legend_size": args.selected_route_legend_font_size,
        "small_size": args.selected_route_value_font_size,
    }
    for slot in list(PAPER_LAYOUTS):
        dest_prefix = f"paper_{slot}"
        PAPER_LAYOUTS[slot] = {
            "width": getattr(args, f"{dest_prefix}_width"),
            "height": getattr(args, f"{dest_prefix}_height"),
            "tick_size": getattr(args, f"{dest_prefix}_tick_font_size"),
            "xtick_size": getattr(args, f"{dest_prefix}_xtick_font_size"),
            "xtick_rotation": getattr(args, f"{dest_prefix}_xtick_rotation"),
            "label_size": getattr(args, f"{dest_prefix}_label_font_size"),
            "xlabel_size": getattr(args, f"{dest_prefix}_xlabel_font_size"),
            "ylabel_size": getattr(args, f"{dest_prefix}_ylabel_font_size"),
            "legend_size": getattr(args, f"{dest_prefix}_legend_font_size"),
            "small_size": getattr(args, f"{dest_prefix}_value_font_size"),
            "ylabel_x": getattr(args, f"{dest_prefix}_ylabel_x"),
            "legend_position": getattr(args, f"{dest_prefix}_legend_position"),
        }
    REPRESENTATIVE = {
        "scenario_key": args.zoom_scenario_key or args.representative_scenario_key,
        "scenario_label": args.zoom_scenario_label or args.representative_scenario_label,
        "route_id": args.zoom_route_id if args.zoom_route_id is not None else args.representative_route_id,
        "mode": args.zoom_mode or args.representative_mode,
    }


def esc(text: object) -> str:
    return html.escape(str(text), quote=True)


def read_results() -> list[dict]:
    rows: list[dict] = []
    for key, label, density, folder in SCENARIOS:
        folder_path = Path(folder)
        matches = sorted(glob.glob(str(folder_path / "ev_matrix_results_*" / "ev_matrix_results.csv")))
        direct_csv = folder_path / "ev_matrix_results.csv"
        if direct_csv.exists():
            matches.append(str(direct_csv))
        if not matches:
            raise FileNotFoundError(f"No ev_matrix_results.csv found under {folder}")
        path = matches[-1]
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                row = dict(row)
                row["source_csv"] = path
                row["scenario_key"] = key
                row["scenario_label"] = label
                row["density_count_norm"] = density
                row["route_id_int"] = int(row["route_id"])
                row["travel_time_float"] = safe_float(row.get("travel_time_s"))
                row["waiting_time_float"] = safe_float(row.get("waiting_time_s"))
                row["waiting_count_float"] = safe_float(row.get("waiting_count_n"))
                row["time_loss_float"] = safe_float(row.get("time_loss_s"))
                row["stop_time_float"] = safe_float(row.get("stop_time_s"))
                row["route_length_float"] = safe_float(row.get("route_length_m"))
                row["wall_elapsed_float"] = safe_float(row.get("wall_elapsed_s"))
                row["arrived_int"] = int(float(row.get("arrived") or 0))
                rows.append(row)
    return rows


def safe_float(value: object, default: float = math.nan) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except Exception:
        return default


def write_csv(path: Path, rows: list[dict], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys = []
        seen = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    keys.append(key)
                    seen.add(key)
        fieldnames = keys
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def export_svgs_to_pdf(out_dir: Path) -> list[dict]:
    """Convert SVG plots in out_dir to PDFs under out_dir/pdf, preserving subfolders."""
    svg_paths = sorted(p for p in out_dir.rglob("*.svg") if p.is_file() and "pdf" not in p.parts)
    pdf_dir = out_dir / "pdf"
    rows: list[dict] = []
    if not svg_paths:
        return rows

    rsvg = shutil.which("rsvg-convert")
    inkscape = shutil.which("inkscape")
    converter = "none"
    if rsvg:
        converter = "rsvg-convert"
    elif inkscape:
        converter = "inkscape"
    else:
        try:
            import cairosvg  # type: ignore
            converter = "cairosvg"
        except Exception:
            converter = "none"

    pdf_dir.mkdir(parents=True, exist_ok=True)
    for svg_path in svg_paths:
        rel = svg_path.relative_to(out_dir)
        pdf_path = pdf_dir / rel.with_suffix(".pdf")
        pdf_path.parent.mkdir(parents=True, exist_ok=True)
        row = {
            "svg": str(svg_path),
            "pdf": str(pdf_path),
            "converter": converter,
            "status": "pending",
            "error": "",
        }
        try:
            if converter == "rsvg-convert":
                subprocess.run(
                    [rsvg, "-f", "pdf", "-o", str(pdf_path), str(svg_path)],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            elif converter == "inkscape":
                subprocess.run(
                    [inkscape, str(svg_path), "--export-type=pdf", f"--export-filename={pdf_path}"],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            elif converter == "cairosvg":
                import cairosvg  # type: ignore
                cairosvg.svg2pdf(url=str(svg_path), write_to=str(pdf_path))
            else:
                row["status"] = "skipped"
                row["error"] = "No SVG-to-PDF converter found. Install librsvg/rsvg-convert, Inkscape, or CairoSVG."
                rows.append(row)
                continue
            row["status"] = "ok"
        except Exception as exc:
            row["status"] = "failed"
            row["error"] = str(exc)
        rows.append(row)

    write_csv(out_dir / "pdf_export_manifest.csv", rows, ["svg", "pdf", "converter", "status", "error"])
    return rows


class Svg:
    def __init__(self, width: int, height: int, title: str, *, font_sizes: dict[str, int] | None = None):
        tick_size = 15 if latex_composite_profile() else 12
        label_size = 17 if latex_composite_profile() else 13
        xtick_size = 18 if latex_composite_profile() else 16
        legend_size = 15 if latex_composite_profile() else 12
        title_size = 24
        subtitle_size = 13
        small_size = 11
        if not latex_composite_profile():
            tick_size = 18
            label_size = 20
            xtick_size = 21
            legend_size = 19
            title_size = 26
            subtitle_size = 14
            small_size = 16
        layout = active_paper_layout()
        if layout:
            tick_size = int(layout.get("tick_size", tick_size))
            label_size = int(layout.get("label_size", label_size))
            xtick_size = int(layout.get("xtick_size", xtick_size))
            legend_size = int(layout.get("legend_size", legend_size))
            small_size = int(layout.get("small_size", small_size))
        if font_sizes:
            tick_size = int(font_sizes.get("tick_size", tick_size))
            label_size = int(font_sizes.get("label_size", label_size))
            xtick_size = int(font_sizes.get("xtick_size", xtick_size))
            legend_size = int(font_sizes.get("legend_size", legend_size))
            small_size = int(font_sizes.get("small_size", small_size))
        self.width = width
        self.height = height
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            "<defs>",
            "<style>",
            "text{font-family:Arial,Helvetica,sans-serif;fill:#263238}",
            f".title{{font-size:{title_size}px;font-weight:700}}.subtitle{{font-size:{subtitle_size}px;fill:#56616b}}",
            ".axis{stroke:#70808f;stroke-width:1}.grid{stroke:#d9e1e8;stroke-width:1}",
            f".tick{{font-size:{tick_size}px;fill:#596773}}.label{{font-size:{label_size}px;fill:#43515c;font-weight:600}}",
            f".xtick{{font-size:{xtick_size}px;fill:#43515c;font-weight:700}}",
            f".small{{font-size:{small_size}px;fill:#596773}}.legend{{font-size:{legend_size}px;fill:#263238}}",
            f".legend{{font-size:{legend_size}px;fill:#263238}}",
            "</style>",
            "</defs>",
        ]
        if title and SHOW_TITLES:
            self.text(36, 34, title, cls="title")

    def rect(self, x, y, w, h, fill="#fff", stroke="none", sw=1, rx=0, opacity=1):
        self.parts.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" rx="{rx}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"/>'
        )

    def line(self, x1, y1, x2, y2, stroke="#000", sw=1, opacity=1, dash=None):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{sw}" opacity="{opacity}"{dash_attr}/>'
        )

    def polyline(self, pts: Iterable[tuple[float, float]], stroke="#000", sw=2, fill="none", opacity=1):
        points = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
        self.parts.append(
            f'<polyline points="{points}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'opacity="{opacity}" stroke-linejoin="round" stroke-linecap="round"/>'
        )

    def circle(self, x, y, r=3, fill="#000", stroke="none", sw=1, opacity=1):
        self.parts.append(
            f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{sw}" opacity="{opacity}"/>'
        )

    def text(self, x, y, text, cls="", anchor="start", size=None, fill=None, rotate=None):
        if cls == "subtitle" and not SHOW_SUBTITLES:
            return
        cls_attr = f' class="{cls}"' if cls else ""
        size_attr = f' font-size="{size}"' if size else ""
        fill_attr = f' fill="{fill}"' if fill else ""
        transform = f' transform="rotate({rotate} {x:.2f} {y:.2f})"' if rotate else ""
        self.parts.append(
            f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}"{cls_attr}{size_attr}{fill_attr}{transform}>{esc(text)}</text>'
        )

    def finish(self) -> str:
        self.parts.append("</svg>")
        return "\n".join(self.parts)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.finish())


def y_scale(v, vmin, vmax, top, bottom):
    if vmax <= vmin:
        return bottom
    return bottom - (v - vmin) / (vmax - vmin) * (bottom - top)


def compact_number(value: float) -> str:
    v = float(value)
    sign = "-" if v < 0 else ""
    v = abs(v)
    if v >= 1_000_000:
        return f"{sign}{v / 1_000_000:.1f}M".replace(".0M", "M")
    if v >= 1_000:
        return f"{sign}{v / 1_000:.0f}K"
    return f"{sign}{v:.0f}"


def compact_k_tick(value: float) -> str:
    """Format large count-axis ticks compactly while retaining one decimal."""
    v = float(value)
    if abs(v) >= 1000:
        return f"{v / 1000.0:.1f} K"
    return f"{v:.0f}"


def draw_y_axis(svg: Svg, x, top, bottom, vmin, vmax, ticks=6, label="", tick_fmt=None, label_x=18, grid_right=None, label_size=None):
    grid_right = svg.width - 40 if grid_right is None else grid_right
    for i in range(ticks + 1):
        value = vmin + (vmax - vmin) * i / ticks
        y = y_scale(value, vmin, vmax, top, bottom)
        svg.line(x, y, grid_right, y, stroke="#edf2f6", sw=1, opacity=0.68)
        svg.line(x - 5, y, x, y, stroke="#70808f")
        svg.text(x - 9, y + 4, tick_fmt(value) if tick_fmt else f"{value:.0f}", cls="tick", anchor="end")
    svg.line(x, top, x, bottom, stroke="#70808f")
    svg.line(x, bottom, grid_right, bottom, stroke="#70808f")
    if label:
        size = label_size or paper_y_label_size()
        parts = str(label).replace("|", "\n").splitlines()
        if len(parts) <= 1:
            svg.text(label_x, (top + bottom) / 2, label, cls="label", size=size, anchor="middle", rotate=-90)
        else:
            line_h = (size or 20) * 0.95
            center_y = (top + bottom) / 2
            for i, part in enumerate(parts):
                dx = (i - (len(parts) - 1) / 2.0) * line_h
                svg.text(label_x + dx, center_y, part, cls="label", size=size, anchor="middle", rotate=-90)


def plot_travel_time_boxplot(rows: list[dict], path: Path) -> None:
    selected = [
        r for r in rows
        if r["route_id_int"] in SELECTED_ROUTES
        and r["mode"] in MODES
        and r["arrived_int"] == 1
        and math.isfinite(r["travel_time_float"])
    ]
    values = [r["travel_time_float"] for r in selected]
    vmin = max(0, math.floor(min(values) / 50) * 50 - 20)
    vmax = math.ceil(max(values) / 50) * 50 + 30
    if latex_composite_profile():
        svg = Svg(*composite_canvas(), "Emergency Vehicle - DT travel time by scenario", font_sizes=SELECTED_ROUTE_FONT_SIZES)
        top = 122 if LEGEND_STYLE == "stacked" else 72
        bottom, left, right = composite_plot_bottom(), 82, 718
        legend_y = 28
        box_w = 15
        offsets = [-25, -8, 8, 25]
    else:
        svg = Svg(1320, 760, "Emergency Vehicle - DT travel time by scenario", font_sizes=SELECTED_ROUTE_FONT_SIZES)
        top, bottom, left, right = 118, 620, 82, 1260
        legend_y = 82
        box_w = 24
        offsets = [-42, -14, 14, 42]
    draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=6 if latex_composite_profile() else 7, label="Mean Travel Time (s)")

    # Legend top-left using paper-facing mode names.
    draw_mode_legend(svg, left + 8, legend_y, spacing=128, x1=right)

    group_w = (right - left) / len(SCENARIOS)
    rng = np.random.default_rng(42)
    for si, (key, label, _, _) in enumerate(SCENARIOS):
        center = left + group_w * (si + 0.5)
        svg.text(center, bottom + (composite_xtick_offset() if latex_composite_profile() else 38), label, cls="xtick", anchor="middle")
        for mi, mode in enumerate(MODES):
            mode_values = np.array([
                r["travel_time_float"] for r in selected
                if r["scenario_key"] == key and r["mode"] == mode
            ], dtype=float)
            if len(mode_values) == 0:
                continue
            x = center + offsets[mi]
            q1, med, q3 = np.percentile(mode_values, [25, 50, 75])
            low, high = float(np.min(mode_values)), float(np.max(mode_values))
            yq1, ymed, yq3 = (y_scale(v, vmin, vmax, top, bottom) for v in (q1, med, q3))
            ylow, yhigh = y_scale(low, vmin, vmax, top, bottom), y_scale(high, vmin, vmax, top, bottom)
            color = MODE_COLORS[mode]
            svg.line(x, ylow, x, yhigh, stroke=color, sw=1.7)
            svg.line(x - box_w * 0.35, ylow, x + box_w * 0.35, ylow, stroke=color, sw=1.7)
            svg.line(x - box_w * 0.35, yhigh, x + box_w * 0.35, yhigh, stroke=color, sw=1.7)
            svg.rect(x - box_w / 2, yq3, box_w, yq1 - yq3, fill=color, stroke=color, sw=1, opacity=0.30, rx=3)
            svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2.2)
            m = float(np.mean(mode_values))
            ym = y_scale(m, vmin, vmax, top, bottom)
            svg.parts.append(
                f'<path d="M {x:.2f},{ym-5:.2f} L {x+5:.2f},{ym:.2f} L {x:.2f},{ym+5:.2f} L {x-5:.2f},{ym:.2f} Z" '
                f'fill="{color}" stroke="#fff" stroke-width="1"/>'
            )
            for value in mode_values:
                jitter = float(rng.uniform(-4, 4))
                svg.circle(x + jitter, y_scale(value, vmin, vmax, top, bottom), r=2.2, fill=color, opacity=0.55)

    svg.text(left, 716, f"Selected routes {SELECTED_ROUTES}; excluded routes {sorted(EXCLUDED_ROUTES)}. Diamond marker = route-set mean.", cls="subtitle")
    svg.save(path)


def plot_domain_metric_boxplots(rows: list[dict], path: Path) -> None:
    metrics = [
        ("waiting_time_float", "Waiting Time (s)", "waiting_time_s"),
        ("time_loss_float", "Time Loss (s)", "time_loss_s"),
        ("waiting_count_n", "Stops / Waiting Count", "waiting_count_n"),
    ]
    panel_w, panel_h = 1320, 390
    svg = Svg(panel_w, panel_h * len(metrics) + 80, "Domain-specific emergency response metrics")
    svg.text(36, 56, f"Boxplots across selected routes {SELECTED_ROUTES}; excluded routes {sorted(EXCLUDED_ROUTES)}.", cls="subtitle")
    lx, ly = 82, 86
    for i, mode in enumerate(MODES):
        x = lx + i * 128
        svg.rect(x, ly - 14, 18, 13, fill=MODE_COLORS[mode], opacity=0.45, stroke=MODE_COLORS[mode])
        svg.text(x + 25, ly - 3, MODE_DISPLAY[mode], cls="legend")

    for pi, (field, ylabel, csv_field) in enumerate(metrics):
        selected = []
        for r in rows:
            if r["route_id_int"] not in SELECTED_ROUTES or r["mode"] not in MODES or r["arrived_int"] != 1:
                continue
            value = safe_float(r.get(field, r.get(csv_field)))
            if math.isfinite(value):
                rr = dict(r)
                rr["_domain_value"] = value
                selected.append(rr)
        if not selected:
            continue
        vals = [r["_domain_value"] for r in selected]
        vmin = 0
        vmax = max(1, math.ceil(max(vals) / 25) * 25)
        top = 126 + pi * panel_h
        bottom = top + 260
        left, right = 82, 1260
        svg.text(left, top - 18, ylabel, cls="label")
        draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=5, label=ylabel)
        group_w = (right - left) / len(SCENARIOS)
        box_w = 22
        offsets = [-40, -13, 13, 40]
        rng = np.random.default_rng(100 + pi)
        for si, (key, label, _, _) in enumerate(SCENARIOS):
            center = left + group_w * (si + 0.5)
            svg.text(center, bottom + 34, label, cls="xtick", anchor="middle")
            for mi, mode in enumerate(MODES):
                mode_values = np.array([
                    r["_domain_value"] for r in selected
                    if r["scenario_key"] == key and r["mode"] == mode
                ], dtype=float)
                if len(mode_values) == 0:
                    continue
                x = center + offsets[mi]
                q1, med, q3 = np.percentile(mode_values, [25, 50, 75])
                low, high = float(np.min(mode_values)), float(np.max(mode_values))
                yq1, ymed, yq3 = (y_scale(v, vmin, vmax, top, bottom) for v in (q1, med, q3))
                ylow, yhigh = y_scale(low, vmin, vmax, top, bottom), y_scale(high, vmin, vmax, top, bottom)
                color = MODE_COLORS[mode]
                svg.line(x, ylow, x, yhigh, stroke=color, sw=1.5)
                svg.rect(x - box_w / 2, yq3, box_w, max(1, yq1 - yq3), fill=color, stroke=color, sw=1, opacity=0.30, rx=3)
                svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2)
                for value in mode_values:
                    svg.circle(x + float(rng.uniform(-3.5, 3.5)), y_scale(value, vmin, vmax, top, bottom), r=2.0, fill=color, opacity=0.50)
    svg.save(path)


def plot_wall_runtime_boxplot(rows: list[dict], path: Path) -> None:
    selected = [
        r for r in rows
        if r["route_id_int"] in SELECTED_ROUTES
        and r["mode"] in MODES
        and math.isfinite(r["wall_elapsed_float"])
    ]
    values = [r["wall_elapsed_float"] for r in selected]
    vmin = max(0, math.floor(min(values) / 50) * 50 - 20)
    vmax = math.ceil(max(values) / 50) * 50 + 30
    if latex_composite_profile():
        svg = Svg(*composite_canvas(), "End-to-end runner wall-clock time by scenario and mode", font_sizes=SELECTED_ROUTE_FONT_SIZES)
        top = 122 if LEGEND_STYLE == "stacked" else 72
        bottom, left, right = composite_plot_bottom(), 82, 718
        legend_y = 28
        box_w = 15
        offsets = [-25, -8, 8, 25]
    else:
        svg = Svg(1320, 760, "End-to-end runner wall-clock time by scenario and mode", font_sizes=SELECTED_ROUTE_FONT_SIZES)
        top, bottom, left, right = 118, 620, 82, 1260
        legend_y = 82
        box_w = 24
        offsets = [-42, -14, 14, 42]
    svg.text(36, 56, "Wall elapsed time from ev_matrix_results.csv. End-to-end run duration, not isolated CPU or middleware-only overhead.", cls="subtitle")
    draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=6 if latex_composite_profile() else 7, label="End-to-End Execution Time (s)")
    draw_mode_legend(svg, left + 8, legend_y, spacing=128, x1=right)
    group_w = (right - left) / len(SCENARIOS)
    rng = np.random.default_rng(7)
    for si, (key, label, _, _) in enumerate(SCENARIOS):
        center = left + group_w * (si + 0.5)
        svg.text(center, bottom + (composite_xtick_offset() if latex_composite_profile() else 38), label, cls="xtick", anchor="middle")
        for mi, mode in enumerate(MODES):
            mode_values = np.array([
                r["wall_elapsed_float"] for r in selected
                if r["scenario_key"] == key and r["mode"] == mode
            ], dtype=float)
            if len(mode_values) == 0:
                continue
            x = center + offsets[mi]
            q1, med, q3 = np.percentile(mode_values, [25, 50, 75])
            low, high = float(np.min(mode_values)), float(np.max(mode_values))
            yq1, ymed, yq3 = (y_scale(v, vmin, vmax, top, bottom) for v in (q1, med, q3))
            ylow, yhigh = y_scale(low, vmin, vmax, top, bottom), y_scale(high, vmin, vmax, top, bottom)
            color = MODE_COLORS[mode]
            svg.line(x, ylow, x, yhigh, stroke=color, sw=1.7)
            svg.line(x - box_w * 0.35, ylow, x + box_w * 0.35, ylow, stroke=color, sw=1.7)
            svg.line(x - box_w * 0.35, yhigh, x + box_w * 0.35, yhigh, stroke=color, sw=1.7)
            svg.rect(x - box_w / 2, yq3, box_w, yq1 - yq3, fill=color, stroke=color, sw=1, opacity=0.30, rx=3)
            svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2.2)
            m = float(np.mean(mode_values))
            ym = y_scale(m, vmin, vmax, top, bottom)
            svg.parts.append(
                f'<path d="M {x:.2f},{ym-5:.2f} L {x+5:.2f},{ym:.2f} L {x:.2f},{ym+5:.2f} L {x-5:.2f},{ym:.2f} Z" '
                f'fill="{color}" stroke="#fff" stroke-width="1"/>'
            )
            for value in mode_values:
                svg.circle(x + float(rng.uniform(-4, 4)), y_scale(value, vmin, vmax, top, bottom), r=2.2, fill=color, opacity=0.55)
    svg.save(path)


def plot_mean_lines(summary_rows: list[dict], path: Path) -> None:
    svg = Svg(1180, 650, "Mean EV travel time over congestion scenarios")
    svg.text(36, 56, "Selected-route mean across clean route subset; lower is better.", cls="subtitle")
    top, bottom, left, right = 90, 530, 78, 1110
    vals = [float(r[f"mean_{m}"]) for r in summary_rows for m in MODES if r.get(f"mean_{m}")]
    vmin = max(0, math.floor(min(vals) / 50) * 50 - 20)
    vmax = math.ceil(max(vals) / 50) * 50 + 30
    draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=6, label="Mean Travel Time (s)")
    xs = []
    for i, r in enumerate(summary_rows):
        x = left + (right - left) * i / (len(summary_rows) - 1)
        xs.append(x)
        svg.text(x, bottom + 30, r["scenario_label"], cls="label", anchor="middle")
        svg.line(x, bottom, x, bottom + 5, stroke="#70808f")
    for mode in MODES:
        pts = []
        for x, r in zip(xs, summary_rows):
            y = y_scale(float(r[f"mean_{mode}"]), vmin, vmax, top, bottom)
            pts.append((x, y))
        svg.polyline(pts, stroke=MODE_COLORS[mode], sw=3)
        for x, y in pts:
            svg.circle(x, y, r=4.2, fill=MODE_COLORS[mode], stroke="#fff", sw=1)
    for i, mode in enumerate(MODES):
        x = left + 20 + i * 120
        svg.line(x, 588, x + 24, 588, stroke=MODE_COLORS[mode], sw=4)
        svg.circle(x + 12, 588, r=4, fill=MODE_COLORS[mode])
        svg.text(x + 34, 592, MODE_DISPLAY[mode], cls="legend")
    svg.save(path)


def summarize_domain_means(rows: list[dict]) -> list[dict]:
    selected = [
        r for r in rows
        if r["route_id_int"] in SELECTED_ROUTES
        and r["mode"] in MODES
        and r["arrived_int"] == 1
    ]
    metric_fields = [
        ("travel_time_s", "travel_time_float"),
        ("waiting_time_s", "waiting_time_float"),
        ("time_loss_s", "time_loss_float"),
        ("stops_waiting_count_n", "waiting_count_float"),
        ("stop_time_s", "stop_time_float"),
        ("wall_elapsed_s", "wall_elapsed_float"),
    ]
    out_rows = []
    for key, label, density, _ in SCENARIOS:
        row = {
            "scenario_key": key,
            "scenario_label": label,
            "density_count": density,
            "selected_routes": " ".join(map(str, SELECTED_ROUTES)),
        }
        for mode in MODES:
            mode_rows = [r for r in selected if r["scenario_key"] == key and r["mode"] == mode]
            row[f"n_{MODE_DISPLAY[mode]}"] = len(mode_rows)
            for metric_name, field in metric_fields:
                vals = [r[field] for r in mode_rows if math.isfinite(r[field])]
                row[f"mean_{metric_name}_{MODE_DISPLAY[mode]}"] = f"{mean(vals):.3f}" if vals else ""
                row[f"median_{metric_name}_{MODE_DISPLAY[mode]}"] = f"{float(np.median(vals)):.3f}" if vals else ""
        out_rows.append(row)
    return out_rows


def get_domain_mean(summary_rows: list[dict], metric: str, mode: str) -> list[float]:
    col = f"mean_{metric}_{MODE_DISPLAY[mode]}"
    return [safe_float(r.get(col)) for r in summary_rows]


def nice_max(values: list[float], step: int = 25, pad: float = 1.12) -> float:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return 1.0
    return max(step, math.ceil(max(vals) * pad / step) * step)


def plot_metric_mean_lines(
    summary_rows: list[dict],
    metric: str,
    title: str,
    ylabel: str,
    path: Path,
    y_step: int = 25,
) -> None:
    if latex_composite_profile():
        svg = Svg(*composite_canvas(), title)
        top = 122 if LEGEND_STYLE == "stacked" else 72
        bottom, left, right = composite_plot_bottom(), 82, 718
        legend_y = 28
    else:
        svg = Svg(1180, 650, title)
        top, bottom, left, right = 92, 530, 78, 1110
        legend_y = 588
    svg.text(36, 56, f"Mean across selected routes {SELECTED_ROUTES}; lower is better.", cls="subtitle")
    all_vals = [v for mode in MODES for v in get_domain_mean(summary_rows, metric, mode)]
    vmax = nice_max(all_vals, step=y_step)
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label=ylabel)
    xs = []
    for i, r in enumerate(summary_rows):
        x = left + (right - left) * i / (len(summary_rows) - 1)
        xs.append(x)
        svg.text(x, bottom + (composite_xtick_offset() if latex_composite_profile() else 34), r["scenario_label"], cls="xtick", anchor="middle")
        svg.line(x, bottom, x, bottom + 5, stroke="#70808f")
    for mode in MODES:
        vals = get_domain_mean(summary_rows, metric, mode)
        pts = [(x, y_scale(v, 0, vmax, top, bottom)) for x, v in zip(xs, vals) if math.isfinite(v)]
        svg.polyline(pts, stroke=MODE_COLORS[mode], sw=3)
        for x, v in zip(xs, vals):
            if math.isfinite(v):
                svg.circle(x, y_scale(v, 0, vmax, top, bottom), r=4.2, fill=MODE_COLORS[mode], stroke="#fff", sw=1)
    draw_mode_legend(svg, left + 20, legend_y, x1=right)
    svg.save(path)


def draw_mode_legend(svg: Svg, x0: float, y: float, spacing: float = 142.0, x1: float | None = None) -> None:
    item_w = 120 if latex_composite_profile() else spacing
    if LEGEND_STYLE == "stacked":
        total_w = 120
    else:
        total_w = item_w * len(MODES)
    right_bound = x1 if x1 is not None else svg.width - 48
    if LEGEND_POSITION == "center":
        x0 = max(20, (x0 + right_bound - total_w) / 2)
    elif LEGEND_POSITION == "right":
        x0 = max(20, right_bound - total_w)
    for i, mode in enumerate(MODES):
        if LEGEND_STYLE == "stacked":
            x = x0
            yy = y + i * (22 if latex_composite_profile() else 20)
        else:
            x = x0 + i * item_w
            yy = y
        svg.rect(x, yy - 14, 18, 13, fill=MODE_COLORS[mode], opacity=0.45, stroke=MODE_COLORS[mode])
        svg.text(x + 25, yy - 3, MODE_DISPLAY[mode], cls="legend")


def plot_metric_grouped_bars(
    summary_rows: list[dict],
    metric: str,
    title: str,
    ylabel: str,
    path: Path,
    y_step: int = 25,
    font_sizes: dict[str, int] | None = None,
) -> None:
    # Keep selected-route grouped metrics visually interchangeable with the
    # travel-time and wall-runtime panels used in the same paper figure.
    effective_font_sizes = font_sizes or SELECTED_ROUTE_FONT_SIZES
    if latex_composite_profile():
        svg = Svg(*composite_canvas(), title, font_sizes=effective_font_sizes)
        top = 122 if LEGEND_STYLE == "stacked" else 72
        bottom, left, right = composite_plot_bottom(), 82, 718
        legend_y = 28
        bar_w = 12
        offsets = [-24, -8, 8, 24]
    else:
        svg = Svg(1320, 760, title, font_sizes=effective_font_sizes)
        top, bottom, left, right = 118, 620, 82, 1260
        legend_y = 82
        bar_w = 24
        offsets = [-42, -14, 14, 42]
    svg.text(36, 56, f"Grouped bars show route-set mean across selected routes {SELECTED_ROUTES}.", cls="subtitle")
    all_vals = [v for mode in MODES for v in get_domain_mean(summary_rows, metric, mode)]
    vmax = nice_max(all_vals, step=y_step)
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label=ylabel)
    group_w = (right - left) / len(summary_rows)
    for si, r in enumerate(summary_rows):
        center = left + group_w * (si + 0.5)
        svg.text(center, bottom + (composite_xtick_offset() if latex_composite_profile() else 38), r["scenario_label"], cls="xtick", anchor="middle")
        for mi, mode in enumerate(MODES):
            v = get_domain_mean(summary_rows, metric, mode)[si]
            if not math.isfinite(v):
                continue
            x = center + offsets[mi] - bar_w / 2
            y = y_scale(v, 0, vmax, top, bottom)
            svg.rect(x, y, bar_w, bottom - y, fill=MODE_COLORS[mode], opacity=0.72, rx=3)
            svg.text(x + bar_w / 2, y - 5, f"{v:.0f}", cls="small", anchor="middle")
    draw_mode_legend(svg, left + 8, legend_y, x1=right)
    svg.save(path)


def plot_dual_axis_metric(
    summary_rows: list[dict],
    secondary_metric: str,
    secondary_label: str,
    title: str,
    path: Path,
    secondary_step: int,
) -> None:
    primary_metric = "time_loss_s"
    if latex_composite_profile():
        svg = Svg(*composite_canvas(), title)
        top = 128 if LEGEND_STYLE == "stacked" else 76
        bottom, left, right = composite_plot_bottom(), 82, 682
        legend_y = 30
        line_note_x, line_note_y = left + 350, 31
    else:
        svg = Svg(1320, 720, title)
        top, bottom, left, right = 110, 565, 86, 1230
        legend_y = 84
        line_note_x, line_note_y = left + 610, 81
    svg.text(36, 56, "Solid lines = mean time loss (left axis). Dashed lines = secondary metric (right axis). Means are across selected routes.", cls="subtitle")
    primary_vals = [v for mode in MODES for v in get_domain_mean(summary_rows, primary_metric, mode)]
    secondary_vals = [v for mode in MODES for v in get_domain_mean(summary_rows, secondary_metric, mode)]
    primary_max = nice_max(primary_vals, step=25)
    secondary_max = nice_max(secondary_vals, step=secondary_step)
    draw_y_axis(svg, left, top, bottom, 0, primary_max, ticks=6, label="Mean Time Loss (s)")
    draw_right_y_axis(svg, right, top, bottom, 0, secondary_max, ticks=6, label=secondary_label)

    xs = []
    for i, r in enumerate(summary_rows):
        x = left + (right - left) * i / (len(summary_rows) - 1)
        xs.append(x)
        svg.text(x, bottom + (composite_xtick_offset() if latex_composite_profile() else 38), r["scenario_label"], cls="xtick", anchor="middle")
    for mode in MODES:
        color = MODE_COLORS[mode]
        primary = get_domain_mean(summary_rows, primary_metric, mode)
        secondary = get_domain_mean(summary_rows, secondary_metric, mode)
        p_pts = [(x, y_scale(v, 0, primary_max, top, bottom)) for x, v in zip(xs, primary) if math.isfinite(v)]
        s_pts = [(x, y_scale(v, 0, secondary_max, top, bottom)) for x, v in zip(xs, secondary) if math.isfinite(v)]
        svg.polyline(p_pts, stroke=color, sw=3.0)
        dashed_polyline(svg, s_pts, stroke=color, sw=2.4, dash="7 5", opacity=0.85)
        for x, v in zip(xs, primary):
            if math.isfinite(v):
                svg.circle(x, y_scale(v, 0, primary_max, top, bottom), r=4.2, fill=color, stroke="#fff", sw=1)
        for x, v in zip(xs, secondary):
            if math.isfinite(v):
                svg.circle(x, y_scale(v, 0, secondary_max, top, bottom), r=3.2, fill="#fff", stroke=color, sw=1.8)
    draw_mode_legend(svg, left + 8, legend_y, x1=right)
    svg.text(line_note_x, line_note_y, "Solid: time loss (s)   Dashed: secondary", cls="legend")
    svg.save(path)


def draw_right_y_axis(svg: Svg, x, top, bottom, vmin, vmax, ticks=6, label="") -> None:
    for i in range(ticks + 1):
        value = vmin + (vmax - vmin) * i / ticks
        y = y_scale(value, vmin, vmax, top, bottom)
        svg.line(x, y, x + 5, y, stroke="#70808f")
        svg.text(x + 9, y + 4, f"{value:.0f}", cls="tick", anchor="start")
    svg.line(x, top, x, bottom, stroke="#70808f")
    if label:
        svg.text(svg.width - 18, (top + bottom) / 2, label, cls="label", anchor="middle", rotate=90)


def dashed_polyline(svg: Svg, pts: Iterable[tuple[float, float]], stroke="#000", sw=2, dash="6 4", opacity=1) -> None:
    points = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
    svg.parts.append(
        f'<polyline points="{points}" fill="none" stroke="{stroke}" stroke-width="{sw}" '
        f'opacity="{opacity}" stroke-linejoin="round" stroke-linecap="round" stroke-dasharray="{dash}"/>'
    )


def summarize_selected(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    selected = [
        r for r in rows
        if r["route_id_int"] in SELECTED_ROUTES
        and r["mode"] in MODES
        and r["arrived_int"] == 1
        and math.isfinite(r["travel_time_float"])
    ]
    route_table = []
    for key, label, density, _ in SCENARIOS:
        for route in SELECTED_ROUTES:
            out = {"scenario_key": key, "scenario_label": label, "density_count": density, "route_id": route}
            for mode in MODES:
                matches = [r for r in selected if r["scenario_key"] == key and r["route_id_int"] == route and r["mode"] == mode]
                out[f"{mode}_travel_time_s"] = f"{matches[0]['travel_time_float']:.1f}" if matches else ""
                out[f"{mode}_waiting_time_s"] = f"{matches[0]['waiting_time_float']:.1f}" if matches else ""
                out[f"{mode}_time_loss_s"] = f"{matches[0]['time_loss_float']:.1f}" if matches else ""
            route_table.append(out)
    summary = []
    for key, label, density, _ in SCENARIOS:
        out = {
            "scenario_key": key,
            "scenario_label": label,
            "density_count": density,
            "selected_routes": " ".join(map(str, SELECTED_ROUTES)),
            "excluded_routes": "; ".join(f"{k}: {v}" for k, v in sorted(EXCLUDED_ROUTES.items())),
        }
        vals_by_mode = {}
        for mode in MODES:
            vals = [
                r["travel_time_float"] for r in selected
                if r["scenario_key"] == key and r["mode"] == mode
            ]
            vals_by_mode[mode] = vals
            out[f"n_{mode}"] = len(vals)
            out[f"mean_{mode}"] = f"{mean(vals):.3f}" if vals else ""
            out[f"median_{mode}"] = f"{float(np.median(vals)):.3f}" if vals else ""
            out[f"q1_{mode}"] = f"{float(np.percentile(vals, 25)):.3f}" if vals else ""
            out[f"q3_{mode}"] = f"{float(np.percentile(vals, 75)):.3f}" if vals else ""
        if vals_by_mode.get("B0"):
            b0 = mean(vals_by_mode["B0"])
            for mode in ["B1", "F2", "F2P"]:
                if vals_by_mode.get(mode):
                    m = mean(vals_by_mode[mode])
                    out[f"mean_improvement_{mode}_vs_B0_pct"] = f"{(b0 - m) / b0 * 100:.2f}"
        if vals_by_mode.get("F2") and vals_by_mode.get("F2P"):
            f2 = mean(vals_by_mode["F2"])
            f2p = mean(vals_by_mode["F2P"])
            out["mean_improvement_F2P_vs_F2_pct"] = f"{(f2 - f2p) / f2 * 100:.2f}"
        summary.append(out)
    return selected, summary, route_table


def consolidated_domain_metrics(rows: list[dict]) -> list[dict]:
    selected = [
        r for r in rows
        if r["route_id_int"] in SELECTED_ROUTES
        and r["mode"] in MODES
    ]
    out_rows = []
    for key, label, density, _ in SCENARIOS:
        for route in SELECTED_ROUTES:
            out = {
                "scenario_key": key,
                "scenario_label": label,
                "density_count": density,
                "route_id": route,
            }
            for mode in MODES:
                matches = [
                    r for r in selected
                    if r["scenario_key"] == key and r["route_id_int"] == route and r["mode"] == mode
                ]
                if not matches:
                    continue
                r = matches[0]
                prefix = MODE_DISPLAY[mode]
                out[f"{prefix}_arrived"] = r.get("arrived", "")
                out[f"{prefix}_travel_time_s"] = fmt_float(r["travel_time_float"])
                out[f"{prefix}_waiting_time_s"] = fmt_float(r["waiting_time_float"])
                out[f"{prefix}_time_loss_s"] = fmt_float(r["time_loss_float"])
                out[f"{prefix}_stops_waiting_count_n"] = fmt_float(r["waiting_count_float"], decimals=0)
                out[f"{prefix}_stop_time_s"] = fmt_float(r["stop_time_float"])
                out[f"{prefix}_route_length_m"] = fmt_float(r["route_length_float"])
                out[f"{prefix}_wall_elapsed_s"] = fmt_float(r["wall_elapsed_float"])
                out[f"{prefix}_sim_stop_reason"] = r.get("sim_stop_reason", "")
                out[f"{prefix}_ev_nonarrival_censored"] = r.get("ev_nonarrival_censored", "")
            out_rows.append(out)
    return out_rows


def fmt_float(value: float, decimals: int = 1) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.{decimals}f}"


def find_representative_files() -> dict[str, Path]:
    scenario_folder = None
    for key, _, _, folder in SCENARIOS:
        if key == REPRESENTATIVE["scenario_key"]:
            scenario_folder = folder
            break
    if scenario_folder is None:
        raise FileNotFoundError(f"Representative scenario not configured: {REPRESENTATIVE['scenario_key']}")

    base = Path(scenario_folder)
    route = int(REPRESENTATIVE["route_id"])
    mode = str(REPRESENTATIVE["mode"])
    scenario_key = str(REPRESENTATIVE["scenario_key"])
    event_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/fed_outcomes.events.jsonl"))
    decision_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/intersection_decisions.csv"))
    run_context_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/run_context.json"))
    policy_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/policy_args_effective.json"))
    raw_matches = sorted(base.glob(f"fnm_data*/**/*r{route}_{mode}/*/raw_messages.jsonl"))
    core_log_matches = sorted(base.glob("federation_core_logs*/membership.jsonl"))
    if not event_matches or not decision_matches or not raw_matches:
        raise FileNotFoundError(
            f"Representative traces are incomplete under {base} "
            f"for scenario={scenario_key}, route={route}, mode={mode}. "
            "Use --skip-middleware if you only need scenario/domain plots."
        )
    return {
        "event_jsonl": event_matches[-1],
        "decisions_csv": decision_matches[-1],
        "run_context_json": run_context_matches[-1] if run_context_matches else Path(""),
        "policy_args_json": policy_matches[-1] if policy_matches else Path(""),
        "raw_glob_root": raw_matches[0].parent.parent,
        "core_log_root": core_log_matches[-1].parent if core_log_matches else Path(""),
    }


def find_representative_files_for_mode(mode: str) -> dict[str, Path]:
    scenario_folder = None
    for key, _, _, folder in SCENARIOS:
        if key == REPRESENTATIVE["scenario_key"]:
            scenario_folder = folder
            break
    if scenario_folder is None:
        raise FileNotFoundError(f"Representative scenario not configured: {REPRESENTATIVE['scenario_key']}")
    base = Path(scenario_folder)
    route = int(REPRESENTATIVE["route_id"])
    scenario_key = str(REPRESENTATIVE["scenario_key"])
    event_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/fed_outcomes.events.jsonl"))
    decision_matches = sorted(base.glob(f"ev_matrix_results_*/scenario_runs/scenario_{scenario_key}*_r{route}/**/route_{route}/{mode}/intersection_decisions.csv"))
    raw_matches = sorted(base.glob(f"fnm_data*/**/*r{route}_{mode}/*/raw_messages.jsonl"))
    if not event_matches or not raw_matches:
        raise FileNotFoundError(f"Missing representative event/raw traces for mode={mode}, route={route}, scenario={scenario_key}")
    return {
        "event_jsonl": event_matches[-1],
        "decisions_csv": decision_matches[-1] if decision_matches else Path(""),
        "raw_glob_root": raw_matches[0].parent.parent,
    }


def load_representative_mode_data() -> dict[str, dict]:
    data: dict[str, dict] = {}
    for mode in MODES:
        try:
            paths = find_representative_files_for_mode(mode)
        except FileNotFoundError:
            continue
        events = load_events(paths["event_jsonl"])
        decisions_path = paths["decisions_csv"]
        decisions = load_decisions(decisions_path) if decisions_path.is_file() else []
        messages = load_raw_messages(paths["raw_glob_root"])
        data[mode] = {
            "paths": paths,
            "events": events,
            "decisions": decisions,
            "messages": messages,
        }
    return data


def bucket_key(event_type: str) -> str:
    if event_type == "passive_intersection.context_pub":
        return "Passive context"
    if event_type.startswith("ev.intersection.discovery"):
        return "EV discovery"
    if event_type.startswith("ev.request"):
        return "EV request"
    if event_type.startswith("f2p."):
        return "F2P guards/rescue"
    if event_type in {"f2.apply", "f2.strict_b1_floor.apply", "f2.b1_continuity.apply"}:
        return "Coordination apply"
    if "skip" in event_type or "guard" in event_type:
        return "Policy guard/skip"
    if event_type.startswith("ev.service_window"):
        return "Service window"
    return "Other"


def load_events(path: Path) -> list[dict]:
    events = []
    with open(path) as f:
        for line in f:
            if line.strip():
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return events


def load_decisions(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_raw_messages(raw_root: Path) -> list[dict]:
    messages = []
    for path in sorted(raw_root.glob("*/raw_messages.jsonl")):
        gateway = path.parent.name
        with open(path) as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    o = json.loads(line)
                except json.JSONDecodeError:
                    continue
                payload = o.get("payload")
                topic = o.get("topic", "")
                payload_bytes = len(json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")) if payload is not None else 0
                o["_gateway"] = gateway
                o["_topic_group"] = topic_group(topic)
                o["_payload_bytes"] = payload_bytes
                messages.append(o)
    return messages


def topic_group(topic: str) -> str:
    if "membership" in topic:
        return "Membership"
    if "catalog" in topic:
        return "Catalogue"
    if "discovery" in topic:
        return "Discovery"
    if "state/intersection" in topic or "/tls/" in topic:
        return "Intersection state"
    if "request" in topic:
        return "Requests"
    if "decision" in topic:
        return "Decisions"
    return "Other"


def binned_counts(items: list[tuple[float, str]], bin_sec: float) -> tuple[list[float], dict[str, list[int]]]:
    if not items:
        return [], {}
    tmin = min(t for t, _ in items)
    tmax = max(t for t, _ in items)
    n = int(math.ceil((tmax - tmin) / bin_sec)) + 1
    labels = sorted({label for _, label in items})
    counts = {label: [0] * n for label in labels}
    centers = [tmin + (i + 0.5) * bin_sec for i in range(n)]
    for t, label in items:
        idx = min(n - 1, max(0, int((t - tmin) // bin_sec)))
        counts[label][idx] += 1
    return centers, counts


def binned_counts_fixed(
    items: list[tuple[float, str]],
    bin_sec: float,
    window: tuple[float, float],
    labels: list[str] | None = None,
) -> tuple[list[float], dict[str, list[int]]]:
    xmin, xmax = window
    n = int(math.ceil((xmax - xmin) / bin_sec)) + 1
    labels = labels or sorted({label for _, label in items})
    counts = {label: [0] * n for label in labels}
    centers = [xmin + (i + 0.5) * bin_sec for i in range(n)]
    for t, label in items:
        if label not in counts or not (xmin <= t <= xmax):
            continue
        idx = min(n - 1, max(0, int((t - xmin) // bin_sec)))
        counts[label][idx] += 1
    return centers, counts


def representative_time_window(events: list[dict], pad_before_s: float = 10.0) -> tuple[float, float]:
    """Return the EV-appearance zoom window for route-level middleware plots.

    Passive DT context may exist from t=0, but the paper zoom should focus on
    the EV mission window. We infer EV appearance from operational EV/request/
    coordination events and extend to the last event so arrival/completion is
    retained.
    """
    all_times: list[float] = []
    active_times: list[float] = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        if not math.isfinite(sim):
            continue
        all_times.append(sim)
        et = str(e.get("event_type", "") or "")
        if et.startswith("passive_intersection.") or et.endswith(".config") or et == "ev.state.trace":
            continue
        if (
            et.startswith("ev.focus")
            or et.startswith("ev.request")
            or et.startswith("ev.intersection.discovery")
            or et.startswith("f2")
            or et.startswith("service_window")
        ):
            active_times.append(sim)
    if not all_times:
        return 0.0, 1.0
    start = min(active_times) - float(pad_before_s) if active_times else min(all_times)
    return max(0.0, start), max(all_times)


def in_window(t: float, window: tuple[float, float]) -> bool:
    return math.isfinite(t) and window[0] <= t <= window[1]


EVENT_CATEGORY_ORDER = [
    "EV discovery",
    "EV request",
    "Apply",
    "Passive-supported guard/rescue",
    "Guard/skip",
    "Passive context",
]

# Compact layers for paper figures. Raw passive context publications are still
# exported in CSVs, but they are dense and redundant in visual summaries once
# passive-supported guards/rescue events show that the context was consumed.
EVENT_CATEGORY_PAPER_ORDER = [
    "EV discovery",
    "EV request",
    "Apply",
    "Passive-supported guard/rescue",
    "Guard/skip",
]

EVENT_CATEGORY_CHOREOGRAPHY_ORDER = [
    "EV discovery",
    "EV request",
    "Apply",
    "Passive-supported guard/rescue",
    "Guard/skip",
    "Passive context",
]

EVENT_CATEGORY_COLORS = {
    "EV discovery": "#9467BD",
    "EV request": "#E07A32",
    "Apply": "#2D7DD2",
    "Passive-supported guard/rescue": "#1B9E77",
    "Guard/skip": "#C44E52",
    "Passive context": "#7A9E7E",
    "Other": "#6C757D",
}

EVENT_CATEGORY_LABELS = {
    "EV discovery": "EV-DT discovery (SI-DTs)|Discovery service",
    "EV request": "EV-DT to SI-DT request|FNM interoperability",
    "Apply": "SI-DT active decision|Local DT + FNM",
    "Passive-supported guard/rescue": "Passive SI-DT support|F2P policy",
    "Guard/skip": "Active SI-DT safety guard|Policy + passive context",
    "Passive context": "Local passive SI-DT context|FNM pub/sub",
}

EVENT_CATEGORY_SHORT_LABELS = {
    "EV discovery": "Discovery",
    "EV request": "Priority request",
    "Apply": "Local decision",
    "Passive-supported guard/rescue": "Passive support",
    "Guard/skip": "Safety guard",
    "Passive context": "Passive context",
}

EVENT_CATEGORY_ENABLEMENT_LABELS = {
    "EV discovery": "Discovery service",
    "EV request": "FNM request routing",
    "Apply": "Local SI-DT decision",
    "Passive-supported guard/rescue": "Passive SI-DT support",
    "Guard/skip": "FCDP safety guard",
    "Passive context": "Passive SI-DT context",
}

TOPIC_FAMILY_LABELS = {
    "Membership": "Membership\nService",
    "Catalogue": "Catalogue\nService",
    "Discovery": "Discovery\nService",
    "Intersection state": "FNM-routed\nSI-DT artefacts",
    "Requests": "FNM DT-request\nrouting",
    "Decisions": "FNM decision\nrouting",
    "Other": "Other\nfederation",
}


def svg_multiline_text(svg: Svg, x: float, y: float, text: str, cls: str = "legend", line_h: float = 15.0, anchor: str = "start") -> None:
    for i, part in enumerate(str(text).split("|")):
        svg.text(x, y + i * line_h, part, cls=cls, anchor=anchor)


def draw_category_legend(
    svg: Svg,
    categories: list[str],
    x: float,
    y: float,
    *,
    label_map: dict[str, str] | None = None,
    columns: int = 5,
    col_w: float = 190.0,
    row_h: float = 24.0,
    marker: str = "rect",
    center_rows: bool = False,
    font_size: int | None = None,
) -> None:
    labels = label_map or EVENT_CATEGORY_SHORT_LABELS
    for i, cat in enumerate(categories):
        row = i // columns
        col = i % columns
        items_in_row = min(columns, len(categories) - row * columns)
        row_offset = (columns - items_in_row) * col_w / 2.0 if center_rows else 0.0
        lx = x + row_offset + col * col_w
        ly = y + row * row_h
        if marker == "circle":
            svg.circle(lx + 7, ly - 5, r=5, fill=EVENT_CATEGORY_COLORS[cat], opacity=0.82)
        else:
            svg.rect(lx, ly - 13, 18, 12, fill=EVENT_CATEGORY_COLORS[cat], opacity=0.78)
        svg.text(lx + 26, ly - 3, labels.get(cat, cat), cls="legend", size=font_size)


def draw_centered_category_legend(
    svg: Svg,
    categories: list[str],
    y: float,
    *,
    label_map: dict[str, str] | None = None,
    columns: int = 3,
    col_w: float = 210.0,
    row_h: float = 24.0,
    marker: str = "rect",
) -> None:
    visible_cols = min(columns, max(1, len(categories)))
    total_w = visible_cols * col_w
    x = max(24.0, (svg.width - total_w) / 2.0)
    draw_category_legend(svg, categories, x, y, label_map=label_map, columns=columns, col_w=col_w, row_h=row_h, marker=marker)


def draw_positioned_category_legend(
    svg: Svg,
    categories: list[str],
    y: float,
    *,
    label_map: dict[str, str] | None = None,
    columns: int = 3,
    col_w: float = 210.0,
    row_h: float = 24.0,
    marker: str = "rect",
    position: str | None = None,
    center_rows: bool = False,
) -> None:
    position = position or paper_legend_position("top-center")
    visible_cols = min(columns, max(1, len(categories)))
    total_w = visible_cols * col_w
    if position == "top-left":
        x = 24.0
    elif position == "top-right":
        x = max(24.0, svg.width - total_w - 24.0)
    else:
        x = max(24.0, (svg.width - total_w) / 2.0)
    draw_category_legend(
        svg,
        categories,
        x,
        y,
        label_map=label_map,
        columns=columns,
        col_w=col_w,
        row_h=row_h,
        marker=marker,
        center_rows=center_rows,
    )


def draw_centered_mode_legend(svg: Svg, y: float, *, item_w: float = 125.0) -> None:
    total_w = item_w * len(MODES)
    x0 = max(24.0, (svg.width - total_w) / 2.0)
    for i, mode in enumerate(MODES):
        x = x0 + i * item_w
        svg.rect(x, y - 14, 18, 13, fill=MODE_COLORS[mode], opacity=0.45, stroke=MODE_COLORS[mode])
        svg.text(x + 25, y - 3, MODE_DISPLAY[mode], cls="legend")


def draw_positioned_mode_legend(svg: Svg, y: float, *, item_w: float = 125.0, position: str | None = None) -> None:
    position = position or paper_legend_position("top-center")
    total_w = item_w * len(MODES)
    if position == "top-left":
        x0 = 24.0
    elif position == "top-right":
        x0 = max(24.0, svg.width - total_w - 24.0)
    else:
        x0 = max(24.0, (svg.width - total_w) / 2.0)
    for i, mode in enumerate(MODES):
        x = x0 + i * item_w
        svg.rect(x, y - 14, 18, 13, fill=MODE_COLORS[mode], opacity=1.0, stroke=MODE_COLORS[mode])
        svg.text(x + 25, y - 3, MODE_DISPLAY[mode], cls="legend")


def with_paper_slot(slot: str, fn, *args, **kwargs):
    global CURRENT_PAPER_SLOT
    prev = CURRENT_PAPER_SLOT
    CURRENT_PAPER_SLOT = slot
    try:
        return fn(*args, **kwargs)
    finally:
        CURRENT_PAPER_SLOT = prev


def compact_runtime_window(events: list[dict]) -> tuple[float, float]:
    # Keep compact route/middleware timelines visually aligned for the paper:
    # Route 5 / 1.25K activity starts near this point, so both compact panels
    # use the same left boundary and end at the last relevant activity.
    fixed_start = 830.0
    base = representative_time_window(events)
    pre = (max(0.0, base[0] - 120.0), base[1] + 120.0)
    sims = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if cat in EVENT_CATEGORY_CHOREOGRAPHY_ORDER and in_window(sim, pre):
            sims.append(sim)
    if sims:
        after_start = [v for v in sims if math.isfinite(v) and v >= fixed_start]
        if after_start:
            start = fixed_start
            end = math.ceil(max(after_start))
        else:
            start = max(0.0, math.floor(min(sims)))
            end = math.ceil(max(sims))
    else:
        start = fixed_start
        end = math.ceil(base[1])
    if end <= start:
        end = start + 60.0
    return start, end


def plot_runtime_timeline(events: list[dict], path: Path) -> None:
    window = representative_time_window(events)
    items = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        if not in_window(sim, window):
            continue
        b = bucket_key(e.get("event_type", ""))
        if b != "Other":
            items.append((sim, b))
    centers, counts = binned_counts(items, 10.0)
    svg = Svg(1280, 720, f"Runtime coordination timeline: {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}")
    svg.text(36, 56, "Event count per 10 simulated seconds. Representative route where federation services expose runtime coordination choreography.", cls="subtitle")
    top, bottom, left, right = 92, 585, 82, 1218
    ymax = max((max(v) for v in counts.values()), default=1)
    draw_y_axis(svg, left, top, bottom, 0, max(10, ymax * 1.12), ticks=6, label="Events per 10 s Bin (count)")
    if centers:
        xmin, xmax = window
        def sx(t):
            return left + (t - xmin) / (xmax - xmin or 1) * (right - left)
        for i in range(6):
            t = xmin + (xmax - xmin) * i / 5
            x = sx(t)
            svg.line(x, bottom, x, bottom + 5, stroke="#70808f")
            svg.text(x, bottom + 22, f"{t:.0f}", cls="tick", anchor="middle")
        colors = {
            "Passive context": "#1B9E77",
            "EV discovery": "#9467BD",
            "EV request": "#E07A32",
            "Coordination apply": "#2D7DD2",
            "F2P guards/rescue": "#099268",
            "Policy guard/skip": "#C44E52",
            "Service window": "#8C6D31",
        }
        for label, arr in counts.items():
            pts = [(sx(t), y_scale(v, 0, max(10, ymax * 1.12), top, bottom)) for t, v in zip(centers, arr)]
            svg.polyline(pts, stroke=colors.get(label, "#555"), sw=2.5, opacity=0.9)
        lx, ly = left + 20, 650
        for i, label in enumerate(counts):
            x = lx + (i % 4) * 270
            y = ly + (i // 4) * 22
            svg.line(x, y, x + 24, y, stroke=colors.get(label, "#555"), sw=3)
            svg.text(x + 32, y + 4, label, cls="legend")
    svg.text((left + right) / 2, 624, "Simulation Time (s)", cls="label", anchor="middle")
    svg.save(path)


def plot_event_composition(events: list[dict], path: Path) -> None:
    counts = Counter(bucket_key(e.get("event_type", "")) for e in events)
    counts.pop("Other", None)
    labels = [label for label, _ in counts.most_common()]
    values = [counts[label] for label in labels]
    svg = Svg(980, 640, "Middleware and coordination event composition")
    svg.text(36, 56, f"Representative {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']} trace; counts from fed_outcomes.events.jsonl.", cls="subtitle")
    top, bottom, left, right = 90, 540, 250, 920
    vmax = max(values) if values else 1
    for i, (label, value) in enumerate(zip(labels, values)):
        y = top + i * 58
        w = (right - left) * value / vmax
        color = "#1B9E77" if "Passive" in label else "#2D7DD2" if "Coordination" in label else "#E07A32" if "request" in label else "#6C757D"
        svg.text(left - 12, y + 24, label, cls="label", anchor="end")
        svg.rect(left, y + 6, w, 26, fill=color, opacity=0.75, rx=4)
        svg.text(left + w + 8, y + 25, f"{value:,}", cls="tick")
    svg.save(path)


def plot_message_overhead(messages: list[dict], path: Path) -> None:
    group_counts = Counter(m["_topic_group"] for m in messages)
    group_bytes = Counter()
    for m in messages:
        group_bytes[m["_topic_group"]] += m["_payload_bytes"]
    labels = [label for label, _ in group_counts.most_common()]
    svg = Svg(1280, 720, "Federation communication overhead")
    svg.text(36, 56, f"Route-scoped raw FNM messages for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}. Top: message count. Bottom: payload volume.", cls="subtitle")
    panels = [
        (95, 342, group_counts, "Messages", "#2D7DD2", lambda v: f"{int(v):,}"),
        (430, 677, group_bytes, "Payload bytes", "#1B9E77", human_bytes),
    ]
    left, right = 92, 1210
    group_w = (right - left) / max(1, len(labels))
    bar_w = min(72, group_w * 0.58)
    for top, bottom, data, title, color, fmt in panels:
        vmax = max(data.values()) if data else 1
        svg.text(left, top - 18, title, cls="label")
        draw_y_axis(svg, left, top, bottom, 0, vmax * 1.12, ticks=4, label="")
        for i, label in enumerate(labels):
            x = left + group_w * (i + 0.5) - bar_w / 2
            value = data[label]
            h = (bottom - top) * value / max(1, vmax * 1.12)
            svg.rect(x, bottom - h, bar_w, h, fill=color, opacity=0.76, rx=3)
            svg.text(x + bar_w / 2, bottom - h - 6, fmt(value), cls="small", anchor="middle")
            svg.text(x + bar_w / 2, bottom + 24, label, cls="tick", anchor="middle", rotate=30)
    total_bytes = sum(group_bytes.values())
    svg.text(92, 704, f"Route-scoped total: {len(messages):,} messages; {human_bytes(total_bytes)} payload; mean payload {total_bytes / max(1, len(messages)):.0f} B.", cls="subtitle")
    svg.save(path)


def plot_service_payload_latency_bars(messages: list[dict], path: Path) -> None:
    by_group: dict[str, dict[str, list[float] | float]] = defaultdict(lambda: {"sizes": [], "latencies": []})
    for m in messages:
        group = str(m.get("_topic_group", "Other"))
        by_group[group]["sizes"].append(float(m.get("_payload_bytes", 0)))  # type: ignore[index]
        payload = m.get("payload")
        if isinstance(payload, dict) and "latency_ms" in payload:
            v = safe_float(payload.get("latency_ms"))
            if math.isfinite(v):
                by_group[group]["latencies"].append(v)  # type: ignore[index]
    labels = sorted(by_group, key=lambda g: sum(by_group[g]["sizes"]), reverse=True)  # type: ignore[arg-type]
    svg = Svg(1280, 780, "Service payload and latency by topic family")
    svg.text(36, 56, f"Route-scoped service/topic families for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}. Latency uses payload latency_ms when present.", cls="subtitle")
    left, right = 92, 1210
    group_w = (right - left) / max(1, len(labels))
    bar_w = min(68, group_w * 0.52)
    panels = [
        (105, 330, "Payload Volume (B)", "#1B9E77"),
        (430, 655, "Latency p95 (ms)", "#E07A32"),
    ]
    payload_values = [sum(by_group[g]["sizes"]) for g in labels]  # type: ignore[arg-type]
    latency_values = [
        float(np.percentile(by_group[g]["latencies"], 95)) if by_group[g]["latencies"] else 0.0  # type: ignore[arg-type]
        for g in labels
    ]
    for (top, bottom, title, color), values in zip(panels, [payload_values, latency_values]):
        vmax = max(values) if values else 1.0
        svg.text(left, top - 18, title, cls="label")
        draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=4, label="")
        for i, (label, value) in enumerate(zip(labels, values)):
            x = left + group_w * (i + 0.5) - bar_w / 2
            h = (bottom - top) * value / max(1.0, vmax * 1.15)
            svg.rect(x, bottom - h, bar_w, h, fill=color, opacity=0.76, rx=3)
            out = human_bytes(value) if "Payload" in title else f"{value:.2f}"
            if value > 0:
                svg.text(x + bar_w / 2, bottom - h - 6, out, cls="small", anchor="middle")
            display = TOPIC_FAMILY_LABELS.get(label, label).replace("\n", "|")
            svg_multiline_text(svg, x + bar_w / 2, bottom + 22, display, cls="tick", line_h=14, anchor="middle")
        svg.text((left + right) / 2, bottom + 70, "Service / topic family", cls="label", anchor="middle")
    svg.save(path)


def human_bytes(n: float) -> str:
    units = ["B", "KB", "MB", "GB"]
    v = float(n)
    for u in units:
        if v < 1024 or u == units[-1]:
            return f"{v:.1f} {u}" if u != "B" else f"{v:.0f} B"
        v /= 1024
    return f"{v:.1f} GB"


def plot_payload_latency(messages: list[dict], path: Path) -> None:
    sizes = sorted(m["_payload_bytes"] for m in messages if m["_payload_bytes"] > 0)
    latencies = []
    for m in messages:
        payload = m.get("payload")
        if isinstance(payload, dict) and "latency_ms" in payload:
            v = safe_float(payload.get("latency_ms"))
            if math.isfinite(v):
                latencies.append(v)
    svg = Svg(1180, 650, "Packet size and service latency distributions")
    svg.text(36, 56, "Payload sizes cover route-scoped FNM raw messages; latency_ms fields mostly come from membership/discovery service events.", cls="subtitle")
    draw_hist(svg, sizes, 80, 530, 90, 520, "Payload Size (B)", "#2D7DD2")
    draw_hist(svg, latencies, 670, 1120, 90, 520, "Service Latency (ms)", "#1B9E77")
    if sizes:
        svg.text(80, 592, f"Payload p50={np.percentile(sizes,50):.0f} B, p95={np.percentile(sizes,95):.0f} B, max={max(sizes):.0f} B", cls="subtitle")
    if latencies:
        svg.text(670, 592, f"Latency p50={np.percentile(latencies,50):.2f} ms, p95={np.percentile(latencies,95):.2f} ms, max={max(latencies):.2f} ms", cls="subtitle")
    svg.save(path)


def draw_hist(svg: Svg, values: list[float], left, right, top, bottom, title, color):
    svg.text((left + right) / 2, top - 18, title, cls="label", anchor="middle")
    if not values:
        svg.text((left + right) / 2, (top + bottom) / 2, "No data", cls="subtitle", anchor="middle")
        return
    vmax_data = float(np.percentile(values, 99)) if len(values) > 10 else max(values)
    vmax_data = max(vmax_data, 1.0)
    bins = np.linspace(0, vmax_data, 18)
    counts, edges = np.histogram(np.clip(values, 0, vmax_data), bins=bins)
    ymax = max(counts) if len(counts) else 1
    for i, c in enumerate(counts):
        x0 = left + (edges[i] / vmax_data) * (right - left)
        x1 = left + (edges[i + 1] / vmax_data) * (right - left)
        h = (bottom - top) * c / ymax
        svg.rect(x0 + 1, bottom - h, max(1, x1 - x0 - 2), h, fill=color, opacity=0.7)
    for i in range(5):
        value = vmax_data * i / 4
        x = left + (right - left) * i / 4
        svg.line(x, bottom, x, bottom + 5, stroke="#70808f")
        svg.text(x, bottom + 22, f"{value:.0f}", cls="tick", anchor="middle")
    svg.line(left, bottom, right, bottom, stroke="#70808f")
    svg.line(left, top, left, bottom, stroke="#70808f")


def plot_decision_episodes(decisions: list[dict], events: list[dict], path: Path) -> None:
    by_tls = Counter()
    timeline = []
    node_order = infer_route_node_order(events)
    window = representative_time_window(events)
    for d in decisions:
        tls = d.get("tls_id") or d.get("intersection_id") or "unknown"
        by_tls[tls] += 1
        t = safe_float(d.get("sim_time"))
        if in_window(t, window):
            timeline.append((t, tls))
    svg = Svg(1280, 640, "Runtime coordination episodes")
    svg.text(36, 56, f"Intersection decision applications from intersection_decisions.csv for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}.", cls="subtitle")
    top, bottom, left, right = 92, 505, 122, 1210
    route_tls = [node for node in node_order if node in by_tls]
    tls_order = route_tls + [tls for tls, _ in by_tls.most_common() if tls not in route_tls]
    if timeline:
        xmin, xmax = window
        for i, tls in enumerate(tls_order):
            y_step = (bottom - top) / max(1, len(tls_order) - 1)
            y = top + i * y_step
            svg.line(left, y, right, y, stroke="#e6edf2", sw=1)
            svg.text(left - 8, y + 4, f"{i + 1}. {tls}", cls="small", anchor="end")
        for t, tls in timeline:
            idx = tls_order.index(tls)
            y_step = (bottom - top) / max(1, len(tls_order) - 1)
            x = left + (t - xmin) / (xmax - xmin or 1) * (right - left)
            y = top + idx * y_step
            svg.circle(x, y, r=2, fill="#2D7DD2", opacity=0.45)
        for i in range(6):
            t = xmin + (xmax - xmin) * i / 5
            x = left + (right - left) * i / 5
            svg.line(x, bottom + 10, x, bottom + 15, stroke="#70808f")
            svg.text(x, bottom + 32, f"{t:.0f}", cls="tick", anchor="middle")
        svg.text((left + right) / 2, bottom + 56, "Simulation Time (s)", cls="label", anchor="middle")
    svg.save(path)



def event_node_id(event: dict) -> str:
    for key in ("tls_id", "node_id", "route_current_node", "approach_node", "requester_tls"):
        value = str(event.get(key, "") or "").strip()
        if value:
            return value
    return "unknown"


def event_progress_category(event_type: str) -> str:
    if event_type == "passive_intersection.context_pub":
        return "Passive context"
    if event_type.startswith("ev.intersection.discovery"):
        return "EV discovery"
    if event_type.startswith("ev.request"):
        return "EV request"
    if event_type in {"f2.apply", "f2.strict_b1_floor.apply", "f2.b1_continuity.apply"}:
        return "Apply"
    if event_type.startswith("f2p.passive_stall_rescue") or event_type.startswith("f2p.passive_nearfield_guard"):
        return "Passive-supported guard/rescue"
    if "skip" in event_type or "guard" in event_type:
        return "Guard/skip"
    return "Other"


def edge_node_pair(edge_id: str) -> tuple[str, str] | None:
    text = str(edge_id or "").strip()
    if not text:
        return None
    text = text.split("#", 1)[0]
    if text.startswith(":"):
        return None
    if text.startswith("Edge"):
        text = text[4:]
    if "-" not in text:
        return None
    a, b = text.split("-", 1)
    if not a.startswith("Node"):
        a = f"Node{a}" if a.isdigit() else a
    if not b.startswith("Node"):
        b = f"Node{b}" if b.isdigit() else b
    if a and b:
        return a, b
    return None


def insert_passive_nodes_by_edges(ordered: list[str], passive_edges: dict[str, list[tuple[str, str]]]) -> list[str]:
    out = list(ordered)
    for passive_node, pairs in passive_edges.items():
        if passive_node in out:
            continue
        best_insert: int | None = None
        for a, b in pairs:
            if passive_node not in (a, b):
                continue
            other = b if a == passive_node else a
            if other not in out:
                continue
            other_idx = out.index(other)
            # Place the passive observer next to its closest route neighbor.
            candidate = other_idx + (1 if a == other and b == passive_node else 0)
            if best_insert is None or candidate < best_insert:
                best_insert = candidate
        if best_insert is None:
            out.append(passive_node)
        else:
            out.insert(max(0, min(best_insert, len(out))), passive_node)
    return out


def infer_route_node_order(events: list[dict]) -> list[str]:
    route_nodes: list[str] = []
    passive_first: dict[str, float] = {}
    active_first: dict[str, float] = {}
    passive_edges: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        for key in ("route_nodes", "route_peer_tls", "active_ev_route_intersections"):
            vals = e.get(key)
            if isinstance(vals, list):
                for v in vals:
                    node = str(v or "").strip()
                    if node and node not in route_nodes:
                        route_nodes.append(node)
        node = event_node_id(e)
        if node and node != "unknown" and math.isfinite(sim):
            if str(e.get("event_type", "")) == "passive_intersection.context_pub":
                passive_first.setdefault(node, sim)
                for edge in e.get("target_edges", []) or []:
                    pair = edge_node_pair(str(edge))
                    if pair and pair not in passive_edges[node]:
                        passive_edges[node].append(pair)
            else:
                active_first.setdefault(node, sim)
    ordered = list(route_nodes)
    # Insert passive/non-TLS observer nodes near the route edges they observe
    # whenever possible; otherwise fall back to first observation time. This
    # keeps the visual route progression closer to the SUMO route order.
    ordered = insert_passive_nodes_by_edges(ordered, passive_edges)
    for node, _ in sorted(passive_first.items(), key=lambda kv: kv[1]):
        if node not in ordered:
            ordered.append(node)
    for node, _ in sorted(active_first.items(), key=lambda kv: kv[1]):
        if node not in ordered:
            ordered.append(node)
    return ordered


def paired_runtime_panel_height(events: list[dict], base_height: int) -> int:
    """Keep the paired Route 5 runtime panels aligned and node rows readable."""
    if CURRENT_PAPER_SLOT != "2_1":
        return base_height
    node_count = max(1, len(infer_route_node_order(events)))
    # Grid uses top=128 and bottom=height-82. This leaves about 26 px between
    # adjacent node rows while preserving identical grid bounds in both plots.
    return max(base_height, 184 + 26 * node_count)


def plot_route_progression(events: list[dict], path: Path, *, compact: bool = False) -> None:
    node_order = infer_route_node_order(events)
    if not node_order:
        return
    window = compact_runtime_window(events) if compact else representative_time_window(events)
    # Keep the route progression plot readable: show the operational exchange
    # events and sampled passive context, but do not turn every low-level guard
    # diagnostic into its own visual layer.
    categories = EVENT_CATEGORY_CHOREOGRAPHY_ORDER
    colors = EVENT_CATEGORY_COLORS
    points = []
    for e in events:
        et = str(e.get("event_type", "") or "")
        cat = event_progress_category(et)
        if cat == "Other" or cat not in categories:
            continue
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        node = event_node_id(e)
        if not in_window(sim, window) or node not in node_order:
            continue
        points.append((sim, node, cat, et))
    if not points:
        return
    tmin, tmax = window
    if compact:
        width, base_height = paper_dim(900, 440)
        height = paired_runtime_panel_height(events, base_height)
        left, right, top, bottom = PAPER_2_1_LEFT_MARGIN, width - 42, 128, height - 82
        legend_y = 66
        legend_cols = 3
        legend_col_w = PAPER_2_1_LEGEND_COLUMN_WIDTH
        marker_base = 3.4
        passive_marker = 2.0
        route_tick_size = max(12, int(active_paper_layout().get("xtick_size", 20)))
    else:
        width = 1380
        height = max(720, 170 + 34 * len(node_order))
        left, right, top, bottom = 150, width - 60, 128, height - 88
        legend_y = 92
        legend_cols = 5
        legend_col_w = 205
        marker_base = 4.0
        passive_marker = 2.3
        route_tick_size = None
    svg = Svg(width, height, "" if compact else f"Route progression zoom: {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}")
    svg.text(36, 56, "Node-level timeline of local DT operations during the EV-triggered runtime window.", cls="subtitle")
    if compact:
        draw_centered_category_legend(svg, categories, legend_y, columns=legend_cols, col_w=legend_col_w, marker="rect")
    else:
        draw_category_legend(svg, categories, left, legend_y, columns=legend_cols, col_w=legend_col_w, marker="circle")
    row_h = (bottom - top) / max(1, len(node_order) - 1)
    for i, node in enumerate(node_order):
        y = top + i * row_h
        svg.line(left, y, right, y, stroke="#e6edf2", sw=1)
        svg.text(left - 10, y + 4, node, cls="tick", size=route_tick_size, anchor="end")
    for i in range(7):
        t = tmin + (tmax - tmin) * i / 6
        x = left + (t - tmin) / (tmax - tmin or 1) * (right - left)
        svg.line(x, top, x, bottom, stroke="#edf2f6", sw=1)
        svg.text(x, bottom + 28, f"{t:.0f}", cls="xtick", size=route_tick_size, anchor="middle")
    svg.text((left + right) / 2, bottom + 60, "Simulation Time (s)", cls="label", size=paper_x_label_size(), anchor="middle")
    # Dense passive context can dominate; sample it for readability while
    # preserving all sparse operational markers.
    passive_seen = 0
    last_drawn: dict[tuple[str, str], float] = {}
    min_gap_by_cat = {
        "EV discovery": 0.0,
        "EV request": 3.0,
        "Apply": 4.0,
        "Passive-supported guard/rescue": 6.0,
        "Guard/skip": 8.0,
        "Passive context": 12.0,
    }
    category_offsets = {
        "EV discovery": -7.0,
        "EV request": -4.0,
        "Apply": -1.0,
        "Passive-supported guard/rescue": 2.0,
        "Guard/skip": 5.0,
        "Passive context": 8.0,
    }
    if compact and CURRENT_PAPER_SLOT:
        category_offsets = {cat: 0.0 for cat in category_offsets}
    for sim, node, cat, _et in points:
        key = (node, cat)
        min_gap = min_gap_by_cat.get(cat, 0.0)
        if key in last_drawn and sim - last_drawn[key] < min_gap:
            continue
        last_drawn[key] = sim
        if cat == "Passive context":
            passive_seen += 1
            if passive_seen % 8 != 0:
                continue
        x = left + (sim - tmin) / (tmax - tmin or 1) * (right - left)
        y = top + node_order.index(node) * row_h + (category_offsets.get(cat, 0.0) if compact else 0.0)
        svg.circle(x, y, r=passive_marker if cat == "Passive context" else marker_base, fill=colors[cat], opacity=0.38 if cat == "Passive context" else 0.78)
    svg.save(path)


def plot_node_activity_summary(events: list[dict], path: Path, *, exclude_highest_node: bool = False) -> None:
    node_order = infer_route_node_order(events)
    if not node_order:
        return
    original_rank = {node: i + 1 for i, node in enumerate(node_order)}
    window = representative_time_window(events)
    categories = EVENT_CATEGORY_CHOREOGRAPHY_ORDER
    colors = EVENT_CATEGORY_COLORS
    counts = {node: Counter() for node in node_order}
    for e in events:
        node = event_node_id(e)
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        if node in counts and cat in categories and in_window(sim, window):
            counts[node][cat] += 1
    totals = {n: sum(counts[n].values()) for n in node_order}
    if exclude_highest_node and totals:
        highest = max(totals, key=totals.get)
        node_order = [n for n in node_order if n != highest]
        totals = {n: totals[n] for n in node_order}
    vmax = max(totals.values()) if totals else 1
    default_width = max(1320, 200 + 66 * len(node_order))
    width, height = paper_dim(default_width, 800)
    svg = Svg(width, height, "" if exclude_highest_node else f"Node-level middleware activity: {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}")
    subtitle = "Vertical stacked counts during the EV appearance window, ordered by route progression."
    if exclude_highest_node:
        subtitle += " Highest-activity entry node excluded for readability."
    svg.text(36, 56, subtitle, cls="subtitle")
    left, right, top, bottom = 104, width - 60, 170, height - 155
    draw_positioned_category_legend(svg, categories, 104, columns=3, col_w=270, position=paper_legend_position("top-center"))
    draw_y_axis(svg, left, top, bottom, 0, vmax * 1.10, ticks=5, label="Federation Service|Activity (count)", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / max(1, len(node_order))
    bar_w = min(42, group_w * 0.68)
    for i, node in enumerate(node_order):
        x0 = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for cat in categories:
            value = counts[node][cat]
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, vmax * 1.10)
            svg.rect(x0, y_cursor - h, bar_w, h, fill=colors[cat], opacity=1.0, rx=2)
            y_cursor -= h
        if totals[node] > 0:
            svg.text(x0 + bar_w / 2, y_cursor - 6, f"{totals[node]:,}", cls="small", size=paper_value_size(17), anchor="middle")
        rank_label = f"I{original_rank.get(node, i + 1)}"
        svg.text(x0 + bar_w / 2, bottom + 28, rank_label, cls="xtick", size=paper_x_label_size(22), anchor="middle")
        svg.text(x0 + bar_w / 2, bottom + 60, node, cls="tick", size=paper_value_size(18), anchor="middle", rotate=paper_xtick_rotation(35))
    svg.save(path)


def write_node_activity_summary_csv(events: list[dict], path: Path) -> None:
    node_order = infer_route_node_order(events)
    window = representative_time_window(events)
    categories = ["EV discovery", "EV request", "Apply", "Passive-supported guard/rescue", "Guard/skip", "Passive context"]
    counts = {node: Counter() for node in node_order}
    first_seen = {node: math.nan for node in node_order}
    last_seen = {node: math.nan for node in node_order}
    for e in events:
        node = event_node_id(e)
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        if node not in counts or cat not in categories or not in_window(sim, window):
            continue
        counts[node][cat] += 1
        if math.isfinite(sim):
            first_seen[node] = sim if not math.isfinite(first_seen[node]) else min(first_seen[node], sim)
            last_seen[node] = sim if not math.isfinite(last_seen[node]) else max(last_seen[node], sim)
    rows = []
    for idx, node in enumerate(node_order, start=1):
        row = {
            "representative_scenario": REPRESENTATIVE["scenario_label"],
            "representative_route": REPRESENTATIVE["route_id"],
            "representative_mode": REPRESENTATIVE["mode"],
            "route_progression_idx": idx,
            "node_id": node,
            "first_event_sim_time_s": f"{first_seen[node]:.3f}" if math.isfinite(first_seen[node]) else "",
            "last_event_sim_time_s": f"{last_seen[node]:.3f}" if math.isfinite(last_seen[node]) else "",
        }
        total = 0
        for cat in categories:
            key = cat.lower().replace("/", "_").replace("-", "_").replace(" ", "_")
            value = int(counts[node][cat])
            row[f"{key}_n"] = value
            total += value
        row["total_middleware_activity_n"] = total
        rows.append(row)
    fieldnames = [
        "representative_scenario",
        "representative_route",
        "representative_mode",
        "route_progression_idx",
        "node_id",
        "first_event_sim_time_s",
        "last_event_sim_time_s",
        "ev_discovery_n",
        "ev_request_n",
        "apply_n",
        "passive_supported_guard_rescue_n",
        "guard_skip_n",
        "passive_context_n",
        "total_middleware_activity_n",
    ]
    write_csv(path, rows, fieldnames)


def request_to_decision_latency_rows(events: list[dict]) -> list[dict]:
    node_order = infer_route_node_order(events)
    node_rank = {node: i + 1 for i, node in enumerate(node_order)}
    window = representative_time_window(events)
    requests: list[dict] = []
    applies: list[dict] = []
    apply_events = {"f2.apply", "f2.strict_b1_floor.apply", "f2.b1_continuity.apply"}
    for e in events:
        et = str(e.get("event_type", "") or "")
        tls = event_node_id(e)
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        wall_ms = safe_float(e.get("ts_wall_ms"))
        if tls == "unknown" or not in_window(sim, window):
            continue
        if et == "ev.request.dispatched":
            requests.append({"tls": tls, "sim": sim, "wall_ms": wall_ms, "event_type": et})
        elif et in apply_events:
            applies.append({"tls": tls, "sim": sim, "wall_ms": wall_ms, "event_type": et})
    applies_by_tls: dict[str, list[dict]] = defaultdict(list)
    for a in applies:
        applies_by_tls[str(a["tls"])].append(a)
    for arr in applies_by_tls.values():
        arr.sort(key=lambda x: float(x["sim"]))

    rows = []
    for req in sorted(requests, key=lambda x: float(x["sim"])):
        tls = str(req["tls"])
        candidate = None
        for app in applies_by_tls.get(tls, []):
            dt = float(app["sim"]) - float(req["sim"])
            if 0.0 <= dt <= 5.0:
                candidate = app
                break
        if candidate is None:
            continue
        sim_latency_ms = (float(candidate["sim"]) - float(req["sim"])) * 1000.0
        wall_latency_ms = math.nan
        if math.isfinite(float(req["wall_ms"])) and math.isfinite(float(candidate["wall_ms"])):
            wall_latency_ms = float(candidate["wall_ms"]) - float(req["wall_ms"])
        rows.append({
            "representative_scenario": REPRESENTATIVE["scenario_label"],
            "representative_route": REPRESENTATIVE["route_id"],
            "representative_mode": REPRESENTATIVE["mode"],
            "route_progression_idx": node_rank.get(tls, 9999),
            "node_id": tls,
            "request_sim_time_s": f"{float(req['sim']):.3f}",
            "decision_sim_time_s": f"{float(candidate['sim']):.3f}",
            "request_to_decision_sim_latency_ms": f"{sim_latency_ms:.3f}",
            "request_to_decision_wall_latency_ms": f"{wall_latency_ms:.3f}" if math.isfinite(wall_latency_ms) else "",
            "decision_event_type": str(candidate["event_type"]),
        })
    rows.sort(key=lambda r: (int(r["route_progression_idx"]), safe_float(r["request_sim_time_s"])))
    return rows


def write_request_to_decision_latency_csv(events: list[dict], path: Path) -> None:
    write_csv(path, request_to_decision_latency_rows(events))


def plot_request_to_decision_latency_boxplot(events: list[dict], path: Path) -> None:
    rows = request_to_decision_latency_rows(events)
    if not rows:
        return
    nodes = []
    for r in rows:
        node = str(r["node_id"])
        if node not in nodes:
            nodes.append(node)
    by_node = {
        node: [
            safe_float(r["request_to_decision_wall_latency_ms"])
            for r in rows
            if r["node_id"] == node and math.isfinite(safe_float(r["request_to_decision_wall_latency_ms"]))
        ]
        for node in nodes
    }
    # Fall back to simulation-step latency if wall timestamps are unavailable.
    if not any(by_node.values()):
        by_node = {
            node: [
                safe_float(r["request_to_decision_sim_latency_ms"])
                for r in rows
                if r["node_id"] == node and math.isfinite(safe_float(r["request_to_decision_sim_latency_ms"]))
            ]
            for node in nodes
        }
        ylabel = "Simulation-Time Request-to-Decision Latency (ms)"
    else:
        ylabel = "Wall-Clock Request-to-Decision Latency (ms)"
    values = [v for arr in by_node.values() for v in arr]
    if not values:
        return
    vmax = max(values)
    svg = Svg(1380, 680, "Request-to-decision latency by route node")
    svg.text(36, 56, f"Pairing EV request dispatch with the next local coordination apply at the same TLS for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}.", cls="subtitle")
    left, right, top, bottom = 90, 1320, 105, 520
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label=ylabel)
    group_w = (right - left) / max(1, len(nodes))
    box_w = min(42, group_w * 0.55)
    rng = np.random.default_rng(33)
    for i, node in enumerate(nodes):
        vals = np.array(by_node[node], dtype=float)
        if len(vals) == 0:
            continue
        x = left + group_w * (i + 0.5)
        q1, med, q3 = np.percentile(vals, [25, 50, 75])
        low, high = float(np.min(vals)), float(np.max(vals))
        yq1, ymed, yq3 = (y_scale(v, 0, max(1.0, vmax * 1.15), top, bottom) for v in (q1, med, q3))
        ylow, yhigh = y_scale(low, 0, max(1.0, vmax * 1.15), top, bottom), y_scale(high, 0, max(1.0, vmax * 1.15), top, bottom)
        color = "#E07A32"
        svg.line(x, ylow, x, yhigh, stroke=color, sw=1.5)
        svg.line(x - box_w * 0.35, ylow, x + box_w * 0.35, ylow, stroke=color, sw=1.5)
        svg.line(x - box_w * 0.35, yhigh, x + box_w * 0.35, yhigh, stroke=color, sw=1.5)
        svg.rect(x - box_w / 2, yq3, box_w, max(1, yq1 - yq3), fill=color, stroke=color, sw=1, opacity=0.30, rx=3)
        svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2.0)
        for v in vals:
            svg.circle(x + float(rng.uniform(-4, 4)), y_scale(v, 0, max(1.0, vmax * 1.15), top, bottom), r=2.1, fill=color, opacity=0.45)
        svg.text(x, bottom + 22, f"I{i + 1}", cls="tick", anchor="middle")
        svg.text(x, bottom + 48, node, cls="tick", anchor="middle", rotate=35)
    svg.save(path)


def plot_request_to_apply_latency_boxplot_focused(events: list[dict], path: Path, *, cap_ms: float = 500.0) -> None:
    rows = request_to_decision_latency_rows(events)
    if not rows:
        return
    route_order = infer_route_node_order(events)
    route_rank = {node: i + 1 for i, node in enumerate(route_order)}
    nodes = []
    for r in rows:
        node = str(r["node_id"])
        if node not in nodes:
            nodes.append(node)
    nodes.sort(key=lambda n: route_rank.get(n, 9999))
    by_node = {
        node: [
            safe_float(r["request_to_decision_wall_latency_ms"])
            for r in rows
            if r["node_id"] == node and math.isfinite(safe_float(r["request_to_decision_wall_latency_ms"])) and safe_float(r["request_to_decision_wall_latency_ms"]) >= 0
        ]
        for node in nodes
    }
    if not any(by_node.values()):
        by_node = {
            node: [
                safe_float(r["request_to_decision_sim_latency_ms"])
                for r in rows
                if r["node_id"] == node and math.isfinite(safe_float(r["request_to_decision_sim_latency_ms"])) and safe_float(r["request_to_decision_sim_latency_ms"]) >= 0
            ]
            for node in nodes
        }
        ylabel = "Simulation-Time Request-to-Apply Latency (ms)"
    else:
        ylabel = "Request-to-Apply Latency (ms)"
    nodes = [n for n in nodes if by_node.get(n)]
    if not nodes:
        return
    width, height = paper_dim(1380, 675)
    svg = Svg(width, height, "")
    svg.text(36, 56, f"Focused active SI-DT request-to-apply latency. Values above {cap_ms:.0f} ms are counted as delayed coordination outliers.", cls="subtitle")
    left, right, top, bottom = 90, width - 60, 104, height - 165
    draw_y_axis(svg, left, top, bottom, 0, cap_ms, ticks=5, label=ylabel, label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    box_w = min(42, group_w * 0.55)
    rng = np.random.default_rng(33)
    color = "#E07A32"
    outlier_color = "#C44E52"
    for i, node in enumerate(nodes):
        vals_all = np.array(by_node[node], dtype=float)
        vals = np.array([min(v, cap_ms) for v in vals_all], dtype=float)
        x = left + group_w * (i + 0.5)
        q1, med, q3 = np.percentile(vals, [25, 50, 75])
        low, high = float(np.min(vals)), float(np.max(vals))
        yq1, ymed, yq3 = (y_scale(v, 0, cap_ms, top, bottom) for v in (q1, med, q3))
        ylow, yhigh = y_scale(low, 0, cap_ms, top, bottom), y_scale(high, 0, cap_ms, top, bottom)
        svg.line(x, ylow, x, yhigh, stroke=color, sw=1.5)
        svg.line(x - box_w * 0.35, ylow, x + box_w * 0.35, ylow, stroke=color, sw=1.5)
        svg.line(x - box_w * 0.35, yhigh, x + box_w * 0.35, yhigh, stroke=color, sw=1.5)
        svg.rect(x - box_w / 2, yq3, box_w, max(1, yq1 - yq3), fill=color, stroke=color, sw=1, opacity=0.32, rx=3)
        svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2.2)
        inliers = [v for v in vals_all if v <= cap_ms]
        for v in inliers:
            svg.circle(x + float(rng.uniform(-4, 4)), y_scale(v, 0, cap_ms, top, bottom), r=2.0, fill=color, opacity=0.42)
        outliers = int(sum(v > cap_ms for v in vals_all))
        if outliers:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 2:.2f} {x - 5:.2f},{top + 8:.2f} {x + 5:.2f},{top + 8:.2f}" fill="{outlier_color}" opacity="0.88"/>'
            )
            svg.text(x, top + 24, f"+{outliers}", cls="small", size=max(11, paper_value_size(14) - 2), fill=outlier_color, anchor="middle")
        rank = route_rank.get(node, i + 1)
        svg.text(x, bottom + 26, f"I{rank}", cls="xtick", anchor="middle")
        svg.text(x, bottom + 58, node, cls="tick", anchor="middle", rotate=paper_xtick_rotation(35))
    svg.text(left + 10, top - 20, f"Triangle/+n = samples above {cap_ms:.0f} ms", cls="legend", fill=outlier_color)
    svg.save(path)


def plot_request_to_decision_latency_normalized(events: list[dict], path: Path) -> None:
    rows = request_to_decision_latency_rows(events)
    if not rows:
        return
    nodes = []
    for r in rows:
        node = str(r["node_id"])
        if node not in nodes:
            nodes.append(node)
    metric_key = "request_to_decision_wall_latency_ms"
    by_node = {
        node: [
            safe_float(r[metric_key])
            for r in rows
            if r["node_id"] == node and math.isfinite(safe_float(r[metric_key]))
        ]
        for node in nodes
    }
    if not any(by_node.values()):
        metric_key = "request_to_decision_sim_latency_ms"
        by_node = {
            node: [
                safe_float(r[metric_key])
                for r in rows
                if r["node_id"] == node and math.isfinite(safe_float(r[metric_key]))
            ]
            for node in nodes
        }
    p95s = {node: float(np.percentile(vals, 95)) for node, vals in by_node.items() if vals}
    if not p95s:
        return
    max_p95 = max(p95s.values()) or 1.0
    svg = Svg(1180, 620, "Normalized request-to-decision latency")
    svg.text(36, 56, "Node p95 normalized by the largest p95 in this route; useful when raw latency ranges are heterogeneous.", cls="subtitle")
    left, right, top, bottom = 90, 1120, 100, 470
    draw_y_axis(svg, left, top, bottom, 0, 1.0, ticks=5, label="Normalized p95 Latency")
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(46, group_w * 0.58)
    for i, node in enumerate(nodes):
        if node not in p95s:
            continue
        value = p95s[node] / max_p95
        x = left + group_w * (i + 0.5) - bar_w / 2
        y = y_scale(value, 0, 1.0, top, bottom)
        svg.rect(x, y, bar_w, bottom - y, fill="#E07A32", opacity=0.72, rx=3)
        svg.text(x + bar_w / 2, bottom + 24, f"I{i + 1}", cls="tick", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 50, node, cls="tick", anchor="middle", rotate=35)
    svg.save(path)


def request_latency_component_rows(events: list[dict]) -> list[dict]:
    node_order = infer_route_node_order(events)
    node_rank = {node: i + 1 for i, node in enumerate(node_order)}
    window = representative_time_window(events)
    requests: list[dict] = []
    applies: list[dict] = []
    apply_events = {"f2.apply", "f2.strict_b1_floor.apply", "f2.b1_continuity.apply"}
    for e in events:
        et = str(e.get("event_type", "") or "")
        tls = event_node_id(e)
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        wall_ms = safe_float(e.get("ts_wall_ms"))
        if tls == "unknown" or not in_window(sim, window):
            continue
        if et == "ev.request.dispatched":
            age_ms = safe_float(e.get("age_ms"))
            if not math.isfinite(age_ms):
                age_ms = max(0.0, (safe_float(e.get("dispatch_sim_time")) - safe_float(e.get("request_sim_time"))) * 1000.0)
            requests.append({"tls": tls, "sim": sim, "wall_ms": wall_ms, "fnm_ms": age_ms})
        elif et in apply_events:
            applies.append({"tls": tls, "sim": sim, "wall_ms": wall_ms, "event_type": et})
    applies_by_tls: dict[str, list[dict]] = defaultdict(list)
    for a in applies:
        applies_by_tls[str(a["tls"])].append(a)
    for arr in applies_by_tls.values():
        arr.sort(key=lambda x: float(x["sim"]))
    rows = []
    for req in sorted(requests, key=lambda x: float(x["sim"])):
        tls = str(req["tls"])
        candidate = None
        for app in applies_by_tls.get(tls, []):
            dt = float(app["sim"]) - float(req["sim"])
            if 0.0 <= dt <= 5.0:
                candidate = app
                break
        if candidate is None:
            continue
        wall_delta = math.nan
        if math.isfinite(float(req["wall_ms"])) and math.isfinite(float(candidate["wall_ms"])):
            wall_delta = max(0.0, float(candidate["wall_ms"]) - float(req["wall_ms"]))
        sim_delta = max(0.0, (float(candidate["sim"]) - float(req["sim"])) * 1000.0)
        decision_ms = wall_delta if math.isfinite(wall_delta) else sim_delta
        fnm_ms = max(0.0, float(req["fnm_ms"])) if math.isfinite(float(req["fnm_ms"])) else 0.0
        rows.append({
            "route_progression_idx": node_rank.get(tls, 9999),
            "node_id": tls,
            "request_sim_time_s": float(req["sim"]),
            "fnm_request_mediation_ms": fnm_ms,
            "si_dt_decision_apply_ms": decision_ms,
            "total_request_to_apply_ms": fnm_ms + decision_ms,
            "decision_event_type": str(candidate["event_type"]),
        })
    rows.sort(key=lambda r: (int(r["route_progression_idx"]), float(r["request_sim_time_s"])))
    return rows


def plot_request_latency_components_by_node(events: list[dict], path: Path) -> None:
    rows = request_latency_component_rows(events)
    if not rows:
        return
    route_order = infer_route_node_order(events)
    route_rank = {node: i + 1 for i, node in enumerate(route_order)}
    nodes = []
    for r in rows:
        node = str(r["node_id"])
        if node not in nodes:
            nodes.append(node)
    means = {}
    for node in nodes:
        subset = [r for r in rows if r["node_id"] == node]
        means[node] = {
            "fnm": mean([float(r["fnm_request_mediation_ms"]) for r in subset]),
            "decision": mean([float(r["si_dt_decision_apply_ms"]) for r in subset]),
            "active_request_n": len(subset),
        }
    vmax = max(v["fnm"] + v["decision"] for v in means.values()) if means else 1.0
    width, height = paper_dim(1380, 675)
    svg = Svg(width, height, "")
    svg.text(36, 56, "Active SI-DT request handling only: EV/FNM request mediation plus local decision/apply latency, averaged by route-ordered controllable node.", cls="subtitle")
    left, right, top, bottom = 108, width - 60, 108, height - 165
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label="Request-to-Decision|Mean Latency (ms)", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(44, group_w * 0.58)
    colors = {"fnm": "#1B9E77", "decision": "#2D7DD2"}
    for i, node in enumerate(nodes):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for key in ["fnm", "decision"]:
            value = means[node][key]
            h = (bottom - top) * value / max(1.0, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors[key], opacity=1.0, rx=2)
            y_cursor -= h
        total = means[node]["fnm"] + means[node]["decision"]
        svg.text(x + bar_w / 2, y_cursor - 8, f"{total:.0f}", cls="small", size=paper_value_size(18), anchor="middle")
        rank = route_rank.get(node, i + 1)
        svg.text(x + bar_w / 2, bottom + 28, f"I{rank}", cls="xtick", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 60, node, cls="tick", anchor="middle", rotate=paper_xtick_rotation(35))
    legend_w = 680
    pos = paper_legend_position("top-center")
    lx = 24 if pos == "top-left" else max(24, width - legend_w - 24) if pos == "top-right" else (width - legend_w) / 2
    ly = 82
    svg.rect(lx, ly - 12, 18, 12, fill=colors["fnm"], opacity=1.0)
    svg.text(lx + 26, ly - 2, "FNM request mediation", cls="legend")
    svg.rect(lx + 300, ly - 12, 18, 12, fill=colors["decision"], opacity=1.0)
    svg.text(lx + 326, ly - 2, "Local SI-DT decision/apply", cls="legend")
    svg.save(path)


def plot_request_to_decision_latency_summary(events: list[dict], path: Path) -> None:
    rows = request_to_decision_latency_rows(events)
    if not rows:
        return
    nodes = []
    for r in rows:
        node = str(r["node_id"])
        if node not in nodes:
            nodes.append(node)
    by_node = {}
    metric_key = "request_to_decision_wall_latency_ms"
    for node in nodes:
        vals = [
            safe_float(r[metric_key])
            for r in rows
            if r["node_id"] == node and math.isfinite(safe_float(r[metric_key]))
        ]
        by_node[node] = vals
    if not any(by_node.values()):
        metric_key = "request_to_decision_sim_latency_ms"
        by_node = {
            node: [
                safe_float(r[metric_key])
                for r in rows
                if r["node_id"] == node and math.isfinite(safe_float(r[metric_key]))
            ]
            for node in nodes
        }
    stats = {}
    for node, vals in by_node.items():
        if vals:
            stats[node] = (float(np.percentile(vals, 50)), float(np.percentile(vals, 95)))
    if not stats:
        return
    vmax = max(v[1] for v in stats.values())
    svg = Svg(1380, 680, "Request-to-decision latency summary")
    svg.text(36, 56, "Median bars with p95 markers provide a cleaner alternative to dense node-level boxplots.", cls="subtitle")
    left, right, top, bottom = 92, 1320, 105, 520
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label="Latency (ms)")
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(44, group_w * 0.58)
    for i, node in enumerate(nodes):
        if node not in stats:
            continue
        med, p95 = stats[node]
        x = left + group_w * (i + 0.5) - bar_w / 2
        h = (bottom - top) * med / max(1.0, vmax * 1.15)
        y_p95 = y_scale(p95, 0, max(1.0, vmax * 1.15), top, bottom)
        svg.rect(x, bottom - h, bar_w, h, fill="#E07A32", opacity=0.72, rx=2)
        svg.line(x - 4, y_p95, x + bar_w + 4, y_p95, stroke="#263238", sw=1.5)
        svg.text(x + bar_w / 2, y_p95 - 6, f"{p95:.0f}", cls="small", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 22, f"I{i + 1}", cls="tick", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 48, node, cls="tick", anchor="middle", rotate=35)
    svg.text(left + 10, 82, "Bar = median; black mark = p95", cls="legend")
    svg.save(path)


def plot_runtime_event_burst(events: list[dict], path: Path) -> None:
    base_window = representative_time_window(events)
    window = (max(0.0, base_window[0] - 50.0), base_window[1] + 50.0)
    items = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if cat in EVENT_CATEGORY_CHOREOGRAPHY_ORDER and in_window(sim, window):
            items.append((sim, cat))
    labels = [c for c in EVENT_CATEGORY_CHOREOGRAPHY_ORDER if any(label == c for _, label in items)]
    centers, counts = binned_counts_fixed(items, 5.0, window, labels)
    if not centers:
        return
    totals = [sum(counts[label][i] for label in labels) for i in range(len(centers))]
    ymax = max(totals) if totals else 1
    svg = Svg(1280, 720, "Runtime federation activity burst")
    svg.text(36, 56, "Stacked activity per 5 simulated seconds; 50s margins show the federation burst around the EV-triggered coordination window.", cls="subtitle")
    left, right, top, bottom = 88, 1210, 136, 560
    draw_y_axis(svg, left, top, bottom, 0, max(1, ymax * 1.15), ticks=6, label="Events per 5 s Bin (count)")
    xmin, xmax = window
    bar_w = max(2.0, (right - left) / max(1, len(centers)) * 0.85)
    for i, t in enumerate(centers):
        x = left + (t - xmin) / (xmax - xmin or 1) * (right - left) - bar_w / 2
        y_cursor = bottom
        for label in labels:
            value = counts[label][i]
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, ymax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=EVENT_CATEGORY_COLORS[label], opacity=0.72)
            y_cursor -= h
    x_ticks = 6
    for i in range(x_ticks):
        t = xmin + (xmax - xmin) * i / max(1, x_ticks - 1)
        x = left + (right - left) * i / max(1, x_ticks - 1)
        svg.text(x, bottom + 24, f"{t:.0f}", cls="tick", anchor="middle")
    draw_category_legend(svg, labels, left, 90, columns=5, col_w=205)
    svg.text((left + right) / 2, 604, "Simulation Time (s)", cls="label", anchor="middle")
    svg.save(path)


def plot_runtime_event_burst_area(
    events: list[dict],
    path: Path,
    *,
    label_map: dict[str, str] | None = None,
    title: str = "Runtime federation activity area",
    compact: bool = False,
) -> None:
    base_window = representative_time_window(events)
    window = compact_runtime_window(events) if compact else (max(0.0, base_window[0] - 50.0), base_window[1] + 50.0)
    items = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if cat in EVENT_CATEGORY_CHOREOGRAPHY_ORDER and in_window(sim, window):
            items.append((sim, cat))
    labels = [c for c in EVENT_CATEGORY_CHOREOGRAPHY_ORDER if any(label == c for _, label in items)]
    centers, counts = binned_counts_fixed(items, 5.0, window, labels)
    if not centers:
        return
    totals = [sum(counts[label][i] for label in labels) for i in range(len(centers))]
    ymax = max(totals) if totals else 1
    if compact:
        width, base_height = paper_dim(900, 455)
        height = paired_runtime_panel_height(events, base_height)
        left, right, top, bottom = PAPER_2_1_LEFT_MARGIN, width - 42, 128, height - 82
        legend_y = 66
        legend_cols = 3
        legend_col_w = PAPER_2_1_LEGEND_COLUMN_WIDTH
    else:
        width, height = 1280, 720
        left, right, top, bottom = 88, 1210, 136, 560
        legend_y = 90
        legend_cols = 5
        legend_col_w = 205
    svg = Svg(width, height, "" if compact else title)
    svg.text(36, 56, "Stacked area version of the federation activity burst; quiet margins make the EV-triggered runtime episode visible.", cls="subtitle")
    draw_y_axis(
        svg,
        left,
        top,
        bottom,
        0,
        max(1, ymax * 1.15),
        ticks=6,
        label="Federation Service|Activity (count)" if compact else "Events per 5 s Bin (count)",
        label_x=PAPER_2_1_SERVICE_YLABEL_X if compact else 20,
        grid_right=right,
    )
    xmin, xmax = window
    x_ticks = 7 if compact else 6
    for i in range(x_ticks):
        x = left + (right - left) * i / max(1, x_ticks - 1)
        svg.line(x, top, x, bottom, stroke="#edf2f6", sw=1)
    baseline = [0.0] * len(centers)
    for label in labels:
        upper = [baseline[i] + counts[label][i] for i in range(len(centers))]
        pts_top = [
            (left + (t - xmin) / (xmax - xmin or 1) * (right - left), y_scale(v, 0, max(1, ymax * 1.15), top, bottom))
            for t, v in zip(centers, upper)
        ]
        pts_bottom = [
            (left + (t - xmin) / (xmax - xmin or 1) * (right - left), y_scale(v, 0, max(1, ymax * 1.15), top, bottom))
            for t, v in reversed(list(zip(centers, baseline)))
        ]
        points = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts_top + pts_bottom)
        svg.parts.append(f'<polygon points="{points}" fill="{EVENT_CATEGORY_COLORS[label]}" opacity="0.86"/>')
        baseline = upper
    for i in range(x_ticks):
        t = xmin + (xmax - xmin) * i / max(1, x_ticks - 1)
        x = left + (right - left) * i / max(1, x_ticks - 1)
        svg.text(x, bottom + 28, f"{t:.0f}", cls="xtick" if compact else "tick", size=int(active_paper_layout().get("xtick_size", 20)) if compact else None, anchor="middle")
    if compact:
        draw_positioned_category_legend(svg, labels, legend_y, label_map=label_map, columns=legend_cols, col_w=legend_col_w, position=paper_legend_position("top-center"))
    else:
        draw_category_legend(svg, labels, left, legend_y, label_map=label_map, columns=legend_cols, col_w=legend_col_w)
    svg.text((left + right) / 2, bottom + (60 if compact else 58), "Simulation Time (s)", cls="label", size=paper_x_label_size(), anchor="middle")
    svg.save(path)


def plot_critical_node_timeline(events: list[dict], path: Path) -> None:
    window = representative_time_window(events)
    counts = Counter()
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        node = event_node_id(e)
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if node != "unknown" and cat in EVENT_CATEGORY_ORDER and in_window(sim, window):
            counts[node] += 1
    if not counts:
        return
    critical_node = counts.most_common(1)[0][0]
    lanes = ["EV discovery", "EV request", "Apply", "Passive-supported guard/rescue", "Guard/skip", "Passive context"]
    points = []
    for e in events:
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if event_node_id(e) == critical_node and cat in lanes and in_window(sim, window):
            points.append((sim, cat))
    if not points:
        return
    svg = Svg(1280, 720, f"Critical SI-DT interaction timeline ({critical_node})")
    svg.text(36, 56, "Most active route node; shows when EV requests, TLS applies, passive support, and guards occur.", cls="subtitle")
    left, right, top, bottom = 220, 1210, 105, 560
    xmin, xmax = window
    lane_h = (bottom - top) / max(1, len(lanes) - 1)
    for i, lane in enumerate(lanes):
        y = top + i * lane_h
        svg.line(left, y, right, y, stroke="#e6edf2")
        svg.text(left - 12, y + 4, EVENT_CATEGORY_SHORT_LABELS[lane], cls="label", anchor="end")
    rng = np.random.default_rng(73)
    passive_seen = 0
    for sim, lane in points:
        if lane == "Passive context":
            passive_seen += 1
            if passive_seen % 10 != 0:
                continue
        x = left + (sim - xmin) / (xmax - xmin or 1) * (right - left)
        y = top + lanes.index(lane) * lane_h + float(rng.uniform(-2.5, 2.5))
        svg.circle(x, y, r=3.8 if lane != "Passive context" else 2.2, fill=EVENT_CATEGORY_COLORS[lane], opacity=0.72)
    for i in range(6):
        t = xmin + (xmax - xmin) * i / 5
        x = left + (right - left) * i / 5
        svg.line(x, bottom + 8, x, bottom + 14, stroke="#70808f")
        svg.text(x, bottom + 32, f"{t:.0f}", cls="tick", anchor="middle")
    svg.text((left + right) / 2, 632, "Simulation Time (s)", cls="label", anchor="middle")
    svg.save(path)


def plot_service_latency_boxplots(messages: list[dict], path: Path) -> None:
    by_group: dict[str, list[float]] = defaultdict(list)
    for m in messages:
        payload = m.get("payload")
        if isinstance(payload, dict):
            v = safe_float(payload.get("latency_ms"))
            if math.isfinite(v):
                by_group[str(m.get("_topic_group", "Other"))].append(v)
    labels = [g for g, vals in sorted(by_group.items(), key=lambda kv: len(kv[1]), reverse=True) if vals]
    if not labels:
        return
    values = [v for g in labels for v in by_group[g]]
    vmax = max(values)
    svg = Svg(1180, 650, "Service latency by topic family")
    svg.text(36, 56, "Boxplots use payload latency_ms fields found in route-scoped raw FNM messages.", cls="subtitle")
    left, right, top, bottom = 90, 1120, 100, 500
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label="Latency (ms)")
    group_w = (right - left) / max(1, len(labels))
    box_w = min(64, group_w * 0.55)
    for i, label in enumerate(labels):
        vals = np.array(by_group[label], dtype=float)
        x = left + group_w * (i + 0.5)
        q1, med, q3 = np.percentile(vals, [25, 50, 75])
        low, high = float(np.min(vals)), float(np.max(vals))
        yq1, ymed, yq3 = (y_scale(v, 0, max(1.0, vmax * 1.15), top, bottom) for v in (q1, med, q3))
        ylow, yhigh = y_scale(low, 0, max(1.0, vmax * 1.15), top, bottom), y_scale(high, 0, max(1.0, vmax * 1.15), top, bottom)
        svg.line(x, ylow, x, yhigh, stroke="#263238", sw=1.2)
        svg.rect(x - box_w / 2, yq3, box_w, max(1, yq1 - yq3), fill="#2D7DD2", stroke="#2D7DD2", opacity=0.32, rx=3)
        svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke="#2D7DD2", sw=2)
        svg_multiline_text(svg, x, bottom + 24, TOPIC_FAMILY_LABELS.get(label, label).replace("\n", "|"), cls="tick", line_h=14, anchor="middle")
    svg.text((left + right) / 2, 628, "Service / topic family", cls="label", anchor="middle")
    svg.save(path)


def draw_svg_boxplot(svg: Svg, x: float, vals: list[float], *, ymin: float, ymax: float, top: float, bottom: float, box_w: float, color: str, rng: np.random.Generator | None = None) -> None:
    if not vals:
        return
    arr = np.array(vals, dtype=float)
    q1, med, q3 = np.percentile(arr, [25, 50, 75])
    low, high = float(np.min(arr)), float(np.max(arr))
    yq1, ymed, yq3 = (y_scale(v, ymin, ymax, top, bottom) for v in (q1, med, q3))
    ylow, yhigh = y_scale(low, ymin, ymax, top, bottom), y_scale(high, ymin, ymax, top, bottom)
    svg.line(x, ylow, x, yhigh, stroke=color, sw=1.5)
    svg.line(x - box_w * 0.35, ylow, x + box_w * 0.35, ylow, stroke=color, sw=1.5)
    svg.line(x - box_w * 0.35, yhigh, x + box_w * 0.35, yhigh, stroke=color, sw=1.5)
    svg.rect(x - box_w / 2, yq3, box_w, max(1, yq1 - yq3), fill=color, stroke=color, sw=1, opacity=0.30, rx=3)
    svg.line(x - box_w / 2, ymed, x + box_w / 2, ymed, stroke=color, sw=2.2)
    if rng is not None:
        sample = arr if len(arr) <= 160 else rng.choice(arr, size=160, replace=False)
        for v in sample:
            svg.circle(x + float(rng.uniform(-5, 5)), y_scale(float(v), ymin, ymax, top, bottom), r=1.8, fill=color, opacity=0.30)


def latency_candidate_values(events: list[dict], messages: list[dict], *, defer_threshold_ms: float = 500.0) -> list[dict]:
    out: list[dict] = []
    for e in events:
        if str(e.get("event_type", "")) != "ev.request.dispatched":
            continue
        v = safe_float(e.get("age_ms"))
        if not math.isfinite(v):
            v = max(0.0, (safe_float(e.get("dispatch_sim_time")) - safe_float(e.get("request_sim_time"))) * 1000.0)
        if math.isfinite(v):
            out.append({
                "process": "FNM request mediation",
                "latency_ms": v,
                "source": "ev.request.dispatched.age_ms",
            })
    for m in messages:
        payload = m.get("payload")
        if not isinstance(payload, dict):
            continue
        v = safe_float(payload.get("latency_ms"))
        if math.isfinite(v):
            out.append({
                "process": "Core service handling",
                "latency_ms": v,
                "source": "payload.latency_ms",
                "topic_group": str(m.get("_topic_group", "Other")),
            })
    for r in request_to_decision_latency_rows(events):
        v = safe_float(r.get("request_to_decision_wall_latency_ms"))
        source = "wall"
        if not math.isfinite(v) or v < 0:
            v = safe_float(r.get("request_to_decision_sim_latency_ms"))
            source = "sim"
        if not math.isfinite(v) or v < 0:
            continue
        process = "Active request-to-apply"
        if v > defer_threshold_ms:
            process = "Deferred/gated apply"
        out.append({
            "process": process,
            "latency_ms": v,
            "source": f"request_to_apply_{source}",
            "node_id": r.get("node_id", ""),
            "route_progression_idx": r.get("route_progression_idx", ""),
            "decision_event_type": r.get("decision_event_type", ""),
        })
    return out


def plot_latency_candidate_boxplots(events: list[dict], messages: list[dict], path: Path) -> None:
    rows = latency_candidate_values(events, messages)
    order = [
        "Core service handling",
        "FNM request mediation",
        "Active request-to-apply",
        "Deferred/gated apply",
    ]
    by_process = {
        label: [float(r["latency_ms"]) for r in rows if r.get("process") == label and math.isfinite(float(r.get("latency_ms", math.nan)))]
        for label in order
    }
    labels = [label for label in order if by_process.get(label)]
    if not labels:
        return
    all_vals = [v for label in labels for v in by_process[label]]
    vmax = max(1.0, float(np.percentile(all_vals, 98)) * 1.15)
    width, height = paper_dim(1180, 650)
    svg = Svg(width, height, "")
    svg.text(36, 56, "Candidate latency decomposition by observable process. Deferred/gated apply separates policy/control-loop waiting from normal active request handling.", cls="subtitle")
    left, right, top, bottom = 108, width - 60, 104, height - 140
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / max(1, len(labels))
    box_w = min(86, group_w * 0.48)
    colors = {
        "Core service handling": "#9467BD",
        "FNM request mediation": "#1B9E77",
        "Active request-to-apply": "#2D7DD2",
        "Deferred/gated apply": "#C44E52",
    }
    rng = np.random.default_rng(41)
    for i, label in enumerate(labels):
        vals_all = by_process[label]
        vals_visible = [min(v, vmax) for v in vals_all]
        x = left + group_w * (i + 0.5)
        draw_svg_boxplot(svg, x, vals_visible, ymin=0, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color=colors[label], rng=rng)
        over = sum(v > vmax for v in vals_all)
        if over:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 2:.2f} {x - 6:.2f},{top + 10:.2f} {x + 6:.2f},{top + 10:.2f}" fill="{colors[label]}" opacity="0.88"/>'
            )
            svg.text(x, top + 26, f"+{over}", cls="small", size=max(11, paper_value_size(14) - 2), fill=colors[label], anchor="middle")
        svg.text(x, bottom + 28, label.replace(" ", "|"), cls="xtick", anchor="middle")
        svg.text(x, bottom + 74, f"n={len(vals_all)}", cls="small", anchor="middle")
    svg.text(left + 8, top - 20, "Whiskers/points capped at p98-derived axis; +n marks larger delayed samples", cls="legend", fill="#596773")
    svg.save(path)


def write_latency_candidate_csv(events: list[dict], messages: list[dict], path: Path) -> None:
    rows = latency_candidate_values(events, messages)
    fieldnames = ["process", "latency_ms", "source", "topic_group", "node_id", "route_progression_idx", "decision_event_type"]
    for r in rows:
        r["latency_ms"] = f"{float(r['latency_ms']):.3f}"
    write_csv(path, rows, fieldnames)


def latency_component_process_rows(component_rows: list[dict], *, deferred_threshold_ms: float = 500.0) -> list[dict]:
    """Split request handling into process buckets that are defensible from the logged fields."""
    out: list[dict] = []
    for r in component_rows:
        node = str(r.get("node_id", ""))
        route_idx = r.get("route_progression_idx", "")
        decision_type = str(r.get("decision_event_type", ""))
        fnm_ms = safe_float(r.get("fnm_request_mediation_ms"))
        local_ms = safe_float(r.get("si_dt_decision_apply_ms"))
        total_ms = safe_float(r.get("total_request_to_apply_ms"))
        if math.isfinite(fnm_ms):
            out.append({
                "process": "FNM request mediation",
                "latency_ms": fnm_ms,
                "node_id": node,
                "route_progression_idx": route_idx,
                "decision_event_type": decision_type,
            })
        if math.isfinite(local_ms):
            out.append({
                "process": "Local SI-DT decision/apply",
                "latency_ms": local_ms,
                "node_id": node,
                "route_progression_idx": route_idx,
                "decision_event_type": decision_type,
            })
        if math.isfinite(total_ms):
            out.append({
                "process": "Active request-to-apply" if total_ms <= deferred_threshold_ms else "Deferred/gated apply",
                "latency_ms": total_ms,
                "node_id": node,
                "route_progression_idx": route_idx,
                "decision_event_type": decision_type,
            })
    return out


def plot_latency_component_process_boxplots(component_rows: list[dict], path: Path) -> None:
    rows = latency_component_process_rows(component_rows)
    order = [
        "FNM request mediation",
        "Local SI-DT decision/apply",
        "Active request-to-apply",
        "Deferred/gated apply",
    ]
    by_process: dict[str, list[float]] = defaultdict(list)
    for r in rows:
        v = safe_float(r.get("latency_ms"))
        if math.isfinite(v):
            by_process[str(r.get("process", ""))].append(v)
    labels = [label for label in order if by_process.get(label)]
    if not labels:
        return
    all_vals = [v for label in labels for v in by_process[label]]
    vmax = max(1.0, float(np.percentile(all_vals, 98)) * 1.15)
    width, height = paper_dim(1180, 640)
    svg = Svg(width, height, "")
    svg.text(36, 56, "Request handling decomposition. Delayed gated samples are separated so normal middleware/process latency is not visually dominated by policy waiting.", cls="subtitle")
    left, right, top, bottom = 116, width - 58, 104, height - 142
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(labels))
    box_w = min(92, group_w * 0.50)
    colors = {
        "FNM request mediation": "#1B9E77",
        "Local SI-DT decision/apply": "#2D7DD2",
        "Active request-to-apply": "#9467BD",
        "Deferred/gated apply": "#C44E52",
    }
    rng = np.random.default_rng(61)
    for i, label in enumerate(labels):
        vals_all = by_process[label]
        vals_visible = [min(v, vmax) for v in vals_all]
        x = left + group_w * (i + 0.5)
        draw_svg_boxplot(svg, x, vals_visible, ymin=0, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color=colors[label], rng=rng)
        over = sum(v > vmax for v in vals_all)
        if over:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 3:.2f} {x - 7:.2f},{top + 11:.2f} {x + 7:.2f},{top + 11:.2f}" fill="{colors[label]}" opacity="0.90"/>'
            )
            svg.text(x, top + 29, f"+{over}", cls="small", size=paper_value_size(14), fill=colors[label], anchor="middle")
        svg_multiline_text(svg, x, bottom + 28, label.replace(" request-to-", " request-to-|").replace(" decision/", " decision/|").replace(" ", "|"), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 100, f"n={len(vals_all)}", cls="small", anchor="middle")
    svg.text(left + 8, top - 20, "Whiskers/points capped at p98-derived axis; +n marks larger delayed samples", cls="legend", fill="#596773")
    svg.save(path)


def policy_latency_label(event_type: str) -> str:
    if event_type == "f2.strict_b1_floor.apply":
        return "Strict B1-floor apply"
    if event_type == "f2.b1_continuity.apply":
        return "B1-continuity apply"
    if "passive" in event_type or "f2p" in event_type:
        return "F2P passive-supported apply"
    if event_type == "f2.apply":
        return "F2 apply"
    return event_type or "Other apply"


def plot_latency_by_policy_boxplots(component_rows: list[dict], path: Path) -> None:
    by_policy: dict[str, list[float]] = defaultdict(list)
    for r in component_rows:
        v = safe_float(r.get("total_request_to_apply_ms"))
        if math.isfinite(v):
            by_policy[policy_latency_label(str(r.get("decision_event_type", "")))].append(v)
    labels = [label for label, vals in sorted(by_policy.items(), key=lambda kv: len(kv[1]), reverse=True) if vals]
    if not labels:
        return
    all_vals = [v for label in labels for v in by_policy[label]]
    vmax = max(1.0, float(np.percentile(all_vals, 98)) * 1.15)
    width, height = paper_dim(980, 610)
    svg = Svg(width, height, "")
    svg.text(36, 56, "End-to-end request-to-apply latency grouped by the policy path that produced the SI-DT apply event.", cls="subtitle")
    left, right, top, bottom = 112, width - 58, 104, height - 136
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Request-to-apply latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(labels))
    box_w = min(94, group_w * 0.55)
    colors = ["#2D7DD2", "#1B9E77", "#9467BD", "#C44E52", "#E07A32"]
    rng = np.random.default_rng(62)
    for i, label in enumerate(labels):
        vals_all = by_policy[label]
        vals_visible = [min(v, vmax) for v in vals_all]
        x = left + group_w * (i + 0.5)
        color = colors[i % len(colors)]
        draw_svg_boxplot(svg, x, vals_visible, ymin=0, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color=color, rng=rng)
        over = sum(v > vmax for v in vals_all)
        if over:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 3:.2f} {x - 7:.2f},{top + 11:.2f} {x + 7:.2f},{top + 11:.2f}" fill="{color}" opacity="0.90"/>'
            )
            svg.text(x, top + 29, f"+{over}", cls="small", size=paper_value_size(14), fill=color, anchor="middle")
        svg_multiline_text(svg, x, bottom + 28, label.replace(" ", "|"), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 100, f"n={len(vals_all)}", cls="small", anchor="middle")
    svg.save(path)


def plot_active_pipeline_latency_boxplots(component_rows: list[dict], path: Path, *, active_threshold_ms: float = 500.0) -> None:
    active = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms]
    if not active:
        return
    groups = [
        ("FNM request mediation", [safe_float(r.get("fnm_request_mediation_ms")) for r in active], "#1B9E77"),
        ("Local SI-DT decision/apply", [safe_float(r.get("si_dt_decision_apply_ms")) for r in active], "#2D7DD2"),
        ("Active request-to-apply", [safe_float(r.get("total_request_to_apply_ms")) for r in active], "#9467BD"),
    ]
    groups = [(label, [v for v in vals if math.isfinite(v)], color) for label, vals, color in groups]
    groups = [(label, vals, color) for label, vals, color in groups if vals]
    if not groups:
        return
    vmax = max(active_threshold_ms, max(v for _, vals, _ in groups for v in vals) * 1.10)
    width, height = paper_dim(960, 560)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Normal in-window coordination only; delayed safety/control-loop waits are excluded from this active pipeline view.", cls="subtitle")
    left, right, top, bottom = 112, width - 54, 96, height - 124
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(groups))
    box_w = min(92, group_w * 0.50)
    rng = np.random.default_rng(63)
    for i, (label, vals, color) in enumerate(groups):
        x = left + group_w * (i + 0.5)
        draw_svg_boxplot(svg, x, vals, ymin=0, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color=color, rng=rng)
        svg_multiline_text(svg, x, bottom + 28, label.replace(" request-to-", " request-to-|").replace(" decision/", " decision/|").replace(" ", "|"), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 94, f"n={len(vals)}", cls="small", anchor="middle")
    svg.save(path)


def plot_deferred_guard_latency_boxplots(component_rows: list[dict], path: Path, *, active_threshold_ms: float = 500.0) -> None:
    delayed = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) > active_threshold_ms]
    if not delayed:
        return
    groups = [
        ("Local gated wait/apply", [safe_float(r.get("si_dt_decision_apply_ms")) for r in delayed], "#C44E52"),
        ("Deferred request-to-apply", [safe_float(r.get("total_request_to_apply_ms")) for r in delayed], "#8C1D40"),
    ]
    groups = [(label, [v for v in vals if math.isfinite(v)], color) for label, vals, color in groups]
    groups = [(label, vals, color) for label, vals, color in groups if vals]
    if not groups:
        return
    vmax = max(1.0, float(np.percentile([v for _, vals, _ in groups for v in vals], 98)) * 1.15)
    width, height = paper_dim(860, 560)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Delayed samples represent gated/safety-controlled actuation, not steady-state middleware communication latency.", cls="subtitle")
    left, right, top, bottom = 112, width - 54, 96, height - 124
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(groups))
    box_w = min(104, group_w * 0.45)
    rng = np.random.default_rng(64)
    for i, (label, vals_all, color) in enumerate(groups):
        vals = [min(v, vmax) for v in vals_all]
        x = left + group_w * (i + 0.5)
        draw_svg_boxplot(svg, x, vals, ymin=0, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color=color, rng=rng)
        over = sum(v > vmax for v in vals_all)
        if over:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 3:.2f} {x - 7:.2f},{top + 11:.2f} {x + 7:.2f},{top + 11:.2f}" fill="{color}" opacity="0.90"/>'
            )
            svg.text(x, top + 29, f"+{over}", cls="small", size=paper_value_size(14), fill=color, anchor="middle")
        svg_multiline_text(svg, x, bottom + 28, label.replace(" ", "|"), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 76, f"n={len(vals_all)}", cls="small", anchor="middle")
    svg.save(path)


def route_ordered_latency_nodes(component_rows: list[dict]) -> list[str]:
    order: list[tuple[int, str]] = []
    seen = set()
    for r in sorted(component_rows, key=lambda x: (int(safe_float(x.get("route_progression_idx")) if math.isfinite(safe_float(x.get("route_progression_idx"))) else 9999), str(x.get("node_id", "")))):
        node = str(r.get("node_id", ""))
        if not node or node in seen:
            continue
        idx = int(safe_float(r.get("route_progression_idx")) if math.isfinite(safe_float(r.get("route_progression_idx"))) else len(order) + 1)
        order.append((idx, node))
        seen.add(node)
    return [node for _, node in sorted(order, key=lambda x: (x[0], x[1]))]


def latency_node_label(node: str, component_rows: list[dict], fallback_idx: int) -> str:
    idxs = [
        int(safe_float(r.get("route_progression_idx")))
        for r in component_rows
        if str(r.get("node_id", "")) == node and math.isfinite(safe_float(r.get("route_progression_idx")))
    ]
    idx = min(idxs) if idxs else fallback_idx
    return f"I{idx}|{node.replace('Node', 'N')}"


def plot_active_pipeline_latency_by_node(component_rows: list[dict], path: Path, *, active_threshold_ms: float = 500.0) -> None:
    active = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms]
    nodes = route_ordered_latency_nodes(active)
    if not nodes:
        return
    means = {}
    for node in nodes:
        subset = [r for r in active if str(r.get("node_id", "")) == node]
        means[node] = {
            "fnm": mean([safe_float(r.get("fnm_request_mediation_ms")) for r in subset]),
            "local": mean([safe_float(r.get("si_dt_decision_apply_ms")) for r in subset]),
            "n": len(subset),
        }
    vmax = max(1.0, max(v["fnm"] + v["local"] for v in means.values()) * 1.22)
    width, height = paper_dim(1320, 620)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Route-ordered active request pipeline. Bars show the mean FNM mediation and local SI-DT decision/apply contribution at each active node.", cls="subtitle")
    left, right, top, bottom = 112, width - 50, 108, height - 142
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Mean Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(46, group_w * 0.58)
    colors = {"fnm": "#1B9E77", "local": "#2D7DD2"}
    for i, node in enumerate(nodes):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for key in ["fnm", "local"]:
            value = means[node][key]
            if not math.isfinite(value):
                value = 0.0
            h = (bottom - top) * value / vmax
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors[key], opacity=0.92, rx=2)
            y_cursor -= h
        total = means[node]["fnm"] + means[node]["local"]
        svg.text(x + bar_w / 2, y_cursor - 9, f"{total:.0f}", cls="small", size=paper_value_size(15), anchor="middle")
        svg.text(x + bar_w / 2, y_cursor - 27, f"n={means[node]['n']}", cls="small", size=max(10, paper_value_size(12)), fill="#596773", anchor="middle")
        svg_multiline_text(svg, x + bar_w / 2, bottom + 26, latency_node_label(node, active, i + 1), cls="xtick", line_h=18, anchor="middle")
    legend_y = 82
    legend_x = (left + right) / 2 - 190
    svg.rect(legend_x, legend_y - 13, 16, 12, fill=colors["fnm"], opacity=0.92)
    svg.text(legend_x + 23, legend_y - 2, "FNM interoperability mediation", cls="legend")
    svg.rect(legend_x + 285, legend_y - 13, 16, 12, fill=colors["local"], opacity=0.92)
    svg.text(legend_x + 308, legend_y - 2, "Local SI-DT decision/apply", cls="legend")
    svg.save(path)


def key_latency_pipeline_nodes(component_rows: list[dict], *, active_threshold_ms: float = 500.0, max_nodes: int = 11) -> list[str]:
    nodes = route_ordered_latency_nodes(component_rows)
    if not nodes:
        return []
    scores = {}
    for node in nodes:
        subset = [r for r in component_rows if str(r.get("node_id", "")) == node]
        active_n = sum(1 for r in subset if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms)
        deferred_n = sum(1 for r in subset if safe_float(r.get("total_request_to_apply_ms")) > active_threshold_ms)
        scores[node] = (deferred_n > 0, deferred_n, active_n)
    selected = []
    if nodes:
        selected.append(nodes[0])
    deferred_nodes = [node for node in nodes if scores[node][0] and node not in selected]
    selected.extend(deferred_nodes)
    if len(selected) < max_nodes:
        remaining = [node for node in nodes if node not in selected]
        remaining.sort(key=lambda n: scores[n][2], reverse=True)
        selected.extend(remaining[: max_nodes - len(selected)])
    selected = selected[:max_nodes]
    rank = {node: i for i, node in enumerate(nodes)}
    return sorted(selected, key=lambda n: rank.get(n, 9999))


def plot_active_pipeline_latency_by_key_node(component_rows: list[dict], path: Path, *, active_threshold_ms: float = 500.0) -> None:
    active = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms]
    nodes = [node for node in key_latency_pipeline_nodes(component_rows, active_threshold_ms=active_threshold_ms) if any(str(r.get("node_id", "")) == node for r in active)]
    if not nodes:
        return
    means = {}
    for node in nodes:
        subset = [r for r in active if str(r.get("node_id", "")) == node]
        means[node] = {
            "fnm": mean([safe_float(r.get("fnm_request_mediation_ms")) for r in subset]),
            "local": mean([safe_float(r.get("si_dt_decision_apply_ms")) for r in subset]),
            "n": len(subset),
            "deferred_n": sum(1 for r in component_rows if str(r.get("node_id", "")) == node and safe_float(r.get("total_request_to_apply_ms")) > active_threshold_ms),
        }
    vmax = max(1.0, max(v["fnm"] + v["local"] for v in means.values()) * 1.22)
    width, height = paper_dim(1120, 610)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Key route nodes only: entry/high-volume node plus nodes where deferred safety-gated coordination appears.", cls="subtitle")
    left, right, top, bottom = 112, width - 50, 108, height - 144
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=6, label="Mean Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(46, group_w * 0.58)
    colors = {"fnm": "#1B9E77", "local": "#2D7DD2"}
    for i, node in enumerate(nodes):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for key in ["fnm", "local"]:
            value = means[node][key]
            if not math.isfinite(value):
                value = 0.0
            h = (bottom - top) * value / vmax
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors[key], opacity=0.92, rx=2)
            y_cursor -= h
        total = means[node]["fnm"] + means[node]["local"]
        svg.text(x + bar_w / 2, y_cursor - 9, f"{total:.0f}", cls="small", size=paper_value_size(15), anchor="middle")
        dn = means[node]["deferred_n"]
        sample_label = f"n={means[node]['n']}" + (f"; d={dn}" if dn else "")
        svg.text(x + bar_w / 2, y_cursor - 27, sample_label, cls="small", size=max(10, paper_value_size(12)), fill="#596773", anchor="middle")
        svg_multiline_text(svg, x + bar_w / 2, bottom + 26, latency_node_label(node, component_rows, i + 1), cls="xtick", line_h=18, anchor="middle")
    legend_y = 82
    legend_x = (left + right) / 2 - 190
    svg.rect(legend_x, legend_y - 13, 16, 12, fill=colors["fnm"], opacity=0.92)
    svg.text(legend_x + 23, legend_y - 2, "FNM interoperability mediation", cls="legend")
    svg.rect(legend_x + 285, legend_y - 13, 16, 12, fill=colors["local"], opacity=0.92)
    svg.text(legend_x + 308, legend_y - 2, "Local SI-DT decision/apply", cls="legend")
    svg.text(left + 8, bottom + 112, "d = number of deferred/gated samples at that node", cls="small", fill="#596773")
    svg.save(path)


def plot_active_request_to_apply_boxplots_by_node(
    component_rows: list[dict],
    path: Path,
    *,
    active_threshold_ms: float = 500.0,
    y_min_ms: float = 200.0,
) -> None:
    active = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms]
    nodes = route_ordered_latency_nodes(active)
    if not nodes:
        return
    by_node = {
        node: [safe_float(r.get("total_request_to_apply_ms")) for r in active if str(r.get("node_id", "")) == node and math.isfinite(safe_float(r.get("total_request_to_apply_ms")))]
        for node in nodes
    }
    nodes = [n for n in nodes if by_node.get(n)]
    if not nodes:
        return
    all_vals = [v for vals in by_node.values() for v in vals]
    vmin = min(y_min_ms, min(all_vals) * 0.80)
    vmax = max(vmin + 1.0, active_threshold_ms, max(all_vals) * 1.10)
    width, height = paper_dim(1320, 600)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Route-ordered active request-to-apply latency distribution by node; delayed/gated samples are excluded.", cls="subtitle")
    left, right, top, bottom = 112, width - 50, 100, height - 132
    draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    box_w = min(58, group_w * 0.45)
    rng = np.random.default_rng(65)
    for i, node in enumerate(nodes):
        vals = by_node[node]
        x = left + group_w * (i + 0.5)
        vals = [max(vmin, min(v, vmax)) for v in vals]
        draw_svg_boxplot(svg, x, vals, ymin=vmin, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color="#9467BD", rng=rng)
        svg_multiline_text(svg, x, bottom + 24, latency_node_label(node, active, i + 1), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 78, f"n={len(vals)}", cls="small", anchor="middle")
    svg.save(path)


def plot_deferred_guard_latency_by_node(
    component_rows: list[dict],
    path: Path,
    *,
    active_threshold_ms: float = 500.0,
    y_min_ms: float = 200.0,
) -> None:
    delayed = [r for r in component_rows if safe_float(r.get("total_request_to_apply_ms")) > active_threshold_ms]
    nodes = route_ordered_latency_nodes(delayed)
    if not nodes:
        return
    by_node = {
        node: [safe_float(r.get("total_request_to_apply_ms")) for r in delayed if str(r.get("node_id", "")) == node and math.isfinite(safe_float(r.get("total_request_to_apply_ms")))]
        for node in nodes
    }
    nodes = [n for n in nodes if by_node.get(n)]
    if not nodes:
        return
    all_vals = [v for vals in by_node.values() for v in vals]
    vmin = min(y_min_ms, min(all_vals) * 0.80)
    vmax = max(vmin + 1.0, float(np.percentile(all_vals, 98)) * 1.15)
    width, height = paper_dim(1080, 590)
    svg = Svg(width, height, "")
    svg.text(36, 54, "Where delayed/gated request-to-apply samples occur. These are safety/control-loop waits, not steady-state middleware latency.", cls="subtitle")
    left, right, top, bottom = 112, width - 50, 100, height - 128
    draw_y_axis(svg, left, top, bottom, vmin, vmax, ticks=6, label="Latency (ms)", label_x=paper_y_label_x(24), grid_right=right)
    group_w = (right - left) / max(1, len(nodes))
    box_w = min(70, group_w * 0.50)
    rng = np.random.default_rng(66)
    for i, node in enumerate(nodes):
        vals_all = by_node[node]
        vals = [max(vmin, min(v, vmax)) for v in vals_all]
        x = left + group_w * (i + 0.5)
        draw_svg_boxplot(svg, x, vals, ymin=vmin, ymax=vmax, top=top, bottom=bottom, box_w=box_w, color="#C44E52", rng=rng)
        over = sum(v > vmax for v in vals_all)
        if over:
            svg.parts.append(
                f'<polygon points="{x:.2f},{top - 3:.2f} {x - 7:.2f},{top + 11:.2f} {x + 7:.2f},{top + 11:.2f}" fill="#C44E52" opacity="0.90"/>'
            )
            svg.text(x, top + 29, f"+{over}", cls="small", size=paper_value_size(14), fill="#C44E52", anchor="middle")
        svg_multiline_text(svg, x, bottom + 24, latency_node_label(node, delayed, i + 1), cls="xtick", line_h=17, anchor="middle")
        svg.text(x, bottom + 78, f"n={len(vals_all)}", cls="small", anchor="middle")
    svg.save(path)


def write_latency_pipeline_by_node_csv(component_rows: list[dict], path: Path, *, active_threshold_ms: float = 500.0) -> None:
    nodes = route_ordered_latency_nodes(component_rows)
    rows = []
    for node in nodes:
        all_subset = [r for r in component_rows if str(r.get("node_id", "")) == node]
        active = [r for r in all_subset if safe_float(r.get("total_request_to_apply_ms")) <= active_threshold_ms]
        delayed = [r for r in all_subset if safe_float(r.get("total_request_to_apply_ms")) > active_threshold_ms]
        values_active = [safe_float(r.get("total_request_to_apply_ms")) for r in active if math.isfinite(safe_float(r.get("total_request_to_apply_ms")))]
        values_delayed = [safe_float(r.get("total_request_to_apply_ms")) for r in delayed if math.isfinite(safe_float(r.get("total_request_to_apply_ms")))]
        route_idx = min(
            [int(safe_float(r.get("route_progression_idx"))) for r in all_subset if math.isfinite(safe_float(r.get("route_progression_idx")))]
            or [9999]
        )
        rows.append({
            "route_progression_idx": route_idx,
            "node_id": node,
            "active_request_n": len(active),
            "active_mean_fnm_ms": f"{mean([safe_float(r.get('fnm_request_mediation_ms')) for r in active]):.3f}" if active else "",
            "active_mean_local_apply_ms": f"{mean([safe_float(r.get('si_dt_decision_apply_ms')) for r in active]):.3f}" if active else "",
            "active_mean_total_request_to_apply_ms": f"{mean(values_active):.3f}" if values_active else "",
            "active_p95_total_request_to_apply_ms": f"{float(np.percentile(values_active, 95)):.3f}" if values_active else "",
            "deferred_request_n": len(delayed),
            "deferred_median_total_request_to_apply_ms": f"{float(np.median(values_delayed)):.3f}" if values_delayed else "",
            "deferred_p95_total_request_to_apply_ms": f"{float(np.percentile(values_delayed, 95)):.3f}" if values_delayed else "",
        })
    write_csv(path, rows, [
        "route_progression_idx",
        "node_id",
        "active_request_n",
        "active_mean_fnm_ms",
        "active_mean_local_apply_ms",
        "active_mean_total_request_to_apply_ms",
        "active_p95_total_request_to_apply_ms",
        "deferred_request_n",
        "deferred_median_total_request_to_apply_ms",
        "deferred_p95_total_request_to_apply_ms",
    ])


def write_latency_component_process_csv(component_rows: list[dict], path: Path) -> None:
    rows = latency_component_process_rows(component_rows)
    for r in rows:
        r["latency_ms"] = f"{safe_float(r.get('latency_ms')):.3f}"
    write_csv(path, rows, ["process", "latency_ms", "node_id", "route_progression_idx", "decision_event_type"])


def plot_artifact_volume_by_gateway(messages: list[dict], path: Path) -> None:
    by_gateway_group: dict[str, Counter] = defaultdict(Counter)
    route_nodes = []
    for m in messages:
        gateway = str(m.get("_gateway", "unknown"))
        node = gateway.replace("gw-tls-node", "Node").replace("gw-ev-", "")
        if node not in route_nodes:
            route_nodes.append(node)
        by_gateway_group[node][str(m.get("_topic_group", "Other"))] += 1
    # Keep plot compact by emphasizing high-volume gateways.
    totals = {node: sum(c.values()) for node, c in by_gateway_group.items()}
    nodes = [n for n, _ in sorted(totals.items(), key=lambda kv: kv[1], reverse=True)[:18]]
    groups = [g for g, _ in Counter(g for c in by_gateway_group.values() for g in c).most_common()]
    if not nodes or not groups:
        return
    colors = {
        "Membership": "#6C757D",
        "Catalogue": "#8C6D31",
        "Discovery": "#9467BD",
        "Intersection state": "#2D7DD2",
        "Requests": "#E07A32",
        "Decisions": "#C44E52",
        "Other": "#1B9E77",
    }
    vmax = max(totals[n] for n in nodes)
    svg = Svg(1380, 720, "Artifact exchange volume by node gateway")
    svg.text(36, 56, "Top FNM gateways by raw message count; stacked by federation topic family.", cls="subtitle")
    left, right, top, bottom = 92, 1320, 120, 540
    draw_y_axis(svg, left, top, bottom, 0, max(1, vmax * 1.15), ticks=6, label="Messages (count)")
    group_w = (right - left) / max(1, len(nodes))
    bar_w = min(44, group_w * 0.62)
    for i, node in enumerate(nodes):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for group in groups:
            value = by_gateway_group[node][group]
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors.get(group, "#999"), opacity=0.76)
            y_cursor -= h
        svg.text(x + bar_w / 2, bottom + 20, node, cls="tick", anchor="middle", rotate=35)
    lx, ly = left, 92
    for i, group in enumerate(groups[:6]):
        x = lx + i * 165
        svg.rect(x, ly - 12, 16, 12, fill=colors.get(group, "#999"), opacity=0.76)
        svg.text(x + 22, ly - 2, group, cls="legend")
    svg.save(path)


def representative_route_rows(rows: list[dict]) -> list[dict]:
    return [
        r for r in rows
        if r["scenario_key"] == REPRESENTATIVE["scenario_key"]
        and r["route_id_int"] == int(REPRESENTATIVE["route_id"])
        and r["mode"] in MODES
    ]


def plot_route_outcomes_by_mode(rows: list[dict], path: Path) -> None:
    route_rows = representative_route_rows(rows)
    by_mode = {r["mode"]: r for r in route_rows}
    metrics = [
        ("travel_time_float", "Travel Time (s)"),
        ("waiting_time_float", "Waiting Time (s)"),
        ("time_loss_float", "Time Loss (s)"),
        ("waiting_count_float", "Stops (count)"),
    ]
    svg = Svg(1280, 780, "Route outcome by mode")
    svg.text(36, 56, f"Same scenario/route comparison: {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']}.", cls="subtitle")
    panel_specs = [(84, 610, 100, 330), (704, 1230, 100, 330), (84, 610, 440, 670), (704, 1230, 440, 670)]
    for (field, label), (left, right, top, bottom) in zip(metrics, panel_specs):
        values = [safe_float(by_mode.get(m, {}).get(field)) for m in MODES]
        vmax = nice_max(values, step=25 if "count" not in label.lower() else 2)
        svg.text(left, top - 18, label, cls="label")
        draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="")
        group_w = (right - left) / len(MODES)
        bar_w = min(62, group_w * 0.48)
        for i, mode in enumerate(MODES):
            value = safe_float(by_mode.get(mode, {}).get(field))
            x = left + group_w * (i + 0.5) - bar_w / 2
            if math.isfinite(value):
                y = y_scale(value, 0, vmax, top, bottom)
                svg.rect(x, y, bar_w, bottom - y, fill=MODE_COLORS[mode], opacity=0.75, rx=4)
                svg.text(x + bar_w / 2, y - 7, f"{value:.1f}", cls="small", anchor="middle")
            else:
                svg.text(x + bar_w / 2, (top + bottom) / 2, "N/A", cls="tick", anchor="middle")
            svg.text(x + bar_w / 2, bottom + 28, MODE_DISPLAY[mode], cls="xtick", anchor="middle")
    svg.save(path)


def plot_route_outcomes_grouped_metrics(rows: list[dict], path: Path) -> None:
    route_rows = representative_route_rows(rows)
    by_mode = {r["mode"]: r for r in route_rows}
    metrics = [
        ("travel_time_float", "Travel Time", 1.0),
        ("waiting_time_float", "Waiting Time", 1.0),
        ("time_loss_float", "Time Loss", 1.0),
    ]
    values = []
    for field, _label, scale in metrics:
        for mode in MODES:
            values.append(safe_float(by_mode.get(mode, {}).get(field)) * scale)
    vmax = nice_max([v for v in values if math.isfinite(v)], step=50)
    width, height = paper_dim(720, 420)
    if CURRENT_PAPER_SLOT == "3_1":
        width += 70
        height = max(height, PAIRED_3_1_MIN_HEIGHT)
    svg = Svg(width, height, "")
    left, right, top, bottom = 92, width - 40, 96, height - 72
    draw_y_axis(svg, left, top, bottom, 0, vmax, ticks=5, label="Mean Time (s)", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / len(metrics)
    bar_w = min(34, group_w / (len(MODES) + 1.2))
    offsets = [(i - (len(MODES) - 1) / 2) * bar_w * 1.18 for i in range(len(MODES))]
    for mi, (field, label, scale) in enumerate(metrics):
        center = left + group_w * (mi + 0.5)
        if mi > 0:
            sep_x = left + group_w * mi
            svg.line(sep_x, top + 4, sep_x, bottom - 2, stroke="#9aa7b2", sw=1, opacity=0.72, dash="5,5")
        for oi, mode in enumerate(MODES):
            value_raw = safe_float(by_mode.get(mode, {}).get(field))
            value = value_raw * scale
            x = center + offsets[oi] - bar_w / 2
            if math.isfinite(value):
                y = y_scale(value, 0, vmax, top, bottom)
                svg.rect(x, y, bar_w, bottom - y, fill=MODE_COLORS[mode], opacity=1.0, rx=3)
                svg.text(x + bar_w / 2, y - 6, f"{value_raw:.1f}", cls="small", size=paper_value_size(), anchor="middle")
        svg.text(center, bottom + 30, label, cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    draw_positioned_mode_legend(svg, 48, item_w=140, position=paper_legend_position("top-center"))
    svg.save(path)


def mode_event_counts(events: list[dict]) -> Counter:
    counts = Counter()
    for e in events:
        cat = event_progress_category(str(e.get("event_type", "") or ""))
        if cat in EVENT_CATEGORY_PAPER_ORDER:
            counts[cat] += 1
    return counts


def plot_cross_mode_runtime_events(mode_data: dict[str, dict], path: Path, *, compact: bool = False) -> None:
    counts_by_mode = {mode: mode_event_counts(data["events"]) for mode, data in mode_data.items()}
    categories = [c for c in EVENT_CATEGORY_PAPER_ORDER if any(counts_by_mode.get(m, Counter()).get(c, 0) for m in MODES)]
    if not categories:
        return
    totals = {mode: sum(counts_by_mode.get(mode, Counter()).values()) for mode in MODES}
    vmax = max(totals.values()) if totals else 1
    if compact:
        width, height = paper_dim(640, 385)
        if CURRENT_PAPER_SLOT == "3_1":
            # Match the outcomes panel's +70 px paper width so equal-width
            # LaTeX subfigures preserve identical rendered heights.
            width += 70
        width = max(width, RUNTIME_EVENTS_3_1_WIDTH)
        height = max(height, PAIRED_3_1_MIN_HEIGHT)
        left, right, top, bottom = RUNTIME_EVENTS_3_1_LEFT_MARGIN, width - 42, 96, height - 72
        legend_y = 48
        legend_cols = 3
        legend_col_w = RUNTIME_EVENTS_3_1_LEGEND_COLUMN_WIDTH
    else:
        width, height = 1180, 700
        left, right, top, bottom = 92, 1110, 142, 540
        legend_y = 94
        legend_cols = 5
        legend_col_w = 190
    svg = Svg(width, height, "" if compact else "Runtime federation activity by mode")
    svg.text(36, 56, "Stacked route-scoped event counts from fed_outcomes.events.jsonl; B0 has little/no coordination activity by design.", cls="subtitle")
    if compact:
        draw_category_legend(
            svg,
            categories,
            left,
            legend_y,
            columns=legend_cols,
            col_w=legend_col_w,
            marker="rect",
            center_rows=False,
            font_size=RUNTIME_EVENTS_3_1_LEGEND_FONT_SIZE,
        )
    else:
        draw_category_legend(svg, categories, left, legend_y, columns=legend_cols, col_w=legend_col_w)
    y_label_x = RUNTIME_EVENTS_3_1_YLABEL_X if compact else 22
    draw_y_axis(
        svg,
        left,
        top,
        bottom,
        0,
        max(1, vmax * 1.15),
        ticks=6,
        label="Federation Service|Activity (count)",
        tick_fmt=compact_k_tick if compact else None,
        label_x=y_label_x,
        grid_right=right,
    )
    group_w = (right - left) / len(MODES)
    bar_w = min(52 if compact else 80, group_w * (0.38 if compact else 0.48))
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for cat in categories:
            value = counts_by_mode.get(mode, Counter()).get(cat, 0)
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=EVENT_CATEGORY_COLORS[cat], opacity=1.0)
            y_cursor -= h
        svg.text(x + bar_w / 2, y_cursor - 8, f"{totals.get(mode, 0):,}", cls="small", size=paper_value_size(), anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    svg.save(path)


def _event_payload_bytes(event: dict) -> int:
    try:
        return len(json.dumps(event, separators=(",", ":"), sort_keys=True).encode("utf-8"))
    except Exception:
        return 0


def semantic_event_volume_by_mode(mode_data: dict[str, dict]) -> tuple[dict[str, Counter], dict[str, Counter], list[str]]:
    counts_by_mode: dict[str, Counter] = {}
    bytes_by_mode: dict[str, Counter] = {}
    order = EVENT_CATEGORY_CHOREOGRAPHY_ORDER + ["Other"]
    for mode, data in mode_data.items():
        counts = Counter()
        bytes_counter = Counter()
        for event in data.get("events", []):
            cat = event_progress_category(str(event.get("event_type", "") or ""))
            if cat not in EVENT_CATEGORY_CHOREOGRAPHY_ORDER:
                cat = "Other"
            counts[cat] += 1
            bytes_counter[cat] += _event_payload_bytes(event)
        counts_by_mode[mode] = counts
        bytes_by_mode[mode] = bytes_counter
    categories = [c for c in order if any(counts_by_mode.get(m, Counter()).get(c, 0) for m in MODES)]
    return counts_by_mode, bytes_by_mode, categories


def plot_cross_mode_runtime_artifact_count(mode_data: dict[str, dict], path: Path) -> None:
    counts_by_mode, _bytes_by_mode, categories = semantic_event_volume_by_mode(mode_data)
    if not categories:
        return
    totals = {mode: sum(counts_by_mode.get(mode, Counter()).values()) for mode in MODES}
    vmax = max(totals.values()) if totals else 1
    width, height = paper_dim(720, 390)
    svg = Svg(width, height, "")
    left, right, top, bottom = 90, width - 42, 104, height - 62
    draw_positioned_category_legend(svg, categories, 44, columns=3, col_w=190, position=paper_legend_position("top-center"))
    draw_y_axis(svg, left, top, bottom, 0, max(1, vmax * 1.15), ticks=5, label="Runtime artefacts (count)", tick_fmt=compact_number, label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / len(MODES)
    bar_w = min(58, group_w * 0.46)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for cat in categories:
            value = counts_by_mode.get(mode, Counter()).get(cat, 0)
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=EVENT_CATEGORY_COLORS.get(cat, "#6C757D"), opacity=1.0)
            y_cursor -= h
        svg.text(x + bar_w / 2, y_cursor - 7, compact_number(totals.get(mode, 0)), cls="small", size=paper_value_size(), anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    svg.save(path)


def plot_cross_mode_runtime_artifact_volume(mode_data: dict[str, dict], path: Path) -> None:
    _counts_by_mode, bytes_by_mode, categories = semantic_event_volume_by_mode(mode_data)
    if not categories:
        return
    mb_by_mode = {
        mode: Counter({cat: value / 1_000_000.0 for cat, value in counter.items()})
        for mode, counter in bytes_by_mode.items()
    }
    totals = {mode: sum(mb_by_mode.get(mode, Counter()).values()) for mode in MODES}
    vmax = max(totals.values()) if totals else 1.0
    width, height = paper_dim(720, 390)
    svg = Svg(width, height, "")
    left, right, top, bottom = 90, width - 42, 104, height - 62
    draw_positioned_category_legend(svg, categories, 44, columns=3, col_w=190, position=paper_legend_position("top-center"))
    draw_y_axis(svg, left, top, bottom, 0, max(0.01, vmax * 1.15), ticks=5, label="Runtime Artefact Volume (MB)", tick_fmt=lambda v: f"{v:.1f}", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / len(MODES)
    bar_w = min(58, group_w * 0.46)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for cat in categories:
            value = mb_by_mode.get(mode, Counter()).get(cat, 0.0)
            if value <= 0:
                continue
            h = (bottom - top) * value / max(0.01, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=EVENT_CATEGORY_COLORS.get(cat, "#6C757D"), opacity=1.0)
            y_cursor -= h
        svg.text(x + bar_w / 2, y_cursor - 7, f"{totals.get(mode, 0.0):.1f}", cls="small", size=paper_value_size(), anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    svg.save(path)


def write_runtime_artifact_volume_csv(mode_data: dict[str, dict], path: Path) -> None:
    counts_by_mode, bytes_by_mode, categories = semantic_event_volume_by_mode(mode_data)
    rows = []
    for mode in MODES:
        for cat in categories:
            rows.append({
                "mode": mode,
                "paper_mode": MODE_DISPLAY.get(mode, mode),
                "runtime_artifact_category": cat,
                "event_count": int(counts_by_mode.get(mode, Counter()).get(cat, 0)),
                "event_json_bytes": int(bytes_by_mode.get(mode, Counter()).get(cat, 0)),
                "event_json_mb": f"{bytes_by_mode.get(mode, Counter()).get(cat, 0) / 1_000_000.0:.6f}",
            })
    write_csv(path, rows, ["mode", "paper_mode", "runtime_artifact_category", "event_count", "event_json_bytes", "event_json_mb"])


def plot_cross_mode_artifact_volume(mode_data: dict[str, dict], path: Path) -> None:
    groups_all = ["Membership", "Catalogue", "Discovery", "Intersection state", "Requests", "Decisions", "Other"]
    colors = {
        "Membership": "#6C757D",
        "Catalogue": "#8C6D31",
        "Discovery": "#9467BD",
        "Intersection state": "#2D7DD2",
        "Requests": "#E07A32",
        "Decisions": "#C44E52",
        "Other": "#1B9E77",
    }
    counts: dict[str, Counter] = {}
    bytes_by_mode: dict[str, Counter] = {}
    for mode, data in mode_data.items():
        counts[mode] = Counter(m["_topic_group"] for m in data["messages"])
        c = Counter()
        for m in data["messages"]:
            c[m["_topic_group"]] += m["_payload_bytes"]
        bytes_by_mode[mode] = c
    total_count = sum(sum(c.values()) for c in counts.values())
    total_bytes = sum(sum(c.values()) for c in bytes_by_mode.values())
    groups = [
        g for g in groups_all
        if (
            sum(counts.get(m, Counter()).get(g, 0) for m in MODES) >= max(1, total_count * 0.005)
            or sum(bytes_by_mode.get(m, Counter()).get(g, 0) for m in MODES) >= max(1, total_bytes * 0.005)
        )
    ]
    if not groups:
        groups = groups_all
    bytes_mb_by_mode = {
        mode: Counter({group: value / 1_000_000.0 for group, value in counter.items()})
        for mode, counter in bytes_by_mode.items()
    }
    panels = [
        (105, 350, counts, "Message count", lambda v: f"{int(v):,}", "Messages (count)"),
        (455, 700, bytes_mb_by_mode, "Payload Volume", lambda v: f"{v:.2f} MB", "Payload Volume (MB)"),
    ]
    svg = Svg(1280, 780, "Federation artifact exchange volume by mode")
    svg.text(36, 56, f"Route-scoped raw FNM messages for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']}; stacked by service/topic family.", cls="subtitle")
    left, right = 92, 1210
    group_w = (right - left) / len(MODES)
    bar_w = min(80, group_w * 0.48)
    for top, bottom, data_dict, title, fmt, ylabel in panels:
        totals = {mode: sum(data_dict.get(mode, Counter()).values()) for mode in MODES}
        vmax = max(totals.values()) if totals else 1
        svg.text(left, top - 18, title, cls="label")
        draw_y_axis(svg, left, top, bottom, 0, max(1, vmax * 1.15), ticks=4, label=ylabel)
        for i, mode in enumerate(MODES):
            x = left + group_w * (i + 0.5) - bar_w / 2
            y_cursor = bottom
            for group in groups:
                value = data_dict.get(mode, Counter()).get(group, 0)
                if value <= 0:
                    continue
                h = (bottom - top) * value / max(1, vmax * 1.15)
                svg.rect(x, y_cursor - h, bar_w, h, fill=colors[group], opacity=0.76)
                y_cursor -= h
            svg.text(x + bar_w / 2, y_cursor - 6, fmt(totals.get(mode, 0)), cls="small", anchor="middle")
            svg.text(x + bar_w / 2, bottom + 26, MODE_DISPLAY[mode], cls="xtick", anchor="middle")
    lx, ly = left, 740
    for i, group in enumerate(groups):
        x = lx + i * 168
        svg.rect(x, ly - 12, 16, 12, fill=colors[group], opacity=0.76)
        svg.text(x + 22, ly - 2, group, cls="legend")
    svg.save(path)


def plot_cross_mode_payload_volume(mode_data: dict[str, dict], path: Path) -> None:
    groups_all = ["Membership", "Catalogue", "Discovery", "Intersection state", "Requests", "Decisions", "Other"]
    colors = {
        "Membership": "#6C757D",
        "Catalogue": "#8C6D31",
        "Discovery": "#9467BD",
        "Intersection state": "#2D7DD2",
        "Requests": "#E07A32",
        "Decisions": "#C44E52",
        "Other": "#1B9E77",
    }
    bytes_by_mode: dict[str, Counter] = {}
    for mode, data in mode_data.items():
        c = Counter()
        for m in data["messages"]:
            c[m["_topic_group"]] += m["_payload_bytes"]
        bytes_by_mode[mode] = c
    total_bytes = sum(sum(c.values()) for c in bytes_by_mode.values())
    groups = [
        g for g in groups_all
        if sum(bytes_by_mode.get(m, Counter()).get(g, 0) for m in MODES) >= max(1, total_bytes * 0.01)
    ]
    if not groups:
        groups = groups_all
    mb_by_mode = {
        mode: Counter({group: value / 1_000_000.0 for group, value in counter.items()})
        for mode, counter in bytes_by_mode.items()
    }
    totals = {mode: sum(mb_by_mode.get(mode, Counter()).values()) for mode in MODES}
    vmax = max(totals.values()) if totals else 1
    width, height = paper_dim(720, 380)
    svg = Svg(width, height, "")
    left, right, top, bottom = 90, width - 40, 96, height - 62
    draw_y_axis(svg, left, top, bottom, 0, max(0.01, vmax * 1.15), ticks=5, label="Payload Volume (MB)", tick_fmt=lambda v: f"{v:.1f}", label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / len(MODES)
    bar_w = min(70, group_w * 0.48)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for group in groups:
            value = mb_by_mode.get(mode, Counter()).get(group, 0)
            if value <= 0:
                continue
            h = (bottom - top) * value / max(0.01, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors[group], opacity=1.0)
            y_cursor -= h
        svg.text(x + bar_w / 2, y_cursor - 7, f"{totals.get(mode, 0):.2f}", cls="small", size=paper_value_size(), anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    legend_cols = min(4, max(1, len(groups)))
    legend_w = legend_cols * 150
    pos = paper_legend_position("top-center")
    lx = 24 if pos == "top-left" else max(20, svg.width - legend_w - 24) if pos == "top-right" else max(20, (svg.width - legend_w) / 2)
    ly = 54
    for i, group in enumerate(groups):
        x = lx + (i % legend_cols) * 150
        y = ly + (i // legend_cols) * 22
        svg.rect(x, y - 12, 16, 12, fill=colors[group], opacity=1.0)
        svg.text(x + 22, y - 2, group, cls="legend")
    svg.save(path)


def plot_cross_mode_message_count(mode_data: dict[str, dict], path: Path) -> None:
    groups_all = ["Membership", "Catalogue", "Discovery", "Intersection state", "Requests", "Decisions", "Other"]
    colors = {
        "Membership": "#6C757D",
        "Catalogue": "#8C6D31",
        "Discovery": "#9467BD",
        "Intersection state": "#2D7DD2",
        "Requests": "#E07A32",
        "Decisions": "#C44E52",
        "Other": "#1B9E77",
    }
    counts: dict[str, Counter] = {}
    for mode, data in mode_data.items():
        counts[mode] = Counter(m["_topic_group"] for m in data["messages"])
    total_count = sum(sum(c.values()) for c in counts.values())
    groups = [
        g for g in groups_all
        if sum(counts.get(m, Counter()).get(g, 0) for m in MODES) >= max(1, total_count * 0.01)
    ]
    if not groups:
        groups = groups_all
    totals = {mode: sum(counts.get(mode, Counter()).values()) for mode in MODES}
    vmax = max(totals.values()) if totals else 1
    width, height = paper_dim(720, 380)
    svg = Svg(width, height, "")
    left, right, top, bottom = 90, width - 40, 96, height - 62
    draw_y_axis(svg, left, top, bottom, 0, max(1, vmax * 1.15), ticks=5, label="Message count", tick_fmt=compact_number, label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / len(MODES)
    bar_w = min(58, group_w * 0.46)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        y_cursor = bottom
        for group in groups:
            value = counts.get(mode, Counter()).get(group, 0)
            if value <= 0:
                continue
            h = (bottom - top) * value / max(1, vmax * 1.15)
            svg.rect(x, y_cursor - h, bar_w, h, fill=colors[group], opacity=1.0)
            y_cursor -= h
        svg.text(x + bar_w / 2, y_cursor - 7, compact_number(totals.get(mode, 0)), cls="small", size=paper_value_size(), anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle", rotate=paper_xtick_rotation(0.0))
    legend_cols = min(4, max(1, len(groups)))
    legend_w = legend_cols * 150
    pos = paper_legend_position("top-center")
    lx = 24 if pos == "top-left" else max(20, svg.width - legend_w - 24) if pos == "top-right" else max(20, (svg.width - legend_w) / 2)
    ly = 54
    for i, group in enumerate(groups):
        x = lx + (i % legend_cols) * 150
        y = ly + (i // legend_cols) * 22
        svg.rect(x, y - 12, 16, 12, fill=colors[group], opacity=1.0)
        svg.text(x + 22, y - 2, group, cls="legend")
    svg.save(path)


def payload_latencies(messages: list[dict]) -> list[float]:
    vals = []
    for m in messages:
        payload = m.get("payload")
        if isinstance(payload, dict):
            v = safe_float(payload.get("latency_ms"))
            if math.isfinite(v):
                vals.append(v)
    return vals


def plot_cross_mode_service_latency(mode_data: dict[str, dict], path: Path) -> None:
    stats = {}
    for mode, data in mode_data.items():
        vals = payload_latencies(data["messages"])
        if vals:
            stats[mode] = (float(np.percentile(vals, 50)), float(np.percentile(vals, 95)), max(vals), len(vals))
    if not stats:
        return
    vmax = max(v[2] for v in stats.values())
    svg = Svg(980, 620, "Service latency by mode")
    svg.text(36, 56, "Route-scoped latency_ms values from service payloads; bars are p50 and markers are p95.", cls="subtitle")
    left, right, top, bottom = 90, 920, 100, 470
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label="Latency (ms)")
    group_w = (right - left) / len(MODES)
    bar_w = min(70, group_w * 0.46)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        if mode not in stats:
            svg.text(x + bar_w / 2, (top + bottom) / 2, "N/A", cls="tick", anchor="middle")
            svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle")
            continue
        p50, p95, _mx, n = stats[mode]
        y50 = y_scale(p50, 0, max(1.0, vmax * 1.15), top, bottom)
        y95 = y_scale(p95, 0, max(1.0, vmax * 1.15), top, bottom)
        svg.rect(x, y50, bar_w, bottom - y50, fill=MODE_COLORS[mode], opacity=0.72, rx=3)
        svg.line(x - 6, y95, x + bar_w + 6, y95, stroke="#263238", sw=1.8)
        svg.text(x + bar_w / 2, y95 - 8, f"{p95:.2f}", cls="small", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 52, f"n={n}", cls="small", anchor="middle")
    svg.text(left + 10, 84, "Bar = p50; black mark = p95", cls="legend")
    svg.save(path)


def plot_cross_mode_request_decision_latency(mode_data: dict[str, dict], path: Path) -> None:
    stats = {}
    for mode, data in mode_data.items():
        rows = request_to_decision_latency_rows(data["events"])
        vals = [
            safe_float(r["request_to_decision_wall_latency_ms"])
            for r in rows
            if math.isfinite(safe_float(r["request_to_decision_wall_latency_ms"]))
        ]
        if not vals:
            vals = [
                safe_float(r["request_to_decision_sim_latency_ms"])
                for r in rows
                if math.isfinite(safe_float(r["request_to_decision_sim_latency_ms"]))
            ]
        if vals:
            stats[mode] = (float(np.percentile(vals, 50)), float(np.percentile(vals, 95)), max(vals), len(vals))
    if not stats:
        return
    vmax = max(v[2] for v in stats.values())
    svg = Svg(980, 620, "Request-to-decision latency by mode")
    svg.text(36, 56, "EV request dispatch paired with next same-TLS apply; B0 has no coordination decision path.", cls="subtitle")
    left, right, top, bottom = 90, 920, 100, 470
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=6, label="Latency (ms)")
    group_w = (right - left) / len(MODES)
    bar_w = min(70, group_w * 0.46)
    for i, mode in enumerate(MODES):
        x = left + group_w * (i + 0.5) - bar_w / 2
        if mode not in stats:
            svg.text(x + bar_w / 2, (top + bottom) / 2, "N/A", cls="tick", anchor="middle")
        else:
            p50, p95, _mx, n = stats[mode]
            y50 = y_scale(p50, 0, max(1.0, vmax * 1.15), top, bottom)
            y95 = y_scale(p95, 0, max(1.0, vmax * 1.15), top, bottom)
            svg.rect(x, y50, bar_w, bottom - y50, fill=MODE_COLORS[mode], opacity=0.72, rx=3)
            svg.line(x - 6, y95, x + bar_w + 6, y95, stroke="#263238", sw=1.8)
            svg.text(x + bar_w / 2, y95 - 8, f"{p95:.0f}", cls="small", anchor="middle")
            svg.text(x + bar_w / 2, bottom + 52, f"n={n}", cls="small", anchor="middle")
        svg.text(x + bar_w / 2, bottom + 30, MODE_DISPLAY[mode], cls="xtick", anchor="middle")
    svg.text(left + 10, 84, "Bar = p50; black mark = p95", cls="legend")
    svg.save(path)


def write_cross_mode_summary(rows: list[dict], mode_data: dict[str, dict], path: Path) -> None:
    route_rows = {r["mode"]: r for r in representative_route_rows(rows)}
    out = []
    for mode in MODES:
        data = mode_data.get(mode, {})
        messages = data.get("messages", [])
        events = data.get("events", [])
        latency = payload_latencies(messages)
        payload_bytes = [m["_payload_bytes"] for m in messages if m.get("_payload_bytes", 0) > 0]
        r = route_rows.get(mode, {})
        out.append({
            "scenario": REPRESENTATIVE["scenario_label"],
            "route_id": REPRESENTATIVE["route_id"],
            "mode": mode,
            "paper_mode": MODE_DISPLAY[mode],
            "arrived": r.get("arrived", ""),
            "travel_time_s": fmt_float(safe_float(r.get("travel_time_float"))),
            "waiting_time_s": fmt_float(safe_float(r.get("waiting_time_float"))),
            "time_loss_s": fmt_float(safe_float(r.get("time_loss_float"))),
            "stops_count": fmt_float(safe_float(r.get("waiting_count_float")), decimals=0),
            "runtime_events_n": len(events),
            "raw_messages_n": len(messages),
            "raw_payload_bytes_total": int(sum(payload_bytes)),
            "raw_payload_p95_b": f"{np.percentile(payload_bytes, 95):.1f}" if payload_bytes else "",
            "service_latency_p50_ms": f"{np.percentile(latency, 50):.3f}" if latency else "",
            "service_latency_p95_ms": f"{np.percentile(latency, 95):.3f}" if latency else "",
        })
    write_csv(path, out)


def write_node_activity_csv(events: list[dict], path: Path) -> None:
    rows = []
    for e in events:
        et = str(e.get("event_type", "") or "")
        cat = event_progress_category(et)
        if cat == "Other":
            continue
        sim = safe_float(e.get("sim_time", e.get("ts_sim_s")))
        rows.append({
            "representative_scenario": REPRESENTATIVE["scenario_label"],
            "representative_route": REPRESENTATIVE["route_id"],
            "representative_mode": REPRESENTATIVE["mode"],
            "sim_time": f"{sim:.3f}" if math.isfinite(sim) else "",
            "node_id": event_node_id(e),
            "event_type": et,
            "category": cat,
            "ev_id": str(e.get("ev_id", "") or ""),
            "selected_in_edge": str(e.get("selected_in_edge", "") or ""),
            "ev_edge": str(e.get("ev_edge", "") or ""),
            "plan_type": str(e.get("plan_type", e.get("applied_plan_type", "")) or ""),
            "decision_source": str(e.get("decision_source", "") or ""),
            "reason": str(e.get("reason", "") or ""),
            "worst_edge": str(e.get("worst_edge", "") or ""),
        })
    write_csv(path, rows)

def write_middleware_summary(events: list[dict], messages: list[dict], decisions: list[dict], paths: dict[str, Path], path: Path) -> None:
    event_counts = Counter(e.get("event_type", "") for e in events)
    msg_groups = Counter(m["_topic_group"] for m in messages)
    payload_sizes = [m["_payload_bytes"] for m in messages if m["_payload_bytes"] > 0]
    latency = []
    for m in messages:
        payload = m.get("payload")
        if isinstance(payload, dict) and "latency_ms" in payload:
            v = safe_float(payload.get("latency_ms"))
            if math.isfinite(v):
                latency.append(v)
    rows = [{
        "representative_scenario": REPRESENTATIVE["scenario_label"],
        "representative_route": REPRESENTATIVE["route_id"],
        "representative_mode": REPRESENTATIVE["mode"],
        "event_jsonl": str(paths["event_jsonl"]),
        "decisions_csv": str(paths["decisions_csv"]),
        "raw_message_root": str(paths["raw_glob_root"]),
        "runtime_events_n": len(events),
        "decision_rows_n": len(decisions),
        "raw_messages_n": len(messages),
        "raw_payload_bytes_total": sum(payload_sizes),
        "raw_payload_size_mean_b": f"{mean(payload_sizes):.2f}" if payload_sizes else "",
        "raw_payload_size_p50_b": f"{np.percentile(payload_sizes, 50):.2f}" if payload_sizes else "",
        "raw_payload_size_p95_b": f"{np.percentile(payload_sizes, 95):.2f}" if payload_sizes else "",
        "service_latency_n": len(latency),
        "service_latency_mean_ms": f"{mean(latency):.3f}" if latency else "",
        "service_latency_p50_ms": f"{np.percentile(latency, 50):.3f}" if latency else "",
        "service_latency_p95_ms": f"{np.percentile(latency, 95):.3f}" if latency else "",
        "passive_context_pub_n": event_counts.get("passive_intersection.context_pub", 0),
        "ev_discovery_observed_n": event_counts.get("ev.intersection.discovery.observed", 0),
        "ev_request_dispatched_n": event_counts.get("ev.request.dispatched", 0),
        "f2_apply_n": event_counts.get("f2.apply", 0),
        "f2_strict_b1_floor_apply_n": event_counts.get("f2.strict_b1_floor.apply", 0),
        "f2p_passive_stall_rescue_apply_allow_n": event_counts.get("f2p.passive_stall_rescue.apply_allow", 0),
        "f2p_passive_nearfield_guard_release_n": event_counts.get("f2p.passive_nearfield_guard.release", 0),
        "message_group_counts": json.dumps(dict(msg_groups), sort_keys=True),
    }]
    write_csv(path, rows)


def load_core_service_counts(core_root: Path) -> list[dict]:
    rows = []
    for service in ["membership", "catalog", "discovery", "lifecycle", "metrics", "adaptive_connectivity"]:
        path = core_root / f"{service}.jsonl"
        if not path.exists():
            continue
        events = Counter()
        latency_scopes = Counter()
        latency_ms: list[float] = []
        n = 0
        first_ts = math.nan
        last_ts = math.nan
        with open(path) as f:
            for line in f:
                if not line.strip():
                    continue
                n += 1
                try:
                    o = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if n == 1:
                    first_ts = safe_float(o.get("ts", o.get("ts_wall_s")))
                last_ts = safe_float(o.get("ts", o.get("ts_wall_s")))
                events[o.get("event") or o.get("event_type") or "unknown"] += 1
                v = safe_float(o.get("latency_ms"))
                if math.isfinite(v):
                    latency_ms.append(v)
                    scope = str(o.get("latency_scope", "") or "unspecified")
                    latency_scopes[scope] += 1
        duration = max(0.001, last_ts - first_ts) if math.isfinite(first_ts) and math.isfinite(last_ts) else math.nan
        rows.append({
            "service": service,
            "jsonl_path": str(path),
            "events_n": n,
            "file_bytes": path.stat().st_size,
            "duration_wall_s": f"{duration:.3f}" if math.isfinite(duration) else "",
            "event_rate_per_s": f"{(n / duration):.3f}" if math.isfinite(duration) and duration > 0 else "",
            "latency_samples_n": len(latency_ms),
            "latency_p50_ms": f"{np.percentile(latency_ms, 50):.3f}" if latency_ms else "",
            "latency_p95_ms": f"{np.percentile(latency_ms, 95):.3f}" if latency_ms else "",
            "latency_definition": "service_loop_processing_ms",
            "latency_scopes": json.dumps(dict(latency_scopes.most_common(8))),
            "top_events": json.dumps(dict(events.most_common(8))),
        })
    return rows


def plot_core_service_activity(rows: list[dict], path: Path) -> None:
    svg = Svg(1180, 680, "Federation core service activity")
    svg.text(
        36,
        56,
        f"Scenario-level core service logs for {REPRESENTATIVE['scenario_label']} route {REPRESENTATIVE['route_id']} {REPRESENTATIVE['mode']}; activity volume shows internal middleware load.",
        cls="subtitle",
    )
    services = [r["service"].replace("_", " ") for r in rows]
    counts = [int(r["events_n"]) for r in rows]
    sizes = [int(r["file_bytes"]) for r in rows]
    panels = [(95, 510, counts, "Events", "#2D7DD2", lambda v: f"{v:,}"), (665, 1080, sizes, "Log volume", "#1B9E77", human_bytes)]
    top = 104
    for left, right, values, title, color, fmt in panels:
        vmax = max(values) if values else 1
        svg.text((left + right) / 2, 86, title, cls="label", anchor="middle")
        for i, (service, value) in enumerate(zip(services, values)):
            y = top + i * 70
            w = (right - left) * value / vmax
            svg.text(left - 12, y + 24, service, cls="tick", anchor="end")
            svg.rect(left, y + 7, w, 28, fill=color, opacity=0.75, rx=4)
            svg.text(left + w + 8, y + 27, fmt(value), cls="tick")
    svg.text(95, 620, "Note: CPU/memory counters are not present in these traces; event rate, log volume, message count, and wall runtime are used as overhead proxies.", cls="subtitle")
    svg.save(path)


def plot_core_service_load_latency(rows: list[dict], path: Path) -> None:
    if not rows:
        return
    display = {
        "membership": "Membership",
        "catalog": "Catalogue",
        "discovery": "Discovery",
        "lifecycle": "Lifecycle",
        "adaptive_connectivity": "Adaptive\nConnectivity",
        "metrics": "Metrics",
    }
    # Adaptive connectivity can dominate single-run log/event counts by orders
    # of magnitude because it snapshots graph state frequently. Keep it in the
    # CSV/core activity summary, but omit it from this compact service-load
    # figure so membership/catalogue/discovery/lifecycle remain legible.
    omitted = [r for r in rows if r["service"] == "adaptive_connectivity"]
    rows = [r for r in rows if r["service"] in display and r["service"] != "adaptive_connectivity"]
    services = [display[r["service"]] for r in rows]
    events = [int(r["events_n"]) for r in rows]
    log_mb = [int(r["file_bytes"]) / 1_000_000.0 for r in rows]
    p95_latency = [safe_float(r.get("latency_p95_ms")) for r in rows]
    svg = Svg(1280, 820, "Core federation service load and latency")
    svg.text(
        36,
        56,
        "Core service logs for the representative scenario run. Latency is local service-loop processing time, not full EV request-to-apply latency.",
        cls="subtitle",
    )
    left, right = 95, 1210
    group_w = (right - left) / max(1, len(services))
    bar_w = min(62, group_w * 0.5)
    panels = [
        (105, 300, events, "Federation Service Activity", "#2D7DD2", compact_number),
        (405, 600, log_mb, "Log Volume (MB)", "#1B9E77", lambda v: f"{v:.2f}"),
        (705, 790, p95_latency, "p95 Service-Loop Latency (ms)", "#E07A32", lambda v: f"{v:.2f}" if math.isfinite(v) else "N/A"),
    ]
    for top, bottom, vals, title, color, fmt in panels:
        finite_vals = [v for v in vals if math.isfinite(float(v))]
        vmax = max(finite_vals) if finite_vals else 1.0
        svg.text(left, top - 18, title, cls="label")
        tick_fmt = compact_number if title == "Federation Service Activity" else None
        draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.15), ticks=4, label="", tick_fmt=tick_fmt)
        for i, (service, value) in enumerate(zip(services, vals)):
            x = left + group_w * (i + 0.5) - bar_w / 2
            if math.isfinite(float(value)) and float(value) > 0:
                h = (bottom - top) * float(value) / max(1.0, vmax * 1.15)
                svg.rect(x, bottom - h, bar_w, h, fill=color, opacity=0.76, rx=3)
                svg.text(x + bar_w / 2, bottom - h - 7, fmt(value), cls="small", anchor="middle")
            else:
                svg.text(x + bar_w / 2, (top + bottom) / 2, "N/A", cls="small", anchor="middle")
            if top == panels[-1][0]:
                svg_multiline_text(svg, x + bar_w / 2, bottom + 24, service.replace("\n", "|"), cls="tick", line_h=14, anchor="middle")
    if omitted:
        ev = int(omitted[0].get("events_n", 0))
        mb = int(omitted[0].get("file_bytes", 0)) / 1_000_000.0
        svg.text(left, 812, f"Adaptive Connectivity omitted from bars for scale: {ev:,} events, {mb:.2f} MB logs in this run.", cls="subtitle")
    svg.save(path)


def core_service_display_rows(rows: list[dict], *, omit_adaptive: bool = True) -> tuple[list[dict], dict[str, str]]:
    display = {
        "membership": "Membership",
        "catalog": "Catalogue",
        "discovery": "Discovery",
        "lifecycle": "Lifecycle",
        "adaptive_connectivity": "Adaptive\nConnectivity",
        "metrics": "Metrics",
    }
    out = [r for r in rows if r["service"] in display]
    if omit_adaptive:
        out = [r for r in out if r["service"] != "adaptive_connectivity"]
    return out, display


def plot_core_service_single_metric(
    rows: list[dict],
    path: Path,
    *,
    metric: str,
    title: str,
    ylabel: str,
    color: str,
    value_fmt,
    tick_fmt=None,
    finite_only: bool = False,
) -> None:
    if not rows:
        return
    # Activity/log-volume plots omit Adaptive Connectivity for scale, but the
    # latency plot should include it because latency magnitudes are comparable.
    display_rows, display = core_service_display_rows(rows, omit_adaptive=(metric != "latency_p95_ms"))
    values = []
    for r in display_rows:
        if metric == "events_n":
            values.append(float(int(r.get("events_n") or 0)))
        elif metric == "file_mb":
            values.append(float(int(r.get("file_bytes") or 0)) / 1_000_000.0)
        else:
            values.append(safe_float(r.get(metric)))
    pairs = [(r, v) for r, v in zip(display_rows, values) if (math.isfinite(v) if finite_only else True)]
    if not pairs:
        return
    display_rows = [r for r, _v in pairs]
    values = [v for _r, v in pairs]
    services = [display[r["service"]] for r in display_rows]
    vmax = max([v for v in values if math.isfinite(v)] or [1.0])
    width, height = paper_dim(640, 330)
    if CURRENT_PAPER_SLOT == "3_1":
        width = max(width, CORE_SERVICE_3_1_WIDTH)
    svg = Svg(width, height, "")
    left, right, top, bottom = 112, width - 30, 42, height - 120
    draw_y_axis(svg, left, top, bottom, 0, max(1.0, vmax * 1.18), ticks=5, label=ylabel, tick_fmt=tick_fmt, label_x=paper_y_label_x(22), grid_right=right)
    group_w = (right - left) / max(1, len(services))
    bar_w = min(48, group_w * 0.44)
    service_tick_size = min(int(active_paper_layout().get("xtick_size", 16)), 15) if CURRENT_PAPER_SLOT == "3_1" else None
    for i, (service, value) in enumerate(zip(services, values)):
        x = left + group_w * (i + 0.5) - bar_w / 2
        if math.isfinite(value) and value > 0:
            y = y_scale(value, 0, max(1.0, vmax * 1.18), top, bottom)
            svg.rect(x, y, bar_w, bottom - y, fill=color, opacity=1.0, rx=3)
            svg.text(x + bar_w / 2, y - 8, value_fmt(value), cls="small", size=paper_value_size(16), anchor="middle")
        else:
            svg.text(x + bar_w / 2, (top + bottom) / 2, "N/A", cls="small", size=paper_value_size(16), anchor="middle")
        rotation = paper_xtick_rotation(0.0)
        if rotation:
            svg.text(x + bar_w / 2, bottom + 24, service.replace("\n", " "), cls="xtick", size=service_tick_size, anchor="middle", rotate=rotation)
        else:
            if service_tick_size:
                svg.text(x + bar_w / 2, bottom + 24, service.replace("\n", " "), cls="xtick", size=service_tick_size, anchor="middle")
            else:
                svg_multiline_text(svg, x + bar_w / 2, bottom + 24, service.replace("\n", "|"), cls="xtick", line_h=18, anchor="middle")
    svg.save(path)


def plot_service_choreography(rows: list[dict], messages: list[dict], path: Path) -> None:
    counts = {r["service"]: int(r["events_n"]) for r in rows}
    fnm_messages = len(messages)
    svg = Svg(1380, 640, "Middleware service choreography")
    svg.text(36, 56, "Service-oriented flow enabling DT registration, discovery, binding, and route-time coordination.", cls="subtitle")
    boxes = [
        ("Membership", "DT presence\nand status", counts.get("membership", 0), 80, 130),
        ("Catalogue", "Capabilities\nand schemas", counts.get("catalog", 0), 300, 130),
        ("Discovery", "Peer/service\nresolution", counts.get("discovery", 0), 520, 130),
        ("Adaptive\nConnectivity", "Dynamic peer\nbindings", counts.get("adaptive_connectivity", 0), 760, 130),
        ("FNM", "DT protocol\nmediation", fnm_messages, 1010, 130),
        ("EV-DT / SI-DTs /\nPassive SI-DTs", "Runtime artefacts:\nstate, request, decision, context", 0, 1010, 360),
        ("Lifecycle", "Health and\navailability", counts.get("lifecycle", 0), 300, 360),
        ("Metrics", "Audit and\ntelemetry", counts.get("metrics", 0), 520, 360),
    ]
    box_w, box_h = 165, 112
    for title, subtitle, count, x, y in boxes:
        svg.rect(x, y, box_w, box_h, fill="#F7FAFC", stroke="#9AA7B2", sw=1.2, rx=10, opacity=1)
        svg_multiline_text(svg, x + box_w / 2, y + 30, title, cls="label", line_h=18, anchor="middle")
        svg_multiline_text(svg, x + box_w / 2, y + 64, subtitle, cls="small", line_h=13, anchor="middle")
        if count:
            svg.text(x + box_w / 2, y + 100, f"{count:,} events/messages", cls="small", anchor="middle")
    def arrow(x1, y1, x2, y2, label):
        svg.line(x1, y1, x2, y2, stroke="#596773", sw=2)
        # small arrow head
        dx, dy = x2 - x1, y2 - y1
        norm = math.hypot(dx, dy) or 1
        ux, uy = dx / norm, dy / norm
        px, py = -uy, ux
        p1 = (x2 - ux * 12 + px * 5, y2 - uy * 12 + py * 5)
        p2 = (x2 - ux * 12 - px * 5, y2 - uy * 12 - py * 5)
        svg.parts.append(f'<polygon points="{x2:.2f},{y2:.2f} {p1[0]:.2f},{p1[1]:.2f} {p2[0]:.2f},{p2[1]:.2f}" fill="#596773"/>')
        svg.text((x1 + x2) / 2, (y1 + y2) / 2 - 7, label, cls="small", anchor="middle")
    arrow(245, 186, 300, 186, "registration")
    arrow(465, 186, 520, 186, "capability lookup")
    arrow(685, 186, 760, 186, "peer set")
    arrow(925, 186, 1010, 186, "bindings")
    arrow(1092, 242, 1092, 360, "artefacts")
    arrow(382, 360, 382, 242, "availability")
    arrow(602, 360, 602, 242, "audit")
    svg.save(path)


def main() -> None:
    args = parse_args()
    configure_from_args(args)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_results()
    selected, summary, route_table = summarize_selected(rows)
    domain_summary = summarize_domain_means(rows)
    write_csv(OUT_DIR / "selected_route_rows.csv", selected)
    write_csv(OUT_DIR / "selected_route_travel_time_summary.csv", summary)
    write_csv(OUT_DIR / "selected_route_domain_metric_means.csv", domain_summary)
    write_csv(OUT_DIR / "selected_route_mode_table.csv", route_table)
    write_csv(OUT_DIR / "selected_route_domain_metrics_consolidated.csv", consolidated_domain_metrics(rows))
    write_csv(
        OUT_DIR / "excluded_routes_notes.csv",
        [{"route_id": k, "reason": v} for k, v in sorted(EXCLUDED_ROUTES.items())],
        ["route_id", "reason"],
    )
    plot_travel_time_boxplot(rows, OUT_DIR / "selected_routes_travel_time_boxplot.svg")
    plot_domain_metric_boxplots(rows, OUT_DIR / "selected_routes_domain_metrics_boxplots.svg")
    plot_metric_mean_lines(
        domain_summary,
        "waiting_time_s",
        "Mean waiting time by scenario",
        "Mean Waiting Time (s)",
        OUT_DIR / "selected_routes_waiting_time_mean_lines.svg",
        y_step=25,
    )
    plot_metric_grouped_bars(
        domain_summary,
        "waiting_time_s",
        "Mean waiting time by scenario",
        "Mean Waiting Time (s)",
        OUT_DIR / "selected_routes_waiting_time_mean_grouped_bars.svg",
        y_step=25,
    )
    plot_dual_axis_metric(
        domain_summary,
        "waiting_time_s",
        "Mean Waiting Time (s)",
        "Mean Time Loss vs Waiting Time by Scenario",
        OUT_DIR / "selected_routes_time_loss_vs_waiting_time_dual_axis.svg",
        secondary_step=25,
    )
    # Historical filename kept for LaTeX compatibility. The plot is now a
    # single-metric grouped bar chart, not a dual-axis figure.
    plot_metric_grouped_bars(
        domain_summary,
        "time_loss_s",
        "Mean Time Loss by Scenario",
        "Mean Time Loss (s)",
        OUT_DIR / "selected_routes_time_loss_vs_stops_dual_axis.svg",
        y_step=25,
        font_sizes=SELECTED_ROUTE_FONT_SIZES,
    )
    plot_metric_grouped_bars(
        domain_summary,
        "stops_waiting_count_n",
        "Mean stops count by scenario",
        "Mean Stops (count)",
        OUT_DIR / "selected_routes_stops_mean_grouped_bars.svg",
        y_step=2,
        font_sizes=SELECTED_ROUTE_FONT_SIZES,
    )
    plot_wall_runtime_boxplot(rows, OUT_DIR / "selected_routes_wall_runtime_boxplot.svg")
    plot_mean_lines(summary, OUT_DIR / "selected_routes_mean_travel_time_lines.svg")

    representative_manifest = dict(REPRESENTATIVE)
    if not args.skip_middleware:
        paths = find_representative_files()
        events = load_events(paths["event_jsonl"])
        decisions = load_decisions(paths["decisions_csv"])
        messages = load_raw_messages(paths["raw_glob_root"])
        middleware_tag = f"route{REPRESENTATIVE['route_id']}_{REPRESENTATIVE['scenario_label'].replace('.', 'p').replace('K', 'k').lower()}_{REPRESENTATIVE['mode'].lower()}"
        plot_runtime_timeline(events, OUT_DIR / f"middleware_{middleware_tag}_runtime_timeline.svg")
        plot_event_composition(events, OUT_DIR / f"middleware_{middleware_tag}_event_composition.svg")
        plot_message_overhead(messages, OUT_DIR / f"middleware_{middleware_tag}_message_overhead.svg")
        plot_payload_latency(messages, OUT_DIR / f"middleware_{middleware_tag}_payload_latency.svg")
        plot_service_payload_latency_bars(messages, OUT_DIR / f"middleware_{middleware_tag}_service_payload_latency_bars.svg")
        plot_decision_episodes(decisions, events, OUT_DIR / f"middleware_{middleware_tag}_coordination_episodes.svg")
        plot_route_progression(events, OUT_DIR / f"middleware_{middleware_tag}_route_progression_nodes.svg")
        plot_route_progression(events, OUT_DIR / f"middleware_{middleware_tag}_route_progression_nodes_compact.svg", compact=True)
        plot_node_activity_summary(events, OUT_DIR / f"middleware_{middleware_tag}_node_activity_summary.svg")
        plot_node_activity_summary(events, OUT_DIR / f"middleware_{middleware_tag}_node_activity_summary_focus.svg", exclude_highest_node=True)
        plot_request_to_decision_latency_boxplot(events, OUT_DIR / f"middleware_{middleware_tag}_request_to_decision_latency_by_node.svg")
        plot_request_to_apply_latency_boxplot_focused(events, OUT_DIR / f"middleware_{middleware_tag}_request_to_apply_latency_by_node_focused.svg")
        additional_dir = OUT_DIR / "additional"
        plot_runtime_event_burst(events, additional_dir / f"middleware_{middleware_tag}_runtime_event_burst.svg")
        plot_critical_node_timeline(events, additional_dir / f"middleware_{middleware_tag}_critical_node_timeline.svg")
        request_component_rows = request_latency_component_rows(events)
        plot_request_latency_components_by_node(events, additional_dir / f"middleware_{middleware_tag}_request_latency_components_by_node.svg")
        plot_latency_component_process_boxplots(request_component_rows, additional_dir / f"middleware_{middleware_tag}_latency_component_process_boxplots.svg")
        plot_active_pipeline_latency_boxplots(request_component_rows, additional_dir / f"middleware_{middleware_tag}_active_pipeline_latency_boxplots.svg")
        plot_active_pipeline_latency_by_node(request_component_rows, additional_dir / f"middleware_{middleware_tag}_active_pipeline_latency_by_node.svg")
        plot_active_pipeline_latency_by_key_node(request_component_rows, additional_dir / f"middleware_{middleware_tag}_active_pipeline_latency_by_key_node.svg")
        plot_active_request_to_apply_boxplots_by_node(request_component_rows, additional_dir / f"middleware_{middleware_tag}_active_request_to_apply_latency_by_node.svg")
        plot_deferred_guard_latency_boxplots(request_component_rows, additional_dir / f"middleware_{middleware_tag}_deferred_guard_latency_boxplots.svg")
        plot_deferred_guard_latency_by_node(request_component_rows, additional_dir / f"middleware_{middleware_tag}_deferred_guard_latency_by_node.svg")
        plot_latency_by_policy_boxplots(request_component_rows, additional_dir / f"middleware_{middleware_tag}_latency_by_policy_boxplots.svg")
        plot_request_to_decision_latency_summary(events, additional_dir / f"middleware_{middleware_tag}_request_to_decision_latency_summary.svg")
        plot_request_to_decision_latency_normalized(events, additional_dir / f"middleware_{middleware_tag}_request_to_decision_latency_normalized.svg")
        plot_latency_candidate_boxplots(events, messages, additional_dir / f"middleware_{middleware_tag}_latency_process_candidates.svg")
        plot_service_latency_boxplots(messages, additional_dir / f"middleware_{middleware_tag}_service_latency_boxplots.svg")
        plot_artifact_volume_by_gateway(messages, additional_dir / f"middleware_{middleware_tag}_artifact_volume_by_gateway.svg")
        plot_runtime_event_burst_area(events, additional_dir / f"middleware_{middleware_tag}_runtime_event_burst_area.svg")
        plot_runtime_event_burst_area(
            events,
            additional_dir / f"middleware_{middleware_tag}_runtime_event_burst_area_services.svg",
            label_map=EVENT_CATEGORY_ENABLEMENT_LABELS,
            title="Runtime federation activity area by enabling service",
        )
        plot_runtime_event_burst_area(
            events,
            additional_dir / f"middleware_{middleware_tag}_runtime_event_burst_area_services_compact.svg",
            label_map=EVENT_CATEGORY_ENABLEMENT_LABELS,
            title="Runtime federation activity area by enabling service",
            compact=True,
        )
        write_node_activity_csv(events, OUT_DIR / f"middleware_{middleware_tag}_node_activity_events.csv")
        write_node_activity_summary_csv(events, OUT_DIR / f"middleware_{middleware_tag}_node_activity_summary.csv")
        write_request_to_decision_latency_csv(events, OUT_DIR / f"middleware_{middleware_tag}_request_to_decision_latency_by_node.csv")
        write_csv(additional_dir / f"middleware_{middleware_tag}_request_latency_components_by_node.csv", request_component_rows)
        write_latency_component_process_csv(request_component_rows, additional_dir / f"middleware_{middleware_tag}_latency_component_process_boxplots.csv")
        write_latency_pipeline_by_node_csv(request_component_rows, additional_dir / f"middleware_{middleware_tag}_latency_pipeline_by_node.csv")
        write_latency_candidate_csv(events, messages, additional_dir / f"middleware_{middleware_tag}_latency_process_candidates.csv")
        write_middleware_summary(events, messages, decisions, paths, OUT_DIR / f"middleware_{middleware_tag}_summary.csv")
        mode_data = load_representative_mode_data()
        cross_mode_dir = OUT_DIR / "cross_mode"
        route_tag = f"route{REPRESENTATIVE['route_id']}_{REPRESENTATIVE['scenario_label'].replace('.', 'p').replace('K', 'k').lower()}"
        plot_route_outcomes_by_mode(rows, cross_mode_dir / f"{route_tag}_outcomes_by_mode.svg")
        outcomes_grouped_path = cross_mode_dir / f"{route_tag}_outcomes_grouped_metrics.svg"
        runtime_events_compact_path = cross_mode_dir / f"{route_tag}_runtime_events_by_mode_compact.svg"
        payload_volume_path = cross_mode_dir / f"{route_tag}_payload_volume_by_mode.svg"
        message_count_path = cross_mode_dir / f"{route_tag}_message_count_by_mode.svg"
        runtime_artifact_count_path = cross_mode_dir / f"{route_tag}_runtime_artifact_count_by_mode.svg"
        runtime_artifact_volume_path = cross_mode_dir / f"{route_tag}_runtime_artifact_volume_by_mode.svg"
        plot_route_outcomes_grouped_metrics(rows, outcomes_grouped_path)
        plot_cross_mode_runtime_events(mode_data, cross_mode_dir / f"{route_tag}_runtime_events_by_mode.svg")
        plot_cross_mode_runtime_events(mode_data, runtime_events_compact_path, compact=True)
        plot_cross_mode_runtime_artifact_count(mode_data, runtime_artifact_count_path)
        plot_cross_mode_runtime_artifact_volume(mode_data, runtime_artifact_volume_path)
        plot_cross_mode_artifact_volume(mode_data, cross_mode_dir / f"{route_tag}_artifact_volume_by_mode.svg")
        plot_cross_mode_payload_volume(mode_data, payload_volume_path)
        plot_cross_mode_message_count(mode_data, message_count_path)
        plot_cross_mode_service_latency(mode_data, cross_mode_dir / f"{route_tag}_service_latency_by_mode.svg")
        plot_cross_mode_request_decision_latency(mode_data, cross_mode_dir / f"{route_tag}_request_to_decision_latency_by_mode.svg")
        write_cross_mode_summary(rows, mode_data, cross_mode_dir / f"{route_tag}_cross_mode_summary.csv")
        write_runtime_artifact_volume_csv(mode_data, cross_mode_dir / f"{route_tag}_runtime_artifact_volume_by_mode.csv")
        core_rows_for_suffix = []
        if paths["core_log_root"]:
            core_rows = load_core_service_counts(paths["core_log_root"])
            core_rows_for_suffix = core_rows
            write_csv(OUT_DIR / "middleware_core_service_activity.csv", core_rows)
            plot_core_service_activity(core_rows, OUT_DIR / "middleware_core_service_activity.svg")
            plot_core_service_load_latency(core_rows, additional_dir / f"middleware_{middleware_tag}_core_service_load_latency.svg")
            core_service_activity_path = additional_dir / f"middleware_{middleware_tag}_core_service_federation_activity.svg"
            core_service_log_path = additional_dir / f"middleware_{middleware_tag}_core_service_log_volume.svg"
            core_service_latency_path = additional_dir / f"middleware_{middleware_tag}_core_service_p95_latency.svg"
            plot_core_service_single_metric(
                core_rows,
                core_service_activity_path,
                metric="events_n",
                title="Core service federation activity",
                ylabel="Federation Service|Activity (count)",
                color="#2D7DD2",
                value_fmt=compact_number,
                tick_fmt=compact_number,
            )
            plot_core_service_single_metric(
                core_rows,
                core_service_log_path,
                metric="file_mb",
                title="Core service log volume",
                ylabel="Log Volume Size (MB)",
                color="#1B9E77",
                value_fmt=lambda v: f"{v:.2f}",
                tick_fmt=lambda v: f"{v:.1f}",
            )
            plot_core_service_single_metric(
                core_rows,
                core_service_latency_path,
                metric="latency_p95_ms",
                title="Core service p95 service-loop latency",
                ylabel="Service Latency (ms)",
                color="#E07A32",
                value_fmt=lambda v: f"{v:.2f}",
                tick_fmt=lambda v: f"{v:.1f}",
                finite_only=True,
            )
            plot_service_choreography(core_rows, messages, additional_dir / f"middleware_{middleware_tag}_service_choreography.svg")
        if GENERATE_PAPER_SUFFIX_PLOTS:
            with_paper_slot("2_1", plot_route_progression, events, slot_suffix(OUT_DIR / f"middleware_{middleware_tag}_route_progression_nodes_compact.svg", "2_1"), compact=True)
            with_paper_slot(
                "2_1",
                plot_runtime_event_burst_area,
                events,
                slot_suffix(additional_dir / f"middleware_{middleware_tag}_runtime_event_burst_area_services_compact.svg", "2_1"),
                label_map=EVENT_CATEGORY_ENABLEMENT_LABELS,
                title="Runtime federation activity area by enabling service",
                compact=True,
            )
            with_paper_slot("1_1", plot_request_latency_components_by_node, events, slot_suffix(additional_dir / f"middleware_{middleware_tag}_request_latency_components_by_node.svg", "1_1"))
            with_paper_slot("1_1", plot_node_activity_summary, events, slot_suffix(OUT_DIR / f"middleware_{middleware_tag}_node_activity_summary_focus.svg", "1_1"), exclude_highest_node=True)
            with_paper_slot("3_1", plot_route_outcomes_grouped_metrics, rows, slot_suffix(outcomes_grouped_path, "3_1"))
            with_paper_slot("3_1", plot_cross_mode_runtime_events, mode_data, slot_suffix(runtime_events_compact_path, "3_1"), compact=True)
            with_paper_slot("3_1", plot_cross_mode_runtime_artifact_count, mode_data, slot_suffix(runtime_artifact_count_path, "3_1"))
            with_paper_slot("3_1", plot_cross_mode_runtime_artifact_volume, mode_data, slot_suffix(runtime_artifact_volume_path, "3_1"))
            with_paper_slot("3_1", plot_cross_mode_payload_volume, mode_data, slot_suffix(payload_volume_path, "3_1"))
            with_paper_slot("3_1", plot_cross_mode_message_count, mode_data, slot_suffix(message_count_path, "3_1"))
            if core_rows_for_suffix:
                with_paper_slot("3_1", plot_core_service_single_metric, core_rows_for_suffix, slot_suffix(core_service_activity_path, "3_1"), metric="events_n", title="Core service federation activity", ylabel="Federation Service|Activity (count)", color="#2D7DD2", value_fmt=compact_number, tick_fmt=compact_number)
                with_paper_slot("3_1", plot_core_service_single_metric, core_rows_for_suffix, slot_suffix(core_service_log_path, "3_1"), metric="file_mb", title="Core service log volume", ylabel="Log Volume Size (MB)", color="#1B9E77", value_fmt=lambda v: f"{v:.2f}", tick_fmt=lambda v: f"{v:.1f}")
                with_paper_slot("3_1", plot_core_service_single_metric, core_rows_for_suffix, slot_suffix(core_service_latency_path, "3_1"), metric="latency_p95_ms", title="Core service p95 service-loop latency", ylabel="Service Latency (ms)", color="#E07A32", value_fmt=lambda v: f"{v:.2f}", tick_fmt=lambda v: f"{v:.1f}", finite_only=True)
        representative_manifest.update({
            "event_jsonl": str(paths["event_jsonl"]),
            "decisions_csv": str(paths["decisions_csv"]),
            "raw_message_root": str(paths["raw_glob_root"]),
            "core_log_root": str(paths["core_log_root"]),
        })

    pdf_export_rows = export_svgs_to_pdf(OUT_DIR) if EXPORT_PDF else []

    manifest = {
        "output_dir": str(OUT_DIR),
        "selected_routes": SELECTED_ROUTES,
        "excluded_routes": EXCLUDED_ROUTES,
        "scenarios": [{"key": k, "label": l, "density": d, "folder": f} for k, l, d, f in SCENARIOS],
        "modes": MODES,
        "representative_zoom": representative_manifest,
        "skip_middleware": bool(args.skip_middleware),
        "export_pdf": bool(EXPORT_PDF),
        "paper_suffix_plots": bool(GENERATE_PAPER_SUFFIX_PLOTS),
        "paper_layouts": PAPER_LAYOUTS,
        "pdf_export_ok_n": sum(1 for r in pdf_export_rows if r.get("status") == "ok"),
        "pdf_export_failed_n": sum(1 for r in pdf_export_rows if r.get("status") == "failed"),
        "pdf_export_skipped_n": sum(1 for r in pdf_export_rows if r.get("status") == "skipped"),
        "outputs": sorted(p.name for p in OUT_DIR.iterdir()),
    }
    (OUT_DIR / "plot_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True))
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
