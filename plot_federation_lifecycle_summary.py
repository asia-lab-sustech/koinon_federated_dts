#!/usr/bin/env python3
"""Draw the implemented federation lifecycle subset used in the experiments.

The figure intentionally folds the transient ONBOARDING implementation detail
into the registration/readiness phase, because the paper narrative is about
federation participation and runtime availability rather than a full governance
state machine.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import xml.sax.saxutils as sx
from pathlib import Path


def esc(x: object) -> str:
    return sx.escape(str(x))


class Svg:
    def __init__(self, width: int, height: int, font: str = "Arial") -> None:
        self.width = int(width)
        self.height = int(height)
        self.font = str(font)
        self.parts: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" viewBox="0 0 {self.width} {self.height}">',
            "<defs>",
            '<marker id="arrow" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto" markerUnits="strokeWidth">',
            '<path d="M0,0 L0,6 L9,3 z" fill="#344054"/>',
            "</marker>",
            '<marker id="arrow-muted" markerWidth="10" markerHeight="10" refX="9" refY="3" orient="auto" markerUnits="strokeWidth">',
            '<path d="M0,0 L0,6 L9,3 z" fill="#98A2B3"/>',
            "</marker>",
            "</defs>",
            f'<rect x="0" y="0" width="{self.width}" height="{self.height}" fill="none"/>',
        ]

    def save(self, path: Path) -> None:
        self.parts.append("</svg>")
        path.write_text("\n".join(self.parts), encoding="utf-8")

    def rect(self, x: float, y: float, w: float, h: float, *, fill: str, stroke: str = "#344054", sw: float = 1.5, rx: float = 14, dash: str = "", opacity: float = 1.0) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx:.1f}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw:.1f}"{dash_attr} opacity="{opacity:.3f}"/>'
        )

    def text(self, x: float, y: float, txt: str, *, size: int = 18, weight: str = "400", fill: str = "#101828", anchor: str = "middle") -> None:
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" font-family="{esc(self.font)}" '
            f'font-size="{size}" font-weight="{weight}" fill="{fill}">{esc(txt)}</text>'
        )

    def multiline(self, x: float, y: float, lines: list[str], *, size: int = 16, weight: str = "400", fill: str = "#101828", anchor: str = "middle", line_h: int | None = None) -> None:
        lh = int(line_h or round(size * 1.25))
        for i, line in enumerate(lines):
            self.text(x, y + i * lh, line, size=size, weight=weight, fill=fill, anchor=anchor)

    def line(self, x1: float, y1: float, x2: float, y2: float, *, color: str = "#344054", sw: float = 2.0, dash: str = "", arrow: bool = True) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        marker = "url(#arrow-muted)" if color == "#98A2B3" else "url(#arrow)"
        marker_attr = f' marker-end="{marker}"' if arrow else ""
        self.parts.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{color}" stroke-width="{sw:.1f}" fill="none"{dash_attr}{marker_attr}/>'
        )

    def path(self, d: str, *, color: str = "#344054", sw: float = 2.0, dash: str = "", arrow: bool = True) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        marker = "url(#arrow-muted)" if color == "#98A2B3" else "url(#arrow)"
        marker_attr = f' marker-end="{marker}"' if arrow else ""
        self.parts.append(
            f'<path d="{esc(d)}" stroke="{color}" stroke-width="{sw:.1f}" fill="none"{dash_attr}{marker_attr}/>'
        )


def box(svg: Svg, x: float, y: float, w: float, h: float, title: str, body: list[str], *, fill: str, stroke: str = "#344054", dash: str = "", title_size: int = 19, body_size: int = 14) -> None:
    svg.rect(x, y, w, h, fill=fill, stroke=stroke, dash=dash)
    svg.text(x + w / 2, y + 30, title, size=title_size, weight="700")
    if body:
        svg.multiline(x + w / 2, y + 56, body, size=body_size, fill="#344054", line_h=body_size + 5)


def label(svg: Svg, x: float, y: float, txt: str, *, size: int = 13, fill: str = "#475467") -> None:
    svg.rect(x - 8, y - 17, len(txt) * size * 0.32 + 16, 24, fill="#FFFFFF", stroke="#FFFFFF", sw=0, rx=4)
    svg.text(x, y, txt, size=size, fill=fill)


def draw(args: argparse.Namespace) -> Path:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_svg = out_dir / f"{args.prefix}.svg"
    svg = Svg(args.width, args.height, font=args.font_family)

    title_size = args.title_font_size
    subtitle_size = args.subtitle_font_size
    box_title = args.box_title_font_size
    box_body = args.box_body_font_size

    svg.rect(0, 0, args.width, args.height, fill="#FFFFFF", stroke="#FFFFFF", sw=0, rx=0)
    svg.text(args.width / 2, 44, "Implemented Federation Lifecycle Subset", size=title_size, weight="800")
    svg.text(
        args.width / 2,
        74,
        "Current prototype: lifecycle-aware registration, availability, discoverability, and runtime traceability",
        size=subtitle_size,
        fill="#475467",
    )

    # Panel boundaries.
    svg.rect(38, 104, args.width - 76, 346, fill="#F8FAFC", stroke="#D0D5DD", sw=1.2, rx=18)
    svg.rect(38, 490, args.width - 76, 276, fill="#FCFCFD", stroke="#D0D5DD", sw=1.2, rx=18)
    svg.text(70, 136, "Federation membership and availability", size=20, weight="700", anchor="start")
    svg.text(70, 522, "Runtime operation for active DTs", size=20, weight="700", anchor="start")

    # Membership lifecycle boxes.
    future_fill = "#F2F4F7"
    reg_fill = "#E0F2FE"
    active_fill = "#DCFCE7"
    suspended_fill = "#FEF3C7"
    revoked_fill = "#FEE4E2"

    box(svg, 78, 186, 170, 94, "Candidate / Verify", ["design model", "future governance"], fill=future_fill, stroke="#98A2B3", dash="6 5", title_size=box_title, body_size=box_body)
    box(svg, 326, 174, 220, 118, "Registered", ["join accepted", "catalog + heartbeat", "readiness evidence"], fill=reg_fill, title_size=box_title, body_size=box_body)
    box(svg, 634, 174, 220, 118, "Active", ["healthy member", "discoverable", "coordination eligible"], fill=active_fill, title_size=box_title, body_size=box_body)
    box(svg, 942, 174, 220, 118, "Suspended", ["heartbeat stale", "link unavailable", "not discoverable"], fill=suspended_fill, title_size=box_title, body_size=box_body)
    box(svg, 634, 348, 220, 72, "Revoked / Retired / Deleted", ["future policy, security, retention"], fill=revoked_fill, stroke="#98A2B3", dash="6 5", title_size=box_title - 1, body_size=box_body)

    svg.line(248, 233, 326, 233, color="#98A2B3", dash="6 5")
    label(svg, 287, 218, "verification")
    svg.line(546, 233, 634, 233)
    label(svg, 590, 218, "health + catalogue")
    svg.line(854, 233, 942, 233)
    label(svg, 898, 218, "timeout")
    svg.path("M 942 270 C 845 330, 720 330, 634 270")
    label(svg, 786, 326, "recovery")
    svg.line(744, 292, 744, 348, color="#98A2B3", dash="6 5")
    label(svg, 800, 316, "future governance")

    svg.text(
        326,
        436,
        "Internal ONBOARDING is folded into registration/readiness evidence, not shown as a stable paper-level state.",
        size=12,
        fill="#667085",
        anchor="start",
    )

    # Runtime lifecycle boxes.
    box(svg, 100, 568, 190, 88, "Ready / Idle", ["active but waiting", "drone or SI-DT"], fill=active_fill, title_size=box_title, body_size=box_body)
    box(svg, 384, 568, 190, 88, "Request Handling", ["EV priority request", "or drone scouting request"], fill="#EDE9FE", title_size=box_title, body_size=box_body)
    box(svg, 668, 568, 190, 88, "Executing", ["local decision", "scouting / inspection"], fill="#FFF7ED", title_size=box_title, body_size=box_body)
    box(svg, 952, 568, 190, 88, "Data Exchange", ["state, event,", "context artefacts"], fill="#ECFDF3", title_size=box_title, body_size=box_body)
    box(svg, 668, 684, 190, 58, "Out of Sync", ["stale / disconnected"], fill=suspended_fill, title_size=box_title, body_size=box_body)

    svg.line(290, 612, 384, 612)
    label(svg, 337, 601, "request")
    svg.line(574, 612, 668, 612)
    label(svg, 621, 601, "accepted")
    svg.line(858, 612, 952, 612)
    label(svg, 905, 601, "publish")
    svg.path("M 952 642 C 785 682, 455 682, 290 642")
    label(svg, 620, 674, "completed -> idle")
    svg.line(763, 656, 763, 684, color="#98A2B3", dash="6 5")
    label(svg, 809, 676, "failure")
    svg.path("M 668 716 C 500 744, 300 704, 195 656", color="#98A2B3", dash="6 5")
    label(svg, 420, 744, "resync / restore")

    # Service responsibility legend.
    y = args.height - 58
    svg.text(70, y - 28, "Implemented by:", size=14, weight="700", fill="#344054", anchor="start")
    legend = [
        ("Membership", "#E0F2FE", "member state + events"),
        ("Lifecycle", "#FEF3C7", "heartbeat availability"),
        ("Catalogue", "#ECFDF3", "capability publication"),
        ("Discovery", "#EDE9FE", "discoverability gating"),
        ("Adaptive Connectivity", "#F2F4F7", "peer binding traces"),
        ("Metrics", "#FFFFFF", "audit aggregation"),
    ]
    x0 = 190
    col_w = 330
    for i, (name, color, desc) in enumerate(legend):
        x = x0 + (i % 3) * col_w
        yy = y - 34 + (i // 3) * 30
        svg.rect(x, y - 16, 18, 18, fill=color, stroke="#98A2B3", sw=1, rx=4)
        # Correct the y coordinate for the two-row legend.
        svg.parts[-1] = svg.parts[-1].replace(f'y="{y - 16:.1f}"', f'y="{yy - 16:.1f}"')
        svg.text(x + 26, yy - 1, f"{name}: {desc}", size=13, fill="#475467", anchor="start")

    svg.save(out_svg)
    if args.export_pdf:
        out_pdf = out_dir / f"{args.prefix}.pdf"
        subprocess.run(["rsvg-convert", "-f", "pdf", "-o", str(out_pdf), str(out_svg)], check=True)
    return out_svg


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Plot implemented federation lifecycle subset.")
    ap.add_argument("--out-dir", default="./tmp/plots/lifecycle")
    ap.add_argument("--prefix", default="implemented_federation_lifecycle_subset")
    ap.add_argument("--width", type=int, default=1240)
    ap.add_argument("--height", type=int, default=860)
    ap.add_argument("--font-family", default="Arial")
    ap.add_argument("--title-font-size", type=int, default=28)
    ap.add_argument("--subtitle-font-size", type=int, default=15)
    ap.add_argument("--box-title-font-size", type=int, default=18)
    ap.add_argument("--box-body-font-size", type=int, default=13)
    ap.add_argument("--export-pdf", action="store_true")
    return ap.parse_args()


def main() -> int:
    out = draw(parse_args())
    print(str(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
