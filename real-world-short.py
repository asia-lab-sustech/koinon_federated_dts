import argparse
import csv
import json
import math
import os
import shlex
import socket
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Sequence, Tuple
from urllib import error as url_error
from urllib import request as url_request
from urllib.parse import urlparse
import sys
import paho.mqtt.client as mqtt
import traci
import traci.constants as tc
import os, glob
import multiprocessing as mp

from shadow_rollout_workers import ShadowRolloutPool, ShadowRolloutPoolConfig
import intersection_agent as intersection_agent_module
from intersection_agent import IntersectionAgent, IntersectionAgentConfig, EvRequest
from vehicle_agent import EmergencyVehicleAgent, EmergencyVehicleProfile
from ers_agent import EmergencyResponseSystemAgent, ERSConfig

# Toy experiments with DT
#from intersection_agent_DT import IntersectionAgent_DT, IntersectionAgentConfig, EvRequest

STATIC_PROGRAM = False
inspect_node = "Node6"

MODE_EVALUATION = ["B0", "B1", "F1", "F2", "F3"]
CURRENT_EVALUATION = "F2"


def _ev_kpi_write_csv(path: str, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _ev_kpi_svg_line_plot(path: str, title: str, x_label: str, y_label: str, rows: Sequence[Tuple[float, float]]) -> None:
    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    width, height = 980, 420
    margin = {"left": 72, "right": 20, "top": 38, "bottom": 54}
    left, right, top, bottom = margin["left"], margin["right"], margin["top"], margin["bottom"]
    plot_w = width - left - right
    plot_h = height - top - bottom
    pts = [(float(x), float(y)) for (x, y) in rows if x is not None and y is not None and math.isfinite(float(x)) and math.isfinite(float(y))]
    lines: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="24" text-anchor="middle" font-size="16" font-family="Arial">{title}</text>',
        f'<line x1="{left}" y1="{top+plot_h}" x2="{left+plot_w}" y2="{top+plot_h}" stroke="#333" stroke-width="1"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}" stroke="#333" stroke-width="1"/>',
        f'<text x="{left+plot_w/2:.1f}" y="{height-10}" text-anchor="middle" font-size="12" font-family="Arial">{x_label}</text>',
        f'<text x="16" y="{top+plot_h/2:.1f}" text-anchor="middle" transform="rotate(-90 16 {top+plot_h/2:.1f})" font-size="12" font-family="Arial">{y_label}</text>',
    ]
    if not pts:
        lines.append("</svg>")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        return
    x_min = min(x for x, _ in pts)
    x_max = max(x for x, _ in pts)
    y_min = min(y for _, y in pts)
    y_max = max(y for _, y in pts)
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0
    y_pad = 0.05 * (y_max - y_min)
    y_min -= y_pad
    y_max += y_pad

    def sx(x: float) -> float:
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y: float) -> float:
        return top + plot_h - (y - y_min) / (y_max - y_min) * plot_h

    for i in range(6):
        fx = x_min + i * (x_max - x_min) / 5.0
        px = sx(fx)
        lines.append(f'<line x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top+plot_h}" stroke="#eee" stroke-width="1"/>')
        lines.append(f'<text x="{px:.1f}" y="{top+plot_h+16}" text-anchor="middle" font-size="11" font-family="Arial">{fx:.1f}</text>')
    for i in range(6):
        fy = y_min + i * (y_max - y_min) / 5.0
        py = sy(fy)
        lines.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left+plot_w}" y2="{py:.1f}" stroke="#eee" stroke-width="1"/>')
        lines.append(f'<text x="{left-8}" y="{py+4:.1f}" text-anchor="end" font-size="11" font-family="Arial">{fy:.2f}</text>')

    pts_attr = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
    lines.append(f'<polyline fill="none" stroke="#0b84f3" stroke-width="2" points="{pts_attr}"/>')
    lines.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def b1_worker_proto_eval(task: Dict[str, object]) -> Dict[str, object]:
    """Prototype snapshot-only B1 advisory (no TraCI, no actuation).

    This is intentionally limited: it demonstrates a worker-safe boundary and provides
    latency/comparison signals while main control remains authoritative.
    """
    t0 = time.perf_counter()
    try:
        tls_id = str(task.get("tls_id", ""))
        sim_time = float(task.get("sim_time", 0.0))
        eta = float(task.get("eta", sim_time))
        erl = int(task.get("erl", 1))
        clrs = int(task.get("clrs", 1))
        tul = int(task.get("tul", 1))
        window_ok = bool(task.get("window_ok", False))
        baseline_clrs_max = int(task.get("baseline_clrs_max", 2))
        baseline_tul_max = int(task.get("baseline_tul_max", 1))
        sat_to_preempt_gap_sec = float(task.get("sat_to_preempt_gap_sec", 30.0))
        weights = dict(task.get("drrs_weights", {}) or {})
        clusters = list(task.get("drrs_clusters", []) or [])
        target_phase_idx = int(task.get("target_phase_idx", 0))

        drrs = (
            float(weights.get("erl", 0.1031)) * float(erl)
            + float(weights.get("clrs", 0.6053)) * float(clrs)
            + float(weights.get("tul", 0.2915)) * float(tul)
        )
        ext = 0.0
        if clusters:
            # nearest DRRS centroid -> extension
            ext = float(min(clusters, key=lambda x: abs(float(x[0]) - drrs))[1])

        eta_gap = max(0.0, float(eta - sim_time))
        if window_ok and clrs <= baseline_clrs_max and tul <= baseline_tul_max:
            proto_plan_type = "none"
        elif eta_gap <= sat_to_preempt_gap_sec:
            proto_plan_type = "preemption_candidate"
        else:
            proto_plan_type = "saturation_reduction"
        return {
            "ok": True,
            "tls_id": tls_id,
            "sim_time": sim_time,
            "eta_gap": float(eta_gap),
            "drrs": float(drrs),
            "ext": float(ext),
            "proto_plan_type": str(proto_plan_type),
            "target_phase_idx": int(target_phase_idx),
            "wall_ms": float((time.perf_counter() - t0) * 1000.0),
        }
    except Exception as e:
        return {
            "ok": False,
            "error": f"{type(e).__name__}:{e}",
            "wall_ms": float((time.perf_counter() - t0) * 1000.0),
        }

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo-bin", default="sumonew", help="sumo_new or sumogui_new")
    ap.add_argument("--sumo-cfg", required=True)
    ap.add_argument("--net-file", required=True)
    ap.add_argument("--mqtt-host", default="localhost")
    ap.add_argument("--step-length", type=float, default=0.1)
    ap.add_argument(
        "--sumo-lateral-resolution",
        type=float,
        default=-1.0,
        help="if > 0, pass --lateral-resolution to SUMO (enables sublane model)",
    )
    ap.add_argument(
        "--sumo-extra-args",
        default="",
        help="extra raw SUMO args appended at startup (quoted string, e.g. '--lanechange.duration 1.0')",
    )
    ap.add_argument("--decision-log-csv", default="/tmp/intersection_decisions.csv",
                    help="CSV file to append intersection decision logs (apply_plan/apply_offer)")
    ap.add_argument("--disable-decision-log", dest="decision_log", action="store_false",
                    help="disable CSV decision logging from intersection agents")
    ap.set_defaults(decision_log=True)

    ap.add_argument("--vehicles", default="veh1,veh2,veh3")
    ap.add_argument("--emergency-veh", default="emergency1")
    ap.add_argument(
        "--terminate-on-ev-finish",
        action="store_true",
        default=False,
        help="end simulation loop as soon as --emergency-veh reaches destination (instead of waiting all background vehicles)",
    )
    ap.add_argument(
        "--allow-missing-emergency-veh",
        action="store_true",
        default=False,
        help="allow startup when --emergency-veh is not found in route-files from --sumo-cfg",
    )
    ap.add_argument(
        "--ev-edge-source",
        choices=["auto", "snapshot", "traci"],
        default="auto",
        help="source for current EV edge in control loop (auto prefers snapshot but falls back to TraCI roadID)",
    )
    ap.add_argument(
        "--legacy-ev-request",
        dest="legacy_ev_request",
        action="store_true",
        default=False,
        help="build EvRequest inline from TraCI (small-grid style) instead of using EmergencyVehicleAgent.build_ev_request",
    )
    ap.add_argument(
        "--ev-request-delivery",
        choices=["direct", "mqtt", "both"],
        default="direct",
        help="delivery mode for EVRequest generated by EmergencyVehicleAgent: direct=in-process, mqtt=publish+consume via broker, both=direct + MQTT publish telemetry",
    )
    ap.add_argument(
        "--ev-request-topic-prefix",
        default="federation/ev/request",
        help="MQTT prefix for EVRequest publications in mqtt/both mode; final topic is <prefix>/<tls_id>",
    )
    ap.add_argument(
        "--ev-request-source-tag",
        default="",
        help="optional run/source tag appended to EVRequest traces (e.g., no_bridge, bridge_http)",
    )
    ap.add_argument(
        "--disable-internal-ev-request-generation",
        action="store_true",
        default=False,
        help="disable local EVRequest generation from TraCI loop; use only externally injected EVRequest (e.g., HTTP bridge -> MQTT)",
    )
    ap.add_argument(
        "--ev-http-adapter-enable",
        action="store_true",
        default=False,
        help="enable HTTP->MQTT EV request adapter (poll EV state from HTTP and publish canonical EVRequest to MQTT)",
    )
    ap.add_argument(
        "--ev-http-state-url",
        default="",
        help="HTTP endpoint returning EV state JSON for adapter mode",
    )
    ap.add_argument(
        "--ev-http-poll-sec",
        type=float,
        default=1.0,
        help="polling period (seconds) for EV HTTP state adapter",
    )
    ap.add_argument(
        "--ev-http-timeout-sec",
        type=float,
        default=0.8,
        help="HTTP timeout (seconds) for EV state adapter request",
    )
    ap.add_argument(
        "--ev-http-header",
        action="append",
        default=[],
        help="optional HTTP header for EV adapter, format 'Key: Value' (repeatable)",
    )
    ap.add_argument(
        "--ev-http-publish-state-topic",
        default="rw/vehicle_agent/http/state",
        help="optional MQTT topic to mirror raw HTTP EV state snapshots",
    )
    ap.add_argument(
        "--ev-http-state-server-enable",
        action="store_true",
        default=False,
        help="enable built-in HTTP endpoint that serves current EV state from vehicle_agent snapshots",
    )
    ap.add_argument(
        "--ev-http-state-server-host",
        default="127.0.0.1",
        help="bind host for built-in EV state HTTP server",
    )
    ap.add_argument(
        "--ev-http-state-server-port",
        type=int,
        default=18083,
        help="bind port for built-in EV state HTTP server",
    )
    ap.add_argument(
        "--ev-http-state-server-verbose",
        action="store_true",
        default=False,
        help="print verbose logs for built-in EV state HTTP server requests",
    )
    ap.add_argument(
        "--ev-pipeline-log-period-sec",
        type=float,
        default=1.0,
        help="periodic log period (seconds) for EV request pipeline counters; set <=0 to disable",
    )
    ap.add_argument(
        "--ev-state-trace-period-sec",
        type=float,
        default=1.0,
        help="periodic EV state trace period (seconds) in fed log; set <=0 to disable",
    )
    ap.add_argument(
        "--tls-signal-trace-enable",
        action="store_true",
        default=False,
        help="enable intersection TLS signal-change event tracing (phase/state transitions) in events JSONL",
    )
    ap.add_argument("--ev-unit-id", default="ambulance_1")
    ap.add_argument("--ev-description", default="Emergency medical transport unit")
    ap.add_argument("--ev-agency", default="EMS")
    ap.add_argument("--erl-level", type=int, default=1)  # 1..4
    ap.add_argument("--evaluation", choices=MODE_EVALUATION, default=CURRENT_EVALUATION,
                    help="evaluation mode to start with (can also be changed at runtime via MQTT cmd/sim/setEvaluation)")
    ap.add_argument(
        "--b1-worker-prototype",
        dest="b1_worker_prototype",
        action="store_true",
        default=False,
        help="run a snapshot-only multiprocessing advisory worker in B1 (prototype; main TraCI control remains authoritative)",
    )
    ap.add_argument(
        "--b1-worker-prototype-processes",
        type=int,
        default=1,
        help="number of worker processes for the B1 prototype advisory pool",
    )
    ap.add_argument(
        "--enable-ers",
        dest="enable_ers",
        action="store_true",
        default=True,
        help="enable Emergency Response System (ERS) agent orchestration",
    )
    ap.add_argument(
        "--disable-ers",
        dest="enable_ers",
        action="store_false",
        help="disable ERS orchestration",
    )

    ap.add_argument("--tls-nodes", default="auto",
                    help="comma list of junction node IDs you want as DTs, or 'auto' to use all TLS junctions in the net-file")
    ap.add_argument(
        "--debug-tls",
        default="",
        help="optional comma list of TLS controller IDs for extra local debug hooks",
    )
    ap.add_argument(
        "--agent-subset",
        choices=["all", "ev-route"],
        default="ev-route",
        help="which intersections get instantiated as agents: all selected TLS or EV-route corridor subset",
    )
    ap.add_argument(
        "--agent-subset-neighbor-hops",
        type=int,
        default=1,
        help="expand EV-route TLS corridor by N neighbor hops when --agent-subset ev-route is used",
    )
    ap.add_argument(
        "--agent-activation-mode",
        choices=["static", "on-demand"],
        default="static",
        help="intersection-agent activation strategy: static (instantiate selected subset at startup) or on-demand (activate from corridor/federation signals)",
    )
    ap.add_argument(
        "--agent-on-demand-max-new-per-tick",
        type=int,
        default=8,
        help="maximum number of new TLS agents to instantiate per simulation tick in on-demand mode",
    )
    ap.add_argument(
        "--agent-on-demand-lookahead-hops",
        type=int,
        default=6,
        help="in on-demand mode, max future TLS hops from current/optimized route to pre-activate",
    )
    ap.add_argument(
        "--auto-induction-loops",
        dest="auto_induction_loops",
        action="store_true",
        default=True,
        help="auto-generate E1 induction loops on inbound lanes of target TLS nodes (default: enabled)",
    )
    ap.add_argument(
        "--no-auto-induction-loops",
        dest="auto_induction_loops",
        action="store_false",
        help="disable auto loop generation",
    )
    ap.add_argument(
        "--loop-distance-to-stopline",
        type=float,
        default=5.0,
        help="detector placement distance (m) from lane end / stop line",
    )
    ap.add_argument("--loop-freq", type=int, default=1, help="E1 detector sampling period (s)")
    ap.add_argument(
        "--loop-deploy-scope",
        choices=["all", "ev-route"],
        default="ev-route",
        help="where to deploy generated induction loops: all selected TLSs or only EV-route corridor subset",
    )
    ap.add_argument(
        "--loop-sense-scope",
        choices=["auto", "core", "expanded", "all", "ev-active"],
        default="auto",
        help="which TLS agents to poll for loop detections (auto=B0/B1 core, F*=expanded, ev-active=current EV frontier)",
    )
    ap.add_argument(
        "--loop-sense-period-sec",
        type=float,
        default=1.0,
        help="poll/process induction-loop detections at this sim-time period (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--loop-sense-active-hops",
        type=int,
        default=3,
        help="when --loop-sense-scope ev-active, number of future route TLS to poll from current EV position",
    )
    ap.add_argument(
        "--publish-step-period-sec",
        type=float,
        default=1.0,
        help="publish rw/step at this sim-time period (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--publish-vehicle-period-sec",
        type=float,
        default=1.0,
        help="publish vehicle/EV state topics at this sim-time period (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--ev-local-state-mqtt-enable",
        dest="ev_local_state_mqtt_enable",
        action="store_true",
        default=True,
        help="enable local EV MQTT state/profile publication (default: enabled)",
    )
    ap.add_argument(
        "--no-ev-local-state-mqtt",
        dest="ev_local_state_mqtt_enable",
        action="store_false",
        help="disable local EV MQTT state/profile publication (EV HTTP-only local interface)",
    )
    ap.add_argument(
        "--publish-tls-state-period-sec",
        type=float,
        default=1.0,
        help="publish rw/tls/<id>/state at this sim-time period (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--publish-tls-lanes-period-sec",
        type=float,
        default=-1.0,
        help="publish rw/tls/<id>/lanes at this sim-time period (0=every step, <0=disabled; default disabled for performance)",
    )
    ap.add_argument(
        "--apply-corridor-route-advice",
        dest="apply_corridor_route_advice",
        action="store_true",
        default=False,
        help="apply GTCO route recommendations to the tracked EV route via TraCI (default: disabled)",
    )
    ap.add_argument(
        "--route-apply-cooldown-sec",
        type=float,
        default=10.0,
        help="minimum time between EV route applications from GTCO advice",
    )
    ap.add_argument(
        "--route-apply-min-improvement-sec",
        type=float,
        default=8.0,
        help="minimum absolute cost improvement before applying GTCO route advice",
    )
    ap.add_argument(
        "--route-apply-min-improvement-ratio",
        type=float,
        default=0.15,
        help="minimum relative cost improvement before applying GTCO route advice",
    )
    ap.add_argument(
        "--route-apply-min-remaining-lane-m",
        type=float,
        default=12.0,
        help="minimum remaining lane distance required before applying a GTCO reroute",
    )
    ap.add_argument(
        "--route-apply-in-modes",
        default="advisory,arbitration",
        help="comma-separated GTCO coordinator modes allowed to auto-apply reroute advice",
    )
    ap.add_argument(
        "--route-apply-stuck-blocked-sec",
        type=float,
        default=12.0,
        help="if EV remains blocked near stopline this long, enable stuck-recovery reroute gating",
    )
    ap.add_argument(
        "--route-apply-stuck-min-improvement-sec",
        type=float,
        default=0.0,
        help="minimum improvement (sec) required while stuck-recovery override is active",
    )
    ap.add_argument(
        "--route-apply-stuck-min-improvement-ratio",
        type=float,
        default=0.0,
        help="minimum improvement ratio required while stuck-recovery override is active",
    )
    ap.add_argument(
        "--route-apply-stuck-speed-threshold",
        type=float,
        default=0.5,
        help="max EV speed (m/s) considered blocked near stopline for stuck-recovery",
    )
    ap.add_argument(
        "--route-apply-stuck-stopline-dist-m",
        type=float,
        default=2.0,
        help="max distance to stopline (m) considered blocked for stuck-recovery",
    )
    ap.add_argument(
        "--print-period-sec",
        type=float,
        default=1.0,
        help="print the step banner at this sim-time period (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--main-loop-sleep-sec",
        type=float,
        default=0.0,
        help="optional wall-clock sleep at end of each loop iteration (default 0 for max throughput)",
    )
    ap.add_argument(
        "--perf-log-period-sec",
        type=float,
        default=-1.0,
        help="print aggregated wall-time breakdown every N sim-seconds (0=every step, <0=disabled)",
    )
    ap.add_argument(
        "--loop-add-file",
        default="",
        help="optional path for generated loop add.xml (default: /tmp/auto_loops_<pid>.add.xml)",
    )
    ap.add_argument(
        "--paper-strict-metrics",
        dest="paper_strict_metrics",
        action="store_true",
        default=False,
        help="enable strict Zhong/Chen queue metrics mode (full entrance-lane N, fixed T_lost/YT, explicit loop-source gating)",
    )
    ap.add_argument(
        "--no-paper-strict-metrics",
        dest="paper_strict_metrics",
        action="store_false",
        help="disable strict paper metrics mode (default robust mode)",
    )
    ap.add_argument(
        "--fed-debug",
        dest="fed_debug",
        action="store_true",
        default=False,
        help="enable detailed federation debug traces (candidate ranking, req/resp, warmup)",
    )
    ap.add_argument(
        "--no-fed-debug",
        dest="fed_debug",
        action="store_false",
        help="disable federation debug traces",
    )
    ap.add_argument(
        "--fed-force-route-top1",
        dest="fed_force_route_top1",
        action="store_true",
        default=False,
        help="force route-indicated next-hop neighbor to dominate next-hop probability",
    )
    ap.add_argument(
        "--no-fed-force-route-top1",
        dest="fed_force_route_top1",
        action="store_false",
        help="disable route-priority override for next-hop probability",
    )
    ap.add_argument(
        "--fed-route-prob-floor",
        type=float,
        default=0.80,
        help="minimum probability assigned to route-indicated next hop when route-top1 override is enabled",
    )
    ap.add_argument(
        "--fed-enable-warmup",
        dest="fed_enable_warmup",
        action="store_true",
        default=True,
        help="enable reservation-driven warmup pre-actuation at downstream intersections (default: enabled)",
    )
    ap.add_argument(
        "--no-fed-enable-warmup",
        dest="fed_enable_warmup",
        action="store_false",
        help="disable reservation-driven warmup pre-actuation",
    )
    ap.add_argument(
        "--fed-warmup-hard-only",
        dest="fed_warmup_hard_only",
        action="store_true",
        default=False,
        help="when enabled, warmup only uses HARD accepted reservations",
    )
    ap.add_argument(
        "--no-fed-warmup-hard-only",
        dest="fed_warmup_hard_only",
        action="store_false",
        help="allow warmup from SOFT and HARD reservations (default)",
    )
    ap.add_argument(
        "--fed-warmup-period-sec",
        type=float,
        default=0.5,
        help="warmup polling period in seconds",
    )
    ap.add_argument(
        "--fed-warm-horizon-sec",
        type=float,
        default=25.0,
        help="maximum ETA horizon (sec) for considering reservations in warmup selection",
    )
    ap.add_argument(
        "--fed-hard-min-queue-margin-sec",
        type=float,
        default=-0.5,
        help="hard reservation acceptance gate on downstream queue-clear margin (sec)",
    )
    ap.add_argument(
        "--fed-hard-max-spillback-risk",
        type=float,
        default=0.85,
        help="hard reservation acceptance gate on downstream spillback risk [0,1]",
    )
    ap.add_argument(
        "--fed-readiness-use-improved-queue",
        dest="fed_readiness_use_improved_queue",
        action="store_true",
        default=True,
        help="use improved/paper-grounded queue metrics in downstream readiness checks (default: enabled)",
    )
    ap.add_argument(
        "--no-fed-readiness-use-improved-queue",
        dest="fed_readiness_use_improved_queue",
        action="store_false",
        help="fallback to legacy queue readiness metrics for downstream checks",
    )
    ap.add_argument(
        "--f2-ev-guard-enable",
        dest="f2_ev_guard_enable",
        action="store_true",
        default=True,
        help="enable F2 EV guardrail to fallback to local anchor when refined offer is worse for EV terms",
    )
    ap.add_argument(
        "--no-f2-ev-guard-enable",
        dest="f2_ev_guard_enable",
        action="store_false",
        help="disable F2 EV guardrail (pure federation refine decisions)",
    )
    ap.add_argument(
        "--f2-ev-guard-wait-penalty-sec",
        type=float,
        default=2.0,
        help="max extra expected_wait_sec tolerated vs local anchor before guard fallback",
    )
    ap.add_argument(
        "--f2-ev-guard-miss-penalty-sec",
        type=float,
        default=0.3,
        help="max extra expected_miss_sec tolerated vs local anchor before guard fallback",
    )
    ap.add_argument(
        "--f2-ev-guard-require-feasible",
        dest="f2_ev_guard_require_feasible",
        action="store_true",
        default=True,
        help="if enabled, fallback when refined chosen offer is infeasible but local anchor is feasible",
    )
    ap.add_argument(
        "--no-f2-ev-guard-require-feasible",
        dest="f2_ev_guard_require_feasible",
        action="store_false",
        help="allow infeasible refined offer to pass guard feasibility check",
    )
    ap.add_argument(
        "--f2-selection-policy",
        choices=["legacy_guard", "measured"],
        default="measured",
        help="F2 offer selection policy in intersection agent (legacy_guard or measured robust comparison)",
    )
    ap.add_argument(
        "--f2-block-infeasible-actuation",
        dest="f2_block_infeasible_actuation",
        action="store_true",
        default=True,
        help="prevent applying infeasible F2 selected offers and fallback to local continuity",
    )
    ap.add_argument(
        "--no-f2-block-infeasible-actuation",
        dest="f2_block_infeasible_actuation",
        action="store_false",
        help="allow applying infeasible F2 selected offers",
    )
    ap.add_argument(
        "--f2-refine-require-feedback",
        dest="f2_refine_require_feedback",
        action="store_true",
        default=True,
        help="allow federation refine only when recent downstream reservation feedback exists",
    )
    ap.add_argument(
        "--no-f2-refine-require-feedback",
        dest="f2_refine_require_feedback",
        action="store_false",
        help="do not require recent downstream feedback before federation refine",
    )
    ap.add_argument(
        "--f2-refine-feedback-max-age-sec",
        type=float,
        default=6.0,
        help="maximum feedback age for allowing federation refine",
    )
    ap.add_argument(
        "--f2-refine-require-loop-coverage",
        dest="f2_refine_require_loop_coverage",
        action="store_true",
        default=True,
        help="require minimum loop mapping coverage on EV approach before federation refine",
    )
    ap.add_argument(
        "--no-f2-refine-require-loop-coverage",
        dest="f2_refine_require_loop_coverage",
        action="store_false",
        help="disable loop-coverage gating for federation refine",
    )
    ap.add_argument(
        "--f2-refine-min-loop-coverage-ratio",
        type=float,
        default=0.5,
        help="minimum mapped-loop ratio on EV approach lanes to allow federation refine",
    )
    ap.add_argument(
        "--fed-debug-log-file",
        default="",
        help="optional path to append curated federation debug outcomes (text lines)",
    )
    ap.add_argument(
        "--fed-debug-log-reset",
        dest="fed_debug_log_reset",
        action="store_true",
        default=False,
        help="reset federation debug log file at startup when --fed-debug-log-file is set",
    )
    ap.add_argument(
        "--no-fed-debug-log-reset",
        dest="fed_debug_log_reset",
        action="store_false",
        help="do not reset federation debug log file at startup",
    )
    ap.add_argument(
        "--federation-bootstrap-profile",
        choices=["none", "baseline"],
        default="none",
        help="bootstrap preset for federation core interaction (baseline auto-enables register/heartbeat/catalog/discovery probe)",
    )
    ap.add_argument(
        "--federation-bootstrap-enable",
        dest="federation_bootstrap_enable",
        action="store_true",
        default=False,
        help="publish membership/catalog/heartbeat directly from real-world.py (without connector sidecars)",
    )
    ap.add_argument(
        "--federation-bootstrap-participants",
        choices=["hub", "dts", "hybrid"],
        default="hub",
        help="bootstrap participant model: hub=single participant, dts=EV+TLS participants, hybrid=hub plus EV+TLS",
    )
    ap.add_argument(
        "--federation-bootstrap-gateway-id",
        default="gw-realworld-main",
        help="gateway_id used by federation bootstrap publications",
    )
    ap.add_argument(
        "--federation-bootstrap-node-id",
        default="realworld-main",
        help="node_id used by federation bootstrap publications",
    )
    ap.add_argument(
        "--federation-bootstrap-role",
        default="simulation_hub",
        help="role used by federation bootstrap register/catalog payloads",
    )
    ap.add_argument(
        "--federation-bootstrap-domain",
        default="traffic",
        help="domain used by federation bootstrap register payload",
    )
    ap.add_argument(
        "--federation-bootstrap-heartbeat-sec",
        type=float,
        default=5.0,
        help="heartbeat period for federation bootstrap membership events",
    )
    ap.add_argument(
        "--federation-bootstrap-catalog-sec",
        type=float,
        default=30.0,
        help="catalog upsert period for federation bootstrap",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-probe-sec",
        type=float,
        default=0.0,
        help="optional periodic discovery query probe period (0 disables probe)",
    )
    ap.add_argument(
        "--federation-bootstrap-register-topic",
        default="federation/membership/register",
        help="membership register topic for federation bootstrap",
    )
    ap.add_argument(
        "--federation-bootstrap-heartbeat-topic",
        default="federation/membership/heartbeat",
        help="membership heartbeat topic for federation bootstrap",
    )
    ap.add_argument(
        "--federation-bootstrap-catalog-topic",
        default="federation/catalog/upsert",
        help="catalog upsert topic for federation bootstrap",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-query-topic",
        default="federation/discovery/query",
        help="discovery query topic for federation bootstrap probes",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-reply-prefix",
        default="federation/discovery/resp",
        help="discovery reply prefix for federation bootstrap probes",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-event-filter",
        default="",
        help="optional event_type filter used in bootstrap discovery probes",
    )
    ap.add_argument(
        "--ev-kpi-debug",
        dest="ev_kpi_debug",
        action="store_true",
        default=False,
        help="print EV KPI trace lines ([EV_KPI]) to stdout",
    )
    ap.add_argument(
        "--no-ev-kpi-debug",
        dest="ev_kpi_debug",
        action="store_false",
        help="disable EV KPI trace stdout printing",
    )
    ap.add_argument(
        "--ev-kpi-log-file",
        default="",
        help="optional path to append EV KPI trace lines (EV_STATE / EV_CHECKPOINT / EV_KPI_SUMMARY)",
    )
    ap.add_argument(
        "--ev-kpi-log-reset",
        dest="ev_kpi_log_reset",
        action="store_true",
        default=False,
        help="reset EV KPI log file at startup when --ev-kpi-log-file is set",
    )
    ap.add_argument(
        "--no-ev-kpi-log-reset",
        dest="ev_kpi_log_reset",
        action="store_false",
        help="do not reset EV KPI log file at startup",
    )
    ap.add_argument(
        "--ev-kpi-log-period-sec",
        type=float,
        default=0.1,
        help="period for EV KPI state logging (0=every step, >0 sim-time period, <0 disabled)",
    )
    ap.add_argument(
        "--ev-kpi-csv-file",
        default="",
        help="optional CSV file to write EV KPI time-series samples at end of run",
    )
    ap.add_argument(
        "--ev-kpi-checkpoints-csv-file",
        default="",
        help="optional CSV file to write EV checkpoint events at end of run",
    )
    ap.add_argument(
        "--ev-kpi-csv-reset",
        dest="ev_kpi_csv_reset",
        action="store_true",
        default=False,
        help="reset EV KPI CSV outputs at startup when paths are provided",
    )
    ap.add_argument(
        "--no-ev-kpi-csv-reset",
        dest="ev_kpi_csv_reset",
        action="store_false",
        help="do not reset EV KPI CSV outputs at startup",
    )
    ap.add_argument(
        "--ev-kpi-fig-dir",
        default="",
        help="optional directory to write EV KPI SVG figures (speed and distance-to-stopline)",
    )
    ap.add_argument(
        "--ev-kpi-fig-reset",
        dest="ev_kpi_fig_reset",
        action="store_true",
        default=False,
        help="remove previous generated EV KPI figures in --ev-kpi-fig-dir at startup",
    )
    ap.add_argument(
        "--no-ev-kpi-fig-reset",
        dest="ev_kpi_fig_reset",
        action="store_false",
        help="do not remove prior EV KPI figures at startup",
    )
    
    # --- Optional: shadow rollout (micro-simulation) ---
    ap.add_argument(
        "--enable-shadow",
        dest="shadow",
        action="store_true",
        default=True,
        help="enable warm, parallel shadow rollouts (default: enabled)",
    )
    ap.add_argument(
        "--disable-shadow",
        "--no-shadow",
        dest="shadow",
        action="store_false",
        help="disable shadow rollouts",
    )
    ap.add_argument(
        "--shadow-sumo-bin",
        default="sumo",
        help="headless SUMO binary for shadow workers (recommend: 'sumo', not GUI)",
    )
    ap.add_argument("--shadow-workers", type=int, default=2, help="number of warm shadow SUMO workers")
    ap.add_argument("--shadow-base-port", type=int, default=9910, help="base TraCI port for shadow workers")
    ap.add_argument(
        "--shadow-timeout",
        type=float,
        default=1.0,
        help="max wall-clock seconds to wait for shadow results before falling back",
    )
    ap.add_argument(
        "--shadow-state-dir",
        default="/tmp/shadow_states",
        help="directory where the main process writes SUMO state snapshots for shadow workers",
    )
    ap.add_argument("--shadow-w-ev", type=float, default=1.0, help="weight for EV travel-time term in rollout cost")
    ap.add_argument("--shadow-w-queue", type=float, default=0.05, help="weight for queue/disruption term in rollout cost")
    ap.add_argument("--shadow-horizon", type=float, default=3.0, help="shadow rollout horizon in seconds")
    ap.add_argument(
        "--shadow-max-horizon",
        type=float,
        default=25.0,
        help="upper bound for adaptive shadow horizon; if ETA is beyond this, rollout is skipped for that step",
    )
    ap.add_argument("--shadow-topk", type=int, default=2, help="evaluate top-K offers in shadow")
    ap.add_argument(
        "--shadow-period",
        type=float,
        default=1.0,
        help="run a shadow rollout at most once per N seconds of sim-time",
    )
    #return ap.parse_args()

    # Snapshots of atual simulation state
    ap.add_argument("--state-dir", default="tmp")
    ap.add_argument("--state-period", type=float, default=2.0)
    ap.add_argument("--state-keep", type=int, default=30)

    return ap.parse_args()

def get_current_tls_detils(tls_id: str):
    phase_state = traci.trafficlight.getPhase(tls_id)
    next_switch_time = traci.trafficlight.getNextSwitch(tls_id)
    program_duration = traci.trafficlight.getAllProgramLogics(tls_id)
    program_logics = traci.trafficlight.getAllProgramLogics(tls_id)
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
    print(f"Current program in {tls_id} phase state: {phase_state}, next switch time {next_switch_time}, program: {program_duration}, logics: {program_logics}, controlled links: {controlled_links}\n")

def discover_tls_map_from_net(net_file: str):
    """
    Returns:
      node_to_tls: junctionNodeId -> set(tlsControllerId)
      tls_to_nodes: tlsControllerId -> set(junctionNodeId)
    """
    root = ET.parse(net_file).getroot()
    tls_to_nodes = defaultdict(set)

    for c in root.findall("connection"):
        tl = c.get("tl")
        via = c.get("via", "")
        if not tl or not via.startswith(":"):
            continue
        node = via[1:].split("_")[0]  # ':Node10_2_0' -> 'Node10'
        tls_to_nodes[tl].add(node)

    node_to_tls = defaultdict(set)
    for tl, nodes in tls_to_nodes.items():
        for n in nodes:
            node_to_tls[n].add(tl)

    return node_to_tls, tls_to_nodes


def _resolve_cfg_relative_path(cfg_path: str, maybe_rel: str) -> str:
    if os.path.isabs(maybe_rel):
        return maybe_rel
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(cfg_path)), maybe_rel))


def _read_sumocfg_additional_files(sumo_cfg: str) -> List[str]:
    """Read existing additional-files from sumocfg (if present), resolved to absolute paths."""
    try:
        root = ET.parse(sumo_cfg).getroot()
    except Exception:
        return []

    node = root.find("./input/additional-files")
    if node is None:
        return []
    raw = str(node.get("value", "")).strip()
    if not raw:
        return []
    out: List[str] = []
    for part in raw.split(","):
        p = part.strip()
        if not p:
            continue
        out.append(_resolve_cfg_relative_path(sumo_cfg, p))
    return out


def _read_sumocfg_route_files(sumo_cfg: str) -> List[str]:
    """Read route-files from sumocfg (if present), resolved to absolute paths."""
    try:
        root = ET.parse(sumo_cfg).getroot()
    except Exception:
        return []

    node = root.find("./input/route-files")
    if node is None:
        return []
    raw = str(node.get("value", "")).strip()
    if not raw:
        return []
    out: List[str] = []
    for part in raw.split(","):
        p = part.strip()
        if not p:
            continue
        out.append(_resolve_cfg_relative_path(sumo_cfg, p))
    return out


def extract_vehicle_route_from_sumocfg(sumo_cfg: str, vehicle_id: str) -> Optional[List[str]]:
    """Return the planned route edges for a vehicle by scanning route-files referenced by the sumocfg."""
    route_files = _read_sumocfg_route_files(sumo_cfg)
    if not route_files:
        return None

    for route_file in route_files:
        try:
            root = ET.parse(route_file).getroot()
        except Exception as e:
            print(f"[WARN] failed parsing route file '{route_file}' while looking for {vehicle_id}: {e}")
            continue

        route_defs: Dict[str, List[str]] = {}
        for r in root.findall("route"):
            rid = str(r.get("id", "") or "").strip()
            edges = [x for x in str(r.get("edges", "") or "").split() if x]
            if rid and edges:
                route_defs[rid] = edges

        for veh in root.findall("vehicle"):
            if str(veh.get("id", "")) != str(vehicle_id):
                continue
            child_route = veh.find("route")
            if child_route is not None:
                edges = [x for x in str(child_route.get("edges", "") or "").split() if x]
                if edges:
                    return edges
            route_ref = str(veh.get("route", "") or "").strip()
            if route_ref and route_ref in route_defs:
                return list(route_defs[route_ref])
            return None
    return None


def select_tls_subset_for_ev_route(
    route_edges: List[str],
    edge_to_tls: Dict[str, str],
    tls_neighbors: Dict[str, set],
    neighbor_hops: int = 1,
) -> Tuple[List[str], List[str]]:
    """Map EV route edges to downstream TLS IDs and expand by neighbor hops.

    Returns (expanded_subset_tls, core_route_tls).
    """
    core: List[str] = []
    seen_core = set()
    for e in list(route_edges or []):
        tls_id = edge_to_tls.get(str(e))
        if not tls_id or tls_id in seen_core:
            continue
        seen_core.add(tls_id)
        core.append(str(tls_id))

    if not core:
        return [], []

    if int(neighbor_hops) <= 0:
        out = sorted(core)
        return out, list(core)

    # Undirected expansion around the route corridor (captures side intersections too).
    undirected: Dict[str, set] = defaultdict(set)
    for src, nbs in dict(tls_neighbors or {}).items():
        ssrc = str(src)
        for nb in list(nbs or []):
            snb = str(nb)
            undirected.setdefault(ssrc, set()).add(snb)
            undirected.setdefault(snb, set()).add(ssrc)

    visited = set(str(x) for x in core)
    frontier = set(visited)
    hops = max(0, int(neighbor_hops))
    for _ in range(hops):
        nxt = set()
        for cur in list(frontier):
            for nb in undirected.get(str(cur), set()):
                if nb not in visited:
                    visited.add(nb)
                    nxt.add(nb)
        if not nxt:
            break
        frontier = nxt

    return sorted(visited), list(core)


def build_auto_induction_loops_additional(
    net_file: str,
    target_nodes: List[str],
    out_add_file: str,
    distance_to_stopline_m: float = 5.0,
    freq_sec: int = 1,
) -> Tuple[str, Dict[str, List[str]], int]:
    """Generate E1 loops on inbound lanes to target nodes; return (path, lane->loop_ids, count)."""
    root = ET.parse(net_file).getroot()
    target_set = set(str(n) for n in target_nodes if str(n))

    additional = ET.Element("additional")
    lane_to_loops: Dict[str, List[str]] = defaultdict(list)
    loop_count = 0
    dist = max(0.5, float(distance_to_stopline_m))
    freq = max(1, int(freq_sec))

    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        if not edge_id or edge_id.startswith(":"):
            continue
        to_node = edge.get("to")
        if target_set and str(to_node) not in target_set:
            continue

        for lane in edge.findall("lane"):
            lane_id = lane.get("id")
            if not lane_id:
                continue
            try:
                lane_len = float(lane.get("length", "0") or 0.0)
            except Exception:
                lane_len = 0.0
            pos = max(0.1, lane_len - dist) if lane_len > 0.0 else 0.1
            loop_id = f"autoLoop__{str(lane_id).replace(':', '_').replace('/', '_')}"
            ET.SubElement(
                additional,
                "inductionLoop",
                {
                    "id": loop_id,
                    "lane": str(lane_id),
                    "pos": f"{pos:.2f}",
                    "freq": str(freq),
                    "file": "/dev/null",
                    "friendlyPos": "true",
                },
            )
            lane_to_loops[str(lane_id)].append(loop_id)
            loop_count += 1

    out_path = os.path.abspath(out_add_file)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    tree = ET.ElementTree(additional)
    tree.write(out_path, encoding="UTF-8", xml_declaration=True)
    return out_path, dict(lane_to_loops), int(loop_count)


def edge_to_approach_node_from_net(edge_id: str, edge_to_to_node: dict) -> str | None:
    """Return the junction node that this edge feeds into, using the net.xml (authoritative)."""
    if not edge_id or edge_id.startswith(":"):
        return None
    return edge_to_to_node.get(edge_id)


def dist_to_stopline_on_current_lane(ev_id: str) -> float:
    """Distance from EV to the end of its current lane (≈ stop line for incoming lanes)."""
    try:
        lane_id = traci.vehicle.getLaneID(ev_id)
        lane_pos = float(traci.vehicle.getLanePosition(ev_id))     # meters from lane start
        lane_len = float(traci.lane.getLength(lane_id))            # total lane length
        return max(0.0, lane_len - lane_pos)
    except Exception:
        return 999999.0


def dist_to_junction_m(veh_id: str, junction_id: str) -> float:
    """
    Euclidean distance from vehicle to junction position.
    """
    try:
        x, y = traci.vehicle.getPosition(veh_id)
        jx, jy = traci.junction.getPosition(junction_id)
        return float(math.hypot(x - jx, y - jy))
    except Exception:
        return 999999.0

def route_inbound_edge_for_tls(ev_id: str, tls_id: str, tls_to_nodes: dict, edge_to_to_node: dict) -> str | None:
    """
    Find the first route edge ahead whose 'to' junction is one of the nodes controlled by tls_id.
    This gives a correct in_edge_id for the specific TLS agent (important for phase mapping).
    """
    tls_nodes = set(tls_to_nodes.get(tls_id, []))
    if not tls_nodes:
        return None

    try:
        route = traci.vehicle.getRoute(ev_id)
        ridx = int(traci.vehicle.getRouteIndex(ev_id))
    except Exception:
        return None

    # search forward in route for an edge ending at a node controlled by tls_id
    for e in route[ridx:]:
        if e.startswith(":"):
            continue
        to_node = edge_to_to_node.get(e)
        if to_node in tls_nodes:
            return e
    return None


# -------------------------
# Shadow rollout (micro-sim)
# -------------------------


def _offer_to_jsonable(offer) -> Dict[str, Any]:
    d = asdict(offer)
    # action is Enum
    if hasattr(offer.action, "value"):
        d["action"] = offer.action.value
    return d

def save_sumo_state_snapshot(state_dir: str, sim_time: float, keep_last: int = 25) -> str:
    """Save a SUMO state snapshot for shadow workers and prune older snapshots."""
    os.makedirs(state_dir, exist_ok=True)
    # SUMO accepts arbitrary filename; keep it stable & unique-ish per sim time.
    fname = f"state_{sim_time:.2f}.xml"
    path = os.path.join(state_dir, fname)
    tmp = path + ".tmp"
    try:
        traci.simulation.saveState(tmp)
        os.replace(tmp, path)
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except Exception:
            pass

    # Keep a lightweight pointer for debugging
    try:
        latest = os.path.join(state_dir, "latest_state.xml")
        # best-effort symlink; fall back to copy if not supported
        if os.path.islink(latest) or os.path.exists(latest):
            try:
                os.remove(latest)
            except Exception:
                pass
        try:
            os.symlink(os.path.abspath(path), latest)
        except Exception:
            # Windows / restrictive FS: just copy (small overhead)
            import shutil
            shutil.copy2(path, latest)
    except Exception:
        pass

    # prune old snapshots
    try:
        snaps = []
        for fn in os.listdir(state_dir):
            if fn.startswith("state_") and fn.endswith(".xml"):
                fp = os.path.join(state_dir, fn)
                try:
                    snaps.append((os.path.getmtime(fp), fp))
                except Exception:
                    continue
        snaps.sort(reverse=True)
        for _, fp in snaps[keep_last:]:
            try:
                os.remove(fp)
            except Exception:
                pass
    except Exception:
        pass

    return path


def build_inbound_edge_tls_index(live_tls_ids):
    idx = {} 
    for tid in live_tls_ids:
        try:
            links = traci.trafficlight.getControlledLinks(tid)
        except Exception:
            continue
        for group in links:
            if not group:
                continue
            for fr_lane, _to_lane, _via in group:
                in_edge = fr_lane.rsplit("_", 1)[0]
                idx.setdefault(in_edge, set()).add(tid)
    return {k: sorted(v) for k, v in idx.items()}

def build_tls_neighbors_from_net(net_file: str, node_to_tls: dict, tls_to_nodes: dict):
    """
    Returns:
      tls_neighbors: Dict[tls_id, Set[tls_id]]
      edge_to_tls: Dict[edge_id, tls_id]   # edge -> downstream controlling tls (if any)
    """
    root = ET.parse(net_file).getroot()

    # edge -> downstream node
    edge_to_to_node = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if eid and not eid.startswith(":"):
            edge_to_to_node[eid] = e.get("to")

    # edge -> downstream tls
    edge_to_tls = {}
    for eid, to_node in edge_to_to_node.items():
        cands = sorted(node_to_tls.get(to_node, []))
        if cands:
            edge_to_tls[eid] = cands[0]  # keep your "single tls per node" simplification

    # tls -> neighbor tls set (1-hop via outgoing edges of its controlled links)
    tls_neighbors = {tls: set() for tls in tls_to_nodes.keys()}

    live_tls_ids = set(traci.trafficlight.getIDList())
    for tls_id in live_tls_ids:
        try:
            controlled = traci.trafficlight.getControlledLinks(tls_id)
        except Exception:
            continue

        for group in controlled:
            if not group:
                continue
            for conn in group:
                # conn tuple: (fromLane, toLane, viaLane)
                to_lane = conn[1]
                if not to_lane:
                    continue
                out_edge = to_lane.rsplit("_", 1)[0]
                nb = edge_to_tls.get(out_edge)
                if nb and nb != tls_id:
                    tls_neighbors.setdefault(tls_id, set()).add(nb)

    return tls_neighbors, edge_to_tls


class ShadowThrottle:
    """Runs shadow rollouts at most once per N seconds of simulation time."""

    def __init__(self, period_sec: float):
        self.period_sec = float(period_sec)
        self._last_sim_t = -1e18

    def should_run(self, sim_time: float) -> bool:
        if self.period_sec <= 0:
            return True
        if (sim_time - self._last_sim_t) >= self.period_sec:
            self._last_sim_t = sim_time
            return True
        return False

def main():
    global CURRENT_EVALUATION
    args = parse_args()
    bootstrap_profile = str(getattr(args, "federation_bootstrap_profile", "none") or "none").strip().lower()
    if bootstrap_profile == "baseline":
        # Baseline preset: one-flag middleware bootstrap for local experiments.
        args.federation_bootstrap_enable = True
        if float(getattr(args, "federation_bootstrap_catalog_sec", 30.0)) == 30.0:
            args.federation_bootstrap_catalog_sec = 20.0
        if float(getattr(args, "federation_bootstrap_discovery_probe_sec", 0.0)) == 0.0:
            args.federation_bootstrap_discovery_probe_sec = 10.0
        if not str(getattr(args, "federation_bootstrap_discovery_event_filter", "") or "").strip():
            # Empty filter means "discover all advertised catalog services".
            args.federation_bootstrap_discovery_event_filter = ""
        print(
            "[FED_BOOTSTRAP] profile=baseline "
            f"enable={1 if bool(args.federation_bootstrap_enable) else 0} "
            f"participants={str(getattr(args, 'federation_bootstrap_participants', 'hub'))} "
            f"heartbeat_sec={float(args.federation_bootstrap_heartbeat_sec):.1f} "
            f"catalog_sec={float(args.federation_bootstrap_catalog_sec):.1f} "
            f"probe_sec={float(args.federation_bootstrap_discovery_probe_sec):.1f} "
            f"event_filter={str(args.federation_bootstrap_discovery_event_filter) or '*'}"
        )
    CURRENT_EVALUATION = str(getattr(args, "evaluation", CURRENT_EVALUATION) or CURRENT_EVALUATION).upper()
    print(f"[EVAL] startup evaluation={CURRENT_EVALUATION}")
    # Keep IntersectionAgent bound to the same live TraCI module used here.
    try:
        intersection_agent_module.traci = traci
    except Exception:
        pass

    # Runtime fingerprint to verify which intersection_agent module is loaded.
    ia_file = str(getattr(intersection_agent_module, "__file__", "") or "")
    ia_cached = str(getattr(intersection_agent_module, "__cached__", "") or "")
    mk_req_out_warn = 0
    mk_pref_fallback = 0
    mk_nextedge_fallback = 0
    try:
        if ia_file and os.path.exists(ia_file):
            with open(ia_file, "r", encoding="utf-8") as fh:
                src = fh.read()
            mk_req_out_warn = int("evt=REQ_OUT_WARN" in src)
            mk_pref_fallback = int("preferred_next_tls is None and str(to_tls) in self.neighbor_map" in src)
            mk_nextedge_fallback = int("next_edge_id is None and ninfo is not None" in src)
    except Exception:
        pass
    print(
        "[FED_RUNTIME] "
        f"cwd={os.getcwd()} "
        f"real_world_file={__file__} "
        f"intersection_agent_file={ia_file} "
        f"intersection_agent_cached={ia_cached} "
        f"mk_req_out_warn={mk_req_out_warn} "
        f"mk_pref_fallback={mk_pref_fallback} "
        f"mk_nextedge_fallback={mk_nextedge_fallback}"
    )

    edge_to_to_node = {}
    last_ev_diag = {}

    import datetime as datetime
    now = datetime.datetime.now()
    date_string = now.strftime("%Y%m%d%H%M")

    fed_debug_log_file = str(getattr(args, "fed_debug_log_file", "") or "").strip()
    fed_event_jsonl_main_path = (
        str(fed_debug_log_file).replace(".txt", ".events.jsonl")
        if str(fed_debug_log_file).strip()
        else ""
    )
    if fed_debug_log_file and bool(getattr(args, "fed_debug_log_reset", False)):
        fed_debug_log_file = fed_debug_log_file.split('.txt')[0]
        fed_debug_log_file = str(fed_debug_log_file)+'_'+str(CURRENT_EVALUATION)+'_'+date_string+('.txt')

        parent = os.path.dirname(fed_debug_log_file) or "."
        print(f"[FED_DEBUG] preparing log file: {fed_debug_log_file} (dir={parent})")
        os.makedirs(parent, exist_ok=True)  # do not swallow exceptions
        with open(fed_debug_log_file, "w", encoding="utf-8") as f:
            f.write(f"# FED_DEBUG log for {CURRENT_EVALUATION}\n")
        if fed_event_jsonl_main_path:
            try:
                with open(fed_event_jsonl_main_path, "w", encoding="utf-8") as f:
                    f.write("")
            except Exception:
                pass

    ev_kpi_log_file = str(getattr(args, "ev_kpi_log_file", "") or "").strip()
    if ev_kpi_log_file and bool(getattr(args, "ev_kpi_log_reset", False)):
        try:
            os.makedirs(os.path.dirname(ev_kpi_log_file), exist_ok=True)
        except Exception:
            pass
        try:
            with open(ev_kpi_log_file, "w", encoding="utf-8") as f:
                f.write("# EV_KPI log\n")
        except Exception as e:
            print(f"[EV_KPI][WARN] failed to reset log file '{ev_kpi_log_file}': {e}")
    ev_kpi_csv_file = str(getattr(args, "ev_kpi_csv_file", "") or "").strip()
    ev_kpi_checkpoints_csv_file = str(getattr(args, "ev_kpi_checkpoints_csv_file", "") or "").strip()
    ev_kpi_fig_dir = str(getattr(args, "ev_kpi_fig_dir", "") or "").strip()
    if bool(getattr(args, "ev_kpi_csv_reset", False)):
        for _p in [ev_kpi_csv_file, ev_kpi_checkpoints_csv_file]:
            if not _p:
                continue
            try:
                os.makedirs(os.path.dirname(_p), exist_ok=True)
            except Exception:
                pass
            try:
                if os.path.exists(_p):
                    os.remove(_p)
            except Exception as e:
                print(f"[EV_KPI][WARN] failed to reset csv '{_p}': {e}")
    if ev_kpi_fig_dir and bool(getattr(args, "ev_kpi_fig_reset", False)):
        try:
            os.makedirs(ev_kpi_fig_dir, exist_ok=True)
            for fn in ("ev_speed.svg", "ev_dstop.svg"):
                fp = os.path.join(ev_kpi_fig_dir, fn)
                if os.path.exists(fp):
                    os.remove(fp)
        except Exception as e:
            print(f"[EV_KPI][WARN] failed to reset fig dir '{ev_kpi_fig_dir}': {e}")

    def _fed_dbg_main(msg: str) -> None:
        if not (bool(getattr(args, "fed_debug", False)) or bool(fed_debug_log_file)):
            return
        try:
            t_now = float(traci.simulation.getTime())
        except Exception:
            t_now = -1.0
        line = f"[FED_DEBUG_MAIN] t={t_now:.2f} {msg}"
        if bool(getattr(args, "fed_debug", False)):
            print(line)
        if fed_debug_log_file:
            try:
                os.makedirs(os.path.dirname(fed_debug_log_file), exist_ok=True)
            except Exception:
                pass
            try:
                with open(fed_debug_log_file, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass

    def _fed_evt_main(event_type: str, **payload: object) -> None:
        path = str(fed_event_jsonl_main_path or "").strip()
        if not path:
            return
        try:
            t_now = float(traci.simulation.getTime())
        except Exception:
            t_now = -1.0
        rec = {
            "event_type": str(event_type),
            "ts_sim_s": float(t_now),
            "ts_wall_ms": float(time.time() * 1000.0),
            "source_service": "real_world",
            "role": str(payload.get("role", "ev") or "ev"),
        }
        for k, v in payload.items():
            rec[str(k)] = v
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        except Exception:
            pass
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=True) + "\n")
        except Exception:
            pass

    def _ev_kpi_dbg(msg: str, t_override: Optional[float] = None) -> None:
        if not (bool(getattr(args, "ev_kpi_debug", False)) or bool(ev_kpi_log_file)):
            return
        if t_override is None:
            try:
                t_now = float(traci.simulation.getTime())
            except Exception:
                t_now = -1.0
        else:
            t_now = float(t_override)
        line = f"[EV_KPI] t={t_now:.2f} {msg}"
        if bool(getattr(args, "ev_kpi_debug", False)):
            print(line)
        if ev_kpi_log_file:
            try:
                os.makedirs(os.path.dirname(ev_kpi_log_file), exist_ok=True)
            except Exception:
                pass
            try:
                with open(ev_kpi_log_file, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass
            try:
                with open(fed_debug_log_file, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:
                pass

    def _periodic_due(sim_time: float, state: Dict[str, float], key: str, period_sec: float) -> bool:
        """Return True when a periodic action should run at sim_time.

        period < 0: disabled
        period = 0: every step
        period > 0: run when elapsed >= period
        """
        p = float(period_sec)
        if p < 0.0:
            return False
        if p == 0.0:
            state[str(key)] = float(sim_time)
            return True
        last = float(state.get(str(key), -1e18))
        if (float(sim_time) - last) >= p - 1e-12:
            state[str(key)] = float(sim_time)
            return True
        return False

    def _drain_b1_worker_results(sim_time: float) -> None:
        if b1_worker_pool is None or not b1_worker_pending:
            return
        done_keys: List[str] = []
        for k, item in list(b1_worker_pending.items()):
            try:
                async_res, submit_t = item
            except Exception:
                done_keys.append(str(k))
                continue
            try:
                if not async_res.ready():
                    continue
                out = async_res.get(timeout=0.0)
                print(
                    f"[B1-WORKER-PROTO] t={float(sim_time):.2f} tls={k} "
                    f"submit_t={float(submit_t):.2f} result={out}"
                )
            except Exception as e:
                print(f"[B1-WORKER-PROTO][WARN] result get failed tls={k}: {e}")
            done_keys.append(str(k))
        for k in done_keys:
            b1_worker_pending.pop(str(k), None)



    net_root = ET.parse(args.net_file).getroot()
    for e in net_root.findall("edge"):
        eid = e.get("id")
        if eid and not eid.startswith(":"):
            edge_to_to_node[eid] = e.get("to")

    target_veh = [v.strip() for v in args.vehicles.split(",") if v.strip()]
    raw_tls_nodes = str(getattr(args, "tls_nodes", "") or "").strip()
    auto_tls_nodes = (not raw_tls_nodes) or (raw_tls_nodes.lower() in {"auto", "all", "*"})
    debug_tls_ids = {x.strip() for x in str(getattr(args, "debug_tls", "")).split(",") if x.strip()}
    node_to_tls, tls_to_nodes = discover_tls_map_from_net(args.net_file)
    if auto_tls_nodes:
        target_nodes = sorted(node_to_tls.keys())
    else:
        target_nodes = [n.strip() for n in raw_tls_nodes.split(",") if n.strip()]

    # Map requested junction nodes -> tls controller IDs
    target_tls = []
    for n in target_nodes:
        tls_ids = sorted(node_to_tls.get(n, []))
        if not tls_ids:
            print(f"[WARN] junction node '{n}' has no tl-controller mapping in net-file")
        else:
            target_tls.extend(tls_ids)
    target_tls = sorted(set(target_tls))
    if auto_tls_nodes:
        print(f"Target junction nodes: auto-selected {len(target_nodes)} TLS junctions from net-file")
    else:
        print(f"Target junction nodes (manual): {target_nodes}")
    print(f"Mapped TLS controllers: {len(target_tls)}")
    for tl in target_tls[:20]:
        print(f"  {tl} controls nodes {sorted(tls_to_nodes.get(tl, []))}")
    if len(target_tls) > 20:
        print(f"  ... ({len(target_tls) - 20} more)")

    # Pre-start static TLS neighbor graph (net-only) for loop generation subseting.
    edge_to_tls_static: Dict[str, str] = {}
    for eid, to_node in edge_to_to_node.items():
        cands = sorted(node_to_tls.get(to_node, []))
        if cands:
            edge_to_tls_static[str(eid)] = str(cands[0])

    tls_neighbors_static: Dict[str, set] = {str(tl): set() for tl in tls_to_nodes.keys()}
    for c in net_root.findall("connection"):
        cur_tl = c.get("tl")
        out_edge = c.get("to")
        if not cur_tl or not out_edge or str(out_edge).startswith(":"):
            continue
        nb = edge_to_tls_static.get(str(out_edge))
        if nb and str(nb) != str(cur_tl):
            tls_neighbors_static.setdefault(str(cur_tl), set()).add(str(nb))

    loop_target_nodes = list(target_nodes)
    loop_subset_tls_for_loops: List[str] = []
    loop_core_tls_for_loops: List[str] = []
    loop_deploy_scope = str(getattr(args, "loop_deploy_scope", "ev-route") or "ev-route").strip().lower()
    ev_route_edges_cfg = extract_vehicle_route_from_sumocfg(args.sumo_cfg, str(args.emergency_veh))
    if not ev_route_edges_cfg:
        ev_missing_msg = (
            f"Selected emergency vehicle '{args.emergency_veh}' was not found in route-files from {args.sumo_cfg}. "
            "Pick an existing vehicle ID via --emergency-veh or regenerate routes."
        )
        if bool(getattr(args, "allow_missing_emergency_veh", False)):
            print(f"[ev-select][WARN] {ev_missing_msg}")
        else:
            raise RuntimeError(ev_missing_msg)
    else:
        print(
            f"[ev-select] using emergency vehicle '{args.emergency_veh}' from route-files "
            f"(route_edges={len(ev_route_edges_cfg)})"
        )
    if bool(getattr(args, "auto_induction_loops", False)) and loop_deploy_scope == "ev-route":
        ev_route_edges_for_loops = list(ev_route_edges_cfg or [])
        if ev_route_edges_for_loops:
            loop_subset_tls, loop_core_tls = select_tls_subset_for_ev_route(
                route_edges=list(ev_route_edges_for_loops),
                edge_to_tls=edge_to_tls_static,
                tls_neighbors=tls_neighbors_static,
                neighbor_hops=int(getattr(args, "agent_subset_neighbor_hops", 1)),
            )
            if loop_subset_tls:
                base_target_tls = set(str(t) for t in target_tls)
                filtered_loop_tls = [str(tl) for tl in loop_subset_tls if str(tl) in base_target_tls]
                if filtered_loop_tls:
                    loop_subset_tls_for_loops = list(filtered_loop_tls)
                    loop_core_tls_for_loops = [str(tl) for tl in list(loop_core_tls or []) if str(tl) in set(filtered_loop_tls)]
                    loop_target_nodes = sorted({
                        str(n)
                        for tl in filtered_loop_tls
                        for n in list(tls_to_nodes.get(str(tl), []))
                    })
                    print(
                        f"[loop-subset] mode=ev-route ev={args.emergency_veh} route_edges={len(ev_route_edges_for_loops)} "
                        f"core_tls={len(loop_core_tls)} expanded_tls={len(loop_subset_tls)} loop_target_nodes={len(loop_target_nodes)}"
                    )
                else:
                    print("[loop-subset][WARN] EV-route subset resolved but none overlap requested target TLS; using base loop target nodes")
            else:
                print("[loop-subset][WARN] Could not map EV route to TLS for loop subset; using base loop target nodes")
        else:
            print(f"[loop-subset][WARN] Could not extract route for EV '{args.emergency_veh}' before SUMO start; using base loop target nodes")
    elif bool(getattr(args, "auto_induction_loops", False)):
        print(f"[loop-subset] deploy_scope=all loop_target_nodes={len(loop_target_nodes)} (no EV-route restriction)")

    # MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(args.mqtt_host, 1883, 60)

    cmd_queue = deque()
    corridor_state_cache: Dict[str, Dict[str, Any]] = {}
    corridor_route_advice_by_ev: Dict[str, Dict[str, Any]] = {}
    last_graph_cost_print_sec: int = -1
    fed_bootstrap_enabled = bool(getattr(args, "federation_bootstrap_enable", False))
    fed_bootstrap_participants_mode = str(getattr(args, "federation_bootstrap_participants", "hub") or "hub").strip().lower()
    if fed_bootstrap_participants_mode not in ("hub", "dts", "hybrid"):
        fed_bootstrap_participants_mode = "hub"
    fed_bootstrap_gateway_id = str(getattr(args, "federation_bootstrap_gateway_id", "gw-realworld-main") or "gw-realworld-main")
    fed_bootstrap_node_id = str(getattr(args, "federation_bootstrap_node_id", "realworld-main") or "realworld-main")
    fed_bootstrap_role = str(getattr(args, "federation_bootstrap_role", "simulation_hub") or "simulation_hub")
    fed_bootstrap_domain = str(getattr(args, "federation_bootstrap_domain", "traffic") or "traffic")
    fed_bootstrap_ev_id = str(getattr(args, "emergency_veh", "emergency1") or "emergency1")
    fed_bootstrap_register_topic = str(getattr(args, "federation_bootstrap_register_topic", "federation/membership/register") or "federation/membership/register")
    fed_bootstrap_heartbeat_topic = str(getattr(args, "federation_bootstrap_heartbeat_topic", "federation/membership/heartbeat") or "federation/membership/heartbeat")
    fed_bootstrap_catalog_topic = str(getattr(args, "federation_bootstrap_catalog_topic", "federation/catalog/upsert") or "federation/catalog/upsert")
    fed_bootstrap_discovery_query_topic = str(getattr(args, "federation_bootstrap_discovery_query_topic", "federation/discovery/query") or "federation/discovery/query")
    fed_bootstrap_discovery_reply_prefix = str(getattr(args, "federation_bootstrap_discovery_reply_prefix", "federation/discovery/resp") or "federation/discovery/resp").rstrip("/")
    fed_bootstrap_discovery_reply_topic = f"{fed_bootstrap_discovery_reply_prefix}/{fed_bootstrap_node_id}"
    fed_bootstrap_discovery_event_filter = str(getattr(args, "federation_bootstrap_discovery_event_filter", "") or "").strip()
    ev_request_delivery_mode = str(getattr(args, "ev_request_delivery", "direct") or "direct").strip().lower()
    if ev_request_delivery_mode not in {"direct", "mqtt", "both"}:
        ev_request_delivery_mode = "direct"
    internal_ev_request_enabled = not bool(getattr(args, "disable_internal_ev_request_generation", False))
    ev_request_topic_prefix = str(getattr(args, "ev_request_topic_prefix", "federation/ev/request") or "federation/ev/request").rstrip("/")
    ev_request_source_tag = str(getattr(args, "ev_request_source_tag", "") or "").strip()
    ev_http_adapter_enabled = bool(getattr(args, "ev_http_adapter_enable", False))
    ev_http_state_url = str(getattr(args, "ev_http_state_url", "") or "").strip()
    ev_http_poll_sec = max(0.1, float(getattr(args, "ev_http_poll_sec", 1.0) or 1.0))
    ev_http_timeout_sec = max(0.1, float(getattr(args, "ev_http_timeout_sec", 0.8) or 0.8))
    ev_http_headers_raw = list(getattr(args, "ev_http_header", []) or [])
    ev_http_headers: Dict[str, str] = {}
    for hdr in ev_http_headers_raw:
        h = str(hdr or "").strip()
        if not h or ":" not in h:
            continue
        k, v = h.split(":", 1)
        k = str(k).strip()
        v = str(v).strip()
        if k:
            ev_http_headers[k] = v
    ev_http_publish_state_topic = str(getattr(args, "ev_http_publish_state_topic", "rw/vehicle_agent/http/state") or "").strip()
    ev_http_last_poll_sim = -1e9
    ev_http_last_err_key: str = ""
    ev_http_state_server_enabled = bool(getattr(args, "ev_http_state_server_enable", False))
    ev_http_state_server_host = str(getattr(args, "ev_http_state_server_host", "127.0.0.1") or "127.0.0.1")
    ev_http_state_server_port = int(getattr(args, "ev_http_state_server_port", 18083) or 18083)
    ev_http_state_server_verbose = bool(getattr(args, "ev_http_state_server_verbose", False))
    ev_pipeline_log_period_sec = float(getattr(args, "ev_pipeline_log_period_sec", 1.0) or 0.0)
    ev_state_trace_period_sec = float(getattr(args, "ev_state_trace_period_sec", 1.0) or 0.0)
    fed_bootstrap_heartbeat_sec = max(1.0, float(getattr(args, "federation_bootstrap_heartbeat_sec", 5.0)))
    fed_bootstrap_catalog_sec = max(2.0, float(getattr(args, "federation_bootstrap_catalog_sec", 30.0)))
    fed_bootstrap_discovery_probe_sec = max(0.0, float(getattr(args, "federation_bootstrap_discovery_probe_sec", 0.0)))
    fed_bootstrap_last_heartbeat_wall = 0.0
    fed_bootstrap_last_catalog_wall = 0.0
    fed_bootstrap_last_probe_wall = 0.0
    fed_bootstrap_probe_counter = 0
    fed_bootstrap_registered_gateways: set[str] = set()
    

    # Optional: auto-generate induction loops before SUMO startup.
    auto_loop_lane_map: Dict[str, List[str]] = {}
    generated_loop_add_file = ""
    if bool(args.auto_induction_loops):
        if str(args.loop_add_file).strip():
            out_add = str(args.loop_add_file).strip()
        else:
            out_add = os.path.join(tempfile.gettempdir(), f"auto_loops_{os.getpid()}.add.xml")
        try:
            generated_loop_add_file, auto_loop_lane_map, loop_n = build_auto_induction_loops_additional(
                net_file=args.net_file,
                target_nodes=loop_target_nodes,
                out_add_file=out_add,
                distance_to_stopline_m=float(args.loop_distance_to_stopline),
                freq_sec=int(args.loop_freq),
            )
            print(f"[loops] generated {loop_n} induction loops at {generated_loop_add_file}")
        except Exception as e:
            print(f"[loops][WARN] failed generating induction loops: {e}")
            generated_loop_add_file = ""
            auto_loop_lane_map = {}

    existing_additional_files = _read_sumocfg_additional_files(args.sumo_cfg)
    merged_additional_files = list(existing_additional_files)
    if generated_loop_add_file:
        if generated_loop_add_file not in merged_additional_files:
            merged_additional_files.append(generated_loop_add_file)

    # Start SUMO
    sumo_cmd = [
        args.sumo_bin,
        "--seed", "1111",
        "-c", args.sumo_cfg,
        "--step-length", str(args.step_length),
        "--start",
    ]
    if float(getattr(args, "sumo_lateral_resolution", -1.0)) > 0.0:
        sumo_cmd += ["--lateral-resolution", str(float(args.sumo_lateral_resolution))]
    extra_sumo_args = str(getattr(args, "sumo_extra_args", "") or "").strip()
    if extra_sumo_args:
        try:
            sumo_cmd += list(shlex.split(extra_sumo_args))
        except Exception as e:
            print(f"[SUMO][WARN] could not parse --sumo-extra-args='{extra_sumo_args}': {e}")
    if merged_additional_files:
        sumo_cmd += ["--additional-files", ",".join(merged_additional_files)]
    print("Starting SUMO:", " ".join(sumo_cmd))
    traci.start(sumo_cmd, label="main")
    if bool(args.auto_induction_loops):
        try:
            loaded_loop_ids = list(traci.inductionloop.getIDList())
            print(f"[loops] runtime loaded loops: {len(loaded_loop_ids)}")
            for lp in loaded_loop_ids[:20]:
                try:
                    lp_lane = str(traci.inductionloop.getLaneID(str(lp)))
                except Exception:
                    lp_lane = "<lane_read_failed>"
                print(f"[loops] id={lp} lane={lp_lane}")
            if len(loaded_loop_ids) == 0:
                print("[loops][WARN] No induction loops loaded at runtime. Check --additional-files and generated add.xml.")
        except Exception as e:
            print(f"[loops][WARN] Could not inspect runtime loop IDs: {e}")
    

    # Create agents AFTER TraCI is live (warm_start needs TraCI)
    agents = {}
    live_tls_ids = set(traci.trafficlight.getIDList())
    print(f"Live TLS controllers in SUMO: {len(live_tls_ids)}")
    missing = [tl for tl in target_tls if tl not in live_tls_ids]
    if missing:
        print("[WARN] These target TLS IDs were not found in SUMO:", missing)

    tls_neighbors, edge_to_tls = build_tls_neighbors_from_net(args.net_file, node_to_tls, tls_to_nodes)
    print(f"Neighbour graph built for {len(tls_neighbors)} TLS controllers")

    eligible_tls_ids = sorted([tl for tl in target_tls if tl in live_tls_ids]) if target_tls else sorted(live_tls_ids)
    if not eligible_tls_ids and live_tls_ids:
        print("[WARN] No mapped target TLS IDs were found at runtime; falling back to all live TLS controllers.")
        eligible_tls_ids = sorted(live_tls_ids)
    eligible_tls_ids_set = set(str(x) for x in eligible_tls_ids)

    selected_tls_ids = list(eligible_tls_ids)
    subset_tls_ids: List[str] = []
    core_route_tls_ids: List[str] = []

    if str(getattr(args, "agent_subset", "all")) == "ev-route":
        if ev_route_edges_cfg:
            subset_tls_ids, core_route_tls_ids = select_tls_subset_for_ev_route(
                route_edges=list(ev_route_edges_cfg),
                edge_to_tls=edge_to_tls,
                tls_neighbors=tls_neighbors,
                neighbor_hops=int(getattr(args, "agent_subset_neighbor_hops", 1)),
            )
            if subset_tls_ids:
                before_n = len(selected_tls_ids)
                subset_set = set(subset_tls_ids)
                selected_tls_ids = sorted([tl for tl in selected_tls_ids if tl in subset_set]) if selected_tls_ids else sorted(subset_set)
                print(
                    f"[agent-subset] mode=ev-route ev={args.emergency_veh} route_edges={len(ev_route_edges_cfg)} "
                    f"core_tls={len(core_route_tls_ids)} expanded_tls={len(subset_tls_ids)} selected={len(selected_tls_ids)} "
                    f"neighbor_hops={int(getattr(args, 'agent_subset_neighbor_hops', 1))} from_base={before_n}"
                )
                if core_route_tls_ids:
                    print(f"[agent-subset] core route TLS (first 20): {core_route_tls_ids[:20]}")
            else:
                print(f"[agent-subset][WARN] EV route found for {args.emergency_veh} but no TLS mapped from its edges; using base selection")
        else:
            print(f"[agent-subset][WARN] Could not extract route for EV '{args.emergency_veh}' from sumo cfg route-files; using base selection")

    agent_activation_mode = str(getattr(args, "agent_activation_mode", "static") or "static").strip().lower()
    on_demand_agent_activation = bool(agent_activation_mode == "on-demand")
    on_demand_max_new_per_tick = max(1, int(getattr(args, "agent_on_demand_max_new_per_tick", 8)))
    on_demand_lookahead_hops = max(1, int(getattr(args, "agent_on_demand_lookahead_hops", 6)))

    if on_demand_agent_activation:
        startup_seed: List[str] = []
        seed_source = list(core_route_tls_ids or loop_core_tls_for_loops or selected_tls_ids)
        for tls_id in list(seed_source):
            stls = str(tls_id)
            if not stls or stls in startup_seed or stls not in eligible_tls_ids_set:
                continue
            startup_seed.append(stls)
            if len(startup_seed) >= on_demand_lookahead_hops:
                break
        if not startup_seed:
            for tls_id in list(selected_tls_ids):
                stls = str(tls_id)
                if not stls or stls in startup_seed:
                    continue
                startup_seed.append(stls)
                if len(startup_seed) >= on_demand_lookahead_hops:
                    break
        if startup_seed:
            selected_tls_ids = list(startup_seed)
            print(
                f"[agent-activation] on-demand startup_seed={len(selected_tls_ids)} "
                f"lookahead_hops={on_demand_lookahead_hops}"
            )

    active_agent_tls_ids = list(selected_tls_ids)
    active_agent_tls_set = set(str(x) for x in active_agent_tls_ids)
    loop_sense_tls_ids_expanded = [str(tl) for tl in (loop_subset_tls_for_loops or active_agent_tls_ids) if str(tl) in active_agent_tls_set]
    loop_sense_tls_ids_core = [str(tl) for tl in (loop_core_tls_for_loops or loop_sense_tls_ids_expanded) if str(tl) in active_agent_tls_set]
    loop_sense_scope_mode = str(getattr(args, "loop_sense_scope", "auto") or "auto").strip().lower()
    loop_sense_route_edges_last: Optional[Tuple[str, ...]] = None
    loop_sense_active_tls_ids: List[str] = list(loop_sense_tls_ids_core or loop_sense_tls_ids_expanded or active_agent_tls_ids)
    if bool(getattr(args, "auto_induction_loops", False)):
        print(
            f"[loop-sense] expanded_tls={len(loop_sense_tls_ids_expanded)} core_tls={len(loop_sense_tls_ids_core)} "
            f"period={float(getattr(args, 'loop_sense_period_sec', 1.0)):.2f}s scope={loop_sense_scope_mode}"
        )

    print(
        f"[agent-activation] mode={agent_activation_mode} "
        f"initial={len(active_agent_tls_ids)} eligible={len(eligible_tls_ids_set)} "
        f"on_demand_max_new_per_tick={on_demand_max_new_per_tick} lookahead_hops={on_demand_lookahead_hops}"
    )

    def _loop_lane_map_for_tls(tls_id: str) -> Dict[str, List[str]]:
        per_tls_loop_lane_map: Dict[str, List[str]] = {}
        if bool(args.auto_induction_loops) and auto_loop_lane_map:
            try:
                controlled_lanes_raw = list(traci.trafficlight.getControlledLanes(tls_id))
            except Exception:
                controlled_lanes_raw = []
            controlled_lanes = {str(ln) for ln in controlled_lanes_raw if str(ln)}
            if controlled_lanes:
                per_tls_loop_lane_map = {
                    str(lane_id): [str(x) for x in (loop_ids or [])]
                    for lane_id, loop_ids in dict(auto_loop_lane_map).items()
                    if str(lane_id) in controlled_lanes
                }
            else:
                per_tls_loop_lane_map = {}
        return per_tls_loop_lane_map

    def _create_agent_for_tls(tls_id: str) -> Optional[IntersectionAgent]:
        stls = str(tls_id)
        if not stls:
            return None
        if stls not in live_tls_ids:
            return None
        debug = bool(stls in debug_tls_ids)
        per_tls_loop_lane_map = _loop_lane_map_for_tls(stls)
        cfg = IntersectionAgentConfig(
            intersection_id=stls,
            tls_id=stls,
            decision_period_sec=1.0,
            enable_volatile_connectivity=False,
            drop_prob=0.1,
            max_delay_sec=1.0,
            enable_decision_csv_log=bool(args.decision_log),
            decision_log_csv_path=str(args.decision_log_csv),
            decision_log_run_label=str(CURRENT_EVALUATION),
            enable_queue_metrics_debug=True,
            queue_use_induction_loops=bool(args.auto_induction_loops),
            queue_metrics_paper_strict_mode=bool(args.paper_strict_metrics),
            queue_loop_ids_by_lane=dict(per_tls_loop_lane_map),
           
        )
        cfg.queue_loop_count_mode = "adaptive"
        cfg.queue_loop_detector_freq_sec = float(getattr(args, "loop_freq", 1))
        cfg.queue_loop_interval_min_poll_gap_sec = max(0.1, float(getattr(args, "loop_sense_period_sec", 1.0)))
        cfg.queue_loop_interval_sparse_factor = 1.5
        # Runtime federation tuning/debug knobs (not required in dataclass ctor).
        cfg.enable_federation_debug = bool(args.fed_debug) or bool(fed_debug_log_file)
        cfg.fed_debug_print = bool(args.fed_debug)
        cfg.fed_debug_log_path = str(fed_debug_log_file)
        cfg.enable_fed_event_jsonl = True
        cfg.fed_event_jsonl_path = (
            str(fed_debug_log_file).replace('.txt', '.events.jsonl')
            if str(fed_debug_log_file).strip() else ''
        )
        cfg.ev_request_source_tag = str(ev_request_source_tag)
        cfg.tls_signal_trace_enable = bool(getattr(args, "tls_signal_trace_enable", False))
        cfg.fed_force_route_hint_top1 = bool(args.fed_force_route_top1)
        cfg.fed_route_hint_prob_floor = float(args.fed_route_prob_floor)
        cfg.fed_enable_warmup = bool(args.fed_enable_warmup)
        cfg.fed_warmup_hard_only = bool(args.fed_warmup_hard_only)
        cfg.fed_warmup_period_sec = float(args.fed_warmup_period_sec)
        cfg.fed_warm_horizon_sec = float(args.fed_warm_horizon_sec)
        cfg.fed_hard_min_queue_margin_sec = float(args.fed_hard_min_queue_margin_sec)
        cfg.fed_hard_max_spillback_risk = float(args.fed_hard_max_spillback_risk)
        cfg.fed_readiness_use_improved_queue = bool(args.fed_readiness_use_improved_queue)
        cfg.f2_ev_guard_enable = bool(args.f2_ev_guard_enable)
        cfg.f2_ev_guard_wait_penalty_sec = float(args.f2_ev_guard_wait_penalty_sec)
        cfg.f2_ev_guard_miss_penalty_sec = float(args.f2_ev_guard_miss_penalty_sec)
        cfg.f2_ev_guard_require_feasible = bool(args.f2_ev_guard_require_feasible)
        cfg.f2_selection_policy = str(getattr(args, "f2_selection_policy", "measured") or "measured")
        cfg.f2_block_infeasible_actuation = bool(getattr(args, "f2_block_infeasible_actuation", True))
        cfg.f2_refine_require_feedback = bool(getattr(args, "f2_refine_require_feedback", True))
        cfg.f2_refine_feedback_max_age_sec = float(getattr(args, "f2_refine_feedback_max_age_sec", 6.0))
        cfg.f2_refine_require_loop_coverage = bool(getattr(args, "f2_refine_require_loop_coverage", True))
        cfg.f2_refine_min_loop_coverage_ratio = float(getattr(args, "f2_refine_min_loop_coverage_ratio", 0.5))
        _fed_dbg_main(
            f"evt=CFG_WARMUP tls={tls_id} warmup={1 if cfg.fed_enable_warmup else 0} "
            f"hard_only={1 if cfg.fed_warmup_hard_only else 0} period={float(cfg.fed_warmup_period_sec):.2f} "
            f"horizon={float(cfg.fed_warm_horizon_sec):.2f} "
            f"q_margin_min={float(cfg.fed_hard_min_queue_margin_sec):.2f} "
            f"spill_max={float(cfg.fed_hard_max_spillback_risk):.2f} "
            f"readiness_improved={1 if cfg.fed_readiness_use_improved_queue else 0}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_GUARD tls={tls_id} enabled={1 if cfg.f2_ev_guard_enable else 0} "
            f"wait_eps={float(cfg.f2_ev_guard_wait_penalty_sec):.2f} "
            f"miss_eps={float(cfg.f2_ev_guard_miss_penalty_sec):.2f} "
            f"require_feasible={1 if cfg.f2_ev_guard_require_feasible else 0}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_POLICY tls={tls_id} policy={str(cfg.f2_selection_policy)} "
            f"block_infeasible={1 if cfg.f2_block_infeasible_actuation else 0} "
            f"require_feedback={1 if cfg.f2_refine_require_feedback else 0} "
            f"feedback_age_max={float(cfg.f2_refine_feedback_max_age_sec):.2f} "
            f"require_loop_cov={1 if cfg.f2_refine_require_loop_coverage else 0} "
            f"loop_cov_min={float(cfg.f2_refine_min_loop_coverage_ratio):.2f}"
        )
        _fed_dbg_main(
            f"evt=CFG_EVENTS tls={tls_id} enabled={int(bool(getattr(cfg, 'enable_fed_event_jsonl', False)))} "
            f"path={str(getattr(cfg, 'fed_event_jsonl_path', '') or '')}"
        )
        ag = IntersectionAgent(cfg, step_length_sec=args.step_length)
        ag.warm_start()
        ag.capture_default_tls_program()
        nbs = sorted(tls_neighbors.get(stls, set()))
        ag.neighbour_map_federation = {nb: {"last_seen": -1.0, "confidence": 0.0} for nb in nbs}
        return ag

    def _activate_tls_agents(candidate_tls_ids: List[str], reason: str, sim_time: float = -1.0, max_new: Optional[int] = None) -> int:
        max_new_eff = on_demand_max_new_per_tick if max_new is None else max(1, int(max_new))
        created = 0
        seen_local: set = set()
        for tls_id in list(candidate_tls_ids or []):
            stls = str(tls_id)
            if not stls or stls in seen_local:
                continue
            seen_local.add(stls)
            if stls in agents:
                if stls not in active_agent_tls_set:
                    active_agent_tls_set.add(stls)
                    active_agent_tls_ids.append(stls)
                continue
            if stls not in eligible_tls_ids_set:
                _fed_dbg_main(f"evt=AGENT_ACTIVATE_SKIP tls={stls} reason=outside_eligible_set trigger={reason}")
                continue
            if stls not in live_tls_ids:
                _fed_dbg_main(f"evt=AGENT_ACTIVATE_SKIP tls={stls} reason=not_live_in_sumo trigger={reason}")
                continue
            ag_new = _create_agent_for_tls(stls)
            if ag_new is None:
                _fed_dbg_main(f"evt=AGENT_ACTIVATE_SKIP tls={stls} reason=create_failed trigger={reason}")
                continue
            agents[stls] = ag_new
            if stls not in active_agent_tls_set:
                active_agent_tls_set.add(stls)
                active_agent_tls_ids.append(stls)
            if stls not in loop_sense_tls_ids_expanded:
                loop_sense_tls_ids_expanded.append(stls)
            if stls not in loop_sense_active_tls_ids:
                loop_sense_active_tls_ids.append(stls)
            created += 1
            _fed_dbg_main(
                f"evt=AGENT_ACTIVATE_OK tls={stls} trigger={reason} sim={float(sim_time):.2f} "
                f"active_total={len(active_agent_tls_ids)}"
            )
            print(
                f"[agent-activation] activated tls={stls} trigger={reason} sim={float(sim_time):.1f} "
                f"active_total={len(active_agent_tls_ids)}"
            )
            if created >= max_new_eff:
                break
        return int(created)

    def _route_advice_tls_candidates(route_adv_payload: Dict[str, Any]) -> List[str]:
        route_opt = dict(route_adv_payload.get("route_optimization", {}) or {})
        out: List[str] = []
        seen: set = set()

        def _push(v: Any) -> None:
            s = str(v or "").strip()
            if not s or s in seen:
                return
            seen.add(s)
            out.append(s)

        for key in ("optimized_next_tls", "destination_tls", "current_next_tls"):
            _push(route_opt.get(key, ""))
        for key in ("optimized_path_tls", "current_path_tls"):
            for tls_id in list(route_opt.get(key, []) or []):
                _push(tls_id)
        for edge_id in list(route_opt.get("optimized_path_edges", []) or []):
            tls_id = str(edge_to_tls.get(str(edge_id), "") or "")
            if tls_id:
                _push(tls_id)
        max_hops = max(1, int(on_demand_lookahead_hops))
        if len(out) > max_hops:
            return list(out[:max_hops])
        return out

    def _ev_http_fetch_state() -> Optional[Dict[str, Any]]:
        if not ev_http_state_url:
            return None
        req = url_request.Request(ev_http_state_url, headers=dict(ev_http_headers), method="GET")
        try:
            with url_request.urlopen(req, timeout=float(ev_http_timeout_sec)) as resp:
                raw = resp.read()
                txt = raw.decode("utf-8", errors="replace")
                obj = json.loads(txt)
                if isinstance(obj, dict):
                    return dict(obj)
                return None
        except (url_error.URLError, TimeoutError, socket.timeout, ValueError):
            return None
        except Exception:
            return None

    def _as_float(v: Any, default: float) -> float:
        try:
            return float(v)
        except Exception:
            return float(default)

    def _as_int(v: Any, default: int) -> int:
        try:
            return int(v)
        except Exception:
            return int(default)

    def _tls_from_edge(edge_id: str) -> str:
        e = str(edge_id or "")
        if not e or e.startswith(":"):
            return ""
        n = edge_to_approach_node_from_net(e, edge_to_to_node)
        if n:
            cand = sorted(node_to_tls.get(str(n), []) or [])
            if cand:
                return str(cand[0])
        return str(edge_to_tls.get(e, "") or "")

    def _ev_http_state_to_requests(state_obj: Dict[str, Any], sim_time_now: float, default_ev_id: str, default_erl_level: int) -> List[Tuple[str, Dict[str, Any]]]:
        out: List[Tuple[str, Dict[str, Any]]] = []
        ev_id_raw = str(
            state_obj.get("ev_id")
            or state_obj.get("evId")
            or state_obj.get("vehicle_id")
            or state_obj.get("vehicleId")
            or default_ev_id
        )
        sim_t = _as_float(state_obj.get("sim_time", state_obj.get("simTime", sim_time_now)), sim_time_now)
        speed = _as_float(state_obj.get("speed_mps", state_obj.get("speedMps", state_obj.get("speed", 0.0))), 0.0)
        edge_fallback = str(state_obj.get("in_edge_id", state_obj.get("inEdgeId", state_obj.get("edge_id", state_obj.get("edge", "")))) or "")
        route_veh = list(state_obj.get("route_veh", state_obj.get("routeVeh", state_obj.get("route_edges", state_obj.get("routeEdges", [])))) or [])
        route_intersections = list(state_obj.get("route_intersections", state_obj.get("routeIntersections", [])) or [])
        erl = _as_int(state_obj.get("erl_level", state_obj.get("erlLevel", default_erl_level)), default_erl_level)
        delta_sec = _as_float(state_obj.get("delta_sec", state_obj.get("deltaSec", 2.0)), 2.0)

        # Preferred schema: next_tls list with per-target distance.
        next_tls = list(state_obj.get("next_tls", state_obj.get("nextTls", [])) or [])
        for item in next_tls[:1]:
            if not isinstance(item, dict):
                continue
            tls_id = str(item.get("tls_id", item.get("tlsId", "")) or "")
            if not tls_id:
                continue
            in_edge = str(item.get("in_edge_id", item.get("inEdgeId", edge_fallback)) or "")
            dist = _as_float(
                item.get(
                    "distance_to_intersection_m",
                    item.get("distanceToIntersectionM", item.get("distance_m", item.get("distanceM", state_obj.get("dist_to_stopline_m", state_obj.get("distToStoplineM", 1e9))))),
                ),
                1e9,
            )
            req = {
                "ev_id": str(ev_id_raw),
                "sim_time": float(sim_t),
                "erl_level": int(erl),
                "speed_mps": float(speed),
                "distance_to_intersection_m": float(dist),
                "in_edge_id": str(in_edge),
                "target_phase_idx": None,
                "delta_sec": float(delta_sec),
                "route_intersections": list(route_intersections) if route_intersections else None,
                "route_veh": [str(x) for x in list(route_veh or [])],
                "source_service": "ev_http_adapter",
                "source_tag": str(ev_request_source_tag or "ev_http_adapter"),
                "delivery": "mqtt",
            }
            out.append((tls_id, req))

        # Fallback schema: single target from tls_id or edge mapping.
        if not out:
            tls_id_fallback = str(state_obj.get("tls_id", state_obj.get("tlsId", "")) or "")
            if not tls_id_fallback and edge_fallback:
                tls_id_fallback = _tls_from_edge(edge_fallback)
            if tls_id_fallback:
                dist_fb = _as_float(
                    state_obj.get(
                        "distance_to_intersection_m",
                        state_obj.get("distanceToIntersectionM", state_obj.get("dist_to_stopline_m", state_obj.get("distToStoplineM", 1e9))),
                    ),
                    1e9,
                )
                req = {
                    "ev_id": str(ev_id_raw),
                    "sim_time": float(sim_t),
                    "erl_level": int(erl),
                    "speed_mps": float(speed),
                    "distance_to_intersection_m": float(dist_fb),
                    "in_edge_id": str(edge_fallback),
                    "target_phase_idx": None,
                    "delta_sec": float(delta_sec),
                    "route_intersections": list(route_intersections) if route_intersections else None,
                    "route_veh": [str(x) for x in list(route_veh or [])],
                    "source_service": "ev_http_adapter",
                    "delivery": "mqtt",
                }
                out.append((tls_id_fallback, req))
        return out

    _activate_tls_agents(list(active_agent_tls_ids), reason="startup_initial", sim_time=-1.0, max_new=max(1, len(active_agent_tls_ids)))
    print(f"Agents instantiated: {len(agents)}")
    print("Agents instantiated for (first 30):", list(agents.keys())[:30])

    if fed_bootstrap_enabled:
        print(
            f"[FED_BOOTSTRAP] enabled mode={fed_bootstrap_participants_mode} gateway={fed_bootstrap_gateway_id} node={fed_bootstrap_node_id} "
            f"role={fed_bootstrap_role} hb={fed_bootstrap_heartbeat_sec:.1f}s catalog={fed_bootstrap_catalog_sec:.1f}s "
            f"probe={fed_bootstrap_discovery_probe_sec:.1f}s"
        )

    def _fed_bootstrap_member_defs() -> List[Dict[str, Any]]:
        members: List[Dict[str, Any]] = []
        mode = str(fed_bootstrap_participants_mode or "hub").strip().lower()
        if mode in ("hub", "hybrid"):
            members.append(
                {
                    "kind": "hub",
                    "gateway_id": fed_bootstrap_gateway_id,
                    "node_id": fed_bootstrap_node_id,
                    "role": fed_bootstrap_role,
                    "domain": fed_bootstrap_domain,
                }
            )
        if mode in ("dts", "hybrid"):
            members.append(
                {
                    "kind": "ev",
                    "gateway_id": f"gw-ev-{fed_bootstrap_ev_id}",
                    "node_id": str(fed_bootstrap_ev_id),
                    "role": "EmergencyVehicle",
                    "domain": fed_bootstrap_domain,
                }
            )
            for tls_id in sorted(set(str(x) for x in list(active_agent_tls_ids or []))):
                members.append(
                    {
                        "kind": "tls",
                        "gateway_id": f"gw-tls-{tls_id}",
                        "node_id": str(tls_id),
                        "role": "TrafficLightSystem",
                        "domain": fed_bootstrap_domain,
                    }
                )
        return members

    def _fed_bootstrap_service_entries(member: Dict[str, Any]) -> List[Dict[str, Any]]:
        kind = str(member.get("kind", "") or "").lower()
        node_id = str(member.get("node_id", "") or "")
        if kind == "hub":
            return [
                {
                    "name": "rw_step_pub",
                    "direction": "local_to_fed",
                    "event_type": "sim_step",
                    "publish_topic": "rw/step",
                    "subscribe_topic": "cmd/sim/#",
                },
                {
                    "name": "rw_vehicle_state_pub",
                    "direction": "local_to_fed",
                    "event_type": "vehicle_state",
                    "publish_topic": "rw/vehicle/+/state",
                    "subscribe_topic": "cmd/vehicle/+/#",
                },
                {
                    "name": "rw_tls_state_pub",
                    "direction": "local_to_fed",
                    "event_type": "tls_state",
                    "publish_topic": "rw/tls/+/state",
                    "subscribe_topic": "cmd/tls/+/#",
                },
                {
                    "name": "federation_reservation_bridge",
                    "direction": "local_to_fed",
                    "event_type": "federation_reservation",
                    "publish_topic": "federation/reservation/req/+",
                    "subscribe_topic": "federation/reservation/resp/+",
                },
                {
                    "name": "ev_request_bridge",
                    "direction": "local_to_fed",
                    "event_type": "ev_request",
                    "publish_topic": f"{ev_request_topic_prefix}/+",
                    "subscribe_topic": f"{ev_request_topic_prefix}/+",
                },
                {
                    "name": "corridor_route_advice_bridge",
                    "direction": "fed_to_local",
                    "event_type": "corridor_route_advice",
                    "publish_topic": "rw/vehicle/+/route_advice",
                    "subscribe_topic": "federation/corridor/route_advice/+",
                },
                {
                    "name": "active_intersection_scope",
                    "direction": "local_to_fed",
                    "event_type": "scope",
                    "publish_topic": f"rw/agents/active/{len(active_agent_tls_ids)}",
                    "subscribe_topic": "rw/agent/+/plan",
                },
            ]
        if kind == "ev":
            return [
                {
                    "name": f"ev_state_pub_{node_id}",
                    "direction": "local_to_fed",
                    "event_type": "vehicle_state",
                    "publish_topic": f"rw/vehicle/{node_id}/state",
                    "subscribe_topic": f"cmd/vehicle/{node_id}/#",
                },
                {
                    "name": f"ev_route_advice_sub_{node_id}",
                    "direction": "fed_to_local",
                    "event_type": "corridor_route_advice",
                    "publish_topic": f"rw/vehicle/{node_id}/route_advice",
                    "subscribe_topic": f"federation/corridor/route_advice/{node_id}",
                },
            ]
        # tls
        return [
            {
                "name": f"tls_state_pub_{node_id}",
                "direction": "local_to_fed",
                "event_type": "tls_state",
                "publish_topic": f"rw/tls/{node_id}/state",
                "subscribe_topic": f"cmd/tls/{node_id}/#",
            },
            {
                "name": f"tls_plan_pub_{node_id}",
                "direction": "local_to_fed",
                "event_type": "tls_plan",
                "publish_topic": f"rw/agent/{node_id}/plan",
                "subscribe_topic": f"federation/corridor/advice/{node_id}",
            },
            {
                "name": f"tls_warmup_pub_{node_id}",
                "direction": "local_to_fed",
                "event_type": "tls_warmup",
                "publish_topic": f"rw/agent/{node_id}/warmup_plan",
                "subscribe_topic": f"federation/corridor/verdict/{node_id}",
            },
            {
                "name": f"tls_reservation_bridge_{node_id}",
                "direction": "local_to_fed",
                "event_type": "federation_reservation",
                "publish_topic": f"federation/reservation/resp/{node_id}",
                "subscribe_topic": f"federation/reservation/req/{node_id}",
            },
        ]

    def _fed_bootstrap_dt_profile(member: Dict[str, Any]) -> Dict[str, Any]:
        kind = str(member.get("kind", "") or "").lower()
        node_id = str(member.get("node_id", "") or "")
        role = str(member.get("role", "") or "")
        network_name = str(os.path.basename(str(getattr(args, "net_file", "") or "")) or "")
        city_name = str(getattr(args, "federation_bootstrap_city", "Madrid") or "Madrid")
        owner_org = str(getattr(args, "federation_bootstrap_owner_org", "prototype-lab") or "prototype-lab")
        owner_operator = str(getattr(args, "federation_bootstrap_owner_operator", "real-world.py") or "real-world.py")
        if kind == "ev":
            geo_scope = {
                "type": "route",
                "city": city_name,
                "zone": "dynamic",
                "network": network_name,
                "id": node_id,
            }
            policy_tags = ["emergency", "mobile", "route-advice-consumer"]
            update_period_sec = max(0.1, float(getattr(args, "publish_vehicle_period_sec", 1.0) or 1.0))
            latency_budget_ms = 300.0
        elif kind == "tls":
            geo_scope = {
                "type": "intersection",
                "city": city_name,
                "zone": "dynamic",
                "network": network_name,
                "id": node_id,
            }
            policy_tags = ["intersection", "signal-control", "federation-participant"]
            update_period_sec = max(0.1, float(getattr(args, "publish_tls_state_period_sec", 1.0) or 1.0))
            latency_budget_ms = 250.0
        else:
            geo_scope = {
                "type": "city",
                "city": city_name,
                "zone": "global",
                "network": network_name,
                "id": str(member.get("gateway_id", "") or ""),
            }
            policy_tags = ["orchestrator", "bridge", "federation-core"]
            update_period_sec = max(0.1, float(getattr(args, "publish_step_period_sec", 1.0) or 1.0))
            latency_budget_ms = 500.0

        return {
            "dt_description": f"{role} digital twin for {node_id}",
            "geo_scope": geo_scope,
            "policy_tags": sorted(set(policy_tags)),
            "qos_sla": {
                "update_period_sec": float(update_period_sec),
                "latency_budget_ms": float(latency_budget_ms),
                "availability_target": "best_effort",
            },
            "interface_version": "federation.catalog.paper_profile.v1",
            "ownership": {
                "organization": owner_org,
                "domain": str(fed_bootstrap_domain),
                "operator": owner_operator,
            },
        }

    def _fed_bootstrap_publish_register(member: Dict[str, Any]) -> None:
        gateway_id = str(member.get("gateway_id", "") or "")
        node_id = str(member.get("node_id", "") or "")
        role = str(member.get("role", "") or "")
        domain = str(member.get("domain", "") or fed_bootstrap_domain)
        services = _fed_bootstrap_service_entries(member)
        caps = sorted({str(x.get("event_type", "")) for x in services if str(x.get("event_type", ""))})
        payload = {
            "schema": "federation.membership.v1",
            "request_id": f"realworld-reg-{gateway_id}-{int(time.time() * 1000)}",
            "gateway_id": gateway_id,
            "node_id": node_id,
            "role": role,
            "domain": domain,
            "capabilities": caps,
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_register_topic, json.dumps(payload))
        _fed_dbg_main(
            f"evt=FED_BOOTSTRAP_REGISTER topic={fed_bootstrap_register_topic} gateway={gateway_id} "
            f"node={node_id} role={role} kind={member.get('kind')} caps={len(caps)}"
        )

    def _fed_bootstrap_publish_heartbeat(member: Dict[str, Any]) -> None:
        gateway_id = str(member.get("gateway_id", "") or "")
        node_id = str(member.get("node_id", "") or "")
        payload = {
            "schema": "federation.membership.v1",
            "event": "heartbeat",
            "gateway_id": gateway_id,
            "node_id": node_id,
            "status": "alive",
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_heartbeat_topic, json.dumps(payload))
        _fed_dbg_main(
            f"evt=FED_BOOTSTRAP_HEARTBEAT topic={fed_bootstrap_heartbeat_topic} gateway={gateway_id} node={node_id}"
        )

    def _fed_bootstrap_publish_catalog(member: Dict[str, Any]) -> None:
        gateway_id = str(member.get("gateway_id", "") or "")
        node_id = str(member.get("node_id", "") or "")
        role = str(member.get("role", "") or "")
        services = _fed_bootstrap_service_entries(member)
        dt_profile = _fed_bootstrap_dt_profile(member)
        payload = {
            "schema": "federation.catalog.v1",
            "request_id": f"realworld-cat-{gateway_id}-{int(time.time() * 1000)}",
            "gateway_id": gateway_id,
            "node_id": node_id,
            "role": role,
            "services": services,
            "dt_profile": dt_profile,
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_catalog_topic, json.dumps(payload))
        _fed_dbg_main(
            f"evt=FED_BOOTSTRAP_CATALOG topic={fed_bootstrap_catalog_topic} gateway={gateway_id} "
            f"node={node_id} role={role} kind={member.get('kind')} services={len(services)} "
            f"profile_if={dt_profile.get('interface_version', '-')}"
        )

    def _fed_bootstrap_publish_register_all(force: bool = False) -> int:
        n_pub = 0
        for member in _fed_bootstrap_member_defs():
            gid = str(member.get("gateway_id", "") or "")
            if not gid:
                continue
            if (not force) and (gid in fed_bootstrap_registered_gateways):
                continue
            _fed_bootstrap_publish_register(member)
            fed_bootstrap_registered_gateways.add(gid)
            n_pub += 1
        return int(n_pub)

    def _fed_bootstrap_publish_heartbeat_all() -> int:
        n_pub = 0
        for member in _fed_bootstrap_member_defs():
            gid = str(member.get("gateway_id", "") or "")
            if not gid:
                continue
            _fed_bootstrap_publish_heartbeat(member)
            n_pub += 1
        return int(n_pub)

    def _fed_bootstrap_publish_catalog_all() -> int:
        n_pub = 0
        for member in _fed_bootstrap_member_defs():
            gid = str(member.get("gateway_id", "") or "")
            if not gid:
                continue
            _fed_bootstrap_publish_catalog(member)
            n_pub += 1
        return int(n_pub)

    def _fed_bootstrap_publish_probe() -> None:
        nonlocal fed_bootstrap_probe_counter
        fed_bootstrap_probe_counter += 1
        req_id = f"realworld-dq-{fed_bootstrap_probe_counter:06d}"
        requester_node = (
            fed_bootstrap_node_id
            if str(fed_bootstrap_participants_mode) in ("hub", "hybrid")
            else str(fed_bootstrap_ev_id)
        )
        reply_topic = f"{fed_bootstrap_discovery_reply_prefix}/{requester_node}"
        payload = {
            "schema": "federation.discovery.v1",
            "request_id": req_id,
            "query_id": req_id,
            "trace_id": req_id,
            "requester": requester_node,
            "requester_dt_id": requester_node,
            "requester_role": "orchestrator",
            "source_service": "real_world",
            "purpose": "bootstrap_discovery_probe",
            "reply_topic": reply_topic,
            "filters": {
                "event_type": fed_bootstrap_discovery_event_filter,
                "result_mode": "dt",
            },
            "max_results": 64,
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_discovery_query_topic, json.dumps(payload))
        _fed_dbg_main(
            f"evt=FED_BOOTSTRAP_DISCOVERY_QUERY topic={fed_bootstrap_discovery_query_topic} "
            f"reply={reply_topic} req_id={req_id} event_filter={fed_bootstrap_discovery_event_filter or '-'}"
        )

    def on_message(_client, _userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd_queue.append((msg.topic, payload))
            if str(msg.topic).startswith("federation/"):
                _fed_dbg_main(
                    f"evt=RX_ENQUEUE topic={msg.topic} req_id={payload.get('req_id')} "
                    f"from={payload.get('from_tls')} to={payload.get('to_tls')}"
                )
        except Exception as e:
            print("Bad command payload:", e, msg.topic)

    client.on_message = on_message
    client.subscribe("cmd/#")
    client.subscribe("federation/#")
    if ev_request_delivery_mode in ("mqtt", "both") or bool(ev_http_adapter_enabled):
        client.subscribe(f"{ev_request_topic_prefix}/+")
    client.loop_start()
    if fed_bootstrap_enabled:
        try:
            n_reg = _fed_bootstrap_publish_register_all(force=True)
            n_cat = _fed_bootstrap_publish_catalog_all()
            n_hb = _fed_bootstrap_publish_heartbeat_all()
            fed_bootstrap_last_heartbeat_wall = float(time.time())
            fed_bootstrap_last_catalog_wall = float(time.time())
            fed_bootstrap_last_probe_wall = float(time.time())
            _fed_dbg_main(
                f"evt=FED_BOOTSTRAP_STARTUP participants_mode={fed_bootstrap_participants_mode} "
                f"registered_n={n_reg} catalog_n={n_cat} heartbeat_n={n_hb}"
            )
            if fed_bootstrap_discovery_probe_sec > 0.0:
                _fed_bootstrap_publish_probe()
        except Exception as e:
            _fed_dbg_main(f"evt=FED_BOOTSTRAP_ERR phase=startup err={type(e).__name__}:{e}")


    #shadow = None
    #shadow = ShadowRollout(args.shadow_sumo_bin, args.sumo_cfg, args.step_length) if args.enable_shadow else None
    
    shadow_pool = None
    shadow_throttle = ShadowThrottle(args.shadow_period)
    
    if args.shadow:
        shadow_cfg = ShadowRolloutPoolConfig(
            sumo_binary=args.shadow_sumo_bin,
            sumo_cfg=args.sumo_cfg,
            step_length=args.step_length,
            num_workers=args.shadow_workers,
            base_port=args.shadow_base_port,
            w_ev=args.shadow_w_ev,
            w_queue=args.shadow_w_queue,
        )
        shadow_pool = ShadowRolloutPool(shadow_cfg)
        shadow_pool.start()
        if hasattr(shadow_pool, "has_live_workers") and not shadow_pool.has_live_workers():
            print("[main] Shadow rollout workers unavailable; using analytical offer selection only.")
            shadow_pool = None

    b1_worker_pool = None
    b1_worker_pending: Dict[str, object] = {}
    if bool(getattr(args, "b1_worker_prototype", False)):
        try:
            mp_ctx = mp.get_context("spawn")
            b1_worker_pool = mp_ctx.Pool(processes=max(1, int(getattr(args, "b1_worker_prototype_processes", 1))))
            print(f"[B1-WORKER-PROTO] enabled workers={max(1, int(getattr(args, 'b1_worker_prototype_processes', 1)))}")
        except Exception as e:
            print(f"[B1-WORKER-PROTO][WARN] failed to start worker pool: {e}")
            b1_worker_pool = None


    subscribed_veh = set()
    step = 0
    periodic_last_t: Dict[str, float] = {}
    perf_acc: Dict[str, float] = defaultdict(float)
    perf_steps_acc = 0

    ev_id = args.emergency_veh
    erl_level = int(args.erl_level)
    ev_profile = EmergencyVehicleProfile(
        ev_id=str(ev_id),
        unit_id=str(args.ev_unit_id),
        description=str(args.ev_description),
        agency=str(args.ev_agency),
        erl_level=int(erl_level),
        metadata={"evaluation": str(CURRENT_EVALUATION)},
    )
    ev_agent = EmergencyVehicleAgent(profile=ev_profile, default_delta_sec=2.0)
    ev_local_state_mqtt_enable = bool(getattr(args, "ev_local_state_mqtt_enable", True))
    ev_http_state_lock = threading.Lock()
    ev_http_state_cache: Dict[str, Any] = {
        "ready": False,
        "payload": {},
        "last_update_wall": 0.0,
    }
    ev_http_state_server: Optional[ThreadingHTTPServer] = None
    _fed_dbg_main(
        f"evt=EV_REQ_DELIVERY mode={ev_request_delivery_mode} topic_prefix={ev_request_topic_prefix}"
    )
    _fed_dbg_main(
        f"evt=EV_REQ_INTERNAL enabled={1 if internal_ev_request_enabled else 0}"
    )
    _fed_dbg_main(
        f"evt=EV_HTTP_ADAPTER enabled={1 if ev_http_adapter_enabled else 0} url={ev_http_state_url or '-'} "
        f"poll_sec={float(ev_http_poll_sec):.2f} timeout_sec={float(ev_http_timeout_sec):.2f} "
        f"headers_n={len(ev_http_headers)}"
    )
    _fed_dbg_main(
        f"evt=EV_HTTP_STATE_SERVER enabled={1 if ev_http_state_server_enabled else 0} "
        f"host={ev_http_state_server_host} port={ev_http_state_server_port} "
        f"verbose={1 if ev_http_state_server_verbose else 0}"
    )
    _fed_dbg_main(
        f"evt=EV_LOCAL_STATE_MQTT enabled={1 if ev_local_state_mqtt_enable else 0}"
    )

    if ev_http_state_server_enabled:
        state_paths = {
            "/state",
            "/vehicle_agent/state",
            f"/vehicle_agent/{ev_id}/state",
        }

        class _EVStateHandler(BaseHTTPRequestHandler):
            def _json(self, code: int, payload: Dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
                self.send_response(int(code))
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                p = urlparse(self.path).path
                if p in ("/health", "/healthz"):
                    self._json(
                        200,
                        {
                            "ok": True,
                            "service": "real_world.vehicle_agent.http_state",
                            "evId": str(ev_id),
                            "tsWall": float(time.time()),
                        },
                    )
                    return
                if p in state_paths:
                    with ev_http_state_lock:
                        ready = bool(ev_http_state_cache.get("ready", False))
                        payload = dict(ev_http_state_cache.get("payload", {}) or {})
                    if not ready:
                        self._json(503, {"ok": False, "error": "state_not_ready"})
                        return
                    if ev_http_state_server_verbose:
                        try:
                            snap = dict(payload.get("snapshot", {}) or {})
                            print(
                                "[ev-http-state] "
                                f"path={p} sim={float(payload.get('simTime', -1.0)):.2f} "
                                f"ev={payload.get('evId', '-') } edge={snap.get('edgeId', '-') } "
                                f"speed={float(snap.get('speedMps', 0.0)):.2f} "
                                f"next_tls_n={len(list(snap.get('nextTls', []) or []))}"
                            )
                        except Exception:
                            pass
                    self._json(200, payload)
                    return
                self._json(404, {"ok": False, "error": "not_found"})

            def log_message(self, _fmt: str, *_args: object) -> None:
                return

        class _ReusableEVHTTPServer(ThreadingHTTPServer):
            # Allow immediate re-bind across sequential matrix mode runs.
            allow_reuse_address = True

        try:
            ev_http_state_server = _ReusableEVHTTPServer(
                (str(ev_http_state_server_host), int(ev_http_state_server_port)),
                _EVStateHandler,
            )
            threading.Thread(
                target=ev_http_state_server.serve_forever,
                name="ev-http-state-server",
                daemon=True,
            ).start()
            _fed_dbg_main(
                f"evt=EV_HTTP_STATE_SERVER_START ok=1 "
                f"url=http://{ev_http_state_server_host}:{ev_http_state_server_port}/vehicle_agent/{ev_id}/state"
            )
        except Exception as e:
            ev_http_state_server = None
            _fed_dbg_main(
                f"evt=EV_HTTP_STATE_SERVER_START ok=0 err={type(e).__name__}:{e}"
            )

    if ev_local_state_mqtt_enable:
        client.publish("rw/vehicle_agent/profile", json.dumps(ev_profile.to_dict()))
    client.publish(
        "rw/evaluation/state",
        json.dumps({
            "evaluation": str(CURRENT_EVALUATION),
            "allowed": list(MODE_EVALUATION),
            "source": "startup",
        }),
    )
    ers_agent = None
    if bool(args.enable_ers):
        ers_agent = EmergencyResponseSystemAgent(
            ERSConfig(system_id="ers_main")
        )
        ers_agent.register_vehicle(ev_profile.to_dict())
        client.publish("rw/ers/vehicle_registered", json.dumps(ev_profile.to_dict()))

    ev_last_external_edge = None
    handoff_sent = set()
    last_route_apply_sim_by_ev: Dict[str, float] = {}
    ev_blocked_since_by_ev: Dict[str, Optional[float]] = {}
    ev_blocked_contig_sec_by_ev: Dict[str, float] = {}
    ev_stuck_active_by_ev: Dict[str, bool] = {}
    route_apply_modes_allowed = {str(x).strip().lower() for x in str(getattr(args, "route_apply_in_modes", "advisory,arbitration") or "").split(",") if str(x).strip()}
    if not route_apply_modes_allowed:
        route_apply_modes_allowed = {"advisory", "arbitration"}
    print(f"[GTCO-ROUTE-APPLY] allowed_modes={sorted(route_apply_modes_allowed)}")
    ev_trigger_diag_last: Optional[Tuple[str, str, str]] = None
    b1_map_diag_seen: set[Tuple[str, str]] = set()
    ev_kpi_last_checkpoint: Optional[Tuple[str, str]] = None
    ev_kpi_stats: Dict[str, Any] = {
        "samples": 0,
        "speed_sum": 0.0,
        "speed_min": None,
        "speed_max": None,
        "stop_time_sec": 0.0,
        "slow_time_sec": 0.0,
        "near_stopline_time_sec": 0.0,
        "blocked_near_stopline_time_sec": 0.0,
        "first_t": None,
        "last_t": None,
        "last_sample_t": None,
        "last_speed": None,
        "last_d_stop": None,
    }
    ev_kpi_samples: List[Dict[str, Any]] = []
    ev_kpi_checkpoints: List[Dict[str, Any]] = []
    ev_req_pipeline_stats: Dict[str, int] = {
        "rx_total": 0,
        "dispatch_ok": 0,
        "drop_no_agent": 0,
        "drop_replay_both_mode": 0,
        "drop_missing_ev_id": 0,
        "drop_parse_err": 0,
    }
    ev_req_pipeline_last: Dict[str, Any] = {}
    terminate_on_ev_finish = bool(getattr(args, "terminate_on_ev_finish", False))
    ev_seen_once = False

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            t_iter_wall = time.perf_counter()

            # apply pending commands (DT/middleware -> SUMO)
            t_cmd_wall = time.perf_counter()
            while cmd_queue:
                topic, payload = cmd_queue.popleft()
                try:
                    if topic.startswith("cmd/vehicle/") and topic.endswith("/setRoute"):
                        vid = topic.split("/")[2]
                        traci.vehicle.setRoute(vid, payload["route"])
                    elif topic.startswith("cmd/vehicle/") and topic.endswith("/setSpeed"):
                        vid = topic.split("/")[2]
                        traci.vehicle.setSpeed(vid, float(payload["speed"]))
                    elif topic.startswith("cmd/tls/") and topic.endswith("/setPhase"):
                        tls_id = topic.split("/")[2]
                        traci.trafficlight.setPhase(tls_id, int(payload["phase"]))
                    elif topic.startswith("cmd/tls/") and topic.endswith("/setProgram"):
                        tls_id = topic.split("/")[2]
                        traci.trafficlight.setProgram(tls_id, str(payload["program"]))
                    elif topic.startswith("cmd/tls/") and topic.endswith("/setPhaseDuration"):
                        tls_id = topic.split("/")[2]
                        traci.trafficlight.setPhaseDuration(tls_id, float(payload["duration"]))
                    elif topic.startswith("cmd/tls/") and topic.endswith("/setState"):
                        tls_id = topic.split("/")[2]
                        traci.trafficlight.setRedYellowGreenState(tls_id, str(payload["state"]))
                    elif topic in ("cmd/sim/setEvaluation", "cmd/evaluation/set", "cmd/evaluation"):
                        req_eval = str(
                            payload.get("evaluation")
                            or payload.get("mode")
                            or payload.get("value")
                            or ""
                        ).upper()
                        if req_eval in MODE_EVALUATION:
                            prev_eval = str(CURRENT_EVALUATION)
                            CURRENT_EVALUATION = str(req_eval)
                            try:
                                ev_profile.metadata["evaluation"] = str(CURRENT_EVALUATION)
                            except Exception:
                                pass
                            for _ag in agents.values():
                                try:
                                    _ag.cfg.decision_log_run_label = str(CURRENT_EVALUATION)
                                except Exception:
                                    pass
                            try:
                                t_eval = float(traci.simulation.getTime())
                            except Exception:
                                t_eval = -1.0
                            msg_eval = {
                                "evaluation": str(CURRENT_EVALUATION),
                                "previous": prev_eval,
                                "allowed": list(MODE_EVALUATION),
                                "simTime": float(t_eval),
                                "step": int(step),
                                "source": "mqtt",
                                "topic": str(topic),
                            }
                            print(f"[EVAL] switched {prev_eval} -> {CURRENT_EVALUATION} via {topic}")
                            client.publish("rw/evaluation/state", json.dumps(msg_eval))
                        else:
                            print(f"[EVAL][WARN] invalid evaluation request '{req_eval}' on topic {topic}")
                            client.publish(
                                "rw/evaluation/error",
                                json.dumps({
                                    "requested": req_eval,
                                    "allowed": list(MODE_EVALUATION),
                                    "topic": str(topic),
                                }),
                            )

                    elif topic.startswith("federation/reservation/req/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="federation_reservation_req", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=req dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} req_id={payload.get('req_id')}"
                        )
                        if ag_dst is not None:
                            try:
                                resp = ag_dst.on_reservation_req(payload)
                            except Exception as e:
                                _fed_dbg_main(
                                    f"evt=REQ_HANDLER_ERR topic={topic} dst={dst_tls} req_id={payload.get('req_id')} "
                                    f"err={type(e).__name__}:{e}"
                                )
                                to_tls_err = str(payload.get("from_tls", "") or "")
                                if to_tls_err:
                                    try:
                                        err_t = float(traci.simulation.getTime())
                                    except Exception:
                                        err_t = -1.0
                                    err_resp = {
                                        "req_id": str(payload.get("req_id", "")),
                                        "ev_id": str(payload.get("ev_id", "")),
                                        "from_tls": str(dst_tls),
                                        "to_tls": to_tls_err,
                                        "status": "ERROR",
                                        "reason": f"req_handler_exception:{type(e).__name__}",
                                        "mode": str(payload.get("mode", "")),
                                        "sim_time": float(err_t),
                                    }
                                    _fed_dbg_main(
                                        f"evt=PUB topic=federation/reservation/resp/{to_tls_err} req_id={err_resp.get('req_id')} "
                                        f"from={dst_tls} to={to_tls_err} status=ERROR"
                                    )
                                    client.publish(
                                        f"federation/reservation/resp/{to_tls_err}",
                                        json.dumps(err_resp),
                                    )
                                continue
                            to_tls = resp.get("to_tls", "")
                            if to_tls:
                                _fed_dbg_main(
                                    f"evt=PUB topic=federation/reservation/resp/{to_tls} req_id={resp.get('req_id')} "
                                    f"from={resp.get('from_tls')} to={to_tls} status={resp.get('status')}"
                                )
                                client.publish(f"federation/reservation/resp/{to_tls}", json.dumps(resp))

                    elif topic.startswith("federation/reservation/resp/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="federation_reservation_resp", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=resp dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} req_id={payload.get('req_id')}"
                        )
                        if ag_dst is not None:
                            ag_dst.on_reservation_resp(payload)

                    elif topic.startswith("federation/handoff/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="federation_handoff", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=handoff dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} ev={payload.get('ev_id')}"
                        )
                        if ag_dst is not None:
                            ag_dst.on_handoff(payload)

                    elif topic.startswith("federation/corridor/advice/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="corridor_advice", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=corridor_advice dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} ev={payload.get('ev_id')}"
                        )
                        if ag_dst is not None:
                            ag_dst.on_corridor_advice(payload)

                    elif topic.startswith("federation/corridor/verdict/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="corridor_verdict", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=corridor_verdict dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} req_id={payload.get('req_id')}"
                        )
                        if ag_dst is not None:
                            ag_dst.on_corridor_verdict(payload)

                    elif topic.startswith(f"{ev_request_topic_prefix}/"):
                        dst_tls = topic.split("/")[-1]
                        ev_req_pipeline_stats["rx_total"] = int(ev_req_pipeline_stats.get("rx_total", 0)) + 1
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="ev_request_mqtt", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=ev_request dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} ev={payload.get('ev_id') or payload.get('evId')}"
                        )
                        ev_req_payload = dict(payload or {})
                        if ag_dst is None:
                            ev_req_pipeline_stats["drop_no_agent"] = int(ev_req_pipeline_stats.get("drop_no_agent", 0)) + 1
                            _fed_evt_main(
                                "ev.request.drop",
                                role="ev",
                                reason="agent_not_found",
                                tls_id=str(dst_tls),
                                ev_id=str(ev_req_payload.get("ev_id", ev_req_payload.get("evId", "")) or ""),
                                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                            )
                            continue
                        if ev_request_delivery_mode == "both" and str(ev_req_payload.get("source_service", "")) == "vehicle_agent":
                            # In dual mode, direct dispatch already happened in the EV control path.
                            # Skip replaying local vehicle-agent requests from MQTT to avoid duplicate intake.
                            ev_req_pipeline_stats["drop_replay_both_mode"] = int(ev_req_pipeline_stats.get("drop_replay_both_mode", 0)) + 1
                            continue
                        if isinstance(ev_req_payload.get("ev_request"), dict):
                            ev_req_payload = dict(ev_req_payload.get("ev_request") or {})
                        # Support both snake_case and camelCase fields.
                        if "ev_id" not in ev_req_payload and "evId" in ev_req_payload:
                            ev_req_payload["ev_id"] = ev_req_payload.get("evId")
                        if "sim_time" not in ev_req_payload and "simTime" in ev_req_payload:
                            ev_req_payload["sim_time"] = ev_req_payload.get("simTime")
                        if "erl_level" not in ev_req_payload and "erlLevel" in ev_req_payload:
                            ev_req_payload["erl_level"] = ev_req_payload.get("erlLevel")
                        if "speed_mps" not in ev_req_payload and "speedMps" in ev_req_payload:
                            ev_req_payload["speed_mps"] = ev_req_payload.get("speedMps")
                        if "distance_to_intersection_m" not in ev_req_payload and "distanceToIntersectionM" in ev_req_payload:
                            ev_req_payload["distance_to_intersection_m"] = ev_req_payload.get("distanceToIntersectionM")
                        if "in_edge_id" not in ev_req_payload and "inEdgeId" in ev_req_payload:
                            ev_req_payload["in_edge_id"] = ev_req_payload.get("inEdgeId")
                        if "target_phase_idx" not in ev_req_payload and "targetPhaseIdx" in ev_req_payload:
                            ev_req_payload["target_phase_idx"] = ev_req_payload.get("targetPhaseIdx")
                        if "delta_sec" not in ev_req_payload and "deltaSec" in ev_req_payload:
                            ev_req_payload["delta_sec"] = ev_req_payload.get("deltaSec")
                        if "route_intersections" not in ev_req_payload and "routeIntersections" in ev_req_payload:
                            ev_req_payload["route_intersections"] = ev_req_payload.get("routeIntersections")
                        if "route_veh" not in ev_req_payload and "routeVeh" in ev_req_payload:
                            ev_req_payload["route_veh"] = ev_req_payload.get("routeVeh")
                        ev_id_in = str(ev_req_payload.get("ev_id", "") or "")
                        in_edge_in = str(ev_req_payload.get("in_edge_id", "") or "")
                        # Normalize inbound edge against destination TLS mapping.
                        # Keeps adapter-fed EV requests compatible with local phase maps.
                        try:
                            inbound_map = dict(getattr(ag_dst, "_inbound_edge_to_phase", {}) or {})
                        except Exception:
                            inbound_map = {}
                        if inbound_map:
                            infer_edge = ""
                            infer_src = ""
                            if in_edge_in and (inbound_map.get(in_edge_in) is not None):
                                infer_edge = str(in_edge_in)
                                infer_src = "payload"
                            if not infer_edge:
                                for _e in [str(x) for x in list(ev_req_payload.get("route_veh", []) or []) if str(x)]:
                                    if inbound_map.get(str(_e)) is not None:
                                        infer_edge = str(_e)
                                        infer_src = "route_veh"
                                        break
                            if (not infer_edge) and ev_id_in and (traci is not None):
                                try:
                                    live_route = [str(e) for e in list(traci.vehicle.getRoute(str(ev_id_in)) or []) if str(e)]
                                    ridx = int(traci.vehicle.getRouteIndex(str(ev_id_in)))
                                except Exception:
                                    live_route = []
                                    ridx = -1
                                if live_route:
                                    start = max(0, ridx) if ridx >= 0 else 0
                                    for _e in live_route[start : min(len(live_route), start + 10)]:
                                        if inbound_map.get(str(_e)) is not None:
                                            infer_edge = str(_e)
                                            infer_src = "traci_route"
                                            break
                            if infer_edge and infer_edge != in_edge_in:
                                ev_req_payload["in_edge_id"] = str(infer_edge)
                                _fed_dbg_main(
                                    f"evt=EV_REQ_IN_EDGE_INFER ev={ev_id_in or '-'} tls={dst_tls} "
                                    f"source={infer_src or '-'} in_edge_src={in_edge_in or '-'} in_edge={infer_edge}"
                                )
                                _fed_evt_main(
                                    "ev.request.in_edge.infer",
                                    role="ev",
                                    source_service="real_world",
                                    ev_id=str(ev_id_in),
                                    tls_id=str(dst_tls),
                                    infer_source=str(infer_src or ""),
                                    in_edge_src=str(in_edge_in or ""),
                                    in_edge_resolved=str(infer_edge),
                                )
                        if not ev_id_in:
                            ev_req_pipeline_stats["drop_missing_ev_id"] = int(ev_req_pipeline_stats.get("drop_missing_ev_id", 0)) + 1
                            _fed_evt_main(
                                "ev.request.drop",
                                role="ev",
                                reason="missing_ev_id",
                                tls_id=str(dst_tls),
                                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                            )
                            continue
                        try:
                            _sim_now_for_req = float(traci.simulation.getTime())
                        except Exception:
                            _sim_now_for_req = -1.0
                        try:
                            ev_msg_in = EvRequest(
                                ev_id=str(ev_req_payload.get("ev_id", "")),
                                sim_time=float(ev_req_payload.get("sim_time", _sim_now_for_req)),
                                erl_level=int(ev_req_payload.get("erl_level", erl_level)),
                                speed_mps=float(ev_req_payload.get("speed_mps", 0.0)),
                                distance_to_intersection_m=float(ev_req_payload.get("distance_to_intersection_m", 1e9)),
                                in_edge_id=str(ev_req_payload.get("in_edge_id", "")),
                                target_phase_idx=ev_req_payload.get("target_phase_idx"),
                                delta_sec=float(ev_req_payload.get("delta_sec", 2.0)),
                                route_intersections=list(ev_req_payload.get("route_intersections", []) or []) or None,
                                route_veh=list(ev_req_payload.get("route_veh", []) or []) or None,
                                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                                source_tag=str(ev_req_payload.get("source_tag", ev_request_source_tag) or ""),
                                delivery="mqtt",
                            )
                        except Exception as e:
                            ev_req_pipeline_stats["drop_parse_err"] = int(ev_req_pipeline_stats.get("drop_parse_err", 0)) + 1
                            _fed_evt_main(
                                "ev.request.drop",
                                role="ev",
                                reason=f"parse_error:{type(e).__name__}",
                                tls_id=str(dst_tls),
                                ev_id=str(ev_id_in),
                                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                            )
                            continue

                        ag_dst.receive_ev_message(ev_msg_in)
                        ev_req_pipeline_stats["dispatch_ok"] = int(ev_req_pipeline_stats.get("dispatch_ok", 0)) + 1
                        age_ms = max(0.0, (float(_sim_now_for_req) - float(ev_msg_in.sim_time)) * 1000.0) if _sim_now_for_req >= 0 else -1.0
                        ev_req_pipeline_last = {
                            "ev_id": str(ev_msg_in.ev_id),
                            "tls_id": str(dst_tls),
                            "source": str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                            "source_tag": str(ev_req_payload.get("source_tag", ev_request_source_tag) or ""),
                            "distance_m": float(ev_msg_in.distance_to_intersection_m),
                            "speed_mps": float(ev_msg_in.speed_mps),
                            "in_edge_id": str(ev_msg_in.in_edge_id),
                            "age_ms": float(age_ms),
                        }
                        _fed_dbg_main(
                            f"evt=EV_REQ_PIPE ev={ev_msg_in.ev_id} tls={dst_tls} src={ev_req_pipeline_last.get('source')} "
                            f"src_tag={ev_req_pipeline_last.get('source_tag') or '-'} "
                            f"dist={float(ev_msg_in.distance_to_intersection_m):.2f} speed={float(ev_msg_in.speed_mps):.2f} "
                            f"edge={ev_msg_in.in_edge_id or '-'} age_ms={float(age_ms):.1f}"
                        )
                        _fed_evt_main(
                            "ev.request.dispatched",
                            role="ev",
                            ev_id=str(ev_msg_in.ev_id),
                            tls_id=str(dst_tls),
                            in_edge_id=str(ev_msg_in.in_edge_id),
                            distance_to_intersection_m=float(ev_msg_in.distance_to_intersection_m),
                            delivery="mqtt",
                            ev_request_source=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                            ev_request_source_tag=str(ev_req_payload.get("source_tag", ev_request_source_tag) or ""),
                            age_ms=float(age_ms),
                        )

                    elif topic.startswith("federation/corridor/state/"):
                        corridor_id = topic.split("/")[-1]
                        corridor_state_cache[str(corridor_id)] = dict(payload or {})

                    elif topic.startswith("federation/corridor/route_advice/"):
                        advised_ev_id = topic.split("/")[-1]
                        route_adv_payload = dict(payload or {})
                        corridor_route_advice_by_ev[str(advised_ev_id)] = route_adv_payload
                        route_opt = dict(route_adv_payload.get("route_optimization", {}) or {})
                        if on_demand_agent_activation and str(advised_ev_id) == str(ev_id):
                            cand_tls = _route_advice_tls_candidates(route_adv_payload)
                            if cand_tls:
                                n_new = _activate_tls_agents(
                                    list(cand_tls),
                                    reason="corridor_route_advice",
                                    sim_time=float(step),
                                    max_new=on_demand_max_new_per_tick,
                                )
                                _fed_dbg_main(
                                    f"evt=AGENT_ACTIVATE_HINT source=corridor_route_advice ev={advised_ev_id} "
                                    f"cand_tls_n={len(cand_tls)} activated_n={n_new}"
                                )
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=corridor_route_advice ev={advised_ev_id} "
                            f"mode={route_adv_payload.get('mode', '-')} coordinator_mode={route_adv_payload.get('coordinator_mode', '-')} "
                            f"reroute={int(bool(route_opt.get('should_advise_reroute', False)))}"
                        )
                        _fed_evt_main(
                            "corridor.route_advice.received",
                            role="ev",
                            ev_id=str(advised_ev_id),
                            topic=str(topic),
                            mode=str(route_adv_payload.get("mode", "") or ""),
                            coordinator_mode=str(route_adv_payload.get("coordinator_mode", "") or ""),
                            should_advise_reroute=bool(route_opt.get("should_advise_reroute", False)),
                            improvement_sec=float(route_opt.get("improvement_sec", 0.0) or 0.0),
                            improvement_ratio=float(route_opt.get("improvement_ratio", 0.0) or 0.0),
                        )
                        try:
                            client.publish(f"rw/vehicle/{advised_ev_id}/route_advice", json.dumps(route_adv_payload))
                            _fed_dbg_main(f"evt=PUB topic=rw/vehicle/{advised_ev_id}/route_advice ev={advised_ev_id}")
                        except Exception:
                            pass
                        try:
                            client.publish(f"rw/vehicle_agent/{advised_ev_id}/route_advice", json.dumps(route_adv_payload))
                            _fed_dbg_main(f"evt=PUB topic=rw/vehicle_agent/{advised_ev_id}/route_advice ev={advised_ev_id}")
                        except Exception:
                            pass

                    elif topic.startswith(fed_bootstrap_discovery_reply_prefix + "/"):
                        req_id = str(payload.get("request_id", "") or "")
                        n_results = int(payload.get("n_results", 0) or 0)
                        _fed_dbg_main(
                            f"evt=FED_BOOTSTRAP_DISCOVERY_RESP topic={topic} req_id={req_id} "
                            f"n_results={n_results}"
                        )


                except Exception as e:
                    _fed_dbg_main(f"evt=RX_DISPATCH_ERROR topic={topic} err={type(e).__name__}:{e}")
                    print("Failed applying command:", topic, e)

            perf_acc["cmd_dispatch"] += float(time.perf_counter() - t_cmd_wall)

            # step physics
            t_simstep_wall = time.perf_counter()
            traci.simulationStep()
            perf_acc["sim_step"] += float(time.perf_counter() - t_simstep_wall)
            sim_time = float(traci.simulation.getTime())
            if terminate_on_ev_finish:
                try:
                    veh_ids_now = traci.vehicle.getIDList()
                except Exception:
                    veh_ids_now = []
                try:
                    arrived_ids = traci.simulation.getArrivedIDList()
                except Exception:
                    arrived_ids = []
                if str(ev_id) in set(veh_ids_now):
                    ev_seen_once = True
                if str(ev_id) in set(arrived_ids):
                    _fed_dbg_main(
                        f"evt=EV_FINISH_EARLY_STOP ev={ev_id} sim={sim_time:.2f} step={step} seen_once={int(ev_seen_once)}"
                    )
                    _fed_evt_main(
                        "ev.finish.early_stop",
                        role="ev",
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        step=int(step),
                        seen_once=bool(ev_seen_once),
                    )
                    try:
                        client.publish(
                            "rw/ev/finish",
                            json.dumps(
                                {
                                    "evId": str(ev_id),
                                    "simTime": float(sim_time),
                                    "step": int(step),
                                    "source": "real_world",
                                    "reason": "terminate_on_ev_finish",
                                }
                            ),
                        )
                    except Exception:
                        pass
                    break
            if ev_http_adapter_enabled and ev_http_state_url:
                if (float(sim_time) - float(ev_http_last_poll_sim)) >= float(ev_http_poll_sec):
                    ev_http_last_poll_sim = float(sim_time)
                    t_http0 = time.perf_counter()
                    state_http = _ev_http_fetch_state()
                    wall_ms_http = float((time.perf_counter() - t_http0) * 1000.0)
                    if state_http is None:
                        err_key = "fetch_or_parse_failed"
                        if err_key != ev_http_last_err_key:
                            ev_http_last_err_key = err_key
                            _fed_dbg_main(
                                f"evt=EV_HTTP_ADAPTER_ERR ev={ev_id} url={ev_http_state_url} err={err_key}"
                            )
                    else:
                        ev_http_last_err_key = ""
                        if ev_http_publish_state_topic and ev_local_state_mqtt_enable:
                            try:
                                client.publish(
                                    ev_http_publish_state_topic,
                                    json.dumps({
                                        "simTime": float(sim_time),
                                        "evId": str(state_http.get("ev_id", state_http.get("evId", ev_id)) or ev_id),
                                        "source": "ev_http_adapter",
                                        "state": dict(state_http),
                                    }),
                                )
                            except Exception:
                                pass
                        reqs_http = _ev_http_state_to_requests(
                            state_obj=dict(state_http),
                            sim_time_now=float(sim_time),
                            default_ev_id=str(ev_id),
                            default_erl_level=int(erl_level),
                        )
                        n_pub = 0
                        for tls_http, req_http in reqs_http:
                            stls = str(tls_http or "")
                            if not stls:
                                continue
                            topic_http = f"{ev_request_topic_prefix}/{stls}"
                            try:
                                client.publish(topic_http, json.dumps(req_http))
                                n_pub += 1
                            except Exception:
                                continue
                        _fed_dbg_main(
                            f"evt=EV_HTTP_ADAPTER_POLL ev={ev_id} url={ev_http_state_url} ok=1 "
                            f"req_published_n={n_pub} wall_ms={wall_ms_http:.2f}"
                        )
                        _fed_evt_main(
                            "ev.http_adapter.poll",
                            role="ev",
                            ev_id=str(state_http.get("ev_id", state_http.get("evId", ev_id)) or ev_id),
                            status="ok",
                            url=str(ev_http_state_url),
                            req_published_n=int(n_pub),
                            wall_ms=float(wall_ms_http),
                        )
            if fed_bootstrap_enabled:
                now_wall = float(time.time())
                try:
                    # Register any newly appeared virtual participants (e.g., on-demand TLS activation).
                    _fed_bootstrap_publish_register_all(force=False)
                    if (now_wall - float(fed_bootstrap_last_heartbeat_wall)) >= float(fed_bootstrap_heartbeat_sec):
                        _fed_bootstrap_publish_heartbeat_all()
                        fed_bootstrap_last_heartbeat_wall = now_wall
                    if (now_wall - float(fed_bootstrap_last_catalog_wall)) >= float(fed_bootstrap_catalog_sec):
                        _fed_bootstrap_publish_catalog_all()
                        fed_bootstrap_last_catalog_wall = now_wall
                    if fed_bootstrap_discovery_probe_sec > 0.0 and (now_wall - float(fed_bootstrap_last_probe_wall)) >= float(fed_bootstrap_discovery_probe_sec):
                        _fed_bootstrap_publish_probe()
                        fed_bootstrap_last_probe_wall = now_wall
                except Exception as e:
                    _fed_dbg_main(f"evt=FED_BOOTSTRAP_ERR phase=tick err={type(e).__name__}:{e}")
            do_pub_step = _periodic_due(sim_time, periodic_last_t, "rw_step", float(getattr(args, "publish_step_period_sec", 1.0)))
            do_pub_vehicle = _periodic_due(sim_time, periodic_last_t, "veh_state", float(getattr(args, "publish_vehicle_period_sec", 1.0)))
            do_loop_sense = _periodic_due(sim_time, periodic_last_t, "loop_sense", float(getattr(args, "loop_sense_period_sec", 1.0)))
            do_pub_tls_state = _periodic_due(sim_time, periodic_last_t, "tls_state", float(getattr(args, "publish_tls_state_period_sec", 1.0)))
            do_pub_tls_lanes = _periodic_due(sim_time, periodic_last_t, "tls_lanes", float(getattr(args, "publish_tls_lanes_period_sec", -1.0)))
            do_print_step = _periodic_due(sim_time, periodic_last_t, "step_print", float(getattr(args, "print_period_sec", 1.0)))
            do_ev_kpi = _periodic_due(sim_time, periodic_last_t, "ev_kpi", float(getattr(args, "ev_kpi_log_period_sec", -1.0)))
            do_ev_pipeline_log = _periodic_due(sim_time, periodic_last_t, "ev_pipeline", float(ev_pipeline_log_period_sec))
            do_ev_state_trace = _periodic_due(sim_time, periodic_last_t, "ev_state_trace", float(ev_state_trace_period_sec))
            blocked_contig_sec = 0.0
            try:
                ev_speed_now = float(traci.vehicle.getSpeed(str(ev_id)))
                ev_edge_now = str(traci.vehicle.getRoadID(str(ev_id)) or "")
                d_stop_now = float(dist_to_stopline_on_current_lane(str(ev_id)))
                blocked_cond = (
                    bool(ev_edge_now)
                    and not str(ev_edge_now).startswith(":")
                    and float(ev_speed_now) <= float(getattr(args, "route_apply_stuck_speed_threshold", 0.5))
                    and float(d_stop_now) <= float(getattr(args, "route_apply_stuck_stopline_dist_m", 2.0))
                )
            except Exception:
                blocked_cond = False
                ev_speed_now = 0.0
                d_stop_now = 1e9
            if blocked_cond:
                t0 = ev_blocked_since_by_ev.get(str(ev_id))
                if t0 is None:
                    t0 = float(sim_time)
                    ev_blocked_since_by_ev[str(ev_id)] = float(t0)
                blocked_contig_sec = max(0.0, float(sim_time) - float(t0))
            else:
                ev_blocked_since_by_ev[str(ev_id)] = None
                blocked_contig_sec = 0.0
            ev_blocked_contig_sec_by_ev[str(ev_id)] = float(blocked_contig_sec)
            stuck_thr_sec = max(0.0, float(getattr(args, "route_apply_stuck_blocked_sec", 12.0)))
            stuck_now = bool(blocked_contig_sec >= stuck_thr_sec)
            stuck_prev = bool(ev_stuck_active_by_ev.get(str(ev_id), False))
            if stuck_now and not stuck_prev:
                _fed_dbg_main(
                    f"evt=EV_STUCK_ENTER ev={ev_id} edge={ev_edge_now or '-'} blocked_sec={blocked_contig_sec:.2f} "
                    f"speed={ev_speed_now:.2f} d_stop={d_stop_now:.2f}"
                )
                _fed_evt_main(
                    "ev.stuck.enter",
                    role="ev",
                    ev_id=str(ev_id),
                    edge=str(ev_edge_now or ""),
                    blocked_sec=float(blocked_contig_sec),
                    speed_mps=float(ev_speed_now),
                    d_stop_m=float(d_stop_now),
                )
                print(
                    "[GTCO-STUCK] "
                    f"sim={sim_time:.1f} ev={ev_id} edge={ev_edge_now or '-'} blocked={blocked_contig_sec:.1f}s "
                    f"speed={ev_speed_now:.2f} d_stop={d_stop_now:.2f}"
                )
            elif (not stuck_now) and stuck_prev:
                _fed_dbg_main(
                    f"evt=EV_STUCK_EXIT ev={ev_id} edge={ev_edge_now or '-'} blocked_sec={blocked_contig_sec:.2f} "
                    f"speed={ev_speed_now:.2f} d_stop={d_stop_now:.2f}"
                )
                _fed_evt_main(
                    "ev.stuck.exit",
                    role="ev",
                    ev_id=str(ev_id),
                    edge=str(ev_edge_now or ""),
                    blocked_sec=float(blocked_contig_sec),
                    speed_mps=float(ev_speed_now),
                    d_stop_m=float(d_stop_now),
                )
            ev_stuck_active_by_ev[str(ev_id)] = bool(stuck_now)
            if do_pub_step:
                client.publish("rw/step", json.dumps({"step": step, "simTime": sim_time}))
            sim_sec_int = int(sim_time)
            if sim_sec_int != last_graph_cost_print_sec:
                last_graph_cost_print_sec = sim_sec_int
                for corridor_id, cstate in list(corridor_state_cache.items()):
                    missions = list(cstate.get("missions", []) or [])
                    tracked = next((m for m in missions if str(m.get("ev_id", "")) == str(ev_id)), None)
                    if tracked is None:
                        continue
                    gpc = dict(tracked.get("graph_path_cost", {}) or {})
                    route_advice = dict(corridor_route_advice_by_ev.get(str(ev_id), {}) or {})
                    route_opt = dict(route_advice.get("route_optimization", {}) or {})
                    print(
                        "[GTCO-GRAPH] "
                        f"corridor={corridor_id} sim={sim_time:.1f} ev={ev_id} "
                        f"curr_tls={tracked.get('current_tls') or '-'} "
                        f"next_tls={gpc.get('current_next_tls') or '-'} "
                        f"alt_next={gpc.get('optimized_next_tls') or '-'} "
                        f"cost={float(gpc.get('current_route_cost_sec', 0.0)):.1f}s "
                        f"alt_cost={float(gpc.get('optimized_route_cost_sec', 0.0)):.1f}s "
                        f"improve={float(gpc.get('improvement_sec', 0.0)):.1f}s "
                        f"reroute={int(bool(gpc.get('should_advise_reroute', False)))}"
                    )
                    if route_advice:
                        stuck_sec_now = float(ev_blocked_contig_sec_by_ev.get(str(ev_id), 0.0) or 0.0)
                        _fed_dbg_main(
                            f"evt=EV_ROUTE_ADVICE_SEEN ev={ev_id} corridor={route_advice.get('corridor_id', corridor_id)} "
                            f"mode={route_advice.get('mode', '-')} reroute={int(bool(route_opt.get('should_advise_reroute', False)))} "
                            f"improve_sec={float(route_opt.get('improvement_sec', 0.0) or 0.0):.2f} "
                            f"improve_ratio={float(route_opt.get('improvement_ratio', 0.0) or 0.0):.4f} "
                            f"blocked_contig_sec={stuck_sec_now:.2f}"
                        )
                        _fed_evt_main(
                            "corridor.route_advice.seen",
                            role="ev",
                            ev_id=str(ev_id),
                            corridor_id=str(route_advice.get("corridor_id", corridor_id)),
                            mode=str(route_advice.get("mode", "") or ""),
                            should_advise_reroute=bool(route_opt.get("should_advise_reroute", False)),
                            improvement_sec=float(route_opt.get("improvement_sec", 0.0) or 0.0),
                            improvement_ratio=float(route_opt.get("improvement_ratio", 0.0) or 0.0),
                            blocked_contig_sec=float(stuck_sec_now),
                        )
                        print(
                            "[GTCO-ROUTE-ADVICE] "
                            f"corridor={route_advice.get('corridor_id', corridor_id)} sim={sim_time:.1f} ev={ev_id} "
                            f"recommended_next={route_opt.get('optimized_next_tls') or '-'} "
                            f"destination_tls={route_opt.get('destination_tls') or '-'} "
                            f"ratio={float(route_opt.get('improvement_ratio', 0.0)):.3f} "
                            f"mode={route_advice.get('mode', '-')}"
                        )

                    if bool(getattr(args, "apply_corridor_route_advice", False)) and route_advice:
                        mode_lbl = str(route_advice.get("mode", "") or "")
                        coordinator_mode = str(route_advice.get("coordinator_mode", "") or "").strip().lower()
                        if not coordinator_mode:
                            coordinator_mode = "observe" if mode_lbl == "observation_only" else "advisory"
                        _fed_dbg_main(
                            f"evt=EV_ROUTE_APPLY_GATE ev={ev_id} enabled=1 mode_lbl={mode_lbl or '-'} "
                            f"coordinator_mode={coordinator_mode or '-'} allowed={int(coordinator_mode in route_apply_modes_allowed and mode_lbl != 'observation_only')}"
                        )
                        if coordinator_mode in route_apply_modes_allowed and mode_lbl != "observation_only":
                            reroute = bool(route_opt.get("should_advise_reroute", False))
                            improve_sec = float(route_opt.get("improvement_sec", 0.0) or 0.0)
                            improve_ratio = float(route_opt.get("improvement_ratio", 0.0) or 0.0)
                            blocked_contig = float(ev_blocked_contig_sec_by_ev.get(str(ev_id), 0.0) or 0.0)
                            stuck_override = bool(
                                blocked_contig >= max(0.0, float(getattr(args, "route_apply_stuck_blocked_sec", 12.0)))
                            )
                            req_min_sec = float(getattr(args, "route_apply_min_improvement_sec", 8.0))
                            req_min_ratio = float(getattr(args, "route_apply_min_improvement_ratio", 0.15))
                            if stuck_override:
                                req_min_sec = min(req_min_sec, float(getattr(args, "route_apply_stuck_min_improvement_sec", 0.0)))
                                req_min_ratio = min(req_min_ratio, float(getattr(args, "route_apply_stuck_min_improvement_ratio", 0.0)))
                            _fed_dbg_main(
                                f"evt=EV_ROUTE_APPLY_CHECK ev={ev_id} reroute={int(reroute)} "
                                f"improve_sec={improve_sec:.2f} improve_ratio={improve_ratio:.4f} "
                                f"th_sec={req_min_sec:.2f} th_ratio={req_min_ratio:.4f} "
                                f"stuck_override={int(stuck_override)} blocked_contig_sec={blocked_contig:.2f}"
                            )
                            if (reroute or stuck_override) and improve_sec >= req_min_sec and improve_ratio >= req_min_ratio:
                                last_apply = float(last_route_apply_sim_by_ev.get(str(ev_id), -1e9))
                                cooldown = max(0.0, float(getattr(args, "route_apply_cooldown_sec", 10.0)))
                                _fed_dbg_main(
                                    f"evt=EV_ROUTE_APPLY_COOLDOWN ev={ev_id} elapsed={(sim_time - last_apply):.2f} cooldown={cooldown:.2f} "
                                    f"ready={int((sim_time - last_apply) >= cooldown)}"
                                )
                                if (sim_time - last_apply) >= cooldown:
                                    try:
                                        curr_edge = str(traci.vehicle.getRoadID(str(ev_id)) or "")
                                    except Exception:
                                        curr_edge = ""
                                    _fed_dbg_main(
                                        f"evt=EV_ROUTE_CURR_EDGE ev={ev_id} edge={curr_edge or '-'} internal={int(bool(curr_edge and curr_edge.startswith(':')))}"
                                    )
                                    if curr_edge and not curr_edge.startswith(":"):
                                        try:
                                            lane_id = str(traci.vehicle.getLaneID(str(ev_id)) or "")
                                            lane_len = float(traci.lane.getLength(lane_id)) if lane_id else 0.0
                                            lane_pos = float(traci.vehicle.getLanePosition(str(ev_id)))
                                            rem_lane = max(0.0, lane_len - lane_pos)
                                        except Exception:
                                            rem_lane = 1e9
                                        min_rem = max(0.0, float(getattr(args, "route_apply_min_remaining_lane_m", 12.0)))
                                        _fed_dbg_main(
                                            f"evt=EV_ROUTE_LANE_GAP ev={ev_id} rem_lane={rem_lane:.2f} min_rem={min_rem:.2f} "
                                            f"ok={int(rem_lane >= min_rem or stuck_override)} stuck_override={int(stuck_override)}"
                                        )
                                        if rem_lane >= min_rem or stuck_override:
                                            optimized_edges = [str(x) for x in list(route_opt.get("optimized_path_edges", []) or []) if str(x) and not str(x).startswith(":")]
                                            _fed_dbg_main(
                                                f"evt=EV_ROUTE_OPT_PATH ev={ev_id} optimized_edges_n={len(optimized_edges)} "
                                                f"contains_curr={int(curr_edge in optimized_edges)}"
                                            )
                                            if len(optimized_edges) >= 2:
                                                if curr_edge in optimized_edges:
                                                    start_idx = optimized_edges.index(curr_edge)
                                                    new_route = optimized_edges[start_idx:]
                                                else:
                                                    new_route = [curr_edge] + optimized_edges
                                                if len(new_route) >= 2:
                                                    try:
                                                        curr_route_full = [str(e) for e in list(traci.vehicle.getRoute(str(ev_id)) or []) if str(e)]
                                                    except Exception:
                                                        curr_route_full = []
                                                    if curr_edge in curr_route_full:
                                                        curr_route_suffix = curr_route_full[curr_route_full.index(curr_edge):]
                                                    else:
                                                        curr_route_suffix = [curr_edge] + curr_route_full
                                                    if list(curr_route_suffix) == list(new_route):
                                                        _fed_dbg_main(
                                                            f"evt=EV_ROUTE_APPLY_SKIP ev={ev_id} reason=noop_route stuck_override={int(stuck_override)}"
                                                        )
                                                        _fed_evt_main(
                                                            "corridor.route_advice.apply_skipped",
                                                            role="ev",
                                                            ev_id=str(ev_id),
                                                            reason="noop_route",
                                                            stuck_override=bool(stuck_override),
                                                        )
                                                        print(
                                                            "[GTCO-ROUTE-APPLY-SKIP] "
                                                            f"sim={sim_time:.1f} ev={ev_id} reason=noop_route"
                                                        )
                                                    else:
                                                        candidate_tls = {
                                                            str(edge_to_tls.get(str(eid), ""))
                                                            for eid in list(new_route)
                                                            if str(edge_to_tls.get(str(eid), ""))
                                                        }
                                                        missing_tls = sorted([str(tl) for tl in list(candidate_tls) if str(tl) not in active_agent_tls_set])
                                                        _fed_dbg_main(
                                                            f"evt=EV_ROUTE_CANDIDATE ev={ev_id} route_edges_n={len(new_route)} "
                                                            f"tls_n={len(candidate_tls)} missing_tls_n={len(missing_tls)}"
                                                        )
                                                        if missing_tls and on_demand_agent_activation:
                                                            n_new = _activate_tls_agents(
                                                                list(missing_tls),
                                                                reason="ev_route_apply",
                                                                sim_time=float(sim_time),
                                                                max_new=on_demand_max_new_per_tick,
                                                            )
                                                            missing_tls = sorted([str(tl) for tl in list(candidate_tls) if str(tl) not in active_agent_tls_set])
                                                            _fed_dbg_main(
                                                                f"evt=EV_ROUTE_CANDIDATE_ACTIVATE ev={ev_id} "
                                                                f"activated_n={n_new} missing_tls_n={len(missing_tls)}"
                                                            )
                                                        if missing_tls:
                                                            _fed_dbg_main(
                                                                f"evt=EV_ROUTE_APPLY_SKIP ev={ev_id} reason=missing_agent_tls n={len(missing_tls)} sample={missing_tls[:6]}"
                                                            )
                                                            _fed_evt_main(
                                                                "corridor.route_advice.apply_skipped",
                                                                role="ev",
                                                                ev_id=str(ev_id),
                                                                reason="missing_agent_tls",
                                                                missing_tls_n=int(len(missing_tls)),
                                                            )
                                                            print(
                                                                "[GTCO-ROUTE-APPLY-SKIP] "
                                                                f"sim={sim_time:.1f} ev={ev_id} reason=missing_agent_tls n={len(missing_tls)} sample={missing_tls[:6]}"
                                                            )
                                                        else:
                                                            try:
                                                                traci.vehicle.setRoute(str(ev_id), list(new_route))
                                                                last_route_apply_sim_by_ev[str(ev_id)] = float(sim_time)
                                                                _fed_dbg_main(
                                                                    f"evt=EV_ROUTE_APPLY_OK ev={ev_id} edges={len(new_route)} "
                                                                    f"improve_sec={improve_sec:.2f} improve_ratio={improve_ratio:.4f} "
                                                                    f"stuck_override={int(stuck_override)} blocked_contig_sec={blocked_contig:.2f}"
                                                                )
                                                                _fed_evt_main(
                                                                    "corridor.route_advice.applied",
                                                                    role="ev",
                                                                    ev_id=str(ev_id),
                                                                    edges_n=int(len(new_route)),
                                                                    improvement_sec=float(improve_sec),
                                                                    improvement_ratio=float(improve_ratio),
                                                                    stuck_override=bool(stuck_override),
                                                                    blocked_contig_sec=float(blocked_contig),
                                                                )
                                                                print(
                                                                    "[GTCO-ROUTE-APPLY] "
                                                                    f"sim={sim_time:.1f} ev={ev_id} edges={len(new_route)} "
                                                                    f"improve={improve_sec:.1f}s ratio={improve_ratio:.3f} "
                                                                    f"stuck_override={int(stuck_override)}"
                                                                )
                                                            except Exception as e:
                                                                _fed_dbg_main(
                                                                    f"evt=EV_ROUTE_APPLY_SKIP ev={ev_id} reason=setRoute_failed:{type(e).__name__}:{e}"
                                                                )
                                                                _fed_evt_main(
                                                                    "corridor.route_advice.apply_skipped",
                                                                    role="ev",
                                                                    ev_id=str(ev_id),
                                                                    reason=f"setRoute_failed:{type(e).__name__}",
                                                                )
                                                                print(
                                                                    "[GTCO-ROUTE-APPLY-SKIP] "
                                                                    f"sim={sim_time:.1f} ev={ev_id} reason=setRoute_failed:{type(e).__name__}:{e}"
                                                                )
                                        else:
                                            _fed_dbg_main(
                                                f"evt=EV_ROUTE_APPLY_SKIP ev={ev_id} reason=near_junction rem_lane={rem_lane:.2f} "
                                                f"stuck_override={int(stuck_override)}"
                                            )
                                            _fed_evt_main(
                                                "corridor.route_advice.apply_skipped",
                                                role="ev",
                                                ev_id=str(ev_id),
                                                reason="near_junction",
                                                rem_lane_m=float(rem_lane),
                                                stuck_override=bool(stuck_override),
                                            )
                                            print(
                                                "[GTCO-ROUTE-APPLY-SKIP] "
                                                f"sim={sim_time:.1f} ev={ev_id} reason=near_junction rem_lane={rem_lane:.2f} "
                                                f"stuck_override={int(stuck_override)}"
                                            )
                    break
            _drain_b1_worker_results(sim_time)

            t_ev_update_wall = time.perf_counter()
            ev_snapshot = ev_agent.update(sim_time=sim_time)
            perf_acc["ev_update"] += float(time.perf_counter() - t_ev_update_wall)
            if do_ev_state_trace:
                try:
                    nxt = list(ev_snapshot.next_tls or [])
                    nxt0_tls = str(nxt[0][0]) if nxt else "-"
                    nxt0_dist = float(nxt[0][1]) if nxt else -1.0
                    _fed_dbg_main(
                        f"evt=EV_STATE_TRACE sim={float(sim_time):.2f} ev={ev_id} live={1 if ev_snapshot.exists_in_sim else 0} "
                        f"edge={ev_snapshot.edge_id or '-'} speed={float(ev_snapshot.speed_mps):.2f} "
                        f"dist_stop={float(ev_snapshot.dist_to_stopline_m):.2f} next_tls={nxt0_tls} next_tls_dist={nxt0_dist:.2f}"
                    )
                    _fed_evt_main(
                        "ev.state.trace",
                        role="ev",
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        exists_in_sim=bool(ev_snapshot.exists_in_sim),
                        edge_id=str(ev_snapshot.edge_id or ""),
                        speed_mps=float(ev_snapshot.speed_mps),
                        dist_to_stopline_m=float(ev_snapshot.dist_to_stopline_m),
                        next_tls=str(nxt0_tls if nxt0_tls != "-" else ""),
                        next_tls_dist_m=float(nxt0_dist if nxt else -1.0),
                    )
                except Exception:
                    pass
            if do_ev_pipeline_log:
                try:
                    _fed_dbg_main(
                        "evt=EV_PIPELINE_STATS "
                        f"sim={float(sim_time):.2f} "
                        f"rx_total={int(ev_req_pipeline_stats.get('rx_total', 0))} "
                        f"dispatch_ok={int(ev_req_pipeline_stats.get('dispatch_ok', 0))} "
                        f"drop_no_agent={int(ev_req_pipeline_stats.get('drop_no_agent', 0))} "
                        f"drop_missing_ev_id={int(ev_req_pipeline_stats.get('drop_missing_ev_id', 0))} "
                        f"drop_parse_err={int(ev_req_pipeline_stats.get('drop_parse_err', 0))} "
                        f"drop_replay={int(ev_req_pipeline_stats.get('drop_replay_both_mode', 0))} "
                        f"last_ev={str(ev_req_pipeline_last.get('ev_id', '-'))} "
                        f"last_tls={str(ev_req_pipeline_last.get('tls_id', '-'))} "
                        f"last_src={str(ev_req_pipeline_last.get('source', '-'))} "
                        f"last_dist={float(ev_req_pipeline_last.get('distance_m', -1.0)):.2f} "
                        f"last_edge={str(ev_req_pipeline_last.get('in_edge_id', '-'))} "
                        f"last_age_ms={float(ev_req_pipeline_last.get('age_ms', -1.0)):.1f}"
                    )
                except Exception:
                    pass
            if ev_http_state_server_enabled:
                with ev_http_state_lock:
                    ev_http_state_cache["ready"] = True
                    ev_http_state_cache["last_update_wall"] = float(time.time())
                    ev_http_state_cache["payload"] = {
                        "ok": True,
                        "source": "real_world.vehicle_agent",
                        "simTime": float(sim_time),
                        "evId": str(ev_id),
                        "profile": ev_profile.to_dict(),
                        "snapshot": ev_snapshot.to_dict(),
                    }
            if do_pub_vehicle and ev_local_state_mqtt_enable:
                client.publish(
                    f"rw/vehicle_agent/{ev_id}/state",
                    json.dumps({
                        "profile": ev_profile.to_dict(),
                        "snapshot": ev_snapshot.to_dict(),
                    }),
                )
            if ers_agent is not None:
                ers_agent.update_vehicle_state(str(ev_id), ev_snapshot.to_dict())

            # Keep loop cumulative counters updated from simulation start
            # (independent of EV-triggered edge-specific tracing).
            t_loop_poll_wall = time.perf_counter()
            if bool(args.auto_induction_loops) and do_loop_sense:
                if loop_sense_scope_mode == "core":
                    loop_sense_target_tls_ids = list(loop_sense_tls_ids_core or loop_sense_tls_ids_expanded or active_agent_tls_ids)
                elif loop_sense_scope_mode == "expanded":
                    loop_sense_target_tls_ids = list(loop_sense_tls_ids_expanded or active_agent_tls_ids)
                elif loop_sense_scope_mode == "all":
                    loop_sense_target_tls_ids = list(active_agent_tls_ids)
                elif loop_sense_scope_mode == "ev-active":
                    loop_sense_target_tls_ids = list(loop_sense_active_tls_ids or loop_sense_tls_ids_core or loop_sense_tls_ids_expanded or active_agent_tls_ids)
                else:
                    if str(CURRENT_EVALUATION) in ("B0", "B1"):
                        loop_sense_target_tls_ids = list(loop_sense_tls_ids_core or loop_sense_tls_ids_expanded or active_agent_tls_ids)
                    else:
                        loop_sense_target_tls_ids = list(loop_sense_tls_ids_expanded or active_agent_tls_ids)
                for _tls_id in loop_sense_target_tls_ids:
                    _ag = agents.get(str(_tls_id))
                    if _ag is None:
                        continue
                    try:
                        _ag.trace_loop_detections(sim_time=sim_time, emit_log=False)
                    except Exception:
                        pass
            perf_acc["loop_poll"] += float(time.perf_counter() - t_loop_poll_wall)

            # Federation warmup hook:
            # allow downstream intersections to pre-actuate from accepted reservations
            # before direct EV contact.
            t_warmup_wall = time.perf_counter()
            if CURRENT_EVALUATION == "F2" and not STATIC_PROGRAM:
                for warm_tls_id, warm_ag in agents.items():
                    warm_plan = warm_ag.maybe_warmup_from_federation(sim_time)
                    if warm_plan is None:
                        continue
                    print(f"[F2-WARMUP] tls={warm_tls_id} plan={warm_plan}")
                    warm_ag.apply_plan_to_tls(sim_time, warm_plan, decision_source="federation_warmup")
                    client.publish(
                        f"rw/agent/{warm_tls_id}/warmup_plan",
                        json.dumps({
                            "step": step,
                            "simTime": sim_time,
                            "tlsId": warm_tls_id,
                            "plan": warm_plan.__dict__,
                        }),
                    )
            perf_acc["warmup"] += float(time.perf_counter() - t_warmup_wall)

            # Snapshots
            '''
            if args.state_dir and (sim_time - last_state_dump) >= args.state_period:
                fname = os.path.join(args.state_dir, f"main_t={sim_time:.1f}.xml")
                traci.simulation.saveState(fname)
                last_state_dump = sim_time
            # rotate old snapshots
            snaps = sorted(glob.glob(os.path.join(args.state_dir, "main_t=*.xml")))

            if len(snaps) > args.state_keep:
                for old in snaps[: len(snaps) - args.state_keep]:
                    try: os.remove(old)
                    except Exception: pass
            '''


            # vehicles: subscribe when appear
            t_vehicle_io_wall = time.perf_counter()
            live_veh = set(traci.vehicle.getIDList())
            if bool(args.auto_induction_loops) and do_loop_sense and (ev_id in live_veh):
                try:
                    ev_route_rt = tuple(str(x) for x in list(traci.vehicle.getRoute(ev_id)) if str(x))
                    ev_route_idx_rt = int(traci.vehicle.getRouteIndex(ev_id))
                except Exception:
                    ev_route_rt = ()
                    ev_route_idx_rt = -1
                try:
                    ev_curr_edge_rt = str(traci.vehicle.getRoadID(ev_id) or "")
                except Exception:
                    ev_curr_edge_rt = ""

                if ev_route_rt and ev_route_rt != loop_sense_route_edges_last:
                    loop_sense_route_edges_last = tuple(ev_route_rt)
                    try:
                        rt_subset_tls, rt_core_tls = select_tls_subset_for_ev_route(
                            route_edges=list(ev_route_rt),
                            edge_to_tls=edge_to_tls,
                            tls_neighbors=tls_neighbors,
                            neighbor_hops=int(getattr(args, "agent_subset_neighbor_hops", 1)),
                        )
                    except Exception:
                        rt_subset_tls, rt_core_tls = [], []
                    if on_demand_agent_activation:
                        rt_activation_cands: List[str] = []
                        for tls_id in list(rt_core_tls or []):
                            stls = str(tls_id)
                            if stls and stls not in rt_activation_cands:
                                rt_activation_cands.append(stls)
                            if len(rt_activation_cands) >= on_demand_lookahead_hops:
                                break
                        if len(rt_activation_cands) < on_demand_lookahead_hops:
                            for tls_id in list(rt_subset_tls or []):
                                stls = str(tls_id)
                                if stls and stls not in rt_activation_cands:
                                    rt_activation_cands.append(stls)
                                if len(rt_activation_cands) >= on_demand_lookahead_hops:
                                    break
                        if rt_activation_cands:
                            _activate_tls_agents(
                                rt_activation_cands,
                                reason="ev_route_runtime_update",
                                sim_time=float(sim_time),
                                max_new=on_demand_max_new_per_tick,
                            )
                    if rt_subset_tls or rt_core_tls:
                        active_set = set(str(x) for x in active_agent_tls_ids)
                        loop_sense_tls_ids_expanded = [str(tl) for tl in list(rt_subset_tls or []) if str(tl) in active_set]
                        loop_sense_tls_ids_core = [str(tl) for tl in list(rt_core_tls or []) if str(tl) in active_set]
                        uncovered = max(0, len(list(rt_subset_tls or [])) - len(loop_sense_tls_ids_expanded))
                        print(
                            f"[loop-sense][route-update] ev={ev_id} route_edges={len(ev_route_rt)} "
                            f"core_tls={len(loop_sense_tls_ids_core)} expanded_tls={len(loop_sense_tls_ids_expanded)} "
                            f"uncovered_expanded={uncovered}"
                        )

                active_tls_window: List[str] = []
                seen_active = set()
                active_set = set(str(x) for x in active_agent_tls_ids)
                # Include current controlling TLS when available.
                if ev_curr_edge_rt and not str(ev_curr_edge_rt).startswith(":"):
                    cur_tls = edge_to_tls.get(str(ev_curr_edge_rt))
                    if cur_tls and str(cur_tls) in active_set:
                        active_tls_window.append(str(cur_tls))
                        seen_active.add(str(cur_tls))
                # Add future route TLS up to configured active hops.
                active_hops = max(1, int(getattr(args, "loop_sense_active_hops", 3)))
                start_idx = max(0, int(ev_route_idx_rt))
                for e_future in list(ev_route_rt[start_idx:] if ev_route_rt else []):
                    tls_future = edge_to_tls.get(str(e_future))
                    if not tls_future:
                        continue
                    stls = str(tls_future)
                    if stls not in active_set or stls in seen_active:
                        continue
                    active_tls_window.append(stls)
                    seen_active.add(stls)
                    if len(active_tls_window) >= active_hops:
                        break
                if active_tls_window:
                    loop_sense_active_tls_ids = list(active_tls_window)
            for vid in target_veh + [ev_id]:
                if vid in live_veh and vid not in subscribed_veh:
                    traci.vehicle.subscribe(
                        vid,
                        [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE, tc.VAR_ROAD_ID, tc.VAR_LANE_INDEX]
                    )
                    subscribed_veh.add(vid)

            if do_ev_kpi and (ev_id in live_veh):
                try:
                    ev_res = traci.vehicle.getSubscriptionResults(ev_id) or {}
                    ex, ey = ev_res.get(tc.VAR_POSITION, traci.vehicle.getPosition(ev_id))
                    ev_speed_kpi = float(ev_res.get(tc.VAR_SPEED, traci.vehicle.getSpeed(ev_id)))
                    ev_edge_kpi = str(ev_res.get(tc.VAR_ROAD_ID, traci.vehicle.getRoadID(ev_id)))
                    ev_lane_idx_kpi = int(ev_res.get(tc.VAR_LANE_INDEX, traci.vehicle.getLaneIndex(ev_id)))
                    try:
                        ev_lane_id_kpi = str(traci.vehicle.getLaneID(ev_id))
                    except Exception:
                        ev_lane_id_kpi = ""
                    try:
                        ev_lane_pos_kpi = float(traci.vehicle.getLanePosition(ev_id))
                    except Exception:
                        ev_lane_pos_kpi = None
                    d_stop_kpi = dist_to_stopline_on_current_lane(ev_id)

                    # Update simple runtime KPI accumulators
                    if ev_kpi_stats["first_t"] is None:
                        ev_kpi_stats["first_t"] = float(sim_time)
                    ev_kpi_stats["last_t"] = float(sim_time)
                    ev_kpi_stats["samples"] = int(ev_kpi_stats.get("samples", 0)) + 1
                    ev_kpi_stats["speed_sum"] = float(ev_kpi_stats.get("speed_sum", 0.0)) + float(ev_speed_kpi)
                    smin = ev_kpi_stats.get("speed_min")
                    smax = ev_kpi_stats.get("speed_max")
                    ev_kpi_stats["speed_min"] = float(ev_speed_kpi) if smin is None else min(float(smin), float(ev_speed_kpi))
                    ev_kpi_stats["speed_max"] = float(ev_speed_kpi) if smax is None else max(float(smax), float(ev_speed_kpi))
                    last_sample_t = ev_kpi_stats.get("last_sample_t")
                    if last_sample_t is not None:
                        dt_s = max(0.0, float(sim_time) - float(last_sample_t))
                        if float(ev_speed_kpi) <= 0.5:
                            ev_kpi_stats["stop_time_sec"] = float(ev_kpi_stats.get("stop_time_sec", 0.0)) + dt_s
                        if float(ev_speed_kpi) <= 1.0:
                            ev_kpi_stats["slow_time_sec"] = float(ev_kpi_stats.get("slow_time_sec", 0.0)) + dt_s
                        if float(d_stop_kpi) <= 2.0:
                            ev_kpi_stats["near_stopline_time_sec"] = float(ev_kpi_stats.get("near_stopline_time_sec", 0.0)) + dt_s
                            if float(ev_speed_kpi) <= 0.5:
                                ev_kpi_stats["blocked_near_stopline_time_sec"] = float(ev_kpi_stats.get("blocked_near_stopline_time_sec", 0.0)) + dt_s
                    ev_kpi_stats["last_sample_t"] = float(sim_time)
                    ev_kpi_stats["last_speed"] = float(ev_speed_kpi)
                    ev_kpi_stats["last_d_stop"] = float(d_stop_kpi)

                    _ev_kpi_dbg(
                        f"evt=EV_STATE ev={ev_id} step={int(step)} edge={ev_edge_kpi} lane={ev_lane_id_kpi} "
                        f"laneIndex={int(ev_lane_idx_kpi)} lanePos={'' if ev_lane_pos_kpi is None else f'{float(ev_lane_pos_kpi):.2f}'} "
                        f"x={float(ex):.2f} y={float(ey):.2f} speed={float(ev_speed_kpi):.2f} d_stop={float(d_stop_kpi):.2f}"
                    )
                    ev_kpi_samples.append({
                        "t": float(sim_time),
                        "step": int(step),
                        "edge": str(ev_edge_kpi),
                        "lane": str(ev_lane_id_kpi),
                        "laneIndex": int(ev_lane_idx_kpi),
                        "lanePos": (None if ev_lane_pos_kpi is None else float(ev_lane_pos_kpi)),
                        "x": float(ex),
                        "y": float(ey),
                        "speed": float(ev_speed_kpi),
                        "d_stop": float(d_stop_kpi),
                    })
                except Exception as e:
                    _ev_kpi_dbg(f"evt=EV_STATE_WARN ev={ev_id} err={type(e).__name__}:{e}", t_override=sim_time)

            # publish vehicles
            if do_pub_vehicle:
                for vid in target_veh + [ev_id]:
                    if (not ev_local_state_mqtt_enable) and str(vid) == str(ev_id):
                        continue
                    if vid not in live_veh:
                        continue
                    res = traci.vehicle.getSubscriptionResults(vid) or {}
                    x, y = res.get(tc.VAR_POSITION, traci.vehicle.getPosition(vid))
                    msg = {
                        "vehId": vid,
                        "step": step,
                        "simTime": sim_time,
                        "x": float(x),
                        "y": float(y),
                        "angle": float(res.get(tc.VAR_ANGLE, traci.vehicle.getAngle(vid))),
                        "speed": float(res.get(tc.VAR_SPEED, traci.vehicle.getSpeed(vid))),
                        "edge": str(res.get(tc.VAR_ROAD_ID, traci.vehicle.getRoadID(vid))),
                        "laneIndex": int(res.get(tc.VAR_LANE_INDEX, traci.vehicle.getLaneIndex(vid))),
                    }
                    if str(vid) == str(ev_id):
                        try:
                            msg["route_veh"] = [str(e) for e in list(traci.vehicle.getRoute(vid) or []) if str(e)]
                            msg["route_index"] = int(traci.vehicle.getRouteIndex(vid))
                        except Exception:
                            pass
                    client.publish(f"rw/vehicle/{vid}/state", json.dumps(msg))
            perf_acc["vehicle_io"] += float(time.perf_counter() - t_vehicle_io_wall)

            # ---- Embedded IntersectionAgent control (EV -> pick TLS -> decide -> actuate) ----
            t_ev_logic_wall = time.perf_counter()
            current_ev_tls_id = None

            if ev_id in live_veh:
                ev_edge_snapshot = str(getattr(ev_snapshot, "edge_id", "") or "")
                try:
                    ev_edge_traci = str(traci.vehicle.getRoadID(ev_id) or "")
                except Exception:
                    ev_edge_traci = ""
                ev_edge_source_mode = str(getattr(args, "ev_edge_source", "auto") or "auto").strip().lower()
                if ev_edge_source_mode == "snapshot":
                    ev_edge_raw = str(ev_edge_snapshot)
                elif ev_edge_source_mode == "traci":
                    ev_edge_raw = str(ev_edge_traci)
                else:
                    snap_valid = bool(ev_edge_snapshot) and (not str(ev_edge_snapshot).startswith(":"))
                    traci_valid = bool(ev_edge_traci) and (not str(ev_edge_traci).startswith(":"))
                    # In auto mode, prefer TraCI when both are valid but disagree (snapshot can lag at handoff boundaries).
                    if snap_valid and traci_valid and str(ev_edge_snapshot) != str(ev_edge_traci):
                        ev_edge_raw = str(ev_edge_traci)
                    elif snap_valid:
                        ev_edge_raw = str(ev_edge_snapshot)
                    elif traci_valid:
                        ev_edge_raw = str(ev_edge_traci)
                    else:
                        ev_edge_raw = str(ev_edge_traci or ev_edge_snapshot)

                # Track last external edge (incoming road), skip pure internal edges
                if not ev_edge_raw.startswith(":"):
                    ev_last_external_edge = ev_edge_raw

                # For the baseline, we only act when we have an external incoming edge
                ev_edge = ev_last_external_edge
                if not ev_edge:
                    # EV is still internal and we don't know its inbound edge this tick
                    pass
                else:
                    # Determine the junction at the END of this edge using net.xml.
                    approach_node = edge_to_approach_node_from_net(ev_edge, edge_to_to_node)
                    selected_in_edge = str(ev_edge)
                    lookahead_hops = 0
                    route_veh_runtime: List[str] = []

                    # First choice: TLS directly controlling the current edge's approach node.
                    tls_candidates = sorted(node_to_tls.get(approach_node, [])) if approach_node else []

                    # Fallback: if current approach node is unmanaged (priority connector),
                    # look ahead in the EV route and bind to the first downstream TLS.
                    if not tls_candidates:
                        try:
                            route_veh_runtime = list(traci.vehicle.getRoute(ev_id))
                            route_idx_runtime = int(traci.vehicle.getRouteIndex(ev_id))
                        except Exception:
                            route_veh_runtime = []
                            route_idx_runtime = -1

                        if route_veh_runtime:
                            for hop, e_future in enumerate(route_veh_runtime[max(0, route_idx_runtime):], start=0):
                                e_future = str(e_future)
                                if not e_future or e_future.startswith(":"):
                                    continue
                                future_node = edge_to_approach_node_from_net(e_future, edge_to_to_node)
                                future_tls = sorted(node_to_tls.get(future_node, [])) if future_node else []
                                if future_tls:
                                    tls_candidates = list(future_tls)
                                    selected_in_edge = str(e_future)
                                    lookahead_hops = int(hop)
                                    break
                    if tls_candidates:
                        tls_id = str(tls_candidates[0])
                        current_ev_tls_id = str(tls_id)
                        ag = agents.get(tls_id)
                        diag_key = (str(ev_edge_snapshot), str(ev_edge_traci), str(tls_id))
                        if diag_key != ev_trigger_diag_last:
                            ev_trigger_diag_last = diag_key
                            _fed_dbg_main(
                                f"evt=EV_EDGE_SELECT ev={ev_id} mode={ev_edge_source_mode} "
                                f"snapshot_edge={ev_edge_snapshot} traci_edge={ev_edge_traci} chosen_edge={ev_edge_raw} "
                                f"approach_node={approach_node} tls={tls_id} agent_found={1 if ag is not None else 0} "
                                f"selected_in_edge={selected_in_edge} lookahead_hops={lookahead_hops}"
                            )
                            if (not str(ev_edge_raw).startswith(":")):
                                cp_key = (str(tls_id), str(ev_edge_raw))
                                if cp_key != ev_kpi_last_checkpoint:
                                    ev_kpi_last_checkpoint = cp_key
                                    _ev_kpi_dbg(
                                        f"evt=EV_CHECKPOINT ev={ev_id} tls={tls_id} edge={ev_edge_raw} "
                                        f"approach_node={approach_node} agent_found={1 if ag is not None else 0}",
                                        t_override=sim_time,
                                    )
                                    ev_kpi_checkpoints.append({
                                        "t": float(sim_time),
                                        "step": int(step),
                                        "ev": str(ev_id),
                                        "tls": str(tls_id),
                                        "edge": str(ev_edge_raw),
                                        "approach_node": str(approach_node) if approach_node is not None else "",
                                        "agent_found": 1 if ag is not None else 0,
                                    })

                        if tls_id in debug_tls_ids:
                            pass
                            #get_current_tls_detils(tls_id)

                        if ag is not None:
                            # Distance-to-stopline on current lane (better than Euclidean)
                            d_stop = dist_to_stopline_on_current_lane(ev_id)
                            # Standalone loop trace (independent of queue metric calls)
                            if do_loop_sense:
                                try:
                                    ag.trace_loop_detections(sim_time=sim_time, edge_id=str(ev_edge))
                                except Exception as e:
                                    # Surface loop-trace failures instead of silently suppressing them.
                                    print(f"[loops][WARN] trace_loop_detections failed for tls={tls_id} edge={ev_edge}: {e}")

                            # Optional: only trigger when "close enough" (matches your cfg.min_trigger_distance_m intent)
                            if d_stop <= float(ag.cfg.min_trigger_distance_m):
                                skip_internal_emit = False
                                if not internal_ev_request_enabled:
                                    _fed_dbg_main(
                                        f"evt=EV_TRIGGER_SKIP ev={ev_id} tls={tls_id} reason=internal_ev_request_disabled"
                                    )
                                    skip_internal_emit = True
                                    # Keep local decision path alive in bridge/MQTT mode by refreshing
                                    # the currently tracked EV request with high-rate TraCI telemetry.
                                    try:
                                        if ag.active_ev is not None and str(getattr(ag.active_ev, "ev_id", "")) == str(ev_id):
                                            ag.active_ev.sim_time = float(sim_time)
                                            ag.active_ev.speed_mps = float(traci.vehicle.getSpeed(ev_id))
                                            ag.active_ev.distance_to_intersection_m = float(d_stop)
                                            ag.active_ev.in_edge_id = str(selected_in_edge)
                                            _route_now = list(traci.vehicle.getRoute(ev_id) or [])
                                            if _route_now:
                                                ag.active_ev.route_veh = list(_route_now)
                                            _fed_dbg_main(
                                                f"evt=EV_TRIGGER_REFRESH ev={ev_id} tls={tls_id} edge={selected_in_edge} "
                                                f"d_stop={float(d_stop):.2f} speed={float(ag.active_ev.speed_mps):.2f}"
                                            )
                                    except Exception as _ev_refresh_err:
                                        _fed_dbg_main(
                                            f"evt=EV_TRIGGER_REFRESH_ERR ev={ev_id} tls={tls_id} "
                                            f"err={type(_ev_refresh_err).__name__}:{_ev_refresh_err}"
                                        )
                                _fed_dbg_main(
                                    f"evt=EV_TRIGGER ev={ev_id} tls={tls_id} edge={ev_edge_raw} "
                                    f"d_stop={float(d_stop):.2f} trigger_thr={float(ag.cfg.min_trigger_distance_m):.2f}"
                                )
                                route_nodes: List[str] = []
                                if not bool(getattr(args, "legacy_ev_request", False)):
                                    try:
                                        route_nodes = ev_agent.infer_route_intersections(
                                            edge_to_to_node=edge_to_to_node,
                                            max_hops=8,
                                        )
                                    except Exception:
                                        route_nodes = []
                                if bool(getattr(args, "legacy_ev_request", False)):
                                    ev_route_veh = list(traci.vehicle.getRoute(ev_id))
                                    ev_msg = EvRequest(
                                        ev_id=str(ev_id),
                                        sim_time=float(sim_time),
                                        erl_level=int(erl_level),
                                        speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                        distance_to_intersection_m=float(d_stop),
                                        in_edge_id=str(selected_in_edge),
                                        target_phase_idx=None,
                                        route_intersections=list(route_nodes) if route_nodes else None,
                                        route_veh=ev_route_veh,
                                    )
                                    _fed_dbg_main(
                                        f"evt=EV_MSG_BUILD mode=legacy ev={ev_id} tls={tls_id} "
                                        f"edge={selected_in_edge} route_len={len(ev_route_veh)} route_nodes={len(route_nodes)} "
                                        f"lookahead_hops={lookahead_hops}"
                                    )
                                else:
                                    ev_msg = ev_agent.build_ev_request(
                                        sim_time=sim_time,
                                        approach_edge=str(selected_in_edge),  # IMPORTANT: inbound edge of selected TLS
                                        distance_to_intersection_m=float(d_stop),
                                        target_phase_idx=None,               # agent auto-selects
                                        erl_level=erl_level,
                                        route_intersections=route_nodes,
                                        route_veh = list(traci.vehicle.getRoute(ev_id))
                                    )
                                    _fed_dbg_main(
                                        f"evt=EV_MSG_BUILD mode=vehicle_agent ev={ev_id} tls={tls_id} "
                                        f"edge={selected_in_edge} route_nodes={len(route_nodes)} "
                                        f"lookahead_hops={lookahead_hops}"
                                    )
                                print(f"Current distance to tls_id: {tls_id} is {d_stop} m")

                                ev_msg.source_service = "vehicle_agent"
                                ev_msg.source_tag = str(ev_request_source_tag or "direct")
                                ev_msg.delivery = "direct"

                                if (not skip_internal_emit) and ev_request_delivery_mode in ("mqtt", "both"):
                                    req_topic = f"{ev_request_topic_prefix}/{tls_id}"
                                    req_payload = asdict(ev_msg)
                                    req_payload["tls_id"] = str(tls_id)
                                    req_payload["source_service"] = "vehicle_agent"
                                    req_payload["source_tag"] = str(ev_request_source_tag or "direct")
                                    req_payload["delivery"] = "mqtt"
                                    client.publish(req_topic, json.dumps(req_payload))
                                    _fed_dbg_main(
                                        f"evt=PUB topic={req_topic} kind=ev_request ev={ev_msg.ev_id} tls={tls_id} "
                                        f"in_edge={ev_msg.in_edge_id} dist={float(ev_msg.distance_to_intersection_m):.2f}"
                                    )
                                    _fed_evt_main(
                                        "ev.request.published",
                                        role="ev",
                                        ev_id=str(ev_msg.ev_id),
                                        tls_id=str(tls_id),
                                        in_edge_id=str(ev_msg.in_edge_id),
                                        distance_to_intersection_m=float(ev_msg.distance_to_intersection_m),
                                        delivery="mqtt",
                                        ev_request_source="vehicle_agent",
                                        ev_request_source_tag=str(ev_request_source_tag or "direct"),
                                    )
                                if (not skip_internal_emit) and ev_request_delivery_mode in ("direct", "both"):
                                    ag.receive_ev_message(ev_msg)
                                    _fed_evt_main(
                                        "ev.request.dispatched",
                                        role="ev",
                                        ev_id=str(ev_msg.ev_id),
                                        tls_id=str(tls_id),
                                        in_edge_id=str(ev_msg.in_edge_id),
                                        distance_to_intersection_m=float(ev_msg.distance_to_intersection_m),
                                        delivery="direct",
                                        ev_request_source="vehicle_agent",
                                        ev_request_source_tag=str(ev_request_source_tag or "direct"),
                                    )

                                if CURRENT_EVALUATION == 'B0':
                                    plan = None
                                    print("Currently on Fixed program")

                                if CURRENT_EVALUATION == 'B1':
                                    # B1 diagnostics: inspect edge->phase mapping and decision context at the
                                    # same cadence as the agent decision period (or when the returned plan is non-null).
                                    try:
                                        mapped_target_phase = int(getattr(ag, "_inbound_edge_to_phase", {}).get(str(ev_edge), 0))
                                    except Exception:
                                        mapped_target_phase = 0
                                    map_diag_key = (str(tls_id), str(ev_edge))
                                    if map_diag_key not in b1_map_diag_seen:
                                        b1_map_diag_seen.add(map_diag_key)
                                        _fed_dbg_main(
                                            f"evt=B1_MAP tls={tls_id} ev={ev_id} edge={ev_edge} "
                                            f"mapped_target_phase={mapped_target_phase} "
                                            f"map_size={len(getattr(ag, '_inbound_edge_to_phase', {}) or {})}"
                                        )

                                    t_i_b1 = None
                                    target_phase_b1 = None
                                    arr_win_b1 = None
                                    base_win_b1 = None
                                    window_cover_b1 = None
                                    eta_b1 = None
                                    clrs_b1 = None
                                    tul_b1 = None
                                    stage_before_tick_b1 = str(ag.stage)
                                    try:
                                        if ag.active_ev is not None:
                                            t_i_b1 = float(ag._estimate_arrival_time(float(sim_time), ag.active_ev))
                                            eta_b1 = float(t_i_b1)
                                            target_phase_b1 = int(ag.active_ev.target_phase_idx or 0)
                                            arr_win_b1 = (
                                                float(t_i_b1) - float(getattr(ag.active_ev, "delta_sec", 2.0)),
                                                float(t_i_b1) + float(getattr(ag.active_ev, "delta_sec", 2.0)),
                                            )
                                            base_win_b1 = ag._predict_next_phase_window(float(sim_time), int(target_phase_b1))
                                            if base_win_b1 is not None and arr_win_b1 is not None:
                                                window_cover_b1 = bool(
                                                    (float(base_win_b1[0]) <= float(arr_win_b1[0]))
                                                    and (float(base_win_b1[1]) >= float(arr_win_b1[1]))
                                                )
                                            clrs_b1 = int(ag._compute_clrs_level(str(ev_edge)))
                                            tul_b1 = int(ag._compute_tul_level(float(sim_time), float(t_i_b1)))
                                    except Exception:
                                        pass

                                    try:
                                        tls_phase_before_b1 = int(traci.trafficlight.getPhase(tls_id))
                                    except Exception:
                                        tls_phase_before_b1 = -1
                                    try:
                                        tls_prog_before_b1 = str(traci.trafficlight.getProgram(tls_id))
                                    except Exception:
                                        tls_prog_before_b1 = ""
                                    try:
                                        tls_state_before_b1 = str(traci.trafficlight.getRedYellowGreenState(tls_id))
                                    except Exception:
                                        tls_state_before_b1 = ""
                                    try:
                                        tls_next_switch_before_b1 = float(traci.trafficlight.getNextSwitch(tls_id))
                                    except Exception:
                                        tls_next_switch_before_b1 = -1.0

                                    next_decision_before_b1 = getattr(ag, "_next_decision_time", None)
                                    decision_due_before_b1 = (
                                        (next_decision_before_b1 is None)
                                        or (float(sim_time) >= float(next_decision_before_b1))
                                    )

                                    plan = ag.tick(sim_time)
                                    stage_after_tick_b1 = str(ag.stage)

                                    next_decision_after_b1 = getattr(ag, "_next_decision_time", None)
                                    if decision_due_before_b1 or (plan is not None):
                                        if plan is None:
                                            plan_kind_b1 = "tick_none"
                                            plan_type_b1 = "NA"
                                            plan_target_b1 = "NA"
                                            plan_ext_b1 = "NA"
                                            plan_hurry_b1 = "NA"
                                            plan_jump_b1 = "NA"
                                        else:
                                            plan_kind_b1 = "plan"
                                            plan_type_b1 = str(getattr(plan, "plan_type", ""))
                                            plan_target_b1 = str(getattr(plan, "target_phase_idx", ""))
                                            plan_ext_b1 = f"{float(getattr(plan, 'extend_green_sec', 0.0) or 0.0):.3f}"
                                            _h_b1 = getattr(plan, "hurry_current_phase_to_sec", None)
                                            _jt_b1 = getattr(plan, "jump_time_sec", None)
                                            _jp_b1 = getattr(plan, "jump_to_phase_idx", None)
                                            plan_hurry_b1 = "NA" if _h_b1 is None else f"{float(_h_b1):.3f}"
                                            plan_jump_b1 = (
                                                "NA"
                                                if (_jt_b1 is None and _jp_b1 is None)
                                                else f"{('NA' if _jt_b1 is None else f'{float(_jt_b1):.3f}')}->{('NA' if _jp_b1 is None else int(_jp_b1))}"
                                            )
                                        _fed_dbg_main(
                                            f"evt=B1_TICK tls={tls_id} ev={ev_id} due={1 if decision_due_before_b1 else 0} "
                                            f"sim={float(sim_time):.2f} d_stop={float(d_stop):.2f} speed={float(traci.vehicle.getSpeed(ev_id)):.2f} "
                                            f"stage_before={stage_before_tick_b1} stage_after={stage_after_tick_b1} "
                                            f"tls_phase={tls_phase_before_b1} prog={tls_prog_before_b1} "
                                            f"next_switch={tls_next_switch_before_b1:.2f} mapped_target={mapped_target_phase} "
                                            f"target_phase={('NA' if target_phase_b1 is None else int(target_phase_b1))} "
                                            f"eta={('NA' if eta_b1 is None else f'{float(eta_b1):.2f}')} "
                                            f"arr_win={('NA' if arr_win_b1 is None else f'({float(arr_win_b1[0]):.2f},{float(arr_win_b1[1]):.2f})')} "
                                            f"base_win={('NA' if base_win_b1 is None else f'({float(base_win_b1[0]):.2f},{float(base_win_b1[1]):.2f})')} "
                                            f"window_cover={('NA' if window_cover_b1 is None else int(bool(window_cover_b1)))} "
                                            f"clrs={('NA' if clrs_b1 is None else int(clrs_b1))} tul={('NA' if tul_b1 is None else int(tul_b1))} "
                                            f"result={plan_kind_b1} plan_type={plan_type_b1} plan_target={plan_target_b1} "
                                            f"plan_ext={plan_ext_b1} plan_hurry={plan_hurry_b1} plan_jump={plan_jump_b1} "
                                            f"next_decision_before={('NA' if next_decision_before_b1 is None else f'{float(next_decision_before_b1):.2f}')} "
                                            f"next_decision_after={('NA' if next_decision_after_b1 is None else f'{float(next_decision_after_b1):.2f}')}"
                                        )

                                    # Prototype: snapshot-only B1 advisory in worker process (no TraCI in worker).
                                    if b1_worker_pool is not None and ag.active_ev is not None:
                                        pending_item = b1_worker_pending.get(str(tls_id))
                                        busy = False
                                        if pending_item is not None:
                                            try:
                                                _pending_async, _pending_t = pending_item
                                                busy = (not _pending_async.ready())
                                            except Exception:
                                                busy = False
                                        if not busy:
                                            try:
                                                ev_proto = ag.active_ev
                                                eta_proto = float(ag._estimate_arrival_time(sim_time, ev_proto))
                                                clrs_proto = int(ag._compute_clrs_level(str(ev_edge)))
                                                tul_proto = int(ag._compute_tul_level(float(sim_time), float(eta_proto)))
                                                target_phase_proto = int(ev_proto.target_phase_idx or 0)
                                                win_proto = ag._predict_next_phase_window(float(sim_time), target_phase_proto)
                                                window_ok_proto = False
                                                if win_proto is not None:
                                                    a0p = float(eta_proto) - float(ev_proto.delta_sec)
                                                    a1p = float(eta_proto) + float(ev_proto.delta_sec)
                                                    window_ok_proto = bool((win_proto[0] <= a0p) and (win_proto[1] >= a1p))
                                                task = {
                                                    "tls_id": str(tls_id),
                                                    "sim_time": float(sim_time),
                                                    "eta": float(eta_proto),
                                                    "erl": int(getattr(ev_proto, "erl_level", erl_level)),
                                                    "clrs": int(clrs_proto),
                                                    "tul": int(tul_proto),
                                                    "window_ok": bool(window_ok_proto),
                                                    "target_phase_idx": int(target_phase_proto),
                                                    "baseline_clrs_max": int(getattr(ag.cfg, "baseline_clrs_max", 2)),
                                                    "baseline_tul_max": int(getattr(ag.cfg, "baseline_tul_max", 1)),
                                                    "sat_to_preempt_gap_sec": float(getattr(ag.cfg, "saturation_to_preempt_gap_sec", 30.0)),
                                                    "drrs_weights": {
                                                        "erl": float(getattr(ag.cfg, "w_erl", 0.1031)),
                                                        "clrs": float(getattr(ag.cfg, "w_clrs", 0.6053)),
                                                        "tul": float(getattr(ag.cfg, "w_tul", 0.2915)),
                                                    },
                                                    "drrs_clusters": list(getattr(ag.cfg, "drrs_clusters", []) or []),
                                                }
                                                b1_worker_pending[str(tls_id)] = (
                                                    b1_worker_pool.apply_async(b1_worker_proto_eval, (task,)),
                                                    float(sim_time),
                                                )
                                            except Exception as e:
                                                print(f"[B1-WORKER-PROTO][WARN] submit failed tls={tls_id}: {e}")

                                    #if tls_id == "Node2":
                                    

                                    if plan and not STATIC_PROGRAM:
                                        print(f"**** Selected plan: {plan} ****")
                                        ag.apply_plan_to_tls(sim_time, plan)
                                        try:
                                            tls_phase_after_b1 = int(traci.trafficlight.getPhase(tls_id))
                                        except Exception:
                                            tls_phase_after_b1 = -1
                                        try:
                                            tls_prog_after_b1 = str(traci.trafficlight.getProgram(tls_id))
                                        except Exception:
                                            tls_prog_after_b1 = ""
                                        try:
                                            tls_state_after_b1 = str(traci.trafficlight.getRedYellowGreenState(tls_id))
                                        except Exception:
                                            tls_state_after_b1 = ""
                                        try:
                                            tls_next_switch_after_b1 = float(traci.trafficlight.getNextSwitch(tls_id))
                                        except Exception:
                                            tls_next_switch_after_b1 = -1.0
                                        _fed_dbg_main(
                                            f"evt=B1_APPLY tls={tls_id} ev={ev_id} sim={float(sim_time):.2f} "
                                            f"plan_type={str(getattr(plan, 'plan_type', ''))} "
                                            f"target={str(getattr(plan, 'target_phase_idx', ''))} "
                                            f"before_phase={tls_phase_before_b1} after_phase={tls_phase_after_b1} "
                                            f"before_prog={tls_prog_before_b1} after_prog={tls_prog_after_b1} "
                                            f"before_next_switch={tls_next_switch_before_b1:.2f} after_next_switch={tls_next_switch_after_b1:.2f} "
                                            f"before_state={tls_state_before_b1} after_state={tls_state_after_b1}"
                                        )
                                        payload = json.dumps({
                                            "step": step,
                                            "simTime": sim_time,
                                            "tlsId": tls_id,
                                            "approachNode": approach_node,
                                            "evEdge": ev_edge,
                                            "distToStoplineM": d_stop,
                                            "plan": plan.__dict__,
                                        })
                                        client.publish(f"rw/agent/{tls_id}/plan", payload)
                                        print(payload)
                                        print(f"Emergency vehicle {ev_id} detected in {ev_edge} requesting support from {tls_id}")
                                    
                                    #'''
                                        print("\n------------------------------------------")
                                        print(f"Current situation in intersection {tls_id}")
                                        print(f"Current stage: {ag.stage}")
                                        print(f"Current plan: {ag.current_plan}")
                                        print(f"Current sim tick: {sim_time}")
                                    #'''
                                    print(f"\nCurrent plan: {ag.current_plan}")

                                #if sim_time == 10:
                                """
                                if tls_id == "J24":
                                    pass
                                    print(ag._inbound_edge_to_phase)
                                    print(ag.out_edge_to_neighbor)
                                    print(ag.neighbor_map)
                                    #sys.exit()
                                """
                                
                                # Apply plan only on events 
                                if CURRENT_EVALUATION == 'F1':
                                    print("Currently on federation plan 1")

                                    #last_ev_diag[tls_id] = {"eta": eta, "clrs": clrs, "tul": tul, "plan_type": plan.plan_type}

                                    last_contact = ag.get_last_ev_diag()
                                    
                                    diag = last_ev_diag.get(tls_id, {})
                                    print(f"Current request message: {ev_msg}")
                                    print(f"Current TLS: {ag.cfg.tls_id}, ag.active_ev: {ag.active_ev}")

                                    eta = ag._estimate_arrival_time(sim_time, ag.active_ev)  # or compute eta = sim_time + d_stop/max(speed,0.1)
                                    clrs = ag._compute_clrs_level(ev_edge)                   # yes "private", but fine for experiments
                                    tul  = ag._compute_tul_level(sim_time, eta) 

                                    trigger = False
                                    reasons = []

                                    if not diag:
                                        trigger, reasons = True, ["first_contact"]

                                    if diag and abs(eta - diag["eta"]) > 2.0:
                                        trigger, reasons = True, reasons + ["eta_drift"]

                                    if diag and clrs != diag["clrs"]:
                                        trigger, reasons = True, reasons + ["clrs_change"]

                                    # feasibility flip: does baseline predicted window cover arrival?
                                    target_phase = int(ag.active_ev.target_phase_idx or 0)
                                    base_win = ag._predict_next_phase_window(sim_time, target_phase)

                                    if base_win is not None:
                                        arrival_start, arrival_end = eta - ag.active_ev.delta_sec, eta + ag.active_ev.delta_sec
                                        if not (base_win[0] <= arrival_start and base_win[1] >= arrival_end):
                                            trigger, reasons = True, reasons + ["will_miss_window"]

                                    # --- only replan on trigger ---
                                    if trigger:
                                        plan = ag.tick(sim_time)
                                        if plan:
                                            #ag.active_ev.__dict__
                                            ag.apply_plan_to_tls(sim_time, plan)
                                            #print(ag.active_ev.__dict__)
                                            print(f"Proposed plan: {plan}")
                                            last_ev_diag[tls_id] = {"eta": eta, "clrs": clrs, "tul": tul, "plan_type": plan.plan_type}
                                    else:
                                        # keep executing the last committed plan (if any) without replanning
                                        if ag.current_plan:
                                            ag.apply_plan_to_tls(sim_time, ag.current_plan)

                                if CURRENT_EVALUATION == 'F2':
                                    print("Currently on federation approach 2 (what-if)")
                                    stage_before_tick = str(ag.stage)
                                    offers = ag.compute_offers(sim_time)
                                    stage_after_tick = str(ag.stage)
                                    print(f"[F2-STAGE] tls={tls_id} stage_before_tick={stage_before_tick} stage_after_tick={stage_after_tick}")
                                    print("\n+++++ OFFERS +++++\n")
                                    print(offers)
                                    print('\n')

                                    # Offer-based control
                                    #offers = ag.compute_offers(sim_time)

                                    if offers:
                                        offer_payloads = [_offer_to_jsonable(o) for o in offers]

                                        if ers_agent is not None:
                                            req_offer_msg = ers_agent.request_offer(
                                                ev_id=str(ev_id),
                                                tls_id=str(tls_id),
                                                sim_time=float(sim_time),
                                                approach_edge=str(selected_in_edge),
                                                route_intersections=list(route_nodes or []),
                                            )
                                            req_offer_msg["offers"] = list(offer_payloads)
                                            client.publish("ers/request_offer", json.dumps(req_offer_msg))

                                        # Publish offers for coordination (EV or middleware can select)
                                        client.publish(
                                            f"rw/agent/{tls_id}/offers",
                                            json.dumps({
                                                "step": step,
                                                "simTime": sim_time,
                                                "tlsId": tls_id,
                                                "evId": ev_id,
                                                "offers": offer_payloads,
                                            }),
                                        )

                                        # 1) Prepare optional external candidate (shadow/ERS).
                                        # Final F2 selection/refine happens inside IntersectionAgent.select_f2_offer().
                                        chosen_external = None
                                        print(f"\nChosen option at analytical: {ag.pick_best_offer(offers)}")
                                        
                                        # 2) Optional shadow simulations
                                        # Adaptive horizon: evaluate until (at least) end of EV arrival window,
                                        # bounded by --shadow-max-horizon to keep runtime predictable.
                                        eta_shadow = None
                                        eta_window_horizon = None
                                        adaptive_shadow_horizon = float(args.shadow_horizon)
                                        if ag.active_ev is not None:
                                            eta_shadow = float(ag._estimate_arrival_time(sim_time, ag.active_ev))
                                            eta_window_horizon = max(
                                                0.0,
                                                float(eta_shadow + float(ag.active_ev.delta_sec) - float(sim_time)),
                                            )
                                            adaptive_shadow_horizon = min(
                                                float(args.shadow_max_horizon),
                                                max(float(args.shadow_horizon), float(eta_window_horizon)),
                                            )

                                        if (shadow_pool
                                            and (not hasattr(shadow_pool, "has_live_workers") or shadow_pool.has_live_workers()) 
                                            and shadow_throttle.should_run(sim_time)):                                         

                                            if eta_window_horizon is not None and float(eta_window_horizon) > float(args.shadow_max_horizon):
                                                print(
                                                    f"[F2-SHADOW] tls={tls_id} skip rollout: "
                                                    f"eta_window_horizon={eta_window_horizon:.2f}s > shadow_max_horizon={float(args.shadow_max_horizon):.2f}s"
                                                )
                                            else:
                                                candidates = sorted(offers, key=ag.score_offer)[: max(1, args.shadow_topk)]
                                                state_path = save_sumo_state_snapshot(args.shadow_state_dir, sim_time)

                                                best_dict, shadow_results = shadow_pool.pick_best_offer(
                                                    base_state_path=state_path,
                                                    offers=candidates,
                                                    ev_id=ev_id,
                                                    tls_id=tls_id,
                                                    horizon_sec=float(adaptive_shadow_horizon),
                                                    timeout_sec=args.shadow_timeout,
                                                    edge_ids_for_cost=[ev_edge] if ev_edge else None,
                                                )

                                                # Per-offer rollout diagnostics for auditability.
                                                for sr in sorted(shadow_results, key=lambda x: float(getattr(x, "cost", float("inf")))):
                                                    od = dict(getattr(sr, "offer", {}) or {})
                                                    ev_tt = getattr(sr, "ev_travel_time", None)
                                                    ev_tt_str = "NA" if ev_tt is None else f"{float(ev_tt):.3f}"
                                                    print(
                                                        f"[F2-SHADOW] tls={tls_id} horizon={adaptive_shadow_horizon:.2f}s "
                                                        f"offer_id={od.get('offer_id', '')} action={od.get('action', '')} "
                                                        f"phase={od.get('target_phase_idx', '')} cost={float(getattr(sr, 'cost', float('inf'))):.4f} "
                                                        f"ev_tt={ev_tt_str} queue={float(getattr(sr, 'queue_cost', 0.0)):.4f}"
                                                    )

                                                if best_dict:
                                                    best_id = best_dict.get("offer_id")
                                                    chosen_shadow = next(
                                                        (o for o in candidates if getattr(o, "offer_id", None) == best_id),
                                                        None,
                                                    )
                                                    if chosen_shadow is None:
                                                        # fallback match if offer_id was regenerated in serialization
                                                        for o in candidates:
                                                            if (
                                                                getattr(o, "action", None) == best_dict.get("action")
                                                                and getattr(o, "target_phase_idx", None) == best_dict.get("target_phase_idx")
                                                            ):
                                                                chosen_shadow = o
                                                                break
                                                    if chosen_shadow is not None:
                                                        chosen_external = chosen_shadow

                                        # 2.5) ERS analytical selection (over same offer set)
                                        if ers_agent is not None:
                                            ers_choice = ers_agent.select_offer(
                                                ev_id=str(ev_id),
                                                tls_id=str(tls_id),
                                                sim_time=float(sim_time),
                                                offers=offer_payloads,
                                            )
                                            if ers_choice is not None:
                                                client.publish("ers/select_offer", json.dumps(ers_choice))
                                                ers_offer_id = str(ers_choice.get("selectedOfferId", ""))
                                                if ers_offer_id:
                                                    ers_offer = next(
                                                        (o for o in offers if str(getattr(o, "offer_id", "")) == ers_offer_id),
                                                        None,
                                                    )
                                                    if ers_offer is not None:
                                                        chosen_external = ers_offer
                                        
                                        print(f"\nChosen option at rollout: {chosen_external}")

                                        # 3) F2 selection/refinement owned by intersection agent.
                                        chosen, f2_meta = ag.select_f2_offer(
                                            sim_time=sim_time,
                                            ev_id=str(ev_id),
                                            offers=offers,
                                            external_offer=chosen_external,
                                        )

                                        # publish queued federation messages (reqs etc.)
                                        fed_msgs = ag.drain_federation_outbox()
                                        if fed_msgs:
                                            _fed_dbg_main(f"evt=OUTBOX_DRAIN tls={tls_id} n={len(fed_msgs)}")
                                        for fed_topic, fed_payload in fed_msgs:
                                            _fed_dbg_main(
                                                f"evt=PUB topic={fed_topic} req_id={fed_payload.get('req_id')} "
                                                f"from={fed_payload.get('from_tls')} to={fed_payload.get('to_tls')} "
                                                f"mode={fed_payload.get('mode')}"
                                            )
                                            client.publish(fed_topic, json.dumps(fed_payload))
                                        
                                        print(f"\nChosen option at federation: {chosen} meta={f2_meta}")

                                        # Chosen application after evaluation either anlaytical or using simulation states

                                        if chosen is not None and not STATIC_PROGRAM:
                                            # Even through some possibilities exists, not any action is applied.

                                            print(f"Applying offered selected")
                                            stage_before_apply_offer = str(ag.stage)
                                            ag.apply_offer_to_tls(sim_time, chosen)
                                            stage_after_apply_offer = str(ag.stage)
                                            print(f"[F2-STAGE] tls={tls_id} stage_after_apply_offer={stage_after_apply_offer} (before_apply={stage_before_apply_offer})")

                                            # Publish the selected offer & optional speed advice
                                            advice = None
                                            if getattr(chosen, "speed_range_mps", None) is not None:
                                                lo, hi = chosen.speed_range_mps
                                                advice = {
                                                    "min": lo,
                                                    "max": hi,
                                                    # simple point suggestion (mid)
                                                    "suggested": (float(lo) + float(hi)) / 2.0,
                                                }

                                            client.publish(
                                                f"rw/agent/{tls_id}/selected_offer",
                                                json.dumps({
                                                    "step": step,
                                                    "simTime": sim_time,
                                                    "tlsId": tls_id,
                                                    "evId": ev_id,
                                                    "selected": _offer_to_jsonable(chosen),
                                                    "speedAdvice": advice,
                                                }),
                                            )
                                        elif (not STATIC_PROGRAM) and (ag.current_plan is not None):
                                            # If agent-level F2 selection blocks actuation (e.g., infeasible),
                                            # keep local actuation continuity via current committed plan.
                                            try:
                                                ag.apply_plan_to_tls(
                                                    sim_time,
                                                    ag.current_plan,
                                                    decision_source="f2_selected_none",
                                                )
                                                _fed_dbg_main(
                                                    f"evt=F2_SELECTED_NONE_APPLY tls={tls_id} ev={ev_id} "
                                                    f"reason={str((f2_meta or {}).get('final_reason', ''))} sim={float(sim_time):.2f}"
                                                )
                                            except Exception as e:
                                                _fed_dbg_main(
                                                    f"evt=F2_SELECTED_NONE_WARN tls={tls_id} ev={ev_id} "
                                                    f"err={type(e).__name__}:{e}"
                                                )
                                    elif (not STATIC_PROGRAM) and (ag.current_plan is not None):
                                        # Local control fallback for F2:
                                        # federation/offer generation may be sparse between decision ticks,
                                        # but actuation must still be driven by local tick() logic.
                                        try:
                                            ag.apply_plan_to_tls(sim_time, ag.current_plan, decision_source="f2_local_fallback")
                                            _fed_dbg_main(
                                                f"evt=F2_LOCAL_FALLBACK_APPLY tls={tls_id} ev={ev_id} "
                                                f"plan_type={getattr(ag.current_plan, 'plan_type', None)} sim={float(sim_time):.2f}"
                                            )
                                        except Exception as e:
                                            _fed_dbg_main(
                                                f"evt=F2_LOCAL_FALLBACK_WARN tls={tls_id} ev={ev_id} "
                                                f"err={type(e).__name__}:{e}"
                                            )

                                    print(f"Node Ag: {ag.cfg.tls_id}, ag.ev_passed: {ag.ev_passed} current stage: {ag.stage}, current active_ev: {ag.active_ev}")
                                    print(f"Currently transitioning from Node: {ag.cfg.tls_id}")

                                    handoff_ev_id = ag.claim_pending_handoff_ev_id(sim_time)
                                    if handoff_ev_id is not None:
                                        handoff_key = (str(ag.cfg.tls_id), str(handoff_ev_id))
                                        if handoff_key not in handoff_sent:
                                            print(f"ag: {ag.cfg.tls_id}, ag.ev_passed: {ag.ev_passed}")
                                            print(f"------ EV passed from {ag.cfg.tls_id} ----- ")

                                            handoff_msgs = ag.build_handoff_messages(
                                                ev_id=str(handoff_ev_id),
                                                sim_time=sim_time
                                            )
                                            if ers_agent is not None:
                                                cand_tls = [dst for dst, _ in handoff_msgs]
                                                handoff_policy = ers_agent.handoff_policy(
                                                    ev_id=str(handoff_ev_id),
                                                    from_tls=str(ag.cfg.tls_id),
                                                    to_tls_candidates=cand_tls,
                                                    sim_time=float(sim_time),
                                                )
                                                client.publish("ers/handoff_policy", json.dumps(handoff_policy))
                                                allowed = set(handoff_policy.get("allowedNextTls", []))
                                                handoff_msgs = [(dst, msg_obj) for dst, msg_obj in handoff_msgs if dst in allowed]
                                            for dst_tls, msg_obj in handoff_msgs:
                                                print(
                                                    "[EV_HANDOFF_MSG] "
                                                    f"from_tls={ag.cfg.tls_id} "
                                                    f"to_tls={dst_tls} "
                                                    f"ev={handoff_ev_id} "
                                                    f"pass_t={msg_obj.get('pass_time')} "
                                                    f"pass_detect_t={msg_obj.get('pass_detect_time')} "
                                                    f"pass_proxy_t={msg_obj.get('pass_proxy_time')} "
                                                    f"edge_transition={msg_obj.get('pass_from_edge_id')}->{msg_obj.get('pass_to_edge_id')} "
                                                    f"current_edge={msg_obj.get('current_edge_id')} "
                                                    f"next_edge={msg_obj.get('next_edge_id')}"
                                                )
                                                _fed_dbg_main(f"evt=PUB topic=federation/handoff/{dst_tls} ev={msg_obj.get('ev_id')} from={msg_obj.get('from_tls')} to={dst_tls}")
                                                client.publish(f"federation/handoff/{dst_tls}", json.dumps(msg_obj))
                                            handoff_sent.add(handoff_key)                            

            # Backfill pass detection and handoff on non-current agents.
            # This preserves upstream pass visibility (e.g., Node2) after focus
            # moves to the next downstream node (e.g., Node6).
            if CURRENT_EVALUATION == "F2":
                for ag_id, ag_state in agents.items():
                    if ag_state is None:
                        continue
                    ag_id = str(ag_id)

                    if (
                        current_ev_tls_id is not None
                        and ag_id != str(current_ev_tls_id)
                        and ag_state.active_ev is not None
                        and (not ag_state.ev_passed)
                    ):
                        try:
                            passed_probe = bool(ag_state._detect_ev_passed(float(sim_time)))
                        except Exception as e:
                            passed_probe = False
                            print(f"[EV_PASS_BACKFILL][WARN] tls={ag_id} pass probe failed: {e}")
                        if passed_probe:
                            ag_state.ev_passed = True
                            print(
                                f"[EV_PASS_BACKFILL] tls={ag_id} ev={ag_state.active_ev.ev_id} "
                                f"t={float(sim_time):.2f} reason={getattr(ag_state, '_ev_pass_reason', '')}"
                            )

                    handoff_ev_id = ag_state.claim_pending_handoff_ev_id(sim_time)
                    if handoff_ev_id is not None:
                        handoff_key = (str(ag_state.cfg.tls_id), str(handoff_ev_id))
                        if handoff_key not in handoff_sent:
                            print(f"[EV_HANDOFF] tls={ag_state.cfg.tls_id} ev={handoff_ev_id} via=backfill")
                            handoff_msgs = ag_state.build_handoff_messages(
                                ev_id=str(handoff_ev_id),
                                sim_time=sim_time
                            )
                            if ers_agent is not None:
                                cand_tls = [dst for dst, _ in handoff_msgs]
                                handoff_policy = ers_agent.handoff_policy(
                                    ev_id=str(handoff_ev_id),
                                    from_tls=str(ag_state.cfg.tls_id),
                                    to_tls_candidates=cand_tls,
                                    sim_time=float(sim_time),
                                )
                                client.publish("ers/handoff_policy", json.dumps(handoff_policy))
                                allowed = set(handoff_policy.get("allowedNextTls", []))
                                handoff_msgs = [(dst, msg_obj) for dst, msg_obj in handoff_msgs if dst in allowed]
                            for dst_tls, msg_obj in handoff_msgs:
                                print(
                                    "[EV_HANDOFF_MSG] "
                                    f"from_tls={ag_state.cfg.tls_id} "
                                    f"to_tls={dst_tls} "
                                    f"ev={handoff_ev_id} "
                                    f"pass_t={msg_obj.get('pass_time')} "
                                    f"pass_detect_t={msg_obj.get('pass_detect_time')} "
                                    f"pass_proxy_t={msg_obj.get('pass_proxy_time')} "
                                    f"edge_transition={msg_obj.get('pass_from_edge_id')}->{msg_obj.get('pass_to_edge_id')} "
                                    f"current_edge={msg_obj.get('current_edge_id')} "
                                    f"next_edge={msg_obj.get('next_edge_id')}"
                                )
                                _fed_dbg_main(f"evt=PUB topic=federation/handoff/{dst_tls} ev={msg_obj.get('ev_id')} from={msg_obj.get('from_tls')} to={dst_tls}")
                                client.publish(f"federation/handoff/{dst_tls}", json.dumps(msg_obj))
                            handoff_sent.add(handoff_key)

            perf_acc["ev_logic"] += float(time.perf_counter() - t_ev_logic_wall)

            t_tls_pub_wall = time.perf_counter()
            if do_pub_tls_state or do_pub_tls_lanes:
                for tls_id in active_agent_tls_ids:
                    if tls_id not in live_tls_ids:
                        continue
                    if do_pub_tls_state:
                        tls_state = {
                            "tlsId": tls_id,
                            "step": step,
                            "simTime": sim_time,
                            "phase": int(traci.trafficlight.getPhase(tls_id)),
                            "program": str(traci.trafficlight.getProgram(tls_id)),
                            "rgy": str(traci.trafficlight.getRedYellowGreenState(tls_id)),
                            "nextSwitch": float(traci.trafficlight.getNextSwitch(tls_id)),
                            "controlsNodes": sorted(tls_to_nodes.get(tls_id, [])),
                        }
                        client.publish(f"rw/tls/{tls_id}/state", json.dumps(tls_state))

                    if do_pub_tls_lanes:
                        lanes = traci.trafficlight.getControlledLanes(tls_id)
                        lane_stats = {}
                        for ln in lanes:
                            try:
                                lane_len = float(traci.lane.getLength(ln))
                                veh_n = int(traci.lane.getLastStepVehicleNumber(ln))
                                occ_pct = float(traci.lane.getLastStepOccupancy(ln))
                                lane_stats[ln] = {
                                    "vehN": int(veh_n),
                                    "haltN": int(traci.lane.getLastStepHaltingNumber(ln)),
                                    "meanSpeed": float(traci.lane.getLastStepMeanSpeed(ln)),
                                    "occupancyPct": float(occ_pct),
                                    "densityVehPerKm": float((veh_n * 1000.0 / lane_len) if lane_len > 1e-6 else 0.0),
                                }
                            except Exception:
                                pass
                        client.publish(f"rw/tls/{tls_id}/lanes", json.dumps({
                            "tlsId": tls_id, "step": step, "simTime": sim_time, "lanes": lane_stats
                        }))
            perf_acc["tls_publish"] += float(time.perf_counter() - t_tls_pub_wall)

            t_sleep_print_wall = time.perf_counter()
            loop_sleep_sec = float(getattr(args, "main_loop_sleep_sec", 0.0))
            if loop_sleep_sec > 0.0:
                time.sleep(loop_sleep_sec)
            if do_print_step:
                print("\n --- Step", step, " SimTime:", sim_time, " ---\n")
            perf_acc["sleep_print"] += float(time.perf_counter() - t_sleep_print_wall)

            perf_acc["loop_total"] += float(time.perf_counter() - t_iter_wall)
            perf_steps_acc += 1
            if _periodic_due(sim_time, periodic_last_t, "perf_log", float(getattr(args, "perf_log_period_sec", -1.0))):
                n_steps = max(1, int(perf_steps_acc))
                keys = [
                    "sim_step",
                    "loop_poll",
                    "ev_logic",
                    "tls_publish",
                    "vehicle_io",
                    "cmd_dispatch",
                    "ev_update",
                    "warmup",
                    "sleep_print",
                    "loop_total",
                ]
                parts = []
                for k in keys:
                    if k in perf_acc:
                        parts.append(f"{k}={1000.0 * float(perf_acc.get(k, 0.0)) / n_steps:.1f}ms")
                print(f"[PERF] t={float(sim_time):.2f} steps={n_steps} " + " ".join(parts))
                perf_acc = defaultdict(float)
                perf_steps_acc = 0

            step += 1
            

    finally:
        try:
            if shadow_pool is not None:
                shadow_pool.close()
        except Exception:
            pass
        try:
            if ev_http_state_server is not None:
                ev_http_state_server.shutdown()
                ev_http_state_server.server_close()
        except Exception:
            pass
        try:
            if b1_worker_pool is not None:
                b1_worker_pool.close()
                b1_worker_pool.join()
        except Exception:
            pass

        try:
            traci.close()
        except Exception:
            pass
        try:
            if bool(getattr(args, "ev_kpi_debug", False)) or bool(ev_kpi_log_file):
                samples = int(ev_kpi_stats.get("samples", 0) or 0)
                speed_avg = (float(ev_kpi_stats.get("speed_sum", 0.0)) / float(samples)) if samples > 0 else None
                _ev_kpi_dbg(
                    "evt=EV_KPI_SUMMARY "
                    f"ev={ev_id} samples={samples} "
                    f"t_start={ev_kpi_stats.get('first_t')} t_end={ev_kpi_stats.get('last_t')} "
                    f"speed_avg={speed_avg} speed_min={ev_kpi_stats.get('speed_min')} speed_max={ev_kpi_stats.get('speed_max')} "
                    f"stop_time={float(ev_kpi_stats.get('stop_time_sec', 0.0)):.2f} "
                    f"slow_time={float(ev_kpi_stats.get('slow_time_sec', 0.0)):.2f} "
                    f"near_stopline_time={float(ev_kpi_stats.get('near_stopline_time_sec', 0.0)):.2f} "
                    f"blocked_near_stopline_time={float(ev_kpi_stats.get('blocked_near_stopline_time_sec', 0.0)):.2f}",
                    t_override=float(ev_kpi_stats.get("last_t") if ev_kpi_stats.get("last_t") is not None else -1.0),
                )
        except Exception:
            pass
        try:
            if ev_kpi_csv_file:
                _ev_kpi_write_csv(
                    ev_kpi_csv_file,
                    [
                        {
                            "t": f"{float(r.get('t', 0.0)):.3f}",
                            "step": int(r.get("step", 0)),
                            "edge": str(r.get("edge", "")),
                            "lane": str(r.get("lane", "")),
                            "laneIndex": int(r.get("laneIndex", 0)),
                            "lanePos": "" if r.get("lanePos") is None else f"{float(r['lanePos']):.3f}",
                            "x": f"{float(r.get('x', 0.0)):.3f}",
                            "y": f"{float(r.get('y', 0.0)):.3f}",
                            "speed_mps": f"{float(r.get('speed', 0.0)):.3f}",
                            "d_stop_m": f"{float(r.get('d_stop', 0.0)):.3f}",
                        }
                        for r in ev_kpi_samples
                    ],
                    ["t", "step", "edge", "lane", "laneIndex", "lanePos", "x", "y", "speed_mps", "d_stop_m"],
                )
            if ev_kpi_checkpoints_csv_file:
                _ev_kpi_write_csv(
                    ev_kpi_checkpoints_csv_file,
                    [
                        {
                            "t": f"{float(r.get('t', 0.0)):.3f}",
                            "step": int(r.get("step", 0)),
                            "ev": str(r.get("ev", "")),
                            "tls": str(r.get("tls", "")),
                            "edge": str(r.get("edge", "")),
                            "approach_node": str(r.get("approach_node", "")),
                            "agent_found": int(r.get("agent_found", 0)),
                        }
                        for r in ev_kpi_checkpoints
                    ],
                    ["t", "step", "ev", "tls", "edge", "approach_node", "agent_found"],
                )
            if ev_kpi_fig_dir and ev_kpi_samples:
                t0 = float(ev_kpi_samples[0].get("t", 0.0))
                speed_series = []
                dstop_series = []
                for r in ev_kpi_samples:
                    try:
                        tx = float(r.get("t", 0.0)) - t0
                        speed_series.append((tx, float(r.get("speed", 0.0))))
                        dstop_series.append((tx, float(r.get("d_stop", 0.0))))
                    except Exception:
                        pass
                _ev_kpi_svg_line_plot(
                    os.path.join(ev_kpi_fig_dir, "ev_speed.svg"),
                    "EV Speed Over Time",
                    "Time Since First EV KPI Sample (s)",
                    "Speed (m/s)",
                    speed_series,
                )
                _ev_kpi_svg_line_plot(
                    os.path.join(ev_kpi_fig_dir, "ev_dstop.svg"),
                    "EV Distance to Stopline Over Time",
                    "Time Since First EV KPI Sample (s)",
                    "Distance to Stopline (m)",
                    dstop_series,
                )
        except Exception as e:
            try:
                print(f"[EV_KPI][WARN] artifact export failed: {type(e).__name__}:{e}")
            except Exception:
                pass
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    try:
        mp.set_start_method("spawn")
    except RuntimeError:
        pass
    main()
