import argparse
import csv
import hashlib
import json
import math
import os
import shlex
import socket
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Sequence, Tuple
from urllib import error as url_error
from urllib import request as url_request
from urllib.parse import parse_qs, urlparse
import sys
import paho.mqtt.client as mqtt
import traci
import traci.constants as tc
import os, glob
import multiprocessing as mp

from shadow_rollout_workers import ShadowRolloutPool, ShadowRolloutPoolConfig
import intersection_agent as intersection_agent_module
from intersection_agent import IntersectionAgent, IntersectionAgentConfig, EvRequest, PassiveIntersectionDT
from vehicle_agent import EmergencyVehicleAgent, EmergencyVehicleProfile
from ers_agent import EmergencyResponseSystemAgent, ERSConfig

# Toy experiments with DT
#from intersection_agent_DT import IntersectionAgent_DT, IntersectionAgentConfig, EvRequest

STATIC_PROGRAM = False


def _short_mqtt_client_id(prefix: str, seed: str = "", max_len: int = 16) -> str:
    p = "".join(ch for ch in str(prefix or "rw") if ch.isalnum())[:4] or "rw"
    material = f"{prefix}:{seed}:{os.getpid()}:{uuid.uuid4().hex}:{time.time_ns()}"
    digest = hashlib.sha1(material.encode("utf-8")).hexdigest()[: max(6, int(max_len) - len(p))]
    return (p + digest)[: int(max_len)]


def _make_mqtt_client(client_id: str):
    kwargs = {"client_id": str(client_id or "").strip()} if str(client_id or "").strip() else {}
    protocol = str(os.environ.get("FNM_MQTT_PROTOCOL") or os.environ.get("FED_MQTT_PROTOCOL") or "").strip().lower()
    if protocol in {"mqttv5", "v5", "5"} and hasattr(mqtt, "MQTTv5"):
        kwargs["protocol"] = mqtt.MQTTv5
    elif protocol in {"mqttv311", "v311", "3.1.1", "311"} and hasattr(mqtt, "MQTTv311"):
        kwargs["protocol"] = mqtt.MQTTv311
    elif protocol in {"mqttv31", "v31", "3.1", "31"} and hasattr(mqtt, "MQTTv31"):
        kwargs["protocol"] = mqtt.MQTTv31
    cb_versions = getattr(mqtt, "CallbackAPIVersion", None)
    cb_api = getattr(cb_versions, "VERSION1", None)
    if cb_api is not None:
        try:
            return mqtt.Client(cb_api, **kwargs)
        except TypeError:
            pass
    try:
        return mqtt.Client(**kwargs)
    except TypeError:
        cb_api = getattr(cb_versions, "VERSION2", None)
        if cb_api is not None:
            try:
                return mqtt.Client(cb_api, **kwargs)
            except TypeError:
                pass
        if "client_id" in kwargs:
            return mqtt.Client(kwargs["client_id"])
        return mqtt.Client()
inspect_node = "Node6"

MODE_EVALUATION = ["B0", "B1", "F1", "F2", "F2P", "F2P-Q", "F2D", "F2D-Q", "F2PD", "F3"]
CURRENT_EVALUATION = "F2"


F2_FAMILY_EVALUATIONS = {"F2", "F2P", "F2P-Q", "F2D", "F2D-Q", "F2PD"}
PASSIVE_DT_EVALUATIONS = {"F2P", "F2P-Q", "F2PD"}
DRONE_AUGMENTED_EVALUATIONS = {"F2D", "F2D-Q", "F2PD"}
F2P_QUEUE_RELEASE_EVALUATIONS = {"F2P-Q"}
F2D_QUEUE_RELEASE_EVALUATIONS = {"F2D-Q"}


def _evaluation_family(mode: str) -> str:
    mode_u = str(mode or "").upper()
    if mode_u in F2_FAMILY_EVALUATIONS:
        return "F2"
    return mode_u


def _is_f2_family(mode: str) -> bool:
    return _evaluation_family(mode) == "F2"


def _is_passive_dt_mode(mode: str) -> bool:
    return str(mode or "").upper() in PASSIVE_DT_EVALUATIONS


def _is_drone_augmented_mode(mode: str) -> bool:
    return str(mode or "").upper() in DRONE_AUGMENTED_EVALUATIONS


def _is_f2p_queue_release_mode(mode: str) -> bool:
    return str(mode or "").upper() in F2P_QUEUE_RELEASE_EVALUATIONS


def _is_f2d_queue_release_mode(mode: str) -> bool:
    return str(mode or "").upper() in F2D_QUEUE_RELEASE_EVALUATIONS


def _is_f2d_prescout_mode(mode: str) -> bool:
    return str(mode or "").upper() == "F2D"


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
    ap.add_argument("--mqtt-port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument(
        "--mqtt-topic-namespace",
        default="",
        help="optional topic namespace prefix for run isolation (applied to all MQTT publish/subscribe)",
    )
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
        "--max-sim-time-sec",
        type=float,
        default=0.0,
        help=(
            "hard stop at this SUMO simulation time in seconds; 0 disables. "
            "Useful for censoring EV non-arrival/gridlock screening runs without unbounded logs."
        ),
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
        "--fed-ev-request-deterministic-apply-enable",
        dest="fed_ev_request_deterministic_apply_enable",
        action="store_true",
        default=False,
        help=(
            "buffer MQTT EV requests and release them at deterministic SUMO sim-time barriers; "
            "useful for repeatability when FNM/HTTP/MQTT timing otherwise races the control loop"
        ),
    )
    ap.add_argument(
        "--no-fed-ev-request-deterministic-apply-enable",
        dest="fed_ev_request_deterministic_apply_enable",
        action="store_false",
        help="disable deterministic sim-time buffering for MQTT EV requests",
    )
    ap.add_argument(
        "--fed-ev-request-deterministic-grace-sec",
        type=float,
        default=0.25,
        help="sim-time grace window before a buffered MQTT EV request becomes eligible for dispatch",
    )
    ap.add_argument(
        "--fed-ev-request-deterministic-max-buffer-sec",
        type=float,
        default=3.0,
        help="maximum sim-time age before a buffered MQTT EV request is force-released",
    )
    ap.add_argument(
        "--fed-ev-request-wait-for-fnm-enable",
        dest="fed_ev_request_wait_for_fnm_enable",
        action="store_true",
        default=False,
        help=(
            "when internal EV request generation is disabled, briefly hold the SUMO control loop at a trigger "
            "window until the matching FNM-originated MQTT EV request arrives"
        ),
    )
    ap.add_argument(
        "--no-fed-ev-request-wait-for-fnm-enable",
        dest="fed_ev_request_wait_for_fnm_enable",
        action="store_false",
        help="disable bounded wait for matching FNM-originated EV requests",
    )
    ap.add_argument(
        "--fed-ev-request-wait-for-fnm-timeout-sec",
        type=float,
        default=0.0,
        help="maximum wall-clock seconds to wait for a matching FNM EV request at a SUMO trigger window",
    )
    ap.add_argument(
        "--fed-ev-request-wait-for-fnm-poll-sec",
        type=float,
        default=0.01,
        help="wall-clock polling interval while waiting for a matching FNM EV request",
    )
    ap.add_argument(
        "--fed-ev-request-wait-for-fnm-retry-sim-sec",
        type=float,
        default=0.5,
        help="minimum sim-time gap before retrying a timed-out FNM request wait for the same EV/TLS",
    )
    ap.add_argument(
        "--fed-ev-request-wait-for-fnm-raw-dispatch-enable",
        dest="fed_ev_request_wait_for_fnm_raw_dispatch_enable",
        action="store_true",
        default=False,
        help=(
            "allow the wall-clock FNM wait path to dispatch matching raw MQTT EV requests directly from "
            "the receive queue; disabled by default so deterministic sim-time barrier mode cannot be bypassed"
        ),
    )
    ap.add_argument(
        "--no-fed-ev-request-wait-for-fnm-raw-dispatch-enable",
        dest="fed_ev_request_wait_for_fnm_raw_dispatch_enable",
        action="store_false",
        help="keep matching FNM EV requests behind the deterministic sim-time barrier during wall-clock waits",
    )
    ap.add_argument(
        "--strict-foreign-ev-filter",
        dest="strict_foreign_ev_filter",
        action="store_true",
        default=True,
        help="drop inbound federated messages carrying ev_id different from --emergency-veh (recommended for matrix runs)",
    )
    ap.add_argument(
        "--no-strict-foreign-ev-filter",
        dest="strict_foreign_ev_filter",
        action="store_false",
        help="disable strict ev_id filtering on inbound federated messages",
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
        "--passive-intersection-dt-enable",
        action="store_true",
        default=False,
        help="instantiate context-only passive DTs for non-TLS junctions traversed by the EV route",
    )
    ap.add_argument(
        "--passive-intersection-context-period-sec",
        type=float,
        default=1.0,
        help="simulation-time period for passive non-TLS intersection context publication",
    )
    ap.add_argument(
        "--passive-intersection-max-nodes",
        type=int,
        default=0,
        help="maximum passive non-TLS route nodes to instantiate; 0 means all eligible route nodes",
    )
    ap.add_argument(
        "--passive-intersection-lookahead-edges",
        type=int,
        default=3,
        help="number of route edges each passive intersection DT observes around its junction",
    )
    ap.add_argument(
        "--passive-intersection-context-route-fanout-enable",
        dest="passive_intersection_context_route_fanout_enable",
        action="store_true",
        default=True,
        help="deliver passive context only to active TLS agents near the passive node route segment",
    )
    ap.add_argument(
        "--no-passive-intersection-context-route-fanout-enable",
        dest="passive_intersection_context_route_fanout_enable",
        action="store_false",
        help="broadcast passive context to all active TLS agents",
    )
    ap.add_argument(
        "--passive-intersection-context-fanout-back-edges",
        type=int,
        default=2,
        help="route edges behind a passive observation used when selecting TLS fanout targets",
    )
    ap.add_argument(
        "--passive-intersection-context-fanout-forward-edges",
        type=int,
        default=4,
        help="route edges ahead of a passive observation used when selecting TLS fanout targets",
    )
    ap.add_argument(
        "--f2p-passive-context-policy",
        choices=[
            "disabled",
            "missing_feedback_only",
            "severe_or_missing",
            "immediate_missing_severe",
            "always",
        ],
        default="immediate_missing_severe",
        help=(
            "how active TLS agents fuse passive non-TLS context in F2P/F2PD. "
            "immediate_missing_severe uses passive context only as a nearby severe missing-feedback rescue"
        ),
    )
    ap.add_argument(
        "--f2p-passive-context-max-age-sec",
        type=float,
        default=5.0,
        help="maximum age of passive non-TLS context used by active TLS F2P decision scoring",
    )
    ap.add_argument(
        "--f2p-passive-context-lookahead-edges",
        type=int,
        default=4,
        help="maximum downstream route edges considered when matching passive context to an active TLS decision",
    )
    ap.add_argument(
        "--f2p-passive-context-max-worst-edge-offset",
        type=int,
        default=1,
        help="maximum passive worst-edge route offset that can affect F2P decision scoring",
    )
    ap.add_argument(
        "--f2p-active-tls-metering-floor-enable",
        dest="f2p_active_tls_metering_floor_enable",
        action="store_true",
        default=True,
        help=(
            "in F2P/F2PD, preserve the normal F2 downstream metering window when "
            "the blocked downstream edge is controlled by an active TLS"
        ),
    )
    ap.add_argument(
        "--no-f2p-active-tls-metering-floor-enable",
        dest="f2p_active_tls_metering_floor_enable",
        action="store_false",
        help="allow passive F2P nearfield policy to release active-TLS downstream blockage earlier than F2",
    )
    ap.add_argument(
        "--f2p-active-tls-metering-floor-max-worst-edge-offset",
        type=int,
        default=1,
        help=(
            "maximum route offset for preserving the normal F2 metering floor when "
            "passive F2P context reports a blocked edge controlled by an active TLS"
        ),
    )
    ap.add_argument(
        "--f2p-passive-context-severe-min-halt-n",
        type=int,
        default=4,
        help="passive context is severe if halting vehicles reach this count",
    )
    ap.add_argument(
        "--f2p-passive-context-severe-min-veh-n",
        type=int,
        default=6,
        help="passive context low-speed severity requires at least this many vehicles",
    )
    ap.add_argument(
        "--f2p-passive-context-severe-max-mean-speed-mps",
        type=float,
        default=0.5,
        help="passive context low-speed severity threshold",
    )
    ap.add_argument(
        "--f2p-passive-context-severe-max-occupancy-pct",
        type=float,
        default=45.0,
        help="passive context severe occupancy threshold",
    )
    ap.add_argument(
        "--f2p-passive-context-missing-feedback-floor-enable",
        dest="f2p_passive_context_missing_feedback_floor_enable",
        action="store_true",
        default=True,
        help="cap passive-context scoring penalties when passive context only substitutes missing active feedback",
    )
    ap.add_argument(
        "--no-f2p-passive-context-missing-feedback-floor-enable",
        dest="f2p_passive_context_missing_feedback_floor_enable",
        action="store_false",
        help="disable the F2P missing-feedback passive-context penalty cap",
    )
    ap.add_argument(
        "--f2p-passive-context-missing-feedback-max-queue-deficit-sec",
        type=float,
        default=2.0,
        help="maximum queue-deficit penalty seconds contributed by passive context when active feedback is missing",
    )
    ap.add_argument(
        "--f2p-passive-context-missing-feedback-max-spillback-risk",
        type=float,
        default=0.15,
        help="maximum spillback-risk scoring contribution from passive context when active feedback is missing",
    )
    ap.add_argument(
        "--f2p-passive-context-missing-feedback-max-timing-sec",
        type=float,
        default=1.0,
        help="maximum downstream timing penalty seconds from passive context when active feedback is missing",
    )
    ap.add_argument(
        "--f2p-passive-context-clear-missing-feedback-enable",
        dest="f2p_passive_context_clear_missing_feedback_enable",
        action="store_true",
        default=True,
        help="allow clear passive context to reduce missing-peer-feedback uncertainty in F2P/F2PD",
    )
    ap.add_argument(
        "--no-f2p-passive-context-clear-missing-feedback-enable",
        dest="f2p_passive_context_clear_missing_feedback_enable",
        action="store_false",
        help="ignore clear passive context when active peer feedback is missing",
    )
    ap.add_argument(
        "--f2p-passive-context-clear-missing-feedback-no-feedback-penalty",
        type=float,
        default=0.25,
        help="remaining no-feedback scoring penalty when clear passive context substitutes missing active feedback",
    )
    ap.add_argument(
        "--f2p-passive-stall-rescue-enable",
        dest="f2p_passive_stall_rescue_enable",
        action="store_true",
        default=True,
        help=(
            "in F2P/F2PD, allow a bounded B1/F2 continuity apply when passive "
            "near-field context keeps vetoing the current TLS while the EV is stalled"
        ),
    )
    ap.add_argument(
        "--no-f2p-passive-stall-rescue-enable",
        dest="f2p_passive_stall_rescue_enable",
        action="store_false",
        help="disable F2P passive-stall rescue fallback",
    )
    ap.add_argument(
        "--f2p-passive-stall-rescue-min-blocked-sec",
        type=float,
        default=6.0,
        help="seconds of repeated passive near-field downstream veto before F2P permits continuity rescue",
    )
    ap.add_argument(
        "--f2p-passive-stall-rescue-max-speed-mps",
        type=float,
        default=0.5,
        help="maximum EV speed considered stalled for F2P passive-stall rescue",
    )
    ap.add_argument(
        "--f2p-passive-stall-rescue-require-selected-edge",
        dest="f2p_passive_stall_rescue_require_selected_edge",
        action="store_true",
        default=True,
        help="only permit F2P passive-stall rescue when the EV is on the selected inbound edge for the current TLS",
    )
    ap.add_argument(
        "--no-f2p-passive-stall-rescue-require-selected-edge",
        dest="f2p_passive_stall_rescue_require_selected_edge",
        action="store_false",
        help="allow F2P passive-stall rescue even when the EV is upstream of the selected inbound edge",
    )
    ap.add_argument(
        "--f2p-queue-release-enable",
        dest="f2p_queue_release_enable",
        action="store_true",
        default=False,
        help=(
            "enable experimental F2P-Q queue-release actuation: when passive "
            "non-TLS context identifies a blocked edge controlled by an active "
            "downstream TLS, briefly select the green phase for that inbound edge. "
            "Plain F2P remains observability/guard-only unless this knob or "
            "evaluation=F2P-Q is used."
        ),
    )
    ap.add_argument(
        "--no-f2p-queue-release-enable",
        dest="f2p_queue_release_enable",
        action="store_false",
        help="disable F2P-Q queue-release actuation even when passive context is present",
    )
    ap.add_argument(
        "--f2p-queue-release-hold-sec",
        type=float,
        default=3.0,
        help="bounded green hold duration applied by experimental F2P-Q queue release",
    )
    ap.add_argument(
        "--f2p-queue-release-min-interval-sec",
        type=float,
        default=3.0,
        help="minimum repeat interval per passive source/TLS/edge for F2P-Q queue release",
    )
    ap.add_argument(
        "--f2p-queue-release-max-worst-edge-offset",
        type=int,
        default=4,
        help="maximum passive-context worst-edge route offset eligible for F2P-Q queue release",
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
        "--realtime-sumo-enable",
        action="store_true",
        default=False,
        help="pace the SUMO loop so simulation time follows wall-clock time; useful for physical drone F2D runs",
    )
    ap.add_argument(
        "--realtime-sumo-factor",
        type=float,
        default=1.0,
        help="wall-clock pacing factor when --realtime-sumo-enable is set; 1.0 means 1 sim-second per 1 wall-second",
    )
    ap.add_argument(
        "--realtime-sumo-max-sleep-sec",
        type=float,
        default=0.5,
        help="maximum wall-clock sleep inserted after one SUMO step for real-time pacing",
    )
    ap.add_argument(
        "--realtime-sumo-log-period-sec",
        type=float,
        default=5.0,
        help="simulation-time period for real-time pacing trace logs; <=0 disables periodic logs",
    )
    ap.add_argument(
        "--realtime-sumo-start-sim-time-sec",
        type=float,
        default=0.0,
        help=(
            "simulation time at which real-time SUMO pacing starts. "
            "Use >0 for fast pre-roll before physical F2D drone synchronization."
        ),
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
        "--b1-downstream-blockage-guard-enable",
        dest="b1_downstream_blockage_guard_enable",
        action="store_true",
        default=False,
        help="prevent aggressive B1 local-priority plans when near downstream route edges already look blocked",
    )
    ap.add_argument(
        "--no-b1-downstream-blockage-guard-enable",
        dest="b1_downstream_blockage_guard_enable",
        action="store_false",
        help="disable the B1 downstream blockage guard",
    )
    ap.add_argument(
        "--b1-downstream-blockage-lookahead-edges",
        type=int,
        default=3,
        help="number of downstream EV route edges inspected by the B1 blockage guard",
    )
    ap.add_argument(
        "--b1-downstream-blockage-min-halt-n",
        type=int,
        default=3,
        help="minimum halting vehicles on a downstream edge to classify B1 downstream blockage",
    )
    ap.add_argument(
        "--b1-downstream-blockage-max-mean-speed-mps",
        type=float,
        default=1.0,
        help="mean-speed threshold for downstream queue classification when vehicles are present",
    )
    ap.add_argument(
        "--b1-downstream-blockage-min-veh-n",
        type=int,
        default=2,
        help="minimum vehicles on a downstream edge for speed-based B1 blockage classification",
    )
    ap.add_argument(
        "--b1-downstream-blockage-max-occupancy-pct",
        type=float,
        default=35.0,
        help="occupancy percent threshold for downstream B1 blockage classification",
    )
    ap.add_argument(
        "--b1-strict-local-baseline-enable",
        dest="b1_strict_local_baseline_enable",
        action="store_true",
        default=False,
        help=(
            "make B1 a strict one-hop EV-to-current-TLS baseline: no downstream TLS fallback "
            "through non-TLS nodes, no route/corridor hints in EV requests, no high-rate "
            "TraCI refresh after FNM delivery, and no multi-edge downstream blockage guard"
        ),
    )
    ap.add_argument(
        "--no-b1-strict-local-baseline-enable",
        dest="b1_strict_local_baseline_enable",
        action="store_false",
        help="restore legacy B1 behavior with downstream TLS fallback and route-aware local guards",
    )
    ap.add_argument(
        "--ev-intersection-discovery-enable",
        dest="ev_intersection_discovery_enable",
        action="store_true",
        default=False,
        help=(
            "require an explicit EV-to-current-intersection discovery gate before emitting "
            "EV priority requests in selected modes"
        ),
    )
    ap.add_argument(
        "--no-ev-intersection-discovery-enable",
        dest="ev_intersection_discovery_enable",
        action="store_false",
        help="disable the EV-to-current-intersection discovery gate",
    )
    ap.add_argument(
        "--ev-intersection-discovery-delay-sec",
        type=float,
        default=0.0,
        help="simulation-time delay added between EV discovery query and request emission",
    )
    ap.add_argument(
        "--ev-intersection-discovery-modes",
        default="B1",
        help="comma-separated evaluation modes where EV-to-intersection discovery gates request emission",
    )
    ap.add_argument(
        "--ev-intersection-discovery-repeat-scope",
        choices=["tls", "edge"],
        default="edge",
        help="repeat discovery once per TLS or once per TLS approach edge",
    )
    ap.add_argument(
        "--ev-intersection-discovery-wait-log-period-sec",
        type=float,
        default=1.0,
        help="minimum sim-time spacing for repeated discovery wait traces",
    )
    ap.add_argument(
        "--downstream-immediate-blockage-guard-enable",
        dest="downstream_immediate_blockage_guard_enable",
        action="store_true",
        default=True,
        help=(
            "suppress active B1/F2 priority when the worst blocked edge is immediately downstream; "
            "this avoids pushing the EV into a queue that cannot absorb it"
        ),
    )
    ap.add_argument(
        "--no-downstream-immediate-blockage-guard-enable",
        dest="downstream_immediate_blockage_guard_enable",
        action="store_false",
        help="disable the shared immediate-downstream blockage guard",
    )
    ap.add_argument(
        "--downstream-immediate-blockage-max-worst-edge-offset",
        type=int,
        default=1,
        help="maximum downstream edge offset considered immediate for the shared blockage guard",
    )
    ap.add_argument(
        "--downstream-immediate-blockage-min-halt-n",
        type=int,
        default=3,
        help="minimum halted vehicles on the immediate downstream edge for severe blockage suppression",
    )
    ap.add_argument(
        "--downstream-immediate-blockage-min-veh-n",
        type=int,
        default=6,
        help="minimum vehicles on the immediate downstream edge for severe blockage suppression",
    )
    ap.add_argument(
        "--downstream-immediate-blockage-max-mean-speed-mps",
        type=float,
        default=0.5,
        help="maximum mean speed on the immediate downstream edge for severe blockage suppression",
    )
    ap.add_argument(
        "--external-downstream-context-enable",
        dest="external_downstream_context_enable",
        action="store_true",
        default=False,
        help=(
            "enable optional external downstream context providers such as Drone-DTs; "
            "disabled by default so B0/B1/F2 remain comparable without drone augmentation"
        ),
    )
    ap.add_argument(
        "--no-external-downstream-context-enable",
        dest="external_downstream_context_enable",
        action="store_false",
        help="disable external drone/mobile downstream context merge",
    )
    ap.add_argument(
        "--external-downstream-context-max-age-sec",
        type=float,
        default=2.0,
        help="maximum age of external downstream context accepted by B1/F2 guards",
    )
    ap.add_argument(
        "--f2-drone-context-request-enable",
        dest="f2_drone_context_request_enable",
        action="store_true",
        default=False,
        help=(
            "enable F2 intersection-agent requests to Drone-DTs when peer/downstream "
            "context is missing or stale"
        ),
    )
    ap.add_argument(
        "--no-f2-drone-context-request-enable",
        dest="f2_drone_context_request_enable",
        action="store_false",
        help="disable F2 Drone-DT downstream context requests",
    )
    ap.add_argument(
        "--f2-drone-context-provider-id",
        default="crazyflie_01",
        help="Drone-DT provider id used for downstream inspection requests",
    )
    ap.add_argument(
        "--f2-drone-context-request-ttl-sec",
        type=float,
        default=3.0,
        help="decision deadline / TTL for Drone-DT downstream inspection requests",
    )
    ap.add_argument(
        "--f2-drone-context-request-min-interval-sec",
        type=float,
        default=3.0,
        help="minimum interval between repeated Drone-DT requests per EV/provider/reason",
    )
    ap.add_argument(
        "--f2-drone-context-request-max-edges",
        type=int,
        default=8,
        help="maximum EV route edges included in a Drone-DT downstream inspection request",
    )
    ap.add_argument(
        "--f2-drone-context-include-route-context",
        dest="f2_drone_context_include_route_context",
        action="store_true",
        default=True,
        help=(
            "include the EV route and remaining EV route as metadata in F2D Drone-DT "
            "inspection requests; only active when drone requests are enabled"
        ),
    )
    ap.add_argument(
        "--no-f2-drone-context-include-route-context",
        dest="f2_drone_context_include_route_context",
        action="store_false",
        help="omit full/remaining EV route metadata from Drone-DT inspection requests",
    )
    ap.add_argument(
        "--f2-drone-context-route-context-max-edges",
        type=int,
        default=64,
        help="maximum full/remaining EV route edges serialized as Drone-DT route context metadata",
    )
    ap.add_argument(
        "--f2-drone-context-emit-discovery-query",
        dest="f2_drone_context_emit_discovery_query",
        action="store_true",
        default=True,
        help="emit a federation discovery query before the direct Drone-DT request",
    )
    ap.add_argument(
        "--no-f2-drone-context-emit-discovery-query",
        dest="f2_drone_context_emit_discovery_query",
        action="store_false",
        help="skip the discovery-query hint and emit only the direct Drone-DT request",
    )
    ap.add_argument(
        "--f2-drone-context-discovery-gate-enable",
        dest="f2_drone_context_discovery_gate_enable",
        action="store_true",
        default=False,
        help=(
            "F2D-only: require a recent discovery response for an AerialScoutSystem "
            "downstream-context provider before emitting the Drone-DT inspection request"
        ),
    )
    ap.add_argument(
        "--no-f2-drone-context-discovery-gate-enable",
        dest="f2_drone_context_discovery_gate_enable",
        action="store_false",
        help="disable discovery-gated Drone-DT provider selection",
    )
    ap.add_argument(
        "--f2-drone-context-discovery-cache-ttl-sec",
        type=float,
        default=5.0,
        help="maximum age of cached Drone-DT discovery responses accepted by SI-DTs",
    )
    ap.add_argument(
        "--f2-drone-context-discovery-query-min-interval-sec",
        type=float,
        default=1.0,
        help="minimum repeated SI-DT drone discovery query interval while awaiting a provider",
    )
    ap.add_argument(
        "--f2d-mobile-passive-context-enable",
        dest="f2d_mobile_passive_context_enable",
        action="store_true",
        default=True,
        help=(
            "in F2D/F2PD only, treat fresh Drone-DT downstream context as mobile "
            "passive observability and fan it out to active TLS on inspected route edges"
        ),
    )
    ap.add_argument(
        "--no-f2d-mobile-passive-context-enable",
        dest="f2d_mobile_passive_context_enable",
        action="store_false",
        help="disable F2D mobile-passive context fanout while keeping direct drone context reception",
    )
    ap.add_argument(
        "--f2d-directed-context-delivery-enable",
        dest="f2d_directed_context_delivery_enable",
        action="store_true",
        default=True,
        help=(
            "F2D-only: deliver Drone-DT observations as directed SI-DT downstream-context "
            "artifacts before they enter each intersection decision cache"
        ),
    )
    ap.add_argument(
        "--no-f2d-directed-context-delivery-enable",
        dest="f2d_directed_context_delivery_enable",
        action="store_false",
        help="disable directed SI-DT context delivery and use the legacy in-process F2D cache fanout",
    )
    ap.add_argument(
        "--f2d-directed-context-self-delivery-enable",
        dest="f2d_directed_context_self_delivery_enable",
        action="store_true",
        default=False,
        help=(
            "debug fallback: allow real-world to consume its own directed federation context "
            "artifact without waiting for the intersection FNM local delivery"
        ),
    )
    ap.add_argument(
        "--no-f2d-directed-context-self-delivery-enable",
        dest="f2d_directed_context_self_delivery_enable",
        action="store_false",
        help="require directed F2D context to arrive through the local SI-DT/FNM delivery topic",
    )
    ap.add_argument(
        "--f2d-contextual-topic-delivery-enable",
        dest="f2d_contextual_topic_delivery_enable",
        action="store_true",
        default=False,
        help=(
            "F2D-only: use Drone-DT node/edge/region contextual topics plus intersection FNM "
            "subscriptions as the SI-DT delivery path; the generic drone provider topic becomes trace-only"
        ),
    )
    ap.add_argument(
        "--no-f2d-contextual-topic-delivery-enable",
        dest="f2d_contextual_topic_delivery_enable",
        action="store_false",
        help="use directed SI-DT fanout from the generic Drone-DT provider topic",
    )
    ap.add_argument(
        "--f2d-advisory-reroute-enable",
        dest="f2d_advisory_reroute_enable",
        action="store_true",
        default=True,
        help=(
            "in F2D/F2PD only, emit advisory reroute traces when Drone-DT context "
            "indicates a downstream blockage beyond local TLS clearing capability"
        ),
    )
    ap.add_argument(
        "--no-f2d-advisory-reroute-enable",
        dest="f2d_advisory_reroute_enable",
        action="store_false",
        help="disable F2D reroute advisory traces",
    )
    ap.add_argument(
        "--f2d-advisory-reroute-min-worst-edge-offset",
        type=int,
        default=2,
        help="minimum blocked downstream edge offset that triggers F2D advisory reroute traces",
    )
    ap.add_argument(
        "--f2d-queue-release-enable",
        dest="f2d_queue_release_enable",
        action="store_true",
        default=False,
        help=(
            "enable experimental F2D-Q queue-release actuation: when Drone-DT context "
            "identifies a blocked edge controlled by an active downstream TLS, briefly "
            "select the green phase for that inbound edge. F2D remains advisory unless "
            "this knob or evaluation=F2D-Q is used."
        ),
    )
    ap.add_argument(
        "--no-f2d-queue-release-enable",
        dest="f2d_queue_release_enable",
        action="store_false",
        help="disable F2D-Q queue-release actuation even when drone context is present",
    )
    ap.add_argument(
        "--f2d-queue-release-hold-sec",
        type=float,
        default=3.0,
        help="bounded green hold duration applied by experimental F2D-Q queue release",
    )
    ap.add_argument(
        "--f2d-queue-release-min-interval-sec",
        type=float,
        default=3.0,
        help="minimum repeat interval per EV/TLS/edge for F2D-Q queue-release actuation",
    )
    ap.add_argument(
        "--f2d-queue-release-max-worst-edge-offset",
        type=int,
        default=8,
        help="maximum drone-reported worst-edge route offset eligible for F2D-Q queue release",
    )
    ap.add_argument(
        "--f2d-drone-prescout-enable",
        dest="f2d_drone_prescout_enable",
        action="store_true",
        default=False,
        help=(
            "enable F2D-only proactive Drone-DT scouting when the EV reaches the "
            "first active TLS on its route"
        ),
    )
    ap.add_argument(
        "--no-f2d-drone-prescout-enable",
        dest="f2d_drone_prescout_enable",
        action="store_false",
        help="disable proactive F2D Drone-DT pre-scouting",
    )
    ap.add_argument(
        "--f2d-drone-prescout-first-tls-only",
        dest="f2d_drone_prescout_first_tls_only",
        action="store_true",
        default=True,
        help="only allow the first active route TLS to emit the proactive F2D drone scout request",
    )
    ap.add_argument(
        "--no-f2d-drone-prescout-first-tls-only",
        dest="f2d_drone_prescout_first_tls_only",
        action="store_false",
        help="allow any active TLS receiving the EV request to emit a proactive F2D drone scout request",
    )
    ap.add_argument(
        "--f2d-drone-prescout-max-edges",
        type=int,
        default=16,
        help="maximum route-ahead edges in proactive F2D Drone-DT pre-scout requests",
    )
    ap.add_argument(
        "--f2d-drone-prescout-min-interval-sec",
        type=float,
        default=30.0,
        help="minimum repeated proactive F2D pre-scout interval per EV/TLS",
    )
    ap.add_argument(
        "--b1-lookahead-actuation-guard-enable",
        dest="b1_lookahead_actuation_guard_enable",
        action="store_true",
        default=True,
        help="limit full local TLS priority when the selected TLS is only a route-lookahead target",
    )
    ap.add_argument(
        "--no-b1-lookahead-actuation-guard-enable",
        dest="b1_lookahead_actuation_guard_enable",
        action="store_false",
        help="disable route-lookahead local actuation guard",
    )
    ap.add_argument(
        "--b1-lookahead-full-preemption-distance-m",
        type=float,
        default=70.0,
        help="route distance to selected inbound edge below which lookahead priority may use full local actuation",
    )
    ap.add_argument(
        "--b1-lookahead-warmup-max-extension-sec",
        type=float,
        default=4.0,
        help="maximum green extension allowed while the EV is upstream of a route-lookahead selected TLS",
    )
    ap.add_argument(
        "--b1-lookahead-upstream-stop-speed-mps",
        type=float,
        default=0.5,
        help="EV speed threshold for treating a route-lookahead target as blocked upstream",
    )
    ap.add_argument(
        "--b1-lookahead-upstream-stop-min-distance-m",
        type=float,
        default=30.0,
        help="minimum route distance to selected inbound edge for upstream-stopped lookahead suppression",
    )
    ap.add_argument(
        "--b1-lookahead-skip-upstream-stopped-enable",
        dest="b1_lookahead_skip_upstream_stopped_enable",
        action="store_true",
        default=True,
        help="skip active local actuation when the EV is stopped upstream of a lookahead selected TLS",
    )
    ap.add_argument(
        "--no-b1-lookahead-skip-upstream-stopped-enable",
        dest="b1_lookahead_skip_upstream_stopped_enable",
        action="store_false",
        help="allow warmup actuation even when the EV is stopped upstream of a lookahead selected TLS",
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
        "--f2-measured-override-min-robust-improvement",
        type=float,
        default=0.0,
        help=(
            "minimum robust-cost improvement required before a peer-refined measured F2 offer "
            "may override the B1 local anchor; 0 preserves legacy measured behavior"
        ),
    )
    ap.add_argument(
        "--f2-measured-override-min-ev-wait-improvement-sec",
        type=float,
        default=0.0,
        help="EV expected-wait improvement that can justify a measured F2 override even below the robust margin",
    )
    ap.add_argument(
        "--f2-measured-override-min-ev-miss-improvement-sec",
        type=float,
        default=0.0,
        help="EV expected-miss improvement that can justify a measured F2 override even below the robust margin",
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
        "--f2-refine-feedback-age-adaptive-enable",
        dest="f2_refine_feedback_age_adaptive_enable",
        action="store_true",
        default=True,
        help="adapt feedback freshness age by EV distance (stricter near, looser far)",
    )
    ap.add_argument(
        "--no-f2-refine-feedback-age-adaptive-enable",
        dest="f2_refine_feedback_age_adaptive_enable",
        action="store_false",
        help="disable adaptive feedback-age window",
    )
    ap.add_argument(
        "--f2-refine-feedback-max-age-near-sec",
        type=float,
        default=-1.0,
        help="feedback max age near stopline; <=0 uses adaptive default from base max-age",
    )
    ap.add_argument(
        "--f2-refine-feedback-max-age-far-sec",
        type=float,
        default=-1.0,
        help="feedback max age far from stopline; <=0 uses adaptive default from base max-age",
    )
    ap.add_argument(
        "--f2-refine-feedback-adaptive-far-distance-m",
        type=float,
        default=250.0,
        help="distance threshold (m) considered far for adaptive feedback-age interpolation",
    )
    ap.add_argument(
        "--f2-refine-feedback-bootstrap-enable",
        dest="f2_refine_feedback_bootstrap_enable",
        action="store_true",
        default=True,
        help="allow initial refine attempts before first downstream feedback exists",
    )
    ap.add_argument(
        "--no-f2-refine-feedback-bootstrap-enable",
        dest="f2_refine_feedback_bootstrap_enable",
        action="store_false",
        help="disable bootstrap refine when no feedback exists yet",
    )
    ap.add_argument(
        "--f2-refine-feedback-bootstrap-distance-m",
        type=float,
        default=450.0,
        help="max EV distance (m) to allow bootstrap refine before first feedback",
    )
    ap.add_argument(
        "--f2-refine-feedback-bootstrap-max-age-sec",
        type=float,
        default=20.0,
        help="max local EV-context age (s) for bootstrap refine before first feedback",
    )
    ap.add_argument(
        "--f2-refine-stale-feedback-gate-enable",
        dest="f2_refine_stale_feedback_gate_enable",
        action="store_true",
        default=True,
        help="gate refine when responder phase-state feedback is stale",
    )
    ap.add_argument(
        "--no-f2-refine-stale-feedback-gate-enable",
        dest="f2_refine_stale_feedback_gate_enable",
        action="store_false",
        help="disable stale feedback gate for federation refine",
    )
    ap.add_argument(
        "--f2-refine-max-responder-phase-state-age-ms",
        type=float,
        default=4000.0,
        help="max responder phase-state age (ms) allowed for federation refine in normal distance",
    )
    ap.add_argument(
        "--f2-refine-near-distance-m",
        type=float,
        default=40.0,
        help="EV distance threshold (m) for stricter refine freshness near stopline",
    )
    ap.add_argument(
        "--f2-refine-near-max-responder-phase-state-age-ms",
        type=float,
        default=1200.0,
        help="max responder phase-state age (ms) for refine when EV is near stopline",
    )
    ap.add_argument(
        "--f2-refine-require-preferred-feedback-when-near",
        dest="f2_refine_require_preferred_feedback_when_near",
        action="store_true",
        default=True,
        help="when EV is near, require feedback from the preferred next-hop TLS before refine",
    )
    ap.add_argument(
        "--no-f2-refine-require-preferred-feedback-when-near",
        dest="f2_refine_require_preferred_feedback_when_near",
        action="store_false",
        help="allow non-preferred responder feedback near stopline",
    )
    ap.add_argument(
        "--f2-refine-preferred-feedback-near-distance-m",
        type=float,
        default=60.0,
        help="distance threshold (m) to enforce preferred next-hop feedback gate",
    )
    ap.add_argument(
        "--f2-refine-neighbor-state-fallback-enable",
        dest="f2_refine_neighbor_state_fallback_enable",
        action="store_true",
        default=True,
        help="allow F2 refine fallback using recent neighboring TLS live state when reservation feedback is missing",
    )
    ap.add_argument(
        "--no-f2-refine-neighbor-state-fallback-enable",
        dest="f2_refine_neighbor_state_fallback_enable",
        action="store_false",
        help="disable neighboring TLS live-state fallback for F2 refine",
    )
    ap.add_argument(
        "--f2-refine-neighbor-state-max-age-sec",
        type=float,
        default=4.0,
        help="max age (s) of neighboring TLS state for F2 fallback in normal distance",
    )
    ap.add_argument(
        "--f2-refine-neighbor-state-near-max-age-sec",
        type=float,
        default=1.5,
        help="max age (s) of neighboring TLS state for F2 fallback when EV is near stopline",
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
        "--f2-skip-redundant-apply",
        dest="f2_skip_redundant_apply",
        action="store_true",
        default=True,
        help="skip repeated F2 fallback applies for unchanged plans within a short interval",
    )
    ap.add_argument(
        "--no-f2-skip-redundant-apply",
        dest="f2_skip_redundant_apply",
        action="store_false",
        help="allow repeated F2 fallback applies even when unchanged",
    )
    ap.add_argument(
        "--f2-skip-redundant-apply-min-interval-sec",
        type=float,
        default=0.8,
        help="minimum interval between identical noisy F2 fallback applies",
    )
    ap.add_argument(
        "--f2-skip-redundant-apply-min-interval-near-sec",
        type=float,
        default=0.8,
        help="near-stopline interval for redundant F2 apply suppression",
    )
    ap.add_argument(
        "--f2-skip-redundant-apply-min-interval-far-sec",
        type=float,
        default=0.8,
        help="far-distance interval for redundant F2 apply suppression",
    )
    ap.add_argument(
        "--f2-skip-redundant-apply-near-distance-m",
        type=float,
        default=120.0,
        help="distance threshold (m) considered near for redundant-apply interval interpolation",
    )
    ap.add_argument(
        "--f2-skip-redundant-apply-far-distance-m",
        type=float,
        default=300.0,
        help="distance threshold (m) considered far for redundant-apply interval interpolation",
    )
    ap.add_argument(
        "--f2-offer-preapply-dedupe-min-interval-sec",
        type=float,
        default=2.0,
        help="minimum interval between identical selected-offer pre-applies",
    )
    ap.add_argument(
        "--f2-offer-preapply-dedupe-min-interval-near-sec",
        type=float,
        default=2.0,
        help="near-stopline interval for selected-offer pre-apply dedupe",
    )
    ap.add_argument(
        "--f2-offer-preapply-dedupe-min-interval-far-sec",
        type=float,
        default=2.0,
        help="far-distance interval for selected-offer pre-apply dedupe",
    )
    ap.add_argument(
        "--f2-offer-preapply-dedupe-near-distance-m",
        type=float,
        default=120.0,
        help="distance threshold (m) considered near for offer pre-apply interval interpolation",
    )
    ap.add_argument(
        "--f2-offer-preapply-dedupe-far-distance-m",
        type=float,
        default=300.0,
        help="distance threshold (m) considered far for offer pre-apply interval interpolation",
    )
    ap.add_argument(
        "--f2-selected-offer-min-effective-extend-sec",
        type=float,
        default=0.5,
        help=(
            "minimum realized extension seconds required before a selected F2 extend offer "
            "may actuate over the preserved B1/local-anchor path"
        ),
    )
    ap.add_argument(
        "--f2-selected-offer-recompute-local-fallback",
        dest="f2_selected_offer_recompute_local_fallback",
        action="store_true",
        default=True,
        help=(
            "when a selected F2 offer has negligible actuation effect, recompute the local "
            "B1-equivalent plan at the same sim tick before giving up"
        ),
    )
    ap.add_argument(
        "--no-f2-selected-offer-recompute-local-fallback",
        dest="f2_selected_offer_recompute_local_fallback",
        action="store_false",
        help="disable same-tick local recomputation fallback for weak selected F2 offers",
    )
    ap.add_argument(
        "--f2-selected-none-slow-ev-guard-enable",
        dest="f2_selected_none_slow_ev_guard_enable",
        action="store_true",
        default=True,
        help=(
            "skip active selected-none F2 continuity when the EV is already slow and "
            "the target phase is green; this prevents no-offer fallback from pushing "
            "the EV into a blocked downstream approach"
        ),
    )
    ap.add_argument(
        "--no-f2-selected-none-slow-ev-guard-enable",
        dest="f2_selected_none_slow_ev_guard_enable",
        action="store_false",
        help="disable the slow-EV safety guard for selected-none F2 continuity",
    )
    ap.add_argument(
        "--f2-selected-none-slow-ev-guard-max-speed-mps",
        type=float,
        default=3.0,
        help="maximum EV speed considered slow/stalled for selected-none F2 continuity guard",
    )
    ap.add_argument(
        "--f2-selected-none-slow-ev-guard-min-distance-m",
        type=float,
        default=40.0,
        help=(
            "minimum distance to stopline for the selected-none slow-EV guard; avoids "
            "suppressing final-meter approach rescue/creep behavior"
        ),
    )
    ap.add_argument(
        "--f2-selected-none-slow-ev-guard-plan-types",
        default="saturation_reduction,intrusive",
        help="comma-separated plan types guarded by selected-none slow-EV protection",
    )
    ap.add_argument(
        "--f2-fallback-slow-ev-guard-enable",
        dest="f2_fallback_slow_ev_guard_enable",
        action="store_true",
        default=True,
        help=(
            "skip F2 fallback/local-anchor actuation when the EV is slow, still upstream, "
            "and the target phase is already green"
        ),
    )
    ap.add_argument(
        "--no-f2-fallback-slow-ev-guard-enable",
        dest="f2_fallback_slow_ev_guard_enable",
        action="store_false",
        help="disable the slow-EV guard for F2 fallback/local-anchor actuation",
    )
    ap.add_argument(
        "--f2-fallback-slow-ev-guard-max-speed-mps",
        type=float,
        default=3.0,
        help="maximum EV speed considered slow/stalled for F2 fallback/local-anchor guard",
    )
    ap.add_argument(
        "--f2-fallback-slow-ev-guard-min-distance-m",
        type=float,
        default=40.0,
        help="minimum distance to stopline for the F2 fallback/local-anchor slow-EV guard",
    )
    ap.add_argument(
        "--f2-fallback-slow-ev-guard-plan-types",
        default="saturation_reduction,intrusive",
        help="comma-separated plan types guarded by F2 fallback/local-anchor slow-EV protection",
    )
    ap.add_argument(
        "--f2-fallback-cadence-guard-enable",
        dest="f2_fallback_cadence_guard_enable",
        action="store_true",
        default=True,
        help="rate-limit repeated F2 fallback/local-anchor applies so fail-soft behaves like B1 cadence",
    )
    ap.add_argument(
        "--no-f2-fallback-cadence-guard-enable",
        dest="f2_fallback_cadence_guard_enable",
        action="store_false",
        help="disable F2 fallback/local-anchor cadence guard",
    )
    ap.add_argument(
        "--f2-fallback-cadence-min-interval-sec",
        type=float,
        default=1.0,
        help="minimum seconds between F2 fallback/local-anchor applies per EV/TLS/source",
    )
    ap.add_argument(
        "--f2-fallback-cadence-min-distance-m",
        type=float,
        default=40.0,
        help=(
            "minimum EV distance to stopline for F2 fallback/local-anchor cadence suppression; "
            "below this distance F2 keeps B1-like final approach continuity"
        ),
    )
    ap.add_argument(
        "--f2-fallback-cadence-plan-types",
        default="saturation_reduction,intrusive",
        help=(
            "comma-separated plan types rate-limited by F2 fallback/local-anchor cadence guard; "
            "non_intrusive is intentionally excluded by default to preserve the B1 floor"
        ),
    )
    ap.add_argument(
        "--f2-lookahead-upstream-stopped-rescue-enable",
        dest="f2_lookahead_upstream_stopped_rescue_enable",
        action="store_true",
        default=True,
        help=(
            "allow a bounded F2 warmup extension when the EV is stopped upstream of a near "
            "route-lookahead TLS; keeps the B1 guard conservative outside F2"
        ),
    )
    ap.add_argument(
        "--no-f2-lookahead-upstream-stopped-rescue-enable",
        dest="f2_lookahead_upstream_stopped_rescue_enable",
        action="store_false",
        help="disable F2 upstream-stopped route-lookahead rescue",
    )
    ap.add_argument(
        "--f2-lookahead-upstream-stopped-rescue-max-distance-m",
        type=float,
        default=120.0,
        help="maximum route distance to selected inbound edge for F2 upstream-stopped rescue",
    )
    ap.add_argument(
        "--f2-lookahead-upstream-stopped-rescue-max-hops",
        type=int,
        default=1,
        help="maximum route-lookahead hops eligible for F2 upstream-stopped rescue",
    )
    ap.add_argument(
        "--f2-lookahead-upstream-stopped-rescue-extension-sec",
        type=float,
        default=4.0,
        help="green-extension cap used by F2 upstream-stopped rescue",
    )
    ap.add_argument(
        "--f2-weak-offer-last-local-fallback-enable",
        dest="f2_weak_offer_last_local_fallback_enable",
        action="store_true",
        default=True,
        help=(
            "when a selected F2 offer has negligible effect and no current local plan exists, "
            "reuse a recent applicable B1-equivalent local-anchor plan for continuity"
        ),
    )
    ap.add_argument(
        "--no-f2-weak-offer-last-local-fallback-enable",
        dest="f2_weak_offer_last_local_fallback_enable",
        action="store_false",
        help="disable recent local-anchor fallback for weak selected F2 offers",
    )
    ap.add_argument(
        "--f2-weak-offer-last-local-fallback-max-age-sec",
        type=float,
        default=20.0,
        help="maximum age of cached local-anchor plan eligible for weak selected-offer fallback",
    )
    ap.add_argument(
        "--f2-strict-b1-floor-enable",
        dest="f2_strict_b1_floor_enable",
        action="store_true",
        default=False,
        help=(
            "force F2 no-offer/fail-soft behavior through the B1-continuity path instead "
            "of the separate selected-none F2 apply path; peer offers may still apply "
            "when they pass freshness/usefulness/downstream guards"
        ),
    )
    ap.add_argument(
        "--no-f2-strict-b1-floor-enable",
        dest="f2_strict_b1_floor_enable",
        action="store_false",
        help="allow the legacy F2 selected-none continuity path",
    )
    ap.add_argument(
        "--f2-strict-b1-floor-peer-override-only",
        dest="f2_strict_b1_floor_peer_override_only",
        action="store_true",
        default=True,
        help=(
            "under strict B1 floor, allow the selected-offer F2 apply path only for "
            "source-classified peer overrides; local-anchor/fallback selections are "
            "applied through the B1 floor path"
        ),
    )
    ap.add_argument(
        "--no-f2-strict-b1-floor-peer-override-only",
        dest="f2_strict_b1_floor_peer_override_only",
        action="store_false",
        help="allow legacy strict-floor selected-offer application without peer-source gating",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-enable",
        dest="f2_approach_phase_rescue_enable",
        action="store_true",
        default=True,
        help=(
            "allow a narrow F2 current-approach rescue when federation/local fallback keeps "
            "the EV target phase non-green close to the stop line"
        ),
    )
    ap.add_argument(
        "--no-f2-approach-phase-rescue-enable",
        dest="f2_approach_phase_rescue_enable",
        action="store_false",
        help="disable the F2 current-approach target-phase rescue",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-max-distance-m",
        type=float,
        default=120.0,
        help="maximum current-edge distance to stop line for F2 approach phase rescue",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-max-speed-mps",
        type=float,
        default=14.5,
        help="maximum EV speed for F2 approach phase rescue",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-min-interval-sec",
        type=float,
        default=4.0,
        help="minimum interval between F2 approach phase rescues for the same EV/TLS",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-blocked-only",
        dest="f2_approach_phase_rescue_blocked_only",
        action="store_true",
        default=True,
        help="only trigger F2 approach phase rescue when F2 selection/refine reports a blocked or infeasible local path",
    )
    ap.add_argument(
        "--no-f2-approach-phase-rescue-blocked-only",
        dest="f2_approach_phase_rescue_blocked_only",
        action="store_false",
        help="allow F2 approach phase rescue even without a blocked/infeasible final reason",
    )
    ap.add_argument(
        "--f2-approach-phase-rescue-require-current-edge",
        dest="f2_approach_phase_rescue_require_current_edge",
        action="store_true",
        default=True,
        help="only trigger F2 approach phase rescue when the EV is already on the selected TLS inbound edge",
    )
    ap.add_argument(
        "--no-f2-approach-phase-rescue-require-current-edge",
        dest="f2_approach_phase_rescue_require_current_edge",
        action="store_false",
        help="allow F2 approach phase rescue from route-lookahead edges",
    )
    ap.add_argument(
        "--f2-current-tls-stopped-rescue-enable",
        dest="f2_current_tls_stopped_rescue_enable",
        action="store_true",
        default=True,
        help=(
            "allow a narrow F2 rescue when the active TLS plan is blocked because the EV is "
            "already slow/stopped near that TLS"
        ),
    )
    ap.add_argument(
        "--no-f2-current-tls-stopped-rescue-enable",
        dest="f2_current_tls_stopped_rescue_enable",
        action="store_false",
        help="disable the F2 current-TLS stopped/near rescue",
    )
    ap.add_argument(
        "--f2-current-tls-stopped-rescue-max-distance-m",
        type=float,
        default=80.0,
        help="maximum stop-line distance for F2 current-TLS stopped/near rescue",
    )
    ap.add_argument(
        "--f2-current-tls-stopped-rescue-max-speed-mps",
        type=float,
        default=2.0,
        help="maximum EV speed for F2 current-TLS stopped/near rescue",
    )
    ap.add_argument(
        "--f2-current-tls-stopped-rescue-max-lookahead-hops",
        type=int,
        default=2,
        help="maximum active TLS lookahead hops eligible for F2 current-TLS stopped/near rescue",
    )
    ap.add_argument(
        "--f2-current-tls-stopped-rescue-min-interval-sec",
        type=float,
        default=6.0,
        help="minimum interval between F2 current-TLS stopped/near rescues for the same EV/TLS",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-enable",
        dest="f2_downstream_release_guard_enable",
        action="store_true",
        default=True,
        help=(
            "suppress F2 stopped-release rescues when the EV route ahead already shows "
            "spillback/low-speed blockage"
        ),
    )
    ap.add_argument(
        "--no-f2-downstream-release-guard-enable",
        dest="f2_downstream_release_guard_enable",
        action="store_false",
        help="disable the F2 downstream-spillback release guard",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-lookahead-edges",
        type=int,
        default=8,
        help="number of downstream EV route edges inspected before allowing F2 stopped-release rescue",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-min-halt-n",
        type=int,
        default=2,
        help="minimum halting vehicles on a downstream edge to suppress F2 stopped-release rescue",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-max-mean-speed-mps",
        type=float,
        default=2.0,
        help="mean-speed threshold for downstream low-speed suppression of F2 stopped-release rescue",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-min-veh-n",
        type=int,
        default=3,
        help="minimum vehicles on a downstream edge for low-speed F2 release suppression",
    )
    ap.add_argument(
        "--f2-downstream-release-guard-max-occupancy-pct",
        type=float,
        default=35.0,
        help="occupancy threshold for downstream F2 stopped-release suppression",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-enable",
        dest="f2_downstream_replay_guard_enable",
        action="store_true",
        default=True,
        help=(
            "suppress repeated F2 local-anchor/continuity replay when upcoming EV route "
            "edges already show spillback/low-speed blockage"
        ),
    )
    ap.add_argument(
        "--no-f2-downstream-replay-guard-enable",
        dest="f2_downstream_replay_guard_enable",
        action="store_false",
        help="disable the F2 downstream replay/keepalive spillback guard",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-lookahead-edges",
        type=int,
        default=3,
        help="number of downstream EV route edges inspected before F2 replay/keepalive apply",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-min-route-progress-frac",
        type=float,
        default=0.58,
        help=(
            "minimum EV route progress fraction before suppressing F2 replay/keepalive; "
            "keeps upstream coordination from being silenced by downstream queues"
        ),
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-max-route-progress-frac",
        type=float,
        default=0.70,
        help=(
            "maximum EV route progress fraction for F2 replay/keepalive suppression; "
            "prevents late destination-area queues from silencing upstream corridor service"
        ),
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-max-worst-edge-offset",
        type=int,
        default=2,
        help=(
            "maximum downstream route-edge offset allowed to suppress F2 replay/keepalive; "
            "keeps the guard focused on near-corridor blockage instead of distant queues"
        ),
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-offset1-min-halt-n",
        type=int,
        default=4,
        help=(
            "stricter halting-vehicle threshold for suppressing F2 replay when the worst "
            "blocked edge is the immediate next route edge"
        ),
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-offset1-max-mean-speed-mps",
        type=float,
        default=1.0,
        help=(
            "stricter mean-speed threshold for suppressing F2 replay when the worst "
            "blocked edge is the immediate next route edge"
        ),
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-min-halt-n",
        type=int,
        default=2,
        help="minimum halting vehicles on a downstream edge to suppress F2 replay/keepalive",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-max-mean-speed-mps",
        type=float,
        default=2.0,
        help="mean-speed threshold for downstream low-speed suppression of F2 replay/keepalive",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-min-veh-n",
        type=int,
        default=3,
        help="minimum vehicles on a downstream edge for low-speed F2 replay suppression",
    )
    ap.add_argument(
        "--f2-downstream-replay-guard-max-occupancy-pct",
        type=float,
        default=35.0,
        help="occupancy threshold for downstream F2 replay/keepalive suppression",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-enable",
        dest="f2_downstream_apply_guard_enable",
        action="store_true",
        default=True,
        help="suppress any aggressive F2 actuation when downstream EV route edges are already blocked",
    )
    ap.add_argument(
        "--no-f2-downstream-apply-guard-enable",
        dest="f2_downstream_apply_guard_enable",
        action="store_false",
        help="disable generic downstream blockage suppression for F2 applies",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-lookahead-edges",
        type=int,
        default=8,
        help="number of downstream EV route edges inspected before any aggressive F2 apply",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-max-worst-edge-offset",
        type=int,
        default=3,
        help=(
            "maximum downstream route-edge offset that can hard-suppress an F2 apply; "
            "farther blockage is treated as advisory so F2 does not collapse to B0 "
            "because of distant queues"
        ),
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-min-halt-n",
        type=int,
        default=2,
        help="minimum halting vehicles on a downstream edge to suppress aggressive F2 apply",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-max-mean-speed-mps",
        type=float,
        default=2.0,
        help="mean-speed threshold for downstream low-speed suppression of aggressive F2 apply",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-min-veh-n",
        type=int,
        default=3,
        help="minimum vehicles on a downstream edge for low-speed F2 apply suppression",
    )
    ap.add_argument(
        "--f2-downstream-apply-guard-max-occupancy-pct",
        type=float,
        default=35.0,
        help="occupancy threshold for downstream F2 apply suppression",
    )
    ap.add_argument(
        "--f2-active-coord-window-relax-enable",
        dest="f2_active_coord_window_relax_enable",
        action="store_true",
        default=False,
        help="relax apply dedupe intervals only during active federation coordination windows",
    )
    ap.add_argument(
        "--no-f2-active-coord-window-relax-enable",
        dest="f2_active_coord_window_relax_enable",
        action="store_false",
        help="disable active coordination window dedupe relaxation",
    )
    ap.add_argument(
        "--f2-active-coord-window-recent-sec",
        type=float,
        default=2.5,
        help="recentness window (sim sec) used to classify active coordination",
    )
    ap.add_argument(
        "--f2-active-coord-window-ev-near-m",
        type=float,
        default=180.0,
        help="EV distance threshold (m) for active coordination window relaxation",
    )
    ap.add_argument(
        "--f2-active-coord-window-min-active-reservations",
        type=int,
        default=1,
        help="minimum active reservations to consider coordination window active",
    )
    ap.add_argument(
        "--f2-active-coord-window-interval-scale",
        type=float,
        default=0.5,
        help="scale factor applied to dedupe intervals during active coordination windows",
    )
    ap.add_argument(
        "--f2-refine-local-cooldown-enable",
        dest="f2_refine_local_cooldown_enable",
        action="store_true",
        default=True,
        help="enable temporary local-only cooldown when near-stopline refine gates repeatedly block F2",
    )
    ap.add_argument(
        "--no-f2-refine-local-cooldown-enable",
        dest="f2_refine_local_cooldown_enable",
        action="store_false",
        help="disable temporary local-only cooldown for repeated near-stopline F2 refine blocks",
    )
    ap.add_argument(
        "--f2-refine-local-cooldown-trigger-count",
        type=int,
        default=3,
        help="number of consecutive near-stopline refine gate blocks before entering local cooldown",
    )
    ap.add_argument(
        "--f2-refine-local-cooldown-window-sec",
        type=float,
        default=2.5,
        help="duration (s) of temporary local-only cooldown once triggered",
    )
    ap.add_argument(
        "--f2-refine-local-cooldown-near-distance-m",
        type=float,
        default=-1.0,
        help="near-stopline distance (m) for cooldown logic; <=0 reuses --f2-refine-near-distance-m",
    )
    ap.add_argument(
        "--f2-usefulness-gate-enable",
        dest="f2_usefulness_gate_enable",
        action="store_true",
        default=True,
        help="enable F2 usefulness hold after repeated hard-request skip streaks",
    )
    ap.add_argument(
        "--no-f2-usefulness-gate-enable",
        dest="f2_usefulness_gate_enable",
        action="store_false",
        help="disable F2 usefulness hold for deterministic repeatability diagnostics",
    )
    ap.add_argument(
        "--f2-usefulness-gate-skip-streak-trigger",
        type=int,
        default=6,
        help="hard-request skip streak needed before entering F2 usefulness hold",
    )
    ap.add_argument(
        "--f2-usefulness-gate-hold-sec",
        type=float,
        default=3.0,
        help="sim-time duration of F2 usefulness hold once entered",
    )
    ap.add_argument(
        "--f2-usefulness-gate-near-only",
        dest="f2_usefulness_gate_near_only",
        action="store_true",
        default=True,
        help="only allow F2 usefulness hold while EV is near the stopline",
    )
    ap.add_argument(
        "--no-f2-usefulness-gate-near-only",
        dest="f2_usefulness_gate_near_only",
        action="store_false",
        help="allow F2 usefulness hold regardless of EV distance",
    )
    ap.add_argument(
        "--f2-usefulness-gate-near-distance-m",
        type=float,
        default=150.0,
        help="near-stopline distance threshold for F2 usefulness hold",
    )
    ap.add_argument(
        "--f2-usefulness-gate-require-no-hard-accept",
        dest="f2_usefulness_gate_require_no_hard_accept",
        action="store_true",
        default=True,
        help="enter usefulness hold only when recent hard reservations have no acceptances",
    )
    ap.add_argument(
        "--no-f2-usefulness-gate-require-no-hard-accept",
        dest="f2_usefulness_gate_require_no_hard_accept",
        action="store_false",
        help="allow usefulness hold even when some hard reservations were accepted",
    )
    ap.add_argument(
        "--f2-usefulness-gate-failsoft-local",
        dest="f2_usefulness_gate_failsoft_local",
        action="store_true",
        default=True,
        help="fallback to local offer while usefulness hold is active",
    )
    ap.add_argument(
        "--no-f2-usefulness-gate-failsoft-local",
        dest="f2_usefulness_gate_failsoft_local",
        action="store_false",
        help="do not force local fallback while usefulness hold is active",
    )
    ap.add_argument(
        "--fed-req-send-min-gap-sec",
        type=float,
        default=0.60,
        help="minimum sim-time gap between reservation req publishes to the same peer TLS",
    )
    ap.add_argument(
        "--fed-req-send-min-gap-near-sec",
        type=float,
        default=0.60,
        help="near-stopline min gap (sim sec) for reservation req publishes",
    )
    ap.add_argument(
        "--fed-req-send-min-gap-far-sec",
        type=float,
        default=0.60,
        help="far-distance min gap (sim sec) for reservation req publishes",
    )
    ap.add_argument(
        "--fed-req-send-min-gap-near-distance-m",
        type=float,
        default=120.0,
        help="distance threshold (m) considered near for req-gap interpolation",
    )
    ap.add_argument(
        "--fed-req-send-min-gap-far-distance-m",
        type=float,
        default=300.0,
        help="distance threshold (m) considered far for req-gap interpolation",
    )
    ap.add_argument(
        "--fed-req-pending-per-peer-cap",
        type=int,
        default=2,
        help="max pending req IDs per peer TLS before suppressing additional reservation req publishes",
    )
    ap.add_argument(
        "--fed-req-pending-stale-sec",
        type=float,
        default=6.0,
        help="stale-pending GC threshold (sim seconds) for unresolved reservation req IDs",
    )
    ap.add_argument(
        "--fed-min-hard-overlap-sec",
        type=float,
        default=0.50,
        help="minimum overlap (sec) for hard reservation feasibility at responder intersections",
    )
    ap.add_argument(
        "--fed-hard-overlap-grace-sec",
        type=float,
        default=0.80,
        help="grace tolerance (sec) to accept near-miss hard reservation windows",
    )
    ap.add_argument(
        "--fed-soft-window-grace-sec",
        type=float,
        default=6.0,
        help="soft reservation window look-ahead grace (sec) when no direct overlap is found",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-relax-enable",
        dest="fed_hard_window_adaptive_relax_enable",
        action="store_true",
        default=False,
        help="enable safety-gated adaptive rescue for hard near-miss reservation windows",
    )
    ap.add_argument(
        "--no-fed-hard-window-adaptive-relax-enable",
        dest="fed_hard_window_adaptive_relax_enable",
        action="store_false",
        help="disable adaptive rescue for hard near-miss reservation windows",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-extra-grace-sec",
        type=float,
        default=0.6,
        help="extra grace (sec) added to hard overlap grace under adaptive rescue",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-conf-min",
        type=float,
        default=0.65,
        help="minimum hard-request confidence required for adaptive hard-window rescue",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-readiness-min",
        type=float,
        default=0.55,
        help="minimum downstream readiness score required for adaptive hard-window rescue",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-spillback-max",
        type=float,
        default=0.80,
        help="maximum spillback risk allowed for adaptive hard-window rescue",
    )
    ap.add_argument(
        "--fed-hard-window-adaptive-queue-margin-min-sec",
        type=float,
        default=-1.5,
        help="minimum queue margin (sec) required for adaptive hard-window rescue",
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
        "--federation-bootstrap-cadence-mode",
        choices=["wall", "sim"],
        default="sim",
        help="clock source for bootstrap periodic tasks: wall=real time, sim=SUMO simulation time",
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
        "--federation-bootstrap-discovery-target-filter-enable",
        dest="federation_bootstrap_discovery_target_filter_enable",
        action="store_true",
        default=True,
        help="when enabled, EV target TLS selection can be filtered by recent discovery responses",
    )
    ap.add_argument(
        "--no-federation-bootstrap-discovery-target-filter-enable",
        dest="federation_bootstrap_discovery_target_filter_enable",
        action="store_false",
        help="disable discovery-validated target TLS filtering",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-target-max-age-sec",
        type=float,
        default=20.0,
        help="max age (seconds) for discovery response data used in target filtering",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-target-role",
        default="TrafficLightSystem",
        help="role expected from discovery results to treat a DT as a targetable intersection",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-filter-modes",
        default="F2",
        help="comma-separated evaluation modes where discovery target filtering is applied (e.g., F2)",
    )
    ap.add_argument(
        "--federation-peer-selection-source",
        choices=["realworld", "fnm"],
        default="fnm",
        help="who applies EV->intersection peer filtering: realworld dispatcher or per-FNM context manager",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-fail-open",
        dest="federation_bootstrap_discovery_fail_open",
        action="store_true",
        default=True,
        help="allow targeting when discovery/membership evidence is missing or stale",
    )
    ap.add_argument(
        "--no-federation-bootstrap-discovery-fail-open",
        dest="federation_bootstrap_discovery_fail_open",
        action="store_false",
        help="strict mode: reject targeting when discovery/membership evidence is missing or stale",
    )
    ap.add_argument(
        "--federation-bootstrap-discovery-require-membership-valid",
        dest="federation_bootstrap_discovery_require_membership_valid",
        action="store_true",
        default=True,
        help="require discovery targets to also have active/valid membership status",
    )
    ap.add_argument(
        "--no-federation-bootstrap-discovery-require-membership-valid",
        dest="federation_bootstrap_discovery_require_membership_valid",
        action="store_false",
        help="do not require membership validity when accepting discovery targets",
    )
    ap.add_argument(
        "--federation-bootstrap-active-member-statuses",
        default="ACTIVE,REGISTERED,ALIVE",
        help="comma-separated membership statuses treated as valid/active",
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


def _route_edge_length(edge_id: str) -> Optional[float]:
    """Best-effort SUMO edge length from the first non-internal lane."""
    edge = str(edge_id or "")
    if not edge or edge.startswith(":"):
        return None
    try:
        lane_count = int(traci.edge.getLaneNumber(edge))
    except Exception:
        lane_count = 1
    for lane_idx in range(max(1, lane_count)):
        lane_id = f"{edge}_{lane_idx}"
        try:
            length = float(traci.lane.getLength(lane_id))
            if math.isfinite(length) and length > 0.0:
                return length
        except Exception:
            continue
    return None


def route_distance_to_edge_stopline(ev_id: str, selected_in_edge: str) -> Optional[float]:
    """Distance from the EV's current position to the stopline of a future inbound edge.

    This is deterministic from TraCI route/lane state and avoids letting the most recent
    EV request payload decide the local B1/F2 timing when the target TLS is selected by
    route lookahead.
    """
    selected = str(selected_in_edge or "")
    if not selected:
        return None
    try:
        current_edge = str(traci.vehicle.getRoadID(ev_id) or "")
        route = [str(e) for e in list(traci.vehicle.getRoute(ev_id) or []) if str(e)]
        route_idx = int(traci.vehicle.getRouteIndex(ev_id))
        lane_id = str(traci.vehicle.getLaneID(ev_id) or "")
        lane_pos = float(traci.vehicle.getLanePosition(ev_id))
        lane_len = float(traci.lane.getLength(lane_id))
    except Exception:
        return None
    if not route or route_idx < 0:
        return None
    if current_edge == selected:
        return max(0.0, lane_len - lane_pos)

    start_idx = max(0, min(int(route_idx), len(route) - 1))
    selected_idx = -1
    for idx in range(start_idx, len(route)):
        if str(route[idx]) == selected:
            selected_idx = idx
            break
    if selected_idx < 0:
        return None

    distance = max(0.0, lane_len - lane_pos)
    for edge in route[start_idx + 1 : selected_idx + 1]:
        edge_len = _route_edge_length(edge)
        if edge_len is None:
            return None
        distance += float(edge_len)
    return max(0.0, float(distance))


def _edge_queue_snapshot(edge_id: str) -> Dict[str, Any]:
    """Best-effort aggregate queue snapshot for a non-internal SUMO edge."""
    edge = str(edge_id or "")
    out: Dict[str, Any] = {
        "edge": edge,
        "veh_n": 0,
        "halt_n": 0,
        "mean_speed_mps": -1.0,
        "occupancy_pct": 0.0,
        "lane_n": 0,
    }
    if not edge or edge.startswith(":"):
        return out
    try:
        lane_n = int(traci.edge.getLaneNumber(edge))
    except Exception:
        lane_n = 0
    total_veh = 0
    total_halt = 0
    total_occ = 0.0
    speed_weighted = 0.0
    speed_weight = 0
    valid_lanes = 0
    for lane_idx in range(max(0, lane_n)):
        lane_id = f"{edge}_{lane_idx}"
        try:
            veh_n = int(traci.lane.getLastStepVehicleNumber(lane_id))
            halt_n = int(traci.lane.getLastStepHaltingNumber(lane_id))
            occ = float(traci.lane.getLastStepOccupancy(lane_id))
            mean_speed = float(traci.lane.getLastStepMeanSpeed(lane_id))
        except Exception:
            continue
        valid_lanes += 1
        total_veh += max(0, int(veh_n))
        total_halt += max(0, int(halt_n))
        total_occ += max(0.0, float(occ))
        if veh_n > 0 and math.isfinite(mean_speed):
            speed_weighted += float(mean_speed) * float(veh_n)
            speed_weight += int(veh_n)
    out["lane_n"] = int(valid_lanes)
    out["veh_n"] = int(total_veh)
    out["halt_n"] = int(total_halt)
    out["occupancy_pct"] = float(total_occ / max(1, valid_lanes))
    out["mean_speed_mps"] = float(speed_weighted / speed_weight) if speed_weight > 0 else -1.0
    return out


def downstream_edges_assessment_diag(
    *,
    target_edges: Sequence[str],
    min_halt_n: int,
    max_mean_speed_mps: float,
    min_veh_n: int,
    max_occupancy_pct: float,
) -> Dict[str, Any]:
    """Inspect an explicit downstream edge list for queue/spillback risk.

    This is the server-side SUMO proxy used by mobile context providers such as
    a Drone-DT. It mirrors the compact F2 downstream diagnostic shape without
    requiring the requester to be colocated with TraCI.
    """
    edges = [str(e).strip() for e in list(target_edges or []) if str(e).strip() and not str(e).strip().startswith(":")]
    diag: Dict[str, Any] = {
        "enabled": True,
        "blocked": False,
        "reason": "clear",
        "lookahead_edges": list(edges),
        "lookahead_edges_n": int(len(edges)),
        "worst_edge": "",
        "worst_edge_offset": -1,
        "max_halt_n": 0,
        "max_veh_n": 0,
        "max_occupancy_pct": 0.0,
        "min_mean_speed_mps": -1.0,
        "edge_snapshots": [],
    }
    if not edges:
        diag["reason"] = "no_target_edges"
        return diag

    worst = None
    min_speed_seen: Optional[float] = None
    reasons: List[str] = []
    for offset, edge in enumerate(edges, start=1):
        snap = _edge_queue_snapshot(str(edge))
        veh_n = int(snap.get("veh_n", 0) or 0)
        halt_n = int(snap.get("halt_n", 0) or 0)
        occ = float(snap.get("occupancy_pct", 0.0) or 0.0)
        speed = float(snap.get("mean_speed_mps", -1.0) or -1.0)
        snap["offset"] = int(offset)
        diag["edge_snapshots"].append(dict(snap))
        diag["max_halt_n"] = max(int(diag["max_halt_n"]), int(halt_n))
        diag["max_veh_n"] = max(int(diag["max_veh_n"]), int(veh_n))
        diag["max_occupancy_pct"] = max(float(diag["max_occupancy_pct"]), float(occ))
        if speed >= 0.0:
            min_speed_seen = speed if min_speed_seen is None else min(float(min_speed_seen), float(speed))
        edge_reasons: List[str] = []
        if halt_n >= int(min_halt_n):
            edge_reasons.append("halting")
        if veh_n >= int(min_veh_n) and speed >= 0.0 and speed <= float(max_mean_speed_mps):
            edge_reasons.append("low_speed")
        if occ >= float(max_occupancy_pct):
            edge_reasons.append("occupancy")
        if edge_reasons:
            score = (len(edge_reasons), halt_n, occ, veh_n)
            if worst is None or score > worst[0]:
                worst = (score, str(edge), int(offset), ",".join(edge_reasons))
            reasons.extend(edge_reasons)

    diag["min_mean_speed_mps"] = -1.0 if min_speed_seen is None else float(min_speed_seen)
    if worst is not None:
        diag["blocked"] = True
        diag["worst_edge"] = str(worst[1])
        diag["worst_edge_offset"] = int(worst[2])
        diag["reason"] = str(worst[3])
    return diag


def b1_downstream_blockage_diag(
    *,
    ev_id: str,
    current_edge: str,
    selected_in_edge: str,
    lookahead_edges: int,
    min_halt_n: int,
    max_mean_speed_mps: float,
    min_veh_n: int,
    max_occupancy_pct: float,
    edge_to_tls_map: Optional[Dict[str, str]] = None,
    stop_at_non_tls: bool = False,
) -> Dict[str, Any]:
    """Inspect the next EV route edges for queue/spillback risk before local B1 priority.

    This is intentionally domain-light: it does not decide traffic strategy. It only
    prevents aggressive local priority from pushing the EV into already blocked route
    segments, while preserving non-intrusive B1 behavior.
    """
    diag: Dict[str, Any] = {
        "enabled": True,
        "blocked": False,
        "reason": "clear",
        "current_edge": str(current_edge or ""),
        "selected_in_edge": str(selected_in_edge or ""),
        "route_index": -1,
        "route_len": 0,
        "route_progress_frac": -1.0,
        "lookahead_edges_n": 0,
        "lookahead_edges": [],
        "worst_edge": "",
        "worst_edge_offset": -1,
        "max_halt_n": 0,
        "max_veh_n": 0,
        "max_occupancy_pct": 0.0,
        "min_mean_speed_mps": -1.0,
        "scan_scope": "tls_bounded" if bool(stop_at_non_tls) else "route_lookahead",
        "scan_limited_by_non_tls": False,
        "non_tls_boundary_edge": "",
    }
    try:
        route = [str(e) for e in list(traci.vehicle.getRoute(str(ev_id)) or []) if str(e) and not str(e).startswith(":")]
        route_idx = int(traci.vehicle.getRouteIndex(str(ev_id)))
    except Exception as e:
        diag["reason"] = f"route_unavailable:{type(e).__name__}"
        return diag
    if not route or route_idx < 0:
        diag["reason"] = "route_unavailable"
        return diag
    diag["route_index"] = int(route_idx)
    diag["route_len"] = int(len(route))
    current = str(current_edge or "")
    start_idx = max(0, min(int(route_idx), len(route) - 1))
    if current and str(route[start_idx]) != current:
        # Keep the TraCI route index authoritative; only repair it when the EV
        # edge appears later in the route. Using route.index(current) can jump
        # backwards on routes that traverse the same edge more than once.
        repaired_idx = -1
        for idx in range(start_idx, len(route)):
            if str(route[idx]) == current:
                repaired_idx = idx
                break
        if repaired_idx >= 0:
            start_idx = repaired_idx
    diag["route_progress_frac"] = (
        float(start_idx) / float(max(1, len(route) - 1))
        if len(route) > 1
        else 1.0
    )
    n = max(0, int(lookahead_edges))
    downstream = [str(e) for e in route[start_idx + 1 : start_idx + 1 + n] if str(e) and not str(e).startswith(":")]
    if bool(stop_at_non_tls):
        edge_to_tls_lookup = dict(edge_to_tls_map or {})
        bounded: List[str] = []
        boundary_edge = ""
        for edge in downstream:
            if not str(edge_to_tls_lookup.get(str(edge), "") or ""):
                boundary_edge = str(edge)
                break
            bounded.append(str(edge))
        if boundary_edge:
            diag["scan_limited_by_non_tls"] = True
            diag["non_tls_boundary_edge"] = str(boundary_edge)
            diag["requested_lookahead_edges"] = list(downstream)
            downstream = list(bounded)
    diag["lookahead_edges"] = list(downstream)
    diag["lookahead_edges_n"] = int(len(downstream))
    if not downstream:
        diag["reason"] = "non_tls_boundary" if bool(diag.get("scan_limited_by_non_tls", False)) else "no_downstream_edges"
        return diag

    worst = None
    min_speed_seen: Optional[float] = None
    reasons: List[str] = []
    for edge in downstream:
        snap = _edge_queue_snapshot(str(edge))
        veh_n = int(snap.get("veh_n", 0) or 0)
        halt_n = int(snap.get("halt_n", 0) or 0)
        occ = float(snap.get("occupancy_pct", 0.0) or 0.0)
        speed = float(snap.get("mean_speed_mps", -1.0) or -1.0)
        diag["max_halt_n"] = max(int(diag["max_halt_n"]), int(halt_n))
        diag["max_veh_n"] = max(int(diag["max_veh_n"]), int(veh_n))
        diag["max_occupancy_pct"] = max(float(diag["max_occupancy_pct"]), float(occ))
        if speed >= 0.0:
            min_speed_seen = speed if min_speed_seen is None else min(float(min_speed_seen), float(speed))
        edge_reasons: List[str] = []
        if halt_n >= int(min_halt_n):
            edge_reasons.append("halting")
        if veh_n >= int(min_veh_n) and speed >= 0.0 and speed <= float(max_mean_speed_mps):
            edge_reasons.append("low_speed")
        if occ >= float(max_occupancy_pct):
            edge_reasons.append("occupancy")
        if edge_reasons:
            score = (len(edge_reasons), halt_n, occ, veh_n)
            if worst is None or score > worst[0]:
                worst = (score, str(edge), snap, ",".join(edge_reasons))
            reasons.extend(edge_reasons)
    diag["min_mean_speed_mps"] = -1.0 if min_speed_seen is None else float(min_speed_seen)
    if worst is not None:
        diag["blocked"] = True
        diag["worst_edge"] = str(worst[1])
        try:
            diag["worst_edge_offset"] = int(list(downstream).index(str(worst[1])) + 1)
        except Exception:
            diag["worst_edge_offset"] = -1
        diag["reason"] = str(worst[3])
    return diag


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
        _peer_src_boot = str(
            getattr(args, "federation_peer_selection_source", "realworld") or "realworld"
        ).strip().lower()
        if _peer_src_boot not in {"realworld", "fnm"}:
            _peer_src_boot = "realworld"
        if float(getattr(args, "federation_bootstrap_catalog_sec", 30.0)) == 30.0:
            args.federation_bootstrap_catalog_sec = 20.0
        if (
            float(getattr(args, "federation_bootstrap_discovery_probe_sec", 0.0)) == 0.0
            and _peer_src_boot == "realworld"
        ):
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
            f"cadence={str(getattr(args, 'federation_bootstrap_cadence_mode', 'wall'))} "
            f"event_filter={str(args.federation_bootstrap_discovery_event_filter) or '*'} "
            f"peer_selection_source={_peer_src_boot}"
        )
    CURRENT_EVALUATION = str(getattr(args, "evaluation", CURRENT_EVALUATION) or CURRENT_EVALUATION).upper()
    if _is_passive_dt_mode(CURRENT_EVALUATION):
        args.passive_intersection_dt_enable = True
        if _is_f2p_queue_release_mode(CURRENT_EVALUATION):
            args.f2p_queue_release_enable = True
    else:
        args.f2p_queue_release_enable = False
    b1_strict_local_baseline = bool(
        CURRENT_EVALUATION == "B1"
        and getattr(args, "b1_strict_local_baseline_enable", False)
    )
    if b1_strict_local_baseline:
        # Keep B1 as the one-hop local baseline. Corridor/downstream context is
        # evaluated in F2/F2P/F2D families, not in the local baseline.
        args.b1_downstream_blockage_guard_enable = False
    if _is_drone_augmented_mode(CURRENT_EVALUATION):
        args.f2_drone_context_request_enable = True
        args.external_downstream_context_enable = True
        if _is_f2d_queue_release_mode(CURRENT_EVALUATION):
            args.f2d_queue_release_enable = True
    else:
        if bool(getattr(args, "f2_drone_context_request_enable", False)) or bool(
            getattr(args, "external_downstream_context_enable", False)
        ):
            print(
                "[EVAL][WARN] drone context flags ignored because evaluation="
                f"{CURRENT_EVALUATION}; use F2D, F2D-Q, or F2PD to enable drone-augmented F2"
            )
        args.f2_drone_context_request_enable = False
        args.external_downstream_context_enable = False
        args.f2d_queue_release_enable = False
        args.f2d_drone_prescout_enable = False
    print(
        f"[EVAL] startup evaluation={CURRENT_EVALUATION} "
        f"b1_strict_local={1 if b1_strict_local_baseline else 0}"
    )
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
            "topic_namespace": str(getattr(args, "mqtt_topic_namespace", "") or "").strip().strip("/"),
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

    # Route-window diagnostics are observability only; they must not influence policy or actuation.
    _service_window_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _late_rescue_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _apply_diag_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_noop_diag_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_local_anchor_preapply_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_last_local_anchor_plan_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_target_pending_bridge_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_approach_phase_rescue_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_fallback_cadence_state: Dict[Tuple[str, str, str], Dict[str, object]] = {}
    _f2_strict_b1_floor_preapply_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2p_passive_stall_rescue_state: Dict[Tuple[str, str], Dict[str, object]] = {}
    _f2_noop_diag_heartbeat_sec = 2.0

    def _flush_f2_noop_diag_summary(key: Tuple[str, str], sim_time: float, reason: str) -> None:
        prev = dict(_f2_noop_diag_state.get(key, {}) or {})
        suppressed = int(prev.get("suppressed_count", 0) or 0)
        if suppressed <= 0:
            return
        _fed_evt_main(
            "f2.apply.noop_suppressed",
            role="intersection",
            ev_id=str(key[0]),
            tls_id=str(key[1]),
            sim_time=float(sim_time),
            reason=str(reason),
            suppressed_count=int(suppressed),
            first_suppressed_time=prev.get("first_suppressed_time"),
            last_suppressed_time=prev.get("last_suppressed_time"),
            signature=list(prev.get("signature", []) or []),
        )
        prev["suppressed_count"] = 0
        _f2_noop_diag_state[key] = prev

    def _should_emit_f2_apply_diag(
        *,
        key: Tuple[str, str],
        sim_time: float,
        effective_apply: bool,
        signature: Tuple[object, ...],
    ) -> bool:
        if bool(effective_apply):
            _flush_f2_noop_diag_summary(key, float(sim_time), "effective_apply")
            _f2_noop_diag_state.pop(key, None)
            return True
        prev = dict(_f2_noop_diag_state.get(key, {}) or {})
        prev_sig = tuple(prev.get("signature", ()) or ())
        if prev and prev_sig == tuple(signature):
            last_emit = float(prev.get("last_emit_time", float(sim_time)))
            if (float(sim_time) - last_emit) < _f2_noop_diag_heartbeat_sec:
                prev["suppressed_count"] = int(prev.get("suppressed_count", 0) or 0) + 1
                prev.setdefault("first_suppressed_time", float(sim_time))
                prev["last_suppressed_time"] = float(sim_time)
                _f2_noop_diag_state[key] = prev
                return False
            _flush_f2_noop_diag_summary(key, float(sim_time), "heartbeat")
        elif prev:
            _flush_f2_noop_diag_summary(key, float(sim_time), "signature_changed")
        _f2_noop_diag_state[key] = {
            "signature": tuple(signature),
            "last_emit_time": float(sim_time),
            "suppressed_count": 0,
        }
        return True

    def _tls_diag_snapshot(tls_id: str, sim_time: float) -> Dict[str, object]:
        phase = -1
        program = ""
        state = ""
        next_switch = -1.0
        try:
            phase = int(traci.trafficlight.getPhase(str(tls_id)))
        except Exception:
            pass
        try:
            program = str(traci.trafficlight.getProgram(str(tls_id)))
        except Exception:
            pass
        try:
            state = str(traci.trafficlight.getRedYellowGreenState(str(tls_id)))
        except Exception:
            pass
        try:
            next_switch = float(traci.trafficlight.getNextSwitch(str(tls_id)))
        except Exception:
            pass
        return {
            "phase": int(phase),
            "program": str(program),
            "state": str(state),
            "next_switch": float(next_switch),
            "next_switch_rem_s": float(next_switch - float(sim_time)) if next_switch >= 0.0 else -1.0,
        }

    def _f2_redundant_apply_min_interval(distance_m: object) -> float:
        base = float(getattr(args, "f2_skip_redundant_apply_min_interval_sec", 0.8) or 0.8)
        near = float(getattr(args, "f2_skip_redundant_apply_min_interval_near_sec", base) or base)
        far = float(getattr(args, "f2_skip_redundant_apply_min_interval_far_sec", base) or base)
        near_m = float(getattr(args, "f2_skip_redundant_apply_near_distance_m", 120.0) or 120.0)
        far_m = float(getattr(args, "f2_skip_redundant_apply_far_distance_m", 300.0) or 300.0)
        try:
            dist = float(distance_m)
        except Exception:
            return float(base)
        if not math.isfinite(dist) or dist < 0.0:
            return float(base)
        if far_m <= near_m:
            return float(near if dist <= near_m else far)
        if dist <= near_m:
            return float(near)
        if dist >= far_m:
            return float(far)
        ratio = (dist - near_m) / max(1e-6, far_m - near_m)
        return float(near + ratio * (far - near))

    def _f2_plan_preapply_signature(plan: object, tls_before: Dict[str, object], lookahead_diag: Dict[str, object]) -> Tuple[object, ...]:
        plan_type = str(getattr(plan, "plan_type", "") or "")
        return (
            str(plan_type),
            int(getattr(plan, "target_phase_idx", -1) if getattr(plan, "target_phase_idx", None) is not None else -1),
            round(float(getattr(plan, "extend_green_sec", 0.0) or 0.0), 2),
            None
            if getattr(plan, "hurry_current_phase_to_sec", None) is None
            else round(float(getattr(plan, "hurry_current_phase_to_sec", 0.0) or 0.0), 2),
            None
            if getattr(plan, "jump_time_sec", None) is None
            else round(float(getattr(plan, "jump_time_sec", 0.0) or 0.0), 1),
            None if getattr(plan, "jump_to_phase_idx", None) is None else int(getattr(plan, "jump_to_phase_idx", -1)),
            int(tls_before.get("phase", -1)),
            str(tls_before.get("state", "")),
            str(lookahead_diag.get("action", "")),
            str(lookahead_diag.get("reason", "")),
            bool(lookahead_diag.get("preemption_eligible", True)),
        )

    def _f2_target_service_window_at_risk(
        *,
        plan: object,
        tls_before: Dict[str, object],
        sim_time: float,
        min_interval_s: float,
    ) -> Tuple[bool, Dict[str, object]]:
        """Detect when dedupe would let the current EV-serving green expire."""
        plan_type = str(getattr(plan, "plan_type", "") or "")
        if plan is None or plan_type in ("", "none", "restore"):
            return False, {"reason": "weak_plan", "plan_type": str(plan_type)}
        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            return False, {"reason": "missing_target_phase", "plan_type": str(plan_type)}

        target_green = _target_is_green_for_diag(tls_before, target_phase)
        try:
            next_switch = float(tls_before.get("next_switch", -1.0))
        except Exception:
            next_switch = -1.0
        next_switch_rem_s = float(next_switch - float(sim_time)) if next_switch >= 0.0 else -1.0
        expiry_guard_s = max(
            0.5,
            float(getattr(args, "f2_b1_continuity_expiry_guard_sec", 1.5) or 1.5),
        )
        dynamic_guard_s = max(float(expiry_guard_s), min(float(min_interval_s), float(expiry_guard_s) + 1.0))
        at_risk = bool(target_green and 0.0 <= float(next_switch_rem_s) <= float(dynamic_guard_s))
        return at_risk, {
            "reason": "target_green_expiring" if at_risk else "not_expiring",
            "plan_type": str(plan_type),
            "target_phase": int(target_phase),
            "target_green": bool(target_green),
            "next_switch": float(next_switch),
            "next_switch_rem_s": float(next_switch_rem_s),
            "expiry_guard_s": float(dynamic_guard_s),
        }

    def _should_skip_f2_local_anchor_preapply(
        *,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        plan: object,
        tls_before: Dict[str, object],
        lookahead_diag: Dict[str, object],
    ) -> Tuple[bool, Dict[str, object]]:
        if not bool(getattr(args, "f2_skip_redundant_apply", True)):
            return False, {"reason": "disabled"}
        plan_type = str(getattr(plan, "plan_type", "") or "")
        if plan_type == "intrusive":
            return False, {"reason": "intrusive_not_suppressed"}
        distance = lookahead_diag.get("route_distance_to_selected_edge_m")
        if distance is None:
            distance = lookahead_diag.get("distance_to_stopline_m")
        min_interval = _f2_redundant_apply_min_interval(distance)
        signature = _f2_plan_preapply_signature(plan, tls_before, lookahead_diag)
        key = (str(ev_id), str(tls_id))
        prev = dict(_f2_local_anchor_preapply_state.get(key, {}) or {})
        prev_signature = tuple(prev.get("signature", ()) or ())
        last_apply_time = float(prev.get("last_apply_time", -1e9))
        dt_since_last = float(sim_time) - float(last_apply_time)
        skip = bool(prev and prev_signature == tuple(signature) and dt_since_last >= 0.0 and dt_since_last < min_interval)
        diag = {
            "signature": list(signature),
            "min_interval_s": float(min_interval),
            "dt_since_last_s": float(dt_since_last) if prev else None,
            "distance_for_interval_m": distance,
            "previous_apply_time": last_apply_time if prev else None,
            "suppressed_count": int(prev.get("suppressed_count", 0) or 0),
        }
        if skip:
            at_risk, risk_diag = _f2_target_service_window_at_risk(
                plan=plan,
                tls_before=tls_before,
                sim_time=float(sim_time),
                min_interval_s=float(min_interval),
            )
            diag["service_window_risk"] = dict(risk_diag)
            if at_risk:
                _f2_local_anchor_preapply_state[key] = {
                    "signature": tuple(signature),
                    "last_apply_time": float(sim_time),
                    "last_suppressed_time": None,
                    "suppressed_count": 0,
                }
                diag["skip_bypassed"] = True
                diag["bypass_reason"] = str(risk_diag.get("reason", "service_window_at_risk"))
                return False, diag
            prev["suppressed_count"] = int(prev.get("suppressed_count", 0) or 0) + 1
            prev["last_suppressed_time"] = float(sim_time)
            _f2_local_anchor_preapply_state[key] = prev
            diag["suppressed_count"] = int(prev["suppressed_count"])
            return True, diag
        _f2_local_anchor_preapply_state[key] = {
            "signature": tuple(signature),
            "last_apply_time": float(sim_time),
            "last_suppressed_time": None,
            "suppressed_count": 0,
        }
        return False, diag

    def _should_skip_f2_strict_b1_floor_preapply(
        *,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        plan: object,
        tls_before: Dict[str, object],
        lookahead_diag: Dict[str, object],
        d_stop: Optional[float] = None,
    ) -> Tuple[bool, Dict[str, object]]:
        """Rate-limit strict B1-floor fallback so F2 does not out-actuate B1."""
        if not bool(getattr(args, "f2_skip_redundant_apply", True)):
            return False, {"reason": "disabled"}
        plan_type = str(getattr(plan, "plan_type", "") or "")
        if plan_type == "intrusive":
            return False, {"reason": "intrusive_not_suppressed", "plan_type": str(plan_type)}

        distance = lookahead_diag.get("route_distance_to_selected_edge_m")
        if distance is None:
            distance = lookahead_diag.get("distance_to_stopline_m")
        if distance is None:
            distance = d_stop
        min_interval = _f2_redundant_apply_min_interval(distance)
        signature = ("strict_b1_floor",) + tuple(_f2_plan_preapply_signature(plan, tls_before, lookahead_diag))
        key = (str(ev_id), str(tls_id))
        prev = dict(_f2_strict_b1_floor_preapply_state.get(key, {}) or {})
        prev_signature = tuple(prev.get("signature", ()) or ())
        last_apply_time = float(prev.get("last_apply_time", -1e9))
        dt_since_last = float(sim_time) - float(last_apply_time)
        skip = bool(
            prev
            and prev_signature == tuple(signature)
            and dt_since_last >= 0.0
            and dt_since_last < float(min_interval)
        )
        diag = {
            "signature": list(signature),
            "min_interval_s": float(min_interval),
            "dt_since_last_s": float(dt_since_last) if prev else None,
            "distance_for_interval_m": distance,
            "previous_apply_time": last_apply_time if prev else None,
            "plan_type": str(plan_type),
            "suppressed_count": int(prev.get("suppressed_count", 0) or 0),
        }
        if skip:
            at_risk, risk_diag = _f2_target_service_window_at_risk(
                plan=plan,
                tls_before=tls_before,
                sim_time=float(sim_time),
                min_interval_s=float(min_interval),
            )
            diag["service_window_risk"] = dict(risk_diag)
            if at_risk:
                _f2_strict_b1_floor_preapply_state[key] = {
                    "signature": tuple(signature),
                    "last_apply_time": float(sim_time),
                    "last_suppressed_time": None,
                    "suppressed_count": 0,
                    "plan_type": str(plan_type),
                }
                diag["reason"] = "service_window_at_risk"
                diag["skip_bypassed"] = True
                return False, diag
            prev["suppressed_count"] = int(prev.get("suppressed_count", 0) or 0) + 1
            prev["last_suppressed_time"] = float(sim_time)
            _f2_strict_b1_floor_preapply_state[key] = prev
            diag["reason"] = "cadence_guard"
            diag["suppressed_count"] = int(prev["suppressed_count"])
            return True, diag

        _f2_strict_b1_floor_preapply_state[key] = {
            "signature": tuple(signature),
            "last_apply_time": float(sim_time),
            "last_suppressed_time": None,
            "suppressed_count": 0,
            "plan_type": str(plan_type),
        }
        diag["reason"] = "allowed"
        return False, diag

    def _plan_diag_snapshot(plan: object) -> Dict[str, object]:
        if plan is None:
            return {
                "plan_type": "",
                "plan_target_phase": "",
                "plan_extend_green_sec": None,
                "plan_hurry_current_phase_to_sec": None,
                "plan_jump_time_sec": None,
                "plan_jump_to_phase_idx": None,
            }
        return {
            "plan_type": str(getattr(plan, "plan_type", "") or ""),
            "plan_target_phase": str(getattr(plan, "target_phase_idx", "") or ""),
            "plan_extend_green_sec": getattr(plan, "extend_green_sec", None),
            "plan_hurry_current_phase_to_sec": getattr(plan, "hurry_current_phase_to_sec", None),
            "plan_jump_time_sec": getattr(plan, "jump_time_sec", None),
            "plan_jump_to_phase_idx": getattr(plan, "jump_to_phase_idx", None),
        }

    def _offer_diag_snapshot(offer: object) -> Dict[str, object]:
        if offer is None:
            return {
                "offer_id": "",
                "offer_action": "",
                "offer_target_phase": "",
                "offer_feasible": None,
                "offer_wait_sec": None,
                "offer_miss_sec": None,
                "offer_confidence": None,
                "offer_action_params": {},
            }
        return {
            "offer_id": str(getattr(offer, "offer_id", "") or ""),
            "offer_action": str(getattr(offer, "action", "") or ""),
            "offer_target_phase": str(getattr(offer, "target_phase_idx", "") or ""),
            "offer_feasible": getattr(offer, "feasible", None),
            "offer_wait_sec": getattr(offer, "expected_wait_sec", None),
            "offer_miss_sec": getattr(offer, "expected_miss_sec", None),
            "offer_confidence": getattr(offer, "confidence", None),
            "offer_action_params": dict(getattr(offer, "action_params", {}) or {}),
        }

    def _selected_offer_effect_diag(ag: object, offer: object) -> Dict[str, object]:
        """Inspect the realized actuation size of a selected F2 offer before applying it."""
        offer_diag = _offer_diag_snapshot(offer)
        plan = None
        try:
            cache = getattr(ag, "_offer_plan_cache", {}) or {}
            plan = cache.get(str(offer_diag.get("offer_id", "") or ""))
        except Exception:
            plan = None
        if plan is None:
            try:
                plan = ag._fallback_plan_from_offer(offer)  # type: ignore[attr-defined]
            except Exception:
                plan = None
        plan_diag = _plan_diag_snapshot(plan)
        action = str(offer_diag.get("offer_action", "") or "")
        params = dict(offer_diag.get("offer_action_params", {}) or {})
        plan_extend = plan_diag.get("plan_extend_green_sec")
        try:
            effective_extend = float(plan_extend if plan_extend is not None else params.get("ext", 0.0) or 0.0)
        except Exception:
            effective_extend = 0.0
        try:
            threshold = float(getattr(getattr(ag, "cfg", None), "f2_selected_offer_min_effective_extend_sec", 0.5))
        except Exception:
            threshold = 0.5
        weak_effect = bool(action == "extend" and effective_extend < threshold)
        return {
            **offer_diag,
            **plan_diag,
            "effective_extend_sec": float(effective_extend),
            "min_effective_extend_sec": float(threshold),
            "weak_effect": bool(weak_effect),
        }

    def _active_ev_diag_snapshot(ag: object, ev_id: str, selected_in_edge: str, ev_edge: str, d_stop: float) -> Dict[str, object]:
        active_ev = getattr(ag, "active_ev", None)
        speed = None
        try:
            speed = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed = getattr(active_ev, "speed_mps", None)
        return {
            "ev_edge": str(ev_edge or ""),
            "selected_in_edge": str(selected_in_edge or ""),
            "distance_to_stopline_m": float(d_stop),
            "speed_mps": speed,
            "active_ev_in_edge": "" if active_ev is None else str(getattr(active_ev, "in_edge_id", "") or ""),
            "active_ev_distance_m": None if active_ev is None else getattr(active_ev, "distance_to_intersection_m", None),
            "active_ev_target_phase": None if active_ev is None else getattr(active_ev, "target_phase_idx", None),
            "active_ev_route_intersections": [] if active_ev is None else list(getattr(active_ev, "route_intersections", []) or []),
        }

    def _route_window_context(route_nodes: Sequence[str], tls_id: str) -> Dict[str, object]:
        route = [str(x) for x in list(route_nodes or []) if str(x)]
        idx = -1
        try:
            idx = route.index(str(tls_id))
        except Exception:
            idx = -1
        peer_ids = set()
        try:
            peer_ids = {str(k) for k in agents.keys()}
        except Exception:
            peer_ids = set()
        route_peer_tls = [node for node in route if (not peer_ids or node in peer_ids)]
        peer_idx = -1
        try:
            peer_idx = route_peer_tls.index(str(tls_id))
        except Exception:
            peer_idx = -1
        prev_node = route[idx - 1] if idx > 0 else ""
        current_node = route[idx] if idx >= 0 and idx < len(route) else str(tls_id)
        next_node = route[idx + 1] if idx >= 0 and idx + 1 < len(route) else ""
        next2_node = route[idx + 2] if idx >= 0 and idx + 2 < len(route) else ""
        prev_peer_tls = route_peer_tls[peer_idx - 1] if peer_idx > 0 else ""
        current_peer_tls = route_peer_tls[peer_idx] if peer_idx >= 0 and peer_idx < len(route_peer_tls) else str(tls_id)
        next_peer_tls = route_peer_tls[peer_idx + 1] if peer_idx >= 0 and peer_idx + 1 < len(route_peer_tls) else ""
        next2_peer_tls = route_peer_tls[peer_idx + 2] if peer_idx >= 0 and peer_idx + 2 < len(route_peer_tls) else ""
        return {
            "route_nodes": route,
            "route_index": int(idx),
            "route_prev_node": str(prev_node),
            "route_current_node": str(current_node),
            "route_next_node": str(next_node),
            "route_next2_node": str(next2_node),
            "route_peer_tls": route_peer_tls,
            "route_peer_index": int(peer_idx),
            "route_prev_peer_tls": str(prev_peer_tls),
            "route_current_peer_tls": str(current_peer_tls),
            "route_next_peer_tls": str(next_peer_tls),
            "route_next2_peer_tls": str(next2_peer_tls),
            # Backward-compatible names now refer to peer TLS, not arbitrary route nodes.
            "route_prev_tls": str(prev_peer_tls),
            "route_current_tls": str(current_peer_tls),
            "route_next_tls": str(next_peer_tls),
            "route_next2_tls": str(next2_peer_tls),
        }

    def _target_phase_for_diag(ag: object, selected_in_edge: str) -> int:
        active_ev = getattr(ag, "active_ev", None)
        try:
            if active_ev is not None and getattr(active_ev, "target_phase_idx", None) is not None:
                return int(getattr(active_ev, "target_phase_idx"))
        except Exception:
            pass
        try:
            return int(getattr(ag, "_inbound_edge_to_phase", {}).get(str(selected_in_edge), -1))
        except Exception:
            return -1

    def _target_window_for_diag(ag: object, sim_time: float, target_phase: int) -> Tuple[Optional[float], Optional[float]]:
        if int(target_phase) < 0:
            return None, None
        try:
            win = ag._predict_next_phase_window(float(sim_time), int(target_phase))
            if win is not None:
                return float(win[0]), float(win[1])
        except Exception:
            pass
        return None, None

    def _target_is_green_for_diag(tls_snap: Dict[str, object], target_phase: int) -> bool:
        try:
            return int(tls_snap.get("phase", -1)) == int(target_phase) and ("G" in str(tls_snap.get("state", "")))
        except Exception:
            return False

    def _f2_target_pending_route_distance(lookahead_diag: Optional[Dict[str, object]], d_stop: float) -> float:
        diag = dict(lookahead_diag or {})
        value = diag.get("route_distance_to_selected_edge_m")
        if value is None:
            value = diag.get("distance_to_stopline_m")
        if value is None:
            value = d_stop
        try:
            return float(value)
        except Exception:
            try:
                return float(d_stop)
            except Exception:
                return -1.0

    def _f2_target_pending_bridge_signature(
        *,
        stage: str,
        plan: object,
        target_phase: int,
        selected_in_edge: str,
        lookahead_diag: Optional[Dict[str, object]],
    ) -> Tuple[object, ...]:
        diag = dict(lookahead_diag or {})
        return (
            str(stage),
            str(getattr(plan, "plan_type", "") or ""),
            int(target_phase),
            str(selected_in_edge or ""),
            bool(diag.get("route_lookahead", False)),
            int(diag.get("lookahead_hops", 0) or 0),
        )

    def _f2_target_pending_bridge_limits() -> Tuple[float, int, int, float]:
        return (
            max(1.0, float(getattr(args, "f2_target_pending_bridge_window_sec", 8.0) or 8.0)),
            max(1, int(getattr(args, "f2_target_pending_bridge_rescue_trigger_n", 2) or 2)),
            max(1, int(getattr(args, "f2_target_pending_bridge_suppress_after_n", 3) or 3)),
            max(0.0, float(getattr(args, "f2_target_pending_bridge_max_distance_m", 160.0) or 160.0)),
        )

    def _record_f2_target_pending_bridge(
        *,
        stage: str,
        decision_source: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        selected_in_edge: str,
        ev_edge: str,
        d_stop: float,
        plan: object,
        before: Dict[str, object],
        after: Dict[str, object],
        lookahead_diag: Optional[Dict[str, object]] = None,
        reason: str = "",
        source: str = "",
    ) -> Dict[str, object]:
        if not bool(getattr(args, "f2_target_pending_bridge_guard_enable", True)):
            return {"enabled": False, "count": 0}
        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        key = (str(ev_id), str(tls_id))
        if int(target_phase) < 0:
            _f2_target_pending_bridge_state.pop(key, None)
            return {"enabled": True, "reason": "missing_target_phase", "count": 0}
        target_green_after = _target_is_green_for_diag(after, int(target_phase))
        if target_green_after:
            prev = dict(_f2_target_pending_bridge_state.pop(key, {}) or {})
            return {
                "enabled": True,
                "reason": "target_green",
                "count": int(prev.get("count", 0) or 0),
                "cleared": bool(prev),
            }
        window_s, rescue_trigger_n, suppress_after_n, max_dist_m = _f2_target_pending_bridge_limits()
        route_dist = _f2_target_pending_route_distance(lookahead_diag, float(d_stop))
        signature = _f2_target_pending_bridge_signature(
            stage=str(stage),
            plan=plan,
            target_phase=int(target_phase),
            selected_in_edge=str(selected_in_edge),
            lookahead_diag=lookahead_diag,
        )
        prev = dict(_f2_target_pending_bridge_state.get(key, {}) or {})
        prev_sig = tuple(prev.get("signature", ()) or ())
        last_time = float(prev.get("last_time", -1e9))
        same_series = bool(prev and prev_sig == tuple(signature) and 0.0 <= float(sim_time) - last_time <= float(window_s))
        count = int(prev.get("count", 0) or 0) + 1 if same_series else 1
        first_time = float(prev.get("first_time", sim_time)) if same_series else float(sim_time)
        _f2_target_pending_bridge_state[key] = {
            "signature": tuple(signature),
            "count": int(count),
            "first_time": float(first_time),
            "last_time": float(sim_time),
            "last_route_distance_m": float(route_dist),
            "target_phase": int(target_phase),
            "stage": str(stage),
            "decision_source": str(decision_source),
        }
        rescue_started = False
        rescue_eligible = bool(
            int(count) >= int(rescue_trigger_n)
            and route_dist >= 0.0
            and route_dist <= float(max_dist_m)
        )
        if rescue_eligible and key not in _late_rescue_state:
            try:
                speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
            except Exception:
                speed_f = -1.0
            _late_rescue_state[key] = {
                "start_time": float(sim_time),
                "start_distance_m": float(route_dist),
                "start_speed_mps": float(speed_f),
                "start_phase": int(before.get("phase", -1)),
                "target_phase": int(target_phase),
                "started_by": f"target_pending_bridge:{str(stage)}:{str(reason or source or 'target_not_green')}",
            }
            rescue_started = True
            _fed_evt_main(
                "f2.late_rescue.start",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                stage=str(stage),
                decision_source=str(decision_source),
                final_reason=str(reason),
                plan_type=str(getattr(plan, "plan_type", "") or ""),
                offer_action="",
                phase=int(before.get("phase", -1)),
                target_phase=int(target_phase),
                target_green=False,
                speed_mps=float(speed_f),
                distance_to_stopline_m=float(route_dist),
                bridge_pending_count=int(count),
                bridge_pending_window_s=float(window_s),
                bridge_pending_max_distance_m=float(max_dist_m),
            )
        if int(count) >= int(rescue_trigger_n):
            _fed_evt_main(
                "f2.target_pending_bridge.detected",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                stage=str(stage),
                decision_source=str(decision_source),
                reason=str(reason),
                source=str(source),
                plan_type=str(getattr(plan, "plan_type", "") or ""),
                target_phase=int(target_phase),
                target_green_after=False,
                before_phase=int(before.get("phase", -1)),
                after_phase=int(after.get("phase", -1)),
                route_distance_to_selected_edge_m=float(route_dist),
                count=int(count),
                first_time=float(first_time),
                elapsed_s=float(sim_time) - float(first_time),
                rescue_started=bool(rescue_started),
                suppress_after_n=int(suppress_after_n),
            )
        return {
            "enabled": True,
            "count": int(count),
            "target_phase": int(target_phase),
            "route_distance_to_selected_edge_m": float(route_dist),
            "rescue_started": bool(rescue_started),
            "rescue_eligible": bool(rescue_eligible),
            "suppress_after_n": int(suppress_after_n),
            "signature": list(signature),
        }

    def _should_skip_f2_target_pending_bridge(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        selected_in_edge: str,
        d_stop: float,
        plan: object,
        tls_before: Dict[str, object],
        lookahead_diag: Optional[Dict[str, object]] = None,
    ) -> Tuple[bool, Dict[str, object]]:
        if not bool(getattr(args, "f2_target_pending_bridge_guard_enable", True)):
            return False, {"enabled": False, "reason": "disabled"}
        plan_type = str(getattr(plan, "plan_type", "") or "")
        if plan is None or plan_type in ("", "none", "restore", "intrusive"):
            return False, {"enabled": True, "reason": "plan_not_suppressed", "plan_type": str(plan_type)}
        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        if target_phase < 0 or _target_is_green_for_diag(tls_before, int(target_phase)):
            return False, {"enabled": True, "reason": "target_green_or_unknown", "target_phase": int(target_phase)}
        window_s, _rescue_trigger_n, suppress_after_n, max_dist_m = _f2_target_pending_bridge_limits()
        route_dist = _f2_target_pending_route_distance(lookahead_diag, float(d_stop))
        signature = _f2_target_pending_bridge_signature(
            stage=str(stage),
            plan=plan,
            target_phase=int(target_phase),
            selected_in_edge=str(selected_in_edge),
            lookahead_diag=lookahead_diag,
        )
        key = (str(ev_id), str(tls_id))
        prev = dict(_f2_target_pending_bridge_state.get(key, {}) or {})
        count = int(prev.get("count", 0) or 0)
        last_time = float(prev.get("last_time", -1e9))
        same_series = bool(
            prev
            and tuple(prev.get("signature", ()) or ()) == tuple(signature)
            and 0.0 <= float(sim_time) - last_time <= float(window_s)
        )
        skip = bool(
            same_series
            and count >= int(suppress_after_n)
            and route_dist >= 0.0
            and route_dist <= float(max_dist_m)
            and key in _late_rescue_state
        )
        if skip:
            prev["last_suppressed_time"] = float(sim_time)
            prev["suppressed_count"] = int(prev.get("suppressed_count", 0) or 0) + 1
            _f2_target_pending_bridge_state[key] = prev
        return skip, {
            "enabled": True,
            "reason": "target_pending_bridge_suppressed" if skip else "not_suppressed",
            "plan_type": str(plan_type),
            "target_phase": int(target_phase),
            "target_green_before": False,
            "count": int(count),
            "suppress_after_n": int(suppress_after_n),
            "window_s": float(window_s),
            "route_distance_to_selected_edge_m": float(route_dist),
            "max_distance_m": float(max_dist_m),
            "late_rescue_active": bool(key in _late_rescue_state),
            "suppressed_count": int(prev.get("suppressed_count", 0) or 0),
        }

    external_downstream_context_cache: Dict[Tuple[str, str], Dict[str, object]] = {}
    external_downstream_context_missing_trace_recent: Dict[Tuple[str, str], float] = {}
    f2d_recovery_trace_seen: set = set()
    f2d_directed_context_seen: set = set()
    f2d_queue_release_recent: Dict[Tuple[str, str, str], float] = {}
    f2p_queue_release_recent: Dict[Tuple[str, str, str], float] = {}

    def _f2d_mobile_passive_context_enabled() -> bool:
        return bool(
            _is_drone_augmented_mode(CURRENT_EVALUATION)
            and bool(getattr(args, "external_downstream_context_enable", False))
            and bool(getattr(args, "f2d_mobile_passive_context_enable", True))
        )

    def _f2d_directed_context_delivery_enabled() -> bool:
        return bool(
            _f2d_mobile_passive_context_enabled()
            and bool(getattr(args, "f2d_directed_context_delivery_enable", True))
        )

    def _f2d_contextual_topic_delivery_enabled() -> bool:
        return bool(
            _f2d_mobile_passive_context_enabled()
            and bool(getattr(args, "f2d_contextual_topic_delivery_enable", False))
        )

    def _f2d_queue_release_enabled() -> bool:
        return bool(
            _is_drone_augmented_mode(CURRENT_EVALUATION)
            and bool(getattr(args, "external_downstream_context_enable", False))
            and (
                _is_f2d_queue_release_mode(CURRENT_EVALUATION)
                or bool(getattr(args, "f2d_queue_release_enable", False))
            )
        )

    def _f2p_queue_release_enabled() -> bool:
        return bool(
            _is_passive_dt_mode(CURRENT_EVALUATION)
            and bool(getattr(args, "passive_intersection_dt_enable", False))
            and (
                _is_f2p_queue_release_mode(CURRENT_EVALUATION)
                or bool(getattr(args, "f2p_queue_release_enable", False))
            )
        )

    def _lane_edge_id(lane_id: object) -> str:
        lane_s = str(lane_id or "").strip()
        if not lane_s:
            return ""
        try:
            return str(traci.lane.getEdgeID(lane_s))
        except Exception:
            # SUMO lane ids are usually <edge>_<index>; keep this as a fallback
            # for offline/mock contexts where traci.lane may not be available.
            return lane_s.rsplit("_", 1)[0]

    def _best_tls_phase_for_inbound_edge(tls_id: str, inbound_edge: str) -> Tuple[Optional[int], Dict[str, object]]:
        tls_s = str(tls_id or "").strip()
        edge_s = str(inbound_edge or "").strip()
        if not tls_s or not edge_s:
            return None, {"reason": "missing_tls_or_edge"}
        try:
            controlled_links = list(traci.trafficlight.getControlledLinks(tls_s) or [])
        except Exception as exc:
            return None, {"reason": "controlled_links_error", "error": f"{type(exc).__name__}:{exc}"}

        link_indices: List[int] = []
        for link_idx, links in enumerate(controlled_links):
            for link in list(links or []):
                try:
                    from_lane = str(link[0])
                except Exception:
                    continue
                if _lane_edge_id(from_lane) == edge_s:
                    link_indices.append(int(link_idx))
                    break
        if not link_indices:
            return None, {
                "reason": "edge_not_controlled_by_tls",
                "controlled_links_n": int(len(controlled_links)),
            }

        try:
            current_phase = int(traci.trafficlight.getPhase(tls_s))
        except Exception:
            current_phase = -1
        try:
            logics = list(traci.trafficlight.getAllProgramLogics(tls_s) or [])
        except Exception:
            try:
                logics = list(traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_s) or [])
            except Exception as exc:
                return None, {"reason": "program_logic_error", "error": f"{type(exc).__name__}:{exc}"}
        if not logics:
            return None, {"reason": "no_program_logic"}
        phases = list(getattr(logics[0], "phases", []) or [])
        if not phases:
            return None, {"reason": "no_program_phases"}

        best_phase: Optional[int] = None
        best_score = -1
        best_strong_score = -1
        for phase_idx, phase in enumerate(phases):
            state = str(getattr(phase, "state", "") or "")
            green_score = 0
            strong_green_score = 0
            for link_idx in link_indices:
                if 0 <= int(link_idx) < len(state):
                    sig = state[int(link_idx)]
                    if sig in {"g", "G"}:
                        green_score += 1
                    if sig == "G":
                        strong_green_score += 1
            if green_score <= 0:
                continue
            tie_break_current = bool(int(phase_idx) == int(current_phase))
            if (
                green_score > best_score
                or (green_score == best_score and strong_green_score > best_strong_score)
                or (
                    green_score == best_score
                    and strong_green_score == best_strong_score
                    and tie_break_current
                )
            ):
                best_phase = int(phase_idx)
                best_score = int(green_score)
                best_strong_score = int(strong_green_score)
        if best_phase is None:
            return None, {
                "reason": "no_green_phase_for_edge",
                "link_indices": list(link_indices),
                "current_phase": int(current_phase),
            }
        return best_phase, {
            "reason": "ok",
            "link_indices": list(link_indices),
            "current_phase": int(current_phase),
            "green_link_score": int(best_score),
            "strong_green_link_score": int(best_strong_score),
            "phases_n": int(len(phases)),
        }

    def _try_apply_f2d_queue_release(
        *,
        rec: Dict[str, object],
        requester_tls: str,
        ev_id: str,
        sim_time: float,
        worst_edge_tls: str,
        worst_edge: str,
        recovery_action: str,
        context_age_sec: float,
    ) -> Tuple[bool, Dict[str, object]]:
        tls_s = str(worst_edge_tls or "").strip()
        edge_s = str(worst_edge or "").strip()
        ev_s = str(ev_id or rec.get("ev_id", "") or "").strip()
        if not _f2d_queue_release_enabled():
            return False, {"queue_release_application": "disabled"}
        if not tls_s or not edge_s:
            return False, {"queue_release_application": "missing_tls_or_edge"}
        if tls_s not in agents:
            return False, {"queue_release_application": "target_tls_not_active_agent"}
        try:
            worst_offset = int(rec.get("worst_edge_offset", -1) or -1)
        except Exception:
            worst_offset = -1
        max_offset = max(1, int(getattr(args, "f2d_queue_release_max_worst_edge_offset", 8) or 8))
        if worst_offset < 1 or worst_offset > max_offset:
            return False, {
                "queue_release_application": "worst_edge_offset_out_of_range",
                "worst_edge_offset": int(worst_offset),
                "max_worst_edge_offset": int(max_offset),
            }
        min_interval = max(0.0, float(getattr(args, "f2d_queue_release_min_interval_sec", 3.0) or 3.0))
        recent_key = (str(tls_s), str(edge_s), str(ev_s))
        last_t = float(f2d_queue_release_recent.get(recent_key, -1e9))
        if float(sim_time) - last_t < min_interval:
            return False, {
                "queue_release_application": "rate_limited",
                "last_apply_sim_time": float(last_t),
                "min_interval_sec": float(min_interval),
            }
        phase, phase_diag = _best_tls_phase_for_inbound_edge(tls_s, edge_s)
        if phase is None:
            out = {"queue_release_application": "phase_unavailable"}
            out.update(dict(phase_diag or {}))
            return False, out
        hold_sec = max(0.1, float(getattr(args, "f2d_queue_release_hold_sec", 3.0) or 3.0))
        try:
            prev_phase = int(traci.trafficlight.getPhase(tls_s))
        except Exception:
            prev_phase = -1
        try:
            if prev_phase != int(phase):
                traci.trafficlight.setPhase(tls_s, int(phase))
            traci.trafficlight.setPhaseDuration(tls_s, float(hold_sec))
            f2d_queue_release_recent[recent_key] = float(sim_time)
        except Exception as exc:
            return False, {
                "queue_release_application": "traci_apply_error",
                "error": f"{type(exc).__name__}:{exc}",
                **dict(phase_diag or {}),
            }
        apply_diag = {
            "queue_release_application": "traci_set_phase_duration",
            "queue_release_phase": int(phase),
            "queue_release_prev_phase": int(prev_phase),
            "queue_release_hold_sec": float(hold_sec),
            "queue_release_target_tls": str(tls_s),
            "queue_release_target_edge": str(edge_s),
            **dict(phase_diag or {}),
        }
        _fed_evt_main(
            "f2d.queue_release.applied",
            **_drone_context_trace_payload(
                rec=rec,
                tls_id=str(requester_tls),
                ev_id=str(ev_s),
                sim_time=float(sim_time),
                selected_action=str(recovery_action),
                decision_source="f2d_queue_release_actuator",
                reason="drone_confirmed_blockage_queue_release_applied",
                context_age_sec=float(context_age_sec),
                extra=dict(apply_diag),
            ),
        )
        return True, apply_diag

    def _try_apply_f2p_queue_release(
        *,
        payload: Dict[str, object],
        source_node: str,
        ev_id: str,
        sim_time: float,
    ) -> Tuple[bool, Dict[str, object]]:
        source_s = str(source_node or payload.get("node_id", "") or "").strip()
        edge_s = str(payload.get("worst_edge", "") or "").strip()
        ev_s = str(ev_id or payload.get("ev_id", "") or "").strip()
        if not _f2p_queue_release_enabled():
            return False, {"queue_release_application": "disabled"}
        if not bool(payload.get("blocked", False)):
            return False, {"queue_release_application": "not_blocked"}
        if not edge_s:
            return False, {"queue_release_application": "missing_worst_edge"}
        try:
            tls_s = str(edge_to_tls.get(edge_s, "") or "")
        except Exception:
            tls_s = ""
        if not tls_s:
            return False, {"queue_release_application": "worst_edge_has_no_tls"}
        if tls_s not in agents:
            return False, {"queue_release_application": "target_tls_not_active_agent", "worst_edge_tls": str(tls_s)}
        try:
            worst_offset = int(payload.get("worst_edge_offset", -1) or -1)
        except Exception:
            worst_offset = -1
        max_offset = max(1, int(getattr(args, "f2p_queue_release_max_worst_edge_offset", 4) or 4))
        if worst_offset < 1 or worst_offset > max_offset:
            return False, {
                "queue_release_application": "worst_edge_offset_out_of_range",
                "worst_edge_tls": str(tls_s),
                "worst_edge_offset": int(worst_offset),
                "max_worst_edge_offset": int(max_offset),
            }
        min_interval = max(0.0, float(getattr(args, "f2p_queue_release_min_interval_sec", 3.0) or 3.0))
        recent_key = (str(source_s), str(tls_s), str(edge_s))
        last_t = float(f2p_queue_release_recent.get(recent_key, -1e9))
        if float(sim_time) - last_t < min_interval:
            return False, {
                "queue_release_application": "rate_limited",
                "worst_edge_tls": str(tls_s),
                "last_apply_sim_time": float(last_t),
                "min_interval_sec": float(min_interval),
            }
        phase, phase_diag = _best_tls_phase_for_inbound_edge(tls_s, edge_s)
        if phase is None:
            out = {"queue_release_application": "phase_unavailable", "worst_edge_tls": str(tls_s)}
            out.update(dict(phase_diag or {}))
            return False, out
        hold_sec = max(0.1, float(getattr(args, "f2p_queue_release_hold_sec", 3.0) or 3.0))
        try:
            prev_phase = int(traci.trafficlight.getPhase(tls_s))
        except Exception:
            prev_phase = -1
        try:
            if prev_phase != int(phase):
                traci.trafficlight.setPhase(tls_s, int(phase))
            traci.trafficlight.setPhaseDuration(tls_s, float(hold_sec))
            f2p_queue_release_recent[recent_key] = float(sim_time)
        except Exception as exc:
            return False, {
                "queue_release_application": "traci_apply_error",
                "worst_edge_tls": str(tls_s),
                "error": f"{type(exc).__name__}:{exc}",
                **dict(phase_diag or {}),
            }
        apply_diag = {
            "queue_release_application": "traci_set_phase_duration",
            "queue_release_phase": int(phase),
            "queue_release_prev_phase": int(prev_phase),
            "queue_release_hold_sec": float(hold_sec),
            "queue_release_target_tls": str(tls_s),
            "queue_release_target_edge": str(edge_s),
            "worst_edge_tls": str(tls_s),
            **dict(phase_diag or {}),
        }
        _fed_evt_main(
            "f2p.queue_release.applied",
            role="intersection",
            source_node=str(source_s),
            ev_id=str(ev_s),
            tls_id=str(tls_s),
            sim_time=float(sim_time),
            blocked=bool(payload.get("blocked", False)),
            reason=str(payload.get("reason", "")),
            worst_edge=str(edge_s),
            worst_edge_offset=int(worst_offset),
            decision_source="f2p_passive_queue_release_actuator",
            selected_action="queue_release_downstream_tls",
            **dict(apply_diag),
        )
        return True, apply_diag

    def _drone_context_trace_payload(
        *,
        rec: Optional[Dict[str, object]] = None,
        tls_id: str = "",
        ev_id: str = "",
        sim_time: float = -1.0,
        selected_action: str = "",
        decision_source: str = "",
        reason: str = "",
        context_age_sec: float = -1.0,
        request_latency_ms: float = -1.0,
        response_latency_ms: float = -1.0,
        extra: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        r = dict(rec or {})
        metrics = dict(r.get("metrics", {}) or {})
        extra_d = dict(extra or {})
        trace_wall_ts = float(time.time())
        age_ms = float(context_age_sec) * 1000.0 if float(context_age_sec) >= 0.0 else -1.0
        request_wall_ts = float(r.get("request_wall_ts", 0.0) or 0.0)
        drone_rx_wall_ts = float(r.get("drone_rx_wall_ts", 0.0) or 0.0)
        context_publish_wall_ts = float(r.get("context_publish_wall_ts", metrics.get("context_publish_wall_ts", 0.0)) or 0.0)
        realworld_rx_wall_ts = float(r.get("_rx_wall", r.get("realworld_rx_wall_ts", 0.0)) or 0.0)
        si_dt_rx_wall_ts = float(r.get("si_dt_rx_wall_ts", r.get("_rx_wall", realworld_rx_wall_ts)) or 0.0)
        directed_publish_wall_ts = float(r.get("directed_publish_wall_ts", extra_d.get("directed_publish_wall_ts", 0.0)) or 0.0)
        ttl_sec = float(r.get("ttl_sec", extra_d.get("ttl_sec", 0.0)) or 0.0)
        max_age_sec = float(extra_d.get("max_age_sec", ttl_sec) or ttl_sec or 0.0)
        freshness_window_sec = max(float(ttl_sec), float(max_age_sec))
        freshness_evaluated = bool(context_age_sec >= 0.0 and freshness_window_sec > 0.0)
        context_fresh = bool(
            freshness_evaluated and float(context_age_sec) <= float(freshness_window_sec)
        )
        context_validity_margin_ms = (
            (float(freshness_window_sec) - float(context_age_sec)) * 1000.0
            if freshness_evaluated
            else -1.0
        )
        request_to_realworld_rx_ms = (
            max(0.0, (realworld_rx_wall_ts - request_wall_ts) * 1000.0)
            if request_wall_ts > 0.0 and realworld_rx_wall_ts > 0.0
            else -1.0
        )
        request_to_si_dt_rx_ms = (
            max(0.0, (si_dt_rx_wall_ts - request_wall_ts) * 1000.0)
            if request_wall_ts > 0.0 and si_dt_rx_wall_ts > 0.0
            else -1.0
        )
        request_to_trace_ms = (
            max(0.0, (trace_wall_ts - request_wall_ts) * 1000.0)
            if request_wall_ts > 0.0
            else -1.0
        )
        drone_publish_to_realworld_rx_ms = (
            max(0.0, (realworld_rx_wall_ts - context_publish_wall_ts) * 1000.0)
            if context_publish_wall_ts > 0.0 and realworld_rx_wall_ts > 0.0
            else -1.0
        )
        drone_publish_to_si_dt_rx_ms = (
            max(0.0, (si_dt_rx_wall_ts - context_publish_wall_ts) * 1000.0)
            if context_publish_wall_ts > 0.0 and si_dt_rx_wall_ts > 0.0
            else -1.0
        )
        directed_publish_to_si_dt_rx_ms = (
            max(0.0, (si_dt_rx_wall_ts - directed_publish_wall_ts) * 1000.0)
            if directed_publish_wall_ts > 0.0 and si_dt_rx_wall_ts > 0.0
            else -1.0
        )
        si_dt_rx_to_trace_ms = (
            max(0.0, (trace_wall_ts - si_dt_rx_wall_ts) * 1000.0)
            if si_dt_rx_wall_ts > 0.0
            else -1.0
        )
        return {
            "role": "intersection",
            "sim_time": float(sim_time),
            "trace_wall_ts": float(trace_wall_ts),
            "trace_wall_ms": float(trace_wall_ts * 1000.0),
            "request_id": str(r.get("request_id", "")),
            "ev_id": str(ev_id or r.get("ev_id", "")),
            "requester_tls": str(tls_id or r.get("requester_tls", r.get("tls_id", ""))),
            "tls_id": str(tls_id or r.get("requester_tls", r.get("tls_id", ""))),
            "provider_id": str(r.get("provider_id", "")),
            "provider_type": str(r.get("provider_type", "")),
            "target_edges": list(r.get("target_route_edges", r.get("lookahead_edges", [])) or []),
            "context_scope": str(r.get("context_scope", "")),
            "context_request_model": str(r.get("context_request_model", "")),
            "mission_request_id": str(r.get("mission_request_id", "")),
            "mission_name": str(r.get("mission_name", "")),
            "mission_ev_id": str(r.get("mission_ev_id", r.get("ev_id", ""))),
            "mission_route_id": str(r.get("mission_route_id", r.get("route_id", ""))),
            "waypoint_index": int(r.get("waypoint_index", -1) or -1),
            "waypoint_id": str(r.get("waypoint_id", "")),
            "waypoint_edge": str(r.get("waypoint_edge", "")),
            "waypoint_kind": str(r.get("waypoint_kind", "")),
            "waypoint_node": str(r.get("waypoint_node", "")),
            "waypoint_node_type": str(r.get("waypoint_node_type", "")),
            "waypoint_region_id": str(r.get("waypoint_region_id", "")),
            "waypoint_region_label": str(r.get("waypoint_region_label", "")),
            "waypoint_region_from": str(r.get("waypoint_region_from", "")),
            "waypoint_region_to": str(r.get("waypoint_region_to", "")),
            "waypoint_sumo_x": float(r.get("waypoint_sumo_x", -1.0) or -1.0),
            "waypoint_sumo_y": float(r.get("waypoint_sumo_y", -1.0) or -1.0),
            "decision_deadline_sec": float(ttl_sec),
            "ttl_sec": float(ttl_sec),
            "max_age_sec": float(max_age_sec),
            "freshness_window_sec": float(freshness_window_sec),
            "context_freshness_evaluated": bool(freshness_evaluated),
            "context_fresh": bool(context_fresh),
            "context_valid": bool(context_fresh),
            "context_validity_margin_ms": float(context_validity_margin_ms),
            "request_latency_ms": float(
                request_latency_ms
                if request_latency_ms >= 0.0
                else metrics.get("task_acceptance_latency_ms", -1.0) or -1.0
            ),
            "response_latency_ms": float(
                response_latency_ms
                if response_latency_ms >= 0.0
                else metrics.get("request_to_context_latency_ms", -1.0) or -1.0
            ),
            "request_to_drone_rx_latency_ms": float(metrics.get("request_to_drone_rx_latency_ms", -1.0) or -1.0),
            "mission_latency_ms": float(metrics.get("mission_latency_ms", 0.0) or 0.0),
            "observation_latency_ms": float(metrics.get("observation_latency_ms", -1.0) or -1.0),
            "sumo_proxy_latency_ms": float(metrics.get("sumo_proxy_latency_ms", -1.0) or -1.0),
            "request_to_publish_latency_ms": float(metrics.get("request_to_publish_latency_ms", -1.0) or -1.0),
            "drone_rx_to_publish_latency_ms": float(metrics.get("drone_rx_to_publish_latency_ms", -1.0) or -1.0),
            "request_to_realworld_rx_latency_ms": float(request_to_realworld_rx_ms),
            "request_to_si_dt_rx_latency_ms": float(request_to_si_dt_rx_ms),
            "request_to_trace_latency_ms": float(request_to_trace_ms),
            "drone_publish_to_realworld_rx_latency_ms": float(drone_publish_to_realworld_rx_ms),
            "drone_publish_to_si_dt_rx_latency_ms": float(drone_publish_to_si_dt_rx_ms),
            "directed_publish_to_si_dt_rx_latency_ms": float(directed_publish_to_si_dt_rx_ms),
            "si_dt_rx_to_trace_latency_ms": float(si_dt_rx_to_trace_ms),
            "si_dt_rx_to_context_use_latency_ms": float(si_dt_rx_to_trace_ms),
            "context_age_ms": float(age_ms),
            "request_wall_ts": float(request_wall_ts),
            "drone_rx_wall_ts": float(drone_rx_wall_ts),
            "context_publish_wall_ts": float(context_publish_wall_ts),
            "realworld_rx_wall_ts": float(realworld_rx_wall_ts),
            "si_dt_rx_wall_ts": float(si_dt_rx_wall_ts),
            "si_dt_rx_wall_ms": float(si_dt_rx_wall_ts * 1000.0 if si_dt_rx_wall_ts > 0.0 else 0.0),
            "directed_publish_wall_ts": float(directed_publish_wall_ts),
            "directed_publish_wall_ms": float(directed_publish_wall_ts * 1000.0 if directed_publish_wall_ts > 0.0 else 0.0),
            "directed_topic": str(r.get("directed_topic", extra_d.get("directed_topic", ""))),
            "contextual_topic": str(r.get("contextual_topic", extra_d.get("contextual_topic", ""))),
            "request_payload_size_bytes": int(r.get("request_payload_size_bytes", metrics.get("request_payload_size_bytes", 0)) or 0),
            "response_payload_size_bytes": int(metrics.get("payload_size_bytes", r.get("payload_size_bytes", 0)) or 0),
            "drone_rx_payload_size_bytes": int(metrics.get("drone_rx_payload_size_bytes", 0) or 0),
            "directed_payload_size_bytes": int(r.get("directed_payload_size_bytes", extra_d.get("directed_payload_size_bytes", 0)) or 0),
            "directed_payload_size_bytes_rx": int(r.get("directed_payload_size_bytes_rx", extra_d.get("directed_payload_size_bytes_rx", 0)) or 0),
            "blocked": bool(r.get("blocked", False)),
            "reason": str(reason or r.get("reason", "")),
            "worst_edge": str(r.get("worst_edge", "")),
            "worst_edge_offset": int(r.get("worst_edge_offset", -1) or -1),
            "confidence": float(r.get("confidence", 0.0) or 0.0),
            "selected_action": str(selected_action),
            "decision_source": str(decision_source),
            **extra_d,
        }

    def _f2d_context_fanout_tls(rec: Dict[str, object], requester_tls: str) -> List[str]:
        """Map a Drone-DT route-edge observation to active TLS consumers.

        F2D is the only mode that uses this path. Plain F2 and F2P keep their
        existing context sources because external context is force-disabled for
        non-drone modes during evaluation startup.
        """
        out: List[str] = []

        def _add_tls(tls_id: object) -> None:
            tls_s = str(tls_id or "").strip()
            if tls_s and tls_s not in out:
                out.append(tls_s)

        _add_tls(requester_tls)
        for key in ("preferred_next_tls", "feedback_responder_tls", "target_tls", "next_tls"):
            _add_tls(rec.get(key, ""))
        route_ctx = dict(rec.get("route_context", {}) or {})
        for key in ("current_tls", "requester_tls", "preferred_next_tls", "feedback_responder_tls"):
            _add_tls(route_ctx.get(key, ""))
        for tls_id in list(route_ctx.get("downstream_tls", []) or []):
            _add_tls(tls_id)

        edges: List[str] = []
        for key in ("target_route_edges", "lookahead_edges", "target_edges", "edges"):
            raw = rec.get(key, [])
            if isinstance(raw, str):
                vals = [x.strip() for x in raw.replace(";", ",").split(",")]
            elif isinstance(raw, list):
                vals = [str(x).strip() for x in raw]
            else:
                vals = []
            for edge_id in vals:
                if edge_id and edge_id not in edges:
                    edges.append(edge_id)
        for edge_id in edges:
            try:
                _add_tls(edge_to_tls.get(str(edge_id), ""))
            except Exception:
                pass
        return out

    def _cache_directed_external_downstream_context(
        *,
        rec: Dict[str, object],
        tls_id: str,
        ev_id: str,
        requester_tls: str,
        sim_time: float,
        decision_source: str,
        selected_action: str,
    ) -> None:
        """Commit a delivered external context artifact into one SI-DT cache."""
        tls_s = str(tls_id or "").strip()
        ev_s = str(ev_id or rec.get("ev_id", "") or "").strip()
        if not tls_s:
            return
        rx_wall = float(time.time())
        rec_for_tls = dict(rec or {})
        delivery_key = (
            str(rec_for_tls.get("request_id", "")),
            str(tls_s),
            str(ev_s),
            str(rec_for_tls.get("waypoint_index", "")),
            str(rec_for_tls.get("waypoint_id", rec_for_tls.get("waypoint_edge", ""))),
        )
        if delivery_key in f2d_directed_context_seen:
            _fed_evt_main(
                "f2d.context.si_dt_duplicate_drop",
                **_drone_context_trace_payload(
                    rec=rec_for_tls,
                    tls_id=str(tls_s),
                    ev_id=str(ev_s),
                    sim_time=float(sim_time),
                    selected_action="drop_duplicate_directed_context",
                    decision_source=str(decision_source),
                    reason=str(rec_for_tls.get("reason", "")),
                    context_age_sec=0.0,
                    extra={
                        "target_tls_id": str(tls_s),
                        "delivery_model": str(rec_for_tls.get("delivery_model", "f2d_directed_si_context")),
                    },
                ),
            )
            return
        f2d_directed_context_seen.add(delivery_key)
        rec_for_tls["_rx_wall"] = float(rx_wall)
        rec_for_tls["si_dt_rx_wall_ts"] = float(rx_wall)
        rec_for_tls["si_dt_rx_wall_ms"] = float(rx_wall * 1000.0)
        rec_for_tls["_rx_sim_time"] = float(sim_time)
        rec_for_tls["_fanout_from_tls"] = str(requester_tls or rec_for_tls.get("_fanout_from_tls", ""))
        rec_for_tls["_fanout_tls"] = str(tls_s)
        rec_for_tls["target_tls_id"] = str(tls_s)
        rec_for_tls["delivery_model"] = str(
            rec_for_tls.get("delivery_model", "f2d_directed_si_context")
        )
        try:
            rec_for_tls["directed_payload_size_bytes_rx"] = int(
                len(json.dumps(rec_for_tls, ensure_ascii=True).encode("utf-8"))
            )
        except Exception:
            rec_for_tls["directed_payload_size_bytes_rx"] = 0
        directed_publish_wall_ts = float(rec_for_tls.get("directed_publish_wall_ts", 0.0) or 0.0)
        si_dt_delivery_latency_ms = (
            max(0.0, (rx_wall - directed_publish_wall_ts) * 1000.0)
            if directed_publish_wall_ts > 0.0
            else -1.0
        )
        external_downstream_context_cache[(str(tls_s), str(ev_s))] = dict(rec_for_tls)
        if ev_s:
            external_downstream_context_cache[(str(tls_s), "")] = dict(rec_for_tls)
        trace_extra = {
            "requester_tls": str(requester_tls or rec_for_tls.get("_fanout_from_tls", "")),
            "fanout_from_tls": str(rec_for_tls.get("_fanout_from_tls", "")),
            "fanout_tls": str(tls_s),
            "target_tls_id": str(tls_s),
            "delivery_model": str(rec_for_tls.get("delivery_model", "")),
            "directed_publish_wall_ts": float(directed_publish_wall_ts),
            "si_dt_rx_wall_ts": float(rx_wall),
            "si_dt_delivery_latency_ms": float(si_dt_delivery_latency_ms),
            "directed_payload_size_bytes_rx": int(rec_for_tls.get("directed_payload_size_bytes_rx", 0) or 0),
            "source_equivalent": str(
                rec_for_tls.get(
                    "source_equivalent",
                    rec_for_tls.get("context_source_equivalent", "mobile_passive_dt"),
                )
            ),
        }
        _fed_evt_main(
            "f2d.context.si_dt_received",
            **_drone_context_trace_payload(
                rec=rec_for_tls,
                tls_id=str(tls_s),
                ev_id=str(ev_s),
                sim_time=float(sim_time),
                selected_action=str(selected_action),
                decision_source=str(decision_source),
                reason=str(rec_for_tls.get("reason", "")),
                context_age_sec=0.0,
                extra=trace_extra,
            ),
        )
        _fed_evt_main(
            "f2d.context.si_dt_cache_update",
            **_drone_context_trace_payload(
                rec=rec_for_tls,
                tls_id=str(tls_s),
                ev_id=str(ev_s),
                sim_time=float(sim_time),
                selected_action="cache_delivered_context",
                decision_source=str(decision_source),
                reason=str(rec_for_tls.get("reason", "")),
                context_age_sec=0.0,
                extra=trace_extra,
            ),
        )

    def _evict_external_downstream_context_rec(rec: Dict[str, object], tls_id: str, ev_id: str) -> int:
        """Remove stale aliases for one external context record."""
        req_id = str(rec.get("request_id", "") or "")
        provider_id = str(rec.get("provider_id", "") or "")
        fanout_from = str(rec.get("_fanout_from_tls", "") or "")
        fanout_tls = str(rec.get("_fanout_tls", "") or "")
        removed = 0
        for key, cached in list(external_downstream_context_cache.items()):
            cached_d = dict(cached or {})
            same_identity = bool(req_id and str(cached_d.get("request_id", "") or "") == req_id)
            if not same_identity:
                same_identity = bool(
                    provider_id
                    and str(cached_d.get("provider_id", "") or "") == provider_id
                    and str(key[0]) == str(tls_id)
                    and str(key[1]) in {str(ev_id), ""}
                )
            if same_identity or (
                fanout_from
                and fanout_tls
                and str(cached_d.get("_fanout_from_tls", "") or "") == fanout_from
                and str(cached_d.get("_fanout_tls", "") or "") == fanout_tls
                and str(key[1]) in {str(ev_id), ""}
            ):
                external_downstream_context_cache.pop(key, None)
                removed += 1
        return int(removed)

    def _merge_external_downstream_context(
        diag: Dict[str, Any],
        *,
        tls_id: str,
        ev_id: str,
        sim_time: float,
        max_age_sec: float = 2.0,
    ) -> Dict[str, Any]:
        """Merge fresh Drone-DT downstream context into local guard diagnostics.

        External context is safety-only: it can mark the corridor as blocked or
        provide a worse edge, but it never clears a locally observed blockage.
        """
        out = dict(diag or {})
        if not bool(getattr(args, "external_downstream_context_enable", False)):
            out["external_downstream_context_enabled"] = False
            out["external_downstream_context_used"] = False
            return out
        out["external_downstream_context_enabled"] = True
        keys = [(str(tls_id), str(ev_id)), (str(tls_id), "")]
        rec: Dict[str, object] = {}
        for key in keys:
            candidate = dict(external_downstream_context_cache.get(key, {}) or {})
            if candidate:
                rec = candidate
                break
        if not rec:
            out["external_downstream_context_used"] = False
            if _is_drone_augmented_mode(CURRENT_EVALUATION):
                missing_key = (str(tls_id), str(ev_id))
                try:
                    missing_now = float(sim_time) if float(sim_time) >= 0.0 else float(time.time())
                except Exception:
                    missing_now = float(time.time())
                last_missing = float(external_downstream_context_missing_trace_recent.get(missing_key, -1.0e12))
                if missing_now - last_missing >= 1.0:
                    external_downstream_context_missing_trace_recent[missing_key] = float(missing_now)
                    _fed_evt_main(
                        "f2.drone_context.missing",
                        **_drone_context_trace_payload(
                            rec={},
                            tls_id=str(tls_id),
                            ev_id=str(ev_id),
                            sim_time=float(sim_time),
                            selected_action="continue_without_drone_context",
                            decision_source="external_downstream_context_merge",
                            reason="no_cached_context_for_si_dt",
                            context_age_sec=-1.0,
                            extra={
                                "max_age_sec": float(max_age_sec),
                                "ttl_sec": float(max_age_sec),
                                "context_required": True,
                                "cache_hit": False,
                                "cache_keys_checked": [list(k) for k in keys],
                                "missing_trace_rate_limit_sec": 1.0,
                            },
                        ),
                    )
            return out
        try:
            now_wall = float(time.time())
            rec_wall = float(rec.get("_rx_wall", 0.0) or 0.0)
            ttl = float(rec.get("ttl_sec", max_age_sec) or max_age_sec)
            age = max(0.0, now_wall - rec_wall)
        except Exception:
            ttl = float(max_age_sec)
            age = 999999.0
        if age > max(float(max_age_sec), float(ttl)):
            out["external_downstream_context_used"] = False
            out["external_downstream_context_stale"] = True
            out["external_downstream_context_age_sec"] = float(age)
            evicted_n = _evict_external_downstream_context_rec(rec, str(tls_id), str(ev_id))
            _fed_evt_main(
                "f2.drone_context.stale",
                **_drone_context_trace_payload(
                    rec=rec,
                    tls_id=str(tls_id),
                    ev_id=str(ev_id),
                    sim_time=float(sim_time),
                    selected_action="ignore_stale_context",
                    decision_source="external_downstream_context_merge",
                    reason="context_ttl_expired",
                    context_age_sec=float(age),
                    extra={
                        "max_age_sec": float(max_age_sec),
                        "ttl_sec": float(ttl),
                        "cache_evicted": True,
                        "cache_evicted_n": int(evicted_n),
                    },
                ),
            )
            if _f2d_mobile_passive_context_enabled():
                _fed_evt_main(
                    "f2d.mobile_passive.stale",
                    **_drone_context_trace_payload(
                        rec=rec,
                        tls_id=str(tls_id),
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        selected_action="ignore_stale_mobile_passive_context",
                        decision_source="external_downstream_context_merge",
                        reason="context_ttl_expired",
                        context_age_sec=float(age),
                        extra={
                            "max_age_sec": float(max_age_sec),
                            "ttl_sec": float(ttl),
                            "fanout_from_tls": str(rec.get("_fanout_from_tls", "")),
                            "fanout_tls": str(rec.get("_fanout_tls", "")),
                            "cache_evicted": True,
                            "cache_evicted_n": int(evicted_n),
                        },
                    ),
                )
            return out

        ext_blocked = bool(rec.get("blocked", False))
        local_blocked_before = bool(out.get("blocked", False))
        local_reason_before = str(out.get("reason", ""))
        local_worst_before = str(out.get("worst_edge", ""))
        out["external_downstream_context_used"] = True
        out["external_downstream_context_provider"] = str(rec.get("provider_id", ""))
        out["external_downstream_context_provider_type"] = str(rec.get("provider_type", ""))
        out["external_downstream_context_age_sec"] = float(age)
        out["external_downstream_context_request_id"] = str(rec.get("request_id", ""))
        out["external_downstream_context_blocked"] = bool(ext_blocked)
        out["external_downstream_context_reason"] = str(rec.get("reason", ""))
        out["external_downstream_context_confidence"] = float(rec.get("confidence", 0.0) or 0.0)
        out["external_downstream_context_fanout_from_tls"] = str(rec.get("_fanout_from_tls", ""))
        out["external_downstream_context_fanout_tls"] = str(rec.get("_fanout_tls", ""))
        out["external_downstream_context_source_equivalent"] = str(
            rec.get("source_equivalent", rec.get("context_source_equivalent", ""))
        )
        conflict = bool(
            (local_blocked_before != ext_blocked)
            or (
                local_blocked_before
                and ext_blocked
                and local_worst_before
                and str(rec.get("worst_edge", "")) not in ("", local_worst_before)
            )
        )
        if conflict:
            _fed_evt_main(
                "f2.drone_context.conflict_with_peer",
                **_drone_context_trace_payload(
                    rec=rec,
                    tls_id=str(tls_id),
                    ev_id=str(ev_id),
                    sim_time=float(sim_time),
                    selected_action="keep_safety_union",
                    decision_source="external_downstream_context_merge",
                    reason="local_vs_drone_context_disagreement",
                    context_age_sec=float(age),
                    extra={
                        "local_blocked": bool(local_blocked_before),
                        "local_reason": str(local_reason_before),
                        "local_worst_edge": str(local_worst_before),
                        "drone_blocked": bool(ext_blocked),
                    },
                ),
            )
        _fed_evt_main(
            "f2.drone_context.used",
            **_drone_context_trace_payload(
                rec=rec,
                tls_id=str(tls_id),
                ev_id=str(ev_id),
                sim_time=float(sim_time),
                selected_action="merge_safety_context",
                decision_source="external_downstream_context_merge",
                reason=str(rec.get("reason", "clear")),
                context_age_sec=float(age),
                extra={
                    "local_blocked_before": bool(local_blocked_before),
                    "local_reason_before": str(local_reason_before),
                    "local_worst_edge_before": str(local_worst_before),
                    "conflict": bool(conflict),
                },
            ),
        )
        if _f2d_mobile_passive_context_enabled():
            _fed_evt_main(
                "f2d.mobile_passive.used",
                **_drone_context_trace_payload(
                    rec=rec,
                    tls_id=str(tls_id),
                    ev_id=str(ev_id),
                    sim_time=float(sim_time),
                    selected_action="merge_mobile_passive_context",
                    decision_source="external_downstream_context_merge",
                    reason=str(rec.get("reason", "clear")),
                    context_age_sec=float(age),
                    extra={
                        "local_blocked_before": bool(local_blocked_before),
                        "local_reason_before": str(local_reason_before),
                        "local_worst_edge_before": str(local_worst_before),
                        "conflict": bool(conflict),
                        "fanout_from_tls": str(rec.get("_fanout_from_tls", "")),
                        "fanout_tls": str(rec.get("_fanout_tls", "")),
                        "source_equivalent": str(
                            rec.get("source_equivalent", rec.get("context_source_equivalent", "mobile_passive_dt"))
                        ),
                    },
                ),
            )
        if not ext_blocked:
            return out

        # Only tighten the guard. Do not let external context erase local evidence.
        out["blocked"] = True
        out["reason"] = f"external_drone:{str(rec.get('reason', 'blocked'))}"
        out["worst_edge"] = str(rec.get("worst_edge", out.get("worst_edge", "")))
        out["worst_edge_offset"] = int(rec.get("worst_edge_offset", out.get("worst_edge_offset", -1)) or -1)
        out["lookahead_edges"] = list(rec.get("lookahead_edges", out.get("lookahead_edges", [])) or [])
        out["lookahead_edges_n"] = int(rec.get("lookahead_edges_n", len(out.get("lookahead_edges", []) or [])) or 0)
        out["max_halt_n"] = max(int(out.get("max_halt_n", 0) or 0), int(rec.get("max_halt_n", 0) or 0))
        out["max_veh_n"] = max(int(out.get("max_veh_n", 0) or 0), int(rec.get("max_veh_n", 0) or 0))
        out["max_occupancy_pct"] = max(
            float(out.get("max_occupancy_pct", 0.0) or 0.0),
            float(rec.get("max_occupancy_pct", 0.0) or 0.0),
        )
        ext_speed = float(rec.get("min_mean_speed_mps", -1.0) or -1.0)
        cur_speed = float(out.get("min_mean_speed_mps", -1.0) or -1.0)
        if ext_speed >= 0.0 and (cur_speed < 0.0 or ext_speed < cur_speed):
            out["min_mean_speed_mps"] = float(ext_speed)
        if _f2d_mobile_passive_context_enabled():
            worst_offset_for_advisory = int(rec.get("worst_edge_offset", out.get("worst_edge_offset", -1)) or -1)
            worst_edge_for_recovery = str(rec.get("worst_edge", out.get("worst_edge", "")) or "")
            try:
                worst_edge_tls = str(edge_to_tls.get(worst_edge_for_recovery, "") or "")
            except Exception:
                worst_edge_tls = ""
            active_tls_available = bool(worst_edge_tls and worst_edge_tls in agents)
            requester_tls_s = str(tls_id)
            queue_release_candidate = bool(
                active_tls_available
                and worst_edge_tls == requester_tls_s
                and int(worst_offset_for_advisory) <= 1
            )
            downstream_queue_release_candidate = bool(
                active_tls_available
                and worst_edge_tls != requester_tls_s
                and int(worst_offset_for_advisory) >= 1
            )
            recovery_action = (
                "queue_release_local_tls"
                if queue_release_candidate
                else "queue_release_downstream_tls"
                if downstream_queue_release_candidate
                else "reroute_advisory"
            )
            recovery_seen_key = (
                str(rec.get("request_id", "")),
                str(tls_id),
                str(ev_id),
                str(recovery_action),
            )
            if recovery_seen_key not in f2d_recovery_trace_seen:
                f2d_recovery_trace_seen.add(recovery_seen_key)
                _fed_evt_main(
                    "f2d.drone_context.recovery_candidate",
                    **_drone_context_trace_payload(
                        rec=rec,
                        tls_id=str(tls_id),
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        selected_action=str(recovery_action),
                        decision_source="f2d_mobile_passive_context",
                        reason=str(rec.get("reason", "blocked")),
                        context_age_sec=float(age),
                        extra={
                            "worst_edge_tls": str(worst_edge_tls),
                            "active_tls_available": bool(active_tls_available),
                            "queue_release_candidate": bool(queue_release_candidate),
                            "downstream_queue_release_candidate": bool(downstream_queue_release_candidate),
                            "reroute_candidate": bool(
                                not queue_release_candidate and not downstream_queue_release_candidate
                            ),
                            "route_edges_n": int(len(list(rec.get("route_edges", []) or []))),
                            "remaining_route_edges_n": int(len(list(rec.get("remaining_route_edges", []) or []))),
                            "target_edges_n": int(len(list(rec.get("target_route_edges", rec.get("lookahead_edges", [])) or []))),
                        },
                    ),
                )
                if queue_release_candidate or downstream_queue_release_candidate:
                    queue_release_applied, queue_release_diag = _try_apply_f2d_queue_release(
                        rec=rec,
                        requester_tls=str(tls_id),
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        worst_edge_tls=str(worst_edge_tls),
                        worst_edge=str(worst_edge_for_recovery),
                        recovery_action=str(recovery_action),
                        context_age_sec=float(age),
                    )
                    _fed_evt_main(
                        "f2d.queue_release.requested",
                        **_drone_context_trace_payload(
                            rec=rec,
                            tls_id=str(tls_id),
                            ev_id=str(ev_id),
                            sim_time=float(sim_time),
                            selected_action=str(recovery_action),
                            decision_source="f2d_mobile_passive_context",
                            reason="drone_confirmed_blockage_queue_release_candidate",
                            context_age_sec=float(age),
                            extra={
                                "queue_release_applied": bool(queue_release_applied),
                                "queue_release_application": str(
                                    dict(queue_release_diag or {}).get(
                                        "queue_release_application",
                                        "traci_set_phase_duration" if queue_release_applied else "advisory_trace_only",
                                    )
                                ),
                                "worst_edge_tls": str(worst_edge_tls),
                                "active_tls_available": bool(active_tls_available),
                                **dict(queue_release_diag or {}),
                            },
                        ),
                    )
            _fed_evt_main(
                "f2d.mobile_passive.blockage_detected",
                **_drone_context_trace_payload(
                    rec=rec,
                    tls_id=str(tls_id),
                    ev_id=str(ev_id),
                    sim_time=float(sim_time),
                    selected_action="tighten_downstream_guard",
                    decision_source="external_downstream_context_merge",
                    reason=str(rec.get("reason", "blocked")),
                    context_age_sec=float(age),
                    extra={
                        "fanout_from_tls": str(rec.get("_fanout_from_tls", "")),
                        "fanout_tls": str(rec.get("_fanout_tls", "")),
                        "source_equivalent": str(
                            rec.get("source_equivalent", rec.get("context_source_equivalent", "mobile_passive_dt"))
                        ),
                    },
                ),
            )
            advisory_enabled = bool(getattr(args, "f2d_advisory_reroute_enable", True))
            advisory_min_offset = max(
                1,
                int(getattr(args, "f2d_advisory_reroute_min_worst_edge_offset", 2) or 2),
            )
            if advisory_enabled and int(worst_offset_for_advisory) >= int(advisory_min_offset):
                _fed_evt_main(
                    "f2d.ev_advisory.reroute_recommended",
                    **_drone_context_trace_payload(
                        rec=rec,
                        tls_id=str(tls_id),
                        ev_id=str(ev_id),
                        sim_time=float(sim_time),
                        selected_action="advise_ev_reroute",
                        decision_source="f2d_mobile_passive_context",
                        reason="drone_confirmed_blockage_beyond_local_tls",
                        context_age_sec=float(age),
                        extra={
                            "fanout_from_tls": str(rec.get("_fanout_from_tls", "")),
                            "fanout_tls": str(rec.get("_fanout_tls", "")),
                            "advisory_only": True,
                            "reroute_applied": False,
                            "min_worst_edge_offset": int(advisory_min_offset),
                            "local_blocked_before": bool(local_blocked_before),
                            "local_reason_before": str(local_reason_before),
                            "source_equivalent": str(
                                rec.get("source_equivalent", rec.get("context_source_equivalent", "mobile_passive_dt"))
                            ),
                        },
                    ),
                )
        return out

    def _f2_local_current_plan_for_actuation(ag: object) -> Tuple[Optional[object], str]:
        """
        F2 is B1 plus downstream federation, not a replacement for local EV priority.
        When the current local plan is actionable, preserve it at actuation time so
        peer offer scoring cannot weaken the EV-facing TLS service window.
        """
        if getattr(ag, "active_ev", None) is None:
            return None, ""
        # Prefer the freshly computed B1-equivalent primary plan generated by
        # compute_offers(). current_plan can already reflect a previous
        # selected F2 offer, so using it first lets weak peer offers shadow the
        # local rescue path.
        for plan in (getattr(ag, "_last_f2_primary_plan", None), getattr(ag, "current_plan", None)):
            if plan is None:
                continue
            plan_type = str(getattr(plan, "plan_type", "") or "")
            if plan_type in ("", "none", "restore"):
                continue
            return plan, plan_type
        plan = getattr(ag, "_last_f2_primary_plan", None) or getattr(ag, "current_plan", None)
        return None, str(getattr(plan, "plan_type", "") or "")

    def _downstream_immediate_blockage_severe(diag: Dict[str, object]) -> bool:
        """Classify route-edge-1 spillback that should suppress active priority."""
        if not bool(getattr(args, "downstream_immediate_blockage_guard_enable", True)):
            return False
        if not bool((diag or {}).get("blocked", False)):
            return False
        try:
            worst_offset = int((diag or {}).get("worst_edge_offset", -1) or -1)
        except Exception:
            worst_offset = -1
        max_offset = max(
            1,
            int(getattr(args, "downstream_immediate_blockage_max_worst_edge_offset", 1) or 1),
        )
        if worst_offset <= 0 or worst_offset > max_offset:
            return False
        try:
            max_halt_n = int((diag or {}).get("max_halt_n", 0) or 0)
        except Exception:
            max_halt_n = 0
        try:
            max_veh_n = int((diag or {}).get("max_veh_n", 0) or 0)
        except Exception:
            max_veh_n = 0
        try:
            min_mean_speed_mps = float((diag or {}).get("min_mean_speed_mps", 999.0))
        except Exception:
            min_mean_speed_mps = 999.0
        return bool(
            max_halt_n >= int(getattr(args, "downstream_immediate_blockage_min_halt_n", 3) or 3)
            and max_veh_n >= int(getattr(args, "downstream_immediate_blockage_min_veh_n", 6) or 6)
            and min_mean_speed_mps <= float(
                getattr(args, "downstream_immediate_blockage_max_mean_speed_mps", 0.5) or 0.5
            )
        )

    def _annotate_immediate_blockage_diag(diag: Dict[str, object]) -> Dict[str, object]:
        out = dict(diag or {})
        out["immediate_blockage_guard_enabled"] = bool(
            getattr(args, "downstream_immediate_blockage_guard_enable", True)
        )
        out["immediate_blockage_max_worst_edge_offset"] = max(
            1,
            int(getattr(args, "downstream_immediate_blockage_max_worst_edge_offset", 1) or 1),
        )
        out["immediate_blockage_min_halt_n"] = int(
            getattr(args, "downstream_immediate_blockage_min_halt_n", 3) or 3
        )
        out["immediate_blockage_min_veh_n"] = int(
            getattr(args, "downstream_immediate_blockage_min_veh_n", 6) or 6
        )
        out["immediate_blockage_max_mean_speed_mps"] = float(
            getattr(args, "downstream_immediate_blockage_max_mean_speed_mps", 0.5) or 0.5
        )
        out["immediate_blockage_severe"] = bool(_downstream_immediate_blockage_severe(out))
        return out

    def _plain_f2_tls_bounded_downstream_scan() -> bool:
        """Keep F2/F2D/F2D-Q from using non-TLS observability that belongs to F2P/F2PD."""
        return str(CURRENT_EVALUATION).upper() in {"F2", "F2D", "F2D-Q"}

    def _clone_plan_with_extension_cap(plan: object, cap_sec: float, reason: str) -> object:
        """Return a PreemptionPlan copy with a bounded extension budget."""
        cap = max(0.0, float(cap_sec))
        notes = str(getattr(plan, "notes", "") or "")
        if notes:
            notes = f"{notes}; "
        notes = f"{notes}lookahead_guard:{reason}:ext_cap={cap:.2f}s"
        return intersection_agent_module.PreemptionPlan(
            plan_type=str(getattr(plan, "plan_type", "") or ""),
            target_phase_idx=int(getattr(plan, "target_phase_idx", 0) or 0),
            extend_green_sec=min(float(getattr(plan, "extend_green_sec", 0.0) or 0.0), cap),
            hurry_current_phase_to_sec=getattr(plan, "hurry_current_phase_to_sec", None),
            jump_time_sec=getattr(plan, "jump_time_sec", None),
            jump_to_phase_idx=getattr(plan, "jump_to_phase_idx", None),
            planned_green_window=getattr(plan, "planned_green_window", None),
            phase_duration_overrides=getattr(plan, "phase_duration_overrides", None),
            override_start_time_sec=getattr(plan, "override_start_time_sec", None),
            override_end_time_sec=getattr(plan, "override_end_time_sec", None),
            notes=notes,
        )

    def _f2_downstream_release_guard_diag(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        plan: object,
        lookahead_diag: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """Detect when an F2 release would push the EV into already-blocked route edges."""
        enabled = bool(getattr(args, "f2_downstream_release_guard_enable", True))
        plan_type = str(getattr(plan, "plan_type", "") or "") if plan is not None else ""
        diag: Dict[str, object] = {
            "enabled": bool(enabled),
            "blocked": False,
            "skip_release": False,
            "reason": "disabled" if not enabled else "clear",
            "stage": str(stage),
            "ev_edge": str(ev_edge or ""),
            "selected_in_edge": str(selected_in_edge or ""),
            "plan_type": str(plan_type),
        }
        if not enabled:
            return diag
        if plan is None or plan_type in ("", "none", "restore"):
            diag["reason"] = "weak_plan"
            return diag

        guard = b1_downstream_blockage_diag(
            ev_id=str(ev_id),
            current_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_edges=int(getattr(args, "f2_downstream_release_guard_lookahead_edges", 8)),
            min_halt_n=int(getattr(args, "f2_downstream_release_guard_min_halt_n", 2)),
            max_mean_speed_mps=float(getattr(args, "f2_downstream_release_guard_max_mean_speed_mps", 2.0)),
            min_veh_n=int(getattr(args, "f2_downstream_release_guard_min_veh_n", 3)),
            max_occupancy_pct=float(getattr(args, "f2_downstream_release_guard_max_occupancy_pct", 35.0)),
            edge_to_tls_map=edge_to_tls,
            stop_at_non_tls=bool(_plain_f2_tls_bounded_downstream_scan()),
        )
        diag.update(
            {
                "blocked": bool(guard.get("blocked", False)),
                "reason": str(guard.get("reason", "clear")),
                "worst_edge": str(guard.get("worst_edge", "")),
                "lookahead_edges": list(guard.get("lookahead_edges", []) or []),
                "lookahead_edges_n": int(guard.get("lookahead_edges_n", 0) or 0),
                "scan_scope": str(guard.get("scan_scope", "")),
                "scan_limited_by_non_tls": bool(guard.get("scan_limited_by_non_tls", False)),
                "non_tls_boundary_edge": str(guard.get("non_tls_boundary_edge", "")),
                "max_halt_n": int(guard.get("max_halt_n", 0) or 0),
                "max_veh_n": int(guard.get("max_veh_n", 0) or 0),
                "max_occupancy_pct": float(guard.get("max_occupancy_pct", 0.0) or 0.0),
                "min_mean_speed_mps": float(guard.get("min_mean_speed_mps", -1.0) or -1.0),
            }
        )
        diag["skip_release"] = bool(guard.get("blocked", False))
        if bool(diag["skip_release"]):
            la = dict(lookahead_diag or {})
            _fed_evt_main(
                "f2.downstream_release_guard.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                route_lookahead=bool(la.get("route_lookahead", False)),
                lookahead_hops=int(la.get("lookahead_hops", 0) or 0),
                route_distance_to_selected_edge_m=la.get("route_distance_to_selected_edge_m"),
                upstream_stopped=bool(la.get("upstream_stopped", False)),
                lookahead_action=str(la.get("action", "")),
                lookahead_reason=str(la.get("reason", "")),
                **diag,
            )
        return diag

    def _f2_downstream_replay_guard_diag(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        plan: object,
        lookahead_diag: Optional[Dict[str, object]] = None,
        source: str = "",
    ) -> Dict[str, object]:
        """Suppress repeated F2 continuity/keepalive when downstream route edges are blocked."""
        enabled = bool(getattr(args, "f2_downstream_replay_guard_enable", True))
        plan_type = str(getattr(plan, "plan_type", "") or "") if plan is not None else ""
        diag: Dict[str, object] = {
            "enabled": bool(enabled),
            "blocked": False,
            "skip_replay": False,
            "reason": "disabled" if not enabled else "clear",
            "stage": str(stage),
            "source": str(source),
            "ev_edge": str(ev_edge or ""),
            "selected_in_edge": str(selected_in_edge or ""),
            "plan_type": str(plan_type),
        }
        if not enabled:
            return diag
        if plan is None or plan_type in ("", "none", "restore"):
            diag["reason"] = "weak_plan"
            return diag
        if plan_type == "intrusive":
            diag["reason"] = "intrusive_preserved"
            return diag

        guard = b1_downstream_blockage_diag(
            ev_id=str(ev_id),
            current_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_edges=int(getattr(args, "f2_downstream_replay_guard_lookahead_edges", 3)),
            min_halt_n=int(getattr(args, "f2_downstream_replay_guard_min_halt_n", 2)),
            max_mean_speed_mps=float(getattr(args, "f2_downstream_replay_guard_max_mean_speed_mps", 2.0)),
            min_veh_n=int(getattr(args, "f2_downstream_replay_guard_min_veh_n", 3)),
            max_occupancy_pct=float(getattr(args, "f2_downstream_replay_guard_max_occupancy_pct", 35.0)),
            edge_to_tls_map=edge_to_tls,
            stop_at_non_tls=bool(_plain_f2_tls_bounded_downstream_scan()),
        )
        diag.update(
            {
                "blocked": bool(guard.get("blocked", False)),
                "reason": str(guard.get("reason", "clear")),
                "route_index": int(guard.get("route_index", -1)),
                "route_len": int(guard.get("route_len", 0)),
                "route_progress_frac": float(guard.get("route_progress_frac", -1.0)),
                "worst_edge": str(guard.get("worst_edge", "")),
                "worst_edge_offset": int(guard.get("worst_edge_offset", -1)),
                "lookahead_edges": list(guard.get("lookahead_edges", []) or []),
                "lookahead_edges_n": int(guard.get("lookahead_edges_n", 0)),
                "scan_scope": str(guard.get("scan_scope", "")),
                "scan_limited_by_non_tls": bool(guard.get("scan_limited_by_non_tls", False)),
                "non_tls_boundary_edge": str(guard.get("non_tls_boundary_edge", "")),
                "max_halt_n": int(guard.get("max_halt_n", 0)),
                "max_veh_n": int(guard.get("max_veh_n", 0)),
                "max_occupancy_pct": float(guard.get("max_occupancy_pct", 0.0)),
                "min_mean_speed_mps": float(guard.get("min_mean_speed_mps", -1.0)),
            }
        )
        route_progress = float(diag.get("route_progress_frac", -1.0))
        min_progress = float(getattr(args, "f2_downstream_replay_guard_min_route_progress_frac", 0.58) or 0.58)
        max_progress = float(getattr(args, "f2_downstream_replay_guard_max_route_progress_frac", 0.70) or 0.70)
        max_worst_offset = max(
            1,
            int(getattr(args, "f2_downstream_replay_guard_max_worst_edge_offset", 2) or 2),
        )
        worst_offset = int(diag.get("worst_edge_offset", -1))
        max_halt_n = int(diag.get("max_halt_n", 0) or 0)
        min_mean_speed_mps = float(diag.get("min_mean_speed_mps", 999.0))
        offset1_min_halt = int(getattr(args, "f2_downstream_replay_guard_offset1_min_halt_n", 4) or 4)
        offset1_max_speed = float(
            getattr(args, "f2_downstream_replay_guard_offset1_max_mean_speed_mps", 1.0) or 1.0
        )
        progress_known = route_progress >= 0.0
        progress_in_window = (
            (not progress_known)
            or (route_progress >= float(min_progress) and route_progress <= float(max_progress))
        )
        offset_known = int(worst_offset) > 0
        offset_in_window = (not offset_known) or int(worst_offset) <= int(max_worst_offset)
        offset1_severe = (
            int(worst_offset) != 1
            or (max_halt_n >= int(offset1_min_halt) and min_mean_speed_mps <= float(offset1_max_speed))
        )
        diag["min_route_progress_frac"] = float(min_progress)
        diag["max_route_progress_frac"] = float(max_progress)
        diag["route_progress_in_window"] = bool(progress_in_window)
        diag["max_worst_edge_offset"] = int(max_worst_offset)
        diag["worst_edge_offset_in_window"] = bool(offset_in_window)
        diag["offset1_min_halt_n"] = int(offset1_min_halt)
        diag["offset1_max_mean_speed_mps"] = float(offset1_max_speed)
        diag["offset1_severe_enough"] = bool(offset1_severe)
        if bool(guard.get("blocked", False)) and not bool(progress_in_window):
            diag["skip_replay"] = False
            diag["reason"] = f"outside_route_progress_window:{str(guard.get('reason', 'blocked'))}"
        elif bool(guard.get("blocked", False)) and not bool(offset_in_window):
            diag["skip_replay"] = False
            diag["reason"] = f"outside_worst_edge_offset_window:{str(guard.get('reason', 'blocked'))}"
        elif bool(guard.get("blocked", False)) and not bool(offset1_severe):
            diag["skip_replay"] = False
            diag["reason"] = f"offset1_not_severe:{str(guard.get('reason', 'blocked'))}"
        else:
            diag["skip_replay"] = bool(guard.get("blocked", False))
        if bool(diag["skip_replay"]):
            la = dict(lookahead_diag or {})
            _fed_evt_main(
                "f2.downstream_replay_guard.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                route_lookahead=bool(la.get("route_lookahead", False)),
                lookahead_hops=int(la.get("lookahead_hops", 0) or 0),
                route_distance_to_selected_edge_m=la.get("route_distance_to_selected_edge_m"),
                upstream_stopped=bool(la.get("upstream_stopped", False)),
                lookahead_action=str(la.get("action", "")),
                lookahead_reason=str(la.get("reason", "")),
                **diag,
            )
        return diag

    def _f2_downstream_apply_guard_diag(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        plan: object = None,
        lookahead_diag: Optional[Dict[str, object]] = None,
        source: str = "",
        plan_type_override: str = "",
        offer_action: str = "",
    ) -> Dict[str, object]:
        """Fail closed when F2 would actuate into a blocked downstream corridor.

        Release/replay guards cover specific rescue paths. This guard is broader:
        any active F2 actuation that cannot be consumed downstream is suppressed so
        federation degrades toward fixed timing instead of amplifying spillback.
        """
        enabled = bool(getattr(args, "f2_downstream_apply_guard_enable", True))
        plan_type = str(plan_type_override or getattr(plan, "plan_type", "") or "") if plan is not None or plan_type_override else ""
        diag: Dict[str, object] = {
            "enabled": bool(enabled),
            "blocked": False,
            "skip_apply": False,
            "reason": "disabled" if not enabled else "clear",
            "stage": str(stage),
            "source": str(source),
            "ev_edge": str(ev_edge or ""),
            "selected_in_edge": str(selected_in_edge or ""),
            "plan_type": str(plan_type),
            "offer_action": str(offer_action or ""),
        }
        if not enabled:
            return diag
        if plan_type in ("", "none", "restore"):
            diag["reason"] = "weak_plan"
            return diag

        guard = b1_downstream_blockage_diag(
            ev_id=str(ev_id),
            current_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_edges=int(getattr(args, "f2_downstream_apply_guard_lookahead_edges", 8)),
            min_halt_n=int(getattr(args, "f2_downstream_apply_guard_min_halt_n", 2)),
            max_mean_speed_mps=float(getattr(args, "f2_downstream_apply_guard_max_mean_speed_mps", 2.0)),
            min_veh_n=int(getattr(args, "f2_downstream_apply_guard_min_veh_n", 3)),
            max_occupancy_pct=float(getattr(args, "f2_downstream_apply_guard_max_occupancy_pct", 35.0)),
            edge_to_tls_map=edge_to_tls,
            stop_at_non_tls=bool(_plain_f2_tls_bounded_downstream_scan()),
        )
        guard = _merge_external_downstream_context(
            guard,
            tls_id=str(tls_id),
            ev_id=str(ev_id),
            sim_time=float(sim_time),
            max_age_sec=float(getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0),
        )
        guard = _annotate_immediate_blockage_diag(guard)
        diag.update(
            {
                "blocked": bool(guard.get("blocked", False)),
                "reason": str(guard.get("reason", "clear")),
                "route_index": int(guard.get("route_index", -1)),
                "route_len": int(guard.get("route_len", 0)),
                "route_progress_frac": float(guard.get("route_progress_frac", -1.0)),
                "worst_edge": str(guard.get("worst_edge", "")),
                "worst_edge_offset": int(guard.get("worst_edge_offset", -1)),
                "lookahead_edges": list(guard.get("lookahead_edges", []) or []),
                "lookahead_edges_n": int(guard.get("lookahead_edges_n", 0) or 0),
                "scan_scope": str(guard.get("scan_scope", "")),
                "scan_limited_by_non_tls": bool(guard.get("scan_limited_by_non_tls", False)),
                "non_tls_boundary_edge": str(guard.get("non_tls_boundary_edge", "")),
                "max_halt_n": int(guard.get("max_halt_n", 0) or 0),
                "max_veh_n": int(guard.get("max_veh_n", 0) or 0),
                "max_occupancy_pct": float(guard.get("max_occupancy_pct", 0.0) or 0.0),
                "min_mean_speed_mps": float(guard.get("min_mean_speed_mps", -1.0) or -1.0),
                "immediate_blockage_guard_enabled": bool(
                    guard.get("immediate_blockage_guard_enabled", False)
                ),
                "immediate_blockage_severe": bool(guard.get("immediate_blockage_severe", False)),
                "immediate_blockage_max_worst_edge_offset": int(
                    guard.get("immediate_blockage_max_worst_edge_offset", 1) or 1
                ),
                "immediate_blockage_min_halt_n": int(
                    guard.get("immediate_blockage_min_halt_n", 3) or 3
                ),
                "immediate_blockage_min_veh_n": int(
                    guard.get("immediate_blockage_min_veh_n", 6) or 6
                ),
                "immediate_blockage_max_mean_speed_mps": float(
                    guard.get("immediate_blockage_max_mean_speed_mps", 0.5) or 0.5
                ),
                "external_downstream_context_used": bool(
                    guard.get("external_downstream_context_used", False)
                ),
                "external_downstream_context_provider": str(
                    guard.get("external_downstream_context_provider", "")
                ),
                "external_downstream_context_request_id": str(
                    guard.get("external_downstream_context_request_id", "")
                ),
                "external_downstream_context_age_sec": float(
                    guard.get("external_downstream_context_age_sec", -1.0) or -1.0
                ),
            }
        )
        f2_max_worst_offset = max(
            1,
            int(getattr(args, "f2_downstream_apply_guard_max_worst_edge_offset", 3) or 3),
        )
        max_worst_offset = int(f2_max_worst_offset)
        passive_nearfield_guard = bool(
            _is_passive_dt_mode(CURRENT_EVALUATION)
            and str(guard.get("scan_scope", "")) == "route_lookahead"
            and not bool(guard.get("external_downstream_context_used", False))
        )
        worst_offset = int(diag.get("worst_edge_offset", -1) or -1)
        worst_edge = str(diag.get("worst_edge", "") or "")
        worst_edge_has_active_tls = bool(worst_edge and str(edge_to_tls.get(worst_edge, "") or ""))
        active_tls_metering_floor_max_offset = max(
            1,
            int(getattr(args, "f2p_active_tls_metering_floor_max_worst_edge_offset", 1) or 1),
        )
        active_tls_metering_floor = bool(
            passive_nearfield_guard
            and bool(getattr(args, "f2p_active_tls_metering_floor_enable", True))
            and bool(worst_edge_has_active_tls)
            and int(worst_offset) > 0
            and int(worst_offset) <= int(active_tls_metering_floor_max_offset)
        )
        if passive_nearfield_guard:
            # F2P adds passive non-TLS observability. Treat that extra visibility
            # as a near-field safety veto, not a broad corridor veto; otherwise
            # F2P can become slower than F2 simply because it sees farther.
            #
            # Exception: if the immediate blocked edge is controlled by an
            # active TLS, keep F2's metering floor. Farther active-TLS blockages
            # should not make F2P more conservative than F2; passive context is
            # useful there as advisory lookahead rather than a broad veto.
            if not bool(active_tls_metering_floor):
                max_worst_offset = min(
                    int(max_worst_offset),
                    max(1, int(getattr(args, "f2p_passive_context_max_worst_edge_offset", 1) or 1)),
                )
        offset_known = int(worst_offset) > 0
        offset_in_window = (not offset_known) or int(worst_offset) <= int(max_worst_offset)
        diag["max_worst_edge_offset"] = int(max_worst_offset)
        diag["f2_max_worst_edge_offset"] = int(f2_max_worst_offset)
        diag["worst_edge_offset_in_window"] = bool(offset_in_window)
        diag["passive_nearfield_guard"] = bool(passive_nearfield_guard)
        diag["passive_nearfield_guard_reason"] = (
            "passive_route_lookahead_nearfield_only" if passive_nearfield_guard else ""
        )
        diag["f2p_active_tls_metering_floor"] = bool(active_tls_metering_floor)
        diag["f2p_active_tls_metering_floor_enabled"] = bool(
            getattr(args, "f2p_active_tls_metering_floor_enable", True)
        )
        diag["f2p_active_tls_metering_floor_max_worst_edge_offset"] = int(
            active_tls_metering_floor_max_offset
        )
        diag["f2p_active_tls_metering_floor_reason"] = (
            "active_tls_worst_edge_inside_f2p_metering_floor_window" if active_tls_metering_floor else ""
        )
        diag["worst_edge_has_active_tls"] = bool(worst_edge_has_active_tls)
        if (
            bool(guard.get("blocked", False))
            and bool(guard.get("immediate_blockage_severe", False))
            and bool(offset_in_window)
        ):
            diag["skip_apply"] = True
            diag["reason"] = f"immediate_downstream_blockage:{str(guard.get('reason', 'blocked'))}"
        elif bool(guard.get("blocked", False)) and not bool(offset_in_window):
            diag["skip_apply"] = False
            diag["reason"] = f"outside_worst_edge_offset_window:{str(guard.get('reason', 'blocked'))}"
        else:
            diag["skip_apply"] = bool(guard.get("blocked", False))

        stall_rescue_enabled = bool(getattr(args, "f2p_passive_stall_rescue_enable", True))
        stall_rescue_min_blocked_sec = max(
            0.0,
            float(getattr(args, "f2p_passive_stall_rescue_min_blocked_sec", 6.0) or 6.0),
        )
        stall_rescue_max_speed_mps = max(
            0.0,
            float(getattr(args, "f2p_passive_stall_rescue_max_speed_mps", 0.5) or 0.5),
        )
        stall_rescue_require_selected = bool(
            getattr(args, "f2p_passive_stall_rescue_require_selected_edge", True)
        )
        stall_key = (str(ev_id), str(tls_id))
        stall_signature = (
            str(diag.get("worst_edge", "")),
            str(diag.get("reason", "")),
            str(selected_in_edge or ""),
            str(plan_type),
        )
        ev_speed = -1.0
        try:
            ev_speed = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            ev_speed = -1.0
        on_selected_in_edge = bool(str(ev_edge or "") and str(ev_edge or "") == str(selected_in_edge or ""))
        stall_rescue_candidate = bool(
            stall_rescue_enabled
            and passive_nearfield_guard
            and bool(diag.get("skip_apply", False))
            and bool(diag.get("immediate_blockage_severe", False))
            and (ev_speed < 0.0 or ev_speed <= float(stall_rescue_max_speed_mps))
            and ((not stall_rescue_require_selected) or on_selected_in_edge)
        )
        prev_stall = dict(_f2p_passive_stall_rescue_state.get(stall_key, {}) or {})
        prev_signature = tuple(prev_stall.get("signature", ()) or ())
        prev_last = float(prev_stall.get("last_sim_time", -999999.0) or -999999.0)
        continuity_gap = max(1.0, float(stall_rescue_min_blocked_sec) * 1.5)
        if stall_rescue_candidate:
            if prev_signature == stall_signature and float(sim_time) - float(prev_last) <= float(continuity_gap):
                first_blocked_time = float(prev_stall.get("first_blocked_time", sim_time) or sim_time)
                repeat_count = int(prev_stall.get("repeat_count", 0) or 0) + 1
            else:
                first_blocked_time = float(sim_time)
                repeat_count = 1
            blocked_duration_sec = max(0.0, float(sim_time) - float(first_blocked_time))
            _f2p_passive_stall_rescue_state[stall_key] = {
                "signature": tuple(stall_signature),
                "first_blocked_time": float(first_blocked_time),
                "last_sim_time": float(sim_time),
                "repeat_count": int(repeat_count),
                "last_reason": str(diag.get("reason", "")),
                "last_worst_edge": str(diag.get("worst_edge", "")),
            }
            diag["passive_stall_rescue_enabled"] = bool(stall_rescue_enabled)
            diag["passive_stall_rescue_candidate"] = True
            diag["passive_stall_rescue_blocked_duration_sec"] = float(blocked_duration_sec)
            diag["passive_stall_rescue_repeat_count"] = int(repeat_count)
            diag["passive_stall_rescue_min_blocked_sec"] = float(stall_rescue_min_blocked_sec)
            diag["passive_stall_rescue_ev_speed_mps"] = float(ev_speed)
            diag["passive_stall_rescue_on_selected_edge"] = bool(on_selected_in_edge)
            diag["passive_stall_rescue_applied"] = False
            if float(blocked_duration_sec) >= float(stall_rescue_min_blocked_sec):
                original_reason = str(diag.get("reason", "blocked"))
                diag["skip_apply"] = False
                diag["reason"] = f"passive_stall_rescue:{original_reason}"
                diag["passive_stall_rescue_applied"] = True
                _fed_evt_main(
                    "f2p.passive_stall_rescue.apply_allow",
                    role="intersection",
                    ev_id=str(ev_id),
                    tls_id=str(tls_id),
                    sim_time=float(sim_time),
                    reason=str(diag.get("reason", "")),
                    original_reason=str(original_reason),
                    worst_edge=str(diag.get("worst_edge", "")),
                    worst_edge_offset=int(diag.get("worst_edge_offset", -1) or -1),
                    max_worst_edge_offset=int(diag.get("max_worst_edge_offset", 1) or 1),
                    blocked_duration_sec=float(blocked_duration_sec),
                    repeat_count=int(repeat_count),
                    ev_speed_mps=float(ev_speed),
                    ev_edge=str(ev_edge or ""),
                    selected_in_edge=str(selected_in_edge or ""),
                    on_selected_in_edge=bool(on_selected_in_edge),
                    scan_scope=str(diag.get("scan_scope", "")),
                    lookahead_edges=list(diag.get("lookahead_edges", []) or []),
                    immediate_blockage_severe=bool(diag.get("immediate_blockage_severe", False)),
                    selected_action="allow_current_apply",
                    decision_source="f2p_passive_stall_rescue",
                    stage=str(stage),
                    source=str(source),
                    plan_type=str(plan_type),
                    offer_action=str(offer_action or ""),
                )
        else:
            if prev_stall:
                _f2p_passive_stall_rescue_state.pop(stall_key, None)
            diag["passive_stall_rescue_enabled"] = bool(stall_rescue_enabled)
            diag["passive_stall_rescue_candidate"] = False
            diag["passive_stall_rescue_applied"] = False
            diag["passive_stall_rescue_ev_speed_mps"] = float(ev_speed)
            diag["passive_stall_rescue_on_selected_edge"] = bool(on_selected_in_edge)

        if (
            passive_nearfield_guard
            and bool(guard.get("blocked", False))
            and not bool(diag.get("skip_apply", False))
            and not bool(offset_in_window)
        ):
            _fed_evt_main(
                "f2p.passive_nearfield_guard.release",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(diag.get("reason", "")),
                worst_edge=str(diag.get("worst_edge", "")),
                worst_edge_offset=int(diag.get("worst_edge_offset", -1) or -1),
                max_worst_edge_offset=int(diag.get("max_worst_edge_offset", 1) or 1),
                f2_max_worst_edge_offset=int(diag.get("f2_max_worst_edge_offset", 3) or 3),
                scan_scope=str(diag.get("scan_scope", "")),
                lookahead_edges=list(diag.get("lookahead_edges", []) or []),
                blocked=bool(diag.get("blocked", False)),
                immediate_blockage_severe=bool(diag.get("immediate_blockage_severe", False)),
                worst_edge_has_active_tls=bool(diag.get("worst_edge_has_active_tls", False)),
                f2p_active_tls_metering_floor=bool(diag.get("f2p_active_tls_metering_floor", False)),
                selected_action="allow_current_apply",
                decision_source="f2p_passive_nearfield_guard",
                stage=str(stage),
                source=str(source),
                plan_type=str(plan_type),
                offer_action=str(offer_action or ""),
            )
        if bool(diag["skip_apply"]):
            la = dict(lookahead_diag or {})
            _fed_evt_main(
                "f2.downstream_apply_guard.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                route_lookahead=bool(la.get("route_lookahead", False)),
                lookahead_hops=int(la.get("lookahead_hops", 0) or 0),
                route_distance_to_selected_edge_m=la.get("route_distance_to_selected_edge_m"),
                upstream_stopped=bool(la.get("upstream_stopped", False)),
                lookahead_action=str(la.get("action", "")),
                lookahead_reason=str(la.get("reason", "")),
                **diag,
            )
            if bool(diag.get("external_downstream_context_used", False)):
                _fed_evt_main(
                    "f2.downstream_apply_guard.drone_skip",
                    role="intersection",
                    sim_time=float(sim_time),
                    request_id=str(diag.get("external_downstream_context_request_id", "")),
                    ev_id=str(ev_id),
                    requester_tls=str(tls_id),
                    tls_id=str(tls_id),
                    provider_id=str(diag.get("external_downstream_context_provider", "")),
                    provider_type="drone",
                    target_edges=list(diag.get("lookahead_edges", []) or []),
                    decision_deadline_sec=float(
                        getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0
                    ),
                    request_latency_ms=-1.0,
                    response_latency_ms=-1.0,
                    context_age_ms=float(diag.get("external_downstream_context_age_sec", -1.0) or -1.0) * 1000.0,
                    blocked=bool(diag.get("blocked", False)),
                    reason=str(diag.get("reason", "")),
                    worst_edge=str(diag.get("worst_edge", "")),
                    worst_edge_offset=int(diag.get("worst_edge_offset", -1) or -1),
                    confidence=-1.0,
                    selected_action="skip_f2_apply",
                    decision_source="f2_downstream_apply_guard",
                    stage=str(stage),
                    source=str(source),
                    plan_type=str(plan_type),
                    offer_action=str(offer_action or ""),
                )
        return diag

    def _f2_blocked_local_reason(final_reason: str, stage: str) -> bool:
        text = f"{str(final_reason or '')} {str(stage or '')}".lower()
        return any(
            token in text
            for token in (
                "blocked",
                "infeasible",
                "no_local",
                "no local",
                "selected_none",
                "target_pending",
            )
        )

    def _maybe_f2_approach_phase_rescue_plan(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        ev_edge: str,
        selected_in_edge: str,
        d_stop: float,
        plan: object,
        tls_before: Dict[str, object],
        lookahead_diag: Dict[str, object],
        f2_meta: Optional[Dict[str, object]] = None,
    ) -> Tuple[object, Dict[str, object]]:
        """Convert a blocked F2 local fallback into a bounded direct target-phase rescue.

        This is deliberately narrower than the generic late-rescue path: it only acts while
        the EV is on the selected TLS inbound edge and the target phase is still non-green.
        """
        diag: Dict[str, object] = {
            "enabled": bool(getattr(args, "f2_approach_phase_rescue_enable", True)),
            "applied": False,
            "reason": "not_evaluated",
        }
        if not bool(diag["enabled"]):
            diag["reason"] = "disabled"
            return plan, diag
        plan_type = str(getattr(plan, "plan_type", "") or "")
        if plan is None or plan_type in ("", "none", "restore"):
            diag["reason"] = "weak_plan"
            diag["plan_type"] = str(plan_type)
            return plan, diag
        if plan_type == "intrusive":
            diag["reason"] = "already_intrusive"
            diag["plan_type"] = str(plan_type)
            return plan, diag

        meta = dict(f2_meta or {})
        final_reason = str(meta.get("final_reason", "") or "")
        if bool(getattr(args, "f2_approach_phase_rescue_blocked_only", True)) and not _f2_blocked_local_reason(
            final_reason,
            str(stage),
        ):
            diag["reason"] = "not_blocked_local_reason"
            diag["final_reason"] = str(final_reason)
            return plan, diag

        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        if target_phase < 0:
            diag["reason"] = "missing_target_phase"
            return plan, diag

        target_green = _target_is_green_for_diag(tls_before, int(target_phase))
        before_phase = int(tls_before.get("phase", -1))
        if target_green or before_phase == int(target_phase):
            diag["reason"] = "target_green_or_current"
            diag["target_phase"] = int(target_phase)
            diag["before_phase"] = int(before_phase)
            return plan, diag

        route_dist = _f2_target_pending_route_distance(lookahead_diag, float(d_stop))
        max_dist = max(0.0, float(getattr(args, "f2_approach_phase_rescue_max_distance_m", 120.0) or 120.0))
        if route_dist < 0.0 or route_dist > float(max_dist):
            diag["reason"] = "distance_out_of_range"
            diag["route_distance_to_selected_edge_m"] = float(route_dist)
            diag["max_distance_m"] = float(max_dist)
            return plan, diag

        try:
            speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed_f = -1.0
        max_speed = max(0.0, float(getattr(args, "f2_approach_phase_rescue_max_speed_mps", 14.5) or 14.5))
        if speed_f > float(max_speed):
            diag["reason"] = "speed_too_high"
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return plan, diag

        ev_edge_s = str(ev_edge or "")
        selected_s = str(selected_in_edge or "")
        on_selected = bool(lookahead_diag.get("on_selected_in_edge", False)) or bool(selected_s and ev_edge_s == selected_s)
        if bool(getattr(args, "f2_approach_phase_rescue_require_current_edge", True)) and not on_selected:
            diag["reason"] = "not_on_selected_in_edge"
            diag["ev_edge"] = str(ev_edge_s)
            diag["selected_in_edge"] = str(selected_s)
            return plan, diag

        key = (str(ev_id), str(tls_id))
        min_interval = max(
            0.0,
            float(getattr(args, "f2_approach_phase_rescue_min_interval_sec", 4.0) or 4.0),
        )
        prev = dict(_f2_approach_phase_rescue_state.get(key, {}) or {})
        dt = float(sim_time) - float(prev.get("last_apply_time", -1e9))
        if (
            prev
            and int(prev.get("target_phase", -999)) == int(target_phase)
            and dt >= 0.0
            and dt < float(min_interval)
        ):
            diag["reason"] = "recent_rescue"
            diag["dt_since_last_s"] = float(dt)
            diag["min_interval_s"] = float(min_interval)
            diag["target_phase"] = int(target_phase)
            return plan, diag

        notes = str(getattr(plan, "notes", "") or "")
        if notes:
            notes = f"{notes}; "
        notes = f"{notes}f2_approach_phase_rescue:{str(stage)}:{final_reason or 'target_not_green'}"
        rescue_plan = intersection_agent_module.PreemptionPlan(
            plan_type="intrusive",
            target_phase_idx=int(target_phase),
            jump_to_phase_idx=int(target_phase),
            jump_time_sec=float(sim_time),
            planned_green_window=getattr(plan, "planned_green_window", None),
            notes=notes,
        )
        _f2_approach_phase_rescue_state[key] = {
            "last_apply_time": float(sim_time),
            "target_phase": int(target_phase),
            "stage": str(stage),
            "route_distance_to_selected_edge_m": float(route_dist),
            "speed_mps": float(speed_f),
        }
        if key not in _late_rescue_state:
            _late_rescue_state[key] = {
                "start_time": float(sim_time),
                "start_distance_m": float(route_dist),
                "start_speed_mps": float(speed_f),
                "start_phase": int(before_phase),
                "target_phase": int(target_phase),
                "started_by": f"approach_phase_rescue:{str(stage)}:{final_reason or 'target_not_green'}",
            }
            rescue_started = True
        else:
            rescue_started = False
        diag.update(
            {
                "applied": True,
                "reason": "approach_target_not_green",
                "original_plan_type": str(plan_type),
                "plan_type": "intrusive",
                "target_phase": int(target_phase),
                "before_phase": int(before_phase),
                "target_green_before": bool(target_green),
                "route_distance_to_selected_edge_m": float(route_dist),
                "max_distance_m": float(max_dist),
                "speed_mps": float(speed_f),
                "max_speed_mps": float(max_speed),
                "on_selected_in_edge": bool(on_selected),
                "final_reason": str(final_reason),
                "min_interval_s": float(min_interval),
                "rescue_started": bool(rescue_started),
            }
        )
        _fed_evt_main(
            "f2.approach_phase_rescue.plan",
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            stage=str(stage),
            decision_source="f2_approach_phase_rescue",
            reason=str(diag["reason"]),
            final_reason=str(final_reason),
            original_plan_type=str(plan_type),
            rescue_plan_type="intrusive",
            target_phase=int(target_phase),
            before_phase=int(before_phase),
            before_state=str(tls_before.get("state", "")),
            before_next_switch=float(tls_before.get("next_switch", -1.0)),
            route_distance_to_selected_edge_m=float(route_dist),
            distance_to_stopline_m=float(d_stop),
            speed_mps=float(speed_f),
            max_distance_m=float(max_dist),
            max_speed_mps=float(max_speed),
            on_selected_in_edge=bool(on_selected),
            rescue_started=bool(rescue_started),
        )
        return rescue_plan, diag

    def _should_skip_f2_selected_none_slow_ev(
        *,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        plan: object,
        tls_before: Dict[str, object],
        selected_in_edge: str,
        d_stop: float,
        f2_meta: Optional[Dict[str, object]],
    ) -> Tuple[bool, Dict[str, object]]:
        """Prevent no-offer F2 fallback from repeatedly actuating into a slow/blocked approach."""
        diag: Dict[str, object] = {
            "enabled": bool(getattr(args, "f2_selected_none_slow_ev_guard_enable", True)),
            "reason": "not_evaluated",
        }
        if not bool(diag["enabled"]):
            diag["reason"] = "disabled"
            return False, diag

        plan_type = str(getattr(plan, "plan_type", "") or "")
        guarded_types = {
            str(x).strip()
            for x in str(getattr(args, "f2_selected_none_slow_ev_guard_plan_types", "") or "").split(",")
            if str(x).strip()
        }
        if plan_type not in guarded_types:
            diag["reason"] = "plan_type_not_guarded"
            diag["plan_type"] = str(plan_type)
            diag["guarded_plan_types"] = sorted(guarded_types)
            return False, diag

        meta = dict(f2_meta or {})
        final_reason = str(meta.get("final_reason", "") or "")
        if final_reason and not _f2_blocked_local_reason(final_reason, "selected_none_continuity"):
            diag["reason"] = "not_blocked_local_reason"
            diag["final_reason"] = str(final_reason)
            return False, diag

        try:
            speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed_f = -1.0
        max_speed = max(
            0.0,
            float(getattr(args, "f2_selected_none_slow_ev_guard_max_speed_mps", 3.0) or 3.0),
        )
        if speed_f < 0.0 or speed_f > float(max_speed):
            diag["reason"] = "speed_not_slow"
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        min_distance = max(
            0.0,
            float(getattr(args, "f2_selected_none_slow_ev_guard_min_distance_m", 40.0) or 40.0),
        )
        if float(d_stop) < float(min_distance):
            diag["reason"] = "distance_below_guard_min"
            diag["distance_to_stopline_m"] = float(d_stop)
            diag["min_distance_m"] = float(min_distance)
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        target_green = _target_is_green_for_diag(tls_before, int(target_phase)) if target_phase >= 0 else False
        if not bool(target_green):
            diag["reason"] = "target_not_green"
            diag["target_phase"] = int(target_phase)
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        diag.update(
            {
                "reason": "slow_ev_target_already_green",
                "plan_type": str(plan_type),
                "final_reason": str(final_reason),
                "target_phase": int(target_phase),
                "target_green": bool(target_green),
                "speed_mps": float(speed_f),
                "max_speed_mps": float(max_speed),
                "distance_to_stopline_m": float(d_stop),
                "min_distance_m": float(min_distance),
            }
        )
        _fed_evt_main(
            "f2.selected_none.skip",
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            reason="slow_ev_target_already_green",
            plan_type=str(plan_type),
            final_reason=str(final_reason),
            target_phase=int(target_phase),
            target_green=bool(target_green),
            speed_mps=float(speed_f),
            max_speed_mps=float(max_speed),
            distance_to_stopline_m=float(d_stop),
            min_distance_m=float(min_distance),
            before_phase=int(tls_before.get("phase", -1)),
            before_state=str(tls_before.get("state", "")),
            before_next_switch=float(tls_before.get("next_switch", -1.0)),
            before_next_switch_rem_s=float(tls_before.get("next_switch_rem_s", -1.0)),
        )
        return True, diag

    def _should_skip_f2_fallback_slow_green(
        *,
        stage: str,
        event_type: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        plan: object,
        tls_before: Dict[str, object],
        selected_in_edge: str,
        d_stop: float,
        f2_meta: Optional[Dict[str, object]],
    ) -> Tuple[bool, Dict[str, object]]:
        """Prevent F2 fallback paths from over-actuating when B1-equivalent green is already present."""
        diag: Dict[str, object] = {
            "enabled": bool(getattr(args, "f2_fallback_slow_ev_guard_enable", True)),
            "reason": "not_evaluated",
            "stage": str(stage),
        }
        if not bool(diag["enabled"]):
            diag["reason"] = "disabled"
            return False, diag

        plan_type = str(getattr(plan, "plan_type", "") or "")
        guarded_types = {
            str(x).strip()
            for x in str(getattr(args, "f2_fallback_slow_ev_guard_plan_types", "") or "").split(",")
            if str(x).strip()
        }
        if plan_type not in guarded_types:
            diag["reason"] = "plan_type_not_guarded"
            diag["plan_type"] = str(plan_type)
            diag["guarded_plan_types"] = sorted(guarded_types)
            return False, diag

        final_reason = str((f2_meta or {}).get("final_reason", "") or "")
        try:
            speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed_f = -1.0
        max_speed = max(
            0.0,
            float(getattr(args, "f2_fallback_slow_ev_guard_max_speed_mps", 3.0) or 3.0),
        )
        if speed_f < 0.0 or speed_f > float(max_speed):
            diag["reason"] = "speed_not_slow"
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        min_distance = max(
            0.0,
            float(getattr(args, "f2_fallback_slow_ev_guard_min_distance_m", 40.0) or 40.0),
        )
        if float(d_stop) < float(min_distance):
            diag["reason"] = "distance_below_guard_min"
            diag["distance_to_stopline_m"] = float(d_stop)
            diag["min_distance_m"] = float(min_distance)
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        target_green = _target_is_green_for_diag(tls_before, int(target_phase)) if target_phase >= 0 else False
        if not bool(target_green):
            diag["reason"] = "target_not_green"
            diag["target_phase"] = int(target_phase)
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return False, diag

        diag.update(
            {
                "reason": "slow_ev_target_already_green",
                "plan_type": str(plan_type),
                "final_reason": str(final_reason),
                "target_phase": int(target_phase),
                "target_green": bool(target_green),
                "speed_mps": float(speed_f),
                "max_speed_mps": float(max_speed),
                "distance_to_stopline_m": float(d_stop),
                "min_distance_m": float(min_distance),
            }
        )
        _fed_evt_main(
            str(event_type),
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            reason="slow_ev_target_already_green",
            stage=str(stage),
            plan_type=str(plan_type),
            final_reason=str(final_reason),
            target_phase=int(target_phase),
            target_green=bool(target_green),
            speed_mps=float(speed_f),
            max_speed_mps=float(max_speed),
            distance_to_stopline_m=float(d_stop),
            min_distance_m=float(min_distance),
            before_phase=int(tls_before.get("phase", -1)),
            before_state=str(tls_before.get("state", "")),
            before_next_switch=float(tls_before.get("next_switch", -1.0)),
            before_next_switch_rem_s=float(tls_before.get("next_switch_rem_s", -1.0)),
        )
        return True, diag

    def _should_skip_f2_fallback_cadence(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        plan: object,
        d_stop: Optional[float] = None,
    ) -> Tuple[bool, Dict[str, object]]:
        """Keep F2 fallback fail-soft from applying materially faster than local B1 cadence."""
        diag: Dict[str, object] = {
            "enabled": bool(getattr(args, "f2_fallback_cadence_guard_enable", True)),
            "reason": "not_evaluated",
            "stage": str(stage),
        }
        if not bool(diag["enabled"]):
            diag["reason"] = "disabled"
            return False, diag

        plan_type = str(getattr(plan, "plan_type", "") or "")
        guarded_types = {
            str(x).strip()
            for x in str(getattr(args, "f2_fallback_cadence_plan_types", "") or "").split(",")
            if str(x).strip()
        }
        if plan_type not in guarded_types:
            diag["reason"] = "plan_type_not_guarded"
            diag["plan_type"] = str(plan_type)
            diag["guarded_plan_types"] = sorted(guarded_types)
            return False, diag

        min_distance = max(
            0.0,
            float(getattr(args, "f2_fallback_cadence_min_distance_m", 40.0) or 40.0),
        )
        if d_stop is not None:
            try:
                d_stop_f = float(d_stop)
            except Exception:
                d_stop_f = -1.0
            if d_stop_f >= 0.0 and d_stop_f < float(min_distance):
                diag["reason"] = "distance_below_guard_min"
                diag["distance_to_stopline_m"] = float(d_stop_f)
                diag["min_distance_m"] = float(min_distance)
                diag["plan_type"] = str(plan_type)
                return False, diag

        min_interval = max(
            0.0,
            float(getattr(args, "f2_fallback_cadence_min_interval_sec", 1.0) or 1.0),
        )
        key = (str(ev_id), str(tls_id), str(stage))
        prev = dict(_f2_fallback_cadence_state.get(key, {}) or {})
        dt = float(sim_time) - float(prev.get("last_apply_time", -1e9))
        diag.update(
            {
                "plan_type": str(plan_type),
                "min_interval_s": float(min_interval),
                "min_distance_m": float(min_distance),
                "distance_to_stopline_m": None if d_stop is None else d_stop,
                "dt_since_last_s": float(dt) if prev else None,
                "previous_apply_time": prev.get("last_apply_time") if prev else None,
                "suppressed_count": int(prev.get("suppressed_count", 0) or 0),
            }
        )
        if prev and dt >= 0.0 and dt < float(min_interval):
            prev["suppressed_count"] = int(prev.get("suppressed_count", 0) or 0) + 1
            prev["last_suppressed_time"] = float(sim_time)
            _f2_fallback_cadence_state[key] = prev
            diag["reason"] = "cadence_guard"
            diag["suppressed_count"] = int(prev["suppressed_count"])
            return True, diag

        _f2_fallback_cadence_state[key] = {
            "last_apply_time": float(sim_time),
            "last_suppressed_time": None,
            "suppressed_count": 0,
            "plan_type": str(plan_type),
        }
        diag["reason"] = "allowed"
        return False, diag

    def _maybe_f2_current_tls_stopped_rescue_plan(
        *,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        ev_edge: str,
        selected_in_edge: str,
        d_stop: float,
        plan: object,
        tls_before: Dict[str, object],
        lookahead_diag: Dict[str, object],
        f2_meta: Optional[Dict[str, object]] = None,
    ) -> Tuple[object, Dict[str, object]]:
        """Recover only when F2's lookahead guard blocks a near-stopped active TLS plan."""
        diag: Dict[str, object] = {
            "enabled": bool(getattr(args, "f2_current_tls_stopped_rescue_enable", True)),
            "applied": False,
            "reason": "not_evaluated",
        }
        if not bool(diag["enabled"]):
            diag["reason"] = "disabled"
            return plan, diag

        plan_type = str(getattr(plan, "plan_type", "") or "") if plan is not None else ""
        if plan is None or plan_type in ("", "none", "restore"):
            diag["reason"] = "weak_plan"
            diag["plan_type"] = str(plan_type)
            return plan, diag

        guard_reason = str(lookahead_diag.get("reason", "") or "")
        eligible_guard = guard_reason in {
            "upstream_stopped_before_selected_tls",
            "lookahead_intrusive_not_preemption_eligible",
        }
        if not eligible_guard:
            diag["reason"] = "guard_reason_not_eligible"
            diag["guard_reason"] = str(guard_reason)
            return plan, diag

        try:
            hops = int(lookahead_diag.get("lookahead_hops", 0) or 0)
        except Exception:
            hops = 0
        max_hops = max(
            0,
            int(getattr(args, "f2_current_tls_stopped_rescue_max_lookahead_hops", 2) or 2),
        )
        if hops > int(max_hops):
            diag["reason"] = "lookahead_hops_out_of_range"
            diag["lookahead_hops"] = int(hops)
            diag["max_lookahead_hops"] = int(max_hops)
            return plan, diag

        max_dist = max(
            0.0,
            float(getattr(args, "f2_current_tls_stopped_rescue_max_distance_m", 80.0) or 80.0),
        )
        d_stop_f = float(d_stop)
        if d_stop_f < 0.0 or d_stop_f > float(max_dist):
            diag["reason"] = "distance_out_of_range"
            diag["distance_to_stopline_m"] = float(d_stop_f)
            diag["max_distance_m"] = float(max_dist)
            return plan, diag

        try:
            speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed_f = -1.0
        max_speed = max(
            0.0,
            float(getattr(args, "f2_current_tls_stopped_rescue_max_speed_mps", 2.0) or 2.0),
        )
        if speed_f < 0.0 or speed_f > float(max_speed):
            diag["reason"] = "speed_out_of_range"
            diag["speed_mps"] = float(speed_f)
            diag["max_speed_mps"] = float(max_speed)
            return plan, diag

        try:
            target_phase = int(getattr(plan, "target_phase_idx", -1))
        except Exception:
            target_phase = -1
        if target_phase < 0:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        if target_phase < 0:
            diag["reason"] = "missing_target_phase"
            return plan, diag

        target_green = _target_is_green_for_diag(tls_before, int(target_phase))
        before_phase = int(tls_before.get("phase", -1))
        if target_green or before_phase == int(target_phase):
            diag["reason"] = "target_green_or_current"
            diag["target_phase"] = int(target_phase)
            diag["before_phase"] = int(before_phase)
            return plan, diag

        downstream_release_guard = _f2_downstream_release_guard_diag(
            stage=str(stage),
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            plan=plan,
            lookahead_diag=lookahead_diag,
        )
        if bool(downstream_release_guard.get("skip_release", False)):
            diag.update(
                {
                    "reason": "downstream_release_guard",
                    "guard_reason": str(guard_reason),
                    "target_phase": int(target_phase),
                    "before_phase": int(before_phase),
                    "distance_to_stopline_m": float(d_stop_f),
                    "speed_mps": float(speed_f),
                    "downstream_release_guard": dict(downstream_release_guard),
                }
            )
            return plan, diag

        key = (str(ev_id), str(tls_id))
        min_interval = max(
            0.0,
            float(getattr(args, "f2_current_tls_stopped_rescue_min_interval_sec", 6.0) or 6.0),
        )
        prev = dict(_f2_approach_phase_rescue_state.get(key, {}) or {})
        dt = float(sim_time) - float(prev.get("last_apply_time", -1e9))
        if (
            prev
            and int(prev.get("target_phase", -999)) == int(target_phase)
            and dt >= 0.0
            and dt < float(min_interval)
        ):
            diag["reason"] = "recent_rescue"
            diag["dt_since_last_s"] = float(dt)
            diag["min_interval_s"] = float(min_interval)
            diag["target_phase"] = int(target_phase)
            return plan, diag

        final_reason = str((f2_meta or {}).get("final_reason", "") or "")
        notes = str(getattr(plan, "notes", "") or "")
        if notes:
            notes = f"{notes}; "
        rescue_reason = (
            "current_tls_stopped_intrusive_reallow"
            if plan_type == "intrusive"
            else "current_tls_stopped_target_not_green"
        )
        notes = f"{notes}f2_current_tls_stopped_rescue:{str(stage)}:{guard_reason}:{plan_type or 'plan'}"
        if plan_type == "intrusive":
            rescue_plan = intersection_agent_module.PreemptionPlan(
                plan_type=str(getattr(plan, "plan_type", "") or "intrusive"),
                target_phase_idx=int(target_phase),
                extend_green_sec=float(getattr(plan, "extend_green_sec", 0.0) or 0.0),
                hurry_current_phase_to_sec=getattr(plan, "hurry_current_phase_to_sec", None),
                jump_time_sec=getattr(plan, "jump_time_sec", None),
                jump_to_phase_idx=getattr(plan, "jump_to_phase_idx", None),
                planned_green_window=getattr(plan, "planned_green_window", None),
                phase_duration_overrides=getattr(plan, "phase_duration_overrides", None),
                override_start_time_sec=getattr(plan, "override_start_time_sec", None),
                override_end_time_sec=getattr(plan, "override_end_time_sec", None),
                notes=notes,
            )
        else:
            rescue_plan = intersection_agent_module.PreemptionPlan(
                plan_type="intrusive",
                target_phase_idx=int(target_phase),
                jump_to_phase_idx=int(target_phase),
                jump_time_sec=float(sim_time),
                planned_green_window=getattr(plan, "planned_green_window", None),
                notes=notes,
            )
        route_dist = lookahead_diag.get("route_distance_to_selected_edge_m")
        _f2_approach_phase_rescue_state[key] = {
            "last_apply_time": float(sim_time),
            "target_phase": int(target_phase),
            "stage": str(stage),
            "route_distance_to_selected_edge_m": route_dist,
            "distance_to_stopline_m": float(d_stop_f),
            "speed_mps": float(speed_f),
            "reason": "current_tls_stopped_rescue",
        }
        if key not in _late_rescue_state:
            _late_rescue_state[key] = {
                "start_time": float(sim_time),
                "start_distance_m": float(d_stop_f),
                "start_speed_mps": float(speed_f),
                "start_phase": int(before_phase),
                "target_phase": int(target_phase),
                "started_by": f"current_tls_stopped_rescue:{str(stage)}:{guard_reason}",
            }
            rescue_started = True
        else:
            rescue_started = False

        diag.update(
            {
                "applied": True,
                "reason": str(rescue_reason),
                "guard_reason": str(guard_reason),
                "final_reason": str(final_reason),
                "original_plan_type": str(plan_type),
                "plan_type": str(getattr(rescue_plan, "plan_type", "") or "intrusive"),
                "target_phase": int(target_phase),
                "before_phase": int(before_phase),
                "distance_to_stopline_m": float(d_stop_f),
                "route_distance_to_selected_edge_m": route_dist,
                "speed_mps": float(speed_f),
                "max_distance_m": float(max_dist),
                "max_speed_mps": float(max_speed),
                "lookahead_hops": int(hops),
                "max_lookahead_hops": int(max_hops),
                "rescue_started": bool(rescue_started),
            }
        )
        _fed_evt_main(
            "f2.current_tls_stopped_rescue.plan",
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            stage=str(stage),
            decision_source="f2_current_tls_stopped_rescue",
            **diag,
        )
        return rescue_plan, diag

    def _route_lookahead_actuation_filter(
        *,
        mode: str,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        lookahead_hops: int,
        d_stop: float,
        plan: object,
    ) -> Tuple[Optional[object], Dict[str, object]]:
        """Gate local actuation when the selected TLS is only a downstream route lookahead target.

        Discovery and request routing may legitimately bind the EV to the first downstream TLS
        before the EV reaches that TLS's controlled inbound edge. In that state, full preemption
        can be premature: the EV cannot consume the service window yet, and repeated extension may
        worsen downstream backpressure. This filter preserves discovery while separating
        lookahead warmup from direct-approach actuation.
        """
        plan_type = str(getattr(plan, "plan_type", "") or "") if plan is not None else ""
        ev_edge_s = str(ev_edge or "")
        selected_s = str(selected_in_edge or "")
        try:
            speed_f = float(traci.vehicle.getSpeed(str(ev_id)))
        except Exception:
            speed_f = -1.0
        route_dist = None
        if selected_s:
            try:
                if ev_edge_s == selected_s:
                    route_dist = float(d_stop)
                else:
                    route_dist = route_distance_to_edge_stopline(str(ev_id), selected_s)
            except Exception:
                route_dist = None
        route_dist_f = None if route_dist is None else float(route_dist)
        route_lookahead = bool(selected_s and ev_edge_s != selected_s) or int(lookahead_hops) > 0
        on_selected_in_edge = bool(selected_s and ev_edge_s == selected_s)
        full_dist = max(0.0, float(getattr(args, "b1_lookahead_full_preemption_distance_m", 70.0)))
        warmup_cap = max(0.0, float(getattr(args, "b1_lookahead_warmup_max_extension_sec", 4.0)))
        stop_speed = max(0.0, float(getattr(args, "b1_lookahead_upstream_stop_speed_mps", 0.5)))
        stop_min_dist = max(0.0, float(getattr(args, "b1_lookahead_upstream_stop_min_distance_m", 30.0)))
        guard_enabled = bool(getattr(args, "b1_lookahead_actuation_guard_enable", True))
        skip_upstream_stopped_enabled = bool(getattr(args, "b1_lookahead_skip_upstream_stopped_enable", True))
        f2_upstream_rescue_enabled = bool(
            _is_f2_family(str(mode))
            and getattr(args, "f2_lookahead_upstream_stopped_rescue_enable", True)
        )
        f2_upstream_rescue_max_dist = max(
            0.0,
            float(getattr(args, "f2_lookahead_upstream_stopped_rescue_max_distance_m", 120.0)),
        )
        f2_upstream_rescue_max_hops = max(
            0,
            int(getattr(args, "f2_lookahead_upstream_stopped_rescue_max_hops", 1)),
        )
        f2_upstream_rescue_cap = max(
            0.0,
            float(getattr(args, "f2_lookahead_upstream_stopped_rescue_extension_sec", warmup_cap)),
        )
        near_enough_for_full = bool(
            route_dist_f is not None
            and math.isfinite(float(route_dist_f))
            and float(route_dist_f) <= float(full_dist)
        )
        preemption_eligible = bool((not guard_enabled) or (not route_lookahead) or on_selected_in_edge or near_enough_for_full)
        upstream_stopped = bool(
            guard_enabled
            and route_lookahead
            and (not on_selected_in_edge)
            and speed_f >= 0.0
            and speed_f <= float(stop_speed)
            and (
                route_dist_f is None
                or not math.isfinite(float(route_dist_f))
                or float(route_dist_f) >= float(stop_min_dist)
            )
        )
        active_plan = bool(plan is not None and plan_type not in ("", "none", "restore"))
        f2_upstream_rescue_eligible = bool(
            f2_upstream_rescue_enabled
            and active_plan
            and route_lookahead
            and (not on_selected_in_edge)
            and upstream_stopped
            and plan_type in ("saturation_reduction", "non_intrusive")
            and int(lookahead_hops) <= int(f2_upstream_rescue_max_hops)
            and route_dist_f is not None
            and math.isfinite(float(route_dist_f))
            and float(route_dist_f) <= float(f2_upstream_rescue_max_dist)
        )
        f2_downstream_release_guard: Dict[str, object] = {}
        f2_downstream_release_blocked = False
        if f2_upstream_rescue_eligible:
            f2_downstream_release_guard = _f2_downstream_release_guard_diag(
                stage=str(stage),
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                ev_edge=str(ev_edge_s),
                selected_in_edge=str(selected_s),
                plan=plan,
                lookahead_diag={
                    "route_lookahead": bool(route_lookahead),
                    "lookahead_hops": int(lookahead_hops),
                    "route_distance_to_selected_edge_m": route_dist_f,
                    "upstream_stopped": bool(upstream_stopped),
                    "action": "cap",
                    "reason": "upstream_stopped_rescue_extension_cap",
                },
            )
            if bool(f2_downstream_release_guard.get("skip_release", False)):
                f2_downstream_release_blocked = True
                f2_upstream_rescue_eligible = False
        filtered_plan = plan
        reason = "eligible"
        action = "apply"
        capped = False
        rescue_applied = False
        original_ext = float(getattr(plan, "extend_green_sec", 0.0) or 0.0) if plan is not None else 0.0
        applied_ext = original_ext
        f2_downstream_apply_guard: Dict[str, object] = {}

        if _is_f2_family(str(mode)) and active_plan:
            f2_downstream_apply_guard = _f2_downstream_apply_guard_diag(
                stage=str(stage),
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                ev_edge=str(ev_edge_s),
                selected_in_edge=str(selected_s),
                plan=plan,
                lookahead_diag={
                    "route_lookahead": bool(route_lookahead),
                    "lookahead_hops": int(lookahead_hops),
                    "route_distance_to_selected_edge_m": route_dist_f,
                    "upstream_stopped": bool(upstream_stopped),
                    "action": str(action),
                    "reason": str(reason),
                },
                source=str(stage),
            )
            if bool(f2_downstream_apply_guard.get("skip_apply", False)):
                filtered_plan = None
                action = "skip"
                reason = "downstream_apply_guard"
                applied_ext = 0.0

        if filtered_plan is not None and guard_enabled and active_plan and route_lookahead and (not on_selected_in_edge):
            if skip_upstream_stopped_enabled and upstream_stopped:
                if f2_upstream_rescue_eligible:
                    filtered_plan = _clone_plan_with_extension_cap(
                        plan,
                        min(float(warmup_cap), float(f2_upstream_rescue_cap)),
                        "upstream_stopped_rescue",
                    )
                    action = "cap"
                    reason = "upstream_stopped_rescue_extension_cap"
                    capped = True
                    rescue_applied = True
                    applied_ext = float(getattr(filtered_plan, "extend_green_sec", 0.0) or 0.0)
                else:
                    filtered_plan = None
                    action = "skip"
                    reason = (
                        "downstream_release_guard"
                        if bool(f2_downstream_release_blocked)
                        else "upstream_stopped_before_selected_tls"
                    )
                    applied_ext = 0.0
            elif not preemption_eligible:
                if plan_type == "intrusive":
                    filtered_plan = None
                    action = "skip"
                    reason = "lookahead_intrusive_not_preemption_eligible"
                    applied_ext = 0.0
                elif plan_type in ("saturation_reduction", "non_intrusive") and original_ext > warmup_cap:
                    filtered_plan = _clone_plan_with_extension_cap(plan, warmup_cap, "warmup_cap")
                    action = "cap"
                    reason = "lookahead_warmup_extension_cap"
                    capped = True
                    applied_ext = float(getattr(filtered_plan, "extend_green_sec", 0.0) or 0.0)
                else:
                    reason = "lookahead_warmup_allowed"

        diag = {
            "mode": str(mode),
            "stage": str(stage),
            "guard_enabled": bool(guard_enabled),
            "route_lookahead": bool(route_lookahead),
            "lookahead_hops": int(lookahead_hops),
            "ev_edge": str(ev_edge_s),
            "selected_in_edge": str(selected_s),
            "on_selected_in_edge": bool(on_selected_in_edge),
            "route_distance_to_selected_edge_m": route_dist_f,
            "full_preemption_distance_m": float(full_dist),
            "preemption_eligible": bool(preemption_eligible),
            "upstream_stopped": bool(upstream_stopped),
            "speed_mps": float(speed_f),
            "distance_to_stopline_m": float(d_stop),
            "plan_type": str(plan_type),
            "original_extend_green_sec": float(original_ext),
            "applied_extend_green_sec": float(applied_ext),
            "warmup_max_extension_sec": float(warmup_cap),
            "f2_upstream_stopped_rescue_enabled": bool(f2_upstream_rescue_enabled),
            "f2_upstream_stopped_rescue_eligible": bool(f2_upstream_rescue_eligible),
            "f2_upstream_stopped_rescue_applied": bool(rescue_applied),
            "f2_upstream_stopped_rescue_max_distance_m": float(f2_upstream_rescue_max_dist),
            "f2_upstream_stopped_rescue_max_hops": int(f2_upstream_rescue_max_hops),
            "f2_upstream_stopped_rescue_extension_sec": float(f2_upstream_rescue_cap),
            "f2_downstream_apply_guard": dict(f2_downstream_apply_guard),
            "f2_downstream_release_guard": dict(f2_downstream_release_guard),
            "action": str(action),
            "reason": str(reason),
            "capped": bool(capped),
            "apply_allowed": bool(filtered_plan is not None),
        }
        if guard_enabled and active_plan and route_lookahead:
            _fed_evt_main(
                "local_actuation.lookahead_guard",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                **diag,
            )
            _fed_dbg_main(
                f"evt=LOCAL_ACTUATION_LOOKAHEAD_GUARD mode={mode} stage={stage} tls={tls_id} ev={ev_id} "
                f"action={action} reason={reason} plan_type={plan_type} ev_edge={ev_edge_s} "
                f"selected_edge={selected_s} hops={int(lookahead_hops)} "
                f"route_dist={('NA' if route_dist_f is None else f'{float(route_dist_f):.2f}')} "
                f"speed={speed_f:.2f} preemption_eligible={int(bool(preemption_eligible))} "
                f"upstream_stopped={int(bool(upstream_stopped))} "
                f"f2_rescue={int(bool(rescue_applied))} ext={original_ext:.2f}->{applied_ext:.2f}"
            )
        return filtered_plan, diag

    def _remember_f2_last_local_anchor_plan(
        *,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        plan: object,
        plan_type: str,
        selected_in_edge: str,
        lookahead_diag: Dict[str, object],
        source: str,
    ) -> None:
        active_plan_type = str(getattr(plan, "plan_type", "") or plan_type or "")
        if plan is None or active_plan_type in ("", "none", "restore"):
            return
        _f2_last_local_anchor_plan_state[(str(ev_id), str(tls_id))] = {
            "sim_time": float(sim_time),
            "plan": plan,
            "plan_type": str(active_plan_type),
            "selected_in_edge": str(selected_in_edge or ""),
            "lookahead_diag": dict(lookahead_diag or {}),
            "source": str(source or ""),
        }

    def _recent_f2_last_local_anchor_plan(
        *,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        selected_in_edge: str,
    ) -> Tuple[Optional[object], str, Dict[str, object]]:
        if not bool(getattr(args, "f2_weak_offer_last_local_fallback_enable", True)):
            return None, "", {"reason": "disabled"}
        max_age = max(0.0, float(getattr(args, "f2_weak_offer_last_local_fallback_max_age_sec", 20.0)))
        rec = dict(_f2_last_local_anchor_plan_state.get((str(ev_id), str(tls_id)), {}) or {})
        if not rec:
            return None, "", {"reason": "missing"}
        age = float(sim_time) - float(rec.get("sim_time", float(sim_time)))
        if age < 0.0 or age > max_age:
            return None, str(rec.get("plan_type", "") or ""), {
                "reason": "stale",
                "age_sec": float(age),
                "max_age_sec": float(max_age),
                "source": str(rec.get("source", "") or ""),
            }
        cached_selected = str(rec.get("selected_in_edge", "") or "")
        current_selected = str(selected_in_edge or "")
        if cached_selected and current_selected and cached_selected != current_selected:
            return None, str(rec.get("plan_type", "") or ""), {
                "reason": "selected_edge_changed",
                "age_sec": float(age),
                "cached_selected_in_edge": str(cached_selected),
                "selected_in_edge": str(current_selected),
                "source": str(rec.get("source", "") or ""),
            }
        plan = rec.get("plan")
        plan_type = str(rec.get("plan_type", "") or getattr(plan, "plan_type", "") or "")
        if plan is None or plan_type in ("", "none", "restore"):
            return None, plan_type, {
                "reason": "weak_cached_plan",
                "age_sec": float(age),
                "source": str(rec.get("source", "") or ""),
            }
        return plan, plan_type, {
            "reason": "ok",
            "age_sec": float(age),
            "max_age_sec": float(max_age),
            "source": str(rec.get("source", "") or ""),
            "cached_selected_in_edge": str(cached_selected),
        }

    def _try_f2_b1_continuity_apply(
        *,
        ag: object,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        lookahead_hops: int,
        d_stop: float,
        trigger_reason: str,
        f2_meta: Optional[Dict[str, object]] = None,
    ) -> bool:
        """Keep B1-equivalent local actuation alive while F2 peer coordination is idle."""
        local_plan, local_plan_type = _f2_local_current_plan_for_actuation(ag)
        source = "current_local_plan"
        cached_diag: Dict[str, object] = {}
        if local_plan is None:
            local_plan, local_plan_type, cached_diag = _recent_f2_last_local_anchor_plan(
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                selected_in_edge=str(selected_in_edge),
            )
            source = "recent_local_anchor_plan" if local_plan is not None else "none"

        if local_plan is None:
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(cached_diag.get("reason", "missing_local_plan")),
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                selected_in_edge=str(selected_in_edge),
                ev_edge=str(ev_edge),
                distance_to_stopline_m=float(d_stop),
            )
            return False

        plan_for_apply, lookahead_diag = _route_lookahead_actuation_filter(
            mode="F2",
            stage="b1_continuity_idle_gap",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_hops=int(lookahead_hops),
            d_stop=float(d_stop),
            plan=local_plan,
        )
        if plan_for_apply is None:
            tls_before_for_rescue = _tls_diag_snapshot(str(tls_id), float(sim_time))
            plan_for_apply, stopped_rescue_diag = _maybe_f2_current_tls_stopped_rescue_plan(
                stage="b1_continuity_idle_gap",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                ag=ag,
                ev_edge=str(ev_edge),
                selected_in_edge=str(selected_in_edge),
                d_stop=float(d_stop),
                plan=local_plan,
                tls_before=tls_before_for_rescue,
                lookahead_diag=lookahead_diag,
                f2_meta=f2_meta,
            )
            if plan_for_apply is None or not bool(stopped_rescue_diag.get("applied", False)):
                _fed_evt_main(
                    "f2.b1_continuity.skip",
                    role="intersection",
                    ev_id=str(ev_id),
                    tls_id=str(tls_id),
                    sim_time=float(sim_time),
                    reason=str(lookahead_diag.get("reason", "lookahead_guard")),
                    stopped_rescue_reason=str(stopped_rescue_diag.get("reason", "")),
                    trigger_reason=str(trigger_reason),
                    source=str(source),
                    local_plan_type=str(local_plan_type or ""),
                    route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                    route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                    lookahead_hops=int(lookahead_diag.get("lookahead_hops", 0) or 0),
                    selected_in_edge=str(selected_in_edge),
                    ev_edge=str(ev_edge),
                    distance_to_stopline_m=float(d_stop),
                )
                return False
            lookahead_diag = dict(lookahead_diag or {})
            lookahead_diag["action"] = "rescue"
            lookahead_diag["reason"] = str(stopped_rescue_diag.get("reason", "current_tls_stopped_rescue"))
            lookahead_diag["current_tls_stopped_rescue_applied"] = True

        tls_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
        plan_for_apply, approach_rescue_diag = _maybe_f2_approach_phase_rescue_plan(
            stage="b1_continuity_idle_gap",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ag=ag,
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            d_stop=float(d_stop),
            plan=plan_for_apply,
            tls_before=tls_before,
            lookahead_diag=lookahead_diag,
            f2_meta=f2_meta,
        )
        apply_guard_diag = _f2_downstream_apply_guard_diag(
            stage="b1_continuity_idle_gap",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            plan=plan_for_apply,
            lookahead_diag=lookahead_diag,
            source=str(source),
        )
        if bool(apply_guard_diag.get("skip_apply", False)) and not bool(approach_rescue_diag.get("applied", False)):
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason="downstream_apply_guard",
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                lookahead_hops=int(lookahead_diag.get("lookahead_hops", 0) or 0),
                downstream_apply_guard=dict(apply_guard_diag),
                selected_in_edge=str(selected_in_edge),
                ev_edge=str(ev_edge),
                distance_to_stopline_m=float(d_stop),
            )
            return False
        replay_guard_diag = _f2_downstream_replay_guard_diag(
            stage="b1_continuity_idle_gap",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            plan=plan_for_apply,
            lookahead_diag=lookahead_diag,
            source=str(source),
        )
        if bool(replay_guard_diag.get("skip_replay", False)) and not bool(approach_rescue_diag.get("applied", False)):
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason="downstream_replay_guard",
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                lookahead_hops=int(lookahead_diag.get("lookahead_hops", 0) or 0),
                downstream_replay_guard=dict(replay_guard_diag),
                selected_in_edge=str(selected_in_edge),
                ev_edge=str(ev_edge),
                distance_to_stopline_m=float(d_stop),
            )
            return False
        skip_apply, skip_diag = _should_skip_f2_local_anchor_preapply(
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            plan=plan_for_apply,
            tls_before=tls_before,
            lookahead_diag=lookahead_diag,
        )
        if skip_apply:
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason="redundant_local_anchor_preapply",
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                lookahead_hops=int(lookahead_diag.get("lookahead_hops", 0) or 0),
                before_phase=int(tls_before.get("phase", -1)),
                before_state=str(tls_before.get("state", "")),
                before_next_switch=float(tls_before.get("next_switch", -1.0)),
                before_next_switch_rem_s=float(tls_before.get("next_switch_rem_s", -1.0)),
                redundant_apply_min_interval_s=skip_diag.get("min_interval_s"),
                redundant_apply_dt_since_last_s=skip_diag.get("dt_since_last_s"),
                redundant_apply_distance_m=skip_diag.get("distance_for_interval_m"),
                redundant_apply_suppressed_count=skip_diag.get("suppressed_count"),
                service_window_risk=skip_diag.get("service_window_risk"),
            )
            return False

        fallback_slow_skip, fallback_slow_diag = False, {"reason": "approach_rescue_applied"}
        if not bool(approach_rescue_diag.get("applied", False)):
            fallback_slow_skip, fallback_slow_diag = _should_skip_f2_fallback_slow_green(
                stage="b1_continuity_idle_gap",
                event_type="f2.b1_continuity.skip",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                ag=ag,
                plan=plan_for_apply,
                tls_before=tls_before,
                selected_in_edge=str(selected_in_edge),
                d_stop=float(d_stop),
                f2_meta=f2_meta,
            )
        if fallback_slow_skip:
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(fallback_slow_diag.get("reason", "fallback_slow_green_guard")),
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                selected_in_edge=str(selected_in_edge),
                ev_edge=str(ev_edge),
                distance_to_stopline_m=float(d_stop),
                fallback_slow_green_guard=dict(fallback_slow_diag),
            )
            return False

        cadence_skip, cadence_diag = False, {"reason": "approach_rescue_applied"}
        if not bool(approach_rescue_diag.get("applied", False)):
            cadence_skip, cadence_diag = _should_skip_f2_fallback_cadence(
                stage="b1_continuity_idle_gap",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            plan=plan_for_apply,
            d_stop=float(d_stop),
        )
        if cadence_skip:
            _fed_evt_main(
                "f2.b1_continuity.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(cadence_diag.get("reason", "cadence_guard")),
                trigger_reason=str(trigger_reason),
                source=str(source),
                local_plan_type=str(local_plan_type or ""),
                applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
                route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
                selected_in_edge=str(selected_in_edge),
                ev_edge=str(ev_edge),
                distance_to_stopline_m=float(d_stop),
                fallback_cadence_guard=dict(cadence_diag),
            )
            return False

        ag.apply_plan_to_tls(
            float(sim_time),
            plan_for_apply,
            decision_source="f2_b1_continuity",
        )
        tls_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
        _remember_f2_last_local_anchor_plan(
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            plan=plan_for_apply,
            plan_type=str(local_plan_type or getattr(plan_for_apply, "plan_type", "") or ""),
            selected_in_edge=str(selected_in_edge),
            lookahead_diag=lookahead_diag,
            source="b1_continuity_idle_gap",
        )
        _fed_evt_main(
            "f2.b1_continuity.apply",
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            trigger_reason=str(trigger_reason),
            source=str(source),
            local_plan_type=str(local_plan_type or ""),
            applied_local_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
            final_reason=str((f2_meta or {}).get("final_reason", "")),
            route_lookahead=bool(lookahead_diag.get("route_lookahead", False)),
            route_distance_to_selected_edge_m=lookahead_diag.get("route_distance_to_selected_edge_m"),
            lookahead_hops=int(lookahead_diag.get("lookahead_hops", 0) or 0),
            lookahead_action=str(lookahead_diag.get("action", "")),
            lookahead_reason=str(lookahead_diag.get("reason", "")),
            current_tls_stopped_rescue_applied=bool(lookahead_diag.get("current_tls_stopped_rescue_applied", False)),
            approach_phase_rescue_applied=bool(approach_rescue_diag.get("applied", False)),
            approach_phase_rescue_reason=str(approach_rescue_diag.get("reason", "")),
            downstream_replay_guard=dict(replay_guard_diag),
            before_phase=int(tls_before.get("phase", -1)),
            after_phase=int(tls_after.get("phase", -1)),
            before_state=str(tls_before.get("state", "")),
            after_state=str(tls_after.get("state", "")),
            before_next_switch=float(tls_before.get("next_switch", -1.0)),
            after_next_switch=float(tls_after.get("next_switch", -1.0)),
            before_next_switch_rem_s=float(tls_before.get("next_switch_rem_s", -1.0)),
            after_next_switch_rem_s=float(tls_after.get("next_switch_rem_s", -1.0)),
            dedupe_bypassed=bool(skip_diag.get("skip_bypassed", False)),
            dedupe_bypass_reason=str(skip_diag.get("bypass_reason", "")),
            service_window_risk=skip_diag.get("service_window_risk"),
            selected_in_edge=str(selected_in_edge),
            ev_edge=str(ev_edge),
            distance_to_stopline_m=float(d_stop),
        )
        _emit_f2_apply_effect(
            stage="b1_continuity_idle_gap",
            decision_source="f2_b1_continuity",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ag=ag,
            selected_in_edge=str(selected_in_edge),
            ev_edge=str(ev_edge),
            d_stop=float(d_stop),
            before=tls_before,
            after=tls_after,
            plan=plan_for_apply,
            f2_meta=f2_meta,
        )
        _fed_dbg_main(
            f"evt=F2_B1_CONTINUITY_APPLY tls={tls_id} ev={ev_id} "
            f"trigger={trigger_reason} source={source} plan_type={local_plan_type} "
            f"phase={tls_before.get('phase')}->{tls_after.get('phase')} "
            f"next_switch={float(tls_before.get('next_switch', -1.0)):.2f}->{float(tls_after.get('next_switch', -1.0)):.2f} "
            f"sim={float(sim_time):.2f}"
        )
        return True

    def _f2_strict_selected_offer_allowed(
        *,
        ag: object,
        chosen: object,
        f2_meta: Optional[Dict[str, object]],
    ) -> Tuple[bool, Dict[str, object]]:
        """Allow F2 to diverge from B1 only for fresh, useful peer-refined offers."""
        meta = dict(f2_meta or {})
        final_reason = str(meta.get("final_reason", "") or "")
        refine_reason = str(meta.get("refine_reason", "") or "")
        refine_allowed = int(meta.get("refine_allowed", 0) or 0)
        diag: Dict[str, object] = {
            "final_reason": str(final_reason),
            "refine_reason": str(refine_reason),
            "refine_allowed": int(refine_allowed),
            "reason": "not_evaluated",
        }
        if chosen is None:
            diag["reason"] = "no_selected_offer"
            return False, diag
        offer_id = str(getattr(chosen, "offer_id", "") or "")
        selected_source = str(meta.get("selected_source", "") or "")
        if not selected_source and offer_id:
            try:
                selected_source = str(
                    getattr(ag, "_f2_selected_offer_source_by_id", {}).get(offer_id, "") or ""
                )
            except Exception:
                selected_source = ""
        diag["selected_offer_id"] = str(offer_id)
        diag["selected_source"] = str(selected_source)
        if bool(getattr(args, "f2_strict_b1_floor_peer_override_only", True)):
            if selected_source != "peer_override":
                diag["reason"] = "not_peer_override"
                return False, diag
        if final_reason.startswith("fallback_local_") or final_reason in {
            "no_offers",
            "blocked_infeasible_no_local_feasible",
        }:
            diag["reason"] = "local_or_blocked_final_reason"
            return False, diag
        if refine_allowed <= 0 or refine_reason != "ok":
            diag["reason"] = "no_fresh_peer_refine"
            return False, diag
        effect_diag = _selected_offer_effect_diag(ag, chosen)
        diag["selected_effect"] = dict(effect_diag)
        if bool(effect_diag.get("weak_effect", False)):
            diag["reason"] = "weak_selected_offer_effect"
            return False, diag
        plan_type = str(effect_diag.get("plan_type", "") or "")
        offer_action = str(effect_diag.get("offer_action", "") or "")
        if plan_type in ("", "none", "restore") or offer_action == "none":
            diag["reason"] = "noop_selected_offer"
            return False, diag
        try:
            fb = ag._reservation_feedback(  # type: ignore[attr-defined]
                str(getattr(getattr(ag, "active_ev", None), "ev_id", "") or ""),
                max_age_sec=max(2.0, float(getattr(ag, "fed_soft_ttl_sec", 8.0))),
            )
        except Exception:
            fb = {}
        hard_accepted = int((fb or {}).get("hard_accepted", 0) or 0)
        soft_accepted = int((fb or {}).get("soft_accepted", 0) or 0)
        diag["hard_accepted"] = int(hard_accepted)
        diag["soft_accepted"] = int(soft_accepted)
        if hard_accepted <= 0 and soft_accepted <= 0:
            diag["reason"] = "no_recent_peer_acceptance"
            return False, diag
        diag["reason"] = "fresh_useful_peer_offer"
        return True, diag

    def _try_f2_strict_b1_floor_apply(
        *,
        ag: object,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ev_edge: str,
        selected_in_edge: str,
        lookahead_hops: int,
        d_stop: float,
        trigger_reason: str,
        f2_meta: Optional[Dict[str, object]] = None,
        precomputed_plan: object = None,
    ) -> bool:
        """Apply the same local B1 tick/filter/guard path as the F2 fail-soft floor."""
        plan = precomputed_plan
        plan_source = "precomputed_f2_primary_plan" if plan is not None else "tick"
        if plan is None:
            plan = ag.tick(float(sim_time))
        plan_type = str(getattr(plan, "plan_type", "") or "") if plan is not None else ""
        plan_for_apply, lookahead_diag = _route_lookahead_actuation_filter(
            # This is still the B1 floor plan, but it is being applied inside F2.
            # Use F2 mode so the bounded upstream-stopped rescue can prevent the
            # floor from repeatedly deferring a downstream lookahead TLS while the
            # EV is stopped before the selected inbound edge.
            mode="F2",
            stage="f2_strict_b1_floor",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_hops=int(lookahead_hops),
            d_stop=float(d_stop),
            plan=plan,
        )
        b1_downstream_diag = b1_downstream_blockage_diag(
            ev_id=str(ev_id),
            current_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            lookahead_edges=int(getattr(args, "b1_downstream_blockage_lookahead_edges", 3)),
            min_halt_n=int(getattr(args, "b1_downstream_blockage_min_halt_n", 3)),
            max_mean_speed_mps=float(getattr(args, "b1_downstream_blockage_max_mean_speed_mps", 1.0)),
            min_veh_n=int(getattr(args, "b1_downstream_blockage_min_veh_n", 2)),
            max_occupancy_pct=float(getattr(args, "b1_downstream_blockage_max_occupancy_pct", 35.0)),
            edge_to_tls_map=edge_to_tls,
            stop_at_non_tls=bool(_plain_f2_tls_bounded_downstream_scan()),
        )
        b1_downstream_diag = _merge_external_downstream_context(
            b1_downstream_diag,
            tls_id=str(tls_id),
            ev_id=str(ev_id),
            sim_time=float(sim_time),
            max_age_sec=float(getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0),
        )
        b1_downstream_diag = _annotate_immediate_blockage_diag(b1_downstream_diag)
        blocked_by_lookahead = bool(plan is not None and plan_for_apply is None)
        applied_plan_type = str(getattr(plan_for_apply, "plan_type", "") or "") if plan_for_apply is not None else ""
        aggressive_plan = applied_plan_type in ("intrusive", "saturation_reduction")
        active_plan = applied_plan_type not in ("", "none", "restore")
        immediate_blocked = bool(b1_downstream_diag.get("immediate_blockage_severe", False))
        blocked_by_downstream = bool(
            bool(getattr(args, "b1_downstream_blockage_guard_enable", True))
            and bool(b1_downstream_diag.get("blocked", False))
            and (aggressive_plan or (immediate_blocked and active_plan))
        )
        if plan is None or blocked_by_lookahead or blocked_by_downstream:
            reason = "tick_none" if plan is None else (
                str(lookahead_diag.get("reason", "lookahead_guard"))
                if blocked_by_lookahead
                else ("immediate_downstream_blockage_guard" if immediate_blocked else "downstream_blockage_guard")
            )
            _fed_evt_main(
                "f2.strict_b1_floor.skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(reason),
                trigger_reason=str(trigger_reason),
                final_reason=str((f2_meta or {}).get("final_reason", "")),
                refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                plan_source=str(plan_source),
                plan_type=str(plan_type),
                applied_plan_type=str(applied_plan_type),
                ev_edge=str(ev_edge),
                selected_in_edge=str(selected_in_edge),
                distance_to_stopline_m=float(d_stop),
                lookahead_diag=dict(lookahead_diag or {}),
                downstream_blockage=dict(b1_downstream_diag or {}),
            )
            if bool(b1_downstream_diag.get("external_downstream_context_used", False)):
                _fed_evt_main(
                    "f2.strict_b1_floor.apply_drone_guard",
                    role="intersection",
                    sim_time=float(sim_time),
                    request_id=str(b1_downstream_diag.get("external_downstream_context_request_id", "")),
                    ev_id=str(ev_id),
                    requester_tls=str(tls_id),
                    tls_id=str(tls_id),
                    provider_id=str(b1_downstream_diag.get("external_downstream_context_provider", "")),
                    provider_type=str(b1_downstream_diag.get("external_downstream_context_provider_type", "drone")),
                    target_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                    decision_deadline_sec=float(
                        getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0
                    ),
                    request_latency_ms=-1.0,
                    response_latency_ms=-1.0,
                    context_age_ms=float(
                        b1_downstream_diag.get("external_downstream_context_age_sec", -1.0) or -1.0
                    ) * 1000.0,
                    blocked=bool(b1_downstream_diag.get("blocked", False)),
                    reason=str(reason),
                    worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                    worst_edge_offset=int(b1_downstream_diag.get("worst_edge_offset", -1) or -1),
                    confidence=float(
                        b1_downstream_diag.get("external_downstream_context_confidence", -1.0) or -1.0
                    ),
                    selected_action="skip_b1_floor_apply",
                    decision_source="f2_strict_b1_floor",
                    trigger_reason=str(trigger_reason),
                    final_reason=str((f2_meta or {}).get("final_reason", "")),
                    plan_type=str(plan_type),
                    applied_plan_type=str(applied_plan_type),
                )
            return False

        before = _tls_diag_snapshot(str(tls_id), float(sim_time))
        cadence_skip, cadence_diag = _should_skip_f2_strict_b1_floor_preapply(
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            plan=plan_for_apply,
            tls_before=before,
            lookahead_diag=dict(lookahead_diag or {}),
            d_stop=float(d_stop),
        )
        if bool(cadence_skip):
            _fed_evt_main(
                "f2.apply_skipped",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason="strict_b1_floor_cadence_guard",
                trigger_reason=str(trigger_reason),
                decision_source="f2_strict_b1_floor",
                plan_type=str(plan_type),
                applied_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                ev_edge=str(ev_edge),
                selected_in_edge=str(selected_in_edge),
                distance_to_stopline_m=float(d_stop),
                strict_b1_floor_cadence_guard=dict(cadence_diag),
                lookahead_diag=dict(lookahead_diag or {}),
                downstream_blockage=dict(b1_downstream_diag or {}),
            )
            _fed_evt_main(
                "f2.strict_b1_floor.cadence_skip",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason=str(cadence_diag.get("reason", "cadence_guard")),
                trigger_reason=str(trigger_reason),
                final_reason=str((f2_meta or {}).get("final_reason", "")),
                refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                plan_source=str(plan_source),
                plan_type=str(plan_type),
                applied_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
                ev_edge=str(ev_edge),
                selected_in_edge=str(selected_in_edge),
                distance_to_stopline_m=float(d_stop),
                strict_b1_floor_cadence_guard=dict(cadence_diag),
                lookahead_diag=dict(lookahead_diag or {}),
                downstream_blockage=dict(b1_downstream_diag or {}),
            )
            return False
        ag.apply_plan_to_tls(float(sim_time), plan_for_apply, decision_source="f2_strict_b1_floor")
        after = _tls_diag_snapshot(str(tls_id), float(sim_time))
        _fed_evt_main(
            "f2.strict_b1_floor.apply",
            role="intersection",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            reason="b1_floor_apply",
            trigger_reason=str(trigger_reason),
            final_reason=str((f2_meta or {}).get("final_reason", "")),
            refine_reason=str((f2_meta or {}).get("refine_reason", "")),
            plan_source=str(plan_source),
            plan_type=str(plan_type),
            applied_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
            strict_b1_floor_cadence_guard=dict(cadence_diag),
            before_phase=int(before.get("phase", -1)),
            after_phase=int(after.get("phase", -1)),
            before_next_switch=float(before.get("next_switch", -1.0)),
            after_next_switch=float(after.get("next_switch", -1.0)),
            ev_edge=str(ev_edge),
            selected_in_edge=str(selected_in_edge),
            distance_to_stopline_m=float(d_stop),
            lookahead_diag=dict(lookahead_diag or {}),
            downstream_blockage=dict(b1_downstream_diag or {}),
        )
        if bool(b1_downstream_diag.get("external_downstream_context_used", False)):
            _fed_evt_main(
                "f2.strict_b1_floor.apply_drone_guard",
                role="intersection",
                sim_time=float(sim_time),
                request_id=str(b1_downstream_diag.get("external_downstream_context_request_id", "")),
                ev_id=str(ev_id),
                requester_tls=str(tls_id),
                tls_id=str(tls_id),
                provider_id=str(b1_downstream_diag.get("external_downstream_context_provider", "")),
                provider_type=str(b1_downstream_diag.get("external_downstream_context_provider_type", "drone")),
                target_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                decision_deadline_sec=float(
                    getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0
                ),
                request_latency_ms=-1.0,
                response_latency_ms=-1.0,
                context_age_ms=float(
                    b1_downstream_diag.get("external_downstream_context_age_sec", -1.0) or -1.0
                ) * 1000.0,
                blocked=bool(b1_downstream_diag.get("blocked", False)),
                reason="b1_floor_apply_with_drone_context",
                worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                worst_edge_offset=int(b1_downstream_diag.get("worst_edge_offset", -1) or -1),
                confidence=float(
                    b1_downstream_diag.get("external_downstream_context_confidence", -1.0) or -1.0
                ),
                selected_action="apply_b1_floor",
                decision_source="f2_strict_b1_floor",
                trigger_reason=str(trigger_reason),
                final_reason=str((f2_meta or {}).get("final_reason", "")),
                plan_type=str(plan_type),
                applied_plan_type=str(getattr(plan_for_apply, "plan_type", "") or ""),
            )
        _emit_f2_apply_effect(
            stage="strict_b1_floor",
            decision_source="f2_strict_b1_floor",
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ag=ag,
            selected_in_edge=str(selected_in_edge),
            ev_edge=str(ev_edge),
            d_stop=float(d_stop),
            before=before,
            after=after,
            plan=plan_for_apply,
            f2_meta=f2_meta,
        )
        return True

    def _emit_service_window_diag(
        *,
        mode: str,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        route_ctx: Dict[str, object],
        tls_snap: Dict[str, object],
        ev_snap: Dict[str, object],
        target_phase: int,
        target_green: bool,
        next_target_window_start: Optional[float],
        next_target_window_end: Optional[float],
    ) -> None:
        key = (str(ev_id), str(tls_id))
        speed = ev_snap.get("speed_mps")
        try:
            speed_f = float(speed)
        except Exception:
            speed_f = -1.0
        try:
            dist_f = float(ev_snap.get("distance_to_stopline_m", -1.0))
        except Exception:
            dist_f = -1.0
        prev = dict(_service_window_state.get(key, {}) or {})
        last_apply = dict(_apply_diag_state.get(key, {}) or {})
        prev_green = bool(prev.get("target_green", False))
        if prev and prev_green and (not bool(target_green)) and dist_f >= 0.0 and dist_f <= 160.0:
            _fed_dbg_main(
                f"evt=EV_SERVICE_WINDOW_MISSED mode={mode} ev={ev_id} tls={tls_id} sim={float(sim_time):.2f} "
                f"target_phase={int(target_phase)} phase={tls_snap.get('phase')} d_stop={dist_f:.2f} "
                f"speed={speed_f:.2f} next_target_green={next_target_window_start}"
            )
            _fed_evt_main(
                "ev.service_window.missed",
                role="intersection",
                mode=str(mode),
                stage=str(stage),
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                target_phase=int(target_phase),
                phase=int(tls_snap.get("phase", -1)),
                state=str(tls_snap.get("state", "")),
                next_switch=float(tls_snap.get("next_switch", -1.0)),
                next_target_window_start=next_target_window_start,
                next_target_window_end=next_target_window_end,
                speed_mps=float(speed_f),
                distance_to_stopline_m=float(dist_f),
                last_apply_time=last_apply.get("last_apply_time"),
                last_apply_mode=str(last_apply.get("last_apply_mode", "") or ""),
                last_apply_plan_type=str(last_apply.get("last_apply_plan_type", "") or ""),
                last_apply_decision_source=str(last_apply.get("last_apply_decision_source", "") or ""),
                last_apply_effective=last_apply.get("last_apply_effective"),
                **route_ctx,
            )
        if (not bool(target_green)) and speed_f >= 0.0 and speed_f < 1.0 and 0.0 <= dist_f <= 30.0:
            if not bool(prev.get("stop_wait_active", False)):
                _fed_evt_main(
                    "ev.service_window.stop_wait",
                    role="intersection",
                    mode=str(mode),
                    stage=str(stage),
                    ev_id=str(ev_id),
                    tls_id=str(tls_id),
                    sim_time=float(sim_time),
                    target_phase=int(target_phase),
                    phase=int(tls_snap.get("phase", -1)),
                    state=str(tls_snap.get("state", "")),
                    next_switch=float(tls_snap.get("next_switch", -1.0)),
                    next_target_window_start=next_target_window_start,
                    next_target_window_end=next_target_window_end,
                    speed_mps=float(speed_f),
                    distance_to_stopline_m=float(dist_f),
                    last_apply_time=last_apply.get("last_apply_time"),
                    last_apply_mode=str(last_apply.get("last_apply_mode", "") or ""),
                    last_apply_plan_type=str(last_apply.get("last_apply_plan_type", "") or ""),
                    last_apply_decision_source=str(last_apply.get("last_apply_decision_source", "") or ""),
                    last_apply_effective=last_apply.get("last_apply_effective"),
                    **route_ctx,
                )
                if _is_f2_family(str(mode)) and key not in _late_rescue_state:
                    _late_rescue_state[key] = {
                        "start_time": float(sim_time),
                        "start_distance_m": float(dist_f),
                        "start_speed_mps": float(speed_f),
                        "start_phase": int(tls_snap.get("phase", -1)),
                        "target_phase": int(target_phase),
                        "started_by": "service_window_stop_wait",
                    }
                    _fed_evt_main(
                        "f2.late_rescue.start",
                        role="intersection",
                        ev_id=str(ev_id),
                        tls_id=str(tls_id),
                        sim_time=float(sim_time),
                        stage=str(stage),
                        decision_source="service_window",
                        final_reason="target_not_green_stop_wait",
                        plan_type=str(last_apply.get("last_apply_plan_type", "") or ""),
                        offer_action="",
                        phase=int(tls_snap.get("phase", -1)),
                        target_phase=int(target_phase),
                        target_green=bool(target_green),
                        speed_mps=float(speed_f),
                        distance_to_stopline_m=float(dist_f),
                        last_apply_time=last_apply.get("last_apply_time"),
                        last_apply_mode=str(last_apply.get("last_apply_mode", "") or ""),
                        last_apply_plan_type=str(last_apply.get("last_apply_plan_type", "") or ""),
                        last_apply_decision_source=str(last_apply.get("last_apply_decision_source", "") or ""),
                        last_apply_effective=last_apply.get("last_apply_effective"),
                        **route_ctx,
                    )
        _service_window_state[key] = {
            "target_green": bool(target_green),
            "stop_wait_active": bool((not bool(target_green)) and speed_f >= 0.0 and speed_f < 1.0 and 0.0 <= dist_f <= 30.0),
            "last_time": float(sim_time),
            "last_phase": int(tls_snap.get("phase", -1)),
            "last_distance_m": float(dist_f),
            "last_speed_mps": float(speed_f),
        }

    def _emit_focus_trace(
        *,
        mode: str,
        stage: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        selected_in_edge: str,
        ev_edge: str,
        d_stop: float,
        lookahead_hops: int,
        route_nodes: Sequence[str],
    ) -> None:
        tls_snap = _tls_diag_snapshot(str(tls_id), float(sim_time))
        ev_snap = _active_ev_diag_snapshot(ag, str(ev_id), str(selected_in_edge), str(ev_edge), float(d_stop))
        route_ctx = _route_window_context(route_nodes, str(tls_id))
        target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        next_target_window_start, next_target_window_end = _target_window_for_diag(ag, float(sim_time), int(target_phase))
        target_green = _target_is_green_for_diag(tls_snap, int(target_phase))
        _fed_dbg_main(
            f"evt=EV_FOCUS_TRACE mode={mode} stage={stage} ev={ev_id} tls={tls_id} sim={float(sim_time):.2f} "
            f"edge={ev_snap.get('ev_edge')} selected_edge={ev_snap.get('selected_in_edge')} "
            f"d_stop={float(d_stop):.2f} speed={ev_snap.get('speed_mps')} "
            f"phase={tls_snap.get('phase')} prog={tls_snap.get('program')} "
            f"next_switch={float(tls_snap.get('next_switch', -1.0)):.2f} "
            f"target_phase={int(target_phase)} target_green={int(bool(target_green))} "
            f"prev_tls={route_ctx.get('route_prev_tls') or '-'} next_tls={route_ctx.get('route_next_tls') or '-'} "
            f"next2_tls={route_ctx.get('route_next2_tls') or '-'} lookahead_hops={int(lookahead_hops)}"
        )
        _fed_evt_main(
            "ev.focus_trace",
            role="intersection",
            mode=str(mode),
            stage=str(stage),
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            lookahead_hops=int(lookahead_hops),
            target_phase=int(target_phase),
            target_green=bool(target_green),
            next_target_window_start=next_target_window_start,
            next_target_window_end=next_target_window_end,
            **route_ctx,
            **tls_snap,
            **ev_snap,
        )
        _emit_service_window_diag(
            mode=str(mode),
            stage=str(stage),
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            ag=ag,
            route_ctx=route_ctx,
            tls_snap=tls_snap,
            ev_snap=ev_snap,
            target_phase=int(target_phase),
            target_green=bool(target_green),
            next_target_window_start=next_target_window_start,
            next_target_window_end=next_target_window_end,
        )

    def _emit_f2_apply_effect(
        *,
        stage: str,
        decision_source: str,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        ag: object,
        selected_in_edge: str,
        ev_edge: str,
        d_stop: float,
        before: Dict[str, object],
        after: Dict[str, object],
        offer: object = None,
        plan: object = None,
        f2_meta: Optional[Dict[str, object]] = None,
    ) -> None:
        offer_diag = _offer_diag_snapshot(offer)
        plan_diag = _plan_diag_snapshot(plan)
        ev_diag = _active_ev_diag_snapshot(ag, str(ev_id), str(selected_in_edge), str(ev_edge), float(d_stop))
        meta = dict(f2_meta or {})
        route_ctx = _route_window_context(list(ev_diag.get("active_ev_route_intersections", []) or []), str(tls_id))
        try:
            target_phase = int(plan_diag.get("plan_target_phase") or offer_diag.get("offer_target_phase") or _target_phase_for_diag(ag, str(selected_in_edge)))
        except Exception:
            target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
        target_green_after = _target_is_green_for_diag(after, int(target_phase))
        try:
            speed_f = float(ev_diag.get("speed_mps"))
        except Exception:
            speed_f = -1.0
        try:
            dist_f = float(ev_diag.get("distance_to_stopline_m", -1.0))
        except Exception:
            dist_f = -1.0
        final_reason = str(meta.get("final_reason", "") or "")
        plan_type = str(plan_diag.get("plan_type", "") or "")
        ineffective = bool(
            plan_type in ("", "none", "restore")
            or "blocked" in final_reason
            or "infeasible" in final_reason
        )
        rescue_key = (str(ev_id), str(tls_id))
        if speed_f >= 0.0 and speed_f < 1.0 and 0.0 <= dist_f <= 30.0 and ineffective:
            if rescue_key not in _late_rescue_state:
                _late_rescue_state[rescue_key] = {
                    "start_time": float(sim_time),
                    "start_distance_m": float(dist_f),
                    "start_speed_mps": float(speed_f),
                    "start_phase": int(before.get("phase", -1)),
                    "target_phase": int(target_phase),
                    "started_by": f"{str(stage)}:{final_reason or plan_type or 'no_effect'}",
                }
                _fed_evt_main(
                    "f2.late_rescue.start",
                    role="intersection",
                    ev_id=str(ev_id),
                    tls_id=str(tls_id),
                    sim_time=float(sim_time),
                    stage=str(stage),
                    decision_source=str(decision_source),
                    final_reason=str(final_reason),
                    plan_type=str(plan_type),
                    offer_action=str(offer_diag.get("offer_action", "") or ""),
                    phase=int(before.get("phase", -1)),
                    target_phase=int(target_phase),
                    target_green=bool(target_green_after),
                    speed_mps=float(speed_f),
                    distance_to_stopline_m=float(dist_f),
                    **route_ctx,
                )
        effective_apply = bool(
            plan_type in ("saturation_reduction", "non_intrusive", "intrusive")
            or str(offer_diag.get("offer_action", "") or "") in ("extend", "hurry", "jump")
            or int(before.get("phase", -1)) != int(after.get("phase", -1))
            or abs(float(before.get("next_switch", -1.0)) - float(after.get("next_switch", -1.0))) > 0.01
        )
        _apply_diag_state[rescue_key] = {
            "last_apply_time": float(sim_time),
            "last_apply_mode": "F2",
            "last_apply_plan_type": str(plan_type),
            "last_apply_decision_source": str(decision_source),
            "last_apply_effective": bool(effective_apply),
            "last_apply_phase_before": int(before.get("phase", -1)),
            "last_apply_phase_after": int(after.get("phase", -1)),
            "last_apply_target_phase": int(target_phase),
        }
        if rescue_key in _late_rescue_state and effective_apply:
            rescue = dict(_late_rescue_state.get(rescue_key, {}) or {})
            apply_count = int(rescue.get("apply_count", 0)) + 1
            rescue["apply_count"] = int(apply_count)
            _late_rescue_state[rescue_key] = rescue
            _fed_evt_main(
                "f2.late_rescue.apply",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                stage=str(stage),
                decision_source=str(decision_source),
                final_reason=str(final_reason),
                plan_type=str(plan_type),
                offer_action=str(offer_diag.get("offer_action", "") or ""),
                phase_before=int(before.get("phase", -1)),
                phase_after=int(after.get("phase", -1)),
                next_switch_before=float(before.get("next_switch", -1.0)),
                next_switch_after=float(after.get("next_switch", -1.0)),
                target_phase=int(target_phase),
                target_green_after=bool(target_green_after),
                speed_mps=float(speed_f),
                distance_to_stopline_m=float(dist_f),
                rescue_start_time=float(rescue.get("start_time", float(sim_time))),
                rescue_elapsed_s=float(float(sim_time) - float(rescue.get("start_time", float(sim_time)))),
                rescue_start_distance_m=float(rescue.get("start_distance_m", -1.0)),
                rescue_apply_count=int(apply_count),
                **route_ctx,
            )
        if rescue_key in _late_rescue_state and (target_green_after and speed_f > 1.0):
            rescue = dict(_late_rescue_state.pop(rescue_key, {}) or {})
            _fed_evt_main(
                "f2.late_rescue.clear",
                role="intersection",
                ev_id=str(ev_id),
                tls_id=str(tls_id),
                sim_time=float(sim_time),
                reason="target_green_and_moving",
                target_phase=int(target_phase),
                phase=int(after.get("phase", -1)),
                speed_mps=float(speed_f),
                distance_to_stopline_m=float(dist_f),
                rescue_start_time=float(rescue.get("start_time", float(sim_time))),
                rescue_elapsed_s=float(float(sim_time) - float(rescue.get("start_time", float(sim_time)))),
                **route_ctx,
            )
        f2_noop_signature = (
            str(stage),
            str(decision_source),
            str(final_reason),
            str(meta.get("refine_reason", "") or ""),
            str(plan_type),
            str(offer_diag.get("offer_action", "") or ""),
            int(before.get("phase", -1)),
            int(after.get("phase", -1)),
            int(target_phase),
            bool(target_green_after),
        )
        if not _should_emit_f2_apply_diag(
            key=rescue_key,
            sim_time=float(sim_time),
            effective_apply=bool(effective_apply),
            signature=f2_noop_signature,
        ):
            return
        _fed_dbg_main(
            f"evt=F2_APPLY stage={stage} source={decision_source} tls={tls_id} ev={ev_id} sim={float(sim_time):.2f} "
            f"final_reason={str(meta.get('final_reason', '')) or '-'} refine_reason={str(meta.get('refine_reason', '')) or '-'} "
            f"offer_id={offer_diag.get('offer_id') or '-'} action={offer_diag.get('offer_action') or '-'} "
            f"plan_type={plan_diag.get('plan_type') or '-'} target={plan_diag.get('plan_target_phase') or offer_diag.get('offer_target_phase') or '-'} "
            f"edge={ev_diag.get('ev_edge')} selected_edge={ev_diag.get('selected_in_edge')} "
            f"d_stop={float(d_stop):.2f} speed={ev_diag.get('speed_mps')} "
            f"before_phase={before.get('phase')} after_phase={after.get('phase')} "
            f"before_next_switch={float(before.get('next_switch', -1.0)):.2f} "
            f"after_next_switch={float(after.get('next_switch', -1.0)):.2f}"
        )
        _fed_evt_main(
            "f2.apply",
            role="intersection",
            stage=str(stage),
            decision_source=str(decision_source),
            ev_id=str(ev_id),
            tls_id=str(tls_id),
            sim_time=float(sim_time),
            final_reason=str(meta.get("final_reason", "") or ""),
            refine_reason=str(meta.get("refine_reason", "") or ""),
            selected_offer_source=str(meta.get("selected_source", meta.get("source", "")) or ""),
            before_phase=int(before.get("phase", -1)),
            after_phase=int(after.get("phase", -1)),
            before_program=str(before.get("program", "")),
            after_program=str(after.get("program", "")),
            before_state=str(before.get("state", "")),
            after_state=str(after.get("state", "")),
            before_next_switch=float(before.get("next_switch", -1.0)),
            after_next_switch=float(after.get("next_switch", -1.0)),
            before_next_switch_rem_s=float(before.get("next_switch_rem_s", -1.0)),
            after_next_switch_rem_s=float(after.get("next_switch_rem_s", -1.0)),
            target_phase=int(target_phase),
            target_green_after=bool(target_green_after),
            effective_apply=bool(effective_apply),
            **route_ctx,
            **offer_diag,
            **plan_diag,
            **ev_diag,
        )

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

    def _derive_passive_route_nodes(route_edges: Sequence[str]) -> List[Tuple[str, int]]:
        out: List[Tuple[str, int]] = []
        seen: set = set()
        for idx, edge_id in enumerate(list(route_edges or [])):
            edge = str(edge_id or "")
            if not edge or edge.startswith(":"):
                continue
            node = str(edge_to_to_node.get(edge, "") or "")
            if not node:
                continue
            if node_to_tls.get(node):
                continue
            if node in seen:
                continue
            seen.add(node)
            out.append((node, int(idx)))
        return out

    def _passive_observed_edges(route_edges: Sequence[str], idx: int, lookahead_edges: int) -> List[str]:
        edges = [str(e) for e in list(route_edges or []) if str(e) and not str(e).startswith(":")]
        if not edges:
            return []
        i = max(0, min(int(idx), len(edges) - 1))
        # Include the incoming edge to the passive junction and a small downstream window.
        end = min(len(edges), i + max(1, int(lookahead_edges or 1)))
        return list(edges[i:end])



    net_root = ET.parse(args.net_file).getroot()
    edge_to_from_node: Dict[str, str] = {}
    for e in net_root.findall("edge"):
        eid = e.get("id")
        if eid and not eid.startswith(":"):
            edge_to_to_node[eid] = e.get("to")
            edge_to_from_node[eid] = e.get("from")

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

    # MQTT. Keep the broker-visible id compact but leave protocol/transport at
    # Paho defaults to match the stable experiment path.
    mqtt_client_id = _short_mqtt_client_id(
        "rw",
        f"{getattr(args, 'mode', '')}:{getattr(args, 'emergency_veh', '')}:{getattr(args, 'mqtt_topic_namespace', '')}",
    )
    client = _make_mqtt_client(mqtt_client_id)
    client.connect(args.mqtt_host, int(getattr(args, "mqtt_port", 1883) or 1883), 60)

    mqtt_topic_namespace = str(getattr(args, "mqtt_topic_namespace", "") or "").strip().strip("/")
    mqtt_topic_ns_prefix = f"{mqtt_topic_namespace}/" if mqtt_topic_namespace else ""
    ev_request_topic_prefix_compat = str(
        getattr(args, "ev_request_topic_prefix", "federation/ev/request") or "federation/ev/request"
    ).rstrip("/")

    def _topic_out_ns(topic: str) -> str:
        t = str(topic or "").strip()
        if not t or not mqtt_topic_ns_prefix:
            return t
        if t.startswith(mqtt_topic_ns_prefix):
            return t
        return f"{mqtt_topic_ns_prefix}{t}"

    def _topic_in_ns(topic: str) -> Optional[str]:
        t = str(topic or "").strip()
        if not mqtt_topic_ns_prefix:
            return t
        if t.startswith(mqtt_topic_ns_prefix):
            return t[len(mqtt_topic_ns_prefix):]
        # EV requests are the bridge between the FNM sidecar and the SUMO
        # orchestrator. Accept the bare request topic as a compatibility path
        # so a namespace race/mismatch cannot silently degrade B1/F2 to B0.
        if t.startswith(f"{ev_request_topic_prefix_compat}/"):
            return t
        # Drone/F2D context can arrive from an edge FNM that preserves the
        # request namespace, but also support the bare canonical context topic
        # as a compatibility path for provider-side FNMs without a run namespace.
        if bool(getattr(args, "external_downstream_context_enable", False)) and t.startswith(
            "federation/v1/context/downstream/"
        ):
            return t
        return None

    # Apply namespace at transport edge so existing topic logic remains unchanged.
    _mqtt_publish_orig = client.publish
    _mqtt_subscribe_orig = client.subscribe

    def _mqtt_publish_ns(topic, payload=None, *pargs, **kwargs):
        logical = str(topic)
        wire = _topic_out_ns(logical)
        if logical.startswith("federation/ev/request/") or logical == "federation/ev/request":
            _fed_dbg_main(f"evt=MQTT_PUBLISH_MAP logical={logical} wire={wire} namespace={mqtt_topic_namespace or '-'}")
        return _mqtt_publish_orig(wire, payload, *pargs, **kwargs)

    def _mqtt_subscribe_ns(topic, *pargs, **kwargs):
        if isinstance(topic, list):
            out = []
            for item in topic:
                if isinstance(item, (tuple, list)) and item:
                    logical = str(item[0])
                    wire = _topic_out_ns(logical)
                    _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_MAP logical={logical} wire={wire} namespace={mqtt_topic_namespace or '-'}")
                    out.append((wire, *list(item[1:])))
                else:
                    logical = str(item)
                    wire = _topic_out_ns(logical)
                    _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_MAP logical={logical} wire={wire} namespace={mqtt_topic_namespace or '-'}")
                    out.append(wire)
            return _mqtt_subscribe_orig(out, *pargs, **kwargs)
        if isinstance(topic, tuple) and topic:
            logical = str(topic[0])
            wire = _topic_out_ns(logical)
            _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_MAP logical={logical} wire={wire} namespace={mqtt_topic_namespace or '-'}")
            mapped = (wire, *list(topic[1:]))
            return _mqtt_subscribe_orig(mapped, *pargs, **kwargs)
        logical = str(topic)
        wire = _topic_out_ns(logical)
        _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_MAP logical={logical} wire={wire} namespace={mqtt_topic_namespace or '-'}")
        return _mqtt_subscribe_orig(wire, *pargs, **kwargs)

    client.publish = _mqtt_publish_ns  # type: ignore[assignment]
    client.subscribe = _mqtt_subscribe_ns  # type: ignore[assignment]

    if mqtt_topic_namespace:
        print(f"[mqtt] topic namespace enabled: {mqtt_topic_namespace}")

    cmd_queue = deque()
    ev_request_det_apply_enabled = bool(getattr(args, "fed_ev_request_deterministic_apply_enable", False))
    ev_request_det_grace_sec = max(0.0, float(getattr(args, "fed_ev_request_deterministic_grace_sec", 0.25) or 0.0))
    ev_request_det_max_buffer_sec = max(
        ev_request_det_grace_sec,
        float(getattr(args, "fed_ev_request_deterministic_max_buffer_sec", 3.0) or 3.0),
    )
    ev_request_wait_for_fnm_enabled = bool(getattr(args, "fed_ev_request_wait_for_fnm_enable", False))
    ev_request_wait_for_fnm_timeout_sec = max(
        0.0,
        float(getattr(args, "fed_ev_request_wait_for_fnm_timeout_sec", 0.0) or 0.0),
    )
    ev_request_wait_for_fnm_poll_sec = max(
        0.001,
        float(getattr(args, "fed_ev_request_wait_for_fnm_poll_sec", 0.01) or 0.01),
    )
    ev_request_wait_for_fnm_retry_sim_sec = max(
        0.0,
        float(getattr(args, "fed_ev_request_wait_for_fnm_retry_sim_sec", 0.5) or 0.5),
    )
    ev_request_wait_for_fnm_raw_dispatch_enabled = bool(
        getattr(args, "fed_ev_request_wait_for_fnm_raw_dispatch_enable", False)
    )
    ev_request_pending_barrier: List[Dict[str, Any]] = []
    ev_request_barrier_seq = 0
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
    fed_bootstrap_discovery_target_filter_enable = bool(
        getattr(args, "federation_bootstrap_discovery_target_filter_enable", True)
    )
    fed_bootstrap_discovery_target_max_age_sec = max(
        1.0,
        float(getattr(args, "federation_bootstrap_discovery_target_max_age_sec", 20.0) or 20.0),
    )
    fed_bootstrap_discovery_target_role = str(
        getattr(args, "federation_bootstrap_discovery_target_role", "TrafficLightSystem")
        or "TrafficLightSystem"
    ).strip()
    fed_bootstrap_discovery_filter_modes = {
        str(x).strip().upper()
        for x in str(
            getattr(args, "federation_bootstrap_discovery_filter_modes", "F2")
            or "F2"
        ).split(",")
        if str(x).strip()
    }
    if not fed_bootstrap_discovery_filter_modes:
        fed_bootstrap_discovery_filter_modes = {"F2"}
    fed_bootstrap_discovery_fail_open = bool(
        getattr(args, "federation_bootstrap_discovery_fail_open", True)
    )
    fed_peer_selection_source = str(
        getattr(args, "federation_peer_selection_source", "realworld")
        or "realworld"
    ).strip().lower()
    if fed_peer_selection_source not in {"realworld", "fnm"}:
        fed_peer_selection_source = "realworld"
    fed_bootstrap_discovery_require_membership_valid = bool(
        getattr(args, "federation_bootstrap_discovery_require_membership_valid", True)
    )
    fed_bootstrap_active_member_statuses = {
        str(x).strip().upper()
        for x in str(
            getattr(args, "federation_bootstrap_active_member_statuses", "ACTIVE,REGISTERED,ALIVE")
            or "ACTIVE,REGISTERED,ALIVE"
        ).split(",")
        if str(x).strip()
    }
    if not fed_bootstrap_active_member_statuses:
        fed_bootstrap_active_member_statuses = {"ACTIVE", "REGISTERED", "ALIVE"}
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
    fed_bootstrap_cadence_mode = str(getattr(args, "federation_bootstrap_cadence_mode", "wall") or "wall").strip().lower()
    if fed_bootstrap_cadence_mode not in {"wall", "sim"}:
        fed_bootstrap_cadence_mode = "wall"
    fed_bootstrap_last_heartbeat_tick = 0.0
    fed_bootstrap_last_catalog_tick = 0.0
    fed_bootstrap_last_probe_tick = 0.0
    fed_bootstrap_probe_counter = 0
    fed_bootstrap_registered_gateways: set[str] = set()
    fed_bootstrap_member_status_by_gateway: Dict[str, str] = {}
    fed_bootstrap_discovery_req_sent_wall: Dict[str, float] = {}
    fed_bootstrap_discovery_tls_last_seen_wall: Dict[str, float] = {}
    fed_bootstrap_discovery_tls_gateway: Dict[str, str] = {}
    fed_bootstrap_discovery_tls_last_req_id: Dict[str, str] = {}
    

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

    passive_intersections: Dict[str, PassiveIntersectionDT] = {}
    passive_intersection_ids: List[str] = []
    passive_context_period_state: Dict[str, float] = {}
    if bool(getattr(args, "passive_intersection_dt_enable", False)):
        route_for_passive = list(ev_route_edges_cfg or [])
        passive_candidates = _derive_passive_route_nodes(route_for_passive)
        max_passive = int(getattr(args, "passive_intersection_max_nodes", 0) or 0)
        if max_passive > 0:
            passive_candidates = passive_candidates[:max_passive]
        look_edges = max(1, int(getattr(args, "passive_intersection_lookahead_edges", 3) or 3))
        for node_id, idx in passive_candidates:
            observed = _passive_observed_edges(route_for_passive, int(idx), int(look_edges))
            passive_intersections[str(node_id)] = PassiveIntersectionDT(
                node_id=str(node_id),
                observed_edges=list(observed),
                route_edges=list(route_for_passive),
                max_edges=int(look_edges),
            )
        passive_intersection_ids = sorted(passive_intersections.keys())
        print(
            f"[passive-dt] enabled route_non_tls_nodes={len(passive_candidates)} "
            f"instantiated={len(passive_intersection_ids)} lookahead_edges={look_edges}"
        )
        if passive_intersection_ids:
            print(f"[passive-dt] nodes(first20)={passive_intersection_ids[:20]}")
            _fed_dbg_main(
                f"evt=PASSIVE_DT_INIT enabled=1 nodes={len(passive_intersection_ids)} "
                f"lookahead_edges={look_edges} route_edges={len(route_for_passive)}"
            )

    def _passive_context_fanout_tls(payload: dict) -> List[str]:
        """Return active TLS agents near the passive observation's route segment."""
        if not bool(getattr(args, "passive_intersection_context_route_fanout_enable", True)):
            return sorted(str(k) for k in agents.keys())
        route_edges = [str(e) for e in list(ev_route_edges_cfg or []) if str(e)]
        if not route_edges:
            return sorted(str(k) for k in agents.keys())
        target_edges = [
            str(e)
            for e in list((payload or {}).get("target_edges", (payload or {}).get("lookahead_edges", [])) or [])
            if str(e)
        ]
        idxs: List[int] = []
        for edge_id in target_edges:
            try:
                idxs.append(route_edges.index(str(edge_id)))
            except ValueError:
                continue
        if not idxs:
            return sorted(str(k) for k in agents.keys())
        back_n = max(0, int(getattr(args, "passive_intersection_context_fanout_back_edges", 2) or 2))
        forward_n = max(0, int(getattr(args, "passive_intersection_context_fanout_forward_edges", 4) or 4))
        lo = max(0, min(idxs) - int(back_n))
        hi = min(len(route_edges), max(idxs) + int(forward_n) + 1)
        fanout: List[str] = []
        for edge_id in route_edges[lo:hi]:
            tls_id = str(edge_to_tls.get(str(edge_id), "") or "")
            if tls_id and tls_id in agents and tls_id not in fanout:
                fanout.append(tls_id)
        if fanout:
            return sorted(fanout)
        return sorted(str(k) for k in agents.keys())

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
        cfg.dt_mode = "active_tls"
        cfg.can_actuate = True
        cfg.can_coordinate = True
        cfg.can_observe_downstream = True
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
        # Event-stream metadata for downstream run-level traceability.
        cfg.fed_topic_namespace = str(mqtt_topic_namespace or "")
        _run_tok = str(mqtt_topic_namespace or "").strip() or str(os.path.basename(str(args.sumo_cfg or "")) or "")
        cfg.fed_run_id = f"{str(CURRENT_EVALUATION)}:{_run_tok}" if _run_tok else str(CURRENT_EVALUATION)
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
        cfg.f2_measured_override_min_robust_improvement = float(
            getattr(args, "f2_measured_override_min_robust_improvement", 0.0) or 0.0
        )
        cfg.f2_measured_override_min_ev_wait_improvement_sec = float(
            getattr(args, "f2_measured_override_min_ev_wait_improvement_sec", 0.0) or 0.0
        )
        cfg.f2_measured_override_min_ev_miss_improvement_sec = float(
            getattr(args, "f2_measured_override_min_ev_miss_improvement_sec", 0.0) or 0.0
        )
        cfg.f2_block_infeasible_actuation = bool(getattr(args, "f2_block_infeasible_actuation", True))
        cfg.f2_refine_require_feedback = bool(getattr(args, "f2_refine_require_feedback", True))
        cfg.f2_refine_feedback_max_age_sec = float(getattr(args, "f2_refine_feedback_max_age_sec", 6.0))
        cfg.f2_refine_feedback_age_adaptive_enable = bool(
            getattr(args, "f2_refine_feedback_age_adaptive_enable", True)
        )
        cfg.f2_refine_feedback_max_age_near_sec = float(
            getattr(args, "f2_refine_feedback_max_age_near_sec", -1.0)
        )
        cfg.f2_refine_feedback_max_age_far_sec = float(
            getattr(args, "f2_refine_feedback_max_age_far_sec", -1.0)
        )
        cfg.f2_refine_feedback_adaptive_far_distance_m = float(
            getattr(args, "f2_refine_feedback_adaptive_far_distance_m", 250.0)
        )
        cfg.f2_refine_feedback_bootstrap_enable = bool(
            getattr(args, "f2_refine_feedback_bootstrap_enable", True)
        )
        cfg.f2_refine_feedback_bootstrap_distance_m = float(
            getattr(args, "f2_refine_feedback_bootstrap_distance_m", 450.0)
        )
        cfg.f2_refine_feedback_bootstrap_max_age_sec = float(
            getattr(args, "f2_refine_feedback_bootstrap_max_age_sec", 20.0)
        )
        cfg.f2_refine_stale_feedback_gate_enable = bool(
            getattr(args, "f2_refine_stale_feedback_gate_enable", True)
        )
        cfg.f2_refine_max_responder_phase_state_age_ms = float(
            getattr(args, "f2_refine_max_responder_phase_state_age_ms", 4000.0)
        )
        cfg.f2_refine_near_distance_m = float(getattr(args, "f2_refine_near_distance_m", 40.0))
        cfg.f2_refine_near_max_responder_phase_state_age_ms = float(
            getattr(args, "f2_refine_near_max_responder_phase_state_age_ms", 1200.0)
        )
        cfg.f2_refine_require_preferred_feedback_when_near = bool(
            getattr(args, "f2_refine_require_preferred_feedback_when_near", True)
        )
        cfg.f2_refine_preferred_feedback_near_distance_m = float(
            getattr(args, "f2_refine_preferred_feedback_near_distance_m", 60.0)
        )
        cfg.f2_refine_neighbor_state_fallback_enable = bool(
            getattr(args, "f2_refine_neighbor_state_fallback_enable", True)
        )
        cfg.f2_refine_neighbor_state_max_age_sec = float(
            getattr(args, "f2_refine_neighbor_state_max_age_sec", 4.0)
        )
        cfg.f2_refine_neighbor_state_near_max_age_sec = float(
            getattr(args, "f2_refine_neighbor_state_near_max_age_sec", 1.5)
        )
        cfg.f2_refine_require_loop_coverage = bool(getattr(args, "f2_refine_require_loop_coverage", True))
        cfg.f2_refine_min_loop_coverage_ratio = float(getattr(args, "f2_refine_min_loop_coverage_ratio", 0.5))
        cfg.f2_skip_redundant_apply = bool(getattr(args, "f2_skip_redundant_apply", True))
        cfg.f2_skip_redundant_apply_min_interval_sec = float(
            getattr(args, "f2_skip_redundant_apply_min_interval_sec", 0.8)
        )
        cfg.f2_skip_redundant_apply_min_interval_near_sec = float(
            getattr(args, "f2_skip_redundant_apply_min_interval_near_sec", cfg.f2_skip_redundant_apply_min_interval_sec)
        )
        cfg.f2_skip_redundant_apply_min_interval_far_sec = float(
            getattr(args, "f2_skip_redundant_apply_min_interval_far_sec", cfg.f2_skip_redundant_apply_min_interval_sec)
        )
        cfg.f2_skip_redundant_apply_near_distance_m = float(
            getattr(args, "f2_skip_redundant_apply_near_distance_m", 120.0)
        )
        cfg.f2_skip_redundant_apply_far_distance_m = float(
            getattr(args, "f2_skip_redundant_apply_far_distance_m", 300.0)
        )
        cfg.f2_offer_preapply_dedupe_min_interval_sec = float(
            getattr(args, "f2_offer_preapply_dedupe_min_interval_sec", 2.0)
        )
        cfg.f2_offer_preapply_dedupe_min_interval_near_sec = float(
            getattr(
                args,
                "f2_offer_preapply_dedupe_min_interval_near_sec",
                cfg.f2_offer_preapply_dedupe_min_interval_sec,
            )
        )
        cfg.f2_offer_preapply_dedupe_min_interval_far_sec = float(
            getattr(
                args,
                "f2_offer_preapply_dedupe_min_interval_far_sec",
                cfg.f2_offer_preapply_dedupe_min_interval_sec,
            )
        )
        cfg.f2_offer_preapply_dedupe_near_distance_m = float(
            getattr(args, "f2_offer_preapply_dedupe_near_distance_m", 120.0)
        )
        cfg.f2_offer_preapply_dedupe_far_distance_m = float(
            getattr(args, "f2_offer_preapply_dedupe_far_distance_m", 300.0)
        )
        cfg.f2_selected_offer_min_effective_extend_sec = float(
            getattr(args, "f2_selected_offer_min_effective_extend_sec", 0.5)
        )
        cfg.f2_selected_offer_recompute_local_fallback = bool(
            getattr(args, "f2_selected_offer_recompute_local_fallback", True)
        )
        cfg.f2_lookahead_upstream_stopped_rescue_enable = bool(
            getattr(args, "f2_lookahead_upstream_stopped_rescue_enable", True)
        )
        cfg.f2_lookahead_upstream_stopped_rescue_max_distance_m = float(
            getattr(args, "f2_lookahead_upstream_stopped_rescue_max_distance_m", 120.0)
        )
        cfg.f2_lookahead_upstream_stopped_rescue_max_hops = int(
            getattr(args, "f2_lookahead_upstream_stopped_rescue_max_hops", 1)
        )
        cfg.f2_lookahead_upstream_stopped_rescue_extension_sec = float(
            getattr(args, "f2_lookahead_upstream_stopped_rescue_extension_sec", 4.0)
        )
        cfg.f2_weak_offer_last_local_fallback_enable = bool(
            getattr(args, "f2_weak_offer_last_local_fallback_enable", True)
        )
        cfg.f2_weak_offer_last_local_fallback_max_age_sec = float(
            getattr(args, "f2_weak_offer_last_local_fallback_max_age_sec", 20.0)
        )
        cfg.f2_active_coord_window_relax_enable = bool(
            getattr(args, "f2_active_coord_window_relax_enable", False)
        )
        cfg.f2_active_coord_window_recent_sec = float(
            getattr(args, "f2_active_coord_window_recent_sec", 2.5)
        )
        cfg.f2_active_coord_window_ev_near_m = float(
            getattr(args, "f2_active_coord_window_ev_near_m", 180.0)
        )
        cfg.f2_active_coord_window_min_active_reservations = int(
            getattr(args, "f2_active_coord_window_min_active_reservations", 1)
        )
        cfg.f2_active_coord_window_interval_scale = float(
            getattr(args, "f2_active_coord_window_interval_scale", 0.5)
        )
        cfg.f2_refine_local_cooldown_enable = bool(
            getattr(args, "f2_refine_local_cooldown_enable", True)
        )
        cfg.f2_refine_local_cooldown_trigger_count = int(
            getattr(args, "f2_refine_local_cooldown_trigger_count", 3)
        )
        cfg.f2_refine_local_cooldown_window_sec = float(
            getattr(args, "f2_refine_local_cooldown_window_sec", 2.5)
        )
        cfg.f2_refine_local_cooldown_near_distance_m = float(
            getattr(args, "f2_refine_local_cooldown_near_distance_m", -1.0)
        )
        cfg.f2_usefulness_gate_enable = bool(getattr(args, "f2_usefulness_gate_enable", True))
        cfg.f2_usefulness_gate_skip_streak_trigger = int(
            getattr(args, "f2_usefulness_gate_skip_streak_trigger", 6)
        )
        cfg.f2_usefulness_gate_hold_sec = float(getattr(args, "f2_usefulness_gate_hold_sec", 3.0))
        cfg.f2_usefulness_gate_near_only = bool(getattr(args, "f2_usefulness_gate_near_only", True))
        cfg.f2_usefulness_gate_near_distance_m = float(
            getattr(args, "f2_usefulness_gate_near_distance_m", 150.0)
        )
        cfg.f2_usefulness_gate_require_no_hard_accept = bool(
            getattr(args, "f2_usefulness_gate_require_no_hard_accept", True)
        )
        cfg.f2_usefulness_gate_failsoft_local = bool(
            getattr(args, "f2_usefulness_gate_failsoft_local", True)
        )
        cfg.f2_drone_context_request_enable = bool(
            getattr(args, "f2_drone_context_request_enable", False)
        )
        cfg.f2_drone_context_provider_id = str(
            getattr(args, "f2_drone_context_provider_id", "crazyflie_01") or "crazyflie_01"
        )
        cfg.f2_drone_context_request_ttl_sec = float(
            getattr(args, "f2_drone_context_request_ttl_sec", 3.0)
        )
        cfg.f2_drone_context_request_min_interval_sec = float(
            getattr(args, "f2_drone_context_request_min_interval_sec", 3.0)
        )
        cfg.f2_drone_context_request_max_edges = int(
            getattr(args, "f2_drone_context_request_max_edges", 8)
        )
        cfg.f2_drone_context_include_route_context = bool(
            getattr(args, "f2_drone_context_include_route_context", True)
        )
        cfg.f2_drone_context_route_context_max_edges = int(
            getattr(args, "f2_drone_context_route_context_max_edges", 64) or 64
        )
        cfg.f2_drone_context_emit_discovery_query = bool(
            getattr(args, "f2_drone_context_emit_discovery_query", True)
        )
        cfg.f2_drone_context_discovery_gate_enable = bool(
            getattr(args, "f2_drone_context_discovery_gate_enable", False)
            and _is_drone_augmented_mode(CURRENT_EVALUATION)
        )
        cfg.f2_drone_context_discovery_cache_ttl_sec = float(
            getattr(args, "f2_drone_context_discovery_cache_ttl_sec", 5.0) or 5.0
        )
        cfg.f2_drone_context_discovery_query_min_interval_sec = float(
            getattr(args, "f2_drone_context_discovery_query_min_interval_sec", 1.0) or 1.0
        )
        cfg.f2d_drone_prescout_enable = bool(
            getattr(args, "f2d_drone_prescout_enable", False)
            and _is_f2d_prescout_mode(CURRENT_EVALUATION)
        )
        cfg.f2d_drone_prescout_first_tls_only = bool(
            getattr(args, "f2d_drone_prescout_first_tls_only", True)
        )
        cfg.f2d_drone_prescout_max_edges = int(
            getattr(args, "f2d_drone_prescout_max_edges", 16) or 16
        )
        cfg.f2d_drone_prescout_min_interval_sec = float(
            getattr(args, "f2d_drone_prescout_min_interval_sec", 30.0) or 30.0
        )
        cfg.f2p_passive_context_policy = str(
            getattr(args, "f2p_passive_context_policy", "severe_or_missing") or "severe_or_missing"
        )
        cfg.f2p_passive_context_max_age_sec = float(
            getattr(args, "f2p_passive_context_max_age_sec", 5.0) or 5.0
        )
        cfg.f2p_passive_context_lookahead_edges = int(
            getattr(args, "f2p_passive_context_lookahead_edges", 4) or 4
        )
        cfg.f2p_passive_context_max_worst_edge_offset = int(
            getattr(args, "f2p_passive_context_max_worst_edge_offset", 1) or 1
        )
        cfg.f2p_passive_context_severe_min_halt_n = int(
            getattr(args, "f2p_passive_context_severe_min_halt_n", 4) or 4
        )
        cfg.f2p_passive_context_severe_min_veh_n = int(
            getattr(args, "f2p_passive_context_severe_min_veh_n", 6) or 6
        )
        cfg.f2p_passive_context_severe_max_mean_speed_mps = float(
            getattr(args, "f2p_passive_context_severe_max_mean_speed_mps", 0.5) or 0.5
        )
        cfg.f2p_passive_context_severe_max_occupancy_pct = float(
            getattr(args, "f2p_passive_context_severe_max_occupancy_pct", 45.0) or 45.0
        )
        cfg.f2p_passive_context_missing_feedback_floor_enable = bool(
            getattr(args, "f2p_passive_context_missing_feedback_floor_enable", True)
        )
        cfg.f2p_passive_context_missing_feedback_max_queue_deficit_sec = float(
            getattr(args, "f2p_passive_context_missing_feedback_max_queue_deficit_sec", 2.0) or 2.0
        )
        cfg.f2p_passive_context_missing_feedback_max_spillback_risk = float(
            getattr(args, "f2p_passive_context_missing_feedback_max_spillback_risk", 0.15) or 0.15
        )
        cfg.f2p_passive_context_missing_feedback_max_timing_sec = float(
            getattr(args, "f2p_passive_context_missing_feedback_max_timing_sec", 1.0) or 1.0
        )
        cfg.f2p_passive_context_clear_missing_feedback_enable = bool(
            getattr(args, "f2p_passive_context_clear_missing_feedback_enable", True)
        )
        cfg.f2p_passive_context_clear_missing_feedback_no_feedback_penalty = float(
            getattr(args, "f2p_passive_context_clear_missing_feedback_no_feedback_penalty", 0.25) or 0.25
        )
        cfg.f2p_passive_stall_rescue_enable = bool(
            getattr(args, "f2p_passive_stall_rescue_enable", True)
        )
        cfg.f2p_passive_stall_rescue_min_blocked_sec = float(
            getattr(args, "f2p_passive_stall_rescue_min_blocked_sec", 6.0) or 6.0
        )
        cfg.f2p_passive_stall_rescue_max_speed_mps = float(
            getattr(args, "f2p_passive_stall_rescue_max_speed_mps", 0.5) or 0.5
        )
        cfg.f2p_passive_stall_rescue_require_selected_edge = bool(
            getattr(args, "f2p_passive_stall_rescue_require_selected_edge", True)
        )
        cfg.fed_req_send_min_gap_sec = float(getattr(args, "fed_req_send_min_gap_sec", 0.60))
        cfg.fed_req_send_min_gap_near_sec = float(
            getattr(args, "fed_req_send_min_gap_near_sec", cfg.fed_req_send_min_gap_sec)
        )
        cfg.fed_req_send_min_gap_far_sec = float(
            getattr(args, "fed_req_send_min_gap_far_sec", cfg.fed_req_send_min_gap_sec)
        )
        cfg.fed_req_send_min_gap_near_distance_m = float(
            getattr(args, "fed_req_send_min_gap_near_distance_m", 120.0)
        )
        cfg.fed_req_send_min_gap_far_distance_m = float(
            getattr(args, "fed_req_send_min_gap_far_distance_m", 300.0)
        )
        cfg.fed_req_pending_per_peer_cap = int(getattr(args, "fed_req_pending_per_peer_cap", 2))
        cfg.fed_req_pending_stale_sec = float(getattr(args, "fed_req_pending_stale_sec", 6.0))
        cfg.fed_min_hard_overlap_sec = float(getattr(args, "fed_min_hard_overlap_sec", 0.50))
        cfg.fed_hard_overlap_grace_sec = float(getattr(args, "fed_hard_overlap_grace_sec", 0.80))
        cfg.fed_soft_window_grace_sec = float(getattr(args, "fed_soft_window_grace_sec", 6.0))
        cfg.fed_hard_window_adaptive_relax_enable = bool(
            getattr(args, "fed_hard_window_adaptive_relax_enable", False)
        )
        cfg.fed_hard_window_adaptive_extra_grace_sec = float(
            getattr(args, "fed_hard_window_adaptive_extra_grace_sec", 0.6)
        )
        cfg.fed_hard_window_adaptive_conf_min = float(
            getattr(args, "fed_hard_window_adaptive_conf_min", 0.65)
        )
        cfg.fed_hard_window_adaptive_readiness_min = float(
            getattr(args, "fed_hard_window_adaptive_readiness_min", 0.55)
        )
        cfg.fed_hard_window_adaptive_spillback_max = float(
            getattr(args, "fed_hard_window_adaptive_spillback_max", 0.80)
        )
        cfg.fed_hard_window_adaptive_queue_margin_min_sec = float(
            getattr(args, "fed_hard_window_adaptive_queue_margin_min_sec", -1.5)
        )
        _fed_dbg_main(
            f"evt=CFG_WARMUP tls={tls_id} warmup={1 if cfg.fed_enable_warmup else 0} "
            f"hard_only={1 if cfg.fed_warmup_hard_only else 0} period={float(cfg.fed_warmup_period_sec):.2f} "
            f"horizon={float(cfg.fed_warm_horizon_sec):.2f} "
            f"q_margin_min={float(cfg.fed_hard_min_queue_margin_sec):.2f} "
            f"spill_max={float(cfg.fed_hard_max_spillback_risk):.2f} "
            f"readiness_improved={1 if cfg.fed_readiness_use_improved_queue else 0}"
        )
        _fed_dbg_main(
            f"evt=CFG_B1_POLICY tls={tls_id} "
            f"strict_local={1 if b1_strict_local_baseline else 0} "
            f"downstream_guard={1 if bool(getattr(args, 'b1_downstream_blockage_guard_enable', False)) else 0} "
            f"downstream_edges={int(getattr(args, 'b1_downstream_blockage_lookahead_edges', 0) or 0)} "
            f"lookahead_guard={1 if bool(getattr(args, 'b1_lookahead_actuation_guard_enable', True)) else 0} "
            f"fnm_request_wait={1 if bool(getattr(args, 'fed_ev_request_wait_for_fnm_enable', False)) else 0} "
            f"fnm_wait_timeout={float(getattr(args, 'fed_ev_request_wait_for_fnm_timeout_sec', 0.0) or 0.0):.2f}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_GUARD tls={tls_id} enabled={1 if cfg.f2_ev_guard_enable else 0} "
            f"wait_eps={float(cfg.f2_ev_guard_wait_penalty_sec):.2f} "
            f"miss_eps={float(cfg.f2_ev_guard_miss_penalty_sec):.2f} "
            f"require_feasible={1 if cfg.f2_ev_guard_require_feasible else 0}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_POLICY tls={tls_id} policy={str(cfg.f2_selection_policy)} "
            f"measured_min_robust_improvement={float(cfg.f2_measured_override_min_robust_improvement):.3f} "
            f"measured_min_wait_improvement={float(cfg.f2_measured_override_min_ev_wait_improvement_sec):.3f} "
            f"measured_min_miss_improvement={float(cfg.f2_measured_override_min_ev_miss_improvement_sec):.3f} "
            f"strict_b1_floor_peer_override_only={1 if bool(getattr(args, 'f2_strict_b1_floor_peer_override_only', True)) else 0} "
            f"block_infeasible={1 if cfg.f2_block_infeasible_actuation else 0} "
            f"require_feedback={1 if cfg.f2_refine_require_feedback else 0} "
            f"feedback_age_max={float(cfg.f2_refine_feedback_max_age_sec):.2f} "
            f"bootstrap_feedback={1 if cfg.f2_refine_feedback_bootstrap_enable else 0} "
            f"bootstrap_dist_m={float(cfg.f2_refine_feedback_bootstrap_distance_m):.1f} "
            f"bootstrap_age_s={float(cfg.f2_refine_feedback_bootstrap_max_age_sec):.1f} "
            f"stale_gate={1 if cfg.f2_refine_stale_feedback_gate_enable else 0} "
            f"stale_age_max_ms={float(cfg.f2_refine_max_responder_phase_state_age_ms):.0f} "
            f"near_dist_m={float(cfg.f2_refine_near_distance_m):.1f} "
            f"near_stale_age_max_ms={float(cfg.f2_refine_near_max_responder_phase_state_age_ms):.0f} "
            f"require_pref_near={1 if cfg.f2_refine_require_preferred_feedback_when_near else 0} "
            f"pref_near_dist_m={float(cfg.f2_refine_preferred_feedback_near_distance_m):.1f} "
            f"require_loop_cov={1 if cfg.f2_refine_require_loop_coverage else 0} "
            f"loop_cov_min={float(cfg.f2_refine_min_loop_coverage_ratio):.2f} "
            f"skip_redundant_apply={1 if cfg.f2_skip_redundant_apply else 0} "
            f"skip_redundant_apply_min={float(cfg.f2_skip_redundant_apply_min_interval_sec):.2f} "
            f"skip_redundant_apply_near={float(cfg.f2_skip_redundant_apply_min_interval_near_sec):.2f} "
            f"skip_redundant_apply_far={float(cfg.f2_skip_redundant_apply_min_interval_far_sec):.2f} "
            f"skip_redundant_apply_near_m={float(cfg.f2_skip_redundant_apply_near_distance_m):.1f} "
            f"skip_redundant_apply_far_m={float(cfg.f2_skip_redundant_apply_far_distance_m):.1f} "
            f"preapply_min={float(cfg.f2_offer_preapply_dedupe_min_interval_sec):.2f} "
            f"f2p_passive_policy={str(cfg.f2p_passive_context_policy)} "
            f"f2p_passive_age_s={float(cfg.f2p_passive_context_max_age_sec):.1f} "
            f"f2p_passive_edges={int(cfg.f2p_passive_context_lookahead_edges)} "
            f"f2p_passive_max_offset={int(cfg.f2p_passive_context_max_worst_edge_offset)} "
            f"f2p_active_tls_metering_floor={1 if bool(getattr(args, 'f2p_active_tls_metering_floor_enable', True)) else 0} "
            f"f2p_active_tls_metering_floor_max_offset={int(getattr(args, 'f2p_active_tls_metering_floor_max_worst_edge_offset', 1) or 1)} "
            f"f2p_passive_severe_halt={int(cfg.f2p_passive_context_severe_min_halt_n)} "
            f"f2p_passive_severe_veh={int(cfg.f2p_passive_context_severe_min_veh_n)} "
            f"f2p_passive_severe_speed={float(cfg.f2p_passive_context_severe_max_mean_speed_mps):.2f} "
            f"f2p_passive_severe_occ={float(cfg.f2p_passive_context_severe_max_occupancy_pct):.1f} "
            f"f2p_passive_missing_floor={1 if bool(cfg.f2p_passive_context_missing_feedback_floor_enable) else 0} "
            f"f2p_passive_missing_max_q_def={float(cfg.f2p_passive_context_missing_feedback_max_queue_deficit_sec):.1f} "
            f"f2p_passive_missing_max_spill={float(cfg.f2p_passive_context_missing_feedback_max_spillback_risk):.2f} "
            f"f2p_passive_missing_max_timing={float(cfg.f2p_passive_context_missing_feedback_max_timing_sec):.1f} "
            f"f2p_passive_clear_missing={1 if bool(cfg.f2p_passive_context_clear_missing_feedback_enable) else 0} "
            f"f2p_passive_clear_no_fb_pen={float(cfg.f2p_passive_context_clear_missing_feedback_no_feedback_penalty):.2f} "
            f"f2p_passive_stall_rescue={1 if bool(cfg.f2p_passive_stall_rescue_enable) else 0} "
            f"f2p_passive_stall_rescue_min_s={float(cfg.f2p_passive_stall_rescue_min_blocked_sec):.1f} "
            f"f2p_passive_stall_rescue_max_speed={float(cfg.f2p_passive_stall_rescue_max_speed_mps):.2f} "
            f"f2p_passive_stall_rescue_req_selected={1 if bool(cfg.f2p_passive_stall_rescue_require_selected_edge) else 0} "
            f"f2p_queue_release={1 if _f2p_queue_release_enabled() else 0} "
            f"f2p_queue_release_hold_s={float(getattr(args, 'f2p_queue_release_hold_sec', 3.0) or 3.0):.2f} "
            f"f2p_queue_release_min_interval_s={float(getattr(args, 'f2p_queue_release_min_interval_sec', 3.0) or 3.0):.2f} "
            f"f2p_queue_release_max_offset={int(getattr(args, 'f2p_queue_release_max_worst_edge_offset', 4) or 4)} "
            f"passive_route_fanout={1 if bool(getattr(args, 'passive_intersection_context_route_fanout_enable', True)) else 0} "
            f"passive_fanout_back_edges={int(getattr(args, 'passive_intersection_context_fanout_back_edges', 2) or 2)} "
            f"passive_fanout_forward_edges={int(getattr(args, 'passive_intersection_context_fanout_forward_edges', 4) or 4)} "
            f"preapply_near={float(cfg.f2_offer_preapply_dedupe_min_interval_near_sec):.2f} "
            f"preapply_far={float(cfg.f2_offer_preapply_dedupe_min_interval_far_sec):.2f} "
            f"selected_offer_min_effective_extend={float(getattr(cfg, 'f2_selected_offer_min_effective_extend_sec', 0.5)):.2f} "
            f"selected_offer_recompute_local_fallback={1 if cfg.f2_selected_offer_recompute_local_fallback else 0} "
            f"upstream_stopped_rescue={1 if cfg.f2_lookahead_upstream_stopped_rescue_enable else 0} "
            f"upstream_stopped_rescue_max_m={float(cfg.f2_lookahead_upstream_stopped_rescue_max_distance_m):.1f} "
            f"upstream_stopped_rescue_max_hops={int(cfg.f2_lookahead_upstream_stopped_rescue_max_hops)} "
            f"upstream_stopped_rescue_ext={float(cfg.f2_lookahead_upstream_stopped_rescue_extension_sec):.2f} "
            f"weak_offer_last_local_fallback={1 if cfg.f2_weak_offer_last_local_fallback_enable else 0} "
            f"weak_offer_last_local_fallback_max_age={float(cfg.f2_weak_offer_last_local_fallback_max_age_sec):.2f} "
            f"approach_phase_rescue={1 if bool(getattr(args, 'f2_approach_phase_rescue_enable', True)) else 0} "
            f"approach_phase_rescue_max_m={float(getattr(args, 'f2_approach_phase_rescue_max_distance_m', 120.0) or 120.0):.1f} "
            f"approach_phase_rescue_max_speed={float(getattr(args, 'f2_approach_phase_rescue_max_speed_mps', 14.5) or 14.5):.1f} "
            f"approach_phase_rescue_blocked_only={1 if bool(getattr(args, 'f2_approach_phase_rescue_blocked_only', True)) else 0} "
            f"approach_phase_rescue_current_edge={1 if bool(getattr(args, 'f2_approach_phase_rescue_require_current_edge', True)) else 0} "
            f"current_tls_stopped_rescue={1 if bool(getattr(args, 'f2_current_tls_stopped_rescue_enable', True)) else 0} "
            f"current_tls_stopped_rescue_max_m={float(getattr(args, 'f2_current_tls_stopped_rescue_max_distance_m', 80.0) or 80.0):.1f} "
            f"current_tls_stopped_rescue_max_speed={float(getattr(args, 'f2_current_tls_stopped_rescue_max_speed_mps', 2.0) or 2.0):.1f} "
            f"current_tls_stopped_rescue_max_hops={int(getattr(args, 'f2_current_tls_stopped_rescue_max_lookahead_hops', 2) or 2)} "
            f"current_tls_stopped_rescue_min_interval={float(getattr(args, 'f2_current_tls_stopped_rescue_min_interval_sec', 6.0) or 6.0):.1f} "
            f"active_coord_relax={1 if cfg.f2_active_coord_window_relax_enable else 0} "
            f"active_coord_recent_s={float(cfg.f2_active_coord_window_recent_sec):.2f} "
            f"active_coord_near_m={float(cfg.f2_active_coord_window_ev_near_m):.1f} "
            f"active_coord_scale={float(cfg.f2_active_coord_window_interval_scale):.2f} "
            f"local_cooldown={1 if cfg.f2_refine_local_cooldown_enable else 0} "
            f"local_cooldown_trigger={int(cfg.f2_refine_local_cooldown_trigger_count)} "
            f"local_cooldown_window={float(cfg.f2_refine_local_cooldown_window_sec):.2f} "
            f"local_cooldown_near_m={float(cfg.f2_refine_local_cooldown_near_distance_m):.1f} "
            f"usefulness_gate={1 if cfg.f2_usefulness_gate_enable else 0} "
            f"usefulness_trigger={int(cfg.f2_usefulness_gate_skip_streak_trigger)} "
            f"usefulness_hold_s={float(cfg.f2_usefulness_gate_hold_sec):.2f} "
            f"usefulness_near_only={1 if cfg.f2_usefulness_gate_near_only else 0} "
            f"usefulness_near_m={float(cfg.f2_usefulness_gate_near_distance_m):.1f} "
            f"usefulness_require_no_accept={1 if cfg.f2_usefulness_gate_require_no_hard_accept else 0} "
            f"usefulness_failsoft_local={1 if cfg.f2_usefulness_gate_failsoft_local else 0} "
            f"drone_request={1 if cfg.f2_drone_context_request_enable else 0} "
            f"drone_provider={str(cfg.f2_drone_context_provider_id)} "
            f"drone_ttl_s={float(cfg.f2_drone_context_request_ttl_sec):.2f} "
            f"drone_min_interval_s={float(cfg.f2_drone_context_request_min_interval_sec):.2f} "
            f"drone_max_edges={int(cfg.f2_drone_context_request_max_edges)} "
            f"drone_route_context={1 if bool(cfg.f2_drone_context_include_route_context) else 0} "
            f"drone_route_context_max_edges={int(cfg.f2_drone_context_route_context_max_edges)} "
            f"drone_discovery_query={1 if cfg.f2_drone_context_emit_discovery_query else 0} "
            f"drone_discovery_gate={1 if bool(getattr(cfg, 'f2_drone_context_discovery_gate_enable', False)) else 0} "
            f"drone_discovery_cache_ttl_s={float(getattr(cfg, 'f2_drone_context_discovery_cache_ttl_sec', 5.0)):.2f} "
            f"f2d_drone_prescout={1 if cfg.f2d_drone_prescout_enable else 0} "
            f"f2d_drone_prescout_first_tls_only={1 if cfg.f2d_drone_prescout_first_tls_only else 0} "
            f"f2d_drone_prescout_max_edges={int(cfg.f2d_drone_prescout_max_edges)} "
            f"f2d_drone_prescout_min_interval_s={float(cfg.f2d_drone_prescout_min_interval_sec):.2f} "
            f"external_downstream_context={1 if bool(getattr(args, 'external_downstream_context_enable', False)) else 0} "
            f"external_downstream_context_max_age_s={float(getattr(args, 'external_downstream_context_max_age_sec', 2.0) or 2.0):.2f} "
            f"f2d_directed_context_delivery={1 if _f2d_directed_context_delivery_enabled() else 0} "
            f"f2d_directed_context_self_delivery={1 if bool(getattr(args, 'f2d_directed_context_self_delivery_enable', False)) else 0} "
            f"f2d_contextual_topic_delivery={1 if _f2d_contextual_topic_delivery_enabled() else 0} "
            f"f2d_queue_release={1 if _f2d_queue_release_enabled() else 0} "
            f"f2d_queue_release_hold_s={float(getattr(args, 'f2d_queue_release_hold_sec', 3.0) or 3.0):.2f} "
            f"f2d_queue_release_min_interval_s={float(getattr(args, 'f2d_queue_release_min_interval_sec', 3.0) or 3.0):.2f} "
            f"f2d_queue_release_max_offset={int(getattr(args, 'f2d_queue_release_max_worst_edge_offset', 8) or 8)}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_CHURN tls={tls_id} req_gap_s={float(cfg.fed_req_send_min_gap_sec):.2f} "
            f"req_gap_near_s={float(cfg.fed_req_send_min_gap_near_sec):.2f} "
            f"req_gap_far_s={float(cfg.fed_req_send_min_gap_far_sec):.2f} "
            f"req_gap_near_m={float(cfg.fed_req_send_min_gap_near_distance_m):.1f} "
            f"req_gap_far_m={float(cfg.fed_req_send_min_gap_far_distance_m):.1f} "
            f"pending_cap={int(cfg.fed_req_pending_per_peer_cap)} "
            f"pending_stale_s={float(cfg.fed_req_pending_stale_sec):.1f}"
        )
        _fed_dbg_main(
            f"evt=CFG_F2_FEAS tls={tls_id} hard_min_overlap_s={float(cfg.fed_min_hard_overlap_sec):.2f} "
            f"hard_grace_s={float(cfg.fed_hard_overlap_grace_sec):.2f} "
            f"soft_window_grace_s={float(cfg.fed_soft_window_grace_sec):.2f} "
            f"hard_adapt={1 if cfg.fed_hard_window_adaptive_relax_enable else 0} "
            f"hard_adapt_extra_grace_s={float(cfg.fed_hard_window_adaptive_extra_grace_sec):.2f} "
            f"hard_adapt_conf_min={float(cfg.fed_hard_window_adaptive_conf_min):.2f} "
            f"hard_adapt_ready_min={float(cfg.fed_hard_window_adaptive_readiness_min):.2f} "
            f"hard_adapt_spill_max={float(cfg.fed_hard_window_adaptive_spillback_max):.2f} "
            f"hard_adapt_qmargin_min_s={float(cfg.fed_hard_window_adaptive_queue_margin_min_sec):.2f}"
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

    def _fed_member_status_active(status: str) -> bool:
        st = str(status or "").strip().upper()
        if not st:
            return False
        return st in fed_bootstrap_active_member_statuses

    def _fed_discovery_filter_active() -> bool:
        if str(fed_peer_selection_source) != "realworld":
            return False
        if not fed_bootstrap_enabled:
            return False
        if not bool(fed_bootstrap_discovery_target_filter_enable):
            return False
        if not fed_bootstrap_discovery_filter_modes:
            return True
        return str(CURRENT_EVALUATION).upper() in fed_bootstrap_discovery_filter_modes

    def _fed_discovery_tls_allowed(tls_id: str, now_wall: Optional[float] = None) -> Tuple[bool, str]:
        tls = str(tls_id or "")
        if not tls:
            return False, "empty_tls"
        if not _fed_discovery_filter_active():
            return True, "filter_inactive"
        wall = float(time.time()) if now_wall is None else float(now_wall)
        seen_wall = fed_bootstrap_discovery_tls_last_seen_wall.get(tls)
        if seen_wall is None:
            return (True, "fail_open_not_discovered") if fed_bootstrap_discovery_fail_open else (False, "not_discovered")
        age_sec = max(0.0, wall - float(seen_wall))
        if age_sec > float(fed_bootstrap_discovery_target_max_age_sec):
            return (True, "fail_open_stale_discovery") if fed_bootstrap_discovery_fail_open else (False, "stale_discovery")
        if bool(fed_bootstrap_discovery_require_membership_valid):
            gw = str(fed_bootstrap_discovery_tls_gateway.get(tls, "") or "")
            st = str(fed_bootstrap_member_status_by_gateway.get(gw, "") or "").upper()
            if not gw:
                return (True, "fail_open_missing_gateway") if fed_bootstrap_discovery_fail_open else (False, "missing_gateway")
            if not _fed_member_status_active(st):
                return (True, "fail_open_inactive_member") if fed_bootstrap_discovery_fail_open else (False, "inactive_member")
        return True, "discovered_active"

    def _fed_filter_tls_candidates(
        candidate_tls_ids: Sequence[str],
        *,
        context: str,
        ev_id_ctx: str = "",
        sim_time_ctx: float = -1.0,
    ) -> Tuple[List[str], Dict[str, str]]:
        cand = [str(x) for x in list(candidate_tls_ids or []) if str(x)]
        if not cand:
            return [], {}
        if not _fed_discovery_filter_active():
            return list(cand), {str(x): "filter_inactive" for x in cand}
        wall_now = float(time.time())
        accepted: List[str] = []
        reasons: Dict[str, str] = {}
        rejected: List[str] = []
        rejected_reasons: List[str] = []
        for tls in cand:
            ok, reason = _fed_discovery_tls_allowed(tls, now_wall=wall_now)
            if ok:
                accepted.append(str(tls))
                reasons[str(tls)] = str(reason)
            else:
                rejected.append(str(tls))
                rejected_reasons.append(str(reason))
        if rejected:
            _fed_dbg_main(
                f"evt=DISCOVERY_TARGET_FILTER context={context} ev={ev_id_ctx or '-'} "
                f"sim={sim_time_ctx:.2f} rejected_n={len(rejected)} accepted_n={len(accepted)} "
                f"rejected={','.join(rejected[:8])} reasons={','.join(rejected_reasons[:8])}"
            )
            _fed_evt_main(
                "discovery.target.filter",
                role="ev",
                context=str(context),
                ev_id=str(ev_id_ctx or ""),
                sim_time=float(sim_time_ctx),
                rejected_n=int(len(rejected)),
                accepted_n=int(len(accepted)),
                rejected_tls=list(rejected[:16]),
                rejected_reasons=list(rejected_reasons[:16]),
            )
        return accepted, reasons

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
        for item in next_tls:
            if not isinstance(item, dict):
                continue
            tls_id = str(item.get("tls_id", item.get("tlsId", "")) or "")
            if not tls_id:
                continue
            tls_ok, tls_reason = _fed_discovery_tls_allowed(tls_id)
            if not tls_ok:
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
                "discovery_target_reason": str(tls_reason),
                "delivery": "mqtt",
            }
            out.append((tls_id, req))
            break

        # Fallback schema: single target from tls_id or edge mapping.
        if not out:
            tls_id_fallback = str(state_obj.get("tls_id", state_obj.get("tlsId", "")) or "")
            if not tls_id_fallback and edge_fallback:
                tls_id_fallback = _tls_from_edge(edge_fallback)
            if tls_id_fallback:
                tls_ok_fb, tls_reason_fb = _fed_discovery_tls_allowed(tls_id_fallback)
                if not tls_ok_fb:
                    return out
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
                    "source_tag": str(ev_request_source_tag or "ev_http_adapter"),
                    "discovery_target_reason": str(tls_reason_fb),
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
            f"probe={fed_bootstrap_discovery_probe_sec:.1f}s "
            f"cadence={fed_bootstrap_cadence_mode} "
            f"discovery_filter={1 if fed_bootstrap_discovery_target_filter_enable else 0} "
            f"peer_selection_source={fed_peer_selection_source} "
            f"modes={','.join(sorted(fed_bootstrap_discovery_filter_modes))} "
            f"max_age={fed_bootstrap_discovery_target_max_age_sec:.1f}s "
            f"fail_open={1 if fed_bootstrap_discovery_fail_open else 0} "
            f"require_membership={1 if fed_bootstrap_discovery_require_membership_valid else 0}"
        )
        if str(fed_peer_selection_source) == "fnm":
            print(
                "[FED_BOOTSTRAP] discovery_authority=fnm (real-world discovery probes disabled; "
                "real-world keeps membership/catalog/heartbeat only)"
            )

    def _fed_bootstrap_now(sim_now: Optional[float] = None) -> float:
        if fed_bootstrap_cadence_mode == "sim":
            if sim_now is not None:
                return float(sim_now)
            try:
                return float(traci.simulation.getTime())
            except Exception:
                return 0.0
        return float(time.time())

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
            for node_id in sorted(set(str(x) for x in list(passive_intersection_ids or []))):
                members.append(
                    {
                        "kind": "passive_intersection",
                        "gateway_id": f"gw-passive-{node_id}",
                        "node_id": str(node_id),
                        "role": "PassiveIntersectionObserver",
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
                    "publish_topic": "federation/rw/step",
                    "subscribe_topic": "rw/step",
                },
                {
                    "name": "rw_vehicle_state_pub",
                    "direction": "local_to_fed",
                    "event_type": "vehicle_state",
                    "publish_topic": "federation/rw/vehicle/+/state",
                    "subscribe_topic": "rw/vehicle/+/state",
                },
                {
                    "name": "rw_tls_state_pub",
                    "direction": "local_to_fed",
                    "event_type": "tls_state",
                    "publish_topic": "federation/rw/tls/+/state",
                    "subscribe_topic": "rw/tls/+/state",
                },
                {
                    "name": "federation_reservation_bridge",
                    "direction": "bidirectional",
                    "event_type": "federation_reservation",
                    "publish_topic": "federation/reservation/req/+",
                    "subscribe_topic": "federation/reservation/resp/+",
                },
                {
                    "name": "ev_request_bridge",
                    "direction": "fed_to_local",
                    "event_type": "ev_request",
                    "publish_topic": "rw/ev/request/+",
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
                    "publish_topic": f"federation/rw/agents/active/{len(active_agent_tls_ids)}",
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
        if kind == "passive_intersection":
            return [
                {
                    "name": f"passive_intersection_state_pub_{node_id}",
                    "direction": "local_to_fed",
                    "event_type": "passive_intersection_context",
                    "publish_topic": f"federation/v1/state/passive_intersection/{node_id}",
                    "subscribe_topic": f"federation/v1/request/passive_intersection/{node_id}",
                },
                {
                    "name": f"passive_downstream_context_provider_{node_id}",
                    "direction": "local_to_fed",
                    "event_type": "downstream_context",
                    "publish_topic": f"federation/v1/state/passive_intersection/{node_id}",
                    "subscribe_topic": f"federation/v1/request/downstream_context/{node_id}",
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
        elif kind == "passive_intersection":
            geo_scope = {
                "type": "intersection",
                "city": city_name,
                "zone": "dynamic",
                "network": network_name,
                "id": node_id,
            }
            policy_tags = ["intersection", "passive-observer", "downstream-context", "no-actuation"]
            update_period_sec = max(
                0.1,
                float(getattr(args, "passive_intersection_context_period_sec", 1.0) or 1.0),
            )
            latency_budget_ms = 300.0
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
            "event": "register",
            "request_id": f"realworld-reg-{gateway_id}-{int(time.time() * 1000)}",
            "gateway_id": gateway_id,
            "node_id": node_id,
            "role": role,
            "domain": domain,
            "capabilities": caps,
            "status": "REGISTERED",
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_register_topic, json.dumps(payload))
        if gateway_id:
            fed_bootstrap_member_status_by_gateway[str(gateway_id)] = "REGISTERED"
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
            "status": "ACTIVE",
            "ts": float(time.time()),
        }
        client.publish(fed_bootstrap_heartbeat_topic, json.dumps(payload))
        if gateway_id:
            fed_bootstrap_member_status_by_gateway[str(gateway_id)] = "ACTIVE"
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
        if str(fed_peer_selection_source) != "realworld":
            _fed_dbg_main(
                f"evt=FED_BOOTSTRAP_DISCOVERY_SKIP reason=peer_selection_source_{fed_peer_selection_source}"
            )
            return
        if float(fed_bootstrap_discovery_probe_sec) <= 0.0:
            _fed_dbg_main("evt=FED_BOOTSTRAP_DISCOVERY_SKIP reason=probe_sec_zero")
            return
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
        fed_bootstrap_discovery_req_sent_wall[str(req_id)] = float(time.time())
        _fed_dbg_main(
            f"evt=FED_BOOTSTRAP_DISCOVERY_QUERY topic={fed_bootstrap_discovery_query_topic} "
            f"reply={reply_topic} req_id={req_id} event_filter={fed_bootstrap_discovery_event_filter or '-'}"
        )

    def on_message(_client, _userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            logical_topic = _topic_in_ns(str(msg.topic))
            if logical_topic is None:
                if str(msg.topic).startswith("federation/") or str(msg.topic).startswith("mx/"):
                    _fed_dbg_main(
                        f"evt=RX_DROP_OUT_OF_NAMESPACE wire_topic={str(msg.topic)} "
                        f"namespace={mqtt_topic_namespace or '-'}"
                    )
                return
            cmd_queue.append((logical_topic, payload))
            if str(logical_topic).startswith("federation/"):
                _fed_dbg_main(
                    f"evt=RX_ENQUEUE topic={logical_topic} wire_topic={str(msg.topic)} req_id={payload.get('req_id')} "
                    f"from={payload.get('from_tls')} to={payload.get('to_tls')}"
                )
        except Exception as e:
            print("Bad command payload:", e, msg.topic)

    def _subscribe_runtime_topics(reason: str) -> None:
        topics = ["cmd/#", "federation/#"]
        if bool(getattr(args, "external_downstream_context_enable", False)):
            topics.append("rw/tls/+/downstream_context")
        if ev_request_delivery_mode in ("mqtt", "both") or bool(ev_http_adapter_enabled):
            topics.append(f"{ev_request_topic_prefix}/+")
        for topic in topics:
            try:
                res = client.subscribe(topic)
                rc = res[0] if isinstance(res, tuple) and res else getattr(res, "rc", res)
                _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_REQ reason={reason} logical={topic} rc={rc}")
            except Exception as e:
                _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_ERR reason={reason} logical={topic} err={type(e).__name__}:{e}")
        if mqtt_topic_namespace and (ev_request_delivery_mode in ("mqtt", "both") or bool(ev_http_adapter_enabled)):
            # Compatibility receive path for FNM sidecars or tools that publish
            # the canonical federation request topic without the run namespace.
            bare_topic = f"{ev_request_topic_prefix}/+"
            try:
                _fed_dbg_main(
                    f"evt=MQTT_SUBSCRIBE_BARE_COMPAT reason={reason} logical={bare_topic} "
                    f"wire={bare_topic} namespace={mqtt_topic_namespace}"
                )
                res = _mqtt_subscribe_orig(bare_topic)
                rc = res[0] if isinstance(res, tuple) and res else getattr(res, "rc", res)
                _fed_dbg_main(f"evt=MQTT_SUBSCRIBE_BARE_COMPAT_REQ reason={reason} logical={bare_topic} rc={rc}")
            except Exception as e:
                _fed_dbg_main(
                    f"evt=MQTT_SUBSCRIBE_BARE_COMPAT_ERR reason={reason} logical={bare_topic} "
                    f"err={type(e).__name__}:{e}"
                )
        if mqtt_topic_namespace and bool(getattr(args, "external_downstream_context_enable", False)):
            for bare_drone_context in (
                "federation/v1/context/downstream/+",
                "federation/v1/context/downstream/si/+",
            ):
                try:
                    _fed_dbg_main(
                        f"evt=MQTT_SUBSCRIBE_BARE_COMPAT reason={reason} logical={bare_drone_context} "
                        f"wire={bare_drone_context} namespace={mqtt_topic_namespace}"
                    )
                    res = _mqtt_subscribe_orig(bare_drone_context)
                    rc = res[0] if isinstance(res, tuple) and res else getattr(res, "rc", res)
                    _fed_dbg_main(
                        f"evt=MQTT_SUBSCRIBE_BARE_COMPAT_REQ reason={reason} logical={bare_drone_context} rc={rc}"
                    )
                except Exception as e:
                    _fed_dbg_main(
                        f"evt=MQTT_SUBSCRIBE_BARE_COMPAT_ERR reason={reason} logical={bare_drone_context} "
                        f"err={type(e).__name__}:{e}"
                    )

    def _on_connect(_client, _userdata, _flags, reason_code, _properties=None):
        _fed_dbg_main(
            f"evt=MQTT_CONNECTED host={args.mqtt_host} port={int(getattr(args, 'mqtt_port', 1883) or 1883)} "
            f"rc={reason_code} namespace={mqtt_topic_namespace or '-'} client_id={mqtt_client_id} "
            f"client_id_len={len(mqtt_client_id)}"
        )
        _subscribe_runtime_topics("on_connect")

    def _on_disconnect(_client, _userdata, *cb_args):
        reason_code = cb_args[-2] if len(cb_args) >= 2 else (cb_args[-1] if cb_args else "")
        _fed_dbg_main(
            f"evt=MQTT_DISCONNECTED host={args.mqtt_host} port={int(getattr(args, 'mqtt_port', 1883) or 1883)} "
            f"rc={reason_code} namespace={mqtt_topic_namespace or '-'} client_id={mqtt_client_id}"
        )

    client.on_connect = _on_connect
    client.on_disconnect = _on_disconnect
    client.on_message = on_message
    client.loop_start()
    _subscribe_runtime_topics("post_loop_start")
    if fed_bootstrap_enabled:
        try:
            n_reg = _fed_bootstrap_publish_register_all(force=True)
            n_cat = _fed_bootstrap_publish_catalog_all()
            n_hb = _fed_bootstrap_publish_heartbeat_all()
            startup_now = _fed_bootstrap_now()
            fed_bootstrap_last_heartbeat_tick = float(startup_now)
            fed_bootstrap_last_catalog_tick = float(startup_now)
            fed_bootstrap_last_probe_tick = float(startup_now)
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
    realtime_sumo_enabled = bool(getattr(args, "realtime_sumo_enable", False))
    realtime_sumo_factor = max(1e-6, float(getattr(args, "realtime_sumo_factor", 1.0) or 1.0))
    realtime_sumo_max_sleep_sec = max(0.0, float(getattr(args, "realtime_sumo_max_sleep_sec", 0.5) or 0.5))
    realtime_sumo_log_period_sec = float(getattr(args, "realtime_sumo_log_period_sec", 5.0) or 5.0)
    realtime_sumo_start_sim_time_sec = max(
        0.0,
        float(getattr(args, "realtime_sumo_start_sim_time_sec", 0.0) or 0.0),
    )
    realtime_wall_start = 0.0
    realtime_sim_start = 0.0
    realtime_pacing_active = False
    realtime_sleep_acc_sec = 0.0
    realtime_lag_acc_sec = 0.0
    realtime_steps_acc = 0

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
        f"evt=EV_REQ_DETERMINISTIC_BARRIER enabled={1 if ev_request_det_apply_enabled else 0} "
        f"grace_sec={float(ev_request_det_grace_sec):.3f} "
        f"max_buffer_sec={float(ev_request_det_max_buffer_sec):.3f}"
    )
    _fed_dbg_main(
        f"evt=EV_REQ_WAIT_FOR_FNM enabled={1 if ev_request_wait_for_fnm_enabled else 0} "
        f"timeout_sec={float(ev_request_wait_for_fnm_timeout_sec):.3f} "
        f"poll_sec={float(ev_request_wait_for_fnm_poll_sec):.3f} "
        f"retry_sim_sec={float(ev_request_wait_for_fnm_retry_sim_sec):.3f} "
        f"raw_dispatch={1 if ev_request_wait_for_fnm_raw_dispatch_enabled else 0}"
    )
    _fed_evt_main(
        "ev.request.deterministic_barrier.config",
        role="ev",
        enabled=bool(ev_request_det_apply_enabled),
        grace_sec=float(ev_request_det_grace_sec),
        max_buffer_sec=float(ev_request_det_max_buffer_sec),
    )
    _fed_evt_main(
        "ev.request.wait_for_fnm.config",
        role="ev",
        enabled=bool(ev_request_wait_for_fnm_enabled),
        timeout_sec=float(ev_request_wait_for_fnm_timeout_sec),
        poll_sec=float(ev_request_wait_for_fnm_poll_sec),
        retry_sim_sec=float(ev_request_wait_for_fnm_retry_sim_sec),
        raw_dispatch_enabled=bool(ev_request_wait_for_fnm_raw_dispatch_enabled),
        deterministic_barrier_enabled=bool(ev_request_det_apply_enabled),
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
    strict_foreign_ev_filter = bool(getattr(args, "strict_foreign_ev_filter", True))
    _fed_dbg_main(
        f"evt=EV_SCOPE_FILTER enabled={1 if strict_foreign_ev_filter else 0} expected_ev={str(ev_id)}"
    )
    _fed_dbg_main(
        f"evt=REALTIME_SUMO_CONFIG enabled={1 if realtime_sumo_enabled else 0} "
        f"factor={float(realtime_sumo_factor):.3f} max_sleep_sec={float(realtime_sumo_max_sleep_sec):.3f} "
        f"log_period_sec={float(realtime_sumo_log_period_sec):.3f} "
        f"start_sim_time_sec={float(realtime_sumo_start_sim_time_sec):.3f}"
    )
    _fed_evt_main(
        "realtime_sumo.config",
        role="simulation",
        enabled=bool(realtime_sumo_enabled),
        factor=float(realtime_sumo_factor),
        max_sleep_sec=float(realtime_sumo_max_sleep_sec),
        log_period_sec=float(realtime_sumo_log_period_sec),
        start_sim_time_sec=float(realtime_sumo_start_sim_time_sec),
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
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:  # noqa: N802
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_GET(self) -> None:  # noqa: N802
                parsed_url = urlparse(self.path)
                p = parsed_url.path
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
                if p in ("/downstream_assessment", "/federation/downstream_assessment"):
                    with ev_http_state_lock:
                        ready = bool(ev_http_state_cache.get("ready", False))
                        payload = dict(ev_http_state_cache.get("payload", {}) or {})
                    if not ready:
                        self._json(503, {"ok": False, "error": "state_not_ready"})
                        return
                    qs = parse_qs(parsed_url.query or "")

                    def _q_first(name: str, default: str = "") -> str:
                        vals = qs.get(name, [])
                        return str(vals[0]) if vals else str(default)

                    def _q_int(name: str, default: int) -> int:
                        try:
                            return int(float(_q_first(name, str(default))))
                        except Exception:
                            return int(default)

                    def _q_float(name: str, default: float) -> float:
                        try:
                            return float(_q_first(name, str(default)))
                        except Exception:
                            return float(default)

                    edge_tokens: List[str] = []
                    for raw in list(qs.get("edge", []) or []) + list(qs.get("edges", []) or []):
                        for part in str(raw or "").replace(";", ",").split(","):
                            part = part.strip()
                            if part:
                                edge_tokens.append(part)
                    if not edge_tokens:
                        self._json(400, {"ok": False, "error": "missing_edges"})
                        return
                    diag = downstream_edges_assessment_diag(
                        target_edges=list(edge_tokens),
                        min_halt_n=_q_int("min_halt_n", int(getattr(args, "b1_downstream_blockage_min_halt_n", 3))),
                        max_mean_speed_mps=_q_float(
                            "max_mean_speed_mps",
                            float(getattr(args, "b1_downstream_blockage_max_mean_speed_mps", 1.0)),
                        ),
                        min_veh_n=_q_int("min_veh_n", int(getattr(args, "b1_downstream_blockage_min_veh_n", 2))),
                        max_occupancy_pct=_q_float(
                            "max_occupancy_pct",
                            float(getattr(args, "b1_downstream_blockage_max_occupancy_pct", 35.0)),
                        ),
                    )
                    response = {
                        "ok": True,
                        "source": "real_world.sumo_proxy",
                        "context_type": "downstream_assessment",
                        "request_id": _q_first("request_id", ""),
                        "requester_tls": _q_first("requester_tls", ""),
                        "ev_id": _q_first("ev_id", str(ev_id)),
                        "simTime": float(payload.get("simTime", -1.0)),
                        "tsWall": float(time.time()),
                        **diag,
                    }
                    if ev_http_state_server_verbose:
                        try:
                            print(
                                "[downstream-http] "
                                f"edges={len(edge_tokens)} blocked={int(bool(diag.get('blocked', False)))} "
                                f"reason={diag.get('reason', '')} worst={diag.get('worst_edge', '')}"
                            )
                        except Exception:
                            pass
                    self._json(200, response)
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
    ev_intersection_discovery_modes = {
        str(x).strip().upper()
        for x in str(getattr(args, "ev_intersection_discovery_modes", "B1") or "B1").split(",")
        if str(x).strip()
    }
    ev_intersection_discovery_seen: set[Tuple[str, str, str, str]] = set()
    ev_intersection_discovery_ready_at: Dict[Tuple[str, str, str, str], float] = {}
    ev_intersection_discovery_response_logged: set[Tuple[str, str, str, str]] = set()
    ev_intersection_discovery_wait_last: Dict[Tuple[str, str, str, str], float] = {}
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
        "drop_foreign_ev_id": 0,
        "drop_parse_err": 0,
    }
    ev_req_pipeline_last: Dict[str, Any] = {}
    ev_req_wait_last_timeout_sim_by_key: Dict[Tuple[str, str], float] = {}
    terminate_on_ev_finish = bool(getattr(args, "terminate_on_ev_finish", False))
    ev_seen_once = False
    expected_ev_id = str(ev_id)

    def _drop_foreign_ev(topic: str, payload_obj: Dict[str, Any], *, kind: str, dst_tls: str) -> bool:
        if not strict_foreign_ev_filter:
            return False
        try:
            p = dict(payload_obj or {})
        except Exception:
            p = {}
        ev_in = str(p.get("ev_id", p.get("evId", "")) or "").strip()
        if not ev_in:
            return False
        if ev_in == expected_ev_id:
            return False
        ev_req_pipeline_stats["drop_foreign_ev_id"] = int(ev_req_pipeline_stats.get("drop_foreign_ev_id", 0)) + 1
        _fed_dbg_main(
            f"evt=RX_DROP_FOREIGN_EV topic={topic} kind={kind} dst={dst_tls} "
            f"ev={ev_in} expected_ev={expected_ev_id}"
        )
        _fed_evt_main(
            "federation.message.drop",
            role="ev",
            reason="foreign_ev_id",
            kind=str(kind),
            tls_id=str(dst_tls),
            ev_id=str(ev_in),
            expected_ev_id=str(expected_ev_id),
            topic=str(topic),
        )
        return True

    def _ev_req_payload_sim_time(payload_obj: Dict[str, Any], fallback_sim: float) -> float:
        """Best-effort extraction of request sim-time from flat or wrapped EVRequest payloads."""
        try:
            p = dict(payload_obj or {})
        except Exception:
            p = {}
        if isinstance(p.get("ev_request"), dict):
            try:
                p = dict(p.get("ev_request") or {})
            except Exception:
                pass
        for key in ("sim_time", "simTime"):
            try:
                if key in p:
                    return float(p.get(key))
            except Exception:
                pass
        return float(fallback_sim)

    def _ev_req_payload_ev_id(payload_obj: Dict[str, Any]) -> str:
        try:
            p = dict(payload_obj or {})
        except Exception:
            p = {}
        if isinstance(p.get("ev_request"), dict):
            try:
                p = dict(p.get("ev_request") or {})
            except Exception:
                pass
        return str(p.get("ev_id", p.get("evId", "")) or "")

    def _ev_req_payload_normalized(payload_obj: Dict[str, Any]) -> Dict[str, Any]:
        """Normalize flat/wrapped EV request payloads into EvRequest kwargs."""
        try:
            p = dict(payload_obj or {})
        except Exception:
            p = {}
        if isinstance(p.get("ev_request"), dict):
            try:
                p = dict(p.get("ev_request") or {})
            except Exception:
                pass
        aliases = {
            "evId": "ev_id",
            "simTime": "sim_time",
            "erlLevel": "erl_level",
            "speedMps": "speed_mps",
            "distanceToIntersectionM": "distance_to_intersection_m",
            "inEdgeId": "in_edge_id",
            "targetPhaseIdx": "target_phase_idx",
            "deltaSec": "delta_sec",
            "routeIntersections": "route_intersections",
            "routeVeh": "route_veh",
        }
        for src, dst in aliases.items():
            if dst not in p and src in p:
                p[dst] = p.get(src)
        return p

    def _resolve_ev_req_in_edge_for_agent(ev_req_payload: Dict[str, Any], ag_dst: object, dst_tls: str) -> Tuple[Dict[str, Any], str, str]:
        """Resolve request inbound edge against the destination TLS phase map."""
        p = dict(ev_req_payload or {})
        ev_id_in = str(p.get("ev_id", "") or "")
        in_edge_in = str(p.get("in_edge_id", "") or "")
        try:
            inbound_map = dict(getattr(ag_dst, "_inbound_edge_to_phase", {}) or {})
        except Exception:
            inbound_map = {}
        infer_edge = ""
        infer_src = ""
        if inbound_map:
            if in_edge_in and (inbound_map.get(in_edge_in) is not None):
                infer_edge = str(in_edge_in)
                infer_src = "payload"
            if not infer_edge:
                for _e in [str(x) for x in list(p.get("route_veh", []) or []) if str(x)]:
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
            p["in_edge_id"] = str(infer_edge)
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
        return p, str(infer_edge or in_edge_in or ""), str(infer_src or "")

    def _dispatch_ev_request_payload_now(
        topic: str,
        payload_obj: Dict[str, Any],
        *,
        current_sim: float,
        dispatch_reason: str,
        deterministic_released: bool = False,
        det_meta: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Dispatch one EV request payload to the local TLS agent without advancing SUMO."""
        if not str(topic).startswith(f"{ev_request_topic_prefix}/"):
            return False
        dst_tls = str(topic).split("/")[-1]
        ev_req_pipeline_stats["rx_total"] = int(ev_req_pipeline_stats.get("rx_total", 0)) + 1
        ev_req_payload = _ev_req_payload_normalized(dict(payload_obj or {}))
        ev_id_rx = str(ev_req_payload.get("ev_id", "") or "")
        filtered_dst_tls, filtered_dst_reasons = _fed_filter_tls_candidates(
            [str(dst_tls)],
            context=f"ev_request_{dispatch_reason}",
            ev_id_ctx=str(ev_id_rx),
            sim_time_ctx=float(current_sim),
        )
        if not filtered_dst_tls:
            ev_req_pipeline_stats["drop_discovery_filter"] = int(
                ev_req_pipeline_stats.get("drop_discovery_filter", 0)
            ) + 1
            _fed_evt_main(
                "ev.request.drop",
                role="ev",
                reason="discovery_target_filter",
                tls_id=str(dst_tls),
                ev_id=str(ev_id_rx),
                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                dispatch_reason=str(dispatch_reason),
            )
            return False
        dst_reason = str(filtered_dst_reasons.get(str(dst_tls), "") or "")
        ag_dst = agents.get(dst_tls)
        if ag_dst is None and on_demand_agent_activation:
            _activate_tls_agents([str(dst_tls)], reason=f"ev_request_{dispatch_reason}", sim_time=float(current_sim), max_new=1)
            ag_dst = agents.get(dst_tls)
        if _drop_foreign_ev(str(topic), ev_req_payload, kind="ev_request", dst_tls=str(dst_tls)):
            return False
        _fed_dbg_main(
            f"evt=RX_DISPATCH topic={topic} kind=ev_request dst={dst_tls} "
            f"agent_found={1 if ag_dst is not None else 0} ev={ev_req_payload.get('ev_id') or ev_req_payload.get('evId')} "
            f"discovery_reason={dst_reason or '-'} dispatch_reason={dispatch_reason}"
        )
        if ag_dst is None:
            ev_req_pipeline_stats["drop_no_agent"] = int(ev_req_pipeline_stats.get("drop_no_agent", 0)) + 1
            _fed_evt_main(
                "ev.request.drop",
                role="ev",
                reason="agent_not_found",
                tls_id=str(dst_tls),
                ev_id=str(ev_req_payload.get("ev_id", "") or ""),
                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                dispatch_reason=str(dispatch_reason),
            )
            return False
        if ev_request_delivery_mode == "both" and str(ev_req_payload.get("source_service", "")) == "vehicle_agent":
            ev_req_pipeline_stats["drop_replay_both_mode"] = int(ev_req_pipeline_stats.get("drop_replay_both_mode", 0)) + 1
            return False
        if b1_strict_local_baseline:
            try:
                inbound_map = dict(getattr(ag_dst, "_inbound_edge_to_phase", {}) or {})
            except Exception:
                inbound_map = {}
            current_edge = str(ev_req_payload.get("in_edge_id", "") or "")
            if (not current_edge or current_edge.startswith(":")) and ev_id_rx and traci is not None:
                try:
                    traci_edge = str(traci.vehicle.getRoadID(str(ev_id_rx)) or "")
                    if traci_edge and not traci_edge.startswith(":"):
                        current_edge = str(traci_edge)
                except Exception:
                    pass
            if not current_edge or inbound_map.get(str(current_edge)) is None:
                ev_req_pipeline_stats["drop_b1_strict_nonlocal_tls"] = int(
                    ev_req_pipeline_stats.get("drop_b1_strict_nonlocal_tls", 0)
                ) + 1
                _fed_dbg_main(
                    f"evt=B1_STRICT_EV_REQUEST_DROP ev={ev_id_rx or '-'} tls={dst_tls} "
                    f"reason=nonlocal_or_non_tls_current_edge current_edge={current_edge or '-'} "
                    f"dispatch_reason={dispatch_reason}"
                )
                _fed_evt_main(
                    "b1.local_discovery.request_drop",
                    role="ev",
                    ev_id=str(ev_id_rx),
                    tls_id=str(dst_tls),
                    sim_time=float(current_sim),
                    reason="nonlocal_or_non_tls_current_edge",
                    current_edge=str(current_edge or ""),
                    dispatch_reason=str(dispatch_reason),
                    source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                )
                return False
            ev_req_payload["in_edge_id"] = str(current_edge)
            ev_req_payload["route_intersections"] = []
            ev_req_payload["route_veh"] = []
            ev_req_payload["source_tag"] = (
                f"{str(ev_req_payload.get('source_tag', ev_request_source_tag) or '')}:strict_b1_local"
            ).strip(":")
        ev_req_payload, _infer_edge, _infer_src = _resolve_ev_req_in_edge_for_agent(ev_req_payload, ag_dst, str(dst_tls))
        ev_id_in = str(ev_req_payload.get("ev_id", "") or "")
        if not ev_id_in:
            ev_req_pipeline_stats["drop_missing_ev_id"] = int(ev_req_pipeline_stats.get("drop_missing_ev_id", 0)) + 1
            _fed_evt_main(
                "ev.request.drop",
                role="ev",
                reason="missing_ev_id",
                tls_id=str(dst_tls),
                source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                dispatch_reason=str(dispatch_reason),
            )
            return False
        try:
            ev_msg_in = EvRequest(
                ev_id=str(ev_req_payload.get("ev_id", "")),
                sim_time=float(ev_req_payload.get("sim_time", current_sim)),
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
                dispatch_reason=str(dispatch_reason),
            )
            return False

        ag_dst.receive_ev_message(ev_msg_in)
        ev_req_pipeline_stats["dispatch_ok"] = int(ev_req_pipeline_stats.get("dispatch_ok", 0)) + 1
        age_ms = max(0.0, (float(current_sim) - float(ev_msg_in.sim_time)) * 1000.0)
        ev_req_pipeline_last.clear()
        ev_req_pipeline_last.update(
            {
                "ev_id": str(ev_msg_in.ev_id),
                "tls_id": str(dst_tls),
                "source": str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                "source_tag": str(ev_req_payload.get("source_tag", ev_request_source_tag) or ""),
                "distance_m": float(ev_msg_in.distance_to_intersection_m),
                "speed_mps": float(ev_msg_in.speed_mps),
                "in_edge_id": str(ev_msg_in.in_edge_id),
                "age_ms": float(age_ms),
            }
        )
        det_meta = dict(det_meta or {})
        det_queue_delay_sim_s = None
        if deterministic_released and det_meta:
            try:
                det_queue_delay_sim_s = max(0.0, float(current_sim) - float(det_meta.get("enqueue_sim_time", current_sim)))
            except Exception:
                det_queue_delay_sim_s = None
        _fed_dbg_main(
            f"evt=EV_REQ_PIPE ev={ev_msg_in.ev_id} tls={dst_tls} src={ev_req_pipeline_last.get('source')} "
            f"src_tag={ev_req_pipeline_last.get('source_tag') or '-'} "
            f"dist={float(ev_msg_in.distance_to_intersection_m):.2f} speed={float(ev_msg_in.speed_mps):.2f} "
            f"edge={ev_msg_in.in_edge_id or '-'} age_ms={float(age_ms):.1f} "
            f"det_barrier={1 if deterministic_released else 0} "
            f"det_seq={int(det_meta.get('seq', 0) or 0) if det_meta else 0} "
            f"det_queue_sim_s={('-1' if det_queue_delay_sim_s is None else f'{float(det_queue_delay_sim_s):.3f}')} "
            f"dispatch_reason={dispatch_reason}"
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
            request_sim_time=float(ev_msg_in.sim_time),
            dispatch_sim_time=float(current_sim),
            deterministic_barrier=bool(deterministic_released),
            barrier_seq=int(det_meta.get("seq", 0) or 0) if det_meta else 0,
            barrier_enqueue_sim_time=float(det_meta.get("enqueue_sim_time", -1.0) or -1.0) if det_meta else -1.0,
            barrier_release_sim_time=float(det_meta.get("release_sim_time", -1.0) or -1.0) if det_meta else -1.0,
            barrier_queue_delay_sim_s=det_queue_delay_sim_s,
            dispatch_reason=str(dispatch_reason),
        )
        return True

    def _agent_has_current_ev_request(ag_obj: object, ev_id_expected: str, selected_in_edge: str, current_sim: float) -> bool:
        active_ev = getattr(ag_obj, "active_ev", None)
        if active_ev is None:
            return False
        if str(getattr(active_ev, "ev_id", "") or "") != str(ev_id_expected):
            return False
        edge = str(getattr(active_ev, "in_edge_id", "") or "")
        if selected_in_edge and edge and edge != str(selected_in_edge):
            return False
        try:
            age = float(current_sim) - float(getattr(active_ev, "sim_time", current_sim))
            if age > max(2.0, float(ev_request_det_max_buffer_sec) + 1.0):
                return False
        except Exception:
            pass
        return True

    def _dispatch_matching_ev_request_from_cmd_queue(ev_id_expected: str, tls_id_expected: str, current_sim: float) -> bool:
        if not cmd_queue:
            return False
        kept = deque()
        dispatched = False
        while cmd_queue:
            topic, payload = cmd_queue.popleft()
            if (
                not dispatched
                and str(topic) == f"{ev_request_topic_prefix}/{tls_id_expected}"
                and _ev_req_payload_ev_id(dict(payload or {})) == str(ev_id_expected)
            ):
                dispatched = _dispatch_ev_request_payload_now(
                    str(topic),
                    dict(payload or {}),
                    current_sim=float(current_sim),
                    dispatch_reason="wait_for_fnm",
                    deterministic_released=False,
                    det_meta=None,
                )
                if dispatched:
                    continue
            kept.append((topic, payload))
        cmd_queue.extend(kept)
        return bool(dispatched)

    def _count_matching_ev_request_in_cmd_queue(ev_id_expected: str, tls_id_expected: str) -> int:
        if not cmd_queue:
            return 0
        n = 0
        for topic, payload in list(cmd_queue):
            try:
                if (
                    str(topic) == f"{ev_request_topic_prefix}/{tls_id_expected}"
                    and _ev_req_payload_ev_id(dict(payload or {})) == str(ev_id_expected)
                ):
                    n += 1
            except Exception:
                continue
        return int(n)

    def _dispatch_matching_ev_request_from_barrier(ev_id_expected: str, tls_id_expected: str, current_sim: float) -> bool:
        if not ev_request_pending_barrier:
            return False
        match_idx = -1
        for idx, rec in enumerate(list(ev_request_pending_barrier)):
            if str(rec.get("dst_tls", "")) != str(tls_id_expected):
                continue
            if str(rec.get("ev_id", "")) != str(ev_id_expected):
                continue
            req_sim = float(rec.get("request_sim_time", current_sim))
            enqueue_sim = float(rec.get("enqueue_sim_time", current_sim))
            force_release = (float(current_sim) - enqueue_sim) >= float(ev_request_det_max_buffer_sec)
            eligible_by_grace = req_sim <= (float(current_sim) - float(ev_request_det_grace_sec)) + 1e-9
            if eligible_by_grace or force_release:
                rec["force_release"] = bool(force_release)
                match_idx = idx
                break
        if match_idx < 0:
            return False
        rec = dict(ev_request_pending_barrier.pop(match_idx))
        payload_out = dict(rec.get("payload", {}) or {})
        det_meta = {
            "seq": int(rec.get("seq", 0)),
            "enqueue_sim_time": float(rec.get("enqueue_sim_time", 0.0)),
            "enqueue_wall_ms": float(rec.get("enqueue_wall_ms", 0.0)),
            "request_sim_time": float(rec.get("request_sim_time", 0.0)),
            "release_sim_time": float(current_sim),
            "force_release": bool(rec.get("force_release", False)),
        }
        return _dispatch_ev_request_payload_now(
            str(rec.get("topic", f"{ev_request_topic_prefix}/{tls_id_expected}")),
            payload_out,
            current_sim=float(current_sim),
            dispatch_reason="wait_for_fnm_barrier",
            deterministic_released=True,
            det_meta=det_meta,
        )

    def _count_matching_ev_request_in_barrier(ev_id_expected: str, tls_id_expected: str) -> int:
        if not ev_request_pending_barrier:
            return 0
        n = 0
        for rec in list(ev_request_pending_barrier):
            if str(rec.get("dst_tls", "")) != str(tls_id_expected):
                continue
            if str(rec.get("ev_id", "")) != str(ev_id_expected):
                continue
            n += 1
        return int(n)

    def _wait_for_fnm_ev_request_if_needed(
        *,
        ev_id_expected: str,
        tls_id_expected: str,
        selected_in_edge: str,
        ag_obj: object,
        current_sim: float,
    ) -> bool:
        if not ev_request_wait_for_fnm_enabled:
            return False
        if internal_ev_request_enabled:
            return False
        if ev_request_wait_for_fnm_timeout_sec <= 0.0:
            return False
        if _agent_has_current_ev_request(ag_obj, str(ev_id_expected), str(selected_in_edge), float(current_sim)):
            return False
        wait_key = (str(ev_id_expected), str(tls_id_expected))
        last_timeout = ev_req_wait_last_timeout_sim_by_key.get(wait_key)
        if last_timeout is not None and (float(current_sim) - float(last_timeout)) < float(ev_request_wait_for_fnm_retry_sim_sec):
            return False
        _fed_dbg_main(
            f"evt=EV_REQ_WAIT_FOR_FNM_START ev={ev_id_expected} tls={tls_id_expected} "
            f"sim={float(current_sim):.2f} timeout_sec={float(ev_request_wait_for_fnm_timeout_sec):.3f}"
        )
        _fed_evt_main(
            "ev.request.wait_for_fnm.start",
            role="ev",
            ev_id=str(ev_id_expected),
            tls_id=str(tls_id_expected),
            sim_time=float(current_sim),
            selected_in_edge=str(selected_in_edge),
            timeout_sec=float(ev_request_wait_for_fnm_timeout_sec),
        )
        deadline = time.perf_counter() + float(ev_request_wait_for_fnm_timeout_sec)
        spins = 0
        hit = False
        raw_match_seen = 0
        barrier_match_seen = 0
        raw_dispatch_allowed = bool(
            ev_request_wait_for_fnm_raw_dispatch_enabled or not ev_request_det_apply_enabled
        )
        while time.perf_counter() <= deadline:
            spins += 1
            barrier_match_seen = max(
                barrier_match_seen,
                _count_matching_ev_request_in_barrier(str(ev_id_expected), str(tls_id_expected)),
            )
            if _dispatch_matching_ev_request_from_barrier(str(ev_id_expected), str(tls_id_expected), float(current_sim)):
                hit = True
                break
            raw_match_seen = max(
                raw_match_seen,
                _count_matching_ev_request_in_cmd_queue(str(ev_id_expected), str(tls_id_expected)),
            )
            if raw_dispatch_allowed:
                if _dispatch_matching_ev_request_from_cmd_queue(str(ev_id_expected), str(tls_id_expected), float(current_sim)):
                    hit = True
                    break
            if _agent_has_current_ev_request(ag_obj, str(ev_id_expected), str(selected_in_edge), float(current_sim)):
                hit = True
                break
            time.sleep(float(ev_request_wait_for_fnm_poll_sec))
        if hit:
            _fed_dbg_main(
                f"evt=EV_REQ_WAIT_FOR_FNM_HIT ev={ev_id_expected} tls={tls_id_expected} "
                f"sim={float(current_sim):.2f} spins={int(spins)}"
            )
            _fed_evt_main(
                "ev.request.wait_for_fnm.hit",
                role="ev",
                ev_id=str(ev_id_expected),
                tls_id=str(tls_id_expected),
                sim_time=float(current_sim),
                spins=int(spins),
                raw_dispatch_allowed=bool(raw_dispatch_allowed),
                raw_match_seen_n=int(raw_match_seen),
                barrier_match_seen_n=int(barrier_match_seen),
            )
            return True
        ev_req_wait_last_timeout_sim_by_key[wait_key] = float(current_sim)
        _fed_dbg_main(
            f"evt=EV_REQ_WAIT_FOR_FNM_TIMEOUT ev={ev_id_expected} tls={tls_id_expected} "
            f"sim={float(current_sim):.2f} spins={int(spins)}"
        )
        _fed_evt_main(
            "ev.request.wait_for_fnm.timeout",
            role="ev",
            ev_id=str(ev_id_expected),
            tls_id=str(tls_id_expected),
            sim_time=float(current_sim),
            spins=int(spins),
            raw_dispatch_allowed=bool(raw_dispatch_allowed),
            raw_match_seen_n=int(raw_match_seen),
            barrier_match_seen_n=int(barrier_match_seen),
            deterministic_barrier_enabled=bool(ev_request_det_apply_enabled),
        )
        return False

    def _release_ev_request_barrier(current_sim: float) -> int:
        """Move deterministic-barrier eligible EV requests back into cmd_queue in stable order."""
        if not ev_request_det_apply_enabled or not ev_request_pending_barrier:
            return 0
        eligible: List[Dict[str, Any]] = []
        waiting: List[Dict[str, Any]] = []
        threshold_sim = float(current_sim) - float(ev_request_det_grace_sec)
        for rec in ev_request_pending_barrier:
            req_sim = float(rec.get("request_sim_time", current_sim))
            enqueue_sim = float(rec.get("enqueue_sim_time", current_sim))
            force_release = (float(current_sim) - enqueue_sim) >= float(ev_request_det_max_buffer_sec)
            if req_sim <= threshold_sim + 1e-9 or force_release:
                rec["force_release"] = bool(force_release)
                eligible.append(rec)
            else:
                waiting.append(rec)
        if not eligible:
            return 0
        ev_request_pending_barrier[:] = waiting
        eligible.sort(
            key=lambda r: (
                round(float(r.get("request_sim_time", 0.0)), 6),
                str(r.get("dst_tls", "")),
                str(r.get("ev_id", "")),
                str(r.get("message_id", "")),
                int(r.get("seq", 0)),
            )
        )
        for rec in reversed(eligible):
            payload_out = dict(rec.get("payload", {}) or {})
            payload_out["_rw_deterministic_release"] = True
            payload_out["_rw_det_meta"] = {
                "seq": int(rec.get("seq", 0)),
                "enqueue_sim_time": float(rec.get("enqueue_sim_time", 0.0)),
                "enqueue_wall_ms": float(rec.get("enqueue_wall_ms", 0.0)),
                "request_sim_time": float(rec.get("request_sim_time", 0.0)),
                "release_sim_time": float(current_sim),
                "force_release": bool(rec.get("force_release", False)),
            }
            cmd_queue.appendleft((str(rec.get("topic", "")), payload_out))
        _fed_dbg_main(
            f"evt=EV_REQ_BARRIER_RELEASE sim={float(current_sim):.2f} "
            f"eligible_n={len(eligible)} pending_n={len(waiting)} "
            f"grace_sec={float(ev_request_det_grace_sec):.3f}"
        )
        _fed_evt_main(
            "ev.request.batch.release",
            role="ev",
            sim_time=float(current_sim),
            eligible_n=int(len(eligible)),
            pending_n=int(len(waiting)),
            grace_sec=float(ev_request_det_grace_sec),
        )
        return int(len(eligible))

    max_sim_time_sec = max(0.0, float(getattr(args, "max_sim_time_sec", 0.0) or 0.0))
    if realtime_sumo_enabled and realtime_sumo_start_sim_time_sec <= 0.0:
        try:
            realtime_sim_start = float(traci.simulation.getTime())
        except Exception:
            realtime_sim_start = 0.0
        realtime_wall_start = float(time.perf_counter())
        realtime_pacing_active = True
        _fed_dbg_main(
            f"evt=REALTIME_SUMO_START sim_start={float(realtime_sim_start):.3f} "
            f"wall_start={float(realtime_wall_start):.6f}"
        )
        _fed_evt_main(
            "realtime_sumo.start",
            role="simulation",
            sim_start=float(realtime_sim_start),
            wall_start=float(realtime_wall_start),
            delayed_start=False,
        )
    elif realtime_sumo_enabled:
        _fed_dbg_main(
            f"evt=REALTIME_SUMO_PREROLL enabled=1 start_sim_time_sec={float(realtime_sumo_start_sim_time_sec):.3f}"
        )
        _fed_evt_main(
            "realtime_sumo.preroll",
            role="simulation",
            start_sim_time_sec=float(realtime_sumo_start_sim_time_sec),
        )

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            t_iter_wall = time.perf_counter()

            # apply pending commands (DT/middleware -> SUMO)
            t_cmd_wall = time.perf_counter()
            try:
                _cmd_sim_time = float(traci.simulation.getTime())
            except Exception:
                _cmd_sim_time = float(step) * float(getattr(args, "step_length", 0.1))
            _release_ev_request_barrier(float(_cmd_sim_time))
            while cmd_queue:
                topic, payload = cmd_queue.popleft()
                if (
                    ev_request_det_apply_enabled
                    and str(topic).startswith(f"{ev_request_topic_prefix}/")
                    and not bool((payload or {}).get("_rw_deterministic_release", False))
                ):
                    try:
                        ev_request_barrier_seq += 1
                        _req_sim_t = _ev_req_payload_sim_time(dict(payload or {}), float(_cmd_sim_time))
                        _dst_tls = str(topic).split("/")[-1]
                        _ev_in = _ev_req_payload_ev_id(dict(payload or {}))
                        _msg_id = ""
                        try:
                            _p_msg = dict(payload or {})
                            if isinstance(_p_msg.get("ev_request"), dict):
                                _p_msg = dict(_p_msg.get("ev_request") or {})
                            _msg_id = str(_p_msg.get("message_id", "") or "")
                        except Exception:
                            _msg_id = ""
                        ev_request_pending_barrier.append(
                            {
                                "seq": int(ev_request_barrier_seq),
                                "topic": str(topic),
                                "payload": dict(payload or {}),
                                "dst_tls": str(_dst_tls),
                                "ev_id": str(_ev_in),
                                "message_id": str(_msg_id),
                                "request_sim_time": float(_req_sim_t),
                                "enqueue_sim_time": float(_cmd_sim_time),
                                "enqueue_wall_ms": float(time.time() * 1000.0),
                            }
                        )
                        _fed_dbg_main(
                            f"evt=EV_REQ_BARRIER_BUFFER seq={int(ev_request_barrier_seq)} "
                            f"ev={_ev_in or '-'} tls={_dst_tls} enqueue_sim={float(_cmd_sim_time):.2f} "
                            f"request_sim={float(_req_sim_t):.2f} "
                            f"eligible_sim={float(_req_sim_t + ev_request_det_grace_sec):.2f} "
                            f"pending_n={len(ev_request_pending_barrier)}"
                        )
                        _fed_evt_main(
                            "ev.request.buffered",
                            role="ev",
                            ev_id=str(_ev_in),
                            tls_id=str(_dst_tls),
                            seq=int(ev_request_barrier_seq),
                            enqueue_sim_time=float(_cmd_sim_time),
                            request_sim_time=float(_req_sim_t),
                            eligible_sim_time=float(_req_sim_t + ev_request_det_grace_sec),
                            pending_n=int(len(ev_request_pending_barrier)),
                        )
                    except Exception as e:
                        _fed_dbg_main(f"evt=EV_REQ_BARRIER_BUFFER_ERR topic={topic} err={type(e).__name__}:{e}")
                    continue
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
                            if _is_passive_dt_mode(CURRENT_EVALUATION):
                                args.passive_intersection_dt_enable = True
                                args.f2p_queue_release_enable = bool(_is_f2p_queue_release_mode(CURRENT_EVALUATION))
                            else:
                                args.f2p_queue_release_enable = False
                            if _is_drone_augmented_mode(CURRENT_EVALUATION):
                                args.f2_drone_context_request_enable = True
                                args.external_downstream_context_enable = True
                                args.f2d_queue_release_enable = bool(_is_f2d_queue_release_mode(CURRENT_EVALUATION))
                            else:
                                args.f2_drone_context_request_enable = False
                                args.external_downstream_context_enable = False
                                args.f2d_queue_release_enable = False
                                args.f2d_drone_prescout_enable = False
                            try:
                                ev_profile.metadata["evaluation"] = str(CURRENT_EVALUATION)
                            except Exception:
                                pass
                            for _ag in agents.values():
                                try:
                                    _ag.cfg.decision_log_run_label = str(CURRENT_EVALUATION)
                                    _ag.cfg.f2_drone_context_request_enable = bool(
                                        getattr(args, "f2_drone_context_request_enable", False)
                                    )
                                    _ag.cfg.f2d_drone_prescout_enable = bool(
                                        getattr(args, "f2d_drone_prescout_enable", False)
                                        and _is_f2d_prescout_mode(CURRENT_EVALUATION)
                                    )
                                    _ag.cfg.f2_drone_context_discovery_gate_enable = bool(
                                        getattr(args, "f2_drone_context_discovery_gate_enable", False)
                                        and _is_drone_augmented_mode(CURRENT_EVALUATION)
                                    )
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

                    elif topic.startswith("rw/tls/") and topic.endswith("/downstream_context"):
                        if not bool(getattr(args, "external_downstream_context_enable", False)):
                            _fed_dbg_main(
                                "evt=DOWNSTREAM_CONTEXT_IGNORED reason=external_context_disabled delivery=local_si_bus"
                            )
                            continue
                        parts = topic.split("/")
                        target_tls = str(payload.get("target_tls_id", parts[2] if len(parts) > 2 else "") or "")
                        requester_tls = str(payload.get("requester_tls", payload.get("_fanout_from_tls", "")) or "")
                        ctx_ev_id = str(payload.get("ev_id", "") or "")
                        ctx_ok = bool(payload.get("ok", True))
                        ctx_accepted = bool(payload.get("accepted", True))
                        if not (ctx_ok and ctx_accepted):
                            _fed_evt_main(
                                "f2d.context.local_si_rejected",
                                **_drone_context_trace_payload(
                                    rec=dict(payload or {}),
                                    tls_id=str(target_tls),
                                    ev_id=str(ctx_ev_id),
                                    sim_time=float(_cmd_sim_time),
                                    selected_action="reject_local_si_context",
                                    decision_source="fnm_local_si_context_rx",
                                    reason=str(payload.get("reason", payload.get("error", "context_not_ok"))),
                                    context_age_sec=0.0,
                                    extra={
                                        "ok": bool(ctx_ok),
                                        "accepted": bool(ctx_accepted),
                                        "error": str(payload.get("error", "")),
                                        "target_tls_id": str(target_tls),
                                        "requester_tls": str(requester_tls),
                                        "delivery_model": str(payload.get("delivery_model", "f2d_directed_si_context")),
                                    },
                                ),
                            )
                            continue
                        _cache_directed_external_downstream_context(
                            rec=dict(payload or {}),
                            tls_id=str(target_tls),
                            ev_id=str(ctx_ev_id),
                            requester_tls=str(requester_tls),
                            sim_time=float(_cmd_sim_time),
                            decision_source="fnm_local_si_context_rx",
                            selected_action="receive_local_si_context",
                        )

                    elif topic.startswith("federation/v1/context/downstream/si/"):
                        if not bool(getattr(args, "external_downstream_context_enable", False)):
                            _fed_dbg_main(
                                "evt=DOWNSTREAM_CONTEXT_IGNORED reason=external_context_disabled delivery=directed_si"
                            )
                            continue
                        target_tls = str(payload.get("target_tls_id", topic.split("/")[-1]) or "")
                        requester_tls = str(payload.get("requester_tls", payload.get("_fanout_from_tls", "")) or "")
                        ctx_ev_id = str(payload.get("ev_id", "") or "")
                        ctx_ok = bool(payload.get("ok", True))
                        ctx_accepted = bool(payload.get("accepted", True))
                        if not (ctx_ok and ctx_accepted):
                            _fed_evt_main(
                                "f2d.context.si_dt_rejected",
                                **_drone_context_trace_payload(
                                    rec=dict(payload or {}),
                                    tls_id=str(target_tls),
                                    ev_id=str(ctx_ev_id),
                                    sim_time=float(_cmd_sim_time),
                                    selected_action="reject_directed_context",
                                    decision_source="directed_si_context_rx",
                                    reason=str(payload.get("reason", payload.get("error", "context_not_ok"))),
                                    context_age_sec=0.0,
                                    extra={
                                        "ok": bool(ctx_ok),
                                        "accepted": bool(ctx_accepted),
                                        "error": str(payload.get("error", "")),
                                        "target_tls_id": str(target_tls),
                                        "requester_tls": str(requester_tls),
                                        "delivery_model": str(payload.get("delivery_model", "f2d_directed_si_context")),
                                    },
                                ),
                            )
                            continue
                        rec = dict(payload or {})
                        if not bool(getattr(args, "f2d_directed_context_self_delivery_enable", False)):
                            _fed_evt_main(
                                "f2d.context.directed_observed",
                                **_drone_context_trace_payload(
                                    rec=rec,
                                    tls_id=str(target_tls),
                                    ev_id=str(ctx_ev_id),
                                    sim_time=float(_cmd_sim_time),
                                    selected_action="observe_directed_context_wait_for_fnm_local_delivery",
                                    decision_source="directed_si_context_rx",
                                    reason=str(rec.get("reason", "")),
                                    context_age_sec=0.0,
                                    extra={
                                        "target_tls_id": str(target_tls),
                                        "requester_tls": str(requester_tls),
                                        "delivery_model": str(rec.get("delivery_model", "f2d_directed_si_context")),
                                        "self_delivery_enabled": False,
                                    },
                                ),
                            )
                            continue
                        _cache_directed_external_downstream_context(
                            rec=rec,
                            tls_id=str(target_tls),
                            ev_id=str(ctx_ev_id),
                            requester_tls=str(requester_tls),
                            sim_time=float(_cmd_sim_time),
                            decision_source="directed_si_context_rx",
                            selected_action="receive_directed_context",
                        )

                    elif topic.startswith("federation/v1/context/downstream/"):
                        if not bool(getattr(args, "external_downstream_context_enable", False)):
                            _fed_dbg_main(
                                "evt=DOWNSTREAM_CONTEXT_IGNORED reason=external_context_disabled"
                            )
                            continue
                        provider_id = str(topic.split("/")[-1] or payload.get("provider_id", ""))
                        requester_tls = str(payload.get("requester_tls", payload.get("tls_id", "")) or "")
                        ctx_ev_id = str(payload.get("ev_id", "") or "")
                        ctx_ok = bool(payload.get("ok", True))
                        ctx_accepted = bool(payload.get("accepted", True))
                        if not (ctx_ok and ctx_accepted):
                            _fed_evt_main(
                                "f2.drone_context.rejected",
                                **_drone_context_trace_payload(
                                    rec=dict(payload or {}),
                                    tls_id=str(requester_tls),
                                    ev_id=str(ctx_ev_id),
                                    sim_time=float(_cmd_sim_time),
                                    selected_action="reject_context",
                                    decision_source="mqtt_context_rx",
                                    reason=str(payload.get("reason", payload.get("error", "context_not_ok"))),
                                    context_age_sec=0.0,
                                    extra={
                                        "ok": bool(ctx_ok),
                                        "accepted": bool(ctx_accepted),
                                        "error": str(payload.get("error", "")),
                                        "provider_id": str(provider_id),
                                    },
                                ),
                            )
                            _fed_dbg_main(
                                f"evt=DOWNSTREAM_CONTEXT_DROP reason=context_not_ok provider={provider_id} "
                                f"requester_tls={requester_tls} ok={int(ctx_ok)} accepted={int(ctx_accepted)} "
                                f"error={payload.get('error', '')}"
                            )
                            continue
                        if requester_tls:
                            realworld_rx_wall_ts = float(time.time())
                            rec = dict(payload or {})
                            rec["provider_id"] = str(rec.get("provider_id", provider_id) or provider_id)
                            rec["_rx_wall"] = float(realworld_rx_wall_ts)
                            rec["realworld_rx_wall_ts"] = float(realworld_rx_wall_ts)
                            rec["realworld_rx_wall_ms"] = float(realworld_rx_wall_ts * 1000.0)
                            try:
                                rec["response_payload_size_bytes_rx"] = int(
                                    len(json.dumps(payload, ensure_ascii=True).encode("utf-8"))
                                )
                            except Exception:
                                rec["response_payload_size_bytes_rx"] = 0
                            rec["_rx_sim_time"] = float(_cmd_sim_time)
                            fanout_tls = [str(requester_tls)]
                            if (
                                _f2d_mobile_passive_context_enabled()
                                and str(rec.get("provider_type", "") or "").lower() == "drone"
                            ):
                                fanout_tls = _f2d_context_fanout_tls(rec, str(requester_tls)) or [str(requester_tls)]
                            directed_delivery = bool(
                                _f2d_directed_context_delivery_enabled()
                                and str(rec.get("provider_type", "") or "").lower() == "drone"
                                and not _f2d_contextual_topic_delivery_enabled()
                            )
                            if _f2d_contextual_topic_delivery_enabled() and str(rec.get("provider_type", "") or "").lower() == "drone":
                                _fed_evt_main(
                                    "f2d.context.provider_observed_contextual_delivery",
                                    **_drone_context_trace_payload(
                                        rec=rec,
                                        tls_id=str(requester_tls),
                                        ev_id=str(ctx_ev_id),
                                        sim_time=float(_cmd_sim_time),
                                        selected_action="observe_provider_context_wait_for_contextual_topics",
                                        decision_source="mqtt_context_rx",
                                        reason=str(payload.get("reason", "")),
                                        context_age_sec=0.0,
                                        extra={
                                            "fanout_tls": list(fanout_tls),
                                            "fanout_tls_n": int(len(fanout_tls)),
                                            "contextual_topic_delivery": True,
                                            "generic_provider_stream_trace_only": True,
                                        },
                                    ),
                                )
                            for fanout_tls_id in ([] if _f2d_contextual_topic_delivery_enabled() else list(fanout_tls)):
                                rec_for_tls = dict(rec)
                                rec_for_tls["_fanout_from_tls"] = str(requester_tls)
                                rec_for_tls["_fanout_tls"] = str(fanout_tls_id)
                                rec_for_tls["target_tls_id"] = str(fanout_tls_id)
                                rec_for_tls["fanout_tls_n"] = int(len(fanout_tls))
                                rec_for_tls["delivery_model"] = (
                                    "f2d_directed_si_context" if directed_delivery else "legacy_in_process_context_fanout"
                                )
                                rec_for_tls["context_request_model"] = str(
                                    rec_for_tls.get("context_request_model", "f2d_drone_scouting")
                                )
                                if directed_delivery:
                                    directed_publish_wall_ts = float(time.time())
                                    rec_for_tls["directed_publish_wall_ts"] = float(directed_publish_wall_ts)
                                    rec_for_tls["directed_publish_wall_ms"] = float(directed_publish_wall_ts * 1000.0)
                                    rec_for_tls["directed_publish_sim_time"] = float(_cmd_sim_time)
                                    directed_topic = f"federation/v1/context/downstream/si/{fanout_tls_id}"
                                    try:
                                        rec_for_tls["directed_payload_size_bytes"] = int(
                                            len(json.dumps(rec_for_tls, ensure_ascii=True).encode("utf-8"))
                                        )
                                    except Exception:
                                        rec_for_tls["directed_payload_size_bytes"] = 0
                                    client.publish(directed_topic, json.dumps(rec_for_tls))
                                    _fed_evt_main(
                                        "f2d.context.directed_publish",
                                        **_drone_context_trace_payload(
                                            rec=rec_for_tls,
                                            tls_id=str(fanout_tls_id),
                                            ev_id=str(ctx_ev_id),
                                            sim_time=float(_cmd_sim_time),
                                            selected_action="publish_directed_context_to_si_dt",
                                            decision_source="mqtt_context_rx",
                                            reason=str(payload.get("reason", "")),
                                            context_age_sec=0.0,
                                            extra={
                                                "requester_tls": str(requester_tls),
                                                "fanout_from_tls": str(requester_tls),
                                                "fanout_tls": str(fanout_tls_id),
                                                "target_tls_id": str(fanout_tls_id),
                                                "fanout_tls_n": int(len(fanout_tls)),
                                                "directed_topic": str(directed_topic),
                                                "directed_publish_wall_ts": float(directed_publish_wall_ts),
                                                "directed_payload_size_bytes": int(
                                                    rec_for_tls.get("directed_payload_size_bytes", 0) or 0
                                                ),
                                                "source_equivalent": str(
                                                    rec_for_tls.get(
                                                        "source_equivalent",
                                                        rec_for_tls.get("context_source_equivalent", "mobile_passive_dt"),
                                                    )
                                                ),
                                            },
                                        ),
                                    )
                                    if str(fanout_tls_id) != str(requester_tls):
                                        _fed_evt_main(
                                            "f2d.mobile_passive.context_fanout",
                                            **_drone_context_trace_payload(
                                                rec=rec_for_tls,
                                                tls_id=str(fanout_tls_id),
                                                ev_id=str(ctx_ev_id),
                                                sim_time=float(_cmd_sim_time),
                                                selected_action="publish_context_for_downstream_tls",
                                                decision_source="mqtt_context_rx",
                                                reason=str(payload.get("reason", "")),
                                                context_age_sec=0.0,
                                                extra={
                                                    "requester_tls": str(requester_tls),
                                                    "fanout_from_tls": str(requester_tls),
                                                    "fanout_tls": str(fanout_tls_id),
                                                    "fanout_tls_n": int(len(fanout_tls)),
                                                    "delivery_model": "f2d_directed_si_context",
                                                },
                                            ),
                                        )
                                else:
                                    _cache_directed_external_downstream_context(
                                        rec=rec_for_tls,
                                        tls_id=str(fanout_tls_id),
                                        ev_id=str(ctx_ev_id),
                                        requester_tls=str(requester_tls),
                                        sim_time=float(_cmd_sim_time),
                                        decision_source="mqtt_context_rx",
                                        selected_action="cache_context_for_downstream_tls",
                                    )
                                    if (
                                        _f2d_mobile_passive_context_enabled()
                                        and str(fanout_tls_id) != str(requester_tls)
                                    ):
                                        _fed_evt_main(
                                            "f2d.mobile_passive.context_fanout",
                                            **_drone_context_trace_payload(
                                                rec=rec_for_tls,
                                                tls_id=str(fanout_tls_id),
                                                ev_id=str(ctx_ev_id),
                                                sim_time=float(_cmd_sim_time),
                                                selected_action="cache_context_for_downstream_tls",
                                                decision_source="mqtt_context_rx",
                                                reason=str(payload.get("reason", "")),
                                                context_age_sec=0.0,
                                                extra={
                                                    "requester_tls": str(requester_tls),
                                                    "fanout_from_tls": str(requester_tls),
                                                    "fanout_tls": str(fanout_tls_id),
                                                    "fanout_tls_n": int(len(fanout_tls)),
                                                    "delivery_model": "legacy_in_process_context_fanout",
                                                },
                                            ),
                                        )
                            _fed_evt_main(
                                "downstream_context.external_rx",
                                role="intersection",
                                provider_id=str(provider_id),
                                provider_type=str(payload.get("provider_type", "")),
                                requester_tls=str(requester_tls),
                                ev_id=str(ctx_ev_id),
                                sim_time=float(_cmd_sim_time),
                                request_id=str(payload.get("request_id", "")),
                                blocked=bool(payload.get("blocked", False)),
                                reason=str(payload.get("reason", "")),
                                worst_edge=str(payload.get("worst_edge", "")),
                                worst_edge_offset=int(payload.get("worst_edge_offset", -1) or -1),
                                max_halt_n=int(payload.get("max_halt_n", 0) or 0),
                                max_veh_n=int(payload.get("max_veh_n", 0) or 0),
                                max_occupancy_pct=float(payload.get("max_occupancy_pct", 0.0) or 0.0),
                                min_mean_speed_mps=float(payload.get("min_mean_speed_mps", -1.0) or -1.0),
                                ttl_sec=float(payload.get("ttl_sec", 2.0) or 2.0),
                                f2d_mobile_passive_context=bool(_f2d_mobile_passive_context_enabled()),
                                f2d_directed_context_delivery=bool(directed_delivery),
                                fanout_tls=list(fanout_tls),
                                fanout_tls_n=int(len(fanout_tls)),
                            )
                            _fed_evt_main(
                                "f2.drone_context.received",
                                **_drone_context_trace_payload(
                                    rec=rec,
                                    tls_id=str(requester_tls),
                                    ev_id=str(ctx_ev_id),
                                    sim_time=float(_cmd_sim_time),
                                    selected_action=(
                                        "receive_provider_context"
                                        if bool(directed_delivery)
                                        else "cache_context"
                                    ),
                                    decision_source="mqtt_context_rx",
                                    reason=str(payload.get("reason", "")),
                                    context_age_sec=0.0,
                                    extra={
                                        "f2d_mobile_passive_context": bool(_f2d_mobile_passive_context_enabled()),
                                        "f2d_directed_context_delivery": bool(directed_delivery),
                                        "fanout_tls": list(fanout_tls),
                                        "fanout_tls_n": int(len(fanout_tls)),
                                    },
                                ),
                            )
                            if _f2d_mobile_passive_context_enabled():
                                _fed_evt_main(
                                    "f2d.mobile_passive.received",
                                    **_drone_context_trace_payload(
                                        rec=rec,
                                        tls_id=str(requester_tls),
                                        ev_id=str(ctx_ev_id),
                                        sim_time=float(_cmd_sim_time),
                                        selected_action=(
                                            "receive_mobile_passive_provider_context"
                                            if bool(directed_delivery)
                                            else "cache_mobile_passive_context"
                                        ),
                                        decision_source="mqtt_context_rx",
                                        reason=str(payload.get("reason", "")),
                                        context_age_sec=0.0,
                                        extra={
                                            "fanout_tls": list(fanout_tls),
                                            "fanout_tls_n": int(len(fanout_tls)),
                                            "source_equivalent": str(
                                                rec.get(
                                                    "source_equivalent",
                                                    rec.get("context_source_equivalent", "mobile_passive_dt"),
                                                )
                                            ),
                                        },
                                    ),
                                )
                        else:
                            _fed_dbg_main(
                                f"evt=DOWNSTREAM_CONTEXT_DROP reason=missing_requester_tls provider={provider_id}"
                            )

                    elif topic.startswith("federation/v1/state/intersection/"):
                        src_tls = topic.split("/")[-1]
                        fanout = 0
                        fanout_tls = _passive_context_fanout_tls(dict(payload or {}))
                        for dst_tls in list(fanout_tls):
                            ag_dst = agents.get(str(dst_tls))
                            if ag_dst is None:
                                continue
                            if str(dst_tls) == str(src_tls):
                                continue
                            try:
                                if str(src_tls) not in set(str(k) for k in getattr(ag_dst, "neighbor_map", {}).keys()):
                                    continue
                            except Exception:
                                pass
                            try:
                                try:
                                    _sim_now = float(traci.simulation.getTime())
                                except Exception:
                                    _sim_now = float(step) * float(getattr(args, "step_length", 0.1))
                                ag_dst.on_neighbor_state(
                                    source_tls=str(src_tls),
                                    payload=dict(payload or {}),
                                    sim_time=float(_sim_now),
                                )
                                fanout += 1
                            except Exception as e:
                                _fed_dbg_main(
                                    f"evt=NEIGHBOR_STATE_ERR src={src_tls} dst={dst_tls} err={type(e).__name__}:{e}"
                                )
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=neighbor_state src={src_tls} fanout={fanout}"
                        )

                    elif topic.startswith("federation/v1/state/passive_intersection/"):
                        src_node = topic.split("/")[-1]
                        if not _is_passive_dt_mode(CURRENT_EVALUATION):
                            _fed_evt_main(
                                "f2.passive_context.rejected",
                                role="intersection",
                                source_node=str(src_node),
                                sim_time=float(_cmd_sim_time),
                                mode=str(CURRENT_EVALUATION),
                                reason="passive_mode_disabled",
                                selected_action="ignore",
                                decision_source="real_world_mqtt_dispatch_guard",
                            )
                            _fed_dbg_main(
                                f"evt=PASSIVE_CONTEXT_REJECT src={src_node} mode={CURRENT_EVALUATION} "
                                "reason=passive_mode_disabled"
                            )
                            continue
                        fanout = 0
                        for dst_tls, ag_dst in agents.items():
                            if ag_dst is None:
                                continue
                            try:
                                try:
                                    _sim_now = float(traci.simulation.getTime())
                                except Exception:
                                    _sim_now = float(step) * float(getattr(args, "step_length", 0.1))
                                ag_dst.on_passive_context(
                                    source_node=str(src_node),
                                    payload=dict(payload or {}),
                                    sim_time=float(_sim_now),
                                )
                                fanout += 1
                            except Exception as e:
                                _fed_dbg_main(
                                    f"evt=PASSIVE_CONTEXT_ERR src={src_node} dst={dst_tls} err={type(e).__name__}:{e}"
                                )
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=passive_context src={src_node} "
                            f"fanout={fanout} route_fanout={1 if bool(getattr(args, 'passive_intersection_context_route_fanout_enable', True)) else 0}"
                        )
                        if _f2p_queue_release_enabled() and bool(payload.get("blocked", False)):
                            queue_release_applied, queue_release_diag = _try_apply_f2p_queue_release(
                                payload=dict(payload or {}),
                                source_node=str(src_node),
                                ev_id=str(payload.get("ev_id", ev_id) or ev_id),
                                sim_time=float(_cmd_sim_time),
                            )
                            _fed_evt_main(
                                "f2p.queue_release.requested",
                                role="intersection",
                                source_node=str(src_node),
                                ev_id=str(payload.get("ev_id", ev_id) or ev_id),
                                tls_id=str(
                                    dict(queue_release_diag or {}).get(
                                        "queue_release_target_tls",
                                        dict(queue_release_diag or {}).get("worst_edge_tls", ""),
                                    )
                                ),
                                sim_time=float(_cmd_sim_time),
                                blocked=bool(payload.get("blocked", False)),
                                reason=str(payload.get("reason", "")),
                                worst_edge=str(payload.get("worst_edge", "")),
                                worst_edge_offset=int(payload.get("worst_edge_offset", -1) or -1),
                                queue_release_applied=bool(queue_release_applied),
                                selected_action="queue_release_downstream_tls",
                                decision_source="f2p_passive_context",
                                **dict(queue_release_diag or {}),
                            )

                    elif topic == fed_bootstrap_register_topic:
                        gw = str(payload.get("gateway_id", "") or "")
                        node_id = str(payload.get("node_id", "") or "")
                        status_raw = str(payload.get("status", "REGISTERED") or "REGISTERED")
                        status = str(status_raw).strip().upper() or "REGISTERED"
                        if gw:
                            fed_bootstrap_member_status_by_gateway[str(gw)] = str(status)
                        _fed_dbg_main(
                            f"evt=FED_MEMBERSHIP_SEEN topic={topic} kind=register gateway={gw or '-'} "
                            f"node={node_id or '-'} status={status or '-'}"
                        )
                    elif topic == fed_bootstrap_heartbeat_topic:
                        gw = str(payload.get("gateway_id", "") or "")
                        node_id = str(payload.get("node_id", "") or "")
                        status_raw = str(payload.get("status", "ALIVE") or "ALIVE")
                        status = str(status_raw).strip().upper() or "ALIVE"
                        if gw:
                            fed_bootstrap_member_status_by_gateway[str(gw)] = str(status)
                        _fed_dbg_main(
                            f"evt=FED_MEMBERSHIP_SEEN topic={topic} kind=heartbeat gateway={gw or '-'} "
                            f"node={node_id or '-'} status={status or '-'}"
                        )
                    elif topic == "federation/membership/events":
                        gw = str(payload.get("gateway_id", "") or "")
                        status_raw = str(payload.get("status", "") or "")
                        status = str(status_raw).strip().upper()
                        if gw and status:
                            fed_bootstrap_member_status_by_gateway[str(gw)] = str(status)
                            _fed_dbg_main(
                                f"evt=FED_MEMBERSHIP_SEEN topic={topic} kind=event gateway={gw} status={status}"
                            )
                    elif topic == "federation/membership/state":
                        n_upd = 0
                        for mem in list(payload.get("members", []) or []):
                            gw = str(mem.get("gateway_id", "") or "")
                            status_raw = str(mem.get("status", "") or "")
                            status = str(status_raw).strip().upper()
                            if gw and status:
                                fed_bootstrap_member_status_by_gateway[str(gw)] = str(status)
                                n_upd += 1
                        if n_upd > 0:
                            _fed_dbg_main(
                                f"evt=FED_MEMBERSHIP_SEEN topic={topic} kind=state members_updated={n_upd}"
                            )

                    elif topic.startswith("federation/reservation/req/"):
                        dst_tls = topic.split("/")[-1]
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="federation_reservation_req", sim_time=float(step), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        if _drop_foreign_ev(topic, payload, kind="reservation_req", dst_tls=str(dst_tls)):
                            continue
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
                        if _drop_foreign_ev(topic, payload, kind="reservation_resp", dst_tls=str(dst_tls)):
                            continue
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
                        if _drop_foreign_ev(topic, payload, kind="handoff", dst_tls=str(dst_tls)):
                            continue
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
                        if _drop_foreign_ev(topic, payload, kind="corridor_advice", dst_tls=str(dst_tls)):
                            continue
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
                        if _drop_foreign_ev(topic, payload, kind="corridor_verdict", dst_tls=str(dst_tls)):
                            continue
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=corridor_verdict dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} req_id={payload.get('req_id')}"
                        )
                        if ag_dst is not None:
                            ag_dst.on_corridor_verdict(payload)

                    elif topic.startswith(f"{ev_request_topic_prefix}/"):
                        dst_tls = topic.split("/")[-1]
                        ev_req_pipeline_stats["rx_total"] = int(ev_req_pipeline_stats.get("rx_total", 0)) + 1
                        ev_id_rx = str(payload.get("ev_id", payload.get("evId", "")) or "")
                        filtered_dst_tls, filtered_dst_reasons = _fed_filter_tls_candidates(
                            [str(dst_tls)],
                            context="ev_request_mqtt_dispatch",
                            ev_id_ctx=str(ev_id_rx),
                            sim_time_ctx=float(_cmd_sim_time),
                        )
                        if not filtered_dst_tls:
                            ev_req_pipeline_stats["drop_discovery_filter"] = int(
                                ev_req_pipeline_stats.get("drop_discovery_filter", 0)
                            ) + 1
                            _fed_evt_main(
                                "ev.request.drop",
                                role="ev",
                                reason="discovery_target_filter",
                                tls_id=str(dst_tls),
                                ev_id=str(ev_id_rx),
                                source_service=str(payload.get("source_service", "unknown") or "unknown"),
                            )
                            continue
                        dst_reason = str(filtered_dst_reasons.get(str(dst_tls), "") or "")
                        ag_dst = agents.get(dst_tls)
                        if ag_dst is None and on_demand_agent_activation:
                            _activate_tls_agents([str(dst_tls)], reason="ev_request_mqtt", sim_time=float(_cmd_sim_time), max_new=1)
                            ag_dst = agents.get(dst_tls)
                        if _drop_foreign_ev(topic, payload, kind="ev_request", dst_tls=str(dst_tls)):
                            continue
                        _fed_dbg_main(
                            f"evt=RX_DISPATCH topic={topic} kind=ev_request dst={dst_tls} "
                            f"agent_found={1 if ag_dst is not None else 0} ev={payload.get('ev_id') or payload.get('evId')} "
                            f"discovery_reason={dst_reason or '-'}"
                        )
                        ev_req_payload = dict(payload or {})
                        ev_req_det_meta = dict(ev_req_payload.pop("_rw_det_meta", {}) or {})
                        ev_req_det_released = bool(ev_req_payload.pop("_rw_deterministic_release", False))
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
                        if b1_strict_local_baseline:
                            try:
                                inbound_map_strict = dict(getattr(ag_dst, "_inbound_edge_to_phase", {}) or {})
                            except Exception:
                                inbound_map_strict = {}
                            current_edge_strict = str(in_edge_in or "")
                            if (not current_edge_strict or current_edge_strict.startswith(":")) and ev_id_in and traci is not None:
                                try:
                                    traci_edge = str(traci.vehicle.getRoadID(str(ev_id_in)) or "")
                                    if traci_edge and not traci_edge.startswith(":"):
                                        current_edge_strict = str(traci_edge)
                                except Exception:
                                    pass
                            if not current_edge_strict or inbound_map_strict.get(str(current_edge_strict)) is None:
                                ev_req_pipeline_stats["drop_b1_strict_nonlocal_tls"] = int(
                                    ev_req_pipeline_stats.get("drop_b1_strict_nonlocal_tls", 0)
                                ) + 1
                                _fed_dbg_main(
                                    f"evt=B1_STRICT_EV_REQUEST_DROP ev={ev_id_in or '-'} tls={dst_tls} "
                                    f"reason=nonlocal_or_non_tls_current_edge current_edge={current_edge_strict or '-'} "
                                    f"dispatch_reason=mqtt_queue"
                                )
                                _fed_evt_main(
                                    "b1.local_discovery.request_drop",
                                    role="ev",
                                    ev_id=str(ev_id_in),
                                    tls_id=str(dst_tls),
                                    sim_time=float(_cmd_sim_time),
                                    reason="nonlocal_or_non_tls_current_edge",
                                    current_edge=str(current_edge_strict or ""),
                                    dispatch_reason="mqtt_queue",
                                    source_service=str(ev_req_payload.get("source_service", "unknown") or "unknown"),
                                )
                                continue
                            ev_req_payload["in_edge_id"] = str(current_edge_strict)
                            ev_req_payload["route_intersections"] = []
                            ev_req_payload["route_veh"] = []
                            ev_req_payload["source_tag"] = (
                                f"{str(ev_req_payload.get('source_tag', ev_request_source_tag) or '')}:strict_b1_local"
                            ).strip(":")
                            in_edge_in = str(current_edge_strict)
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
                        det_queue_delay_sim_s = None
                        if ev_req_det_released and ev_req_det_meta:
                            try:
                                det_queue_delay_sim_s = max(
                                    0.0,
                                    float(_sim_now_for_req) - float(ev_req_det_meta.get("enqueue_sim_time", _sim_now_for_req)),
                                )
                            except Exception:
                                det_queue_delay_sim_s = None
                        _fed_dbg_main(
                            f"evt=EV_REQ_PIPE ev={ev_msg_in.ev_id} tls={dst_tls} src={ev_req_pipeline_last.get('source')} "
                            f"src_tag={ev_req_pipeline_last.get('source_tag') or '-'} "
                            f"dist={float(ev_msg_in.distance_to_intersection_m):.2f} speed={float(ev_msg_in.speed_mps):.2f} "
                            f"edge={ev_msg_in.in_edge_id or '-'} age_ms={float(age_ms):.1f} "
                            f"det_barrier={1 if ev_req_det_released else 0} "
                            f"det_seq={int(ev_req_det_meta.get('seq', 0) or 0) if ev_req_det_meta else 0} "
                            f"det_queue_sim_s={('-1' if det_queue_delay_sim_s is None else f'{float(det_queue_delay_sim_s):.3f}')}"
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
                            request_sim_time=float(ev_msg_in.sim_time),
                            dispatch_sim_time=float(_sim_now_for_req),
                            deterministic_barrier=bool(ev_req_det_released),
                            barrier_seq=int(ev_req_det_meta.get("seq", 0) or 0) if ev_req_det_meta else 0,
                            barrier_enqueue_sim_time=float(ev_req_det_meta.get("enqueue_sim_time", -1.0) or -1.0) if ev_req_det_meta else -1.0,
                            barrier_release_sim_time=float(ev_req_det_meta.get("release_sim_time", -1.0) or -1.0) if ev_req_det_meta else -1.0,
                            barrier_queue_delay_sim_s=det_queue_delay_sim_s,
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
                                cand_tls, _cand_reasons = _fed_filter_tls_candidates(
                                    list(cand_tls),
                                    context="corridor_route_advice_activation",
                                    ev_id_ctx=str(advised_ev_id),
                                    sim_time_ctx=float(step),
                                )
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
                        drone_discovery_gate_enabled = bool(
                            getattr(args, "f2_drone_context_discovery_gate_enable", False)
                            and _is_drone_augmented_mode(CURRENT_EVALUATION)
                        )
                        purpose = str(payload.get("purpose", "") or "")
                        query = dict(payload.get("query", payload.get("filters", {})) or {})
                        query_provider_type = str(query.get("provider_type", payload.get("provider_type", "")) or "")
                        query_caps_raw = query.get("capability", query.get("capabilities", query.get("capability_names", [])))
                        if isinstance(query_caps_raw, str):
                            query_caps = {query_caps_raw}
                        else:
                            query_caps = {str(x) for x in list(query_caps_raw or []) if str(x)}
                        is_drone_discovery_response = bool(
                            purpose == "drone_downstream_context_discovery"
                            or query_provider_type == "drone"
                            or "downstream_context_provider" in query_caps
                        )
                        if drone_discovery_gate_enabled and is_drone_discovery_response:
                            requester_tls = str(
                                payload.get("requester_tls", payload.get("requester_node_id", payload.get("requester", "")))
                                or topic.split("/")[-1]
                                or ""
                            )
                            ag_req = agents.get(str(requester_tls))
                            if ag_req is not None:
                                try:
                                    ag_req.on_drone_discovery_response(dict(payload or {}), sim_time=float(_cmd_sim_time))
                                    _fed_evt_main(
                                        "f2.drone_context.discovery_response_delivered",
                                        role="intersection",
                                        requester_tls=str(requester_tls),
                                        request_id=str(payload.get("request_id", "") or ""),
                                        n_results=int(payload.get("n_results", len(list(payload.get("results", []) or []))) or 0),
                                        latency_ms=float(payload.get("latency_ms", -1.0) or -1.0),
                                        sim_time=float(_cmd_sim_time),
                                        selected_action="cache_drone_discovery_response",
                                        decision_source="real_world_mqtt_dispatch",
                                    )
                                    _fed_dbg_main(
                                        f"evt=DRONE_DISCOVERY_RESP_DELIVERED requester_tls={requester_tls} "
                                        f"req_id={payload.get('request_id', '')} n={payload.get('n_results', '-')}"
                                    )
                                except Exception as e:
                                    _fed_dbg_main(
                                        f"evt=DRONE_DISCOVERY_RESP_ERR requester_tls={requester_tls} "
                                        f"err={type(e).__name__}:{e}"
                                    )
                            else:
                                _fed_dbg_main(
                                    f"evt=DRONE_DISCOVERY_RESP_DROP reason=unknown_requester_tls "
                                    f"requester_tls={requester_tls or '-'} topic={topic}"
                                )
                            continue
                        # In decentralized mode, discovery authority belongs to FNM/FCM.
                        # Real-world must not ingest discovery responses nor update any
                        # local discovery cache/state, otherwise it can silently interfere
                        # with peer gating and produce misleading logs.
                        if str(fed_peer_selection_source) != "realworld":
                            _fed_dbg_main(
                                f"evt=FED_BOOTSTRAP_DISCOVERY_RESP_IGNORED "
                                f"reason=peer_selection_source_{fed_peer_selection_source} topic={topic}"
                            )
                            continue
                        req_id = str(payload.get("request_id", "") or "")
                        n_results = int(payload.get("n_results", 0) or 0)
                        resp_wall = float(time.time())
                        sent_wall = float(fed_bootstrap_discovery_req_sent_wall.pop(str(req_id), 0.0) or 0.0)
                        roundtrip_ms = (1000.0 * (resp_wall - sent_wall)) if sent_wall > 0.0 else -1.0
                        accepted_n = 0
                        rejected_n = 0
                        for res in list(payload.get("results", []) or []):
                            if not isinstance(res, dict):
                                continue
                            role = str(res.get("role", "") or "")
                            if fed_bootstrap_discovery_target_role and str(role) != str(fed_bootstrap_discovery_target_role):
                                continue
                            tls_id = str(res.get("node_id", "") or "")
                            gw = str(res.get("gateway_id", "") or "")
                            if not tls_id:
                                continue
                            member_ok = True
                            member_status = str(fed_bootstrap_member_status_by_gateway.get(str(gw), "") or "").upper()
                            if bool(fed_bootstrap_discovery_require_membership_valid):
                                member_ok = bool(gw) and _fed_member_status_active(member_status)
                            if member_ok:
                                fed_bootstrap_discovery_tls_last_seen_wall[str(tls_id)] = float(resp_wall)
                                fed_bootstrap_discovery_tls_gateway[str(tls_id)] = str(gw)
                                fed_bootstrap_discovery_tls_last_req_id[str(tls_id)] = str(req_id)
                                accepted_n += 1
                            else:
                                rejected_n += 1
                        _fed_evt_main(
                            "discovery.query.response",
                            role="ev",
                            request_id=str(req_id),
                            n_results=int(n_results),
                            accepted_n=int(accepted_n),
                            rejected_n=int(rejected_n),
                            roundtrip_ms=float(roundtrip_ms),
                        )
                        _fed_dbg_main(
                            f"evt=FED_BOOTSTRAP_DISCOVERY_RESP topic={topic} req_id={req_id} "
                            f"n_results={n_results} accepted_n={accepted_n} rejected_n={rejected_n} "
                            f"rt_ms={roundtrip_ms:.1f}"
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
            if passive_intersections and _periodic_due(
                float(sim_time),
                passive_context_period_state,
                "passive_context",
                float(getattr(args, "passive_intersection_context_period_sec", 1.0) or 1.0),
            ):
                for node_id, passive_dt in list(passive_intersections.items()):
                    try:
                        passive_payload = passive_dt.build_context(
                            sim_time=float(sim_time),
                            ev_id=str(ev_id),
                            min_halt_n=int(getattr(args, "f2_downstream_apply_guard_min_halt_n", 3)),
                            max_mean_speed_mps=float(getattr(args, "f2_downstream_apply_guard_max_mean_speed_mps", 1.0)),
                            min_veh_n=int(getattr(args, "f2_downstream_apply_guard_min_veh_n", 3)),
                            max_occupancy_pct=float(getattr(args, "f2_downstream_apply_guard_max_occupancy_pct", 35.0)),
                        )
                        topic_passive = f"federation/v1/state/passive_intersection/{node_id}"
                        client.publish(topic_passive, json.dumps(passive_payload))
                        fanout_tls = _passive_context_fanout_tls(dict(passive_payload))
                        fanout_n = 0
                        for _dst_tls in list(fanout_tls):
                            _ag_dst = agents.get(str(_dst_tls))
                            if _ag_dst is None:
                                continue
                            try:
                                _ag_dst.on_passive_context(
                                    source_node=str(node_id),
                                    payload=dict(passive_payload),
                                    sim_time=float(sim_time),
                                )
                                fanout_n += 1
                            except Exception:
                                pass
                        _fed_evt_main(
                            "passive_intersection.context_pub",
                            role="passive_intersection",
                            node_id=str(node_id),
                            ev_id=str(ev_id),
                            sim_time=float(sim_time),
                            topic=str(topic_passive),
                            target_edges=list(passive_payload.get("target_edges", []) or []),
                            blocked=bool(passive_payload.get("blocked", False)),
                            reason=str(passive_payload.get("reason", "")),
                            worst_edge=str(passive_payload.get("worst_edge", "")),
                            max_halt_n=int(passive_payload.get("max_halt_n", 0) or 0),
                            fanout_tls=list(fanout_tls),
                            fanout_tls_n=int(fanout_n),
                            route_fanout_enabled=bool(
                                getattr(args, "passive_intersection_context_route_fanout_enable", True)
                            ),
                            can_actuate=False,
                            can_coordinate=False,
                            can_observe_downstream=True,
                        )
                        if _f2p_queue_release_enabled() and bool(passive_payload.get("blocked", False)):
                            queue_release_applied, queue_release_diag = _try_apply_f2p_queue_release(
                                payload=dict(passive_payload),
                                source_node=str(node_id),
                                ev_id=str(ev_id),
                                sim_time=float(sim_time),
                            )
                            _fed_evt_main(
                                "f2p.queue_release.requested",
                                role="passive_intersection",
                                source_node=str(node_id),
                                ev_id=str(ev_id),
                                tls_id=str(
                                    dict(queue_release_diag or {}).get(
                                        "queue_release_target_tls",
                                        dict(queue_release_diag or {}).get("worst_edge_tls", ""),
                                    )
                                ),
                                sim_time=float(sim_time),
                                blocked=bool(passive_payload.get("blocked", False)),
                                reason=str(passive_payload.get("reason", "")),
                                worst_edge=str(passive_payload.get("worst_edge", "")),
                                worst_edge_offset=int(passive_payload.get("worst_edge_offset", -1) or -1),
                                queue_release_applied=bool(queue_release_applied),
                                selected_action="queue_release_downstream_tls",
                                decision_source="f2p_passive_context",
                                **dict(queue_release_diag or {}),
                            )
                    except Exception as e:
                        _fed_dbg_main(
                            f"evt=PASSIVE_CONTEXT_PUB_ERR node={node_id} err={type(e).__name__}:{e}"
                        )
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
            if max_sim_time_sec > 0.0 and float(sim_time) >= float(max_sim_time_sec):
                try:
                    veh_ids_now = set(str(x) for x in traci.vehicle.getIDList())
                except Exception:
                    veh_ids_now = set()
                ev_live_now = str(ev_id) in veh_ids_now
                ev_edge_now = "-"
                ev_speed_now = -1.0
                if ev_live_now:
                    try:
                        ev_edge_now = str(traci.vehicle.getRoadID(str(ev_id)))
                    except Exception:
                        ev_edge_now = "-"
                    try:
                        ev_speed_now = float(traci.vehicle.getSpeed(str(ev_id)))
                    except Exception:
                        ev_speed_now = -1.0
                _fed_dbg_main(
                    f"evt=EV_MAX_SIM_TIME_STOP ev={ev_id} sim={sim_time:.2f} "
                    f"max_sim={float(max_sim_time_sec):.2f} step={step} "
                    f"seen_once={int(ev_seen_once)} live={int(ev_live_now)} "
                    f"edge={ev_edge_now} speed={float(ev_speed_now):.2f}"
                )
                _fed_evt_main(
                    "ev.max_sim_time.stop",
                    role="ev",
                    ev_id=str(ev_id),
                    sim_time=float(sim_time),
                    max_sim_time_sec=float(max_sim_time_sec),
                    step=int(step),
                    seen_once=bool(ev_seen_once),
                    live=bool(ev_live_now),
                    edge=str(ev_edge_now),
                    speed_mps=float(ev_speed_now),
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
                                "reason": "max_sim_time",
                                "maxSimTimeSec": float(max_sim_time_sec),
                                "seenOnce": bool(ev_seen_once),
                                "live": bool(ev_live_now),
                                "edge": str(ev_edge_now),
                                "speedMps": float(ev_speed_now),
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
                            _disc_reason = str(req_http.get("discovery_target_reason", "") or "")
                            try:
                                client.publish(topic_http, json.dumps(req_http))
                                n_pub += 1
                                if _disc_reason:
                                    _fed_dbg_main(
                                        f"evt=EV_HTTP_ADAPTER_TARGET tls={stls} discovery_reason={_disc_reason}"
                                    )
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
                now_bootstrap = _fed_bootstrap_now(sim_time)
                try:
                    # Register any newly appeared virtual participants (e.g., on-demand TLS activation).
                    _fed_bootstrap_publish_register_all(force=False)
                    if (now_bootstrap - float(fed_bootstrap_last_heartbeat_tick)) >= float(fed_bootstrap_heartbeat_sec):
                        _fed_bootstrap_publish_heartbeat_all()
                        fed_bootstrap_last_heartbeat_tick = now_bootstrap
                    if (now_bootstrap - float(fed_bootstrap_last_catalog_tick)) >= float(fed_bootstrap_catalog_sec):
                        _fed_bootstrap_publish_catalog_all()
                        fed_bootstrap_last_catalog_tick = now_bootstrap
                    if fed_bootstrap_discovery_probe_sec > 0.0 and (now_bootstrap - float(fed_bootstrap_last_probe_tick)) >= float(fed_bootstrap_discovery_probe_sec):
                        _fed_bootstrap_publish_probe()
                        fed_bootstrap_last_probe_tick = now_bootstrap
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
                        f"drop_foreign_ev_id={int(ev_req_pipeline_stats.get('drop_foreign_ev_id', 0))} "
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
            if _is_f2_family(CURRENT_EVALUATION) and not STATIC_PROGRAM:
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
                    # Strict B1 intentionally does not use this fallback: a local
                    # EV-to-intersection baseline has nobody to talk to at a non-TLS node.
                    if not tls_candidates:
                        if b1_strict_local_baseline:
                            _fed_evt_main(
                                "b1.local_discovery.attempt",
                                role="ev",
                                ev_id=str(ev_id),
                                sim_time=float(sim_time),
                                ev_edge=str(ev_edge),
                                selected_in_edge=str(selected_in_edge),
                                approach_node=str(approach_node or ""),
                                tls_candidates=[],
                                discovery_scope="current_tls_control_area",
                                discovery_authority="local_current_edge_tls_mapping",
                                can_preempt=False,
                                advisory_only=True,
                                reason="no_tls_at_current_approach",
                            )
                            _fed_dbg_main(
                                f"evt=B1_STRICT_LOCAL_DISCOVERY_MISS ev={ev_id} edge={ev_edge} "
                                f"approach_node={approach_node or '-'} reason=no_tls_at_current_approach"
                            )
                            _fed_evt_main(
                                "b1.local_discovery.miss",
                                role="ev",
                                ev_id=str(ev_id),
                                sim_time=float(sim_time),
                                ev_edge=str(ev_edge),
                                approach_node=str(approach_node or ""),
                                reason="no_tls_at_current_approach",
                                can_preempt=False,
                                advisory_only=True,
                            )
                        else:
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
                    elif b1_strict_local_baseline:
                        _fed_evt_main(
                            "b1.local_discovery.attempt",
                            role="ev",
                            ev_id=str(ev_id),
                            sim_time=float(sim_time),
                            ev_edge=str(ev_edge),
                            selected_in_edge=str(selected_in_edge),
                            approach_node=str(approach_node or ""),
                            tls_candidates=list(tls_candidates),
                            discovery_scope="current_tls_control_area",
                            discovery_authority="local_current_edge_tls_mapping",
                            can_preempt=True,
                            advisory_only=False,
                            reason="tls_at_current_approach",
                        )
                    if tls_candidates:
                        tls_candidates, tls_candidate_reasons = _fed_filter_tls_candidates(
                            list(tls_candidates),
                            context="ev_trigger_target_select",
                            ev_id_ctx=str(ev_id),
                            sim_time_ctx=float(sim_time),
                        )
                    if tls_candidates:
                        tls_id = str(tls_candidates[0])
                        tls_discovery_reason = str(tls_candidate_reasons.get(str(tls_id), "") or "")
                        current_ev_tls_id = str(tls_id)
                        ag = agents.get(tls_id)
                        if b1_strict_local_baseline:
                            _fed_evt_main(
                                "b1.local_discovery.hit",
                                role="ev",
                                ev_id=str(ev_id),
                                tls_id=str(tls_id),
                                sim_time=float(sim_time),
                                ev_edge=str(ev_edge),
                                selected_in_edge=str(selected_in_edge),
                                approach_node=str(approach_node or ""),
                                lookahead_hops=int(lookahead_hops),
                                discovery_scope="current_tls_control_area",
                                discovery_authority="local_current_edge_tls_mapping+federation_filter",
                                discovery_target_reason=str(tls_discovery_reason or ""),
                                agent_found=bool(ag is not None),
                                can_preempt=bool(ag is not None),
                                advisory_only=False,
                            )
                        diag_key = (str(ev_edge_snapshot), str(ev_edge_traci), str(tls_id))
                        if diag_key != ev_trigger_diag_last:
                            ev_trigger_diag_last = diag_key
                            _fed_dbg_main(
                                f"evt=EV_EDGE_SELECT ev={ev_id} mode={ev_edge_source_mode} "
                                f"snapshot_edge={ev_edge_snapshot} traci_edge={ev_edge_traci} chosen_edge={ev_edge_raw} "
                                f"approach_node={approach_node} tls={tls_id} agent_found={1 if ag is not None else 0} "
                                f"selected_in_edge={selected_in_edge} lookahead_hops={lookahead_hops} "
                                f"discovery_reason={tls_discovery_reason or '-'}"
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
                                    _wait_for_fnm_ev_request_if_needed(
                                        ev_id_expected=str(ev_id),
                                        tls_id_expected=str(tls_id),
                                        selected_in_edge=str(selected_in_edge),
                                        ag_obj=ag,
                                        current_sim=float(sim_time),
                                    )
                                    # Keep local decision path alive in bridge/MQTT mode by refreshing
                                    # the currently tracked EV request with high-rate TraCI telemetry.
                                    try:
                                        if ag.active_ev is not None and str(getattr(ag.active_ev, "ev_id", "")) == str(ev_id):
                                            ag.active_ev.sim_time = float(sim_time)
                                            ag.active_ev.speed_mps = float(traci.vehicle.getSpeed(ev_id))
                                            if b1_strict_local_baseline:
                                                _fed_dbg_main(
                                                    f"evt=B1_STRICT_REFRESH_SKIP ev={ev_id} tls={tls_id} "
                                                    f"reason=fnm_request_cadence_authoritative"
                                                )
                                                _fed_evt_main(
                                                    "b1.local_discovery.refresh_skipped",
                                                    role="intersection",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    reason="fnm_request_cadence_authoritative",
                                                )
                                            else:
                                                # Refresh local decision context from SUMO sim state. For route-lookahead
                                                # targets, use route distance to the selected inbound edge stopline instead
                                                # of preserving the latest request payload; otherwise B1/F2 decisions become
                                                # sensitive to MQTT/HTTP request cadence.
                                                if int(lookahead_hops) <= 0 or str(ev_edge) == str(selected_in_edge):
                                                    ag.active_ev.distance_to_intersection_m = float(d_stop)
                                                    refresh_distance_mode = "current_edge"
                                                else:
                                                    route_dist = route_distance_to_edge_stopline(str(ev_id), str(selected_in_edge))
                                                    if route_dist is not None:
                                                        ag.active_ev.distance_to_intersection_m = float(route_dist)
                                                        refresh_distance_mode = "route_distance_to_selected_edge"
                                                    else:
                                                        refresh_distance_mode = "preserve_lookahead_distance_unresolved"
                                                ag.active_ev.in_edge_id = str(selected_in_edge)
                                                _route_now = list(traci.vehicle.getRoute(ev_id) or [])
                                                if _route_now:
                                                    ag.active_ev.route_veh = list(_route_now)
                                                _fed_dbg_main(
                                                    f"evt=EV_TRIGGER_REFRESH ev={ev_id} tls={tls_id} edge={selected_in_edge} "
                                                    f"d_stop={float(d_stop):.2f} speed={float(ag.active_ev.speed_mps):.2f} "
                                                    f"lookahead_hops={int(lookahead_hops)} distance_mode={refresh_distance_mode} "
                                                    f"active_dist={float(getattr(ag.active_ev, 'distance_to_intersection_m', -1.0)):.2f}"
                                                )
                                                _fed_evt_main(
                                                    "ev.decision_context.refresh",
                                                    role="intersection",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    ev_edge=str(ev_edge),
                                                    selected_in_edge=str(selected_in_edge),
                                                    lookahead_hops=int(lookahead_hops),
                                                    distance_mode=str(refresh_distance_mode),
                                                    distance_to_stopline_m=float(d_stop),
                                                    active_distance_m=float(getattr(ag.active_ev, "distance_to_intersection_m", -1.0)),
                                                    speed_mps=float(getattr(ag.active_ev, "speed_mps", -1.0)),
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
                                discovery_scope_edge = (
                                    str(selected_in_edge)
                                    if str(getattr(args, "ev_intersection_discovery_repeat_scope", "edge") or "edge") == "edge"
                                    else ""
                                )
                                discovery_key = (
                                    str(CURRENT_EVALUATION),
                                    str(ev_id),
                                    str(tls_id),
                                    str(discovery_scope_edge),
                                )
                                discovery_delay_sec = max(
                                    0.0,
                                    float(getattr(args, "ev_intersection_discovery_delay_sec", 0.0) or 0.0),
                                )
                                discovery_enabled_for_mode = bool(
                                    getattr(args, "ev_intersection_discovery_enable", False)
                                    and str(CURRENT_EVALUATION).upper() in set(ev_intersection_discovery_modes)
                                )
                                discovery_latency_sec = 0.0
                                if str(CURRENT_EVALUATION).upper() != "B0" and discovery_key not in ev_intersection_discovery_seen:
                                    ev_intersection_discovery_seen.add(discovery_key)
                                    _fed_evt_main(
                                        "ev.intersection.discovery.observed",
                                        role="ev",
                                        ev_id=str(ev_id),
                                        tls_id=str(tls_id),
                                        mode=str(CURRENT_EVALUATION),
                                        sim_time=float(sim_time),
                                        ev_edge=str(ev_edge),
                                        selected_in_edge=str(selected_in_edge),
                                        approach_node=str(approach_node or ""),
                                        distance_to_intersection_m=float(d_stop),
                                        discovery_scope="current_tls_control_area",
                                        repeat_scope=str(getattr(args, "ev_intersection_discovery_repeat_scope", "edge") or "edge"),
                                        gate_enabled=bool(discovery_enabled_for_mode),
                                    )
                                if discovery_enabled_for_mode:
                                    if discovery_key not in ev_intersection_discovery_ready_at:
                                        ready_at = float(sim_time) + float(discovery_delay_sec)
                                        ev_intersection_discovery_ready_at[discovery_key] = float(ready_at)
                                        _fed_evt_main(
                                            "ev.intersection.discovery.query",
                                            role="ev",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            mode=str(CURRENT_EVALUATION),
                                            sim_time=float(sim_time),
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            approach_node=str(approach_node or ""),
                                            distance_to_intersection_m=float(d_stop),
                                            discovery_scope="current_tls_control_area",
                                            discovery_authority="ev_to_current_tls_service_discovery",
                                            expected_response_sim_time=float(ready_at),
                                            discovery_delay_sec=float(discovery_delay_sec),
                                            can_preempt=True,
                                        )
                                    ready_at = float(ev_intersection_discovery_ready_at.get(discovery_key, float(sim_time)))
                                    discovery_latency_sec = max(0.0, float(ready_at) - float(sim_time - discovery_delay_sec))
                                    if float(sim_time) < float(ready_at):
                                        last_wait = float(ev_intersection_discovery_wait_last.get(discovery_key, -1e18))
                                        wait_period = max(
                                            0.0,
                                            float(
                                                getattr(args, "ev_intersection_discovery_wait_log_period_sec", 1.0)
                                                or 1.0
                                            ),
                                        )
                                        if (float(sim_time) - last_wait) >= float(wait_period):
                                            ev_intersection_discovery_wait_last[discovery_key] = float(sim_time)
                                            _fed_evt_main(
                                                "ev.intersection.discovery.wait",
                                                role="ev",
                                                ev_id=str(ev_id),
                                                tls_id=str(tls_id),
                                                mode=str(CURRENT_EVALUATION),
                                                sim_time=float(sim_time),
                                                ev_edge=str(ev_edge),
                                                selected_in_edge=str(selected_in_edge),
                                                approach_node=str(approach_node or ""),
                                                distance_to_intersection_m=float(d_stop),
                                                ready_at_sim_time=float(ready_at),
                                                remaining_sec=max(0.0, float(ready_at) - float(sim_time)),
                                                discovery_scope="current_tls_control_area",
                                                selected_action="hold_ev_request_until_discovered",
                                            )
                                        continue
                                    if discovery_key not in ev_intersection_discovery_response_logged:
                                        ev_intersection_discovery_response_logged.add(discovery_key)
                                        _fed_evt_main(
                                            "ev.intersection.discovery.response",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            mode=str(CURRENT_EVALUATION),
                                            sim_time=float(sim_time),
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            approach_node=str(approach_node or ""),
                                            discovery_scope="current_tls_control_area",
                                            discovery_authority="ev_to_current_tls_service_discovery",
                                            discovery_latency_sec=float(discovery_delay_sec),
                                            can_preempt=True,
                                            accepted=True,
                                        )
                                route_nodes: List[str] = []
                                if (not b1_strict_local_baseline) and not bool(getattr(args, "legacy_ev_request", False)):
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
                                        route_intersections=([] if b1_strict_local_baseline else (list(route_nodes) if route_nodes else None)),
                                        route_veh=([] if b1_strict_local_baseline else ev_route_veh),
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
                                        route_intersections=([] if b1_strict_local_baseline else route_nodes),
                                        route_veh=([] if b1_strict_local_baseline else list(traci.vehicle.getRoute(ev_id)))
                                    )
                                    _fed_dbg_main(
                                        f"evt=EV_MSG_BUILD mode=vehicle_agent ev={ev_id} tls={tls_id} "
                                        f"edge={selected_in_edge} route_nodes={len(route_nodes)} "
                                        f"lookahead_hops={lookahead_hops}"
                                    )
                                print(f"Current distance to tls_id: {tls_id} is {d_stop} m")

                                ev_msg.source_service = "vehicle_agent"
                                ev_msg.source_tag = (
                                    f"{str(ev_request_source_tag or 'direct')}:strict_b1_local"
                                    if b1_strict_local_baseline
                                    else str(ev_request_source_tag or "direct")
                                )
                                ev_msg.delivery = "direct"
                                if b1_strict_local_baseline:
                                    _fed_evt_main(
                                        "b1.local_request.emit",
                                        role="ev",
                                        ev_id=str(ev_msg.ev_id),
                                        tls_id=str(tls_id),
                                        sim_time=float(sim_time),
                                        in_edge_id=str(ev_msg.in_edge_id),
                                        distance_to_intersection_m=float(ev_msg.distance_to_intersection_m),
                                        speed_mps=float(ev_msg.speed_mps),
                                        delivery_mode=str(ev_request_delivery_mode),
                                        discovery_scope="current_tls_control_area",
                                        discovery_gate_enabled=bool(discovery_enabled_for_mode),
                                        discovery_latency_sec=float(discovery_delay_sec if discovery_enabled_for_mode else 0.0),
                                        route_intersections_n=0,
                                        route_veh_n=0,
                                        can_preempt=True,
                                        advisory_only=False,
                                    )

                                if (not skip_internal_emit) and ev_request_delivery_mode in ("mqtt", "both"):
                                    req_topic = f"{ev_request_topic_prefix}/{tls_id}"
                                    req_payload = asdict(ev_msg)
                                    req_payload["tls_id"] = str(tls_id)
                                    req_payload["source_service"] = "vehicle_agent"
                                    req_payload["source_tag"] = str(ev_msg.source_tag)
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

                                _emit_focus_trace(
                                    mode=str(CURRENT_EVALUATION),
                                    stage="post_ev_request",
                                    ev_id=str(ev_id),
                                    tls_id=str(tls_id),
                                    sim_time=float(sim_time),
                                    ag=ag,
                                    selected_in_edge=str(selected_in_edge),
                                    ev_edge=str(ev_edge),
                                    d_stop=float(d_stop),
                                    lookahead_hops=int(lookahead_hops),
                                    route_nodes=list(route_nodes or []),
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

                                    b1_downstream_diag = b1_downstream_blockage_diag(
                                        ev_id=str(ev_id),
                                        current_edge=str(ev_edge),
                                        selected_in_edge=str(selected_in_edge),
                                        lookahead_edges=(
                                            0
                                            if b1_strict_local_baseline
                                            else int(getattr(args, "b1_downstream_blockage_lookahead_edges", 3))
                                        ),
                                        min_halt_n=int(getattr(args, "b1_downstream_blockage_min_halt_n", 3)),
                                        max_mean_speed_mps=float(getattr(args, "b1_downstream_blockage_max_mean_speed_mps", 1.0)),
                                        min_veh_n=int(getattr(args, "b1_downstream_blockage_min_veh_n", 2)),
                                        max_occupancy_pct=float(getattr(args, "b1_downstream_blockage_max_occupancy_pct", 35.0)),
                                    )
                                    b1_downstream_diag = _merge_external_downstream_context(
                                        b1_downstream_diag,
                                        tls_id=str(tls_id),
                                        ev_id=str(ev_id),
                                        sim_time=float(sim_time),
                                        max_age_sec=float(getattr(args, "external_downstream_context_max_age_sec", 2.0) or 2.0),
                                    )
                                    b1_downstream_diag = _annotate_immediate_blockage_diag(b1_downstream_diag)

                                    next_decision_before_b1 = getattr(ag, "_next_decision_time", None)
                                    decision_due_before_b1 = (
                                        (next_decision_before_b1 is None)
                                        or (float(sim_time) >= float(next_decision_before_b1))
                                    )

                                    plan = ag.tick(sim_time)
                                    stage_after_tick_b1 = str(ag.stage)
                                    plan_for_b1_apply, b1_lookahead_diag = _route_lookahead_actuation_filter(
                                        mode="B1",
                                        stage="local_tick",
                                        ev_id=str(ev_id),
                                        tls_id=str(tls_id),
                                        sim_time=float(sim_time),
                                        ev_edge=str(ev_edge),
                                        selected_in_edge=str(selected_in_edge),
                                        lookahead_hops=int(lookahead_hops),
                                        d_stop=float(d_stop),
                                        plan=plan,
                                    )
                                    _b1_route_dist_dbg = b1_lookahead_diag.get("route_distance_to_selected_edge_m")
                                    _b1_route_dist_dbg_s = "NA" if _b1_route_dist_dbg is None else f"{float(_b1_route_dist_dbg):.2f}"

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
                                            f"downstream_blocked={int(bool(b1_downstream_diag.get('blocked', False)))} "
                                            f"downstream_reason={str(b1_downstream_diag.get('reason', ''))} "
                                            f"downstream_worst={str(b1_downstream_diag.get('worst_edge', '')) or '-'} "
                                            f"downstream_offset={int(b1_downstream_diag.get('worst_edge_offset', -1) or -1)} "
                                            f"downstream_halt={int(b1_downstream_diag.get('max_halt_n', 0) or 0)} "
                                            f"downstream_veh={int(b1_downstream_diag.get('max_veh_n', 0) or 0)} "
                                            f"downstream_speed={float(b1_downstream_diag.get('min_mean_speed_mps', -1.0) or -1.0):.2f} "
                                            f"downstream_occ={float(b1_downstream_diag.get('max_occupancy_pct', 0.0) or 0.0):.1f} "
                                            f"immediate_blockage={int(bool(b1_downstream_diag.get('immediate_blockage_severe', False)))} "
                                            f"lookahead_action={str(b1_lookahead_diag.get('action', ''))} "
                                            f"lookahead_reason={str(b1_lookahead_diag.get('reason', ''))} "
                                            f"route_dist={_b1_route_dist_dbg_s} "
                                            f"preemption_eligible={int(bool(b1_lookahead_diag.get('preemption_eligible', True)))} "
                                            f"next_decision_before={('NA' if next_decision_before_b1 is None else f'{float(next_decision_before_b1):.2f}')} "
                                            f"next_decision_after={('NA' if next_decision_after_b1 is None else f'{float(next_decision_after_b1):.2f}')}"
                                        )
                                        _fed_evt_main(
                                            "b1.tick",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            sim_time=float(sim_time),
                                            due=bool(decision_due_before_b1),
                                            distance_to_stopline_m=float(d_stop),
                                            speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                            stage_before=str(stage_before_tick_b1),
                                            stage_after=str(stage_after_tick_b1),
                                            tls_phase=int(tls_phase_before_b1),
                                            target_phase=target_phase_b1,
                                            eta=eta_b1,
                                            arrival_window_start=(None if arr_win_b1 is None else float(arr_win_b1[0])),
                                            arrival_window_end=(None if arr_win_b1 is None else float(arr_win_b1[1])),
                                            base_window_start=(None if base_win_b1 is None else float(base_win_b1[0])),
                                            base_window_end=(None if base_win_b1 is None else float(base_win_b1[1])),
                                            window_cover=window_cover_b1,
                                            clrs=clrs_b1,
                                            tul=tul_b1,
                                            result=str(plan_kind_b1),
                                            plan_type=str(plan_type_b1),
                                            plan_target_phase=str(plan_target_b1),
                                            downstream_blocked=bool(b1_downstream_diag.get("blocked", False)),
                                            downstream_block_reason=str(b1_downstream_diag.get("reason", "")),
                                            downstream_worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                                            downstream_worst_edge_offset=int(b1_downstream_diag.get("worst_edge_offset", -1) or -1),
                                            downstream_lookahead_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                                            downstream_max_halt_n=int(b1_downstream_diag.get("max_halt_n", 0) or 0),
                                            downstream_max_veh_n=int(b1_downstream_diag.get("max_veh_n", 0) or 0),
                                            downstream_max_occupancy_pct=float(b1_downstream_diag.get("max_occupancy_pct", 0.0) or 0.0),
                                            downstream_min_mean_speed_mps=float(b1_downstream_diag.get("min_mean_speed_mps", -1.0) or -1.0),
                                            downstream_immediate_blockage_severe=bool(
                                                b1_downstream_diag.get("immediate_blockage_severe", False)
                                            ),
                                            downstream_immediate_blockage_guard_enabled=bool(
                                                b1_downstream_diag.get("immediate_blockage_guard_enabled", False)
                                            ),
                                            external_downstream_context_used=bool(
                                                b1_downstream_diag.get("external_downstream_context_used", False)
                                            ),
                                            external_downstream_context_provider=str(
                                                b1_downstream_diag.get("external_downstream_context_provider", "")
                                            ),
                                            external_downstream_context_request_id=str(
                                                b1_downstream_diag.get("external_downstream_context_request_id", "")
                                            ),
                                            external_downstream_context_age_sec=float(
                                                b1_downstream_diag.get("external_downstream_context_age_sec", -1.0) or -1.0
                                            ),
                                            lookahead_guard_enabled=bool(b1_lookahead_diag.get("guard_enabled", False)),
                                            route_lookahead=bool(b1_lookahead_diag.get("route_lookahead", False)),
                                            lookahead_hops=int(b1_lookahead_diag.get("lookahead_hops", 0) or 0),
                                            route_distance_to_selected_edge_m=b1_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                            on_selected_in_edge=bool(b1_lookahead_diag.get("on_selected_in_edge", False)),
                                            preemption_eligible=bool(b1_lookahead_diag.get("preemption_eligible", True)),
                                            upstream_stopped=bool(b1_lookahead_diag.get("upstream_stopped", False)),
                                            lookahead_action=str(b1_lookahead_diag.get("action", "")),
                                            lookahead_reason=str(b1_lookahead_diag.get("reason", "")),
                                            lookahead_original_extend_green_sec=float(b1_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                            lookahead_applied_extend_green_sec=float(b1_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                            next_decision_before=next_decision_before_b1,
                                            next_decision_after=next_decision_after_b1,
                                        )
                                    if bool(b1_downstream_diag.get("blocked", False)):
                                        _fed_evt_main(
                                            "b1.downstream_blockage",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            sim_time=float(sim_time),
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            distance_to_stopline_m=float(d_stop),
                                            speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                            plan_type=str(getattr(plan, "plan_type", "")) if plan is not None else "",
                                            guard_enabled=bool(getattr(args, "b1_downstream_blockage_guard_enable", True)),
                                            blocked=bool(b1_downstream_diag.get("blocked", False)),
                                            reason=str(b1_downstream_diag.get("reason", "")),
                                            worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                                            worst_edge_offset=int(b1_downstream_diag.get("worst_edge_offset", -1) or -1),
                                            lookahead_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                                            max_halt_n=int(b1_downstream_diag.get("max_halt_n", 0) or 0),
                                            max_veh_n=int(b1_downstream_diag.get("max_veh_n", 0) or 0),
                                            max_occupancy_pct=float(b1_downstream_diag.get("max_occupancy_pct", 0.0) or 0.0),
                                            min_mean_speed_mps=float(b1_downstream_diag.get("min_mean_speed_mps", -1.0) or -1.0),
                                            immediate_blockage_severe=bool(
                                                b1_downstream_diag.get("immediate_blockage_severe", False)
                                            ),
                                            external_context_used=bool(
                                                b1_downstream_diag.get("external_downstream_context_used", False)
                                            ),
                                            external_context_provider=str(
                                                b1_downstream_diag.get("external_downstream_context_provider", "")
                                            ),
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
                                    

                                    b1_plan_type_for_guard = str(getattr(plan_for_b1_apply, "plan_type", "") or "") if plan_for_b1_apply is not None else ""
                                    b1_aggressive_plan = b1_plan_type_for_guard in ("intrusive", "saturation_reduction")
                                    b1_active_plan = b1_plan_type_for_guard not in ("", "none", "restore")
                                    b1_immediate_blocked = bool(b1_downstream_diag.get("immediate_blockage_severe", False))
                                    b1_blocked_by_lookahead_guard = bool(plan is not None and plan_for_b1_apply is None)
                                    b1_blocked_by_downstream_guard = bool(
                                        bool(getattr(args, "b1_downstream_blockage_guard_enable", True))
                                        and bool(b1_downstream_diag.get("blocked", False))
                                        and (b1_aggressive_plan or (b1_immediate_blocked and b1_active_plan))
                                    )
                                    if b1_blocked_by_lookahead_guard:
                                        _fed_dbg_main(
                                            f"evt=B1_APPLY_SKIP tls={tls_id} ev={ev_id} sim={float(sim_time):.2f} "
                                            f"reason={str(b1_lookahead_diag.get('reason', 'lookahead_guard'))} "
                                            f"plan_type={str(getattr(plan, 'plan_type', '') or '')} "
                                            f"edge={ev_edge} selected_edge={selected_in_edge} d_stop={float(d_stop):.2f} "
                                            f"route_dist={_b1_route_dist_dbg_s} "
                                            f"speed={float(traci.vehicle.getSpeed(ev_id)):.2f}"
                                        )
                                        _fed_evt_main(
                                            "b1.apply_skipped",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            sim_time=float(sim_time),
                                            reason=str(b1_lookahead_diag.get("reason", "lookahead_guard")),
                                            plan_type=str(getattr(plan, "plan_type", "") or ""),
                                            target_phase=str(getattr(plan, "target_phase_idx", "")) if plan is not None else "",
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            distance_to_stopline_m=float(d_stop),
                                            speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                            lookahead_guard_enabled=bool(b1_lookahead_diag.get("guard_enabled", False)),
                                            route_lookahead=bool(b1_lookahead_diag.get("route_lookahead", False)),
                                            lookahead_hops=int(b1_lookahead_diag.get("lookahead_hops", 0) or 0),
                                            route_distance_to_selected_edge_m=b1_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                            on_selected_in_edge=bool(b1_lookahead_diag.get("on_selected_in_edge", False)),
                                            preemption_eligible=bool(b1_lookahead_diag.get("preemption_eligible", True)),
                                            upstream_stopped=bool(b1_lookahead_diag.get("upstream_stopped", False)),
                                            lookahead_action=str(b1_lookahead_diag.get("action", "")),
                                            before_phase=int(tls_phase_before_b1),
                                            before_program=str(tls_prog_before_b1),
                                            before_next_switch=float(tls_next_switch_before_b1),
                                            before_state=str(tls_state_before_b1),
                                        )
                                    if b1_blocked_by_downstream_guard:
                                        b1_downstream_skip_reason = (
                                            "immediate_downstream_blockage_guard"
                                            if b1_immediate_blocked
                                            else "downstream_blockage_guard"
                                        )
                                        _fed_dbg_main(
                                            f"evt=B1_APPLY_SKIP tls={tls_id} ev={ev_id} sim={float(sim_time):.2f} "
                                            f"reason={b1_downstream_skip_reason} plan_type={b1_plan_type_for_guard} "
                                            f"edge={ev_edge} selected_edge={selected_in_edge} d_stop={float(d_stop):.2f} "
                                            f"worst_edge={str(b1_downstream_diag.get('worst_edge', '')) or '-'} "
                                            f"worst_offset={int(b1_downstream_diag.get('worst_edge_offset', -1) or -1)} "
                                            f"block_reason={str(b1_downstream_diag.get('reason', ''))} "
                                            f"max_halt={int(b1_downstream_diag.get('max_halt_n', 0) or 0)} "
                                            f"max_veh={int(b1_downstream_diag.get('max_veh_n', 0) or 0)} "
                                            f"min_speed={float(b1_downstream_diag.get('min_mean_speed_mps', -1.0) or -1.0):.2f} "
                                            f"max_occ={float(b1_downstream_diag.get('max_occupancy_pct', 0.0) or 0.0):.1f}"
                                        )
                                        _fed_evt_main(
                                            "b1.apply_skipped",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            sim_time=float(sim_time),
                                            reason=str(b1_downstream_skip_reason),
                                            plan_type=str(b1_plan_type_for_guard),
                                            target_phase=str(getattr(plan, "target_phase_idx", "")) if plan is not None else "",
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            distance_to_stopline_m=float(d_stop),
                                            speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                            downstream_block_reason=str(b1_downstream_diag.get("reason", "")),
                                            downstream_worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                                            downstream_worst_edge_offset=int(
                                                b1_downstream_diag.get("worst_edge_offset", -1) or -1
                                            ),
                                            downstream_lookahead_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                                            downstream_max_halt_n=int(b1_downstream_diag.get("max_halt_n", 0) or 0),
                                            downstream_max_veh_n=int(b1_downstream_diag.get("max_veh_n", 0) or 0),
                                            downstream_max_occupancy_pct=float(b1_downstream_diag.get("max_occupancy_pct", 0.0) or 0.0),
                                            downstream_min_mean_speed_mps=float(
                                                b1_downstream_diag.get("min_mean_speed_mps", -1.0) or -1.0
                                            ),
                                            downstream_immediate_blockage_severe=bool(b1_immediate_blocked),
                                            before_phase=int(tls_phase_before_b1),
                                            before_program=str(tls_prog_before_b1),
                                            before_next_switch=float(tls_next_switch_before_b1),
                                            before_state=str(tls_state_before_b1),
                                        )

                                    if plan_for_b1_apply and not STATIC_PROGRAM and not b1_blocked_by_lookahead_guard and not b1_blocked_by_downstream_guard:
                                        print(f"**** Selected plan: {plan_for_b1_apply} ****")
                                        ag.apply_plan_to_tls(sim_time, plan_for_b1_apply)
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
                                            f"plan_type={str(getattr(plan_for_b1_apply, 'plan_type', ''))} "
                                            f"original_plan_type={str(getattr(plan, 'plan_type', '') if plan is not None else '')} "
                                            f"target={str(getattr(plan_for_b1_apply, 'target_phase_idx', ''))} "
                                            f"edge={ev_edge} selected_edge={selected_in_edge} d_stop={float(d_stop):.2f} "
                                            f"speed={float(traci.vehicle.getSpeed(ev_id)):.2f} "
                                            f"lookahead_action={str(b1_lookahead_diag.get('action', ''))} "
                                            f"lookahead_reason={str(b1_lookahead_diag.get('reason', ''))} "
                                            f"before_phase={tls_phase_before_b1} after_phase={tls_phase_after_b1} "
                                            f"before_prog={tls_prog_before_b1} after_prog={tls_prog_after_b1} "
                                            f"before_next_switch={tls_next_switch_before_b1:.2f} after_next_switch={tls_next_switch_after_b1:.2f} "
                                            f"before_state={tls_state_before_b1} after_state={tls_state_after_b1}"
                                        )
                                        _fed_evt_main(
                                            "b1.apply",
                                            role="intersection",
                                            ev_id=str(ev_id),
                                            tls_id=str(tls_id),
                                            sim_time=float(sim_time),
                                            plan_type=str(getattr(plan_for_b1_apply, "plan_type", "")),
                                            target_phase=str(getattr(plan_for_b1_apply, "target_phase_idx", "")),
                                            original_plan_type=str(getattr(plan, "plan_type", "")) if plan is not None else "",
                                            before_phase=int(tls_phase_before_b1),
                                            after_phase=int(tls_phase_after_b1),
                                            before_program=str(tls_prog_before_b1),
                                            after_program=str(tls_prog_after_b1),
                                            before_next_switch=float(tls_next_switch_before_b1),
                                            after_next_switch=float(tls_next_switch_after_b1),
                                            before_next_switch_rem_s=float(tls_next_switch_before_b1 - float(sim_time)) if float(tls_next_switch_before_b1) >= 0.0 else -1.0,
                                            after_next_switch_rem_s=float(tls_next_switch_after_b1 - float(sim_time)) if float(tls_next_switch_after_b1) >= 0.0 else -1.0,
                                            before_state=str(tls_state_before_b1),
                                            after_state=str(tls_state_after_b1),
                                            ev_edge=str(ev_edge),
                                            selected_in_edge=str(selected_in_edge),
                                            speed_mps=float(traci.vehicle.getSpeed(ev_id)),
                                            distance_to_stopline_m=float(d_stop),
                                            downstream_blocked=bool(b1_downstream_diag.get("blocked", False)),
                                            downstream_block_reason=str(b1_downstream_diag.get("reason", "")),
                                            downstream_worst_edge=str(b1_downstream_diag.get("worst_edge", "")),
                                            downstream_lookahead_edges=list(b1_downstream_diag.get("lookahead_edges", []) or []),
                                            downstream_max_halt_n=int(b1_downstream_diag.get("max_halt_n", 0) or 0),
                                            downstream_max_veh_n=int(b1_downstream_diag.get("max_veh_n", 0) or 0),
                                            downstream_max_occupancy_pct=float(b1_downstream_diag.get("max_occupancy_pct", 0.0) or 0.0),
                                            downstream_min_mean_speed_mps=float(b1_downstream_diag.get("min_mean_speed_mps", -1.0) or -1.0),
                                            lookahead_guard_enabled=bool(b1_lookahead_diag.get("guard_enabled", False)),
                                            route_lookahead=bool(b1_lookahead_diag.get("route_lookahead", False)),
                                            lookahead_hops=int(b1_lookahead_diag.get("lookahead_hops", 0) or 0),
                                            route_distance_to_selected_edge_m=b1_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                            on_selected_in_edge=bool(b1_lookahead_diag.get("on_selected_in_edge", False)),
                                            preemption_eligible=bool(b1_lookahead_diag.get("preemption_eligible", True)),
                                            upstream_stopped=bool(b1_lookahead_diag.get("upstream_stopped", False)),
                                            lookahead_action=str(b1_lookahead_diag.get("action", "")),
                                            lookahead_reason=str(b1_lookahead_diag.get("reason", "")),
                                            lookahead_original_extend_green_sec=float(b1_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                            lookahead_applied_extend_green_sec=float(b1_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                        )
                                        try:
                                            _b1_plan_type = str(getattr(plan_for_b1_apply, "plan_type", "") or "")
                                            _b1_effective = bool(
                                                _b1_plan_type not in ("", "none", "restore")
                                                or int(tls_phase_before_b1) != int(tls_phase_after_b1)
                                                or abs(float(tls_next_switch_before_b1) - float(tls_next_switch_after_b1)) > 0.01
                                            )
                                            _apply_diag_state[(str(ev_id), str(tls_id))] = {
                                                "last_apply_time": float(sim_time),
                                                "last_apply_mode": "B1",
                                                "last_apply_plan_type": str(_b1_plan_type),
                                                "last_apply_decision_source": "local",
                                                "last_apply_effective": bool(_b1_effective),
                                                "last_apply_phase_before": int(tls_phase_before_b1),
                                                "last_apply_phase_after": int(tls_phase_after_b1),
                                                "last_apply_target_phase": str(getattr(plan_for_b1_apply, "target_phase_idx", "") or ""),
                                            }
                                        except Exception:
                                            pass
                                        payload = json.dumps({
                                            "step": step,
                                            "simTime": sim_time,
                                            "tlsId": tls_id,
                                            "approachNode": approach_node,
                                            "evEdge": ev_edge,
                                            "distToStoplineM": d_stop,
                                            "plan": plan_for_b1_apply.__dict__,
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

                                if _is_f2_family(CURRENT_EVALUATION):
                                    print("Currently on federation approach 2 (what-if)")
                                    next_decision_due = float(getattr(ag, "_next_decision_time", 0.0))
                                    next_refine_due = float(getattr(ag, "_next_refine_time", 0.0))
                                    f2_eval_due = (
                                        float(sim_time) + 1e-6 >= next_decision_due
                                        or float(sim_time) + 1e-6 >= next_refine_due
                                    )
                                    offers = []
                                    f2_meta = {}
                                    chosen = None
                                    if not f2_eval_due:
                                        _fed_dbg_main(
                                            f"evt=F2_EVAL_SKIP tls={tls_id} ev={ev_id} sim={float(sim_time):.2f} "
                                            f"next_decision={float(next_decision_due):.2f} "
                                            f"next_refine={float(next_refine_due):.2f}"
                                        )
                                    else:
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

                                        strict_b1_floor = bool(getattr(args, "f2_strict_b1_floor_enable", False))
                                        if strict_b1_floor and not STATIC_PROGRAM:
                                            allow_f2_selected, strict_offer_diag = _f2_strict_selected_offer_allowed(
                                                ag=ag,
                                                chosen=chosen,
                                                f2_meta=f2_meta,
                                            )
                                            if not allow_f2_selected:
                                                _fed_evt_main(
                                                    "f2.strict_b1_floor.selected_offer_deferred",
                                                    role="intersection",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    reason=str(strict_offer_diag.get("reason", "strict_floor")),
                                                    final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                    refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                    selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                    selected_offer_diag=dict(strict_offer_diag),
                                                )
                                                try:
                                                    _try_f2_strict_b1_floor_apply(
                                                        ag=ag,
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        lookahead_hops=int(lookahead_hops),
                                                        d_stop=float(d_stop),
                                                        trigger_reason=(
                                                            "strict_selected_deferred"
                                                            if chosen is not None
                                                            else ("strict_eval_due_no_offers" if f2_eval_due else "strict_eval_not_due")
                                                        ),
                                                        f2_meta=f2_meta,
                                                        precomputed_plan=getattr(ag, "_last_f2_primary_plan", None) if f2_eval_due else None,
                                                    )
                                                except Exception as e:
                                                    _fed_dbg_main(
                                                        f"evt=F2_STRICT_B1_FLOOR_WARN tls={tls_id} ev={ev_id} "
                                                        f"err={type(e).__name__}:{e}"
                                                    )
                                                continue

                                        if chosen is not None and not STATIC_PROGRAM:
                                            # Even through some possibilities exists, not any action is applied.

                                            selected_offer_plan_type = ""
                                            try:
                                                selected_offer_plan_type = str(ag._plan_type_from_offer(chosen) or "")
                                            except Exception:
                                                selected_offer_plan_type = str(getattr(chosen, "action", "") or "")
                                            selected_offer_action = str(getattr(chosen, "action", "") or "")
                                            local_current_plan, local_current_plan_type = _f2_local_current_plan_for_actuation(ag)
                                            if local_current_plan is not None and not strict_b1_floor:
                                                local_current_plan_for_apply, f2_local_lookahead_diag = _route_lookahead_actuation_filter(
                                                    mode="F2",
                                                    stage="local_anchor_current_tls",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    ev_edge=str(ev_edge),
                                                    selected_in_edge=str(selected_in_edge),
                                                    lookahead_hops=int(lookahead_hops),
                                                    d_stop=float(d_stop),
                                                    plan=local_current_plan,
                                                )
                                                # Preserve the B1-equivalent current-TLS actuation. F2 still
                                                # publishes/drains peer coordination, but the selected peer offer
                                                # cannot weaken the local EV priority plan at the intersection
                                                # currently serving the EV.
                                                if local_current_plan_for_apply is None:
                                                    f2_tls_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                    local_current_plan_for_apply, stopped_rescue_diag = (
                                                        _maybe_f2_current_tls_stopped_rescue_plan(
                                                            stage="local_anchor_current_tls",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            ag=ag,
                                                            ev_edge=str(ev_edge),
                                                            selected_in_edge=str(selected_in_edge),
                                                            d_stop=float(d_stop),
                                                            plan=local_current_plan,
                                                            tls_before=f2_tls_before,
                                                            lookahead_diag=f2_local_lookahead_diag,
                                                            f2_meta=f2_meta,
                                                        )
                                                    )
                                                    if (
                                                        local_current_plan_for_apply is None
                                                        or not bool(stopped_rescue_diag.get("applied", False))
                                                    ):
                                                        _fed_dbg_main(
                                                            f"evt=F2_LOCAL_ANCHOR_CURRENT_TLS_SKIP tls={tls_id} ev={ev_id} "
                                                            f"reason={str(f2_local_lookahead_diag.get('reason', 'lookahead_guard'))} "
                                                            f"stopped_rescue_reason={str(stopped_rescue_diag.get('reason', ''))} "
                                                            f"local_plan_type={local_current_plan_type} "
                                                            f"selected_plan_type={selected_offer_plan_type} action={selected_offer_action} "
                                                            f"sim={float(sim_time):.2f}"
                                                        )
                                                        _fed_evt_main(
                                                            "f2.local_anchor.current_tls.skip",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(f2_local_lookahead_diag.get("reason", "lookahead_guard")),
                                                            stopped_rescue_reason=str(stopped_rescue_diag.get("reason", "")),
                                                            local_plan_type=str(local_current_plan_type),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                            final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                            refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                            route_lookahead=bool(f2_local_lookahead_diag.get("route_lookahead", False)),
                                                            lookahead_hops=int(f2_local_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                            route_distance_to_selected_edge_m=f2_local_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            on_selected_in_edge=bool(f2_local_lookahead_diag.get("on_selected_in_edge", False)),
                                                            preemption_eligible=bool(f2_local_lookahead_diag.get("preemption_eligible", True)),
                                                            upstream_stopped=bool(f2_local_lookahead_diag.get("upstream_stopped", False)),
                                                            lookahead_action=str(f2_local_lookahead_diag.get("action", "")),
                                                            lookahead_reason=str(f2_local_lookahead_diag.get("reason", "")),
                                                            lookahead_original_extend_green_sec=float(f2_local_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                                            lookahead_applied_extend_green_sec=float(f2_local_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                                        )
                                                        continue
                                                    f2_local_lookahead_diag = dict(f2_local_lookahead_diag or {})
                                                    f2_local_lookahead_diag["action"] = "rescue"
                                                    f2_local_lookahead_diag["reason"] = str(
                                                        stopped_rescue_diag.get("reason", "current_tls_stopped_rescue")
                                                    )
                                                    f2_local_lookahead_diag["current_tls_stopped_rescue_applied"] = True
                                                    ag.apply_plan_to_tls(
                                                        sim_time,
                                                        local_current_plan_for_apply,
                                                        decision_source="f2_current_tls_stopped_rescue",
                                                    )
                                                    f2_tls_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                    _remember_f2_last_local_anchor_plan(
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        plan=local_current_plan_for_apply,
                                                        plan_type=str(getattr(local_current_plan_for_apply, "plan_type", "") or local_current_plan_type),
                                                        selected_in_edge=str(selected_in_edge),
                                                        lookahead_diag=f2_local_lookahead_diag,
                                                        source="current_tls_stopped_rescue",
                                                    )
                                                    _fed_evt_main(
                                                        "f2.local_anchor.current_tls.apply",
                                                        role="intersection",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        reason="current_tls_stopped_rescue",
                                                        local_plan_type=str(local_current_plan_type),
                                                        applied_local_plan_type=str(getattr(local_current_plan_for_apply, "plan_type", "") or ""),
                                                        selected_offer_plan_type=str(selected_offer_plan_type),
                                                        selected_offer_action=str(selected_offer_action),
                                                        selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                        final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                        refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                        route_lookahead=bool(f2_local_lookahead_diag.get("route_lookahead", False)),
                                                        lookahead_hops=int(f2_local_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                        route_distance_to_selected_edge_m=f2_local_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                        on_selected_in_edge=bool(f2_local_lookahead_diag.get("on_selected_in_edge", False)),
                                                        preemption_eligible=bool(f2_local_lookahead_diag.get("preemption_eligible", True)),
                                                        upstream_stopped=bool(f2_local_lookahead_diag.get("upstream_stopped", False)),
                                                        lookahead_action=str(f2_local_lookahead_diag.get("action", "")),
                                                        lookahead_reason=str(f2_local_lookahead_diag.get("reason", "")),
                                                        lookahead_original_extend_green_sec=float(f2_local_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                                        lookahead_applied_extend_green_sec=float(f2_local_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                                        current_tls_stopped_rescue_applied=True,
                                                        current_tls_stopped_rescue_reason=str(stopped_rescue_diag.get("reason", "")),
                                                        before_phase=int(f2_tls_before.get("phase", -1)),
                                                        after_phase=int(f2_tls_after.get("phase", -1)),
                                                        before_state=str(f2_tls_before.get("state", "")),
                                                        after_state=str(f2_tls_after.get("state", "")),
                                                        before_next_switch=float(f2_tls_before.get("next_switch", -1.0)),
                                                        after_next_switch=float(f2_tls_after.get("next_switch", -1.0)),
                                                        before_next_switch_rem_s=float(f2_tls_before.get("next_switch_rem_s", -1.0)),
                                                        after_next_switch_rem_s=float(f2_tls_after.get("next_switch_rem_s", -1.0)),
                                                    )
                                                    continue
                                                else:
                                                    f2_tls_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                    local_current_plan_for_apply, approach_rescue_diag = _maybe_f2_approach_phase_rescue_plan(
                                                        stage="local_anchor_current_tls",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        ag=ag,
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        d_stop=float(d_stop),
                                                        plan=local_current_plan_for_apply,
                                                        tls_before=f2_tls_before,
                                                        lookahead_diag=f2_local_lookahead_diag,
                                                        f2_meta=f2_meta,
                                                    )
                                                    local_replay_guard_diag = _f2_downstream_replay_guard_diag(
                                                        stage="local_anchor_current_tls",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        plan=local_current_plan_for_apply,
                                                        lookahead_diag=f2_local_lookahead_diag,
                                                        source="current_local_plan",
                                                    )
                                                    if (
                                                        bool(local_replay_guard_diag.get("skip_replay", False))
                                                        and not bool(approach_rescue_diag.get("applied", False))
                                                    ):
                                                        skip_local_anchor_apply = True
                                                        skip_diag = {
                                                            "reason": "downstream_replay_guard",
                                                            "downstream_replay_guard": dict(local_replay_guard_diag),
                                                        }
                                                    else:
                                                        skip_local_anchor_apply, skip_diag = _should_skip_f2_local_anchor_preapply(
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            plan=local_current_plan_for_apply,
                                                            tls_before=f2_tls_before,
                                                            lookahead_diag=f2_local_lookahead_diag,
                                                        )
                                                        if (
                                                            not skip_local_anchor_apply
                                                            and not bool(approach_rescue_diag.get("applied", False))
                                                        ):
                                                            skip_local_anchor_apply, skip_diag = _should_skip_f2_fallback_slow_green(
                                                                stage="local_anchor_current_tls",
                                                                event_type="f2.local_anchor.current_tls.skip",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ag=ag,
                                                                plan=local_current_plan_for_apply,
                                                                tls_before=f2_tls_before,
                                                                selected_in_edge=str(selected_in_edge),
                                                                d_stop=float(d_stop),
                                                                f2_meta=f2_meta,
                                                            )
                                                        if (
                                                            not skip_local_anchor_apply
                                                            and not bool(approach_rescue_diag.get("applied", False))
                                                        ):
                                                            skip_local_anchor_apply, skip_diag = _should_skip_f2_fallback_cadence(
                                                                stage="local_anchor_current_tls",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                plan=local_current_plan_for_apply,
                                                                d_stop=float(d_stop),
                                                            )
                                                    if skip_local_anchor_apply:
                                                        local_anchor_skip_reason = str(
                                                            skip_diag.get("reason", "redundant_local_anchor_preapply")
                                                        )
                                                        _fed_dbg_main(
                                                            f"evt=F2_LOCAL_ANCHOR_CURRENT_TLS_SKIP tls={tls_id} ev={ev_id} "
                                                            f"reason={local_anchor_skip_reason} local_plan_type={local_current_plan_type} "
                                                            f"selected_plan_type={selected_offer_plan_type} action={selected_offer_action} "
                                                            f"dt_since_last={skip_diag.get('dt_since_last_s')} "
                                                            f"min_interval={float(skip_diag.get('min_interval_s', 0.0) or 0.0):.2f} "
                                                            f"sim={float(sim_time):.2f}"
                                                        )
                                                        _fed_evt_main(
                                                            "f2.local_anchor.current_tls.skip",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(local_anchor_skip_reason),
                                                            local_plan_type=str(local_current_plan_type),
                                                            applied_local_plan_type=str(getattr(local_current_plan_for_apply, "plan_type", "") or ""),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                            final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                            refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                            route_lookahead=bool(f2_local_lookahead_diag.get("route_lookahead", False)),
                                                            lookahead_hops=int(f2_local_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                            route_distance_to_selected_edge_m=f2_local_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            on_selected_in_edge=bool(f2_local_lookahead_diag.get("on_selected_in_edge", False)),
                                                            preemption_eligible=bool(f2_local_lookahead_diag.get("preemption_eligible", True)),
                                                            upstream_stopped=bool(f2_local_lookahead_diag.get("upstream_stopped", False)),
                                                            lookahead_action=str(f2_local_lookahead_diag.get("action", "")),
                                                            lookahead_reason=str(f2_local_lookahead_diag.get("reason", "")),
                                                            lookahead_original_extend_green_sec=float(f2_local_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                                            lookahead_applied_extend_green_sec=float(f2_local_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                                            before_phase=int(f2_tls_before.get("phase", -1)),
                                                            before_state=str(f2_tls_before.get("state", "")),
                                                            before_next_switch=float(f2_tls_before.get("next_switch", -1.0)),
                                                            before_next_switch_rem_s=float(f2_tls_before.get("next_switch_rem_s", -1.0)),
                                                            redundant_apply_min_interval_s=skip_diag.get("min_interval_s"),
                                                            redundant_apply_dt_since_last_s=skip_diag.get("dt_since_last_s"),
                                                            redundant_apply_distance_m=skip_diag.get("distance_for_interval_m"),
                                                            redundant_apply_suppressed_count=skip_diag.get("suppressed_count"),
                                                            redundant_apply_signature=skip_diag.get("signature"),
                                                            downstream_replay_guard=skip_diag.get("downstream_replay_guard"),
                                                        )
                                                    else:
                                                        ag.apply_plan_to_tls(
                                                            sim_time,
                                                            local_current_plan_for_apply,
                                                            decision_source="f2_local_anchor_current_tls",
                                                        )
                                                        f2_tls_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                        _remember_f2_last_local_anchor_plan(
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            plan=local_current_plan_for_apply,
                                                            plan_type=str(local_current_plan_type),
                                                            selected_in_edge=str(selected_in_edge),
                                                            lookahead_diag=f2_local_lookahead_diag,
                                                            source="local_anchor_current_tls",
                                                        )
                                                        _fed_evt_main(
                                                            "f2.local_anchor.current_tls.apply",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason="preserve_b1_local_priority",
                                                            local_plan_type=str(local_current_plan_type),
                                                            applied_local_plan_type=str(getattr(local_current_plan_for_apply, "plan_type", "") or ""),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                            final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                            refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                            route_lookahead=bool(f2_local_lookahead_diag.get("route_lookahead", False)),
                                                            lookahead_hops=int(f2_local_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                            route_distance_to_selected_edge_m=f2_local_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            on_selected_in_edge=bool(f2_local_lookahead_diag.get("on_selected_in_edge", False)),
                                                            preemption_eligible=bool(f2_local_lookahead_diag.get("preemption_eligible", True)),
                                                            upstream_stopped=bool(f2_local_lookahead_diag.get("upstream_stopped", False)),
                                                            lookahead_action=str(f2_local_lookahead_diag.get("action", "")),
                                                            lookahead_reason=str(f2_local_lookahead_diag.get("reason", "")),
                                                            lookahead_original_extend_green_sec=float(f2_local_lookahead_diag.get("original_extend_green_sec", 0.0) or 0.0),
                                                            lookahead_applied_extend_green_sec=float(f2_local_lookahead_diag.get("applied_extend_green_sec", 0.0) or 0.0),
                                                            approach_phase_rescue_applied=bool(approach_rescue_diag.get("applied", False)),
                                                            approach_phase_rescue_reason=str(approach_rescue_diag.get("reason", "")),
                                                            downstream_replay_guard=dict(local_replay_guard_diag),
                                                            redundant_apply_min_interval_s=skip_diag.get("min_interval_s"),
                                                            redundant_apply_dt_since_last_s=skip_diag.get("dt_since_last_s"),
                                                            redundant_apply_distance_m=skip_diag.get("distance_for_interval_m"),
                                                            redundant_apply_signature=skip_diag.get("signature"),
                                                        )
                                                        _fed_dbg_main(
                                                            f"evt=F2_LOCAL_ANCHOR_CURRENT_TLS_APPLY tls={tls_id} ev={ev_id} "
                                                            f"reason=preserve_b1_local_priority local_plan_type={local_current_plan_type} "
                                                            f"applied_local_plan_type={str(getattr(local_current_plan_for_apply, 'plan_type', '') or '')} "
                                                            f"lookahead_action={str(f2_local_lookahead_diag.get('action', ''))} "
                                                            f"selected_plan_type={selected_offer_plan_type} action={selected_offer_action} "
                                                            f"final_reason={str((f2_meta or {}).get('final_reason', ''))} "
                                                            f"sim={float(sim_time):.2f}"
                                                        )
                                                        _emit_f2_apply_effect(
                                                            stage="local_anchor_current_tls",
                                                            decision_source="f2_local_anchor_current_tls",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            ag=ag,
                                                            selected_in_edge=str(selected_in_edge),
                                                            ev_edge=str(ev_edge),
                                                            d_stop=float(d_stop),
                                                            before=f2_tls_before,
                                                            after=f2_tls_after,
                                                            offer=chosen,
                                                            plan=local_current_plan_for_apply,
                                                            f2_meta=f2_meta,
                                                        )
                                            elif selected_offer_plan_type in ("none", "restore") or selected_offer_action == "none":
                                                keepalive_applied = False
                                                keepalive_effective_target_green = False
                                                keepalive_reason = "weak_offer_idempotent"
                                                keepalive_source = "none"
                                                keepalive_plan_type = ""
                                                cached_plan, cached_plan_type, cached_diag = _recent_f2_last_local_anchor_plan(
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    selected_in_edge=str(selected_in_edge),
                                                )
                                                if cached_plan is not None:
                                                    keepalive_source = "recent_local_anchor_plan"
                                                    keepalive_plan_type = str(cached_plan_type)
                                                    keepalive_plan_for_apply, keepalive_lookahead_diag = _route_lookahead_actuation_filter(
                                                        mode="F2",
                                                        stage="local_anchor_keepalive",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        lookahead_hops=int(lookahead_hops),
                                                        d_stop=float(d_stop),
                                                        plan=cached_plan,
                                                    )
                                                    if keepalive_plan_for_apply is None:
                                                        keepalive_reason = str(
                                                            keepalive_lookahead_diag.get("reason", "lookahead_guard")
                                                        )
                                                        _fed_evt_main(
                                                            "f2.local_anchor.keepalive.skip",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(keepalive_reason),
                                                            source=str(keepalive_source),
                                                            cached_reason=str(cached_diag.get("reason", "")),
                                                            cached_age_sec=cached_diag.get("age_sec"),
                                                            cached_plan_type=str(cached_plan_type),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            route_lookahead=bool(keepalive_lookahead_diag.get("route_lookahead", False)),
                                                            lookahead_hops=int(keepalive_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                            route_distance_to_selected_edge_m=keepalive_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                        )
                                                    else:
                                                        keepalive_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                        keepalive_rescue_meta = dict(f2_meta or {})
                                                        keepalive_original_reason = str(
                                                            keepalive_rescue_meta.get("final_reason", "") or ""
                                                        )
                                                        if not _f2_blocked_local_reason(
                                                            keepalive_original_reason,
                                                            "local_anchor_keepalive",
                                                        ):
                                                            keepalive_rescue_meta["final_reason"] = (
                                                                "local_anchor_keepalive_target_pending"
                                                                + (
                                                                    f":{keepalive_original_reason}"
                                                                    if keepalive_original_reason
                                                                    else ""
                                                                )
                                                            )
                                                        keepalive_plan_for_apply, keepalive_approach_rescue_diag = (
                                                            _maybe_f2_approach_phase_rescue_plan(
                                                                stage="local_anchor_keepalive",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ag=ag,
                                                                ev_edge=str(ev_edge),
                                                                selected_in_edge=str(selected_in_edge),
                                                                d_stop=float(d_stop),
                                                                plan=keepalive_plan_for_apply,
                                                                tls_before=keepalive_before,
                                                                lookahead_diag=keepalive_lookahead_diag,
                                                                f2_meta=keepalive_rescue_meta,
                                                            )
                                                        )
                                                        if bool(keepalive_approach_rescue_diag.get("applied", False)):
                                                            keepalive_skip_apply = False
                                                            keepalive_skip_diag = {
                                                                "reason": "approach_phase_rescue",
                                                                "approach_phase_rescue_applied": True,
                                                            }
                                                            _fed_evt_main(
                                                                "f2.local_anchor.keepalive.approach_phase_rescue.apply",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(keepalive_approach_rescue_diag.get("reason", "")),
                                                                source=str(keepalive_source),
                                                                cached_reason=str(cached_diag.get("reason", "")),
                                                                cached_age_sec=cached_diag.get("age_sec"),
                                                                cached_plan_type=str(cached_plan_type),
                                                                original_final_reason=str(keepalive_original_reason),
                                                                rescue_final_reason=str(keepalive_rescue_meta.get("final_reason", "")),
                                                                target_phase=keepalive_approach_rescue_diag.get("target_phase"),
                                                                before_phase=keepalive_approach_rescue_diag.get("before_phase"),
                                                                route_distance_to_selected_edge_m=keepalive_approach_rescue_diag.get(
                                                                    "route_distance_to_selected_edge_m"
                                                                ),
                                                                speed_mps=keepalive_approach_rescue_diag.get("speed_mps"),
                                                                max_distance_m=keepalive_approach_rescue_diag.get("max_distance_m"),
                                                            )
                                                        else:
                                                            keepalive_replay_guard_diag = _f2_downstream_replay_guard_diag(
                                                                stage="local_anchor_keepalive",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ev_edge=str(ev_edge),
                                                                selected_in_edge=str(selected_in_edge),
                                                                plan=keepalive_plan_for_apply,
                                                                lookahead_diag=keepalive_lookahead_diag,
                                                                source=str(keepalive_source),
                                                            )
                                                            if bool(keepalive_replay_guard_diag.get("skip_replay", False)):
                                                                keepalive_skip_apply = True
                                                                keepalive_skip_diag = {
                                                                    "reason": "downstream_replay_guard",
                                                                    "downstream_replay_guard": dict(keepalive_replay_guard_diag),
                                                                }
                                                            else:
                                                                keepalive_skip_apply, keepalive_skip_diag = _should_skip_f2_local_anchor_preapply(
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    plan=keepalive_plan_for_apply,
                                                                    tls_before=keepalive_before,
                                                                    lookahead_diag=keepalive_lookahead_diag,
                                                                )
                                                                if not keepalive_skip_apply:
                                                                    keepalive_skip_apply, keepalive_skip_diag = _should_skip_f2_target_pending_bridge(
                                                                        stage="local_anchor_keepalive",
                                                                        ev_id=str(ev_id),
                                                                        tls_id=str(tls_id),
                                                                        sim_time=float(sim_time),
                                                                        ag=ag,
                                                                        selected_in_edge=str(selected_in_edge),
                                                                        d_stop=float(d_stop),
                                                                        plan=keepalive_plan_for_apply,
                                                                        tls_before=keepalive_before,
                                                                        lookahead_diag=keepalive_lookahead_diag,
                                                                    )
                                                                if not keepalive_skip_apply:
                                                                    keepalive_skip_apply, keepalive_skip_diag = _should_skip_f2_fallback_slow_green(
                                                                        stage="local_anchor_keepalive",
                                                                        event_type="f2.local_anchor.keepalive.skip",
                                                                        ev_id=str(ev_id),
                                                                        tls_id=str(tls_id),
                                                                        sim_time=float(sim_time),
                                                                        ag=ag,
                                                                        plan=keepalive_plan_for_apply,
                                                                        tls_before=keepalive_before,
                                                                        selected_in_edge=str(selected_in_edge),
                                                                        d_stop=float(d_stop),
                                                                        f2_meta=f2_meta,
                                                                    )
                                                                if not keepalive_skip_apply:
                                                                    keepalive_skip_apply, keepalive_skip_diag = _should_skip_f2_fallback_cadence(
                                                                        stage="local_anchor_keepalive",
                                                                        ev_id=str(ev_id),
                                                                        tls_id=str(tls_id),
                                                                        sim_time=float(sim_time),
                                                                        plan=keepalive_plan_for_apply,
                                                                        d_stop=float(d_stop),
                                                                    )
                                                        if keepalive_skip_apply:
                                                            keepalive_reason = str(
                                                                keepalive_skip_diag.get("reason", "redundant_local_anchor_preapply")
                                                            )
                                                            _fed_evt_main(
                                                                "f2.local_anchor.keepalive.skip",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(keepalive_reason),
                                                                source=str(keepalive_source),
                                                                cached_reason=str(cached_diag.get("reason", "")),
                                                                cached_age_sec=cached_diag.get("age_sec"),
                                                                cached_plan_type=str(cached_plan_type),
                                                                selected_offer_plan_type=str(selected_offer_plan_type),
                                                                selected_offer_action=str(selected_offer_action),
                                                                redundant_apply_min_interval_s=keepalive_skip_diag.get("min_interval_s"),
                                                                redundant_apply_dt_since_last_s=keepalive_skip_diag.get("dt_since_last_s"),
                                                                redundant_apply_distance_m=keepalive_skip_diag.get("distance_for_interval_m"),
                                                                redundant_apply_suppressed_count=keepalive_skip_diag.get("suppressed_count"),
                                                                target_phase=keepalive_skip_diag.get("target_phase"),
                                                                target_pending_count=keepalive_skip_diag.get("count"),
                                                                target_pending_suppress_after_n=keepalive_skip_diag.get("suppress_after_n"),
                                                                target_pending_late_rescue_active=keepalive_skip_diag.get("late_rescue_active"),
                                                                downstream_replay_guard=keepalive_skip_diag.get("downstream_replay_guard"),
                                                            )
                                                            if keepalive_reason == "target_pending_bridge_suppressed":
                                                                _fed_evt_main(
                                                                    "f2.b1_bridge.skip",
                                                                    role="intersection",
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    reason=str(keepalive_reason),
                                                                    source=str(keepalive_source),
                                                                    stage="local_anchor_keepalive",
                                                                    bridge_plan_type=str(
                                                                        getattr(keepalive_plan_for_apply, "plan_type", "") or cached_plan_type
                                                                    ),
                                                                    target_phase=keepalive_skip_diag.get("target_phase"),
                                                                    target_pending_count=keepalive_skip_diag.get("count"),
                                                                    target_pending_suppress_after_n=keepalive_skip_diag.get("suppress_after_n"),
                                                                    route_distance_to_selected_edge_m=keepalive_skip_diag.get(
                                                                        "route_distance_to_selected_edge_m"
                                                                    ),
                                                                )
                                                        else:
                                                            ag.apply_plan_to_tls(
                                                                sim_time,
                                                                keepalive_plan_for_apply,
                                                                decision_source="f2_local_anchor_keepalive",
                                                            )
                                                            keepalive_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                            keepalive_target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
                                                            keepalive_effective_target_green = _target_is_green_for_diag(
                                                                keepalive_after,
                                                                int(keepalive_target_phase),
                                                            )
                                                            keepalive_applied = True
                                                            keepalive_reason = (
                                                                "applied_local_keepalive"
                                                                if keepalive_effective_target_green
                                                                else "applied_but_target_phase_not_green"
                                                            )
                                                            _record_f2_target_pending_bridge(
                                                                stage="local_anchor_keepalive",
                                                                decision_source="f2_local_anchor_keepalive",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ag=ag,
                                                                selected_in_edge=str(selected_in_edge),
                                                                ev_edge=str(ev_edge),
                                                                d_stop=float(d_stop),
                                                                plan=keepalive_plan_for_apply,
                                                                before=keepalive_before,
                                                                after=keepalive_after,
                                                                lookahead_diag=keepalive_lookahead_diag,
                                                                reason=str(keepalive_reason),
                                                                source=str(keepalive_source),
                                                            )
                                                            # A B1-equivalent local plan can be useful even when the
                                                            # target phase is not green immediately after apply: it may
                                                            # be hurrying the current phase or preserving a transition.
                                                            # Do not erase the only local continuity memory just because
                                                            # the service window is still pending.
                                                            _remember_f2_last_local_anchor_plan(
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                plan=keepalive_plan_for_apply,
                                                                plan_type=str(getattr(keepalive_plan_for_apply, "plan_type", "") or cached_plan_type),
                                                                selected_in_edge=str(selected_in_edge),
                                                                lookahead_diag=keepalive_lookahead_diag,
                                                                source=(
                                                                    "local_anchor_keepalive"
                                                                    if keepalive_effective_target_green
                                                                    else "local_anchor_keepalive_target_pending"
                                                                ),
                                                            )
                                                            _fed_evt_main(
                                                                "f2.b1_bridge.apply",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(keepalive_reason),
                                                                source=str(keepalive_source),
                                                                stage="local_anchor_keepalive",
                                                                bridge_plan_type=str(getattr(keepalive_plan_for_apply, "plan_type", "") or cached_plan_type),
                                                                target_phase=int(keepalive_target_phase),
                                                                target_green_after=bool(keepalive_effective_target_green),
                                                                before_phase=int(keepalive_before.get("phase", -1)),
                                                                after_phase=int(keepalive_after.get("phase", -1)),
                                                                before_state=str(keepalive_before.get("state", "")),
                                                                after_state=str(keepalive_after.get("state", "")),
                                                                before_next_switch=float(keepalive_before.get("next_switch", -1.0)),
                                                                after_next_switch=float(keepalive_after.get("next_switch", -1.0)),
                                                                route_lookahead=bool(keepalive_lookahead_diag.get("route_lookahead", False)),
                                                                lookahead_hops=int(keepalive_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                                route_distance_to_selected_edge_m=keepalive_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            )
                                                            _fed_evt_main(
                                                                "f2.local_anchor.keepalive.apply",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(keepalive_reason),
                                                                source=str(keepalive_source),
                                                                cached_reason=str(cached_diag.get("reason", "")),
                                                                cached_age_sec=cached_diag.get("age_sec"),
                                                                cached_plan_type=str(cached_plan_type),
                                                                applied_local_plan_type=str(getattr(keepalive_plan_for_apply, "plan_type", "") or ""),
                                                                selected_offer_plan_type=str(selected_offer_plan_type),
                                                                selected_offer_action=str(selected_offer_action),
                                                                target_phase=int(keepalive_target_phase),
                                                                target_green_after=bool(keepalive_effective_target_green),
                                                                before_phase=int(keepalive_before.get("phase", -1)),
                                                                after_phase=int(keepalive_after.get("phase", -1)),
                                                                before_state=str(keepalive_before.get("state", "")),
                                                                after_state=str(keepalive_after.get("state", "")),
                                                                before_next_switch=float(keepalive_before.get("next_switch", -1.0)),
                                                                after_next_switch=float(keepalive_after.get("next_switch", -1.0)),
                                                                route_lookahead=bool(keepalive_lookahead_diag.get("route_lookahead", False)),
                                                                lookahead_hops=int(keepalive_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                                route_distance_to_selected_edge_m=keepalive_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            )
                                                            _emit_f2_apply_effect(
                                                                stage="local_anchor_keepalive",
                                                                decision_source="f2_local_anchor_keepalive",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ag=ag,
                                                                selected_in_edge=str(selected_in_edge),
                                                                ev_edge=str(ev_edge),
                                                                d_stop=float(d_stop),
                                                                before=keepalive_before,
                                                                after=keepalive_after,
                                                                offer=chosen,
                                                                plan=keepalive_plan_for_apply,
                                                                f2_meta=f2_meta,
                                                            )
                                                else:
                                                    keepalive_reason = f"recent_local_anchor_plan_{cached_diag.get('reason', 'unavailable')}"
                                                _fed_dbg_main(
                                                    f"evt=F2_SELECTED_OFFER_SKIP tls={tls_id} ev={ev_id} "
                                                    f"reason=weak_offer_idempotent plan_type={selected_offer_plan_type} "
                                                    f"action={selected_offer_action} "
                                                    f"keepalive_applied={int(bool(keepalive_applied))} "
                                                    f"keepalive_effective_target_green={int(bool(keepalive_effective_target_green))} "
                                                    f"keepalive_reason={keepalive_reason} "
                                                    f"final_reason={str((f2_meta or {}).get('final_reason', ''))} "
                                                    f"sim={float(sim_time):.2f}"
                                                )
                                                _fed_evt_main(
                                                    "f2.selected_offer.skip",
                                                    role="intersection",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    reason="weak_offer_idempotent",
                                                    plan_type=str(selected_offer_plan_type),
                                                    offer_action=str(selected_offer_action),
                                                    final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                    selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                    keepalive_applied=bool(keepalive_applied),
                                                    keepalive_effective_target_green=bool(keepalive_effective_target_green),
                                                    keepalive_reason=str(keepalive_reason),
                                                    keepalive_source=str(keepalive_source),
                                                    keepalive_plan_type=str(keepalive_plan_type),
                                                )
                                            else:
                                                selected_effect_diag = _selected_offer_effect_diag(ag, chosen)
                                                if bool(selected_effect_diag.get("weak_effect", False)):
                                                    weak_fallback_applied = False
                                                    weak_fallback_reason = "no_local_fallback"
                                                    weak_fallback_plan_type = ""
                                                    weak_fallback_source = "none"
                                                    weak_fallback_before = None
                                                    weak_fallback_after = None
                                                    weak_skip_diag = {}
                                                    weak_local_plan, weak_local_plan_type = _f2_local_current_plan_for_actuation(ag)
                                                    if weak_local_plan is not None:
                                                        weak_fallback_source = "current_local_plan"
                                                    elif bool(getattr(args, "f2_selected_offer_recompute_local_fallback", True)):
                                                        try:
                                                            recomputed_local_plan = ag.tick(sim_time)
                                                        except Exception as e:
                                                            recomputed_local_plan = None
                                                            weak_fallback_reason = f"recompute_local_tick_error:{type(e).__name__}"
                                                            _fed_dbg_main(
                                                                f"evt=F2_SELECTED_OFFER_LOCAL_RECOMPUTE_WARN tls={tls_id} ev={ev_id} "
                                                                f"sim={float(sim_time):.2f} err={type(e).__name__}:{e}"
                                                            )
                                                        if recomputed_local_plan is not None:
                                                            recomputed_plan_type = str(getattr(recomputed_local_plan, "plan_type", "") or "")
                                                            if recomputed_plan_type in ("", "none", "restore"):
                                                                weak_fallback_reason = "recomputed_local_tick_weak_plan"
                                                                weak_fallback_plan_type = recomputed_plan_type
                                                            else:
                                                                weak_local_plan = recomputed_local_plan
                                                                weak_local_plan_type = recomputed_plan_type
                                                                weak_fallback_source = "recomputed_local_tick"
                                                        elif weak_fallback_reason == "no_local_fallback":
                                                            weak_fallback_reason = "recomputed_local_tick_none"
                                                    if weak_local_plan is None:
                                                        cached_plan, cached_plan_type, cached_diag = _recent_f2_last_local_anchor_plan(
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            selected_in_edge=str(selected_in_edge),
                                                        )
                                                        if cached_plan is not None:
                                                            weak_local_plan = cached_plan
                                                            weak_local_plan_type = cached_plan_type
                                                            weak_fallback_source = "recent_local_anchor_plan"
                                                            weak_fallback_reason = "recent_local_anchor_plan_available"
                                                            _fed_evt_main(
                                                                "f2.selected_offer.weak_local_cache.hit",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(cached_diag.get("reason", "")),
                                                                age_sec=cached_diag.get("age_sec"),
                                                                max_age_sec=cached_diag.get("max_age_sec"),
                                                                source=str(cached_diag.get("source", "")),
                                                                cached_selected_in_edge=str(cached_diag.get("cached_selected_in_edge", "")),
                                                                selected_in_edge=str(selected_in_edge),
                                                                cached_plan_type=str(cached_plan_type),
                                                            )
                                                        else:
                                                            weak_fallback_reason = (
                                                                weak_fallback_reason
                                                                if weak_fallback_reason not in ("no_local_fallback", "")
                                                                else f"recent_local_anchor_plan_{cached_diag.get('reason', 'unavailable')}"
                                                            )
                                                    if weak_local_plan is not None:
                                                        weak_local_plan_for_apply, weak_local_lookahead_diag = _route_lookahead_actuation_filter(
                                                            mode="F2",
                                                            stage="selected_offer_weak_effect_local_fallback",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            ev_edge=str(ev_edge),
                                                            selected_in_edge=str(selected_in_edge),
                                                            lookahead_hops=int(lookahead_hops),
                                                            d_stop=float(d_stop),
                                                            plan=weak_local_plan,
                                                        )
                                                        weak_fallback_plan_type = str(weak_local_plan_type)
                                                        if weak_local_plan_for_apply is None:
                                                            weak_fallback_reason = str(
                                                                weak_local_lookahead_diag.get("reason", "lookahead_guard")
                                                            )
                                                        else:
                                                            weak_fallback_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                            weak_skip_apply, weak_skip_diag = _should_skip_f2_local_anchor_preapply(
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                plan=weak_local_plan_for_apply,
                                                                tls_before=weak_fallback_before,
                                                                lookahead_diag=weak_local_lookahead_diag,
                                                            )
                                                            if not weak_skip_apply:
                                                                weak_skip_apply, weak_skip_diag = _should_skip_f2_target_pending_bridge(
                                                                    stage="selected_offer_weak_effect_local_fallback",
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    ag=ag,
                                                                    selected_in_edge=str(selected_in_edge),
                                                                    d_stop=float(d_stop),
                                                                    plan=weak_local_plan_for_apply,
                                                                    tls_before=weak_fallback_before,
                                                                    lookahead_diag=weak_local_lookahead_diag,
                                                                )
                                                            if weak_skip_apply:
                                                                weak_fallback_reason = str(
                                                                    weak_skip_diag.get("reason", "redundant_local_anchor_preapply")
                                                                )
                                                            else:
                                                                ag.apply_plan_to_tls(
                                                                    sim_time,
                                                                    weak_local_plan_for_apply,
                                                                    decision_source="f2_selected_offer_weak_effect_local_fallback",
                                                                )
                                                                weak_fallback_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                                weak_fallback_target_phase = _target_phase_for_diag(ag, str(selected_in_edge))
                                                                weak_fallback_target_green_after = _target_is_green_for_diag(
                                                                    weak_fallback_after,
                                                                    int(weak_fallback_target_phase),
                                                                )
                                                                weak_fallback_applied = True
                                                                weak_fallback_reason = (
                                                                    "applied_local_fallback"
                                                                    if bool(weak_fallback_target_green_after)
                                                                    else "applied_local_fallback_target_not_green"
                                                                )
                                                                weak_fallback_plan_type = str(
                                                                    getattr(weak_local_plan_for_apply, "plan_type", "") or weak_local_plan_type
                                                                )
                                                                _record_f2_target_pending_bridge(
                                                                    stage="selected_offer_weak_effect_local_fallback",
                                                                    decision_source="f2_selected_offer_weak_effect_local_fallback",
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    ag=ag,
                                                                    selected_in_edge=str(selected_in_edge),
                                                                    ev_edge=str(ev_edge),
                                                                    d_stop=float(d_stop),
                                                                    plan=weak_local_plan_for_apply,
                                                                    before=weak_fallback_before,
                                                                    after=weak_fallback_after,
                                                                    lookahead_diag=weak_local_lookahead_diag,
                                                                    reason=str(weak_fallback_reason),
                                                                    source=str(weak_fallback_source),
                                                                )
                                                                # Preserve the local B1 bridge even when the target phase
                                                                # is still pending. Replaying this bridge is how B1 keeps
                                                                # the service window alive while the EV is upstream.
                                                                _remember_f2_last_local_anchor_plan(
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    plan=weak_local_plan_for_apply,
                                                                    plan_type=str(weak_fallback_plan_type),
                                                                    selected_in_edge=str(selected_in_edge),
                                                                    lookahead_diag=weak_local_lookahead_diag,
                                                                    source=(
                                                                        "selected_offer_weak_effect_local_fallback"
                                                                        if bool(weak_fallback_target_green_after)
                                                                        else "selected_offer_weak_effect_local_fallback_target_pending"
                                                                    ),
                                                                )
                                                                _fed_evt_main(
                                                                    "f2.b1_bridge.apply",
                                                                    role="intersection",
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    reason=str(weak_fallback_reason),
                                                                    source=str(weak_fallback_source),
                                                                    stage="selected_offer_weak_effect_local_fallback",
                                                                    bridge_plan_type=str(weak_fallback_plan_type),
                                                                    target_phase=int(weak_fallback_target_phase),
                                                                    target_green_after=bool(weak_fallback_target_green_after),
                                                                    before_phase=int(weak_fallback_before.get("phase", -1)),
                                                                    after_phase=int(weak_fallback_after.get("phase", -1)),
                                                                    before_state=str(weak_fallback_before.get("state", "")),
                                                                    after_state=str(weak_fallback_after.get("state", "")),
                                                                    before_next_switch=float(weak_fallback_before.get("next_switch", -1.0)),
                                                                    after_next_switch=float(weak_fallback_after.get("next_switch", -1.0)),
                                                                    route_lookahead=bool(weak_local_lookahead_diag.get("route_lookahead", False)),
                                                                    lookahead_hops=int(weak_local_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                                    route_distance_to_selected_edge_m=weak_local_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                                )
                                                                _emit_f2_apply_effect(
                                                                    stage="selected_offer_weak_effect_local_fallback",
                                                                    decision_source="f2_selected_offer_weak_effect_local_fallback",
                                                                    ev_id=str(ev_id),
                                                                    tls_id=str(tls_id),
                                                                    sim_time=float(sim_time),
                                                                    ag=ag,
                                                                    selected_in_edge=str(selected_in_edge),
                                                                    ev_edge=str(ev_edge),
                                                                    d_stop=float(d_stop),
                                                                    before=weak_fallback_before,
                                                                    after=weak_fallback_after,
                                                                    offer=chosen,
                                                                    plan=weak_local_plan_for_apply,
                                                                    f2_meta=f2_meta,
                                                                )
                                                    _fed_dbg_main(
                                                        f"evt=F2_SELECTED_OFFER_SKIP tls={tls_id} ev={ev_id} "
                                                        f"reason=weak_offer_effective_extend "
                                                        f"plan_type={selected_effect_diag.get('plan_type') or selected_offer_plan_type} "
                                                        f"action={selected_effect_diag.get('offer_action') or selected_offer_action} "
                                                        f"effective_extend={float(selected_effect_diag.get('effective_extend_sec', 0.0) or 0.0):.3f} "
                                                        f"min_effective_extend={float(selected_effect_diag.get('min_effective_extend_sec', 0.5) or 0.5):.3f} "
                                                        f"fallback_applied={1 if weak_fallback_applied else 0} "
                                                        f"fallback_reason={weak_fallback_reason} "
                                                        f"fallback_source={weak_fallback_source} "
                                                        f"fallback_plan_type={weak_fallback_plan_type or '-'} "
                                                        f"final_reason={str((f2_meta or {}).get('final_reason', ''))} "
                                                        f"sim={float(sim_time):.2f}"
                                                    )
                                                    _fed_evt_main(
                                                        "f2.selected_offer.skip",
                                                        role="intersection",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        reason="weak_offer_effective_extend",
                                                        plan_type=str(selected_effect_diag.get("plan_type") or selected_offer_plan_type),
                                                        offer_action=str(selected_effect_diag.get("offer_action") or selected_offer_action),
                                                        effective_extend_sec=float(selected_effect_diag.get("effective_extend_sec", 0.0) or 0.0),
                                                        min_effective_extend_sec=float(
                                                            selected_effect_diag.get("min_effective_extend_sec", 0.5) or 0.5
                                                        ),
                                                        local_plan_type=str(local_current_plan_type),
                                                        fallback_applied=bool(weak_fallback_applied),
                                                        fallback_reason=str(weak_fallback_reason),
                                                        fallback_source=str(weak_fallback_source),
                                                        fallback_plan_type=str(weak_fallback_plan_type),
                                                        fallback_target_green_after=(
                                                            None
                                                            if weak_fallback_after is None
                                                            else bool(
                                                                _target_is_green_for_diag(
                                                                    weak_fallback_after,
                                                                    int(_target_phase_for_diag(ag, str(selected_in_edge))),
                                                                )
                                                            )
                                                        ),
                                                        final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                        refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                        selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                        selected_offer_plan_type=str(selected_offer_plan_type),
                                                        selected_offer_action=str(selected_offer_action),
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        distance_to_stopline_m=float(d_stop),
                                                    )
                                                    if not weak_fallback_applied:
                                                        _fed_evt_main(
                                                            "f2.selected_offer.skip_no_local_fallback",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(weak_fallback_reason),
                                                            local_plan_type=str(local_current_plan_type),
                                                            fallback_source=str(weak_fallback_source),
                                                            fallback_plan_type=str(weak_fallback_plan_type),
                                                            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            effective_extend_sec=float(
                                                                selected_effect_diag.get("effective_extend_sec", 0.0) or 0.0
                                                            ),
                                                            min_effective_extend_sec=float(
                                                                selected_effect_diag.get("min_effective_extend_sec", 0.5) or 0.5
                                                            ),
                                                        )
                                                        _fed_evt_main(
                                                            "f2.b1_bridge.skip",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(weak_fallback_reason),
                                                            source=str(weak_fallback_source),
                                                            stage="selected_offer_weak_effect_local_fallback",
                                                            local_plan_type=str(local_current_plan_type),
                                                            fallback_plan_type=str(weak_fallback_plan_type),
                                                            selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                            selected_offer_plan_type=str(selected_offer_plan_type),
                                                            selected_offer_action=str(selected_offer_action),
                                                            effective_extend_sec=float(
                                                                selected_effect_diag.get("effective_extend_sec", 0.0) or 0.0
                                                            ),
                                                            min_effective_extend_sec=float(
                                                                selected_effect_diag.get("min_effective_extend_sec", 0.5) or 0.5
                                                            ),
                                                            ev_edge=str(ev_edge),
                                                            selected_in_edge=str(selected_in_edge),
                                                            distance_to_stopline_m=float(d_stop),
                                                            target_phase=weak_skip_diag.get("target_phase"),
                                                            target_pending_count=weak_skip_diag.get("count"),
                                                            target_pending_suppress_after_n=weak_skip_diag.get("suppress_after_n"),
                                                            target_pending_late_rescue_active=weak_skip_diag.get("late_rescue_active"),
                                                        )
                                                    continue
                                                stage_before_apply_offer = str(ag.stage)
                                                f2_tls_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                selected_apply_guard_diag = _f2_downstream_apply_guard_diag(
                                                    stage="selected_offer",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    ev_edge=str(ev_edge),
                                                    selected_in_edge=str(selected_in_edge),
                                                    plan=None,
                                                    lookahead_diag={
                                                        "route_lookahead": bool(ev_edge != selected_in_edge or int(lookahead_hops) > 0),
                                                        "lookahead_hops": int(lookahead_hops),
                                                        "route_distance_to_selected_edge_m": float(d_stop) if str(ev_edge) == str(selected_in_edge) else None,
                                                        "upstream_stopped": False,
                                                        "action": str(selected_offer_action),
                                                        "reason": "selected_offer",
                                                    },
                                                    source="selected_offer",
                                                    plan_type_override=str(selected_offer_plan_type),
                                                    offer_action=str(selected_offer_action),
                                                )
                                                if bool(selected_apply_guard_diag.get("skip_apply", False)):
                                                    _fed_dbg_main(
                                                        f"evt=F2_SELECTED_OFFER_SKIP tls={tls_id} ev={ev_id} "
                                                        f"reason=downstream_apply_guard plan_type={selected_offer_plan_type} "
                                                        f"action={selected_offer_action} sim={float(sim_time):.2f} "
                                                        f"worst_edge={str(selected_apply_guard_diag.get('worst_edge', '')) or '-'}"
                                                    )
                                                    _fed_evt_main(
                                                        "f2.selected_offer.skip",
                                                        role="intersection",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        reason="downstream_apply_guard",
                                                        plan_type=str(selected_offer_plan_type),
                                                        offer_action=str(selected_offer_action),
                                                        selected_offer_id=str(getattr(chosen, "offer_id", "") or ""),
                                                        selected_offer_plan_type=str(selected_offer_plan_type),
                                                        selected_offer_action=str(selected_offer_action),
                                                        final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                        refine_reason=str((f2_meta or {}).get("refine_reason", "")),
                                                        downstream_apply_guard=dict(selected_apply_guard_diag),
                                                        before_phase=int(f2_tls_before.get("phase", -1)),
                                                        before_state=str(f2_tls_before.get("state", "")),
                                                        before_next_switch=float(f2_tls_before.get("next_switch", -1.0)),
                                                        before_next_switch_rem_s=float(f2_tls_before.get("next_switch_rem_s", -1.0)),
                                                    )
                                                    continue
                                                print(f"Applying offered selected")
                                                ag.apply_offer_to_tls(sim_time, chosen)
                                                f2_tls_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                stage_after_apply_offer = str(ag.stage)
                                                print(f"[F2-STAGE] tls={tls_id} stage_after_apply_offer={stage_after_apply_offer} (before_apply={stage_before_apply_offer})")
                                                _emit_f2_apply_effect(
                                                    stage="selected_offer",
                                                    decision_source="offer",
                                                    ev_id=str(ev_id),
                                                    tls_id=str(tls_id),
                                                    sim_time=float(sim_time),
                                                    ag=ag,
                                                    selected_in_edge=str(selected_in_edge),
                                                    ev_edge=str(ev_edge),
                                                    d_stop=float(d_stop),
                                                    before=f2_tls_before,
                                                    after=f2_tls_after,
                                                    offer=chosen,
                                                    plan=getattr(ag, "current_plan", None),
                                                    f2_meta=f2_meta,
                                                )

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
                                        elif (
                                            (not STATIC_PROGRAM)
                                            and (ag.current_plan is not None)
                                            and (not bool(getattr(args, "f2_strict_b1_floor_enable", False)))
                                        ):
                                            # If agent-level F2 selection blocks actuation (e.g., infeasible),
                                            # keep local actuation continuity via current committed plan.
                                            try:
                                                selected_none_plan_type = str(getattr(ag.current_plan, "plan_type", "") or "")
                                                if selected_none_plan_type in ("none", "restore"):
                                                    _fed_dbg_main(
                                                        f"evt=F2_SELECTED_NONE_SKIP tls={tls_id} ev={ev_id} "
                                                        f"reason=weak_plan_idempotent plan_type={selected_none_plan_type} "
                                                        f"final_reason={str((f2_meta or {}).get('final_reason', ''))} "
                                                        f"sim={float(sim_time):.2f}"
                                                    )
                                                else:
                                                    selected_none_plan_for_apply, selected_none_lookahead_diag = _route_lookahead_actuation_filter(
                                                        mode="F2",
                                                        stage="selected_none_continuity",
                                                        ev_id=str(ev_id),
                                                        tls_id=str(tls_id),
                                                        sim_time=float(sim_time),
                                                        ev_edge=str(ev_edge),
                                                        selected_in_edge=str(selected_in_edge),
                                                        lookahead_hops=int(lookahead_hops),
                                                        d_stop=float(d_stop),
                                                        plan=getattr(ag, "current_plan", None),
                                                    )
                                                    if selected_none_plan_for_apply is None:
                                                        _fed_evt_main(
                                                            "f2.selected_none.skip",
                                                            role="intersection",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            reason=str(selected_none_lookahead_diag.get("reason", "lookahead_guard")),
                                                            plan_type=str(selected_none_plan_type),
                                                            final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                            route_lookahead=bool(selected_none_lookahead_diag.get("route_lookahead", False)),
                                                            route_distance_to_selected_edge_m=selected_none_lookahead_diag.get("route_distance_to_selected_edge_m"),
                                                            lookahead_hops=int(selected_none_lookahead_diag.get("lookahead_hops", 0) or 0),
                                                        )
                                                    else:
                                                        f2_tls_before = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                        selected_none_plan_for_apply, approach_rescue_diag = _maybe_f2_approach_phase_rescue_plan(
                                                            stage="selected_none_continuity",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            ag=ag,
                                                            ev_edge=str(ev_edge),
                                                            selected_in_edge=str(selected_in_edge),
                                                            d_stop=float(d_stop),
                                                            plan=selected_none_plan_for_apply,
                                                            tls_before=f2_tls_before,
                                                            lookahead_diag=selected_none_lookahead_diag,
                                                            f2_meta=f2_meta,
                                                        )
                                                        skip_selected_none_slow, selected_none_slow_diag = False, {"reason": "approach_rescue_applied"}
                                                        if not bool(approach_rescue_diag.get("applied", False)):
                                                            skip_selected_none_slow, selected_none_slow_diag = _should_skip_f2_selected_none_slow_ev(
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                ag=ag,
                                                                plan=selected_none_plan_for_apply,
                                                                tls_before=f2_tls_before,
                                                                selected_in_edge=str(selected_in_edge),
                                                                d_stop=float(d_stop),
                                                                f2_meta=f2_meta,
                                                            )
                                                        if skip_selected_none_slow:
                                                            _fed_dbg_main(
                                                                f"evt=F2_SELECTED_NONE_SKIP tls={tls_id} ev={ev_id} "
                                                                f"reason=slow_ev_target_already_green "
                                                                f"plan_type={str(getattr(selected_none_plan_for_apply, 'plan_type', '') or '')} "
                                                                f"speed={float(selected_none_slow_diag.get('speed_mps', -1.0)):.2f} "
                                                                f"sim={float(sim_time):.2f}"
                                                            )
                                                            continue
                                                        selected_none_cadence_skip, selected_none_cadence_diag = False, {"reason": "approach_rescue_applied"}
                                                        if not bool(approach_rescue_diag.get("applied", False)):
                                                            selected_none_cadence_skip, selected_none_cadence_diag = _should_skip_f2_fallback_cadence(
                                                                stage="selected_none_continuity",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                plan=selected_none_plan_for_apply,
                                                                d_stop=float(d_stop),
                                                            )
                                                        if selected_none_cadence_skip:
                                                            _fed_evt_main(
                                                                "f2.selected_none.skip",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(selected_none_cadence_diag.get("reason", "cadence_guard")),
                                                                plan_type=str(getattr(selected_none_plan_for_apply, "plan_type", "") or ""),
                                                                final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                                fallback_cadence_guard=dict(selected_none_cadence_diag),
                                                            )
                                                            continue
                                                        ag.apply_plan_to_tls(
                                                            sim_time,
                                                            selected_none_plan_for_apply,
                                                            decision_source="f2_selected_none",
                                                        )
                                                        f2_tls_after = _tls_diag_snapshot(str(tls_id), float(sim_time))
                                                        _remember_f2_last_local_anchor_plan(
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            plan=selected_none_plan_for_apply,
                                                            plan_type=str(selected_none_plan_type),
                                                            selected_in_edge=str(selected_in_edge),
                                                            lookahead_diag=selected_none_lookahead_diag,
                                                            source="selected_none_continuity",
                                                        )
                                                        _emit_f2_apply_effect(
                                                            stage="selected_none_continuity",
                                                            decision_source="f2_selected_none",
                                                            ev_id=str(ev_id),
                                                            tls_id=str(tls_id),
                                                            sim_time=float(sim_time),
                                                            ag=ag,
                                                            selected_in_edge=str(selected_in_edge),
                                                            ev_edge=str(ev_edge),
                                                            d_stop=float(d_stop),
                                                            before=f2_tls_before,
                                                            after=f2_tls_after,
                                                            plan=selected_none_plan_for_apply,
                                                            f2_meta=f2_meta,
                                                        )
                                                        if bool(approach_rescue_diag.get("applied", False)):
                                                            _fed_evt_main(
                                                                "f2.selected_none.approach_phase_rescue.apply",
                                                                role="intersection",
                                                                ev_id=str(ev_id),
                                                                tls_id=str(tls_id),
                                                                sim_time=float(sim_time),
                                                                reason=str(approach_rescue_diag.get("reason", "")),
                                                                final_reason=str((f2_meta or {}).get("final_reason", "")),
                                                                original_plan_type=str(approach_rescue_diag.get("original_plan_type", "")),
                                                                applied_local_plan_type=str(getattr(selected_none_plan_for_apply, "plan_type", "") or ""),
                                                                target_phase=approach_rescue_diag.get("target_phase"),
                                                                before_phase=int(f2_tls_before.get("phase", -1)),
                                                                after_phase=int(f2_tls_after.get("phase", -1)),
                                                                target_green_after=bool(
                                                                    _target_is_green_for_diag(
                                                                        f2_tls_after,
                                                                        int(approach_rescue_diag.get("target_phase", -1) or -1),
                                                                    )
                                                                ),
                                                                route_distance_to_selected_edge_m=approach_rescue_diag.get("route_distance_to_selected_edge_m"),
                                                                speed_mps=approach_rescue_diag.get("speed_mps"),
                                                            )
                                                        _fed_dbg_main(
                                                            f"evt=F2_SELECTED_NONE_APPLY tls={tls_id} ev={ev_id} "
                                                            f"reason={str((f2_meta or {}).get('final_reason', ''))} "
                                                            f"plan_type={selected_none_plan_type} "
                                                            f"lookahead_action={str(selected_none_lookahead_diag.get('action', ''))} "
                                                            f"sim={float(sim_time):.2f}"
                                                        )
                                            except Exception as e:
                                                _fed_dbg_main(
                                                    f"evt=F2_SELECTED_NONE_WARN tls={tls_id} ev={ev_id} "
                                                    f"err={type(e).__name__}:{e}"
                                                )
                                    elif not STATIC_PROGRAM:
                                        # Local B1-equivalent continuity for F2:
                                        # if no peer offer is available/due, still preserve the EV-serving
                                        # TLS actuation window instead of letting F2 go idle.
                                        try:
                                            _try_f2_b1_continuity_apply(
                                                ag=ag,
                                                ev_id=str(ev_id),
                                                tls_id=str(tls_id),
                                                sim_time=float(sim_time),
                                                ev_edge=str(ev_edge),
                                                selected_in_edge=str(selected_in_edge),
                                                lookahead_hops=int(lookahead_hops),
                                                d_stop=float(d_stop),
                                                trigger_reason="eval_due_no_offers" if f2_eval_due else "eval_not_due",
                                                f2_meta=f2_meta,
                                            )
                                        except Exception as e:
                                            _fed_dbg_main(
                                                f"evt=F2_B1_CONTINUITY_WARN tls={tls_id} ev={ev_id} "
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
            if _is_f2_family(CURRENT_EVALUATION):
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
            if realtime_sumo_enabled:
                try:
                    if (
                        not realtime_pacing_active
                        and float(sim_time) + 1e-9 >= float(realtime_sumo_start_sim_time_sec)
                    ):
                        realtime_sim_start = float(sim_time)
                        realtime_wall_start = float(time.perf_counter())
                        realtime_sleep_acc_sec = 0.0
                        realtime_lag_acc_sec = 0.0
                        realtime_steps_acc = 0
                        realtime_pacing_active = True
                        _fed_dbg_main(
                            f"evt=REALTIME_SUMO_START sim_start={float(realtime_sim_start):.3f} "
                            f"wall_start={float(realtime_wall_start):.6f} delayed_start=1"
                        )
                        _fed_evt_main(
                            "realtime_sumo.start",
                            role="simulation",
                            sim_start=float(realtime_sim_start),
                            wall_start=float(realtime_wall_start),
                            delayed_start=True,
                            start_sim_time_sec=float(realtime_sumo_start_sim_time_sec),
                        )
                    if not realtime_pacing_active:
                        raise RuntimeError("realtime pacing not active during pre-roll")
                    sim_elapsed_rt = max(0.0, float(sim_time) - float(realtime_sim_start))
                    target_wall = float(realtime_wall_start) + (float(sim_elapsed_rt) / float(realtime_sumo_factor))
                    now_wall = float(time.perf_counter())
                    sleep_sec = max(0.0, min(float(realtime_sumo_max_sleep_sec), float(target_wall - now_wall)))
                    if sleep_sec > 0.0:
                        time.sleep(float(sleep_sec))
                        realtime_sleep_acc_sec += float(sleep_sec)
                    after_sleep_wall = float(time.perf_counter())
                    lag_sec = max(0.0, float(after_sleep_wall - target_wall))
                    realtime_lag_acc_sec += float(lag_sec)
                    realtime_steps_acc += 1
                    if _periodic_due(
                        float(sim_time),
                        periodic_last_t,
                        "realtime_sumo",
                        float(realtime_sumo_log_period_sec),
                    ):
                        n_rt = max(1, int(realtime_steps_acc))
                        mean_sleep_sec = float(realtime_sleep_acc_sec) / float(n_rt)
                        mean_lag_sec = float(realtime_lag_acc_sec) / float(n_rt)
                        _fed_dbg_main(
                            f"evt=REALTIME_SUMO_PACE sim={float(sim_time):.2f} "
                            f"factor={float(realtime_sumo_factor):.3f} "
                            f"mean_sleep_ms={1000.0 * mean_sleep_sec:.1f} "
                            f"mean_lag_ms={1000.0 * mean_lag_sec:.1f} "
                            f"steps={int(n_rt)}"
                        )
                        _fed_evt_main(
                            "realtime_sumo.pace",
                            role="simulation",
                            sim_time=float(sim_time),
                            factor=float(realtime_sumo_factor),
                            mean_sleep_ms=float(1000.0 * mean_sleep_sec),
                            mean_lag_ms=float(1000.0 * mean_lag_sec),
                            steps=int(n_rt),
                        )
                        realtime_sleep_acc_sec = 0.0
                        realtime_lag_acc_sec = 0.0
                        realtime_steps_acc = 0
                except RuntimeError:
                    pass
                except Exception as e:
                    _fed_dbg_main(f"evt=REALTIME_SUMO_PACE_ERR err={type(e).__name__}:{e}")
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
