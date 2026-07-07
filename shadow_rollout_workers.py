#!/usr/bin/env python3
"""
shadow_rollout_workers.py

Warm, parallel "shadow" SUMO rollouts for evaluating SignalWindowOffer candidates.

Design goals:
- Keep N shadow SUMO instances warm (one per worker process).
- Use TraCI loadState() to reset quickly between rollouts (no SUMO restart per offer).
- Evaluate offers in parallel across processes to reduce wall-clock latency.
- Return a per-offer cost + details for selection in the main process.

This module is intentionally self-contained and avoids importing your agent classes.
It expects offers to be either:
  - dataclass-like objects with attributes: offer_id, action, action_params, target_phase_idx
  - or dicts with equivalent keys.

It also expects the main simulation to provide a SUMO state snapshot path created via:
  traci.simulation.saveState(state_path)
"""

from __future__ import annotations

import os
import time
import socket
import uuid
import queue
import signal
import logging
import traceback
import subprocess
from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import multiprocessing as mp

try:
    import traci  # type: ignore
except Exception as e:  # pragma: no cover
    traci = None  # type: ignore


log = logging.getLogger("shadow_rollout")


OfferLike = Union[Dict[str, Any], Any]


def _offer_get(offer: OfferLike, key: str, default=None):
    if isinstance(offer, dict):
        return offer.get(key, default)
    return getattr(offer, key, default)


def _offer_to_dict(offer: OfferLike) -> Dict[str, Any]:
    """Serialize an offer into a dict that can be shipped across processes."""
    if isinstance(offer, dict):
        return dict(offer)
    # dataclass offer
    if hasattr(offer, "__dataclass_fields__"):
        try:
            return asdict(offer)  # type: ignore[arg-type]
        except Exception:
            pass
    # generic object
    out = {}
    for k in ("offer_id", "tls_id", "ev_id", "created_time", "target_phase_idx", "action", "action_params",
              "green_window", "expected_wait_sec", "expected_time_to_stopline_sec", "suggested_speed_mps",
              "notes"):
        if hasattr(offer, k):
            out[k] = getattr(offer, k)
    # ensure required keys exist
    out.setdefault("offer_id", str(uuid.uuid4()))
    out.setdefault("action_params", {})
    out.setdefault("target_phase_idx", int(out.get("target_phase_idx", 0)))
    out.setdefault("action", str(out.get("action", "none")))
    return out


def _port_is_free(port: int, host: str = "127.0.0.1") -> bool:
    """True when host:port appears free."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            return sock.connect_ex((host, int(port))) != 0
    except Exception:
        return False


def _reserve_free_ports(start_port: int, count: int, max_scan: int = 4000) -> List[int]:
    """Find a list of free ports scanning upward from start_port."""
    ports: List[int] = []
    p = int(start_port)
    scanned = 0
    while len(ports) < int(count) and scanned < int(max_scan):
        if _port_is_free(p):
            ports.append(p)
        p += 1
        scanned += 1
    if len(ports) < int(count):
        raise RuntimeError(f"Could not reserve {count} free ports from {start_port} (scan={max_scan}).")
    return ports


@dataclass
class ShadowRolloutResult:
    offer: Dict[str, Any]
    cost: float
    ev_travel_time: Optional[float]
    queue_cost: float
    detail: Dict[str, Any]


@dataclass
class ShadowRolloutPoolConfig:
    sumo_binary: str
    sumo_cfg: str
    step_length: float = 0.1
    num_workers: int = 2
    base_port: int = 9910
    warmup_steps: int = 1
    controlled_lanes_cache: bool = True

    # Cost weights
    w_ev: float = 1.0
    w_queue: float = 0.05

    # Restart worker SUMO if it errors this many times in a row
    max_consecutive_errors: int = 3


class ShadowRolloutPool:
    """
    Manages a pool of warm shadow SUMO worker processes.

    Main usage:
        pool = ShadowRolloutPool(cfg)
        pool.start()
        best_offer, results = pool.pick_best_offer(state_path, offers, ev_id, tls_id, horizon_sec=30, timeout_sec=1.0)
        pool.close()
    """

    def __init__(self, cfg: ShadowRolloutPoolConfig):
        if traci is None:
            raise RuntimeError("TraCI (traci) is not available in this Python environment.")
        self.cfg = cfg
        self._ctx = mp.get_context("spawn")
        self._req_q: mp.Queue = self._ctx.Queue()
        self._resp_q: mp.Queue = self._ctx.Queue()
        self._procs: List[mp.Process] = []
        self._started = False
        self._rollout_enabled = False
        self._ports: List[int] = []
        self._shutdown = self._ctx.Event()

    def start(self) -> None:
        if self._started:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.cfg.sumo_cfg)) or ".", exist_ok=True)

        # Prefer free ports to avoid collisions with stale/orphan SUMO workers.
        self._ports = _reserve_free_ports(int(self.cfg.base_port), int(self.cfg.num_workers))

        for wid in range(self.cfg.num_workers):
            port = int(self._ports[wid])
            p = self._ctx.Process(
                target=_shadow_worker_main,
                args=(wid, port, self.cfg, self._req_q, self._resp_q, self._shutdown),
                daemon=True,
            )
            p.start()
            self._procs.append(p)

        # Give workers a brief moment to either come up or fail fast.
        time.sleep(0.25)
        alive = [p.is_alive() for p in self._procs]
        self._rollout_enabled = any(alive)
        if not self._rollout_enabled:
            print("[ShadowRolloutPool] No live shadow workers after startup; rollout disabled.")
        else:
            print(f"[ShadowRolloutPool] Live workers: {sum(alive)}/{len(self._procs)} on ports {self._ports}")
        self._started = True

    def has_live_workers(self) -> bool:
        return any(p.is_alive() for p in self._procs)

    def close(self) -> None:
        if not self._started:
            return
        self._shutdown.set()
        # try to drain/notify
        try:
            for _ in self._procs:
                self._req_q.put_nowait({"type": "shutdown"})
        except Exception:
            pass
        for p in self._procs:
            p.join(timeout=2.0)
        for p in self._procs:
            if p.is_alive():
                p.kill()
        self._started = False

    def pick_best_offer(
        self,
        base_state_path: str,
        offers: Sequence[OfferLike],
        ev_id: str,
        tls_id: str,
        horizon_sec: float,
        timeout_sec: float,
        edge_ids_for_cost: Optional[List[str]] = None,
    ) -> Tuple[Optional[Dict[str, Any]], List[ShadowRolloutResult]]:
        """
        Evaluate offers in parallel and return the best offer (dict) plus all results.
        If no results arrive within timeout, returns (None, []).
        """
        if not self._started:
            raise RuntimeError("ShadowRolloutPool.start() must be called before pick_best_offer().")

        if not self._rollout_enabled:
            return None, []
        if not any(p.is_alive() for p in self._procs):
            self._rollout_enabled = False
            return None, []

        job_id = str(uuid.uuid4())
        offer_dicts = [_offer_to_dict(o) for o in offers]

        # One task per offer (lets the pool parallelize).
        for idx, od in enumerate(offer_dicts):
            self._req_q.put({
                "type": "rollout",
                "job_id": job_id,
                "task_id": f"{job_id}:{idx}",
                "base_state_path": base_state_path,
                "offer": od,
                "ev_id": ev_id,
                "tls_id": tls_id,
                "horizon_sec": float(horizon_sec),
                "edge_ids_for_cost": edge_ids_for_cost or [],
            })

        deadline = time.time() + float(timeout_sec)
        results: List[ShadowRolloutResult] = []
        expected = len(offer_dicts)

        while time.time() < deadline and len(results) < expected:
            remaining = max(0.0, deadline - time.time())
            try:
                msg = self._resp_q.get(timeout=min(0.05, remaining))
            except queue.Empty:
                continue
            if not isinstance(msg, dict):
                continue
            if msg.get("job_id") != job_id:
                continue
            if msg.get("type") != "result":
                continue
            try:
                results.append(ShadowRolloutResult(
                    offer=msg["offer"],
                    cost=float(msg["cost"]),
                    ev_travel_time=msg.get("ev_travel_time"),
                    queue_cost=float(msg.get("queue_cost", 0.0)),
                    detail=msg.get("detail", {}) or {},
                ))
            except Exception:
                continue

        if not results:
            return None, []

        best = min(results, key=lambda r: r.cost)
        return best.offer, results


# ------------------------------- Worker internals -------------------------------

def _start_warm_sumo(sumo_binary: str, sumo_cfg: str, step_length: float, port: int) -> "traci.Connection":
    cmd = [
        sumo_binary,
        "-c", sumo_cfg,
        "--step-length", str(step_length),
        "--start",
        "--no-step-log", "true",
        "--duration-log.disable", "true",
    ]
    traci.start(cmd, port=port, label=f"shadow-{port}")
    return traci.getConnection(f"shadow-{port}")


def _restart_warm_sumo(conn: Optional["traci.Connection"], sumo_binary: str, sumo_cfg: str, step_length: float, port: int) -> "traci.Connection":
    try:
        if conn is not None:
            conn.close()
    except Exception:
        pass
    try:
        # ensure the old label is gone
        traci.switch(f"shadow-{port}")
    except Exception:
        pass
    return _start_warm_sumo(sumo_binary, sumo_cfg, step_length, port)


def _apply_offer_semantics(conn: "traci.Connection", offer: Dict[str, Any], tls_id: str, t0: float, t_now: float, state: Dict[str, Any]) -> None:
    """
    Applies offer semantics with internal state to ensure one-shot application:
      - extend: when phase becomes target, extend once
      - hurry: at t0, shorten current phase once
      - jump: at jump_time, set phase once
    """
    action = str(offer.get("action", "none"))
    params = offer.get("action_params", {}) or {}
    target_phase = int(offer.get("target_phase_idx", 0))

    if action == "none":
        return

    # jump
    if action == "jump":
        jump_time = float(params.get("jump_time", t0))
        if not state.get("jump_done", False) and t_now >= jump_time:
            try:
                conn.trafficlight.setPhase(tls_id, target_phase)
                # keep it from immediately switching
                conn.trafficlight.setPhaseDuration(tls_id, max(1.0, float(params.get("hold_sec", 1.0))))
            except Exception:
                pass
            state["jump_done"] = True
        return

    # hurry: apply once at start (t0)
    if action == "hurry":
        if state.get("hurry_done", False):
            return
        hurry_to = float(params.get("hurry_to", params.get("remaining_sec", 0.0)))
        min_rem = float(params.get("min_remaining_sec", 1.0))
        try:
            sim_time = float(conn.simulation.getTime())
            next_switch = float(conn.trafficlight.getNextSwitch(tls_id))
            rem = max(0.0, next_switch - sim_time)
            new_rem = max(min_rem, min(hurry_to, rem))
            if new_rem < rem:
                conn.trafficlight.setPhaseDuration(tls_id, float(new_rem))
        except Exception:
            pass
        state["hurry_done"] = True
        return

    # extend: apply once when target becomes current
    if action == "extend":
        if state.get("extend_done", False):
            return
        ext = float(params.get("ext", 0.0))
        if ext <= 0:
            state["extend_done"] = True
            return
        try:
            cur = int(conn.trafficlight.getPhase(tls_id))
            if cur == target_phase:
                sim_time = float(conn.simulation.getTime())
                next_switch = float(conn.trafficlight.getNextSwitch(tls_id))
                rem = max(0.0, next_switch - sim_time)
                conn.trafficlight.setPhaseDuration(tls_id, float(rem + ext))
                state["extend_done"] = True
        except Exception:
            pass
        return


def _vehicle_passed_tls(conn: "traci.Connection", ev_id: str, tls_id: str) -> bool:
    """Return True if EV has passed this TLS (tls_id no longer in nextTLS list)."""
    try:
        if ev_id not in conn.vehicle.getIDList():
            return True
    except Exception:
        return True

    try:
        nxt = conn.vehicle.getNextTLS(ev_id)
        # nxt elements: (tlsID, dist, state, nextSignal, ...)
        for item in nxt:
            if len(item) >= 1 and str(item[0]) == str(tls_id):
                return False
        return True
    except Exception:
        # fallback: if query fails, assume not passed
        return False


def _queue_cost_integral(conn: "traci.Connection", tls_id: str, step_length: float, edge_ids: Optional[List[str]] = None, cached: Optional[List[str]] = None) -> float:
    """Approx queue cost: integrate halting vehicles on controlled lanes or specified edges."""
    cost = 0.0
    if edge_ids:
        for e in edge_ids:
            try:
                cost += float(conn.edge.getLastStepHaltingNumber(e)) * step_length
            except Exception:
                pass
        return cost

    lanes = cached
    if lanes is None:
        try:
            lanes = list(dict.fromkeys(conn.trafficlight.getControlledLanes(tls_id)))
        except Exception:
            lanes = []

    for ln in lanes:
        try:
            cost += float(conn.lane.getLastStepHaltingNumber(ln)) * step_length
        except Exception:
            pass
    return cost


def _simulate_offer(
    conn: "traci.Connection",
    cfg: ShadowRolloutPoolConfig,
    base_state_path: str,
    offer: Dict[str, Any],
    ev_id: str,
    tls_id: str,
    horizon_sec: float,
    edge_ids_for_cost: Optional[List[str]],
    cached_lanes: Optional[List[str]],
) -> ShadowRolloutResult:
    # Reset to base state
    if not hasattr(conn.simulation, "loadState"):
        raise RuntimeError("This SUMO/TraCI build does not support simulation.loadState(). Warm rollouts need it.")

    conn.simulation.loadState(base_state_path)

    t0 = float(conn.simulation.getTime())
    t_end = t0 + float(horizon_sec)
    step_len = float(cfg.step_length)

    sched_state: Dict[str, Any] = {}
    queue_cost = 0.0
    passed_time: Optional[float] = None

    # One immediate tick to ensure detectors update
    for _ in range(max(1, int(cfg.warmup_steps))):
        # apply any immediate actions at the very start
        _apply_offer_semantics(conn, offer, tls_id, t0=t0, t_now=float(conn.simulation.getTime()), state=sched_state)
        conn.simulationStep()

    while float(conn.simulation.getTime()) < t_end:
        t_now = float(conn.simulation.getTime())
        _apply_offer_semantics(conn, offer, tls_id, t0=t0, t_now=t_now, state=sched_state)

        # Step simulation
        conn.simulationStep()

        # Disruption cost
        queue_cost += _queue_cost_integral(
            conn, tls_id, step_length=step_len, edge_ids=edge_ids_for_cost, cached=cached_lanes
        )

        # EV crossing
        if passed_time is None and _vehicle_passed_tls(conn, ev_id, tls_id):
            passed_time = float(conn.simulation.getTime())
            break

    ev_travel_time = None if passed_time is None else max(0.0, passed_time - t0)

    # If not passed within horizon, penalize with horizon time.
    ev_term = float(horizon_sec) if ev_travel_time is None else float(ev_travel_time)

    cost = float(cfg.w_ev) * ev_term + float(cfg.w_queue) * float(queue_cost)
    detail = {"t0": t0, "t_end": t_end, "passed_time": passed_time, "sched_state": sched_state}

    return ShadowRolloutResult(
        offer=offer,
        cost=cost,
        ev_travel_time=ev_travel_time,
        queue_cost=float(queue_cost),
        detail=detail,
    )


def _shadow_worker_main(
    worker_id: int,
    port: int,
    cfg: ShadowRolloutPoolConfig,
    req_q: "mp.Queue",
    resp_q: "mp.Queue",
    shutdown_evt: "mp.Event",
) -> None:
    logging.basicConfig(level=logging.INFO, format=f"[shadow-worker {worker_id}] %(levelname)s: %(message)s")

    # Ensure SIGINT/SIGTERM terminate cleanly.
    def _handle(sig, frame):
        shutdown_evt.set()
    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)

    conn = None
    consecutive_errors = 0
    cached_lanes: Optional[List[str]] = None

    try:
        conn = _start_warm_sumo(cfg.sumo_binary, cfg.sumo_cfg, cfg.step_length, port)
        # Step a tiny bit so the connection is fully alive.
        for _ in range(max(1, int(cfg.warmup_steps))):
            conn.simulationStep()
    except Exception:
        log.exception("Failed to start warm shadow SUMO (worker %s, port %s).", worker_id, port)
        return

    while not shutdown_evt.is_set():
        try:
            msg = req_q.get(timeout=0.2)
        except queue.Empty:
            continue
        except Exception:
            continue

        if not isinstance(msg, dict):
            continue
        if msg.get("type") == "shutdown":
            break
        if msg.get("type") != "rollout":
            continue

        job_id = msg.get("job_id")
        try:
            base_state_path = str(msg["base_state_path"])
            offer = dict(msg["offer"])
            ev_id = str(msg["ev_id"])
            tls_id = str(msg["tls_id"])
            horizon_sec = float(msg["horizon_sec"])
            edge_ids = list(msg.get("edge_ids_for_cost") or [])

            # Cache controlled lanes per worker/tls_id if enabled
            if cfg.controlled_lanes_cache:
                if cached_lanes is None:
                    try:
                        cached_lanes = list(dict.fromkeys(conn.trafficlight.getControlledLanes(tls_id)))
                    except Exception:
                        cached_lanes = []

            res = _simulate_offer(
                conn=conn,
                cfg=cfg,
                base_state_path=base_state_path,
                offer=offer,
                ev_id=ev_id,
                tls_id=tls_id,
                horizon_sec=horizon_sec,
                edge_ids_for_cost=edge_ids if edge_ids else None,
                cached_lanes=cached_lanes,
            )
            consecutive_errors = 0

            resp_q.put({
                "type": "result",
                "job_id": job_id,
                "offer": res.offer,
                "cost": res.cost,
                "ev_travel_time": res.ev_travel_time,
                "queue_cost": res.queue_cost,
                "detail": res.detail,
            })

        except Exception as e:
            consecutive_errors += 1
            resp_q.put({
                "type": "result",
                "job_id": job_id,
                "offer": msg.get("offer", {}),
                "cost": float("inf"),
                "ev_travel_time": None,
                "queue_cost": 0.0,
                "detail": {"error": repr(e), "traceback": traceback.format_exc()},
            })

            # Restart SUMO if it's stuck or repeatedly failing
            if consecutive_errors >= int(cfg.max_consecutive_errors):
                try:
                    conn = _restart_warm_sumo(conn, cfg.sumo_binary, cfg.sumo_cfg, cfg.step_length, port)
                    cached_lanes = None
                    consecutive_errors = 0
                except Exception:
                    log.exception("Failed to restart warm shadow SUMO after errors.")
                    break

    try:
        if conn is not None:
            conn.close()
    except Exception:
        pass
