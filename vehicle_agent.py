from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    import traci
except Exception:
    traci = None  # type: ignore

from intersection_agent import EvRequest


@dataclass
class EmergencyVehicleProfile:
    ev_id: str
    unit_id: str = "ambulance_1"
    description: str = "Emergency medical transport unit"
    agency: str = "EMS"
    erl_level: int = 1
    metadata: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {
            "evId": self.ev_id,
            "unitId": self.unit_id,
            "description": self.description,
            "agency": self.agency,
            "erlLevel": int(self.erl_level),
            "metadata": dict(self.metadata),
        }


@dataclass
class VehicleAgentSnapshot:
    sim_time: float
    ev_id: str
    exists_in_sim: bool
    x: float = 0.0
    y: float = 0.0
    speed_mps: float = 0.0
    acceleration_mps2: float = 0.0
    angle_deg: float = 0.0
    edge_id: str = ""
    lane_id: str = ""
    lane_index: int = -1
    lane_pos_m: float = 0.0
    lane_length_m: float = 0.0
    dist_to_stopline_m: float = 0.0
    route_index: int = -1
    route_edges: List[str] = field(default_factory=list)
    next_tls: List[Tuple[str, float]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, object]:
        return {
            "simTime": float(self.sim_time),
            "evId": self.ev_id,
            "existsInSim": bool(self.exists_in_sim),
            "x": float(self.x),
            "y": float(self.y),
            "speedMps": float(self.speed_mps),
            "accelMps2": float(self.acceleration_mps2),
            "angleDeg": float(self.angle_deg),
            "edgeId": self.edge_id,
            "laneId": self.lane_id,
            "laneIndex": int(self.lane_index),
            "lanePosM": float(self.lane_pos_m),
            "laneLengthM": float(self.lane_length_m),
            "distToStoplineM": float(self.dist_to_stopline_m),
            "routeIndex": int(self.route_index),
            "routeEdges": list(self.route_edges),
            "nextTls": [[str(t), float(d)] for t, d in self.next_tls],
        }


class EmergencyVehicleAgent:
    """
    Mobile EV-side agent.
    Maintains EV profile + live kinematic/routing snapshot and provides
    helper methods to generate intersection-facing EvRequest payloads.
    """

    def __init__(
        self,
        profile: EmergencyVehicleProfile,
        default_delta_sec: float = 2.0,
    ) -> None:
        self.profile = profile
        self.default_delta_sec = float(default_delta_sec)
        self.current_snapshot: Optional[VehicleAgentSnapshot] = None
        self.last_seen_sim_time: float = -1.0

    def _distance_to_stopline(self, ev_id: str) -> float:
        if traci is None:
            return 1e9
        try:
            lane_id = traci.vehicle.getLaneID(ev_id)
            lane_pos = float(traci.vehicle.getLanePosition(ev_id))
            lane_len = float(traci.lane.getLength(lane_id))
            return max(0.0, lane_len - lane_pos)
        except Exception:
            return 1e9

    def update(self, sim_time: float) -> VehicleAgentSnapshot:
        ev_id = str(self.profile.ev_id)
        if traci is None:
            snap = VehicleAgentSnapshot(sim_time=float(sim_time), ev_id=ev_id, exists_in_sim=False)
            self.current_snapshot = snap
            return snap

        live = set(traci.vehicle.getIDList())
        if ev_id not in live:
            snap = VehicleAgentSnapshot(sim_time=float(sim_time), ev_id=ev_id, exists_in_sim=False)
            self.current_snapshot = snap
            return snap

        try:
            x, y = traci.vehicle.getPosition(ev_id)
        except Exception:
            x, y = 0.0, 0.0

        try:
            speed = float(traci.vehicle.getSpeed(ev_id))
        except Exception:
            speed = 0.0

        try:
            accel = float(traci.vehicle.getAcceleration(ev_id))
        except Exception:
            accel = 0.0

        try:
            angle = float(traci.vehicle.getAngle(ev_id))
        except Exception:
            angle = 0.0

        try:
            edge = str(traci.vehicle.getRoadID(ev_id))
        except Exception:
            edge = ""

        try:
            lane_id = str(traci.vehicle.getLaneID(ev_id))
            lane_idx = int(traci.vehicle.getLaneIndex(ev_id))
            lane_pos = float(traci.vehicle.getLanePosition(ev_id))
            lane_len = float(traci.lane.getLength(lane_id))
        except Exception:
            lane_id = ""
            lane_idx = -1
            lane_pos = 0.0
            lane_len = 0.0

        try:
            route_index = int(traci.vehicle.getRouteIndex(ev_id))
        except Exception:
            route_index = -1

        try:
            route_edges = list(traci.vehicle.getRoute(ev_id))
        except Exception:
            route_edges = []

        try:
            nxt = traci.vehicle.getNextTLS(ev_id) or []
            next_tls = []
            for it in nxt:
                try:
                    tls_id = str(it[0])
                    # TraCI getNextTLS tuple is typically (tls_id, tls_index, dist_m, state).
                    # Use dist_m when present; fall back to index-1 only for legacy 2-tuples.
                    if isinstance(it, (list, tuple)) and len(it) >= 3:
                        dist_m = float(it[2])
                    elif isinstance(it, (list, tuple)) and len(it) >= 2:
                        dist_m = float(it[1])
                    else:
                        continue
                    next_tls.append((tls_id, dist_m))
                except Exception:
                    continue
        except Exception:
            next_tls = []

        snap = VehicleAgentSnapshot(
            sim_time=float(sim_time),
            ev_id=ev_id,
            exists_in_sim=True,
            x=float(x),
            y=float(y),
            speed_mps=float(speed),
            acceleration_mps2=float(accel),
            angle_deg=float(angle),
            edge_id=edge,
            lane_id=lane_id,
            lane_index=lane_idx,
            lane_pos_m=float(lane_pos),
            lane_length_m=float(lane_len),
            dist_to_stopline_m=float(self._distance_to_stopline(ev_id)),
            route_index=route_index,
            route_edges=route_edges,
            next_tls=next_tls,
        )
        self.current_snapshot = snap
        self.last_seen_sim_time = float(sim_time)
        return snap

    def infer_route_intersections(
        self,
        edge_to_to_node: Dict[str, str],
        max_hops: int = 8,
    ) -> List[str]:
        snap = self.current_snapshot
        if snap is None or not snap.route_edges:
            return []
        idx = max(0, int(snap.route_index))
        out: List[str] = []
        seen = set()
        for e in snap.route_edges[idx:]:
            if e.startswith(":"):
                continue
            node = edge_to_to_node.get(e)
            if not node:
                continue
            if node in seen:
                continue
            seen.add(node)
            out.append(str(node))
            if len(out) >= int(max_hops):
                break
        return out

    def build_ev_request(
        self,
        sim_time: float,
        approach_edge: str,
        distance_to_intersection_m: float,
        target_phase_idx: Optional[int] = None,
        erl_level: Optional[int] = None,
        delta_sec: Optional[float] = None,
        route_intersections: Optional[List[str]] = None,
        route_veh: Optional[List[str]] = None,
    ) -> EvRequest:
        snap = self.current_snapshot
        speed = float(snap.speed_mps) if snap is not None else 0.0
        return EvRequest(
            ev_id=str(self.profile.ev_id),
            sim_time=float(sim_time),
            erl_level=int(erl_level if erl_level is not None else self.profile.erl_level),
            speed_mps=float(speed),
            distance_to_intersection_m=float(distance_to_intersection_m),
            in_edge_id=str(approach_edge),
            target_phase_idx=target_phase_idx,
            delta_sec=float(self.default_delta_sec if delta_sec is None else delta_sec),
            route_intersections=list(route_intersections) if route_intersections else None,
            route_veh=list(route_veh) if route_veh else None,
        )
