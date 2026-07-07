from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class ERSConfig:
    system_id: str = "ers_main"
    w_wait: float = 1.0
    w_miss: float = 4.0
    w_cost: float = 0.05
    w_conf_risk: float = 0.25
    prefer_feasible: bool = True
    enable_handoff_policy: bool = True
    handoff_top_k: int = 2


class EmergencyResponseSystemAgent:
    """
    ERS-level coordinator for EV-trip supervision.

    Minimal message-contract methods:
    - request_offer(...)
    - select_offer(...)
    - handoff_policy(...)
    """

    def __init__(self, cfg: ERSConfig):
        self.cfg = cfg
        self.vehicle_profiles: Dict[str, Dict[str, Any]] = {}
        self.vehicle_state: Dict[str, Dict[str, Any]] = {}
        self.last_offer_selection: Dict[Tuple[str, str], Dict[str, Any]] = {}

    def register_vehicle(self, profile: Dict[str, Any]) -> None:
        ev_id = str(profile.get("evId", ""))
        if not ev_id:
            return
        self.vehicle_profiles[ev_id] = dict(profile)

    def update_vehicle_state(self, ev_id: str, snapshot: Dict[str, Any]) -> None:
        self.vehicle_state[str(ev_id)] = dict(snapshot)

    def request_offer(
        self,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        approach_edge: str,
        route_intersections: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        return {
            "kind": "request_offer",
            "systemId": str(self.cfg.system_id),
            "evId": str(ev_id),
            "tlsId": str(tls_id),
            "simTime": float(sim_time),
            "approachEdge": str(approach_edge),
            "routeIntersections": list(route_intersections or []),
        }

    def _score_offer(self, offer: Dict[str, Any]) -> float:
        feasible = bool(offer.get("feasible", False))
        wait = max(0.0, float(offer.get("expected_wait_sec", 0.0)))
        miss = max(0.0, float(offer.get("expected_miss_sec", 0.0)))
        cost = max(0.0, float(offer.get("cost_to_others_veh_sec", 0.0)))
        conf = max(0.0, min(1.0, float(offer.get("confidence", 0.5))))
        conf_risk = 1.0 - conf

        score = (
            float(self.cfg.w_wait) * wait
            + float(self.cfg.w_miss) * miss
            + float(self.cfg.w_cost) * cost
            + float(self.cfg.w_conf_risk) * conf_risk
        )
        if self.cfg.prefer_feasible and not feasible:
            score += 1e6
        return float(score)

    def select_offer(
        self,
        ev_id: str,
        tls_id: str,
        sim_time: float,
        offers: List[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        if not offers:
            return None

        ranked = sorted(
            (
                {
                    "offerId": str(o.get("offer_id", "")),
                    "score": self._score_offer(o),
                    "action": str(o.get("action", "")),
                    "targetPhase": int(o.get("target_phase_idx", 0)),
                }
                for o in offers
            ),
            key=lambda x: float(x["score"]),
        )
        best = ranked[0]
        msg = {
            "kind": "select_offer",
            "systemId": str(self.cfg.system_id),
            "evId": str(ev_id),
            "tlsId": str(tls_id),
            "simTime": float(sim_time),
            "selectedOfferId": str(best["offerId"]),
            "selectedAction": str(best["action"]),
            "selectedTargetPhase": int(best["targetPhase"]),
            "selectedScore": float(best["score"]),
            "ranked": ranked[: min(5, len(ranked))],
        }
        self.last_offer_selection[(str(ev_id), str(tls_id))] = dict(msg)
        return msg

    def handoff_policy(
        self,
        ev_id: str,
        from_tls: str,
        to_tls_candidates: List[str],
        sim_time: float,
    ) -> Dict[str, Any]:
        ranked = list(dict.fromkeys(str(t) for t in to_tls_candidates if str(t)))
        top = ranked[: max(1, int(self.cfg.handoff_top_k))]
        return {
            "kind": "handoff_policy",
            "systemId": str(self.cfg.system_id),
            "evId": str(ev_id),
            "fromTls": str(from_tls),
            "simTime": float(sim_time),
            "allowedNextTls": top,
            "allCandidates": ranked,
        }

