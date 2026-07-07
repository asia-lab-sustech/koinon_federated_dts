from __future__ import annotations

from typing import Any, Dict, List, Tuple


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _as_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _as_list(v: Any) -> List[Any]:
    if v is None:
        return []
    if isinstance(v, list):
        return v
    return [v]


def _as_str_list(v: Any) -> List[str]:
    out: List[str] = []
    for x in _as_list(v):
        s = str(x).strip()
        if s:
            out.append(s)
    return out


def _as_float_map(value: Any) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not isinstance(value, dict):
        return out
    for k, v in value.items():
        kk = str(k).strip()
        if not kk:
            continue
        vv = _as_float(v, -1.0)
        if vv >= 0.0:
            out[kk] = vv
    return out


class PeerSelectionPolicy:
    """Domain-agnostic policy interface used by FederationContextManager."""

    name = "default"

    def normalize_context(self, raw_context: Dict[str, Any]) -> Dict[str, Any]:
        return dict(raw_context or {})

    def candidate_allowed(self, node_id: str, context: Dict[str, Any]) -> bool:
        return True

    def candidate_rank_key(self, node_id: str, context: Dict[str, Any]) -> Tuple[float, float, str]:
        return (0.0, 0.0, str(node_id or ""))


class DefaultPeerSelectionPolicy(PeerSelectionPolicy):
    name = "default"

    def candidate_rank_key(self, node_id: str, context: Dict[str, Any]) -> Tuple[float, float, str]:
        return (0.0, 0.0, str(node_id or ""))


class TransportEVCorridorPolicy(PeerSelectionPolicy):
    """
    Transport-specific policy:
    - Route-aware gating around current TLS
    - Route/proximity ranking to keep nearest relevant peers first
    """

    name = "transport_ev_corridor"

    def __init__(self, cfg: Dict[str, Any]) -> None:
        c = dict(cfg or {})
        self.context_gate_enabled = bool(c.get("context_gate_enabled", True))
        self.context_back_hops = max(0, _as_int(c.get("context_back_hops", 1), 1))
        self.allow_fallback_without_route = bool(c.get("allow_fallback_without_route", True))
        self.prefer_forward_only = bool(c.get("prefer_forward_only", False))
        self.use_next_tls_order_when_route_missing = bool(c.get("use_next_tls_order_when_route_missing", True))
        self.max_candidates_min = max(0, _as_int(c.get("max_candidates_min", 0), 0))
        self.max_candidates_max = max(0, _as_int(c.get("max_candidates_max", 0), 0))
        weights = dict(c.get("rank_weights", {}) or {})
        self.weight_route_order = max(0.0, _as_float(weights.get("route_order", 1.0), 1.0))
        self.weight_proximity_m = max(0.0, _as_float(weights.get("proximity_m", 1.0), 1.0))
        self.route_backward_penalty = max(0.0, _as_float(c.get("route_backward_penalty", 1000.0), 1000.0))
        self.distance_unknown_penalty = max(0.0, _as_float(c.get("distance_unknown_penalty", 1e9), 1e9))
        self.route_unknown_penalty = max(0.0, _as_float(c.get("route_unknown_penalty", 1e9), 1e9))

    def _route_sequence(self, context: Dict[str, Any]) -> List[str]:
        route_seq = [str(x).strip() for x in _as_str_list(context.get("route_tls_sequence", [])) if str(x).strip()]
        if not route_seq:
            route_seq = [str(x).strip() for x in _as_str_list(context.get("route_sequence", [])) if str(x).strip()]
        if not route_seq and self.use_next_tls_order_when_route_missing:
            route_seq = [str(x).strip() for x in _as_str_list(context.get("next_tls_order", [])) if str(x).strip()]
        return route_seq

    def normalize_context(self, raw_context: Dict[str, Any]) -> Dict[str, Any]:
        c = dict(raw_context or {})
        route_seq = self._route_sequence(c)
        next_order = [str(x).strip() for x in _as_str_list(c.get("next_tls_order", [])) if str(x).strip()]
        next_dist = _as_float_map(c.get("next_tls_distance_m", {}))
        current_tls = str(c.get("current_tls", "") or "").strip()
        if not current_tls and next_order:
            current_tls = next_order[0]
        if not route_seq and next_order and self.use_next_tls_order_when_route_missing:
            route_seq = list(next_order)
        max_candidates = max(0, _as_int(c.get("max_candidates", 0), 0))
        if self.max_candidates_min > 0:
            max_candidates = max(max_candidates, self.max_candidates_min)
        if self.max_candidates_max > 0:
            max_candidates = min(max_candidates, self.max_candidates_max)
        return {
            "route_tls_sequence": route_seq,
            "next_tls_order": next_order,
            "next_tls_distance_m": next_dist,
            "current_tls": current_tls,
            "lookahead_hops": max(0, _as_int(c.get("lookahead_hops", 0), 0)),
            "max_candidates": max_candidates,
        }

    def candidate_allowed(self, node_id: str, context: Dict[str, Any]) -> bool:
        if not self.context_gate_enabled:
            return True
        route_seq = self._route_sequence(context)
        cur_tls = str(context.get("current_tls", "") or "").strip()
        if not route_seq or not cur_tls:
            return bool(self.allow_fallback_without_route)
        try:
            idx = route_seq.index(cur_tls)
        except ValueError:
            return bool(self.allow_fallback_without_route)
        lookahead = max(0, _as_int(context.get("lookahead_hops", 0), 0))
        lo = max(0, idx - int(self.context_back_hops))
        hi = min(len(route_seq) - 1, idx + lookahead)
        allowed = set(route_seq[lo : hi + 1])
        return str(node_id or "") in allowed

    def candidate_rank_key(self, node_id: str, context: Dict[str, Any]) -> Tuple[float, float, str]:
        route_seq = self._route_sequence(context)
        cur_tls = str(context.get("current_tls", "") or "").strip()
        dist_map = _as_float_map(context.get("next_tls_distance_m", {}))

        route_rank = self.route_unknown_penalty
        if route_seq and cur_tls and node_id in route_seq:
            try:
                idx_cur = route_seq.index(cur_tls)
                idx_peer = route_seq.index(node_id)
                if idx_peer >= idx_cur:
                    route_rank = float(idx_peer - idx_cur)
                else:
                    if self.prefer_forward_only:
                        route_rank = self.route_unknown_penalty
                    else:
                        route_rank = float((idx_cur - idx_peer) + self.route_backward_penalty)
            except ValueError:
                route_rank = self.route_unknown_penalty
        dist_rank = float(dist_map.get(str(node_id), self.distance_unknown_penalty))
        return (
            float(route_rank) * float(self.weight_route_order),
            float(dist_rank) * float(self.weight_proximity_m),
            str(node_id),
        )


def build_peer_selection_policy(name: str, cfg: Dict[str, Any]) -> PeerSelectionPolicy:
    n = str(name or "default").strip().lower()
    if n in {"transport_ev_corridor", "transport", "route_proximity"}:
        return TransportEVCorridorPolicy(cfg)
    return DefaultPeerSelectionPolicy()
