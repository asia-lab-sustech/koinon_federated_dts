ROOT="/home/christian/projects/fdt_project/fdt_prototype/sample_world/real_map/paper_experiments"
PY="/home/christian/projects/fdt_venv/bin/python3"   # or python3
LOG="$ROOT/tmp/fed_core_$(date +%Y%m%d_%H%M%S)"
SESSION="fedcore"

mkdir -p "$LOG"

tmux new-session -d -s "$SESSION" -c "$ROOT"

# Pane 0: membership
tmux send-keys -t "$SESSION:0.0" \
"mkdir -p '$LOG'; $PY '$ROOT/federation_membership_service.py' --mqtt-host localhost --mqtt-port 1883 --heartbeat-mode monitor --log-jsonl '$LOG/membership.jsonl'" C-m

# Pane 1: catalog
tmux split-window -h -t "$SESSION:0.0" -c "$ROOT"
tmux send-keys -t "$SESSION:0.1" \
"$PY '$ROOT/federation_catalog_service.py' --mqtt-host localhost --mqtt-port 1883 --log-jsonl '$LOG/catalog.jsonl'" C-m

# Pane 2: discovery
tmux split-window -v -t "$SESSION:0.0" -c "$ROOT"
tmux send-keys -t "$SESSION:0.2" \
"$PY '$ROOT/federation_discovery_service.py' --mqtt-host localhost --mqtt-port 1883 --log-jsonl '$LOG/discovery.jsonl'" C-m

# Pane 3: lifecycle
tmux split-window -v -t "$SESSION:0.1" -c "$ROOT"
tmux send-keys -t "$SESSION:0.3" \
"$PY '$ROOT/federation_lifecycle_health_service.py' --mqtt-host localhost --mqtt-port 1883 --log-jsonl '$LOG/lifecycle.jsonl'" C-m

tmux select-layout -t "$SESSION" tiled
tmux attach -t "$SESSION"
