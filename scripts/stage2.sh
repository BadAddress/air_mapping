#!/bin/bash
# ============================================================
# stage2_graph_opt startup script
# Usage:
#   ./scripts/stage2.sh
#   ./scripts/stage2.sh /path/to/config.yaml
# ============================================================

BINARY="/opt/apollo/neo/bin/stage2_graph_opt"
DEFAULT_CONFIG="/apollo_workspace/modules/air_mapping/conf/stage2_graph_opt.yaml"

CONFIG="${1:-$DEFAULT_CONFIG}"

if [ ! -f "$BINARY" ]; then
    echo "[ERROR] Binary not found: $BINARY"
    echo "        Please build first: buildtool build -p modules/air_mapping"
    exit 1
fi

if [ ! -f "$CONFIG" ]; then
    echo "[ERROR] Config not found: $CONFIG"
    exit 1
fi

echo "============================================"
echo " stage2_graph_opt"
echo " config: $CONFIG"
echo " binary: $BINARY"
echo "============================================"

exec "$BINARY" --config="$CONFIG"
