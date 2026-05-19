#!/bin/bash
# ============================================================
# stage3_graph_refine startup script
# Usage:
#   ./scripts/stage3.sh
#   ./scripts/stage3.sh /path/to/config.yaml
# ============================================================

BINARY="/opt/apollo/neo/bin/stage3_graph_refine"
DEFAULT_CONFIG="/apollo_workspace/modules/air_mapping/conf/stage3_graph_refine.yaml"

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
echo " stage3_graph_refine"
echo " config: $CONFIG"
echo " binary: $BINARY"
echo "============================================"

exec "$BINARY" --config="$CONFIG"
