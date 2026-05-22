#!/bin/bash
# ============================================================
# stage2_graph_opt startup script
# Usage:
#   ./scripts/stage2.sh
# ============================================================

BINARY="/opt/apollo/neo/bin/stage2_graph_opt"
DEFAULT_CONFIG="/apollo_workspace/modules/air_mapping/conf/current_vehicle.yaml"

if [ ! -f "$BINARY" ]; then
    echo "[ERROR] Binary not found: $BINARY"
    echo "        Please build first: buildtool build -p modules/air_mapping"
    exit 1
fi

if [ ! -f "$DEFAULT_CONFIG" ]; then
    echo "[ERROR] Top-level config not found: $DEFAULT_CONFIG"
    exit 1
fi

echo "============================================"
echo " stage2_graph_opt"
echo " config: $DEFAULT_CONFIG"
echo " binary: $BINARY"
echo "============================================"

exec "$BINARY"
