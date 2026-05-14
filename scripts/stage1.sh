#!/bin/bash
# ============================================================
# stage1_lio 启动脚本
# 用法:
#   ./scripts/stage1.sh                     # 使用默认配置
#   ./scripts/stage1.sh /path/to/config.yaml # 指定配置文件
# ============================================================

BINARY="/opt/apollo/neo/bin/stage1_lio"
DEFAULT_CONFIG="/apollo_workspace/modules/air_mapping/conf/stage1_lio.yaml"

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
echo " stage1_lio"
echo " config: $CONFIG"
echo " binary: $BINARY"
echo "============================================"

exec "$BINARY" --config="$CONFIG"
