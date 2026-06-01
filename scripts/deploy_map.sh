#!/bin/bash
# ============================================================
# Deploy stage4 final map artifacts to air_localization.
# Usage:
#   ./scripts/deploy_map.sh --es6
#   ./scripts/deploy_map.sh --minibus
# ============================================================

usage() {
    echo "Usage: $0 --es6|--minibus"
}

if [ "$#" -ne 1 ]; then
    echo "[ERROR] Expected exactly one vehicle option."
    usage
    exit 1
fi

case "$1" in
    --es6)
        VEHICLE="es6"
        ;;
    --minibus)
        VEHICLE="minibus"
        ;;
    *)
        echo "[ERROR] Unknown vehicle option: $1"
        usage
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${MODULE_ROOT}" || {
    echo "[ERROR] Failed to enter air_mapping module root: ${MODULE_ROOT}"
    exit 1
}

SRC_DIR="data/${VEHICLE}/stage4_map_export"
DST_DIR="../air_localization/data/new_map"

SRC_PCD="${SRC_DIR}/global.pcd"
SRC_ALIGNMENT="${SRC_DIR}/utm_alignment.txt"
DST_PCD="${DST_DIR}/global.pcd"
DST_ALIGNMENT="${DST_DIR}/utm_alignment.txt"

if [ ! -d "${SRC_DIR}" ]; then
    echo "[ERROR] Stage4 output directory not found: ${SRC_DIR}"
    echo "        Run stage4 export for vehicle '${VEHICLE}' before deploying."
    exit 1
fi

if [ ! -f "${SRC_PCD}" ]; then
    echo "[ERROR] Missing stage4 map file: ${SRC_PCD}"
    exit 1
fi

if [ ! -f "${SRC_ALIGNMENT}" ]; then
    echo "[ERROR] Missing stage4 alignment file: ${SRC_ALIGNMENT}"
    exit 1
fi

if [ ! -d "../air_localization" ]; then
    echo "[ERROR] air_localization module not found at: ../air_localization"
    exit 1
fi

if ! mkdir -p "${DST_DIR}"; then
    echo "[ERROR] Failed to create deployment directory: ${DST_DIR}"
    exit 1
fi

if ! cp -f "${SRC_PCD}" "${DST_PCD}"; then
    echo "[ERROR] Failed to deploy map file to: ${DST_PCD}"
    exit 1
fi

if ! cp -f "${SRC_ALIGNMENT}" "${DST_ALIGNMENT}"; then
    echo "[ERROR] Failed to deploy alignment file to: ${DST_ALIGNMENT}"
    exit 1
fi

echo "============================================"
echo " deploy_map"
echo " vehicle: ${VEHICLE}"
echo " source:  ${SRC_DIR}"
echo " target:  ${DST_DIR}"
echo " files:   global.pcd, utm_alignment.txt"
echo "============================================"
echo "[OK] Stage4 map artifacts deployed."
