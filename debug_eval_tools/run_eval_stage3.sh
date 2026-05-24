#!/bin/bash
# Stage3 graph refinement evaluation runner.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VEHICLE="es6"
DATA_ROOT="${SCRIPT_DIR}/../data"
DEBUG_ROOT="${SCRIPT_DIR}/../data/debug"
STAGE3_DIR=""
OUTPUT_DIR=""
PASS_ARGS=()

load_run_config() {
    local config_path="${SCRIPT_DIR}/../conf/current_vehicle.yaml"
    if [[ ! -f "${config_path}" ]]; then
        return
    fi
    local parsed_vehicle
    local parsed_data_root
    local parsed_debug_root
    parsed_vehicle=$(grep -E '^[[:space:]]*active_vehicle:' "${config_path}" | head -n1 | sed -E 's/.*active_vehicle:[[:space:]]*"?([^"#]+)"?.*/\1/' | tr -d "'" | tr -d '"' || true)
    parsed_data_root=$(grep -E '^[[:space:]]*data_root:' "${config_path}" | head -n1 | sed -E 's/.*data_root:[[:space:]]*"?([^"#]+)"?.*/\1/' | tr -d "'" | tr -d '"' || true)
    parsed_debug_root=$(grep -E '^[[:space:]]*debug_root:' "${config_path}" | head -n1 | sed -E 's/.*debug_root:[[:space:]]*"?([^"#]+)"?.*/\1/' | tr -d "'" | tr -d '"' || true)
    VEHICLE="${parsed_vehicle:-${VEHICLE}}"
    DATA_ROOT="${parsed_data_root:-${DATA_ROOT}}"
    DEBUG_ROOT="${parsed_debug_root:-${DEBUG_ROOT}}"
    VEHICLE="${VEHICLE:-es6}"
    DATA_ROOT="${DATA_ROOT:-${SCRIPT_DIR}/../data}"
    DEBUG_ROOT="${DEBUG_ROOT:-${SCRIPT_DIR}/../data/debug}"
}

load_run_config

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stage3_dir)
            if [[ $# -lt 2 ]]; then
                echo "Error: --stage3_dir requires a path"
                exit 1
            fi
            STAGE3_DIR="$2"
            shift 2
            ;;
        --output_dir)
            if [[ $# -lt 2 ]]; then
                echo "Error: --output_dir requires a path"
                exit 1
            fi
            OUTPUT_DIR="$2"
            shift 2
            ;;
        *)
            PASS_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ -z "${STAGE3_DIR}" || -z "${OUTPUT_DIR}" ]]; then
    VEHICLE_NAME="${VEHICLE:-es6}"
    DEFAULT_STAGE3_DIR="${DATA_ROOT}/${VEHICLE_NAME}/stage3_graph_refine"
    DEFAULT_OUTPUT_DIR="${DEBUG_ROOT}/stage3_refine/${VEHICLE_NAME}/stage3"
    STAGE3_DIR="${STAGE3_DIR:-${DEFAULT_STAGE3_DIR}}"
    OUTPUT_DIR="${OUTPUT_DIR:-${DEFAULT_OUTPUT_DIR}}"
fi

echo ""
echo "=============================================="
echo "  Stage3 Graph Refinement Evaluation"
echo "=============================================="
echo ""
echo "Stage3 dir: ${STAGE3_DIR}"
echo "Output dir: ${OUTPUT_DIR}"
echo ""

python3 "${SCRIPT_DIR}/eval_stage3_refine.py" \
    --stage3_dir "${STAGE3_DIR}" \
    --output_dir "${OUTPUT_DIR}" \
    "${PASS_ARGS[@]}"
