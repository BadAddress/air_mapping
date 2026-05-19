#!/bin/bash
# Stage2 alignment evaluation runner.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAGE2_DIR="${SCRIPT_DIR}/../data/stage2_graph_opt"
OUTPUT_DIR="${SCRIPT_DIR}/eval_results/stage2_alignment"
PASS_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stage2_dir)
            if [[ $# -lt 2 ]]; then
                echo "Error: --stage2_dir requires a path"
                exit 1
            fi
            STAGE2_DIR="$2"
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

echo ""
echo "=============================================="
echo "  Stage2 Alignment Evaluation"
echo "=============================================="
echo ""
echo "Stage2 dir: ${STAGE2_DIR}"
echo "Output dir: ${OUTPUT_DIR}"
echo ""

python3 "${SCRIPT_DIR}/eval_stage2_alignment.py" \
    --stage2_dir "${STAGE2_DIR}" \
    --output_dir "${OUTPUT_DIR}" \
    "${PASS_ARGS[@]}"

