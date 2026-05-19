#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PORT="${1:-12322}"
CONFIG_FILE="${MODULE_ROOT}/conf/air_mapping_viz.yaml"
DOC_ROOT="${MODULE_ROOT}/viz/frontend"
BINARY="/opt/apollo/neo/bin/air_mapping_viz"

if [ ! -f "${BINARY}" ]; then
  echo "Error: binary not found: ${BINARY}"
  echo "Build target: //modules/air_mapping/viz:air_mapping_viz"
  exit 1
fi

echo "Air Mapping Viz"
echo "  URL:         http://localhost:${PORT}"
echo "  Module root: ${MODULE_ROOT}"
echo "  Doc root:    ${DOC_ROOT}"
echo "  Config:      ${CONFIG_FILE}"

export GLOG_logtostderr=1
exec "${BINARY}" \
  --port="${PORT}" \
  --module_root="${MODULE_ROOT}" \
  --doc_root="${DOC_ROOT}" \
  --config="${CONFIG_FILE}"
