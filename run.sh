#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# must be set in module
source ${ROS_ENV}

# OVERLAYS to source comes from MODULE
for OL in $(OVERLAYS//;/); do
  source ${OL}
done

if [[ ! -f ${SCRIPT_DIR}/venv/bin/python ]]; then
  echo "setting up python environment"
  python3 -m venv "${SCRIPT_DIR}"/venv --system-site-packages
  "${SCRIPT_DIR}"/venv/bin/python -m pip install -r "${SCRIPT_DIR}"/requirements.txt
fi

# TODO: Error checking for empty string
RUST_UTILS_SO=$(find $SCRIPT_DIR -name libviam_rust_utils.so -printf '%h')

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${RUST_UTILS_SO}

exec "${SCRIPT_DIR}"/venv/bin/python "${SCRIPT_DIR}"/main.py "$@"