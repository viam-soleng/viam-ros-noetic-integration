#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

#
# setup environment - will modify here as needed
#
source /opt/ros/noetic/setup.bash
ROS_MASTER=http://localhost:11311

if [[ ! -f ${SCRIPT_DIR}/venv/bin/python ]]; then
  echo "setting up python environment"
  python3 -m venv "${SCRIPT_DIR}"/venv --system-site-packages
  "${SCRIPT_DIR}"/venv/bin/python -m pip install -r "${SCRIPT_DIR}"/requirements.txt
fi

exec "${SCRIPT_DIR}"/venv/bin/python "${SCRIPT_DIR}"/main.py "$@"