#!/usr/bin/env bash
set -euo pipefail

WS="${CRAIC_WS:-/home/n150/CRAIC_ws}"
SETUP="${WS}/devel/setup.bash"
BAG_DIR="${BAG_DIR:-${WS}/log/super_tests}"
STAMP="$(date +%Y%m%d_%H%M%S)"
PREFIX="${BAG_PREFIX:-super_real_${STAMP}}"

if [[ ! -f "${SETUP}" ]]; then
  echo "Cannot find ${SETUP}. Run catkin_make first or set CRAIC_WS." >&2
  exit 1
fi

mkdir -p "${BAG_DIR}"

cd "${WS}"
source "${SETUP}"

echo "Recording SUPER real-flight test bag"
echo "  output: ${BAG_DIR}/${PREFIX}_*.bag"
echo "  stop:   Ctrl-C"

exec rosbag record \
  --split \
  --duration=5m \
  -O "${BAG_DIR}/${PREFIX}" \
  /mavros/state \
  /mavros/local_position/odom \
  /odin1/odometry_highfreq \
  /odin1/cloud_slam \
  /move_base_simple/goal \
  /setpoints_cmd \
  /planning_cmd/poly_traj \
  /fsm_node/fsm/current_goal \
  /fsm_node/fsm/current_goalpath \
  /fsm_node/fsm/path \
  /fsm_node/visualization/goal \
  /fsm_node/visualization/astar \
  /fsm_node/visualization/exp_traj \
  /fsm_node/visualization/exp_sfcs \
  /fsm_node/visualization/receding_traj \
  /fsm_node/rog_map/occ \
  /fsm_node/rog_map/inf_occ \
  /fsm_node/rog_map/unk \
  /fsm_node/rog_map/inf_unk
