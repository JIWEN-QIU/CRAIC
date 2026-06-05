#!/usr/bin/env bash
set -euo pipefail

WS="${CRAIC_WS:-/home/n150/CRAIC_ws}"
SETUP="${WS}/devel/setup.bash"
FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
ODOM_TOPIC="${ODOM_TOPIC:-/odin1/odometry_highfreq}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/odin1/cloud_slam}"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--no-rviz]

Environment overrides:
  CRAIC_WS=/home/n150/CRAIC_ws
  FCU_URL=/dev/ttyACM0:921600
  ODOM_TOPIC=/odin1/odometry_highfreq
  CLOUD_TOPIC=/odin1/cloud_slam
EOF
}

START_RVIZ=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      START_RVIZ=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${SETUP}" ]]; then
  echo "Cannot find ${SETUP}. Run catkin_make first or set CRAIC_WS." >&2
  exit 1
fi

if command -v gnome-terminal >/dev/null 2>&1; then
  open_term() {
    local title="$1"
    local cmd="$2"
    gnome-terminal --title="${title}" -- bash -lc "${cmd}; echo; echo '[${title}] exited. Press Enter to close.'; read"
  }
elif command -v x-terminal-emulator >/dev/null 2>&1; then
  open_term() {
    local title="$1"
    local cmd="$2"
    x-terminal-emulator -T "${title}" -e bash -lc "${cmd}; echo; echo '[${title}] exited. Press Enter to close.'; read"
  }
else
  echo "No supported terminal emulator found. Install gnome-terminal or x-terminal-emulator." >&2
  exit 1
fi

ros_cmd() {
  printf 'cd %q && source %q && %s' "${WS}" "${SETUP}" "$1"
}

echo "Starting CRAIC real-flight SUPER stack"
echo "  workspace: ${WS}"
echo "  fcu_url:   ${FCU_URL}"
echo "  odom:      ${ODOM_TOPIC}"
echo "  cloud:     ${CLOUD_TOPIC}"

open_term "01_odin" "$(ros_cmd 'roslaunch odin_ros_driver odin1_ros1.launch')"
sleep 3

open_term "02_mavros" "$(ros_cmd "roslaunch mavros px4.launch fcu_url:=${FCU_URL}")"
sleep 3

open_term "03_odin_bridge" "$(ros_cmd "roslaunch craic_odin_px4_adapter odin_to_mavros_pose.launch input_odom_topic:=${ODOM_TOPIC}")"
sleep 1

open_term "04_px4ctrl" "$(ros_cmd 'roslaunch px4ctrl run_ctrl.launch')"
sleep 1

open_term "05_super" "$(ros_cmd "roslaunch super_planner exploration.launch odom_topic:=${ODOM_TOPIC} cloud_topic:=${CLOUD_TOPIC}")"
sleep 1

if [[ "${START_RVIZ}" -eq 1 ]]; then
  open_term "06_rviz" "$(ros_cmd 'roslaunch super_planner real_viz.launch')"
fi

echo "All launch terminals have been opened."
