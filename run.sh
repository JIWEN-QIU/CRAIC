#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

orbit_laps="${1:-1}"
if ! [[ "$orbit_laps" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [orbit_laps_positive_integer]" >&2
  exit 1
fi

source "$script_dir/devel/setup.bash"

export DISABLE_ROS1_EOL_WARNINGS=1

roslaunch odin_ros_driver odin1_ros1.launch & sleep 2

roslaunch mavros px4.launch & sleep 2

roslaunch craic_odin_px4_adapter odin_to_mavros_pose.launch & sleep 2

roslaunch px4ctrl run_ctrl.launch & sleep 2

roslaunch super_planner exploration.launch & sleep 2

roslaunch craic_odin_px4_adapter craic2026_hold2s_mission_runner.launch \
  dry_run:=false \
  start_paused:=true \
  drop_enabled:=true \
  magnet_actuator_slots:="[1, 2, 3]" \
  magnet_release_sequence_slots:="[1, 3, 2]" \
  orbit_laps:="$orbit_laps" & sleep 2

roslaunch super_planner real_viz.launch 
