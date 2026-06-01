#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
  magnet_release_sequence_slots:="[1, 3, 2]" & sleep 2

roslaunch super_planner real_viz.launch 
