# CRAIC2026 Drop-Enabled Odom-Relative Y-Flip Mission

This is the `odom_relative_yflip` version of the CRAIC2026 fixed-route mission used by the current `run.sh` baseline.

The current baseline enables block dropping through the mission runner. The final landing task can trigger px4ctrl's built-in automatic landing after reaching the selected landing approach point.

## Auto Takeoff

The mission runner can call px4ctrl's built-in auto-takeoff before publishing mission setpoints.

It publishes `quadrotor_msgs/TakeoffLand` with `TAKEOFF = 1` to:

```text
/px4ctrl/takeoff_land
```

This is controlled by runner parameters:

```text
auto_takeoff: true
auto_takeoff_height: 1.0
auto_takeoff_timeout: 35.0
post_takeoff_climb_enabled: true
post_takeoff_climb_point: [0.0, 0.0, 1.35]
post_takeoff_climb_timeout: 15.0
takeoff_land_topic: /px4ctrl/takeoff_land
```

The default `auto_takeoff_height` matches the current px4ctrl config value in `ctrl_param_fpv.yaml`. After px4ctrl reaches that auto-takeoff height, the runner first publishes a direct `[0.0, 0.0, 1.35]` climb point and waits for the real z check to pass. It then starts the YAML waypoints through the normal mission flow.

Important: px4ctrl rejects AUTO_TAKEOFF if recent `/setpoints_cmd` commands are already being published. The runner therefore does not publish mission `PositionCommand` messages until auto-takeoff has completed.

## SUPER Goal Sequencing

The runner defaults to `output_mode: super_goal`. In this mode it does not publish mission `PositionCommand` messages to `/setpoints_cmd`. Instead, it publishes each waypoint once as a `geometry_msgs/PoseStamped` goal to:

```text
/move_base_simple/goal
```

This matches the current manual RViz workflow: SUPER receives the goal, plans with obstacle avoidance, and SUPER/px4ctrl handle the actual `/setpoints_cmd` stream.

Default runner parameters:

```text
output_mode: super_goal
goal_topic: /move_base_simple/goal
check_z_reached: false
```

With the current SUPER config, `fsm/click_height` is fixed to `1.35`, so normal `/move_base_simple/goal` travel goals are planned at 1.35 m. For that reason, the runner's default arrival check uses XY distance only during SUPER-guided travel.

## Mission 3 Image Scan

`mission_03_scan_targets.yaml` now uses a per-waypoint scan sequence for all four image targets:

- Fly to the image target XY through SUPER at the 1.35 m click height.
- Descend in place to 0.42 m using direct `/setpoints_cmd` publishing from the mission runner.
- Start `roslaunch uav_4class_infer uav_4class.launch`.
- Wait up to 10 s for `/target_info` to match the required drop target class.
- If the result matches, descend to 0.14 m and hold at the drop height for at least 2 s before releasing a magnet.
- If no target class matches in 10 s, climb back to 1.35 m and continue to the next image target.
- Fallback drops also follow the 0.42 m recognition -> 0.14 m release route: drop at the third point when total releases are 0, and at the fourth point when total releases are 1.
- Once mission 3 reaches two total releases, it completes early after climbing back to 1.35 m and enters mission 4.
- Stop the launched inference process.
- Climb back to 1.35 m using direct `/setpoints_cmd`.
- Continue to the next image target.

If two releases are not completed earlier, mission 3 completes after the fourth image target finishes the same descend/infer/ascend sequence. It no longer holds 2 s at `image_bottom_left`.

This scan phase is a deliberate exception to the normal SUPER-only goal flow, because fixed `fsm/click_height=1.35` would otherwise prevent low-altitude detect/drop commands from taking effect through `/move_base_simple/goal`.

## Completion Policy

This version uses `final_waypoint_hold` as the normal task completion condition, with a landing-task exception:

- Each task follows its waypoints in order.
- Intermediate waypoints switch immediately after entering the accept radius.
- For tasks 1-5, only the final waypoint holds for 2 seconds.
- `mission_03_scan_targets.yaml` is an exception: each image waypoint performs descend/infer/ascend, and the task completes after the fourth point without a 2 s final hold.
- After the final waypoint has been held for 2 seconds, the task is complete.
- For `mission_06_land.yaml`, the selected landing approach waypoint has `hold_time: 0.0`; on arrival, the runner publishes px4ctrl `LAND = 2` immediately.
- `action` fields are for logging only and do not affect completion.
- The mission does not wait for QR, image classification, special target, or ring detection results.

Final waypoints:

- `mission_01_takeoff_qr.yaml`: `qr_observe`
- `mission_02_orbit.yaml`: `orbit_exit_right`
- `mission_03_scan_targets.yaml`: `image_bottom_left`
- `mission_04_special.yaml`: `special_target_observe`
- `mission_05_ring.yaml`: `ring_search_end`
- `mission_06_land.yaml`: `landing_left_approach` or `landing_right_approach`, then px4ctrl LAND

## Auto Landing

At the selected landing approach point, the runner can call px4ctrl's built-in automatic landing.

It publishes `quadrotor_msgs/TakeoffLand` with `LAND = 2` to:

```text
/px4ctrl/takeoff_land
```

Default runner parameters:

```text
auto_land: true
land_command_duration: 5.0
takeoff_land_topic: /px4ctrl/takeoff_land
```

The runner does not auto-disarm. px4ctrl must accept the LAND command, which normally requires it to be back in AUTO_HOVER rather than actively consuming command setpoints. If SUPER is still publishing `/setpoints_cmd` at the landing approach point, px4ctrl may reject LAND until command input stops or times out.

## Coordinate Rule

The waypoints in this directory have already been converted from `field_map` coordinates into `odom_relative` coordinates.

Conversion formula:

```text
odom_x = field_x - 1.5
odom_y = -(field_y - 3.0)
odom_z = field_z
```

This formula is based on the verified real-vehicle relationship:

- odom x 与比赛场地图 field_map x 同向
- odom y 与比赛场地图 field_map y 反向
- odom z 与高度方向同向
- 起飞点为 odom 近似原点

## Safety Checks Before Flight

Before executing these waypoints, check:

- 起飞点处 `/odin/odom_for_px4ctrl` 的 x/y 接近 0
- 手动移动到二维码方向时 odom x 增加
- 手动移动到比赛场地图 +y 方向时 odom y 减小
- 第一次先测试 `takeoff_climb -> qr_observe` 短路径，不要直接跑完整路线

This YAML set depends on repeatable takeoff placement and on the odom origin being near the takeoff point. If the takeoff position changes, Odin initialization changes, or the odom origin/direction changes, regenerate and revalidate this directory.

## Landing Y Flip

Because odom y is opposite to field_map y:

- `landing_left` field_map `[1.5, 1.4]` becomes odom_relative `[0.0, 1.6]`
- `landing_right` field_map `[1.5, 4.6]` becomes odom_relative `[0.0, -1.6]`

The mission triggers px4ctrl LAND at the selected landing-point approach position. The runner does not auto-disarm.
