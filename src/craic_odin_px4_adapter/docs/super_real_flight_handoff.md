# SUPER Real-Flight Handoff

Date: 2026-05-19

This note records the current real-flight state for the Odin + SUPER + px4ctrl
stack. The system is already usable for controlled indoor/local obstacle
avoidance tests: take off manually, switch px4ctrl to `AUTO_HOVER`, then send a
goal from RViz `2D Nav Goal`. SUPER plans from Odin odometry and point cloud,
publishes trajectory commands to px4ctrl, and the vehicle follows the planned
local trajectory.

## Current Launch Flow

Recommended one-command startup:

```bash
cd ~/CRAIC_ws
source devel/setup.bash
./src/craic_odin_px4_adapter/scripts/start_super_real_flight.sh
```

The script starts:

```text
roslaunch odin_ros_driver odin1_ros1.launch
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
roslaunch craic_odin_px4_adapter odin_to_mavros_pose.launch
roslaunch px4ctrl run_ctrl.launch
roslaunch super_planner exploration.launch
roslaunch super_planner real_viz.launch
```

Runtime topic defaults:

```text
Odin odom:       /odin1/odometry_highfreq
Odin cloud:      /odin1/cloud_slam
RViz goal:       /move_base_simple/goal
SUPER command:   /setpoints_cmd
Poly trajectory: /planning_cmd/poly_traj
```

## Flight Procedure

1. Start the stack and wait until Odin odometry and point cloud are stable.
2. Take off manually to about 1 m.
3. Switch px4ctrl to `AUTO_HOVER`.
4. Confirm RViz shows the robot, Odin cloud, ROG-Map, and SUPER visualization.
5. Send a short RViz `2D Nav Goal`, initially 2-4 m away.
6. Keep manual takeover ready during the whole test.

Use soft, non-sharp obstacles. Start with a single obstacle in front of the
vehicle, then expand to more complex layouts only after several clean runs.

## Active Real-Flight Config

SUPER entry point:

```text
src/super/super_planner/launch/exploration.launch
```

Main config files:

```text
src/super/super_planner/config/exp_advanced_params.yaml
src/super/super_planner/config/exp.yaml
```

`exploration.launch` loads `exp_advanced_params.yaml` first and `exp.yaml`
second. When a parameter appears in both files, the value in `exp.yaml` is the
effective real-flight value.

Current important values:

```yaml
fsm/click_goal_topic: "/move_base_simple/goal"
fsm/click_height: -999
super_planner/frontend_in_known_free: false
super_planner/robot_r: 0.30
super_planner/yaw_mode: 1
super_planner/planning_horizon: 3.0
super_planner/receding_dis: 1.0
traj_opt/boundary/max_vel: 0.8
traj_opt/boundary/max_acc: 1.5
rog_map/resolution: 0.1
rog_map/inflation_resolution: 0.1
rog_map/inflation_step: 3
rog_map/virtual_ceil_height: 3.0
rog_map/raycasting/ray_range: [0.5, 30]
rog_map/raycasting/blind_filter_en: false
```

Notes:

- `click_height: -999` disables fixed click height, so RViz 2D goals use the
  current odometry z.
- `robot_r: 0.30` matches the current guarded quadrotor size, about 60 cm
  diagonal.
- `inflation_resolution: 0.1` and `inflation_step: 3` produce about 0.3 m
  occupied inflation for A* frontend planning.

## Verified State

- Odin odometry and point cloud are connected to SUPER/ROG-Map.
- RViz `2D Nav Goal` triggers SUPER planning.
- px4ctrl receives SUPER output and drives the vehicle in real flight.
- Real-flight goal height is fixed by using current odom z instead of a fixed
  clicked z.
- Near-body point contamination no longer blocks the start area in the tested
  setup.
- With the current inflation settings, SUPER avoids a front obstacle in real
  flight instead of stopping at corridor generation.

## Previous Failure Signature

The earlier "flies in free space, does not move with obstacle" case was not a
PX4/MAVROS/px4ctrl command-chain failure. SUPER was failing to generate a safe
corridor near the obstacle:

```text
CIRI line decomposition failed
GeneratePolytopeFromLine failed
GenerateExpTrajectory failed
```

The practical cause was that the A* frontend path passed too close to occupied
cells, so corridor decomposition contained too many obstacle points. Increasing
ROG-Map occupied inflation to about 0.3 m made the frontend path consistent with
the real vehicle radius and resolved the tested case.

## Next Test Matrix

Run several short, repeatable tests before increasing speed or range.

| Test | Goal distance | Obstacle | Expected result |
| --- | ---: | --- | --- |
| Free-space baseline | 2 m | none | Straight, smooth motion |
| Free-space repeat | 4 m | none | Stable replan and stop near goal |
| Single front obstacle | 2-3 m | soft obstacle centered | Side detour, no stop |
| Offset obstacle | 3-4 m | soft obstacle left/right | Chooses open side |
| Narrower passage | 2-3 m | two soft obstacles | Only test after clean single-obstacle runs |

For each run, record:

```text
date/time
battery voltage
takeoff height
goal distance and direction
obstacle size and distance
max visible speed
minimum obstacle clearance by visual estimate
manual takeover needed: yes/no
notable log messages
```

## Bag Recording

Start a bag before sending RViz goals:

```bash
cd ~/CRAIC_ws
source devel/setup.bash
./src/craic_odin_px4_adapter/scripts/record_super_test_bag.sh
```

Stop it with `Ctrl-C` after landing or after the specific test run. The bag is
written under:

```text
~/CRAIC_ws/log/super_tests/
```

## Clockwise Obstacle Orbit

For a 0.6 m x 0.6 m obstacle, use the helper node to send RViz-style goals
around the obstacle while SUPER performs local avoidance. By default it assumes
the obstacle center is 2.5 m straight ahead of the vehicle's current yaw:

```bash
cd ~/CRAIC_ws
source devel/setup.bash
roslaunch craic_odin_px4_adapter orbit_obstacle_goals.launch
```

The node first sends the vehicle to the front waypoint, 0.5 m from the obstacle
surface. It then sends the remaining waypoints to `/move_base_simple/goal` in
this order:

```text
front -> left -> back -> right -> front
```

The default waypoint radius is:

```text
0.60 / 2 + 0.50 = 0.80 m from obstacle center
```

So the route leaves about 0.5 m nominal clearance from each obstacle face before
SUPER's own local replanning and map inflation are applied.

Optional fixed-center launch:

```bash
roslaunch craic_odin_px4_adapter orbit_obstacle_goals.launch obstacle_x:=2.0 obstacle_y:=0.0
```

Optional RViz clicked-center launch:

```bash
roslaunch craic_odin_px4_adapter orbit_obstacle_goals.launch use_clicked_center:=true
```

Useful tuning args:

```text
obstacle_distance:=2.50
clearance:=0.50
xy_tolerance:=0.35
hold_time:=1.5
segment_timeout:=45.0
republish_period:=1.0
```

## Safety Gates Before Parameter Increase

Keep the current conservative parameters until all of these are true:

- At least three clean 2-4 m obstacle-avoidance runs.
- No `GeneratePolytopeFromLine failed` or repeated trajectory generation
  failures during goal execution.
- Actual trajectory visually follows the planned trajectory in RViz.
- Yaw behavior is acceptable and does not create sudden large attitude changes.
- Manual takeover has not been needed for normal obstacle layouts.

After that, increase only one dimension at a time: either goal distance,
velocity, acceleration, or obstacle complexity.
