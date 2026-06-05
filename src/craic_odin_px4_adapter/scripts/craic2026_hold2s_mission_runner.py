#!/usr/bin/env python3
import copy
import json
import math
import os
import signal
import subprocess
from pathlib import Path

import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Craic2026Hold2sMissionRunner:
    def __init__(self):
        default_dir = (
            "/home/n150/CRAIC_ws/src/craic_odin_px4_adapter/config/"
            "craic2026_mission_no_drop_odom_relative_yflip"
        )
        self.mission_dir = Path(rospy.get_param("~mission_dir", default_dir))
        self.mission_main = rospy.get_param("~mission_main", "mission_main.yaml")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.start_paused = bool(rospy.get_param("~start_paused", True))
        self.qr_landing_side = rospy.get_param("~qr_landing_side", "right")
        self.qr_result_topic = rospy.get_param("~qr_result_topic", "/qr_landing_side")
        self.qr_payload_topic = rospy.get_param("~qr_payload_topic", "/qr_data")
        self.qr_wait_timeout = float(rospy.get_param("~qr_wait_timeout", 10.0))
        self.qr_scan_z = float(rospy.get_param("~qr_scan_z", 0.58))
        self.qr_cruise_z = float(rospy.get_param("~qr_cruise_z", 1.35))
        self.qr_launch_package = rospy.get_param("~qr_launch_package", "two_stage_infer")
        self.qr_launch_file = rospy.get_param("~qr_launch_file", "two_stage.launch")
        self.max_task_id = int(rospy.get_param("~max_task_id", -1))
        self.max_waypoint_index = int(rospy.get_param("~max_waypoint_index", -1))
        self.auto_takeoff = bool(rospy.get_param("~auto_takeoff", True))
        self.auto_takeoff_height = float(rospy.get_param("~auto_takeoff_height", 1.0))
        self.auto_takeoff_timeout = float(rospy.get_param("~auto_takeoff_timeout", 35.0))
        self.takeoff_command_duration = float(rospy.get_param("~takeoff_command_duration", 1.0))
        self.post_takeoff_climb_enabled = bool(
            rospy.get_param("~post_takeoff_climb_enabled", True)
        )
        self.post_takeoff_climb_point = [
            float(rospy.get_param("~post_takeoff_climb_x", 0.0)),
            float(rospy.get_param("~post_takeoff_climb_y", 0.0)),
            float(rospy.get_param("~post_takeoff_climb_z", self.qr_cruise_z)),
        ]
        self.post_takeoff_climb_timeout = float(
            rospy.get_param("~post_takeoff_climb_timeout", 15.0)
        )
        self.auto_land = bool(rospy.get_param("~auto_land", True))
        self.land_command_duration = float(rospy.get_param("~land_command_duration", 5.0))
        self.scan_infer_enabled = bool(rospy.get_param("~scan_infer_enabled", True))
        self.scan_task_name = rospy.get_param("~scan_task_name", "scan_image_targets")
        self.scan_detect_z = float(
            rospy.get_param("~scan_detect_z", rospy.get_param("~scan_descent_z", 0.42))
        )
        self.scan_drop_z = float(rospy.get_param("~scan_drop_z", 0.09))
        self.scan_descent_z = self.scan_detect_z
        self.scan_cruise_z = float(rospy.get_param("~scan_cruise_z", 1.35))
        self.orbit_laps = int(rospy.get_param("~orbit_laps", 1))
        if self.orbit_laps < 1:
            raise ValueError("orbit_laps must be >= 1")
        self.scan_result_timeout = float(rospy.get_param("~scan_result_timeout", 10.0))
        self.scan_launch_package = rospy.get_param("~scan_launch_package", "two_stage_infer")
        self.scan_launch_file = rospy.get_param("~scan_launch_file", "two_stage.launch")
        self.scan_process_prestarted = bool(rospy.get_param("~scan_process_prestarted", False))
        self.scan_result_topic = rospy.get_param("~scan_result_topic", "/target_info")
        self.pre_scan_launch_task_name = rospy.get_param(
            "~pre_scan_launch_task_name", "orbit_obstacle"
        )
        self.pre_scan_launch_hold_time = float(rospy.get_param("~pre_scan_launch_hold_time", 6.0))
        self.drop_enabled = bool(rospy.get_param("~drop_enabled", False))
        self.drop_target_classes = self.parse_class_list(rospy.get_param("~drop_target_classes", ""))
        self.drop_stable_count = max(1, int(rospy.get_param("~drop_stable_count", 5)))
        self.drop_release_service = rospy.get_param(
            "~drop_release_service", "/electromagnet_control/release_next"
        )
        self.drop_wait_before_release = float(rospy.get_param("~drop_wait_before_release", 2.0))
        self.drop_wait_after_release = float(rospy.get_param("~drop_wait_after_release", 1.0))
        self.special_drop_task_name = rospy.get_param("~special_drop_task_name", "special_target")
        self.special_drop_descent_z = float(rospy.get_param("~special_drop_descent_z", 0.09))
        self.special_drop_cruise_z = float(rospy.get_param("~special_drop_cruise_z", 1.35))
        self.special_detect_z = float(rospy.get_param("~special_detect_z", 0.62))
        self.special_detector_enabled = bool(rospy.get_param("~special_detector_enabled", True))
        self.special_detector_launch_package = rospy.get_param(
            "~special_detector_launch_package", "special_target_detection_n150"
        )
        self.special_detector_launch_file = rospy.get_param(
            "~special_detector_launch_file", "special_target_detector.launch"
        )
        self.special_camera_topic = rospy.get_param("~special_camera_topic", "/usb_cam/image_raw")
        self.special_detector_enable_usb_cam = bool(
            rospy.get_param("~special_detector_enable_usb_cam", True)
        )
        self.special_result_topic = rospy.get_param("~special_result_topic", "/special_target/result")
        self.special_confirm_count = max(1, int(rospy.get_param("~special_confirm_count", 3)))
        self.special_detect_timeout = float(rospy.get_param("~special_detect_timeout", 10.0))
        self.special_timeout_release_fallback = bool(
            rospy.get_param("~special_timeout_release_fallback", True)
        )
        self.ring_detector_after_special_enabled = bool(
            rospy.get_param("~ring_detector_after_special_enabled", True)
        )
        self.ring_detector_turn_yaw = float(
            rospy.get_param("~ring_detector_turn_yaw", -math.pi / 2.0)
        )
        self.ring_detector_yaw_tolerance = float(
            rospy.get_param("~ring_detector_yaw_tolerance", 0.15)
        )
        self.ring_detector_turn_timeout = float(
            rospy.get_param("~ring_detector_turn_timeout", 8.0)
        )
        self.ring_detector_wait_time = float(
            rospy.get_param("~ring_detector_wait_time", 60.0)
        )
        self.ring_detector_launch_package = rospy.get_param(
            "~ring_detector_launch_package", "ring_detector"
        )
        self.ring_detector_launch_file = rospy.get_param(
            "~ring_detector_launch_file", "ring_detector.launch"
        )
        self.ring_detector_center_estimation_mode = rospy.get_param(
            "~ring_detector_center_estimation_mode", "ring_size"
        )
        self.ring_detector_size_shape_source = rospy.get_param(
            "~ring_detector_size_shape_source", "min_area_rect"
        )
        self.ring_detector_size_alpha_policy = rospy.get_param(
            "~ring_detector_size_alpha_policy", "max"
        )
        self.ring_detector_camera_xyz_offset_base = rospy.get_param(
            "~ring_detector_camera_xyz_offset_base", "[0.0, 0.04, 0.0]"
        )
        self.ring_detector_show_window = bool(
            rospy.get_param("~ring_detector_show_window", True)
        )
        self.ring_task_name = rospy.get_param("~ring_task_name", "ring_placeholder_search")
        self.ring_center_topic = rospy.get_param(
            "~ring_center_topic", "/ring_detector/center_odom_latched"
        )
        self.ring_center_max_age = float(rospy.get_param("~ring_center_max_age", 3.0))
        self.ring_approach_distance = float(rospy.get_param("~ring_approach_distance", 0.8))
        self.ring_exit_distance = float(rospy.get_param("~ring_exit_distance", 0.8))
        self.ring_min_z = float(rospy.get_param("~ring_min_z", 0.8))
        self.ring_max_z = float(rospy.get_param("~ring_max_z", 1.8))
        self.super_ring_radius_service = rospy.get_param(
            "~super_ring_radius_service", "/super_planner/set_ring_radius_mode"
        )

        self.odom_topic = rospy.get_param("~odom_topic", "/odin/odom_for_px4ctrl")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/setpoints_cmd")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.output_mode = rospy.get_param("~output_mode", "super_goal")
        self.check_z_reached = bool(rospy.get_param("~check_z_reached", False))
        self.takeoff_land_topic = rospy.get_param("~takeoff_land_topic", "/px4ctrl/takeoff_land")
        self.status_topic = rospy.get_param("~status_topic", "/craic2026/mission/status")

        self.accept_radius_xy = 0.15
        self.accept_radius_z = 0.10
        self.latest_odom = None
        self.latest_odom_time = None
        self.current_task_idx = 0
        self.current_waypoint_idx = 0
        self.final_hold_start = None
        self.last_target = None
        self.last_goal_publish_key = None
        self.last_yaw = 0.0
        self.state = "PAUSED" if self.start_paused else self.initial_active_state()
        self.start_time = rospy.Time.now()
        self.odom_error_reported = False
        self.takeoff_command_start = None
        self.takeoff_start_z = None
        self.takeoff_done = not self.auto_takeoff
        self.post_takeoff_climb_start = None
        self.post_takeoff_climb_done = not self.post_takeoff_climb_enabled
        self.land_command_start = None
        self.land_done = False
        self.scan_phase = None
        self.scan_wait_start = None
        self.scan_result_after_time = None
        self.scan_last_result = None
        self.scan_last_result_time = None
        self.scan_process = None
        self.scan_process_started_for_mission = False
        self.qr_phase = None
        self.qr_wait_start = None
        self.qr_payload_after_time = None
        self.qr_last_payload = None
        self.qr_last_payload_time = None
        self.qr_targets_from_payload = []
        self.qr_payload_confirmed = False
        self.qr_last_update_time = None
        self.qr_process = None
        self.drop_stable_class = None
        self.drop_stable_count_seen = 0
        self.drop_non_target_confirm_rounds = 0
        self.drop_processed_result_time = None
        self.scan_drop_ready_start = None
        self.scan_pending_drop_reason = None
        self.scan_pending_drop_class = None
        self.drop_wait_start = None
        self.dropped_target_classes = set()
        self.drop_release_count = 0
        self.special_drop_phase = None
        self.special_drop_ready_start = None
        self.special_drop_wait_start = None
        self.special_drop_done = False
        self.special_detect_wait_start = None
        self.special_result_after_time = None
        self.special_last_result = None
        self.special_last_result_time = None
        self.special_confirm_seen = 0
        self.special_detector_process = None
        self.special_detector_started = False
        self.special_ring_wait_start = None
        self.special_ring_turn_start = None
        self.ring_detector_process = None
        self.ring_detector_started = False
        self.ring_detector_start_time = None
        self.ring_detector_health_last_check = None
        self.ring_phase = None
        self.ring_targets = None
        self.ring_wait_point = None
        self.ring_center_odom = None
        self.ring_center_time = None
        self.super_ring_radius_enabled = False
        self.super_ring_radius_target = None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.takeoff_land_pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.drop_release_srv = None
        if self.drop_enabled:
            self.drop_release_srv = rospy.ServiceProxy(self.drop_release_service, Trigger)
        self.super_ring_radius_srv = rospy.ServiceProxy(
            self.super_ring_radius_service, SetBool
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_cb, queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber("/craic2026/mission/start", Bool, self.start_cb, queue_size=1)
        rospy.Subscriber("/craic2026/mission/pause", Bool, self.pause_cb, queue_size=1)
        rospy.Subscriber("/craic2026/mission/resume", Bool, self.resume_cb, queue_size=1)
        rospy.Subscriber("/craic2026/mission/abort", Bool, self.abort_cb, queue_size=1)
        rospy.Subscriber(self.scan_result_topic, String, self.scan_result_cb, queue_size=10)
        rospy.Subscriber(self.special_result_topic, String, self.special_result_cb, queue_size=10)
        rospy.Subscriber(self.qr_result_topic, String, self.qr_landing_side_cb, queue_size=10)
        rospy.Subscriber(self.qr_payload_topic, String, self.qr_payload_cb, queue_size=10)
        rospy.Subscriber(
            self.ring_center_topic, PointStamped, self.ring_center_cb, queue_size=1
        )
        rospy.on_shutdown(self.cleanup_processes)

        self.tasks = self.load_mission()
        self.validate_mission()
        self.print_mission()
        if not self.start_paused and not self.takeoff_done and self.has_qr_read_task():
            self.start_qr_process()

        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN enabled: no PositionCommand will be published.")
        if self.start_paused:
            rospy.logwarn("[craic2026_runner] start_paused=true: waiting for /craic2026/mission/start.")
        if self.auto_takeoff:
            rospy.logwarn(
                "[craic2026_runner] auto_takeoff=true: will publish TAKEOFF to %s before setpoints.",
                self.takeoff_land_topic,
            )
        if self.post_takeoff_climb_enabled:
            rospy.logwarn(
                "[craic2026_runner] post_takeoff_climb=true: after takeoff, hold direct point %s before first mission goal.",
                self.post_takeoff_climb_point,
            )
        if self.auto_land:
            rospy.logwarn(
                "[craic2026_runner] auto_land=true: landing task arrival will publish LAND to %s.",
                self.takeoff_land_topic,
            )
        if self.scan_infer_enabled:
            rospy.logwarn(
                "[craic2026_runner] scan infer enabled: task=%s prelaunch_after=%s prelaunch_hold=%.1fs detect_z=%.2f drop_z=%.2f cruise_z=%.2f timeout=%.1fs prestarted=%s launch='roslaunch %s %s'.",
                self.scan_task_name,
                self.pre_scan_launch_task_name,
                self.pre_scan_launch_hold_time,
                self.scan_detect_z,
                self.scan_drop_z,
                self.scan_cruise_z,
                self.scan_result_timeout,
                self.scan_process_prestarted,
                self.scan_launch_package,
                self.scan_launch_file,
            )
        rospy.logwarn(
            "[craic2026_runner] QR mission: payload_topic=%s side_topic=%s scan_z=%.2f cruise_z=%.2f timeout=%.1fs.",
            self.qr_payload_topic,
            self.qr_result_topic,
            self.qr_scan_z,
            self.qr_cruise_z,
            self.qr_wait_timeout,
        )
        if self.drop_enabled:
            rospy.logwarn(
                "[craic2026_runner] drop enabled: targets=%s stable_count=%d release_service=%s wait_before_release=%.1fs wait_after_release=%.1fs.",
                ",".join(self.drop_target_classes),
                self.drop_stable_count,
                self.drop_release_service,
                self.drop_wait_before_release,
                self.drop_wait_after_release,
            )
        if self.special_detector_enabled:
            rospy.logwarn(
                "[craic2026_runner] special detector enabled: camera=%s result_topic=%s detect_z=%.2f confirm=%d timeout=%.1fs fallback_release=%s launch='roslaunch %s %s'.",
                self.special_camera_topic,
                self.special_result_topic,
                self.special_detect_z,
                self.special_confirm_count,
                self.special_detect_timeout,
                self.special_timeout_release_fallback,
                self.special_detector_launch_package,
                self.special_detector_launch_file,
            )
        if self.ring_detector_after_special_enabled:
            rospy.logwarn(
                "[craic2026_runner] ring detector after special: yaw=%.3f wait=%.1fs launch='roslaunch %s %s'.",
                self.ring_detector_turn_yaw,
                self.ring_detector_wait_time,
                self.ring_detector_launch_package,
                self.ring_detector_launch_file,
            )
        rospy.logwarn(
            "[craic2026_runner] ring traverse: task=%s center_topic=%s max_age=%.1fs approach=%.2fm exit=%.2fm z=[%.2f, %.2f].",
            self.ring_task_name,
            self.ring_center_topic,
            self.ring_center_max_age,
            self.ring_approach_distance,
            self.ring_exit_distance,
            self.ring_min_z,
            self.ring_max_z,
        )

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.tick)

    def initial_active_state(self):
        return "AUTO_TAKEOFF" if self.auto_takeoff else "RUNNING"

    def has_qr_read_task(self):
        for task in getattr(self, "tasks", []):
            for wp in task.get("waypoints", []):
                if wp.get("action") == "read_qr":
                    return True
        return False

    def parse_class_list(self, value):
        if isinstance(value, list):
            return [str(v).strip() for v in value if str(v).strip()]
        return [item.strip() for item in str(value).split(",") if item.strip()]

    def split_qr_target_text(self, value):
        return [
            item.strip()
            for item in str(value).replace(",", " ").replace(";", " ").replace("|", " ").split()
            if item.strip()
        ]

    def load_yaml(self, path):
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def load_mission(self):
        if self.qr_landing_side not in ("left", "right"):
            raise ValueError("qr_landing_side must be 'left' or 'right'")

        main_path = self.mission_dir / self.mission_main
        main = self.load_yaml(main_path)
        global_cfg = self.load_yaml(self.mission_dir / main["global_config"])
        motion = global_cfg.get("motion", {})
        self.accept_radius_xy = float(motion.get("accept_radius_xy", 0.15))
        self.accept_radius_z = float(motion.get("accept_radius_z", 0.10))

        tasks = []
        for seq_idx, item in enumerate(main.get("task_sequence", [])):
            if not item.get("enabled", True):
                continue
            if self.max_task_id >= 0 and seq_idx > self.max_task_id:
                continue

            task_cfg = self.load_yaml(self.mission_dir / item["file"])
            waypoints = self.extract_waypoints(task_cfg, item)
            waypoints = self.expand_orbit_waypoints(task_cfg, waypoints)
            if self.max_waypoint_index >= 0:
                waypoints = waypoints[: self.max_waypoint_index + 1]
            if not waypoints:
                raise ValueError("Task %s has no waypoints after filtering" % item["file"])

            tasks.append(
                {
                    "task_id": seq_idx,
                    "file": item["file"],
                    "name": task_cfg.get("mission_segment", item["file"]),
                    "branch_by": item.get("branch_by"),
                    "waypoints": waypoints,
                }
            )
        if not tasks:
            raise ValueError("No enabled tasks loaded")
        return tasks

    def extract_waypoints(self, task_cfg, item):
        if item.get("branch_by") == "qr_landing_side":
            branches = task_cfg.get("branches", {})
            branch = branches.get(self.qr_landing_side)
            if branch is None:
                raise ValueError("Missing landing branch: %s" % self.qr_landing_side)
            return branch.get("waypoints", [])
        return task_cfg.get("waypoints", [])

    def expand_orbit_waypoints(self, task_cfg, waypoints):
        if task_cfg.get("mission_segment") != "orbit_obstacle" or self.orbit_laps <= 1:
            return waypoints
        if len(waypoints) < 5:
            rospy.logwarn(
                "[craic2026_runner] orbit_laps=%d ignored: orbit task has only %d waypoints.",
                self.orbit_laps,
                len(waypoints),
            )
            return waypoints

        entry = copy.deepcopy(waypoints[0])
        cycle = waypoints[1:]
        expanded = [entry]
        for lap_idx in range(1, self.orbit_laps + 1):
            for cycle_idx, wp in enumerate(cycle):
                lap_wp = copy.deepcopy(wp)
                base_id = lap_wp.get("id", "orbit_waypoint")
                lap_wp["id"] = "%s_lap%d" % (base_id, lap_idx)
                if lap_idx < self.orbit_laps and cycle_idx == len(cycle) - 1:
                    lap_wp["hold_time"] = 0.0
                expanded.append(lap_wp)

        rospy.logwarn(
            "[craic2026_runner] orbit_laps=%d expanded orbit waypoints: %d -> %d.",
            self.orbit_laps,
            len(waypoints),
            len(expanded),
        )
        return expanded

    def validate_mission(self):
        if self.output_mode not in ("super_goal", "direct_setpoint"):
            raise ValueError("output_mode must be 'super_goal' or 'direct_setpoint'")
        for task in self.tasks:
            for idx, wp in enumerate(task["waypoints"]):
                point = wp.get("point", [])
                if len(point) != 3:
                    raise ValueError("Invalid point in %s waypoint %s" % (task["file"], wp.get("id")))
                x, y, z = [float(v) for v in point]
                if abs(x) > 10.0 or abs(y) > 10.0 or z > 3.0:
                    raise ValueError(
                        "Unsafe waypoint in %s[%d] %s: [%.3f, %.3f, %.3f]"
                        % (task["file"], idx, wp.get("id"), x, y, z)
                    )
                if wp.get("action") == "read_qr":
                    expected_hold = 0.0
                elif idx == len(task["waypoints"]) - 1 and self.should_auto_land_task(task):
                    expected_hold = 0.0
                elif self.is_scan_task(task):
                    expected_hold = 0.0
                else:
                    expected_hold = 2.0 if idx == len(task["waypoints"]) - 1 else 0.0
                actual_hold = float(wp.get("hold_time", 0.0))
                if abs(actual_hold - expected_hold) > 1e-6:
                    raise ValueError(
                        "Unexpected hold_time in %s waypoint %s: got %.3f expected %.3f"
                        % (task["file"], wp.get("id"), actual_hold, expected_hold)
                    )

    def print_mission(self):
        rospy.loginfo("[craic2026_runner] mission_dir: %s", self.mission_dir)
        rospy.loginfo("[craic2026_runner] dry_run=%s start_paused=%s", self.dry_run, self.start_paused)
        rospy.loginfo(
            "[craic2026_runner] output_mode=%s goal_topic=%s cmd_topic=%s check_z_reached=%s",
            self.output_mode,
            self.goal_topic,
            self.cmd_topic,
            self.check_z_reached,
        )
        rospy.loginfo(
            "[craic2026_runner] auto_takeoff=%s takeoff_height=%.2f topic=%s",
            self.auto_takeoff,
            self.auto_takeoff_height,
            self.takeoff_land_topic,
        )
        rospy.loginfo(
            "[craic2026_runner] post_takeoff_climb=%s point=%s timeout=%.1f",
            self.post_takeoff_climb_enabled,
            self.post_takeoff_climb_point,
            self.post_takeoff_climb_timeout,
        )
        rospy.loginfo(
            "[craic2026_runner] auto_land=%s land_command_duration=%.1f",
            self.auto_land,
            self.land_command_duration,
        )
        rospy.loginfo(
            "[craic2026_runner] scan_infer=%s task=%s detect_z=%.2f drop_z=%.2f cruise_z=%.2f timeout=%.1f result_topic=%s launch='roslaunch %s %s'",
            self.scan_infer_enabled,
            self.scan_task_name,
            self.scan_detect_z,
            self.scan_drop_z,
            self.scan_cruise_z,
            self.scan_result_timeout,
            self.scan_result_topic,
            self.scan_launch_package,
            self.scan_launch_file,
        )
        rospy.loginfo("[craic2026_runner] qr_landing_side=%s", self.qr_landing_side)
        rospy.loginfo("[craic2026_runner] qr_result_topic=%s", self.qr_result_topic)
        rospy.loginfo(
            "[craic2026_runner] qr_wait_timeout=%.1fs launch='roslaunch %s %s enable_infer:=false enable_qr_reader:=true'",
            self.qr_wait_timeout,
            self.qr_launch_package,
            self.qr_launch_file,
        )
        for task in self.tasks:
            rospy.loginfo(
                "[craic2026_runner] task %d %s (%s)",
                task["task_id"],
                task["name"],
                task["file"],
            )
            for idx, wp in enumerate(task["waypoints"]):
                rospy.loginfo(
                    "[craic2026_runner]   wp %d %s point=%s hold=%.1f action=%s final=%s",
                    idx,
                    wp.get("id"),
                    wp.get("point"),
                    float(wp.get("hold_time", 0.0)),
                    wp.get("action"),
                    idx == len(task["waypoints"]) - 1,
                )

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.latest_odom_time = rospy.Time.now()
        q = msg.pose.pose.orientation
        self.last_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def scan_result_cb(self, msg):
        self.scan_last_result = msg.data
        self.scan_last_result_time = rospy.Time.now()

    def special_result_cb(self, msg):
        self.special_last_result = msg.data
        self.special_last_result_time = rospy.Time.now()

    def qr_payload_cb(self, msg):
        self.qr_last_payload = msg.data
        self.qr_last_payload_time = rospy.Time.now()

    def ring_center_cb(self, msg):
        self.ring_center_odom = [
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        ]
        self.ring_center_time = rospy.Time.now()

    def qr_landing_side_cb(self, msg):
        side = self.normalize_qr_landing_side(msg.data)
        if side is None:
            rospy.logwarn("[craic2026_runner] ignoring invalid QR landing side: %s", msg.data)
            return
        if side == self.qr_landing_side:
            self.qr_last_update_time = rospy.Time.now()
            return
        old_side = self.qr_landing_side
        self.qr_landing_side = side
        self.qr_last_update_time = rospy.Time.now()
        rospy.logwarn(
            "[craic2026_runner] QR landing side updated: %s -> %s",
            old_side,
            self.qr_landing_side,
        )
        if hasattr(self, "tasks"):
            self.refresh_pending_landing_branch()

    def normalize_qr_landing_side(self, value):
        text = str(value).strip().lower()
        if text in ("left", "l", "landing_left", "land_left", "左", "左侧", "左边"):
            return "left"
        if text in ("right", "r", "landing_right", "land_right", "右", "右侧", "右边"):
            return "right"
        if "left" in text or "左" in text:
            return "left"
        if "right" in text or "右" in text:
            return "right"
        return None

    def normalize_drop_class(self, value):
        text = str(value).strip()
        if not text:
            return None
        return text

    def parse_qr_payload(self, raw_payload):
        text = str(raw_payload).strip()
        if not text:
            return None

        targets = []
        side = None
        try:
            obj = json.loads(text)
        except Exception:
            obj = None

        if isinstance(obj, dict):
            for key in ("side", "landing_side", "qr_landing_side", "land"):
                if key in obj:
                    side = self.normalize_qr_landing_side(obj.get(key))
                    if side is not None:
                        break
            for key in ("targets", "target_classes", "drop_targets"):
                if key not in obj:
                    continue
                value = obj.get(key)
                if isinstance(value, str):
                    raw_targets = self.split_qr_target_text(value)
                elif isinstance(value, list):
                    raw_targets = value
                else:
                    raw_targets = []
                targets = [self.normalize_drop_class(item) for item in raw_targets]
                targets = [item for item in targets if item]
                if targets:
                    break
        else:
            tokens = self.split_qr_target_text(text)
            for token in tokens:
                token_side = self.normalize_qr_landing_side(token)
                if token_side is not None:
                    side = token_side
                else:
                    cls = self.normalize_drop_class(token)
                    if cls:
                        targets.append(cls)

        if side is None or len(targets) != 2:
            rospy.logwarn(
                "[craic2026_runner] invalid QR payload; need 2 targets and side, got targets=%s side=%s raw=%s",
                targets,
                side,
                text,
            )
            return None
        return {"targets": targets, "landing_side": side}

    def apply_qr_payload_result(self, result):
        self.drop_target_classes = list(result["targets"])
        self.qr_targets_from_payload = list(result["targets"])
        self.qr_payload_confirmed = True
        old_side = self.qr_landing_side
        self.qr_landing_side = result["landing_side"]
        self.qr_last_update_time = rospy.Time.now()
        self.refresh_pending_landing_branch()
        rospy.logwarn(
            "[craic2026_runner] QR payload confirmed: targets=%s landing_side=%s (old_side=%s).",
            ",".join(self.drop_target_classes),
            self.qr_landing_side,
            old_side,
        )

    def consume_valid_qr_payload(self):
        if (
            self.qr_last_payload_time is None
            or self.qr_payload_after_time is None
            or self.qr_last_payload_time < self.qr_payload_after_time
        ):
            return None
        result = self.parse_qr_payload(self.qr_last_payload)
        self.qr_payload_after_time = rospy.Time.now()
        return result

    def refresh_pending_landing_branch(self):
        for task_idx, task in enumerate(self.tasks):
            if task.get("branch_by") != "qr_landing_side":
                continue
            if task_idx <= self.current_task_idx:
                rospy.logwarn(
                    "[craic2026_runner] QR side changed after landing task started; keeping current landing waypoints."
                )
                return

            task_cfg = self.load_yaml(self.mission_dir / task["file"])
            task["waypoints"] = self.extract_waypoints(
                task_cfg,
                {"branch_by": "qr_landing_side"},
            )
            task["waypoints"] = self.expand_orbit_waypoints(task_cfg, task["waypoints"])
            rospy.logwarn(
                "[craic2026_runner] refreshed landing branch to %s: %s",
                self.qr_landing_side,
                [wp.get("id") for wp in task["waypoints"]],
            )
            return

    def start_cb(self, msg):
        if msg.data and self.state in ("PAUSED", "IDLE"):
            if not self.takeoff_done and self.has_qr_read_task():
                self.start_qr_process()
            self.state = self.initial_active_state() if not self.takeoff_done else "RUNNING"
            rospy.logwarn("[craic2026_runner] START received.")

    def pause_cb(self, msg):
        if msg.data and self.state == "RUNNING":
            self.state = "PAUSED"
            rospy.logwarn("[craic2026_runner] PAUSE received.")

    def resume_cb(self, msg):
        if msg.data and self.state == "PAUSED":
            self.state = self.initial_active_state() if not self.takeoff_done else "RUNNING"
            rospy.logwarn("[craic2026_runner] RESUME received.")

    def abort_cb(self, msg):
        if msg.data:
            self.state = "ABORTED"
            self.cleanup_scan_process()
            self.reset_ring_traverse()
            self.set_super_ring_radius_mode(False, force=True)
            rospy.logerr("[craic2026_runner] ABORT received. Publishing stops.")

    def current_task(self):
        if self.current_task_idx >= len(self.tasks):
            return None
        return self.tasks[self.current_task_idx]

    def current_waypoint(self):
        task = self.current_task()
        if task is None:
            return None
        return task["waypoints"][self.current_waypoint_idx]

    def is_final_waypoint(self):
        task = self.current_task()
        if task is None:
            return True
        return self.current_waypoint_idx == len(task["waypoints"]) - 1

    def current_position(self):
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        return [p.x, p.y, p.z]

    def reached(self, wp):
        if self.dry_run:
            return True
        if self.latest_odom is None:
            return False
        p = self.latest_odom.pose.pose.position
        x, y, z = [float(v) for v in wp["point"]]
        xy = math.hypot(p.x - x, p.y - y)
        dz = abs(p.z - z)
        if self.check_z_reached:
            return xy <= self.accept_radius_xy and dz <= self.accept_radius_z
        return xy <= self.accept_radius_xy

    def make_goal(self, wp):
        x, y, z = [float(v) for v in wp["point"]]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        yaw = self.last_yaw if wp.get("yaw") is None else float(wp.get("yaw"))
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.pose.orientation = Quaternion(*q)
        return goal

    def make_cmd(self, wp):
        x, y, z = [float(v) for v in wp["point"]]
        return self.make_cmd_from_point([x, y, z], wp.get("yaw"))

    def make_cmd_from_point(self, point, yaw_value=None):
        x, y, z = [float(v) for v in point]
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "odom"
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0
        cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0
        cmd.jerk.x = cmd.jerk.y = cmd.jerk.z = 0.0
        cmd.yaw = self.last_yaw if yaw_value is None else float(yaw_value)
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = 2026
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return cmd

    def publish_direct_point(self, point, yaw_value=None):
        self.last_target = {"id": "scan_direct", "point": list(point), "action": "scan_direct_control", "yaw": yaw_value}
        if not self.dry_run:
            self.cmd_pub.publish(self.make_cmd_from_point(point, yaw_value))

    def reached_point(self, point):
        if self.dry_run:
            return True
        if self.latest_odom is None:
            return False
        p = self.latest_odom.pose.pose.position
        x, y, z = [float(v) for v in point]
        return math.hypot(p.x - x, p.y - y) <= self.accept_radius_xy and abs(p.z - z) <= self.accept_radius_z

    def publish_target(self, wp):
        self.last_target = wp
        if self.dry_run:
            return
        if self.output_mode == "super_goal":
            key = (self.current_task_idx, self.current_waypoint_idx, wp.get("id"))
            if self.last_goal_publish_key == key:
                return
            self.goal_pub.publish(self.make_goal(wp))
            self.last_goal_publish_key = key
            rospy.loginfo(
                "[craic2026_runner] SUPER goal published: %s point=%s topic=%s action=%s",
                wp.get("id"),
                wp.get("point"),
                self.goal_topic,
                wp.get("action"),
            )
        else:
            self.cmd_pub.publish(self.make_cmd(wp))

    def publish_takeoff_command(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self.takeoff_land_pub.publish(msg)

    def publish_land_command(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.LAND
        self.takeoff_land_pub.publish(msg)

    def should_auto_land_task(self, task):
        return self.auto_land and task is not None and task.get("name") == "landing_approach_only"

    def is_scan_task(self, task):
        return self.scan_infer_enabled and task is not None and task.get("name") == self.scan_task_name

    def should_prelaunch_scan_after_task(self, task):
        return (
            self.scan_infer_enabled
            and task is not None
            and task.get("name") == self.pre_scan_launch_task_name
        )

    def is_qr_read_waypoint(self, wp):
        return wp is not None and wp.get("action") == "read_qr"

    def is_special_drop_task(self, task):
        return self.drop_enabled and task is not None and task.get("name") == self.special_drop_task_name

    def is_ring_task(self, task):
        return task is not None and task.get("name") == self.ring_task_name

    def is_special_task_name(self, task):
        return task is not None and task.get("name") == self.special_drop_task_name

    def should_use_ring_radius(self, task):
        return self.is_special_task_name(task) or self.is_ring_task(task)

    def set_super_ring_radius_mode(self, enabled, force=False):
        if self.dry_run:
            self.super_ring_radius_enabled = bool(enabled)
            self.super_ring_radius_target = bool(enabled)
            return True
        enabled = bool(enabled)
        if (
            not force
            and self.super_ring_radius_target == enabled
            and self.super_ring_radius_enabled == enabled
        ):
            return True
        self.super_ring_radius_target = enabled
        try:
            resp = self.super_ring_radius_srv(enabled)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0,
                "[craic2026_runner] failed to set super ring radius mode=%s via %s: %s",
                enabled,
                self.super_ring_radius_service,
                exc,
            )
            return False
        if not resp.success:
            rospy.logwarn_throttle(
                1.0,
                "[craic2026_runner] super ring radius mode=%s rejected: %s",
                enabled,
                resp.message,
            )
            return False
        self.super_ring_radius_enabled = enabled
        rospy.logwarn(
            "[craic2026_runner] super ring radius mode set to %s: %s",
            enabled,
            resp.message,
        )
        return True

    def update_super_ring_radius_for_task(self, task):
        if self.should_use_ring_radius(task):
            self.set_super_ring_radius_mode(True)
        elif self.super_ring_radius_enabled or self.super_ring_radius_target:
            self.set_super_ring_radius_mode(False)

    def should_force_scan_drop_at_current_waypoint(self):
        task = self.current_task()
        return (
            self.drop_enabled
            and self.is_scan_task(task)
            and (
                (self.current_waypoint_idx == 2 and self.drop_release_count == 0)
                or (self.current_waypoint_idx == 3 and self.drop_release_count == 1)
            )
        )

    def should_finish_scan_task_after_drops(self, task):
        return False

    def finish_current_task_early(self, reason):
        task = self.current_task()
        if task is None:
            return
        rospy.logwarn(
            "[craic2026_runner] task %d %s complete early: %s",
            task["task_id"],
            task.get("name"),
            reason,
        )
        if self.is_scan_task(task):
            self.finish_scan_task()
        if self.is_special_drop_task(task):
            self.cleanup_special_detector_process()
        self.current_task_idx += 1
        self.current_waypoint_idx = 0
        self.final_hold_start = None
        self.last_goal_publish_key = None
        self.qr_phase = None
        self.qr_wait_start = None
        self.qr_payload_after_time = None
        self.cleanup_qr_process()
        self.scan_phase = None
        self.scan_wait_start = None
        self.scan_result_after_time = None
        self.reset_drop_detection_state()
        self.special_drop_phase = None
        self.special_drop_ready_start = None
        self.special_drop_wait_start = None
        self.special_ring_turn_start = None
        self.special_ring_wait_start = None
        self.reset_ring_traverse()
        if self.current_task_idx >= len(self.tasks):
            self.state = "COMPLETE"
            rospy.logwarn("[craic2026_runner] mission complete. Holding last target; no land/disarm.")

    def fresh_ring_center(self, now):
        if self.ring_center_odom is None or self.ring_center_time is None:
            return None
        age = (now - self.ring_center_time).to_sec()
        if age > self.ring_center_max_age:
            rospy.loginfo_throttle(
                1.0,
                "[craic2026_runner] ring center stale: age=%.1fs max=%.1fs.",
                age,
                self.ring_center_max_age,
            )
            return None
        return list(self.ring_center_odom)

    def make_ring_targets(self, center):
        yaw = self.ring_detector_turn_yaw
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        z = max(self.ring_min_z, min(self.ring_max_z, float(center[2]) + 0.1))
        center = [float(center[0]), float(center[1]), z]
        approach = [
            center[0] - dx * self.ring_approach_distance,
            center[1] - dy * self.ring_approach_distance,
            z,
        ]
        exit_point = [
            center[0] + dx * self.ring_exit_distance,
            center[1] + dy * self.ring_exit_distance,
            z,
        ]
        return {
            "APPROACH": approach,
            "CENTER": center,
            "EXIT": exit_point,
        }

    def reset_ring_traverse(self):
        self.ring_phase = None
        self.ring_targets = None
        self.ring_wait_point = None

    def finish_special_ring_wait(self, skip_ring_task=False):
        self.special_drop_phase = None
        self.special_drop_ready_start = None
        self.special_drop_wait_start = None
        self.special_ring_turn_start = None
        self.special_ring_wait_start = None
        self.special_drop_done = True
        self.advance_waypoint_or_task()
        if not skip_ring_task:
            return
        task = self.current_task()
        if task is None or not self.is_ring_task(task):
            self.set_super_ring_radius_mode(False, force=True)
            return
        rospy.logwarn(
            "[craic2026_runner] skip ring task %s after ring center wait timeout; enter next task.",
            task.get("name"),
        )
        self.set_super_ring_radius_mode(False, force=True)
        self.current_task_idx += 1
        self.current_waypoint_idx = 0
        self.final_hold_start = None
        self.last_goal_publish_key = None
        self.reset_ring_traverse()
        if self.current_task_idx >= len(self.tasks):
            self.state = "COMPLETE"
            rospy.logwarn("[craic2026_runner] mission complete after skipping ring task.")

    def run_ring_sequence(self, now):
        task = self.current_task()
        wp = self.current_waypoint()
        if not self.is_ring_task(task):
            return False

        if wp is not None and wp.get("action") in ("start_ring_detection", "detect_ring"):
            self.start_ring_detector_process()

        if self.ring_phase is None:
            center = self.fresh_ring_center(now)
            if center is None:
                if self.ring_wait_point is None:
                    self.ring_wait_point = self.current_position()
                    if self.ring_wait_point is not None:
                        rospy.logwarn(
                            "[craic2026_runner] ring task waiting for latched center; hold current point=%s.",
                            [round(v, 3) for v in self.ring_wait_point],
                        )
                if self.ring_wait_point is not None:
                    self.publish_direct_point(self.ring_wait_point, self.ring_detector_turn_yaw)
                rospy.loginfo_throttle(
                    1.0,
                    "[craic2026_runner] ring task has no fresh center yet; search waypoints are disabled.",
                )
                return True
            self.ring_targets = self.make_ring_targets(center)
            self.ring_phase = "APPROACH"
            rospy.logwarn(
                "[craic2026_runner] ring center locked: center=%s approach=%s exit=%s.",
                [round(v, 3) for v in self.ring_targets["CENTER"]],
                [round(v, 3) for v in self.ring_targets["APPROACH"]],
                [round(v, 3) for v in self.ring_targets["EXIT"]],
            )

        target = self.ring_targets[self.ring_phase]
        self.publish_direct_point(target, self.ring_detector_turn_yaw)
        if not self.reached_point(target):
            rospy.loginfo_throttle(
                0.5,
                "[craic2026_runner] ring %s target=%s.",
                self.ring_phase.lower(),
                [round(v, 3) for v in target],
            )
            return True

        if self.ring_phase == "APPROACH":
            self.ring_phase = "CENTER"
            rospy.logwarn("[craic2026_runner] ring approach reached; fly through center.")
            return True
        if self.ring_phase == "CENTER":
            self.ring_phase = "EXIT"
            rospy.logwarn("[craic2026_runner] ring center reached; continue to exit point.")
            return True

        rospy.logwarn("[craic2026_runner] ring exit reached; mission5 complete.")
        self.reset_ring_traverse()
        self.set_super_ring_radius_mode(False, force=True)
        if task is not None:
            self.current_waypoint_idx = len(task["waypoints"]) - 1
        self.advance_waypoint_or_task()
        return True

    def reset_drop_detection_state(self):
        self.drop_stable_class = None
        self.drop_stable_count_seen = 0
        self.drop_non_target_confirm_rounds = 0
        self.drop_processed_result_time = None
        self.scan_drop_ready_start = None
        self.scan_pending_drop_reason = None
        self.scan_pending_drop_class = None
        self.drop_wait_start = None

    def parse_target_class_name(self, raw_result):
        try:
            payload = json.loads(raw_result)
        except Exception as exc:
            rospy.logwarn("[craic2026_runner] invalid /target_info JSON: %s", exc)
            return None
        class_name = payload.get("class_name")
        if class_name is None:
            return None
        class_name = str(class_name).strip()
        if not class_name or class_name in ("unknown", "error"):
            return None
        return class_name

    def consume_stable_scan_class(self):
        if (
            self.scan_last_result_time is None
            or self.scan_result_after_time is None
            or self.scan_last_result_time < self.scan_result_after_time
        ):
            return None
        if (
            self.drop_processed_result_time is not None
            and self.scan_last_result_time <= self.drop_processed_result_time
        ):
            return None

        self.drop_processed_result_time = self.scan_last_result_time
        class_name = self.parse_target_class_name(self.scan_last_result)
        if class_name is None:
            self.drop_stable_class = None
            self.drop_stable_count_seen = 0
            return None

        if class_name == self.drop_stable_class:
            self.drop_stable_count_seen += 1
        else:
            self.drop_stable_class = class_name
            self.drop_stable_count_seen = 1

        rospy.loginfo(
            "[craic2026_runner] scan class=%s stable=%d/%d.",
            class_name,
            self.drop_stable_count_seen,
            self.drop_stable_count,
        )
        if self.drop_stable_count_seen >= self.drop_stable_count:
            return class_name
        return None

    def release_next_drop(self, reason):
        if self.dry_run:
            self.drop_release_count += 1
            rospy.logwarn("[craic2026_runner] DRY RUN drop release: %s.", reason)
            return True
        if self.drop_release_srv is None:
            rospy.logerr("[craic2026_runner] drop release service is not configured.")
            return False
        try:
            resp = self.drop_release_srv()
        except rospy.ServiceException as exc:
            rospy.logerr("[craic2026_runner] drop release service failed: %s", exc)
            return False
        if not resp.success:
            rospy.logerr("[craic2026_runner] drop release rejected: %s", resp.message)
            return False
        self.drop_release_count += 1
        rospy.logwarn("[craic2026_runner] drop release ok: %s (%s).", reason, resp.message)
        return True

    def start_qr_process(self):
        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN QR perception: skip roslaunch.")
            return True
        if self.qr_process is not None and self.qr_process.poll() is None:
            rospy.loginfo("[craic2026_runner] QR perception already running.")
            return True
        if self.qr_process is not None:
            rospy.logwarn(
                "[craic2026_runner] previous QR process exited with code %s; restarting.",
                self.qr_process.poll(),
            )
            self.qr_process = None
        cmd = [
            "roslaunch",
            self.qr_launch_package,
            self.qr_launch_file,
            "enable_infer:=false",
            "enable_qr_reader:=true",
        ]
        rospy.logwarn("[craic2026_runner] starting QR perception: %s", " ".join(cmd))
        self.qr_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        return True

    def cleanup_qr_process(self):
        if self.qr_process is None:
            return
        if self.qr_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.qr_process.pid), signal.SIGINT)
                self.qr_process.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.qr_process.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.qr_process = None

    def start_scan_process(self):
        if self.scan_process_prestarted:
            rospy.loginfo("[craic2026_runner] scan infer marked prestarted; skip roslaunch.")
            self.scan_process_started_for_mission = True
            return True
        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN scan infer: skip roslaunch.")
            self.scan_process_started_for_mission = True
            return True
        if self.scan_process is not None and self.scan_process.poll() is None:
            self.scan_process_started_for_mission = True
            rospy.loginfo("[craic2026_runner] scan infer already running.")
            return True
        if self.scan_process is not None:
            rospy.logwarn(
                "[craic2026_runner] previous scan infer process exited with code %s; restarting.",
                self.scan_process.poll(),
            )
            self.scan_process = None
        cmd = ["roslaunch", self.scan_launch_package, self.scan_launch_file]
        rospy.logwarn("[craic2026_runner] starting scan infer: %s", " ".join(cmd))
        self.scan_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.scan_process_started_for_mission = True
        return True

    def ensure_scan_process_running(self):
        if self.scan_process_prestarted:
            return True
        if self.dry_run:
            return True
        if self.scan_process is not None and self.scan_process.poll() is None:
            return True
        rospy.logwarn("[craic2026_runner] scan infer is not running; starting fallback launch now.")
        return self.start_scan_process()

    def finish_scan_task(self):
        self.cleanup_scan_process()
        self.scan_process_started_for_mission = False

    def cleanup_scan_process(self):
        if self.scan_process_prestarted:
            return
        if self.scan_process is None:
            return
        if self.scan_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.scan_process.pid), signal.SIGINT)
                self.scan_process.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.scan_process.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.scan_process = None
        self.scan_process_started_for_mission = False

    def cleanup_ring_detector_process(self):
        if self.ring_detector_process is None:
            self.ring_detector_started = False
            self.ring_detector_start_time = None
            self.ring_detector_health_last_check = None
            return
        if self.ring_detector_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.ring_detector_process.pid), signal.SIGINT)
                self.ring_detector_process.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.ring_detector_process.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.ring_detector_process = None
        self.ring_detector_started = False
        self.ring_detector_start_time = None
        self.ring_detector_health_last_check = None

    def cleanup_special_detector_process(self):
        if self.special_detector_process is None:
            self.special_detector_started = False
            return
        if self.special_detector_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.special_detector_process.pid), signal.SIGINT)
                self.special_detector_process.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.special_detector_process.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.special_detector_process = None
        self.special_detector_started = False

    def cleanup_processes(self):
        self.set_super_ring_radius_mode(False, force=True)
        self.cleanup_qr_process()
        self.cleanup_scan_process()
        self.cleanup_special_detector_process()
        self.cleanup_ring_detector_process()

    def start_special_detector_process(self):
        if not self.special_detector_enabled:
            return True
        if self.special_detector_started:
            return True
        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN special detector: skip roslaunch.")
            self.special_detector_started = True
            return True
        if self.special_detector_process is not None and self.special_detector_process.poll() is None:
            self.special_detector_started = True
            rospy.loginfo("[craic2026_runner] special detector already running.")
            return True
        if self.special_detector_process is not None:
            rospy.logwarn(
                "[craic2026_runner] previous special detector process exited with code %s; restarting.",
                self.special_detector_process.poll(),
            )
            self.special_detector_process = None
        cmd = [
            "roslaunch",
            self.special_detector_launch_package,
            self.special_detector_launch_file,
            "camera_topic:=%s" % self.special_camera_topic,
            "enable_usb_cam:=%s" % str(self.special_detector_enable_usb_cam).lower(),
            "stable_count:=%d" % self.special_confirm_count,
            "result_topic:=%s" % self.special_result_topic,
        ]
        rospy.logwarn("[craic2026_runner] starting special detector: %s", " ".join(cmd))
        self.special_detector_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.special_detector_started = True
        return True

    def start_ring_detector_process(self):
        if self.ring_detector_started:
            if self.ring_detector_process is not None and self.ring_detector_process.poll() is None:
                return True
            exit_code = (
                self.ring_detector_process.poll()
                if self.ring_detector_process is not None
                else None
            )
            rospy.logwarn(
                "[craic2026_runner] ring detector marked started but process is not running (exit=%s); restarting.",
                exit_code,
            )
            self.ring_detector_process = None
            self.ring_detector_started = False
        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN ring detector: skip roslaunch.")
            self.ring_detector_started = True
            self.ring_detector_start_time = rospy.Time.now()
            return True
        if self.ring_detector_process is not None and self.ring_detector_process.poll() is None:
            self.ring_detector_started = True
            if self.ring_detector_start_time is None:
                self.ring_detector_start_time = rospy.Time.now()
            rospy.loginfo("[craic2026_runner] ring detector already running.")
            return True
        if self.ring_detector_process is not None:
            rospy.logwarn(
                "[craic2026_runner] previous ring detector process exited with code %s; restarting.",
                self.ring_detector_process.poll(),
            )
            self.ring_detector_process = None
        cmd = [
            "roslaunch",
            self.ring_detector_launch_package,
            self.ring_detector_launch_file,
            "center_estimation_mode:=%s" % self.ring_detector_center_estimation_mode,
            "ring_size_shape_source:=%s" % self.ring_detector_size_shape_source,
            "ring_size_alpha_policy:=%s" % self.ring_detector_size_alpha_policy,
            "camera_xyz_offset_base:=%s" % self.ring_detector_camera_xyz_offset_base,
            "show_debug_window:=%s" % str(self.ring_detector_show_window).lower(),
        ]
        rospy.logwarn("[craic2026_runner] starting ring detector: %s", " ".join(cmd))
        self.ring_detector_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.ring_detector_started = True
        self.ring_detector_start_time = rospy.Time.now()
        self.ring_detector_health_last_check = None
        return True

    def is_ros_node_registered(self, node_name):
        try:
            rosgraph.Master(rospy.get_name()).lookupNode(node_name)
            return True
        except Exception:
            return False

    def ensure_ring_detector_process_running(self, now):
        if not self.start_ring_detector_process():
            return False
        if self.dry_run:
            return True
        if (
            self.ring_detector_process is None
            or self.ring_detector_process.poll() is not None
        ):
            return self.start_ring_detector_process()
        if self.ring_detector_start_time is None:
            self.ring_detector_start_time = now

        since_start = (now - self.ring_detector_start_time).to_sec()
        if since_start < 8.0:
            return True
        if (
            self.ring_detector_health_last_check is not None
            and (now - self.ring_detector_health_last_check).to_sec() < 1.0
        ):
            return True
        self.ring_detector_health_last_check = now

        if self.is_ros_node_registered("/ring_detector"):
            return True

        rospy.logwarn(
            "[craic2026_runner] ring detector launch is alive but /ring_detector is not registered after %.1fs; restarting.",
            since_start,
        )
        self.cleanup_ring_detector_process()
        return self.start_ring_detector_process()

    def yaw_error(self, target_yaw):
        return math.atan2(math.sin(target_yaw - self.last_yaw), math.cos(target_yaw - self.last_yaw))

    def begin_qr_sequence(self, now, wp):
        self.qr_phase = "DESCEND"
        self.qr_wait_start = None
        self.qr_payload_after_time = None
        self.qr_payload_confirmed = False
        self.start_qr_process()
        rospy.logwarn(
            "[craic2026_runner] QR waypoint %s reached at cruise; descend to %.2fm for QR.",
            wp.get("id"),
            self.qr_scan_z,
        )

    def run_qr_sequence(self, now):
        wp = self.current_waypoint()
        if wp is None:
            self.qr_phase = None
            return
        x, y, _ = [float(v) for v in wp["point"]]
        yaw = wp.get("yaw")
        scan_point = [x, y, self.qr_scan_z]
        cruise_point = [x, y, self.qr_cruise_z]

        if self.qr_phase == "DESCEND":
            self.publish_direct_point(scan_point, yaw)
            if self.reached_point(scan_point):
                self.qr_phase = "WAIT_QR"
                self.qr_wait_start = now
                self.qr_payload_after_time = now
                rospy.logwarn(
                    "[craic2026_runner] QR scan point reached at %.2fm; wait payload on %s for %.1fs.",
                    self.qr_scan_z,
                    self.qr_payload_topic,
                    self.qr_wait_timeout,
                )
            return

        if self.qr_phase == "WAIT_QR":
            self.publish_direct_point(scan_point, yaw)
            result = self.consume_valid_qr_payload()
            if result is not None:
                self.apply_qr_payload_result(result)
                self.qr_phase = "ASCEND"
                return
            elapsed = (now - self.qr_wait_start).to_sec() if self.qr_wait_start else 0.0
            if elapsed >= self.qr_wait_timeout:
                fallback = {"targets": ["apple", "motorcycle"], "landing_side": "left"}
                rospy.logwarn(
                    "[craic2026_runner] QR payload timeout at %s after %.1fs; use fallback targets=apple,motorcycle landing_side=left and continue.",
                    wp.get("id"),
                    elapsed,
                )
                self.apply_qr_payload_result(fallback)
                self.qr_phase = "ASCEND"
                return
            rospy.loginfo_throttle(
                1.0,
                "[craic2026_runner] waiting QR payload at %s: %.1f/%.1fs.",
                wp.get("id"),
                elapsed,
                self.qr_wait_timeout,
            )
            return

        if self.qr_phase == "ASCEND":
            self.publish_direct_point(cruise_point, yaw)
            if self.reached_point(cruise_point):
                rospy.logwarn(
                    "[craic2026_runner] QR mission complete; back to %.2fm and stop perception.",
                    self.qr_cruise_z,
                )
                self.cleanup_qr_process()
                self.qr_phase = None
                self.qr_wait_start = None
                self.qr_payload_after_time = None
                self.advance_waypoint_or_task()
            return

    def begin_scan_sequence(self, now, wp):
        self.scan_phase = "DESCEND"
        self.scan_wait_start = None
        self.scan_result_after_time = None
        self.scan_last_result = None
        self.scan_last_result_time = None
        self.reset_drop_detection_state()
        rospy.logwarn(
            "[craic2026_runner] scan waypoint %s reached; descend to %.2fm for inference.",
            wp.get("id"),
            self.scan_detect_z,
        )

    def run_scan_sequence(self, now):
        wp = self.current_waypoint()
        if wp is None:
            self.scan_phase = None
            return
        x, y, _ = [float(v) for v in wp["point"]]
        yaw = wp.get("yaw")
        detect_point = [x, y, self.scan_detect_z]
        drop_point = [x, y, self.scan_drop_z]
        cruise_point = [x, y, self.scan_cruise_z]

        if self.scan_phase == "DESCEND":
            self.publish_direct_point(detect_point, yaw)
            if self.reached_point(detect_point):
                self.ensure_scan_process_running()
                self.scan_phase = "WAIT_INFER"
                self.scan_wait_start = now
                self.scan_drop_ready_start = None
                self.scan_result_after_time = now
                rospy.logwarn(
                    "[craic2026_runner] scan waypoint %s at %.2fm; wait /target_info or %.1fs timeout.",
                    wp.get("id"),
                    self.scan_detect_z,
                    self.scan_result_timeout,
                )
            return

        if self.scan_phase == "DESCEND_DROP":
            self.publish_direct_point(drop_point, yaw)
            if self.reached_point(drop_point):
                self.scan_drop_ready_start = now
                self.scan_phase = "WAIT_DROP_RELEASE"
                rospy.logwarn(
                    "[craic2026_runner] scan waypoint %s at drop height %.2fm; hold %.1fs before release.",
                    wp.get("id"),
                    self.scan_drop_z,
                    self.drop_wait_before_release,
                )
            return

        if self.scan_phase == "WAIT_DROP_RELEASE":
            self.publish_direct_point(drop_point, yaw)
            ready_start = self.scan_drop_ready_start or self.scan_wait_start or now
            elapsed = (now - ready_start).to_sec()
            if elapsed < self.drop_wait_before_release:
                rospy.loginfo_throttle(
                    0.5,
                    "[craic2026_runner] holding at drop height before release at %s: %.1f/%.1fs.",
                    wp.get("id"),
                    elapsed,
                    self.drop_wait_before_release,
                )
                return
            reason = self.scan_pending_drop_reason or "mission3 delayed drop at %s" % wp.get("id")
            if self.release_next_drop(reason):
                if self.scan_pending_drop_class is not None:
                    self.dropped_target_classes.add(self.scan_pending_drop_class)
                self.scan_pending_drop_reason = None
                self.scan_pending_drop_class = None
                self.drop_wait_start = now
                self.scan_phase = "DROP_WAIT"
            else:
                self.state = "ERROR"
            return

        if self.scan_phase == "WAIT_INFER":
            self.publish_direct_point(detect_point, yaw)
            elapsed = (now - self.scan_wait_start).to_sec() if self.scan_wait_start else 0.0
            if self.drop_enabled:
                stable_class = self.consume_stable_scan_class()
                if stable_class is not None:
                    is_target = (
                        stable_class in self.drop_target_classes
                        and stable_class not in self.dropped_target_classes
                    )
                    if is_target:
                        rospy.logwarn(
                            "[craic2026_runner] target %s confirmed at %s; release next magnet after %.1fs at drop height.",
                            stable_class,
                            wp.get("id"),
                            self.drop_wait_before_release,
                        )
                        self.scan_drop_ready_start = None
                        self.scan_pending_drop_reason = "mission3 target %s at %s" % (stable_class, wp.get("id"))
                        self.scan_pending_drop_class = stable_class
                        self.scan_phase = "DESCEND_DROP"
                        return

                    rospy.logwarn(
                        "[craic2026_runner] non-target %s confirmed at %s; keep waiting until timeout.",
                        stable_class,
                        wp.get("id"),
                    )
                    self.drop_stable_class = None
                    self.drop_stable_count_seen = 0
                    self.drop_processed_result_time = None
                    self.scan_result_after_time = now
                    return

                if elapsed >= self.scan_result_timeout:
                    if self.should_force_scan_drop_at_current_waypoint():
                        self.scan_pending_drop_reason = "mission3 fallback drop at %s" % wp.get("id")
                        self.scan_pending_drop_class = None
                        self.scan_phase = "DESCEND_DROP"
                        rospy.logwarn(
                            "[craic2026_runner] scan timeout at %s after %.1fs; fallback drop via %.2fm -> %.2fm route.",
                            wp.get("id"),
                            elapsed,
                            self.scan_detect_z,
                            self.scan_drop_z,
                        )
                    else:
                        rospy.logwarn(
                            "[craic2026_runner] scan timeout at %s after %.1fs without target match; continue.",
                            wp.get("id"),
                            elapsed,
                        )
                        self.scan_phase = "ASCEND"
                return

            has_result = (
                self.scan_last_result_time is not None
                and self.scan_result_after_time is not None
                and self.scan_last_result_time >= self.scan_result_after_time
            )
            if has_result:
                rospy.logwarn(
                    "[craic2026_runner] scan result received at %s: %s",
                    wp.get("id"),
                    self.scan_last_result,
                )
            if has_result or elapsed >= self.scan_result_timeout:
                if not has_result:
                    rospy.logwarn(
                        "[craic2026_runner] scan timeout at %s after %.1fs; continue.",
                        wp.get("id"),
                        elapsed,
                    )
                self.scan_phase = "ASCEND"
            return

        if self.scan_phase == "DROP_WAIT":
            self.publish_direct_point(drop_point, yaw)
            elapsed = (now - self.drop_wait_start).to_sec() if self.drop_wait_start else 0.0
            if elapsed >= self.drop_wait_after_release:
                self.scan_phase = "ASCEND"
            return

        if self.scan_phase == "ASCEND":
            self.publish_direct_point(cruise_point, yaw)
            if self.reached_point(cruise_point):
                task = self.current_task()
                rospy.logwarn(
                    "[craic2026_runner] scan waypoint %s complete; back to %.2fm.",
                    wp.get("id"),
                    self.scan_cruise_z,
                )
                if self.should_finish_scan_task_after_drops(task):
                    self.finish_current_task_early(
                        "drop_release_count=%d reached target count 2" % self.drop_release_count
                    )
                    return
                self.scan_phase = None
                self.scan_wait_start = None
                self.scan_result_after_time = None
                self.reset_drop_detection_state()
                if self.is_final_waypoint():
                    self.finish_scan_task()
                self.advance_waypoint_or_task()
            return

    def reset_special_detection_state(self, now=None):
        self.special_detect_wait_start = now
        self.special_result_after_time = now
        self.special_last_result = None
        self.special_last_result_time = None
        self.special_confirm_seen = 0

    def consume_special_target_confirmed(self):
        if (
            self.special_last_result_time is None
            or self.special_result_after_time is None
            or self.special_last_result_time < self.special_result_after_time
        ):
            return False
        try:
            payload = json.loads(self.special_last_result)
        except Exception as exc:
            rospy.logwarn("[craic2026_runner] invalid special target JSON: %s", exc)
            self.special_confirm_seen = 0
            return False

        if "stable_present" in payload:
            stable_present = bool(payload.get("stable_present", False))
            present_streak = int(payload.get("present_streak", 0) or 0)
            self.special_confirm_seen = (
                max(self.special_confirm_count, present_streak)
                if stable_present
                else present_streak
            )
            best_conf = float(payload.get("best_confidence", 0.0) or 0.0)
            rospy.loginfo(
                "[craic2026_runner] special target stable_present=%s best=%.3f confirm=%d/%d.",
                stable_present,
                best_conf,
                self.special_confirm_seen,
                self.special_confirm_count,
            )
            self.special_result_after_time = self.special_last_result_time + rospy.Duration(1e-6)
            return stable_present

        present = bool(payload.get("present", False))
        best_conf = float(payload.get("best_confidence", 0.0) or 0.0)
        if present:
            self.special_confirm_seen += 1
        else:
            self.special_confirm_seen = 0

        rospy.loginfo(
            "[craic2026_runner] special target present=%s best=%.3f confirm=%d/%d.",
            present,
            best_conf,
            self.special_confirm_seen,
            self.special_confirm_count,
        )
        self.special_result_after_time = self.special_last_result_time + rospy.Duration(1e-6)
        return self.special_confirm_seen >= self.special_confirm_count

    def begin_special_drop_sequence(self, now, wp):
        self.special_drop_phase = "DETECT_DESCEND" if self.special_detector_enabled else "DESCEND"
        self.special_drop_ready_start = None
        self.special_drop_wait_start = None
        self.reset_special_detection_state(None)
        if self.special_detector_enabled:
            self.start_special_detector_process()
            rospy.logwarn(
                "[craic2026_runner] special waypoint %s reached; descend to %.2fm and wait %s for %.1fs.",
                wp.get("id"),
                self.special_detect_z,
                self.special_result_topic,
                self.special_detect_timeout,
            )
        else:
            rospy.logwarn(
                "[craic2026_runner] special waypoint %s reached; detector disabled, descend to %.2fm for drop.",
                wp.get("id"),
                self.special_drop_descent_z,
            )

    def run_special_drop_sequence(self, now):
        wp = self.current_waypoint()
        if wp is None:
            self.special_drop_phase = None
            return

        x, y, _ = [float(v) for v in wp["point"]]
        yaw = wp.get("yaw")
        drop_point = [x, y, self.special_drop_descent_z]
        detect_point = [x, y, self.special_detect_z]
        cruise_point = [x, y, self.special_drop_cruise_z]

        if self.special_drop_phase == "DETECT_DESCEND":
            self.publish_direct_point(detect_point, yaw)
            if self.reached_point(detect_point):
                self.reset_special_detection_state(now)
                rospy.logwarn(
                    "[craic2026_runner] special detect point reached at %.2fm; wait target confirmation for %.1fs.",
                    self.special_detect_z,
                    self.special_detect_timeout,
                )
                self.special_drop_phase = "WAIT_DETECT"
            return

        if self.special_drop_phase == "WAIT_DETECT":
            self.publish_direct_point(detect_point, yaw)
            if self.consume_special_target_confirmed():
                rospy.logwarn(
                    "[craic2026_runner] special target confirmed at %s; descend to %.2fm for drop.",
                    wp.get("id"),
                    self.special_drop_descent_z,
                )
                self.special_drop_phase = "DESCEND"
                return

            elapsed = (
                (now - self.special_detect_wait_start).to_sec()
                if self.special_detect_wait_start is not None
                else 0.0
            )
            if elapsed >= self.special_detect_timeout:
                if self.special_timeout_release_fallback:
                    rospy.logwarn(
                        "[craic2026_runner] special target timeout at %s after %.1fs; fallback release is enabled, descend anyway.",
                        wp.get("id"),
                        elapsed,
                    )
                    self.special_drop_phase = "DESCEND"
                else:
                    rospy.logwarn(
                        "[craic2026_runner] special target timeout at %s after %.1fs; skip special release.",
                        wp.get("id"),
                        elapsed,
                    )
                    self.special_drop_phase = "ASCEND"
                return

            rospy.loginfo_throttle(
                1.0,
                "[craic2026_runner] waiting special target confirmation at %s: %.1f/%.1fs confirm=%d/%d.",
                wp.get("id"),
                elapsed,
                self.special_detect_timeout,
                self.special_confirm_seen,
                self.special_confirm_count,
            )
            return

        if self.special_drop_phase == "DESCEND":
            self.publish_direct_point(drop_point, yaw)
            if self.reached_point(drop_point):
                if self.drop_release_count != 2:
                    self.state = "ERROR"
                    rospy.logerr(
                        "[craic2026_runner] refusing mission4 drop: expected 2 prior releases, got %d.",
                        self.drop_release_count,
                    )
                    return
                self.special_drop_ready_start = now
                rospy.logwarn(
                    "[craic2026_runner] special drop point reached at %.2fm; hold %.1fs before release.",
                    self.special_drop_descent_z,
                    self.drop_wait_before_release,
                )
                self.special_drop_phase = "WAIT_DROP_RELEASE"
            return

        if self.special_drop_phase == "WAIT_DROP_RELEASE":
            self.publish_direct_point(drop_point, yaw)
            ready_start = self.special_drop_ready_start or now
            elapsed = (now - ready_start).to_sec()
            if elapsed < self.drop_wait_before_release:
                rospy.loginfo_throttle(
                    0.5,
                    "[craic2026_runner] holding at special drop height before release: %.1f/%.1fs.",
                    elapsed,
                    self.drop_wait_before_release,
                )
                return
            rospy.logwarn(
                "[craic2026_runner] special drop hold complete at %.2fm; release next magnet.",
                self.special_drop_descent_z,
            )
            if self.release_next_drop("mission4 special target %s" % wp.get("id")):
                self.special_drop_wait_start = now
                self.special_drop_phase = "DROP_WAIT"
            else:
                self.state = "ERROR"
            return

        if self.special_drop_phase == "DROP_WAIT":
            self.publish_direct_point(drop_point, yaw)
            elapsed = (
                (now - self.special_drop_wait_start).to_sec()
                if self.special_drop_wait_start is not None
                else 0.0
            )
            if elapsed >= self.drop_wait_after_release:
                self.special_drop_phase = "ASCEND"
            return

        if self.special_drop_phase == "ASCEND":
            self.publish_direct_point(cruise_point, yaw)
            if self.reached_point(cruise_point):
                if self.ring_detector_after_special_enabled:
                    self.special_drop_phase = "RING_TURN"
                    self.special_ring_turn_start = now
                    rospy.logwarn(
                        "[craic2026_runner] special drop back to %.2fm; turn in place to yaw %.3f before ring detector.",
                        self.special_drop_cruise_z,
                        self.ring_detector_turn_yaw,
                    )
                    return
                rospy.logwarn(
                    "[craic2026_runner] special drop complete; back to %.2fm.",
                    self.special_drop_cruise_z,
                )
                self.special_drop_phase = None
                self.special_drop_ready_start = None
                self.special_drop_wait_start = None
                self.special_drop_done = True
                self.advance_waypoint_or_task()
            return

        if self.special_drop_phase == "RING_TURN":
            self.publish_direct_point(cruise_point, self.ring_detector_turn_yaw)
            elapsed = (now - self.special_ring_turn_start).to_sec() if self.special_ring_turn_start else 0.0
            yaw_err = abs(self.yaw_error(self.ring_detector_turn_yaw))
            if self.reached_point(cruise_point) and yaw_err <= self.ring_detector_yaw_tolerance:
                rospy.logwarn(
                    "[craic2026_runner] ring detector yaw ready: current=%.3f target=%.3f err=%.3f.",
                    self.last_yaw,
                    self.ring_detector_turn_yaw,
                    yaw_err,
                )
                self.start_ring_detector_process()
                self.special_ring_wait_start = now
                self.special_drop_phase = "RING_WAIT"
                return
            if elapsed >= self.ring_detector_turn_timeout:
                rospy.logwarn(
                    "[craic2026_runner] ring detector turn timeout after %.1fs: current_yaw=%.3f target=%.3f err=%.3f; start detector anyway.",
                    elapsed,
                    self.last_yaw,
                    self.ring_detector_turn_yaw,
                    yaw_err,
                )
                self.start_ring_detector_process()
                self.special_ring_wait_start = now
                self.special_drop_phase = "RING_WAIT"
                return
            rospy.loginfo_throttle(
                0.5,
                "[craic2026_runner] turning before ring detector: yaw=%.3f target=%.3f err=%.3f elapsed=%.1fs.",
                self.last_yaw,
                self.ring_detector_turn_yaw,
                yaw_err,
                elapsed,
            )
            return

        if self.special_drop_phase == "RING_WAIT":
            self.publish_direct_point(cruise_point, self.ring_detector_turn_yaw)
            self.ensure_ring_detector_process_running(now)
            elapsed = (now - self.special_ring_wait_start).to_sec() if self.special_ring_wait_start else 0.0
            center = self.fresh_ring_center(now)
            if center is not None:
                rospy.logwarn(
                    "[craic2026_runner] fresh ring center received after %.1fs; mission4 complete, enter mission5.",
                    elapsed,
                )
                self.finish_special_ring_wait(skip_ring_task=False)
                return
            if elapsed >= self.ring_detector_wait_time:
                rospy.logwarn(
                    "[craic2026_runner] no fresh ring center after %.1fs; skip mission5 and continue.",
                    elapsed,
                )
                self.finish_special_ring_wait(skip_ring_task=True)
                return
            rospy.loginfo_throttle(
                1.0,
                "[craic2026_runner] waiting for fresh ring center after mission4: %.1f/%.1fs.",
                elapsed,
                self.ring_detector_wait_time,
            )
            return

    def advance_waypoint_or_task(self):
        task = self.current_task()
        wp = self.current_waypoint()
        if self.is_final_waypoint():
            rospy.loginfo(
                "[craic2026_runner] task %d complete at final waypoint %s",
                task["task_id"],
                wp.get("id"),
            )
            if self.is_special_drop_task(task):
                self.cleanup_special_detector_process()
            self.current_task_idx += 1
            self.current_waypoint_idx = 0
            self.final_hold_start = None
            self.last_goal_publish_key = None
            self.qr_phase = None
            self.qr_wait_start = None
            self.qr_payload_after_time = None
            self.cleanup_qr_process()
            self.special_drop_phase = None
            self.special_drop_ready_start = None
            self.special_drop_wait_start = None
            self.special_ring_turn_start = None
            self.special_ring_wait_start = None
            self.reset_ring_traverse()
            if self.current_task_idx >= len(self.tasks):
                self.state = "COMPLETE"
                rospy.logwarn("[craic2026_runner] mission complete. Holding last target; no land/disarm.")
        else:
            rospy.loginfo(
                "[craic2026_runner] reached intermediate waypoint %s; switching immediately",
                wp.get("id"),
            )
            self.current_waypoint_idx += 1
            self.final_hold_start = None
            self.last_goal_publish_key = None
            self.qr_phase = None
            self.qr_wait_start = None
            self.qr_payload_after_time = None
            self.cleanup_qr_process()
            self.special_drop_phase = None
            self.special_drop_ready_start = None
            self.special_drop_wait_start = None
            self.special_ring_turn_start = None
            self.special_ring_wait_start = None
            self.reset_ring_traverse()

    def check_real_odom_ready(self, now):
        if self.dry_run or self.latest_odom is not None:
            return True
        if (now - self.start_time).to_sec() > 2.0 and not self.odom_error_reported:
            self.state = "ERROR"
            self.odom_error_reported = True
            rospy.logerr(
                "[craic2026_runner] no odom on %s within 2s; refusing real publish",
                self.odom_topic,
            )
        return False

    def run_auto_takeoff(self, now):
        if not self.auto_takeoff or self.takeoff_done:
            self.state = "POST_TAKEOFF_CLIMB" if self.needs_post_takeoff_climb() else "RUNNING"
            return

        if self.dry_run:
            rospy.logwarn("[craic2026_runner] DRY RUN auto_takeoff: simulate TAKEOFF complete.")
            self.takeoff_done = True
            self.state = "POST_TAKEOFF_CLIMB" if self.needs_post_takeoff_climb() else "RUNNING"
            return

        if self.latest_odom is None:
            return

        if self.takeoff_command_start is None:
            self.takeoff_command_start = now
            self.takeoff_start_z = self.latest_odom.pose.pose.position.z
            rospy.logwarn(
                "[craic2026_runner] publishing TAKEOFF to %s; wait z >= %.3f",
                self.takeoff_land_topic,
                self.takeoff_start_z + self.auto_takeoff_height,
            )

        elapsed = (now - self.takeoff_command_start).to_sec()
        if elapsed <= self.takeoff_command_duration:
            self.publish_takeoff_command()

        current_z = self.latest_odom.pose.pose.position.z
        if current_z >= self.takeoff_start_z + self.auto_takeoff_height - self.accept_radius_z:
            self.takeoff_done = True
            self.state = "POST_TAKEOFF_CLIMB" if self.needs_post_takeoff_climb() else "RUNNING"
            rospy.logwarn(
                "[craic2026_runner] auto_takeoff complete at z=%.3f; next state=%s.",
                current_z,
                self.state,
            )
            return

        if elapsed > self.auto_takeoff_timeout:
            self.state = "ERROR"
            rospy.logerr(
                "[craic2026_runner] auto_takeoff timeout: z=%.3f target=%.3f",
                current_z,
                self.takeoff_start_z + self.auto_takeoff_height,
            )

    def needs_post_takeoff_climb(self):
        return self.post_takeoff_climb_enabled and not self.post_takeoff_climb_done

    def run_post_takeoff_climb(self, now):
        if self.dry_run:
            self.post_takeoff_climb_done = True
            self.state = "RUNNING"
            rospy.logwarn("[craic2026_runner] DRY RUN post_takeoff_climb: simulate complete.")
            return

        if self.latest_odom is None:
            return

        if self.post_takeoff_climb_start is None:
            self.post_takeoff_climb_start = now
            rospy.logwarn(
                "[craic2026_runner] post_takeoff_climb: publish direct point %s before mission.",
                self.post_takeoff_climb_point,
            )

        self.publish_direct_point(self.post_takeoff_climb_point, None)
        if self.reached_point(self.post_takeoff_climb_point):
            self.post_takeoff_climb_done = True
            self.state = "RUNNING"
            rospy.logwarn(
                "[craic2026_runner] post_takeoff_climb complete at z=%.3f; starting mission waypoints.",
                self.latest_odom.pose.pose.position.z,
            )
            return

        elapsed = (now - self.post_takeoff_climb_start).to_sec()
        if elapsed > self.post_takeoff_climb_timeout:
            self.state = "ERROR"
            rospy.logerr(
                "[craic2026_runner] post_takeoff_climb timeout: current=%s target=%s",
                self.current_position(),
                self.post_takeoff_climb_point,
            )
            return

        rospy.loginfo_throttle(
            0.5,
            "[craic2026_runner] climbing after takeoff: current=%s target=%s elapsed=%.1fs.",
            self.current_position(),
            self.post_takeoff_climb_point,
            elapsed,
        )

    def run_auto_land(self, now):
        if self.dry_run:
            self.land_done = True
            self.state = "COMPLETE"
            rospy.logwarn("[craic2026_runner] DRY RUN auto_land: simulate LAND command complete.")
            return

        if self.land_command_start is None:
            self.land_command_start = now
            rospy.logwarn(
                "[craic2026_runner] publishing LAND to %s for %.1fs; no auto-disarm by runner.",
                self.takeoff_land_topic,
                self.land_command_duration,
            )

        elapsed = (now - self.land_command_start).to_sec()
        if elapsed <= self.land_command_duration:
            self.publish_land_command()
            return

        self.land_done = True
        self.state = "COMPLETE"
        rospy.logwarn("[craic2026_runner] LAND command window complete. Runner will not publish more goals.")

    def tick(self, event):
        now = rospy.Time.now()
        self.publish_status(now)

        if self.state in ("ABORTED", "ERROR"):
            return
        if self.state == "COMPLETE":
            if self.land_done:
                return
            if self.last_target is not None:
                self.publish_target(self.last_target)
            return
        if not self.check_real_odom_ready(now):
            return

        if self.state == "AUTO_TAKEOFF":
            self.run_auto_takeoff(now)
            return
        if self.state == "POST_TAKEOFF_CLIMB":
            self.run_post_takeoff_climb(now)
            return
        if self.state == "AUTO_LAND":
            self.run_auto_land(now)
            return

        wp = self.current_waypoint()
        if wp is None:
            self.state = "COMPLETE"
            return

        if self.state == "PAUSED":
            if self.last_target is not None:
                self.publish_target(self.last_target)
            return
        if self.state != "RUNNING":
            return

        task = self.current_task()
        self.update_super_ring_radius_for_task(task)
        if (
            self.is_special_drop_task(task)
            and self.special_detector_enabled
            and not self.special_drop_done
        ):
            self.start_special_detector_process()
        if self.qr_phase is not None:
            self.run_qr_sequence(now)
            return
        if self.scan_phase is not None:
            self.run_scan_sequence(now)
            return
        if self.special_drop_phase is not None:
            self.run_special_drop_sequence(now)
            return
        if self.run_ring_sequence(now):
            return

        self.publish_target(wp)
        if (
            task is not None
            and self.should_prelaunch_scan_after_task(task)
            and self.is_final_waypoint()
            and self.final_hold_start is not None
        ):
            elapsed = (now - self.final_hold_start).to_sec()
            if elapsed >= self.pre_scan_launch_hold_time:
                rospy.logwarn(
                    "[craic2026_runner] scan infer prelaunch hold complete at %s; enter mission3.",
                    wp.get("id"),
                )
                self.advance_waypoint_or_task()
            return

        if not self.reached(wp):
            self.final_hold_start = None
            return

        if self.is_qr_read_waypoint(wp):
            self.begin_qr_sequence(now, wp)
            return

        if self.is_scan_task(task):
            self.begin_scan_sequence(now, wp)
            return

        if (
            self.is_special_drop_task(task)
            and self.is_final_waypoint()
            and not self.special_drop_done
        ):
            self.begin_special_drop_sequence(now, wp)
            return

        if not self.is_final_waypoint():
            self.advance_waypoint_or_task()
            return

        if self.should_auto_land_task(task):
            rospy.logwarn(
                "[craic2026_runner] landing waypoint %s reached; triggering px4ctrl LAND immediately.",
                wp.get("id"),
            )
            self.state = "AUTO_LAND"
            self.land_command_start = None
            return

        if self.should_prelaunch_scan_after_task(task):
            if self.final_hold_start is None:
                self.final_hold_start = now
                rospy.logwarn(
                    "[craic2026_runner] final waypoint %s reached; start scan infer and hold %.1fs before mission3.",
                    wp.get("id"),
                    self.pre_scan_launch_hold_time,
                )
                self.start_scan_process()
            elapsed = (now - self.final_hold_start).to_sec()
            if elapsed >= self.pre_scan_launch_hold_time:
                rospy.logwarn(
                    "[craic2026_runner] scan infer prelaunch hold complete at %s; enter mission3.",
                    wp.get("id"),
                )
                self.advance_waypoint_or_task()
            return

        hold_time = float(wp.get("hold_time", 2.0))
        if self.final_hold_start is None:
            self.final_hold_start = now
            rospy.loginfo(
                "[craic2026_runner] final waypoint %s reached; holding %.1fs",
                wp.get("id"),
                hold_time,
            )
        elapsed_real = (now - self.final_hold_start).to_sec()
        elapsed = hold_time if self.dry_run else elapsed_real
        if elapsed >= hold_time:
            self.advance_waypoint_or_task()

    def publish_status(self, now):
        task = self.current_task()
        wp = self.current_waypoint()
        required = float(wp.get("hold_time", 0.0)) if wp else 0.0
        if task is not None and wp is not None and self.should_prelaunch_scan_after_task(task) and self.is_final_waypoint():
            required = self.pre_scan_launch_hold_time
        if wp is not None and self.is_qr_read_waypoint(wp):
            required = self.qr_wait_timeout
        elapsed = 0.0
        if self.final_hold_start is not None:
            elapsed = (now - self.final_hold_start).to_sec()
        if self.dry_run and self.final_hold_start is not None:
            elapsed = required
        status = {
            "current_task_id": task["task_id"] if task else None,
            "current_task_name": task["name"] if task else None,
            "current_waypoint_name": wp.get("id") if wp else None,
            "current_waypoint_index": self.current_waypoint_idx if wp else None,
            "is_final_waypoint": self.is_final_waypoint(),
            "final_hold_elapsed": elapsed,
            "final_hold_required": required if self.is_final_waypoint() else 0.0,
            "state": self.state,
            "target_position": wp.get("point") if wp else None,
            "current_position": self.current_position(),
            "dry_run": self.dry_run,
            "output_mode": self.output_mode,
            "goal_topic": self.goal_topic,
            "check_z_reached": self.check_z_reached,
            "auto_takeoff": self.auto_takeoff,
            "takeoff_done": self.takeoff_done,
            "takeoff_height": self.auto_takeoff_height,
            "post_takeoff_climb_enabled": self.post_takeoff_climb_enabled,
            "post_takeoff_climb_done": self.post_takeoff_climb_done,
            "post_takeoff_climb_point": self.post_takeoff_climb_point,
            "auto_land": self.auto_land,
            "land_done": self.land_done,
            "scan_phase": self.scan_phase,
            "scan_detect_z": self.scan_detect_z,
            "scan_drop_z": self.scan_drop_z,
            "scan_descent_z": self.scan_descent_z,
            "scan_cruise_z": self.scan_cruise_z,
            "scan_result_timeout": self.scan_result_timeout,
            "scan_last_result": self.scan_last_result,
            "qr_phase": self.qr_phase,
            "qr_landing_side": self.qr_landing_side,
            "qr_payload_topic": self.qr_payload_topic,
            "qr_last_payload": self.qr_last_payload,
            "qr_targets_from_payload": self.qr_targets_from_payload,
            "qr_payload_confirmed": self.qr_payload_confirmed,
            "qr_last_update_time": self.qr_last_update_time.to_sec() if self.qr_last_update_time else None,
            "drop_enabled": self.drop_enabled,
            "drop_target_classes": self.drop_target_classes,
            "dropped_target_classes": sorted(self.dropped_target_classes),
            "drop_release_count": self.drop_release_count,
            "drop_stable_class": self.drop_stable_class,
            "drop_stable_count_seen": self.drop_stable_count_seen,
            "drop_non_target_confirm_rounds": self.drop_non_target_confirm_rounds,
            "special_drop_phase": self.special_drop_phase,
            "special_detect_z": self.special_detect_z,
            "special_detector_enabled": self.special_detector_enabled,
            "special_detector_started": self.special_detector_started,
            "special_camera_topic": self.special_camera_topic,
            "special_result_topic": self.special_result_topic,
            "special_detect_timeout": self.special_detect_timeout,
            "special_timeout_release_fallback": self.special_timeout_release_fallback,
            "special_last_result": self.special_last_result,
            "special_confirm_seen": self.special_confirm_seen,
            "special_confirm_count": self.special_confirm_count,
            "ring_detector_after_special_enabled": self.ring_detector_after_special_enabled,
            "ring_detector_started": self.ring_detector_started,
            "ring_detector_wait_time": self.ring_detector_wait_time,
            "ring_detector_turn_yaw": self.ring_detector_turn_yaw,
            "ring_phase": self.ring_phase,
            "ring_center_topic": self.ring_center_topic,
            "ring_center_odom": self.ring_center_odom,
            "ring_center_age": (
                (now - self.ring_center_time).to_sec()
                if self.ring_center_time is not None
                else None
            ),
            "ring_targets": self.ring_targets,
            "super_ring_radius_service": self.super_ring_radius_service,
            "super_ring_radius_enabled": self.super_ring_radius_enabled,
            "super_ring_radius_target": self.super_ring_radius_target,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


if __name__ == "__main__":
    rospy.init_node("craic2026_hold2s_mission_runner")
    try:
        Craic2026Hold2sMissionRunner()
        rospy.spin()
    except Exception as exc:
        rospy.logerr("[craic2026_runner] failed to start: %s", exc)
        raise
