#!/usr/bin/env python3
import json
import math
import time

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from ultralytics import YOLO


class RingDetectorNode:
    def __init__(self):
        self.camera_topic = rospy.get_param("~camera_topic", "/odin1/image")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_slam")
        self.odom_topic = rospy.get_param("~odom_topic", "/odin1/odometry_highfreq")
        self.wiwc_topic = rospy.get_param("~wiwc_topic", "/odin1/wiwc")
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/n150/CRAIC_ws/src/camera/ring/n150_benchmark_package/"
            "deployment_n150/openvino_model/best_openvino_model",
        )
        self.calib_path = rospy.get_param(
            "~calib_path", "/home/n150/CRAIC_ws/src/odin_ros_driver/config/calib.yaml"
        )
        self.imgsz = int(rospy.get_param("~imgsz", 512))
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.max_fps = float(rospy.get_param("~max_fps", 10.0))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.publish_center_odom = bool(rospy.get_param("~publish_center_odom", True))
        self.center_estimation_mode = str(
            rospy.get_param("~center_estimation_mode", "known_height")
        ).strip()
        self.center_pixel_mode = str(
            rospy.get_param("~center_pixel_mode", "mask_centroid")
        ).strip()
        self.center_pixel_offset_uv = self.parse_float_list_param(
            "~center_pixel_offset_uv", [0.0, 0.0], 2
        )
        self.use_known_center_height = bool(
            rospy.get_param("~use_known_center_height", True)
        )
        self.known_center_z = float(rospy.get_param("~known_center_z", 1.3))
        self.known_height_ray_elevation_bias_deg = float(
            rospy.get_param("~known_height_ray_elevation_bias_deg", 0.0)
        )
        self.known_height_horizontal_scale = float(
            rospy.get_param("~known_height_horizontal_scale", 1.0)
        )
        self.min_known_height_delta = float(
            rospy.get_param("~min_known_height_delta", 0.2)
        )
        self.ring_outer_diameter_m = float(
            rospy.get_param("~ring_outer_diameter_m", 1.2)
        )
        self.ring_inner_diameter_m = float(
            rospy.get_param("~ring_inner_diameter_m", 0.9)
        )
        self.ring_size_diameter_source = str(
            rospy.get_param("~ring_size_diameter_source", "outer")
        ).strip()
        self.ring_size_shape_source = str(
            rospy.get_param("~ring_size_shape_source", "min_area_rect")
        ).strip()
        self.ring_size_alpha_policy = str(
            rospy.get_param("~ring_size_alpha_policy", "max")
        ).strip()
        self.ring_size_depth_scale = float(
            rospy.get_param("~ring_size_depth_scale", 1.0)
        )
        self.latch_center_enabled = bool(
            rospy.get_param("~latch_center_enabled", True)
        )
        self.latch_min_samples = int(rospy.get_param("~latch_min_samples", 8))
        self.latch_max_std_m = float(rospy.get_param("~latch_max_std_m", 0.12))
        self.latch_max_samples = int(rospy.get_param("~latch_max_samples", 20))
        self.camera_extrinsic_mode = str(
            rospy.get_param("~camera_extrinsic_mode", "odin_calib")
        ).strip()
        self.camera_xyz_offset_base = self.parse_xyz_param(
            "~camera_xyz_offset_base", [0.0, 0.0, 0.0]
        )
        self.manual_camera_xyz_base = self.parse_xyz_param(
            "~manual_camera_xyz_base", [0.0, 0.0, 0.0]
        )
        self.manual_camera_rpy_deg = self.parse_xyz_param(
            "~manual_camera_rpy_deg", [0.0, 0.0, 0.0]
        )
        self.min_ring_points = int(rospy.get_param("~min_ring_points", 30))
        self.max_cloud_age = float(rospy.get_param("~max_cloud_age", 3.0))
        self.max_odom_age = float(rospy.get_param("~max_odom_age", 5.0))
        self.projection_mask_dilate_px = int(
            rospy.get_param("~projection_mask_dilate_px", 8)
        )
        self.max_cloud_points = int(rospy.get_param("~max_cloud_points", 60000))
        self.plane_inlier_threshold = float(
            rospy.get_param("~plane_inlier_threshold", 0.08)
        )

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="segment")
        self.load_calibration(self.calib_path)
        self.last_process_time = 0.0
        self.logged_encoding = False
        self.latest_cloud = None
        self.latest_odom = None
        self.latest_wiwc = None
        self.latest_tcl = None
        self.latest_til = None
        self.latest_cloud_receive_time = None
        self.latest_odom_receive_time = None
        self.latest_wiwc_receive_time = None
        self.center_latch_samples = []
        self.latched_center_odom = None

        self.center_pub = rospy.Publisher("~center_px", PointStamped, queue_size=10)
        self.center_odom_pub = rospy.Publisher(
            "~center_odom", PointStamped, queue_size=10
        )
        self.center_odom_latched_pub = rospy.Publisher(
            "~center_odom_latched", PointStamped, queue_size=1, latch=True
        )
        self.detection_pub = rospy.Publisher("~detection", String, queue_size=10)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True
        )
        self.wiwc_sub = rospy.Subscriber(
            self.wiwc_topic, Odometry, self.wiwc_callback, queue_size=1, tcp_nodelay=True
        )

        rospy.loginfo("[ring_detector] model: %s", self.model_path)
        rospy.loginfo("[ring_detector] image: %s", self.camera_topic)
        rospy.loginfo("[ring_detector] cloud: %s", self.cloud_topic)
        rospy.loginfo("[ring_detector] odom:  %s", self.odom_topic)
        rospy.loginfo("[ring_detector] wiwc:  %s", self.wiwc_topic)
        rospy.loginfo("[ring_detector] calib: %s", self.calib_path)
        rospy.loginfo(
            "[ring_detector] imgsz=%d conf=%.2f device=%s max_fps=%.1f min_points=%d estimation=%s known_z=%s %.2f elev_bias=%.2f xy_scale=%.3f center_mode=%s center_offset_uv=%s",
            self.imgsz,
            self.conf,
            self.device,
            self.max_fps,
            self.min_ring_points,
            self.center_estimation_mode,
            self.use_known_center_height,
            self.known_center_z,
            self.known_height_ray_elevation_bias_deg,
            self.known_height_horizontal_scale,
            self.center_pixel_mode,
            np.round(self.center_pixel_offset_uv, 2).tolist(),
        )
        rospy.loginfo(
            "[ring_detector] camera_extrinsic_mode=%s xyz_offset_base=%s manual_xyz_base=%s manual_rpy_deg=%s",
            self.camera_extrinsic_mode,
            np.round(self.camera_xyz_offset_base, 4).tolist(),
            np.round(self.manual_camera_xyz_base, 4).tolist(),
            np.round(self.manual_camera_rpy_deg, 4).tolist(),
        )
        rospy.loginfo(
            "[ring_detector] ring_size outer=%.3f inner=%.3f source=%s shape=%s alpha=%s scale=%.3f latch=%s min_samples=%d max_std=%.3f",
            self.ring_outer_diameter_m,
            self.ring_inner_diameter_m,
            self.ring_size_diameter_source,
            self.ring_size_shape_source,
            self.ring_size_alpha_policy,
            self.ring_size_depth_scale,
            self.latch_center_enabled,
            self.latch_min_samples,
            self.latch_max_std_m,
        )

    def cloud_callback(self, msg):
        self.latest_cloud = msg
        self.latest_cloud_receive_time = rospy.Time.now()

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.latest_odom_receive_time = rospy.Time.now()

    def wiwc_callback(self, msg):
        tcl = np.asarray(msg.pose.covariance[:16], dtype=np.float64).reshape((4, 4))
        til = np.asarray(msg.twist.covariance[:16], dtype=np.float64).reshape((4, 4))
        if self.valid_transform(tcl) and self.valid_transform(til):
            self.latest_wiwc = msg
            self.latest_tcl = tcl
            self.latest_til = til
            self.latest_wiwc_receive_time = rospy.Time.now()

    def image_callback(self, msg):
        now = time.time()
        if self.max_fps > 0.0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        frame = self.ros_image_to_bgr(msg)
        if frame is None:
            return
        frame_context = self.make_frame_context()

        t0 = time.perf_counter()
        try:
            results = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                device=self.device,
                conf=self.conf,
                verbose=False,
            )
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[ring_detector] inference failed: %s", exc)
            return

        latency_ms = (time.perf_counter() - t0) * 1000.0
        best = self.pick_best_detection(results[0], frame.shape)
        detection_msg = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "detected": best is not None,
            "latency_ms": round(latency_ms, 2),
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
        }

        if best is not None:
            public_best = dict(best)
            public_best.pop("_mask", None)
            detection_msg.update(public_best)
            center_msg = PointStamped()
            center_msg.header = msg.header
            center_msg.point.x = float(best["center_u"])
            center_msg.point.y = float(best["center_v"])
            center_msg.point.z = float(best["confidence"])
            self.center_pub.publish(center_msg)

            odom_result = self.estimate_center_odom(
                best, frame.shape, msg.header.stamp, frame_context
            )
            detection_msg.update(odom_result)
            if (
                self.publish_center_odom
                and odom_result.get("center_odom_xyz") is not None
            ):
                center_odom_msg = PointStamped()
                center_odom_msg.header.stamp = msg.header.stamp
                center_odom_msg.header.frame_id = "odom"
                x, y, z = odom_result["center_odom_xyz"]
                center_odom_msg.point.x = x
                center_odom_msg.point.y = y
                center_odom_msg.point.z = z
                self.center_odom_pub.publish(center_odom_msg)
                latch_result = self.update_center_latch(
                    np.array([x, y, z], dtype=np.float64), msg.header.stamp
                )
                detection_msg.update(latch_result)
            else:
                detection_msg.update(self.current_latch_status())

            rospy.loginfo_throttle(
                0.5,
                "[ring_detector] ring center=(%.1f, %.1f) status=%s conf=%.2f latency=%.1fms",
                best["center_u"],
                best["center_v"],
                detection_msg.get("center_status", "unknown"),
                best["confidence"],
                latency_ms,
            )
        else:
            detection_msg.update(
                {
                    "center_status": "no_detection",
                    "center_odom_xyz": None,
                    "point_count": 0,
                    "range_m": None,
                }
            )
            rospy.loginfo_throttle(
                1.0, "[ring_detector] no ring latency=%.1fms", latency_ms
            )

        self.detection_pub.publish(String(data=json.dumps(detection_msg, sort_keys=True)))

        if self.publish_debug_image and self.debug_pub.get_num_connections() > 0:
            annotated = self.draw_debug(frame, best)
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            )

    def ros_image_to_bgr(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[ring_detector] cv_bridge failed: %s", exc)
            return None

        encoding = (msg.encoding or "").lower()
        if not self.logged_encoding:
            rospy.loginfo(
                "[ring_detector] input encoding=%s shape=%s",
                msg.encoding,
                getattr(img, "shape", None),
            )
            self.logged_encoding = True

        if encoding in ("rgb8", "rgb16"):
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding in ("rgba8", "rgba16"):
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding in ("bgra8", "bgra16"):
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if encoding in ("bgr8", "bgr16"):
            return img
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def pick_best_detection(self, result, image_shape):
        if result.boxes is None or len(result.boxes) == 0:
            return None

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        best_idx = int(np.argmax(confs))
        x1, y1, x2, y2 = boxes[best_idx]
        bbox_center_u = (x1 + x2) * 0.5
        bbox_center_v = (y1 + y2) * 0.5
        mask_center_u = bbox_center_u
        mask_center_v = bbox_center_v
        center_u = bbox_center_u
        center_v = bbox_center_v
        area_px = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        ring_mask = None
        min_area_rect_center = [bbox_center_u, bbox_center_v]
        min_area_rect_size = [max(0.0, x2 - x1), max(0.0, y2 - y1)]
        min_area_rect_angle = 0.0
        min_circle_center = [bbox_center_u, bbox_center_v]
        min_circle_radius = 0.5 * max(max(0.0, x2 - x1), max(0.0, y2 - y1))

        if result.masks is not None and result.masks.xy is not None:
            polygons = result.masks.xy
            if best_idx < len(polygons) and len(polygons[best_idx]) >= 3:
                contour = np.asarray(polygons[best_idx], dtype=np.float32)
                moments = cv2.moments(contour)
                if abs(moments["m00"]) > 1e-6:
                    mask_center_u = moments["m10"] / moments["m00"]
                    mask_center_v = moments["m01"] / moments["m00"]
                    area_px = cv2.contourArea(contour)
                ring_mask = self.mask_from_contour(contour, image_shape[:2])
                rect = cv2.minAreaRect(contour)
                min_area_rect_center = [float(rect[0][0]), float(rect[0][1])]
                min_area_rect_size = [float(rect[1][0]), float(rect[1][1])]
                min_area_rect_angle = float(rect[2])
                circle_center, circle_radius = cv2.minEnclosingCircle(contour)
                min_circle_center = [float(circle_center[0]), float(circle_center[1])]
                min_circle_radius = float(circle_radius)

        height, width = image_shape[:2]
        if ring_mask is None:
            ring_mask = self.mask_from_box((x1, y1, x2, y2), (height, width))
        if self.center_pixel_mode == "mask_centroid":
            center_u = mask_center_u
            center_v = mask_center_v
        elif self.center_pixel_mode == "bbox_center":
            center_u = bbox_center_u
            center_v = bbox_center_v
        else:
            rospy.logwarn_throttle(
                2.0,
                "[ring_detector] unknown center_pixel_mode=%s, using bbox_center",
                self.center_pixel_mode,
            )
            center_u = bbox_center_u
            center_v = bbox_center_v
        center_u += float(self.center_pixel_offset_uv[0])
        center_v += float(self.center_pixel_offset_uv[1])
        return {
            "center_u": round(float(center_u), 2),
            "center_v": round(float(center_v), 2),
            "bbox_center_uv": [
                round(float(bbox_center_u), 2),
                round(float(bbox_center_v), 2),
            ],
            "mask_center_uv": [
                round(float(mask_center_u), 2),
                round(float(mask_center_v), 2),
            ],
            "center_pixel_mode": self.center_pixel_mode,
            "center_pixel_offset_uv": [
                round(float(v), 2) for v in self.center_pixel_offset_uv
            ],
            "norm_x": round(float((center_u / width) * 2.0 - 1.0), 4),
            "norm_y": round(float((center_v / height) * 2.0 - 1.0), 4),
            "confidence": round(float(confs[best_idx]), 4),
            "area_px": round(float(area_px), 1),
            "bbox_xyxy": [round(float(v), 1) for v in (x1, y1, x2, y2)],
            "min_area_rect_center_uv": [
                round(float(v), 2) for v in min_area_rect_center
            ],
            "min_area_rect_size_px": [
                round(float(v), 2) for v in min_area_rect_size
            ],
            "min_area_rect_angle_deg": round(float(min_area_rect_angle), 2),
            "min_circle_center_uv": [
                round(float(v), 2) for v in min_circle_center
            ],
            "min_circle_radius_px": round(float(min_circle_radius), 2),
            "_mask": ring_mask,
        }

    def mask_from_contour(self, contour, image_shape):
        height, width = image_shape
        mask = np.zeros((height, width), dtype=np.uint8)
        contour_i = np.round(contour).astype(np.int32)
        cv2.fillPoly(mask, [contour_i], 255)
        return mask

    def mask_from_box(self, box, image_shape):
        height, width = image_shape
        x1, y1, x2, y2 = [int(round(v)) for v in box]
        x1 = max(0, min(width - 1, x1))
        x2 = max(0, min(width - 1, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(0, min(height - 1, y2))
        mask = np.zeros((height, width), dtype=np.uint8)
        if x2 > x1 and y2 > y1:
            mask[y1:y2 + 1, x1:x2 + 1] = 255
        return mask

    def estimate_center_odom(self, best, image_shape, image_stamp, context):
        base = {
            "center_status": "unavailable",
            "center_odom_xyz": None,
            "point_count": 0,
            "range_m": None,
            "plane_normal_odom": None,
        }
        if context["odom"] is None:
            base["center_status"] = "no_odom"
            return base

        odom_age, odom_age_source, odom_stamp_delta = self.message_age(
            image_stamp, context["odom"].header.stamp, context["odom_receive_time"]
        )
        base["odom_age_s"] = round(odom_age, 3)
        base["odom_age_source"] = odom_age_source
        base["odom_stamp_delta_s"] = round(odom_stamp_delta, 3)
        if context["wiwc"] is not None:
            wiwc_age, wiwc_age_source, wiwc_stamp_delta = self.message_age(
                image_stamp, context["wiwc"].header.stamp, context["wiwc_receive_time"]
            )
            base["wiwc_age_s"] = round(wiwc_age, 3)
            base["wiwc_age_source"] = wiwc_age_source
            base["wiwc_stamp_delta_s"] = round(wiwc_stamp_delta, 3)
        if odom_age > self.max_odom_age:
            base["center_status"] = "odom_stale"
            return base

        base["center_estimation_mode"] = self.center_estimation_mode
        base["camera_extrinsic_mode"] = self.camera_extrinsic_mode
        base["camera_xyz_offset_base"] = [
            round(float(v), 4) for v in self.camera_xyz_offset_base
        ]

        if self.center_estimation_mode == "ring_size":
            ring_size_result = self.estimate_center_from_ring_size(best, context)
            base.update(ring_size_result)
            return base

        if self.center_estimation_mode == "known_height":
            raw_center_odom, raw_height_debug = self.intersect_known_height(
                best, self.known_center_z, context, elevation_bias_rad=0.0
            )
            center_odom, height_debug = self.intersect_known_height(
                best,
                self.known_center_z,
                context,
                elevation_bias_rad=math.radians(
                    self.known_height_ray_elevation_bias_deg
                ),
            )
            base.update(height_debug)
            if (
                abs(height_debug.get("height_delta_m", 999.0))
                < self.min_known_height_delta
            ):
                base["center_status"] = "known_height_degenerate"
                base["height_plane_status"] = "camera_too_close_to_height_plane"
                base["known_center_z"] = round(float(self.known_center_z), 4)
                base["min_known_height_delta"] = round(
                    float(self.min_known_height_delta), 4
                )
                return base
            if center_odom is not None:
                center_odom = self.apply_known_height_xy_scale(center_odom, context)
                base["center_status"] = "known_height_plane"
                base["center_odom_xyz"] = [round(float(v), 4) for v in center_odom]
                if raw_center_odom is not None:
                    base["raw_center_odom_xyz"] = [
                        round(float(v), 4) for v in raw_center_odom
                    ]
                base["range_m"] = round(
                    float(np.linalg.norm(center_odom - self.odom_position(context))), 4
                )
                base["known_center_z"] = round(float(self.known_center_z), 4)
                base["known_height_ray_elevation_bias_deg"] = round(
                    float(self.known_height_ray_elevation_bias_deg), 4
                )
                base["known_height_horizontal_scale"] = round(
                    float(self.known_height_horizontal_scale), 4
                )
                base["camera_origin_odom_xyz"] = [
                    round(float(v), 4) for v in self.t_odom_cam(context)[:3, 3]
                ]
                base["point_count"] = 0
                return base
            base["height_plane_status"] = height_debug.get(
                "height_plane_status", "ray_parallel_or_behind"
            )

        if self.center_estimation_mode not in ("cloud_plane", "known_height"):
            base["center_status"] = "unknown_estimation_mode"
            return base

        if context["cloud"] is None:
            base["center_status"] = "no_cloud"
            return base

        cloud_age, cloud_age_source, cloud_stamp_delta = self.message_age(
            image_stamp, context["cloud"].header.stamp, context["cloud_receive_time"]
        )
        base["cloud_age_s"] = round(cloud_age, 3)
        base["cloud_age_source"] = cloud_age_source
        base["cloud_stamp_delta_s"] = round(cloud_stamp_delta, 3)
        if cloud_age > self.max_cloud_age:
            base["center_status"] = "cloud_stale"
            return base

        selected_odom, selected_cam = self.select_ring_cloud_points(
            best, image_shape, context
        )
        base["point_count"] = int(len(selected_odom))
        if len(selected_odom) < self.min_ring_points:
            base["center_status"] = "insufficient_points"
            return base

        centroid, normal, inlier_count = self.fit_plane(selected_odom)
        base["plane_normal_odom"] = [round(float(v), 4) for v in normal]
        base["plane_inlier_count"] = int(inlier_count)

        center_odom = self.intersect_center_ray(best, centroid, normal, context)
        if center_odom is not None:
            center_odom = np.asarray(center_odom, dtype=np.float64)
            base["center_status"] = "plane_intersection"
        else:
            center_odom = np.median(selected_odom, axis=0)
            base["center_status"] = "centroid_fallback"

        odom_position = self.odom_position(context)
        range_m = float(np.linalg.norm(center_odom - odom_position))
        base["center_odom_xyz"] = [round(float(v), 4) for v in center_odom]
        base["range_m"] = round(range_m, 4)
        base["depth_m"] = round(float(np.median(selected_cam[:, 2])), 4)
        base["camera_extrinsic_mode"] = self.camera_extrinsic_mode
        base["camera_xyz_offset_base"] = [
            round(float(v), 4) for v in self.camera_xyz_offset_base
        ]
        base["camera_origin_odom_xyz"] = [
            round(float(v), 4) for v in self.t_odom_cam(context)[:3, 3]
        ]
        return base

    def estimate_center_from_ring_size(self, best, context):
        x1, y1, x2, y2 = [float(v) for v in best["bbox_xyxy"]]
        center_u = float(best["center_u"])
        center_v = float(best["center_v"])
        bbox_center_u = (x1 + x2) * 0.5
        bbox_center_v = (y1 + y2) * 0.5

        alpha_candidates = self.ring_size_alpha_candidates(best)
        horizontal_alpha = alpha_candidates.get("bbox_horizontal")
        vertical_alpha = alpha_candidates.get("bbox_vertical")
        alphas = [
            a
            for a in alpha_candidates.values()
            if a is not None and math.isfinite(a) and a > math.radians(1.0)
        ]
        result = {
            "center_status": "ring_size_unavailable",
            "center_odom_xyz": None,
            "point_count": 0,
            "range_m": None,
            "ring_outer_diameter_m": round(float(self.ring_outer_diameter_m), 4),
            "ring_inner_diameter_m": round(float(self.ring_inner_diameter_m), 4),
            "ring_size_diameter_source": self.ring_size_diameter_source,
            "ring_size_shape_source": self.ring_size_shape_source,
            "ring_size_alpha_policy": self.ring_size_alpha_policy,
            "ring_size_depth_scale": round(float(self.ring_size_depth_scale), 4),
            "ring_size_alpha_horizontal_deg": (
                round(math.degrees(horizontal_alpha), 4)
                if horizontal_alpha is not None
                else None
            ),
            "ring_size_alpha_vertical_deg": (
                round(math.degrees(vertical_alpha), 4)
                if vertical_alpha is not None
                else None
            ),
            "ring_size_alpha_candidates_deg": {
                key: round(math.degrees(value), 4)
                for key, value in alpha_candidates.items()
                if value is not None and math.isfinite(value)
            },
            "camera_origin_odom_xyz": [
                round(float(v), 4) for v in self.t_odom_cam(context)[:3, 3]
            ],
        }
        if not alphas:
            result["center_status"] = "ring_size_bad_angular_diameter"
            return result

        diameter = self.ring_outer_diameter_m
        if self.ring_size_diameter_source == "inner":
            diameter = self.ring_inner_diameter_m
        elif self.ring_size_diameter_source != "outer":
            result["center_status"] = "ring_size_bad_diameter_source"
            return result
        if diameter <= 1e-6:
            result["center_status"] = "ring_size_bad_diameter"
            return result

        if self.ring_size_alpha_policy == "max":
            alpha = float(np.max(alphas))
        elif self.ring_size_alpha_policy == "median":
            alpha = float(np.median(alphas))
        else:
            result["center_status"] = "ring_size_bad_alpha_policy"
            return result
        range_cam = diameter / (2.0 * math.tan(alpha * 0.5))
        range_cam *= self.ring_size_depth_scale
        if range_cam <= 0.05 or range_cam > 30.0:
            result["center_status"] = "ring_size_range_invalid"
            result["ring_size_range_cam_m"] = round(float(range_cam), 4)
            return result

        ray_cam = self.cam_to_world(center_u, center_v)
        center_cam = ray_cam * range_cam
        t_odom_cam = self.t_odom_cam(context)
        center_odom_h = t_odom_cam.dot(
            np.array([center_cam[0], center_cam[1], center_cam[2], 1.0])
        )
        center_odom = center_odom_h[:3]
        result.update(
            {
                "center_status": "ring_size",
                "center_odom_xyz": [round(float(v), 4) for v in center_odom],
                "range_m": round(
                    float(np.linalg.norm(center_odom - self.odom_position(context))), 4
                ),
                "ring_size_alpha_used_deg": round(math.degrees(alpha), 4),
                "ring_size_range_cam_m": round(float(range_cam), 4),
                "ring_size_center_cam_xyz": [
                    round(float(v), 4) for v in center_cam
                ],
                "center_ray_cam": [round(float(v), 5) for v in ray_cam],
                "center_ray_odom": [
                    round(float(v), 5)
                    for v in t_odom_cam[:3, :3].dot(ray_cam)
                ],
            }
        )
        return result

    def ring_size_alpha_candidates(self, best):
        x1, y1, x2, y2 = [float(v) for v in best["bbox_xyxy"]]
        bbox_center_u = (x1 + x2) * 0.5
        bbox_center_v = (y1 + y2) * 0.5
        candidates = {}
        if self.ring_size_shape_source in ("bbox", "all"):
            candidates["bbox_horizontal"] = self.angular_separation_px(
                x1, bbox_center_v, x2, bbox_center_v
            )
            candidates["bbox_vertical"] = self.angular_separation_px(
                bbox_center_u, y1, bbox_center_u, y2
            )
        if self.ring_size_shape_source in ("min_circle", "all"):
            cu, cv = [float(v) for v in best["min_circle_center_uv"]]
            radius = float(best["min_circle_radius_px"])
            candidates["min_circle_horizontal"] = self.angular_separation_px(
                cu - radius, cv, cu + radius, cv
            )
            candidates["min_circle_vertical"] = self.angular_separation_px(
                cu, cv - radius, cu, cv + radius
            )
        if self.ring_size_shape_source in ("min_area_rect", "all"):
            cu, cv = [float(v) for v in best["min_area_rect_center_uv"]]
            w, h = [float(v) for v in best["min_area_rect_size_px"]]
            angle = math.radians(float(best["min_area_rect_angle_deg"]))
            axis_a = np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)
            axis_b = np.array([-math.sin(angle), math.cos(angle)], dtype=np.float64)
            center = np.array([cu, cv], dtype=np.float64)
            p1 = center - axis_a * (w * 0.5)
            p2 = center + axis_a * (w * 0.5)
            p3 = center - axis_b * (h * 0.5)
            p4 = center + axis_b * (h * 0.5)
            candidates["min_rect_axis_a"] = self.angular_separation_px(
                p1[0], p1[1], p2[0], p2[1]
            )
            candidates["min_rect_axis_b"] = self.angular_separation_px(
                p3[0], p3[1], p4[0], p4[1]
            )
        if not candidates:
            rospy.logwarn_throttle(
                2.0,
                "[ring_detector] unknown ring_size_shape_source=%s, using min_area_rect",
                self.ring_size_shape_source,
            )
            old_source = self.ring_size_shape_source
            self.ring_size_shape_source = "min_area_rect"
            candidates = self.ring_size_alpha_candidates(best)
            self.ring_size_shape_source = old_source
        return candidates

    def angular_separation_px(self, u1, v1, u2, v2):
        r1 = self.cam_to_world(float(u1), float(v1))
        r2 = self.cam_to_world(float(u2), float(v2))
        dot = float(np.dot(r1, r2))
        dot = max(-1.0, min(1.0, dot))
        return math.acos(dot)

    def update_center_latch(self, center_odom, stamp):
        if not self.latch_center_enabled:
            return {"center_latch_status": "disabled", "center_latched_xyz": None}

        if self.latched_center_odom is not None:
            self.publish_latched_center(stamp)
            return {
                "center_latch_status": "latched",
                "center_latched_xyz": [
                    round(float(v), 4) for v in self.latched_center_odom
                ],
            }

        self.center_latch_samples.append(np.asarray(center_odom, dtype=np.float64))
        if len(self.center_latch_samples) > self.latch_max_samples:
            self.center_latch_samples = self.center_latch_samples[-self.latch_max_samples :]

        samples = np.vstack(self.center_latch_samples)
        sample_count = len(samples)
        std_xyz = np.std(samples, axis=0)
        max_std = float(np.max(std_xyz)) if sample_count > 1 else float("inf")
        status = "collecting"
        if sample_count >= self.latch_min_samples and max_std <= self.latch_max_std_m:
            self.latched_center_odom = np.median(samples, axis=0)
            self.publish_latched_center(stamp)
            status = "latched"

        result = {
            "center_latch_status": status,
            "center_latch_sample_count": int(sample_count),
            "center_latch_std_xyz": [
                round(float(v), 4) for v in std_xyz
            ],
            "center_latch_max_std_m": round(float(max_std), 4)
            if math.isfinite(max_std)
            else None,
            "center_latch_required_samples": int(self.latch_min_samples),
            "center_latch_allowed_std_m": round(float(self.latch_max_std_m), 4),
        }
        if self.latched_center_odom is not None:
            result["center_latched_xyz"] = [
                round(float(v), 4) for v in self.latched_center_odom
            ]
        else:
            result["center_latched_xyz"] = None
        return result

    def current_latch_status(self):
        if not self.latch_center_enabled:
            return {"center_latch_status": "disabled", "center_latched_xyz": None}
        if self.latched_center_odom is None:
            return {
                "center_latch_status": "not_latched",
                "center_latch_sample_count": int(len(self.center_latch_samples)),
                "center_latched_xyz": None,
            }
        return {
            "center_latch_status": "latched",
            "center_latched_xyz": [
                round(float(v), 4) for v in self.latched_center_odom
            ],
        }

    def publish_latched_center(self, stamp):
        if self.latched_center_odom is None:
            return
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.point.x = float(self.latched_center_odom[0])
        msg.point.y = float(self.latched_center_odom[1])
        msg.point.z = float(self.latched_center_odom[2])
        self.center_odom_latched_pub.publish(msg)

    def select_ring_cloud_points(self, best, image_shape, context):
        height, width = image_shape[:2]
        mask = best.get("_mask")
        if mask is None:
            return np.empty((0, 3)), np.empty((0, 3))
        if self.projection_mask_dilate_px > 0:
            size = self.projection_mask_dilate_px * 2 + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
            mask = cv2.dilate(mask, kernel)

        t_cam_odom = self.t_cam_odom(context)
        selected_odom = []
        selected_cam = []
        count = 0
        for point in pc2.read_points(
            context["cloud"], field_names=("x", "y", "z"), skip_nans=True
        ):
            count += 1
            if self.max_cloud_points > 0 and count > self.max_cloud_points:
                break
            p_odom = np.array([point[0], point[1], point[2]], dtype=np.float64)
            p_cam_h = t_cam_odom.dot(np.array([p_odom[0], p_odom[1], p_odom[2], 1.0]))
            p_cam = p_cam_h[:3]
            if not np.all(np.isfinite(p_cam)) or p_cam[2] <= 0.05:
                continue
            uv = self.world_to_cam(p_cam)
            if uv is None:
                continue
            u, v = int(round(uv[0])), int(round(uv[1]))
            if u < 0 or u >= width or v < 0 or v >= height:
                continue
            if mask[v, u] == 0:
                continue
            selected_odom.append(p_odom)
            selected_cam.append(p_cam)

        if not selected_odom:
            return np.empty((0, 3)), np.empty((0, 3))
        return np.vstack(selected_odom), np.vstack(selected_cam)

    def fit_plane(self, points):
        points = np.asarray(points, dtype=np.float64)
        centroid = np.median(points, axis=0)
        centered = points - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-9:
            return centroid, np.array([0.0, 0.0, 1.0]), 0
        normal = normal / normal_norm
        distances = np.abs((points - centroid).dot(normal))
        inliers = points[distances <= self.plane_inlier_threshold]
        if len(inliers) >= self.min_ring_points:
            centroid = np.median(inliers, axis=0)
            centered = inliers - centroid
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            normal = vh[-1]
            normal /= max(np.linalg.norm(normal), 1e-9)
            return centroid, normal, len(inliers)
        return centroid, normal, len(inliers)

    def intersect_center_ray(self, best, plane_point, plane_normal, context):
        t_odom_cam = self.t_odom_cam(context)
        ray_cam = self.cam_to_world(float(best["center_u"]), float(best["center_v"]))
        ray_odom = t_odom_cam[:3, :3].dot(ray_cam)
        ray_norm = np.linalg.norm(ray_odom)
        if ray_norm < 1e-9:
            return None
        ray_odom = ray_odom / ray_norm
        origin_odom = t_odom_cam[:3, 3]
        denom = float(np.dot(plane_normal, ray_odom))
        if abs(denom) < 1e-6:
            return None
        dist = float(np.dot(plane_normal, plane_point - origin_odom) / denom)
        if dist <= 0.05 or dist > 30.0:
            return None
        return origin_odom + dist * ray_odom

    def apply_known_height_xy_scale(self, center_odom, context=None):
        scale = self.known_height_horizontal_scale
        if abs(scale - 1.0) < 1e-9:
            return center_odom
        origin = self.odom_position(context)
        corrected = center_odom.copy()
        corrected[:2] = origin[:2] + scale * (center_odom[:2] - origin[:2])
        corrected[2] = center_odom[2]
        return corrected

    def intersect_known_height(self, best, center_z, context, elevation_bias_rad=0.0):
        t_odom_cam = self.t_odom_cam(context)
        ray_cam = self.cam_to_world(float(best["center_u"]), float(best["center_v"]))
        ray_odom = t_odom_cam[:3, :3].dot(ray_cam)
        ray_norm = np.linalg.norm(ray_odom)
        if ray_norm < 1e-9:
            return None, {"height_plane_status": "invalid_ray"}
        ray_odom = ray_odom / ray_norm
        if abs(elevation_bias_rad) > 1e-9:
            ray_odom = self.apply_ray_elevation_bias(ray_odom, elevation_bias_rad)
        origin_odom = t_odom_cam[:3, 3]
        height_delta = float(center_z - origin_odom[2])
        debug = {
            "camera_origin_odom_xyz": [round(float(v), 4) for v in origin_odom],
            "center_ray_odom": [round(float(v), 5) for v in ray_odom],
            "height_delta_m": round(height_delta, 4),
        }
        denom = float(ray_odom[2])
        debug["height_ray_z"] = round(denom, 5)
        if abs(denom) < 1e-5:
            debug["height_plane_status"] = "ray_parallel"
            return None, debug
        dist = float(height_delta / denom)
        debug["height_intersection_dist_m"] = round(dist, 4)
        if dist <= 0.05 or dist > 30.0:
            debug["height_plane_status"] = "intersection_behind_or_too_far"
            return None, debug
        debug["height_plane_status"] = "ok"
        return origin_odom + dist * ray_odom, debug

    def apply_ray_elevation_bias(self, ray_odom, elevation_bias_rad):
        horizontal = np.array([ray_odom[0], ray_odom[1]], dtype=np.float64)
        horizontal_norm = np.linalg.norm(horizontal)
        if horizontal_norm < 1e-9:
            return ray_odom
        horizontal_dir = horizontal / horizontal_norm
        elevation = math.atan2(ray_odom[2], horizontal_norm) + elevation_bias_rad
        corrected = np.array(
            [
                math.cos(elevation) * horizontal_dir[0],
                math.cos(elevation) * horizontal_dir[1],
                math.sin(elevation),
            ],
            dtype=np.float64,
        )
        return corrected / max(np.linalg.norm(corrected), 1e-9)

    def message_age(self, reference_stamp, message_stamp, receive_time):
        stamp_delta = abs((reference_stamp - message_stamp).to_sec())
        if stamp_delta < 1e-3 or stamp_delta <= max(self.max_cloud_age, self.max_odom_age):
            return stamp_delta, "header", stamp_delta
        if receive_time is None:
            return stamp_delta, "header", stamp_delta
        receipt_age = max(0.0, (rospy.Time.now() - receive_time).to_sec())
        return receipt_age, "receipt", stamp_delta

    def load_calibration(self, path):
        with open(path, "r", encoding="utf-8") as handle:
            calib = yaml.safe_load(handle)
        cam = calib["cam_0"]
        self.image_width = int(cam.get("image_width", 1600))
        self.image_height = int(cam.get("image_height", 1296))
        self.fx = float(cam["A11"])
        self.skew = float(cam.get("A12", 0.0))
        self.fy = float(cam["A22"])
        self.cx = float(cam["u0"])
        self.cy = float(cam["v0"])
        self.k2 = float(cam.get("k2", 0.0))
        self.k3 = float(cam.get("k3", 0.0))
        self.k4 = float(cam.get("k4", 0.0))
        self.k5 = float(cam.get("k5", 0.0))
        self.k6 = float(cam.get("k6", 0.0))
        self.k7 = float(cam.get("k7", 0.0))
        self.has_distortion = abs(self.k2) > 1e-7

        tcl = np.asarray(calib["Tcl_0"], dtype=np.float64)
        if tcl.size != 16:
            raise ValueError("Tcl_0 must have 16 values")
        self.fallback_tcl = tcl.reshape((4, 4))
        self.fallback_til = np.eye(4, dtype=np.float64)
        self.fallback_til[0, 3] = -0.02663
        self.fallback_til[1, 3] = 0.03447
        self.fallback_til[2, 3] = 0.02174

    def make_frame_context(self):
        return {
            "cloud": self.latest_cloud,
            "cloud_receive_time": self.latest_cloud_receive_time,
            "odom": self.latest_odom,
            "odom_receive_time": self.latest_odom_receive_time,
            "wiwc": self.latest_wiwc,
            "wiwc_receive_time": self.latest_wiwc_receive_time,
            "tcl": None if self.latest_tcl is None else self.latest_tcl.copy(),
            "til": None if self.latest_til is None else self.latest_til.copy(),
            "snapshot_time": rospy.Time.now(),
        }

    def odom_matrix(self, context=None):
        odom = context["odom"] if context is not None else self.latest_odom
        pose = odom.pose.pose
        q = pose.orientation
        p = pose.position
        rot = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        mat[:3, 3] = [p.x, p.y, p.z]
        return mat

    def odom_position(self, context=None):
        odom = context["odom"] if context is not None else self.latest_odom
        p = odom.pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=np.float64)

    def t_cam_odom(self, context=None):
        return self.t_cam_imu(context).dot(np.linalg.inv(self.odom_matrix(context)))

    def t_odom_cam(self, context=None):
        return self.odom_matrix(context).dot(self.tic_matrix(context))

    def tic_matrix(self, context=None):
        if self.camera_extrinsic_mode == "manual":
            mat = np.eye(4, dtype=np.float64)
            mat[:3, :3] = self.rpy_to_matrix(
                *[math.radians(v) for v in self.manual_camera_rpy_deg]
            )
            mat[:3, 3] = self.manual_camera_xyz_base
            mat[:3, 3] += self.camera_xyz_offset_base
            return mat

        if context is not None:
            tcl = context["tcl"] if context["tcl"] is not None else self.fallback_tcl
            til = context["til"] if context["til"] is not None else self.fallback_til
        else:
            tcl = self.latest_tcl if self.latest_tcl is not None else self.fallback_tcl
            til = self.latest_til if self.latest_til is not None else self.fallback_til
        mat = til.dot(np.linalg.inv(tcl))
        mat[:3, 3] += self.camera_xyz_offset_base
        return mat

    def t_cam_imu(self, context=None):
        return np.linalg.inv(self.tic_matrix(context))

    def valid_transform(self, mat):
        if mat.shape != (4, 4) or not np.all(np.isfinite(mat)):
            return False
        if abs(mat[3, 3]) < 1e-6:
            return False
        if np.linalg.norm(mat - np.eye(4)) < 1e-6:
            return False
        return np.linalg.det(mat[:3, :3]) > 0.1

    def parse_xyz_param(self, name, default):
        return self.parse_float_list_param(name, default, 3)

    def parse_float_list_param(self, name, default, expected_len):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = value.strip()
            if value.startswith("[") and value.endswith("]"):
                value = value[1:-1]
            parts = [p for p in value.replace(",", " ").split() if p]
        else:
            parts = value
        if len(parts) != expected_len:
            raise ValueError(
                "%s must contain exactly %d numbers" % (name, expected_len)
            )
        return np.asarray([float(v) for v in parts], dtype=np.float64)

    def rpy_to_matrix(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return rz.dot(ry).dot(rx)

    def world_to_cam(self, xyz):
        norm = np.linalg.norm(xyz)
        if norm < 1e-9:
            return None
        if not self.has_distortion:
            x = xyz[0] / xyz[2]
            y = xyz[1] / xyz[2]
            return np.array([self.fx * x + self.skew * y + self.cx, self.fy * y + self.cy])

        r = math.sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])
        if r < 1e-9:
            return np.array([self.cx, self.cy])
        theta = math.acos(max(-1.0, min(1.0, xyz[2] / norm)))
        thetad = self.thetad_from_theta(theta)
        scaling = thetad / r
        xd = xyz[0] * scaling
        yd = xyz[1] * scaling
        return np.array([xd * self.fx + yd * self.skew + self.cx, yd * self.fy + self.cy])

    def cam_to_world(self, u, v):
        y = (v - self.cy) / self.fy
        x = (u - self.cx - y * self.skew) / self.fx
        if self.has_distortion:
            thetad = math.sqrt(x * x + y * y)
            theta = thetad
            for _ in range(7):
                theta2 = theta * theta
                theta3 = theta2 * theta
                theta4 = theta3 * theta
                theta5 = theta4 * theta
                theta6 = theta5 * theta
                denom = (
                    1.0
                    + self.k2 * theta
                    + self.k3 * theta2
                    + self.k4 * theta3
                    + self.k5 * theta4
                    + self.k6 * theta5
                    + self.k7 * theta6
                )
                if abs(denom) < 1e-9:
                    break
                theta = thetad / denom
            if thetad > 1e-9:
                scaling = math.tan(theta) / thetad
                x *= scaling
                y *= scaling
        ray = np.array([x, y, 1.0], dtype=np.float64)
        return ray / max(np.linalg.norm(ray), 1e-9)

    def thetad_from_theta(self, theta):
        theta2 = theta * theta
        theta3 = theta2 * theta
        theta4 = theta3 * theta
        theta5 = theta4 * theta
        theta6 = theta5 * theta
        theta7 = theta6 * theta
        return (
            theta
            + self.k2 * theta2
            + self.k3 * theta3
            + self.k4 * theta4
            + self.k5 * theta5
            + self.k6 * theta6
            + self.k7 * theta7
        )

    def quaternion_to_matrix(self, x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def draw_debug(self, frame, best):
        annotated = frame.copy()
        if best is None:
            cv2.putText(
                annotated,
                "NO RING",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )
            return annotated

        x1, y1, x2, y2 = [int(round(v)) for v in best["bbox_xyxy"]]
        u = int(round(best["center_u"]))
        v = int(round(best["center_v"]))
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.drawMarker(
            annotated,
            (u, v),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )
        cv2.putText(
            annotated,
            "ring %.2f" % best["confidence"],
            (max(0, x1), max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        return annotated


def main():
    rospy.init_node("ring_detector")
    RingDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
