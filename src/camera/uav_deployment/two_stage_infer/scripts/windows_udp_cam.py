#!/usr/bin/env python3
"""
QR code reader for the UAV-side ROS camera.

This replaces the old Windows webcam UDP sender.  It subscribes to the image
topic started by two_stage.launch, decodes QR codes locally on the UAV, and
publishes both the raw QR payload and the normalized landing side.
"""
import json
import threading
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class UavQrReader:
    def __init__(self):
        rospy.init_node("uav_qr_reader", anonymous=False)

        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.qr_data_topic = rospy.get_param("~qr_data_topic", "/qr_data")
        self.qr_side_topic = rospy.get_param("~qr_side_topic", "/qr_landing_side")
        self.debug_topic = rospy.get_param("~debug_topic", "/qr_debug_image")
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.resize_width = int(rospy.get_param("~resize_width", 640))
        self.stable_count = max(1, int(rospy.get_param("~stable_count", 3)))
        self.set_global_param = bool(rospy.get_param("~set_global_param", True))
        self.process_rate = float(rospy.get_param("~process_rate", 8.0))
        self.try_preprocess = bool(rospy.get_param("~try_preprocess", True))
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.window_name = str(rospy.get_param("~window_name", "uav_qr_reader"))

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.recent_sides = deque(maxlen=self.stable_count)
        self.last_published_side = None
        self.frame_count = 0
        self.decode_count = 0
        self.skip_count = 0
        self.last_process_time = rospy.Time(0)
        self.logged_encoding = False
        self.gui_lock = threading.Lock()
        self.latest_annotated = self.build_waiting_panel()

        self.qr_data_pub = rospy.Publisher(self.qr_data_topic, String, queue_size=10)
        self.qr_side_pub = rospy.Publisher(self.qr_side_topic, String, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.print_status)
        self.last_payload = ""
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 640, 480)

        rospy.logwarn(
            "[uav_qr_reader] listening on %s, publishing payload=%s side=%s, process_rate=%.1fHz",
            self.camera_topic,
            self.qr_data_topic,
            self.qr_side_topic,
            self.process_rate,
        )

    def image_callback(self, msg):
        self.frame_count += 1
        if self.should_skip_frame():
            self.skip_count += 1
            return

        try:
            frame = self.ros_image_to_bgr(msg)
        except Exception as exc:
            rospy.logerr("[uav_qr_reader] image conversion failed: %s", exc)
            return

        self.decode_count += 1
        frame = self.resize_for_decode(frame)
        payloads, points = self.decode_qr(frame)

        annotated = frame.copy()
        if payloads:
            self.draw_qr_boxes(annotated, points)
        self.draw_status(annotated, payloads)

        for payload in payloads:
            payload = payload.strip()
            if not payload:
                continue
            self.last_payload = payload
            self.qr_data_pub.publish(String(data=payload))

            side = self.normalize_landing_side(payload)
            if side is None:
                rospy.logwarn_throttle(
                    1.0,
                    "[uav_qr_reader] QR payload does not map to left/right: %s",
                    payload,
                )
                continue
            self.handle_side(side, payload)

        if self.publish_debug_image and self.frame_count % 3 == 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "[uav_qr_reader] debug image publish failed: %s", exc)

        if self.show_window:
            with self.gui_lock:
                self.latest_annotated = annotated.copy()

    def should_skip_frame(self):
        if self.process_rate <= 0.0:
            return False
        now = rospy.Time.now()
        min_period = rospy.Duration(1.0 / self.process_rate)
        if self.last_process_time != rospy.Time(0) and now - self.last_process_time < min_period:
            return True
        self.last_process_time = now
        return False

    def ros_image_to_bgr(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        encoding = (msg.encoding or "").lower()
        if not self.logged_encoding:
            rospy.loginfo(
                "[uav_qr_reader] input encoding=%s shape=%s",
                msg.encoding,
                getattr(img, "shape", None),
            )
            self.logged_encoding = True

        if img.ndim == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if encoding in ("rgb8", "rgb16"):
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding in ("rgba8", "rgba16"):
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding in ("bgra8", "bgra16"):
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if encoding in ("bgr8", "bgr16"):
            return img
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def resize_for_decode(self, frame):
        if self.resize_width <= 0:
            return frame
        height, width = frame.shape[:2]
        if width <= self.resize_width:
            return frame
        scale = float(self.resize_width) / float(width)
        return cv2.resize(frame, (self.resize_width, max(1, int(height * scale))))

    def decode_qr(self, frame):
        for candidate in self.build_decode_candidates(frame):
            payloads, points = self.decode_one(candidate)
            if payloads:
                return payloads, points
        return [], None

    def decode_one(self, image):
        payloads = []
        points = None
        try:
            ok, decoded_info, decoded_points, _ = self.detector.detectAndDecodeMulti(image)
            if ok and decoded_info:
                payloads = [text for text in decoded_info if text]
                points = decoded_points
        except Exception:
            payloads = []

        if payloads:
            return payloads, points

        data, single_points, _ = self.detector.detectAndDecode(image)
        if data:
            payloads = [data]
            points = single_points
        return payloads, points

    def build_decode_candidates(self, frame):
        yield frame
        if not self.try_preprocess:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        yield gray

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        yield clahe

        blur = cv2.GaussianBlur(clahe, (3, 3), 0)
        yield cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            5,
        )

        _, otsu = cv2.threshold(clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        yield otsu
        yield cv2.bitwise_not(otsu)

        gamma = self.apply_gamma(frame, 0.65)
        yield gamma

    def apply_gamma(self, frame, gamma):
        inv_gamma = 1.0 / max(gamma, 1e-3)
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(frame, table)

    def normalize_landing_side(self, payload):
        text = payload.strip().lower()
        try:
            obj = json.loads(payload)
            for key in ("side", "landing_side", "qr_landing_side", "land"):
                if isinstance(obj, dict) and key in obj:
                    return self.normalize_landing_side(str(obj[key]))
        except Exception:
            pass

        if text in ("left", "l", "landing_left", "land_left", "左", "左侧", "左边"):
            return "left"
        if text in ("right", "r", "landing_right", "land_right", "右", "右侧", "右边"):
            return "right"
        if "left" in text or "左" in text:
            return "left"
        if "right" in text or "右" in text:
            return "right"
        return None

    def handle_side(self, side, payload):
        self.recent_sides.append(side)
        if len(self.recent_sides) < self.stable_count:
            return
        if any(item != side for item in self.recent_sides):
            return

        self.qr_side_pub.publish(String(data=side))
        if self.set_global_param:
            rospy.set_param("/qr_landing_side", side)
            rospy.set_param("/craic2026/qr_landing_side", side)

        if side != self.last_published_side:
            rospy.logwarn(
                "[uav_qr_reader] QR landing side confirmed: %s (payload=%s)",
                side,
                payload,
            )
            self.last_published_side = side

    def draw_qr_boxes(self, frame, points):
        if points is None:
            return
        if len(points.shape) == 2:
            points = points.reshape(1, -1, 2)
        for qr_points in points:
            pts = qr_points.astype(int)
            for i in range(len(pts)):
                p1 = tuple(pts[i])
                p2 = tuple(pts[(i + 1) % len(pts)])
                cv2.line(frame, p1, p2, (0, 255, 0), 2)

    def draw_status(self, frame, payloads):
        side_text = self.last_published_side if self.last_published_side else "none"
        payload_text = payloads[0] if payloads else self.last_payload
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 64), (20, 20, 20), -1)
        cv2.putText(
            frame,
            "QR side: %s" % side_text,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            "payload: %s" % str(payload_text)[:80],
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )

    def build_waiting_panel(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (0, 0), (640, 76), (20, 20, 20), -1)
        cv2.putText(
            frame,
            "uav_qr_reader",
            (18, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            "waiting for camera frames...",
            (18, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )
        return frame

    def print_status(self, _event):
        rospy.loginfo(
            "[uav_qr_reader] alive frames=%d decoded=%d skipped=%d last_payload=%s last_side=%s",
            self.frame_count,
            self.decode_count,
            self.skip_count,
            self.last_payload,
            self.last_published_side,
        )

    def spin_gui(self):
        if not self.show_window:
            rospy.spin()
            return

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            frame = None
            with self.gui_lock:
                if self.latest_annotated is not None:
                    frame = self.latest_annotated.copy()

            try:
                if frame is not None:
                    cv2.imshow(self.window_name, frame)
                key = cv2.waitKey(1)
                if key & 0xFF == ord("q"):
                    rospy.signal_shutdown("User requested quit")
                    break
            except Exception as exc:
                rospy.logwarn("[uav_qr_reader] OpenCV window failed, disabling: %s", exc)
                self.show_window = False
                break

            rate.sleep()


if __name__ == "__main__":
    node = UavQrReader()
    try:
        node.spin_gui()
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
