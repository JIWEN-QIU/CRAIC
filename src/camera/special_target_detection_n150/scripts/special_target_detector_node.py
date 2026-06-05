#!/usr/bin/env python3
import json
import time
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class SpecialTargetDetectorNode:
    def __init__(self):
        rospy.init_node("special_target_detector", anonymous=False)

        package_dir = Path(__file__).resolve().parents[1]
        default_weights = (
            package_dir
            / "special_target_detection_n150"
            / "weights"
            / "best.pt"
        )
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.weights = Path(rospy.get_param("~weights", str(default_weights))).expanduser()
        self.conf = float(rospy.get_param("~conf", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.7))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.max_fps = float(rospy.get_param("~max_fps", 3.0))
        self.stable_count_required = max(1, int(rospy.get_param("~stable_count", 3)))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))

        if not self.weights.exists():
            raise FileNotFoundError("special target weights not found: %s" % self.weights)

        self.bridge = CvBridge()
        self.model = YOLO(str(self.weights))
        self.last_process_time = rospy.Time(0)
        self.frame_count = 0
        self.present_streak = 0
        self.absent_streak = 0
        self.logged_encoding = False

        self.result_pub = rospy.Publisher("~result", String, queue_size=10)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.camera_topic,
            Image,
            self.image_cb,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.print_status)

        rospy.logwarn(
            "[special_target_detector] weights=%s camera=%s conf=%.2f iou=%.2f device=%s max_fps=%.1f stable_count=%d",
            self.weights,
            self.camera_topic,
            self.conf,
            self.iou,
            self.device,
            self.max_fps,
            self.stable_count_required,
        )

    def should_skip_frame(self):
        if self.max_fps <= 0.0:
            return False
        now = rospy.Time.now()
        min_period = rospy.Duration(1.0 / self.max_fps)
        if self.last_process_time != rospy.Time(0) and now - self.last_process_time < min_period:
            return True
        self.last_process_time = now
        return False

    def ros_image_to_bgr(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        encoding = (msg.encoding or "").lower()
        if not self.logged_encoding:
            rospy.logwarn(
                "[special_target_detector] input encoding=%s shape=%s",
                msg.encoding,
                getattr(image, "shape", None),
            )
            self.logged_encoding = True
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if encoding in ("rgb8", "rgb16"):
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding in ("rgba8", "rgba16"):
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding in ("bgra8", "bgra16"):
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ("bgr8", "bgr16"):
            return image
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def result_to_record(self, result, msg):
        detections = []
        best_conf = 0.0
        names = result.names if hasattr(result, "names") else {}

        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls.item())
                conf = float(box.conf.item())
                if cls_id != 0 or conf < self.conf:
                    continue
                xyxy = [float(value) for value in box.xyxy[0].tolist()]
                detections.append(
                    {
                        "class_id": cls_id,
                        "class_name": names.get(cls_id, "special_target"),
                        "confidence": round(conf, 6),
                        "xyxy": [round(value, 2) for value in xyxy],
                    }
                )
                best_conf = max(best_conf, conf)

        present = len(detections) > 0
        if present:
            self.present_streak += 1
            self.absent_streak = 0
        else:
            self.present_streak = 0
            self.absent_streak += 1

        return {
            "present": present,
            "stable_present": self.present_streak >= self.stable_count_required,
            "present_streak": self.present_streak,
            "absent_streak": self.absent_streak,
            "stable_count_required": self.stable_count_required,
            "best_confidence": round(best_conf, 6),
            "detections": detections,
            "frame_stamp": msg.header.stamp.to_sec() if msg.header.stamp else None,
            "frame_id": msg.header.frame_id,
            "processed_at": time.time(),
        }

    def draw_debug(self, image, record):
        annotated = image.copy()
        color = (0, 255, 0) if record["present"] else (0, 0, 255)
        for det in record["detections"]:
            x1, y1, x2, y2 = [int(round(v)) for v in det["xyxy"]]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = "special %.2f" % det["confidence"]
            cv2.putText(
                annotated,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
        status = "stable" if record["stable_present"] else ("present" if record["present"] else "absent")
        cv2.putText(
            annotated,
            "special_target: %s streak=%d/%d" % (
                status,
                record["present_streak"],
                record["stable_count_required"],
            ),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
        )
        return annotated

    def image_cb(self, msg):
        if self.should_skip_frame():
            return
        try:
            image = self.ros_image_to_bgr(msg)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[special_target_detector] cv_bridge failed: %s", exc)
            return

        self.frame_count += 1
        try:
            results = self.model.predict(
                source=image,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                verbose=False,
            )
            record = self.result_to_record(results[0], msg)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[special_target_detector] inference failed: %s", exc)
            return

        self.result_pub.publish(String(data=json.dumps(record, ensure_ascii=False)))

        if self.publish_debug_image:
            try:
                debug = self.draw_debug(image, record)
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "[special_target_detector] debug publish failed: %s", exc)

        rospy.loginfo_throttle(
            1.0,
            "[special_target_detector] frame=%d present=%s stable=%s best=%.3f streak=%d/%d",
            self.frame_count,
            record["present"],
            record["stable_present"],
            record["best_confidence"],
            record["present_streak"],
            self.stable_count_required,
        )

    def print_status(self, _event):
        if self.frame_count == 0:
            rospy.loginfo("[special_target_detector] waiting for images on %s", self.camera_topic)
        else:
            rospy.loginfo(
                "[special_target_detector] alive frames=%d present_streak=%d absent_streak=%d",
                self.frame_count,
                self.present_streak,
                self.absent_streak,
            )


if __name__ == "__main__":
    try:
        SpecialTargetDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
