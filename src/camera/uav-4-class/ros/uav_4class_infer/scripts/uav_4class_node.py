#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


def add_deploy_src_to_path(config_path: Path) -> None:
    root = config_path.resolve().parent
    src = root / "src"
    if src.exists() and str(src) not in sys.path:
        sys.path.insert(0, str(src))


class UAV4ClassNode:
    def __init__(self):
        rospy.init_node("uav_4class_infer", anonymous=False)
        config_path = Path(rospy.get_param("~config_path", str(Path.home() / "uav-4-class" / "config.json")))
        add_deploy_src_to_path(config_path)

        from pipeline import PipelineConfig, UAV4ClassPipeline, draw_results

        self.draw_results = draw_results
        self.pipeline = UAV4ClassPipeline(PipelineConfig.from_file(config_path))
        self.bridge = CvBridge()
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.show_gui = bool(rospy.get_param("~show_gui", False))
        self.enable_center_roi = bool(rospy.get_param("~enable_center_roi", True))
        self.center_roi_ratio = float(rospy.get_param("~center_roi_ratio", 0.60))

        self.result_pub = rospy.Publisher("~target_info", String, queue_size=10)
        self.coords_pub = rospy.Publisher("~target_coords", Float32MultiArray, queue_size=10)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(
            "uav_4class_infer ready, camera_topic=%s config=%s center_roi=%s ratio=%.2f",
            self.camera_topic,
            config_path,
            self.enable_center_roi,
            self.center_roi_ratio,
        )

    def center_roi_box(self, frame):
        h, w = frame.shape[:2]
        ratio = max(0.05, min(1.0, self.center_roi_ratio))
        roi_w = int(round(w * ratio))
        roi_h = int(round(h * ratio))
        x1 = max(0, (w - roi_w) // 2)
        y1 = max(0, (h - roi_h) // 2)
        return x1, y1, min(w, x1 + roi_w), min(h, y1 + roi_h)

    @staticmethod
    def offset_results(results, dx, dy):
        for item in results:
            item["box"] = [
                float(item["box"][0] + dx),
                float(item["box"][1] + dy),
                float(item["box"][2] + dx),
                float(item["box"][3] + dy),
            ]
            item["center"] = [
                float(item["center"][0] + dx),
                float(item["center"][1] + dy),
            ]
        return results

    @staticmethod
    def draw_roi(frame, roi_box):
        x1, y1, x2, y2 = roi_box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 220, 0), 2)
        cv2.putText(
            frame,
            "ROI",
            (x1 + 8, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "ROI",
            (x1 + 8, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 220, 0),
            1,
            cv2.LINE_AA,
        )
        return frame

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            roi_box = self.center_roi_box(frame) if self.enable_center_roi else None
            if roi_box is not None:
                x1, y1, x2, y2 = roi_box
                results = self.offset_results(self.pipeline.infer(frame[y1:y2, x1:x2]), x1, y1)
            else:
                results = self.pipeline.infer(frame)
        except Exception as exc:
            rospy.logerr("uav_4class_infer failed: %s", exc)
            return

        if results:
            best = max(results, key=lambda item: item["det_conf"] * item["class_conf"])
            payload = {
                "stamp": msg.header.stamp.to_sec(),
                "results": results,
                **best,
                "detection_conf": best["det_conf"],
                "center_x": best["center"][0],
                "center_y": best["center"][1],
            }
            self.result_pub.publish(json.dumps(payload, ensure_ascii=False))

            cx, cy = best["center"]
            self.coords_pub.publish(
                Float32MultiArray(
                    data=[
                        float(cx),
                        float(cy),
                        float(best["det_conf"]),
                        float(best["class_conf"]),
                        float(best["class_id"]),
                    ]
                )
            )

        if self.publish_debug:
            annotated = self.draw_results(frame, results)
            if roi_box is not None:
                annotated = self.draw_roi(annotated, roi_box)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
            if self.show_gui:
                cv2.imshow("uav_4class_infer", annotated)
                cv2.waitKey(1)


def main():
    node = UAV4ClassNode()
    rospy.spin()


if __name__ == "__main__":
    main()
