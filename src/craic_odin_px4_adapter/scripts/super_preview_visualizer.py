#!/usr/bin/env python3
import copy
import math

import rospy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from quadrotor_msgs.msg import PositionCommand
from visualization_msgs.msg import Marker, MarkerArray


class SuperPreviewVisualizer:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/super/setpoints_cmd_raw")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.min_point_distance = float(rospy.get_param("~min_point_distance", 0.03))
        self.max_points = int(rospy.get_param("~max_points", 3000))

        self.path_topic = rospy.get_param("~path_topic", "/super_preview/command_path")
        self.path_marker_topic = rospy.get_param("~path_marker_topic", "/super_preview/command_path_marker")

        self.marker_pairs = [
            (
                rospy.get_param("~exp_traj_topic", "/super_fsm_node/visualization/exp_traj"),
                rospy.get_param("~exp_traj_latched_topic", "/super_preview/exp_traj"),
            ),
            (
                rospy.get_param("~exp_sfcs_topic", "/super_fsm_node/visualization/exp_sfcs"),
                rospy.get_param("~exp_sfcs_latched_topic", "/super_preview/exp_sfcs"),
            ),
            (
                rospy.get_param("~receding_traj_topic", "/super_fsm_node/visualization/receding_traj"),
                rospy.get_param("~receding_traj_latched_topic", "/super_preview/receding_traj"),
            ),
            (
                rospy.get_param("~goal_marker_topic", "/super_fsm_node/visualization/goal"),
                rospy.get_param("~goal_marker_latched_topic", "/super_preview/goal"),
            ),
        ]

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.last_point = None
        self.latched_markers = {}

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.path_marker_pub = rospy.Publisher(self.path_marker_topic, MarkerArray, queue_size=1, latch=True)
        self.marker_pubs = {
            output_topic: rospy.Publisher(output_topic, MarkerArray, queue_size=1, latch=True)
            for _, output_topic in self.marker_pairs
        }

        rospy.Subscriber(self.cmd_topic, PositionCommand, self.cmd_cb, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1, tcp_nodelay=True)

        for input_topic, output_topic in self.marker_pairs:
            rospy.Subscriber(
                input_topic,
                MarkerArray,
                lambda msg, out=output_topic: self.marker_cb(msg, out),
                queue_size=10,
                tcp_nodelay=True,
            )

        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_cb)

        rospy.loginfo("[super_preview_visualizer] command path: %s", self.path_topic)
        rospy.loginfo("[super_preview_visualizer] command marker: %s", self.path_marker_topic)
        for input_topic, output_topic in self.marker_pairs:
            rospy.loginfo("[super_preview_visualizer] latch %s -> %s", input_topic, output_topic)

    @staticmethod
    def has_visible_markers(msg):
        for marker in msg.markers:
            if marker.action != Marker.DELETEALL and marker.action != Marker.DELETE:
                return True
        return False

    def goal_cb(self, _msg):
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.last_point = None
        self.publish_path()

    def cmd_cb(self, msg):
        p = msg.position
        if self.last_point is not None:
            d = math.sqrt(
                (p.x - self.last_point.x) ** 2
                + (p.y - self.last_point.y) ** 2
                + (p.z - self.last_point.z) ** 2
            )
            if d < self.min_point_distance:
                return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        pose.pose.position.z = p.z
        pose.pose.orientation.w = 1.0

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.header.frame_id = pose.header.frame_id
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses = self.path_msg.poses[-self.max_points:]

        self.last_point = Point(p.x, p.y, p.z)
        self.publish_path()

    def marker_cb(self, msg, output_topic):
        if not self.has_visible_markers(msg):
            return

        out = copy.deepcopy(msg)
        for marker in out.markers:
            marker.header.frame_id = self.frame_id
            if marker.lifetime.to_sec() > 0.0:
                marker.lifetime = rospy.Duration(0.0)

        self.latched_markers[output_topic] = out
        self.marker_pubs[output_topic].publish(out)

    def publish_path(self):
        self.path_pub.publish(self.path_msg)

        marker = Marker()
        marker.header = self.path_msg.header
        marker.ns = "super_preview_command_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [pose.pose.position for pose in self.path_msg.poses]

        arr = MarkerArray()
        arr.markers.append(marker)
        self.path_marker_pub.publish(arr)

    def timer_cb(self, _event):
        self.publish_path()
        for output_topic, msg in self.latched_markers.items():
            self.marker_pubs[output_topic].publish(msg)


if __name__ == "__main__":
    rospy.init_node("super_preview_visualizer")
    SuperPreviewVisualizer()
    rospy.spin()
