#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


class OdinToPx4ctrlOdom:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_odom_topic", "/odin1/odometry_highfreq")
        self.output_topic = rospy.get_param("~output_odom_topic", "/odin/odom_for_px4ctrl")
        self.output_frame_id = rospy.get_param("~output_frame_id", "odom")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "base_link")
        self.publish_rate = float(rospy.get_param("~publish_rate", 120.0))
        self.override_z_enabled = bool(rospy.get_param("~override_z_enabled", False))
        self.override_z = float(rospy.get_param("~override_z", 1.3))

        self.latest_odom = None

        self.sub = rospy.Subscriber(
            self.input_topic,
            Odometry,
            self.odom_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        self.pub = rospy.Publisher(
            self.output_topic,
            Odometry,
            queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_callback
        )

        rospy.loginfo("[odin_to_px4ctrl_odom] input:  %s", self.input_topic)
        rospy.loginfo("[odin_to_px4ctrl_odom] output: %s", self.output_topic)
        rospy.loginfo("[odin_to_px4ctrl_odom] rate:   %.1f Hz", self.publish_rate)
        rospy.loginfo(
            "[odin_to_px4ctrl_odom] override_z_enabled=%s override_z=%.3f",
            self.override_z_enabled,
            self.override_z,
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self, event):
        if self.latest_odom is None:
            return

        out = Odometry()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.output_frame_id
        out.child_frame_id = self.output_child_frame_id

        out.pose = self.latest_odom.pose
        out.twist = self.latest_odom.twist
        if self.override_z_enabled:
            out.pose.pose.position.z = self.override_z
            out.twist.twist.linear.z = 0.0

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odin_to_px4ctrl_odom")
    OdinToPx4ctrlOdom()
    rospy.spin()
