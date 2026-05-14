#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdinToMavrosPose:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_odom_topic", "/odin1/odometry_highfreq")
        self.output_topic = rospy.get_param("~output_pose_topic", "/mavros/vision_pose/pose")
        self.output_frame_id = rospy.get_param("~output_frame_id", "odom")
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))

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
            PoseStamped,
            queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_callback
        )

        rospy.loginfo("[odin_to_mavros_pose] input:  %s", self.input_topic)
        rospy.loginfo("[odin_to_mavros_pose] output: %s", self.output_topic)
        rospy.loginfo("[odin_to_mavros_pose] rate:   %.1f Hz", self.publish_rate)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self, event):
        if self.latest_odom is None:
            return

        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.output_frame_id

        out.pose.position = self.latest_odom.pose.pose.position
        out.pose.orientation = self.latest_odom.pose.pose.orientation

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odin_to_mavros_pose")
    OdinToMavrosPose()
    rospy.spin()
