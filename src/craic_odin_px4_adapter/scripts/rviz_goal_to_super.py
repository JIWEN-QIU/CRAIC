#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class RvizGoalToSuper:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odin/odom_for_px4ctrl")
        self.goal_height = float(rospy.get_param("~goal_height", 1.0))
        self.use_current_height = bool(rospy.get_param("~use_current_height", False))
        self.output_frame_id = rospy.get_param("~output_frame_id", "odom")

        self.latest_odom = None

        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.goal_sub = rospy.Subscriber(
            self.input_topic,
            PoseStamped,
            self.goal_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo("[rviz_goal_to_super] input:  %s", self.input_topic)
        rospy.loginfo("[rviz_goal_to_super] output: %s", self.output_topic)
        rospy.loginfo("[rviz_goal_to_super] odom:   %s", self.odom_topic)
        rospy.loginfo(
            "[rviz_goal_to_super] height: %.3f use_current_height=%s",
            self.goal_height,
            self.use_current_height,
        )

    def odom_cb(self, msg):
        self.latest_odom = msg

    def goal_cb(self, msg):
        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.output_frame_id
        out.pose = msg.pose

        if self.use_current_height and self.latest_odom is not None:
            out.pose.position.z = self.latest_odom.pose.pose.position.z
        else:
            out.pose.position.z = self.goal_height

        self.pub.publish(out)
        rospy.loginfo(
            "[rviz_goal_to_super] goal -> x=%.3f y=%.3f z=%.3f frame=%s",
            out.pose.position.x,
            out.pose.position.y,
            out.pose.position.z,
            out.header.frame_id,
        )


if __name__ == "__main__":
    rospy.init_node("rviz_goal_to_super")
    RvizGoalToSuper()
    rospy.spin()
