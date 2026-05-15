#!/usr/bin/env python3
import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import euler_from_quaternion


class SimpleWaypointTest:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odin/odom_for_px4ctrl")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/setpoints_cmd")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/traj_start_trigger")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.step = float(rospy.get_param("~step", 0.5))
        self.hold_time = float(rospy.get_param("~hold_time", 5.0))

        self.latest_odom = None
        self.trigger_pose = None
        self.origin = None
        self.origin_yaw = 0.0

        self.pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=1,
            tcp_nodelay=True
        )
        self.trigger_sub = rospy.Subscriber(
            self.trigger_topic,
            PoseStamped,
            self.trigger_cb,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo("[simple_waypoint_test] waiting for odom: %s", self.odom_topic)
        rospy.loginfo("[simple_waypoint_test] waiting for trigger: %s", self.trigger_topic)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def trigger_cb(self, msg):
        self.trigger_pose = msg

    def get_yaw(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def wait_for_odom(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.latest_odom is None:
            rate.sleep()

        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        self.origin = [p.x, p.y, p.z]
        self.origin_yaw = self.get_yaw(q)

        rospy.loginfo(
            "[simple_waypoint_test] current odom: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            self.origin[0], self.origin[1], self.origin[2], math.degrees(self.origin_yaw)
        )

    def wait_for_trigger(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.trigger_pose is None:
            rate.sleep()

        p = self.trigger_pose.pose.position
        q = self.trigger_pose.pose.orientation
        self.origin = [p.x, p.y, p.z]
        self.origin_yaw = self.get_yaw(q)

        rospy.loginfo(
            "[simple_waypoint_test] trigger received, origin set: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            self.origin[0], self.origin[1], self.origin[2], math.degrees(self.origin_yaw)
        )

    def make_cmd(self, x, y, z, yaw):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "odom"

        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z

        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0

        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0

        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0

        cmd.yaw = yaw
        cmd.yaw_dot = 0.0

        cmd.trajectory_id = 1
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        return cmd

    def publish_hold(self, target, duration, label):
        rospy.loginfo(
            "[simple_waypoint_test] target %s: x=%.3f y=%.3f z=%.3f",
            label, target[0], target[1], target[2]
        )

        rate = rospy.Rate(self.rate_hz)
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            cmd = self.make_cmd(target[0], target[1], target[2], self.origin_yaw)
            self.pub.publish(cmd)
            rate.sleep()

    def run(self):
        self.wait_for_odom()
        self.wait_for_trigger()

        ox, oy, oz = self.origin
        s = self.step

        sequence = [
            ("hold_origin_1", [ox,     oy,     oz], self.hold_time),
            ("forward",       [ox+s,   oy,     oz], self.hold_time),
            ("back_origin",   [ox,     oy,     oz], self.hold_time),
            ("left",          [ox,     oy+s,   oz], self.hold_time),
            ("origin_final",  [ox,     oy,     oz], self.hold_time),
        ]

        for label, target, duration in sequence:
            self.publish_hold(target, duration, label)

        rospy.loginfo("[simple_waypoint_test] sequence completed. Holding origin.")
        while not rospy.is_shutdown():
            self.publish_hold([ox, oy, oz], 1.0, "origin_hold")


if __name__ == "__main__":
    rospy.init_node("simple_waypoint_test")
    node = SimpleWaypointTest()
    node.run()
