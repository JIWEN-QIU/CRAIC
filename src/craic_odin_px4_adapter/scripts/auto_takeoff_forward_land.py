#!/usr/bin/env python3
import math

import rospy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class AutoTakeoffForwardLand:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odin/odom_for_px4ctrl")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.takeoff_land_topic = rospy.get_param("~takeoff_land_topic", "/px4ctrl/takeoff_land")
        self.stop_setpoints_topic = rospy.get_param("~stop_setpoints_topic", "/mission/stop_setpoints")

        self.takeoff_height = float(rospy.get_param("~takeoff_height", 1.3))
        self.forward_distance = float(rospy.get_param("~forward_distance", 3.0))
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.15))
        self.xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.30))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 35.0))
        self.forward_timeout = float(rospy.get_param("~forward_timeout", 45.0))
        self.settle_time = float(rospy.get_param("~settle_time", 2.0))
        self.land_command_wait = float(rospy.get_param("~land_command_wait", 1.0))

        self.latest_odom = None
        self.start_pose = None
        self.goal_xy = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.takeoff_land_pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=10)
        self.stop_setpoints_pub = rospy.Publisher(self.stop_setpoints_topic, Bool, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo("[auto_takeoff_forward_land] odom: %s", self.odom_topic)
        rospy.loginfo("[auto_takeoff_forward_land] goal: %s", self.goal_topic)
        rospy.loginfo("[auto_takeoff_forward_land] takeoff_land: %s", self.takeoff_land_topic)
        rospy.loginfo("[auto_takeoff_forward_land] stop_setpoints: %s", self.stop_setpoints_topic)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def wait_for_takeoff_land_subscriber(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.takeoff_land_pub.get_num_connections() < 1:
            rospy.loginfo_throttle(
                1.0,
                "[auto_takeoff_forward_land] waiting for takeoff_land subscriber: %s",
                self.takeoff_land_topic,
            )
            rate.sleep()

    def wait_for_odom(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.latest_odom is None:
            rate.sleep()

    def current_xyz_yaw(self):
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return p.x, p.y, p.z, yaw

    def publish_takeoff_land(self, command, duration=1.0):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        rate = rospy.Rate(10)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.takeoff_land_pub.publish(msg)
            rate.sleep()

    def set_stop_setpoints(self, stop):
        msg = Bool()
        msg.data = stop
        self.stop_setpoints_pub.publish(msg)

    def wait_until(self, predicate, timeout, label):
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if predicate():
                return True
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("[auto_takeoff_forward_land] timeout while waiting for %s", label)
                return False
            rate.sleep()
        return False

    def publish_forward_goal(self):
        x, y, z, yaw = self.current_xyz_yaw()
        goal_x = x + self.forward_distance * math.cos(yaw)
        goal_y = y + self.forward_distance * math.sin(yaw)
        self.goal_xy = (goal_x, goal_y)

        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = z
        goal.pose.orientation = Quaternion(*q)

        for _ in range(5):
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            rospy.sleep(0.05)

        rospy.loginfo(
            "[auto_takeoff_forward_land] forward goal: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            goal_x,
            goal_y,
            z,
            math.degrees(yaw),
        )

    def reached_takeoff_height(self):
        _, _, z, _ = self.current_xyz_yaw()
        return z >= self.start_pose[2] + self.takeoff_height - self.height_tolerance

    def reached_forward_goal(self):
        x, y, _, _ = self.current_xyz_yaw()
        dx = x - self.goal_xy[0]
        dy = y - self.goal_xy[1]
        return math.hypot(dx, dy) <= self.xy_tolerance

    def run(self):
        self.wait_for_odom()
        self.wait_for_takeoff_land_subscriber()
        self.set_stop_setpoints(False)
        self.start_pose = self.current_xyz_yaw()
        rospy.loginfo(
            "[auto_takeoff_forward_land] start: x=%.3f y=%.3f z=%.3f yaw=%.1f deg",
            self.start_pose[0],
            self.start_pose[1],
            self.start_pose[2],
            math.degrees(self.start_pose[3]),
        )

        rospy.loginfo("[auto_takeoff_forward_land] TAKEOFF to %.2fm", self.takeoff_height)
        self.publish_takeoff_land(TakeoffLand.TAKEOFF)
        if not self.wait_until(self.reached_takeoff_height, self.takeoff_timeout, "takeoff height"):
            return

        rospy.sleep(self.settle_time)
        self.publish_forward_goal()
        if not self.wait_until(self.reached_forward_goal, self.forward_timeout, "forward goal"):
            return

        rospy.loginfo("[auto_takeoff_forward_land] forward goal reached, settling before LAND")
        rospy.sleep(self.settle_time)
        self.set_stop_setpoints(True)
        rospy.sleep(self.land_command_wait)
        self.publish_takeoff_land(TakeoffLand.LAND)
        rospy.loginfo("[auto_takeoff_forward_land] LAND command sent")


if __name__ == "__main__":
    rospy.init_node("auto_takeoff_forward_land")
    AutoTakeoffForwardLand().run()
