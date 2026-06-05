#!/usr/bin/env python3
import math

import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OrbitObstacleGoals:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odin1/odometry_highfreq")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.center_topic = rospy.get_param("~center_topic", "/clicked_point")
        self.frame_id = rospy.get_param("~frame_id", "odom")

        self.obstacle_size = float(rospy.get_param("~obstacle_size", 0.60))
        self.obstacle_distance = float(rospy.get_param("~obstacle_distance", 2.50))
        self.clearance = float(rospy.get_param("~clearance", 0.70))
        self.xy_tolerance = float(rospy.get_param("~xy_tolerance", 0.35))
        self.hold_time = float(rospy.get_param("~hold_time", 1.5))
        self.segment_timeout = float(rospy.get_param("~segment_timeout", 45.0))
        self.publish_repeats = int(rospy.get_param("~publish_repeats", 5))
        self.republish_period = float(rospy.get_param("~republish_period", 1.0))
        self.use_clicked_center = bool(rospy.get_param("~use_clicked_center", False))

        self.center_x = rospy.get_param("~obstacle_x", None)
        self.center_y = rospy.get_param("~obstacle_y", None)
        self.center_z = rospy.get_param("~obstacle_z", None)

        self.latest_odom = None
        self.clicked_center = None
        self.started = False

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.center_sub = rospy.Subscriber(
            self.center_topic,
            PointStamped,
            self.center_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo("[orbit_obstacle_goals] odom:   %s", self.odom_topic)
        rospy.loginfo("[orbit_obstacle_goals] goal:   %s", self.goal_topic)
        rospy.loginfo("[orbit_obstacle_goals] center: %s", self.center_topic)
        rospy.loginfo(
            "[orbit_obstacle_goals] obstacle_size=%.2fm obstacle_distance=%.2fm clearance=%.2fm waypoint_radius=%.2fm",
            self.obstacle_size,
            self.obstacle_distance,
            self.clearance,
            self.waypoint_radius(),
        )

    def odom_cb(self, msg):
        self.latest_odom = msg

    def center_cb(self, msg):
        self.clicked_center = msg
        rospy.loginfo(
            "[orbit_obstacle_goals] clicked obstacle center: x=%.3f y=%.3f z=%.3f frame=%s",
            msg.point.x,
            msg.point.y,
            msg.point.z,
            msg.header.frame_id,
        )

    def waypoint_radius(self):
        return self.obstacle_size * 0.5 + self.clearance

    def wait_for_odom(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.latest_odom is None:
            rospy.loginfo_throttle(1.0, "[orbit_obstacle_goals] waiting for odom: %s", self.odom_topic)
            rate.sleep()

    def current_xyz_yaw(self):
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return p.x, p.y, p.z, yaw

    def wait_for_center(self):
        if self.center_x is not None and self.center_y is not None:
            z = self.center_z if self.center_z is not None else 0.0
            return float(self.center_x), float(self.center_y), float(z)

        if not self.use_clicked_center:
            x, y, z, yaw = self.current_xyz_yaw()
            cx = x + self.obstacle_distance * math.cos(yaw)
            cy = y + self.obstacle_distance * math.sin(yaw)
            rospy.loginfo(
                "[orbit_obstacle_goals] inferred obstacle center %.2fm ahead: x=%.3f y=%.3f z=%.3f",
                self.obstacle_distance,
                cx,
                cy,
                z,
            )
            return cx, cy, z

        rate = rospy.Rate(20)
        rospy.loginfo("[orbit_obstacle_goals] waiting for RViz Publish Point obstacle center")
        while not rospy.is_shutdown() and self.clicked_center is None:
            rate.sleep()

        p = self.clicked_center.point
        return p.x, p.y, p.z

    def normalize2(self, x, y):
        norm = math.hypot(x, y)
        if norm < 1.0e-3:
            return 1.0, 0.0
        return x / norm, y / norm

    def build_waypoints(self, cx, cy):
        sx, sy, _, yaw = self.current_xyz_yaw()

        # "front" is the near side of the obstacle, facing the current vehicle
        # position after manual takeoff.
        to_obstacle_x, to_obstacle_y = self.normalize2(cx - sx, cy - sy)
        front_x, front_y = -to_obstacle_x, -to_obstacle_y
        left_x, left_y = -to_obstacle_y, to_obstacle_x
        r = self.waypoint_radius()

        return [
            ("front", cx + front_x * r, cy + front_y * r),
            ("left", cx + left_x * r, cy + left_y * r),
            ("back", cx - front_x * r, cy - front_y * r),
            ("right", cx - left_x * r, cy - left_y * r),
            ("front_final", cx + front_x * r, cy + front_y * r),
        ]

    def make_goal(self, x, y, z, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation = Quaternion(*q)
        return goal

    def publish_goal(self, label, x, y, z, yaw):
        goal = self.make_goal(x, y, z, yaw)
        for _ in range(max(1, self.publish_repeats)):
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            rospy.sleep(0.05)
        rospy.loginfo(
            "[orbit_obstacle_goals] goal %-11s x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            label,
            x,
            y,
            z,
            math.degrees(yaw),
        )

    def reached_xy(self, x, y):
        cx, cy, _, _ = self.current_xyz_yaw()
        return math.hypot(cx - x, cy - y) <= self.xy_tolerance

    def wait_until_reached(self, label, x, y, z, yaw):
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        last_publish_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.reached_xy(x, y):
                rospy.loginfo("[orbit_obstacle_goals] reached %s", label)
                rospy.sleep(self.hold_time)
                return True
            if (rospy.Time.now() - last_publish_time).to_sec() >= self.republish_period:
                self.publish_goal(label, x, y, z, yaw)
                last_publish_time = rospy.Time.now()
            if (rospy.Time.now() - start_time).to_sec() > self.segment_timeout:
                rospy.logerr("[orbit_obstacle_goals] timeout before reaching %s", label)
                return False
            rate.sleep()
        return False

    def yaw_to_next(self, waypoints, index):
        _, x, y = waypoints[index]
        if index >= len(waypoints) - 1:
            _, px, py = waypoints[index - 1]
            return math.atan2(y - py, x - px)
        _, nx, ny = waypoints[index + 1]
        return math.atan2(ny - y, nx - x)

    def run(self):
        self.wait_for_odom()
        cx, cy, cz = self.wait_for_center()
        _, _, flight_z, _ = self.current_xyz_yaw()
        waypoints = self.build_waypoints(cx, cy)

        rospy.loginfo(
            "[orbit_obstacle_goals] obstacle center x=%.3f y=%.3f, flight_z=%.3f, center_z=%.3f",
            cx,
            cy,
            flight_z,
            cz,
        )

        for i, (label, x, y) in enumerate(waypoints):
            yaw = self.yaw_to_next(waypoints, i)
            self.publish_goal(label, x, y, flight_z, yaw)
            if not self.wait_until_reached(label, x, y, flight_z, yaw):
                return

        rospy.loginfo("[orbit_obstacle_goals] clockwise orbit sequence completed")


if __name__ == "__main__":
    rospy.init_node("orbit_obstacle_goals")
    OrbitObstacleGoals().run()
