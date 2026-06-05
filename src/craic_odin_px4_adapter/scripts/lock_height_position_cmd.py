#!/usr/bin/env python3
import copy
import math

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool


class LockHeightPositionCmd:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/super/setpoints_cmd_raw")
        self.output_topic = rospy.get_param("~output_topic", "/setpoints_cmd")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odin/odom_for_px4ctrl")
        self.stop_topic = rospy.get_param("~stop_topic", "/mission/stop_setpoints")
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.zero_vertical_terms = bool(rospy.get_param("~zero_vertical_terms", True))
        self.yaw_follow_velocity = bool(rospy.get_param("~yaw_follow_velocity", True))
        self.yaw_min_speed = float(rospy.get_param("~yaw_min_speed", 0.15))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 1.0))

        self.lock_z = None
        self.latest_odom_z = None
        self.latest_odom_yaw = None
        self.last_yaw = None
        self.last_cmd_time = None
        self.stop_output = False

        self.pub = rospy.Publisher(self.output_topic, PositionCommand, queue_size=10)
        self.cmd_sub = rospy.Subscriber(
            self.input_topic,
            PositionCommand,
            self.cmd_cb,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.goal_sub = rospy.Subscriber(
            self.goal_topic,
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
        self.stop_sub = rospy.Subscriber(
            self.stop_topic,
            Bool,
            self.stop_cb,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo("[lock_height_position_cmd] input:  %s", self.input_topic)
        rospy.loginfo("[lock_height_position_cmd] output: %s", self.output_topic)
        rospy.loginfo("[lock_height_position_cmd] enabled=%s zero_vertical_terms=%s",
                      self.enabled, self.zero_vertical_terms)
        rospy.loginfo(
            "[lock_height_position_cmd] yaw_follow_velocity=%s yaw_min_speed=%.3f max_yaw_rate=%.3f",
            self.yaw_follow_velocity,
            self.yaw_min_speed,
            self.max_yaw_rate,
        )
        rospy.loginfo("[lock_height_position_cmd] stop topic: %s", self.stop_topic)

    def odom_cb(self, msg):
        self.latest_odom_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_cb(self, msg):
        self.stop_output = msg.data
        rospy.logwarn("[lock_height_position_cmd] stop_output=%s", self.stop_output)

    def goal_cb(self, msg):
        self.lock_z = msg.pose.position.z
        rospy.loginfo("[lock_height_position_cmd] lock z=%.3f from goal", self.lock_z)

    def cmd_cb(self, msg):
        if self.stop_output:
            return

        out = copy.deepcopy(msg)
        now = rospy.Time.now()

        if self.enabled:
            if self.lock_z is None and self.latest_odom_z is not None:
                self.lock_z = self.latest_odom_z
                rospy.loginfo("[lock_height_position_cmd] lock z=%.3f from odom", self.lock_z)

            if self.lock_z is not None:
                out.position.z = self.lock_z
                if self.zero_vertical_terms:
                    out.velocity.z = 0.0
                    out.acceleration.z = 0.0
                    out.jerk.z = 0.0

        if self.yaw_follow_velocity:
            speed_xy = math.hypot(out.velocity.x, out.velocity.y)
            if self.last_yaw is None:
                if self.latest_odom_yaw is not None:
                    self.last_yaw = self.latest_odom_yaw
                else:
                    self.last_yaw = out.yaw

            if speed_xy >= self.yaw_min_speed:
                desired_yaw = math.atan2(out.velocity.y, out.velocity.x)
                dt = 0.0
                if self.last_cmd_time is not None:
                    dt = max(0.0, (now - self.last_cmd_time).to_sec())

                if dt > 0.0 and self.max_yaw_rate > 0.0:
                    max_step = self.max_yaw_rate * dt
                    yaw_error = self.wrap_pi(desired_yaw - self.last_yaw)
                    yaw_step = max(-max_step, min(max_step, yaw_error))
                    out.yaw = self.wrap_pi(self.last_yaw + yaw_step)
                    out.yaw_dot = yaw_step / dt
                else:
                    out.yaw = desired_yaw
                    out.yaw_dot = 0.0

                self.last_yaw = out.yaw
            else:
                out.yaw = self.last_yaw
                out.yaw_dot = 0.0

        self.last_cmd_time = now
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("lock_height_position_cmd")
    LockHeightPositionCmd()
    rospy.spin()
