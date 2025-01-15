#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from visualization_msgs.msg import Marker

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.morai_pose_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.heading_callback)
        self.pose_sub = rospy.Subscriber('/ego_vehicle', Marker, self.pose_callback)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)

        self.lookahead_distance = rospy.get_param('~lookahead_distance', 5.0)
        self.path = []
        self.current_pose = PoseStamped()
        self.current_heading = None

    def path_callback(self, msg):
        """Update the planned path."""
        self.path = [pose.pose.position for pose in msg.poses]

    def pose_callback(self, msg):
        # self.current_pose = PoseStamped()
        # self.current_pose.header.frame_id = "waypoint"
        # self.current_pose.header.stamp = rospy.Time.now()
        # pose = Pose()
        # pose.position.x = msg.pose.position.x
        # pose.position.y = msg.pose.position.y
        # self.current_pose.pose = pose

        self.current_pose.pose.position.x = msg.pose.position.x
        self.current_pose.pose.position.y = msg.pose.position.y

    def heading_callback(self, msg):
        """Update the current heading and trigger control calculation."""
        self.current_heading = math.radians(msg.heading)  # Convert heading to radians
        self.calculate_control()

    def visualize_lookahead(self, lookahead_point):
        """Visualize the lookahead point in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lookahead_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lookahead_point.x
        marker.pose.position.y = lookahead_point.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def calculate_control(self):
        if not self.path:
            rospy.logwarn("Path is empty. Stopping the vehicle.")
            cmd = CtrlCmd()
            cmd.accel = 0.0
            cmd.steering = 0.0
            self.cmd_pub.publish(cmd)
            return

        current_position = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_heading
        ]

        if self.current_pose is None or not self.path:
            return

        # Lookahead point 계산
        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            return

        # Visualize the lookahead point
        self.visualize_lookahead(lookahead_point)

        # Use legacy Pure Pursuit logic to calculate steering angle
        steering_angle = self.calculate_steering_angle(current_position, lookahead_point)

        # Publish control command
        cmd = CtrlCmd()
        cmd.steering = steering_angle  # Normalized value (-0.5 ~ 0.5)
        print(steering_angle)
        cmd.accel = 1.0  # Set a constant acceleration
        # cmd.accel = 0.8  # Set a constant acceleration
        self.cmd_pub.publish(cmd)

    def calculate_steering_angle(self, current_pose, lookahead_point):
        # Calculate steering angle
        steering_angle = math.atan2(lookahead_point.y - current_pose[1], lookahead_point.x - current_pose[0])
        angle_difference = steering_angle - current_pose[2]
        steering_angle = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        # Normalize steering angle to -0.5 ~ 0.5
        steering_angle = self.map_steering_angle(steering_angle, -math.pi, math.pi, -0.5, 0.5)

        return steering_angle

    def find_lookahead_point(self):
        """Find the lookahead point based on purepursuit.py logic."""
        if not self.path or self.current_pose is None:
            return None

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        for i, pose in enumerate(self.path):
            dx = pose.x - current_x
            dy = pose.y - current_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance >= self.lookahead_distance:
                # Remove passed waypoints
                self.path = self.path[i:]
                return pose

        return None

    def map_steering_angle(self, value, from_min, from_max, to_min, to_max):
        """Map a value from one range to another while preserving the ratio."""
        return (value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


if __name__ == '__main__':
    try:
        PurePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
