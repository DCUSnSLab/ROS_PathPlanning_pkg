#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from visualization_msgs.msg import Marker

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.path = None

        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)

        self.current_pose = PoseStamped()
        self.current_heading = None

        self.morai_pose_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.heading_callback, queue_size=1)
        # self.pose_sub = rospy.Subscriber('/ego_vehicle', Marker, self.pose_callback, queue_size=1)
        self.utm_sub = rospy.Subscriber('/ego_utm', Point, self.pose_callback, queue_size=1)
        # self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.lookahead_distance = rospy.get_param('~lookahead_distance', 2.5)

        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback, queue_size=1)
        # self.path_receiver()
    def path_receiver(self):
        topic = "/planned_path"
        rospy.loginfo(f"Waiting for a message on {topic}...")
        msg = rospy.wait_for_message(topic, Path)  # 토픽 타입에 맞게 변경
        # rospy.loginfo(f"Received message: {msg.data}")
        self.path = [pose.pose.position for pose in msg.poses]

    def path_callback(self, msg):
        """Update the planned path."""
        self.path = [pose.pose.position for pose in msg.poses]

    def pose_callback(self, msg):
        self.current_pose.pose.position.x = msg.x
        self.current_pose.pose.position.y = msg.y

    def heading_callback(self, msg):
        """Update the current heading and trigger control calculation."""
        self.current_heading = math.radians(msg.heading)  # Convert heading to radians
        self.calculate_control()

    def visualize_lookahead(self, lookahead_point):
        """Visualize the lookahead point in RViz."""
        marker = Marker()
        # marker.header.frame_id = "base_link"
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lookahead_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lookahead_point.x - self.current_pose.pose.position.x
        marker.pose.position.y = lookahead_point.y - self.current_pose.pose.position.y
        #marker.pose.position.x = lookahead_point.x
        #marker.pose.position.y = lookahead_point.y
        marker.pose.position.z = 0 # temp val
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def calculate_control(self):
        if self.path == None:
            # rospy.logwarn("Path is empty. Stopping the vehicle.")
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
        """Find the lookahead point based on Pure Pursuit logic with interpolation."""
        if not self.path or self.current_pose is None:
            return None

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        # 순차적으로 경로를 탐색하며 Lookahead 거리 확인
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]

            # 두 점 사이 거리 계산
            dx1 = p1.x - current_x
            dy1 = p1.y - current_y
            d1 = math.sqrt(dx1 ** 2 + dy1 ** 2)

            dx2 = p2.x - current_x
            dy2 = p2.y - current_y
            d2 = math.sqrt(dx2 ** 2 + dy2 ** 2)

            # Lookahead 거리가 p1과 p2 사이에 있는 경우 보간
            if d1 < self.lookahead_distance and d2 >= self.lookahead_distance:
                ratio = (self.lookahead_distance - d1) / (d2 - d1)
                lookahead_x = p1.x + ratio * (p2.x - p1.x)
                lookahead_y = p1.y + ratio * (p2.y - p1.y)

                lookahead_point = Point()
                lookahead_point.x = lookahead_x
                lookahead_point.y = lookahead_y
                lookahead_point.z = 0.0  # 지면 적용을 위한 기본값

                # 지나간 경로 삭제
                self.path = self.path[i:]
                return lookahead_point

        return None  # Lookahead 거리 내에 점이 없을 경우 None 반환

    def map_steering_angle(self, value, from_min, from_max, to_min, to_max):
        """Map a value from one range to another while preserving the ratio."""
        return (value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


if __name__ == '__main__':
    try:
        PurePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
