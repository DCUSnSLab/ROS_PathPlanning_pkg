#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from visualization_msgs.msg import Marker
from sensor_msgs import point_cloud2
from dynamic_reconfigure.server import Server
from dwa_local_planner.cfg import DWAPlannerConfig

class DWA:
    def __init__(self):
        rospy.init_node('dwa_planner', anonymous=True)
        self.path = None
        self.current_pose = PoseStamped()
        self.current_heading = None
        self.current_velocity = Twist()
        self.obstacles = []

        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)  # 궤적 시각화를 위한 퍼블리셔

        self.morai_pose_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.heading_callback, queue_size=1)
        self.utm_sub = rospy.Subscriber('/ego_utm', Point, self.pose_callback, queue_size=1)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback, queue_size=1)

        self.lookahead_distance = rospy.get_param('~lookahead_distance', 2.5)
        self.max_speed = 1.0
        self.max_steering_angle = 0.5
        self.dt = 0.1

    def path_callback(self, msg):
        """Update the planned path."""
        self.path = [pose.pose.position for pose in msg.poses]

    def pose_callback(self, msg):
        self.current_pose.pose.position.x = msg.x
        self.current_pose.pose.position.y = msg.y

    def heading_callback(self, msg):
        """Update the current heading and trigger control calculation."""
        self.current_heading = math.radians(msg.heading)  # Convert heading to radians
        vel = Twist()
        vel.linear.x = msg.velocity.x
        vel.angular.z = msg.velocity.y
        self.current_velocity = vel
        self.calculate_control()

    def odom_callback(self, msg):
        """Update the current velocity."""
        self.current_velocity = msg.twist.twist

    def pointcloud_callback(self, msg):
        """Update the list of obstacles from PointCloud2 data."""
        self.obstacles = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self.obstacles.append((point[0], point[1]))

    def calculate_control(self):
        if self.path is None:
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

        # DWA 알고리즘을 사용하여 최적의 속도와 조향각 계산
        best_speed, best_steering = self.dwa_planner(current_position, self.current_velocity, self.obstacles)

        # Publish control command
        cmd = CtrlCmd()
        cmd.steering = best_steering
        cmd.accel = best_speed
        self.cmd_pub.publish(cmd)

    def dwa_planner(self, current_pose, current_velocity, obstacles):
        """Dynamic Window Approach 알고리즘을 사용하여 최적의 속도와 조향각을 계산합니다."""
        # 동적 윈도우 생성
        v_min = max(0, current_velocity.linear.x - self.max_speed * self.dt)
        v_max = min(self.max_speed, current_velocity.linear.x + self.max_speed * self.dt)
        w_min = max(-self.max_steering_angle, current_velocity.angular.z - self.max_steering_angle * self.dt)
        w_max = min(self.max_steering_angle, current_velocity.angular.z + self.max_steering_angle * self.dt)

        best_score = -float('inf')
        best_speed = 0.0
        best_steering = 0.0
        best_trajectory = []  # 최적 궤적 저장

        # 모든 가능한 속도와 조향각에 대해 평가
        for v in np.arange(v_min, v_max, 0.1):
            for w in np.arange(w_min, w_max, 0.1):
                trajectory, score = self.evaluate_trajectory(current_pose, v, w, obstacles)
                if score > best_score:
                    best_score = score
                    best_speed = v
                    best_steering = w
                    best_trajectory = trajectory  # 최적 궤적 업데이트

        # 최적 궤적 시각화
        self.visualize_trajectory(best_trajectory)

        return best_speed, best_steering

    def evaluate_trajectory(self, current_pose, v, w, obstacles):
        """주어진 속도와 조향각에 대한 궤적을 평가합니다."""
        # 궤적 시뮬레이션
        x, y, theta = current_pose
        dt = self.dt
        trajectory = []
        for _ in range(10):  # 10 steps ahead
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            trajectory.append((x, y))

        # 궤적과 장애물 간의 거리 계산
        min_distance = float('inf')
        for point in trajectory:
            for obstacle in obstacles:
                distance = math.sqrt((point[0] - obstacle[0]) ** 2 + (point[1] - obstacle[1]) ** 2)
                if distance < min_distance:
                    min_distance = distance

        # 목표 지점과의 거리 계산
        goal_distance = math.sqrt((x - self.path[-1].x) ** 2 + (y - self.path[-1].y) ** 2)

        # 점수 계산 (장애물과의 거리, 목표 지점과의 거리, 속도 등을 고려)
        score = min_distance - goal_distance + v

        return trajectory, score

    def visualize_trajectory(self, trajectory):
        """궤적을 RViz에서 시각화합니다."""
        marker = Marker()
        marker.header.frame_id = "map"  # 기준 프레임
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 선의 두께
        marker.color.a = 1.0  # 투명도
        marker.color.r = 0.0  # 빨간색
        marker.color.g = 1.0  # 초록색
        marker.color.b = 0.0  # 파란색

        # 궤적 포인트 추가
        for point in trajectory:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # 2D 궤적
            marker.points.append(p)

        # 마커 발행
        self.trajectory_pub.publish(marker)

if __name__ == '__main__':
    try:
        DWA()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass