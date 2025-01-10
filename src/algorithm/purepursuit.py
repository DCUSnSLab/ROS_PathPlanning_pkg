#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from pyproj import Proj, CRS, transform, Transformer
from morai_msgs.msg import EgoVehicleStatus, GPSMessage, CtrlCmd
from visualization_msgs.msg import Marker

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.morai_pose_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.morai_pose_callback)
        # self.gps_pose_sub = rospy.Subscriber('/gps', GPSMessage, self.pose_callback)
        self.gps_pose_sub = rospy.Subscriber('/ego_vehicle', Marker, self.pose_callback)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        self.origin_path_coord = None

        # Marker 퍼블리셔
        self.marker_pub = rospy.Publisher('/lookahead_marker', Marker, queue_size=10)

        self.lookahead_distance = rospy.get_param('~lookahead_distance', 2.5)

        self.utm_zone = 52
        self.transformer_to_utm = Transformer.from_crs("epsg:4326", f"epsg:326{self.utm_zone}", always_xy=True)

        self.current_pose = PoseStamped()
        self.path = []

    def pose_callback(self, msg):
        # utm_x, utm_y = self.transformer_to_utm.transform(msg.longitude, msg.latitude)
        # self.current_position_utm = (utm_x, utm_y)
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "waypoint"
        self.current_pose.header.stamp = rospy.Time.now()

        pose = Pose()
        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y

        self.current_pose.pose = pose
        self.calculate_control()

    def morai_pose_callback(self, msg):
        self.current_pose.pose.orientation = self.heading_to_quaternion(msg.heading)

    def path_callback(self, msg):
        """Update the path points from the Path message."""
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.path = msg.poses

        # Set origin_path_coord if it's None
        if not hasattr(self, 'origin_path_coord') or self.origin_path_coord is None:
            if msg.poses:  # Ensure the path has at least one point
                first_pose = msg.poses[0].pose.position
                self.origin_path_coord = [first_pose.x, first_pose.y]
                rospy.loginfo(f"Origin path coordinate set to: {self.origin_path_coord}")

    def heading_to_quaternion(self, heading_deg):
        # Degree to Radian
        heading_rad = math.radians(heading_deg)

        # Quaternion Calculation
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(heading_rad / 2.0)
        q.w = math.cos(heading_rad / 2.0)

        return q

    def visualize_lookahead(self, lookahead_point):
        marker = Marker()
        marker.header.frame_id = "waypoint"  # RViz의 좌표계에 맞춰 설정
        marker.header.stamp = rospy.Time.now()

        marker.ns = "lookahead_point"
        marker.id = 0
        marker.type = Marker.SPHERE  # 구 형태로 표시
        marker.action = Marker.ADD

        # Lookahead 위치 설정
        marker.pose.position.x = lookahead_point.x
        marker.pose.position.y = lookahead_point.y
        marker.pose.position.z = 0.0  # 지면에 가까운 높이
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 크기 및 색상 설정
        marker.scale.x = 0.5  # x, y, z 방향 크기
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # 투명도
        marker.color.r = 1.0  # 빨강
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Marker 퍼블리시
        self.marker_pub.publish(marker)

    def calculate_control(self):
        if self.current_pose is None or not self.path:
            return

        # Lookahead point 계산
        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            return

        # Lookahead point 시각화
        self.visualize_lookahead(lookahead_point)

        # Pure Pursuit 계산
        dx = lookahead_point.x - self.current_pose.pose.position.x
        dy = lookahead_point.y - self.current_pose.pose.position.y
        ld = math.sqrt(dx**2 + dy**2)
        raw_steering_angle = 2 * dy / ld**2

        # 정규화: steering 값을 -0.5 ~ 0.5로 제한
        max_steering_angle = 0.5
        min_steering_angle = -0.5
        steering_angle = max(min(raw_steering_angle, max_steering_angle), min_steering_angle)

        # Ackermann 메시지 발행
        cmd = CtrlCmd()
        #print(f"Raw steering: {raw_steering_angle}, Clamped steering: {steering_angle}")
        cmd.steering = steering_angle
        cmd.accel = 1.0  # 속도 설정
        self.cmd_pub.publish(cmd)


    def find_lookahead_point(self):
        """Find the lookahead point and remove past waypoints."""
        if not self.path or self.current_pose is None:
            return None

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        # 필터링: 지나간 포인트 제거
        remaining_path = []
        for pose in self.path:
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            # lookahead_distance 이상 떨어진 포인트만 남김
            if distance >= self.lookahead_distance:
                remaining_path.append(pose)

        # 업데이트된 경로를 반영
        self.path = remaining_path

        # 가장 가까운 lookahead 포인트 반환
        for pose in self.path:
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.lookahead_distance:
                return pose.pose.position

        return None


if __name__ == '__main__':
    try:
        PurePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
