#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import GPSMessage
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import utm
from tf.transformations import quaternion_from_euler


class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher', anonymous=True)

        # 지도 기준 GPS 좌표 (Lat, Long, Alt)
        self.map_gps_coordinates = (37.23923857150045, 126.7731611307903, 0)
        self.map_utm = utm.from_latlon(self.map_gps_coordinates[0], self.map_gps_coordinates[1])

        self.odom_pub = rospy.Publisher("/odom/coordinate", Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()

        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        self.gps_data = (0.0, 0.0, 0.0)
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # IMU에서 받은 orientation 값 저장

        self.rate = rospy.Rate(10)  # 10Hz
        self.run()

    def gps_callback(self, msg):
        """GPS 데이터를 UTM 좌표로 변환하여 저장"""
        current_utm = utm.from_latlon(msg.latitude, msg.longitude)
        self.gps_data = (
            current_utm[0] - self.map_utm[0],
            current_utm[1] - self.map_utm[1],
            msg.altitude
        )

    def imu_callback(self, msg):
        """IMU 데이터를 받아서 orientation을 저장 (map -> odom에서 사용)"""
        self.orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def publish_tf(self, parent, child, x, y, z, qx, qy, qz, qw):
        """TF 변환을 발행하는 함수"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # map -> odom 변환을 GPS 데이터와 IMU 방향을 기반으로 설정
            self.publish_tf("map", "odom", self.gps_data[0], self.gps_data[1], 0,
                            self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3])

            # odom -> base_link 변환을 (0,0,0) 위치 및 단위 quaternion으로 고정
            self.publish_tf("odom", "base_link", 0, 0, 0, 0, 0, 0, 1)

            # base_link -> velodyne, gps, imu (고정된 변환)
            self.publish_tf("base_link", "velodyne", 0.5, 0.0, 1.2, 0, 0, 0, 1)
            self.publish_tf("base_link", "gps", -0.2, 0.0, 1.5, 0, 0, 0, 1)
            self.publish_tf("base_link", "imu", 0.0, 0.0, 1.0, 0, 0, 0, 1)

            # Odometry 메시지 생성 및 발행
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = 0  # odom -> base_link 변환 고정
            odom.pose.pose.position.y = 0
            odom.pose.pose.position.z = 0

            odom.pose.pose.orientation.x = 0  # odom -> base_link 변환을 단위 quaternion으로 고정
            odom.pose.pose.orientation.y = 0
            odom.pose.pose.orientation.z = 0
            odom.pose.pose.orientation.w = 1

            self.odom_pub.publish(odom)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        OdomPublisher()
    except rospy.ROSInterruptException:
        pass