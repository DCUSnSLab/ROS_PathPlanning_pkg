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
        self.orientation = (0.0, 0.0, 0.0, 1.0)

        self.rate = rospy.Rate(10)  # 10Hz
        self.run()

    def gps_callback(self, msg):
        current_utm = utm.from_latlon(msg.latitude, msg.longitude)
        self.gps_data = (
            current_utm[0] - self.map_utm[0],
            current_utm[1] - self.map_utm[1],
            msg.altitude
        )

    def imu_callback(self, msg):
        self.orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def publish_tf(self, parent, child, x, y, z, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # map -> odom (고정 프레임, 최초 설정 후 변경 없음)
            self.publish_tf("map", "odom", 0, 0, 0, 0, 0, 0)

            # odom -> base_link (실시간 업데이트)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"

            t.transform.translation.x = self.gps_data[0]
            t.transform.translation.y = self.gps_data[1]
            t.transform.translation.z = 0

            t.transform.rotation.x = self.orientation[0]
            t.transform.rotation.y = self.orientation[1]
            t.transform.rotation.z = self.orientation[2]
            t.transform.rotation.w = self.orientation[3]

            self.br.sendTransform(t)

            # base_link -> velodyne, gps, imu (주기적으로 갱신)
            self.publish_tf("base_link", "velodyne", 0.5, 0.0, 1.2, 0, 0, 0)
            self.publish_tf("base_link", "gps", -0.2, 0.0, 1.5, 0, 0, 0)
            self.publish_tf("base_link", "imu", 0.0, 0.0, 1.0, 0, 0, 0)

            # Odometry 메시지 생성 및 발행
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"  # 기존 "map"에서 "odom"으로 변경
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.gps_data[0]
            odom.pose.pose.position.y = self.gps_data[1]
            odom.pose.pose.position.z = 0

            odom.pose.pose.orientation.x = self.orientation[0]
            odom.pose.pose.orientation.y = self.orientation[1]
            odom.pose.pose.orientation.z = self.orientation[2]
            odom.pose.pose.orientation.w = self.orientation[3]

            self.odom_pub.publish(odom)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        OdomPublisher()
    except rospy.ROSInterruptException:
        pass
