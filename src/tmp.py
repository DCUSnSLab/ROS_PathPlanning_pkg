#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from tkinter import Tk, filedialog, messagebox
from path_planning.srv import CreateGraph, MapGraph
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
import math
import heapq
import json
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, CRS, transform, Transformer
import tf
from tf.transformations import quaternion_from_euler

from visualize_path import parse_json_and_visualize
from path_planning.msg import Graph, Node, NodeArray, Link, LinkArray
from scipy.spatial.transform import Rotation as R

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
import utm


class TFBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)

        self.map_gps_coordinates = (37.23923857150045, 126.7731611307903, 28.940864029884338) # Lat, Long, Alt
        self.map_utm = None

        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)

        self.static_transforms = [
            self.create_static_tf("base_link", "velodyne", 0.5, 0.0, 1.2, 0, 0, 0),
            self.create_static_tf("base_link", "gps", -0.2, 0.0, 1.5, 0, 0, 0),
            self.create_static_tf("base_link", "imu", 0.0, 0.0, 1.0, 0, 0, 0)
        ]

    def create_static_tf(self, parent, child, x, y, z, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def gps_callback(self, msg):
        current_utm = utm.from_latlon(msg.latitude, msg.longitude)
        if self.map_gps_coordinates is None:
            self.map_utm = current_utm
        else:
            self.map_utm = utm.from_latlon(self.map_gps_coordinates[0], self.map_gps_coordinates[1])
        self.gps_data = (current_utm[0] - self.map_utm[0], current_utm[1] - self.map_utm[1], msg.altitude)

        self.static_br.sendTransform(self.static_transforms)
        self.run()

    def run(self):
        if hasattr(self, 'gps_data'):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"

            t.transform.translation.x = self.gps_data[0]  # UTM 변환 후 차이값 적용
            t.transform.translation.y = self.gps_data[1]
            t.transform.translation.z = self.gps_data[2]

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)


if __name__ == '__main__':
    try:
        TFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
