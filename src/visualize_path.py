#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Rename to GPSVisualizer

import json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from pyproj import Proj, transform
import tf

def gps_to_utm(lat, lon):
    wgs84 = Proj(proj="latlong", datum="WGS84")
    utm = Proj(proj="utm", zone=52, datum="WGS84")  # Adjust zone as needed
    x, y = transform(wgs84, utm, lon, lat)  # Note: (lon, lat) order
    return x, y

def create_marker(marker_id, marker_type, position, scale, color):
    marker = Marker()
    marker.header.frame_id = "waypoint"
    marker.header.stamp = rospy.Time.now()  # Ensures consistent timestamp
    marker.ns = "waypoint_visualization"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.lifetime = rospy.Duration(0)

    return marker

def parse_json_and_visualize(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    marker_array = MarkerArray()
    node_positions = {}

    # 첫 번째 노드의 UTM 좌표와 고도를 기준점으로 설정
    first_node = data["Node"][0]
    ref_lat, ref_lon, ref_alt = first_node["GpsInfo"]["Lat"], first_node["GpsInfo"]["Long"], first_node["GpsInfo"]["Alt"]
    ref_x, ref_y = gps_to_utm(ref_lat, ref_lon)
    ref_z = ref_alt

    # Process Nodes
    for i, node in enumerate(data["Node"]):
        node_id = node["ID"]
        gps_info = node["GpsInfo"]
        lat, lon, alt = gps_info["Lat"], gps_info["Long"], gps_info["Alt"]
        utm_x, utm_y = gps_to_utm(lat, lon)

        # 기준점을 기준으로 상대 좌표 계산
        relative_x = utm_x - ref_x
        relative_y = utm_y - ref_y
        relative_z = alt
        position = [relative_x, relative_y, relative_z]
        node_positions[node_id] = position

        cube_marker = create_marker(
            marker_id=i,
            marker_type=Marker.CUBE,
            position=position,
            scale=[0.5, 0.5, 0.5],
            color=[0.0, 1.0, 0.0, 1.0]  # Green
        )
        marker_array.markers.append(cube_marker)

    # Process Links
    for j, link in enumerate(data.get("Link", [])):
        from_node = link["FromNodeID"]
        to_node = link["ToNodeID"]

        if from_node in node_positions and to_node in node_positions:
            line_marker = Marker()
            line_marker.header.frame_id = "waypoint"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "waypoint_visualization"
            line_marker.id = len(data["Node"]) + j
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.1
            line_marker.color.r = 1.0
            line_marker.color.g = 0.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            # 링크의 시작점과 끝점 좌표를 추가
            start_position = node_positions[from_node]
            end_position = node_positions[to_node]
            line_marker.points.append(Point(*start_position))
            line_marker.points.append(Point(*end_position))

            marker_array.markers.append(line_marker)

    return marker_array, ref_x, ref_y, ref_z


def main():
    rospy.init_node('waypoint_visualization')
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10, latch=True)
    tf_broadcaster = tf.TransformBroadcaster()

    file_path = '/home/ros/SCV2/src/scv_system/global_path/ROS_PathPlanning_pkg/src/20250107_waypoint_graph_2.json'  # Update with actual path

    # JSON 파일을 읽어 MarkerArray와 기준 좌표(Utm 좌표 및 고도)를 생성
    marker_array, ref_x, ref_y, ref_z = parse_json_and_visualize(file_path)

    #rate = rospy.Rate(1)  # 2 Hz
    while not rospy.is_shutdown():
        # TF 브로드캐스트 수행 (기준 좌표 사용)
        current_time = rospy.Time.now()
        tf_broadcaster.sendTransform(
            (ref_x, ref_y, ref_z),  # Translation (기준 Lat, Long, Alt에 해당하는 UTM 좌표)
            (0, 0, 0, 1),           # Rotation (identity quaternion)
            current_time,
            "waypoint",
            "map"
        )
        # 미리 생성한 MarkerArray를 Publish
        marker_pub.publish(marker_array)
        #rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
