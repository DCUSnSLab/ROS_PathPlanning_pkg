#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tkinter import Tk, filedialog, messagebox
from path_planning.srv import CreateGraph
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from morai_msgs.msg import GPSMessage
import math
import heapq
import rospy
import json
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from pyproj import Proj, CRS, transform, Transformer
import tf

from visualize_path import parse_json_and_visualize

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')

        self.origin_utm = None

        self.graph = None  # Change to None to allow lazy initialization
        self.nodes = None

        self.file_path = None

        self.load_graph_data()
        self.current_position_gps = None
        self.current_position_utm = None
        self.goal_position = None

        self.tf_broadcaster = tf.TransformBroadcaster()

        self.northern_hemisphere = None # Northern and Southern Hemisphere distinction flag variable

        self.marker_array, self.ref_x, self.ref_y, self.ref_z = parse_json_and_visualize(self.file_path)

        self.utm_zone = 52 # UTM zone 52N (example, change based on your region)

        self.wgs84 = Proj(init='epsg:4326')  # WGS84 coordinate system
        # self.wgs84 = CRS.from_epsg(4326)
        self.utm = Proj(zone=self.utm_zone, init='epsg:32633')
        # utm = Proj(proj="utm", datum="WGS84")  # Adjust zone as needed

        self.transformer_to_utm = Transformer.from_crs("epsg:4326", f"epsg:326{self.utm_zone}", always_xy=True)
        self.transformer_to_gps = Transformer.from_crs(f"epsg:326{self.utm_zone}", "epsg:4326", always_xy=True)

        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.utm_marker_pub = rospy.Publisher('/ego_vehicle', Marker, queue_size=10)

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10, latch=True)

        rate = rospy.Rate(1)  # 2 Hz
        while not rospy.is_shutdown():
            # TF 브로드캐스트 수행 (기준 좌표 사용)
            current_time = rospy.Time.now()
            self.tf_broadcaster.sendTransform(
                # (self.ref_x, self.ref_y, self.ref_z),  # Translation (기준 Lat, Long, Alt에 해당하는 UTM 좌표)
                (0, 0, 0),
                (0, 0, 0, 1),  # Rotation (identity quaternion)
                current_time,
                "waypoint",
                "map"
            )
            # 미리 생성한 MarkerArray를 Publish
            self.marker_pub.publish(self.marker_array)
            rate.sleep()

    def load_graph_data(self):
        root = Tk()
        root.withdraw()  # Hide the root window
        root.title("Select a JSON File")

        # Open file dialog
        self.file_path = filedialog.askopenfilename(
            title="Select a JSON File",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )

        if self.file_path:
            rospy.loginfo("Selected file: %s", self.file_path)
            self.call_service(self.file_path)
        else:
            rospy.logwarn("No file selected.")

    def call_service(self, file_path):
        """Call the ROS service with the selected JSON file path."""
        rospy.wait_for_service('map_server')
        try:
            # Create a proxy for the service
            service_proxy = rospy.ServiceProxy('map_server', CreateGraph)
            # Call the service with the file path
            response = service_proxy(file_path)
            rospy.loginfo("Service call successful!")
            self.graph = json.loads(response.graph_json)
            self.nodes = json.loads(response.node_json)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            messagebox.showerror("Service Error", f"Service call failed: {e}")

    def gps_callback(self, gps_msg):
        """Callback to update the current position with UTM coordinates."""
        self.current_position_gps = (gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
        utm_x, utm_y = self.transformer_to_utm.transform(gps_msg.longitude, gps_msg.latitude)
        self.current_position_utm = (utm_x, utm_y)

        if self.origin_utm is None:
            self.origin_utm = (utm_x, utm_y)
            rospy.loginfo(f"Set origin UTM: {self.origin_utm}")

        relative_x = utm_x - self.origin_utm[0]
        relative_y = utm_y - self.origin_utm[1]

        # self.publish_utm_marker(utm_x, utm_y, gps_msg.altitude)
        self.publish_utm_marker(relative_x, relative_y, gps_msg.altitude) # Display with relative coord

    def publish_utm_marker(self, utm_x, utm_y, altitude):
        """Publish the UTM position as a visualization marker."""
        marker = Marker()
        marker.header.frame_id = "waypoint"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "utm_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = utm_x
        marker.pose.position.y = utm_y
        marker.pose.position.z = altitude
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 4.0
        marker.scale.y = 4.0
        marker.scale.z = 4.0
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.utm_marker_pub.publish(marker)

    def utm_to_gps(self, easting, northing, zone, northern_hemisphere=True):
        """
        Converts UTM coordinates to GPS coordinates (latitude, longitude).

        Parameters:
        - easting (float): Easting value of UTM coordinate.
        - northing (float): Northing value of UTM coordinate.
        - zone (int): UTM zone number.
        - northern_hemisphere (bool): True if the coordinate is in the northern hemisphere, False otherwise.

        Returns:
        - (float, float): Latitude and Longitude in degrees.
        """
        # Define UTM CRS (Coordinate Reference System)
        utm_crs = Proj(proj="utm", zone=zone, datum="WGS84", south=not northern_hemisphere)

        # Define WGS84 CRS for GPS coordinates
        wgs84_crs = Proj(proj="latlong", datum="WGS84")

        # Create a transformer
        transformer = Transformer.from_proj(utm_crs, wgs84_crs)

        # Transform UTM to GPS
        longitude, latitude = transformer.transform(easting, northing)

        return latitude, longitude

    def goal_callback(self, goal_msg):
        if "Start" in self.nodes:
            del self.nodes["Start"]
            if "Start" in self.graph:
                for neighbor, _ in self.graph["Start"]:
                    self.graph[neighbor] = [
                        (n, w) for n, w in self.graph[neighbor] if n != "Start"
                    ]
                del self.graph["Start"]

        if "Goal" in self.nodes:
            del self.nodes["Goal"]
            if "Goal" in self.graph:
                for neighbor, _ in self.graph["Goal"]:
                    self.graph[neighbor] = [
                        (n, w) for n, w in self.graph[neighbor] if n != "Goal"
                    ]
                del self.graph["Goal"]

        # print(goal_msg)
        # print(self.current_position_gps[0], self.current_position_gps[1])
        #print(self.graph)
        #print(self.nodes)
        print("self.current_position_gps")
        print(self.current_position_gps)
        start_node = self.add_node(id="Start", lat=self.current_position_gps[0], lon=self.current_position_gps[1], alt=self.current_position_gps[2], graph=self.graph, nodes=self.nodes)

        # RViz Goal 좌표를 self.origin_utm을 기준으로 보정하여 가상의 UTM 좌표 계산
        relative_goal_x = goal_msg.pose.position.x
        relative_goal_y = goal_msg.pose.position.y
        relative_goal_z = goal_msg.pose.position.z

        # 보정된 UTM 좌표 계산
        adjusted_goal_x = self.origin_utm[0] + relative_goal_x
        adjusted_goal_y = self.origin_utm[1] + relative_goal_y
        adjusted_goal_z = relative_goal_z  # Z 값은 그대로 사용

        #print("goal callback")
        #print(goal_msg)

        # goal_gps = self.utm_to_gps(goal_msg.pose.position.x, goal_msg.pose.position.y, self.utm_zone, True)

        goal_lat, goal_lon = self.utm_to_gps(adjusted_goal_x, adjusted_goal_y, self.utm_zone, True)
        print("goal_lat, goal_lon")
        print(goal_lat, goal_lon)
        # print(goal_gps[0])
        # print(goal_gps[1])
        goal_node = self.add_node(id="Goal", lat=goal_lat, lon=goal_lon, alt=28.9, graph=self.graph, nodes=self.nodes) # Temp val for altitude

        path = self.a_star(self.graph, self.nodes, start_node, goal_node)
        print("path output")
        print(path)

        if path:
            self.publish_path(path)

    def heuristic(self, node, goal):
        """Heuristic function for A* (Euclidean distance)."""
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371  # Earth's radius in km
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def a_star(self, graph, nodes, start, goal):
        def heuristic(node_id):
            lat1, lon1 = nodes[node_id]['GpsInfo']['Lat'], nodes[node_id]['GpsInfo']['Long']
            lat2, lon2 = nodes[goal]['GpsInfo']['Lat'], nodes[goal]['GpsInfo']['Long']
            return self.haversine(lat1, lon1, lat2, lon2)

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {node: float('inf') for node in graph}
        g_score[start] = 0
        f_score = {node: float('inf') for node in graph}
        f_score[start] = heuristic(start)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor, weight in graph[current]:
                tentative_g_score = g_score[current] + weight
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # Path not found

    def find_nearest_node(self, lat, lon, nodes):
        min_distance = float('inf')
        nearest_node_id = None

        for node_id, node in nodes.items():
            node_lat = node['GpsInfo']['Lat']
            node_lon = node['GpsInfo']['Long']
            distance = self.haversine(lat, lon, node_lat, node_lon)

            if distance < min_distance:
                min_distance = distance
                nearest_node_id = node_id

        return nearest_node_id, min_distance

    def add_node(self, id, lat, lon, alt, graph, nodes):
        node_id = id
        nearest_node_id, distance = self.find_nearest_node(lat, lon, nodes)

        # GPS → UTM 변환
        utm_x, utm_y = transform(self.wgs84, self.utm, lon, lat)

        # 상대 UTM 좌표 계산
        relative_x = utm_x - self.origin_utm[0]
        relative_y = utm_y - self.origin_utm[1]

        nodes[node_id] = {
            "ID": node_id,
            "GpsInfo": {"Lat": lat, "Long": lon, "Alt": alt},
            "UTM": {"X": relative_x, "Y": relative_y},
        }

        graph[node_id] = [(nearest_node_id, distance)]
        graph[nearest_node_id].append((node_id, distance))

        return node_id

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from the came_from map."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return path

    def publish_path(self, path):
        """Publish the planned path as a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'waypoint'

        for node_id in path:
            if node_id not in self.nodes:
                rospy.logwarn(f"Node ID {node_id} not found in self.nodes.")
                continue

            node_info = self.nodes[node_id]
            gps_info = node_info['GpsInfo']
            print(gps_info)
            lat, lon, alt = gps_info['Lat'], gps_info['Long'], gps_info['Alt']

            # GPS 정보를 UTM 좌표로 변환
            utm_x, utm_y = transform(self.wgs84, self.utm, lon, lat)

            # 상대 UTM 좌표 계산
            relative_x = utm_x - self.origin_utm[0]
            relative_y = utm_y - self.origin_utm[1]

            # PoseStamped 메시지를 생성하여 추가
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'waypoint'
            #pose_stamped.pose.position.x = utm_x
            #pose_stamped.pose.position.y = utm_y
            pose_stamped.pose.position.x = relative_x
            pose_stamped.pose.position.y = relative_y
            pose_stamped.pose.position.z = alt

            # Path 메시지에 추가
            path_msg.poses.append(pose_stamped)

        # 경로를 Publish
        self.path_pub.publish(path_msg)


if __name__ == '__main__':
    planner = PathPlanner()
    rospy.spin()