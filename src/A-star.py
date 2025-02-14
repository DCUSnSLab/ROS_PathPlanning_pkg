#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tkinter import Tk, filedialog, messagebox
from path_planning.srv import CreateGraph, MapGraph
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
import math
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, CRS, transform, Transformer
import tf
from tf.transformations import quaternion_from_euler

from visualize_path import parse_json_and_visualize
from path_planning.msg import GraphMsg, Node, NodeArray, Link, LinkArray

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')

        self.origin_utm = None
        self.waypoint_relative_utm = None
        self.waypoint_TF = False

        self.map_gps_coordinates = (37.23923857150045, 126.7731611307903, 28.940864029884338)  # Lat, Long, Alt
        self.map_utm = (302473.690, 4123735.933)

        self.nodes = None
        self.links = None

        self.file_path = None

        self.utm_zone = 52  # UTM zone 52N (example, change based on your region)

        self.transformer_to_utm = Transformer.from_crs("epsg:4326", f"epsg:326{self.utm_zone}", always_xy=True)
        self.transformer_to_gps = Transformer.from_crs(f"epsg:326{self.utm_zone}", "epsg:4326", always_xy=True)

        self.tf_broadcaster = tf.TransformBroadcaster()

        self.rviz_correction_val = (0, 0, 0)

        self.load_graph_data()

        self.current_position_gps = None
        self.current_position_utm = None
        self.goal_position = None

        self.heading_quaternion = None

        self.marker_array, self.ref_x, self.ref_y, self.ref_z = parse_json_and_visualize(self.file_path, self.rviz_correction_val)

        self.wgs84 = Proj(init='epsg:4326')  # WGS84 coordinate system
        self.utm = Proj(zone=self.utm_zone, init='epsg:32633')

        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.odom_pub = rospy.Publisher('/odom/coordinate', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.utm_marker_pub = rospy.Publisher('/ego_vehicle', Marker, queue_size=10)
        self.utm_coord = rospy.Publisher('/ego_utm', Point, queue_size=10)

        self.debug_arrow = rospy.Publisher('/correction_pose', PoseStamped, queue_size=10)

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10, latch=True)

        self.marker_pub.publish(self.marker_array)

    def gps_callback(self, gps_msg):
        """Callback to update the current position with UTM coordinates."""
        self.current_position_gps = (gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
        utm_x, utm_y = self.transformer_to_utm.transform(gps_msg.longitude, gps_msg.latitude)
        self.current_position_utm = (utm_x, utm_y)

        # origin_utm은 Node의 첫 번째 좌표로 설정되므로 여기서는 설정하지 않음

        if self.waypoint_relative_utm == None:
            self.waypoint_relative_utm = (utm_x - self.origin_utm[0], utm_y - self.origin_utm[1])
        relative_x = utm_x - self.origin_utm[0]
        relative_y = utm_y - self.origin_utm[1]

        self.publish_odometry(self.waypoint_relative_utm[0], self.waypoint_relative_utm[1], gps_msg.altitude, self.heading_quaternion)

        # self.publish_utm_marker(utm_x, utm_y, gps_msg.altitude)
        self.publish_utm_marker(self.waypoint_relative_utm[0], self.waypoint_relative_utm[1], gps_msg.latitude, gps_msg.longitude, gps_msg.altitude, self.heading_quaternion) # Display with relative coord

        utm_coord = Point()
        # utm_coord.x = relative_x
        # utm_coord.y = relative_y
        utm_coord.x = relative_x + self.rviz_correction_val[0]
        utm_coord.y = relative_y + self.rviz_correction_val[1]
        self.utm_coord.publish(utm_coord)

    def publish_odometry(self, x, y, z, quaternion):
        """Odometry 메시지를 생성하고 발행"""
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        # 위치 정보 설정
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        # 방향 정보 설정 (쿼터니언)
        if quaternion is not None and quaternion.size == 4:  # 길이 체크
            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
        else:
            odom_msg.pose.pose.orientation.w = 1.0  # 기본 값

        # 발행
        self.odom_pub.publish(odom_msg)

    def ego_callback(self, ego_msg):
        self.heading_quaternion = self.angle_to_quaternion(ego_msg.heading)

    def goal_callback(self, goal_msg):
        corrected_pose = PoseStamped()
        corrected_pose.header = goal_msg.header  # 기존 헤더 유지

        # 위치(Position) 보정
        corrected_pose.pose.position.x = goal_msg.pose.position.x + self.origin_utm[0] - self.rviz_correction_val[0]
        corrected_pose.pose.position.y = goal_msg.pose.position.y + self.origin_utm[1] - self.rviz_correction_val[1]
        corrected_pose.pose.position.z = goal_msg.pose.position.z + self.rviz_correction_val[2]

        # 방향(Orientation)은 그대로 유지
        corrected_pose.pose.orientation = goal_msg.pose.orientation

        # 수정된 PoseStamped 발행
        self.debug_arrow.publish(corrected_pose)

        # Start 및 Goal 노드 삭제
        self.remove_node("Start")
        self.remove_node("Goal")
        self.remove_link("Start")
        self.remove_link("Goal")


        # Start 노드 생성
        start_node = self.add_node(
            id="Start",
            lat=self.current_position_gps[0],
            lon=self.current_position_gps[1],
            alt=self.current_position_gps[2],
            direction=True
        )

        rospy.loginfo(f"Start node")
        rospy.loginfo(f"Lat : {self.current_position_gps[0]} ")
        rospy.loginfo(f"Long : {self.current_position_gps[1]} ")
        rospy.loginfo(f"Alt : {self.current_position_gps[2]} ")

        # RViz Goal 좌표를 self.origin_utm 기준으로 보정
        relative_goal_x = goal_msg.pose.position.x
        relative_goal_y = goal_msg.pose.position.y
        relative_goal_z = goal_msg.pose.position.z
        adjusted_goal_x = self.origin_utm[0] + relative_goal_x - self.rviz_correction_val[0]
        adjusted_goal_y = self.origin_utm[1] + relative_goal_y - self.rviz_correction_val[1]
        adjusted_goal_z = relative_goal_z + self.rviz_correction_val[2]

        # 보정된 UTM → GPS 변환
        goal_lat, goal_lon = self.utm_to_gps(adjusted_goal_x, adjusted_goal_y, self.utm_zone, True)

        goal_node = self.add_node(
            id="Goal",
            lat=goal_lat,
            lon=goal_lon,
            alt=adjusted_goal_z,
            direction=False
        )

        rospy.loginfo(f"Goal node")
        rospy.loginfo(f"Lat : {goal_lat}")
        rospy.loginfo(f"Long : {goal_lon}")
        rospy.loginfo(f"Alt : {adjusted_goal_z}")

        # A* 경로 계산 및 Publish
        path = self.find_path(start_node, goal_node)
        if path:
            self.publish_path(path)

    def angle_to_quaternion(self, yaw_deg):
        """
        -180 ~ 180 도 사이의 각도를 쿼터니언으로 변환합니다.

        :param yaw_deg: Yaw 각도 (도 단위)
        :return: 쿼터니언 (x, y, z, w)
        """
        # 각도를 라디안으로 변환
        yaw_rad = math.radians(yaw_deg)

        # 쿼터니언 계산 (Roll=0, Pitch=0, Yaw=yaw_rad)
        quaternion = quaternion_from_euler(0, 0, yaw_rad)

        return quaternion

    def load_graph_data(self):
        root = Tk()
        root.withdraw()  # Hide the root window
        root.title("Select a JSON File")

        self.file_path = filedialog.askopenfilename(
            title="Select a JSON File",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )

        if not self.file_path:
            rospy.logwarn("No file selected.")
            return

        rospy.loginfo(f"Selected file: {self.file_path}")
        self.call_service(self.file_path)

        # Node의 첫 번째 좌표를 UTM으로 변환하여 origin 설정
        if self.nodes:
            first_node = self.nodes.nodes[0]
            lat, lon = first_node.Lat, first_node.Long
            utm_x, utm_y = self.transformer_to_utm.transform(lon, lat)
            self.origin_utm = (utm_x, utm_y) # Coord for visualize markers 0, 0, 0 in RViz

            self.rviz_correction_val = (self.origin_utm[0] - self.map_utm[0], self.origin_utm[1] - self.map_utm[1], first_node.Alt)
            rospy.loginfo(f"Set origin UTM from first Node: {self.origin_utm}")

    def call_service(self, file_path):
        """Call the ROS service with the selected JSON file path."""
        rospy.wait_for_service('map_server')
        try:
            # Create a proxy for the service
            service_proxy = rospy.ServiceProxy('map_server', MapGraph)
            # Call the service with the file path
            response = service_proxy(file_path)
            rospy.loginfo("Service call successful!")

            self.nodes = response.map_graph.node_array
            self.links = response.map_graph.link_array

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            messagebox.showerror("Service Error", f"Service call failed: {e}")

    def heuristic(self, current_node, goal_node):
        """Euclidean distance as heuristic"""
        current = next((node for node in self.nodes.nodes if node.ID == current_node), None)
        goal = next((node for node in self.nodes.nodes if node.ID == goal_node), None)
        return math.sqrt((current.Lat - goal.Lat)**2 + (current.Long - goal.Long)**2)

    def get_neighbors(self, node_id):
        """Get neighbors of the current node from links"""
        neighbors = []
        for link in self.links.links:
            if link.FromNodeID == node_id:
                neighbors.append((link.ToNodeID, link.Length))
            #elif link.ToNodeID == node_id:
            #    neighbors.append((link.FromNodeID, link.Length))
        return neighbors

    def find_path(self, start_id, goal_id):
        """Find the shortest path using A*"""
        open_set = set([start_id])
        came_from = {}

        g_score = {}
        f_score = {}

        # Need to find nearst and possible nodes in start node

        for node in self.nodes.nodes:
            g_score[node.ID] = float('inf')
        g_score[start_id] = 0

        for node in self.nodes.nodes:
            f_score[node.ID] = float('inf')
        f_score[start_id] = self.heuristic(start_id, goal_id)

        while open_set:
            current = min(open_set, key=lambda node: f_score[node])
            #print("current")
            #print(current)

            if current == goal_id:
                #print("came_from")
                #print(came_from)
                #print("current")
                #print(current)
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor, link_cost in self.get_neighbors(current):
                tentative_g_score = g_score[current] + link_cost
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_id)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        rospy.loginfo(f"No path found in current position")

        # When cannot found available path

        return None  # No path found

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from the came_from dictionary"""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return path

    def publish_utm_marker(self, utm_x, utm_y, lat, long, alt, heading):
        """Publish the UTM position as a visualization marker."""
        marker = Marker()
        # marker.header.frame_id = "ego_vehicle"
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "utm_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        # marker.pose.position.x = utm_x
        # marker.pose.position.y = utm_y
        # marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 4.0
        marker.scale.y = 4.0
        marker.scale.z = 4.0
        marker.color.a = 0.6  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # self.tf_broadcaster.sendTransform(
        #     # (self.ref_x, self.ref_y, self.ref_z),  # Translation (기준 Lat, Long, Alt에 해당하는 UTM 좌표)
        #     (utm_x, utm_y, alt),
        #     # (0, 0, 0),
        #     self.heading_quaternion,  # Rotation (identity quaternion)
        #     #(0, 0, 0, 1),
        #     rospy.Time.now(),
        #     "ego_vehicle",
        #     "base_link"
        # )

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

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371  # Earth's radius in km
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def find_nearest_node(self, lat, lon):
        min_distance = float('inf')
        nearest_node_id = None

        for node in self.nodes.nodes:
            node_lat = node.Lat
            node_lon = node.Long
            distance = self.haversine(lat, lon, node_lat, node_lon)

            if distance < min_distance:
                min_distance = distance
                nearest_node_id = node.ID

        return nearest_node_id, min_distance

    def calculate_utm_zone(self, lon):
        return int((lon + 180) // 6) + 1

    def add_node(self, id, lat, lon, alt, direction):
        nearest_node_id, distance = self.find_nearest_node(lat, lon)

        # GPS → UTM 변환 (Transformer 사용)
        utm_x, utm_y = self.transformer_to_utm.transform(lon, lat)

        # 상대 UTM 좌표 계산
        relative_x = utm_x - self.origin_utm[0]
        relative_y = utm_y - self.origin_utm[1]

        node = Node()
        node.ID = id
        node.AdminCode = ""
        node.NodeType = ""
        node.ITSNodeID = ""
        node.Maker =""
        node.UpdateDate = ""
        node.Version = ""
        node.Remark = ""
        node.HistType = ""
        node.HistRemark = ""
        node.Lat = lat
        node.Long = lon
        node.Alt = alt
        node.Easting = utm_x
        node.Northing = utm_y
        node.Zone = self.calculate_utm_zone(lon)

        self.nodes.nodes.append(node)

        link = Link()
        link.ID = str(id + "_link")
        link.AdminCode = ""
        link.RoadRank = ""
        link.RoadType = ""
        link.RoadNo = ""
        link.LinkType = ""
        link.LaneNo = ""
        link.R_LinkID = ""
        link.L_LinkID = ""
        link.FromNodeID = (id if direction else nearest_node_id)
        link.ToNodeID = (nearest_node_id if direction else id)
        link.SectionID = ""
        link.Length = 0
        link.ITSLinkID = ""
        link.Maker = ""
        link.UpdateDate = ""
        link.Version = ""
        link.Remark = ""
        link.HistType = ""
        link.HistRemark = ""

        self.links.links.append(link)

        return id

    def remove_node(self, node_id):
        """Remove a node from the graph and nodes."""
        for idx, node in enumerate(self.nodes.nodes):
            if node.ID == node_id:
                rospy.logwarn(f"Node ID {node_id} Deleted.")
                del self.nodes.nodes[idx]

    def remove_link(self, link_id):
        for idx, link in enumerate(self.links.links):
            if link.ID == str(link_id + "_link"):
                rospy.logwarn(f"Link ID {link_id} Deleted.")
                del self.links.links[idx]

    def publish_path(self, path):
        """Publish the planned path as a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        #path_msg.header.frame_id = 'waypoint'
        path_msg.header.frame_id = 'map'

        for node_id in path:

            # node_info = self.nodes[node_id]
            node_info = next((node for node in self.nodes.nodes if node.ID == node_id), None)
            # gps_info = node_info['GpsInfo']
            # lat, lon, alt = gps_info['Lat'], gps_info['Long'], gps_info['Alt']
            lat, lon, alt = node_info.Lat, node_info.Long, node_info.Alt

            # GPS → UTM 변환 (Transformer 사용)
            utm_x, utm_y = self.transformer_to_utm.transform(lon, lat)

            # 상대 UTM 좌표 계산
            relative_x = utm_x - self.origin_utm[0]
            relative_y = utm_y - self.origin_utm[1]

            # PoseStamped 메시지를 생성하여 추가
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = relative_x + self.rviz_correction_val[0]
            pose_stamped.pose.position.y = relative_y + self.rviz_correction_val[1]
            #pose_stamped.pose.position.x = utm_x
            #pose_stamped.pose.position.y = utm_y
            pose_stamped.pose.position.z = self.rviz_correction_val[2]
            pose_stamped.pose.orientation.w = 1.0

            # Path 메시지에 추가
            path_msg.poses.append(pose_stamped)

        # 경로를 Publish
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        pathfinder = PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass