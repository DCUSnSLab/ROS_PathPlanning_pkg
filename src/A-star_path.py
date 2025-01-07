#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from nav_msgs.msg import Path
from morai_msgs.msg import GPSMessage
import math
import heapq
from pyproj import Proj, transform

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')

        self.graph = None  # Change to None to allow lazy initialization
        self.current_position = None
        self.goal_position = None

        self.wgs84 = Proj(init='epsg:4326')  # WGS84 coordinate system
        self.utm = Proj(zone=52, init='epsg:32633')  # UTM zone 33N (example, change based on your region)
        # utm = Proj(proj="utm", datum="WGS84")  # Adjust zone as needed

        self.marker_array_received = False  # Track whether the marker array has been processed

        rospy.Subscriber('/visualization_marker_array', MarkerArray, self.marker_array_callback)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.utm_marker_pub = rospy.Publisher('/ego_vehicle', Marker, queue_size=10)

    def marker_array_callback(self, marker_array):
        """Callback to process the MarkerArray and construct the graph."""
        if not self.marker_array_received:  # Process only once
            rospy.loginfo("Processing MarkerArray for the first time.")
            self.graph = {}
            for marker in marker_array.markers:
                node = (marker.pose.position.x, marker.pose.position.y)
                neighbors = [(edge.x, edge.y) for edge in marker.points]
                self.graph[node] = neighbors
            print(self.graph)
            self.marker_array_received = True

    def gps_callback(self, gps_msg):
        """Callback to update the current position with UTM coordinates."""
        rospy.loginfo(f"Received GPS coordinates: Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}")
        utm_x, utm_y = transform(self.wgs84, self.utm, gps_msg.longitude, gps_msg.latitude)
        rospy.loginfo(f"Converted UTM coordinates: UTM_X: {utm_x}, UTM_Y: {utm_y}")
        self.current_position = (utm_x, utm_y)
        self.publish_utm_marker(utm_x, utm_y, gps_msg.altitude)

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

    def goal_callback(self, goal_msg):
        print("goal callback")
        """Callback to update the goal position and compute the path."""
        self.goal_position = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        if self.current_position and self.goal_position:
            print(self.current_position, self.goal_position)
            path = self.a_star(self.current_position, self.goal_position)
            print(path)
            self.publish_path(path)

    def heuristic(self, node, goal):
        """Heuristic function for A* (Euclidean distance)."""
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def a_star(self, start, goal):
        """A* algorithm implementation."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {node: float('inf') for node in self.graph}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.graph}
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.graph.get(current, []):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # Return an empty path if no path is found

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

        for node in path:
            pose = Point()
            pose.x, pose.y = node
            print(pose.x, pose.y)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.spin()