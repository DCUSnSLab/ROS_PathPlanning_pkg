#!/usr/bin/python2
# -*- coding: utf-8 -*-

import rospy
import json
import math
import os
import sys

from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry

import cProfile

class Graph(object):
	def __init__(self, init_graph):
		self.nodes = [str(n) for n in range(len(init_graph))]
		# 노드 이름 정의
		self.graph = self.construct_graph(init_graph)

	def construct_graph(self, init_graph):
		# init_graph에 명시된 값을 바탕으로 그래프를 생성한다.
		print(init_graph)
		graph = {}
		for name in init_graph:
			graph[name] = {}

		graph.update(init_graph)

		for node, edges in graph.items():
			for adjacent_node, value in edges.items():
				if graph[adjacent_node].get(node, False) == False:
					graph[adjacent_node][node] = value

		return graph

	def get_nodes(self):
		return self.nodes

	def get_outgoing_edges(self, node):
		connections = []
		for out_node in self.nodes:
			if self.graph[node].get(out_node, False) != False:
				connections.append(out_node)
		return connections

	def value(self, node1, node2):
		''' node1, node2의 거리에 해당하는 값을 리턴한다. '''
		return self.graph[node1][node2]

class Dijkstra():
	def __init__(self, vertices):
		print("class Dijkstra():")
		self.vertices = vertices
		self.init_graph = {}

		for idx, vertex in enumerate(vertices):
			self.init_graph[str(idx)] = {}
			if vertex["adjacent"]:  # 인접한 Vertex가 있는 경우
				for adjacent in vertex["adjacent"]:  # 각 인접 vertex에 대한 dict 생성 및 거리값 초기화
					self.init_graph[str(idx)][adjacent] = self.calcdistance(idx, adjacent)

		self.graph = Graph(self.init_graph)
		print("Dijkstra() called")

		# self.graph_visualize(vertices, path) # 생성된 경로 시각화 함수
	def insert_vertex(self):
		pass

	def find_nearest_vertex(self, target_vertex):

		distance = sys.maxsize
		index = None
		for idx, vertex in enumerate(self.vertices):
			result = self.calcdistance_between_vertex(target_vertex, vertex)
			if result < distance:
				distance = result
				index = idx
		return index

	def calc_path(self, start, goal):
		previous_nodes, shortest_path = self.dijkstra_algorithm(graph=self.graph, start_node=start)
		path = self.print_result(previous_nodes, shortest_path, start_node=start, target_node=goal)
		self.path = path

	def dijkstra_algorithm(self, graph, start_node):
		unvisited_nodes = list(graph.get_nodes())

		# 이 dict를 통해 각 노드 방문 비용을 절약하고 그래프를 따라 이동할 때 갱신한다.
		shortest_path = {}

		# 이 dict를 통해 지금까지 발견된 노드에 대한 알려진 최단 경로 저장
		previous_nodes = {}

		# 미방문한 노드들에 대해서는 표현가능한 최대 값 사용
		max_value = sys.maxsize
		for node in unvisited_nodes:
			shortest_path[node] = max_value

		# 시작 노드에 대한 최단 경로는 0
		shortest_path[start_node] = 0

		# 모든 노드를 방문할 때 까지 수행
		while unvisited_nodes:
			# 아래 코드에서는 점수가 가장 낮은 노드를 찾는다
			current_min_node = None
			for node in unvisited_nodes:
				if current_min_node == None:
					current_min_node = node
				elif shortest_path[node] < shortest_path[current_min_node]:
					current_min_node = node

			# 현재 노드 이웃을 검색하고 거리를 업데이트
			neighbors = graph.get_outgoing_edges(current_min_node)
			for neighbor in neighbors:
				tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
				if tentative_value < shortest_path[neighbor]:
					shortest_path[neighbor] = tentative_value
					previous_nodes[neighbor] = current_min_node

			# 이웃을 방문한 후 노드를 "방문함"으로 표시합니다.
			unvisited_nodes.remove(current_min_node)

		return previous_nodes, shortest_path

	def print_result(self, previous_nodes, shortest_path, start_node, target_node):
		path = []
		refined_path = list()
		node = target_node

		while node != start_node:  # 시작 노드에 도달할 때 까지 반복
			path.append(node)
			node = previous_nodes[node]

		path.append(start_node)

		print("최단 경로에 대한 거리값 : {}.".format(shortest_path[target_node]))
		print(" -> ".join(reversed(path)))
		for vertex in reversed(path):
			refined_path.append(vertex)
		return refined_path

	def get_path(self):
		return self.path

	def calcdistance_between_vertex(self, start, dst):
		""" 입력받은 start, dst Vertex의 거리 값 계산 후 리턴"""
		return math.sqrt(math.pow(start["xy"][0] - dst["xy"][0], 2) + math.pow(start["xy"][1] - dst["xy"][1], 2))

	def calcdistance(self, start, dst):
		""" 입력받은 start, dst Vertex의 거리 값 계산 후 리턴"""
		return math.sqrt(math.pow(self.vertices[int(start)]["xy"][0] - self.vertices[int(dst)]["xy"][0], 2) + math.pow(self.vertices[int(start)]["xy"][1] - self.vertices[int(dst)]["xy"][1], 2))

class DefinedWaypoints():
	def __init__(self):
		print("DefinedWaypoints() __init__ called")
		self.pub = rospy.Publisher("/defined_vertices", MarkerArray, queue_size=1)
		self.waypointlist = MarkerArray()
		self.rate = rospy.Rate(1)

		self.pointlist = list()

		self.start_vertex = None

		self.json_file_path = rospy.get_param('~json_file')

		with open(self.json_file_path) as f:
			json_input = json.load(f)

		print("self.dijkstra = Dijkstra(json_input) before")
		self.dijkstra = Dijkstra(json_input)
		print("self.dijkstra = Dijkstra(json_input) after")

		for vertex in json_input:
			self.pointlist.append(vertex["xy"])


		for idx, point in enumerate(self.pointlist):
			self.add_waypoint(idx, point)

		self.nav_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_point_callback, queue_size=1)
		self.nav_start_sub = rospy.Subscriber("/odom", Odometry, self.current_pos_callback, queue_size=1)
		rospy.spin()

	def goal_point_callback(self, data):
		print("data")
		print(data)
		refined_data = {
			"xy": [data.pose.position.x, data.pose.position.y],
			"adjacent": None
		}
		start_nearest_vertex = self.dijkstra.find_nearest_vertex(refined_data)
		print(start_nearest_vertex)
		print(self.dijkstra.vertices[start_nearest_vertex])
		goal_nearest_vertex = self.dijkstra.find_nearest_vertex(refined_data)
		"""
		현재 차량의 좌표와 가장 가까운 Vertex를 고른 뒤,
		해당 Vertex ~ 목표와 가장 가까운 Vertex 까지의 경로 계산
		그러나 원본 Graph에 영향이 없는 한도에서 진행해야 함.
		"""
		self.dijkstra.calc_path("0", "1")
		path = self.dijkstra.get_path()
		print(path)

	def current_pos_callback(self, data):
		self.start_vertex = data

	def publish(self):
		self.pub.publish(self.waypointlist)

	def add_waypoint(self, idx, point):
		waypoint = Marker()

		waypoint.ns = str(idx)  # Marker namespace
		waypoint.id = idx  # Marker id value, no duplicates
		waypoint.text = str(idx)  # Marker namespace

		waypoint.type = 8  # line strip
		waypoint.lifetime = rospy.Duration.from_sec(100)
		waypoint.header = self.make_header("map")

		waypoint.action = 0 # ?

		# Set waypoint size
		waypoint.scale.x = 0.5
		waypoint.scale.y = 0.5
		waypoint.scale.z = 0.1

		# Set waypoint color, alpha
		waypoint.color.r = 0.85
		waypoint.color.g = 0.25
		waypoint.color.b = 0.25
		waypoint.color.a = 1.0

		pt = Point32()
		pt.x = point[0]
		pt.y = point[1]
		pt.z = 0.0 # Waypoint의 z축은 현재는 불필요함
		waypoint.points.append(pt)
		self.waypointlist.markers.append(waypoint)

	def make_header(self, frame_id, stamp=None):
		if stamp == None:
			stamp = rospy.Time.now()
		header = Header()
		header.stamp = stamp
		header.frame_id = frame_id
		return header

if __name__=="__main__":
	rospy.init_node("trajectory_search")
	try:
		DefinedWaypoints()
	except:
		pass

	# while True:
	# 	dw.publish()