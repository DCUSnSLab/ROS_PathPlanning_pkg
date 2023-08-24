#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
import sys

from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry

class Graph(object):
	def __init__(self, init_graph):
		self.nodes = [n for n in range(len(init_graph))]
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
		print("Dijkstra() __init__ called.")
		self.vertices = vertices # 원본 json 파일
		self.init_graph = {}
		self.construct_graph()

	def construct_graph(self):
		print("construct_graph() called.")
		for idx, vertex in enumerate(self.vertices):
			self.init_graph[idx] = {}
			print("for loop")
			print(vertex["adjacent"])
			if vertex["adjacent"]:  # 인접한 Vertex가 있는 경우
				print("if vertex['adjacent']:")
				for adjacent in vertex["adjacent"]:  # 각 인접 vertex에 대한 dict 생성 및 거리값 초기화
					print("adjacent")
					print(adjacent)
					print("for adjacent in vertex")
					self.init_graph[idx][adjacent] = self.calcdistance(idx, adjacent)
					print("calcdistance end")

		self.graph = Graph(self.init_graph)
		print("construct_graph() end.")
		# self.graph_visualize(vertices, path) # 생성된 경로 시각화 함수
	def insert_vertex(self, target_vertex, adjacent_vertex_idx, adjacent_vertex):
		"""
		self.init_graph에 현재 위치, 목표 위치 값을 추가한 뒤
		self.graph에 새로 생성된 그래프를 추가
		"""
		self.init_graph[len(self.init_graph)] = {}
		self.init_graph[len(self.init_graph) - 1][adjacent_vertex_idx] = self.calcdistance_between_vertex(target_vertex, adjacent_vertex)

	def find_nearest_vertex(self, target_vertex):
		"""
		target_vertex와 가장 근접한 vertex를 찾은 뒤
		해당 vertex의 index와 실제 값을 리턴한다.
		"""
		distance = sys.maxsize
		nearest_vertex = None
		index = None
		for idx, vertex in enumerate(self.vertices):
			result = self.calcdistance_between_vertex(target_vertex, vertex)
			if result < distance:
				distance = result
				nearest_vertex = self.vertices[idx]
				index = idx
		return nearest_vertex, index

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

		for vertex in reversed(path):
			refined_path.append(vertex)
		# return refined_path
		return path

	def get_path(self):
		return self.path

	def calcdistance_between_vertex(self, start, dst):
		""" 입력받은 start, dst Vertex의 거리 값 계산 후 리턴"""
		return math.sqrt(math.pow(start["xy"][0] - dst["xy"][0], 2) + math.pow(start["xy"][1] - dst["xy"][1], 2))

	def calcdistance(self, start, dst):
		print("calcdistance() called.")
		""" 입력받은 start, dst Vertex의 index 값을 사용하여 거리 값 계산 후 리턴"""
		return math.sqrt(math.pow(self.vertices[int(start)]["xy"][0] - self.vertices[int(dst)]["xy"][0], 2) + math.pow(self.vertices[int(start)]["xy"][1] - self.vertices[int(dst)]["xy"][1], 2))

class DefinedWaypoints():
	def __init__(self):
		print("DefinedWaypoints() __init__ called")
		self.pub = rospy.Publisher("/defined_vertices", MarkerArray, queue_size=1)
		self.path_pub = rospy.Publisher("/refined_vertices", MarkerArray, queue_size=1)
		self.waypointlist = MarkerArray()
		self.rate = rospy.Rate(1)

		self.pointlist = list()

		self.start_vertex = None

		args = sys.argv
	
		self.json_file_path = None
	
		if (args[1] != None and args[1][0:6] != "__name"):
			self.json_file_path = args[1]

		else:
			self.json_file_path = rospy.get_param('~json_file')

		with open(self.json_file_path) as f:
			json_input = json.load(f)

		self.dijkstra = Dijkstra(json_input)


		for vertex in json_input:
			self.pointlist.append(vertex["xy"])

		for idx, point in enumerate(self.pointlist):
			self.waypointlist.markers.append(self.add_waypoint(idx, point))

		self.nav_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_point_callback, queue_size=1)
		self.nav_start_sub = rospy.Subscriber("/odom", Odometry, self.current_pos_callback, queue_size=1)
		print("DefinedWaypoints() __init__ end")
		self.pub.publish(self.waypointlist)
		rospy.spin()

	def goal_point_callback(self, data):
		goal_position = {
			"xy": [data.pose.position.x, data.pose.position.y],
			"adjacent": None
		}
		current_position = {
			"xy": [self.start_vertex.pose.pose.position.x, self.start_vertex.pose.pose.position.y],
			"adjacent": None
		}
		start_nearest_vertex, start_nearest_vertex_idx = self.dijkstra.find_nearest_vertex(goal_position)
		goal_position["adjacent"] = start_nearest_vertex_idx

		goal_nearest_vertex, goal_nearest_vertex_idx = self.dijkstra.find_nearest_vertex(current_position)
		current_position["adjacent"] = goal_nearest_vertex_idx

		self.dijkstra.insert_vertex(current_position, start_nearest_vertex_idx, start_nearest_vertex)
		self.dijkstra.insert_vertex(goal_position, goal_nearest_vertex_idx, goal_nearest_vertex)
		self.dijkstra.construct_graph()

		self.dijkstra.calc_path(len(self.dijkstra.init_graph) - 2, len(self.dijkstra.init_graph) - 1)

		# 생성된 경로 값 가져오기
		path = self.dijkstra.get_path()

		# 생성된 경로 path의 x, y 좌표만을 저장하기 위한 리스트 변수
		refined_path = list()

		# 생성된 경로 path의 x, y 좌표를 사용하여 생성된 Marker() 메시지를 저장하기 위한 변수
		refined_waypointlist = MarkerArray()

		# 생성된 경로 path를 사용하여 (x, y) 좌표 형태로 기록된 리스트를 만든다.
		for idx in path[1:len(path) - 1]:
			print(idx)
			refined_path.append(self.dijkstra.vertices[int(idx)]["xy"])

		refined_path.insert(0, current_position["xy"])
		refined_path.append(goal_position["xy"])

		for idx, point in enumerate(refined_path):
			refined_waypointlist.markers.append(self.add_waypoint(idx, point))

		#self.pub.publish(self.waypointlist)
		self.path_pub.publish(refined_waypointlist)

		# matplotlib를 통한 시각화

		# 경로 생성 완료 후 생성된 시작 지점과 도착 지점은 제거한다.
		del(self.dijkstra.init_graph[len(self.dijkstra.init_graph) - 1])
		del(self.dijkstra.init_graph[len(self.dijkstra.init_graph) - 1])

	def current_pos_callback(self, data):
		self.start_vertex = data

	def add_waypoint(self, idx, point):
		"""
		Marker() 메시지 자료형을 리턴하는 함수
		"""
		waypoint = Marker()

		waypoint.ns = str(idx)  # Marker namespace
		waypoint.id = idx  # Marker id value, no duplicates
		waypoint.text = str(idx)  # Marker namespace

		waypoint.type = 8  # line strip
		waypoint.lifetime = rospy.Duration.from_sec(5)
		waypoint.header = self.make_header("map")

		waypoint.action = 0 # 정화한 용도를 모르겠음

		# Set waypoint size
		waypoint.scale.x = 0.7
		waypoint.scale.y = 0.7
		waypoint.scale.z = 0.1

		# Set waypoint color, alpha
		waypoint.color.r = 0.25
		waypoint.color.g = 1.0
		waypoint.color.b = 0.25
		waypoint.color.a = 1.0

		pt = Point32()
		pt.x = point[0]
		pt.y = point[1]
		pt.z = 0.0 # Waypoint의 z축은 현재는 불필요함

		waypoint.points.append(pt)

		return waypoint
		# self.waypointlist.markers.append(waypoint)

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
