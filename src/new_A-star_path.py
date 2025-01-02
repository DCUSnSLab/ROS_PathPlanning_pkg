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
		# print(init_graph)
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

class HeuristicGraph(Graph):
    def __init__(self, init_graph):
        super().__init__(init_graph)

    def heuristic(self, node1, node2):
        """
        Heuristic function: Estimate the distance between node1 and node2.
        """
        return math.sqrt(
            math.pow(self.vertices[int(node1)]["xy"][0] - self.vertices[int(node2)]["xy"][0], 2) +
            math.pow(self.vertices[int(node1)]["xy"][1] - self.vertices[int(node2)]["xy"][1], 2)
        )

class AStar(HeuristicGraph):
    def __init__(self, vertices):
        super().__init__(vertices)

    def a_star_algorithm(self, graph, start_node, goal_node):
        open_set = {start_node}
        came_from = {}

        # g_score: Cost from start_node to this node.
        g_score = {node: sys.maxsize for node in graph.get_nodes()}
        g_score[start_node] = 0

        # f_score: Estimated total cost from start_node to goal_node through this node.
        f_score = {node: sys.maxsize for node in graph.get_nodes()}
        f_score[start_node] = self.heuristic(start_node, goal_node)

        while open_set:
            # Current node with the lowest f_score value.
            current_node = min(open_set, key=lambda node: f_score[node])

            if current_node == goal_node:
                return self.reconstruct_path(came_from, current_node)

            open_set.remove(current_node)

            for neighbor in graph.get_outgoing_edges(current_node):
                tentative_g_score = g_score[current_node] + graph.value(current_node, neighbor)

                if tentative_g_score < g_score[neighbor]:
                    # This path to neighbor is better than any previously found.
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_node)

                    if neighbor not in open_set:
                        open_set.add(neighbor)

        raise ValueError("No path found from start to goal.")

    def reconstruct_path(self, came_from, current_node):
        """Reconstruct the path from start to goal."""
        path = []
        while current_node in came_from:
            path.append(current_node)
            current_node = came_from[current_node]
        path.append(current_node)  # Add the start node
        return path[::-1]  # Return reversed path

    def calc_path(self, start, goal):
        self.path = self.a_star_algorithm(graph=self.graph, start_node=start, goal_node=goal)

# DefinedWaypoints 클래스 내에서 AStar를 활용하도록 수정
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

        self.astar = AStar(json_input)

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
        start_nearest_vertex, start_nearest_vertex_idx = self.astar.find_nearest_vertex(goal_position)
        goal_position["adjacent"] = str(start_nearest_vertex_idx)

        goal_nearest_vertex, goal_nearest_vertex_idx = self.astar.find_nearest_vertex(current_position)
        current_position["adjacent"] = str(goal_nearest_vertex_idx)

        self.astar.insert_vertex(current_position, start_nearest_vertex_idx, start_nearest_vertex)
        self.astar.insert_vertex(goal_position, goal_nearest_vertex_idx, goal_nearest_vertex)
        self.astar.construct_graph()

        self.astar.calc_path(str(len(self.astar.init_graph) - 2), str(len(self.astar.init_graph) - 1))

        path = self.astar.get_path()
        refined_path = list()
        refined_waypointlist = MarkerArray()

        for idx in path[1:len(path) - 1]:
            refined_path.append(self.astar.vertices[int(idx)]["xy"])

        refined_path.insert(0, current_position["xy"])
        refined_path.append(goal_position["xy"])

        for idx, point in enumerate(refined_path):
            refined_waypointlist.markers.append(self.add_waypoint(idx, point))

        self.path_pub.publish(refined_waypointlist)

        del(self.astar.init_graph[str(len(self.astar.init_graph) - 1)])
        del(self.astar.init_graph[str(len(self.astar.init_graph) - 1)])