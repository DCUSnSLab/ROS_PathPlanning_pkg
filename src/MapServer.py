#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from path_planning.srv import CreateGraph
from pyproj import Proj, transform

def load_graph(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    graph = {}
    nodes = {node['ID']: node for node in data['Node']}

    for link in data['Link']:
        from_node = link['FromNodeID']
        to_node = link['ToNodeID']
        length = link['Length']

        if from_node not in graph:
            graph[from_node] = []
        if to_node not in graph:
            graph[to_node] = []

        graph[from_node].append((to_node, length))
        graph[to_node].append((from_node, length))  # Assuming undirected graph

    return graph, nodes

def handle_multi_type_response(req):
    rospy.loginfo("Processing file: %s", req.file_path)
    try:

        # Convert MarkerArray to graph dictionary
        graph, nodes = load_graph(req.file_path)
        graph_json = json.dumps(graph)  # Convert to JSON string
        node_json = json.dumps(nodes)  # Convert to JSON string

        return dict(graph_json=graph_json, node_json=node_json)
    except Exception as e:
        rospy.logerr("Failed to process file %s: %s", req.file_path, str(e))
        return dict(graph_json="", node_json="")


def multi_type_response_server():
    rospy.init_node('map_server')
    rospy.Service('map_server', CreateGraph, handle_multi_type_response)
    rospy.spin()


if __name__ == "__main__":
    try:
        multi_type_response_server()
    except rospy.ROSInterruptException:
        pass
