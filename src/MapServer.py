#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from path_planning.srv import MapGraph, DisplayMarkerMap

from visualization_msgs.msg import Marker, MarkerArray
from path_planning.msg import GraphMsg, Node, NodeArray, Link, LinkArray

import sys
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('path_planning')
sys.path.append(package_path + "/src")

from visualize_path import parse_json_and_visualize

def load_node_from_json(json_node):
    node = Node()
    node.ID = json_node["ID"]
    node.AdminCode = json_node.get("AdminCode", None)
    node.NodeType = json_node.get("NodeType", 0)
    node.ITSNodeID = json_node.get("ITSNodeID", "")
    node.Maker = json_node.get("Maker", "")
    node.UpdateDate = json_node.get("UpdateDate", "")
    node.Version = json_node.get("Version", "")
    node.Remark = json_node.get("Remark", "")
    node.HistType = json_node.get("HistType", "")
    node.HistRemark = json_node.get("HistRemark", "")
    node.Lat = json_node["GpsInfo"]["Lat"]
    node.Long = json_node["GpsInfo"]["Long"]
    node.Alt = json_node["GpsInfo"]["Alt"]
    node.Easting = json_node["UtmInfo"]["Easting"]
    node.Northing = json_node["UtmInfo"]["Northing"]
    node.Zone = json_node["UtmInfo"]["Zone"]
    return node

def load_link_from_json(json_link):
    link = Link()
    link.ID = json_link["ID"]
    link.AdminCode = json_link.get("AdminCode", "")
    link.RoadRank = json_link.get("RoadRank", 0)
    link.RoadType = json_link.get("RoadType", 0)
    link.RoadNo = json_link.get("RoadNo", "")
    link.LinkType = json_link.get("LinkType", 0)
    link.LaneNo = json_link.get("LaneNo", 0)
    link.R_LinkID = json_link.get("R_LinkID", "")
    link.L_LinkID = json_link.get("L_LinkID", "")
    link.FromNodeID = json_link["FromNodeID"]
    link.ToNodeID = json_link["ToNodeID"]
    link.SectionID = json_link.get("SectionID", "")
    link.Length = json_link.get("Length", 0.0)
    link.ITSLinkID = json_link.get("ITSLinkID", "")
    link.Maker = json_link.get("Maker", "")
    link.UpdateDate = json_link.get("UpdateDate", "")
    link.Version = json_link.get("Version", "")
    link.Remark = json_link.get("Remark", "")
    link.HistType = json_link.get("HistType", "")
    link.HistRemark = json_link.get("HistRemark", "")
    return link

def load_graph_from_json(json_data):
    node_array = NodeArray()
    link_array = LinkArray()

    node_array.nodes = [load_node_from_json(node) for node in json_data["Node"]]
    link_array.links = [load_link_from_json(link) for link in json_data["Link"]]

    graph = GraphMsg()
    graph.node_array = node_array
    graph.link_array = link_array
    return graph

def graph_response(req):
    rospy.loginfo("Processing file: %s", req.file_path)
    try:
        with open(req.file_path, 'r', encoding='utf-8') as f:
            json_data = json.load(f)

        graph_message = load_graph_from_json(json_data)

        return graph_message

    except Exception as e:
        rospy.logerr("Failed to process file %s: %s", req.file_path, str(e))

def marker_response(req):
    rospy.loginfo("Processing file: %s", req.file_path)
    try:
        marker_array, ref_x, ref_y, ref_z = parse_json_and_visualize(req.file_path, (req.correction_val.x, req.correction_val.y, req.correction_val.z))

        marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10, latch=True)

        marker_pub.publish(marker_array)

        return True

    except Exception as e:
        rospy.logerr("Failed to process file %s: %s", req.file_path, str(e))
        return False


def multi_type_response_server():
    rospy.init_node('map_server')
    rospy.Service('map_server', MapGraph, graph_response)
    rospy.Service('map_display_server', DisplayMarkerMap, marker_response)
    rospy.spin()


if __name__ == "__main__":
    try:
        multi_type_response_server()
    except rospy.ROSInterruptException:
        pass
