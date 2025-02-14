#include <pluginlib/class_list_macros.h>
#include "../include/graph_planner/graph_planner.h"
#include "path_planning/MapGraph.h"
#include "morai_msgs/GPSMessage.h"
#include <iostream>

 //register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

using namespace graph_planner;

 //Default Constructor
GraphPlanner::GraphPlanner (){
    ROS_DEBUG("This is test msg for GraphPlanner(no attr)");
    std::cout << "This is test output(GraphPlanner(no attr))" << std::endl;


    gps_sub_ = private_nh_.subscribe("/gps", 10, &GraphPlanner::gpsCallback, this);
}

GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_DEBUG("This is test msg for GraphPlanner");
    std::cout << "This is test output(GraphPlanner)" << std::endl;
    initialize(name, costmap_ros);
}

void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    std::cout << "initialize called." << std::endl;
    ROS_DEBUG("initialize called.");
    // GraphPlanner::Node node; // instance example
    if(!initialized_) {
        std::cout << "This is test output(init)" << std::endl;
        ROS_DEBUG("This is test output(init)");

        mapservice_ = private_nh_.serviceClient<path_planning::MapGraph>("/map_server");
        path_planning::MapGraph map_srv;

        map_srv.request.file_path = "/home/ros/SCV2/src/scv_system/global_path/ROS_PathPlanning_pkg/data/graph(map)/20250115_k-city.json";

        if (mapservice_.call(map_srv)) {
            ROS_INFO("success");
            std::cout << "map service call" << std::endl;
            std::cout << map_srv.response.map_graph.node_array.nodes[0].ID << std::endl; // cout for debug
            //graph_ = map_srv.response.map_graph;
            //ndarr_ = map_srv.response.map_graph.node_array;
            bool initMapcoord = false;

            for (const auto &node : map_srv.response.map_graph.node_array.nodes) {
                graph_.addNode(node.ID, node.Lat, node.Long, node.Easting, node.Northing);
                if (!initMapcoord) {
                    map_gps_coordinates_ = std::make_tuple(node.Lat, node.Long, node.Alt);
                    map_utm_.first = node.Easting;
                    map_utm_.second = node.Northing;
                    utm_zone_ = node.Zone;
                    initMapcoord = true;
                }
            }

            for (const auto &link : map_srv.response.map_graph.link_array.links) {
                graph_.addLink(link.FromNodeID, link.ToNodeID, link.Length);
            }

            ROS_INFO("Create map graph");

        } else {
            std::cout << "error" << std::endl;
            ROS_ERROR("error");
        }

        initialized_ = true;
    }
}

bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("Global Planner is not initialized");
        return false;
        plan.clear();
        // 최소한의 샘플 경로 생성
        plan.push_back(start);
        plan.push_back(goal);
        return true;
    }
    else {

        ROS_INFO("Header:");
        ROS_INFO("  - Seq: %d", start.header.seq);
        ROS_INFO("  - Stamp: %d.%d", (int)start.header.stamp.sec, (int)start.header.stamp.nsec);
        ROS_INFO("  - Frame ID: %s", start.header.frame_id.c_str());

        // Position 정보
        ROS_INFO("Position:");
        ROS_INFO("  - x: %f", start.pose.position.x);
        ROS_INFO("  - y: %f", start.pose.position.y);
        ROS_INFO("  - z: %f", start.pose.position.z);

        // Orientation 정보 (Quaternion)
        ROS_INFO("Orientation:");
        ROS_INFO("  - x: %f", start.pose.orientation.x);
        ROS_INFO("  - y: %f", start.pose.orientation.y);
        ROS_INFO("  - z: %f", start.pose.orientation.z);
        ROS_INFO("  - w: %f", start.pose.orientation.w);

        ROS_INFO("Header:");
        ROS_INFO("  - Seq: %d", goal.header.seq);
        ROS_INFO("  - Stamp: %d.%d", (int)goal.header.stamp.sec, (int)goal.header.stamp.nsec);
        ROS_INFO("  - Frame ID: %s", goal.header.frame_id.c_str());

        // Position 정보
        ROS_INFO("Position:");
        ROS_INFO("  - x: %f", goal.pose.position.x);
        ROS_INFO("  - y: %f", goal.pose.position.y);
        ROS_INFO("  - z: %f", goal.pose.position.z);

        // Orientation 정보 (Quaternion)
        ROS_INFO("Orientation:");
        ROS_INFO("  - x: %f", goal.pose.orientation.x);
        ROS_INFO("  - y: %f", goal.pose.orientation.y);
        ROS_INFO("  - z: %f", goal.pose.orientation.z);
        ROS_INFO("  - w: %f", goal.pose.orientation.w);

        string start_id = "Start";
        string goal_id = "Goal";
        ROS_DEBUG("This is test output(makeplan)");
        std::cout << current_utm_.first << std::fixed << std::endl;
        std::cout << current_utm_.second << std::fixed << std::endl;

        GraphPlanner::Node start("Start", current_gps_.first, current_gps_.second, current_utm_.first, current_utm_.second);
        goal_utm_.first = map_utm_.first + goal.pose.position.x;
        goal_utm_.second = map_utm_.second + goal.pose.position.y;
        utmToLatLon(goal_utm_.first, goal_utm_.second, goal_gps_.first, goal_gps_.second, utm_zone_);
        GraphPlanner::Node goal("Goal", goal_gps_.first, goal_gps_.second, goal_utm_.first, goal_utm_.second);

        GraphPlanner::gpsPathfinder(start, goal, plan);
    }
    // Below code block is important.
    // If global plan failed, there are no data in vector(plan) and it occurs error to move_base Node
    // DO NOT DELETE theses lines!!!
    if (plan.empty()) {
        plan.clear();
        plan.push_back(start);
        plan.push_back(goal);
        return true;
    }
    else {
        std::cout << "planning.." << std::endl;
        return true;
    }
    // REAL
}