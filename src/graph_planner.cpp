#include <pluginlib/class_list_macros.h>
#include "../include/graph_planner/graph_planner.h"
#include "path_planning/MapGraph.h"
#include "path_planning/DisplayMarkerMap.h"
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

        gps_sub_ = private_nh_.subscribe("/gps", 10, &GraphPlanner::gpsCallback, this);

        private_nh_.getParam("MAP_FILE", file_path_);

        std::cout << file_path_ << std::endl;

        Callgraph();

        initialized_ = true;
    }
}

bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        std::cout << "Global Planner is not initialized" << std::endl;
        ROS_ERROR("Global Planner is not initialized");
        return false;
        plan.clear();
        plan.push_back(start);
        plan.push_back(goal);
        return true;
    }
    else {
//        if (goal.header.stamp.toSec() > 0) {
//            std::cout << "wait new goal" << std::endl;
//        }

//        ROS_INFO("Header:");
//        ROS_INFO("  - Seq: %d", start.header.seq);
//        ROS_INFO("  - Stamp: %d.%d", (int)start.header.stamp.sec, (int)start.header.stamp.nsec);
//        ROS_INFO("  - Frame ID: %s", start.header.frame_id.c_str());
//
//        // Position 정보
//        ROS_INFO("Position:");
//        ROS_INFO("  - x: %f", start.pose.position.x);
//        ROS_INFO("  - y: %f", start.pose.position.y);
//        ROS_INFO("  - z: %f", start.pose.position.z);
//
//        // Orientation 정보 (Quaternion)
//        ROS_INFO("Orientation:");
//        ROS_INFO("  - x: %f", start.pose.orientation.x);
//        ROS_INFO("  - y: %f", start.pose.orientation.y);
//        ROS_INFO("  - z: %f", start.pose.orientation.z);
//        ROS_INFO("  - w: %f", start.pose.orientation.w);
//
//        ROS_INFO("Header:");
//        ROS_INFO("  - Seq: %d", goal.header.seq);
//        ROS_INFO("  - Stamp: %d.%d", (int)goal.header.stamp.sec, (int)goal.header.stamp.nsec);
//        ROS_INFO("  - Frame ID: %s", goal.header.frame_id.c_str());
//
//        // Position 정보
//        ROS_INFO("Position:");
//        ROS_INFO("  - x: %f", goal.pose.position.x);
//        ROS_INFO("  - y: %f", goal.pose.position.y);
//        ROS_INFO("  - z: %f", goal.pose.position.z);
//
//        // Orientation 정보 (Quaternion)
//        ROS_INFO("Orientation:");
//        ROS_INFO("  - x: %f", goal.pose.orientation.x);
//        ROS_INFO("  - y: %f", goal.pose.orientation.y);
//        ROS_INFO("  - z: %f", goal.pose.orientation.z);
//        ROS_INFO("  - w: %f", goal.pose.orientation.w);
//        else {
        if (goalinit_ == false) {
            string start_id = "Start";
            string goal_id = "Goal";
            ROS_DEBUG("This is test output(makeplan)");

            GraphPlanner::Node start("Start", current_gps_.first, current_gps_.second, current_utm_.first, current_utm_.second);
            goal_utm_.first = map_utm_.first + goal.pose.position.x;
            goal_utm_.second = map_utm_.second + goal.pose.position.y;
            utmToLatLon(goal_utm_.first, goal_utm_.second, goal_gps_.first, goal_gps_.second, utm_zone_);
            GraphPlanner::Node goal("Goal", goal_gps_.first, goal_gps_.second, goal_utm_.first, goal_utm_.second);

            GraphPlanner::gpsPathfinder(start, goal, plan);

                // std::cout << "planning.." << std::endl;
            goalinit_ = true;
        }
        return true;
    }
    // REAL
}