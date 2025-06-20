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

        // gps_sub_ = private_nh_.subscribe("/gps", 10, &GraphPlanner::gpsCallback, this);
        // gps_sub_ = private_nh_.subscribe("/current_utm_relative_position", 10, &GraphPlanner::gpsCallback, this);
        // 0, 0 기준 상대 좌표를 사용하는 경우, UTMtoGPS 함수의 결과가 이상하게 나와 Global planning 수행 불가
        gps_sub_ = private_nh_.subscribe("/ublox_f9k/fix", 10, &GraphPlanner::gpsCallback, this);

        private_nh_.getParam("map_file", file_path_);

        std::cout << file_path_ << std::endl;

        Callgraph();

        initialized_ = true;
    }
}

bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan) {
    plan.clear();
    if (!initialized_) {
        std::cout << "Global Planner is not initialized" << std::endl;
        ROS_ERROR("Global Planner is not initialized");
        return false;
        plan.clear();
        plan.push_back(start);
        plan.push_back(goal);
        return true;
    }

    if (std::isnan(goal.pose.position.x) || std::isnan(goal.pose.position.y)) {
        return false;
    }

    graph_.removeStartGoalNode(); // Before planning a route, delete previous start and goal points.

    std::cout << "New goal init" << std::endl;

    string start_id = "Start";
    string goal_id = "Goal";
    ROS_DEBUG("This is test output(makeplan)");

    GraphPlanner::Node start_node("Start", current_gps_.first, current_gps_.second, current_utm_.first, current_utm_.second);

//    goal_utm_.first = map_utm_.first + goal.pose.position.x;
//    goal_utm_.second = map_utm_.second + goal.pose.position.y;
    goal_utm_.first = goal.pose.position.x + init_utm_.first;
    goal_utm_.second = goal.pose.position.y + init_utm_.second;
    std::cout << "goal pose position" << goal.pose.position.x << " " << goal.pose.position.y << std::endl;
    std::cout << "init utm" << init_utm_.first << " " << init_utm_.second << std::endl;
//    goal_utm_.first = current_utm_.first;
//    goal_utm_.second = current_utm_.second;
    std::cout << utm_zone_ << std::endl;

    utmToLatLon(goal_utm_.first, goal_utm_.second, goal_gps_.first, goal_gps_.second, utm_zone_);
    GraphPlanner::Node goal_node("Goal", goal_gps_.first, goal_gps_.second, goal_utm_.first, goal_utm_.second); // 아마도 제대로 안되는듯?

    std::cout << goal_utm_.first << " " << goal_utm_.second << std::endl;
    std::cout << current_utm_.first << " " << current_utm_.second << std::endl;

    std::cout << goal_gps_.first << " " << goal_gps_.second << std::endl;
    std::cout << current_gps_.first << " " << current_gps_.second << std::endl;

    GraphPlanner::gpsPathfinder(start_node, goal_node, plan);
    /*
    gpsPathfinder()
    In this function, a path search is performed between start_node and goal_node
    and the generated path is loaded into the plan.
    */

    goalinit_ = true;
    return true;
}