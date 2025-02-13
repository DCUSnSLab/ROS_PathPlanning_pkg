/** include the libraries you need in your planner here */
/** for global path planner interface */

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>

#include "ros/ros.h"
#include "path_planning/Node.h"
#include "path_planning/NodeArray.h"
#include "path_planning/Link.h"
#include "path_planning/LinkArray.h"

using std::string;

namespace graph_planner {
    class GraphPlanner : public nav_core::BaseGlobalPlanner {
    public:
        GraphPlanner();
        GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);
        class Node {
        private:
            string id;
            double lat, lon;
            std::vector<std::pair<string, double>> neighbors;
        public:
            Node() {}
            Node(const string& id, double lat, double lon) : id(id), lat(lat), lon(lon) {}
            void addNeighbor(const string& neighbor_id, double weight);
            const std::vector<std::pair<string, double>>& getNeighbors();

            string getID();
            double getLat();
            double getLon();
        };
        class Graph {
        private:
            std::unordered_map<string, Node> nodes;
        public:
            void addNode(const string& id, double lat, double lon);

            // 링크 추가 (양방향)
            void addLink(const string& from_id, const string& to_id, double weight);
            // 노드 가져오기
            const std::unordered_map<string, Node>& getNodes();
            // 특정 노드 가져오기
            const Node* getNode(const string& id);
        };
    private:
        GraphPlanner::Graph graph_;

        bool initialized_;

        std::pair<double, double> origin_utm_;      // UTM 좌표 원점 (x, y)
        std::pair<double, double> waypoint_relative_utm_;  // 상대 UTM 좌표 (x, y)

        std::tuple<double, double, double> map_gps_coordinates_;  // (위도, 경도, 고도)
        std::pair<double, double> map_utm_;  // (x, y)

        std::vector<path_planning::Node> nodes_;
        std::vector<path_planning::Link> links_;

        path_planning::NodeArray nodearr_;
        path_planning::NodeArray linkarr_;

        int utm_zone_;
    };
};
#endif