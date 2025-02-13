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
#include <cmath>

#include "ros/ros.h"
#include "path_planning/Node.h"
#include "path_planning/NodeArray.h"
#include "path_planning/Link.h"
#include "path_planning/LinkArray.h"
#include "morai_msgs/GPSMessage.h"

using std::string;

namespace graph_planner {
    class GraphPlanner : public nav_core::BaseGlobalPlanner {
    public:
        GraphPlanner();
        ~GraphPlanner() {
        };
        GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);
        void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
            ROS_INFO("Received GPS Data:");
            ROS_INFO("Latitude: %f", msg->latitude);
            ROS_INFO("Longitude: %f", msg->longitude);
            ROS_INFO("Altitude: %f", msg->altitude);
        }

        class Node {
        private:
            string id;
            double lat, lon;
            std::vector<std::pair<string, double>> neighbors;
        public:
            Node() {
                //ROS_DEBUG("Node() Created");
                //std::cout << "Node() Created" << std::endl;
            }
            ~Node() {}
            Node(const string& id, double lat, double lon) : id(id), lat(lat), lon(lon) {}
            void addNeighbor(const string& neighbor_id, double weight) {
                neighbors.emplace_back(neighbor_id, weight);
            }
            const std::vector<std::pair<string, double>>& getNeighbors() const { return neighbors; }

            string getID() const { return id; }
            double getLat() const { return lat; }
            double getLon() const { return lon; }


        };
        class Graph {
        private:
            std::unordered_map<string, Node> nodes;
        public:
            Graph() {
                ROS_DEBUG("Graph() Created");
                std::cout << "Graph() Created" << std::endl;
            }
            ~Graph() {}
            void addNode(const string& id, double lat, double lon) {
                nodes[id] = Node(id, lat, lon);
            }

            void addLink(const string& from_id, const string& to_id, double weight) {
                if (nodes.find(from_id) != nodes.end() && nodes.find(to_id) != nodes.end()) {
                    nodes[from_id].addNeighbor(to_id, weight);
                    nodes[to_id].addNeighbor(from_id, weight);
                }
            }

            const std::unordered_map<string, Node>& getNodes() const { return nodes; }

            const Node* getNode(const string& id) const {
                auto it = nodes.find(id);
                return (it != nodes.end()) ? &(it->second) : nullptr;
            }
        };

        double heuristic(const Node& a, const Node& b) {
            return sqrt(pow(a.getLat() - b.getLat(), 2) + pow(a.getLon() - b.getLon(), 2));
        }

        std::vector<string> findPath(const Graph& graph, const string& start_id, const string& goal_id) {
            struct AStarNode {
                string id;
                double g_cost, h_cost;

                double f_cost() const { return g_cost + h_cost; }
                bool operator>(const AStarNode &other) const { return f_cost() > other.f_cost(); }
            };

            std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
            std::unordered_map<string, double> g_cost;
            std::unordered_map<string, string> parent;

            const Node* start = graph.getNode(start_id);
            const Node* goal = graph.getNode(goal_id);
            if (!start || !goal) {
                ROS_WARN("시작 또는 목표 노드가 그래프에 없습니다!");
                return {};
            }

            g_cost[start_id] = 0;
            pq.push({start_id, 0, heuristic(*start, *goal)});

            while (!pq.empty()) {
                AStarNode current = pq.top();
                pq.pop();

                if (current.id == goal_id) break;

                const Node* currentNode = graph.getNode(current.id);
                if (!currentNode) continue;

                for (const auto &[neighbor_id, weight] : currentNode->getNeighbors()) {
                    double new_g_cost = g_cost[current.id] + weight;
                    if (g_cost.find(neighbor_id) == g_cost.end() || new_g_cost < g_cost[neighbor_id]) {
                        g_cost[neighbor_id] = new_g_cost;
                        parent[neighbor_id] = current.id;
                        pq.push({neighbor_id, new_g_cost, heuristic(*graph.getNode(neighbor_id), *goal)});
                    }
                }
            }

            // 경로 재구성
            std::vector<string> path;
            for (string at = goal_id; parent.find(at) != parent.end(); at = parent[at])
                path.push_back(at);

            if (!path.empty()) path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            return path;
        }
    private:
        GraphPlanner::Graph graph_;

        ros::Subscriber gps_sub;

        bool initialized_;
        bool arrived_;

        std::pair<double, double> origin_utm_;      // UTM 좌표 원점 (x, y)
        std::pair<double, double> waypoint_relative_utm_;  // 상대 UTM 좌표 (x, y)

        std::tuple<double, double, double> map_gps_coordinates_;  // (위도, 경도, 고도)
        std::pair<double, double> map_utm_;  // (x, y)

        std::vector<path_planning::Node> nodes_;
        std::vector<path_planning::Link> links_;

        path_planning::NodeArray nodearr_;
        path_planning::NodeArray linkarr_;

        ros::ServiceClient mapservice_;
        ros::Subscriber gps_sub_;

        int utm_zone_;
    };
};
#endif