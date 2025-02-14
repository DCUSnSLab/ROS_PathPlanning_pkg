/** include the libraries you need in your planner here */
/** for global path planner interface */

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <GeographicLib/UTMUPS.hpp>
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
            void addNode(Node node) {
                nodes[node.getID()] = node;
            }

            void addNode(const string& id, double lat, double lon) {
                nodes[id] = Node(id, lat, lon);
            }

            void addLink(const string& from_id, const string& to_id, double weight) {
                if (nodes.find(from_id) != nodes.end() && nodes.find(to_id) != nodes.end()) {
                    nodes[from_id].addNeighbor(to_id, weight);
                    // nodes[to_id].addNeighbor(from_id, weight);
                }
            }

            const std::unordered_map<string, Node>& getNodes() const { return nodes; }

            const Node* getNode(const string& id) const {
                auto it = nodes.find(id);
                return (it != nodes.end()) ? &(it->second) : nullptr;
            }
        };
    private:
        GraphPlanner::Graph graph_;

        ros::Subscriber gps_sub;

        ros::NodeHandle private_nh_;

        bool initialized_ = false;
        bool arrived_;

        std::pair<double, double> origin_utm_;      // UTM 좌표 원점 (x, y)
        std::pair<double, double> current_utm_;
        std::pair<double, double> waypoint_relative_utm_;  // 상대 UTM 좌표 (x, y)

        std::tuple<double, double, double> map_gps_coordinates_;  // (위도, 경도, 고도)
        std::pair<double, double> map_utm_;  // (x, y)

        std::vector<path_planning::Node> nodes_;
        std::vector<path_planning::Link> links_;

        path_planning::NodeArray nodearr_;
        path_planning::NodeArray linkarr_;

        ros::ServiceClient mapservice_;
        ros::Subscriber gps_sub_;

        string utm_zone_;
    public:
        GraphPlanner();
        ~GraphPlanner() {
        };
        GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);

        double heuristic(double a_lat, double a_lon, double b_lat, double b_lon) {
            return sqrt(pow(a_lat - b_lat, 2) + pow(a_lon - b_lon, 2));
        }

        double heuristic(const Node& a, const Node& b) {
            return sqrt(pow(a.getLat() - b.getLat(), 2) + pow(a.getLon() - b.getLon(), 2));
        }

        std::string findClosestNode(double lat, double lon) {
            std::cout << "findClosestNode called." << std::endl;
            std::string closest_node_id;
            double min_distance = std::numeric_limits<double>::max();

            for (const auto& [node_id, node] : graph_.getNodes()) {
                double node_lat = node.getLat();
                double node_lon = node.getLon();
                double distance = heuristic(lat, lon, node_lat, node_lon);

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_node_id = node_id;
                }
            }
            return closest_node_id;
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
        void latLonToUtm(double lat, double lon, double &utm_x, double &utm_y, std::string &utm_zone) {
            /*
            Temp function,
            Replaced with corresponding code due to delayed availability testing of GPS-related libraries
            Future changes required
            */
            int zone = static_cast<int>(std::floor((lon + 180) / 6) + 1);
            bool north = (lat >= 0);

            double a = 6378137.0; // WGS84 타원체의 반경
            double f = 1 / 298.257223563;
            double k0 = 0.9996;
            double e = std::sqrt(f * (2 - f));
            double n = f / (2 - f);

            double lambda0 = (zone - 1) * 6 - 180 + 3;
            double phi = lat * M_PI / 180.0;
            double lambda = lon * M_PI / 180.0;
            double lambda_diff = lambda - lambda0 * M_PI / 180.0;

            double N = a / std::sqrt(1 - e * e * std::sin(phi) * std::sin(phi));
            double T = std::tan(phi) * std::tan(phi);
            double C = (e * e / (1 - e * e)) * std::cos(phi) * std::cos(phi);
            double A = std::cos(phi) * lambda_diff;

            double M = a * ((1 - e * e / 4 - 3 * e * e * e * e / 64 - 5 * e * e * e * e * e * e / 256) * phi
                            - (3 * e * e / 8 + 3 * e * e * e * e / 32 + 45 * e * e * e * e * e * e / 1024) * std::sin(2 * phi)
                            + (15 * e * e * e * e / 256 + 45 * e * e * e * e * e * e / 1024) * std::sin(4 * phi)
                            - (35 * e * e * e * e * e * e / 3072) * std::sin(6 * phi));

            utm_x = k0 * N * (A + (1 - T + C) * A * A * A / 6.0 + (5 - 18 * T + T * T + 72 * C - 58 * (e * e / (1 - e * e))) * A * A * A * A * A / 120.0) + 500000.0;
            utm_y = k0 * (M + N * std::tan(phi) * (A * A / 2.0 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24.0
                             + (61 - 58 * T + T * T + 600 * C - 330 * (e * e / (1 - e * e))) * A * A * A * A * A * A / 720.0));

            if (!north) {
                utm_y += 10000000.0; // 남반구 보정
            }

            utm_zone = std::to_string(zone) + (north ? "N" : "S");
        }
        void gpsPathfinder(Node& start, Node& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
            /*
            Load the path from the current location to the target point into the PoseStamped message vector.
            */

            // find nearest node from start
            string start_near = findClosestNode(start.getLat(), start.getLon());

            // find nearest node from goal
            string goal_near = findClosestNode(goal.getLat(), goal.getLon());

            graph_.addNode(start);
            graph_.addNode(goal);

            graph_.addLink(start.getID(), start_near, 0.5);
            // graph_.addLink(goal.getID(), goal_near, 0.5);
            graph_.addLink(goal_near, goal.getID(), 0.5);

            std::vector<string> path = findPath(graph_, start.getID(), goal.getID());

            if (!path.empty()) {
                for (const auto &id : path) {
                    ROS_INFO("%s", id.c_str());
                }
            } else {
                ROS_WARN("Warn");
            }

            std::cout << "Path finder called." << std::endl;
        }

        void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
            latLonToUtm(msg->latitude, msg->longitude, current_utm_.first, current_utm_.second, utm_zone_);
//            ROS_INFO("Received GPS Data:");
//            ROS_INFO("Latitude: %f", msg->latitude);
//            ROS_INFO("Longitude: %f", msg->longitude);
//            ROS_INFO("Altitude: %f", msg->altitude);
        }
    };
};
#endif