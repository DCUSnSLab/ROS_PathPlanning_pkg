/** include the libraries you need in your planner here */
/** for global path planner interface */

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include "path_planning/MapGraph.h"
#include "path_planning/DisplayMarkerMap.h"

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
            double easting, northing;
            string zone;
            std::vector<std::pair<string, double>> neighbors;
        public:
            Node() {
                //ROS_DEBUG("Node() Created");
                //std::cout << "Node() Created" << std::endl;
            }
            ~Node() {}
            Node(const string& id, double lat, double lon, double easting, double northing) : id(id), lat(lat), lon(lon), easting(easting), northing(northing) {}
            void addNeighbor(const string& neighbor_id, double weight) {
                neighbors.emplace_back(neighbor_id, weight);
            }
            const std::vector<std::pair<string, double>>& getNeighbors() const { return neighbors; }

            string getID() const { return id; }
            double getLat() const { return lat; }
            double getLon() const { return lon; }
            double getEasting() const { return easting; }
            double getNorthing() const { return northing; }

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

            void addNode(const string& id, double lat, double lon, double easting, double northing) {
                nodes[id] = Node(id, lat, lon, easting, northing);
            }

            void removeStartGoalNode() {
                nodes.erase("Start");
                nodes.erase("Goal");
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
    private:
        GraphPlanner::Graph graph_;

        ros::Subscriber gps_sub;

        ros::NodeHandle private_nh_;

        path_planning::MapGraph map_srv;

        // string file_path_ = "/home/ros/SCV2/src/scv_system/global_path/ROS_PathPlanning_pkg/data/graph(map)/20250115_k-city.json";
        string file_path_ = "";

        ros::ServiceClient mapservice_ = private_nh_.serviceClient<path_planning::MapGraph>("/map_server");
        ros::ServiceClient mapdisplayservice_ = private_nh_.serviceClient<path_planning::DisplayMarkerMap>("/map_display_server");

        bool initialized_ = false;
        bool arrived_;
        bool goalinit_ = false;
        bool mapinit_ = false;

        /*
        In order to accurately project a map based on relative coordinates,
        arbitrary reference coordinates are required.
        Do it that way first and modify it later.
        */
        std::tuple<double, double, double> origin_gps_coordinates_ = std::make_tuple(37.23923857150045, 126.7731611307903, 28.940864029884338);  // origin coord
        // std::pair<double, double> origin_utm_ = std::make_pair(302473.690, 4123735.933);
        std::pair<double, double> origin_utm_;

        std::tuple<double, double, double> map_gps_coordinates_;  // first node's Coordinate in map data
        std::pair<double, double> map_utm_;  // same but UTM

        std::pair<double, double> current_gps_;
        std::pair<double, double> current_utm_;
        std::pair<double, double> goal_gps_;
        std::pair<double, double> goal_utm_;

        std::vector<geometry_msgs::Point> gps_points;
        bool first_point = true;

        std::pair<double, double> rviz_correction_;

        // std::pair<double, double> waypoint_relative_utm_;  // 상대 UTM 좌표 (x, y)

        std::vector<path_planning::Node> nodes_;
        std::vector<path_planning::Link> links_;

        path_planning::NodeArray nodearr_;
        path_planning::NodeArray linkarr_;

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
            /*
            This function performs a simple string output for the optimal path,
            so you must use the separately implemented function below.
            */
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
            for (string at = goal_id; parent.find(at) != parent.end(); at = parent[at]) {
                path.push_back(at);
                std::cout << at << std::endl;
            }

            if (!path.empty()) path.push_back(start_id);
            std::reverse(path.begin(), path.end());
            return path;
        }

        void findPath(const Graph& graph, const string& start_id, const string& goal_id, std::vector<geometry_msgs::PoseStamped>& plan) {
            struct AStarNode {
                string id;
                double g_cost, h_cost;
                double lat, lon, easting, northing;

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
            }
            else {
                g_cost[start_id] = 0;
                pq.push({start_id, 0, heuristic(*start, *goal), start->getLat(), start->getLon(), start->getEasting(), start->getNorthing()});

                while (!pq.empty()) {
                    AStarNode current = pq.top();
                    pq.pop();

                    if (current.id == goal_id) break;

                    const Node* currentNode = graph.getNode(current.id);
                    if (!currentNode) continue;

                    for (const auto &[neighbor_id, weight] : currentNode->getNeighbors()) {
                        const Node* neighborNode = graph.getNode(neighbor_id);
                        if (!neighborNode) continue;

                        double new_g_cost = g_cost[current.id] + weight;
                        if (g_cost.find(neighbor_id) == g_cost.end() || new_g_cost < g_cost[neighbor_id]) {
                            g_cost[neighbor_id] = new_g_cost;
                            parent[neighbor_id] = current.id;
                            pq.push({neighbor_id, new_g_cost, heuristic(*neighborNode, *goal), neighborNode->getLat(), neighborNode->getLon(), neighborNode->getEasting(), neighborNode->getNorthing()});
                        }
                    }
                }

                std::vector<string> path;
                for (string at = goal_id; parent.find(at) != parent.end(); at = parent[at]) {
                    path.push_back(at);
                }

                std::reverse(path.begin(), path.end());

                ros::Time current_time = ros::Time::now();
                Node const *tmp;

                for (size_t i = 0; i < path.size(); i++) {
                    tmp = graph_.getNode(path[i]);

                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = current_time;
                    pose.header.frame_id = "map";  // 좌표계 설정

                    pose.pose.position.x = tmp->getEasting();  // x 좌표
                    pose.pose.position.y = tmp->getNorthing(); // y 좌표

//                    pose.pose.position.x = tmp->getEasting() - origin_utm_.first;  // x 좌표
//                    pose.pose.position.y = tmp->getNorthing() - origin_utm_.second; // y 좌표

                    pose.pose.position.z = 0.0;  // 기본값으로 z=0 설정

                    pose.pose.orientation.w = 1.0;  // 기본적으로 방향 설정 (회전 없음)
                    plan.push_back(pose);
                }
            }
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

        void utmToLatLon(double utm_x, double utm_y, double &lat, double &lon, const std::string& utm_zone) {
            std::pair<double, double> tmp;
            int zone = std::stoi(utm_zone.substr(0, utm_zone.size() - 1));  // 숫자 부분 (예: "52N" → 52)
            bool north = (utm_zone.back() == 'N'); // 북반구 여부 확인

            double a = 6378137.0; // WGS84 타원체 반경
            double f = 1 / 298.257223563;
            double k0 = 0.9996;
            double e = std::sqrt(f * (2 - f));
            double e1 = (1 - std::sqrt(1 - e * e)) / (1 + std::sqrt(1 - e * e));

            // 중앙 경도 계산
            double lambda0 = (zone - 1) * 6 - 180 + 3;

            // 남반구 보정
            if (!north) {
                utm_y -= 10000000.0;
            }

            // 위도 계산
            double M = utm_y / k0;
            double mu = M / (a * (1 - e * e / 4 - 3 * e * e * e * e / 64 - 5 * e * e * e * e * e * e / 256));

            double phi1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * std::sin(2 * mu)
                              + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * std::sin(4 * mu)
                              + (151 * e1 * e1 * e1 / 96) * std::sin(6 * mu)
                              + (1097 * e1 * e1 * e1 * e1 / 512) * std::sin(8 * mu);

            double N1 = a / std::sqrt(1 - e * e * std::sin(phi1) * std::sin(phi1));
            double T1 = std::tan(phi1) * std::tan(phi1);
            double C1 = (e * e / (1 - e * e)) * std::cos(phi1) * std::cos(phi1);
            double R1 = a * (1 - e * e) / std::pow(1 - e * e * std::sin(phi1) * std::sin(phi1), 1.5);
            double D = (utm_x - 500000.0) / (N1 * k0);

            // 최종 위도 계산
            lat = phi1 - (N1 * std::tan(phi1) / R1) *
                  (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * (e * e / (1 - e * e))) * D * D * D * D / 24
                   + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * (e * e / (1 - e * e)) - 3 * C1 * C1) * D * D * D * D * D * D / 720);

            lat = lat * 180.0 / M_PI; // 라디안을 도(degree)로 변환

            // 최종 경도 계산
            lon = lambda0 + (D - (1 + 2 * T1 + C1) * D * D * D / 6
                   + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * (e * e / (1 - e * e)) + 24 * T1 * T1) * D * D * D * D * D / 120) / std::cos(phi1);

            lon = lon * 180.0 / M_PI; // 라디안을 도(degree)로 변환
        }

        void gpsPathfinder(Node& start, Node& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
            /*
            Load the path from the current location to the target point into the PoseStamped message vector.
            */

            // find nearest node from start
            string start_near = findClosestNode(start.getLat(), start.getLon());
            //std::cout << "start_near" << std::endl;
            //std::cout << start_near << std::endl;
            // find nearest node from goal
            string goal_near = findClosestNode(goal.getLat(), goal.getLon());
            //std::cout << "goal_near" << std::endl;
            //std::cout << goal_near << std::endl;

            // Remove previous start, goal node
            graph_.removeStartGoalNode();

            graph_.addNode(start);
            graph_.addNode(goal);

            graph_.addLink(start.getID(), start_near, 0.1);
            graph_.addLink(goal_near, goal.getID(), 0.1);

            // find path in graph
            // findPath(graph_, "N0000", "N0028", plan);
            findPath(graph_, start.getID(), goal.getID(), plan);

            //std::cout << "Path finder called." << std::endl;
        }

        void Umeyama() {
            // 1. 대응점 데이터 설정 (A 좌표계의 점들)
            Eigen::MatrixXd X(3, 2);
            X << 2, 3,
                 4, 5,
                 3, 7;

            // 2. B 좌표계의 대응 점들 생성
            //    (예제에서는 A 좌표계의 점들에 대해,
            //     스케일 1.5, 회전 30도, 평행이동 (1, -2)를 적용했다고 가정)
            double theta = M_PI / 6; // 30도 (라디안 단위)
            double s_true = 1.5;
            Eigen::Vector2d t_true(1, -2);

            Eigen::Matrix2d R_true;
            R_true << cos(theta), -sin(theta),
                      sin(theta),  cos(theta);

            Eigen::MatrixXd Y(3, 2);
            for (int i = 0; i < X.rows(); i++) {
                Eigen::Vector2d xi = X.row(i);
                // B 좌표계: Y = s_true * (R_true * xi) + t_true
                Eigen::Vector2d yi = s_true * (R_true * xi) + t_true;
                Y.row(i) = yi;
            }

            // 3. Umeyama 알고리즘을 통한 변환 파라미터 추정

            // (a) 각 좌표계의 중심(centroid) 계산
            Eigen::Vector2d mu_X = X.colwise().mean();
            Eigen::Vector2d mu_Y = Y.colwise().mean();

            // (b) 각 점들을 중심화 (centered coordinates)
            Eigen::MatrixXd Xc = X.rowwise() - mu_X.transpose();
            Eigen::MatrixXd Yc = Y.rowwise() - mu_Y.transpose();

            // (c) 공분산 행렬 계산: Sigma = (Yc^T * Xc) / N
            Eigen::Matrix2d Sigma = (Yc.transpose() * Xc) / X.rows();

            // (d) SVD 분해: Sigma = U * D * V^T
            Eigen::JacobiSVD<Eigen::Matrix2d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2d U = svd.matrixU();
            Eigen::Matrix2d V = svd.matrixV();
            Eigen::Vector2d D = svd.singularValues();

            // (e) 회전 행렬 결정
            double d = (U.determinant() * V.determinant());
            Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
            S(1, 1) = d;
            Eigen::Matrix2d R_est = U * S * V.transpose();

            // (f) 스케일 팩터 결정
            double var_X = (Xc.array().square().sum()) / X.rows();
            double s_est = (D.dot(S.diagonal())) / var_X;

            // (g) 평행 이동 (translation) 추정
            Eigen::Vector2d t_est = mu_Y - s_est * R_est * mu_X;

            // 4. 결과 출력
            std::cout << "Estimated Scale: " << s_est << std::endl;
            std::cout << "Estimated Rotation Matrix:\n" << R_est << std::endl;
            std::cout << "Estimated Translation: " << t_est.transpose() << std::endl;
        }

        void Callgraph() {
            map_srv.request.file_path = file_path_;

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
        }

        void DisplayMap() {
            path_planning::DisplayMarkerMap map_display_srv;
            map_display_srv.request.file_path = file_path_;

            map_display_srv.request.correction_val.x = current_utm_.first;
            map_display_srv.request.correction_val.y = current_utm_.second;

            map_display_srv.request.correction_val.z = 0.0; // alt

            std::cout << current_utm_.first << " " << origin_utm_.first << " " << map_utm_.first << std::endl;
            std::cout << current_utm_.second << " " << origin_utm_.second << " " << map_utm_.second << std::endl;

            if (mapdisplayservice_.call(map_display_srv)) {
                std::cout << "map display service call" << std::endl;
            } else {
                std::cout << "map display error" << std::endl;
                ROS_ERROR("map display error");
            }
        }

        double distance2D(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
            return std::sqrt((a.x - b.x) * (a.x - b.x) +
                             (a.y - b.y) * (a.y - b.y));
        }

        void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
            current_gps_.first = msg->latitude;
            current_gps_.second = msg->longitude;

            /*
            store gps coord for coordinate registration
            */

            geometry_msgs::Point tmp;
            tmp.x = current_gps_.first;
            tmp.y = current_gps_.second;

            if(first_point) {
                gps_points.push_back(tmp);
                first_point = false;
            }
            else {
                const geometry_msgs::Point &last_point = gps_points.back();
                double dist = distance2D(tmp, last_point);
                if (dist >= 0.1) {
                    gps_points.push_back(tmp);
                }
            }

            latLonToUtm(msg->latitude, msg->longitude, current_utm_.first, current_utm_.second, utm_zone_);
            if (!mapinit_) {
                DisplayMap();
                mapinit_ = true;
            }
//            ROS_INFO("Received GPS Data:");
//            ROS_INFO("Latitude: %f", msg->latitude);
//            ROS_INFO("Longitude: %f", msg->longitude);
//            ROS_INFO("Altitude: %f", msg->altitude);
        }
    };
};
#endif