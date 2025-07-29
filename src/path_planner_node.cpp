#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gmserver/srv/load_map.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>

// A* Node structure
struct AStarNode {
    int id;
    geometry_msgs::msg::Pose pose;
    double g_cost;  // Cost from start
    double h_cost;  // Heuristic cost to goal
    double f_cost;  // Total cost (g + h)
    int parent_id;  // Parent node ID for path reconstruction
    
    AStarNode(int node_id, geometry_msgs::msg::Pose node_pose) 
        : id(node_id), pose(node_pose), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent_id(-1) {}
    
    AStarNode(int node_id, geometry_msgs::msg::Pose node_pose, double g, double h, int parent)
        : id(node_id), pose(node_pose), g_cost(g), h_cost(h), f_cost(g + h), parent_id(parent) {}
};

// Comparator for priority queue (min-heap based on f_cost)
struct AStarNodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f_cost > b->f_cost;  // Min-heap
    }
};

// Link structure for graph representation
struct Link {
    int from_node_id;
    int to_node_id;
    double length;
    
    Link(int from, int to, double len) : from_node_id(from), to_node_id(to), length(len) {}
};

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("global_path_planner_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("map_file_path", 
            "/home/ros2/ros2_ws/src/gmserver/maps/3x3_map.json");
        this->declare_parameter<double>("gps_reference_latitude", 37.5665);
        this->declare_parameter<double>("gps_reference_longitude", 126.9780);
        this->declare_parameter<double>("gps_reference_altitude", 50.0);
        
        // Get GPS reference coordinates
        this->get_parameter("gps_reference_latitude", gps_ref_lat_);
        this->get_parameter("gps_reference_longitude", gps_ref_lon_);
        this->get_parameter("gps_reference_altitude", gps_ref_alt_);
        
        // Calculate GPS reference UTM coordinates for goal transformation
        gpsToUTM(gps_ref_lat_, gps_ref_lon_, gps_ref_utm_easting_, gps_ref_utm_northing_);
        
        // Initialize state variables
        current_gps_received_ = false;
        goal_received_ = false;
        
        // Create service client for map loading
        map_client_ = this->create_client<gmserver::srv::LoadMap>("load_map");
        
        // Create subscribers
        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10,
            std::bind(&PathPlannerNode::gpsCallback, this, std::placeholders::_1));
            
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));
        
        // Create publishers
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        nodes_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("map_nodes_viz", 10);
        links_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("map_links_viz", 10);
        map_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("map_graph_viz", 10);
        
        // Create timer for checking path planning conditions
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PathPlannerNode::checkAndPlanPath, this));
        
        RCLCPP_INFO(this->get_logger(), "Global path planner node initialized with A* algorithm");
        
        // Load map initially
        loadMapData();
    }

private:
    void loadMapData()
    {
        RCLCPP_INFO(this->get_logger(), "Starting map data loading...");
        
        // Wait for service to be available
        if (!map_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Map service not available after 10 seconds");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Map service is available");
        
        // Get map file path parameter
        std::string map_file_path;
        this->get_parameter("map_file_path", map_file_path);
        
        // Create service request
        auto request = std::make_shared<gmserver::srv::LoadMap::Request>();
        request->map_file_path = map_file_path;
        
        RCLCPP_INFO(this->get_logger(), "Requesting map data from: %s", map_file_path.c_str());
        
        // Call service asynchronously
        auto future = map_client_->async_send_request(request);
        
        RCLCPP_INFO(this->get_logger(), "Map service request sent, waiting for response...");
        
        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Map service response received: success=%s", 
                       response->success ? "true" : "false");
            
            if (response->success) {
                // Store map data
                map_nodes_ = response->nodes;
                map_links_ = response->links;
                
                RCLCPP_INFO(this->get_logger(), 
                           "Map data stored: %zu nodes, %zu links", 
                           map_nodes_.poses.size(), map_links_.poses.size());
                
                // Build graph from map data
                buildGraph();
                
                RCLCPP_INFO(this->get_logger(), 
                           "Map loaded successfully: %zu nodes, %zu links", 
                           map_nodes_.poses.size(), map_links_.poses.size());
                
                // Publish visualization data
                publishVisualizationData();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", 
                            response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call map service - timeout or error");
        }
    }
    
    void buildGraph()
    {
        // Clear existing graph
        node_map_.clear();
        adjacency_list_.clear();
        
        // Build node map
        for (size_t i = 0; i < map_nodes_.poses.size(); ++i) {
            node_map_[static_cast<int>(i)] = std::make_shared<AStarNode>(static_cast<int>(i), map_nodes_.poses[i]);
        }
        
        // Build adjacency list from actual link data
        // This builds proper graph connectivity based on map_links_ data
        for (size_t i = 0; i < map_links_.poses.size(); i += 2) {
            if (i + 1 < map_links_.poses.size()) {
                // Each link consists of two poses (from and to)
                int from_id = static_cast<int>(i / 2);  // Link index maps to connections
                int to_id = static_cast<int>((i / 2) + 1);
                
                // Find closest nodes for the link endpoints
                auto from_pose = map_links_.poses[i];
                auto to_pose = map_links_.poses[i + 1];
                
                int from_node_id = findClosestNodeToPosition(from_pose.position.x, from_pose.position.y);
                int to_node_id = findClosestNodeToPosition(to_pose.position.x, to_pose.position.y);
                
                if (from_node_id != -1 && to_node_id != -1 && from_node_id != to_node_id) {
                    double distance = calculateDistance(map_nodes_.poses[from_node_id], map_nodes_.poses[to_node_id]);
                    
                    // Add bidirectional links
                    adjacency_list_[from_node_id].emplace_back(to_node_id, from_node_id, distance);
                    adjacency_list_[to_node_id].emplace_back(from_node_id, to_node_id, distance);
                }
            }
        }
        
        // If no links data available, create 3x3 grid connections based on node positions
        if (adjacency_list_.empty() && map_nodes_.poses.size() == 9) {
            RCLCPP_WARN(this->get_logger(), "No valid links found, creating 3x3 grid connections");
            
            // Create 3x3 grid connections (assuming nodes are ordered as grid)
            // Horizontal connections: 0-1, 1-2, 3-4, 4-5, 6-7, 7-8
            std::vector<std::pair<int, int>> grid_connections = {
                {0, 1}, {1, 2},  // Row 0
                {3, 4}, {4, 5},  // Row 1  
                {6, 7}, {7, 8},  // Row 2
                {0, 3}, {3, 6},  // Col 0
                {1, 4}, {4, 7},  // Col 1
                {2, 5}, {5, 8}   // Col 2
            };
            
            for (auto& connection : grid_connections) {
                int from_id = connection.first;
                int to_id = connection.second;
                
                if (from_id < map_nodes_.poses.size() && to_id < map_nodes_.poses.size()) {
                    double distance = calculateDistance(map_nodes_.poses[from_id], map_nodes_.poses[to_id]);
                    
                    adjacency_list_[from_id].emplace_back(to_id, from_id, distance);
                    adjacency_list_[to_id].emplace_back(from_id, to_id, distance);
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Graph built with %zu nodes and connectivity for %zu nodes", 
                   node_map_.size(), adjacency_list_.size());
        
        // Debug: Print adjacency list
        for (const auto& pair : adjacency_list_) {
            std::string connections;
            for (const auto& link : pair.second) {
                int neighbor = (link.from_node_id == pair.first) ? link.to_node_id : link.from_node_id;
                connections += std::to_string(neighbor) + " ";
            }
            RCLCPP_DEBUG(this->get_logger(), "Node %d connected to: %s", pair.first, connections.c_str());
        }
    }
    
    void publishVisualizationData()
    {
        visualization_msgs::msg::MarkerArray viz_graph;
        visualization_msgs::msg::Marker viz_marker;

        int i = 0;

        // Adjust map nodes for RViz visualization
        if (!map_nodes_.poses.empty()) {
            geometry_msgs::msg::PoseArray viz_nodes = map_nodes_;
            for (auto& pose : viz_nodes.poses) {
                pose.position.x -= gps_ref_utm_easting_;
                pose.position.y -= gps_ref_utm_northing_;

                viz_marker.header.frame_id = "map";
                viz_marker.header.stamp = this->get_clock()->now();
                viz_marker.ns = "graph";
                viz_marker.id = i;
                viz_marker.type = visualization_msgs::msg::Marker::CUBE;
                viz_marker.scale.x = 10;
                viz_marker.scale.y = 10;
                viz_marker.scale.z = 10;
                viz_marker.color.a = 1.0;
                viz_marker.color.r = 0.0;
                viz_marker.color.g = 1.0;
                viz_marker.color.b = 0.0;
                viz_marker.pose.position.x = pose.position.x;
                viz_marker.pose.position.y = pose.position.y;
                viz_marker.pose.position.z = 0;
                viz_graph.markers.push_back(viz_marker);

                i++;
            }
            viz_nodes.header.stamp = this->get_clock()->now();
            nodes_publisher_->publish(viz_nodes);
        }
        
        // Adjust map links for RViz visualization
        if (!map_links_.poses.empty()) {
            geometry_msgs::msg::PoseArray viz_links = map_links_;
            for (auto& pose : viz_links.poses) {
                pose.position.x -= gps_ref_utm_easting_;
                pose.position.y -= gps_ref_utm_northing_;

                viz_marker.header.frame_id = "map";
                viz_marker.header.stamp = this->get_clock()->now();
                viz_marker.ns = "graph";
                viz_marker.id = i;
                viz_marker.type = visualization_msgs::msg::Marker::SPHERE;
                viz_marker.scale.x = 10;
                viz_marker.scale.y = 10;
                viz_marker.scale.z = 10;
                viz_marker.color.a = 1.0;
                viz_marker.color.r = 1.0;
                viz_marker.color.g = 0.0;
                viz_marker.color.b = 0.0;
                viz_marker.pose.position.x = pose.position.x;
                viz_marker.pose.position.y = pose.position.y;
                viz_marker.pose.position.z = 0;
                viz_graph.markers.push_back(viz_marker);

                i++;
            }
            viz_links.header.stamp = this->get_clock()->now();
            links_publisher_->publish(viz_links);
            
        }

        map_viz_publisher_->publish(viz_graph);
        
        RCLCPP_DEBUG(this->get_logger(), "Published visualization data");
    }
    
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg->status.status < 0) {
            return; // Invalid GPS fix
        }
        
        current_gps_ = *msg;
        current_gps_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "GPS received: lat=%.6f, lon=%.6f", 
                    msg->latitude, msg->longitude);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!current_gps_received_) {
            RCLCPP_WARN(this->get_logger(), "Goal received, but no GPS data yet. Waiting for GPS.");
            return;
        }

        // RViz goal is relative to map origin, convert to absolute UTM coordinates
        // Add GPS reference UTM coordinates to get absolute position
        goal_pose_ = *msg;
        goal_pose_.pose.position.x += gps_ref_utm_easting_;
        goal_pose_.pose.position.y += gps_ref_utm_northing_;
        // z coordinate can remain relative
        
        goal_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Goal received - RViz: (%.2f, %.2f) -> Absolute UTM: (%.2f, %.2f)", 
                   msg->pose.position.x, msg->pose.position.y,
                   goal_pose_.pose.position.x, goal_pose_.pose.position.y);
        
        // Trigger immediate path planning
        planPathFromGpsToGoal();
    }
    
    void checkAndPlanPath()
    {
        // Only plan path when we have both current GPS and goal
        if (current_gps_received_ && goal_received_) {
            planPathFromGpsToGoal();
        }
    }
    
    void planPathFromGpsToGoal()
    {
        if (node_map_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No map data available for path planning");
            return;
        }
        
        if (!current_gps_received_ || !goal_received_) {
            RCLCPP_WARN(this->get_logger(), "GPS or Goal not available for path planning");
            return;
        }
        
        // Convert GPS to UTM coordinates
        double start_utm_easting, start_utm_northing;
        gpsToUTM(current_gps_.latitude, current_gps_.longitude, start_utm_easting, start_utm_northing);
        
        // Goal has been converted to UTM coordinates in goalCallback
        double goal_x = goal_pose_.pose.position.x;
        double goal_y = goal_pose_.pose.position.y;
        
        // Find closest nodes to start and goal positions
        int start_node_id = findClosestNode(start_utm_easting, start_utm_northing);
        int goal_node_id = findClosestNode(goal_x, goal_y);
        
        if (start_node_id == -1 || goal_node_id == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not find valid start or goal nodes");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning path from GPS (UTM: %.2f, %.2f) -> node %d to Goal (%.2f, %.2f) -> node %d", 
                   start_utm_easting, start_utm_northing, start_node_id, goal_x, goal_y, goal_node_id);
        
        // Plan path using A*
        auto path_nodes = planAStarPath(start_node_id, goal_node_id);
        
        if (!path_nodes.empty()) {
            // Convert to ROS Path message and adjust for RViz
            nav_msgs::msg::Path planned_path;
            planned_path.header.frame_id = "map";
            planned_path.header.stamp = this->get_clock()->now();
            
            for (const auto& node : path_nodes) {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "map";
                pose_stamped.header.stamp = this->get_clock()->now();
                pose_stamped.pose = node->pose;
                pose_stamped.pose.position.x -= gps_ref_utm_easting_;
                pose_stamped.pose.position.y -= gps_ref_utm_northing_;
                
                planned_path.poses.push_back(pose_stamped);
            }
            
            // Publish planned path
            path_publisher_->publish(planned_path);
            
            RCLCPP_INFO(this->get_logger(), "Published A* path with %zu waypoints", 
                       planned_path.poses.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "No path found from node %d to node %d", start_node_id, goal_node_id);
        }
    }
    
    // A* path planning algorithm implementation
    std::vector<std::shared_ptr<AStarNode>> planAStarPath(int start_id, int goal_id)
    {
        if (node_map_.find(start_id) == node_map_.end() || 
            node_map_.find(goal_id) == node_map_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid start or goal node ID");
            return {};
        }
        
        // Priority queue for open set (min-heap)
        std::priority_queue<std::shared_ptr<AStarNode>, 
                           std::vector<std::shared_ptr<AStarNode>>, 
                           AStarNodeComparator> open_set;
        
        // Sets to track visited nodes
        std::unordered_set<int> open_set_ids;
        std::unordered_set<int> closed_set;
        
        // Map to store best g_cost for each node
        std::unordered_map<int, double> best_g_cost;
        
        // Map to store parent relationships for path reconstruction
        std::unordered_map<int, int> parent_map;
        
        // Initialize start node
        auto start_node = std::make_shared<AStarNode>(*node_map_[start_id]);
        start_node->g_cost = 0.0;
        start_node->h_cost = calculateHeuristic(start_node->pose, node_map_[goal_id]->pose);
        start_node->f_cost = start_node->g_cost + start_node->h_cost;
        start_node->parent_id = -1;
        
        open_set.push(start_node);
        open_set_ids.insert(start_id);
        best_g_cost[start_id] = 0.0;
        
        while (!open_set.empty()) {
            // Get node with lowest f_cost
            auto current = open_set.top();
            open_set.pop();
            open_set_ids.erase(current->id);
            
            // Add to closed set
            closed_set.insert(current->id);
            
            // Check if we reached the goal
            if (current->id == goal_id) {
                RCLCPP_INFO(this->get_logger(), "A* path found with cost: %.2f", current->f_cost);
                return reconstructPath(current, parent_map);
            }
            
            // Explore neighbors
            if (adjacency_list_.find(current->id) != adjacency_list_.end()) {
                for (const auto& link : adjacency_list_[current->id]) {
                    int neighbor_id = link.from_node_id == current->id ? link.to_node_id : link.from_node_id;
                    
                    // Skip if in closed set
                    if (closed_set.find(neighbor_id) != closed_set.end()) {
                        continue;
                    }
                    
                    // Calculate tentative g_cost
                    double tentative_g = current->g_cost + link.length;
                    
                    // Check if this path to neighbor is better
                    if (best_g_cost.find(neighbor_id) == best_g_cost.end() || 
                        tentative_g < best_g_cost[neighbor_id]) {
                        
                        // Update best g_cost and parent
                        best_g_cost[neighbor_id] = tentative_g;
                        parent_map[neighbor_id] = current->id;
                        
                        // Create neighbor node
                        auto neighbor = std::make_shared<AStarNode>(*node_map_[neighbor_id]);
                        neighbor->g_cost = tentative_g;
                        neighbor->h_cost = calculateHeuristic(neighbor->pose, node_map_[goal_id]->pose);
                        neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                        neighbor->parent_id = current->id;
                        
                        // Add to open set if not already there
                        if (open_set_ids.find(neighbor_id) == open_set_ids.end()) {
                            open_set.push(neighbor);
                            open_set_ids.insert(neighbor_id);
                        }
                    }
                }
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "No A* path found from %d to %d", start_id, goal_id);
        return {};
    }
    
    // Reconstruct path from goal to start using parent relationships
    std::vector<std::shared_ptr<AStarNode>> reconstructPath(
        std::shared_ptr<AStarNode> goal_node,
        const std::unordered_map<int, int>& parent_map)
    {
        std::vector<std::shared_ptr<AStarNode>> path;
        int current_id = goal_node->id;
        
        // Build path backwards from goal to start
        while (current_id != -1) {
            path.push_back(node_map_[current_id]);
            
            auto parent_it = parent_map.find(current_id);
            current_id = (parent_it != parent_map.end()) ? parent_it->second : -1;
        }
        
        // Reverse to get path from start to goal
        std::reverse(path.begin(), path.end());
        
        return path;
    }
    
    // Calculate heuristic (Euclidean distance)
    double calculateHeuristic(const geometry_msgs::msg::Pose& a, 
                             const geometry_msgs::msg::Pose& b)
    {
        return calculateDistance(a, b);
    }
    
    // Calculate Euclidean distance between two poses
    double calculateDistance(const geometry_msgs::msg::Pose& a, 
                           const geometry_msgs::msg::Pose& b)
    {
        double dx = a.position.x - b.position.x;
        double dy = a.position.y - b.position.y;
        double dz = a.position.z - b.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Universal GPS to UTM conversion
    void gpsToUTM(double lat, double lon, double& easting, double& northing)
    {
        // WGS84 ellipsoid parameters
        const double a = 6378137.0;           // Semi-major axis
        const double f = 1.0 / 298.257223563; // Flattening
        const double k0 = 0.9996;            // UTM scale factor

        // Determine the UTM zone
        int zone = static_cast<int>((lon + 180.0) / 6.0) + 1;

        // Calculate the central meridian for the zone
        double lon0_deg = (zone - 1) * 6 - 180 + 3;
        double lon0_rad = lon0_deg * M_PI / 180.0;

        // False easting and northing
        double false_easting = 500000.0;
        double false_northing = (lat < 0) ? 10000000.0 : 0.0; // Southern hemisphere

        // Convert lat/lon to radians
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        // Equations for conversion
        double e2 = 2 * f - f * f;
        double e_prime_sq = e2 / (1.0 - e2);

        double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
        double T = std::tan(lat_rad) * std::tan(lat_rad);
        double C = e_prime_sq * std::cos(lat_rad) * std::cos(lat_rad);
        double A = std::cos(lat_rad) * (lon_rad - lon0_rad);

        double M = a * ((1.0 - e2/4.0 - 3.0*e2*e2/64.0 - 5.0*e2*e2*e2/256.0) * lat_rad
                       - (3.0*e2/8.0 + 3.0*e2*e2/32.0 + 45.0*e2*e2*e2/1024.0) * std::sin(2.0*lat_rad)
                       + (15.0*e2*e2/256.0 + 45.0*e2*e2*e2/1024.0) * std::sin(4.0*lat_rad)
                       - (35.0*e2*e2*e2/3072.0) * std::sin(6.0*lat_rad));

        easting = false_easting + k0 * N * (A + (1.0 - T + C) * std::pow(A, 3) / 6.0
                                           + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_prime_sq) * std::pow(A, 5) / 120.0);

        northing = false_northing + k0 * (M + N * std::tan(lat_rad) * (std::pow(A, 2) / 2.0
                                                                      + (5.0 - T + 9.0*C + 4.0*C*C) * std::pow(A, 4) / 24.0
                                                                      + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*e_prime_sq) * std::pow(A, 6) / 720.0));
    }
    
    // Find closest node to given UTM coordinates
    int findClosestNode(double utm_x, double utm_y)
    {
        if (node_map_.empty()) {
            return -1;
        }
        
        int closest_id = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& pair : node_map_) {
            int node_id = pair.first;
            const auto& node = pair.second;
            
            double dx = node->pose.position.x - utm_x;
            double dy = node->pose.position.y - utm_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_id = node_id;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Closest node to (%.2f, %.2f) is node %d at distance %.2f", 
                    utm_x, utm_y, closest_id, min_distance);
        
        return closest_id;
    }
    
    // Helper function for buildGraph - find closest node to a position
    int findClosestNodeToPosition(double x, double y)
    {
        if (map_nodes_.poses.empty()) {
            return -1;
        }
        
        int closest_id = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < map_nodes_.poses.size(); ++i) {
            double dx = map_nodes_.poses[i].position.x - x;
            double dy = map_nodes_.poses[i].position.y - y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_id = static_cast<int>(i);
            }
        }
        
        return closest_id;
    }
    
    // Member variables
    rclcpp::Client<gmserver::srv::LoadMap>::SharedPtr map_client_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr nodes_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr links_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_viz_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseArray map_nodes_;
    geometry_msgs::msg::PoseArray map_links_;
    
    // GPS and Goal state
    sensor_msgs::msg::NavSatFix current_gps_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool current_gps_received_;
    bool goal_received_;
    
    // GPS reference coordinates for goal transformation
    double gps_ref_lat_;
    double gps_ref_lon_;
    double gps_ref_alt_;
    double gps_ref_utm_easting_;
    double gps_ref_utm_northing_;
    
    // A* algorithm data structures
    std::unordered_map<int, std::shared_ptr<AStarNode>> node_map_;
    std::unordered_map<int, std::vector<Link>> adjacency_list_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}