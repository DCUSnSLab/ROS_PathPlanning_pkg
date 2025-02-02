# ROS_PathPlanning_pkg
Pathplanning on specified vertices

## Run example(in morai sim)

- roslaunch rosbridge_server rosbridge_websocket.launch
- roslaunch path_planning A*asfasf
- 

## Node and Topics

- map_server
  - A service node that receives a map file path and returns a graph (consisting of Node and Link) created using the data.
- a_star_path
  - Finding a path between current location and target point using A* algorithm
    - **Subscribed topic**
      - **/move_base_simple/goal** for Set Goal point in RViz
      - **/gps** estimate vehicle position and pose in simulator 
    - **Published topic**
      - **/ego_utm** current position Transformed to UTM Coord
      - **/visualization_marker_array** Visualize map(Graph) in RViz
      - **/ego_vehicle** ego vehicle info
      - **/planned_path** Output of path planning
- pure_pursuit
  - Trajectory following algorithm that drives a vehicle based on a generated path
    - **Subscribed topic**
      - /Ego_topic
      - /ego_utm
      - /planned_path
    - **Published topic**
      - /ctrl_cmd
      - /lookahead_marker

## TF Tree
- Map
  - waypoint
    - ego_vehicle