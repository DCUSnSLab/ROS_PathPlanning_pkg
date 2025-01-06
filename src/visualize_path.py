import json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from pyproj import Proj, transform

def gps_to_utm(lat, lon):
    wgs84 = Proj(proj="latlong", datum="WGS84")
    utm = Proj(proj="utm", zone=33, datum="WGS84")  # Adjust zone as needed
    x, y = transform(wgs84, utm, lon, lat)  # Note: (lon, lat) order
    return x, y

def create_marker(marker_id, marker_type, position, scale, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoint_visualization"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

def parse_json_and_visualize(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)

    marker_array = MarkerArray()
    node_positions = {}

    # Process Nodes
    for i, node in enumerate(data["Node"]):
        node_id = node["ID"]
        gps_info = node["GpsInfo"]
        lat, lon, alt = gps_info["Lat"], gps_info["Long"], gps_info["Alt"]
        utm_x, utm_y = gps_to_utm(lat, lon)
        print(utm_x, utm_y)
        position = [utm_x, utm_y, alt]
        node_positions[node_id] = position

        cube_marker = create_marker(
            marker_id=i,
            marker_type=Marker.CUBE,
            position=position,
            scale=[0.5, 0.5, 0.5],
            color=[0.0, 1.0, 0.0, 1.0]  # Green
        )
        marker_array.markers.append(cube_marker)

    # Process Links
    for j, link in enumerate(data.get("Link", [])):
        from_node = link["FromNodeID"]
        to_node = link["ToNodeID"]

        if from_node in node_positions and to_node in node_positions:
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "waypoint_visualization"
            line_marker.id = len(data["Node"]) + j
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.1
            line_marker.color.r = 1.0
            line_marker.color.g = 0.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            start_position = node_positions[from_node]
            end_position = node_positions[to_node]

            line_marker.points.append(Point(*start_position))
            line_marker.points.append(Point(*end_position))

            marker_array.markers.append(line_marker)

    return marker_array

def main():
    rospy.init_node('waypoint_visualization')
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    file_path = '/home/ros/SCV2/src/scv_system/global_path/ROS_PathPlanning_pkg/src/waypoint_graph.json'  # Update with actual path
    marker_array = parse_json_and_visualize(file_path)

    rate = rospy.Rate(1)  # 10 Hz
    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
