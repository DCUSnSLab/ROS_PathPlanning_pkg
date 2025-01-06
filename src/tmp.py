import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_marker(marker_id, marker_type, position, scale, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "example"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD

    # Position and scale
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

    # Color
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

def main():
    rospy.init_node('marker_array_example')
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    marker_array = MarkerArray()

    # Cube 1
    cube1 = create_marker(
        marker_id=0,
        marker_type=Marker.CUBE,
        position=[0.0, 0.0, 0.0],
        scale=[0.2, 0.2, 0.2],
        color=[1.0, 0.0, 0.0, 1.0]  # Red
    )
    marker_array.markers.append(cube1)

    # Cube 2
    cube2 = create_marker(
        marker_id=1,
        marker_type=Marker.CUBE,
        position=[1.0, 1.0, 0.0],
        scale=[0.2, 0.2, 0.2],
        color=[0.0, 1.0, 0.0, 1.0]  # Green
    )
    marker_array.markers.append(cube2)

    # LineStrip connecting the cubes
    line_strip = Marker()
    line_strip.header.frame_id = "world"
    line_strip.header.stamp = rospy.Time.now()
    line_strip.ns = "example"
    line_strip.id = 2
    line_strip.type = Marker.LINE_STRIP
    line_strip.action = Marker.ADD

    line_strip.scale.x = 0.05  # Line width

    line_strip.color.r = 0.0
    line_strip.color.g = 0.0
    line_strip.color.b = 1.0
    line_strip.color.a = 1.0  # Blue

    # Points for the line
    point1 = Point(0.0, 0.0, 0.0)
    point2 = Point(1.0, 1.0, 0.0)
    line_strip.points.append(point1)
    line_strip.points.append(point2)

    marker_array.markers.append(line_strip)

    # Publish the MarkerArray
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
