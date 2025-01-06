#!/usr/bin/env python
import rospy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Pose

class InteractiveMarkerDemo:
    def __init__(self):
        rospy.init_node("interactive_marker_demo", anonymous=True)

        # Interactive Marker 서버 생성
        self.server = InteractiveMarkerServer("interactive_marker_server")

        # 초기 마커 생성
        self.create_interactive_marker()

        rospy.loginfo("Interactive marker server started. Open RViz to interact with the marker.")
        rospy.spin()

    def create_interactive_marker(self):
        # Interactive Marker 생성
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "move_marker"
        int_marker.description = "Drag to move the marker"
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0

        # Visual Marker (표시용 마커)
        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = 1.0
        visual_marker.scale.y = 1.0
        visual_marker.scale.z = 1.0
        visual_marker.color.r = 0.0
        visual_marker.color.g = 1.0
        visual_marker.color.b = 0.0
        visual_marker.color.a = 1.0

        # Control 생성 (조작을 위한 핸들)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(visual_marker)
        int_marker.controls.append(control)

        # Drag & Drop을 위한 Control 추가
        move_control = InteractiveMarkerControl()
        move_control.name = "move_x_y"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(move_control)

        # 서버에 마커 등록
        self.server.insert(int_marker, self.process_feedback)

        # 변경사항 적용
        self.server.applyChanges()

    def process_feedback(self, feedback):
        """마커가 움직일 때 호출되는 콜백 함수"""
        rospy.loginfo(
            f"Marker moved to: x={feedback.pose.position.x}, y={feedback.pose.position.y}, z={feedback.pose.position.z}"
        )

if __name__ == "__main__":
    try:
        InteractiveMarkerDemo()
    except rospy.ROSInterruptException:
        pass
