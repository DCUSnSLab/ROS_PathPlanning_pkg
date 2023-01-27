import rospy
import numpy as np

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd

class PurePursuit:
    def __init__(self, lookahead_distance, max_steering_angle):
        self.lookahead_distance = lookahead_distance
        self.max_steering_angle = max_steering_angle

    def calculate_steering_angle(self, current_pose, path):
        # Initialize closest point and distance to maximum value
        closest_point = None
        closest_distance = float('inf')

        # Find the point on the path closest to the current position
        for point in path:
            # distance = np.linalg.norm(point - current_pose[:2]) # Original code
            norm_input = [point.x - current_pose[0], point.y - current_pose[1]]
            distance = np.linalg.norm(norm_input)
            if distance < closest_distance:
                closest_distance = distance
                closest_point = point

        # Calculate the lookahead point

        closest_current_sub = [closest_point.x - current_pose[0], closest_point.y - current_pose[1]]
        # print("closest")
        # print(closest_current_sub)
        
        # print(closest_point)
        # print(closest_distance)
        
        refined_closest_point = [closest_point.x, closest_point.y]

        lookahead_point = refined_closest_point + self.lookahead_distance * (closest_current_sub) / closest_distance

        # Calculate the steering angle
        steering_angle = np.arctan2(lookahead_point[1] - current_pose[1], lookahead_point[0] - current_pose[0]) - current_pose[2]

        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        print("steering_angle")
        print(steering_angle)

        return steering_angle

class Controller:
    def __init__(self):
        print("Controller __init__ called.")
        self.path = []
        self.pure_pursuit = PurePursuit(1, 3)
        self.pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.path_sub = rospy.Subscriber("/refined_vertices", MarkerArray, self.path_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.spin()
        
    def path_callback(self, msg):
        print("path_callback called.")
        self.path = msg.markers
        
    def odom_callback(self, msg):
        print("odom_callback called.")
        current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z]
        path_points = [Point(p.pose.position.x, p.pose.position.y, 0.0) for p in self.path]
        steering_angle = self.pure_pursuit.calculate_steering_angle(current_pose, path_points)
        self.pub.publish(steering_angle)
        
if __name__=="__main__":
        rospy.init_node("Pure_pursuit")
        try:
            Controller()
        except:
            pass
