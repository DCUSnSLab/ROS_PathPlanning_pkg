import rospy
import numpy as np
import time
import utils

from geometry_msgs.msg import PolygonStamped, Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class PurePursuit(object):
	def __init__(self):
		self.nearest_point   = None
		self.lookahead_point = None
		# set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
		self.viz_namespace = "/pure_pursuit"
		self.nearest_point_pub = rospy.Publisher(self.viz_namespace + "/nearest_point", Marker, queue_size = 1)
		self.lookahead_point_pub = rospy.Publisher(self.viz_namespace + "/lookahead_point", Marker, queue_size = 1)
		# topic to send drive commands to
		self.control_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", Twist, queue_size =1 )
		# topic to listen for trajectories
		self.traj_sub = rospy.Subscriber("/path", PolygonStamped, self.trajectory_callback, queue_size=1)
		# topic to listen for odometry messages, either from particle filter or the simulator
		self.odom_sub = rospy.Subscriber("/odom",  Odometry, self.odom_callback, queue_size=1)
		print("Initialized. Waiting on messages...")


	def trajectory_callback(self, msg):
		''' Clears the currently followed trajectory, and loads the new one from the message
		'''
		print("Receiving new trajectory:", len(msg.polygon.points), "points")
		self.trajectory.clear()
		self.trajectory.fromPolygon(msg.polygon)
		self.trajectory.publish_viz(duration=0.0)

	def odom_callback(self, msg):
		''' Extracts robot state information from the message, and executes pure pursuit control.
		'''
		pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils.quaternion_to_angle(msg.pose.pose.orientation)])
		self.pure_pursuit(pose)
		
		# this is for timing info
		self.odom_timer.tick()
		self.iters += 1
		if self.iters % 20 == 0:
			print("Control fps:", self.odom_timer.fps())

if __name__=="__main__":
	rospy.init_node("pure_pursuit")
	pp = PurePursuit()
	rospy.spin()

