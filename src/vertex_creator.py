#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

'''

'''

class Path_creator():
    def __init__(self):
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
        self.end_sub = rospy.Subscriber('/end', Int32, self.end)
        self.path_pub = rospy.Publisher('/current_vertices', Path, queue_size=1)

        self.thrsh = 2.5 # threshold for connect each vertex
        self.pose_list = list()
        self.path = Path()
        self.prev_pose = None
        self.vertex_count = 0

        self.coordinates_list = list()
        rospy.on_shutdown(self.shutdown)

    def callback(self, goal):
        self.path.header.stamp = goal.header.stamp
        self.path.header.frame_id = goal.header.frame_id

        self.path.poses.append(goal)

        vertex_dict = dict()
        x = goal.pose.position.x
        y = goal.pose.position.y
        vertex_dict["id"] = self.vertex_count
        vertex_dict["xy"] = (x, y)

        if self.prev_pose == None:
            vertex_dict["adjacent"] = None
            self.prev_pose = vertex_dict
        else:
            # print(self.vertex_count)
            # print(str(self.vertex_count))
            each_coord = [d["xy"] for d in self.pose_list]
            print("each_coord")
            print(each_coord)
            print()
            np_each_coord = np.array(each_coord)
            each_distances = np.linalg.norm(np_each_coord - (x, y), axis=1)
            print(each_distances)
            below_thrsh_indices = np.where(each_distances < self.thrsh)[0]
            print("below_thrsh_indices")
            print(below_thrsh_indices)
            print()
            below_thrsh_coords = np_each_coord[below_thrsh_indices]
            print("below_thrsh_coords")
            print(below_thrsh_coords)
            print()
            # vertex_dict["adjacent"] = below_thrsh_coords.tolist()
            vertex_dict["adjacent"] = below_thrsh_indices.tolist()
            if self.vertex_count not in vertex_dict["adjacent"]:
                vertex_dict["adjacent"].append(self.vertex_count)
            self.vertex_count = self.vertex_count + 1

        self.pose_list.append(vertex_dict)

        self.path_pub.publish(self.path)
        
        # print(self.pose_list)
        # print("callback")
        
    def end(self, end):
        print("end")
        
    def shutdown(self):
        self.pose_list[len(self.pose_list) - 1]["adjacent"].append(0)
        
        current_time = time.strftime("%Y-%B-%d-%H:%M:%S", time.localtime())
        
        with open('./path_' + current_time + '.json','w') as f:
        # with open('./path_test.json','w') as f:
            json.dump(self.pose_list, f, ensure_ascii=False, indent=4)
        print("Path file created in $HOME/.ros/ directory")

if __name__ == "__main__":
    try:
        print("Start vertex create..")
        print()
        print("2D Nav Goal : Create vertex")
        print()
        print("Ctrl + C to exit")
        
        rospy.init_node("Create_Path")
        pc = Path_creator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

