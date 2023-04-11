#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import json
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

'''

'''

class Path_creator():
    def __init__(self):
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
        self.end_sub = rospy.Subscriber('/end', Int32, self.end)
        self.pose_list = list()
        self.prev_pose = None
        self.vertex_count = 0
        rospy.on_shutdown(self.shutdown)

    def callback(self, goal):
        vertex_dict = dict()
        x = goal.pose.position.x
        y = goal.pose.position.y
        vertex_dict["xy"] = (x, y)

        if self.prev_pose == None:
            vertex_dict["adjacent"] = None
            self.prev_pose = vertex_dict
        else:
            print(self.vertex_count)
            print(str(self.vertex_count))
            vertex_dict["adjacent"] = list([str(self.vertex_count)])
            self.vertex_count = self.vertex_count + 1

        self.pose_list.append(vertex_dict)
        
        # print(self.pose_list)
        # print("callback")
        
    def end(self, end):
        print("end")
        
    def shutdown(self):
        self.pose_list[len(self.pose_list) - 1]["adjacent"].append("0")
        
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

