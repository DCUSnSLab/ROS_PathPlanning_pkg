#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd

class CmdVelToCtrlCmd:
    def __init__(self):
        rospy.init_node('cmd_vel_to_ctrl_cmd', anonymous=True)

        # /cmd_vel 구독
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # /ctrl_cmd 발행
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

    def cmd_vel_callback(self, msg):
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.accel = msg.linear.x  # 속도 변환
        ctrl_cmd.steering = msg.angular.z  # 조향 변환

        # 변환된 메시지 발행
        self.ctrl_cmd_pub.publish(ctrl_cmd)

if __name__ == '__main__':
    try:
        node = CmdVelToCtrlCmd()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
