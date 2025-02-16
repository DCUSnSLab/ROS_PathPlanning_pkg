#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from geometry_msgs.msg import Twist
from morai_msgs.msg import CtrlCmd

class PIDController:
    """ 간단한 PID 컨트롤러 클래스 """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # 비례 계수
        self.Ki = Ki  # 적분 계수
        self.Kd = Kd  # 미분 계수

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, target, current):
        """ PID 연산을 수행하여 제어 출력을 계산 """
        error = target - current
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1e-3

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.last_time = current_time
        return output

class TwistTo4WD:
    def __init__(self):
        rospy.init_node("twist_to_4wd", anonymous=True)

        # /cmd_vel 구독
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 차량 제어 명령 발행
        self.control_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        # 차량 파라미터 설정
        self.max_speed = 1.0  # 최대 선속도 (m/s)
        self.max_steering_angle = 28.2  # 최대 조향각 (degree)
        self.wheel_base = 3.0  # 차량의 휠베이스 (m)

        # 현재 차량 속도 및 조향각 저장 변수 (초기값 0)
        self.current_speed = 0.0
        self.current_steering = 0.0

        # PID 컨트롤러 초기화 (속도 및 조향)
        self.speed_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)   # 속도 PID (튜닝 가능)
        #self.steering_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.02)  # 조향 PID (튜닝 가능)
        self.steering_pid = PIDController(Kp=0.3, Ki=0.01, Kd=0.04)  # 조향 PID (튜닝 가능)

    def cmd_vel_callback(self, msg):
        """ Twist 메시지를 받아 차량 제어 명령을 생성 """
        ctrl_cmd = CtrlCmd()

        # 목표 속도 설정
        target_speed = msg.linear.x
        self.current_speed += self.speed_pid.compute(target_speed, self.current_speed)  # PID 기반 속도 제어

        # 목표 조향각 계산 (Ackermann 모델 적용)
        if msg.angular.z != 0 and msg.linear.x != 0:
            turning_radius = msg.linear.x / msg.angular.z
            target_steering_angle = math.degrees(math.atan(self.wheel_base / turning_radius))
        else:
            target_steering_angle = 0.0

        # PID로 조향각 보정
        self.current_steering += self.steering_pid.compute(target_steering_angle, self.current_steering)

        # 조향각을 -1.0 ~ 1.0 범위로 정규화
        normalized_steering = self.current_steering / self.max_steering_angle
        normalized_steering = max(min(normalized_steering, 1.0), -1.0)

        ctrl_cmd.steering = normalized_steering
        ctrl_cmd.accel = max(min(self.current_speed / self.max_speed, 1.0), -1.0)  # -1 ~ 1 범위 유지

        # 제어 명령 발행
        self.control_pub.publish(ctrl_cmd)

        # rospy.loginfo(f"Publishing -> Accel: {ctrl_cmd.accel:.2f}, Steering: {ctrl_cmd.steering:.2f}")
if __name__ == '__main__':
    try:
        converter = TwistTo4WD()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass