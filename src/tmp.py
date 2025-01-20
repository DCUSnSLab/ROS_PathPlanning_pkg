#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion


def check_tf_transform(parent_frame, child_frame):
    rospy.init_node('tf_transform_checker')

    # TF Buffer와 Listener 설정
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # 두 프레임 간 변환 가져오기
            transform: TransformStamped = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))

            # 변환 정보 출력
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            rospy.loginfo(f"Transform from {parent_frame} to {child_frame}:")
            rospy.loginfo(f"  Translation - x: {translation.x}, y: {translation.y}, z: {translation.z}")

            # Quaternion → Euler 각도로 변환
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            rospy.loginfo(f"  Rotation (Euler) - roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        except tf2_ros.LookupException:
            rospy.logwarn(f"Transform from {parent_frame} to {child_frame} not found.")
        except tf2_ros.ConnectivityException:
            rospy.logwarn("TF connectivity error.")
        except tf2_ros.ExtrapolationException:
            rospy.logwarn("TF extrapolation error.")

        rate.sleep()


if __name__ == '__main__':
    try:
        parent_frame = "waypoint"  # 부모 프레임
        child_frame = "ego_vehicle"  # 자식 프레임
        check_tf_transform(parent_frame, child_frame)
    except rospy.ROSInterruptException:
        pass
