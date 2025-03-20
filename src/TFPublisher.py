#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations

def publish_root_tf():
    # ROS 노드 초기화
    rospy.init_node('world_tf_publisher')

    # TF Broadcaster 생성
    broadcaster = tf2_ros.TransformBroadcaster()

    # 발행할 TF 메시지 생성
    transform = geometry_msgs.msg.TransformStamped()

    # 헤더 설정
    transform.header.frame_id = "map"  # 루트 프레임 (map 또는 world)
    transform.child_frame_id = "waypoint"  # 자식 프레임 (로봇의 기준 프레임)

    # 변환(Transform) 설정
    transform.transform.translation.x = 0.0  # X 축 이동
    transform.transform.translation.y = 0.0  # Y 축 이동
    transform.transform.translation.z = 0.0  # Z 축 이동

    # 회전 설정 (쿼터니언)
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)  # Roll, Pitch, Yaw (라디안)
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    # 주기적으로 TF 발행
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()  # 타임스탬프 업데이트
        broadcaster.sendTransform(transform)       # TF 발행
        rospy.loginfo("Publishing TF: {} -> {}".format(transform.header.frame_id, transform.child_frame_id))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_root_tf()
    except rospy.ROSInterruptException:
        pass