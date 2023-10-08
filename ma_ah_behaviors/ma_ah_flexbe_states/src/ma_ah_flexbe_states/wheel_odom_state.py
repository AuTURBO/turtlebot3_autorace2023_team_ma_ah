#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
def set_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # ROS 시스템이 초기화될 때까지 대기

    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = "map"  # 초기 위치는 'map' 프레임 기준으로 설정
    initial_pose_msg.header.stamp = rospy.Time.now()

    # 원하는 초기 위치와 방향 설정
    initial_pose_msg.pose.pose.position.x = -2.5
    initial_pose_msg.pose.pose.position.y = 2.4

    # 원하는 방향 (1.57 라디안, 즉 π/2 라디안) 설정
    qt = transformations.quaternion_from_euler(0, 0, -1.57)
    initial_pose_msg.pose.pose.orientation = Quaternion(*qt)
    # initial_pose_msg.pose.pose.orientation.z = qt[2]
    pub.publish(initial_pose_msg)
    rospy.loginfo("Initial pose set to (1.0, 5.0)")

if __name__ == '__main__':
    try:
        set_initial_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
