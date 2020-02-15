#!/usr/bin/env python
# coding=utf8
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

pose_1 = PoseStamped()
pose_2 = PoseStamped()

def pose1_clb(data):
    global  pose_1
    pose_1 = data

def pose2_clb(data):
    global  pose_2
    pose_2 = data


def get_dist(pose1, pose2):
    """

    :type pose1: PoseStamped
    :type pose2: PoseStamped
    :return:
    """

    dist = math.sqrt((pose_1.pose.position.x - pose_2.pose.position.x)**2+ \
    (pose_1.pose.position.y - pose_2.pose.position.y) ** 2 + \
    (pose_1.pose.position.z - pose_2.pose.position.z) ** 2)
    return dist

if __name__ == '__main__':
    # Инициализируем узел ROS
    rospy.init_node("drone_error", anonymous=True)
    print("init node")
    rate = rospy.Rate(50)

    rospy.Subscriber("/drone_0/geo/local_pose", PoseStamped, pose1_clb)
    rospy.Subscriber("/drone_1/geo/local_pose", PoseStamped, pose2_clb)
    pub_target = rospy.Publisher("/nav_pos_error", Float64, queue_size=10)
    dist = 1.315

    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        error = dist - get_dist(pose_1, pose_2)
        print("error: ", error)
        pub_target.publish(error)
        rate.sleep()
