#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

import math
from utils import global_path_reader
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import csv
import os

def global_path_publisher(path):
    frame_id = 'map'
    msg = Path()
    msg.header.frame_id = frame_id
    for i in range(len(path)):
        x = path[i][0]
        y = path[i][1]
        yaw = path[i][2]
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        msg.poses.append(pose)
    return msg

if __name__ == "__main__":
    rospy.init_node('global_path_reference')
    path_pub = rospy.Publisher('reference_trajectory', Path, queue_size=2)
    dirname = os.path.dirname(__file__)
    global_trajectory = rospy.get_param("/racing_line")
    filename = os.path.join(dirname, global_trajectory)
    global_path = global_path_reader(filename)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        path_message = global_path_publisher(global_path)
        path_pub.publish(path_message)
        rate.sleep()