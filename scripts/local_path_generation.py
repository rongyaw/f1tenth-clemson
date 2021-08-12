#!/usr/bin/env python3

import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import global_path_reader, nearest_reference_pose

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vel = msg.twist.twist.linear.x
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    
    def ouput_vehicle_state(self):
        state = np.array([self.x, self.y, self.yaw, self.vel])
        return state

def local_goal_selection(global_path, reference_pose_id, distance_interval, num_path):
    distance = 0.0
    i = reference_pose_id + 1
    delta_i = 0
    while distance < distance_interval:
        i += 1
        if i < num_path:
            distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
            delta_i += 1
        else:
            i -= num_path
            distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
            delta_i += 1
        # if delta_i > 500:
        #     break
    reference_pose = global_path[i,:]
    return reference_pose

def local_goal_publisher_message(local_goal):
    frame_id = 'map'
    local_goal_msg = PoseStamped()
    local_goal_msg.header.frame_id = frame_id
    local_goal_msg.pose.position.x = local_goal[0]
    local_goal_msg.pose.position.y = local_goal[1]
    
    local_goal_quaternion = quaternion_from_euler(0.0, 0.0, local_goal[2])
    local_goal_msg.pose.orientation.x = local_goal_quaternion[0]
    local_goal_msg.pose.orientation.y = local_goal_quaternion[1]
    local_goal_msg.pose.orientation.z = local_goal_quaternion[2]
    local_goal_msg.pose.orientation.w = local_goal_quaternion[3]
    return local_goal_msg

if __name__ == '__main__':
    rospy.init_node('local_path_generation')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)

    # Load the global trajectory
    dirname = os.path.dirname(__file__)
    global_trajectory = rospy.get_param("/racing_line")
    filename = os.path.join(dirname, global_trajectory)
    global_path = global_path_reader(filename)
    num_path = len(global_path[:,0])

    # Initiate vehicle state listener
    rate = rospy.Rate(10)
    vehicle_message = ros_message_listener()
    distance = 3.0
    dist_to_goal = 0.0

    while not rospy.is_shutdown():
        vehicle_state = vehicle_message.ouput_vehicle_state()
        nearest_pose_id = nearest_reference_pose(global_path, vehicle_state)
        goal_pose = local_goal_selection(global_path, nearest_pose_id, distance, num_path)
        goal_pose_message = local_goal_publisher_message(goal_pose)
        goal_pub.publish(goal_pose_message)
        rate.sleep()
