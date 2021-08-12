#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from utils import *

import math
from utils import *
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import csv
import os

# Define necessary vehicle parameters
WB = 0.25 # Wheelbase of the vehicle
vel_max = 7.0 # Set the maximum velocity
vel_min = 1.0 # Set the minimum velocity
steering_rate_max = 4.2 # Set the maximum steering angle velocity
acceleration_max = 4.5 # Set the maximum acceleration
max_steer = 0.4189 # Set the maximum steering angle

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.current_speed = 0.0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.current_speed = msg.twist.twist.linear.x

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.current_speed])
        return vehicle_state

def pure_pursuit_control(goal_state, current_state):
    v_pose, v_goal_pose = vehicle_coordinate_transformation(goal_state, current_state)
    
    # Calculate steering angle
    d_yaw = math.atan(v_goal_pose[1]/v_goal_pose[0])
    L = ((v_goal_pose[0])**2 + (v_goal_pose[1])**2)**0.5
    angle = math.atan(2 * WB * math.sin(d_yaw) / L)

    # Apply steering angle constraints
    angle = np.clip(angle, -max_steer,  max_steer)

    # Apply acceleration
    d_v = goal_state[3] - current_state[3]
    a = d_v*1.25
    return angle, a

def reference_pose_selection(global_path, reference_pose_id, ahead_distance, num_path):
    distance = 0.0
    i = reference_pose_id + 1
    while distance < ahead_distance:
        if i < num_path:
            distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
            i += 1
        else:
            i -= num_path
            distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
            i += 1
    reference_pose = np.array([global_path[i,0], global_path[i,1], global_path[i,2], global_path[i,3]])
    return reference_pose

def look_ahead_distance_modification(v):
    ahead_distance_max = 1.0
    ahead_distance_min = 0.5
    k = (ahead_distance_max - ahead_distance_min)/(vel_max - vel_min)
    ahead_distance = k * (v - vel_min) + ahead_distance_min
    return ahead_distance

if __name__ == "__main__":
    rospy.init_node('pure_pursuit_controller', anonymous=True)
    drive_pub = rospy.Publisher('car/mpc_control_signal', AckermannDriveStamped, queue_size=1)
    
    # Load the information of global trajectory
    dirname = os.path.dirname(__file__)
    global_trajectory = rospy.get_param("/racing_line")
    filename = os.path.join(dirname, str(global_trajectory))
    global_path = global_path_reader(filename)
    num_path = len(global_path[:,0])

    # Initiate ros message listener and initial vehicle conditions
    rate = rospy.Rate(100)
    vehicle_pose_msg = ros_message_listener() 
    ahead_distance = 0.75

    while not rospy.is_shutdown():
        try:
            current_state = vehicle_pose_msg.vehicle_state_output()
            nearest_reference_id = nearest_reference_pose(global_path, current_state)
            reference_pose = reference_pose_selection(global_path, nearest_reference_id, ahead_distance, num_path)
            vehicle_speed = vehicle_pose_msg.current_speed

            # Adjust pure pursuit look ahead distance
            ahead_distance = look_ahead_distance_modification(vehicle_speed)
            
            # Calculate the required steering angle
            steer_control, a = pure_pursuit_control(reference_pose, current_state)
            speed_control = np.clip(current_state[3] + a, vel_min, vel_max)

            # Initiate ros message for publishing
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed_control
            drive_msg.drive.steering_angle = steer_control
            
            # Publish the goal and drive message
            drive_pub.publish(drive_msg)
        except IndexError:
            continue
        except RuntimeError:
            continue
        rate.sleep()
