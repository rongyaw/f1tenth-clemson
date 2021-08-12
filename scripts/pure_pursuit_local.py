#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry, Path
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
vel_max = 5.0 # Set the maximum velocity
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
        self.path = []
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.local_path_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.path_callback)
    
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

    def path_callback(self, msg):
        path = msg.poses
        for state in path:
            x = state.pose.position.x
            y = state.pose.position.y
            qx = state.pose.orientation.x
            qy = state.pose.orientation.y
            qz = state.pose.orientation.z
            qw = state.pose.orientation.w
            rotation_quaternion = (qx, qy, qz, qw)
            rotation_euler = euler_from_quaternion(rotation_quaternion)
            yaw = rotation_euler[2]
            self.path.append([float(x), float(y), float(yaw)])

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.current_speed])
        return vehicle_state
    
    def path_info_output(self):
        local_path = np.array(self.path)
        return local_path

def pure_pursuit_control(goal_state, current_state):
    v_pose, v_goal_pose = vehicle_coordinate_transformation(goal_state, current_state)

    # Calculate steering angle
    d_yaw = math.atan(v_goal_pose[1]/v_goal_pose[0])
    L = ((v_goal_pose[0])**2 + (v_goal_pose[1])**2)**0.5
    angle = math.atan(2 * WB * math.sin(d_yaw) / L)

    # Apply steering angle constraints
    angle = np.clip(angle, -max_steer,  max_steer)
    return angle

def reference_pose_selection(local_path, current_pose, ahead_distance):
    distance = 0.0
    i = 0
    while True:
        goal_pose = local_path[i,:]
        v_pose, v_goal_pose = vehicle_coordinate_transformation(goal_pose, current_pose)
        distance = (v_goal_pose[0]**2 + v_goal_pose[1]**2)**0.5
        if distance > ahead_distance and v_goal_pose[0] > 0:
            break
        else:
            i += 1
    return local_path[i,:]

def look_ahead_distance_modification(v):
    v = np.clip(v, vel_min, vel_max)
    ahead_distance_max = 1.5
    ahead_distance_min = 0.5
    k = (ahead_distance_max - ahead_distance_min)/(vel_max - vel_min)
    ahead_distance = k * (v - vel_min) + ahead_distance_min
    return ahead_distance

if __name__ == "__main__":
    rospy.init_node('pure_pursuit_controller', anonymous=True)
    dirname = os.path.dirname(__file__)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    # Initiate ros message listener and initial vehicle conditions
    rate = rospy.Rate(10)
    vehicle_pose_msg = ros_message_listener()
    ahead_distance = 0.5

    while not rospy.is_shutdown():
        try:
            current_state = vehicle_pose_msg.vehicle_state_output()
            path = vehicle_pose_msg.path_info_output()
            reference_pose = reference_pose_selection(path, current_state, ahead_distance)
            vehicle_pose_msg.path = []
            vehicle_speed = vehicle_pose_msg.current_speed

            # Adjust pure pursuit look ahead distance
            # ahead_distance = look_ahead_distance_modification(vehicle_speed)
            
            # Calculate the required steering angle
            steer_control = pure_pursuit_control(reference_pose, current_state)
            speed_control = 1.0 #reference_pose[3]
            
            # Initiate ros message for publishing
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed_control
            drive_msg.drive.steering_angle = steer_control
            
            # Publish the goal and drive message
            # drive_pub.publish(drive_msg)

            # Record the vehicle state
            # with open(dirname + '/path_curvature_100_lap_4.csv', 'a') as f:
            #     f.write(str(current_state[0]) + ',' + str(current_state[1]) + ',' + str(current_state[2]) + ',' + str(4.0) + '\n')
        except IndexError:
            continue
        except RuntimeError:
            continue
        rate.sleep()
