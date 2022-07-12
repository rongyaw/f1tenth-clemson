#!/usr/bin/env python3

'''
Topic: Kinematic Nonlinear Model Predictive Controller for F1tenth simulator
Author: Rongyao Wang
Instiution: Clemson University Mechanical Engineering
'''

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

import math
from utils import yaw_change_correction, vehicle_coordinate_transformation, goal_reach_check, global_path_reader, nearest_reference_pose
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import casadi
from timeit import default_timer as timer
import csv
import os

# Define necessary vehicle parameters
WB = 0.25 # Wheelbase of the vehicle
vel_max = 7.0 # Set the maximum velocity
vel_min = 1.5 # Set the minimum velocity
max_steer = 0.4189 # Set the maximum steering angle
N_x = 4 # Number of state for MPC design
N_u = 2 # Number of control for MPC design
dt = 0.1 # Time interval

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vel = msg.twist.twist.linear.x
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.x = x
        self.y = y
        self.vel = vel
        self.yaw = yaw

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.vel])
        return vehicle_state

def nonlinear_kinematic_mpc_solver(x_ref, x_0, N):
    opti = casadi.Opti()

    x = opti.variable(N_x, N+1)
    u = opti.variable(N_u, N)
    x_ref = casadi.MX(x_ref)
    x_0 = casadi.MX(x_0)

    Q = casadi.diag([10.0, 10.0, 1.0, 1.0])
    R = casadi.diag([0.001, 0.001])
    cost = 0

    for t in range(N-1):
        cost += u[:, t].T @ R @ u[:, t]
        if t != 0:
            cost += (x_ref[:, t].T - x[:, t].T) @ Q @ (x_ref[:, t] - x[:, t])
        opti.subject_to(x[0, t + 1] == x[0, t] + x[3, t]*casadi.cos(x[2, t])*dt)
        opti.subject_to(x[1, t + 1] == x[1, t] + x[3, t]*casadi.sin(x[2, t])*dt)
        opti.subject_to(x[2, t + 1] == x[2, t] + x[3, t]*casadi.tan(u[1, t])/WB*dt)
        opti.subject_to(x[3, t + 1] == x[3, t] + u[0, t]*dt)

        if t < N-2:
            opti.subject_to(u[1, t+1] - u[1, t] >= -0.06)
            opti.subject_to(u[1, t+1] - u[1, t] <= 0.06)
            opti.subject_to(u[0, t+1] - u[0, t] <= 0.1)
            opti.subject_to(u[0, t+1] - u[0, t] >= -0.1)

    opti.subject_to(x[:, 0] == x_0)
    opti.subject_to(u[1, :] <= max_steer)
    opti.subject_to(u[1, :] >= -max_steer)
    opti.subject_to(u[0, :] <= 6.0)
    opti.subject_to(u[0, :] >= -8.0)

    opti.minimize(cost)
    opti.solver('ipopt',{"print_time": False}, {"print_level": 0})#, {"acceptable_tol": 0.0001}
    sol = opti.solve()

    acceleration = sol.value(u[0,0])
    steering = sol.value(u[1,0])

    return acceleration, steering

def reference_pose_selection(global_path, reference_pose_id, distance_interval, N, num_path):
    distance = 0.0
    i = reference_pose_id + 1
    reference_pose = np.zeros((N, 4))
    for k in range(N):
        while distance < distance_interval:
            if i < num_path:
                distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
                i += 1
            else:
                i -= num_path
                distance = ((global_path[i,0] - global_path[reference_pose_id,0])**2 + (global_path[i,1] - global_path[reference_pose_id,1])**2)**0.5
                i += 1
        reference_pose[k, :] = np.array([global_path[i,0], global_path[i,1], global_path[i,2], global_path[i,3]])
        reference_pose_id = i
        distance = 0.0
    return reference_pose

def goal_pose_publisher(goal_pose, N):
    frame_id = 'map'
    msg = PoseArray()
    msg.header.frame_id = frame_id
    for i in range(N):
        x = goal_pose[i,0]
        y = goal_pose[i,1]
        yaw = goal_pose[i,2]
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        msg.poses.append(pose)
    return msg

if __name__ == "__main__":
    rospy.init_node('nonlinear_kinematic_mpc_controller')
    drive_pub = rospy.Publisher('car/mpc_control_signal', AckermannDriveStamped, queue_size=1)
    goal_pub = rospy.Publisher('/local_goal', PoseArray, queue_size=1)

    dirname = os.path.dirname(__file__)
    global_trajectory = rospy.get_param("/racing_line")
    filename = os.path.join(dirname, global_trajectory)
    global_path = global_path_reader(filename)
    num_path = len(global_path[:,0])
    
    rate = rospy.Rate(100)
    vehicle_pose_msg = ros_message_listener()
    interval_distance = 0.2
    N = 5

    while not rospy.is_shutdown():
        try:
            current_state = vehicle_pose_msg.vehicle_state_output()
            nearest_reference_id = nearest_reference_pose(global_path, current_state)
            reference_pose = reference_pose_selection(global_path, nearest_reference_id, interval_distance, N, num_path)
            
            # Transform the reference pose to vehicle coordinate
            x_ref = np.zeros((N, 4))
            for i in range(N):
                x, ref = vehicle_coordinate_transformation(reference_pose[i,:], current_state)
                x_ref[i, :] = ref

            # Publish reference pose from the global trajectory
            local_goal_message = goal_pose_publisher(reference_pose, N)
            goal_pub.publish(local_goal_message)
            
            # Compute Control Output from Nonlinear Model Predictive Control
            acceleration, steering = nonlinear_kinematic_mpc_solver(x_ref.T, x.T, N)
            speed = np.clip(current_state[3] + acceleration, vel_min, vel_max)
            
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed + 1.0
            drive_msg.drive.steering_angle = steering
            drive_pub.publish(drive_msg)
        except IndexError:
            continue
        except RuntimeError:
            continue
        rate.sleep()
