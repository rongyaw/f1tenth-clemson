#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

import math
import numpy as np
from numpy import linalg as LA
from utils import yaw_change_correction, vehicle_coordinate_transformation, goal_reach_check, global_path_reader, nearest_reference_pose, matrix_loading
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
from utils import matrix_loading
import time

import casadi

N_o = 12
N_u = 2
vel_max = 7.0
vel_min = 1.0
max_steer = 0.4189 # Set the maximum steering angle
eps_speed = 0.5 # Set slack variable for speed control input
eps_steer = 0.05 # Set slack variable for steering control input

class Model(nn.Module):
    def __init__(self, in_features = 4, h1 = 512, h2 = 512, h3 = 512, out_features = N_o):
        super().__init__()
        self.fc1 = nn.Linear(in_features, h1)
        self.fc2 = nn.Linear(h1, h2)
        self.out = nn.Linear(h2, out_features)

    def forward(self, x):
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x = self.out(x)
        return x

class ros_message_listener:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.vesc_odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
    
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
        self.yaw = yaw
        self.vel = vel

    def vehicle_state_output(self):
        vehicle_state = np.array([self.x, self.y, self.yaw, self.vel])
        return vehicle_state

def koopman_mpc_control(zt, zt_ref, N, A, B, C):
    opti = casadi.Opti()

    z = opti.variable(N_o, N+1)
    u = opti.variable(N_u, N)
    zt_ref = casadi.MX(zt_ref.T)
    zt = casadi.MX(zt.T)

    A = casadi.MX(A)
    B = casadi.MX(B)
    C = casadi.MX(C)

    Q = casadi.diag([1.0, 1.0, 1.0, 1.0])
    R = casadi.diag([0.001, 0.001])
    Q_final = Q
    Q_lift = casadi.mtimes(casadi.mtimes(C.T, Q), C)
    Q_final_lift = casadi.mtimes(casadi.mtimes(C.T, Q_final), C)

    cost = 0
    constraint = []

    for t in range(N-1):
        cost += casadi.mtimes(casadi.mtimes(u[:, t].T, R),u[:, t])
        if t != 0:
            cost += casadi.mtimes(casadi.mtimes((zt_ref[:, t].T - z[:, t].T), Q_lift), (zt_ref[:, t] - z[:, t]))
        opti.subject_to(z[:, t+1] == casadi.mtimes(A, z[:, t]) + casadi.mtimes(B, u[:, t]))
        if t < N-2:
            opti.subject_to(u[1, t+1] - u[1, t] >= -0.05)
            opti.subject_to(u[1, t+1] - u[1, t] <= 0.05)
            opti.subject_to(u[0, t+1] - u[0, t] <= 0.1)
            opti.subject_to(u[0, t+1] - u[0, t] >= -0.1)

    opti.subject_to(z[:, 0] == zt)
    opti.subject_to(u[1, :] <= max_steer)
    opti.subject_to(u[1, :] >= -max_steer)
    opti.subject_to(u[0, :] <= 6.0)
    opti.subject_to(u[0, :] >= -8.0)
    
    opti.minimize(cost)
    
    opti.solver('ipopt',{"print_time": False},{"print_level": 0})
    sol = opti.solve()

    steering = sol.value(u[1,:])
    acceleration = sol.value(u[0,:])

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
    # Load the neural network and A, B, C matrix
    koopman_operator = Model()
    dirname = os.path.dirname(__file__)
    koopman_operator.load_state_dict(torch.load(dirname + '/koopman_lifting_function.pt'))
    koopman_operator.eval()
    interval_distance = 0.2
    N = 5
    
    # Load the learned state-space dynamics
    A_f = open(dirname + '/A_mat', 'r')
    B_f = open(dirname + '/B_mat', 'r')
    C_f = open(dirname + '/C_mat', 'r')
    A = matrix_loading(A_f)
    B = matrix_loading(B_f)
    C = matrix_loading(C_f)
    A_f.close()
    B_f.close()
    C_f.close()

    # Listen to the message from planner path
    rospy.init_node('koopman_mpc')
    drive_pub = rospy.Publisher("car/mpc_control_signal", AckermannDriveStamped, queue_size=1)
    
    global_trajectory = rospy.get_param("/racing_line")
    filename = os.path.join(dirname, global_trajectory)
    global_path = global_path_reader(filename)
    goal_pub = rospy.Publisher('/local_goal', PoseArray, queue_size=1)
    num_path = len(global_path[:,0])

    rate = rospy.Rate(100)
    path_msg = ros_message_listener()
    
    # Initiate MPC ros node
    while not rospy.is_shutdown():
        try:
            xt = path_msg.vehicle_state_output()
            nearest_reference_id = nearest_reference_pose(global_path, xt)
            xt_ref = reference_pose_selection(global_path, nearest_reference_id, interval_distance, N, num_path)
            
            # Perform vehicle coordinate transformation
            x_ref = np.zeros((N, 4))
            for i in range(N):
                x, ref = vehicle_coordinate_transformation(xt_ref[i,:], xt)
                x_ref[i,:] = ref
            
            local_goal_message = goal_pose_publisher(xt_ref, N)
            goal_pub.publish(local_goal_message)
            x = torch.FloatTensor(x)
            x_ref = torch.FloatTensor(x_ref)
            zt = koopman_operator.forward(x)
            zt_ref = koopman_operator.forward(x_ref)
            
            # Apply Model Predictive Control
            acceleration, steering = koopman_mpc_control(np.array(zt.tolist()), np.array(zt_ref.tolist()), N, np.array(A), np.array(B), np.array(C))
            
            # Transform control message from convex optimization to low-level controller
            speed_control = np.clip((xt.tolist()[3] + acceleration[0]), vel_min + eps_speed, vel_max - eps_speed)
            steer_control = np.clip(steering[0], -max_steer, max_steer)
            
            # Publish Drive Message
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed_control + 1.0
            drive_msg.drive.steering_angle = steer_control
            drive_pub.publish(drive_msg)
        except IndexError:
            continue
        except RuntimeError:
            continue
        rate.sleep()
