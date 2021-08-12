#!/usr/bin/env python3

import math
import csv
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def matrix_loading(f):
    f = f.readlines()
    mat = []
    for row in f:
        line = []
        row = row[:-2].split(',')
        for col in row:
            line.append(float(col))
        mat.append(line)
    return mat

def yaw_change_correction(delta_yaw):
    if delta_yaw > math.pi:
        delta_yaw = delta_yaw - 2*math.pi
    elif delta_yaw < -math.pi:
        delta_yaw = delta_yaw + 2*math.pi
    else:
        delta_yaw = delta_yaw
    return delta_yaw

def vehicle_coordinate_transformation(goal_pose, vehicle_pose):
    dx = goal_pose[0] - vehicle_pose[0]
    dy = goal_pose[1] - vehicle_pose[1]
    v_yaw = yaw_change_correction(goal_pose[2] - vehicle_pose[2])
    v_x = dx * math.cos(vehicle_pose[2]) + dy * math.sin(vehicle_pose[2])
    v_y = dy * math.cos(vehicle_pose[2]) - dx * math.sin(vehicle_pose[2])
    v_pose = np.array([0,0,0,vehicle_pose[3]])
    v_goal_pose = np.array([v_x, v_y, v_yaw, goal_pose[3]])
    return v_pose, v_goal_pose

def goal_reach_check(goal_pose, current_pose):
    distance = ((goal_pose[0] - current_pose[0])**2 + (goal_pose[1] - current_pose[1])**2)**0.5
    if distance > 0.5:
        return False
    else:
        return True

def global_path_reader(global_path_name):
    with open(global_path_name) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1]), float(point[2]), float(point[3])) for point in path_points]
    path_points_pos_x = [float(point[0]) for point in path_points]
    path_points_pos_y = [float(point[1]) for point in path_points]
    path_point_pos_yaw = [float(point[2]) for point in path_points]
    path_point_pos_velocity = [float(point[3]) for point in path_points]
    global_path = np.transpose(np.array([path_points_pos_x, path_points_pos_y, path_point_pos_yaw, path_point_pos_velocity]))
    return global_path

def nearest_reference_pose(global_path, vehicle_pose):
    d_pose = np.array(global_path[:,0:2]) - np.array(vehicle_pose[0:2])
    distance_array = np.linalg.norm(d_pose, axis=1)
    nearest_pose_id = np.argmin(distance_array)
    return nearest_pose_id

def goal_pose_message_converter(goal_pose):
    frame_id = 'map'
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    x = goal_pose[0]
    y = goal_pose[1]
    yaw = goal_pose[2]
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    return msg

def initial_pose_message_converter(initial_pose):
    frame_id = 'map'
    sequence = 0
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    x = initial_pose[0]
    y = initial_pose[1]
    yaw = initial_pose[2]
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]
    return msg