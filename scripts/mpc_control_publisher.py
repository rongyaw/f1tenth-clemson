#!/usr/bin/env python3

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import os

max_acceleration = 6.0
max_steering_rate = 3.0
dt = 0.01

class message_listener():
    def __init__(self):
        self.steering = 0.0
        self.speed = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.ackermann_msg_sub = rospy.Subscriber('car/mpc_control_signal', AckermannDriveStamped, self.mpc_control_signal_callback, queue_size=1)
    
    def mpc_control_signal_callback(self, msg):
        self.steering = msg.drive.steering_angle
        self.speed = msg.drive.speed
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
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
        self.vel = msg.twist.twist.linear
    
    def vehicle_state_reader(self):
        return [self.x, self.y, self.yaw]

if __name__ == '__main__':
    rospy.init_node('mpc_control_publisher')
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    dirname = os.path.dirname(__file__)
    rate = rospy.Rate(1000)
    drive_message_sub = message_listener()

    speed = 0.0
    steering = 0.0

    while not rospy.is_shutdown():
        mpc_speed = drive_message_sub.speed
        mpc_steering = drive_message_sub.steering
        speed = np.clip(mpc_speed, speed - max_acceleration * dt, speed + max_acceleration * dt)
        steering = np.clip(mpc_steering, steering - max_steering_rate * dt, steering + max_steering_rate * dt)
        
        # Create control object
        ackerman_control_output = AckermannDriveStamped()
        
        # Uncomment the following section to record the vehicle control
        ackerman_control_output.drive.speed = speed
        ackerman_control_output.drive.steering_angle = steering

        # Publish control signal
        drive_pub.publish(ackerman_control_output)
        
        # Record the vehicle states
        # Uncomment the following section to record the vehicle state
        # with open(dirname + '/control_record.csv', 'a') as f:
        #     f.write(str(speed) + ',' + str(steering) + '\n')
        # s = drive_message_sub.vehicle_state_reader()
        # with open(dirname + '/path_20_curvature.csv', 'a') as f:
        #     f.write(str(s[0]) + ',' + str(s[1]) + ',' + str(s[2]) + '\n')
        
        rate.sleep()