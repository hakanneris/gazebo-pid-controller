#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 30 11:57:58 2023

@author: hakan
"""
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from angles import shortest_angular_distance
from tf.transformations import euler_from_quaternion as efq


class PIDController:
    def __init__(self, kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_err=0
    
    def update(self, ang_des, ang_act, dt):
        err = ang_act - ang_des
        d_err = (err-self.prev_err)/dt
        i_err = err + err*dt
        self.prev_err = err
        
        u = self.kp*err + self.ki*i_err + self.kd*d_err
        
        return u
    
class ControlNode:
    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.controller = PIDController(1.3,0.0001,0.1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
    def imu_callback(self, data):
        print("Ready")
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = efq(orientation_list)
        
        diff = shortest_angular_distance(yaw, 0.0)
        
        tw = Twist()
        tw.angular.z = self.controller.update(0.0, diff, 0.001)
        self.cmd_pub.publish(tw)

if __name__ == "__main__":
    rospy.init_node("pid_node")
    a = ControlNode()
    rospy.spin()