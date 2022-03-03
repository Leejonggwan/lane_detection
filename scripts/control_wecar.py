#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import rospy
import cv2
import numpy as np
from time import sleep
from std_msgs.msg import Float64, Float32, Bool, String
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
import math

class move_wecar:
    def __init__(self):
        rospy.init_node('control_wecar', anonymous=True)
        self.steer = rospy.Subscriber(
            "/wecar/steer",
            Float32,
            self.run
        )
        self.ackermann_pub = rospy.Publisher(
            '/vesc/high_level/ackermann_cmd_mux/input/nav_1',
            AckermannDriveStamped,
            queue_size=5
        )

        self.sub_dflag = rospy.Subscriber(
            '/traffic_flag',
            String,
            callback=self.flag_change
        )
	
        self.sub_lflag = rospy.Subscriber(
            '/lidar_flag',
            Bool,
            callback=self.e_stop
        )

        self.c_steer = 0
        self.c_speed = 0

        self.t_flag = True
        self.l_flag = True

    def run(self, data):
        steer = data.data
        if steer == -99:
            steer = 0
        if steer == 0:
            position_value = 0.00
            speed_value = 0.6
        else:
            position_value = steer / 180 * math.pi * 1.0
            # position_value = steer
            speed_value = 0.60
        if not self.t_flag or not self.l_flag:
            #self.cycle = 1
            print("t_flag = {0}".format(self.t_flag))
            print("l_flag = {0}".format(self.l_flag))
            print("Stop!")
            sleep(2)
            position_value = 0.0
            speed_value = 0.0
	
        self.c_speed = speed_value
        self.c_steer = position_value

        self.ackermann_pub.publish(self.make_msg())

    def e_stop(self, _flag):
        self.l_flag = _flag.data

    def flag_change(self, _b):
        if _b.data == "G":
            self.t_flag = True
        elif _b.data == "Y":
            self.t_flag = False
        elif _b.data == "R":
            self.t_flag = False
        else :
            self.t_flag = True

    def make_msg(self):
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        drive_msg.speed = self.c_speed
        drive_msg.steering_angle = self.c_steer * 1.0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
	
        return drive_msg_stamped


if __name__ == '__main__':
    MoveCar = move_wecar()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")
