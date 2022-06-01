#!/usr/bin/env python
# -*- coding: utf-8 -*-
import z_kinematics
import time,math
import numpy as np
import numbers
import rospy
from std_msgs.msg import String,Int8,Int16MultiArray

class Arm_Put():
    def __init__(self):
        self.l0 = 100    # 底盘到第二个舵机中心轴的距离10cm
        self.l1 = 90   # 第二个舵机到第三 个舵机的距离9cm
        self.l2 = 75   # 第三个舵机到第四 个舵机的距离7.5cm
        self.l3 = 150   # 第四个舵机到机械臂(闭合后)最高点的距离20cm
        self.h = 20

        self.pub1 = rospy.Publisher('robot/arm/move1',Int8, queue_size=1)
        self.pub2 = rospy.Publisher('robot/arm/move2_red',Int8, queue_size=1)
        self.pub3 = rospy.Publisher('robot/arm/move2_green',Int8, queue_size=1)

        self.pub_to_stm = rospy.Publisher("roboticarm", String, queue_size=5)

        rospy.Subscriber('robot/arm/start1',Int8, self.callback1, queue_size=1)
        rospy.Subscriber('robot/arm/start2_red',Int8, self.callback2, queue_size=1)
        rospy.Subscriber('robot/arm/start2_green',Int8, self.callback3, queue_size=1)

        self.mystring = String()

    def callback1(self):
        self.mystring.data = "#150018002000220015001200"
        self.pub_to_stm.publish(self.mystring)
        time.sleep(2)
        self.pub1.publish(1)
        self.pub1.publish(self.mystring)
        time.sleep(0.5)
        self.pub2.publish(1)
        pwm0 = rospy.wait_for_message("robot/arm/move1/returm",Int8)
        servo_angle = [0,0,0,0]
        servo_pwm = rospy.wait_for_message("robot/arm/move1/returm2",Int16MultiArray)
        servo_angle[0] = math.radians((270.0/2000.0)*(1500-servo_pwm[0]))
        servo_angle[1] = math.radians((270.0/2000.0)*(servo_pwm[1]-1500))
        servo_angle[2] = math.radians((270.0/2000.0)*(servo_pwm[2]-1500))
        servo_angle[3] = math.radians((270.0/2000.0)*(1500-servo_pwm[3]))

        delta_theta = -1*servo_angle[1]-1*servo_angle[3]+servo_angle[2]-1.57
        # S = (dis+l3)*math.cos(delta_theta) + l1*math.sin(-1*servo_angle[1])+l2*math.sin(servo_angle[2]-servo_angle[1])
        z_ = l0 + l1*math.cos(-1*servo_angle[1])+l2*math.cos(servo_angle[2]-servo_angle[1]) - l3*math.sin(delta_theta)+h
        S = z_/math.tan(delta_theta) + l3*math.cos(delta_theta)+ l1*math.sin(-1*servo_angle[1])+l2*math.sin(servo_angle[2]-servo_angle[1])

        x_ = S *math.sin(servo_angle[0])
        y_ = S *math.cos(servo_angle[0])
        # print([x_,y_,z_])
        self.mystring.data= z_kinematics.kinematics_move(x_,y_,z_,1000) #运用 单位mm

        if isinstance(self.mystring.data,numbers.Number):
            self.mystring.data="#%04d10001600160015001200"%pwm0
        self.pub1.publish(self.mystring)
        time.sleep(2)

        # 抓取
        self.mystring.data = self.mystring.data[0:-4]+"15001900"
        self.pub1.publish(self.mystring)

        time.sleep(2)

        # 归位
        self.mystring.data = "#150018001800230015001900"
        self.pub1.publish(self.mystring)
        time.sleep(2)



