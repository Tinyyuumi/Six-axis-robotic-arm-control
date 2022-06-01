#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time,math
import z_kinematics
import rospy
from std_msgs.msg import String,Int8,Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import darknet_images

class Hand():
    def __init__(self):
        rospy.init_node("arm_node")
        self.pub = rospy.Publisher("roboticarm", String, queue_size=1)
        rospy.Subscriber('robot/arm/move1',Int8, self.callback1, queue_size=1)

        self.testStr = String()
        self.pwm = Int16MultiArray()
        self.bridge = CvBridge()


        self.red_lower = np.array([156,43,46])
        self.red_upper = np.array([180,255,255]) #设置红色区间

        self.green_lower = np.array([37,43,46])
        self.green_upper = np.array([77,255,255]) #设置绿色区间

        self.color_lower = self.green_lower
        self.color_upper = self.green_upper

        self.color_lower2 = self.green_lower
        self.color_upper2 = self.green_upper

        self.servo_pwm = [1500,1800,2000,2200,1500,1200]

        self.x_mid = 160
        self.y_mid = 160

        self.x_pre = 0
        self.y_pre = 0
        self.num = 0

        self.yolo = darknet_images.Yolo()

    def callback1(self,data):
        if data.data == 1:
            # back = Int8()
            # back.data = 1
            self.mystring.data = "#150018002000220015001200"
            self.pub1.publish(self.mystring)
            time.sleep(2)

            back.data = self.moving_apple() # 抓苹果
            # self.pub_back1.publish(back)

            servo_angle = [0,0,0,0]
            # servo_pwm = rospy.wait_for_message("robot/arm/move1/returm2",Int16MultiArray)
            servo_angle[0] = math.radians((270.0/2000.0)*(1500-self.servo_pwm[0]))
            servo_angle[1] = math.radians((270.0/2000.0)*(self.servo_pwm[1]-1500))
            servo_angle[2] = math.radians((270.0/2000.0)*(self.servo_pwm[2]-1500))
            servo_angle[3] = math.radians((270.0/2000.0)*(1500-self.servo_pwm[3]))

            delta_theta = -1*servo_angle[1]-1*servo_angle[3]+servo_angle[2]-1.57
            # S = (dis+l3)*math.cos(delta_theta) + l1*math.sin(-1*servo_angle[1])+l2*math.sin(servo_angle[2]-servo_angle[1])
            z_ = l0 + l1*math.cos(-1*servo_angle[1])+l2*math.cos(servo_angle[2]-servo_angle[1]) - l3*math.sin(delta_theta)+h
            S = z_/math.tan(delta_theta) + l3*math.cos(delta_theta)+ l1*math.sin(-1*servo_angle[1])+l2*math.sin(servo_angle[2]-servo_angle[1])

            x_ = S *math.sin(servo_angle[0])
            y_ = S *math.cos(servo_angle[0])
            # print([x_,y_,z_])
            self.mystring.data= z_kinematics.kinematics_move(x_,y_,z_,1000) #运用 单位mm

            if isinstance(self.mystring.data,numbers.Number):
                self.mystring.data="#%04d10001600160015001200"%self.servo_pwm[0]
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
            
    def Tracing(self,x, y):
        if self.x_mid - 5 < int(x) < self.x_mid + 5:
                pass
        elif(int(x) > self.x_mid+10):
            if(int(x)-self.x_mid > 90):
                self.servo_pwm[0] = self.servo_pwm[0]-10
            elif(int(x)-self.x_mid > 60):
                self.servo_pwm[0] = self.servo_pwm[0]-5
            elif(int(x)-self.x_mid > 10):
                self.servo_pwm[0] = self.servo_pwm[0]-0.5
        elif(int(x) < self.x_mid-10):
            if(int(x)-self.x_mid < -90):
                self.servo_pwm[0] = self.servo_pwm[0]+10
            elif(int(x)-self.x_mid < -60):
                self.servo_pwm[0] = self.servo_pwm[0]+5
            elif(int(x)-self.x_mid < -10):
                self.servo_pwm[0] = self.servo_pwm[0]+0.5

        if self.y_mid - 20 < int(y) < self.y_mid + 20:
                pass
        elif(int(y) > self.y_mid+10):
            if(int(y)-self.y_mid > 90):
                self.servo_pwm[3] = self.servo_pwm[3]+10
            elif(int(y)-self.y_mid > 60):
                self.servo_pwm[3] = self.servo_pwm[3]+5
            elif(int(y)-self.y_mid > 10):
                self.servo_pwm[3] = self.servo_pwm[3]+1
        elif(int(y) < self.y_mid-10):
            if(int(y)-self.y_mid < -90):
                self.servo_pwm[3] = self.servo_pwm[3]-10
            elif(int(y)-self.y_mid < -60):
                self.servo_pwm[3] = self.servo_pwm[3]-5
            elif(int(y)-self.y_mid < -10):
                self.servo_pwm[3] = self.servo_pwm[3]-1

    def moving_apple(self):
        while True:
            data = rospy.wait_for_message("image_raw", Image)
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite("img2.jpg",frame)

            # 运行yolo得到坐标
            # fruit_list,x,y = darknet_images.main("img.png")
            x,y = self.yolo.control("img2.jpg")
            
            self.Tracing(x,y)
            self.testStr.data = "#%04d%04d%04d%04d%04d%04d" % (self.servo_pwm[0],self.servo_pwm[1],self.servo_pwm[2],self.servo_pwm[3],self.servo_pwm[4],self.servo_pwm[5])
            self.pub.publish(self.testStr)
            # self.myUart.uart_send_str(testStr)

            time.sleep(0.2)
            if abs(self.x_pre-x)<3 and abs(self.y_pre-y)<3:
                self.num = self.num + 1
            else:
                self.num = 0

            if self.num >= 20:
                break

            self.x_pre = x
            self.y_pre = y

    
if __name__ == '__main__':
    robotHand = Hand()
    rospy.spin()
    # robotHand.myUart.uart_send_str("#150018002000220015001200")
    # time.sleep(2)
    # robotHand.moving()