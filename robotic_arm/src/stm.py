#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String,Float32
import serial
import time
import threading


class Port():
    def __init__(self):
        rospy.init_node("stm_node")
        self.sub = rospy.Subscriber("roboticarm", String, self.callback, queue_size=5)
        self.pub = rospy.Publisher("ultrasound", Float32, queue_size=1)

        # 串口
        self.ser = serial.Serial("/dev/ttyUSB3", 115200)
        self.ser.flushInput()

        self.timer1 = rospy.Timer(rospy.Duration((1.0)/10),self.time1callback)
    
    def time1callback(self):
        data_dis = Float32()
        data_dis.data = self.uart_receive_str()
        self.pub.publish(data_dis)

    
    def callback(self,data):
        self.uart_send_str(msg.data)
        

    #发送字符串 只需传入要发送的字符串即可
    def uart_send_str(self,string):
        self.ser.write(string.encode("utf-8"))
        time.sleep(0.01)
        self.ser.flushInput()

    def uart_receive_str(self):
        self.ser.flushInput()
        receive_buf = self.ser.read(10).decode()
        dis = float(receive_buf)
        return dis



if __name__ == "__main__":
    port = Port()
    rospy.spin()

