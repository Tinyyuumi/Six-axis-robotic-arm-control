#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import *
# import z_beep as myBeep
# import z_uart as myUart
import time

pi = 3.1415926

l0 = 100    # 底盘到第二个舵机中心轴的距离10cm
l1 = 90   # 第二个舵机到第三 个舵机的距离9cm
l2 = 75   # 第三个舵机到第四 个舵机的距离7.5cm
l3 = 230   # 第四个舵机到机械臂(闭合后)最高点的距离20cm

servo_angle = [0,0,0,0]#记录角度
servo_pwm = [0,0,0,0]#记录pwm

'''
    设置四个关节的长度
    单位0.1mm
'''

def setup_kinematics(L0, L1, L2, L3) :
    global l0, l1 ,l2 ,l3
    #放大10倍
    l0 = L0*10;
    l1 = L1*10;
    l2 = L2*10;
    l3 = L3*10;

'''
    x,y 为映射到平面的坐标
    z为距离地面的距离
    Alpha 为爪子和平面的夹角 -25~-65范围比较好
'''
#x,y,z为爪子闭合时的末端点的坐标，alpha为爪子与水平面的角度
def kinematics_analysis(x, y, z, Alpha):
    global pi,l0, l1 ,l2 ,l3,servo_angle,servo_pwm
    #float theta3, theta4, theta5, theta6;
    #float l0, l1, l2, l3;
    #float aaa, bbb, ccc, zf_flag;
    
    #放大10倍
    x = x*10;
    y = y*10;
    z = z*10;
    
    #l0 = kinematics->L0;
    #l1 = kinematics->L1;
    #l2 = kinematics->L2;
    #l3 = kinematics->L3;
    
    if(x == 0 and y != 0) :
        theta6 = 0.0;
    elif(x > 0 and y == 0):
        theta6 = 90;
    elif(x < 0 and y == 0):
        theta6 = -90;
    else :
        theta6 = atan(x/y)*180.0/pi;#计算云台旋转角度
    
    
    y = sqrt(x*x + y*y);#x y 坐标的斜边 
    y = y-l3 * cos(Alpha*pi/180.0);  #求y1+y2
    z = z-l0-l3*sin(Alpha*pi/180.0); #求z1+z2
    
    if(z < -l0) :
        return 1;
    if(sqrt(y*y + z*z) > (l1+l2)) :
        return 2;
        
    ccc = acos(y / sqrt(y * y + z * z));
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z));
    if(bbb > 1 or bbb < -1) :
        return 3;
    
    if (z < 0) :
        zf_flag = -1;
    else :
        zf_flag = 1;
    
    theta5 = ccc * zf_flag + acos(bbb);     #计算1号舵机的弧度
    theta5 = theta5 * 180.0 / pi;           #转化为角度 
    if(theta5 > 180.0 or theta5 < 0.0) :
        return 4;
    
    
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if (aaa > 1 or aaa < -1) :
        return 5;
    
    theta4 = acos(aaa); #2号舵机的弧度
    theta4 = 180.0 - theta4 * 180.0 / pi ;  #转化为角度
    if (theta4 > 135.0 or theta4 < -135.0) :
        return 6;
    

    theta3 = Alpha - theta5 + theta4;   #计算3号舵机角度
    if(theta3 > 90.0 or theta3 < -90.0) :
        return 7;
    
    
    servo_angle[0] = theta6;
    servo_angle[1] = theta5-90;
    servo_angle[2] = theta4;
    servo_angle[3] = theta3;    
    
    servo_pwm[0] = (int)(1500-2000.0 * servo_angle[0] / 270.0);
    servo_pwm[1] = (int)(1500+2000.0 * servo_angle[1] / 270.0);
    servo_pwm[2] = (int)(1500+2000.0 * servo_angle[2] / 270.0);
    servo_pwm[3] = (int)(1500-2000.0 * servo_angle[3] / 270.0);
    
    return 0;

#此函数可移植到要执行的主程序里面去
def kinematics_move(x,y,z,mytime): 
    global servo_angle,servo_pwm
    setup_kinematics(100,90,75,200)#初始化
    if(y < 0):
        return 0;
    #寻找3号舵机的最佳角度
    flag = 0;
    # my_min = 0
    # for i in range(-135,0) :
    #     if(0 == kinematics_analysis(x,y,z,i)):
    #         if(i<my_min):
    #             my_min = i;
    #         flag = 1;

    my_max = 0
    for i in range(-135,0) :
        if(0 == kinematics_analysis(x,y,z,i)):
            if(i>my_max):
                my_max = i;
            flag = 1  
    #用3号舵机与水平最大的夹角作为最佳值
    if(flag) :
        kinematics_analysis(x,y,z,my_max);
        
        testStr = "#%04d%04d%04d%04d15001200" % (servo_pwm[0],servo_pwm[1],servo_pwm[2],servo_pwm[3])
        # print(testStr)
        # myUart.uart_send_str(testStr)
        return testStr;
    
    return 0;

#程序反复执行处
if __name__ == "__main__":
    # myBeep.setup_beep()
    # myUart.setup_uart(115200)
    # myUart.uart_send_str("#255P1500T1000!")

    setup_kinematics(100,90,75,200)#初始化
    a = kinematics_move(0,250,220,1000)#运用 单位mm
    try:
        while True:
            pass
    except KeyboardInterrupt:
        off()
