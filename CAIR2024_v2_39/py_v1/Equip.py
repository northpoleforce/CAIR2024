#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import time
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from uarm.wrapper import SwiftAPI

"""
pressure test: move
"""


swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, cmd_pend_size=2, callback_thread_pool_size=1)

swift.waiting_ready()

device_info = swift.get_device_info()
print(device_info)
firmware_version = device_info['firmware_version']
if firmware_version and not firmware_version.startswith(('0.', '1.', '2.', '3.')):
    swift.set_speed_factor(0.00005)

swift.set_mode(0)

speed = 100000

swift.set_servo_angle(2,0)  # 机械臂上抬
time.sleep(2)
swift.set_servo_angle(1,100)  # 机械臂向前伸
time.sleep(2)
swift.set_servo_angle(0,90)  # 底座左右移动
time.sleep(3)

def Equip():
    #########  装货（狗前部的容器）BEGIN   ###########
    
    swift.set_wrist(80)  # 夹爪张开
    swift.set_servo_angle(0,20)  # 底座左右移动
    time.sleep(3)
    swift.set_servo_angle(1,30)  # 机械臂向前伸
    time.sleep(3)
    swift.set_servo_angle(2,58)  # 机械臂下降
    time.sleep(3)
    swift.set_wrist(180)         # 夹爪闭合
    time.sleep(1)
    swift.set_servo_angle(2,0)  # 机械臂上抬
    time.sleep(2)
    swift.set_servo_angle(1,100)  # 机械臂向前伸           ### Finish
    time.sleep(2)
    swift.set_servo_angle(0,90)  # 底座左右移动
    time.sleep(3)
    swift.set_servo_angle(1,115)  # 机械臂向后缩
    time.sleep(2)
    swift.set_servo_angle(2,18)  # 机械臂下降
    time.sleep(2)
    swift.set_wrist(80)          # 夹爪张开
    time.sleep(1)
    swift.set_servo_angle(2,0)  # 机械臂上抬
    time.sleep(2)
    swift.set_servo_angle(1,100)  # 机械臂向前伸
    time.sleep(2)
    swift.set_servo_angle(0,90)  # 底座左右移动
    time.sleep(3)

#########  装货（狗前部的容器）END   ###########


#########  装货（狗尾部的容器）BEGIN   ###########
    
    swift.set_wrist(80)  # 夹爪张开
    swift.set_servo_angle(0,160)  # 底座左右移动
    time.sleep(3)
    swift.set_servo_angle(1,30)  # 机械臂向前伸
    time.sleep(3)
    swift.set_servo_angle(2,58)  # 机械臂下降
    time.sleep(3)
    swift.set_wrist(180)         # 夹爪闭合
    time.sleep(1)
    swift.set_servo_angle(2,0)  # 机械臂上抬
    time.sleep(2)
    swift.set_servo_angle(1,100)  # 机械臂向前伸           ### Finish
    time.sleep(2)
    swift.set_servo_angle(0,90)  # 底座左右移动
    time.sleep(3)
    swift.set_servo_angle(1,90)  # 机械臂向前伸
    time.sleep(2)
    swift.set_servo_angle(1,60)  # 机械臂向前伸
    time.sleep(2)
    swift.set_servo_angle(2,18)  # 机械臂下降
    time.sleep(2)
    swift.set_wrist(80)          # 夹爪张开
    time.sleep(1)
    swift.set_servo_angle(2,0)  # 机械臂上抬
    time.sleep(2)
    swift.set_servo_angle(1,90)  # 机械臂向前伸
    time.sleep(2)
    swift.set_servo_angle(0,90)  # 底座左右移动
    time.sleep(3)
#########  装货（狗尾部的容器）END   ###########

while swift.connected:
    
    #### 0 模式后续参数越小底盘转动角度越大   
    #### 1 模式后续参数越小机械臂往前伸越大
    #### 2 模式后续参数越小机械臂下降程度越小
    #### 夹爪开合程度：swift.set_wrist(参数)，参数越大则夹爪开合越小   
    #### 以上参数范围都是[0，180]
    Equip()
    swift.disconnect()
    break



