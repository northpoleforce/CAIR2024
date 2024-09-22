#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import time
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

#z:控制吸盘上下
#x:控制吸盘前后
#y:控制机械臂底盘转动

swift.reset(speed=speed)
while swift.connected:

    ########  定义狗身上有GO1字样丝印的一侧为右侧      ############
    ########  定义狗身上有Unitree字样丝印的一侧为左侧  ############
    ########  (y=30)则为右格子，(y=-30)则为左格子    ############
    ########  (y=290)则为狗右侧，(y=-290)则为狗左侧  ############
    ########  在(y=±150)->(y=±290)之间做 time.sleep(2) 是为了防止刮蹭到物品格子  ##########
    ########  一、二号物品往狗左边扔  三、四号物品往狗右边扔

    #######  抽到一号和二号物品，左格子放一号，右格子放二号，则需要： 左格子->狗左侧 和 右格子->狗左侧  ######
    #######  抽到一号和三号物品，左格子放一号，右格子放三号，则需要： 左格子->狗左侧 和 右格子->狗右侧  ######
    #######  抽到一号和四号物品，左格子放一号，右格子放四号，则需要： 左格子->狗左侧 和 右格子->狗右侧  ######
    #######  抽到二号和三号物品，左格子放二号，右格子放三号，则需要： 左格子->狗左侧 和 右格子->狗右侧  ######
    #######  抽到二号和四号物品，左格子放二号，右格子放四号，则需要： 左格子->狗左侧 和 右格子->狗右侧  ######
    #######  抽到三号和四号物品，左格子放三号，右格子放四号，则需要： 左格子->狗右侧 和 右格子->狗右侧  ######

    for i in range(2):
        #######左格子---狗左侧########
        swift.set_pump(on=True)  # 打开气泵
        swift.set_position(y=-30)  # 转到物品方格
        swift.set_position(z=0)  # 吸盘向下移动
        swift.set_position(z=150)  # 吸盘向上移动
        swift.set_position(y=-150)  # 转到侧面
        time.sleep(2)
        swift.set_position(y=-290)  # 转到侧面
        swift.set_position(z=-30)  # 吸盘向下移动
        time.sleep(2)
        swift.set_pump(on=False)  # 关闭气泵
        swift.reset(speed=speed)  # 复位
    break
    
while swift.connected:
    swift.disconnect()