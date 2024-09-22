#!/bin/bash

# 定义主机B的地址和用户名
HOST='192.168.123.13'
USER='unitree'
# 使用ssh连接到主机B并执行命令
ssh -t ${USER}@${HOST} 'bash -ic "source ~/.bashrc; rc2_123"'