#!/usr/bin/env python3  

import rospy
import os
import paramiko
from parameters import DriverParameters

rospy.init_node("autoSSH")
robotID = rospy.get_param('ID', 4)
# print(f"Robot ID: {robotID}")
robotIP = DriverParameters.REAL_ROBOT_IP + "1" + f"{robotID:02d}"

ssh = paramiko.SSHClient()

ssh.load_system_host_keys()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

ssh.connect(robotIP, username="bingda", password="bingda")
# ssh.exec_command("export ROS_PACKAGE_PATH:=/home/bingda/catkin_ws/src:/opt/ros/melodic/share:/home/bingda/td_ws/src")
ssh.exec_command(
    "export ROS_MASTER_URI=http://" + DriverParameters.ROS_MASTER_IP + ":11311 && "
    "export ROS_IP=" + robotIP + " && "
    "roslaunch base_control base_startup.launch"
)

rospy.spin()

ssh.exec_command("rosnode kill --all")
ssh.exec_command("killall -9 base_control.py")
ssh.close()