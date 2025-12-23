#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from marmot.msg import Reset, Ack
import tf
from std_msgs.msg import Int8
from hybridController import MAPFRobot, TrafficRobot
from parameters import *

rospy.init_node('roboNode')
rospy.sleep(1)


RESOLUTION = DriverParameters.RESOLUTION

listOfRobotIDs = rospy.get_param("list_of_robots")

if(DriverParameters.SCENARIO == 0):
    MAPF_VirtualRobotList = [(MAPFRobot(i, "robot"+str(i), rospy.get_param("/StartX/robot"+str(i)), rospy.get_param("/StartY/robot"+str(i)))) for i in listOfRobotIDs]
else:
    Traffic_VirtualRobots = [(TrafficRobot(i, "robot"+str(i))) for i in listOfRobotIDs]

if __name__ == '__main__':

    while not rospy.is_shutdown():
        if(DriverParameters.SCENARIO == 0):
            for i in MAPF_VirtualRobotList:
                # print("Robot: "+str(i.virtualID)+" Running")
                i.goalTF.update()
                i.helper()
                i.publishDoneDependentTasks()
                i.move()
                
                i.updateDoneTasks()
            
        else:
            for i in Traffic_VirtualRobots:
                # if not i.done:
                i.helper()
                i.move()
                    
                i.updateDoneTasks()            
        rospy.sleep(0.005)


## TODO: MAybe Fix later?

# @ray.remote
# def run(robot):
#     while not rospy.is_shutdown():
#         robot.goalTF.update()
#         robot.helper()
#         robot.publishDoneDependentTasks()
#         robot.move()
#         robot.updateDoneTasks()
        
#         rospy.sleep(0.05)

# if __name__ == '__main__':

#     rospy.init_node('roboNode')
#     rospy.sleep(1)

#     ray.get([run.remote(i) for i in MAPF_VirtualRobotList])