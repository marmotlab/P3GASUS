#!/usr/bin/env python3

import rospy
import numpy as np
from marmot.msg import Reset, Ack
from geometry_msgs.msg import Point
from unifiedCommsUtil import *
from parameters import *

HYBRID_ROBOT_COUNT = DriverParameters.HYBRID_ROBOT_COUNT
RESOLUTION = DriverParameters.RESOLUTION
WORLD = np.asarray(rospy.get_param("WORLD"))
ACK_TIMEOUT = 0.02

class GetRobotPosPubAck(PubAck):
    def __init__(self, topicName="/getRobotPos", MainType=Ack, AckType=Reset, ACK_TIMEOUT=0.2) -> None:
        super().__init__(topicName, MainTopic=MainType, AckTopic=AckType, ACK_TIMEOUT=ACK_TIMEOUT)
        self.allPos = None
        
    def publishData(self, data):
        self.allPos = np.zeros((len(data), 2))
        super().publishData(data)
        return self.allPos
    
    def ackCallback(self, data):
        self.ACK_STORE[data.robotID] = 1
        self.allPos[data.robotID][0] = data.goal.x
        self.allPos[data.robotID][1] = data.goal.y

if __name__=="__main__":
    rospy.init_node('newRun')
    rospy.sleep(1)
    
    robotPos = GetRobotPosPubAck().publishData([Ack(-1, i) for i in range(HYBRID_ROBOT_COUNT)])
    
    allFreeCells = np.swapaxes(np.where(WORLD==0), 0,1).tolist()
    allFreeCells = np.array([getCoord(i, RESOLUTION) for i in allFreeCells])
    STARTS = []
    
    for i in range(HYBRID_ROBOT_COUNT):
        ref_point = np.array(robotPos[i])
        # print(allFreeCells.shape, ref_point.shape)

        diff = allFreeCells - ref_point
        # print(diff.shape)
        # Calculate the Euclidean distance for each point
        distances = np.sqrt(np.sum(diff**2, axis=1))
        # print(distances.shape)
        STARTS.append(allFreeCells[np.argmin(distances)])
        # print(STARTS)
        
        allFreeCells = np.delete(allFreeCells, ([np.argmin(distances)]), axis=0)
    PubAck("/reset").publishData([Reset(idx, Point(val[0], val[1],0)) for idx, val in enumerate(STARTS)])

    print(STARTS)