#!/usr/bin/env python3

import rospy
import numpy as np
from marmot.msg import Reset, Ack, TaskBroadcast
from geometry_msgs.msg import Point
import discreteUtil
import cv2
import continuousUtil
from unifiedCommsUtil import *
from parameters import *
import copy
from discreteUtil import getCoord

INTERACTIVE_GOAL_NUM = MAPFParameters.INTERACTIVE_GOAL_NUM
PATH = rospy.get_param("PATH")

if MAPFParameters.PARK:
    GOALS = [[MAPFParameters.WORLD_SIZE[0]-1, i] for i in range(8)]

else:
    GOALS = []
print(GOALS)
assert len(GOALS)+INTERACTIVE_GOAL_NUM<=DriverParameters.HYBRID_ROBOT_COUNT

if DriverParameters.SCENARIO == 0:
    WORLD = np.asarray(rospy.get_param("WORLD"), dtype=np.int64)
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

class GoalPosPubAck(PubAck):
    def __init__(self, topicName="/goalPos", MainTopic=Reset, AckTopic=Ack, ACK_TIMEOUT=0.2) -> None:
        super().__init__(topicName, MainTopic, AckTopic, ACK_TIMEOUT)

    def ackCallback(self, data):
        if len(self.ACK_STORE)==1:
            self.ACK_STORE[0] = 1
        else:
            self.ACK_STORE[data.fromRobotID] = 1

def getAndSetGoals():
    goalPosPubAck = GoalPosPubAck(ACK_TIMEOUT=ACK_TIMEOUT)
    goalPosPubAck.publishData([Reset(i, Point(0,0,0)) for i in range(DriverParameters.HYBRID_ROBOT_COUNT)])
    
    robotPos = GetRobotPosPubAck(ACK_TIMEOUT=ACK_TIMEOUT).publishData([Ack(-1, i) for i in range(DriverParameters.HYBRID_ROBOT_COUNT)])
    STARTS = [(int(np.round((x - DriverParameters.RESOLUTION / 2) / DriverParameters.RESOLUTION)), (int(np.round((y - DriverParameters.RESOLUTION / 2) / DriverParameters.RESOLUTION)))) for (x,y) in robotPos]
    

    viewStarts = copy.deepcopy(STARTS)
    viewStarts = [(MAPFParameters.WORLD_SIZE[0]-y-1, x) for (x, y) in viewStarts]
    # GOALS = []
    viewGOALS = []
    SCALE = 50
    for i in range(INTERACTIVE_GOAL_NUM):
        selectedCell = cv2.selectROI("Select Goal", discreteUtil.renderWorld(scale = SCALE,world=-1*WORLD[:DriverParameters.REAL_WORLD_SIZE[0], :DriverParameters.REAL_WORLD_SIZE[1]], agents=viewStarts, goals=viewGOALS))[:2]
        selectedCell = selectedCell[::-1]
        selectedCell = [i//SCALE for i in selectedCell]
        viewGOALS.append(selectedCell)
        goalPosPubAck.publishData([Reset(i, Point(*getCoord((selectedCell[1], MAPFParameters.WORLD_SIZE[0]-1-selectedCell[0]), DriverParameters.RESOLUTION),0))])
    cv2.destroyAllWindows()
    for i in viewGOALS:
        GOALS.append((i[1], MAPFParameters.WORLD_SIZE[0]-i[0]-1))
    while(len(GOALS)!=DriverParameters.HYBRID_ROBOT_COUNT):
        try:
            DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING[len(GOALS)]
            GOALS.append(discreteUtil.getFreeCell(GOALS, WORLD[:DriverParameters.REAL_WORLD_SIZE[0], :DriverParameters.REAL_WORLD_SIZE[1]]))
        except KeyError:
            GOALS.append(discreteUtil.getFreeCell(GOALS, WORLD))
        
    goalPosPubAck.publishData([Reset(idx, Point(*getCoord((val[0], val[1]), DriverParameters.RESOLUTION),0)) for idx, val in enumerate(GOALS)])
    planner = discreteUtil.LACAM3(WORLD, STARTS, GOALS, ADG_TYPE=discreteUtil.Reduced_TD_Graph, TIMEOUT=3)
    
    if(DriverParameters.DEBUG):
        planner.adg.fileWrite(rospy.get_param("/debug_folder"))
            # with open(rospy.get_param("/debug_folder")+"/Tasks.txt", "+a")  as f:
            #     f.write(allTasks.__str__())
    
    return planner.allTasks
    
if __name__=="__main__":
    rospy.init_node('newRun')
    rospy.sleep(1)
    
    if DriverParameters.SCENARIO == 0:
        
        if MAPFParameters.IMITATE:
            goalPosPubAck = GoalPosPubAck(ACK_TIMEOUT=ACK_TIMEOUT)
            goalPosPubAck.publishData([Reset(i, Point(0,0,0)) for i in range(DriverParameters.HYBRID_ROBOT_COUNT)])
            
            actions = np.load(PATH+MAPFParameters.IMITATE_FOLDER+"ACTIONS.npy")
            actions = actions[MAPFParameters.IMITATION_LIST]
            if(actions.shape[0] > DriverParameters.HYBRID_ROBOT_COUNT):
                actions = actions[:DriverParameters.HYBRID_ROBOT_COUNT]
            if(actions.shape[1] > MAPFParameters.IMITATION_LENGTH):
                actions = actions[:, :MAPFParameters.IMITATION_LENGTH]
            
            
            starts = np.load(PATH+MAPFParameters.IMITATE_FOLDER+"STARTS.npy")
            starts = starts[MAPFParameters.IMITATION_LIST]
            if(starts.shape[0] > DriverParameters.HYBRID_ROBOT_COUNT):
                starts = starts[:DriverParameters.HYBRID_ROBOT_COUNT]
            
            GOALS = np.load(PATH+MAPFParameters.IMITATE_FOLDER+"GOALS.npy")
            GOALS = GOALS[MAPFParameters.IMITATION_LIST]
            if(GOALS.shape[0] > DriverParameters.HYBRID_ROBOT_COUNT):
                GOALS = GOALS[:DriverParameters.HYBRID_ROBOT_COUNT]
            
            goalPosPubAck.publishData([Reset(idx, Point(*getCoord((val[0], val[1]), DriverParameters.RESOLUTION),0)) for idx, val in enumerate(GOALS)])  
            
            # planner = discreteUtil.IMITATION(starts, actions, ADG_TYPE=discreteUtil.FullySynchGraph)
            # planner = discreteUtil.IMITATION(starts, actions, ADG_TYPE=discreteUtil.NoSynchGraph)
            planner = discreteUtil.IMITATION(starts, actions, ADG_TYPE=discreteUtil.Reduced_TD_Graph)
            
            if(DriverParameters.DEBUG):
                planner.adg.fileWrite(rospy.get_param("/debug_folder"))
            
            allTasks = planner.allTasks
        
        else:
            allTasks = getAndSetGoals()
        

    elif DriverParameters.SCENARIO == 1:
        allTasks = continuousUtil.PathPlanner(PATH, DriverParameters.HYBRID_ROBOT_COUNT).allTasks
    
    tasksPubAck = TasksPubAck(ACK_TIMEOUT=ACK_TIMEOUT)
    
    tasksPubAck.publishData(allTasks)
    
    data = []
    for i in range(DriverParameters.HYBRID_ROBOT_COUNT):
        temp = TaskBroadcast()
        temp.robotID = i
        temp.taskID = -1
        data.append(copy.deepcopy(temp))
        
    tasksPubAck.publishData(data)

    
    

    
              
    