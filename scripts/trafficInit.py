#!/usr/bin/env python3

import rospy
import numpy as np
from marmot.msg import Reset, Ack
from geometry_msgs.msg import Point
import discreteUtil
import cv2
import continuousUtil
from unifiedCommsUtil import *
from parameters import *
from oneRun import GoalPosPubAck
import json

rospy.init_node('newRun')
rospy.sleep(1)


class RealRobotTasks:
    def __init__(self, virtualID):
        self.robotID = virtualID
        
        self.realStartTask = None
        self.realEndTask = None

        self.realStartPos = Point()
        self.realStartDependency = None

        self.currentPos = Point()

        self.realEndPos = Point()

    def __repr__(self):
        return f"\nRobotID: {self.robotID},\n Start TaskID: {self.realStartTask.taskID if self.realStartTask else None}, Start Time: {self.realStartTask.time if self.realStartTask else None},\nEnd TaskID: {self.realEndTask.taskID if self.realEndTask else None}, End Time: {self.realEndTask.time if self.realEndTask else None},\nStart Position: ({self.realStartPos.x}, {self.realStartPos.y}, {self.realStartPos.z})\n, Current Position: ({self.currentPos.x}, {self.currentPos.y}, {self.currentPos.z})\n, End Position: ({self.realEndPos.x}, {self.realEndPos.y}, {self.realEndPos.z})\n"
            


PATH = rospy.get_param("PATH")
GOALS = []
if DriverParameters.SCENARIO == 0:
    WORLD = np.asarray(rospy.get_param("WORLD"), dtype=np.int64)
ACK_TIMEOUT = 0.02
allTasks = continuousUtil.PathPlanner(PATH, DriverParameters.HYBRID_ROBOT_COUNT).adg.taskList
# print(allTasks.taskList)
realRobotTasks = {i: RealRobotTasks(i) for i in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING}

originTransformed = getCoord(TrafficParameters.ORIGIN, MAPF=False)
realWorldShift = (DriverParameters.RESOLUTION*np.array(DriverParameters.REAL_WORLD_SIZE))/2
leftTop = originTransformed - realWorldShift
bottomRight = originTransformed + realWorldShift

print("Left Top: ", leftTop)
print("Bottom Right: ", bottomRight)


for taskID in allTasks:
    i = allTasks[taskID]
    if i.robotID in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING:
        p = Point()
        p.x,p.y = getCoord(i.goalPos, MAPF=False)
        if leftTop[0] <= p.x <= bottomRight[0] and leftTop[1] <= p.y <= bottomRight[1]:
            if(realRobotTasks[i.robotID].realStartTask is None):
                temp = Point()
                temp.x, temp.y = np.subtract(
                [p.x, p.y],
                originTransformed)
                direction = None
                if(abs(temp.x)>abs(temp.y)):
                    if(temp.x<0):
                        direction = 0
                    else:
                        direction = np.pi
                else:
                    if(temp.y>0):
                        direction = -np.pi/2
                    else:
                        direction = np.pi/2  
                realRobotTasks[i.robotID].realStartTask = i
                realRobotTasks[i.robotID].realStartPos = Point(0, 0, direction)

        else:
            if(realRobotTasks[i.robotID].realStartTask is not None and realRobotTasks[i.robotID].realEndTask is None):
                temp = Point()
                temp.x, temp.y = np.subtract(
                [p.x, p.y],
                originTransformed)
                direction = None
                if(abs(temp.x)>abs(temp.y)):
                    if(temp.x<0):
                        direction = 0
                    else:
                        direction = np.pi
                else:
                    if(temp.y>0):
                        direction = -np.pi/2
                    else:
                        direction = np.pi/2  
                realRobotTasks[i.robotID].realEndTask = i
                realRobotTasks[i.robotID].realEndPos = Point(0, 0, direction)

# print("Real Robot Tasks: ", realRobotTasks) 

sorted_start_dict = {k: v for k, v in sorted(realRobotTasks.items(), key=lambda item: item[1].realStartTask.time if item[1].realStartTask else float('inf'))}
sorted_end_dict = {k: v for k, v in sorted(realRobotTasks.items(), key=lambda item: item[1].realStartTask.time if item[1].realStartTask else float('inf'), reverse=True)}
# print("Sorted Start Dict: ", sorted_start_dict)

GOALS = {}
startDependency = {}

for val in sorted_start_dict:
    if(sorted_start_dict[val].realStartPos.z == 0):
        temp = [0, int(DriverParameters.REAL_WORLD_SIZE[1]/2)]
        while tuple(temp) in GOALS.values():
            startDependency[val] = next(key for key, value in GOALS.items() if value == tuple(temp))
            temp[1] -= 1
    elif(sorted_start_dict[val].realStartPos.z == np.pi):
        temp = [DriverParameters.REAL_WORLD_SIZE[0]-1, int(DriverParameters.REAL_WORLD_SIZE[1]/2)]
        while tuple(temp) in GOALS.values():
            startDependency[val] = next(key for key, value in GOALS.items() if value == tuple(temp))
            temp[1] += 1
    elif(sorted_start_dict[val].realStartPos.z == -np.pi/2):
        temp = [int(DriverParameters.REAL_WORLD_SIZE[0]/2), 0]
        while tuple(temp) in GOALS.values():
            startDependency[val] = next(key for key, value in GOALS.items() if value == tuple(temp))
            temp[0] += 1
    elif(sorted_start_dict[val].realStartPos.z == np.pi/2):
        temp = [int(DriverParameters.REAL_WORLD_SIZE[0]/2), DriverParameters.REAL_WORLD_SIZE[1]-1]
        while tuple(temp) in GOALS.values():
            startDependency[val] = next(key for key, value in GOALS.items() if value == tuple(temp))
            temp[0] -= 1
    GOALS[val] = tuple(temp)
    realRobotTasks[val].realStartPos.x, realRobotTasks[val].realStartPos.y = temp

# Save startDependency to a file for future use

start_dependency_file = PATH+"/scenarios/Traffic/start.json"
with open(start_dependency_file, 'w') as file:
    json.dump(startDependency, file)

print(f"Start Dependency saved to {start_dependency_file}")

endPos = {}

for val in sorted_end_dict:
    if(sorted_end_dict[val].realEndPos.z == 0):
        temp = [leftTop[0], (leftTop[1]+bottomRight[1])/2]
        while tuple(temp) in endPos.values():
            temp[1] -= 0.3
    elif(sorted_end_dict[val].realEndPos.z == np.pi):
        temp = [bottomRight[0], (leftTop[1]+bottomRight[1])/2]
        while tuple(temp) in endPos.values():
            temp[1] += 0.3
    elif(sorted_end_dict[val].realEndPos.z == np.pi/2):
        temp = [(leftTop[0]+bottomRight[0])/2, leftTop[1]]
        while tuple(temp) in endPos.values():
            temp[0] += 0.3
    elif(sorted_end_dict[val].realEndPos.z == -np.pi/2):
        temp = [(leftTop[0]+bottomRight[0])/2, bottomRight[1]]
        while tuple(temp) in endPos.values():
            temp[0] -= 0.3
    endPos[val] = tuple(temp)
    realRobotTasks[val].realEndPos.x, realRobotTasks[val].realEndPos.y = temp
print(endPos)
# Save endPos to a file for future use
end_pos_file = PATH + "/scenarios/Traffic/end.json"
with open(end_pos_file, 'w') as file:
    json.dump({k: list(v) for k, v in endPos.items()}, file)

print(f"End Positions saved to {end_pos_file}")

# print("GOALS: ", GOALS)
# print("Start Dependency: ", startDependency)
goalPosPubAck = GoalPosPubAck(ACK_TIMEOUT=ACK_TIMEOUT)
goalPosPubAck.publishData([Reset(i, Point(0,0,0)) for i in range(DriverParameters.HYBRID_ROBOT_COUNT)])

robotPos = GetRobotPosPubAck(ACK_TIMEOUT=ACK_TIMEOUT).publishData([Ack(-1, i) for i in range(DriverParameters.HYBRID_ROBOT_COUNT)])

STARTS = [(int(np.round((x - DriverParameters.RESOLUTION / 2) / DriverParameters.RESOLUTION)), (int(np.round((y - DriverParameters.RESOLUTION / 2) / DriverParameters.RESOLUTION)))) for (x,y) in robotPos]
print(STARTS)
for idx, val in enumerate(STARTS):
    if(idx in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING):
        realRobotTasks[idx].currentPos.x, realRobotTasks[idx].currentPos.y = val

print("Real Robot Tasks: ", realRobotTasks) 

# print("also")s
# for i in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING:
#     print(i)

STARTS = [[realRobotTasks[i].currentPos.x, realRobotTasks[i].currentPos.y] for i in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING]
GOALS = [[realRobotTasks[i].realStartPos.x, realRobotTasks[i].realStartPos.y] for i in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING]
# print(STARTS)
# print(GOALS)

planner = discreteUtil.LACAM3(WORLD, STARTS, GOALS, ADG_TYPE=discreteUtil.Reduced_TD_Graph)


for idx, val in enumerate(planner.allTasks):
    # print(idx, val)
    
    val.robotID = list(DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING.keys())[val.robotID]
    val.goal.z = realRobotTasks[val.robotID].realStartPos.z
    print("Task ID:", val.taskID, "Robot ID:", val.robotID, "Start Position:", (val.start.x, val.start.y), "Goal Position:", (val.goal.x, val.goal.y), "Direction:", val.goal.z)


tasksPubAck = TasksPubAck(ACK_TIMEOUT=ACK_TIMEOUT)

tasksPubAck.publishData(planner.allTasks)

data = []
for i in (DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING):
    temp = TaskBroadcast()
    temp.robotID = i
    temp.taskID = -1
    data.append(copy.deepcopy(temp))
    
tasksPubAck.publishData(data)