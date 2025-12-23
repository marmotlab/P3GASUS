#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from marmot.msg import Reset, Ack
import tf
from std_msgs.msg import Int8
from util import *
from unifiedCommsUtil import *
from parameters import *
import json
from discreteUtil import getCoord
import copy

RESOLUTION = DriverParameters.RESOLUTION

class RobotData:
    def __init__(self):
        self.realID = None
        self.virtualID = None
        self.robotName = None
        self.pose = Point()
        self.goal = Point()
        self.possibleNextTask = None
        self.doneTasks = dict()
        self.taskDict = dict()
        self.tasksIncoming = None
        self.lastDoneTask = None
        self.currentQueue = list()
        self.chainDependency = ChainDependencySub(self)
        self.done = True
        
        self.commsPubAck = None
        self.posePub = None
        
        self.virtualController = None
        self.realController = None
        
    def getRemainingDependentTasks(self,taskID):
        if taskID not in self.doneTasks:
            return{i.taskIDFrom for i in self.taskDict[taskID].allFrom}
            
        return {i.taskIDFrom for i in self.taskDict[taskID].allFrom} - self.doneTasks[taskID] 

    def log(self):
        with open(rospy.get_param("/debug_folder")+"/"+str(self.virtualID)+".txt", "+a")  as f:
            if self.taskDict is not None and self.currentQueue:
                f.writelines("Task Dict: "+str(self.taskDict[self.currentQueue[-1]])+"\n")
            f.writelines("Task Incoming: "+str(self.tasksIncoming)+"\n")
            f.writelines("Possible Next Task: "+str(self.possibleNextTask)+"\n")
            f.writelines("Current Queue: "+str(self.currentQueue)+"\n")
            f.writelines("Last Done Task: "+str(self.lastDoneTask)+"\n")
            f.writelines("Done Tasks: "+str(self.doneTasks)+"\n")
            f.writelines("Pose: "+str(self.pose)+"\n")
            f.writelines("Goal: "+str(self.goal)+"\n")
            f.writelines("Dist to goal"+": "+str(l2(self.pose, self.goal))+"\n")
            f.writelines("Done: "+str(self.done)+"\n")
            f.writelines("\n")

class MAPFRobot(RobotData):
    def __init__(self, virtualID, robotName, startX, startY): 
        super().__init__()
        print("Robot: "+str(virtualID)+" Created")
        
        self.virtualID = virtualID
        self.robotName = robotName
        
        self.pose.x = startX
        self.pose.y =  startY

        self.goal.x = self.pose.x
        self.goal.y = self.pose.y

        try:
            self.realID = DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING[virtualID]
        except KeyError:
            self.realID = None
        
        
        self.goalTF = MAPF_GoalTF(virtualID)
        # if(MAPFParameters.IMITATE):
        #     self.futureGoals = rospy.get_param("/futureGoals")[virtualID]
        #     self.futureGoalsIndex = 0
        #     # print("Future Goals: "+str(self.futureGoals))
        #     self.futureGoals = [getCoord((i[0], i[1]), RESOLUTION) for i in self.futureGoals]
        #     self.goalTF.pose.x = self.futureGoals[0][0]
        #     self.goalTF.pose.y = self.futureGoals[0][1]
            
        ResetSubAck(self)
        RobotPosSubAck(self)
        CommsSubAck(self)
        MAPF_TasksSubAck(topicName="/tasksBroadcast", robotData=self)
        self.commsPubAck = CommsPubAck()
        self.posePub = rospy.Publisher("/poseStream_"+str(virtualID),Reset, queue_size=1)
        
        if(self.realID is not None):
            self.controller = MAPF_RealController(self.realID, self.virtualID)
            rospy.sleep(0.5)
            self.goal = copy.deepcopy(self.controller.pos)
            # self.goal.z = np.pi
        else:
            self.controller = MAPF_VirtualController(self.pose, self.virtualID)

        
    
    def move(self):
        # print(self.goal)
        self.pose = self.controller.move(self.goal) 
    
    def publishDoneDependentTasks(self):
        if(self.lastDoneTask is not None and self.lastDoneTask+1 in self.currentQueue):
            taskInfo = self.taskDict[self.lastDoneTask+1]
            
            temp = [i.robotIDTo for i in taskInfo.allTo if i.dependencyType==3]
            if(len(temp)!=0):
                msg = Reset(temp[0], self.pose)
                self.posePub.publish(msg)
        
        if(self.lastDoneTask is None and 1 in self.currentQueue):
            taskInfo = self.taskDict[1]
            
            temp = [i.robotIDTo for i in taskInfo.allTo if i.dependencyType==3]
            if(len(temp)!=0):
                msg = Reset(temp[0], self.pose)
                self.posePub.publish(msg)
                
                
    def updateDoneTasks(self):  
        if(self.lastDoneTask is None):
            st = 0
        else:
            st = self.currentQueue.index(self.lastDoneTask)+1
            
        for i in range(len(self.currentQueue)-1, st-1, -1):
            if(l2(self.pose, self.taskDict[self.currentQueue[i]].goal, True)<MAPFParameters.VIRTUAL_FAR_THRESHOLD):
                
                for j in range(st, i+1):
                    data = [x for x in self.taskDict[self.currentQueue[j]].allTo if x.robotIDTo!=self.virtualID]
                    
                    if(self.currentQueue[j]+1 in self.taskDict):
                        if(self.currentQueue[j]+1 not in self.doneTasks):
                            self.doneTasks[self.currentQueue[j]+1] = set()    
                        self.doneTasks[self.currentQueue[j]+1].add(self.currentQueue[j])

                    self.commsPubAck.publishData(data) 
                    
                self.lastDoneTask = self.currentQueue[i]
                return
                
                
    def helper(self):
         
        if(self.tasksIncoming is None or self.tasksIncoming): #If no tasks in queue, or if new tasks incoming, do nothing
            return
        
        if(self.possibleNextTask is None):
            self.possibleNextTask = min(self.taskDict) 
    
        if(self.possibleNextTask>max(self.taskDict)):
            self.done = True
            return    
        
        if(DriverParameters.DEBUG):
            self.log()
        dependentTasksRem = self.getRemainingDependentTasks(self.possibleNextTask)
        logDebugData("Possible Next Task: "+str(self.possibleNextTask)+"\n", self.virtualID)
        logDebugData("Dependent Tasks Remaining: "+str(dependentTasksRem)+"\n", self.virtualID)
        
        
        while(not rospy.is_shutdown() and len(dependentTasksRem)<=1):
            ENQUEUE = False
            
            if(len(dependentTasksRem)==0):
                self.chainDependency.closeSub()
                ENQUEUE = True
            else:
                taskInfo = [i for i in self.taskDict[self.possibleNextTask].allFrom if i.taskIDFrom == list(dependentTasksRem)[0]][0]
            
                if(taskInfo.dependencyType==2):
                    pass
                elif(taskInfo.dependencyType==3):
                    self.chainDependency.setLeader(taskInfo.robotIDFrom)
                    temp = self.chainDependency.getGoal(self.taskDict[self.possibleNextTask].action)
                    
                    g = copy.deepcopy(self.taskDict[self.possibleNextTask].goal)
                    g.x, g.y = getCoord((g.x, g.y), RESOLUTION)
                    
                    if(temp[0] is None and temp[1] is None):
                        return
                    
                    elif(temp[0] is None):
                        self.goal.x = g.x
                        self.goal.y = temp[1]
                        return
                        
                    else:
                        self.goal.x = temp[0]
                        self.goal.y = g.y
                        return
                    
                else:
                    if(self.taskDict[self.currentQueue[-1]].action == self.taskDict[self.possibleNextTask].action or self.taskDict[self.possibleNextTask].action==0):
                        ENQUEUE = True
                    else:
                        pass
            
            
            if(not ENQUEUE):
                return
            else:
                self.goal = copy.deepcopy(self.taskDict[self.possibleNextTask].goal)
                self.goal.x, self.goal.y = getCoord((self.goal.x, self.goal.y), RESOLUTION)
                # print(goal)
                self.currentQueue.append(self.possibleNextTask)
                
                self.possibleNextTask = self.possibleNextTask+1 
                if(self.possibleNextTask in self.taskDict):    
                    dependentTasksRem = self.getRemainingDependentTasks(self.possibleNextTask)
                else:
                    return

class MAPF_GoalTF:
    def __init__(self, virtualID):
        self.br = tf.TransformBroadcaster()
        self.robotNumber = virtualID
        self.pose = Point()
        self.pose.x = -1
        self.pose.y = -1
        self.goalPoseSub = GetGoalPosSubAck(self)
        
    def update(self):
        self.br.sendTransform(
            (self.pose.x, self.pose.y, 0.002),
            (0,0,0,1),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"_goal/pole_footprint",
            "/map"
            )      
        self.br.sendTransform(
            (0, 0, 0),
            (0,0,0,1),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"_goal/pole_link",
            "/nexus"+str(self.robotNumber)+"_goal/pole_footprint",
            ) 

class TrafficRobot(RobotData):
    def __init__(self, virtualID, robotName):
        super().__init__()
        self.virtualID = virtualID
        self.robotName = robotName
        self.pose = Point()
        self.goal = Point()
        
        
        try:
            self.realID = DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING[virtualID]
            self.realTaskStart = 0
            self.realTaskEnd = 0

        except KeyError:
            self.realID = None
            self.realTaskStart = None
            self.realTaskEnd = None

        Traffic_TasksSubAck(topicName="/tasksBroadcast", robotData=self)
        CommsSubAck(self)
        self.commsPubAck = CommsPubAck()

        if(self.realID is not None):
            self.realController = TRAFFIC_RealController(self.realID, self.virtualID)
            self.realStartTask, self.realEndTask = None, None

            # Read the start.json file and store its contents in a dictionary
            start_file_path = rospy.get_param("/PATH") + "/scenarios/Traffic/start.json"
            with open(start_file_path, 'r') as file:
                start_data = json.load(file)

            self.leader = start_data.get(str(self.virtualID), None)
            self.slave = [key for key, value in start_data.items() if value == self.virtualID]
            self.slave = int(self.slave[0]) if self.slave else None

            # Read the end.json file and store its contents in a dictionary
            end_file_path = rospy.get_param("/PATH") + "/scenarios/Traffic/end.json"
            with open(end_file_path, 'r') as file:
                end_data = json.load(file)

            self.realTaskEnd = end_data.get(str(self.virtualID), {})
            print(self.realTaskEnd, len(self.realTaskEnd))
            self.realEndPos = Point()
            self.realEndPos.x, self.realEndPos.y = float(self.realTaskEnd[0]), float(self.realTaskEnd[1])
            

            self.posePub = rospy.Publisher("/poseStream_"+str(virtualID), Reset, queue_size=1)
            self.chainDependency = ChainDependencySub(self)

            rospy.sleep(0.5)

            self.goal = copy.deepcopy(self.realController.pos)
        # else:
        self.virtualController = TRAFFIC_VirtualController(self.virtualID)
        

    def move(self):
        if(self.realID is not None):
            if self.realStartTask is None or self.realEndTask is None:
                self.realController.move2d(self.goal)
                self.posePub.publish(Reset(self.slave, self.realController.pos))
                self.chainDependency.setLeader(self.leader)
                
                # print("Leader: ", self.leader, "Slave:", self.slave, self.chainDependency.leaderPose)
                
                # self.pose = self.realController.move(None, self.goal)
            
            else:
                if self.lastDoneTask is not None and self.realStartTask <= self.lastDoneTask <= self.realEndTask:
                    self.chainDependency.closeSub()
                    self.posePub.publish(Reset(self.slave, Point(1e8, 1e8,0))) # Publish a dummy point to stop the pose stream

                    self.realController.setTfPublish(True)
                    self.pose = self.realController.move(None, self.goal)

                elif(self.lastDoneTask is not None and self.lastDoneTask > self.realEndTask): # End of the real robot mmovement
                    self.realController.setTfPublish(False)
                    self.realController.move2d(self.realEndPos)

                    self.pose = self.virtualController.move(self.pose, self.goal)

                else:
                    self.posePub.publish(Reset(self.slave, self.realController.pos))
                    
                    self.realController.setTfPublish(False)
                    realGoal = Point()
                    realGoal.x, realGoal.y = getCoord((self.taskDict[self.realStartTask].goal.x, self.taskDict[self.realStartTask].goal.y), MAPF=False)
                    realGoal.z = np.arctan2(
                        self.taskDict[self.realStartTask + 1].goal.y - self.taskDict[self.realStartTask].goal.y,
                        self.taskDict[self.realStartTask + 1].goal.x - self.taskDict[self.realStartTask].goal.x
                    )

                    # if(self.chainDependency.leaderPose is not None):
                    #     print()
                    if (self.leader is None or (self.chainDependency.leaderPose is not None and l2(self.realController.pos, self.chainDependency.leaderPose, MAPF=False) > MAPFParameters.SAFETY_DISTANCE)):
                        self.realController.move2d(realGoal)

                    # self.realController.move2d(realGoal)
                    self.pose = self.virtualController.move(self.pose, self.goal)
        else:
            self.pose = self.virtualController.move(self.pose, self.goal)
        # pass
        # self.pose = self.controller.move(self.pose, self.goal) 
    
    def helper(self):
    
        if(self.tasksIncoming is None or self.tasksIncoming): #If no tasks in queue, or if new tasks incoming, do nothing
            return
        
        if(self.possibleNextTask is None):
            self.possibleNextTask = min(self.taskDict) 
            # self.getRealGoalPos()
        
        if(self.possibleNextTask>=max(self.taskDict)):
            self.updateDoneTasks()
            self.pose = Point()
            self.goal = Point()
            return
    
    
        if(DriverParameters.DEBUG):
            self.log()
        
        dependentTasksRem = self.getRemainingDependentTasks(self.possibleNextTask)
        
        for _ in range(TrafficParameters.FUTURE_TASKS):
            if(rospy.is_shutdown() or len(dependentTasksRem)>=1):
                return
        
            ENQUEUE = False
        
            if(len(dependentTasksRem)==0):
                ENQUEUE=True
            else:
                taskInfo = [i for i in self.taskDict[self.possibleNextTask].allFrom if i.taskIDFrom == list(dependentTasksRem)[0]][0]
                
                if(taskInfo.dependencyType==2):
                    return
                else:
                    # if(l2(goal, taskDict[possibeNextTask].goal, True)<FAR_THRESHOLD):
                    ENQUEUE = True
                    
            if(ENQUEUE):
                self.goal = copy.deepcopy(self.taskDict[self.possibleNextTask].goal)
                if(self.goal.x == -2 and self.goal.y == -2):
                    self.currentQueue = list(self.taskDict.keys())
                    self.updateDoneTasks()
                    self.pose = Point()
                    self.goal = Point()
                    return
                
                self.goal.x, self.goal.y = getCoord((self.goal.x, self.goal.y), MAPF=False)
                self.currentQueue.append(self.possibleNextTask)
                
                self.possibleNextTask = self.possibleNextTask+1 
                
                if(self.possibleNextTask>=max(self.taskDict)):
                    self.pose = Point()
                    self.goal = Point()
                    return
                
                dependentTasksRem = self.getRemainingDependentTasks(self.possibleNextTask)
                
            else:
                return
    
    def updateDoneTasks(self):  
        if(self.lastDoneTask is None):
            st = 0
        else:
            st = self.currentQueue.index(self.lastDoneTask)+1
            
        for i in range(len(self.currentQueue)-1, st-1, -1):
            if(l2(self.pose, self.taskDict[self.currentQueue[i]].goal, True, MAPF=False)<TrafficParameters.FAR_THRESHOLD):
                for j in range(st, i+1):
                    data = [x for x in self.taskDict[self.currentQueue[j]].allTo if x.robotIDTo!=self.virtualID]
                    
                    if(self.currentQueue[j]+1 in self.taskDict):
                        if(self.currentQueue[j]+1 not in self.doneTasks):
                            self.doneTasks[self.currentQueue[j]+1] = set()    
                        self.doneTasks[self.currentQueue[j]+1].add(self.currentQueue[j])

                    self.commsPubAck.publishData(data) 
                    
                self.lastDoneTask = self.currentQueue[i]
                return        