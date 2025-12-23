#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, PoseStamped, Point
from parameters import *    
from marmot.msg import Reset, Ack, TaskBroadcast, TaskAck
from util import *
import copy
from discreteUtil import getCoord
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import math
# -------------------------------------------------------------------------------   
# ------------------------------Publisher Subscriber Acknowledgement Util ------------------------------------
# -------------------------------------------------------------------------------

def logDebugData(data, robotID):
    if(DriverParameters.DEBUG):
        with open(rospy.get_param("/debug_folder")+"/"+str(robotID)+".txt", "+a")  as f:
            f.writelines(data)

class PubAck:
    def __init__(self, topicName = "reset", MainTopic = Reset, AckTopic = Ack, ACK_TIMEOUT = 0.2) -> None:
        self.publisher = rospy.Publisher(topicName, MainTopic, queue_size=10000)
        self.subscriber = rospy.Subscriber(topicName+"_ack", AckTopic, self.ackCallback)
        rospy.sleep(0.0001)

        self.ACK_TIMEOUT = ACK_TIMEOUT

    def publishData(self, data):
        dataCount = len(data)
        self.ACK_STORE = np.zeros(dataCount)

        self.WAITTIME = np.zeros(dataCount)
        while not rospy.is_shutdown() and not np.array_equal(np.ones_like(self.ACK_STORE), self.ACK_STORE):
            pending = np.where(self.ACK_STORE==0)[0]
            for i in pending:
                if((rospy.get_time()-self.WAITTIME[i])>self.ACK_TIMEOUT):
                    self.publisher.publish(data[i])
                    self.WAITTIME[i] = rospy.get_time()
                    
            rospy.sleep(0.0001)
        
    def ackCallback(self, data):
        self.ACK_STORE[data.fromRobotID] = 1
        
class SubAck:
    def __init__(self, robotData, topicName = "reset", MainTopic = Reset, AckTopic = Ack) -> None:
        self.publisher = rospy.Publisher(topicName+"_ack", AckTopic, queue_size=10000)
        self.subscriber = rospy.Subscriber(topicName, MainTopic, self.callback)
        self.robotData = robotData
        
        self.ACK_MSG = AckTopic()
        
    def callback(self, data):
        pass


class ResetSubAck(SubAck):
    def __init__(self, robotData, topicName="/reset", MainTopic=Reset, AckTopic=Ack) -> None:
        super().__init__(robotData, topicName, MainTopic, AckTopic)
        self.ACK_MSG.fromRobotID = self.robotData.virtualID
    
    def callback(self, data): 
        if(data.robotID == self.robotData.virtualID):
            self.publisher.publish(self.ACK_MSG)
            self.robotData.goal = data.goal


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

class RobotPosSubAck(SubAck):
    def __init__(self, robotData, topicName="/getRobotPos", MainTopic=Ack, AckTopic=Reset) -> None:
        super().__init__(robotData, topicName, MainTopic, AckTopic)
        
    def callback(self, data):
        if(data.toRobotID == self.robotData.virtualID):
            # print(data)
            self.robotData.pose = self.robotData.goal
            self.ACK_MSG.robotID = self.robotData.virtualID
            self.ACK_MSG.goal = self.robotData.pose
            self.publisher.publish(self.ACK_MSG)
            
class TasksPubAck(PubAck):
    def __init__(self, topicName="/tasksBroadcast", MainTopic=TaskBroadcast, AckTopic=Int32, ACK_TIMEOUT=0.2) -> None:
        super().__init__(topicName, MainTopic, AckTopic, ACK_TIMEOUT)
    def publishData(self, data):
        if(data[0].taskID == -1):
            self.transform = {val.robotID:idx for idx,val in enumerate(data)}
        else:
            self.transform = {val.taskID:idx for idx,val in enumerate(data)}
        
        return super().publishData(data)
    def ackCallback(self, data):
        try:
            self.ACK_STORE[self.transform[data.data]] = 1
        except:
            pass
        
class MAPF_TasksSubAck(SubAck):
    def __init__(self, robotData, topicName="/tasksBroadcast", MainTopic=TaskBroadcast, AckTopic=Int32) -> None:
        super().__init__(robotData, topicName, MainTopic, AckTopic) 
    
    def callback(self, data):
        # print(data)
        # if(DriverParameters.DEBUG):
        #     with open(rospy.get_param("/debug_folder")+"/"+str(self.robotData.virtualID)+".txt", "+a")  as f:
        #         f.writelines("Task Incoming: "+str(data.robotID)+", "+str(data.taskID)+", "+str(data.goal.x)+", "+str(data.goal.y)+"\n")
        if(data.robotID == self.robotData.virtualID):
            if(data.taskID == -1):
                self.robotData.tasksIncoming = False
                self.ACK_MSG = data.robotID
                self.publisher.publish(self.ACK_MSG)
                return
            
            elif(self.robotData.tasksIncoming is not None):
                self.robotData.tasksIncoming = None
                self.robotData.taskDict = dict()
                self.robotData.doneTasks = dict()
                self.robotData.possibleNextTask = None
                self.robotData.currentQueue = list()
                self.robotData.lastDoneTask = None
            self.done = False
            self.robotData.taskDict[data.taskID] = data
            self.ACK_MSG = data.taskID
            self.publisher.publish(self.ACK_MSG)

            
class CommsPubAck(PubAck):
    def __init__(self, topicName="/comms", MainTopic=TaskAck, AckTopic=Ack, ACK_TIMEOUT=0.2) -> None:
        super().__init__(topicName, MainTopic, AckTopic, ACK_TIMEOUT)
        self.transform = dict()
        
    def publishData(self, data):
        self.transform = {(val.taskIDTo, val.robotIDTo):idx for idx,val in enumerate(data)}
        return super().publishData(data)
        
    def ackCallback(self, data):
        if((data.fromRobotID, data.toRobotID) in self.transform):
            self.ACK_STORE[self.transform[(data.fromRobotID, data.toRobotID)]] = 1

class CommsSubAck(SubAck):
    def __init__(self, robotData, topicName="/comms", MainTopic=TaskAck, AckTopic=Ack) -> None:
        super().__init__(robotData, topicName, MainTopic, AckTopic)
        
    def callback(self, data):
        if(data.robotIDTo == self.robotData.virtualID):
            self.ACK_MSG.fromRobotID = data.taskIDTo
            self.ACK_MSG.toRobotID = data.robotIDTo
            self.publisher.publish(self.ACK_MSG)
            
            if data.taskIDTo not in self.robotData.doneTasks:
                self.robotData.doneTasks[data.taskIDTo] = set()
            
            self.robotData.doneTasks[data.taskIDTo].add(data.taskIDFrom)

class GetGoalPosSubAck(SubAck):
    def __init__(self, goalData, topicName="/goalPos", MainTopic=Reset, AckTopic=Ack) -> None:
        super().__init__(goalData, topicName, MainTopic, AckTopic)
        self.ACK_MSG = Ack()
        self.ACK_MSG.fromRobotID = self.robotData.robotNumber
    
    def callback(self, data): 
        if(data.robotID == self.robotData.robotNumber):
            self.publisher.publish(self.ACK_MSG)
            self.robotData.pose = data.goal

class ChainDependencySub():
    def __init__(self, robotData) -> None:
        self.robotData = robotData
        self.sub = None
        self.leadingRobot = None
        self.leaderPose = None
        
    def callback(self, data):
        if(data.robotID == self.robotData.virtualID):
            self.leaderPose = data.goal
            
    def setLeader(self, leaderID):
        if(self.leadingRobot is not None and self.leadingRobot==leaderID):
            return
        
        if(self.leadingRobot is not None):
            self.closeSub()    
            
        self.leadingRobot = leaderID
        self.sub = rospy.Subscriber("/poseStream_"+str(self.leadingRobot), Reset, self.callback)
    
    def getGoal(self, action):
        SAFETY_DISTANCE = MAPFParameters.SAFETY_DISTANCE
        if(self.leaderPose is None):
            return [None, None]
        if(action == 2):
            return [None, self.leaderPose.y-SAFETY_DISTANCE]
    
        elif(action == 3):
            return [self.leaderPose.x+SAFETY_DISTANCE, None]
        
        elif(action == 4):
            return [None, self.leaderPose.y+SAFETY_DISTANCE]
        
        else:
            return [self.leaderPose.x-SAFETY_DISTANCE, None]
            
    def closeSub(self):
        if(self.sub is not None):
            self.sub.unregister()
            self.sub = None
            self.leaderPose = None
            self.leadingRobot = None

class Traffic_TasksSubAck(SubAck):
    def __init__(self, robotData, topicName="/tasksBroadcast", MainTopic=TaskBroadcast, AckTopic=Int32) -> None:
        super().__init__(robotData, topicName, MainTopic, AckTopic) 
    
    def callback(self, data):
        # print(data)
        # if(DriverParameters.DEBUG):
        #     with open(rospy.get_param("/debug_folder")+"/"+str(self.robotData.virtualID)+".txt", "+a")  as f:
        #         f.writelines("Task Incoming: "+str(data.robotID)+", "+str(data.taskID)+", "+str(data.goal.x)+", "+str(data.goal.y)+"\n")
        if(data.robotID == self.robotData.virtualID):
            if(data.taskID == -1):
                self.robotData.tasksIncoming = False
                self.ACK_MSG = data.robotID
                self.publisher.publish(self.ACK_MSG)
                
                st = self.robotData.taskDict[min(self.robotData.taskDict)].start
                self.robotData.pose.x, self.robotData.pose.y = getCoord((st.x, st.y), MAPF=False)
                
                temp = Point()
                temp.x, temp.y = np.subtract(
                    [self.robotData.pose.x, self.robotData.pose.y],
                    getCoord(TrafficParameters.ORIGIN, MAPF=False)
                )
                if(abs(temp.x)>abs(temp.y)):
                    if(temp.x<0):
                        self.robotData.pose.z = 0
                    else:
                        self.robotData.pose.z = np.pi
                else:
                    if(temp.y>0):
                        self.robotData.pose.z = -np.pi/2
                    else:
                        self.robotData.pose.z = np.pi/2        
                self.robotData.goal = copy.deepcopy(self.robotData.pose)

                if(self.robotData.realID is not None):
                    self.robotData.realStartTask, self.robotData.realEndTask = getRealRobotStartAndEndTasks(self.robotData.taskDict)
                    # print(self.robotData.realStartTask, self.robotData.realEndTask)
                    # print(self.robotData.taskDict[self.robotData.realStartTask], self.robotData.taskDict[self.robotData.realEndTask])
                return
            
            elif(self.robotData.tasksIncoming is not None):
                self.robotData.tasksIncoming = None
                self.robotData.taskDict = dict()
                self.robotData.doneTasks = dict()
                self.robotData.possibleNextTask = None
                self.robotData.currentQueue = list()
                self.robotData.lastDoneTask = None
            self.done = False
            self.robotData.taskDict[data.taskID] = data
            self.ACK_MSG = data.taskID
            self.publisher.publish(self.ACK_MSG)

def getRealRobotStartAndEndTasks(taskDict):
    originTransformed = getCoord(TrafficParameters.ORIGIN, MAPF=False)
    realWorldShift = (DriverParameters.RESOLUTION*np.array(DriverParameters.REAL_WORLD_SIZE))/2
    leftTop = originTransformed - realWorldShift
    bottomRight = originTransformed + realWorldShift

    startTask = None
    endTask = None

    for taskID in taskDict:
        # print(taskID, taskDict[taskID])
        p = Point()
        p.x,p.y = getCoord([taskDict[taskID].goal.x, taskDict[taskID].goal.y], MAPF=False)
        # print(p.x, p.y)
        if leftTop[0] <= p.x <= bottomRight[0] and leftTop[1] <= p.y <= bottomRight[1]:
            if(startTask is None):
                startTask = taskID

        else:
            if(startTask is not None and endTask is None):
                endTask = taskID
                return startTask, endTask         
# --------------------------------------------------------------------------------
# ------------------------------- MAPF Controllers --------------------------------
# --------------------------------------------------------------------------------
class MAPF_RealController:
    def __init__(self, realRobotNumber, robotNumber) -> None:
        robotName = "Robo"+str(realRobotNumber)
        self.robotNumber = robotNumber
        
        self.pos = Point()
        self.br = tf.TransformBroadcaster()
        self.tfPublish = True
        self.velPub = rospy.Publisher("/"+robotName+"/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/vrpn_client_node/RigidBody"+str(realRobotNumber)+"/pose", PoseStamped, self.odomCallback)
        
    def setTfPublish(self, tfPublish):  
        self.tfPublish = tfPublish

    def odomCallback(self, data):
        a,b = data.pose.position.x, -data.pose.position.y
        self.pos.x=a
        self.pos.y=b
        # print(self.pos)
        orientation_q = data.pose.orientation
        roll, pitch, yaw = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.pos.z = ((yaw) % (2*np.pi)) - np.pi

        # print(self.pos)

        if(self.tfPublish):
            
            self.br.sendTransform(
            (self.pos.x, self.pos.y, 0.075),
            quaternion_from_euler(0,0, self.pos.z),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/base_footprint",
            "/map"
            )
            self.br.sendTransform(
            (0.0, 0.0, 0),
            (0,0,0,1),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/base_link",
            "/nexus"+str(self.robotNumber)+"/base_footprint",
            )
            self.br.sendTransform(
            (0.075, 0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel1",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (-0.075, -0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel2",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (0.075, -0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel3",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (-0.075, 0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel4",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
        
    def move(self, goal):
        # goal.z = np.pi/2
        vel = getMAPFVel(self.pos, goal, real=True)
        self.velPub.publish(vel)
        # print(vel)
        return self.pos
        

class MAPF_VirtualController:
    def __init__(self, pose, robotNumber):
        self.vel = Twist()
        self.pos = pose
        self.br = tf.TransformBroadcaster()
        self.robotNumber = robotNumber
        
    def __ramp_velocity__(self, target, current, max_change):
        """Gradually change the current velocity towards the target velocity."""
        delta = target - current
        # Limit the change to the maximum allowed change (acceleration or deceleration)
        if abs(delta) > max_change:
            delta = max_change * np.copysign(1, delta)  # Preserve the sign
        return current + delta 
    
    def move(self, goal):
            
        vel = getMAPFVel(self.pos, goal, real=False, robotID=self.robotNumber)
        # print(vel)
        prev_vel = self.vel
        vel.linear.x = self.__ramp_velocity__(vel.linear.x, prev_vel.linear.x, (MAPFParameters.VIRTUAL_FAST_VEL+MAPFParameters.VIRTUAL_SLOW_VEL)/4)
        vel.linear.y = self.__ramp_velocity__(vel.linear.y, prev_vel.linear.y, (MAPFParameters.VIRTUAL_FAST_VEL+MAPFParameters.VIRTUAL_SLOW_VEL)/4)
        
        self.pos.x += vel.linear.x
        self.pos.y += vel.linear.y
        
        self.br.sendTransform(
        (self.pos.x, self.pos.y, 0.075),
        quaternion_from_euler(0,0, self.pos.z+np.pi/2),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/base_footprint",
        "/map"
        )
        self.br.sendTransform(
        (0.0, 0.0, 0.0),
        (0,0,0,1),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/base_link",
        "/nexus"+str(self.robotNumber)+"/base_footprint",
        )
        self.br.sendTransform(
        (0.075, 0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel1",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (-0.075, -0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel2",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (0.075, -0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel3",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (-0.075, 0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel4",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        
        return self.pos        


def getMAPFVel(pose,goal,real=False, robotID=None):

    vel = Twist()
    x = getVel(pose.x, goal.x, real=real, robotID=robotID)
    y = getVel(pose.y, goal.y, real=real, robotID=robotID)
    if(real):
        y = -y
    # Compute angular offset between current orientation and goal orientation
    angOffset = ((goal.z - pose.z) + np.pi) % (2 * np.pi) - np.pi

    # Transform velocities to align with the robot's current orientation
    vel.linear.x = x * np.cos(pose.z) + y * np.sin(pose.z)
    vel.linear.y = -x * np.sin(pose.z) + y * np.cos(pose.z)

    # Set angular velocity if angular offset exceeds threshold
    if abs(angOffset) > MAPFParameters.ANGULAR_REACH_THRESHOLD:
        vel.angular.z = MAPFParameters.REAL_ANGULAR_VEL * (angOffset / abs(angOffset))
    else:
        vel.angular.z = 0  # Stop rotating if within threshold
    

    return vel

def l2(pose, goal, convertGoal = False, MAPF=True):
        pose = np.array([pose.x, pose.y])
        goal = np.array([goal.x, goal.y])
        if(convertGoal):
            goal[:2]  = getCoord(goal[:2], DriverParameters.RESOLUTION, MAPF=MAPF)
        ans = np.linalg.norm(pose-goal, 2)  # L2 norm
        return ans

def getVel(pose,goal, real=False, robotID=None):
    if(real):
        FAST_VEL = MAPFParameters.REAL_FAST_VEL 
        SLOW_VEL = MAPFParameters.REAL_SLOW_VEL
        FAR_THRESHOLD = MAPFParameters.REAL_FAR_THRESHOLD
        REACH_THRESHOLD = MAPFParameters.REAL_REACH_THRESHOLD
    else:
        FAST_VEL = MAPFParameters.VIRTUAL_FAST_VEL #+ (DriverParameters.VELOCITY_ERROR[robotID] if robotID is not None else 0)* MAPFParameters.VIRTUAL_FAST_VEL
        SLOW_VEL = MAPFParameters.VIRTUAL_SLOW_VEL
        FAR_THRESHOLD = MAPFParameters.VIRTUAL_FAR_THRESHOLD
        REACH_THRESHOLD = MAPFParameters.VIRTUAL_REACH_THRESHOLD
    
    diff = goal-pose
    if abs(diff)>FAR_THRESHOLD:
        vel = FAST_VEL*(diff/abs(diff))
    elif abs(diff)>REACH_THRESHOLD:
        vel = (((FAST_VEL-SLOW_VEL)*(diff-REACH_THRESHOLD))/(FAR_THRESHOLD-REACH_THRESHOLD))+SLOW_VEL*(diff/abs(diff))
    else:
        vel = 0
    return vel

# -------------------------------------------------------------------------
# ------------------------------- TRAFFIC Controllers -------------------------------   
# -------------------------------------------------------------------------
           
    
class TRAFFIC_RealController:
    
    def __init__(self, realRobotNumber, virtualID) -> None:
        robotName = "Robo"+str(realRobotNumber)
        self.realPos = False
        self.br = tf.TransformBroadcaster()
        self.tfPublish = True
        self.robotNumber = virtualID
        self.pos = Point()
        self.velPub = rospy.Publisher("/"+robotName+"/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/vrpn_client_node/RigidBody"+str(realRobotNumber)+"/pose", PoseStamped, self.odomCallback)
    
    def setTfPublish(self, tfPublish):
        self.tfPublish = tfPublish

    def odomCallback(self, data):
        
        a, b = data.pose.position.x, data.pose.position.y
        originTransformed = getCoord(TrafficParameters.ORIGIN, MAPF=False)
        realWorldShift = (DriverParameters.RESOLUTION*np.array(DriverParameters.REAL_WORLD_SIZE))/2
        # realWorldShift = (0,0)
        self.pos.x=a+originTransformed[0]-realWorldShift[0]
        self.pos.y=b+originTransformed[1]+realWorldShift[1]
        # print(self.pos)
        orientation_q = data.pose.orientation
        roll, pitch, yaw = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.pos.z = ((yaw) % (2*np.pi)) - np.pi

        if(self.tfPublish):
            self.br.sendTransform(
            (self.pos.x, self.pos.y, 0.075),
            quaternion_from_euler(0,0, self.pos.z),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/base_footprint",
            "/map"
            )
            self.br.sendTransform(
            (0.0, 0.0, 0.0),
            (0,0,0,1),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/base_link",
            "/nexus"+str(self.robotNumber)+"/base_footprint",
            )
            self.br.sendTransform(
            (0.075, 0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel1",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (-0.075, -0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel2",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (0.075, -0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel3",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )
            self.br.sendTransform(
            (-0.075, 0.09, -0.03),
            (quaternion_from_euler(np.pi/2, 0,0)),
            rospy.Time.now(),  
            "/nexus"+str(self.robotNumber)+"/wheel4",
            "/nexus"+str(self.robotNumber)+"/base_link",
            )

    def move2d(self, goal):
        vel = getMAPFVel(self.pos, goal, real=False)
        vel.linear.x = 10*vel.linear.x
        vel.linear.y = 10*vel.linear.y
        # print(vel)
        self.velPub.publish(vel)
        return self.pos


    def getVel(self, goal):
        vel  = Twist()
        pose = self.pos
        
        diff_x = goal.x-pose.x
        diff_y = goal.y-pose.y

        # print(diff_x, diff_y)
        
        linearOffset = l2(pose, goal) #Distance to goal

        # if linearOffset>TrafficParameters.REACH_THRESHOLD:
        if True:

            angularOffset = np.arctan2(diff_y,diff_x)
            vel.angular.z = -(-angularOffset+pose.z) #Relative offset
            vel.angular.z = ((vel.angular.z+np.pi)%(2*np.pi))-np.pi #Normalize b/w -pi to pi

            if(abs(vel.angular.z)<=TrafficParameters.ANG_THRESHOLD):
                if abs(linearOffset)>TrafficParameters.FAR_THRESHOLD:
                    vel.linear.x = TrafficParameters.REAL_FAST_VEL*(linearOffset/abs(linearOffset))
                else:
                    vel.linear.x = (((TrafficParameters.REAL_FAST_VEL-TrafficParameters.REAL_SLOW_VEL)*(linearOffset-TrafficParameters.REACH_THRESHOLD))/(TrafficParameters.FAR_THRESHOLD-TrafficParameters.REACH_THRESHOLD))+TrafficParameters.REAL_SLOW_VEL
                
            if(vel.angular.z!=0):
                if(abs(vel.angular.z)>TrafficParameters.ANG_THRESHOLD):
                    vel.angular.z = TrafficParameters.MAX_ANG_VEL*vel.angular.z/abs(vel.angular.z)
        self.velPub.publish(vel)

    def move(self, pose, goal):
        # print("Goal:", goal)
        # print("Pose:", self.pos)
        self.getVel(goal)
        return self.pos
        
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
    
def rk4_step(pose, v, omega, dt):
    x, y, theta = pose

    def f(p, v, omega):
        x, y, theta = p
        return (
            v * math.cos(theta),
            v * math.sin(theta),
            omega
        )

    # k1
    dx1, dy1, dtheta1 = f((x, y, theta), v, omega)

    # k2
    dx2, dy2, dtheta2 = f((x + 0.5 * dt * dx1, y + 0.5 * dt * dy1, theta + 0.5 * dt * dtheta1), v, omega)

    # k3
    dx3, dy3, dtheta3 = f((x + 0.5 * dt * dx2, y + 0.5 * dt * dy2, theta + 0.5 * dt * dtheta2), v, omega)

    # k4
    dx4, dy4, dtheta4 = f((x + dt * dx3, y + dt * dy3, theta + dt * dtheta3), v, omega)

    # Integrate
    x_new = x + (dt / 6.0) * (dx1 + 2*dx2 + 2*dx3 + dx4)
    y_new = y + (dt / 6.0) * (dy1 + 2*dy2 + 2*dy3 + dy4)
    theta_new = theta + (dt / 6.0) * (dtheta1 + 2*dtheta2 + 2*dtheta3 + dtheta4)

    # Normalize theta
    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))

    return (x_new, y_new, theta_new)

def compute_velocity(robot_pose, waypoint, heading_pid, dt):
    x, y, theta = robot_pose.x, robot_pose.y, robot_pose.z
    x_g, y_g = waypoint.x, waypoint.y

    # Compute angle and distance to goal
    dx = x_g - x
    dy = y_g - y
    distance = math.hypot(dx, dy)
    
    if(distance < TrafficParameters.REACH_THRESHOLD):
        return 0, 0
    
    goal_theta = math.atan2(dy, dx)

    # Compute heading error
    heading_error = goal_theta - theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize [-pi, pi]

    # Use PID for angular velocity
    angular_velocity = heading_pid.update(heading_error, dt)

    # Linear velocity proportional to distance (with a cap)
    max_speed = TrafficParameters.VIRTUAL_MAX_VEL  # m/s
    linear_velocity = max(min(TrafficParameters.SPEED_MULTIPLIER * distance, max_speed), TrafficParameters.VIRTUAL_MIN_VEL)

    return linear_velocity, angular_velocity

class TRAFFIC_VirtualController:
    
    def __init__(self, robotNumber) -> None:
        self.vel = Twist()
        self.br = tf.TransformBroadcaster()
        self.robotNumber = robotNumber
        self.PID = PID(2.0, 0.0, 0.2)  # Example PID coefficients

        
    def move(self, pose, goal):
        self.pos = copy.deepcopy(pose)
        v, omega = compute_velocity(pose, goal, self.PID, 0.05)

        logDebugData("Robot: "+str(self.robotNumber)+", Vel: "+str(v)+", "+str(omega)+"\n", self.robotNumber)
        
        self.pos.x, self.pos.y, self.pos.z = rk4_step((self.pos.x, self.pos.y, self.pos.z), v, omega, 0.05)

        self.br.sendTransform(
        (self.pos.x, self.pos.y, 0.075),
        quaternion_from_euler(0,0, self.pos.z),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/base_footprint",
        "/map"
        )
        self.br.sendTransform(
        (0.0, 0.0, 0.0),
        (0,0,0,1),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/base_link",
        "/nexus"+str(self.robotNumber)+"/base_footprint",
        )
        self.br.sendTransform(
        (0.075, 0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel1",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (-0.075, -0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel2",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (0.075, -0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel3",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        self.br.sendTransform(
        (-0.075, 0.09, -0.03),
        (quaternion_from_euler(np.pi/2, 0,0)),
        rospy.Time.now(),  
        "/nexus"+str(self.robotNumber)+"/wheel4",
        "/nexus"+str(self.robotNumber)+"/base_link",
        )
        
        return self.pos  