#!/usr/bin/env python3
import rospy
from marmot.msg import TaskBroadcast
from unifiedCommsUtil import TasksPubAck

rospy.init_node('newRun')
rospy.sleep(1)

allTasks = []

temp = TaskBroadcast()
temp.taskID = 1
temp.robotID = 3
temp.action = 0

temp.start.x = 50   
temp.start.y = 0

temp.goal.x = 80
temp.goal.y = 20
temp.allFrom = []
temp.allTo = []

tasksPubAck = TasksPubAck(ACK_TIMEOUT=0.2)

tasksPubAck.publishData([temp])

temp = TaskBroadcast()
temp.robotID = 3
temp.taskID = -1
tasksPubAck.publishData([temp])