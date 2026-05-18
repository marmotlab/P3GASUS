import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import networkx.algorithms.isomorphism as iso

from parameters import PATH_TO_P3GASUS_GRAPH_CREATION
from sklearn.neighbors import KDTree
import json
from p3gasus.msg import TaskBroadcast, TaskAck

import csv
from scipy.signal import savgol_filter
import sys

sys.setrecursionlimit(600000)
sys.path.insert(0, PATH_TO_P3GASUS_GRAPH_CREATION)
from continuousUtil import SAGE, MAGE, OriginalADG, jsonToNpy

class PathPlanner:
    def __init__(self, PATH, robotCount):
        with open(PATH+"/scenarios/Traffic/data.json", "r") as f:
            data = json.load(f)
        
        # pos,times = jsonToArr(data, 26)
        pos = jsonToNpy(data, robotCount)
        adg = MAGE(pos)
        
        allTasks = []
        for tID in adg.taskList:
            temp = TaskBroadcast()
            task = adg.taskList[tID]
            
            temp.taskID = task.taskID
            temp.robotID = task.robotID
            temp.start.x = task.startPos[0]
            temp.start.y = task.startPos[1]
            temp.goal.x = task.goalPos[0]
            temp.goal.y = task.goalPos[1]
            
            for tID_, _ in adg.graph.in_edges(tID):
                # print("in", )
                task_ = adg.taskList[tID_]
                temp_ = TaskAck()
                
                temp_.taskIDFrom = tID_
                temp_.robotIDFrom = task_.robotID
                temp_.taskIDTo = tID
                temp_.robotIDTo = task.robotID
                
                if(temp_.robotIDFrom == temp_.robotIDTo):
                        temp_.dependencyType = 1
                else:
                    temp_.dependencyType = 2    
                temp.allFrom.append(temp_)
                
                
            for _, tID_  in adg.graph.out_edges(tID):
                # print("in", )
                task_ = adg.taskList[tID_]
                temp_ = TaskAck()
                
                temp_.taskIDFrom = tID
                temp_.robotIDFrom = task.robotID
                temp_.taskIDTo = tID_
                temp_.robotIDTo = task_.robotID
                
                if(temp_.robotIDFrom == temp_.robotIDTo):
                    temp_.dependencyType = 1
                else:
                    temp_.dependencyType = 2
                
                temp.allTo.append(temp_)
            
            allTasks.append(temp)
            self.adg = adg
            self.allTasks = allTasks
            
    def smoothenCurve(self, pos):
        allPos = list()
        allTimes = list()
        for rid in range(pos.shape[0]):
            xs = [x[0] for x in pos[rid] if not (x[0]==-2 and x[1]==-2)]
            ys = [x[1] for x in pos[rid] if not (x[0]==-2 and x[1]==-2)]
            filtered_points_x = []
            filtered_points_y = []
            filtered_times = [] 

            filtered_points_x.append(xs[0])
            filtered_points_y.append(ys[0])
            filtered_times.append(1)
            for i in range(1, len(xs)):
                prev = np.array([filtered_points_x[-1], filtered_points_y[-1]])
                # prev = np.array([xs[i-1], ys[i-1]])
                curr = np.array([xs[i], ys[i]])
                # print(curr, prev)
                # print(np.linalg.norm(curr-prev, 2))
                if(np.linalg.norm(curr-prev, 2)>0.4):
                    filtered_points_x.append(xs[i])
                    filtered_points_y.append(ys[i])
                    filtered_times.append(i+1)
                    
            window_len = 7
            polyorder = 2

            # Apply Savitzky-Golay filter to x and y with a polynomial of degree 3 or higher
            x_smooth = savgol_filter(filtered_points_x, window_length=window_len, polyorder=polyorder)
            y_smooth = savgol_filter(filtered_points_y, window_length=window_len, polyorder=polyorder)
            
            temp = np.stack([x_smooth, y_smooth], axis=1)  
            allPos.append(temp)
            allTimes.append(filtered_times)
        return allPos, allTimes
