import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import networkx.algorithms.isomorphism as iso

from sklearn.neighbors import KDTree
import json
from marmot.msg import TaskBroadcast, TaskAck

import csv
from scipy.signal import savgol_filter
import sys
sys.setrecursionlimit(600000)

class ContinuousTask:

    def __init__(self, tid, rid, start, goal, time) -> None:
        self.taskID = tid
        self.robotID = rid
        self.startPos = np.array(start)
        self.goalPos = np.array(goal)
        self.time = time

    def __repr__(self) -> str:
        asd = "\n"
        for i in self.__dir__():
            if not i.startswith('__'):
                asd+=i
                asd+=": "
                asd+=str(getattr(self,i))
                asd+=", "

        return asd
    

class ContinuousADGraph:
    
    def __init__(self, positions = None) -> None:
        assert positions is not None

        self.graph = nx.DiGraph()
        self.taskList = dict()
        self.robotList = []

        self.THRESH = 1e8
        
        for i in range(positions.shape[1]):
            for r in range(positions.shape[0]):
                for r_ in range(r+1, positions.shape[0]):
                    # print(i,r,r_, self.getDistance(positions[r,i], positions[r_,i]))
                    self.THRESH = min(self.THRESH, self.getDistance(positions[r,i], positions[r_,i]))
    
        # print(self.THRESH)

    def checkCollision(self, a, b):
        return self.getDistance(a,b)<self.THRESH

    def getDistance(self, a,b):
        a = np.array(a)
        b = np.array(b)

        if(np.array_equal([-2,-2], a) or np.array_equal([-2,-2], b)):
            return 1e8

        return np.linalg.norm(a-b, 2)

    def fileWrite(self, path):
        
        # print()
        nx.write_adjlist(self.graph,path+"/"+(type(self)).__name__+"_Graph.txt")

        with open(path+"/"+(type(self)).__name__+"_TaskList.txt", "w") as f:
            for i in self.taskList:
                f.write(self.taskList[i].__repr__())

class OriginalADG(ContinuousADGraph):
    
    def __init__(self, allPositions) -> None:
        super().__init__(allPositions)
        ## Create Graph Skeleton and type1 dependencies
        numRobots = allPositions.shape[0]
        tId = 1
        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(allPositions[rid, :-1]):
                t = ContinuousTask(tId, rid, task, allPositions[rid,i+1], i)
                # print(t)
                self.taskList[tId] = t
                tId+=1
                self.graph.add_node(t.taskID)
                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        ## Create type2 dependencies

        for rid in range(numRobots):
            firstTid = self.robotList[rid].taskID
            for taskID in range(firstTid, firstTid+allPositions.shape[1]-1):
                task = self.taskList[taskID]
                if(task.startPos[0]==-2 and task.startPos[1]==-2):
                    break
                for rid_ in range(numRobots):
                    if(rid != rid_):
                        # print(rid, rid_)
                        firstTid_ = self.robotList[rid_].taskID
                        for taskID_ in range(firstTid_, firstTid_+allPositions.shape[1]-1):
                            task_ = self.taskList[taskID_]
                            # print(task.startPos, task_.goalPos)
                            if self.checkCollision(task.startPos, task_.goalPos) and task.time<=task_.time:
                                self.graph.add_edge(task.taskID, task_.taskID)
                                break

class TD_ADG(ContinuousADGraph):
    
    def __init__(self, allPositions, gridGap = None, collisionDistance = None) -> None:
        super().__init__(allPositions) 

        if(collisionDistance is None):
            self.collisionDistance = self.THRESH
        else:
            self.collisionDistance = collisionDistance

        if gridGap is None:
            self.gridGap=self.collisionDistance
        else:
            assert gridGap<=self.collisionDistance
            self.gridGap=gridGap


        ## Dependency Detection

        positions = {}

        numRobots = allPositions.shape[0]
        tId = 1

        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(allPositions[rid,:-1]):
                t = ContinuousTask(tId, rid, task, allPositions[rid,i+1], i)

                self.taskList[tId] = t
                tId+=1
                self.graph.add_node(t.taskID)

                self.addTaskToPositionMap(t, positions)


                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        # self.pos =  copy.deepcopy(positions)

        ## Dependency Filtering

        previousTime = -1
        tasksToClear = []

        taskQueue = [i.taskID for i in self.robotList]

        while len(taskQueue)!=0:
            # print("TASK LIST", taskQueue)
            tID = taskQueue.pop(0)
            t = self.taskList[tID]

            if(t.time>previousTime):
                previousTime+=1
                # #Remove t from all positions
                for ttc in tasksToClear:
                    t__ = self.taskList[ttc]
                    leftLimit,rightLimit,topLimit,bottomLimit = self.getLimits(t__.goalPos)
                    for i in range(int(topLimit), int(bottomLimit)+1):
                        for j in range(int(leftLimit), int(rightLimit)+1):
                            gridCenter = tuple([i,j])
                            assert positions[gridCenter][t__.robotID][0]==t__.taskID
                            positions[gridCenter][t__.robotID].pop(0)
                tasksToClear = []
            tasksToClear.append(tID)

            if t.startPos[0]==-2 and t.startPos[1]==-2:
                continue

            if(tID+1) in self.taskList and self.taskList[tID+1].time!=0:
                taskQueue.append(tID+1)

            #Search for and all possible dependencies for t
            leftLimit,rightLimit,topLimit,bottomLimit = self.getLimits(t.startPos)
            possibeDependencies = dict() #robotID->taskqueue


            for i in range(int(topLimit), int(bottomLimit)+1):
                for j in range(int(leftLimit), int(rightLimit)+1):
                    gridCenter = tuple([i,j])
                    for rID in positions[gridCenter]:
                        if(rID==t.robotID): ## Ignore if same robot
                            continue
                        
                        if rID not in possibeDependencies:
                            possibeDependencies[rID] = set()
                        possibeDependencies[rID] = (possibeDependencies[rID].union(positions[gridCenter][rID]))
            # print(tID, possibeDependencies)
            
            for rid in possibeDependencies:
                pos_temp = sorted(possibeDependencies[rid])
                for t_ in pos_temp:
                    if(self.checkCollision(t.startPos, self.taskList[t_].goalPos)):
                        # print("Found dependency", tID, t_)
                        # print("before edge insert", self.graph.out_edges(tID))
                        self.graph.add_edge(tID, t_)
                        # print("after edge insert", self.graph.out_edges(tID))

                        break

    def getLimits(self, pos):
        x,y = pos
        leftLimit = ((x-self.collisionDistance)//self.gridGap)+1
        rightLimit = ((x+self.collisionDistance)//self.gridGap)

        topLimit = ((y-self.collisionDistance)//self.gridGap)+1
        bottomLimit = ((y+self.collisionDistance)//self.gridGap)

        return leftLimit,rightLimit,topLimit,bottomLimit
    

    def addTaskToPositionMap(self, t:ContinuousTask, positions:dict):
        leftLimit,rightLimit,topLimit,bottomLimit = self.getLimits(t.goalPos)

        # print(topLimit, bottomLimit, rightLimit, leftLimit)

        for i in range(int(topLimit), int(bottomLimit)+1):
            for j in range(int(leftLimit), int(rightLimit)+1):
                gridCenter = tuple([i,j])

                if(gridCenter not in positions):
                    positions[gridCenter] = dict()
                # positions[gridCenter].taskList.append(t)
                if(t.robotID not in positions[gridCenter]):
                    positions[gridCenter][t.robotID] = list()
                positions[gridCenter][t.robotID].append(t.taskID)

class KDTree_ADG(ContinuousADGraph):
    def __init__(self, allPositions=None) -> None:
        super().__init__(allPositions)

        numRobots = allPositions.shape[0]
        tId = 1

        positions = []

        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(allPositions[rid,:-1]):
                t = ContinuousTask(tId, rid, task, allPositions[rid,i+1], i)

                self.taskList[tId] = t
                tId+=1
                self.graph.add_node(t.taskID)

                positions.append(t.goalPos)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        tree = KDTree(positions)  
        # self.tree = tree

        taskQueue = [i.taskID for i in self.robotList]

        while len(taskQueue)!=0:
            # print("TASK LIST", taskQueue)
            tID = taskQueue.pop(0)
            t = self.taskList[tID]

            if t.startPos[0]==-2 and t.startPos[1]==-2:
                continue

            if(tID+1) in self.taskList and self.taskList[tID+1].time!=0:
                taskQueue.append(tID+1)
            
            possibeDependencies = tree.query_radius([t.startPos], r=self.THRESH)[0]
            possibeDependencies = sorted(possibeDependencies)
            dependentRobots = [t.robotID]

            for tID__ in possibeDependencies:
                tID_ = tID__+1
                t_ = self.taskList[tID_]
                # print(tID, tID_)
                if(t_.robotID not in dependentRobots and t.time<=t_.time):
                    if(self.checkCollision(t.startPos, t_.goalPos)):
                        # print("Found")
                        self.graph.add_edge(tID, tID_)
                        dependentRobots.append(t_.robotID)

class Multi_KDTree_ADG(ContinuousADGraph):
    def __init__(self, allPositions=None) -> None:
        super().__init__(allPositions)

        numRobots = allPositions.shape[0]
        tId = 1

        trees = []

        positions = []

        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(allPositions[rid,:-1]):
                t = ContinuousTask(tId, rid, task, allPositions[rid,i+1], i)

                self.taskList[tId] = t
                tId+=1
                self.graph.add_node(t.taskID)

                positions.append(t.goalPos)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

            trees.append(KDTree(positions,leaf_size=4))
            positions = []

        taskQueue = [i.taskID for i in self.robotList]

        while len(taskQueue)!=0:

            tID = taskQueue.pop(0)
            t = self.taskList[tID]

            if t.startPos[0]==-2 and t.startPos[1]==-2:
                continue

            if(tID+1) in self.taskList and self.taskList[tID+1].time!=0:
                taskQueue.append(tID+1)
            
            for rID_ in range(numRobots):
                if rID_==t.robotID:
                    continue
                
                possibeDependencies = trees[rID_].query_radius([t.startPos], r=self.THRESH)[0]
                possibeDependencies = sorted(possibeDependencies)
                for i in possibeDependencies:
                    if i>=t.time:
                        tID_ = self.robotList[rID_].taskID+i
                        self.graph.add_edge(tID, tID_)
                        break

                    
class MinExGraph(KDTree_ADG):
    def __init__(self, allPositions=None, filename=None) -> None:
        super().__init__(allPositions)
        
        if((len(self.taskList)+2)>50000):
            assert filename is not None
            dp = np.memmap(filename, dtype='bool', mode='w+', shape=(len(self.taskList)+2, len(self.taskList)+2))
        else:
            dp = np.zeros((len(self.taskList)+2, len(self.taskList)+2), dtype=bool)

        for t in self.robotList:
            self.reduceGraph(t.taskID, dp)

        # self.dp = dp
        

    def reduceGraph(self, root, dp):

        if(dp[root][root])==0:    
            dp[root][root]=1
            children = [i[1] for i in self.graph.out_edges(root)]

            if(root+1) in children:
                dp[root] = np.logical_or(self.reduceGraph(root+1, dp), dp[root])
                children.remove(root+1)
            
            children = sorted(children, key=lambda x: self.taskList[x].time)

            while(len(children)!=0):
                if(dp[root][children[0]]==1):
                    self.graph.remove_edge(root, children[0])
                else:
                    dp[root] = np.logical_or(self.reduceGraph(children[0], dp), dp[root])
                children.pop(0)

        return dp[root]

def jsonToNpy(data, NUM_AGENTS):
    temp = []
    for i in range(NUM_AGENTS):
        temp_ = []
        for j in range(len(data['agent'+str(i)])):
            temp_.append(data['agent'+str(i)][j]["position"])
        temp.append(temp_)
    # print(len(temp), len(temp[0]))

    
    positions = np.full((len(temp), max([len(i) for i in temp]),2), -2.0)
    # print(positions.shape)

    for idx, _ in np.ndenumerate(positions):
        if(idx[1]<len(temp[idx[0]])):
            positions[idx] = temp[idx[0]][idx[1]][idx[2]]
        # else:
        #     positions[idx] = temp[idx[0]][len(temp[idx[0]])-1][idx[2]]
    return positions

class PathPlanner:
    def __init__(self, PATH, robotCount):
        with open(PATH+"/scenarios/Traffic/data.json", "r") as f:
            data = json.load(f)
        
        # pos,times = jsonToArr(data, 26)
        pos = jsonToNpy(data, robotCount)
        adg = MinExGraph(pos)
        
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
            
    
    # def jsonToArr(self, data, numAgents=83):
    #     pos = []
    #     time = []
    #     for rid in range(numAgents):
    #         listOfPos = data["agent"+str(rid)]
    #         onePos = []
    #         oneTime = []
    #         for i in listOfPos:
    #             onePos.append(i["position"])
    #             oneTime.append(i["timestep"])
    #         pos.append(onePos)
    #         time.append(oneTime)
        # return pos,time  
    
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
