import numpy as np
import matplotlib.pyplot as plt
import copy
import networkx as nx
import networkx.algorithms.isomorphism as iso
import lacam
# from util import *
import imageio

from marmot.msg import TaskBroadcast, TaskAck
import cv2
import math
import time
import os
import colorsys
class Position:
    def __init__(self) -> None:
        # self.taskList = list()   # ordered list of tasks in precenence with time
        self.robotDict = dict()   # robot->list of tasks

    def __repr__(self) -> str:
        asd = "\n"
        for i in self.__dir__():
            if not i.startswith('__'):
                asd+=i
                asd+=": "
                asd+=str(getattr(self,i))
                asd+=", "

        return asd

class Task:
    __actionDict__ = {0:np.array([0,0]), 1:np.array([1,0]), 2:np.array([0,1]), 3:np.array([-1,0]), 4:np.array([0,-1])}

    def __init__(self, tid, rid, start, action, time) -> None:
        self.taskID = tid
        self.robotID = rid
        self.action = action
        self.startPos = np.array(start)
        self.goalPos = np.array(self.__actionDict__[action]+start)
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
  

class ADGraph:
    
    def __init__(self) -> None:
        self.graph = nx.DiGraph()
        self.taskList = dict()
        self.robotList = []

    def fileWrite(self, path):
        
        # print()
        with open(path+"/"+(type(self)).__name__+"_Graph.txt", "w") as f:
            for i in sorted(self.taskList):
                t = []
                for j in self.graph.out_edges(i):
                    t.append(j[1])
                f.write(str(i)+":"+str(sorted(t))+"\n")

        with open(path+"/"+(type(self)).__name__+"_TaskList.txt", "w") as f:
            for i in sorted(self.taskList):
                f.write(self.taskList[i].__repr__())

class NoSynchGraph(ADGraph):
    def __init__(self, taskList, startPositions):
        super().__init__()
        taskList = np.array(taskList)
        startPositions = np.array(startPositions)
        ## Create Graph Skeleton and type1 dependencies
        currentPositions = copy.deepcopy(startPositions)
        numRobots = len(startPositions)

        tId = 1
        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(taskList[rid, :]):
                t = Task(tId, rid, currentPositions[rid], task, i)
                # print(t)
                self.taskList[tId] = t
                tId+=1
                currentPositions[rid] = t.goalPos
                self.graph.add_node(t.taskID)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

class FullySynchGraph(ADGraph):
    def __init__(self, taskList, startPositions):
        super().__init__()
        taskList = np.array(taskList)
        startPositions = np.array(startPositions)
        ## Create Graph Skeleton and type1 dependencies
        currentPositions = copy.deepcopy(startPositions)
        numRobots = len(startPositions)

        tId = 1
        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(taskList[rid, :]):
                t = Task(tId, rid, currentPositions[rid], task, i)
                # print(t)
                self.taskList[tId] = t
                tId+=1
                currentPositions[rid] = t.goalPos
                self.graph.add_node(t.taskID)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        ## Create type2 dependencies

        for rid in range(numRobots):
            firstTid = self.robotList[rid].taskID
            for taskID in range(firstTid, firstTid+len(taskList[1])):
                task = self.taskList[taskID]
                
                for rid_ in range(numRobots):
                    if(rid != rid_):
                        # print(rid, rid_)
                        firstTid_ = self.robotList[rid_].taskID
                        for taskID_ in range(firstTid_, firstTid_+len(taskList[1])):
                            task_ = self.taskList[taskID_]
                            if(task_.time == task.time+1) or (np.array_equal(task.startPos, task_.goalPos) and task.time<=task_.time):
                            # # print(task.startPos, task_.goalPos)
                            # if np.array_equal(task.startPos, task_.goalPos) and task.time<=task_.time:
                            #     # print(task.taskID, task_.taskID)
                                self.graph.add_edge(task.taskID, task_.taskID)

class OriginalADG(ADGraph):
    def __init__(self, taskList, startPositions):
        super().__init__()
        taskList = np.array(taskList)
        startPositions = np.array(startPositions)
        ## Create Graph Skeleton and type1 dependencies
        currentPositions = copy.deepcopy(startPositions)
        numRobots = len(startPositions)

        tId = 1
        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(taskList[rid, :]):
                t = Task(tId, rid, currentPositions[rid], task, i)
                # print(t)
                self.taskList[tId] = t
                tId+=1
                currentPositions[rid] = t.goalPos
                self.graph.add_node(t.taskID)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        ## Create type2 dependencies

        for rid in range(numRobots):
            firstTid = self.robotList[rid].taskID
            for taskID in range(firstTid, firstTid+len(taskList[1])):
                task = self.taskList[taskID]
                
                for rid_ in range(numRobots):
                    if(rid != rid_):
                        # print(rid, rid_)
                        firstTid_ = self.robotList[rid_].taskID
                        for taskID_ in range(firstTid_, firstTid_+len(taskList[1])):
                            task_ = self.taskList[taskID_]
                            # print(task.startPos, task_.goalPos)
                            if np.array_equal(task.startPos, task_.goalPos) and task.time<=task_.time:
                                # print(task.taskID, task_.taskID)
                                self.graph.add_edge(task.taskID, task_.taskID)
                                break

class TD_ADG(ADGraph):
    def __init__(self, taskList, startPositions):
        super().__init__()

        taskList = np.array(taskList)
        startPositions = np.array(startPositions)

        positions = {}
        currentPositions = copy.deepcopy(startPositions)
        numRobots = len(taskList)
        tId = 1
        for rid in range(numRobots):
            prevTask = None
            for i, task in enumerate(taskList[rid,:]):
                t = Task(tId, rid, currentPositions[rid], task, i)
                # print(t)
                self.taskList[tId] = t
                tId+=1
                currentPositions[rid] = t.goalPos
                self.graph.add_node(t.taskID)

                if(tuple(t.goalPos) not in positions):
                    positions[tuple(t.goalPos)] = Position()
                # positions[tuple(t.goalPos)].taskList.append(t)
                if(rid not in positions[tuple(t.goalPos)].robotDict):
                    positions[tuple(t.goalPos)].robotDict[rid] = list()
                positions[tuple(t.goalPos)].robotDict[rid].append(t.taskID)
                
                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask.taskID, t.taskID)

                prevTask = t

        taskQueue = [i.taskID for i in self.robotList]

        previousTime = -1
        tasksToClear = []

        while len(taskQueue)!=0:

            tID = taskQueue.pop(0)
            t = self.taskList[tID]

            if(t.time>previousTime):
                previousTime+=1
                # Remove t from position.robotDict
                for ttc in tasksToClear:
                    t__ = self.taskList[ttc]
                    rDict = positions[tuple(t__.goalPos)].robotDict
                    rDict[t__.robotID].pop(0)
                    if(len(rDict[t__.robotID])==0):
                        rDict.pop(t__.robotID)
                tasksToClear = []

            tasksToClear.append(tID)

            if(tID+1) in self.taskList and self.taskList[tID+1].time!=0:
                taskQueue.append(tID+1)

            st = tuple(t.startPos)
            if st in positions:
                for rid_ in positions[st].robotDict.keys():
                    if(t.robotID!=rid_):
                        self.graph.add_edge(tID, self.taskList[positions[st].robotDict[rid_][0]].taskID)
            

class Reduced_TD_Graph(ADGraph):
    def __init__(self, taskList, startPositions):
        super().__init__()

        taskList = np.array(taskList)
        startPositions = np.array(startPositions)
        tasksPerRobot = len(taskList[0])
        positions = {}
        currentPositions = copy.deepcopy(startPositions)
        numRobots = len(taskList)

        for time in range(0, tasksPerRobot):
            for rid in range(numRobots):
                tid = rid*tasksPerRobot+time+1
                prevTask = tid-1 if tid-1 in self.taskList else None

                t =  Task(tid, rid, currentPositions[rid], taskList[rid, time], time)
                self.taskList[tid] = t
                currentPositions[rid] = t.goalPos
                self.graph.add_node(t.taskID)

                if(prevTask is None):
                    self.robotList.append(t)
                else:
                    self.graph.add_edge(prevTask, t.taskID)

                if(tuple(t.startPos) not in positions):
                    positions[tuple(t.startPos)] = t.taskID
                else:
                    t_ = self.taskList[positions[tuple(t.startPos)]]
                    if t.robotID != t_.robotID:
                        self.graph.add_edge(positions[tuple(t.startPos)], t.taskID-1)
                    positions[tuple(t.startPos)] = t.taskID
        
        for rid in range(numRobots):
            tid = (rid+1)*tasksPerRobot
            t = self.taskList[tid]
            if(tuple(t.goalPos) in positions and t.action!=0):
                self.graph.add_edge(positions[tuple(t.goalPos)], t.taskID)

class MinimalGraph(Reduced_TD_Graph):
    def __init__(self, taskList, startPositions):
        super().__init__( taskList, startPositions)
        dp = dict()

        for t in self.robotList:
            self.reduceGraph(t.taskID, dp)

    def reduceGraph(self, root, dp):
        if(root not in dp):
            dp[root] = list()
        
            temp = [root]

            children = [i[1] for i in self.graph.out_edges(root)]

            if(root+1) in children:
                temp+=self.reduceGraph(root+1, dp)
                children.remove(root+1)
            
            if(len(children)!=0):
                if(children[0] in temp):
                    self.graph.remove_edge(root, children[0])
                else:
                    temp+=self.reduceGraph(children[0], dp)
            
            dp[root] = temp
        if(len(dp[root])==0):
            raise Exception("Contains Cycle")
        return dp[root]

class MinimalGraphTwo(Reduced_TD_Graph):
    def __init__(self, taskList, startPositions):
        super().__init__( taskList, startPositions)
        dp = np.zeros((len(self.taskList)+2, len(self.taskList)+2))

        for t in self.robotList:
            self.reduceGraph(t.taskID, dp)

    def reduceGraph(self, root, dp):

        if(dp[root][root])==0:    
            dp[root][root]=1
            children = [i[1] for i in self.graph.out_edges(root)]

            if(root+1) in children:
                dp[root] = np.logical_or(self.reduceGraph(root+1, dp), dp[root])
                children.remove(root+1)
            
            if(len(children)!=0):
                if(dp[root][children[0]]==1):
                    self.graph.remove_edge(root, children[0])
                else:
                    dp[root] = np.logical_or(self.reduceGraph(children[0], dp), dp[root])

            dp[root] = dp[root]

        return dp[root]
    
def getActionFromPos(currentPos, nextPos):
    temp = np.subtract(nextPos,currentPos)
    if np.array_equal(temp, np.array([1,0])):
        return 1
    elif np.array_equal(temp, np.array([0,1])):
        return 2
    elif np.array_equal(temp,np.array([-1,0])):
        return 3
    elif np.array_equal(temp, np.array([0,-1])):
        return 4
    elif np.array_equal(temp, np.array([0,0])):
        return 0
    else:
        return -1
    
def verifyPaths(paths):
    NUM_AGENTS = len(paths)
    idx = max(len(i) for i in paths)

    PATHS = np.zeros((NUM_AGENTS, idx, 2))

    for i in range(NUM_AGENTS):
        for j in range(len(paths[i])):
            PATHS[i,j] = paths[i][j]
        for k in range(len(paths[i]), idx):
            PATHS[i,k] = paths[i][j]

    for i in range(idx):
        if(len(np.unique(PATHS[:,i,:], axis=0))!=NUM_AGENTS):
            return False
    
    return True

def generateWarehouse(num_block=[-1,-1], length=40, shelfSize=5,lbRatio=2/3,freeSpaceRatio=1/3, shelfWidth=1):
    if(num_block[0]!=-1):
        length = np.random.randint(low=num_block[0], high=num_block[1]+1)
    breadth = int(length/lbRatio)
    world = np.zeros((length,breadth), dtype='int64')
    # Generate shelves
    noShelves = int((breadth*(1-freeSpaceRatio))/(shelfSize+1))
    freeSpace = int((breadth - noShelves*(shelfSize+1))/2)
    for i in range(freeSpace, freeSpace+noShelves*(shelfSize+1), shelfSize+1):
        for j in range(1, shelfWidth+1):
            for k in range(j, length-1, shelfWidth+1):
                world[k,i:i+shelfSize] = -1
    return world

def getFreeCell(targetList, world):
    listOfFree = np.swapaxes(np.where(world==0), 0,1)
    np.random.shuffle(listOfFree)
    for i in listOfFree:
        if(tuple(i) in targetList):
            continue
        else:
            return tuple(i)

def int_to_rgba(value: int):
    hue = (value * 0.618033988749895) % 1  # Using the golden ratio to generate well-distributed hues
    rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.95)  # High saturation and value for vibrant colors
    rgba = (*rgb, 1.0)
    # rgba_255 = tuple(int(255 * c) for c in rgba)
    return rgba

def getCoord(i, RES=0, MAPF = True):
    if(MAPF):
        return [i[0]*RES+RES/2, i[1]*RES+RES/2]
    else:
        if(i[0]==-2 and i[1]==-2):
            return [False, False]
        return [i[0]/10, i[1]/10]

def getCenter(img, data):
    center = -np.asarray(data['origin'][:2])/data['resolution']
    print(center)

    center = -np.floor(np.asarray(data['origin'][:2])/data['resolution']).astype('int32')
    print(center)

    center = img.shape[1]-center[1], center[0]

    center = tuple(center)
    # Radius = 2
    # for i in range(-Radius,Radius+1):
    #     for j in range(-Radius, Radius+1):
    #         img[), (center[0]+j)] = 128
    return center

def getCoordinatesFromCell(cell, img, data):
    cell = np.array([cell[1], img.shape[0]-cell[0]])
    coord = cell*data['resolution']+np.array(data['origin'][:2]) - np.array([data['resolution']/2, data['resolution']/2])
    coord = np.round(coord, 2)
    return coord

def transformCell(cell = [13, 20],left = [930,920], scale = 8):
    _left = np.array(left)
    _cell = np.array(cell)
    # print(_left, _cell*scale)
    return _left+(_cell*scale)


def init_colors(n):
    """the colors of agents and goals"""

    c = {i+1:[int_to_rgba(i)[2], int_to_rgba(i)[1], int_to_rgba(i)[0]] for i in range(n)}
    
    c[0] = [1,1,1]
    c[-1] = [0,0,0]
    c[-2] = [0.5,0.5,0.5]
    return c


def getRectPoints(coord, scale, OFFSET=0):
    base = [coord[1]*scale, coord[0]*scale]
    return np.array([[base[0]+OFFSET, base[1]+OFFSET], [base[0]+scale-1-OFFSET, base[1]+OFFSET], [base[0]+scale-1-OFFSET,base[1]+scale-1-OFFSET], [base[0]+OFFSET, base[1]+scale-1-OFFSET]])    

def pixelForText(coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return (int(math.floor(base[0]+scale*1/4)), int(math.floor(base[1]+scale*3/4)))


def getCenter(coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return (int(math.floor(base[0]+scale/2)), int(math.floor(base[1]+scale/2)))

def getTriPoints( coord, scale):
    base = [coord[1]*scale, coord[0]*scale]
    return  np.array([[int(math.floor(base[0]+scale/2)), base[1]], [base[0]+scale-1,base[1]+scale-1], [base[0], base[1]+scale-1]])    

def renderWorld(scale=20, world = np.zeros(1),agents=[], goals=[], OFFSET=2):
    size = world.shape
    numAgents = len(agents)
        
    screen_height = scale*size[0]
    screen_width = scale*size[1]

    colours = init_colors(numAgents)

    scene = np.zeros([screen_height, screen_width, 3])

    for coord,val in np.ndenumerate(world):
        cv2.fillPoly(scene, pts=[getRectPoints(coord=coord, scale=scale)], color=colours[val])


    for val,coord in enumerate(goals):
        # print(val+1)
        # print(getCenter(coord=coord, scale=scale))
        cv2.circle(scene, getCenter(coord=coord, scale=scale), math.floor(scale/2)-1, colours[val+1], -1)
        cv2.putText(scene, str(val+1), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(40*(int(np.log10(val+1))+1)), (0,0,0), int(scale/20))
        # cv2.putText(scene, str(val), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(80), (0,0,0), int(scale/20))


    for val,coord in enumerate(agents):
        cv2.fillPoly(scene, pts=[getRectPoints(coord=coord, scale=scale,OFFSET=OFFSET)], color=colours[val+1])
        cv2.putText(scene, str(val+1), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(40*(int(np.log10(val+1))+1)), (0,0,0), int(scale/20))
        # cv2.putText(scene, str(val), pixelForText(coord, scale), cv2.FONT_HERSHEY_SIMPLEX,scale/(80), (0,0,0), int(scale/20))

    scene = scene*255
    scene = scene.astype(dtype='uint8')
    return scene


class PathPlanner:
    def __init__(self, WORLD, STARTS, GOALS, TIMEOUT = 300, ADG_TYPE = Reduced_TD_Graph) -> None:
        self.numAgents= len(STARTS)
        
        path = self.oneCase(WORLD, STARTS, GOALS, TIMEOUT)
        
        if(path is None):
            raise Exception("No Solution")
        actions = self.pathsToActions(path)

        self.adg = ADG_TYPE(actions, STARTS)
        
        while self.hasCycle(self.adg.graph):
            print("Found Cycle")
            path = self.oneCase(WORLD, STARTS, GOALS, TIMEOUT)
            actions = self.pathsToActions(path)
            self.adg = ADG_TYPE(actions, STARTS)
        
        print("Found Good Path")
        self.tasksToBroadcast()
        
    def tasksToBroadcast(self):
        allTasks = []
        for tID in self.adg.taskList:
            temp = TaskBroadcast()
            task = self.adg.taskList[tID]
            
            temp.taskID = task.taskID
            temp.robotID = task.robotID
            temp.action = int(task.action)
            temp.goal.x = task.goalPos[0]
            temp.goal.y = task.goalPos[1]
            
            for tID_, _ in self.adg.graph.in_edges(tID):
                # print("in", )
                task_ = self.adg.taskList[tID_]
                temp_ = TaskAck()
                
                temp_.taskIDFrom = tID_
                temp_.robotIDFrom = task_.robotID
                temp_.taskIDTo = tID
                temp_.robotIDTo = task.robotID
                
                if(temp_.robotIDFrom == temp_.robotIDTo):
                    temp_.dependencyType = 1
                elif(not isinstance(self.adg, (FullySynchGraph, NoSynchGraph)) and task.action==task_.action):
                    temp_.dependencyType = 3
                else:
                    temp_.dependencyType = 2
                    
                temp.allFrom.append(temp_)
                    
                    
            for _, tID_  in self.adg.graph.out_edges(tID):
                # print("in", )
                task_ = self.adg.taskList[tID_]
                temp_ = TaskAck()
                
                temp_.taskIDFrom = tID
                temp_.robotIDFrom = task.robotID
                temp_.taskIDTo = tID_
                temp_.robotIDTo = task_.robotID
                
                if(temp_.robotIDFrom == temp_.robotIDTo):
                    temp_.dependencyType = 1
                elif(not isinstance(self.adg, (FullySynchGraph, NoSynchGraph)) and task.action==task_.action):
                    temp_.dependencyType = 3
                else:
                    temp_.dependencyType = 2
                
                temp.allTo.append(temp_)
                
            allTasks.append(temp)
            
            self.allTasks = allTasks
        print("Tasks ready to Broadcast")

    def hasCycle(self, graph):
        try:
            nx.find_cycle(graph)
            return True
        except:
            return False
        
        

    def oneCase(self):
        return None
    
    
    def pathsToActions(self, paths):
        ACTIONS = []
        for i in range(self.numAgents):
            action = []
            for j in range(len(paths[i])-1):
                action.append(self.getActionFromPos(paths[i][j], paths[i][j+1]))
            ACTIONS.append(action)

        idx = max(len(i) for i in ACTIONS)

        temp = np.zeros((self.numAgents, idx))
        for i in range(len(ACTIONS)):
            for j in range(len(ACTIONS[i])):
                temp[i,j] = ACTIONS[i][j]

        ACTIONS = temp

        return ACTIONS
    
    def getActionFromPos(self, currentPos, nextPos):
        temp = np.subtract(nextPos,currentPos)
        if np.array_equal(temp, np.array([1,0])):
            return 1
        elif np.array_equal(temp, np.array([0,1])):
            return 2
        elif np.array_equal(temp,np.array([-1,0])):
            return 3
        elif np.array_equal(temp, np.array([0,-1])):
            return 4
        elif np.array_equal(temp, np.array([0,0])):
            return 0
        else:
            return -1
        
    def verifyPaths(self, paths):
        idx = max(len(i) for i in paths)

        PATHS = np.zeros((self.numAgents, idx, 2))

        for i in range(self.numAgents):
            for j in range(len(paths[i])):
                PATHS[i,j] = paths[i][j]
            for k in range(len(paths[i]), idx):
                PATHS[i,k] = paths[i][j]

        for i in range(idx):
            if(len(np.unique(PATHS[:,i,:], axis=0))!=self.numAgents):
                return False
        
        return True
    
    

class LACAM3(PathPlanner):
    def __init__(self, WORLD, STARTS, GOALS, TIMEOUT=1, ADG_TYPE=Reduced_TD_Graph) -> None:
        super().__init__(WORLD, STARTS, GOALS, TIMEOUT, ADG_TYPE)
        
    def oneCase(self, world, STARTS, GOALS, TIMEOUT):
        world = world.tolist()
        paths = lacam.solve(world, STARTS, GOALS, TIMEOUT)
        # print("Paths found: ", paths)
        return paths

class IMITATION(PathPlanner):
    def __init__(self, starts, actions, ADG_TYPE=Reduced_TD_Graph) -> None:
        
        self.adg = ADG_TYPE(actions, starts)
        
        # if self.hasCycle(self.adg.graph):
        #     raise Exception("Contains Cycle")
        
        print("Found Good Path")
        self.tasksToBroadcast()
        
def make_gif(images, file_name):
    """record gif"""
    imageio.mimwrite(file_name, images, subrectangles=True)
    print("wrote gif")