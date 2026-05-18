import numpy as np
import networkx as nx

from parameters import GRAPH_BINDINGS, PATH_TO_P3GASUS_GRAPH_CREATION
import json
from p3gasus.msg import TaskBroadcast, TaskAck

from scipy.signal import savgol_filter
from pathlib import Path
from types import SimpleNamespace
import sys

sys.setrecursionlimit(600000)
GRAPH_CREATION_PATH = Path(PATH_TO_P3GASUS_GRAPH_CREATION).expanduser().resolve()
GRAPH_BINDING_PATH = GRAPH_CREATION_PATH / "binding"
sys.path.insert(0, str(GRAPH_CREATION_PATH))
sys.path.insert(0, str(GRAPH_BINDING_PATH))
from continuousUtil import SAGE as PySAGE, MAGE as PyMAGE, OriginalADG as PyOriginalADG, jsonToNpy


def _normalize_binding_choice(value):
    binding = str(value).strip().lower()
    if binding not in {"python", "cpp"}:
        raise ValueError("GRAPH_BINDINGS must be either 'python' or 'cpp'")
    return binding


def _task_from_dict(task):
    return SimpleNamespace(
        taskID=task["taskID"],
        robotID=task["robotID"],
        time=task["time"],
        startPos=np.array(task["startPos"]),
        goalPos=np.array(task["goalPos"]),
    )


class _CppADGAdapter:
    def __init__(self, cpp_graph):
        self._cpp_graph = cpp_graph
        self.THRESH = cpp_graph.threshold()
        self.graph = nx.DiGraph()
        self.graph.add_nodes_from(cpp_graph.nodes())
        self.graph.add_edges_from(cpp_graph.edges())
        self.taskList = {
            int(task_id): _task_from_dict(task)
            for task_id, task in cpp_graph.task_list().items()
        }
        self.robotList = [_task_from_dict(task) for task in cpp_graph.robot_list()]

    def fileWrite(self, path):
        self._cpp_graph.file_write(path)


def _load_cpp_adgs():
    import p3gasus_continuous_cpp as cpp

    class OriginalADG(_CppADGAdapter):
        def __init__(self, allPositions):
            super().__init__(cpp.OriginalADG(allPositions))

    class SAGE(_CppADGAdapter):
        def __init__(self, allPositions=None):
            super().__init__(cpp.SAGE(allPositions))

    class MAGE(_CppADGAdapter):
        def __init__(self, allPositions=None, filename="temp.dat"):
            super().__init__(cpp.MAGE(allPositions))

    return SAGE, MAGE, OriginalADG


if _normalize_binding_choice(GRAPH_BINDINGS) == "cpp":
    SAGE, MAGE, OriginalADG = _load_cpp_adgs()
else:
    SAGE, MAGE, OriginalADG = PySAGE, PyMAGE, PyOriginalADG

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
