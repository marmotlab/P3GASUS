import pandas as pd
import numpy as np
from discreteUtil import *
from collections import Counter
NumSteps = 500
NumAgents = 200

directionDict = {'R': 2, 'D': 1, 'L': 4, 'U':3, 'W': 0}
# for episode in range(11, 20):
def resolveCycles(starts, actions, method=TD_ADG):
    adg = method(actions, starts)
    cycles = list(nx.simple_cycles(adg.graph))
    # print("Cycles: ", len(cycles))
    if(len(cycles) == 0):
        return starts, actions
    
    else:
        robot_frequency = Counter()
        for cycle in cycles:
            for node in cycle:
                robot_id = adg.taskList[node].robotID
                robot_frequency[robot_id] += 1
        
        max_robot = max(robot_frequency, key=robot_frequency.get)
        # print("Removing robot: ", max_robot)
        starts = np.delete(starts, max_robot, 0)
        actions = np.delete(actions, max_robot, 0)
        
        return resolveCycles(starts, actions)
if __name__ == "__main__":
    print("Running cycle resolution...")
    for episode in range(0, 20):
        Fname = "/media/marmot/Backup/catkin_ws/src/Tanishq_MAPF/scenarios/MAPF/log/log_"+str(episode)+".json"

        chunksize = 10**7  # Adjust the chunk size as needed

        # Initialize an empty list to store the chunks
        chunks = []

        # Read the file in chunks
        for chunk in pd.read_json(Fname, lines=True, chunksize=chunksize):
            chunks.append(chunk)

        # Concatenate all chunks into a single DataFrame
        df = pd.concat(chunks, axis=0)

        starts = np.zeros((len(df.start[0]), 2), dtype=int)

        actions = np.zeros(((NumAgents,NumSteps)), dtype=int)


        for i in range(len(df.start[0])):    
            starts[i] = df.start[0][i][:2]

        for idx,_ in np.ndenumerate(actions):
            temp = df.actualPaths[0][idx[0]].split(',')
            actions[idx] = directionDict[temp[idx[1]]]
            
        temp = np.copy(starts)
        temp2 = np.copy(actions)
        
        starts, actions = resolveCycles(temp, temp2)
        print(episode, starts.shape, actions.shape)
        
        np.save('/media/marmot/Backup/catkin_ws/src/Tanishq_MAPF/scenarios/MAPF/log/actions_'+str(episode)+'.npy', actions)
        np.save('/media/marmot/Backup/catkin_ws/src/Tanishq_MAPF/scenarios/MAPF/log/starts_'+str(episode)+'.npy', starts)
        
        # adg = TD_ADG(actions, starts)
        # print(episode)
        # print(len(list(nx.simple_cycles(adg.graph))))
