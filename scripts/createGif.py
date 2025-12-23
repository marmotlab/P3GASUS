from discreteUtil import *

with open('../scenarios/MAPF/Warehouse/warehouse_large.map', 'r') as file:
    warehouse_map = file.readlines()

# Print the first few lines to verify
print("".join(warehouse_map[:4]))

## Create the world
world = np.zeros((140,500))
for i, line in enumerate(warehouse_map[4:]):
    for j, char in enumerate(line.strip()):
        if char == '@':
            world[i, j] = -1
        else:
            world[i, j] = 0
            
np.save('../scenarios/MAPF/Warehouse/warehouse_large.npy', world)

#load the actions and starts
starts = np.load('../scenarios/MAPF/Warehouse/starts.npy')
actions = np.load('../scenarios/MAPF/Warehouse/actions.npy')
goalAssignment = np.load('../scenarios/MAPF/Warehouse/goals.npy')

###Optionally load the world
# world = np.load('../scenarios/MAPF/Warehouse/warehouse_large.npy')

position = np.zeros((*actions.shape, 2), dtype=int)
goal = np.zeros_like(position)

position[:, 0, :] = starts


def get_next_position(position, action, actionDict = {0:np.array([0,0]), 1:np.array([1,0]), 2:np.array([0,1]), 3:np.array([-1,0]), 4:np.array([0,-1])}):
    return np.array(position + actionDict[action])

for i in range(actions.shape[0]):
# for i in range(20):
    k = 0
    for j in range(actions.shape[1]):
        if(j != actions.shape[1]-1):   
            position[i, j+1, :] = get_next_position(position[i, j, :], actions[i, j])
        if(j == goalAssignment[i, k+1, 2]):
            k+= 1
        goal[i, j, :] = goalAssignment[i, k, :2]
        
        
np.save('../scenarios/MAPF/Warehouse/positions.npy', position)
np.save('../scenarios/MAPF/Warehouse/goalPositions.npy', goal)
        