from discreteUtil import *

with open('../scenarios/MAPF/Sortation/sortation_large.map', 'r') as file:
    map = file.readlines()

# Print the first few lines to verify
print("".join(map[:4]))

world = np.zeros((140,500))
for i, line in enumerate(map[4:]):
    for j, char in enumerate(line.strip()):
        if char == '@':
            world[i, j] = -1
        else:
            world[i, j] = 0
    # print(i, line)
    
plt.imsave("/media/marmot/Backup/catkin_ws/src/Tanishq_MAPF/scenarios/MAPF/Sortation/temp.png",renderWorld(50, world))
    