import colorsys
from parameters import DriverParameters

def int_to_rgba(value: int):
    hue = (value * 0.618033988749895) % 1  # Using the golden ratio to generate well-distributed hues
    rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.95)  # High saturation and value for vibrant colors
    rgba = (*rgb, 1.0)
    # rgba_255 = tuple(int(255 * c) for c in rgba)
    return rgba

def rviz_write(PATH, HYBRID_ROBOT_COUNT):
    # Part 1: Write a non changing piece for an rviz file
    read = open(PATH+"/rviz/part1", "r")
    w = open(PATH+"/rviz/dynamic_rviz.rviz","w")
    temp = read.readlines()

    w.writelines(temp)
    read.close()

    # Part 2: Read the part that we will change to visualize the number of robots we need
    read = open(PATH+"/rviz/agent", "r")
    temp = read.readlines()
    read.close()

    # Write for part 2
    for i in range(HYBRID_ROBOT_COUNT):
        for j in temp:
            k = j.replace("nexus0", 'nexus'+str(i))
            w.writelines(k)
    if(DriverParameters.SCENARIO == 0):
        read = open(PATH+"/rviz/goal", "r")
        temp = read.readlines()
        read.close()

        # Write for part 2
        for i in range(HYBRID_ROBOT_COUNT):
            for j in temp:
                k = j.replace("nexus0", 'nexus'+str(i))
                w.writelines(k)

    # Part 3: Write some more constant lines for rviz files
    read = open(PATH+"/rviz/part2", "r")
    temp = read.readlines()  
    w.writelines(temp)
    read.close()
    w.close() 

