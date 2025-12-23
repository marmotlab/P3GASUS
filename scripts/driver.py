#!/usr/bin/env python3

import rospy
import numpy as np
import rospkg
import cv2
import matplotlib.pyplot as plt
from util import *
import roslaunch
from parameters import *
from discreteUtil import *
from roslaunch.core import Node
import xacro
import os
from datetime import datetime

rospack = rospkg.RosPack()
rospy.init_node('driverNode')

PATH = rospack.get_path('marmot')
rospy.set_param("PATH", PATH)

realRobotCount = len(DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING)

if DriverParameters.DEBUG:
    timestamp = datetime.now().strftime("%d.%m.%y.%H.%M.%S")
    debug_folder = PATH + "/debug/" + timestamp
    os.makedirs(debug_folder, exist_ok=True)
    rospy.set_param("debug_folder", debug_folder)

if DriverParameters.SCENARIO == 0:
    
    rospy.set_param("HYBRID_ROBOT_COUNT", DriverParameters.HYBRID_ROBOT_COUNT)
    rospy.set_param("OPTITRACK_TO_MAP_SHIFT", DriverParameters.OPTITRACK_TO_MAP_SHIFT)
    rospy.set_param("RESOLUTION", DriverParameters.RESOLUTION)
    
    if MAPFParameters.IMITATE:
        MAPFParameters.WORLD =  np.load(PATH+MAPFParameters.IMITATE_FOLDER+'WORLD.npy')
        if MAPFParameters.WORLD.shape[0] > MAPFParameters.WORLD_SIZE[0] or MAPFParameters.WORLD.shape[1] > MAPFParameters.WORLD_SIZE[1]:
            rospy.logwarn("WORLD is larger than MAPFParameters.WORLD_SIZE. Truncating WORLD to fit.")
            MAPFParameters.WORLD = MAPFParameters.WORLD[:MAPFParameters.WORLD_SIZE[0], :MAPFParameters.WORLD_SIZE[1]]
            
        MAPFParameters.STARTS  = np.load(PATH+MAPFParameters.IMITATE_FOLDER+'STARTS.npy')
        MAPFParameters.STARTS = MAPFParameters.STARTS[MAPFParameters.IMITATION_LIST]
        
        if MAPFParameters.STARTS.shape[0] > DriverParameters.HYBRID_ROBOT_COUNT:
            MAPFParameters.STARTS = MAPFParameters.STARTS[:DriverParameters.HYBRID_ROBOT_COUNT]
        
        # goals = np.load(PATH+MAPFParameters.IMITATE_FOLDER+'GOALS.npy')
        # goals = goals[MAPFParameters.IMITATION_LIST]
        
        # rospy.set_param('futureGoals', goals.tolist())
        
    else:
        MAPFParameters.WORLD = np.zeros((MAPFParameters.WORLD_SIZE[0], MAPFParameters.WORLD_SIZE[1]), dtype=np.int64)
        MAPFParameters.STARTS = list()
        while(len(MAPFParameters.STARTS)!=DriverParameters.HYBRID_ROBOT_COUNT):
            MAPFParameters.STARTS.append(getFreeCell(MAPFParameters.STARTS, MAPFParameters.WORLD))

    rospy.set_param('WORLD', MAPFParameters.WORLD.tolist())
    # print("STARTS: ", MAPFParameters.STARTS)
    # print("Transformed Starts: ", [getCoord((x[0], x[1]), DriverParameters.RESOLUTION) for x in MAPFParameters.STARTS])

    # Write world to rviz    
    temp = np.copy(MAPFParameters.WORLD)
    temp[temp==0] = 255
    temp[temp==1] = 0
    
    cv2.imwrite(PATH+"/map/dynamic.pgm",np.rot90(temp))
    

      
elif DriverParameters.SCENARIO == 1:   
    rospy.set_param("HYBRID_ROBOT_COUNT", DriverParameters.HYBRID_ROBOT_COUNT)
    rospy.set_param("OPTITRACK_TO_MAP_SHIFT", DriverParameters.OPTITRACK_TO_MAP_SHIFT)
    rospy.set_param("RESOLUTION", DriverParameters.RESOLUTION) 
    
    
rviz_write(PATH, DriverParameters.HYBRID_ROBOT_COUNT)
rospy.sleep(1)

# Start an instance of ROSLAUNCH API 
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

launch_files = list()
## this starts our barebone environment and initilizes a list which stores all files we need to launch
cli_args = [PATH+'/launch/base.launch', 
    'MapName:=' + ("dynamic" if DriverParameters.SCENARIO == 0 else "trafficMap"), 
    'OptitrackToMapTransform_X:='+str(DriverParameters.OPTITRACK_TO_MAP_SHIFT[0]), 
    'OptitrackToMapTransform_Y:='+str(DriverParameters.OPTITRACK_TO_MAP_SHIFT[1]), 
    'LaunchOptitrack:='+str(len(DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING)!=0),
    'server:='+DriverParameters.VRPN_IP]

roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
roslaunch_args = cli_args[1:]
launch_files.append([(roslaunch_file, roslaunch_args)])

parent = {}
for id, val in enumerate(launch_files):
    print(val)
    parent[id] = roslaunch.parent.ROSLaunchParent(uuid, val)
    parent[id].start()
    
for ID in range (DriverParameters.HYBRID_ROBOT_COUNT): # For each robot of that type (given by the count in SIMULATION.yaml)

    # Get coordinates for the start node of the robot
    if(DriverParameters.SCENARIO == 0):
        start = getCoord(MAPFParameters.STARTS[ID],DriverParameters.RESOLUTION)
        
    else:
        start = (0,0)
    
    cols = int_to_rgba(ID)
    
    
    robot_description = xacro.process_file(
        PATH+"/urdf/robot.xacro",
        mappings={
        "robot_color_r": str(cols[0]),  # Red
        "robot_color_g": str(cols[1]),  # Green
        "robot_color_b": str(cols[2]),  # Blue
        "robot_color_a": str(cols[3])   # Alpha (transparency)
        }
    ).toxml()
    
    
    if(DriverParameters.SCENARIO == 0):
        goal_description = xacro.process_file(
            PATH+"/urdf/goal.xacro",
            mappings={
            "robot_color_r": str(cols[0]),  # Red
            "robot_color_g": str(cols[1]),  # Green
            "robot_color_b": str(cols[2]),  # Blue
            "robot_color_a": str(cols[3])   # Alpha (transparency)
            }
        ).toxml()
        rospy.set_param("/nexus"+str(ID)+"_goal/robot_description", goal_description)
    # print("Start", start)
    rospy.set_param('StartX/robot'+str(ID), float(start[0]))
    rospy.set_param('StartY/robot'+str(ID), float(start[1]))
    rospy.set_param("/nexus"+str(ID)+"/robot_description", robot_description)
    

data = list(range(DriverParameters.HYBRID_ROBOT_COUNT))
np.random.shuffle(data)
partitions = np.array_split(data, DriverParameters.GROUP_COUNT)
partitioned_lists = [list(map(int, part)) for part in partitions]

for i in range(DriverParameters.GROUP_COUNT):
    rospy.set_param("/group_"+str(i)+"/list_of_robots", partitioned_lists[i])
    launch.launch(Node(package='marmot', node_type='groupController.py', name='group_controller', namespace="group_"+str(i) , output='screen'))


for i in DriverParameters.VIRTUAL_TO_REAL_ROBOT_MAPPING.values():    
    rospy.set_param("/autosshNode_"+str(i)+"/ID", i)
    launch.launch(Node(package='marmot', node_type='autossh.py', name='autoSSH', namespace="autosshNode_"+str(i) , output='screen'))

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
        
rospy.spin()


