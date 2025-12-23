cmake_minimum_required(VERSION 3.0.2)
project(marmot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  move_base
  nav_msgs
  rospy
  sensor_msgs
  diff_drive_controller
  std_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  message_generation
)

add_message_files(
  FILES
  Ack.msg
  Reset.msg
  TaskAck.msg
  TaskBroadcast.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs   
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS gazebo_ros geometry_msgs move_base nav_msgs rospy sensor_msgs std_msgs tf roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
  INCLUDE_DIRS include
  LIBRARIES nexus_ros_force_based_move
)

###########
## Build ##
###########
add_library(nexus_ros_force_based_move include/nexus_ros_force_based_move.cpp)
target_link_libraries(nexus_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  nexus_ros_force_based_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nexus_4wd_mecanum_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
