cmake_minimum_required(VERSION 2.8.3)
project(easondrone_mission)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
        mavros
        std_srvs
        tf2_ros
        tf2_eigen
  nav_msgs
  std_msgs
        geometry_msgs
        sensor_msgs
  mavros_msgs
        easondrone_msgs
        quadrotor_msgs
        message_generation
)
##  System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
        mavros_msgs
        easondrone_msgs
        quadrotor_msgs
        nav_msgs
)

catkin_package(
  CATKIN_DEPENDS  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/EasonDrone/Modules/common/include
)

###############################
##      声明可执行cpp文件     ##
###############################
#add_executable(gimbal_control_circle gimbal_control/gimbal_control_circle.cpp)
#add_dependencies(gimbal_control_circle ${PROJECT_NAME}_gencpp)
#target_link_libraries(gimbal_control_circle ${catkin_LIBRARIES})
#
#add_executable(gimbal_control_demo gimbal_control/gimbal_control_demo.cpp)
#add_dependencies(gimbal_control_demo ${PROJECT_NAME}_gencpp)
#target_link_libraries(gimbal_control_demo ${catkin_LIBRARIES})
#
#add_executable(gimbal_control gimbal_control/gimbal_control.cpp)
#add_dependencies(gimbal_control ${PROJECT_NAME}_gencpp)
#target_link_libraries(gimbal_control ${catkin_LIBRARIES})
#
#add_executable(gimbal_control_vel gimbal_control/gimbal_control_vel.cpp)
#add_dependencies(gimbal_control_vel ${PROJECT_NAME}_gencpp)
#target_link_libraries(gimbal_control_vel ${catkin_LIBRARIES})
#
#add_executable(pad_tracking test/pad_tracking.cpp)
#add_dependencies(pad_tracking ${PROJECT_NAME}_gencpp)
#target_link_libraries(pad_tracking ${catkin_LIBRARIES})

add_executable(mission_cmd_pub test/mission_cmd_pub.cpp)
add_dependencies(mission_cmd_pub ${PROJECT_NAME}_gencpp)
target_link_libraries(mission_cmd_pub ${catkin_LIBRARIES})

#add_executable(turtlebot_formation formation_flight/turtlebot_formation.cpp)
#add_dependencies(turtlebot_formation ${PROJECT_NAME}_gencpp)
#target_link_libraries(turtlebot_formation ${catkin_LIBRARIES})
#
#add_executable(circle_crossing circle_crossing/circle_crossing.cpp)
#add_dependencies(circle_crossing ${PROJECT_NAME}_gencpp)
#target_link_libraries(circle_crossing ${catkin_LIBRARIES})

#add_executable(turtlebot_follow formation_flight/turtlebot_follow.cpp)
#add_dependencies(turtlebot_follow ${PROJECT_NAME}_gencpp)
#target_link_libraries(turtlebot_follow ${catkin_LIBRARIES})

add_executable(planning_mission planning_mission/planning_mission.cpp)
add_dependencies(planning_mission ${PROJECT_NAME}_gencpp)
target_link_libraries(planning_mission ${catkin_LIBRARIES})

add_executable(pub_goal planning_mission/pub_goal.cpp)
add_dependencies(pub_goal ${PROJECT_NAME}_gencpp)
target_link_libraries(pub_goal ${catkin_LIBRARIES})

add_executable(waypoint_tracking waypoint_tracking/waypoint_tracking.cpp)
add_dependencies(waypoint_tracking ${PROJECT_NAME}_gencpp)
target_link_libraries(waypoint_tracking ${catkin_LIBRARIES})

#add_executable(autonomous_landing autonomous_landing/autonomous_landing.cpp)
#add_dependencies(autonomous_landing ${PROJECT_NAME}_gencpp)
#target_link_libraries(autonomous_landing ${catkin_LIBRARIES})
#
#add_executable(autonomous_landing_aruco autonomous_landing/autonomous_landing_aruco.cpp)
#add_dependencies(autonomous_landing_aruco ${PROJECT_NAME}_gencpp)
#target_link_libraries(autonomous_landing_aruco ${catkin_LIBRARIES})
#
#add_executable(object_tracking object_tracking/object_tracking.cpp)
#add_dependencies(object_tracking ${PROJECT_NAME}_gencpp)
#target_link_libraries(object_tracking ${catkin_LIBRARIES})

#add_executable(indoor_competition indoor_competition/indoor_competition.cpp)
#add_dependencies(indoor_competition ${PROJECT_NAME}_gencpp)
#target_link_libraries(indoor_competition ${catkin_LIBRARIES})

#add_executable(fake_detectioninfo test/fake_detectioninfo.cpp)
#add_dependencies(fake_detectioninfo ${PROJECT_NAME}_gencpp)
#target_link_libraries(fake_detectioninfo ${catkin_LIBRARIES})

#add_executable(formation_getpose formation/formation_getpose.cpp)
#add_dependencies(formation_getpose ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_getpose ${catkin_LIBRARIES})
#
#add_executable(formation_state formation/formation_state.cpp)
#add_dependencies(formation_state ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_state ${catkin_LIBRARIES})
#
#add_executable(formation_move formation/formation_move.cpp)
#add_dependencies(formation_move ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_move ${catkin_LIBRARIES})
#
#add_executable(formation_change formation/formation_change.cpp)
#add_dependencies(formation_change ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_change ${catkin_LIBRARIES})
#
#add_executable(formation_setmode formation/formation_setmode.cpp)
#add_dependencies(formation_setmode ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_setmode ${catkin_LIBRARIES})
#
#add_executable(formation_control formation/formation_control.cpp)
#add_dependencies(formation_control ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_control ${catkin_LIBRARIES})
#
#add_executable(formation_square formation/formation_square.cpp)
#add_dependencies(formation_square ${PROJECT_NAME}_gencpp)
#target_link_libraries(formation_square ${catkin_LIBRARIES})

add_executable(pub_goal_from_qgc planning_mission/pub_goal_from_qgc.cpp)
add_dependencies(pub_goal_from_qgc ${PROJECT_NAME}_gencpp)
target_link_libraries(pub_goal_from_qgc ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
