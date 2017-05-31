cmake_minimum_required(VERSION 3.5.1)
set(PROJECT_NAME controller)
project(${PROJECT_NAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  simulator_messages
)

add_definitions(-std=c++11 -Wall -g)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
	CATKIN_DEPENDS std_msgs
)

include_directories(  
	include/${PROJECT_NAME}
  	${catkin_INCLUDE_DIRS}
  	${GTEST_INCLUDE_DIRS}
)

add_executable(controller src/main.cpp src/controller.cpp src/waypointController.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
