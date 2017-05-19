cmake_minimum_required(VERSION 3.5.1)
set(PROJECT_NAME simulator_messages)
project(${PROJECT_NAME})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
)

add_definitions(-std=c++11 -Wall -g)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES 
  Environment.msg 
  Gps.msg
  ActuatorMessage.msg
  AIS.msg
  detectedTarget.msg
)





## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	CATKIN_DEPENDS message_runtime std_msgs
)


###########
## Build ##
########### 

include_directories(  
	include/${PROJECT_NAME}
  	${catkin_INCLUDE_DIRS}
)


