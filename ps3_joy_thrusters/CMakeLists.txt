cmake_minimum_required(VERSION 3.5.1)
set(PROJECT_NAME ps3_joy_thrusters)
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


add_message_files(
  FILES 
  ActuatorMessage.msg
)

catkin_package(
	CATKIN_DEPENDS std_msgs
)

include_directories(  
	include/${PROJECT_NAME}
  	${catkin_INCLUDE_DIRS}
  	${GTEST_INCLUDE_DIRS}
)

add_executable(ps3_joy_thrusters src/main.cpp)
target_link_libraries(ps3_joy_thrusters ${catkin_LIBRARIES})
