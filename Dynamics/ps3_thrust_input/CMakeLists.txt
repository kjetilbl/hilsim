cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME ps3_thrust_input)
project(${PROJECT_NAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS std_msgs
)

include_directories(  
	include/${PROJECT_NAME}
  	${catkin_INCLUDE_DIRS}
  	${GTEST_INCLUDE_DIRS}
)

add_executable(ps3_thrust_input src/main.cpp)
target_link_libraries(ps3_thrust_input ${catkin_LIBRARIES})
