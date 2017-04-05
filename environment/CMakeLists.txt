set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")  # Kan visst lage uventede problemer
cmake_minimum_required(VERSION 2.8.3)
project(environment)

SET(environment_SOURCES 
	src/environment.cpp 
	src/obstacleControl.cpp
	)

SET(environment_HEADERS 
	src/obstacleControl.h
	)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	message_generation
)

## Declare ROS messages and services
 add_message_files(
   DIRECTORY
   msg
   FILES
   obstacleUpdate.msg
   obstacleCmd.msg
 )

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(environment
	src/environment.cpp
    ${environment_SOURCES}
    ${environment_HEADERS_MOC}
)

target_link_libraries(environment ${catkin_LIBRARIES})
add_dependencies(environment environment_generate_messages_cpp)
