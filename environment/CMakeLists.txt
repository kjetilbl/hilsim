set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")  # Kan visst lage uventede problemer
cmake_minimum_required(VERSION 2.8.3)
project(environment)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

SET(environment_SOURCES 
	src/environment.cpp 
	src/obstacleControl.cpp
	src/sensorsim.cpp
	src/navData.cpp
	src/gpsTools.cpp
	src/simObject.cpp
	)

SET(environment_HEADERS 
	src/obstacleControl.h
	src/sensorsim.h
	src/navData.h
	src/gpsTools.h
	src/simObject.h
	)

QT4_WRAP_CPP(environment_HEADERS_MOC ${environment_HEADERS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	simulator_messages
	message_generation
	tf
)

## Declare ROS messages and services
# add_message_files(
#   DIRECTORY
#   msg
#   FILES
#   obstacleUpdate.msg
#   obstacleCmd.msg
# )

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

add_dependencies(environment environment_generate_messages_cpp)
target_link_libraries(environment ${QT_LIBRARIES})
target_link_libraries(environment ${catkin_LIBRARIES})

