<<<<<<< HEAD
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")  # Kan visst lage uventede problemer
cmake_minimum_required(VERSION 2.8.3)
project(environment)

SET(environment_SOURCES 
	src/environment.cpp 
	src/obstacleControl.cpp
=======
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
	src/gps.cpp
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
	)

SET(environment_HEADERS 
	src/obstacleControl.h
<<<<<<< HEAD
	)

=======
	src/sensorsim.h
	src/gps.h
	)

QT4_WRAP_CPP(environment_HEADERS_MOC ${environment_HEADERS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
<<<<<<< HEAD
=======
	simulator_messages
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
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

<<<<<<< HEAD
target_link_libraries(environment ${catkin_LIBRARIES})
add_dependencies(environment environment_generate_messages_cpp)
=======
add_dependencies(environment environment_generate_messages_cpp)
target_link_libraries(environment ${QT_LIBRARIES})
target_link_libraries(environment ${catkin_LIBRARIES})



>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
