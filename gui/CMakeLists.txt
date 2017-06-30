set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")  # Kan visst lage uventede problemer
cmake_minimum_required(VERSION 2.8.3)
project(gui)


FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

SET(gui_SOURCES 
	src/main.cpp 
	src/mainwindow.cpp 
	qcustomplot.cpp
	src/realtimeplot.cpp
	src/satelliteview.cpp
	src/obstacleinterface.cpp
	src/posUpdateHandler.cpp
	src/RVIZ_Interface.cpp
	src/detectedTargetVisualizer.cpp
	../environment/src/gpsTools.cpp
	)
SET(gui_FORMS 
	src/mainwindow.ui
	)
SET(gui_HEADERS 
	src/mainwindow.h 
	qcustomplot.h
	src/realtimeplot.h
	src/satelliteview.h
	src/obstacleinterface.h
	src/watchDog.h
	src/posUpdateHandler.h
	src/RVIZ_Interface.h
	src/detectedTargetVisualizer.h
	../environment/src/gpsTools.h
	)

QT4_WRAP_CPP(gui_HEADERS_MOC ${gui_HEADERS})
QT4_WRAP_UI(gui_FORMS_HEADERS ${gui_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs
	environment
	simulator_messages
	message_generation
)


## Declare a catkin package
catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(gui src/main.cpp)
ADD_EXECUTABLE(gui
	src/main.cpp
    ${gui_SOURCES}
    ${gui_HEADERS_MOC}
    ${gui_FORMS_HEADERS}
)

TARGET_LINK_LIBRARIES(gui ${QT_LIBRARIES})
target_link_libraries(gui ${catkin_LIBRARIES})
# add_dependencies(gui gui_generate_messages_cpp)

