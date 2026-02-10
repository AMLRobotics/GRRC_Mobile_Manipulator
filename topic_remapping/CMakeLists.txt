cmake_minimum_required(VERSION 3.0.2)
project(topic_remapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  geometry_msgs
  autoware_msgs
  rospy
  runtime_manager
)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder

## Generate added messages and services with any dependencies listed here


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Add the new executable target
add_executable(remapping_topic src/remapping_topic.cpp)
add_executable(detect_stop src/detect_stop.cpp)

target_include_directories(remapping_topic PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(detect_stop PRIVATE ${catkin_INCLUDE_DIRS})

# Link the executable against necessary libraries (roscpp, std_msgs, autoware_msgs, etc.)
target_link_libraries(detect_stop ${catkin_LIBRARIES}) 
target_link_libraries(remapping_topic ${catkin_LIBRARIES})

add_dependencies(remapping_topic ${catkin_EXPORTED_TARGETS})
add_dependencies(detect_stop ${catkin_EXPORTED_TARGETS})

install(TARGETS remapping_topic detect_stop
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

