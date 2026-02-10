cmake_minimum_required(VERSION 3.0.2)
project(ur_ros_cartesian_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  ur_kinematics
  ur_description
  message_generation
  xacro
  robot_state_publisher
  joint_state_publisher
)

## Generate messages in the 'srv' folder
 add_service_files(
   FILES
   position.srv
   cartesian_point.srv
 )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   RobotiqVacuumGrippers_robot_output.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package( 
CATKIN_DEPENDS ur_kinematics
)

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/ur_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ur_joy_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ur_point_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
