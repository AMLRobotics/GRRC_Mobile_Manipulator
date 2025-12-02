cmake_minimum_required(VERSION 3.0.2)
project(vision_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'srv' folder
## add_service_files(
##   FILES
## )

## Generate messages in the 'srv' folder
add_message_files(
  FILES
  prediction_data.msg
  PlaneResults.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   src/realsense_point_cloud.py
   src/calibrate.py
   src/test_data_collect.py
   src/detection_apriltag.py
   src/gripper_tip_tf.py
   src/gripping_point_tf.py
   src/hand_eye_tf.py
   src/tool0_destination_tf.py
   src/prediction_model.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

