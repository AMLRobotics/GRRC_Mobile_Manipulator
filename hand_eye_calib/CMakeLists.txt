cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calib)

find_package(catkin REQUIRED COMPONENTS
    )

# RealSense description
catkin_package(
    )

# Install files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()