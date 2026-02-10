# syntax=docker/dockerfile:1
FROM osrf/ros:noetic-desktop-full

# Install Packages for Installing Python Packages
RUN /bin/bash -c "chmod 1777 /tmp &&\
    apt-get update -y &&\
    apt-get install -y python3-pip &&\
    apt-get install -y python3-all-dev &&\
    apt-get install -y python3-rospkg &&\
    apt-get install -y ros-noetic-desktop-full --fix-missing"

RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y build-essential &&\
    apt-get install -y libssl-dev &&\
    apt-get install -y libffi-dev &&\
    apt-get install -y libxml2-dev &&\
    apt-get install -y libxslt1-dev &&\
    apt-get install -y git &&\
    apt-get install -y zlib1g-dev"

# Install Custom Package(ur_ros_cartesian_control)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/ur_ros_cartesian_control.git src/ur_ros_cartesian_control &&\
    source /opt/ros/noetic/setup.bash"

# Create catkin workspace
# Install UR Robot Driver
RUN /bin/bash -c "apt-get update &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver &&\
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot &&\    
    sudo apt update -qq &&\
    rosdep update --include-eol-distros &&\
    rosdep install --from-paths src --ignore-src -y &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    chmod +x src/Universal_Robots_ROS_Driver/ur_robot_driver/launch/ur10_cartesian_passthrough_bringup.launch &&\
    source devel/setup.bash"

# Install Pynput Package for Keyboard Input Control
RUN python3 -m pip install pynput

# Install Cartesian Controller Packages
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs.git src/Universal_Robots_ROS_cartesian_control_msgs &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian &&\
    sudo apt update -qq &&\
    rosdep update --include-eol-distros &&\
    rosdep install --from-paths src --ignore-src -y &&\
    catkin_make &&\
    source devel/setup.bash"

# Install scipy
RUN python3 -m pip install scipy

# Install Custom Package(ur_ros_joint_control)
RUN /bin/bash -c "cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/ur_ros_joint_control.git src/ur_ros_joint_control &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

# Additional
RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y python3-tk &&\
    apt-get install -y tk-dev"

# Install CAN Driver and canlib
RUN /bin/bash -c "apt-get update -y && \
    apt-get install -y build-essential && \
    apt-get install -y linux-headers-`uname -r` && \
    apt-get install -y pkg-config && \
    apt-get install -y wget"

ADD https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest /root/
    
RUN /bin/bash -c "apt-get update -y && \
    cd ~/ && \
    mv downloads-kvaser linuxcan.tar.gz && \
    tar xvzf linuxcan.tar.gz && \
    cd ~/linuxcan && \
    apt-get install -y linux-headers-5.15.0-131-generic && \
    apt-get install -y dkms && \
    make dkms && \
    make dkms_install"

# Install Twist keaboard and joy node
RUN /bin/bash -c "cd ~/catkin_ws &&\
    git clone https://github.com/methylDragon/teleop_twist_keyboard_cpp.git src/teleop_twist_keyboard_cpp &&\
    # git clone -b melodic-devel https://github.com/ros-teleop/teleop_twist_joy src/teleop_twist_joy &&\ # only melodic
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

# Install AMR controller using CAN protocol(from FieldRo)
RUN /bin/bash -c "cd ~/catkin_ws &&\
    apt-get install -y ros-noetic-serial && \
    git clone https://github.com/Hangijun/CAN_sending src/can_sending &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

RUN /bin/bash -c "apt-get update -y &&\
    sudo apt-get install -y ros-noetic-moveit &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/moveit/moveit_calibration.git src/moveit_calibration && \
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"