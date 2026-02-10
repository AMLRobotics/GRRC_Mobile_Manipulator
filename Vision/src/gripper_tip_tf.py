#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.msg
import ros_numpy
import tf2_ros
import tf2_msgs.msg
import std_msgs.msg
import math
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('gripper_tip_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    #AprilTag to gripping point tf 
    tag2point = geometry_msgs.msg.TransformStamped()

    tag2point.header.stamp = rospy.Time.now()
    tag2point.header.frame_id = "ur_tool0"
    tag2point.child_frame_id = "gripper_end"

    tag2point.transform.translation.x = 0
    tag2point.transform.translation.y = 0
    tag2point.transform.translation.z = 0.180

    tag2point.transform.rotation.x = 0
    tag2point.transform.rotation.y = 0
    tag2point.transform.rotation.z = 0
    tag2point.transform.rotation.w = 1

    broadcaster.sendTransform(tag2point)
    rospy.spin()