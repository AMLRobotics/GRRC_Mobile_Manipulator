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
    rospy.init_node('gripping_point_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    #AprilTag to gripping point tf 
    tag2point = geometry_msgs.msg.TransformStamped()

    tag2point.header.stamp = rospy.Time.now()
    tag2point.header.frame_id = "Object"
    tag2point.child_frame_id = "gripping_point"

    tag2point.transform.translation.x = 0.0
    tag2point.transform.translation.y = 0.049
    tag2point.transform.translation.z = -0.003

    tag2point.transform.rotation.x = 0.7071068
    tag2point.transform.rotation.y = -0.7071068
    tag2point.transform.rotation.z = 0
    tag2point.transform.rotation.w = 0

    broadcaster.sendTransform(tag2point)
    rospy.spin()