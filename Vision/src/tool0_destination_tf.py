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
    rospy.init_node('tool0_destination_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    #Robot tool effector to gripper tip tf
    end2end = geometry_msgs.msg.TransformStamped()

    end2end.header.stamp = rospy.Time.now()
    end2end.header.frame_id = "gripping_point"
    end2end.child_frame_id = "tool0_destination"

    end2end.transform.translation.x = 0.0
    end2end.transform.translation.y = 0.0
    end2end.transform.translation.z = -0.174

    end2end.transform.rotation.x = 0
    end2end.transform.rotation.y = 0
    end2end.transform.rotation.z = 0
    end2end.transform.rotation.w = 1

    broadcaster.sendTransform(end2end)
    rospy.spin()