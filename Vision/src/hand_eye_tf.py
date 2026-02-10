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
    rospy.init_node('hand_eye_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    #Robot tool effector to gripper tip tf
    end2end = geometry_msgs.msg.TransformStamped()

    end2end.header.stamp = rospy.Time.now()
    end2end.header.frame_id = "ur_tool0"
    end2end.child_frame_id = "camera_color_optical_frame"

    end2end.transform.translation.x = -0.0769723
    end2end.transform.translation.y = 0.0389689
    end2end.transform.translation.z = 0.0609386

    end2end.transform.rotation.x = 0.00252123
    end2end.transform.rotation.y = 0.000157021
    end2end.transform.rotation.z = -0.716313
    end2end.transform.rotation.w = 0.697774

    broadcaster.sendTransform(end2end)
    rospy.spin()