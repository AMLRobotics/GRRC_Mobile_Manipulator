#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.msg
import ros_numpy
import open3d as o3d
from cv_bridge import CvBridge, CvBridgeError
from pynput import keyboard
import os
import threading
import sys
import random
import matplotlib.pyplot as plt
import ctypes
import itertools
from apriltag_ros.msg import AprilTagDetectionArray
from vision_detection.srv import *
import std_msgs.msg
import math

class VisionAT:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.joints = None
        self.cam2tag = None
        self.action = 0
        self.pos = np.array([0.0, 0.0, 0.0])
        self.c_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.img_sub = rospy.Subscriber('/tag_detections_image', sensor_msgs.msg.Image, self.image_callback)
        self.tag_info = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.imu_sub = rospy.Subscriber('/camera/imu', sensor_msgs.msg.Imu, self.imu_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_callback)
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)

        # Camera 2 EE(mm)
        self.x = 0.0325
        self.y = 0.079
        self.z = 0.11713

        # Event handler for Keyboard Input 
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    # Function for Released Keyboard Event
    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
            return False

     # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)
        # check robot is moving, if robot is moving, then do nothing.
        threading.Thread(target=self.manual_move, args=(key,)).start()
        
    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #self.detect_rectangles(self.cv_image)
            #cv2.circle(self.cv_image, (320, 240), 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.imshow('tag_image', self.cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def tag_callback(self, data):
        if data.detections[0] is not None:
            position = data.detections[0].pose.pose.pose.position
            orientation = data.detections[0].pose.pose.pose.orientation
            
            #position = position * 1000

            pitch = np.arcsin(2 * (orientation.x * orientation.z + orientation.w * orientation.y))
            yaw = np.arccos((1 - 2 * (orientation.y ** 2 + orientation.z ** 2)) / np.cos(pitch))
            roll = np.arccos((1 - 2 * (orientation.y ** 2 + orientation.x ** 2)) / np.cos(pitch))

            tag_ang = np.array([roll, pitch, yaw])
            self.tag_ang = tag_ang * (180 / math.pi)

            rospy.wait_for_service('current_cartesian_position')
            cartesian_position = rospy.ServiceProxy('current_cartesian_position', cartesian_point)
            current_position = cartesian_position(self.joints)

            self.c_pose[0] = current_position.x
            self.c_pose[1] = current_position.y
            self.c_pose[2] = current_position.z
            self.c_pose[3] = current_position.roll * 180.0 / math.pi
            self.c_pose[4] = current_position.pitch * 180.0 / math.pi
            self.c_pose[5] = current_position.yaw * 180.0 / math.pi

            self.cam2tag = np.array([self.c_pose[3] - self.tag_ang[0], self.c_pose[4] - self.tag_ang[1], self.c_pose[5] - self.tag_ang[2]])

            self.pos[0] = - position.x + self.x
            self.pos[1] = position.y - self.y - 0.049
            self.pos[2] = -(position.z - self.z + 0.012)

            print(self.c_pose[5], self.tag_ang[2])

    def imu_callback(self, data):
        temp = data

    def joint_callback(self, data):
        self.joints = np.array(data.position)
        #print(self.joints)
        temp = self.joints[0]
        self.joints[0] = self.joints[2]
        self.joints[2] = temp
        #print(self.joints)

    def manual_move(self, key):
        if key == keyboard.KeyCode(char='u'):
            command = "{},{},{},{},{},{},O".format(self.c_pose[0] + self.pos[0], self.c_pose[1] + self.pos[1], self.c_pose[2] + self.pos[2], self.c_pose[3], self.c_pose[4], self.c_pose[5])
            command = command + ' ' + "{},{},{},{},{},{},O".format(self.c_pose[0], self.c_pose[1], self.c_pose[2], self.c_pose[3], self.c_pose[4], self.c_pose[5])
            command = command + ' ' + "{},{},{},{},{},{},X".format(self.c_pose[0] + self.pos[0], self.c_pose[1] + self.pos[1], self.c_pose[2] + self.pos[2], self.c_pose[3], self.c_pose[4], self.c_pose[5])
            command = command + ' ' + "{},{},{},{},{},{},X".format(self.c_pose[0], self.c_pose[1], self.c_pose[2], self.c_pose[3], self.c_pose[4], self.c_pose[5])
       
            self.str_pub_pos.publish(command)

        if key == keyboard.KeyCode(char='i'):
            if self.action == 0:
                command = "0.0,-0.55,0.5,180.0,0.0,90.0,X"

                self.str_pub_pos.publish(command)
                self.action = 1
            
            elif self.action == 1:
                command = "{},{},{},{},{},{},X".format(self.pos[0], -0.55 + self.pos[1], 0.5, 180.0, 0.0, 90.0)
                command = command + ' ' + "{},{},{},{},{},{},O".format(self.pos[0], -0.55 + self.pos[1], 0.5 + self.pos[2], 180.0, 0.0, 90.0)
                command = command + ' ' + "{},{},{},{},{},{},O".format(self.pos[0], -0.55 + self.pos[1], 0.5, 180.0, 0.0, 90.0)
                command = command + ' ' + "-0.3,0,0.5,180.0,0.0,0.0,O"
                command = command + ' ' + "-0.33,0,{},180.0,0.0,0.0,X".format(0.02 + self.z)
                command = command + ' ' + "-0.3,0,0.5,180.0,0.0,0.0,X"
        
                self.str_pub_pos.publish(command)
                self.action = 0

    def finalize(self):
        cv2.destroyAllWindows() 
        rospy.sleep(0.5)
    
def main(args):
    import ctypes
    # XInitThreads 호출
    libX11 = ctypes.CDLL('libX11.so')
    libX11.XInitThreads()

    rospy.init_node('detection_apriltag', anonymous=True)
    try:
        vi = VisionAT()
        cv2.namedWindow('tag_image', cv2.WINDOW_NORMAL)
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)