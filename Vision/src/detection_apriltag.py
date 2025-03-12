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
import tf2_ros
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from vision_detection.srv import *
import std_msgs.msg
import math
import ctypes

class VisionAT:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.action = 0
        #self.img_sub = rospy.Subscriber('/tag_detections_image', sensor_msgs.msg.Image, self.image_callback)
        self.tf_sub = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.tf_callback)
        self.imu_sub = rospy.Subscriber('/camera/imu', sensor_msgs.msg.Imu, self.imu_callback)
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.command_pos = None

        #cv2.namedWindow('tag_image', cv2.WINDOW_NORMAL)

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
        
    """def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #self.detect_rectangles(self.cv_image)
            #cv2.circle(self.cv_image, (320, 240), 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.imshow('tag_image', self.cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)"""

    def tf_callback(self, data):
        #base to tool end destination transformation
        base = self.tfBuffer.lookup_transform('base', 'tool0_destination', rospy.Time(0))

        #rotoation
        q = base.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w], axes = "sXYZ")
        rot = np.array([roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi])

        #translation
        t = base.transform.translation
        trans = np.array([t.x, t.y, t.z])

        #set robot command
        self.command_pos = np.array([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]])
        
        print(self.command_pos)

    def imu_callback(self, data):
        temp = data

    def manual_move(self, key):
        if key == keyboard.KeyCode(char='u'):
            command = "{},{},{},{},{},{},O".format(self.command_pos[0], self.command_pos[1], self.command_pos[2], self.command_pos[3], self.command_pos[4], self.command_pos[5])
            command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,O"
            command = command + ' ' + "{},{},{},{},{},{},X".format(self.command_pos[0], self.command_pos[1], self.command_pos[2], self.command_pos[3], self.command_pos[4], self.command_pos[5])
            command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,X"
       
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
    # XInitThreads 호출
    #libX11 = ctypes.CDLL('libX11.so')
    #libX11.XInitThreads()

    rospy.init_node('detection_apriltag', anonymous=True)
    try:
        vi = VisionAT()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)