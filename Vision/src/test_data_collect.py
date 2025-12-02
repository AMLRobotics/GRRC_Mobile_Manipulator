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
import std_msgs.msg
import math
import ctypes
import openpyxl as xl
import pandas as pd
from ultralytics import YOLO
    
def get_YOLO_detection_parameter(filename, imagename):    
    data_dir = "/root/catkin_ws/src/vision_detection/data/" + filename
    model = YOLO("/root/catkin_ws/src/vision_detection/vision_detection_weight/best.pt")
    writer = pd.ExcelWriter(data_dir, engine ='openpyxl', mode = 'a', if_sheet_exists="overlay")   
    df = pd.read_excel("/root/catkin_ws/src/vision_detection/data/test_YOLO_marker_data.xlsx", sheet_name ='test_YOLO', engine ='openpyxl')

    for i in range(0, 5):
        img_name = "/root/catkin_ws/src/vision_detection/image/" + imagename + str(i) + ".jpeg"
        results = model(img_name, conf = 0.7)

        xywhr = results[0].obb.xywhr[0].cpu().numpy()
        data = [[xywhr[0], xywhr[1], xywhr[2], xywhr[3], xywhr[4]]]

        if i == 0:
            df = pd.DataFrame(data, columns=['x', 'y', 'w', 'h', 'r'])

        else:
            df.loc[i] = data[0]

        df.to_excel(writer, sheet_name ='test_YOLO') 

    writer.close()

if __name__ == "__main__":
    get_YOLO_detection_parameter("test_YOLO_marker_data.xlsx", "test_image_")




