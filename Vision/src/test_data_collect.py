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

class ActionCommand:
    def __init__(self):

        self.action_srv = rospy.Service('/manipulator_action', std_msgs.msg.String, move)

    def move(req):    
        # pick and place task in place
        if req == "MANUAL_PAP_TEST_1":
            command = "{},{},{},{},{},{},O".format(self.object_pose[0], self.object_pose[1], self.object_pose[2], self.object_pose[3], self.object_pose[4], self.object_pose[5])
            command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,O"
            command = command + ' ' + "{},{},{},{},{},{},X".format(self.object_pose[0], self.object_pose[1], self.object_pose[2], self.object_pose[3], self.object_pose[4], self.object_pose[5])
            command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,X"

            self.str_pub_pos.publish(command)

        # collect data
        elif key == keyboard.KeyCode(char='i'):
            if self.action == 0:
                command = "0.0,-0.65,0.7,180.0,0.0,90.0,X"
                self.str_pub_pos.publish(command)
                rospy.sleep(5.0)

                command = "{},{},0.7,180.0,0.0,90.0,X".format(-self.tag_pos[0][0], -0.65 + self.tag_pos[0][1])
                self.center = np.array([-self.tag_pos[0][0], -0.65 + self.tag_pos[0][1]])
                print(self.center)
                self.str_pub_pos.publish(command)

                self.action = 1
            
            elif self.action == 1:
                writer_t = pd.ExcelWriter("/root/catkin_ws/src/vision_detection/data/test_marker_position_data.xlsx", engine ='openpyxl', mode = 'a', if_sheet_exists="overlay")   
                writer_v = pd.ExcelWriter("/root/catkin_ws/src/vision_detection/data/test_vision_position_data.xlsx", engine ='openpyxl', mode = 'a', if_sheet_exists="overlay")

                for i in range(9):
                    self.df_t[i] = pd.read_excel("/root/catkin_ws/src/vision_detection/data/test_marker_position_data.xlsx", sheet_name ='-30_-45', engine ='openpyxl')
                    self.df_v[i] = pd.read_excel("/root/catkin_ws/src/vision_detection/data/test_vision_position_data.xlsx", sheet_name ='-30_-45', engine ='openpyxl')
               
                for i in range(0, 50):
                    for j in range(0, 9):
                        x = ((j % 3) - 1) * 0.1
                        y = (int(j / 3) - 1) * 0.05
                        command = "{},{},0.7,{},0.0,90.0,X".format(self.center[0] - x, self.center[1] + y, 180.0 + (-x) * 200.0)
                        #print(x, y)

                        self.str_pub_pos.publish(command)
                        self.is_moving = True
                        rospy.sleep(6.0)
                        self.is_moving = False

                        while self.vec_renew < 3:
                            if self.vec_renew >= 3:
                                break
                        self.vec_renew = 0

                        if i == 0:
                            self.df_t[j] = pd.DataFrame(self.marker_position.tolist(), columns=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
                            self.df_v[j] = pd.DataFrame(self.model_position.tolist(), columns=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])

                        else:
                            self.df_t[j].loc[i] = self.marker_position[0].tolist()
                            self.df_v[j].loc[i] = self.model_position[0].tolist()

                        self.df_t[j].to_excel(writer_t, sheet_name ='-30_-45', startcol = 8 * j, startrow = 0, index = False) 
                        self.df_v[j].to_excel(writer_v, sheet_name ='-30_-45', startcol = 8 * j, startrow = 0, index = False) 

                writer_t.close()
                writer_v.close()

                self.action = 0

        # ddd
        elif key == keyboard.KeyCode(char='p'):
            if self.action == 0:
                command = "-0.3,0.0,0.5,180.0,0.0,0.0,X"
                self.str_pub_pos.publish(command)
                rospy.sleep(5.0)

                self.action = 1
            
            elif self.action == 1:         
                #target_position = np.array([self.object_pose[0], self.object_pose[1], self.object_pose[2]])
                #print(target_position)
                #-0.33169  -0.0086888     0.12158
                for i in range(300):
                    rospy.sleep(2.0)
                    command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,X"
                    command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,X"
                    command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,O"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(8.0)

                    command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,O"
                    command = command + ' ' + "0.0,-0.65,0.6,180.0,0.0,90.0,O"
                    self.str_pub_pos.publish(command)
                    #print(1)
                    rospy.sleep(8.0)

                    #destination_position = np.array([self.place2_pose[0], self.place2_pose[1], self.place2_pose[2]])
                    #print(destination_position)
                    #    0.21495    -0.78669     0.10557    0.21708    -0.78433    0.093992
                    
                    command = "0.01695,-0.78,0.6,180.0,0.0,90.0,O"
                    command = command + ' ' + "0.01695,-0.78,0.271,180.0,0.0,90.0,O"
                    command = command + ' ' + "0.01695,-0.78,0.271,180.0,0.0,90.0,X"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(6.0)

                    command = "0.01695,-0.78,0.6,180.0,0.0,90.0,X"
                    command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,X"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(4.0)

                    command = "0,-0.65,0.6,180.0,0.0,90.0,X"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(3.0)

                    command = "0.01695,-0.78,0.6,180.0,0.0,90.0,X"
                    command = command + ' ' + "0.01695,-0.78,0.271,180.0,0.0,90.0,X"
                    command = command + ' ' + "0.01695,-0.78,0.271,180.0,0.0,90.0,O"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(4.0)

                    command = "0.01695,-0.78,0.6,180.0,0.0,90.0,O"
                    command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,O"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(8.0)

                    #destination_position = np.array([self.place_pose[0], self.place_pose[1], self.place_pose[2]])

                    command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,O"
                    command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,O"
                    command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,X"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(6.0)

                    command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,X"
                    command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,X"
                    self.str_pub_pos.publish(command)
                    rospy.sleep(4.0)

                self.action = 0

        # count reset
        elif key == keyboard.KeyCode(char='l'):
            self.action = 0

        # position estimation data
        elif key == keyboard.KeyCode(char='y'):
            if self.action == 0:
                command = "0.0,-0.3,0.5,180.0,0.0,90.0,X"
                command = command + ' ' + "0.0,-0.65,0.6,180.0,0.0,90.0,X"

                self.str_pub_pos.publish(command)
                self.action = 1

            elif self.action == 1:
                writer_t = pd.ExcelWriter("/root/catkin_ws/src/vision_detection/data/learning_marker_position_data.xlsx", engine ='openpyxl', mode = 'a', if_sheet_exists="overlay")   
                writer_v = pd.ExcelWriter("/root/catkin_ws/src/vision_detection/data/learning_vision_position_data.xlsx", engine ='openpyxl', mode = 'a', if_sheet_exists="overlay")
                df_t = pd.read_excel("/root/catkin_ws/src/vision_detection/data/learning_marker_position_data.xlsx", sheet_name ='marker_data', engine ='openpyxl')
                df_v = pd.read_excel("/root/catkin_ws/src/vision_detection/data/learning_vision_position_data.xlsx", sheet_name ='vision_data', engine ='openpyxl')

                for i in range(0, 2):
                    for j in range(0, 25):
                        x = ((j % 5) - 2) * 0.1
                        y = (int(j / 5) - 2) * 0.1
                        roll = 180.0 + (-x) * 200.0
                        pitch = y * 100.0
                        yaw = 90.0

                        if j == 12:
                            yaw = 90.0 - 10 + (20 * i)

                        command = "{},{},{},{},{},{},X".format(0 - x * 2, -(0.65 - y), 0.6 + 0.05 * i, roll, pitch, yaw)
                        #print(180.0 + (-y * 10) * 15.0, x * 10 * 15.0)

                        self.str_pub_pos.publish(command)
                        self.is_moving = True
                        rospy.sleep(6.0)
                        self.is_moving = False

                        while self.vec_renew < 3:
                            if self.vec_renew >= 3:
                                break
                        self.vec_renew = 0

                        vision_data = np.array([self.obb_center])
                        vision_data = np.append(vision_data, [[self.obb_param[0][2]]], axis = 1)
                        vision_data = np.append(vision_data, self.object_norm, axis = 1)
                        #print()

                        self.tag_pos[0][0] = self.tag_pos[0][0] * 1000
                        self.tag_pos[0][1] = self.tag_pos[0][1] * 1000
                        self.tag_pos[0][2] = self.tag_pos[0][2] * 1000

                        if len(df_t) == 0 and len(df_v) == 0:
                            #print(1)
                            df_t = pd.DataFrame(self.tag_pos, columns=['x', 'y', 'z', 'x_o', 'y_o', 'z_o', 'w_o'])
                            df_v = pd.DataFrame(vision_data, columns=['x', 'y', 'z', 'r', 'x_n', 'y_n', 'z_n'])

                        else:
                            df_t.loc[len(df_t)] = self.tag_pos[0].tolist()
                            df_v.loc[len(df_v)] = vision_data[0].tolist()

                        df_t.to_excel(writer_t, sheet_name ='marker_data', index = False)
                        df_v.to_excel(writer_v, sheet_name ='vision_data', index = False)
                        print("Data are written.")
                writer_t.close()
                writer_v.close()

                self.action = 0

    def mobile_move(self):
        if(self.is_stop and not self.is_moving):
            self.is_moving = True
            rospy.sleep(2.0)
            command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,X"
            command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,X"
            command = command + ' ' + "-0.337,-0.0086888,0.12,180.0,0.0,0.0,O"
            self.str_pub_pos.publish(command)
            rospy.sleep(8.0)

            command = "-0.337,-0.0086888,0.5,180.0,0.0,0.0,O"
            command = command + ' ' + "0.0,-0.6,0.6,180.0,0.0,90.0,O"
            self.str_pub_pos.publish(command)
            #print(1)
            rospy.sleep(8.0)

            #destination_position = np.array([self.place2_pose[0], self.place2_pose[1], self.place2_pose[2]])
            #print(destination_position)
            #    0.21495    -0.78669     0.10557    0.21708    -0.78433    0.093992
            
            command = "0.01695,-0.69,0.6,180.0,0.0,90.0,O"
            command = command + ' ' + "0.01695,-0.69,0.271,180.0,0.0,90.0,O"
            command = command + ' ' + "0.01695,-0.69,0.271,180.0,0.0,90.0,X"
            self.str_pub_pos.publish(command)
            rospy.sleep(6.0)

            command = "0.01695,-0.69,0.6,180.0,0.0,90.0,X"
            command = command + ' ' + "-0.3,0.0,0.5,180.0,0.0,0.0,X"
            self.str_pub_pos.publish(command)
            rospy.sleep(4.0)

            #self.is_moving = False


if __name__ == "__main__":
    get_YOLO_detection_parameter("test_YOLO_marker_data.xlsx", "test_image_")




