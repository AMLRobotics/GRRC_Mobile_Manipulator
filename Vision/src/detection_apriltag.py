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
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import std_msgs.msg
import math
import ctypes
import openpyxl as xl
import pandas as pd
from ultralytics import YOLO
from vision_detection.msg import PlaneResults, prediction_data
import pyrealsense2 as rs
import tensorflow as tf
from scipy.spatial.transform import Rotation as R

class VisionAT:
    def __init__(self):
        self.model = YOLO("/root/catkin_ws/src/vision_detection/vision_detection_weight/best.pt")
        self.bridge = CvBridge()
        self.cv_image = None
        self.action = 0
        self.img_sub = rospy.Subscriber('/camera/color/image_rect_color', sensor_msgs.msg.Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, self.depth_image_callback)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.tf_sub = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.tf_callback)
        self.imu_sub = rospy.Subscriber('/camera/imu', sensor_msgs.msg.Imu, self.imu_callback)
        self.vector_sub = rospy.Subscriber('/detect_plane/results', PlaneResults, self.vector_callback)
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)
        self.info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', sensor_msgs.msg.CameraInfo, self.camera_info_callback)
        self.data_pub = rospy.Publisher("/prediction_data", prediction_data, queue_size = 10)
        self.model_data_sub = rospy.Subscriber("/model_data", prediction_data, self.model_callback)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.df_t = list(0 for i in range(0,9))
        self.df_v = list(0 for i in range(0,9))

        self.object_pose = None
        self.place_pose = None
        self.place2_pose = None
        self.model_position = None
        self.marker_position = None
        self.tag_name = None
        self.tag_pos = None
        self.obb_points = None
        self.vec_renew = 0
        self.obb_center = None
        self.obb_param = None
        self.object_norm = None
        self.is_moving = True
        self.intrinsics = None

        #cv2.namedWindow('tag_image', cv2.WINDOW_NORMAL)

        # Event handler for Keyboard Input 
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def coordinate_restore(self, normals, yaws):
        """
        normals: (N, 3)
        yaws: (N,)
        return: quaternions (N, 4)
        """
        normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)
        z_axis = normals  # shape: (N, 3)

        # x2d: shape (N, 3)
        x2d = np.stack([np.cos(yaws), np.sin(yaws), np.zeros_like(yaws)], axis=1)

        # x_proj = x2d - (x2d ⋅ z) * z
        dot = np.sum(x2d * z_axis, axis=1, keepdims=True)
        x_proj = x2d - dot * z_axis
        x_proj = x_proj / np.linalg.norm(x_proj, axis=1, keepdims=True)
        x_axis = x_proj

        # y_axis = z × x
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis, axis=1, keepdims=True)

        # Stack into rotation matrices
        R_mat = np.stack([x_axis, y_axis, z_axis], axis=-1)  # (N, 3, 3)

        # Convert to quaternion
        r = R.from_matrix(R_mat)
        quats = r.as_quat()  # (N, 4), format: [x, y, z, w]
        return quats

    def apply_r_x_180_to_outputs(self, X_extended):
        R_x_180 = R.from_euler('x', 180, degrees=True)
        X_rotated = X_extended.copy()

        # 1. normal 회전
        normals = X_extended[:, 4:7]
        normals_rotated = R_x_180.inv().apply(normals)
        X_rotated[:, 4:7] = normals_rotated

        # 2. 입력 quaternion 회전
        q1 = X_extended[:, 7:]
        r1 = R.from_quat(q1)
        q1_rotated = (R_x_180.inv() * r1).as_quat()
        X_rotated[:, 7:] = q1_rotated

        return X_rotated


    def quaternion_to_6d(self, q_array):
        """
        (N, 4) → (N, 6) 회전 표현
        """
        rot = R.from_quat(q_array)
        matrix = rot.as_matrix()  # (N, 3, 3)
        return matrix[:, :, :2].reshape(-1, 6)  # 앞의 2열만 사용

    def standardize_quaternion(self, q):
        q = q / np.linalg.norm(q, axis=-1, keepdims=True)
        if q.ndim == 1:
            return q if q[3] >= 0 else -q
        sign = np.where(q[:, 3:4] >= 0, 1.0, -1.0)
        return q * sign

    def tf_rotation_6d_to_matrix(self, x):
        """
        (N, 6) → (N, 3, 3) 회전행렬 복원
        """
        x = tf.reshape(x, [-1, 3, 2])
        b1 = tf.linalg.l2_normalize(x[:, :, 0], axis=1)
        b2 = tf.linalg.l2_normalize(
            x[:, :, 1] - tf.reduce_sum(b1 * x[:, :, 1], axis=1, keepdims=True) * b1,
            axis=1,
        )
        b3 = tf.linalg.cross(b1, b2)
        return tf.stack([b1, b2, b3], axis=-1)


    def calculate_orientation(self, width, height, angle):
        if width >= height:
            orientation = angle
        else:
            orientation = -(math.pi / 2 - angle)

        return orientation

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

            #image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            results = self.model.track(source = self.cv_image, conf = 0.7)
            cv2.imshow('tag_image', results[0].plot())
            cv2.waitKey(1)

            obbox = results[0].obb.xywhr[0].cpu().numpy()
            self.obb_points = results[0].obb.xyxyxyxy[0].cpu().numpy()

            self.obb_center = rs.rs2_deproject_pixel_to_point(self.intrinsics, [int(obbox[0]), int(obbox[1])], self.depth_frame[int(obbox[1])][int(obbox[0])])
            self.obb_center[2] = self.obb_center[2] / 1000
            self.obb_param = np.array([[obbox[2], obbox[3], obbox[4]]])
  
            self.obb_param[0][2] = self.calculate_orientation(obbox[2], obbox[3], obbox[4])
            #calculate_target_position()

        except CvBridgeError as e:
            print(e)

    def camera_info_callback(self, msg):
        """카메라 내부 파라미터 가져오기"""
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.K[2]
        self.intrinsics.ppy = msg.K[5]
        self.intrinsics.fx = msg.K[0]
        self.intrinsics.fy = msg.K[4]
        self.intrinsics.model = rs.distortion.none
        self.intrinsics.coeffs = [i for i in msg.D]

    def depth_image_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

        except CvBridgeError as e:
            print(e)

    def tf_callback(self, data):  
        temp = data

    def imu_callback(self, data):
        temp = data

    def tag_callback(self, data):
        for i in range(len(data.detections)):
            if i == 0:
                self.tag_name = list(data.detections[i].id)
            else:
                self.tag_name.extend(list(data.detections[i].id))

        if len([self.tag_name for i in self.tag_name if i == 2]) == 1:
            self.object_pose = self.calculate_target_position('tool0_destination')
            #print(self.object_pose)

        if len([self.tag_name for i in self.tag_name if i == 3]) == 1:    
            self.place_pose = self.calculate_target_position('Place')
            #print(self.place_pose)

        if len([self.tag_name for i in self.tag_name if i == 4]) == 1:       
            self.place2_pose = self.calculate_target_position('Place2')
            #print(self.place2_pose)


        tag = data.detections[0].pose.pose.pose
        self.tag_pos = np.array([[tag.position.x, tag.position.y, tag.position.z, tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.orientation.w]])
        #print(self.tag_pos)

    def vector_callback(self, data):
        #print(len(data.center_2d) / 2)
        is_object = None
        data_3d = None
        object_vec = None
        for i in range(int(len(data.center_2d) / 2)):
            center_2d = [int(data.center_2d[i * 2]), int(data.center_2d[i * 2 + 1])]
            center_3d = [data.center_3d[i * 3], data.center_3d[i * 3 + 1], data.center_3d[i * 3 + 2]]
            #cv2.circle(self.cv_image, center_2d, 10, (255, i * 100, 0), -1)

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, center_2d, self.depth_frame[center_2d[1]][center_2d[0]])
            #print(center_3d[2] * 1000,  point_3d[2])

            result = cv2.pointPolygonTest(self.obb_points, center_2d, measureDist=False)

            if i == 0:
                data_3d = np.array([[center_3d, point_3d]])
            else:
                data_3d = np.append(data_3d, np.array([[center_3d, point_3d]]), axis = 0)

            #print(result)
            if result > 0:
                if is_object is None:
                    #print(0)
                    is_object = np.array([i])
                else:
                    #print(1)
                    is_object = np.append(is_object, np.array([i]), axis = 0)
        #print(data_3d[1][0][2] * 1000, data_3d[1][1][2])

        for i in is_object:
            z_comp = data_3d[i][0][2] * 1000 - data_3d[i][1][2]

            if abs(z_comp) > 2:
                object_vec = i
        object_norm = np.array([[data.norms[object_vec * 3], data.norms[object_vec * 3 + 1], data.norms[object_vec * 3 + 2]]])
        #print(object_norm, object_vec)

        self.object_norm = object_norm

        """if self.action == 1 and not self.is_moving and self.vec_renew < 3:
            self.vec_renew = self.vec_renew + 1
            #print(self.vec_renew)"""

        vision_data = np.array([self.obb_center])
        vision_data[0][2] = vision_data[0][2] * 1000
        vision_data = np.append(vision_data, np.array([[self.obb_param[0][2]]]), axis = 1)
        vision_data = np.append(vision_data, self.object_norm, axis = 1)

        quat = self.coordinate_restore(vision_data[:, 4:], vision_data[:, 3])

        vision_data_rotated = self.apply_r_x_180_to_outputs(np.hstack([vision_data, quat]))

        rot_6d = self.quaternion_to_6d(vision_data_rotated[:, 7:])
        vision_data = np.hstack([vision_data_rotated[:, :7], rot_6d])

        pred_data = prediction_data()
        pred_data.input_data = np.array(vision_data[0], dtype=np.float32).tolist()
        pred_data.output_data = np.array(self.tag_pos[0], dtype=np.float32).tolist()

        self.data_pub.publish(pred_data)
        print("Data Published!")

    def model_callback(self, data):
        t = np.array([data.input_data])
        v = np.array([data.output_data])

        q1 = self.standardize_quaternion(t[:, 3:])
        q2 = self.standardize_quaternion(v[:, 3:])
        euler1 = R.from_quat(q1).as_euler('xyz', degrees=True)
        euler2 = R.from_quat(q2).as_euler('xyz', degrees=True)

        self.model_position = np.hstack([t[:, :3], euler1])
        self.marker_position = np.hstack([v[:, :3], euler2])

        print("Data Subscribed!")
        print(self.action, self.is_moving, self.vec_renew)

        if self.action == 1 and not self.is_moving and self.vec_renew < 3:
            self.vec_renew = self.vec_renew + 1
            print(self.vec_renew)


    def calculate_target_position(self, tf_name):
        #base to tool end destination transformation
        base = self.tfBuffer.lookup_transform('base', tf_name, rospy.Time(0))

        #rotoation
        q = base.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w], axes = "sXYZ")
        rot = np.array([roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi])

        #translation
        t = base.transform.translation
        trans = np.array([t.x, t.y, t.z])

        #set robot command
        pose = np.array([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]])
        return pose
        
        #print(self.object_pose)

    def manual_move(self, key):
        # pick and place task in place
        if key == keyboard.KeyCode(char='u'):
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
 
    def finalize(self):
        cv2.destroyAllWindows() 
        rospy.sleep(0.5)
    
def main(args):
    # XInitThreads 호출
    libX11 = ctypes.CDLL('libX11.so')
    libX11.XInitThreads()

    rospy.init_node('detection_apriltag', anonymous=True)
    try:
        vi = VisionAT()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)