#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.msg
import ros_numpy
import open3d as o3d
from cv_bridge import CvBridge, CvBridgeError
import os
import threading
import sys
import random
import matplotlib.pyplot as plt
import ctypes

class VisionProcess:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.depth_frame = None  # depth_frame 속성 초기화
        self.depth_intrin = None  # depth_intrin 속성 초기화
        self.cam_open = False
        self.image_sub = rospy.Subscriber('/tag_detections_image', sensor_msgs.msg.Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, self.depth_image_callback)
        self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', sensor_msgs.msg.CameraInfo, self.depth_info_callback)
        
    def apply_voxel_filter(self, cloud):
    # Voxel downsampling
        voxel_size = 0.01  # Adjust the voxel size as needed
        downsampled_cloud = cloud.voxel_down_sample(voxel_size)
        #self.detect_objects(downsampled_cloud)
        self.visualize_points(downsampled_cloud)

    def is_rectangular_object(self, points):
        # Implement your own logic to determine if the cluster points form a rectangular object
        # This can include fitting a 3D bounding box and checking dimensions
        return True  # Placeholder

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_rectangles(self.cv_image)
            if not self.cam_open:
                cv2.namedWindow('test', cv2.WINDOW_NORMAL)
                self.cam_open = True
        except CvBridgeError as e:
            print(e)

    def depth_image_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
            #self.depth_frame = self.depth_frame / 1000
            self.visualize_points()

        except CvBridgeError as e:
            print(e)
        
        #self.depth_frame = np.array(depth_image, dtype=np.float32)

    def depth_info_callback(self, data):
        self.depth_intrin = data.K

    def detect_rectangles(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                if w * h >= 1000:
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def visualize_points(self):
        
        tmp_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        image = o3d.geometry.Image(tmp_image)
        depth = o3d.geometry.Image(self.depth_frame)

        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image, depth)

        rgbd_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        rgbd_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        voxel_size = 0.01  # Adjust the voxel size as needed
        downsampled_pcd = rgbd_pcd.voxel_down_sample(voxel_size)
        o3d.visualization.draw_geometries([downsampled_pcd])

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.cv_image is not None:
                    cv2.imshow('test', self.cv_image)
                    cv2.waitKey(1)
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

def main(args):
    import ctypes
    # XInitThreads 호출
    libX11 = ctypes.CDLL('libX11.so')
    libX11.XInitThreads()

    rospy.init_node('realsense_object_detection', anonymous=True)
    try:
        vi = VisionProcess()
        vi.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)