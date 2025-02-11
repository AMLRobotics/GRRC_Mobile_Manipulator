#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.msg
import ros_numpy
import open3d as o3d
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
import os
import threading
import sys
import random
import matplotlib.pyplot as plt
import ctypes
import itertools

class VisionCV:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.center_point = [0, 0, 0]
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', sensor_msgs.msg.Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, self.depth_image_callback)
        self.info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', sensor_msgs.msg.CameraInfo, self.camera_info_callback)
        
    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_rectangles(self.cv_image)
            cv2.imshow('test', self.cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def depth_image_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
            depth_image = self.depth_frame / 1000
            cv2.namedWindow('depth', cv2.WINDOW_NORMAL)
            cv2.imshow('depth', depth_image)
            cv2.waitKey(1)
            """if len(self.inter_points) == 4:
                self.center_point[2] = self.compute_average_depth(self.depth_frame, self.inter_points)
                print(self.center_point)"""

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

    def detect_rectangles(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.blur(gray, (3,3))
        _, binary = cv2.threshold(gray_blur, 166, 255, cv2.THRESH_BINARY)
        canny = cv2.Canny(binary, 110, 300)
        
        img = cv_image.copy()

        hough = cv2.HoughLines(canny, 1, np.pi/180, 112)

        if hough is not None:
            for line in hough:
                rho, theta = line[0]
                a = np.cos(theta) # θ에서의 cosine 값
                b = np.sin(theta) # θ에서의 sin 값
                x0 = a * rho # 직선과 x축이 만나는 점의 x 좌표
                y0 = b * rho # 직선과 y축이 만나는 점의 y 좌표 
                x1 = int(x0 + 1000 * (-b)) 
                # x1 : 직선 위의 한 점으로부터 직선을 따라 왼쪽으로 1000 픽셀 떨어진 점의 x 좌표
                y1 = int(y0 + 1000 * (a))
                # y1 : 동일한 점으로부터 위쪽으로 1000 픽셀 떨어진 점의 y 좌표
                x2 = int(x0 - 1000 * (-b))
                # x2 : 직선 위의 한 점으로부터 직선을 따라 오른쪽으로 1000 픽셀 떨어진 점의 x 좌표
                y2 = int(y0 - 1000 * (a))
                # y2 : 동일한 점으로부터 아래쪽으로 1000 픽셀 떨어진 점의 y 좌표
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        intersections = []
        for line1, line2 in itertools.combinations(hough[:, 0], 2):
            intersection = self.compute_intersection(line1, line2)
            if intersection and abs(intersection[0]) < 641 and abs(intersection[1]) < 481:
                intersections.append(intersection)

        x_sum = 0.0
        y_sum = 0.0
        for i in intersections:
            cv2.circle(img, i, 3, (255, 0, 0), -1, cv2.LINE_AA)
            
            x_sum = x_sum + i[0]
            y_sum = y_sum + i[1]

        if len(intersections) == 4:
            center = (int(x_sum / len(intersections)), int(y_sum / len(intersections)))
            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, center, self.depth_frame[center[0]][center[1]])

            #self.center_point[0] = center[0]
            #self.center_point[1] = center[1]
            #self.center_point[2] = 
            print(point_3d)
            cv2.circle(img, center, 5, (0, 255, 0), -1, cv2.LINE_AA)

        cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
        cv2.imshow('gray', gray_blur)
        cv2.namedWindow('binary', cv2.WINDOW_NORMAL)
        cv2.imshow('binary', binary)
        cv2.namedWindow('canny', cv2.WINDOW_NORMAL)
        cv2.imshow('canny', canny)
        cv2.namedWindow('visual', cv2.WINDOW_NORMAL)
        cv2.imshow('visual', img)
        cv2.waitKey(1)

    def compute_intersection(self, line1, line2):
        (rho1, theta1) = line1
        (rho2, theta2) = line2

        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([[rho1], [rho2]])

        if np.linalg.det(A) == 0:
            return None  # 두 직선이 평행한 경우

        intersection = np.linalg.inv(A).dot(b)
        return int(intersection[0]), int(intersection[1])

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.cv_image is not None:
                    cv2.namedWindow('test', cv2.WINDOW_NORMAL)

        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

def main(args):
    import ctypes
    # XInitThreads 호출
    libX11 = ctypes.CDLL('libX11.so')
    libX11.XInitThreads()

    rospy.init_node('detection_opencv', anonymous=True)
    try:
        vi = VisionCV()
        vi.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main(sys.argv)