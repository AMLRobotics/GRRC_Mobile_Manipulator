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
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/color/points', sensor_msgs.msg.PointCloud2, self.pointcloud_callback)
        self.image_sub = rospy.Subscriber('/tag_detections_image', sensor_msgs.msg.Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', sensor_msgs.msg.Image, self.depth_image_callback)
        self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', sensor_msgs.msg.CameraInfo, self.depth_info_callback)
        self.open3d_vis = o3d.visualization.Visualizer()
        self.open3d_vis.create_window(window_name='Open3D Point Cloud')
        
    def pointcloud2_to_open3d(self, pointcloud2_msg):
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud2_msg)
        points = np.zeros((pc_array.shape[0], 3))
        points[:, 0] = pc_array['x']
        points[:, 1] = pc_array['y']
        points[:, 2] = pc_array['z']
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)
        return o3d_cloud

    def print_open3d_pointcloud_info(self, o3d_cloud):
        points = np.asarray(o3d_cloud.points)
        for i, point in enumerate(points):
            print(f"Point {i}: x={point[0]}, y={point[1]}, z={point[2]}")

    def pointcloud_callback(self, data):
        try:
            self.o3d_data = self.pointcloud2_to_open3d(data)
            # Perform your point cloud processing here
            # For example, detecting objects using Open3D
            # self.print_open3d_pointcloud_info(self.o3d_data)

            self.apply_voxel_filter(self.o3d_data)
        except CvBridgeError as e:
            print(e)

    def apply_voxel_filter(self, cloud):
    # Voxel downsampling
        voxel_size = 0.01  # Adjust the voxel size as needed
        downsampled_cloud = cloud.voxel_down_sample(voxel_size)
        #self.detect_objects(downsampled_cloud)
        self.visualize_points(downsampled_cloud)

    def detect_objects(self, cloud):
        # Plane segmentation
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        inlier_cloud = cloud.select_by_index(inliers)
        outlier_cloud = cloud.select_by_index(inliers, invert=True)

        # Euclidean Cluster Extraction
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(outlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
        
        max_label = labels.max()
        print(f"point clouder has {max_label + 1} clusters")
        
        # Visualize clusters
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        self.open3d_vis.clear_geometries()
        self.open3d_vis.add_geometry(outlier_cloud)
        self.open3d_vis.poll_events()
        self.open3d_vis.update_renderer()

        # Analyze clusters to find rectangular objects
        for i in range(max_label + 1):
            cluster = outlier_cloud.select_by_index(np.where(labels == i)[0])
            points = np.asarray(cluster.points)
            if self.is_rectangular_object(points):
                print(f"Cluster {i} is a rectangular object")
                self.visualize_points(points)

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

    def visualize_points(self, points):
        self.open3d_vis.clear_geometries()

        tmp_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        image = o3d.geometry.Image(tmp_image)
        depth = o3d.geometry.Image(self.depth_frame)

        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image, depth)

        rgbd_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        rgbd_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.visualization.draw_geometries([rgbd_pcd])
        
        #self.open3d_vis.add_geometry(o3d_rgbd)
       
        o3d_cloud = o3d.geometry.PointCloud()
        #o3d_cloud.points = o3d.utility.Vector3dVector(points)
        o3d_cloud = points
        #o3d_cloud.paint_uniform_color([1, 0, 0])
        o3d_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        self.open3d_vis.poll_events()
        self.open3d_vis.update_renderer()

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