#!/usr/bin/env python3
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
from sensor_msgs.msg import Image as msg_Image
import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

# keyboard module of pynput python package for keyboard input
from pynput import keyboard
# standard messages for various purpose (e.g. String)
import std_msgs.msg


class Calibrate:
    def __init__(self):
      # Member initialize
      self.bridge = CvBridge()
      self.count = 0
      self.objpoints = []
      self.imgpoints = []
      self.color = None
      self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

      self.res_data = []
      # Pub Sub initialize
      rospy.Subscriber('camera/color/image_raw', msg_Image, self.callback_code)
      self.res_img_pub = rospy.Publisher('class_image', msg_Image, queue_size=10)
      self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)
      

      # Event handler for Keyboard Input 
      with keyboard.Listener(
          on_press=self.on_press) as listener:
          listener.join()

    # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)

        if key == keyboard.Key.f6:
          # Test moving
          self.str_pub_pos.publish("-0.5,0.0,0.5,180.0,90.0,0.0,X")#1
          #self.str_pub_pos.publish("-0.400,0.274,0.484,180.0,0.0,180.0,2.0,X")#1
          # one = cv2.imread('/root/catkin_ws/src/vision/src/images/2dcalib_0323_cen.png',cv2.IMREAD_COLOR)
          # h, w = one.shape[:2]
          # print(one.shape)
          # one = cv2.circle(one, (int(w/2),int(h/2.0)), 3, (0,0,255), 1) # Center
          # one = cv2.circle(one, (620,376), 3, (255,0,0), 1) # Cx Cy
          # cv2.imwrite('/root/catkin_ws/src/vision/src/images/2dcalib_0323_cen_r.jpg', one)

        if key == keyboard.Key.f7:
          print('end')
          return False

        # check robot is moving, if robot is moving, then do nothing.
        if key == keyboard.Key.f9:
          self.start_calib()
          
    def start_calib(self):
      for test_count in range(3):
        print('test count: ' + str(test_count))
        
        self.objpoints = []
        self.imgpoints = []
        deg = 10.0
        dist = 0.6

        for self.count in range(18):
          print('sub count: ' + str(self.count))

          if self.count % 9 == 0:
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = -dist * math.sin(10 * math.pi / 180)
            z = 0.5 + (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0 + deg * (1 + int(self.count / 9))
            pitch = 90.0 - deg * (1 + int(self.count / 9))

          if self.count % 9  == 1:
            x = -0.5
            y = 0.0
            z = 0.5 + (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0
            pitch = 90.0 - deg * (1 + int(self.count / 9))

          if self.count % 9  == 2:
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = dist * math.sin(10 * math.pi / 180)
            z = 0.5 + (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0 - deg * (1 + int(self.count / 9))
            pitch = 90.0 - deg * (1 + int(self.count / 9))

          if self.count % 9  == 3:
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = -dist * math.sin(10 * math.pi / 180)
            z = 0.5
            roll = 180.0 + deg * (1 + int(self.count / 9))
            pitch = 90.0

          if self.count % 9  == 4:
            x = -0.5
            y = 0.0
            z = 0.5
            roll = 180.0
            pitch = 90.0

          if self.count % 9  == 5:           
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = dist * math.sin(10 * math.pi / 180)
            z = 0.5
            roll = 180.0 - deg * (1 + int(self.count / 9))
            pitch = 90.0

          if self.count % 9  == 6:
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = -dist * math.sin(10 * math.pi / 180)
            z = 0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0 + deg * (1 + int(self.count / 9))
            pitch = 90.0 + deg * (1 + int(self.count / 9))

          if self.count % 9  == 7:
            x = -0.5
            y = 0.0
            z = 0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0
            pitch = 90.0 + deg * (1 + int(self.count / 9))

          if self.count % 9  == 8:
            x = -0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            y = dist * math.sin(10 * math.pi / 180)
            z = 0.5 - (dist - dist * math.cos(10 * math.pi / 180))
            roll = 180.0 - deg * (1 + int(self.count / 9))
            pitch = 90.0 + deg * (1 + int(self.count / 9))

          yaw = (test_count - 1) * 20.0
            
          self.str_pub_pos.publish("{},{},{},{},{},{},X".format(round(x, 4), round(y, 4), round(z, 4), roll, pitch, yaw))
          rospy.sleep(3.5)
          """self.calib_stack(test_count)

      np.savetxt('/root/catkin_ws/src/vision/src/images/2dcalib_0324/result.csv',self.res_data,delimiter=",")"""

    def calib_stack(self, kcount):
      self.gray = cv2.cvtColor(self.color,cv2.COLOR_BGR2GRAY)
      objp = np.zeros((8*6,3), np.float32)
      objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)
      criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

      ret_f, corners = cv2.findChessboardCorners(gray, (8,6),None)

      if ret_f == True:
        self.objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        self.imgpoints.append(corners2)

        imgpts, jac = cv2.projectPoints(self.axis, rvec, tvec, mtx, dist)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret_f)
        img = self.draw(self.color, corners2, imgpts)
        img_name = 't' + str(kcount) + '_' + str(self.count % 8) + '.jpg'
        cv2.imwrite('/root/catkin_ws/src/vision/image/' + img_name, img)  #Image save

    def calibration(self):
      ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1],None,None)
      print(ret)
      print(mtx)
      print(dist)
      print(rvecs)
      print(tvecs)
      # Find the rotation and translation vectors.
      _, rvec, tvec, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
      # project 3D points to image plane
      print(rvec)
      #R, _ = cv2.Rodrigues(rvecs)
      #print(R)
      print(tvec)

      cam_info = 'image width: 640 \nimage height: 480 \ncamera_name: arm_camera \n'  
      
      cam_info = cam_info + 'camera_matrix: \n  rows: 3 \n  cols: 3 \n  data: '

      for i in range(3):
        for j in range(3):
          if i == 0 and j == 0: 
            cam_info = cam_info + '['

          cam_info = cam_info + mtx[i][j] + ', ' 
          
          if i == 2 and j == 2: 
            cam_info = cam_info + ']\n'

      cam_info = cam_info + 'distortion mdel: plumb_bob \n distortion_coefficients: \n  rows: 1 \n  cols: 5 \n  data: '

      for i in range(5):
        if i == 0: 
          cam_info = cam_info + '[  '

        cam_info = cam_info + dist[0][i] + ', ' 
          
        if i == 4: 
          cam_info = cam_info + ']\n'

      cam_info = cam_info + 'rectification_matrix: \n rows: 3 \n cols: 3 \n data: '
      
      for i in range(3):
        for j in range(3):
          if i == 0 and j == 0: 
            cam_info = cam_info + '['

          cam_info = cam_info + mtx[i][j] + ', ' 
          
          if i == 2 and j == 2: 
            cam_info = cam_info + ']\n'


      
    """def save_image(self)
      for i in range(kcount * self.count):
        imgpts, jac = cv2.projectPoints(self.axis, rvec, tvec, mtx, dist)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        img = self.draw(self.color, corners2, imgpts)
        img_name = 't' + str(kcount) + '_' + str(self.count % 8) + '.jpg'
        cv2.imwrite('/root/catkin_ws/src/vision/image/' + img_name, img)  #Image save
        self.res_img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))  #Image publish
        #Data process
        data = np.concatenate((
          [ret],
          np.array(mtx).flatten(),
          np.array(dist).flatten(),
          np.array(rvecs).flatten(),
          np.array(tvecs).flatten(),
          np.array(rvec).flatten(),
          np.array(tvec).flatten()))
        self.res_data.append(data.T)"""

    def callback_code(self, image):
      self.color = self.bridge.imgmsg_to_cv2(image)
    
    def draw(self, img, corners, imgpts):
      corner = tuple(corners[0].ravel())
      img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
      img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
      img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
      return img


def main(args):
  rospy.init_node('calibrate',anonymous=True)
  try:
    cal = Calibrate()
  except rospy.ROSInterruptException: pass


if __name__ == '__main__':
    main(sys.argv)