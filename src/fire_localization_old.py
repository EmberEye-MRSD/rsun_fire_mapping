import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf.transformations import euler_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler, inverse_matrix

import message_filters
import yaml

class CameraParams:
   def __init__(self, name) -> None:
      self.name = name
      self.K = np.ones((3,3))
      self.D = np.ones((5,1))
      self.R = np.ones((3,3))
      self.T = np.zeros((3,1))

class FireLocalization:

   def __init__(self):
      self.image = None

      self.bridge = CvBridge()
      self.H_imu_flu = np.eye(4)
      # rotation about Z axis by 180 degrees
      self.H_imu_flu[:3,:3] = euler_matrix(0, 0, np.pi,'rxyz')[:3,:3]

      self.left_info  = self.loadCameraParams("../params/thermal_left.old.yaml", "left")
      self.right_info = self.loadCameraParams("../params/thermal_right.old.yaml", "right")

      self.left_info_og = CameraParams("left")
      self.right_info_og = CameraParams("right")

      self.left_info_og.K = self.left_info.K
      self.left_info_og.D = self.left_info.D
      self.left_info_og.R = self.left_info.R
      self.left_info_og.T = self.left_info.T


      # self.left_info_aligned = CameraParams("left_aligned")
      # self.right_info_aligned = CameraParams("right_aligned")
      self.processExtrinsics()

      # exit()


      # print(self.H_imu_flu)


      self.H_imu_world = np.eye(4)

      self.flag_got_left = False
      self.flag_got_right = False
      self.flag_got_odom = False

      self.left_stamp = None
      self.right_stamp = None
      self.odom_stamp = None

      self.left_sub = rospy.Subscriber("/thermal_left/image", Image, self.left_cb)
      self.right_sub = rospy.Subscriber("/thermal_right/image", Image, self.right_cb)
      self.odom_sub = rospy.Subscriber("/mso_estimator/pose_imu_transformed", PoseStamped, self.odom_cb)


      self.hotspot_pub = rospy.Publisher("/hotspots", Marker, queue_size=10)


      # self.image_syncer = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub, self.odom_sub], 10, 0.05)
      # self.image_syncer.registerCallback(self.image_callback)
      self.main_loop_timer = rospy.Timer(rospy.Duration(0.1), self.main_loop)

   def left_cb(self, msg):
      self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      self.left_stamp = msg.header.stamp
      self.flag_got_left = True
   
   def right_cb(self, msg):
      self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      self.right_stamp = msg.header.stamp
      self.flag_got_right = True
   
   def odom_cb(self, msg):
      self.odom_pose = msg
      self.odom_stamp = msg.header.stamp

      rpy = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], axes='rxyz')
      rpy = list(rpy)
      rpy[0] = -rpy[0]
      rpy[1] = -rpy[1]

      quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='rxyz')


      self.H_imu_world[:3,:3] = quaternion_matrix(quat)[:3,:3]
      self.H_imu_world[:3,3] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
      self.flag_got_odom = True   
   
   def loadCameraParams(self, kalibr_path, camera_name):
      with open(kalibr_path, 'r') as file:
         print("Loaded params from file:", kalibr_path, "for camera:", camera_name)
         cam_params = yaml.load(file, Loader=yaml.FullLoader)
         camera = cam_params["cam0"]
         cam_info = CameraParams(camera_name)
         fx, fy, cx, cy = camera["intrinsics"]
         cam_info.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
         cam_info.D = np.array(camera["distortion_coeffs"]+[0.0]).reshape(5,1)
         H = np.linalg.inv(np.array(camera["T_cam_imu"]).reshape(4,4))
         H = self.H_imu_flu@H
         cam_info.R = H[:3,:3]
         cam_info.T = H[:3,3]
         print("--------------------")
         print("Camera Name: ", camera_name)
         print("Rotation Matrix: ", cam_info.R)
         print("Translation Vector: ", cam_info.T)
         print("--------------------")
         # print("Rotation Matrix: ", cam_info.R)
         # print("Translation Vector: ", cam_info.T)

         return cam_info
      
   def display_image(self, image):
      # normalize the image
      cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
      image = np.uint8(image)
      cv2.imshow("Image", image)
      cv2.waitKey(1)


   def processImage(self, image):
      # binary thresholding
      image = cv2.inRange(image, 90, 200)
      # dilate the image
      image = cv2.dilate(image, np.ones((10,10), np.uint8), iterations=1)

      countours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      centers = []

      for contour in countours:
         M = cv2.moments(contour)
         if M["m00"] == 0:
            continue
         cX = int(M["m10"] / M["m00"])
         cY = int(M["m01"] / M["m00"])
         centers.append((cX, cY))
      
      # cluster the centers
      clusters = [[]]
      for center in centers:
         if len(clusters[-1]) == 0:
            clusters[-1].append(center)
         else:
            if np.linalg.norm(np.array(center) - np.array(clusters[-1][-1])) < 50:
               clusters[-1].append(center)
            else:
               clusters.append([center])
      
      # find the center of each cluster
      cluster_centers = []
      for cluster in clusters:
         cluster_center = np.mean(cluster, axis=0)
         cluster_centers.append(cluster_center)

      return cluster_centers
   
   def get_corresspondences(self, left_centers, right_centers, thresh = 10): # thresh is in pixels
      if(len(left_centers) != len(right_centers)):
         return None, None
         
      left_centers_np = np.array(left_centers)
      right_centers_np = np.array(right_centers)

      idxs_L = np.argsort(left_centers_np, axis=0)
      idxs_R = np.argsort(right_centers_np, axis=0)

      sorted_L = np.take_along_axis(left_centers_np, idxs_L, axis=0)
      sorted_R = np.take_along_axis(right_centers_np, idxs_R, axis=0)

      # for i in range(sorted_L.shape[0]):
      #    if(np.hypot(sorted_L[i, 0] - sorted_R[i, 0], sorted_L[i, 1] - sorted_R[i, 1]) > thresh):
      #       return None, None

      return sorted_L, sorted_R
   
   def getHotspots(self, left_centers, right_centers):
      hotspots = []
      for i in range(left_centers.shape[0]):
         disp = left_centers[i, 0] - right_centers[i, 0]
         baseline = np.linalg.norm(self.left_info.T - self.right_info.T)
         depth = (self.left_info.K[0,0]*baseline)/disp

         pos_3d = np.zeros(3)
         pos_3d[0] = (left_centers[i, 0] - self.left_info.K[0,2])*depth/self.left_info.K[0,0]
         pos_3d[1] = (left_centers[i, 1] - self.left_info.K[1,2])*depth/self.left_info.K[1,1]
         pos_3d[2] = depth


         Hcam_imu = np.eye(4)
         Hcam_imu[:3,:3] = self.left_info_og.R
         Hcam_imu[:3,3] = self.left_info_og.T

         # Hcam_imu = inverse_matrix(Hcam_imu)

         # print("Cam IMU",Hcam_imu)

         Hfire_cam = np.eye(4)
         Hfire_cam[:3,3] = pos_3d
         print("pos_3d: ", pos_3d)  

         print("Fire Camera:",Hfire_cam[:3,3])
         print("IMU World:",self.H_imu_world[:3,3])
         # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
         print("Fire World",(self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3])
         # Hcam_world = Hfire_cam@(Hcam_imu @ self.H_imu_flu@self.H_imu_world)
         # print("Fire Camera",Hfire_cam[:3,3])
         # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
         # Hcam_world[:3,3] = Hcam_world[:3,3] + np.array([0,0,0.52])



         # pos_3d = Hcam_world[:3,3]

         # print("Pos_3d: ", pos_3d)

         pos_3d = (self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3]

         hotspots.append(pos_3d)
      
      # print(hotspots)
      
      return np.array(hotspots)
   
   def displayHotspots(self, hotspots):
      marker = Marker()
      marker.header.frame_id = "odom"
      marker.header.stamp = rospy.Time.now()
      marker.type = Marker.POINTS
      marker.action = Marker.ADD
      marker.pose.orientation.w = 1.0
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0

      for hotspot in hotspots:
         point = Point()
         point.x = hotspot[0]
         point.y = hotspot[1]
         point.z = hotspot[2]
         marker.points.append(point)
      
      self.hotspot_pub.publish(marker)

   def main_loop(self, event):
      if not(self.flag_got_left and self.flag_got_right and self.flag_got_odom):
         return
      
      print("Left-Right Time Delta: ", self.left_stamp.to_sec() - self.right_stamp.to_sec())
      print("Left-Odom Time Delta: ", self.left_stamp.to_sec() - self.odom_stamp.to_sec())
      self.flag_got_left = False
      self.flag_got_right = False
      self.flag_got_odom = False
      

      if self.left_stamp.to_sec() - self.right_stamp.to_sec() > 0.1 or self.left_stamp.to_sec() - self.odom_stamp.to_sec() > 0.1:
         return

      # perform stereo rectification
      R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
         cameraMatrix1=self.left_info.K,
         distCoeffs1=self.left_info.D,
         cameraMatrix2=self.right_info.K,
         distCoeffs2=self.right_info.D,
         imageSize=(640, 512),
         R=self.right_info.R.T,
         T=self.right_info.T,
         flags=cv2.CALIB_ZERO_DISPARITY,
         alpha=-1  # You might want to experiment with this parameter
      )

      map1x, map1y = cv2.initUndistortRectifyMap(self.left_info.K, self.left_info.D, R1, P1, (640, 512), cv2.CV_32FC1)
      map2x, map2y = cv2.initUndistortRectifyMap(self.right_info.K, self.right_info.D, R2, P2, (640, 512), cv2.CV_32FC1)

      left_rect = cv2.remap(self.left_image, map1x, map1y, cv2.INTER_LINEAR)
      right_rect = cv2.remap(self.right_image, map2x, map2y, cv2.INTER_LINEAR)


      # make both image intensities similar
      # left_rect = cv2.normalize(left_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
      # right_rect = cv2.normalize(right_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

      # self.display_image(np.hstack((left_rect, right_rect)))

      # Draw epipolar lines
      # # Step 1: Detect features use ORB
      # orb = cv2.ORB_create()
      # keypoints_left, descriptors_left = orb.detectAndCompute(left_rect, None)
      # keypoints_right, descriptors_right = orb.detectAndCompute(right_rect, None)

      # # Step 2: Match features
      # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
      # matches = bf.match(descriptors_left, descriptors_right)
      # matches = sorted(matches, key = lambda x:x.distance)

      # # Step 3: Draw the best matches and epipolar lines
      # img_matches = np.empty((max(left_rect.shape[0], right_rect.shape[0]), left_rect.shape[1]+right_rect.shape[1], 3), dtype=np.uint8)
      # cv2.drawMatches(left_rect, keypoints_left, right_rect, keypoints_right, matches[:50], img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

      # # Draw horizontal lines every 50 pixels to visualize epipolar lines
      # for y in range(0, img_matches.shape[0], 25):
      #    cv2.line(img_matches, (0, y), (img_matches.shape[1], y), (255, 0, 0))

      # # Display the images with epipolar lines
      # cv2.imshow('Epipolar Lines across Stereo Images', img_matches)
      # cv2.waitKey(1)
      # cv2.destroyAllWindows()

      left_temp = 5.0 * (0.0114 * left_rect - 79 - 32) / (9.0)
      right_temp = 5.0 * (0.0114 * right_rect - 79 - 32) / (9.0)
      left_centers = self.processImage(left_temp)
      right_centers = self.processImage(right_temp)

      # for center in left_centers:
      #    cv2.circle(left_temp, (int(center[0]), int(center[1])), 5, (0*255, 255, 255), -1)
      
      # for center in right_centers:
      #    cv2.circle(right_temp, (int(center[0]), int(center[1])), 5, (0*255, 255, 255), -1)

      self.display_image(np.hstack((left_temp, right_temp)))

      # call correspondence function
      # simple sorting rn, need to make more robust using NN
      left_centers, right_centers = self.get_corresspondences(left_centers, right_centers)

      if left_centers is None or right_centers is None:
         return
      # print("TEST")
      
      hotspots = self.getHotspots(left_centers, right_centers)

      self.displayHotspots(hotspots)


      
   def processExtrinsics(self):

      R_imu_flu_cam = euler_matrix(-np.pi/2, np.pi/2, 0 , 'rxyz')[:3,:3]

      # # Calculate relative rotation matrix
      relR = np.dot(self.right_info.R, self.left_info.R.T)

      # Calculate the translation of the left camera back to the world frame,
      # then find the relative translation vector
      relT = self.right_info.T - np.dot(relR, self.left_info.T)

      # Update the right camera's extrinsics to be relative to the left camera
      self.right_info.R = relR
      # Note: Depending on how you define your coordinate system,
      # you might need to adjust the order or signs of the components in relT.
      self.right_info.T = R_imu_flu_cam.T@relT

      # Set the left camera as the origin of the new coordinate system
      self.left_info.R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      self.left_info.T = np.array([0, 0, 0])

      print("--------------------")
      print("Processed Extrinsics:")
      print("Left Rotation:\n", self.left_info.R)
      print("Left Translation:\n",self.left_info.T)
      print("Right Rotation:\n", self.right_info.R)
      print("Right Translation:\n",self.right_info.T)
      print("--------------------")
      # print("Left R:", R_imu_flu_cam.T@self.left_info.R)
      # print("Left R:", R_imu_flu_cam.T@self.right_info.R)
   


if __name__ == "__main__":
   rospy.init_node("fire_localization")
   fire_localization = FireLocalization()
   rospy.spin()

   