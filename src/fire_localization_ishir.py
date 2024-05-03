import rospy
import cv2
import numpy as np
import math
import copy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf.transformations import euler_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler, inverse_matrix

import matplotlib.pyplot as plt

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

      self.left_info  = self.loadCameraParams("../params/thermal_left.yaml", "left")
      self.right_info = self.loadCameraParams("../params/thermal_right.yaml", "right")

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
      self.main_loop_timer = rospy.Timer(rospy.Duration(0.05), self.main_loop)

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
      cv2.imshow("Image test", image)
      cv2.waitKey(1)


   def processImage(self, image, flag):
      # binary thresholding
      image = cv2.inRange(image, 130, 255)

      ## Mask to be used for detecting features only on the hotspot
      # dilate the image
      image = cv2.dilate(image, np.ones((10,10), np.uint8), iterations=3)

      if flag == 1:
         cv2.imshow("Image", image)
         cv2.waitKey(1)
      # cv2.imshow("Image", image)
      # cv2.waitKey(1)

      countours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      centers = []

      # if countours is not None:
      #    cnt = countours
      #    cv2.drawContours(image, cnt , -1, (0,255,0), 3)
      #    cv2.imshow("Image", image)
      #    cv2.waitKey(1)
         # cv2.destroyAllWindows()


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
      if not clusters == [[]]:
         for cluster in clusters:
            cluster_center = np.mean(cluster, axis=0)
            cluster_centers.append(cluster_center)
         
      return image, cluster_centers
   
   # def get_corresspondences(self, left_centers, right_centers, thresh = 10): # thresh is in pixels
   #    if(len(left_centers) != len(right_centers)):
   #       return None, None
         
   #    left_centers_np = np.array(left_centers)
   #    right_centers_np = np.array(right_centers)

   #    idxs_L = np.argsort(left_centers_np, axis=0)
   #    idxs_R = np.argsort(right_centers_np, axis=0)

   #    sorted_L = np.take_along_axis(left_centers_np, idxs_L, axis=0)
   #    sorted_R = np.take_along_axis(right_centers_np, idxs_R, axis=0)

   #    # for i in range(sorted_L.shape[0]):
   #    #    if(np.hypot(sorted_L[i, 0] - sorted_R[i, 0], sorted_L[i, 1] - sorted_R[i, 1]) > thresh):
   #    #       return None, None

   #    return sorted_L, sorted_R


   def get_corresspondences(self, left_centers, right_centers, thresh = 10): # thresh is in pixels
      if(len(left_centers) != len(right_centers)):
         return_len = min(len(left_centers), len(right_centers))
      else:
         return_len = len(left_centers)
      left_centers_np = np.array(left_centers)
      right_centers_np = np.array(right_centers)
      idxs_L = np.argsort(left_centers_np, axis=0)
      idxs_R = np.argsort(right_centers_np, axis=0)
      sorted_L = np.take_along_axis(left_centers_np, idxs_L, axis=0)
      sorted_R = np.take_along_axis(right_centers_np, idxs_R, axis=0)
      # for i in range(sorted_L.shape[0]):
      #    if(np.hypot(sorted_L[i, 0] - sorted_R[i, 0], sorted_L[i, 1] - sorted_R[i, 1]) > thresh):
      #       return None, None
      return sorted_L[:return_len], sorted_R[:return_len]
   
   def getHotspots(self, left_centers, right_centers):
      hotspots = []

      # if left_centers == None or right_centers == None:
      #    return None

      if left_centers.shape == (1,) or right_centers.shape == (1,):
         return None
      # print(left_centers.shape, "left_centers")
      # print(right_centers.shape, "right_centers")

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
   



   def getHotspotsfrommatches(self, left_coords, right_coords):
      
      # hotspots = []
      # print(len(left_coords))
      # print(len(right_coords))
      # if left_centers.shape == (1,) or right_centers.shape == (1,):
      #    return None

      # for i in range(len(left_coords)):

      left_mean = np.mean(left_coords, axis=0)
      disp = np.mean( [left_coords[i][0] - right_coords[i][0] for i in range(len(left_coords))] )

      baseline = np.linalg.norm(self.left_info.T - self.right_info.T)
      depth = (self.left_info.K[0,0]*baseline)/disp

      pos_3d = np.zeros(3)
      # print(left_centers)
      pos_3d[0] = (left_mean[ 0] - self.left_info.K[0,2])*depth/self.left_info.K[0,0]
      pos_3d[1] = (left_mean[ 1] - self.left_info.K[1,2])*depth/self.left_info.K[1,1]
      pos_3d[2] = depth


      Hcam_imu = np.eye(4)
      Hcam_imu[:3,:3] = self.left_info_og.R
      Hcam_imu[:3,3] = self.left_info_og.T

      # Hcam_imu = inverse_matrix(Hcam_imu)

      # print("Cam IMU",Hcam_imu)

      Hfire_cam = np.eye(4)
      Hfire_cam[:3,3] = pos_3d
      # print("pos_3d: ", pos_3d)  

      # print("Fire Camera:",Hfire_cam[:3,3])
      # print("IMU World:",self.H_imu_world[:3,3])
      # # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
      # print("Fire World from Matches",(self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3])

      # Hcam_world = Hfire_cam@(Hcam_imu @ self.H_imu_flu@self.H_imu_world)
      # print("Fire Camera",Hfire_cam[:3,3])
      # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
      # Hcam_world[:3,3] = Hcam_world[:3,3] + np.array([0,0,0.52])



      # pos_3d = Hcam_world[:3,3]

      # print("Pos_3d: ", pos_3d)

      pos_3d = (self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3]

      # hotspots.append(pos_3d)
      
      # print(hotspots)
      
      return np.array(pos_3d)
   
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

   def get_matched_coords(self, matches, keypoints_left, keypoints_right):
      coords = []
      left_coords = []
      right_coords = []
      for match in matches:
         left_idx = match.queryIdx
         right_idx = match.trainIdx

         left_coord = keypoints_left[left_idx].pt
         right_coord = keypoints_right[right_idx].pt

         left_coords.append(left_coord)
         right_coords.append(right_coord)
      
      return left_coords, right_coords
   
   def display_coords(self, left_coords, right_coords, left_rect, right_rect):
      cv2.circle(left_rect, (int(left_coords[0][0]), int(left_coords[0][1])), 5, (0, 0, 255), -1)
      cv2.circle(right_rect, (int(right_coords[0][0]), int(right_coords[0][1])), 5, (0, 0, 255), -1)

      self.display_image(np.hstack((left_rect, right_rect)))

   def closest_center(self, kp, centers):
      min_dist = 100000
      min_center = None
      for center in centers:
         dist = np.linalg.norm(np.array(kp.pt) - np.array(center))
         if dist < min_dist:
            min_dist = dist
            min_center = center
      
      # print("Min center is", min_center, "out of ", centers)
      return min_center
   
   def get_keypoints_for_hotspots(self, keypoints_total, descriptors_total, centers):
      
      if len(centers) == 0:
         return None
      
      ## Create a list of lists to sort the keypoints according to the closest center  and also the descriptors for each keypoint
      centers = np.array(centers)
      centers = centers.tolist()
      keypoints_hotspots = [[] for i in range(len(centers))]
      descriptors_hotspots = [[] for i in range(len(centers))]
      for kp in keypoints_total:
         closest_center = self.closest_center(kp, centers)
         idx = centers.index(closest_center)
         keypoints_hotspots[idx].append(kp)
         kp_idx = keypoints_total.index(kp)
         # print('kp index', kp_idx)

         # print(descriptors_total[kp_idx])
         descriptors_hotspots[idx].append(descriptors_total[kp_idx])



      # for kp in keypoints_total:
      #    pt_extract = kp.pt
      #    closest_center = self.closest_center(kp, centers)
      return keypoints_hotspots, descriptors_hotspots
         

      
      # print()

   def get_matches_for_hotspots(self, descriptors_left, descriptors_right):
      bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
      matches = bf.match(descriptors_left, descriptors_right)
      matches = sorted(matches, key = lambda x:x.distance)

      return matches


   def get_good_matched_coords(self, left_coords, right_coords):

      best_left_coords=[]
      best_right_coords=[]

      for i in range(len(left_coords)):
         left_coord = left_coords[i]
         right_coord = right_coords[i]
         left_coord = np.array(left_coord)
         right_coord = np.array(right_coord)
         left_coord = np.append(left_coord, 1)
         right_coord = np.append(right_coord, 1)
         left_coord = left_coord.reshape(3,1)
         right_coord = right_coord.reshape(3,1)
         # F = np.dot(np.linalg.inv(self.left_info.K).T, np.dot(np.linalg.inv(self.left_info.R).T, np.dot(self.right_info.R, np.dot(self.right_info.K, np.linalg.inv(self.left_info.K)))))

         P1 = np.eye(4)
         P1[:3,:3] = self.left_info.R
         P1[:3,3] = self.left_info.T
         P2 = np.eye(4)
         P2[:3,:3] = self.right_info.R
         P2[:3,3] = self.right_info.T

         P = P1 @ np.linalg.inv(P2)
         E = self.skew(P[:3, 3]) @ P[:3, :3]

         F = np.linalg.inv(self.left_info.K).T @ E @ np.linalg.inv(self.right_info.K)
         F = F / F[2,2]


         # print(F)
         # print(left_coord)
         # print(right_coord)
         # print(np.dot(left_coord.T, np.dot(F, right_coord)))
         # print(np.linalg.norm(np.dot(left_coord.T, np.dot(F, right_coord))))
         # print("---------")
         # print(np.linalg.norm(np.dot(left_coord.T, np.dot(F, right_coord))))

         if np.linalg.norm(np.dot(right_coord.T, np.dot(F, left_coord))) < 0.3:
            # print("Matched")
            best_left_coords.append(left_coord)
            best_right_coords.append(right_coord)
         else:
            # print("Not Matched")
            pass
         # print("---------")

      print(len(best_left_coords), "points satisfied epipolar constraint out of ", len(left_coords))


      if len(best_left_coords) < 1:
         best_left_coords = left_coords
      if len(best_right_coords) < 1:
         best_right_coords = right_coords

      return best_left_coords, best_right_coords
   
   def display_centers_on_rectified(self, left_rect, right_rect, left_centers, right_centers):
      img6 = right_rect
      ##convert img6 to 3 channel
      img6 = cv2.cvtColor(img6, cv2.COLOR_GRAY2BGR)

      for center in right_centers:
         # print(kp.pt)
         cv2.circle(img6, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

      # cv2.drawKeypoints(img6, right_kp, None, color=(0,0,255), flags=0)

      cv2.imshow("Cetners in right", img6)
      cv2.waitKey(1)

      if (len(left_centers)!=len(right_centers)):
         print("Exiting as number of hotspots in left and right images are not same")
         # exit()

   def display_clustered_kp(self, left_rect, right_rect, left_kp, right_kp):
      img6 = left_rect
      ##convert img6 to 3 channel
      img6 = cv2.cvtColor(img6, cv2.COLOR_GRAY2BGR)

      for kp in left_kp:
         # print(kp.pt)
         cv2.circle(img6, (int(kp.pt[0]), int(kp.pt[1])), 5, (0, 0, 255), -1)

      # cv2.drawKeypoints(img6, left_kp, None, color=(0,0,255), flags=0)

      cv2.imshow("Keypoints in Left", img6)
      cv2.waitKey(1)

   def enhance_image(self, image):
         #after hist_99, do clache + bilateral filtering
      clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
      clahe = clahe.apply(image)
      bilateral = cv2.bilateralFilter(clahe, 5, 20, 15)
      return bilateral
   
   def hist_99(self, image):
         im_srt = np.sort(image.reshape(-1))
         upper_bound = im_srt[round(len(im_srt) * 0.99) - 1]
         lower_bound = im_srt[round(len(im_srt) * 0.01)]
         img = image
         img[img < lower_bound] = lower_bound
         img[img > upper_bound] = upper_bound
         image_out = ((img - lower_bound) / (upper_bound - lower_bound)) * 255.0
         image_out = image_out.astype(np.uint8)
         return image_out

   def skew(self, x):
      return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

   def main_loop(self, event):
      if not(self.flag_got_left and self.flag_got_right and self.flag_got_odom):
         # print(self.flag_got_left, self.flag_got_right, self.flag_got_odom)
         # print("Not all images received")
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

      left_temp = 5.0 * (0.0114 * left_rect - 79 - 32) / (9.0)
      right_temp = 5.0 * (0.0114 * right_rect - 79 - 32) / (9.0)


      # make both image intensities similar
      # left_rect = cv2.normalize(left_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
      # right_rect = cv2.normalize(right_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

      left_hist = self.hist_99(left_rect)
      right_hist = self.hist_99(right_rect)

      left_rect = self.enhance_image(left_hist)
      right_rect = self.enhance_image(right_hist)

      left_mask, left_centers = self.processImage(left_temp, 1)
      right_mask, right_centers = self.processImage(right_temp, 0)

      

      self.display_centers_on_rectified(left_rect, right_rect, left_centers, right_centers)

      left_centers, right_centers = self.get_corresspondences(left_centers, right_centers)
      #############################################################
      ## Checking for cluster centers by plotting for multiple hotspots
      # print(len(left_centers))
      # if left_centers is not None and right_centers is not None:
         
      #    if len(left_centers) > 0:

      #       for left_center in left_centers:
      #          print(left_center)
      #          cv2.circle(left_rect, (int(left_center[0]), int(left_center[1])), 5, (0, 255, 255), -1)

      #       for right_center in right_centers:
      #          cv2.circle(right_rect, (int(right_center[0]), int(right_center[1])), 5, (0, 255, 255), -1)

      #       cv2.imshow("cluster centers", left_rect)
      #       cv2.waitKey(1)
      #############################################################

      ## Creating mask from the normalized rectified images
      # left_mask, left_centers = self.processImage(left_rect, 1)
      # right_proc_img, right_centers = self.processImage(right_rect, 0)


      # self.display_image(np.hstack((left_rect, right_rect)))

      ## Use the mask on rectified images
      left_rect_masked = cv2.bitwise_and(left_rect, left_mask)
      right_rect_masked = cv2.bitwise_and(right_rect, right_mask)


      # cv2.imshow("Left Rect Masked", left_rect_masked)
      # # cv2.imshow("Right Rect Masked", right_rect_masked)
      # cv2.waitKey(1)

      

      # Draw epipolar lines
      # # Step 1: Detect features use ORB
      orb = cv2.ORB_create()
      # keypoints_left, descriptors_left = orb.detectAndCompute(left_rect, None)
      # keypoints_right, descriptors_right = orb.detectAndCompute(right_rect, None)

      keypoints_left, descriptors_left = orb.detectAndCompute(left_rect_masked, None)
      keypoints_right, descriptors_right = orb.detectAndCompute(right_rect_masked, None)

      img5 = left_rect
      cv2.drawKeypoints(img5, keypoints_left, None, color=(0,255,0), flags=0)
      cv2.imshow("Keypoints in Total", img5)
      cv2.waitKey(1)
      pt_uv = []
      # for pt in keypoints_left:
      #    pt_uv.append(pt.pt)
      ## if no descriptors found return None (if no hotspot visible)
      if descriptors_left is None or descriptors_right is None:
         return None
      
      ## Get keypoints for hotspots
      # left_keypoints_hotspots = self.get_keypoints_for_hotspots(keypoints_left, left_centers)
      # right_keypoints_hotspots = self.get_keypoints_for_hotspots(keypoints_right, right_centers)

      # print("Got", len(left_keypoints_hotspots), "hotspots in left image")
      # print("Got", len(right_keypoints_hotspots), "hotspots in right image")
      # print(left_keypoints_hotspots)

      # img3 = cv2.drawKeypoints(left_rect, left_keypoints_hotspots, None, color=(0,255,0), flags
      ############################ To check multiple hotspots 3 ############################
      # img3 = left_rect
      # for hotspots in left_keypoints_hotspots:
      # #    print(hotspots[0].pt)
      #    if hotspots == []:
      #       continue
      #    else:
      #       cv2.circle(img3, (int(hotspots[0].pt[0]), int(hotspots[0].pt[1])), 5, (255, 0, 0), -1)
      #    # cv2.drawKeypoints(img3, hotspots, left_rect, color=(0,255,0), flags=0)
      # #    cv2.circle(left_rect, (int(hotspots[0].pt[0]), int(hotspots[0].pt[1])), 5, (0, 255, 255), -1)
      # # img3 = cv2.drawKeypoints(left_rect, left_keypoints_hotspots[0], None, color=(0,255,0), flags=0)
      # cv2.imshow("Keypoints for two hotspots", img3)
      # cv2.waitKey(1)

      # print("Kps in", len(left_keypoints_hotspots), "hotspots", len(left_keypoints_hotspots))
      # print(keypoints_left[0].pt)
      #####################################################################
      ## extract keypoints and descriptors for hotspots using optical flow

      # pt_uv = np.array(pt_uv)
      # p1, st, err = cv2.calcOpticalFlowPyrLK(left_rect_masked, right_rect_masked, pt_uv, None, winSize = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

      # for i in range(len(p1)):
      #    cv2.circle(left_rect, (int(pt_uv[i][0]), int(pt_uv[i][1])), 5, (0, 255, 255), -1)
      #    cv2.circle(right_rect, (int(p1[i][0]), int(p1[i][1])), 5, (0, 255, 255), -1)

      # cv2.imshow("Optical Flow", np.hstack((left_rect, right_rect)))
      # cv2.waitKey(1)

      # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
      # # print(type(descriptors_left), 'is the type of descriptors')
      # matches = bf.match(descriptors_left, descriptors_right)
      # matches = sorted(matches, key = lambda x:x.distance)

      # # # Step 3: Draw the best matches and epipolar lines
      # img_matches = np.empty((max(left_rect.shape[0], right_rect.shape[0]), left_rect.shape[1]+right_rect.shape[1], 3), dtype=np.uint8)
      # cv2.drawMatches(left_rect, keypoints_left, right_rect, keypoints_right, matches[:50], img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

      # # # # Draw horizontal lines every 50 pixels to visualize epipolar lines
      # # # for y in range(0, img_matches.shape[0], 25):
      # # #    cv2.line(img_matches, (0, y), (img_matches.shape[1], y), (255, 0, 0))

      # # # Display the images with epipolar lines
      # cv2.imshow('Epipolar Lines across Stereo Images', img_matches)
      # cv2.waitKey(1)
      

      # left_coords, right_coords = self.get_matched_coords(matches, keypoints_left, keypoints_right)


      left_keypoints_hotspots, left_descriptors_hotspots = self.get_keypoints_for_hotspots(keypoints_left, descriptors_left, left_centers)
      right_keypoints_hotspots, right_descriptros_hotspots = self.get_keypoints_for_hotspots(keypoints_right, descriptors_right, right_centers)

      print("Got", len(left_keypoints_hotspots), "hotspots in left image")
      print("Got", len(right_keypoints_hotspots), "hotspots in right image")


      # img3 = left_rect
      # for hotspots in left_keypoints_hotspots:
      # #    print(hotspots[0].pt)
      #    if hotspots == []:
      #       continue
      #    else:
      #       # cv2.circle(img3, (int(hotspots[0][0]), int(hotspots[0][1])), 5, (255, 0, 0), -1)
      #       pass
      #    # cv2.drawKeypoints(img3, hotspots, left_rect, color=(0,255,0), flags=0)
      # #    cv2.circle(left_rect, (int(hotspots[0].pt[0]), int(hotspots[0].pt[1])), 5, (0, 255, 255), -1)
      # # img3 = cv2.drawKeypoints(left_rect, left_keypoints_hotspots[0], None, color=(0,255,0), flags=0)
      # cv2.imshow("Keypoints for two hotspots", img3)
      # cv2.waitKey(1)

      #####################################################################

      ## Loop over all the hotspots
      
      ## check if number of hotspots is same in both cameras
      mismatched_length = 0
      if len(left_keypoints_hotspots) != len(right_keypoints_hotspots):
         mismatched_length = min(len(left_keypoints_hotspots), len(right_keypoints_hotspots))

      else:
         mismatched_length = len(left_keypoints_hotspots)

      ##  hotspots accumulation
      hotspots_global_frame = []

      print("Number of hotspots found: ", mismatched_length)

      img4 = left_rect

      ## convert img4 from single channel to 3 channel
      img4 = cv2.cvtColor(img4, cv2.COLOR_GRAY2BGR)

      # print(img4.shape)


      for i in range(mismatched_length):

         left_kp = left_keypoints_hotspots[i]
         right_kp = right_keypoints_hotspots[i]
         descriptors_left = left_descriptors_hotspots[i]
         descriptors_right = right_descriptros_hotspots[i]

         # print(len(descriptors_left), 'descriptors of a single hotspot in left image')
         # print(len(descriptors_right), 'descriptors of a single hotspot in right image')
         ## print data type of descriptors_left and descriptors_right
         descriptors_left = np.array(descriptors_left)
         descriptors_right = np.array(descriptors_right)
         # print(type(descriptors_left[0]))
         # print(descriptors_left[0])
         # print(type(descriptors_right))

         print(len(left_kp), 'kps of a single hotspot in left image')
         print(len(right_kp), 'kps of a single hotspot in right image')

         print(len(descriptors_left), 'descriptors of a single hotspot in left image')
         print(len(descriptors_right), 'descriptors of a single hotspot in right image')

         if left_kp == [] or right_kp == []:
            continue
         
         min_len = min(len(left_kp), len(right_kp))
         # if len(left_kp) != len(right_kp):

         left_kp = left_kp[:min_len]
         right_kp = right_kp[:min_len]

         self.display_clustered_kp(left_rect, right_rect, left_kp, right_kp)


         min_len = min(len(descriptors_left), len(descriptors_right))
         descriptors_left = descriptors_left[:min_len]
         descriptors_right = descriptors_right[:min_len]

         bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
         matches = bf.match(descriptors_left, descriptors_right)
         matches = sorted(matches, key = lambda x:x.distance)

         # Step 3: Draw the best matches and epipolar lines
         img_matches = np.empty((max(left_rect.shape[0], right_rect.shape[0]), left_rect.shape[1]+right_rect.shape[1], 3), dtype=np.uint8)
         cv2.drawMatches(left_rect, left_kp, right_rect, right_kp, matches[:50], img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

         # # Draw horizontal lines every 50 pixels to visualize epipolar lines
         # for y in range(0, img_matches.shape[0], 25):
         #    cv2.line(img_matches, (0, y), (img_matches.shape[1], y), (255, 0, 0))

         # Display the images with epipolar lines
         cv2.imshow('Epipolar Lines across Stereo Images', img_matches)
         cv2.waitKey(1)

         left_coords, right_coords = self.get_matched_coords(matches, left_kp, right_kp)

         # cv2.circle(img4, (int(left_centers[i][0]), int(left_centers[i][1])), 5, (255*i, 0, 255*(1-i)), -1)
         # cv2.circle(img4, (int(left_kp[0][0]), int(left_kp[0][1])), 5, (255*i, 0, 255*(1-i)), -1)
         
         
         # matches = self.get_matches_for_hotspots(, )

         # left_coords, right_coords = self.get_matched_coords(matches, left_kp, right_kp)

         # best_left_coords=[]
         # best_right_coords=[]

         # print(left_coords)

         # print(right_coords)

         #get best matches if the point on right image is close to the epipolar line of point on left image
         best_left_coords, best_right_coords = self.get_good_matched_coords(left_coords, right_coords)




         ## get the 3d pose of the hotspot
         pose_3d = self.getHotspotsfrommatches(best_left_coords, best_right_coords)

         # print('One of the hotspot is at', pose_3d)
         hotspots_global_frame.append(pose_3d)


      cv2.imshow("Keypoints for two hotspots", img4)
      cv2.waitKey(1)





      #####################################################################
      #####################################################################
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

      ##########################################################################
      ##########################################################################

      # self.display_image(np.hstack((left_rect, right_rect)))

      # img2 = cv2.drawKeypoints(left_rect, keypoints_left, None, color=(0,255,0), flags=0)
      # cv2.imshow("Keypoints", img2)
      # cv2.waitKey(1)


      # breakpoint

      # left_coords, right_coords = self.get_matched_coords(matches, keypoints_left, keypoints_right)

      ## For viewing the matched coordinates
      # self.display_coords(left_coords, right_coords, left_rect, right_rect) ## not needed now, remove
      # take first 10 matched coordinates
      # coords = coords[:10]

      # print(left_coords)
      # left_coords = left_coords[:10]
      # right_coords = right_coords[:10]
      # best_left_coords=[]
      # best_right_coords=[]

      #get best matches if the point on right image is close to the epipolar line of point on left image

      # for i in range(len(left_coords)):
      #    left_coord = left_coords[i]
      #    right_coord = right_coords[i]
      #    left_coord = np.array(left_coord)
      #    right_coord = np.array(right_coord)
      #    left_coord = np.append(left_coord, 1)
      #    right_coord = np.append(right_coord, 1)
      #    left_coord = left_coord.reshape(3,1)
      #    right_coord = right_coord.reshape(3,1)
      #    # F = np.dot(np.linalg.inv(self.left_info.K).T, np.dot(np.linalg.inv(self.left_info.R).T, np.dot(self.right_info.R, np.dot(self.right_info.K, np.linalg.inv(self.left_info.K)))))

      #    P1 = np.eye(4)
      #    P1[:3,:3] = self.left_info.R
      #    P1[:3,3] = self.left_info.T
      #    P2 = np.eye(4)
      #    P2[:3,:3] = self.right_info.R
      #    P2[:3,3] = self.right_info.T

      #    P = P1 @ np.linalg.inv(P2)
      #    E = self.skew(P[:3, 3]) @ P[:3, :3]

      #    F = np.linalg.inv(self.left_info.K).T @ E @ np.linalg.inv(self.right_info.K)
      #    F = F / F[2,2]


      #    # print(F)
      #    # print(left_coord)
      #    # print(right_coord)
      #    # print(np.dot(left_coord.T, np.dot(F, right_coord)))
      #    # print(np.linalg.norm(np.dot(left_coord.T, np.dot(F, right_coord))))
      #    # print("---------")
      #    # print(np.linalg.norm(np.dot(left_coord.T, np.dot(F, right_coord))))

      #    if np.linalg.norm(np.dot(right_coord.T, np.dot(F, left_coord))) < 0.3:
      #       # print("Matched")
      #       best_left_coords.append(left_coord)
      #       best_right_coords.append(right_coord)
      #    else:
      #       # print("Not Matched")
      #       pass
      #    # print("---------")

      # print(len(best_left_coords), "points satisfied epipolar constraint out of ", len(left_coords))


      # if len(best_left_coords) < 1:
      #    best_left_coords = left_coords
      # if len(best_right_coords) < 1:
      #    best_right_coords = right_coords

      # left_coords = left_coords[:10]
      # right_coords = right_coords[:10]

      # good = []
      # for m,n in matches:
      #    if m.distance < 0.75*n.distance:
      #       good.append([m])
      
      # cv2.drawMatchesKnn expects list of lists as matches.
      # img3 = cv2.drawMatchesKnn(left_rect, keypoints_left, right_rect, keypoints_right, good,flags=2)

      # plt.imshow(img3),plt.show()

      
      #######################################################################33333
      ############################################################################
      # left_temp = 5.0 * (0.0114 * left_rect - 79 - 32) / (9.0)
      # right_temp = 5.0 * (0.0114 * right_rect - 79 - 32) / (9.0)
      # left_centers = self.processImage(left_temp)
      # right_centers = self.processImage(right_temp)

      # for center in left_centers:
      #    cv2.circle(left_temp, (int(center[0]), int(center[1])), 5, (0*255, 255, 255), -1)
      
      # for center in right_centers:
      #    cv2.circle(right_temp, (int(center[0]), int(center[1])), 5, (0*255, 255, 255), -1)

      ## Display the 
      # self.display_image(np.hstack((left_temp, right_temp)))

      # call correspondence function
      # simple sorting rn, need to make more robust using NN
      # left_centers, right_centers = self.get_corresspondences(left_centers, right_centers)

      # if left_centers is None or right_centers is None:
      #    return
      # # print("TEST")
      # # if left_centers is None and right_centers is None:
      # #    print("No Hotspots in view")
      # #    return
      
      # hotspots_matched = self.getHotspotsfrommatches(best_left_coords, best_right_coords, left_centers, right_centers)
      # hotspots = self.getHotspots(left_centers, right_centers)

      print("Hotspots in Global Frame: ", hotspots_global_frame)

      if hotspots_global_frame is not None:
         self.displayHotspots(hotspots_global_frame)


      
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

   