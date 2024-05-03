# import rospy
# import cv2
# import numpy as np
# import math
# import copy

# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# from geometry_msgs.msg import PoseArray, Pose

# from geometry_msgs.msg import Point
# from visualization_msgs.msg import Marker
# from tf.transformations import euler_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler, inverse_matrix

# import matplotlib.pyplot as plt

# import message_filters
# import yaml


# # TODO(@Shashwat): Change these to ROS Params 
# # Debugger prints
# print_extrinsics_ = False
# print_camera_params_ = False
# print_timestamp_diff_ = False
# print_descriptors_keypoints = False
# print_hotspot_location  = True


# display_process_img = False
# display_contours_ = False
# display_epipolar_lines = False
# display_keypoints = False
# display_hotspot_centres = False

# # Param to Perform Hist99 & Image Enhancement
# perform_img_enhancement_ = True
# # Publisher for enhanced Img(will only work if img enhancement is True)
# publish_enhanced_img = True

# # Main Loop Time
# loop_timer_ = 0.10


# class CameraParams:
#    def __init__(self, name) -> None:
#       self.name = name
#       self.K = np.ones((3,3))
#       self.D = np.ones((5,1))
#       self.R = np.ones((3,3))
#       self.T = np.zeros((3,1))

# class FireLocalization:

#    def __init__(self):
#       self.image = None

#       self.bridge = CvBridge()
#       self.H_imu_flu = np.eye(4)
#       # rotation about Z axis by 180 degrees
#       self.H_imu_flu[:3,:3] = euler_matrix(0, 0, np.pi,'rxyz')[:3,:3]


#       # Load Camera Params & Process Extrinsics
#       self.left_info  = self.loadCameraParams("../params/thermal_left.yaml", "left")
#       self.right_info = self.loadCameraParams("../params/thermal_right.yaml", "right")
#       self.left_info_og = CameraParams("left")
#       self.right_info_og = CameraParams("right")

#       self.left_info_og.K = self.left_info.K
#       self.left_info_og.D = self.left_info.D
#       self.left_info_og.R = self.left_info.R
#       self.left_info_og.T = self.left_info.T

#       self.processExtrinsics()
#       self.H_imu_world = np.eye(4)


#       # Time Sync Params
#       self.flag_got_left = False
#       self.flag_got_right = False
#       self.flag_got_odom = False

#       self.left_stamp = None
#       self.right_stamp = None
#       self.odom_stamp = None

#       # ROS Pubs and Subs
#       self.left_sub = rospy.Subscriber("/thermal_left/image", Image, self.left_cb)
#       self.right_sub = rospy.Subscriber("/thermal_right/image", Image, self.right_cb)
#       self.odom_sub = rospy.Subscriber("/mso_estimator/pose_imu_transformed", PoseStamped, self.odom_cb)
#       self.hotspot_pub = rospy.Publisher("/hotspots", Marker, queue_size=10)
#       self.hotspot_array_pub = rospy.Publisher("/hotspots/array", PoseArray, queue_size=10)

#       if publish_enhanced_img is True:
#          self.thermal_left_enhanced_pub = rospy.Publisher("thermal_left/enhanced_img", Image, queue_size=10)
#          self.thermal_right_enhanced_pub = rospy.Publisher("thermal_right/enhanced_img", Image, queue_size=10)
#          self.cv_bridge = CvBridge()

#       self.main_loop_timer = rospy.Timer(rospy.Duration(0.05), self.main_loop)


#    # ROS Subscriber Callbacks
#    def left_cb(self, msg):
#       self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#       self.left_stamp = msg.header.stamp
#       self.flag_got_left = True
#    def right_cb(self, msg):
#       self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#       self.right_stamp = msg.header.stamp
#       self.flag_got_right = True
#    def odom_cb(self, msg):
#       self.odom_pose = msg
#       self.odom_stamp = msg.header.stamp
#       rpy = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], axes='rxyz')
#       rpy = list(rpy)
#       rpy[0] = -rpy[0]
#       rpy[1] = -rpy[1]
#       quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='rxyz')
#       self.H_imu_world[:3,:3] = quaternion_matrix(quat)[:3,:3]
#       self.H_imu_world[:3,3] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
#       self.flag_got_odom = True  
   
#    # Camera Params Loader
#    def loadCameraParams(self, kalibr_path, camera_name):
#       with open(kalibr_path, 'r') as file:
#          print("Loaded params from file:", kalibr_path, "for camera:", camera_name)
#          cam_params = yaml.load(file, Loader=yaml.FullLoader)
#          camera = cam_params["cam0"]
#          cam_info = CameraParams(camera_name)
#          fx, fy, cx, cy = camera["intrinsics"]
#          cam_info.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
#          cam_info.D = np.array(camera["distortion_coeffs"]+[0.0]).reshape(5,1)
#          H = np.linalg.inv(np.array(camera["T_cam_imu"]).reshape(4,4))
#          H = self.H_imu_flu@H
#          cam_info.R = H[:3,:3]
#          cam_info.T = H[:3,3]
#          if print_camera_params_ is True:
#             print("--------------------")
#             print("Camera Name: ", camera_name)
#             print("Rotation Matrix: ", cam_info.R)
#             print("Translation Vector: ", cam_info.T)
#             print("--------------------")
#             # print("Rotation Matrix: ", cam_info.R)
#             # print("Translation Vector: ", cam_info.T)
#          return cam_info
      
#    def processImage(self, image):
#       # binary thresholding
#       image = cv2.inRange(image, 130, 255)

#       ## Mask to be used for detecting features only on the hotspot
#       # dilate the image
#       image = cv2.dilate(image, np.ones((10,10), np.uint8), iterations=2)

#       if display_process_img is True:
#          cv2.imshow("Processed Image", image)
#          cv2.waitKey(1)

#       countours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#       centers = []

#       if display_contours_ is True: 
#          if countours is not None:
#             cnt = countours
#             cv2.drawContours(image, cnt , -1, (0,255,0), 3)
#             cv2.imshow("Contours", image)
#             cv2.waitKey(1)
            
#       for contour in countours:
#          M = cv2.moments(contour)
#          if M["m00"] == 0:
#             continue
#          cX = int(M["m10"] / M["m00"])
#          cY = int(M["m01"] / M["m00"])
#          centers.append((cX, cY))
      
#       # cluster the centers
#       clusters = [[]]
#       for center in centers:
#          if len(clusters[-1]) == 0:
#             clusters[-1].append(center)
#          else:
#             if np.linalg.norm(np.array(center) - np.array(clusters[-1][-1])) < 50:
#                clusters[-1].append(center)
#             else:
#                clusters.append([center])
      
#       # find the center of each cluster
#       cluster_centers = []
#       if not clusters == [[]]:
#          for cluster in clusters:
#             cluster_center = np.mean(cluster, axis=0)
#             cluster_centers.append(cluster_center)
         
#       return image, cluster_centers

#    def get_corresspondences(self, left_centers, right_centers, thresh = 10): # thresh is in pixels
#       if(len(left_centers) != len(right_centers)):
#          return_len = min(len(left_centers), len(right_centers))
#       else:
#          return_len = len(left_centers)
#       left_centers_np = np.array(left_centers)
#       right_centers_np = np.array(right_centers)
#       idxs_L = np.argsort(left_centers_np, axis=0)
#       idxs_R = np.argsort(right_centers_np, axis=0)
#       sorted_L = np.take_along_axis(left_centers_np, idxs_L, axis=0)
#       sorted_R = np.take_along_axis(right_centers_np, idxs_R, axis=0)
#       return sorted_L[:return_len], sorted_R[:return_len]
   

#    def getHotspotsfrommatches(self, left_coords, right_coords):
#       left_mean = np.mean(left_coords, axis=0)
#       disp = np.mean( [left_coords[i][0] - right_coords[i][0] for i in range(len(left_coords))] )

#       baseline = np.linalg.norm(self.left_info.T - self.right_info.T)
#       depth = (self.left_info.K[0,0]*baseline)/disp

#       pos_3d = np.zeros(3)
#       # print(left_centers)
#       pos_3d[0] = (left_mean[ 0] - self.left_info.K[0,2])*depth/self.left_info.K[0,0]
#       pos_3d[1] = (left_mean[ 1] - self.left_info.K[1,2])*depth/self.left_info.K[1,1]
#       pos_3d[2] = depth


#       Hcam_imu = np.eye(4)
#       Hcam_imu[:3,:3] = self.left_info_og.R
#       Hcam_imu[:3,3] = self.left_info_og.T

#       Hfire_cam = np.eye(4)
#       Hfire_cam[:3,3] = pos_3d

#       if print_hotspot_location is True:
#          print("Fire Camera:",Hfire_cam[:3,3])
#          print("IMU World:",self.H_imu_world[:3,3])
#          # # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
#          print("Fire World from Matches",(self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3])

#       pos_3d = (self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3]
#       pos_imu_3d = (Hcam_imu@Hfire_cam)[:3,3]

#       return np.array(pos_3d) , np.array(pos_imu_3d)
   
#    def display_image(self, image):
#       # normalize the image
#       cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
#       image = np.uint8(image)
#       cv2.imshow("Image test", image)
#       cv2.waitKey(1)

   
#    def displayHotspots(self, hotspots):
#       marker = Marker()
#       marker.header.frame_id = "odom"
#       marker.header.stamp = rospy.Time.now()
#       marker.type = Marker.POINTS
#       marker.action = Marker.ADD
#       marker.pose.orientation.w = 1.0
#       marker.scale.x = 0.2
#       marker.scale.y = 0.2
#       marker.scale.z = 0.2
#       marker.color.a = 1.0
#       marker.color.r = 1.0
#       marker.color.g = 0.0
#       marker.color.b = 0.0

#       for hotspot in hotspots:
#          point = Point()
#          point.x = hotspot[0]
#          point.y = hotspot[1]
#          point.z = hotspot[2]
#          marker.points.append(point)
      
#       self.hotspot_pub.publish(marker)

#    def get_matched_coords(self, matches, keypoints_left, keypoints_right):
#       coords = []
#       left_coords = []
#       right_coords = []
#       for match in matches:
#          left_idx = match.queryIdx
#          right_idx = match.trainIdx

#          left_coord = keypoints_left[left_idx].pt
#          right_coord = keypoints_right[right_idx].pt

#          left_coords.append(left_coord)
#          right_coords.append(right_coord)
      
#       return left_coords, right_coords
   
#    def display_coords(self, left_coords, right_coords, left_rect, right_rect):
#       cv2.circle(left_rect, (int(left_coords[0][0]), int(left_coords[0][1])), 5, (0, 0, 255), -1)
#       cv2.circle(right_rect, (int(right_coords[0][0]), int(right_coords[0][1])), 5, (0, 0, 255), -1)

#       self.display_image(np.hstack((left_rect, right_rect)))

#    def closest_center(self, kp, centers):
#       min_dist = 100000
#       min_center = None
#       for center in centers:
#          dist = np.linalg.norm(np.array(kp.pt) - np.array(center))
#          if dist < min_dist:
#             min_dist = dist
#             min_center = center
      
#       # print("Min center is", min_center, "out of ", centers)
#       return min_center
   
#    def get_keypoints_for_hotspots(self, keypoints_total, descriptors_total, centers):
      
#       if len(centers) == 0:
#          return None
      
#       ## Create a list of lists to sort the keypoints according to the closest center  and also the descriptors for each keypoint
#       centers = np.array(centers)
#       centers = centers.tolist()
#       keypoints_hotspots = [[] for i in range(len(centers))]
#       descriptors_hotspots = [[] for i in range(len(centers))]
#       for kp in keypoints_total:
#          closest_center = self.closest_center(kp, centers)
#          idx = centers.index(closest_center)
#          keypoints_hotspots[idx].append(kp)
#          kp_idx = keypoints_total.index(kp)
#          descriptors_hotspots[idx].append(descriptors_total[kp_idx])

#       return keypoints_hotspots, descriptors_hotspots
      
#    def get_matches_for_hotspots(self, descriptors_left, descriptors_right):
#       bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#       matches = bf.match(descriptors_left, descriptors_right)
#       matches = sorted(matches, key = lambda x:x.distance)

#       return matches


#    def get_good_matched_coords(self, left_coords, right_coords):

#       best_left_coords=[]
#       best_right_coords=[]

#       for i in range(len(left_coords)):
#          left_coord = left_coords[i]
#          right_coord = right_coords[i]
#          left_coord = np.array(left_coord)
#          right_coord = np.array(right_coord)
#          left_coord = np.append(left_coord, 1)
#          right_coord = np.append(right_coord, 1)
#          left_coord = left_coord.reshape(3,1)
#          right_coord = right_coord.reshape(3,1)
#          # F = np.dot(np.linalg.inv(self.left_info.K).T, np.dot(np.linalg.inv(self.left_info.R).T, np.dot(self.right_info.R, np.dot(self.right_info.K, np.linalg.inv(self.left_info.K)))))

#          P1 = np.eye(4)
#          P1[:3,:3] = self.left_info.R
#          P1[:3,3] = self.left_info.T
#          P2 = np.eye(4)
#          P2[:3,:3] = self.right_info.R
#          P2[:3,3] = self.right_info.T

#          P = P1 @ np.linalg.inv(P2)
#          E = self.skew(P[:3, 3]) @ P[:3, :3]

#          F = np.linalg.inv(self.left_info.K).T @ E @ np.linalg.inv(self.right_info.K)
#          F = F / F[2,2]

#          if np.linalg.norm(np.dot(right_coord.T, np.dot(F, left_coord))) < 0.3:
#             # print("Matched")
#             best_left_coords.append(left_coord)
#             best_right_coords.append(right_coord)
#          else:
#             # print("Not Matched")
#             pass
#          # print("---------")


#       # print(len(best_left_coords), "points satisfied epipolar constraint out of ", len(left_coords))

#       if len(best_left_coords) < 1:
#          best_left_coords = left_coords
#       if len(best_right_coords) < 1:
#          best_right_coords = right_coords

#       return best_left_coords, best_right_coords
   
#    def display_centers_on_rectified(self, left_rect, right_rect, left_centers, right_centers):
#       ##convert img6 to 3 channel
#       img6 = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR)
#       for center in right_centers:
#          cv2.circle(img6, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

#       cv2.imshow("Centres in right", img6)

#       ##convert img6 to 3 channel
#       img7 = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
#       for center in left_centers:
#          cv2.circle(img7, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

#       cv2.imshow("Centres in left", img7)
      
#       cv2.waitKey(1)


#    def display_clustered_kp(self, left_rect, right_rect, left_kp, right_kp):
#       ##convert left to 3 channel
#       img6 = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
#       for kp in left_kp:
#          # print(kp.pt)
#          cv2.circle(img6, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 0, 255), -1)

#       ##convert left to 3 channel
#       img7 = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR)
#       for kp in right_kp:
#          # print(kp.pt)
#          cv2.circle(img7, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 255, 0), -1)

#       cv2.imshow("Keypoints in Left", img6)
#       cv2.imshow("Keypoints in Right", img7)
#       cv2.waitKey(1)



#    def enhance_image(self, image):
#          #after hist_99, do clache + bilateral filtering
#       clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
#       clahe = clahe.apply(image)
#       bilateral = cv2.bilateralFilter(clahe, 5, 20, 15)
#       return bilateral
   
#    def hist_99(self, image):
#          im_srt = np.sort(image.reshape(-1))
#          upper_bound = im_srt[round(len(im_srt) * 0.99) - 1]
#          lower_bound = im_srt[round(len(im_srt) * 0.01)]
#          img = image
#          img[img < lower_bound] = lower_bound
#          img[img > upper_bound] = upper_bound
#          image_out = ((img - lower_bound) / (upper_bound - lower_bound)) * 255.0
#          image_out = image_out.astype(np.uint8)
#          return image_out


#    # Perform Histogram Normalization and Image Enhancement
#    def perform_image_enhancement(self, image_):
#       # Histogram Normalization and then Image enhancement
#       return self.enhance_image(self.hist_99(image_))

#    def skew(self, x):
#       return np.array([[0, -x[2], x[1]],
#                      [x[2], 0, -x[0]],
#                      [-x[1], x[0], 0]])

#    def main_loop(self, event):
#       if not(self.flag_got_left and self.flag_got_right and self.flag_got_odom):
#          print("[ERROR] Subscriber Flags are not set, Returning")
#          # print(self.flag_got_left, self.flag_got_right, self.flag_got_odom)
#          return

#       self.flag_got_left = False
#       self.flag_got_right = False
#       self.flag_got_odom = False
      
#       if print_timestamp_diff_ == True:
#          print("Left-Right Time Delta: ", self.left_stamp.to_sec() - self.right_stamp.to_sec())
#          print("Left-Odom Time Delta: ", self.left_stamp.to_sec() - self.odom_stamp.to_sec())
      
#       # If Difference is higher than set limit, return 
#       if self.left_stamp.to_sec() - self.right_stamp.to_sec() > 0.1 or self.left_stamp.to_sec() - self.odom_stamp.to_sec() > 0.1:
#          # print("[ERROR] Subscriber Timestamps Different, Returning")
#          return


#       # perform stereo rectification
#       R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
#          cameraMatrix1=self.left_info.K,
#          distCoeffs1=self.left_info.D,
#          cameraMatrix2=self.right_info.K,
#          distCoeffs2=self.right_info.D,
#          imageSize=(640, 512),
#          R=self.right_info.R.T,
#          T=self.right_info.T,
#          flags=cv2.CALIB_ZERO_DISPARITY,
#          alpha=-1  # You might want to experiment with this parameter
#       )

#       map1x, map1y = cv2.initUndistortRectifyMap(self.left_info.K, self.left_info.D, R1, P1, (640, 512), cv2.CV_32FC1)
#       map2x, map2y = cv2.initUndistortRectifyMap(self.right_info.K, self.right_info.D, R2, P2, (640, 512), cv2.CV_32FC1)
#       left_rect = cv2.remap(self.left_image, map1x, map1y, cv2.INTER_LINEAR)
#       right_rect = cv2.remap(self.right_image, map2x, map2y, cv2.INTER_LINEAR)

#       # Calculate Temp Mask on Rectified Images
#       left_temp = 5.0 * (0.0114 * left_rect - 79 - 32) / (9.0)
#       right_temp = 5.0 * (0.0114 * right_rect - 79 - 32) / (9.0)
#       left_mask, left_centers = self.processImage(left_temp)
#       right_mask, right_centers = self.processImage(right_temp)


#       # If Image Enhacement not provided then Normalize
#       if perform_img_enhancement_ is True: 
#          # Perform Histogram Normalization and Image Enhancement
#          left_rect = self.perform_image_enhancement(left_rect)
#          right_rect = self.perform_image_enhancement(right_rect)

#          left_msg_ = self.cv_bridge.cv2_to_imgmsg(left_rect, "mono8")
#          right_msg_ = self.cv_bridge.cv2_to_imgmsg(right_rect, "mono8")
#          self.thermal_left_enhanced_pub.publish(left_msg_)
#          self.thermal_right_enhanced_pub.publish(right_msg_)
         
#       else:
#          # make both image intensities similar
#          left_rect = cv2.normalize(left_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
#          right_rect = cv2.normalize(right_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

#       if display_hotspot_centres is True:
#          self.display_centers_on_rectified(left_rect, right_rect, left_centers, right_centers)

#       left_centers, right_centers = self.get_corresspondences(left_centers, right_centers)

#       ## Use the mask on rectified images
#       left_rect_masked = cv2.bitwise_and(left_rect, left_mask)
#       right_rect_masked = cv2.bitwise_and(right_rect, right_mask)


#       # # Step 1: Detect features use ORB
#       orb = cv2.ORB_create()
      
#       keypoints_left, descriptors_left = orb.detectAndCompute(left_rect_masked, None)
#       keypoints_right, descriptors_right = orb.detectAndCompute(right_rect_masked, None)
#       ## if no descriptors found return None (if no hotspot visible)
#       if descriptors_left is None or descriptors_right is None:
#          return None
      
#       left_keypoints_hotspots, left_descriptors_hotspots = self.get_keypoints_for_hotspots(keypoints_left, descriptors_left, left_centers)
#       right_keypoints_hotspots, right_descriptros_hotspots = self.get_keypoints_for_hotspots(keypoints_right, descriptors_right, right_centers)

#       # print("Got", len(left_keypoints_hotspots), "hotspots in left image")
#       # print("Got", len(right_keypoints_hotspots), "hotspots in right image")

#       ## Loop over all the hotspots
      
#       ## check if number of hotspots is same in both cameras
#       mismatched_length = 0
#       if len(left_keypoints_hotspots) != len(right_keypoints_hotspots):
#          mismatched_length = min(len(left_keypoints_hotspots), len(right_keypoints_hotspots))

#       else:
#          mismatched_length = len(left_keypoints_hotspots)

#       ##  hotspots accumulation
#       hotspots_global_frame = []

#       print("Number of hotspots found: ", mismatched_length)

#       for i in range(mismatched_length):

#          left_kp = left_keypoints_hotspots[i]
#          right_kp = right_keypoints_hotspots[i]
#          descriptors_left = left_descriptors_hotspots[i]
#          descriptors_right = right_descriptros_hotspots[i]
         
#          descriptors_left = np.array(descriptors_left)
#          descriptors_right = np.array(descriptors_right)
         
#          if print_descriptors_keypoints is True:
#             print(len(left_kp), 'kps of a single hotspot in left image')
#             print(len(right_kp), 'kps of a single hotspot in right image')
#             print(len(descriptors_left), 'descriptors of a single hotspot in left image')
#             print(len(descriptors_right), 'descriptors of a single hotspot in right image')

#          if left_kp == [] or right_kp == []:
#             continue
         
#          min_len = min(len(left_kp), len(right_kp))
#          # if len(left_kp) != len(right_kp):

#          left_kp = left_kp[:min_len]
#          right_kp = right_kp[:min_len]

#          if display_keypoints is True:
#             self.display_clustered_kp(left_rect, right_rect, left_kp, right_kp)


#          min_len = min(len(descriptors_left), len(descriptors_right))
#          descriptors_left = descriptors_left[:min_len]
#          descriptors_right = descriptors_right[:min_len]

#          bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#          matches = bf.match(descriptors_left, descriptors_right)
#          matches = sorted(matches, key = lambda x:x.distance)

#          if display_epipolar_lines is True:
#             # Step 3: Draw the best matches and epipolar lines
#             img_matches = np.empty((max(left_rect.shape[0], right_rect.shape[0]), left_rect.shape[1]+right_rect.shape[1], 3), dtype=np.uint8)
#             cv2.drawMatches(left_rect, left_kp, right_rect, right_kp, matches[:50], img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

#             # # Draw horizontal lines every 50 pixels to visualize epipolar lines
#             # for y in range(0, img_matches.shape[0], 25):
#             #    cv2.line(img_matches, (0, y), (img_matches.shape[1], y), (255, 0, 0))

#             # Display the images with epipolar lines
#             cv2.imshow('Epipolar Lines across Stereo Images', img_matches)
#             cv2.waitKey(1)
         
#          left_coords, right_coords = self.get_matched_coords(matches, left_kp, right_kp)

#          #get best matches if the point on right image is close to the epipolar line of point on left image
#          best_left_coords, best_right_coords = self.get_good_matched_coords(left_coords, right_coords)


#          ## get the 3d pose of the hotspot
#          pose_3d, pose_imu_3d = self.getHotspotsfrommatches(best_left_coords, best_right_coords)

#          # print('One of the hotspot is at', pose_3d)
#          if np.linalg.norm(pose_imu_3d) < 5:
#             hotspots_global_frame.append(pose_3d)

#       # print("Hotspots in Global Frame: ", hotspots_global_frame)

#       if hotspots_global_frame is not None:
#          self.displayHotspots(hotspots_global_frame)
#          self.publishHotspots(hotspots_global_frame)


#    def publishHotspots(self, hotspots):
#       hostpots_msg_ = PoseArray()
#       hostpots_msg_.header.stamp = rospy.Time.now()
#       hostpots_msg_.header.frame_id = 'odom'
      
#       # Publish 1st array point as drone location
#       drone_location = Pose()
      
#       drone_location.position.x = self.H_imu_world[0, 3]
#       drone_location.position.y = self.H_imu_world[1, 3]
#       drone_location.position.z = self.H_imu_world[2, 3]
#       hostpots_msg_.poses.append(drone_location)

#       for hotspot in hotspots:
#          pose_ = Pose()
#          pose_.position.x = hotspot[0]
#          pose_.position.y = hotspot[1]
#          pose_.position.z = hotspot[2]
#          hostpots_msg_.poses.append(pose_)
      
#       self.hotspot_array_pub.publish(hostpots_msg_)

      
#    # Extrincs Processor
#    def processExtrinsics(self):
#       R_imu_flu_cam = euler_matrix(-np.pi/2, np.pi/2, 0 , 'rxyz')[:3,:3]
#       # # Calculate relative rotation matrix
#       relR = np.dot(self.right_info.R, self.left_info.R.T)

#       # Calculate the translation of the left camera back to the world frame,
#       # then find the relative translation vector
#       relT = self.right_info.T - np.dot(relR, self.left_info.T)

#       # Update the right camera's extrinsics to be relative to the left camera
#       self.right_info.R = relR
#       # Note: Depending on how you define your coordinate system,
#       # you might need to adjust the order or signs of the components in relT.
#       self.right_info.T = R_imu_flu_cam.T@relT

#       # Set the left camera as the origin of the new coordinate system
#       self.left_info.R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
#       self.left_info.T = np.array([0, 0, 0])

#       if print_extrinsics_ is True:
#          print("--------------------")
#          print("Processed Extrinsics:")
#          print("Left Rotation:\n", self.left_info.R)
#          print("Left Translation:\n",self.left_info.T)
#          print("Right Rotation:\n", self.right_info.R)
#          print("Right Translation:\n",self.right_info.T)
#          print("--------------------")
#          # print("Left R:", R_imu_flu_cam.T@self.left_info.R)
#          # print("Left R:", R_imu_flu_cam.T@self.right_info.R)
   


# if __name__ == "__main__":
#    rospy.init_node("fire_localization")
#    fire_localization = FireLocalization()
#    rospy.spin()

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import copy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf.transformations import euler_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler, inverse_matrix

import matplotlib.pyplot as plt

import message_filters
import yaml


# TODO(@Shashwat): Change these to ROS Params 
# Debugger prints
print_extrinsics_ = False
print_camera_params_ = False
print_timestamp_diff_ = False
print_descriptors_keypoints = False
print_hotspot_location  = True


display_process_img = True
display_contours_ = True
display_epipolar_lines = True
display_keypoints = True
display_hotspot_centres = True

# Param to Perform Hist99 & Image Enhancement
perform_img_enhancement_ = False
# Publisher for enhanced Img(will only work if img enhancement is True)
publish_enhanced_img = False
# Publish Left Thermal Centre and Feature Points
publish_feature_left = False
publish_centre_left = False

# Main Loop Time
loop_timer_ = 0.10


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


      # Load Camera Params & Process Extrinsics
      self.left_path = rospy.get_param('~left_info_path',"../params/thermal_left.yaml")
      self.right_path = rospy.get_param('~right_info_path',"../params/thermal_right.yaml")


      self.left_info  = self.loadCameraParams(self.left_path, "left")
      self.right_info = self.loadCameraParams(self.right_path , "right")
      self.left_info_og = CameraParams("left")
      self.right_info_og = CameraParams("right")

      self.left_info_og.K = self.left_info.K
      self.left_info_og.D = self.left_info.D
      self.left_info_og.R = self.left_info.R
      self.left_info_og.T = self.left_info.T

      self.processExtrinsics()
      self.H_imu_world = np.eye(4)


      # Time Sync Params
      self.flag_got_left = False
      self.flag_got_right = False
      self.flag_got_odom = False

      self.left_stamp = None
      self.right_stamp = None
      self.odom_stamp = None

      # ROS Pubs and Subs
      self.left_sub = rospy.Subscriber("/thermal_left/image", Image, self.left_cb)
      self.right_sub = rospy.Subscriber("/thermal_right/image", Image, self.right_cb)
      self.odom_sub = rospy.Subscriber("/mso_estimator/pose_imu_transformed", PoseStamped, self.odom_cb)
      self.hotspot_pub = rospy.Publisher("/hotspots", Marker, queue_size=10)
      self.hotspot_array_pub = rospy.Publisher("/hotspots/array", PoseArray, queue_size=10)

      self.cv_bridge = CvBridge()
      if publish_enhanced_img is True:
         self.thermal_left_enhanced_pub = rospy.Publisher("thermal_left/enhanced_img", Image, queue_size=10)
         self.thermal_right_enhanced_pub = rospy.Publisher("thermal_right/enhanced_img", Image, queue_size=10)
         

      if publish_centre_left is True:
         self.thermal_left_centre_pub = rospy.Publisher("thermal_left/centre", Image, queue_size=10)
         # self.cv_bridge_left_centre = CvBridge()

      if publish_feature_left is True:
         self.thermal_left_feature_pub = rospy.Publisher("thermal_left/features", Image, queue_size=10)
         # self.cv_bridge_left_features = CvBridge()

      self.main_loop_timer = rospy.Timer(rospy.Duration(0.05), self.main_loop)


   # ROS Subscriber Callbacks
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
   
   # Camera Params Loader
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
         if print_camera_params_ is True:
            print("--------------------")
            print("Camera Name: ", camera_name)
            print("Rotation Matrix: ", cam_info.R)
            print("Translation Vector: ", cam_info.T)
            print("--------------------")
            # print("Rotation Matrix: ", cam_info.R)
            # print("Translation Vector: ", cam_info.T)
         return cam_info
      
   def processImage(self, image):
      # binary thresholding
      image = cv2.inRange(image, 130, 255)

      ## Mask to be used for detecting features only on the hotspot
      # dilate the image
      image = cv2.dilate(image, np.ones((10,10), np.uint8), iterations=2)

      if display_process_img is True:
         cv2.imshow("Processed Image", image)
         cv2.waitKey(1)

      countours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      centers = []

      if display_contours_ is True: 
         if countours is not None:
            cnt = countours
            cv2.drawContours(image, cnt , -1, (0,255,0), 3)
            cv2.imshow("Contours", image)
            cv2.waitKey(1)
            
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
      return sorted_L[:return_len], sorted_R[:return_len]
   

   def getHotspotsfrommatches(self, left_coords, right_coords):
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

      Hfire_cam = np.eye(4)
      Hfire_cam[:3,3] = pos_3d

      if print_hotspot_location is True:
         print("Fire Camera:",Hfire_cam[:3,3])
         print("IMU World:",self.H_imu_world[:3,3])
         # # print("Fire FLU",(Hfire_cam@Hcam_imu@self.H_imu_flu)[:3,3])
         print("Fire World from Matches",(self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3])

      pos_3d = (self.H_imu_world@Hcam_imu@Hfire_cam)[:3,3]
      pos_imu_3d = (Hcam_imu@Hfire_cam)[:3,3]

      return np.array(pos_3d) , np.array(pos_imu_3d)
   
   def display_image(self, image):
      # normalize the image
      cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
      image = np.uint8(image)
      cv2.imshow("Image test", image)
      cv2.waitKey(1)

   
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
         descriptors_hotspots[idx].append(descriptors_total[kp_idx])

      return keypoints_hotspots, descriptors_hotspots
      
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

         if np.linalg.norm(np.dot(right_coord.T, np.dot(F, left_coord))) < 0.3:
            # print("Matched")
            best_left_coords.append(left_coord)
            best_right_coords.append(right_coord)
         else:
            # print("Not Matched")
            pass
         # print("---------")


      # print(len(best_left_coords), "points satisfied epipolar constraint out of ", len(left_coords))

      if len(best_left_coords) < 1:
         best_left_coords = left_coords
      if len(best_right_coords) < 1:
         best_right_coords = right_coords

      return best_left_coords, best_right_coords
   
   def display_centers_on_rectified(self, left_rect, right_rect, left_centers, right_centers):
      ##convert img6 to 3 channel
      img6 = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR)
      for center in right_centers:
         cv2.circle(img6, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

      cv2.imshow("Centres in right", img6)

      ##convert img6 to 3 channel
      img7 = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
      for center in left_centers:
         cv2.circle(img7, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

      cv2.imshow("Centres in left", img7)
      
      cv2.waitKey(1)


   def display_clustered_kp(self, left_rect, right_rect, left_kp, right_kp):
      ##convert left to 3 channel
      img6 = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
      for kp in left_kp:
         # print(kp.pt)
         cv2.circle(img6, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 0, 255), -1)

      ##convert left to 3 channel
      img7 = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR)
      for kp in right_kp:
         # print(kp.pt)
         cv2.circle(img7, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 255, 0), -1)

      cv2.imshow("Keypoints in Left", img6)
      cv2.imshow("Keypoints in Right", img7)
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


   # Perform Histogram Normalization and Image Enhancement
   def perform_image_enhancement(self, image_):
      # Histogram Normalization and then Image enhancement
      return self.enhance_image(self.hist_99(image_))

   def skew(self, x):
      return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

   def main_loop(self, event):
      if not(self.flag_got_left and self.flag_got_right and self.flag_got_odom):
         print("[ERROR] Subscriber Flags are not set, Returning")
         # print(self.flag_got_left, self.flag_got_right, self.flag_got_odom)
         return

      self.flag_got_left = False
      self.flag_got_right = False
      self.flag_got_odom = False
      
      if print_timestamp_diff_ == True:
         print("Left-Right Time Delta: ", self.left_stamp.to_sec() - self.right_stamp.to_sec())
         print("Left-Odom Time Delta: ", self.left_stamp.to_sec() - self.odom_stamp.to_sec())
      
      # If Difference is higher than set limit, return 
      if self.left_stamp.to_sec() - self.right_stamp.to_sec() > 0.1 or self.left_stamp.to_sec() - self.odom_stamp.to_sec() > 0.1:
         # print("[ERROR] Subscriber Timestamps Different, Returning")
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

      # Calculate Temp Mask on Rectified Images
      left_temp = 5.0 * (0.0114 * left_rect - 79 - 32) / (9.0)
      right_temp = 5.0 * (0.0114 * right_rect - 79 - 32) / (9.0)
      left_mask, left_centers = self.processImage(left_temp)
      right_mask, right_centers = self.processImage(right_temp)


      # If Image Enhacement not provided then Normalize
      if perform_img_enhancement_ is True: 
         # Perform Histogram Normalization and Image Enhancement
         left_rect = self.perform_image_enhancement(left_rect)
         right_rect = self.perform_image_enhancement(right_rect)

         left_msg_ = self.cv_bridge.cv2_to_imgmsg(left_rect, "mono8")
         right_msg_ = self.cv_bridge.cv2_to_imgmsg(right_rect, "mono8")
         self.thermal_left_enhanced_pub.publish(left_msg_)
         self.thermal_right_enhanced_pub.publish(right_msg_)
         
      else:
         # make both image intensities similar
         left_rect = cv2.normalize(left_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
         right_rect = cv2.normalize(right_rect, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

      if display_hotspot_centres is True:
         self.display_centers_on_rectified(left_rect, right_rect, left_centers, right_centers)

      if publish_centre_left is True:
         ##convert img6 to 3 channel
         img7 = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
         for center in left_centers:
            cv2.circle(img7, (int(center[0]), int(center[1])), 5, (255, 0, 0), -1)

         left_centre_msg_ = self.cv_bridge.cv2_to_imgmsg(img7, "bgr8")
         self.thermal_left_centre_pub.publish(left_centre_msg_)
         

      left_centers, right_centers = self.get_corresspondences(left_centers, right_centers)

      ## Use the mask on rectified images
      left_rect_masked = cv2.bitwise_and(left_rect, left_mask)
      right_rect_masked = cv2.bitwise_and(right_rect, right_mask)


      # # Step 1: Detect features use ORB
      orb = cv2.ORB_create()
      
      keypoints_left, descriptors_left = orb.detectAndCompute(left_rect_masked, None)
      keypoints_right, descriptors_right = orb.detectAndCompute(right_rect_masked, None)
      ## if no descriptors found return None (if no hotspot visible)
      if descriptors_left is None or descriptors_right is None:
         return None
      
      left_keypoints_hotspots, left_descriptors_hotspots = self.get_keypoints_for_hotspots(keypoints_left, descriptors_left, left_centers)
      right_keypoints_hotspots, right_descriptros_hotspots = self.get_keypoints_for_hotspots(keypoints_right, descriptors_right, right_centers)

      # print("Got", len(left_keypoints_hotspots), "hotspots in left image")
      # print("Got", len(right_keypoints_hotspots), "hotspots in right image")

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

      for i in range(mismatched_length):

         left_kp = left_keypoints_hotspots[i]
         right_kp = right_keypoints_hotspots[i]
         descriptors_left = left_descriptors_hotspots[i]
         descriptors_right = right_descriptros_hotspots[i]
         
         descriptors_left = np.array(descriptors_left)
         descriptors_right = np.array(descriptors_right)
         
         if print_descriptors_keypoints is True:
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

         if display_keypoints is True:
            self.display_clustered_kp(left_rect, right_rect, left_kp, right_kp)


         if publish_feature_left is True:
            ##convert left to 3 channel
            img_ft = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
            for kp in left_kp:
               # print(kp.pt)
               cv2.circle(img_ft, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 0, 255), -1)

            left_feature_msg_ = self.cv_bridge.cv2_to_imgmsg(img_ft, "bgr8")
            self.thermal_left_feature_pub.publish(left_feature_msg_)

         min_len = min(len(descriptors_left), len(descriptors_right))
         descriptors_left = descriptors_left[:min_len]
         descriptors_right = descriptors_right[:min_len]

         bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
         matches = bf.match(descriptors_left, descriptors_right)
         matches = sorted(matches, key = lambda x:x.distance)

         if display_epipolar_lines is True:
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

         #get best matches if the point on right image is close to the epipolar line of point on left image
         best_left_coords, best_right_coords = self.get_good_matched_coords(left_coords, right_coords)


         ## get the 3d pose of the hotspot
         pose_3d, pose_imu_3d = self.getHotspotsfrommatches(best_left_coords, best_right_coords)

         # print('One of the hotspot is at', pose_3d)
         if np.linalg.norm(pose_imu_3d) < 5:
            hotspots_global_frame.append(pose_3d)

      # print("Hotspots in Global Frame: ", hotspots_global_frame)

      if hotspots_global_frame is not None:
         self.displayHotspots(hotspots_global_frame)
         self.publishHotspots(hotspots_global_frame)


   def publishHotspots(self, hotspots):
      hostpots_msg_ = PoseArray()
      hostpots_msg_.header.stamp = rospy.Time.now()
      hostpots_msg_.header.frame_id = 'odom'
      
      # Publish 1st array point as drone location
      drone_location = Pose()
      
      drone_location.position.x = self.H_imu_world[0, 3]
      drone_location.position.y = self.H_imu_world[1, 3]
      drone_location.position.z = self.H_imu_world[2, 3]
      hostpots_msg_.poses.append(drone_location)

      for hotspot in hotspots:
         pose_ = Pose()
         pose_.position.x = hotspot[0]
         pose_.position.y = hotspot[1]
         pose_.position.z = hotspot[2]
         hostpots_msg_.poses.append(pose_)
      
      self.hotspot_array_pub.publish(hostpots_msg_)

      
   # Extrincs Processor
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
      self.right_info.T[0] = 0.244
      self.right_info.T[1] = 0.0
      self.right_info.T[2] = 0.0

      # Set the left camera as the origin of the new coordinate system
      self.left_info.R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      self.left_info.T = np.array([0, 0, 0])

      if print_extrinsics_ is True:
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