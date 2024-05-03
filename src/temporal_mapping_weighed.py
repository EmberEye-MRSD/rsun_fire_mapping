#!/usr/bin/env python3

import rospy
import math
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

class Filter:
    def __init__(self):
        self.poses_sub = rospy.Subscriber("/hotspots/array", PoseArray, self.hotspots_cb)
        self.hotspot_pub = rospy.Publisher("/hotspots/weighed_global_map", MarkerArray, queue_size=10)

        self.measurements = [] # list of lists of measurements for each hotspot
        self.global_poses = [] # list of filtered pose for each hotspot
        self.measurement_weights = [] # list of lists confidence weights of each measurement for each hotspot
        self.total_weights = [] # list of total weights for each hotspot

        nn_threshold_param = rospy.get_param('/temporal_mapping/nn_threshold', default=2.5)
        clipping_distance_param = rospy.get_param('/temporal_mapping/clipping_distance', default=6.0)
        # outlier_frame_threshold_param = rospy.get_param('/outlier_frame_threshold', default=5)
        hotspot_inflation_radius_param = rospy.get_param('/hotspot_inflation_radius', default=0.0)

        print("[INFO] nn_threshold :", nn_threshold_param)
        print("[INFO] clipping_distance :", clipping_distance_param)
        # print("[INFO] outlier_frame_threshold :", outlier_frame_threshold_param)
        print("[INFO] hotspot_inflation_radius :", hotspot_inflation_radius_param)

        self.nn_thresh = nn_threshold_param # nn radius in meters
        self.clipping_distance = clipping_distance_param # clipping distance for thermal stereo depth
        # self.outlier_frame_threshold = outlier_frame_threshold_param # number of frames for a hotspot to be considered legit
        self.hotspot_inflation_radius = hotspot_inflation_radius_param # in the worst case, min dist between drone and hotspot
        self.height_threshold = 0.0
    
    def distance(self, p1, p2, z=False):
        if z==True:
            dist = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2
        else:
            dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dist

    def isFar(self, drone_odom, pose):
        if self.distance(drone_odom, pose) > self.clipping_distance:
            return True
        return False

    def find_closest_hotspot(self, point):
        """
        input: query point
        processing: uses global list of hotspot locations
        output: nearest neighbour, idx of nn, distance (default None, None, inf)
        """
        closest_point = None
        min_dist = np.inf
        idx = None

        if len(self.global_poses) == 0:
            return None, None, np.inf

        for i, hotspot_pose in enumerate(self.global_poses):
            dist = self.distance(point, hotspot_pose)
            # print(f"Distance to hotspot", hotspot_pose, )
            if dist < min_dist:
                min_dist = dist
                closest_point = hotspot_pose
                idx = i
        
        return closest_point, idx, min_dist

    def get_weight(self, point, curr_pose):
        weight = 1.0 / (self.distance(point, curr_pose, z=True))
        # weight = 1.0 / (self.distance(point, curr_pose, z=True) * math.cos(curr_pose[3] - math.atan2(point[1] - curr_pose[1], point[0] - curr_pose[0])))
        return weight * weight

    def add_new_hotspot(self, point, curr_pose):
        """
        input: point reading
        """
        self.measurements.append([point])
        self.global_poses.append(point)

        weight = self.get_weight(point, curr_pose)
        self.measurement_weights.append([weight])
        self.total_weights.append(weight)

    def update_nn(self, idx, point, curr_pose):
        """
        input: idx of nn, point reading
        """
        weight = self.get_weight(point, curr_pose)
        self.measurements[idx].append(point)
        self.measurement_weights[idx].append(weight)

        list_readings = np.array(self.measurements[idx])
        xs = list_readings[:, 0]
        ys = list_readings[:, 1]
        zs = list_readings[:, 2]

        self.total_weights[idx] += weight
        
        updated_reading = [np.mean(xs), np.mean(ys), np.mean(zs)]
        print("Vanilla : ", updated_reading)
        print("weights : ", self.measurement_weights[idx])
        updated_reading = [ np.sum(np.multiply(xs, self.measurement_weights[idx])) / self.total_weights[idx],
                            np.sum(np.multiply(ys, self.measurement_weights[idx])) / self.total_weights[idx],
                            np.sum(np.multiply(zs, self.measurement_weights[idx])) / self.total_weights[idx]]
        print("Weighed : ", updated_reading)

        self.global_poses[idx][0] = np.sum(np.multiply(xs, self.measurement_weights[idx])) / self.total_weights[idx]
        self.global_poses[idx][1] = np.sum(np.multiply(ys, self.measurement_weights[idx])) / self.total_weights[idx]
        self.global_poses[idx][2] = np.sum(np.multiply(zs, self.measurement_weights[idx])) / self.total_weights[idx]
    
    def get_marker(self, point, i):
        marker = Marker()
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # stddev = np.std(np.array(self.measurements[i]))
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5

        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "odom"
        marker.ns = "weighed_predicted_hotspots"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        return marker
        
    def publish_updated_hotspots(self):
        marker_array = MarkerArray()
        for i, point in enumerate(self.global_poses):

            # create new marker for given hotspot
            marker = self.get_marker(point, i)

            marker_array.markers.append(marker)
        
        self.hotspot_pub.publish(marker_array)
    
    def outlier_rejection(self):
        for i in range(len(self.measurements)):
            if len(self.measurements[i]) < self.outlier_frame_threshold:
                self.global_poses.pop(i)

    def update_global_poses(self, poses_reading):
        
        theta = euler_from_quaternion([poses_reading[0].orientation.x,
                                       poses_reading[0].orientation.y,
                                       poses_reading[0].orientation.z,
                                       poses_reading[0].orientation.w], axes='rxyz')

        curr_pose = [poses_reading[0].position.x,
                    poses_reading[0].position.y,
                    poses_reading[0].position.z,
                    theta[2]]

        for pose in poses_reading[1:]:
            

            # get each new hotspot reading
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            point = [x, y, z]

            if z > self.height_threshold:
                continue

            if self.isFar(curr_pose, point):
                print("Point too far : ", point, curr_pose, self.distance(point, curr_pose))
                # print("Threshold : ", self.clipping_distance)
                continue

            # check if any existing hotspot exists nearby
            nn, idx, nn_dist = self.find_closest_hotspot(point)

            # use new reading to update existing hotspot, or add new hotspot
            if (nn_dist < self.nn_thresh) and (idx is not None):
                print("Updating hotspot : ", point)
                self.update_nn(idx, point, curr_pose)
            else:
                print("Adding new hotspot : ", point)
                self.add_new_hotspot(point, curr_pose)
            
        # remove outliers (frame jumps)
        # self.outlier_rejection()

        # publish MarkerArray
        self.publish_updated_hotspots()

        # np.save('meas.npy', np.array(self.measurements))
        # np.save('hotspots.npy', np.array(self.global_poses))

    def hotspots_cb(self, msg):
        poses_reading = msg.poses
        self.update_global_poses(poses_reading)

def main():
    rospy.init_node("temporal_mapping_weighed")
    filter = Filter()
    rospy.spin()

if __name__ == "__main__":
    main()