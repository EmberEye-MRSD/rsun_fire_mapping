#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
import yaml

class HotspotGTVisualizer:
    FEET2METERS = 0.3048

    def __init__(self):
      rospy.loginfo("[HotspotGTPublisher] Ground truth marker publisher node started")
      self.pub = rospy.Publisher('hotspots/ground_truth_markers', MarkerArray, queue_size=10)
      self.rate = rospy.Rate(10)

      self.marker_size = rospy.get_param('~marker_size', 0.3)
      self.gt_path = rospy.get_param('~gt_path', '../params/hotspot_ground_truth2.yaml')

      self.drone_offset = -0.52


    def publish_marker_poses(self):
        # Load YAML file
        with open(self.gt_path, 'r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)

        hotspots = config['hotspots']

        marker_array = MarkerArray()
        marker_id = 0
        for obj_name, obj_data in hotspots.items():
            marker = self.create_marker(obj_name, obj_data, marker_id, Marker.SPHERE)
            marker_array.markers.append(marker)
            marker_id += 1

            marker = self.create_marker(obj_name, obj_data, marker_id, Marker.CYLINDER)
            marker_array.markers.append(marker)
            marker_id += 1

        while not rospy.is_shutdown():
            self.pub.publish(marker_array)
            # rospy.loginfo("Published marker array poses")
            self.rate.sleep()

    def create_marker(self, obj_name, obj_data, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ground_truth_objects"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = obj_data['position'][0] * self.FEET2METERS
        marker.pose.position.y = obj_data['position'][1] * self.FEET2METERS
        marker.pose.position.z = self.drone_offset
        marker.pose.orientation.x = obj_data['orientation'][0]
        marker.pose.orientation.y = obj_data['orientation'][1]
        marker.pose.orientation.z = obj_data['orientation'][2]
        marker.pose.orientation.w = obj_data['orientation'][3]
        if marker_type == Marker.SPHERE:
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0  # Alpha
            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0  # Blue
        elif marker_type == Marker.CYLINDER:
            marker.scale.x = 4.0
            marker.scale.y = 4.0
            marker.scale.z = 2.0
            marker.color.a = 0.3 # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 1.0  # Blue
        return marker


if __name__ == '__main__':
    try:
        rospy.init_node('ground_truth_marker_publisher', anonymous=True)

        gt_visualizer = HotspotGTVisualizer()
        gt_visualizer.publish_marker_poses()
        
    except rospy.ROSInterruptException:
        pass