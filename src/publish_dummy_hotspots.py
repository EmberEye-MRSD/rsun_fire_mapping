import rospy
import numpy as np

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

hotspot_pub = rospy.Publisher("/hotspots/array", PoseArray, queue_size=10)

def publish_dummy_pose_array(iter):
    pose_array = PoseArray()

    for i in range(0, (iter + 100) // 100):
        pose = Pose()
        
        # get random noise
        mu = i * 10
        sigma = 0.5
        s = np.random.normal(mu, sigma, 3)

        pose.position.x = s[0]
        pose.position.y = s[1]
        pose.position.z = s[2]

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)
    
    hotspot_pub.publish(pose_array)

def main():
    rospy.init_node('dummy_hotspots', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    iter = 0

    while not rospy.is_shutdown():
        iter += 1
        publish_dummy_pose_array(iter)
        rate.sleep()
    
if __name__ == "__main__":
    main()