#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "stereo_msgs/DisparityImage.h"
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <vector>

#include "mapping/kmeans.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Included files for Time Synchronizatoin
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <vector>


/*
Node that subscribes to both the thermal cameras feeds, maps it to absolute
temperature, and segments a fire map, and publishes

Subscribers:
fire_map_left/image
depth_left/image
pose_left

Publishers:
Lorem Ipsum

TODOs:
- @Jaskaran: Lorem Ipsum
*/

class FireMapping
{
private:
    ros::NodeHandle n;

    ros::Subscriber fireMapLeft_sub;
    ros::Subscriber depthLeft_sub;
    ros::Subscriber odomLeft_sub;

    ros::Publisher hotspots_pub;

    cv::Mat fireMapLeft;
    cv::Mat depthMapLeft;
    // std::vector<double> depthMapLeft;
    cv::Mat color_image;

    float fx = 406.33233091474426;
    float fy = 406.9536696029995;
    float cx = 311.51174613074784;
    float cy = 241.75862889759748;

    float tx, ty, tz, qx, qy, qz, qw;
    Eigen::Matrix3f  K;

    // Fetching points from file
    int pointId = 1;
    vector<Point> all_points;
    string line;

    std::vector<std::vector<float>> centers;

    //Variables for message filters 
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> vio_sub;
    message_filters::Subscriber<stereo_msgs::DisparityImage> depth_sub;
    
    // Approx Time policy
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped, stereo_msgs::DisparityImage> MySyncPolicy;
    double sync_buffer_epsilon = 5.0;
    
    message_filters::Synchronizer<MySyncPolicy> sync;

    
public:
    // Constructor of Fire Mapping class - subscribes to left fire map, left depth map, and left odometry
    FireMapping(ros::NodeHandle n_): n(n_),
                                    image_sub(n, "/fire_map_left/image", 1),
                                    vio_sub(n, "/mso_estimator/pose_transformed", 1),
                                    depth_sub(n, "/thermal_depth/stamped_array", 1),
                                    sync(MySyncPolicy(sync_buffer_epsilon), image_sub, vio_sub, depth_sub)
    {
        // n = n_;
        n.getParam("/fire_mapping/fx", fx);
        n.getParam("/fire_mapping/fy", fy);
        n.getParam("/fire_mapping/cx", cx);
        n.getParam("/fire_mapping/cy", cy);

        K <<fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0;

        fireMapLeft_sub = n.subscribe("fire_map_left/image", 10, &FireMapping::fireMapLeftCallback, this);
        depthLeft_sub = n.subscribe("/thermal/disparity", 10, &FireMapping::depthMapLeftCallback, this);
        odomLeft_sub = n.subscribe("/mso_estimator/pose_transformed", 10, &FireMapping::poseLeftCallback, this);

        hotspots_pub = n.advertise<geometry_msgs::PoseArray>("/hotspots", 1);

        sync.registerCallback(boost::bind(&FireMapping::timeSyncCallback, this, _1, _2, _3));
    }
    

    void timeSyncCallback(const sensor_msgs::Image::ConstPtr& image_msg,
                                   const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                                   const stereo_msgs::DisparityImage::ConstPtr& disparity_msg)
    {
        ROS_INFO("[DEBUG] Timesync(): Synced data received");
        std::cout<<"SegIm Timestamp :"<<image_msg->header.stamp<<std::endl;
        std::cout<<"MSO   Timestamp :"<<pose_msg->header.stamp<<std::endl;
        std::cout<<"Depth Timestamp :"<<disparity_msg->header.stamp<<std::endl;
    }



    void fireMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

        fireMapLeft = cv_image.clone();
        // std::cout << " firemapleft dims : " << fireMapLeft.rows << " " << fireMapLeft.cols << std::endl;
        std::cout << "[callback] fireMapLeft : " << msg->header.stamp << std::endl;
        
        // Initialise all_points with all indices
        // this is done only once
        if(all_points.size() == 0)
        {
            for(int i = 0; i < fireMapLeft.rows; i++)
            {
                for(int j = 0; j < fireMapLeft.cols; j++)
                {
                    if(fireMapLeft.at<uchar>(i,j) > 200)
                    {
                        line = std::to_string(i) + " " + std::to_string(j);
                        Point point(pointId, line);
                        all_points.push_back(point);
                        pointId++;
                    }
                }
            }
        }

    }
    
    void depthMapLeftCallback(const stereo_msgs::DisparityImage::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->image.data[0]);
        cv::Mat cv_image(msg->image.height, msg->image.width, CV_8UC1, const_cast<uint8_t*>(img_data));
        
        // Note: Currently in msg, position arg contains the depth data. In future, use custom ros msg. also, data is timestamped
        depthMapLeft = cv_image;
        // std::cout << "depthmapleft dims : " << depthMapLeft.rows << " " << depthMapLeft.cols << std::endl;
        std::cout << "[callback] depthMapLeft : " << msg->header.stamp << std::endl;
        
        // std::cout << "msg.data : " << msg->position << std::endl;
        // depthMapLeft = cv_image.clone();
    }
    
    void poseLeftCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        tx = msg->pose.position.x;
        ty = msg->pose.position.y;
        tz = msg->pose.position.z;

        qx = msg->pose.orientation.x;
        qy = msg->pose.orientation.y;
        qz = msg->pose.orientation.z;
        qw = msg->pose.orientation.w;
        // std::cout << "MSO pose : " << tx << " " << ty << " " << tz << std::endl;
        std::cout << "[callback] poseLeft : " << msg->header.stamp << std::endl;
    }
    
    // only run when all_points.size() > K
    std::vector<std::vector<float>> cluster_hotspots(int K = 5, int iters = 100)
    {
        std::vector<std::vector<float>> centers;
        if(all_points.size() > 0)
            centers = get_centers(all_points, K, iters);
        return centers;
    }

    std::vector<std::vector<float>> get_projections(std::vector<std::vector<float>> centers)
    {
        Eigen::Vector3f projection;
        std::vector<std::vector<float>> projections;
        float pixel_depth = 0.0;
        int u, v;

        for(int i = 0; i < centers.size(); i++)
        {
            u = std::floor(centers[i][0]);
            v = std::floor(centers[i][1]);
            std::cout << "u,v : " << u << " " << v << std::endl;
            // std::cout << "Depth map : " << depthMapLeft.rows << " " << depthMapLeft.cols << std::endl;

            pixel_depth = depthMapLeft.at<uchar>(u, v);
            // pixel_depth = depthMapLeft[u * fireMapLeft.cols + v];
            std::cout << "Pixel depth : " << pixel_depth << std::endl;
            Eigen::Vector3f point(v, u, 1.0);
            std::cout << "Intrinsic : " << K.inverse() << std::endl;
            projection = K.inverse() * point;
            projection(0) /= projection(2);
            projection(1) /= projection(2);
            projection(2) /= projection(2);
            // std::cout << "Projection : " << projection << std::endl;
            projection *= pixel_depth;

            std::vector<float> p = {projection(0), projection(1), projection(2)};
            projections.push_back(p);
        }
        return projections;
    }

    Eigen::Matrix4f get_drone_H()
    {
        Eigen::Matrix4f H;
        H.setIdentity();

        Eigen::Matrix3f R;
        Eigen::Vector3f T;
        T << tx, ty, tz;

        Eigen::Quaternionf q;
        q.x() = qx;
        q.y() = qy;
        q.z() = qz;
        q.w() = qw;

        R = q.normalized().toRotationMatrix();
        
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = T;

        return H;
    }

    Eigen::Matrix4f get_point_H(std::vector<std::vector<float>> projections, int i)
    {
        Eigen::Matrix4f H_drone_point;
        H_drone_point.setIdentity();

        Eigen::Vector3f T(projections[i][0], projections[i][1], projections[i][2]);
        H_drone_point.block<3,1>(0,3) = T;

        return H_drone_point;
    }

    void run(int num_hotspots = 2, int iter_converge = 50)
    {
        // get hotspot pixel locations
        std::vector<std::vector<float>> centers = cluster_hotspots(num_hotspots, iter_converge);

        // get hotspot location in camera frame
        std::vector<std::vector<float>> projections = get_projections(centers);

        // get world-to-drone transform
        Eigen::Matrix4f H_world_drone = get_drone_H();

        // for each projected point, get drone to hotspot transform
        geometry_msgs::PoseArray pose_array;
        for(int i = 0; i < projections.size(); i++)
        {
            // get transform of hotspot wrt drone
            Eigen::Matrix4f H_drone_point = get_point_H(projections, i);

            Eigen::Matrix4f H_world_point;
            H_world_point = H_world_drone * H_drone_point;

            // add pose to pose_array
            pose_array.header.stamp = ros::Time::now();
            pose_array.header.frame_id = "odom";

            geometry_msgs::Pose pose;
            pose.position.x = H_world_point(0,3);
            pose.position.y = H_world_point(1,3);
            pose.position.z = H_world_point(2,3);
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            pose_array.poses.push_back(pose);
        }
        hotspots_pub.publish(pose_array);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fire_mapping");
    ros::NodeHandle n;

    FireMapping fm(n);

    while (ros::ok())
    {
        // fm.run();
        ros::spinOnce();
    }


    cv::destroyAllWindows();

  return 0;
}