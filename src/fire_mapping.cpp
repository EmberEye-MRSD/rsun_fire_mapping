#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <vector>

#include "mapping/kmeans.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
    // cv::Mat depthMapLeft;
    std::vector<float> depthMapLeft;
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

public:
    // Constructor of Fire Mapping class - subscribes to left fire map, left depth map, and left odometry
    FireMapping(ros::NodeHandle n_)
    {
        n = n_;
        n.getParam("/fire_mapping/fx", fx);
        n.getParam("/fire_mapping/fy", fy);
        n.getParam("/fire_mapping/cx", cx);
        n.getParam("/fire_mapping/cy", cy);

        K <<fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0;

        fireMapLeft_sub = n.subscribe("fire_map_left/image", 10, &FireMapping::fireMapLeftCallback, this);
        depthLeft_sub = n.subscribe("/thermal_depth/image_raw", 10, &FireMapping::depthMapLeftCallback, this);
        odomLeft_sub = n.subscribe("/mso_estimator/pose_transformed", 10, &FireMapping::poseLeftCallback, this);

        hotspots_pub = n.advertise<geometry_msgs::PoseArray>("/hotspots", 1);
    }
    
    void fireMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

        fireMapLeft = cv_image.clone();
        std::cout << "size : " << fireMapLeft.rows << " " << fireMapLeft.cols << std::endl;
        
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
    void depthMapLeftCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        // const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        // cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));
        depthMapLeft = msg->data;
        // std::cout << "msg.data : " << msg.data << std::endl;


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
        std::cout << "MSO pose : " << tx << " " << ty << " " << tz << std::endl;
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

            // pixel_depth = depthMapLeft.at<uchar>(u, v);
            pixel_depth = depthMapLeft[u * fireMapLeft.cols + v];
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
        if(centers.size() > 0)
        {
            std::cout << "\n\n\n";
            std::cout << "--- Hotspot Pixel Locations ---" << std::endl;
            std::cout << "# clusters : " << centers.size() << std::endl;
            std::cout << "Center 1 : " << centers[0][0] << " " << centers[0][1] << std::endl;
            std::cout << "Center 2 : " << centers[1][0] << " " << centers[1][1] << std::endl;
        }

        // get hotspot location in camera frame
        std::vector<std::vector<float>> projections = get_projections(centers);
        if(projections.size() > 0)
        {
            std::cout << "--- Hotspot Projections ---" << std::endl;
            std::cout << "# clusters : " << projections.size() << std::endl;
            std::cout << "Hotspot 1 : " << projections[0][0] << " " << projections[0][1] << projections[0][2] << std::endl;
            std::cout << "Hotspot 2 : " << projections[1][0] << " " << projections[1][1] << projections[1][2] << std::endl;
        }

        // get world-to-drone transform
        Eigen::Matrix4f H_world_drone = get_drone_H();
        std::cout << "--- Drone H wrt World ---" << std::endl;
        std::cout << H_world_drone << std::endl;

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

            std::cout << "idx : " << i << " | pose : " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;

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
        fm.run();
        ros::spinOnce();
    }

    cv::destroyAllWindows();

  return 0;
}