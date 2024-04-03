#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include<vector>

#include "mapping/kmeans.h"

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

    cv::Mat fireMapLeft;
    cv::Mat depthMapLeft;
    cv::Mat color_image;

    float tx, ty, tz, qx, qy, qz, qw;

    // Fetching points from file
    int pointId = 1;
    vector<Point> all_points;
    string line;

    std::vector<std::vector<float>> centers;

public:
    // Constructor of Fire Mapping class - subscribes to left fire map, left depth map, and left odometry
    FireMapping()
    {
        // fireMapLeft_sub = n.subscribe("fire_map_left/image", 10, &FireMapping::fireMapLeftCallback, this);
        // depthLeft_sub = n.subscribe("depth_left/image", 10, &FireMapping::depthMapLeftCallback, this);
        // odomLeft_sub = n.subscribe("pose_left", 10, &FireMapping::poseLeftCallback, this);

        // leftTemperature_pub = n.advertise<sensor_msgs::Image>("temperature_left/image", 1);
        // rightTemperature_pub = n.advertise<sensor_msgs::Image>("temperature_right/image", 1);
        fireMapLeft = cv::imread("/home/development/embereye_ws/src/mapping/src/mask.png", cv::IMREAD_GRAYSCALE);
        depthMapLeft = cv::imread("/home/development/embereye_ws/src/mapping/src/mask.png", cv::IMREAD_GRAYSCALE);
        color_image = cv::imread("/home/development/embereye_ws/src/mapping/src/mask.png", cv::IMREAD_COLOR);

        if(all_points.size() == 0)
        {
            for(int i = 0; i < fireMapLeft.rows; i++)
            {
                for(int j = 0; j < fireMapLeft.cols; j++)
                {
                    line = std::to_string(i) + " " + std::to_string(j);
                    Point point(pointId, line);
                    all_points.push_back(point);
                    pointId++;
                }
            }
        }

        while(true && ros::ok())
        {
            int K = 2;
            int iters = 50;
            centers = cluster_hotspots(K, iters);
            for(int i = 0; i < centers.size(); i++)
            {
                int x = centers[i][0];
                int y = centers[i][1];
                std::cout << ros::Time::now() << " : (" << x  << "," << y << ")" << std::endl;
                cv::circle(color_image, cv::Point(x, y), 20, cv::Scalar(0, 0, 255), -1);
                cv::imwrite("/home/development/embereye_ws/src/mapping/src/mask_read.png", color_image);
            }
        }
    }
    
    void fireMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

        fireMapLeft = cv_image.clone();
        
        // Initialise all_points with all indices
        // this is done only once
        if(all_points.size() == 0)
        {
            for(int i = 0; i < fireMapLeft.rows; i++)
            {
                for(int j = 0; j < fireMapLeft.cols; j++)
                {
                    line = std::to_string(i) + " " + std::to_string(j);
                    Point point(pointId, line);
                    all_points.push_back(point);
                    pointId++;
                }
            }
        }
    }
    void depthMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

        depthMapLeft = cv_image.clone();
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
    }
    // only run when all_points.size() > K
    std::vector<std::vector<float>> cluster_hotspots(int K = 5, int iters = 100)
    {
        std::vector<std::vector<float>> centers = get_centers(all_points, K=K, iters=iters);
        return centers;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fire_mapping");

    FireMapping fm;

    while (ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();

  return 0;
}