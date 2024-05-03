#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Range.h"
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

    ros::Subscriber fireMapLeft_sub; // (512x640)
    ros::Subscriber odomLeft_sub;
    // ros::Subscriber rangeFinder_sub;
    

    ros::Publisher hotspots_pub;

    cv::Mat fireMapLeft;

    float fx = 407.64366374683186;
    float fy = 406.69498575397546;
    float cx = 313.1370938061567;
    float cy = 239.03696784390246;

    float tx, ty, tz, qx, qy, qz, qw; // from VIO
    Eigen::Matrix3f  K; // TODO: load params from yaml

    // Fetching points from file
    int pointId = 1;
    vector<Point> all_points;
    string line;

    std::vector<std::vector<float>> centers;

    //Variables for message filters 
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> vio_sub;
    
    // Approx Time policy
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> MySyncPolicy;
    double sync_buffer_epsilon = 5.0;
    
    message_filters::Synchronizer<MySyncPolicy> sync;
    
    Eigen::Matrix4f IMU_thermal_H;
    float drone_height = 1.28;

    
public:
    // Constructor of Fire Mapping class - subscribes to left fire map, left depth map, and left odometry
    FireMapping(ros::NodeHandle n_): n(n_),
                                    image_sub(n, "/fire_map_left/image", 1),
                                    vio_sub(n, "/mso_estimator/pose_transformed", 1),
                                    sync(MySyncPolicy(10), image_sub, vio_sub)
    {
        // n = n_;
        n.getParam("/fire_mapping/fx", fx);
        n.getParam("/fire_mapping/fy", fy);
        n.getParam("/fire_mapping/cx", cx);
        n.getParam("/fire_mapping/cy", cy);

        K <<fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0;

        IMU_thermal_H = get_IMU_to_thermal_H();

        fireMapLeft_sub = n.subscribe("fire_map_left/image", 10, &FireMapping::fireMapLeftCallback, this);
        odomLeft_sub = n.subscribe("/mso_estimator/pose_imu_transformed", 10, &FireMapping::poseLeftCallback, this);

        hotspots_pub = n.advertise<geometry_msgs::PoseArray>("/hotspots", 1);

        sync.registerCallback(boost::bind(&FireMapping::timeSyncCallback, this, _1, _2));
    }
    

    void timeSyncCallback(  const sensor_msgs::Image::ConstPtr& image_msg,
                            const geometry_msgs::PoseStamped::ConstPtr& pose_msg
                            )
    {
        ROS_INFO("[DEBUG] Timesync(): Synced data received");
        std::cout<<"SegIm Timestamp :"<<image_msg->header.stamp<<std::endl;
        std::cout<<"MSO   Timestamp :"<<pose_msg->header.stamp<<std::endl;
    }



    void fireMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
        cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

        fireMapLeft = cv_image.clone();

        all_points.clear();
        for(int i = 0; i < fireMapLeft.rows; i++)
        {
            for(int j = 0; j < fireMapLeft.cols; j++)
            {
                if(fireMapLeft.at<uchar>(i,j) > 20) // should be > 200
                {
                    line = std::to_string(i) + " " + std::to_string(j);
                    Point point(pointId, line);
                    all_points.push_back(point);
                    pointId++;
                }
            }
        }
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
    std::vector<std::vector<float>> cluster_hotspots(int K = 1, int iters = 100)
    {
        std::vector<std::vector<float>> centers;
        if(all_points.size() > 0)
            centers = get_centers(all_points, K, iters);
        return centers;
    }

    Eigen::Matrix4f get_IMU_to_thermal_H()
    {
        Eigen::Matrix4f H;
        H << 0.01723243, 0.99914308, 0.03763171, 0.13482769,
            -0.00649153, 0.03774831, -0.99926619, 0.03385016,
            -0.99983044, 0.0169755,  0.00713646, -0.34002503,
            0.0,         0.0,        0.0,        1.0;

        return H;
    }

    std::vector<std::vector<float>> get_projections(std::vector<std::vector<float>> centers)
    {
        Eigen::Vector3f projection;
        std::vector<std::vector<float>> projections;
        int u, v;

        for(int i = 0; i < centers.size(); i++)
        {
            u = std::floor(centers[i][0]);
            v = std::floor(centers[i][1]);
            // std::cout << "u,v : " << u << " " << v << std::endl;

            Eigen::Vector3f point(v, u, 1.0);
            projection = K.inverse() * point;
            // std::cout << "Ray : " << projection << std::endl;
            // std::cout << "K inv : " << K.inverse() << std::endl;
            projection(0) /= projection(1);
            projection(2) /= projection(1);
            projection(1) /= projection(1);
            // projection /= projection.norm();
            projection *= (tz + 0.52 + 0.033 - 1.25);
            std::cout << ">>> Projection : " << projection << std::endl;

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

    Eigen::Matrix4f get_point_H(std::vector<float> projection)
    {
        Eigen::Matrix4f H_drone_point;
        H_drone_point.setIdentity();

        Eigen::Vector3f T(projection[0], projection[1], projection[2]);
        H_drone_point.block<3,1>(0,3) = T;

        return H_drone_point;
    }

    void run(int num_hotspots = 1, int iter_converge = 50)
    {
        // -------------------------- Receive All Inputs (Done) ----------------------------------------

        std::cout << ">>>>>>>> RUN ITERATION <<<<<<<<<<<<<" << std::endl;

        // get hotspot pixel locations
        std::vector<std::vector<float>> centers = cluster_hotspots(num_hotspots, iter_converge);
        std::cout << "Number of clusters : " << centers.size() << std::endl;
        if(centers.size() > 0)
            std::cout << "Center : " << centers[0][0] << " " << centers[0][1] << std::endl;
        cv::Mat viz = fireMapLeft.clone();
        for(int i = 0; i < centers.size(); i++)
        {
            cv::cvtColor(viz, viz, cv::COLOR_GRAY2RGB);
            cv::circle(viz, cv::Point(std::floor(centers[i][1]), std::floor(centers[i][0])), 10, cv::Scalar(0, 0, 255), -1);
        }
        if(fireMapLeft.rows > 0)
            cv::imwrite("/home/development/embereye_ws/src/rsun_fire_mapping/src/cluster_viz.png", viz);

        // -------------------------- Get Hotspots Centroid (Done) ----------------------------------------

        // get hotspot location in camera frame
        std::vector<std::vector<float>> projections = get_projections(centers);

        // -------------------------- Get Hotspots Location List in Cam Frame (Done) ----------------------------------------

        // get world-to-drone transform
        Eigen::Matrix4f H_world_IMU = get_drone_H();
        Eigen::Matrix4f H_world_point;
        // std::cout << "Drone H : " << H_world_IMU << std::endl;

        // -------------------------- Get Drone Pose (Done) ----------------------------------------

        // for each projected point, get drone to hotspot transform
        geometry_msgs::PoseArray pose_array;
        // std::cout << "Number of hotspots found : " << projections.size() << std::endl;
        for(int i = 0; i < projections.size(); i++)
        {
            std::cout << "\n--------------- Timestep -------------------" << std::endl;

            // std::cout <<    "Projection 3D point : " <<
            //                 projections[i][0] << " " <<
            //                 projections[i][1] << " " <<
            //                 projections[i][2] << std::endl;
            // std::cout << "Distance : " << sqrt(projections[i][0] * projections[i][0] +
                                                // projections[i][1] * projections[i][1] +
                                                // projections[i][2] * projections[i][2]) << std::endl;


            // get transform of hotspot wrt drone
            Eigen::Matrix4f H_thermal_point = get_point_H(projections[i]);
            
            if(H_thermal_point(1,3) > 0)
            std::cout <<    "Point in camera frame : " <<
                            H_thermal_point(0,3) << " " <<
                            H_thermal_point(1,3) << " " <<
                            H_thermal_point(2,3) << std::endl;

            // std::cout <<    "IMU to camera transform : " << IMU_thermal_H << std::endl;
            // std::cout <<    "World to IMU transform : " << H_world_IMU << std::endl;

            // -------------------------- Get Hotspot H for each Hotspot (Done) ----------------------------------------

            H_world_point = H_world_IMU * IMU_thermal_H * H_thermal_point;

            // std::cout <<    "World to Hotspot transform : " << H_world_point << std::endl;

            // -------------------------- Get Hotspot H in World Frame (TODO: IMU extrinsics) ----------------------------------------

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

            // -------------------------- Subtract drone height in IMU frame (TODO) ----------------------------------------

            // std::cout << "Checkboard position : " <<
            //             pose.position.x << " " <<
            //             pose.position.y << " " <<
            //             pose.position.z << std::endl;

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