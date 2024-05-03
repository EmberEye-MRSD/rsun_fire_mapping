// #include "ros/ros.h"
// #include "sensor_msgs/Image.h"
// #include <sensor_msgs/image_encodings.h>
// #include "sensor_msgs/Range.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseArray.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "stereo_msgs/DisparityImage.h"
// #include <iostream>

// #include "cv_bridge/cv_bridge.h"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc_c.h"

// #include <vector>

// #include "mapping/kmeans.h"
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// // Included files for Time Synchronizatoin
// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <vector>


// /*
// Node that subscribes to both the thermal cameras feeds, maps it to absolute
// temperature, and segments a fire map, and publishes

// Subscribers:
// fire_map_left/image
// depth_left/image
// pose_left

// Publishers:
// Lorem Ipsum

// TODOs:
// - @Jaskaran: Lorem Ipsum
// */

// class FireMapping
// {
// private:
//     ros::NodeHandle n;

//     ros::Subscriber fireMapLeft_sub; // (512x640)
//     ros::Subscriber fireMapRight_sub; // (512x640)
//     ros::Subscriber odomLeft_sub;
    

//     ros::Publisher centerL_pub;
//     ros::Publisher centerR_pub;
//     ros::Publisher hotspots_pub;

//     cv::Mat fireMapLeft;
//     cv::Mat fireMapRight;

//     float fx = 407.64366374683186;
//     float fy = 406.69498575397546;
//     float cx = 313.1370938061567;
//     float cy = 239.03696784390246;

//     float tx, ty, tz, qx, qy, qz, qw; // from VIO
//     Eigen::Matrix3f  K; // TODO: load params from yaml

//     // Fetching points from file for thermal camera L
//     int pointIdL = 1;
//     vector<Point> all_points_L;
//     string lineL;

//     // Fetching points from file for thermal camera R
//     int pointIdR = 1;
//     vector<Point> all_points_R;
//     string lineR;

//     std::vector<std::vector<float>> centers;

//     //Variables for message filters 
//     message_filters::Subscriber<sensor_msgs::Image> imageL_sub;
//     message_filters::Subscriber<sensor_msgs::Image> imageR_sub;
//     message_filters::Subscriber<geometry_msgs::PoseStamped> vio_sub;
    
//     // Approx Time policy
//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> MySyncPolicy;
//     double sync_buffer_epsilon = 5.0;
    
//     message_filters::Synchronizer<MySyncPolicy> sync;
    
//     Eigen::Matrix4f IMU_thermal_H;
//     float drone_height = 1.28;

    
// public:
//     // Constructor of Fire Mapping class - subscribes to left fire map, left depth map, and left odometry
//     FireMapping(ros::NodeHandle n_): n(n_),
//                                     imageL_sub(n, "/fire_map_left/image", 1),
//                                     imageR_sub(n, "/fire_map_right/image", 1),
//                                     vio_sub(n, "/mso_estimator/pose_imu_transformed", 1),
//                                     sync(MySyncPolicy(10), imageL_sub, imageR_sub, vio_sub)
//     {
//         // n = n_;
//         n.getParam("/fire_mapping/fx", fx);
//         n.getParam("/fire_mapping/fy", fy);
//         n.getParam("/fire_mapping/cx", cx);
//         n.getParam("/fire_mapping/cy", cy);

//         K <<fx, 0.0, cx,
//             0.0, fy, cy,
//             0.0, 0.0, 1.0;

//         IMU_thermal_H = get_IMU_to_thermalL_H();

//         fireMapLeft_sub = n.subscribe("fire_map_left/image", 10, &FireMapping::fireMapLeftCallback, this);
//         fireMapRight_sub = n.subscribe("fire_map_right/image", 10, &FireMapping::fireMapRightCallback, this);
//         odomLeft_sub = n.subscribe("/mso_estimator/pose_imu_transformed", 10, &FireMapping::poseCallback, this);

//         centerL_pub = n.advertise<sensor_msgs::Image>("centerL/image", 1);
//         centerR_pub = n.advertise<sensor_msgs::Image>("centerR/image", 1);
//         hotspots_pub = n.advertise<geometry_msgs::PoseArray>("/hotspots", 1);

//         sync.registerCallback(boost::bind(&FireMapping::timeSyncCallback, this, _1, _2, _3));
//     }
    

//     void timeSyncCallback(  const sensor_msgs::Image::ConstPtr& image_msg_L,
//                             const sensor_msgs::Image::ConstPtr& image_msg_R,
//                             const geometry_msgs::PoseStamped::ConstPtr& pose_msg
//                             )
//     {
//         ROS_INFO("[DEBUG] Timesync(): Synced data received");
//         std::cout<<"SegImL Timestamp :"<<image_msg_L->header.stamp<<std::endl;
//         std::cout<<"SegImR Timestamp :"<<image_msg_R->header.stamp<<std::endl;
//         std::cout<<"MSO   Timestamp :"<<pose_msg->header.stamp<<std::endl;
//     }



//     void fireMapLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
//     {
//         const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
//         cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

//         fireMapLeft = cv_image.clone();

//         all_points_L.clear();
//         for(int i = 0; i < fireMapLeft.rows; i++)
//         {
//             for(int j = 0; j < fireMapLeft.cols; j++)
//             {
//                 if(fireMapLeft.at<uchar>(i,j) > 20) // should be > 200
//                 {
//                     lineL = std::to_string(i) + " " + std::to_string(j);
//                     Point point(pointIdL, lineL);
//                     all_points_L.push_back(point);
//                     pointIdL++;
//                 }
//             }
//         }
//     }

//     void fireMapRightCallback(const sensor_msgs::Image::ConstPtr& msg)
//     {
//         const uint8_t* img_data = reinterpret_cast<const uint8_t*>(&msg->data[0]);
//         cv::Mat cv_image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(img_data));

//         fireMapRight = cv_image.clone();

//         all_points_R.clear();
//         for(int i = 0; i < fireMapRight.rows; i++)
//         {
//             for(int j = 0; j < fireMapRight.cols; j++)
//             {
//                 if(fireMapRight.at<uchar>(i,j) > 20) // should be > 200
//                 {
//                     lineR = std::to_string(i) + " " + std::to_string(j);
//                     Point point(pointIdR, lineR);
//                     all_points_R.push_back(point);
//                     pointIdR++;
//                 }
//             }
//         }
//     }
    
//     void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//     {
//         tx = msg->pose.position.x;
//         ty = msg->pose.position.y;
//         tz = msg->pose.position.z;

//         qx = msg->pose.orientation.x;
//         qy = msg->pose.orientation.y;
//         qz = msg->pose.orientation.z;
//         qw = msg->pose.orientation.w;
//         // std::cout << "MSO pose : " << tx << " " << ty << " " << tz << std::endl;
//     }
    
//     // only run when all_points_L.size() > K
//     std::vector<std::vector<float>> cluster_hotspots_L(int K = 1, int iters = 100)
//     {
//         std::vector<std::vector<float>> centers;
//         if(all_points_L.size() > 0)
//             centers = get_centers(all_points_L, K, iters);
//         return centers;
//     }

//     // only run when all_points_L.size() > K
//     std::vector<std::vector<float>> cluster_hotspots_R(int K = 1, int iters = 100)
//     {
//         std::vector<std::vector<float>> centers;
//         if(all_points_R.size() > 0)
//             centers = get_centers(all_points_R, K, iters);
//         return centers;
//     }

    

//     std::vector<std::vector<int>> get_corresspondences(std::vector<std::vector<float>> centers_L,
//                                                         std::vector<std::vector<float>> centers_R)
//     {
//         for(int i = 0; i < centers_L.size(); i++)
//         {

//         }
//     }

//     Eigen::Matrix4f get_IMU_to_thermalL_H()
//     {
//         Eigen::Matrix4f H;
//         H << 0.01723243, 0.99914308, 0.03763171, 0.13482769,
//             -0.00649153, 0.03774831, -0.99926619, 0.03385016,
//             -0.99983044, 0.0169755,  0.00713646, -0.34002503,
//             0.0,         0.0,        0.0,        1.0;

//         return H;
//     }

//     // TODO
//     Eigen::Matrix4f get_IMU_to_thermalR_H()
//     {
//         Eigen::Matrix4f H;
//         H << 0.01723243, 0.99914308, 0.03763171, 0.13482769,
//             -0.00649153, 0.03774831, -0.99926619, 0.03385016,
//             -0.99983044, 0.0169755,  0.00713646, -0.34002503,
//             0.0,         0.0,        0.0,        1.0;

//         return H;
//     }

//     std::vector<std::vector<float>> get_projections(std::vector<std::vector<float>> centers)
//     {
//         Eigen::Vector3f projection;
//         std::vector<std::vector<float>> projections;
//         int u, v;

//         for(int i = 0; i < centers.size(); i++)
//         {
//             u = std::floor(centers[i][0]);
//             v = std::floor(centers[i][1]);
//             // std::cout << "u,v : " << u << " " << v << std::endl;

//             Eigen::Vector3f point(v, u, 1.0);
//             projection = K.inverse() * point;
//             // std::cout << "Ray : " << projection << std::endl;
//             // std::cout << "K inv : " << K.inverse() << std::endl;
//             projection(0) /= projection(1);
//             projection(2) /= projection(1);
//             projection(1) /= projection(1);
//             // projection /= projection.norm();
//             projection *= (tz + 0.52 + 0.033 - 1.25);
//             std::cout << ">>> Projection : " << projection << std::endl;

//             std::vector<float> p = {projection(0), projection(1), projection(2)};
//             projections.push_back(p);
//         }
//         return projections;
//     }

//     Eigen::Matrix4f get_drone_H()
//     {
//         Eigen::Matrix4f H;
//         H.setIdentity();

//         Eigen::Matrix3f R;
//         Eigen::Vector3f T;
//         T << tx, ty, tz;

//         Eigen::Quaternionf q;
//         q.x() = qx;
//         q.y() = qy;
//         q.z() = qz;
//         q.w() = qw;

//         R = q.normalized().toRotationMatrix();
        
//         H.block<3,3>(0,0) = R;
//         H.block<3,1>(0,3) = T;

//         return H;
//     }

//     Eigen::Matrix4f get_point_H(std::vector<float> projection)
//     {
//         Eigen::Matrix4f H_drone_point;
//         H_drone_point.setIdentity();

//         Eigen::Vector3f T(projection[0], projection[1], projection[2]);
//         H_drone_point.block<3,1>(0,3) = T;

//         return H_drone_point;
//     }

//     void run(int num_hotspots = 2, int iter_converge = 50)
//     {
//         // -------------------------- Receive All Inputs (Done) ----------------------------------------

//         std::cout << ">>>>>>>> RUN ITERATION <<<<<<<<<<<<<" << std::endl;

//         // get hotspot pixel locations
//         std::vector<std::vector<float>> centers_L, centers_R;

//         int curr_hotspots = num_hotspots;
//         while(curr_hotspots > 0)
//         {
//             try
//             {
//                 std::cout << "Trying H = " << curr_hotspots << std::endl;
//                 centers_L = cluster_hotspots_L(num_hotspots, iter_converge);
//                 centers_R = cluster_hotspots_R(num_hotspots, iter_converge);
//             }
//             catch(const std::exception& e)
//             {
//                 int a = 1; // placeholder
//             }
//             if(centers_L.size() > 0 && centers_R.size() > 0 && centers_L.size() == centers_R.size())
//                 break;
//             curr_hotspots -= 1;
//         }
        

//         std::cout << "Clusters L : " << centers_L.size() << std::endl;
//         std::cout << "Clusters R : " << centers_R.size() << std::endl;
        
//         // plotting code
//         cv::Mat plotCentersLeft, plotCentersRight;
//         for(int i = 0; i < centers_L.size(); i++)
//         {
//             if(i == 0)
//             {
//                 plotCentersLeft = fireMapLeft.clone();
//                 // std::cout << "Plotting centers" << std::endl;
//                 cv::cvtColor(plotCentersLeft, plotCentersLeft, cv::COLOR_GRAY2RGB);
//             }
//             cv::circle(plotCentersLeft, cv::Point(std::floor(centers_L[i][1]), std::floor(centers_L[i][0])), 10, cv::Scalar(0, 0, 255), -1);
//         }
//         for(int i = 0; i < centers_R.size(); i++)
//         {
//             if(i == 0)
//             {
//                 plotCentersRight = fireMapRight.clone();
//                 cv::cvtColor(plotCentersRight, plotCentersRight, cv::COLOR_GRAY2RGB);
//             }
//             cv::circle(plotCentersRight, cv::Point(std::floor(centers_R[i][1]), std::floor(centers_R[i][0])), 10, cv::Scalar(0, 0, 255), -1);
//         }
//         if(fireMapLeft.rows > 0)
//         {
//             // publish Left Fire Map
//             cv_bridge::CvImage leftClusterMap;
//             leftClusterMap.header.frame_id = "thermal_L"; // Same timestamp and tf frame as input image
//             leftClusterMap.header.stamp = ros::Time::now();
//             leftClusterMap.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
//             leftClusterMap.image    = plotCentersLeft; // Your cv::Mat
//             centerL_pub.publish(leftClusterMap.toImageMsg());

//             // cv::imwrite("/home/development/embereye_ws/src/rsun_fire_mapping/src/cluster_viz.png", cv::hconcat(plotCentersLeft, plotCentersRight, plotCentersLeft));
//         }
//         if(fireMapRight.rows > 0)
//         {
//             // publish Right Fire Map
//             cv_bridge::CvImage rightClusterMap;
//             rightClusterMap.header.frame_id = "thermal_R"; // Same timestamp and tf frame as input image
//             rightClusterMap.header.stamp = ros::Time::now();
//             rightClusterMap.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
//             rightClusterMap.image    = plotCentersRight; // Your cv::Mat
//             centerR_pub.publish(rightClusterMap.toImageMsg());
//         }

//         // -------------------------- Find Corresspondences --------------------------------------------
//         std::vector<std::vector<float>> centers = get_correspondences(centers);

//         // -------------------------- Get Hotspots Centroid (Done) ----------------------------------------

//         // get hotspot location in camera frame
//         // std::vector<std::vector<float>> projections = get_projections(centers);

//         // // get world-to-drone transform
//         // Eigen::Matrix4f H_world_IMU = get_drone_H();
//         // Eigen::Matrix4f H_world_point;

//         // // for each projected point, get drone to hotspot transform
//         // geometry_msgs::PoseArray pose_array;
//         // // std::cout << "Number of hotspots found : " << projections.size() << std::endl;
//         // for(int i = 0; i < projections.size(); i++)
//         // {
//         //     std::cout << "\n--------------- Timestep -------------------" << std::endl;

//         //     // get transform of hotspot wrt drone
//         //     Eigen::Matrix4f H_thermal_point = get_point_H(projections[i]);
            
//         //     // get world coordinate of hotspot
//         //     H_world_point = H_world_IMU * IMU_thermal_H * H_thermal_point;

//         //     // add pose to pose_array
//         //     pose_array.header.stamp = ros::Time::now();
//         //     pose_array.header.frame_id = "odom";

//         //     geometry_msgs::Pose pose;
//         //     pose.position.x = H_world_point(0,3);
//         //     pose.position.y = H_world_point(1,3);
//         //     pose.position.z = H_world_point(2,3);
//         //     pose.orientation.x = 0.0;
//         //     pose.orientation.y = 0.0;
//         //     pose.orientation.z = 0.0;
//         //     pose.orientation.w = 1.0;

//         //     pose_array.poses.push_back(pose);
//         // }
//         // hotspots_pub.publish(pose_array);
//     }
// };


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "fire_mapping_stereo");
//     ros::NodeHandle n;

//     FireMapping fm(n);

//     while (ros::ok())
//     {
//         fm.run();
//         ros::spinOnce();
//     }


//     cv::destroyAllWindows();

//   return 0;
// }