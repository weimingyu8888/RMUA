#ifndef _PATH_SENDER_HPP_
#define _PATH_SENDER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <path_sender/WayPoints.h>

class PathSender
{
private:
    // ================= 固定点（完全保留你的） =================
    cv::Point3f station_[13] = {
        {0.0, 0.0, 0.0},
        {0, 0, 0},
        {143, -286, 8},
        {552, -518, 33},
        {1349, 103, 9.8},
        {1176, 424, 78},
        {783, 675, 5},
        {2, 287, 3},
        {713, 728, 14},
        {1032, 483, -59},
        {1327, -153, 35},
        {676, -539, 48},
        {263, -399, -0.2}
    };

    cv::Point3f Transit_hub_[13] = {
        {0, 0, 0},
        {650.39, 75.15, 136.33},
        {665.5, 44.8, 136.33},
        {706.9, 47, 136.33},
        {726, 88, 136.33},
        {703, 122, 136.33},
        {657, 126, 136.33},
        {650.39, 75.15, 163.9},
        {657, 126, 163.9},
        {703, 122, 163.9},
        {726, 88, 163.9},
        {706.9, 47, 163.9},
        {665.5, -44.8, 163.9}
    };

    cv::Point3f end_point_[13] = {
        {0, 0, 0},
        {-13.5671, 1.03407e-05, 2.7},
        {129.405, -287.628, 10.8939},
        {543.648, -522.705, 32.8722},
        {1363.09, 102.74, 10.20006},
        {1189.11, 424.908, 80.1408},
        {794.359, 682.403, 5.89},
        {-13.5633, 292.444, 3.84371},
        {710.375, 743.833, 14.8641},
        {1034.66, 496.756, 59.5603},
        {1340.24, -157.917, 35.3093},
        {689.856, -550.316, 49.5634},
        {260.547, -414.272, 2.46838}
    };

public:
    // 当前无人机位置（用于计算距离/悬停）
geometry_msgs::PoseStamped current_pose;
    // ================= ROS =================
    ros::Subscriber initial_pose_suber, end_pose_suber, gps_pose_suber;
    ros::Publisher waypoint_publisher, edited_gps_publisher;
    ros::Timer timer;

    // ================= 回调 =================
    void initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void end_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gps_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void timeCB(const ros::TimerEvent& event);

    // ================= 数据 =================
    geometry_msgs::Point station[13];
    geometry_msgs::Point Transit_hub[13];
    geometry_msgs::Point end_point[13];

    std::vector<std::vector<geometry_msgs::Point>> paths;
    std::vector<geometry_msgs::Point> path;

    int initial_num, end_num;
    bool initial_num_get = false;
    bool end_num_get = false;
    bool path_get  = false;

    // ================= ⭐⭐⭐ 只新增这些（最小改动） =================

    int stage = 1;   // 当前阶段

    bool hovering = false;   // 是否悬停
    ros::Time hover_start_time;
    double hover_duration = 3.0;

    std::vector<int> stage_targets = {3,5};  // 目标点（路线3）

    // ================= 工具 =================
    void POintSet();

    PathSender(ros::NodeHandle *nh);
    ~PathSender();
};

#endif