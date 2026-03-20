#ifndef _PATH_SENDER_CPP_
#define _PATH_SENDER_CPP_

#include <yaml-cpp/yaml.h>
#include "path_sender.hpp"
#include <filesystem>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_sender");
    ros::NodeHandle n;

    PathSender go(&n);
    return 0;
}

std::vector<std::vector<geometry_msgs::Point>> loadPathsFromYAML(const std::string &filename)
{
    std::vector<std::vector<geometry_msgs::Point>> paths;

    try {
        YAML::Node config = YAML::LoadFile(filename);
        if (!config["paths"]) {
            ROS_ERROR("YAML file does not contain 'paths' key.");
            return paths;
        }

        for (const auto &path_node : config["paths"]) {
            std::vector<geometry_msgs::Point> path;
            for (const auto &point_node : path_node.second) {
                if (point_node.size() != 3) continue;

                geometry_msgs::Point point;
                point.x = point_node[0].as<double>();
                point.y = point_node[1].as<double>();
                point.z = point_node[2].as<double>();

                path.push_back(point);
            }
            paths.push_back(path);
        }
    } catch (const YAML::Exception &e) {
        ROS_ERROR("Error loading YAML file: %s", e.what());
    }

    return paths;
}

void PathSender::POintSet()
{
    for (int i = 0; i < 13; ++i)
    {
        station[i].x = station_[i].x;
        station[i].y = station_[i].y;
        station[i].z = station_[i].z;

        Transit_hub[i].x = Transit_hub_[i].x;
        Transit_hub[i].y = Transit_hub_[i].y;
        Transit_hub[i].z = Transit_hub_[i].z;

        end_point[i].x = end_point_[i].x;
        end_point[i].y = end_point_[i].y;
        end_point[i].z = end_point_[i].z;
    }
}

PathSender::PathSender(ros::NodeHandle *nh)
{
    POintSet();

    paths = loadPathsFromYAML("/home/jason/IntelligentUAVChampionship/basic_dev/src/path_sender/config/paths.yaml");

    initial_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/initial_pose", 1,
        std::bind(&PathSender::initial_pose_cb, this, std::placeholders::_1));

    end_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/end_goal", 1,
        std::bind(&PathSender::end_pose_cb, this, std::placeholders::_1));

    gps_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/drone_1/gps", 1,
        std::bind(&PathSender::gps_pose_cb, this, std::placeholders::_1));

    timer = nh->createTimer(ros::Duration(1.0), &PathSender::timeCB, this);

    waypoint_publisher = nh->advertise<path_sender::WayPoints>("/waypoints", 1);
    edited_gps_publisher = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/airsim_node/drone_1/edited_gps", 1);

    ros::spin();
}

PathSender::~PathSender(){}

// ================= 起点识别 =================
void PathSender::initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!initial_num_get)
    {
        for(int i=1;i<=12;i++)
        {
            if(abs(msg->pose.position.x - station[i].x) <5)
            {
                initial_num = i;
                initial_num_get = true;
                break;
            }
        }
    }
}

// ================= 终点识别 =================
void PathSender::end_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!end_num_get)
    {
        for(int i=1;i<=12;i++)
        {
            if(abs(msg->pose.position.x - station[i].x) <5)
            {
                end_num = i;
                end_num_get = true;
                break;
            }
        }
    }
}

// ================= GPS回调（加入到达判断） =================
void PathSender::gps_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped edited_gps;
    edited_gps.header.stamp = ros::Time::now();
    edited_gps.header.frame_id = "map";
    edited_gps.pose.pose = msg->pose;

    edited_gps.pose.covariance[0] = 1e-7;
    edited_gps.pose.covariance[7] = 1e-7;
    edited_gps.pose.covariance[14] = 1e-7;
    edited_gps.pose.covariance[21] = 0.1;
    edited_gps.pose.covariance[28] = 0.1;
    edited_gps.pose.covariance[35] = 0.1;

    edited_gps_publisher.publish(edited_gps);

    // ===== 悬停判断（新增）=====
    if(hover_point_set && !reached_hover_point)
    {
        double dx = msg->pose.position.x - hover_point.x;
        double dy = msg->pose.position.y - hover_point.y;
        double dz = msg->pose.position.z - hover_point.z;

        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        ROS_WARN("dist to hover point: %.2f", dist);
        if(dist < 8.0)
        {
            reached_hover_point = true;
            ROS_WARN("Reached hover point!");
        }
    }
    
}

// ================= 主控制 =================
void PathSender::timeCB(const ros::TimerEvent& event)
{
    if(!path_get)
    {
        if(initial_num_get && end_num_get)
        {
            path = paths[initial_num-1];

            path.emplace_back(Transit_hub[initial_num]);
            path.emplace_back(Transit_hub[end_num]);

            std::vector<geometry_msgs::Point> temp_path = paths[end_num-1];
            std::reverse(temp_path.begin(), temp_path.end());

            path.insert(path.end(), temp_path.begin(), temp_path.end());

            path.emplace_back(end_point[end_num]);

            // ===== 设置悬停点 =====
            hover_point = end_point[end_num];
            hover_point_set = true;

            geometry_msgs::Point temp_point;

            temp_point = end_point[end_num];
            temp_point.z += 150;
            path.emplace_back(temp_point);

            temp_point = end_point[initial_num];
            temp_point.z += 150;
            path.emplace_back(temp_point);

            path.emplace_back(end_point[initial_num]);
            path.emplace_back(station[initial_num]);

            ROS_WARN("Path generated!");
            path_get = true;
        }
    }
    else
    {
        // ===== 悬停逻辑 =====
        if(reached_hover_point && !hovering)
        {
            hovering = true;
            hover_start_time = ros::Time::now();
            ROS_WARN("Start hovering...");
        }

        if(hovering)
        {
            double t = (ros::Time::now() - hover_start_time).toSec();

            if(t < hover_duration)
            {
                return; // 悬停期间停止发布路径
            }
            else
            {
                hovering = false;
                ROS_WARN("Hover finished!");
            }
        }

        // ===== 正常发布 =====
        path_sender::WayPoints path_msg;
        path_msg.points = this->path;
        waypoint_publisher.publish(path_msg);
    }
}

#endif
