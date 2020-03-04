/* 
 * scout_webots_interface.hpp
 * 
 * Created on: Sep 26, 2019 23:04
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_WEBOTS_INTERFACE_HPP
#define SCOUT_WEBOTS_INTERFACE_HPP

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include "scout_base/scout_messenger.hpp"

namespace wescore
{
class ScoutWebotsInterface
{
public:
    ScoutWebotsInterface(ros::NodeHandle *nh, ScoutROSMessenger* msger, uint32_t time_step);

    void InitComponents();
    void UpdateSimState();

private:
    ros::NodeHandle *nh_;
    ScoutROSMessenger* messenger_;
    uint32_t time_step_;

    ros::Subscriber pointcloud_sub_;
    ros::Publisher pointcloud2_pub_;

    ros::Subscriber accel_sub_;
    ros::Subscriber gyro_sub_;
    ros::Publisher imu_pub_;

    sensor_msgs::Imu imu_data_;
    uint32_t imu_msg_seq_ = 0;

    const std::string robot_name_ = "agilex_scout";
    const std::vector<std::string> motor_names_{"motor_fr", "motor_fl", "motor_rl", "motor_rr"};

    void LidarCallback(const sensor_msgs::PointCloud::ConstPtr &pc);
    void MPU6050AccelCallback(const sensor_msgs::Imu::ConstPtr &accel);
    void MPU6050GyroCallback(const sensor_msgs::Imu::ConstPtr &gyro);
};
} // namespace wescore

#endif /* SCOUT_WEBOTS_INTERFACE_HPP */
