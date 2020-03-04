/* 
 * scout_webots_interface.cpp
 * 
 * Created on: Sep 26, 2019 23:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_webots_sim/scout_webots_interface.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>

#include <webots_ros/set_float.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>

#include "scout_webots_sim/scout_sim_params.hpp"

namespace wescore
{
ScoutWebotsInterface::ScoutWebotsInterface(ros::NodeHandle *nh, ScoutROSMessenger *msger, uint32_t time_step)
    : nh_(nh), messenger_(msger), time_step_(time_step)
{
    pointcloud2_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("rslidar_points", 50);
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>("imu", 50);

    imu_data_.orientation.x = 0.0;
    imu_data_.orientation.y = 0.0;
    imu_data_.orientation.z = 0.0;
    imu_data_.orientation.w = 0.0;

    imu_data_.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_data_.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    imu_data_.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
}

void ScoutWebotsInterface::InitComponents()
{
    // init motors
    for (int i = 0; i < 4; ++i)
    {
        // position
        webots_ros::set_float set_position_srv;
        ros::ServiceClient set_position_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                                           std::string("/set_position"));

        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
            ROS_INFO("Position set to INFINITY for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_position on motor %s.", motor_names_[i].c_str());

        // speed
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv;
        set_velocity_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                        std::string("/set_velocity"));

        set_velocity_srv.request.value = 0.0;
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }

    // enable lidar
    ros::ServiceClient set_lidar_client;
    webots_ros::set_int lidar_srv;
    ros::ServiceClient set_lidar_pointcloud_client;
    webots_ros::set_bool lidar_pointcloud_srv;

    set_lidar_pointcloud_client = nh_->serviceClient<webots_ros::set_bool>(robot_name_ + "/rslidar_16/enable_point_cloud");
    lidar_pointcloud_srv.request.value = true;

    set_lidar_client = nh_->serviceClient<webots_ros::set_int>(robot_name_ + "/rslidar_16/enable");
    lidar_srv.request.value = time_step_;

    if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success)
    {
        if (set_lidar_pointcloud_client.call(lidar_pointcloud_srv) && lidar_pointcloud_srv.response.success)
        {

            ROS_INFO("Lidar enabled.");
            pointcloud_sub_ = nh_->subscribe(robot_name_ + "/rslidar_16/point_cloud", 10, &ScoutWebotsInterface::LidarCallback, this);
            ROS_INFO("Topic for lidar initialized.");
            while (pointcloud_sub_.getNumPublishers() == 0)
            {
            }
            ROS_INFO("Topic for lidar scan connected.");
        }
    }
    else
    {
        if (!lidar_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable lidar.");
    }

    // enable camera
    ros::ServiceClient set_camera_client;
    webots_ros::set_int camera_srv;
    ros::Subscriber sub_camera;
    set_camera_client = nh_->serviceClient<webots_ros::set_int>(robot_name_ + "/camera/enable");
    camera_srv.request.value = 64;
    set_camera_client.call(camera_srv);

    // enable accelerometer
    ros::ServiceClient set_accelerometer_client;
    webots_ros::set_int accelerometer_srv;
    ros::Subscriber sub_accelerometer;
    set_accelerometer_client = nh_->serviceClient<webots_ros::set_int>(robot_name_ + "/mpu6050_accel/enable");
    accelerometer_srv.request.value = time_step_;
    if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success)
    {
        accel_sub_ = nh_->subscribe(robot_name_ + "/mpu6050_accel/values", 10, &ScoutWebotsInterface::MPU6050AccelCallback, this);
        while (accel_sub_.getNumPublishers() == 0)
        {
        }
        ROS_INFO("Topic for accelerometer connected.");
    }
    else
    {
        ROS_ERROR("Failed to enable accelerometer.");
    }

    // enable gyro
    ros::ServiceClient set_gyro_client;
    webots_ros::set_int gyro_srv;
    ros::Subscriber sub_gyro;
    set_gyro_client = nh_->serviceClient<webots_ros::set_int>(robot_name_ + "/mpu6050_gyro/enable");
    gyro_srv.request.value = time_step_;
    if (set_gyro_client.call(gyro_srv) && gyro_srv.response.success)
    {
        gyro_sub_ = nh_->subscribe(robot_name_ + "/mpu6050_gyro/values", 10, &ScoutWebotsInterface::MPU6050GyroCallback, this);
        while (gyro_sub_.getNumPublishers() == 0)
        {
        }
        ROS_INFO("Topic for gyro connected.");
    }
    else
    {
        ROS_ERROR("Failed to enable gyro.");
    }
}

void ScoutWebotsInterface::LidarCallback(const sensor_msgs::PointCloud::ConstPtr &pc)
{
    // convert pointcloud to pointcloud2
    sensor_msgs::PointCloud2 output;
    sensor_msgs::convertPointCloudToPointCloud2(*pc, output);
    output.header.frame_id = "rslidar";

    sensor_msgs::PointCloud2 converted_pc;
    tf::Quaternion rotation;
    Eigen::Matrix4f webots_ref_trans = Eigen::Matrix4f::Zero();
    Eigen::Matrix3f rot;
    // rot = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX());
    rot = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            webots_ref_trans(i, j) = rot(i, j);
    webots_ref_trans(3, 3) = 1;
    pcl_ros::transformPointCloud(webots_ref_trans, output, converted_pc);

    pointcloud2_pub_.publish(converted_pc);
    // pointcloud2_pub_.publish(output);
}

void ScoutWebotsInterface::MPU6050AccelCallback(const sensor_msgs::Imu::ConstPtr &accel)
{
    imu_data_.linear_acceleration.x = accel->linear_acceleration.x;
    imu_data_.linear_acceleration.y = accel->linear_acceleration.y;
    imu_data_.linear_acceleration.z = accel->linear_acceleration.z;
}

void ScoutWebotsInterface::MPU6050GyroCallback(const sensor_msgs::Imu::ConstPtr &gyro)
{
    imu_data_.angular_velocity.x = gyro->angular_velocity.x;
    imu_data_.angular_velocity.y = gyro->angular_velocity.y;
    imu_data_.angular_velocity.z = gyro->angular_velocity.z;

    imu_data_.header.seq = imu_msg_seq_++;
    imu_data_.header.stamp = ros::Time::now();
    imu_data_.header.frame_id = "imu_link";

    imu_pub_.publish(imu_data_);
}

void ScoutWebotsInterface::UpdateSimState()
{
    // constants for calculation
    constexpr double rotation_radius = std::hypot(ScoutSimParams::wheelbase / 2.0, ScoutSimParams::track / 2.0) * 2.0;
    constexpr double rotation_theta = std::atan2(ScoutSimParams::wheelbase, ScoutSimParams::track);

    // update robot state
    double wheel_speeds[4];
    for (int i = 0; i < 4; ++i)
    {
        webots_ros::get_float get_velocity_srv;
        ros::ServiceClient get_velocity_client = nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                                           std::string("/get_velocity"));

        if (get_velocity_client.call(get_velocity_srv))
        {
            wheel_speeds[i] = get_velocity_srv.response.value;
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        }
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }
    float left_speed = (wheel_speeds[1] + wheel_speeds[2]) / 2.0 * ScoutSimParams::wheel_radius;
    float right_speed = (wheel_speeds[0] + wheel_speeds[3]) / 2.0 * ScoutSimParams::wheel_radius;
    double linear_speed = (right_speed + left_speed) / 2.0;
    double angular_speed = (right_speed - left_speed) * std::cos(rotation_theta) / rotation_radius;

    messenger_->PublishSimStateToROS(linear_speed, angular_speed);

    // send robot command
    double linear, angular;
    messenger_->GetCurrentMotionCmdForSim(linear, angular);

    if (linear > ScoutSimParams::max_linear_speed)
        linear = ScoutSimParams::max_linear_speed;
    if (linear < -ScoutSimParams::max_linear_speed)
        linear = -ScoutSimParams::max_linear_speed;

    if (angular > ScoutSimParams::max_angular_speed)
        angular = ScoutSimParams::max_angular_speed;
    if (angular < -ScoutSimParams::max_angular_speed)
        angular = -ScoutSimParams::max_angular_speed;

    double vel_left_cmd = (linear - angular * rotation_radius / std::cos(rotation_theta)) / ScoutSimParams::wheel_radius;
    double vel_right_cmd = (linear + angular * rotation_radius / std::cos(rotation_theta)) / ScoutSimParams::wheel_radius;

    double wheel_cmds[4];
    wheel_cmds[0] = vel_right_cmd;
    wheel_cmds[1] = vel_left_cmd;
    wheel_cmds[2] = vel_left_cmd;
    wheel_cmds[3] = vel_right_cmd;
    for (int i = 0; i < 4; ++i)
    {
        ros::ServiceClient set_velocity_client;
        webots_ros::set_float set_velocity_srv;
        set_velocity_client = nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" + std::string(motor_names_[i]) +
                                                                        std::string("/set_velocity"));

        set_velocity_srv.request.value = wheel_cmds[i];
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
        else
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_[i].c_str());
    }
}

} // namespace wescore