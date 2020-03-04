/* 
 * imu_broadcast_node.cpp
 * 
 * Created on: Oct 02, 2019 19:09
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/wescore.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include "scout_msgs/UltrasonicW200D.h"
#include "scout_msgs/JideTechPTZ.h"

ros::Publisher imu_pub;
ros::Publisher range_pub[8];
ros::Publisher ultrasonic_pub;
ros::ServiceServer ptz_service;

struct MessageBroadcaster
{
    MessageBroadcaster()
    {
	lcm_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=1");

        // setup LCM
        if (!lcm_->good())
            std::cerr << "Failed to initialize LCM" << std::endl;

        lcm_->subscribe("sensor_hub_raw_imu", &MessageBroadcaster::IMULCMCallback, this);
        lcm_->subscribe("sensor_hub_ultrasonic", &MessageBroadcaster::UltrasonicLCMCallback, this);
		
    }

    void IMULCMCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const wescore_lcm_msgs::IMU *msg)
    {
        static uint64_t count = 0;
        // std::cout << "imu msg received" << std::endl;
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.seq = count++;

        imu_msg.angular_velocity.x = msg->angular_velocity.x;
        imu_msg.angular_velocity.y = msg->angular_velocity.y;
        imu_msg.angular_velocity.z = msg->angular_velocity.z;

        imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
        imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
        imu_msg.linear_acceleration.z = msg->linear_acceleration.z;

        for (int i = 0; i < 9; ++i)
        {
            imu_msg.orientation_covariance[i] = msg->orientation_covariance[i];
            imu_msg.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
            imu_msg.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
        }

        imu_pub.publish(imu_msg);
    }

    void UltrasonicLCMCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const wescore_lcm_msgs::UltrasonicW200D *msg)
    {
        static uint64_t count = 0;
        // std::cout << "ultrasonic msg received" << std::endl;
        scout_msgs::UltrasonicW200D ultrasonic_msg;

        ultrasonic_msg.header.stamp = ros::Time::now();
        ultrasonic_msg.header.seq = count++;

        for (int i = 0; i < 8; ++i)
        {
            ultrasonic_msg.header.frame_id = "ultrasonic_link_" + std::to_string(i);

            ultrasonic_msg.ranges[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
            ultrasonic_msg.ranges[i].field_of_view = msg->ranges[i].field_of_view;
            ultrasonic_msg.ranges[i].min_range = msg->ranges[i].min_range;
            ultrasonic_msg.ranges[i].max_range = msg->ranges[i].max_range;
            ultrasonic_msg.ranges[i].range = msg->ranges[i].range;
            range_pub[i].publish(ultrasonic_msg.ranges[i]);
        }

        ultrasonic_pub.publish(ultrasonic_msg);
    }

    bool JideTechPTZControlService(scout_msgs::JideTechPTZ::Request &req,
                                   scout_msgs::JideTechPTZ::Response &res)
    {
        ROS_INFO("PTZ control requested");
        
        wescore_lcm_msgs::JideTechCam ptz_msg;
        ptz_msg.ptz_cmd = req.ptz_cmd;
        ptz_msg.param1 = req.param1;
        ptz_msg.param2 = req.param2;
        lcm_->publish("sensor_hub_ipcam_ptz", &ptz_msg);

        res.ptz_cmd_received = true;
        return true;
    }


    std::shared_ptr<lcm::LCM> lcm_;
};
/*
void PtzLCMCallback(const wescore_lcm_msgs::JideTechCam &msg)
{	
//static uint64_t count = 0;
wescore_lcm_msgs::JideTechCam ptz_msg;

//ptz_msg.header.stamp = ros::Time::now();
//ptz_msg.header.seq = count++;

ptz_msg.ptz_cmd = 6;
ptz_msg.param1 = 0;
ptz_msg.param2 = 2;
//lcm_->publish("sensor_hub_ipcam_ptz", &ptz_msg);
}
*/

int main(int argc, char **argv)
{
    MessageBroadcaster mb;

    // setup ROS node
    ros::init(argc, argv, "sensorhub_node");
    ros::NodeHandle nh;

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);

    for (int i = 0; i < 8; ++i)
        range_pub[i] = nh.advertise<sensor_msgs::Range>("ultrasonic_range_" + std::to_string(i), 1000);
    ultrasonic_pub = nh.advertise<scout_msgs::UltrasonicW200D>("ultrasonic", 1000);

    ptz_service = nh.advertiseService("ipcam_ptz_control", &MessageBroadcaster::JideTechPTZControlService, &mb);
    //ros::Subscriber ptz_sub = nh.subscribe("ptz_auto_control", 1000, PtzLCMCallback,this);

    ROS_INFO("Started sensorhub broadcasting");

    while (ros::ok())
    {
        mb.lcm_->handleTimeout(5);
    	ros::spinOnce();
    }

    return 0;
}
