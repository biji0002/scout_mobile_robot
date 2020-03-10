#include "ros/ros.h"
#include "scout_msgs/JideTechPTZ.h"
#include "scout_msgs/PTZ.h"
#include <cstdlib>


class ptz
{
public:
	ptz();
private:
	ros::NodeHandle nh;
	ros::ServiceClient client;
    ros::Subscriber listener;
    void messangeCallback(const scout_msgs::PTZ::ConstPtr& msg);
};


ptz::ptz()
{
	client = nh.serviceClient<scout_msgs::JideTechPTZ>("ipcam_ptz_control");
	listener = nh.subscribe("ptz_ctrl_pub",1000,&ptz::messangeCallback,this);
}



void ptz::messangeCallback(const scout_msgs::PTZ::ConstPtr& msg)
{
    scout_msgs::JideTechPTZ srv;
    srv.request.ptz_cmd = msg->ptz_cmd;
    srv.request.param1 = msg->param1;
    srv.request.param2 = msg->param2;

	if (client.call(srv))
	{
	    ROS_INFO("PTZ control requested");
	}
	else
	{
	    ROS_ERROR("Failed to call service ipcam_ptz_control");
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptz_service_call_node");
    ROS_INFO("PTZ_service_node start!");
    ptz ptz_ctrl;

	ros::spin();
    return 0;
}

    
