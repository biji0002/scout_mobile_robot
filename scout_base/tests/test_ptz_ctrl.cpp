#include "ros/ros.h"
#include "scout_msgs/JideTechPTZ.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ipcam_ptz_control");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<scout_msgs::JideTechPTZ>("ipcam_ptz_control");
    scout_msgs::JideTechPTZ srv;
    srv.request.ptz_cmd = srv.request.CMD_MOVE_UP;
    srv.request.param1 = 2;
    srv.request.param2 = 2;
    if (client.call(srv))
    {
        ROS_INFO("PTZ control requested");
    }
    else
    {
        ROS_ERROR("Failed to call service ipcam_ptz_control");
        return 1;
    }

    return 0;
}