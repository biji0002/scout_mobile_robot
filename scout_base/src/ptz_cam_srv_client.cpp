#include "ros/ros.h"
#include <cstdlib>
#include "scout_msgs/JideTechPTZ.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ipcam_ptz_control_client");
  if (argc != 4)
  {
    ROS_INFO("usage: ipcam_ptz_control_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<scout_msgs::JideTechPTZ>("ipcam_ptz_control");
  scout_msgs::JideTechPTZ srv;

  srv.request.ptz_cmd = atoll(argv[1]);
  srv.request.param1 = atoll(argv[2]);
  srv.request.param2 = atoll(argv[3]);

  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ROS_INFO("Message Received");
  }
  else
  {
    ROS_ERROR("Failed to call service ipcam_ptz_control");
    return 1;
  }

  return 0;
}
