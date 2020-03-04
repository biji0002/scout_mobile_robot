#include "ros/ros.h"
#include <cstdlib>
#include "scout_msgs/JideTechPTZ.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ipcam_ptz_control_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<scout_msgs::JideTechPTZ>("ipcam_ptz_control",true);
  scout_msgs::JideTechPTZ srv;
  ROS_INFO("anti clockwise rotate");
  srv.request.ptz_cmd = 9;  
  srv.request.param1 = 2;
  srv.request.param2 = 0;
  
  float ty = 90/11.69;
  ros::Duration(90/11.69).sleep();
  ROS_INFO("lookup");
  srv.request.ptz_cmd = 8;
  srv.request.param1 = 2;
  srv.request.param2 = 0;
  ROS_INFO("clockwise rotate");
  ros::Duration(10).sleep();
  ROS_INFO("stop");

  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ROS_INFO("Message Received1");
  }
  else
  {
    ROS_ERROR("Failed to call service ipcam_ptz_control");
    return 1;
  }
  
  return 0;
}


//vx_clockwise = 9
//vx_anti_clk = 9
//vy_up = 11.69
//vy_down = 11.3
//init_x = 0
//init_y = 0


/*
int ptz_right()
{
  float ty,tx;
  scout_msgs::JideTechPTZ srv;
  //ty = ptz_timer_y_up(0,90);
  //tx = ptz_timer_x_clockwise(0,90);
  ty = 90/11.69;
  tx = 90/9.0;
  srv.request.ptz_cmd = 6;
  ros::Duration(ty).sleep();
  srv.request.ptz_cmd = 8;
  ros::Duration(tx).sleep();

  return 0;
}

float ptz_timer_y_up(int init, int des)
{  return (des - init) / 11.69;  }
float ptz_timer_x_clockwise(int init, int des)
{  return (des - init) / 9.0;  }
*/
