#include "scout_msgs/PTZ.h"
#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <ncurses.h>


void counter_clockwise(ros::Publisher pb,scout_msgs::PTZ call1);

void clockwise(ros::Publisher pb,scout_msgs::PTZ call0);

void going_up(ros::Publisher pb,scout_msgs::PTZ call2);

void going_down(ros::Publisher pb,scout_msgs::PTZ call3);

void pause(ros::Publisher pb,scout_msgs::PTZ call4);



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ptz_ctrl_publisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<scout_msgs::PTZ>("ptz_ctrl_pub",1000);

	scout_msgs::PTZ msg0;//clockwise
	msg0.ptz_cmd = 8;
	msg0.param1 = 2;
	msg0.param2 = 0;

	scout_msgs::PTZ msg1;//counter clockwise
	msg1.ptz_cmd = 9;
	msg1.param1 = 2;
	msg1.param2 = 0;

	scout_msgs::PTZ msg2;//up
	msg2.ptz_cmd = 6;
	msg2.param1 = 0;
	msg2.param2 = 2;

	scout_msgs::PTZ msg3;//down
	msg3.ptz_cmd = 7;
	msg3.param1 = 0;
	msg3.param2 = 2;

	scout_msgs::PTZ msg4;//stop
	msg4.ptz_cmd = 0;
	msg4.param1 = 0;
	msg4.param2 = 0;

	ROS_INFO("publisher ready");
	while(ros::ok())
	{
		ROS_INFO("ROS OK, waiting for command..");

		char i = getchar();

		switch(i)
		{
			case 'd':ROS_INFO("counter_clockwise");counter_clockwise(pub,msg1);break;
			case 'a':ROS_INFO("clockwise");clockwise(pub,msg0);break;
			case 'w':ROS_INFO("going_up");going_up(pub,msg2);break;
			case 's':ROS_INFO("going_down");going_down(pub,msg3);break;
			case 'q':ROS_INFO("pause");pause(pub,msg4);break;
		}
		

	}
	
	ros::spinOnce();
	
    return 0;
}

void counter_clockwise(ros::Publisher pb,scout_msgs::PTZ call1)
{	ros::Rate loop_rate(10);
	ROS_INFO("counter clockwise");
	pb.publish(call1);		
	loop_rate.sleep();
}

void clockwise(ros::Publisher pb,scout_msgs::PTZ call0)
{	ros::Rate loop_rate(10);
	ROS_INFO("clockwise");
	pb.publish(call0);		
	loop_rate.sleep();
}

void going_up(ros::Publisher pb,scout_msgs::PTZ call2)
{	ros::Rate loop_rate(10);
	ROS_INFO("up");
	pb.publish(call2);		
	loop_rate.sleep();
}

void going_down(ros::Publisher pb,scout_msgs::PTZ call3)
{	ros::Rate loop_rate(10);
	ROS_INFO("down");
	pb.publish(call3);		
	loop_rate.sleep();
}

void pause(ros::Publisher pb,scout_msgs::PTZ call4)
{	ros::Rate loop_rate(10);
	ROS_INFO("pause");
	pb.publish(call4);
	loop_rate.sleep();
}

