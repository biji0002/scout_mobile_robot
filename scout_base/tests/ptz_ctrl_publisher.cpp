#include "ros/ros.h"
#include "scout_msgs/PTZ.h"
#include <cstdlib>
#include <sstream>
#include <stdio.h>




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


	ros::Rate loop_rate(10);
	int cnt = 0;

	while(ros::ok())
	{
		while (cnt<100)
		{

			pub.publish(msg1);
			ROS_INFO("counter clockwise");
			cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}

		//pub.publish(msg4);
		//pub.publish(msg4);
		//ROS_INFO("stop");
		while (cnt>=100 && cnt<180)
		{

			pub.publish(msg2);
			ROS_INFO("up");
			cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}

		while (cnt>=180 && cnt<280)
		{

			pub.publish(msg0);
			ROS_INFO("up");
			cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		while (cnt>=280 && cnt<360)
		{

			pub.publish(msg3);
			ROS_INFO("down");
			cnt++;
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		if (cnt==120)
			cnt = 0;

	}

	
	

	
    return 0;
}

/*
			pub.publish(msg3);
			pub.publish(msg3);
			pub.publish(msg3);
			pub.publish(msg3);
			ROS_INFO("up");
			ros::Duration(3).sleep();

			//ROS_INFO("stop clockwise");

			pub.publish(msg2);
			pub.publish(msg2);
			pub.publish(msg2);
			pub.publish(msg2);
			ROS_INFO("down");		
			ros::Duration(3).sleep();

			pub.publish(msg0);
			pub.publish(msg0);
			pub.publish(msg0);
			pub.publish(msg0);
			ROS_INFO("clockwise");		
			ros::Duration(3).sleep();

			pub.publish(msg1);
			pub.publish(msg1);
			pub.publish(msg1);
			pub.publish(msg1);
			ROS_INFO("counter clockwise");
*/
