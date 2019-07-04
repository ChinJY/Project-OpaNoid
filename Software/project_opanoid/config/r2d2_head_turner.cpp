#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
//	Read log file
	FILE *myFile;
	myFile = fopen("/home/cjy/C++ programs/head_turn_instructions.txt", "r");
	int i;
	int ch, character = 0, line = -1;

	while ((ch = fgetc(myFile)) != EOF)
		{
			character++;
			if (ch == '\n')
				line++;
		}

	rewind(myFile);

	float numberArray[line];

	for (i = 0; i < line + 1; i++)
	{
		fscanf(myFile, "%f", &numberArray [i]);
	}

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/r2d2_head_controller/command", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  std_msgs::Float64 msg;
	msg.data = 0;

//  while (ros::ok())
//  {
		for (i = 0; i < line + 1; i++)
		{
			msg.data = numberArray[i];
			ROS_INFO("%f", msg.data);
			chatter_pub.publish(msg);

		  ros::spinOnce();
			std::this_thread::sleep_for(std::chrono::milliseconds(500));

//		  loop_rate.sleep();
		}
//  }


  return 0;
}
