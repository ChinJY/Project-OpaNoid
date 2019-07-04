#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
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
	myFile = fopen("/home/cjy/Cpp_programs/data_log_full_legs_walk.txt", "r");
	int i;
	int ch, character = 0, line = 0;

	while ((ch = fgetc(myFile)) != EOF)
		{
			character++;
			if (ch == '\n')
				line++;
		}

	rewind(myFile);

	float leftThighX[line];
	float leftThighY[line];
	float numberArray2[line];
	float leftKnee[line];
	float rightThighX[line];
	float rightThighY[line];
	float numberArray6[line];
	float rightKnee[line];
	float rightAnkleX[line];
	float rightAnkleZ[line];
	float rightToe[line];
	float leftAnkleX[line];
	float leftAnkleZ[line];
	float leftToe[line];
	
	for (i = 0; i < line; i++)
	{
		fscanf(myFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f", &leftThighX[i], &leftThighY[i], &numberArray2[i], &leftKnee[i], &rightThighX[i], &rightThighY[i], &numberArray6[i], &rightKnee[i], &leftAnkleX[i], &leftAnkleZ[i], &leftToe[i], &rightAnkleX[i], &rightAnkleZ[i], &rightToe[i]);
	}

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/servo_controller/command", 1000);

//  ros::Rate loop_rate(10);

  std_msgs::Float64MultiArray msg;

	for (i = 0; i < line; i=i+25)
	{
		msg.data = {leftThighX[i]/90*1.5707, leftThighY[i]/90*1.5707, leftKnee[i]/90*1.5707, rightThighX[i]/90*1.5707, rightThighY[i]/90*1.5707, rightKnee[i]/90*1.5707, leftAnkleX[i]/90*1.5707, leftAnkleZ[i]/90*1.5707, leftToe[i]/90*1.5707, rightAnkleX[i]/90*1.5707, rightAnkleZ[i]/90*1.5707, rightToe[i]/90*1.5707};
		ROS_INFO("%f %f %f %f %f %f %f %f %f %f %f %f", leftThighX[i], leftThighY[i], leftKnee[i], rightThighX[i], rightThighY[i], rightKnee[i], leftAnkleX[i], leftAnkleZ[i], leftToe[i], rightAnkleX[i], rightAnkleZ[i], rightToe[i]);
		chatter_pub.publish(msg);

//	  ros::spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	
  return 0;
}
