/*
 * arduros.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: ams
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <getopt.h>
#include <sstream>
#include "FirmataProtocol.hpp"


using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
	/**
	 * Get the port and speed for the serial port if specified on the command line
	 */
	char *serSpeed = "9600";
	char *serPortName = "/dev/ttyUSB0";
	int c;

	while ((c = getopt(argc, argv, "s:p:")) != -1)
		switch (c) {
		case 's':
			serSpeed = optarg;
			break;
		case 'p':
			serPortName = optarg;
			break;
		default:
			break;
		}

	ros::init(argc, argv, "arduros");
	ros::NodeHandle n;

	/**
	 * Publishing string messages on the chatter topic with a queue of 1000
	 */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	/**
	 * Set up a Firmata Protocol instance and start using it
	 */
	FirmataProtocol fp;
	fp.init(serPortName);
	char reply[1024];
	fp.queryFirmware(reply);

	int count = 0;
	while (ros::ok()) {
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		std_msgs::String msg;

		//Stuff the message
		std::stringstream ss;
		ss << "hello world " << serSpeed << " " << serPortName;
		msg.data = ss.str();

		//For debugging
		ROS_INFO("%s", msg.data.c_str());

		//Publish the message
		chatter_pub.publish(msg);

		//Allow other stuff to run
		ros::spinOnce();

		//Sleep based on our loop rate and increment counter
		loop_rate.sleep();
		++count;
	}

	return 0;
}

