#include <iostream>
#include <ros/ros.h>

#include "Timer.hpp"


int main(int argc, char* argv[]) {
	ros::init(argc,argv,"hello_node");
  	ros::NodeHandle nh;

  	ROS_INFO_STREAM("Hello, world!");	
	return 0;
}
