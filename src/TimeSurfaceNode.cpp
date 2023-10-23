#include <iostream>
#include <ros/ros.h>

#include "TimeSurface.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc,argv,"e_demo_time_surface");
    
    ros::NodeHandle nhPublic;
    ros::NodeHandle nhPrivate("~");

  	e_demo::TimeSurface ts(nhPublic, nhPrivate);
    
    ROS_INFO_STREAM("TimeSurface Running...");	
    ros::spin();
    return 0;
}
