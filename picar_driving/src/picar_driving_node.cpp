#include <ros/ros.h>
#include <ros/package.h>
#include "../include/picar_driving/picar_driving.h"


int main (int argc, char** argv)
{
  ros::init (argc, argv, "picar_driving");
  
  // New instance, constructor initializes all the path segments
  Driver node;
  node.configure();

  ros::Rate loop_rate(5); // Hz
  while (ros::ok())
  {
    node.update();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
