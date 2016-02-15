
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include "testmb/vito_skelethon.hpp"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ParseVito");
  ros::NodeHandle node("~");

  VitoSkelethon vito_skelethon;
    double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);
  ros::Rate rate(spin_rate); 

  std::string left_arm_tip, right_arm_tip, base;

  node.setParam("global_param", 5);

  if (!node.getParam("left_arm/end_effector", left_arm_tip)){
    ROS_INFO("Not parameter for left_arm/end_effector");
    left_arm_tip = "left_arm_7_link";
  }
  if (!node.getParam("right_arm/end_effector", right_arm_tip))
    right_arm_tip = "right_arm_7_link";
  if (!node.getParam("base", base))
    base = "world";

  ROS_INFO("Using Base: %s", base.c_str());
  ROS_INFO("Using left tip link %s", left_arm_tip.c_str());
  ROS_INFO("Using right tip link %s", right_arm_tip.c_str());
  
  std::vector<std::string> list_of_collision_points
;
  if(!vito_skelethon.addChain(base, left_arm_tip, list_of_collision_points )){
    ROS_ERROR("Vino non parsed");
  }

  ROS_INFO ("++++");

  // std::vector<LineCollisions::Line> Lines = vito_skelethon.getLines();


  while (node.ok())
  {
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}

