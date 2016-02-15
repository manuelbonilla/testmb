
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include "testmb/vito_skelethon.hpp"

std::vector<std::string> getSkelethonPoints(std::string branch)
{
    std::vector<std::string> links_in_brach;
    XmlRpc::XmlRpcValue my_list;
    nh_.getParam(links + "/links", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
     
    for ( int i = 0; i < my_list.size(); ++i) 
      {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        links_in_brach.push_back(static_cast<std::string>(my_list[i]).c_str());
        
      }

      return links_in_brach;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ParseVito");
  ros::NodeHandle node("~");

  VitoSkelethon vito_skelethon;
    double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);
  ros::Rate rate(spin_rate); 

  std::string  base;
  std::vector<std::string> list_brach_names;
  list_brach_names.push_back("left_arm");
  list_brach_names.push_back("right_arm");

  // node.setParam("global_param", 5);

  // if (!node.getParam("left_arm/end_effector", left_arm_tip)){
  //   ROS_INFO("Not parameter for left_arm/end_effector");
  //   left_arm_tip = "left_arm_7_link";
  // }
  // if (!node.getParam("right_arm/end_effector", right_arm_tip))
  //   right_arm_tip = "right_arm_7_link";
  // if (!node.getParam("base", base))
  //   base = "world";


  std::vector<std::string> tip_names;
  tip_names.push_back(left_arm_tip);
  tip_names.push_back(right_arm_tip);
  ROS_INFO("Using Base: %s", base.c_str());
  ROS_INFO("Using left tip link %s", left_arm_tip.c_str());
  ROS_INFO("Using right tip link %s", right_arm_tip.c_str());
  



  for (unsigned int i; i< tip_names.size(); i++)
  {

    
    getSkelethonPoints(std::string branch)
    if(!vito_skelethon.addChain(base.c_str(), tip_names[i] )){
      ROS_ERROR("Vino non parsed");
    }
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

