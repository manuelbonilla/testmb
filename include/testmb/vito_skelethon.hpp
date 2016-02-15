#ifndef VITO_SKELETHON_HPP
#define VITO_SKELETHON_HPP

// #include <ros/ros.h>
#include <iostream>
#include <kdl_wrapper/kdl_wrapper.h>
#include "testmb/distance_between_lines.hpp"


class VitoSkelethon
{
private:
  // KDLWrapper left_arm, right_arm;
  std::vector<KDLWrapper> list_of_chains;

public:
  VitoSkelethon(){};
  ~VitoSkelethon(){};
  bool addChain(std::string root, std::string tip_name, std::vector<std::string> list_of_collision_points);
  std::vector<LineCollisions::Line> getLines();
  
};

bool VitoSkelethon::addChain(std::string root, std::string tip_name, std::vector<std::string> list_of_collision_points) 
{

    KDLWrapper kdlwrapper_temp;
    list_of_chains.push_back(kdlwrapper_temp);
    if (list_of_chains.back().init(root.c_str(), tip_name.c_str()))
    {
      std::cout << "Chain from " << root.c_str() << " to " << tip_name.c_str() << " initialized" << std::endl;
    }
    else
    {
      std::cout << "Not possible to initialize chain from " << root.c_str() << " to " << tip_name.c_str() << std::endl;
      return false;
    }
  return true;
}

std::vector<LineCollisions::Line> VitoSkelethon::getLines(){

  // std::cout << "Num Segments: " << left_arm.getKDLChain().getNrOfSegments();

  unsigned int i = 0;
  KDL::Chain chain_temp;
  // for (unsigned int i = 0; i < chain_temp.getNrOfSegments(); ++i)
  // {
  //   if (chain_temp.getSegment(i) ==
  // }


  // for (int i = 0; i < left_arm.getKDLChain().getNrOfSegments(); ++i)
  // {
  //    code 
  // }

  std::vector<LineCollisions::Line> Lines;
  return Lines;

}



#endif