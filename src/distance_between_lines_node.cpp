
#include <ros/ros.h>
#include <ros/console.h>
#include "testmb/distance_between_lines.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DistanceBetweenLines");
  ros::NodeHandle node;
  LineCollisions LineCollisionsLocal;
  ROS_INFO("[DistanceBetweenLines] Node is ready");

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  ros::Rate rate(spin_rate); 

  LineCollisions::Point Point1(0.0,0.0,0.0);
  LineCollisions::Point Point2(-0.1,0.0,1.0);

  LineCollisions::Point Point3(2.0,0.0,2.0);
  LineCollisions::Point Point4(2.1,0,3.0);

  LineCollisions::Line Line1(Point1, Point2);
  LineCollisions::Line Line2(Point3, Point4);

  LineCollisions::Line ClosestPoints;
  
  ClosestPoints = LineCollisionsLocal.getClosestPoints(Line1, Line2);

  ROS_INFO_STREAM("The closest Points are:");
  ROS_INFO_STREAM("P1: ");
  ROS_INFO_STREAM(ClosestPoints.P1.transpose());
  ROS_INFO_STREAM("P2: ");
  ROS_INFO_STREAM(ClosestPoints.P2.transpose());
  ROS_INFO_STREAM("The distance is :");
  ROS_INFO_STREAM(ClosestPoints.norm);


     // Line

  // while (node.ok())
  // {
    ros::spinOnce(); 
    rate.sleep();
  // }
  return 0;
}

