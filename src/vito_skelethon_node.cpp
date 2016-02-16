
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
// #include "testmb/vito_skelethon.hpp"
#include "testmb/distance_between_lines.hpp"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tuple>

// std::vector<sensor_msgs::JointState::Ptr> joint_states_vec(list_brach_names.size());

std::vector<std::string> getSkelethonPoints(ros::NodeHandle &node, std::string branch)
{
    std::vector<std::string> links_in_brach;
    XmlRpc::XmlRpcValue my_list;
    node.getParam((branch + "links").c_str(), my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
     
    for ( int i = 0; i < my_list.size(); ++i) 
      {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        links_in_brach.push_back(static_cast<std::string>(my_list[i]).c_str());
        
      }

      return links_in_brach;
}

visualization_msgs::Marker plot_lines(std::vector<std::vector<LineCollisions::Line>> list_list_lines, std::string base,
                     float r = 1.0, float g = 1.0, float b = 1.0, int id = 0, int type = visualization_msgs::Marker::POINTS)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = base;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = type;
  line_list.id = id;
  line_list.scale.x = 0.01;
  line_list.color.r = r;
  line_list.color.g = g;
  line_list.color.b = b;
  line_list.color.a = 1.0;

  for (unsigned i = 0; i < list_list_lines.size(); ++i)
  {
    for (unsigned int j = 0; j < list_list_lines[i].size(); ++j)
    {
      geometry_msgs::Point p1;
      p1.x = list_list_lines[i][j].P1[0];
      p1.y = list_list_lines[i][j].P1[1];
      p1.z = list_list_lines[i][j].P1[2];

      geometry_msgs::Point p2;
      p2.x = list_list_lines[i][j].P2[0];
      p2.y = list_list_lines[i][j].P2[1];
      p2.z = list_list_lines[i][j].P2[2];

      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
    }
  }

  return line_list;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ParseVito");
  ros::NodeHandle node("~");

  // VitoSkelethon vito_skelethon;
   LineCollisions LineCollisionsLocal;
    double spin_rate = 1000;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);
  ros::Rate rate(spin_rate); 

  std::string  base("world");
  double threshold = .15;
  std::vector<std::string> list_brach_names;
  list_brach_names.push_back("left_arm/");
  list_brach_names.push_back("right_arm/");

  node.param<std::string>("base", base, "world");
  node.param<double>("threshold", threshold, .15);



  std::vector<std::string> tip_names;

  ROS_INFO_STREAM("Using Base: " <<  base.c_str());
  ROS_INFO_STREAM("Threshold: " <<  threshold);

  tf::TransformListener tf_listener;

  std::vector<std::vector<std::string>> links_all_branch;

  ros::Publisher list_pub = node.advertise<visualization_msgs::Marker>("skelethon_lines", 10);
  ros::Publisher point_pub = node.advertise<visualization_msgs::Marker>("skelethon_point", 10);
  ros::Publisher collisions_lines_pub = node.advertise<visualization_msgs::Marker>("skelethon_collision_lines", 10);
  ros::Publisher collisions_points_pub = node.advertise<visualization_msgs::Marker>("skelethon_collision_point", 10);

  for (unsigned int i = 0; i< list_brach_names.size(); i++)
  {

    std::vector<std::string> links_in_brach = getSkelethonPoints(node, list_brach_names[i]);
    links_all_branch.push_back(links_in_brach);

  } 
 
  // typedef std::tuple <LineCollisions::Line, LineCollisions::Line, LineCollisions::Line > tuple_lines;
  // std::vector<tuple_lines> lines_and _collisions;

  while (node.ok())
  {
    tf::StampedTransform trans, trans2;
    std::vector<std::vector<LineCollisions::Line>> List_lines_in_all_chains, lines_and_collisions_multiple;
    std::vector<LineCollisions::Line> lines_and_collisions;

    for (unsigned int i = 0; i < links_all_branch.size(); ++i)
    {
      std::vector<LineCollisions::Line> list_lines_one_chain;
      for (unsigned int j = 0; j < links_all_branch[i].size() -1 ; ++j)
      {
        try
        {
          tf_listener.waitForTransform(base, links_all_branch[i][j], ros::Time(0), ros::Duration(2.0));
          tf_listener.lookupTransform( base , links_all_branch[i][j], ros::Time(0), trans);
          tf_listener.waitForTransform(base, links_all_branch[i][j+1], ros::Time(0), ros::Duration(2.0));
          tf_listener.lookupTransform( base , links_all_branch[i][j+1], ros::Time(0), trans2);
          tf::Vector3 trasnX = trans.getOrigin();
          tf::Vector3 trasnX2 = trans2.getOrigin();
          LineCollisions::Point Point1( trasnX.getX(), trasnX.getY(), trasnX.getZ() );
          LineCollisions::Point Point2( trasnX2.getX(), trasnX2.getY(), trasnX2.getZ() );
          list_lines_one_chain.push_back( LineCollisions::Line( Point1, Point2) );        
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR("%s", ex.what()); 
        }        
      } 
      List_lines_in_all_chains.push_back(list_lines_one_chain);      
    }

    for (unsigned int i = 0; i < List_lines_in_all_chains.size() - 1; ++i)
    {
      for (unsigned int j = i+1; j < List_lines_in_all_chains.size() ; ++j)
      {
        for (unsigned int k = 0; k < List_lines_in_all_chains[i].size() ; ++k)
        {
          for (unsigned int l = 0; l < List_lines_in_all_chains[j].size(); ++l)
          {
            LineCollisions::Line collision_line = LineCollisionsLocal.getClosestPoints( List_lines_in_all_chains[i][k], List_lines_in_all_chains[j][l] );
            if (collision_line.norm <= .15){
              lines_and_collisions.push_back( collision_line ) ;
            }
          }
        }
      }
    }


    // ROS_INFO_STREAM("Num lines for collision " << lines_and_collisions.size());

    list_pub.publish(plot_lines(List_lines_in_all_chains, base, 1.0, 0.0, 0.0, 0, visualization_msgs::Marker::LINE_LIST)); 
    point_pub.publish(plot_lines(List_lines_in_all_chains, base, 0.0, 1.0, 0.0, 1, visualization_msgs::Marker::POINTS)); 
    
    std::vector<std::vector<LineCollisions::Line>> lis;
    lis.push_back(lines_and_collisions);
    collisions_lines_pub.publish(plot_lines(lis, base, 0.0, 0.0, 1.0, 2, visualization_msgs::Marker::LINE_LIST));
    collisions_points_pub.publish(plot_lines(lis, base, 1.0, 0.0, 1.0, 3, visualization_msgs::Marker::POINTS));
    // tf_listener.lookupTransform(base, (which+arm_names[i]), ros::Time(0), trans);

    ros::spinOnce(); 
    rate.sleep();

  }
  return 0;
}

