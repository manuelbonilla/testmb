
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
// #include "testmb/vito_skelethon.hpp"
#include "testmb/distance_between_lines.hpp"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tuple>


  struct PointSkelethon
  {
    LineCollisions::Point P;
    std::string link_name;
    PointSkelethon(){};
    PointSkelethon(LineCollisions::Point Point_in, std::string string_in){P = Point_in; link_name = string_in;};
  }point_skelethon;

  struct LineSkelethon
  {
    PointSkelethon P1;
    PointSkelethon P2;
    LineSkelethon(){};
    LineSkelethon(PointSkelethon P1_in, PointSkelethon P2_in){P1 = P1_in; P2 = P2_in;};
  };

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

visualization_msgs::Marker plot_lines(std::vector<std::vector<LineSkelethon>> list_list_lines, std::string base,
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
      p1.x = list_list_lines[i][j].P1.P[0];
      p1.y = list_list_lines[i][j].P1.P[1];
      p1.z = list_list_lines[i][j].P1.P[2];

      geometry_msgs::Point p2;
      p2.x = list_list_lines[i][j].P2.P[0];
      p2.y = list_list_lines[i][j].P2.P[1];
      p2.z = list_list_lines[i][j].P2.P[2];

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
 
  while (node.ok())
  {
    tf::StampedTransform trans, trans2;
    std::vector<std::vector<LineSkelethon>> List_lines_in_all_chains, lines_and_collisions_multiple;
    std::vector<LineSkelethon> lines_and_collisions;

    for (unsigned int i = 0; i < links_all_branch.size(); ++i)
    {
      std::vector<LineSkelethon> list_lines_one_chain;
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
          PointSkelethon Point1( LineCollisions::Point(trasnX.getX(), trasnX.getY(), trasnX.getZ()), links_all_branch[i][j] );
          PointSkelethon Point2( LineCollisions::Point(trasnX2.getX(), trasnX2.getY(), trasnX2.getZ()), links_all_branch[i][j] );
          list_lines_one_chain.push_back( LineSkelethon( Point1, Point2) );        
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
            LineCollisions::Line L1(List_lines_in_all_chains[i][k].P1.P, List_lines_in_all_chains[i][k].P2.P);
            LineCollisions::Line L2(List_lines_in_all_chains[j][l].P1.P, List_lines_in_all_chains[j][l].P2.P);
            LineCollisions::Line collision_line = LineCollisionsLocal.getClosestPoints( L1, L2 );
            if (collision_line.norm <= .15)
            {
              lines_and_collisions.push_back( LineSkelethon( PointSkelethon( collision_line.P1, List_lines_in_all_chains[i][k].P1.link_name ),
                                                             PointSkelethon( collision_line.P2, List_lines_in_all_chains[j][l].P1.link_name ) )) ;
              ROS_INFO_STREAM("Collision in Point:" << lines_and_collisions.back().P1.P.transpose() << " of link " << lines_and_collisions.back().P1.link_name);
              ROS_INFO_STREAM("Collision in Point:" << lines_and_collisions.back().P2.P.transpose() << " of link " << lines_and_collisions.back().P2.link_name);
            }
          }
        }
      }
    }

    // list_pub.publish(plot_lines(List_lines_in_all_chains, base, 1.0, 0.0, 0.0, 0, visualization_msgs::Marker::LINE_LIST)); 
    point_pub.publish(plot_lines(List_lines_in_all_chains, base, 0.0, 1.0, 0.0, 1, visualization_msgs::Marker::POINTS)); 
    
    std::vector<std::vector<LineSkelethon>> lis;
    lis.push_back(lines_and_collisions);
    collisions_lines_pub.publish(plot_lines(lis, base, 0.0, 0.0, 1.0, 2, visualization_msgs::Marker::LINE_LIST));
    collisions_points_pub.publish(plot_lines(lis, base, 1.0, 0.0, 1.0, 3, visualization_msgs::Marker::POINTS));

    ros::spinOnce(); 
    rate.sleep();

  }
  return 0;
}

