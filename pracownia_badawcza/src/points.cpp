#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include "../include/pracownia_badawcza/points.hpp"

Point::Point(int x,int y,std::string name,float r,float g,float b,float roll,float pitch,float yaw,float w){
    marker_pub = n.advertise<visualization_msgs::Marker>(name, 1000);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = roll;
    marker.pose.orientation.y = pitch;
    marker.pose.orientation.z = yaw;
    marker.pose.orientation.w = w;
    marker.scale.x=4.0;
    marker.scale.y=1.2;
    marker.scale.z=0.01;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
}
void Point::publish(){
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
  double q_start_x = 0.0;
  double q_start_y = 0.0;
  double q_start_z = -0.258819;
  double q_start_w = 0.9659258;
  // double q_start_z = 0.0;
  // double q_start_w = 1.0;

  // double q_goal_x = 0.0;
  // double q_goal_y = 0.0;
  // double q_goal_z = 0.0;
  // double q_goal_w = 1.0;

  double q_goal_x = 0.0;
  double q_goal_y = 0.0;
  double q_goal_z = -0.1305262;
  double q_goal_w = 0.9914449;
  // double q_goal_z = 0.0;
  // double q_goal_w = 1.0;
  // double q_goal_z = 1.0;
  // double q_goal_w = 0.0;

 
  ros::init(argc, argv, "points");
  //Point start_point(310.5,301.5,"start_point",1.0,1.0,0.5,q_start_x,q_start_y,q_start_z,q_start_w);
  Point start_point(150.0,53.0,"start_point",0.0,1.0,0.0,q_start_x,q_start_y,q_start_z,q_start_w); //----------- dziala ----------- 250 i 53 obrot -30
  // Point start_point(250.0,333.0,"start_point",0.0,1.0,0.0,q_start_x,q_start_y,q_start_z,q_start_w); //part2
  // map1
  // Point end_point(210.5,351.5,"end_point",1.0,0.0,0.0);
  // map4
  //Point end_point(210.5,301.5,"end_point",1.0,0.0,0.0,q_goal_x,q_goal_y,q_goal_z,q_goal_w);
  //Point end_point(350.0, 370.0,"end_point",1.0,0.0,0.0,q_goal_x,q_goal_y,q_goal_z,q_goal_w);
  Point end_point(320.0,80.0,"end_point",1.0,0.0,0.0,q_goal_x,q_goal_y,q_goal_z,q_goal_w); // ----------- dziala ----------- 320 i 80 obrot 15
  //Point end_point(290.0,203.0,"end_point",1.0,0.0,0.0,q_goal_x,q_goal_y,q_goal_z,q_goal_w); //part2
  //Point robot(start_point.marker.pose.position.x,start_point.marker.pose.position.y,"robot_coordinates",0.0,0.0,1.0);
  ros::Rate loop_rate(10);



  while (ros::ok())
  {

  start_point.publish();
  end_point.publish();
  //robot.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

