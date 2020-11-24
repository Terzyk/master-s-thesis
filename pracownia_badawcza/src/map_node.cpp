#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/MapMetaData.h>
#include "std_msgs/Header.h"
#include "../include/pracownia_badawcza/ompl_lib.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>



// global variables for subscirbers
nav_msgs::OccupancyGrid globalMap;
// variables for points coordinates from file
visualization_msgs::Marker start2_point;
visualization_msgs::Marker goal2_point;
//geometry_msgs::Point start2_point;
//geometry_msgs::Point goal2_point;
// robot coordinates
visualization_msgs::Marker robot_coord;
// time variables 
ros::WallTime start_planning, end_planning;
// variable for counting, how many times should the program plan path, if = 1 only one path will be made
int iter_paths = 10;
// variable for counting, how many times program planned path
int made_paths = 0;
// robot coord publisher
ros::Publisher marker_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  globalMap=*msg;
}

void start2Callback(const visualization_msgs::Marker::ConstPtr& msg)
{
    start2_point.pose.position.x = msg->pose.position.x;
    start2_point.pose.position.y = msg->pose.position.y;
    start2_point.pose.position.z = msg->pose.position.z;
    start2_point.pose.orientation.x = msg->pose.orientation.x;
    start2_point.pose.orientation.y = msg->pose.orientation.y;
    start2_point.pose.orientation.z = msg->pose.orientation.z;
    start2_point.pose.orientation.w = msg->pose.orientation.w;
}

void goal2Callback(const visualization_msgs::Marker::ConstPtr& msg)
{
    goal2_point.pose.position.x = msg->pose.position.x;
    goal2_point.pose.position.y = msg->pose.position.y;
    goal2_point.pose.position.z = msg->pose.position.z;
    goal2_point.pose.orientation.x = msg->pose.orientation.x;
    goal2_point.pose.orientation.y = msg->pose.orientation.y;
    goal2_point.pose.orientation.z = msg->pose.orientation.z;
    goal2_point.pose.orientation.w = msg->pose.orientation.w;
}
/*
void start2Callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  start2_point.x = msg->pose.position.x;
  start2_point.y = msg->pose.position.y;
  start2_point.z = msg->pose.position.z;
  
}

void goal2Callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  goal2_point.x = msg->pose.position.x;
  goal2_point.y = msg->pose.position.y;
  goal2_point.z = msg->pose.position.z;
}
*/
void robot_move(nav_msgs::Path path)
{
    robot_coord.header.frame_id = "map";
    robot_coord.ns = "robot_coordinates";
    robot_coord.id = 0;
    robot_coord.type = visualization_msgs::Marker::CUBE;
    robot_coord.action = visualization_msgs::Marker::ADD;
    robot_coord.scale.x = 4.0;
    robot_coord.scale.y = 1.2;
    robot_coord.scale.z = 0.01;
    robot_coord.color.r = 0.0;
    robot_coord.color.g = 0.0;
    robot_coord.color.b = 1.0;
    robot_coord.color.a = 1.0;
    robot_coord.pose.position.z = 0;
    robot_coord.lifetime = ros::Duration();
    robot_coord.header.stamp = ros::Time::now();
    robot_coord.pose.position.x = start2_point.pose.position.x;
    robot_coord.pose.position.y = start2_point.pose.position.y;
    marker_pub.publish(robot_coord);
    ros::Duration(0.5).sleep();
    for (int i=0;i<path.poses.size();i++)
      {
          robot_coord.header.stamp = ros::Time::now();
          robot_coord.pose.position.x = path.poses[i].pose.position.x;
          robot_coord.pose.position.y = path.poses[i].pose.position.y;
          robot_coord.pose.orientation.x = path.poses[i].pose.orientation.x;
          robot_coord.pose.orientation.y = path.poses[i].pose.orientation.y;
          robot_coord.pose.orientation.z = path.poses[i].pose.orientation.z;
          robot_coord.pose.orientation.w = path.poses[i].pose.orientation.w;
          ROS_INFO("x_pos: %f",path.poses[i].pose.position.x);
          ROS_INFO("y_pos: %f",path.poses[i].pose.position.y);
          ROS_INFO("x: %f",path.poses[i].pose.orientation.x);
          ROS_INFO("y: %f",path.poses[i].pose.orientation.y);
          ROS_INFO("z: %f",path.poses[i].pose.orientation.z);
          ROS_INFO("w: %f",path.poses[i].pose.orientation.w);
          marker_pub.publish(robot_coord);
          ros::Duration(0.5).sleep();
      }
    }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "map_node");
  //ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);

    // create node handler
    ros::NodeHandle nodeHandle("~");
    map_node::Planner2D planner_(nodeHandle);

    // setup the ROS loop rate
    ros::Rate loop_rate(10);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    // occupancy map subscriber
    ros::Subscriber sub = nodeHandle.subscribe("/map", 1000, mapCallback);
    // start and goal point subscribers
    ros::Subscriber nav2_start = nodeHandle.subscribe("/start_point", 1000, start2Callback);
    ros::Subscriber nav2_goal = nodeHandle.subscribe("/end_point", 1000, goal2Callback);
    // robot coord publisher
    marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("robot_coordinates", 1000);

  while (ros::ok())
    {
    if ((start2_point.pose.position.x > 0.0 && start2_point.pose.position.y > 0.0) && (goal2_point.pose.position.x > 0.0 && goal2_point.pose.position.y > 0.0 ))
    {
        nav_msgs::Path plannedPath;
        // planning and measuring time
        start_planning = ros::WallTime::now();
        plannedPath = planner_.planPath(globalMap,start2_point,goal2_point);
        end_planning = ros::WallTime::now();
        double execution_time = (end_planning - start_planning).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
        // publish the planned path
        path_pub.publish(plannedPath);
        
        made_paths++;
        if (made_paths == iter_paths)
        {
          robot_move(plannedPath);
          break;
        }
    }
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
  
} // main


