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
visualization_msgs::Marker start_point;
visualization_msgs::Marker goal_point;

// robot coordinates
visualization_msgs::Marker robot_coord;

// time variables 
ros::WallTime start_planning, end_planning;

// variable for counting, how many times should the program plan path, if = 1 only one path will be made
int iter_paths = 1 ;
// variable for counting, how many times program planned path
int made_paths = 0;

// robot coord publisher
ros::Publisher marker_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  globalMap=*msg;
}

void startCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    start_point.pose.position.x = msg->pose.position.x;
    start_point.pose.position.y = msg->pose.position.y;
    start_point.pose.position.z = msg->pose.position.z;
    start_point.pose.orientation.x = msg->pose.orientation.x;
    start_point.pose.orientation.y = msg->pose.orientation.y;
    start_point.pose.orientation.z = msg->pose.orientation.z;
    start_point.pose.orientation.w = msg->pose.orientation.w;
}

void goalCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    goal_point.pose.position.x = msg->pose.position.x;
    goal_point.pose.position.y = msg->pose.position.y;
    goal_point.pose.position.z = msg->pose.position.z;
    goal_point.pose.orientation.x = msg->pose.orientation.x;
    goal_point.pose.orientation.y = msg->pose.orientation.y;
    goal_point.pose.orientation.z = msg->pose.orientation.z;
    goal_point.pose.orientation.w = msg->pose.orientation.w;
}

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
    robot_coord.pose.position.x = start_point.pose.position.x;
    robot_coord.pose.position.y = start_point.pose.position.y;
    robot_coord.pose.orientation.x = start_point.pose.orientation.x;
    robot_coord.pose.orientation.y = start_point.pose.orientation.y;
    robot_coord.pose.orientation.z = start_point.pose.orientation.z;
    robot_coord.pose.orientation.w = start_point.pose.orientation.w;
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

          marker_pub.publish(robot_coord);
          ros::Duration(0.5).sleep();
      }
    }


int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_node");

    ros::NodeHandle nodeHandle;

    ros::ServiceClient client = nodeHandle.serviceClient<pracownia_badawcza::LNP>("LNP",true);

    Planner2D plannerr(client);

    // setup the ROS loop rate
    ros::Rate loop_rate(10);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    // occupancy map subscriber
    ros::Subscriber sub = nodeHandle.subscribe("/map", 1000, mapCallback);

    // start and goal point subscribers
    ros::Subscriber nav_start = nodeHandle.subscribe("/start_point", 1000, startCallback);
    ros::Subscriber nav_goal = nodeHandle.subscribe("/end_point", 1000, goalCallback);

    // robot coord publisher
    marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("robot_coordinates", 1000);

  while (ros::ok())
    {
    if ((start_point.pose.position.x > 0.0 && start_point.pose.position.y > 0.0) && (goal_point.pose.position.x > 0.0 && goal_point.pose.position.y > 0.0 ))
    {
        nav_msgs::Path plannedPath;

        // planning and measuring time
        start_planning = ros::WallTime::now();
        plannedPath = plannerr.planPath(globalMap,start_point,goal_point);
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


