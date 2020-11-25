#ifndef POINTS_HPP
#define POINTS_HPP
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>

class Point{
    public:
        ros::Publisher marker_pub;
        ros::NodeHandle n;
        visualization_msgs::Marker marker;
        Point(int,int,std::string,float,float,float,float,float,float,float);
        void publish();
};
#endif 
