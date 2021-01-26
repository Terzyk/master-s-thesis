#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/MapMetaData.h>
#include "std_msgs/Header.h"
#include "../include/pracownia_badawcza/ompl_lib.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <math.h>
#include "../include/pracownia_badawcza/points.hpp"


double robot_width = 4.0;
double robot_height = 1.2;
double a,b,c,d,e,f; //  wspolczynniki wielomianu

double x0_in_s0,y0_in_s0,th0_in_s0,x1_in_s0,y1_in_s0,th1_in_s0;

double signn;
double x_wielomian,y_wielomian,tan_th_wielomian,th_wielomian;
double beta_defined = M_PI/3;
double curve = 0.0;
double max_curve = 0.0;

visualization_msgs::Marker start2_point;
visualization_msgs::Marker goal2_point;


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



struct factors
{
    double value1;
    double value2;
    double value3;
    double value4;
    double value5;
    double value6;
};






factors calculate_factors(double k_val_st0,double k_val_st1,double th0,double x0,double y0,double th1,double x1, double y1)
{
    // double g1,w,z;
    // double aa,bb,cc,dd,ee,ff;

    // ff = 0.0;
    // ee = tan(th0);
    // dd = 0.5*(k_val_st0 *  pow((1+pow(ee,2)),(1.5)));
    
    // z = y1 - dd*pow(x1,2) - ee*x1 - ff;
    // w = tan(th1) - ee - 2*dd*x1;
    // g1 = k_val_st1 * pow((1+pow(tan(th1),2)),(1.5)) - 2*dd;

    // aa = (12.0*z - 6.0*w*x1 + g1*x1*x1)/(2.0*pow(x1,5));
    // bb = (54.0*w*x1 - 120.0*z - 8.0*g1*x1*x1)/(8.0*pow(x1,4));
    // cc = (20.0*z + g1*x1*x1 - 8.0*w*x1)/(2.0*pow(x1,3));

    // najlepsze 

    // double aa,bb,cc,dd,ee,ff;
    // //double g0,g1,m,k,p,v,j,jv;

    // ff = y0;
    // ee = tan(th0);
    // dd = 0.0;
    // aa = ((6.0*y1+(-3.0)*x1*tan(th1))/pow(x1,5));
    // bb = (x1*tan(th1)+(-3.0)*y1+(-2.0)*aa*pow(x1,5))/(pow(x1,4));
    // //cc = -((-1.0*y1+aa*pow(x1,5)+bb*pow(x1,4))/(pow(x1,3)));
    // cc = (y1-aa*pow(x1,5)-bb*pow(x1,4))/(pow(x1,3));

    double aa,bb,cc,dd,ee,ff,w,z;
    //double g0,g1,m,k,p,v,j,jv;

    

    ff = y0;
    ee = tan(th0);
    dd = 0.0;
    w = y1 - ee*x1;
    z = tan(th1) - ee;
    aa = (4.0*w*z - x1*x1*z + 3*z*x1)/(4.0*pow(x1,5));
    cc = (4.0*w*z + 16.0*w - x1*x1*z - x1*z)/(4.0*x1*x1*x1);
    bb = (z - 5*aa*pow(x1,4) - 3.0*cc*x1*x1)/(4.0*x1*x1*x1);

    // aa = ((6.0*y1+(-3.0)*x1*tan(th1))/pow(x1,5));
    // bb = (x1*tan(th1)+(-3.0)*y1+(-2.0)*aa*pow(x1,5))/(pow(x1,4));
    // //cc = -((-1.0*y1+aa*pow(x1,5)+bb*pow(x1,4))/(pow(x1,3)));
    // cc = (y1-aa*pow(x1,5)-bb*pow(x1,4))/(pow(x1,3));


    factors result = {aa,bb,cc,dd,ee,ff};

    return result;
}

nav_msgs::Path calcPath(double x, double y, double theta)
{
    nav_msgs::Path plannedPath;

    if (abs(theta) >= M_PI/2)
    {
        ROS_INFO("------------------------- war1 ------------------------------");
    }
    if (x < 0)
    {
    signn= -1.0;
    }
    else signn= 1.0;

    for (double i = 0.0; i <= abs(x); i=i+0.1)
        {
            // tan_th_wielomian = 5*a*pow(x_wielomian,4)+4*b*pow(x_wielomian,3)+3*c*pow(x_wielomian,2),2*d*x_wielomian+e;
            // th_wielomian = atan(tan_th_wielomian);
            // th_wielomian_global = state0_coordYaw->values[0] + th_wielomian;

            tan_th_wielomian = tan(theta);
            th_wielomian = atan(tan_th_wielomian);

            x_wielomian = x0_in_s0+i*signn;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;

            max_curve = tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
            curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
            if (curve>max_curve)
            {
                ROS_INFO("---------------------------- war3 --------------------------------");
                break;
            }

        tf::Quaternion q_path;
        q_path.setRPY(0.0,0.0,th_wielomian);

        geometry_msgs::PoseStamped poseMsg;
        plannedPath.header.frame_id = "map";
        plannedPath.header.stamp = ros::Time::now();

        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.pose.position.x = x_wielomian;
        poseMsg.pose.position.y = y_wielomian;
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = q_path[3];
        poseMsg.pose.orientation.x = q_path[0];
        poseMsg.pose.orientation.y = q_path[1];
        poseMsg.pose.orientation.z = q_path[2];

        plannedPath.poses.push_back(poseMsg);
        ROS_INFO("x_wielomian: %f",x_wielomian);
        ROS_INFO("y_wielomian: %f", y_wielomian);
        ROS_INFO("th_wielomian: %f",th_wielomian);
        if (((x_wielomian >= x-0.1) && (x_wielomian <= x+0.1)) && ((y_wielomian >= y-0.1) && (y_wielomian <= y+0.1))) //&& (th_wielomian == th1_in_s0)) 
        {

            ROS_INFO("x_wielomian: %f",x_wielomian);
            ROS_INFO("y_wielomian: %f", y_wielomian);
            ROS_INFO("th_wielomian: %f",th_wielomian);
            ROS_INFO("goal_x: %f",x);
            ROS_INFO("goal_y: %f",y);
            ROS_INFO("yaw_goal: %f",theta);
            ROS_INFO("MAM KONIEC");
            //path_pub2.publish(plannedPath);
            break;
        }
        }
        return plannedPath;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "local_plan");
    ros::NodeHandle n;

    // setup the ROS loop rate
    ros::Rate loop_rate(10);
    
    // robot coord publisher
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("planned_path", 1000);

    x0_in_s0 = 250.0;
    y0_in_s0 = 53.0;
    double q_start_x = 0.0;
    double q_start_y = 0.0;
    // double q_start_z = -0.258819;
    // double q_start_w = 0.9659258;
    double q_start_z = 0.0;
    double q_start_w = 1.0;
    th0_in_s0 = 0.0;
    // th0_in_s0 = -0.5235988;

    x1_in_s0 = -320.0;
    y1_in_s0 = 80.0;
    double q_goal_x = 0.0;
    double q_goal_y = 0.0;
    double q_goal_z = -0.1305262;
    double q_goal_w = 0.9914449;
    // double q_goal_z = 0.0;
    // double q_goal_w = 1.0;
    th1_in_s0 = -0.2617994;
    // th1_in_s0 = 0.0;

    Point start_point(x0_in_s0,y0_in_s0,"start_point",0.0,1.0,0.0,q_start_x,q_start_y,q_start_z,q_start_w);
    Point end_point(x1_in_s0,y1_in_s0,"end_point",1.0,0.0,0.0,q_goal_x,q_goal_y,q_goal_z,q_goal_w);

  while (ros::ok())
    {
        start_point.publish();
        end_point.publish();

        double k_vall_st0 = 0.0;
        double k_vall_st1 = 0.0;
        factors res;

        x0_in_s0 = 0.0;
        y0_in_s0 = 0.0;
        th0_in_s0 = 0.0;

        res = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
        a = res.value1;
        b = res.value2;
        c = res.value3;
        d = res.value4;
        e = res.value5;
        f = res.value6;

        nav_msgs::Path plannedPath2 = calcPath(x1_in_s0,y1_in_s0,th1_in_s0);



        
        path_pub.publish(plannedPath2);
        ROS_INFO("len path: %d",sizeof(plannedPath2.poses));
        
        ros::spinOnce();
        loop_rate.sleep();
        std::cin.get();

    }

  return 0;
  
} // main


