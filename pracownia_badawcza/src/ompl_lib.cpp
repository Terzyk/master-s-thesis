#include "../include/pracownia_badawcza/ompl_lib.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <tf/tf.h>
#include <math.h>
#include <cmath>
#include <fstream>
#include <ostream>
#include "../include/pracownia_badawcza/points.hpp"
#include "ompl/base/MotionValidator.h"
#include "ompl/tools/benchmark/Benchmark.h"
# define M_PI           3.14159265358979323846  /* pi */
#include <boost/graph/adjacency_list.hpp>
#include "../include/pracownia_badawcza/planner.hpp"

#include <ompl/geometric/planners/prm/PRM.h>



using namespace std;
using namespace ros;

using Vertex = boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Point::Point(int x,int y,std::string name,float r,float g,float b,float roll,float pitch,float yaw,float w){
//     marker_pub = n.advertise<visualization_msgs::Marker>(name, 1000);
//     marker.header.frame_id = "map";
//     marker.header.stamp = ros::Time::now();
//     marker.ns = name;
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.position.x = x;
//     marker.pose.position.y = y;
//     marker.pose.position.z = 0;
//     marker.pose.orientation.x = roll;
//     marker.pose.orientation.y = pitch;
//     marker.pose.orientation.z = yaw;
//     marker.pose.orientation.w = w;
//     marker.scale.x=4.0;
//     marker.scale.y=1.2;
//     marker.scale.z=0.01;
//     marker.color.r = r;
//     marker.color.g = g;
//     marker.color.b = b;
//     marker.color.a = 1.0;
//     marker.lifetime = ros::Duration();
// }
// void Point::publish(){
//     marker_pub.publish(marker);
// }

int ix = 0;




// pracownia_badawcza::AddTwoInts *nn_srv;
// ros::ServiceClient *nn_client;


ros::ServiceClient *nn_client;

//og::LazyPRMstar* planner2;
// file for results
std::filebuf fb;
std::ostream os(&fb);
std::ofstream myfile2;
// global goal point
double goal_x;
double goal_y;
double goal_angle;
double roll_goal,pitch_goal,yaw_goal;
// global start point
double start_x;
double start_y;
double start_angle;
double roll_start,pitch_start,yaw_start;
// occupancy map used for planning
nav_msgs::OccupancyGrid occupancyMap;
// 2D problem
int dim = 2;
// robot size
double robot_width = 4.0;
double robot_height = 1.2;
// path length
int path_length;
// create bounds vector
ob::RealVectorBounds bounds(3);
// coordinates for creating space - sprobowac zamienic na jeden wektor se2statespace
auto coordX(std::make_shared<ob::RealVectorStateSpace>(dim-1));
auto coordY(std::make_shared<ob::RealVectorStateSpace>(dim-1));
auto coordYaw(std::make_shared<ob::RealVectorStateSpace>(dim-1));
// space
ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
// state property
boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::type my_stateProperty;

std::vector< double > top;
std::vector< double > bottom;
std::vector< double > dist;



double a,b,c,d,e,f; //  wspolczynniki wielomianu
struct factors
{
    double value1;
    double value2;
    double value3;
    double value4;
    double value5;
    double value6;
};



// class_cf::class_cf(ompl::geometric::LazyPRMstar& cf_planner)
// {
//     pb_planner = &cf_planner;
// }

// constructor in which we create our space
Planner2D::Planner2D(ros::ServiceClient& my_client)
{

    nn_client = &my_client;
    //nn_srv = &my_srv;
    // std::cout<<&my_client<<std::endl;
    // std::cout<<&nn_client<<std::endl;
    // std::cout<<*my_client<<std::endl;
    // std::cout<<*nn_client<<std::endl;

    // ros::ServiceClient nn_client = nodeHandle.serviceClient<pracownia_badawcza::AddTwoInts>("add_two_ints");
    // pracownia_badawcza::AddTwoInts nn_srv;
    
    
    for (double i=0.2; i<=robot_width ;i=i+0.2)
    {
        top.push_back(i);
    }
    top.push_back(4.0);
    for (double i=0.2; i <=robot_height ;i=i+0.2)
    {
        bottom.push_back(i);
    }

    // space bounds creation
	bounds.setLow(0,0);
	bounds.setHigh(0,512);
	bounds.setLow(1,0);
	bounds.setHigh(1,512);
    bounds.setLow(2,(-1.0*M_PI));
    bounds.setHigh(2,M_PI);
    coordX->setBounds(bounds.low[0],bounds.high[0]);
    coordY->setBounds(bounds.low[1],bounds.high[1]);
    coordYaw->setBounds(bounds.low[2],bounds.high[2]);
    space = coordX +coordY +coordYaw;
    ROS_INFO("Controlling UR node started.");
}

Planner2D::~Planner2D()
{
}



// not used so far
bool myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const
{
    std::cout<<"#################################################################";
    std::cout<<"TRUE"<<std::endl;
    ROS_INFO("JESTEM TUUUUUUUUU");
    return false;
}

// check if current state us valid -> checking only edges of the car
bool isStateValid(const ob::State *state)
{
    if ((occupancyMap.info.width>0)&&(occupancyMap.info.height>0)){
    // get x coord of the robot
    const ob::RealVectorStateSpace::StateType *coordX =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const ob::RealVectorStateSpace::StateType *coordY =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    // get yaw coord of the robot
    const ob::RealVectorStateSpace::StateType *coordYaw =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    int x_pos = (coordX->values[0]-occupancyMap.info.origin.position.x)/occupancyMap.info.resolution;
    int y_pos = (coordY->values[0]-occupancyMap.info.origin.position.y)/occupancyMap.info.resolution;
    int yaw = coordYaw->values[0];
    bool temp=true;

    double left_top_x_angled = x_pos + (robot_width/2.0)*cos(yaw);
    double left_top_y_angled = y_pos + (robot_height/2.0)*cos(yaw);
    double right_down_x_angled = x_pos - (robot_width/2.0)*cos(yaw);
    double right_down_y_angled = y_pos - (robot_height/2.0)*cos(yaw);

    int mapIndex = 0;
    int occupancyMapValue = 0;
    // sprawdzanie boku krotszego prostokata
    for (int i=0; i <= bottom.size(); i++) {
        mapIndex = (int)((left_top_y_angled+bottom[i]))*occupancyMap.info.width+(int)(left_top_x_angled);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100)
        {
            temp=true;
            break;
        }
        else temp=false;
        mapIndex = (int)((right_down_y_angled-bottom[i]))*occupancyMap.info.width+int(right_down_x_angled);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100)
        {
            temp=true;
            break;
        }
        else temp=false;
    }
    if(temp==true) return false;
    else return true;
    // sprawdzanie boku dluzszego prostokata
    for (int i=0; i <= top.size(); i++) {
        mapIndex = (int)(left_top_y_angled)*occupancyMap.info.width+(int)(left_top_x_angled-top[i]);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100){
            temp=true;
            break;
            }
            else temp=false;
        mapIndex = (int)(right_down_y_angled)*occupancyMap.info.width+(int)(right_down_x_angled+top[i]);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100){
            temp=true;
            break;
            }
            else temp=false;
    }
    if(temp==true) return false;
    else return true;
    }
} // end isStateValid

// motion validator -> checking motion
/*
bool myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state1_coordX = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state2_coordX = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state2_coordY = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state2_coordYaw = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    //ROS_INFO("###############################################################");
    //ob::State *check_state = space->allocState();
    //ob::ScopedState<ob::SE2StateSpace> scoped_check_state(space);
    int mapIndex = 0;
    int occupancyMapValue = 0;
    bool is_available = true;
    int part_line = 0; // how many parts of the line we want to check
    double part_distance = 2.0;
    double alfa = 0.0;
    int x_part = 0;
    int y_part = 0;
    //length between states
    double distance = std::sqrt(((state1_coordX->values[0] - state2_coordX->values[0])*(state1_coordX->values[0] - state2_coordX->values[0])) + ((state1_coordY->values[0] - state2_coordY->values[0])*(state1_coordY->values[0] - state2_coordY->values[0])));
    part_line = (int)(ceil(distance/part_distance)); // not sure if (int) rounds up
    ROS_INFO("part_line: %d",part_line);
    ROS_INFO("distance: %f",distance);
    for (int i=1; i<=part_line; i=i+1)
    {
        if ((state1_coordX->values[0] >= state2_coordX->values[0]) && (state1_coordY->values[0] >= state2_coordY->values[0]))
        {
            ROS_INFO("1");
            alfa = atan2((state1_coordY->values[0]-state2_coordY->values[0]),(state1_coordX->values[0]-state2_coordX->values[0]));
            x_part = cos(alfa)*i*part_distance;
            y_part = sin(alfa)*i*part_distance;
            mapIndex = (int)(state2_coordY->values[0]+y_part)*occupancyMap.info.width +(int)(x_part+state2_coordX->values[0]);
            occupancyMapValue = occupancyMap.data[mapIndex];
            ROS_INFO("mapIndex1: %d",mapIndex);
            ROS_INFO("occupancyMapValue1: %d",occupancyMapValue);
            if (occupancyMapValue == 100)
            {
                is_available=false;
                break;
            }
            else is_available=true;
        }
        if ((state1_coordX->values[0] >= state2_coordX->values[0]) && (state1_coordY->values[0] < state2_coordY->values[0]))
        {
            ROS_INFO("2");
            alfa = atan2((state2_coordY->values[0]-state1_coordY->values[0]),(state1_coordX->values[0]-state2_coordX->values[0]));
            x_part = (int)(cos(alfa)*i*part_distance);
            y_part = (int)(sin(alfa)*i*part_distance);
            mapIndex = (int)(state1_coordY->values[0]+y_part)*occupancyMap.info.width +(int)(state1_coordX->values[0]-x_part);
            occupancyMapValue = occupancyMap.data[mapIndex];
            ROS_INFO("mapIndex2: %d",mapIndex);
            ROS_INFO("occupancyMapValue2: %d",occupancyMapValue);
            if (occupancyMapValue == 100)
            {
                is_available=false;
                break;
            }
            else is_available=true;
        }
        if ((state1_coordX->values[0] < state2_coordX->values[0]) && (state1_coordY->values[0] >= state2_coordY->values[0]))
        {
            ROS_INFO("3");
            alfa = atan2((state1_coordY->values[0]-state2_coordY->values[0]),(state2_coordX->values[0]-state1_coordX->values[0]));
            x_part = (int)(cos(alfa)*i*part_distance);
            y_part = (int)(sin(alfa)*i*part_distance);
            mapIndex = (int)(state2_coordY->values[0]+y_part)*occupancyMap.info.width +(int)(state2_coordX->values[0]-x_part);
            occupancyMapValue = occupancyMap.data[mapIndex];
            ROS_INFO("mapIndex3: %d",mapIndex);
            ROS_INFO("occupancyMapValue3: %d",occupancyMapValue);
            if (occupancyMapValue == 100)
            {
                is_available=false;
                break;
            }
            else is_available=true;
        }
        if ((state1_coordX->values[0] < state2_coordX->values[0]) && (state1_coordY->values[0] < state2_coordY->values[0]))
        {
            ROS_INFO("4");
            //ROS_INFO("state2_coordY->values[0]: %f",state2_coordY->values[0]);
            //ROS_INFO("state1_coordY->values[0]: %f",state1_coordY->values[0]);
            //ROS_INFO("state2_coordX->values[0]: %f",state2_coordX->values[0]);
            //ROS_INFO("state1_coordX->values[0]: %f",state1_coordX->values[0]);
            alfa = atan2((state2_coordY->values[0]-state1_coordY->values[0]),(state2_coordX->values[0]-state1_coordX->values[0]));
            ROS_INFO("angle: %f",alfa);
            x_part = (int)(cos(alfa)*i*part_distance);
            y_part = (int)(sin(alfa)*i*part_distance);
            mapIndex = (int)(state1_coordY->values[0]+y_part)*occupancyMap.info.width +(int)(x_part+state1_coordX->values[0]);
            occupancyMapValue = occupancyMap.data[mapIndex];
            ROS_INFO("mapIndex4: %d",mapIndex);
            ROS_INFO("occupancyMapValue4: %d",occupancyMapValue);
            if (occupancyMapValue == 100)
            {
                is_available=false;
                break;
            }
            else is_available=true;
        }
    }
    if (is_available) return true;
    else return false;
}
*/
factors calculate_factors(double k_val_st0,double k_val_st1,double th0,double x0,double y0,double th1,double x1, double y1)
{
    // double g1,w,z;
    // double aa,bb,cc,dd,ee,ff;
    // //kval dac zero

    
    // ff = 0.0;
    // ee = tan(th0);
    // dd = 0.5*(k_val_st0 *  pow((1+pow(ee,2)),(1.5)));
    
    // z = y1 - dd*pow(x1,2) - ee*x1 - ff;
    // w = tan(th1) - ee - 2*dd*x1;
    // g1 = k_val_st1 * pow((1+pow(tan(th1),2)),(1.5)) - 2*dd;

    // aa = (12.0*z - 6.0*w*x1 + g1*x1*x1)/(2.0*pow(x1,5));
    // bb = (54.0*w*x1 - 120.0*z - 8.0*g1*x1*x1)/(8.0*pow(x1,4));
    // cc = (20.0*z + g1*x1*x1 - 8.0*w*x1)/(2.0*pow(x1,3));

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


    // g0=k_val*(pow((1+tan(th0)*tan(th0)),(3/2)));
    // g1=k_val*(pow((1+tan(th1)*tan(th1)),(3/2)));
    // m = (g0/2)*pow(x1,2) + tan(th0)*x1 - y1;
    // k = g0*x1 + tan(th0) - tan(th1);
    // p = g0 - g1;
    // v = 6.0*m+pow(x1,2)*p;
    // j = 2.0*k + x1*p;
    // jv = j*(-0.9)*x1+v;

    // ff = y0;
    // ee = tan(th0);
    // dd = g0/2;
    // aa = (jv)/(pow(x1,5));
    // bb = ((-1.5)*(1/pow(x1,4))*jv + (j/((-1)*20*pow(x1,3))));
    // cc = (-1)*((p+a*20.0*pow(x1,3)+b*12.0*pow(x1,2))/(6*x1));

    /*



    
    ff = y0;
    ee = tan(th0);
    dd = 0.0;
    aa = (-3.0*tan(th1))/(pow(x1,4));
    bb = (7.0*tan(th1))/(pow(x1,3));
    cc = (-4.0*tan(th1))/(pow(x1,2));

    ff = y0;
    ee = tan(th0);
    dd = 0.0;
    aa = ((6.0*y1+(-3.0)*x1*tan(th1))/pow(x1,5));
    bb = (x1*tan(th1)+(-3.0)*y1+(-2.0)*aa*pow(x1,5))/(pow(x1,4));
    //cc = -((-1.0*y1+aa*pow(x1,5)+bb*pow(x1,4))/(pow(x1,3)));
    cc = (y1-aa*pow(x1,5)-bb*pow(x1,4))/(pow(x1,3));
*/
    // ff = y0;
    // ee = tan(th0);
    // dd = 0.0;
    // bb = (7.0*x1*tan(th1)+(-15.0)*y1)/(pow(x1,4));
    // aa = ((6.0*y1+6.0*pow(x1,4)*bb)/((-14.0)*pow(x1,5)));
    // //cc = -((-1.0*y1+aa*pow(x1,5)+bb*pow(x1,4))/(pow(x1,3)));
    // cc = ((-20.0*aa*pow(x1,2)+(-12.0)*bb*x1)/6.0);


    ROS_INFO("aa: %.10f",aa);
    ROS_INFO("bb: %.10f",bb);
    ROS_INFO("cc: %.10f",cc);
    ROS_INFO("dd: %f",dd);
    ROS_INFO("ee: %f",ee);
    ROS_INFO("ff: %f",ff);

    factors result = {aa,bb,cc,dd,ee,ff};

    return result;
}

// bool class_cf::operator() (const Vertex &vertex1, const Vertex &vertex2)
// {

//     boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::type my_stateProperty;
//     const ob::State *s1 = planner2->getStateProperty(vertex1);


//     return true;
// }



bool class_cf::cf_client(const ob::State *state1, const ob::State *state2)
{
    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state1_coordX = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state2_coordX = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state2_coordY = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state2_coordYaw = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    pracownia_badawcza::AddTwoInts nn_srv;
    nn_srv.request.s1_x = state1_coordX->values[0];
    nn_srv.request.s1_y = state1_coordY->values[0];
    nn_srv.request.s1_yaw = state1_coordYaw->values[0];
    nn_srv.request.s2_x = state2_coordX->values[0];
    nn_srv.request.s2_y = state2_coordY->values[0];
    nn_srv.request.s2_yaw = state2_coordYaw->values[0];

    // ROS_INFO("client request: s1_x: %f", nn_srv.request.s1_x);
    // ROS_INFO("client request: s1_y: %f", nn_srv.request.s1_y);
    // ROS_INFO("client request: s1_yaw: %f", nn_srv.request.s1_yaw);
    // ROS_INFO("client request: s2_x: %f", nn_srv.request.s2_x);
    // ROS_INFO("client request: s2_y: %f", nn_srv.request.s2_y);
    // ROS_INFO("client request: s2_yaw: %f", nn_srv.request.s2_yaw);

    if ((*nn_client).call(nn_srv))
    {
        ROS_INFO("SERVER RESPONSE:");
        // ROS_INFO("server response: s1_x: %f", (float)nn_srv.response.factors[0]);
        // ROS_INFO("server response: s1_y: %f", (float)nn_srv.response.factors[1]);
        // ROS_INFO("server response: s1_yaw: %f", (float)nn_srv.response.factors[2]);
        // ROS_INFO("server response: s2_x: %f", (float)nn_srv.response.factors[3]);
        // ROS_INFO("server response: s2_y: %f", (float)nn_srv.response.factors[4]);
        // ROS_INFO("server response: s2_yaw: %f", (float)nn_srv.response.factors[5]);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
    }

    // std::cin.get();
    return true;
}




bool myMotionValidator::checkMotion(const ob::State *s0, const ob::State *s1) const
{

    a,b,c,d,e,f = 0.0;
    double beta_defined = M_PI/3;
    double curve = 0.0;
    double x_wielomian_global = 0.0;
    double y_wielomian_global = 0.0;
    double th_wielomian_global = 0.0;
    double x_wielomian = 0.0;
    double y_wielomian = 0.0;
    double th_wielomian = 0.0;
    double tan_th_wielomian = 0.0;
    double max_curve = 0.0;
    double x1_in_s0,y1_in_s0,th1_in_s0;

    int mapIndex = 0;
    int occupancyMapValue = 0;
    double signn = 1.0;
    double sign_x,sign_y,sign_yaw;
    bool is_available = true;
    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state1_coordX = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state0_coordX = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state0_coordY = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state0_coordYaw = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // ROS_INFO("#######################");
    // ROS_INFO("Motion Validator dla stanow:");
    // ROS_INFO("stan0 X: %f",state0_coordX->values[0]);
    // ROS_INFO("stan0 Y: %f",state0_coordY->values[0]);
    // ROS_INFO("stan0 Yaw %f",state0_coordYaw->values[0]);
    // ROS_INFO("stan1 X: %f",state1_coordX->values[0]);
    // ROS_INFO("stan1 Y: %f",state1_coordY->values[0]);
    // ROS_INFO("stan1 Yaw: %f",state1_coordYaw->values[0]);
    // ROS_INFO("######################");
    // ROS_INFO("WSPOLRZEDNE PUNKTU PIERWSZEGO W UKLADZIE 0");

    // wyrazenie punktu 1 w ukladzie punktu 0
    //th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0]; // na pewno 0 - 1
    // double x1_in_s0 = state1_coordX->values[0]*cos(th1_in_s0) - state1_coordY->values[0]*sin(th1_in_s0);
    // double y1_in_s0 = state1_coordX->values[0]*sin(th1_in_s0) + state1_coordY->values[0]*cos(th1_in_s0);

    th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
    y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);


    //basic 1
        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("1");
        //     // if (state1_coordYaw->values[0] >= state0_coordYaw->values[0]) th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     // else th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        //     y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
        // }
        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("4");
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (-1.0)*((state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0));
        //     y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (-1.0)*((state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0));
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("2");
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0)) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        //     y1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0)) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("3"); //ogarnac temat katow
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0));
        //     y1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0));
        // }

    // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
    // {
    //     ROS_INFO("1");
    //     sign_x = 1.0;
    //     sign_y = 1.0;
    // }
    // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
    // {
    //     ROS_INFO("4");
    //     sign_x = 1.0;
    //     sign_y = -1.0;
    // }
    // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
    // {
    //     ROS_INFO("2");
    //     sign_x = -1.0;
    //     sign_y = 1.0;
    // }
    // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
    // {
    //     ROS_INFO("3");
    //     sign_x = -1.0;
    //     sign_y = -1.0;
    // }


    // uklad 0
    double x0_in_s0 = 0.0;
    double y0_in_s0 = 0.0;
    double th0_in_s0 = 0.0;

    // //zalozenie ze orientacja punktu 1 w lokalnym ukladzie jest co do bezwglednej wartosci mniejsza niz pi/2
     if (abs(th1_in_s0) >= M_PI/2)
     {
        ROS_INFO("warunek nr 1");
        return false;
     }
         


    // ROS_INFO("th1_in_s0: %f",th1_in_s0);
    // ROS_INFO("x1_in_s0: %f",x1_in_s0);
    // ROS_INFO("y1_in_s0: %f",y1_in_s0);

  
    double k_vall_st0 = 0.0;
    double k_vall_st1 = 0.0;
    factors res;
    res = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
    a = res.value1;
    b = res.value2;
    c = res.value3;
    d = res.value4;
    e = res.value5;
    f = res.value6;


    // // sprawdzenie znaku 
    if (x1_in_s0 < 0)
    {
        signn= -1.0;
        // return false;
    } 
    else signn= 1.0;

    // // wartosc funkcji wielomianowej licze dla wspolrzednych ukladu lokalnego  i zamieniam spowrotem na uklad globalny
    for (double i = 0.0; i <= abs(x1_in_s0); i=i+0.1)
    {
            tan_th_wielomian = tan(th1_in_s0);
            th_wielomian = atan(tan_th_wielomian);
            th_wielomian_global = th_wielomian;//state0_coordYaw->values[0] - th_wielomian;//state0_coordYaw->values[0] + th_wielomian;

            //x_wielomian = (x0_in_s0+i)*signn;
            x_wielomian = x0_in_s0+i*signn;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
            // zamiana na wartosci globalne
            // x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos((-1.0)*th_wielomian_global) - y_wielomian*sin((-1.0)*th_wielomian_global);
            // y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos((-1.0)*th_wielomian_global);

            x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos(th_wielomian_global) + y_wielomian*sin(th_wielomian_global);
            y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos(th_wielomian_global);


            // x_wielomian_global = state0_coordX->values[0] + sign_x*(x_wielomian*cos(th1_in_s0) + y_wielomian*sin(th1_in_s0));
            // y_wielomian_global = state0_coordY->values[0] + sign_y*(x_wielomian*(-1.0)*sin(th1_in_s0) + y_wielomian*cos(th1_in_s0));


            // tan_th_wielomian = 5*a*pow(x_wielomian,4)+4*b*pow(x_wielomian,3)+3*c*pow(x_wielomian,2),2*d*x_wielomian+e;
            // th_wielomian = atan(tan_th_wielomian);
            // th_wielomian_global = th1_in_s0 + th_wielomian;
        
            ROS_INFO("x_wielomian: %f",x_wielomian_global);
            ROS_INFO("y_wielomian: %f", y_wielomian_global);
            ROS_INFO("th_wielomian_global: %f",th_wielomian_global);
            ROS_INFO("th_wielomian: %f",th_wielomian);            
            ROS_INFO("state0_coordYaw->values[0]: %f",state0_coordYaw->values[0]);
            ROS_INFO("state1_coordYaw->values[0]: %f",state1_coordYaw->values[0]);



        if ((x_wielomian_global >= occupancyMap.info.width || x_wielomian_global <= 0.0)||(y_wielomian_global >= occupancyMap.info.height || y_wielomian_global <= 0.0))
        { 
            is_available=false;
            ROS_INFO("warunek nr 2");
            // ROS_INFO("y_wielomian_global: %f",y_wielomian_global);
            // ROS_INFO("x_wielomian_global: %f",x_wielomian_global);
            break;
        }
        else 
        {
            is_available=true;
        }

        // ROS_INFO("x_wielomian: %f",x_wielomian);
        // ROS_INFO("y_wielomian: %f",y_wielomian);
        // ROS_INFO("th1_in_s0: %f",th1_in_s0);

        // ROS_INFO("x_wielomian_global: %f",x_wielomian_global);
        // ROS_INFO("y_wielomian_global: %f",y_wielomian_global);

        mapIndex = (int)(y_wielomian_global)*occupancyMap.info.width +(int)(x_wielomian_global);
        occupancyMapValue = occupancyMap.data[mapIndex];
        // ROS_INFO("mapIndex: %d",mapIndex);
        //ROS_INFO("occupancyMapValue: %d",occupancyMapValue);
         //if (occupancyMapValue == 100)
        // if ((occupancyMapValue > 0) || (occupancyMapValue < 0))
        //     {
        //         is_available=false;
        //         //ROS_INFO("warunek nr 3");
        //         break;
        //     }
        // else 
        //     {
        //         is_available=true;
        //     }
        
        if (occupancyMapValue == 0)
            {
                is_available=true;
                ROS_INFO("warunek nr 3");
            }
        else 
            {
                is_available=false;
                break;
            }


        // maksymalna krzywizna:
        max_curve = tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
        // L=1.2
        // dodalem abs
        //curve = (20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
        curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
        // ROS_INFO("curve: %f",curve);
        // ROS_INFO("max_curve: %f",max_curve);
        if (curve<=max_curve) is_available=true;
        else 
        {
            is_available=false;
            //ROS_INFO("warunek nr 4");
            break;
        }

        // zapis danych do pliku
        // myfile2.open("dane_global9.txt",ofstream::app);
        // myfile2 << x_wielomian_global <<","<<y_wielomian_global<<","<<th_wielomian_global<<std::endl;
        // myfile2.close();

    }


    // bool is_available = false;
   
    // if (ix==0) {
    //      ROS_INFO("BBB %d",ix);
    //     ix=ix+1;
    //     is_available = false;
    // }
    // else {
    // ROS_INFO("AIII");
    // is_available = true;
    // }
    // ROS_INFO("is_available %d",is_available);


    //is_available = true;
    ix = 1;
    if (ix==0) 
    {   
        ROS_INFO("First loop");
        ix = 1;
        is_available = true;
        return false;
    }
    else 
    {
    //ROS_INFO("Rest of the loop");
    if (is_available)
    {
        ROS_INFO("Motion Validator: True");
        return true;
    }
    else 
    {
        ROS_INFO("Motion Validator: False");
        return false;
    }
    }

    
    //return true;
     
}

// void myPlannerData::getPlannerData(ob::PlannerData &data)
// {
//     ROS_INFO("plannerdata");
// }



// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){
    double th1_in_s0,x1_in_s0,y1_in_s0;
    double th0_in_s0,x0_in_s0,y0_in_s0;
    double signn;
    double x_wielomian,y_wielomian,x_wielomian_global,y_wielomian_global,tan_th_wielomian,th_wielomian,th_wielomian_global;
    double beta_defined = M_PI/3;
    double curve = 0.0;
    double max_curve = 0.0;
    double sign_x,sign_y;

    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "map";
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
    path->print(std::cout);
    
    // convert to geometric path
    og::PathGeometric *path2 = path.get()->as<og::PathGeometric>();
    //get path length
    path_length = path2->getStateCount();
    // iterate over each position
    ROS_INFO("path_len: %d",path_length);
    ROS_INFO("width: %d",occupancyMap.info.width);
    ROS_INFO("height: %d",occupancyMap.info.height);

    //std::cin.get();
    for(int i=0; i<path_length-1; ++i)
    {

        // get states 
        ob::State *state0 = path2->getState(i);
        ob::State *state1 = path2->getState(i+1);
        ob::RealVectorStateSpace::StateType *state0_coordX = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType *state0_coordY = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        ob::RealVectorStateSpace::StateType *state0_coordYaw = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
        ob::RealVectorStateSpace::StateType *state1_coordX = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType *state1_coordY = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        ob::RealVectorStateSpace::StateType *state1_coordYaw = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

        // // ROS_INFO("extract path dla stanow:");
        // // ROS_INFO("stan0 X: %f",state0_coordX->values[0]);
        // // ROS_INFO("stan0 Y: %f",state0_coordY->values[0]);
        // // ROS_INFO("stan0 Yaw %f",state0_coordYaw->values[0]);
        // // ROS_INFO("stan1 X: %f",state1_coordX->values[0]);
        // // ROS_INFO("stan1 Y: %f",state1_coordY->values[0]);
        // // ROS_INFO("stan1 Yaw: %f",state1_coordYaw->values[0]);


        // // // ROS_INFO("OTRZYMANE WSPOLRZEDNE 2 STANOW");
        // // // ROS_INFO(" ");
        // // // ROS_INFO("state0_coordX->values[0]: %f",state0_coordX->values[0]);
        // // // ROS_INFO("state0_coordY->values[0]: %f",state0_coordY->values[0]);
        // // // ROS_INFO("state0_coordYaw->values[0]: %f",state0_coordYaw->values[0]);
        // // // ROS_INFO("state1_coordX->values[0]: %f",state1_coordX->values[0]);
        // // // ROS_INFO("state1_coordY->values[0]: %f",state1_coordY->values[0]);
        // // // ROS_INFO("state1_coordYaw->values[0]: %f",state1_coordYaw->values[0]);
        // // // ROS_INFO("######################");

        // // // // punkt 0
        th0_in_s0 = 0.0;
        x0_in_s0 = 0.0;
        y0_in_s0 = 0.0;

        // // // // // wyrazenie punktu 1 w ukladzie punktu 0
        // th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0]; // na pewno 0 - 1
        // x1_in_s0 = state1_coordX->values[0]*cos(th1_in_s0) - state1_coordY->values[0]*sin(th1_in_s0);
        // y1_in_s0 = state1_coordX->values[0]*sin(th1_in_s0) + state1_coordY->values[0]*cos(th1_in_s0);
        // th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0]; // na pewno 0 - 1
        // basic
        ROS_INFO("------------------------- NEW POINTS ------------------------------");
        th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);

        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("1");
        //     // if (state1_coordYaw->values[0] >= state0_coordYaw->values[0]) th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     // else th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        //     y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
        // }
        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("4");
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (-1.0)*((state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0));
        //     y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (-1.0)*((state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0));
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("2");
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0)) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        //     y1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0)) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("3"); //ogarnac temat katow
        //     th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        //     x1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0));
        //     y1_in_s0 = (-1.0)*((state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0));
        // }
        // x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        // y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);

        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("1");
        //     sign_x = 1.0;
        //     sign_y = 1.0;
        // }
        // if ((state1_coordX->values[0] >= state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("4");
        //     sign_x = 1.0;
        //     sign_y = -1.0;
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] >= state0_coordY->values[0]))
        // {
        //     ROS_INFO("2");
        //     sign_x = -1.0;
        //     sign_y = 1.0;
        // }
        // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
        // {
        //     ROS_INFO("3");
        //     sign_x = -1.0;
        //     sign_y = -1.0;
        // }

        // // ROS_INFO("1: %f",state1_coordX->values[0]*cos(th1_in_s0));
        // // ROS_INFO("2: %f",state1_coordY->values[0]*sin(th1_in_s0));
        // // ROS_INFO("3 %f",state1_coordX->values[0]*sin(th1_in_s0));
        // // ROS_INFO("4 %f",state1_coordY->values[0]*cos(th1_in_s0));

        // ROS_INFO("#############");
        // ROS_INFO("th1_in_s0: %f",th1_in_s0);
        // ROS_INFO("x1_in_s0: %f",x1_in_s0);
        // ROS_INFO("y1_in_s0 %f",y1_in_s0);



        double k_vall_st0 = 0.0;
        double k_vall_st1 = 0.0;
        factors res;
        res = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
        a = res.value1;
        b = res.value2;
        c = res.value3;
        d = res.value4;
        e = res.value5;
        f = res.value6;


        if (abs(th1_in_s0) >= M_PI/2)
        {
            ROS_INFO("------------------------- war1 ------------------------------");
            continue;
        }

        if (x1_in_s0 < 0)
        {
            signn= -1.0;
            //continue;
        }
        else signn= 1.0;

        // // // ROS_INFO("WSPOLRZEDNE PUNKTU PIERWSZEGO W UKLADZIE 0");
        ROS_INFO("state1_coordYaw->values[0]: %f",state1_coordYaw->values[0]);
        ROS_INFO("state0_coordYaw->values[0]: %f",state0_coordYaw->values[0]);
        // ROS_INFO("y1_in_s0: %f",y1_in_s0);
        
        // // // ROS_INFO("######################");

        for (double i = 0.1; i <= abs(x1_in_s0); i=i+0.5)
        {
            x_wielomian = x0_in_s0+i*signn;

            tan_th_wielomian = tan(th1_in_s0);
            th_wielomian = atan(tan_th_wielomian);
            th_wielomian_global = th_wielomian;//state0_coordYaw->values[0] - th_wielomian;

            // tan_th_wielomian = tan(th1_in_s0);
            // th_wielomian = atan(tan_th_wielomian);
            // th_wielomian_global = state0_coordYaw->values[0] + th_wielomian;

            //x_wielomian = (x0_in_s0+i)*signn;

            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
            // zamiana na wartosci globalne
            // x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos((-1.0)*th_wielomian_global) - y_wielomian*sin((-1.0)*th_wielomian_global);
            // y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos((-1.0)*th_wielomian_global);

            x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos(th_wielomian_global) + y_wielomian*sin(th_wielomian_global);
            y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos(th_wielomian_global);

            ROS_INFO("x_wielomian: %f",x_wielomian_global);
            ROS_INFO("y_wielomian: %f", y_wielomian_global);
            ROS_INFO("th_wielomian_global: %f",th_wielomian_global);
            ROS_INFO("th_wielomian: %f",th_wielomian);            
            ROS_INFO("th1_ins0: %f",th1_in_s0);
            // ROS_INFO("state1_coordYaw->values[0]: %f",state1_coordYaw->values[0]);
    


            // x_wielomian_global = state0_coordX->values[0] + sign_x*(x_wielomian*cos(th1_in_s0) + y_wielomian*sin(th1_in_s0));
            // y_wielomian_global = state0_coordY->values[0] + sign_y*(x_wielomian*(-1.0)*sin(th1_in_s0) + y_wielomian*cos(th1_in_s0));


            // tan_th_wielomian = 5*a*pow(x_wielomian,4)+4*b*pow(x_wielomian,3)+3*c*pow(x_wielomian,2),2*d*x_wielomian+e;
            // th_wielomian = atan(tan_th_wielomian);
            // th_wielomian_global = th1_in_s0 + th_wielomian;
        
            // ROS_INFO("x_wielomian: %f",x_wielomian_global);
            // ROS_INFO("y_wielomian: %f", y_wielomian_global);
            // ROS_INFO("th_wielomian_global: %f",th_wielomian_global);
            //ROS_INFO("th1_in_s0: %f",th1_in_s0);
        //     //std::cin.get();

            if ((x_wielomian_global >= occupancyMap.info.width || x_wielomian_global <= 0.0)||(y_wielomian_global >= occupancyMap.info.height || y_wielomian_global <= 0.0)) {
                ROS_INFO("------------------------------ war2 -----------------------------------");
                break;
            }

            max_curve = tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
            curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
            if (curve>max_curve)
            {
                ROS_INFO("---------------------------- war3 --------------------------------");
                break;
            }

        // ROS_INFO("goal_x: %f",goal_x);
        // ROS_INFO("goal_y: %f",goal_y);
        // ROS_INFO("goal_yaw: %f",yaw_goal);

        //ob::RealVectorStateSpace::StateType *coordX = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        //ob::RealVectorStateSpace::StateType *coordY = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        //ob::RealVectorStateSpace::StateType *coordYaw = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
        // make quaternion
        tf::Quaternion q_path;
        q_path.setRPY(0.0,0.0,th_wielomian_global);
        // q_path.setRPY(0.0,0.0,state0_coordYaw->values[0]);
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = x_wielomian_global;
        poseMsg.pose.position.y = y_wielomian_global;
        // poseMsg.pose.position.x = state0_coordX->values[0];
        // poseMsg.pose.position.y = state0_coordY->values[0];
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = q_path[3];
        poseMsg.pose.orientation.x = q_path[0];
        poseMsg.pose.orientation.y = q_path[1];
        poseMsg.pose.orientation.z = q_path[2];
        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = ros::Time::now();
        plannedPath.poses.push_back(poseMsg);
        ROS_INFO("goal_x: %f",goal_x);
        ROS_INFO("goal_y: %f",goal_y);
        ROS_INFO("yaw_goal: %f",yaw_goal);

        //Point st_point(state0_coordX->values[0],state0_coordY->values[0],"st_point",1.0,1.0,0.2,q_path[0],q_path[1],q_path[2],q_path[3]);
        //st_point.publish();
        
        if (((x_wielomian_global <= goal_x + 0.1) && (x_wielomian_global >= goal_x - 0.1)) && ((y_wielomian_global <= goal_y + 0.1) && (y_wielomian_global >= goal_y - 0.1)) && ((th_wielomian_global <= yaw_goal + 0.05) && (th_wielomian_global >= yaw_goal - 0.05))) 
        // if (((state0_coordX->values[0] <= goal_x + 0.1) && (state0_coordX->values[0] >= goal_x - 0.1)) && ((state0_coordY->values[0] <= goal_y + 0.1) && (state0_coordY->values[0] >= goal_y - 0.1)) && ((state0_coordYaw->values[0] <= yaw_goal + 0.05) && (state0_coordYaw->values[0] >= yaw_goal - 0.05))) 
        {
            ROS_INFO("MAM KONIEC");
            break;
        }
        }
    }
    return plannedPath;
}




/*
// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){

    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
    path->print(std::cout);
    // convert to geometric path
    og::PathGeometric *path2 = path.get()->as<og::PathGeometric>();
    //get path length
    path_length = path2->getStateCount();
    // iterate over each position
    for(int i=0; i<path_length; ++i){
        // get state
        ob::State *state = path2->getState(i);
        // get x coord of the robot
        //double coordX = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        ob::RealVectorStateSpace::StateType *coordX =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        //double coordY = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        ob::RealVectorStateSpace::StateType *coordY =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        // get yaw coord of the robot
        ob::RealVectorStateSpace::StateType *coordYaw =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
        // make quaternion
        tf::Quaternion q_path;
        q_path.setRPY(0.0,0.0,coordYaw->values[0]);
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[0];
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = q_path[3];
        poseMsg.pose.orientation.x = q_path[0];
        poseMsg.pose.orientation.y = q_path[1];
        poseMsg.pose.orientation.z = q_path[2];
        poseMsg.header.frame_id = "/map";
        poseMsg.header.stamp = ros::Time::now();
        plannedPath.poses.push_back(poseMsg);

        if (coordX->values[0] == goal_x && coordY->values[0] == goal_y)
        {
            break;
        }
    }
    return plannedPath;
}
*/


// plan path
nav_msgs::Path Planner2D::planPath(const nav_msgs::OccupancyGrid& globalMap,const visualization_msgs::Marker &st_pt,const visualization_msgs::Marker &gl_pt)
{

    // nn_client = &my_client;
    // nn_srv = &my_srv;
    // nn_client = &my_client;
    // nn_srv = &my_srv;
    // std::cout<<&my_client<<std::endl;
    // std::cout<<&nn_client<<std::endl;

    // my_srv.request.a = 5;
    // my_srv.request.b = 3;
    
    // if (my_client.call(my_srv))
    // {
    //     ROS_INFO("Sum: %ld", (long int)my_srv.response.sum);
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service add_two_ints");
    // }
    // std::cin.get();

    occupancyMap = globalMap;
    goal_x = gl_pt.pose.position.x;
    goal_y = gl_pt.pose.position.y;
    start_x = st_pt.pose.position.x;
    start_y = st_pt.pose.position.y; 
    // goal quaternion
    tf::Quaternion q_goal(gl_pt.pose.orientation.x,gl_pt.pose.orientation.y,gl_pt.pose.orientation.z,gl_pt.pose.orientation.w);
    tf::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
    // start quaternion
    tf::Quaternion q_start(st_pt.pose.orientation.x,st_pt.pose.orientation.y,st_pt.pose.orientation.z,st_pt.pose.orientation.w);
    tf::Matrix3x3(q_start).getRPY(roll_start, pitch_start, yaw_start);
    // Create an instance of ompl::base::SpaceInformation for the state space 
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    // Set the state validity checker 
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    //si->setStateValidityCheckingResolution(0.5);
    // Set the motion validator
    ob::MotionValidatorPtr mv(new myMotionValidator(si));
    si->setMotionValidator(mv);
    si->setup();


    // create start and goal states
    std::vector<double> start_vector = {start_x,start_y,yaw_start};
    std::vector<double> goal_vector = {goal_x,goal_y,yaw_goal};
    ob::ScopedState<>start(space);
    ob::ScopedState<>goal(space);

    start=start_vector;
    goal=goal_vector;

    start.print(std::cout);
    goal.print(std::cout);
    // Create an instance of ompl::base::ProblemDefinition
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // Set the start and goal states for the problem definition. 
    pdef->setStartAndGoalStates(start, goal);
    // Create an instance of a planner 
    my_planner* planner2 = new my_planner(si);
    //planner2 = new my_planner(si);
    //planner2->setConnectionStrategy();
    planner2->setRange(10.0);

    //my_stateProperty = planner2->getStateProperty();

    class_cf cc(planner2);
    std::function<bool (const Vertex &, const Vertex &)> myConnectionFilter = cc;
    planner2->setConnectionFilter(myConnectionFilter);
    //class_cf c_filter = new class_cf(planner2);
    //class_cf c = class_cf(planner2);

    // planner2->setConnectionFilter(myConnectionFilter);
    //planner2->setNearestNeighbors();
    //og::KStarStrategy<> strategy
    //og::KStarStrategy<Milestone> strategy = new og::KStrategy();
    //planner2->setConnectionStrategy(strategy);
    //const auto strategy = KBoundedStrategy<Vertex>(5, 10.0, getDefaultNearestNeighbors(planner2));
    // unsigned int k = 20;
    // unsigned int milestone_Count = 0;
    // unsigned int edge_Count = 0;
    //planner2->og::LazyPRM::setMaxNearestNeighbors(k);
    // milestone_Count = planner2->milestoneCount();
    // edge_Count = planner2->edgeCount();

    // ROS_INFO("milestone_count: %u",milestone_Count);
    // ROS_INFO("edge_Count: %u",edge_Count);

    // ob::PlannerPtr planner(planner2);
    // ob::PlannerPtr planner_RRT(new og::RRT(si));
    // Tell the planner which problem we are interested in solving 
    planner2->setProblemDefinition(pdef);

    // CONN_FILTER my_ConnectionFilter;

    // Make sure all the settings for the space and planner are in order. 
    // This will also lead to the runtime computation of the state validity checking resolution. 
    planner2->setup();
    // close file with data
    myfile2.close();
 
    // solve motion planning problem
    ob::PlannerStatus solved = planner2->ob::Planner::solve(20.0);


    // ob::PlannerData* planner_data = new ob::PlannerData(si); 
    // planner->getPlannerData(*planner_data);


    // --------- benchmark --------------
    // ompl::geometric::SimpleSetup ss(si);
    // ss.setStartState(start);
    // ss.setGoalState(goal);
    // // First we create a benchmark class:
    // ompl::tools::Benchmark b(ss, "my experiment");
    // // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
    // b.addExperimentParameter("num_dofs", "INTEGER", "6");
    // b.addExperimentParameter("num_obstacles", "INTEGER", "10");
    // // We add the planners to evaluate.
    // b.addPlanner(planner);
    // b.addPlanner(planner_RRT);

    // ompl::tools::Benchmark::Request req;
    // req.maxTime = 5.0;
    // req.maxMem = 100.0;
    // req.runCount = 50;
    // req.displayProgress = true;
    // b.benchmark(req);
    // b.saveResultsToFile();

    // --------------- saving conf to graph ----------------
    // fb.open ("properties",std::ios::out);
    // planner->printProperties(os);
    // fb.close();
    // fb.open ("settings",std::ios::out);
    // planner->printSettings(os);
    // fb.close();
    // fb.open ("graph2",std::ios::out);
    // planner_data->printGraphviz(os);
    // fb.close();

    std::cout<< solved <<"\n";
    ROS_INFO("@ %f @",planner2->getRange());
    //std::cin.get();
    // if solved == true, a solution was found
    nav_msgs::Path plannedPath;
    
    if (solved) 
    {
        std::cout<<"SOLVEEEEEEEED";
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
}



