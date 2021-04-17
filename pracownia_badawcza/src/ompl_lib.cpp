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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/planners/prm/PRM.h>
//#include <ompl/base/OptimizationObjective.h>

// ---------------- namespaces ----------------
using namespace std;
using namespace ros;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// ---------------- variables ----------------

// map data cut
std::vector< int > globalMap_copy;

// vertex 
using Vertex = boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;

// 2D problem
int dim = 2;

// client for connecting with local neural planner
ros::ServiceClient *nn_client;

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

// polynomial path
double k_vall_st0 = 0.0;
double k_vall_st1 = 0.0;

// occupancy map used for planning
nav_msgs::OccupancyGrid occupancyMap;

// robot size
double rear_axle_to_front = 3.375;
double rear_axle_to_back = 0.67;
double W = 1.72;
double max_curvature = 1 / (5.3 - 1.792 / 2);
double robot_width = 4.0;
double robot_height = 1.2;

// max beta value
double beta_defined = M_PI/3;

// path length
int path_length;

// first scan - helps with picking good path in motion_validator
int first_scan = 0;

// vectors that store values that are helpful with checking robots shape while isStateValid function
std::vector< double > rear_fb;

std::vector< double > bottom;
std::vector< double > top;
std::vector< double > dist;

std::vector< double > bottom2;
std::vector< double > top2;

// polynomial factors
long double a,b,c,d,e,f; 
struct factors
{
    double value1;
    double value2;
    double value3;
    double value4;
    double value5;
    double value6;
};

std::vector<double> cpp_linspace(double x, double krok)
{
    std::vector< double > return_vect;
    for (double i=0.0; i <=x ;i=i+krok)
    {
        return_vect.push_back(i);
    }
    return_vect.push_back(x);
    return return_vect;
}

std::vector<double> cpp_linspace2(double x, double krok)
{
    std::vector< double > return_vect;
    int counter = 0;
    // ROS_INFO("x: %f",x);
    // ROS_INFO("krok: %f",krok);
    for (double i=0.0; i < abs(x) ;i=i+krok)
    {
        
        if (counter<127.0)
        {
        return_vect.push_back(i);
        counter++;
        // ROS_INFO("counter: %d",counter);
        }
    }
    return_vect.push_back(x);
    return return_vect;
}

// constructor in which we create our space
Planner2D::Planner2D(ros::ServiceClient& my_client)
{
    // globalMap_copy = occupancyMap;

    // getting client info from map_node file, where we have access to NodeHandle
    nn_client = &my_client;
   
    std::cout<<'WIDTH'<<endl;
    // checking points along robot's width
    for (double i=0.1; i<=W ;i=i+0.1)
    {
        bottom.push_back(i);
        std::cout<<i<<endl;
    }
    bottom.push_back(1.72);

    std::cout<<"REAR"<<endl;
    // checking points along robot's rear front height
    for (double i=0.1; i <=rear_axle_to_front + rear_axle_to_back;i=i+0.1)
    {
        rear_fb.push_back(i);
        std::cout<<i<<endl;
    }
    rear_fb.push_back(rear_axle_to_front + rear_axle_to_back);


   // checking points along robot's width
    for (double i=0.2; i<=robot_width ;i=i+0.2)
    {
        top2.push_back(i);
    }
    top2.push_back(4.0);

    // checking points along robot's height
    for (double i=0.2; i <=robot_height ;i=i+0.2)
    {
        bottom2.push_back(i);
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
    ROS_INFO("Starting global planner");
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
    // wait till map is being read
    if ((occupancyMap.info.width > 0) && (occupancyMap.info.height > 0)){
    // get x coord of the robot
    const ob::RealVectorStateSpace::StateType *coordX =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const ob::RealVectorStateSpace::StateType *coordY =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    // get yaw coord of the robot
    const ob::RealVectorStateSpace::StateType *coordYaw =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    // calculate 2D map postions
    double x_pos = (coordX->values[0]-occupancyMap.info.origin.position.x)/occupancyMap.info.resolution;
    double y_pos = (coordY->values[0]-occupancyMap.info.origin.position.y)/occupancyMap.info.resolution;
    double yaw = coordYaw->values[0];

    // ROS_INFO("x_pos %f", x_pos);
    // ROS_INFO("y_pos %f", y_pos);
    // ROS_INFO("yaw %f", yaw);

    // help variable for storing value that tell us, if robot state is valid
    bool temp=true;

    // get robot edges coordinates
    // double left_top_x_angled = x_pos - (rear_axle_to_front*cos(yaw) - (W/2.0)*sin(yaw));
    // double left_top_y_angled = y_pos + (rear_axle_to_front*sin(yaw) + (W/2.0)*cos(yaw));

    // double left_down_x_angled = x_pos - (rear_axle_to_back*cos(yaw) - (W/2.0)*sin(yaw));
    // double left_down_y_angled = y_pos - (rear_axle_to_back*sin(yaw) + (W/2.0)*cos(yaw));

    // double right_down_x_angled = x_pos + (rear_axle_to_back*cos(yaw) - (W/2.0)*sin(yaw));
    // double right_down_y_angled = y_pos - (rear_axle_to_back*sin(yaw) + (W/2.0)*cos(yaw));

    // double right_top_x_angled = x_pos + (rear_axle_to_front*cos(yaw) - (W/2.0)*sin(yaw));
    // double right_top_y_angled = y_pos + (rear_axle_to_front*sin(yaw) + (W/2.0)*cos(yaw));

    double TL_x_local = rear_axle_to_front;
    double TL_y_local = W/2.0;

    double DL_x_local = -1.0*rear_axle_to_back;
    double DL_y_local = W/2.0;

    double DR_x_local = -1.0*rear_axle_to_back;
    double DR_y_local = -1.0*W/2.0;

    double TR_x_local = rear_axle_to_front;
    double TR_y_local = -1.0*W/2.0;

    // #####################################

    double TL_x_global = TL_x_local*cos(yaw) - TL_y_local*sin(yaw);
    double TL_y_global = TL_x_local*sin(yaw) + TL_y_local*cos(yaw);

    double DL_x_global = DL_x_local*cos(yaw) - DL_y_local*sin(yaw);
    double DL_y_global = DL_x_local*sin(yaw) + DL_y_local*cos(yaw);

    double DR_x_global = DR_x_local*cos(yaw) - DR_y_local*sin(yaw);
    double DR_y_global = DR_x_local*sin(yaw) + DR_y_local*cos(yaw);

    double TR_x_global = TR_x_local*cos(yaw) - TR_y_local*sin(yaw);
    double TR_y_global = TR_x_local*sin(yaw) + TR_y_local*cos(yaw);

    // ######################################

    double left_top_x_angled = x_pos + TL_x_global;
    double left_top_y_angled = y_pos + TL_y_global;

    double left_down_x_angled = x_pos + DL_x_global;
    double left_down_y_angled = y_pos + DL_y_global;

    double right_down_x_angled = x_pos + DR_x_global;
    double right_down_y_angled = y_pos + DR_y_global;

    double right_top_x_angled = x_pos + TR_x_global;
    double right_top_y_angled = y_pos + TR_y_global;

    double a_TL_DL = (left_top_y_angled-left_down_y_angled)/(left_top_x_angled-left_down_x_angled);
    double b_TL_DL = left_top_y_angled-(a_TL_DL*left_top_x_angled);
    
    double a_TL_TR = (left_top_y_angled-right_top_y_angled)/(left_top_x_angled-right_top_x_angled);
    double b_TL_TR = left_top_y_angled-(a_TL_TR*left_top_x_angled);

    double a_DR_TR = (right_down_y_angled-right_top_y_angled)/(right_down_x_angled-right_top_x_angled);
    double b_DR_TR = right_down_y_angled-(a_DR_TR*right_down_x_angled);

    double a_DR_DL = (right_down_y_angled-left_down_y_angled)/(right_down_x_angled-left_down_x_angled);
    double b_DR_DL = right_down_y_angled-(a_DR_DL*right_down_x_angled);

    double sign_rear = 1.0;
    double sign_bott = 1.0;

    std::vector<double> longer_x = cpp_linspace(abs(left_top_x_angled - left_down_x_angled), abs(left_top_x_angled - left_down_x_angled)/20);
    std::vector<double> longer_y = cpp_linspace(abs(left_top_y_angled - left_down_y_angled), abs(left_top_y_angled - left_down_y_angled)/20);

    std::vector<double> shorter_x = cpp_linspace(abs(left_top_x_angled - right_top_x_angled), abs(left_top_x_angled - right_top_x_angled)/20);
    std::vector<double> shorter_y = cpp_linspace(abs(left_top_y_angled - right_top_y_angled), abs(left_top_y_angled - right_top_y_angled)/20);

    ROS_INFO("srodek X: %f", coordX->values[0]);
    ROS_INFO("srodek Y: %f", coordY->values[0]);
    ROS_INFO("kat: %f", coordYaw->values[0]);

    ROS_INFO("################################");

    ROS_INFO("longer_x : %f", abs(left_top_x_angled - left_down_x_angled));
    ROS_INFO("dzielnik longer_x : %f", abs(left_top_x_angled - left_down_x_angled)/20);
    ROS_INFO("longer_y : %f", abs(left_top_y_angled - left_down_y_angled));
    ROS_INFO("dzielnik longer_y : %f", abs(left_top_y_angled - left_down_y_angled)/20);
    ROS_INFO("shorter_x: %f", abs(left_top_x_angled - right_top_x_angled));
    ROS_INFO("dzielnik shorter_x: %f", abs(left_top_x_angled - right_top_x_angled)/20);
    ROS_INFO("shorter_y: %f", abs(left_top_y_angled - right_top_y_angled));
    ROS_INFO("dzielnik shorter_y: %f", abs(left_top_y_angled - right_top_y_angled)/20);

    ROS_INFO("################################");


    ROS_INFO("left_top_x_angled: %f", left_top_x_angled);
    ROS_INFO("left_top_y_angled: %f", left_top_y_angled);
    ROS_INFO("left_down_x_angled: %f", left_down_x_angled);
    ROS_INFO("left_down_y_angled: %f", left_down_y_angled);
    ROS_INFO("right_down_x_angled: %f", right_down_x_angled);
    ROS_INFO("right_down_y_angled: %f", right_down_y_angled);
    ROS_INFO("right_top_x_angled: %f", right_top_x_angled);
    ROS_INFO("right_top_y_angled: %f", right_top_y_angled);

    ROS_INFO("################################");

    ROS_INFO("a_TL_DL %f", a_TL_DL);
    ROS_INFO("b_TL_DL %f", b_TL_DL);
    ROS_INFO("a_TL_TR %f", a_TL_TR);
    ROS_INFO("b_TL_TR %f", b_TL_TR);
    ROS_INFO("a_DR_TR %f", a_DR_TR);
    ROS_INFO("b_DR_TR %f", b_DR_TR);
    ROS_INFO("a_DR_DL %f", a_DR_DL);
    ROS_INFO("b_DR_DL %f", b_DR_DL);



    // map variables
    int mapIndex1 = 0;
    int occupancyMapValue1 = 0;
    int mapIndex2 = 0;
    int occupancyMapValue2 = 0;

    // cwiartka I
    if (((yaw >= 0.0) && (yaw < M_PI/2)) || ((yaw >= -2.0*M_PI) && (yaw < (-3.0*M_PI)/2))) 
    {
        ROS_INFO("1");
        sign_rear = -1.0;
        sign_bott = 1.0;
    }
    // cwiartka II
    if (((yaw >= M_PI/2) && (yaw < M_PI)) || ((yaw >= (-3.0*M_PI)/2) && (yaw < (-1.0*M_PI)))) 
    {
        ROS_INFO("2");
        sign_rear = 1.0;
        sign_bott = 1.0;
    }
    // cwiartka III
    if (((yaw >= M_PI) && (yaw < (3.0*M_PI)/2)) || ((yaw >= -1.0*M_PI) && (yaw < -1.0*(M_PI)/2))) 
    {
        ROS_INFO("3");
        sign_rear = 1.0;
        sign_bott = -1.0;
    }
    // cwiartka IV - never seen that it goes here
    if (((yaw >= 3.0*M_PI/2.0) && (yaw < (2.0*M_PI))) || ((yaw >= -1.0*M_PI/2.0) && (yaw < 0.0))) 
    {
        ROS_INFO("4");
        sign_rear = -1.0;
        sign_bott = -1.0;
    }
    //std::cin.get();
    // ROS_INFO("map index tab len: %ld", occupancyMap.data.size());
    // ROS_INFO("rear_fb tab len: %ld", rear_fb.size());
    for (int i=0; i < longer_x.size(); i++) 
    {
        // ROS_INFO("TL_Y next1: %f" ,a_TL_DL*(left_top_x_angled + (sign_rear*longer_x[i])+b_TL_DL));
        // ROS_INFO("TL_X next1: %f" ,left_top_x_angled + (sign_rear*longer_x[i]));

        // ROS_INFO("TL_Y next1: %f" ,a_TL_DL*(left_top_x_angled + (sign_rear*longer_x[i])+b_TL_DL));
        // ROS_INFO("TL_X next1: %f" ,left_top_x_angled + (sign_rear*longer_x[i]));
        // std::cin.get();

        // ROS_INFO("1 wart: %f", (a_TL_DL*(left_top_x_angled + (sign_rear*rear_fb[i]))+b_TL_DL));
        // ROS_INFO("width: %d", occupancyMap.info.width);
        // ROS_INFO("all: %d", (int)((a_TL_DL*(left_top_x_angled + (sign_rear*rear_fb[i]))+b_TL_DL)*occupancyMap.info.width)+(int)(left_top_x_angled + (sign_rear*rear_fb[i])));
        // ROS_INFO("2 wart: %f", (left_top_x_angled + (sign_rear*rear_fb[i])));

        // ROS_INFO("Indeks %f: ", longer_x[i]);
        // ROS_INFO("WSPOLRZEDNE PUNKTU 1_1: ");
        // ROS_INFO("(%f , %f) ", (left_top_x_angled + (sign_rear*longer_x[i])),(a_TL_DL*(left_top_x_angled + (sign_rear*longer_x[i]))+b_TL_DL));

        // ROS_INFO("Mnoze %d z %d i dodaje %d",(int)(a_TL_DL*(left_top_x_angled + (sign_rear*rear_fb[i]))+b_TL_DL),occupancyMap.info.width,(int)(left_top_x_angled + (sign_rear*rear_fb[i])));
        // ROS_INFO("Mnoze %f z %d i dodaje %f",(a_TL_DL*(left_top_x_angled + (sign_rear*rear_fb[i]))+b_TL_DL),occupancyMap.info.width,(left_top_x_angled + (sign_rear*rear_fb[i])));
        
        // TL - DL

        mapIndex1 = (int)((a_TL_DL*(left_top_x_angled + (sign_rear*longer_x[i])) + b_TL_DL))*occupancyMap.info.width+(int)(left_top_x_angled + (sign_rear*longer_x[i]));
        occupancyMapValue1 = occupancyMap.data[mapIndex1];
        // ROS_INFO("all: %d", (int)((a_DR_TR*(right_top_x_angled + (sign_rear*rear_fb[i]))+b_DR_TR)*occupancyMap.info.width)+(int)(right_top_x_angled + (sign_rear*rear_fb[i])));
        
        // TR - DR
        // ROS_INFO("Szukam Indeksu 1_2: ");
        // ROS_INFO("Mnoze %d z %d i dodaje %d",(int)(a_DR_TR*(right_top_x_angled + (sign_rear*rear_fb[i]))+b_DR_TR),occupancyMap.info.width,(int)(right_top_x_angled + (sign_rear*rear_fb[i])));
        // ROS_INFO("Mnoze %f z %d i dodaje %f",(a_DR_TR*(right_top_x_angled + (sign_rear*rear_fb[i]))+b_DR_TR),occupancyMap.info.width,(right_top_x_angled + (sign_rear*rear_fb[i])));
        // ROS_INFO("WSPOLRZEDNE PUNKTU 1_2: ");
        // ROS_INFO("(%f , %f) ", (right_top_x_angled + (sign_rear*longer_x[i])),(a_DR_TR*(right_top_x_angled + (sign_rear*longer_x[i]))+b_DR_TR));

        mapIndex2 = (int)((a_DR_TR*(right_top_x_angled + (sign_rear*longer_x[i]))+b_DR_TR))*occupancyMap.info.width+(int)(right_top_x_angled + (sign_rear*longer_x[i]));
        occupancyMapValue2 = occupancyMap.data[mapIndex2];
        // ROS_INFO("OCP1_1: %d", occupancyMapValue1);
        // ROS_INFO("OCP1_2: %d", occupancyMapValue2);
        //std::cin.get();
        if ((occupancyMapValue1 == 100) || (occupancyMapValue2 == 100))
        {
            temp=true;
            break;
        }
        else temp=false;
    }

    if(temp==true) return false;
    else return true;

    // ROS_INFO("1 wart: %f", (a_TL_DL*(left_top_x_angled + (sign_rear*rear_fb[i]))+b_TL_DL));
    // ROS_INFO("width: %f", occupancyMap.info.width);
    // ROS_INFO("2 wart: %f", (left_top_x_angled + (sign_rear*rear_fb[i])));
    for (int i=0; i < shorter_x.size(); i++) 
    {
        // TL - TR

        // ROS_INFO("Szukam Indeksu 2_1: ");
        // ROS_INFO("Mnoze %d z %d i dodaje %d",(int)(a_TL_TR*(left_top_x_angled + (sign_bott*shorter_x[i]))+b_TL_TR),occupancyMap.info.width,(int)(left_top_x_angled + (sign_bott*shorter_x[i])));
        
        mapIndex1 = (int)((a_TL_TR*(left_top_x_angled + (sign_bott*shorter_x[i]))+b_TL_TR))*occupancyMap.info.width+(int)(left_top_x_angled + (sign_bott*shorter_x[i]));
        occupancyMapValue1 = occupancyMap.data[mapIndex1];


        // ROS_INFO("rear_fb: %f" ,bottom[i]);
        // ROS_INFO("1 wart: %f", (a_DR_DL*(left_down_x_angled + (sign_bott*bottom[i]))+b_DR_DL));
        // ROS_INFO("width: %d", occupancyMap.info.width);
        // ROS_INFO("2 wart: %f", (left_down_x_angled + (sign_bott*bottom[i])));
        // DL - DR
        // ROS_INFO("IDX2_2");
        // ROS_INFO("Szukam Indeksu 2_2: ");
        // ROS_INFO("Mnoze %d z %d i dodaje %d",(int)(a_DR_DL*(left_down_x_angled + (sign_bott*shorter_x[i]))+b_DR_DL),occupancyMap.info.width,(int)(left_down_x_angled + (sign_bott*shorter_x[i])));

        mapIndex2 = (int)((a_DR_DL*(left_down_x_angled + (sign_bott*shorter_x[i]))+b_DR_DL))*occupancyMap.info.width+(int)(left_down_x_angled + (sign_bott*shorter_x[i]));
        occupancyMapValue2 = occupancyMap.data[mapIndex2];
        // ROS_INFO("IDX2_1: %d", mapIndex1);
        // ROS_INFO("OCP2_2: %d", occupancyMapValue1);
        // ROS_INFO("IDX2_1: %d", mapIndex2);
        // ROS_INFO("OCP2_2: %d", occupancyMapValue2);
        if ((occupancyMapValue1 == 100) || (occupancyMapValue2 == 100))
        {
            temp=true;
            break;
        }
        else temp=false;
    }

    // ROS_INFO("left_top_x_angled %f", left_top_x_angled);
    // ROS_INFO("left_top_y_angled %f", left_top_y_angled);
    // ROS_INFO("left_down_x_angled %f", left_down_x_angled);
    // ROS_INFO("left_down_y_angled %f", left_down_y_angled);
    // ROS_INFO("right_down_x_angled %f", right_down_x_angled);
    // ROS_INFO("right_down_y_angled %f", right_down_y_angled);
    // ROS_INFO("right_top_x_angled %f", right_top_x_angled);
    // ROS_INFO("right_top_y_angled %f", right_top_y_angled);

    



    // // checking shorter robot side
    // for (int i=0; i <= rear_fb.size(); i++) {
    //     mapIndex = (int)((left_top_y_angled-rear_fb[i]))*occupancyMap.info.width+(int)(left_top_x_angled);
    //     occupancyMapValue = occupancyMap.data[mapIndex];
    //     if (occupancyMapValue == 100)
    //     {
    //         temp=true;
    //         break;
    //     }
    //     else temp=false;
    //     mapIndex = (int)((right_down_y_angled+rear_fb[i]))*occupancyMap.info.width+int(right_down_x_angled);
    //     occupancyMapValue = occupancyMap.data[mapIndex];
    //     if (occupancyMapValue == 100)
    //     {
    //         temp=true;
    //         break;
    //     }
    //     else temp=false;
    // }
    // if(temp==true) return false;
    // else return true;

    // // checking longer robot side
    // for (int i=0; i <= bottom.size(); i++) {
    //     mapIndex = (int)(left_top_y_angled)*occupancyMap.info.width+(int)(left_top_x_angled-bottom[i]);
    //     occupancyMapValue = occupancyMap.data[mapIndex];
    //     if (occupancyMapValue == 100){
    //         temp=true;
    //         break;
    //         }
    //         else temp=false;
    //     mapIndex = (int)(right_down_y_angled)*occupancyMap.info.width+(int)(right_down_x_angled+bottom[i]);
    //     occupancyMapValue = occupancyMap.data[mapIndex];
    //     if (occupancyMapValue == 100){
    //         temp=true;
    //         break;
    //         }
    //         else temp=false;
    // }
    // ROS_INFO("Tuuuuuuuu");
    if(temp==true) return false;
    else return true;
    }

} // end isStateValid


bool isStateValid2(double xX, double yY, double zZ)
{
     // wait till map is being read
    if ((occupancyMap.info.width > 0) && (occupancyMap.info.height > 0)){
    // get x coord of the robot
    double coordX = xX;
    double coordY = yY;
    double coordYaw = zZ;

    // calculate 2D map postions
    double x_pos = (coordX-occupancyMap.info.origin.position.x)/occupancyMap.info.resolution;
    double y_pos = (coordY-occupancyMap.info.origin.position.y)/occupancyMap.info.resolution;
    double yaw = coordYaw;

    // ROS_INFO("x_pos %f", x_pos);
    // ROS_INFO("y_pos %f", y_pos);
    // ROS_INFO("yaw %f", yaw);

    // help variable for storing value that tell us, if robot state is valid
    bool temp=true;

    // get robot edges coordinates
    // double left_top_x_angled = x_pos - (rear_axle_to_front*cos(yaw) - (W/2.0)*sin(yaw));
    // double left_top_y_angled = y_pos + (rear_axle_to_front*sin(yaw) + (W/2.0)*cos(yaw));

    // double left_down_x_angled = x_pos - (rear_axle_to_back*cos(yaw) - (W/2.0)*sin(yaw));
    // double left_down_y_angled = y_pos - (rear_axle_to_back*sin(yaw) + (W/2.0)*cos(yaw));

    // double right_down_x_angled = x_pos + (rear_axle_to_back*cos(yaw) - (W/2.0)*sin(yaw));
    // double right_down_y_angled = y_pos - (rear_axle_to_back*sin(yaw) + (W/2.0)*cos(yaw));

    // double right_top_x_angled = x_pos + (rear_axle_to_front*cos(yaw) - (W/2.0)*sin(yaw));
    // double right_top_y_angled = y_pos + (rear_axle_to_front*sin(yaw) + (W/2.0)*cos(yaw));

    double TL_x_local = rear_axle_to_front;
    double TL_y_local = W/2.0;

    double DL_x_local = -1.0*rear_axle_to_back;
    double DL_y_local = W/2.0;

    double DR_x_local = -1.0*rear_axle_to_back;
    double DR_y_local = -1.0*W/2.0;

    double TR_x_local = rear_axle_to_front;
    double TR_y_local = -1.0*W/2.0;

    // #####################################

    double TL_x_global = TL_x_local*cos(yaw) - TL_y_local*sin(yaw);
    double TL_y_global = TL_x_local*sin(yaw) + TL_y_local*cos(yaw);

    double DL_x_global = DL_x_local*cos(yaw) - DL_y_local*sin(yaw);
    double DL_y_global = DL_x_local*sin(yaw) + DL_y_local*cos(yaw);

    double DR_x_global = DR_x_local*cos(yaw) - DR_y_local*sin(yaw);
    double DR_y_global = DR_x_local*sin(yaw) + DR_y_local*cos(yaw);

    double TR_x_global = TR_x_local*cos(yaw) - TR_y_local*sin(yaw);
    double TR_y_global = TR_x_local*sin(yaw) + TR_y_local*cos(yaw);

    // ######################################

    double left_top_x_angled = x_pos + TL_x_global;
    double left_top_y_angled = y_pos + TL_y_global;

    double left_down_x_angled = x_pos + DL_x_global;
    double left_down_y_angled = y_pos + DL_y_global;

    double right_down_x_angled = x_pos + DR_x_global;
    double right_down_y_angled = y_pos + DR_y_global;

    double right_top_x_angled = x_pos + TR_x_global;
    double right_top_y_angled = y_pos + TR_y_global;

    double a_TL_DL = (left_top_y_angled-left_down_y_angled)/(left_top_x_angled-left_down_x_angled);
    double b_TL_DL = left_top_y_angled-(a_TL_DL*left_top_x_angled);
    
    double a_TL_TR = (left_top_y_angled-right_top_y_angled)/(left_top_x_angled-right_top_x_angled);
    double b_TL_TR = left_top_y_angled-(a_TL_TR*left_top_x_angled);

    double a_DR_TR = (right_down_y_angled-right_top_y_angled)/(right_down_x_angled-right_top_x_angled);
    double b_DR_TR = right_down_y_angled-(a_DR_TR*right_down_x_angled);

    double a_DR_DL = (right_down_y_angled-left_down_y_angled)/(right_down_x_angled-left_down_x_angled);
    double b_DR_DL = right_down_y_angled-(a_DR_DL*right_down_x_angled);

    double sign_rear = 1.0;
    double sign_bott = 1.0;

    std::vector<double> longer_x = cpp_linspace(abs(left_top_x_angled - left_down_x_angled), abs(left_top_x_angled - left_down_x_angled)/20);
    std::vector<double> longer_y = cpp_linspace(abs(left_top_y_angled - left_down_y_angled), abs(left_top_y_angled - left_down_y_angled)/20);

    std::vector<double> shorter_x = cpp_linspace(abs(left_top_x_angled - right_top_x_angled), abs(left_top_x_angled - right_top_x_angled)/20);
    std::vector<double> shorter_y = cpp_linspace(abs(left_top_y_angled - right_top_y_angled), abs(left_top_y_angled - right_top_y_angled)/20);

    // map variables
    int mapIndex1 = 0;
    int occupancyMapValue1 = 0;
    int mapIndex2 = 0;
    int occupancyMapValue2 = 0;

    // cwiartka I
    if (((yaw >= 0.0) && (yaw < M_PI/2)) || ((yaw >= -2.0*M_PI) && (yaw < (-3.0*M_PI)/2))) 
    {
        ROS_INFO("1");
        sign_rear = -1.0;
        sign_bott = 1.0;
    }
    // cwiartka II
    if (((yaw >= M_PI/2) && (yaw < M_PI)) || ((yaw >= (-3.0*M_PI)/2) && (yaw < (-1.0*M_PI)))) 
    {
        ROS_INFO("2");
        sign_rear = 1.0;
        sign_bott = 1.0;
    }
    // cwiartka III
    if (((yaw >= M_PI) && (yaw < (3.0*M_PI)/2)) || ((yaw >= -1.0*M_PI) && (yaw < -1.0*(M_PI)/2))) 
    {
        ROS_INFO("3");
        sign_rear = 1.0;
        sign_bott = -1.0;
    }
    // cwiartka IV - never seen that it goes here
    if (((yaw >= 3.0*M_PI/2.0) && (yaw < (2.0*M_PI))) || ((yaw >= -1.0*M_PI/2.0) && (yaw < 0.0))) 
    {
        ROS_INFO("4");
        sign_rear = -1.0;
        sign_bott = -1.0;
    }

    for (int i=0; i < longer_x.size(); i++) 
    {
        mapIndex1 = (int)((a_TL_DL*(left_top_x_angled + (sign_rear*longer_x[i])) + b_TL_DL))*occupancyMap.info.width+(int)(left_top_x_angled + (sign_rear*longer_x[i]));
        occupancyMapValue1 = occupancyMap.data[mapIndex1];

        mapIndex2 = (int)((a_DR_TR*(right_top_x_angled + (sign_rear*longer_x[i]))+b_DR_TR))*occupancyMap.info.width+(int)(right_top_x_angled + (sign_rear*longer_x[i]));
        occupancyMapValue2 = occupancyMap.data[mapIndex2];
        if ((occupancyMapValue1 == 100) || (occupancyMapValue2 == 100))
        {
            temp=true;
            break;
        }
        else temp=false;
    }

    if(temp==true) return false;
    else return true;
    for (int i=0; i < shorter_x.size(); i++) 
    {
        mapIndex1 = (int)((a_TL_TR*(left_top_x_angled + (sign_bott*shorter_x[i]))+b_TL_TR))*occupancyMap.info.width+(int)(left_top_x_angled + (sign_bott*shorter_x[i]));
        occupancyMapValue1 = occupancyMap.data[mapIndex1];
        mapIndex2 = (int)((a_DR_DL*(left_down_x_angled + (sign_bott*shorter_x[i]))+b_DR_DL))*occupancyMap.info.width+(int)(left_down_x_angled + (sign_bott*shorter_x[i]));
        occupancyMapValue2 = occupancyMap.data[mapIndex2];
        if ((occupancyMapValue1 == 100) || (occupancyMapValue2 == 100))
        {
            temp=true;
            break;
        }
        else temp=false;
    }
    if(temp==true) return false;
    else return true;
    }

} // end isStateValid





// calculating polynomial factors given 2 states
factors calculate_factors(double k_val_st0,double k_val_st1,double th0,double x0,double y0,double th1,double x1, double y1)
{
    // local variables
    double aa,bb,cc,dd,ee,ff,w,z;

    // calculations
    ff = y0;
    ee = tan(th0);
    dd = 0.0;
    w = y1 - ee*x1;
    z = tan(th1) - ee;
    aa = (4.0*w*z - x1*x1*z + 3*z*x1)/(4.0*pow(x1,5));
    cc = (4.0*w*z + 16.0*w - x1*x1*z - x1*z)/(4.0*x1*x1*x1);
    bb = (z - 5*aa*pow(x1,4) - 3.0*cc*x1*x1)/(4.0*x1*x1*x1);

    // ROS_INFO("A: %f",aa*10000);
    // ROS_INFO("B: %f",bb*10000);
    // ROS_INFO("C: %f",cc*10000);
    // ROS_INFO("D: %f",dd*10000);
    // ROS_INFO("E: %f",ee*10000);
    // ROS_INFO("F: %f",ff*10000);

    // store values to structure and return it
    factors result = {aa,bb,cc,dd,ee,ff};
    return result;
} // end calculate factors

// overload function for class that is responsible for communicating with local neural planner
bool class_cf::cf_client(const ob::State *state0, const ob::State *state1)
{
    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state0_coordX = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state0_coordY = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state0_coordYaw = state0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state1_coordX = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = state1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    // create service 
    pracownia_badawcza::LNP nn_srv;

    // global polynomial variables
    double x_wielomian_global = 0.0;
    double y_wielomian_global = 0.0;
    double th_wielomian_global = 0.0;

    // local polynomial variables
    double x_wielomian = 0.0;
    double y_wielomian = 0.0;
    double th_wielomian = 0.0;

    double tan_th_wielomian = 0.0;
    double curve = 0.0;
    double max_curve = 0.0;
    double signn = 1.0;

    double x_diff_lok = 0.0;
    double y_diff_lok = 0.0;

    // double left_top_pic_x = 0.0;
    // double left_top_pic_y = 0.0;

    // double right_bottom_pic_x = 0.0;
    // double right_bottom_pic_y = 0.0;

    // map variables
    int mapIndex = 0;
    int mapIndex2 = 0;
    int occupancyMapValue = 0;

    bool is_available = true;
    // ROS_INFO("state0 X: %f",state0_coordX->values[0]);
    // ROS_INFO("state0 Y: %f",state0_coordY->values[0]);
    // ROS_INFO("state0 TH: %f",state0_coordYaw->values[0]);
    // ROS_INFO("state1 X: %f",state1_coordX->values[0]);
    // ROS_INFO("state1 Y: %f",state1_coordY->values[0]);
    // ROS_INFO("state1 TH: %f",state1_coordYaw->values[0]);

    // calculating state1 in state0 coordinates 
    double th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    double x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
    double y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);

    // uklad 0
    double x0_in_s0 = 0.4;
    double y0_in_s0 = 0.0;
    double th0_in_s0 = 0.0;

    // condition no.1 - abs value of orientation in local state1 can't be greater as pi/2
    if (abs(th1_in_s0) >= M_PI/2)
     {
        // ROS_INFO("condition no. 1");
        return false;
     }

    // store proper values
    // nn_srv.request.s1_x = x0_in_s0;
    // nn_srv.request.s1_y = y0_in_s0;
    // nn_srv.request.s1_yaw = th0_in_s0;
    nn_srv.request.s_x = x1_in_s0;
    nn_srv.request.s_y = y1_in_s0;
    nn_srv.request.s_yaw = th1_in_s0;






    double TL_x_local = 122;
    double TL_y_local = 128/2.0;

    double DL_x_local = -1.0*6.0;;
    double DL_y_local = 128/2.0;

    double DR_x_local = -1.0*6.0;
    double DR_y_local = -1.0*128/2.0;

    double TR_x_local = 122.0;
    double TR_y_local = -1.0*128/2.0;

    double yaw = state0_coordYaw->values[0];

    // #####################################

    double TL_x_global = TL_x_local*cos(yaw) - TL_y_local*sin(yaw);
    double TL_y_global = TL_x_local*sin(yaw) + TL_y_local*cos(yaw);

    double DL_x_global = DL_x_local*cos(yaw) - DL_y_local*sin(yaw);
    double DL_y_global = DL_x_local*sin(yaw) + DL_y_local*cos(yaw);

    double DR_x_global = DR_x_local*cos(yaw) - DR_y_local*sin(yaw);
    double DR_y_global = DR_x_local*sin(yaw) + DR_y_local*cos(yaw);

    double TR_x_global = TR_x_local*cos(yaw) - TR_y_local*sin(yaw);
    double TR_y_global = TR_x_local*sin(yaw) + TR_y_local*cos(yaw);

    // ######################################

    double left_top_x_angled = state0_coordX->values[0] + TL_x_global;
    double left_top_y_angled = state0_coordY->values[0] + TL_y_global;

    double left_down_x_angled = state0_coordX->values[0] + DL_x_global;
    double left_down_y_angled = state0_coordY->values[0] + DL_y_global;

    double right_down_x_angled = state0_coordX->values[0] + DR_x_global;
    double right_down_y_angled = state0_coordY->values[0] + DR_y_global;

    double right_top_x_angled = state0_coordX->values[0] + TR_x_global;
    double right_top_y_angled = state0_coordY->values[0] + TR_y_global;

    double a_TL_DL = (left_top_y_angled-left_down_y_angled)/(left_top_x_angled-left_down_x_angled);
    double b_TL_DL = left_top_y_angled-(a_TL_DL*left_top_x_angled);
    
    double a_TL_TR = (left_top_y_angled-right_top_y_angled)/(left_top_x_angled-right_top_x_angled);
    double b_TL_TR = left_top_y_angled-(a_TL_TR*left_top_x_angled);

    double a_DR_TR = (right_down_y_angled-right_top_y_angled)/(right_down_x_angled-right_top_x_angled);
    double b_DR_TR = right_down_y_angled-(a_DR_TR*right_down_x_angled);

    double a_DR_DL = (right_down_y_angled-left_down_y_angled)/(right_down_x_angled-left_down_x_angled);
    double b_DR_DL = right_down_y_angled-(a_DR_DL*right_down_x_angled);



    // ROS_INFO("srodek X: %f", state0_coordX->values[0]);
    // ROS_INFO("srodek Y: %f", state0_coordY->values[0]);
    // ROS_INFO("kat: %f", state0_coordYaw->values[0]);


    // double TL_x_local = 100;
    // double TL_y_local = 128/2.0;

    // double DL_x_local = -1.0*28.0;;
    // double DL_y_local = 128/2.0;

    // double DR_x_local = -1.0*28.0;
    // double DR_y_local = -1.0*128/2.0;

    // double TR_x_local = 100.0;
    // double TR_y_local = -1.0*128/2.0;

    // ROS_INFO("a_TL_DL %f", a_TL_DL);
    // ROS_INFO("b_TL_DL %f", b_TL_DL);
    // ROS_INFO("a_TL_TR %f", a_TL_TR);
    // ROS_INFO("b_TL_TR %f", b_TL_TR);
    // ROS_INFO("a_DR_TR %f", a_DR_TR);
    // ROS_INFO("b_DR_TR %f", b_DR_TR);
    // ROS_INFO("a_DR_DL %f", a_DR_DL);
    // ROS_INFO("b_DR_DL %f", b_DR_DL);


    // ROS_INFO("LT X: %f", left_top_x_angled);
    // ROS_INFO("LT Y: %f", left_top_y_angled);

    // ROS_INFO("RT X: %f", right_top_x_angled);
    // ROS_INFO("RT Y: %f", right_top_y_angled);

    // ROS_INFO("LD X: %f", left_down_x_angled);
    // ROS_INFO("LD Y: %f", left_down_y_angled);

    // ROS_INFO("RD X: %f", right_down_x_angled);
    // ROS_INFO("RD Y: %f", right_down_y_angled);

    //std::cin.get();


    double LT_choosed_x = 0.0;
    double LT_choosed_y = 0.0;
    double a1_choosed = 0.0;
    double b1_choosed = 0.0;
    double a2_choosed = 0.0;
    double b2_choosed = 0.0;

    std::vector<double> px;
    std::vector<double> ey; 

    double sign_test_x = 1.0;
    double sign_test_y = 1.0;
    // cwiartka I
    if (((yaw >= 0.0) && (yaw < M_PI/2)) || ((yaw >= -2.0*M_PI) && (yaw < (-3.0*M_PI)/2))) 
    {
        // ROS_INFO("1");
        sign_test_x = 1.0;
        sign_test_y = 1.0;
        LT_choosed_x = left_down_x_angled;
        LT_choosed_y = left_down_y_angled;
        a1_choosed = a_TL_DL;
        b1_choosed = b_TL_DL;
        a2_choosed = a_DR_DL;
        b2_choosed = b_DR_DL;
        px = cpp_linspace2(abs(left_down_x_angled - left_top_x_angled), abs(left_down_x_angled - left_top_x_angled)/128);
        ey = cpp_linspace2(abs(left_down_y_angled - right_down_y_angled), abs(left_down_y_angled - right_down_y_angled)/128);

    }
    // cwiartka II
    if (((yaw >= M_PI/2) && (yaw < M_PI)) || ((yaw >= (-3.0*M_PI)/2) && (yaw < (-1.0*M_PI)))) 
    {
        // ROS_INFO("2");
        sign_test_x = -1.0;
        sign_test_y = -1.0;
        LT_choosed_x = left_top_x_angled;
        LT_choosed_y = left_top_y_angled;
        a1_choosed = a_TL_TR;
        b1_choosed = a_TL_TR;
        a2_choosed = a_TL_DL;
        b2_choosed = b_TL_DL;
        px = cpp_linspace2(abs(left_top_x_angled - right_top_x_angled), abs(left_top_x_angled - right_top_x_angled)/128);
        ey = cpp_linspace2(abs(left_top_y_angled - left_down_y_angled), abs(left_top_y_angled - left_down_y_angled)/128);

    }
    // cwiartka III
    if (((yaw >= M_PI) && (yaw < (3.0*M_PI)/2)) || ((yaw >= -1.0*M_PI) && (yaw < -1.0*(M_PI)/2))) 
    {
        // ROS_INFO("3");
        sign_test_x = -1.0;
        sign_test_y = -1.0;
        LT_choosed_x = right_top_x_angled;
        LT_choosed_y = right_top_y_angled;
        a1_choosed = a_DR_TR;
        b1_choosed = b_DR_TR;
        a2_choosed = a_TL_TR;
        b2_choosed = b_TL_TR;
        // tu maja byc brane y
        px = cpp_linspace2(abs(right_top_y_angled - right_down_y_angled), abs(right_top_y_angled - right_down_y_angled)/128);
        // tu maja byc brane x
        ey = cpp_linspace2(abs(right_top_x_angled - left_top_x_angled), abs(right_top_x_angled - left_top_x_angled)/128);

    }
    // cwiartka IV - never seen that it goes here
    if (((yaw >= 3.0*M_PI/2.0) && (yaw < (2.0*M_PI))) || ((yaw >= -1.0*M_PI/2.0) && (yaw < 0.0))) 
    {
        // ROS_INFO("4");
        sign_test_x = 1.0;
        sign_test_y = 1.0;
        LT_choosed_x = right_down_x_angled;
        LT_choosed_y = right_down_y_angled;
        a1_choosed = a_DR_DL;
        b1_choosed = b_DR_DL;
        a2_choosed = a_DR_TR;
        b2_choosed = b_DR_TR;
        
        px = cpp_linspace2(abs(right_down_x_angled - left_down_x_angled), abs(right_down_x_angled - left_down_x_angled)/128);

        ey = cpp_linspace2(abs(right_down_y_angled - right_top_y_angled), abs(right_down_y_angled - right_top_y_angled)/128);

    }



    //std::cin.get();

    // left_top_x = x0 - (64.0*cos(th0) - 28.0*sin(th0));
    // left_top_y = y0 + (64.0*sin(th0) + 28.0*cos(th0));

    double py_calculated,px_calculated,b_szukane,map_idx_x,map_idx_y = 0.0;

    if (sqrt(x_diff_lok*x_diff_lok + y_diff_lok*y_diff_lok) < sqrt(128.0*128.0))
    {

        // // push to the list points only in the 128x128 square
        // for (int i = left_top_pic_y; i > (left_top_pic_y - 128); i--)
        // {
        //     // ROS_INFO("y  %f : ", left_top_pic_y);
        //     for (int j = left_top_pic_x; j < (left_top_pic_x + 128); j++)
        //     {
        //         // ROS_INFO("x  %f : ", left_top_pic_x);
        //         mapIndex2 = (int)(left_top_pic_y-i)*occupancyMap.info.width +(int)(x_wielomian_global+j);
        //         // ROS_INFO("%d : ", mapIndex);
        //         nn_srv.request.grid_map.push_back(occupancyMap.data[mapIndex2]);
        //     }
        // }
        //std::cin.get();
        // push to the list points only in the 128x128 square


        // ROS_INFO("ey size %d", ey.size());
        // ROS_INFO("px size %d", px.size());
        // std::cin.get();

        for (int i=0; i < ey.size(); i++) 
        {

            py_calculated = LT_choosed_y - sign_test_x*ey[i]*cos(yaw); // dla 2,3 tu ma byc +, 
            px_calculated = (py_calculated-b2_choosed)/a2_choosed;
            b_szukane = py_calculated - a1_choosed*px_calculated;

            for (int j=0; j < px.size(); j++) 
            {
                map_idx_x = px_calculated + sign_test_y*px[j]*cos(yaw); // dla 2,3 tu ma byc -
                map_idx_y = a1_choosed*map_idx_x + b_szukane;

                // ROS_INFO("adding to tab point with x: %f y: %f : ", map_idx_x,map_idx_y);
                mapIndex2 = (int)(map_idx_y)*occupancyMap.info.width +(int)(map_idx_x);
                // ROS_INFO("%d : ", mapIndex);
                nn_srv.request.grid_map.push_back(occupancyMap.data[mapIndex2]);
            }
        }

        // for (int i = 0; i < 128; i++)
        // {
        //     py_calculated = LT_choosed_y - i*cos(yaw);
        //     px_calculated = (py_calculated-b2_choosed)/a2_choosed;
        //     b_szukane = py_calculated - a1_choosed*px_calculated;

            
        //     // ROS_INFO("y  %f : ", left_top_pic_y);
        //     for (int j = 0; j < 128; j++)
        //     {
        //         map_idx_x = px_calculated + j*cos(yaw);
        //         map_idx_y = a1_choosed*map_idx_x + b_szukane;

        //         // ROS_INFO("adding to tab point with x: %f y: %f : ", map_idx_x,map_idx_y);
        //         mapIndex2 = (int)(map_idx_y)*occupancyMap.info.width +(int)(map_idx_x);
        //         // ROS_INFO("%d : ", mapIndex);
        //         nn_srv.request.grid_map.push_back(occupancyMap.data[mapIndex2]);
        //     }
        // }




    // nn_srv.request.grid_map = occupancyMap.data;
    // ROS_INFO("client request: s1_x: %f", nn_srv.request.s_x);
    // ROS_INFO("client request: s1_y: %f", nn_srv.request.s_y);
    // ROS_INFO("client request: s1_yaw: %f", nn_srv.request.s_yaw);
    // ROS_INFO("client request: s2_x: %f", nn_srv.request.s2_x);
    // ROS_INFO("client request: s2_y: %f", nn_srv.request.s2_y);
    // ROS_INFO("client request: s2_yaw: %f", nn_srv.request.s2_yaw);

    // x_diff_lok = abs(x1_in_s0-x0_in_s0);
    // y_diff_lok = abs(y1_in_s0-y0_in_s0);


    // ROS_INFO("x_diff_lok : %f", x_diff_lok);
    // ROS_INFO("y_diff_lok: %f", y_diff_lok);


    // finding left_top_corner depending on position's points

    // if ((state1_coordX->values[0] > state0_coordX->values[0]) && (state1_coordY->values[0] > state0_coordY->values[0]))
    // {
    //     left_top_pic_x = (int)(state1_coordX->values[0] - 128.0);
    //     left_top_pic_y = (int)(state1_coordY->values[0]);
    // }
    // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] > state0_coordY->values[0]))
    // {
    //     left_top_pic_x = (int)(state1_coordX->values[0]);
    //     left_top_pic_y = (int)(state1_coordY->values[0]);
    // }
    // if ((state1_coordX->values[0] < state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
    // {
    //     left_top_pic_x = (int)(state1_coordX->values[0]);
    //     left_top_pic_y = (int)(state1_coordY->values[0] + 128.0);
    // }
    // if ((state1_coordX->values[0] > state0_coordX->values[0]) && (state1_coordY->values[0] < state0_coordY->values[0]))
    // {
    //     left_top_pic_x = (int)(state0_coordX->values[0]);
    //     left_top_pic_y = (int)(state0_coordY->values[0]);
    // }

    // call server only when diagonal is lower than 145.0 ( could be sqrt(128*128+128*128) but i decided to take less)

        // ROS_INFO("ODL EUKLIDESOWA W LOKALNYM UKLADZIE: %f", sqrt(x_diff_lok*x_diff_lok + y_diff_lok*y_diff_lok));
        // ROS_INFO("ODL EUKLIDESOWA W GLOBALNYM UKLADZIE: %f", sqrt((state1_coordX->values[0]-state0_coordX->values[0])*(state1_coordX->values[0]-state0_coordX->values[0]) + (state1_coordY->values[0]-state0_coordY->values[0])*(state1_coordY->values[0]-state0_coordY->values[0])));
        // ROS_INFO("wspolrzedna lokalna X punktu 1:  %f  ", x1_in_s0);
        // ROS_INFO("y  %f : ", left_top_pic_y);
        // std::cin.get();


    
    // ROS_INFO("HEJ: %ld", nn_srv.request.grid_map.size());
    //  nn_srv.request.grid_map = globalMap_copy;
    // std::cin.get();


    if ((*nn_client).call(nn_srv))
    {
        // ROS_INFO("SERVER RESPONSE:");
        int receiveed_shape = nn_srv.response.factors.size();

        // double a = 0.0;
        // double b = 0.0;
        // double c = 0.0;
        // double d = 0.0;
        // double e = 0.0;
        // double f = 0.0;

        for (int i=0; i < receiveed_shape/6;i++)
        {

            // condition no.1 - abs value of orientation in local state1 can't be greater as pi/2
            if (abs(th1_in_s0) >= M_PI/2)
            {
                // ROS_INFO("condition no. 1");
                return false;
            }


            // factors factor_reply;
            // factor_reply = calculate_factors(k_vall_st0,k_vall_st1,nn_srv.response.factors[i*6+2],
            //                                                        nn_srv.response.factors[i*6],
            //                                                        nn_srv.response.factors[i*6+1],
            //                                                        nn_srv.response.factors[i*6+5],
            //                                                        nn_srv.response.factors[i*6+3],
            //                                                        nn_srv.response.factors[i*6+4]);
 

            a = nn_srv.response.factors[i*6]/10000.0;
            b = nn_srv.response.factors[i*6+1]/10000.0;
            c = nn_srv.response.factors[i*6+2]/10000.0;
            d = nn_srv.response.factors[i*6+3]/10000.0;
            e = nn_srv.response.factors[i*6+4]/10000.0;
            f = nn_srv.response.factors[i*6+5]/10000.0;

            // ROS_INFO("a: %Lf", a);
            // ROS_INFO("b: %Lf", b);
            // ROS_INFO("c: %Lf", c);
            // ROS_INFO("d: %Lf", d);
            // ROS_INFO("e: %Lf", e);
            // ROS_INFO("f: %Lf", f);


            // condition no.2 - checking robot direction , if we comment line 370 it means that robot can't go backward
            if (x1_in_s0 < 0)
            {
                signn= -1.0;
                // return false;
            } 
            else signn= 1.0;

            // calculating value of polynomial function and changing local coordinate system to global one
            for (double i = 0.0; i <= abs(x1_in_s0); i=i+0.1)
            {

            tan_th_wielomian = tan(th1_in_s0);
            th_wielomian = atan(tan_th_wielomian);
            th_wielomian_global = th_wielomian;


            x_wielomian = x0_in_s0+i*signn;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;

            // ROS_INFO("x_wielomian: %f",x_wielomian);
            // ROS_INFO("y_wielomian: %f",y_wielomian);
            // ROS_INFO("th_wielomian: %f",th_wielomian);
            // ROS_INFO("---------------------------------------------");

            x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos(th_wielomian_global) + y_wielomian*sin(th_wielomian_global);
            y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos(th_wielomian_global);

            // condition no. 3 - values of global coordinates can't be greater than map size
            if ((x_wielomian_global >= occupancyMap.info.width || x_wielomian_global <= 0.0)||(y_wielomian_global >= occupancyMap.info.height || y_wielomian_global <= 0.0))
            { 
                is_available=false;
                // ROS_INFO("condition no. 3");
                break;
            }
            else 
            {
                is_available=true;
            }

            // ROS_INFO("x_wielomian_global: %f",x_wielomian_global);
            // ROS_INFO("y_wielomian_global: %f",y_wielomian_global);
            // ROS_INFO("th_wielomian_global: %f",th_wielomian_global);
            // ROS_INFO("state1 X: %f",state1_coordX->values[0]);
            // ROS_INFO("state1 Y: %f",state1_coordY->values[0]);
            // ROS_INFO("state1 TH: %f",state1_coordYaw->values[0]);

            mapIndex = (int)(y_wielomian_global)*occupancyMap.info.width +(int)(x_wielomian_global);
            // ROS_INFO("mapIndex: %d",mapIndex);
            occupancyMapValue = occupancyMap.data[mapIndex];
            // ROS_INFO("CZESC");
            // condition no. 4 - values of mapIndex can only be equal to 0 - free cells
            if (occupancyMapValue == 0)
            {
                is_available=true;
                // ROS_INFO("condition no. 4");
            }
            else 
            {
                is_available=false;
                break;
            }

            // maximal curve of polynomial function
            max_curve = tan(beta_defined)/W;//1 / (5.3 - 1.792 / 2); //tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
            // actual curve
            curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
            // condition no. 5 - actual curve cannot be greater than maximal curve
            if (curve<=max_curve) is_available=true;
            else 
            {
                is_available=false;
                // ROS_INFO("condition no. 5");
                break;
            }

    
            }

            if (is_available)
            {
                continue;// ROS_INFO("Motion Validator: True");
                //return true;
            }
            else 
            {
                break;
                // ROS_INFO("Motion Validator: False");
                //return false;
            }                             
            // a.push_back(nn_srv.response.factors[i*6]);
            // b.push_back(nn_srv.response.factors[i*6+1]);
            // c.push_back(nn_srv.response.factors[i*6+2]);
            // d.push_back(nn_srv.response.factors[i*6+3]);
            // e.push_back(nn_srv.response.factors[i*6+4]);
            // f.push_back(nn_srv.response.factors[i*6+5]);

            //ROS_INFO("server response: %d: %f",i, (float)nn_srv.response.factors[i]);
        }
        // ROS_INFO("Connection filter returning: %d ", is_available);
        // ROS_INFO("-----------------------------------");
        if (is_available)
        {
            return true;
        }
        else 
        {
            return false;
        }

        // ROS_INFO("#####################################:");

        // for (int i=0; i < a.size();i++)
        // {

        //     ROS_INFO("a: %d: %f",i, a[i]);
        //     ROS_INFO("b: %d: %f",i, b[i]);
        //     ROS_INFO("c: %d: %f",i, c[i]);
        //     ROS_INFO("d: %d: %f",i, d[i]);
        //     ROS_INFO("e: %d: %f",i, e[i]);
        //     ROS_INFO("f: %d: %f",i, f[i]);
        // }
        // std::cin.get();
        // ROS_INFO("server response: s1_x: %f", (float)nn_srv.response.factors[0]);
        // ROS_INFO("server response: s1_y: %f", (float)nn_srv.response.factors[1]);
        // ROS_INFO("server response: s1_yaw: %f", (float)nn_srv.response.factors[2]);
    }
    else
    {
        ROS_ERROR("Failed to call service local neural planner");
    }
    //return true;
    }
    else
    {
        return false;
    }



    
} // end constructor



// motion validator - checks if motion between 2 states is avaliable
bool myMotionValidator::checkMotion(const ob::State *s0, const ob::State *s1) const
{

    // polynomial factors
    a,b,c,d,e,f = 0.0;

    // curve values
    double max_curve = 0.0;
    double curve = 0.0;

    // global polynomial variables
    double x_wielomian_global = 0.0;
    double y_wielomian_global = 0.0;
    double th_wielomian_global = 0.0;

    // local polynomial variables
    double x_wielomian = 0.0;
    double y_wielomian = 0.0;
    double th_wielomian = 0.0;

    double tan_th_wielomian = 0.0;
    // variables for definig state1 in coordinates of state0
    double x1_in_s0,y1_in_s0,th1_in_s0;

    // map variables
    int mapIndex = 0;
    int occupancyMapValue = 0;

    // direction
    double signn = 1.0;

    // variable storing availability of state value
    bool is_available = true;

    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state0_coordX = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state0_coordY = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state0_coordYaw = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state1_coordX = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    // ROS_INFO("GOT 1X: %f",state0_coordX->values[0]);
    // ROS_INFO("GOT 1Y: %f",state0_coordY->values[0]);
    // ROS_INFO("GOT 1Yaw: %f",state0_coordYaw->values[0]);
    // ROS_INFO("GOT 2X: %f",state1_coordX->values[0]);
    // ROS_INFO("GOT 2Y: %f",state1_coordY->values[0]);
    // ROS_INFO("GOT 2Yaw: %f",state1_coordYaw->values[0]);

    // calculating state1 in state0 coordinates 
    th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
    y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
    // ROS_INFO("GOT 2X: %f",x1_in_s0);
    // ROS_INFO("GOT 2Y: %f",y1_in_s0);
    // ROS_INFO("GOT 2Yaw: %f",th1_in_s0);
    // uklad 0
    double x0_in_s0 = 0.0;
    double y0_in_s0 = 0.0;
    double th0_in_s0 = 0.0;

    // condition no.1 - abs value of orientation in local state1 can't be greater as pi/2
     if (abs(th1_in_s0) >= M_PI/2)
     {
        // ROS_INFO("condition no. 1");
        return false;
     }
    // calculating polynomial factors 

    factors factor_reply;
    factor_reply = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
    a = factor_reply.value1;
    b = factor_reply.value2;
    c = factor_reply.value3;
    d = factor_reply.value4;
    e = factor_reply.value5;
    f = factor_reply.value6;

    // condition no.2 - checking robot direction , if we comment line 370 it means that robot can't go backward
    if (x1_in_s0 < 0)
    {
        signn= -1.0;
        // return false;
    } 
    else signn= 1.0;

    // calculating value of polynomial function and changing local coordinate system to global one
    for (double i = 0.0; i <= abs(x1_in_s0); i=i+0.1)
    {
            tan_th_wielomian = tan(th1_in_s0);
            th_wielomian = atan(tan_th_wielomian);
            th_wielomian_global = th_wielomian; //state0_coordYaw->values[0] - th_wielomian;


            x_wielomian = x0_in_s0+i*signn;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
            
            x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos(th_wielomian_global) + y_wielomian*sin(th_wielomian_global);
            y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos(th_wielomian_global);


            // ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
            // ompl::base::ScopedState<ompl::base::SE2StateSpace> state(space);
            // state->setX(x_wielomian_global);
            // state->setY(y_wielomian_global);
            // state->setYaw(th_wielomian_global);
            

            // // ompl::base::ScopedState<> backup = state;
            // // // backup maintains its internal state as State*, so setX() is not available.
            // // // the content of backup is copied from state
            
            // ompl::base::State *abstractState = space->allocState();
            
            
            // // // this will copy the content of abstractState to state and
            // // // cast it internally as ompl::base::SE2StateSpace::StateType
            // state = abstractState;
            // const ob::RealVectorStateSpace::StateType *state0_coordX = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
            
            is_available = isStateValid2(x_wielomian_global,y_wielomian_global, th_wielomian_global);
            if (is_available)
            {
                // ROS_INFO("Motion Validator: True");
                // return true;
                continue;
            }
            else 
            {
                // ROS_INFO("Motion Validator: False");
                // return false;
                break;
            }
            // ROS_INFO("%d",is_available);
            //std::cin.get();

        // condition no. 3 - values of global coordinates can't be greater than map size
        if ((x_wielomian_global >= occupancyMap.info.width || x_wielomian_global <= 0.0)||(y_wielomian_global >= occupancyMap.info.height || y_wielomian_global <= 0.0))
        { 
            is_available=false;
            // ROS_INFO("condition no. 3");
            break;
        }
        else 
        {
            is_available=true;
        }


        mapIndex = (int)(y_wielomian_global)*occupancyMap.info.width +(int)(x_wielomian_global);
        occupancyMapValue = occupancyMap.data[mapIndex];
        // ROS_INFO("occ map val: %d",occupancyMapValue);
        // condition no. 4 - values of mapIndex can only be equal to 0 - free cells
        if (occupancyMapValue == 0)
            {
                is_available=true;
                // ROS_INFO("condition no. 4");
            }
        else 
            {
                is_available=false;
                break;
            }


        // maximal curve of polynomial function
        max_curve = tan(beta_defined)/W;//1 / (5.3 - 1.792 / 2);//tan(beta_defined)/W;//1 / (5.3 - 1.792 / 2); //tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
        // actual curve
        curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
        // condition no. 5 - actual curve cannot be greater than maximal curve
        if (curve<=max_curve) is_available=true;
        else 
        {
            is_available=false;
            // ROS_INFO("condition no. 5");
            break;
        }

        // saving data to file
        // myfile2.open("dane_global9.txt",ofstream::app);
        // myfile2 << x_wielomian_global <<","<<y_wielomian_global<<","<<th_wielomian_global<<std::endl;
        // myfile2.close();

    } 
    // ROS_INFO("##############################");
    if (is_available)
    {
        // ROS_INFO("Motion Validator: True");
        return true;
    }
    else 
    {
        // ROS_INFO("Motion Validator: False");
        return false;
    }
    
} // end myMotionValidator





// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){

    // variables similar to the ones in myMotionValidator - so i dont put many comments

    // state 0 and state 1 coordinates
    double th1_in_s0,x1_in_s0,y1_in_s0;
    double th0_in_s0,x0_in_s0,y0_in_s0;

    double signn;

    double x_wielomian,y_wielomian,x_wielomian_global,y_wielomian_global,tan_th_wielomian,th_wielomian,th_wielomian_global;

    double curve = 0.0;
    double max_curve = 0.0;


    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "map";
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
    path->print(std::cout);
    
    // convert to geometric path
    og::PathGeometric *path2 = path.get()->as<og::PathGeometric>();
    // get path length
    path_length = path2->getStateCount();

    // iterate over each position
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


        th0_in_s0 = 0.0;
        x0_in_s0 = 0.0;
        y0_in_s0 = 0.0;

        th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
        x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
        y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);


        double k_vall_st0 = 0.0;
        double k_vall_st1 = 0.0;
        factors factor_reply;
        factor_reply = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
        a = factor_reply.value1;
        b = factor_reply.value2;
        c = factor_reply.value3;
        d = factor_reply.value4;
        e = factor_reply.value5;
        f = factor_reply.value6;


        if (abs(th1_in_s0) >= M_PI/2)
        {
            continue;
        }

        if (x1_in_s0 < 0)
        {
            signn= -1.0;
            //continue;
        }
        else signn= 1.0;

        for (double i = 0.1; i <= abs(x1_in_s0); i=i+0.2)
        {
            x_wielomian = x0_in_s0+i*signn;

            tan_th_wielomian = tan(th1_in_s0);
            th_wielomian = atan(tan_th_wielomian);
            th_wielomian_global = th_wielomian; //state0_coordYaw->values[0] - th_wielomian;



            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
            x_wielomian_global = state0_coordX->values[0] + x_wielomian*cos(th_wielomian_global) + y_wielomian*sin(th_wielomian_global);
            y_wielomian_global = state0_coordY->values[0] + x_wielomian*sin((-1.0)*th_wielomian_global) + y_wielomian*cos(th_wielomian_global);



            if ((x_wielomian_global >= occupancyMap.info.width || x_wielomian_global <= 0.0)||(y_wielomian_global >= occupancyMap.info.height || y_wielomian_global <= 0.0)) {
                break;
            }

            max_curve = tan(beta_defined)/W;//1 / (5.3 - 1.792 / 2);//tan(beta_defined)/W;// 1 / (5.3 - 1.792 / 2); //tan(beta_defined)/robot_height;
            curve = abs(20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d)/(pow(1+tan_th_wielomian*tan_th_wielomian,(3/2)));
            if (curve>max_curve)
            {
                break;
            }

        // make quaternion
        tf::Quaternion q_path;
        q_path.setRPY(0.0,0.0,th_wielomian_global);
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = x_wielomian_global;
        poseMsg.pose.position.y = y_wielomian_global;
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = q_path[3];
        poseMsg.pose.orientation.x = q_path[0];
        poseMsg.pose.orientation.y = q_path[1];
        poseMsg.pose.orientation.z = q_path[2];
        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = ros::Time::now();
        plannedPath.poses.push_back(poseMsg);

        if (((x_wielomian_global <= goal_x + 0.15) && (x_wielomian_global >= goal_x - 0.15)) && ((y_wielomian_global <= goal_y + 0.15) && (y_wielomian_global >= goal_y - 0.15)) && ((th_wielomian_global <= yaw_goal + 0.05) && (th_wielomian_global >= yaw_goal - 0.05))) 
        {
            ROS_INFO("Robot achieved goal position");
            break;
        }
        }
    }
    return plannedPath;

} // end extract path


// ob::OptimizationObjectivePtr Planner2D::getOptimizationCost(const ob::State *s0, const ob::State *s1)
// {
// 	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
// 	// obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
// 	return obj;
// }

// funkcja do scalkowania
double f1(double x, double aa, double bb, double cc, double dd, double ee) 
{ 
    return 5*aa*pow(x,4) + 4*bb*pow(x,3) + 3*cc*pow(x,2) + 2*dd*x + ee;
}


ob::Cost myOptimizer::motionCost(const ob::State *s0, const ob::State *s1) const
{
    //ROS_INFO("OPTIMIZER");

    // polynomial factors
    a,b,c,d,e,f = 0.0;

    // curve values
    double max_curve = 0.0;
    double curve = 0.0;

    // global polynomial variables
    double x_wielomian_global = 0.0;
    double y_wielomian_global = 0.0;
    double th_wielomian_global = 0.0;

    // local polynomial variables
    double x_wielomian = 0.0;
    double y_wielomian = 0.0;
    double th_wielomian = 0.0;
    double tan_th_wielomian = 0.0;

    // variables for definig state1 in coordinates of state0
    double x1_in_s0,y1_in_s0,th1_in_s0;

    // map variables
    int mapIndex = 0;
    int occupancyMapValue = 0;

    // direction
    double signn = 1.0;

    // variable storing availability of state value
    bool is_available = true;

    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state0_coordX = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state0_coordY = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state0_coordYaw = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state1_coordX = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    // calculating state1 in state0 coordinates 
    th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
    y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);
    // ROS_INFO("GOT 2X: %f",x1_in_s0);
    // ROS_INFO("GOT 2Y: %f",y1_in_s0);
    // ROS_INFO("GOT 2Yaw: %f",th1_in_s0);
    // uklad 0
    double x0_in_s0 = 0.0;
    double y0_in_s0 = 0.0;
    double th0_in_s0 = 0.0;

    // condition no.1 - abs value of orientation in local state1 can't be greater as pi/2
     if (abs(th1_in_s0) >= M_PI/2)
     {
        // ROS_INFO("condition no. 1");
        ob::Cost obc2(9999999999.0);
        return obc2;
     }
    // calculating polynomial factors 

    factors factor_reply;
    factor_reply = calculate_factors(k_vall_st0,k_vall_st1,th0_in_s0,x0_in_s0,y0_in_s0,th1_in_s0,x1_in_s0,y1_in_s0);
    a = factor_reply.value1;
    b = factor_reply.value2;
    c = factor_reply.value3;
    d = factor_reply.value4;
    e = factor_reply.value5;
    f = factor_reply.value6;

    ROS_INFO("################# CALKOWANIE #####################");

    ROS_INFO("X0_in_s0: %f",x0_in_s0);
    ROS_INFO("X1_in_s0: %f",x1_in_s0);


    // zmienne do calkowania
    double xp, xk, h, calka;
    int n;

    n = 10000;

    if (x1_in_s0 > x0_in_s0)
    {
        xp = x0_in_s0;
        xk = x1_in_s0;
        h = (xk - xp) / (double)n;
        ROS_INFO("krok calkowania1: %f",h);
    }
    else
    {
        xp = x1_in_s0;
        xk = x0_in_s0;
        h = (xp - xk) / (double)n;
        ROS_INFO("krok calkowania2: %f",h);
    }

    

    // h = (xk - xp) / (double)n;
    // ROS_INFO("krok calkowania: %f",h);

    calka = 0;
    for (int i=1; i<n; i++)
    {
        calka += sqrt(1+pow(f1(xp + i * h,a,b,c,d,e),2));
    }
    calka += sqrt(1+pow(f1(xp,a,b,c,d,e),2)) / 2;
    calka += sqrt(1+pow(f1(xk,a,b,c,d,e),2)) / 2;
    calka *= h;

    ROS_INFO("Wynik calkowania: %f",calka);
    ob::Cost obc(abs(calka));
    ROS_INFO("OPTIMIZER %f #############",obc.value());

    return obc;
}

ob::Cost myOptimizer::stateCost(const ob::State *s3) const
{

    ob::Cost obc(5.5);
    //ROS_INFO("OPTIMIZER %f #############",obc.value());
    return obc;
}

// plan path
nav_msgs::Path Planner2D::planPath(const nav_msgs::OccupancyGrid& globalMap,const visualization_msgs::Marker &st_pt,const visualization_msgs::Marker &gl_pt)
{
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
    // set State Validity Checking Resolution
    // si->setStateValidityCheckingResolution(0.5);

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

    ROS_INFO("Start state: ");
    start.print(std::cout);
    ROS_INFO("Goal state: ");
    goal.print(std::cout);

    // Create an instance of ompl::base::ProblemDefinition
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));



    // Set the start and goal states for the problem definition. 
    pdef->setStartAndGoalStates(start, goal);

    // Setting optimizer -> to be done;
    ob::OptimizationObjectivePtr opti(new myOptimizer(si));
    pdef->setOptimizationObjective(opti);
    
    // Create an instance of a planner 
    my_planner* planner2 = new my_planner(si);
    planner2->setRange(10.0);

    // own connection filter class
    class_cf cf_planner(planner2);
    std::function<bool (const Vertex &, const Vertex &)> myConnectionFilter = cf_planner;
    planner2->setConnectionFilter(myConnectionFilter);

    // Tell the planner which problem we are interested in solving 
    planner2->setProblemDefinition(pdef);

    // Make sure all the settings for the space and planner are in order. 
    // This will also lead to the runtime computation of the state validity checking resolution. 
    planner2->setup();
    // close file with data
    myfile2.close();
 
    // solve motion planning problem
    ob::PlannerStatus solved = planner2->ob::Planner::solve(180.0);

    // if solved == true, a solution was found
    nav_msgs::Path plannedPath;
    
    if (solved) 
    {
        std::cout<<"Found path from start state to goal state";
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
} // end of plan path function



