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


// ---------------- namespaces ----------------
using namespace std;
using namespace ros;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// ---------------- variables ----------------

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

// occupancy map used for planning
nav_msgs::OccupancyGrid occupancyMap;

// robot size
double robot_width = 4.0;
double robot_height = 1.2;

// max beta value
double beta_defined = M_PI/3;

// path length
int path_length;

// first scan - helps with picking good path in motion_validator
int first_scan = 0;

// vectors that store values that are helpful with checking robots shape while isStateValid function
std::vector< double > top;
std::vector< double > bottom;
std::vector< double > dist;

// polynomial factors
double a,b,c,d,e,f; 
struct factors
{
    double value1;
    double value2;
    double value3;
    double value4;
    double value5;
    double value6;
};

// constructor in which we create our space
Planner2D::Planner2D(ros::ServiceClient& my_client)
{
    // getting client info from map_node file, where we have access to NodeHandle
    nn_client = &my_client;
   
    // checking points along robot's width
    for (double i=0.2; i<=robot_width ;i=i+0.2)
    {
        top.push_back(i);
    }
    top.push_back(4.0);

    // checking points along robot's height
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
    int x_pos = (coordX->values[0]-occupancyMap.info.origin.position.x)/occupancyMap.info.resolution;
    int y_pos = (coordY->values[0]-occupancyMap.info.origin.position.y)/occupancyMap.info.resolution;
    int yaw = coordYaw->values[0];

    // help variable for storing value that tell us, if robot state is valid
    bool temp=true;

    // get robot edges coordinates
    double left_top_x_angled = x_pos + (robot_width/2.0)*cos(yaw);
    double left_top_y_angled = y_pos + (robot_height/2.0)*cos(yaw);
    double right_down_x_angled = x_pos - (robot_width/2.0)*cos(yaw);
    double right_down_y_angled = y_pos - (robot_height/2.0)*cos(yaw);

    // map variables
    int mapIndex = 0;
    int occupancyMapValue = 0;

    // checking shorter robot side
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

    // checking longer robot side
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

    // store values to structure and return it
    factors result = {aa,bb,cc,dd,ee,ff};
    return result;
} // end calculate factors

// overload function for class that is responsible for communicating with local neural planner
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

    // create service 
    pracownia_badawcza::LNP nn_srv;
    
    // store proper values
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

    // call server
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
        ROS_ERROR("Failed to call service local neural planner");
    }


    return true;
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

    // calculating state1 in state0 coordinates 
    th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    x1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*cos(th1_in_s0) - (state1_coordY->values[0]-state0_coordY->values[0])*sin(th1_in_s0);
    y1_in_s0 = (state1_coordX->values[0]-state0_coordX->values[0])*sin(th1_in_s0) + (state1_coordY->values[0]-state0_coordY->values[0])*cos(th1_in_s0);

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


        mapIndex = (int)(y_wielomian_global)*occupancyMap.info.width +(int)(x_wielomian_global);
        occupancyMapValue = occupancyMap.data[mapIndex];

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
        max_curve = tan(beta_defined)/robot_height; // zmienic nazwy robot_height z robot_width
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

        for (double i = 0.1; i <= abs(x1_in_s0); i=i+0.5)
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

            max_curve = tan(beta_defined)/robot_height;
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
    ob::PlannerStatus solved = planner2->ob::Planner::solve(20.0);

    // if solved == true, a solution was found
    nav_msgs::Path plannedPath;
    
    if (solved) 
    {
        std::cout<<"Found path from start state to goal state";
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
} // end of plan path function



