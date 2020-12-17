#include "../include/pracownia_badawcza/ompl_lib.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <tf/tf.h>
#include <math.h>
#include <cmath>
#include "ompl/base/MotionValidator.h"
# define M_PI           3.14159265358979323846  /* pi */

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace map_node {
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

std::vector< double > top;
std::vector< double > bottom;
std::vector< double > dist;

double a,b,c,d,e,f; //  wspolczynniki wielomianu
double g0,g1,v,j,jv,k_val,m,k,p;

// constructor in which we create our space
Planner2D::Planner2D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
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


bool myMotionValidator::checkMotion(const ob::State *s0, const ob::State *s1) const
{
    a=0.0;
    b=0.0;
    c=0.0;
    d=0.0;
    e=0.0;
    f=0.0;
    double beta_defined = sqrt(3.0);
    double beta = 0.0;
    double x_wielomian_global = 0.0;
    double y_wielomian_global = 0.0;
    double th_wielomian_global = 0.0;
    double x_wielomian = 0.0;
    double y_wielomian = 0.0;
    double th_wielomian = 0.0;
    double x1_in_s0 = 0.0;
    double y1_in_s0 = 0.0;
    double th1_in_s0 = 0.0;
    double x0_in_s0 = 0.0;
    double y0_in_s0 = 0.0;
    double tan_th_wielomian = 0.0;
    int mapIndex = 0;
    int occupancyMapValue = 0;
    bool is_available = true;
    // get coord of the first state
    const ob::RealVectorStateSpace::StateType *state1_coordX = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state1_coordY = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state1_coordYaw = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    // get coord of the second state
    const ob::RealVectorStateSpace::StateType *state0_coordX = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *state0_coordY = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const ob::RealVectorStateSpace::StateType *state0_coordYaw = s0->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
    std::cout<<"#################################################################";

    // wyrazenie punktu 1 w ukladzie punktu 0
    th1_in_s0 = state0_coordYaw->values[0] - state1_coordYaw->values[0];
    x1_in_s0 = state1_coordX->values[0]*cos(th1_in_s0) - state1_coordY->values[0]*sin(th1_in_s0);
    y1_in_s0 = state1_coordX->values[0]*sin(th1_in_s0) - state1_coordY->values[0]*cos(th1_in_s0);
    ROS_INFO("th1_in_s0: %f",th1_in_s0);
    ROS_INFO("x1_in_s0: %f",x1_in_s0);
    ROS_INFO("y1_in_s0: %f",y1_in_s0);
    // consts
    k_val = 1.0;
    g0=k_val*(pow((1+tan(state0_coordYaw->values[0])*tan(state0_coordYaw->values[0])),(3/2)));
    g1=k_val*(pow((1+tan(state1_coordYaw->values[0])*tan(state1_coordYaw->values[0])),(3/2)));
    m = (g0/2)*pow(x1_in_s0,2) + tan(state0_coordYaw->values[0])*x1_in_s0 - y1_in_s0;
    k = g0*x1_in_s0 + tan(state0_coordYaw->values[0]) - tan(state1_coordYaw->values[0]);
    p = g0 - g1;
    v = 6*m+pow(x1_in_s0,2)*p;
    j = 2*k + x1_in_s0*p;
    jv = j*(((-1)*18)/20)*x1_in_s0 + v;

    // rownania
    f = y0_in_s0;
    e = tan(state0_coordYaw->values[0]);
    d = g0/2;
    a = (jv)/(pow(x1_in_s0,5));
    b = (((-1.5)*x1_in_s0)*(1/pow(x1_in_s0,4)))*jv + (j/((-1)*20*pow(x1_in_s0,3)));
    c = (-1)*((p+a*20.0*pow(x1_in_s0,3)+b*12.0*pow(x1_in_s0,2))/(6*x1_in_s0));
    
    ROS_INFO("a: %f",a);
    ROS_INFO("b: %f",b);
    ROS_INFO("c: %f",c);
    ROS_INFO("d: %f",d);
    ROS_INFO("e: %f",e);
    ROS_INFO("f: %f",f);
    
    // wartosc funkcji wielomianowej licze dla wspolrzednych ukladu lokalnego  i zamieniam spowrotem na uklad globalny
    for (double i = x0_in_s0; i < abs(x1_in_s0); i=i+0.1)
    {
        ROS_INFO("i: %f",i);
        if (x1_in_s0 > x0_in_s0)
        {
            x_wielomian = x0_in_s0+i;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
        }
        else
        {
            x_wielomian = x1_in_s0+i;
            y_wielomian = a*pow(x_wielomian,5)+b*pow(x_wielomian,4)+c*pow(x_wielomian,3)+d*pow(x_wielomian,2)+e*x_wielomian + f;
        }
        ROS_INFO("elo");
        // zamiana na wartosci globalne
        ROS_INFO("x_wielomian: %f",x_wielomian);
        ROS_INFO("y_wielomian: %f",y_wielomian);
        ROS_INFO("th1_in_s0: %f",th1_in_s0);
        x_wielomian_global = x_wielomian*cos((-1)*th1_in_s0) - y_wielomian*sin((-1)*th1_in_s0);
        y_wielomian_global = x_wielomian*sin((-1)*th1_in_s0) + y_wielomian*cos((-1)*th1_in_s0);
        tan_th_wielomian = 5*a*pow(x_wielomian,4)+4*b*pow(x_wielomian,3)+3*c*pow(x_wielomian,2),2*d*x_wielomian+e;
        th_wielomian = atan2(1.0,tan_th_wielomian);
        th_wielomian_global = state0_coordYaw->values[0] + th_wielomian;
        ROS_INFO("elo2");
        // spr warunkow
        ROS_INFO("y_wielomian_global: %f",y_wielomian_global);
        ROS_INFO("x_wielomian_global: %f",x_wielomian_global);
        mapIndex = (int)(y_wielomian_global)*occupancyMap.info.width +(int)(x_wielomian_global);
        occupancyMapValue = occupancyMap.data[mapIndex];
        ROS_INFO("mapIndex: %d",mapIndex);
        ROS_INFO("occupancyMapValue: %d",occupancyMapValue);
        if (occupancyMapValue == 100)
            {
                is_available=false;
                break;
            }
            else is_available=true;
        
        
        beta = atan2(pow(1+tan(th_wielomian)*tan(th_wielomian),(3/2)),20.0*a*pow(x_wielomian,3)+12.0*b*pow(x_wielomian,2)+6.0*c*x_wielomian+2.0*d);
        if (beta<=beta_defined) return true;
        else return false;
    }
    if (is_available) return true;
    else return false;
     
}



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
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.02);
    // Set the motion validator
    ob::MotionValidatorPtr mv(new myMotionValidator(si));
    si->setMotionValidator(mv);
    si->setup();
    std::cout<<"############################";

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
    ob::PlannerPtr planner(new og::LazyPRMstar(si));
    // Tell the planner which problem we are interested in solving 
    planner->setProblemDefinition(pdef);
    // Make sure all the settings for the space and planner are in order. 
    // This will also lead to the runtime computation of the state validity checking resolution. 
    planner->setup();

    // solve motion planning problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
    // if solved == true, a solution was found
    nav_msgs::Path plannedPath;
    if (solved) 
    {
        std::cout<<"SOLVEEEEEEEED";
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
}

} 

