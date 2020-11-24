#include "../include/pracownia_badawcza/ompl_lib.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <tf/tf.h>
#include <math.h>
#include <cmath>
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
        mapIndex = (int)((left_top_y_angled+bottom[i]))*occupancyMap.info.width+left_top_x_angled;
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100)
        {
            temp=true;
            break;
        }
        else temp=false;
        mapIndex = (int)((right_down_y_angled-bottom[i]))*occupancyMap.info.width+right_down_x_angled;
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
        mapIndex = (left_top_y_angled)*occupancyMap.info.width+(int)(left_top_x_angled-top[i]);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100){
            temp=true;
            break;
            }
            else temp=false;
        mapIndex = (right_down_y_angled)*occupancyMap.info.width+(int)(right_down_x_angled+top[i]);
        occupancyMapValue = occupancyMap.data[mapIndex];
        if (occupancyMapValue == 100){
            temp=true;
            break;
            }
            else temp=false;
    }
    if(temp==true) return false;
    else return true;
   
 /*
    for(int x=(-1)*robot_width; x<robot_width; x++)
    {
        for(int y=(-1)*robot_height; y<robot_height; y++)
        {
            int indeks = (y_pos+y)*occupancyMap.info.width+x_pos+x;
            int value = occupancyMap.data[indeks];
            ROS_INFO("occupancyMap.info.width: %d",occupancyMap.info.width);
            ROS_INFO("y: %d",(y_pos+y));
            ROS_INFO("x: %d",x_pos+x);
            ROS_INFO("mapIndex1: %d",indeks);
            ROS_INFO("occupancyMapValue1: %d",value);
            if (value == 100){
            temp=true;
            break;
            }
            else temp=false;
        }
    }
    if(temp==true) return false;
    else return true;
    */
    }
}

// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){
    ROS_INFO("#######################################################################");
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
    tf::Quaternion q_goal(gl_pt.pose.orientation.x,gl_pt.pose.orientation.y,gl_pt.pose.orientation.z,gl_pt.pose.orientation.w);
    tf::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
    /*
    ROS_INFO("x_goal: %f",gl_pt.pose.orientation.x);
    ROS_INFO("y_goal: %f",gl_pt.pose.orientation.y);
    ROS_INFO("z_goal: %f",gl_pt.pose.orientation.z);
    ROS_INFO("w_goal: %f",gl_pt.pose.orientation.w);
    */
    start_x = st_pt.pose.position.x;
    start_y = st_pt.pose.position.y;
    tf::Quaternion q_start(st_pt.pose.orientation.x,st_pt.pose.orientation.y,st_pt.pose.orientation.z,st_pt.pose.orientation.w);
    tf::Matrix3x3(q_start).getRPY(roll_start, pitch_start, yaw_start);
    /*
    ROS_INFO("x_start: %f",st_pt.pose.orientation.x);
    ROS_INFO("y_start: %f",st_pt.pose.orientation.y);
    ROS_INFO("z_start: %f",st_pt.pose.orientation.z);
    ROS_INFO("w_start: %f",st_pt.pose.orientation.w);

    ROS_INFO("roll_start: %f",roll_start);
    ROS_INFO("pitch_start: %f",pitch_start);
    ROS_INFO("yaw_start: %f",yaw_start);
    ROS_INFO("roll_goal: %f",roll_goal);
    ROS_INFO("pitch_goal: %f",pitch_goal);
    ROS_INFO("yaw_goal: %f",yaw_goal);
    */
    // Create an instance of ompl::base::SpaceInformation for the state space 
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    /*
    ROS_INFO("space_dim: %d",space->getDimension());
    ROS_INFO("space_dim: %d",si->getStateDimension());
    */
    // Set the state validity checker 
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.02);
    // create start and goal states
    std::vector<double> start_vector = {start_x,start_y,yaw_start};
    std::vector<double> goal_vector = {goal_x,goal_y,yaw_goal};
    ob::ScopedState<>start(space);
    start=start_vector;
    ob::ScopedState<>goal(space);
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
        //std::cout<<"SOLVEEEEEEEED";
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
}




} 

