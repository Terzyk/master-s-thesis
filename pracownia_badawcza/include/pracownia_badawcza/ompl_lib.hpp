#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include <ompl/base/PlannerData.h>
#include <vector>
#include <ompl/tools/config/SelfConfig.h>
#include "pracownia_badawcza/LNP.h"
#include <cstdlib>

namespace ob = ompl::base;

// 2D planner example class
class Planner2D
{
public:
    ros::ServiceClient client;
    
    // Constructor
    Planner2D(ros::ServiceClient& my_client);

 
    // Destructor.
    virtual ~Planner2D();


    // plan path
    nav_msgs::Path planPath(const nav_msgs::OccupancyGrid& globalMap,const visualization_msgs::Marker& st_pt,const visualization_msgs::Marker& gl_pt);
    
    // move robot
    void robot_move(nav_msgs::Path path);
    
    // extract path
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

class myMotionValidator : public ob::MotionValidator
{
public:
    myMotionValidator(ob::SpaceInformationPtr si) : MotionValidator(si)
    {

    }
    // implement checkMotion()
    bool checkMotion(const ob::State *s1, const ob::State *s2) const override;
    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;
};




