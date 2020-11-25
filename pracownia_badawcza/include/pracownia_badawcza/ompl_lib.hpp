#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>


namespace ob = ompl::base;
namespace map_node {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    Planner2D(ros::NodeHandle& _nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
    nav_msgs::Path planPath(const nav_msgs::OccupancyGrid& globalMap,const visualization_msgs::Marker& st_pt,const visualization_msgs::Marker& gl_pt);
    /*!
   * move robot
   */
    void robot_move(nav_msgs::Path path);


private:
    /// node handle
    ros::NodeHandle& nodeHandle;

    /// extract path
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
