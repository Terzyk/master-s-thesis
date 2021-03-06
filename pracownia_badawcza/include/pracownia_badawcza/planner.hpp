#include "ros/ros.h"
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <sstream>

class my_planner : public ompl::geometric::LazyPRMstar
{
    public: 
        my_planner(ob::SpaceInformationPtr si) : LazyPRMstar(si)
            {

            }

        // boost::property_map<Graph, vertex_state_t>::type getStateProperty()
        // {
        //     return this -> stateProperty_;
        // }
        ompl::base::State* getStateProperty(const Vertex v1)
        {
            return this -> stateProperty_[v1];
        }
};

