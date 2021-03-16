#include "ros/ros.h"
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <boost/graph/adjacency_list.hpp>
#include <sstream>
#include <ompl/geometric/planners/prm/PRM.h>

using Vertex = boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;


namespace ob = ompl::base;
namespace og = ompl::geometric;

class my_planner : public ompl::geometric::LazyPRMstar
{
    public: 

        my_planner(ob::SpaceInformationPtr si) : LazyPRMstar(si)
            {

            }

        ompl::base::State* getStateProperty(const Vertex v1) const
        {
            return this -> stateProperty_[v1];
        }
};


class class_cf
{
    public:

        const ob::State *s1;
        const ob::State *s2;
        bool wynik;

        class_cf(const my_planner* cf_planner) : pb_planner_(cf_planner) {}

        bool operator() (const Vertex &vertex1, const Vertex &vertex2)
        {
            boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::type my_stateProperty;
            s1 = pb_planner_->getStateProperty(vertex1);
            s2 = pb_planner_->getStateProperty(vertex2);
            
            wynik = cf_client(s1,s2);
            return true;
        }

        bool cf_client(const ob::State *state1, const ob::State *state2);
        
    private:
        const my_planner* pb_planner_;

};
