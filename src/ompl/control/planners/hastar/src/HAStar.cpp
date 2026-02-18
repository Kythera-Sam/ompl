
#include "ompl/control/planners/hastar/HAStar.hpp"

#include <unordered_map>
#include <unordered_set>
#include <functional>

#include <boost/heap/fibonacci_heap.hpp>
#include "ompl/base/spaces/SE2StateSpace.h"

namespace HybridAStar
{
    using State = ompl::base::SE2StateSpace::StateType;

    struct Node
    {
        using openSet_t = typename boost::heap::fibonacci_heap<Node>;
        using heapHandle_t = typename openSet_t::handle_type;

        heapHandle_t handle;
    };

    struct PrecursorState
    {
        State state;
    };

    using openSet_t = Node::openSet_t;
    using heapHandle_t = Node::heapHandle_t;

}  // namespace HybridAStar

namespace std
{
    template <>
    struct hash<HybridAStar::State>
    {
        size_t operator()(const HybridAStar::State &state)
        {
            return 0;
        }
    };
}  // namespace std

void ompl::control::HAStar::setup()
{
    base::Planner::setup();
}

void ompl::control::HAStar::clear()
{
    base::Planner::clear();
}

ompl::base::PlannerStatus ompl::control::HAStar::solve(const base::PlannerTerminationCondition &ptc)
{

  const HybridAStar::State* start = pdef_->getStartState(0);
  base::Goal* goal = pdef_->getGoal().get();

    HybridAStar::openSet_t openSet;

    std::unordered_map<uint64_t, HybridAStar::heapHandle_t> stateToHeap;
    std::unordered_set<uint64_t> closedSet;
    std::unordered_map<HybridAStar::State, HybridAStar::PrecursorState, std::hash<HybridAStar::State>> precursors;

    return base::PlannerStatus::INVALID_START;
}
