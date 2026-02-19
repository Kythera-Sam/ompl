
#include "ompl/control/planners/hastar/HAStar.hpp"

#include <unordered_map>
#include <unordered_set>
#include <functional>

#include <boost/heap/fibonacci_heap.hpp>
#include "ompl/base/spaces/SE2StateSpace.h"

namespace HybridAStar
{
    using State = ompl::base::SE2StateSpace::StateType;

    using Action = int;
    using Cost = double;

    struct Motion
    {
        Motion Clone() const
        {
            return Motion{.state = state, .action = action, .cost = cost};
        }

        State state;
        Action action;
        Cost cost;
    };

    struct Node
    {
        using openSet_t = typename boost::heap::fibonacci_heap<Node>;
        using heapHandle_t = typename openSet_t::handle_type;

        Node Clone() const
        {
            return Node{.motion = motion.Clone(), .heuristic = heuristic, .handle = handle};
        }

        Motion motion;
        Cost heuristic;
        heapHandle_t handle;
    };

    struct PrecursorState
    {
        Motion motion;
        Cost heuristic;
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

bool IsSolution(const HybridAStar::Node &node)
{
    return false;
}

uint64_t HashState(const HybridAStar::State &state)
{
    return 0;
}

std::vector<HybridAStar::Motion> GetNeighbours(const HybridAStar::Node &node)
{
    return {};
}

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
    using State = HybridAStar::State;
    using Node = HybridAStar::Node;
    using Motion = HybridAStar::Motion;
    using Neighbour = HybridAStar::Motion;

    base::Goal *goal = pdef_->getGoal().get();

    HybridAStar::openSet_t openSet;

    std::unordered_map<uint64_t, HybridAStar::heapHandle_t> stateToHeap;
    std::unordered_set<uint64_t> closedSet;
    std::unordered_map<State, HybridAStar::PrecursorState, std::hash<State>> precursors;

    while (const base::State *it = pis_.nextStart())
    {
        auto handle = openSet.push(HybridAStar::Node());
        (*handle).handle = handle;
    }

    std::vector<Neighbour> neighbours;
    neighbours.reserve(10);

    while (!openSet.empty())
    {
        const Node &amortizedCurrent = openSet.top();
        if (IsSolution(amortizedCurrent))
        {
            // Walk back to the start, pushing to solution.
            // reverse ect.
            return base::PlannerStatus::EXACT_SOLUTION;
        }

        Node current = amortizedCurrent;
        openSet.pop();
        const uint64_t currentHash = HashState(current.state);
        stateToHeap.erase(currentHash);
        closedSet.insert(currentHash);

        neighbours = GetNeighbours(current);

        for (const auto &neighbour : neighbours)
        {
            const uint64_t neighbourHash = HashState(neighbour.state);
            if (closedSet.find(neighbourHash) != closedSet.end())
            {
                continue;
            }
        }
    }

    return base::PlannerStatus::INVALID_START;
}
