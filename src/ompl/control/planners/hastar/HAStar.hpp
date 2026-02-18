

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/DirectedControlSampler.h"


namespace ompl
{
    namespace control
    {
        class HAStar : public base::Planner
        {

        public:
            HAStar(const SpaceInformationPtr &si) : ompl::base::Planner(si, "Hybrid A*")
            {
            }

            void setup() override;
            void clear() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        private:
        };
    }  // namespace control
}  // namespace ompl
