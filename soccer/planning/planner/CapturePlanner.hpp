#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"
/*
 * manages SettlePathPlanner and CollectPathPlanner and calls them when necessary
 * todo(Ethan) make this use a CaptureCommand -> Robot should only issue Capture Commands---not Settle/Collect/Intercept/...etc
 */
namespace Planning {
    class CapturePlanner: public Planner {
    public:
        void plan(PlanRequest &&request) override;
        bool isApplicable(const MotionCommand& command) const {
            return std::holds_alternative<SettleCommand>(command) || std::holds_alternative<CollectCommand>(command);
        }

    private:
    };
}