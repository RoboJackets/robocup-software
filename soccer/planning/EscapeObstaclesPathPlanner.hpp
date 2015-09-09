#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>

namespace Planning {

class EscapeObstaclesPathPlanner : public SingleRobotPathPlanner {
public:
    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, MotionCommand cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

    /// The MotionCommand type that this planner handles
    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::None;
    }

    /// TODO: refactor this into two methods - one applies the filtering
    static Geometry2d::Point findNonBlockedGoal(
        Geometry2d::Point pt, boost::optional<Geometry2d::Point> prevPt,
        const Geometry2d::ShapeSet* obstacles, int maxItr = 300);
};

}  // namespace Planning
