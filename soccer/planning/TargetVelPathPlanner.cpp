#include "TargetVelPathPlanner.hpp"

using namespace Geometry2d;

namespace Planning {

class TargetVelPath : public Path {
public:
    TargetVelPath(Time startTime, MotionInstant start, Point targetVel,
                  float accel)
        : Path(startTime),
          _start(start),
          _targetVel(targetVel),
          _accel(accel) {}

    virtual boost::optional<MotionInstant> evaluate(float t) const {

    }

    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime) const {

    }

    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const {

    }

    virtual float getDuration() const {

    }

    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const {

    }

    virtual boost::optional<MotionInstant> destination() const {

    }

    virtual std::unique_ptr<Path> clone() const {
        return std::unique_ptr<Path>(new TargetVelPath(startTime(), _start, _targetVel, _accel));
    }


private:
    friend class TargetVelPathPlanner;

    const MotionInstant _start;
    const Point _targetVel;
    const float _accel;
};

bool TargetVelPathPlanner::shouldReplan(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {

    if (!prevPath) return true;


    if (prevPath)


    return false;
}

std::unique_ptr<Path> TargetVelPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    assert(cmd.getCommandType() == MotionCommand::WorldVel);

    if (shouldReplan(startInstant, cmd, motionConstraints, obstacles,
                     prevPath.get())) {
        Point vel = cmd.getWorldVel();

        // TODO: obstacles
        // TODO: trapezoidal motion profile

        // TODO: max velocity

        // TODO: when to replan?
    } else {
        return std::move(prevPath);
    }

    return nullptr;
}

}  // namespace Planning
