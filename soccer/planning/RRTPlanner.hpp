#pragma once

#include "SingleRobotPathPlanner.hpp"
#include "Tree.hpp"
#include <Geometry2d/ShapeSet.hpp>
#include <Geometry2d/Point.hpp>
#include <planning/InterpolatedPath.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <list>

namespace Planning {

/** generate a random point on the floor */
Geometry2d::Point randomPoint();

/**
 * @brief Given a start point and an end point and some conditions, plans a path
 * for a robot to get there.
 *
 * @details There are many ways to plan paths.  This planner uses bidirectional
 * [RRTs](http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree).
 * You can check out our interactive RRT applet on GitHub here:
 * https://github.com/RoboJackets/rrt.
 */
class RRTPlanner : public SingleRobotPathPlanner {
public:
    RRTPlanner(int maxIterations);
    /**
     * gets the maximum number of iterations for the RRT algorithm
     */
    int maxIterations() const { return _maxIterations; }
    /**
     * sets the maximum number of iterations for th RRT algorithm
     */
    void maxIterations(int value) { _maxIterations = value; }

    MotionCommand::CommandType commandType() const override {
        return MotionCommand::CommandType::PathTarget;
    }

    /// run the path RRTplanner
    /// this will always populate path to be the path we need to travel
    std::unique_ptr<Path> run(
        MotionInstant start,
        const std::unique_ptr<MotionCommand>& cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

protected:
    /// maximum number of rrt iterations to run
    /// this does not include connect attempts
    unsigned int _maxIterations;

    /// Check to see if the previous path (if any) should be discarded and
    /// replaced with a newly-planned one
    bool shouldReplan(MotionInstant start, MotionInstant goal,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath) const;

    /// If the given goal point is in an obstacle, uses an RRT to attempt to
    /// find a point that is close, but not blocked.
    Geometry2d::Point findNonBlockedGoal(
        Geometry2d::Point goal, boost::optional<Geometry2d::Point> prevGoal,
        const Geometry2d::ShapeSet* obstacles, int maxItr = 100);

    /// Runs a bi-directional RRT to attempt to join the start and end states.
    Planning::InterpolatedPath* runRRT(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles);

    /** optimize the path
     *  Calls the cubicBezier optimization function.
     */
    Planning::InterpolatedPath* optimize(
        Planning::InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Uses a cubicBezier to interpolate between the points on the path and add
     * velocity planning
     */
    Planning::InterpolatedPath* cubicBezier(
        Planning::InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Helper function for cubicBezier() which uses Eigen matrices to solve for
     * the
     * cubic bezier equations.
     */
    Eigen::VectorXd cubicBezierCalc(double vi, double vf,
                                    std::vector<double>& points,
                                    std::vector<double>& ks,
                                    std::vector<double>& ks2);
};

}  // namespace Planning
