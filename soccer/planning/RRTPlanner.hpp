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

struct CubicBezierControlPoints {
    Geometry2d::Point p0, p1, p2, p3;

    CubicBezierControlPoints(Geometry2d::Point p0, Geometry2d::Point p1, Geometry2d::Point p2, Geometry2d::Point p3)
            :p0(p0), p1(p1), p2(p2), p3(p3) {}
};

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
        return MotionCommand::PathTarget;
    }

    /// run the path RRTplanner
    /// this will always populate path to be the path we need to travel
    std::unique_ptr<Path> run(
        MotionInstant start, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

    /**
     * Takes in a point path and returns a InterpolatedPath with a generated
     * Velocity Profile.
     */
    static std::unique_ptr<Planning::InterpolatedPath> generateVelocityPath(
        std::vector<Geometry2d::Point>& points,
        const Geometry2d::ShapeSet& obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Takes in a point path and returns a InterpolatedPath with a generated
     * Velocity Profile.
     */
    static std::unique_ptr<Planning::InterpolatedPath> generateCubicBezier(
            std::vector<Geometry2d::Point>& points,
            const Geometry2d::ShapeSet& obstacles,
            const MotionConstraints& motionConstraints, Geometry2d::Point vi,
            Geometry2d::Point vf);


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

    /// Runs a bi-directional RRT to attempt to join the start and end states.
    std::unique_ptr<Planning::InterpolatedPath> runRRT(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles);

    /** optimize the path
     *  Calls the cubicBezier optimization function.
     */
    std::unique_ptr<Planning::InterpolatedPath> optimize(
        std::unique_ptr<InterpolatedPath> path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Uses a cubicBezier to interpolate between the points on the path and add
     * velocity planning
     */
    static std::unique_ptr<Planning::InterpolatedPath> cubicBezier(
        std::unique_ptr<Planning::InterpolatedPath> path, const Geometry2d::ShapeSet* obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Generates a Cubic Bezier Path based on Albert's random Bezier Velocity Path Algorithm
     */
    static std::vector<CubicBezierControlPoints> generateCubicBezierPath(const std::vector<Geometry2d::Point> &points,
                                                                    const MotionConstraints &motionConstraints,
                                                                    Geometry2d::Point vi, Geometry2d::Point vf,
                                                                    const boost::optional<std::vector<float>> &times = boost::none);
    /**
     * Generates a Cubic Bezier Path based on some attempted heuristical Control Point Placement
     */
    static std::vector<CubicBezierControlPoints> generateNormalCubicBezierPath(const std::vector<Geometry2d::Point>& points,
                                                                    const MotionConstraints& motionConstraints,
                                                                    Geometry2d::Point vi, Geometry2d::Point vf);

    /**
     * Helper function for cubicBezier() which uses Eigen matrices to solve for
     * the
     * cubic bezier equations.
     */
    static Eigen::VectorXd cubicBezierCalc(double vi, double vf,
                                           std::vector<double>& points,
                                           std::vector<double>& ks,
                                           std::vector<double>& ks2);
};
}  // namespace Planning
