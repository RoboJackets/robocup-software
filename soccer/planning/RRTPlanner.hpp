#pragma once

#include <optional>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <planning/InterpolatedPath.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>
#include "SingleRobotPathPlanner.hpp"

#include <rrt/BiRRT.hpp>

#include "SystemState.hpp"

#include <Eigen/Dense>
#include <list>

namespace Planning {

struct CubicBezierControlPoints {
    Geometry2d::Point p0, p1, p2, p3;

    CubicBezierControlPoints(Geometry2d::Point p0, Geometry2d::Point p1,
                             Geometry2d::Point p2, Geometry2d::Point p3)
        : p0(p0), p1(p1), p2(p2), p3(p3) {}
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
    /**
     * Constructor taking in the max iterations the RRT planner should run
     */
    RRTPlanner(int minIterations, int maxIterations);

    /**
     * gets the maximum number of iterations for the RRT algorithm
     */
    int maxIterations() const { return _maxIterations; }

    /**
     * sets the maximum number of iterations for th RRT algorithm
     */
    void maxIterations(int value) { _maxIterations = value; }

    int minIterations() const { return _minIterations; }
    void minIterations(int value) { _minIterations = value; }

    /**
     * Takes in a waypoints and returns a full InterpolatedPath with a generated
     * Velocity Profile.
     */
    static std::unique_ptr<Planning::InterpolatedPath> generatePath(
        const std::vector<Geometry2d::Point>& points,
        const Geometry2d::ShapeSet& obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    // Overridden methods

    MotionCommand::CommandType commandType() const override {
        return MotionCommand::PathTarget;
    }

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    static void createConfiguration(Configuration* cfg);

    static RJ::Seconds getPartialReplanLeadTime();

private:
    int reusePathTries = 0;

protected:
    /// minimum and maximum number of rrt iterations to run
    /// this does not include connect attempts
    int _minIterations, _maxIterations;

    /// Check to see if the previous path (if any) should be discarded and
    /// replaced with a newly-planned one
    bool shouldReplan(const PlanRequest& planRequest,
                      const std::vector<DynamicObstacle> dynamicObs,
                      std::string* debugOut = nullptr) const;

    /// Runs a bi-directional RRT to attempt to join the start and end states.
    std::vector<Geometry2d::Point> runRRT(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet& obstacles, Context* context,
        unsigned shellID,
        const std::optional<std::vector<Geometry2d::Point>>& biasWaypoints =
            std::nullopt);

    std::unique_ptr<InterpolatedPath> generateRRTPath(
        const MotionInstant& start, const MotionInstant& goal,
        const MotionConstraints& motionConstraints,
        Geometry2d::ShapeSet& origional,
        const std::vector<DynamicObstacle> dyObs, Context* context,
        unsigned shellID,
        const std::optional<std::vector<Geometry2d::Point>>& biasWayPoints =
            std::nullopt);

    /**
     * Takes in waypoints and returns a InterpolatedPath with a generated
     * Velocity Profile.
     *
     * 1. It generates a bezier path from the waypoints given
     * 2. It generates a velocity profile to fit the constraints given
     * 3. It returns a interpolated path combining the bezier path and the
     *velocity profile
     */
    static std::unique_ptr<Planning::InterpolatedPath> generateCubicBezier(
        const std::vector<Geometry2d::Point>& points,
        const Geometry2d::ShapeSet& obstacles,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     *  Removes unnecesary waypoints in the path
     */
    static void optimize(std::vector<Geometry2d::Point>& path,
                         const Geometry2d::ShapeSet& obstacles,
                         Geometry2d::Point vi, Geometry2d::Point vf);

    /**
     * Generates a Cubic Bezier Path based on Albert's Bezier Velocity
     * Path Algorithm using waypoints. It creates a path with continuous
     * velocity and acceleration
     * that estimates speed using the trapezoidal motion heuristic
     */
    static std::vector<CubicBezierControlPoints> generateCubicBezierPath(
        const std::vector<Geometry2d::Point>& points,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf,
        const std::optional<std::vector<double>>& times = std::nullopt);

    /**
     * Generates a velocity profile from a Cubic Bezier Path under the given
     * motion constratins
     */
    static std::vector<InterpolatedPath::Entry> generateVelocityPath(
        const std::vector<CubicBezierControlPoints>& controlPoints,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf, int interpolations = 40);

    /**
     * Generates a Cubic Bezier Path based on some attempted heuristical Control
     * Point Placement
     */
    static std::vector<CubicBezierControlPoints> generateNormalCubicBezierPath(
        const std::vector<Geometry2d::Point>& points,
        const MotionConstraints& motionConstraints, Geometry2d::Point vi,
        Geometry2d::Point vf);

    /**
     * Helper function for cubicBezier() which uses Eigen matrices to solve for
     * the
     * cubic bezier equations.
     */
    static Eigen::VectorXd cubicBezierCalc(double vi, double vf,
                                           std::vector<double>& points,
                                           std::vector<double>& ks,
                                           std::vector<double>& ks2);

    /**
 * Helper method for runRRT(), which creates a vector of points representing
 * the RRT path.
 */
    std::vector<Geometry2d::Point> runRRTHelper(
        MotionInstant start, MotionInstant goal,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet& obstacles, Context* context,
        unsigned shellID,
        const std::optional<std::vector<Geometry2d::Point>>& biasWaypoints,
        bool straightLine);

    static ConfigDouble* _partialReplanLeadTime;
};
}  // namespace Planning
