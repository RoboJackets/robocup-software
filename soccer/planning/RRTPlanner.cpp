#include "RRTPlanner.hpp"
#include <Constants.hpp>
#include <Utils.hpp>
#include <protobuf/LogFrame.pb.h>
#include "motion/TrapezoidalMotion.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace Planning {

Geometry2d::Point randomPoint() {
    const auto& dims = Field_Dimensions::Current_Dimensions;
    float x = dims.FloorWidth() * (drand48() - 0.5f);
    float y = dims.FloorLength() * drand48() - dims.Border();

    return Geometry2d::Point(x, y);
}

RRTPlanner::RRTPlanner(int maxIterations) : _maxIterations(maxIterations) {}

bool RRTPlanner::shouldReplan(MotionInstant start, MotionInstant goal,
                              const MotionConstraints& motionConstraints,
                              const Geometry2d::ShapeSet* obstacles,
                              const Path* prevPath) const {
    if (!prevPath || !prevPath->valid()) return true;

    // if this number of microseconds passes since our last path plan,
    // we automatically replan
    const Time kPathExpirationInterval = replanTimeout() * SecsToTimestamp;
    if ((timestamp() - prevPath->startTime()) > kPathExpirationInterval) {
        return true;
    }

    float timeIntoPath =
        ((float)(timestamp() - prevPath->startTime())) * TimestampToSecs +
        1.0f / 60.0f;

    MotionInstant target;
    boost::optional<MotionInstant> optTarget = prevPath->evaluate(timeIntoPath);
    if (optTarget) {
        target = *optTarget;
    } else {
        // We went off the end of the path, so use the end for
        // calculations.
        target = *prevPath->destination();
    }

    float pathError = (target.pos - start.pos).mag();
    float replanThreshold = *motionConstraints._replan_threshold;
    // state()->drawCircle(target.pos, replanThreshold, Qt::green,
    //                     "MotionControl");
    // addText(QString("velocity: %1 %2").arg(this->vel.x).arg(this->vel.y));

    //  invalidate path if current position is more than the
    //  replanThreshold
    if (*motionConstraints._replan_threshold != 0 &&
        pathError > replanThreshold) {
        return true;
        // addText("pathError", Qt::red, "Motion");
    }

    if (std::isnan(target.pos.x) || std::isnan(target.pos.y)) {
        return true;
        // addText("Evaulate Returned an invalid result", Qt::red, "Motion");
    }

    float hitTime = 0;
    if (prevPath->hit(*obstacles, hitTime, timeIntoPath)) {
        return true;
        // addText("Hit Obstacle", Qt::red, "Motion");
    }

    // if the destination of the current path is greater than X m away
    // from the target destination, we invalidate the path. This
    // situation could arise if the path destination changed.
    float goalPosDiff = (prevPath->destination()->pos - goal.pos).mag();
    float goalVelDiff = (prevPath->destination()->vel - goal.vel).mag();
    if (goalPosDiff > goalChangeThreshold() ||
        goalVelDiff > goalChangeThreshold()) {
        // FIXME: goalChangeThreshold shouldn't be used for velocities as it
        // is above
        return true;
    }

    return false;
}

std::unique_ptr<Path> RRTPlanner::run(
    MotionInstant start, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    // This planner only works with commands of type 'PathTarget'
    assert(cmd.getCommandType() == Planning::MotionCommand::PathTarget);

    MotionInstant goal = cmd.getPlanningTarget();

    // Simple case: no path
    if (start.pos == goal.pos) {
        InterpolatedPath* path = new InterpolatedPath();
        path->setStartTime(timestamp());
        path->waypoints.emplace_back(
            MotionInstant(start.pos, Geometry2d::Point()), 0);
        return unique_ptr<Path>(path);
    }

    // Locate a goal point that is obstacle-free
    boost::optional<Geometry2d::Point> prevGoal;
    if (prevPath) prevGoal = prevPath->destination()->pos;
    goal.pos = findNonBlockedGoal(goal.pos, prevGoal, obstacles);

    // Replan if needed, otherwise return the previous path unmodified
    if (shouldReplan(start, goal, motionConstraints, obstacles,
                     prevPath.get())) {
        // Run bi-directional RRT to generate a path.
        InterpolatedPath* path =
            runRRT(start, goal, motionConstraints, obstacles);

        // If RRT failed, the path will be empty, so we need to add a single
        // point to make it valid.
        if (path && path->waypoints.empty()) {
            path->waypoints.emplace_back(
                MotionInstant(start.pos, Geometry2d::Point()), 0);
        }
        return unique_ptr<Path>(path);
    } else {
        return prevPath;
    }
}

Geometry2d::Point RRTPlanner::findNonBlockedGoal(
    Geometry2d::Point goal, boost::optional<Geometry2d::Point> prevGoal,
    const Geometry2d::ShapeSet* obstacles, int maxItr) {
    if (obstacles && obstacles->hit(goal)) {
        FixedStepTree goalTree;
        goalTree.init(goal, obstacles);
        goalTree.step = .1f;

        // The starting point is in an obstacle extend the tree until we find an
        // unobstructed point
        Geometry2d::Point newGoal;
        for (int i = 0; i < maxItr; ++i) {
            Geometry2d::Point r = randomPoint();

            // extend to a random point
            Tree::Point* newPoint = goalTree.extend(r);

            // if the new point is not blocked, it becomes the new goal
            if (newPoint && newPoint->hit.empty()) {
                newGoal = newPoint->pos;
                break;
            }
        }

        if (!prevGoal) return newGoal;

        // Only use this newly-found point if it's closer to the desired goal by
        // at least one robot radius or the old goal now collides with
        // obstacles.
        float oldDist = (*prevGoal - goal).mag();
        float newDist = (newGoal - goal).mag();
        if (newDist + Robot_Radius < oldDist || obstacles->hit(goal)) {
            return newGoal;
        } else {
            return *prevGoal;
        }
    }

    return goal;
}

InterpolatedPath* RRTPlanner::runRRT(MotionInstant start, MotionInstant goal,
                                     const MotionConstraints& motionConstraints,
                                     const Geometry2d::ShapeSet* obstacles) {
    InterpolatedPath* path = new InterpolatedPath();
    path->setStartTime(timestamp());

    // Initialize two RRT trees
    FixedStepTree startTree;
    FixedStepTree goalTree;
    startTree.init(start.pos, obstacles);
    goalTree.init(goal.pos, obstacles);
    startTree.step = goalTree.step = .15f;

    // Run bi-directional RRT algorithm
    Tree* ta = &startTree;
    Tree* tb = &goalTree;
    for (unsigned int i = 0; i < _maxIterations; ++i) {
        Geometry2d::Point r = randomPoint();

        Tree::Point* newPoint = ta->extend(r);

        if (newPoint) {
            // try to connect the other tree to this point
            if (tb->connect(newPoint->pos)) {
                // trees connected
                // done with global path finding
                // the path is from start to goal
                // runRRT will handle the rest
                break;
            }
        }

        swap(ta, tb);
    }

    Tree::Point* p0 = startTree.last();
    Tree::Point* p1 = goalTree.last();

    // sanity check
    if (!p0 || !p1 || p0->pos != p1->pos) {
        return path;
    }

    // extract path from RRTs
    // add the start tree first...normal order (aka from root to p0)
    startTree.addPath(*path, p0);
    // add the goal tree in reverse (aka p1 to root)
    goalTree.addPath(*path, p1, true);

    path = optimize(*path, obstacles, motionConstraints, start.vel, goal.vel);

    return path;
}

InterpolatedPath* RRTPlanner::optimize(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    unsigned int start = 0;

    if (path.empty()) {
        delete &path;
        return nullptr;
    }

    vector<InterpolatedPath::Entry>& pts = path.waypoints;

    // The set of obstacles the starting point was inside of
    const auto startHitSet = obstacles->hitSet(pts[start].pos());
    int span = 2;
    while (span < pts.size()) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            bool transitionValid = true;
            const auto newHitSet = obstacles->hitSet(
                Geometry2d::Segment(pts[i].pos(), pts[i + span].pos()));
            if (!newHitSet.empty()) {
                for (std::shared_ptr<Geometry2d::Shape> hit : newHitSet) {
                    if (startHitSet.find(hit) == startHitSet.end()) {
                        transitionValid = false;
                        break;
                    }
                }
            }

            if (transitionValid) {
                for (int x = 1; x < span; x++) {
                    pts.erase(pts.begin() + i + 1);
                }
                changed = true;
            }
        }

        if (!changed) span++;
    }
    // Done with the path
    return cubicBezier(path, obstacles, motionConstraints, vi, vf);
}

float getTime(InterpolatedPath& path, int index,
              const MotionConstraints& motionConstraints, float startSpeed,
              float endSpeed) {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}

// TODO: Use targeted end velocity
InterpolatedPath* RRTPlanner::cubicBezier(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    int length = path.waypoints.size();
    int curvesNum = length - 1;
    if (curvesNum <= 0) {
        delete &path;
        return nullptr;
    }

    // TODO: Get the actual values
    vector<double> pointsX(length);
    vector<double> pointsY(length);
    vector<double> ks(length - 1);
    vector<double> ks2(length - 1);

    for (int i = 0; i < length; i++) {
        pointsX[i] = path.waypoints[i].pos().x;
        pointsY[i] = path.waypoints[i].pos().y;
    }
    float startSpeed = vi.mag();

    // This is pretty hacky;
    Geometry2d::Point startDirection =
        (path.waypoints[1].pos() - path.waypoints[0].pos()).normalized();
    if (startSpeed < 0.3) {
        startSpeed = 0.3;
        vi = startDirection * startSpeed;
    } else {
        vi = vi.mag() * (startDirection + vi.normalized()) / 2.0 * 0.8;
    }

    const float endSpeed = vf.mag();

    for (int i = 0; i < curvesNum; i++) {
        ks[i] = 1.0 /
                (getTime(path, i + 1, motionConstraints, startSpeed, endSpeed) -
                 getTime(path, i, motionConstraints, startSpeed, endSpeed));
        ks2[i] = ks[i] * ks[i];
        if (std::isnan(ks[i])) {
            delete &path;
            return nullptr;
        }
    }

    VectorXd solutionX = cubicBezierCalc(vi.x, vf.x, pointsX, ks, ks2);
    VectorXd solutionY = cubicBezierCalc(vi.y, vf.y, pointsY, ks, ks2);

    Geometry2d::Point p0, p1, p2, p3;
    vector<InterpolatedPath::Entry> pts;
    const int interpolations = 10;
    double time = 0;

    for (int i = 0; i < curvesNum; i++) {
        p0 = path.waypoints[i].pos();
        p3 = path.waypoints[i + 1].pos();
        p1 = Geometry2d::Point(solutionX(i * 2), solutionY(i * 2));
        p2 = Geometry2d::Point(solutionX(i * 2 + 1), solutionY(i * 2 + 1));

        for (int j = 0; j < interpolations; j++) {
            double k = ks[i];
            float t = (((float)j / (float)(interpolations)));
            Geometry2d::Point pos =
                pow(1.0 - t, 3) * p0 + 3.0 * pow(1.0 - t, 2) * t * p1 +
                3 * (1.0 - t) * pow(t, 2) * p2 + pow(t, 3) * p3;
            t = ((float)j / (float)(interpolations)) / k;
            // 3 k (-(A (-1 + k t)^2) + k t (2 C - 3 C k t + D k t) + B (1 - 4 k
            // t + 3 k^2 t^2))
            Geometry2d::Point vel =
                3 * k * (-(p0 * pow(-1 + k * t, 2)) +
                         k * t * (2 * p2 - 3 * p2 * k * t + p3 * k * t) +
                         p1 * (1 - 4 * k * t + 3 * pow(k, 2) * pow(t, 2)));
            pts.emplace_back(MotionInstant(pos, vel), time + t);
        }
        time += 1.0 / ks[i];
    }
    pts.emplace_back(MotionInstant(path.waypoints[length - 1].pos(), vf), time);
    path.waypoints = pts;
    return &path;
}

VectorXd RRTPlanner::cubicBezierCalc(double vi, double vf,
                                     vector<double>& points, vector<double>& ks,
                                     vector<double>& ks2) {
    int curvesNum = points.size() - 1;

    if (curvesNum == 1) {
        VectorXd vector(2);
        vector[0] = vi / (3.0 * ks[0]) + points[0];
        vector[1] = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);
        return vector;
    } else {
        int matrixSize = curvesNum * 2;
        MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
        VectorXd answer(matrixSize);
        equations(0, 0) = 1;
        answer(0) = vi / (3.0 * ks[0]) + points[0];
        equations(1, matrixSize - 1) = 1;
        answer(1) = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);

        int i = 2;
        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2 + 1) = ks[n];
            equations(i, n * 2 + 2) = ks[n + 1];
            answer(i) = (ks[n] + ks[n + 1]) * points[n + 1];
            i++;
        }

        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2) = ks2[n];
            equations(i, n * 2 + 1) = -2 * ks2[n];
            equations(i, n * 2 + 2) = 2 * ks2[n + 1];
            equations(i, n * 2 + 3) = -ks2[n + 1];
            answer(i) = points[n + 1] * (ks2[n + 1] - ks2[n]);
            i++;
        }

        ColPivHouseholderQR<MatrixXd> solver(equations);
        VectorXd solution = solver.solve(answer);
        return solution;
    }
}

}  // namespace Planning
