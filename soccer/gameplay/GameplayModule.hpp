
#pragma once

// note: for an odd Qt-related issue, this python include has to come before
// the Qt includes (because of the 'slots' macro)
#include <boost/python.hpp>

#include <Geometry2d/TransformMatrix.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/ShapeSet.hpp>

#include <set>
#include <QMutex>
#include <QString>

#include <boost/ptr_container/ptr_vector.hpp>

class OurRobot;
class SystemState;

/**
 * @brief Higher-level logic for soccer
 * @details The Gameplay namespace contains things like Plays, Behaviors,
 * Tactics
 * and the GameplayModule.
 */
namespace Gameplay {

/**
 * @brief Coordinator of high-level logic
 *
 * @details The gameplay module has an embedded python interpreter and serves as
 * the bridge between our python and c++ code.
 * The python side of things is responsible for high-level gameplay.  At each
 * iteration of the main run loop, the GameplayModule
 * calls into python, which does all of the high-level planning, resulting in
 * updated motion targets, etc for the robots.  The
 * GameplayModule then executes path planning for each OurRobot.
 */
class GameplayModule {
public:
    GameplayModule(SystemState* state);
    virtual ~GameplayModule();

    SystemState* state() const { return _state; }

    virtual void run();

    void setupUI();

    /**
     * @brief Loads a playbook file to enable specified plays.
     * If isAbsolute is false, the path is treated as relative to the
     * playbooks directory. Otherwise, it is treated as an absolute path.
     */
    void loadPlaybook(const std::string& playbookFile, bool isAbsolute = false);

    /**
     * @brief Saves the currently enabled plays to a playbook file
     * If isAbsolute is false, the path is treated as relative to the
     * playbooks directory. Otherwise, it is treated as an absolute path.
     */
    void savePlaybook(const std::string& playbookFile, bool isAbsolute = false);

    void goalieID(int value);
    int goalieID() { return _goalieID; }

    /**
     * @defgroup matrices Coordinate Conversion Matrices
     * Each of these matrices converts coordinates from some other system
     * to team space.
     *
     * Example:
     * team = _gameplay->oppMatrix() * Geometry2d::Point(1, 0);
     */

    /**
     * Centered on the ball
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix ballMatrix() const { return _ballMatrix; }

    /**
     * Center of the field
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix centerMatrix() const { return _centerMatrix; }

    /**
     * Opponent's coordinates
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix oppMatrix() const { return _oppMatrix; }

    /// All robots on our team that are usable by plays
    const std::set<OurRobot*>& playRobots() const { return _playRobots; }

    void sendFieldDimensionsToPython();

    void calculateFieldObstacles();

    /**
     * Returns the current set of global obstacles, including the field
     */
    Geometry2d::ShapeSet globalObstacles() const;

    /// Returns a ShapeSet containing both goal zones
    Geometry2d::ShapeSet goalZoneObstacles() const;

protected:
    boost::python::object getRootPlay();

    /// gets the instance of the main.py module that's loaded at GameplayModule
    boost::python::object getMainModule();

    boost::python::object getConstantsModule();

private:
    /// This protects all of Gameplay.
    /// This is held while plays are running.
    QMutex _mutex;

    SystemState* _state;

    std::set<OurRobot*> _playRobots;

    Geometry2d::TransformMatrix _ballMatrix;
    Geometry2d::TransformMatrix _centerMatrix;
    Geometry2d::TransformMatrix _oppMatrix;

    /// Obstacles to prevent using half the field
    std::shared_ptr<Geometry2d::Polygon> _ourHalf;
    std::shared_ptr<Geometry2d::Polygon> _opponentHalf;

    std::shared_ptr<Geometry2d::Shape> _sideObstacle;

    /// outside of the floor boundaries
    std::shared_ptr<Geometry2d::Shape> _nonFloor[4];

    /// goal areas
    std::shared_ptr<Geometry2d::CompositeShape> _ourGoalArea;
    std::shared_ptr<Geometry2d::CompositeShape> _theirGoalArea;

    std::shared_ptr<Geometry2d::Polygon> _ourGoal;
    std::shared_ptr<Geometry2d::Polygon> _theirGoal;

    /// utility functions

    int _our_score_last_frame;

    // Shell ID of the robot to assign the goalie position
    int _goalieID;

    // python
    boost::python::object _mainPyNamespace;
};
}
