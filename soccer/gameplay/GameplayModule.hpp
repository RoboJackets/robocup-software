
#pragma once

// note: for an odd Qt-related issue, this python include has to come before
// the Qt includes (because of the 'slots' macro)
#include <geometry2d/composite_shape.h>
#include <geometry2d/point.h>
#include <geometry2d/polygon.h>
#include <geometry2d/shape_set.h>
#include <geometry2d/transform_matrix.h>

#include <Configuration.hpp>
#include <Context.hpp>
#include <GrSimCommunicator.hpp>
#include <QString>
#include <Referee.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/python.hpp>
#include <set>

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
    GameplayModule(Context* const context, Referee* refereeModule);
    virtual ~GameplayModule();

    SystemState* state() const { return &_context->state; }

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

    void clearPlays();
    bool checkPlaybookStatus();

    /**
     * @defgroup matrices Coordinate Conversion Matrices
     * Each of these matrices converts coordinates from some other system
     * to team space.
     *
     * Example:
     * team = _gameplay->oppMatrix() * geometry2d::Point(1, 0);
     */

    /**
     * Centered on the ball
     * @ingroup matrices
     */
    geometry2d::TransformMatrix ballMatrix() const { return _ballMatrix; }

    /**
     * Center of the field
     * @ingroup matrices
     */
    geometry2d::TransformMatrix centerMatrix() const { return _centerMatrix; }

    /**
     * Opponent's coordinates
     * @ingroup matrices
     */
    geometry2d::TransformMatrix oppMatrix() const { return _oppMatrix; }

    /// All robots on our team that are usable by plays
    const std::set<OurRobot*>& playRobots() const { return _playRobots; }

    void sendFieldDimensionsToPython();

    void calculateFieldObstacles();

    bool hasFieldEdgeInsetChanged() const;

    static void createConfiguration(Configuration* cfg);

    /**
     * Returns the current set of global obstacles, including the field
     */
    geometry2d::ShapeSet globalObstacles() const;

    /// Returns a ShapeSet containing both goal zones
    geometry2d::ShapeSet goalZoneObstacles() const;

    /// Resends the current field dimensions to python. This should be called
    /// whenever the current field dimensions change
    void updateFieldDimensions();

    /// adds tests to the list of tests to run
    void addTests();

    /// remove selected test from the list of tests to run
    void removeTest();

    /// loads a test for the testing tab
    void loadTest();

    /// go to the next test for the testing tab
    void nextTest();

protected:
    boost::python::object getRootPlay();

    /// gets the instance of the main.py module that's loaded at GameplayModule
    boost::python::object getMainModule();

private:
    static ConfigDouble* _fieldEdgeInset;
    double _oldFieldEdgeInset;

    Context* const _context;
    Referee* const _refereeModule;

    std::set<OurRobot*> _playRobots;

    geometry2d::TransformMatrix _ballMatrix;
    geometry2d::TransformMatrix _centerMatrix;
    geometry2d::TransformMatrix _oppMatrix;

    /// Obstacles to prevent using half the field
    std::shared_ptr<geometry2d::Polygon> _ourHalf;
    std::shared_ptr<geometry2d::Polygon> _opponentHalf;

    std::shared_ptr<geometry2d::Shape> _sideObstacle;

    /// outside of the floor boundaries
    std::shared_ptr<geometry2d::Shape> _nonFloor[4];

    /// goal areas
    std::shared_ptr<geometry2d::CompositeShape> _ourGoalArea;
    std::shared_ptr<geometry2d::CompositeShape> _theirGoalArea;

    std::shared_ptr<geometry2d::Polygon> _ourGoal;
    std::shared_ptr<geometry2d::Polygon> _theirGoal;

    /// utility functions

    int _our_score_last_frame;

    // Shell ID of the robot to assign the goalie position
    int _goalieID;

    // python
    boost::python::object _mainPyNamespace;

    // Testing
    bool runningTests = false;
};
}
