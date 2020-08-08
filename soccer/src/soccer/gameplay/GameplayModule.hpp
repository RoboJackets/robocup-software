
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
#include <QString>

#include <Configuration.hpp>
#include <Context.hpp>
#include <GrSimCommunicator.hpp>
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
    GameplayModule(Context* context);
    virtual ~GameplayModule();

    SystemState* state() const { return &context_->state; }

    virtual void run();

    void setup_ui();

    /**
     * @brief Loads a playbook file to enable specified plays.
     * If is_absolute is false, the path is treated as relative to the
     * playbooks directory. Otherwise, it is treated as an absolute path.
     */
    void load_playbook(const std::string& playbook_file, bool is_absolute = false);

    /**
     * @brief Saves the currently enabled plays to a playbook file
     * If is_absolute is false, the path is treated as relative to the
     * playbooks directory. Otherwise, it is treated as an absolute path.
     */
    void save_playbook(const std::string& playbook_file, bool is_absolute = false);

    void clear_plays();
    bool check_playbook_status();

    /**
     * @defgroup matrices Coordinate Conversion Matrices
     * Each of these matrices converts coordinates from some other system
     * to team space.
     *
     * Example:
     * team = gameplay_->opp_matrix() * Geometry2d::Point(1, 0);
     */

    /**
     * Centered on the ball
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix ball_matrix() const { return ball_matrix_; }

    /**
     * Center of the field
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix center_matrix() const { return center_matrix_; }

    /**
     * Opponent's coordinates
     * @ingroup matrices
     */
    Geometry2d::TransformMatrix opp_matrix() const { return opp_matrix_; }

    /// All robots on our team that are usable by plays
    const std::set<OurRobot*>& play_robots() const { return play_robots_; }

    void send_field_dimensions_to_python();

    void calculate_field_obstacles();

    bool has_field_edge_inset_changed() const;

    static void create_configuration(Configuration* cfg);

    /**
     * Returns the current set of global obstacles, including the field
     */
    Geometry2d::ShapeSet global_obstacles() const;

    /// Returns a ShapeSet containing both goal zones
    Geometry2d::ShapeSet goal_zone_obstacles() const;

    /// Resends the current field dimensions to python. This should be called
    /// whenever the current field dimensions change
    void update_field_dimensions();

    /// adds tests to the list of tests to run
    void add_tests();

    /// remove selected test from the list of tests to run
    void remove_test();

    /// loads a test for the testing tab
    void load_test();

    /// go to the next test for the testing tab
    void next_test();

protected:
    boost::python::object get_root_play();

    /// gets the instance of the main.py module that's loaded at GameplayModule
    boost::python::object get_main_module();

private:
    static ConfigDouble* field_edge_inset;
    double old_field_edge_inset_;

    Context* const context_;

    std::set<OurRobot*> play_robots_;

    Geometry2d::TransformMatrix ball_matrix_;
    Geometry2d::TransformMatrix center_matrix_;
    Geometry2d::TransformMatrix opp_matrix_;

    /// Obstacles to prevent using half the field
    std::shared_ptr<Geometry2d::Shape> our_half_;
    std::shared_ptr<Geometry2d::Shape> opponent_half_;

    std::shared_ptr<Geometry2d::Shape> side_obstacle_;

    /// outside of the floor boundaries
    std::shared_ptr<Geometry2d::Shape> non_floor_[4];

    /// goal areas
    std::shared_ptr<Geometry2d::Shape> our_goal_area_;
    std::shared_ptr<Geometry2d::Shape> their_goal_area_;

    std::shared_ptr<Geometry2d::Shape> our_goal_;
    std::shared_ptr<Geometry2d::Shape> their_goal_;

    int our_score_last_frame_ = 0;

    // python
    boost::python::object main_py_namespace_;

    // Testing
    bool running_tests_ = false;
};
}
