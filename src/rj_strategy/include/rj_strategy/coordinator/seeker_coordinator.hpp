#pragma once

#include <algorithm>
#include <array>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_msgs/msg/seeker_coordinator.hpp>
#include <rj_msgs/srv/seeker_coordinator.hpp>
#include <rj_msgs/msg/world_state.hpp>


#include "rj_strategy/coordinator.hpp"

namespace strategy {

class SeekerCoordinator
    : public Coordinator<SeekerCoordinator, rj_msgs::srv::SeekerCoordinator, rj_msgs::msg::SeekerCoordinator> {
public:

    SeekerCoordinator();
    ~SeekerCoordinator() override = default;
    SeekerCoordinator(const SeekerCoordinator&) = delete;
    SeekerCoordinator& operator=(const SeekerCoordinator&) = delete;
    SeekerCoordinator(SeekerCoordinator&&) = delete;
    SeekerCoordinator& operator=(SeekerCoordinator&&) = delete;

    void service_callback(RequestPtr request, ResponsePtr response);
    static rj_geometry::Point invalidPoint() { return rj_geometry::Point{-1,-1}; }

private:
    /**
     * @brief publishes current seeker target points whenever a new target point is acquired
     */
    void publish_seeker_points();

    /**
     * @brief updates target point for robot_id whenever a robot joins the seeker group or polls for a new target.
     */
    void update_target(int robot_id);

    /**
     * @brief Returns the point which is most 'open'
     *
     * @param world_state The current WorldState
     * @param current_position The current position of the seeker
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The target point
     */
    rj_geometry::Point get_open_point(const WorldState world_state,
                                      rj_geometry::Point current_position,
                                      const FieldDimensions& field_dimensions) const;

    /**
     * @brief Calculates which point is the best by iteratively searching around the robot
     *
     * @param current_prec A double that represents how far away to look from the robot
     * @param min_prec A double that represents the minimum distance to look from the robot
     * @param current_point The robot's current position
     * @param world_state The current WorldState
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The best point found
     */
    rj_geometry::Point calculate_open_point(double current_prec, double min_prec,
                                            rj_geometry::Point current_point,
                                            const WorldState world_state,
                                            const FieldDimensions& field_dimensions) const;

    /**
     * @brief Corrects the point to be within the field
     *
     * @param point The point to correct
     * @param field_dimensions The dimensions of the field
     *
     * @return rj_geometry::Point The corrected point
     */
    [[nodiscard]] rj_geometry::Point correct_point(rj_geometry::Point point,
                                                   const FieldDimensions& field_dimensions) const;

    /**
     * @brief Calculates how 'good' a target point is
     *
     * @param ball_pos The current position of the ball
     * @param current_point The point that is being evaluated
     * @param world_state The current world state
     *
     * @return double The evaluation of that target point
     */
    [[nodiscard]] double eval_point(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                                    const WorldState world_state,
                                    const FieldDimensions& field_dimensions) const;


    WorldState last_world_state_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    std::array<rj_geometry::Point, kNumShells> seeker_points_{rj_geometry::Point{-1,-1}};
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;
    std::array<bool, kNumShells> is_seeking_ {false};
};

}  // namespace strategy