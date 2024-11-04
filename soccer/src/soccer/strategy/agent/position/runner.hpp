#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() = default;
    Runner(const Position& other);

    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_pass() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_pass_ball() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        TOP_LEFT,
        TOP_RIGHT,
        BOT_RIGHT,
        BOT_LEFT,
    };
    State curr_state_ = State::TOP_LEFT;

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};
}  // namespace strategy
