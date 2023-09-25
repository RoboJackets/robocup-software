#include "strategy/agent/position/position.hpp"
namespace strategy {
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    enum State { SIDE_LEFT, SIDE_TOP, SIDE_RIGHT, SIDE_BOTTOM };
    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the offensive agent (state machine)
    State current_state_ = SIDE_LEFT;
};
}  // namespace strategy
