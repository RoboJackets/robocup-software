#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {
class Idle : public Position {
public:
    Idle(int r_id);
    ~Idle() = default;

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

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};
}  // namespace strategy
