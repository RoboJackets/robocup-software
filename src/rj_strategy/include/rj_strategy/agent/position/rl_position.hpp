#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * Position subclass driven by a trained reinforcement learning policy.
 *
 * Loads a simple feedforward neural network (MLP) whose weights were
 * exported from the Python rj_rl training pipeline, encodes the current
 * WorldState into the same observation format the policy was trained on,
 * runs a forward pass, and maps the resulting discrete action to a
 * RobotIntent using the existing motion-command vocabulary.
 *
 * The coordinate systems differ between the C++ stack and the RL
 * training environment:
 *   C++: x across field, y along field (0 = our goal, length = theirs)
 *   RL:  x along field (centered), y across field
 * All transforms are handled internally by encode_state().
 */
class RLPosition : public Position {
public:
    RLPosition(int r_id);
    ~RLPosition() override = default;
    RLPosition(Position&& other);

    /**
     * Set the path to the exported actor weights file (.bin format).
     * Must be called before the first get_task() invocation.
     */
    static void set_weights_path(const std::string& path);

    /** True once the actor network has been loaded successfully. */
    static bool weights_loaded();

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // -----------------------------------------------------------------
    // Neural network internals
    // -----------------------------------------------------------------
    struct Layer {
        std::vector<float> weights;  // row-major [rows x cols]
        std::vector<float> biases;   // [cols]
        int rows = 0;
        int cols = 0;
    };

    static std::vector<Layer> actor_layers_;
    static bool loaded_;
    static std::string weights_path_;

    static bool load_weights();
    std::vector<float> forward(const std::vector<float>& input) const;

    // -----------------------------------------------------------------
    // State encoding  (WorldState → RL observation)
    // -----------------------------------------------------------------
    std::vector<float> encode_state() const;

    // C++ → RL coordinate helpers
    std::pair<float, float> to_rl_pos(const rj_geometry::Point& p) const;
    std::pair<float, float> to_rl_vel(const rj_geometry::Point& v) const;

    // -----------------------------------------------------------------
    // Action mapping  (RL action → RobotIntent)
    // -----------------------------------------------------------------
    int select_action(const std::vector<float>& logits) const;
    std::optional<RobotIntent> action_to_intent(int action, RobotIntent intent) const;

    rj_geometry::Point calculate_best_shot() const;
    int find_nearest_teammate() const;

    // -----------------------------------------------------------------
    // Constants matching the Python training configuration
    // -----------------------------------------------------------------
    static constexpr int kNumTeammates = 5;
    static constexpr int kNumOpponents = 6;
    static constexpr int kObsSize = 14 + kNumTeammates * 2 + kNumOpponents * 2;

    static constexpr float kRLHalfLength = 4.5f;
    static constexpr float kRLHalfWidth = 3.0f;
    static constexpr float kRLMaxSpeed = 7.0f;

    enum RLAction {
        MOVE_TO_BALL = 0,
        SHOOT_ON_GOAL = 1,
        PASS_TO_NEAREST = 2,
        DEFEND_GOAL = 3,
        POSITION_OFFENSE = 4,
        POSITION_DEFENSE = 5,
        RL_IDLE = 6,
        NUM_ACTIONS = 7,
    };

    mutable std::string last_action_name_{"none"};
};

}  // namespace strategy
