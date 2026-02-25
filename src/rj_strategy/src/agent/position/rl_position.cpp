#include "rj_strategy/agent/position/rl_position.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>

namespace strategy {

// Static member definitions
std::vector<RLPosition::Layer> RLPosition::actor_layers_;
bool RLPosition::loaded_ = false;
std::string RLPosition::weights_path_ = "checkpoints/policy_final_actor.bin";

// -----------------------------------------------------------------
// Construction
// -----------------------------------------------------------------

RLPosition::RLPosition(int r_id) : Position{r_id, "RLPosition"} {}

RLPosition::RLPosition(Position&& other) : Position{std::move(other)} {
    position_name_ = "RLPosition";
}

void RLPosition::set_weights_path(const std::string& path) {
    weights_path_ = path;
    loaded_ = false;
}

bool RLPosition::weights_loaded() { return loaded_; }

// -----------------------------------------------------------------
// Weight loading (simple binary format exported by export_weights.py)
//
//   uint32  num_layers
//   For each layer:
//     uint32  rows   (fan_in)
//     uint32  cols   (fan_out)
//     float32 weights[rows * cols]   (row-major)
//     float32 biases[cols]
// -----------------------------------------------------------------

bool RLPosition::load_weights() {
    std::ifstream file(weights_path_, std::ios::binary);
    if (!file.is_open()) {
        SPDLOG_ERROR("RLPosition: cannot open weights file '{}'", weights_path_);
        return false;
    }

    auto read_u32 = [&]() -> uint32_t {
        uint32_t v = 0;
        file.read(reinterpret_cast<char*>(&v), sizeof(v));
        return v;
    };

    uint32_t num_layers = read_u32();
    if (num_layers == 0 || num_layers > 20) {
        SPDLOG_ERROR("RLPosition: invalid num_layers={}", num_layers);
        return false;
    }

    actor_layers_.clear();
    actor_layers_.reserve(num_layers);

    for (uint32_t i = 0; i < num_layers; ++i) {
        Layer layer;
        layer.rows = static_cast<int>(read_u32());
        layer.cols = static_cast<int>(read_u32());

        if (layer.rows <= 0 || layer.cols <= 0) {
            SPDLOG_ERROR("RLPosition: bad layer dims {}x{}", layer.rows, layer.cols);
            return false;
        }

        layer.weights.resize(layer.rows * layer.cols);
        file.read(reinterpret_cast<char*>(layer.weights.data()),
                  layer.weights.size() * sizeof(float));

        layer.biases.resize(layer.cols);
        file.read(reinterpret_cast<char*>(layer.biases.data()),
                  layer.biases.size() * sizeof(float));

        actor_layers_.push_back(std::move(layer));
    }

    if (file.fail()) {
        SPDLOG_ERROR("RLPosition: truncated weights file");
        actor_layers_.clear();
        return false;
    }

    loaded_ = true;
    SPDLOG_INFO("RLPosition: loaded {} actor layers from '{}'", num_layers, weights_path_);
    return true;
}

// -----------------------------------------------------------------
// MLP forward pass: ReLU hidden layers, linear output
// -----------------------------------------------------------------

std::vector<float> RLPosition::forward(const std::vector<float>& input) const {
    std::vector<float> h = input;

    for (size_t li = 0; li < actor_layers_.size(); ++li) {
        const auto& layer = actor_layers_[li];
        std::vector<float> out(layer.cols, 0.0f);

        for (int c = 0; c < layer.cols; ++c) {
            float sum = layer.biases[c];
            for (int r = 0; r < layer.rows; ++r) {
                sum += h[r] * layer.weights[r * layer.cols + c];
            }
            if (li < actor_layers_.size() - 1) {
                sum = std::max(sum, 0.0f);  // ReLU
            }
            out[c] = sum;
        }

        h = std::move(out);
    }

    return h;
}

// -----------------------------------------------------------------
// State encoding
// -----------------------------------------------------------------

std::pair<float, float> RLPosition::to_rl_pos(const rj_geometry::Point& p) const {
    float length = field_dimensions_.length();
    return {static_cast<float>(p.y()) - length / 2.0f, static_cast<float>(p.x())};
}

std::pair<float, float> RLPosition::to_rl_vel(const rj_geometry::Point& v) const {
    return {static_cast<float>(v.y()), static_cast<float>(v.x())};
}

std::vector<float> RLPosition::encode_state() const {
    std::vector<float> obs(kObsSize, 0.0f);
    int idx = 0;

    const auto& robot = last_world_state_->get_robot(true, robot_id_);
    const auto& ball = last_world_state_->ball;

    auto norm_pos = [&](float rl_x, float rl_y) {
        return std::make_pair(rl_x / kRLHalfLength, rl_y / kRLHalfWidth);
    };
    auto norm_vel = [&](float rl_vx, float rl_vy) {
        return std::make_pair(rl_vx / kRLMaxSpeed, rl_vy / kRLMaxSpeed);
    };

    // Robot position
    auto [rx, ry] = to_rl_pos(robot.pose.position());
    auto [nrx, nry] = norm_pos(rx, ry);
    obs[idx++] = nrx;
    obs[idx++] = nry;

    // Robot velocity
    auto [rvx, rvy] = to_rl_vel(robot.velocity.linear());
    auto [nvx, nvy] = norm_vel(rvx, rvy);
    obs[idx++] = nvx;
    obs[idx++] = nvy;

    // Robot heading: rl_heading = π/2 − cpp_heading
    // cos(rl_h) = sin(cpp_h), sin(rl_h) = cos(cpp_h)
    double heading = robot.pose.heading();
    obs[idx++] = static_cast<float>(std::sin(heading));
    obs[idx++] = static_cast<float>(std::cos(heading));

    // Ball position
    auto [bx, by] = to_rl_pos(ball.position);
    auto [nbx, nby] = norm_pos(bx, by);
    obs[idx++] = nbx;
    obs[idx++] = nby;

    // Ball velocity
    auto [bvx, bvy] = to_rl_vel(ball.velocity);
    auto [nbvx, nbvy] = norm_vel(bvx, bvy);
    obs[idx++] = nbvx;
    obs[idx++] = nbvy;

    // Own goal
    auto [ogx, ogy] = to_rl_pos(field_dimensions_.our_goal_loc());
    auto [nogx, nogy] = norm_pos(ogx, ogy);
    obs[idx++] = nogx;
    obs[idx++] = nogy;

    // Opponent goal
    auto [tgx, tgy] = to_rl_pos(field_dimensions_.their_goal_loc());
    auto [ntgx, ntgy] = norm_pos(tgx, tgy);
    obs[idx++] = ntgx;
    obs[idx++] = ntgy;

    // Teammate positions (up to kNumTeammates, excluding self)
    int tm_count = 0;
    for (int i = 0; i < static_cast<int>(kNumShells) && tm_count < kNumTeammates; ++i) {
        if (i == robot_id_) continue;
        const auto& tm = last_world_state_->our_robots[i];
        if (!tm.visible) continue;
        auto [tx, ty] = to_rl_pos(tm.pose.position());
        auto [ntx, nty] = norm_pos(tx, ty);
        obs[idx++] = ntx;
        obs[idx++] = nty;
        ++tm_count;
    }
    idx += (kNumTeammates - tm_count) * 2;

    // Opponent positions (up to kNumOpponents)
    int op_count = 0;
    for (int i = 0; i < static_cast<int>(kNumShells) && op_count < kNumOpponents; ++i) {
        const auto& op = last_world_state_->their_robots[i];
        if (!op.visible) continue;
        auto [ox, oy] = to_rl_pos(op.pose.position());
        auto [nox, noy] = norm_pos(ox, oy);
        obs[idx++] = nox;
        obs[idx++] = noy;
        ++op_count;
    }

    return obs;
}

// -----------------------------------------------------------------
// Action selection (deterministic argmax over softmax)
// -----------------------------------------------------------------

int RLPosition::select_action(const std::vector<float>& logits) const {
    return static_cast<int>(
        std::distance(logits.begin(), std::max_element(logits.begin(), logits.end())));
}

// -----------------------------------------------------------------
// Action → RobotIntent translation
// -----------------------------------------------------------------

std::optional<RobotIntent> RLPosition::action_to_intent(int action, RobotIntent intent) const {
    const auto& ball_pos = last_world_state_->ball.position;
    const auto& robot_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

    switch (action) {
        case MOVE_TO_BALL: {
            last_action_name_ = "MOVE_TO_BALL";
            intent.motion_command = planning::MotionCommand{"collect"};
            return intent;
        }

        case SHOOT_ON_GOAL: {
            last_action_name_ = "SHOOT_ON_GOAL";
            rj_geometry::Point shot_target = calculate_best_shot();
            planning::LinearMotionInstant target{shot_target};
            intent.motion_command =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            intent.trigger_mode = RobotIntent::TriggerMode::AT_END;
            intent.kick_speed = 4.0f;
            return intent;
        }

        case PASS_TO_NEAREST: {
            last_action_name_ = "PASS";
            int teammate_id = find_nearest_teammate();
            if (teammate_id < 0) {
                intent.motion_command = planning::MotionCommand{"collect"};
                return intent;
            }
            rj_geometry::Point tm_pos =
                last_world_state_->get_robot(true, teammate_id).pose.position();
            planning::LinearMotionInstant target{tm_pos};
            intent.motion_command =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            intent.trigger_mode = RobotIntent::TriggerMode::AT_END;

            double dist = tm_pos.dist_to(robot_pos);
            intent.kick_speed =
                static_cast<float>(std::sqrt(std::max(0.0, -2.0 * kBallDecel * dist)));
            return intent;
        }

        case DEFEND_GOAL: {
            last_action_name_ = "DEFEND_GOAL";
            auto own_goal = field_dimensions_.our_goal_loc();
            auto direction = ball_pos - own_goal;
            double dist = direction.mag();
            if (dist > 1e-6) {
                direction = direction / dist;
            }
            auto target_pt = own_goal + direction * std::min(dist * 0.3, 1.5);
            planning::LinearMotionInstant target{target_pt};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceBall{}};
            return intent;
        }

        case POSITION_OFFENSE: {
            last_action_name_ = "POS_OFFENSE";
            double field_len = field_dimensions_.length();
            double field_w = field_dimensions_.width();
            double target_y = field_len * 0.75;
            double target_x = std::clamp(ball_pos.x() + 1.0, -field_w / 2.0, field_w / 2.0);
            planning::LinearMotionInstant target{rj_geometry::Point{target_x, target_y}};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceBall{}};
            return intent;
        }

        case POSITION_DEFENSE: {
            last_action_name_ = "POS_DEFENSE";
            double field_len = field_dimensions_.length();
            double field_w = field_dimensions_.width();
            double target_y = field_len * 0.25;
            double target_x = std::clamp(static_cast<double>(ball_pos.x()),
                                         -field_w / 2.0, field_w / 2.0);
            planning::LinearMotionInstant target{rj_geometry::Point{target_x, target_y}};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceBall{}};
            return intent;
        }

        case RL_IDLE:
        default: {
            last_action_name_ = "IDLE";
            intent.motion_command = planning::MotionCommand{};
            return intent;
        }
    }
}

rj_geometry::Point RLPosition::calculate_best_shot() const {
    rj_geometry::Point their_goal = field_dimensions_.their_goal_loc();
    double goal_w = field_dimensions_.goal_width();
    rj_geometry::Point ball_pos = last_world_state_->ball.position;

    rj_geometry::Point best = their_goal;
    double best_dist = -1.0;
    rj_geometry::Point inc{0.05, 0};
    rj_geometry::Point curr = their_goal - rj_geometry::Point{goal_w / 2.0, 0} + inc;

    for (int i = 0; i < 19; ++i) {
        rj_geometry::Point vec = curr - ball_pos;
        double min_clearance = std::numeric_limits<double>::max();

        for (const auto& enemy : last_world_state_->their_robots) {
            if (!enemy.visible) continue;
            rj_geometry::Point enemy_vec = enemy.pose.position() - ball_pos;
            if (enemy_vec.dot(vec) < 0) continue;
            double proj = enemy_vec.dot(vec) / vec.dot(vec);
            rj_geometry::Point perp = enemy_vec - proj * vec;
            min_clearance = std::min(min_clearance, perp.mag());
        }

        if (min_clearance > best_dist) {
            best_dist = min_clearance;
            best = curr;
        }
        curr = curr + inc;
    }
    return best;
}

int RLPosition::find_nearest_teammate() const {
    auto robot_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    double min_dist = std::numeric_limits<double>::max();
    int best_id = -1;

    for (int i = 0; i < static_cast<int>(kNumShells); ++i) {
        if (i == robot_id_) continue;
        const auto& tm = last_world_state_->our_robots[i];
        if (!tm.visible) continue;
        double d = tm.pose.position().dist_to(robot_pos);
        if (d < min_dist) {
            min_dist = d;
            best_id = i;
        }
    }
    return best_id;
}

// -----------------------------------------------------------------
// Main tick: encode → forward → act
// -----------------------------------------------------------------

std::optional<RobotIntent> RLPosition::derived_get_task(RobotIntent intent) {
    if (!loaded_ && !load_weights()) {
        SPDLOG_WARN("RLPosition: no weights, falling back to idle");
        intent.motion_command = planning::MotionCommand{};
        return intent;
    }

    auto obs = encode_state();
    auto logits = forward(obs);
    int action = select_action(logits);
    return action_to_intent(action, std::move(intent));
}

// -----------------------------------------------------------------
// Stubs for communication interface
// -----------------------------------------------------------------

void RLPosition::derived_acknowledge_pass() {}
void RLPosition::derived_pass_ball() {}
void RLPosition::derived_acknowledge_ball_in_transit() {}

std::string RLPosition::get_current_state() {
    return "RL:" + last_action_name_;
}

}  // namespace strategy
