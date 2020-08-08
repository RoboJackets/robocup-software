#include <random>

#include <gtest/gtest.h>

#include <world_state.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

RobotState get_random_robot_state() {
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    auto rand = [&]() -> double { return uniform(e1); };

    const Geometry2d::Point position{rand(), rand()};
    const double heading = rand();
    const Geometry2d::Point linear{rand(), rand()};
    const double angular = rand();
    const RJ::Time timestamp = RJ::now();
    const bool visible = true;

    const Geometry2d::Pose pose{position, heading};
    const Geometry2d::Twist twist{linear, angular};
    return RobotState{pose, twist, timestamp, visible};
}

BallState get_random_ball_state() {
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    auto rand = [&]() -> double { return uniform(e1); };

    const Geometry2d::Point position{rand(), rand()};
    const Geometry2d::Point velocity{rand(), rand()};
    const RJ::Time timestamp = RJ::now();

    return BallState{position, velocity, timestamp};
}

WorldState get_random_world_state() {
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_int_distribution<int> uniform(0, 10);
    auto rand = [&]() -> int { return uniform(e1); };

    std::vector<RobotState> their_robots;
    for (int i = 0; i < rand(); i++) {
        their_robots.emplace_back(get_random_robot_state());
    }

    std::vector<RobotState> our_robots;
    for (int i = 0; i < rand(); i++) {
        our_robots.emplace_back(get_random_robot_state());
    }

    const BallState ball_state = get_random_ball_state();

    return WorldState{std::move(their_robots), std::move(our_robots), ball_state};
}

bool operator==(const RobotState& a, const RobotState& b) {
    return a.pose == b.pose && a.velocity == b.velocity && a.timestamp == b.timestamp &&
           a.visible == b.visible;
}

bool operator==(const BallState& a, const BallState& b) {
    return a.position == b.position && a.velocity == b.velocity && a.timestamp == b.timestamp &&
           a.visible == b.visible;
}

bool operator==(const WorldState& a, const WorldState& b) {
    // Check their_robots.
    if (a.their_robots.size() != b.their_robots.size()) {
        return false;
    }

    for (size_t robot_idx = 0; robot_idx < a.their_robots.size(); robot_idx++) {
        if (!(a.their_robots[robot_idx] == b.their_robots[robot_idx])) {
            return false;
        }
    }

    // Check our_robots.
    if (a.our_robots.size() != b.our_robots.size()) {
        return false;
    }

    for (size_t robot_idx = 0; robot_idx < a.our_robots.size(); robot_idx++) {
        if (!(a.our_robots[robot_idx] == b.our_robots[robot_idx])) {
            return false;
        }
    }

    // Check ball_state.
    return a.ball == b.ball;
}

TEST(ROSMsgConversionNoop, RobotState) {
    test_lossless_convert_cpp_value(get_random_robot_state());
}

TEST(ROSMsgConversionNoop, BallState) { test_lossless_convert_cpp_value(get_random_ball_state()); }

TEST(ROSMsgConversionNoop, WorldState) {
    test_lossless_convert_cpp_value(get_random_world_state());
}