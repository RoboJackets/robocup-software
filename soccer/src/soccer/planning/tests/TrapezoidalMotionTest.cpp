#include <gtest/gtest.h>

#include "planning/primitives/TrapezoidalMotion.hpp"

using namespace Planning;

class TrapezoidalMotionTest : public ::testing::Test {
public:
    void run_test() {
        double dt = 0.005;
        double total_time =
            Trapezoid::time_remaining(initial_, goal_, max_velocity_, max_acceleration_);

        Trapezoid::State previous =
            Trapezoid::predict_in(initial_, goal_, max_velocity_, max_acceleration_, 0);

        // NOLINTNEXTLINE
        for (double t = dt; t <= total_time; t += dt) {
            Trapezoid::State next =
                Trapezoid::predict_in(initial_, goal_, max_velocity_, max_acceleration_, t);

            double velocity_estimate = (next.position - previous.position) / dt;
            double acceleration_estimate = (next.velocity - previous.velocity) / dt;

            EXPECT_NEAR(velocity_estimate, (previous.velocity + next.velocity) / 2, 1.5e-3);
            EXPECT_LE(std::abs(acceleration_estimate), max_acceleration_ + 1e-6);

            previous = next;
        }

        Trapezoid::State final =
            Trapezoid::predict_in(initial_, goal_, max_velocity_, max_acceleration_, total_time);
        EXPECT_NEAR(final.position, goal_.position, 1e-6);
        EXPECT_NEAR(final.velocity, goal_.velocity, 1e-6);
    }

protected:
    double max_acceleration_ = 1.0;
    double max_velocity_ = 1.0;
    Trapezoid::State initial_{};
    Trapezoid::State goal_{};
};

TEST_F(TrapezoidalMotionTest, Time) {
    initial_ = Trapezoid::State{0, 0};
    goal_ = Trapezoid::State{1, 0};
    double total_time = Trapezoid::time_remaining(initial_, goal_, max_velocity_, max_acceleration_);
    EXPECT_NEAR(total_time, 2, 1e-6);
}

TEST_F(TrapezoidalMotionTest, TriangleProfile) {
    initial_ = Trapezoid::State{0, 0};
    goal_ = Trapezoid::State{1, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, ShortTriangleProfile) {
    initial_ = Trapezoid::State{0, 0};
    goal_ = Trapezoid::State{0.5, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, TrapezoidProfile) {
    initial_ = Trapezoid::State{0, 0};
    goal_ = Trapezoid::State{2, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, NegativeGoal) {
    initial_ = Trapezoid::State{0, 0};
    goal_ = Trapezoid::State{-2, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, InitialVelocity) {
    initial_ = Trapezoid::State{0, 0.5};
    goal_ = Trapezoid::State{2, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, LargeInitialVelocity) {
    initial_ = Trapezoid::State{0, 1};
    goal_ = Trapezoid::State{0, 0};
    run_test();
}

TEST_F(TrapezoidalMotionTest, NegativeVelocity) {
    initial_ = Trapezoid::State{0, -1};
    goal_ = Trapezoid::State{2, 0};
    run_test();
}

// TODO(#1513): Re-enable this test.
TEST_F(TrapezoidalMotionTest, DISABLED_VeryLargeInitialVelocity) {
    // Initial velocity violates constraints. We should decelerate to constant,
    // then decelerate to zero.
    initial_ = Trapezoid::State{0, 2};
    goal_ = Trapezoid::State{4, 0};
    run_test();
}
