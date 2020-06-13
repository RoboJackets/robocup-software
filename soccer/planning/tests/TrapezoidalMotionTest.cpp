#include <gtest/gtest.h>

#include "planning/low_level/TrapezoidalMotion.hpp"

using namespace Planning;

class TrapezoidalMotionTest : public ::testing::Test {
public:
    void RunTest() {
        double dt = 0.005;
        double total_time = Trapezoid::timeRemaining(initial,
                                                     goal,
                                                     max_velocity,
                                                     max_acceleration);

        Trapezoid::State previous = Trapezoid::predictIn(initial,
                                                         goal,
                                                         max_velocity,
                                                         max_acceleration,
                                                         0);

        // NOLINTNEXTLINE
        for (double t = dt; t <= total_time; t += dt) {
            Trapezoid::State next = Trapezoid::predictIn(initial,
                                                         goal,
                                                         max_velocity,
                                                         max_acceleration,
                                                         t);

            double velocity_estimate = (next.position - previous.position) / dt;
            double acceleration_estimate =
                (next.velocity - previous.velocity) / dt;

            EXPECT_NEAR(velocity_estimate,
                        (previous.velocity + next.velocity) / 2,
                        1.5e-3);
            EXPECT_LE(std::abs(acceleration_estimate),
                      max_acceleration + 1e-6);

            previous = next;
        }

        Trapezoid::State final = Trapezoid::predictIn(initial,
                                                      goal,
                                                      max_velocity,
                                                      max_acceleration,
                                                      total_time);
        EXPECT_NEAR(final.position, goal.position, 1e-6);
        EXPECT_NEAR(final.velocity, goal.velocity, 1e-6);
    }

    /**
     * Test trapezoidal motion with exact end times. Returns true if the motion
     * could be completed, false, if it was requested to finish too quickly.
     */
    bool RunTestExactDuration(double duration) {
        double dt = 0.005;

        std::optional<Trapezoid::State> maybe_previous =
            Trapezoid::predictWithExactEndTime(initial,
                                               goal,
                                               max_velocity,
                                               max_acceleration,
                                               0,
                                               duration);

        if (!maybe_previous.has_value()) {
            return false;
        }

        Trapezoid::State previous = maybe_previous.value();

        // NOLINTNEXTLINE
        for (double t = dt; t <= duration; t += dt) {
            std::optional<Trapezoid::State> maybe_next =
                Trapezoid::predictWithExactEndTime(initial,
                                                   goal,
                                                   max_velocity,
                                                   max_acceleration,
                                                   t,
                                                   duration);

            EXPECT_TRUE(maybe_next.has_value())
                << "Trapezoidal motion success should be consistent regardless"
                   " of evaluation time.";
            if (!maybe_next.has_value()) {
                return false;
            }

            Trapezoid::State next = maybe_next.value();

            std::cout << next.position << ", " << next.velocity << std::endl;
            double velocity_estimate = (next.position - previous.position) / dt;
            double acceleration_estimate =
                (next.velocity - previous.velocity) / dt;

            EXPECT_NEAR(velocity_estimate,
                        (previous.velocity + next.velocity) / 2,
                        1.5e-3);
            EXPECT_LE(std::abs(acceleration_estimate),
                      max_acceleration + 1e-6);

            previous = next;
        }

        std::optional<Trapezoid::State> maybe_final =
            Trapezoid::predictWithExactEndTime(initial,
                                               goal,
                                               max_velocity,
                                               max_acceleration,
                                               duration,
                                               duration);
        EXPECT_TRUE(maybe_final.has_value());
        if (!maybe_final.has_value()) {
            return false;
        }

        Trapezoid::State final = maybe_final.value();
        EXPECT_NEAR(final.position, goal.position, 1e-6);
        EXPECT_NEAR(final.velocity, goal.velocity, 1e-6);
        return true;
    }

protected:
    double max_acceleration = 1.0;
    double max_velocity = 1.0;
    Trapezoid::State initial{};
    Trapezoid::State goal{};
};

TEST_F(TrapezoidalMotionTest, Time) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{1, 0};
    double total_time = Trapezoid::timeRemaining(initial,
                                                 goal,
                                                 max_velocity,
                                                 max_acceleration);
    EXPECT_NEAR(total_time, 2, 1e-6);
}

TEST_F(TrapezoidalMotionTest, TriangleProfile) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{1, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, ShortTriangleProfile) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{0.5, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, TrapezoidProfile) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{2, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, NegativeGoal) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{-2, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, InitialVelocity) {
    initial = Trapezoid::State{0, 0.5};
    goal = Trapezoid::State{2, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, LargeInitialVelocity) {
    initial = Trapezoid::State{0, 1};
    goal = Trapezoid::State{0, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, NegativeVelocity) {
    initial = Trapezoid::State{0, -1};
    goal = Trapezoid::State{2, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, VeryLargeInitialVelocity) {
    // Initial velocity violates constraints. We should decelerate to constant,
    // then decelerate to zero.
    initial = Trapezoid::State{0, 2};
    goal = Trapezoid::State{4, 0};
    RunTest();
}

TEST_F(TrapezoidalMotionTest, ExactEndTrapezoidProfile) {
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{2, 0};
    EXPECT_TRUE(RunTestExactDuration(5))
        << "Slow trapezoid is feasible";
}

TEST_F(TrapezoidalMotionTest, ExactEndTrapezoidProfileShort) {
    // This one is originally a triangle before it is cut off
    initial = Trapezoid::State{0, 0};
    goal = Trapezoid::State{0.5, 0};
    EXPECT_TRUE(RunTestExactDuration(5))
        << "Slow trapezoid is feasible";
}

TEST_F(TrapezoidalMotionTest, ExactEndTrapezoidInitialVelocity) {
    // Give it an initial velocity, but not bigger than the velocity it gets
    // truncated to.
    initial = Trapezoid::State{0, 0.2};
    goal = Trapezoid::State{2, 0};
    EXPECT_TRUE(RunTestExactDuration(5))
        << "Slow trapezoid is feasible";
}

TEST_F(TrapezoidalMotionTest, ExactEndTrapezoidInitialVelocityHigh) {
    // Give it an initial velocity bigger than what it gets truncated to.
    initial = Trapezoid::State{0, 1};
    goal = Trapezoid::State{2, 0};
    EXPECT_TRUE(RunTestExactDuration(5))
                    << "Slow trapezoid is feasible";
}
