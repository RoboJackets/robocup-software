
#include <cmath>
#include <iostream>

#include <control/trapezoidal_motion.hpp>
#include <rj_geometry/point.hpp>

#include "gtest/gtest.h"

using namespace std;

bool trapezoid1(double t, double& pos_out, double& speed_out) {
    double path_length = 10;
    double max_speed = 2;
    double max_acc = 1;
    double start_speed = 0;
    double final_speed = 0;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        auto time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                          final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

bool trapezoid2(double t, double& pos_out, double& speed_out) {
    double path_length = 9.5;
    double max_speed = 2;
    double max_acc = 1;
    double start_speed = 1;
    double final_speed = 0;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        double time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                            final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

// Triangular path of length 2
// start_speed = 0, high_speed = 1, end_speed = 0, average = 0.5
// time = 4 seconds
bool triangle1(double t, double& pos_out, double& speed_out) {
    double path_length = 2;
    double max_speed = 4;
    double max_acc = 0.5;
    double start_speed = 0;
    double final_speed = 0;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        double time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                            final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

// Triangle path of length 8
// start_speed = 0, high_speed = 2, end_speed = 0
// time = 8 seconds
bool triangle2(double t, double& pos_out, double& speed_out) {
    double path_length = 8;
    double max_speed = 4;
    double max_acc = 0.5;
    double start_speed = 0;
    double final_speed = 0;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        double time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                            final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

// Triangle path of length 6
// start_speed = 1, high_speed = 2, end_speed = 1
// time = 4 seconds
bool triangle3(double t, double& pos_out, double& speed_out) {
    double path_length = 6;
    double max_speed = 4;
    double max_acc = 0.5;
    double start_speed = 1;
    double final_speed = 1;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        double time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                            final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

// Triangle path of length 6
// start_speed = 1, high_speed = 2, end_speed = 1
// time = 4 seconds
bool triangle4(double t, double& pos_out, double& speed_out) {
    double path_length = 7;
    double max_speed = 4;
    double max_acc = 0.5;
    double start_speed = 1;
    double final_speed = 0;

    bool valid = Trapezoidal::trapezoidal_motion(path_length, max_speed, max_acc, t, start_speed, final_speed,
                                    pos_out, speed_out);

    if (valid) {
        double time = Trapezoidal::get_time(pos_out, path_length, max_speed, max_acc, start_speed,
                                            final_speed);
        EXPECT_NEAR(time, t, 0.001);
    }
    return valid;
}

TEST(TrapezoidalMotion, pre_start) {
    // make sure it gives good values for negative t values
    double pos_out, speed_out;
    bool path_valid = trapezoid1(-2, pos_out, speed_out);
    EXPECT_FALSE(path_valid);
    EXPECT_NEAR(pos_out, 0, 0.001);
    EXPECT_NEAR(speed_out, 0, 0.001);
}

TEST(TrapezoidalMotion, start) {
    // beginning of the trapezoid, t = 0
    double pos_out, speed_out;
    bool path_valid = trapezoid1(0, pos_out, speed_out);
    EXPECT_NEAR(speed_out, 0, 0.001);
    EXPECT_NEAR(pos_out, 0, 0.001);
    EXPECT_TRUE(path_valid);
}

TEST(TrapezoidalMotion, end) {
    // way after the trapezoid finishes
    double pos_out, speed_out;
    bool path_valid = trapezoid1(50, pos_out, speed_out);
    EXPECT_NEAR(speed_out, 0, 0.001) << "Speed should be zero at the end of the run";
    EXPECT_NEAR(pos_out, 10, 0.001) << "Position should stay at end of path after path finishes";
    EXPECT_FALSE(path_valid);
}

TEST(TrapezoidalMotion2, more_tests) {
    for (double i = 0; i < 6; i += 0.01) {
        double pos_out1, speed_out1, pos_out2, speed_out2;
        bool path_valid1 = trapezoid1(i + 1, pos_out1, speed_out1);
        bool path_valid2 = trapezoid2(i, pos_out2, speed_out2);
        ASSERT_TRUE(path_valid1);
        ASSERT_TRUE(path_valid2);
        ASSERT_NEAR(pos_out1, pos_out2 + 0.5, 0.00001);
        ASSERT_NEAR(speed_out1, speed_out2, 0.00001);
    }
}

TEST(TrapezoidalMotion3, more_tests) {
    double pos_out = 1.11101;
    double path_length = 1.11101;
    double max_speed = 2.2;
    double max_acc = 1;
    double start_speed = 0.901393;
    double final_speed = 0;

    double result = Trapezoidal::get_time(2.03294, 2.03294, 2.2, 1, 0.176091, 0);
    EXPECT_FALSE(std::isnan(result));
}

TEST(TrapezoidalMotion, TriangleRampUp) {
    double pos_out, speed_out;
    bool path_valid = triangle1(1, pos_out, speed_out);
    EXPECT_TRUE(path_valid);
    EXPECT_NEAR(pos_out, 0.5 * 0.5, 0.00001);
    EXPECT_NEAR(speed_out, 0.5, 0.00001);

    path_valid = triangle1(2, pos_out, speed_out);
    EXPECT_TRUE(path_valid);
    EXPECT_NEAR(pos_out, 1, 0.00001);
    EXPECT_NEAR(speed_out, 1, 0.00001);

    path_valid = triangle1(3, pos_out, speed_out);
    EXPECT_TRUE(path_valid);
    EXPECT_NEAR(pos_out, 2 - 0.5 * 0.5, 0.00001);
    EXPECT_NEAR(speed_out, 0.5, 0.00001);

    path_valid = triangle1(4, pos_out, speed_out);
    EXPECT_FALSE(path_valid);
    EXPECT_NEAR(pos_out, 2, 0.00001);
    EXPECT_NEAR(speed_out, 0, 0.00001);
}

TEST(TrapezoidalMotion2, TriangleRampUp) {
    bool path_valid;
    double pos_out, speed_out;
    for (double i = 0; i < 4; i += 0.01) {
        path_valid = triangle2(i, pos_out, speed_out);
        ASSERT_TRUE(path_valid);
        ASSERT_NEAR(speed_out, i / 2, 0.00001);
    }

    path_valid = triangle2(4, pos_out, speed_out);
    EXPECT_TRUE(path_valid);
    EXPECT_NEAR(pos_out, 4, 0.00001);
    EXPECT_NEAR(speed_out, 2, 0.00001);

    for (double i = 0; i <= 4; i += 0.01) {
        path_valid = triangle2(i + 4, pos_out, speed_out);
        ASSERT_TRUE(path_valid);
        ASSERT_NEAR(speed_out, 2 - i / 2, 0.00001);
    }

    double i = 4;
    path_valid = triangle2(i + 4, pos_out, speed_out);
    ASSERT_FALSE(path_valid);
    ASSERT_NEAR(speed_out, 2 - i / 2, 0.00001);
}

TEST(TrapezoidalMotion3, triangle_ramp_up) {
    for (double i = 0; i < 4; i += 0.01) {
        double pos_out2, speed_out2, pos_out3, speed_out3;
        bool path_valid2 = triangle2(2 + i, pos_out2, speed_out2);
        bool path_valid3 = triangle3(i, pos_out3, speed_out3);
        ASSERT_EQ(path_valid2, path_valid3);
        ASSERT_NEAR(pos_out2, pos_out3 + 1, 0.00001);
        ASSERT_NEAR(speed_out2, speed_out3, 0.00001);
    }

    for (double i = 0; i < 6; i += 0.01) {
        double pos_out2, speed_out2, pos_out4, speed_out4;
        bool path_valid2 = triangle2(2 + i, pos_out2, speed_out2);
        bool path_valid4 = triangle4(i, pos_out4, speed_out4);
        ASSERT_EQ(path_valid2, path_valid4);
        ASSERT_NEAR(pos_out2, pos_out4 + 1, 0.00001);
        ASSERT_NEAR(speed_out2, speed_out4, 0.00001);
    }

    double i = 6;
    double pos_out2, speed_out2, pos_out4, speed_out4;
    bool path_valid2 = triangle2(2 + i, pos_out2, speed_out2);
    bool path_valid4 = triangle4(i, pos_out4, speed_out4);
    ASSERT_EQ(path_valid2, path_valid4);
    ASSERT_NEAR(pos_out2, pos_out4 + 1, 0.00001);
    ASSERT_NEAR(speed_out2, speed_out4, 0.00001);
}
