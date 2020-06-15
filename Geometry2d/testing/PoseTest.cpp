#include <gtest/gtest.h>
#include "Geometry2d/Pose.hpp"

using namespace Geometry2d;

TEST(Pose, Constructors) {
    // Default constructor
    {
        Pose pose;
        EXPECT_EQ(pose.position().x(), 0);
        EXPECT_EQ(pose.position().y(), 0);
        EXPECT_EQ(pose.heading(), 0);
    }

    // Eigen matrix constructor (and conversion)
    {
        Eigen::Vector3d eigen(0, 1, 2);
        Pose pose(eigen);
        EXPECT_EQ(pose.position().x(), 0);
        EXPECT_EQ(pose.position().y(), 1);
        EXPECT_EQ(pose.heading(), 2);

        Eigen::Vector3d original = pose;
        EXPECT_EQ(original(0), eigen(0));
        EXPECT_EQ(original(1), eigen(1));
        EXPECT_EQ(original(2), eigen(2));
    }
}

TEST(Pose, Operators) {
    Pose a(1, 2, -1);
    Pose b(3, 4, 2);

    EXPECT_EQ((a + b).position(), Point(4, 6));
    EXPECT_EQ((a + b).heading(), 1);

    EXPECT_EQ((a - b).position(), Point(-2, -2));
    EXPECT_EQ((a - b).heading(), -3);

    EXPECT_EQ((a * 2).position(), Point(2, 4));
    EXPECT_EQ((a * 2).heading(), -2);

    EXPECT_EQ((a / 2).position(), Point(0.5, 1));
    EXPECT_EQ((a / 2).heading(), -0.5);

    // Stream operator
    {
        std::stringstream stream;
        stream << a;
        EXPECT_EQ(stream.str(), "Pose(1, 2, -1)");
    }
}

TEST(Pose, Transforms) {
    // Only linear offsets
    {
        Pose pose1(Point(1, 2), 0);
        Pose pose2(Point(3, 4), 0);
        Pose composed = pose1.withOrigin(pose2);
        EXPECT_EQ(composed.position().x(), 4);
        EXPECT_EQ(composed.position().y(), 6);
        EXPECT_EQ(composed.heading(), 0);
    }

    // Translation and rotation
    {
        Pose pose1(Point(1, 2), M_PI / 2);
        Pose pose2(Point(0, 1), 0);
        Pose composed = pose2.withOrigin(pose1);
        EXPECT_EQ(composed.position().x(), 0);
        EXPECT_EQ(composed.position().y(), 2);
        EXPECT_EQ(composed.heading(), M_PI / 2);
    }
}

TEST(Twist, ExponentialMapping) {
    // Curve from (0, 0) to (1, 1):
    //                |
    //                |
    //               /
    //              /
    //            _/
    //          _/
    //     ____/
    // +---
    {
        Twist twist(Point(1, 0), 1);
        Pose applied = twist.deltaRelative(M_PI / 2);
        EXPECT_NEAR(applied.position().x(), 1, 1e-6);
        EXPECT_NEAR(applied.position().y(), 1, 1e-6);
        EXPECT_NEAR(applied.heading(), M_PI / 2, 1e-6);
    }

    // Curve from (0, 0) to (1, 1):
    //         ____---
    //       _/
    //     _/
    //    /
    //   /
    //  /
    // |
    // |
    // +
    {
        Twist twist(Point(0, 1), -1);
        Pose applied = twist.deltaRelative(M_PI / 2);
        EXPECT_NEAR(applied.position().x(), 1, 1e-6);
        EXPECT_NEAR(applied.position().y(), 1, 1e-6);
        EXPECT_NEAR(applied.heading(), -M_PI / 2, 1e-6);
    }

    // Go straight up for 2 seconds
    {
        Twist twist(Point(0, 1), 0);
        Pose applied = twist.deltaRelative(2);
        EXPECT_NEAR(applied.position().x(), 0, 1e-6);
        EXPECT_NEAR(applied.position().y(), 2, 1e-6);
        EXPECT_NEAR(applied.heading(), 0, 1e-6);
    }

    // Spin in place
    {
        Twist twist(Point(0, 0), 1);
        Pose applied = twist.deltaRelative(1);
        EXPECT_NEAR(applied.position().x(), 0, 1e-6);
        EXPECT_NEAR(applied.position().y(), 0, 1e-6);
        EXPECT_NEAR(applied.heading(), 1, 1e-6);
    }

    // Sit still
    {
        Twist twist(Point(0, 0), 0);
        Pose applied = twist.deltaRelative(1);
        EXPECT_NEAR(applied.position().x(), 0, 1e-6);
        EXPECT_NEAR(applied.position().y(), 0, 1e-6);
        EXPECT_NEAR(applied.heading(), 0, 1e-6);
    }

    // Move 2PI in a circle (of radius 1) to end up where we started
    {
        Twist twist(Point(0, 1), 1);
        Pose applied = twist.deltaRelative(2 * M_PI);
        EXPECT_NEAR(applied.position().x(), 0, 1e-6);
        EXPECT_NEAR(applied.position().y(), 0, 1e-6);
        EXPECT_NEAR(applied.heading(), 2 * M_PI, 1e-6);
    }
}

TEST(Twist, Curvature) {
    // Standard
    {
        Twist twist(Point(1, 0), 2);
        EXPECT_NEAR(twist.curvature(), 2, 1e-5);
    }

    // Zero
    {
        Twist twist(Point(1, 0), 0);
        EXPECT_NEAR(twist.curvature(), 0, 1e-5);
    }

    // Infinite
    {
        Twist twist(Point(0, 0), 1);
        EXPECT_EQ(twist.curvature(), std::numeric_limits<double>::infinity());
    }
}

TEST(Twist, Operators) {
    Twist a(1, 2, -1);
    Twist b(3, 4, 2);

    EXPECT_EQ((a + b).linear(), Point(4, 6));
    EXPECT_EQ((a + b).angular(), 1);

    EXPECT_EQ((a - b).linear(), Point(-2, -2));
    EXPECT_EQ((a - b).angular(), -3);

    EXPECT_EQ((a * 2).linear(), Point(2, 4));
    EXPECT_EQ((a * 2).angular(), -2);

    EXPECT_EQ((a / 2).linear(), Point(0.5, 1));
    EXPECT_EQ((a / 2).angular(), -0.5);

    // Stream operator
    {
        std::stringstream stream;
        stream << a;
        EXPECT_EQ(stream.str(), "Twist(1, 2, -1)");
    }
}
