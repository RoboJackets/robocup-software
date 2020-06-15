#include <gtest/gtest.h>

#include <Eigen/Dense>
#include "Geometry2d/TransformMatrix.hpp"

using namespace Geometry2d;

TEST(TransformMatrix, Convert) {
    // Test conversion to/from Eigen
    TransformMatrix transform(Point(0, 1), 1.0);
    Eigen::Matrix<double, 3, 3> transformEigen = transform;
    EXPECT_EQ(transformEigen * Eigen::Vector3d(0, 0, 1),
              Eigen::Vector3d(0, 1, 1));
    TransformMatrix transformedBack = transformEigen;
    EXPECT_EQ(transform * Point(0, 1), transformedBack * Point(0, 1));
    EXPECT_EQ(transform * Point(1, 0), transformedBack * Point(1, 0));
}

TEST(TransformMatrix, Compose) {
    // Test composition: self-compose a 90-degree rotation with an offset from
    // the origin
    TransformMatrix transform1(Point(0, 1), M_PI / 2);
    TransformMatrix transform2(Point(1, 0), M_PI / 2);

    EXPECT_LT(((transform1 * transform2) * Point(1, 0) -
               transform1 * (transform2 * Point(1, 0)))
                  .mag(),
              1e-6)
        << "Composition should be associative!";

    EXPECT_LT(((transform2 * transform1) * Point(1, 0) -
               transform2 * (transform1 * Point(1, 0)))
                  .mag(),
              1e-6)
        << "Composition should be associative!";
}

TEST(TransformMatrix, Reconstruct) {
    Eigen::Matrix<double, 3, 3> transformEigen;
    transformEigen << 0, 1, 0, -1, 0, 1, 0, 0, 1;
    TransformMatrix transform = transformEigen;
    EXPECT_EQ(transform.origin(), Point(0, 1));
    EXPECT_NEAR(transform.rotation(), -M_PI / 2 + 2 * M_PI, 1e-6);
}
