#pragma once

#include <Eigen/Dense>

#include "Point.hpp"

namespace Geometry2d {
// A 2x3 transformation matrix.
//
// This is the 2D equivalent of the usual 3D tranformation matrix with the
// bottom row omitted because the bottom (third) element of a 2D point is always
// 1.  The third row of the matrix is understood to be [0 0 1].
class TransformMatrix {
public:
    TransformMatrix() {
        m_[0] = 1;
        m_[1] = 0;
        m_[2] = 0;
        m_[3] = 0;
        m_[4] = 1;
        m_[5] = 0;
    }

    TransformMatrix(float a, float b, float c, float d, float e, float f) {
        m_[0] = a;
        m_[1] = b;
        m_[2] = c;
        m_[3] = d;
        m_[4] = e;
        m_[5] = f;
    }

    TransformMatrix(Geometry2d::Point origin, float rotation = 0,
                    bool mirror = false, float scale = 1);

    TransformMatrix(const Eigen::Matrix<double, 3, 3>& other) {
        m_[0] = other(0, 0);
        m_[1] = other(0, 1);
        m_[2] = other(0, 2);
        m_[3] = other(1, 0);
        m_[4] = other(1, 1);
        m_[5] = other(1, 2);
    }

    TransformMatrix operator*(const TransformMatrix& other) const {
        float a = m_[0] * other.m_[0] + m_[1] * other.m_[3];
        float b = m_[0] * other.m_[1] + m_[1] * other.m_[4];
        float c = m_[0] * other.m_[2] + m_[1] * other.m_[5] + m_[2];
        float d = m_[3] * other.m_[0] + m_[4] * other.m_[3];
        float e = m_[3] * other.m_[1] + m_[4] * other.m_[4];
        float f = m_[3] * other.m_[2] + m_[4] * other.m_[5] + m_[5];

        return TransformMatrix(a, b, c, d, e, f);
    }

    TransformMatrix& operator*=(const TransformMatrix& other) {
        float a = m_[0] * other.m_[0] + m_[1] * other.m_[3];
        float b = m_[0] * other.m_[1] + m_[1] * other.m_[4];
        float c = m_[0] * other.m_[2] + m_[1] * other.m_[5] + m_[2];
        float d = m_[3] * other.m_[0] + m_[4] * other.m_[3];
        float e = m_[3] * other.m_[1] + m_[4] * other.m_[4];
        float f = m_[3] * other.m_[2] + m_[4] * other.m_[5] + m_[5];

        m_[0] = a;
        m_[1] = b;
        m_[2] = c;
        m_[3] = d;
        m_[4] = e;
        m_[5] = f;

        return *this;
    }

    operator Eigen::Matrix<double, 3, 3>() const {
        Eigen::Matrix<double, 3, 3> result;
        result << m_[0], m_[1], m_[2], m_[3], m_[4], m_[5], 0, 0, 1;
        return result;
    }

    Point operator*(const Point& pt) const {
        return Point(pt.x() * m_[0] + pt.y() * m_[1] + m_[2],
                     pt.x() * m_[3] + pt.y() * m_[4] + m_[5]);
    }

    // Transforms a direction vector (3rd element is zero)
    Point transform_direction(const Point& dir) const {
        return Point(dir.x() * m_[0] + dir.y() * m_[1],
                     dir.x() * m_[3] + dir.y() * m_[4]);
    }

    // Transforms the given angle in radians
    float transform_angle(float angle) const;

    // Returns the vector that represents the direction of the transformed
    // X-axis.
    Point x() const { return Point(m_[0], m_[3]); }

    // Returns the vector that represents the direction of the transformed
    // Y-axis.
    Point y() const { return Point(m_[1], m_[4]); }

    // Returns the origin of the transformed coordinate system
    Point origin() const { return Point(m_[2], m_[5]); }

    // Returns the scaling along the transformed X-axis.
    float x_scale() const { return x().mag(); }

    // Returns the scaling along the transformed Y-axis.
    float y_scale() const { return y().mag(); }

    // Returns the clockwise angle from the transformed Y axis to the original Y
    // axis. This is not affected by horizontal reflection.
    float rotation() const;

    // Returns true if the coordinate system has been mirrored (i.e. is now
    // left-handed).
    bool mirrored() const;

    const float* m() const { return m_; }

    ////////////////
    // Functions to build common transformations:

    // Translation
    static TransformMatrix translate(const Point& delta) {
        return TransformMatrix(1, 0, delta.x(), 0, 1, delta.y());
    }

    static TransformMatrix translate(float x, float y) {
        return translate(Point(x, y));
    }

    // Rotation in radians around origin
    static TransformMatrix rotate(float angle) {
        float c = cos(angle);
        float s = sin(angle);

        return TransformMatrix(c, -s, 0, s, c, 0);
    }

    // Uniform scale
    static TransformMatrix scale(float s) {
        return TransformMatrix(s, 0, 0, 0, s, 0);
    }

    // Non-uniform scale
    static TransformMatrix scale(float x, float y) {
        return TransformMatrix(x, 0, 0, 0, y, 0);
    }

    // Returns a matrix to rotate <angle> radians CCW around <center>.
    static TransformMatrix rotate_around_point(const Point& center, float angle);

    // Returns a matrix to reflect along the line parallel to the Y-axis
    // containing <center>.
    static TransformMatrix mirror_around_point(const Point& center);

    static const TransformMatrix kIdentity;
    static const TransformMatrix kMirrorX;

protected:
    // Matrix values in row-major order.
    //
    // Indices:
    //   [0 1 2
    //    3 4 5]
    float m_[6]{};
};
}  // namespace Geometry2d
