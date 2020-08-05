#include <cmath>

#include <Geometry2d/TransformMatrix.hpp>
#include <Geometry2d/Util.hpp>

namespace Geometry2d {

const TransformMatrix TransformMatrix::kIdentity(1, 0, 0, 0, 1, 0);

const TransformMatrix TransformMatrix::kMirrorX(-1, 0, 0, 0, 1, 0);

TransformMatrix::TransformMatrix(Point origin, float rotation, bool mirror,
                                 float s) {
    // Set up translation
    _m[0] = 1;
    _m[1] = 0;
    _m[2] = static_cast<float>(origin.x());
    _m[3] = 0;
    _m[4] = 1;
    _m[5] = static_cast<float>(origin.y());

    *this *= rotate(rotation);
    *this *= scale(s);
    if (mirror) {
        *this *= kMirrorX;
    }
}

float TransformMatrix::transformAngle(float angle) const {
    // Multiply the matrix by a unit vector in the given direction with a 3rd
    // element of zero and find the direction of the result.

    float px = std::cos(angle);
    float py = std::sin(angle);

    float rx = px * _m[0] + py * _m[1];
    float ry = px * _m[3] + py * _m[4];

    return std::atan2(ry, rx);
}

TransformMatrix TransformMatrix::rotateAroundPoint(const Point& center,
                                                   float angle) {
    TransformMatrix xf = translate(center);
    xf *= rotate(angle);
    xf *= translate(-center);

    return xf;
}

TransformMatrix TransformMatrix::mirrorAroundPoint(const Point& center) {
    TransformMatrix xf = translate(center);
    xf *= kMirrorX;
    xf *= translate(-center);

    return xf;
}

float TransformMatrix::rotation() const {
    auto angle = static_cast<float>(std::atan2(_m[4], _m[1]) - M_PI_2);
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

bool TransformMatrix::mirrored() const {
    // This matrix contains a reflection iff the Z component of the transformed
    // x vector cross the transformed y vector is negative.  Conveniently, this
    // is the determinant of the matrix formed by the two vectors.
    //
    // In a right-handed coordinate system, +X cross +Y = +Z. A reflection flips
    // the coordinate system to be left-handed so the (right-handed) cross
    // product becomes -Z.
    return (_m[0] * _m[4] - _m[1] * _m[3]) < 0;
}

}  // namespace Geometry2d
