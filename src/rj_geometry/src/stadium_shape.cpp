#include <rj_geometry/stadium_shape.hpp>

namespace rj_geometry {

Shape* StadiumShape::clone() const { return new StadiumShape(*this); }

StadiumShape::StadiumShape(Point c1, Point c2, float r) {
    // Create the two circular end caps
    auto c1_obs_ptr = std::make_shared<Circle>(c1, r);
    auto c2_obs_ptr = std::make_shared<Circle>(c2, r);

    // Create the rectangular middle section connecting the circles
    Point leftToRight{c2.x() - c1.x(), c2.y() - c1.y()};
    Point leftToRightN = leftToRight.norm().perp_ccw();

    Point leftTop = c1 + (leftToRightN * r);
    Point leftBottom = c1 - (leftToRightN * r);
    Point rightTop = c2 + (leftToRightN * r);
    Point rightBottom = c2 - (leftToRightN * r);

    std::vector<Point> verts = {leftTop, rightTop, rightBottom, leftBottom};
    auto rect_obs_ptr = std::make_shared<Polygon>(verts);

    // Use CompositeShape's add() method
    add(c1_obs_ptr);
    add(rect_obs_ptr);
    add(c2_obs_ptr);

    // Also store in drawshapes_ for backward compatibility
    drawshapes_.add(c1_obs_ptr);
    drawshapes_.add(rect_obs_ptr);
    drawshapes_.add(c2_obs_ptr);
}

}  // namespace rj_geometry