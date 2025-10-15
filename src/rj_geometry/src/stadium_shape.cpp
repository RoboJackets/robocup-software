#include <rj_geometry/stadium_shape.hpp>

namespace rj_geometry {

Shape* StadiumShape::clone() const { return new StadiumShape(*this); }

void StadiumShape::init(Point c1, Point c2, float r) {
    rj_geometry::Circle first_circle = rj_geometry::Circle{c1, static_cast<float>(r)};
    rj_geometry::Circle second_circle = rj_geometry::Circle{c2, static_cast<float>(r)};

    rj_geometry::Segment vect{c1, c2};
    rj_geometry::Point leftToRight{c2.x() - c1.x(), c2.y() - c1.y()};
    rj_geometry::Point leftToRightN = leftToRight.norm().perp_ccw();

    rj_geometry::Point leftTop = c1 + (leftToRightN * r);
    rj_geometry::Point leftBottom = c1 - (leftToRightN * r);
    rj_geometry::Point rightTop = c2 + (leftToRightN * r);
    rj_geometry::Point rightBottom = c2 - (leftToRightN * r);

    std::vector<Point> verts{};
    verts.push_back(leftTop);
    verts.push_back(leftBottom);
    verts.push_back(rightBottom);
    verts.push_back(rightTop);

    rj_geometry::Polygon rect_obs{verts};

    std::shared_ptr<rj_geometry::Circle> c1_obs_ptr =
        std::make_shared<rj_geometry::Circle>(first_circle);
    std::shared_ptr<rj_geometry::Polygon> rect_obs_ptr =
        std::make_shared<rj_geometry::Polygon>(rect_obs);
    std::shared_ptr<rj_geometry::Circle> c2_obs_ptr =
        std::make_shared<rj_geometry::Circle>(second_circle);

    subshapes_.push_back(c1_obs_ptr);
    subshapes_.push_back(rect_obs_ptr);
    subshapes_.push_back(c2_obs_ptr);

    drawshapes_.add(c1_obs_ptr);
    drawshapes_.add(rect_obs_ptr);
    drawshapes_.add(c2_obs_ptr);
}

bool StadiumShape::contains_point(Point pt) const {
    for (const auto& subshape : subshapes_) {
        if (subshape->contains_point(pt)) {
            return true;
        }
    }
    return false;
}

bool StadiumShape::near_point(Point pt, float threshold) const {
    for (const auto& subshape : subshapes_) {
        if (subshape->near_point(pt, threshold)) {
            return true;
        }
    }
    return false;
}

}  // namespace rj_geometry