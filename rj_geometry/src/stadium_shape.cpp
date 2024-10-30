#include <rj_geometry/stadium_shape.hpp>

namespace rj_geometry {

Shape* StadiumShape::clone() const { return new StadiumShape(*this); }

void StadiumShape::init(Point c1, Point c2, float r) {
    rj_geometry::Circle first_circle = rj_geometry::Circle{c1, static_cast<float>(r)};
    rj_geometry::Circle second_circle = rj_geometry::Circle{c2, static_cast<float>(r)};

    rj_geometry::Segment vect{c1, c2};

    rj_geometry::Point end1{c1.x() + r * (c2.x() - c1.x()) / vect.length(),
                            c1.y() + r * (c2.y() - c1.y()) / vect.length()};
    rj_geometry::Point end2{c2.x() - r * (c2.x() - c1.x()) / vect.length(),
                            c2.y() - r * (c2.y() - c1.y()) / vect.length()};

    rj_geometry::Segment vect_updated{end1, end2};
    rj_geometry::Polygon rect_obs{vect_updated, r};

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