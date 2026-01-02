#pragma once
#include <rj_common/context.hpp>
#include <rj_common/planning/robot_constraints.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_geometry/shape.hpp>
#include <rj_geometry/stadium_shape.hpp>

namespace planning {

class Obstacle {
public:
    Obstacle(std::shared_ptr<rj_geometry::Shape> obs, std::shared_ptr<rj_geometry::Shape> pad)
        : obstacle(obs), padding(pad) {}

    virtual ~Obstacle() = default;
    Obstacle(const Obstacle& other) = default;
    Obstacle(Obstacle&& other) = default;
    Obstacle& operator=(const Obstacle& other) = default;
    Obstacle& operator=(Obstacle&& other) = default;

    virtual bool obstacle_hit(rj_geometry::Point pt) { return obstacle->hit(pt); }

    virtual bool obstacle_hit(const rj_geometry::Segment& seg) { return obstacle->hit(seg); }

    virtual bool padding_hit(rj_geometry::Point pt) { return padding->hit(pt); }

    virtual bool padding_hit(const rj_geometry::Segment& seg) { return padding->hit(seg); }

    virtual bool obstacle_near(rj_geometry::Point pt, float thresh) {
        return obstacle->near_point(pt, thresh);
    }

    virtual bool padding_near(rj_geometry::Point pt, float thresh) {
        return padding->near_point(pt, thresh);
    }

    // Convenience methods: hit() delegates to padding_hit() for compatibility
    virtual bool hit(rj_geometry::Point pt) { return padding_hit(pt); }

    virtual bool hit(const rj_geometry::Segment& seg) { return padding_hit(seg); }

    virtual std::shared_ptr<rj_geometry::Shape> get_obstacle() { return obstacle; }

    virtual std::shared_ptr<rj_geometry::Shape> get_padding() { return padding; }

protected:
    std::shared_ptr<rj_geometry::Shape> obstacle;
    std::shared_ptr<rj_geometry::Shape> padding;
};

}  // namespace planning