#pragma once

#include <rj_common/field_dimensions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/shape_set.hpp>
#include <rj_planning/obstacle_set.hpp>
#include <rj_rrt/2dplane/PlaneStateSpace.hpp>

namespace planning {

/**
 * Represents the robocup field for path-planning purposes.
 */
class RoboCupStateSpace : public RRT::StateSpace<rj_geometry::Point> {
public:
    RoboCupStateSpace(const FieldDimensions& dims, const ObstacleSet& obstacles)
        : obstacles_(obstacles), field_dimensions_(dims) {}

    rj_geometry::Point randomState() const override {
        double x = field_dimensions_.floor_width() * (drand48() - 0.5f);
        double y = field_dimensions_.floor_length() * drand48() -
                   field_dimensions_.border();
        return rj_geometry::Point(x, y);
    }

    double distance(const rj_geometry::Point& from,
                    const rj_geometry::Point& to) const override {
        return from.dist_to(to);
    }

    bool stateValid(const rj_geometry::Point& state) const override {
        // note: obstacles_ contains obstacles that define the limits of the
        // field, so we shouldn't have to check separately that the point is
        // within the field boundaries.

        return !obstacles_.hit(state);
    }

    rj_geometry::Point intermediateState(const rj_geometry::Point& source,
                                        const rj_geometry::Point& target,
                                        double step_size) const override {
        auto dir = (target - source).norm();
        return source + dir * step_size;
    }

    rj_geometry::Point intermediateState(const rj_geometry::Point& source,
                                        const rj_geometry::Point& target,
                                        double min_step_size,
                                        double max_step_size) const override {
        throw std::runtime_error("Adaptive stepsize control not implemented");
    }

    bool transitionValid(const rj_geometry::Point& from,
                         const rj_geometry::Point& to) const override {
        // Ensure that @to doesn't hit any obstacles that @from doesn't. This
        // allows the RRT to start inside an obstacle, but prevents it from
        // entering a new obstacle.
        rj_geometry::Segment seg(from, to);

        // Check if segment hits any obstacle that the start point doesn't hit
        for (const auto& obs : obstacles_.obstacles()) {
            if (obs->hit(seg) && !obs->hit(from)) {
                return false;
            }
        }
        return true;
    }

private:
    const ObstacleSet& obstacles_;
    const FieldDimensions field_dimensions_;
};

}  // namespace planning
