#pragma once

#include <memory>
#include <set>
#include <sstream>
#include <vector>

#include <rj_common/ros_debug_drawer.hpp>
#include <rj_geometry/shape_set.hpp>

#include "obstacle.hpp"

namespace planning {

/// This class maintains a collection of Obstacle objects.
class ObstacleSet {
public:
    ObstacleSet() = default;

    /// Initializes the set by iterating from @first to @last, which are
    /// iterators into a collection of std::shared_ptr<Obstacle>.
    template <class InputIt>
    ObstacleSet(InputIt first, InputIt last) {
        while (first != last) {
            add(*first++);
        }
    }

    std::vector<std::shared_ptr<Obstacle>>& obstacles() { return obstacles_; }
    [[nodiscard]] const std::vector<std::shared_ptr<Obstacle>>& obstacles() const {
        return obstacles_;
    }

    void add(std::shared_ptr<Obstacle> obstacle) {
        assert(obstacle != nullptr);
        obstacles_.push_back(obstacle);
    }

    void add(const ObstacleSet& other) {
        for (const auto& obstacle : other.obstacles()) {
            add(obstacle);
        }
    }

    /// Remove all obstacles
    void clear() { obstacles_.clear(); }

    /// Check if the set is empty
    [[nodiscard]] bool empty() const { return obstacles_.empty(); }

    /// Get the number of obstacles
    [[nodiscard]] size_t size() const { return obstacles_.size(); }

    /**
     * Get a set of which obstacles "hit" the given object (checks padding).
     *
     * @param obj The object to collision test
     * @return A set of all obstacles whose padding collides with the given object
     */
    template <typename T>
    std::set<std::shared_ptr<Obstacle>> hit_set(const T& obj) const {
        std::set<std::shared_ptr<Obstacle>> hits;
        for (const auto& obstacle : obstacles_) {
            if (obstacle->padding_hit(obj)) {
                hits.insert(obstacle);
            }
        }
        return hits;
    }

    /**
     * Check if any of the obstacle paddings in this set "hit" the given object.
     *
     * @param obj The object to collision test
     * @return True if one of the contained obstacle paddings hits the object.
     */
    template <typename T>
    bool hit(const T& obj) const {
        for (const auto& obstacle : obstacles_) {
            if (obstacle->padding_hit(obj)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Alias for hit() - checks padding collision.
     */
    template <typename T>
    bool padding_hit(const T& obj) const {
        return hit(obj);
    }

    /**
     * Check if any of the obstacle cores (not padding) hit the given object.
     *
     * @param obj The object to collision test
     * @return True if one of the contained obstacle cores hits the object.
     */
    template <typename T>
    bool obstacle_hit(const T& obj) const {
        for (const auto& obstacle : obstacles_) {
            if (obstacle->obstacle_hit(obj)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Convert to a ShapeSet containing all padding shapes.
     * Useful for interfacing with code that expects ShapeSet.
     *
     * @return A ShapeSet containing the padding shape from each obstacle
     */
    rj_geometry::ShapeSet to_padding_shape_set() const {
        rj_geometry::ShapeSet shape_set;
        for (const auto& obstacle : obstacles_) {
            shape_set.add(obstacle->get_padding());
        }
        return shape_set;
    }

    /**
     * Convert to a ShapeSet containing all obstacle cores.
     * Useful for debugging or visualization.
     *
     * @return A ShapeSet containing the core obstacle shape from each obstacle
     */
    rj_geometry::ShapeSet to_obstacle_shape_set() const {
        rj_geometry::ShapeSet shape_set;
        for (const auto& obstacle : obstacles_) {
            shape_set.add(obstacle->get_obstacle());
        }
        return shape_set;
    }

    /**
     * Draw all obstacle padding shapes for visualization.
     * This shows the "avoid zones" that robots will try to stay out of.
     *
     * @param debug_drawer The debug drawer to render with
     * @param color Color to use for drawing (default: semi-transparent red)
     */
    void draw_padding(rj_drawing::RosDebugDrawer* debug_drawer,
                      const QColor& color = QColor(255, 0, 0, 50)) const {
        if (debug_drawer == nullptr) {
            return;
        }
        debug_drawer->draw_shapes(to_padding_shape_set(), color);
    }

    /**
     * Draw all obstacle core shapes for visualization.
     * This shows the actual obstacle shapes (without padding).
     *
     * @param debug_drawer The debug drawer to render with
     * @param color Color to use for drawing (default: semi-transparent dark red)
     */
    void draw_cores(rj_drawing::RosDebugDrawer* debug_drawer,
                    const QColor& color = QColor(180, 0, 0, 80)) const {
        if (debug_drawer == nullptr) {
            return;
        }
        debug_drawer->draw_shapes(to_obstacle_shape_set(), color);
    }

    /**
     * Draw both obstacle cores and padding for complete visualization.
     * Cores are drawn in darker color, padding in lighter color.
     *
     * @param debug_drawer The debug drawer to render with
     * @param show_cores Whether to draw the obstacle cores (default: true)
     * @param show_padding Whether to draw the padding zones (default: true)
     */
    void draw(rj_drawing::RosDebugDrawer* debug_drawer, bool show_cores = true,
              bool show_padding = true) const {
        if (show_cores) {
            draw_cores(debug_drawer, QColor(180, 0, 0, 80));
        }
        if (show_padding) {
            draw_padding(debug_drawer, QColor(255, 0, 0, 30));
        }
    }

    friend std::ostream& operator<<(std::ostream& out, const ObstacleSet& obstacle_set) {
        out << "ObstacleSet: {";
        for (const auto& obstacle : obstacle_set.obstacles()) {
            out << obstacle->get_padding()->to_string() << ", ";
        }
        out << "}";
        return out;
    }

private:
    std::vector<std::shared_ptr<Obstacle>> obstacles_;
};

}  // namespace planning
