#pragma once

#include <string>
#include <vector>

#include <rj_geometry/point.hpp>

struct DebugPath {
    int layer() const { return layer_; }
    uint32_t color() const { return color_; }
    int points_size() const { return static_cast<int>(points_.size()); }
    const rj_geometry::Point& points(int i) const { return points_.at(i); }

    int layer_ = -1;
    uint32_t color_ = 0;
    std::vector<rj_geometry::Point> points_;
};

struct DebugRobotPath {
    struct DebugRobotPathPoint {
        const rj_geometry::Point& pos() const { return pos_; }
        const rj_geometry::Point& vel() const { return vel_; }
        rj_geometry::Point& mutable_pos() { return pos_; }
        rj_geometry::Point& mutable_vel() { return vel_; }

        rj_geometry::Point pos_;
        rj_geometry::Point vel_;
    };

    int layer() const { return layer_; }
    void set_layer(int layer) { layer_ = layer; }
    int points_size() const { return static_cast<int>(points_.size()); }
    const DebugRobotPathPoint& points(int i) const { return points_.at(i); }
    DebugRobotPathPoint& add_points() { return points_.emplace_back(); }

    int layer_ = -1;
    std::vector<DebugRobotPathPoint> points_;
};

struct DebugCircle {
    int layer() const { return layer_; }
    uint32_t color() const { return color_; }
    const rj_geometry::Point& center() const { return center_; }
    float radius() const { return radius_; }

    int layer_ = -1;
    uint32_t color_ = 0;
    rj_geometry::Point center_;
    float radius_ = 0;
};

struct DebugArc {
    int layer() const { return layer_; }
    uint32_t color() const { return color_; }
    const rj_geometry::Point& center() const { return center_; }
    float radius() const { return radius_; }
    float start() const { return start_; }
    float end() const { return end_; }

    int layer_ = -1;
    uint32_t color_ = 0;
    rj_geometry::Point center_;
    float radius_ = 0;
    float start_ = 0;
    float end_ = 0;
};

struct DebugText {
    int layer() const { return layer_; }
    uint32_t color() const { return color_; }
    const rj_geometry::Point& pos() const { return pos_; }
    const std::string& text() const { return text_; }
    bool center() const { return center_; }

    int layer_ = -1;
    uint32_t color_ = 0;
    rj_geometry::Point pos_;
    std::string text_;
    bool center_ = true;
};

struct DebugDrawFrame {
    std::vector<DebugPath> paths;
    std::vector<DebugPath> polygons;
    std::vector<DebugCircle> circles;
    std::vector<DebugArc> arcs;
    std::vector<DebugText> texts;
    std::vector<DebugRobotPath> robot_paths;
    std::vector<std::string> debug_layers;

    void clear() {
        paths.clear();
        polygons.clear();
        circles.clear();
        arcs.clear();
        texts.clear();
        robot_paths.clear();
        debug_layers.clear();
    }
};
