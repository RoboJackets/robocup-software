#pragma once

#include <string>
#include <vector>

#include <rj_geometry/point.hpp>

struct DebugPath {
    int layer = -1;
    uint32_t color = 0;
    std::vector<rj_geometry::Point> points;
};

struct DebugRobotPath {
    struct DebugRobotPathPoint {
        rj_geometry::Point pos;
        rj_geometry::Point vel;
    };

    int layer = -1;
    std::vector<DebugRobotPathPoint> points;
};

struct DebugCircle {
    int layer = -1;
    uint32_t color = 0;
    rj_geometry::Point center;
    float radius = 0;
};

struct DebugArc {
    int layer = -1;
    uint32_t color = 0;
    rj_geometry::Point center;
    float radius = 0;
    float start = 0;
    float end = 0;
};

struct DebugText {
    int layer = -1;
    uint32_t color = 0;
    rj_geometry::Point pos;
    std::string text;
    bool center = true;
};

class DebugDrawFrame {
public:
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
