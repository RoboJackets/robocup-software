#include "Tree.hpp"

#include <Utils.hpp>

#include <stdio.h>
#include <iostream>

using namespace Planning;
using namespace std;

//// Point ////
Tree::Point::Point(Geometry2d::Point p, Tree::Point* parent) : pos(p) {
    _parent = parent;
    leaf = true;

    if (_parent) {
        _parent->children.push_back(this);
        _parent->leaf = false;
    }
}

void Tree::Point::addEdges(std::list<Geometry2d::Segment>& edges) {
    for (Tree::Point* next : children) {
        edges.push_back(Geometry2d::Segment(pos, next->pos));
        next->addEdges(edges);
    }
}

//// Tree ////

Tree::Tree() {
    step = .1;
    _obstacles = nullptr;
}

Tree::~Tree() { clear(); }

void Tree::clear() {
    _obstacles = nullptr;

    // Delete all points
    for (Point* pt : points) {
        delete pt;
    }
    points.clear();
}

void Tree::init(Geometry2d::Point start,
                const Geometry2d::ShapeSet* obstacles) {
    clear();

    _obstacles = obstacles;

    Point* p = new Point(start, nullptr);
    p->hit = _obstacles->hitSet(p->pos);
    points.push_back(p);
}

void Tree::addPath(vector<Geometry2d::Point> &path, Point *dest,
                   const bool rev) const {
    list<Point*> points;

    int n = 0;
    while (dest) {
        if (rev) {
            points.push_back(dest);
        } else {
            points.push_front(dest);
        }
        dest = dest->parent();
        ++n;
    }

    path.reserve(points.size() + n);

    for (Point* pt : points) {
        // Each point in the path is given a time of zero - the actual time will
        // be calculated later by the planner
        path.push_back(pt->pos);
    }
}

Tree::Point* Tree::nearest(Geometry2d::Point pt) {
    float bestDistance = -1;
    Point* best = nullptr;

    for (Point* other : points) {
        float d = (other->pos - pt).magsq();
        if (bestDistance < 0 || d < bestDistance) {
            bestDistance = d;
            best = other;
        }
    }

    return best;
}

Tree::Point* Tree::start() const {
    if (points.empty()) {
        return nullptr;
    }

    return points.front();
}

Tree::Point* Tree::last() const {
    if (points.empty()) {
        return nullptr;
    }

    return points.back();
}

//// Fixed Step Tree ////
Tree::Point* FixedStepTree::extend(Geometry2d::Point pt, Tree::Point* base) {
    // if we don't have a base point, try to find a close point
    if (!base) {
        base = nearest(pt);
        if (!base) {
            return nullptr;
        }
    }

    Geometry2d::Point delta = pt - base->pos;
    float d = delta.mag();

    Geometry2d::Point pos;
    if (d < step) {
        pos = pt;
    } else {
        pos = base->pos + delta / d * step;
    }

    // Check for obstacles.

    // moveHit is the set of obstacles that this move touches.
    // If this move touches any obstacles that the starting point didn't already
    // touch,
    // it has entered an obstacle and will be rejected.
    std::set<shared_ptr<Geometry2d::Shape>> moveHit =
        _obstacles->hitSet(Geometry2d::Segment(pos, base->pos));
    if (!moveHit.empty()) {
        // We only care if there are any items in moveHit that are not in
        // point->hit, so
        // we don't store the result of set_difference.
        try {
            set_difference(
                moveHit.begin(), moveHit.end(), base->hit.begin(),
                base->hit.end(),
                ExceptionIterator<std::shared_ptr<Geometry2d::Shape>>());
        } catch (exception& e) {
            // We hit a new obstacle
            return nullptr;
        }
    }

    // Allow this point to be added to the tree
    Point* p = new Point(pos, base);
    p->hit = std::move(moveHit);
    points.push_back(p);

    return p;
}

bool FixedStepTree::connect(Geometry2d::Point pt) {
    // try to reach the goal pt
    const unsigned int maxAttemps = 50;

    Point* from = nullptr;

    for (unsigned int i = 0; i < maxAttemps; ++i) {
        Point* newPt = extend(pt, from);

        // died
        if (!newPt) {
            return false;
        }

        if (newPt->pos == pt) {
            return true;
        }

        from = newPt;
    }

    return false;
}
