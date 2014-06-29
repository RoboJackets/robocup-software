#pragma once

#include "Point.hpp"
#include "Shape.hpp"
#include "Segment.hpp"
#include <vector>
#include <memory>
#include <set>

class Obstacle;

namespace Geometry2d {

    /**
     * A Geometry2d::CompositeShape is a Shape that is made up of other shapes.
     */
    class CompositeShape : public Shape {
    public:
        CompositeShape(const std::shared_ptr<Shape> shape) {
            _subshapes.push_back(shape);
        }

        CompositeShape() {}

        virtual ~CompositeShape() {
            clear();
        }

        CompositeShape(const CompositeShape &other) {
            for (auto itr : other) {
                _subshapes.push_back(std::shared_ptr<Shape>((*itr).clone()));
            }
        }

        Shape *clone() const;

        virtual bool containsPoint(const Point &pt) const;

        void add(const std::shared_ptr<Shape> shape);

        /// adds @compShape's subshapes to the receiver
        void add(const CompositeShape &compShape);

        const std::vector<std::shared_ptr<Shape> > &subshapes() const {
            return _subshapes;
        }

        /// removes all subshapes
        void clear();

        bool empty() {
            return _subshapes.empty();
        }

        unsigned int size() const {
            return _subshapes.size();
        }


        /**
         * Checks if a given object hits obstacles in the group
         *
         * @param obj The object to collision test
         * @param hitSet A set to add the colliding obstacles to
         * @return A bool telling whether or not there were any collisions
         */
        template<typename T>
        bool hit(const T &obj, std::set<std::shared_ptr<Shape> > &hitSet) const
        {
            for (const_iterator it = begin(); it != end(); ++it)
            {
                if ((*it)->hit(obj))
                {
                    hitSet.insert(*it);
                }
            }

            return !hitSet.empty();
        }

        bool hit(const Point &pt, std::set<std::shared_ptr<Shape> > &hitSet) const {
            return hit<Point>(pt, hitSet);
        }

        bool hit(const Segment &seg, std::set<std::shared_ptr<Shape> > &hitSet) const {
            return hit<Segment>(seg, hitSet);
        }

        /**
         * Checks if a given shape is in it
         *
         * @param obj The object to collision test
         * @return A bool telling whether or not there were any collisions
         */
        template<typename T>
        bool hit(const T &obj) const
        {
            for (const_iterator it = begin(); it!=end(); ++it)
            {
                if ((*it)->hit(obj))
                {
                    return true;
                }
            }

            return false;
        }

        bool hit(const Point &pt) const {
            return hit<Point>(pt);
        }

        bool hit(const Segment &seg) const {
            return hit<Segment>(seg);
        }


        // STL typedefs
        typedef std::vector<std::shared_ptr<Shape> >::const_iterator const_iterator;
        typedef std::vector<std::shared_ptr<Shape> >::iterator iterator;
        typedef std::shared_ptr<Shape> value_type;
        
        // STL Interface
        const_iterator begin() const { return _subshapes.begin(); }
        const_iterator end() const { return _subshapes.end(); }

        iterator begin() { return _subshapes.begin(); }
        iterator end() { return _subshapes.end(); }


    private:
        std::vector<std::shared_ptr<Shape> > _subshapes;
    };
}
