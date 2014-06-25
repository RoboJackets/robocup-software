#pragma once

#include "Point.hpp"
#include <vector>
#include <memory>


namespace Geometry2d {

    class CompositeShape {
    public:
        CompositeShape(std::shared_ptr<Shape> shape) {
            _subshapes.push_back(shape);
        }

        CompositeShape() {}

        virtual ~CompositeShape() {
            clear();
        }

        virtual bool containsPoint(const Point &pt) const;

        void addSubshape(std::shared_ptr<Shape> shape);

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
        bool hit(const T &obj, std::set<std::shared_ptr<Obstacle> > &hitSet) const
        {
            for (const_iterator it = begin(); it != end(); ++it)
            {
                if ((*it)->hit(obj))
                {
                    hitSet.add(*it);
                }
            }

            return !hitSet.empty();
        }


        // STL typedefs
        typedef std::set<std::shared_ptr<Obstacle> >::const_iterator const_iterator;
        typedef std::set<std::shared_ptr<Obstacle> >::iterator iterator;
        typedef std::shared_ptr<Obstacle> value_type;
        
        // STL Interface
        const_iterator begin() const { return _obstacles.begin(); }
        const_iterator end() const { return _obstacles.end(); }

        iterator begin() { return _obstacles.begin(); }
        iterator end() { return _obstacles.end(); }


    private:
        std::vector<shared_ptr<Shape> > _subshapes;
    }
}
