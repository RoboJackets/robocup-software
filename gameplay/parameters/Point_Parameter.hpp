#pragma once

#include "Parameter.hpp"

namespace Gameplay
{    
    class Point_Parameter: public Parameter
    {
    public:
        Point_Parameter(Behavior *tactic, const char *name);
        
        virtual void clear();
        virtual void set(Robot *robot);
        virtual void set(const Geometry2d::Point &point);
        virtual void set(const Geometry2d::TransformMatrix *matrix, const Geometry2d::Point &point);
        
        Geometry2d::Point point() const;
        
    protected:
        // If non-null, the point is transformed by this matrix every time
        // point() is called.
        const Geometry2d::TransformMatrix *_matrix;
        Geometry2d::Point _point;
        Robot *_robot;
    };
}
