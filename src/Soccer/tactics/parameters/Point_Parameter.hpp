#ifndef _TACTICS__POINT_PARAMETER_HPP_
#define _TACTICS__POINT_PARAMETER_HPP_

#include "Parameter.hpp"

namespace Tactics
{    
    class Point_Parameter: public Parameter
    {
    public:
        Point_Parameter(Base *tactic, const char *name);
        
        virtual void clear();
        virtual void set(Robot *robot);
        virtual void set(const Geometry::Point2d &point);
        virtual void set(const Geometry::TransformMatrix *matrix, const Geometry::Point2d &point);
        
        Geometry::Point2d point() const;
        
    protected:
        // If non-null, the point is transformed by this matrix every time
        // point() is called.
        const Geometry::TransformMatrix *_matrix;
        Geometry::Point2d _point;
        Robot *_robot;
    };
}

#endif // _TACTICS__POINT_PARAMETER_HPP_
