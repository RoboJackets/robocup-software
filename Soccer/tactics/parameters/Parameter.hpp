#ifndef _TACTICS__PARAMETER_H_
#define _TACTICS__PARAMETER_H_

#include <string>
#include <Geometry/Point2d.hpp>
#include <Geometry/TransformMatrix.hpp>

class Robot;

namespace Tactics
{
    class Base;

    class Parameter
    {
    public:
        Parameter(Base *tactic, const char *name);
        virtual ~Parameter();
        
        const std::string &name() const { return _name; }
        bool valid() const { return _valid; }
        
        virtual void clear();
        
        // These functions set the value of the parameter given various input types.
        // They will throw an exception if the input data cannot be assigned to this parameter.
        //
        // The default implementations all throw std::invalid_argument.
        virtual void set(float value);
        virtual void set(const Geometry::Point2d &point);
        virtual void set(const Geometry::TransformMatrix *matrix, const Geometry::Point2d &point);
        virtual void set(Robot *robot);
        virtual void set(const std::string &value);
        
    protected:
        Base *_tactic;
        std::string _name;
        bool _valid;
    };
}

#endif // _TACTICS__PARAMETER_H_
