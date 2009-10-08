#pragma once

#include <string>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>

namespace Gameplay
{
    class Robot;
    class Behavior;
    
    class Parameter
    {
    public:
        Parameter(Behavior *behavior, const char *name);
        virtual ~Parameter();
        
        const std::string &name() const { return _name; }
        bool valid() const { return _valid; }
        
        virtual void clear();
        
        // These functions set the value of the parameter given various input types.
        // They will throw an exception if the input data cannot be assigned to this parameter.
        //
        // The default implementations all throw std::invalid_argument.
        virtual void set(float value);
        virtual void set(const Geometry2d::Point &point);
        virtual void set(const Geometry2d::TransformMatrix *matrix, const Geometry2d::Point &point);
        virtual void set(Robot *robot);
        virtual void set(const std::string &value);
        
    protected:
        Behavior *_behavior;
        std::string _name;
        bool _valid;
    };
}
