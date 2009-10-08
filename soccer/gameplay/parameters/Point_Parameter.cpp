#include "Point_Parameter.hpp"
#include "../Role.hpp"

Gameplay::Point_Parameter::Point_Parameter(Behavior *behavior, const char *name): Parameter(behavior, name)
{
    _matrix = 0;
    _robot = 0;
}

void Gameplay::Point_Parameter::clear()
{
    _valid = false;
    _matrix = 0;
    _robot = 0;
}

void Gameplay::Point_Parameter::set(Robot *robot)
{
    _valid = true;
    _matrix = 0;
    _robot = robot;
}

void Gameplay::Point_Parameter::set(const Geometry2d::Point &point)
{
    _valid = true;
    _matrix = 0;
    _point = point;
    _robot = 0;
}

void Gameplay::Point_Parameter::set(const Geometry2d::TransformMatrix *matrix, const Geometry2d::Point &point)
{
    _valid = true;
    _matrix = matrix;
    _point = point;
    _robot = 0;
}

Geometry2d::Point Gameplay::Point_Parameter::point() const
{
    if (_matrix)
    {
        return (*_matrix) * _point;
    } else if (_robot)
    {
        if (_robot->visible())
        {
            return _robot->state()->pos;
        } else {
            return Geometry2d::Point();
        }
    } else {
        return _point;
    }
}
