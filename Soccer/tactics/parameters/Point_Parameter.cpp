#include "Point_Parameter.hpp"
#include "../../Role.hpp"

Tactics::Point_Parameter::Point_Parameter(Tactics::Base *tactic, const char *name): Parameter(tactic, name)
{
    _matrix = 0;
    _robot = 0;
}

void Tactics::Point_Parameter::clear()
{
    _valid = false;
    _matrix = 0;
    _robot = 0;
}

void Tactics::Point_Parameter::set(Robot *robot)
{
    _valid = true;
    _matrix = 0;
    _robot = robot;
}

void Tactics::Point_Parameter::set(const Geometry::Point2d &point)
{
    _valid = true;
    _matrix = 0;
    _point = point;
    _robot = 0;
}

void Tactics::Point_Parameter::set(const Geometry::TransformMatrix *matrix, const Geometry::Point2d &point)
{
    _valid = true;
    _matrix = matrix;
    _point = point;
    _robot = 0;
}

Geometry::Point2d Tactics::Point_Parameter::point() const
{
    if (_matrix)
    {
        return (*_matrix) * _point;
    } else if (_robot)
    {
        if (_robot->visible())
        {
            return _robot->vision()->pos;
        } else {
            return Geometry::Point2d();
        }
    } else {
        return _point;
    }
}
