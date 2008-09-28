#include "Dot_ID.h"
#include "../Spanner.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace boost;
using namespace boost::assign;

Vision::Dot_ID::Dot_ID(Process *process, Color center_color)
{
    _process = process;
    if (process)
    {
        spanner = process->spanner;
    } else {
        spanner = 0;
    }
    
    _center_color = center_color;
    _dot_colors += center_color, White, Green, Pink;
}

void Vision::Dot_ID::collect(const Group *center_group)
{
    if (!spanner)
    {
        throw logic_error("Dot_ID tried to run with no spanners");
    }
    
    _dots.clear();
    _dots.reserve(6);
    
    _center = Geometry::Point2d();
    
    int n = 0;
    
    float rsq;
    if (_process)
    {
        rsq = _process->robot_radius();
    } else {
        rsq = 1000;
    }
    rsq *= rsq;
    
    BOOST_FOREACH(Color color, _dot_colors)
    {
        BOOST_FOREACH(Group *group, spanner[color]->groups())
        {
            float dx = group->center.x - center_group->center.x;
            float dy = group->center.y - center_group->center.y;
            float dist_sq = dx * dx + dy * dy;
            if (dist_sq <= rsq)
            {
                _dots.push_back(Dot(group));
                
                _center += group->center;
                ++n;
            }
        }
    }
    
    // Get average center
    _center /= n;
    
    // Calculate angle from center to each dot
    BOOST_FOREACH(Dot &dot, _dots)
    {
        dot.angle = (dot.group->center - _center).angle();
    }
}

void Vision::Dot_ID::remove_center()
{
    for (vector<Dot>::iterator i = _dots.begin(); i != _dots.end();)
    {
        if (i->group->color == _center_color)
        {
            _dots.erase(i);
        } else {
            ++i;
        }
    }
}

void Vision::Dot_ID::align_largest_gap()
{
    // Sort by angles
    sort(_dots.begin(), _dots.end(), Dot::compare_angles);
    
    // Find the largest gap
    float best_gap = 0;
    int best = 0;
    unsigned int i = _dots.size() - 1;
    for (unsigned int j = 0; j < _dots.size(); ++j)
    {
        float delta = _dots[j].angle - _dots[i].angle;
        if (delta < -M_PI)
        {
            delta += 2 * M_PI;
        } else if (delta > M_PI)
        {
            delta -= 2 * M_PI;
        }
        
        if (delta > best_gap)
        {
            best_gap = delta;
            best = i;
        }
        
        i = j;
    }
    
    rotate(_dots.begin(), _dots.begin() + best, _dots.end());
}

void Vision::Dot_ID::assign_dot_ids()
{
    for (unsigned int i = 0; i < _dots.size(); ++i)
    {
        _dots[i].group->id = i;
    }
}

int Vision::Dot_ID::find_id(const Pattern &pattern) const
{
    for (unsigned int i = 0; i < pattern.sequences.size(); ++i)
    {
        if (match_color_sequence(pattern.sequences[i]))
        {
            return i;
        }
    }
    
    return -1;
}

bool Vision::Dot_ID::match_color_sequence(const Color_Sequence &seq) const
{
    if (_dots.size() != seq.size())
    {
        // Wrong number of dots
        return false;
    }
    
    for (unsigned int i = 0; i < _dots.size(); ++i)
    {
        if (seq[i] != _dots[i].group->color)
        {
            return false;
        }
    }
    
    return true;
}

void Vision::Dot_ID::set_report(Group *center_group, const Pattern &pattern, const Geometry::Point2d &center, const Geometry::Point2d &facing)
{
    center_group->id = find_id(pattern);
}

void Vision::Dot_ID::set_report(Group *center_group, const Pattern &pattern, int id, const Geometry::Point2d &center, const Geometry::Point2d &facing)
{
    center_group->id = id;
    //center_group->direction = fix_angle(facing.angle() * 180.0 / M_PI + pattern.angle_offset);
}
