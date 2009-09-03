#include "Offset3_ID.h"

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace boost;
using namespace boost::assign;

namespace Vision
{
}

Vision::Offset3_ID::Offset3_ID(Process *process, Color center_color, const Vector_Pattern pattern):
    Vector_ID(process, center_color, pattern)
{
}

void Vision::Offset3_ID::run()
{
    BOOST_FOREACH(Group *center_group, spanner[_center_color]->groups())
    {
        identify(center_group);
        
        if (center_group->id == 3 && _dots.size() >= 3)
        {
            // The color sequence does not uniquely identify the robot.
            // The final criterion is whether dot 2 is closer to dot 0 or 1.
            Geometry2d::Point p2 = _dots[2].group->center;
            float d0 = (_dots[0].group->center - p2).magsq();
            float d1 = (_dots[1].group->center - p2).magsq();
            if (d0 < d1)
            {
                center_group->id = 4;
            }
        }
    }
}
