#include "Vector_ID.h"

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

using namespace boost;
using namespace boost::assign;

namespace Vision
{
    Vector_Pattern Vector_ID::Nav = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Pink, Pink))
        (Colors(Pink, Green, Pink))
        (Colors(Pink, Pink, Green))
        (Colors(Pink, Green, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        );

    Vector_Pattern Vector_ID::ZJUNlict = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Green, Green))
//        (Colors(Green, Pink, Pink))
        (Colors(Pink, Pink, Green))
        (Colors(Green, Pink, Green))
        (Colors(Green, Green, Pink)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        );

    Vector_Pattern Vector_ID::Botnia = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Pink, Green))
        (Colors(Green, Green, Pink))
        (Colors(Green, Pink, Green))
        (Colors(Pink, Green, Pink))
        (Colors(Pink, Green, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        );

    Vector_Pattern Vector_ID::BSmart = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(White, Pink, Green))
        (Colors(Pink, White, Green))
        (Colors(Green, Pink, White))
        (Colors(Pink, Green, White))
        (Colors(Pink, Pink, White)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -45))
        );
    
    Vector_Pattern Vector_ID::Strive = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Pink, Pink))
        (Colors(Pink, Green, Pink))
        (Colors(Pink, Green, Green))
        (Colors(Green, Green, Pink))
        (Colors(Pink, Pink, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        );

    Vector_Pattern Vector_ID::CMU = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Green, Green, Green, Green))
        (Colors(Pink, Green, Green, Green))
        (Colors(Green, Pink, Green, Green))
        (Colors(Pink, Green, Pink, Green))
        (Colors(Green, Pink, Green, Pink)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );
    
    Vector_Pattern Vector_ID::Robodragons = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Green, Green, Green, Green))
        (Colors(Green, Green, Green, White))
        (Colors(Green, Green, White, Green))
        (Colors(Green, White, Green, White))
        (Colors(White, Green, White, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );
    
    Vector_Pattern Vector_ID::Khainui = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Green, Green, Green, Green))
        (Colors(Pink, Green, Green, Green))
        (Colors(Green, Pink, Green, Green))
        (Colors(Green, Green, Pink, Green))
        (Colors(Green, Green, Green, Pink)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );
    
    Vector_Pattern Vector_ID::GaTech_Old = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Pink, Pink, Pink))
        (Colors(Pink, Pink, Pink, Green))
        (Colors(Pink, Pink, Green, Green))
        (Colors(Green, Green, Green, Pink))
        (Colors(Green, Green, Green, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );

    Vector_Pattern Vector_ID::GaTech = Vector_Pattern(180, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Green, Green, Green, Green))
        (Colors(Green, Green, Pink, Green))
        (Colors(Pink, Pink, Green, Green))
        (Colors(Green, Pink, Pink, Pink))
        (Colors(Pink, Pink, Pink, Pink)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );

    Vector_Pattern Vector_ID::Unknown2 = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Green, Green, Green, Green))
        (Colors(Green, Green, Pink, Green))
        (Colors(Pink, Green, Green, Green))
        (Colors(Green, Green, Green, Pink))
        (Colors(Green, Pink, Green, Pink)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        (Vector_Pattern::Pair(3, 2, -90))
        );

    Vector_Pattern Vector_ID::PlasmaZ = Vector_Pattern(0, Geometry::Point2d(0, 0),
        // Colors
        list_of
        (Colors(Pink, Green, Green))
        (Colors(Green, Green, Pink))
        (Colors(Pink, Green, Pink))
        (Colors(Green, Green, Green))
        (Colors(Green, Pink, Green)),
        
        // Pairs
        list_of
        (Vector_Pattern::Pair(0, 1, -90))
        );
}

Vision::Vector_ID::Vector_ID(Process *process, Color center_color, const Vector_Pattern &pattern):
    Dot_ID(process, center_color), _pattern(pattern)
{
}

void Vision::Vector_ID::run()
{
    BOOST_FOREACH(Group *center_group, spanner[_center_color]->groups())
    {
        identify(center_group);
    }
}

void Vision::Vector_ID::identify(Group *center_group)
{
    collect(center_group);
    remove_center();
    align_largest_gap();
    assign_dot_ids();
    
    if (_dots.size() >= _pattern.min_dots)
    {
        Geometry::Point2d facing;
        BOOST_FOREACH(const Vector_Pattern::Pair &pair, _pattern.pairs)
        {
            Geometry::Point2d v = _dots[pair.i1].group->center - _dots[pair.i0].group->center;
            v.rotate(Geometry::Point2d(), pair.angle);
            facing += v;
        }
        
        set_report(center_group, _pattern, _center, facing);
    }
}
