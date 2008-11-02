#include "Vector_ID.h"

#include <QDomElement>
#include <QDebug>
#include <boost/foreach.hpp>

using namespace boost;
using namespace Vision;

Vector_ID::Vector_ID(Process *process, Color center_color, const Vector_Pattern pattern):
    Dot_ID(process, center_color), _pattern(pattern)
{
}

Vector_ID* Vector_ID::load(QDomElement element, Process* process, Color center)
{
	QDomElement child = element.firstChildElement();
	
	std::vector<Vector_Pattern::Pair> pairs;
	std::vector<Color_Sequence> sequences;
	
	while (!child.isNull())
	{
		QString tname = child.tagName();
		if (tname == "vectors")
		{
			QDomElement vec = child.firstChildElement();
			//make pairs
			while (!vec.isNull())
			{
				bool ok = true;
				int first = vec.attribute("start").toInt(&ok);
				int end = vec.attribute("end").toInt(&ok);
				float angle = vec.attribute("angle").toFloat(&ok);
				
				pairs.push_back(Vector_Pattern::Pair(first, end, angle));
				
				vec = vec.nextSiblingElement();
			}
		}
		else if (tname == "shells")
		{
			QDomElement shell = child.firstChildElement();
			
			while (!shell.isNull())
			{
				Color_Sequence colors;
				QDomElement color = shell.firstChildElement();
				
				while (!color.isNull())
				{
					QString col = color.attribute("value");
					Color c = Green;
					
					if (col == "pink")
					{
						c = Pink;
					}
					else if (col == "white")
					{
						c = White;
					}
						
					colors.push_back(c);
					color = color.nextSiblingElement();
				}
				
				sequences.push_back(colors);
				
				shell = shell.nextSiblingElement();
			}
		}
		
		child = child.nextSiblingElement();
	}
	
	Vector_Pattern pat(0, Geometry::Point2d(0,0), sequences, pairs);
	Vector_ID* vecid = new Vector_ID(process, center, pat);
	
	return vecid;
}

void Vector_ID::run()
{
	_robots.clear();
	
    BOOST_FOREACH(Group *center_group, spanner[_center_color]->groups())
    {
        identify(center_group);
    }
}

void Vector_ID::identify(Group *center_group)
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
