#ifndef SIMPLEPLANNER_HPP_
#define SIMPLEPLANNER_HPP_

#include <vector>

#include <Geometry/Line2d.hpp>
#include <Geometry/Segment.hpp>

#include "PathPlanner.hpp"

class SimplePlanner : public PathPlanner
{
	private:
		/** single path*/
		typedef struct
		{
			Geometry::Segment seg1, seg2;
						
			float dist() const
			{
				return seg1.length() + seg2.length();
			}
		} Path;
		
	public:
		SimplePlanner(Packet::VisionData& vision);
		
		/** plans a path from source robot to destination
		 *  Vision should not be changed while this is happening */
		virtual PathPlanner::PPOut plan(const unsigned int rid, Geometry::Point2d dest);
	
	private:
		static void genLines(const Geometry::Point2d point, 
				const Geometry::Circle2d zone, std::vector<Geometry::Line2d>& vect);
	
	/// members ///
	private:
		/** potential paths to take */
		std::vector<Path> _paths;
		
		std::vector<Geometry::Line2d> _orig;
		std::vector<Geometry::Line2d> _dest;
};

#endif /*SIMPLEPLANNER_HPP_*/
