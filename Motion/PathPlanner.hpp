#ifndef PATHPLANNER_HPP_
#define PATHPLANNER_HPP_

#include <VisionData>

#include <Geometry/Point2d.hpp>
#include <Geometry/Circle2d.hpp>

#include <vector>

class PathPlanner
{
	public:
		/** Output of a planned path */
		typedef struct
		{
			/** distance along path */
			float distance;
			
			/** direction */
			Geometry::Point2d direction;
		} PPOut;
		
		PathPlanner(Packet::VisionData& vision) : _vision(vision) {}
		virtual ~PathPlanner() {}
		
		virtual PathPlanner::PPOut plan(const unsigned int rid, Geometry::Point2d dest) = 0;
		
		void addNoZone(const Geometry::Circle2d circle) { _noZones.push_back(circle); }
		void clearNoZones() { _noZones.clear(); }
		
		//static void addGlobalNoZone(const Geometry::Circle2d circle) { _globalNoZones.push_back(circle); }
		//static void clearGlobalNoZones() { _globalNoZones.clear(); }
		
		//static void setAvoidOp(bool avoid) { _avoidOpp = avoid; }
		void setAvoidOp(bool avoid) { _avoidOpp = avoid; }
		
	protected:
		Packet::VisionData& _vision;
		
		/** local nozones for a single instance */
		std::vector<Geometry::Circle2d> _noZones;
		
		/** if true, robots must not go into the opponent side of the field */
		bool _avoidOpp;
		
	private:
};

#endif /*PATHPLANNER_HPP_*/
