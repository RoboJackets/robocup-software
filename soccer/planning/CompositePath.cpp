#include "CompositePath.hpp"


namespace Planning
{
	CompositePath::CompositePath(Path *path) {
		append(path);
	}

	void CompositePath::append(Path *path) {
		paths.push_back(std::unique_ptr<Path>(path));
	}

	/**
	 * A path describes the position and velocity a robot should be at for a
	 * particular time interval.  This method evalates the path at a given time and
	 * returns the target position and velocity of the robot.
	 *
	 * @param[in] 	t Time (in seconds) since the robot started the path
	 * @param[out] 	targetPosOut The position the robot would ideally be at at the given time
	 * @param[out] 	targetVelOut The target velocity of the robot at the given time
	 * @return 		true if the path is valid at time @t, false if you've gone past the end
	 */
	bool CompositePath::evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) {
		if (t<0) {
			return false;
		}
		for (std::unique_ptr<Path> &path: paths)
		{
			float timeLength = path->getTime();
			t -= timeLength;
			if (t<=0 || timeLength == -1) {
				t += timeLength;
				path->evaluate(t, targetPosOut, targetVelOut);
				return true;
			}
		}
		return false;
	}

	/**
	 * Returns true if the path never touches an obstacle or additionally, when exitObstacles is true, if the path
	 * starts out in an obstacle but leaves and never re-enters any obstacle.
	 *
	 * @param[in]	shape The obstacles on the field
	 * @param[in] 	start The point on the path to start checking from
	 * @return 		true if the path is valid, false if it hits an obstacle
	 */
	bool CompositePath::hit(const Geometry2d::CompositeShape &shape, float startTime) {
		int start = 0;
		for (std::unique_ptr<Path> &path: paths)
		{
			start++;
			float timeLength = path->getTime();
			if (timeLength == -1) {
				return false;
			}
			startTime -= timeLength;
			if (startTime<=0) {
				startTime += timeLength;
				if(paths[start]->hit(shape, startTime)) {
					return true;
				}
				break;
			}
		}

		for (;start<paths.size(); start++) {
			if (paths[start]->hit(shape)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Draws the path
	 *
	 * @param[in]	state The SystemState to draw the path on
	 * @param[in] 	color The color the path should be drawn
	 * @param[in] 	layer The layer to draw the path on
	 */
	void CompositePath::draw(SystemState * const state, const QColor &color, const QString &layer) {
		for (std::unique_ptr<Path> &path: paths)
		{
			path->draw(state, color, layer);
		}
	}
	
	/** 
	 * Estimates how long it would take for the robot to traverse the entire path
	 *
	 * @return 	The time from start to path completion or -1 if there is no destination
	 */
	float CompositePath::getTime() {
		float time = 0;
		for (std::unique_ptr<Path> &path: paths)
		{
			float timeLength = path->getTime();
			if (timeLength == -1) {
				return -1;
			}
			time += path->getTime();
		}
		return time;
	}

	/**
	 * Returns the destination point of the path if it has one
	 */
	boost::optional<Geometry2d::Point> CompositePath::destination() {
		if (paths.empty()) {
			return boost::none;
		}
		return paths.back()->destination();
	}


	
}