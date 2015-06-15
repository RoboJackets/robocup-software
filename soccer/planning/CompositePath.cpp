#include "CompositePath.hpp"

using namespace std;
namespace Planning
{
	CompositePath::CompositePath(unique_ptr<Path> path) {
		append(std::move(path));
	}

	CompositePath::CompositePath(Path *path) {
		append(path);
	}

	void CompositePath::append(Path *path) {
		append(std::unique_ptr<Path>(path));
	}

	void CompositePath::append(unique_ptr<Path> path) {
		if (duration<std::numeric_limits<float>::infinity()) {
			float pathDuration = path->getDuration();
			if (pathDuration > 0) {
				duration += pathDuration;
				paths.push_back(std::move(path));
			} else {
				debugThrow(invalid_argument("The path passed is invalid"));
			}
		} else {
			debugThrow(runtime_error("You can't append to this path. It is already infinitely long."));
		}
	}

	bool CompositePath::evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const 
	{
		if (paths.empty()) {
			return false;
		}
		if (t<0) {
			return false;
		}
		for (const std::unique_ptr<Path> &path: paths)
		{
			float timeLength = path->getDuration();
			t -= timeLength;
			if (t<=0 || timeLength == -1) {
				t += timeLength;
				path->evaluate(t, targetPosOut, targetVelOut);
				return true;
			}
		}
		return false;
	}

	bool CompositePath::hit(const Geometry2d::CompositeShape &shape, float startTime) const
	{
		if (paths.empty()) {
			return false;
		}
		int start = 0;
		for (const std::unique_ptr<Path> &path: paths)
		{
			start++;
			float timeLength = path->getDuration();
			if (timeLength == -1) {
				return path->hit(shape, startTime);
			}
			startTime -= timeLength;
			if (startTime<=0) {
				startTime += timeLength;
				if (path->hit(shape, startTime)) {
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

	void CompositePath::draw(SystemState * const state, const QColor &color, const QString &layer) const
	{
		for (const std::unique_ptr<Path> &path: paths)
		{
			path->draw(state, color, layer);
		}
	}

	float CompositePath::getDuration() const
	{
		return duration;
	}
	
	boost::optional<Geometry2d::Point> CompositePath::destination() const
	{
		if (paths.empty()) {
			return boost::none;
		}
		return paths.back()->destination();
	}

	unique_ptr<Path> CompositePath::subPath(float startTime, float endTime) const
	{
		throw runtime_error("Unimplemented Method CompositePath::subPath");
	}

	
}