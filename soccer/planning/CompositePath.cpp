#include "CompositePath.hpp"

using namespace std;
using namespace Geometry2d;
namespace Planning
{
	CompositePath::CompositePath(unique_ptr<Path> path) {
		append(std::move(path));
	}

	void CompositePath::append(unique_ptr<Path> path) {
		if (duration < numeric_limits<float>::infinity()) {
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

	bool CompositePath::evaluate(float t, MotionInstant &targetMotionInstant) const
	{
		if (t<0) {
			debugThrow(invalid_argument("A time less than 0 was entered for time t."));
		}

		if (paths.empty()) {
			return false;
		}
		for (const std::unique_ptr<Path> &path: paths)
		{
			float timeLength = path->getDuration();
			t -= timeLength;
			if (t<=0 || timeLength == -1) {
				t += timeLength;
				path->evaluate(t, targetMotionInstant);
				return true;
			}
		}
		targetMotionInstant = destination().get();
		return false;
	}

	bool CompositePath::hit(const CompositeShape &shape, float startTime) const
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
	
	boost::optional<MotionInstant> CompositePath::destination() const
	{
		if (paths.empty()) {
			return boost::none;
		}
		return paths.back()->destination();
	}

	unique_ptr<Path> CompositePath::subPath(float startTime, float endTime) const
	{
		//Check for valid arguments
		if (startTime<0) {
			throw invalid_argument("CompositePath::subPath(): startTime(" + to_string(startTime) + ") can't be less than zero");
		}

		if (endTime<0) {
			throw invalid_argument("CompositePath::subPath(): endTime(" + to_string(endTime) + ") can't be less than zero");
		}

		if (startTime > endTime) {
			throw invalid_argument("CompositePath::subPath(): startTime(" + to_string(startTime) + ") can't be after endTime(" + to_string(endTime) + ")");
		}

		if (startTime >= duration) {
			debugThrow(invalid_argument("CompositePath::subPath(): startTime(" + to_string(startTime) + ") can't be greater than the duration(" + to_string(duration) + ") of the path"));
			return unique_ptr<Path>(new CompositePath());
		}

		if (startTime == 0 && endTime>=duration) {
			return this->clone();
		}

		//Find the first Path in the vector of paths which will be included in the subPath
		size_t start = 0;
		float time = 0;
		float lastTime = 0;
		while (time <= startTime) {
			lastTime = paths[start]->getDuration();
			time += lastTime;
			start++;
		}

		//Get the time into the Path in the vector of paths which the subPath will start
		float firstStartTime = (time - lastTime);

		//If the path will only contain that one Path just return a subPath of that Path
		if (time >= endTime) {
			return paths[start-1]->subPath(startTime - firstStartTime, endTime - firstStartTime );
		} else {
			//Create a CompositePath initialized with only that first path.
			CompositePath *path = new CompositePath(paths[start-1]->subPath(startTime - firstStartTime));
			unique_ptr<Path> lastPath;
			size_t end;

			//Find the last Path in the vector of paths which will be included in the subPath and store it in lastPath
			if (endTime>= duration) {
				lastPath = paths.back()->clone();
				end = paths.size()-1;
			} else {
				end = start;
				while (time<endTime) {
					lastTime = paths[start]->getDuration();
					time += lastTime;
					end++;
				}
				end--;
				lastPath = paths[end]->subPath(0, endTime - (time - lastTime));
			}

			//Add the ones in the middle
			while (start<end) {
				path->append(paths[start]->clone());
				start++;
			}

			//Add the last one
			path->append(std::move(lastPath));
			
			return unique_ptr<Path>(path);
		}
	}

	unique_ptr<Path> CompositePath::clone() const{
		CompositePath *newPath = new CompositePath();
		for (const unique_ptr<Path> &path: paths) {
			newPath->append(path->clone());
		}
		return unique_ptr<Path>(newPath);
	}
}