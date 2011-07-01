#pragma once

#include <Geometry2d/Point.hpp>
#include <boost/shared_ptr.hpp>

class BallFilter;
class SystemState;

class BallObservation
{
public:
	BallObservation(Geometry2d::Point pos = Geometry2d::Point(), uint64_t time = 0):
		pos(pos), time(time)
	{
	}

	Geometry2d::Point pos;
	uint64_t time;
};

class BallTracker
{
public:
	BallTracker();
	
	void run(const std::vector<BallObservation> &obs, SystemState *state);

private:
	// Time of the last observation given to the filter
	uint64_t _lastTrackTime;
	
	// Possible balls that were not used on the last frame
	struct PossibleTrack
	{
		PossibleTrack(): current(false), numFrames(0) {}
		PossibleTrack(const BallObservation &obs): current(true), numFrames(1), obs(obs) {}
		
		// True if this track was seen in the most recent set of vision observations
		bool current;
		
		// Number of frames over which we have been following this track
		int numFrames;
		
		BallObservation obs;
	};
	std::vector<PossibleTrack> _possibleTracks;
	
	boost::shared_ptr<BallFilter> _ballFilter;
};