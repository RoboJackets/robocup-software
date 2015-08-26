#include "BallTracker.hpp"
#include "BallFilter.hpp"

#include <Utils.hpp>
#include <Processor.hpp>
#include <SystemState.hpp>

using namespace std;
using namespace boost;
using namespace Geometry2d;

// Balls closer together than this will be removed
static const float Distance_Limit = 0.2;

// Maximum distance between observations on consecutive frames to
// result in a possible acquisition.
static const float Acquisition_Match_Distance = 0.5;

// Age of a track, in microseconds, at which is it dropped
static const Time Drop_Possible_Track_Time = 500000;
static const Time Drop_Real_Track_Time = 500000;

static const float Position_Uncertainty = 0.5;

// Quickly removes element <i> from a vector without the preserving order of later elements
template<class T>
void fastRemove(T &v, unsigned int i)
{
	assert(i < v.size());
	int last = v.size() - 1;
	swap(v[i], v[last]);
	v.resize(last);
}

BallTracker::BallTracker()
{
}

void drawX(SystemState *state, Point center, const QColor &color = Qt::red)
{
	static const float R = Ball_Radius;
	state->drawLine(center + Point(-R, R), center + Point(R, -R), color);
	state->drawLine(center + Point(R, R), center + Point(-R, -R), color);
}

void BallTracker::run(const vector< BallObservation >& obs, SystemState *state)
{
	unsigned int n = obs.size();
	
	vector<const BallObservation *> goodObs;
	goodObs.resize(obs.size());
	for (unsigned int i = 0; i < n; ++i)
	{
		goodObs[i] = &obs[i];
	}
	
#if 0
	// Remove balls that are too close to other balls
	if (n <= 20)
	{
		for (unsigned int i = 0; i < n; ++i)
		{
			for (unsigned int j = 0; j < n; ++j)
			{
				if (i != j && (goodObs[i] || goodObs[j]))
				{
					if (obs[i].pos.nearPoint(obs[j].pos, Distance_Limit))
					{
						// Balls are too close together: remove them both
						if (goodObs[i])
						{
							drawX(state, obs[i].pos);
							goodObs[i] = 0;
						}
						if (goodObs[j])
						{
							drawX(state, obs[j].pos);
							goodObs[j] = 0;
						}
					}
				}
			}
		}
		
		// Remove null entries from goodObs, not preserving order
		for (unsigned int i = 0; i < goodObs.size(); ++i)
		{
			if (goodObs[i] == 0)
			{
				fastRemove(goodObs, i);
				--i;
			}
		}
	}
#endif

	Time now = timestamp();
	
	//FIXME - What time?
	Time predictTime = now;
	
	// Rectangle that defines the boundaries of the field.
	// Note that we are working in team space.
	const Rect field(Point(-Field_Dimensions::Current_Dimensions.Width() / 2, 0), Point(Field_Dimensions::Current_Dimensions.Width() / 2, Field_Dimensions::Current_Dimensions.Length()));
	
	// Update the real ball
	if (_ballFilter)
	{
		// Get a prediction for this frame from the filter
		Ball prediction;
		float velocityUncertainty = 0;
		_ballFilter->predict(predictTime, &prediction, &velocityUncertainty);
		
		Point windowCenter = prediction.pos;
		float windowRadius = Position_Uncertainty + velocityUncertainty * (predictTime - _lastTrackTime) / 1000000.0f;
		state->drawCircle(windowCenter, windowRadius, Qt::white);
		
		// Find the closest new observation to the real ball's predicted position
		float bestDist = -1;
		unsigned int best = 0;
		for (unsigned int i = 0; i < goodObs.size(); ++i)
		{
			float d = goodObs[i]->pos.distTo(windowCenter);
			if (goodObs[i]->time > _lastTrackTime && d <= windowRadius && (bestDist < 0 || d < bestDist))
			{
				bestDist = d;
				best = i;
			}
		}
		
		if (bestDist >= 0)
		{
			// Update the filter
			_ballFilter->update(goodObs[best]);
			_lastTrackTime = goodObs[best]->time;
			
			// Update the real track
			_ballFilter->predict(state->logFrame->command_time(), &state->ball, nullptr);
			
			// Don't use this observation for a possible track since it's the real track
			fastRemove(goodObs, best);
		}
		
		// If we haven't found an update in a long time, drop the real ball track
		if ((now - _lastTrackTime) >= Drop_Real_Track_Time)
		{
			_ballFilter.reset();
			state->ball.valid = false;
		}
	} else {
		state->ball.valid = false;
	}
	
	// Update possible tracks
	for (unsigned int i = 0; i < _possibleTracks.size(); ++i)
	{
		// Find the closest new observation to this track which is within Acquisition_Match_Distance
		float bestDist = -1;
		unsigned int best = 0;
		for (unsigned int j = 0; j < goodObs.size(); ++j)
		{
			float d = goodObs[j]->pos.distTo(_possibleTracks[i].obs.pos);
			if (field.contains(goodObs[j]->pos) && d <= Acquisition_Match_Distance && (bestDist < 0 || d < bestDist))
			{
				bestDist = d;
				best = j;
			}
		}
		
		if (bestDist >= 0)
		{
			// Update this track
			_possibleTracks[i].current = true;
			_possibleTracks[i].obs = *goodObs[best];
			++_possibleTracks[i].numFrames;
			
			// Remove from goodObs
			fastRemove(goodObs, best);
			
			drawX(state, _possibleTracks[i].obs.pos, Qt::yellow);
		} else {
			_possibleTracks[i].current = false;
		}
		
		if ((now - _possibleTracks[i].obs.time) >= Drop_Possible_Track_Time)
		{
			fastRemove(_possibleTracks, i);
			--i;
		}
	}
	
	// Any remaining observations will start new tracks
	for (unsigned int i = 0; i < goodObs.size(); ++i)
	{
		if (field.contains(goodObs[i]->pos))
		{
			_possibleTracks.push_back(PossibleTrack(*goodObs[i]));
		}
	}
	
	// Try to acquire the real ball
	if (!_ballFilter)
	{
		for (unsigned int i = 0; i < _possibleTracks.size(); ++i)
		{
			if (_possibleTracks[i].current && _possibleTracks[i].numFrames >= 3)
			{
				_ballFilter = std::make_shared<BallFilter>();
				
				// First update and prediction
				_ballFilter->update(&_possibleTracks[i].obs);
				_lastTrackTime = _possibleTracks[i].obs.time;
				_ballFilter->predict(state->logFrame->command_time(), &state->ball, nullptr);
				
				fastRemove(_possibleTracks, i);
				break;
			}
		}
	}
}
