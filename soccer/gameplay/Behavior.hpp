#pragma once

#include <string>

#include <Geometry2d/TransformMatrix.hpp>
#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <AutoName.hpp>

#include "Play.hpp"
#include "GameplayModule.hpp"

namespace Gameplay
{
	class Behavior: public AutoName
	{
	public:
		Behavior(GameplayModule *gameplay);
		virtual ~Behavior();

		GameplayModule *gameplay() const
		{
			return _gameplay;
		}
		
		Robot *robot() const
		{
			return _robot;
		}
		
		// Returns a score judging how good the given robot is for this role containing this tactic.
		// The robot which receives the lowest score will be assigned to this behavior.
		//
		// The default implementation always returns 0.
		virtual float score(Robot *robot);

		// Returns the best robot for this behavior (the one with the lowest score).
		// This is guaranteed to return a robot if any are available.
		Robot *selectRobot();

		// Assigns a robot to this tactic.
		// stop() and start() are called as appropriate.
		void robot(Robot *robot);

		// Called when this becomes the current tactic for a role.
		// The default implementation does nothing.
		virtual void start();

		// Called when this stops being the current tactic for a role
		// (another tactic will become current or the play has ended).
		// The default implementation does nothing.
		virtual void stop();

		// Called each frame when this tactic is current.
		// The default implementation does nothing.
		virtual void run();

		// Returns true if this tactic allows the play to advance.
		// The default implementation always returns true.
		virtual bool done();

	protected:
		GameplayModule *_gameplay;
		Robot *_robot;
		
		// Convenience function for getting the ball state
		const Packet::LogFrame::Ball &ball() const
		{
			return gameplay()->state()->ball;
		}
	};
}
