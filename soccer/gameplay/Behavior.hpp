#pragma once

#include <AutoName.hpp>

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
		
		SystemState *state() const
		{
			return _gameplay->state();
		}

		// Removes robot assignments and resets the behavior.
		// Calls assign() with an empty set.
		void unassign();
		
		// Returns true if any robots are assigned to this behavior
		bool assigned() const
		{
			return !_robots.empty();
		}

		// Returns true if all robots assigned to this behavior are visible.
		// Returns false if no robots are assigned.
		virtual bool allVisible() const;

		std::set<Robot *> &robots()
		{
			return _robots;
		}

		// Simple way to get the robot for one-robot behaviors
		Robot *robot() const
		{
			if (_robots.empty())
			{
				return 0;
			} else {
				return *_robots.begin();
			}
		}

		// Assigns a single robot to this behavior
		void assignOne(Robot *r)
		{
			std::set<Robot *> available;
			available.insert(r);
			assign(available);
		}

		// Assigns robots to this behavior.  The behavior is responsible for picking as many robots as it wants from <available>.
		// The assigned robots shall be removed from <available> when this function returns.
		//
		// The default implementation just calls takeBest(available).
		virtual bool assign(std::set<Robot *> &available);

		// Called each frame when this behavior is current.
		// The default implementation does nothing.
		//
		// Returns true if this behavior needs to continue or false if it is done and may be replaced by another behavior.
		// The behavior may continue to be used after run() returns false.
		virtual bool run() = 0;

	protected:
		GameplayModule *_gameplay;
		std::set<Robot *> _robots;

		// Finds the best (lowest-scoring) robot in <available>, removes it from <available>, adds it to _robots, and returns it.
		// Returns 0 if <available> is empty.
		Robot *takeBest(std::set<Robot *> &available);

		// Assigns all robots in <available> to this behavior.
		bool takeAll(std::set<Robot *> &available);

		// Returns a score for the given robot.
		// This is used to pick from available robots in takeBest(), which is typically called by assign().
		// The robot with the LOWEST score will be used.
		//
		// The default implementation always returns zero.
		virtual float score(Robot *r);

		// Convenience functions
		const SystemState::Ball &ball() const
		{
			return gameplay()->state()->ball;
		}

		const GameState &gameState() const
		{
			return _gameplay->state()->gameState;
		}

		Gameplay::Robot *self(int i) const
		{
			return _gameplay->self[i];
		}

		const Gameplay::Robot *opp(int i) const
		{
			return _gameplay->opp[i];
		}
	};
}
