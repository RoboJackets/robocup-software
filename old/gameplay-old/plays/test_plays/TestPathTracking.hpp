#pragma once

#include "../../TestFixture.hpp"


namespace Gameplay {
class GameplayModule;
	namespace Testing {

		/**
		 * This test evaluates the robot's tracking of a given path over time
		 */
		class TestPathTracking : public TestFixture {
		public:

			TestPathTracking(GameplayModule* gameplay);

			/**
			 * Sets the state so that there is only one robot on the field
			 */
			virtual bool setupTest();

			/**
			 * Moves the robot through a path
			 * TEST: robot gets to destination
			 * TEST: robot does not deviate too much
			 */
			virtual ResultSet excuteTest();
		};
	}
}
