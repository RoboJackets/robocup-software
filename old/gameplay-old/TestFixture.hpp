#pragma once

#include "Play.hpp"

namespace Gameplay
{
class GameplayModule;
	namespace Testing
	{

		/**
		 * Storage for failures
		 * Each failure contains a string noting the failure type and location
		 * Failures will be largely determined by the evaluation functions
		 * TODO: make this more robust
		 */
		class TestResult {
		public:
			TestResult(const std::string& label) : _label(label) {}

		protected:
			std::string _label;
		};

		/**
		 * A test fixture wraps an existing play/behavior and allows it to be called
		 * in various states.  The user should override the run function to provide
		 * to create the test structure - basically run the behavior under test and
		 * then evaluate its state.
		 *
		 * The basic approach to testing is to check for failures
		 */
		class TestFixture : public Play
		{
		public:
			typedef std::vector<TestResult> ResultSet;

			/** generic play constructor: user should initialize the behavior under test */
			TestFixture(GameplayModule *gameplay);

			/** play is always enabled */
			static float score(GameplayModule *gameplay) { return 0; }

			/** sets up the test, then runs the evaluation function, and update result
			 * indicators as necessary
			 */
			virtual bool run();

			/**
			 * Setup test - moves robots/ball in to correct state to make starting
			 * cases predictable.  Will be called upon first run
			 */
			virtual bool setupTest();

			/**
			 * Runs the test, checking status of each element that is under test
			 */
			virtual ResultSet excuteTest();

			// TODO: add evaluation functions to determine pass/fail
			// TODO: add indicators for pass/fail
		};
	}
}
