#pragma once

#include <gameplay/ActionBehavior.hpp>

namespace Gameplay {

	class PassReceiver : public ActionBehavior {
	public:
		static void createConfiguration(Configuration *cfg);

		PassReceiver(GameplayModule *gameplay);

		bool run();

		bool isSettingUp();
		bool isSetup();
		bool isActing();
		bool isDone();
		void restart();



		void partnerDidKick();




		Geometry2d::Point kickPoint;
		float kickAngle;
		// bool _passStarted;



	private:
		enum State {
			Positioning,	//	we're working on getting into position for receiving
			Positioned,		//	we're where we want to be, it's time for the passer to kick the ball
			Receiving,		//	make a best effort to get to where the ball's going
			Failed,			//	we missed the pass :(
			Completed		//	we got the pass!
		};






		//	what angle the robot's mouth should be relative to the ball
		//	a value of zero means the robot should look at the ball head-on
		// float _relativeMouthAngle;	//	note: not used anymore




		// Return line up position
		Geometry2d::Point receivePosition();

		float receivePositionError();

		bool isAtReceivePosition();

		// bool isFailedReceive();




		Geometry2d::Point getFacePoint();
		Geometry2d::Point targetMouthPosition();
		Geometry2d::Point currentMouthPosition();
		// float targetGlobalMouthAngle();
		Geometry2d::Point targetCenterPosition();
		void getPassRay(Geometry2d::Point &startPt, float &angle);
		bool passMissed();
		bool passStarted();
		bool ballBehindRobot();
		bool passTimedOut();
		Geometry2d::Point closestPointOnRayToPoint(Geometry2d::Point &rayStart, float rayAngle, Geometry2d::Point &pt);



		State _state;
		bool _success;
		uint64_t _kickStartTime;


		static ConfigDouble *_backoffDistance;
		static ConfigDouble *_maxExecutionError;
		static ConfigDouble *_kickTimeout;

		static ConfigBool *_debug;
	};
}
