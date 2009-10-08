#include "KickerCalibration.hpp"

#include "../../Role.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::KickerCalibration> behavior("kicker_calibration");

Gameplay::Behaviors::KickerCalibration::KickerCalibration(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role),
	_dribble(gameplay, 0)
{
	fp = 0;
}

void Gameplay::Behaviors::KickerCalibration::start()
{
	if (fp)
	{
		fclose(fp);
	}

	fp = fopen("kicker_calibration.txt", "wt");
	if (!fp)
	{
		printf("Can't write to kicker_calibration.txt: %m\n");
	}

	_state = Wait;
	_strength = 16;
	_dribble.robot(robot());
	_dribble.start();
//	_dribble.dest.set(Geometry2d::Point(0, 1));
}

void Gameplay::Behaviors::KickerCalibration::run()
{
	// Starting position for each kick
	const Geometry2d::Point start(0, 1);

	const Geometry2d::Point pos = robot()->pos();
	const Geometry2d::Point ballPos = ball().pos;

	const uint64_t timestamp = gameplay()->state()->timestamp;

	robot()->face(ballPos);

	switch (_state)
	{
		case Wait:
#if 1
			robot()->move(start);
			if (robot()->pos().nearPoint(start, 0.1f) && robot()->pos().nearPoint(ball().pos, 0.3f))
			{
				_state = Setup;
			}
#else
			_dribble.run();
			if (_dribble.done())
			{
				_state = Setup;
			}
#endif
			break;

		case Setup:
			robot()->willKick = true;
			robot()->dribble(20);
			robot()->move(ballPos);
			if (robot()->haveBall())
			{
				robot()->kick(_strength);
				_state = Shoot;
				if (fp)
				{
					fprintf(fp, "kick %d\n", _strength);
				}
			}
			break;

		case Shoot:
			robot()->willKick = true;
			robot()->kick(_strength);
			if (!robot()->haveBall())
			{
				_startPos = ballPos;
				_state = Record;
			}
			break;

		case Record:
			if (fp)
			{
				// Distance and speed
				fprintf(fp, "%f %f\n", (ballPos - _startPos).mag(), ball().vel.mag());
			}
			if (ballPos.nearPoint(_oldBallPos, 0.1f))
			{
				if ((timestamp - _oldTime) > 1000000)
				{
					// The ball has stopped
					_strength += 16;
					if (_strength < 256)
					{
						_oldBallPos = ballPos;
						_oldTime = timestamp;
						_dribble.start();
						_state = Wait;
					} else {
						_state = Done;
					}
				}
			} else {
				_oldBallPos = ballPos;
				_oldTime = timestamp;
			}
			break;

		case Done:
			// Face away from the ball to indicate that we're done
			robot()->face(pos + pos - ballPos);

			// Close the output file
			if (fp)
			{
				fprintf(fp, "done\n");
				fclose(fp);
				fp = 0;
			}
			break;
	}
}

bool Gameplay::Behaviors::KickerCalibration::done()
{
	return _state == Done;
}
