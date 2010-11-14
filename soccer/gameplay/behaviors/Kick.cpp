#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;

const float Min_Segment = 1.5 * Ball_Diameter; //Tuning Required (Anthony)

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	setTargetGoal();
 
        //Whether or not the shot is feasible
        hasShot = false;
        //The segment of the best shot 
        _shotSegment = Geometry2d::Segment(Geometry2d::Point(0,0), Geometry2d::Point(0,0));
	restart();
}

bool Gameplay::Behaviors::Kick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}

        //Only end the behavior is the ball can't be seen and the robot doesn't have it
        //This prevents the behavior from stopping when the robot blocks the ball
        if(!ball().valid && !robot->hasBall)
        {
            return false;
        }
     
        //If the ball is being blocked set its location to the location of the robot 
        //Accounting for the direction the robot is facing
        Geometry2d::Point ballPos;

        if(!ball().valid)
        {
           float x = Robot_Radius * sin(robot->angle * DegreesToRadians);
           float y = Robot_Radius * cos(robot->angle * DegreesToRadians);
           Geometry2d::Point pt = Geometry2d::Point(x, y);
           ballPos = robot->pos + pt; 
        }
        else
        {
            ballPos = ball().pos;
        }

        //Get the best ublocked area in the target to aim at
        WindowEvaluator e = WindowEvaluator(state());
        e.run(ballPos, _target);
        Window *w = e.best;
        
        //The target to use
        Geometry2d::Segment target;

        //Prevents the segfault from using a non existent window
        if(w != NULL)
        {
            if(w->segment.length() < Min_Segment)
            {
                hasShot = false;
            }
            else
            {
                hasShot = true;
            }

            target = w->segment;
            _shotSegment = target;
        }
        else
        {
            hasShot = false;
            target = _target;

            //There is not shot therefore set the best segment to a single point
            _shotSegment = Geometry2d::Segment(Geometry2d::Point(0,0), Geometry2d::Point(0,0));
        }

	// Some calculations depend on the order of the target endpoints.
	// Ensure that t0 x t1 > 0.
	// We have to do this each frame since the ball may move to the other side of the target.
	//FIXME - Actually, doesn't that mean the kick/pass is done?
	//FIXME - What about kicking towards a point?  cross product is zero...
	if ((target.pt[0] - ballPos).cross(target.pt[1] - ballPos) < 0)
	{
		swap(target.pt[0], target.pt[1]);
	}

	
	// State transitions
	switch (_state)
	{
		case State_Approach1:
                {
                        Geometry2d::Point targetCenter = target.center();
			
                        // Vector from ball to center of target
			Geometry2d::Point toTarget = targetCenter - ballPos;
                        
                        // Robot position relative to the ball
			Geometry2d::Point relPos = robot->pos - ballPos;

                        //The Point to compute with
                        Geometry2d::Point point = target.pt[0];
			
			//Behind the ball: move to the nearest line containing the ball and a target endpoint.
			//the robot is behind the ball, while the target vectors all point in *front* of the ball.
			if (toTarget.cross(relPos) < 0)
			{
				// Above the center line: nearest endpoint-line includes target.pt[1]
				point = target.pt[1];
	               	}

                        //Change state when the robot is in the right location
                        //facing the right direction
                        Geometry2d::Point b = (point - ballPos + robot->pos).normalized();

                        float angleError = b.dot(Geometry2d::Point::direction(robot->angle * DegreesToRadians));
			bool nearBall = robot->pos.nearPoint(ballPos, Robot_Radius + Ball_Radius + 0.25);
                       
                        //angleError is greater than because cos(0) is 1 which is perfect 
                        if (nearBall && angleError > cos(20 * DegreesToRadians))
			{
				_state = State_Approach2;
			}
			break;
                }
		
		case State_Approach2:
                {
                        if (robot->hasBall && robot->charged())
			{
				robot->addText("Aim");
				_state = State_Aim;
				_lastError = INFINITY;
			}
                        
			bool nearBall = robot->pos.nearPoint(ballPos, Robot_Radius + Ball_Radius + 0.30);

                        //Go back to state one if needed
                        if(!nearBall)
                        {
                            _state = State_Approach1;
                        }
                       
			break;
                }

		case State_Aim:
			if (!robot->hasBall)
			{
				_state = State_Approach2;
			} else {
				state()->drawLine(ballPos, target.pt[0], Qt::red);
				state()->drawLine(ballPos, target.pt[1], Qt::white);
				
				Geometry2d::Point rd = Geometry2d::Point::direction(robot->angle * DegreesToRadians);

				_kickSegment = Geometry2d::Segment(robot->pos, robot->pos + rd * Field_Length);
				state()->drawLine(robot->pos, target.center(), Qt::gray);
				float error = acos(rd.dot((target.center() - robot->pos).normalized())) * RadiansToDegrees;
			
                                //The distance between the trajectory and the target center
                                float distOff = _kickSegment.distTo(target.center());
                                
                                //The width of half the target 
                                float width = target.pt[0].distTo(target.center());

                                robot->addText(QString("Aim %1").arg(error));
				
				if (!isinf(_lastError))
				{
					//FIXME - Depends on t0 x t1 > 0
					bool inT0 = (target.pt[0] - ballPos).cross(rd) > 0;
					bool inT1 = rd.cross(target.pt[1] - ballPos) > 0;
					robot->addText(QString("in %1 %2").arg(inT0).arg(inT1));

                                        //How close to the center line printout
                                        robot->addText(QString("Width %1 Distance %2").arg(width).arg(distOff));					
                                        
                                        //If the tarjectory is within the target bounds
                                        if (inT0 && inT1)
                                        {
                                            //Shoot if the shot is getting worse or the shot is
                                            //very good (within half of the width of half the window) (Tuning required)
                                            if((error > _lastError) || (distOff < (width * .5)))
		                            {
						// Past the best position
						_state = State_Kick;
					    }
                                        }
				}
				_lastError = error;
			}
			break;
		
		case State_Kick:

			if (!robot->charged())
			{
				_state = State_Done;
			}
                        
                        //If the robot loses the ball whilst trying to kick before the kick is done
                        //go back to approach1 to reacquire the ball
                        else if(!robot->hasBall)
                        {
                            _state = State_Approach1;
                        }
			break;
		
		case State_Done:
			break;
	}
	
	switch (_state)
	{
		case State_Approach1:
                {
                        robot->addText("Approach1");
			
                        Geometry2d::Point targetCenter = target.center();
		
			// Vector from ball to center of target
			Geometry2d::Point toTarget = targetCenter - ballPos;
                        
                        // Robot position relative to the ball
			Geometry2d::Point relPos = robot->pos - ballPos;
			
                        //The Point to compute with
                        Geometry2d::Point point = target.pt[0];
			
			//Behind the ball: move to the nearest line containing the ball and a target endpoint.
			//the robot is behind the ball, while the target vectors all point in *front* of the ball.
			if (toTarget.cross(relPos) < 0)
			{
				// Above the center line: nearest endpoint-line includes target.pt[1]
				point = target.pt[1];
	               	}
			
                        //Face that point and move to the appropriate line
                        robot->move(ballPos + (ballPos -
                                    point).normalized() * (Robot_Radius +
                                        Ball_Radius + .07));
                        robot->face(point - ballPos + robot->pos);

		        state()->drawLine(ballPos, point, Qt::red);

			//FIXME - Real robots overshoot and hit the ball in this state.  This shouldn't be necessary.
                        //Hopefully moving the target point back and pivoting whilst moving will prevent the ball 
                        //from getting hit thus there shouldn't need to be a dribbler
			robot->dribble(127);
			
			robot->avoidBall = true;
			break;
                }

		case State_Approach2:
			robot->addText("Approach2");
                        robot->move(ballPos);

                        //Should this face be ballPos or the quanity found in Approach1?
			robot->face(ballPos);
			robot->dribble(127);
			break;
			
		case State_Aim:
		{
			Geometry2d::Point targetCenter = target.center();
			
			// Vector from ball to center of target
			Geometry2d::Point toTarget = targetCenter - ballPos;
			
			// Robot position relative to the ball
			Geometry2d::Point relPos = robot->pos - ballPos;
			
			// True if the robot is in front of the ball
			bool inFrontOfBall = toTarget.perpCCW().cross(relPos) > 0;
			
			MotionCmd::PivotType dir;
			if (inFrontOfBall)
			{
				// Move behind the ball
				dir = (toTarget.cross(relPos) > 0) ? MotionCmd::CCW : MotionCmd::CW;
			} else {
				// Behind the ball: move to the nearest line containing the ball and a target endpoint.
				// Note that the robot is behind the ball, while the target vectors all point in *front* of the ball.
				//FIXME - These assume t0 x t1 > 0.  Enforce this here or above.
				if (toTarget.cross(relPos) > 0)
				{
					// Below the center line: nearest endpoint-line includes target.pt[0]
					dir = (target.pt[0] - ballPos).cross(relPos) > 0 ? MotionCmd::CW : MotionCmd::CCW;
				} else {
					// Above the center line: nearest endpoint-line includes target.pt[1]
					dir = (target.pt[1] - ballPos).cross(relPos) > 0 ? MotionCmd::CCW : MotionCmd::CW;
				}
			}
			
			robot->dribble(127);

                        //Robot has trouble handling a moving ball when trying to pivot (I don't know how to fix this)
			robot->pivot(ballPos, dir);
			state()->drawLine(_kickSegment);
			break;
		}
			
		case State_Kick:
			robot->addText("Kick");
                        if(hasShot)
                        {
                            robot->move(ballPos);
		            robot->face(ballPos);
			    robot->dribble(127);
			    robot->kick(255);
                        }
			state()->drawLine(_kickSegment);
                        break;
		
		case State_Done:
			robot->addText("Done");
			state()->drawLine(_kickSegment);
			break;
	}
	
	return _state != State_Done;
}

void Gameplay::Behaviors::Kick::restart()
{
	_state = State_Approach1;
}

void Gameplay::Behaviors::Kick::setTargetGoal()
{
        //Set the target to be a little inside of the goal posts to prevent noise errors from 
        //causing a post shot
	setTarget(Geometry2d::Segment(
		Geometry2d::Point((Field_GoalWidth / 2 - Ball_Diameter), Field_Length),
		Geometry2d::Point((-Field_GoalWidth / 2 + Ball_Diameter), Field_Length)));
}

void Gameplay::Behaviors::Kick::setTarget(const Geometry2d::Segment& seg)
{
	_target = seg;
}
