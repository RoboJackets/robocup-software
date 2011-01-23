#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;

#define DEBUG 1

//Smallest segment that the robot will shoot at
const float Min_Segment = 1.5 * Ball_Diameter; //Tuning Required (Anthony)

//Speed at which the ball is effectively not moving
const float ballVelThreshold = 0.009;   //Tuning Required (Anthony)

//Position behind the ball by an amount where the robot will have space to align itself 
//without hitting the ball
const float yOffset = 1.5 * Robot_Radius; //Tuning Required (Anthony) 

//Used for the case where the ball is directly behind the robot so that i
//the robot doesn't run over it attempting to get behind it
const float xOffset = 2 * Robot_Radius; //Tuning Required (Anthony) 

//const int Max_Approach1_Timeout = 400; //Tuning
const int Max_Face_Timeout = 60; //Tuning 
const int Max_Approach2_Timeout = 100; //Tuning
const int Max_Aim_Timeout = 75; //Tuning

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	setTargetGoal();
 
        //Whether or not the shot is feasible
        hasShot = false;
        //The segment of the best shot 
        _shotSegment = Geometry2d::Segment(Geometry2d::Point(0,0), Geometry2d::Point(0,0));
	_faceTimeout = 0;   
	_aimTimeout = 0;   
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
        if(!ball().valid && !robot->hasBall && !_override)
        {
            return false;
        }
     
        //If the ball is being blocked set its location to the location of the robot 
        //Accounting for the direction the robot is facing
        Geometry2d::Point ballPos = ball().pos;

/*      if(!ball().valid)
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
*/

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

       
//Find the point to use the the first approach method  
        //The interceptPoint Always behind the ball
        //Eventually this should take into account where the robot starts in relation to the ball 
        Geometry2d::Point interceptPoint;

        //Create the interceptPoint by choosing a point behind the ball
        //Account for a moving ball and guess at where the ball will end up
        Geometry2d::Point ballVel = ball().vel;

        //Ball is effectively stationary
        if(ballVel.magsq() <= (ballVelThreshold * ballVelThreshold)) 
        {
                interceptPoint = ballPos;
        }
        //Ball is moving calculate where to intercept it
        else 
        {
                interceptPoint = calculateInterceptPoint();
        }

        //Offset the target to avoid running into it
        if((ballPos.y - Robot_Radius) < robot->pos.y)
        {
                //Always compensate to the outside
                if((ballPos.x > Robot_Radius) && (ballPos.x > robot->pos.x))
                {
                        interceptPoint.x -= xOffset; 
                }
                else if((ballPos.x > Robot_Radius) && (ballPos.x < robot->pos.x))
                {
                        interceptPoint.x += xOffset;
                }
                else if((ballPos.x < -Robot_Radius) && (ballPos.x < robot->pos.x))
                {
                        interceptPoint.x += xOffset;
                }
                else if((ballPos.x < -Robot_Radius) && (ballPos.x > robot->pos.x))
                {
                        interceptPoint.x -= xOffset;
                }
                else //The ball is effectively in the center (width) of the field
                {
                        if(robot->pos.x > Robot_Radius)
                        {
                                interceptPoint.x += xOffset;
                        }
                        else if(robot->pos.x < -Robot_Radius)
                        {
                                interceptPoint.x -= xOffset;
                        }
                        else //Robot is effectively in the center (width) doesn't matter which is used
                        {
                                interceptPoint.x -= xOffset;
                        }
                }
        }
        else
        {
                //Always compensate to the outside
                //Use Robot Radius to cancel out chance of a stuck zone
                if(ballPos.x > Robot_Radius)
                {
                        interceptPoint.x += xOffset; 
                }
                else if(ballPos.x < -Robot_Radius)
                {
                        interceptPoint.x -= xOffset;
                }
                else //The ball is effectively in the center (Width) of the field
                {
                        //Here just flip the value from the first part
                        if(robot->pos.x > (4 * Robot_Radius)) //Widen the margins to prevent oscillation
                        {
                                interceptPoint.x -= xOffset;
                        }
                        else if(robot->pos.x < -(4 * Robot_Radius))
                        {
                                interceptPoint.x += xOffset;
                        }
                        else //Robot is effectively in the center (width) doesn't matter which is used
                        {
                                interceptPoint.x += xOffset;
                        }
                }
        }
       
        interceptPoint.y -= yOffset;

        //Check to make sure the robot doesn't leave the field
        if(interceptPoint.x < - (Field_Width / 2))
        {
                interceptPoint.x = - (Field_Width /2);
        }
        else if(interceptPoint.x > (Field_Width / 2))
        {
                interceptPoint.x = (Field_Width / 2);
        }

        if(interceptPoint.y < 0)
        {
                interceptPoint.y = 0;
        }
        else if(interceptPoint.y > Field_Length)
        {
                interceptPoint.y = Field_Length;
        }

//Information about the Ball location relative to the robot used in multiple states
        Geometry2d::Point targetCenter = target.center();
			
        // Vector from ball to center of target
	Geometry2d::Point toTarget = targetCenter - ballPos;
                        
        // Robot position relative to the ball
	Geometry2d::Point relPos = robot->pos - ballPos;

        //The Point to compute with
        Geometry2d::Point targetEdge = target.pt[0];
			
	//Behind the ball: move to the nearest line containing the ball and a target endpoint.
	//the robot is behind the ball, while the target vectors all point in *front* of the ball.
	if (toTarget.cross(relPos) < 0)
	{
		// Above the center line: nearest endpoint-line includes target.pt[1]
		targetEdge = target.pt[1];
      	}
        
//Information about whether the robot should skip to future steps 
        //Determines whether or not the shot is there
        Geometry2d::Point rd = Geometry2d::Point::direction(robot->angle * DegreesToRadians);
	_kickSegment = Geometry2d::Segment(robot->pos, robot->pos + rd * Field_Length);
	
//Skip to Kick Utilities
        //Checks to see if within the target
        bool inT0 = (target.pt[0] - ballPos).cross(rd) > 0;
	bool inT1 = rd.cross(target.pt[1] - ballPos) > 0;
        //How far off center we are
        float distOff = _kickSegment.distTo(target.center());
        //Distance from center to edge of target 
        float width = target.pt[0].distTo(target.center());
        
//Skip to Approach2 Utilities
        Geometry2d::Point b = (targetEdge - ballPos + robot->pos).normalized();
        float angleError = b.dot(Geometry2d::Point::direction(robot->angle * DegreesToRadians));


        bool skipToKick = (inT0 && inT1 && (distOff < width * .7) && robot->hasBall && robot->charged());
        bool skipToAim = robot->hasBall && robot->charged();
        bool skipToApproach2 = angleError > cos(15 * DegreesToRadians); 

	// State transitions
	switch (_state)
	{
		case State_Approach1:
                {
                        //Clear/Increment Timeouts
                        _approach1Timeout++;
                        _aimTimeout = 0;

                        bool nearIntercept = robot->pos.nearPoint(interceptPoint, Robot_Radius + 0.05);
                        
#ifdef DEBUG
                        robot->addText("Approach1");
	                robot->addText("");
                        robot->addText(QString("Near Target %1").arg(nearIntercept));
                        robot->addText(QString("X Pos %1 Target X %2").arg(robot->pos.x).arg(interceptPoint.x));
                        robot->addText(QString("Y Pos %1 Target Y %2").arg(robot->pos.y).arg(interceptPoint.y));
                        robot->addText(QString("Skip to Approach2 %1").arg(skipToApproach2));
                        robot->addText(QString("Skip to Aim %1").arg(skipToAim));
                        robot->addText(QString("Skip to Kick %1").arg(skipToKick));
#endif

                        //Change States do in order so that the last one is the futherest along
                        if (nearIntercept) // || _approach1Timeout > Max_Approach1_Timeout)
			{
				_state = State_Face;
                        }
                        if(skipToApproach2)
                        {
                                _state = State_Approach2;
                        }
                        if(skipToAim)
                        {
                                _state = State_Aim;
                        }
                        if(skipToKick)
                        {
                                _state = State_Kick;
                        }

			break;
                }
	
                case State_Face:
                {
                        //Clear/Increment Timeouts
                        _faceTimeout++;
			
                        bool nearIntercept = robot->pos.nearPoint(interceptPoint, Robot_Radius + 0.20);

#ifdef DEBUG
                        robot->addText("Face");
	                robot->addText("");
                        robot->addText(QString("Angle %1 Threshold %2").arg(angleError).arg(cos(15 * DegreesToRadians)));
                        robot->addText(QString("Timeout %1").arg(_faceTimeout));
                        robot->addText(QString("Skip to Aim %1").arg(skipToAim));
                        robot->addText(QString("Skip to Kick %1").arg(skipToKick));
#endif

                        //Change States do in order so that the last one is the futherest along
                        if (!nearIntercept)
			{
				_state = State_Approach1;
                        }
                        if(angleError > cos(15 * DegreesToRadians) || (_faceTimeout >= Max_Face_Timeout))
                        {
                            _state = State_Approach2;
                        }
                        if(skipToAim)
                        {
                                _state = State_Aim;
                        }
                        if(skipToKick)
                        {
                                _state = State_Kick;
                        }

                        break;
                }

		case State_Approach2:
                {
                        //Clear/Increment Timeouts
                        _approach1Timeout = 0;
                        _approach2Timeout++;
			
                        bool nearIntercept = robot->pos.nearPoint(interceptPoint, Robot_Radius + 0.25);
                        bool nearBall = robot->pos.nearPoint(ballPos, Robot_Radius + .05);
                       
#ifdef DEBUG
                        robot->addText("Approach2");
	                robot->addText("");
                        robot->addText(QString("Face Timeout %1").arg(_faceTimeout));
                        robot->addText(QString("Approach2 Timeout %1").arg(_approach2Timeout));
                        robot->addText(QString("Skip to Kick %1").arg(skipToKick));
#endif

                        //Change States do in order so that the last one is the futherest along
                        if (!nearIntercept && !nearBall)
			{
				_state = State_Approach1;
                        }
                        if ((robot->hasBall && robot->charged()) || _approach2Timeout > Max_Approach2_Timeout)
			{
				_state = State_Aim;
				_lastError = INFINITY;
			}
                        if(skipToKick)
                        {
                                _state = State_Kick;
                        }
                        
			break;
                }

		case State_Aim:
                {
                        //Clear/Increment Timeouts
                        _approach1Timeout = 0;
                        _faceTimeout = 0;
                        _aimTimeout++;
                        
                        bool nearBall = robot->pos.nearPoint(ballPos, Robot_Radius + .10);
			
#ifdef DEBUG                        
                        robot->addText("Aim");
	                robot->addText("");
#endif                        

                        if (((!robot->hasBall && !nearBall) || !robot->charged()) && _approach2Timeout <=
                                Max_Approach2_Timeout)  
			{
				_state = State_Approach2;
			}
                        else 
                        {
				state()->drawLine(ballPos, target.pt[0], Qt::red);
				state()->drawLine(ballPos, target.pt[1], Qt::white);
				
				state()->drawLine(robot->pos, target.center(), Qt::gray);
				float error = acos(rd.dot((target.center() - robot->pos).normalized())) * RadiansToDegrees;
			
#ifdef DEBUG
                                robot->addText(QString("Error %1").arg(error));
                                robot->addText(QString("Timeout %1").arg(_aimTimeout));
#endif

				if (!isinf(_lastError) )
				{

#ifdef DEBUG
                                        robot->addText(QString("in %1 %2").arg(inT0).arg(inT1));
                                        robot->addText(QString("Width %1 DistOff %2").arg(width).arg(distOff));					
#endif

                                        //If the tarjectory is within the target bounds
                                        if (inT0 && inT1)
                                        {
                                            //Shoot if the shot is getting worse or the shot is very good
                                            if((((distOff < (width * .95)) && (error > _lastError)) || (distOff < (width * .7))) && hasShot)
		                            {
						_state = State_Kick;
					    }
                                        }
				}
				_lastError = error;

                                if(_aimTimeout >= Max_Aim_Timeout)
                                {
                                    _state = State_Kick;
                                }
			}

			break;
                }

		case State_Kick:
                {
                        //Clear/Increment Timeouts
                        _approach1Timeout = 0;
                        _faceTimeout = 0;
                        _approach2Timeout = 0;
                        _aimTimeout++;

#ifdef DEBUG
                        robot->addText("Kick");
                        robot->addText("");
                        robot->addText(QString("Has Shot %1").arg(hasShot));
                        robot->addText(QString("Timeout %1").arg(_aimTimeout));
#endif

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
                }

		case State_Done:
                {
                        //Clear all the Timeouts
                        _approach1Timeout = 0;
                        _faceTimeout = 0;
                        _approach2Timeout = 0;
                        _aimTimeout = 0;

#ifdef DEBUG
                        robot->addText("Done");
                        robot->addText("");
#endif

                        break;
                }
	}
	
	switch (_state)
	{
		case State_Approach1:
                {
                        //Move to the appropriate point
                        robot->move(interceptPoint);
                        
		        state()->drawLine(ballPos, targetEdge, Qt::red);

                        state()->drawLine(robot->pos, Geometry2d::Point::direction(robot->angle * DegreesToRadians) + robot->pos, Qt::gray);
			
			robot->avoidBall = true;
			break;
                }

                case State_Face:
                {
                        MotionCmd::PivotType dir = (targetEdge - ballPos).cross(relPos) > 0 ? MotionCmd::CW : MotionCmd::CCW;
			
                        if (toTarget.cross(relPos) < 0)
			{
				dir = (targetEdge - ballPos).cross(relPos) > 0 ? MotionCmd::CCW : MotionCmd::CW;
	               	}
                        
                        robot->pivot(targetEdge - ballPos + robot->pos, dir);
                        //robot->face(ballPos);
                        robot->dribble(127);

                        state()->drawLine(ballPos, targetEdge, Qt::red);

                        state()->drawLine(robot->pos, Geometry2d::Point::direction(robot->angle * DegreesToRadians) + robot->pos, Qt::gray);
                     
			robot->avoidBall = true;
                        break;
                }

		case State_Approach2:
                {
                        robot->avoidBall = false;

                        //Create a point that is offset from the center of the ball so that the robot doesn't hit the ball out of the way
                        Geometry2d::Point point;
                        point.x = ballPos.x;
                        point.y = ballPos.y - (Ball_Radius);

                        robot->move(point);

                        //Should this face be ballPos or the quanity found in Approach1?
			robot->face(ballPos);
			robot->dribble(127);
			break;
                }

		case State_Aim:
		{
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
			
			robot->dribble(50); // previously 127

                        //Robot has trouble handling a moving ball when trying to pivot (I don't know how to fix this)
			robot->pivot(ballPos, dir);
		        robot->move(ballPos);
                        state()->drawLine(_kickSegment);
			break;
		}
			
		case State_Kick:
                {
                        Geometry2d::Point p = ballPos;
                        p.y += yOffset;

                        if(hasShot || _aimTimeout >= Max_Aim_Timeout)
                        {
                            robot->move(p);
		            robot->face(ballPos);
			    robot->dribble(60); // previously 127
			    robot->kick(255);
                        }
			state()->drawLine(_kickSegment);
                        break;
                }

		case State_Done:
                {
			state()->drawLine(_kickSegment);
			break;
                }
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

Geometry2d::Point Gameplay::Behaviors::Kick::calculateInterceptPoint()
{
        Geometry2d::Point interceptPoint;
        Geometry2d::Point ballPos = ball().pos;
        Geometry2d::Point ballVel = ball().vel;
        float dist = ballPos.distTo(robot->pos);
        //TODO: Make time a better function based on distance apart
        float time = dist * .6; //Tuning Required

        interceptPoint = ballPos + Geometry2d::Point::saturate(ballVel * time, 1);

        return interceptPoint;
}
