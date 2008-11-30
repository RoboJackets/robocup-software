#ifndef SYSTEMSTATE_HPP_
#define SYSTEMSTATE_HPP_

#include <LogFrame.hpp>
#include <map>

/** The system state is an aggregate data structure for storing current system 
 * state information through a loop of the system. A state variable is not valid 
 * until that module has run */
class SystemState : public Packet::LogFrame
{
	public:

	  /** Robot state - Position: X, Y coords and angle (degrees) */
	  typedef struct {
		double px, py, pa; 
	  } robot_p;
	  /** Robot state  - Position, Velocity: X, Y coords and angle (degrees) */
	  typedef struct {
		double px, py, pa;
		double vx, vy, va;
	  }	robot_pv;
	  /** Ball State - Position: X, Y coords */
	  typedef struct {
		double px, py;
	  } ball_p;
	  /** Ball State - Position, Velocity: X, Y coords */
	  typedef struct {
		double px, py;
		double vx, vy;
	  } ball_pv;
	  
	  /** World model PV states for robots on the home team.  
		* Use id to look up robot. Currently allows for 
		* more than 5 robots on the team */
	  std::map<int, robot_pv> home_team;

	  /** World model PV states for robots on the opposing team.  
		* Use id to look up robot. Currently allows for 
		* more than 5 robots on the team */
	  std::map<int, robot_pv> opp_team;

	  /** Current best estimate at state of the ball.
		* May be switched with a better estimate at any 
		* time. */
	  ball_pv ball;


		
	private:
};

#endif /* SYSTEMSTATE_HPP_ */
