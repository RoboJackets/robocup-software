#ifndef _STRUCTURES_HPP
#define _STRUCTURES_HPP

#include <map>

/* 
 * Structures for passing information through framework
 */

typedef struct {
  double px;
  double py; 
  double ptheta;
} robot_p;

typedef struct {
  double px;
  double py; 
  double ptheta;
  double vx;
  double vy; 
  double vtheta;
} robot_pv;

typedef struct {
  double px;
  double py; 
} ball_p;

typedef struct {
  double px;
  double py; 
  double vx;
  double vy;
} ball_pv;

typedef struct {
  Map<int, robot_p> robots;
  Map<int, ball_p> balls;
} state_p;

typedef struct {
  Map<int, robot_pv> robots;
  Map<int, ball_pv> balls;
} state_pv;

typedef struct {
  double vx; 
  double vy;
  double vtheta;
} robot_control;

//TODO: Replace with real command structure
typedef int command;
typedef Map<int, command> command_set;


typedef Map<int, robot_control> robot_control_set;






#endif //_STRUCTURES_HPP
