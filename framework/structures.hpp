#ifndef _STRUCTURES_HPP
#define _STRUCTURES_HPP

#include <map>
using namespace std;

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
  map<int, robot_p> robots;
  map<int, ball_p> balls;
} state_p;

typedef struct {
  map<int, robot_pv> robots;
  map<int, ball_pv> balls;
} state_pv;

typedef struct {
  double vx; 
  double vy;
  double vtheta;
} robot_control;

//TODO: Replace with real command structure
typedef int command;
typedef map<int, command> command_set;


typedef map<int, robot_control> robot_control_set;






#endif //_STRUCTURES_HPP
