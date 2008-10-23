#ifndef _WORLD_MODEL_HPP_
#define _WORLD_MODEL_HPP_

#include <Vector>

#include <KalmanFilter.hpp>

/** World model module, which contains a 
 *  Kalman filter for each entity, as well
 *  as particle filtering for ball location
*/
class WorldModel {

public:
  
  //Typedefs:
  //A Position point
  typedef struct {
    //position
    double px;
    double py;
    double ptheta; 
  } p_point;

  //Position-Velocity-Acceleration point
  typedef struct {
    //position
    double px;
    double py;
    double ptheta; 
    //velocity
    double vx;
    double vy; 
    double vtheta;
    //acceleration
    double ax;
    double ay; 
    double atheta;
  } pva_point;

  //entity
  typedef struct {
    pva_point state;
    int id
  } entity; 
  
  //Vector of entities
  typedef Vector<entity> entityset;

  //Construction and configuration
  WorldModel();
  ~WorldModel();
  void clearWorld();

  //Vision Interface:
  //Adds a measurement to the world model, using an entity 
  // id to associate to known 
  void addMeasurement(entityset meas);

  //System Interface:
  entityset getAll();
  entityset getBlueTeam();
  entityset getYellowTeam();
  entity getBall();
  entity getEntity(int id);
  

private: 

};


#endif /* _WORLD_MODEL_HPP_ */
