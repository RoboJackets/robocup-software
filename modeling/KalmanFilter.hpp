#ifndef _KALMAN_FILTER_HPP_
#define _KALMAN_FILTER_HPP_

//Should be removed for non-debug versions
#include "opencv/cv.h"
#include <time.h>
#include <string>

#include <WorldModel.hpp>

/** This is a simple wrapper of the OpenCV 
 *  Allows for multiple configurations
 *  including robots or balls, with different 
 *  levels of sophistication. */
class KalmanFilter {

public:
  //Enum for configuration type
  enum filterConfig{ROBOT, BALL};


    
  //Constructor
  KalmanFilter(filterConfig config, int id, string mat_file);
  ~KalmanFilter();
  
  //use functions

  //Returns the most recent corrected state
  pva_point getCurState();

  //Returns an interpolated prediction 
  //This value is adjusted to account for time since last 
  //filter update
  pva_point getInterpState();

  //Adds a new measurement
  // Returns the corrected pva state
  pva_point update(p_point meas);

  int getID();
  void setID(int i);
  
private: 

  //Members
  CvKalman* kalman;
  clock_t updateTimestamp;
  filterConfig config;
  int id;

  //Methods
  static pva_point state2pva(CvMat* m);
  static void pva2state(pva_point p);

};


#endif /* _KALMAN_FILTER_HPP_ */
