#include "RobotWM.hpp"
#include <vector>
#include <algorithm>
#include <Geometry/Point2d.hpp>

using namespace Modeling;
using namespace Geometry;
using namespace std;

RobotWM::RobotWM()
{
  
  
}

RobotWM::RobotWM(ConfigFile::RobotFilterCfg r, bool isSelf)
{
  //Fill position/angle buffer
  _bufsize = 10;
  _angleBuf.assign(_bufsize, 0);
  _posBuf.assign(_bufsize, Point2d()); 
  _isSelf = isSelf;
  _kfEnabled = r.kf_enable;

  //configure Kalman Filter
  if (_kfEnabled)
    {
      //create side-specific kalman configurations
      if (_isSelf)
	{
	  //Use pv model for state, full state output, v ctrl
	  _kf = cvCreateKalman(6,6,3);
	  //Fill in ctrl matrix with values
          cvSetIdentity(_kf->control_matrix, cvRealScalar(0));
	  cvmSet(_kf->control_matrix, 2, 1, 1);
	  cvmSet(_kf->control_matrix, 4, 2, 1);
	  cvmSet(_kf->control_matrix, 6, 3, 1);
	  //Fill in transition matrix
	  cvSetIdentity(_kf->transition_matrix, cvRealScalar(1));
	  cvmSet(_kf->transition_matrix, 2, 2, 0);
	  cvmSet(_kf->transition_matrix, 4, 4, 0);
	  cvmSet(_kf->transition_matrix, 6, 6, 0);
	}
      else 
	{
	  //use p model for opposing team robots
	  _kf = cvCreateKalman(3,3,0);
	  //simple state transition matrix
	  cvSetIdentity(_kf->transition_matrix, cvRealScalar(1));
	}
      //always have access to full state
      cvSetIdentity(_kf->measurement_matrix, cvRealScalar(1));
      //process covariance
      cvSetIdentity(_kf->process_noise_cov, cvRealScalar(1e-1));
      //measurement covariance
      cvSetIdentity(_kf->measurement_noise_cov, cvRealScalar(1e-1));
      //set post state to origin
      cvSetZero(_kf->state_post);
    }
}

RobotWM::~RobotWM()
{


}

void RobotWM::process()
{

  //TEMP: passthrough
  _pos = _measPos;
  _posAngle =_measAngle;
  _vel = Point2d();
  _velAngle = 0;

  //buffer positions
  rotate(_posBuf.begin(), _posBuf.end(), _posBuf.end());
  rotate(_angleBuf.begin(), _angleBuf.end(), _angleBuf.end());
  _posBuf[0] = _measPos;
  _angleBuf[0] = _measAngle;

  //calculate velocities
  _vel.x = 0; _vel.y = 0; _velAngle = 0;
  for (unsigned int i = 0; i<_bufsize-1; ++i)
    {
      _vel += (_posBuf[i]-_posBuf[i+1])/_bufsize;
      _velAngle += (_angleBuf[i]-_angleBuf[i+1])/_bufsize;
    }
  
  
}
