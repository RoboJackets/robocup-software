#include "RobotWM.hpp"
#include <vector>
#include <algorithm>
#include <Geometry/Point2d.hpp>

using namespace Modeling;
using namespace Geometry;
using namespace std;

RobotWM::RobotWM()
{
    _shell = 0;
    _measAngle = 0;
    _valid = false;
    _cmdAngle = 0;
    _cmdValid = false;
    _cmd_scale_factor = 0;
    _cmd_scale_factor_angle = 0;
    _velAngle = 0;
    _posAngle = 0;
    _isSelf = false;
    _kfEnabled = false;
}

RobotWM::RobotWM(ConfigFile::RobotFilterCfg r, bool isSelf)
{
  //Fill position/angle buffer
  _angleBuf.resize(10);
  _posBuf.resize(10);
//  _angleBuf.assign(_bufsize, 0);
//  _posBuf.assign(_bufsize, Point2d()); 
  _isSelf = isSelf;

  //FIXME Get values from config file
  _cmd_scale_factor = 1;  //FIXME check value
  _cmd_scale_factor_angle = 1; //FIXME check value

  if (_isSelf)
    {
      _kfEnabled = r.kf_self_enable;
    }
  else 
    {
      _kfEnabled = r.kf_opp_enable;
    }

  //configure Kalman Filter
  if (_kfEnabled)
    {
      //create side-specific kalman configurations
      if (_isSelf)
	{
	  //Use pv model for state, full state output, v ctrl
	  _kf = cvCreateKalman(6,3,3);
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
	  //Create measurement matrix
	  cvSetZero(_kf->measurement_matrix);
	  cvmSet(_kf->measurement_matrix, 0, 0, 1);
	  cvmSet(_kf->measurement_matrix, 1, 2, 1);
	  cvmSet(_kf->measurement_matrix, 2, 4, 1);
	}
      else 
	{
	  //use p model for opposing team robots
	  _kf = cvCreateKalman(3,3,0);
	  //simple state transition matrix
	  cvSetIdentity(_kf->transition_matrix, cvRealScalar(1));
	  //get full state (only position)
	  cvSetIdentity(_kf->measurement_matrix, cvRealScalar(1));
	}
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
  if (_kfEnabled)
    {
      if (_isSelf)
	{
	  selfProcess();
	}
      else
	{
	  oppProcess();
	}
    }
  else
    {
      //passthrough
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
      for (unsigned int i = 0; i<_posBuf.size()-1; ++i)
	{
	  _vel += (_posBuf[i]-_posBuf[i+1]);
	}
    _vel /= _posBuf.size();
      for (unsigned int i = 0; i<_angleBuf.size()-1; ++i)
      {
      _velAngle += (_angleBuf[i]-_angleBuf[i+1]);
      }
      _velAngle /= _angleBuf.size();
    }
}

void RobotWM::selfProcess() 
{

  //create control input
  CvMat* ctrl = cvCreateMat(3,1,CV_32FC1);
  cvmSet(ctrl, 0, 0, _cmdVel.x*_cmd_scale_factor); //vx
  cvmSet(ctrl, 1, 0, _cmdVel.y*_cmd_scale_factor); //vy
  cvmSet(ctrl, 2, 0, _cmdAngle*_cmd_scale_factor_angle); //va
  
  //predict new state
  cvKalmanPredict(_kf, ctrl);

  //create measurement vector
  CvMat* z = cvCreateMat(3,1,CV_32FC1);
  cvmSet(z,0,0, _measPos.x); //px
  cvmSet(z,1,0, _measPos.y); //py
  cvmSet(z,2,0, _measAngle); //pa
  
  //correct new state
  const CvMat* state = cvKalmanCorrect(_kf, z);
  
  //copy out the state variables
  _pos.x = cvmGet(state, 0,0);
  _vel.x = cvmGet(state, 1,0);
  _pos.y = cvmGet(state, 2,0);
  _vel.y = cvmGet(state, 3,0);
  _posAngle = cvmGet(state, 4,0);
  _velAngle = cvmGet(state, 5,0);

  //cleanup
  cvReleaseMat(&ctrl);
  cvReleaseMat(&z);

}

void RobotWM::oppProcess() 
{

  //predict new state
  cvKalmanPredict(_kf);

  //create measurement vector
  CvMat* z = cvCreateMat(3,1,CV_32FC1);
  cvmSet(z,0,0, _measPos.x); //px
  cvmSet(z,1,0, _measPos.y); //py
  cvmSet(z,2,0, _measAngle); //pa
  
  //correct new state
  const CvMat* state = cvKalmanCorrect(_kf, z);
  
  //copy out the state variables
  _pos.x = cvmGet(state, 0,0);
  _pos.y = cvmGet(state, 1,0);
  _posAngle = cvmGet(state, 2,0);

  //cleanup
  cvReleaseMat(&z);

  //buffer positions to calculate velocity
  _vel = Point2d();
  _velAngle = 0;
  
  //buffer positions
  rotate(_posBuf.begin(), _posBuf.end(), _posBuf.end());
  rotate(_angleBuf.begin(), _angleBuf.end(), _angleBuf.end());
  _posBuf[0] = _pos;
  _angleBuf[0] = _posAngle;
  
  //calculate velocities
  _vel.x = 0; _vel.y = 0; _velAngle = 0;
    for (unsigned int i = 0; i<_posBuf.size()-1; ++i)
    {
        _vel += _posBuf[i]-_posBuf[i+1];
    }
    _vel /= _posBuf.size();
    for (unsigned int i = 0; i<_angleBuf.size()-1; ++i)
    {
        _velAngle += _angleBuf[i]-_angleBuf[i+1];
    }
    _velAngle /= _angleBuf.size();
}

