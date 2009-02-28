#include "RobotWM.hpp"
#include <vector>
#include <algorithm>
#include <Geometry/Point2d.hpp>

using namespace Modeling;
using namespace Geometry;
using namespace std;

RobotWM::RobotWM()
{
  _bufsize = 10;
  _angleBuf.assign(_bufsize, 0);
  _posBuf.assign(_bufsize, Point2d()); 
}

RobotWM::~RobotWM()
{


}

void RobotWM::init(bool isSelf)
{
  _isSelf = isSelf;

  //create the kalman filter
  if (_isSelf) 
    {
      //create kalman filter with control
      _kf = cvCreateKalman(3,3,3);
      //load B matrix
    }
  else 
    {
      //create kalman filter without control
      _kf = cvCreateKalman(3,3,0);
    }
  //Load A matrix
  //load C matrix
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
