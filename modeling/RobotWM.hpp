#ifndef ROBOT_WM_HPP_
#define ROBOT_WM_HPP_

#include <vector>
#include <Geometry/Point2d.hpp>
#include <opencv/cv.h>

/** Wrapper for individual */
namespace Modeling
{
    class RobotWM 
	{
		public:
			RobotWM();
			~RobotWM();
			void process();
			void init(bool isSelf);
			
		public:
			//////Input data
			// From vision
			char _shell;
			Geometry::Point2d _measPos;
			float _measAngle;
			bool _valid;
            
			// From commands
			Geometry::Point2d _cmdPos;
			Geometry::Point2d _cmdVel;
			float _cmdAngle;   
			bool _cmdValid;

			//output data
			Geometry::Point2d _vel;
			float  _velAngle;
			Geometry::Point2d _pos;
			float _posAngle;
			
		private:
			std::vector<Geometry::Point2d> _posBuf;
			std::vector<float> _angleBuf;
			unsigned int _bufsize;
			bool _isSelf;
			CvKalman * _kf;
			
	  
	};
}

#endif /* ROBOT_WM_HPP_ */
