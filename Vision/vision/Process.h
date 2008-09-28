#ifndef _VISION__PROCESS_H_
#define _VISION__PROCESS_H_

#include <QMutex>
#include <list>

//#include "../Image.h"
#include "Colors.h"


class Camera_Thread;

namespace Vision
{
	class Colorseg;
	class Spanner;
	class Distortion;
	class Transform;
	
	class Process
	{
		public:
			Process();
			~Process();

			// The image may change size from one frame to the next as
			// different cameras are selected.
			//
			// The image format will always be an RGB32 format
			// (4 bytes per pixel: BB GG RR xx)
			//
			// This will be called from the camera thread, so it must
			// not do any GUI operations.
			// Use QApplication::postEvent() to make the GUI do things.
			void run();
			
			//void proc(const Image* img); //todo vision data
			
			void robot_radius(float r);
			float robot_radius() const;
			
			Colorseg *colorseg;
			Spanner *spanner[Num_Colors];
			//Processor *blue_id, *yellow_id;
			
			Distortion *distortion;
			Transform *ball_transform;
			Transform *robot_transform;

		protected:
			mutable QMutex mutex;
			
			float _robot_radius;
	};
}
;

#endif // _VISION__PROCESS_H_
