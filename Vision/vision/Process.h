#ifndef _VISION__PROCESS_H_
#define _VISION__PROCESS_H_

#include <QMutex>
#include <list>

#include <SerDes/Sender.hpp>

#include <Geometry/Point2d.hpp>
#include <Vision.hpp>

#include "../Image.h"
#include "Colors.h"
#include "../VisionData.hpp"

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
			
			// The image format will always be an RGB32 format
			// (4 bytes per pixel: BB GG RR xx)
			void proc(const Image* img, VisionData& data);
			
			void robot_radius(float r);
			float robot_radius() const;
			
			Colorseg* colorseg;
			Spanner* spanner[Num_Colors];
			//Processor *blue_id, *yellow_id;
			
			Distortion *distortion;
			Transform *ball_transform;
			Transform *robot_transform;

		private:
			uint64_t timestamp() const;
			
		private:
			
			mutable QMutex mutex;
			
			float _robot_radius;
			
			Sender _sender;
			Packet::Vision _visionPacket;
			
			unsigned int _procID;
			
			static unsigned int _nextID;
	};
}
;

#endif // _VISION__PROCESS_H_
