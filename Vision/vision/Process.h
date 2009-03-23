#ifndef _VISION__PROCESS_H_
#define _VISION__PROCESS_H_

#include <QMutex>
#include <list>

#include <Network/Sender.hpp>

#include <Geometry/Point2d.hpp>
#include <Vision.hpp>

#include "Colors.h"
#include "../Image.h"
#include "identification/Identifier.hpp"

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
			void proc(const Image* img);

			void robot_radius(float r);
			float robot_radius() const;

			Colorseg* colorseg;
			Spanner* spanner[Num_Colors];

			Identifier* blueId;
			Identifier* yellowId;

			Distortion *distortion;
			Transform *ball_transform;
			Transform *robot_transform;

		private:
			//create a vision timestamp
			uint64_t timestamp() const;

		private:

			mutable QMutex mutex;

			float _robot_radius;

			Network::Sender _sender;
			Packet::Vision _visionPacket;

			unsigned int _procID;

			static unsigned int _nextID;
	};
}
;

#endif // _VISION__PROCESS_H_
