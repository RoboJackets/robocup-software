#ifndef SIMCMD_HPP_
#define SIMCMD_HPP_

#include "Ports.hpp"

#include <Geometry/Point2d.hpp>

namespace Packet
{
	class SimCmd
	{
		public:
			static const int Type = SoccSimPort;

			SimCmd() : setBallPos(false) {}
			
			/** if true the position will be set */
			bool setBallPos;
			
			/** new ball position, in world space */
			Geometry::Point2d ballPos;
			
			/** new ball velocity */
			Geometry::Point2d ballVel;

	} __attribute__((__packed__));
}

#endif /* SIMCMD_HPP_ */
