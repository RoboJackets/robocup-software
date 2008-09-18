#ifndef SKILLSTATUS_HPP_
#define SKILLSTATUS_HPP_

#include "Ports.hpp"

namespace Packet
{
	class SkillStatus
	{
		public:
			static const int Type = SkillStatusPort;
			
			typedef enum
			{
				/** no skill selected or not running */
				None,
				/** skill is currently being performed */
				Running,
				/** skill is done being performed */
				Done,
				/** skill cannot be completed */
				Failed
			} Status;
			
            class Robot
            {
            public:
                Robot(): status(None), sequence(-1) {}
                
                Status status;
                int sequence;
            } __attribute__((__packed__));
            
			Robot robots[5];
	} __attribute__((__packed__));
}

#endif /* SKILLSTATUS_HPP_ */
