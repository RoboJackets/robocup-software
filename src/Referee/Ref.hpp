#ifndef REF_HPP_
#define REF_HPP_

#include <Team.h>

#include <Packet/IO.hpp>
#include <Packet/Ref.hpp>
#include <Packet/VisionData.hpp>

#include <Geometry/Point2d.hpp>

#include "RefBox.h"

class Ref
{
	/// types ///
	public:
		
	
	/// methods ///
	public:
		Ref(Team t);
		~Ref() {}
		
		void refPacketHandler(const RefPacket* data);
		void visionHandler(const Packet::VisionData* vd);
		
		/** process the last input refdata */
		void proc();
		
	private:
		void printState();
		
	/// members ///
	private:
		/** team to process refbox commands for */
		Team _team;
		
		/** outgoing packet sender */
		Packet::Sender<Packet::Ref> _sender;
		
		/** last command count number */
		char _lastCounter;
		
		/** incoming packet from refbox */
		RefPacket _refPacket;

		/** outgoing packet, also holds our internal state */
		Packet::Ref _refState;

		/** location of ball at setup */
		Geometry::Point2d _setupBall;
};

#endif /* REF_HPP_ */
