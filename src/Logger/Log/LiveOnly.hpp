#ifndef LIVELOG_HPP_
#define LIVELOG_HPP_

#include "Loggable.hpp"

namespace Log
{
	class LiveOnly : public Loggable
	{
		private:
			typedef struct PacketChain
			{
				PacketChain()
				{
					end = start = dispStart = chain.end();
				}
				
				/** the chain of packets */
				QLinkedList<LogPacket*> chain;
				
				/** iterator to end packet to display */
				QLinkedList<LogPacket*>::iterator end;
				
				/** start of packets to display */
				QLinkedList<LogPacket*>::iterator dispStart;
				
				/** start of all available packets */
				QLinkedList<LogPacket*>::iterator start;
			} PacketChain;
		
		public:
			LiveOnly();
			~LiveOnly();
			
			virtual void log(LogPacket* packet);
			virtual QList<LogPacket*> dispPackets();
			virtual QList<LogPacket*> lastDispPackets();
			
			virtual uint64_t latestDisplayTime() const;
			
			virtual void seek(uint64_t location);
			
		private:
			/** packet chains for packet types */
			mutable QMutex qMutex;		
			QMap< int, PacketChain* > queues;
	};
}

#endif /*LIVELOG_HPP_*/
