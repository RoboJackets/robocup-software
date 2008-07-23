#ifndef LOGGABLE_HPP_
#define LOGGABLE_HPP_

#include "Packet.hpp"

#include <QLinkedList>
#include <QMutex>

namespace Log
{
	class Loggable
	{
		public:
			Loggable() { _paused = false; }
			
			virtual void log(LogPacket* packet) = 0;
			virtual QList<LogPacket*> dispPackets() = 0;
			virtual QList<LogPacket*> lastDispPackets() = 0;
			
			void span(uint64_t span) { _span = span; }
			
			uint64_t startTime() const { return _startTime; }
			uint64_t latestReceivedTime() const { return _latestTimestamp; }
			
			/** return the last timestamp to display, default = latestTimestamp */
			virtual uint64_t latestDisplayTime() const { return _latestTimestamp; }

			void pause(bool p = true) { _paused = p; }
			void play() { pause(false); }
			
			/** seek to a certain display packet time */
			virtual void seek(uint64_t location) = 0;
			
		protected:
			/** start time of the loggable packets */
			uint64_t _startTime;
			
			/** latest timestamp received */
			uint64_t _latestTimestamp;
			
			/** time from lastTimestamp to display packets */
			uint64_t _span;
			
			/** true if playback is paused */
			bool _paused;
	};
}

#endif /*LOGGABLE_HPP_*/
