#include "LiveOnly.hpp"

#include <QMutexLocker>
#include <sys/time.h>

#include "Packet.hpp"

using namespace Log;

LiveOnly::LiveOnly() :
	Loggable()
{
	struct timeval time;
	gettimeofday(&time, 0);
	_startTime = (uint64_t)time.tv_sec * 1000000 + time.tv_usec;
	
	_latestTimestamp = _startTime;
	_span = 0;
	
	//printf("start: %lld\n", _startTime);
}

LiveOnly::~LiveOnly()
{
	
}

QList<LogPacket*> LiveOnly::dispPackets()
{
	QMutexLocker ml(&qMutex);
	
	QList<LogPacket*> dPackets;
	
	//add all relevant packets to dPackets
	QList<PacketChain*> chains = queues.values();
	
	for (unsigned int i=0 ; i<chains.size() ; ++i)
	{
		PacketChain* chain = chains[i];
		
		//go from start to end
		if (chain->dispStart != chain->chain.end() && 
				chain->end != chain->chain.end())
		{
			//add all from start to end
			for (QLinkedList<LogPacket*>::Iterator iter = chain->dispStart ; 
				iter != chain->end+1 ; ++iter)
			{
				dPackets.append((*iter)->clone());
			}
		}
	}
	
	return dPackets;
}

QList<LogPacket*> LiveOnly::lastDispPackets()
{
	QMutexLocker ml(&qMutex);
	
	QList<LogPacket*> dPackets;
		
	//add all relevant packets to dPackets
	QList<PacketChain*> chains = queues.values();
	
	for (unsigned int i=0 ; i<chains.size() ; ++i)
	{
		PacketChain* chain = chains[i];
		
		if (chain->end != chain->chain.end())
		{
			dPackets.append((*chain->end)->clone());
		}
	}
	
	return dPackets;
}

uint64_t LiveOnly::latestDisplayTime() const
{
	QMutexLocker ml(&qMutex);
	
	//add all relevant packets to dPackets
	QList<PacketChain*> chains = queues.values();
	
	uint64_t time = _startTime;
	for (unsigned int i=0 ; i<chains.size() ; ++i)
	{
		if ((chains[i]->end != chains[i]->chain.end()) 
				&& (*chains[i]->end)->id() > time)
		{
			time = (*chains[i]->end)->id(); 
		}
	}
	
	return time;
}

void LiveOnly::seek(uint64_t location)
{
	uint64_t time = location + _startTime;
	//printf("seek:  %lld\n", time);
	
	QList<PacketChain*> chains = queues.values();
	
	for (unsigned int i=0 ; i<chains.size() ; ++i)
	{
		PacketChain* pc = chains[i];
		QLinkedList<LogPacket*>::iterator eIter = pc->end;
		QLinkedList<LogPacket*>::iterator sIter = pc->dispStart;
		
		if (eIter != pc->chain.end() && sIter != pc->chain.end())
		{
			//decrease until location
			if (time < (*eIter)->id())
			{
				//increase the iterator up to < _lastTimestamp
				while ((eIter != pc->chain.begin()) && (*eIter)->id() > time)
				{
					pc->end = eIter--;
				}
				
				if (_span == 0)
				{
					pc->dispStart = pc->end;
				}
				else
				{
					uint64_t startTime = time - _span;
					
					//want first packet (<=) before the startTime
					while ((sIter != pc->chain.begin()) && (*sIter)->id() >= startTime)
					{
						pc->dispStart = sIter--;
					}
				}
			}
			//increase until location
			else
			{
				//increase the iterator up to < _lastTimestamp
				while ((eIter != pc->chain.end()) && (*eIter)->id() < _latestTimestamp 
						&& (*eIter)->id() < time)
				{
					pc->end = eIter++;
				}
				
				if (_span == 0)
				{
					pc->dispStart = pc->end;
				}
				else
				{
					uint64_t startTime = time - _span;
					
					//want first packet (<=) before the startTime
					while ((sIter != pc->chain.end()) && (*sIter)->id() <= startTime)
					{
						pc->dispStart = sIter++;
					}
				}
			}
		}
	}
}

void LiveOnly::log(LogPacket* packet)
{	
	QMutexLocker ml(&qMutex);
	
	if (!queues.contains(packet->type()->id()))
	{
		queues.insert(packet->type()->id(), new PacketChain());
	}
	PacketChain* pc = queues[packet->type()->id()];
	
	//if timestamp is greater, then increase it
	if (packet->id() > _latestTimestamp)
	{
		_latestTimestamp = packet->id();
	}
	
	//add the packet to the chain
	pc->chain.append(packet);
	
	if (!_paused)
	{
		QLinkedList<LogPacket*>::iterator eIter = pc->end;
		QLinkedList<LogPacket*>::iterator dIter = pc->dispStart;
		QLinkedList<LogPacket*>::iterator sIter = pc->start;
		
		//start at beginning if nothing current
		if (sIter == pc->chain.end())
		{
			sIter = pc->chain.begin();
		}
		else
		{
			sIter++;
		}
		
		if (eIter == pc->chain.end())
		{
			eIter = pc->chain.begin();
		}
		else
		{
			eIter++;
		}
		
		if (dIter == pc->chain.end())
		{
			dIter = pc->chain.begin();
		}
		else
		{
			dIter++;
		}
			
		//increase the iterator up to < _lastTimestamp
		while ((eIter != pc->chain.end()) && (*eIter)->id() < _latestTimestamp)
		{
			pc->end = eIter++;
		}
		
		uint64_t startTime = _latestTimestamp - _span;
		//want first packet (<=) before the startTime
		
		//if (_span == 0)
		{
			//pc->start = pc->dispStart = pc->end;
		}
		//else
		{
			while ((dIter != pc->chain.end()) && (*dIter)->id() < startTime)
			{
				pc->dispStart = dIter++;
			}
			
			while ((sIter != pc->chain.end()) && (*sIter)->id() < startTime)
			{
				if (pc->start != pc->chain.end())
				{
					//printf("erasing: %lld\n", (*pc->start)->id());
					delete *pc->start;
					pc->chain.erase(pc->start);
				}
				pc->start = sIter++;
			}
		}
	}
}
