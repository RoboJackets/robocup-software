#pragma once

#include <QTime>

#include <sys/poll.h>
#include <vector>

#include "Receiver.hpp"

namespace Network
{
    /** Receives different types of packets and calls the appropriate handlers (packet or timeout)
     *  To deal with the information */
    class PacketReceiver
    {
        private:
            /** base class for all packet receivers
             *  Necessary because the actual receiver is templated and cannot be stored in a vector
             *  Because each packet is a different type and uses a different receiver template */
            class _io
            {
                public:
                    _io(uint16_t port, unsigned int msec) :
                        _port(port), _msecTimeout(msec)
                    {
                        lastRecvTime.start();
                    }

                    virtual ~_io() {};

                    virtual int fd() = 0;

                    /** read a packet using the templated receiver */
                    virtual void read() = 0;

                    /** call the timeout function */
                    virtual void timeout() = 0;

                    /** port used for the receiver */
                    uint16_t port() const { return _port; }

                    void timeoutCheck()
                    {
                        if (_msecTimeout && (unsigned int)lastRecvTime.elapsed() > _msecTimeout)
                        {
                            timeout();
                        }
                    };

                    /** maintains the last time a packet was received */
                    QTime lastRecvTime;

                protected:
                    /** the port number for this receiver
                     *  Each packet type has a distinct port number */
                    uint16_t _port;

                    const unsigned int _msecTimeout;
            };
            
            /** implementation of the base _io class for each particular packet type
             *  Reads data of that type and also handles the calls to the packet/timeout handler */
            template <typename T>
            class _recv : public PacketReceiver::_io, public Receiver
            {
                public:
                    _recv(const char* addr, uint16_t port, void (*packetHandler)(const T*), unsigned int msec):
                        _io(port, msec),
                        Receiver(addr, port)
                    {
                        _packetHandler = packetHandler;
                    }

                    virtual int fd()
                    {
                        return fileDescriptor();
                    }

                    virtual void read()
                    {
                        T packet;
                        receive(packet);
                        lastRecvTime.restart();

                        if (_packetHandler)
                        {
                            _packetHandler(&packet);
                        }
                    }

                    virtual void timeout()
                    {
                        //on timeout, return a null pointer to the handler
                        if (_packetHandler)
                        {
                            _packetHandler(0);
                        }
                    }

                    /** handler for the received packet */
                    void (*_packetHandler)(const T*);
            };

            template <typename T, typename C>
            class _mem_recv : public PacketReceiver::_io, public Receiver
            {
                public:
                    _mem_recv(const char* addr, uint16_t port, C *obj, void (C::*handler)(const T*), unsigned int msec):
                        _io(port, msec), Receiver(addr, port)
                    {
                        _obj = obj;
                        _handler = handler;
                    }

                    virtual int fd()
                    {
                        return fileDescriptor();
                    }

                    virtual void read()
                    {
                        T packet;
                        receive(packet);
                        lastRecvTime.restart();

                        if (_handler && _obj)
                        {
                            (_obj->*_handler)(&packet);
                        }
                    }

                    virtual void timeout()
                    {
                        //on timeout, return a null pointer to the handler
                        if (_handler)
                        {
                            (_obj->*_handler)(0);
                        }
                    }

                    C *_obj;
                    void (C::*_handler)(const T*);
            };

        public:
            PacketReceiver() {}
            ~PacketReceiver()
            {
                for (unsigned int i = 0; i < _receivers.size(); ++i)
                {
                    delete _receivers[i];
                }
            }

            /** receive ALL the next available packets.
             *  This will call the handlers registered for each packet
             *  If there was a packet of the type the packetHandler is called,
             *  If there was no packet and the timeout interval has passed, 
             *  timeout handler is called */
            void receive(bool block)
            {
                //check for possible packets
                //return value 0 = timeout on call
                
                if (block)
                {
                    //wait for first packet blocking
                    if (poll(&_pollfds[0], _pollfds.size(), -1 ) >= 1) //timeout is in ms
                    {
                        for (unsigned int i=0 ; i<_pollfds.size() ; ++i)
                        {
                            if (_pollfds[i].revents & POLLIN)
                            {
                                //read the packet
                                _receivers[i]->read();
                            }
                        }
                        //get any other queued packets using non-blocking
                        while (poll(&_pollfds[0], _pollfds.size(), 0) >= 1) //timeout is in ms
                        {
                            for (unsigned int i=0 ; i<_pollfds.size() ; ++i)
                            {
                                if (_pollfds[i].revents & POLLIN)
                                {
                                    //read the packet
                                    _receivers[i]->read();
                                }
                            }
                        }
                    }
                }
                else
                {               
                    while (poll(&_pollfds[0], _pollfds.size(), 0) >= 1) //timeout is in ms
                    {
                        for (unsigned int i=0 ; i<_pollfds.size() ; ++i)
                        {
                            if (_pollfds[i].revents & POLLIN)
                            {
                                //read the packet
                                _receivers[i]->read();
                            }
                        }
                    }
                }

                //check each receiver for a timeout
                for (unsigned int i=0 ; i<_receivers.size() ; ++i)
                {
                    _receivers[i]->timeoutCheck();
                }
            }

            template <typename T>
            void addType(const char* addr, uint16_t port, void (*packetHandler)(const T*) = 0, unsigned int msec = 0)
            {
                addReceiver(new _recv<T>(addr, port, packetHandler, msec));
            }

            // Member function version of the above.
            template <typename T, typename C>
            void addType(const char* addr, uint16_t port, C *obj, void (C::*handler)(const T*) = 0, unsigned int msec = 0)
            {
                addReceiver(new _mem_recv<T, C>(addr, port, obj, handler, msec));
            }

        private:
            /** add the receiver to the list to poll */
            void addReceiver(_io *r)
            {
                // If a handler for this type exists, replace the old handler.
                for (unsigned int i=0 ; i<_receivers.size() ; ++i)
                {
                    if (_receivers[i]->port() == r->port())
                    {
                        delete _receivers[i];
                        _receivers[i] = r;
                        return;
                    }
                }

                //add the receiver to the vector
                _receivers.push_back(r);

                //add the pollfd struct to a separate vector
                //used for the poll() call
                struct pollfd pfd;
                pfd.fd = _receivers.back()->fd();
                pfd.events = POLLIN;
                pfd.revents = 0;
                _pollfds.push_back(pfd);
            }

            std::vector< _io* > _receivers;
            std::vector<struct pollfd> _pollfds;
    };
}
