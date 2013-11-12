#pragma once

#include <stdint.h>
#include <string>
#include <gtkmm.h>

#define SocketCreationError -1
#define SendFailure -1
#define SetSocketOptionError -1
#define ConnectionError -1

class UDP_Broadcast
{
	public:
		
		/** IO Error Handler */
		class IOError: public Glib::IOChannelError
		{
			public:
				/** constructors */
				IOError(const std::string &msg): Glib::IOChannelError(Glib::IOChannelError::IO_ERROR, msg) {}
				IOError(const std::string &msg, int err): Glib::IOChannelError(Glib::IOChannelError::IO_ERROR, msg + ": " + strerror(err)) {}
				
				/** destructors */
				virtual ~IOError() throw() {};
		};
		
		/// methods ///
    
		UDP_Broadcast() throw (IOError);
		~UDP_Broadcast();
 
		/** This function sets the host address and port number */
		void setDestination(const std::string &host, const uint16_t port) throw (IOError);
		
		/** This function sends a packet */
		void sendPacket(const std::string &buffer) throw (IOError);
		
		/** This function sends a packet */
		void sendPacket(const void *buffer, const size_t buflen) throw (IOError);
		
		/// members ///
	protected:
		/** sock used for socket creation */
		int sock;
};
