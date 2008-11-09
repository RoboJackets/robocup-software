#ifndef UDP_BROADCAST_HPP
#define UDP_BROADCAST_HPP

#include <stdint.h>
#include <string>
#include <gtkmm.h>

class UDP_Broadcast
{
	public:
	
		UDP_Broadcast() throw (IOError);
		~UDP_Broadcast();
    
		/** IOError Class */
		class IOError: public Glib::IOChannelError
		{
    		public:
    			IOError(const std::string& msg): Glib::IOChannelError(
    					Glib::IOChannelError::IO_ERROR, msg)
    			{
    			}
    			IOError(const std::string& msg, int err): Glib::IOChannelError(
    					Glib::IOChannelError::IO_ERROR, msg + ": " + strerror(err)) 
    			{
    			}
    			virtual ~IOError() throw() {};
		};
    


		/** Prototypes */
		void set_destination(const std::string& host, const uint16_t port) throw (IOError);
		void send(const std::string& buffer) throw (IOError);
		void send(const void* buffer, const size_t buflen) throw (IOError); 
    
	protected:
		int sock;
    
#endif /*UDP_BROADCAST_HPP_*/