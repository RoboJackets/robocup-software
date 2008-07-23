#ifndef UDP_BROADCAST_H
#define UDP_BROADCAST_H

#include <stdint.h>
#include <string>
#include <gtkmm.h>

class UDP_Broadcast
{
public:
    /** TODO */
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
    
    UDP_Broadcast() throw (IOError);
    ~UDP_Broadcast();

    /** TODO */
    void set_destination(const std::string& host,
                         const uint16_t port) throw (IOError);

    /** TODO */
    void send(const std::string& buffer) throw (IOError);
    void send(const void* buffer, const size_t buflen) throw (IOError);    
protected:
    int sock;
};

#endif
