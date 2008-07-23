#ifndef _LABJACK_U3_HPP_
#define _LABJACK_U3_HPP_

#include <stdint.h>

class USB_Device;

namespace Labjack
{
    class Stream_Channel
    {
    public:
        Stream_Channel()
        {
            pos = 0;
            neg = 31;
        }
        
        // Single-ended
        Stream_Channel(int p)
        {
            pos = p;
            neg = 31;
        }
        
        // Differential
        Stream_Channel(int p, int n)
        {
            pos = p;
            neg = n;
        }
        
        int pos, neg;
    };
    
    class U3
    {
    public:
        U3();
        ~U3();
        
        void open();
        void close();
        
        void command(uint8_t *cmd, unsigned int cmd_len, uint8_t *resp, unsigned int resp_len);
        
        // Throws an exception if the error is nonzero.
        void error(int err);
        
        // Performs a soft reset.
        void reset();
        
        // Reads a 32-byte block of calibration data into buf.
        void read_calibration(int block, uint8_t *buf);
        
        // Configures streaming.
        // ch points to num_channels Stream_Channels.
        // freq is the scan frequency in scans per second.
        void stream_config(Stream_Channel *ch, int num_channels, int freq);
        
        void stream_start();
        
        typedef void (*Stream_Callback)(int num_channels, uint16_t *scan, void *param);
        void stream_read(Stream_Callback cb, void *param = 0);
        
        void stream_stop();
        
        // How long to wait for a response to a command, in milliseconds.
        int timeout() const { return _timeout; }
        void timeout(int ms);
        
        bool hv() const { return _hv; }
        
        USB_Device *device() const { return _device; }
        
        // Returns a calibration constant as a double.
        double cal_constant(int block, int offset) const;
        
        const uint8_t *calibration() const { return _calibration; }
        
        double convert_hv(uint16_t sample, int channel);
        
    protected:
        // Sets the checksum for a normal command
        void normal_checksum(uint8_t *buf, unsigned int len);
        
        // Sets the checksum for an extended command
        void extended_checksum(uint8_t *buf, unsigned int len);
        
        // One's complement sum of all bytes in buf.
        uint16_t checksum(uint8_t *buf, unsigned int len);
        
        USB_Device *_device;
        int _timeout;
        bool _hv;
        
        // Number of channels to stream
        int _stream_channels;
        
        // Index of the last channel received in a scan
        int _stream_pos;
        
        // Stream scan data (only valid after stream_config)
        uint16_t *_stream_scan;
        
        // Calibration data.  Blocks 3 and 4 are only present in HV models.
        uint8_t _calibration[32 * 5];
        
        double _vref;
        double _hv_slope[4], _hv_offset[4];
    };
}

#endif // _LABJACK_U3_HPP_
