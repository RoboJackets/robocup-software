#include <stdexcept>
#include <boost/format.hpp>

#include "Labjack_U3.hpp"
#include "USB_Device.hpp"
#include "errno_exception.hpp"

using namespace std;
using namespace boost;

Labjack::U3::U3()
{
    _device = 0;
    _timeout = 500;
    _hv = false;
    _stream_channels = 0;
    _stream_scan = 0;
}

Labjack::U3::~U3()
{
    if (_device)
    {
        delete _device;
    }
    
    if (_stream_scan)
    {
        delete[] _stream_scan;
    }
}

void Labjack::U3::open()
{
    if (_device)
    {
        throw runtime_error("Device already open");
    }
    
    _device = USB_Device::find_first(0x0cd5, 0x003);
    if (!_device)
    {
        throw runtime_error("USB device not found");
    }
    
    errno_assert(_device->open());
    errno_assert(_device->set_default());

    // It may take a while for the U3 to start responding to commands.
    // Try to read non-volatile configuration.
    uint8_t cmd[26] =
    {
        0,                      // Checksum8
        0xf8, 0x0a, 0x08        // Command and length
    };
    
    int old_timeout = timeout();
    timeout(10);
    
    uint8_t resp[38];
    int i;
    for (i = 0; i < 10; ++i)
    {
        try
        {
            command(cmd, sizeof(cmd), resp, sizeof(resp));
            
#if 0
            printf("Local ID:           %d\n", resp[8]);
            printf("Firmware version:   %d.%d\n", resp[10], resp[9]);
            printf("Bootloader version: %d.%d\n", resp[12], resp[11]);
            printf("Hardware version:   %d.%d\n", resp[14], resp[13]);
            printf("FIOAnalog    %02x\n", resp[23]);
            printf("FIODirection %02x\n", resp[24]);
#endif
            
            break;
        } catch (exception &ex)
        {
        }
    }
    
    if (i == 10)
    {
        close();
        throw runtime_error("Timed out reading configuration");
    }
    
    // Restore the timeout value
    timeout(old_timeout);

    _hv = (resp[37] & 0x10) != 0;

    // Read calibration data
    int n = hv() ? 5 : 3;
    for (i = 0; i < n; ++i)
    {
        read_calibration(i, _calibration + i * 32);
    }

    _vref = cal_constant(2, 8);
    if (_hv)
    {
        for (i = 0; i < 4; ++i)
        {
            _hv_slope[i] = cal_constant(3, i * 8);
            _hv_offset[i] = cal_constant(4, i * 8);
        }
    }
}

void Labjack::U3::close()
{
    if (!_device)
    {
        throw runtime_error("Device not open");
    }
    
    delete _device;
    _device = 0;
}

void Labjack::U3::command(uint8_t *cmd, unsigned int cmd_len, uint8_t *resp, unsigned int resp_len)
{
    if (!_device)
    {
        throw runtime_error("Device not open");
    }
    
    if ((cmd[1] & 0xf0) == 0xf0)
    {
        extended_checksum(cmd, cmd_len);
    } else {
        normal_checksum(cmd, cmd_len);
    }
    
    errno_assert(_device->bulk_write(1, cmd, cmd_len));
    errno_assert(_device->bulk_read(2, resp, resp_len, _timeout));
}

void Labjack::U3::error(int err)
{
    if (err)
    {
        //FIXME - Description
        throw runtime_error(str(format("LJ error %1%") % err));
    }
}

void Labjack::U3::reset()
{
    uint8_t cmd[4] = {0, 0x99, 0, 0};
    uint8_t resp[4];
    command(cmd, sizeof(cmd), resp, sizeof(resp));
    error(resp[3]);
}

void Labjack::U3::read_calibration(int block, uint8_t *buf)
{
    uint8_t cmd[8] = {0x00, 0xf8, 0x01, 0x2d, 0x00, 0x00, 0x00, block};
    uint8_t resp[40];
    command(cmd, sizeof(cmd), resp, sizeof(resp));
    error(cmd[6]);
    memcpy(buf, resp + 8, 32);
}

void Labjack::U3::stream_config(Stream_Channel *ch, int num_channels, int freq)
{
    //FIXME - More flexibility here
    uint8_t config = 3;
    uint32_t clock = 4000000;
    uint16_t div = clock / freq;
    uint8_t cmd[12 + num_channels * 2];
    
    cmd[0] = 0;                     // Checksum8
    cmd[1] = 0xf8;                  // Normal
    cmd[2] = num_channels + 3;      // Length
    cmd[3] = 0x11;                  // Extended command
    cmd[4] = 0;                     // Checksum16 LSB
    cmd[5] = 0;                     // Checksum16 MSB
    cmd[6] = num_channels;          // Number of channels
    cmd[7] = 25;                    // Samples per packet (1-25)
    cmd[8] = 0;                     // Reserved
    cmd[9] = config;                // ScanConfig
    cmd[10] = div;                  // Clock divisor
    cmd[11] = div >> 8;
    
    // Channel numbers
    for (int i = 0; i < num_channels; ++i)
    {
        cmd[12 + i * 2] = ch[i].pos;
        cmd[13 + i * 2] = ch[i].neg;
    }
    
    uint8_t resp[8];
    command(cmd, sizeof(cmd), resp, 8);
    error(resp[6]);
    
    // Allocate a new scan buffer
    if (_stream_channels != num_channels)
    {
        if (_stream_scan)
        {
            delete[] _stream_scan;
        }
        
        _stream_scan = new uint16_t[num_channels];
        _stream_channels = num_channels;
    }
}

void Labjack::U3::stream_start()
{
    uint8_t cmd[2] = {0xa8, 0xa8};
    uint8_t resp[4];
    command(cmd, sizeof(cmd), resp, 4);
    error(resp[2]);
    
    _stream_pos = 0;
}

void Labjack::U3::stream_read(Stream_Callback cb, void *param)
{
    uint8_t buf[14 + 25 * 2];
    errno_assert(_device->bulk_read(3, buf, sizeof(buf), _timeout));
    if (buf[11])
        printf("err %d\n", buf[11]);
    //error(buf[11]);
    
    for (int i = 0; i < 25; ++i)
    {
        uint16_t sample = buf[12 + i * 2] | buf[13 + i * 2] * 256;
        
        _stream_scan[_stream_pos++] = sample;
        
        if (_stream_pos == _stream_channels)
        {
            _stream_pos = 0;
            
            if (cb)
            {
                cb(_stream_channels, _stream_scan, param);
            }
        }
    }
}

void Labjack::U3::stream_stop()
{
    uint8_t cmd[2] = {0xb0, 0xb0};
    uint8_t resp[4];
    command(cmd, sizeof(cmd), resp, 4);
    error(resp[2]);
}

void Labjack::U3::timeout(int ms)
{
    _timeout = ms;
}

double Labjack::U3::cal_constant(int block, int offset) const
{
    uint32_t f = *(const uint32_t *)(_calibration + block * 32 + offset);
    int32_t i = *(const uint32_t *)(_calibration + block * 32 + offset + 4);
    
    return (double)i + (double)f / 4294967296.0;
}

double Labjack::U3::convert_hv(uint16_t sample, int channel)
{
    return _hv_slope[channel] * sample + _hv_offset[channel];
}

uint16_t Labjack::U3::checksum(uint8_t *buf, unsigned int len)
{
    uint32_t sum = 0;
    for (unsigned int i = 0; i < len; ++i)
    {
        sum += buf[i];
        
        // End-around carry
        if (sum & 0x10000)
        {
            sum &= 0xffff;
            sum++;
        }
    }
    
    return sum;
}

void Labjack::U3::normal_checksum(uint8_t *buf, unsigned int len)
{
    uint16_t sum = checksum(buf + 1, len - 1);
    
    // Checksum goes in the first byte
    sum = (sum >> 8) + (sum & 0xff);
    buf[0] = (sum >> 8) + (sum & 0xff);
}

void Labjack::U3::extended_checksum(uint8_t *buf, unsigned int len)
{
    // 16-bit checksum
    uint32_t sum = checksum(buf + 6, len - 6);
    buf[4] = sum & 0xff;
    buf[5] = sum >> 8;
    
    // 8-bit checksum
    normal_checksum(buf, 6);
}
