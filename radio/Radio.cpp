#include <stdexcept>
#include <boost/format.hpp>

#include "Radio.hpp"
#include "USB_Device.hpp"
#include "cc1101.h"
#include "radio_config.h"

using namespace std;
using namespace boost;

Radio::Radio(int n)
{
    vector<USB_Device *> devs;
    USB_Device::find_all(devs, 0x3141, 0x0004);
    
    if (devs.empty())
    {
        throw runtime_error(str(format("Radio::Radio(%1%): No base stations found") % n));
    }
    
    _device = devs[n];
    if (!_device->open())
    {
        throw runtime_error(str(format("Radio::Radio(%1%): can't open base station") % n));
    }
    
    if (!_device->set_default())
    {
        throw runtime_error(str(format("Radio::Radio(%1%): can't set default configuration on base station") % n));
    }
    
    configure();
}

Radio::~Radio()
{
}

void Radio::write_packet(const void *data, unsigned int size)
{
    if (!_device->bulk_write(1, data, size))
    {
        throw runtime_error("Radio::write_packet(): bulk_write failed");
    }
}

bool Radio::read_packet(void *data, unsigned int size, int timeout)
{
    return _device->bulk_read(2, data, size, timeout);
}

void Radio::command(uint8_t cmd)
{
    if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 2, 0, cmd))
    {
        throw runtime_error("Radio::command control write failed");
    }
}

void Radio::write(uint8_t reg, uint8_t value)
{
    if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 1, value, reg))
    {
        throw runtime_error("Radio::write control write failed");
    }
}

uint8_t Radio::read(uint8_t reg)
{
    uint8_t value = 0;
    if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 3, 0, reg, &value, 1))
    {
        throw runtime_error("Radio::read control write failed");
    }
    
    return value;
}

void Radio::reverse_size(int size)
{
    if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 5, Reverse_Size, 0))
    {
        throw runtime_error("Radio::reverse_size control write failed");
    }
}

void Radio::configure()
{
    auto_calibrate(false);
    
    command(SIDLE);
    command(SFTX);
    command(SFRX);
    
    // Write configuration.
    // This is mainly for frequency, bit rate, and packetization.
    for (unsigned int i = 0; i < sizeof(cc1101_regs); i += 2)
    {
        write(cc1101_regs[i], cc1101_regs[i + 1]);
    }

    //FIXME - Should be in config table
    write(PKTCTRL1, 0x4c);
    write(PKTCTRL0, 0x04);
    
    reverse_size(Reverse_Size);

    auto_calibrate(true);
}

void Radio::auto_calibrate(bool enable)
{
    int flag = enable ? 1 : 0;
    assert(_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 4, flag, 0));
}
