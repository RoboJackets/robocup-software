#include "USB_Device.hpp"

bool USB_Device::inited = false;

USB_Device *USB_Device::find_first(uint16_t vendor, uint16_t product)
{
	init();
	
	for (struct usb_bus *bus = usb_busses; bus; bus = bus->next)
	{
		for (struct usb_device *dev = bus->devices; dev; dev = dev->next)
		{
			if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
			{
				return new USB_Device(dev);
			}
		}
	}
	
	return 0;
}

void USB_Device::find_all(std::vector<USB_Device *> &devs, uint16_t vendor, uint16_t product)
{
	init();

	for (struct usb_bus *bus = usb_busses; bus; bus = bus->next)
	{
		for (struct usb_device *dev = bus->devices; dev; dev = dev->next)
		{
			if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
			{
				devs.push_back(new USB_Device(dev));
			}
		}
	}
}

void USB_Device::init()
{
	if (!inited)
	{
		usb_init();
		inited = true;
	}
	
	usb_find_busses();
	usb_find_devices();
}

USB_Device::USB_Device(struct usb_device *dev)
{
	_dev = dev;
	_handle = 0;
}

USB_Device::~USB_Device()
{
	if (_handle)
	{
		usb_close(_handle);
		_handle = 0;
	}
}

bool USB_Device::open()
{
	if (!_dev)
	{
		return false;
	}
	
	if (_handle)
	{
		return true;
	}
	
	_handle = usb_open(_dev);
	
	return _handle != 0;
}

bool USB_Device::set_configuration(int n)
{
	return usb_set_configuration(_handle, n) == 0;
}

bool USB_Device::set_altinterface(int n)
{
	return usb_set_altinterface(_handle, n) == 0;
}

bool USB_Device::claim_interface(int n)
{
	return usb_claim_interface(_handle, n) == 0;
}

bool USB_Device::release_interface(int n)
{
	return usb_release_interface(_handle, n) == 0;
}

bool USB_Device::set_default()
{
	if (!_dev || !_dev->config || !_dev->config->interface || !_dev->config->interface->altsetting)
	{
		return false;
	}

	int config = _dev->config->bConfigurationValue;
	int interface = _dev->config->interface->altsetting->bInterfaceNumber;
	int alt = _dev->config->interface->altsetting->bAlternateSetting;
	
	return set_configuration(config) &&
		claim_interface(interface) &&
		set_altinterface(alt);
}

bool USB_Device::control(uint8_t type, uint8_t request, uint16_t value, uint16_t index, void *data, int size, int timeout, int *bytes_done)
{
	int ret = usb_control_msg(_handle, type, request, value, index, (char *)data, size, timeout);
	if (bytes_done)
	{
		if (ret >= 0)
		{
			*bytes_done = ret;
		} else {
			*bytes_done = 0;
		}
	}
	
	return ret == size;
}

bool USB_Device::bulk_write(int endpoint, const void *data, unsigned int size, int timeout, unsigned int *bytes_done)
{
	int ret = usb_bulk_write(_handle, endpoint, (char *)data, size, timeout);
	if (bytes_done)
	{
		if (ret >= 0)
		{
			*bytes_done = ret;
		} else {
			*bytes_done = 0;
		}
	}
	
	return ret == (int)size;
}

bool USB_Device::bulk_read(int endpoint, void *data, unsigned int size, int timeout, unsigned int *bytes_done)
{
	int ret = usb_bulk_read(_handle, endpoint, (char *)data, size, timeout);
	if (bytes_done)
	{
		if (ret >= 0)
		{
			*bytes_done = ret;
		} else {
			*bytes_done = 0;
		}
	}
	
	return ret == (int)size;
}
