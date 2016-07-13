import os


def find(vendor, product):
    # Try to find the device with sysfs.
    # This is Linux-specific.
    usb_path = '/sys/bus/usb/devices'
    usbs = os.listdir(usb_path)
    devices = []
    for u in usbs:
        try:

            def contents(filename):
                f = file(filename)
                x = f.read()
                f.close()
                return x

            v = int(contents('%s/%s/idVendor' % (usb_path, u)), 16)
            p = int(contents('%s/%s/idProduct' % (usb_path, u)), 16)
            if v == vendor and p == product:
                t = '/dev/' + os.listdir('%s/%s/%s:1.0/tty' %
                                         (usb_path, u, u))[0]
                if os.path.exists(t):
                    devices.append(t)
        except:
            pass
    return devices
