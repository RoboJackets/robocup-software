import sys, os
import usb.core, usb.util
from os.path import join, abspath, dirname
from optparse import OptionParser

# Command line argument options
parser = OptionParser()
parser.add_option("-f", "--file", type="string", dest="filename",
                  help="binary file sent to the mbed")
(options, args) = parser.parse_args()

# thisDir = os.path.dirname(os.path.abspath(__file__))
 
if options.filename:
    srcFile = os.path.abspath(options.filename)

devsClass = usb.core.find(bDeviceClass=2)
if devsClass is None:
    raise ValueError('No mbeds connected')


# find our device
devs = list(usb.core.find(bDeviceClass=2, idVendor=0x0d28, idProduct=0x0204, find_all=True))

if devs is None:
    raise ValueError('Device not found')
else:

    print "--  %u mbed%s found" % (len(devs), "a" if len(devs) > 2 else "")

    for dev in devs:

        print dev

        if dev.is_kernel_driver_active(0):
            try:
                dev.detach_kernel_driver(0)
                print "kernel driver detached"

            except usb.core.USBError as e:
                sys.exit("Could not detach kernel driver.")
        
        cfgs = dev.configurations()
        numIntfs = cfgs[0].interfaces()

        for i in range(0,len(numIntfs)):
            try:
                if dev.is_kernel_driver_active(i):
                    dev.detach_kernel_driver(i)
            except:
                pass

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        dev.set_configuration(configuration=1)

        # get an endpoint instance
        intfs = dev.get_active_configuration()
        usb_intf = intfs[(0,0)]

        #print usb_intf

        ep = usb.util.find_descriptor(
        usb_intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)


#        msg = 'test'
#        ret = dev.ctrl_transfer(0x82, CTRL_LOOPBACK_READ, 0, 0, len(msg))


        if srcFile:
            with open(srcFile, "rb") as f:
                byte = f.read(1)
                while byte != "":
                    ep.write(byte)


        for i in range(0,len(numIntfs)):
            try:
                if dev.is_kernel_driver_active(i) is False:
                    dev.attach_kernel_driver(i)
            except:
                pass

        dev.reset()

        # ep.write("");

        # intf = cfg[(0,0)]

        # ep = usb.util.find_descriptor(
        #     intf,
        #     # match the first OUT endpoint
        #     custom_match = \
        #     lambda e: \
        #         usb.util.endpoint_direction(e.bEndpointAddress) == \
        #         usb.util.ENDPOINT_OUT)

        # assert ep is not None

        # write the data
        # ep.write('test')
