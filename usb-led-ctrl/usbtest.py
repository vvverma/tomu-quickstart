import usb.core


if __name__ == "__main__":
    dev = usb.core.find(idVendor=0x1209, idProduct=0x70b1)
    dev.reset()

    if dev is None:
    	raise ValueError('Device not found')
    dev.set_configuration()
    endpoint_OUT = dev[0][(0,0)][0]
    print "Detected: EP: HOST -> DEVICE"
    print endpoint_OUT
    endpoint_IN = dev[0][(0,0)][1]
    print "Detected: EP: DEVICE -> HOST"
    print endpoint_IN

    print "Enter the color of led R / G / OFF"
    msg = raw_input()
    dev.write(endpoint_OUT.bEndpointAddress, msg, 100)
    ret = dev.read(endpoint_IN.bEndpointAddress, len(msg), 100)
    sret = ''.join([chr(x) for x in ret])
    assert sret == msg
    print "Device -> Host: %s" % sret
    usb.util.dispose_resources(dev)
