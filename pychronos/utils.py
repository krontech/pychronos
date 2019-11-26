#!/usr/bin/python3
# Utility methods to enumerate storage devices.
import os
import fcntl
import subprocess
import json
import re

def __readsysfile(name):
    with open(name, 'r') as fp:
        return fp.read().strip()

def getStorageDevices():
    # Get a list of the mounted devices.
    df = subprocess.check_output(['df']).decode("utf-8").splitlines(keepends=False)
    del df[0]

    results = {}
    for line in df:
        device, size, used, available, percent, mountpoint = line.split()
        if device == 'tmpfs' or not mountpoint.startswith("/media/"):
            continue
        
        name = mountpoint[7:]
        results[name] = {
            "device": device,
            "mount": mountpoint,
            "size":   int(size),
            "used":   int(used),
            "available": int(available)
        }
        
    return results

def getNetworkDevices():
    results = {}
    for name in os.listdir("/sys/class/net"):
        carrier = __readsysfile("/sys/class/net/%s/carrier" % (name))
        if carrier == '1':
            results[name] = {
                
            }

    return results


# hostname utility
def getHostname():
    """Gets the current hostname from /etc
    """
    with open('/etc/hostname', 'r') as fp:
        return fp.read().strip()

setHostnameMask = re.compile('chronos-', re.IGNORECASE)
def setHostname(name):
    """Sets the hostname in /etc
    Note: this enforces a hostname that starts with Chronos- to limit possible abuse
    """
    with open('/etc/hostname', 'w') as fp:
        newHostname = setHostnameMask.sub('', name)
        fp.write('Chronos-%s\n' % newHostname)


def smbusRead(addr, length, setup=None):
    fd = os.open("/dev/i2c-1", os.O_RDWR)
    try:
        I2C_SLAVE = 0x0703 # From linux/i2c-dev.h
        fcntl.ioctl(fd, I2C_SLAVE, addr)

        # Write an optional setup blob first.
        if setup:
            os.write(fd, setup)

        data = os.read(fd, length)
        os.close(fd)
        return data
    except Exception as e:
        os.close(fd)
        raise e

if __name__ == '__main__':
    print("externalStorage = " + json.dumps(getStorageDevices(), sort_keys=True, indent=4))
    print("networkInterfaces = " + json.dumps(getNetworkDevices(), sort_keys=True, indent=4))

