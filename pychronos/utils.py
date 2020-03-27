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
    results = {}
    with open("/proc/mounts", 'r') as fp:
        mounts = fp.readlines()
        for line in mounts:
            device, mountpoint, fstype, options, dump, tries = line.split()
            if device == 'tmpfs':
                continue
            if not mountpoint.startswith("/media/"):
                continue
            
            # Get the block device, if any.
            try:
                args = ['lsblk', '-no', 'pkname', device]
                blkdev = subprocess.check_output(args, stderr=subprocess.DEVNULL).decode('utf-8').strip()
            except Exception:
                blkdev = None

            # Gather a description of the block device.
            if (fstype == "cifs"):
                desc = "SMB Share"
            elif (fstype == "nfs"):
                desc = "NFS Share"
            elif (__readsysfile("/sys/class/block/%s/device/type" % blkdev) == "SD"):
                desc = "MMC/SD Card"
            elif blkdev is not None:
                vendor = __readsysfile("/sys/class/block/%s/device/vendor" % blkdev)
                model = __readsysfile("/sys/class/block/%s/device/model" % blkdev)
                desc = vendor + " " + model

            # Is this device a partition?
            if device.startswith("/dev/"):
                devname = device[5:]
                if os.path.exists("/sys/class/block/%s/partition" % devname):
                   desc += " Partiton " + __readsysfile("/sys/class/block/%s/partition" % devname)

            # If it's also a FUSE filesystem, get the real filesystem.
            if (fstype == "fuseblk"):
                fstype = subprocess.check_output(['lsblk', '-no', 'fstype', device]).decode('utf-8').strip()

            name = mountpoint[7:]
            results[name] = {
                "device": device,
                "description": desc,
                "mount": mountpoint,
                "fstype": fstype
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

def getBoardRevision():
    with open("/proc/cpuinfo", "r") as fp:
        for line in fp.readlines():
            tokens = line.split(':')
            if (tokens[0].strip() == "Revision"):
                return tokens[1].strip()
    
    # If all else fails, return an original Chronos 1.4.
    return '0000'

if __name__ == '__main__':
    print("externalStorage = " + json.dumps(getStorageDevices(), sort_keys=True, indent=4))
    print("networkInterfaces = " + json.dumps(getNetworkDevices(), sort_keys=True, indent=4))

