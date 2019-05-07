#!/usr/bin/python3
# Utility methods to enumerate storage devices.
import os
import subprocess
import json

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

if __name__ == '__main__':
    print("externalStorage = " + json.dumps(getStorageDevices(), sort_keys=True, indent=4))
    print("networkInterfaces = " + json.dumps(getNetworkDevices(), sort_keys=True, indent=4))

