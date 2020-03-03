#!/usr/bin/python3

import os, sys, pdb
import signal
import json
import argparse

import inspect
import logging
import traceback
import time

import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib

from pychronos import camera
from pychronos.error import *

from pychronos.utils import getBoardRevision

from camControl import controlApi
from camControl import jsonRpcBridge

class cameraTimer:
    def __init__(self, camera):
        self.camera = camera

    def __call__(self, *args):
        self.camera.tick()
        return True

def main():
    # Enable logging.
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s [%(funcName)s] %(message)s')

    # Default sensors to use by top byte of board revision.
    sensorRevLookup = {
        '00': 'lux1310',    # Chronos 1.4 original hardware.
        '14': 'lux1310',    # Chronos 1.4 new production.
        '21': 'lux2100'     # Chronso 2.1
    }

    # Do argument parsing
    parser = argparse.ArgumentParser(description="Chronos control daemon")
    parser.add_argument('--sensor', metavar='NAME', action='store',
                        default=sensorRevLookup.get(getBoardRevision()[0:2], 'lux1310'),
                        help="Override the image sensor driver to use")
    parser.add_argument('--config', metavar='FILE', action='store',
                        default='/var/camera/apiConfig.json',
                        help="Configuration file path")
    parser.add_argument('--jsrpc', metavar='SOCKET', action='store',
                        default='/tmp/camControl.sock',
                        help="JSON-RPC socket name")
    parser.add_argument('--debug', default=False, action='store_true',
                        help="Enable debug logging")
    parser.add_argument('--pdb', default=False, action='store_true',
                        help="Drop into a python debug console on exception")
    args = parser.parse_args()

    if not args.debug:
        logging.getLogger().setLevel(logging.INFO)
    else:
        logging.getLogger().setLevel(logging.DEBUG)

    # Install exception handlers for interactive debug on exception.
    if args.pdb:
        def excepthook(t,v,tb):
            pdb.traceback.print_exception(t, v, tb)
            pdb.post_mortem(t=tb)
        sys.excepthook = excepthook
        dbg, brk = pdb.set_trace, pdb.set_trace #convenience debugging

        #Fix system not echoing keystrokes in debugger, after restart.
        try:
            os.system('stty sane')
        except Exception:
            pass

    # Use the GLib mainloop.
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    mainloop = GLib.MainLoop()

    # Import the desired sensor class.
    sensorModule = __import__("pychronos.sensors", globals(), None, fromlist=[args.sensor])
    sensorClass = sensorModule.__getattribute__(args.sensor)

    # Create the camera and objects.
    sensor = sensorClass()
    cam  = camera(sensor)

    # Install a timer for battery data monitoring
    timer = cameraTimer(cam)
    GLib.timeout_add(1000, timer)

    bus = dbus.SystemBus()
    name = dbus.service.BusName('ca.krontech.chronos.control', bus=bus)
    obj  = controlApi(bus, '/ca/krontech/chronos/control', mainloop, cam, configFile=args.config)

    if args.jsrpc:
        jsrpc = jsonRpcBridge(obj, args.jsrpc)

    # Run the mainloop.
    logging.info("Running control service...")
    mainloop.run()

    # Handle reboot
    logging.info("Stopping control service... %s", obj.rebootMode)
    if obj.rebootMode.get('settings', False):
        os.remove(args.config)

    if obj.rebootMode.get('power', False):
        os.system("shutdown -hr now")
    elif obj.rebootMode.get('reload', True):
        os.kill(os.getpid(), signal.SIGHUP)

if __name__ == "__main__":
    main()
