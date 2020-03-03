
import os
import sys
import json
import socket
import logging

# Some debug stuff.
import time
import traceback

from gi.repository import GLib

# Standard JSON-RPC error codes.
JSRPC_PARSE_ERROR       = -32700
JSRPC_INVALID_REQUEST   = -32600
JSRPC_METHOD_NOT_FOUND  = -32601
JSRPC_INVALID_PARAMS    = -32603
JSRPC_INTERNAL_ERROR    = -32603

class jsonRpcBatchCall:
    """Helper class to process a JSON-RPC call, or a batched list of calls, and send the response."""
    def __init__(self, sock, address, dbusObj, request):
        self.sock = sock
        self.address = address
        self.batch = []

        # Start the in-progress counter at 1 keep from finishing.
        self.inProgress = 1
        self.isList = isinstance(request, list)

        # A JSON-RPC call can be a list of calls, or just one call.
        if self.isList:
            for x in request:
                self.callMethod(dbusObj, x)
        else:
            self.callMethod(dbusObj, request)
        
        # Decrement the in-progress counter after all calls have been started.
        self.inProgress -= 1
        if self.inProgress == 0:
            self.sendResponse()
    
    def sendResponse(self):
        data = json.dumps(self.batch if self.isList else self.batch[0])
        #logging.debug("Raw JSON-RPC response: %s", data)
        if self.address:
            self.sock.sendto(data.encode('utf-8'), 0, self.address)

    def __del__(self):
        logging.debug("JSON-RPC batch call completed")
    
    def onAsyncReply(self, result, reqId):
        self.inProgress -= 1
        self.batch.append({ "jsonrpc": "2.0", "result": result, "id": reqId })

        if self.inProgress == 0:
            self.sendResponse()

    def onAsyncError(self, exc, reqId):
        self.inProgress -= 1
        self.batch.append({ "jsonrpc": "2.0", "error": {"code": JSRPC_INVALID_REQUEST, "message": str(exc)}, "id": reqId })

        if self.inProgress == 0:
            self.sendResponse()
    
    def callMethod(self, obj, request):
        """Make another JSON-RPC call to the D-Bus object"""
        reqId = request.get("id", None)
        method = getattr(obj, request["method"], None)
        if not method or not method._dbus_is_method:
           self.batch.append({
               "jsonrpc": "2.0",
               "error": {"code": JSRPC_METHOD_NOT_FOUND, "message": "Method not found"},
               "id": reqId
           })

        self.inProgress += 1
        try:
            if method._dbus_async_callbacks:
                callbacks = {}
                callbacks[method._dbus_async_callbacks[0]] = lambda result: self.onAsyncReply(result, reqId)
                callbacks[method._dbus_async_callbacks[1]] = lambda exc: self.onAsyncError(exc, reqId)
                method(request["params"], **callbacks) if method._dbus_in_signature else method(**callback)
            else:
                result = method(request["params"]) if method._dbus_in_signature else method()
                self.onAsyncReply(result, reqId)
        except Exception as exc:
            self.onAsyncError(exc, reqId)

class jsonRpcBridge:
    def __init__(self, dbusObj, sockname):
        ## Delete the socket if it already exists.
        if os.path.exists(sockname):
            os.unlink(sockname)

        ## Create a UNIX datagram socket to receive JSON-RPC.
        logging.info("Creating JSON-RPC socket at %s", sockname)
        self.sockname = sockname
        self.dbusObj = dbusObj
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(sockname)

        ## Attach a GLib callback to handle socket IO
        self.id = GLib.io_add_watch(self.sock.fileno(), GLib.IO_IN, self.onInput)
    
    def __del__(self):
        self.sock.close()
        os.unlink(self.sockname)
        GLib.source_remove(self.id)

    def onInput(self, sourcefd, condition):
        # Receive a datagram for the RPC request.
        try:
            (data, address) = self.sock.recvfrom(16384)
            #logging.info("Received request at time=%f", time.monotonic())
            #logging.debug("Raw JSON-RPC request: %s", data.decode("utf-8"))
            request = json.loads(data.decode("utf-8"))
            jsonRpcBatchCall(self.sock, address, self.dbusObj, request)
        except Exception as e:
            logging.info("Failed to process JSON-RPC socket.")
            logging.debug(traceback.format_exc())

        # Keep receiving data.
        return True
