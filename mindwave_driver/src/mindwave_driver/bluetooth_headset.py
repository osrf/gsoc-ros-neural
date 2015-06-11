import sys
import bluetooth
from bluetooth.btcommon import BluetoothError
import time

from headset import Headset
from stream import Stream
from parser import Parser
from listener import Listener
from common import *


class BluetoothHeadset(Headset):
    def __init__(self, addr=None):
        self.addr = addr
        self.isConnected = False
        self.name = ""
        
        Headset.__init__(self, version=Version.MINDWAVE_MOBILE)
        
        if addr is None:
            self.addr = self.find()
            
        if self.addr is None:
            self.status = Status.NOFOUND
            print "No Mindwave Mobile found"
            sys.exit(-1)
        else:
            print self.name + " " + self.addr

        self.stream = Stream(version=Version.MINDWAVE_MOBILE)
        time.sleep(1)

        self.connect(self.addr)
            
        self.run(self.stream)

    def connect(self, addr):
      
        while not self.isConnected:
            try:
                if not self.stream is None:
                    self.stream.getStream().connect((addr, 1))
                    self.stream.getStream().setblocking(False)
                    self.isConnected = True
                    self.status = Status.CONNECTED
            except BluetoothError, e:
                self.status = Status.DISCONNECTED
                print "Connection error: ", e, "Retriyin in 5 secs.."
                time.sleep(5)

    def find(self):
        try:
            devices = bluetooth.discover_devices(lookup_names=True, duration=5)
            
            #from common import Version
            print devices
            for addr, name in devices:
                if (name == Version.MINDWAVE_MOBILE):
                    self.name = name
                    return addr
        except BluetoothError, e:
            print e
        return None

    def echo_raw(self):
        while 1:
            time.sleep(0.1)
            try:
                data = self.stream.getStream().recv(DEFAULT_BYTES)
            except BluetoothError, e:
                print e
                time.sleep(0.5)
                continue
        
            for b in data:
                print '0x%s, ' % b.encode('hex'),
            print ""

    def read(self): # read without daemon thread 
        while True:
            data = self.stream.getStream().recv(DEFAULT_BYTES)
            self.parser.parser(data)
            time.sleep(0.5)
            