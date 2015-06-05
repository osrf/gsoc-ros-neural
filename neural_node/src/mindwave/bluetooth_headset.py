import sys
import bluetooth
from bluetooth.btcommon import BluetoothError
import time

from headset import Headset
from stream import Stream
#from parser import Parser
#from listener import DongleListener
from common import *


class BluetoothHeadset(Headset):
    def __init__(self, addr=None):
        self.addr = addr
        self.isConnected = False
        
        Headset.__init__(self, version=Version.MINDWAVE_MOBILE)
        
        if addr is None:
            self.addr = self.find()
            
        if self.addr is None:
            print "No Mindwave Mobile found"
            sys.exit(-1)

        self.stream = Stream(version=Version.MINDWAVE_MOBILE)
        time.sleep(1)

        print self.stream

        self.connect(addr)
        print self.status

       
        
        self.run(self.stream)

        # self.parser = Parser(self, self.stream)
        # self.listener = DongleListener(self.parser)
            
        # if not self.listener.isAlive():
        #     self.listener.daemon = True
        #     self.listener.start()   

    def connect(self, addr):
      
        while not self.isConnected:
            try:
                if not self.stream is None:
                    self.stream.getStream().connect((addr, 1))
                    self.stream.getStream().setblocking(False)
                    self.isConnected = True
                    self.status = Status.CONNECTED
            except BluetoothError, e:
                print "Connection error: ", e, "Retriyin in 5 secs.."
                time.sleep(5)

    def find(self):
        try:
            devices = bluetooth.discover_devices(lookup_names=True, duration=5)
            
            #from common import Version
            print devices
            for addr, name in devices:
                print name, addr    
                if (name == "MindWave Mobile"):
                    return addr
            self.status = Status.NOFOUND
        except BluetoothError, e:
            print e
        return None

