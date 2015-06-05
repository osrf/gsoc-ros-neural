import bluetooth 
from bluetooth.btcommon import BluetoothError
import serial, time

from common import Version

class Stream(object):

    def __init__(self, device=None, baudrate=115200, version=Version.MINDWAVE):
        
        self.stream = None
        self.version = version
        if version == Version.MINDWAVE_MOBILE:
            self.stream =  bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        elif version == Version.MINDWAVE:
            self.stream = serial.Serial(device, baudrate, timeout=0.001, rtscts=True)

    def read(self, bytes=1):
        missing = bytes
        received = ""
        print "bytes ",  bytes
        print "version ", self.version
        while missing > 0:
            print "inside loop"
            if self.version == Version.MINDWAVE_MOBILE:
                print "before"
                received = received + self.stream.recv(missing)
                print len(received)
                print "after"
                missing = bytes - len(received)
            elif self.version == Version.MINDWAVE:
                received +=  self.stream.read(missing)
                missing = bytes - len(received)

        print "reading ", received
        return received

    def close(self):
        self.stream.close()    

    def getStream(self):
        return self.stream
    
    def isOpen(self):

        if version == Version.MINDWAVE_MOBILE:
            return True
        elif version == Version.MINDWAVE:
            self.stream.isOpen()
