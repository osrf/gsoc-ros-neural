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
        data = ""

        # while missing > 0:
        #     if self.version == Version.MINDWAVE_MOBILE:
        #         data = data + self.stream.recv(missing)
        #         missing = bytes - len(data)
        #     elif self.version == Version.MINDWAVE:
        #         data +=  self.stream.read(missing)
        #         missing = bytes - len(data)
        # return data
        
        if self.version == Version.MINDWAVE_MOBILE:
            data = self.stream.recv(bytes)
        elif self.version == Version.MINDWAVE:
            data = self.stream.read(bytes)
        return data
        
    def close(self):
        self.stream.close()    

    def getStream(self):
        return self.stream
    
    def isOpen(self):

        if version == Version.MINDWAVE_MOBILE:
            return True
        elif version == Version.MINDWAVE:
            self.stream.isOpen()
