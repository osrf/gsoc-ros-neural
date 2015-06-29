import time
import bluetooth 
from bluetooth.btcommon import BluetoothError
import serial

from common import Version

class Stream(object):

    def __init__(self, device=None, baudrate=115200, version=Version.MINDWAVE_MOBILE):
        
        self.stream = None
        self.version = version
        
        self.open()

    def open(self):
        try:
            if self.version == Version.MINDWAVE_MOBILE:
                self.stream =  bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            elif self.version == Version.MINDWAVE:
                if not self.stream or not self.stream.isOpen():
                    self.stream = serial.Serial(device, baudrate, timeout=0.001, rtscts=True)
        except Exception, e:
            print "error open port: " + str(e)
            exit()
        
    def read(self, bytes=1):
        #missing = bytes
        data = ""

        try:
            if self.version == Version.MINDWAVE_MOBILE:
                data = self.stream.recv(bytes)
            elif self.version == Version.MINDWAVE:
                data = self.stream.read(bytes)
            return data
        except BluetoothError, e:
            print "Caught BluetoothError: ", e
            time.sleep(3)
            #self.open()
            pass

    def close(self):
        self.stream.close()    

    def getStream(self):
        return self.stream
    
    def isOpen(self):

        if self.version == Version.MINDWAVE_MOBILE:
            return True
        elif self.version == Version.MINDWAVE:
            self.stream.isOpen()
