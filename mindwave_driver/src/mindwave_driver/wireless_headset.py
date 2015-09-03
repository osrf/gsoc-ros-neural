import time


from headset import Headset
from stream import Stream
from common import Version, BytesStatus

class WirelessHeadset(Headset):
    """This class represents the wireless version of the mindwave

    Args:
        dev: device link 
        headset: the id of mindwave wireless version

    It has the basic functionality to connect, autoconnect and disconnect

    """

    def __init__(self, dev=None, headset_id=None, rate=None):

        Headset.__init__(self, headset_id)

        self.device = dev
        self.bauderate = rate

        self.stream = Stream(device=self.device, bauderate=rate, version=Version.MINDWAVE)
        time.sleep(2)

        self.connect()
        self.run(self.stream)

    # def open(self):
    #     if not self.stream or not self.stream.IsOpen():
    #         #self.stream = stream.stream(self.device, baudrate=115200, parity=stream.PARITY_NONE, stopbits=stream.STOPBITS_ONE,
    #         #    bytesize=stream.EIGHTBITS, writeTimeout=0, timeout=3, rtscts=True, xonxoff=False)
    #         self.stream = serial.Serial(self.device, self.baudrate, timeout=0.001, rtscts=True)

    def autoconnect(self):
        """This method autoconnects to the mindwave every."""

        self.stream.getStream().write(BytesStatus.AUTOCONNECT)  
        #the dongle switch to autoconnect mode it must wait 10 second to connect any headset
        time.sleep(10)

    def connect(self):
        """This method connects to the mindwave with the id."""

        if self.id is not None:
            # we send a byte to CONNECTED and other byte in hex of headset id
            self.stream.getStream().write(''.join([BytesStatus.CONNECT, self.id.decode('hex')]))
        else:
            self.autoconnect()

    def disconnect(self):
        """This method disconnects the mindwave."""

        self.stream.getStream().write(BytesStatus.DISCONNECT)
         
    def echo_raw(self):
        """This method prints the raw data from mindwave."""
        
        while 1:
            #time.sleep()
            data = self.stream.read(1)
    
            for b in data:
                print '0x%s, ' % b.encode('hex'),
            print ""