import time


from headset import Headset
from stream import Stream
from common import Version, BytesStatus

class WirelessHeadset(Headset):

    def __init__(self, dev="/dev/ttyUSB0", headset_id=None):

        Headset.__init__(self, headset_id, version=Version.MINDWAVE)

        self.device = dev
        
        self.stream = Stream(device=dev, version=Version.MINDWAVE)
        time.sleep(2)

        self.connect()
        self.run(self.stream)

    # def open(self):
    #     if not self.stream or not self.stream.IsOpen():
    #         #self.stream = stream.stream(self.device, baudrate=115200, parity=stream.PARITY_NONE, stopbits=stream.STOPBITS_ONE,
    #         #    bytesize=stream.EIGHTBITS, writeTimeout=0, timeout=3, rtscts=True, xonxoff=False)
    #         self.stream = serial.Serial(self.device, self.baudrate, timeout=0.001, rtscts=True)

    def autoconnect(self):
        #self.debug_hex(AUTOCONNECT)
        self.stream.getStream().write(BytesStatus.AUTOCONNECT)  
        #the dongle switch to autoconnect mode it must wait 10 second to connect any headset
        time.sleep(10)

    def connect(self):
        if self.id is not None:
            # we send a byte to CONNECTED and other byte in hex of headset id
            self.stream.getStream().write(''.join([BytesStatus.CONNECT, self.id.decode('hex')]))
        else:
            self.autoconnect()

    def disconnect(self):
        self.stream.getStream().write(BytesStatus.DISCONNECT)

    def print_connection_data(self):
        logger.info("checking if port is opened:")
        logger.info(self.stream.isOpen())
        logger.info("checking if we have characters to read:")
        logger.info(self.stream.inWaiting())
        logger.info("getting settings dict:")
        logger.info(self.stream.getSettingsDict())
    
    def echo_raw(self):
        while 1:
            #time.sleep()
            data = self.stream.read(1)
    
            for b in data:
                print '0x%s, ' % b.encode('hex'),
            print ""