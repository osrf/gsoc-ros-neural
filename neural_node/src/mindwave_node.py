import serial, time, binascii, threading

# protocol
SYNC            = '\xaa'
POOR_SIGNAL     = '\x02'
ATTENTION       = '\x04'
MEDITATION      = '\x05'
BLINK           = '\x16'

# connection status
CONNECTED       = '\xc0'
DISCONNECTED    = '\xc1'
AUTOCONNECT     = '\xc2'


MINDWAVE_CONNECTED      = '\xd0'
MINDWAVE_NOFOUND        = '\xd1'
MINDWAVE_DISCONNECTED   = '\xd2'
MINDWAVE_REQUESTDENIED  = '\xd3'
MINDWAVE_STANDBY        = '\xd4'

MINDWAVE_STATUS_CONNECTED       = 0
MINDWAVE_STATUS_NOFOUND         = 1
MINDWAVE_STATUS_DISCONNECTED    = 2
MINDWAVE_STATUS_DENIED          = 3
MINDWAVE_STATUS_STANDBY         = 4


class DongleReader(threading.Thread):
    def __init__(self, parser, *args, **kargs):
        self.parser = parser
        self.running = True
        super(DongleReader, self).__init__(*args, **kwargs)
    
    def run(self):
        while True:
            pass        
    def stop(self):
        self.running = False
        self._Thread__stop()

class Mindwave(object):

    def __init__(self, dev, headset_id=None, baudrate=115200):
        
        if headset_id:
            self.id = headset_id
            self.auto_connect = True
        else:
            self.id = headset_id
            self.auto_connect = False

        self.device = dev
        self.baudrate = baudrate

        self.meditation = 0 # 0-100
        self.attention = 0 # 0-100
        self.signal = 0
        self.status = None
        self.serial = None
        self.package = []

        self.open()

    def debug_hex(self, value):
        print 'printed 0x%s ', value.encode('hex')

    def open(self):
        if not self.serial or not self.serial.IsOpen():
            #self.serial = serial.Serial(self.device, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            #    bytesize=serial.EIGHTBITS, writeTimeout=0, timeout=3, rtscts=True, xonxoff=False)
            self.serial = serial.Serial(self.device, self.baudrate, timeout=0.001)

    def close(self):
        self.serial.close()

    def autoconnect(self):
        #self.debug_hex(AUTOCONNECT)
        self.serial.write(AUTOCONNECT)  
        #if autoconnect we must wait 10 seconds
        time.sleep(10)

    def connect(self):
        if self.id is not None:
            # we send a byte to CONNECTED and other byte in hex of headset id
            self.serial.write(''.join([CONNECTED, self.id.decode('hex')]))
        else:
            self.autoconnect()

    def disconnect(self):
        self.serial.write(DISCONNECTED)

    def to_hex(t, nbytes):
        "Format text t as a sequence of nbyte long values separated by spaces."
        chars_per_item = nbytes * 2
        hex_version = binascii.hexlify(t)
        num_chunks = len(hex_version) / chars_per_item
        def chunkify():
            for start in xrange(0, len(hex_version), chars_per_item):
                yield hex_version[start:start + chars_per_item]
        return ' '.join(chunkify())
    
    def print_connection_data(self):
        logger.info("checking if port is opened:")
        logger.info(self.serial.isOpen())
        logger.info("checking if we have characters to read:")
        logger.info(self.serial.inWaiting())
        logger.info("getting settings dict:")
        logger.info(self.serial.getSettingsDict())

    def parser_payload(self, payload):

        vlength = 0
        
        code = payload[0]
        payload = payload[1:] 
        
        self.package.append(code)
        if code >= 0x80:
            if code == MINDWAVE_CONNECTED:
                # headset found
                # format: 0xaa 0xaa 0x04 0xd0 0x02 0x05 0x05 0x23
                self.status = MINDWAVE_STATUS_CONNECTED
                vlength = payload[0]
                self.package.append(vlength)
                self.package.append( payload[1])
                self.package.append( payload[2])
                     
            elif code == MINDWAVE_NOFOUND:  # it can be 0 or 2 bytes
                # headset no found
                # format: 0xaa 0xaa 0x04 0xd1 0x02 0x05 0x05 0xf2

                self.status = MINDWAVE_STATUS_NOFOUND
                vlength = payload[0]
                self.package.append(vlength)
                
                # 0xAA 0xAA 0x02 0xD1 0x00 0xD9
                if ord(vlength) == 2:
                    self.package.append( payload[1])
                    self.package.append( payload[2])
                else:
                    pass

            elif code == MINDWAVE_DISCONNECTED: # dongle send 4 bytes
                # headset found
                # format: 0xaa 0xaa 0x04 0xd2 0x02 0x05 0x05 0x21
                self.status = MINDWAVE_STATUS_DISCONNECTED
                vlength = payload[0]
                self.package.append(vlength)
                self.package.append( payload[1])
                self.package.append( payload[2])

            elif code == MINDWAVE_REQUESTDENIED:
                # headset found
                # format: 0xaa 0xaa 0x02 0xd3 0x00 0x2c
                self.status = MINDWAVE_STATUS_DENIED
                vlength = payload[0]
                self.package.append(vlength)

            elif code == MINDWAVE_STANDBY: # waiting for a command the device send a byte 0x00
                # standby/scanning mode
                # format: 0xaa 0xaa 0x03 0xd4 0x01 0x00 0x2a
                self.status = MINDWAVE_STATUS_STANDBY
                vlength = payload[0]
                value = payload[1]
                self.package.append(vlength)
                self.package.append(value)
                 
            else:
                #unknow multibyte
                pass
        else: 
            # single byte there isn't vlength
            # 0-127
            byte, payload = ord(payload[0]), payload[1:0]

            if code == POOR_SIGNAL:
                self.signal = byte
            elif code == ATTENTION:
                self.attention = byte
            elif code == MEDITATION:
                self.meditation = byte
            elif code == BLINK:
                self.blink = byte
            else: 
                pass
        
        for b in self.package:
            print '0x%s, ' % b.encode('hex'),
        print ""

        self.package = []    

    def listen(self):

        #self.disconnect()
        settings = self.serial.getSettingsDict()   

        for i in xrange(2):
            settings['rtscts'] = not settings['rtscts']
            self.serial.applySettingsDict(settings)

        if self.serial.isOpen():
            #while True:
                #print ord(self.serial.read())
                #print "Data packet: [%s, %s] " % (self.to_hex(self.serial.read()), self.to_hex(self.serial.read()))
                #print "Data packet: [%s, %s] " % (self.to_hex(self.serial.read()), self.serial.read().encode())
            while True:
                byte1 = self.serial.read()
                byte2 = self.serial.read()                      
                
                self.package.append(byte1)
                self.package.append(byte2)

                if byte1 == SYNC and byte2 == SYNC:
                    #print '0x%s , 0x%s ' % (byte1.encode('hex'), byte2.encode('hex'))
                
                    while True:

                        plength = self.serial.read() # 0-169
                        self.package.append(plength)
                        plength = ord(plength)
                        if plength != 170: 
                            break
                    if plength > 169: # return to while
                        continue        
                    
                    payload = self.serial.read(plength)
                    checksum = 0
                    checksum = sum(ord(b) for b in payload[:-1])
                    checksum &= 0xff
                    checksum = ~checksum & 0xff

                    chksum = ord(self.serial.read())

                    if checksum != chksum:
                        pass

                    #print 'payload ', payload.encode('hex')
                    self.parser_payload(payload)
                else:
                    pass
                
                if self.status != MINDWAVE_STATUS_CONNECTED:
                    #self.disconnect()                
                    self.connect()
                    time.sleep(1)

                if self.status == MINDWAVE_STATUS_CONNECTED:
                    print "quality signal: %s , attention %s , %meditation " % (self.signal, self.attention, self.meditation)
        else:
            print 'no serial open'

usb = Mindwave('/dev/ttyUSB0')
#usb = Mindwave('/dev/ttyUSB0',, 57600)

time.sleep(2)
usb.connect()
usb.listen() # listen for incomming bytes
