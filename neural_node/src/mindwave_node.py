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


class DongleListener(threading.Thread):
    def __init__(self, parser, *args, **kwargs):
        self.parser = parser
        self.running = True
        super(DongleListener, self).__init__(*args, **kwargs)
    
    def run(self):
        while True:
            #time.sleep(0.5)
            # listeng for incoming bytes             
            self.parser.listen()
            time.sleep(1)

    def stop(self):
        self.running = False
        self._Thread__stop()

class Parser(object):
    def __init__(self, headset, stream):
        self.headset = headset
        self.stream = stream
        self.package = []

    def listen(self):
        #self.disconnect()
        settings = self.stream.getSettingsDict()   

        for i in xrange(2):
            settings['rtscts'] = not settings['rtscts']
            self.stream.applySettingsDict(settings)

        if self.stream.isOpen():
            #while True:
                #print ord(self.stream.read())
                #print "Data packet: [%s, %s] " % (self.to_hex(self.stream.read()), self.to_hex(self.stream.read()))
                #print "Data packet: [%s, %s] " % (self.to_hex(self.stream.read()), self.stream.read().encode())
            while True:
                byte1 = self.stream.read()
                byte2 = self.stream.read()                      
                
                self.package.append(byte1)
                self.package.append(byte2)

                if byte1 == SYNC and byte2 == SYNC:
                    #print '0x%s , 0x%s ' % (byte1.encode('hex'), byte2.encode('hex'))
                
                    while True:
                        plength = self.stream.read() # 0-169
                        self.package.append(plength)
                        plength = ord(plength)
                        if plength != 170: 
                            break
                    if plength > 169: # return to while
                        continue        
                    
                    payload = self.stream.read(plength)
                    checksum = 0
                    checksum = sum(ord(b) for b in payload[:-1])
                    checksum &= 0xff
                    checksum = ~checksum & 0xff

                    chksum = ord(self.stream.read())

                    if checksum != chksum:
                        pass

                    #print 'payload ', payload.encode('hex')
                    self.parser_payload(payload)

                    for b in self.package:
                        print '0x%s, ' % b.encode('hex'),
                    print ""
        
                    self.package = []
                else:
                    pass        
        else:
            print 'no stream open'

    def parser_payload(self, payload):

        while payload:
            code, payload = payload[0], payload[1:] 
            self.package.append(code)
        
            if code >= 0x80:
                vlength, payload = payload[0], payload[1:]
                value, payload = payload[:ord(vlength)], payload[ord(vlength):]
                
                self.package.append(vlength)
                self.package.append(value)
                        
                if code == MINDWAVE_CONNECTED:
                    # headset found
                    # format: 0xaa 0xaa 0x04 0xd0 0x02 0x05 0x05 0x23
                    self.headset.status = MINDWAVE_STATUS_CONNECTED
                    self.headset.id = value 
                                            
                elif code == MINDWAVE_NOFOUND:  # it can be 0 or 2 bytes
                    # headset no found
                    # format: 0xaa 0xaa 0x04 0xd1 0x02 0x05 0x05 0xf2

                    self.headset.status = MINDWAVE_STATUS_NOFOUND
                    
                    # 0xAA 0xAA 0x02 0xD1 0x00 0xD9
                    
                elif code == MINDWAVE_DISCONNECTED: # dongle send 4 bytes
                    # headset found
                    # format: 0xaa 0xaa 0x04 0xd2 0x02 0x05 0x05 0x21
                    self.headset.status = MINDWAVE_STATUS_DISCONNECTED
                
                elif code == MINDWAVE_REQUESTDENIED:
                    # headset found
                    # format: 0xaa 0xaa 0x02 0xd3 0x00 0x2c
                    self.headset.status = MINDWAVE_STATUS_DENIED
                
                elif code == MINDWAVE_STANDBY: # waiting for a command the device send a byte 0x00
                    # standby/scanning mode
                    # format: 0xaa 0xaa 0x03 0xd4 0x01 0x00 0x2a
                    self.headset.status = MINDWAVE_STATUS_STANDBY
                     
                else:
                    #unknow multibyte
                    pass
            else: 
                # single byte there isn't vlength
                # 0-127
                value, payload = payload[0], payload[1:]
                
                self.package.append(value)

                if code == POOR_SIGNAL:
                    self.headset.signal = ord(value)
                elif code == ATTENTION:
                    self.headset.attention = ord(value)
                elif code == MEDITATION:
                    self.headset.meditation = ord(value)
                elif code == BLINK:
                    self.headset.blink = ord(value)
                else: 
                    pass


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
        self.stream = None

        self.open()
        time.sleep(0.5)
        
        self.parser = Parser(self, self.stream)
        self.listener = DongleListener(self.parser)
        
        if not self.listener.isAlive():
            self.listener.daemon = True
            self.listener.start()
        
    def open(self):
        if not self.stream or not self.stream.IsOpen():
            #self.stream = stream.stream(self.device, baudrate=115200, parity=stream.PARITY_NONE, stopbits=stream.STOPBITS_ONE,
            #    bytesize=stream.EIGHTBITS, writeTimeout=0, timeout=3, rtscts=True, xonxoff=False)
            self.stream = serial.Serial(self.device, self.baudrate, timeout=0.001)

    def close(self):
        self.stream.close()

    def autoconnect(self):
        #self.debug_hex(AUTOCONNECT)
        self.stream.write(AUTOCONNECT)  
        #the dongle switch to autoconnect mode it must wait 10 second to connect any headset
        time.sleep(10)

    def connect(self):
        if self.id is not None:
            # we send a byte to CONNECTED and other byte in hex of headset id
            self.stream.write(''.join([CONNECTED, self.id.decode('hex')]))
        else:
            self.autoconnect()

    def disconnect(self):
        self.stream.write(DISCONNECTED)

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
        logger.info(self.stream.isOpen())
        logger.info("checking if we have characters to read:")
        logger.info(self.stream.inWaiting())
        logger.info("getting settings dict:")
        logger.info(self.stream.getSettingsDict())

headset = Mindwave('/dev/ttyUSB0')
#headset = Mindwave('/dev/ttyUSB0','7B04')

while True:
    if headset.status == MINDWAVE_STATUS_STANDBY:     
        print "trying connecting"
        headset.connect()
        time.sleep(0.5)

    elif headset.status == MINDWAVE_STATUS_CONNECTED:
        print "quality signal: %s , attention %s , %s meditation " % (headset.signal, headset.attention, headset.meditation)
    