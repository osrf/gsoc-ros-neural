import serial, time, binascii

# protocol
SYNC 			= '\xaa'

# connection status
CONNECTED 		= '\xc0'
DISCONNECTED 	= '\xc1'
AUTOCONNECT		= '\xc2'


MINDWAVE_CONNECTED 		= '\xd0'
MINDWAVE_STANDBY 		= '\xd4'
MINDWAVE_DISCONNECTED 	= '\xd2'


class Mindwave(object):

	def __init__(self, dev, headset=None):
		
		self.device = dev
		self.idHeadset = headset
		self.meditation = 0 # 0-100
		self.attention = 0 # 0-100
		self.state = None
		self.serial = None
		
		self.open()
	def debug_hex(self, value):
		print 'printed 0x%s ', value.encode('hex')

	def open(self):
		if not self.serial or not self.serial.IsOpen():
			#self.serial = serial.Serial(self.device, baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS, writeTimeout=0, timeout=10, rtscts=False, xonxoff=False)
			self.serial = serial.Serial(self.device, 9600)

	def close(self):
		self.serial.close()

	def connect(self, headset=None):
		if not self.idHeadset:
			self.idHeadset = headset
		# we send a byte to CONNECTED and other byte in hex of headset id
		value = ''.join([CONNECTED, self.idHeadset.decode('hex')])
		self.debug_hex(value)
		self.serial.write(value)

	def autoconnect(self):
		self.debug_hex(AUTOCONNECT)
		self.serial.write(AUTOCONNECT)	

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

	def listen(self):	
		if self.serial.isOpen():
			#while True:
		  		#print ord(self.serial.read())
				#print "Data packet: [%s, %s] " % (self.to_hex(self.serial.read()), self.to_hex(self.serial.read()))
				#print "Data packet: [%s, %s] " % (self.to_hex(self.serial.read()), self.serial.read().encode())
			while True:
				byte1 = self.serial.read()
				byte2 = self.serial.read()						
			
				print '0x%s , 0x%s ' % (byte1.encode('hex'), byte2.encode('hex'))
			
				if byte1 == SYNC and byte2 == SYNC:
					while True:
						plength = self.serial.read() # 0-169
						print ord(plength)
						if plength != 170: 
							break
					if plength > 169:
						continue		

					payload = self.serial.read(plength)

					checksum = 0
					checksum = sum(ord(b) for b in payload[:-1])
					checksum &= 0xff
                    checksum = ~checksum & 0xff

                    chksum = ord(self.serial.read())

                    if checksum != chksum:
                    	continue

                    #print 'payload ', payload.encode('hex')
                    self.parser_payload(payload)
                else:
					pass
				

usb = Mindwave('/dev/ttyUSB0','7B04')
#usb = Mindwave('/dev/ttyUSB0')

time.sleep(2)
usb.connect()
usb.listen()