import serial, time, binascii

# protocol
SYNC = '\xaa'

# connection status
CONNECTED 		= '\xC0'
DISCONNECTED 	= '\xC1'
AUTOCONNECT		= '\xC2'

class Mindwave(object):

	def __init__(self, dev, headset):
		
		self.device = dev
		self.idHeadset = headset
		self.meditation = 0 # 0-100
		self.attention = 0 # 0-100
		self.state = None
		self.serial = None
		
		self.open()

	def open(self):
		if not self.serial or not self.serial.IsOpen():
			self.serial = serial.Serial(self.device, baudrate=57600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS, writeTimeout=0, timeout=10, rtscts=False, xonxoff=False)
	def close(self):
		self.serial.close()

	def connect(self, headset=None):
		if not self.idHeadset:
			self.idHeadset = headset
		# we send a byte to CONNECTED and other byte in hex of headset id
		self.serial.write(''.join([CONNECTED, self.idHeadset.decode('hex')])) 

	def autoconnect():
		self.serial.write(AUTOCONNECT)	

	def disconnect():
		self.serial.write(DISCONNECTED)

		   
	def to_hex(t, nbytes=1):
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
			while True:
		  		#print ord(self.serial.read())
				#print "Data packet: [%s, %s] " % (self.to_hex(self.serial.read()), self.to_hex(self.serial.read()))
				print "Data packet: [%s, %s] " % (self.serial.read(), self.serial.read().encode())

		  # while True:
		  #   print 'debug'
		  #   if port.read() == SYNC and port.read() == SYNC:
		  #     while True:
		  #       plength = port.read() # 0-169
		  #       print ord(plength)
		  #       print 'debug2' 

usb = Mindwave('/dev/ttyUSB0','7B04')
time.sleep(5)
usb.connect()
usb.listen()