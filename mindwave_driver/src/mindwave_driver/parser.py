

def bigend_24b(b1, b2, b3):
    return b1* 255 * 255 + 255 * b2 + b3

class Parser(object):
    def __init__(self, headset, stream):
        self.headset = headset
        self.stream = stream
        self.buffer = []
    
    def __call__(self):
        return self

    def print_bytes(self, data):
       for b in data:
            print '0x%s, ' % b.encode('hex'),

    def parser(self, data):
        # settings = self.stream.getSettingsDict()   

        # for i in xrange(2):
        #     settings['rtscts'] = not settings['rtscts']
        #     self.stream.applySettingsDict(settings)

        while len(data)> 1 :
            try:
                byte1, byte2 = data[0],  data[1]

                # SYNC | SYNC | PLENGTH | PAYLOAD | CHKSUM
                # PAYLOAD: (EXCODE) | CODE |(VLENGTH) | VALUE
                if byte1 == Bytes.SYNC and byte2 == Bytes.SYNC:

                    #self.buffer.append(byte1)
                    #self.buffer.append(byte2)
                    data = data[2:]
                    while True:
                        plength = data[0] # 0-169
                        #self.buffer.append(plength)
                        plength = ord(plength)
                        if plength != 170:
                            break
                    if plength > 170:
                        pass #continue

                    data = data[1:]

                    payload = data[:plength]
                    checksum = 0
                    checksum = sum(ord(b) for b in payload[:-1])
                    checksum &= 0xff
                    checksum = ~checksum & 0xff

                    chksum = data[plength]

                    if checksum != ord(chksum):
                        pass

                    self.parser_payload(payload)
                    #self.buffer.append(chksum)

                    data = data[plength+1:]

                    # for b in self.buffer:
                    #     if not b == "":
                    #         print '0x%s, ' % b.encode('hex'),
                    # print ""

                    #self.buffer = []
                else:
                   data = data[1:]

            except IndexError:
                pass



    def listen(self):
        # settings = self.stream.getSettingsDict()   

        # for i in xrange(2):
        #     settings['rtscts'] = not settings['rtscts']
        #     self.stream.applySettingsDict(settings)

        #if self.stream.isOpen():
           #while True:
        byte1 = self.stream.read(1)
        byte2 = self.stream.read(1)                      

        self.buffer.append(byte1)
        self.buffer.append(byte2)
        
        # SYNC | SYNC | PLENGTH | (EXCODE) | CODE |(VLENGTH) | VALUE
        if byte1 == Bytes.SYNC and byte2 == Bytes.SYNC:
         
            while True:
                plength = self.stream.read() # 0-169
                self.buffer.append(plength)
                plength = ord(plength)
                if plength != 170: 
                    break
            if plength > 169: # return to while
                pass #continue        
            
            payload = self.stream.read(plength)
            checksum = 0
            checksum = sum(ord(b) for b in payload[:-1])
            checksum &= 0xff
            checksum = ~checksum & 0xff

            chksum = self.stream.read()

            if checksum != ord(chksum):
                pass

            #print 'payload ', payload.encode('hex')
            self.parser_payload(payload)
            self.buffer.append(chksum)

            # for b in self.buffer:
            #     if not b == "":
            #         print '0x%s, ' % b.encode('hex'),
            # print ""

            self.buffer = []
        else:
            pass        
        #else:
        #    print 'no stream open'

    def parser_payload(self, payload):

        while payload:
            
            try:
                code, payload = payload[0], payload[1:] 
            except IndexError:
                pass    
            
            #self.buffer.append(code)
            # multibytes
            if ord(code) >= 0x80:
                
                try:
                    vlength, payload = payload[0], payload[1:]
                    value, payload = payload[:ord(vlength)], payload[ord(vlength):]                  
                except IndexError:
                    pass    
            
                #self.buffer.append(vlength)
                      
                if code == BytesStatus.RESPONSE_CONNECTED:
                    # headset found
                    # format: 0xaa 0xaa 0x04 0xd0 0x02 0x05 0x05 0x23
                    self.headset.status = Status.CONNECTED
                    self.headset.id = value 
                                            
                elif code == BytesStatus.RESPONSE_NOFOUND:  # it can be 0 or 2 bytes
                    # headset no found
                    # format: 0xaa 0xaa 0x04 0xd1 0x02 0x05 0x05 0xf2

                    self.headset.status = Status.NOFOUND
                    
                    # 0xAA 0xAA 0x02 0xD1 0x00 0xD9
                    
                elif code == BytesStatus.RESPONSE_DISCONNECTED: # dongle send 4 bytes
                    # headset found
                    # format: 0xaa 0xaa 0x04 0xd2 0x02 0x05 0x05 0x21
                    self.headset.status = Status.DISCONNECTED
                
                elif code == BytesStatus.RESPONSE_REQUESTDENIED:
                    # headset found
                    # format: 0xaa 0xaa 0x02 0xd3 0x00 0x2c
                    self.headset.status = Status.DENIED
                
                elif code == 0xd4: # waiting for a command the device send a byte 0x00
                    # standby/scanning mode
                    # format: 0xaa 0xaa 0x03 0xd4 0x01 0x00 0x2a
                    print 'scanning'
                    self.headset.status = Status.STANDBY
                     
                elif code == Bytes.RAW_VALUE:
                    hight = value[0] 
                    low = value[1]
                    #self.buffer.append(hight)
                    #self.buffer.append(low)
                    self.headset.raw_value = ord(hight)*255+ord(low)
 
                elif code == Bytes.ASIC_EEG_POWER:
                    # ASIC_EEG_POWER_INT
                    # delta, theta, low-alpha, high-alpha, low-beta, high-beta,
                    # low-gamma, high-gamma

                    self.headset.asig_eeg_power = []
                    for i in range(8):
                        self.headset.asig_eeg_power.append(bigend_24b(value[0], value[1], value[2]))
                else: #unknow multibyte
                    pass
            else:   
                # single byte there isn't vlength
                # 0-127
                value, payload = payload[0], payload[1:]
                #self.buffer.append(value)

                if code == Bytes.POOR_SIGNAL:
                    self.headset.signal = ord(value)
                elif code == Bytes.ATTENTION:
                    self.headset.attention = ord(value)
                elif code == Bytes.MEDITATION:
                    self.headset.meditation = ord(value)
                elif code == Bytes.BLINK:
                    self.headset.blink = ord(value)
                else: 
                    pass

from common import *
