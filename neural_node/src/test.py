import time

from mindwave.wireless_headset import WirelessHeadset
from mindwave.common import *

#headset = Mindwave('/dev/ttyUSB0')
headset = WirelessHeadset('/dev/ttyUSB0','7B04')

while True:
    if headset.status == Status.STANDBY:
        print 'trying connecting'
        headset.connect()
        time.sleep(2)
    elif headset.status == Status.CONNECTED:
        print "quality signal: %s , attention %s , %s meditation " % (headset.signal, headset.attention, headset.meditation)
    else:
        print "status " , headset.status
    time.sleep(0.5)