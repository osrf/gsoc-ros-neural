#!/usr/bin/env python

import rospy
import time
from mindwave_driver.bluetooth_headset import BluetoothHeadset
from mindwave_driver.common import *

# 20:68:9D:70:CA:96
headset = BluetoothHeadset("20:68:9D:70:CA:96")
#headset = BluetoothHeadset()

#headset.echo_raw()
#headset.read()

while True:
    if headset.status != Status.CONNECTED:
        print 'trying connecting'
        headset.connect(headset.addr)
        time.sleep(1)
    elif headset.status == Status.CONNECTED:
        print "quality signal: %s, attention %s, meditation %s, blink %s, raw %s" % (headset.signal, headset.attention, \
        	headset.meditation, headset.blink, headset.raw_value)
        # delta, theta, low-alpha, high-alpha, low-beta, high-beta,
        # low-gamma, high-gamma
        for e in headset.asig_eeg_power:
            print e,
        print ""
    else:
        pass
    time.sleep(0.2)