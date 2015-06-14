#!/usr/bin/env python

import rospy
import roslib
import time

# old version of ros
roslib.load_manifest('mindwave_driver')

from mindwave_driver.bluetooth_headset import BluetoothHeadset
from mindwave_driver.common import *

from mindwave_msgs.msg import Mindwave

class MindwaveNode:
    def __init__(self):
        rospy.init_node('mindwave_node', anonymous=True)
        self.addr = rospy.get_param("~addr")
        print "Initializing node with addr ... ", self.addr
        #self.dev = rospy.get_param("~dev")
        #self.bauderate = rospy.get_param("~bauderate")

        if self.addr is not None:
            headset = BluetoothHeadset(self.addr)

        pub = rospy.Publisher('mindwave', MindwaveValues)
        
        rate = rospy.Rate(10)


    def update(self):
        msg = MindwaveValues()
            
        while not rospy.is_shutdown():
            if headset.status != Status.CONNECTED:
                print 'trying connecting'
                headset.connect(headset.addr)
                time.sleep(1)
            elif headset.status == Status.CONNECTED:
                print "quality signal: %s , attention %s , meditation %s , raw %s " % (headset.signal, headset.attention, \
                       headset.meditation, headset.raw_value)
                    # delta, theta, low-alpha, high-alpha, low-beta, high-beta,
                    # # low-gamma, high-gamma
                    # for e in headset.asig_eeg_power:
                    #   print ord(e),
                    # print ""
            else:
                pass

            msg.status = status
            msg.attention = headset.attention
            msg.meditation = headset.meditation
            msg.signal = headset.signal

            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MindwaveNode()       
        rospy.spin()
    except rospy.ROSInterruptException:
        pass