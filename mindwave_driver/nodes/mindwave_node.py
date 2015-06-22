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
            self.headset = BluetoothHeadset(self.addr)
            
        self.pub = rospy.Publisher('mindwave', Mindwave, queue_size=10)
        
        self.loop_rate = rospy.Rate(2) #hz  T = 1/f


    def update(self):
               
        while not rospy.is_shutdown():
            msg = Mindwave()
            
            if self.headset.status != Status.CONNECTED:
                rospy.loginfo('trying connecting')
                self.headset.connect(headset.addr)
                rospy.sleep(1)             
            else:
                pass
            
            msg.status = self.headset.status
            msg.signal = self.headset.signal
            msg.attention = self.headset.attention
            msg.meditation = self.headset.meditation
      
            #rospy.loginfo(msg)
            self.pub.publish(msg)
            self.loop_rate.sleep()
            #rospy.sleep(0.5)
            #rospy.spin()
                
        self.headset.close()

if __name__ == '__main__':
    try:
        node = MindwaveNode()
        node.update()       
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass