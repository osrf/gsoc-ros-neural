import rospy
from neural_node.msg import MindwaveValues

from mindwave.wireless_headset import WirelessHeadset
from mindwave.common import status

headset = WirelessHeadset('/dev/ttyUSB0','7B04')

def mindwave_publisher():
    pub = rospy.Publisher('mindwave', MindwaveValues)
    rospy.init_node('neural_node', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if headset.status == Status.STANDBY:     
            print "trying connecting"
            headset.connect()
            time.sleep(2)
        elif headset.status == Status.CONNECTED:
            msg = MindwaveValues()
            msg.status = status
            msg.attention = headset.attention
            msg.meditation = headset.meditation
            msg.signal = headset.signal

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        mindwave_publisher()
    except rospy.ROSInterruptException:
        pass