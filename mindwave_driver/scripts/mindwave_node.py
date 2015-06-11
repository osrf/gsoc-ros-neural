import rospy
import mindwave
from mindwave_msg.msg import Mindwave

def mindwave_publisher():
    pub = rospy.Publisher('mindwave', MindwaveValues)
    print "Initializing node... "
    rospy.init_node('neural_node', anonymous=True)
    rate = rospy.Rate(10)

    headset = BlueetoothHeadset("20:68:9D:70:CA:96")

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