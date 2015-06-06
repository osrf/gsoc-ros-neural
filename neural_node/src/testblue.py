from mindwave.bluetooth_headset import BluetoothHeadset
from mindwave.common import *

# 20:68:9D:70:CA:96
#headset = BluetoothHeadset("20:68:9D:70:CA:96")
headset = BluetoothHeadset()

#headset.echo_raw()
headset.parser()
