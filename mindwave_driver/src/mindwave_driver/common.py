
# global variables 
DEFAULT_BYTES = 500

class Bytes:
    SYNC            = '\xaa'
    POOR_SIGNAL     = '\x02'
    ATTENTION       = '\x04'
    MEDITATION      = '\x05'
    BLINK           = '\x16'
    #multibytes
    RAW_VALUE       = '\x80'
    ASIC_EEG_POWER  = '\x83'
    
class BytesStatus:
    RESPONSE_CONNECTED      = '\xd0'
    RESPONSE_NOFOUND        = '\xd1'
    RESPONSE_DISCONNECTED   = '\xd2'
    RESPONSE_REQUESTDENIED  = '\xd3'
    RESPONSE_STANDBY        = '\xd4'
    # connection status
    CONNECT       = '\xc0'
    DISCONNECT    = '\xc1'
    AUTOCONNECT   = '\xc2'

class Status:
    CONNECTED       = "connected"
    NOFOUND         = "not found"
    DISCONNECTED    = "disconnected"
    DENIED          = "denied"
    STANDBY         = "scanning"

class Version:
    MINDWAVE = "MindWave"
    MINDWAVE_MOBILE = "MindWave Mobile"

