from parser import Parser
from listener import DongleListener

class Headset:
    """The main class of any version of headset

    It has common values for both versions

    Args:
        id: the id of headset
    """

    def __init__(self, headset_id=None):
        
        if headset_id:
            self.id = headset_id
            self.auto_connect = True
        else:
            self.id = headset_id
            self.auto_connect = False

        self.stream = None
        self.meditation = 0 # 0-100
        self.attention = 0 # 0-100
        self.signal = 0
        self.blink = 0
        self.status = None

        self.raw_value = 0
        self.asig_eeg_power = []
        
    def run(self, stream):  # Stream class
        """This method creates a Listener as a daemon."""

        self.stream = stream

        if self.stream is not None:
            self.parser = Parser(self, self.stream)
            self.listener = DongleListener(self.parser)

        if not self.listener.isAlive():
            self.listener.daemon = True
            self.listener.start()

    def close(self):
        """This method closes the stream."""

        self.stream.close()
        
            