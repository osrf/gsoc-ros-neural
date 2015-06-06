from parser import Parser
from listener import DongleListener

class Headset:

    def __init__(self, headset_id=None, version=None):
        
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
        self.status = None

    def run(self, stream):  # Stream class
        self.stream = stream

        if self.stream is not None:
            self.parser = Parser(self, self.stream)
            self.listener = DongleListener(self.parser)

        if not self.listener.isAlive():
            self.listener.daemon = True
            self.listener.start()

    def connect(self):
        pass
    def disconnect(self):
        pass