import threading, time
from common import DEFAULT_BYTES

class DongleListener(threading.Thread):
    def __init__(self, parser, *args, **kwargs):
        self.parser = parser
        self.running = True
        super(DongleListener, self).__init__(*args, **kwargs)
    
    def run(self):
        while True:
            # listeng for incoming bytes 
            bytes = self.parser.stream.read(DEFAULT_BYTES)
            #self.parser.print_bytes(bytes)
            self.parser.parser(bytes)
            time.sleep(0.25)

    def stop(self):
        self.running = False
        self._Thread__stop()


class Listener(object):
 
    def __init__(self, parser, interval=1, ):
        self.interval = interval
        self.parser = parser
        self.running = True

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True # Daemonize thread
        thread.start()       # Start the execution
 
    def run(self):
        """ Method that runs forever """
        while True:
            # Do something
            self.parser.listen2() 
            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self._Thread__stop()

        #   *689400 jorge orreaga pacho
