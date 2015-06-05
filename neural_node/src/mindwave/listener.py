import threading, time
 
class DongleListener(threading.Thread):
    def __init__(self, parser, *args, **kwargs):
        self.parser = parser
        self.running = True
        super(DongleListener, self).__init__(*args, **kwargs)
    
    def run(self):
        while True:
            # listeng for incoming bytes 
            print 'listener 0'            
            self.parser.listen()
            time.sleep(0.5)
            print 'listener 1'

    def stop(self):
        self.running = False
        self._Thread__stop()
