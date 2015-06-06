import threading, time
 
class DongleListener(threading.Thread):
    def __init__(self, parser, *args, **kwargs):
        self.parser = parser
        self.running = True
        super(DongleListener, self).__init__(*args, **kwargs)
    
    def run(self):
        while True:
            # listeng for incoming bytes 
         
            self.parser.listen2()
            time.sleep(0.1)
            print 'listener 1'

    def stop(self):
        self.running = False
        self._Thread__stop()

    #   *689400 jorge orreaga pacho
