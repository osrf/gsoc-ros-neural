import threading, time
from common import DEFAULT_BYTES

class DongleListener(threading.Thread):
    """This class represents a thread listener of the headset"""

    def __init__(self, parser, *args, **kwargs):
        """Initializes method's

        It uses the Parser class without a deamon

        Args:
            parser: the parser class 
        """

        self.parser = parser
        self.running = True
        super(DongleListener, self).__init__(*args, **kwargs)
    
    def run(self):
        """This method reads the incomming bytes in background

        It reads and parser the bytes from bluetooth 
        """

        while True:
            bytes = self.parser.stream.read(DEFAULT_BYTES)

            if bytes is not None:
                self.parser.parser(bytes)

            time.sleep(0.35)

    def stop(self):
        """This method stops the thread"""

        self.running = False
        self._Thread__stop()


# class Listener(object):
#     """This class represents a thread listener of the headset"""
 
#     def __init__(self, parser, interval=1, ):
#         """Initializes method's

#         It uses the Parser class with a deamon, sometimes it doesn't
#         work with the main ros loop.
#         """
#         self.interval = interval
#         self.parser = parser
#         self.running = True

#         thread = threading.Thread(target=self.run, args=())
#         thread.daemon = True # Daemonize thread
#         thread.start()       # Start the execution
 
#     def run(self):
#         """This method read the incomming bytes in background

#         It uses default bytes and parser the incomming data in a while
#         """        
#         while True:
#             # Do something
#             self.parser.listen2() 
#             time.sleep(self.interval)
