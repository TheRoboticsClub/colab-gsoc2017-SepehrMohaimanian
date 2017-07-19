import traceback
import jderobot
import threading
import Ice
from .threadSensor import ThreadSensor

class GazeboActionIceClient:
    def __init__(self, ic, prefix):
        self.lock = threading.Lock()
        prop = ic.getProperties()

        try:
            base = ic.propertyToProxy(prefix+".Proxy")
            self.proxy = jderobot.GazeboActionsPrx.checkedCast(base)

            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)

    def start(self):
        pass

    def stop(self):
        pass
        
    def hasproxy (self):
        return hasattr(self,"proxy") and self.proxy

    def sendReset(self):
        if self.hasproxy():
            self.lock.acquire()
            self.proxy.resetGazebo()
            self.lock.release()
