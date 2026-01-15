import sys, os
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
import Receive, CanData, time, json
from datetime import datetime
from Recode2Log import ReceiveThread

# for arm64
class PyCanDump:
    def __init__(self, can_port, bus_type):
        import can, threading
        self.bus = []
        self.thread = []
        for port in can_port:
            print(port)
            canbus = can.interface.Bus(port, bustype=bus_type)
            self.bus.append(canbus)
            self.thread.append(threading.Thread(target=self.MsgCallback, args=(canbus,)))
        self.thread[0].setDaemon(True)
        for th in self.thread:
            th.start()

    def MsgCallback(self, canbus):
        import can
        with can.Logger(self.save_file_name) as logger:
            for msg in canbus:
                print(msg)

class CandumpApp():
    def __init__(self):
        srcname = "unknown"
        with open('/etc/srcname', 'r') as file:
            srcname = file.read()
        print(srcname)
        if "SRC2000" in srcname:
            self.serialThread = ReceiveThread(self.dispContent)
        elif "SRC880" in srcname or "SRC600" in srcname:
            import can
            import threading
            self.logger = PyCanDump(["can0", "can1"], "socketcan")

    def dispContent(self, a):
        print(a.replace("\r", "").replace("\n", ""))
        pass

if __name__ == '__main__':
    app = CandumpApp()
    app.file.close()
