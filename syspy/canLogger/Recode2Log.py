import sys, os
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
import Receive, CanData, time, json
from datetime import datetime

# for x86_64
class ReceiveThread():
    def __init__(self, dispContent):
        self.dispContent = dispContent
        self.udp_read = Receive.receiveByUdp()
        self.run()

    def run(self):
        while True:
            time.sleep(0.0001)
            rec_str = self.udp_read.read(self.dispContent)
            if rec_str != "":
                pass
            elif rec_str == "receive no data...........":
                self.udp_read.launchEthCanTool()

# for arm64
class PyCanLogger:
    def __init__(self, can_port, bus_type, save_file_name):
        import can, threading
        self.save_file_name = save_file_name
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
                logger.on_message_received(msg)

class Recode2File():
    def __init__(self, save_file_name):
        print("save file name:%s" % (save_file_name))
        self.file = open(save_file_name, 'a')
        if self.file:
            print("open file %s success" % (save_file_name))
        else:
            print("open file %s failed" % (save_file_name))

        srcname = "unknown"
        with open('/etc/srcname', 'r') as file:
            srcname = file.read()
        print(srcname)
        if "SRC2000" in srcname:
            self.serialThread = ReceiveThread(self.dispContent)
        elif "SRC880" in srcname or "SRC600" in srcname:
            import can
            import threading
            self.logger = PyCanLogger(["can0", "can1"], "socketcan", save_file_name)

    def dispContent(self, argvStr):
        self.file.write(argvStr)
        pass

if __name__ == '__main__':
    suff = "_trigger_unknown"
    if len(sys.argv) == 2:
        suff = "_trigger_" + sys.argv[1]
    file_str = datetime.strftime(datetime.now(), 'can_%Y-%m-%d_%H-%M-%S.%f').replace(" ","")
    save_fime_name = "/usr/local/etc/.SeerRobotics/rbk/diagnosis/log/" + file_str + suff + ".log"
    app = Recode2File(save_fime_name)
    app.file.close()
