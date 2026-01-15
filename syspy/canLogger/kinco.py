import sys, os
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
import Receive, CanData, time, json
from datetime import datetime
#  from Recode2Log import ReceiveThread
import time
import threading

exit_event = threading.Event()

# for x86_64
class ReceiveThread():
    def __init__(self, dispContent):
        self.dispContent = dispContent
        self.udp_read = Receive.receiveByUdp()
        th = threading.Thread(target=self.run, args=())
        th.start()

    def run(self):
        while not exit_event.is_set():
            time.sleep(0.0001)
            rec_str = self.udp_read.read(self.dispContent)
            if rec_str != "":
                pass
            elif rec_str == "receive no data...........":
                self.udp_read.launchEthCanTool()

    def send(self, canid, data):
        newdata = 0x00
        for index in range(0, len(data)):
            newdata |= data[index] << (index * 8)
        #self.udp_read.send_frame(canid, newdata)
        self.udp_read.sendCanMessage(0, canid, 8, False, newdata)
        self.udp_read.sendCanMessage(1, canid, 8, False, newdata)
        #print("{:03X}\t{} {:08X}".format(canid, data, newdata))

# for arm64
class PyCanDump:
    def __init__(self, can_port, bus_type, cb):
        self.cb = cb
        import can, threading
        self.bus = []
        self.thread = []
        for port in can_port:
            print(port)
            canbus = can.interface.Bus(port, bustype=bus_type)
            self.bus.append(canbus)
            self.thread.append(threading.Thread(target=self.MsgCallback, args=(canbus,)))
        #  self.thread[0].setDaemon(True)
        for th in self.thread:
            th.start()

    def MsgCallback(self, canbus):
        import can
        with can.Logger(self.save_file_name) as logger:
            for msg in canbus:
                #  print(msg)
                self.cb(msg)

    def send(self, canid, data):
        msg = can.Message(
            arbitration_id=canid, data=data, is_extended_id=False
        )
        try:
            for bus in self.bus:
                bus.send(msg)
                #print(f"Message sent on {bus.channel_info}")
        except can.CanError:
            print("Message {} NOT sent".format(msg))

class SDO():
    def __init__(self, addr, subindex, nbyte, desc):
        self.addr = addr
        self.subindex = subindex
        self.addrall = self.addr << 8 | self.subindex
        self.nbyte = nbyte
        self.val = 0x00
        if 1 == nbyte:
            self.wCS = 0x2F
            self.CS = 0x4F
        elif 2 == nbyte:
            self.wCS = 0x2B
            self.CS = 0x4B
        elif 3 == nbyte:
            self.wCS = 0x27
            self.CS = 0x47
        elif 4 == nbyte:
            self.wCS = 0x23
            self.CS = 0x43
        else:
            self.wCS = 0x00
            self.CS = 0x00
        self.desc = desc
        self.updated = False

    def getQueryData(self):
        data = [0x00] * 8
        data[0] = 0x40
        data[1] = self.addr & 0xFF
        data[2] = (self.addr >> 8) & 0xFF
        data[3] = self.subindex
        return data

    def getConfigData(self, val):
        data = [0x00] * 8
        data[0] = self.wCS
        data[1] = self.addr & 0xFF
        data[2] = (self.addr >> 8) & 0xFF
        data[3] = self.subindex
        data[4] = (val >> 0) & 0xFF
        data[5] = (val >> 8) & 0xFF
        data[6] = (val >> 16) & 0xFF
        data[7] = (val >> 24) & 0xFF
        return data

    def decode(self, data):
        if data[0] == self.CS and (data[1] | data[2] << 8) == self.addr and data[3] == self.subindex:
            self.val = data[4] | data[5] << 8 | data[6] << 16 | data[7] << 24
            self.updated = True
            return True
        return False

    def updated(self):
        return self.updated

    def reset_updated(self):
        self.updated = False

    def getVal(self):
        return self.val

    def print(self):
        print("{:04x}\t{:02x}\t{:04x}\t{}".format(self.addr, self.subindex, self.val, self.desc))

class KincoApp():
    def __init__(self):
        self.node_set = False
        import re
        self.pattern = re.compile(r"(\d{2}:\d{2}:\d{2}\.\d{3})\s+\d+\s+(RX)\s+\d+\s+(0x[0-9A-Fa-f]+)\s+\[(\d+)\]\s*(.*)?")
        self.sdo_list = []
        self.sdo_list.append(SDO(0x6061, 0, 1, "real work mode"))
        self.sdo_list.append(SDO(0x6041, 0, 2, "status word"))
        self.sdo_list.append(SDO(0x6063, 0, 4, "actual pos"))
        self.sdo_list.append(SDO(0x606C, 0, 4, "actual speed"))
        self.sdo_list.append(SDO(0x6078, 0, 2, "actual current"))
        self.sdo_list.append(SDO(0x2680, 0, 2, "warn word"))
        self.sdo_list.append(SDO(0x6060, 0, 1, "req work mode"))
        self.sdo_list.append(SDO(0x6040, 0, 2, "control word"))
        self.sdo_list.append(SDO(0x2020, 0x0D, 1, "work mode 1"))
        self.sdo_list.append(SDO(0x2020, 0x0E, 1, "work mode 2"))
        self.sdo_list.append(SDO(0x100C, 0, 2, "protect period"))
        self.sdo_list.append(SDO(0x100d, 0, 1, "protect coff"))
        self.sdo_list.append(SDO(0x6007, 0, 2, "comm int mode"))
        self.sdo_list.append(SDO(0x2010, 0x03, 2, "DIN1 func"))
        self.sdo_list.append(SDO(0x2010, 0x04, 2, "DIN2 func"))
        self.sdo_list.append(SDO(0x2010, 0x05, 2, "DIN3 func"))
        self.sdo_list.append(SDO(0x2010, 0x06, 2, "DIN4 func"))
        self.sdo_list.append(SDO(0x2010, 0x09, 2, "DIN7 func"))
        self.sdo_list.append(SDO(0x2010, 0x0B, 2, "DIN actual status"))
        self.sdo_dict = { sdo.addrall:sdo for sdo in self.sdo_list }
        #for a,b in self.sdo_dict.items():
        #    print("0x{:06X} {}".format(a, b))

        srcname = "unknown"
        with open('/etc/srcname', 'r') as file:
            srcname = file.read()
        print(srcname)
        self.can_api = None
        if "SRC2000" in srcname:
            self.can_api = ReceiveThread(self.dispContent)
        elif "SRC880" in srcname or "SRC600" in srcname:
            import can
            self.can_api = PyCanDump(["can0", "can1"], "socketcan", self.recv_cb)

        self.req_thread = threading.Thread(target=self.req, args=())
        #self.show_thread = threading.Thread(target=self.show, args=())
        self.req_thread.setDaemon(True)
        self.req_thread.start()
        #self.show_thread.start()
    def setNode(self, node):
        self.node_id = node
        self.node_set = True

    def find_sdo_by_addr(self, addr):
        return self.sdo_dict.get(addr, None)

    def req(self):
        while  not exit_event.is_set():
            if not self.node_set:
                continue
            for sdo in self.sdo_list:
                self.can_api.send(0x600 + self.node_id, sdo.getQueryData())
                time.sleep(0.005)
            time.sleep(0.5)

    def wait_updated_timeout(self, sdo, timeout):
        start_time = time.time()
        while True:
            if True == sdo.updated:
                return True
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                return False
            time.sleep(0.01)

    def save_and_reboot(self):
        if not self.node_set:
            print("node not set yet.")
        print("saving ...")
        # save
        self.can_api.send(0x600+self.node_id, [0x2F, 0xF0, 0x2F, 0x01, 0x01, 0x00, 0x00, 0x00])
        time.sleep(5.0)
        # reboot
        print("reboot ...")
        self.can_api.send(0x600+self.node_id, [0x2B, 0xFF, 0x2F, 0x00, 0x55, 0xAA, 0x00, 0x00])
        time.sleep(5.0)

    def Fix0x201004_DIN2(self):
        if not self.node_set:
            print("node not set yet.")
            return False
        sdo_din2_func = self.find_sdo_by_addr(0x201004)
        if not self.wait_updated_timeout(sdo_din2_func, 2000):
            print("0x201004 updated timeout")
            return False
        print("0x201004 updated.")
        if sdo_din2_func.getVal() == 0x04: # control mode
            print("0x201004: now is 0x{:02x}".format(sdo_din2_func.getVal()))
            print("0x201004: reset to 0x{:02x}".format(0x00))
            sdo_din2_func.reset_updated() # clear updated flag
            self.can_api.send(0x600 + self.node_id, sdo_din2_func.getConfigData(0x00))
            sdo_din2_func.reset_updated() # clear updated flag
            time.sleep(0.10)
            if self.wait_updated_timeout(sdo_din2_func, 5000):
                print("0x201004: after config, now is 0x{:02x}".format(sdo_din2_func.getVal()))
                self.save_and_reboot()
                sdo_din2_func.reset_updated() # clear updated flag
                if self.wait_updated_timeout(sdo_din2_func, 5000):
                    print("0x201004: after reboot, now is 0x{:02x}".format(sdo_din2_func.getVal()))
                    return True
                else:
                    print("0x201004: updated timeout after reboot")
                    return False
            else:
                print("0x201004: updated timeout after config")
                return False
        else:
            print("0x201004: is 0x{:02x}, no need to clear".format(sdo_din2_func.getVal()))
            return False

    def Fix0x20200E_workmode1(self):
        if not self.node_set:
            print("node not set yet.")
        sdo = self.find_sdo_by_addr(0x20200E)
        if not self.wait_updated_timeout(sdo, 2000):
            print("0x20200E updated timeout")
            return False
        if sdo.getVal() == 0xFFFFFFFC: # -4 torque mode
            print("0x20200E: now is 0x{:08x}".format(sdo.getVal()))
            print("0x20200E: reset to 0x{:08x}".format(0xFFFFFFFD))
            self.can_api.send(0x600 + self.node_id, sdo.getConfigData(0xFFFFFFFD))
            sdo.reset_updated() # clear updated flag
            if self.wait_updated_timeout(sdo, 3000):
                print("0x20200E: after config, now is 0x{:08x}".format(sdo.getVal()))
                self.save_and_reboot()
                sdo.reset_updated() # clear updated flag
                if self.wait_updated_timeout(sdo, 5000):
                    print("0x20200E: after reboot, now is 0x{:08x}".format(sdo.getVal()))
                    return True
                else:
                    print("0x20200E: updated timeout after reboot")
                    return False
            else:
                print("0x20200E: updated timeout after config")
                return False
        else:
            print("0x20200E: is 0x{:08x}, no need to config".format(sdo.getVal()))
            return False

    def Show(self):
        #print("\033[H\033[J", end="") # clear screen
        print("{}\t{}\t{}\t{}".format("addr", "subaddr", "value", "description"))
        print("-------------------------------------------------")
        for sdo in self.sdo_list:
            sdo.print()
        if not self.node_set:
            print("node not set yet.")

    def show(self):
        import os
        while  not exit_event.is_set():
            #os.system('clear')
            print("\033[H\033[J", end="") # clear screen
            print("{}\t{}\t{}\t{}".format("addr", "subaddr", "value", "description"))
            print("-------------------------------------------------")
            for sdo in self.sdo_list:
                sdo.print()
            time.sleep(0.2)

    def process_msg(self, canid, dlc, data):
        #  print(canid, dlc, data)
        #  if dlc == 0:
        #      dlc = 8
        #      data = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        #  if dlc == 8:
        if not self.node_set:
            return
        if canid == 0x580 + self.node_id and dlc == 8:
            for sdo in self.sdo_list:
                sdo.decode(data)

    def recv_cb(self, msg):
        self.process_msg(msg.arbitration_id, len(msg.data), msg.data)

    def dispContent(self, a):
        line=a.replace("\r", "").replace("\n", "")
        match = self.pattern.match(line)
        if match:
            timestamp = match.group(1)
            message_id = match.group(3)
            data_length = int(match.group(4))
            data_bytes = match.group(5).strip()

            if data_bytes:
                data_bytes = [int(byte.strip(), 16) for byte in data_bytes.split()]
            else:
                data_bytes = []

            arbtid=int(message_id, 16) # 将十六进制字符串转换为整数
            data=data_bytes
            #is_extended_id=False
            #  print("ID: {} DLC:{} data:{}".format(arbtid, len(data), data))
            self.process_msg(arbtid, len(data), data)

def help():
    print("!!! Please set node first !!!")
    print("!!! Please set node first !!!")
    print("!!! Please set node first !!!")
    print("command:")
    print("\t H/h/help: show this content")
    print("\t show: show dict value")
    print("\t node: set node ID")
    print("\t fix1: auto fix 0x201004 DIN2 func config: from WorkModeControl to None")
    print("\t fix2: auto fix 0x20200E work mode 1: from -4 to -3")
    print("\t autofix: auto fix 0x20200E work mode 1 and 0x201004 DIN2 func config")

if __name__ == '__main__':
    app = KincoApp()

    if sys.version > '3':
        get_input = input
    else:
        get_input = raw_input

    while  not exit_event.is_set():
        s = get_input(">>")
        try:
            if s[:4] == "show":
                app.Show()
            elif s[:4] == "help" or s[:1] == "h" or s[:1] == "H":
                help()
            elif s[:4] == "fix1":
                if not app.Fix0x201004_DIN2():
                    print("fix 0x201004 DIN2 failed.")
                else:
                    print("fix 0x201004 DIN2 success.")
            elif s[:4] == "fix2":
                if not app.Fix0x20200E_workmode1():
                    print("fix 0x20200E work mode 1 failed.")
                else:
                    print("fix 0x20200E work mode 1 success.")
            elif s[:7] == "autofix":
                if not app.Fix0x20200E_workmode1():
                    if not app.Fix0x201004_DIN2():
                        print("fix 0x20200E work mode 1 and 0x201004 DIN2 BOTH failed.")
                    else:
                        print("fix 0x20200E work mode 1 falied BUT fix 0x201004 DIN2 success.")
                else:
                    print("fix 0x20200E work mode 1 success.")
            elif s[:4] == "node":
                arr = s.split(" ")
                print("set node id to {}".format(int(arr[1])))
                app.setNode(int(arr[1]))
            elif s[:4] == "exit" or s[:4] == "quit" or s[:1] == 'q':
                exit_event.set()
                sys.exit()
        except Exception as ex:
            print("Incorrect input format.")
            print(ex)
