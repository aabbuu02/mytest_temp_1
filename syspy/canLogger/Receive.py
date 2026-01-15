import CanFrame_pb2
import socket, struct, time, os, sys
import CanData as can
from datetime import datetime
CommandTab = {
    'sendPassThroughCanFrame': {
        'name': 'sendPassThroughCanFrameCommand',
        'CommandCount': 0x00001089,
        'start_version': 0x109000,
        'end_version': 0xffffffff,
        'with_return': False
    },
    'canPassThroughRxId': {
        'name': 'canPassThroughRxIdCommand',
        'CommandCount': 0x00001090,
        'start_version': 0x109000,
        'end_version': 0xffffffff,
        'with_return': False
    },
    'createCan': {
        'name': 'createCanCommand',
        'CommandCount': 0x00001092,
        'start_version': 0x109000,
        'end_version': 0xffffffff,
        'with_return': False
    }
}

class receiveBase(object):
    def __init__(self):
        self.data_que = []
    def pushData(self, rx_data):
        if isinstance(rx_data, can.rxCanData):
            self.data_que.append(rx_data)
        else:
            print("push data type error %s %s" % (type(rx_data),type(can.txCanData)))
    def read(self):
        pass

class receiveByUdp(receiveBase):
    def __init__(self):
        super(receiveByUdp, self).__init__()
        self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.so.settimeout(2)
        self.launchEthCanTool()
        self.msgdict = {}
        self.framerate = (0, 0)
        self.remoteaddr = ""
        self.data = ""
        self.launchEthCanTool()
        self.queryForFrame(50)
        self.count = 0
        self.use_usrstamp = False

    def setUserTimestamp(self, status):
        self.use_usrstamp = status

    def getCommandVersion(self, command):
        return command['CommandCount']

    def sendConfigCmdNoReply(self, data, size, cmd_brief):
        try:
            self.so.sendto(data[:size], ('192.168.192.4', 15003))
            #print(f"{cmd_brief}: success")
        except socket.error as e:
            print(f"{cmd_brief}: error - {str(e)}")

    def sendCanMessage(self, channel, can_id, dlc, extend, can_data):
        msg = CanFrame_pb2.CanFrame()
        msg.Channel = channel
        msg.ID = can_id
        msg.DLC = dlc
        msg.Extended = extend
        #msg.Data = bytes(can_data[:dlc])
        msg.Data = can_data.to_bytes(8, byteorder='little')
        serialized_msg = msg.SerializeToString()
        command_version = self.getCommandVersion(CommandTab['sendPassThroughCanFrame'])
        header = struct.pack('I', command_version)
        full_message = header + serialized_msg
        self.sendConfigCmdNoReply(full_message, len(full_message), 'sendPassThroughCanFrame')

    def launchEthCanTool(self):
        self.so.sendto(struct.pack('<2I', 0x1016, 0xffffffff), ('192.168.192.4', 15003))
        try:
            (self.data, self.remoteaddr) = self.so.recvfrom(1024)
        except socket.timeout:
            pass
        self.so.sendto(struct.pack('<2I', 0x1018, 0xffffffff), ('192.168.192.4', 15003))
        try:
            (self.data, self.remoteaddr) = self.so.recvfrom(1024)
        except socket.timeout:
            pass

    def queryForFrame(self, num):
        fullMsg = struct.pack('<2I', 0x1019, num)
        self.so.sendto(fullMsg, ('192.168.192.4', 15003))

    def read(self, print_callback):
        try:
            (self.data, self.remoteaddr) = self.so.recvfrom(1024)
        except socket.timeout:
            self.launchEthCanTool()
            return 'receive no data...........'
        (msgId, pbdata) = struct.unpack('<I' + str(len(self.data) - 4) + 's', self.data)
        frame = CanFrame_pb2.CanFrame()
        update = False
        if(0x00001019 == msgId):
            try:
                frame.ParseFromString(pbdata)
            except:
                print('pbdata parse error')
                print(self.data)
                os.system('pause')
            self.msgdict[frame.ID] = frame
            update = True
            self.count = self.count + 1
        elif 0x00001041 == msgId:
            self.framerate = struct.unpack('<2I', pbdata)
        if(update):
            frametype = 'D'
            if frame.Remote:
                frametype = "R"
            isExtended = False
            if frame.Extended:
                isExtended = True
            dirc = ''
            if(self.msgdict[frame.ID].Direction == 1):
                dirc = 'TX'
            else:
                dirc = 'RX'
            if isExtended:
                tmps = '%s\t%d\t%s\t %d 0x%08X [%d]  ' % (
                        datetime.strftime(datetime.now(), '%H:%M:%S.%f')[0:-3],
                        self.count,
                        dirc,
                        self.msgdict[frame.ID].Channel,
                        self.msgdict[frame.ID].ID,
                        self.msgdict[frame.ID].DLC
                        )
            else:
                tmps = '%s\t%d\t%s\t %d 0x%03X [%d]  ' % (
                        datetime.strftime(datetime.now(), '%H:%M:%S.%f')[0:-3],
                        self.count,
                        dirc,
                        self.msgdict[frame.ID].Channel,
                        self.msgdict[frame.ID].ID,
                        self.msgdict[frame.ID].DLC
                        )

            for i in range(0, self.msgdict[frame.ID].DLC):
                tmps += ('%02X ' % self.msgdict[frame.ID].Data[i])
            tmps += "\n"
            print_callback(tmps)
            return tmps
        else:
            return ""

if __name__ == "__main__":
    test = receiveByUdp()
    while True:
        test.read()
