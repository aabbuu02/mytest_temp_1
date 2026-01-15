import socket,time,struct,threading,sys
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
import CanFrame_pb2

# 定义类似 C++ 版本的 CommandTab
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

class udpClient:
    BUFFER_SIZE = 1000
    HEADER_SIZE = 4
    TIMEOUT = 2  # 超时时间设置

    def __init__(self):
        self._ip = "192.168.192.4"
        self._control_port = 15003
        self._receive_port = 5002
        self._config_mutex = threading.Lock()
        self._udp_socket = None
        self.__callback = None
        self._running = True  # 控制线程退出的标志
        self.__msg_thread = threading.Thread(target=self.receiveUDP, name="run")
        self.__msg_thread.start()
        self.setCallBack(self.handleData)
        print("udpClient run")

    def getCommandVersion(self, command):
        return command['CommandCount']

    def initReportUDPSocket(self):
        try:
            if self._udp_socket is None or self._udp_socket.fileno() == -1:
                # 如果socket已经关闭，重新创建并绑定
                self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._udp_socket.bind(("0.0.0.0", self._receive_port))
                self._udp_socket.settimeout(2)
                print(f"UDP Report Socket init success on port {self._receive_port}.")
            return True
        except Exception as e:
            print(f"Init Report UDP socket error: {e}")
            self.clearReportUDPSocket()
            return False

    def clearReportUDPSocket(self):
        if self._udp_socket:
            self._udp_socket.close()
            self._udp_socket = None

    def setCallBack(self, handleData):
        if not handleData:
            print("Set callback error. It should implement the 'handleData' function")
        else:
            self.__callback = handleData

    def receiveUDP(self):
        while self._running:
            if not self.initReportUDPSocket():
                print("reset")
                time.sleep(1)  # 重试延时
                continue
            try:
                udp_rcv_buffer, addr = self._udp_socket.recvfrom(self.BUFFER_SIZE)
                rpt_type = struct.unpack('I', udp_rcv_buffer[:self.HEADER_SIZE])[0]  # 获取包头 rptType
                pb_rcv_buffer = udp_rcv_buffer[self.HEADER_SIZE:]  # 跳过前4字节的报头
                if rpt_type == 20:  # 假设 rptType 20 代表 CAN 消息
                    msg = CanFrame_pb2.CanFrame()
                    msg.ParseFromString(pb_rcv_buffer)
                    self.__callback(msg)
            except socket.timeout:
                # 超时处理，继续接收
                continue
            except Exception as e:
                print(f"Error receiving or processing UDP packet: {e}")
            time.sleep(0.01)  # 轻微延时，避免 CPU 过载

    def sendCanMessage(self, channel, can_id, dlc, extend, can_string):
        can_data = self.split(can_string)
        msg = CanFrame_pb2.CanFrame()
        msg.Channel = channel
        msg.ID = can_id
        msg.DLC = dlc
        msg.Extended = extend
        msg.Data = bytes(can_data[:dlc])
        serialized_msg = msg.SerializeToString()
        command_version = self.getCommandVersion(CommandTab['sendPassThroughCanFrame'])
        header = struct.pack('I', command_version)
        full_message = header + serialized_msg
        print(f'message send: channel={channel}, can_id={hex(can_id)}, dlc={dlc}, extend={extend}, can_string={" ".join(can_string)}')
        self.sendConfigCmdNoReply(full_message, len(full_message), 'sendPassThroughCanFrame')

    def canPassThroughRxId(self, channel, id_nums, *can_id_list):
        if isinstance(can_id_list, int):
            can_id_list = (can_id_list,)
        CMD_HEAD = self.getCommandVersion(CommandTab['canPassThroughRxId'])
        buff = struct.pack('I', CMD_HEAD)  # 打包CMD_HEAD (4字节)
        buff += struct.pack('B', channel)  # 打包channel (1字节)
        buff += b'\x00' * 3  # 填充字节（保证对齐4字节）
        buff += struct.pack('I', id_nums)  # 打包id_nums (4字节)
        for can_id in can_id_list:
            buff += struct.pack('I', can_id)
        self.sendConfigCmdNoReply(buff, len(buff), "canPassThroughRxId")

    def createCan(self, channel:int, baudrate:int, fastmode:bool, terminal:bool):
        CMD_HEAD = self.getCommandVersion(CommandTab['createCan'])
        MSG_MAX = 8
        buff = struct.pack('I', CMD_HEAD) + b'\x00' * (MSG_MAX - 1) * 4
        buff = bytearray(buff)
        struct.pack_into('B', buff, 4, channel)  # 1 byte for channel
        struct.pack_into('I', buff, 5, baudrate)  # 4 bytes for baudrate (uint32)
        struct.pack_into('B', buff, 9, int(fastmode))  # 1 byte for fastmode (bool as 8-bit int)
        struct.pack_into('B', buff, 10,int(terminal))  # 1 byte for terminal (bool as 8-bit int)
        self.sendConfigCmdNoReply(buff, len(buff), "createCan")

    def sendConfigCmdNoReply(self, data, size, cmd_brief):
        try:
            self._udp_socket.sendto(data[:size], (self._ip, self._control_port))
            print(f"{cmd_brief}: success")
        except socket.error as e:
            print(f"{cmd_brief}: error - {str(e)}")

    def split(self, s, delimiters=" "):
        try:
            return [int(token, 16) for token in s.split(delimiters) if token]
        except ValueError as e:
            print(f"error: {e}")
            return []

    def close(self):
        print("Cleaning up udp resources...")
        self.clearReportUDPSocket()  # 在析构时关闭 UDP 套接字
        self._running = False
        self.__msg_thread.join()  # 等待线程结束

if __name__ == "__main__":
    pass

