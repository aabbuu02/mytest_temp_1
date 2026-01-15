import sys
import syspy.lib.pass_through as pt
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
DEFAULT_PASS_ADDR = "ipc:///tmp/CanPass_udp.ipc"
import message_battery_pb2
import CanFrame_pb2

class canPassX86():
    def __init__(self, rpc_client):
        print("canPassX86 start!")
        self.rpc_client = rpc_client
        self.__pass = pt.passThrough()
        self.__pass.canConnect(DEFAULT_PASS_ADDR,"ECanFrame_pass_py")

    def setCallBack(self, handleData):
        if not handleData:
            print("Set callback error.It should be implemented the func 'handleData'")
        else:
            self.__pass.setCallBack(handleData)

    def createBatteryMessage(self):
        return message_battery_pb2.Message_Battery()

    def recCanframe(self,msg):
        """
        接收信息并转化为can类型
        """
        rec_canframe = CanFrame_pb2.CanFrame()
        rec_canframe.ParseFromString(msg)
        return rec_canframe

    def sendCanframe(self, channel, can_id, dlc, extend, can_string):
        print(f'message send: channel={channel}, can_id={hex(can_id)}, dlc={dlc}, extend={extend}, can_string={can_string}')
        self.rpc_client.sendPassThroughCanFrame(channel, can_id, dlc, extend, can_string)

    def attachCanID(self, channel, id_nums, *canid):
        can_ids = []
        for i in range(min(len(canid), 5)):
            can_ids.append(canid[i])
        can_id1, can_id2, can_id3, can_id4, can_id5 = can_ids + [0] * (5 - len(can_ids))
        print(f'channel{channel},id_nums{id_nums},can_id1{hex(can_id1)},can_id2{hex(can_id2)},can_id3{hex(can_id3)}')
        self.rpc_client.canPassThroughRxId(channel, id_nums, can_id1, can_id2, can_id3, can_id4, can_id5)
        print('Attached CAN IDs:', end=' ')
        for id_ in can_ids:
            print(hex(id_), end=' ')
        print()

    def __del__(self):
        if self.__pass:
            try:
                self.__pass.shoutDown()
            except Exception as e:
                print(f"Error shutting down passThrough: {e}")

if __name__ == "__main__":
    pass

