import sys
from google.protobuf.json_format import MessageToJson, Parse
import syspy.lib.udp_debug as ud

sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
DEFAULT_RPC_ADDR = "ipc:///tmp/python2dsp_dmx512.ipc"

import message_dmx512_pb2
import message_movetask_pb2
import message_battery_pb2
import message_navigation_pb2
import message_controller_pb2

class dmx512X86():
    def __init__(self,rpc_client):
        self.rpc_client = rpc_client
        self.__debug_out = ud.udpDebug()
        sys.stdout = self.__debug_out
        print("start dmx512")


    def sendDmx512(self, dmx512_info):
        type_exm = message_dmx512_pb2.Message_Dmx512()
        if (isinstance(dmx512_info, type(type_exm))):
            msg = MessageToJson(dmx512_info)
            self.rpc_client.sendX86DmxInfo(msg)

    def recMoveStatus(self):
        str = self.rpc_client.getMoveStatus()
        movestatus = Parse(str, message_movetask_pb2.Message_MoveStatus())
        return movestatus

    def recBattery(self):
        str = self.rpc_client.getBatterToPython()
        batter_ = Parse(str, message_battery_pb2.Message_Battery())
        return batter_

    def recRobotSpeed(self):
        str = self.rpc_client.getNavSpeed()
        robotSpeed = Parse(str, message_navigation_pb2.Message_NavSpeed())
        return robotSpeed

    def recControllerMsg(self):
        str = self.rpc_client.getController()
        controllerMsg = Parse(str, message_controller_pb2.Message_Controller())
        return controllerMsg

    def createDmx512Message(self):
        return message_dmx512_pb2.Message_Dmx512()

    def createMoveStatusMessage(self):
        return message_movetask_pb2.Message_MoveStatus()

    def createBatteryMessage(self):
        return message_battery_pb2.Message_Battery()

    def createNavSpeedMessage(self):
        return message_navigation_pb2.Message_NavSpeed()

    def __del__(self):
        self.rpc_client.close()

if __name__ == "__main__":
    pass