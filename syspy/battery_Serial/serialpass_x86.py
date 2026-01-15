import sys
import syspy.lib.pass_through as pt
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
DEFAULT_PASS_ADDR = "ipc:///tmp/python2dsp_udp.ipc"
import message_battery_pb2

class serialPassX86():
    def __init__(self):
        print("serialPassx86 start!")
        self.__pass = pt.passThrough()
        self.__pass.serialConnect(DEFAULT_PASS_ADDR)

    def createBatteryMessage(self):
        return message_battery_pb2.Message_Battery()

    def send(self, msg:list):
        if(isinstance(msg, list)):
            self.__pass.send(bytes(msg))
        else:
            print("Write msg format error. please send a list")
    
    def setCallBack(self, handleData):
        if not handleData:
            print("Set callback error.It should be implemented the func 'handleData'")
        else:
            self.__pass.setCallBack(handleData)

    def shutdown(self):
        try:
            if self.__pass:
                self.__pass.shoutDown()  # Assuming typo in original code is fixed here
        except Exception as e:
            print(f"Failed to shutdown properly: {e}")

    def __del__(self):
        self.shutdown()

if __name__ == "__main__":
    pass
