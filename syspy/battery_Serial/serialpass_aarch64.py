import sys,serial,fcntl,threading,subprocess
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
import message_battery_aarch64_pb2
class serialPassAarch64():
    def __init__(self):
        print("serialPassAarch64 start!")
        self.ser = None
        self.__callback = None
        self.__should_close = False
        self.__msg_thread = None

    def createSerial(self, name, baudrate):
        self.ser = serial.Serial(port=name, baudrate=baudrate, bytesize=8, parity='N', stopbits=1)
        command = "cat /etc/srcname"
        output = subprocess.check_output(command, shell=True)
        output = output.decode("utf-8").strip()
        print(output)
        if not output == 'SRC880':
            fcntl.ioctl(self.ser, 0)  # 这行决定了485模式
        self.__msg_thread = threading.Thread(target=self.__run, name="run")
        self.__msg_thread.start()  # FIXME: when to join?
        print('createSerial  name:{},baudrate:{}'.format(name, baudrate))

    def send(self, msg: list):
        self.ser.write(msg)

    def recv(self):
        data = self.ser.read()
        if not self.__callback is None:
            self.__callback(data)
    
    def setCallBack(self,handleData):
        if not handleData:
            print("Set callback error.It should be implemented the func 'handleData'")
        else:
            self.__callback = handleData

    def createBatteryMessage(self):
        return message_battery_aarch64_pb2.Message_Battery()

    def __run(self):
        try:
            while not self.__should_close:
                self.recv()
        except Exception as e:
            print("exception:", e)
        finally:
            self.ser.close()
            pass

    def stop(self):
        self.__should_close = True
        if self.__msg_thread:
            self.__msg_thread.join()

    def __del__(self):
        self.stop()

if __name__ == "__main__":
    pass
