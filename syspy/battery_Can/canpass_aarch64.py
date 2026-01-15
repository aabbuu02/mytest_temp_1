import sys
import can
import threading
import subprocess
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/genetic/syspy/protobuf')
sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
import message_battery_aarch64_pb2

class canPassAarch64():
    def __init__(self):
        print("canPassAarch64 start!")
        self.__callback = None
        self.__should_close = threading.Event()
        
        self.bus_dict = {}
        self.threads = []

    def setCallBack(self, handleData):
        if callable(handleData):
            self.__callback = handleData
        else:
            print("Set callback error.")

    def createBatteryMessage(self):
        return message_battery_aarch64_pb2.Message_Battery()

    def createCanBus(self, channel, bitrate):
        try:
            
            bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate, receive_own_messages=False)
            self.bus_dict[channel] = bus
            print(f"Successfully created bus for channel: {channel}")
            
            
            msg_thread = threading.Thread(target=self.__run, args=(bus,), daemon=True)
            msg_thread.start()
            self.threads.append(msg_thread)
        except Exception as e:
            print(f"Error creating CAN bus for channel {channel}: {e}")

    def __run(self, bus):
        """Receives messages from a specific bus instance."""
        try:
            while not self.__should_close.is_set():
                msg = bus.recv(timeout=1.0)
                if msg is not None and self.__callback is not None:
                    self.__callback(msg)
        except Exception as e:
            print(f"Exception in receive thread for bus {bus.channel_info}: {e}")

    def attachCanID(self, *canid):
        filters = []
        for id_ in canid:
            if id_ < 0x800: 
                can_mask = 0x7FF
            else: 
                can_mask = 0x1FFFFFFF
            filters.append({"can_id": id_, "can_mask": can_mask})
        
        
        if not self.bus_dict:
            print("Warning: No CAN buses created to attach IDs to.")
            return

        print('Attaching CAN IDs:', ' '.join(hex(id_) for id_ in canid))
        for channel, bus in self.bus_dict.items():
            bus.set_filters(filters)
            print(f"Filters set for channel: {channel}")

    def sendCanframe(self, channel, can_id, dlc, extend, can_string: list):
        
        bus = self.bus_dict.get(channel)
        if not bus:
            print(f"Error: CAN bus for channel '{channel}' not found. Please create it first.")
            return
            
        try:
            
            message = can.Message(arbitration_id=can_id, data=can_string, is_extended_id=extend, dlc=dlc)
            bus.send(message)
            hex_can_string = ' '.join(format(byte, '02X') for byte in can_string)
            
            print(f'Message sent: channel={channel}, can_id={hex(can_id)}, dlc={dlc}, can_string={hex_can_string}')
        except Exception as e:
            print(f"Error sending frame on channel {channel}: {e}")

    def close(self):
        self.__should_close.set()
        print("Shutting down CAN buses...")
        for bus in self.bus_dict.values():
            bus.shutdown()
        print("CAN buses shut down.")

    def __del__(self):
        self.close()

if __name__ == "__main__":
    pass