import threading, zmq, time, sys
import syspy.lib.udp_debug as ud

class callBack:
    def handleData(self, msg):
        pass

class passThrough:
    def __init__(self):
        self.context = zmq.Context()
        self.__client_sock = self.context.socket(zmq.DEALER)
        self.__addr = ""
        self.__conn_id = ""
        self.__msg_thread = None
        self.__should_close = False
        self.__callback = None
        self.__lock = threading.Lock()  # 创建锁对象
        print("passThrough start")

    def close(self):
        print("close the socket")
        self.socket.close()

    def setCallBack(self, callback):
        self.__callback = callback

    def serialConnect(self, addr):
        self.__addr = addr
        now = time.time()
        self.__conn_id = "py_client_" + str(now)
        self.__msg_thread = threading.Thread(target=self.__run, name="run")
        self.__msg_thread.start()  # FIXME: when to join?

    def canConnect(self, addr, connid):
        self.__addr = addr
        self.__conn_id = connid
        self.__msg_thread = threading.Thread(target=self.__run, name="run")
        self.__msg_thread.start()  # FIXME: when to join?

    def __run(self):
        identity = self.__conn_id
        self.__client_sock.identity = identity.encode("utf8")
        self.__client_sock.connect(self.__addr)
        poll = zmq.Poller()
        poll.register(self.__client_sock, zmq.POLLIN)
        try:
            while not self.__should_close:
                sockets = dict(poll.poll(10))
                if self.__client_sock in sockets:
                    self.__receive()
        except Exception as e:
            print("passThrough exception:", e)

        finally:
            pass

    def __receive(self):
        with self.__lock:
            msg = self.__client_sock.recv()
            if not self.__callback is None:
                self.__callback(msg)

    def send(self, data):
        with self.__lock:
            self.__client_sock.send(data)

    def shoutDown(self):
        self.__should_close = True


if __name__ == "__main__":
    pass