import zmq,json,threading,sys

class zmqServer(object):
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("ipc:///tmp/dsp2python_rpc.ipc")
        self.data = None
        self.__should_close = threading.Event()  # 线程关闭标志
        self.__lock = threading.Lock()  # 创建锁对象
        self.msg_thread = threading.Thread(target=self.__loop, name="loop")
        self.msg_thread.start()


    def close(self):
        print("close the socket")
        self.__should_close.set()  # 通知线程关闭
        self.socket.close()  # 关闭 socket 会终止 recv 的阻塞状态
        self.context.term()  # 终止 context
        self.msg_thread.join()  # 等待线程结束

    def send(self, data):
        with self.__lock:
            self.socket.send(data)

    def recv(self):
        with self.__lock:
            return self.socket.recv()

    def __loop(self):
        while not self.__should_close.is_set():
            try:
                message = self.socket.recv()  # 阻塞等待消息
                self.data = json.loads(message.decode('utf-8'))
                method_name = self.data['method_name']
                print(f'server: {method_name}')
                res = self.funs[method_name]()
                data = {"res": res}
                self.socket.send(json.dumps(data).encode('utf-8'))
            except zmq.ZMQError as e:
                if self.__should_close.is_set():
                    break  # 关闭线程时会触发 ZMQError，结束循环
                print('server loop error', e)
            except Exception as e:
                print('server loop error', e)
                break


class rpcStub(object):
    def __init__(self):
        self.funs = {}

    def registerFunction(self, function, name=None):
        if name is None:
            name = function.__name__
        self.funs[name] = function


class rpcServer(zmqServer, rpcStub):
    def __init__(self):
        zmqServer.__init__(self)
        rpcStub.__init__(self)


if __name__ == '__main__':
    class Test:
        def setOn(self):
            print("is me ")

        def exit(self):
            print("exit")
            sys.exit()


    a = Test()
    test = rpcServer()
    test.registerFunction(a.setOn, "setOn")
    test.registerFunction(a.exit, "exit")


