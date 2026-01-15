import zmq,json,threading,sys,queue,os

class zmqClient(object):
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.stop_flag = False
        self.func_json = ""
        self.queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker, name="worker")
        self.worker_thread.start()

    def close(self):
        print("close the socket")
        self.stop_flag = True
        self.worker_thread.join()  # 等待线程结束
        self.socket.close()
    
    def connect(self, addr):
        print(f'addr: {addr}')
        self.socket.connect(addr)

    def putQueue(self, data, event):
        # 将请求放入队列，并传入事件对象
        self.queue.put((data, event))

    def recv(self):
        return self.socket.recv()

    def worker(self):
        while not self.stop_flag:
            try:
                data, event = self.queue.get(timeout=1)
                # print(f"get: data{data},event{event}")
                self.socket.send(data)  # 发送数据

                events = dict(self.poller.poll(5000))
                if self.socket in events:
                    response = self.recv()
                    event.result = response
                    # print(f"event.result: {event.result}")
                    event.set()
                else:
                    print("poller Timeout,exit")
                    event.result = None
                    event.set()
                    sys.exit(1)
            except queue.Empty:
                continue
            except Exception as e:
                print(f'worker error:{e},send{self.func_json}')
        print('exit worker')

class rpcStub(object):
    def __getattr__(self, function):
        def _func(*args, **kwargs):
            try:
                d = {'method_name': function, 'method_args': args, 'method_kwargs': kwargs}
                self.func_json = json.dumps(d).encode('utf-8')
                event = threading.Event()
                self.putQueue(self.func_json, event)
                event.wait()
                if event.result:
                    reply = json.loads(event.result.decode())
                    return reply["res"]
                else:
                    print("poller Timeout or No result")
                    os._exit(1)
                    return None
            except Exception as e:
                print('rpcStub error', e)

        setattr(self, function, _func)
        return _func

class rpcClient(zmqClient, rpcStub):
    pass


