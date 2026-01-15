import time

getNowMilliTime = lambda: int(time.time() * 1000)

class Timer:
    def __init__(self, period):
        '''
        :param period:设置定时器的时间周期,单位ms
        '''
        self.__period = period
        self.__last = getNowMilliTime()

    def isTimeUp(self):
        '''
        return:定时时间到则返回True并进入下一个定时周期,否则返回False
        '''
        if getNowMilliTime() - self.__last >= self.__period:
            self.__last = getNowMilliTime()
            return True
        else:
            return False

    def setPeriod(self, period):
        '''
        :param period:设置该定时器周期,设置后刷新定时器
        '''
        self.__period = period
        self.__last = getNowMilliTime()
    
    def reset(self):
        '''
        重启该定时器
        '''
        self.__last = getNowMilliTime()

    def setCallBack(self, callback):
        '''
        未实现
        '''
        pass

def sleep_ms(ms):
    time.sleep(ms / 1000.0)

def sleep_s(s):
    time.sleep(s)

if __name__ == "__main__":
    test = Timer(1000)
    while True:
        if test.isTimeUp():
            print("sdfsdfsdf")