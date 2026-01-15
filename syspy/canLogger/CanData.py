class direction:
    rx = 0
    tx = 1

class CanDataBase(object):
    def __init__(self):
        self.id = 0x000
        self.is_extend = False
        self.is_remote = False
        self.DLC = 8
        self.Data = [0x00,00,0x00,0x00,0x00,0x00,0x00,0x00]
        self.channel = 1
        self.timestamp = 0
        self.direction = direction.tx
    def copy(self, orig):
        self.id = orig.getID()
        self.is_extend = orig.getExtend()
        self.is_remote = orig.getRemote()
        self.DLC = orig.getDLC()
        self.copyData(orig)
        self.channel = orig.getChannel()
        self.timestamp = 0
        self.direction = direction.tx
    def setExtend(self):
        self.is_extend = True
    def getExtend(self):
        return self.is_extend
    def setRemote(self):
        self.is_remote = True
    def getRemote(self):
        return self.is_remote
    def setID(self,id):
        self.id = id
    def getID(self):
        return self.id
    def setDLC(self,num):
        if num > 8:
            self.DLC = 8
        else:
            self.DLC = num
    def getDLC(self):
        return self.DLC
    def copyData(self,org_data):
        if len(org_data.Data) == 8:
            self.Data = org_data.Data.copy()
    def setData(self,array_data):
        if len(array_data) == 8:
            self.Data = array_data.copy()
    def getData(self):
        return self.Data
    def setChannel(self,channel):
        self.channel = channel
    def getChannel(self):
        return self.channel

class txCanData(CanDataBase):
    def __init__(self):
        super(txCanData,self).__init__()
        self.direction = direction.tx

class rxCanData(CanDataBase):
    def __init__(self):
        super(rxCanData,self).__init__()
        self.direction = direction.rx
