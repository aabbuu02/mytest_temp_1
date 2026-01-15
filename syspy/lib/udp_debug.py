import socket,sys
import logging,logging.handlers
class udpDebug:
    def __init__(self):
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception as e:
            pass
        
    def write(self, str1):
        try:
            str1 = str1.strip("\n")
            self.udp_socket.sendto(str1.encode("utf-8"), ("192.168.192.255", 20000))
        except Exception as e:
            pass
    
    def flush(self):
        pass

    def close(self):
        self.udp_socket.close()


class syslogDebug:
    def __init__(self, identifier="my_script"):
        """
        初始化SyslogLogger类，设置将print输出重定向到syslog。

        参数:
        - identifier: 标识符，用于标识日志来源（默认为'my_script'）
        """
        self.identifier = identifier
        self.logger = self._setup_logger()

        # 重定向stdout到syslog
        sys.stdout = self.SyslogRedirector(self.logger)

    def _setup_logger(self):
        # 设置syslog处理器
        syslog_handler = logging.handlers.SysLogHandler(address='/dev/log')

        # 设置日志格式，添加标识符
        formatter = logging.Formatter('%(asctime)s %(name)s: %(message)s')
        syslog_handler.setFormatter(formatter)

        # 获取logger
        logger = logging.getLogger(self.identifier)
        logger.setLevel(logging.INFO)
        logger.addHandler(syslog_handler)
        return logger

    class SyslogRedirector:
        def __init__(self, logger):
            self.logger = logger

        def write(self, message):
            if message.strip():
                self.logger.info(message.strip())

        def flush(self):
            pass

if __name__ == "__main__":
    pass