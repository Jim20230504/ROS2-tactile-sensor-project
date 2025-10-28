import time
import os
import sys
from ctypes import c_byte, c_long, c_uint32
import ctypes
import glob

if os.name == 'nt':
    from ctypes import windll
elif os.name == 'posix':
    from ctypes import cdll

class ClassCh341:
    # 接口固定宏
    _mCH341A_CMD_I2C_STREAM = 0xAA
    _mCH341A_CMD_I2C_STM_STA = 0x74
    _mCH341A_CMD_I2C_STM_STO = 0x75
    _mCH341A_CMD_I2C_STM_OUT = 0x80
    _mCH341A_CMD_I2C_STM_IN = 0xC0
    _mCH341A_CMD_I2C_STM_MAX = 63
    _mCH341A_CMD_I2C_STM_SET = 0x60
    _mCH341A_CMD_I2C_STM_US = 0x40
    _mCH341A_CMD_I2C_STM_MS = 0x50
    _mCH341A_CMD_I2C_STM_DLY = 0x0F
    _mCH341A_CMD_I2C_STM_END = 0x00

    _mStateBitINT = 0x00000400

    IIC_SPEED_20 = 0
    IIC_SPEED_100 = 1
    IIC_SPEED_400 = 2
    IIC_SPEED_750 = 3

    def __init__(self, node=None):
        self.deviceID = ctypes.c_uint32()
        self.fd = -1
        self.ic = None
        
        # ROS 2适配：使用节点日志
        self.node = node
        if node:
            self.logger = node.get_logger()
        else:
            from finger_log_setting import logging
            self.logger = logging.getLogger(__name__)

    def init(self):
        if os.name == 'nt':  # Windows 环境
            libPath = os.path.dirname(
                sys.argv[0]) + r'/lib/ch341/windows/CH341DLLA64.DLL'
        elif os.name == 'posix':

            if hasattr(sys, '_MEIPASS'):
                # 如果运行在 PyInstaller 打包环境中
                # _MEIPASS 指向打包目录的根 (onedir) 或临时解压目录 (onefile)
                # 假设 libch347.so 在打包目录的 _internal 文件夹下
                bundle_root = sys.MEIPASS
                libPath = os.path.join(bundle_root, 'libch347.so')
                self.logger.info(f"运行在打包环境中。尝试从 _MEIPASS 加载库: {libPath}")
            else:
                # 如果运行在标准的 Python 开发环境中
                # 获取当前脚本文件 (start_project.py) 所在的目录
                script_dir = os.path.dirname(os.path.abspath(__file__))
 
                libPath = os.path.join(script_dir, 'lib', 'ch341', 'CH341PAR_LINUX', 'lib', 'x64', 'dynamic', 'libch347.so') 

                self.logger.info(f"运行在开发环境中。尝试从脚本目录相对路径加载库: {libPath}")
            # --- 路径构建结束 ---

        dllExist = os.path.exists(libPath)
        if not dllExist:
            self.logger.error('未找到库文件')
            return False
        else:
            try:
                if os.name == 'nt':  # Windows 环境
                    self.ic = windll.LoadLibrary(libPath)  # ch341接口

                    self.ch341GetInput = self.ic.CH341GetInput
                    self.ch341CloseDevice = self.ic.CH341CloseDevice
                    self.ch341WriteData = self.ic.CH341WriteData
                    self.ch341WriteRead = self.ic.CH341WriteRead
                    self.ch341SetOutput = self.ic.CH341SetOutput
                    self.ch341SetStream = self.ic.CH341SetStream

                elif os.name == 'posix':
                    self.ic = cdll.LoadLibrary(libPath)  # ch341接口

                    self.ch341GetInput = self.ic.CH34xGetInput
                    self.ch341CloseDevice = self.ic.CH34xCloseDevice
                    self.ch341WriteData = self.ic.CH34xWriteData
                    self.ch341WriteRead = self.ic.CH34xWriteRead
                    self.ch341SetOutput = self.ic.CH34xSetOutput
                    self.ch341SetStream = self.ic.CH34xSetStream

                self.logger.info("ch341加载成功")
                return True
            except Exception as e:
                self.logger.error(f"ch341加载失败, err = {e}")
                return False

    def open(self):
        try:
            if os.name == 'nt':  # Windows 环境
                self.fd = self.ch341OpenDevice(0)
                if self.fd == -1:
                    self.logger.error("CH341 device open failed on Windows")
                    return False
                self.logger.info("CH341 device opened successfully on Windows")
                return True

            elif os.name == 'posix':  # Linux 环境
                devices = glob.glob('/dev/ch34x_pis*')
                if not devices:
                    self.logger.error("No CH341 device found on Linux")
                    return False
                    
                device_path = devices[0].encode()
                self.fd = self.ic.CH34xOpenDevice(device_path)
                if self.fd == -1:
                    self.logger.error("CH341 device open failed on Linux")
                    return False
                self.logger.info("CH341 device opened successfully on Linux")
                return True

        except Exception as e:
            self.logger.error(f"Error opening CH341 device: {e}")
            return False

    def disconnect(self):
        if hasattr(self, 'ch341CloseDevice') and self.fd != -1:
            try:
                self.ch341CloseDevice(self.fd)
                self.logger.info("CH341 device closed")
            except Exception as e:
                self.logger.error(f"Error closing CH341 device: {e}")
            self.fd = -1

    def connectCheck(self):
        if not hasattr(self, 'ch341GetInput'):
            return 0
        try:
            return self.ch341GetInput(self.fd, ctypes.byref(self.deviceID))
        except Exception as e:
            self.logger.error(f"Error in connectCheck: {e}")
            return 0

    def write(self, addr, data):
        if not hasattr(self, 'ch341WriteData'):
            self.logger.error("CH341 write method not available")
            return 0
            
        sLen = len(data)
        tmpData = data.copy()
        tmpLen = sLen
        pack = []
        cnt = 20
        packNum = sLen // cnt
        sLen %= cnt

        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append(addr << 1)
        
        for i in range(packNum):
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | cnt)
            pack.extend(tmpData[0:20])
            del tmpData[0:20]
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            
            sendBuf = (c_byte * len(pack))()
            for j in range(len(pack)):
                sendBuf[j] = pack[j]
            sendLen = c_byte(len(pack))
            
            try:
                if not self.ch341WriteData(self.fd, sendBuf, ctypes.byref(sendLen)):
                    return 0
            except Exception as e:
                self.logger.error(f"Write error: {e}")
                return 0
                
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)

        if sLen >= 1:
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | sLen)
            pack.extend(tmpData[0:sLen])
            
        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        
        sendBuf = (c_byte * len(pack))()
        for j in range(len(pack)):
            sendBuf[j] = pack[j]
        sendLen = c_byte(len(pack))
        
        try:
            if not self.ch341WriteData(self.fd, sendBuf, ctypes.byref(sendLen)):
                return 0
        except Exception as e:
            self.logger.error(f"Write error: {e}")
            return 0
            
        return tmpLen

    def read(self, addr, data):
        if not hasattr(self, 'ch341WriteRead'):
            self.logger.error("CH341 read method not available")
            return 0
            
        if not data or len(data) == 0:
            return 0
            
        rLen = len(data)
        pack = []
        readBuf = []
        readLen = 0

        packNum = rLen // 30
        rLen %= 30
        if rLen == 0:
            rLen = 30
            packNum -= 1

        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append((addr << 1) | 0x01)
        pack.append(self._mCH341A_CMD_I2C_STM_MS | 1)
        
        for i in range(packNum):
            pack.append(self._mCH341A_CMD_I2C_STM_IN | 30)
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            
            sendBuf = (c_byte * len(pack))()
            for j in range(len(pack)):
                sendBuf[j] = pack[j]
                
            recLen = c_byte(0)
            recBuf = (c_byte * self._mCH341A_CMD_I2C_STM_MAX)()
            
            try:
                if not self.ch341WriteRead(self.fd,
                                         len(pack),
                                         sendBuf,
                                         self._mCH341A_CMD_I2C_STM_MAX,
                                         1,
                                         ctypes.byref(recLen),
                                         recBuf):
                    return 0
            except Exception as e:
                self.logger.error(f"Read error: {e}")
                return 0
                
            for j in range(recLen.value):
                readBuf.append(recBuf[j])
            readLen += 30
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)

        if rLen > 1:
            pack.append(self._mCH341A_CMD_I2C_STM_IN | (rLen - 1))
        pack.append(self._mCH341A_CMD_I2C_STM_IN | 0)
        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        
        sendBuf = (c_byte * len(pack))()
        for j in range(len(pack)):
            sendBuf[j] = pack[j]
            
        recLen = c_byte(0)
        recBuf = (c_byte * self._mCH341A_CMD_I2C_STM_MAX)()
        
        try:
            if not self.ch341WriteRead(self.fd,
                                     len(pack),
                                     sendBuf,
                                     self._mCH341A_CMD_I2C_STM_MAX,
                                     1,
                                     ctypes.byref(recLen),
                                     recBuf):
                return 0
        except Exception as e:
            self.logger.error(f"Read error: {e}")
            return 0
            
        for j in range(recLen.value):
            readBuf.append(recBuf[j])
            
        data.clear()
        data.extend(readBuf)
        return len(readBuf)

    def set_int(self, lvl):
        if not hasattr(self, 'ch341GetInput') or not hasattr(self, 'ch341SetOutput'):
            return
            
        status = c_long(0)
        try:
            self.ch341GetInput(0, ctypes.byref(status))
            time.sleep(0.01)
            if lvl:
                self.ch341SetOutput(self.fd, 0x03, 0xFF00, status.value | self._mStateBitINT)
            else:
                self.ch341SetOutput(self.fd, 0x03, 0xFF00, status.value & (~self._mStateBitINT))
        except Exception as e:
            self.logger.error(f"Error setting INT: {e}")

    def get_int(self):
        if not hasattr(self, 'ch341GetInput'):
            return 0
            
        status = c_long(0)
        try:
            self.ch341GetInput(0, ctypes.byref(status))
            return (status.value & self._mStateBitINT) >> 10
        except Exception as e:
            self.logger.error(f"Error getting INT: {e}")
            return 0

      # 设置IIC速度
    # return：0错误，1成功
    def set_speed(self, speed):
        if speed != self.IIC_SPEED_20 \
                and speed != self.IIC_SPEED_100 \
                and speed != self.IIC_SPEED_400 \
                and speed != self.IIC_SPEED_750:
            return False
        if self.ch341SetStream(self.fd, speed | 0) is False:
            self.logger.error("speed err")
            return False
        else:
            return True
