# 类似的日志适配
import time
from ctypes import c_uint8


# from finger_log_setting import logging
# logger = logging.getLogger(__name__)

class ClassSensorCmd:
    def __init__(self, ch341, node=None):
        
        # 命令定义
        self.CMD_GET_CHANNEL_NUM = 0x01                 # 读取通道数量
        self.CMD_GET_SENSOR_CAP_DATA = 0x60             # 读取传感器数据
        self.CMD_GET_SENSOR_CHANNEL = 0x62              # 读取通道数量
        self.CMD_SET_SENSOR_AUTO_DAC = 0x63             # 自动dac
        self.CMD_GET_SENSOR_ERR_CODE = 0x64             # 读取错误码
        self.CMD_GET_SENSOR_TEST_HZ = 0x6C              # 读取采样率

        self.CMD_SET_SENSOR_IIC_ADDR = 0x70             # 设置传感器IIC地址
        self.CMD_GET_SENSOR_IIC_ADDR = 0x71             # 读取传感器IIC地址
        self.CMD_SET_SENSOR_CDC_SYNC = 0x72             # 采集停止，用于同步
        self.CMD_SET_SENSOR_CDC_START_OFFSET = 0x73     # 设置电容开始偏移
        self.CMD_SET_SENSOR_RESTART = 0x77              # 软件复位
        self.CMD_SET_SENSOR_SEND_TYPE = 0x7F            # 数据传输类型

        self.CMD_GET_VERSION = 0xA0                     # 读取软件版本
        self.CMD_SOFT_RESTART = 0xA1                    # 软件复位
        self.CMD_GET_TYPE = 0xA2                        # 读取设备类型
        self.CMD_SET_TYPE = 0xA3                        # 设置设备类型
        self.CMD_SET_INF = 0xA5                         # 设置输出接口
        self.CMD_GET_PRG = 0xA6                         # 获取项目类型

        self._ch341 = ch341
        self.sendTimePre = list()
        self.sendTimePre.append(time.time())
        self.sendTimePre.append(time.time())
        self.sendTimeNow = list()
        self.sendTimeNow.append(time.time())
        self.sendTimeNow.append(time.time())
        self.sendCnt = list()
        self.sendCnt.append(0)
        self.sendCnt.append(0)
        
        
        self.node = node
        if node:
            self.logger = node.get_logger()
        # else:
        #     from finger_log_setting import logging
        #     self.logger = logging.getLogger(__name__)
            
        self.sendTimePre = [time.time(), time.time()]
        self.sendTimeNow = [time.time(), time.time()]
        self.sendCnt = [0, 0]

    
    # 计算校验和
    # pack 数据
    def calcSum(self, pack):
        if len(pack) <= 5:
            return 0
        _sum = 0
        for i in range(0, len(pack)):
            _sum += (pack[i] & 0xFF)
        pack.append(_sum & 0xFF)
        pack.append(_sum >> 8)

    # 检查校验值
    # pack：要检查的数据
    # return    0：校验值不一致
    #           1:校验值一致
    def checkSum(self, pack):
        if len(pack) <= 5:
            return False
        _sum = 0
        for i in range(0, len(pack) - 2):
            _sum += (pack[i] & 0xFF)
        chkL = _sum & 0xFF
        chkH = (_sum >> 8) & 0xFF
        rchkL = pack[len(pack) - 2] & 0xFF
        rchkH = pack[len(pack) - 1] & 0xFF
        if chkL == rchkL and chkH == rchkH:
            return True
        else:
            return False

    # 设置传感器iic地址
    # addr：当前地址
    # new_addr：系地址
    # return：读取的地址
    def setAddr(self, addr, new_addr):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_SET_SENSOR_IIC_ADDR)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(new_addr)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(11)))
        time.sleep(0.01)
        self._ch341.read(new_addr, _pack)
        checksum = self.checkSum(_pack)
        if checksum is True \
            and ((self.CMD_SET_SENSOR_IIC_ADDR | 0x80)
                 == c_uint8(_pack[3]).value):
            return _pack[7] & 0xFF
        return 0

    def getAddr(self, addr):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_GET_SENSOR_IIC_ADDR)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(11)))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) is True:
            return _pack[7] & 0xFF
        return 0

    def getSensorVersion(self, addr):
        pass

    # 读取电容通道数
    # addr：传感器地址
    def getSensorNum(self, addr):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_GET_CHANNEL_NUM)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(15)))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) is True:
            return (_pack[5] & 0xFF) + (_pack[6] & 0xFF) * 256
        return 0

    # 读取项目编号
    # addr：传感器地址
    def getSensorProjectIdex(self, addr):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_GET_PRG)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(11)))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) is True:
            return (_pack[7] & 0xFF) + (_pack[8] & 0xFF) * 256
        return 0

    # 设置发送类型
    
    def setSensorSendType(self, addr, sendType):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_SET_SENSOR_SEND_TYPE)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(sendType)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(11)))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) is True \
                and ((self.CMD_SET_SENSOR_SEND_TYPE | 0x80)
                     == c_uint8(_pack[3]).value):
            return True
        return False

    # 设置采集偏置
    
    def setSensorCapOffset(self, addr, offset):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_SET_SENSOR_CDC_START_OFFSET)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(offset)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack.clear()
        _pack.extend(list(range(6)))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        checksum = self.checkSum(_pack)
        if checksum is True \
                and ((self.CMD_SET_SENSOR_CDC_START_OFFSET | 0x80)
                     == c_uint8(_pack[3]).value):
            return True
        return False

    # 读取电容数据
    
    def getSensorCapData(self, addr, buf):
        tarLen = len(buf)
        err = self._ch341.read(addr, buf)
        if err == 0:
            self.logger.error("get data err")
        checkSum = self.checkSum(buf)

        if tarLen != len(buf):
            buf.clear()
            buf.extend(range(tarLen))
        
        if (len(buf) == tarLen and buf[0] & 0xFF) == 0x55 \
                and (buf[1] & 0xFF) == 0xAA \
                and checkSum is True:
            return True
        else:
            # logger.info(f"time={time.time()},len(buf) = {len(buf)}, tarLen={tarLen}, buf[0]={buf[0]}, buf[1]={buf[1]}, sum={checkSum}")
            pass
        return False

    # 设置采集同步
    
    def setSensorSync(self, addr):
        _pack = list()
        _pack.append(0xAA)
        _pack.append(0x55)
        _pack.append(0x03)
        _pack.append(self.CMD_SET_SENSOR_CDC_SYNC)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        _pack.append(0x00)
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)

        return True
