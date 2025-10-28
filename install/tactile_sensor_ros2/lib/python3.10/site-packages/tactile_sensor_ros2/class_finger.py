# 类似的日志适配修改
import sys
import os
from .class_sensorcmd import ClassSensorCmd
from .sensorPara import finger_params
from .sensorPara import DynamicYddsComTs
from .sensorPara import DynamicYddsU16Ts
import time
from ctypes import sizeof
import copy

# ROS 2适配：移除原日志导入
# from finger_log_setting import logging
# logger = logging.getLogger(__name__)

class capData:
    # ... (保持不变)
    def __init__(self):
        self.sensorIndex = 0            # 电容序号，与iic addr相同
        self.channelCapData = list()    # 原始通道数值
        self.tf = list()                # 切向力数组
        self.tfDir = list()             # 切向力方向数组
        self.nf = list()                # 法向力数组
        self.sProxCapData = list()      # 接近(自电容)数组
        self.mProxCapData = list()      # 接近（互电容）数组
        self.cnt = 0                                    # 计数，测试用

    def init(self, addr, yddsNum, sProxNum, mProxNum, capChannelNum):
        self.sensorIndex = addr                             # 电容序号，与iic addr相同
        self.channelCapData = list(range(capChannelNum))    # 原始通道数值
        self.tf = list(range(yddsNum))                      # 切向力数组
        self.tfDir = list(range(yddsNum))                   # 切向力方向数组
        self.nf = list(range(yddsNum))                      # 法向力数组
        self.sProxCapData = list(range(sProxNum))           # 接近(自电容)数组
        self.mProxCapData = list(range(mProxNum))           # 接近（互电容）数组
        self.cnt = 0                                        # 计数，测试用

    def deinit(self):
        self.channelCapData = None
        self.tf = None
        self.tfDir = None
        self.nf = None
        self.sProxCapData = None
        self.mProxCapData = None


class ClassFinger:
    def __init__(self, pca_idx, ch341, node=None):
        self.snsCmd = ClassSensorCmd(ch341, node)  # 传递节点引用
        self.pcaIdx = pca_idx
        self.readData = capData()
        
        # ROS 2适配：使用节点日志
        self.node = node
        if node:
            self.logger = node.get_logger()
        else:
            from finger_log_setting import logging
            self.logger = logging.getLogger(__name__)
            
        self.disconnected()

    def checkSensor(self):
        # ... (逻辑不变，修改日志调用)
        addrRead = self.snsCmd.getAddr(0)
        
        if self.snsCmd.setSensorSendType(addrRead, 0) is not True:
            self.logger.error(f"setSensorSendType error, addr = {addrRead}")

        if self.snsCmd.setSensorCapOffset(addrRead, addrRead) is not True:
            self.logger.error(f"setSensorCapOffset error, addr = {addrRead}")

        projectRead = self.snsCmd.getSensorProjectIdex(addrRead)
        self.logger.info(f"Detected project: {projectRead}")
        
        # ... (其余逻辑不变)
        findProjectFlg = False
        if projectRead > 0:
            for pro in finger_params:
                if pro.prg == projectRead:
                    self.projectPara = copy.deepcopy(pro)
                    self.logger.info(f"finger connected, project = {self.projectPara.name}")
                    findProjectFlg = True
                    break

        if findProjectFlg is False:
            self.logger.error("not found vailed project, project para use default")

        self.connected(addrRead)

        return True

    # 传感器连接，初始化参数
    def connected(self, addr):
        self.addr = addr
        self.connect = True
        self.connectTimer = time.time()
        self.packIdx = 0
        self.data = list()
        self.data.extend(range(self.projectPara.pack_len))

        self.readData.init(addr,
                           self.projectPara.ydds_num,
                           self.projectPara.s_prox_num,
                           self.projectPara.m_prox_num,
                           self.projectPara.sensor_num)

        # logger.info(f"datalen={len(self.data)}")

    # 传感器断开,复位参数
    def disconnected(self):
        self.addr = 0xFF   # iic地址
        self.connect = False   # 连接标志位
        self.packIdx = 0     # 采样序号
        self.connectTimer = 0   # 连接超时计时

        self.projectPara = finger_params[0]

        self.readData.deinit()

    def capRead(self):
        rcvCapDataFlg = False

        for retry in range(0, 3):
            if self.snsCmd.getSensorCapData(self.addr, self.data) is True:
                if self.data[5] != self.projectPara.sensor_num:
                    self.logger.error(f"cap channel num err, read num = {self.data[5]},\
                    expect num = {self.projectPara.sensor_num}")

                if self.data[4] != self.packIdx:
                    self.packIdx = self.data[4]
                    self.connectTimer = time.time()

                    # 根据通道值占用字节大小获取通道数据
                    if self.projectPara.cap_byte == 4:
                        for j in range(0, self.projectPara.sensor_num):
                            self.readData.channelCapData[j] = ((self.data[6 + j * self.projectPara.cap_byte] & 0xFF) +
                                                               ((self.data[6 + j * self.projectPara.cap_byte + 1] & 0xFF) << 8) +
                                                               ((self.data[6 + j * self.projectPara.cap_byte + 2] & 0xFF) << 16) +
                                                               ((self.data[6 + j * 4 + 3] & 0xFF) << 24))
                    else:
                        for j in range(0, self.projectPara.sensor_num):
                            self.readData.channelCapData[j] = ((self.data[6 + j * self.projectPara.cap_byte] & 0xFF) +
                                                               ((self.data[6 + j * self.projectPara.cap_byte + 1] & 0xFF) << 8) +
                                                               ((self.data[6 + j * self.projectPara.cap_byte + 2] & 0xFF) << 16))

                    yddsOffset = 6 + self.projectPara.sensor_num*self.projectPara.cap_byte

                    if self.projectPara.ydds_type == 2:
                        struct_size = sizeof(DynamicYddsComTs)
                        for i in range(self.projectPara.ydds_num):
                            offset = yddsOffset + i * struct_size
                            struct_data = self.data[offset: offset + struct_size]
                            struct_data = [
                                value & 0xFF for value in struct_data]
                            struct_data = bytes(struct_data)  # 转换为 bytes 类型
                            instance = DynamicYddsComTs.from_buffer_copy(
                                struct_data)
                            self.readData.nf[i] = instance.nf
                            self.readData.tf[i] = instance.tf
                            self.readData.tfDir[i] = instance.tfDir
                            self.readData.sProxCapData[i] = instance.prox
                    elif self.projectPara.ydds_type == 4:
                        struct_size = sizeof(DynamicYddsU16Ts)
                        for i in range(self.projectPara.ydds_num):
                            offset = yddsOffset + i * struct_size
                            struct_data = self.data[offset: offset + struct_size]
                            # logger.info(f"struct={struct_data}")
                            struct_data = [
                                value & 0xFF for value in struct_data]
                            struct_data = bytes(struct_data)  # 转换为 bytes 类型
                            instance = DynamicYddsU16Ts.from_buffer_copy(
                                struct_data)
                            self.readData.nf[i] = instance.nf/100.0
                            self.readData.tf[i] = instance.tf/100.0
                            self.readData.tfDir[i] = instance.tfDir
                        sProxOffset = yddsOffset + self.projectPara.ydds_num*struct_size
                        for i in range(self.projectPara.s_prox_num):
                            self.readData.sProxCapData[i] = ((self.data[sProxOffset + i*self.projectPara.cap_byte] & 0xFF) +
                                                             ((self.data[sProxOffset + i*self.projectPara.cap_byte + 1] & 0xFF) << 8) +
                                                             ((self.data[sProxOffset + i*self.projectPara.cap_byte + 2] & 0xFF) << 16))
                        mProxOffset = yddsOffset + self.projectPara.ydds_num*struct_size
                        for i in range(self.projectPara.m_prox_num):
                            self.readData.mProxCapData[i] = ((self.data[mProxOffset + i*self.projectPara.cap_byte] & 0xFF) +
                                                             ((self.data[mProxOffset + i*self.projectPara.cap_byte + 1] & 0xFF) << 8) +
                                                             ((self.data[mProxOffset + i*self.projectPara.cap_byte + 2] & 0xFF) << 16))

                    rcvCapDataFlg = True

                break
            # else:
            #     logger.error("read err")

        # 2S未接收到数据超时
        if (time.time() - self.connectTimer) > 2:
            self.disconnected()
        
        return rcvCapDataFlg


