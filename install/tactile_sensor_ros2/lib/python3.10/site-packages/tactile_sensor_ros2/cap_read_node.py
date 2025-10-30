#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench, WrenchStamped
import json
import threading
import time
from typing import List
from std_msgs.msg import String  


from .class_ch341 import ClassCh341
from .class_finger import ClassFinger
from .sensorPara import FingerParamTS
# from .finger_log_setting import finger_setup_logging
import socket
from dataclasses import dataclass, field

# ROS 2适配：移除队列，使用ROS 2话题

@dataclass
class fingerDataPack:
    sensorIndex: int
    channelCapData: List[int]
    nf: List[float]
    tf: List[float]
    tfDir: List[int]
    sProxCapData: List[int]
    mProxCapData: List[int]
    config: FingerParamTS = field(default=None, repr=False)

class ClassCapRead:
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        # ROS 2适配：从参数服务器读取配置
        self.max_finger_num = self.node.get_parameter('max_finger_num').value
        self.use_vofa_debug = self.node.get_parameter('use_vofa_debug').value
        self.get_cap_ms = self.node.get_parameter('get_cap_ms').value
        self.cap_sync_ms = self.node.get_parameter('cap_sync_ms').value
        self.i2c_speed = self.node.get_parameter('i2c_speed').value
        
        self.logger.info(f"Loaded parameters: max_fingers={self.max_finger_num}, "
                        f"i2c_speed={self.i2c_speed}, get_cap_ms={self.get_cap_ms}")

        # 初始化CH341
        self.ch341 = ClassCh341(node)
        
        # 传感器列表
        self.fingers = []
        for i in range(self.max_finger_num):
            self.fingers.append(ClassFinger(2+i, self.ch341, node))

        # 状态变量
        self.ch341CheckTimer = 0
        self.mcuInit = 0
        self.pcaAddr = 0x70
        self.ch341Init = 0
        self.syncTimer = 0
        self.exitFlg = False
        
        # CH341连接状态
        self.connectStatus = 0  # 0=未初始化, 1=库加载, 2=设备打开, 3=速度设置, 4=运行
        
        # ROS 2适配：VOFA调试
        # self.connectDebug()

        # 原日志系统初始化
        # finger_setup_logging()

    def __del__(self):
        self.deinit()

    def deinit(self):
        # self.disConnectDebug()
        if hasattr(self, 'ch341'):
            self.ch341.disconnect()
        self.exitFlg = True
        self.logger.info("CH341 resources released")

    def set_sensor_enable(self, idx):
        _pack = [idx]
        self.ch341.write(self.pcaAddr, _pack)
        
    def ch341Connect(self):
        """CH341连接状态机"""
        if self.connectStatus == 0:  # 未初始化
            if self.ch341.init():
                self.logger.info("CH341 library initialized")
                self.connectStatus = 1
            else:
                self.logger.error("CH341 library initialization failed")
                
        elif self.connectStatus == 1:  # 库加载成功
            if self.ch341.open():
                self.logger.info("CH341 device opened")
                self.connectStatus = 2
            else:
                self.logger.error("CH341 device open failed")
                self.connectStatus = 0
                
        elif self.connectStatus == 2:  # 设备打开成功
            if self.ch341.set_speed(self.i2c_speed):
                self.logger.info(f"CH341 I2C speed set to {self.i2c_speed}")
                self.ch341.set_int(0)
                time.sleep(1)
                self.ch341.set_int(1)
                self.connectStatus = 3
            else:
                self.logger.error("CH341 speed setting failed")
                
        elif self.connectStatus == 3:  # 速度设置成功
            self.connectStatus = 4  # 进入运行状态
            self.logger.info("CH341 fully initialized and ready")

    def capRead(self):
        """数据采集主循环"""
        if self.exitFlg:
            return

        # 检查CH341连接状态
        if self.connectStatus < 4:
            self.ch341Connect()
            return

        capReadTime = time.time()
        connectedSensorChan = 0
        connectedSensorCnt = 0
        
        for fingerIndex in range(len(self.fingers)):
            try:
                self.set_sensor_enable(1 << (self.fingers[fingerIndex].pcaIdx))
                connectedSensorChan |= 1 << (self.fingers[fingerIndex].pcaIdx)

                if not self.fingers[fingerIndex].connect:
                    if self.fingers[fingerIndex].checkSensor():
                        self.logger.info(f"Sensor[{fingerIndex}] connected")
                    else:
                        self.logger.debug(f"Address {fingerIndex}, connection failed")
                else:
                    if self.fingers[fingerIndex].capRead():
                        self.publish_finger_data(fingerIndex)
                        connectedSensorCnt += 1
            except Exception as e:
                self.logger.error(f"Error reading finger {fingerIndex}: {e}")

        # 多传感器同步逻辑
        if connectedSensorCnt > 1 and (time.time() - self.syncTimer) > (self.cap_sync_ms / 1000.0):
            self.syncTimer = time.time()
            self.set_sensor_enable(connectedSensorChan)
            self.ch341.set_int(1)
            for finger in self.fingers:
                if finger.connect:
                    finger.snsCmd.setSensorSync(0)
                    break

        # VOFA调试输出
        self.debugPrint()

    

    def publish_finger_data(self, finger_index):
        """发布单个手指数据到ROS 2话题"""
        finger = self.fingers[finger_index]
        
        # 创建WrenchStamped消息（用于力数据）
        wrench_msg = WrenchStamped()
        wrench_msg.header = Header()
        wrench_msg.header.stamp = self.node.get_clock().now().to_msg()
        wrench_msg.header.frame_id = f"finger_{finger_index}"
        
        # 如果有法向力和切向力数据，填充到wrench消息
        if finger.readData.nf and len(finger.readData.nf) > 0:
            wrench_msg.wrench.force.z = float(finger.readData.nf[0])  # 法向力
            
        if finger.readData.tf and len(finger.readData.tf) > 0:
            wrench_msg.wrench.force.x = float(finger.readData.tf[0])  # 切向力
            # 可以根据tfDir决定方向分量
            
        # 发布力数据
        self.node.wrench_publishers[finger_index].publish(wrench_msg)
        
        # 创建自定义JSON消息包含所有数据
        sensor_data = {
            'sensor_index': finger_index,
            'timestamp': time.time(),
            'channel_cap_data': finger.readData.channelCapData,
            'normal_forces': finger.readData.nf,
            'tangential_forces': finger.readData.tf,
            'tangential_directions': finger.readData.tfDir,
            'self_prox_cap': finger.readData.sProxCapData,
            'mutual_prox_cap': finger.readData.mProxCapData
        }
        
        # 发布JSON数据
        from std_msgs.msg import String
        json_msg = String()
        json_msg.data = json.dumps(sensor_data)
        self.node.json_publishers[finger_index].publish(json_msg)

    def debugPrint(self):
        if self.use_vofa_debug == 1 and hasattr(self, 'socketConnected') and self.socketConnected:
            if self.fingers:
                fingerIndex = 0
                _log1 = ""
                for index in range(len(self.fingers[fingerIndex].readData.nf)):
                    _log1 += str(int(self.fingers[fingerIndex].readData.nf[index]*1000)) + ','
                    _log1 += str(int(self.fingers[fingerIndex].readData.tf[index]*1000)) + ','
                    _log1 += str(self.fingers[fingerIndex].readData.tfDir[index]) + ','
                
                for index in range(len(self.fingers[fingerIndex].readData.sProxCapData)):
                    _log1 += str(self.fingers[fingerIndex].readData.sProxCapData[index]) + ','
                    
                for index in range(len(self.fingers[fingerIndex].readData.mProxCapData)):
                    _log1 += str(self.fingers[fingerIndex].readData.mProxCapData[index]) + ','
                    
                _log1 += "0\r\n"
                try:
                    self.vofaClient.send(_log1.encode())
                except:
                    self.socketConnected = False


class TactileSensorNode(Node):
    def __init__(self):
        super().__init__('tactile_sensor_node')
        
        # ROS 2适配：参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_finger_num', 5),
                ('i2c_speed', 2),
                ('use_vofa_debug', 0),
                ('get_cap_ms', 15),
                ('cap_sync_ms', 1000)
            ]
        )
        
        # ROS 2适配：QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 创建发布器
        self.wrench_publishers = []
        self.json_publishers = []
        
        max_fingers = self.get_parameter('max_finger_num').value
        for i in range(max_fingers):
            self.wrench_publishers.append(
                self.create_publisher(WrenchStamped, f'/tactile_sensor/finger_{i}/wrench', qos_profile)
            )
            self.json_publishers.append(
                self.create_publisher(String, f'/tactile_sensor/finger_{i}/data', qos_profile)
            )
        
        # 初始化传感器读取类
        self.cap_reader = ClassCapRead(self)
        
        # ROS 2适配：使用定时器替代原线程定时
        get_cap_ms = self.get_parameter('get_cap_ms').value
        self.timer = self.create_timer(get_cap_ms / 1000.0, self.timer_callback)
        
        self.get_logger().info("Tactile sensor node initialized")

    def timer_callback(self):
        """定时器回调函数，执行数据采集"""
        try:
            self.cap_reader.capRead()
        except Exception as e:
            self.get_logger().error(f"Error in capRead: {str(e)}")

    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info("Shutting down tactile sensor node...")
        if hasattr(self, 'cap_reader'):
            try:
                self.cap_reader.deinit()
            except Exception as e:
                self.get_logger().error(f"Error during deinit: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TactileSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()