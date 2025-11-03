#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped
import json

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('tactile_data_subscriber')
        
        # 订阅所有手指的数据
        # self.subscriptions = []
        self.my_subscriptions = []  
        # 假设最大5个手指
        for i in range(5):
            # 订阅JSON格式的完整数据
            sub_json = self.create_subscription(
                String,
                f'/tactile_sensor/finger_{i}/data',
                self.create_json_callback(i),
                10
            )
            self.my_subscriptions.append(sub_json)
            
            # 订阅Wrench数据
            sub_wrench = self.create_subscription(
                WrenchStamped,
                f'/tactile_sensor/finger_{i}/wrench',
                self.create_wrench_callback(i),
                10
            )
            self.my_subscriptions.append(sub_wrench)
        
        self.get_logger().info("Tactile data subscriber initialized")

    def create_json_callback(self, finger_index):
        """创建JSON数据回调的闭包"""
        def callback(msg):
            try:
                data = json.loads(msg.data)
                self.get_logger().info(
                    f"Finger {finger_index}\n"
                    f"NF: {data['normal_forces']}\n"
                    f"TF: {data['tangential_forces']}\n"
                    f"Channels: {len(data['channel_cap_data'])}\n "
                    f"tDir:{data['tangential_directions']}"
                )
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON data from finger {finger_index}")
        return callback

    def create_wrench_callback(self, finger_index):
        """创建Wrench数据回调的闭包"""
        def callback(msg):
            self.get_logger().info(
                f"Finger {finger_index} Wrench - "
                # f"Fz: {msg.wrench.force.z:.3f}, "
                # f"Fx: {msg.wrench.force.x:.3f}"
            )
        return callback


def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        


if __name__ == '__main__':
    main()