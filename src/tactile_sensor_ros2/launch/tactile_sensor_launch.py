from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import os

def generate_launch_description():
    
    log_dir = LaunchConfiguration('log_dir', default=os.path.expanduser('~/tactile_ws/logs'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_dir',
            default_value=log_dir,
            description='日志文件存储目录'
        ),
        DeclareLaunchArgument(
            'max_finger_num',
            default_value='5',
            description='Maximum number of fingers to connect (1-5)'
        ),
        DeclareLaunchArgument(
            'i2c_speed',
            default_value='2',
            description='I2C speed: 0=20kHz, 1=100kHz, 2=400kHz, 3=750kHz'
        ),
        DeclareLaunchArgument(
            'use_vofa_debug',
            default_value='0',
            description='Enable VOFA debug output: 0=disable, 1=enable'
        ),
        DeclareLaunchArgument(
            'get_cap_ms',
            default_value='15',
            description='Capacitance data acquisition interval in milliseconds'
        ),
        DeclareLaunchArgument(
            'cap_sync_ms',
            default_value='1000',
            description='Capacitance synchronization interval in milliseconds'
        ),
        
        SetEnvironmentVariable(
            name='ROS_LOG_DIR',  # 环境变量名
            value=log_dir       # 变量值（使用之前声明的参数）
        ),
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_FILE_APPEND',
            value='1'
        ),

        Node(
            package='tactile_sensor_ros2',
            executable='cap_read_node',
            name='tactile_sensor_node',
            output='screen',
            parameters=[{
                'max_finger_num': LaunchConfiguration('max_finger_num'),
                'i2c_speed': LaunchConfiguration('i2c_speed'),
                'use_vofa_debug': LaunchConfiguration('use_vofa_debug'),
                'get_cap_ms': LaunchConfiguration('get_cap_ms'),
                'cap_sync_ms': LaunchConfiguration('cap_sync_ms'),
            }],
            
        ),
        
        Node(
            package='tactile_sensor_ros2',
            executable='data_subscriber',
            name='tactile_data_subscriber',
            output='screen',
        ),
    ])