import logging
import os
import sys
from datetime import datetime


class CustomFormatter(logging.Formatter):
    """自定义日志格式化类，确保微秒部分输出正确"""

    def formatTime(self, record, datefmt=None):
        ct = datetime.fromtimestamp(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            s = ct.strftime("%Y-%m-%d %H:%M:%S.%f")
        return s


def finger_setup_logging():
    # 获取当前日期，创建以日期命名的文件夹
    current_date = datetime.now().strftime("%Y-%m-%d")
    log_dir = os.path.join("logs", current_date)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # 创建日志文件，使用启动时间命名
    log_file = datetime.now().strftime("%H-%M-%S") + ".log"
    log_path = os.path.join(log_dir, log_file)

    # 定义日志格式
    log_format = '%(asctime)s [%(levelname)s] %(filename)s - \
                %(funcName)s(%(lineno)d): %(message)s'
    date_format = '%Y-%m-%d %H:%M:%S.%f'

    # 配置日志记录器
    logging.basicConfig(
        level=logging.INFO,  # 设置最低日志级别为DEBUG
        handlers=[
            logging.StreamHandler(sys.stdout),  # 输出到控制台
            logging.FileHandler(log_path, encoding='utf-8')  # 输出到文件
        ]
    )

    # 设置自定义格式器以确保微秒显示正确
    formatter = CustomFormatter(log_format, datefmt=date_format)
    for handler in logging.getLogger().handlers:
        handler.setFormatter(formatter)
