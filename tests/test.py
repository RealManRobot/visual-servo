import threading
import time

import numpy as np

from src.interface import ImageBaseVisualServo

ibvs = ImageBaseVisualServo(
    ip="192.168.1.18",  # 机械臂IP地址
    port=8080,  # 机械臂透传端口
    init_joint=[-2.288, -8.908, 97.424, -0.788, 88.872, -0.019],  # 给定机械臂的初始位姿
    center=[320, 240, 250],  # 视觉伺服追踪点位对齐位置，前两位代表是画面像素点，表示将物体固定到这个位置上。
    # 最后一位是机械臂末端到物体的距离，单位为mm
    hand_to_camera=np.array([[-0.013, 1, -0.0004, -0.092],  # 手眼标定矩阵
                             [-1, -0.013, 0.01, 0.03],
                             [0.009, 0.004, 1, 0.034],
                             [0, 0, 0, 1]])
)
thread2 = threading.Thread(target=ibvs.run)
thread2.start()

while True:
    """
    以下为模拟每秒发一个坐标信息
    """
    time.sleep(1)
    ibvs.center = [280, 200, 250]
    time.sleep(1)
    ibvs.center = [360, 200, 250]
    time.sleep(1)
    ibvs.center = [360, 280, 250]
    time.sleep(1)
    ibvs.center = [280, 280, 250]
