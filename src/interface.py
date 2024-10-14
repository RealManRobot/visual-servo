#!/usr/bin/env python
'''
Copyright (c) 2023-2025, eliot
All rights reserved.
'''

import json
import math
import time

from loguru import logger
from numpy import ndarray

from src.RmRobotic.rm65 import RM65B
from src.RmRobotic.rm_robotic_util import *


class ImageBaseVisualServo:
    def __init__(self, ip: str, port: int, init_joint: list, center: list, hand_to_camera: ndarray):
        self._deadband_error = 1
        self._control_loop_hz = 200.0

        self.timer_period = 1 / self._control_loop_hz * 1e3  # ms
        self._lambda = 0.03

        self.flag = False

        self._desired_feature = np.array([center[0], center[1]])
        self._desired_z = np.array([center[2]])
        self._current_feature = np.array([1e-10, 1e-10])
        self._current_z = [1e-10]

        self.hand_to_camera = hand_to_camera

        self._center = [center[0], center[1], center[2]]

        self.arm_ip = ip
        self.arm_port = port
        self.grasp_init = [x * 1000 for x in init_joint]
        self.init_robot()

    def init_robot(self):
        self.robot = RM65B()

        # 建立socket通讯
        self.client = socket.socket()
        self.client.connect((self.arm_ip, self.arm_port))

        # 获取机械臂初始关节位置
        cmd = '{"command":"get_joint_degree"}\r\n'
        self.client.send(cmd.encode('utf-8'))
        data = self.client.recv(1024)
        self.q_init = self.util_form_json_get_data(data, "joint")

        # 做些准备工作
        self.go_init()

    def go_init(self):
        joint_str = str(self.grasp_init[0]) + ',' + str(self.grasp_init[1]) + ',' + str(self.grasp_init[2]) + ',' + str(
            self.grasp_init[3]) + ',' + str(self.grasp_init[4]) + ',' + str(self.grasp_init[5])
        cmd = '{"command":"movej", "joint":[' + joint_str + '],"v":20,"r":0}\r\n'  # 机械臂初始姿态
        self.client.send(cmd.encode('utf-8'))
        self.client.recv(1024)

    def feature_tracker(self, oRc):
        # 计算x、y轴上的误差
        error = self._current_feature - self._desired_feature
        mean_error = np.linalg.norm(error)
        if np.all((mean_error) <= self._deadband_error):
            error = np.array([0, 0])
        dcxy = self._lambda * error.reshape((2, 1))

        # 计算z轴误差
        z_error = -(self._current_z[0] - self._desired_z[0])
        if abs(z_error) <= 1e-3:  # 如果z轴误差在一定范围内，可以认为z轴已到达目标位置
            z_error = 0.0

        # x, y 和 z 轴的平移量
        dp = np.array([dcxy[0][0], dcxy[1][0], -self._lambda * z_error, 0.0, 0.0, 0.0]).T

        # 如果z轴已达到目标深度，停止z方向移动
        # if (self._current_z[0] <= self._desired_z[0]):
        #     dp[2] = 0

        # 数据整合
        Rv = np.vstack((np.hstack((oRc, np.zeros((3, 3)))),
                        np.hstack((np.zeros((3, 3)), oRc))))
        v = np.dot(Rv, dp)
        return v

    def util_delay_msecond(self, t):
        # delay t ms, 精度±1ms
        start, end = 0, 0
        start = time.time_ns()
        while (end - start < t * 1000000):
            end = time.time_ns()

    def util_form_json_get_data(self, json_str, key):
        text = json.loads(json_str)
        return text[key]

    def util_form_json_get_data_arm_state(self, json_str, key):
        text = json.loads(json_str)
        return text["arm_state"][key]

    @property
    def center(self):
        return self._center

    @center.setter
    def center(self, value):
        if value != self._center:
            self._center = value
            self._current_feature = np.array([float(self._center[0]), float(self._center[1])])  # ibvs 物体中心点
            self._current_z = [float(self._center[2])]  # 深度
            self.flag = True

    def check(self):
        pass

    def run(self):
        # 检查必要条件
        self.check()
        while True:
            try:
                time.sleep(0.000001)
                if self.flag:
                    self.q_init = [math.radians(item * 1e-3) for item in self.q_init]
                    base_to_hand = self.robot.fkine(self.q_init)  # 坐标系转换
                    oTc = np.dot(base_to_hand, self.hand_to_camera)
                    oRc = oTc[:3, :3]
                    if self.flag:
                        vel = self.feature_tracker(oRc)
                        jacobian = self.robot.jacob0(self.q_init)
                        jacobian_inv = np.linalg.pinv(jacobian)
                        dq = np.dot(jacobian_inv, vel)
                        vector = np.concatenate([np.array(matrix).ravel() for matrix in dq])
                        q = np.degrees(self.q_init + vector.T * self.timer_period * 1e-3) * 1e3  # s
                    else:
                        q = np.degrees(self.q_init) * 1e3  # s
                    pos_str = str(q[0]) + ',' + str(q[1]) + ',' + str(q[2]) + ',' + str(q[3]) + ',' + str(
                        q[4]) + ',' + str(
                        q[5])

                    # 方式一 使用joint_canfd
                    # joint_canfd_cmd = '{"command":"joint_canfd","joint":[' + pos_str + '],"follow":false}\r\n'  # 将转化后的物体坐标信息透传发送给机械臂
                    # joint_canfd_cmd = '{"command":"movej_follow","joint":[' + pos_str + '],"follow":false}\r\n'  # 将转化后的物体坐标信息透传发送给机械臂
                    # print(f"joint_canfd_cmd is [{joint_canfd_cmd}]")
                    # self.client.send(joint_canfd_cmd.encode('utf-8'))
                    # self.util_delay_msecond(self.timer_period)

                    # 方式二，使用movej_canfd，然后再手动获取一下当前位姿
                    joint_canfd_cmd = '{"command":"movej_canfd","joint":[' + pos_str + '],"follow":false}\r\n'
                    self.client.send(joint_canfd_cmd.encode('utf-8'))
                    get_data_cmd = '{"command":"get_joint_degree"}'
                    self.client.send(get_data_cmd.encode('utf-8'))
                    self.util_delay_msecond(self.timer_period)

                    # update q_init
                    ret = self.client.recv(1024)
                    # print(ret)
                    feedback_data = ret.decode('utf-8').strip()
                    try:
                        self.q_init = self.util_form_json_get_data(feedback_data, "joint")  # 判断透传值格式
                    except Exception as e:
                        logger.warning(f"Arm Recv Error：【{str(e)}】")
                    logger.info(f"Visual Servo Move...")
            except Exception as e:
                logger.error(f"Visual Servo Catch Error：【{str(e)}】")
