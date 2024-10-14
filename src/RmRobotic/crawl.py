#!/usr/bin/env python
# !coding=utf-8
"""

在 机械臂 零位状态 通过设置 相机坐标系下物体得 位置 x y z 来验证 计算出来得 其次变换矩阵是否准确

"""
import logging
import math
import time

import numpy as np

from move_robot import robotic_arm
from move_robot import config


from move_robot.config import CODE_fi, HOST_fi
from move_robot.convert2 import convert, pose_to_homogeneous_matrix
from move_robot.log_setting import CommonLog


logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


def chage_pose(pose, num):
    """
    根据物体和基座的其次变换矩阵 求得 物体z轴 0 0 num 所在位置对应 基座标系的位姿
    y轴补偿6cm
    Args:
        pose:

    Returns:

    """

    matrix = pose_to_homogeneous_matrix(pose)

    obj_init = np.array([0, 0, num])

    obj_init = np.append(obj_init, [1])  # 将物体坐标转换为齐次坐标

    obj_base_init = matrix.dot(obj_init)

    return [i for i in obj_base_init[:3]] + pose[3:]


def cb_target_pose(x, y, z, rx, ry, rz, arm):

    # x ,y , z = 0,0,0.1
    # x ,y , z = 0.1,0,0
    # x, y, z = 0.056631, 0, 0.12
    # x, y, z = -0.033, -0.005, 0.15
    # rx, ry, rz = 2.90267785 * (math.pi / 180.0), 17.03540637 * (math.pi / 180.0), -96.51562633 * (math.pi / 180.0)
    # rx, ry, rz = 0.47395006,  0.42707277, -1.83218898

    logger_.info(f'\nxMeters：{x}\nyMeters：{y}\nzMeters：{z}')

    joint, current_pose, error_code = arm.get_curr_arm_state()  # 获取当前机械臂状态
    print(current_pose)
    obj_pose = convert(x, y, z, rx, ry, rz, *current_pose)
    obj_pose = [i for i in obj_pose]

    print(f'obj_pose:{obj_pose}')

    init_pose_ = chage_pose(obj_pose, -0.18)
    print(f'init_pose_:{init_pose_}')

    # arm.release()

    a = arm.movej_p(init_pose_, v=10)

    finally_pose = chage_pose(obj_pose, -0.11)

    print(f'finally_pose:{finally_pose}')

    arm.movej_p(finally_pose, v=10)

    # arm.clamping()

    finally_pose[2] += 0.05
    arm.movej_p(finally_pose, v=10)


def side_target_pose(x, y, z, rx, ry, rz, arm, current_pose):
    logger_.info(f'\nxMeters：{x}\nyMeters：{y}\nzMeters：{z}')
    # joint, current_pose, error_code = arm.get_curr_arm_state()  # 获取当前机械臂状态
    obj_pose = convert(x, y, z, rx, ry, rz, *current_pose)
    obj_pose = [i for i in obj_pose]

    init_pose_ = chage_pose(obj_pose, -0.18)

    a = arm.movej_p(init_pose_, v=10)


if __name__ == '__main__':
    arm = robotic_arm.Arm(CODE_fi, HOST_fi)
    arm.change_frame()  # 看是否机械臂基座
    cb_target_pose(-0.032, -0.005, 0.22, 0.47746567,  0.41939016, -1.82222933)


# -0.031, 0.006, 0.156
# 21.90112794 * math.pi / 180.0, -15.70235614 * math.pi / 180.0, -17.69677465 * math.pi / 180.0
# test4

# -0.03 -0.005 0.204
# 2.90267785  17.03540637 -96.51562633
