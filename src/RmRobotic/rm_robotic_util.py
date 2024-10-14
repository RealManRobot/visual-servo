import logging

from src.RmRobotic.convert import *
from src.RmRobotic.robotic_arm_package.log_setting import CommonLog

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)
from scipy.spatial.transform import Rotation
import socket


def extract_euler_angles(transform_matrix):
    # Extract rotation matrix from the homogeneous transformation matrix
    rotation_matrix = transform_matrix[:3, :3]

    # Calculate rotation angles
    # Assuming ZYX Euler angles order (yaw, pitch, roll)
    # Note: The order of Euler angles depends on your convention
    # If your convention is different, adjust accordingly
    # pitch = np.arcsin(rotation_matrix[2, 0])  # Pitch angle
    # roll = np.arctan2(-rotation_matrix[2, 1], rotation_matrix[2, 2])  # Roll angle
    # yaw = np.arctan2(-rotation_matrix[1, 0], rotation_matrix[0, 0])  # Yaw angle
    # return np.array([yaw, pitch, roll])
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles = r.as_euler('xyz', degrees=True)
    return euler_angles


def rm_pick(grippers):
    # euler_angles = extract_euler_angles(grippers[0])
    euler_angles = extract_euler_angles(grippers[1].rotation_matrices)
    translations = grippers[1].translations

    return translations, euler_angles


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


def cb_target_pose(arm, coordinate, angle_radians):
    # arm_init_pose = [2.171,-11.541,71.654,-5.137,105.673,182.688]    #RM65
    # arm_init_pose = [1.057,-21.371,66.662,-2.621,109.632,30.636]    #视觉版
    arm_init_pose = [-1.826, -35.606, 68.662, 0.621, 118.632, 177.636]  # 视觉版

    # ret = arm.Movej_Cmd(arm_init_pose, 20)

    # arm.Set_Gripper_Release(500) #张开夹爪

    # if grippers is not None:
    # translation, euler_angles = rm_pick(grippers)

    # x, y, z = translation[0][0], translation[0][1], translation[0][2]
    #
    # rx, ry, rz = euler_angles[0][0], euler_angles[0][1], euler_angles[0][2]

    # x = grippers[0, 3]
    # y = grippers[1, 3]
    # z = grippers[2, 3]
    x = coordinate[0]
    y = coordinate[1]
    z = coordinate[2]
    # xyzw = R.from_matrix(grippers[0:3, 0:3]).as_quat()
    # print(xyzw)
    #
    # rx = xyzw[0]
    # ry = xyzw[1]
    # rz = xyzw[2]
    # rw = xyzw[3]

    # joint, current_pose, error_code = arm.Get_Current_Arm_State()  # 获取当前机械臂状态
    error_code, joint, curr_pose, _, _ = arm.Get_Current_Arm_State()  # 获取当前机械臂状态
    print('curr_pose', curr_pose)
    # print('-----------------------------', curr_pose)
    # obj_pose = convert(x, y, z, rx, ry, rz, *curr_pose)
    # obj_pose = [i for i in obj_pose]
    # logger_.info(f'相机识别的物体位姿是：{obj_pose}')
    #
    # # obj_pose[3:] = current_pose[3:]
    #
    # # logger_.info(f'相机识别的物体位姿是：{obj_pose}')
    # # obj_pose[0]=0.0
    # # obj_pose[1]=0.0
    # # obj_pose[2]=0.0
    #
    # arm.Movej_P_Cmd(obj_pose,v=20)
    # # arm.movej(obj_pose)
    # obj_pose = convert(coordinate[0],coordinate[1],coordinate[2], rx, ry, rz, *curr_pose)
    # obj_pose = convert(x,y,z, curr_pose[0], curr_pose[1], angle_radians, *curr_pose)
    obj_pose = convert(x, y, z, *curr_pose)
    obj_pose = [i for i in obj_pose]
    # a = arm.Movej_P_Cmd(obj_pose, v=10)
    # joint[5] = angle_radians + 90
    # ret = arm.Movej_Cmd(joint, 20)

    print(f'obj_pose:{obj_pose}')

    # error_code, joint, curr_pose, _, _ = arm.Get_Current_Arm_State()  # 获取当前机械臂状态

    init_pose_ = chage_pose(obj_pose, -0.25)
    # print(f'init_pose_:{init_pose_}')

    # arm.release()

    # client = socket.socket()
    client = socket.socket()
    port = 8080
    host = "192.168.1.18"
    client.connect((host, port))

    pos_str = str(obj_pose[0]) + ',' + str(obj_pose[1]) + ',' + str(obj_pose[2] - 0.15) + ',' + str(
        obj_pose[3]) + ',' + str(obj_pose[4]) + ',' + str(obj_pose[5])

    # print('pos_str',pos_str)

    joint_canfd_cmd = '{"command":"movep_canfd","pose":[' + pos_str + '],"follow":True}\r\n'

    num = 0
    while num < 20:
        #
        client.send(joint_canfd_cmd.encode('utf-8'))
        # speed = client.recv(1024).decode()
        # ret = client.recv(1024)
        # print(ret)
        # feedback_data = ret.decode('utf-8').strip()
        # print(feedback_data)
        num += 1
        # a = arm.Movep_CANFD(obj_pose,follow=False)
    # a = arm.Movej_P_Cmd(init_pose_, v=10)

    # finally_pose = chage_pose(obj_pose, -0.11)
    # finally_pose = chage_pose(obj_pose, -0.13)
    #
    # print(f'finally_pose:{finally_pose}')
    #
    # arm.Movej_P_Cmd(finally_pose, v=10)
    # ret = arm.Set_Gripper_Pick_On(500, 500)  #加紧夹爪

    # arm.clamping()

    # finally_pose[2] += 0.05
    # arm.Movej_P_Cmd(finally_pose, v=10)
    # ret = arm.Set_Gripper_Pick_On(500, 500)  #加紧夹爪
    # ret = arm.Movej_Cmd(arm_init_pose, 20)
