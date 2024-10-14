
import numpy as np
from scipy.spatial.transform import Rotation as R
def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]

    return H
def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz@Ry@Rx  # 先绕 z轴旋转 再绕y轴旋转  最后绕x轴旋转
    return R


def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）

    """
    # #D435:
    # rotation_matrix = np.array([
    #     [0.01401726 , -0.99981211 , 0.01338879],
    #     [0.99990154 , 0.01400729 ,-0.00083854],
    #     [ 0.00065084 , 0.01339923 , 0.99991001]])
    #
    # translation_vector = np.array([ 0.09592796, -0.03548515, -0.01668493])

    # # D405:
    # rotation_matrix = np.array([
    #     [ 0.0106455 , -0.99930429 ,-0.03574355],
    #     [0.99544663 , 0.01397702 ,-0.09429029],
    #     [ 0.09472428 ,-0.03457703 , 0.99490288]])
    # translation_vector = np.array([ 0.05371584, 0.03373042, 0.00536736])

    #无序抓取标定结果：
    # rotation_matrix = np.array([  # 05271511
    #     [0.03429764, -0.99941061, -0.00145184],
    #     [0.99939739, 0.03430487, -0.00529531],
    #     [0.00534199, -0.00126935, 0.99998493]])

    # q = np.array([0.09421105, -0.03133249, 0.02143211])  # 05271511


    # rotation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # 05271511
    # translation_vector = np.array([0.0942, -0.0341, 0.0255])  # 05271511



    # 65-B——1635
    # rotation_matrix = np.array([
    #     [0.00440894, -0.99997927, 0.00469353],
    #     [0.99998694,  0.00442102,  0.00256528],
    #     [-0.00258598,  0.00468216,  0.99998569]])
    #
    # translation_vector = np.array([0.09219051, -0.0373679, 0.02840859])

    # rotation_matrix = np.array([[-0.49080823, 0.85367428, -0.17420535],
    #                             [-0.87122698, -0.47894361, 0.10759447],
    #                             [0.00841609, 0.20458066, 0.97881353]])
    # translation_vector = np.array([-0.0406866, 0.06493766, 0.00491178])


    rotation_matrix = np.array([[0.10502558, -0.98873205, 0.10667037],
                                [0.99133807, 0.0955853, -0.09006818],
                                [0.07885718, 0.11520586, 0.99020662]])
    translation_vector = np.array([-0.02975279, 0.01143345, 0.13394508])


    # #63_B相机标定结果：
    # rotation_matrix = np.array([  # 05271511
    #     [-0.01977814 ,-0.99939789 , 0.02850775],
    #     [0.99967816 ,-0.02022069, -0.01532003],
    #     [0.01588725 , 0.02819557 , 0.99947617]])
    #
    # translation_vector = np.array([0.09194169, -0.02892147, 0.02716095])  # 05271511

    # 深度相机识别物体返回的坐标
    obj_camera_coordinates = np.array([x, y, z])

    # 机械臂末端的位姿，单位为弧度
    end_effector_pose = np.array([x1, y1, z1,
                                  rx, ry, rz])

    # 将旋转矩阵和平移向量转换为齐次变换矩阵
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector
    # print(T_camera_to_end_effector)
    # 机械臂末端的位姿转换为齐次变换矩阵
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()

    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position

    # 计算物体相对于机械臂基座的位姿
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
    # obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)

    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)

    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)

    obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标

    # 计算物体的旋转
    obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(rotation_matrix)
    obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)

    # 组合结果
    obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))

    obj_base_pose[3:] = rx, ry, rz

    return obj_base_pose
