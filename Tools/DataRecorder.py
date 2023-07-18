import numpy as np
from collections import deque
import scipy.io as scio

class DATA_Recorder():
    def __init__(self):
        self.imu_pos = deque([])
        self.imu_quat = deque([])
        self.imu_vel = deque([])
        self.imu_acc = deque([])
        self.imu_gyro = deque([])

    def update(self, env):
        self.imu_pos.append(env.pos)
        self.imu_quat.append(env.quat)
        self.imu_vel.append(env.vel)
        self.imu_acc.append(env.acc)
        self.imu_gyro.append(env.gyro)

    def savePath_Basic(self, FileName):
        imu_pos = np.array(self.imu_pos)
        imu_quat = np.array(self.imu_quat)
        imu_vel = np.array(self.imu_vel)
        imu_acc = np.array(self.imu_acc)
        imu_gyro = np.array(self.imu_gyro)
        scio.savemat(FileName + '.mat', {'pos': imu_pos, 'quat': imu_quat, 'vel': imu_vel,
                                         'acc': imu_acc, 'gyro': imu_gyro})  # 写入mat文件

    def savePath2(self, FileName, theMouse, SELFINFO=False):
        # scio.savemat('mvpath.mat', {'H_range': self.movePath})  # 写入mat文件
        imu_pos = np.array(self.imu_pos)
        imu_quat = np.array(self.imu_quat)
        imu_vel = np.array(self.imu_vel)
        imu_acc = np.array(self.imu_acc)
        imu_gyro = np.array(self.imu_gyro)
        if SELFINFO:
            legRealPoint_x = np.array(theMouse.legRealPoint_x)
            legRealPoint_y = np.array(theMouse.legRealPoint_y)
            legRealPoint_x_ori = np.array(theMouse.legRealPoint_x_ori)
            legRealPoint_y_ori = np.array(theMouse.legRealPoint_y_ori)
            scio.savemat(FileName + '.mat', {'pos': imu_pos, 'quat': imu_quat, 'vel': imu_vel,
                                             'acc': imu_acc, 'gyro': imu_gyro,
                                             'legRealPoint_x': legRealPoint_x, 'legRealPoint_y': legRealPoint_y,
                                             'legRealPoint_x_ori': legRealPoint_x_ori,
                                             'legRealPoint_y_ori': legRealPoint_y_ori, })  # 写入mat文件
        else:
            scio.savemat(FileName + '.mat', {'pos': imu_pos, 'quat': imu_quat, 'vel': imu_vel,
                                             'acc': imu_acc, 'gyro': imu_gyro})  # 写入mat文件

    def savePath_TOSIM(self, FileName, theMouse):
        legRealPoint_x = np.array(theMouse.legRealPoint_x)
        legRealPoint_y = np.array(theMouse.legRealPoint_y)
        legRealPoint_x_ori = np.array(theMouse.legRealPoint_x_ori)
        legRealPoint_y_ori = np.array(theMouse.legRealPoint_y_ori)
        scio.savemat(FileName + '.mat', {'legRealPoint_x': legRealPoint_x, 'legRealPoint_y': legRealPoint_y,
                                         'legRealPoint_x_ori': legRealPoint_x_ori,
                                         'legRealPoint_y_ori': legRealPoint_y_ori, })  # 写入mat文件