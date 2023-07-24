import matplotlib.pyplot as plt
import numpy as np
import math
import mujoco
import mujoco.viewer as viewer
from SimuEnv_Rigid.Controller import MouseController
import time

class SimModel_fl(object):
    """docstring for SimModel"""

    def __init__(self, modelPath, render=False):
        super(SimModel_fl, self).__init__()
        self.model = mujoco.MjModel.from_xml_path(modelPath)
        self.data = mujoco.MjData(self.model)
        self._timestep = self.model.opt.timestep

        self.render = render
        if render:
            self.viewer = viewer.launch_passive(self.model, self.data)
            # self.viewer.cam.azimuth = 0
            # self.viewer.cam.lookat[0] += 0.25
            # self.viewer.cam.lookat[1] += -0.5
            # self.viewer.cam.distance = self.model.stat.extent * 0.5

        self.legPosName = [
            ["router_shoulder_fl", "foot_s_fl"],
            ["router_shoulder_fr", "foot_s_fr"],
            ["router_hip_rl", "foot_s_rl"],
            ["router_hip_rr", "foot_s_rr"]]
        self.fixPoint = "body_ss"  # "neck_ss"
        self.legRealPoint_x = [[], [], [], []]
        self.legRealPoint_y = [[], [], [], []]
        self.movePath = [[], [], []]

    def initializing(self):
        self.movePath = [[], [], []]
        self.legRealPoint_x = [[], [], [], []]
        self.legRealPoint_y = [[], [], [], []]

    def do_simulation(self, ctrl, n_frames):
        # ------------------------------------------ #
        # ID 0, 1 left-fore leg and coil
        # Note: For leg, it has [-1: front; 1: back]
        # Note: For fore coil, it has [-1: leg up; 1: leg down]
        # Note: For hide coil, it has [-1: leg down; 1: leg up]
        # Note: range is [-1, 1]
        # ------------------------------------------ #
        step_start = time.time()
        self.data.ctrl[:] = ctrl
        for _ in range(n_frames):
            mujoco.mj_step(self.model, self.data)
            if self.render:
                self.viewer.sync()

        # Data Record
        originPoint = self.data.site("leg_link_fl").xpos.copy()
        currentPoint = self.data.site("ankle_fl").xpos.copy()
        tX = currentPoint[1]-originPoint[1]  # Global: Y
        tY = currentPoint[2]-originPoint[2]  # Global: Z
        self.legRealPoint_x[0].append(tX)
        self.legRealPoint_y[0].append(tY)

        time_until_next_step = self._timestep * n_frames - (time.time() - step_start)
        # 	print(time_until_next_step)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    def getTime(self):
        return self.data.time

    def point_distance_line(self, point, line_point1, line_point2):
        # 计算向量
        vec1 = line_point1 - point
        vec2 = line_point2 - point
        distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
        return distance

    def drawPath(self):
        # print(self.movePath)
        path_X = self.movePath[0]
        path_Y = self.movePath[1]
        tL = len(path_X)

        """
        order = 3
        tY = []
        for i in range(tL):
            tY.append(-path_Y[i])
        parameter = np.polyfit(tY, path_X, order)
        smooth_x = []
        for i in range(tL):
            tVal = 0
            for j in range(order):
                tVal = tVal + parameter[order] * tY[i] ** j
            smooth_x.append(tVal)
        dis = 0
        for i in range(tL-1):
            dX = smooth_x[i]-smooth_x[i+1]
            dY = path_Y[i]-path_Y[i+1]
            dis = dis + math.sqrt(dX*dX + dY*dY)
        print("Dis --> ", dis)
        """

        ds = 1
        dL = int(tL / ds)
        check_x = []
        check_y = []
        print(tL)
        for i in range(dL):
            check_x.append(path_X[i * ds])
            check_y.append(path_Y[i * ds])

        check_x.append(path_X[-1])
        check_y.append(path_Y[-1])

        # dis = 50
        # for i in range(dL):
        #	dX = check_x[i]-check_x[i+1]
        #	dY = check_y[i]-check_y[i+1]
        #	dis = dis + math.sqrt(dX*dX + dY*dY)
        # dis = path_Y[0] - path_Y[-1]
        dX = path_X[0] - path_X[-1]
        dY = path_Y[0] - path_Y[-1]
        dis = math.sqrt(dX * dX + dY * dY)
        # dis = path_Y[0] - path_Y[-1]
        print("Dis --> ", dis)

        start_p = np.array([check_x[0], check_y[0]])
        end_p = np.array([check_x[-1], check_y[-1]])

        maxDis = 0
        for i in range(tL):
            cur_p = np.array([path_X[i], path_Y[i]])
            tDis = self.point_distance_line(cur_p, start_p, end_p)
            if tDis > maxDis:
                maxDis = tDis
        print("MaxDiff --> ", maxDis)
        plt.plot(path_X, path_Y)
        plt.plot(check_x, check_y)
        plt.grid()
        plt.show()

        return dis

    def savePath(self, flag):
        filePath = "Data/path_" + flag + ".txt"
        trajectoryFile = open(filePath, 'w')
        dL = len(self.movePath[0])
        for i in range(dL):
            for j in range(3):
                trajectoryFile.write(str(self.movePath[j][i]) + ' ')
            trajectoryFile.write('\n')
        trajectoryFile.close()


if __name__ == '__main__':
    RUN_TIME_LENGTH = 10  # seconds
    fre = 0.67

    FL = SimModel_fl("../models/fl_single.xml", render=True)
    dt = 0.002
    n_frames = int(dt / FL._timestep)
    run_steps_num = int(RUN_TIME_LENGTH / dt)
    theController = MouseController(fre, dt, 0)
    theController.pathStore.para_FU = [[-0.00, -0.045], [0.03, 0.01]]
    theController.pathStore.para_FD = [[-0.00, -0.045], [0.03, 0.005]]

    mujoco.mj_resetData(FL.model, FL.data)

    for i in range(10):  # 0.2 s
        ctrlData = [0.0, 1.0]
        # ctrlData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]  # Init
        FL.do_simulation(ctrlData, n_frames=1)
    FL.initializing()
    start = time.time()
    # q_pre = np.array(ctrlData)

    for i in range(run_steps_num):
        ctrlData = theController.runStep()  # No Spine
        ctrlData = ctrlData[0:2]
        # if i==0:
        #     dx = np.arange(q_pre[0], ctrlData[0], -0.01)
        #     np.array(ctrlData) - q_pre
        #     dy = np.interp(dx, [q_pre[0], ctrlData[0]], [q_pre[1], ctrlData[1]])
        #     inters = np.array([dx, dy]).transpose()
        #     for p in inters:
        #         FL.do_simulation(p, n_frames=1)
        FL.do_simulation(ctrlData, n_frames=n_frames)
    end = time.time()
    #
    timeCost = end - start

    fig = plt.figure()
    ax = plt.axes()
    ax.plot(theController.trgXList[0], theController.trgYList[0])
    ax.plot(FL.legRealPoint_x[0], FL.legRealPoint_y[0])
    fig.show()