import numpy as np
import matplotlib.pyplot as plt
# from SimuEnv_Rigid.LegModel.RLforPath import LegPath
from SimuEnv_Rigid.LegModel.forPath import LegPath
# -----------------------------------------------------------
from SimuEnv_Rigid.LegModel.legs import LegModel
from SimuEnv_Rigid.Controller import MouseController


if __name__ == '__main__':
    leg_params = [0.031, 0.0128, 0.0118, 0.040, 0.015, 0.035]
    FL = LegModel(leg_params)
    Range_q1 = np.arange(-1.57, 1.57, 0.01)
    Range_q2 = np.arange(-1.57, 1.57, 0.01)
    Poses = []


    for q1 in Range_q1:
        for q2 in Range_q2:
            pos = FL.angel_2_pos(q1, q2)
            if pos:
                # print("Yes:q1={}, q2={}".format(q1, q2))
                B = pos[1]
                C = pos[2]
                D = pos[3]
                # if ( C[1]*(B[0]-D[0]) - D[1]*(B[0]-C[0]) )/(C[0]-D[0]) < B[1]:
                #     Poses.append(pos[-1])
                Poses.append(pos[-1])
        #     else:
        #         # print("No:q1={}, q2={}".format(q1, q2))
        # pass

    Poses = np.array(Poses)
    fig = plt.figure()
    ax = plt.axes()
    ax.set_xlabel("y")
    ax.set_ylabel("z")
    ax.scatter(Poses[:,0], Poses[:,1])
    ax.set_aspect("equal")
    fig.show()

    # pos = np.array(pos)
    # fig = plt.figure()
    # ax = plt.axes()
    # ax.set_xlabel("y")
    # ax.set_ylabel("z")
    # ax.plot(pos[:,0], pos[:,1], '.o')
    # fig.show()

    # leg_M.pos_2_angle(tX, tY)

    pathStore = LegPath()
    fre = 0.67
    dt = 0.01
    n_frames = int(dt / 0.002)
    theController = MouseController(fre, dt, 0)
    theController.pathStore.para_FU = [[0.01, -0.035], [0.02, 0.015]]
    theController.pathStore.para_FD = [[0.01, -0.035], [0.02, 0.005]]
    theController.pathStore.para_HU = [[0.002, -0.055], [0.02, 0.015]]
    theController.pathStore.para_HD = [[0.002, -0.055], [0.02, 0.005]]
    # theController.pathStore.para_FU = [[-0.005, -0.05], [0.015, 0.02]]
    # theController.pathStore.para_FD = [[-0.005, -0.05], [0.015, 0.005]]
    # theController.pathStore.para_HU = [[0.0125, -0.055], [0.015, 0.02]]
    # theController.pathStore.para_HD = [[0.0125, -0.055], [0.015, 0.005]]
    theController.turn_H = 0 * np.pi / 180
    for _ in range(theController.SteNum):
        ctrlData = theController.runStep()  # No Spine
    ax.plot(theController.trgXList[0], theController.trgYList[0], 'r')
    ax.plot(theController.trgXList[2], theController.trgYList[2], 'r')
    fig.show()
