import numpy as np
import math

from SimuEnv_Rigid.LegModel.RLforPath import LegPath
# -----------------------------------------------------------
from SimuEnv_Rigid.LegModel.legs import LegModel


class MouseController(object):
    """docstring for MouseController"""

    def __init__(self, SteNum, spine_angle, ):
        super(MouseController, self).__init__()
        PI = np.pi
        self.curStep = 0  # Spine

        # Spine A = 0
        # self.turn_F = 0*PI/180
        # self.turn_H = 8*PI/180
        # Spine A = 20
        self.turn_F = 0 * PI / 180
        self.turn_H = 12 * PI / 180  # 12
        self.pathStore = LegPath()
        # [LF, RF, LH, RH]
        # --------------------------------------------------------------------- #
        # self.phaseDiff = [0, PI, PI*1/2, PI*3/2]	# Walk
        # self.period = 3/2
        # self.SteNum = 36							#32 # Devide 2*PI to multiple steps
        # self.spinePhase = self.phaseDiff[3]
        # --------------------------------------------------------------------- #
        self.phaseDiff = [0, PI, PI, 0]  # Trot
        self.period = 2 / 2
        self.SteNum = SteNum
        # self.fre_cyc = 1 / (self.SteNum * dt )  # 1.25  # 0.80 ??
        print("----> ", self.SteNum)
        self.spinePhase = self.phaseDiff[3]
        # --------------------------------------------------------------------- #
        self.spine_A = 2 * spine_angle  # 10 a_s = 2theta_s
        print("angle --> ", spine_angle)  # self.spine_A)
        self.spine_A = self.spine_A * PI / 180
        # --------------------------------------------------------------------- #
        leg_params = [0.031, 0.0128, 0.0118, 0.040, 0.015, 0.035]
        self.fl_left = LegModel(leg_params)
        self.fl_right = LegModel(leg_params)
        self.hl_left = LegModel(leg_params)
        self.hl_right = LegModel(leg_params)
        # --------------------------------------------------------------------- #
        self.stepDiff = [0, 0, 0, 0]
        for i in range(4):
            self.stepDiff[i] = int(self.SteNum * self.phaseDiff[i] / (2 * PI))
        self.stepDiff.append(int(self.SteNum * self.spinePhase / (2 * PI)))
        # self.trgXList = [[], [], [], []]
        # self.trgYList = [[], [], [], []]

    def getLegCtrl(self, leg_M, curStep, leg_ID, ratio=1.0):
        curStep = curStep % self.SteNum
        turnAngle = self.turn_F
        leg_flag = "F"
        if leg_ID > 1:
            leg_flag = "H"
            turnAngle = self.turn_H
        radian = 2 * np.pi * curStep / self.SteNum
        # currentPos = self.pathStore.getRectangle(radian, leg_flag)
        currentPos = self.pathStore.getOvalPathPoint(radian, leg_flag, self.period, ratio=ratio)
        trg_x = currentPos[0]
        trg_y = currentPos[1]
        # self.trgXList[leg_ID].append(trg_x)
        # self.trgYList[leg_ID].append(trg_y)

        tX = math.cos(turnAngle) * trg_x - math.sin(turnAngle) * trg_y;
        tY = math.cos(turnAngle) * trg_y + math.sin(turnAngle) * trg_x;
        qVal = leg_M.pos_2_angle(tX, tY)
        return qVal

    def getSpineVal(self, spineStep):
        temp_step = int(spineStep)  # / 5)
        radian = 2 * np.pi * temp_step / self.SteNum
        return self.spine_A * math.cos(radian - self.spinePhase)

    # spinePhase = 2*np.pi*spineStep/self.SteNum
    # return self.spine_A*math.sin(spinePhase)

    def runStep(self, ActionSignal):
        foreLeg_left_q = self.getLegCtrl(self.fl_left,
                                         self.curStep + self.stepDiff[0], 0, ratio=ActionSignal[0])
        foreLeg_right_q = self.getLegCtrl(self.fl_right,
                                          self.curStep + self.stepDiff[1], 1, ratio=ActionSignal[1])
        hindLeg_left_q = self.getLegCtrl(self.hl_left,
                                         self.curStep + self.stepDiff[2], 2, ratio=ActionSignal[2])
        hindLeg_right_q = self.getLegCtrl(self.hl_right,
                                          self.curStep + self.stepDiff[3], 3, ratio=ActionSignal[3])

        spineStep = self.curStep  # + self.stepDiff[4]
        spine = self.getSpineVal(spineStep)
        # spine = 0
        self.curStep = (self.curStep + 1) % self.SteNum

        ctrlData = []
        # foreLeg_left_q = [1,0]
        # foreLeg_right_q = [1,0]
        # hindLeg_left_q = [-1,0]
        # hindLeg_right_q = [-1,0]
        ctrlData.extend(foreLeg_left_q)
        ctrlData.extend(foreLeg_right_q)
        ctrlData.extend(hindLeg_left_q)
        ctrlData.extend(hindLeg_right_q)
        for i in range(3):
            ctrlData.append(0)
        ctrlData.append(spine)
        return ctrlData
