import gym
from gym import spaces
import argparse

from RatEnv.ToSim import SimModel
from RatEnv.RL_Controller import MouseController
import matplotlib.pyplot as plt
import time
import numpy as np
from collections import deque

class RatRL(gym.Env):
    def __init__(self, SceneName, Render=False):
        super(RatRL, self).__init__()
        # self.action_space = spaces.
        # self.observation_space=
        self.SceneName = SceneName
        # self.reset(Render=Render) should reset yourself

        # Wrapper
        high = np.array([np.inf] * 12).astype(np.float32)
        self.action_space = spaces.Box(
            np.array([-1, -1, -1, -1]).astype(np.float32),
            np.array([1, 1, 1, 1]).astype(np.float32),
        )
        self.observation_space = spaces.Box(-high, high)
        # self._max_episode_steps = 50
        # self._max_episode_steps = 50*2  # V3_1 T/4
        self._max_episode_steps = 50 * 4  # V3_2 T/8  100*

        parser = argparse.ArgumentParser("Description.")
        parser.add_argument('--fre', default=0.67,
                            type=float, help="Gait stride")
        args = parser.parse_args()
        self.theMouse = SimModel(self.SceneName, Render=Render)
        self.theController = MouseController(args.fre)
        self.ActionIndex = 0
        self.MaxActIndex = len(self.theController.Action_Div)  # V3_1 4

        # Act_Per = int(self.SteNum / 2)
        # self.Action_Div = [Act_Per, self.SteNum-Act_Per]  # 186, 187
        # 3_1 piece 4
        # Act_Per = int(self.SteNum / 4)
        # self.Action_Div = [Act_Per, Act_Per, Act_Per, self.SteNum - Act_Per*3]  # 93, 93, 93, 94
        # 3_2 piece 8
        self.SteNum = 376  # 373+3
        self.Action_Div = [47, 47, 47, 47, 47, 47, 47, 47]  # 93, 93, 93, 94

    def reset(self):
        """将环境重置为初始状态，并返回一个初始状态；在环境中有随机性的时候，需要注意每次重置后要保证与之前的环境相互独立
        """
        # self.theMouse.sim.set_state(self.theMouse.sim_state)  # Go to initializing
        self.ActionIndex = 0
        self.theMouse.initializing()
        self._step = 0
        for i in range(500):
            ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
            self.theMouse.runStep(ctrlData)  # 此处有个内置的render
        #  sth in initializing should be done TODO
        self.theController.reset()
        self.States_Init()

        # print("Reset")

        # 是否需要hot up？ TODO
        action_hot = [1., 1., 1., 1.]
        self.done = False
        s, _, _, _ = self.step(action_hot)
        return s

    def States_Init(self):
        self.posY_Dir_pre = 0

        self.N_StepPerT = self.theController.SteNum  # 373
        # Connect Version
        self.vel_list = []

        self.vels = deque([])
        self.gyros = deque([])

        # For States
        self.action = [1., 1., 1., 1.]
        self.Action_Pre = [1., 1., 1., 1.]

    def render(self, mode='human'):
        self.theMouse.viewer.render()

    def close(self):
        """一些环境数据的释放可以在该函数中实现
        """
        pass

    def seed(self, seed=None):
        """设置环境的随机生成器
        """
        return

    def TimestepProcess(self):
        vel_dir = -list(self.theMouse.vel)[1]
        self.vel_list.append(vel_dir)

        vel = self.theMouse.vel
        self.vels.append(vel)
        gyro = self.theMouse.gyro
        self.gyros.append(gyro)


    def ActionProcess(self):
        # pos = self.theMouse.pos
        # posY_Dir = -pos[1]
        # reward = posY_Dir - self.posY_Dir_pre
        # self.posY_Dir_pre = posY_Dir
        #
        # self.Reward_Now = reward*10

        vels_mean = np.array(self.vels).mean(axis=0)
        gyros_mean = np.array(self.gyros).mean(axis=0)

        reward = -vels_mean[1] * 4

        # self.rat = self.FFTProcess(np.array(self.gyros).transpose()[1])
        # if self.rat < 0.6:
        #     reward = reward - 0.3

        self.Reward_Now = reward

        S = [self.Action_Pre, [self.ActionIndex],
             [reward],
             vels_mean, gyros_mean]
        # (4+1) + (1) + (3 +3) = 12
        S = [y for x in S for y in x]
        S = np.array(S)
        self.State_Now = S

        # Optional for test
        self.Vels_mean = vels_mean
        self.Gyros_mean = gyros_mean

    def GetMarkovNode(self):
        # 获得的是 Window_Cover 时刻之前的
        # r = self.Rewards_Base[0] + self.Rewards_Attach[0]
        # s = self.State_deque[0]

        # Simple
        r = self.Reward_Now
        s = self.State_Now

        if self._step > self._max_episode_steps:
            self.done = True  # 超过了一定步数就重置一下环境
            # print("Out")

        info = None
        return s, r, self.done, info

    def step(self, action, Render=False, LegCal=False):
        """环境的主要驱动函数，主逻辑将在该函数中实现。该函数可以按照时间轴，固定时间间隔调用

        参数:
            action (object): an action provided by the agent

        返回值:
            observation (object): agent对环境的观察，在本例中，直接返回环境的所有状态数据
            reward (float) : 奖励值，agent执行行为后环境反馈
            done (bool): 该局游戏时候结束，在本例中，只要自己被吃，该局结束
            info (dict): 函数返回的一些额外信息，可用于调试等
        """
        # ac = [] # rho, theta

        # 一次执行一个half周期
        index = self.ActionIndex
        for _ in range(self.Action_Div[index]):
            ActionSignal = (np.array(action) + 1.0) / 2

            tCtrlData = self.theController.runStep(ActionSignal)  # No Spine

            self.TimestepProcess()
            self.theMouse.runStep(tCtrlData, legposcal=LegCal)
            if Render:
                self.render()

        self.ActionProcess()

        self.ActionIndex = (self.ActionIndex + 1) % self.MaxActIndex  # Go to next Action Piece

        self._step = self._step + 1
        self.Action_Pre = action

        s, r, done, info = self.GetMarkovNode()
        # info = self.vel_list
        # print(np.mean(info))

        self.vel_list = []
        self.vels = deque([])
        self.gyros = deque([])

        return s, r, done, info

    def FFTProcess(self, data, Div=16, show=False, leg=None):
        T = 0.02
        Fs = 1 / T  # 采样频率
        L = len(data)
        n = L
        # if Div is None:
        #     Div = 25
        ncut = int(n / Div)  # 50/25 = 2 Hz
        f = np.linspace(0, Fs, n)

        out = np.fft.fft(data)
        power = abs(out) ** 2
        fcut = f[0:ncut]
        power = power[0:ncut]

        rat = power[0]/sum(power)
        return rat







