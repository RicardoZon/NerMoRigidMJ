import gym
from gym import spaces
import mujoco
import mujoco.viewer as viewer
from SimuEnv_Rigid.RL_Controller import MouseController
from Tools.DataRecorder import DATA_Recorder
import numpy as np
import matplotlib.pyplot as plt
import time
import argparse
from collections import deque
from gym.envs.mujoco import mujoco_env  # REF


class RatRL(gym.Env):
    """
    Sparse Version
    Action: angle of 8 motor
    Simplified from Wrapper V2
    """

    def __init__(self, xml_file: str, fre_cyc=0.67, render=False, recorder=False):
        super(RatRL, self).__init__()
        # Wrapper
        high = np.array([np.inf] * 28).astype(np.float32)  # 28: Rich sensor
        self.action_space = spaces.Box(
            np.array([-1., -1., -1., -1.]).astype(np.float32),
            np.array([+1., +1., +1., +1.]).astype(np.float32),
        )
        self.observation_space = spaces.Box(-high, high)
        self.model = mujoco.MjModel.from_xml_path(filename=xml_file)
        self.data = mujoco.MjData(self.model)
        self.xml_file = xml_file
        self.Render = render
        if render:
            # render must be called mannually
            self.viewer = viewer.launch_passive(self.model, self.data)
            self.viewer.cam.azimuth = 0
            self.viewer.cam.lookat[0] += 0.25
            self.viewer.cam.lookat[1] += -0.5
            self.viewer.cam.distance = self.model.stat.extent * 0.5
        if recorder:
            self.Recorder = DATA_Recorder()
        else:
            self.Recorder = None

        # Hyper Parameter
        self.frame_skip = 5
        self.Ndiv = 8  # divided by Ndiv pieces

        self._timestep = self.model.opt.timestep  # Default = 0.002s per timestep
        self.dt = self._timestep * self.frame_skip  # dt = 0.01s

        self.fre_cyc = fre_cyc  # 0.67  # 1.25 #0.80
        self.SteNum = int(1 / (self.dt * self.fre_cyc))
        self.N_cluster = (int(self.SteNum / 8)+1)  # [19]*8
        self.SteNum = self.N_cluster * self.Ndiv  # Update SteNum
        self.Controller = MouseController(self.SteNum, 0)  # X Spine
        self.ActionIndex = 0

        self._max_episode_steps = int(10000 * 0.002 / (self.dt * self.N_cluster))  # 10000 when timestep = 0.002s

        self.pos = None
        self.quat = None
        self.vel = None
        self.acc = None
        self.gyro = None
        self._step = None
        self.nair = 0

    def do_simulation(self, ctrl, n_frames):
        self.data.ctrl[:] = ctrl
        for _ in range(n_frames):
            mujoco.mj_step(self.model, self.data)
        if self.Render:
            self.viewer.sync()

    def reset(self):
        """将环境重置为初始状态，并返回一个初始状态；在环境中有随机性的时候，需要注意每次重置后要保证与之前的环境相互独立
        """
        # del self.sim
        # del self.model
        # self.model = load_model_from_path(self.xml_file)
        # self.sim = MjSim(self.model)
        self.ActionIndex = 0
        mujoco.mj_resetData(self.model, self.data)
        self._step = 0
        ctrlData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]
        self.do_simulation(ctrlData, n_frames=10)  # 此处有个内置的render
        self.pos = self.data.sensor("com_pos").data.copy()
        self.action = np.array([1.0, 1.0, 1.0, 1.0])
        self.done = False

        s, _, _, _ = self.step(self.action)
        self.Controller.curStep = 0
        return s

    def render(self, mode='human'):
        pass

    def close(self):
        """
        """
        pass

    def seed(self, seed=None):
        """设置环境的随机生成器
        """
        return

    def step(self, action):
        self.Y_Pre = self.pos[1]
        self.action_pre = self.action
        # action x4 [-1, +1]
        self.action = np.array(action)
        ActionSignal = (self.action + 1.0) * 0.5  # 0.0 to 1.0
        # print(self.ActionIndex)
        for _ in range(self.N_cluster):
            ctrlData = self.Controller.runStep(ActionSignal)
            self.do_simulation(ctrlData, n_frames=self.frame_skip)
            # if self.Recorder:
            #     # manually updata recorder
            #     # TODO: write it as a callback
            #     self.Recorder.imu_pos.append(self.data.sensor("com_pos").data.copy())
            #     self.Recorder.imu_quat.append(self.data.sensor("com_quat").data.copy())
            #     self.Recorder.imu_vel.append(self.data.sensor("com_vel").data.copy())
            #     self.Recorder.imu_acc.append(self.data.sensor("imu_acc").data.copy())
            #     self.Recorder.imu_gyro.append(self.data.sensor("imu_gyro").data.copy())
            #     self.Recorder.ctrldata.append(self.data.ctrl.copy())
        self._step = self._step + 1
        self.ActionIndex = (self.ActionIndex + 1) % self.Ndiv  # Go to next Action Piece

        # Sensor
        self.pos = self.data.sensor("com_pos").data.copy()
        self.quat = self.data.sensor("com_quat").data.copy()
        self.vel = self.data.sensor("com_vel").data.copy()
        self.acc = self.data.sensor("imu_acc").data.copy()
        self.gyro = self.data.sensor("imu_gyro").data.copy()
        # contact_sensor = [self.data.sensor("fl_t1").data[0].copy(), self.data.sensor("fr_t1").data[0].copy(),
        #                   self.data.sensor("rl_t1").data[0].copy(), self.data.sensor("rr_t1").data[0].copy()]
        # contact_sensor = (np.array(contact_sensor) != 0.0).astype(int)  # 1 for contact, 0 for air
        # sum_contact = sum(contact_sensor)
        # qpos
        # qposes = [
        #     self.sim.data.get_joint_qpos("knee1_fl"),
        #     self.sim.data.get_joint_qpos("ankle_fl"),4
        #     self.sim.data.get_joint_qpos("knee1_fr"),
        #     self.sim.data.get_joint_qpos("ankle_fr"),
        #     self.sim.data.get_joint_qpos("knee1_rl"),
        #     self.sim.data.get_joint_qpos("ankle_rl"),
        #     self.sim.data.get_joint_qpos("knee1_rr"),
        #     self.sim.data.get_joint_qpos("ankle_rr"),
        # ]

        # Rewards
        reward_forward = (self.pos[1] - self.Y_Pre) / (self.dt * self.N_cluster) * (-10)  # 2~4

        reward_trapped = 0.0
        # if reward_forward > 3.5:
        #     reward_trapped = -5.0  # 71 72

        # if sum_contact == 0:
        #     self.nair += 1
        #     # reward_trapped = -1.0  # weaken air time of front paws?
        # else:
        #     self.nair = 0
        # if self.nair == 10:  # 10 For 65 20 For 66
        #     # Trapped
        #     reward_trapped = -5.0  # 1.5?   15.0 Too Large For 51   5.0 For 52
        #     self.done = True
        # if self.pos[2] < 0.03:  # TODO
        #     reward_trapped = -5.0  # 1.5?   15.0 Too Large For 51   5.0 For 52
        #     self.done = True
        reward_bias = 0.  # -(self.pos[0] * 2) **2  # For 47  *4 For 46
        # # reward_holdon = 0. # 1. * self._step / self._max_episode_steps  #  X
        # reward_height = 0. # 25 * (#self.pos[2]-0.065)  # make jump and trap

        sum_delta_a = sum(abs(self.action - self.action_pre))
        control_cost = 0  # 0.05 * sum_delta_a

        self.Reward_Now = float(reward_forward + reward_trapped + reward_bias - control_cost)  # FLOAT 32

        # # SelfInfo of theta
        # # self.ActionIndex = 0.
        # S = [self.action,
        #      [reward_forward],
        #      self.vel, self.gyro, self.quat, contact_sensor]
        # # S = [self.action,
        # #      [reward_forward],
        # #      vel, gyro, self.quat, contact_sensor]
        # S = [y for x in S for y in x]
        S = self.data.sensordata.copy()  # rich state
        S = np.array(S).astype(np.float32)
        self.State_Now = S
        # print(S)

        # get markov node
        if self._step > self._max_episode_steps:
            self.done = True  # 超过了一定步数就重置一下环境
            # print("Out")
        if self.pos[1] < -2.0:
            self.done = True  # For validation
        info = {
            "reward_forward": reward_forward,
            "reward_bias": reward_bias,
            # "reward_holdon": reward_holdon,
            "sum_delta_a": sum_delta_a,
            # "touch": contact_sensor
        }
        # if not np.any(contact_sensor):
        # print("!!!!!!!!!! Zero!!!!!!!!!! Zero!!!!!!!!!! Zero!!!!!!!!!! Zero!!!!!!!!!! Zero!!!!!!!!!! Zero")

        return S, self.Reward_Now, self.done, info
