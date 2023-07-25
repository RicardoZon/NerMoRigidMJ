from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
import gym
from stable_baselines3 import PPO
# from stable_baselines3 import SAC
# from stable_baselines3 import A2C
import numpy as np

from stable_baselines3.common.evaluation import evaluate_policy
from Tools.DataRecorder import DATA_Recorder

RENDER = True

if __name__ == '__main__':
    SceneFile = "../models/dynamic_4l.xml"
    # SceneFile = "../models/Scenario1_Planks.xml"
    # SceneFile = "../models/Scenario2_Uphill.xml"
    # SceneFile = "../models/Scenario3_Logs.xml"
    # SceneFile = "../models/Scenario4_Stairs.xml"
    Recorder = DATA_Recorder()

    env = RatRL(SceneFile, render=RENDER)
    obs = env.reset()
    action = [1., 1., 1., 1.]
    for _ in range(10000):
        obs, rewards, dones, info = env.step(action)
        print(info)
