# from RatEnv.RL_wrapper2_MujoEnv_Compare import RatRL
from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
import gym
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import A2C
import warnings
import numpy as np
from stable_baselines3.common.callbacks import CheckpointCallback

RENDER_TRAIN = False

if __name__ == '__main__':
    # SceneFile = "../models/dynamic_4l.xml"
    # NAME = "S0_PPO_003"

    # SceneFile = "../models/Scenario1_Planks.xml"
    # NAME = "S1_PPO_018"

    # SceneFile = "../models/Scenario2_Uphill.xml"
    # NAME = "S2_PPO_015"

    # SceneFile = "../models/Scenario3_Logs.xml"
    # NAME = "S3_PPO_020"

    SceneFile = "../models/Scenario4_Stairs.xml"
    NAME = "S4_PPO_017"

    # warnings.filterwarnings("ignore")  # skip
    checkpoint_callback = CheckpointCallback(
        save_freq=200_000,
        save_path="./Local_Logs/",
        name_prefix="NAME",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )
    env = RatRL(SceneFile, fre_cyc = 1.5, render=RENDER_TRAIN)
    env.Controller.pathStore.para_FU = [[0.01, -0.025], [0.015, 0.02]]
    env.Controller.pathStore.para_FD = [[0.01, -0.025], [0.015, 0.005]]
    env.Controller.pathStore.para_HU = [[0.005, -0.045], [0.015, 0.02]]
    env.Controller.pathStore.para_HD = [[0.005, -0.045], [0.015, 0.005]]
    env.Controller.turn_H = 0 * np.pi / 180
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    # model = SAC("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    # model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    model.learn(total_timesteps=2_000_000, tb_log_name=NAME, reset_num_timesteps=True,
                callback=checkpoint_callback)
    model.save("./Local_Data/" + NAME)
    # del model
