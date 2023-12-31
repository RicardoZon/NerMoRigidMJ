# from RatEnv.RL_wrapper2_MujoEnv_Compare import RatRL
from SimuEnv_Rigid.RL_wrapper2_Dir_NewMJ import RatRL
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
    # NAME = "S0_PPO_Native_021"
    # NAME = "S0_SAC_Native_022"
    # NAME = "S0_A2C_Native_023"

    # SceneFile = "../models/Scenario1_Planks.xml"
    # NAME = "S1_PPO_Native_028"
    # NAME = "S1_SAC_Native_032"
    # NAME = "S1_A2C_Native_024"

    # SceneFile = "../models/Scenario2_Uphill.xml"
    # NAME = "S2_PPO_Native_029"
    # NAME = "S2_SAC_Native_033"
    # NAME = "S2_A2C_Native_025"

    # SceneFile = "../models/Scenario3_Logs.xml"
    # NAME = "S3_PPO_Native_030"
    # NAME = "S3_SAC_Native_034"
    # NAME = "S3_A2C_Native_026"

    SceneFile = "../models/Scenario4_Stairs.xml"
    # NAME = "S4_PPO_Native_031"
    NAME = "S4_SAC_Native_035"
    # NAME = "S4_A2C_Native_027"

    # warnings.filterwarnings("ignore")  # skip
    checkpoint_callback = CheckpointCallback(
        save_freq=200_000,
        save_path="./Local_Logs/"+NAME,
        name_prefix="NAME",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )
    env = RatRL(SceneFile, render=RENDER_TRAIN)
    # model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    model = SAC("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    # model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    model.learn(total_timesteps=2_000_000, tb_log_name=NAME, reset_num_timesteps=True,
                callback=checkpoint_callback)
    model.save("./Local_Data/" + NAME)
    # del model
