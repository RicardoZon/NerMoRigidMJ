# from RatEnv.RL_wrapper2_MujoEnv_Compare import RatRL
from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
import gym
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import A2C
import warnings
from stable_baselines3.common.callbacks import CheckpointCallback

RENDER_TRAIN = False

if __name__ == '__main__':
    # SceneFile = "../models/dynamic_4l.xml"
    # NAME = "S0_PPO_003"

    # SceneFile = "../models/Scenario1_Planks.xml"
    # NAME = "S1_PPO_004"

    # SceneFile = "../models/Scenario2_Uphill.xml"
    # NAME = "S2_PPO_005"

    # SceneFile = "../models/Scenario3_Logs.xml"
    # NAME = "S3_PPO_006"

    SceneFile = "../models/Scenario4_Stairs.xml"
    NAME = "S4_PPO_007"

    # warnings.filterwarnings("ignore")  # skip
    checkpoint_callback = CheckpointCallback(
        save_freq=200_000,
        save_path="./Local_Logs/",
        name_prefix="NAME",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )
    env = RatRL(SceneFile, render=RENDER_TRAIN)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    # model = SAC("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    # model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    model.learn(total_timesteps=2_000_000, tb_log_name=NAME, reset_num_timesteps=True,
                callback=checkpoint_callback)
    model.save("./Local_Data/" + NAME)
    # del model
