# from RatEnv.RL_wrapper2_MujoEnv_Compare import RatRL
from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
import gym
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import A2C
import warnings

RENDER_TRAIN = False

if __name__ == '__main__':
    # SceneFile = "../models/dynamic_4l.xml"
    # NAME = "S0_PPO_001"

    SceneFile = "../models/Scenario1_Planks.xml"
    NAME = "S1_PPO_002"

    # SceneFile = "../models/Scenario2_Uphill.xml"
    # NAME = "S2_PPO_114"

    # SceneFile = "../models/Scenario3_Logs.xml"
    # NAME = "S3_PPO_091"

    # SceneFile = "../models/Scenario4_Stairs.xml"
    # NAME = "S4_PPO_096"

    # warnings.filterwarnings("ignore")  # skip
    env = RatRL(SceneFile, render=RENDER_TRAIN)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./Local_Rat_Tensorboard/")
    # model = SAC("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    # model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    model.learn(total_timesteps=2_000_000, tb_log_name=NAME, reset_num_timesteps=True)
    model.save("./Local_Data/" + NAME)

    # del model
