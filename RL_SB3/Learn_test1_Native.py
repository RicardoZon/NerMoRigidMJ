# from RatEnv.RL_wrapper2_MujoEnv_Compare import RatRL
# from RatEnv.RL_wrapper2_Dir import RatRL
from Rat_Env_Rigid.RL_wrapper2_Dir_NewMJ import RatRL
import gym
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import A2C
import warnings

RENDER_TRAIN = False

if __name__ == '__main__':
    SceneFile = "../models/dynamic_4l.xml"
    NAME = "S0_PPO_Native_001"
    # NAME = "S0_PPO_NativeStime_110"
    # NAME = "S0_SAC_NativeStime_104"
    # NAME = "S0_A2C_NativeStime_105"

    # SceneFile = "../models/scene_test1.xml"  # S1
    # NAME = "S1_PPO_NativeStime_083"
    # SceneFile = "../models/scene_test2.xml"  # S2  Spe
    # NAME = "S2_PPO_Native_114"
    # SceneFile = "../models/scene_test3.xml"  # S3
    # NAME = "S3_PPO_Native_091"

    # SceneFile = "../models/scene_S4_stair.xml"  # S4
    # NAME = "S4_PPO_Native_096"

    warnings.filterwarnings("ignore")
    env = RatRL(SceneFile, render=RENDER_TRAIN)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    # model = SAC("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    # model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./Rat_Tensorboard/")
    model.learn(total_timesteps=2_000_000, tb_log_name=NAME, reset_num_timesteps=True)
    model.save("./data/" + NAME)

    # del model
