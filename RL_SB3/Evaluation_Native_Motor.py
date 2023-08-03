# from RatEnv.RL_wrapper3_Connect import RatRL
from SimuEnv_Rigid.RL_wrapper2_Dir_NewMJ import RatRL
# from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
# from RatEnv.RL_wrapper2 import RatRL
import gym
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3 import A2C
import numpy as np

from stable_baselines3.common.evaluation import evaluate_policy
from Tools.DataRecorder import DATA_Recorder

RENDER = True

if __name__ == '__main__':
    # SceneFile = "../models/dynamic_4l.xml"
    # MODELPATH = "Local_Data/S0_PPO_Native_021"
    # MODELPATH = "Local_Data/S0_SAC_Native_022"
    # MODELPATH = "Local_Data/S0_A2C_Native_023"

    SceneFile = "../models/Scenario1_Planks.xml"
    # MODELPATH = "Local_Data/S1_PPO_Native_028"
    # MODELPATH = "Local_Data/S1_SAC_Native_032"
    # MODELPATH = "Local_Data/S1_A2C_Native_024"

    # SceneFile = "../models/Scenario2_Uphill.xml"

    # SceneFile = "../models/Scenario3_Logs.xml"

    # SceneFile = "../models/Scenario4_Stairs.xml"

    Recorder = DATA_Recorder()
    env = RatRL(SceneFile, render=RENDER)
    # model = PPO.load(MODELPATH, env=env)
    # model = SAC.load(MODELPATH, env=env)

    # Enjoy trained agent
    vec_env = model.get_env()
    obs = vec_env.reset()
    pos_Ori = vec_env.envs[0].pos[1]
    pos_end = []
    for i in range(int(10000)):
        pos_pre = vec_env.envs[0].pos[1]

        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = vec_env.step(action)
        print(action)
        # print(info)
        # print(vec_env.envs[0].pos)
        # vec_env.render()
        Recorder.update(vec_env.envs[0])

        if dones[0]:
            pos_end.append(pos_pre)
            print(pos_pre)

    times = np.array(vec_env.envs[0].episode_lengths)* vec_env.envs[0].dt
    v_global = -(np.array(pos_end) - pos_Ori) / np.array(times)
    # print(v_global.mean())

    # Recorder.savePath_Basic("S1_Pass_073")

