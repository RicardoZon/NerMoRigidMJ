# from RatEnv.RL_wrapper3_Connect import RatRL
# from RatEnv_Rigid.RL_wrapper2_Dir_NewMJ import RatRL
from SimuEnv_Rigid.RL_wrapper3_Connect import RatRL
# from RatEnv.RL_wrapper2 import RatRL
import gym
from stable_baselines3 import PPO
# from stable_baselines3 import SAC
# from stable_baselines3 import A2C
import numpy as np

from stable_baselines3.common.evaluation import evaluate_policy
from Tools.DataRecorder import DATA_Recorder

RENDER = True

if __name__ == '__main__':
    # SceneFile = "../models/dynamic_4l.xml"
    # MODELPATH = "Local_Data/S0_PPO_001"

    SceneFile = "../models/Scenario1_Planks.xml"
    MODELPATH = "Local_Data/S1_PPO_002"

    # SceneFile = "../models/scene_test1.xml"
    # MODELPATH = "data/S1_PPO_Native_056"
    # MODELPATH = "data/S1_PPO_NativeStime_083"
    # MODELPATH = "data/S1_SAC_NativeStime_084"
    # MODELPATH = "data/S1_A2C_NativeStime_085"

    # SceneFile = "../models/scene_test2.xml"  # S2
    # MODELPATH = "data/S2_PPO_Native_063"
    # # Failled up

    # SceneFile = "../models/scene_test2pro.xml"  # S2

    # SceneFile = "../models/scene_test3.xml"  # 3
    # MODELPATH = "data/S3_PPO_Native_058"

    # SceneFile = "../models/scene_S4_stair.xml"
    # MODELPATH = "data/S4_PPO_Native_072"

    Recorder = DATA_Recorder()

    env = RatRL(SceneFile, render=RENDER)
    model = PPO.load(MODELPATH, env=env)

    # env = gym.make("Ant-v2")
    # model = PPO.load("data/PPO_Ant_006", env=env)
    # mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=4)

    # Enjoy trained agent
    vec_env = model.get_env()
    obs = vec_env.reset()
    pos_Ori = vec_env.envs[0].pos[1]
    pos_end = []
    for i in range(int(10000/5)):
        pos_pre = vec_env.envs[0].pos[1]

        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = vec_env.step(action)
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

