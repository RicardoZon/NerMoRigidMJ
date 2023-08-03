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
    # MODELPATH = "Local_Data/S0_PPO_024"

    SceneFile = "../models/Scenario1_Planks.xml"
    MODELPATH = "Local_Data/S1_PPO_041"

    # SceneFile = "../models/Scenario2_Uphill.xml"
    # MODELPATH = "Local_Data/S2_PPO_005"

    # SceneFile = "../models/Scenario3_Logs.xml"
    # MODELPATH = "Local_Data/S3_PPO_006"

    # SceneFile = "../models/Scenario4_Stairs.xml"
    # MODELPATH = "Local_Data/S4_PPO_042"

    Recorder = DATA_Recorder()
    env = RatRL(SceneFile, fre_cyc = 1.5, render=RENDER, recorder=True)
    # Num010
    # env.Controller.pathStore.para_FU = [[0.01, -0.025], [0.015, 0.02]]
    # env.Controller.pathStore.para_FD = [[0.01, -0.025], [0.015, 0.005]]
    # env.Controller.pathStore.para_HU = [[0.005, -0.045], [0.015, 0.02]]
    # env.Controller.pathStore.para_HD = [[0.005, -0.045], [0.015, 0.005]]
    env.Controller.pathStore.para_FU = [[0.01, -0.035], [0.015, 0.015]]
    env.Controller.pathStore.para_FD = [[0.01, -0.035], [0.015, 0.005]]
    env.Controller.pathStore.para_HU = [[-0.005, -0.055], [0.015, 0.015]]
    env.Controller.pathStore.para_HD = [[-0.005, -0.055], [0.015, 0.005]]
    # env.Controller.turn_H = 0 * np.pi / 180
    model = PPO.load(MODELPATH, env=env)

    # env = gym.make("Ant-v2")
    # model = PPO.load("data/PPO_Ant_006", env=env)
    # mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=4)

    # Enjoy trained agent
    vec_env = model.get_env()
    obs = vec_env.reset()
    pos_Ori = vec_env.envs[0].pos[1]
    pos_end = []
    for i in range(int(10000)):
        pos_pre = vec_env.envs[0].pos[1]

        # action, _states = model.predict(obs, deterministic=True)
        action = [[1., 1., 1., 1.]]
        obs, rewards, dones, info = vec_env.step(action)
        print(action)
        # print(info)
        # print(vec_env.envs[0].pos)
        # vec_env.render()
        # Recorder.update(vec_env.envs[0])

        if dones[0]:
            pos_end.append(pos_pre)
            print(pos_pre)
            break

    times = np.array(vec_env.envs[0].episode_lengths)* vec_env.envs[0].dt
    v_global = -(np.array(pos_end) - pos_Ori) / np.array(times)
    # print(v_global.mean())

    # Recorder.savePath_Basic(MODELPATH.split('/')[-1])
    vec_env.envs[0].Recorder.savePath_Basic(MODELPATH.split('/')[-1])


