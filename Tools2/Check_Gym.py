from stable_baselines3.common.env_checker import check_env
# from RatEnv.RL_wrapper3_Connect import RatRL
from RatEnv.Wrapper_Dumpped.RL_wrapper2 import RatRL

if __name__ == '__main__':
    SceneFile = "../models/dynamic_4l_t3.xml"
    SceneName = "PlaneNPPO"  # Plane S0

    env = RatRL(SceneFile, Render=True)
    # It will check your custom environment and output additional warnings if needed
    check_env(env)