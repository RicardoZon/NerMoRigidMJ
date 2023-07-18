# NerMoRigidMJ

Online 1st.


### models
For storing xml files and stl models of the rat robot.

### SimuEnv_Rigid
The MuJoCo simulation codes.
Together with the gym wrapper for RL training.

### RL_SB3
Training setups with stable baselines 3.



# Attention
### timestep setup
Because of the conflict of models when building the robot in MuJoCo, if the simulation timestep is larger, conflicting forces between components can lead to unusual robot behavior, like flying out by the tail. So now you have to keep the timestep=0.002s.
