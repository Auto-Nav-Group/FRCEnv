from frcenv import FRCEnv
import torch as th
th.autograd.set_detect_anomaly(True)

import warnings
warnings.filterwarnings("ignore")

from her_replay import HerReplayBuffer
from stable_baselines3 import SAC

LEARNING_STEPS = 100000

env = FRCEnv()

her_kwargs = dict(n_sampled_goal=4, goal_selection_strategy='future', env=env)

model = SAC('MultiInputPolicy', env, replay_buffer_class=HerReplayBuffer,
            replay_buffer_kwargs=her_kwargs, verbose=1,
            tensorboard_log="runs",
            buffer_size=int(1e6),
            learning_rate=1e-4,
            gamma=0.99, batch_size=1024, tau=0.05,
            policy_kwargs=dict(net_arch=[512, 512, 512]))

model.learn(int(LEARNING_STEPS))
model.save("G:/Projects/AutoNav/FRCEnv/out/sac.zip")