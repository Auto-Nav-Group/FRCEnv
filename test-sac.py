from frcenv import FRCEnv
from wandb.integration.sb3 import WandbCallback
import wandb
import torch as th
th.autograd.set_detect_anomaly(True)

import warnings
warnings.filterwarnings("ignore")

#from her_replay import HerReplayBuffer
from mixed_precision_sac import SAC
from stable_baselines3.her.her_replay_buffer import HerReplayBuffer

th.backends.cuda.matmul.allow_tf32 = True
th.backends.cudnn.allow_tf32 = True

LEARNING_STEPS = 150000

env = FRCEnv()

#run = wandb.init(project="frc-env", entity="auto-nav-group", sync_tensorboard=True)

her_kwargs = dict(n_sampled_goal=4, goal_selection_strategy='future', env=env)

model = SAC('MultiInputPolicy', env, replay_buffer_class=HerReplayBuffer,
            replay_buffer_kwargs=her_kwargs, verbose=1,
            tensorboard_log="runs",
            buffer_size=int(1e6),
            learning_rate=1e-3,
            gamma=0.99, batch_size=8192, tau=0.05,
            policy_kwargs=dict(net_arch=[2048, 2048, 2048, 2048, 2048, 2048]),
            use_amp=True)

model.learn(int(LEARNING_STEPS))#, callback=WandbCallback(gradient_save_freq=1000, model_save_path=f"models/{run.id}"))
model.save("G:/Projects/AutoNav/FRCEnv/out/sac.zip")
#run.finish()