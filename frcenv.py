import gymnasium as gym
from gymnasium import spaces



class FRCEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self, render_mode=None):
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        self.observation_space = spaces.Dict(
            {
                "observation" : spaces.Box(low=-1, high=1, shape=(2,)),
                "target_observation" : spaces.Box(low=-1, high=1, shape=(2,)),
            }
        )
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.state = None

    def _get_obs(self):

    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        pass