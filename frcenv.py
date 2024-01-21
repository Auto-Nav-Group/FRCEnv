import gymnasium as gym
import json
from gymnasium import spaces
from unity_env import FRCEngine
from frc_map import Map
import numpy as np
import math
np.seterr(all='raise')

ASSETS = "G:/Projects/AutoNav/FRCEnv/assets"
JSON = "G:/Projects/AutoNav/FRCEnv/assets/BasicMap.json"

class FRCEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self, render_mode=None):
        map = Map(json.loads(open(JSON, "r").read()))
        self.internal_env = FRCEngine()
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        self.observation_space = spaces.Dict(
            {
                "observation" : spaces.Box(low=-1, high=1, shape=(self.internal_env.state_size,)),
                "achieved_goal" : spaces.Box(low=-1, high=1, shape=(self.internal_env.state_size,)),
                "desired_goal": spaces.Box(low=-1, high=1, shape=(self.internal_env.state_size,)),
            }
        )
        assert render_mode is None or render_mode in self.metadata["render_modes"]

        self.state = None
        self.reward_weights = [
            -0.25, -0.25, -0, -0, -0, -0, -0.25, 0, -0, -0, -100, 100
        ]+self.internal_env.lidar_zeros

        self.top_reward = -math.inf
        self.achieved_goal= None

        self.ep_length = 0
        self.max_ep_length = 100

    def _get_obs(self, state):
        return {
            "observation" : self._normalize_state(state),
            "achieved_goal" : self._normalize_state(state),#self.achieved_goal,
            "desired_goal" : self.internal_env.target,
        }

    def _normalize_state(self, state):
        nstate = [
            (state[0] / np.pi),
            (state[1] / self.internal_env.max_mes),
            (2 * state[2] / self.internal_env.width),
            (2 * state[3] / self.internal_env.height),
            (2 * state[4] / self.internal_env.width),
            (2 * state[5] / self.internal_env.height),
            state[6],
            state[7],
            state[8] / self.internal_env.width,
            state[9] / self.internal_env.height,
            state[10],
            state[11]
        ]
        for i in range(len(state[12:])):
            nstate.append(state[12+i] / self.internal_env.max_mes)
        return nstate

    def _get_info(self, done):
        return {
            "is_success" : done
        }

    def step(self, action):
        state, collision, achieved_goal, done = self.internal_env.step(action)
        reward = self.compute_reward(state, self.internal_env.target, None)
        if reward > self.top_reward:
            self.top_reward = reward
            self.achieved_goal = state
        self.ep_length += 1
        if self.ep_length >= self.max_ep_length:
            done = True
            state[11] = 1
        out_state = self._get_obs(state)
        return out_state, reward, done, False, self._get_info(achieved_goal)

    def reset(self, seed=None, options=None):
        state, _, _, _ = self.internal_env.reset()
        self.achieved_goal= state
        out_state = self._get_obs(state)
        out_info = self._get_info(False)
        self.ep_length = 0
        return out_state, out_info

    def render(self):
        pass

    def close(self):
        pass

    def compute_reward(self, achieved_goal, desired_goal, info):
        a_goal = np.array(achieved_goal)
        d_goal = np.array(desired_goal)
        if len(a_goal.shape) == 1:  # Single goal
            reward = np.sum(self.reward_weights * np.abs(a_goal - d_goal))-1
        else:  # Batch of goals
            reward = np.subtract(np.sum(self.reward_weights * np.abs(np.subtract(a_goal, d_goal)), axis=1), 1)
        return reward