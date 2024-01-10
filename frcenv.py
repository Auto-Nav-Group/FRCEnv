import gymnasium as gym
import json
from gymnasium import spaces
from robotenv import RobotVEnv
from frc_map import Map
import numpy as np
import math
np.seterr(all='raise')

ASSETS = "G:/Projects/AutoNav/FRCEnv/assets"
JSON = "G:/Projects/AutoNav/FRCEnv/assets/BasicMap.json"

class FRCEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self, render_mode=None):
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        self.observation_space = spaces.Dict(
            {
                "observation" : spaces.Box(low=-1, high=1, shape=(10,)),
                "achieved_goal" : spaces.Box(low=-1, high=1, shape=(10,)),
                "desired_goal": spaces.Box(low=-1, high=1, shape=(10,)),
            }
        )
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        map = Map(json.loads(open(JSON, "r").read()))

        self.internal_env = RobotVEnv(map, ASSETS)
        self.state = None
        self.reward_weights = [
            0.2, 0.05, 0, 0,0,0,0.025,0.025,0,0
        ]
        self.top_reward = -math.inf
        self.achieved_goal= None

    def _get_obs(self, state):
        return {
            "observation" : state,
            "achieved_goal" : state,#self.achieved_goal,
            "desired_goal" : self.internal_env.target,
        }

    def _normalize_state(self, state):
        nstate = [
            (state[0] / np.pi),
            (state[1] / self.internal_env.max_mes),
            (2 * state[2] / self.internal_env.basis.size.width),
            (2 * state[3] / self.internal_env.basis.size.height),
            (2 * state[4] / self.internal_env.basis.size.width),
            (2 * state[5] / self.internal_env.basis.size.height),
            state[6],
            state[7],
            state[8] / self.internal_env.basis.size.width,
            state[9] / self.internal_env.basis.size.height
        ]
        return nstate

    def _get_info(self, done):
        return {
            "is_success" : done
        }

    def step(self, action):
        state, collision, done, achieved_goal, dist_traveled, min_dist = self.internal_env.step(action)
        reward = self.compute_reward(state, self.internal_env.target, None)
        if reward > self.top_reward:
            self.top_reward = reward
            self.achieved_goal = state
        out_state = self._get_obs(state)
        return out_state, reward, done, False, self._get_info(achieved_goal)

    def reset(self, seed=None, options=None):
        state, dist, min_dist = self.internal_env.reset()
        self.achieved_goal= state
        out_state = self._get_obs(state)
        out_info = self._get_info(False)
        return out_state, out_info

    def render(self):
        pass

    def close(self):
        pass

    def compute_reward(self, achieved_goal, desired_goal, info):
        a_goal = np.array(achieved_goal)
        d_goal = np.array(desired_goal)
        if len(a_goal.shape) == 1:  # Single goal
            reward = np.sum(self.reward_weights * np.abs(a_goal - d_goal) ** 2)
        else:  # Batch of goals
            reward = np.sum(self.reward_weights * np.abs(np.subtract(a_goal, d_goal)) ** 2, axis=1)
        return -reward