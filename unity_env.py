import asyncio
import os

import numpy as np
import requests
import aiohttp
import json
from multiprocessing import shared_memory


class FRCEngine:

    def __init__(self):
        self.url = "http://localhost:5000"
        self.headers = {'Content-Type': 'application/json'}
        self.state_size = 112
        sizeinfo = self.get_size_info()
        self.width = sizeinfo[0]
        self.height = sizeinfo[1]
        self.max_mes = sizeinfo[2]
        self.lidar_zeros = [0 for i in range(self.state_size-12)]
        self.target = [0,0,0,0,0,0,0,0,0,0, 0, 0] + self.lidar_zeros
        self.int2bool = lambda x: True if x==1 else False
        self.use_shm = False
        self.use_hybrid_shm = False
        self.mem_id_in = ""
        self.mem_id_out = ""
        self.shm_in = None
        self.shm_out = None
        self.shm_len = 112
        if self.use_shm or self.use_hybrid_shm:
            self._setup()

    def _setup(self):
        response = requests.get(self.url+"/getmeminfo")
        j = response.json()["Items"]
        self.mem_id_in = j[0]
        self.mem_id_out = j[1]
        self.shm_in = r'\\.\pipe{}'.format(self.mem_id_in)
        self.shm_out = r'\\.\pipe{}'.format(self.mem_id_out)

    def get_size_info(self):
        response = requests.get(self.url+"/getsizeinfo")
        return response.json()["Items"]


    def step(self, action):
        if self.use_shm:
            with open(self.shm_in, 'w') as f:
                f.write(f"1 0 {action[0]} {action[1]}")
            state = np.fromfile(self.shm_out, dtype=np.float32)
        elif self.use_hybrid_shm:
            with open(self.shm_in, 'w') as f:
                f.write(f"1 0 {action[0]} {action[1]}")
            with open(self.shm_out, 'r') as f:
                state_json = f.read()
            state = json.loads(state_json)["Items"][0]
        else:
            a_in = ""
            for i in action:
                a_in += str(float(i)) + " "
            a_in = a_in[:-1]
            response = requests.post(self.url + "/step", json=a_in, headers=self.headers)
            state = response.json()["Items"]
        collided, done = self.int2bool(state[10]), self.int2bool(state[11])
        return state, collided, done, (collided or done)


    def reset(self):
        if self.use_shm:
            with open(self.shm_in, 'w') as f:
                f.write("0 0 0 0")
            state = np.fromfile(self.shm_out, dtype=np.float32)
        elif self.use_hybrid_shm:
            response = requests.post(self.url + "/reset")
            state = response.json()["Items"]
        else:
            response = requests.post(self.url + "/reset")
            state = response.json()["Items"]
        collided, done = self.int2bool(state[10]), self.int2bool(state[11])

        return state, collided, done, (collided or done)

    def crash(self):
        response = requests.get(self.url+"/crash")
        return response.json()

if __name__ == "__main__":
    engine = FRCEngine()
    print(engine.reset())
    simple_action = [1,1]
    while True:
        print(engine.step(simple_action))