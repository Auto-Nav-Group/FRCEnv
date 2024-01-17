import requests
import json


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
        self.target = [0,0,0,0,0,0,0,1,0,0, 0, 0] + self.lidar_zeros
        self.int2bool = lambda x: True if x==1 else False

    def get_size_info(self):
        response = requests.get(self.url+"/getsizeinfo")
        return response.json()["Items"]


    def step(self, action):
        a_in = ""
        for i in action:
            a_in += str(float(i))+" "
        a_in = a_in[:-1]
        response = requests.post(self.url+"/step", json=a_in, headers=self.headers)
        state = response.json()["Items"]
        collided, done = self.int2bool(state[10]), self.int2bool(state[11])
        return state, collided, done, (collided or done)

    def reset(self):
        response = requests.post(self.url+"/reset")
        state = response.json()["Items"]
        collided, done = self.int2bool(state[10]), self.int2bool(state[11])
        return state, collided, done, (collided or done)

    def crash(self):
        response = requests.get(self.url+"/crash")
        return response.json()

if __name__ == "__main__":
    engine = FRCEngine()
    print(engine.reset())
    simple_action = "1 1"
    while True:
        print(engine.step(simple_action))