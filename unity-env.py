import requests
import json


class FRCEngine:

    def __init__(self):
        self.url = "http://localhost:5000"
        self.headers = {'Content-Type': 'application/json'}


    def step(self, action):
        response = requests.post(self.url+"/step", json=action, headers=self.headers)
        return response.json()

    def reset(self):
        response = requests.post(self.url+"/reset")
        return response.json()

    def crash(self):
        response = requests.get(self.url+"/crash")
        return response.json()

if __name__ == "__main__":
    engine = FRCEngine()
    print(engine.reset())
    simple_action = "1 1"
    simple_action = json.dumps(simple_action)
    while True:
        print(engine.step(simple_action))