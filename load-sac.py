from frcenv import FRCEnv
from stable_baselines3 import SAC

env = FRCEnv()
model = SAC.load("G:/Projects/AutoNav/FRCEnv/out/sac.zip", env)

while True:
    observation, _ = env.reset()
    for i in range(100):
        env.render()
        action, _ = model.predict(observation)
        observation, reward, done, trunc, info = env.step(action)
        print(observation)
        if done or trunc:
            print("Episode finished after {} timesteps".format(i+1))
            break