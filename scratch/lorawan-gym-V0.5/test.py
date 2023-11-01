#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 11:33:32 2020
Modified on Sat Oct  9 17:00:00 2021
Author: Rogério S. Silva
Email: rogeriosousaesilva@gmail.com
"""

import argparse
import random

import numpy as np
from ns3gym import ns3env
from colorama import Fore, Back, Style

__author__ = "Rogério S. Silva"
__copyright__ = "Copyright (c) 2023, NumbERS - Federal Institute of Goiás, Inhumas - IFG"
__version__ = "0.1.0"
__email__ = "rogerio.sousa@ifg.edu.br"


def get_q_state(obs):
    x = float(obs[0])
    y = float(obs[1])
    z = float(obs[2])
    q_index = int(x / 1000) + int(y / 1000) * 11  # + int(z / 10) * 121 # For 3D movements
    return q_index

def print_state(obs, reward, done, info):
    if reward > 0:
        print(
            Fore.LIGHTGREEN_EX + f"Step: {step} Action: none "
                                 f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
    else:
        if reward == -1:
            print(
                Fore.LIGHTWHITE_EX + f"Step: {step} Action: none "
                                     f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
        else:
            print(
                Fore.RED + f"Step: {step} Action: none "
                           f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")


parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=10,
                    help='Number of iterations, Default: 10')
parser.add_argument('--steps',
                    type=int,
                    default=200,
                    help='Number of steps, Default: 500')
args = parser.parse_args()
startSim = bool(args.start)
iterations = int(args.iterations)
steps = int(args.steps)

port = 5555
nDevices = 10
nGateways = 1
startX = 5000
startY = 4000
startZ = 30
state = get_q_state([startX, startY, startZ])  # Initial state
simTime = 600  # seconds
stepTime = 600  # seconds
seed = 1
simArgs = {"--nDevices": nDevices,
           "--nGateways": nGateways,
           "--startX": startX,
           "--startY": startY,
           "--startZ": startZ}
startSim = 1
debug = 1

env = ns3env.Ns3Env(port=port, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

## Q-Learning
# Inicializando a "q-table"
state_size = 121  # 11x11 (0..10000, 0..10000) Step: 1000
action_size = 4  # 0: none, 1: up, 2: down, 3: left, 4: right
qtable = np.zeros((state_size, action_size))

# Hiperparâmetros
learning_rate = 0.9
discount_rate = 0.8
epsilon = 1.0
decay_rate = 0.005

# Variáveis de treinamento
num_episodes = 100
max_steps = 99  # por episodio

try:
    for episode in range(num_episodes):
        print(f"Episode: {episode}")
        obs, reward, done, info = env.get_state()
        for step in range(max_steps):

            print_state(obs, reward, done, info);

            if random.uniform(0, 1) < epsilon:
                # exploration
                action = env.action_space.sample()
            else:
                # exploitation
                action = np.argmax(qtable[state, :])

            obs, reward, done, info = env.step(action)
            new_state = get_q_state(obs)


            # Q-learning
            old_value = qtable[state, action]  # Value of the chosen action in the current state
            next_max = np.max(qtable[new_state])  # Maximum value of the next state

            # Q-learning equation
            new_value = (1 - learning_rate) * old_value + learning_rate * (reward + discount_rate * next_max)
            qtable[state, action] = new_value

            # Update to our new state
            state = new_state

        env.reset()

        # Decrease epsilon
        epsilon = np.exp(-decay_rate * episode)

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    print(qtable)
    env.close()
    print("Done")
