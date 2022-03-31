from runner import Runner
from common.arguments import get_args
from common.utils import make_env
import numpy as np
import random
import torch
import gym
import gym_grid

env = gym.make('grid-v0')

if __name__ == '__main__':
    # get the params
    args = get_args()
    args.n_players = 8
    args.n_agents = 8  
    args.obs_shape = [200]*8
    args.action_shape = [4]*8
    args.high_action = 2
    args.low_action = -2

    runner = Runner(args, env)
    
    returns = runner.evaluate()
    print('Average returns is', returns)
    