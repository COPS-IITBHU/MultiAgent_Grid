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
    #env, args = make_env(args)
    args.n_players = 8
    args.n_agents = 8  
    """args.obs_shape = [env.observation_space[i].shape[0] for i in range(args.n_agents)]  
    action_shape = []
    for content in env.action_space:
        action_shape.append(content.n)
    args.action_shape = action_shape[:args.n_agents]"""
    args.obs_shape = [2]*8
    args.action_shape = [2]*8
    args.high_action = 1
    args.low_action = -1

    runner = Runner(args, env)
    if args.evaluate:
        returns = runner.evaluate()
        print('Average returns is', returns)
    else:
        runner.run()
