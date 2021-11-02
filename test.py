import gym
from gym.utils import play
import numpy as np

from bezier import calc_4points_bezier_path
import gym_grid
env = gym.make('grid-v0')

#print(np.random.uniform(-40*0.0254,40*0.0254,(8,2)))



env.reset()
env.render()

# -8*0.0254,0   -8*0.0254,-8*0.0254
#res = calc_4points_bezier_path(-8*0.0254,0,0.52,-8*0.0254,-20*0.0254,0.52,0.5)

s = [(0.00,0.00)]*8
points = np.random.uniform(-40*0.0254,40*0.0254,(8))

env.step()
env.reset()
env.step()
env.reset()
env.step()
env.reset()
env.step()
env.reset()
env.step()
env.reset()
env.step()
env.reset()
env.step()
