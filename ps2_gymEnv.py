import gym
from gym import error, spaces, utils
import numpy as np
from Box2D.Box2D import b2PolygonShape
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2World, b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Transform, b2Mul, b2Vec2,
                   b2_pi)
from Box2D import *

# OBSERVATION STRUCT : ((AGENT(0-7),(POSITION PAIR),(VELOCITY PAIR))

# ACTION STRUCT : ((AGENT(0-7),(FORCE PAIR))


# bot constants
num_agents = 8
radius = 2*0.0254
density = 1.0
restitution = 0.0

high=100
low=-100

observation_shape=(8, 2, 2)
action_shape=(8, 2)
target_pos_shape=(8,2)

class ps2(gym.Env,Framework):

    def __init__(self):

        self.world=Box2D.b2World()
        self.walls()
        self.observation_space = spaces.Box(high, low, dtype=np.float64, shape=observation_shape)
        self.action_space = spaces.Box(high, low, dtype=np.float64, shape=action_shape)
        self.target_pos = spaces.Box(high, low, dtype=np.float64, shape=target_pos_shape)
        self.reset()
    
    def reset(self):
        
        # Initial position of bots
        self.initial_pos=[(7.5*6*0.0254, 1.5*6*0.0254),(-7.5*6*0.0254, 1.5*6*0.0254),(7.5*6*0.0254, -1.5*6*0.0254),(-7.5*6*0.0254, -1.5*6*0.0254),
                            (6.5*6*0.0254, 2.5*6*0.0254),(-6.5*6*0.0254, 2.5*6*0.0254),(6.5*6*0.0254, -2.5*6*0.0254),(-6.5*6*0.0254, -2.5*6*0.0254)]

        # deploy bots
        self.body1 = self.world.CreateDynamicBody(position=self.initial_pos[0], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body2 = self.world.CreateDynamicBody(position=self.initial_pos[1], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body0 = self.world.CreateDynamicBody(position=self.initial_pos[2], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body3 = self.world.CreateDynamicBody(position=self.initial_pos[3], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body4 = self.world.CreateDynamicBody(position=self.initial_pos[4], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body5 = self.world.CreateDynamicBody(position=self.initial_pos[5], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body6 = self.world.CreateDynamicBody(position=self.initial_pos[6], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))
        self.body7 = self.world.CreateDynamicBody(position=self.initial_pos[7], fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))

        self.agents=[self.body0,self.body1,self.body2,self.body3,self.body4,self.body5,self.body6,self.body7]
        
        self.world.gravity=(0,0)

    def walls(self):
        # upper lower wall
        ground = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 7*6*0.0254 ), (7*6*0.0254, 7*6*0.0254)])
        )
        ground1 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -7*6*0.0254 ), (7*6*0.0254, -7*6*0.0254)])
        )
        # middle
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 1*6*0.0254 ), (1*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -1*6*0.0254 ), (1*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, 1*6*0.0254 ), (1*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 1*6*0.0254 ), (-1*6*0.0254, -1*6*0.0254)])
        )
        # middle top
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 5*6*0.0254 ), (1*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 3*6*0.0254 ), (1*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, 5*6*0.0254 ), (1*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 5*6*0.0254 ), (-1*6*0.0254, 3*6*0.0254)])
        )
        # middle low
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -5*6*0.0254 ), (1*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -3*6*0.0254 ), (1*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, -5*6*0.0254 ), (1*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -5*6*0.0254 ), (-1*6*0.0254, -3*6*0.0254)])
        )
        # right left
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 1*6*0.0254 ), (5*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -1*6*0.0254 ), (5*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 1*6*0.0254 ), (3*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, 1*6*0.0254 ), (5*6*0.0254, -1*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 1*6*0.0254 ), (-5*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -1*6*0.0254 ), (-5*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 1*6*0.0254 ), (-3*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, 1*6*0.0254 ), (-5*6*0.0254, -1*6*0.0254)])
        )
        # corner
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 5*6*0.0254 ), (5*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 3*6*0.0254 ), (5*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 5*6*0.0254 ), (3*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, 5*6*0.0254 ), (5*6*0.0254, 3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 5*6*0.0254 ), (-5*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 3*6*0.0254 ), (-5*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 5*6*0.0254 ), (-3*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, 5*6*0.0254 ), (-5*6*0.0254, 3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -5*6*0.0254 ), (5*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -3*6*0.0254 ), (5*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -5*6*0.0254 ), (3*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, -5*6*0.0254 ), (5*6*0.0254, -3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -5*6*0.0254 ), (-5*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -3*6*0.0254 ), (-5*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -5*6*0.0254 ), (-3*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, -5*6*0.0254 ), (-5*6*0.0254, -3*6*0.0254)])
        )
        # side wall
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 7*6*0.0254 ), (7*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 7*6*0.0254 ), (-7*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -7*6*0.0254 ), (7*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -7*6*0.0254 ), (-7*6*0.0254, -2*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 1*6*0.0254 ), (7*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 1*6*0.0254 ), (-7*6*0.0254, -1*6*0.0254)])
        )
        # chute
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(8*6*0.0254, 1*6*0.0254 ), (8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-8*6*0.0254, 1*6*0.0254 ), (-8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(8*6*0.0254, -1*6*0.0254 ), (8*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-8*6*0.0254, -1*6*0.0254 ), (-8*6*0.0254, -2*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 1*6*0.0254 ), (8*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 1*6*0.0254 ), (-8*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -1*6*0.0254 ), (8*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -1*6*0.0254 ), (-8*6*0.0254, -1*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 2*6*0.0254 ), (8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 2*6*0.0254 ), (-8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -2*6*0.0254 ), (8*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -2*6*0.0254 ), (-8*6*0.0254, -2*6*0.0254)])
        )
        

    def get_observation(self):
        self.observation=[]

        for i in range(0,num_agents):
            self.observation.append((self.agents[i].position,self.agents[i].linearVelocity))

        # print(np.shape(self.observation))

        return self.observation

    def step(self,action):

        for i in range(0,num_agents):
            f = self.agents[i].GetWorldVector(localVector=action[i])
            p = self.agents[i].GetWorldPoint(localPoint=(0.0, 0.00))
            self.agents[i].ApplyForce(f, p, True)


# if __name__ == "__main__":
#     env=ps2()
#     env.get_observation()
