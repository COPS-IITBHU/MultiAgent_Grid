import random
import gym 
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import math
from bezier import calc_bezier_path,calc_4points_bezier_path



import pygame
from pygame.locals import(QUIT,KEYDOWN,K_ESCAPE,K_RIGHT,K_LEFT,K_UP,K_DOWN)

import Box2D
from Box2D.Box2D import b2PolygonShape
from Box2D import (b2World, b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape, b2ContactListener,
                   b2Transform, b2Mul, b2Vec2,
                   b2_pi)
#from Box2D.b2 import (world,polygonShape,circleShape,staticBody,dynamicBody,vec2)
from Box2D.b2 import (
    world,
    edgeShape,
    circleShape,
    fixtureDef,
    polygonShape,
    revoluteJointDef,
    contactListener,
)

class ContactDetector(contactListener):

    def __init__(self, env):
        contactListener.__init__(self)
        self.env=env

    def BeginContact(self,contact):
        #print("collision!")
        self.env.reward =  self.env.reward - 10
        




class grid(gym.Env):
   
    
    metadata={'render.modes':['human']}

    def __init__(self, num_agents=9):
        self.num_agents = num_agents
        self.unit = 0.0254*1.2
        self.radius = 0.6 * self.unit
        self.density = 4*1.0
        self.force_scale = 0.001
        self.restitution = 0
        self.max_velocity = 0.01
        self.max_force = 0.01
        self.fps = 60
        self.sfr = 1
        self.screen_height = 600
        self.screen_width = 800
        self.viewer = None
        self.ppm = 240
        self.screen=None
        self.epsilon = self.radius/4
        self.n_points = 1200
        #self.target_pos = np.random.uniform(-40*self.unit,40*self.unit,(8,2))
        self.scalar_force = 1
        
        self.time_step = 1./self.fps*self.sfr

        high_a = np.array([[(45.0*self.unit,40.0*self.unit),(45.0*self.unit,40.0*self.unit)] for i in range(self.num_agents)])
        high_b = np.array([(45.0*self.unit,40.0*self.unit) for i in range(self.num_agents)])

        self.action_space = spaces.Box(-high_a,high_a,
        dtype=np.float64,
        shape=(self.num_agents,2,2)    
            )

        self.maps = [self.map1, self.map2, self.map3, self.map4]
        self.mapind = 0
        
        self.observation_space = spaces.Box(
            -high_b,
            high_b,
            dtype=np.float64,
            shape=(self.num_agents,2)    
        )
       
        def draw_line(line, body, fixture):

            shape = fixture.shape   
            vertices = [(body.transform * v+b2Vec2(1.7,1.25)) * self.ppm for v in shape.vertices]          
            vertices = [(v[0], self.screen_height - v[1]) for v in vertices]
            pygame.draw.line(self.screen, (	57, 255, 20,255),vertices[0],vertices[1],width=1)

        edgeShape.draw = draw_line

        def draw_agents(circle, body, fixture):
            position = (body.transform * circle.pos +b2Vec2(1.7,1.25) ) * self.ppm
            position = (position[0], self.screen_height - position[1])
            pygame.draw.circle(self.screen,(255,255,255,255) , [int(
            x) for x in position], int(circle.radius * self.ppm))

        circleShape.draw =draw_agents

        


    def status(self):
        cnt = 0
        list = np.zeros((self.num_agents))
        for i in range(self.num_agents):
            if math.sqrt((self.agents[i].position[0]-self.target_pos[i][0])**2 + (self.agents[i].position[1]-self.target_pos[i][1])**2)<=  self.epsilon :
                #cnt = cnt+1
                list[i] = 1
        return list

        """if cnt == 8 :
            return True
        else:
            return False"""


    def draw_marker(self):

            for i in self.target_pos:
                vertices = [(i+b2Vec2(1.7,1.25)+b2Vec2(-self.epsilon,-self.epsilon))*self.ppm,(i+b2Vec2(1.7,1.25)+b2Vec2(self.epsilon,self.epsilon))*self.ppm]
                
                vertices = [(v[0], self.screen_height - v[1]) for v in vertices]
                pygame.draw.line(self.screen, (	57, 255, 20,255),vertices[0],vertices[1],width=1)

            for i in self.target_pos:
                vertices = [(i+b2Vec2(1.7,1.25)+b2Vec2(-self.epsilon,self.epsilon))*self.ppm,(i+b2Vec2(1.7,1.25)+b2Vec2(self.epsilon,-self.epsilon))*self.ppm]
                
                vertices = [(v[0], self.screen_height - v[1]) for v in vertices]
                pygame.draw.line(self.screen, (	57, 255, 20,255),vertices[0],vertices[1],width=1)

    def draw_path(self):

        for ind_path in self.path_a:
            for i in ind_path:
                position = (i +b2Vec2(1.7,1.25) ) * self.ppm
                position = (position[0], self.screen_height - position[1])
                pygame.draw.circle(self.screen,(169,169,169.255) , [(x) for x in position], (self.epsilon /2* self.ppm))

        




    def step(self, action):

        #for i in range(8):
            #self.path[i] = calc_bezier_path(self.initial_pos[i], action[i][0], action[i][1], self.target_pos[i])

        
        running = True
        steps=0
        cnt = 0
        finished = False
        #print(self.path[self.iter])
        #print(self.initial_pos)
        while not finished:
            finished = True
            cnt+=1
            # if cnt>=50:break
            for agent,destination in zip(self.agents,self.path[self.iter]):
                delta = b2Vec2(destination) - agent.position
                if delta.length <= self.epsilon/8:
                    agent.linearVelocity = (0,0)
                    self.status()
                    continue
                finished = False
                direction = delta/delta.length
                vel_mag = agent.linearVelocity.length * direction.dot(agent.linearVelocity)
                force_mag = self.max_force*(1 - vel_mag/self.max_velocity)
                force = force_mag*direction
                agent.ApplyForce(force = force,point=agent.position,wake=True)
                self.iter = self.iter + 1
                #print(self.reward)
                if self.iter == self.n_points-1 :
                    finished = True
                    break
                
                

                self.world.Step(self.time_step,10,10)
            if steps%self.sfr == 0:
                self.render() 
            steps = (steps+1)
            if steps*self.time_step > 30:
                raise RuntimeError("environment timestep exceeded 30 seconds")
        self.world.Step(self.time_step,10,10)
        self.world.ClearForces()
        observation = np.array([np.array(agent.position) for agent in self.agents])
        status = self.status()
        if all(self.status()):
            done = True
        for i in status:
            if i == 1:
                self.reward = self.reward + 1

        
        
        
        """for i in range(0,self.num_agents):
                for t in self.path[i]:
                    if ((t[0]-self.path[i,self.n_points-1,0])**2 + (t[1]-self.path[i,self.n_points-1,1])**2 < self.epsilon**2):

                    # keep update new target here
                        #self.update_path(i,new_target)
                        cnt = cnt + 1 


                    elif((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2 > self.epsilon**2):
                            dis=math.sqrt((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2)

                            f = self.agents[i].GetWorldVector(localVector=((t[0]-self.agents[i].position[0])/dis,(t[1]-self.agents[i].position[1])/dis))
                            p = self.agents[i].GetWorldPoint(localPoint=(0.0, 0.00))
                            self.agents[i].ApplyForce(f, p, True)
                            break
            if cnt == 8 :
                running = False"""


        """for t in self.path:
                for i in range(0,self.num_agents):
                    if ((t[i][0]-self.path[self.n_points-1,i,0])**2 + (t[i][1]-self.path[self.n_points-1,i,1])**2 < self.epsilon**2):

                    # keep update new target here
                        #self.update_path(i,new_target)
                        cnt = cnt + 1 


                    elif((t[i][0]-self.agents[i].position[0])**2 + (t[i][1]-self.agents[i].position[1])**2 > self.epsilon**2):
                            dis=math.sqrt((t[i][0]-self.agents[i].position[0])**2 + (t[i][1]-self.agents[i].position[1])**2)*1000

                            f = self.agents[i].GetWorldVector(localVector=((t[i][0]-self.agents[i].position[0])/dis,(t[i][1]-self.agents[i].position[1])/dis))
                            p = self.agents[i].GetWorldPoint(localPoint=(0.0, 0.00))
                            self.agents[i].ApplyForce(f, p, True)"""
            
        """for i in range(0,self.num_agents):    working
                temp=0
                for t in self.path[i]:
                    if((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2 <= self.epsilon**2 and self.path_done[i,temp]==0):
                        self.path_done[i,temp]=1
                        temp+=1
                        #break
                        

                    elif((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2 > self.epsilon**2 and self.path_done[i,temp]==0):
                        dis=math.sqrt((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2)

                        f = self.agents[i].GetWorldVector(localVector=((t[0]-self.agents[i].position[0])/dis,(t[1]-self.agents[i].position[1])/dis))*self.scalar_force
                        p = self.agents[i].GetWorldPoint(localPoint=(0.0, 0.00))
                        self.agents[i].ApplyForce(f, p, True)
                        break"""
            
            

                            
                       
        """if res == 8 :
                running = False

            self.world.Step(self.time_step,10,10)
            if steps%self.sfr == 0:
                self.render() 
            steps = (steps+1)
            if steps*self.time_step > 30:
                raise RuntimeError("environment timestep exceeded 30 seconds")"""



        
            


                
        """obs = []
            for i in range(0,self.num_agents):
            f = self.agents[i].GetWorldVector(localVector=action[i])
            p = self.agents[i].GetWorldPoint(localPoint=(0.0 , 0.0))
            self.agents[i].ApplyForce(f, p, True)
            obs.append([self.agents[i].position[0],self.agents[i].position[1],self.agents[i].linearVelocity[0],self.agents[i].linearVelocity[1]])"""
            
                
                
            
            
            #print(self.status())
            #print('_________________________________')
        return 0,0,0,0

            
            

            

           
            
            
            

    def reset(self):

        
        self.world = world(gravity=(0,0),contactListener=ContactDetector(self))

        self.maps[self.mapind % len(self.maps)]()
        self.mapind += 1
        # Initial position of bots
        self.initial_pos = []
        xlen = int(math.sqrt(self.num_agents))
        for dx in range(xlen):
            for dy in range(self.num_agents//xlen):
                self.initial_pos.append(( (-23 + 2.4*dx)*self.unit, (-23 + 2.4*dy)*self.unit ))
        # self.initial_pos=[(7.5*6*self.unit, 1.5*6*self.unit),(-7.5*6*self.unit, 1.5*6*self.unit),(7.5*6*self.unit, -1.5*6*self.unit),(-7.5*6*self.unit, -1.5*6*self.unit),
        #                     (6.5*6*self.unit, 2.5*6*self.unit),(-6.5*6*self.unit, 2.5*6*self.unit),(6.5*6*self.unit, -2.5*6*self.unit),(-6.5*6*self.unit, -2.5*6*self.unit)]

        #self.initial_pos=[(7.5*6*self.unit, 1.5*6*self.unit),(-7.5*6*self.unit, 1.5*6*self.unit),(-8*self.unit,0),(-7.5*6*self.unit, -1.5*6*self.unit),
                            #(6.5*6*self.unit, 2.5*6*self.unit),(-6.5*6*self.unit, 2.5*6*self.unit),(6.5*6*self.unit, -2.5*6*self.unit),(-6.5*6*self.unit, -2.5*6*self.unit)]
        #self.initial_pos = self.target_pos

        #self.target_pos = (0,0)

        # deploy bots
        self.bodies = []
        for i in range(self.num_agents):
            self.bodies.append(
                self.world.CreateDynamicBody(position=self.initial_pos[i],
                                             fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius),
                                             density=self.density,
                                             restitution=self.restitution
                                             )
                                            )
            )

        # self.body0 = self.world.CreateDynamicBody(position=self.initial_pos[0], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body1 = self.world.CreateDynamicBody(position=self.initial_pos[1], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body2 = self.world.CreateDynamicBody(position=self.initial_pos[2], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body3 = self.world.CreateDynamicBody(position=self.initial_pos[3], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body4 = self.world.CreateDynamicBody(position=self.initial_pos[4], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body5 = self.world.CreateDynamicBody(position=self.initial_pos[5], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body6 = self.world.CreateDynamicBody(position=self.initial_pos[6], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        # self.body7 = self.world.CreateDynamicBody(position=self.initial_pos[7], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))

        #self.body7.linearVelocity = (1,0)

        self.agents=[body for body in self.bodies]
        self.reward=0
        self.collide=[0]*self.num_agents
        self.world.gravity=(0,0)
        self.iter = 1
        self.reward = 0
        
        row1 = [((-36 + 12 * i) * self.unit, 36 * self.unit) for i in range(7)]
        row2 = [((-36 + 24 * i) * self.unit, 24 * self.unit) for i in range(4)]
        row3 = [((-36 + 12 * i) * self.unit, 12 * self.unit) for i in range(7)]
        row4 = [((-36 + 24 * i) * self.unit, 0 * self.unit) for i in range(4)]
        row5 = [((-36 + 12 * i) * self.unit, -12 * self.unit) for i in range(7)]
        row6 = [((-36 + 24 * i) * self.unit, -24 * self.unit) for i in range(4)]
        row7 = [((-36 + 12 * i) * self.unit, -36 * self.unit) for i in range(7)]

        self.grid_centres = row1 + row2 + row3 + row4 + row5 + row6 + row7
        

        target = np.zeros((self.num_agents,2))

        # grid_num = np.random.choice(40,self.num_agents,replace=False)
        # for i in range(self.num_agents):
        #     target[i] = self.grid_centres[grid_num[i]]
        #     target[i][0] = target[i][0] + np.random.uniform(-3*self.unit,3*self.unit,1)
        #     target[i][1] = target[i][1] + np.random.uniform(-3*self.unit,3*self.unit,1)
        for i in range(self.num_agents):
            target[i][0] = np.random.uniform(-25*self.unit, 25*self.unit,1)
            target[i][1] = np.random.uniform(-25*self.unit, 25*self.unit,1)
        self.target_pos = target

        self.path = np.zeros((self.num_agents,self.n_points,2))
        for i in range(self.num_agents):
            temp = calc_4points_bezier_path(self.initial_pos[i][0],self.initial_pos[i][1], 0.15, self.target_pos[i][0],self.target_pos[i][1], 0.15, 3, self.n_points)
            self.path[i] = temp[0]
        #np.concatenate((calc_4points_bezier_path(self.initial_pos[i][0],self.initial_pos[i][1], 0.52, self.target_pos[i][0],self.target_pos[i][1], 0.52, 0.1) for i in range(8)), out = self.path  
        self.path_a =self.path    
        self.path = np.transpose(self.path, (1, 0, 2)) 
        

    
    def map1(self):
        vert = [
            ( -25*self.unit, -25*self.unit ),
            ( -25*self.unit,  25*self.unit ),
            (  25*self.unit,  25*self.unit ),
            (  25*self.unit, -25*self.unit ),
        ]

        for i in range(-1, len(vert)-1):
            self.world.CreateStaticBody(
                shapes=b2EdgeShape(vertices=[vert[i], vert[i+1]])
            )
        

    def map2(self):
        vert = [
            ( -25*self.unit, -25*self.unit ),
            ( -25*self.unit,  25*self.unit ),
            (  25*self.unit,  25*self.unit ),
            (  25*self.unit, -25*self.unit ),
        ]

        for i in range(-1, len(vert)-1):
            self.world.CreateStaticBody(
                shapes=b2EdgeShape(vertices=[vert[i-1], vert[i]])
            )
        
        vert_out = [
            (-4.5*self.unit, -4.5*self.unit),
            (-4.5*self.unit,  4.5*self.unit),
            ( 4.5*self.unit,  4.5*self.unit),
            ( 4.5*self.unit, -4.5*self.unit),
        ]
        
        vert_cent = [
            (-12.5*self.unit, -12.5*self.unit),
            (-12.5*self.unit,  12.5*self.unit),
            ( 12.5*self.unit,  12.5*self.unit),
            ( 12.5*self.unit, -12.5*self.unit),
        ]
        for ctrs in vert_cent:
            for i in range(-1, len(vert_out)-1):
                self.world.CreateStaticBody(
                    shapes=b2EdgeShape(vertices=[(vert_out[i-1][0]+ctrs[0], vert_out[i-1][1]+ctrs[1]), (vert_out[i][0]+ctrs[0], vert_out[i][1]+ctrs[1])])
                )
        

    def map3(self):
        vert = [
            ( -25*self.unit, -25*self.unit ),
            ( -25*self.unit,  25*self.unit ),
            (  25*self.unit,  25*self.unit ),
            (  25*self.unit, -25*self.unit ),
        ]

        for i in range(-1, len(vert)-1):
            self.world.CreateStaticBody(
                shapes=b2EdgeShape(vertices=[vert[i-1], vert[i]])
            )

        wall1 = [
            (  -25*self.unit, 0*self.unit),
            (   -5*self.unit, 0*self.unit),
        ]

        wall2 = [
            (  5*self.unit, -25*self.unit),
            (  5*self.unit, -15*self.unit),
        ]

        self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=wall1)
        )

        self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=wall2)
        )

    def map4(self):
        vert = [
            ( -25*self.unit, -25*self.unit ),
            ( -25*self.unit,  25*self.unit ),
            (  25*self.unit,  25*self.unit ),
            (  25*self.unit,  10*self.unit ),
            ( -10*self.unit,  10*self.unit ),
            ( -10*self.unit, -25*self.unit ),
            # (  25*self.unit, -25*self.unit ),
        ]

        for i in range(-1, len(vert)-1):
            self.world.CreateStaticBody(
                shapes=b2EdgeShape(vertices=[vert[i-1], vert[i]])
            )
        
        wall1 = [
            (  -10*self.unit, 25*self.unit),
            (  -10*self.unit, 23*self.unit),
        ]

        wall2 = [
            (  -10*self.unit, 12*self.unit),
            (  -10*self.unit, 10*self.unit),
        ]
    
        self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=wall1)
        )

        self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=wall2)
        )
        

    def render(self, mode='human',close=False):
        if self.screen is None:
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height),0,32)
            pygame.display.set_caption('grid-2D')
            self.clock = pygame.time.Clock()

        self.screen.fill((0,0,0,0))

        for body in self.world.bodies:
            for fixture in body.fixtures:
                fixture.shape.draw(body,fixture)

        self.draw_marker()
        #self.draw_path()

        

        

        
        
        
        pygame.display.flip()
        self.clock.tick(self.fps)

        


    def close(self):
        pygame.quit()
