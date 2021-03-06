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
from Box2D import (b2World, b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape, b2ContactListener, b2RayCastCallback,
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

class RayCastClosestCallback(b2RayCastCallback):
    """This callback finds the closest hit"""

    def __repr__(self):
        return 'Closest hit'

    def __init__(self, **kwargs):
        b2RayCastCallback.__init__(self, **kwargs)
        self.fixture = None
        self.hit = False

    def ReportFixture(self, fixture, point, normal, fraction):
        '''
        Called for each fixture found in the query. You control how the ray
        proceeds by returning a float that indicates the fractional length of
        the ray. By returning 0, you set the ray length to zero. By returning
        the current fraction, you proceed to find the closest point. By
        returning 1, you continue with the original ray clipping. By returning
        -1, you will filter out the current fixture (the ray will not hit it).
        '''
        self.hit = True
        self.fixture = fixture
        self.point = b2Vec2(point)
        self.normal = b2Vec2(normal)
        # NOTE: You will get this error:
        #   "TypeError: Swig director type mismatch in output value of
        #    type 'float32'"
        # without returning a value
        return fraction


class ContactDetector(contactListener):

    def __init__(self, env):
        contactListener.__init__(self)
        self.env=env

    def BeginContact(self,contact):
        #print("collision!")
        if contact.fixtureA.body.userData != None :
            self.env.reward[contact.fixtureA.body.userData] -= 10

        if contact.fixtureB.body.userData != None :
            self.env.reward[contact.fixtureB.body.userData] -= 10

        
        #print(contact.fixtureA.body.userData)
        #print(contact.fixtureB.body.userData)
        #self.env.reward =  self.env.reward - 10
        




class grid(gym.Env):
   
    
    metadata={'render.modes':['human']}

    def __init__(self):
        self.num_agents = 8
        self.radius = 2 * 0.0254
        self.unit = 0.0254
        self.density = 1.0
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
        self.n_points = 800
        self.call_back = RayCastClosestCallback()
        #self.target_pos = np.random.uniform(-40*self.unit,40*self.unit,(8,2))
        self.scalar_force = 1
        
        self.time_step = 1./self.fps*self.sfr
        a = [[(45.0*self.unit,40.0*self.unit)*3] for i in range(8)]

        b = [[(45.0*self.unit,40.0*self.unit)*20] for i in range(8)]
        high_a = np.array(a)
        high_a = np.squeeze(high_a, axis=1)
        high_b = np.array(b)
        high_b = np.squeeze(high_b, axis=1)
        
        self.action_space = spaces.Box(-high_a,high_a,
        dtype=np.float64,
        shape=(self.num_agents,6)    
            )
        
        self.observation_space = spaces.Box(
            -high_b,
            high_b,
            dtype=np.float64,
            shape=(self.num_agents,40)    
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
        list = np.zeros((8))
        for i in range(8):
            if math.sqrt((self.agents[i].position[0]-self.target_pos[i][0])**2 + (self.agents[i].position[1]-self.target_pos[i][1])**2)<=  4*self.epsilon :
                #cnt = cnt+1
                list[i] = 1
        return list

        """if cnt == 8 :
            return True
        else:
            return False"""

    def path_status(self):                          #check colliding points on the path
        cnt = 1
        for i in range(len(self.path_a[6])):
            
            r1 = b2Vec2(self.path_a[0][i])-b2Vec2(self.path_a[0][i-1])
            #self.world2.RayCast(self.call_back,b2Vec2(self.path_a[0][i-1]),b2Vec2(self.path_a[0][i-1])+ (ray/ray.length)*1.1*self.radius)
            self.world2.RayCast(self.call_back,b2Vec2(self.path_a[0][i-1]),b2Vec2(self.path_a[0][i]))
            
            """if self.call_back.hit :
                #print("HIT")
                print(cnt)
                print(self.call_back.point)
                cnt = cnt + 1"""
            """if ((self.call_back.point[0]-self.path_a[0][i-1][0])**2 + (self.call_back.point[1]-self.path_a[0][i-1][1])**2) < 1.21 * self.radius * self.radius  :  #to ensure a clearance of 1.1r from any walls
                print(cnt)
                cnt = cnt + 1"""
            """if self.call_back.hit :
                ch = str(self.call_back.fixture.shape)
                #oprint(ch[0:11])
                if ch[0:11] == "b2EdgeShape":
                    print(cnt)
                    cnt = cnt + 1"""
            if self.call_back.hit :
                r2 = self.call_back.point-b2Vec2(self.path_a[0][i-1])
                if r2.length <=r1.length :
                    print(cnt)
                    cnt = cnt+1

        print("Done")

            

            
                


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

        for ind_path in self.path_draw:
            for i in ind_path:
                position = (i +b2Vec2(1.7,1.25) ) * self.ppm
                position = (position[0], self.screen_height - position[1])
                pygame.draw.circle(self.screen,(169,169,169.255) , [(x) for x in position], (self.epsilon /2* self.ppm))
        """for i in self.path_a[6]:
                position = (i +b2Vec2(1.7,1.25) ) * self.ppm
                position = (position[0], self.screen_height - position[1])
                pygame.draw.circle(self.screen,(169,169,169.255) , [(x) for x in position], (self.epsilon /2* self.ppm))"""
        

        




    def step(self,action):

            

        #print(self.n_points)
        #ini_pos = np.concatenate((self.initial_pos,self.initial_pos),axis = 1)
        self.path = np.zeros((8,self.n_points,2))
        if self.n_points == 800:
            start = np.zeros((8,2))
            
        else:
            start = [(self.agents[i].position[0]-self.initial_pos[i][0],self.agents[i].position[1]-self.initial_pos[i][1])for i in range(8)]
            self.reward = np.zeros((8))

            

        len_rew = np.zeros((8))
        for i in range(8):    
            #print([start[i], [action[i][0],action[i][1]],[action[i][2],action[i][3]], self.target_pos[i]-self.initial_pos[i]])       
            self.path[i] = calc_bezier_path(np.array([start[i], [action[i][0],action[i][1]],[action[i][2],action[i][3]], self.target_pos[i]-self.initial_pos[i]]),n_points=self.n_points)                
            len_rew[i] = math.sqrt((self.target_pos[i][0]-self.agents[i].position[0])**2+(self.target_pos[i][1]-self.agents[i].position[1])**2)
            
        """self.path_draw =self.path                               #before transpose : size-> (8,n_points,2)
        self.path = np.transpose(self.path, (1, 0, 2))       #after transpose  : size-> (n_points,8,2)
        self.path_iter = self.path
        self.path = self.path - np.array(self.initial_pos)
        self.path_obs = np.transpose(self.path, (1, 0, 2))"""
        self.path_iter = np.transpose(self.path, (1, 0, 2))
        self.path_iter = self.path_iter + np.array(self.initial_pos)
        self.path_draw = np.transpose(self.path_iter, (1, 0, 2))

        observation = []
        temp = []
        dist = np.zeros((8))
        for i in range(8):
            for j in range(100):
                temp.extend([self.path[i][j][0],self.path[i][j][1]])
                if j%2==0 and j!=98:              
                    dist[i]= dist[i] + math.sqrt((self.path[i][j+1][0]-self.path[i][j][0])**2+(self.path[i][j+1][1]-self.path[i][j][1])**2)
            
            temp_2 = (100*len_rew[i]) / (dist[i] * self.n_points)
            if temp_2 > 2:
                len_rew[i] = 2.0
            else:
                len_rew[i] = temp_2
            observation.append(temp)
            temp = []
        

        observation = np.array(observation)
        #print(len_rew)


        

        
        running = True
        steps=0
        cnt = 0
        finished = False
        
        #print(self.path[self.iter])
        #print(self.initial_pos)
        while not finished:
            
            finished = True
            self.iter = 0 
            #for agent,destination in zip(self.agents,self.path[self.iter]):
            for destination in  self.path_iter:
                for agent_num in range(8):
                    delta = b2Vec2(destination[agent_num]) - self.agents[agent_num].position
                    if delta.length <= self.epsilon/8:
                        #print('Yup')
                        self.agents[agent_num].linearVelocity = (0,0)
                        self.status()
                        continue
                    finished = False
                    direction = delta/delta.length
                    vel_mag = self.agents[agent_num].linearVelocity.length * direction.dot(self.agents[agent_num].linearVelocity)
                    force_mag = self.max_force*(1 - vel_mag/self.max_velocity)
                    force = force_mag*direction  
                    self.agents[agent_num].ApplyForce(force = force,point=self.agents[agent_num].position,wake=True)
                self.iter = self.iter + 1
                self.world.Step(self.time_step,10,10)
                if steps%self.sfr == 0:
                    self.render() 
                steps = (steps+1)
                if steps*self.time_step > 30:
                    raise RuntimeError("environment timestep exceeded 30 seconds")
                    finished = True
                if self.iter == 99:
                    finish = True
                    break
            self.world.Step(self.time_step,10,10)
            self.world.ClearForces()
            self.n_points = self.n_points - 100
            
            #observation = np.array([np.array(bots.position) for bots in self.agents])
            #observation = self.path_a
            status = self.status()
            done = False
            if all(self.status()):
                done = True
       
            for i in range(8):
                if status[i] == 1:
                    self.reward[i] = self.reward[i] + 10
            reward = self.reward
            reward = reward + len_rew
            #print(reward)
            
        
            return observation,reward,done,{}

        
        
        
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
                       

            
            

            

           
            
            
            

    def reset(self):

        
        self.world = world(gravity=(0,0),contactListener=ContactDetector(self))
        self.world2 = world(gravity=(0,0))
        self.n_points = 800
        

        row1 = [((-36 + 12 * i) * self.unit, 36 * self.unit) for i in range(7)]
        row2 = [((-36 + 24 * i) * self.unit, 24 * self.unit) for i in range(4)]
        row3 = [((-36 + 12 * i) * self.unit, 12 * self.unit) for i in range(7)]
        row4 = [((-36 + 24 * i) * self.unit, 0 * self.unit) for i in range(4)]
        row5 = [((-36 + 12 * i) * self.unit, -12 * self.unit) for i in range(7)]
        row6 = [((-36 + 24 * i) * self.unit, -24 * self.unit) for i in range(4)]
        row7 = [((-36 + 12 * i) * self.unit, -36 * self.unit) for i in range(7)]

        self.grid_centres = row1 + row2 + row3 + row4 + row5 + row6 + row7
        

        target = np.zeros((8,2))
        spawn = np.zeros((8,2))

        grid_num = np.random.choice(40,8,replace=False)
        for i in range(8):
            target[i] = self.grid_centres[grid_num[i]]
            target[i][0] = target[i][0] + np.random.uniform(-3*self.unit,3*self.unit,1)
            target[i][1] = target[i][1] + np.random.uniform(-3*self.unit,3*self.unit,1)

        """grid_num_spawn = np.random.choice(40,8,replace=False)
        for i in range(8):
            spawn[i] = self.grid_centres[grid_num_spawn[i]]
            spawn[i][0] = spawn[i][0] + np.random.uniform(-3*self.unit,3*self.unit,1)
            spawn[i][1] = spawn[i][1] + np.random.uniform(-3*self.unit,3*self.unit,1)"""


        

        self.target_pos = target
        #self.initial_pos = spawn

        self.walls()
        # Initial position of bots
        self.initial_pos=[(7.5*6*0.0254, 1.5*6*0.0254),(-7.5*6*0.0254, 1.5*6*0.0254),(7.5*6*0.0254, -1.5*6*0.0254),(-7.5*6*0.0254, -1.5*6*0.0254),
                            (6.5*6*0.0254, 2.5*6*0.0254),(-6.5*6*0.0254, 2.5*6*0.0254),(6.5*6*0.0254, -2.5*6*0.0254),(-6.5*6*0.0254, -2.5*6*0.0254)]

        #self.initial_pos=[(7.5*6*0.0254, 1.5*6*0.0254),(-7.5*6*0.0254, 1.5*6*0.0254),(-8*0.0254,0),(-7.5*6*0.0254, -1.5*6*0.0254),
                            #(6.5*6*0.0254, 2.5*6*0.0254),(-6.5*6*0.0254, 2.5*6*0.0254),(6.5*6*0.0254, -2.5*6*0.0254),(-6.5*6*0.0254, -2.5*6*0.0254)]
        #self.initial_pos = self.target_pos

        #self.target_pos = (0,0)

        # deploy bots
        self.body0 = self.world.CreateDynamicBody(position=self.initial_pos[0], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body1 = self.world.CreateDynamicBody(position=self.initial_pos[1], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body2 = self.world.CreateDynamicBody(position=self.initial_pos[2], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body3 = self.world.CreateDynamicBody(position=self.initial_pos[3], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body4 = self.world.CreateDynamicBody(position=self.initial_pos[4], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body5 = self.world.CreateDynamicBody(position=self.initial_pos[5], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body6 = self.world.CreateDynamicBody(position=self.initial_pos[6], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body7 = self.world.CreateDynamicBody(position=self.initial_pos[7], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))

        #self.body7.linearVelocity = (1,0)

        self.agents=[self.body0,self.body1,self.body2,self.body3,self.body4,self.body5,self.body6,self.body7]


        self.agent_iter = 0
        for agent in self.agents:
            agent.userData = self.agent_iter
            self.agent_iter = self.agent_iter + 1
        
        self.collide=[0]*self.num_agents
        self.world.gravity=(0,0)
        self.iter = 1
        self.reward = np.zeros((8))
        
        

        """self.path = np.zeros((8,self.n_points,2))
        for i in range(8):
            temp = calc_4points_bezier_path(self.initial_pos[i][0],self.initial_pos[i][1], 0.15, self.target_pos[i][0],self.target_pos[i][1], 0.15, 3, self.n_points)
            self.path[i] = temp[0]
        #np.concatenate((calc_4points_bezier_path(self.initial_pos[i][0],self.initial_pos[i][1], 0.52, self.target_pos[i][0],self.target_pos[i][1], 0.52, 0.1) for i in range(8)), out = self.path  
        
        self.path_a =self.path                               #before transpose : size-> (8,n_points,2)
        self.path = np.transpose(self.path, (1, 0, 2))       #after transpose  : size-> (n_points,8,2)"""
        #print(self.path_a.shape)
        #print(self.path.shape)

        #self.path_status()
        ret = []
        temp_1 = []
        temp_2 = []
        """for i in range(8):
            temp_1 = [self.initial_pos[i][0],self.initial_pos[i][1]]*20
            temp_2 = [self.target_pos[i][0],self.target_pos[i][1]]*20
            temp_1.extend(temp_2)
            ret.append(temp_1)"""
        for i in range(8):
            temp_1 = [0,0]*50
            temp_2 = [self.target_pos[i][0]-self.initial_pos[i][0],self.target_pos[i][1]-self.initial_pos[i][1]]*50
            temp_1.extend(temp_2)
            ret.append(temp_1)

        ret = np.array(ret)
        
        
        

        return ret
        

    


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
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 1*6*0.0254 ), (1*6*0.0254, 1*6*0.0254)])#12 * 0.0254
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


    def dummy_walls(self):
        # upper lower wall
        ground = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, 40*0.0254 ), (40*0.0254, 40*0.0254)])
        )
        ground1 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, -40*0.0254 ), (40*0.0254, -40*0.0254)])
        )
        # middle
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, 1*8*0.0254 ), (1*8*0.0254, 1*8*0.0254)])#12 * 0.0254
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, -1*8*0.0254 ), (1*8*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*8*0.0254, 1*8*0.0254 ), (1*8*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, 1*8*0.0254 ), (-1*8*0.0254, -1*8*0.0254)])
        )
        # middle top
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, 32*0.0254 ), (1*8*0.0254, 32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, 16*0.0254 ), (1*8*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*8*0.0254, 32*0.0254 ), (1*8*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, 32*0.0254 ), (-1*8*0.0254, 16*0.0254)])
        )
        # middle low
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, -32*0.0254 ), (1*8*0.0254, -32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, -16*0.0254 ), (1*8*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*8*0.0254, -32*0.0254 ), (1*8*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*8*0.0254, -32*0.0254 ), (-1*8*0.0254, -16*0.0254)])
        )
        # right left
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, 1*8*0.0254 ), (32*0.0254, 1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, -1*8*0.0254 ), (32*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, 1*8*0.0254 ), (16*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(32*0.0254, 1*8*0.0254 ), (32*0.0254, -1*8*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, 1*8*0.0254 ), (-32*0.0254, 1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, -1*8*0.0254 ), (-32*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, 1*8*0.0254 ), (-16*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-32*0.0254, 1*8*0.0254 ), (-32*0.0254, -1*8*0.0254)])
        )
        # corner
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, 32*0.0254 ), (32*0.0254, 32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, 16*0.0254 ), (32*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, 32*0.0254 ), (16*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(32*0.0254, 32*0.0254 ), (32*0.0254, 16*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, 32*0.0254 ), (-32*0.0254, 32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, 16*0.0254 ), (-32*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, 32*0.0254 ), (-16*0.0254, 16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-32*0.0254, 32*0.0254 ), (-32*0.0254, 16*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, -32*0.0254 ), (32*0.0254, -32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, -16*0.0254 ), (32*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(16*0.0254, -32*0.0254 ), (16*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(32*0.0254, -32*0.0254 ), (32*0.0254, -16*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, -32*0.0254 ), (-32*0.0254, -32*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, -16*0.0254 ), (-32*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-16*0.0254, -32*0.0254 ), (-16*0.0254, -16*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-32*0.0254, -32*0.0254 ), (-32*0.0254, -16*0.0254)])
        )
        # side wall
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, 40*0.0254 ), (40*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, 40*0.0254 ), (-40*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, -40*0.0254 ), (40*0.0254, -10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, -40*0.0254 ), (-40*0.0254, -10*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, 1*8*0.0254 ), (40*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, 1*8*0.0254 ), (-40*0.0254, -1*8*0.0254)])
        )
        # chute
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(46*0.0254, 1*8*0.0254 ), (46*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-46*0.0254, 1*8*0.0254 ), (-46*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(46*0.0254, -1*8*0.0254 ), (46*0.0254, -10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-46*0.0254, -1*8*0.0254 ), (-46*0.0254, -10*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, 1*8*0.0254 ), (46*0.0254, 1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, 1*8*0.0254 ), (-46*0.0254, 1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, -1*8*0.0254 ), (46*0.0254, -1*8*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, -1*8*0.0254 ), (-46*0.0254, -1*8*0.0254)])
        )

        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, 10*0.0254 ), (46*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, 10*0.0254 ), (-46*0.0254, 10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(40*0.0254, -10*0.0254 ), (46*0.0254, -10*0.0254)])
        )
        ground3 = self.world2.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-40*0.0254, -10*0.0254 ), (-46*0.0254, -10*0.0254)])
        )   

        

        
        
        """ground.userData = "wall"
        ground1.userData = "wall"
        ground3.userData = "wall"""

    
    
        

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
        self.draw_path()

        

        

        
        
        
        pygame.display.flip()
        self.clock.tick(self.fps)

        


    def close(self):
        pygame.quit()
