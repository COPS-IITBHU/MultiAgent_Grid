import random
import gym 
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
import scipy.special
import pygame
from pygame.locals import(QUIT,KEYDOWN,K_ESCAPE)

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
num_agents=8
show_animation = True
epsilon=0.01
n_points=500
scalar_force=0.005

collision_reward=-100

class ContactDetector(contactListener):

    def __init__(self, env):
        contactListener.__init__(self)
        self.env=env

    def BeginContact(self,contact):

        for i in range(0,num_agents):
            if(self.env.agents[i] == contact.fixtureA.body or self.env.body == contact.fixtureB.body):
                self.reward+=collision_reward
                if(self.collide[i]):
                    self.reward+=collision_reward
                self.collide[i]=1

    def EndContact(self, contact):

        for i in range(0,num_agents):
            if self.env.agents[i] in [contact.fixtureA.body, contact.fixture.body]:
                self.collide[i]=0

class grid(gym.Env):
   
    
    metadata={'render.modes':['human']}

    def __init__(self):
        self.num_agents = 8
        self.radius = 2 * 0.0254
        self.density = 1.0
        self.restitution = 0
        self.fps = 50
        self.sfr = 1
        self.screen_height = 600
        self.screen_width = 800
        self.viewer = None
        self.ppm = 240
        self.velocity=0.2
        self.path=np.zeros(shape=(self.num_agents,n_points,2))

        self.time_step = 1./self.fps*self.sfr


        

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

        
    def step(self, action):

        running = True
        steps=0
        while running:

            '''for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                    running = False'''

            self.obs = []
            for i in range(0,self.num_agents):
                f = self.agents[i].GetWorldVector(localVector=action[i])
                p = self.agents[i].GetWorldPoint(localPoint=(0.0 , 0.0))
                self.agents[i].ApplyForce(f, p, True)
                self.obs.append([self.agents[i].position[0],self.agents[i].position[1],self.agents[i].linearVelocity[0],self.agents[i].linearVelocity[1]])
                
            
            print(self.obs)
            print('_________________________________')
              

            self.world.Step(self.time_step,10,10)
            if steps%self.sfr == 0:
                self.render() 
            steps = (steps+1)
            if steps*self.time_step > 30:
                raise RuntimeError("environment timestep exceeded 30 seconds")

            

           
            
            
            

    def reset(self):

        self.screen=None
        self.world = world(gravity=(0,0))

        self.walls()
        # Initial position of bots
        self.initial_pos=[(7.5*6*0.0254, 1.5*6*0.0254),(-7.5*6*0.0254, 1.5*6*0.0254),(7.5*6*0.0254, -1.5*6*0.0254),(-7.5*6*0.0254, -1.5*6*0.0254),
                            (6.5*6*0.0254, 2.5*6*0.0254),(-6.5*6*0.0254, 2.5*6*0.0254),(6.5*6*0.0254, -2.5*6*0.0254),(-6.5*6*0.0254, -2.5*6*0.0254)]

        # deploy bots
        self.body1 = self.world.CreateDynamicBody(position=self.initial_pos[0], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body2 = self.world.CreateDynamicBody(position=self.initial_pos[1], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body0 = self.world.CreateDynamicBody(position=self.initial_pos[2], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body3 = self.world.CreateDynamicBody(position=self.initial_pos[3], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body4 = self.world.CreateDynamicBody(position=self.initial_pos[4], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body5 = self.world.CreateDynamicBody(position=self.initial_pos[5], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body6 = self.world.CreateDynamicBody(position=self.initial_pos[6], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))
        self.body7 = self.world.CreateDynamicBody(position=self.initial_pos[7], fixtures=b2FixtureDef(shape=b2CircleShape(radius=self.radius), density=self.density, restitution=self.restitution))

        self.body7.linearVelocity = (1,0)

        self.agents=[self.body0,self.body1,self.body2,self.body3,self.body4,self.body5,self.body6,self.body7]
        self.reward=0
        self.collide=[0]*self.num_agents
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
        

    
        
    def calc_4points_bezier_path(self, sx, sy, ex, ey,control_points, offset):

        dist = np.hypot(sx - ex, sy - ey) / offset
        # control_points = np.array(
        #     [[sx, sy],
        #      [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
        #      [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
        #      [ex, ey]])
        # control_points = np.array(
        #         [[sx,sy],
        #         [3,-3],
        #         [4,5],
        #         [5,10],
        #         [ex,ey]])

        path = self.calc_bezier_path(control_points, n_points=500)

        return path, control_points


    def calc_bezier_path(self, control_points, n_points=500):
        """
        Compute bezier path (trajectory) given control points.

        :param control_points: (numpy array)
        :param n_points: (int) number of points in the trajectory
        :return: (numpy array)
        """
        traj = []
        for t in np.linspace(0, 1, n_points):
            traj.append(self.bezier(t, control_points))

        return np.array(traj)


    def bernstein_poly(self, n, i, t):
        """
        Bernstein polynom.

        :param n: (int) polynom degree
        :param i: (int)
        :param t: (float)
        :return: (float)
        """
        return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


    def bezier(self, t, control_points):
        """
        Return one point on the bezier curve.

        :param t: (float) number in [0, 1]
        :param control_points: (numpy array)
        :return: (numpy array) Coordinates of the point
        """
        n = len(control_points) - 1
        return np.sum([self.bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


    def bezier_derivatives_control_points(self, control_points, n_derivatives):
        """
        Compute control points of the successive derivatives of a given bezier curve.

        A derivative of a bezier curve is a bezier curve.
        See https://pomax.github.io/bezierinfo/#derivatives
        for detailed explanations

        :param control_points: (numpy array)
        :param n_derivatives: (int)
        e.g., n_derivatives=2 -> compute control points for first and second derivatives
        :return: ([numpy array])
        """
        w = {0: control_points}
        for i in range(n_derivatives):
            n = len(w[i])
            w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                                for j in range(n - 1)])
        return w


    def curvature(self, dx, dy, ddx, ddy):
        """
        Compute curvature at one point given first and second derivatives.

        :param dx: (float) First derivative along x axis
        :param dy: (float)
        :param ddx: (float) Second derivative along x axis
        :param ddy: (float)
        :return: (float)
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


    def bezier_path(self, start_x, start_y, end_x, end_y, control_points):
        """Show the effect of the offset."""
        start_x = 100.0  # [m]
        start_y = 10.0  # [m]
        start_yaw = np.radians(180.0)  # [rad]

        end_x = -0.0  # [m]
        end_y = 2.0  # [m]
        end_yaw = np.radians(-45.0)  # [rad]

        for offset in np.arange(1.0, 5.0, 1.0):
            path, control_points = self.calc_4points_bezier_path(
                start_x, start_y, end_x, end_y, control_points, offset)
            assert path.T[0][0] == start_x, "path is invalid"
            assert path.T[1][0] == start_y, "path is invalid"
            assert path.T[0][-1] == end_x, "path is invalid"
            assert path.T[1][-1] == end_y, "path is invalid"

            if show_animation:  # pragma: no cover
                plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))

        if show_animation:  # pragma: no cover
            # self.plot_arrow(start_x, start_y, start_yaw)
            # self.plot_arrow(end_x, end_y, end_yaw)
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.show()
        print(path)
        return path


    def update_path(self,index,new_target):

        # get bezier points here
        new_path=self.bezier_path(self.path[index,n_points-1,0],self.path[index,n_points-1,1],new_target[0],new_target[1], control_points)

        self.path[index,:,:]=new_path[:,:]
   

    def render(self, mode='human',close=False):
        if self.screen is None:
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height),0,32)
            pygame.display.set_caption('grid-2D')
            self.clock = pygame.time.Clock()

        self.screen.fill((0,0,0,0))

        for body in self.world.bodies:
            for fixture in body.fixtures:
                fixture.shape.draw(body,fixture)
        pygame.display.flip()
        self.clock.tick(self.fps)

    def step(self):

        for i in range(0,self.num_agents):
            for t in self.path[i]:
                if ((t[0]-self.path[i,n_points-1,0])**2 + (t[1]-self.path[i,n_points-1,1])**2 < epsilon**2):

                    # keep update new target here
                    self.update_path(i,new_target) 

                elif((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2 > epsilon**2):
                    dis=sqrt((t[0]-self.agents[i].position[0])**2 + (t[1]-self.agents[i].position[1])**2)

                    f = self.agents[i].GetWorldVector(localVector=(scalar_force*(t[0]-self.agents[i].position[0])/dis,(scalar_force*t[1]-self.agents[i].position[1])/dis))
                    p = self.agents[i].GetWorldPoint(localPoint=(0.0, 0.00))
                    self.agents[i].ApplyForce(f, p, True)
                    break

    def get_reward(self):

        self.world.contactListener=ContactDetector(self)

    def close(self):
        pygame.quit()



if __name__ == '__main__':
    env=grid()
    env.reset()
    env.bezier_path()
