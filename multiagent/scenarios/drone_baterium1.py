import numpy as np
import scipy.io
from multiagent.core1 import World, Landmark, Wall
from multiagent.core1 import flockRobot1 as flockRobot
from multiagent.scenario import BaseScenario

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import math

class Scenario(BaseScenario):
    def make_world(self,shape):
        world = World()
        world.p_shape = shape
        # add agents
        world.agents = [flockRobot(i) for i in range(20)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
        # add landmarks
        world.landmarks = [Landmark() for i in range(1)]
        for i, landmark in enumerate(world.landmarks):
                landmark.name = 'landmark%d' % i
                landmark.collide = False
                landmark.movable = False

        world.walls = [Wall() for i in range(4)]
        for i, wall in enumerate(world.walls):
            if i == 0 or i == 1:
                wall.name = 'wall%d' % i
                wall.collide = True
                wall.movable = False
                wall.length = 2.0
                wall.width = 0.05
                wall.endpoints[0] = -1
                wall.endpoints[1] = 1
            else:
                wall.name = 'wall%d' % i
                wall.collide = True
                wall.movable = False
                wall.orient = 1
                wall.length = 0.05
                wall.width = 2.0
                wall.endpoints[0] = -1
                wall.endpoints[1] = 1

        world.robotInfo = np.zeros([len(world.agents), 4])
        world.neighbourData = np.zeros([len(world.agents),5])
        # make initial conditions
        self.reset_world(world)
        self.calculate_pollution_map(world,shape)

        return world
    
    def reset_world(self, world):
        world.timestep = 0
        # random properties for agents
        for i, agent in enumerate(world.agents):
            agent.color = np.array([2.5,0,0])
        world.agents[0].color = np.array([1.0,1.0,0.0])
        # random properties for landmarks
        for i, landmark in enumerate(world.landmarks):
            landmark.color = np.array([0.0,1.0,0.0])
        # walls     
        for i, wall in enumerate(world.walls):
            wall.color = np.array([0.70,0.05,0.405])

        for i, landmark in enumerate(world.landmarks):
            # landmark.state.p_pos = np.random.uniform(-1,+1, world.dim_p)
            landmark.state.p_pos = np.array([0.0,0.0])
            landmark.state.p_vel = np.zeros(world.dim_p)
            
        for i, wall in enumerate(world.walls):
            if i == 0:
                wall.state.p_pos = np.array([0.0,1.0])
                wall.state.p_vel = np.zeros(world.dim_p)
               
            if i == 1:
                wall.state.p_pos = np.array([0.0,-1.0])
                wall.state.p_vel = np.zeros(world.dim_p)
            if i == 2:
                wall.state.p_pos = np.array([-1.0,0.0])
                wall.state.p_vel = np.zeros(world.dim_p)
            if i == 3:
                wall.state.p_pos = np.array([1.0,0.0])
                wall.state.p_vel = np.zeros(world.dim_p)

        # set random initial states
        self.random_position_spread(world)

        for i, agent in enumerate(world.agents):
            agent.state.p_vel = np.zeros(world.dim_p)
            agent.flockXPos = agent.state.p_pos[0]
            agent.flockYPos = agent.state.p_pos[1]
            # agent.state.heading = np.random.uniform(0,2*np.pi)
            # agent.old_targetDis = self.get_distance(agent.state.p_pos, world.landmarks[0].state.p_pos)  

    def check_inside(self,pos):
        if pos[0] > -0.3 and pos[0] < 0.3 and pos[1] > -0.3 and pos[1] < 0.3:
            return 1
        else:
            return 0

    def random_position_spread(self,world):
        for i, agent in enumerate(world.agents):
            agent.state.p_pos = np.random.uniform(-1,+1, world.dim_p)
            while self.check_inside(agent.state.p_pos):
                agent.state.p_pos = np.random.uniform(-1,+1, world.dim_p)

    def calculate_pollution_map(self, world, shape):
        #the counting has been amplified by 1000 times
        if shape == "circle_nonuni":    
            xPos = int(world.landmarks[0].state.p_pos[0]/world.interval)
            yPos = int(world.landmarks[0].state.p_pos[1]/world.interval)

            for distFrmSrc in range(0, int(world.landmarks[0].size/(3*world.interval)+1)):
                for angle in np.arange(0,360,0.1):
                    xPollu = distFrmSrc * np.math.cos((3.142/180) * angle)
                    yPollu = distFrmSrc * np.math.sin((3.142/180) * angle)
                    
                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)
                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 3

            for distFrmSrc in range(int(world.landmarks[0].size/(3*world.interval)+1), int(world.landmarks[0].size/(1.5*world.interval)+1)):
                for angle in np.arange(0,360,0.1):
                    xPollu = distFrmSrc * np.math.cos((3.142/180) * angle)
                    yPollu = distFrmSrc * np.math.sin((3.142/180) * angle)
                    
                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)

                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 2 
            
            for distFrmSrc in range(int(world.landmarks[0].size/(1.5*world.interval)+1), int(world.landmarks[0].size/world.interval+1)):
                for angle in np.arange(0,360,0.01):
                    xPollu = distFrmSrc * np.math.cos((3.142/180) * angle)
                    yPollu = distFrmSrc * np.math.sin((3.142/180) * angle)
                    
                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)

                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 1

        if shape == "square_uni":
            xPos = int(world.landmarks[0].state.p_pos[0]/world.interval)
            yPos = int(world.landmarks[0].state.p_pos[1]/world.interval)

            for xPollu in range(int(-0.3/world.interval),int(0.3/world.interval)):
                for yPollu in range(int(-0.2/world.interval),int(0.2/world.interval)):
                    
                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)
                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 2

        if shape == "peanut_uni":

            # phi = 0.0:pi:100
            # theta = 0.0:2.0*pi:100
            # x = r*sin(phi)*cos(theta)
            # y = r*sin(phi)*sin(theta)
            # z = r*cos(phi)

            xPos = int(-0.2/world.interval)
            yPos = int(0.0/world.interval)

            xPos1 = int(0.2/world.interval)
            yPos1 = int(0.0/world.interval)

            for phi in np.arange(0,90,0.1):
                for theta in np.arange(0,360,0.1):
                    xPollu = 0.2/(world.interval) * np.math.sin((3.142/180) * phi) * np.math.cos((3.142/180) * theta)
                    yPollu = 0.2/(world.interval) * np.math.sin((3.142/180) * phi) * np.math.sin((3.142/180) * theta)

                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)

                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 3*np.math.cos((3.142/180) * phi)
            
            for phi in np.arange(0,90,0.1):
                for theta in np.arange(0,360,0.1):
                    xPollu = 0.2/(world.interval) * np.math.sin((3.142/180) * phi) * np.math.cos((3.142/180) * theta)
                    yPollu = 0.2/(world.interval) * np.math.sin((3.142/180) * phi) * np.math.sin((3.142/180) * theta)

                    tempX = xPos1 + int(xPollu)
                    tempY = yPos1 + int(yPollu)

                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 3*np.math.cos((3.142/180) * phi)
           
        if shape == "U_uni":
            xPos = 0
            yPos = 0

            xPos1 = int(-0.4/world.interval)
            yPos1 = int(0.2/world.interval)

            xPos2 = int(0.4/world.interval)
            yPos2 = int(0.2/world.interval)

            for xPollu in range(int(-0.3/world.interval),int(0.3/world.interval)):
                for yPollu in range(int(-0.1/world.interval),int(0.1/world.interval)):
                    
                    tempX = xPos + int(xPollu)
                    tempY = yPos + int(yPollu)
                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 2

            for xPollu in range(int(-0.1/world.interval),int(0.1/world.interval)):
                for yPollu in range(int(-0.3/world.interval),int(0.3/world.interval)):
                    
                    tempX = xPos1 + int(xPollu)
                    tempY = yPos1 + int(yPollu)
                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 2

            for xPollu in range(int(-0.1/world.interval),int(0.1/world.interval)):
                for yPollu in range(int(-0.3/world.interval),int(0.3/world.interval)):
                    
                    tempX = xPos2 + int(xPollu)
                    tempY = yPos2 + int(yPollu)
                    #transform coordinate
                    tempX1 = tempX + int(world.xsize/2)
                    tempY1 = tempY + int(world.ysize/2)
                    world.pollution_map[tempX1][tempY1] = 2


        # scipy.io.savemat('pollution_map.mat', mdict={'arr': world.pollution_map})
        # print('Finsh saving the map')

    
    
     