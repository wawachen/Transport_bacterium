import numpy as np
import scipy.io
from multiagent.core import World, Agent, Landmark
from multiagent.scenario import BaseScenario

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Scenario(BaseScenario):
    def make_world(self):
        world = World()
        
        # add agents
        world.agents = [Agent() for i in range(50)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
        # add landmarks
        world.landmarks = [Landmark() for i in range(1)]
        for i, landmark in enumerate(world.landmarks):
            landmark.name = 'landmark %d' % i
            landmark.collide = True
            landmark.movable = True
        # make initial conditions
        self.reset_world(world)
        self.calculate_pollution_map(world)
        world.get_contact_points()
        return world
    
    def reset_world(self, world):
        # random properties for agents
        for i, agent in enumerate(world.agents):
            agent.color = np.array([2.5,0,0])
        # random properties for landmarks
        for i, landmark in enumerate(world.landmarks):
            landmark.color = np.array([0.0,0.705,0.05])
        world.landmarks[0].color = np.array([0.0,1.0,0.0])
        # set random initial states
        for i, agent in enumerate(world.agents):
            agent.state.p_pos = np.zeros(2)
            agent.state.p_pos[0] = 0.25*np.math.cos(np.math.radians((i-50)*7))
            agent.state.p_pos[1] = 0.25*np.math.sin(np.math.radians((i-50)*7))
            agent.state.p_vel = np.zeros(world.dim_p)
            
        for i, landmark in enumerate(world.landmarks):
            # landmark.state.p_pos = np.random.uniform(-1,+1, world.dim_p)
            landmark.state.p_pos = np.array([0.0,0.0])
            landmark.state.p_vel = np.zeros(world.dim_p)

    def calculate_pollution_map(self, world):
        #the counting has been amplified by 1000 times
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

        # scipy.io.savemat('pollution_map.mat', mdict={'arr': world.pollution_map})
        # print('Finsh saving the map')
		
    
    def reward(self, agent, world,obs):
        dist2 = np.sum(np.square(agent.state.p_pos - world.landmarks[0].state.p_pos))
        return -dist2

    def observation(self, agent, world):
        # get positions of all entities in this agent's reference frame
        # get the centration and the relative distance to neighbours in the range 
        concentration = world.getCentreDensity(agent)
        entity_pos = []
        for entity in world.landmarks:
            entity_pos.append(entity.state.p_pos - agent.state.p_pos)
        return np.concatenate([agent.state.p_vel] + entity_pos+[[concentration]])
