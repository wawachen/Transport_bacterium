#!/usr/bin/env python
import os,sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import argparse

from multiagent.bacterium_environment1 import MultiAgentEnv_bacterium1
import multiagent.scenarios as scenarios
# from multiagent.scenarios.drone_baterium1 import Scenario
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import savemat

if __name__ == '__main__':

    try:
        # parse arguments
        parser = argparse.ArgumentParser(description=None)
        parser.add_argument('-s', '--scenario', default='simple.py', help='Path of the scenario Python script.')
        args = parser.parse_args()

        #payload shape
        shape = "circle_nonuni"
        # load scenario from script
        scenario = scenarios.load(args.scenario).Scenario()
        # create world
        world = scenario.make_world(shape)
        # create multiagent environment
        env = MultiAgentEnv_bacterium1(world, shared_viewer = True)
        # render call to create viewer window (necessary only for interactive policies)
        env.render(shape)
        flag = 1
        first_t = 0
        
        while True:
            # query for action from each agent's policy
            # step environment
            env.step()
            oo = env.world.swarm_trigger()
            # #circle:6.4, square:9.6, peanut: 11.9 u :20.8
            # if oo or first_t: #3.98
            #     env.world.landmarks[0].movable = True
            #     env.world.mode = 1
            #     env.world.new = 1
            #     flag = 0
            #     first_t = 1
            # else:
            #     env.world.mode = 0
            # render all agent views
            env.render(shape)
            # display rewards
            #for agent in env.world.agents:
            #    print(agent.name + " reward: %0.3f" % env._get_reward(agent))
    
    finally:
        print("start compute KL divergence")
        savemat('storeX.mat', mdict={'arr': env.storeX})
        savemat('storeY.mat', mdict={'arr': env.storeY})
        savemat('storeT.mat', mdict={'arr': env.T_collection})
        savemat('storeVX.mat', mdict={'arr': env.v_x})
        savemat('storeVY.mat', mdict={'arr': env.v_y})

        savemat('storeattX.mat', mdict={'arr': env.attach_pointsx})
        savemat('storeattY.mat', mdict={'arr': env.attach_pointsy})
        savemat('storePayload.mat', mdict={'arr': env.paypos})

        savemat('stored.mat', mdict={'arr': env.world.d_coll})
        savemat('storecen.mat', mdict={'arr': env.world.cen_coll})
        print('Finsh saving the map')

       