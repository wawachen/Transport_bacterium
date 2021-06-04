import gym
from gym import spaces
from gym.envs.registration import EnvSpec
import numpy as np


# environment for all agents in the multiagent world
# currently code assumes that no agents will be created/destroyed at runtime!
class MultiAgentEnv_bacterium1(gym.Env):
    metadata = {
        'render.modes' : ['human', 'rgb_array']
    }

    def __init__(self, world, shared_viewer=True):

        self.world = world
        self.agents = self.world.policy_agents
        # set required vectorized gym env property
        self.n = len(world.policy_agents)

        self.storeX = []
        self.storeY = []
        self.T_collection = []
        self.v_x = []
        self.v_y = []
        self.goal_x = -0.6 
        self.goal_y = -0.6
        self.attach_pointsx = []
        self.attach_pointsy = []
        self.paypos = []
        self.attach_signal = np.zeros(self.n)

        # rendering
        self.shared_viewer = shared_viewer
        if self.shared_viewer:
            self.viewers = [None]
        else:
            self.viewers = [None] * self.n
        self._reset_render()

    def step(self):
        self.paypos.append(self.world.landmarks[0].state.p_pos)
        # print(self.world.timestep)
        # print(self.world.landmarks[0].state.p_pos[0],self.world.landmarks[0].state.p_pos[1])
        # if self.world.new == 0:
        sx = np.zeros(len(self.agents))
        sy = np.zeros(len(self.agents))

        for i,agent in enumerate(self.agents):
            sx[i] = agent.state.p_pos[0]
            sy[i] = agent.state.p_pos[1]

        self.storeX.append(sx)
        self.storeY.append(sy)

        high_concen = np.zeros(self.n)
        T_s_collection = np.zeros(self.n)
        for i in range(self.n):
            high_concen[i] = self.agents[i].myData[3]
            T_s_collection[i] = self.agents[i].tumbleLenght

        self.T_collection.append(T_s_collection)
        
        highest_concen = np.max(high_concen)

        all_gather = 0

        for i in range(self.n):
            if self.agents[i].myData[3] > (0.9*highest_concen):
                self.agents[i].bac_state = 1
            else:
                self.agents[i].bac_state = 0
            all_gather += self.agents[i].bac_state
        
        action_n = np.zeros([len(self.agents),2])
        for j in range(self.n):
            if self.agents[j].role == 0:
                if self.world.new == 0:
                    self.agents[j].run(self.world)
                    if all_gather == self.n:
                        action_n[j,0] = 0.1*self.agents[j].myXVelocity 
                        action_n[j,1] = 0.1*self.agents[j].myYVelocity 
                    else:
                        if self.agents[j].getCentreDensity(self.world)>(0.9*highest_concen):
                            if self.agents[j].num_neighbours > 0:
                                if self.world.p_shape == "peanut_uni":
                                    action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  + 0.1*self.agents[j].myXVelocity 
                                    action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle + 0.1*self.agents[j].myYVelocity 
                                else:
                                    action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  + 0.1*self.agents[j].myXVelocity + 0.1*self.agents[j].shepherdXVelocity
                                    action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle + 0.1*self.agents[j].myYVelocity + 0.1*self.agents[j].shepherdYVelocity
                                # print("flock",0.1*self.agents[j].myXVelocity,0.1*self.agents[j].myYVelocity,"bac",self.agents[j].bactVelocity * self.agents[j].cosAngle,self.agents[j].bactVelocity * self.agents[j].sinAngle, "shepherd", 0.1*self.agents[j].shepherdXVelocity, 0.1*self.agents[j].shepherdYVelocity)
                            else:
                                if self.world.p_shape == "peanut_uni":
                                    action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  
                                    action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle 
                                else:
                                    action_n[j,0] = 2*self.agents[j].bactVelocity * self.agents[j].cosAngle + 0.1*self.agents[j].shepherdXVelocity 
                                    action_n[j,1] = 2*self.agents[j].bactVelocity * self.agents[j].sinAngle + 0.1*self.agents[j].shepherdYVelocity
                                # print("bac",2*self.agents[j].bactVelocity * self.agents[j].cosAngle,2*self.agents[j].bactVelocity * self.agents[j].sinAngle,"shepherd",0.1*self.agents[j].shepherdXVelocity,0.1*self.agents[j].shepherdYVelocity)

                            # action_n[j,0] = 0.1*self.agents[j].myXVelocity
                            # action_n[j,1] = 0.1*self.agents[j].myYVelocity
                            # action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  #+ 0.1*self.agents[j].myXVelocity 
                            # action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle #+ 0.1*self.agents[j].myYVelocity 
                            # print("bacin",action_n)
                        else:
                            if self.agents[j].myData[3] > (0.5*highest_concen):
                                action_n[j,0] = 2*self.agents[j].bactVelocity * self.agents[j].cosAngle  
                                action_n[j,1] = 2*self.agents[j].bactVelocity * self.agents[j].sinAngle 
                            else:
                                action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  
                                action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle
                            
                            # print("bac",action_n)

                        # action_n[j,0] = self.agents[j].bactVelocity  * self.agents[j].cosAngle
                        # action_n[j,1] = self.agents[j].bactVelocity  * self.agents[j].sinAngle

                        # action_n[j,0] = self.agents[j].bactVelocity * self.agents[j].cosAngle  + self.agents[j].myXVelocity
                        # action_n[j,1] = self.agents[j].bactVelocity * self.agents[j].sinAngle + self.agents[j].myYVelocity
                        # action_n[j,0] =  0.1*self.agents[j].myXVelocity
                        # action_n[j,1] = 0.1*self.agents[j].myYVelocity
                        
                elif self.agents[j].state.p_pos[1]>0:
                    action_n[j,0] = 0.0
                    action_n[j,1] = 0.001
                else:
                    action_n[j,0] = 0.0
                    action_n[j,1] = -0.001
            else:
                if self.attach_signal[j] == 0:
                    self.attach_pointsx.append(self.agents[j].state.p_pos[0])
                    self.attach_pointsy.append(self.agents[j].state.p_pos[1])
                self.attach_signal[j] = 1

                posx_c = np.zeros(len(self.agents))
                posy_c = np.zeros(len(self.agents))
                for i in range(len(self.agents)):
                    pos_c = self.agents[i].state.p_pos
                    posx_c[i] = pos_c[0]
                    posy_c[i] = pos_c[1]
                centroid_x = np.mean(posx_c)
                centroid_y = np.mean(posy_c)

                v_f = np.array([self.goal_x, self.goal_y]) - np.array([centroid_x,centroid_y])
                d = np.sqrt((v_f**2).sum())
                vector_force = v_f/d

                vel = np.zeros(3)
                v_f1 = vector_force*(d)*0.001
                vel[0] = v_f1[0]
                vel[1] = v_f1[1]

                action_n[j,0] = vel[0]
                action_n[j,1] = vel[1]
            
        # set action for each agent
        self.v_x.append(action_n[:,0])
        self.v_y.append(action_n[:,1])
        for i, agent in enumerate(self.agents):
            self._set_action(action_n[i,:], agent)
        # advance world state
        self.world.step()

        
    # def reset(self):
    #     # reset world
    #     self.reset_callback(self.world)
    #     # reset renderer
    #     self._reset_render()
    #     # record observations for each agent
    #     self.agents = self.world.policy_agents


    # set env action for a particular agent
    def _set_action(self, action, agent, time=None):
        agent.action.u = np.zeros(self.world.dim_p)
        agent.action.f = np.zeros(self.world.dim_f)
        # action = [action]

        if agent.movable:
            # physical action
            agent.action.u = action
            sensitivity = 5.0
            if agent.accel is not None:
                sensitivity = agent.accel
            agent.action.u *= sensitivity
            # action = action[1:]
        # make sure we used all elements of action
        assert len(action) == 2

    # # reset rendering assets
    def _reset_render(self):
        self.render_geoms = None
        self.render_geoms_xform = None

    # render environment
    def render(self, shape,mode='human'):
        # if mode == 'human':
        #     alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        #     message = ''
        #     for agent in self.world.agents:
        #         comm = []
        #         for other in self.world.agents:
        #             if other is agent: continue
        #             if np.all(other.state.c == 0):
        #                 word = '_'
        #             else:
        #                 word = alphabet[np.argmax(other.state.c)]
        #             message += (other.name + ' to ' + agent.name + ': ' + word + '   ')
        for i in range(len(self.viewers)):
            # create viewers (if necessary)
            if self.viewers[i] is None:
                # import rendering only if we need it (and don't import for headless machines)
                #from gym.envs.classic_control import rendering
                from multiagent import rendering
                self.viewers[i] = rendering.Viewer(700,700)

        # create rendering geometry
        # if self.render_geoms is None:
            # import rendering only if we need it (and don't import for headless machines)
            #from gym.envs.classic_control import rendering
    
        from multiagent import rendering
        self.render_geoms = [] #agent
        self.render_geoms_xform = []
        self.render_geoms1 = [] #payload
        self.render_geoms_xform1 = []
    
        
        for i, entity in enumerate(self.world.entities):
            # geom = rendering.make_circle(entity.size)
            # xform = rendering.Transform()
            if 'agent' in entity.name:
                geom = rendering.make_circle(entity.size)
                xform = rendering.Transform()
                #print(self.world.timestep,entity.color)
                geom.set_color(*entity.color)

                geom.add_attr(xform)
                self.render_geoms.append(geom)
                self.render_geoms_xform.append(xform)
            else:
                # if iw == 0:
                    # geom1 = rendering.make_circle1(entity.size)
                if shape == "circle_nonuni":
                    geom1 = rendering.make_capsule(0.3)

                # if shape == "L_uni":
                
                if shape == "peanut_uni":
                    geom1 = rendering.make_capsule1(0.2)

                if shape == "square_uni":
                    geom1 = rendering.make_polygon1(0.6, 0.4)
                
                if shape == "U_uni":
                    geom1 = rendering.make_capsule2(0.4)

                xform1 = rendering.Transform()
                geom1.set_color(*entity.color,alpha = 1.0) #alpha is the transparency the lower can be ovelapped

                geom1.add_attr(xform1)
                self.render_geoms1.append(geom1)
                self.render_geoms_xform1.append(xform1)

        for i, wall in enumerate(self.world.walls):
        # geom = rendering.make_circle(entity.size)
        # xform = rendering.Transform()
            if i == 0 or i ==1:  
                
                geom1 = rendering.make_polygon1(wall.length, wall.width)
                xform1 = rendering.Transform()
                geom1.set_color(*wall.color,alpha = 1.0) #alpha is the transparency the lower can be ovelapped

                geom1.add_attr(xform1)
                self.render_geoms1.append(geom1)
                self.render_geoms_xform1.append(xform1)
            
            if i == 2 or i == 3: 

                geom1 = rendering.make_polygon1(wall.length, wall.width)
                xform1 = rendering.Transform()
                geom1.set_color(*wall.color,alpha = 1.0) #alpha is the transparency the lower can be ovelapped

                geom1.add_attr(xform1)
                self.render_geoms1.append(geom1)
                self.render_geoms_xform1.append(xform1)
    
            # geom.add_attr(xform)
            # self.render_geoms.append(geom)
            # self.render_geoms_xform.append(xform)

        # add geoms to viewer
        for viewer in self.viewers:
            viewer.geoms = []
            
            for geom1 in self.render_geoms1:
                viewer.add_geom(geom1)
            for geom in self.render_geoms:
                viewer.add_geom(geom)
        results = []
        for i in range(len(self.viewers)):
            from multiagent import rendering
            # update bounds to center around agent
            cam_range = 1
            if self.shared_viewer:
                pos = np.zeros(self.world.dim_p)
            else:
                pos = self.agents[i].state.p_pos
            self.viewers[i].set_bounds(pos[0]-cam_range,pos[0]+cam_range,pos[1]-cam_range,pos[1]+cam_range)
            # update geometry positions
            wa = 0
            wa1 = 0
            for _,entity in enumerate(self.world.entities):
                if 'agent' in entity.name:
                    self.render_geoms_xform[wa].set_translation(*entity.state.p_pos)
                    wa = wa + 1
                else:
                    self.render_geoms_xform1[wa1].set_translation(*entity.state.p_pos)
                    wa1 = wa1 + 1

            for _, wall in enumerate(self.world.walls):
                self.render_geoms_xform1[wa1].set_translation(*wall.state.p_pos)
                wa1 = wa1 + 1
            # render to display or array
            results.append(self.viewers[i].render(return_rgb_array = mode=='rgb_array'))

        return results

    # create receptor field locations in local coordinate frame
    def _make_receptor_locations(self, agent):
        receptor_type = 'polar'
        range_min = 0.05 * 2.0
        range_max = 1.00
        dx = []
        # circular receptive field
        if receptor_type == 'polar':
            for angle in np.linspace(-np.pi, +np.pi, 8, endpoint=False):
                for distance in np.linspace(range_min, range_max, 3):
                    dx.append(distance * np.array([np.cos(angle), np.sin(angle)]))
            # add origin
            dx.append(np.array([0.0, 0.0]))
        # grid receptive field
        if receptor_type == 'grid':
            for x in np.linspace(-range_max, +range_max, 5):
                for y in np.linspace(-range_max, +range_max, 5):
                    dx.append(np.array([x,y]))
        return dx

    def shutdown(self):
        for viewer in self.viewers:
            viewer.close()

# vectorized wrapper for a batch of multi-agent environments
# assumes all environments have the same observation and action space
class BatchMultiAgentEnv(gym.Env):
    metadata = {
        'runtime.vectorized': True,
        'render.modes' : ['human', 'rgb_array']
    }

    def __init__(self, env_batch):
        self.env_batch = env_batch

    @property
    def n(self):
        return np.sum([env.n for env in self.env_batch])

    @property
    def action_space(self):
        return self.env_batch[0].action_space

    @property
    def observation_space(self):
        return self.env_batch[0].observation_space

    def step(self, action_n, time):
        obs_n = []
        reward_n = []
        done_n = []
        info_n = {'n': []}
        i = 0
        for env in self.env_batch:
            obs, reward, done, _ = env.step(action_n[i:(i+env.n)], time)
            i += env.n
            obs_n += obs
            # reward = [r / len(self.env_batch) for r in reward]
            reward_n += reward
            done_n += done
        return obs_n, reward_n, done_n, info_n

    def reset(self):
        obs_n = []
        for env in self.env_batch:
            obs_n += env.reset()
        return obs_n

    # render environment
    def render(self, mode='human', close=True):
        results_n = []
        for env in self.env_batch:
            results_n += env.render(mode, close)
        return results_n
