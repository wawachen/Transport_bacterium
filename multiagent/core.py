import numpy as np
import math 
import random
from collections import deque

# physical/external base state of all entites
class EntityState(object):
    def __init__(self):
        # physical position
        self.p_pos = None
        # physical velocity
        self.p_vel = None

# state of agents (including communication and internal/mental state)
class AgentState(EntityState):
    def __init__(self):
        super(AgentState, self).__init__()
        # communication utterance
       #self.heading = None # with the reference of y axis
        self.old_targetDis = None
        # communication utterance
        self.c = None

# action of the agent
class Action(object):
    def __init__(self):
        # moving action
        self.u = None
        # transportation action
        self.f = None
        # communication action
        self.c = None

# properties and state of physical world entity
class Entity(object):
    def __init__(self):
        # name 
        self.name = ''
        # properties:
        self.size = 0.010
        # entity can move / be pushed
        self.movable = False
        # entity collides with others
        self.collide = True
        # material density (affects mass)
        self.density = 25.0
        # color
        self.color = None
        # max speed and accel
        self.max_speed = None
        self.accel = None
        # state
        self.state = EntityState()
        # mass
        self.initial_mass = 1.0
         

    @property
    def mass(self):
        return self.initial_mass

# properties of landmark entities
class Landmark(Entity):
    def __init__(self):
        super(Landmark, self).__init__()
        self.size = 0.30
        self.initial_mass = 50

# properties of wall entities
class Wall(Entity):
    def __init__(self):
        super(Wall, self).__init__()
        self.initial_mass = 50
        self.orient = 0 # 0 is horizontal and 1 is vertical
        self.endpoints = np.zeros(2)
        self.length = 0.0
        self.width = 0.0

# properties of agent entities
class Agent(Entity):
    def __init__(self):
        super(Agent, self).__init__()
        # agents are movable by default
        self.movable = True
        # cannot send communication signals
        self.silent = False
        # cannot observe the world
        self.blind = False
        # physical motor noise amount
        self.u_noise = None
        # communication noise amount
        self.c_noise = None
        # control range
        self.u_range = 1.0
        # state
        self.state = AgentState()
        # action
        self.action = Action()
        # script behavior to execute
        self.action_callback = None
        #range of square view
        self.view_range = 0.1 
        self.role = 0 # The role of an agent: 0 is free moving mode, 1 is carrier mode

class flockRobot(Entity):
    def __init__(self, id):
        super(flockRobot, self).__init__()
        # robots are movable by default
        self.movable = True
        # cannot send communication signals
        self.silent = False
        # cannot observe the world
        self.blind = False
        # physical motor noise amount
        self.u_noise = None
        # communication noise amount
        self.c_noise = None
        # control range
        self.u_range = 1.0
        # state
        self.state = AgentState()
        # action
        self.action = Action()
        # script behavior to execute
        self.action_callback = None
        #range of square view
        self.view_range = 0.08 
        self.role = 0 # The role of an agent: 0 is free moving mode, 1 is carrier mode

        self.idNo = id
        self.xGoal = 0.0
        self.yGoal = 0.0
        self.myData = np.zeros(4) 
        self.dataOf2Neighbours = np.zeros(2)   #to store data for the 2 closest neighbours
        
        # For communications
        self.commsRadius = 0.08          #width of communication radius
        self.numberOfNeighbours = 5     #number of neighbours to see in comms radius
        self.dataOfNeighbours = np.zeros([self.numberOfNeighbours,4]) 
        
        #for the jackson model - Bacteria Controller 
        self.polluNCapacity = 4 
        self.polluNDTMemory = np.zeros(self.polluNCapacity) 
        self.polluNVTMemory = np.zeros(self.polluNCapacity) 

        self.arrayCounter = 0  
        self.expCalc = 0 
        self.t = 0 
        self.wDP_Dt  = 0 
        self.vWDP_Dt  = 0 
        self.tumbleLenghtCounter = 0 
        self.tumbleLenght = 1 
    
        self.previousReading = 0 
        #to store positions at border of pollutant.
        
        #   angle    #for random tumble angle
        self.cosAngle = 0.0  #for x direction #for random tumble angle
        #   cosAngle   #for x direction0.8 * self.bactXPos
        self.sinAngle = 0.0 #for y direction
        self.previousAngle = 0.0   #for forward motion
        #   sinAngle   #for y direction
        #   previousAngle   #for forward motion
        
        self.steerNAngle = 0.0
        self.steerNX = 0
        self.steerNY = 0
        self.varyNTumbleLength = 0.0
        
        self.newAngle = False   #to signify a new angle. 
        #   bactXPos 
        #   bactYPos 
        self.bactVelocity = 0
        self.flockXPos = 0.0
        self.flockYPos = 0.0		
        self.myXVelocity = random.uniform(-1,1)
        self.myYVelocity = random.uniform(-1,1)
		 
    def getCentreDensity(self,world):
        pollutionCount = 0
        # amplify it by 1000 times
        #print(agent.state.p_pos[0])
        xDPos = int(self.state.p_pos[0]/world.interval + world.xsize/2)
        yDPos = int(self.state.p_pos[1]/world.interval + world.ysize/2)

        for x in range(-50,50):
            for y in range(-50,50):
                tempX = xDPos + x
                tempY = yDPos + y

                if tempX < 0:
                    tempX = 0
                elif tempX > world.xsize:
                    tempX = world.xsize

                if tempY < 0:
                    tempY = 0
                elif tempY > world.ysize:
                    tempY = world.ysize

                pollutionCount = pollutionCount + world.pollution_map[tempX][tempY]
        
        return pollutionCount

    def collateData(self,world):
        self.myData[0] = self.idNo 
        self.myData[1] = self.state.p_pos[0] 
        self.myData[2] = self.state.p_pos[1] 
        self.myData[3] = self.getCentreDensity(world)
	
    def flockExp2(self, number):
        number =  (math.exp(-number + 40)) 
        number =  (0.5 * math.exp(-0.1 * number)) 

        return number 
			 
    def repulsiveForce(self, d, world):
        v = np.zeros(2) 
        r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 

        a = 100   #50
        b = 1 
        w = 2 
        e = 1 
         
        force = 0.0 
        temp = np.zeros(2) 	
        # repulsionTerm = self.getCentreDensity(world) / 148.0 

		#v/a = (a / pollutantSensor.getCentreDensity(myXPos, myYPos)) + 10 
		#force  = repulsionTerm * (Math.max(((a / ((b * r) - w)) - e),0)) 
        force  = np.max(((a / ((b * r) - w)) - e),0)
		#the greater this value the more the distance between agents.
		#so make sure it never goes to large value leading to scattering of agents in 
		#all directions. 
	
        force  = np.min([force, 0.1]) 
		
        if r == 0:
            r = 1 
		
        temp[0] = force * (d[0] /r) 
        temp[1] = force * (d[1] /r) 
		
        v[0] = -1 * temp[0] 
        v[1] = -1 * temp[1] 
		
        return v 
	
	
	 #uses centre of mass of swarm as attractive force
    def attractForce(self):
        v = np.zeros(2) 
        averageX = 0.0 
        averageY = 0.0 
        counterX = 0.0
        counterY = 0.0 
		#check array for values with zero. This got rid of the constant movement towards zero
		#get the value for the average divisor
		
        for x in range(5):
            if self.dataOfNeighbours[x][1] != 0:
                counterX = counterX + 1
            if self.dataOfNeighbours[x][2] != 0:
                counterY = counterY + 1 
		
		 #set to one if counterX and Y is zero.
        if counterX == 0:
            counterX = 1 
		
        if counterY == 0:
            counterY = 1 
		
        for x in range(5):
            averageX = self.dataOfNeighbours[x][1] + averageX 
            averageY = self.dataOfNeighbours[x][2] + averageY 

        averageX = averageX / counterX 
        averageY = averageY / counterY 
		
        averageX = ((averageX + self.state.p_pos[0]) / 2) 
        averageY = ((averageY + self.state.p_pos[1]) / 2) 
		
		 #System.out.println("averageX is:" + averageX) 
		 #System.out.println("averageY is:" + averageY) 
		
        v[0] = averageX 
        v[1] = averageY 
		
		# self.state.p_pos[0] = averageX 
		# self.state.p_pos[1] = averageY 
		
        return v 
	
	 # potential function using self propelled particles with softe core interaction:Patterns, stability and collaspe
    def separation(self,d):
        v = np.zeros(2) 
        r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 
        value = 0 
        dis = 100 
        k = 0.0003 
        temp = np.zeros(2) 

		 #value = repulsion - attraction 
        value = 5 * (1.4 * math.exp(-r / 20) - 1.39 * math.exp(-r / 20)) 
		 #value = flockExp2(r) 
		 #value = (k * (r - dis )* r)  
		 #temp[1] = (k * (r - dis )* (d[1])) 
		
        temp[0] = value * d[0]/r 
        temp[1] = value * d[1]/r 

		 #temp[0] = temp[0] * d[0]/r 
		 #temp[1] = temp[1] * d[1]/r 
 
        v[0]=- (temp[0]) 
        v[1]=- (temp[1]) 
        return v 
	
	 # match the velocity of 2 nearest neighbours
    def match_velocity(self, d):
        v = np.zeros(2) 
        v[0]=d[0]*.02 
        v[1]=d[1]*.02 
        v[0]+=d[2]*.02 
        v[1]+=d[3]*.02 
 		
        return v 
	
    def transmit(self, info, world):
	    #  #store data in a central location for access by all robots.
        id = int(info[0])    #get Id of robot
        world.robotInfo[id,0] = info[0];  #store Id of robot
        world.robotInfo[id,1] = info[1];  #store x position
        world.robotInfo[id,2] = info[2];  #store y position
        world.robotInfo[id,3] = info[3];  #store pollution reading.

    def run(self,world):
        self.collateData(world) 
		# myRadio.transmit(myData) 
        self.transmit(self.myData,world)
        self.receiveNeighbourData(world) 
        self.bacteriaController() 
		 #this.sineMotion() 
		 #presentReadN = myData[3] 
		 #this.boundaryFollowing2() 
        self.flockingController2(world) 
		 #this.steeringDirection2() 
		# self.combineBehaviours() 
		 #this.fuzzyController() 
	
    def check_bounds(self):	
	
        v = np.zeros(2) 

        if self.state.p_pos[0] < -1:
            v[0] = 0.001 
        elif self.state.p_pos[0] > 1:
            v[0] = -0.001 
		
        if self.state.p_pos[1] < -1:
            v[1] = 0.001 
        elif self.state.p_pos[1] > 1:
            v[1] = -0.001 

        return v 
	
	
    def receive2(self,world):
        relDistX = 0
        relDistY = 0
        newDist = 0
        tempDist = 1000000
		
		
		 #go through robot ids and find the nearest robot to me.
		 #store the x and y distance from me to the nearest robot.

        for id in range(len(world.agents)):
            if self.idNo != id:
                relDistX = world.robotInfo[id][1] - self.state.p_pos[0]
                relDistY = world.robotInfo[id][2] - self.state.p_pos[1]
                newDist =  math.sqrt(relDistX*relDistX + relDistY*relDistY)
			
				 #System.out.println("relDistX:" + robotInfo[id][1]);
				 #System.out.println("relDistY:" + robotInfo[id][2]);
			
                if newDist < tempDist:
                    tempDist = newDist
                    self.dataOf2Neighbours[0] = relDistX
                    self.dataOf2Neighbours[1] = relDistY
	
    def flockingController2(self,world):
        sepDist = np.zeros(2) 
        repulsiveDist = np.zeros(2)  
        attractDist = np.zeros(2) 
        checkBounds = np.zeros(2)  
        highestForagerPos = np.zeros(2) 
		
        self.receive2(world) 
        sepDist = self.separation(self.dataOf2Neighbours)   # separation force
        attractDist = self.attractForce() 
        repulsiveDist = self.repulsiveForce(self.dataOf2Neighbours, world) 
        checkBounds = self.check_bounds() 
        highestForagerPos = self.highestForager() 
		
        self.xGoal = 0.01*( (highestForagerPos[0]) - self.flockXPos)  #highestForagerPos[0] 
        self.yGoal = 0.01*( (highestForagerPos[1]) - self.flockYPos)  #highestForagerPos[1] 
		
        self.myXVelocity = (0.8 * self.myXVelocity + (1.0 * sepDist[0] + 0.0 *  (self.xGoal) + 0.0 * repulsiveDist[0]) + checkBounds[0]) 	 # Combine the rules
        self.myYVelocity = (0.8 * self.myYVelocity + (1.0 * sepDist[1] + 0.0 *  (self.yGoal) + 0.0 * repulsiveDist[1]) + checkBounds[1])   
        
        self.flockXPos = self.flockXPos + self.myXVelocity 
        self.flockYPos = self.flockYPos + self.myYVelocity  
		
        if np.abs(self.myXVelocity) > 0.01:
            self.myXVelocity = (self.myXVelocity/np.abs(self.myXVelocity))*0.01

        if np.abs(self.myYVelocity) > 0.01:
            self.myYVelocity = (self.myYVelocity/np.abs(self.myYVelocity))*0.01
		
	
    def receiveNeighbourData(self,world):
        neighbourData = np.zeros([self.numberOfNeighbours, 4])
        counter = 0
		
        for id in range(len(world.agents)):
            if self.idNo !=  (world.robotInfo[id,0]):
                if (world.robotInfo[id,1] < (self.state.p_pos[0] + self.commsRadius)) and (world.robotInfo[id,1] > (self.state.p_pos[0] - self.commsRadius)) and (world.robotInfo[id,2] < (self.state.p_pos[1] + self.commsRadius)) and (world.robotInfo[id,2] > (self.state.p_pos[1] - self.commsRadius)):
                     #store the x and y coordinates of robots within the comms radius
                    neighbourData[counter,0] = world.robotInfo[id,0]
                    neighbourData[counter,1] = world.robotInfo[id,1]
                    neighbourData[counter,2] = world.robotInfo[id,2]
                    neighbourData[counter,3] = world.robotInfo[id,3]
					
                    counter = counter + 1
						
					 #return the first encountered closest neigbhours
					 #not good way. Should first get all the robots
					 #within the comms radius and return the values of the nearest
					 #number of neighbours. will work on that later
					
                    if counter >= self.numberOfNeighbours:
                        counter = 0
                        break
		
        self.dataOfNeighbours = neighbourData 

	
    def highestForager(self):
	
		 #collect information about the robot wiht the highest reading and return its position
		
        highestPolluLevel = 0 
        highestForagerXPos = 0 
        highestForagerYPos = 0 
        highestForagerData = np.zeros(2)
		
        highestPolluLevel = self.dataOfNeighbours[0][3] 

        for x in range(1, self.numberOfNeighbours):
            if highestPolluLevel > self.dataOfNeighbours[x][3]:
                pass
            elif highestPolluLevel < self.dataOfNeighbours[x][3]:
                highestPolluLevel = self.dataOfNeighbours[x][3] 
                highestForagerXPos = self.dataOfNeighbours[x][1] 
                highestForagerYPos = self.dataOfNeighbours[x][2] 
				
		
        if self.myData[3] >= highestPolluLevel:
            highestForagerXPos = self.state.p_pos[0] 
            highestForagerYPos = self.state.p_pos[1] 
		
		 #highestForagerData[0] = highestPolluLevel 
        highestForagerData[0] = highestForagerXPos 
        highestForagerData[1] = highestForagerYPos 
		
        return highestForagerData 
	
	 #standard bacteria algorithm with dynamic velocity based upon reading. 
    def bacteriaController(self):
	
        moveX = 0 
        moveY = 0 
        tumblePosition = np.zeros(2)  
        newInfo = np.zeros(3)
		
        dP_Dt	= 0 
        kd = 2  #0.05   #2   3  30
        k1 = 0 
        k2 = 0 
        Tm = 1 
        stdRunLength = 0 
        alpha = 10  #0.5   #2
        stdTumbleLenght = 2  #30
        stdVelocity   = 3 

        vDP_Dt	= 0
        vKd = 2; #2   3
        vK1 = 0 
        vK2 = 0 
        vTm = 1 
        vStdRunLength = 0 
        vAlpha = 10 
        presentReading = self.myData[3] 
        presentReadNStore = 0 
		
        self.expCalc = self.arrayCounter 
		
		#calculate dP/dt 
		#*****************Checking ABS and not ABS ********** #
        dP_Dt = (kd / ((kd + presentReading) * (kd + presentReading)) * (presentReading - self.previousReading)) 	
        vDP_Dt = (vKd / ((vKd + presentReading) * (vKd + presentReading)) * (presentReading - self.previousReading)) 	
        presentReadNStore = presentReading 
		
        if presentReadNStore == 0:
            presentReadNStore = 1 
		
		
        vDP_Dt = (vKd / presentReadNStore)  # * (previousReading - presentReading) 
		
        self.polluNDTMemory[self.arrayCounter] = dP_Dt 
        self.polluNVTMemory[self.arrayCounter] = vDP_Dt 
		
        for x in range(self.polluNCapacity):
            self.t = self.t -1
            if self.t<0:
                self.t = self.polluNCapacity-1
            if self.expCalc < 0:
                self.expCalc = self.polluNCapacity - 1

            self.wDP_Dt = self.wDP_Dt + (self.polluNDTMemory[self.expCalc] * (math.exp((self.t - (self.polluNCapacity - 1)) / Tm))) 
            self.vWDP_Dt = self.vWDP_Dt + (self.polluNVTMemory[self.expCalc] * (math.exp((self.t - (self.polluNCapacity - 1)) / Tm))) 

            self.expCalc = self.expCalc - 1
		
        self.wDP_Dt =  (1 / Tm) * self.wDP_Dt 
        self.vWDP_Dt = (1 / Tm) * self.vWDP_Dt 
		
        if self.myData[3] == 0:
            self.varyNTumbleLength = self.varyNTumbleLength + 0.1
            if self.varyNTumbleLength > 20:
                self.varyNTumbleLength = 20 	
        elif self.myData[3] > 0:
            self.varyNTumbleLength = 0 
		
        self.tumbleLenght = (int(self.varyNTumbleLength) + stdTumbleLenght) * math.exp(alpha * self.wDP_Dt)   #increase rate of random direction
		
        self.bactVelocity = (stdVelocity * vDP_Dt)      #reduce velocity as your get closer to source and to stay in source
				
        if self.bactVelocity > 0.01:
            self.bactVelocity = 0.01
		
        self.tumbleRobot(self.tumbleLenght) 
				
        self.previousReading = presentReading 
        self.wDP_Dt = 0       #reset for subsequent calculations
        self.vWDP_Dt = 0 
	    
        self.arrayCounter =  self.arrayCounter + 1

        if self.arrayCounter >= self.polluNCapacity:
            self.arrayCounter = 0 
	
	
    def tumbleRobot(self, tumbleLen):
	
        position2 = np.zeros(2)
        self.tumbleLenghtCounter = self.tumbleLenghtCounter + 1
		
        if self.tumbleLenghtCounter > tumbleLen:
            self.tumbleLenghtCounter = 0 
				
            self.tumbleAngle() 	
            self.newAngle = True 
        else:
			#for tumble Angle
			# self.bactXPos = self.bactXPos + self.bactVelocity  * self.cosAngle 
			# self.bactYPos = self.bactYPos + self.bactVelocity  * self.sinAngle 
            self.newAngle = False 

    def tumbleAngle(self):
        
        # angle = np.random.random() * 360 
        # cosAngle = math.cos(math.radians(angle)) 
        # sinAngle = Math.sin(Math.radians(angle)) 
        presentReading = self.myData[3] 
        if presentReading == 0:
            presentReading = 1 

        direction = 0 

        if bool(random.getrandbits(1)):
            direction = 1 
        else:
            direction = -1 

        angle = (59.0 / 1.0) + (np.random.random() * 9.0) 
            #angle = 47 + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (47 / 1)) + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (60 / presentReading))  # + (g.nextDouble() * 9) 
        angle = angle + self.previousAngle 

        if angle > 360: #avoid angle from getting too large.
            self.steerNX = int(self.bactVelocity * math.cos(math.radians(self.steerNAngle))) 
            self.steerNY = int(self.bactVelocity * math.sin(math.radians(self.steerNAngle))) 
            angle = angle - 360 

        #		this.steeringDirection2() 
        self.cosAngle = math.cos(math.radians(angle)) 
        self.sinAngle = math.sin(math.radians(angle)) 
        self.previousAngle = angle 
    
		
	
	# def combineBehaviours(self):
	# 	self.myXPos = 0.0 * self.flockXPos + 0.8 * self.bactXPos + 0.0 * self.sineWaveX + 0.0 * self.steerNX 
	# 	self.myYPos = 0.0 * self.flockYPos + 0.8 * self.bactYPos + 0.0 * self.sineWaveY + 0.0 * self.steerNY 
class flockRobot1(Entity):
    def __init__(self,id):
        super(flockRobot1, self).__init__()
        # robots are movable by default
        self.movable = True
        # cannot send communication signals
        self.silent = False
        # cannot observe the world
        self.blind = False
        # physical motor noise amount
        self.u_noise = None
        # communication noise amount
        self.c_noise = None
        # control range
        self.u_range = 1.0
        # state
        self.state = AgentState()
        # action
        self.action = Action()
        # script behavior to execute
        self.action_callback = None
        #range of square view
        self.view_range = 0.08 
        self.role = 0 # The role of an agent: 0 is free moving mode, 1 is carrier mode

        self.idNo = id
        self.xGoal = 0.0
        self.yGoal = 0.0
        self.myData = np.zeros(4) 
        self.dataOf2Neighbours = np.zeros(2)   #to store data for the 2 closest neighbours
        
        # For communications
        self.commsRadius = 0.08          #width of communication radius
        self.numberOfNeighbours = 5     #number of neighbours to see in comms radius
        self.dataOfNeighbours = np.zeros([self.numberOfNeighbours,4]) 
        
        # self.previous_concentration = 0

        # self.memory_capacity = 10
        # self.p_rate_record = deque(maxlen = self.memory_capacity)

        # for i in range(self.memory_capacity):
        #     self.p_rate_record.append(0.0)

        # self.counter = 0
        # self.maximum_speed = 20 #40
        # self.T0 = 10.0

        #for the jackson model - Bacteria Controller 
        # self.polluNCapacity = 4
        # self.polluNDTMemory = np.zeros(self.polluNCapacity) 
        # self.polluNVTMemory = np.zeros(self.polluNCapacity) 

        # self.arrayCounter = 0  
        # self.expCalc = 0 
        # self.t = 0 
        # self.wDP_Dt  = 0 
        # self.vWDP_Dt  = 0 
        # self.tumbleLenghtCounter = 0 
        # self.tumbleLenght = 1 
    
        # self.previousReading = 0 
        #to store positions at border of pollutant.

        #new   
        self.polluNCapacity = 4
        self.polluNDTMemory = deque(maxlen = self.polluNCapacity)
        for i in range(self.polluNCapacity):
            self.polluNDTMemory.append(0.0)
        
        self.wDP_Dt  = 0 
        self.vWDP_Dt  = 0 
        self.tumbleLenghtCounter = 0 
        self.tumbleLenght = 1 
    
        self.previousReading = 0 
        #new
        
        # double angle    #for random tumble angle
        self.cosAngle = 0.0  #for x direction #for random tumble angle
        # double cosAngle   #for x direction
        self.sinAngle = 0.0 #for y direction
        self.previousAngle = 0.0  #for forward motion
        # double sinAngle   #for y direction
        # double previousAngle   #for forward motion

        self.steerNAngle = 0.0
        self.steerNX = 0
        self.steerNY = 0
        self.varyNTumbleLength = 0.0
        
        
        self.newAngle = False   #to signify a new angle. 
        # double bactXPos 
        # double bactYPos 
        self.bactVelocity = 0
        self.flockXPos = 0
        self.flockYPos = 0  
		
        self.myXVelocity = 0.0 #random.uniform(-0.01,0.01)
        self.myYVelocity = 0.0 #random.uniform(-0.01,0.01)

        self.last_vm = np.zeros([2,1])
      	
   
    def getCentreDensity(self,world):
        pollutionCount = 0
        # amplify it by 1000 times
        #print(agent.state.p_pos[0])
        xDPos = int(self.state.p_pos[0]/world.interval + world.xsize/2)
        yDPos = int(self.state.p_pos[1]/world.interval + world.ysize/2)

        for x in range(-50,50):
            for y in range(-50,50):
                tempX = xDPos + x
                tempY = yDPos + y

                if tempX < 0:
                    tempX = 0
                elif tempX > world.xsize:
                    tempX = world.xsize

                if tempY < 0:
                    tempY = 0
                elif tempY > world.ysize:
                    tempY = world.ysize

                pollutionCount = pollutionCount + world.pollution_map[tempX][tempY]
        
        return pollutionCount
	
    def collateData(self,world): 
		
		#assemble robot data for sending to others 
        self.myData[0] = self.idNo
        self.myData[1] = self.state.p_pos[0]
        self.myData[2] = self.state.p_pos[1] 
        self.myData[3] = self.getCentreDensity(world)
		# myData[4] = (double)batteryLevel 
		
		#System.out.println("pollutant level is" + pollutantSensor.getCentreDensity(x, y)) 
		#System.out.println("out here is" + myXPos) 
		#wSystem.out.println("myXPos:" + myXPos) 
	
    def flockExp2(self, number):
        number = (math.exp(-number + 40)) 
        number = (0.5 * math.exp(-0.1 * number)) 

        return number 
			 
	
    def repulsiveForce(self, d,world):
        v = np.zeros(2) 
        r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 
		
        a = 50   #50
        b = 1 
        w = 2 
        e = 1 
         
        force = 0 
        temp = np.zeros(2) 	
        #repulsionTerm = self.getCentreDensity() / 148.0 
		
		 #v/a = (a / pollutantSensor.getCentreDensity(myXPos, myYPos)) + 10 
		 #force  = repulsionTerm * (Math.max(((a / ((b * r) - w)) - e),0)) 
        force  = np.max(((a / ((b * r) - w)) - e),0)
		 #the greater this value the more the distance between agents.
		 #so make sure it never goes to large value leading to scattering of agents in 
		 #all directions. 
			
        force  = np.min([force, 0.1]) 
		
        if r == 0:
            r = 1 
		
        temp[0] = force * (d[0] /r) 
        temp[1] = force * (d[1] /r) 
		
        v[0] = -1 * temp[0] 
        v[1] = -1 * temp[1] 
		
        return v 
	
	
	 #uses centre of mass of swarm as attractive force
    def attractForce(self):	
        v = np.zeros(2) 
        averageX = 0.0 
        averageY = 0.0 
        counterX = 0.0
        counterY = 0.0 
		#check array for values with zero. This got rid of the constant movement towards zero
		#get the value for the average divisor
		
        for x in range(5):
            if self.dataOfNeighbours[x][1] != 0:
                counterX = counterX + 1
            if self.dataOfNeighbours[x][2] != 0:
                counterY = counterY + 1 
		
		 #set to one if counterX and Y is zero.
        if counterX == 0:
            counterX = 1 
		
        if counterY == 0:
            counterY = 1 
		
        for x in range(5):
            averageX = self.dataOfNeighbours[x][1] + averageX 
            averageY = self.dataOfNeighbours[x][2] + averageY 

        averageX = averageX / counterX 
        averageY = averageY / counterY 
		
        averageX = ((averageX + self.state.p_pos[0]) / 2) 
        averageY = ((averageY + self.state.p_pos[1]) / 2) 
		
		 #System.out.println("averageX is:" + averageX) 
		 #System.out.println("averageY is:" + averageY) 
		
        v[0] = averageX 
        v[1] = averageY 
		
		
        return v 
	
	
	 # potential function using self propelled particles with softe core interaction:Patterns, stability and collaspe
    def separation(self,d):

        v = np.zeros(2) 
        r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 
        value = 0 
        dis = 100 
        k = 0.0003 
        temp = np.zeros(2) 

            #value = repulsion - attraction 
        value = 10 * (1.4 * math.exp(-r / 20) - 1.39 * math.exp(-r / 20)) 
            #value = flockExp2(r) 
            #value = (k * (r - dis )* r)  
            #temp[1] = (k * (r - dis )* (d[1])) 
        
        temp[0] = value * d[0]/r 
        temp[1] = value * d[1]/r 

            #temp[0] = temp[0] * d[0]/r 
            #temp[1] = temp[1] * d[1]/r 
    #
        v[0]=-(temp[0]) 
        v[1]=-(temp[1]) 

        return v 
	
	 # match the velocity of 2 nearest neighbours
    def match_velocity(self, world):
        vm = np.zeros([2,1])
        counter = 0
        relative_v = np.zeros([2,1])

        for id in range(len(world.agents)):
            if self.idNo !=  (world.robotInfo[id,0]):
                if (world.robotInfo[id,1] < (self.state.p_pos[0] + self.commsRadius)) and (world.robotInfo[id,1] > (self.state.p_pos[0] - self.commsRadius)) and (world.robotInfo[id,2] < (self.state.p_pos[1] + self.commsRadius)) and (world.robotInfo[id,2] > (self.state.p_pos[1] - self.commsRadius)):
                     #store the x and y coordinates of robots within the comms radius
                    vm[0] = vm[0] + world.robotInfo[id,1] - self.state.p_pos[0]
                    vm[1] = vm[1] + world.robotInfo[id,2] - self.state.p_pos[1]
                  
                    counter = counter + 1
						
					 #return the first encountered closest neigbhours
					 #not good way. Should first get all the robots
					 #within the comms radius and return the values of the nearest
					 #number of neighbours. will work on that later
					
                    if counter >= self.numberOfNeighbours:
                        counter = 0
                        break
        
        relative_v[0] = (vm[0] - self.last_vm[0])/0.1
        relative_v[1] = (vm[1] - self.last_vm[1])/0.1

        self.last_vm[0] = vm[0]
        self.last_vm[1] = vm[1]

        v = np.zeros(2) 

        v[0]=relative_v[0]*.02 
        v[1]=relative_v[1]*.02 
        
        return v 
	
    def transmit(self,info,world):

        #  #store data in a central location for access by all robots.
        id = int(info[0])    #get Id of robot
        world.robotInfo[id,0] = info[0];  #store Id of robot
        world.robotInfo[id,1] = info[1];  #store x position
        world.robotInfo[id,2] = info[2];  #store y position
        world.robotInfo[id,3] = info[3];  #store pollution reading.

	
    def run(self, world):
        self.collateData(world) 
        # myRadio.transmit(myData) 
        self.transmit(self.myData,world)
        self.receiveNeighbourData(world) 
        self.bacteriaController00() 
            #this.sineMotion() 
            #presentReadN = myData[3] 
            #this.boundaryFollowing2() 
        self.flockingController2(world) 
            #this.steeringDirection2() 
        # self.combineBehaviours() 
            #this.fuzzyController() 
	

    def check_bounds(self):	

        v = np.zeros(2) 

        if self.state.p_pos[0] < -1:
            v[0] = 0.001 
        elif self.state.p_pos[0] > 1:
            v[0] = -0.001 
		
        if self.state.p_pos[1] < -1:
            v[1] = 0.001 
        elif self.state.p_pos[1] > 1:
            v[1] = -0.001 

        return v 
	
	
    def receive2(self,world):
        relDistX = 0
        relDistY = 0
        newDist = 0
        tempDist = 1000000
		
		
		 #go through robot ids and find the nearest robot to me.
		 #store the x and y distance from me to the nearest robot.

        for id in range(len(world.agents)):
            if self.idNo != id:
                relDistX = world.robotInfo[id][1] - self.state.p_pos[0]
                relDistY = world.robotInfo[id][2] - self.state.p_pos[1]
                newDist =  math.sqrt(relDistX*relDistX + relDistY*relDistY)
			
				 #System.out.println("relDistX:" + robotInfo[id][1]);
				 #System.out.println("relDistY:" + robotInfo[id][2]);
			
                if newDist < tempDist:
                    tempDist = newDist
                    self.dataOf2Neighbours[0] = relDistX
                    self.dataOf2Neighbours[1] = relDistY

	
    def flockingController2(self,world):
        sepDist = np.zeros(2) 
        repulsiveDist = np.zeros(2)  
        attractDist = np.zeros(2) 
        checkBounds = np.zeros(2)  
        highestForagerPos = np.zeros(2) 
        mVel = np.zeros(2)
        
        self.receive2(world) 
        sepDist = self.separation(self.dataOf2Neighbours)   # separation force
        attractDist = self.attractForce() 
        repulsiveDist = self.repulsiveForce(self.dataOf2Neighbours,world) 
        checkBounds = self.check_bounds() 
        highestForagerPos = self.highestForager() 
        mVel = self.match_velocity(world)
        
        self.xGoal = 0.01*( (highestForagerPos[0]) - self.flockXPos)  #highestForagerPos[0] 
        self.yGoal = 0.01*( (highestForagerPos[1]) - self.flockYPos)  #highestForagerPos[1] 
        
        # self.myXVelocity = (0.8 * self.myXVelocity + (0.5 * sepDist[0] + 0.0 *  (self.xGoal) + 0.0 * repulsiveDist[0]) + checkBounds[0]) 	 # Combine the rules
        # self.myYVelocity = (0.8 * self.myYVelocity + (0.5 * sepDist[1] + 0.0 *  (self.yGoal) + 0.0 * repulsiveDist[1]) + checkBounds[1]) 
        self.myXVelocity = (0.5 * sepDist[0]  + 0.1 * attractDist[0] + 0.1*mVel[0]) + checkBounds[0] 	 # Combine the rules
        self.myYVelocity = (0.5 * sepDist[1]  + 0.1 * attractDist[1] + 0.1*mVel[1]) + checkBounds[1]

        self.flockXPos = self.flockXPos + self.myXVelocity* 0.1
        self.flockYPos = self.flockYPos + self.myYVelocity* 0.1
        
        if np.abs(self.myXVelocity) > 0.01:
            self.myXVelocity = (self.myXVelocity/np.abs(self.myXVelocity))*0.01

        if np.abs(self.myYVelocity) > 0.01:
            self.myYVelocity = (self.myYVelocity/np.abs(self.myYVelocity))*0.01
        
	
    def receiveNeighbourData(self,world):
        neighbourData = np.zeros([self.numberOfNeighbours, 4])
        counter = 0
		
        for id in range(len(world.agents)):
            if self.idNo !=  (world.robotInfo[id,0]):
                if (world.robotInfo[id,1] < (self.state.p_pos[0] + self.commsRadius)) and (world.robotInfo[id,1] > (self.state.p_pos[0] - self.commsRadius)) and (world.robotInfo[id,2] < (self.state.p_pos[1] + self.commsRadius)) and (world.robotInfo[id,2] > (self.state.p_pos[1] - self.commsRadius)):
                     #store the x and y coordinates of robots within the comms radius
                    neighbourData[counter,0] = world.robotInfo[id,0]
                    neighbourData[counter,1] = world.robotInfo[id,1]
                    neighbourData[counter,2] = world.robotInfo[id,2]
                    neighbourData[counter,3] = world.robotInfo[id,3]
					
                    counter = counter + 1
						
					 #return the first encountered closest neigbhours
					 #not good way. Should first get all the robots
					 #within the comms radius and return the values of the nearest
					 #number of neighbours. will work on that later
					
                    if counter >= self.numberOfNeighbours:
                        counter = 0
                        break
		
        self.dataOfNeighbours = neighbourData 


    def highestForager(self):

        #collect information about the robot wiht the highest reading and return its position
		
        highestPolluLevel = 0 
        highestForagerXPos = 0 
        highestForagerYPos = 0 
        highestForagerData = np.zeros(2)
		
        highestPolluLevel = self.dataOfNeighbours[0][3] 

        for x in range(1, self.numberOfNeighbours):
            if highestPolluLevel > self.dataOfNeighbours[x][3]:
                pass
            elif highestPolluLevel < self.dataOfNeighbours[x][3]:
                highestPolluLevel = self.dataOfNeighbours[x][3] 
                highestForagerXPos = self.dataOfNeighbours[x][1] 
                highestForagerYPos = self.dataOfNeighbours[x][2] 
				
		
        if self.myData[3] >= highestPolluLevel:
            highestForagerXPos = self.state.p_pos[0] 
            highestForagerYPos = self.state.p_pos[1] 
		
		 #highestForagerData[0] = highestPolluLevel 
        highestForagerData[0] = highestForagerXPos 
        highestForagerData[1] = highestForagerYPos 
		
        return highestForagerData 
	
    def bacteriaControllernew(self):
        T = 0
        # T0 = 20
        Tm = 1
        alpha = 1 #1e5 #300 6000
        kd = 30 #30
        dP_dt = 0
        
        dP_dt_weight = 0

        self.counter+=1
        current_concentration = self.myData[3]
        # print(current_concentration)
        dC_dt = current_concentration - self.previous_concentration
       
        dP_dt = kd/((kd + current_concentration)*(kd + current_concentration)) * dC_dt
  
        self.p_rate_record.append(dP_dt)

        # for i in range(self.memory_capacity):
        #     if i==0:
        #         dP_dt_weight = dP_dt_weight + self.p_rate_record[i] * np.exp(i/Tm)
        #     else:
        #         dP_dt_weight = dP_dt_weight + self.p_rate_record[i] * np.exp((i-9)/Tm)
        for i in range(self.memory_capacity):
            dP_dt_weight = dP_dt_weight + self.p_rate_record[i] * np.exp((i-9)/Tm) 

        if current_concentration == 0:
            current_concentration = 1

        # self.bactVelocity = 2/(current_concentration*0.0001 + 1)
        self.bactVelocity = self.maximum_speed/(current_concentration)
        # print(self.bactVelocity)
        if self.bactVelocity > 0.01:
            self.bactVelocity = 0.01

        # print(self.bactVelocity)
        
        dP_dt_weight = dP_dt_weight/Tm
        # print(dP_dt_weight)

        if current_concentration == 1:
            self.T0 = self.T0 + 1
            if self.T0 > 60:
                self.T0 = 60	
        elif current_concentration > 0:
            self.T0 = 0 
        # if current_concentration == 1:
        #     pass
        # else:
        #     self.T0 = 60.0/30

        T = (self.T0+2) * np.exp(alpha * dP_dt_weight)
        # print(T)
        
        self.previous_concentration = current_concentration

        if self.counter > T:
            self.tumbleAngle()
            self.counter = 0
            # self.bactVelocity = 0.0


    def bacteriaController(self):
	
        moveX = 0 
        moveY = 0 
        tumblePosition = np.zeros(2) 
        newInfo = np.zeros(3)
		
        dP_Dt	= 0 
        kd = 2  #0.05   #2   3
        k1 = 0 
        k2 = 0 
        Tm = 1 
        stdRunLength = 0 
        alpha = 10  #0.5   #2
        stdTumbleLenght = 2  #30
        stdVelocity   = 3 

        vDP_Dt	= 0
        vKd = 2; #2   3
        vK1 = 0 
        vK2 = 0 
        vTm = 1 
        vStdRunLength = 0 
        vAlpha = 10 
        presentReading = self.myData[3] 
        presentReadNStore = 0 
		
        self.expCalc = self.arrayCounter 
		
		#calculate dP/dt 
		#*****************Checking ABS and not ABS ********** #
        dP_Dt = (kd / ((kd + presentReading) * (kd + presentReading)) * (presentReading - self.previousReading)) 	
       	
        presentReadNStore = presentReading 
		
        if presentReadNStore == 0:
            presentReadNStore = 1 
		
        self.polluNDTMemory[self.arrayCounter] = dP_Dt 
		
        for x in range(self.polluNCapacity):
            self.t = self.t -1
            if self.t<0:
                self.t = self.polluNCapacity-1
            if self.expCalc < 0:
                self.expCalc = self.polluNCapacity - 1

            self.wDP_Dt = self.wDP_Dt + (self.polluNDTMemory[self.expCalc] * (math.exp((self.t - (self.polluNCapacity - 1)) / Tm))) 
            # print(self.polluNDTMemory[self.expCalc])
            self.expCalc = self.expCalc - 1
		
        self.wDP_Dt =  (1 / Tm) * self.wDP_Dt 
        print(self.wDP_Dt)
		
        if self.myData[3] == 0:
            self.varyNTumbleLength = self.varyNTumbleLength + 0.1
            if self.varyNTumbleLength > 20:
                self.varyNTumbleLength = 20 	
        elif self.myData[3] > 0:
            self.varyNTumbleLength = 0 
		
        self.tumbleLenght = (int(self.varyNTumbleLength) + stdTumbleLenght) * math.exp(alpha * self.wDP_Dt)   #increase rate of random direction
        # print(math.exp(alpha * self.wDP_Dt))		
        stdVelocity = 64  
        vDP_Dt = (vKd / (presentReadNStore))  # * (previousReading - presentReading) 
        # print(vDP_Dt)
        self.bactVelocity = (stdVelocity * vDP_Dt)      #reduce velocity as your get closer to source and to stay in source
        print(self.bactVelocity)		
        if self.bactVelocity > 0.01:
            self.bactVelocity = 0.01
		
        # print(self.tumbleLenght)
        self.tumbleRobot1(self.tumbleLenght) 
				
        self.previousReading = presentReading 
        self.wDP_Dt = 0       #reset for subsequent calculations
	    
        self.arrayCounter =  self.arrayCounter + 1

        if self.arrayCounter >= self.polluNCapacity:
            self.arrayCounter = 0 
	
    def bacteriaController00(self):
	
        dP_Dt	= 0 
        kd = 2  #0.05   #2   3
        Tm = 1 
        alpha = 10  #0.5   #2
        stdTumbleLenght = 2  #30
        stdVelocity   = 3 

        vDP_Dt	= 0
        vKd = 2; #2   3 
        presentReading = self.myData[3] 
        presentReadNStore = 0 
		
		#calculate dP/dt 
		#*****************Checking ABS and not ABS ********** #
        dP_Dt = (kd / ((kd + presentReading) * (kd + presentReading)) * (presentReading - self.previousReading)) 	
       	
        presentReadNStore = presentReading 
		
        if presentReadNStore == 0:
            presentReadNStore = 1 

        self.polluNDTMemory.append(dP_Dt)

        for i in range(self.polluNCapacity):
            self.wDP_Dt = self.wDP_Dt + self.polluNDTMemory[i] * np.exp((i-3)/Tm)
		
        self.wDP_Dt =  (1 / Tm) * self.wDP_Dt 
        # print(self.wDP_Dt)
		
        if self.myData[3] == 0:
            self.varyNTumbleLength = self.varyNTumbleLength + 0.05
            if self.varyNTumbleLength > 60:
                self.varyNTumbleLength = 60 	
        elif self.myData[3] > 0:
            self.varyNTumbleLength = 0 
		
        self.tumbleLenght = (int(self.varyNTumbleLength) + stdTumbleLenght) * math.exp(alpha * self.wDP_Dt)   #increase rate of random direction
        # print(math.exp(alpha * self.wDP_Dt))		
        stdVelocity = 64  
        vDP_Dt = (vKd / (presentReadNStore))  # * (previousReading - presentReading) 
        # print(vDP_Dt)
        self.bactVelocity = (stdVelocity * vDP_Dt)      #reduce velocity as your get closer to source and to stay in source
        # print(self.bactVelocity)		
        if self.bactVelocity > 0.01:
            self.bactVelocity = 0.01
		
        # print(self.tumbleLenght)
        self.tumbleRobot1(self.tumbleLenght) 
				
        self.previousReading = presentReading 
        self.wDP_Dt = 0       #reset for subsequent calculations
	    
	

    def bacteriaController7(self):

        moveX = 0
        moveY = 0
        newInfo = np.zeros(3)
        
        
        dP_Dt	= 0
        kd = 2
        k1 = 0
        k2 = 0
        Tm = 1
        stdRunLength = 0
        alpha = 2
        stdTumbleLenght = 2
        stdVelocity   = 3
        
        vDP_Dt	= 0
        vKd = 2
        vK1 = 0
        vK2 = 0
        vTm = 1
        vStdRunLength = 0
        vAlpha = 10
        presentReading = self.myData[3]
        presentReadNStore = 0
        
        self.expCalc = self.arrayCounter
        
        
        #calculate dP/dt 
        #*****************Checking ABS and not ABS #**********//
        dP_Dt = (kd / ((kd + presentReading) * (kd + presentReading)) * (presentReading - self.previousReading))	
        #vDP_Dt = (vKd / ((vKd + presentReading) * (vKd + presentReading)) * (presentReading - previousReading));	
        presentReadNStore = presentReading
        
        
        if presentReadNStore == 0:
            presentReadNStore = 1
        
        gradient = presentReading - self.previousReading
        
        if gradient == 0:
            gradient = 1
        
        if presentReading == 0:
            presentReadNStore = 1
        
        vDP_Dt =  gradient
        
        self.polluNDTMemory[self.arrayCounter] = dP_Dt
        self.polluNVTMemory[self.arrayCounter] = vDP_Dt
        
        for x in range(self.polluNCapacity):
            self.t = self.t -1
            if self.t<0:
                self.t = self.polluNCapacity-1
            if self.expCalc < 0:
                self.expCalc = self.polluNCapacity - 1

            self.wDP_Dt = self.wDP_Dt + (self.polluNDTMemory[self.expCalc] * (math.exp((self.t - (self.polluNCapacity - 1)) / Tm))) 
            self.vWDP_Dt = self.vWDP_Dt + (self.polluNVTMemory[self.expCalc] * (math.exp((self.t - (self.polluNCapacity - 1)) / Tm))) 

            self.expCalc = self.expCalc - 1
        
        self.wDP_Dt =  (1 / Tm) * self.wDP_Dt
        self.vWDP_Dt = (1 / Tm) * self.vWDP_Dt
        
                
        self.tumbleLenght = stdTumbleLenght * math.exp(alpha * self.wDP_Dt) #increase rate of random direction
        
        self.vWDP_Dt = self.vWDP_Dt * math.exp(-presentReading) * 100  #0.05

        if presentReading < 1:
            self.bactVelocity = 0.005
        else:
            self.bactVelocity = (self.vWDP_Dt * 1) + 1  #use sigmoid function here..
        
        if self.bactVelocity > 0.01:
            self.bactVelocity = 0.01
        elif self.bactVelocity < -0.01:
            self.bactVelocity = -0.01
        
        self.tumbleRobot(self.tumbleLenght) 
                
        self.previousReading = presentReading 
        
        self.wDP_Dt = 0       #reset for subsequent calculations
        self.vWDP_Dt = 0 
        
        self.arrayCounter =  self.arrayCounter + 1

        if self.arrayCounter >= self.polluNCapacity:
            self.arrayCounter = 0 
        
	
    def tumbleRobot(self, tumbleLen):

        # position2 = np.zeros(2)
        self.tumbleLenghtCounter = self.tumbleLenghtCounter + 1
        
        if self.tumbleLenghtCounter > tumbleLen:
            self.tumbleLenghtCounter = 0 
                
            self.tumbleAngle() 	
            self.newAngle = True 
        else:
                #for tumble Angle
            self.newAngle = False 
            
	
    def tumbleAngle(self):
        
        angle = np.random.random() * 360 
        # cosAngle = math.cos(math.radians(angle)) 
        # sinAngle = Math.sin(Math.radians(angle)) 
        
    #		this.steeringDirection2() 
        self.cosAngle = math.cos(math.radians(angle)) 
        self.sinAngle = math.sin(math.radians(angle)) 
		
    def tumbleRobot1(self, tumbleLen):

        position2 = np.zeros(2)
        self.tumbleLenghtCounter = self.tumbleLenghtCounter + 1
        
        if self.tumbleLenghtCounter > tumbleLen:
            self.tumbleLenghtCounter = 0 
                
            self.tumbleAngle1() 	
            self.newAngle = True 
            self.bactVelocity = 0.0
        else:
            #for tumble Angle
            # self.bactXPos = self.bactXPos + self.bactVelocity  * self.cosAngle 
            # self.bactYPos = self.bactYPos + self.bactVelocity  * self.sinAngle 
            self.newAngle = False 		
	
    def tumbleAngle1(self):
        
        # angle = np.random.random() * 360 
        # cosAngle = math.cos(math.radians(angle)) 
        # sinAngle = Math.sin(Math.radians(angle)) 
        presentReading = self.myData[3] 
        if presentReading == 0:
            presentReading = 1 
        
        direction = 0 
        
        if bool(random.getrandbits(1)):
            direction = 1 
        else:
            direction = -1 
        
        angle = (59.0 / 1.0) + (np.random.random() * 9.0) 
            #angle = 47 + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (47 / 1)) + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (60 / presentReading))  # + (g.nextDouble() * 9) 
        angle = angle + self.previousAngle 
        
        if angle > 360: #avoid angle from getting too large.
            self.steerNX = int(self.bactVelocity * math.cos(math.radians(self.steerNAngle))) 
            self.steerNY = int(self.bactVelocity * math.sin(math.radians(self.steerNAngle))) 
            angle = angle - 360 
        
    #		this.steeringDirection2() 
        self.cosAngle = math.cos(math.radians(angle)) 
        self.sinAngle = math.sin(math.radians(angle)) 
        self.previousAngle = angle 

    def tumbleAnglenew(self):
        
        act_angle = (np.random.uniform(-1,1) * 20.0) 
            #angle = 47 + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (47 / 1)) + (g.nextDouble() * 9) 
            #angle = (g.nextDouble() * (60 / presentReading))  # + (g.nextDouble() * 9) 
        angle = act_angle + self.previousAngle 
        
        if angle > 360: #avoid angle from getting too large.
            self.steerNX = int(self.bactVelocity * math.cos(math.radians(self.steerNAngle))) 
            self.steerNY = int(self.bactVelocity * math.sin(math.radians(self.steerNAngle))) 
            angle = angle - 360 
        
    #		this.steeringDirection2() 
        self.cosAngle = math.cos(math.radians(angle)) 
        self.sinAngle = math.sin(math.radians(angle)) 
        self.previousAngle = angle 
    
    # def combineBehaviours(self):
    #     self.myXPos = 0.8 * self.flockXPos + 0.8 * self.bactXPos
    #     self.myYPos = 0.8 * self.flockYPos + 0.8 * self.bactYPos
        
	

# multi-agent world
class World(object):
    def __init__(self):
        # list of agents and entities (can change at execution-time!)
        self.agents = []
        self.landmarks = []
        self.walls = []
        #self.density_points = [] #0(vacant), 1(circle), 2(circle), 3(circle)
        # force action dimensionality
        self.p_shape = None
        self.dim_f = 2
        # position dimensionality
        self.dim_p = 2
        # color dimensionality
        self.dim_color = 3
        # simulation timestep
        self.dt = 0.1
        self.timestep = 0
        # physical damping
        self.damping = 0.25
        # contact response parameters
        self.contact_force = 1e+2
        self.contact_margin = 1e-3
        # for pollution detection
        self.interval = 0.001
        self.xsize = int(2/self.interval)
        self.ysize = int(2/self.interval)
        self.pollution_map = np.zeros([self.xsize+1,self.ysize+1]) 
        # for collaborative transportation
        self.contact_list = []
        self.mode = 0   #0 is free moving mode, 1 is carrier mode
        self.new = 0
        self.torus = 0
        self.robotInfo = None
        self.neighbourData = None
    # return all entities in the world
    @property
    def entities(self):
        return self.agents + self.landmarks

    # return all agents controllable by external policies
    @property
    def policy_agents(self):
        return [agent for agent in self.agents if agent.action_callback is None]

    # return all agents controlled by world scripts
    @property
    def scripted_agents(self):
        return [agent for agent in self.agents if agent.action_callback is not None]

    def getCentreDensity(self, agent):
        pollutionCount = 0
        # amplify it by 1000 times
        #print(agent.state.p_pos[0])
        xDPos = int(agent.state.p_pos[0]/self.interval + self.xsize/2)
        yDPos = int(agent.state.p_pos[1]/self.interval + self.ysize/2)

        for x in range(-50,50):
            for y in range(-50,50):
                tempX = xDPos + x
                tempY = yDPos + y

                if tempX < 0:
                    tempX = 0
                elif tempX > self.xsize:
                    tempX = self.xsize

                if tempY < 0:
                    tempY = 0
                elif tempY > self.ysize:
                    tempY = self.ysize

                pollutionCount = pollutionCount + self.pollution_map[tempX][tempY]
        
        return pollutionCount


    # update state of the world
    def step(self):
        self.timestep += 1
        if self.mode == 1:
            self.get_contact_points()
        # set actions for scripted agents 
        # for agent in self.scripted_agents:
        #     agent.action = agent.action_callback(agent, self)
        #     print(1)
        # # gather forces applied to entities
        # p_force = [None] * len(self.entities)

        p_force = [None] * len(self.entities)
        #print(len(self.entities))
        p_vel = [None] * len(self.entities)
        # # apply agent physical controls
        # p_force = self.apply_action_force(p_force)
        # apply agent physical controls

        p_vel = self.apply_action_velocity(p_vel)
        # apply environment forces
        p_force = self.apply_environment_force(p_force)
        # integrate physical state
        self.integrate_state(p_force, p_vel)
        # update the color of agents
        self.check_roles()
        # update agent state
        # for agent in self.agents:
        #     self.update_agent_state(agent)

    # # gather agent action forces
    # def apply_action_force(self, p_force):
    #     # set applied forces
    #     for i,agent in enumerate(self.agents):
    #         if agent.movable:
    #             noise = np.random.randn(*agent.action.u.shape) * agent.u_noise if agent.u_noise else 0.0
    #             p_force[i] = agent.action.u + noise                
    #     return p_force

    # check roles for each agent
    def check_roles(self):
        for agent in self.agents:
            if agent.role == 0:
                agent.color = np.array([2.5,0,0])
            else:
                agent.color = np.array([2.5,2.5,0])

    #gather agent action velocity
    def apply_action_velocity(self, p_vel):
        for i, entity in enumerate(self.entities):
            if 'agent' in entity.name:
                if entity.movable:
                    noise = np.random.randn(*entity.action.u.shape) * entity.u_noise if entity.u_noise else 0.0
                    p_vel[i] = entity.action.u + noise 
           
        return p_vel

    # gather physical forces acting on entities
    def apply_environment_force(self, p_force):
        # simple (but inefficient) collision response
        
        for a,entity_a in enumerate(self.entities):
            for b,entity_b in enumerate(self.entities):
                if(b <= a): continue
                [f_a, f_b] = self.get_collision_force(entity_a, entity_b)

                if(f_a is not None):
                    if(p_force[a] is None): p_force[a] = 0.0 
                    p_force[a] = f_a + p_force[a]
                 
                if(f_b is not None):
                    if(p_force[b] is None): p_force[b] = 0.0
                    p_force[b] = f_b + p_force[b]    

            if entity_a.movable:
                for wall in self.walls:
                    wf = self.get_wall_collision_force(entity_a, wall)
                    if wf is not None:
                        if p_force[a] is None:
                            p_force[a] = 0.0
                        p_force[a] = p_force[a] + wf
        return p_force

    # integrate physical state
    def integrate_state(self, p_force, p_vel):
        for i,entity in enumerate(self.entities):
            if 'agent' in entity.name:
                if not entity.movable: continue
                entity.state.p_vel = entity.state.p_vel * (1 - self.damping)
                
                if (p_force[i] is not None):
                    entity.state.p_vel = entity.state.p_vel + (p_force[i] / entity.mass) * self.dt 
                    
                if (p_vel[i] is not None):
                    entity.state.p_vel = entity.state.p_vel + p_vel[i]
                    # print(entity.state.p_vel,p_vel[i])
                
                if entity.max_speed is not None:
                    speed = np.sqrt(np.square(entity.state.p_vel[0]) + np.square(entity.state.p_vel[1]))
                    if speed > entity.max_speed:
                        entity.state.p_vel = entity.state.p_vel / np.sqrt(np.square(entity.state.p_vel[0]) +
                                                                    np.square(entity.state.p_vel[1])) * entity.max_speed
            else:

                if not entity.movable: continue
                entity.state.p_vel = entity.state.p_vel * (1 - self.damping)
                
                if(p_force[i] is None): p_force[i] = 0.0
                for j,agent in enumerate(self.agents):
                    if agent.role == 1:
                        f_p = self.get_payload_force(agent, j)  
                    else: 
                        f_p = 0.0
                    p_force[i] = f_p + p_force[i]
                entity.state.p_vel = entity.state.p_vel + (p_force[i] / entity.mass) * self.dt 

            entity.state.p_pos += entity.state.p_vel * self.dt
            # if 'agent' in entity.name:
            #     entity.state.heading = math.atan2(entity.state.p_vel[0], entity.state.p_vel[1])

            # # closed feild, the agent exceeds the boundary will appear on the opposite side
            # if self.torus:
            #     next_coord = np.where(entity.state.p_pos < -1.0, entity.state.p_pos + 2.0, entity.state.p_pos)
            #     next_coord = np.where(next_coord > 1.0, next_coord - 2.0, next_coord)
            #     #print(next_coord)
            # else:
            #     next_coord = np.where(entity.state.p_pos < -1.0, [-1.0,-1.0], entity.state.p_pos)
            #     next_coord = np.where(next_coord > 1.0, [1.0,1.0], next_coord)
            # entity.state.p_pos = next_coord
        

    def update_agent_state(self, agent):
        # set communication state (directly for now)
        if agent.silent:
            agent.state.c = np.zeros(self.dim_c)
        else:
            noise = np.random.randn(*agent.action.c.shape) * agent.c_noise if agent.c_noise else 0.0
            agent.state.c = agent.action.c + noise      

    # get collision forces for any contact between two entities
    def get_collision_force(self, entity_a, entity_b):
        if (not entity_a.collide) or (not entity_b.collide):
            return [None, None] # not a collider
        if (entity_a is entity_b):
            return [None, None] # don't collide against itself

        # compute actual distance between entities
        delta_pos = entity_a.state.p_pos - entity_b.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        # softmax penetration
        k = self.contact_margin
        # minimum allowable distance
        dist_min = entity_a.size + entity_b.size
        penetration = np.logaddexp(0, -(dist - dist_min)/k)*k                           
    
        #print(self.contact_force * delta_pos / dist * penetration)
        force = self.contact_force * delta_pos / dist * penetration

        force_a = +force if entity_a.movable else None
        force_b = -force if entity_b.movable else None
        
        return [force_a, force_b]
    
    # get collision forces for contact between an entity and a wall
    def get_wall_collision_force(self, entity, wall):
        # if entity.ghost and not wall.hard:
        #     return None  # ghost passes through soft walls
        if wall.orient == 0:
            prll_dim = 0
            perp_dim = 1
        else:
            prll_dim = 1
            perp_dim = 0

        ent_pos = entity.state.p_pos
        if (ent_pos[prll_dim] < wall.endpoints[0] - entity.size or
            ent_pos[prll_dim] > wall.endpoints[1] + entity.size):
            return None  # entity is beyond endpoints of wall
        elif (ent_pos[prll_dim] < wall.endpoints[0] or
              ent_pos[prll_dim] > wall.endpoints[1]):
            # part of entity is beyond wall
            if ent_pos[prll_dim] < wall.endpoints[0]:
                dist_past_end = ent_pos[prll_dim] - wall.endpoints[0]
            else:
                dist_past_end = ent_pos[prll_dim] - wall.endpoints[1]
            theta = np.arcsin(dist_past_end / entity.size)
            if wall.orient == 0:
                dist_min = np.cos(theta) * entity.size + 0.5 * wall.width
            else: 
                dist_min = np.cos(theta) * entity.size + 0.5 * wall.length    
        else:  # entire entity lies within bounds of wall
            theta = 0
            dist_past_end = 0
            if wall.orient == 0:
                dist_min = entity.size + 0.5 * wall.width
            else: 
                dist_min = entity.size + 0.5 * wall.length  
            
        
        # only need to calculate distance in relevant dim
        delta_pos = ent_pos[perp_dim] - wall.state.p_pos[perp_dim]
        dist = np.abs(delta_pos)
        # softmax penetration
        k = self.contact_margin
        penetration = np.logaddexp(0, -(dist - dist_min)/k)*k
        force_mag = self.contact_force * delta_pos / dist * penetration
        force = np.zeros(2)
        force[perp_dim] = np.cos(theta) * force_mag
        force[prll_dim] = np.sin(theta) * np.abs(force_mag)
        return force

     # get collaborative force from each agent to update the motion of the payload (mimic the aerial transportation)
    def get_contact_points(self):
        if self.p_shape == "circle_nonuni":
            for i,agent in enumerate(self.agents):
                if np.sqrt(np.sum(np.square(agent.state.p_pos-self.landmarks[0].state.p_pos))) < 0.3:
                    agent.role = 1
                    self.contact_list.append(agent.state.p_pos-self.landmarks[0].state.p_pos) #relative position  
                else:
                    self.contact_list.append(250)
        if self.p_shape == "square_uni":
            for i,agent in enumerate(self.agents):
                if agent.state.p_pos[0] > -0.3 and  agent.state.p_pos[0] < 0.3 and agent.state.p_pos[1] > -0.2 and agent.state.p_pos[1] < 0.2 :
                    agent.role = 1
                    self.contact_list.append(agent.state.p_pos-self.landmarks[0].state.p_pos) #relative position  
                else:
                    self.contact_list.append(250)
        if self.p_shape == "peanut_uni":
            for i,agent in enumerate(self.agents):
                if agent.state.p_pos[0] > -0.4 and  agent.state.p_pos[0] < 0.4 and agent.state.p_pos[1] > -0.2 and agent.state.p_pos[1] < 0.2 :
                    agent.role = 1
                    self.contact_list.append(agent.state.p_pos-self.landmarks[0].state.p_pos) #relative position  
                else:
                    self.contact_list.append(250)
        if self.p_shape == "U_uni":
            for i,agent in enumerate(self.agents):
                if agent.state.p_pos[0] > -0.5 and  agent.state.p_pos[0] < 0.5 and agent.state.p_pos[1] > -0.1 and agent.state.p_pos[1] < 0.5:
                    agent.role = 1
                    self.contact_list.append(agent.state.p_pos-self.landmarks[0].state.p_pos) #relative position  
                else:
                    self.contact_list.append(250)


    def get_payload_force(self, entity_a, index):
        # if (not self.landmarks[0].collide):
        #     return [None, None] # not a collide

        # assume only agent can apply force to the payload and the payload cannot apply the force to the agent
        # when the agent becomes carrier, it can apply force to the payload. However, if its position to the contact point exceeds the maximum tether distance, the connection will break, it will return to freely move and cannot apply force to the payload.

        # compute actual distance between entities
        delta_pos = entity_a.state.p_pos - (self.landmarks[0].state.p_pos + self.contact_list[index])
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        centre_dist = entity_a.state.p_pos - self.landmarks[0].state.p_pos
        centre_dist1 = np.sqrt(np.sum(np.square(centre_dist)))
        # minimum allowable distance
        dist_min = 0.0
        # softmax penetration
        k = self.contact_margin
        if dist > 0.02:
            penetration = 0.0
            entity_a.role = 0
        else:
            # penetration = np.logaddexp(0, -1000/(dist - dist_min+0.01))*k
            # if (dist - dist_min) == 0.0:
            #     penetration = 0.0
            # else:
            penetration = np.logaddexp(0,-0.0002/(dist - dist_min))*0.0039
        
        force = self.contact_force * delta_pos / dist * penetration
        force_b = force if self.landmarks[0].movable else None
        #print(force_b)
        return force_b

    def swarm_trigger(self):
        dsum = 0.0
        for i in range(len(self.agents)):
            centre_dist = self.agents[i].state.p_pos - self.landmarks[0].state.p_pos
            centre_dist1 = np.sqrt(np.sum(np.square(centre_dist))) 
            dsum = dsum + centre_dist1
        return dsum

    
