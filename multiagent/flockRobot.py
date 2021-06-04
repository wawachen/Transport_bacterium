import numpy as np
import random 
import math


class flockRobot:

    def __init__(self,id,world):
        self.idNo = id
        self.world = world
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
        
        # double angle    #for random tumble angle
        self.cosAngle = 0.0  #for x direction #for random tumble angle
        # double cosAngle   #for x direction
        self.sinAngle = 0.0 #for y direction
        self.previousAngle = 0.0   #for forward motion
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
        # thresholdValue = 60 
        
        #fuzzyCntrllerFR flockingBehaviour 
        
        # /*for the sine wave*/
        self.sineWaveX = 0.0	
        self.sineWaveY = 0.0 
	
		self.myXVelocity = 0.0 
		self.myYVelocity = 0.0  			
		
		self.myXPos = random.uniform(-1,1)  #(double)(widthOfEnvironment) 
		self.myYPos = random.uniform(-1,1)  #(double)(heightOfEnvironment) 
		
		self.bactXPos = self.myXPos 
		self.bactYPos = self.myYPos 
        self.flockXPos = self.bactXPos 
		self.flockYPos = self.bactYPos  
		
		self.myXVelocity = random.uniform(-0.01, 0.01)
		self.myYVelocity = random.uniform(-0.01, 0.01) 		
		 
	
    def getCentreDensity(self):
        pollutionCount = 0
        # amplify it by 1000 times
        #print(agent.state.p_pos[0])
        xDPos = int(self.myXPos/self.world.interval + self.world.xsize/2)
        yDPos = int(self.myYPos/self.world.interval + self.world.ysize/2)

        for x in range(-50,50):
            for y in range(-50,50):
                tempX = xDPos + x
                tempY = yDPos + y

                if tempX < 0:
                    tempX = 0
                elif tempX > self.world.xsize:
                    tempX = self.world.xsize

                if tempY < 0:
                    tempY = 0
                elif tempY > self.world.ysize:
                    tempY = self.world.ysize

                pollutionCount = pollutionCount + self.world.pollution_map[tempX][tempY]
        
        return pollutionCount
	
	def collateData(self): 
		
		#assemble robot data for sending to others 
		self.myData[0] = double(self.idNo) 
		self.myData[1] = self.myXPos 
		self.myData[2] = self.myYPos 
		self.myData[3] = self.getCentreDensity()
		# myData[4] = (double)batteryLevel 
		
		#System.out.println("pollutant level is" + pollutantSensor.getCentreDensity(x, y)) 
		#System.out.println("out here is" + myXPos) 
		#wSystem.out.println("myXPos:" + myXPos) 
	
	
	def flockExp2(self, number):
	
		number = double(math.exp(-number + 40)) 
		number = double(0.5 * math.exp(-0.1 * number)) 

		return number 
			 
	
	def repulsiveForce(self, d):
		v = np.zeros(2) 
		r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 
		
		a = 100   #50
		b = 1 
		w = 2 
		e = 1 
		forceX = 0 
		forceY = 0 
		force = 0 
		temp = np.zeros(2) 	
		repulsionTerm = self.getCentreDensity() / 148.0 
		
		 #v/a = (a / pollutantSensor.getCentreDensity(myXPos, myYPos)) + 10 
		 #force  = repulsionTerm * (Math.max(((a / ((b * r) - w)) - e),0)) 
		force  = (math.max(((a / ((b * r) - w)) - e),0)) 
		 #the greater this value the more the distance between agents.
		 #so make sure it never goes to large value leading to scattering of agents in 
		 #all directions. 
			
		force  = math.min(force, 0.1) 
		
		if(r == 0)
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
		
		averageX = ((averageX + self.myXPos) / 2) 
		averageY = ((averageY + self.myYPos) / 2) 
		
		 #System.out.println("averageX is:" + averageX) 
		 #System.out.println("averageY is:" + averageY) 
		
		v[0] = averageX 
		v[1] = averageY 
		
		self.myXPos = averageX 
		self.myYPos = averageY 
		
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
 #
		v[0]=-double(temp[0]) 
		v[1]=-double(temp[1]) 

		return v 
	
	 # match the velocity of 2 nearest neighbours
	def match_velocity(d):
	
		v = np.zeros(2) 

		v[0]=d[0]*.02 
		v[1]=d[1]*.02 
		v[0]+=d[2]*.02 
		v[1]+=d[3]*.02 
 		
		return v 
	
	def transmit(info):
	
		#  #store data in a central location for access by all robots.
		id = int(info[0])    #get Id of robot
		self.world.robotInfo[id,0] = info[0];  #store Id of robot
		self.world.robotInfo[id,1] = info[1];  #store x position
		self.world.robotInfo[id,2] = info[2];  #store y position
		self.world.robotInfo[id,3] = info[3];  #store pollution reading.

	
	def run():
		self.collateData() 
		# myRadio.transmit(myData) 
        self.transmit(self.myData)
		self.receiveNeighbourData() 
		self.bacteriaController() 
		 #this.sineMotion() 
		 #presentReadN = myData[3] 
		 #this.boundaryFollowing2() 
		self.flockingController2() 
		 #this.steeringDirection2() 
		self.combineBehaviours() 
		 #this.fuzzyController() 
	


	def check_bounds(self):	
	
		v = np.zeros(2) 

		if self.myXPos < -1:
			v[0] = 0.001 
		elif self.myXPos > 1:
			v[0] = -0.001 
		
		if self.myYPos < -1:
			v[1] = 0.001 
		elif self.myYPos > 1:
			v[1] = -0.001 

		return v 
	
	
    def receive2(self):
		relDistX = 0
		relDistY = 0
		newDist = 0
		tempDist = 1000000
		
		
		 #go through robot ids and find the nearest robot to me.
		 #store the x and y distance from me to the nearest robot.

        for id in range(len(self.world.agents)):
            if self.idNo != id:
                relDistX = self.robotInfo[id][1] - self.myXPos
				relDistY = self.robotInfo[id][2] - self.myYPos
				newDist =  math.sqrt(relDistX*relDistX + relDistY*relDistY)
			
				 #System.out.println("relDistX:" + robotInfo[id][1]);
				 #System.out.println("relDistY:" + robotInfo[id][2]);
			
				if(newDist < tempDist):
					tempDist = newDist
					self.dataOf2Neighbours[0] = relDistX
					self.dataOf2Neighbours[1] = relDistY
	
	

	def flockingController2(self):
		sepDist = np.zeros(2) 
		repulsiveDist = np.zeros(2)  
		attractDist = np.zeros(2) 
		checkBounds = np.zeros(2)  
		highestForagerPos = np.zeros(2) 
		
		self.receive2() 
		sepDist = self.separation(self.dataOf2Neighbours)   # separation force
		attractDist = self.attractForce() 
		repulsiveDist = self.repulsiveForce(self.dataOf2Neighbours) 
		checkBounds = self.check_bounds() 
		highestForagerPos = self.highestForager() 
		
		self.xGoal = 0.01*(double(highestForagerPos[0]) - self.flockXPos)  #highestForagerPos[0] 
		self.yGoal = 0.01*(double(highestForagerPos[1]) - self.flockYPos)  #highestForagerPos[1] 
		
		self.myXVelocity = (0.8 * self.myXVelocity + (1.0 * sepDist[0] + 0.0 * double(self.xGoal) + 0.0 * repulsiveDist[0]) + checkBounds[0]) 	 # Combine the rules
		self.myYVelocity = (0.8 * self.myYVelocity + (1.0 * sepDist[1] + 0.0 * double(self.yGoal) + 0.0 * repulsiveDist[1]) + checkBounds[1])   
        
		self.flockXPos = self.flockXPos + self.myXVelocity 
		self.flockYPos = self.flockYPos + self.myYVelocity  
		
		if(math.abs(self.myXVelocity) > 0.01)
			self.myXVelocity = (self.myXVelocity/math.abs(self.myXVelocity))*0.01

		if(math.abs(self.myYVelocity) > 0.01)
			self.myYVelocity = (self.myYVelocity/math.abs(self.myYVelocity))*0.01
		
		
	
	
	def receiveNeighbourData(self):
        neighbourData = np.zeros(self.numberOfNeighbours, 4)
		counter = 0
		
        for id in range(len(self.world.agents)):
            if self.idNo != double(self.world.robotInfo[id,0]):
                if (self.robotInfo[id,1] < (self.myXPos + self.commsRadius)) and (self.robotInfo[id,1] > (self.myXPos - self.commsRadius)) and (self.robotInfo[id,2] < (self.myYPos + self.commsRadius)) and (self.robotInfo[id,2] > (self.myYPos - self.commsRadius)):
                     #store the x and y coordinates of robots within the comms radius
					neighbourData[counter,0] = self.robotInfo[id,0]
					neighbourData[counter,1] = self.robotInfo[id,1]
					neighbourData[counter,2] = self.robotInfo[id,2]
					neighbourData[counter,3] = self.robotInfo[id,3]
					
					counter = counter + 1
						
					 #return the first encountered closest neigbhours
					 #not good way. Should first get all the robots
					 #within the comms radius and return the values of the nearest
					 #number of neighbours. will work on that later
					
					if(counter >= self.numberOfNeighbours):
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
			highestForagerXPos = self.myXPos 
			highestForagerYPos = self.myYPos 
		
		
		
		 #highestForagerData[0] = highestPolluLevel 
		highestForagerData[0] = highestForagerXPos 
		highestForagerData[1] = highestForagerYPos 
		
 #		System.out.println("highestPolluLevel is:" + highestPolluLevel) 
 #		System.out.println("highestForagerData[0] is:" + highestForagerXPos) 
 #		System.out.println("highestForagerData[1] is:" + highestForagerData[1]) 
 #		
		return highestForagerData 
	
	 #standard bacteria algorithm with dynamic velocity based upon reading. 
	def bacteriaController():
	
		moveX = 0 
		moveY = 0 
		tumblePosition = np.zeros(2) 
	    thresholdValue = 14 
		velocity = 3 
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
				
		stdVelocity = 64  
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
			self.bactXPos = self.bactXPos + self.bactVelocity  * self.cosAngle 
			self.bactYPos = self.bactYPos + self.bactVelocity  * self.sinAngle 
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
		
	
	def combineBehaviours(self):
		
		self.myXPos = 0.0 * self.flockXPos + 0.8 * self.bactXPos + 0.0 * self.sineWaveX + 0.0 * self.steerNX 
		self.myYPos = 0.0 * self.flockYPos + 0.8 * self.bactYPos + 0.0 * self.sineWaveY + 0.0 * self.steerNY 


	
class flockRobot1:

    def __init__(self,id,world):
        self.idNo = id
        self.world = world
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
        
        # double angle    #for random tumble angle
        self.cosAngle = 0.0  #for x direction #for random tumble angle
        # double cosAngle   #for x direction
        self.sinAngle = 0.0 #for y direction
        self.previousAngle = 0.0   #for forward motion
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
        
        #fuzzyCntrllerFR flockingBehaviour 
        
        # /*for the sine wave*/
        self.sineWaveX = 0.0	
        self.sineWaveY = 0.0 
	
		self.myXVelocity = 0.0 
		self.myYVelocity = 0.0  			
		
		self.myXPos = random.uniform(-1,1)  #(double)(widthOfEnvironment) 
		self.myYPos = random.uniform(-1,1)  #(double)(heightOfEnvironment) 
		
		self.bactXPos = self.myXPos 
		self.bactYPos = self.myYPos 
        self.flockXPos = self.bactXPos 
		self.flockYPos = self.bactYPos  
		
		self.myXVelocity = random.uniform(-0.01, 0.01)
		self.myYVelocity = random.uniform(-0.01, 0.01) 		
		 
	
    def getCentreDensity(self):
        pollutionCount = 0
        # amplify it by 1000 times
        #print(agent.state.p_pos[0])
        xDPos = int(self.myXPos/self.world.interval + self.world.xsize/2)
        yDPos = int(self.myYPos/self.world.interval + self.world.ysize/2)

        for x in range(-50,50):
            for y in range(-50,50):
                tempX = xDPos + x
                tempY = yDPos + y

                if tempX < 0:
                    tempX = 0
                elif tempX > self.world.xsize:
                    tempX = self.world.xsize

                if tempY < 0:
                    tempY = 0
                elif tempY > self.world.ysize:
                    tempY = self.world.ysize

                pollutionCount = pollutionCount + self.world.pollution_map[tempX][tempY]
        
        return pollutionCount
	
	def collateData(self): 
		
		#assemble robot data for sending to others 
		self.myData[0] = double(self.idNo) 
		self.myData[1] = self.myXPos 
		self.myData[2] = self.myYPos 
		self.myData[3] = self.getCentreDensity()
		# myData[4] = (double)batteryLevel 
		
		#System.out.println("pollutant level is" + pollutantSensor.getCentreDensity(x, y)) 
		#System.out.println("out here is" + myXPos) 
		#wSystem.out.println("myXPos:" + myXPos) 
	
	
	def flockExp2(self, number):
	
		number = double(math.exp(-number + 40)) 
		number = double(0.5 * math.exp(-0.1 * number)) 

		return number 
			 
	
	def repulsiveForce(self, d):
		v = np.zeros(2) 
		r = math.sqrt(d[0] * d[0] + d[1] * d[1]) 
		
		a = 50   #50
		b = 1 
		w = 2 
		e = 1 
		forceX = 0 
		forceY = 0 
		force = 0 
		temp = np.zeros(2) 	
		repulsionTerm = self.getCentreDensity() / 148.0 
		
		 #v/a = (a / pollutantSensor.getCentreDensity(myXPos, myYPos)) + 10 
		 #force  = repulsionTerm * (Math.max(((a / ((b * r) - w)) - e),0)) 
		force  = (math.max(((a / ((b * r) - w)) - e),0)) 
		 #the greater this value the more the distance between agents.
		 #so make sure it never goes to large value leading to scattering of agents in 
		 #all directions. 
			
		force  = math.min(force, 0.1) 
		
		if(r == 0)
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
		
		averageX = ((averageX + self.myXPos) / 2) 
		averageY = ((averageY + self.myYPos) / 2) 
		
		 #System.out.println("averageX is:" + averageX) 
		 #System.out.println("averageY is:" + averageY) 
		
		v[0] = averageX 
		v[1] = averageY 
		
		self.myXPos = averageX 
		self.myYPos = averageY 
		
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
		v[0]=-double(temp[0]) 
		v[1]=-double(temp[1]) 

		return v 
	
	 # match the velocity of 2 nearest neighbours
	def match_velocity(d):
	
		v = np.zeros(2) 

		v[0]=d[0]*.02 
		v[1]=d[1]*.02 
		v[0]+=d[2]*.02 
		v[1]+=d[3]*.02 
 		
		return v 
	
	def transmit(info):
	
		#  #store data in a central location for access by all robots.
		id = int(info[0])    #get Id of robot
		self.world.robotInfo[id,0] = info[0];  #store Id of robot
		self.world.robotInfo[id,1] = info[1];  #store x position
		self.world.robotInfo[id,2] = info[2];  #store y position
		self.world.robotInfo[id,3] = info[3];  #store pollution reading.

	
	def run():
		self.collateData() 
		# myRadio.transmit(myData) 
        self.transmit(self.myData)
		self.receiveNeighbourData() 
		self.bacteriaController7() 
		 #this.sineMotion() 
		 #presentReadN = myData[3] 
		 #this.boundaryFollowing2() 
		self.flockingController2() 
		 #this.steeringDirection2() 
		self.combineBehaviours() 
		 #this.fuzzyController() 
	


	def check_bounds(self):	
	
		v = np.zeros(2) 

		if self.myXPos < -1:
			v[0] = 0.001 
		elif self.myXPos > 1:
			v[0] = -0.001 
		
		if self.myYPos < -1:
			v[1] = 0.001 
		elif self.myYPos > 1:
			v[1] = -0.001 

		return v 
	
	
    def receive2(self):
		relDistX = 0
		relDistY = 0
		newDist = 0
		tempDist = 1000000
		
		
		 #go through robot ids and find the nearest robot to me.
		 #store the x and y distance from me to the nearest robot.

        for id in range(len(self.world.agents)):
            if self.idNo != id:
                relDistX = self.robotInfo[id][1] - self.myXPos
				relDistY = self.robotInfo[id][2] - self.myYPos
				newDist =  math.sqrt(relDistX*relDistX + relDistY*relDistY)
			
				 #System.out.println("relDistX:" + robotInfo[id][1]);
				 #System.out.println("relDistY:" + robotInfo[id][2]);
			
				if(newDist < tempDist):
					tempDist = newDist
					self.dataOf2Neighbours[0] = relDistX
					self.dataOf2Neighbours[1] = relDistY
	
	

	def flockingController2(self):
		sepDist = np.zeros(2) 
		repulsiveDist = np.zeros(2)  
		attractDist = np.zeros(2) 
		checkBounds = np.zeros(2)  
		highestForagerPos = np.zeros(2) 
		
		self.receive2() 
		sepDist = self.separation(self.dataOf2Neighbours)   # separation force
		attractDist = self.attractForce() 
		repulsiveDist = self.repulsiveForce(self.dataOf2Neighbours) 
		checkBounds = self.check_bounds() 
		highestForagerPos = self.highestForager() 
		
		self.xGoal = 0.01*(double(highestForagerPos[0]) - self.flockXPos)  #highestForagerPos[0] 
		self.yGoal = 0.01*(double(highestForagerPos[1]) - self.flockYPos)  #highestForagerPos[1] 
		
		self.myXVelocity = (0.8 * self.myXVelocity + (1.0 * sepDist[0] + 0.0 * double(self.xGoal) + 0.0 * repulsiveDist[0]) + checkBounds[0]) 	 # Combine the rules
		self.myYVelocity = (0.8 * self.myYVelocity + (1.0 * sepDist[1] + 0.0 * double(self.yGoal) + 0.0 * repulsiveDist[1]) + checkBounds[1])   
        
		self.flockXPos = self.flockXPos + self.myXVelocity 
		self.flockYPos = self.flockYPos + self.myYVelocity  
		
		if(math.abs(self.myXVelocity) > 0.01)
			self.myXVelocity = (self.myXVelocity/math.abs(self.myXVelocity))*0.01

		if(math.abs(self.myYVelocity) > 0.01)
			self.myYVelocity = (self.myYVelocity/math.abs(self.myYVelocity))*0.01
		
		
	
	
	def receiveNeighbourData(self):
        neighbourData = np.zeros(self.numberOfNeighbours, 4)
		counter = 0
		
        for id in range(len(self.world.agents)):
            if self.idNo != double(self.world.robotInfo[id,0]):
                if (self.robotInfo[id,1] < (self.myXPos + self.commsRadius)) and (self.robotInfo[id,1] > (self.myXPos - self.commsRadius)) and (self.robotInfo[id,2] < (self.myYPos + self.commsRadius)) and (self.robotInfo[id,2] > (self.myYPos - self.commsRadius)):
                     #store the x and y coordinates of robots within the comms radius
					neighbourData[counter,0] = self.robotInfo[id,0]
					neighbourData[counter,1] = self.robotInfo[id,1]
					neighbourData[counter,2] = self.robotInfo[id,2]
					neighbourData[counter,3] = self.robotInfo[id,3]
					
					counter = counter + 1
						
					 #return the first encountered closest neigbhours
					 #not good way. Should first get all the robots
					 #within the comms radius and return the values of the nearest
					 #number of neighbours. will work on that later
					
					if(counter >= self.numberOfNeighbours):
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
			highestForagerXPos = self.myXPos 
			highestForagerYPos = self.myYPos 
		
		
		
		 #highestForagerData[0] = highestPolluLevel 
		highestForagerData[0] = highestForagerXPos 
		highestForagerData[1] = highestForagerYPos 
		
 #		System.out.println("highestPolluLevel is:" + highestPolluLevel) 
 #		System.out.println("highestForagerData[0] is:" + highestForagerXPos) 
 #		System.out.println("highestForagerData[1] is:" + highestForagerData[1]) 
 #		
		return highestForagerData 
	
	 #standard bacteria algorithm with dynamic velocity based upon reading. 
	def bacteriaController7(self):
	
		moveX = 0
		moveY = 0
		tumblePosition = np.zeros(2)
		thresholdValue = 14
		double velocity = 3
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
			self.bactVelocity = 5
		else:
			self.bactVelocity = (self.vWDP_Dt * 1) + 1  #use sigmoid function here..
		
		if self.bactVelocity > 0.01:
		    self.bactVelocity = 0.01
		elif self.bactVelocity < -0.01
			self.bactVelocity = -0.01
		
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
			self.bactXPos = self.bactXPos + self.bactVelocity  * self.cosAngle 
			self.bactYPos = self.bactYPos + self.bactVelocity  * self.sinAngle 
			self.newAngle = False 
			
	
	def tumbleAngle(self):
	 
		angle = np.random.random() * 360 
		# cosAngle = math.cos(math.radians(angle)) 
		# sinAngle = Math.sin(Math.radians(angle)) 
		
 #		this.steeringDirection2() 
		self.cosAngle = math.cos(math.radians(angle)) 
		self.sinAngle = math.sin(math.radians(angle)) 
		
	
	def combineBehaviours(self):
		self.myXPos = 0.8 * self.flockXPos + 0.8 * self.bactXPos
		self.myYPos = 0.8 * self.flockYPos + 0.8 * self.bactYPos
	


