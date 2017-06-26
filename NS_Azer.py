import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches


################################## Stuff for the model  #######################
class Car(object):
	def __init__(self, vMax, vIn, pos, ID, speed):
		'''
		A simple car object which holds the properties for each individual 'state'
		
		@param vMax: Maximum speed for this car
		@param vIn: Initial speed of the car
		@param pos: Initial position of the car (tuple of coordinates of grid)
		@param ID: The id of the vehicle
		'''
		self.vMax = vMax
		self.v = vIn  # Current speed of the car
		self.posCurrent = pos # Most recent position at timestep i
		self.posPrevious = pos  # Position at timestep i-1 for animation purposes
		self.ID = ID  # Id of the car on the grid
		self.speed = speed

class CAtwoD(object):
	def __init__(self, N, M, start, pSlow, maxvel, pChange, strategy):
		'''
		The object containing the grid and the logic of the model
		
		@param N: The amount of grids in the x direction
		@param M: The amount of grids in the y direction(no. of lanes)
		@param start: A list of car objects to put on the grid
		@param pSlot: The Nagel-Schreckenberg slowdown probability
		@param maxvel: Maximum velocity (deprecated as each car has one..)
		@param pChange: Lane changing probability in the retard model.
		'''
		self.N, self.M = N, M
		self.grid = np.zeros((N,M), dtype=int) # Used mostly as reference
		self.pSlow, self.pChange = pSlow, pChange
		self.strategy = strategy
		self.maxvel = maxvel
		self.cars = len(start)  # Amount of cars on the road..
		
		self.carIndex = {}
		
		for car in start:
			self.carIndex[car.ID] = deepcopy(car)  # Copy the car object according to id
			self.grid[car.posCurrent] = car.ID  # Add the car id's on the grid
			
		self.fluxCounter = 0			

	def changeCriteria(self, ID, lane):
		'''
		Function which finds the gap between the vehicle in front, behind, relative
		velocities between the vehicles. This data should allow for the information
		a car needs to decide between changing lanes
		
		@param ID: ID of the car we want to find the data for
		@param lane: For which lane we want to estimate this (can be current or
		neighbouring lanes)
		'''
		
		currentCar = self.carIndex[ID]
		i, j = currentCar.posCurrent[0], lane
		
		
		# Find the gap in front, and velocity of car in front if applicable
		frontGap = 10**6  # initialize at arbitrary very high number
		frontVel = 0
		frontVelMax = 0
		slowerCarAhead = False
		found = False
		for k in range(1, currentCar.vMax):
			if self.grid[(i+k) % self.N, j] != 0:  # Car is found
				if found == False:
					frontGap = k-1 
					frontVel = self.carIndex[self.grid[(i+k) % self.N, j]].v
					frontVelMax = self.carIndex[self.grid[(i+k) % self.N, j]].vMax
					found = True
				if self.carIndex[self.grid[(i+k) % self.N, j]].vMax < currentCar.vMax:
					slowerCarAhead = True
					break
			
		# Find the gap at back and the velocity if applicable
		backGap = 10**6  # initialize at arbitrary very high number
		backVel = 0
		backVelMax = 0
		for k in range(1, currentCar.vMax):

			if self.grid[(i-k) % self.N, j] != 0:  # Car is found
				backGap = k-1 
				backVel = self.carIndex[self.grid[(i-k) % self.N, j]].v
				backVelMax = self.carIndex[self.grid[(i-k) % self.N, j]].vMax
				break
			
		return frontGap, frontVel, frontVelMax, backGap, backVel, backVelMax, slowerCarAhead

			
	def laneChangeRandom(self):
		'''
		The lane changing logic which is performed before the movement is executed
		'''
		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			# Say a vehicle changes lanes at random if the lane next to it
			# has room (a vacant space) next to it

			possShifts = []
			if j - 1 >= 0:
				if self.grid[i,j-1] == False:
					possShifts.append(-1)
			if j + 1 < self.M:
				if self.grid[i,j+1] == False:
					possShifts.append(1)
					
			if len(possShifts) > 0 and np.random.rand() < self.pChange:
				shift = np.random.choice(possShifts)
			
				car.posCurrent = (i, j+shift) # Change lane in car
				
				# Update the data in the grid
				self.grid[i,j] = 0
				self.grid[i, j+shift] = car.ID
				

	def laneChangeMike(self):
		'''
		The lane changing logic which is performed before the movement is executed
		This one tries to only change lanes when it is desirable to do so
		'''

		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			# Check in either lanes that are possible if it is desirable to change
			# lane, by checking if at least the current speed can be maintained
			# If the current lane allows this, a lane change is not done
			
			possShifts = [] # Store the shifts that might be done
					
			frontGap, frontVel, frontVelMax, backGap, backVel, backVelMax, slowerCarAhead = self.changeCriteria(car.ID, j)
			
			# All logic for the left lane resides here
			if j - 1 >= 0: # The left lane actually exists
				leftFrontGap, leftFrontVel, leftFrontVelMax, leftBackGap, leftBackVel, leftBackVelMax, leftSlowerCarAhead = self.changeCriteria(car.ID, j-1)
				
				# Check if the position in the other lane is free
				if self.grid[i, j-1] == 0:
					switchPos = True
				else:
					switchPos = False
					
				# Check if the lanechange would allow maintaining of increasing speed
				if leftFrontGap >= frontGap:
					switchAdvantage = True
				else:
					switchAdvantage = False
					
				# Check if the car on the other lane would need to brake hard..
				if leftBackGap - leftBackVel >= -1: # arbitrare limit..
					switchSafe = True 
				else:
					switchSafe = False
					
				if switchPos and switchAdvantage and switchSafe:
					possShifts.append(-1)
					
			# Right side logic
			if j + 1 < self.M : # The right lane actually exists
				rightFrontGap, rightFrontVel, rightFrontVelMax, rightBackGap, rightBackVel, rightBackVelMax, rightSlowerCarAhead = self.changeCriteria(car.ID, j+1)
				
				# Check if the position in the other lane is free
				if self.grid[i, j+1] == 0:
					switchPos = True
				else:
					switchPos = False
					
				# Check if the lanechange would allow maintaining of increasing speed
				if rightFrontGap >= frontGap:
					switchAdvantage = True
				else:
					switchAdvantage = False
					
				# Check if the car on the other lane would need to brake hard..
				if rightBackGap - rightBackVel >= -1: # arbitrare limit..
					switchSafe = True 
				else:
					switchSafe = False
					
				if switchPos and switchAdvantage and switchSafe:
					possShifts.append(1)
					
					
			# Logic regarding actually doing the lane change resides here
			if len(possShifts) > 0 and np.random.rand() < self.pChange:
				shift = np.random.choice(possShifts)
				car.posCurrent = (i, j+shift) # Change lane in car
				
				# Update the data in the grid
				self.grid[i,j] = 0
				self.grid[i, j+shift] = car.ID
	
	def laneChangeSymmetric(self):
		# Symmetric lane changing from Schreckenberg paper
		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			possShifts = [] # Store the shifts that might be done
			frontGap, frontVel, frontVelMax, backGap, backVel, backVelMax, slowerCarAhead = self.changeCriteria(car.ID, j)	

			# All logic for the left lane resides here
			if j - 1 >= 0: # The left lane actually exists
				leftFrontGap, leftFrontVel, leftFrontVelMax, leftBackGap, leftBackVel, leftBackVelMax, leftSlowerCarAhead = self.changeCriteria(car.ID, j-1)
				
				# Condition a
				if frontGap < car.v + 1:
					a = True
				else:
					a = False
				
				# Condition b
				if leftFrontGap > frontGap:
					b = True
				else:
					b = False
				
				# Condition c
				if self.grid[i, j-1] == 0:
					c = True
				else:
					c = False		
					
				# Condition d
				if leftBackVelMax < leftBackGap:
					d = True 
				else:
					d = False
					
				if a and b and c and d:
					possShifts.append(-1)
					
			# Right side logic
			if j + 1 < self.M : # The right lane actually exists
				rightFrontGap, rightFrontVel, rightFrontVelMax, rightBackGap, rightBackVel, rightBackVelMax, rightSlowerCarAhead = self.changeCriteria(car.ID, j+1)
				
				# Condition a
				if frontGap < car.v + 1:
					a = True
				else:
					a = False
				
				# Condition b
				if rightFrontGap > frontGap:
					b = True
				else:
					b = False
				
				# Condition c
				if self.grid[i, j+1] == 0:
					c = True
				else:
					c = False		
					
				# Condition d
				if rightBackVelMax < rightBackGap:
					d = True 
				else:
					d = False
					
				if a and b and c and d:
					possShifts.append(1)
					
					
			# Logic regarding actually doing the lane change resides here
			if len(possShifts) > 0 and np.random.rand() < self.pChange:
				shift = np.random.choice(possShifts)
				car.posCurrent = (i, j+shift) # Change lane in car
				
				# Update the data in the grid
				self.grid[i,j] = 0
				self.grid[i, j+shift] = car.ID								

	def laneChangeAsymmetric(self):
		# Asymmetric lane changing from Schreckenberg paper
		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			possShifts = [] # Store the shifts that might be done
			frontGap, frontVel, frontVelMax, backGap, backVel, backVelMax, slowerCarAhead = self.changeCriteria(car.ID, j)	

			# Change to left lane
			if j + 1 < self.M : # The left lane actually exists
				leftFrontGap, leftFrontVel, leftFrontVelMax, leftBackGap, leftBackVel, leftBackVelMax, leftSlowerCarAhead = self.changeCriteria(car.ID, j+1)
				
				# Condition a 
				if (frontGap < car.v + 1) or slowerCarAhead:
					a = True
				else:
					a = False
				
				# Condition b
				if leftFrontGap > frontGap:
					b = True
				else:
					b = False
				
				# Condition c
				if self.grid[i, j+1] == 0:
					c = True
				else:
					c = False		
					
				# Condition d
				if leftBackVelMax < leftBackGap:
					d = True 
				else:
					d = False
					
				if a and b and c and d:
					possShifts.append(1)
					
			# Change to right lane
			if j - 1 >= 0 : # The right lane actually exists
				rightFrontGap, rightFrontVel, rightFrontVelMax, rightBackGap, rightBackVel, rightBackVelMax, rightSlowerCarAhead = self.changeCriteria(car.ID, j-1)
									
				# Conditon a'
				if car.speed == 'fast' and rightSlowerCarAhead:
					a = False
				else:
					a = True
				
				# Condition b'
				if rightFrontGap > car.v:
					b = True
				else:
					b = False
				
				# Condition c'
				if self.grid[i, j-1] == 0:
					c = True
				else:
					c = False		
					
				# Condition d'
				if rightBackVelMax < rightBackGap:
					d = True 
				else:
					d = False		
			
				if a and b and c and d:
					possShifts.append(-1)
					
					
			# Logic regarding actually doing the lane change resides here
			if len(possShifts) > 0 and np.random.rand() < self.pChange:
				shift = np.random.choice(possShifts)
				car.posCurrent = (i, j+shift) # Change lane in car
				
				# Update the data in the grid
				self.grid[i,j] = 0
				self.grid[i, j+shift] = car.ID					
	
	def laneChangeTest(self):
		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			shift = 0
			# Change to left lane
			if j + 1 < self.M and car.speed == 'fast' and self.grid[i,j+1] == 0:
				shift = 1
					
			# Change to right lane
			if j - 1 >= 0 and car.speed == 'slow' and self.grid[i,j-1] == 0:
				shift = -1
				
			# Logic regarding actually doing the lane change resides here
			if np.random.rand() < self.pChange:
				car.posCurrent = (i, j+shift) # Change lane in car
				
				# Update the data in the grid
				self.grid[i,j] = 0
				self.grid[i, j+shift] = car.ID		
				
	def moveTimeStep(self):
		''' 
		Simulate the movement of each of the vehicles according to Nagel-Schrecken
		'''

		# First increase speed with 1 if maxspeed has not been reached
		# All these functions use the positions of the car objects and reference
		# the grid for finding id and updating the position in the grid
		for car in self.carIndex.values():  
			if car.v < car.vMax:
				car.v += 1
				
		# Check if any cars in front are within maxspeed distance and slow down
		for car in self.carIndex.values():
			
			for j in range(1, car.v + 1):
				if self.grid[(car.posCurrent[0] + j) % self.N, car.posCurrent[1]]:
					car.v = j-1  # Reduce speed so a crash is prevented
					break # No need to check other squares further in front
					
		# Randomize speeds/slowdowns
		for car in self.carIndex.values():
			if np.random.rand() < self.pSlow and car.v > 0:
				car.v  -= 1
			
		# Move the cars forward depending on their speed
		for car in self.carIndex.values():
			# Generate the new position of this car
			posCurrent = car.posCurrent
			newPos = ((posCurrent[0] + car.v) % self.N, posCurrent[1])
			
			# Update the grid
			self.grid[posCurrent] = 0
			self.grid[newPos] = car.ID
			
			# The moment when a car passes the periodic boundary
			if (posCurrent[0] + car.v) >= N:
				self.fluxCounter += 1
			
			# Update the position of the car object
			car.posCurrent = newPos
		
		
	def updateGrid(self):
		'''
		The updating step doing all the actions for a timestep
		'''
		
		# First update the previous position for each of the cars to the current pos
		for i in self.carIndex.keys():
			self.carIndex[i].posPrevious = self.carIndex[i].posCurrent
				
		if self.strategy == 'random':
			self.laneChangeRandom()
		elif self.strategy == 'mike':
			self.laneChangeMike()
		elif self.strategy == 'symmetric':
			self.laneChangeSymmetric()
		elif self.strategy == 'asymmetric':
			self.laneChangeAsymmetric()
		elif self.strategy == 'test':
			self.laneChangeTest()		
		self.moveTimeStep()
	
	def returnAverageVelocity(self):
		avSpeed = 0
		
		for car in self.carIndex.values():
			avSpeed += car.v
		return avSpeed/len(self.carIndex.keys())
	
	def returnPlotState(self):
		plotState, distances = [], []
		for car in self.carIndex.values():
			plotState.append((car.posPrevious, car.posCurrent))
			distances.append(car.v)
		return plotState, distances
	
def generateStart(N, M, num, maxV1, maxV2):
	'''
	Generates a list of tuples containing grid coordinates on which vehicles are
	initialized
	
	@param N: Amount of discretizations in the x-direction
	@param M: Amount of discretizations in the y-direction (lanes)
	@param num: Amount of vehicles to generate. Must be <= N*M
	'''
	
	start = set()  # Set to prevent duplicates
	
	while len(start) < num:
		start.add((np.random.randint(0, N), np.random.randint(0,M)))
		
	cars = []
	start = list(start)
	for i in range(int(len(start)/2)+1):
		cars.append(Car(maxV1,1,start[i], i+1, 'slow'))
	for i in range(int(len(start)/2)+1,len(start)):
		cars.append(Car(maxV2,1,start[i], i+1, 'fast'))	
	return cars

'''
__________________________________
Executing an instance of the CA
__________________________________
'''
# Parameters
N, M = 80, 2 # Amount of cells needed for the CA
carnum = 25 # Number of cars to add to the CA
pSlow = 0.1
maxVel1 = 3 # slow
maxVel2 = 10 # fast
pChange = 1
strategy = 'asymmetric' # random, mike, symmetric, asymmetric, test
animatie = True

# Starting cars
start = generateStart(N, M, carnum, maxVel1, maxVel2)

# Create a CA object
test = CAtwoD(N, M, start, pSlow, maxVel1, pChange, strategy)

if animatie == False:
	# Show results
	timesteps = 10000
	averageVeloList = []
	for i in range(timesteps):
		test.updateGrid()
		averageVeloList.append(test.returnAverageVelocity())
	print('Flow rate = ', test.fluxCounter/timesteps)
	print('Average velocity = ', np.mean(averageVeloList))
	
	
	
'''
___________________________________
Everything related to the animation 
___________________________________
'''
	
def findCoors(N, M, xmin, xmax, ymin, ymax ):
    '''
    Subdivide the x and y into N and M parts for the backgrounds
    ''' 
    dx, dy = float(xmax - xmin)/N, float(ymax - ymin)/M 
    coors = []
    trans = {}
    for i in zip(np.linspace(xmin, xmax, N), range(N)):
        for j in zip(np.linspace(ymin, ymax, M), range(M)):
            coors.append((i[0], j[0]))
            trans[(i[1], j[1])] = (i[0] + dx/2, j[0] + dy/2)
            
    return coors, float(xmax - xmin)/N, float(ymax - ymin)/M , trans


def init():
	line.set_data([], [])
	time_text.set_text('')
	return line, time_text

def animateDataGen(lim):
	n = 0
	maxDist = xmax + dx
	while n < lim:
		test.updateGrid()
		points, vels = test.returnPlotState()
		realSpacePoints = []
		realSpaceDistances = []
		
		# Translate the grid coordinates to real space
		for i in range(len(vels)):
			realSpacePoints.append([trans[points[i][0]], trans[points[i][1]]])
			realSpaceDistances.append(dx*vels[i])
			
		# Create two arrays, containing intermediate x and y coordinates
		xPoints, yPoints = [], []
		
		for i in range(len(vels)):
			xCoors = np.linspace(realSpacePoints[i][0][0], 
										realSpacePoints[i][0][0] + realSpaceDistances[i], 
										steps) % maxDist
			yCoors = np.linspace(realSpacePoints[i][0][1], realSpacePoints[i][1][1], steps)

			xPoints.append(xCoors)
			yPoints.append(yCoors)

		# Run through each of the coordinates and yield a list of x and y plot vals
		for i in range(steps - 1):
			xList, yList = [], []
			for j in range(len(vels)):
				xList.append(xPoints[j][i])
				yList.append(yPoints[j][i])
			yield xList, yList
			
		n += 1


def animate2(i):
	thisx, thisy = next(dataGen)
	ycoordinates.extend(thisy)
	
	line.set_data(thisx, thisy)
	time_text.set_text(time_template.format(int(i/steps), i%steps,test.fluxCounter))
	
	
	return line, time_text

xmin, xmax, ymin, ymax = 0, 10, -0.5, 0.5  # For plotting
ycoordinates = []
# Find the translations for plotting the grid
coors,dx,dy,trans = findCoors(N, M, xmin, xmax, ymin, ymax)


# These are variables for the plotting stuff
steps = 30
lim = 2000

if animatie:
	dataGen = animateDataGen(lim)
	fig = plt.figure()
	ax = fig.add_subplot(111, autoscale_on=False, xlim=(xmin, xmax), ylim=(ymin, ymax))

	# Paint a background grid..
	for i in coors:
		ax.add_patch(
				patches.Rectangle(
            i,   # (x,y)
            dx,          # width
            dy,          # height
            color='black', 
				fill=False
        )
    )
	
	line, = ax.plot([], [], 'rs', markersize=xmax/(0.05*N))
	time_template = 'timestep {0}, frame {1}, Counter {2}'
	time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
	plt.axis('equal')
	
	
	ani = animation.FuncAnimation(fig, animate2, lim*(steps),
										   interval=10, blit=True, init_func=init, repeat=False)
	
	plt.show()