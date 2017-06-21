import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from copy import deepcopy


################################## Stuff for the model  #######################
class Car(object):
	def __init__(self, vMax, vIn, pos, ID):
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

class CAtwoD(object):
	def __init__(self, N, M, start, pSlow, maxvel, pChange):
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
		i, j = currentCar.posCurrent
		
		
		# Find the gap in front, and velocity of car in front if applicable
		frontGap = self.maxvel  # init at max value road
		frontVel = 0
		for k in range(1, self.maxvel + 1):
			print(i, (i+k) % self.N, self.grid[(i+k) % self.N, j])
			if self.grid[(i+k) % self.N, j] != 0:  # Car is found
				frontGap = k-1 
				frontVel = self.carIndex[self.grid[(i+k) % self.N, j]].v
				break
			
		# Find the gap at back and the velocity if applicable
		
				# Find the gap in front
		backGap = self.maxvel  # init at max value road
		backVel = 0
		for k in range(1, self.maxvel + 1):
			print(i, (i-k) % self.N, self.grid[(i+k) % self.N, j])
			if self.grid[(i-k) % self.N, j] != 0:  # Car is found
				backGap = k-1 
				backVel = self.carIndex[self.grid[(i-k) % self.N, j]].v
				break
		
		return frontGap, frontVel, backGap, backVel
						
	def laneChange(self):
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
				
						
						
	def laneChange2(self):
		'''
		The lane changing logic which is performed before the movement is executed
		This one tries to only change lanes when it is desirable to do so
		'''
		for car in self.carIndex.values():
			i, j = car.posCurrent[0], car.posCurrent[1]
			# Check in either lanes that are possible if it is desirable to change
			# lane, by checking if at least the current speed can be maintained
			# If the current lane allows this, a lane change is not done
			
			possShifts = []
			
			# Left side logic
			if j - 1 >= 0: # The lane actually exists
				for k in range(0, car.v + 1):
					if self.grid[(i+k) % self.N, j-1] == False:
						break
					
				if k == car.v:
					possShifts.append(-1)
					
			# Right side logic
			if j + 1 < self.M:
				for l in range(0, car.v + 1):
					if self.grid[(i+l) % self.N, j+1] == False:
						break
				if l == car.v:
					possShifts.append(1)
					
			# Check if there is enough space in front to maintain speed
			for m in range(0, car.v+1):
				if self.grid[(i+m) % self.N, j] == False:
					break
			
			if len(possShifts) > 0 and np.random.rand() < self.pChange \
					and m < car.v:
				shift = np.random.choice(possShifts)
			
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
#		for car in self.carIndex.values():			
#			for j in range(1, car.v + 1):
#				if self.grid[(car.posCurrent[0] + j) % self.N, car.posCurrent[1]]:
#					car.v = j-1  # Reduce speed so a crash is prevented
#					break # No need to check other squares further in front	
					
#     waarom werkt dit niet		
		for car in self.carIndex.values():
			frontgap = self.changeCriteria(car.ID, car.posCurrent[1])[0]
			if car.v > frontgap:				
				car.v = frontgap # Reduce speed so a crash is prevented
				# No need to check other squares further in front
					
		# Randomize speeds/slowdowns
		for car in self.carIndex.values():
			if np.random.rand() < self.pSlow and car.v > 0:
				car.v  -= 1
			
		# Move the cars forward depending on their speed
		for car in self.carIndex.values():
			# Generate the new position of this car
			posCurrent = car.posCurrent
			newPos = ((posCurrent[0] + car.v) % self.N, posCurrent[1])
			if (posCurrent[0] + car.v) >= N:
				self.fluxCounter += 1
			
			# Update the grid
			self.grid[posCurrent] = 0
			self.grid[newPos] = car.ID
			
			# Update the position of the car object
			car.posCurrent = newPos
		
		
	def updateGrid(self):
		'''
		The updating step doing all the actions for a timestep
		'''
		
		# First update the previous position for each of the cars to the current pos
		for i in self.carIndex.keys():
			self.carIndex[i].posPrevious = self.carIndex[i].posCurrent
			
		self.laneChange()
		self.moveTimeStep()
	
	def returnAverageVelocity(self):
		return np.sum(self.velocities)/self.cars
	
	def returnPlotState(self):
		plotState, distances = [], []
		for car in self.carIndex.values():
			plotState.append((car.posPrevious, car.posCurrent))
			distances.append(car.v)
		return plotState, distances
	
def generateStart(N, M, num):
	'''
	Generates a list of tuples containing grid coordinates on which vehicles are
	initialized
	
	@param N: Amount of discretizations in the x-direction
	@param M: Amount of discretizations in the y-direction (lanes)
	@param num: Amount of vehicles to generate. Must be <= N*M
	'''
	
	start = set()  # Set to prevent duplicates
	
	while len(start) < num:
		start.add((np.random.randint(0, N), np.random.randint(0, M)))
		
	cars = []
	start = list(start)
	for i in range(len(start)):
		cars.append(Car(5,1,start[i], i+1))
		
	return cars
			
############################# Everything related to the animation #############
	
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


################################# Executing of an instance of the CA #########
N, M = 30, 2 # Amount of cells needed for the CA
carnum = 40 # Number of cars to add to the CA
xmin, xmax, ymin, ymax = 0, 10, -0.5, 0.5  # For plotting

# Starting cars
start = generateStart(N, M, carnum)

# Create a CA object
test = CAtwoD(N, M, start, 0.1, 5, 1.0)
ycoordinates = []

# Find the translations for plotting the grid
coors,dx,dy,trans = findCoors(N, M, xmin, xmax, ymin, ymax)

# These are variables for the plotting stuff
steps = 30
lim = 2000
dataGen = animateDataGen(lim)

animatie = True

if animatie:
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
	time_template = 'timestep {0}, frame {1}, counter {2}'
	time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
	plt.axis('equal')
	
	
	ani = animation.FuncAnimation(fig, animate2, lim*(steps),
										   interval=10, blit=True, init_func=init, repeat=False)
	
	plt.show()
	
	

    