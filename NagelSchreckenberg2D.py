import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from copy import deepcopy

class CAtwoD(object):
	def __init__(self, N, M, start, pSlow, maxvel, pChange):
		self.N = N
		self.M = M
		self.grid = np.zeros((N,M))
		self.velocities = np.zeros((N,M))
		self.pSlow = pSlow
		self.pChange = pChange
		self.maxvel = maxvel
		self.cars = len(start)
		
		for i in start:
			self.grid[i] = 1
			self.velocities[i] = 1
		
		self.plotState = []
		self.distances = []  # Grid distances
			
	def returnGrid(self):
		'''
		Function which returns the grid for plotting purposes. The coordinates
		of the cars will be stored in an array.
		'''
		newGrid = []
		for i in range(self.N):
			if self.grid[i] == True:
				newGrid.append((i, 0))

		return newGrid
	
	def updateGrid(self):
		'''
		Function which updates the grid, considering periodic boundary conditions
		using a basic Nagel-Schreckenberg model
		'''
		self.plotState = [] # Init the plotstate again
		self.distances = [] # Init the distances again
		self.laneChanges = {} # Keep track of the lane changes for the animation
		self.lanes = set([])
		 
		newGrid, newVelocities = np.zeros((self.N, self.M)), np.zeros((self.N, self.M))
		
		# The logic below considers each lane as a separate 1D TCA. For 2D we first
		# Consider the changes of lanes, and then the traffic flow continues
		
		for j in range(self.M):
			for i in range(self.N):
				# Say a vehicle changes lanes at random if the lane next to it
				# has room (a vacant space) next to it
				
					
				if self.grid[i,j] == True: # Car found
					possShifts = []
					if j - 1 >= 0:
						if self.grid[i,j-1] == False:
							possShifts.append(-1)
					if j + 1 < self.M:
						if self.grid[i,j+1] == False:
							possShifts.append(1)
							
							
					if len(possShifts) > 0 and np.random.rand() < self.pChange:
						shift = np.random.choice(possShifts)
						self.laneChanges[(i, j+shift)] = (i, j)  # Store the prev pos
						self.lanes.add((i, j+shift)) # Hashes in set for speed
						
						self.velocities[i,j+shift] = self.velocities[i,j]
						self.velocities[i,j] = 0
						self.grid[i,j] = 0
						self.grid[i, j+shift] = 1

		# First increase speed with 1 if maxspeed has not been reached
		for j in range(self.M):
			for i in range(self.N):
				if self.grid[i,j] == True and self.velocities[i,j] < self.maxvel:
					self.velocities[i,j] += 1
		
		# Check if any cars in front are within maxspeed distance and slow down
		for k in range(self.M):
			for i in range(self.N):
				for j in range(1, int(self.velocities[i,k]) + 1):
					# Use modulo to implement periodic boundaries
					if self.grid[(i+j) % self.N, k]:
						self.velocities[i, k] = j - 1
						break # Found a grid where a crash could occur
					
		# Randomize speeds/slowdowns
		for j in range(self.M):
			for i in range(self.N):
				if np.random.rand() < self.pSlow and self.grid[i,j] == True \
														and self.velocities[i,j] > 0:
					self.velocities[i,j] -= 1
				
		# Move the cars forward depending on their speed
		for k in range(self.M):
			for i in range(self.N):
				j = int(self.velocities[i,k])
				temp = []  # temporary array
				
				if self.grid[i,k] == True:
					if (i,k) in self.lanes:
						temp.append(self.laneChanges[(i, k)])
					else:
						temp.append((i, k))
						
					temp.append(((i+j) % self.N, k))
					
					newGrid[(i+j) % self.N,k] = 1
					newVelocities[(i+j) % self.N,k] = j
					self.plotState.append(temp)
					self.distances.append(j)
				

		self.velocities = newVelocities
		self.grid = newGrid
		
	def returnAverageVelocity(self):
		return np.sum(self.velocities)/self.cars
	
	def returnPlotState(self):
		return self.plotState, self.distances
			
		
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
			if realSpacePoints[i][0][1] != realSpacePoints[i][1][1]:
				print(realSpacePoints[i][0][1], realSpacePoints[i][1][1])
			
			
		# Run through each of the coordinates and yield a list of x and y plot vals
		
#		print(yPoints)
		
		for i in range(steps-1):
			xList, yList = [], []
			for j in range(len(vels)):
				xList.append(xPoints[j][i])
				yList.append(yPoints[j][i])
			yield xList, yList
			
		n += 1

def animate(i):
	thisx, thisy = [], []

	test.updateGrid()
	points = test.returnGrid()
	for point in points:
		coor = trans[point]
		thisx.append(coor[0])
		thisy.append(coor[1])

		
	line.set_data(thisx, thisy)
	time_text.set_text(time_template.format(i))
	
	return line, time_text

def animate2(i):
	thisx, thisy = next(dataGen)
	ycoordinates.extend(thisy)
	
	line.set_data(thisx, thisy)
	time_text.set_text(time_template.format(i))
	
	
	return line, time_text

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
		
	return list(start)



		
N, M = 40, 3 # Amount of cells needed for the CA
xmin, xmax, ymin, ymax = 0, 10, -0.5, 0.5  # For plotting

# Starting cars

#start = [(0,0), (2,0), (3,0), (6, 0), (5,1)]
start = generateStart(N, M, 1)
# Create a CA object
test = CAtwoD(N, M, start, 0.1, 5, 0.3)
ycoordinates = []

# Find the translations for plotting the grid
coors,dx,dy,trans = findCoors(N, M, xmin, xmax, ymin, ymax)

# These are variables for the plotting stuff
steps = 24
lim = 200
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
	time_template = 'timestep {0}'
	time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
	plt.axis('equal')
	
	
	ani = animation.FuncAnimation(fig, animate2, lim*(steps-1),
										   interval=10, blit=True, init_func=init, repeat=False)
	
	plt.show()
	
	

    
   










