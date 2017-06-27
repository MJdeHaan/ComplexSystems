# -*- coding: utf-8 -*-
"""
Created on Thu Jun 22 13:47:50 2017

@author: Mike
File containing functions which can gather and plot statistics of the CA models
"""

import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import itertools

from NS_Functions import *


def returnCAStatistics(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	'''
	Take all the statistics the system can output and store those for each iteration
	The first no. of results are ignored as the system is assumed to go into
	a state where initial conditions dont matter as time progresses
	'''
	avSpeeds, fluxes, jamLengths = [], [], []
	movements = []
	
	start = generateStart(N, M, carnum, maxVel, pChange*np.ones(carnum))
	model = CAtwoD(N, M, start, pSlow, maxVel, pChange, strategy)
	
	for i in range(iterations):
		model.updateGrid()
		if i == warmup:
			model.fluxCounter = 0  # warm up of system is over
		avSpeeds.append(model.returnAverageVelocity())
		jamLengths.append(model.returnJamStatistics())
		fluxes.append(model.fluxCounter)
		movements.append(model.returnMovementCounter())

	return avSpeeds[warmup:], fluxes[warmup:], jamLengths[warmup:], movements[warmup:]


def plotAvSpeedsOverTime(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup)
	
	plt.plot(avSpeeds)
	plt.xlabel('Iteration')
	plt.ylabel('$<v>$')
	plt.title(r'$N = {0}$, $M = {1}$, $n_c = {2}$, $p_c={3}$, $p_s={4}$'.format(N,M,carnum,pChange,pSlow))
	plt.show()
	
	
def plotFluxOverTime(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup)
	
	temp = range(warmup, iterations)
	for i in range(len(fluxes)):
		fluxes[i] = fluxes[i] / temp[i]
	plt.plot(fluxes)
	plt.xlabel('Iteration')
	plt.ylabel('$Flux$')
	plt.title(r'$N = {0}$, $M = {1}$, $n_c = {2}$, $p_c={3}$, $p_s={4}$'.format(N,M,carnum,pChange,pSlow))
	plt.show()
	
def plotLongestJamOverTime(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup)
	
	for i in range(len(jamLengths)):
		if len(jamLengths[i]) > 0:
			jamLengths[i] = np.max(jamLengths[i])
		else:
			jamLengths[i] = 0
			
		
	plt.plot(jamLengths)
	plt.xlabel('Iteration')
	plt.ylabel('$Longest jam$')
	plt.title(r'$N = {0}$, $M = {1}$, $n_c = {2}$, $p_c={3}$, $p_s={4}$'.format(N,M,carnum,pChange,pSlow))
	plt.show()
	
def plotPchangeToAvFlux(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	fluxProb = []
	for i in np.linspace(0.0, 1.0, 40):
		print(i)
		avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, i, pSlow, maxVel, strategy, iterations, warmup)
		fluxProb.append(fluxes[-1]/(iterations-warmup))
		
	plt.plot(np.linspace(0.0, 1.0, 40), fluxProb)
	plt.xlabel('$p_c$')
	plt.ylabel('flux')
	plt.show()
		
def plotPchangeToAvSpeed(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	fluxProb = []
	for i in np.linspace(0.0, 1.0, 40):
		print(i)
		avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, i, pSlow, maxVel, strategy, iterations, warmup)
		fluxProb.append(np.mean(avSpeeds))
		
	plt.plot(np.linspace(0.0, 1.0, 40), fluxProb)
	plt.xlabel('$p_c$')
	plt.ylabel('$<v>$')
	plt.show()
	
	
def plotPchangeToAverages(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	fluxProb = []
	jamProb = []
	speedProb = []
	
	for j in np.linspace(0.0, 1.0, 40):
		print(j)
		avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, carnum, j, pSlow, maxVel, strategy, iterations, warmup)
		# Jams
		for i in range(len(jamLengths)):
			if len(jamLengths[i]) > 0:
				jamLengths[i] = np.max(jamLengths[i])
			else:
				jamLengths[i] = 0
		jamProb.append(np.mean(jamLengths))
		# Avspeeds
		speedProb.append(np.mean(avSpeeds))
		fluxProb.append(fluxes[-1]/(iterations-warmup))
		
	return np.linspace(0.0, 1.0, 40), fluxProb, jamProb, speedProb

	
def plotDensityToAverages(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup):
	fluxProb = []
	jamProb = []
	speedProb = []
	densities = []
	
	for j in range(1, carnum, 5):
		print(j/(N*M))
		densities.append(j/(N*M))
		avSpeeds, fluxes, jamLengths = returnCAStatistics(N, M, j, pChange, pSlow, maxVel, strategy, iterations, warmup)
		# Jams
		for i in range(len(jamLengths)):
			if len(jamLengths[i]) > 0:
				jamLengths[i] = np.max(jamLengths[i])
			else:
				jamLengths[i] = 0
		jamProb.append(np.mean(jamLengths))
		# Avspeeds
		speedProb.append(np.mean(avSpeeds))
		fluxProb.append(fluxes[-1]/(iterations-warmup))
		
	return densities, fluxProb, jamProb, speedProb
	
#plotAvSpeedsOverTime(40, 2, 20, 0.15, 0.1, 5, 'mike', 1500, 200)
#
#plotFluxOverTime(40, 2, 20, 0.15, 0.1, 5, 'mike', 6000, 200)

#plotPchangeToAvFlux(40, 2, 20, 0.1, 0.1, 5, 'random', 4000, 200)
#plotPchangeToLongestJamAverage(40, 2, 20, 0.1, 0.1, 5, 'mike', 3000, 200)
#probsEuro, fluxEuro, jamEuro, speedEuro = plotDensityToAverages(50, 4, 180, 0.1, 0.1, 5, 'euro', 15000, 500)
#probsSymm, fluxSymm, jamSymm, speedSymm = plotDensityToAverages(50, 4, 180, 0.1, 0.1, 5, 'symmetric', 15000, 500)
#probsRand, fluxRand, jamRand, speedRand = plotDensityToAverages(50, 4, 180, 0.1, 0.1, 5, 'random', 15000, 500)


def findStdDev(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup, step):
	stdDevs, rho = [], []
	for i in range(1, carnum+1, step):
		print(i/(N*M))
		speed, fluxes, jamLengths, movements = returnCAStatistics(N, M, i, pChange, pSlow, maxVel, strategy, iterations, warmup)
		rho.append(i/(N*M))
		stdDevs.append(np.std(np.array(movements)))
		
	return rho, stdDevs


'''
Gather the data for the symmetric stuff
take pSlow  = 0.1, maxvel = 5
'''

N, M, carnum = 50, 2, 100
pChange, pSlow = 0.1, 0.1
maxVel = 5
strategy = 'euro'
iterations = 2000
warmup = 500


#speed, fluxes, jamLengths, movements = returnCAStatistics(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup)
#
#movements = np.array(movements)
#
#
#plt.plot(movements)
#
rho, std = findStdDev(N, M, carnum, pChange, pSlow, maxVel, strategy, iterations, warmup, 1)
rho2, std2 = findStdDev(N, 4, 200, pChange, pSlow, maxVel, strategy, iterations, warmup, 2)
rho3, std3 = findStdDev(N, 8, 400, pChange, pSlow, maxVel, strategy, iterations, warmup, 4)

rho4, std4 = findStdDev(N, M, carnum, 0.3, pSlow, maxVel, strategy, iterations, warmup, 1)
rho5, std5 = findStdDev(N, 4, 200, 0.3, pSlow, maxVel, strategy, iterations, warmup, 2)
rho6, std6 = findStdDev(N, 8, 400, 0.3, pSlow, maxVel, strategy, iterations, warmup, 4)


plt.plot(rho, std, '>-', label='2 lanes, $p_c=0.1$')
plt.plot(rho2, std2, '>-', label='4 lanes, $p_c=0.1$')
plt.plot(rho3, std3, '>-', label='8 lanes, $p_c=0.1$')

plt.plot(rho4, std4, '>-', label='2 lanes, $p_c=0.3$')
plt.plot(rho5, std6, '>-', label='4 lanes, $p_c=0.3$')
plt.plot(rho6, std6, '>-', label='8 lanes, $p_c=0.3$')

plt.xlabel(r'$\rho$')
plt.ylabel(r'$\sigma_{move}$')
plt.title('$N=50$, Symmetric')
plt.legend()

plt.show()
#plt.show()
## 4 Lane stuff
#probsSymm1, fluxSymm1, jamSymm1, speedSymm1 = plotDensityToAverages(70, 4, 280, 0.1, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm2, fluxSymm2, jamSymm2, speedSymm2 = plotDensityToAverages(70, 4, 280, 0.3, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm3, fluxSymm3, jamSymm3, speedSymm3 = plotDensityToAverages(70, 4, 280, 0.5, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm4, fluxSymm4, jamSymm4, speedSymm4 = plotDensityToAverages(70, 4, 280, 0.7, 0.1, 5, 'symmetric', 15000, 500)
#
#
## 2 Lane stuff
#probsSymm5, fluxSymm5, jamSymm5, speedSymm5 = plotDensityToAverages(140, 2, 280, 0.1, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm6, fluxSymm6, jamSymm6, speedSymm6 = plotDensityToAverages(140, 2, 280, 0.3, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm7, fluxSymm7, jamSymm7, speedSymm7 = plotDensityToAverages(140, 2, 280, 0.5, 0.1, 5, 'symmetric', 15000, 500)
#probsSymm8, fluxSymm8, jamSymm8, speedSymm8 = plotDensityToAverages(140, 2, 280, 0.7, 0.1, 5, 'symmetric', 15000, 500)
#
#
#
#
#
#
#probsRand1, fluxRand1, jamRand1, speedRand1 = plotDensityToAverages(70, 4, 280, 0.1, 0.1, 5, 'random', 15000, 500)
#probsRand2, fluxRand2, jamRand2, speedRand2 = plotDensityToAverages(70, 4, 280, 0.3, 0.1, 5, 'random', 15000, 500)
#probsRand3, fluxRand3, jamRand3, speedRand3 = plotDensityToAverages(70, 4, 280, 0.5, 0.1, 5, 'random', 15000, 500)
#probsRand4, fluxRand4, jamRand4, speedRand4 = plotDensityToAverages(70, 4, 280, 0.7, 0.1, 5, 'random', 15000, 500)
#
#
## 2 Lane stuff
#probsRand5, fluxRand5, jamRand5, speedRand5 = plotDensityToAverages(140, 2, 280, 0.1, 0.1, 5, 'random', 15000, 500)
#probsRand6, fluxRand6, jamRand6, speedRand6 = plotDensityToAverages(140, 2, 280, 0.3, 0.1, 5, 'random', 15000, 500)
#probsRand7, fluxRand7, jamRand7, speedRand7 = plotDensityToAverages(140, 2, 280, 0.5, 0.1, 5, 'random', 15000, 500)
#probsRand8, fluxRand8, jamRand8, speedRand8 = plotDensityToAverages(140, 2, 280, 0.7, 0.1, 5, 'random', 15000, 500)
#
#
#


#
#plt.subplot(311)
#
#plt.plot(probsSymm, jamEuro, '-o', label='European')
#plt.plot(probsSymm, jamSymm, '-o', label='Symmetric')
#plt.plot(probsSymm, jamRand, '-o', label='Random')
#
#plt.xlabel('$p_c$')
#plt.ylabel('<Longest jam>')
#
#plt.subplot(312)
#
#plt.plot(probsSymm, speedEuro, '-o', label='European')
#plt.plot(probsSymm, speedSymm, '-o', label='Symmetric')
#plt.plot(probsSymm, speedRand, '-o', label='Random')
#
#
#plt.xlabel('$p_c$')
#plt.ylabel('<v>')
#
#plt.subplot(313)
#
#plt.plot(probsSymm, fluxEuro, '-o', label='European')
#plt.plot(probsSymm, fluxSymm, '-o', label='Symmetric')
#plt.plot(probsSymm, fluxRand, '-o', label='Random')
#
#
#plt.xlabel('$p_c$')
#plt.ylabel('<flux>')
#
#
#
#plt.legend()
#plt.show()

#plotLongestJamOverTime(40, 2, 30, 0.15, 0.1, 5, 'mike', 6000, 200)
	




