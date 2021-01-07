
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  3 19:01:47 2020

@author: uddin
"""

import numpy as np
import matplotlib.pyplot as plt

#normalize function to normalize position vector later in the calculations
#normalizing means vector divided by sqr root of all the elements in the vector (to get unit vector)
def normalize(r):
    normalizednum = np.linalg.norm(r)
    if normalizednum == 0:
        return r
    else:
        return r/normalizednum

#timestep, simulation time, array containing timesteps
dt = 1
t_max = 100000
t_array = np.arange(0, t_max, dt)

#mass of planet, gravitational constant, initial position, initial velocity
M = 6.42*(10**23)
G = 6.67408*(10**-11)

#circular orbit initial conditions
initpos = np.array([0,3406000,0])
initvel = np.array([3545,0,0])

#create empty list of 0 for position and velocity of the same length as the timesteps
p_list = [0]*len(t_array)
v_list = [0]*len(t_array)

# 1st terms of displacement and velocity
p_list[0] = initpos
v_list[0] = initvel

#2nd terms of displacement and velocity
p_list[1] = p_list[0] + dt*v_list[0]
v_list[1] = (p_list[1] - p_list[0])/dt

#iterating for new position and velocity
for i in range (1,len(t_array)-1):
    
    posunitvector = normalize(p_list[i])
    posmagnitudesqrd = np.linalg.norm(p_list[i])**2

    p_list[i+1] = 2*p_list[i] - p_list[i-1] - (dt**2)*(G*M*posunitvector)/posmagnitudesqrd
    v_list[i+1] = (p_list[i+1] - p_list[i])/(dt)

#convert lists to array
p_array = np.array(p_list)
v_array = np.array(v_list)

#create empty arrays for x and y coordinates
x_array = np.zeros(len(p_array))
y_array = np.zeros(len(p_array))

#iterate to insert all x and y coordinates from the position array into the x and y arrays
for j in range (len(p_array)):
    x_array[j] = p_array[j][0]
    y_array[j] = p_array[j][1]


#plots trajectory in the orbital plane
plt.figure(1)
plt.clf()
plt.xlabel('x')
plt.ylabel('y')
plt.grid()
plt.plot(x_array, y_array, label='path')
plt.legend()
plt.show()

