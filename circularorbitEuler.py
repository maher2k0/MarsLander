# uncomment the next line if running in a notebook
# %matplotlib inline
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

# mass, grav. constant,initial position and velocity
M = 6.42*(10**23)
G = 6.67408*(10**-11)

initpos = np.array([0,10**7,0])
initvel = np.array([2069.93,0,0])

# simulation time, timestep and time
t_max = 100000
dt = 1
t_array = np.arange(0, t_max, dt)

#create empty list of 0 for position and velocity of the same length as the timesteps
p_list = []
v_list = []


# Euler integration
for t in t_array:

    # append current state to trajectories
    p_list.append(initpos)
    v_list.append(initvel)

    posunitvector = normalize(initpos)
    posmagnitudesqrd = np.linalg.norm(initpos)**2
    a = -(G*M*posunitvector)/(posmagnitudesqrd)    

    # calculate new position and velocity
    initpos = initpos + dt * initvel
    initvel = initvel + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
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