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

initpos = np.array([0,5*10**6,0])
initvel = np.array([0,0,0])

# simulation time, timestep and time
t_max = 900
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


alt_array = np.zeros(len(p_array))
for j in range(len(p_array)):
    alt_array[j] = p_array[j][1]

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.ylabel('altitude (m)')
plt.grid()
plt.plot(t_array, alt_array, label='trajectory')

plt.legend()
plt.show()
