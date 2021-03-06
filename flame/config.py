import numpy as np
#Algorithm BASIC,MAX,WEIGHTED
ALGORITHM = "WEIGHTED"

#Max simulation time
MAX_SIM_TIME = 10
# How many simulated time steps to run the simulation for
#TIME_TO_RUN = 20
#Percent cutoff of hotspots
HOTSPOT_MIN = .25

#Time to wait for hotspots to reappear before simulation ends
TIMEOUT = 400

#Planner
ALPHA = 50

# The rate at which to step through the simulation
TIME_STEP = 0.01

VEHICLE_SPEED = 2

# UAV field of view
FOV = 15

# Radius to consider hotspots the same after each iteration
HS_RADIUS = 20

#AStar timeout
ASTAR_TIMEOUT = 1000

#Declare if using ROS or not
USING_ROS = True

#Wait time between loops
DEADLINE = .5

#Initial Vehicle Location
STARTING_LOCATION = np.array([150.0, 150.0])

#Rate at which to send GPS in seconds
UPDATE_RATE = 2
