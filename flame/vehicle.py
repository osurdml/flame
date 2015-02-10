import numpy as np
import pygame
import time
import threading
import config
import roslib
#from mavros.srv import *
#from mavros.msg import *

class Vehicle(object):
    VEHICLE_SPEED = 20 # pixels / sec

    def __init__(self, planner):
        self.planner = planner
        self.location = np.array([200.0, 150.0])
        self.iteration = 0
        #self.send_gps()
    def update(self, simulation_time):
        # Loop so we have per-pixel accuracy, rather than moving
        # VEHICLE_SPEED pixels per simulation tick. This would cause
        # oscillations in and out of the desired distance from the fire
        # line.
        self.iteration += 1 
        print "iteration = %d" % self.iteration
        self.planner.location = self.location
        (plan, replan) = self.planner.plan(simulation_time)

        global vis_plan
        vis_plan = plan

        if plan is not None:
            # Vector is normalized, scale it by the TIME_STEP
            print 'steps'
            for step in plan[0:2]:
                print step
                self.location += step # config.TIME_STEP * step

    def draw(self, screen):
        pygame.draw.circle(screen, (0,255,0), self.location.astype(np.uint), 3)

    #def send_gps(self):
       # gps_location = 0; #may need to move this so it doesn't get set to zero everytime...
        #gps_start = 0; #change for initial gps
        #print"                   GPS GPS GPS GPS GPS"
        #print self.location

        #Waypoint myGPSMsg
        #myGPSMsg.frame = 0
        #myGPSMsg.command = 16
        #myGPSMsg.x_lat = gps_start + self.location[0]*0.00001784864
        #myGPSMsg.y_lat = gps_start + self.location[1]*0.00001784864
        #myGPSMsg.z_alt = gps_start + 10 #need to make this number ~10 feet somehow


        #threading.Timer(1, self.send_gps).start()



