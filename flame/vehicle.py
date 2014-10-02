import numpy as np
import pygame

import config

class Vehicle(object):
    VEHICLE_SPEED = 20 # pixels / sec

    def __init__(self, planner):
        self.planner = planner
        self.location = np.array([200.0, 150.0])
        self.iteration = 0
    def update(self, simulation_time):
        # Loop so we have per-pixel accuracy, rather than moving
        # VEHICLE_SPEED pixels per simulation tick. This would cause
        # oscillations in and out of the desired distance from the fire
        # line.
        self.iteration += 1 
        print "iteration = %d" % self.iteration
        self.planner.location = self.location
        (plan, replan) = self.planner.plan(simulation_time)
        vis_plan = plan

        if plan is not None:
            # Vector is normalized, scale it by the TIME_STEP
            print 'steps'
            for step in plan[0:2]:
                print step
                self.location += step # config.TIME_STEP * step
                
    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), self.location.astype(np.uint), 3)


