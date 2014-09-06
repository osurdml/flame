import numpy as np
import pygame
from Config import Config

class Vehicle(object):
    VEHICLE_SPEED = 200 # pixels / sec

    def __init__(self, planner):
        self.planner = planner
        self.location = np.array([100.0, 100.0])

    def update(self, simulation_time):
        # Loop so we have per-pixel accuracy, rather than moving
        # VEHICLE_SPEED pixels per simulation tick. This would cause
        # oscillations in and out of the desired distance from the fire
        # line.
        for _ in range(0, self.VEHICLE_SPEED):
            self.planner.location = self.location
            plan = self.planner.plan(simulation_time)

            if plan is not None:
                # Vector is normalized, scale it by the TIME_STEP
                self.location += Config.TIME_STEP * plan

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), self.location.astype(np.uint), 3)


