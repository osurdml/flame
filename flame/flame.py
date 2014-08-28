import pygame
import osgeo.gdal

import numpy as np
import scipy.signal
import sklearn.cluster

# How many simulated time steps to run the simulation for
TIME_TO_RUN = 20

# The rate at which to step through the simulation
TIME_STEP = 0.005

class Fire(object):
    # Minimum normalized value for a point to be considered a hotspot
    HOTSPOT_MIN = 0.6

    def __init__(self, toa_file, fli_file):
        self.time_of_arrival = osgeo.gdal.Open(toa_file).ReadAsArray()
        self.fire_intensity = osgeo.gdal.Open(fli_file).ReadAsArray()

        # Normalize intensity values to [0,1]
        self.fire_intensity[self.fire_intensity < 0] = 0
        self.fire_intensity = self.fire_intensity / np.amax(self.fire_intensity)

    def shape(self):
        return self.time_of_arrival.shape

    def extract_frontier(self):
        # Find pixels that have at least one fire pixel bordering them, but are
        # not entirely surrounded
        frontier = (self.fire_progression > 0).astype(np.uint8)

        # Fast convolve2d
        convolve_row = frontier[1:-1,:] + frontier[:-2,:] + frontier[2:,:]
        convolve_col = convolve_row[:,1:-1] + convolve_row[:,:-2] + convolve_row[:,2:]

        frontier = np.logical_and(convolve_col < 8, convolve_col > 0).nonzero()

        return frontier

    def extract_clusters(self):
        # Copy the frontier to a full matrix so we can compare it with the
        # hotspots
        frontier_mask = np.zeros_like(self.fire_progression)
        frontier_mask[self.frontier] = 1

        hotspots = np.where(self.fire_progression > self.HOTSPOT_MIN,
                np.ones_like(self.fire_progression),
                np.zeros_like(self.fire_progression))
        hotspots = np.logical_and(hotspots, frontier_mask)
        hotspots = np.asarray(np.nonzero(hotspots)).T

        if hotspots.shape[0] > 2:
	    k= int(np.sqrt(hotspots.shape[0]/2))
            self.kmeans = sklearn.cluster.KMeans(n_clusters=k)
            self.kmeans.fit(hotspots)

            return self.kmeans.cluster_centers_
	
        return np.empty((0, 2))

    def update(self, simulation_time):
        self.fire_progression = np.where(self.time_of_arrival <= simulation_time,
                self.fire_intensity, np.zeros_like(self.fire_intensity))
        self.frontier = self.extract_frontier()
        self.clusters = self.extract_clusters()

    def draw(self, screen):
        surface = pygame.Surface(self.fire_progression.shape)
        pygame.surfarray.blit_array(surface, (self.fire_progression * 255).astype(np.uint32))

        screen.blit(surface, (0, 0))

class BasePlanner(object):
    DIRECTION_CW = 1
    DIRECTION_CCW = 2

    def __init__(self, fire):
        self.fire = fire
        self.direction = self.DIRECTION_CW

    def set_location(self, location):
        self.location = location

    def plan(self, simulation_time):
        if self.fire.frontier[0].size > 0:
            # Find the nearest point on the fire frontier
            (xs, ys) = self.fire.frontier
            dists = (xs - self.location[0]) ** 2 + (ys - self.location[1]) ** 2
            nearest = np.argmin(dists)

            # Draw a vector to that point
            vec_to_nearest = np.array([xs[nearest] - self.location[0], ys[nearest] - self.location[1]])
            dist_to_nearest = np.linalg.norm(vec_to_nearest)
            vec_to_nearest = vec_to_nearest / dist_to_nearest

            # Travel towards the fire, away from it, or along it depending
            # on how far the vehicle is from the fire line.
            travel_vec = None
            if dist_to_nearest > 15:
                travel_vec = vec_to_nearest
            elif dist_to_nearest < 11:
                travel_vec = -vec_to_nearest
            else:
                travel_vec = np.array([-vec_to_nearest[1], vec_to_nearest[0]])

                if self.direction == self.DIRECTION_CW:
                    travel_vec = -travel_vec

            return travel_vec

        return None

class HotspotPlanner(object):
    def __init__(self, fire):
        self.sub_planner = BasePlanner(fire)
        self.fire = fire

    def set_location(self, location):
        self.location = location
        self.sub_planner.set_location(location)

    def plan(self, simulation_time):
        if self.fire.clusters.any():
            test = HotspotTracker(self.fire)
            test.set_location(self.location)
            test.update()
            nearest = np.argmin(test.hs_dists)

            # Draw a vector to that point
            vec_to_nearest = np.array([test.xs[nearest] - self.location[0],
                test.ys[nearest] - self.location[1]])
            dist_to_nearest = np.linalg.norm(vec_to_nearest)
            vec_to_nearest = vec_to_nearest / dist_to_nearest

        else:
                self.sub_planner.direction = BasePlanner.DIRECTION_CCW
        return self.sub_planner.plan(simulation_time)

class HotspotTracker(object):
    def __init__(self, fire):
        self.fire = fire
        self.hs_dists = None
        self.xs = None
        self.ys = None
    def set_location(self, location):
        self.location = location

    def update(self):
        if self.fire.clusters.any():
            #print self.fire.clusters
            (self.xs, self.ys) = self.fire.clusters.T
            self.hs_dists = (self.xs - self.location[0]) ** 2 + (self.ys - self.location[1]) ** 2
            #print hs_dists
            return self.hs_dists, self.xs, self.ys
        

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
            self.planner.set_location(self.location)
            plan = self.planner.plan(simulation_time)

            if plan is not None:
                # Vector is normalized, scale it by the TIME_STEP
                self.location += TIME_STEP * plan

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), self.location.astype(np.uint), 3)

def run():
    pygame.init()
    pygame.display.set_caption("Flame: Fire Simulator")

    fire = Fire("data/ash1_raster.toa", "data/ash1_raster.fli")
    planner = HotspotPlanner(fire)

    entities = [
        fire,
        Vehicle(planner)
    ]

    screen = pygame.display.set_mode(fire.shape(), pygame.HWSURFACE | pygame.DOUBLEBUF)
    clock = pygame.time.Clock()

    simulation_time = 0
    while simulation_time < TIME_TO_RUN:
        # clock.tick(100)

        for entity in entities:
            entity.update(simulation_time)
            entity.draw(screen)

        # Draw frontier on alpha surface and blit it to the screen
        frontier_surf = pygame.Surface(fire.shape(), pygame.SRCALPHA)
        frontier = np.asarray(fire.frontier)
        for (x, y) in frontier.T:
            frontier_surf.set_at((x, y), (0, 255, 0, 20))
        screen.blit(frontier_surf, (0, 0))

        # Draw clusters on alpha surface and blit it to the screen
        cluster_surf = pygame.Surface(fire.shape(), pygame.SRCALPHA)
        clusters = fire.clusters.astype(np.uint)
        if clusters.any():
            for (x, y) in clusters:
                pygame.draw.circle(cluster_surf, (255, 0, 0, 128), (x, y), 6)
        screen.blit(cluster_surf, (0, 0))

        simulation_time += TIME_STEP

        pygame.display.flip()
