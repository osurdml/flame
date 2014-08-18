import pygame
import osgeo.gdal

import numpy as np
import scipy.signal
import sklearn.cluster

# The rate at which to step through the simulation
TIME_STEP = 0.05

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
        frontier = self.fire_progression > 0
        frontier = scipy.signal.convolve2d(frontier, [[1, 1, 1], [1, 0, 1], [1, 1, 1]], mode='same')
        frontier = np.logical_and(frontier < 8, frontier > 0).nonzero()

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
            kmeans = sklearn.cluster.KMeans(n_clusters=2)
            kmeans.fit(hotspots)

            return kmeans.cluster_centers_

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

class Vehicle(object):
    VEHICLE_SPEED = 10 # pixels / sec

    def __init__(self):
        self.location = np.array([50.0, 50.0])

    def update(self, simulation_time):
        self.location += self.VEHICLE_SPEED * TIME_STEP

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), self.location.astype(np.uint), 3)

def run():
    pygame.init()
    pygame.display.set_caption("Flame: Fire Simulator")

    fire = Fire("data/ash1_raster.toa", "data/ash1_raster.fli")
    entities = [
        fire,
        Vehicle()
    ]

    screen = pygame.display.set_mode(fire.shape(), pygame.HWSURFACE | pygame.DOUBLEBUF)
    clock = pygame.time.Clock()

    simulation_time = 0
    while True:
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
