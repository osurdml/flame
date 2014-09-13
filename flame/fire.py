import osgeo.gdal
import numpy as np
import pygame
import sklearn.cluster

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

    def extract_frontier_map(self):
        # Copy the frontier to a full matrix so we can compare it with the
        # hotspots
        self.frontier_mask = np.zeros_like(self.fire_progression)
        self.frontier_mask[self.frontier] = 1
        
        return self.frontier_mask

    def extract_clusters(self):
        hotspots = np.where(self.fire_progression > self.HOTSPOT_MIN,
                np.ones_like(self.fire_progression),
                np.zeros_like(self.fire_progression))
        hotspots = np.logical_and(hotspots, self.frontier_mask)
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
        self.frontier_map= self.extract_frontier_map()
        self.clusters = self.extract_clusters()

    def draw(self, screen):
        surface = pygame.Surface(self.fire_progression.shape)
        pygame.surfarray.blit_array(surface, (self.fire_progression * 255).astype(np.uint32))

        screen.blit(surface, (0, 0))


