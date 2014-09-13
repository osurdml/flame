import pygame
import numpy as np
import scipy.signal

import config
from fire import Fire
from hotspot_planner import HotspotPlanner
from hotspot_tracker import HotspotTracker
from vehicle import Vehicle
from base_planner import BasePlanner

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
    while simulation_time < config.TIME_TO_RUN:
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

        simulation_time += config.TIME_STEP

        pygame.display.flip()
