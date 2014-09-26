import pygame
import numpy as np

import config
from fire import Fire
from vehicle import Vehicle
from planners.hotspot_planner import HotspotPlanner
from planners.base_planner import BasePlanner
def run(fires_toa, fires_fli, trial_directory):
    pygame.init()
    pygame.display.set_caption("Flame: Fire Simulator")

    fire = Fire(fires_toa, fires_fli)
    planner = HotspotPlanner(fire, trial_directory)

    entities = [
        fire,
        Vehicle(planner)
    ]

    screen = pygame.display.set_mode(fire.shape(), pygame.HWSURFACE | pygame.DOUBLEBUF)
    clock = pygame.time.Clock()
    print planner.is_done()

    # This makes sure the simulation begins at the start of the fire
    simulation_time = np.asarray(fire.time_of_arrival)
    simulation_time = np.sort(simulation_time[simulation_time >= 0])[0]
    print simulation_time

    while simulation_time and planner.is_done() is False:
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
