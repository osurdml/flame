import pygame
import numpy as np
import osgeo.gdal

TIME_STEP = 0.1

class Fire(object):
    def __init__(self, toa_file, fli_file):
        self.time_of_arrival = osgeo.gdal.Open(toa_file).ReadAsArray()
        self.fire_intensity = osgeo.gdal.Open(fli_file).ReadAsArray()

        # Normalize intensity values to [0,1]
        self.fire_intensity[self.fire_intensity < 0] = 0
        self.fire_intensity = self.fire_intensity / np.amax(self.fire_intensity)

    def shape(self):
        return self.time_of_arrival.shape

    def update(self, simulation_time):
        self.fire_progression = np.where(self.time_of_arrival <= simulation_time,
                self.fire_intensity * 255, np.zeros_like(self.fire_intensity))

    def draw(self, screen):
        surface = pygame.Surface(self.fire_progression.shape)
        pygame.surfarray.blit_array(surface, self.fire_progression.astype(np.uint32))

        screen.blit(surface, (0, 0))

class Vehicle(object):
    VEHICLE_SPEED = 10 # pixels / sec

    def __init__(self):
        self.location = np.array([50.0, 50.0])

    def int_location(self):
        return (int(round(self.location[0])), int(round(self.location[1])))

    def update(self, simulation_time):
        self.location += self.VEHICLE_SPEED * TIME_STEP

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), self.int_location(), 3)

def run():
    pygame.init()
    pygame.display.set_caption("Flame: Fire Simulator")

    fire = Fire("data/ash1_raster.toa", "data/ash1_raster.fli")
    entities = [
        fire,
        Vehicle()
    ]

    screen = pygame.display.set_mode(fire.shape())
    clock = pygame.time.Clock()

    simulation_time = 0
    while True:
        clock.tick(60)

        for entity in entities:
            entity.update(simulation_time)
            entity.draw(screen)

        simulation_time += TIME_STEP

        pygame.display.flip()
