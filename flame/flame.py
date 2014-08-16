import pygame
import numpy as np
import osgeo.gdal


class Fire(object):
    def __init__(self, toa_file, fli_file):
        self.time_of_arrival = osgeo.gdal.Open(toa_file).ReadAsArray()
        self.fire_intensity = osgeo.gdal.Open(fli_file).ReadAsArray()

        print self.time_of_arrival


def run():
    pygame.init()

    screen = pygame.display.set_mode((640, 480))
    clock = pygame.time.Clock()

    fire = Fire("data/ash1_raster.toa", "data/ash1_raster.fli")
    while True:
        clock.tick(60)

        pygame.display.flip()
