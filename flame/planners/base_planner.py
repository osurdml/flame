import numpy as np

from .. import config

class BasePlanner(object):
    DIRECTION_CW = 1
    DIRECTION_CCW = 2

    def __init__(self, fire):
        self.fire = fire
        self.direction = self.DIRECTION_CW

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, location):
        self._location = location

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
            if dist_to_nearest > 20:
                travel_vec = vec_to_nearest
            elif dist_to_nearest < 16:
                travel_vec = -vec_to_nearest
            else:
                travel_vec = np.array([[-vec_to_nearest[1], vec_to_nearest[0]]])

                if self.direction == self.DIRECTION_CW:
                    travel_vec = -travel_vec
            return np.tile(travel_vec, config.VEHICLE_SPEED).reshape((-1,2))

        return None


