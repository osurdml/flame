from BasePlanner import BasePlanner
from HotspotTracker import HotspotTracker
import numpy as np
from AStar import AStar

class HotspotPlanner(object):
    def __init__(self, fire):
        self.sub_planner = BasePlanner(fire)
        self.fire = fire

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, location):
        self._location = location
        self.sub_planner.location = location

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
            

            self.sub_planner.direction = BasePlanner.DIRECTION_CW
            graph= self.fire.frontier
            AStar(self, graph, self.location, self.fire.clusters.centers)

        else:
            self.sub_planner.direction = BasePlanner.DIRECTION_CCW
        return self.sub_planner.plan(simulation_time)


