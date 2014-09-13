import numpy as np

from astar import AStar
from base_planner import BasePlanner
from hotspot_tracker import HotspotTracker

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
            obstacle_map= np.where(self.fire.frontier_map<1,
                    self.fire.frontier_map,
                    1)

            
            path=AStar(obstacle_map, 4, int(self.location[0]), int(self.location[1]), int(self.fire.clusters[0][0]), int(self.fire.clusters[0][1]))

            #print path
        else:
            self.sub_planner.direction = BasePlanner.DIRECTION_CCW

        return self.sub_planner.plan(simulation_time)


