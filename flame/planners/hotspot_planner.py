import numpy as np
import scipy.signal

import astar
from base_planner import BasePlanner

class HotspotPlanner(object):
    def __init__(self, fire):
        self.sub_planner = BasePlanner(fire)
        self.tracker = HotspotTracker(fire)
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
            self.tracker.set_location(self.location)
            self.tracker.update()

            nearest = np.argmin(self.tracker.hs_dists)

            # Draw a vector to that point
            vec_to_nearest = np.array([self.tracker.xs[nearest] - self.location[0],
                                       self.tracker.ys[nearest] - self.location[1]])
            dist_to_nearest = np.linalg.norm(vec_to_nearest)
            vec_to_nearest = vec_to_nearest / dist_to_nearest

            self.sub_planner.direction = BasePlanner.DIRECTION_CW
            # TODO: Grow the obstacle map so the planner doesn't get stuck in an
            # expanding fire. This causes the path planner to stall.
            # obstacle_map = scipy.signal.convolve2d(self.fire.fire_progression,
            #         [[1, 1, 1], [1, 1, 1], [1, 1, 1]])
            obstacle_map = np.where(self.fire.fire_progression, 1000, 1)

            graph, nodes = astar.make_graph(obstacle_map)
            paths = astar.AStarGrid(graph)
            start = nodes[int(self.location[0])][int(self.location[1])]
            end = nodes[int(self.fire.clusters[0][0])][int(self.fire.clusters[0][1])]
            path = paths.search(start, end)

            path_arr = []
            last_n = (int(self.location[0]), int(self.location[1]))
            for n in path[1:]:
                dx, dy = n.x - last_n[0], n.y - last_n[1]
                path_arr.append(np.array([dx, dy]))

                last_n = (n.x, n.y)

            # return (path_arr, False)

            # Figure out if path is CW or CCW
            avg_x, avg_y = 0.0, 0.0
            for n in path:
                avg_x += n.x
                avg_y += n.y
            avg_x, avg_y = avg_x / len(path), avg_y / len(path)

            if avg_x > self.location[0]:
                if avg_y > self.location[1]:
                    self.sub_planner.direction = BasePlanner.DIRECTION_CW
                else:
                    self.sub_planner.direction = BasePlanner.DIRECTION_CCW
            else:
                if avg_y > self.location[1]:
                    self.sub_planner.direction = BasePlanner.DIRECTION_CCW
                else:
                    self.sub_planner.direction = BasePlanner.DIRECTION_CW

            
            # path = AStar(obstacle_map, 8,
            #         int(self.location[0]), int(self.location[1]),
            #         int(self.fire.clusters[0][0]), int(self.fire.clusters[0][1]))
            # path_arr = []
            # for d in path:
            #     dxs = [1, 1, 0, -1, -1, -1, 0, 1]
            #     dys = [0, 1, 1, 1, 0, -1, -1, -1]
            #
            #     dx = dxs[int(d)];
            #     dy = dys[int(d)];
            #
            #     path_arr.append(np.array([dx, dy]))
            #
            # print path_arr
            # return (path_arr, False)

        else:
            self.sub_planner.direction = BasePlanner.DIRECTION_CCW

        return (self.sub_planner.plan(simulation_time), True)

class HotspotTracker(object):
    def __init__(self, fire):
        self.fire = fire

    def set_location(self, location):
        self.location = location

    def update(self):
        if self.fire.clusters.any():
            #print self.fire.clusters
            (self.xs, self.ys) = self.fire.clusters.T
            self.hs_dists = (self.xs - self.location[0]) ** 2 + (self.ys - self.location[1]) ** 2


            #self.hs_dists = (self.xs - previous_hs.location[0]) ** 2 + (self.y - previous.location[1]) ** 2


            #previous_hs= self.fire.clusters.T

            #print hs_dists
            return self.hs_dists, self.xs, self.ys 


