import numpy as np
import scipy.signal
import astar
from base_planner import BasePlanner
from math import sqrt
from .. import config
class HotspotPlanner(object):
    def __init__(self, fire):
        self.time_untracked = [0]
        self.max_untracked = 0
        self.sub_planner = BasePlanner(fire)
        self.previous_hs = [[0, 0]]
        self.tracker = HotspotTracker(fire, self.time_untracked, self.max_untracked, self.previous_hs)
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
            ds, xs, ys, self.time_untracked, self.max_untracked, self.previous_hs = self.tracker.update()

            nearest = np.argmin(self.tracker.hs_dists)

            # Draw a vector to that point
            vec_to_nearest = np.array([self.tracker.xs[nearest] - self.location[0],
                                       self.tracker.ys[nearest] - self.location[1]])
            dist_to_nearest = np.linalg.norm(vec_to_nearest)
            vec_to_nearest = vec_to_nearest / dist_to_nearest

            self.sub_planner.direction = BasePlanner.DIRECTION_CW

            print "\n------------------------------\n"
#            if self.time_untracked % 5 == 0:

            # TODO: Grow the obstacle map so the planner doesn't get stuck in an
            # expanding fire. This causes the path planner to stall.
            obstacle_map = scipy.signal.convolve2d(self.fire.fire_progression, [[1, 1, 1], [1, 1, 1], [1, 1, 1]])
            obstacle_map = np.where(self.fire.fire_progression, 1000, 1)
            graph, nodes = astar.make_graph(obstacle_map)
            #cost_map = scipy.signal.convolve2d(self.fire.fire_progression, [[1, 1, 1], [1, 1, 1], [1, 1, 1]])
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

                return (path_arr, False)

            ## Figure out if path is CW or CCW
            #avg_x, avg_y = 0.0, 0.0
            #for n in path:
            #    avg_x += n.x
            #    avg_y += n.y
            #avg_x, avg_y = avg_x / len(path), avg_y / len(path)

            #if avg_x > self.location[0]:
            #    if avg_y > self.location[1]:
            #        self.sub_planner.direction = BasePlanner.DIRECTION_CW
            #    else:
            #        self.sub_planner.direction = BasePlanner.DIRECTION_CCW
            #else:
            #    if avg_y > self.location[1]:
            #        self.sub_planner.direction = BasePlanner.DIRECTION_CCW
            #    else:
            #        self.sub_planner.direction = BasePlanner.DIRECTION_CW

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
            print "test"
        return (self.sub_planner.plan(simulation_time), True)

class Hotspot(object):
    def __init__(self, x, y):
        self.location = [x, y]
        self.time_untracked = 0

class HotspotTracker(object):
    def __init__(self, fire, time_untracked, max_untracked, previous_hs):
        self.fire = fire
        self.time_untracked = time_untracked
        self.max_untracked = max_untracked
        self.previous_hs = previous_hs
        self.H = {}
        self.id_next= 0

    def set_location(self, location):
        self.location = location

    def distance_calc(self, a, b):
        dist =  sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        return dist

    def find_old(self, nh, H):
        ret = {}
        for h,v in H:
            print h
            if self.distance_calc(nh, h) < 20:
                ret.add(h)
        if len(ret) < 2:
            return ret.get(0)

    def update(self):
        if self.fire.clusters.any():
            (self.xs, self.ys) = self.fire.clusters.T

            H = {}
            H_tmp = {}

            for i in range(len(self.fire.clusters)):
                oh = self.find_old(self.fire.clusters[i], self.previous_hs)
                if oh is not None:
                    H_tmp[oh] = H[oh]
                else:
                    new_hs = Hotspot(self.fire.clusters[i][0], self.fire.clusters[i][1])
                    H_tmp[id_next] = new_hs
                    id_next += 1
                for oh in H:
                    H_tmp[oh].time_untracked += 1
                    if self.max_untracked < H_tmp[oh]:
                        self.max_untracked = H_tmp[oh] 
                    if distance_calc(self.fire.clusters[i], self.location) < 25:
                        H_tmp[oh] = 0
                H = H_tmp
            self.hs_dists = (self.xs - self.location[0]) ** 2 + (self.ys - self.location[1])**2

            # calculate distances between hotspot[i] and all previous
            # hotspots[i]. If distance < setpoint then time_untracked[array location of
            # previous hs] +=1 else append time_untracked =1

           # for i in range(len(self.time_untracked)):
           #     if i>= len(self.time_untracked):
           #         self.time_untracked.append(0)

           #     dist = sqrt((self.fire.clusters[i][0] - self.location[0])**2 + (self.fire.clusters[i][1] - self.location[1])**2)
           #     self.time_untracked[i] = self.time_untracked[i] + 1
           #     if dist < 50:
           #         self.time_untracked[i]= 0
           #     print self.time_untracked

           #     if self.max_untracked < self.time_untracked[i]:
           #         self.max_untracked = self.time_untracked[i]
           #         print self.max_untracked
            #self.hs_dists = (self.xs - previous_hs.location[0]) ** 2 + (self.y - previous.location[1]) ** 2


            self.previous_hs= self.fire.clusters
            return self.hs_dists, self.xs, self.ys, self.time_untracked, self.max_untracked, self.previous_hs
