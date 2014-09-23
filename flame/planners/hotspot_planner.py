import numpy as np
import scipy.signal
import astar
from base_planner import BasePlanner
from math import sqrt
from .. import config
import cv2
class HotspotPlanner(object):
    def __init__(self, fire):
        self.time_untracked = [0]
        self.sub_planner = BasePlanner(fire)
        self.previous_hs = {}
        self.dead_hs = {}
        self.tracker = HotspotTracker()
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
            self.previous_hs, dead_hs = self.tracker.update(self.fire, self.previous_hs, self.location)
            for h_id,h in dead_hs.items():
                self.dead_hs[h_id] = h
            print "Hotspots:"
            for h_id,h in self.previous_hs.items():
                print "%d: (%d, %d) dist %d, untracked for %d" % (h_id, h.location[0], h.location[1], h.dist, h.time)
            print "Max untracked %d" % (self.tracker.max_untracked)
            print "Dead hotspot IDs:", [h_id for h_id,h in self.dead_hs.items()]

            # Find nearest hotspot. Find hotspot with max time untracked.
            h_nearest = None
            h_maxtime = None
            for h_id,h in self.previous_hs.items():
                if h_nearest is None:
                    h_nearest = h
                else:
                    if h.dist < h_nearest.dist:
                        h_nearest = h
                if h_maxtime is None:
                    h_maxtime_id = h_id
                    h_maxtime = h
                else:
                    if h.time > h_maxtime.time:
                        h_maxtime_id = h_id
                        h_maxtime = h
            #print "target hotspot %d location (%d, %d)" % (h_maxtime_id, h_maxtime.location[0], h_maxtime.location[1])
            # Draw a vector to that hotspot.
            vec_to_nearest = np.array([h_nearest.location[0] - self.location[0],
                                       h_nearest.location[1] - self.location[1]])
            dist_to_nearest = np.linalg.norm(vec_to_nearest)   # TODO(syoo): h_nearest.dist?
            vec_to_nearest = vec_to_nearest / dist_to_nearest   # Normalize.

            self.sub_planner.direction = BasePlanner.DIRECTION_CW

           #            if self.time_untracked % 5 == 0:

            # TODO: Grow the obstacle map so the planner doesn't get stuck in an
            # expanding fire. This causes the path planner to stall.
            obstacle_map = scipy.signal.convolve2d(self.fire.fire_progression, [[1, 1, 1], [1, 1, 1], [1, 1, 1]])
            obstacle_map = np.where(self.fire.fire_progression, 100000, 1)
            obstacle_map = cv2.blur(obstacle_map,(5,5)) 
            graph, nodes = astar.make_graph(obstacle_map)
            #cost_map = scipy.signal.convolve2d(self.fire.fire_progression, [[1, 1, 1], [1, 1, 1], [1, 1, 1]])
            paths = astar.AStarGrid(graph)
            start = nodes[int(self.location[0])][int(self.location[1])]

            alpha = .1
            alg = None
            lowest_alg = None
            lowest_id = None
            #insert algorithm here
            for h_id,h in self.previous_hs.items():
                end = nodes[int(h.location[0])][int(h.location[1])]
                res = paths.search(start, end)
                if res is not None:
                    h.set_path(res[0], res[1])
                    alg= -h.time + alpha *  h.path_cost
                    if lowest_alg == None:
                        lowest_alg = alg
                        lowest_id = h_id
                    else:
                        if alg < lowest_alg:
                            lowest_alg = alg
                            lowest_id = h_id
            #end = nodes[int(h_maxtime.location[0])][int(h_maxtime.location[1])]
            #path = paths.search(start, end)

            print "target h_id = %d" % lowest_id
            print "path cost = %d" %self.previous_hs[lowest_id].path_cost
            print "\n------------------------------\n"

            path_arr = []
            last_n = (int(self.location[0]), int(self.location[1]))
            h_id = lowest_id
            for i in range(1,len(self.previous_hs[h_id].path)):
                dx = self.previous_hs[h_id].path[i].x - self.previous_hs[h_id].path[i-1].x
                dy = self.previous_hs[h_id].path[i].y - self.previous_hs[h_id].path[i-1].y
                path_arr.append(np.array([dx, dy]))

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
        return (self.sub_planner.plan(simulation_time), True)


class Hotspot(object):
    def __init__(self, hotspot_loc, uav_loc):
        self.update(hotspot_loc, uav_loc)
        self._time_untracked = 0

    @property
    def location(self):
        return self._location

    @property
    def time(self):
        return self._time_untracked

    @property
    def dist(self):
        return self._dist

    @property
    def path(self):
        return self._path

    @property
    def path_cost(self):
        return self._path_cost

    def set_time(self, new_time):
        self._time_untracked = new_time

    def inc_time(self):
        self._time_untracked += 1

    def reset_time(self):
        self._time_untracked = 0

    def set_path(self, path, cost):
        self._path = path
        self._path_cost = cost

    def update(self, new_hs_loc, new_uav_loc):
        self._location = new_hs_loc
        self._dist = sqrt((self._location[0]-new_uav_loc[0])**2 + (self._location[1]-new_uav_loc[1])**2)


class HotspotTracker(object):
    def __init__(self):
        self._max_untracked = 0
        self._id_next = 0

    @property
    def max_untracked(self):
        return self._max_untracked

    def distance_calc(self, a, b):
        dist =  sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        return dist

    def find_old(self, nh_loc, H):
        maybe_hs = []
        for h_id,h in H.items():
            if self.distance_calc(nh_loc, h.location) < config.HS_RADIUS:
                maybe_hs.append(h_id)
        if len(maybe_hs) == 0:
            return None
        elif len(maybe_hs) == 1:
            return maybe_hs[0]
        else:
            print "WARNING: More than one nearby old hotspot found. Choosing first one."
            return maybe_hs[0]   # TODO(syoo): Maybe this shouldn't just choose the first one!

    def update(self, fire, hs, uav_loc):
        hs_new = {}

        for h_loc in fire.clusters:
            # Find ID of corresponding old hotspot, if any.
            id_old = self.find_old(h_loc, hs)

            if id_old is not None:
                # Update old hotspot
                hs[id_old].update(h_loc, uav_loc)
                hs[id_old].inc_time()

                # Update max untracked time and reset if in view of UAV.
                if hs[id_old].time > self._max_untracked:
                    self._max_untracked = hs[id_old].time
                if hs[id_old].dist < config.FOV:
                    hs[id_old].reset_time()

                # Add hotspot to new list using old ID and remove from old list (so we don't add the same one twice).
                hs_new[id_old] = hs[id_old]
                del hs[id_old]
            else:
                # Add new Hotspot with new ID.
                hs_new[self._id_next] = Hotspot(h_loc, uav_loc)
                self._id_next += 1

        return hs_new, hs   # hs is deadlist.
