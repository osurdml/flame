class HotspotTracker(object):
    def __init__(self, fire):
        self.fire = fire
        self.hs_dists = None
        self.xs = None
        self.ys = None
    def set_location(self, location):
        self.location = location

    def update(self):
        if self.fire.clusters.any():
            #print self.fire.clusters
            (self.xs, self.ys) = self.fire.clusters.T
            self.hs_dists = (self.xs - self.location[0]) ** 2 + (self.ys - self.location[1]) ** 2
            #print hs_dists
            return self.hs_dists, self.xs, self.ys 


