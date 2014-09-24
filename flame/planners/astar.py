from math import sqrt
from itertools import product
import sys
import heapq
class AStar(object):
    def __init__(self, graph):
        self.graph = graph

    def heuristic(self, node, start, end):
        raise NotImplementedError

    def search(self, start, end):
        openset = set()
        closedset = set()
        #openheap = []
        path = []
        current = start
        openset.add(current)
        time_out = 0
        threshold = 10

        #heapq.heapify(openheap)
        #heapq.heappush(openheap, start)
        openset.add(current)
        #openheap.append((current))
        print openheap
        print openset

        while openset:
            #current = heapq.heappop(openheap)[1]
            current = min(openset, key=lambda o:o.g + o.h)
            time_out += 1
            current_dist = sqrt((current.x- end.x)**2 + (current.y - end.y)**2) 
            print "current distance = %d" %current_dist
            if current_dist < threshold or time_out== 10000:
                if time_out == 1000:
                    print "TIMEOUT"
                path = []
                dist = current.g
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1], dist
            openset.remove(current)
            closedset.add(current)
            for node in self.graph[current]:
                if node in closedset:
                    continue
                if node in openset:
                    new_g = current.g + current.move_cost(node)
                    if node.g > new_g:
                        node.g = new_g
                        node.parent = current
                else:
                    node.g = current.g + current.move_cost(node)
                    node.h = self.heuristic(node, start, end)*9
                    node.parent = current
                    openset.add(node)
                    #heapq.heappush(openheap, (node))
        return None

class AStarNode(object):
    def __init__(self):
        self.g = 0
        self.h = 0
        self.parent = None

    def move_cost(self, other):
        raise NotImplementedError

class AStarGrid(AStar):
    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2)

class AStarGridNode(AStarNode):
    def __init__(self, x, y, cost):
        self.x, self.y = x, y
        self.cost = cost
        super(AStarGridNode, self).__init__()

    def move_cost(self, other):
        # diagonal = abs(self.x - other.x) == 1 and abs(self.y - other.y) == 1
        # return 14 if diagonal else 10
        dist_cost = sqrt((self.x-other.x)**2 + (self.y-other.y)**2)
        return dist_cost + self.cost

def make_graph(mapinfo):
    nodes = [[AStarGridNode(x, y, mapinfo[x,y]) for y in range(mapinfo.shape[1])] for x in range(mapinfo.shape[0])]
    graph = {}
    for x, y in product(range(mapinfo.shape[0]), range(mapinfo.shape[1])):
        node = nodes[x][y]
        graph[node] = []
        for i, j in product([-1, 0, 1], [-1, 0, 1]):
            if not (0 <= x + i < mapinfo.shape[0]): continue
            if not (0 <= y + j < mapinfo.shape[1]): continue
            graph[nodes[x][y]].append(nodes[x+i][y+j])
    return graph, nodes

# graph, nodes = make_graph({"width": 8, "height": 8})
# paths = AStarGrid(graph)
# start, end = nodes[1][1], nodes[5][7]
# path = paths.search(start, end)
# if path is None:
#     print "No path found"
# else:
#     print "Path found:", path
