'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''

import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl
import time

from collections import deque
from matplotlib import collections  as mc
from random import random
from scipy import linalg

from cat2 import *
from constants import *
from tools import euclidian_distance


BOARD_SHAPE = (100, 70, 40)

class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {tuple(startpos):0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

        self.opt_cat = []
        self.opt_length = None
        self.opt_total_length = None

    def get_id(self, node):
        return self.vex2idx[tuple(node)]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[tuple(pos)]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[tuple(pos)] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self, board_shape, obstacles, radius):

        deltaX, deltaY = board_shape         
        while True:
            random_point = np.random.rand(2)-0.5
            random_point[0] *= 2*deltaX 
            random_point[1] *= 2*deltaY
        
            distances = []
            new_edges = []
            for node in self.vertices:
                d = euclidian_distance(node, random_point)
                if d <= radius:  
                    idx = self.get_id(node)
                    distances.append(d+self.distances[idx])
                    new_edges.append((node, random_point))

            items = sorted(zip(distances, new_edges))            
            for d, new_edge in items:
                if is_visible(*new_edge, obstacles):
                
                    node_parent, new_node = new_edge 
                    parentidx = self.get_id(node_parent)
                    newidx = self.add_vex(new_node)

                    return euclidian_distance(new_node, node_parent), newidx, parentidx
                

def RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length, n_iter=2*10**3, time_for_ending=200, G=None):
    ''' RRT star algorithm '''
    endpos = T[:2]

    if G == None:
        G = Graph(startpos, endpos)

    gobs_4planning = [[v[:2] for v in oi if v[-1]==0] for oi in ground_obs]

    feasible_dist = math.sqrt(TETHER_LENGTH**2 - (endpos[-1]-MARSUPIAL_HEIGHT)**2) 
    init_t = time.time()
    iter = 0
    # while iter < n_iter and time.time() - init_t < time_for_ending:
    while True:

        d, newidx, parentidx = G.randomPosition(BOARD_SHAPE[:2], gobs_4planning, radius)
        
        G.add_edge(newidx, parentidx, d)
        G.distances[newidx] = G.distances[parentidx] + d
        newvex = G.vertices[newidx]

        ####
        if euclidian_distance(newvex, G.endpos) <= feasible_dist:
            
            top = [*newvex, MARSUPIAL_HEIGHT - UAV_RADIUS]
            cat, length, t = get_min_catenary_rectangles(
                np.array(top), T,
                ground_obs + aerial_obs, 
                euclidian_distance_lists(top, G.endpos), 
                TETHER_LENGTH, k_length, col2=True)
        
            if length>0:

                endidx = G.add_vex(G.endpos)
                G.success = True
                G.add_edge(newidx, endidx, length)

                if len(G.opt_cat) == 0 or G.distances[newidx]+length < G.distances[endidx]:
                    G.distances[endidx] = G.distances[newidx]+length
                    G.opt_cat = cat
                    G.opt_length = length

                if G.distances[endidx] < 78:
                    break

        # dist = euclidian_distance(newvex, G.endpos)
        # if dist < 2 * radius:
        #     endidx = G.add_vex(G.endpos)
        #     G.add_edge(newidx, endidx, dist)
        #     try:
        #         G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
        #     except:
        #         G.distances[endidx] = G.distances[newidx]+dist

        iter += 1

    print("total time", time.time()-init_t)

    return G


def dijkstra(G, endnode):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[tuple(G.startpos)]
    endnode = G.endpos
    endidx = G.vex2idx[tuple(endnode)]
    dist = G.distances[endidx]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = endidx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    length = G.distances[prev[endidx]]
    return list(path)[:-1], length


def plot(G, obstacles, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    if obstacles:
        from scipy.spatial import ConvexHull
        for o in obstacles:
            points = np.array([np.array(p[:2]) for p in o if p[-1]==0])
            h = ConvexHull(points)
            for simplex in h.simplices:
                ax.plot(points[simplex, 0], points[simplex, 1], 'k-')


    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)

    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path


def rrt_sequential(plotting=False):
    
    """
        MASPA for sequential targets
    """

    path = "scenarios/S3.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    S = s["S"] 
    Ts = s["T"]   
    ground_obs = s["ground_obstacles"]
    aerial_obs = s["aerial_obstacles"]
    k_length=26
    radius = 10**6
    
    tt = 0
    gpaths = []
    opt_tops = []
    total_length = 0
    for i, T in enumerate(Ts):
        
        t = time.time()
        G = RRT_star(S[:2],T,ground_obs,aerial_obs, radius, k_length, n_iter=10**6, time_for_ending=20)
        tt += time.time() - t

        ground_path, gpath_length = dijkstra(G, T)
        # print("gpath_length", gpath_length)
        if plotting:
            plot(G, ground_obs, ground_path)

        total_length += gpath_length
        gpaths.append(ground_path)
        optX, cat, apath_length = ground_path[-1], G.opt_cat, G.opt_length
        topX = tuple([*optX[:2], MARSUPIAL_HEIGHT-UAV_RADIUS]) 
        opt_ctop = {topX: {"length":apath_length, "tether":cat}}
        opt_tops.append(opt_ctop)
        
        S = tuple([*optX[:2], 0])
        total_length += apath_length
        # print("apath_length", apath_length)

        if i < len(T)-1:
            total_length += apath_length
            # print("comeback apath_length", apath_length)


    print(total_length, tt)

    d = {
        "total_length": total_length,
        "total_time": tt,
        "scenario": "S3",
        "S": S,
        "Ts": Ts,
        "ground_obs": ground_obs,
        "aerial_obs": aerial_obs,
        "gpaths": gpaths,
        "opt_tops": opt_tops
    }

    with open("scenarios/S3_rrt.pkl", "wb") as f:
        f.write(pkl.dumps(d))


def example():

    # # Best path resutls:
    # time for graph 20.010276794433594
    # total_path length 83.3471842895182
    
    with open("scenarios/S2.pkl", "rb") as f:
        scenario = pkl.load(f)

    S = scenario["S"]
    T = scenario["T"]
    a_obs, g_obs = scenario["aerial_obstacles"], scenario["ground_obstacles"] 
    radius = 10**6 # RRT parameter
    k_length = 26

    tt = time.time()
    G = RRT_star(S[:2], T, g_obs, a_obs, radius, k_length, n_iter=10**6, time_for_ending=20)
    # print("time for graph", time.time()-tt)

    if G.success:
        ground_path, gpath_length = dijkstra(G, T)
        plot(G, g_obs, ground_path)

        optX, cat, apath_length = ground_path[-1], G.opt_cat, G.opt_length
        topX = tuple([*optX[:2], MARSUPIAL_HEIGHT-UAV_RADIUS]) 
        opt_ctop = {topX: {"length":apath_length, "tether":cat}}
        rrt_sol = {
            "S": S,
            "T": T,
            "scenario": "S1",
            "ground_path": ground_path,
            "opt_ctop": opt_ctop
        }

        with open("scenarios/S1_rrt.pkl", "wb") as f:
            f.write(pkl.dumps(rrt_sol))

        print("total_path length", gpath_length + apath_length)

    else:
        print("Not SUCCEDED")
        plot(G, g_obs, radius)



if __name__ == '__main__':

    example()

    # rrt_sequential(plotting=False)
                
            
            
            
            

            