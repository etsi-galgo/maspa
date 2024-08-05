import math
import numpy as np

from cat2 import *
from drawing import *
from tools import *


def get_cvisible_tops(T, g_obs, a_obs, p, q, k_length):
    """
    Compute a set of catenary-visible points in the space using p vertical planes and a sampling
    of q points per plane
    """

    # Divide the space in p vertical planes
    T_proj = np.array([T[0], T[1], HTOP])
    cradius = get_top_circ_radious(T)
    vplanes, tt = get_vertical_planes(T, T_proj, cradius, p, g_obs, a_obs)

    tops3D = {}
    for vp in vplanes:
        tops = get_take_off_points(cradius, vp, q)

        # Get the catenary-visible point in each plane
        cvis_tops, ti = get_cvisible_tops2D(vp, tops, T, k_length)
        tt += ti

        if cvis_tops != None:
            tops3D.update(cvis_tops)        

    return tops3D, tt


def get_tops_bf(T, g_obs, a_obs, p, q, k_length):
    """
    Compute a set of catenary-visible points in the space without using the visibility module
    """

    
    T_proj = np.array([T[0], T[1], HTOP])
    cradius = get_top_circ_radious(T)

    vplanes, tt = get_vertical_planes(T, T_proj, cradius, p, g_obs, a_obs)

    tops3D = {}
    for vp in vplanes:
        tops = get_take_off_points(cradius, vp, q)
        
        tops_cat = {}
        for top in tops:
            minL = euclidian_distance(top, T)
            cat_points, length, t = get_min_catenary_rectangles(top, T, vp["ground_obstacles"]+vp["aerial_obstacles"], minL, TETHER_LENGTH, k_length, col2=True)
            tt += t

            if length > 0 and not math.isnan(cat_points[0][0]):
                tops_cat[tuple(top)] = {"length": length, "tether": cat_points} 
    
            tops3D.update(tops_cat)


    return tops3D, tt


def get_vertical_planes(T, T_proj, cradius, p, ground_obstacles, aerial_obstacles):

    Cx, Cy, _ = T_proj
    planes = []

    for i in range(p):
        border_point = np.array([math.cos(math.pi/p*i) + Cx, math.sin(math.pi/p*i) + Cy, HTOP])
        v = normalize(border_point - T_proj)
        Q = T_proj - v*cradius
        coords = get_plane_coords(Q, T) # find the best coordinates to represent the vertical plane (avoiding null coordinates problems)
    
        vplane_gobs, t1 = intersect_obstacles_and_vertical_plane(border_point, T, T_proj, ground_obstacles)
        vplane_aobs, t2 = intersect_obstacles_and_vertical_plane(border_point, T, T_proj, aerial_obstacles)

        planes.append({
            "Q": Q,
            "top_vector": v,
            "angle": math.pi/p*i,
            "coords": coords,
            "ground_obstacles": vplane_gobs,
            "aerial_obstacles": vplane_aobs
        })
    
    return planes, t1+t2


def get_plane_coords(X, T):
    return [0,2] if abs(X[0]-T[0]) > abs(X[1]-T[1]) else [1,2] 


def get_take_off_points(cradius, vertical_plane, q):

    Q = vertical_plane["Q"]
    v = vertical_plane["top_vector"]
    step = 2*cradius/(q-1)

    return [Q + step*i*v for i in range(q)]
        

def get_cvisible_tops2D(vplane, tops, T, k_length):
    
    c0, c1 = vplane["coords"]
    T_proj = (T[c0], T[c1])
    tops_proj = [[np.array((top[c0], top[c1]))] for top in tops]
    
    tt = 0
    obs_proj = []
    obstacles = vplane["ground_obstacles"]+vplane["aerial_obstacles"]
    for obs in obstacles:
        obs = [np.array((vertex[c0], vertex[c1])) for vertex in obs]
        t = time.time()
        gch = ConvexHull(obs)
        tt += time.time() - t
        obs_proj.append(gch.points[gch.vertices])

    vertices_lists = [[T_proj]] + tops_proj + obs_proj
    try:
        t = time.time()
        visgraph = make_visibility_graph(vertices_lists)
        tt += time.time() - t
    except:
        return None, tt


    try:
        weights, previous = pvisibility_2D(visgraph, T_proj, TETHER_LENGTH)
    except:
        return None, tt

    tops_cat = {}
    
    for top in tops:
        vtop = vg.Point(top[c0], top[c1]) 
        if vtop in weights:
            minL = max(weights[vtop], euclidian_distance(top, T))
            cat_points, length, t = get_min_catenary_rectangles(top, T, obstacles, minL, TETHER_LENGTH, k_length)
            tt += t

            if length > 0 and not math.isnan(cat_points[0][0]):
                tops_cat[tuple(top)] = {"length": length, "tether": cat_points} 
            
    return tops_cat, tt