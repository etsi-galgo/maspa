import matplotlib.pyplot as plt
import numpy as np

from pycatenary import cable
from tools import *
from bisect import bisect
from drawing import *


def get_min_catenary_rectangles(top, T, obstacles, Lmin, Lmax, k_length, col2 = False):    
    '''
    Take k_length lengths between the straight line and the L_max
    
    'min_cat_delta' evita que la libreria te de error con la primera catenaria por ser una recta
    '''

    tt = 0
    for l in np.linspace(Lmin + EPSILON, Lmax - EPSILON, k_length):
        
        a = [0,0,0]
        f = T - top
        l1 = cable.MooringLine(L=l, w=0, EA=None, anchor=a, fairlead=f, floor=False)        
        l1.computeSolution()

        xyzs = []
        for s in np.linspace(0., l):
            xyz = l1.s2xyz(s)
            xyzs.append(xyz)

        if xyzs[0][0] == None or math.isnan(xyzs[0][0]):
            return None, -1, tt
        
        if xyzs[0][-1] < 0:
            xyzs += np.array([0,0,-xyzs[0][-1]]) + top
        else:
            xyzs += top
        
        if euclidian_distance_lists(xyzs[-1], T) > 2*UAV_RADIUS:
            continue


        for xyz in xyzs:
            if xyz[-1] <= 0:
                return None, -1, tt
                
        if col2: 
            is_collision_cat_obs = lambda oi: cat_rectangle_collision2(oi, xyzs)
        else:
            is_collision_cat_obs = lambda oi: cat_rectangle_collision(oi, xyzs)


        collision = False
        for oi in obstacles:  
            t = time.time()  
            if is_collision_cat_obs(oi):
                for v in oi:
                    if v[-1] == 0:
                        return None, -1, tt
                    
                # fig = plt.figure()
                # ax = fig.add_subplot(111, projection='3d')
                # ax.set_xlabel('X Axis')
                # ax.set_ylabel('Y Axis')
                # ax.set_zlabel('Z Axis')
                # xx, yy, zz = zip(*xyzs)
                
                # for v1 in oi:
                #     for v2 in oi:
                #         plt.plot([v1[0], v2[0]],[v1[1], v2[1]],[v1[2], v2[2]], "-k")

                # plt.plot(xx, yy, zz, '-b')  
                # plt.title("NO") 
                # plt.show() 


                collision = True
                break
            tt += time.time() - t                
            
        if collision:
            continue 

        return xyzs, l, tt
    
    return None, -1, tt

            
def cat_rectangle_collision(oi, xyzs):

    # plane_number: 10 point_number: 10 t_first_step: 140.73789429664612 dij_time: 1.1343741416931152
    # plane_number: 10 point_number: 20 t_first_step: 274.8295590877533 dij_time: 1.9045300483703613

    if not len(oi):
        return False
    
    xs, ys, _ = list(zip(*xyzs))
    coord = 0 if abs(xs[0]-xs[-1]) > abs(ys[0]-ys[-1]) else 1
    
    X_cat = list(list(zip(*xyzs))[coord])
    is_reversed = False
    if X_cat[0] > X_cat[-1]:
        X_cat.reverse()
        is_reversed = True
    
    return collision(X_cat, xyzs, is_reversed, oi, coord)
        

    
def collision(X_cat, cat, is_reversed, vertices, coord):
    

    above, below = False, False
    for vi in vertices:
        index = bisect(X_cat, vi[coord])
        if is_reversed:
            index = len(X_cat) - index - 1
        
        if 0 < index < len(X_cat)-1:
            if vi[2] > cat[index][2] and vi[2] > cat[index+1][2]:
                above = True
            else:
                below = True

    
    return above and below




            
def cat_rectangle_collision2(oi, xyzs):

    # plane_number: 10 point_number: 10 t_first_step: 140.73789429664612 dij_time: 1.1343741416931152
    # plane_number: 10 point_number: 20 t_first_step: 274.8295590877533 dij_time: 1.9045300483703613


    if not len(oi):
        return False
    
    X1,X2 = min(oi, key=lambda x: x[0])[0], max(oi, key=lambda x: x[0])[0] 
    Y1,Y2 = min(oi, key=lambda x: x[1])[1], max(oi, key=lambda x: x[1])[1]
    Z1,Z2 = min(oi, key=lambda x: x[2])[2], max(oi, key=lambda x: x[2])[2]

    for x, y, z in xyzs:
        
        if (X1 <= x <= X2) and (Y1 <= y <= Y2) and (Z1 <= z <= Z2):
            
            return True

    return False  
        
        

    
