import numpy as np
import pickle as pkl
import random

from constants import *
from drawing import *
from tools import *


def generate_S1(path):
    
    h = MARSUPIAL_HEIGHT
    x_col1 = 30
    y_col1 = 40
    col_thick = 2

    # First obstacles
    colg1 = np.array([
        [x_col1, y_col1, 0], [x_col1, y_col1+col_thick, 0], [x_col1+col_thick, y_col1, 0], [x_col1+col_thick, y_col1+col_thick, 0],
        [x_col1, y_col1, h], [x_col1, y_col1+col_thick, h], [x_col1+col_thick, y_col1, h], [x_col1+col_thick, y_col1+col_thick, h],
    ])
    cola1 = colg1 + np.array([0,0,h+EPSILON]) 
    colg2 = colg1 + np.array([0,20,0]) 
    cola2 = colg2 + np.array([0,0,h+EPSILON]) 

    bar1 = np.array([
        [x_col1, y_col1+col_thick+EPSILON, 1.5*h], [x_col1, y_col1+20-EPSILON, 1.5*h],
        [x_col1, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1, y_col1+20-EPSILON, 2*h+EPSILON], 
        [x_col1+col_thick, y_col1+col_thick+EPSILON, 1.5*h], [x_col1+col_thick, y_col1+20-EPSILON, 1.5*h],
        [x_col1+col_thick, y_col1+col_thick+EPSILON, 2*h+EPSILON], [x_col1+col_thick, y_col1+20-EPSILON, 2*h+EPSILON], 
    ])

    # Second obstacles
    colg3 = colg1 + np.array([5,-5, 0])
    cola3 = np.array([
        [x_col1+5, y_col1-5, h+EPSILON], [x_col1+5+col_thick, y_col1-5, h+EPSILON], 
        [x_col1+5, y_col1-5+col_thick, h+EPSILON], [x_col1+5+col_thick, y_col1-5+col_thick, h+EPSILON], 
        [x_col1+5, y_col1-5, 5*h], [x_col1+5+col_thick, y_col1-5, 5*h], 
        [x_col1+5, y_col1-5+col_thick, 5*h], [x_col1+5+col_thick, y_col1-5+col_thick, 5*h], 
    ])

    colg4 = colg3 + np.array([0,30, 0])
    cola4 = colg4 + np.array([0,0,h+EPSILON]) 
    cola5 = cola4 + np.array([0,0,h+EPSILON]) 

    bar2 = np.array([        
        [x_col1+5, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5, y_col1+25-EPSILON, 2.5*h], 
        [x_col1+5, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5, y_col1+25-EPSILON, 3*h+2*EPSILON], 
        [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 2.5*h], [x_col1+5+col_thick, y_col1+25-EPSILON, 2.5*h], 
        [x_col1+5+col_thick, y_col1-5+col_thick+EPSILON, 3*h+2*EPSILON], [x_col1+5+col_thick, y_col1+25-EPSILON, 3*h+2*EPSILON], 
    ])

    bar3 = np.array([
        [x_col1+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+5, y_col1+15, 4.8*h], 
        [x_col1+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+5, y_col1+15, 5*h], 
        [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 4.8*h], [x_col1+col_thick+5, y_col1+15, 4.8*h], 
        [x_col1+col_thick+5, y_col1-5+col_thick+EPSILON, 5*h], [x_col1+col_thick+5, y_col1+15, 5*h] 
    ])

    gobs = [colg1, colg2, colg3, colg4]
    aobs = [cola1, cola2, cola3, cola4, cola5, bar1, bar2, bar3]

    S = (20,65,0)
    T = (41,40,13)
    visgraph = make_visibility_graph(gobs)
    plot_visibility_graph(visgraph, gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S1.png")


def generate_S2(path):
    
    h = MARSUPIAL_HEIGHT
    wall_thick = 3
    roof_thick = 1

    # First obstacles
    wallg1 = np.array([
        [0, 0, 0], [0, wall_thick, 0], [70, 0, 0], [70, wall_thick, 0], 
        [0, 0, h], [0, wall_thick, h], [70, 0, h], [70, wall_thick, h] 
    ])
    wallg2 = wallg1 + np.array([0,10+wall_thick,0])

    walla1 = wallg1 + np.array([0,0,h+EPSILON])
    walla2 = wallg2 + np.array([0,0,h+EPSILON])

    roof1 = np.array([
        [0,0,2*h+EPSILON], [0,10+2*wall_thick,2*h+EPSILON], [30,0,2*h+EPSILON], [30,10+2*wall_thick,2*h+EPSILON], 
        [0,0,2*h+EPSILON+roof_thick], [0,10+2*wall_thick,2*h+EPSILON+roof_thick], [30,0,2*h+EPSILON+roof_thick], [30,10+2*wall_thick,2*h+EPSILON+roof_thick], 
    ])
    roof2 = roof1 + np.array([40,0,0])

    chimney1 = np.array([
        [30+EPSILON,0,2*h+EPSILON], [40-EPSILON,0,2*h+EPSILON], [30+EPSILON,wall_thick,2*h+EPSILON], [40-EPSILON,wall_thick,2*h+EPSILON],
        [30+EPSILON,0,5*h], [40-EPSILON,0,5*h], [30+EPSILON,wall_thick,5*h], [40-EPSILON,wall_thick,5*h],
    ]) 
    chimney2 = chimney1 + np.array([0,10+wall_thick,0])

    chimney3 = np.array([
        [30-wall_thick,0,2*h+2*EPSILON], [30-EPSILON,0,2*h+2*EPSILON], 
        [30-wall_thick, 10+2*wall_thick ,2*h+2*EPSILON], [30-EPSILON, 10+2*wall_thick ,2*h+2*EPSILON], 
        [30-wall_thick,0,5*h], [30-EPSILON,0,5*h], 
        [30-wall_thick, 10+2*wall_thick ,5*h], [30-EPSILON, 10+2*wall_thick ,5*h],         
    ]) 
    chimney4 = chimney3 + np.array([10+wall_thick+2*EPSILON,0,0]) 
    
    gobs = [wallg1, wallg2]
    aobs = [walla1, walla2, roof1, roof2, chimney1, chimney2, chimney3, chimney4]

    S = (20,40,0)
    T = (35,5 + wall_thick, 5.2*h)
    visgraph = make_visibility_graph(gobs)
    plot_visibility_graph(visgraph, gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S2.png")


def generate_S3(path):
    
    h = MARSUPIAL_HEIGHT
    wall_thick = 3
    roof_thick = 1

    wallg1 = np.array([
        [wall_thick+EPSILON,0,0], [70-wall_thick-EPSILON,0,0], 
        [wall_thick+EPSILON,wall_thick,0], [70-wall_thick-EPSILON,wall_thick,0],
        [wall_thick+EPSILON,0,h], [70-wall_thick-EPSILON,0,h], 
        [wall_thick+EPSILON,wall_thick,h], [70-wall_thick-EPSILON,wall_thick,h]
    ])
    wallg2 = np.array([
        [0,0,0], [wall_thick,0,0],
        [0,70,0], [wall_thick,70,0], 
        [0,0,h], [wall_thick,0,h],
        [0,70,h], [wall_thick,70,h], 
    ])
    wallg3 = wallg1 + np.array([0, 70 - wall_thick, 0])
    wallg4 = wallg2 + np.array([70 - wall_thick, 0, 0])


    # walla1 = wallg1 + np.array([0,0,8*h])
    walla2 = np.array([
        [-10,20,8*h], [wall_thick,20,8*h],
        [-10,50,8*h], [wall_thick,50,8*h], 
        [-10,20,8*h+wall_thick], [wall_thick,20,8*h+wall_thick],
        [-10,50,8*h+wall_thick], [wall_thick,50,8*h+wall_thick], 
    ])
    # walla3 = wallg3 + np.array([0,0,8*h])
    walla4 = walla2 + np.array([80-wall_thick,0,0])

    squareg1 = np.array([
        [20,10,0], [50,10,0], [20,60,0], [50,60,0],
        [20,10,h], [50,10,h], [20,60,h], [50,60,h],  
    ])

    squarea1 = np.array([
        [20,10,h+EPSILON], [50,10,h+EPSILON], [20,60,h+EPSILON], [50,60,h+EPSILON],  
        [20,10,15*h], [50,10,15*h], [20,60,15*h], [50,60,15*h],
    ])

    balcony1 = np.array([
        [5,20,8*h], [20-EPSILON,20,8*h], [5,50,8*h], [20-EPSILON,50,8*h],   
        [5,20,8*h+wall_thick], [20-EPSILON,20,8*h+wall_thick], [5,50,8*h+wall_thick], [20-EPSILON,50,8*h+wall_thick],   
    ])
    balcony2 = balcony1 + np.array([45+EPSILON,0,0])

    walla5 = np.array([
        [5,20,8*h+wall_thick+EPSILON], [20-EPSILON,20,8*h+wall_thick+EPSILON], [5,20+wall_thick,8*h+wall_thick+EPSILON], [20-EPSILON,20+wall_thick,8*h+wall_thick+EPSILON],   
        [5,20,12*h], [20-EPSILON,20,12*h], [5,20+wall_thick,12*h], [20-EPSILON,20+wall_thick,12*h],   
    ])
    walla6 = walla5 + np.array([0,30-wall_thick, 0])
    walla7 = walla5 + np.array([45+EPSILON,0,0])
    walla8 = walla6 + np.array([45+EPSILON,0,0])

    gobs = [wallg1, wallg2, wallg3, wallg4, squareg1]
    aobs = [squarea1, balcony1, balcony2,  walla2,  walla4, walla5, walla6, walla7, walla8]

    # extra aerial walls
    walla9 = np.array([
        [wall_thick+EPSILON,20,8*h], [5-EPSILON,20,8*h],
        [wall_thick+EPSILON,20+wall_thick,8*h], [5-EPSILON,20+wall_thick,8*h],
        [wall_thick+EPSILON,20,12*h], [5-EPSILON,20,12*h],
        [wall_thick+EPSILON,20+wall_thick,12*h], [5-EPSILON,20+wall_thick,12*h],
    ])
    walla10 = walla9 + np.array([0,30-wall_thick,0]) 
    walla11 = walla9 + np.array([65-wall_thick,0,0])
    walla12 = walla11 + np.array([0,30-wall_thick,0])


    aobs += [walla9, walla10, walla11, walla12]


    S = (10,-50,0)
    T1 = (6,30, 12*h)
    T2 = (64,30, 12*h)
    
    scenario = {
        "S": S,
        "T": [T1, T2],
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": None,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario_multitarget(s, "images/S3.png")


def generate_S33(path):
    
    h = MARSUPIAL_HEIGHT
    wall_thick = 3
    roof_thick = 1

    wallg1 = np.array([
        [wall_thick+EPSILON,0,0], [70-wall_thick-EPSILON,0,0], 
        [wall_thick+EPSILON,wall_thick,0], [70-wall_thick-EPSILON,wall_thick,0],
        [wall_thick+EPSILON,0,h], [70-wall_thick-EPSILON,0,h], 
        [wall_thick+EPSILON,wall_thick,h], [70-wall_thick-EPSILON,wall_thick,h]
    ])
    wallg2 = np.array([
        [0,0,0], [wall_thick,0,0],
        [0,70,0], [wall_thick,70,0], 
        [0,0,h], [wall_thick,0,h],
        [0,70,h], [wall_thick,70,h], 
    ])
    wallg3 = wallg1 + np.array([0, 70 - wall_thick, 0])
    wallg4 = wallg2 + np.array([70 - wall_thick, 0, 0])

    # walla1 = wallg1 + np.array([0,0,8*h])
    walla2 = np.array([
        [-10,20,8*h], [wall_thick,20,8*h],
        [-10,50,8*h], [wall_thick,50,8*h], 
        [-10,20,8*h+wall_thick], [wall_thick,20,8*h+wall_thick],
        [-10,50,8*h+wall_thick], [wall_thick,50,8*h+wall_thick], 
    ])
    # walla3 = wallg3 + np.array([0,0,8*h])
    walla4 = walla2 + np.array([80-wall_thick,0,0])

    squareg1 = np.array([
        [20,10,0], [50,10,0], [20,60,0], [50,60,0],
        [20,10,h], [50,10,h], [20,60,h], [50,60,h],  
    ])

    squarea1 = np.array([
        [20,10,h+EPSILON], [50,10,h+EPSILON], [20,60,h+EPSILON], [50,60,h+EPSILON],  
        [20,10,15*h], [50,10,15*h], [20,60,15*h], [50,60,15*h],
    ])

    balcony1 = np.array([
        [10,20,8*h], [20-EPSILON,20,8*h], [10,50,8*h], [20-EPSILON,50,8*h],   
        [10,20,8*h+wall_thick], [20-EPSILON,20,8*h+wall_thick], [10,50,8*h+wall_thick], [20-EPSILON,50,8*h+wall_thick],   
    ])
    balcony2 = balcony1 + np.array([40+EPSILON,0,0])

    walla5 = np.array([
        [10,20,8*h+wall_thick+EPSILON], [20-EPSILON,20,8*h+wall_thick+EPSILON], [10,20+wall_thick,8*h+wall_thick+EPSILON], [20-EPSILON,20+wall_thick,8*h+wall_thick+EPSILON],   
        [10,20,12*h], [20-EPSILON,20,12*h], [10,20+wall_thick,12*h], [20-EPSILON,20+wall_thick,12*h],   
    ])
    walla6 = walla5 + np.array([0,30-wall_thick, 0])
    walla7 = walla5 + np.array([40+EPSILON,0,0])
    walla8 = walla6 + np.array([40+EPSILON,0,0])

    gobs = [wallg1, wallg2, wallg3, wallg4, squareg1]
    aobs = [squarea1, balcony1, balcony2,  walla2,  walla4, walla5, walla6, walla7, walla8]

    # extra aerial walls
    walla9 = np.array([
        [wall_thick+EPSILON,20,8*h], [10-EPSILON,20,8*h],
        [wall_thick+EPSILON,20+wall_thick,8*h], [10-EPSILON,20+wall_thick,8*h],
        [wall_thick+EPSILON,20,12*h], [10-EPSILON,20,12*h],
        [wall_thick+EPSILON,20+wall_thick,12*h], [10-EPSILON,20+wall_thick,12*h],
    ])
    walla10 = walla9 + np.array([0,30-wall_thick,0]) 
    walla11 = walla9 + np.array([60-wall_thick,0,0])
    walla12 = walla11 + np.array([0,30-wall_thick,0])


    aobs += [walla9, walla10, walla11, walla12]


    S = (10,-50,0)
    T1 = (15,34, 12*h)
    T2 = (55,34, 12*h)
    
    scenario = {
        "S": S,
        "T": [T1, T2],
        "ground_obstacles": gobs,
        "aerial_obstacles": aobs,
        "ground_vis_graph": None,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario_multitarget(s, "images/S3.png")


def get_random_instances(ground_n, aerial_n, block_thick, board_size, minh = 25):
    
    h = MARSUPIAL_HEIGHT
    xmax, ymax, zmax = board_size
    side = np.array([0,0,block_thick])
    
    ## Ground Obstacles
    is_disjoint_vertex_ground = lambda p, vlist: not any(map(
        lambda v: 
            euclidian_distance(p, v[:2]) < 2*block_thick, vlist))
            # (v[0] <= p[0] <= v[0]+block_thick and v[1] <= p[1] <= v[1]+block_thick) or 
            # (p[0] <= v[0] <= p[0]+block_thick and p[1] <= v[1] <= p[1]+block_thick), vlist))     
    
    g_main_vertices = []
    iter = 0
    while iter<MAX_ITERS and len(g_main_vertices)<ground_n:
        x = random.randint(0, xmax-block_thick-1)
        y = random.randint(0, ymax-block_thick-1)

        if is_disjoint_vertex_ground((x,y), g_main_vertices):
            g_main_vertices.append(np.array((x,y,0)))
    
    ## Aerial Obstacles
    is_disjoint_vertex_air = lambda p, vlist: not any(map(
        lambda v: 
            euclidian_distance(p, v) < 2*block_thick, vlist))
            # (v[0] <= p[0] <= v[0]+block_thick and 
            # v[1] <= p[1] <= v[1]+block_thick and
            # v[2] <= p[2] <= v[2]+block_thick) or 
            # (p[0] <= v[0] <= p[0]+block_thick and 
            # p[1] <= v[1] <= p[1]+block_thick and
            # p[2] <= v[2] <= p[2]+block_thick), vlist))     
    
    a_main_vertices = []
    iter = 0
    while iter<MAX_ITERS and len(a_main_vertices)<aerial_n+1:
        x = random.randint(0, xmax-block_thick)
        y = random.randint(0, ymax-block_thick)
        z = random.randint(h, zmax-block_thick)

        if is_disjoint_vertex_air((x,y,z), a_main_vertices):
            a_main_vertices.append(np.array((x,y,z)))

    a_main_vertices.sort(key=lambda p: p[-1], reverse=True)
    target_point = (*a_main_vertices[0][:2], max(a_main_vertices[0][-1], minh)) 

    if euclidian_distance_lists(target_point, np.zeros((3,))) > euclidian_distance_lists(target_point, np.array([xmax, ymax, 0])):  
        starting_point = np.zeros((3,))
    else:
        starting_point = np.array([xmax, ymax,0])

    # Get Obstacles
    get_obs = lambda vlist: [np.array([
        v, v + side, 
        v+np.array([0,block_thick,0]), v+np.array([0,block_thick,0]) + side,
        v+np.array([block_thick,0,0]), v+np.array([block_thick,0,0]) + side, 
        v+np.array([block_thick,block_thick,0]), v+np.array([block_thick,block_thick,0]) + side 
        ]) for v in vlist]

    gobs = get_obs(g_main_vertices)
    aobs = get_obs(a_main_vertices[1:])

    # gobs = [np.array([
    #     v, v + side, 
    #     v+np.array([0,block_thick,0]), v+np.array(0,block_thick,0) + side,
    #     v+np.array([block_thick,0,0]), v+np.array([block_thick,0,0]) + side, 
    #     v+np.array(block_thick,block_thick,0), v+np.array(block_thick,block_thick,0) + side 
    #     ]) for v in g_main_vertices[n_instances:]]

    # aobs = [np.array([
    #     v, v + side, 
    #     v+np.array(0,block_thick,0), v+np.array(0,block_thick,0) + side,
    #     v+np.array(block_thick,0,0), v+np.array(block_thick,0,0) + side, 
    #     v+np.array(block_thick,block_thick,0), v+np.array(block_thick,block_thick,0) + side 
    #     ]) for v in a_main_vertices[n_instances:]]

    return starting_point, target_point, gobs, aobs    


def get_random_scenarios(n_scenarios, ground_n, aerial_n, block_thick, board_size, path, plot=None):

    rscenarios = []
    for i in range(n_scenarios):

        spoint, tpoint, gobs, aobs = get_random_instances(ground_n, aerial_n, block_thick, board_size)
        # visgraph = make_visibility_graph(gobs)
        s = {
            "S": spoint,
            "T": tpoint,
            "ground_obstacles": gobs,
            "aerial_obstacles": aobs
            # "ground_vis_graph": visgraph,
        }
        rscenarios.append(s)

    with open(path, "wb") as f:
        f.write(pkl.dumps(rscenarios))

    if plot:
    
        with open(path, "rb") as f:
            s = pkl.loads(f.read())
    
        print("Number of Scenarios:", len(s))
        for i in range(len(s)):
            plot3D([
                {
                    "point": s[i]["S"], 
                    "label":"S", "color":"k"
                }, 
                {
                    "point": s[i]["T"],
                    "label":"T", "color":"r"
                }], 
                s[i]["ground_obstacles"]+s[i]["aerial_obstacles"])


if __name__ == "__main__":
    
    # generate_S1("scenarios/S1.pkl")
    # generate_S2("scenarios/S2.pkl")
    # generate_S3("scenarios/S3.pkl")
    generate_S33("scenarios/S3.pkl")

    n = 1000
    ground_n = 10 
    aerial_n = 15
    block_thick = 5 
    board_size = (50,50,40)
    path = "scenarios/random_scenarios.pkl"
    
    # get_random_scenarios(n, ground_n, aerial_n, block_thick, board_size, path, plot=False)