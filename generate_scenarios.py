import numpy as np
import pickle as pkl
import random

from constants import *
from drawing import *
from tools import *



def plot_ground_topdown(scenario, path=None):
    fig, ax = plt.subplots(figsize=(12, 12))
    for obs in scenario["ground_obstacles"]:
        pts2d = obs[:, :2]
        hull = ConvexHull(pts2d)
        ax.fill(pts2d[hull.vertices, 0], pts2d[hull.vertices, 1],
                alpha=0.55, color="steelblue", ec="black", lw=0.8)
    S = scenario["S"]
    T = scenario["T"]
    ax.plot(S[0], S[1], "ko", markersize=10, label="S")
    ax.plot(T[0], T[1], "r^", markersize=10, label="T (xy projection)")
    ax.set_xlabel("x"); ax.set_ylabel("y")
    ax.set_aspect("equal"); ax.autoscale_view(); ax.legend()
    if path:
        plt.savefig(path, bbox_inches="tight", dpi=150)
    plt.close(fig)


def plot_aerial_only(scenario, path=None):
    T = scenario["T"]
    fig, ax = plot3D(
        [{"point": T, "label": "T", "color": "r"}],
        scenario["aerial_obstacles"],
        path_to_output=path,
        show=None,
    )
    plt.close(fig)




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
    aobs = [squarea1, balcony1, balcony2,  walla2,  walla4, walla5, walla6, walla7, walla8,]
            # walla9, walla10, walla11, walla12]

    # extra aerial walls
    walla9 = np.array([
        [-10+EPSILON,20,wall_thick+8*h+EPSILON], [5-EPSILON,20,wall_thick+8*h+EPSILON],
        [-10+EPSILON,20+wall_thick,wall_thick+8*h+EPSILON], [5-EPSILON,20+wall_thick,wall_thick+8*h+EPSILON],
        [-10+EPSILON,20,8*h + 2*wall_thick], [5-EPSILON,20,8*h + 2*wall_thick],
        [-10+EPSILON,20+wall_thick,8*h + 2*wall_thick], [5-EPSILON,20+wall_thick,8*h + 2*wall_thick],
    ])
    walla10 = walla9 + np.array([0,30-wall_thick,0]) 
    walla11 = walla9 + np.array([75+EPSILON,0,0])
    walla12 = walla11 + np.array([0,30-wall_thick,0])

    aobs += [walla9, walla10, walla11, walla12]

    # walla13 = np.array([
    #     [5,20,8*h+wall_thick+EPSILON], [20-EPSILON,20,8*h+wall_thick+EPSILON], [5,20+wall_thick,8*h+wall_thick+EPSILON], [20-EPSILON,20+wall_thick,8*h+wall_thick+EPSILON],   
    #     [5,20,12*h], [20-EPSILON,20,12*h], [5,20+wall_thick,12*h], [20-EPSILON,20+wall_thick,12*h],   
    # ])
    # walla14 = walla13 + np.array([0,30-wall_thick, 0])
    # walla15 = walla13 + np.array([45+EPSILON,0,0])
    # walla16 = walla14 + np.array([45+EPSILON,0,0])

    # aobs += [walla13, walla14, walla15, walla16]


    S = (-10,-40,0)
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

    
    plot_scenario_multitarget(s, "images/S3.png", show=True)




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



def generate_S5(path):
    """Ground-dominant stress scenario — dense obstacles.

    Five horizontal serpentine walls + 11 rectangular blocks (2 per corridor)
    force the UGV to zigzag ~160+ units across a 100×130 board, stressing the
    ground visibility module heavily.

    T at low altitude (z=4, 2 m above HTOP=2) sits just north of the last wall
    gap → tether is ~9 units, making ground path clearly dominant (~18:1 ratio).

    Aerial cluster: 12 small boxes in 3 stacked rings around T stress the
    catenary visibility module.

    All ground-obstacle corner coordinates are globally unique to avoid
    collinear-edge issues in pyvisgraph.
    """
    h = MARSUPIAL_HEIGHT

    def block(x0, y0, dx, dy):
        return np.array([
            [x0,      y0,      0], [x0,      y0+dy,   0],
            [x0+dx,   y0,      0], [x0+dx,   y0+dy,   0],
            [x0,      y0,      h], [x0,      y0+dy,   h],
            [x0+dx,   y0,      h], [x0+dx,   y0+dy,   h],
        ])

    # ── Three enclosing boundary walls ──────────────────────────────────────
    # Each wall = thin ground box (z: 0 → h) + tall aerial box (z: h+ε → 150)
    # Wall L (left):  x ∈ [-1,  1],   y ∈ [0, 165]
    # Wall T (top):   x ∈ [ 1, 123],  y ∈ [165, 167]
    # Wall R (right): x ∈ [123, 125], y ∈ [0,  165]
    # Walls touch at corners only — no volume overlap.
    w_air_h = 26 - h   # aerial top reaches z = 150

    def wall_aerial(x0, y0, x1, y1):
        return np.array([
            [x0, y0, h+EPSILON], [x0, y1, h+EPSILON],
            [x1, y0, h+EPSILON], [x1, y1, h+EPSILON],
            [x0, y0, h+EPSILON+w_air_h], [x0, y1, h+EPSILON+w_air_h],
            [x1, y0, h+EPSILON+w_air_h], [x1, y1, h+EPSILON+w_air_h],
        ])

    # EPSILON offsets at every joint so no two wall polygons share an edge
    # coordinate (avoids collinear-edge issues in pyvisgraph).
    # Wall L is the reference; T is offset +ε past L's right/top edges;
    # R is offset so its top sits -ε below T's bottom edge.
    #   L–T joint: x_L_right=1  < x_T_left=1+ε;  y_L_top=165 < y_T_bot=165+ε
    #   T–R joint: x_T_right=123-ε < x_R_left=123; y_T_bot=165+ε > y_R_top=165-ε
    wallg_L = block(-1,           0,             2,           165)           # x∈[-1, 1],      y∈[0,   165]
    wallg_T = block( 1+EPSILON,   165+EPSILON,   122-EPSILON, 2)            # x∈[1+ε, 123-ε], y∈[165+ε,167+ε]
    wallg_R = block(123,          0,             2,           165-EPSILON)  # x∈[123, 125],    y∈[0,   165-ε]

    walla_L = wall_aerial(-1,          0,          1,          165)
    walla_T = wall_aerial( 1+EPSILON,  165+EPSILON, 123-EPSILON, 167+EPSILON)
    walla_R = wall_aerial(123,         0,           125,        165-EPSILON)

    # ── 5 horizontal walls, alternating east / west gap ─────────────────
    # Wall x-corners (all unique): 10,82 | 18,90 | 12,84 | 20,92 | 14,86
    # Wall y-corners (all unique): 23,26 | 46,49 | 69,72 | 92,95 | 115,118
    wallg1 = block(10,  23, 72, 3)   # gap east  (x > 82)
    wallg2 = block(18,  46, 72, 3)   # gap west  (x < 18)
    wallg3 = block(12,  69, 72, 3)   # gap east  (x > 84)
    wallg4 = block(20,  92, 72, 3)   # gap west  (x < 20)
    wallg5 = block(14, 115, 72, 3)   # gap east  (x > 86)

    # ── 11 rectangular blocks — 2 per corridor zone (all unique corners) ─
    # Zone A y=0..23   (below W1)
    b1  = block(35,   9, 5, 4)   # x=[35,40],  y=[9,13]
    b1c = block(58,   4, 4, 4)   # x=[58,62],  y=[4,8]
    # Zone B y=26..46  (east passage between W1 and W2)
    b2  = block(55,  30, 4, 5)   # x=[55,59],  y=[30,35]
    b2c = block(85,  27, 4, 5)   # x=[85,89],  y=[27,32]
    # Zone C y=49..69  (between W2 and W3)
    b3  = block(41,  53, 5, 4)   # x=[41,46],  y=[53,57]
    b3c = block(25,  50, 5, 4)   # x=[25,30],  y=[50,54]
    b3d = block(70,  58, 4, 5)   # x=[70,74],  y=[58,63]
    # Zone D y=72..92  (east passage between W3 and W4)
    b4  = block(65,  76, 4, 5)   # x=[65,69],  y=[76,81]
    b4c = block(87,  73, 4, 5)   # x=[87,91],  y=[73,78]
    # Zone E y=95..115 (between W4 and W5)
    b5  = block(37,  99, 5, 4)   # x=[37,42],  y=[99,103]
    b5c = block(63,  96, 4, 5)   # x=[63,67],  y=[96,101]

    gobs = [wallg_L, wallg_T, wallg_R,
            wallg1, wallg2, wallg3, wallg4, wallg5,
            b1, b1c, b2, b2c, b3, b3c, b3d, b4, b4c, b5, b5c]

    # ── 12 aerial obstacles: 6 current + 6 fat/tall alternating ─────────
    # Current (even i): 3×3×3 boxes at radius 9, z = h,     angles 0°/60°/…/300°
    # Fat    (odd  i): 5×5×7 boxes at radius 10, z = h+1,   angles 30°/90°/…/330°
    # Arrangement clockwise: current → fat → current → fat → …
    # No volumetric intersections (adjacent pairs share at most one face).
    def abox(x0, y0, z0, sx=5, sy=5, sz=10):
        return np.array([
            [x0,    y0,    z0],    [x0,    y0+sy, z0],
            [x0+sx, y0,    z0],    [x0+sx, y0+sy, z0],
            [x0,    y0,    z0+sz], [x0,    y0+sy, z0+sz],
            [x0+sx, y0,    z0+sz], [x0+sx, y0+sy, z0+sz],
        ])

    import math as _math
    _tx, _ty = 88, 125
    aobs = [walla_L, walla_T, walla_R]
    for i in range(14):
        _angle = _math.pi/8 + 2 * _math.pi * i / 14   # 30° steps
        if i % 2 == 0:                    # current: 0°, 60°, 120°, 180°, 240°, 300°
            _r  = 20 + 4*EPSILON*i
            _cx = int(_tx + _r * _math.cos(_angle)) + 4*EPSILON*i
            _cy = int(_ty + _r * _math.sin(_angle)) + 4*EPSILON*i
            aobs.append(abox(_cx - 2, _cy - 1, 5 + 4*EPSILON*i + random.randint(-3, 3), sx=11, sy=11, sz=9))
        else:                             # fat/tall: 30°, 90°, 150°, 210°, 270°, 330°
            _r  = 15 + 4*EPSILON*i
            _cx = int(_tx + _r * _math.cos(_angle)) + 4*EPSILON*i
            _cy = int(_ty + _r * _math.sin(_angle)) + 4*EPSILON*i
            aobs.append(abox(_cx - 1, _cy - 2, 15 + 4*EPSILON*i + random.randint(-3, 3), sx=11, sy=11, sz=9))

    # Large obstacle centred below T — blocks most direct catenary paths,
    # forcing the UAV to approach from the sides.
    # T = (88, 137, 20); box spans x∈[78,98], y∈[127,147], z∈[h, 18]
    aobs.append(abox(85+4*EPSILON, 125, 13+EPSILON, sx=9, sy=9, sz=3))
    aobs.append(abox(85, 131, 17+EPSILON, sx=9, sy=9, sz=2))

    S = (12, 8, 0)
    T = (90, 130, 23)  # high altitude; catenary must thread the obstacle ring

    visgraph = make_visibility_graph(gobs)

    scenario = {
        "S": S,
        "T": T,
        "ground_obstacles": gobs,
        "aerial_obstacles":  aobs,
        "ground_vis_graph":  visgraph,
    }

    with open(path, "wb") as f:
        f.write(pkl.dumps(scenario))

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    plot_scenario(s, "images/S5.png", show=False)
    plot_ground_topdown(s, "images/S5_ground.png")
    plot_aerial_only(s, "images/S5_aerial.png")



if __name__ == "__main__":
    
    # generate_S1("scenarios/S1.pkl")
    # generate_S2("scenarios/S2.pkl")
    generate_S3("scenarios/S3.pkl")
    # generate_S5("scenarios/S5.pkl")

    n = 1000
    ground_n = 10 
    aerial_n = 15
    block_thick = 5 
    board_size = (50,50,40)
    path = "scenarios/random_scenarios.pkl"
    
    # get_random_scenarios(n, ground_n, aerial_n, block_thick, board_size, path, plot=False)