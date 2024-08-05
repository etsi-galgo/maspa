import pickle as pkl
import time

from cvisibility import *
from drawing import *
from tools import *


def path_planning_smpp(S, T, ground_obs, aerial_obs, p=5, q=5, k_length=10, plot=False, visibility=None):
    """
        Path Planning problem  between a starting point S and a target point T.

        In the problem we generate a set of feasible take-off points in the ground with the help of the 
        visibility module using a hanging tether (catenary)
        
        We select the point X in the ground minimizing the sum between the length of the ground path from 
        S to X and the aerial path from X to T. For the ground path we use and updated version of the 
        Dijkstra's algorithm in which we consider many target points. 
    """

    tt = 0
    t = time.time()
    # Compute catenary-visible points
    cvisible_tops, t_error = get_cvisible_tops(T, ground_obs, aerial_obs,  p, q, k_length)
    tt += time.time()-t-t_error
    
    # Compute the visibility graph for the ground path
    ground_obs_proj, ground_obs_vertices = get_obstacles_proj_vertices(ground_obs)
    ground_points = {k[:2]:v for k,v in cvisible_tops.items()}
    
    # Dijkstra's algorithm to find the best candidate point in the ground
    t = time.time()
    Xopt, weigths, previous, t_error, visibility = upd_dijkstra_algorithm(S, ground_points, ground_obs_vertices, ground_obs_proj, visibility)
    tt += time.time()-t-t_error
    

    # Printing, plotting and saving the final results
    if Xopt == None:
        return None, None, None, visibility
    else:
        min_ctop = (*Xopt, 0)
        min_ctop_d = {min_ctop: ground_points[Xopt]}

    ground_path = build_path_to_goal(tuple(min_ctop[:2]), previous)
    print("total time:", tt)
    print("gpath", path_length(ground_path), "apath", min_ctop_d[min_ctop]["length"])
    print("paths sum", path_length(ground_path) + min_ctop_d[min_ctop]["length"], "checking:", weigths[tuple(min_ctop[:2])])

    if plot:
        ground_path[-1] = S        
        plot_optimal_solution(S, T, min_ctop_d, ground_path, ground_obs, aerial_obs)

    maspa_sol = {
        "S": S,
        "T": T,
        "scenario": "S1",
        "ground_path": ground_path,
        "opt_ctop": min_ctop_d
    }

    with open("scenarios/S1_maspa.pkl", "wb") as f:
        f.write(pkl.dumps(maspa_sol))


    return ground_path, (min_ctop, min_ctop_d, min_ctop_d[min_ctop]["length"]), tt, visibility
    

def path_planning_bf(S, T, ground_obs, aerial_obs, p=5, q=5, k_length=10, plot=False, visibility=None):
    """
        MASPA strategy without the visibility module
    """
    tt = 0
    t = time.time()
    tops, t_error = get_tops_bf(T, ground_obs, aerial_obs,  p, q, k_length)
    tt += time.time()-t-t_error

    ground_obs_proj, ground_obs_vertices = get_obstacles_proj_vertices(ground_obs)
    ground_points = {k[:2]:v for k,v in tops.items()}
    t = time.time()
    Xopt, weigths, previous, t_error, visibility = upd_dijkstra_algorithm(S, ground_points, ground_obs_vertices, ground_obs_proj, visibility)
    tt += time.time()-t-t_error

    if Xopt == None:
        return None, None, None, visibility
    else:
        min_ctop = (*Xopt, 0)
        min_ctop_d = {min_ctop: ground_points[Xopt]}

    ground_path = build_path_to_goal(tuple(min_ctop[:2]), previous)
    print("total time:", tt)
    print("gpath", path_length(ground_path), "apath", min_ctop_d[min_ctop]["length"])
    print("paths sum", path_length(ground_path) + min_ctop_d[min_ctop]["length"], "checking:", weigths[tuple(min_ctop[:2])])

    if plot:
        plot_optimal_solution(S, T, min_ctop_d, ground_path, ground_obs, aerial_obs)

    maspabf_sol = {
        "S": S,
        "T": T,
        "scenario": "S1",
        "ground_path": ground_path,
        "opt_ctop": min_ctop_d
    }

    with open("scenarios/S1_maspabf.pkl", "wb") as f:
        f.write(pkl.dumps(maspabf_sol))

    return ground_path, (min_ctop, min_ctop_d, min_ctop_d[min_ctop]["length"]), tt, visibility


def maspa_sequential(plot=False, visibility=None):
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
    p=16
    q=30
    k_length=26
    
    tt = 0
    gpaths = []
    opt_tops = []
    total_length = 0
    for i, T in enumerate(Ts):
        ground_path, opt_info, t, visibility = path_planning_smpp(S, T, ground_obs, aerial_obs, p=p, q=q, k_length=k_length, plot=plot, visibility=visibility)

        # # Uncomment to use MASPA without visibility module
        # ground_path, opt_info, t, visibility = path_planning_bf(S, T, ground_obs, aerial_obs, p=p, q=q, k_length=k_length, plot=plot, visibility=visibility)
        
        tt += t
        gpaths.append(ground_path)
        optX, opt_info, length = opt_info

        opt_tops.append(opt_info)
        S = tuple([*optX[:2], 0])
        total_length += path_length(ground_path) + length
        if i < len(T)-1:
            total_length += length

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

    with open("scenarios/S33_maspa.pkl", "wb") as f:
        f.write(pkl.dumps(d))


def build_path_to_goal(point, previous):
    path = [point]
    while previous[path[-1]] != None:
        path.append(previous[path[-1]])
    return path


def path_length(path):
    length = 0
    for i in range(len(path)-1):
        length += euclidian_distance_lists(path[i], path[i+1])
    return length


def S1():
    
    path = "scenarios/S2.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p=16
    q=30
    
    k_length=26
    path_planning_smpp(
        s["S"], 
        s["T"], 
        s["ground_obstacles"],
        s["aerial_obstacles"], 
        p, q, k_length, True)

    # # Uncomment to use MASPA without the visibility module
    # path_planning_bf(
    #     s["S"], 
    #     s["T"], 
    #     s["ground_obstacles"],
    #     s["aerial_obstacles"], 
    #     p, q, k_length, True)


def run_random_experiments(n, init=0):
    """
    Execute MASPA in a set of randomly generated experiments experiments
    """

    path = "scenarios/random_scenarios.pkl"
    path_output = "scenarios/random_results_16-30.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    p = [16]
    q = [30]

    exps=[2]

    k_length= 26

    results = []
    
    # for i in range(init, n):
    for i in exps:
        si = s[i]
        r = {}
        visibility = None
        for pi in p:    
            for qi in q:
                print(pi, qi)
                S = si["S"] + np.random.rand(3) # Avoid numerical errors
                T = si["T"] + np.random.rand(3) # Avoid numerical errors
                
                gl, al, tt, visibility = path_planning_smpp(
                    tuple(S), tuple(T), 
                    si["ground_obstacles"],
                    si["aerial_obstacles"], 
                    pi, qi, k_length,plot=True,visibility=visibility)


                print("visibility", i, len(list(visibility.keys())))

                r[(pi,qi)] = {"gp_length": gl, "ap_length": al, "tt": tt, "scenario": i}
                
        results.append(r)

        with open(path_output, "wb") as f:
            f.write(pkl.dumps(results))

    with open(path_output, "rb") as f:
            r = pkl.loads(f.read())
    print(len(r))
    print(r)



if __name__ == "__main__":
    
    S1()

    maspa_sequential(plot=True)
    # run_random_experiments(5,init=0)



    