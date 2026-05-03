
import numpy as np
import pickle as pkl


def compute_metrics_random_exp():
    
    path = "scenarios/random_results.pkl"
    with open(path, "rb") as f:
        r = pkl.loads(f.read())

    results = {}
    for ri in r:
        for k, v in ri.items():
            if k in results.keys():                
                results[k] = dict_append_value(results[k], 'gp_length', v['gp_length'])
                results[k] = dict_append_value(results[k], 'ap_length', v['ap_length'])
                results[k] = dict_append_value(results[k], 'tt', v['tt'])
            else:
                results[k] = {'gp_length': [v['gp_length']], 'ap_length': [v['ap_length']], 'tt': [v['tt']]}

    for k, v in results.items():
        print("**************")
        print(k)
        for metric, value in v.items():
            print(metric, "mean:", sum(value)/len(value))
        
        tls = [gl+al for gl, al in zip(list(v['gp_length']), list(v['ap_length']))]
        print("total path length  mean:", sum(tls)/len(tls))
        print(len(tls), tls[:5])
        print()


def dict_append_value(d, key, val):

    if val != None:
        try:
            d[key].append(val)
        except:
            d[key] = [val]

    return d


def compute_metrics_random_exp2():
    
    # paths = [
    #     "scenarios/random_results_1-17.pkl",
    #     "scenarios/random_results_18-25.pkl",
    #     "scenarios/random_results_25-32.pkl",
    #     "scenarios/random_results_32-34.pkl",
    #     "scenarios/random_results_34-75.pkl",
    #     "scenarios/random_results_75-100.pkl",
    #     "scenarios/random_results_100-107.pkl",
    # ]

    paths = [
        "scenarios/random_results_extended.pkl",
    ]


    results = {}
    for p in paths:
        with open(p, "rb") as f:
            r = pkl.loads(f.read())
        
        for ri in r:
            for k, v in ri.items():
                if k in results.keys():                
                    results[k] = dict_append_value(results[k], 'gp_length', v['gp_length'])
                    results[k] = dict_append_value(results[k], 'ap_length', v['ap_length'])
                    results[k] = dict_append_value(results[k], 'tt', v['tt'])
                else:
                    results[k] = {'gp_length': [v['gp_length']], 'ap_length': [v['ap_length']], 'tt': [v['tt']]}


    for k, v in results.items():
        print("**************")
        print(k)
        for metric, value in v.items():
            print(metric, "mean:", np.mean(value), "std:", np.std(value))
        
        tls = [gl+al for gl, al in zip(list(v['gp_length']), list(v['ap_length']))]
        print("total path length mean:", np.mean(tls), "std:", np.std(tls))
        print()



if __name__ == "__main__":

    compute_metrics_random_exp2()