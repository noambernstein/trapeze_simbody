#!/usr/bin/env python

import os, sys
import numpy as np

# ./trapeze --headless --len 25 --pose_seq `cat pose_seq_swing.data` | egrep 'bar_pos -' | nl | head -4 | tail -1

n_pop = 20
n_gen = 20

poses = ["sweep", "curl_in", "flat", "hollow", "sweep", "early_seven", "seven"]
default_times = [0.1, 0.45, 0.8, 1.2, 1.4, 1.6, 1.9 ]

n_params = len(poses)
timing_params = np.zeros((n_pop,n_params))
timing_params[:,:] = default_times

def pert_params(timing_params, mag):
    timing_params += np.random.normal(scale=mag, size=timing_params.shape)
    timing_params.sort(axis=1)
    timing_params[np.where(timing_params < 0.01)] = 0.01
    timing_params[np.where(timing_params > 1.99)] = 1.99


def eval_params(timing_params, poses):
    scores = []
    for i_p in range(timing_params.shape[0]):
        sys.stderr.write("eval {}\n".format(i_p))
        result = os.popen("env DYLD_LIBRARY_PATH=':/Users/noamb/simbody/lib' ./trapeze --headless --len 25 --pose_seq %s | " %
            " ".join([str(timing_params[i_p,i])+" "+poses[i] for i in range(0,len(poses),2)]) +
            "egrep 'bar_pos -' | nl | head -4 | tail -1" ).readline()
        scores.append(float(result.split()[4]))

    return np.array(scores)

pert_params(timing_params, 0.1)

scores = eval_params(timing_params, poses)
for i in range(len(scores)):
    print 0, scores[i], " ".join([str(x) for x in timing_params[i]])
print ""
print ""

for i_gen in range(n_gen):
    # find best half
    best_scores_ind = np.argsort(scores)[n_pop/2:]
    # duplicate best half twice
    timing_params[:n_pop/2] = timing_params[best_scores_ind]
    timing_params[n_pop/2:] = timing_params[0:n_pop/2]
    # permute
    np.random.shuffle(timing_params)
    for i in range(0,n_pop/2,2):
        (c1, c2) = np.random.choice(n_params+1, 2, replace=False)
        if c1 > c2:
            (c1,c2) = (c2, c1)
        mask = [ ii >= c1 and ii < c2 for ii in range(n_params)]
        p1 = np.select([mask, np.logical_not(mask)], [timing_params[i], timing_params[i+1]])
        p2 = np.select([np.logical_not(mask), mask], [timing_params[i], timing_params[i+1]])
        timing_params[i][:] = p1
        timing_params[i+1][:] = p2
    pert_params(timing_params, 0.04)

    scores = eval_params(timing_params, poses)
    for i in range(len(scores)):
        print i_gen+1, scores[i], " ".join([str(x) for x in timing_params[i]])
    print ""
    print ""
