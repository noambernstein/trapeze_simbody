#!/usr/bin/env python

import os, sys, re
import numpy as np
import argparse


parser = argparse.ArgumentParser(description="genetic algorithm optimization of trapeze swing")
parser.add_argument("--initial_params_file",type=str,help="initial parameter file")
parser.add_argument("--initial_pert",type=float,help="initial perturbation",default=0.01)
parser.add_argument("--pert",type=float,help="perturbation for each generation",default=0.006)
parser.add_argument("--n_pop",type=int,help="number of members in population",default=20)
parser.add_argument("--n_gen",type=int,help="number of generations",default=200)
parser.add_argument("prefix",type=str,help="prefix for output files")
args = parser.parse_args()
print "# args", args

output_prefix = args.prefix

if args.initial_params_file is not None:
    with open(args.initial_params_file) as fin:
        initial_params = [float(x) for x in fin.readline().split()]
else:
    initial_params = None

n_pop = args.n_pop
n_gen = args.n_gen

poses = ["sweep1", "curl_in", "flat", "hollow", "sweep2", "early_seven", "seven"]
print "# poses", poses
initial_timings = [0.1, 0.45, 0.8, 1.2, 1.4, 1.6, 1.9 ]

# read list of joints and max torques
joint_limits = {}
with open("flyer.data") as fin:
    for l in fin.readlines():
        m = re.search("^\s*([^_]*)_(max_torque|lower_limit|upper_limit)\s+(.+)\s*$", l.strip())
        if m:
            name = m.group(1)
            prop = m.group(2)
            v = float(m.group(3))
            if name not in joint_limits:
                joint_limits[name] = {}
            joint_limits[name][prop] = v
joints = sorted(joint_limits.keys())
print "# joints", joints

# read initial pose params into dict
pose_params = {}
cur_pose = None
with open("poses.data") as fin:
    for l in fin.readlines():
        m = re.search("^\s*POSE\s+([^ ]+)\s+.\s+[0-9]+\s+([^ ]+)\s*$",l.strip())
        if m:
            cur_pose = m.group(1)
            base_pose = m.group(2)
            if base_pose != "-":
                pose_params[cur_pose] = [x for x in pose_params[base_pose]]
            else:
                pose_params[cur_pose] = [None] * (2*len(joints))
        else:
            m = re.search("^\s*([^ ]+)\s+([^ ]+)\s+([^ ]+)", l.strip())
            name = m.group(1)
            angle = float(m.group(2))
            max_torque = m.group(3)
            if max_torque == "-":
                max_torque = joint_limits[name]["max_torque"]
            else:
                max_torque = float(max_torque)
            pose_params[cur_pose][2*joints.index(name)] = angle
            pose_params[cur_pose][2*joints.index(name)+1] = max_torque
pose_params_vec = []
for pose in sorted(poses):
    pose_params_vec.extend(pose_params[pose])

# initialize params
n_params = len(initial_timings) + len(pose_params_vec)
params = np.zeros((n_pop,n_params))
if initial_params is None:
    params[:,:] = initial_timings + pose_params_vec
else:
    params[:,:] = initial_params

# initialize param limits
params_min = np.zeros((n_pop,n_params))
params_max = np.zeros((n_pop,n_params))
params_min[:,0:len(initial_timings)] = 0.01
params_max[:,0:len(initial_timings)] = 1.99
for (pose_i, pose) in enumerate(sorted(poses)):
    for (joint_i, joint) in enumerate(joints):
        params_min[:,len(initial_timings)+pose_i*len(joints)*2+joint_i*2] = joint_limits[joint]["lower_limit"]
        params_max[:,len(initial_timings)+pose_i*len(joints)*2+joint_i*2] = joint_limits[joint]["upper_limit"]
        params_min[:,len(initial_timings)+pose_i*len(joints)*2+joint_i*2+1] = 0.0
        params_max[:,len(initial_timings)+pose_i*len(joints)*2+joint_i*2+1] = joint_limits[joint]["max_torque"]

def write_poses(fout, pose_params):
    kf = ord('a')
    for (ki, pose) in enumerate(sorted(poses)):
        fout.write("POSE {} {} {} -\n".format(pose, chr(kf+ki), len(joints)))
        for (ji, joint) in enumerate(joints):
            fout.write("   {} {} {}\n".format(joint, pose_params[ki*len(joints)*2+ji*2], pose_params[ki*len(joints)*2+ji*2+1]))

def pert_params(params, mag):
    params_range = params_max - params_min
    params += params_range*np.random.normal(scale=mag, size=params.shape)
    timing_params = params[:,:len(initial_timings)]
    timing_params.sort(axis=1)
    params[:,:len(initial_timings)] = timing_params
    too_low_ind = np.where(params < params_min)
    params[too_low_ind] = params_min[too_low_ind]
    too_high_ind = np.where(params > params_max)
    params[too_high_ind] = params_max[too_high_ind]

def pose_seq_str(params, poses):
    return " ".join([str(params[i])+" "+poses[i] for i in range(len(poses))])

def eval_params(params, poses):
    scores = []
    for i_p in range(params.shape[0]):
        sys.stderr.write("eval {}\n".format(i_p))
        with open("t_poses.data","w") as fout:
            write_poses(fout, params[i_p][len(initial_timings):])
        # print("env DYLD_LIBRARY_PATH=':/Users/noamb/simbody/lib' ./trapeze --headless --len 25 " +
            # "--poses t_poses.data --pose_seq {} " .format(pose_seq_str(params[i_p][0:len(initial_timings)], poses)) +
            # "| egrep 'bar_pos -' | nl | head -4 | tail -1" )
        result = os.popen("env DYLD_LIBRARY_PATH=':/Users/noamb/simbody/lib' ./trapeze --headless --len 25 " +
            "--poses t_poses.data --pose_seq {} " .format(pose_seq_str(params[i_p][0:len(initial_timings)], poses)) +
            "| egrep 'bar_pos -' | nl | head -4 | tail -1" ).readline()
        scores.append(float(result.split()[4]))

    return np.array(scores)

def process_best(max_score, prefix, scores, params, iter_label):
    max_score_ind = np.argmax(scores)

    if scores[max_score_ind] > max_score:
        max_score = scores[max_score_ind]

        print "# BEST ", iter_label, max_score

        with open(prefix+".poses.{}.genetic.restart".format(iter_label),"w") as fout:
            fout.write(" ".join(str(x) for x in params[max_score_ind])+"\n")
        with open(prefix+".poses.{}.genetic.seq_string".format(iter_label),"w") as fout:
            fout.write(pose_seq_str(params[max_score_ind], poses)+"\n")
        with open(prefix+".poses.{}.genetic.data".format(iter_label),"w") as fout:
            write_poses(fout, params[max_score_ind][len(initial_timings):])

        with open(prefix+".poses.{}.genetic.restart".format("BEST"),"w") as fout:
            fout.write(" ".join(str(x) for x in params[max_score_ind])+"\n")
        with open(prefix+".poses.{}.genetic.seq_string".format("BEST"),"w") as fout:
            fout.write(pose_seq_str(params[max_score_ind], poses)+"\n")
        with open(prefix+".poses.{}.genetic.data".format("BEST"),"w") as fout:
            write_poses(fout, params[max_score_ind][len(initial_timings):])

    return max_score

# write initial set
scores = eval_params(np.array([params[0]]), poses)
max_score = process_best(-1.0e38, output_prefix, scores, params, -1)
sys.stdout.flush()

pert_params(params, args.initial_pert)

# write initial perturbation set and the best (if better than initial)
scores = eval_params(params, poses)
for i in range(len(scores)):
    print 0, scores[i], " ".join([str(x) for x in params[i]])
print ""
print ""
max_score = process_best(max_score, output_prefix, scores, params, 0)
sys.stdout.flush()

for i_gen in range(n_gen):
    # find best half
    best_scores_ind = np.argsort(scores)[n_pop/2:]
    # duplicate best half twice
    params[:n_pop/2] = params[best_scores_ind]
    params[n_pop/2:] = params[0:n_pop/2]
    # permute
    np.random.shuffle(params)
    # cross half
    for i in range(0,n_pop/2,2):
        (c1, c2) = np.random.choice(n_params+1, 2, replace=False)
        if c1 > c2:
            (c1,c2) = (c2, c1)
        mask = [ ii >= c1 and ii < c2 for ii in range(n_params)]
        p1 = np.select([mask, np.logical_not(mask)], [params[i], params[i+1]])
        p2 = np.select([np.logical_not(mask), mask], [params[i], params[i+1]])
        params[i][:] = p1
        params[i+1][:] = p2

    # perturb
    pert_params(params, args.pert)

    # new scores
    scores = eval_params(params, poses)
    # print set
    for i in range(len(scores)):
        print i_gen+1, scores[i], " ".join([str(x) for x in params[i]])
    print ""
    print ""
    # check for new best
    max_score = process_best(max_score, output_prefix, scores, params, i_gen+1)
    sys.stdout.flush()
