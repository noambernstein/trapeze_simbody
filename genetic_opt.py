#!/usr/bin/env python

import os, sys, re
import numpy as np
import argparse
from itertools import izip

parser = argparse.ArgumentParser(description="genetic algorithm optimization of trapeze swing")
parser.add_argument("--flyer_file",type=str,help="file describing flyer")
parser.add_argument("--poses_file",type=str,help="file of initial poses")
parser.add_argument("--pose_seq_file",type=str,help="file of initial pose timings")
parser.add_argument("--initial_pert",type=float,help="initial perturbation",default=0.01)
parser.add_argument("--pert",type=float,help="perturbation for each generation",default=0.006)
parser.add_argument("--n_pop",type=int,help="number of members in population",default=20)
parser.add_argument("--n_gen",type=int,help="number of generations",default=200)
parser.add_argument("prefix",type=str,help="prefix for output files")
args = parser.parse_args()
print "# args", args

output_prefix = args.prefix

n_pop = args.n_pop
n_gen = args.n_gen

# read list of joints and max torques
joint_limits = {}
with open(args.flyer_file) as fin:
    for l in fin.readlines():
        m = re.search("^\s*([^_]*)_(max_torque|lower_limit|upper_limit)\s+(.+)\s*$", l.strip())
        if m:
            name = m.group(1)
            prop = m.group(2)
            val = float(m.group(3))
            if name not in joint_limits:
                joint_limits[name] = {}
            joint_limits[name][prop] = val
joints = sorted(joint_limits.keys())
print "# joints", joints

# read initial poses into dict
pose_params = {}
cur_pose = None
with open(args.poses_file) as fin:
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
            joint_name = m.group(1)
            angle = float(m.group(2))
            max_torque = m.group(3)
            if max_torque == "-":
                max_torque = joint_limits[joint_name]["max_torque"]
            else:
                max_torque = float(max_torque)
            pose_params[cur_pose][2*joints.index(joint_name)] = angle
            pose_params[cur_pose][2*joints.index(joint_name)+1] = max_torque
print "got pose_params", pose_params.keys()

pose_timings = []
pose_seq = []
with open(args.pose_seq_file) as fin:
    l = fin.readline()
    for (pose_time, pose_name) in izip(l.split()[0::2], l.split()[1::2]):
        pose_timings.append(float(pose_time))
        pose_seq.append(pose_name)
print "got pose_timings", pose_timings
print "got pose_names", pose_seq
n_pose_seq = len(pose_seq)

# initialize params
initial_params_vec = []
initial_params_vec.extend(pose_timings)
for pose in sorted(pose_seq):
    initial_params_vec.extend(pose_params[pose])
n_params = len(initial_params_vec)
params = np.zeros((n_pop,n_params))
params[:,:] = initial_params_vec

# initialize param limits
params_min = np.zeros((n_pop,n_params))
params_max = np.zeros((n_pop,n_params))
# timings params
params_min[:,0:n_pose_seq] = 0.001
params_max[:,0:n_pose_seq] = 1.999
# pose params
for (pose_i, pose) in enumerate(sorted(pose_seq)):
    for (joint_i, joint) in enumerate(joints):
        params_min[:,n_pose_seq+pose_i*len(joints)*2+joint_i*2] = joint_limits[joint]["lower_limit"]
        params_max[:,n_pose_seq+pose_i*len(joints)*2+joint_i*2] = joint_limits[joint]["upper_limit"]
        params_min[:,n_pose_seq+pose_i*len(joints)*2+joint_i*2+1] = 0.0
        params_max[:,n_pose_seq+pose_i*len(joints)*2+joint_i*2+1] = joint_limits[joint]["max_torque"]

def write_poses(fout, pose_params, pose_seq):
    kf = ord('a')
    for (ki, pose) in enumerate(sorted(pose_seq)):
        fout.write("POSE {} {} {} -\n".format(pose, chr(kf+ki), len(joints)))
        for (ji, joint) in enumerate(joints):
            fout.write("   {} {} {}\n".format(joint, pose_params[ki*len(joints)*2+ji*2], pose_params[ki*len(joints)*2+ji*2+1]))

def pose_seq_str(params, pose_seq):
    return " ".join([str(params[i])+" "+pose_seq[i] for i in range(len(pose_seq))])

def pert_params(params, mag):
    params_range = params_max - params_min
    params += params_range*np.random.normal(scale=mag, size=params.shape)
    timing_params = params[:,:n_pose_seq]
    timing_params.sort(axis=1)
    params[:,:n_pose_seq] = timing_params
    too_low_ind = np.where(params < params_min)
    params[too_low_ind] = params_min[too_low_ind]
    too_high_ind = np.where(params > params_max)
    params[too_high_ind] = params_max[too_high_ind]

def eval_params(params, pose_seq):
    scores = []
    for i_p in range(params.shape[0]):
        sys.stderr.write("eval {}\n".format(i_p))
        with open("t_poses.data","w") as fout:
            write_poses(fout, params[i_p][n_pose_seq:], pose_seq)
        result = os.popen("env DYLD_LIBRARY_PATH=':/Users/noamb/simbody/lib' ./trapeze --headless --len 25 " +
            "--flyer {} --poses t_poses.data --pose_seq {} " .format(args.flyer_file, pose_seq_str(params[i_p][0:n_pose_seq], pose_seq)) +
            "| egrep 'bar_pos -' | nl | head -4 | tail -1" ).readline()
        scores.append(float(result.split()[4]))

    return np.array(scores)

def process_best(max_score, prefix, scores, params, iter_label):
    max_score_ind = np.argmax(scores)

    if scores[max_score_ind] > max_score:
        max_score = scores[max_score_ind]

        print "# BEST ", iter_label, max_score

        with open(prefix+".genetic_opt.{}.pose_seq.data".format(iter_label),"w") as fout:
            fout.write(pose_seq_str(params[max_score_ind], pose_seq)+"\n")
        with open(prefix+".genetic_opt.{}.poses.data".format(iter_label),"w") as fout:
            write_poses(fout, params[max_score_ind][n_pose_seq:], pose_seq)

        with open(prefix+".genetic_opt.{}.pose_seq.data".format("BEST"),"w") as fout:
            fout.write(pose_seq_str(params[max_score_ind], pose_seq)+"\n")
        with open(prefix+".genetic_opt.{}.poses.data".format("BEST"),"w") as fout:
            write_poses(fout, params[max_score_ind][n_pose_seq:], pose_seq)

    return max_score

# write initial set
scores = eval_params(np.array([params[0]]), pose_seq)
max_score = process_best(-1.0e38, output_prefix, scores, params, -1)
sys.stdout.flush()

pert_params(params, args.initial_pert)

# write initial perturbation set and the best (if better than initial)
scores = eval_params(params, pose_seq)
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
    scores = eval_params(params, pose_seq)
    # print set
    for i in range(len(scores)):
        print i_gen+1, scores[i], " ".join([str(x) for x in params[i]])
    print ""
    print ""
    # check for new best
    max_score = process_best(max_score, output_prefix, scores, params, i_gen+1)
    sys.stdout.flush()
