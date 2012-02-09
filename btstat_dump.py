#!/usr/bin/env python
"""
SCL; Feb 2012.
"""

import sys
import pickle
import numpy as np

from btsynth import pretty_world, read_world, dump_world


USAGE = "Usage: %s FILE [n]" % sys.argv[0]
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print USAGE
        exit(1)
    with open(sys.argv[1], "r") as f:
        (times, world_data) = pickle.load(f)

    # Mark failures (caught exception during execution) to avoid
    # counting them as valid trial data.
    valid_trials = []
    for trial in range(len(world_data)):
        if times[trial][0] >= 0:
            valid_trials.append(trial)

    if len(sys.argv) >= 3:
        try:
            req_trial = int(sys.argv[2])
        except ValueError:
            print "Not a possible trial number."
            print USAGE
            exit(1)
        if req_trial < 0 or req_trial >= len(valid_trials):
            print "Requested trial number "+str(req_trial)+" is out of range (max is "+str(len(times)-1)+")."
            exit(1)
    else:
        req_trial = None  # Indicate we are interested in all trials

    if req_trial is not None:
        (W, goal_list, init_list, env_init_list) = read_world(world_data[req_trial][0])
        print "Trial "+str(req_trial)
        print "Nominal: %.4f; Global %.4f; Patching %.4f" % times[req_trial]
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        W[world_data[req_trial][1][0]][world_data[req_trial][1][1]] = 1
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        print "#"*60
        print "# NOMINAL"
        print world_data[req_trial][0]

        print "#"*60
        print "# ACTUAL"
        print dump_world(W, goal_list, init_list, env_init_list)

        exit(0)

    for trial in valid_trials:
        (W, goal_list, init_list, env_init_list) = read_world(world_data[trial][0])
        print "Trial "+str(trial)
        print "Nominal: %.4f; Global %.4f; Patching %.4f" % times[trial]
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        W[world_data[trial][1][0]][world_data[trial][1][1]] = 1
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        print "#"*60+"\n"

    # Statistics in summary
    N = len(valid_trials)
    lg_ratios = []
    for trial in valid_trials:
        if times[trial][2] < 0:
            # Local problem arrived at global
            times[trial] = (times[trial][0], times[trial][1], times[trial][1])
        lg_ratios.append(times[trial][2]/times[trial][1])
    lg_ratios = np.array(lg_ratios)
    
    print "N = "+str(N)
    print "mean and std dev of ratio of local-to-global time: %.4f, %.4f" % (np.mean(lg_ratios), np.std(lg_ratios))
    print lg_ratios
