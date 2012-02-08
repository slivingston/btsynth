#!/usr/bin/env python
"""
SCL; Feb 2012.
"""

import sys
import pickle

from btsynth import pretty_world, read_world


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: %s FILE" % sys.argv[0]
        exit(1)
    with open(sys.argv[1], "r") as f:
        (times, world_data) = pickle.load(f)

    for trial in range(len(world_data)):
        (W, goal_list, init_list, env_init_list) = read_world(world_data[trial][0])
        print "Trial "+str(trial)
        print "Nominal: %.4f; Global %.4f; Patching %.4f" % times[trial]
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        W[world_data[trial][1][0]][world_data[trial][1][1]] = 1
        print pretty_world(W, goal_list, init_list, env_init_list, show_grid=True)
        print "#"*60+"\n"
