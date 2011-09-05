#!/usr/bin/env python
"""
testing on deterministic (non-adversarial) gridworlds...

SCL; 2011 Aug, draft
"""

import tulip
import sys

from btrsynth.btrsynth import *
from btrsynth.automaton import BTAutomaton


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s FILE-NOM FILE-REAL" % sys.argv[0]
        exit(1)

    # Parameters
    num_obs = 1
    env_goal_list=[(2,0)]

    (W, goal_list, init_list, tmp, env_init_list) = read_worldf(sys.argv[1])
    del tmp
    # The "..._actual" naming scheme is a bit obnoxious.
    (W_actual, goal_list_actual, init_list_actual, tmp, env_init_list_actual) = read_worldf(sys.argv[2])
    del tmp

    print "Nominal world:"
    print pretty_world(W, goal_list=goal_list, init_list=init_list,
                       env_init_list=env_init_list)
    print "Trying to solve..."
    aut = gen_dsoln(init_list=init_list,
                    goal_list=goal_list,
                    W=W)
    print "Resulting solution automaton M has %d nodes." % aut.size()
    aut.trimDeadStates()
    print "After trimming dead nodes, M has size %d" % aut.size()
    aut.writeDotFileCoord(fname="tempsyn-ORIG.dot")

    print "Actual world:"
    print pretty_world(W_actual)

    print "sim and patch..."
    (aut_patched, W_patched) = btsim_d(init_list[0], goal_list, aut, W_actual, num_steps=100)
    
    aut_patched.writeDotFileCoord(fname="tempsyn-PATCHED.dot")

    history, intent = dsim(init_list[0], aut_patched, W_actual, num_it=100)
    
    print intent
    print pretty_world(W_actual, goal_list=goal_list, init_list=init_list,
                       simresult=[history, intent])
