#!/usr/bin/env python
"""
testing on gridworlds...

SCL; 2011 Aug, draft
"""

import tulip
from tulip.grsim import grsim
import sys

from btrsynth import *


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s FILE-NOM FILE-REAL" % sys.argv[0]
        exit(1)


    (W, goal_list, init_list) = read_worldf(sys.argv[1])
    (W_actual, goal_list_actual, init_list_actual) = read_worldf(sys.argv[2])
    print "Nominal world:"
    print pretty_world(W, goal_list=goal_list, init_list=init_list)
    print "Trying to solve..."
    # aut = gen_navobs_soln(init_list=init_list,
    #                       goal_list=goal_list,
    #                       W=W, num_obs=num_obs, env_goal_list=[(2,0)])
    aut = gen_dsoln(init_list=init_list,
                    goal_list=goal_list,
                    W=W)
    print "Resulting solution automaton M has %d nodes." % aut.size()
    aut.trimDeadStates()
    print "After trimming dead nodes, M has size %d" % aut.size()
    aut.writeDotFile(fname="tempsyn-ORIG.dot", hideZeros=True)

    # sim_history = grsim([aut], num_it=20, deterministic_env=False)
    # for step in sim_history:
    #     print str(step[0]) + "-> " + str(extract_autcoord(step[1])[0]) \
    #         + "; " + str(extract_autcoord(step[1], var_prefix="Y_0")[0])

    print "Actual world:"
    print pretty_world(W_actual)

    # print "dsim..."
    # simresult = dsim(init_list[0], aut, W_actual)
    # print "navobs_sim..."
    # simresult = navobs_sim(init_list[0], aut, W_actual, num_obs=num_obs)
    # print pretty_world(W_actual, simresult)
    
    print "sim and patch..."
    aut_patched = btsim_d(init_list[0], goal_list, aut, W_actual, num_steps=100)
    
    aut_patched[0].writeDotFile(fname="tempsyn-PATCHED.dot", hideZeros=True)
