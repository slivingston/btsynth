#!/usr/bin/env python
"""
testing on gridworlds...

SCL; 2011 Aug, draft
"""

import tulip
#from tulip.grsim import grsim
import sys

from btrsynth import *


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
    (W_actual, goal_list_actual, init_list_actual, env_goal_list_actual, env_init_list_actual) = read_worldf(sys.argv[2])
    print "Nominal world:"
    print pretty_world(W, goal_list=goal_list, init_list=init_list,
                       env_init_list=env_init_list)
    print "Trying to solve..."
    aut = gen_navobs_soln(init_list=init_list,
                          goal_list=goal_list,
                          W=W, num_obs=num_obs,
                          env_init_list=env_init_list,
                          env_goal_list=env_goal_list)
    # aut = gen_dsoln(init_list=init_list,
    #                 goal_list=goal_list,
    #                 W=W)
    print "Resulting solution automaton M has %d nodes." % aut.size()
    aut.trimDeadStates()
    print "After trimming dead nodes, M has size %d" % aut.size()
    aut.writeDotFile(fname="tempsyn-ORIG.dot", hideZeros=True)

    # sim_history = grsim([aut], num_it=20, deterministic_env=False)
    # for step in sim_history:
    #     print str(step[0]) + "-> " + str(extract_autcoord(step[1])[0]) \
    #         + "; " + str(extract_autcoord(step[1], var_prefix="Y_0")[0])

    exit(0) #DEBUG

    print "Actual world:"
    print pretty_world(W_actual)

    print "sim and patch..."
    #(aut_patched, W_patched) = btsim_d(init_list[0], goal_list, aut, W_actual, num_steps=100)
    (aut_patched, W_patched) = btsim_navobs(init_list[0], goal_list, aut, W_actual,
                                            num_obs=num_obs, num_steps=100)
    
    aut_patched.writeDotFile(fname="tempsyn-PATCHED.dot", hideZeros=True)

    #history, intent = dsim(init_list[0], aut_patched, W_actual, num_it=100)
    history, intent, obs_poses = navobs_sim(init_list[0], aut_patched, W_actual,
                                            num_obs=num_obs, num_it=100)
    print intent
    print pretty_world(W_actual, goal_list=goal_list, init_list=init_list,
                       simresult=[history, intent, obs_poses])
