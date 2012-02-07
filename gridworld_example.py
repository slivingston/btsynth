#!/usr/bin/env python
"""
testing on gridworlds...

SCL; Aug, Sep 2011; Feb 2012.
"""

import tulip
import tulip.congexf as cg
import sys

from btsynth.btsynth import *
from btsynth.automaton import BTAutomaton


if __name__ == "__main__":
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print "Usage: %s FILE-NOM FILE-REAL [nominal-solution]" % sys.argv[0]
        exit(1)

    # Parameters
    num_obs = 1

    (W, goal_list, init_list, env_init_list) = read_worldf(sys.argv[1])
    # The "..._actual" naming scheme is a bit obnoxious.
    (W_actual, goal_list_actual, init_list_actual, env_init_list_actual) = read_worldf(sys.argv[2])

    print "Nominal world:"
    print pretty_world(W, goal_list=goal_list, init_list=init_list,
                       env_init_list=env_init_list)

    if len(sys.argv) >= 4:
        aut = BTAutomaton(sys.argv[3])
        print "Loaded (nominal) solution automaton M has %d nodes." % aut.size()
        # aut.trimDeadStates()
        # print "After trimming dead nodes, M has size %d" % aut.size()

    else:
        print "Trying to solve..."
        aut = gen_navobs_soln(init_list=init_list, goal_list=goal_list,
                              W=W, num_obs=len(env_init_list),
                              env_init_list=env_init_list, restrict_radius=1)
        print "Resulting solution automaton M has %d nodes." % aut.size()
        aut.trimDeadStates()
        print "After trimming dead nodes, M has size %d" % aut.size()

    
    aut.writeDotFileCoord("tempsyn-ORIG.dot")
    with open("tempsyn-ORIG.gexf", "w") as f:
        f.write(cg.dumpGexf(aut, use_viz=True, use_clusters=True))

    # sim_history = grsim([aut], num_it=20, deterministic_env=False)
    # for step in sim_history:
    #     print str(step[0]) + "-> " + str(extract_autcoord(step[1])[0]) \
    #         + "; " + str(extract_autcoord(step[1], var_prefix="Y_0")[0])

    print "Actual world:"
    print pretty_world(W_actual, goal_list=goal_list, init_list=init_list,
                       env_init_list=env_init_list)

    print "sim and patch..."
    (aut_patched, W_patched) = btsim_navobs(init_list[0], goal_list,
                                            aut, W_actual,
                                            env_init_list=env_init_list,
                                            num_steps=100)
    if aut_patched is None:
        print "Global problem recovered while attempting to patch."
    else:
        aut_patched.writeDotFileCoord("tempsyn-PATCHED.dot")
        with open("tempsyn-PATCHED.gexf", "w") as f:
            f.write(cg.dumpGexf(aut_patched, use_viz=True, use_clusters=True))
        print "Size after patching (in nodes):", aut_patched.size()

        history, intent, obs_poses = navobs_sim(init_list[0], aut_patched, W_actual,
                                                num_obs=num_obs, num_it=300)
        print intent
        print pretty_world(W_actual, goal_list=goal_list, init_list=init_list,
                           env_init_list=env_init_list,
                           simresult=[history, intent, obs_poses])

        print history
