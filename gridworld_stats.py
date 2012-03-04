#!/usr/bin/env python
"""
Gather performance statistics from solving and patching in random
gridworlds.

SCL; Feb 2012.
"""

import tulip
import tulip.congexf as cg
import sys
import pickle

from btsynth import *
from btsynth.automaton import BTAutomaton

# Profiling
from cProfile import Profile


USAGE = "Usage: %s FILE N H W DENSITY NUMENV" % sys.argv[0]
if __name__ == "__main__":
    if len(sys.argv) != 7:
        print USAGE
        exit(1)

    # Parameters
    max_blocking_tries = 10
    try:
        num_games = int(sys.argv[2])
        height = int(sys.argv[3])
        width = int(sys.argv[4])
        bdensity = float(sys.argv[5])
        num_env = int(sys.argv[6])
    except ValueError:
        print USAGE
        exit(1)

    times = []
    worlds_data = []
    game_count = 0
    while game_count < num_games:
        (W, goal_list, init_list, env_init_list) = random_world(size=(height, width),
                                                                wall_density=bdensity,
                                                                num_goals=2,
                                                                num_env=num_env)

        print "Nominal world:"
        print pretty_world(W, goal_list=goal_list, init_list=init_list,
                           env_init_list=env_init_list)

        print "Trying to solve..."
        nsprof = Profile()
        nsprof.run("aut = gen_navobs_soln(init_list=init_list, goal_list=goal_list, W=W, num_obs=len(env_init_list), env_init_list=env_init_list, restrict_radius=1)")
        if aut is None:
            print "Nominal spec not feasible."
            print "#"*60
            continue
        
        print "Resulting solution automaton M has %d nodes." % aut.size()
        aut.trimDeadStates()
        print "After trimming dead nodes, M has size %d" % aut.size()

        ind = -1
        while not hasattr(nsprof.getstats()[ind].code, "co_name") or (nsprof.getstats()[ind].code.co_name != "gen_navobs_soln"):
            ind -= 1
        nom_time = nsprof.getstats()[ind].totaltime

        aut.writeDotFileCoord("tempsyn-ORIG.dot")
        with open("tempsyn-ORIG.gexf", "w") as f:
            f.write(cg.dumpGexf(aut, use_viz=True, use_clusters=True))

        # Place block randomly in way of nominal plan (so that
        # patching is indeed necessary).
        W_actual = W.copy()
        block_try_count = 0
        while True:
            block_try_count += 1
            if block_try_count > max_blocking_tries:
                break
            
            avail_ind = np.array(range(W.shape[0]*W.shape[1]))[(W==0).flatten()]
            avail_ind = [(k/W.shape[1], k%W.shape[1]) for k in avail_ind]
            avail_ind = [k for k in avail_ind if (k not in init_list) and (k not in env_init_list) and (k not in goal_list)]
            block_ind = avail_ind[np.random.randint(low=0, high=len(avail_ind))]
            W_actual[block_ind[0]][block_ind[1]] = 1
            (history, intent, obs_poses) = navobs_sim(init_list[0], aut,
                                                      W_actual,
                                                      num_obs=len(env_init_list),
                                                      var_prefix="Y",
                                                      env_prefix="X",
                                                      num_it=1000)
            if intent is True:
                W_actual[block_ind[0]][block_ind[1]] = 0
                continue

            globalprof = Profile()
            globalprof.run("aut_global = gen_navobs_soln(init_list=init_list, goal_list=goal_list, W=W_actual, num_obs=len(env_init_list), env_init_list=env_init_list, restrict_radius=1)")
            if aut_global is None:
                W_actual[block_ind[0]][block_ind[1]] = 0
                continue
            else:
                print "resulting solution automaton m has %d nodes." % aut_global.size()
                aut_global.trimDeadStates()
                print "after trimming dead nodes, m has size %d" % aut_global.size()

                ind = -1
                while not hasattr(globalprof.getstats()[ind].code, "co_name") or (globalprof.getstats()[ind].code.co_name != "gen_navobs_soln"):
                    ind -= 1
                global_time = globalprof.getstats()[ind].totaltime

                aut_global.writeDotFileCoord("tempsyn-global.dot")
                with open("tempsyn-global.gexf", "w") as f:
                    f.write(cg.dumpGexf(aut_global,
                                        use_viz=True, use_clusters=True))
                break

        if block_try_count > max_blocking_tries:
            print "Failed to find block that interrupts nominal plan."
            print "#"*60
            continue

        print "Actual world:"
        print pretty_world(W_actual, goal_list=goal_list, init_list=init_list,
                           env_init_list=env_init_list)

        print "sim and patch..."
        btprof = Profile()
        try:
            btprof.run("(aut_patched, W_patched) = btsim_navobs(init_list[0], goal_list, aut, W_actual, env_init_list=env_init_list, num_steps=100)")
        except:
            print "Error: exception caught during call to btsim_navobs;"
            print "dropping this trial, and inserting marked copy into results list."
            times.append((-1, -1, -1))
            worlds_data.append((dump_world(W, goal_list, init_list, env_init_list), block_ind))
            continue
        if aut_patched is None:
            patch_time = -1  # -1 time indicates global problem recovered
        else:
            ind = -1
            while not hasattr(btprof.getstats()[ind].code, "co_name") or (btprof.getstats()[ind].code.co_name != "btsim_navobs"):
                ind -= 1
            patch_time = btprof.getstats()[ind].totaltime

            aut_patched.writeDotFileCoord("tempsyn-PATCHED.dot")
            with open("tempsyn-PATCHED.gexf", "w") as f:
                f.write(cg.dumpGexf(aut_patched,
                                    use_viz=True, use_clusters=True))
            print "Size after patching (in nodes):", aut_patched.size()

        game_count += 1
        times.append((nom_time, global_time, patch_time))
        worlds_data.append((dump_world(W, goal_list, init_list, env_init_list), block_ind))
        print "Backing up progress..."
        with open(sys.argv[1], "w") as f:
            pickle.dump((times, worlds_data), f)

    print "\n".join([str(nom_time)+", "+str(global_time)+", "+str(patch_time) for (nom_time, global_time, patch_time) in times])
    with open(sys.argv[1], "w") as f:
        pickle.dump((times, worlds_data), f)
