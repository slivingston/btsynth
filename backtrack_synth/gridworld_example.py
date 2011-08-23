#!/usr/bin/env python
"""
World format:
1: R C
2: ...

Any line beginning with "#" is treated as a comment line and ignored.
Blank lines are ignored.  First (non-comment, non-blank) line is
number of rows and columns.  Second and all remaining lines indicate
which columns are occupied for that row; if a row does not have any
obstacles, then its corresponding line should contain a single "-".
E.g. a world that looks like

-----
|* *|
|  *|
|   |
-----

would be described by

3 3
0 2
2
-

SCL; 2011 Aug 22, draft
"""

import tulip
from tulip.grsim import grsim
import sys

from btrsynth import *


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s FILE-NOM FILE-REAL" % sys.argv[0]
        exit(1)

    init = (2,2)
    goal_list = [(2,2)]
    num_obs = 1

    W = read_worldf(sys.argv[1])
    W_actual = read_worldf(sys.argv[2])
    print "Nominal world:"
    print pretty_world(W)
    print "Trying to solve..."
    aut = gen_navobs_soln(init_list=[init],
                          goal_list=goal_list,
                          W=W, num_obs=num_obs, env_goal_list=[(2,0)])
    # aut = gen_dsoln(init_list=[init,],
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

    print "Actual world:"
    print pretty_world(W_actual)

    # print "dsim..."
    # simresult = dsim(init, aut, W_actual)
    print "navobs_sim..."
    simresult = navobs_sim(init, aut, W_actual, num_obs=num_obs)

    print pretty_world(W_actual, simresult)
    
    # aut.writeDotFile(fname="tempsyn-PATCHED.dot", hideZeros=True)
