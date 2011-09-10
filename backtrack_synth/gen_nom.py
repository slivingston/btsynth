#!/usr/bin/env python
"""
command-line wrapper for function btrsynth.create_nominal.

SCL; 2011 Sep, draft
"""

import sys

from btrsynth.btrsynth import *
from btrsynth.automaton import BTAutomaton


if __name__ == "__main__":
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print "Usage: %s [-v] WORLD FILE" % sys.argv[0]
        exit(1)

    obnoxious_flag = False
    if "-v" in sys.argv:
        obnoxious_flag = True
        sys.argv.remove("-v")
    
    (W, goal_list, init_list, env_init_list) = read_worldf(sys.argv[1])
    nomstr_list = []
    with open(sys.argv[2], "r") as f:
        for line in f:
            nomstr_list.append(line)
    aut = create_nominal(W=W, env_init_list=env_init_list,
                         soln_str="\n".join(nomstr_list))
    print "number of states:", aut.size()
    aut.writeFile("tempnom.aut")
    if obnoxious_flag:
        aut.writeDotFile("tempnom.dot")
    else:
        aut.writeDotFileCoord("tempnom.dot")
    
