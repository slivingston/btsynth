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
import sys

from btrsynth import *


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s FILE-NOM FILE-REAL" % sys.argv[0]
        exit(1)

    init = (0,0)
    goal_list = [(0,3), (5,0)]

    W = read_worldf(sys.argv[1])
    W_actual = read_worldf(sys.argv[2])
    print "Nominal world:"
    print pretty_world(W)
    print "Trying to solve..."
    aut = gen_dsoln(init_list=[init,],
                    goal_list=goal_list,
                    W=W)
    aut.writeDotFile(fname="tempsyn.dot", hideZeros=True)

    print "Actual world:"
    print pretty_world(W_actual)

    print "dsim..."
    simresult = dsim(init, aut, W_actual)
    print simresult
    print pretty_world(W_actual, simresult)
    
    aut.writeDotFile(fname="tempsyn.dot", hideZeros=True)
