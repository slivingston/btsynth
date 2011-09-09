#!/usr/bin/env python
"""
Print ``pretty'' diagrams, given world files.

SCL; 2011 Aug 26
"""

import matplotlib.pyplot as plt
import sys
from btrsynth import pretty_world, read_worldf, image_world


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: %s [-i] FILE [...]" % sys.argv[0]
        exit(1)
    show_images = False
    if "-i" in sys.argv:
        show_images = True
        sys.argv.remove("-i")
    for k in range(len(sys.argv)-1):
        (W, goal_list, init_list, env_init_list) = read_worldf(sys.argv[k+1])
        if not show_images:
            print sys.argv[k+1]
            print pretty_world(W=W, goal_list=goal_list, init_list=init_list,
                               env_init_list=env_init_list,
                               show_grid=True)
            print "#"*60+"\n"
        else:
            plt.figure()
            image_world(W, goal_list, init_list, env_init_list,
                        show_grid=False)
            plt.title(sys.argv[k+1])
    plt.show()
