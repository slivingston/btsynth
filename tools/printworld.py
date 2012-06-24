#!/usr/bin/env python
"""
Print "pretty" diagrams, given world files.

SCL; Aug 2011, Feb 2012.
"""

import sys
from btsynth import pretty_world, read_worldf, image_world, random_world, dump_world

try:
    import matplotlib.pyplot as plt
except ImportError:
    print "WARNING: matplotlib unavailable."
    plt = None

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: %s [-r H W] [-i] [FILE] [...]" % sys.argv[0]
        exit(1)
    show_images = False

    if "-r" in sys.argv:  # Random map requested
        index = sys.argv.index("-r")
        width = int(sys.argv[index+2])
        height = int(sys.argv[index+1])
        (W, goal_list, init_list, env_init_list) = random_world((height, width), num_env=1)
        print pretty_world(W=W, goal_list=goal_list, init_list=init_list,
                           env_init_list=env_init_list,
                           show_grid=True)
        print "#"*60+"\n"
        print dump_world(W, goal_list, init_list, env_init_list)
        exit(0)

    if "-i" in sys.argv:
        if plt is None:
            raise Exception("matplotlib missing; image-drawing routines are unavailable.")
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
        elif plt is not None:
            plt.figure()
            image_world(W, goal_list, init_list, env_init_list,
                        show_grid=False)
            plt.title(sys.argv[k+1])
    if show_images and plt is not None:
        plt.show()
