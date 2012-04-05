#!/usr/bin/env python
"""
Concatenate files containing trial data.

SCL; Feb 2012.
"""

import sys
import pickle


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: %s FILE1 [...] OUTPUT" % sys.argv[0]
        exit(1)

    times = []
    world_data = []
    for file_index in range(len(sys.argv)-2):
        print "Reading %s..." % sys.argv[file_index+1]
        with open(sys.argv[file_index+1], "r") as f:
            (this_times, this_world_data) = pickle.load(f)
        times.extend(this_times)
        world_data.extend(this_world_data)
    print "Writing combined set to %s..." % sys.argv[-1]
    with open(sys.argv[-1], "w") as f:
        pickle.dump((times, world_data), f)
