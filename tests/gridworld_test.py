"""
Tests for gridworld-related routines.

SCL; 2011, 2012.
"""

import numpy as np
from btsynth.gridworld import *


def extract_coord_test():
    assert extract_coord("test_3_0") == ("test", 3, 0)
    assert extract_coord("obstacle_5_4_11") == ("obstacle_5", 4, 11)
    assert extract_coord("test3_0") is None

def dumpread_test():
    """Test dump_world and read_world by closing the loop."""
    W_str = """2 9
-
4
I 0 0
G 1 8
G 0 1
E 0 6
"""
    (W, goal_list, init_list, env_list) = read_world(W_str)
    (dW, dgoal_list, dinit_list, denv_list) = read_world(dump_world(W, goal_list, init_list, env_list))
    assert np.all(W == dW)
    assert goal_list == dgoal_list
    assert init_list == dinit_list
    assert env_list == denv_list
