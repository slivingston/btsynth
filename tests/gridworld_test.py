"""
Tests for gridworld-related routines.

SCL; 2011.
"""

from btsynth.btsynth import *


def extract_coord_test():
    assert extract_coord("test_3_0") == ("test", 3, 0)
    assert extract_coord("obstacle_5_4_11") == ("obstacle_5", 4, 11)
    assert extract_coord("test3_0") is None
