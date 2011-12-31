"""
Tests for functions not otherwise specified.

SCL; 2011.
"""

from btsynth.btsynth import *


def prefix_filt_test():
    assert prefix_filt({"Y_0_0": 0, "Y_0_1": 1, "X_0_1_0": 1}, "Y") == {"Y_0_0": 0, "Y_0_1": 1}
