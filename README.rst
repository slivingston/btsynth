Introduction
============

Scott C. Livingston  <slivingston@caltech.edu>

**btsynth** is an implementation of the ``backtracking synthesis''
algorithm, currently only applied to gridworlds (see below for data
file definition). The dependencies include `TuLiP
<http://tulip-control.sourceforge.net/>`_ and `NumPy <http://numpy.org/>`_.

Some functionality remains undocumented, but a good start is
``printworld.py`` for viewing gridworld data files, and
``gridworld.py`` for running examples. ``dgridworld.py`` is the
deterministic case, i.e. no adversarial environment.  E.g., try::

  $ ./printworld.py example_data/paper4{,_real}.world
  $ ./gridworld_example.py example_data/paper4{,_real}.world

The code includes an extension to the Automaton class defined in
`TuLiP <http://tulip-control.sourceforge.net>`_. Examples of new
features are finite memory and switched transitions dependent on the
state of this memory.


Gridworld notes
===============

World format:

::

  1: r c
  2: ...
  3: G r c
  4: ...
  5: I r c
  6: ...

Any line beginning with "#" is treated as a comment line and ignored.
Blank lines are ignored.  First (non-comment, non-blank) line is
number of rows and columns.  Second and all remaining lines indicate
which columns are occupied for that row, unless the line begins with a
"G", in which case it indicates a goal location, or the line begins
with a I, in which case it indicates a possible start location; valid
start locations for environment are indicated by "E"; if a row does
not have any obstacles, then its corresponding line should contain a
single "-".  E.g. a world that looks like

::

  -----
  |* *|
  |  *|
  |   |
  -----

with goal location in the bottom-right cell would be described by

::

  3 3
  0 2
  2
  -
  G 2 2
