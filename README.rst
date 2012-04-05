Introduction
============

Scott C. Livingston  <slivingston@caltech.edu>

**btsynth** is an implementation of the ``backtracking synthesis''
algorithm, currently only applied to gridworlds. The dependencies
include `TuLiP <http://tulip-control.sourceforge.net/>`_ and `NumPy
<http://numpy.org/>`_.

Some functionality remains undocumented, but a good start is
``printworld.py`` for viewing gridworld data files, and
``gridworld_example.py`` for running
examples. ``dgridworld_example.py`` is the deterministic case, i.e.,
no adversarial environment.  E.g., try::

  $ tools/printworld.py examples/data/paper4{,_real}.world
  $ examples/gridworld_example.py examples/data/paper4{,_real}.world

To generate a random gridworld problem of size 4 by 10, try::

  $ tools/printworld.py -r 4 10

The output is a pretty visualization and a world code that can be
saved to a plaintext file for later use (see below).  The problem thus
generated may not be feasible.  A simple-minded but correct way to
automatically test feasibility is to attempt global synthesis on it by
calling gen_navobs_soln (see ``gridworld_example.py`` for example
usage).

The code includes an extension to the Automaton class defined in
`TuLiP <http://tulip-control.sourceforge.net>`_. Examples of new
features are finite memory and switched transitions dependent on the
state of this memory.

To test btsynth, from the root directory run::

  $ nosetests

More detailed documentation is under ``doc`` directory, written in
reStructuredText for Sphinx.  A snapshot of this documentation will
occasionally be posted at http://vehicles.caltech.edu/scott/btsynth/
