Installation
============

btsynth depends on `TuLiP <http://tulip-control.sourceforge.net/>`_
and `NumPy <http://numpy.org/>`_. To use some visualization routines,
you must also have `matplotlib <http://matplotlib.sourceforge.net/>`_
installed.

By default, we use the GR(1) synthesis tool `gr1c
<http://scottman.net/2012/gr1c>`_, or rather the interface to it
provided by TuLiP. Modification to use JTLV basically amounts to using
corresponding functions with name ending in "_JTLV" or setting the
appropriate flag argument when invoking functions. See docstrings for
details.

For the remainder of the installation procedure, it should suffice
to::

  $ python setup.py install

Given the experimental nature of the package, please consider
installing into a `virtualenv <http://www.virtualenv.org/>`_-built
environment.
