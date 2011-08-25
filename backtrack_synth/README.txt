Gridworld notes
===============

World format:
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
with a I, in which case it indicates a possible start location; if a
row does not have any obstacles, then its corresponding line should
contain a single "-".  E.g. a world that looks like

-----
|* *|
|  *|
|   |
-----

with goal location in the bottom-right cell would be described by

3 3
0 2
2
-
G 2 2
