#!/usr/bin/env python
"""
Various routines for working with gridworlds.

Scott C. Livingston,
2011, 2012.
"""

import itertools
import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as mplt_cm
except ImportError:
    print "WARNING: matplotlib unavailable."
    plt = None
    mplt_cm = None


def errmsg(m):
    print "ERROR: "+m


def dump_world(W, goal_list=[], init_list=[], env_list=[]):
    """Inverse of read_world.  Returns a string."""
    output = str(W.shape[0])+" "+str(W.shape[1])+"\n"
    for i in range(W.shape[0]):
        if np.any(W[i][:] != 0):
            for j in range(W.shape[1]):
                if W[i][j] != 0:
                    output += str(j) + " "
            output += "\n"
        else:
            output += "-\n"
    output += "\n".join(["G "+str(i)+" "+str(j) for (i, j) in goal_list]) + "\n"
    output += "\n".join(["I "+str(i)+" "+str(j) for (i, j) in init_list]) + "\n"
    output += "\n".join(["E "+str(i)+" "+str(j) for (i, j) in env_list]) + "\n"
    return output


def random_world(size, wall_density=.2, num_goals=2, num_env=0):
    """Randomly generated problem world. Same return value as read_world.
    
    size is a pair, indicating number of rows and columns.
    wall_density is the ratio of walls to total number of cells.
    """
    num_cells = size[0]*size[1]
    goal_list = []
    env_list = []
    W = np.zeros(num_cells, dtype=np.uint8)
    num_blocks = int(np.round(wall_density*num_cells))
    for i in range(num_blocks):
        avail_inds = np.array(range(num_cells))[W==0]
        W[avail_inds[np.random.randint(low=0, high=len(avail_inds))]] = 1
    for i in range(num_goals):
        avail_inds = np.array(range(num_cells))[W==0]
        avail_inds = [k for k in avail_inds if k not in goal_list]
        goal_list.append(avail_inds[np.random.randint(low=0, high=len(avail_inds))])
    for i in range(num_env):
        avail_inds = np.array(range(num_cells))[W==0]
        avail_inds = [k for k in avail_inds if (k not in goal_list) and (k not in env_list)]
        env_list.append(avail_inds[np.random.randint(low=0, high=len(avail_inds))])
    avail_inds = np.array(range(num_cells))[W==0]
    avail_inds = [k for k in avail_inds if (k not in goal_list) and (k not in env_list)]
    init_list = [avail_inds[np.random.randint(low=0, high=len(avail_inds))]]
    W = W.reshape(size)
    goal_list = [(k/size[1], k%size[1]) for k in goal_list]
    env_list = [(k/size[1], k%size[1]) for k in env_list]
    init_list = [(k/size[1], k%size[1]) for k in init_list]
    return (W, goal_list, init_list, env_list)


def read_world(world_str):
    """Process string as World, return matrix, and goal, init lists for sys and env.

    The returned uint8 NumPy matrix (nd-array) has 0 or 1 value
    entries, with 0 being unoccupied, 1 occupied.  On error, return
    None.  Goal and init lists are built from the file and also
    returned. If no goals (resp. initial locations) found, then empty
    lists are returned.

    Summarizing, the return tuple looks like (W, goal_list, init_list, env_init_list).

    Assume lines are separated by '\n'; note that read_worldf ensures
    this before invoking read_world.
    """
    first_line_found = False
    line_counter = 0
    goal_list = []
    init_list = []
    env_init_list = []
    empty_return = (None, None, None, None, None)
    world = np.array([], dtype=np.uint8)  # In case given string is lame.
    for line in world_str.split("\n"):
        line_counter += 1
        if len(line) == 0 or line[0] == "#":
            continue
        if not first_line_found:
            first_line_found = True
            entries = line.split()
            try:
                if len(entries) != 2:
                    raise ValueError
                num_rows = int(entries[0])
                num_cols = int(entries[1])
            except ValueError:
                errmsg("Malformed size spec at line "+str(line_counter))
                return empty_return
            world = np.zeros(shape=(num_rows, num_cols), dtype=np.uint8)
            row_counter = 0
        else:
            if row_counter > num_rows:
                errmsg("Too many rows! (as of line "+str(line_counter)+")")
                return empty_return
            if ("G" in line) or ("g" in line):  # Sys Goal line?
                val_strlist = line.split()
                if len(val_strlist) != 3:
                    errmsg("Malformed goal spec at line "+str(line_counter))
                    return empty_return
                try:
                    g_row = int(val_strlist[1])
                    if (g_row < 0) or (g_row >= num_rows):
                        raise ValueError
                    g_col = int(val_strlist[2])
                    if (g_col < 0) or (g_col >= num_cols):
                        raise ValueError
                except ValueError:
                    errmsg("Malformed goal spec at line "+str(line_counter))
                    return empty_return
                goal_list.append((g_row, g_col))
                continue
            if ("I" in line) or ("i" in line):  # Sys Initialization line?
                val_strlist = line.split()
                if len(val_strlist) != 3:
                    errmsg("Malformed init spec at line "+str(line_counter))
                    return empty_return
                try:
                    i_row = int(val_strlist[1])
                    if (i_row < 0) or (i_row >= num_rows):
                        raise ValueError
                    i_col = int(val_strlist[2])
                    if (i_col < 0) or (i_col >= num_cols):
                        raise ValueError
                except ValueError:
                    errmsg("Malformed init spec at line "+str(line_counter))
                    return empty_return
                init_list.append((i_row, i_col))
                continue
            if ("E" in line) or ("e" in line):  # Env Initialization line?
                val_strlist = line.split()
                if len(val_strlist) != 3:
                    errmsg("Malformed env init spec at line "+str(line_counter))
                    return empty_return
                try:
                    i_row = int(val_strlist[1])
                    if (i_row < 0) or (i_row >= num_rows):
                        raise ValueError
                    i_col = int(val_strlist[2])
                    if (i_col < 0) or (i_col >= num_cols):
                        raise ValueError
                except ValueError:
                    errmsg("Malformed env init spec at line "+str(line_counter))
                    return empty_return
                env_init_list.append((i_row, i_col))
                continue
            if line[0] == "-":  # Empty row?
                row_counter += 1
                continue
            for col_str in line.split():
                try:
                    col = int(col_str)
                    if (col < 0) or (col >= num_cols):
                        raise ValueError
                except ValueError:
                    errmsg("Malformed row spec at line "+str(line_counter))
                    return empty_return
                world[row_counter][col] = 1
            row_counter += 1
    return world, goal_list, init_list, env_init_list

def read_worldf(fname):
    """File wrapper for read_world function."""
    wstr_list = []
    with open(fname, "r") as f:
        for line in f:
            wstr_list.append(line)
    return read_world("\n".join(wstr_list))

def image_world(W, goal_list=[], init_list=[], env_init_list=[],
                show_grid=False, grid_width=2):
    """Like pretty_world, but now generate and show a matplotlib image.
    """
    if plt is None:
        raise Exception("matplotlib missing; image-drawing routines are unavailable.")
    W = W.copy()
    W = np.ones(shape=W.shape) - W
    plt.imshow(W, cmap=mplt_cm.gray, aspect="equal", interpolation="nearest")
    if show_grid:
        xmin, xmax, ymin, ymax = plt.axis()
        x_steps = np.linspace(xmin, xmax, W.shape[1]+1)
        y_steps = np.linspace(ymin, ymax, W.shape[0]+1)
        for k in x_steps:
            plt.plot([k, k], [ymin, ymax], 'k-', linewidth=grid_width)
        for k in y_steps:
            plt.plot([xmin, xmax], [k, k], 'k-', linewidth=grid_width)
        plt.axis([xmin, xmax, ymin, ymax])

def pretty_world(W, goal_list=[], init_list=[], env_init_list=[],
                 simresult=None,
                 show_grid=False):
    """Given world matrix W, return pretty-for-printing string.

    If simresult is not None, it should be a pair consisting of a list
    of locations (signifying a path) and a failed-but-desired end
    location.  These are exactly returned by the function dsim.  If
    simresult contains a third element, it is regarded as a list of
    obstacle locations.  Also, if the second element is True, then we
    assume a failure did not occur and print the last vehicle location
    as a "O".

    If goal_list or init_list are nonempty, "G" and "I" symbols are
    inserted into the pretty map as appropriate.

    If env_init_list is nonempty, "E" is inserted where appropriate.
    
    If show_grid is True, then grid the pretty world and show row and
    column labels along the outer edges.

    Return None on failure (or lame arguments).
    """
    if W is None:
        return None
    W = W.copy()  # Think globally, act locally.
    # Fill in W with magic values if simresult is given.
    # LEGEND:
    #    1 - "*" wall (as used in original world matrix definition);
    #    2 - "." regularly visited location;
    #    3 - "X" desired but failed location, was occupied;
    #    4 - "?" desired but failed location, was not occupied;
    #    5 - "O" last location, no failure;
    #    6 - "!" obstacle (dynamic, adversarial);
    #   10 - "G" goal location;
    #   11 - "I" possible initial location.
    #   12 - "E" possible initial location.
    if simresult is not None:
        for loc in simresult[0]:
            if W[loc[0]][loc[1]] != 0 and W[loc[0]][loc[1]] != 2:
                raise ValueError("Mismatch between given simresult and world at " \
                                     + "("+str(loc[0])+", "+str(loc[1])+")")
            W[loc[0]][loc[1]] = 2
        if simresult[1] is True:
            W[simresult[0][-1][0]][simresult[0][-1][1]] = 5
        else:
            if W[simresult[1][0]][simresult[1][1]] == 0:
                W[simresult[1][0]][simresult[1][1]] = 4
            else:
                W[simresult[1][0]][simresult[1][1]] = 3
        if len(simresult) > 2:
            for loc in simresult[2]:
                W[loc[0]][loc[1]] = 6
    if show_grid:
        out_str = "  " + "".join([str(k).rjust(2) for k in range(W.shape[1])]) + "\n"
    else:
        out_str = "-"*(W.shape[1]+2) + "\n"
    if (goal_list is not None) and len(goal_list) > 0:
        for loc in goal_list:
            W[loc[0]][loc[1]] = 10
    if (init_list is not None) and len(init_list) > 0:
        for loc in init_list:
            W[loc[0]][loc[1]] = 11
    if (env_init_list is not None) and len(env_init_list) > 0:
        for loc in env_init_list:
            W[loc[0]][loc[1]] = 12
    for i in range(W.shape[0]):
        if show_grid:
            out_str += "  " + "-"*(W.shape[1]*2+1) + "\n"
            out_str += str(i).rjust(2)
        else:
            out_str += "|"
        for j in range(W.shape[1]):
            if show_grid:
                out_str += "|"
            if W[i][j] == 0:
                out_str += " "
            elif W[i][j] == 1:
                out_str += "*"
            elif W[i][j] == 2:
                out_str += "."
            elif W[i][j] == 3:
                out_str += "X"
            elif W[i][j] == 4:
                out_str += "?"
            elif W[i][j] == 5:
                out_str += "O"
            elif W[i][j] == 6:
                out_str += "!"
            elif W[i][j] == 10:
                out_str += "G"
            elif W[i][j] == 11:
                out_str += "I"
            elif W[i][j] == 12:
                out_str += "E"
            else:
                raise ValueError("Unrecognized world W encoding.")
        out_str += "|\n"
    if show_grid:
        out_str += "  " + "-"*(W.shape[1]*2+1) + "\n"
    else:
        out_str += "-"*(W.shape[1]+2) + "\n"
    return out_str


def LTL_world(W, var_prefix="obs",
              center_loc=None, restrict_radius=1):
    """Convert world matrix W into an LTL formula describing transitions.
    
    Syntax is that of gr1c; in particular, "next" variables are
    primed. For example, x' refers to the variable x at the next time
    step.

    The variables are named according to var_prefix_R_C, where
    var_prefix is given, R is the row, and column the cell
    (0-indexed).

    If center_loc is None (default), then object can occupy any open
    cell in the given world matrix.  If center_loc is a pair, then it
    indicates a location (row, column) in the world matrix and
    combined with the argument restrict_radius (same units as world
    matrix; 1 == 1 step by row or column), a restricted region of
    movement for the object is specified.  If part of the restricted
    region is outside the given world, then we must allow a movement
    to nowhere (i.e. ``out of the world W'') and movements back into
    W, where such transitions can occur along the boundary of W at
    which the restricted region leaves W.  Since the restricted region
    is compact and connected, transitions to/from nowhere can happen
    anywhere along this boundary (so, you might view ``nowhere'' as a
    special location connected to all this overflow positions).

    ``nowhere'' has the position (-1, -1).

    Return the formula as a list of transition rules, which can be
    used directly in building a specification in a GRSpec object
    (defined in spec module of TuLiP).  Return None if failure.
    """
    always_nowhere_flag = False
    nowhere = [False,  # corresponds to row_low
               False,  # row_high
               False,  # col_low
               False]  # col_high
    if center_loc is not None:
        row_low = center_loc[0]-restrict_radius
        if row_low < 0:
            row_low = 0
            nowhere[0] = True
        row_high = center_loc[0]+restrict_radius
        if row_high > W.shape[0]-1:
            row_high = W.shape[0]-1
            nowhere[1] = True
        col_low = center_loc[1]-restrict_radius
        if col_low < 0:
            col_low = 0
            nowhere[2] = True
        col_high = center_loc[1]+restrict_radius
        if col_high > W.shape[1]-1:
            col_high = W.shape[1]-1
            nowhere[3] = True
        # Special case:
        if (row_low > W.shape[0]-1 or row_high < 0
            or col_low > W.shape[1]-1 or col_high < 0):
            always_nowhere_flag = True
    else:
        row_low = 0
        row_high = W.shape[0]-1
        col_low = 0
        col_high = W.shape[1]-1
    out_trans = []
    # Safety, transitions
    for i in range(row_low, row_high+1):
        for j in range(col_low, col_high+1):
            if W[i][j] == 1:
                continue  # Cannot start from an occupied cell.
            out_trans.append(var_prefix+"_"+str(i)+"_"+str(j)+" -> (")
            # Normal transitions:
            out_trans[-1] += var_prefix+"_"+str(i)+"_"+str(j)+"'"
            if i > row_low and W[i-1][j] == 0:
                out_trans[-1] += " | " + var_prefix+"_"+str(i-1)+"_"+str(j)+"'"
            if j > col_low and W[i][j-1] == 0:
                out_trans[-1] += " | " + var_prefix+"_"+str(i)+"_"+str(j-1)+"'"
            if i < row_high and W[i+1][j] == 0:
                out_trans[-1] += " | " + var_prefix+"_"+str(i+1)+"_"+str(j)+"'"
            if j < col_high and W[i][j+1] == 0:
                out_trans[-1] += " | " + var_prefix+"_"+str(i)+"_"+str(j+1)+"'"
            # Transitions to ``nowhere'':
            if ((i == row_low and nowhere[0])
                or (i == row_high and nowhere[1])
                or (j == col_low and nowhere[2])
                or (j == col_high and nowhere[3])):
                out_trans[-1] += " | " + var_prefix+"_n_n'"
            out_trans[-1] += ")"
    if nowhere[0] or nowhere[1] or nowhere[2] or nowhere[3]:
        # Add transitions from ``nowhere''
        out_trans.append(var_prefix+"_n_n"+" -> (")
        out_trans[-1] += var_prefix+"_n_n'"
        if not always_nowhere_flag:
            if nowhere[0]:
                for j in range(col_low, col_high+1):
                    out_trans[-1] += " | " + var_prefix+"_"+str(row_low)+"_"+str(j)+"'"
            if nowhere[1]:
                for j in range(col_low, col_high+1):
                    out_trans[-1] += " | " + var_prefix+"_"+str(row_high)+"_"+str(j)+"'"
            if nowhere[2]:
                for i in range(row_low, row_high+1):
                    out_trans[-1] += " | " + var_prefix+"_"+str(i)+"_"+str(col_low)+"'"
            if nowhere[3]:
                for i in range(row_low, row_high+1):
                    out_trans[-1] += " | " + var_prefix+"_"+str(i)+"_"+str(col_high)+"'"
        out_trans[-1] += ")"
    
    # Safety, static
    for i in range(row_low, row_high+1):
        for j in range(col_low, col_high+1):
            if W[i][j] == 1:
                out_trans.append("!(" + var_prefix+"_"+str(i)+"_"+str(j)+"'" + ")")

    # Safety, mutex
    first_subformula = True
    out_trans.append("")
    pos_indices = [k for k in itertools.product(range(row_low, row_high+1), range(col_low, col_high+1))]
    if nowhere[0] or nowhere[1] or nowhere[2] or nowhere[3]:
        pos_indices.append((-1, -1))
    for outer_ind in pos_indices:
        if outer_ind != (-1, -1) and W[outer_ind[0]][outer_ind[1]] == 1:
            continue
        if not first_subformula:
            out_trans[-1] += " | "
        if outer_ind == (-1, -1):
            out_trans[-1] += "(" + var_prefix+"_n_n'"
        else:
            out_trans[-1] += "(" + var_prefix+"_"+str(outer_ind[0])+"_"+str(outer_ind[1])+"'"
        for inner_ind in pos_indices:
            if ((inner_ind != (-1, -1) and W[inner_ind[0]][inner_ind[1]] == 1)
                or outer_ind == inner_ind):
                continue
            if inner_ind == (-1, -1):
                out_trans[-1] += " & (!" + var_prefix+"_n_n')"
            else:
                out_trans[-1] += " & (!" + var_prefix+"_"+str(inner_ind[0])+"_"+str(inner_ind[1])+"'" + ")"
        out_trans[-1] += ")"
        first_subformula = False

    return out_trans


def LTL_world_JTLV(W, var_prefix="obs",
                   center_loc=None, restrict_radius=1):
    """Convert world matrix W into an LTL formula describing transitions.

    Use JTLV syntax in the returned formula.
    
    The variables are named according to var_prefix_R_C, where
    var_prefix is given, R is the row, and column the cell
    (0-indexed).

    If center_loc is None (default), then object can occupy any open
    cell in the given world matrix.  If center_loc is a pair, then it
    indicates a location (row, column) in the world matrix and
    combined with the argument restrict_radius (same units as world
    matrix; 1 == 1 step by row or column), a restricted region of
    movement for the object is specified.  If part of the restricted
    region is outside the given world, then we must allow a movement
    to nowhere (i.e. ``out of the world W'') and movements back into
    W, where such transitions can occur along the boundary of W at
    which the restricted region leaves W.  Since the restricted region
    is compact and connected, transitions to/from nowhere can happen
    anywhere along this boundary (so, you might view ``nowhere'' as a
    special location connected to all this overflow positions).

    ``nowhere'' has the position (-1, -1).

    Return the formula string on success; None if failure.
    """
    always_nowhere_flag = False
    nowhere = [False,  # corresponds to row_low
               False,  # row_high
               False,  # col_low
               False]  # col_high
    if center_loc is not None:
        row_low = center_loc[0]-restrict_radius
        if row_low < 0:
            row_low = 0
            nowhere[0] = True
        row_high = center_loc[0]+restrict_radius
        if row_high > W.shape[0]-1:
            row_high = W.shape[0]-1
            nowhere[1] = True
        col_low = center_loc[1]-restrict_radius
        if col_low < 0:
            col_low = 0
            nowhere[2] = True
        col_high = center_loc[1]+restrict_radius
        if col_high > W.shape[1]-1:
            col_high = W.shape[1]-1
            nowhere[3] = True
        # Special case:
        if (row_low > W.shape[0]-1 or row_high < 0
            or col_low > W.shape[1]-1 or col_high < 0):
            always_nowhere_flag = True
    else:
        row_low = 0
        row_high = W.shape[0]-1
        col_low = 0
        col_high = W.shape[1]-1
    out_str = ""
    first_subformula = True
    # Safety, transitions
    for i in range(row_low, row_high+1):
        for j in range(col_low, col_high+1):
            if W[i][j] == 1:
                continue  # Cannot start from an occupied cell.
            if not first_subformula:
                out_str += " &\n\t"
            out_str += "[]("+var_prefix+"_"+str(i)+"_"+str(j)+" -> next("
            # Normal transitions:
            out_str += var_prefix+"_"+str(i)+"_"+str(j)
            if i > row_low and W[i-1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i-1)+"_"+str(j)
            if j > col_low and W[i][j-1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j-1)
            if i < row_high and W[i+1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i+1)+"_"+str(j)
            if j < col_high and W[i][j+1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j+1)
            # Transitions to ``nowhere'':
            if ((i == row_low and nowhere[0])
                or (i == row_high and nowhere[1])
                or (j == col_low and nowhere[2])
                or (j == col_high and nowhere[3])):
                out_str += " | " + var_prefix+"_n_n"
            out_str += "))"
            first_subformula = False
    if nowhere[0] or nowhere[1] or nowhere[2] or nowhere[3]:
        # Add transitions from ``nowhere''
        if not first_subformula:
            out_str += " &\n\t"
        out_str += "[]("+var_prefix+"_n_n"+" -> next("
        out_str += var_prefix+"_n_n"
        if not always_nowhere_flag:
            if nowhere[0]:
                for j in range(col_low, col_high+1):
                    out_str += " | " + var_prefix+"_"+str(row_low)+"_"+str(j)
            if nowhere[1]:
                for j in range(col_low, col_high+1):
                    out_str += " | " + var_prefix+"_"+str(row_high)+"_"+str(j)
            if nowhere[2]:
                for i in range(row_low, row_high+1):
                    out_str += " | " + var_prefix+"_"+str(i)+"_"+str(col_low)
            if nowhere[3]:
                for i in range(row_low, row_high+1):
                    out_str += " | " + var_prefix+"_"+str(i)+"_"+str(col_high)
        out_str += "))"
        first_subformula = False
    
    # Safety, static
    for i in range(row_low, row_high+1):
        for j in range(col_low, col_high+1):
            if W[i][j] == 1:
                if not first_subformula:
                    out_str += " &\n\t"
                out_str += "[](!(" + var_prefix+"_"+str(i)+"_"+str(j) + "))"
                first_subformula = False

    # Safety, mutex
    first_subformula = True
    out_str += "\n& []("
    pos_indices = [k for k in itertools.product(range(row_low, row_high+1), range(col_low, col_high+1))]
    if nowhere[0] or nowhere[1] or nowhere[2] or nowhere[3]:
        pos_indices.append((-1, -1))
    for outer_ind in pos_indices:
        if outer_ind != (-1, -1) and W[outer_ind[0]][outer_ind[1]] == 1:
            continue
        if not first_subformula:
            out_str += " | "
        if outer_ind == (-1, -1):
            out_str += "(" + var_prefix+"_n_n"
        else:
            out_str += "(" + var_prefix+"_"+str(outer_ind[0])+"_"+str(outer_ind[1])
        for inner_ind in pos_indices:
            if ((inner_ind != (-1, -1) and W[inner_ind[0]][inner_ind[1]] == 1)
                or outer_ind == inner_ind):
                continue
            if inner_ind == (-1, -1):
                out_str += " & (!" + var_prefix+"_n_n)"
            else:
                out_str += " & (!" + var_prefix+"_"+str(inner_ind[0])+"_"+str(inner_ind[1]) + ")"
        out_str += ")"
        first_subformula = False
    out_str += ")"
            
    return out_str


def subworld(W, subregion):
    """Return submatrix of W based on given subregion, and its offset.

    subregion should be a list of locations, preferably contiguous.
    The bounding rectangle of subregion is used to determine
    submatrix.
    """
    min_r = W.shape[0]
    max_r = -1
    min_c = W.shape[1]
    max_c = -1
    for k in subregion:
        if k[0] < min_r:
            min_r = k[0]
        if k[0] > max_r:
            max_r = k[0]
        if k[1] < min_c:
            min_c = k[1]
        if k[1] > max_c:
            max_c = k[1]
    return W[min_r:(max_r+1), min_c:(max_c+1)], (min_r, min_c)


def extract_autcoord(aut_node, var_prefix="Y"):
    """Pick out first true variable with name matching prefix_R_C format.

    aut_node should be an instance of class BTAutomatonNode.

    Return coordinate as given by extract_coord.  If more than one
    true variable has a matching prefix, return the corresponding list
    of coordinates.  If no matches found, return None.
    """
    coords = []
    for (k, v) in aut_node.state.items():
        if v != 1:  # We are only interested in True variables
            continue
        k_parts = k.split("_")
        if len(k_parts) < 3:  # Conforms to locative naming scheme?
            continue
        if "_".join(k_parts[:-2]) == var_prefix:
            coords.append(extract_coord(k)[1:])
    if len(coords) == 0:
        return None
    else:
        return coords

def extract_coord(var_name):
    """Assuming prefix_R_C format, return (prefix,row,column) tuple.

    prefix is of type string, row and column are integers.

    The ``nowhere'' coordinate has form prefix_n_n. To indicate this,
    (-1, -1) is returned as the row, column position.

    If error, return None or throw exception.
    """
    if not isinstance(var_name, str):
        raise TypeError("extract_coord: invalid argument type; must be string.")
    name_frags = var_name.split("_")
    if len(name_frags) < 3:
        return None
    try:
        if name_frags[-1] == "n" and name_frags[-2] == "n":
            # Special ``nowhere'' case
            return ("_".join(name_frags[:-2]), -1, -1)
        col = int(name_frags[-1])
        row = int(name_frags[-2])
    except ValueError:
        return None
    return ("_".join(name_frags[:-2]), row, col)
