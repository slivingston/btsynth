#!/usr/bin/env python
"""
SCL; 2011 Aug, Sep, draft
"""

from automaton import BTAutomaton

import itertools
import numpy as np
import tulip.grgameint


def errmsg(m):
    print "ERROR: "+m

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


def gen_navobs_soln(init_list, goal_list, W, num_obs,
                    env_init_list,
                    goals_disjunct=None,
                    restrict_radius=1,
                    var_prefix="Y", env_prefix="X",
                    fname_prefix="tempsyn"):
    """Generate solution as in gen_dsoln but now with dynamic obstacles.

    This is a limited extension to the problem considered in
    gen_dsoln. Here we introduce a finite number (num_obs) of
    obstacles that navigate in restricted regions of the map W, thus
    requiring a reactive controller to safely avoid them while
    visiting the goal locations infinitely often.

    env_init_list is the center position for each (env-controlled)
    obstacle. Thus it must be that len(env_init_list) = num_obs

    restrict_radius determines the domains of obstacles; cf. notes in
    function LTL_world.

    Return instance of btrsynth.BTAutomaton on success;
    None if not realizable, or an error occurs.
    """
    # Argument error checking
    if (len(init_list) == 0) or (num_obs < 0):
        return None

    # Handle degenerate case of no obstacles (thus, deterministic problem).
    if num_obs < 1:
        return gen_dsoln(init_list=init_list, goal_list=goal_list, W=W,
                         goals_disjunct=goals_disjunct,
                         var_prefix=var_prefix, fname_prefix=fname_prefix)

    ########################################
    # Environment prep
    obs_bounds = []
    env_str = []
    for k in range(num_obs):
        center_loc = env_init_list[k]
        row_low = center_loc[0]-restrict_radius
        nowhere = [False,  # corresponds to row_low
                   False,  # row_high
                   False,  # col_low
                   False]  # col_high
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
        obs_bounds.append((row_low, row_high, col_low, col_high, nowhere))
        env_str.append(LTL_world(W, var_prefix="e."+env_prefix+"_"+str(k),
                                 center_loc=center_loc,
                                 restrict_radius=restrict_radius))

    env_init_str = ""
    if (env_init_list is not None) and len(env_init_list) > 0:
        for loc in env_init_list:
            for obs_ind in range(num_obs):
                if len(env_init_str) > 0:
                    env_init_str += " | "
                if (loc[0] < row_low or loc[0] > row_high
                    or loc[1] < col_low or loc[1] > col_high):
                    env_init_str += "(" + "e."+env_prefix+"_"+str(obs_ind)+"_n_n)"
                else:
                    env_init_str += "(" + "e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    
    ########################################
    # Sys prep
    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    for loc in goal_list:
        if len(goal_str) > 0:
            goal_str += " & "
        goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    
    if (goals_disjunct is not None) and len(goals_disjunct) > 0:
        goal_dstr = ""
        for loc in goals_disjunct:
            if len(goal_dstr) == 0:
                if len(goal_str) == 0:
                    goal_dstr += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
                else:
                    goal_dstr += " & []<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_dstr += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_dstr += " )"
        goal_str += goal_dstr

    ########################################
    # Interaction: avoid collisions
    coll_str = ""
    for k in range(num_obs):
        for i in range(obs_bounds[k][0], obs_bounds[k][1]+1):
            for j in range(obs_bounds[k][2], obs_bounds[k][3]+1):
                if len(coll_str) == 0:
                    coll_str += "[]( "
                else:
                    coll_str += " & "
                coll_str += "!(" + "s."+var_prefix+"_"+str(i)+"_"+str(j)
                coll_str += " & e."+env_prefix+"_"+str(k)+"_"+str(i)+"_"+str(j)
                coll_str += ")"
    if len(coll_str) > 0:
        coll_str += ")"


    ########################################
    # Create SMV file
    with open(fname_prefix+".smv", "w") as f:
        # Some parts of this code are copied from tulip/rhtlp.py
        f.write("MODULE main \n")
        f.write("\tVAR\n")
        f.write("\t\te : env();\n")
        f.write("\t\ts : sys();\n\n")
        f.write("MODULE sys \n")
        f.write("\tVAR\n")
        for i in range(W.shape[0]):
            for j in range(W.shape[1]):
                f.write("\t\t" + var_prefix+"_"+str(i)+"_"+str(j) + " : boolean;\n")
        f.write("MODULE env \n")
        f.write("\tVAR\n")
        for k in range(num_obs):
            if (obs_bounds[k][4][0] or obs_bounds[k][4][1]
                or obs_bounds[k][4][2] or obs_bounds[k][4][3]):
                f.write("\t\t" + env_prefix+"_"+str(k)+"_n_n" + " : boolean;\n")
            for i in range(obs_bounds[k][0], obs_bounds[k][1]+1):
                for j in range(obs_bounds[k][2], obs_bounds[k][3]+1):
                    f.write("\t\t" + env_prefix+"_"+str(k)+"_"+str(i)+"_"+str(j) + " : boolean;\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n")
        if len(env_init_str) > 0:
            f.write("("+env_init_str+") & \n")
        first_obs_flag = True
        for k in range(num_obs):
            if first_obs_flag:
                f.write(env_str[k])
                first_obs_flag = False
            else:
                f.write(" &\n" + env_str[k])
        f.write("\n;\n\nLTLSPEC\n")
        f.write("("+init_str+") & \n" + goal_str + " & \n" + safety_str)
        if len(coll_str) > 0:
            f.write(" & \n" + coll_str)
        f.write("\n;")
    
    # Try JTLV synthesis
    realizable = tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                                           spc_file=fname_prefix+".spc",
                                           init_option=1, file_exist_option="r",
                                           heap_size="-Xmx2048m")

    if not realizable:
        return None
    else:
        return BTAutomaton(fname_prefix+".aut")


def navobs_sim(init, aut, W_actual, num_obs, var_prefix="Y", env_prefix="X",
               num_it=100):
    """Sister to dsim, but now for solutions from gen_navobs_soln.

    If the world is fully known a priori, then tulip.grsim suffices
    for generating a simulation.  navobs_sim simulates uncertainty and
    aborts when given controller breaks.

    Env locative variable prefixes are expected to take the form
    prefix_N_R_C, where R, C are (row, column) as usual, and N is the
    number of the obstacle.

    num_obs could be determined from analysing the automaton... future work.

    Same return values as in dsim, but also list of obstacle positions
    at time of failure.
    """
    # Handle initialization as a special case.
    if W_actual[init[0]][init[1]] == 1:
        return init, None

    # Based on given env_prefix, extract all environment variable names.
    env_vars = []
    for k in aut.states[0].state:
        if k.startswith(env_prefix):
            env_vars.append(k)
    # To avoid re-computing list comprehension at every step...
    fake_env_state = dict([(k,0) for k in env_vars])
    
    # Main simulation execution
    i, j = init
    loc_var = var_prefix+"_"+str(i)+"_"+str(j)
    next_node = aut.findAllAutPartState({loc_var : 1})
    if len(next_node) == 0:
        return init, None
    next_node = next_node[0]
    history = [(i,j),]  # Initialize trace
    it_counter = 0
    while it_counter < num_it:
        it_counter += 1
        this_node = next_node
        next_node_id = aut.execNextAutState(next_node.id,
                                            env_state=fake_env_state,
                                            randNext=True)
        next_node = aut.getAutState(next_node_id)
        next_loc = extract_autcoord(next_node, var_prefix=var_prefix)
        if next_loc is None:
            raise ValueError("Given automaton is incomplete; reached deadend.")
        if len(next_loc) > 1:
            raise ValueError("Given automaton invalid; more than one locative prop true, despite mutual exclusion.")
        next_loc = next_loc[0]
        if W_actual[next_loc[0]][next_loc[1]] == 1:
            obs_poses = []
            for obs_ind in range(num_obs):
                obs_poses.append(extract_autcoord(this_node,
                                                  var_prefix=env_prefix+"_"+str(obs_ind))[0])
            return history, next_loc, obs_poses
        history.append(next_loc)
    obs_poses = []
    for obs_ind in range(num_obs):
        obs_poses.append(extract_autcoord(next_node,
                                          var_prefix=env_prefix+"_"+str(obs_ind))[0])
    return history, True, obs_poses


def gen_dsoln(init_list, goal_list, W, goals_disjunct=None,
              var_prefix="Y", fname_prefix="tempsyn"):
    """Generate deterministic solution, given initial and goal states.

    init_list is a list of pairs (row, col), signifying locations in
    the world matrix W from which the system can be initialized.
    Similarly for goal_list, but locations to visit infinitely often.

    If goals_disjunct is not None, then it must be a list of (row,
    col) pairs specifying goals to be combined disjunctively in a
    single []<>... formula.  The default (None) does nothing.

    Return instance of btrsynth.BTAutomaton on success;
    None if not realizable, or an error occurs.
    """
    if len(init_list) == 0:
        return None

    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    for loc in goal_list:
        if len(goal_str) > 0:
            goal_str += " & "
        goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    if goals_disjunct is not None:
        goal_dstr = ""
        for loc in goals_disjunct:
            if len(goal_dstr) == 0:
                if len(goal_str) == 0:
                    goal_dstr += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
                else:
                    goal_dstr += " & []<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_dstr += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_dstr += " )"
        goal_str += goal_dstr

    # Create SMV file
    with open(fname_prefix+".smv", "w") as f:
        # Some parts of this code are copied from tulip/rhtlp.py
        f.write("MODULE main \n")
        f.write("\tVAR\n")
        f.write("\t\te : env();\n")
        f.write("\t\ts : sys();\n\n")
        f.write("MODULE sys \n")
        f.write("\tVAR\n")
        for i in range(W.shape[0]):
            for j in range(W.shape[1]):
                f.write("\t\t" + var_prefix+"_"+str(i)+"_"+str(j) + " : boolean;\n")
        f.write("MODULE env \n")
        f.write("\tVAR\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n;\n\nLTLSPEC\n")
        f.write("("+init_str+") & \n" + goal_str + " & \n" + safety_str)
        f.write("\n;")
    
    # Try JTLV synthesis
    realizable = tulip.grgameint.solveGame(smv_file=fname_prefix+".smv",
                                           spc_file=fname_prefix+".spc",
                                           init_option=1, file_exist_option="r")

    if not realizable:
        return None
    else:
        return BTAutomaton(fname_prefix+".aut")


def dsim(init, aut, W_actual, var_prefix="Y", num_it=100):
    """Simulate application of controller (automaton) on actual world.

    The thrust is that a solution was synthesized for a world W that
    is not correct, and so you would like to execute that controller
    on the true world W_actual and see when it breaks.

    var_prefix is the name prefix used for locative variables,
    e.g. what was using when calling gen_dsoln (their default settings
    should match).

    Return history trace, ending at location (i.e. position of
    vehicle) of failure, and intended next location (which implies an
    action, since we are treating deterministic problem setting).

    num_it is like a watchdog timer. Only num_it iterations will
    execute before quitting (regardless of failure occurring).

    If initial location is not even possible, then return initial
    location and None (rather than intended action).

    If quit because max number of iterations reached, history is
    returned and True (rather than an intended location).
    """
    # Handle initialization as a special case.
    if W_actual[init[0]][init[1]] == 1:
        return init, None

    # Special case of initial location possible and zero iterations run.
    if num_it == 0:
        return init, True
    
    # Main simulation execution
    i, j = init
    loc_var = var_prefix+"_"+str(i)+"_"+str(j)
    next_node = aut.findAllAutPartState({loc_var : 1})
    if len(next_node) == 0:
        return init, None
    next_node = next_node[0]
    history = [(i,j),]  # Initialize trace
    it_counter = 0
    while it_counter < num_it:
        it_counter += 1
        next_node_id = aut.execNextAutState(next_node.id, env_state={})
        next_node = aut.getAutState(next_node_id)
        # N.B., execNextAutState raises an exception on error.
        next_loc = extract_autcoord(next_node, var_prefix=var_prefix)
        if next_loc is None:
            raise ValueError("Given automaton is incomplete; reached deadend.")
        if len(next_loc) > 1:
            raise ValueError("Given automaton invalid; more than one locative prop true, despite mutual exclusion.")
        next_loc = next_loc[0]
        if W_actual[next_loc[0]][next_loc[1]] == 1:
            return history, next_loc
        history.append(next_loc)
    return history, True


def cond_anynot(memory):
    if len(memory) == 0:
        raise ValueError("Cannot apply transition-conditional on empty memory.")
    if 0 in memory.values():
        return True
    else:
        return False

def cond_all(memory):
    if len(memory) == 0:
        raise ValueError("Cannot apply transition-conditional on empty memory.")
    if 0 in memory.values():
        return False
    else:
        return True

def rule_clearall(aut, memory, prev_node_id, node_id, this_input):
    """Clear all memory values, regardless."""
    return dict([(k, 0) for k in memory.keys()])

def rule_setmatch(aut, memory, prev_node_id, node_id, this_input):
    """Set memory variables nonzero depending on edge-node labeling.

    For each memory variable name that is also an environment or
    system variable, if the present node (from which the rule was
    invoked) is labeled (or its incoming edge is labeled) with this
    variable being nonzero (i.e. True as a Boolean variable), then set
    that memory to 1.

    N.B., this rule acts one-way, i.e. it can only *set* memory
    variables, not clear them.
    """
    node = aut.getAutState(node_id)
    if node == -1:
        raise Exception("FATAL: rule called with invalid node ID.")
    for k in memory.keys():
        if node.state.has_key(k) and node.state[k] != 0:
            memory[k] = 1
    return memory


def btsim_d(init, goal_list, aut, W_actual, num_steps=100, var_prefix="Y"):
    """Backtrack/patching algorithm, applied to deterministic problem.

    This case is elementary and, being non-adversarial, may be better
    addressed by other methods (e.g., graph search or D*).
    Nonetheless it provides a decent base case for testing the idea.

    num_steps is the number of simulation steps to run; this count is
    across corrections.  That is, the basic execution sequence is
      1. sim to fault or num_steps reached;
      2. if fault, run backtrack/patch algorithm to improve automaton;
        goto step 1 (after updating aut)
      3. else (total step count num_steps reached; simulation without
        fault), quit.
    
    (num_steps is not the same as num_it in the function dsim.)

    Returns an updated (to reflect the corrected controller)
    instance of btrsynth.BTAutomaton and the known world map at
    time of completion.  Note that the ``known world'' may not match
    the given W_actual, because some parts of the world may never be
    visited (hence, uncertainty not corrected).

    If patching is impossible or seems as hard as the original
    (overall) problem, then return (None, None).
    """
    step_count = 0
    while True:
        if step_count == num_steps:
            return aut, None

        # Loop invariants
        if num_steps-step_count < 0:
            raise ValueError("overstepped bt dsim loop.")
        
        # Sim
        history, intent = dsim(init, aut, W_actual, var_prefix=var_prefix,
                               num_it=num_steps-step_count)
        if intent is True:
            return aut, None
        step_count += len(history)

        # Detect special case
        if intent in goal_list:
            return None, None

        # Patch (terminology follows that of the paper)
        gamma = 1  # radius
        delta = 1  # increment
        iteration_count = 0
        while True:
            iteration_count += 1
            radius = gamma + (iteration_count-1)*delta
            nbhd_inclusion = []  # Use Manhattan distance as metric
            for i in range(intent[0]-radius, intent[0]+radius+1):
                for j in range(intent[1]-radius, intent[1]+radius+1):
                    if i >= 0 and i < W_actual.shape[0] \
                            and j >= 0 and j < W_actual.shape[1]:
                        nbhd_inclusion.append((i, j))
            if len(nbhd_inclusion) == 0:
                raise ValueError("gamma radius is too small; neighborhood is empty.")
            patch_goal_list = []
            for v in nbhd_inclusion:
                if v in goal_list:
                    patch_goal_list.append(v)
            fail_loc_var = var_prefix+"_"+str(intent[0])+"_"+str(intent[1])
            
            # Set of nodes in M corresponding to abstract nbhd.
            Reg = aut.computeGridReg(nbhd=nbhd_inclusion, var_prefix=var_prefix)
            S0 = aut.getAutInit()
            Init = set([node.id for node in S0]) & set(Reg)
            Entry = aut.findEntry(Reg)
            Exit = aut.findExit(Reg)
            
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            
            patch_auts = []
            fail_flag = False
            for l in Init|set(Entry):
                init_loc = extract_autcoord(aut.getAutState(l), var_prefix=var_prefix)[0]
                init_loc = (init_loc[0]-offset[0], init_loc[1]-offset[1])
                local_goals_IDs = list(aut.computeReach(l, Reg) & set(Exit))
                local_goals = []
                for goal_ID in local_goals_IDs:
                    local_goals.append(extract_autcoord(aut.getAutState(goal_ID),
                                                        var_prefix=var_prefix)[0])
                    local_goals[-1] = (local_goals[-1][0]-offset[0],
                                       local_goals[-1][1]-offset[1])
                aut_patch = gen_dsoln(init_list=[init_loc], goal_list=patch_goal_list,
                                      W=W_patch, goals_disjunct=local_goals,
                                      var_prefix=var_prefix, fname_prefix="tempsyn-"+str(l))
                if aut_patch is not None:
                    patch_auts.append((aut_patch, l, local_goals_IDs))
                else:
                    fail_flag = True
                    break
            if not fail_flag:
                break

        # Merge (in several steps)

        # Set rule to clearing mem cells for nodes in the original M
        for node in aut.states:
            node.addNodeRule(rule_clearall)

        # Adjust map coordinates from local (patch-centric) to global,
        # and expand set of variables of the patch to include all
        # those of the (original) global problem.
        patch_id_maps = []
        full_state = aut.states[0].state.keys()  # Pick out full variable list
        for aut_ind in range(len(patch_auts)):
            Ml = patch_auts[aut_ind][0]
            for node in Ml.states:
                prev_keys = node.state.keys()
                (i, j) = extract_autcoord(node, var_prefix=var_prefix)[0]
                node.state = dict([(k, 0) for k in full_state])
                node.state[var_prefix+"_"+str(i+offset[0])+"_"+str(j+offset[1])] = 1
                node.addNodeRule(rule_setmatch)
            patch_id_maps.append(aut.importChildAut(Ml))

        # Undo offset of the part of sys goal list addressed in patch
        for k in range(len(patch_goal_list)):
            patch_goal_list[k] = (patch_goal_list[k][0]+offset[0],
                                  patch_goal_list[k][1]+offset[1])

        # Add memory for these goals
        aut.memInit([var_prefix+"_"+str(i)+"_"+str(j) for (i, j) in patch_goal_list])

        # Attach entry and exit points
        for aut_ind in range(len(patch_auts)):
            l = patch_auts[aut_ind][1]
            Ml = patch_auts[aut_ind][0]
            local_goals_IDs = patch_auts[aut_ind][2]
            entry_node = aut.getAutState(l)
            match_list = Ml.findAllAutState(entry_node.state)
            if len(match_list) == 0:
                raise Exception("FATAL")
            # Shortcut, given we are only addressing deterministic
            # (non-adversarial) problem in this example.
            entry_node.transition = [patch_id_maps[aut_ind][match_list[0].transition[0]]]

            match_flag = False
            for local_goal_ID in local_goals_IDs:
                goal_node = aut.getAutState(local_goal_ID)
                match_list = Ml.findAllAutState(goal_node.state)
                if len(match_list) > 0:
                    match_flag = True
                for match_node in match_list:
                    if len(aut.getMem()) > 0:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond = [cond_anynot for k in aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition]
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([cond_all for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition.extend(goal_node.transition)
                    else:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond = [None for k in aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition]
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([None for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition = goal_node.transition[:]
                    
                    
            if not match_flag:
                raise Exception("FATAL")
            
        # Delete blocked nodes and dependent edges
        kill_list = []
        for ind in range(len(aut.states)):
            if extract_autcoord(aut.states[ind], var_prefix=var_prefix)[0] == intent:
                kill_list.append(aut.states[ind].id)
        for kill_id in kill_list:
            aut.removeNode(kill_id)
        aut.packIDs()
        
        # Pick-off invalid initial nodes
        aut.removeFalseInits(S0)
        aut.packIDs()


def btsim_navobs(init, goal_list, aut, W_actual,
                 env_init_list, restrict_radius=1,
                 num_obs=None,
                 num_steps=100,
                 var_prefix="Y", env_prefix="X"):
    """Sister to btsim_d, but now for solutions from gen_navobs_soln.
    
    if num_obs is None, set it to len(env_init_list); this is a
    temporary hack till I clean up the code.

    Cf. doc for navobs_sim and gen_navobs_soln.
    """
    if num_obs is None:
        num_obs = len(env_init_list)
    step_count = 0
    while True:
        if step_count == num_steps:
            return aut, None

        # Loop invariants
        if num_steps-step_count < 0:
            raise ValueError("overstepped bt sim_navobs loop.")
        
        # Sim
        history, intent, obs_poses = navobs_sim(init, aut, W_actual,
                                                num_obs=num_obs,
                                                var_prefix=var_prefix,
                                                env_prefix=env_prefix,
                                                num_it=num_steps-step_count)
        if intent is True:
            return aut, None
        step_count += len(history)

        # Detect special case
        if intent in goal_list:
            return None, None

        # Patch (terminology follows that of the paper)
        gamma = 1  # radius
        delta = 1  # increment
        iteration_count = 0
        while True:
            iteration_count += 1
            radius = gamma + (iteration_count-1)*delta
            print "r_inc = "+str(radius)
            nbhd_inclusion = []  # Use Manhattan distance as metric
            for i in range(intent[0]-radius, intent[0]+radius+1):
                for j in range(intent[1]-radius, intent[1]+radius+1):
                    if i >= 0 and i < W_actual.shape[0] \
                            and j >= 0 and j < W_actual.shape[1]:
                        nbhd_inclusion.append((i, j))
            if len(nbhd_inclusion) == 0:
                raise ValueError("gamma radius is too small; neighborhood is empty.")
            patch_goal_list = []
            for v in nbhd_inclusion:
                if v in goal_list:
                    patch_goal_list.append(v)
            fail_loc_var = var_prefix+"_"+str(intent[0])+"_"+str(intent[1])
            
            # Set of nodes in M corresponding to abstract nbhd.
            Reg = aut.computeGridReg(nbhd=nbhd_inclusion, var_prefix=var_prefix)
            S0 = aut.getAutInit()
            Init = set([node.id for node in S0]) & set(Reg)
            Entry = aut.findEntry(Reg)
            Exit = aut.findExit(Reg)
            if len(Exit) == 0:
                raise Exception("FATAL: reduced to original problem, i.e. S = Reg.")
            
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            local_env_init = env_init_list[:]
            for ind in range(len(local_env_init)):
                local_env_init[ind] = (local_env_init[ind][0]-offset[0],
                                       local_env_init[ind][1]-offset[1])
            
            patch_auts = []
            fail_flag = False
            for l in Init|set(Entry):
                init_loc = extract_autcoord(aut.getAutState(l), var_prefix=var_prefix)[0]
                init_loc = (init_loc[0]-offset[0], init_loc[1]-offset[1])
                local_goals_IDs = list(aut.computeReach(l, Reg) & set(Exit))
                local_goals = []
                for goal_ID in local_goals_IDs:
                    local_goals.append(extract_autcoord(aut.getAutState(goal_ID),
                                                        var_prefix=var_prefix)[0])
                    local_goals[-1] = (local_goals[-1][0]-offset[0],
                                       local_goals[-1][1]-offset[1])
                local_goals = list(set(local_goals))  # Remove redundancy
                aut_patch = gen_navobs_soln(init_list=[init_loc], goal_list=patch_goal_list,
                                            W=W_patch, num_obs=num_obs,
                                            env_init_list=local_env_init,
                                            restrict_radius=restrict_radius,
                                            goals_disjunct=local_goals,
                                            var_prefix=var_prefix, env_prefix=env_prefix,
                                            fname_prefix="tempsyn-"+str(l))
                if aut_patch is not None:
                    patch_auts.append((aut_patch, l, local_goals_IDs))
                else:
                    fail_flag = True
                    break
            if not fail_flag:
                break

        # Merge (in several steps)

        # Trim dead nodes in patch automata
        # for aut_ind in range(len(patch_auts)):
        #     print "BEFORE: "+str(patch_auts[aut_ind][0].size())
        #     patch_auts[aut_ind][0].trimDeadStates()
        #     print "AFTER: "+str(patch_auts[aut_ind][0].size())

        # Set rule to clearing mem cells for nodes in the original M
        for node in aut.states:
            node.addNodeRule(rule_clearall)

        # Adjust map coordinates from local (patch-centric) to global,
        # and expand set of variables of the patch to include all
        # those of the (original) global problem.
        patch_id_maps = []
        env_vars_list = []
        env_nowhere_vars = []
        for obs in range(num_obs):
            # Pick out full list of env variables, for each obstacle
            env_vars_list.append(prefix_filt(aut.states[0].state,
                                             prefix=env_prefix+"_"+str(obs)))
            env_vars_list[-1] = env_vars_list[-1].keys()
            env_nowhere_vars.append(env_prefix+"_"+str(obs)+"_n_n")
        # Pick out full list of sys variables
        sys_vars = prefix_filt(aut.states[0].state, prefix=var_prefix)  
        for aut_ind in range(len(patch_auts)):
            Ml = patch_auts[aut_ind][0]
            for obs in range(num_obs):
                Ml.fleshOutGridState(env_vars_list[obs],
                                     special_var=env_nowhere_vars[obs])
            for node in Ml.states:
                # This approach is not general, in that we assume
                # *all* system variables pertain to position in the abstract space (i.e. Z/P).
                (i, j) = extract_autcoord(node, var_prefix=var_prefix)[0]
                for sys_var in sys_vars:
                    node.state[sys_var] = 0
                node.state[var_prefix+"_"+str(i+offset[0])+"_"+str(j+offset[1])] = 1
                node.addNodeRule(rule_setmatch)
                node.cond = [cond_anynot for k in range(len(node.transition))]
            patch_id_maps.append(aut.importChildAut(Ml))

        # Undo offset of the part of sys goal list addressed in patch
        for k in range(len(patch_goal_list)):
            patch_goal_list[k] = (patch_goal_list[k][0]+offset[0],
                                  patch_goal_list[k][1]+offset[1])

        # Add memory for these goals
        aut.memInit([var_prefix+"_"+str(i)+"_"+str(j) for (i, j) in patch_goal_list])

        # Attach entry and exit points
        for aut_ind in range(len(patch_auts)):
            #import pdb; pdb.set_trace() #DEBUG
            l = patch_auts[aut_ind][1]
            Ml = patch_auts[aut_ind][0]
            local_goals_IDs = patch_auts[aut_ind][2]
            entry_node = aut.getAutState(l)
            sys_state = prefix_filt(entry_node.state, prefix=var_prefix)
            match_list = Ml.findAllAutPartState(sys_state)
            if len(match_list) == 0:
                raise Exception("FATAL")
            for match in match_list:
                for k in range(len(entry_node.transition)):
                    next_node = aut.getAutState(entry_node.transition[k])
                    env_state = prefix_filt(next_node.state, prefix=env_prefix)
                    # Find environment (outward edge) labels that are consistent.
                    # Note that multiple matches leads to multiple overwrites, silently!
                    for j in range(len(match.transition)):
                        match_next = Ml.getAutState(match.transition[j])
                        if set(env_state.items()).issubset(set(match_next.state.items())):
                            entry_node.transition[k] = patch_id_maps[aut_ind][match.transition[j]]
                            entry_node.cond[k] = None

            match_flag = False
            for local_goal_ID in local_goals_IDs:
                goal_node = aut.getAutState(local_goal_ID)
                sys_state = prefix_filt(goal_node.state, prefix=var_prefix)
                match_list = Ml.findAllAutPartState(sys_state)
                if len(match_list) > 0:
                    match_flag = True
                for match_node in match_list:
                    if len(aut.getMem()) > 0:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([cond_all for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition.extend(goal_node.transition)
                    else:
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond = [None for k in aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition]
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).cond.extend([None for k in goal_node.cond])
                        aut.getAutState(patch_id_maps[aut_ind][match_node.id]).transition = goal_node.transition[:]
                    
                    
            if not match_flag:
                raise Exception("FATAL")
            
        # Delete blocked nodes and dependent edges
        kill_list = []
        for ind in range(len(aut.states)):
            if extract_autcoord(aut.states[ind], var_prefix=var_prefix)[0] == intent:
                kill_list.append(aut.states[ind].id)
        for kill_id in kill_list:
            aut.removeNode(kill_id)
        aut.packIDs()
        
        # Pick-off invalid initial nodes, and other clean-up
        aut.removeFalseInits(S0)
        aut.packIDs()
        aut.cleanDuplicateTrans()


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

def to_formula(aut_node):
    """Take the given automaton node and return formula equivalent to state.

    NOTE: ...this function, or one like it, may be general enough to
    place in the tulip.automaton module directly.

    ASSUMES ALL VARIABLES APPEARING IN GIVEN NODE ARE BOOLEAN.

    Returns string type.
    """
    out_str = ""
    for (k, v) in aut_node.state.items():
        if len(out_str) > 0:
            out_str += " & "
        if v > 0:
            out_str += "("
        else:
            out_str += "(!"
        out_str += k+")"
    return out_str

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

def prefix_filt(d, prefix):
    """return all items in dictionary d with key with given prefix."""
    match_list = []
    for k in d.keys():
        if isinstance(k, str):
            if k.startswith(prefix):
                match_list.append(k)
    return dict([(k, d[k]) for k in match_list])


########################################
# UNIT TESTS
########################################

def extract_coord_test():
    assert(extract_coord("test_3_0") == ("test", 3, 0))
    assert(extract_coord("obstacle_5_4_11") == ("obstacle_5", 4, 11))
    assert(extract_coord("test3_0") is None)

def prefix_filt_test():
    assert(prefix_filt({"Y_0_0": 0, "Y_0_1": 1, "X_0_1_0": 1}, "Y") == {"Y_0_0": 0, "Y_0_1": 1})
