#!/usr/bin/env python
"""
SCL; 2011 Aug, Sep, draft
"""

from automaton import BTAutomaton

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

    Summarizing, the return tuple looks like (W, goal_list, init_list, env_goal_list, env_init_list).

    NOTE: ENV GOAL LISTS ARE NOT YET IMPLEMENTED IN WORLD FILES!

    Assume lines are separated by '\n'; note that read_worldf ensures
    this before invoking read_world.
    """
    first_line_found = False
    line_counter = 0
    goal_list = []
    init_list = []
    env_goal_list = []
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
    return world, goal_list, init_list, env_goal_list, env_init_list

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

def LTL_world(W, var_prefix="obs"):
    """Convert world matrix W into an LTL formula describing transitions.
    
    The variables are named according to var_prefix_R_C, where
    var_prefix is given, R is the row, and column the cell
    (0-indexed).

    Return the formula string on success; None if failure.
    """
    out_str = ""
    first_subformula = True
    # Safety, transitions
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                continue  # Cannot start from an occupied cell.
            if not first_subformula:
                out_str += " &\n\t"
            out_str += "[]("+var_prefix+"_"+str(i)+"_"+str(j)+" -> next("
            out_str += var_prefix+"_"+str(i)+"_"+str(j)
            if i > 0 and W[i-1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i-1)+"_"+str(j)
            if j > 0 and W[i][j-1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j-1)
            if i < W.shape[0]-1 and W[i+1][j] == 0:
                out_str += " | " + var_prefix+"_"+str(i+1)+"_"+str(j)
            if j < W.shape[1]-1 and W[i][j+1] == 0:
                out_str += " | " + var_prefix+"_"+str(i)+"_"+str(j+1)
            out_str += "))"
            first_subformula = False
    
    # Safety, static
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                if not first_subformula:
                    out_str += " &\n\t"
                out_str += "[](!(" + var_prefix+"_"+str(i)+"_"+str(j) + "))"
                first_subformula = False

    # Safety, mutex
    first_subformula = True
    out_str += "\n& []("
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if W[i][j] == 1:
                continue
            if not first_subformula:
                out_str += " | "
            out_str += "(" + var_prefix+"_"+str(i)+"_"+str(j)
            for i_sub in range(W.shape[0]):
                for j_sub in range(W.shape[1]):
                    if (W[i_sub][j_sub] == 1) or (i == i_sub and j == j_sub):
                        continue
                    out_str += " & (!" + var_prefix+"_"+str(i_sub)+"_"+str(j_sub) + ")"
            out_str += ")"
            first_subformula = False
    out_str += ")"
            
    return out_str


def gen_navobs_soln(init_list, goal_list, W, num_obs,
                    env_init_list, env_goal_list,
                    var_prefix="Y", env_prefix="X",
                    disjunction_goals=False, env_disjunction_goals=False,
                    fname_prefix="tempsyn"):
    """Generate solution as in gen_dsoln but now with dynamic obstacles.

    This is a limited extension to the problem considered in
    gen_dsoln. Here we introduce a finite number (num_obs) of
    obstacles that navigate in the map W, thus requiring a reactive
    controller to safely avoid them while visiting the goal locations
    infinitely often.

    env_init_list specifies valid initial states for environment.  If
    omitted, env can start anywhere consistent with safety and
    progress properties (in assumption).

    env_goal_list is a list of locations that must be visited by the
    environment obstacles infinitely often.  This reduces the
    likelihood that a permissible env strategy is to corner the
    vehicle forever.  If env_goal_list has length greater than one,
    these locations can be combined in a conjunctive or disjunctive
    form, by setting env_disjunction_goals.  See comments in
    gen_dsoln.  Nonetheless, env_goal_list may be empty or None, in
    which case no such restriction is placed on the environment.

    Return instance of btrsynth.BTAutomaton on success;
    None if not realizable, or an error occurs.
    """
    # Argument error checking
    if (len(init_list) == 0) or (num_obs < 0):
        return None

    # Handle degenerate case of no obstacles (thus, deterministic problem).
    if num_obs < 1:
        return gen_dsoln(init_list=init_list, goal_list=goal_list, W=W,
                         var_prefix=var_prefix,
                         disjunction_goals=disjunction_goals,
                         fname_prefix=fname_prefix)

    ########################################
    # Environment prep
    env_str = [LTL_world(W, var_prefix="e."+env_prefix+"_"+str(k)) \
                   for k in range(num_obs)]

    env_init_str = ""
    if (env_init_list is not None) and len(env_init_list) > 0:
        for loc in env_init_list:
            for obs_ind in range(num_obs):
                if len(env_init_str) > 0:
                    env_init_str += " | "
                env_init_str += "(" + "e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    env_goal_str = ""
    if (env_goal_list is not None) and len(env_goal_list) > 0:
        if not env_disjunction_goals:
            for obs_ind in range(num_obs):
                for loc in env_goal_list:
                    if len(env_goal_str) > 0:
                        env_goal_str += " & "
                    env_goal_str += "[]<>(" + "e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
        else:
            for obs_ind in range(num_obs):
                if len(env_goal_str) > 0:
                    env_goal_str += " & "
                for loc in env_goal_list:
                    if len(env_goal_str) == 0:
                        env_goal_str += "[]<>( e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1])
                    env_goal_str += " | e."+env_prefix+"_"+str(obs_ind)+"_"+str(loc[0])+"_"+str(loc[1])
                env_goal_str += " )"
    
    ########################################
    # Sys prep
    safety_str = LTL_world(W, var_prefix="s."+var_prefix)

    init_str = ""
    for loc in init_list:
        if len(init_str) > 0:
            init_str += " | "
        init_str += "(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"

    goal_str = ""
    if not disjunction_goals:
        for loc in goal_list:
            if len(goal_str) > 0:
                goal_str += " & "
            goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    else:
        for loc in goal_list:
            if len(goal_str) == 0:
                goal_str += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_str += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_str += " )"

    ########################################
    # Interaction: avoid collisions
    coll_str = ""
    for i in range(W.shape[0]):
        for j in range(W.shape[1]):
            if len(coll_str) == 0:
                coll_str += "[]( "
            else:
                coll_str += " & "
            coll_str += "!(" + "s."+var_prefix+"_"+str(i)+"_"+str(j)
            for obs_ind in range(num_obs):
                coll_str += " & e."+env_prefix+"_"+str(obs_ind)+"_"+str(i)+"_"+str(j)
            coll_str += ")"
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
        for obs_ind in range(num_obs):
            for i in range(W.shape[0]):
                for j in range(W.shape[1]):
                    f.write("\t\t" + env_prefix+"_"+str(obs_ind)+"_"+str(i)+"_"+str(j) + " : boolean;\n")

    # Create SPC file
    with open(fname_prefix+".spc", "w") as f:
        f.write("LTLSPEC\n")
        if len(env_init_str) > 0:
            f.write("("+env_init_str+") & \n")
        if len(env_goal_str) > 0:
            f.write(env_goal_str + " & \n")
        first_obs_flag = True
        for obs_ind in range(num_obs):
            if first_obs_flag:
                f.write(env_str[obs_ind])
                first_obs_flag = False
            else:
                f.write(" &\n" + env_str[obs_ind])
        f.write("\n;\n\nLTLSPEC\n")
        f.write("("+init_str+") & \n" + goal_str + " & \n" + safety_str)
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
        next_node = aut.findNextAutState(next_node, env_state={},
                                         deterministic_env=False)
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


def gen_dsoln(init_list, goal_list, W, var_prefix="Y",
              disjunction_goals=False,
              fname_prefix="tempsyn"):
    """Generate deterministic solution, given initial and goal states.

    init_list is a list of pairs (row, col), signifying locations in
    the world matrix W from which the system can be initialized.
    Similarly for goal_list, but locations to visit infinitely often.

    If disjunction_goals is True, then the given goal positions appear
    in a single []<>... formula, combined in a disjunction (i.e. "or"
    statements).  The default (disjunction_goals = False) is for each
    goal position to occur infinitely often, hence one []<>... formula
    per goal.
    
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
    if not disjunction_goals:
        for loc in goal_list:
            if len(goal_str) > 0:
                goal_str += " & "
            goal_str += "[]<>(" + "s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1]) + ")"
    else:
        for loc in goal_list:
            if len(goal_str) == 0:
                goal_str += "[]<>( s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
            goal_str += " | s."+var_prefix+"_"+str(loc[0])+"_"+str(loc[1])
        goal_str += " )"

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
        next_node = aut.findNextAutState(next_node, env_state={},
                                         deterministic_env=True)
        if next_node == -1:
            raise ValueError("Reached deadend in Automaton; cannot find next node.")
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


def btsim_d(init, goal_list, aut, W_actual, num_steps=100, var_prefix="Y"):
    """Backtrack/patching algorithm, applied to deterministic problem.

    Note that this case is elementary and, being non-adversarial, may
    be better addressed by other methods (e.g., graph search or D*).
    Nonetheless it provides a decent base case for testing the idea.

    num_steps is the number of simulation steps to run; this count is
    across corrections.  That is, the basic execution sequence is
      1. sim to fault or num_steps reached;
      2. if fault, run backtrack/patch algorithm to improve automaton;
        goto step 1 (after updating aut)
      3. else (total step count num_steps reached), quit.
    
    (Note that this is not quite num_it as in the function dsim.)

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
            S_block = aut.findAllAutPartState({fail_loc_var : 1})
            S_Pre = []
            for node in aut.states:
                for bad_node in S_block:
                    if bad_node.id in node.transition:
                        for bad_node_redund in S_block:
                            # To ensure S_Pre has empty intersection with S_block
                            if node.id == bad_node_redund.id:
                                break
                        S_Pre.append(node)
                        break
            S_Post = {}
            # Dictionary with keys of IDs of nodes in S_Pre, and values in
            # S\S_block (AutomatonState instances), thus
            # expressing a reachability relationship (or ``branchout set''
            # in the terminology of the paper).
            for node in S_Pre:
                S_Post[node.id] = []
                for bad_node in S_block:
                    if bad_node.id in node.transition:
                        for post_node_id in bad_node.transition:
                            post_node = aut.getAutState(post_node_id)
                            if post_node == -1:
                                raise ValueError("inconsistent edge labelling in automaton.")
                            if post_node in S_block:
                                continue  # Do not include outgoing edges going back to S_block.
                            S_Post[node.id].append(post_node)
            for exit_node in S_Post.values():
                # This deviates from the algorithm! (a convenience hack; may fail)
                patch_goal_list.append(extract_autcoord(exit_node[0],
                                                        var_prefix=var_prefix)[0])
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            init_loc = (history[-1][0]-offset[0], history[-1][1]-offset[1])
            aut_patch = gen_dsoln([init_loc], patch_goal_list, W_patch)
            if aut_patch is not None:
                break  # Success! (i.e., patch problem is realizable)

        # Merge (in several steps)

        # Trim bad nodes from aut; note that we just delete ingoing
        # and outgoing edges from all nodes in S_block.  The result is
        # a set of ``floater'' nodes.
        for ind in range(len(S_Pre)):
            for bad_ind in range(len(S_block)):
                try:
                    S_Pre[ind].transition.remove(S_block[bad_ind].id)
                except ValueError:
                    pass
                S_block[bad_ind].transition = []

        # Import all nodes from M_patch into M. Keep track of the
        # assigned IDs.
        max_id = -1
        for node in aut.states:
            if node.id > max_id:
                max_id = node.id
        # Offset all IDs in M_patch by max_id+1
        for node in aut_patch.states:
            node.transition = [max_id+1+k for k in node.transition]
            node.id += max_id+1
            (i, j) = extract_autcoord(node, var_prefix=var_prefix)[0]
            node.state[var_prefix+"_"+str(i)+"_"+str(j)] = 0
            node.state[var_prefix+"_"+str(i+offset[0])+"_"+str(j+offset[1])] = 1
            aut.addAutState(node)

        # Add offset back in
        for ind in range(len(patch_goal_list)):
            patch_goal_list[ind] = (patch_goal_list[ind][0]+offset[0],
                                    patch_goal_list[ind][1]+offset[1])
        init_loc = (history[-1][0]+offset[0], history[-1][1]+offset[1])

        # Patch-in edges

        for ind in range(len(S_Pre)):
            for new_ind in range(len(aut_patch.states)):
                if extract_autcoord(S_Pre[ind], var_prefix=var_prefix)[0] == extract_autcoord(aut_patch.states[new_ind], var_prefix=var_prefix)[0]:
                    # Should be breadth-first search to find out-paths into S_Post[S_Pre[ind].id]
                    S_Pre[ind].transition = aut_patch.states[new_ind].transition[:]
                    break

        for patch_goal in patch_goal_list:
            for new_ind in range(len(aut_patch.states)):
                if extract_autcoord(aut_patch.states[new_ind], var_prefix=var_prefix)[0] != patch_goal:
                    continue
                for pre_node in S_Pre:
                    for post_ind in range(len(S_Post[pre_node.id])):
                        if extract_autcoord(S_Post[pre_node.id][post_ind], var_prefix=var_prefix)[0] == patch_goal:
                            for node_ind in range(len(aut.states)):
                                for trans_ind in range(len(aut.states[node_ind].transition)):
                                    if aut.states[node_ind].transition[trans_ind] == aut_patch.states[new_ind].id:
                                        aut.states[node_ind].transition[trans_ind] = S_Post[pre_node.id][post_ind].id
                            break


def btsim_navobs(init, goal_list, aut, W_actual, num_obs,
                 env_init_list, env_goal_list,
                 num_steps=100,
                 var_prefix="Y", env_prefix="X"):
    """Sister to btsim_d, but now for solutions from gen_navobs_soln.

    Cf. doc for navobs_sim and gen_navobs_soln.
    """
    step_count = 0
    while True:
        if step_count == num_steps:
            return aut, None

        # Loop invariants
        if num_steps-step_count < 0:
            raise ValueError("overstepped btsim_navobs loop.")
        
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
            S_block = aut.findAllAutPartState({fail_loc_var : 1})
            S_Pre = []
            for node in aut.states:
                for bad_node in S_block:
                    if bad_node.id in node.transition:
                        for bad_node_redund in S_block:
                            # To ensure S_Pre has empty intersection with S_block
                            if node.id == bad_node_redund.id:
                                break
                        S_Pre.append(node)
                        break
            S_Post = {}
            # Dictionary with keys of IDs of nodes in S_Pre, and values in
            # S\S_block (AutomatonState instances), thus
            # expressing a reachability relationship (or ``branchout set''
            # in the terminology of the paper).
            for node in S_Pre:
                S_Post[node.id] = []
                for bad_node in S_block:
                    if bad_node.id in node.transition:
                        for post_node_id in bad_node.transition:
                            post_node = aut.getAutState(post_node_id)
                            if post_node == -1:
                                raise ValueError("inconsistent edge labelling in automaton.")
                            if post_node in S_block:
                                continue  # Do not include outgoing edges going back to S_block.
                            S_Post[node.id].append(post_node)
            for exit_node in S_Post.values():
                # This deviates from the algorithm! (a convenience hack; may fail)
                patch_goal_list.append(extract_autcoord(exit_node[0],
                                                        var_prefix=var_prefix)[0])
            W_patch, offset = subworld(W_actual, nbhd_inclusion)
            # Shift coordinates to be w.r.t. W_patch
            for ind in range(len(patch_goal_list)):
                patch_goal_list[ind] = (patch_goal_list[ind][0]-offset[0],
                                        patch_goal_list[ind][1]-offset[1])
            init_loc = (history[-1][0]-offset[0], history[-1][1]-offset[1])
            aut_patch = gen_navobs_soln([init_loc], patch_goal_list, W_patch,
                                        env_init_list=None, env_goal_list=None,
                                        var_prefix=var_prefix, env_prefix=env_prefix)
            if aut_patch is not None:
                break  # Success! (i.e., patch problem is realizable)

        # Merge (in several steps)

        # Trim bad nodes from aut; note that we just delete ingoing
        # and outgoing edges from all nodes in S_block.  The result is
        # a set of ``floater'' nodes.
        for ind in range(len(S_Pre)):
            for bad_ind in range(len(S_block)):
                try:
                    S_Pre[ind].transition.remove(S_block[bad_ind].id)
                except ValueError:
                    pass
                S_block[bad_ind].transition = []

        # Import all nodes from M_patch into M. Keep track of the
        # assigned IDs.
        max_id = -1
        for node in aut.states:
            if node.id > max_id:
                max_id = node.id
        # Offset all IDs in M_patch by max_id+1
        for node in aut_patch.states:
            node.transition = [max_id+1+k for k in node.transition]
            node.id += max_id+1
            (i, j) = extract_autcoord(node, var_prefix=var_prefix)[0]
            node.state[var_prefix+"_"+str(i)+"_"+str(j)] = 0
            node.state[var_prefix+"_"+str(i+offset[0])+"_"+str(j+offset[1])] = 1
            aut.addAutState(node)

        # Add offset back in
        for ind in range(len(patch_goal_list)):
            patch_goal_list[ind] = (patch_goal_list[ind][0]+offset[0],
                                    patch_goal_list[ind][1]+offset[1])
        init_loc = (history[-1][0]+offset[0], history[-1][1]+offset[1])

        # Patch-in edges

        for ind in range(len(S_Pre)):
            for new_ind in range(len(aut_patch.states)):
                if extract_autcoord(S_Pre[ind], var_prefix=var_prefix)[0] == extract_autcoord(aut_patch.states[new_ind], var_prefix=var_prefix)[0]:
                    # Should be breadth-first search to find out-paths into S_Post[S_Pre[ind].id]
                    S_Pre[ind].transition = aut_patch.states[new_ind].transition[:]
                    break

        for patch_goal in patch_goal_list:
            for new_ind in range(len(aut_patch.states)):
                if extract_autcoord(aut_patch.states[new_ind], var_prefix=var_prefix)[0] != patch_goal:
                    continue
                for pre_node in S_Pre:
                    for post_ind in range(len(S_Post[pre_node.id])):
                        if extract_autcoord(S_Post[pre_node.id][post_ind], var_prefix=var_prefix)[0] == patch_goal:
                            for node_ind in range(len(aut.states)):
                                for trans_ind in range(len(aut.states[node_ind].transition)):
                                    if aut.states[node_ind].transition[trans_ind] == aut_patch.states[new_ind].id:
                                        aut.states[node_ind].transition[trans_ind] = S_Post[pre_node.id][post_ind].id
                            break


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

    aut_node should be an instance of class tulip.automaton.AutomatonState

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

    If error, return None or throw exception.
    """
    if not isinstance(var_name, str):
        raise TypeError("extract_coord: invalid argument type; must be string.")
    name_frags = var_name.split("_")
    if len(name_frags) < 3:
        return None
    try:
        col = int(name_frags[-1])
        row = int(name_frags[-2])
    except ValueError:
        return None
    return ("_".join(name_frags[:-2]), row, col)


########################################
# UNIT TESTS
########################################

def extract_coord_test():
    assert(extract_coord("test_3_0") == ("test", 3, 0))
    assert(extract_coord("obstacle_5_4_11") == ("obstacle_5", 4, 11))
    assert(extract_coord("test3_0") is None)
